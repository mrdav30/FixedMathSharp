using MemoryPack;
using System;
using System.Runtime.CompilerServices;

#if NET8_0_OR_GREATER
using System.Text.Json.Serialization;
#endif

namespace FixedMathSharp
{
    /// <summary>
    /// Represents an axis-aligned bounding box with fixed-point precision, capable of encapsulating 3D objects for more complex spatial checks.
    /// </summary>
    /// <remarks>
    /// The BoundingBox provides a more detailed representation of an object's spatial extent compared to BoundingArea. 
    /// It is useful in 3D scenarios requiring more precise volume fitting and supports a wider range of operations, including more complex intersection checks.
    /// 
    /// Use Cases:
    /// - Encapsulating objects in 3D space for collision detection, ray intersection, or visibility testing.
    /// - Used in physics engines, rendering pipelines, and 3D simulations to represent object boundaries.
    /// - Supports more precise intersection tests than BoundingArea, making it ideal for detailed spatial queries.
    /// </remarks>
    [Serializable]
    [MemoryPackable]
    public partial struct BoundingBox : IBound, IEquatable<BoundingBox>
    {
        #region Fields

        /// <summary>
        /// The minimum corner of the bounding box.
        /// </summary>
        [MemoryPackOrder(0)]
        public Vector3d Min { get; private set; }

        /// <summary>
        /// The maximum corner of the bounding box.
        /// </summary>
        [MemoryPackOrder(1)]
        public Vector3d Max { get; private set; }

        /// <summary>
        /// Serialization/compatibility version of this <see cref="BoundingBox"/> instance.
        /// </summary>
        [MemoryPackOrder(2)]
        public byte Version { get; private set; }

        [MemoryPackIgnore]
        private bool _isDirty;

        /// <summary>
        /// Vertices of the bounding box.
        /// </summary>
        [MemoryPackIgnore]
        private Vector3d[] _vertices;

        #endregion

        #region Constructors

        /// <summary>
        /// Initializes a new instance of the BoundingBox struct with the specified center and size.
        /// </summary>

        public BoundingBox(Vector3d center, Vector3d size)
        {
            Vector3d half = Vector3d.Abs(size) * Fixed64.Half;

            Min = center - half;
            Max = center + half;

            _vertices = new Vector3d[8];
            _isDirty = true;
            Version = 1;
        }

#if NET8_0_OR_GREATER
        [JsonConstructor]
#endif
        [MemoryPackConstructor]
        public BoundingBox(Vector3d min, Vector3d max, byte version)
        {
            this.Min = min;
            this.Max = max;
            _vertices = new Vector3d[8];
            _isDirty = true;
            this.Version = version;
        }

        #endregion

        #region Properties and Methods (Instance)

        /// <summary>
        /// The center of the bounding box.
        /// </summary>
        [MemoryPackIgnore]
        public Vector3d Center
        {
            [MethodImpl(MethodImplOptions.AggressiveInlining)]
            get => (Min + Max) * Fixed64.Half;

            [MethodImpl(MethodImplOptions.AggressiveInlining)]
            set
            {
                Vector3d half = (Max - Min) * Fixed64.Half;
                Min = value - half;
                Max = value + half;
                _isDirty = true;
            }
        }

        /// <summary>
        /// The total size of the box (Width, Height, Depth). This is always twice the scope.
        /// </summary>
        [MemoryPackIgnore]
        public Vector3d Proportions
        {
            [MethodImpl(MethodImplOptions.AggressiveInlining)]
            get => Max - Min;

            [MethodImpl(MethodImplOptions.AggressiveInlining)]
            set
            {
                Vector3d half = value * Fixed64.Half;
                Vector3d center = (Min + Max) * Fixed64.Half;
                Min = center - half;
                Max = center + half;
                _isDirty = true;
            }
        }

        /// <summary>
        /// The range (half-size) of the bounding box in all directions. Always half of the total size.
        /// </summary>
        [MemoryPackIgnore]
        public Vector3d Scope
        {
            [MethodImpl(MethodImplOptions.AggressiveInlining)]
            get => (Max - Min) * Fixed64.Half;
        }

        /// <inheritdoc cref="_vertices" />
        [MemoryPackIgnore]
        public Vector3d[] Vertices
        {
            [MethodImpl(MethodImplOptions.AggressiveInlining)]
            get => GetOrGenerateVertices();
        }

        #endregion

        #region Mutators

        /// <summary>
        /// Orients the bounding box with the given center and size.
        /// </summary>
        public void Orient(Vector3d center, Vector3d? size)
        {
            Vector3d half = size.HasValue
                ? size.Value * Fixed64.Half
                : (Max - Min) * Fixed64.Half;

            Min = center - half;
            Max = center + half;
            _isDirty = true;
        }

        /// <summary>
        /// Resizes the bounding box to the specified size, keeping the same center.
        /// </summary>
        public void Resize(Vector3d size)
        {
            Vector3d half = size * Fixed64.Half;
            Vector3d center = (Min + Max) * Fixed64.Half;

            Min = center - half;
            Max = center + half;
            _isDirty = true;
        }

        /// <summary>
        /// Sets the bounds of the bounding box by specifying its minimum and maximum points.
        /// </summary>
        public void SetMinMax(Vector3d min, Vector3d max)
        {
            Min = min;
            Max = max;
            _isDirty = true;
        }

        /// <summary>
        /// Configures the bounding box with the specified center and scope (half-size).
        /// </summary>
        public void SetBoundingBox(Vector3d center, Vector3d scope)
        {
            Min = center - scope;
            Max = center + scope;
            _isDirty = true;
        }

        /// <summary>
        /// Determines if a point is inside the bounding box (including boundaries).
        /// </summary>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public bool Contains(Vector3d point)
        {
            return point.x >= Min.x && point.x <= Max.x
                && point.y >= Min.y && point.y <= Max.y
                && point.z >= Min.z && point.z <= Max.z;
        }

        /// <summary>
        /// Checks if another IBound intersects with this bounding box.
        /// Ensures overlap, not just touching boundaries.
        /// </summary>
        /// <remarks>
        /// It checks for overlap on all axes. If there is no overlap on any axis, they do not intersect.
        /// </remarks>
        public bool Intersects(IBound other)
        {
            switch (other)
            {
                case BoundingBox or BoundingArea:
                    {
                        if (Contains(other.Min) && Contains(other.Max))
                            return true;  // Full containment

                        // General intersection logic (allowing for overlap)
                        return !(Max.x <= other.Min.x || Min.x >= other.Max.x ||
                                 Max.y <= other.Min.y || Min.y >= other.Max.y ||
                                 Max.z <= other.Min.z || Min.z >= other.Max.z);
                    }
                case BoundingSphere sphere:
                    // project the sphere’s center onto the 3D volume and checks the distance to the surface.
                    // If the distance from the closest point to the center is less than or equal to the sphere’s radius, they intersect.
                    return Vector3d.SqrDistance(sphere.Center, this.ProjectPointWithinBounds(sphere.Center)) <= sphere.SqrRadius;

                default:
                    return false; // Default case for unknown or unsupported types
            }
        }

        /// <summary>
        /// Projects a point onto the bounding box. If the point is outside the box, it returns the closest point on the surface.
        /// </summary>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public Vector3d ProjectPoint(Vector3d point)
            => this.ProjectPointWithinBounds(point);

        /// <summary>
        /// Calculates the shortest distance from a given point to the surface of the bounding box.
        /// If the point lies inside the box, the distance is zero.
        /// </summary>
        /// <param name="point">The point from which to calculate the distance.</param>
        /// <returns>
        /// The shortest distance from the point to the surface of the bounding box.
        /// If the point is inside the box, the method returns zero.
        /// </returns>
        /// <remarks>
        /// The method finds the closest point on the box's surface by clamping the given point 
        /// to the box's bounds and returns the Euclidean distance between them. 
        /// This ensures accurate distance calculations, even near corners or edges.
        /// </remarks>
        public Fixed64 DistanceToSurface(Vector3d point)
        {
            // Clamp the point to the nearest point on the box's surface
            Vector3d clampedPoint = new(
                FixedMath.Clamp(point.x, Min.x, Max.x),
                FixedMath.Clamp(point.y, Min.y, Max.y),
                FixedMath.Clamp(point.z, Min.z, Max.z)
            );

            // If the point is inside the box, return 0
            if (Contains(point))
                return Fixed64.Zero;

            // Otherwise, return the Euclidean distance to the clamped point
            return Vector3d.Distance(point, clampedPoint);
        }

        /// <summary>
        /// Finds the closest point on the surface of the bounding box towards a specified object position.
        /// </summary>
        public Vector3d GetPointOnSurfaceTowardsObject(Vector3d objectPosition)
            => ClosestPointOnSurface(ProjectPoint(objectPosition));

        /// <summary>
        /// Finds the closest point on the surface of the bounding box to the specified point.
        /// </summary>
        public Vector3d ClosestPointOnSurface(Vector3d point)
        {
            if (Contains(point))
            {
                // Calculate distances to each face and return the closest face.
                Fixed64 distToMinX = point.x - Min.x;
                Fixed64 distToMaxX = Max.x - point.x;
                Fixed64 distToMinY = point.y - Min.y;
                Fixed64 distToMaxY = Max.y - point.y;
                Fixed64 distToMinZ = point.z - Min.z;
                Fixed64 distToMaxZ = Max.z - point.z;

                Fixed64 minDistToFace = FixedMath.Min(distToMinX,
                    FixedMath.Min(distToMaxX,
                    FixedMath.Min(distToMinY,
                    FixedMath.Min(distToMaxY,
                    FixedMath.Min(distToMinZ, distToMaxZ)))));

                // Adjust the closest point based on the face.
                if (minDistToFace == distToMinX) point.x = Min.x;
                else if (minDistToFace == distToMaxX) point.x = Max.x;

                if (minDistToFace == distToMinY) point.y = Min.y;
                else if (minDistToFace == distToMaxY) point.y = Max.y;

                if (minDistToFace == distToMinZ) point.z = Min.z;
                else if (minDistToFace == distToMaxZ) point.z = Max.z;

                return point;
            }

            // If the point is outside the box, clamp to the nearest surface.
            return new Vector3d(
                FixedMath.Clamp(point.x, Min.x, Max.x),
                FixedMath.Clamp(point.y, Min.y, Max.y),
                FixedMath.Clamp(point.z, Min.z, Max.z)
            );
        }

        /// <summary>
        /// Generates the vertices of the bounding box based on its center and scope.
        /// </summary>
        /// <remarks>
        ///  Vertices[0]  near Bot left 
        ///  Vertices[1]  near Bot right 
        ///  Vertices[2]  near Top left
        ///  Vertices[3]  near Top right
        ///  Vertices[4]  far bot left
        ///  Vertices[5]  far bot right
        ///  Vertices[6]  far top left
        ///  Vertices[7]  far top right
        ///  ----
        ///  near quad
        ///  0 - 1 Bot left near to bot right near
        ///  2 - 3 Top left near to top right near
        ///  0 - 2 Bot left near to top left near
        ///  1 - 3 Bot right near to top right near
        ///  far quad
        ///  4 - 5 Bot left far to bot right far
        ///  6 - 7 Top left far to top right far
        ///  4 - 6 Bot left far to top left far
        ///  5 - 7 Bot right far to top right far
        ///  lines connecting near and far quads
        ///  0 - 4 Bot left near to bot left far
        ///  1 - 5 Bot right near to bot right far
        ///  2 - 6 Top left near to top left far
        ///  3 - 7 Top right near to top right far  
        /// </remarks>
        private Vector3d[] GetOrGenerateVertices()
        {
            _vertices ??= new Vector3d[8];

            if (_isDirty)
            {
                Vector3d min = Min;
                Vector3d max = Max;

                _vertices[0] = new(min.x, min.y, min.z);
                _vertices[1] = new(max.x, min.y, min.z);
                _vertices[2] = new(min.x, max.y, min.z);
                _vertices[3] = new(max.x, max.y, min.z);
                _vertices[4] = new(min.x, min.y, max.z);
                _vertices[5] = new(max.x, min.y, max.z);
                _vertices[6] = new(min.x, max.y, max.z);
                _vertices[7] = new(max.x, max.y, max.z);

                _isDirty = false;
            }

            return _vertices;
        }

        #endregion

        #region Static Ops

        /// <summary>
        /// Creates a new bounding box that is the union of two bounding boxes.
        /// </summary>
        public static BoundingBox Union(BoundingBox a, BoundingBox b)
        {
            return new BoundingBox
            {
                Min = Vector3d.Min(a.Min, b.Min),
                Max = Vector3d.Max(a.Max, b.Max),
                Version = (byte)Math.Max(a.Version, b.Version),
                _isDirty = true
            };
        }

        /// <summary>
        /// Finds the closest points between two bounding boxes.
        /// </summary>
        public static Vector3d FindClosestPointsBetweenBoxes(BoundingBox a, BoundingBox b)
        {
            Vector3d closestPoint = Vector3d.Zero;
            Fixed64 minDistance = Fixed64.MAX_VALUE;

            for (int i = 0; i < b.Vertices.Length; i++)
            {
                Vector3d point = a.ClosestPointOnSurface(b.Vertices[i]);
                Fixed64 distance = Vector3d.Distance(point, b.Vertices[i]);
                if (distance < minDistance)
                {
                    closestPoint = point;
                    minDistance = distance;
                }
            }

            return closestPoint;
        }

        #endregion

        #region Equality

        public static bool operator ==(BoundingBox left, BoundingBox right) => left.Equals(right);
        public static bool operator !=(BoundingBox left, BoundingBox right) => !left.Equals(right);

        #endregion

        #region Equality and HashCode Overrides

        public override bool Equals(object? obj) => obj is BoundingBox other && Equals(other);

        public bool Equals(BoundingBox other)
            => Min.Equals(other.Min) && Max.Equals(other.Max);

        public override int GetHashCode()
        {
            return HashCode.Combine(Min, Max);
        }

        #endregion
    }
}