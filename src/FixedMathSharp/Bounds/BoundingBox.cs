using System;
using System.Runtime.CompilerServices;

namespace FixedMathSharp
{
    /// <summary>
    /// Represents a 3D bounding box with operations like intersection checking, resizing, and vertex generation.
    /// </summary>
    public struct BoundingBox : IBound, IEquatable<BoundingBox>
    {
        /// <summary>
        /// The center of the bounding box.
        /// </summary>
        public Vector3d Center { get; private set; }
        /// <summary>
        /// The total size of the box (Width, Height, Depth). This is always twice the scope.
        /// </summary>
        public Vector3d Size => Scope * 2;

        /// <summary>
        /// The range (half-size) of the bounding box in all directions. Always half of the total size.
        /// </summary>
        public Vector3d Scope { get; private set; }

        private bool _dirtyVertices;
        private Vector3d[] _cachedVertices;
        /// <summary>
        /// Vertices of the bounding box, lazily generated and cached for efficiency.
        /// </summary>
        public Vector3d[] Vertices => _dirtyVertices ? GenerateVertices() : _cachedVertices;

        /// <summary>
        /// The minimum corner of the bounding box.
        /// </summary>
        public Vector3d Min { get; private set; }

        /// <summary>
        /// The maximum corner of the bounding box.
        /// </summary>
        public Vector3d Max { get; private set; }

        /// <summary>
        /// Initializes a new instance of the BoundingBox struct with the specified center and size.
        /// </summary>
        public BoundingBox(Vector3d center, Vector3d size)
        {
            Center = center;
            Scope = size * FixedMath.Half;  // Half of the size represents the scope.
            Min = Center - Scope;
            Max = Center + Scope;
            _dirtyVertices = false;
            _cachedVertices = new Vector3d[8];
        }

        /// <summary>
        /// Orients the bounding box with the given center and size.
        /// </summary>
        public void Orient(Vector3d center, Vector3d size)
        {
            SetBoundingBox(center, size * FixedMath.Half);
        }

        /// <summary>
        /// Resizes the bounding box to the specified size, keeping the same center.
        /// </summary>
        public void Resize(Vector3d size)
        {
            SetBoundingBox(Center, size * FixedMath.Half);
        }

        /// <summary>
        /// Sets the bounds of the bounding box by specifying its minimum and maximum points.
        /// </summary>
        public void SetMinMax(Vector3d min, Vector3d max)
        {
            Vector3d newScope = (max - min) * FixedMath.Half;
            SetBoundingBox(min + newScope, newScope);
        }

        /// <summary>
        /// Configures the bounding box with the specified center and scope (half-size).
        /// </summary>
        public void SetBoundingBox(Vector3d center, Vector3d scope)
        {
            Center = center;
            Scope = scope;
            Min = Center - Scope;
            Max = Center + Scope;
            _dirtyVertices = true;  // Reset cached vertices.
        }

        /// <summary>
        /// Checks if the specified point intersects with the bounding box.
        /// </summary>
        public bool Intersects(Vector3d point)
        {
            return point.x >= Min.x && point.x <= Max.x &&
                   point.y >= Min.y && point.y <= Max.y &&
                   point.z >= Min.z && point.z <= Max.z;
        }

        public bool Contains(Vector3d point)
        {
            return Intersects(point);
        }

        public bool Intersects(IBound other)
        {
            if (other is BoundingBox otherBox)
            {
                return Intersects(otherBox);
            }
            else if (other is BoundingSphere otherSphere)
            {
                return Intersects(otherSphere);
            }
            return false;
        }

        /// <summary>
        /// Checks if another bounding box intersects with this bounding box.
        /// </summary>
        public bool Intersects(BoundingBox other)
        {
            return !(Max.x < other.Min.x || Min.x > other.Max.x ||
                     Max.y < other.Min.y || Min.y > other.Max.y ||
                     Max.z < other.Min.z || Min.z > other.Max.z);
        }

        /// <summary>
        /// Checks if a bounding sphere intersects with this bounding box.
        /// </summary>
        public bool Intersects(BoundingSphere sphere)
        {
            Vector3d closestPoint = ProjectPoint(sphere.Center);
            return Vector3d.SqrDistance(sphere.Center, closestPoint) <= sphere.Radius * sphere.Radius;
        }

        public Fixed64 DistanceToSurface(Vector3d point)
        {
            return Vector3d.Distance(Center, ProjectPoint(point));
        }

        /// <summary>
        /// Creates a new bounding box that is the union of two bounding boxes.
        /// </summary>
        public static BoundingBox Union(BoundingBox a, BoundingBox b)
        {
            Vector3d min = Vector3d.Min(a.Min, b.Min);
            Vector3d max = Vector3d.Max(a.Max, b.Max);

            Vector3d center = (max + min) * FixedMath.Half;
            Vector3d size = max - min;

            return new BoundingBox(center, size);
        }

        /// <summary>
        /// Projects a point onto the bounding box. If the point is outside the box, it returns the closest point on the surface.
        /// </summary>
        public Vector3d ProjectPoint(Vector3d point)
        {
            return Vector3d.Max(Vector3d.Min(point, Max), Min);
        }

        /// <summary>
        /// Finds the closest point on the surface of the bounding box towards a specified object position.
        /// </summary>
        public Vector3d GetPointOnSurfaceTowardsObject(Vector3d objectPosition)
        {
            return ClosestPointOnSurface(ProjectPoint(objectPosition));
        }

        /// <summary>
        /// Finds the closest points between two bounding boxes.
        /// </summary>
        public Vector3d FindClosestPointsBetweenBoxes(BoundingBox other)
        {
            Vector3d closestPoint = Vector3d.Zero;
            Fixed64 minDistance = FixedMath.MaxValue;
            for (int i = 0; i < other.Vertices.Length; i++)
            {
                Vector3d point = ClosestPointOnSurface(other.Vertices[i]);
                Fixed64 distance = Vector3d.Distance(point, other.Vertices[i]);
                if (distance < minDistance)
                {
                    closestPoint = point;
                    minDistance = distance;
                }
            }

            return closestPoint;
        }

        /// <summary>
        /// Finds the closest point on the surface of the bounding box to the specified point.
        /// </summary>
        public Vector3d ClosestPointOnSurface(Vector3d point)
        {
            if (IsInside(point))
            {
                // Calculate distances to each face and return the closest face.
                Fixed64 distToMinX = point.x - Min.x;
                Fixed64 distToMaxX = Max.x - point.x;
                Fixed64 distToMinY = point.y - Min.y;
                Fixed64 distToMaxY = Max.y - point.y;
                Fixed64 distToMinZ = point.z - Min.z;
                Fixed64 distToMaxZ = Max.z - point.z;

                Fixed64 minDistToFace = FixedMath.Min(distToMinX, FixedMath.Min(distToMaxX, FixedMath.Min(distToMinY, FixedMath.Min(distToMaxY, FixedMath.Min(distToMinZ, distToMaxZ)))));

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
        /// Determines if a point is inside the bounding box (including boundaries).
        /// </summary>
        public bool IsInside(Vector3d point)
        {
            return point.x >= Min.x && point.x <= Max.x &&
                   point.y >= Min.y && point.y <= Max.y &&
                   point.z >= Min.z && point.z <= Max.z;
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static bool operator ==(BoundingBox left, BoundingBox right)
        {
            return left.Equals(right);
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static bool operator !=(BoundingBox left, BoundingBox right)
        {
            return !left.Equals(right);
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public override bool Equals(object obj)
        {
            return obj is BoundingBox other && Equals(other);
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public bool Equals(BoundingBox other)
        {
            return Center.Equals(other.Center) && Scope.Equals(other.Scope);
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public override int GetHashCode()
        {
            unchecked
            {
                int hash = 17;
                hash = hash * 23 + Center.GetHashCode();
                hash = hash * 23 + Scope.GetHashCode();
                return hash;
            }
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
        private Vector3d[] GenerateVertices()
        {
            _cachedVertices[0] = Center + new Vector3d(-Scope.x, -Scope.y, -Scope.z);
            _cachedVertices[1] = Center + new Vector3d(Scope.x, -Scope.y, -Scope.z);
            _cachedVertices[2] = Center + new Vector3d(-Scope.x, Scope.y, -Scope.z);
            _cachedVertices[3] = Center + new Vector3d(Scope.x, Scope.y, -Scope.z);
            _cachedVertices[4] = Center + new Vector3d(-Scope.x, -Scope.y, Scope.z);
            _cachedVertices[5] = Center + new Vector3d(Scope.x, -Scope.y, Scope.z);
            _cachedVertices[6] = Center + new Vector3d(-Scope.x, Scope.y, Scope.z);
            _cachedVertices[7] = Center + new Vector3d(Scope.x, Scope.y, Scope.z);

            _dirtyVertices = false;

            return _cachedVertices;
        }
    }
}