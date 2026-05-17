using MemoryPack;
using System;
using System.Collections.Generic;
using System.Runtime.CompilerServices;
using System.Text.Json.Serialization;

namespace FixedMathSharp
{
    /// <summary>
    /// Represents a spherical bounding volume with fixed-point precision, optimized for fast, rotationally invariant spatial checks in 3D space.
    /// </summary>
    /// <remarks>
    /// The BoundingSphere provides a simple yet effective way to represent the spatial extent of objects, especially when rotational invariance is required. 
    /// Compared to BoundingBox, it offers faster intersection checks but is less precise in tightly fitting non-spherical objects.
    /// 
    /// Use Cases:
    /// - Ideal for broad-phase collision detection, proximity checks, and culling in physics engines and rendering pipelines.
    /// - Useful when fast, rotationally invariant checks are needed, such as detecting overlaps or distances between moving objects.
    /// - Suitable for encapsulating objects with roughly spherical shapes or objects that rotate frequently, where the bounding box may need constant updates.
    /// </remarks>
    [Serializable]
    [MemoryPackable]
    public partial struct BoundingSphere : IBound, IEquatable<BoundingSphere>
    {
        #region Fields

        /// <summary>
        /// The center point of the sphere.
        /// </summary>
        [JsonInclude]
        [MemoryPackOrder(0)]
        public Vector3d Center;

        /// <summary>
        /// The radius of the sphere.
        /// </summary>
        [JsonInclude]
        [MemoryPackOrder(1)]
        public Fixed64 Radius;

        #endregion

        #region Constructors

        /// <summary>
        /// Initializes a new instance of the BoundingSphere struct with the specified center and radius.
        /// </summary>
        [JsonConstructor]
        public BoundingSphere(Vector3d center, Fixed64 radius)
        {
            Center = center;
            Radius = radius;
        }

        #endregion

        #region Properties

        /// <summary>
        /// Gets the coordinates of the minimum corner of the bounding box that contains the sphere.
        /// </summary>
        /// <remarks>
        /// The minimum corner is calculated by subtracting the radius from each component of the sphere's center. 
        /// This property is useful for spatial queries and bounding box calculations.
        /// </remarks>
        [JsonIgnore]
        [MemoryPackIgnore]
        public Vector3d Min
        {
            [MethodImpl(MethodImplOptions.AggressiveInlining)]
            get => Center - new Vector3d(Radius, Radius, Radius);
        }

        /// <summary>
        /// Gets the coordinates of the maximum corner of the bounding box that contains the sphere.
        /// </summary>
        /// <remarks>
        /// The maximum corner is calculated as the center of the sphere plus the radius in each dimension. 
        /// This property is useful for spatial queries and bounding volume calculations.
        /// </remarks>
        [JsonIgnore]
        [MemoryPackIgnore]
        public Vector3d Max
        {
            [MethodImpl(MethodImplOptions.AggressiveInlining)]
            get => Center + new Vector3d(Radius, Radius, Radius);
        }

        /// <summary>
        /// The squared radius of the sphere.
        /// </summary>
        [JsonIgnore]
        [MemoryPackIgnore]
        public Fixed64 SqrRadius
        {
            [MethodImpl(MethodImplOptions.AggressiveInlining)]
            get => Radius * Radius;
        }

        #endregion

        #region Methods (Static)

        /// <summary>
        /// Creates a bounding sphere that contains the specified axis-aligned bounding box.
        /// </summary>
        public static BoundingSphere CreateFromBoundingBox(BoundingBox box)
        {
            Vector3d center = (box.Min + box.Max) * Fixed64.Half;
            Fixed64 radius = Vector3d.Distance(center, box.Max);

            return new BoundingSphere(center, radius);
        }

        /// <summary>
        /// Creates a bounding sphere that contains the specified frustum.
        /// </summary>
        public static BoundingSphere CreateFromFrustum(BoundingFrustum frustum)
        {
            if (frustum == null)
                throw new ArgumentNullException(nameof(frustum));

            return CreateFromPoints(frustum.GetCorners());
        }

        /// <summary>
        /// Creates a bounding sphere that contains the specified points.
        /// </summary>
        public static BoundingSphere CreateFromPoints(IEnumerable<Vector3d> points)
        {
            if (points == null)
                throw new ArgumentNullException(nameof(points));

            if (points is IReadOnlyList<Vector3d> pointList)
                return CreateFromPointList(pointList);

            var materialized = new List<Vector3d>();
            foreach (Vector3d point in points)
                materialized.Add(point);

            return CreateFromPointList(materialized);
        }

        /// <summary>
        /// Creates the smallest sphere that contains the two specified spheres.
        /// </summary>
        public static BoundingSphere CreateMerged(BoundingSphere original, BoundingSphere additional)
        {
            Vector3d centerOffset = additional.Center - original.Center;
            Fixed64 distance = centerOffset.Magnitude;

            if (distance + additional.Radius <= original.Radius)
                return original;

            if (distance + original.Radius <= additional.Radius)
                return additional;

            if (distance == Fixed64.Zero)
                return original.Radius >= additional.Radius ? original : additional;

            Fixed64 radius = (distance + original.Radius + additional.Radius) * Fixed64.Half;
            Vector3d center = original.Center + centerOffset * ((radius - original.Radius) / distance);

            return new BoundingSphere(center, radius);
        }

        private static BoundingSphere CreateFromPointList(IReadOnlyList<Vector3d> points)
        {
            if (points.Count == 0)
                throw new ArgumentException("At least one point is required.", nameof(points));

            Vector3d minX = points[0];
            Vector3d maxX = points[0];
            Vector3d minY = points[0];
            Vector3d maxY = points[0];
            Vector3d minZ = points[0];
            Vector3d maxZ = points[0];

            for (int i = 1; i < points.Count; i++)
            {
                Vector3d point = points[i];

                if (point.x < minX.x) minX = point;
                if (point.x > maxX.x) maxX = point;
                if (point.y < minY.y) minY = point;
                if (point.y > maxY.y) maxY = point;
                if (point.z < minZ.z) minZ = point;
                if (point.z > maxZ.z) maxZ = point;
            }

            Fixed64 sqDistX = Vector3d.SqrDistance(maxX, minX);
            Fixed64 sqDistY = Vector3d.SqrDistance(maxY, minY);
            Fixed64 sqDistZ = Vector3d.SqrDistance(maxZ, minZ);

            Vector3d min = minX;
            Vector3d max = maxX;
            Fixed64 largestDistance = sqDistX;

            if (sqDistY > largestDistance)
            {
                min = minY;
                max = maxY;
                largestDistance = sqDistY;
            }

            if (sqDistZ > largestDistance)
            {
                min = minZ;
                max = maxZ;
            }

            Vector3d center = (min + max) * Fixed64.Half;
            Fixed64 radius = Vector3d.Distance(max, center);
            Fixed64 sqRadius = radius * radius;

            for (int i = 0; i < points.Count; i++)
            {
                Vector3d diff = points[i] - center;
                Fixed64 sqDistance = diff.SqrMagnitude;
                if (sqDistance <= sqRadius)
                    continue;

                Fixed64 distance = FixedMath.Sqrt(sqDistance);
                if (distance == Fixed64.Zero)
                    continue;

                Fixed64 newRadius = (radius + distance) * Fixed64.Half;
                center += diff * ((distance - radius) / (Fixed64.Two * distance));
                radius = newRadius;
                sqRadius = EnsureRadiusContainsPoint(points[i], center, ref radius);
            }

            return new BoundingSphere(center, radius);
        }

        private static Fixed64 EnsureRadiusContainsPoint(Vector3d point, Vector3d center, ref Fixed64 radius)
        {
            Fixed64 sqRadius = radius * radius;
            Fixed64 sqDistance = Vector3d.SqrDistance(point, center);

            if (sqDistance <= sqRadius)
                return sqRadius;

            radius = FixedMath.Sqrt(sqDistance);
            sqRadius = radius * radius;

            if (sqRadius < sqDistance)
            {
                radius += Fixed64.MinIncrement;
                sqRadius = radius * radius;
            }

            return sqRadius;
        }

        #endregion

        #region Methods (Instance)

        /// <summary>
        /// Checks if a point is inside the sphere.
        /// </summary>
        /// <param name="point">The point to check.</param>
        /// <returns>True if the point is inside the sphere, otherwise false.</returns>
        public bool Contains(Vector3d point)
        {
            return Vector3d.SqrDistance(Center, point) <= SqrRadius;
        }

        /// <summary>
        /// Tests a bounding box against this sphere.
        /// </summary>
        public ContainmentType Contains(BoundingBox box)
        {
            return ContainsBoxLike(box.Min, box.Max);
        }

        /// <summary>
        /// Tests a bounding area against this sphere.
        /// </summary>
        public ContainmentType Contains(BoundingArea area)
        {
            return ContainsBoxLike(area.Min, area.Max);
        }

        /// <summary>
        /// Tests another sphere against this sphere.
        /// </summary>
        public ContainmentType Contains(BoundingSphere sphere)
        {
            Fixed64 sqDistance = Vector3d.SqrDistance(Center, sphere.Center);
            Fixed64 combinedRadius = Radius + sphere.Radius;

            if (sqDistance > combinedRadius * combinedRadius)
                return ContainmentType.Disjoint;

            Fixed64 radiusDifference = Radius - sphere.Radius;
            if (radiusDifference >= Fixed64.Zero && sqDistance <= radiusDifference * radiusDifference)
                return ContainmentType.Contains;

            return ContainmentType.Intersects;
        }

        /// <summary>
        /// Tests a frustum against this sphere.
        /// </summary>
        public ContainmentType Contains(BoundingFrustum frustum)
        {
            if (frustum == null)
                throw new ArgumentNullException(nameof(frustum));

            Vector3d[] corners = frustum.GetCorners();
            bool containsAllCorners = true;

            for (int i = 0; i < corners.Length; i++)
            {
                if (!Contains(corners[i]))
                {
                    containsAllCorners = false;
                    break;
                }
            }

            if (containsAllCorners)
                return ContainmentType.Contains;

            return frustum.Intersects(this)
                ? ContainmentType.Intersects
                : ContainmentType.Disjoint;
        }

        /// <summary>
        /// Checks if this sphere intersects with another IBound.
        /// </summary>
        /// <param name="other">The other IBound to check for intersection.</param>
        /// <returns>True if the IBounds intersect, otherwise false.</returns>
        public bool Intersects(IBound other)
        {
            switch (other)
            {
                case BoundingBox box: return Intersects(box);
                case BoundingArea area: return Intersects(area);
                case BoundingSphere sphere: return Intersects(sphere);

                default: return false; // Default case for unknown or unsupported types
            }
        }

        /// <summary>
        /// Checks whether a bounding box intersects this sphere.
        /// </summary>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public bool Intersects(BoundingBox box) => Contains(box) != ContainmentType.Disjoint;

        /// <summary>
        /// Checks whether a bounding area intersects this sphere.
        /// </summary>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public bool Intersects(BoundingArea area) => Contains(area) != ContainmentType.Disjoint;

        /// <summary>
        /// Checks whether another sphere intersects this sphere.
        /// </summary>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public bool Intersects(BoundingSphere sphere) => Contains(sphere) != ContainmentType.Disjoint;

        /// <summary>
        /// Checks whether a frustum intersects this sphere.
        /// </summary>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public bool Intersects(BoundingFrustum frustum)
        {
            if (frustum == null)
                throw new ArgumentNullException(nameof(frustum));

            return frustum.Intersects(this);
        }

        /// <summary>
        /// Classifies this sphere relative to a plane.
        /// </summary>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public FixedPlaneIntersectionType Intersects(FixedPlane plane) => plane.Intersects(this);

        /// <summary>
        /// Finds the first forward ray intersection with this sphere.
        /// </summary>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public Fixed64? Intersects(FixedRay ray) => ray.Intersects(this);

        /// <summary>
        /// Projects a point onto the bounding sphere. If the point is outside the sphere, it returns the closest point on the surface.
        /// </summary>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public Vector3d ProjectPoint(Vector3d point)
        {
            var direction = point - Center;
            if (direction.IsZero) return Center; // If the point is the center, return the center itself

            return Center + direction.Normalize() * Radius;
        }

        /// <summary>
        /// Calculates the distance from a point to the surface of the sphere.
        /// </summary>
        /// <param name="point">The point to calculate the distance from.</param>
        /// <returns>The distance from the point to the surface of the sphere.</returns>
        public Fixed64 DistanceToSurface(Vector3d point)
        {
            return Vector3d.Distance(Center, point) - Radius;
        }

        /// <summary>
        /// Creates a sphere that contains this sphere transformed by the specified matrix.
        /// </summary>
        public BoundingSphere Transform(Fixed4x4 matrix)
        {
            Vector3d center = Fixed4x4.TransformPoint(matrix, Center);
            Fixed64 scale = GetMaxBasisScale(matrix);

            return new BoundingSphere(center, Radius * scale);
        }

        /// <summary>
        /// Deconstructs this sphere into its center and radius.
        /// </summary>
        public void Deconstruct(out Vector3d center, out Fixed64 radius)
        {
            center = Center;
            radius = Radius;
        }

        private ContainmentType ContainsBoxLike(Vector3d min, Vector3d max)
        {
            bool containsAllCorners =
                Contains(new Vector3d(min.x, min.y, min.z)) &&
                Contains(new Vector3d(max.x, min.y, min.z)) &&
                Contains(new Vector3d(min.x, max.y, min.z)) &&
                Contains(new Vector3d(max.x, max.y, min.z)) &&
                Contains(new Vector3d(min.x, min.y, max.z)) &&
                Contains(new Vector3d(max.x, min.y, max.z)) &&
                Contains(new Vector3d(min.x, max.y, max.z)) &&
                Contains(new Vector3d(max.x, max.y, max.z));

            if (containsAllCorners)
                return ContainmentType.Contains;

            Vector3d closest = new(
                FixedMath.Clamp(Center.x, min.x, max.x),
                FixedMath.Clamp(Center.y, min.y, max.y),
                FixedMath.Clamp(Center.z, min.z, max.z));

            return Vector3d.SqrDistance(Center, closest) <= SqrRadius
                ? ContainmentType.Intersects
                : ContainmentType.Disjoint;
        }

        private static Fixed64 GetMaxBasisScale(Fixed4x4 matrix)
        {
            Fixed64 row0 = matrix.m00 * matrix.m00 + matrix.m01 * matrix.m01 + matrix.m02 * matrix.m02;
            Fixed64 row1 = matrix.m10 * matrix.m10 + matrix.m11 * matrix.m11 + matrix.m12 * matrix.m12;
            Fixed64 row2 = matrix.m20 * matrix.m20 + matrix.m21 * matrix.m21 + matrix.m22 * matrix.m22;

            return FixedMath.Sqrt(FixedMath.Max(row0, FixedMath.Max(row1, row2)));
        }

        #endregion

        #region Operators

        /// <summary>
        /// Determines whether two BoundingSphere instances are equal.
        /// </summary>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static bool operator ==(BoundingSphere left, BoundingSphere right) => left.Equals(right);

        /// <summary>
        /// Determines whether two BoundingSphere instances are not equal.
        /// </summary>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static bool operator !=(BoundingSphere left, BoundingSphere right) => !left.Equals(right);

        #endregion

        #region Equality and HashCode Overrides

        /// <inheritdoc/>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public override bool Equals(object? obj) => obj is BoundingSphere other && Equals(other);

        /// <inheritdoc/>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public bool Equals(BoundingSphere other) => Center.Equals(other.Center) && Radius.Equals(other.Radius);

        /// <inheritdoc/>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public override int GetHashCode()
        {
            unchecked
            {
                int hash = 17;
                hash = hash * 23 + Center.GetHashCode();
                hash = hash * 23 + Radius.GetHashCode();
                return hash;
            }
        }

        /// <summary>
        /// Returns a string that represents the current BoundingSphere.
        /// </summary>
        public override string ToString()
        {
            return $"{{Center:{Center} Radius:{Radius}}}";
        }

        #endregion
    }
}
