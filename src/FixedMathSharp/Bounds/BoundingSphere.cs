using MemoryPack;
using System;
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
        /// Checks if this sphere intersects with another IBound.
        /// </summary>
        /// <param name="other">The other IBound to check for intersection.</param>
        /// <returns>True if the IBounds intersect, otherwise false.</returns>
        public bool Intersects(IBound other)
        {
            switch (other)
            {
                case BoundingBox or BoundingArea:
                    // Find the closest point on the BoundingArea to the sphere's center
                    // Check if the closest point is within the sphere's radius
                    return Vector3d.SqrDistance(Center, other.ProjectPointWithinBounds(Center)) <= SqrRadius;
                case BoundingSphere otherSphere:
                    {
                        Fixed64 distanceSquared = Vector3d.SqrDistance(Center, otherSphere.Center);
                        Fixed64 combinedRadius = Radius + otherSphere.Radius;
                        return distanceSquared <= combinedRadius * combinedRadius;
                    }

                default: return false; // Default case for unknown or unsupported types
            }
        }

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

        #endregion
    }
}