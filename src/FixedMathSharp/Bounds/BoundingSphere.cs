using System;
using System.Runtime.CompilerServices;

namespace FixedMathSharp
{
    /// <summary>
    /// Represents a bounding sphere in 3D space with a center and radius.
    /// Useful for collision detection or proximity checking.
    /// </summary>
    public struct BoundingSphere : IBound, IEquatable<BoundingSphere>
    {
        /// <summary>
        /// The center point of the sphere.
        /// </summary>
        public Vector3d Center { get; private set; }

        /// <summary>
        /// The radius of the sphere.
        /// </summary>
        public Fixed64 Radius { get; private set; }

        /// <summary>
        /// Initializes a new instance of the BoundingSphere struct with the specified center and radius.
        /// </summary>
        public BoundingSphere(Vector3d center, Fixed64 radius)
        {
            Center = center;
            Radius = radius;
        }

        /// <summary>
        /// Repositions the sphere to a new center position.
        /// </summary>
        public void SetPosition(Vector3d position)
        {
            Center = position;
        }

        /// <summary>
        /// Resizes the sphere to a new radius.
        /// </summary>
        public void SetRadius(Fixed64 radius)
        {
            Radius = radius;
        }

        /// <summary>
        /// Checks if a point is inside the sphere.
        /// </summary>
        /// <param name="point">The point to check.</param>
        /// <returns>True if the point is inside the sphere, otherwise false.</returns>
        public bool Contains(Vector3d point)
        {
            return Vector3d.SqrDistance(Center, point) <= Radius * Radius;
        }

        public bool Intersects(IBound other)
        {
            if (other is BoundingSphere otherSphere)
            {
                return Intersects(otherSphere);
            }

            if (other is BoundingBox otherBox)
            {
                return Intersects(otherBox);
            }

            return false;
        }

        /// <summary>
        /// Checks if this sphere intersects with another sphere.
        /// </summary>
        /// <param name="other">The other sphere to check for intersection.</param>
        /// <returns>True if the spheres intersect, otherwise false.</returns>
        public bool Intersects(BoundingSphere other)
        {
            Fixed64 distanceSquared = Vector3d.SqrDistance(Center, other.Center);
            Fixed64 combinedRadius = Radius + other.Radius;
            return distanceSquared <= combinedRadius * combinedRadius;
        }

        /// <summary>
        /// Checks if this sphere intersects with a box.
        /// </summary>
        public bool Intersects(BoundingBox box)
        {
            // Project the center of the sphere onto the box to find the closest point.
            Vector3d closestPoint = box.ProjectPoint(Center);
            // If the distance from the closest point to the sphere's center is less than or equal to the radius, they intersect.
            return Vector3d.SqrDistance(Center, closestPoint) <= Radius * Radius;
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

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static bool operator ==(BoundingSphere left, BoundingSphere right)
        {
            return left.Equals(right);
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static bool operator !=(BoundingSphere left, BoundingSphere right)
        {
            return !left.Equals(right);
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public override bool Equals(object obj)
        {
            return obj is BoundingSphere other && Equals(other);
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public bool Equals(BoundingSphere other)
        {
            return Center.Equals(other.Center) && Radius.Equals(other.Radius);
        }

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
    }
}