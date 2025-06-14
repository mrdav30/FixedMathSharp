using System.Runtime.CompilerServices;

namespace FixedMathSharp
{
    public static partial class Vector3dExtensions
    {
        #region Vector3d Operations

        /// <summary>
        /// Clamps each component of the vector to the range [-1, 1] in place and returns the modified vector.
        /// </summary>
        /// <param name="v">The vector to clamp.</param>
        /// <returns>The clamped vector with each component between -1 and 1.</returns>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static Vector3d ClampOneInPlace(this Vector3d v)
        {
            v.x = v.x.ClampOne();
            v.y = v.y.ClampOne();
            v.z = v.z.ClampOne();
            return v;
        }

        public static Vector3d ClampMagnitude(this Vector3d value, Fixed64 maxMagnitude)
        {
            return Vector3d.ClampMagnitude(value, maxMagnitude);
        }

        /// <summary>
        /// Checks if the distance between two vectors is less than or equal to a specified factor.
        /// </summary>
        /// <param name="me">The current vector.</param>
        /// <param name="other">The vector to compare distance to.</param>
        /// <param name="factor">The maximum allowable distance.</param>
        /// <returns>True if the distance between the vectors is less than or equal to the factor, false otherwise.</returns>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static bool CheckDistance(this Vector3d me, Vector3d other, Fixed64 factor)
        {
            var dis = Vector3d.Distance(me, other);
            return dis <= factor;
        }

        /// <inheritdoc cref="Vector3d.Rotate(Vector3d, Vector3d, FixedQuaternion)" />
        public static Vector3d Rotate(this Vector3d source, Vector3d position, FixedQuaternion rotation)
        {
            return Vector3d.Rotate(source, position, rotation);
        }

        /// <inheritdoc cref="Vector3d.InverseRotate(Vector3d, Vector3d, FixedQuaternion)" />
        public static Vector3d InverseRotate(this Vector3d source, Vector3d position, FixedQuaternion rotation)
        {
            return Vector3d.InverseRotate(source, position, rotation);
        }

        #endregion

        #region Conversion

        /// <inheritdoc cref="Vector3d.ToDegrees(Vector3d)" />
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static Vector3d ToDegrees(this Vector3d radians)
        {
            return Vector3d.ToDegrees(radians);
        }

        /// <inheritdoc cref="Vector3d.ToRadians(Vector3d)" />
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static Vector3d ToRadians(this Vector3d degrees)
        {
            return Vector3d.ToRadians(degrees);
        }

        /// <inheritdoc cref="Vector3d.Abs(Vector3d)" />
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static Vector3d Abs(this Vector3d value)
        {
            return Vector3d.Abs(value);
        }

        /// <inheritdoc cref="Vector3d.Sign(Vector3d)" />
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static Vector3d Sign(Vector3d value)
        {
            return Vector3d.Sign(value);
        }

        #endregion

        #region Equality

        /// <summary>
        /// Compares two vectors for approximate equality, allowing a fixed absolute difference.
        /// </summary>
        /// <param name="me">The current vector.</param>
        /// <param name="other">The vector to compare against.</param>
        /// <param name="allowedDifference">The allowed absolute difference between each component.</param>
        /// <returns>True if the components are within the allowed difference, false otherwise.</returns>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static bool FuzzyEqualAbsolute(this Vector3d me, Vector3d other, Fixed64 allowedDifference)
        {
            return (me.x - other.x).Abs() <= allowedDifference &&
                   (me.y - other.y).Abs() <= allowedDifference &&
                   (me.z - other.z).Abs() <= allowedDifference;
        }

        /// <summary>
        /// Compares two vectors for approximate equality, allowing a fractional difference (percentage).
        /// Handles zero components by only using the allowed percentage difference.
        /// </summary>
        /// <param name="me">The current vector.</param>
        /// <param name="other">The vector to compare against.</param>
        /// <param name="percentage">The allowed fractional difference (percentage) for each component.</param>
        /// <returns>True if the components are within the allowed percentage difference, false otherwise.</returns>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static bool FuzzyEqual(this Vector3d me, Vector3d other, Fixed64? percentage = null)
        {
            Fixed64 p = percentage ?? Fixed64.Epsilon;
            return me.x.FuzzyComponentEqual(other.x, p) &&
                    me.y.FuzzyComponentEqual(other.y, p) &&
                    me.z.FuzzyComponentEqual(other.z, p);
        }

        #endregion
    }
}