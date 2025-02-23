using System.Runtime.CompilerServices;

namespace FixedMathSharp
{
    public static partial class Vector2dExtensions
    {
        #region Vector2d Operations

        /// <summary>
        /// Clamps each component of the vector to the range [-1, 1] in place and returns the modified vector.
        /// </summary>
        /// <param name="v">The vector to clamp.</param>
        /// <returns>The clamped vector with each component between -1 and 1.</returns>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static Vector2d ClampOneInPlace(this Vector2d v)
        {
            v.x = v.x.ClampOne();
            v.y = v.y.ClampOne();
            return v;
        }

        /// <summary>
        /// Checks if the distance between two vectors is less than or equal to a specified factor.
        /// </summary>
        /// <param name="me">The current vector.</param>
        /// <param name="other">The vector to compare distance to.</param>
        /// <param name="factor">The maximum allowable distance.</param>
        /// <returns>True if the distance between the vectors is less than or equal to the factor, false otherwise.</returns>  
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static bool CheckDistance(this Vector2d me, Vector2d other, Fixed64 factor)
        {
            var dis = Vector2d.Distance(me, other);
            return dis <= factor;
        }

        /// <inheritdoc cref="Vector2d.Rotate(Vector2d, Fixed64)" />
        public static Vector2d Rotate(this Vector2d vec, Fixed64 angleInRadians)
        {
            return Vector2d.Rotate(vec, angleInRadians);
        }

        /// <inheritdoc cref="Vector2d.Abs(Vector2d)" />
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static Vector2d Abs(this Vector2d value)
        {
            return Vector2d.Abs(value);
        }

        /// <inheritdoc cref="Vector2d.Sign(Vector2d)" />
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static Vector2d Sign(this Vector2d value)
        {
            return Vector2d.Sign(value);
        }

        #endregion

        #region Conversion

        /// <inheritdoc cref="Vector2d.ToDegrees(Vector2d)" />
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static Vector2d ToDegrees(this Vector2d radians)
        {
            return Vector2d.ToDegrees(radians);
        }

        /// <inheritdoc cref="Vector2d.ToRadians(Vector2d)" />
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static Vector2d ToRadians(this Vector2d degrees)
        {
            return Vector2d.ToRadians(degrees);
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
        public static bool FuzzyEqualAbsolute(this Vector2d me, Vector2d other, Fixed64 allowedDifference)
        {
            return (me.x - other.x).Abs() <= allowedDifference &&
                   (me.y - other.y).Abs() <= allowedDifference;
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
        public static bool FuzzyEqual(this Vector2d me, Vector2d other, Fixed64? percentage = null)
        {
            Fixed64 p = percentage ?? Fixed64.Epsilon;
            return me.x.FuzzyComponentEqual(other.x, p) && me.y.FuzzyComponentEqual(other.y, p);
        }

        #endregion
    }
}