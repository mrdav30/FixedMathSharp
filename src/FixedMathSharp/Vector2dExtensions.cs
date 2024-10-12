namespace FixedMathSharp
{
    public static class Vector2dExtensions
    {
        /// <summary>
        /// Clamps each component of the vector to the range [-1, 1] in place and returns the modified vector.
        /// </summary>
        /// <param name="v">The vector to clamp.</param>
        /// <returns>The clamped vector with each component between -1 and 1.</returns>
        public static Vector2d ClampOneInPlace(this Vector2d v)
        {
            v.x = v.x.ClampOne();
            v.y = v.y.ClampOne();
            return v;
        }

        /// <summary>
        /// Converts each component of the vector from radians to degrees.
        /// </summary>
        /// <param name="radians">The vector with components in radians.</param>
        /// <returns>A new vector with components converted to degrees.</returns>
        public static Vector2d ToDegrees(this Vector2d radians)
        {
            return new Vector2d(
                FixedMath.Rad2Deg * radians.x,
                FixedMath.Rad2Deg * radians.y
            );
        }

        /// <summary>
        /// Converts each component of the vector from degrees to radians.
        /// </summary>
        /// <param name="degrees">The vector with components in degrees.</param>
        /// <returns>A new vector with components converted to radians.</returns>
        public static Vector2d ToRadians(this Vector2d degrees)
        {
            return new Vector2d(
                FixedMath.Deg2Rad * degrees.x,
                FixedMath.Deg2Rad * degrees.y
            );
        }

        /// <summary>
        /// Compares two vectors for approximate equality, allowing an absolute epsilon difference.
        /// </summary>
        public static bool FuzzyEqual(this Vector2d me, Vector2d other)
        {
            return me.FuzzyEqualAbsolute(other, FixedMath.Epsilon);
        }

        /// <summary>
        /// Compares two vectors for approximate equality, allowing a fixed absolute difference.
        /// </summary>
        /// <param name="me">The current vector.</param>
        /// <param name="other">The vector to compare against.</param>
        /// <param name="allowedDifference">The allowed absolute difference between each component.</param>
        /// <returns>True if the components are within the allowed difference, false otherwise.</returns>
        public static bool FuzzyEqualAbsolute(this Vector2d me, Vector2d other, Fixed64 allowedDifference)
        {
            var dx = me.x - other.x;
            if (dx.Abs() > allowedDifference)
                return false;

            var dy = me.y - other.y;
            return dy.Abs() <= allowedDifference;
        }

        /// <summary>
        /// Compares two vectors for approximate equality, allowing a fractional difference (percentage).
        /// Handles zero components by only using the allowed percentage difference.
        /// </summary>
        /// <param name="me">The current vector.</param>
        /// <param name="other">The vector to compare against.</param>
        /// <param name="percentage">The allowed fractional difference (percentage) for each component.</param>
        /// <returns>True if the components are within the allowed percentage difference, false otherwise.</returns>
        public static bool FuzzyEqual(this Vector2d me, Vector2d other, Fixed64 percentage)
        {
            var dx = me.x - other.x;
            if (dx.Abs() > (me.x == FixedMath.Zero ? percentage : me.x * percentage))
                return false;

            var dy = me.y - other.y;
            return dy.Abs() < (me.y == FixedMath.Zero ? percentage : me.y * percentage);
        }

        /// <summary>
        /// Checks if the distance between two vectors is less than or equal to a specified factor.
        /// </summary>
        /// <param name="me">The current vector.</param>
        /// <param name="other">The vector to compare distance to.</param>
        /// <param name="factor">The maximum allowable distance.</param>
        /// <returns>True if the distance between the vectors is less than or equal to the factor, false otherwise.</returns>
        public static bool CheckDistance(this Vector2d me, Vector2d other, Fixed64 factor)
        {
            var dis = Vector2d.Distance(me, other);
            return dis <= factor;
        }

        /// <summary>
        /// Calculates the squared distance between two vectors, avoiding the need for a square root operation.
        /// </summary>
        /// <param name="vec">The current vector.</param>
        /// <param name="other">The vector to calculate the squared distance to.</param>
        /// <returns>The squared distance between the two vectors.</returns>
        public static Fixed64 V2SqrDistance(this Vector2d vec, Vector2d other)
        {
            Fixed64 temp1 = vec.x - other.x;
            temp1 *= temp1;
            Fixed64 temp2 = vec.y - other.y;
            temp2 *= temp2;
            return temp1 + temp2;
        }

        /// <summary>
        /// Rotates this vector by the specified angle (in radians).
        /// </summary>
        /// <param name="vec">The vector to rotate.</param>
        /// <param name="angleInRadians">The angle in radians.</param>
        /// <returns>The rotated vector.</returns>
        public static Vector2d Rotate(this Vector2d vec, Fixed64 angleInRadians)
        {
            Fixed64 cos = FixedMath.Cos(angleInRadians);
            Fixed64 sin = FixedMath.Sin(angleInRadians);
            return new Vector2d(
                vec.x * cos - vec.y * sin,
                vec.x * sin + vec.y * cos
            );
        }
    }
}