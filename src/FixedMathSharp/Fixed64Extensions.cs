using System.Runtime.CompilerServices;
using System.Threading.Tasks;

namespace FixedMathSharp
{
    public static class Fixed64Extensions
    {
        /// <summary>
        /// Checks if the value is greater than epsilon (positive or negative).
        /// Useful for determining if a value is effectively non-zero with a given precision.
        /// </summary>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static bool MoreThanEpsilon(this Fixed64 f1)
        {
            return f1 > FixedMath.Epsilon || f1 < -FixedMath.Epsilon;
        }

        /// <summary>
        /// Returns a number indicating the sign of a Fix64 number.
        /// Returns 1 if the value is positive, 0 if is 0, and -1 if it is negative.
        /// </summary>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static int Sign(this Fixed64 value)
        {
            // Return the sign of the value, optimizing for branchless comparison
            return value.RawValue < 0 ? -1 : (value.RawValue > 0 ? 1 : 0);
        }

        /// <summary>
        /// Squares the Fixed64 value.
        /// </summary>
        /// <param name="value">The Fixed64 value to square.</param>
        /// <returns>The squared value.</returns>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static Fixed64 Squared(this Fixed64 value)
        {
            return value * value;
        }

        /// <summary>
        /// Rounds the Fixed64 value.
        /// </summary>
        /// <param name="value">The Fixed64 value to round.</param>
        /// <returns>The rounded value.</returns>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static Fixed64 Round(this Fixed64 value, int places = 0)
        {
            return places == 0 ? FixedMath.Round(value) : FixedMath.Round(value, places);
        }

        /// <summary>
        /// Clamps the value between -1 and 1 inclusive.
        /// </summary>
        /// <param name="f1">The Fixed64 value to clamp.</param>
        /// <returns>Returns a value clamped between -1 and 1.</returns>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static Fixed64 ClampOne(this Fixed64 f1)
        {
            if (f1 > FixedMath.One) return FixedMath.One;
            if (f1 < -FixedMath.One) return -FixedMath.One;
            return f1;
        }

        /// <summary>
        /// Returns the absolute value of a Fixed64 number.
        /// Abs(FixedMath.MinValue) returns FixedMath.MaxValue due to overflow behavior.
        /// Optimized for branchless computation.
        /// </summary>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static Fixed64 Abs(this Fixed64 value)
        {
            // For the minimum value, return the max to avoid overflow
            if (value.RawValue == FixedMath.MIN_VALUE_L)
                return FixedMath.MaxValue;

            // Use branchless absolute value calculation
            long mask = value.RawValue >> 63; // If negative, mask will be all 1s; if positive, all 0s
            return Fixed64.FromRaw((value.RawValue + mask) ^ mask);
        }

        /// <summary>
        /// Checks if the absolute value of x is less than y.
        /// </summary>
        /// <param name="x">The value to compare.</param>
        /// <param name="y">The comparison threshold.</param>
        /// <returns>True if |x| &lt; y; otherwise false.</returns>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static bool AbsLessThan(this Fixed64 x, Fixed64 y)
        {
            return Abs(x) < y;
        }

        /// <summary>
        /// Returns the largest integer less than or equal to the specified number (floor function).
        /// </summary>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static Fixed64 Floor(this Fixed64 value)
        {
            // Efficiently zeroes out the fractional part
            return Fixed64.FromRaw((long)((ulong)value.RawValue & FixedMath.MASK_UL));
        }

        /// <summary>
        /// Rounds the Fixed64 value to the nearest integer.
        /// </summary>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static int RoundToInt(this Fixed64 x)
        {
            return (int)FixedMath.Round(x);
        }

        /// <summary>
        /// Rounds up the Fixed64 value to the nearest integer.
        /// </summary>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static int CeilToInt(this Fixed64 x)
        {
            return (int)FixedMath.Ceiling(x);
        }

        /// <summary>
        /// Rounds down the Fixed64 value to the nearest integer.
        /// </summary>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static int FloorToInt(this Fixed64 x)
        {
            return (int)Floor(x);
        }

        /// <summary>
        /// Converts the Fixed64 value to a double with specified decimal precision.
        /// </summary>
        /// <param name="f1">The Fixed64 value to convert.</param>
        /// <param name="precision">The number of decimal places to round to.</param>
        /// <returns>The formatted double value.</returns>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static double ToFormattedDouble(this Fixed64 f1, int precision = 2)
        {
            return System.Math.Round((double)f1, precision, System.MidpointRounding.AwayFromZero);
        }

        /// <summary>
        /// Converts the Fixed64 value to a float with 2 decimal points of precision.
        /// </summary>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static float ToFormattedFloat(this Fixed64 f1)
        {
            return (float)ToFormattedDouble(f1);
        }

        /// <summary>
        /// Converts the Fixed64 value to a precise float representation (without rounding).
        /// </summary>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static float ToPreciseFloat(this Fixed64 f1)
        {
            return (float)(double)f1;
        }

        /// <summary>
        /// Converts the angle in degrees to radians.
        /// </summary>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static Fixed64 ToRadians(this Fixed64 angleInDegrees)
        {
            return FixedMath.DegToRad(angleInDegrees);
        }

        /// <summary>
        /// Converts the angle in radians to degree.
        /// </summary>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static Fixed64 ToDegree(this Fixed64 angleInRadians)
        {
            return FixedMath.RadToDeg(angleInRadians);
        }

        /// <summary>
        /// Converts the Fixed64 value to a string formatted to 2 decimal places.
        /// </summary>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static string ToFormattedString(this Fixed64 f1)
        {
            return f1.ToPreciseFloat().ToString("0.##");
        }
    }
}