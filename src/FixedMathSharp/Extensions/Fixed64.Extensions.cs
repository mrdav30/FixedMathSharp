using System;
using System.Runtime.CompilerServices;
using System.Threading.Tasks;
using static System.Net.Mime.MediaTypeNames;

namespace FixedMathSharp
{
    public static class Fixed64Extensions
    {
        #region Fixed64 Operations

        /// <inheritdoc cref="Fixed64.Sign(Fixed64)" />
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static int Sign(this Fixed64 value)
        {
            return Fixed64.Sign(value);
        }

        /// <inheritdoc cref="Fixed64.IsInteger(Fixed64)" />
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static bool IsInteger(this Fixed64 value)
        {
            return Fixed64.IsInteger(value);
        }
        
        /// <inheritdoc cref="FixedMath.Squared(Fixed64)" />
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static Fixed64 Squared(this Fixed64 value)
        {
            return FixedMath.Squared(value);
        }

        /// <inheritdoc cref="FixedMath.Round(Fixed64, MidpointRounding)" />
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static Fixed64 Round(this Fixed64 value, MidpointRounding mode = MidpointRounding.ToEven)
        {
            return FixedMath.Round(value, mode);
        }

        /// <inheritdoc cref="FixedMath.RoundToPrecision(Fixed64, int, MidpointRounding)" />
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static Fixed64 RoundToPrecision(this Fixed64 value, int places, MidpointRounding mode = MidpointRounding.ToEven)
        {
            return FixedMath.RoundToPrecision(value, places, mode);
        }

        /// <inheritdoc cref="FixedMath.ClampOne(Fixed64)" />
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static Fixed64 ClampOne(this Fixed64 f1)
        {
            return FixedMath.ClampOne(f1);
        }

        /// <inheritdoc cref="FixedMath.Clamp01(Fixed64)" />
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static Fixed64 Clamp01(this Fixed64 f1)
        {
            return FixedMath.Clamp01(f1);
        }

        /// <inheritdoc cref="FixedMath.Abs(Fixed64)" />
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static Fixed64 Abs(this Fixed64 value)
        {
            return FixedMath.Abs(value);
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

        /// <inheritdoc cref="FixedMath.FastAdd(Fixed64, Fixed64)" />
        public static Fixed64 FastAdd(this Fixed64 a, Fixed64 b)
        {
            return FixedMath.FastAdd(a, b);
        }

        /// <inheritdoc cref="FixedMath.FastSub(Fixed64, Fixed64)" />
        public static Fixed64 FastSub(this Fixed64 a, Fixed64 b)
        {
            return FixedMath.FastSub(a, b);
        }

        /// <inheritdoc cref="FixedMath.FastMul(Fixed64, Fixed64)" />
        public static Fixed64 FastMul(this Fixed64 a, Fixed64 b)
        {
            return FixedMath.FastMul(a, b);
        }

        /// <inheritdoc cref="FixedMath.FastMod(Fixed64, Fixed64)" />
        public static Fixed64 FastMod(this Fixed64 a, Fixed64 b)
        {
            return FixedMath.FastMod(a, b);
        }

        /// <inheritdoc cref="FixedMath.Floor(Fixed64)" />
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static Fixed64 Floor(this Fixed64 value)
        {
            return FixedMath.Floor(value);
        }

        /// <inheritdoc cref="FixedMath.Ceiling(Fixed64)" />
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static Fixed64 Ceiling(this Fixed64 value)
        {
            return FixedMath.Ceiling(value);
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

        #endregion

        #region Conversion

        /// <summary>
        /// Converts the Fixed64 value to a string formatted to 2 decimal places.
        /// </summary>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static string ToFormattedString(this Fixed64 f1)
        {
            return f1.ToPreciseFloat().ToString("0.##");
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
            return Math.Round((double)f1, precision, MidpointRounding.AwayFromZero);
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

        #endregion

        #region Equality

        /// <summary>
        /// Checks if the value is greater than epsilon (positive or negative).
        /// Useful for determining if a value is effectively non-zero with a given precision.
        /// </summary>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static bool MoreThanEpsilon(this Fixed64 d)
        {
            return d.Abs() > Fixed64.Epsilon;
        }

        /// <summary>
        /// Checks if the value is less than epsilon (i.e., effectively zero).
        /// Useful for determining if a value is close enough to zero with a given precision.
        /// </summary>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static bool LessThanEpsilon(this Fixed64 d)
        {
            return d.Abs() < Fixed64.Epsilon;
        }

        /// <summary>
        /// Helper method to compare individual vector components for approximate equality, allowing a fractional difference.
        /// Handles zero components by only using the allowed percentage difference.
        /// </summary>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static bool FuzzyComponentEqual(this Fixed64 a, Fixed64 b, Fixed64 percentage)
        {
            var diff = (a - b).Abs();
            var allowedErr = a.Abs() * percentage;
            // Compare directly to percentage if a is zero
            // Otherwise, use percentage of a's magnitude
            return a == Fixed64.Zero ? diff <= percentage : diff <= allowedErr;
        }

        #endregion
    }
}