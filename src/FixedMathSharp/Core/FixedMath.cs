using System;
using System.Runtime.CompilerServices;

namespace FixedMathSharp
{
    /// <summary>
    /// A static class that provides a variety of fixed-point math functions.
    /// Fixed-point numbers are represented as <see cref="Fixed64"/>.
    /// </summary>
    public static partial class FixedMath
    {
        #region Fields and Constants

        /// <summary>
        /// Represents the number of bits to shift for fixed-point representation.
        /// </summary>
        public const int SHIFT_AMOUNT_I = 32;
        /// <summary>
        /// Represents the maximum value that can be produced by left-shifting 1 by SHIFT_AMOUNT_I bits and subtracting 1.
        /// </summary>
        /// <remarks>
        /// This constant is typically used as a bitmask to extract or limit values to the range
        /// defined by SHIFT_AMOUNT_I. 
        /// The value is always non-negative and fits within a 32-bit unsigned
        /// integer.
        /// </remarks>
        public const uint MAX_SHIFTED_AMOUNT_UI = (uint)((1L << SHIFT_AMOUNT_I) - 1);
        /// <summary>
        /// Represents a bitmask with all bits set except for the lowest SHIFT_AMOUNT_I bits.
        /// </summary>
        /// <remarks>
        /// This constant is typically used to isolate or clear the lower SHIFT_AMOUNT_I bits of
        /// an unsigned 64-bit value. 
        /// The value of SHIFT_AMOUNT_I determines how many least significant bits are masked out.
        /// </remarks>
        public const ulong MASK_UL = (ulong)(ulong.MaxValue << SHIFT_AMOUNT_I);

        /// <summary>
        /// Represents the largest possible value for a 64-bit fixed-point number.
        /// </summary>
        /// <remarks>
        /// Use this constant to perform comparisons or to initialize variables that require the
        /// maximum representable value for a 64-bit fixed-point type.
        /// </remarks>
        public const long MAX_VALUE_L = long.MaxValue;
        /// <summary>
        /// Represents the smallest possible value for a 64-bit fixed-point number.
        /// </summary>
        /// <remarks>
        /// Use this constant to check for underflow conditions or to initialize variables that
        /// require the minimum representable value for a 64-bit fixed-point type.
        /// </remarks>
        public const long MIN_VALUE_L = long.MinValue;

        /// <summary>
        /// Represents the value 1 shifted left by the number of bits specified by SHIFT_AMOUNT_I.
        /// </summary>
        public const long ONE_L = 1L << SHIFT_AMOUNT_I;

        // Precomputed scale factors only for performance-critical scenarios to avoid division at runtime

        /// <summary>
        /// Represents the precomputed scale factor used for floating-point calculations.
        /// </summary>
        /// <remarks>
        /// This constant is intended only for converting fixed-point values to floating-point representations in performance-critical scenarios.
        /// </remarks>
        public const float SCALE_FACTOR_F = 1.0f / ONE_L;
        /// <summary>
        /// Represents the precomputed scale factor used for double-precision calculations.
        /// </summary>
        /// <remarks>
        /// This constant is intended only for converting fixed-point values to double-precision representations in performance-critical scenarios.
        /// </remarks>
        public const double SCALE_FACTOR_D = 1.0 / ONE_L;
        /// <summary>
        /// Represents the precomputed scale factor used for decimal calculations.
        /// </summary>
        /// <remarks>
        /// This constant is intended only for converting fixed-point values to decimal representations in performance-critical scenarios.
        /// </remarks>
        public const decimal SCALE_FACTOR_M = 1.0m / ONE_L;

        /// <summary>
        /// The smallest non-zero raw increment representable by Fixed64.
        /// </summary>
        public const long MIN_INCREMENT_L = 1L;

        /// <summary>
        /// Default tolerance for fuzzy comparisons.
        /// Approximately 2^-24 (~5.96e-8) in value space.
        /// </summary>
        public const long DEFAULT_TOLERANCE_L = 1L << (SHIFT_AMOUNT_I - 24);


        #endregion

        #region FixedMath Operations

        /// <summary>
        /// Produces a value with the magnitude of the first argument and the sign of the second argument.
        /// </summary>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static Fixed64 CopySign(Fixed64 x, Fixed64 y)
        {
            return y >= Fixed64.Zero ? x.Abs() : -x.Abs();
        }

        /// <summary>
        /// Clamps value between 0 and 1 and returns value.
        /// </summary>
        /// <param name="value"></param>
        /// <returns></returns>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static Fixed64 Clamp01(Fixed64 value)
        {
            return value < Fixed64.Zero ? Fixed64.Zero : value > Fixed64.One ? Fixed64.One : value;
        }

        /// <summary>
        /// Clamps a fixed-point value between the given minimum and maximum values.
        /// </summary>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static Fixed64 Clamp(Fixed64 f1, Fixed64 min, Fixed64 max)
        {
            return f1 < min ? min : f1 > max ? max : f1;
        }

        /// <summary>
        /// Clamps a value to the inclusive range [min, max].
        /// </summary>
        /// <typeparam name="T">The type of the value, must implement IComparable&lt;T&gt;.</typeparam>
        /// <param name="value">The value to clamp.</param>
        /// <param name="min">The minimum allowed value.</param>
        /// <param name="max">The maximum allowed value.</param>
        /// <returns>The clamped value.</returns>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static T Clamp<T>(T value, T min, T max) where T : IComparable<T>
        {
            if (value.CompareTo(max) > 0) return max;
            if (value.CompareTo(min) < 0) return min;
            return value;
        }

        /// <summary>
        /// Clamps the value between -1 and 1 inclusive.
        /// </summary>
        /// <param name="f1">The Fixed64 value to clamp.</param>
        /// <returns>Returns a value clamped between -1 and 1.</returns>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static Fixed64 ClampOne(Fixed64 f1)
        {
            return f1 > Fixed64.One ? Fixed64.One : f1 < -Fixed64.One ? -Fixed64.One : f1;
        }

        /// <summary>
        /// Returns the absolute value of a Fixed64 number.
        /// </summary>
        public static Fixed64 Abs(Fixed64 value)
        {
            // For the minimum value, return the max to avoid overflow
            if (value.m_rawValue == MIN_VALUE_L)
                return Fixed64.MAX_VALUE;

            // Use branchless absolute value calculation
            long mask = value.m_rawValue >> 63; // If negative, mask will be all 1s; if positive, all 0s
            return Fixed64.FromRaw((value.m_rawValue + mask) ^ mask);
        }

        /// <summary>
        /// Returns the smallest integral value that is greater than or equal to the specified number.
        /// </summary>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static Fixed64 Ceiling(Fixed64 value)
        {
            bool hasFractionalPart = (value.m_rawValue & MAX_SHIFTED_AMOUNT_UI) != 0;
            return hasFractionalPart ? value.Floor() + Fixed64.One : value;
        }

        /// <summary>
        /// Returns the largest integer less than or equal to the specified number (floor function).
        /// </summary>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static Fixed64 Floor(Fixed64 value)
        {
            // Efficiently zeroes out the fractional part
            return Fixed64.FromRaw((long)((ulong)value.m_rawValue & FixedMath.MASK_UL));
        }

        /// <summary>
        /// Returns the larger of two fixed-point values.
        /// </summary>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static Fixed64 Max(Fixed64 f1, Fixed64 f2)
        {
            return f1 >= f2 ? f1 : f2;
        }

        /// <summary>
        /// Returns the smaller of two fixed-point values.
        /// </summary>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static Fixed64 Min(Fixed64 a, Fixed64 b)
        {
            return (a < b) ? a : b;
        }

        /// <summary>
        /// Rounds a fixed-point number to the nearest integral value, based on the specified rounding mode.
        /// </summary>
        public static Fixed64 Round(Fixed64 value, MidpointRounding mode = MidpointRounding.ToEven)
        {
            long fractionalPart = value.m_rawValue & MAX_SHIFTED_AMOUNT_UI;
            Fixed64 integralPart = value.Floor();
            if (fractionalPart < Fixed64.Half.m_rawValue)
                return integralPart;

            if (fractionalPart > Fixed64.Half.m_rawValue)
                return integralPart + Fixed64.One;

            // When value is exactly Fixed64.Halfway between two numbers
            return mode switch
            {
                MidpointRounding.AwayFromZero => value.m_rawValue > 0 ? integralPart + Fixed64.One : integralPart,// For negative midpoints, Floor() is already away from zero
                _ => (integralPart.m_rawValue & ONE_L) == 0 ? integralPart : integralPart + Fixed64.One,// Rounds to the nearest even number (default behavior)
            };
        }

        /// <summary>
        /// Rounds a fixed-point number to a specific number of decimal places.
        /// </summary>
        public static Fixed64 RoundToPrecision(Fixed64 value, int decimalPlaces, MidpointRounding mode = MidpointRounding.ToEven)
        {
            if (decimalPlaces < 0 || decimalPlaces >= Pow10Lookup.Length)
                throw new ArgumentOutOfRangeException(nameof(decimalPlaces), "Decimal places out of range.");

            int factor = Pow10Lookup[decimalPlaces];
            Fixed64 scaled = value * factor;
            long rounded = Round(scaled, mode).m_rawValue;
            return new Fixed64(rounded + (factor / 2)) / factor;
        }

        /// <summary>
        /// Squares the Fixed64 value.
        /// </summary>
        /// <param name="value">The Fixed64 value to square.</param>
        /// <returns>The squared value.</returns>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static Fixed64 Squared(Fixed64 value)
        {
            return value * value;
        }

        /// <summary>
        /// Adds two fixed-point numbers without performing overflow checking.
        /// </summary>  
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static Fixed64 FastAdd(Fixed64 x, Fixed64 y)
        {
            return Fixed64.FromRaw(x.m_rawValue + y.m_rawValue);
        }

        /// <summary>
        /// Subtracts two fixed-point numbers without performing overflow checking.
        /// </summary>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static Fixed64 FastSub(Fixed64 x, Fixed64 y)
        {
            return Fixed64.FromRaw(x.m_rawValue - y.m_rawValue);
        }

        /// <summary>
        /// Multiplies two fixed-point numbers without overflow checking for performance-critical scenarios.
        /// </summary>
        public static Fixed64 FastMul(Fixed64 x, Fixed64 y)
        {
            long xl = x.m_rawValue;
            long yl = y.m_rawValue;

            // Split values into high and low bits for long multiplication
            ulong xlo = (ulong)(xl & MAX_SHIFTED_AMOUNT_UI);
            long xhi = xl >> SHIFT_AMOUNT_I;
            ulong ylo = (ulong)(yl & MAX_SHIFTED_AMOUNT_UI);
            long yhi = yl >> SHIFT_AMOUNT_I;

            // Perform partial products
            ulong lolo = xlo * ylo;
            long lohi = (long)xlo * yhi;
            long hilo = xhi * (long)ylo;
            long hihi = xhi * yhi;

            // Combine the results
            ulong loResult = lolo >> SHIFT_AMOUNT_I;
            long midResult1 = lohi;
            long midResult2 = hilo;
            long hiResult = hihi << SHIFT_AMOUNT_I;

            long sum = (long)loResult + midResult1 + midResult2 + hiResult;
            return Fixed64.FromRaw(sum);
        }

        /// <summary>
        /// Fast modulus without the checks performed by the '%' operator.
        /// </summary>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static Fixed64 FastMod(Fixed64 x, Fixed64 y)
        {
            return Fixed64.FromRaw(x.m_rawValue % y.m_rawValue);
        }

        /// <summary>
        /// Performs a smooth step interpolation between two values.
        /// </summary>
        /// <remarks>
        /// The interpolation follows a cubic Hermite curve where the function starts at `a`,
        /// accelerates, and then decelerates towards `b`, ensuring smooth transitions.
        /// </remarks>
        /// <param name="a">The starting value.</param>
        /// <param name="b">The ending value.</param>
        /// <param name="t">A value between 0 and 1 that represents the interpolation factor.</param>
        /// <returns>The interpolated value between `a` and `b`.</returns>
        public static Fixed64 SmoothStep(Fixed64 a, Fixed64 b, Fixed64 t)
        {
            t = t * t * (Fixed64.Three - Fixed64.Two * t);
            return LinearInterpolate(a, b, t);
        }

        /// <summary>
        /// Performs a cubic Hermite interpolation between two points, using specified tangents.
        /// </summary>
        /// <remarks>
        /// This method interpolates smoothly between `p0` and `p1` while considering the tangents `m0` and `m1`.
        /// It is useful for animation curves and smooth motion transitions.
        /// </remarks>
        /// <param name="p0">The first point.</param>
        /// <param name="p1">The second point.</param>
        /// <param name="m0">The tangent (slope) at `p0`.</param>
        /// <param name="m1">The tangent (slope) at `p1`.</param>
        /// <param name="t">A value between 0 and 1 that represents the interpolation factor.</param>
        /// <returns>The interpolated value between `p0` and `p1`.</returns>
        public static Fixed64 CubicInterpolate(Fixed64 p0, Fixed64 p1, Fixed64 m0, Fixed64 m1, Fixed64 t)
        {
            Fixed64 t2 = t * t;
            Fixed64 t3 = t2 * t;
            return (Fixed64.Two * p0 - Fixed64.Two * p1 + m0 + m1) * t3
                 + (-Fixed64.Three * p0 + Fixed64.Three * p1 - Fixed64.Two * m0 - m1) * t2
                 + m0 * t + p0;
        }

        /// <summary>
        /// Performs linear interpolation between two fixed-point values based on the interpolant t (0 greater or equal to `t` and less than or equal to 1).
        /// </summary>
        public static Fixed64 LinearInterpolate(Fixed64 from, Fixed64 to, Fixed64 t)
        {
            if (t.m_rawValue >= ONE_L)
                return to;
            if (t.m_rawValue <= 0)
                return from;

            return (to * t) + (from * (Fixed64.One - t));
        }

        /// <summary>
        /// Moves a value from 'from' to 'to' by a maximum step of 'maxAmount'. 
        /// Ensures the value does not exceed 'to'.
        /// </summary>
        public static Fixed64 MoveTowards(Fixed64 from, Fixed64 to, Fixed64 maxAmount)
        {
            if (from < to)
            {
                from += maxAmount;
                if (from > to)
                    from = to;
            }
            else if (from > to)
            {
                from -= maxAmount;
                if (from < to)
                    from = to;
            }

            return Fixed64.FromRaw(from.m_rawValue);
        }

        /// <summary>
        /// Adds two <see cref="long"/> values and checks for overflow.
        /// If an overflow occurs during addition, the <paramref name="overflow"/> parameter is set to true.
        /// </summary>
        /// <param name="x">The first operand to add.</param>
        /// <param name="y">The second operand to add.</param>
        /// <param name="overflow">
        /// A reference parameter that is set to true if an overflow is detected during the addition.
        /// The existing value of <paramref name="overflow"/> is preserved if already true.
        /// </param>
        /// <returns>The sum of <paramref name="x"/> and <paramref name="y"/>.</returns>
        /// <remarks>
        /// Overflow is detected by checking for a change in the sign bit that indicates a wrap-around.
        /// Additionally, a special check is performed for adding <see cref="Fixed64.MIN_VALUE"/> and -1, 
        /// as this is a known edge case for overflow.
        /// </remarks>
        public static long AddOverflowHelper(long x, long y, ref bool overflow)
        {
            long sum = x + y;
            // Check for overflow using sign bit changes
            overflow |= ((x ^ y ^ sum) & MIN_VALUE_L) != 0;
            // Special check for the case when x is long.Fixed64.MinValue and y is negative
            if (x == long.MinValue && y == -1)
                overflow = true;
            return sum;
        }

        #endregion
    }
}
