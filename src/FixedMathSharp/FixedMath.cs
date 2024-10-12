using System;

namespace FixedMathSharp
{
    /// <summary>
    /// A static class that provides a variety of fixed-point math functions.
    /// Fixed-point numbers are represented as <see cref="Fixed64"/>.
    /// </summary>
    public static partial class FixedMath
    {
        #region Properties

        public const int SHIFT_AMOUNT_I = 32;
        public const uint MAX_SHIFTED_AMOUNT_UI = (uint)((1L << SHIFT_AMOUNT_I) - 1);
        public const ulong MASK_UL = (ulong)(ulong.MaxValue << SHIFT_AMOUNT_I);
        
        public const long MAX_VALUE_L = long.MaxValue; // Max possible value for Fixed64
        public const long MIN_VALUE_L = long.MinValue; // Min possible value for Fixed64

        public const long ONE_L = 1L << SHIFT_AMOUNT_I;

        // Constant values for Fixed64 representation
        public static readonly Fixed64 MaxValue = new Fixed64(MAX_VALUE_L);
        public static readonly Fixed64 MinValue = new Fixed64(MIN_VALUE_L);

        // Precomputed scale factors
        public static readonly float FloatScaleFactor = 1.0f / ONE_L;
        public static readonly double DoubleScaleFactor = 1.0 / ONE_L;
        public static readonly decimal DecimalScaleFactor = 1.0m / ONE_L;

        /// <summary>
        /// Represents the smallest possible value that can be represented by the Fixed64 format.
        /// Precision of this type is 2^-SHIFT_AMOUNT, 
        /// i.e. 1 / (2^SHIFT_AMOUNT) where SHIFT_AMOUNT defines the fractional bits.
        /// </summary>
        public static readonly Fixed64 Epsilon = new Fixed64(1L);

        public static readonly Fixed64 One = new Fixed64(ONE_L);
        public static readonly Fixed64 Double = One * 2;
        public static readonly Fixed64 Triple = One * 3;
        public static readonly Fixed64 Half = One / 2;
        public static readonly Fixed64 Quarter = One / 4;
        public static readonly Fixed64 Eighth = One / 8;
        public static readonly Fixed64 Zero = new Fixed64(0);

        #endregion

        /// <summary>
        /// Produces a value with the magnitude of the first argument and the sign of the second argument.
        /// </summary>
        public static Fixed64 CopySign(Fixed64 x, Fixed64 y)
        {
            return y >= Zero ? x.Abs() : -x.Abs();
        }

        /// <summary>
        /// Clamps value between 0 and 1 and returns value.
        /// </summary>
        /// <param name="value"></param>
        /// <returns></returns>
        public static Fixed64 Clamp01(Fixed64 value)
        {
            if (value < Zero)
                return Zero;

            if (value > One)
                return One;

            return value;
        }

        /// <summary>
        /// Returns the absolute value of a Fixed64 number.
        /// </summary>
        public static Fixed64 Abs(Fixed64 value)
        {
            // For the minimum value, return the max to avoid overflow
            if (value.RawValue == MIN_VALUE_L)
                return MaxValue;

            // Use branchless absolute value calculation
            long mask = value.RawValue >> 63; // If negative, mask will be all 1s; if positive, all 0s
            return Fixed64.FromRaw((value.RawValue + mask) ^ mask);
        }

        /// <summary>
        /// Returns the smallest integral value that is greater than or equal to the specified number.
        /// </summary>
        public static Fixed64 Ceiling(Fixed64 value)
        {
            bool hasFractionalPart = (value.RawValue & MAX_SHIFTED_AMOUNT_UI) != 0;
            return hasFractionalPart ? value.Floor() + One : value;
        }

        /// <summary>
        /// Returns the larger of two fixed-point values.
        /// </summary>
        public static Fixed64 Max(Fixed64 f1, Fixed64 f2)
        {
            return f1 >= f2 ? f1 : f2;
        }

        /// <summary>
        /// Returns the smaller of two fixed-point values.
        /// </summary>
        public static Fixed64 Min(Fixed64 a, Fixed64 b)
        {
            return (a < b) ? a : b;
        }

        /// <summary>
        /// Rounds a fixed-point number to the nearest integral value, based on the specified rounding mode.
        /// </summary>
        public static Fixed64 Round(Fixed64 value, MidpointRounding mode = MidpointRounding.ToEven)
        {
            long fractionalPart = value.RawValue & MAX_SHIFTED_AMOUNT_UI;
            Fixed64 integralPart = value.Floor();
            if (fractionalPart < Half.RawValue)
                return integralPart;

            if (fractionalPart > Half.RawValue)
                return integralPart + One;

            // When value is exactly halfway between two numbers
            return mode switch
            {
                MidpointRounding.AwayFromZero => value.RawValue > 0 ? integralPart + One : integralPart - One,// If it's exactly halfway, round away from zero
                _ => (integralPart.RawValue & ONE_L) == 0 ? integralPart : integralPart + One,// Rounds to the nearest even number (default behavior)
            };
        }

        /// <summary>
        /// Rounds a fixed-point number to a specific number of decimal places.
        /// </summary>
        public static Fixed64 Round(Fixed64 value, int decimalPlaces, MidpointRounding mode = MidpointRounding.ToEven)
        {
            if (decimalPlaces < 0 || decimalPlaces >= Pow10Lookup.Length)
                throw new ArgumentOutOfRangeException(nameof(decimalPlaces), "Decimal places out of range.");

            int factor = Pow10Lookup[decimalPlaces];
            Fixed64 scaled = value * factor;
            long rounded = Round(scaled, mode).RawValue;
            return new Fixed64(rounded + (factor / 2)) / factor;
        }

        /// <summary>
        /// Adds two fixed-point numbers without performing overflow checking.
        /// </summary>
        public static Fixed64 FastAdd(Fixed64 x, Fixed64 y)
        {
            return Fixed64.FromRaw(x.RawValue + y.RawValue);
        }

        /// <summary>
        /// Subtracts two fixed-point numbers without performing overflow checking.
        /// </summary>
        public static Fixed64 FastSub(Fixed64 x, Fixed64 y)
        {
            return Fixed64.FromRaw(x.RawValue - y.RawValue);
        }

        /// <summary>
        /// Multiplies two fixed-point numbers without overflow checking for performance-critical scenarios.
        /// </summary>
        public static Fixed64 FastMul(Fixed64 x, Fixed64 y)
        {
            long xl = x.RawValue;
            long yl = y.RawValue;

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
        public static Fixed64 FastMod(Fixed64 x, Fixed64 y)
        {
            return Fixed64.FromRaw(x.RawValue % y.RawValue);
        }

        /// <summary>
        /// Performs linear interpolation between two fixed-point values based on the interpolant t (0 <= t <= 1).
        /// </summary>
        public static Fixed64 LinearInterpolate(Fixed64 from, Fixed64 to, Fixed64 t)
        {
            if (t.RawValue >= ONE_L)
                return to;
            if (t.RawValue <= 0)
                return from;

            return (to * t) + (from * (One - t));
        }

        /// <summary>
        /// Clamps a fixed-point value between the given minimum and Fixed64.MaxValue.
        /// </summary>
        public static Fixed64 Clamp(Fixed64 f1, Fixed64 min)
        {
            return Clamp(f1, min, MaxValue);
        }

        /// <summary>
        /// Clamps a fixed-point value between the given minimum and maximum values.
        /// </summary>
        public static Fixed64 Clamp(Fixed64 f1, Fixed64 min, Fixed64 max)
        {
            if (f1 < min) return min;
            if (f1 > max) return max;
            return f1;
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

            return Fixed64.FromRaw(from.RawValue);
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
        /// Additionally, a special check is performed for adding <see cref="long.MinValue"/> and -1, 
        /// as this is a known edge case for overflow.
        /// </remarks>
        public static long AddOverflowHelper(long x, long y, ref bool overflow)
        {
            long sum = x + y;
            // Check for overflow using sign bit changes
            overflow |= ((x ^ y ^ sum) & MIN_VALUE_L) != 0;

            // Special check for the case when x is long.MinValue and y is negative
            if (x == long.MinValue && y == -1)
                overflow = true;

            return sum;
        }

    }
}