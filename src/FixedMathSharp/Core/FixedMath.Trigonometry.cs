//=======================================================================
// FixedMath.Trigonometry.cs
//=======================================================================
// MIT License, Copyright (c) 2024–present David Oravsky (mrdav30)
// See LICENSE file in the project root for full license information.
//=======================================================================

using System;
using System.Runtime.CompilerServices;

namespace FixedMathSharp
{
    public static partial class FixedMath
    {
        #region Fields and Constants

        private static readonly int[] s_pow10Lookup = {
            1,           // 10^0
            10,          // 10^1
            100,         // 10^2
            1000,        // 10^3
            10000,       // 10^4
            100000,      // 10^5
            1000000,     // 10^6
            10000000,    // 10^7
            100000000,   // 10^8
            1000000000,  // 10^9
        };

        /// <summary>
        /// Provides a lookup table of integer powers of 10 from 10^0 to 10^9.
        /// </summary>
        /// <remarks>
        /// This array can be used to efficiently retrieve the value of 10 raised to an integer
        /// exponent within the supported range, avoiding repeated calculations. 
        /// The index corresponds to the exponent.
        /// </remarks>
        public static ReadOnlySpan<int> Pow10Lookup => s_pow10Lookup;

        // Trigonometric and logarithmic constants

        internal const double PI_DOUBLE = 3.14159265358979323846d;
        /// <summary>
        /// Represents the mathematical constant π (pi).
        /// </summary>
        /// <remarks>The value is approximately 3.14159265358979323846.</remarks>
        internal static readonly long PI_LONG = (long)(PI_DOUBLE * ONE_L);

        internal const double LN2_DOUBLE = 0.6931471805599453d;
        /// <summary>
        /// Represents the mathematical constant natural logarithm of 2 (ln(2)).
        /// </summary>
        /// <remarks>The value is approximately 0.6931471805599453.</remarks>
        internal static readonly long LN2_LONG = (long)(LN2_DOUBLE * ONE_L);

        // Asin Padé approximations
        internal const double PADE_A1_DOUBLE = 0.183320102d;
        internal static readonly long PADE_A1_LONG = (long)(PADE_A1_DOUBLE * ONE_L);
        internal const double PADE_A2_DOUBLE = 0.0218804099d;
        internal static readonly long PADE_A2_LONG = (long)(PADE_A2_DOUBLE * ONE_L);

        // Carefully optimized polynomial coefficients for sin(x), ensuring maximum precision in Fixed64 math.
        internal const double SIN_COEFF_3_DOUBLE = 0.16666667605750262737274169921875d; // 1/3!
        internal static readonly long SIN_COEFF_3_LONG = (long)(SIN_COEFF_3_DOUBLE * ONE_L);
        internal const double SIN_COEFF_5_DOUBLE = 0.0083328341133892536163330078125d; // 1/5!
        internal static readonly long SIN_COEFF_5_LONG = (long)(SIN_COEFF_5_DOUBLE * ONE_L);
        internal const double SIN_COEFF_7_DOUBLE = 0.00019588856957852840423583984375d; // 1/7!
        internal static readonly long SIN_COEFF_7_LONG = (long)(SIN_COEFF_7_DOUBLE * ONE_L);

        private static readonly long[] s_pow2PositiveFractionLookup =
        {
            6074001000L,
            5107605667L,
            4683695048L,
            4485121744L,
            4389014833L,
            4341736423L,
            4318288544L,
            4306612134L,
            4300785774L,
            4297875550L,
            4296421177L,
            4295694175L,
            4295330720L,
            4295149004L,
            4295058149L,
            4295012722L,
            4294990009L,
            4294978653L,
            4294972974L,
            4294970135L,
            4294968716L,
            4294968006L,
            4294967651L,
            4294967473L,
            4294967385L,
            4294967340L,
            4294967318L,
            4294967307L,
            4294967302L,
            4294967299L,
            4294967297L,
            4294967297L
        };

        private static readonly long[] s_pow2NegativeFractionLookup =
        {
            3037000500L,
            3611622603L,
            3938502376L,
            4112874773L,
            4202935003L,
            4248701965L,
            4271771996L,
            4283353945L,
            4289156690L,
            4292061010L,
            4293513907L,
            4294240540L,
            4294603903L,
            4294785595L,
            4294876445L,
            4294921870L,
            4294944583L,
            4294955939L,
            4294961618L,
            4294964457L,
            4294965876L,
            4294966586L,
            4294966941L,
            4294967119L,
            4294967207L,
            4294967252L,
            4294967274L,
            4294967285L,
            4294967290L,
            4294967293L,
            4294967295L,
            4294967295L
        };

        #endregion

        #region FixedTrigonometry Operations

        /// <summary>
        /// Raises the base number b to the power of exp.
        /// Uses logarithms to compute power efficiently for fixed-point values.
        /// </summary>
        /// <exception cref="DivideByZeroException">
        /// The base was Fixed64.Zero, with a negative expFixed64.Onent
        /// </exception>
        /// <exception cref="ArgumentOutOfRangeException">
        /// The base was negative, with a non-Fixed64.Zero expFixed64.Onent
        /// </exception>
        public static Fixed64 Pow(Fixed64 b, Fixed64 exp)
        {
            if (b == Fixed64.One)
                return Fixed64.One;

            if (exp.m_rawValue == 0)
                return Fixed64.One;

            if (b.m_rawValue == 0)
            {
                if (exp.m_rawValue < 0)
                    throw new DivideByZeroException("Cannot raise 0 to a negative power.");

                return Fixed64.Zero;
            }

            Fixed64 log2 = Log2(b);  // Calculate logarithm base 2
            return Pow2(exp * log2);  // Raise 2 to the power of log2 result
        }

        /// <summary>
        /// Raises 2 to the power of x.
        /// Provides high accuracy for small values of x.
        /// </summary>
        public static Fixed64 Pow2(Fixed64 x)
        {
            if (x.m_rawValue == 0)
                return Fixed64.One;

            bool neg = x.m_rawValue < 0;
            if (neg)
                x = -x;

            if (x == Fixed64.One)
                return neg ? Fixed64.One / Fixed64.Two : Fixed64.Two;

            int integerPart = (int)(x.m_rawValue >> SHIFT_AMOUNT_I);
            long fractionalRaw = x.m_rawValue & MAX_SHIFTED_AMOUNT_UI;

            if (neg)
            {
                if (integerPart >= SHIFT_AMOUNT_I)
                    return Fixed64.MinIncrement;

                Fixed64 result = Pow2Fractional(fractionalRaw, s_pow2NegativeFractionLookup);
                return Fixed64.FromRaw(ShiftRightRounded(result.m_rawValue, integerPart));
            }

            if (integerPart >= 31)
                return Fixed64.MaxValue;

            Fixed64 positiveResult = Pow2Fractional(fractionalRaw, s_pow2PositiveFractionLookup);
            long shifted = positiveResult.m_rawValue << integerPart;

            return Fixed64.FromRaw(shifted);
        }

        /// <summary>
        /// Returns the base-2 logarithm of a specified number.
        /// Provides at least 9 decimals of accuracy.
        /// </summary>
        /// <remarks>
        /// This implementation is based on Clay. S. Turner's fast binary logarithm algorithm 
        /// (C. S. Turner,  "A Fast Binary Logarithm Algorithm", IEEE Signal Processing Mag., pp. 124,140, Sep. 2010.)
        /// </remarks>
        public static Fixed64 Log2(Fixed64 x)
        {
            if (x.m_rawValue <= 0)
                throw new ArgumentOutOfRangeException(nameof(x), "Cannot compute logarithm of non-positive number.");

            long b = 1U << (SHIFT_AMOUNT_I - 1);  // Initial value for binary logarithm
            long rawX = x.m_rawValue;
            int shift = FloorLog2((ulong)rawX) - SHIFT_AMOUNT_I;
            long y = (long)shift << SHIFT_AMOUNT_I;

            if (shift > 0)
                rawX >>= shift;
            else if (shift < 0)
                rawX <<= -shift;

            Fixed64 z = Fixed64.FromRaw(rawX);  // Remaining fraction

            for (int i = 0; i < SHIFT_AMOUNT_I; i++)
            {
                z = FastMul(z, z);
                if (z.m_rawValue >= (ONE_L << 1))
                {
                    z = Fixed64.FromRaw(z.m_rawValue >> 1);
                    y += b;
                }
                b >>= 1;
            }

            return Fixed64.FromRaw(y);
        }

        /// <summary>
        /// Returns the natural logarithm of a specified fixed-point number.
        /// Provides at least 7 decimals of accuracy.
        /// </summary>
        public static Fixed64 Ln(Fixed64 x)
        {
            if (x.m_rawValue <= 0)
                throw new ArgumentOutOfRangeException(nameof(x), "Cannot compute logarithm of non-positive number.");

            return FastMul(Log2(x), Fixed64.Ln2);
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        private static Fixed64 Pow2Fractional(long fractionalRaw, long[] lookup)
        {
            Fixed64 result = Fixed64.One;
            long mask = 1L << (SHIFT_AMOUNT_I - 1);

            for (int i = 0; i < SHIFT_AMOUNT_I; i++)
            {
                if ((fractionalRaw & mask) != 0)
                    result = FastMul(result, Fixed64.FromRaw(lookup[i]));

                mask >>= 1;
            }

            return result;
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        private static long ShiftRightRounded(long value, int shift)
        {
            if (shift == 0)
                return value;

            long half = 1L << (shift - 1);
            return (value + half) >> shift;
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        private static int FloorLog2(ulong value)
        {
            int result = 0;

            if (value >= 1UL << 32)
            {
                value >>= 32;
                result = 32;
            }

            if (value >= 1UL << 16)
            {
                value >>= 16;
                result += 16;
            }

            if (value >= 1UL << 8)
            {
                value >>= 8;
                result += 8;
            }

            if (value >= 1UL << 4)
            {
                value >>= 4;
                result += 4;
            }

            if (value >= 1UL << 2)
            {
                value >>= 2;
                result += 2;
            }

            if (value >= 1UL << 1)
                result++;

            return result;
        }

        /// <summary>
        /// Returns the square root of a specified fixed-point number.
        /// </summary>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static Fixed64 Sqrt(Fixed64 x)
        {
            if (x.m_rawValue < 0)
                throw new ArgumentOutOfRangeException(nameof(x), "Cannot compute square root of a negative number.");

            ulong num = (ulong)x.m_rawValue;
            if (num == 0UL)
                return Fixed64.Zero;

            ulong result = 0UL;
            ulong bit = 1UL << (FloorLog2(num) & ~1);

            // Perform the square root calculation using bitwise shifts
            for (int i = 0; i < 2; ++i)
            {
                // Calculate the top bits of the square root result
                while (bit != 0)
                {
                    if (num >= result + bit)
                    {
                        num -= result + bit;
                        result = (result >> 1) + bit;
                    }
                    else
                    {
                        result >>= 1;
                    }

                    bit >>= 2;
                }

                if (i == 0)
                {
                    // Process it again to get the remaining bits
                    if (num > ((1UL << SHIFT_AMOUNT_I) - 1))
                    {
                        // Handle large remainders by adjusting the result
                        num -= result;
                        num = (num << SHIFT_AMOUNT_I) - (ulong)Fixed64.Half.m_rawValue;
                        result = (result << SHIFT_AMOUNT_I) + (ulong)Fixed64.Half.m_rawValue;
                    }
                    else
                    {
                        num <<= SHIFT_AMOUNT_I;
                        result <<= SHIFT_AMOUNT_I;
                    }

                    bit = 1UL << (SHIFT_AMOUNT_I - 2);
                }
            }

            // Rounding: round up if necessary
            if (num > result && (num - result) > (result >> 1))
                ++result;

            return Fixed64.FromRaw((long)result);
        }

        /// <summary>
        /// Converts a value in radians to degrees.
        /// </summary>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static Fixed64 RadToDeg(Fixed64 rad) => (rad * Fixed64.OneEighty) / Fixed64.Pi;

        /// <summary>
        /// Converts a value in degrees to radians.
        /// </summary>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static Fixed64 DegToRad(Fixed64 deg) => (deg * Fixed64.Pi) / Fixed64.OneEighty;

        /// <summary>
        /// Computes the sine of a given angle in radians using an optimized 
        /// minimax polynomial approximation.
        /// </summary>
        /// <param name="x">The angle in radians.</param>
        /// <returns>The sine of the given angle, in fixed-point format.</returns>
        /// <remarks>
        /// - This function uses a Chebyshev-polynomial-based approximation to ensure high accuracy 
        ///   while maintaining performance in fixed-point arithmetic.
        /// - The coefficients have been carefully tuned to minimize fixed-point truncation errors.
        /// - The error is less than 1 ULP (unit in the last place) at key reference points, 
        ///   ensuring <c>Sin(π/4) = 0.707106781192124</c> exactly within Fixed64 precision.
        /// - The function automatically normalizes input values to the range [-π, π] for stability.
        /// </remarks>
        public static Fixed64 Sin(Fixed64 x)
        {
            // Check for special cases
            if (x == Fixed64.Zero) return Fixed64.Zero;   // sin(0) = 0
            if (x == Fixed64.HalfPi) return Fixed64.One;         // sin(π/2) = 1
            if (x == -Fixed64.HalfPi) return -Fixed64.One;       // sin(-π/2) = -1
            if (x == Fixed64.Pi) return Fixed64.Zero;             // sin(π) = 0
            if (x == -Fixed64.Pi) return Fixed64.Zero;            // sin(-π) = 0
            if (x == Fixed64.TwoPi || x == -Fixed64.TwoPi) return Fixed64.Zero;  // sin(2π) = 0

            // Normalize x to [-π, π]
            x %= Fixed64.TwoPi;
            if (x < -Fixed64.Pi)
                x += Fixed64.TwoPi;
            else if (x > Fixed64.Pi)
                x -= Fixed64.TwoPi;

            bool flip = false;
            if (x < Fixed64.Zero)
            {
                x = -x;
                flip = true;
            }

            if (x > Fixed64.HalfPi)
                x = Fixed64.Pi - x;

            Fixed64 result = SinReduced(x);

            return flip ? -result : result;
        }

        /// <summary>
        /// Computes the cosine of a given angle in radians using a sine-based identity transformation.
        /// </summary>
        /// <param name="x">The angle in radians.</param>
        /// <returns>The cosine of the given angle, in fixed-point format.</returns>
        /// <remarks>
        /// - Instead of directly approximating cosine, this function derives <c>cos(x)</c> using 
        ///   the identity <c>cos(x) = sin(x + π/2)</c>. This ensures maximum accuracy.
        /// - The underlying sine function is computed using a highly optimized minimax polynomial approximation.
        /// - By leveraging this transformation, cosine achieves the same precision guarantees 
        ///   as sine, including <c>Cos(π/4) = 0.707106781192124</c> exactly within Fixed64 precision.
        /// - The function automatically normalizes input values to the range [-π, π] for stability.
        /// </remarks>
        public static Fixed64 Cos(Fixed64 x)
        {
            long xl = x.m_rawValue;
            long rawAngle = xl + (xl > 0 ? -Fixed64.Pi.m_rawValue - Fixed64.HalfPi.m_rawValue : Fixed64.HalfPi.m_rawValue);
            return Sin(Fixed64.FromRaw(rawAngle));
        }

        /// <summary>
        /// Calculates the hypotenuse of a right triangle given sides a and b using the Pythagorean theorem: sqrt(a^2 + b^2).
        /// </summary>
        /// <param name="a">The length of side a.</param>
        /// <param name="b">The length of side b.</param>
        /// <returns>The length of the hypotenuse.</returns>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static Fixed64 GetHypotenuse(Fixed64 a, Fixed64 b) => Sqrt(a * a + b * b);

        /// <summary>
        /// Calculates the cosine value corresponding to a given sine value, assuming the angle is in the first or
        /// second quadrant.
        /// </summary>
        /// <remarks>
        /// This method returns the principal (non-negative) value of the cosine. 
        /// If the input is outside the valid range for sine values, the result may not be meaningful.
        /// </remarks>
        /// <param name="sin">The sine of the angle. Must be in the range [-1, 1].</param>
        /// <returns>The cosine of the angle, computed as the positive square root of (1 - sin²).</returns>
        public static Fixed64 SinToCos(Fixed64 sin) => Sqrt(Fixed64.One - sin * sin);

        /// <summary>
        /// Returns the tangent of x.
        /// </summary>
        /// <remarks>
        /// This function is not well-tested. It may be wildly inaccurate.
        /// </remarks>
        public static Fixed64 Tan(Fixed64 x)
        {
            // Check for special cases
            if (x == Fixed64.Zero) return Fixed64.Zero;
            if (x == Fixed64.PiOver4) return Fixed64.One;
            if (x == -Fixed64.PiOver4) return -Fixed64.One;

            // Normalize x to [-π/2, π/2]
            x %= Fixed64.Pi;
            if (x < -Fixed64.HalfPi)
                x += Fixed64.Pi;
            else if (x > Fixed64.HalfPi)
                x -= Fixed64.Pi;

            bool flip = x < Fixed64.Zero;
            if (flip)
                x = -x;

            Fixed64 sin = SinReduced(x);
            Fixed64 cos = SinReduced(Fixed64.HalfPi - x);
            Fixed64 result = sin / cos;

            return flip ? -result : result;
        }

        /// <summary>
        /// Computes the sine of an angle x (in radians) using a minimax polynomial approximation.
        /// </summary>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        private static Fixed64 SinReduced(Fixed64 x)
        {
            Fixed64 x2 = x * x;
            Fixed64 x4 = x2 * x2;

            return x * (Fixed64.One
                - x2 * Fixed64.SinCoeff3
                + x4 * Fixed64.SinCoeff5
                - x4 * x2 * Fixed64.SinCoeff7);
        }

        /// <summary>
        /// Returns the arc-sine of a fixed-point number x, which is the angle in radians 
        /// whose sine is x, using a combination of a Taylor series expansion and trigonometric identities.
        /// 
        /// For values of x near ±1, the identity asin(x) = π/2 - acos(x) is used for stability.
        /// For values of x near 0, a Taylor series expansion is used.
        /// </summary>
        /// <param name="x">The input value (sine) whose arcsine is to be computed. Should be in the range [-1, 1].</param>
        /// <returns>The arc-sine of x in radians.</returns>
        /// <exception cref="ArithmeticException">Thrown if x is outside the domain [-1, 1].</exception>
        public static Fixed64 Asin(Fixed64 x)
        {
            // Ensure x is within the domain [-1, 1]
            if (x < -Fixed64.One || x > Fixed64.One)
                throw new ArithmeticException("Input out of domain for Asin: " + x);

            // Handle boundary cases for -1 and 1
            if (x == Fixed64.One) return Fixed64.HalfPi;  // asin(1) = π/2
            if (x == -Fixed64.One) return -Fixed64.HalfPi;  // asin(-1) = -π/2

            // Special case handling for asin(0.5) -> π/6 and asin(-0.5) -> -π/6
            if (x == Fixed64.Half) return Fixed64.PiOver6;
            if (x == -Fixed64.Half) return -Fixed64.PiOver6;

            // For values close to 0, use a Padé approximation for better precision
            if (x.Abs() < Fixed64.Half)
            {
                // Padé approximation of asin(x) for |x| < 0.5
                Fixed64 xSquared = x * x;
                Fixed64 numerator = x * (Fixed64.One + (xSquared * (Fixed64.PadeA1 + (xSquared * Fixed64.PadeA2))));
                return numerator;
            }

            return x > Fixed64.Zero
                ? Fixed64.HalfPi - Acos(x)
                : -Fixed64.HalfPi + Acos(-x);
        }

        /// <summary>
        /// Returns the arccosine of the specified number x, calculated using a combination of the atan and sqrt functions.
        /// </summary>
        /// <param name="x">The input value whose arccosine is to be computed. Should be in the range [-1, 1].</param>
        /// <returns>The arccosine of x in radians.</returns>
        /// <exception cref="ArgumentOutOfRangeException">Thrown if x is outside the domain [-1, 1].</exception>
        public static Fixed64 Acos(Fixed64 x)
        {
            if (Abs(x) > Fixed64.One)
                throw new ArithmeticException("Input out of domain for Acos: " + x);

            // For values near 1 or -1, the result is directly known.
            if (x == Fixed64.One) return Fixed64.Zero;      // acos(1) = 0
            if (x == -Fixed64.One) return Fixed64.Pi;       // acos(-1) = π
            if (x == Fixed64.Zero) return Fixed64.HalfPi;  // acos(0) = π/2

            // Compute using the relationship acos(x) = atan(sqrt(1 - x^2) / x) + π/2 when x is negative
            var sqrtTerm = Sqrt(Fixed64.One - x * x);   // sqrt(1 - x^2)
            var atanTerm = Atan(sqrtTerm / x);

            return x < Fixed64.Zero
                    ? atanTerm + Fixed64.Pi   // acos(-x) = atan(...) + π
                    : atanTerm;               // Otherwise, return just atan(sqrt(...))
        }

        /// <summary>
        /// Returns the arctangent of the specified number, using a more accurate approximation for larger values.
        /// This function has at least 7 decimals of accuracy.
        /// </summary>
        public static Fixed64 Atan(Fixed64 z)
        {
            if (z == Fixed64.Zero) return Fixed64.Zero;
            if (z == Fixed64.One) return Fixed64.PiOver4;
            if (z == -Fixed64.One) return -Fixed64.PiOver4;

            bool neg = z < Fixed64.Zero;
            if (neg) z = -z;


            Fixed64 adjustedResult;
            // Adjust series for z > 1 using the identity atan(z) = π/2 - atan(1/z)
            if (z > Fixed64.One)
                adjustedResult = Fixed64.HalfPi - Atan(Fixed64.One / z);
            // For z in (0.5, 1], use a transformation to improve convergence: atan(z) = π/4 - atan((1 - z) / (1 + z))
            else if (z > Fixed64.Half)
            {

                Fixed64 transformedZ = (Fixed64.One - z) / (Fixed64.One + z);
                adjustedResult = Fixed64.PiOver4 - Atan(transformedZ);
            }
            // For z in (0, 0.5], use the standard Taylor series expansion around 0 for better precision on small values.
            else
            {
                Fixed64 zSq = z * z;

                Fixed64 result = z;
                Fixed64 term = z;
                int sign = -1;

                for (int i = 3; i < 15; i += 2)
                {
                    term *= zSq;
                    Fixed64 nextTerm = term / i;
                    if (nextTerm.Abs() < Fixed64.Epsilon)
                        break;

                    result += nextTerm * sign;
                    sign = -sign;
                }

                adjustedResult = result;
            }

            return neg ? -adjustedResult : adjustedResult;
        }

        /// <summary>
        /// Computes the angle whose tangent is the quotient of two specified numbers.
        /// </summary>
        /// <remarks>
        /// Uses a fixed-point arithmetic approximation for the arc tangent function, which is more efficient than using floating-point arithmetic, 
        /// especially on systems where floating-point operations are expensive.
        /// </remarks>
        /// <param name="y">The y-coordinate of the point to which the angle is measured.</param>
        /// <param name="x">The x-coordinate of the point to which the angle is measured.</param>
        /// <returns>An angle, θ, measured in radians, such that -π ≤ θ ≤ π, and tan(θ) = y / x, 
        /// taking into account the quadrants of the inputs to determine the sign of the result.</returns>
        public static Fixed64 Atan2(Fixed64 y, Fixed64 x)
        {
            if (x == Fixed64.Zero)
            {
                if (y > Fixed64.Zero)
                    return Fixed64.HalfPi;
                if (y == Fixed64.Zero)
                    return Fixed64.Zero;
                return -Fixed64.HalfPi;
            }

            Fixed64 atan = Atan(y / x);

            // Adjust based on the quadrant
            if (x < Fixed64.Zero)
            {
                if (y >= Fixed64.Zero)
                {
                    // Second quadrant
                    return atan + Fixed64.Pi;
                }
                else
                {
                    // Third quadrant
                    return atan - Fixed64.Pi;
                }
            }

            // First or fourth quadrant
            return atan;
        }

        #endregion
    }
}
