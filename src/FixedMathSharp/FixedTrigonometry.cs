using System;
using System.Runtime.CompilerServices;
using System.Threading;

namespace FixedMathSharp
{
    public static partial class FixedMath
    {
        #region Fields and Constants

        public static readonly int[] Pow10Lookup = {
            1,           // 10^0 = 1
            10,          // 10^1 = 10
            100,         // 10^2 = 100
            1000,        // 10^3 = 1000
            10000,       // 10^4 = 10000
            100000,      // 10^5 = 100000
            1000000,     // 10^6 = 1000000
            10000000,    // 10^7 = 1000000
            100000000,   // 10^8 = 1000000
            1000000000,  // 10^9 = 1000000
        };

        // Trigonometric and logarithmic constants
        internal const double PI_D = 3.14159265358979323846;
        public static readonly Fixed64 PI = new Fixed64(PI_D);
        public static readonly Fixed64 TwoPI = PI * 2;
        public static readonly Fixed64 PiOver2 = PI / 2;
        public static readonly Fixed64 PiOver3 = PI / 3;
        public static readonly Fixed64 PiOver4 = PI / 4;
        public static readonly Fixed64 PiOver6 = PI / 6;
        public static readonly Fixed64 Ln2 = new Fixed64(0.6931471805599453);  // Natural logarithm of 2

        public static readonly Fixed64 Log2Max = new Fixed64(63L * ONE_L);
        public static readonly Fixed64 Log2Min = new Fixed64(-64L * ONE_L);

        internal const double DEG2RAD_D = 0.01745329251994329576;  // π / 180
        public static readonly Fixed64 Deg2Rad = new Fixed64(DEG2RAD_D);  // Degrees to radians conversion factor
        internal const double RAD2DEG_D = 57.2957795130823208767;  // 180 / π
        public static readonly Fixed64 Rad2Deg = new Fixed64(RAD2DEG_D);  // Radians to degrees conversion factor

        // Asin Padé approximations
        private static readonly Fixed64 PadeA1 = new Fixed64(0.183320102);
        private static readonly Fixed64 PadeA2 = new Fixed64(0.0218804099);

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

            // Handle negative expFixed64.Onents by using the reciprocal
            bool neg = x.m_rawValue < 0;
            if (neg)
                x = -x;

            if (x == Fixed64.One)
                return neg ? Fixed64.One / Fixed64.Two : Fixed64.Two;

            if (x >= Log2Max)
                return neg ? Fixed64.One / Fixed64.MaxValue : Fixed64.MaxValue;

            if (x <= Log2Min)
                return neg ? Fixed64.MaxValue : Fixed64.Zero;

            /* 
             * Taylor series expansion for exp(x)
             * From term n, we get term n+1 by multiplying with x/n.
             * When the sum term drops to Fixed64.Zero, we can stop summing.
             */
            int integerPart = (int)x.Floor();
            x = Fixed64.FromRaw(x.m_rawValue & MAX_SHIFTED_AMOUNT_UI);  // Fractional part

            var result = Fixed64.One;
            var term = Fixed64.One;
            int i = 1;
            while (term.m_rawValue != 0)
            {
                term = FastMul(FastMul(x, term), Ln2) / (Fixed64)i;
                result += term;
                i++;
            }

            result = Fixed64.FromRaw(result.m_rawValue << integerPart);
            if (neg)
                result = Fixed64.One / result;

            return result;
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
                throw new ArgumentOutOfRangeException("Cannot compute logarithm of non-positive number.");

            long b = 1U << (SHIFT_AMOUNT_I - 1);  // Initial value for binary logarithm
            long y = 0;  // Result accumulator
            long rawX = x.m_rawValue;

            // Adjust rawX to the correct range [1, 2)
            while (rawX < ONE_L)
            {
                rawX <<= 1;
                y -= ONE_L;
            }

            while (rawX >= (ONE_L << 1))
            {
                rawX >>= 1;
                y += ONE_L;
            }

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
                throw new ArgumentOutOfRangeException("Cannot compute logarithm of non-positive number.");

            return FastMul(Log2(x), Ln2).Round();
        }

        /// <summary>
        /// Returns the square root of a specified fixed-point number.
        /// </summary>
        public static Fixed64 Sqrt(Fixed64 x)
        {
            if (x.m_rawValue < 0)
                throw new ArgumentOutOfRangeException("Cannot compute square root of a negative number.");

            ulong num = (ulong)x.m_rawValue;
            ulong result = 0UL;
            ulong bit = 1UL << (sizeof(long) * 8) - 2; // second-to-top bit of a 64-bit integer

            // Adjust the bit position to a suitable starting point
            while (bit > num)
                bit >>= 2;

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
        /// <remarks>
        /// Uses double precision to avoid precision loss
        /// </remarks>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static Fixed64 RadToDeg(Fixed64 rad)
        {
            return new Fixed64((double)rad * RAD2DEG_D);
        }

        /// <summary>
        /// Converts a value in degrees to radians.
        /// </summary>
        /// <remarks>
        /// Uses double precision to avoid precision loss
        /// </remarks>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static Fixed64 DegToRad(Fixed64 deg)
        {
            return new Fixed64((double)deg * DEG2RAD_D);
        }

        /// <summary>
        /// Returns the sine of a specified angle in radians.
        /// </summary>
        /// <remarks>
        /// The relative error is less than 1E-10 for x in [-2PI, 2PI], and less than 1E-7 in the worst case.
        /// </remarks>
        public static Fixed64 Sin(Fixed64 x)
        {
            // Check for special cases
            if (x == Fixed64.Zero) return Fixed64.Zero;
            if (x == PiOver2) return Fixed64.One;
            if (x == -PiOver2) return -Fixed64.One;

            // Ensure x is in the range [-2π, 2π]
            x %= TwoPI;
            if (x < -PI)
                x += TwoPI;
            else if (x > PI)
                x -= TwoPI;

            bool flip = false;
            if (x < Fixed64.Zero)
            {
                x = -x;
                flip = true;
            }

            if (x > PiOver2)
                x = PI - x;

            // Use Taylor series approximation
            Fixed64 result = x;
            Fixed64 term = x;
            Fixed64 x2 = x * x;
            int sign = -1;

            for (int i = 3; i < 15; i += 2)
            {
                term *= x2 / (i * (i - 1));
                if (term.Abs() < Fixed64.Epsilon)
                    break;

                result += term * sign;
                sign = -sign;
            }

            return flip ? -result : result;
        }

        /// <summary>
        /// Returns the cosine of x.
        /// The relative error is less than 1E-10 for x in [-2PI, 2PI], and less than 1E-7 in the worst case.
        /// </summary>
        public static Fixed64 Cos(Fixed64 x)
        {
            long xl = x.m_rawValue;
            long rawAngle = xl + (xl > 0 ? -PI.m_rawValue - PiOver2.m_rawValue : PiOver2.m_rawValue);
            return Sin(Fixed64.FromRaw(rawAngle));
        }

        public static Fixed64 SinToCos(Fixed64 sin)
        {
            return Sqrt(Fixed64.One - sin * sin);
        }

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
            if (x == PiOver4) return Fixed64.One;
            if (x == -PiOver4) return -Fixed64.One;

            // Normalize x to [-π/2, π/2]
            x %= PI;
            if (x < -PiOver2)
                x += PI;
            else if (x > PiOver2)
                x -= PI;

            // Use continued fraction to approximate tan(x)
            Fixed64 x2 = x * x;
            Fixed64 numerator = x;
            Fixed64 denominator = Fixed64.One;

            // Iterate over the continued fraction terms
            Fixed64 prevDenominator = denominator;
            int start = x.Abs() > PiOver6 ? 19 : 13;
            for (int i = start; i >= 1; i -= 2)
            {
                denominator = (Fixed64)i - (x2 / denominator);
                if ((denominator - prevDenominator).Abs() < Fixed64.Precision)
                    break;
                prevDenominator = denominator;
            }

            return numerator / denominator;
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
            if (x == Fixed64.One) return PiOver2;  // asin(1) = π/2
            if (x == -Fixed64.One) return -PiOver2;  // asin(-1) = -π/2

            // Special case handling for asin(0.5) -> π/6 and asin(-0.5) -> -π/6
            if (x == Fixed64.Half) return PiOver6;
            if (x == -Fixed64.Half) return -PiOver6;

            // For values close to 0, use a Padé approximation for better precision
            if (x.Abs() < Fixed64.Half)
            {
                // Padé approximation of asin(x) for |x| < 0.5
                Fixed64 xSquared = x * x;
                Fixed64 numerator = x * (Fixed64.One + (xSquared * (PadeA1 + (xSquared * PadeA2))));
                return numerator;
            }

            // For values closer to ±1, use the identity: asin(x) = π/2 - acos(x) for stability
            return x > Fixed64.Zero
                ? PiOver2 - Acos(x)
                : -PiOver2 + Acos(-x);
        }

        /// <summary>
        /// Returns the arccosine of the specified number x, calculated using a combination of the atan and sqrt functions.
        /// </summary>
        /// <param name="x">The input value whose arccosine is to be computed. Should be in the range [-1, 1].</param>
        /// <returns>The arccosine of x in radians.</returns>
        /// <exception cref="ArgumentOutOfRangeException">Thrown if x is outside the domain [-1, 1].</exception>
        public static Fixed64 Acos(Fixed64 x)
        {
            if (x < -Fixed64.One || x > Fixed64.One)
                throw new ArgumentOutOfRangeException(nameof(x), "Input out of domain for Acos: " + x);

            // For values near 1 or -1, the result is directly known.
            if (x == Fixed64.One) return Fixed64.Zero;      // acos(1) = 0
            if (x == -Fixed64.One) return PI;       // acos(-1) = π
            if (x == Fixed64.Zero) return PiOver2;  // acos(0) = π/2

            // Compute using the relationship acos(x) = atan(sqrt(1 - x^2) / x) + π/2 when x is negative
            var sqrtTerm = Sqrt(Fixed64.One - x * x);   // sqrt(1 - x^2)
            var atanTerm = Atan(sqrtTerm / x);

            return x < Fixed64.Zero
                    ? atanTerm + PI   // acos(-x) = atan(...) + π
                    : atanTerm;               // Otherwise, return just atan(sqrt(...))
        }

        /// <summary>
        /// Returns the arctangent of the specified number, using a more accurate approximation for larger values.
        /// This function has at least 7 decimals of accuracy.
        /// </summary>
        public static Fixed64 Atan(Fixed64 z)
        {
            if (z == Fixed64.Zero) return Fixed64.Zero;
            if (z == Fixed64.One) return PiOver4;
            if (z == -Fixed64.One) return -PiOver4;

            bool neg = z < Fixed64.Zero;
            if (neg) z = -z;

            // Adjust series for z > 0.5 using the identity.
            Fixed64 adjustedResult;
            if (z > Fixed64.Half)
            {
                // Apply the identity: atan(z) = π/4 - atan((1 - z) / (1 + z))
                Fixed64 transformedZ = (Fixed64.One - z) / (Fixed64.One + z);
                adjustedResult = PiOver4 - Atan(transformedZ);
            }
            else
            {
                // Use extended Taylor series directly for better precision on small z.
                Fixed64 zSq = z * z;

                Fixed64 result = z;
                Fixed64 term = z;
                int sign = -1;

                for (int i = 3; i < 15; i += 2)
                {
                    term *= zSq;
                    Fixed64 nextTerm = term / i;
                    if (nextTerm.Abs() < Fixed64.Precision)
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
                    return PiOver2;
                if (y == Fixed64.Zero)
                    return Fixed64.Zero;
                return -PiOver2;
            }

            Fixed64 atan = Atan(y / x);

            // Adjust based on the quadrant
            if (x < Fixed64.Zero)
            {
                if (y >= Fixed64.Zero)
                {
                    // Second quadrant
                    return atan + PI;
                }
                else
                {
                    // Third quadrant
                    return atan - PI;
                }
            }

            // First or fourth quadrant
            return atan;
        }

        #endregion
    }
}