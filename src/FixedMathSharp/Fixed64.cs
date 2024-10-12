using System;
using System.Runtime.CompilerServices;

namespace FixedMathSharp
{
    /// <summary>
    /// Represents a Q(64-SHIFT_AMOUNT).SHIFT_AMOUNT fixed-point number.
    /// Provides high precision for fixed-point arithmetic where SHIFT_AMOUNT bits 
    /// are used for the fractional part and (64 - SHIFT_AMOUNT) bits for the integer part.
    /// The precision is determined by SHIFT_AMOUNT, which defines the resolution of fractional values.
    /// </summary>
    [Serializable]
    public partial struct Fixed64 : IEquatable<Fixed64>, IComparable<Fixed64>
    {
        private long m_rawValue;

        /// <summary>
        /// The underlying raw long value representing the fixed-point number.
        /// </summary>
        public long RawValue => m_rawValue;

        /// <summary>
        /// Internal constructor for a Fixed64 from a raw long value.
        /// </summary>
        /// <param name="rawValue">Raw long value representing the fixed-point number.</param>
        internal Fixed64(long rawValue)
        {
            m_rawValue = rawValue;
        }

        /// <summary>
        /// Constructs a Fixed64 from an integer, with the fractional part set to zero.
        /// </summary>
        /// <param name="value">Integer value to convert to Fixed64.</param>
        public Fixed64(int value)
        {
            m_rawValue = (long)value << FixedMath.SHIFT_AMOUNT_I;
        }

        /// <summary>
        /// Constructs a Fixed64 from a double-precision floating-point value.
        /// </summary>
        /// <param name="value">Double value to convert to Fixed64.</param>
        public Fixed64(double value)
        {
            m_rawValue = (long)Math.Round((double)value * FixedMath.ONE_L);
        }

        /// <summary>
        /// Creates a Fixed64 from a fractional number.
        /// </summary>
        /// <param name="numerator">The numerator of the fraction.</param>
        /// <param name="denominator">The denominator of the fraction.</param>
        /// <returns>A Fixed64 representing the fraction.</returns>
        public static Fixed64 Fraction(int numerator, int denominator)
        {
            return (Fixed64)numerator / denominator;
        }

        #region Explicit and Implicit Conversions

        public static explicit operator long(Fixed64 value)
        {
            return value.m_rawValue >> FixedMath.SHIFT_AMOUNT_I;
        }
        public static explicit operator Fixed64(int value)
        {
            return new Fixed64(value);
        }
        public static explicit operator int(Fixed64 value)
        {
            return (int)(value.m_rawValue >> FixedMath.SHIFT_AMOUNT_I);
        }
        public static explicit operator Fixed64(float value)
        {
            return new Fixed64((double)value);
        }
        public static explicit operator float(Fixed64 value)
        {
            return value.m_rawValue * FixedMath.FloatScaleFactor;
        }
        public static explicit operator Fixed64(double value)
        {
            return new Fixed64(value);
        }
        public static explicit operator double(Fixed64 value)
        {
            return value.m_rawValue * FixedMath.DoubleScaleFactor;
        }
        public static explicit operator Fixed64(decimal value)
        {
            return new Fixed64((double)value);
        }
        public static explicit operator decimal(Fixed64 value)
        {
            return value.m_rawValue * FixedMath.DecimalScaleFactor;
        }

        #endregion

        #region Arithmetic Operators

        /// <summary>
        /// Adds two Fixed64 numbers, with saturating behavior in case of overflow.
        /// </summary>
        public static Fixed64 operator +(Fixed64 x, Fixed64 y)
        {
            long xl = x.RawValue;
            long yl = y.RawValue;
            long sum = xl + yl;

            // Check for overflow, if signs of operands are equal and signs of sum and x are different
            if (((~(xl ^ yl) & (xl ^ sum)) & FixedMath.MIN_VALUE_L) != 0)
            {
                sum = xl > 0 ? FixedMath.MAX_VALUE_L : FixedMath.MIN_VALUE_L;
            }
            return new Fixed64(sum);
        }

        /// <summary>
        /// Adds an int to a Fixed64.
        /// </summary>
        public static Fixed64 operator +(Fixed64 x, int y)
        {
            return new Fixed64((x.RawValue * FixedMath.DoubleScaleFactor) + y);
        }

        /// <summary>
        /// Subtracts one Fixed64 number from another, with saturating behavior in case of overflow.
        /// </summary>
        public static Fixed64 operator -(Fixed64 x, Fixed64 y)
        {
            long xl = x.RawValue;
            long yl = y.RawValue;
            long diff = xl - yl;
            // Check for overflow, if signs of operands are different and signs of sum and x are different
            if ((((xl ^ yl) & (xl ^ diff)) & FixedMath.MIN_VALUE_L) != 0)
                diff = xl < 0 ? FixedMath.MIN_VALUE_L : FixedMath.MAX_VALUE_L;
            return new Fixed64(diff);
        }

        /// <summary>
        /// Subtracts an int from a Fixed64.
        /// </summary>
        public static Fixed64 operator -(Fixed64 x, int y)
        {
            return new Fixed64((x.RawValue * FixedMath.DoubleScaleFactor) - y);
        }

        /// <summary>
        /// Multiplies two Fixed64 numbers, handling overflow and rounding.
        /// </summary>
        public static Fixed64 operator *(Fixed64 x, Fixed64 y)
        {
            long xl = x.m_rawValue;
            long yl = y.m_rawValue;

            ulong xlo = (ulong)(xl & FixedMath.MAX_SHIFTED_AMOUNT_UI);
            long xhi = xl >> FixedMath.SHIFT_AMOUNT_I;
            ulong ylo = (ulong)(yl & FixedMath.MAX_SHIFTED_AMOUNT_UI);
            long yhi = yl >> FixedMath.SHIFT_AMOUNT_I;

            // Perform partial products
            ulong lolo = xlo * ylo;
            long lohi = (long)xlo * yhi;
            long hilo = xhi * (long)ylo;
            long hihi = xhi * yhi;

            // Combine the results, including rounding correction
            ulong loResult = lolo >> FixedMath.SHIFT_AMOUNT_I;
            long midResult1 = lohi;
            long midResult2 = hilo;
            long hiResult = hihi << FixedMath.SHIFT_AMOUNT_I;

            // Adjust rounding for the fractional part of the lolo term
            if ((lolo & (1UL << (FixedMath.SHIFT_AMOUNT_I - 1))) != 0)
                loResult++; // Apply rounding up if the dropped bit is 1 (round half-up)

            bool overflow = false;
            long sum = FixedMath.AddOverflowHelper((long)loResult, midResult1, ref overflow);
            sum = FixedMath.AddOverflowHelper(sum, midResult2, ref overflow);
            sum = FixedMath.AddOverflowHelper(sum, hiResult, ref overflow);

            bool opSignsEqual = ((xl ^ yl) & FixedMath.MIN_VALUE_L) == 0;

            // Handle positive and negative overflow
            if (opSignsEqual)
            {
                if (sum < 0 || (overflow && xl > 0))
                    return FixedMath.MaxValue;
            }
            else
            {
                if (sum > 0)
                    return FixedMath.MinValue;
            }

            // Check for overflow in the higher bits
            long topCarry = hihi >> FixedMath.SHIFT_AMOUNT_I;
            if (topCarry != 0 && topCarry != -1 /*&& xl != -17 && yl != -17*/)
                return opSignsEqual ? FixedMath.MaxValue : FixedMath.MinValue;

            // Check for negative overflow when signs differ
            if (!opSignsEqual)
            {
                long posOp, negOp;
                if (xl > yl)
                {
                    posOp = xl;
                    negOp = yl;
                }
                else
                {
                    posOp = yl;
                    negOp = xl;
                }

                if (sum > negOp && negOp < -FixedMath.ONE_L && posOp > FixedMath.ONE_L)
                    return FixedMath.MinValue;
            }

            return new Fixed64(sum);
        }

        /// <summary>
        /// Multiplies a Fixed64 by an integer.
        /// </summary>
        public static Fixed64 operator *(Fixed64 x, int y)
        {
            return new Fixed64((x.RawValue * FixedMath.DoubleScaleFactor) * y);
        }

        /// <summary>
        /// Multiplies an integer by a Fixed64.
        /// </summary>
        public static Fixed64 operator *(int x, Fixed64 y)
        {
            return y * x;
        }

        /// <summary>
        /// Divides one Fixed64 number by another, handling division by zero and overflow.
        /// </summary>
        public static Fixed64 operator /(Fixed64 x, Fixed64 y)
        {
            long xl = x.RawValue;
            long yl = y.RawValue;

            if (yl == 0)
                throw new DivideByZeroException($"Attempted to divide {x} by zero.");

            ulong remainder = (ulong)(xl < 0 ? -xl : xl);
            ulong divider = (ulong)(yl < 0 ? -yl : yl);
            ulong quotient = 0UL;
            int bitPos = FixedMath.SHIFT_AMOUNT_I + 1;

            // If the divider is divisible by 2^n, take advantage of it.
            while ((divider & 0xF) == 0 && bitPos >= 4)
            {
                divider >>= 4;
                bitPos -= 4;
            }

            while (remainder != 0 && bitPos >= 0)
            {
                int shift = CountLeadingZeroes(remainder);
                if (shift > bitPos)
                    shift = bitPos;

                remainder <<= shift;
                bitPos -= shift;

                ulong div = remainder / divider;
                remainder %= divider;
                quotient += div << bitPos;

                // Detect overflow
                if ((div & ~(0xFFFFFFFFFFFFFFFF >> bitPos)) != 0)
                    return ((xl ^ yl) & FixedMath.MIN_VALUE_L) == 0 ? FixedMath.MaxValue : FixedMath.MinValue;

                remainder <<= 1;
                --bitPos;
            }

            // Rounding logic: "Round half to even" or "Banker's rounding"
            if ((quotient & 0x1) != 0)
                quotient += 1;

            long result = (long)(quotient >> 1);
            if (((xl ^ yl) & FixedMath.MIN_VALUE_L) != 0)
                result = -result;

            return new Fixed64(result);
        }

        /// <summary>
        /// Divides a Fixed64 by an integer.
        /// </summary>
        public static Fixed64 operator /(Fixed64 x, int y)
        {
            return new Fixed64((x.RawValue * FixedMath.DoubleScaleFactor) / y);
        }

        /// <summary>
        /// Computes the remainder of division of one Fixed64 number by another.
        /// </summary>
        public static Fixed64 operator %(Fixed64 x, Fixed64 y)
        {
            if (x.m_rawValue == FixedMath.MIN_VALUE_L && y.m_rawValue == -1)
            {
                return FixedMath.Zero;
            }
            return new Fixed64(x.m_rawValue % y.m_rawValue);
        }

        /// <summary>
        /// Unary negation operator.
        /// </summary>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static Fixed64 operator -(Fixed64 x)
        {
            return x.m_rawValue == FixedMath.MIN_VALUE_L ? FixedMath.MaxValue : new Fixed64(-x.m_rawValue);
        }

        /// <summary>
        /// Pre-increment operator (++x).
        /// </summary>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static Fixed64 operator ++(Fixed64 a)
        {
            a.m_rawValue += FixedMath.One.RawValue;
            return a;
        }

        /// <summary>
        /// Pre-decrement operator (--x).
        /// </summary>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static Fixed64 operator --(Fixed64 a)
        {
            a.m_rawValue -= FixedMath.One.RawValue;
            return a;
        }

        /// <summary>
        /// Bitwise left shift operator.
        /// </summary>
        /// <param name="x">Operand to shift.</param>
        /// <param name="shift">Number of bits to shift.</param>
        /// <returns>The shifted value.</returns>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static Fixed64 operator <<(Fixed64 a, int shift)
        {
            return new Fixed64(a.m_rawValue << shift);
        }

        /// <summary>
        /// Bitwise right shift operator.
        /// </summary>
        /// <param name="x">Operand to shift.</param>
        /// <param name="shift">Number of bits to shift.</param>
        /// <returns>The shifted value.</returns>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static Fixed64 operator >>(Fixed64 a, int shift)
        {
            return new Fixed64(a.m_rawValue >> shift);
        }

        #endregion

        #region Comparison Operators

        /// <summary>
        /// Determines whether one Fixed64 is greater than another.
        /// </summary>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static bool operator >(Fixed64 x, Fixed64 y)
        {
            return x.m_rawValue > y.m_rawValue;
        }

        /// <summary>
        /// Determines whether one Fixed64 is less than another.
        /// </summary>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static bool operator <(Fixed64 x, Fixed64 y)
        {
            return x.m_rawValue < y.m_rawValue;
        }

        /// <summary>
        /// Determines whether one Fixed64 is greater than or equal to another.
        /// </summary>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static bool operator >=(Fixed64 x, Fixed64 y)
        {
            return x.m_rawValue >= y.m_rawValue;
        }

        /// <summary>
        /// Determines whether one Fixed64 is less than or equal to another.
        /// </summary>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static bool operator <=(Fixed64 x, Fixed64 y)
        {
            return x.m_rawValue <= y.m_rawValue;
        }

        /// <summary>
        /// Determines whether two Fixed64 instances are equal.
        /// </summary>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static bool operator ==(Fixed64 left, Fixed64 right)
        {
            return left.Equals(right);
        }

        /// <summary>
        /// Determines whether two Fixed64 instances are not equal.
        /// </summary>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static bool operator !=(Fixed64 left, Fixed64 right)
        {
            return !left.Equals(right);
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public override bool Equals(object obj)
        {
            return obj is Fixed64 other && Equals(other);
        }

        /// <summary>
        /// Determines whether this instance equals another Fixed64.
        /// </summary>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public bool Equals(Fixed64 other)
        {
            return m_rawValue == other.m_rawValue;
        }

        /// <summary>
        /// Returns the hash code for this Fixed64 instance.
        /// </summary>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public override int GetHashCode()
        {
            return m_rawValue.GetHashCode();
        }

        #endregion

        #region Utility Methods

        /// <summary>
        /// x++ (post-increment)
        /// </summary>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static Fixed64 PostIncrement(ref Fixed64 a)
        {
            Fixed64 originalValue = a;
            a.m_rawValue += FixedMath.One.RawValue;
            return originalValue;
        }

        /// <summary>
        /// x-- (post-decrement)
        /// </summary>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static Fixed64 PostDecrement(ref Fixed64 a)
        {
            Fixed64 originalValue = a;
            a.m_rawValue -= FixedMath.One.RawValue;
            return originalValue;
        }

        /// <summary>
        /// Counts the leading zeros in a 64-bit unsigned integer.
        /// </summary>
        /// <param name="x">The number to count leading zeros for.</param>
        /// <returns>The number of leading zeros.</returns>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        static int CountLeadingZeroes(ulong x)
        {
            int result = 0;
            while ((x & 0xF000000000000000) == 0) { result += 4; x <<= 4; }
            while ((x & 0x8000000000000000) == 0) { result += 1; x <<= 1; }
            return result;
        }

        /// <summary>
        /// Determines whether this instance equals another object.
        /// </summary>

        /// <summary>
        /// Compares this instance to another Fixed64.
        /// </summary>
        /// <param name="other">The Fixed64 to compare with.</param>
        /// <returns>-1 if less than, 0 if equal, 1 if greater than other.</returns>
        public int CompareTo(Fixed64 other)
        {
            return m_rawValue.CompareTo(other.m_rawValue);
        }

        /// <summary>
        /// Offsets the current Fixed64 by an integer value.
        /// </summary>
        /// <param name="x">The integer value to add.</param>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public void Offset(int x)
        {
            m_rawValue += (long)x << FixedMath.SHIFT_AMOUNT_I;
        }

        #endregion

        #region Conversion and Parsing

        /// <summary>
        /// Returns the string representation of this Fixed64 instance.
        /// </summary>
        /// <remarks>
        /// Up to 10 decimal places.
        /// </remarks>
        public override string ToString()
        {
            return ((decimal)this).ToString("0.##########");
        }

        /// <summary>
        /// Returns the raw value as a string.
        /// </summary>
        public string ToRawString()
        {
            return RawValue.ToString();
        }

        /// <summary>
        /// Parses a string to create a Fixed64 instance.
        /// </summary>
        /// <param name="s">The string representation of the Fixed64.</param>
        /// <returns>The parsed Fixed64 value.</returns>
        public static Fixed64 Parse(string s)
        {
            if (string.IsNullOrEmpty(s)) throw new ArgumentNullException(nameof(s));

            // Check if the value is negative
            bool isNegative = false;
            if (s[0] == '-')
            {
                isNegative = true;
                s = s.Substring(1);
            }

            if (!long.TryParse(s, out long rawValue))
                throw new FormatException($"Invalid format: {s}");

            // If the value was negative, negate the result
            if (isNegative)
                rawValue = -rawValue;

            return FromRaw(rawValue);
        }

        /// <summary>
        /// Tries to parse a string to create a Fixed64 instance.
        /// </summary>
        /// <param name="s">The string representation of the Fixed64.</param>
        /// <param name="result">The parsed Fixed64 value.</param>
        /// <returns>True if parsing succeeded; otherwise, false.</returns>
        public static bool TryParse(string s, out Fixed64 result)
        {
            result = FixedMath.Zero;
            if (string.IsNullOrEmpty(s)) return false;

            // Check if the value is negative
            bool isNegative = false;
            if (s[0] == '-')
            {
                isNegative = true;
                s = s.Substring(1);
            }

            if (!long.TryParse(s, out long rawValue)) return false;

            // If the value was negative, negate the result
            if (isNegative)
                rawValue = -rawValue;

            result = FromRaw(rawValue);
            return true;
        }

        /// <summary>
        /// Creates a Fixed64 from a raw long value.
        /// </summary>
        /// <param name="rawValue">The raw long value.</param>
        /// <returns>A Fixed64 representing the raw value.</returns>
        public static Fixed64 FromRaw(long rawValue)
        {
            return new Fixed64(rawValue);
        }

        #endregion
    }
}