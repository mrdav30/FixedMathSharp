using MemoryPack;
using System;
using System.Collections.Generic;
using System.Globalization;
using System.Runtime.CompilerServices;
using System.Text.Json.Serialization;

namespace FixedMathSharp;

/// <summary>
/// Represents a Q(64-SHIFT_AMOUNT).SHIFT_AMOUNT fixed-point number.
/// Provides high precision for fixed-point arithmetic where SHIFT_AMOUNT bits 
/// are used for the fractional part and (64 - SHIFT_AMOUNT) bits for the integer part.
/// The precision is determined by SHIFT_AMOUNT, which defines the resolution of fractional values.
/// </summary>
[Serializable]
[MemoryPackable]
public partial struct Fixed64 : IEquatable<Fixed64>, IComparable<Fixed64>, IEqualityComparer<Fixed64>
{
    #region Static Readonly Fields

    /// <inheritdoc cref="FixedMath.MAX_VALUE_L" />
    public static readonly Fixed64 MAX_VALUE = new(FixedMath.MAX_VALUE_L);
    /// <inheritdoc cref="FixedMath.MIN_VALUE_L" />
    public static readonly Fixed64 MIN_VALUE = new(FixedMath.MIN_VALUE_L);

    /// <inheritdoc cref="FixedMath.ONE_L" />
    public static readonly Fixed64 One = new(FixedMath.ONE_L);
    /// <summary>
    /// Represents the value 2 shifted left by the number of bits specified by SHIFT_AMOUNT_I.
    /// </summary>
    public static readonly Fixed64 Two = One * 2;
    /// <summary>
    /// Represents the value 3 shifted left by the number of bits specified by SHIFT_AMOUNT_I.
    /// </summary>
    public static readonly Fixed64 Three = One * 3;
    /// <summary>
    /// Represents the value 0.5 shifted left by the number of bits specified by SHIFT_AMOUNT_I.
    /// </summary>
    public static readonly Fixed64 Half = One / 2;
    /// <summary>
    /// Represents the value 0.25 shifted left by the number of bits specified by SHIFT_AMOUNT_I.
    /// </summary>
    public static readonly Fixed64 Quarter = One / 4;
    /// <summary>
    /// Represents the value 0.125 shifted left by the number of bits specified by SHIFT_AMOUNT_I.
    /// </summary>
    public static readonly Fixed64 Eighth = One / 8;
    /// <summary>
    /// Represents the value 0 as a fixed-point number.
    /// </summary>
    public static readonly Fixed64 Zero = new(0);


    /// <inheritdoc cref="FixedMath.MIN_INCREMENT_L" />
    public static readonly Fixed64 MinIncrement = new(FixedMath.MIN_INCREMENT_L);

    /// <inheritdoc cref="FixedMath.DEFAULT_TOLERANCE_L" />
    public static readonly Fixed64 Epsilon = new(FixedMath.DEFAULT_TOLERANCE_L);

    #endregion

    #region Fields

    /// <summary>
    /// The underlying raw long value representing the fixed-point number.
    /// </summary>
    [JsonInclude]
    [MemoryPackInclude]
    public long m_rawValue;

    #endregion

    #region Constructors

    /// <summary>
    /// Internal constructor for a Fixed64 from a raw long value.
    /// </summary>
    /// <param name="rawValue">Raw long value representing the fixed-point number.</param>
    internal Fixed64(long rawValue) => m_rawValue = rawValue;

    /// <summary>
    /// Constructs a Fixed64 from an integer, with the fractional part set to zero.
    /// </summary>
    /// <param name="value">Integer value to convert to </param>
    public Fixed64(int value) : this((long)value << FixedMath.SHIFT_AMOUNT_I) { }

    /// <summary>
    /// Constructs a Fixed64 from a double-precision floating-point value.
    /// </summary>
    /// <remarks>
    /// The value is multiplied by the scaling factor (2^SHIFT_AMOUNT) and 
    /// rounded to the nearest integer to fit into the fixed-point representation.
    /// </remarks>
    /// <param name="value">Double value to convert to </param>
    public Fixed64(double value) : this((long)Math.Round((double)value * FixedMath.ONE_L)) { }

    #endregion

    #region Methods (Instance)

    /// <summary>
    /// Offsets the current Fixed64 by an integer value.
    /// </summary>
    /// <param name="x">The integer value to add.</param>
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public void Offset(int x)
    {
        m_rawValue += (long)x << FixedMath.SHIFT_AMOUNT_I;
    }

    /// <summary>
    /// Returns the raw value as a string.
    /// </summary>
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public string RawToString()
    {
        return m_rawValue.ToString();
    }

    #endregion

    #region Fixed64 Operations

    /// <summary>
    /// Creates a Fixed64 from a fractional number.
    /// </summary>
    /// <param name="numerator">The numerator of the fraction.</param>
    /// <param name="denominator">The denominator of the fraction.</param>
    /// <returns>A Fixed64 representing the fraction.</returns>
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public static Fixed64 Fraction(double numerator, double denominator)
    {
        return new Fixed64(numerator / denominator);
    }

    /// <summary>
    /// x++ (post-increment)
    /// </summary>
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public static Fixed64 PostIncrement(ref Fixed64 a)
    {
        Fixed64 originalValue = a;
        a.m_rawValue += One.m_rawValue;
        return originalValue;
    }

    /// <summary>
    /// x-- (post-decrement)
    /// </summary>
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public static Fixed64 PostDecrement(ref Fixed64 a)
    {
        Fixed64 originalValue = a;
        a.m_rawValue -= One.m_rawValue;
        return originalValue;
    }

    /// <summary>
    /// Counts the leading zeros in a 64-bit unsigned integer.
    /// </summary>
    /// <param name="x">The number to count leading zeros for.</param>
    /// <returns>The number of leading zeros.</returns>
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    internal static int CountLeadingZeroes(ulong x)
    {
        int result = 0;
        while ((x & 0xF000000000000000) == 0) { result += 4; x <<= 4; }
        while ((x & 0x8000000000000000) == 0) { result += 1; x <<= 1; }
        return result;
    }

    /// <summary>
    /// Returns a number indicating the sign of a Fix64 number.
    /// Returns 1 if the value is positive, 0 if is 0, and -1 if it is negative.
    /// </summary>
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public static int Sign(Fixed64 value)
    {
        // Return the sign of the value, optimizing for branchless comparison
        return value.m_rawValue < 0 ? -1 : (value.m_rawValue > 0 ? 1 : 0);
    }

    /// <summary>
    /// Returns true if the number has no decimal part (i.e., if the number is equivalent to an integer) and False otherwise. 
    /// </summary>
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public static bool IsInteger(Fixed64 value)
    {
        return ((ulong)value.m_rawValue & FixedMath.MAX_SHIFTED_AMOUNT_UI) == 0;
    }

    #endregion

    #region Explicit and Implicit Conversions

    /// <summary>
    /// Converts a 64-bit signed integer to a Fixed64 value using explicit casting.
    /// </summary>
    /// <remarks>
    /// The conversion interprets the input value as the integer part of the fixed-point number. 
    /// Use this operator when an explicit conversion from long to Fixed64 is required.
    /// </remarks>
    /// <param name="value">The 64-bit signed integer to convert to a Fixed64 value.</param>
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public static explicit operator Fixed64(long value)
    {
        return FromRaw(value << FixedMath.SHIFT_AMOUNT_I);
    }

    /// <summary>
    /// Converts a Fixed64 value to a 64-bit signed integer by discarding the fractional part.
    /// </summary>
    /// <remarks>
    /// The conversion truncates any fractional component. 
    /// The result represents the integer portion of the Fixed64 value.</remarks>
    /// <param name="value">The Fixed64 value to convert to a long integer.</param>
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public static explicit operator long(Fixed64 value)
    {
        return value.m_rawValue >> FixedMath.SHIFT_AMOUNT_I;
    }

    /// <summary>
    /// Defines an explicit conversion from a 32-bit signed integer to a Fixed64 value.
    /// </summary>
    /// <param name="value">The 32-bit signed integer to convert to a Fixed64 value.</param>
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public static explicit operator Fixed64(int value)
    {
        return new Fixed64(value);
    }

    /// <summary>
    /// Converts a Fixed64 value to a 32-bit signed integer by discarding the fractional part.
    /// </summary>
    /// <remarks>
    /// The conversion truncates any fractional component. 
    /// The result is equivalent to rounding toward zero.
    /// </remarks>
    /// <param name="value">The Fixed64 value to convert to an integer.</param>
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public static explicit operator int(Fixed64 value)
    {
        return (int)(value.m_rawValue >> FixedMath.SHIFT_AMOUNT_I);
    }

    /// <summary>
    /// Defines an explicit conversion from a single-precision floating-point value to a Fixed64 instance.
    /// </summary>
    /// <param name="value">The single-precision floating-point value to convert to Fixed64.</param>
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public static explicit operator Fixed64(float value)
    {
        return new Fixed64((double)value);
    }

    /// <summary>
    /// Converts a Fixed64 value to its equivalent single-precision floating-point representation.
    /// </summary>
    /// <remarks>
    /// This conversion may result in a loss of precision if the Fixed64 value cannot be exactly  represented as a float.
    /// </remarks>
    /// <param name="value">The Fixed64 value to convert to a float.</param>
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public static explicit operator float(Fixed64 value)
    {
        return value.m_rawValue * FixedMath.SCALE_FACTOR_F;
    }

    /// <summary>
    /// Defines an explicit conversion from a double-precision floating-point number to a Fixed64 value.
    /// </summary>
    /// <remarks>
    /// This conversion may result in loss of precision if the double value cannot be exactly represented as a Fixed64.
    /// </remarks>
    /// <param name="value">The double-precision floating-point number to convert to a Fixed64 value.</param>
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public static explicit operator Fixed64(double value)
    {
        return new Fixed64(value);
    }

    /// <summary>
    /// Converts a Fixed64 value to its equivalent double-precision floating-point representation.
    /// </summary>
    /// <remarks>
    /// This conversion may result in a loss of precision if the Fixed64 value cannot be exactly represented as a double.
    /// </remarks>
    /// <param name="value">The Fixed64 value to convert to a double.</param>
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public static explicit operator double(Fixed64 value)
    {
        return value.m_rawValue * FixedMath.SCALE_FACTOR_D;
    }

    /// <summary>
    /// Defines an explicit conversion from a decimal value to a Fixed64 instance.
    /// </summary>
    /// <param name="value">The decimal value to convert to a Fixed64 instance.</param>
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public static explicit operator Fixed64(decimal value)
    {
        return new Fixed64((double)value);
    }

    /// <summary>
    /// Converts a Fixed64 value to its decimal representation.
    /// </summary>
    /// <remarks>
    /// This operator provides an explicit conversion from Fixed64 to decimal, preserving the numeric value as closely as possible. 
    /// Use this conversion when precise decimal arithmetic is required.
    /// </remarks>
    /// <param name="value">The Fixed64 value to convert to decimal.</param>
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public static explicit operator decimal(Fixed64 value)
    {
        return value.m_rawValue * FixedMath.SCALE_FACTOR_M;
    }

    #endregion

    #region Arithmetic Operators

    /// <summary>
    /// Adds two Fixed64 numbers, with saturating behavior in case of overflow.
    /// </summary>
    public static Fixed64 operator +(Fixed64 x, Fixed64 y)
    {
        long xl = x.m_rawValue;
        long yl = y.m_rawValue;
        long sum = xl + yl;
        // Check for overflow, if signs of operands are equal and signs of sum and x are different
        if (((~(xl ^ yl) & (xl ^ sum)) & FixedMath.MIN_VALUE_L) != 0)
            sum = xl > 0 ? FixedMath.MAX_VALUE_L : FixedMath.MIN_VALUE_L;
        return new Fixed64(sum);
    }

    /// <summary>
    /// Adds an int to x 
    /// </summary>
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public static Fixed64 operator +(Fixed64 x, int y)
    {
        return x + new Fixed64((long)y << FixedMath.SHIFT_AMOUNT_I);
    }

    /// <summary>
    /// Adds an Fixed64 to x 
    /// </summary>
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public static Fixed64 operator +(int x, Fixed64 y)
    {
        return y + x;
    }

    /// <summary>
    /// Subtracts one Fixed64 number from another, with saturating behavior in case of overflow.
    /// </summary>
    public static Fixed64 operator -(Fixed64 x, Fixed64 y)
    {
        long xl = x.m_rawValue;
        long yl = y.m_rawValue;
        long diff = xl - yl;
        // Check for overflow, if signs of operands are different and signs of sum and x are different
        if ((((xl ^ yl) & (xl ^ diff)) & FixedMath.MIN_VALUE_L) != 0)
            diff = xl < 0 ? FixedMath.MIN_VALUE_L : FixedMath.MAX_VALUE_L;
        return new Fixed64(diff);
    }

    /// <summary>
    /// Subtracts an int from x 
    /// </summary>
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public static Fixed64 operator -(Fixed64 x, int y)
    {
        return x - new Fixed64((long)y << FixedMath.SHIFT_AMOUNT_I);
    }

    /// <summary>
    /// Subtracts a Fixed64 from x 
    /// </summary>
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public static Fixed64 operator -(int x, Fixed64 y)
    {
        return new Fixed64((long)x << FixedMath.SHIFT_AMOUNT_I) - y;
    }

    /// <summary>
    /// Multiplies two Fixed64 numbers, handling overflow and rounding.
    /// </summary>
    /// <summary>
    /// Multiplies two Fixed64 numbers using full-width 128-bit intermediate precision
    /// and round-half-to-even semantics on the discarded fractional bits.
    /// </summary>
    public static Fixed64 operator *(Fixed64 x, Fixed64 y)
    {
        long xl = x.m_rawValue;
        long yl = y.m_rawValue;

        int shift = FixedMath.SHIFT_AMOUNT_I;

        if (shift <= 0 || shift >= 64)
            throw new InvalidOperationException($"SHIFT_AMOUNT_I must be in the range 1..63, but was {shift}.");

        // Determine sign of the final result.
        bool negative = ((xl ^ yl) < 0);

        // Convert to unsigned magnitudes safely, including long.MinValue.
        ulong ax = AbsToUInt64(xl);
        ulong ay = AbsToUInt64(yl);

        // Compute exact 128-bit unsigned product: (hi << 64) | lo
        Multiply64To128(ax, ay, out ulong hi, out ulong lo);

        // Shift-right with round-half-to-even using the FULL discarded remainder.
        ulong magnitude = ShiftRightRoundedToEven(hi, lo, shift, out bool roundedOverflow);

        // If rounding overflowed the shifted magnitude, carry it into saturation handling.
        if (!negative)
        {
            if (roundedOverflow || magnitude > long.MaxValue)
                return new Fixed64(FixedMath.MAX_VALUE_L);

            return new Fixed64((long)magnitude);
        }
        else
        {
            // For negative results, magnitude may be exactly 2^63, which maps to long.MinValue.
            const ulong minValueMagnitude = 0x8000000000000000UL;

            if (roundedOverflow || magnitude > minValueMagnitude)
                return new Fixed64(FixedMath.MIN_VALUE_L);

            if (magnitude == minValueMagnitude)
                return new Fixed64(FixedMath.MIN_VALUE_L);

            return new Fixed64(-(long)magnitude);
        }
    }

    /// <summary>
    /// Returns the absolute value of a signed 64-bit integer as an unsigned 64-bit magnitude,
    /// safely handling long.MinValue.
    /// </summary>
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    private static ulong AbsToUInt64(long value)
    {
        return value < 0
            ? unchecked((ulong)(~value + 1))
            : (ulong)value;
    }

    /// <summary>
    /// Computes the exact unsigned 128-bit product of two 64-bit unsigned integers.
    /// The result is returned as hi:lo, where product = (hi &lt;&lt; 64) | lo.
    /// </summary>
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    private static void Multiply64To128(ulong a, ulong b, out ulong hi, out ulong lo)
    {
        ulong aLo = (uint)a;
        ulong aHi = a >> 32;
        ulong bLo = (uint)b;
        ulong bHi = b >> 32;

        ulong p0 = aLo * bLo;
        ulong p1 = aLo * bHi;
        ulong p2 = aHi * bLo;
        ulong p3 = aHi * bHi;

        ulong middle = (p0 >> 32) + (uint)p1 + (uint)p2;

        lo = (p0 & 0xFFFFFFFFUL) | (middle << 32);
        hi = p3 + (p1 >> 32) + (p2 >> 32) + (middle >> 32);
    }

    /// <summary>
    /// Shifts the unsigned 128-bit value (hi:lo) right by <paramref name="shift"/> bits,
    /// applying round-half-to-even to the discarded bits.
    /// </summary>
    /// <param name="hi">Upper 64 bits of the 128-bit value.</param>
    /// <param name="lo">Lower 64 bits of the 128-bit value.</param>
    /// <param name="shift">Number of bits to shift right. Must be in the range 1..63.</param>
    /// <param name="overflowed">
    /// True if rounding caused the 64-bit shifted result to overflow.
    /// </param>
    /// <returns>
    /// The rounded 64-bit result of ((hi:lo) >> shift).
    /// </returns>
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    private static ulong ShiftRightRoundedToEven(ulong hi, ulong lo, int shift, out bool overflowed)
    {
        // Preconditions: 1 <= shift <= 63

        // Integer part after shifting right by 'shift':
        // result = ((hi << (64 - shift)) | (lo >> shift))
        ulong result = (hi << (64 - shift)) | (lo >> shift);

        // Discarded remainder bits are the low 'shift' bits of lo.
        ulong remainderMask = (1UL << shift) - 1UL;
        ulong remainder = lo & remainderMask;

        // Halfway value among the discarded bits.
        ulong half = 1UL << (shift - 1);

        // Round-half-to-even:
        // - round up if remainder > half
        // - if exactly half, round so final result is even
        bool shouldRoundUp =
            remainder > half ||
            (remainder == half && (result & 1UL) != 0);

        overflowed = false;

        if (shouldRoundUp)
        {
            ulong incremented = result + 1UL;
            overflowed = incremented < result;
            result = incremented;
        }

        return result;
    }

    /// <summary>
    /// Multiplies a Fixed64 by an integer.
    /// </summary>
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public static Fixed64 operator *(Fixed64 x, int y)
    {
        return x * new Fixed64((long)y << FixedMath.SHIFT_AMOUNT_I);
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
        long xl = x.m_rawValue;
        long yl = y.m_rawValue;

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
                return ((xl ^ yl) & FixedMath.MIN_VALUE_L) == 0 
                    ? new Fixed64(FixedMath.MAX_VALUE_L) 
                    : new Fixed64(FixedMath.MIN_VALUE_L);

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
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public static Fixed64 operator /(Fixed64 x, int y)
    {
        return x / new Fixed64((long)y << FixedMath.SHIFT_AMOUNT_I);
    }

    /// <summary>
    /// Divides an integer by a Fixed64
    /// </summary>
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public static Fixed64 operator /(int y, Fixed64 x)
    {
        return new Fixed64((long)y << FixedMath.SHIFT_AMOUNT_I) / x;
    }

    /// <summary>
    /// Computes the remainder of division of one Fixed64 number by another.
    /// </summary>
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public static Fixed64 operator %(Fixed64 x, Fixed64 y)
    {
        if (x.m_rawValue == FixedMath.MIN_VALUE_L && y.m_rawValue == -1)
            return Zero;
        return new Fixed64(x.m_rawValue % y.m_rawValue);
    }

    /// <summary>
    /// Unary negation operator.
    /// </summary>
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public static Fixed64 operator -(Fixed64 x)
    {
        return x.m_rawValue == FixedMath.MIN_VALUE_L 
            ? new Fixed64(FixedMath.MAX_VALUE_L) 
            : new Fixed64(-x.m_rawValue);
    }

    /// <summary>
    /// Pre-increment operator (++x).
    /// </summary>
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public static Fixed64 operator ++(Fixed64 a)
    {
        a.m_rawValue += One.m_rawValue;
        return a;
    }

    /// <summary>
    /// Pre-decrement operator (--x).
    /// </summary>
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public static Fixed64 operator --(Fixed64 a)
    {
        a.m_rawValue -= One.m_rawValue;
        return a;
    }

    /// <summary>
    /// Bitwise left shift operator.
    /// </summary>
    /// <param name="a">Operand to shift.</param>
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
    /// <param name="a">Operand to shift.</param>
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

    #endregion

    #region Conversion

    /// <summary>
    /// Returns the string representation of this Fixed64 instance.
    /// </summary>
    /// <remarks>
    /// Up to 10 decimal places.
    /// </remarks>
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public override string ToString()
    {
        return ((double)this).ToString(CultureInfo.InvariantCulture);
    }

    /// <summary>
    /// Converts the numeric value of the current Fixed64 object to its equivalent string representation.
    /// </summary>
    /// <param name="format">A format specification that governs how the current Fixed64 object is converted.</param>
    /// <returns>The string representation of the value of the current Fixed64 object.</returns>  
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public string ToString(string format)
    {
        return ((double)this).ToString(format, CultureInfo.InvariantCulture);
    }

    /// <summary>
    /// Parses a string to create a Fixed64 instance.
    /// </summary>
    /// <param name="s">The string representation of the </param>
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
    /// <param name="s">The string representation of the </param>
    /// <param name="result">The parsed Fixed64 value.</param>
    /// <returns>True if parsing succeeded; otherwise, false.</returns>
    public static bool TryParse(string s, out Fixed64 result)
    {
        result = Zero;
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
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public static Fixed64 FromRaw(long rawValue)
    {
        return new Fixed64(rawValue);
    }

    /// <summary>
    /// Converts a Fixed64s RawValue (Int64) into a double
    /// </summary>
    /// <param name="f1"></param>
    /// <returns></returns>
    public static double ToDouble(long f1)
    {
        return f1 * FixedMath.SCALE_FACTOR_D;
    }

    /// <summary>
    /// Converts a Fixed64s RawValue (Int64) into a float
    /// </summary>
    /// <param name="f1"></param>
    /// <returns></returns>
    public static float ToFloat(long f1)
    {
        return f1 * FixedMath.SCALE_FACTOR_F;
    }

    /// <summary>
    /// Converts a Fixed64s RawValue (Int64) into a decimal
    /// </summary>
    /// <param name="f1"></param>
    /// <returns></returns>
    public static decimal ToDecimal(long f1)
    {
        return f1 * FixedMath.SCALE_FACTOR_M;
    }

    #endregion

    #region Equality, HashCode, Comparable Overrides

    /// <summary>
    /// Determines whether this instance equals another object.
    /// </summary>
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public override bool Equals(object? obj)
    {
        return obj is Fixed64 other && Equals(other);
    }

    /// <inheritdoc/>
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public bool Equals(Fixed64 other)
    {
        return m_rawValue == other.m_rawValue;
    }

    /// <inheritdoc/>
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public bool Equals(Fixed64 x, Fixed64 y)
    {
        return x.Equals(y);
    }

    /// <inheritdoc/>
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public override int GetHashCode()
    {
        return m_rawValue.GetHashCode();
    }

    /// <inheritdoc/>
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public int GetHashCode(Fixed64 obj)
    {
        return obj.GetHashCode();
    }

    /// <summary>
    /// Compares this instance to another 
    /// </summary>
    /// <param name="other">The Fixed64 to compare with.</param>
    /// <returns>-1 if less than, 0 if equal, 1 if greater than other.</returns>
    public int CompareTo(Fixed64 other)
    {
        return m_rawValue.CompareTo(other.m_rawValue);
    }

    #endregion
}