//=======================================================================
// Fixed64.Operators.cs
//=======================================================================
// MIT License, Copyright (c) 2024–present David Oravsky (mrdav30)
// See LICENSE file in the project root for full license information.
//=======================================================================

using System;
using System.Runtime.CompilerServices;

namespace FixedMathSharp;

public partial struct Fixed64
{
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
    /// Adds an int to a Fixed64, with saturating behavior in case of overflow. 
    /// </summary>
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public static Fixed64 operator +(Fixed64 x, int y) => x + new Fixed64((long)y << FixedMath.SHIFT_AMOUNT_I);

    /// <inheritdoc cref="operator +(Fixed64, int)" />
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public static Fixed64 operator +(int x, Fixed64 y) => y + x;

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
    /// Subtracts an int from a Fixed64, with saturating behavior in case of overflow. 
    /// </summary>
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public static Fixed64 operator -(Fixed64 x, int y) =>
        x - new Fixed64((long)y << FixedMath.SHIFT_AMOUNT_I);

    /// <summary>
    /// Subtracts a Fixed64 from an int, with saturating behavior in case of overflow.
    /// </summary>
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public static Fixed64 operator -(int x, Fixed64 y) =>
         new Fixed64((long)x << FixedMath.SHIFT_AMOUNT_I) - y;

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
    /// True if the shifted or rounded result exceeded 64 bits.
    /// </param>
    /// <returns>
    /// The rounded 64-bit result of ((hi:lo) >> shift).
    /// </returns>
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    private static ulong ShiftRightRoundedToEven(ulong hi, ulong lo, int shift, out bool overflowed)
    {
        // Preconditions: 1 <= shift <= 63

        // Bits above the low 64 result bits must be preserved as saturation state
        // before the lower projection can wrap back into range.
        overflowed = (hi >> shift) != 0UL;

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

        if (shouldRoundUp)
        {
            ulong incremented = result + 1UL;
            overflowed |= incremented < result;
            result = incremented;
        }

        return result;
    }

    /// <summary>
    /// Multiplies a Fixed64 by an integer, with overflow handling.
    /// </summary>
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public static Fixed64 operator *(Fixed64 x, int y) =>
        x * new Fixed64((long)y << FixedMath.SHIFT_AMOUNT_I);

    /// <inheritdoc cref="operator *(Fixed64, int)" />
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public static Fixed64 operator *(int x, Fixed64 y) => y * x;

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
    /// Divides a Fixed64 by an integer, handling division by zero and overflow.
    /// </summary>
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public static Fixed64 operator /(Fixed64 x, int y) =>
         x / new Fixed64((long)y << FixedMath.SHIFT_AMOUNT_I);

    /// <summary>
    /// Divides an integer by a Fixed64, handling division by zero and overflow.
    /// </summary>
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public static Fixed64 operator /(int y, Fixed64 x) =>
         new Fixed64((long)y << FixedMath.SHIFT_AMOUNT_I) / x;

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
    /// Computes the remainder of division of a Fixed64 by an int.
    /// </summary>
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public static Fixed64 operator %(Fixed64 x, int y) => x % new Fixed64((long)y << FixedMath.SHIFT_AMOUNT_I);

    /// <summary>
    /// Computes the remainder of division of an int by a Fixed64.
    /// </summary>
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public static Fixed64 operator %(int x, Fixed64 y) => new Fixed64((long)x << FixedMath.SHIFT_AMOUNT_I) % y;

    /// <summary>
    /// Unary negation operator.
    /// </summary>
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public static Fixed64 operator -(Fixed64 x) =>
        x.m_rawValue == FixedMath.MIN_VALUE_L
            ? new Fixed64(FixedMath.MAX_VALUE_L)
            : new Fixed64(-x.m_rawValue);

    /// <summary>
    /// Increments a Fixed64 number by one, with saturating behavior in case of overflow.
    /// </summary>
    /// <param name="a">The Fixed64 number to increment.</param>
    /// <returns>The incremented Fixed64 number.</returns>
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public static Fixed64 operator ++(Fixed64 a) => a + One;

    /// <summary>
    /// Decrements a Fixed64 number by one, with saturating behavior in case of overflow.
    /// </summary>
    /// <param name="a">The Fixed64 number to decrement.</param>
    /// <returns>The decremented Fixed64 number.</returns>
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public static Fixed64 operator --(Fixed64 a) => a - One;

    /// <summary>
    /// Bitwise left shift operator.
    /// </summary>
    /// <param name="a">Operand to shift.</param>
    /// <param name="shift">Number of bits to shift.</param>
    /// <returns>The shifted value.</returns>
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public static Fixed64 operator <<(Fixed64 a, int shift) => new(a.m_rawValue << shift);

    /// <summary>
    /// Bitwise right shift operator.
    /// </summary>
    /// <param name="a">Operand to shift.</param>
    /// <param name="shift">Number of bits to shift.</param>
    /// <returns>The shifted value.</returns>
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public static Fixed64 operator >>(Fixed64 a, int shift) => new(a.m_rawValue >> shift);

    #endregion
    #region Comparison Operators

    /// <summary>
    /// Determines whether one Fixed64 is greater than another.
    /// </summary>
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public static bool operator >(Fixed64 x, Fixed64 y) => x.m_rawValue > y.m_rawValue;

    /// <summary>
    /// Determines whether a Fixed64 is greater than an integer.
    /// </summary>
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public static bool operator >(Fixed64 x, int y) => x.m_rawValue > (long)y << FixedMath.SHIFT_AMOUNT_I;

    /// <summary>
    /// Determines whether an integer is greater than a Fixed64.
    /// </summary>
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public static bool operator >(int y, Fixed64 x) => (long)y << FixedMath.SHIFT_AMOUNT_I > x.m_rawValue;

    /// <summary>
    /// Determines whether one Fixed64 is less than another.
    /// </summary>
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public static bool operator <(Fixed64 x, Fixed64 y) => x.m_rawValue < y.m_rawValue;

    /// <summary>
    /// Determines whether one Fixed64 is less than an integer.
    /// </summary>
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public static bool operator <(Fixed64 x, int y) => x.m_rawValue < (long)y << FixedMath.SHIFT_AMOUNT_I;

    /// <summary>
    /// Determines whether an integer is less than a Fixed64.
    /// </summary>
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public static bool operator <(int y, Fixed64 x) => (long)y << FixedMath.SHIFT_AMOUNT_I < x.m_rawValue;

    /// <summary>
    /// Determines whether one Fixed64 is greater than or equal to another.
    /// </summary>
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public static bool operator >=(Fixed64 x, Fixed64 y) => x.m_rawValue >= y.m_rawValue;

    /// <summary>
    /// Determines whether Fixed64 is greater than or equal to an integer.
    /// </summary>
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public static bool operator >=(Fixed64 x, int y) => x.m_rawValue >= (long)y << FixedMath.SHIFT_AMOUNT_I;

    /// <summary>
    /// Determines whether an integer is greater than or equal to a Fixed64.
    /// </summary>
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public static bool operator >=(int y, Fixed64 x) => (long)y << FixedMath.SHIFT_AMOUNT_I >= x.m_rawValue;

    /// <summary>
    /// Determines whether one Fixed64 is less than or equal to another.
    /// </summary>
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public static bool operator <=(Fixed64 x, Fixed64 y) => x.m_rawValue <= y.m_rawValue;

    /// <summary>
    /// Determines whether a Fixed64 is less than or equal to an integer.
    /// </summary>
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public static bool operator <=(Fixed64 x, int y) => x.m_rawValue <= (long)y << FixedMath.SHIFT_AMOUNT_I;

    /// <summary>
    /// Determines whether an integer is less than or equal to a Fixed64.
    /// </summary>
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public static bool operator <=(int y, Fixed64 x) => (long)y << FixedMath.SHIFT_AMOUNT_I <= x.m_rawValue;

    /// <summary>
    /// Determines whether two Fixed64 instances are equal.
    /// </summary>
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public static bool operator ==(Fixed64 left, Fixed64 right) => left.Equals(right);

    /// <summary>
    /// Determines whether a Fixed64 instance is equal to an integer.
    /// </summary>
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public static bool operator ==(Fixed64 left, int right) => left.m_rawValue == (long)right << FixedMath.SHIFT_AMOUNT_I;

    /// <summary>
    /// Determines whether an integer is equal to a Fixed64 instance.
    /// </summary>
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public static bool operator ==(int left, Fixed64 right) => (long)left << FixedMath.SHIFT_AMOUNT_I == right.m_rawValue;

    /// <summary>
    /// Determines whether two Fixed64 instances are not equal.
    /// </summary>
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public static bool operator !=(Fixed64 left, Fixed64 right) => !left.Equals(right);

    /// <summary>
    /// Determines whether a Fixed64 instance is not equal to an integer.
    /// </summary>
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public static bool operator !=(Fixed64 left, int right) => left.m_rawValue != (long)right << FixedMath.SHIFT_AMOUNT_I;

    /// <summary>
    /// Determines whether an integer is equal to a Fixed64 instance.
    /// </summary>
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public static bool operator !=(int left, Fixed64 right) => (long)left << FixedMath.SHIFT_AMOUNT_I != right.m_rawValue;

    #endregion
}
