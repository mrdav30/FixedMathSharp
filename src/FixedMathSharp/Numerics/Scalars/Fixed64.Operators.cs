//=======================================================================
// Fixed64.Operators.cs
//=======================================================================
// MIT License, Copyright (c) 2024-present David Oravsky (mrdav30)
// See LICENSE file in the project root for full license information.
//=======================================================================

using System;
using System.Runtime.CompilerServices;

namespace FixedMathSharp;

/// <content>
/// Defines arithmetic, comparison, and conversion operators for <see cref="Fixed64"/>.
/// </content>
public partial struct Fixed64
{
    #region Arithmetic Operators

    /// <summary>
    /// Adds two Fixed64 numbers, with saturating behavior in case of overflow.
    /// </summary>
    /// <param name="x">The first addend.</param>
    /// <param name="y">The second addend.</param>
    /// <returns>The sum, saturated at the representable endpoints.</returns>
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public static Fixed64 operator +(Fixed64 x, Fixed64 y)
    {
        long xl = x.m_rawValue;
        long yl = y.m_rawValue;
        long sum = unchecked(xl + yl);
        if (!IsAddOrSubtractResultExact(xl, sum, ~(xl ^ yl)))
            sum = xl < 0 ? FixedMath.MIN_VALUE_L : FixedMath.MAX_VALUE_L;
        return new Fixed64(sum);
    }

    /// <summary>
    /// Adds an int to a Fixed64, with saturating behavior in case of overflow. 
    /// </summary>
    /// <param name="x">The first addend.</param>
    /// <param name="y">The second addend.</param>
    /// <returns>The sum, saturated at the representable endpoints.</returns>
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public static Fixed64 operator +(Fixed64 x, int y) => x + new Fixed64((long)y << FixedMath.SHIFT_AMOUNT_I);

    /// <summary>Adds an integer to a fixed-point value.</summary>
    /// <param name="x">The first addend.</param>
    /// <param name="y">The second addend.</param>
    /// <returns>The sum, saturated at the representable endpoints.</returns>
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public static Fixed64 operator +(int x, Fixed64 y) => y + x;

    /// <summary>
    /// Subtracts one Fixed64 number from another, with saturating behavior in case of overflow.
    /// </summary>
    /// <param name="x">The minuend.</param>
    /// <param name="y">The subtrahend.</param>
    /// <returns>The difference, saturated at the representable endpoints.</returns>
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public static Fixed64 operator -(Fixed64 x, Fixed64 y)
    {
        long xl = x.m_rawValue;
        long yl = y.m_rawValue;
        long diff = unchecked(xl - yl);
        if (!IsAddOrSubtractResultExact(xl, diff, xl ^ yl))
            diff = xl < 0 ? FixedMath.MIN_VALUE_L : FixedMath.MAX_VALUE_L;
        return new Fixed64(diff);
    }

    /// <summary>
    /// Subtracts an int from a Fixed64, with saturating behavior in case of overflow. 
    /// </summary>
    /// <param name="x">The minuend.</param>
    /// <param name="y">The subtrahend.</param>
    /// <returns>The difference, saturated at the representable endpoints.</returns>
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public static Fixed64 operator -(Fixed64 x, int y) =>
        x - new Fixed64((long)y << FixedMath.SHIFT_AMOUNT_I);

    /// <summary>
    /// Subtracts a Fixed64 from an int, with saturating behavior in case of overflow.
    /// </summary>
    /// <param name="x">The minuend.</param>
    /// <param name="y">The subtrahend.</param>
    /// <returns>The difference, saturated at the representable endpoints.</returns>
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public static Fixed64 operator -(int x, Fixed64 y) =>
         new Fixed64((long)x << FixedMath.SHIFT_AMOUNT_I) - y;

    /// <summary>
    /// Multiplies two Fixed64 numbers using full-width 128-bit intermediate precision
    /// and round-half-to-even semantics on the discarded fractional bits.
    /// </summary>
    /// <param name="x">The first factor.</param>
    /// <param name="y">The second factor.</param>
    /// <returns>The product rounded to the nearest representable value with ties to even and saturated at the representable endpoints.</returns>
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
    /// Multiplies a Fixed64 by an integer, with overflow handling.
    /// </summary>
    /// <param name="x">The first factor.</param>
    /// <param name="y">The second factor.</param>
    /// <returns>The product rounded to the nearest representable value with ties to even and saturated at the representable endpoints.</returns>
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public static Fixed64 operator *(Fixed64 x, int y) =>
        x * new Fixed64((long)y << FixedMath.SHIFT_AMOUNT_I);

    /// <summary>Multiplies an integer by a fixed-point value.</summary>
    /// <param name="x">The first factor.</param>
    /// <param name="y">The second factor.</param>
    /// <returns>The product rounded to the nearest representable value with ties to even and saturated at the representable endpoints.</returns>
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public static Fixed64 operator *(int x, Fixed64 y) => y * x;

    /// <summary>
    /// Divides one Fixed64 number by another, handling division by zero and overflow.
    /// </summary>
    /// <param name="x">The dividend.</param>
    /// <param name="y">The divisor.</param>
    /// <returns>The quotient rounded to the nearest representable value with ties to even and saturated at the representable endpoints.</returns>
    /// <exception cref="DivideByZeroException"><paramref name="y"/> is zero.</exception>
    public static Fixed64 operator /(Fixed64 x, Fixed64 y)
    {
        long xl = x.m_rawValue;
        long yl = y.m_rawValue;

        if (yl == 0)
            throw new DivideByZeroException($"Attempted to divide {x} by zero.");

        return DivideMagnitude(
            AbsToUInt64(xl),
            AbsToUInt64(yl),
            (xl ^ yl) < 0);
    }

    /// <summary>
    /// Divides a Fixed64 by an integer, handling division by zero and overflow.
    /// </summary>
    /// <param name="x">The dividend.</param>
    /// <param name="y">The divisor.</param>
    /// <returns>The quotient rounded to the nearest representable value with ties to even and saturated at the representable endpoints.</returns>
    /// <exception cref="DivideByZeroException"><paramref name="y"/> is zero.</exception>
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public static Fixed64 operator /(Fixed64 x, int y) =>
         x / new Fixed64((long)y << FixedMath.SHIFT_AMOUNT_I);

    /// <summary>
    /// Divides an integer by a Fixed64, handling division by zero and overflow.
    /// </summary>
    /// <param name="y">The dividend.</param>
    /// <param name="x">The divisor.</param>
    /// <returns>The quotient rounded to the nearest representable value with ties to even and saturated at the representable endpoints.</returns>
    /// <exception cref="DivideByZeroException"><paramref name="x"/> is zero.</exception>
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public static Fixed64 operator /(int y, Fixed64 x) =>
         new Fixed64((long)y << FixedMath.SHIFT_AMOUNT_I) / x;

    /// <summary>
    /// Computes the remainder of division of one Fixed64 number by another.
    /// </summary>
    /// <param name="x">The dividend.</param>
    /// <param name="y">The divisor.</param>
    /// <returns>The signed remainder, with the sign of the dividend when nonzero.</returns>
    /// <exception cref="DivideByZeroException"><paramref name="y"/> is zero.</exception>
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
    /// <param name="x">The dividend.</param>
    /// <param name="y">The divisor.</param>
    /// <returns>The signed remainder, with the sign of the dividend when nonzero.</returns>
    /// <exception cref="DivideByZeroException"><paramref name="y"/> is zero.</exception>
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public static Fixed64 operator %(Fixed64 x, int y) => x % new Fixed64((long)y << FixedMath.SHIFT_AMOUNT_I);

    /// <summary>
    /// Computes the remainder of division of an int by a Fixed64.
    /// </summary>
    /// <param name="x">The dividend.</param>
    /// <param name="y">The divisor.</param>
    /// <returns>The signed remainder, with the sign of the dividend when nonzero.</returns>
    /// <exception cref="DivideByZeroException"><paramref name="y"/> is zero.</exception>
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public static Fixed64 operator %(int x, Fixed64 y) => new Fixed64((long)x << FixedMath.SHIFT_AMOUNT_I) % y;

    /// <summary>
    /// Unary negation operator.
    /// </summary>
    /// <param name="x">The value to negate.</param>
    /// <returns>The opposite signed value, or <see cref="MaxValue"/> when <paramref name="x"/> is <see cref="MinValue"/>.</returns>
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
    /// <param name="a">The value whose raw Q32.32 payload is shifted.</param>
    /// <param name="shift">The shift count. Only its low six bits are used.</param>
    /// <returns>The shifted raw Q32.32 payload, discarding high bits that shift out.</returns>
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public static Fixed64 operator <<(Fixed64 a, int shift) => new(a.m_rawValue << shift);

    /// <summary>
    /// Bitwise right shift operator.
    /// </summary>
    /// <param name="a">The value whose raw Q32.32 payload is shifted.</param>
    /// <param name="shift">The shift count. Only its low six bits are used.</param>
    /// <returns>The sign-extended shifted raw Q32.32 payload.</returns>
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public static Fixed64 operator >>(Fixed64 a, int shift) => new(a.m_rawValue >> shift);

    #endregion
    #region Comparison Operators

    /// <summary>
    /// Determines whether one Fixed64 is greater than another.
    /// </summary>
    /// <param name="x">The left operand.</param>
    /// <param name="y">The right operand.</param>
    /// <returns><see langword="true"/> if <paramref name="x"/> is greater than <paramref name="y"/>, otherwise <see langword="false"/>.</returns>
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public static bool operator >(Fixed64 x, Fixed64 y) => x.m_rawValue > y.m_rawValue;

    /// <summary>
    /// Determines whether a Fixed64 is greater than an integer.
    /// </summary>
    /// <param name="x">The left operand.</param>
    /// <param name="y">The right operand.</param>
    /// <returns><see langword="true"/> if <paramref name="x"/> is greater than <paramref name="y"/>, otherwise <see langword="false"/>.</returns>
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public static bool operator >(Fixed64 x, int y) => x.m_rawValue > (long)y << FixedMath.SHIFT_AMOUNT_I;

    /// <summary>
    /// Determines whether an integer is greater than a Fixed64.
    /// </summary>
    /// <param name="y">The left operand.</param>
    /// <param name="x">The right operand.</param>
    /// <returns><see langword="true"/> if <paramref name="y"/> is greater than <paramref name="x"/>, otherwise <see langword="false"/>.</returns>
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public static bool operator >(int y, Fixed64 x) => (long)y << FixedMath.SHIFT_AMOUNT_I > x.m_rawValue;

    /// <summary>
    /// Determines whether one Fixed64 is less than another.
    /// </summary>
    /// <param name="x">The left operand.</param>
    /// <param name="y">The right operand.</param>
    /// <returns><see langword="true"/> if <paramref name="x"/> is less than <paramref name="y"/>, otherwise <see langword="false"/>.</returns>
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public static bool operator <(Fixed64 x, Fixed64 y) => x.m_rawValue < y.m_rawValue;

    /// <summary>
    /// Determines whether one Fixed64 is less than an integer.
    /// </summary>
    /// <param name="x">The left operand.</param>
    /// <param name="y">The right operand.</param>
    /// <returns><see langword="true"/> if <paramref name="x"/> is less than <paramref name="y"/>, otherwise <see langword="false"/>.</returns>
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public static bool operator <(Fixed64 x, int y) => x.m_rawValue < (long)y << FixedMath.SHIFT_AMOUNT_I;

    /// <summary>
    /// Determines whether an integer is less than a Fixed64.
    /// </summary>
    /// <param name="y">The left operand.</param>
    /// <param name="x">The right operand.</param>
    /// <returns><see langword="true"/> if <paramref name="y"/> is less than <paramref name="x"/>, otherwise <see langword="false"/>.</returns>
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public static bool operator <(int y, Fixed64 x) => (long)y << FixedMath.SHIFT_AMOUNT_I < x.m_rawValue;

    /// <summary>
    /// Determines whether one Fixed64 is greater than or equal to another.
    /// </summary>
    /// <param name="x">The left operand.</param>
    /// <param name="y">The right operand.</param>
    /// <returns><see langword="true"/> if <paramref name="x"/> is greater than or equal to <paramref name="y"/>, otherwise <see langword="false"/>.</returns>
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public static bool operator >=(Fixed64 x, Fixed64 y) => x.m_rawValue >= y.m_rawValue;

    /// <summary>
    /// Determines whether Fixed64 is greater than or equal to an integer.
    /// </summary>
    /// <param name="x">The left operand.</param>
    /// <param name="y">The right operand.</param>
    /// <returns><see langword="true"/> if <paramref name="x"/> is greater than or equal to <paramref name="y"/>, otherwise <see langword="false"/>.</returns>
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public static bool operator >=(Fixed64 x, int y) => x.m_rawValue >= (long)y << FixedMath.SHIFT_AMOUNT_I;

    /// <summary>
    /// Determines whether an integer is greater than or equal to a Fixed64.
    /// </summary>
    /// <param name="y">The left operand.</param>
    /// <param name="x">The right operand.</param>
    /// <returns><see langword="true"/> if <paramref name="y"/> is greater than or equal to <paramref name="x"/>, otherwise <see langword="false"/>.</returns>
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public static bool operator >=(int y, Fixed64 x) => (long)y << FixedMath.SHIFT_AMOUNT_I >= x.m_rawValue;

    /// <summary>
    /// Determines whether one Fixed64 is less than or equal to another.
    /// </summary>
    /// <param name="x">The left operand.</param>
    /// <param name="y">The right operand.</param>
    /// <returns><see langword="true"/> if <paramref name="x"/> is less than or equal to <paramref name="y"/>, otherwise <see langword="false"/>.</returns>
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public static bool operator <=(Fixed64 x, Fixed64 y) => x.m_rawValue <= y.m_rawValue;

    /// <summary>
    /// Determines whether a Fixed64 is less than or equal to an integer.
    /// </summary>
    /// <param name="x">The left operand.</param>
    /// <param name="y">The right operand.</param>
    /// <returns><see langword="true"/> if <paramref name="x"/> is less than or equal to <paramref name="y"/>, otherwise <see langword="false"/>.</returns>
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public static bool operator <=(Fixed64 x, int y) => x.m_rawValue <= (long)y << FixedMath.SHIFT_AMOUNT_I;

    /// <summary>
    /// Determines whether an integer is less than or equal to a Fixed64.
    /// </summary>
    /// <param name="y">The left operand.</param>
    /// <param name="x">The right operand.</param>
    /// <returns><see langword="true"/> if <paramref name="y"/> is less than or equal to <paramref name="x"/>, otherwise <see langword="false"/>.</returns>
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public static bool operator <=(int y, Fixed64 x) => (long)y << FixedMath.SHIFT_AMOUNT_I <= x.m_rawValue;

    /// <summary>
    /// Determines whether two Fixed64 instances are equal.
    /// </summary>
    /// <param name="left">The left operand.</param>
    /// <param name="right">The right operand.</param>
    /// <returns><see langword="true"/> if <paramref name="left"/> is equal to <paramref name="right"/>, otherwise <see langword="false"/>.</returns>
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public static bool operator ==(Fixed64 left, Fixed64 right) => left.Equals(right);

    /// <summary>
    /// Determines whether a Fixed64 instance is equal to an integer.
    /// </summary>
    /// <param name="left">The left operand.</param>
    /// <param name="right">The right operand.</param>
    /// <returns><see langword="true"/> if <paramref name="left"/> is equal to <paramref name="right"/>, otherwise <see langword="false"/>.</returns>
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public static bool operator ==(Fixed64 left, int right) => left.m_rawValue == (long)right << FixedMath.SHIFT_AMOUNT_I;

    /// <summary>
    /// Determines whether an integer is equal to a Fixed64 instance.
    /// </summary>
    /// <param name="left">The left operand.</param>
    /// <param name="right">The right operand.</param>
    /// <returns><see langword="true"/> if <paramref name="left"/> is equal to <paramref name="right"/>, otherwise <see langword="false"/>.</returns>
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public static bool operator ==(int left, Fixed64 right) => (long)left << FixedMath.SHIFT_AMOUNT_I == right.m_rawValue;

    /// <summary>
    /// Determines whether two Fixed64 instances are not equal.
    /// </summary>
    /// <param name="left">The left operand.</param>
    /// <param name="right">The right operand.</param>
    /// <returns><see langword="true"/> if <paramref name="left"/> is different from <paramref name="right"/>, otherwise <see langword="false"/>.</returns>
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public static bool operator !=(Fixed64 left, Fixed64 right) => !left.Equals(right);

    /// <summary>
    /// Determines whether a Fixed64 instance is not equal to an integer.
    /// </summary>
    /// <param name="left">The left operand.</param>
    /// <param name="right">The right operand.</param>
    /// <returns><see langword="true"/> if <paramref name="left"/> is different from <paramref name="right"/>, otherwise <see langword="false"/>.</returns>
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public static bool operator !=(Fixed64 left, int right) => left.m_rawValue != (long)right << FixedMath.SHIFT_AMOUNT_I;

    /// <summary>
    /// Determines whether an integer differs from a Fixed64 instance.
    /// </summary>
    /// <param name="left">The left operand.</param>
    /// <param name="right">The right operand.</param>
    /// <returns><see langword="true"/> if <paramref name="left"/> is different from <paramref name="right"/>, otherwise <see langword="false"/>.</returns>
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public static bool operator !=(int left, Fixed64 right) => (long)left << FixedMath.SHIFT_AMOUNT_I != right.m_rawValue;

    #endregion
}
