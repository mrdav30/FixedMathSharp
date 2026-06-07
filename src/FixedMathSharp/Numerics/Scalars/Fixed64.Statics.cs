//=======================================================================
// Fixed64.Statics.cs
//=======================================================================
// MIT License, Copyright (c) 2024–present David Oravsky (mrdav30)
// See LICENSE file in the project root for full license information.
//=======================================================================

using System.Runtime.CompilerServices;

namespace FixedMathSharp;

public readonly partial struct Fixed64
{
    #region Static Operations

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
    /// <remarks>
    /// Optimized for branchless comparison.
    /// </remarks>
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public static int Sign(Fixed64 value) =>
        value.m_rawValue < 0
                ? -1
                : (value.m_rawValue > 0 ? 1 : 0);

    /// <summary>
    /// Returns true if the number has no decimal part (i.e., if the number is equivalent to an integer) and False otherwise. 
    /// </summary>
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public static bool IsInteger(Fixed64 value)
    {
        return ((ulong)value.m_rawValue & FixedMath.MAX_SHIFTED_AMOUNT_UI) == 0;
    }

    /// <summary>
    /// Divides by a known-positive fixed-point divisor using the same quotient and rounding
    /// semantics as <see cref="op_Division(Fixed64, Fixed64)"/>.
    /// </summary>
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    internal static Fixed64 DivideByPositive(Fixed64 x, Fixed64 y)
    {
        long xl = x.m_rawValue;
        long yl = y.m_rawValue;

        if (yl <= 0)
            return x / y;

        ulong remainder = (ulong)(xl < 0 ? -xl : xl);
        ulong divider = (ulong)yl;
        ulong quotient = 0UL;
        int bitPos = FixedMath.SHIFT_AMOUNT_I + 1;

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

            if ((div & ~(0xFFFFFFFFFFFFFFFF >> bitPos)) != 0)
                return xl >= 0
                    ? new Fixed64(FixedMath.MAX_VALUE_L)
                    : new Fixed64(FixedMath.MIN_VALUE_L);

            remainder <<= 1;
            --bitPos;
        }

        if ((quotient & 0x1) != 0)
            quotient += 1;

        long result = (long)(quotient >> 1);
        if (xl < 0)
            result = -result;

        return new Fixed64(result);
    }

    #endregion
}
