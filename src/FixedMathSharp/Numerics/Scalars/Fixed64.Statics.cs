//=======================================================================
// Fixed64.Statics.cs
//=======================================================================
// MIT License, Copyright (c) 2024-present David Oravsky (mrdav30)
// See LICENSE file in the project root for full license information.
//=======================================================================

using System.Runtime.CompilerServices;

namespace FixedMathSharp;

/// <content>
/// Static helper operations for <see cref="Fixed64"/>, including sign/integer checks,
/// leading-zero counting, and exact wide-precision product comparisons.
/// </content>
public partial struct Fixed64
{
    /// <summary>
    /// Counts the leading zeros in a 64-bit unsigned integer.
    /// </summary>
    /// <param name="x">The number to count leading zeros for.</param>
    /// <returns>The number of leading zeros.</returns>
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    internal static int CountLeadingZeroes(ulong x)
    {
#if NET10_0_OR_GREATER
        return System.Numerics.BitOperations.LeadingZeroCount(x);
#else
        if (x == 0UL)
            return 64;

        int result = 0;
        while ((x & 0xF000000000000000) == 0) { result += 4; x <<= 4; }
        while ((x & 0x8000000000000000) == 0) { result += 1; x <<= 1; }
        return result;
#endif
    }

    /// <summary>Determines the sign of a fixed-point value.</summary>
    /// <param name="value">The value to classify.</param>
    /// <returns>−1 for a negative value, zero for zero, or 1 for a positive value.</returns>
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public static int Sign(Fixed64 value) =>
        value.m_rawValue < 0
                ? -1
                : (value.m_rawValue > 0 ? 1 : 0);

    /// <summary>Tests whether the value has a fractional part.</summary>
    /// <param name="value">The value to test.</param>
    /// <returns><see langword="true"/> if every fractional bit is zero, otherwise <see langword="false"/>.</returns>
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public static bool IsInteger(Fixed64 value)
    {
        return ((ulong)value.m_rawValue & FixedMath.MAX_SHIFTED_AMOUNT_UI) == 0;
    }

    /// <summary>
    /// Compares the exact mathematical product <paramref name="leftFirst"/> *
    /// <paramref name="leftSecond"/> with <paramref name="rightFirst"/> *
    /// <paramref name="rightSecond"/> without rounding or saturation.
    /// </summary>
    /// <returns>
    /// A negative value when the left product is smaller, zero when the exact
    /// products are equal, or a positive value when the left product is larger.
    /// </returns>
    /// <param name="leftFirst">The first factor in the left product.</param>
    /// <param name="leftSecond">The second factor in the left product.</param>
    /// <param name="rightFirst">The first factor in the right product.</param>
    /// <param name="rightSecond">The second factor in the right product.</param>
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public static int CompareProducts(
        Fixed64 leftFirst,
        Fixed64 leftSecond,
        Fixed64 rightFirst,
        Fixed64 rightSecond) =>
        WideArithmetic.SubtractSigned192(
            GetExactRawProduct(leftFirst, leftSecond),
            GetExactRawProduct(rightFirst, rightSecond)).Sign;

    /// <summary>
    /// Compares the exact four-factor mathematical product
    /// <paramref name="leftFirst"/> * <paramref name="leftSecond"/> *
    /// <paramref name="leftThird"/> * <paramref name="leftFourth"/> with
    /// <paramref name="rightFirst"/> * <paramref name="rightSecond"/> *
    /// <paramref name="rightThird"/> * <paramref name="rightFourth"/> without
    /// rounding or saturation.
    /// </summary>
    /// <returns>
    /// A negative value when the left product is smaller, zero when the exact
    /// products are equal, or a positive value when the left product is larger.
    /// </returns>
    /// <param name="leftFirst">The first factor in the left product.</param>
    /// <param name="leftSecond">The second factor in the left product.</param>
    /// <param name="leftThird">The third factor in the left product.</param>
    /// <param name="leftFourth">The fourth factor in the left product.</param>
    /// <param name="rightFirst">The first factor in the right product.</param>
    /// <param name="rightSecond">The second factor in the right product.</param>
    /// <param name="rightThird">The third factor in the right product.</param>
    /// <param name="rightFourth">The fourth factor in the right product.</param>
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public static int CompareProducts(
        Fixed64 leftFirst,
        Fixed64 leftSecond,
        Fixed64 leftThird,
        Fixed64 leftFourth,
        Fixed64 rightFirst,
        Fixed64 rightSecond,
        Fixed64 rightThird,
        Fixed64 rightFourth) =>
        WideArithmetic.SubtractSigned320(
            WideArithmetic.MultiplySigned192(
                GetExactRawProduct(leftFirst, leftSecond),
                GetExactRawProduct(leftThird, leftFourth)),
            WideArithmetic.MultiplySigned192(
                GetExactRawProduct(rightFirst, rightSecond),
                GetExactRawProduct(rightThird, rightFourth))).Sign;
}
