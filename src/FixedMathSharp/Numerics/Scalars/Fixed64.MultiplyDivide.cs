//=======================================================================
// Fixed64.MultiplyDivide.cs
//=======================================================================
// MIT License, Copyright (c) 2024–present David Oravsky (mrdav30)
// See LICENSE file in the project root for full license information.
//=======================================================================

using System;
using System.Runtime.CompilerServices;

namespace FixedMathSharp;

public partial struct Fixed64
{
    /// <summary>
    /// Attempts to multiply two values and divide by a third with one final
    /// round-half-to-even operation and no intermediate saturation.
    /// </summary>
    /// <param name="left">The first factor.</param>
    /// <param name="right">The second factor.</param>
    /// <param name="divisor">The divisor.</param>
    /// <param name="result">
    /// The fused result when representable; otherwise, <see langword="default"/>.
    /// </param>
    /// <returns>
    /// <see langword="true"/> when the divisor is nonzero and the final result is
    /// representable; otherwise, <see langword="false"/>.
    /// </returns>
    public static bool TryMultiplyDivide(
        Fixed64 left,
        Fixed64 right,
        Fixed64 divisor,
        out Fixed64 result)
    {
        result = MultiplyDivide(left, right, divisor, out bool representable);
        if (representable)
            return true;

        result = default;
        return false;
    }

    /// <summary>
    /// Computes a fused two-factor multiply-divide and returns a saturated value
    /// when the exact result is not representable.
    /// </summary>
    internal static Fixed64 MultiplyDivide(
        Fixed64 left,
        Fixed64 right,
        Fixed64 divisor,
        out bool representable)
    {
        long divisorRaw = divisor.m_rawValue;
        if (divisorRaw == 0L)
        {
            representable = false;
            return default;
        }

        long leftRaw = left.m_rawValue;
        long rightRaw = right.m_rawValue;
        bool negative = (leftRaw ^ rightRaw ^ divisorRaw) < 0L;
        ulong divisorMagnitude = AbsToUInt64(divisorRaw);

        Multiply64To128(
            AbsToUInt64(leftRaw),
            AbsToUInt64(rightRaw),
            out ulong productHigh,
            out ulong productLow);
        CancelCommonPowersOfTwo(
            ref productHigh,
            ref productLow,
            ref divisorMagnitude);

        if (productHigh >= divisorMagnitude)
        {
            representable = false;
            return negative ? MinValue : MaxValue;
        }

        ulong quotient = Divide128By64(
            productHigh,
            productLow,
            divisorMagnitude,
            out ulong remainder);
        ulong twiceRemainder = remainder << 1;
        bool guard = twiceRemainder >= divisorMagnitude;
        bool sticky = guard
            ? twiceRemainder != divisorMagnitude
            : twiceRemainder != 0UL;

        return RoundAndApplySign(quotient, guard, sticky, negative, out representable);
    }

    /// <summary>
    /// Attempts to multiply three values and divide by a fourth with one final
    /// round-half-to-even operation and no intermediate saturation.
    /// </summary>
    /// <param name="first">The first factor.</param>
    /// <param name="second">The second factor.</param>
    /// <param name="third">The third factor.</param>
    /// <param name="divisor">The divisor.</param>
    /// <param name="result">
    /// The fused result when representable; otherwise, <see langword="default"/>.
    /// </param>
    /// <returns>
    /// <see langword="true"/> when the divisor is nonzero and the final result is
    /// representable; otherwise, <see langword="false"/>.
    /// </returns>
    public static bool TryMultiplyDivide(
        Fixed64 first,
        Fixed64 second,
        Fixed64 third,
        Fixed64 divisor,
        out Fixed64 result)
    {
        result = MultiplyDivide(first, second, third, divisor, out bool representable);
        if (representable)
            return true;

        result = default;
        return false;
    }

    /// <summary>
    /// Computes a fused three-factor multiply-divide and returns a saturated value
    /// when the exact result is not representable.
    /// </summary>
    internal static Fixed64 MultiplyDivide(
        Fixed64 first,
        Fixed64 second,
        Fixed64 third,
        Fixed64 divisor,
        out bool representable)
    {
        long divisorRaw = divisor.m_rawValue;
        if (divisorRaw == 0L)
        {
            representable = false;
            return default;
        }

        long firstRaw = first.m_rawValue;
        long secondRaw = second.m_rawValue;
        long thirdRaw = third.m_rawValue;
        bool negative = (firstRaw ^ secondRaw ^ thirdRaw ^ divisorRaw) < 0L;
        ulong thirdMagnitude = AbsToUInt64(thirdRaw);
        ulong divisorMagnitude = AbsToUInt64(divisorRaw);

        Multiply64To128(
            AbsToUInt64(firstRaw),
            AbsToUInt64(secondRaw),
            out ulong productHigh,
            out ulong productLow);
        Multiply64To128(
            productLow,
            thirdMagnitude,
            out ulong lowProductHigh,
            out ulong numeratorLow);
        Multiply64To128(
            productHigh,
            thirdMagnitude,
            out ulong numeratorHigh,
            out ulong highProductLow);

        ulong numeratorMiddle = unchecked(lowProductHigh + highProductLow);
        if (numeratorMiddle < lowProductHigh)
            numeratorHigh++;
        CancelCommonPowersOfTwo(
            ref numeratorHigh,
            ref numeratorMiddle,
            ref numeratorLow,
            ref divisorMagnitude);

        ulong quotientHigh = numeratorHigh / divisorMagnitude;
        ulong remainder = numeratorHigh % divisorMagnitude;
        ulong quotientMiddle = Divide128By64(
            remainder,
            numeratorMiddle,
            divisorMagnitude,
            out remainder);
        ulong quotientLow = Divide128By64(
            remainder,
            numeratorLow,
            divisorMagnitude,
            out remainder);

        if (quotientHigh != 0UL || (quotientMiddle >> FixedMath.SHIFT_AMOUNT_I) != 0UL)
        {
            representable = false;
            return negative ? MinValue : MaxValue;
        }

        ulong quotient = (quotientMiddle << FixedMath.SHIFT_AMOUNT_I)
            | (quotientLow >> FixedMath.SHIFT_AMOUNT_I);
        ulong discardedMask = (1UL << (FixedMath.SHIFT_AMOUNT_I - 1)) - 1UL;
        bool guard = (quotientLow & (1UL << (FixedMath.SHIFT_AMOUNT_I - 1))) != 0UL;
        bool sticky = ((quotientLow & discardedMask) | remainder) != 0UL;

        return RoundAndApplySign(quotient, guard, sticky, negative, out representable);
    }

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    private static int CountTrailingZeroes(ulong value) =>
        63 - CountLeadingZeroes(value & unchecked(0UL - value));

    private static void CancelCommonPowersOfTwo(
        ref ulong high,
        ref ulong low,
        ref ulong divisor)
    {
        if ((divisor & 1UL) != 0UL || (high | low) == 0UL)
            return;

        int numeratorZeroes = low != 0UL
            ? CountTrailingZeroes(low)
            : 64 + CountTrailingZeroes(high);
        int shift = Math.Min(numeratorZeroes, CountTrailingZeroes(divisor));
        if (shift == 0)
            return;

        low = (low >> shift) | (high << (64 - shift));
        high >>= shift;
        divisor >>= shift;
    }

    private static void CancelCommonPowersOfTwo(
        ref ulong high,
        ref ulong middle,
        ref ulong low,
        ref ulong divisor)
    {
        if ((divisor & 1UL) != 0UL || (high | middle | low) == 0UL)
            return;

        int numeratorZeroes = low != 0UL
            ? CountTrailingZeroes(low)
            : middle != 0UL
                ? 64 + CountTrailingZeroes(middle)
                : 128 + CountTrailingZeroes(high);
        int shift = Math.Min(numeratorZeroes, CountTrailingZeroes(divisor));
        if (shift == 0)
            return;

        low = (low >> shift) | (middle << (64 - shift));
        middle = (middle >> shift) | (high << (64 - shift));
        high >>= shift;
        divisor >>= shift;
    }

    /// <summary>
    /// Divides an unsigned 128-bit numerator by a 64-bit divisor when the quotient
    /// is known to fit in 64 bits.
    /// </summary>
    private static ulong Divide128By64(
        ulong high,
        ulong low,
        ulong divisor,
        out ulong remainder)
    {
        if (high == 0UL)
        {
            remainder = low % divisor;
            return low / divisor;
        }

        if (divisor <= uint.MaxValue)
        {
            ulong upper = (high << 32) | (low >> 32);
            ulong quotientHigh = upper / divisor;
            remainder = upper % divisor;
            ulong lower = (remainder << 32) | (uint)low;
            ulong quotientLow = lower / divisor;
            remainder = lower % divisor;
            return (quotientHigh << 32) | quotientLow;
        }

        ulong quotient = 0UL;
        remainder = high;
        for (int bit = 63; bit >= 0; bit--)
        {
            bool carry = (remainder & (1UL << 63)) != 0UL;
            remainder = (remainder << 1) | ((low >> bit) & 1UL);
            if (carry || remainder >= divisor)
            {
                remainder = unchecked(remainder - divisor);
                quotient |= 1UL << bit;
            }
        }

        return quotient;
    }

    private static Fixed64 RoundAndApplySign(
        ulong quotient,
        bool guard,
        bool sticky,
        bool negative,
        out bool representable)
    {
        const ulong minValueMagnitude = 1UL << 63;
        if (quotient >= minValueMagnitude)
        {
            if (quotient == minValueMagnitude && guard && sticky)
                quotient++;

            return ApplySignedMagnitude(quotient, negative, out representable);
        }

        ulong guardedQuotient = (quotient << 1) | (guard ? 1UL : 0UL);
        ulong magnitude = RoundGuardedQuotientToEven(
            guardedQuotient,
            sticky,
            out _);
        return ApplySignedMagnitude(magnitude, negative, out representable);
    }

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    private static Fixed64 ApplySignedMagnitude(
        ulong magnitude,
        bool negative,
        out bool representable)
    {
        const ulong minValueMagnitude = 1UL << 63;
        ulong limit = negative ? minValueMagnitude : long.MaxValue;
        representable = magnitude <= limit;
        if (!representable)
            return negative ? MinValue : MaxValue;
        if (magnitude == minValueMagnitude)
            return MinValue;

        long rawResult = (long)magnitude;
        return new Fixed64(negative ? -rawResult : rawResult);
    }
}
