//=======================================================================
// Fixed64.MultiplyAdd.cs
//=======================================================================
// MIT License, Copyright (c) 2024–present David Oravsky (mrdav30)
// See LICENSE file in the project root for full license information.
//=======================================================================

using System;
using System.Runtime.CompilerServices;

namespace FixedMathSharp;

/// <content>
/// Provides fused arithmetic operations for <see cref="Fixed64"/> that combine multiple
/// multiplications, additions, subtractions, or divisions into a single rounding step for
/// improved precision and no intermediate saturation. This includes fused multiply-add
/// (<c>(left * right) + addend</c>), clamped multiply-subtract for two, three, and four
/// factor products, fused multiply-difference, and fused multiply-divide for two and
/// three factor numerators.
/// </content>
public partial struct Fixed64
{
    /// <summary>
    /// Multiplies two values, adds a third, and performs one final
    /// round-half-to-even conversion. An unrepresentable result saturates.
    /// </summary>
    public static Fixed64 MultiplyAdd(Fixed64 left, Fixed64 right, Fixed64 addend) =>
        MultiplyAdd(left, right, addend, out _);

    /// <summary>
    /// Attempts to multiply two values and add a third with one final
    /// round-half-to-even conversion and no intermediate saturation.
    /// </summary>
    public static bool TryMultiplyAdd(
        Fixed64 left,
        Fixed64 right,
        Fixed64 addend,
        out Fixed64 result)
    {
        result = MultiplyAdd(left, right, addend, out bool representable);
        if (representable)
            return true;

        result = default;
        return false;
    }

    private static Fixed64 MultiplyAdd(
        Fixed64 left,
        Fixed64 right,
        Fixed64 addend,
        out bool representable)
    {
        long leftRaw = left.m_rawValue;
        long rightRaw = right.m_rawValue;
        Multiply64To128(
            AbsToUInt64(leftRaw),
            AbsToUInt64(rightRaw),
            out ulong productHigh,
            out ulong productLow);
        Signed192 product = new(0UL, productHigh, productLow);
        if ((leftRaw ^ rightRaw) < 0L)
            product = WideArithmetic.SubtractSigned192(default, product);

        long addendRaw = addend.m_rawValue;
        ulong extension = addendRaw < 0L ? ulong.MaxValue : 0UL;
        Signed192 scaledAddend = new(
            extension,
            unchecked((ulong)(addendRaw >> FixedMath.SHIFT_AMOUNT_I)),
            unchecked((ulong)addendRaw << FixedMath.SHIFT_AMOUNT_I));
        Signed192 numerator = WideArithmetic.AddSigned192(product, scaledAddend);
        bool negative = numerator.Sign < 0;
        WideArithmetic.GetMagnitude(
            numerator,
            out _,
            out ulong numeratorMiddle,
            out ulong numeratorLow);

        if ((numeratorMiddle >> FixedMath.SHIFT_AMOUNT_I) != 0UL)
        {
            representable = false;
            return negative ? MinValue : MaxValue;
        }

        ulong quotient = (numeratorMiddle << FixedMath.SHIFT_AMOUNT_I)
            | (numeratorLow >> FixedMath.SHIFT_AMOUNT_I);
        ulong guardMask = 1UL << (FixedMath.SHIFT_AMOUNT_I - 1);
        bool guard = (numeratorLow & guardMask) != 0UL;
        bool sticky = (numeratorLow & (guardMask - 1UL)) != 0UL;
        return RoundAndApplySign(
            quotient,
            guard,
            sticky,
            negative,
            out representable);
    }

    /// <summary>
    /// Attempts to compute <c>max(0, minuendFirst * minuendSecond - subtrahendFirst * subtrahendSecond)</c>
    /// with no intermediate saturation and one final round-half-to-even conversion.
    /// </summary>
    /// <param name="minuendFirst">The first factor of the minuend product.</param>
    /// <param name="minuendSecond">The second factor of the minuend product.</param>
    /// <param name="subtrahendFirst">The first factor of the subtrahend product.</param>
    /// <param name="subtrahendSecond">The second factor of the subtrahend product.</param>
    /// <param name="result">
    /// The nonnegative fused result; otherwise, <see langword="default"/> when a positive
    /// rounded result is not representable.
    /// </param>
    /// <returns>
    /// <see langword="true"/> when the exact difference is nonpositive or its final
    /// round-half-to-even result is representable; otherwise, <see langword="false"/>.
    /// </returns>
    public static bool TryMultiplySubtractClamped(
        Fixed64 minuendFirst,
        Fixed64 minuendSecond,
        Fixed64 subtrahendFirst,
        Fixed64 subtrahendSecond,
        out Fixed64 result)
    {
        Signed192 difference = WideArithmetic.SubtractSigned192(
            GetExactRawProduct(minuendFirst, minuendSecond),
            GetExactRawProduct(subtrahendFirst, subtrahendSecond));
        return TryRoundNonnegativeTwoFactorDifference(difference, out result);
    }

    /// <summary>
    /// Attempts to compute <c>max(0, minuendFirst * minuendSecond * minuendThird - subtrahendFirst * subtrahendSecond * subtrahendThird)</c>
    /// with no intermediate saturation and one final round-half-to-even conversion.
    /// </summary>
    /// <param name="minuendFirst">The first factor of the minuend product.</param>
    /// <param name="minuendSecond">The second factor of the minuend product.</param>
    /// <param name="minuendThird">The third factor of the minuend product.</param>
    /// <param name="subtrahendFirst">The first factor of the subtrahend product.</param>
    /// <param name="subtrahendSecond">The second factor of the subtrahend product.</param>
    /// <param name="subtrahendThird">The third factor of the subtrahend product.</param>
    /// <param name="result">
    /// The nonnegative fused result; otherwise, <see langword="default"/> when a positive
    /// rounded result is not representable.
    /// </param>
    /// <returns>
    /// <see langword="true"/> when the exact difference is nonpositive or its final
    /// round-half-to-even result is representable; otherwise, <see langword="false"/>.
    /// </returns>
    public static bool TryMultiplySubtractClamped(
        Fixed64 minuendFirst,
        Fixed64 minuendSecond,
        Fixed64 minuendThird,
        Fixed64 subtrahendFirst,
        Fixed64 subtrahendSecond,
        Fixed64 subtrahendThird,
        out Fixed64 result)
    {
        Signed320 difference = WideArithmetic.SubtractSigned320(
            WideArithmetic.MultiplySigned192(
                GetExactRawProduct(minuendFirst, minuendSecond),
                Signed192.Signed(minuendThird.m_rawValue)),
            WideArithmetic.MultiplySigned192(
                GetExactRawProduct(subtrahendFirst, subtrahendSecond),
                Signed192.Signed(subtrahendThird.m_rawValue)));
        return TryRoundNonnegativeThreeFactorDifference(difference, out result);
    }

    /// <summary>
    /// Attempts to compute <c>max(0, minuendFirst * minuendSecond * minuendThird * minuendFourth - subtrahendFirst * subtrahendSecond * subtrahendThird * subtrahendFourth)</c>
    /// with no intermediate saturation and one final round-half-to-even conversion.
    /// </summary>
    /// <param name="minuendFirst">The first factor of the minuend product.</param>
    /// <param name="minuendSecond">The second factor of the minuend product.</param>
    /// <param name="minuendThird">The third factor of the minuend product.</param>
    /// <param name="minuendFourth">The fourth factor of the minuend product.</param>
    /// <param name="subtrahendFirst">The first factor of the subtrahend product.</param>
    /// <param name="subtrahendSecond">The second factor of the subtrahend product.</param>
    /// <param name="subtrahendThird">The third factor of the subtrahend product.</param>
    /// <param name="subtrahendFourth">The fourth factor of the subtrahend product.</param>
    /// <param name="result">
    /// The nonnegative fused result; otherwise, <see langword="default"/> when a positive
    /// rounded result is not representable.
    /// </param>
    /// <returns>
    /// <see langword="true"/> when the exact difference is nonpositive or its final
    /// round-half-to-even result is representable; otherwise, <see langword="false"/>.
    /// </returns>
    public static bool TryMultiplySubtractClamped(
        Fixed64 minuendFirst,
        Fixed64 minuendSecond,
        Fixed64 minuendThird,
        Fixed64 minuendFourth,
        Fixed64 subtrahendFirst,
        Fixed64 subtrahendSecond,
        Fixed64 subtrahendThird,
        Fixed64 subtrahendFourth,
        out Fixed64 result)
    {
        Signed320 difference = WideArithmetic.SubtractSigned320(
            WideArithmetic.MultiplySigned192(
                GetExactRawProduct(minuendFirst, minuendSecond),
                GetExactRawProduct(minuendThird, minuendFourth)),
            WideArithmetic.MultiplySigned192(
                GetExactRawProduct(subtrahendFirst, subtrahendSecond),
                GetExactRawProduct(subtrahendThird, subtrahendFourth)));
        return TryRoundNonnegativeFourFactorDifference(difference, out result);
    }

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    internal static Signed192 GetExactRawProduct(Fixed64 left, Fixed64 right)
    {
        long leftRaw = left.m_rawValue;
        long rightRaw = right.m_rawValue;
        Multiply64To128(
            AbsToUInt64(leftRaw),
            AbsToUInt64(rightRaw),
            out ulong productHigh,
            out ulong productLow);
        Signed192 product = new(0UL, productHigh, productLow);
        return (leftRaw ^ rightRaw) < 0L
            ? WideArithmetic.SubtractSigned192(default, product)
            : product;
    }

    private static bool TryRoundNonnegativeTwoFactorDifference(
        Signed192 difference,
        out Fixed64 result)
    {
        if (difference.Sign <= 0)
        {
            result = Zero;
            return true;
        }

        WideArithmetic.GetMagnitude(difference, out _, out ulong middle, out ulong low);
        if ((middle >> FixedMath.SHIFT_AMOUNT_I) != 0UL)
        {
            result = default;
            return false;
        }

        return TryRoundNonnegative(
            (middle << FixedMath.SHIFT_AMOUNT_I) | (low >> FixedMath.SHIFT_AMOUNT_I),
            (low & (1UL << (FixedMath.SHIFT_AMOUNT_I - 1))) != 0UL,
            (low & ((1UL << (FixedMath.SHIFT_AMOUNT_I - 1)) - 1UL)) != 0UL,
            out result);
    }

    private static bool TryRoundNonnegativeThreeFactorDifference(
        Signed320 difference,
        out Fixed64 result)
    {
        if (difference.Sign <= 0)
        {
            result = Zero;
            return true;
        }

        WideArithmetic.GetMagnitude(
            difference,
            out ulong word4,
            out ulong word3,
            out ulong word2,
            out ulong word1,
            out ulong word0);
        if ((word4 | word3 | word2) != 0UL)
        {
            result = default;
            return false;
        }

        return TryRoundNonnegative(
            word1,
            (word0 & (1UL << 63)) != 0UL,
            (word0 & ((1UL << 63) - 1UL)) != 0UL,
            out result);
    }

    private static bool TryRoundNonnegativeFourFactorDifference(
        Signed320 difference,
        out Fixed64 result)
    {
        if (difference.Sign <= 0)
        {
            result = Zero;
            return true;
        }

        WideArithmetic.GetMagnitude(
            difference,
            out _,
            out ulong word3,
            out ulong word2,
            out ulong word1,
            out ulong word0);
        if (word3 != 0UL || (word2 >> FixedMath.SHIFT_AMOUNT_I) != 0UL)
        {
            result = default;
            return false;
        }

        return TryRoundNonnegative(
            (word2 << FixedMath.SHIFT_AMOUNT_I) | (word1 >> FixedMath.SHIFT_AMOUNT_I),
            (word1 & (1UL << (FixedMath.SHIFT_AMOUNT_I - 1))) != 0UL,
            word0 != 0UL || (word1 & ((1UL << (FixedMath.SHIFT_AMOUNT_I - 1)) - 1UL)) != 0UL,
            out result);
    }

    private static bool TryRoundNonnegative(
        ulong quotient,
        bool guard,
        bool sticky,
        out Fixed64 result)
    {
        result = RoundAndApplySign(quotient, guard, sticky, negative: false, out bool representable);
        if (representable)
            return true;

        result = default;
        return false;
    }

    /// <summary>
    /// Attempts to evaluate
    /// <c>(value - subtrahend) * firstMultiplier * secondMultiplier</c> with
    /// one final round-half-to-even conversion.
    /// </summary>
    /// <remarks>
    /// The subtraction and both products remain exact even when an intermediate
    /// value lies outside the public Q32.32 domain.
    /// </remarks>
    public static bool TryMultiplyDifference(
        Fixed64 value,
        Fixed64 subtrahend,
        Fixed64 firstMultiplier,
        Fixed64 secondMultiplier,
        out Fixed64 result)
    {
        long valueRaw = value.m_rawValue;
        long subtrahendRaw = subtrahend.m_rawValue;
        bool negativeDifference = valueRaw < subtrahendRaw;
        ulong differenceMagnitude = negativeDifference
            ? unchecked((ulong)subtrahendRaw - (ulong)valueRaw)
            : unchecked((ulong)valueRaw - (ulong)subtrahendRaw);
        long firstRaw = firstMultiplier.m_rawValue;
        long secondRaw = secondMultiplier.m_rawValue;
        bool negative = negativeDifference
            ^ (firstRaw < 0L)
            ^ (secondRaw < 0L);

        Multiply64To128(
            differenceMagnitude,
            AbsToUInt64(firstRaw),
            out ulong productHigh,
            out ulong productLow);
        Multiply64To128(
            productLow,
            AbsToUInt64(secondRaw),
            out ulong lowProductHigh,
            out ulong numeratorLow);
        Multiply64To128(
            productHigh,
            AbsToUInt64(secondRaw),
            out ulong numeratorHigh,
            out ulong highProductLow);
        ulong numeratorMiddle = unchecked(lowProductHigh + highProductLow);
        if (numeratorMiddle < lowProductHigh)
            numeratorHigh++;

        if (numeratorHigh != 0UL)
        {
            result = default;
            return false;
        }

        const ulong GuardMask = 1UL << 63;
        bool guard = (numeratorLow & GuardMask) != 0UL;
        bool sticky = (numeratorLow & (GuardMask - 1UL)) != 0UL;
        result = RoundAndApplySign(
            numeratorMiddle,
            guard,
            sticky,
            negative,
            out bool representable);
        if (representable)
            return true;

        result = default;
        return false;
    }

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
    /// Attempts to multiply four values and divide by the exact sum of four
    /// values with one final round-half-to-even operation.
    /// </summary>
    public static bool TryMultiplyDivideBySum(
        Fixed64 first,
        Fixed64 second,
        Fixed64 third,
        Fixed64 fourth,
        Fixed64 firstDivisorTerm,
        Fixed64 secondDivisorTerm,
        Fixed64 thirdDivisorTerm,
        Fixed64 fourthDivisorTerm,
        out Fixed64 result)
    {
        Signed192 divisor = WideArithmetic.AddSigned192(
            WideArithmetic.AddSigned192(
                Signed192.Raw(firstDivisorTerm),
                Signed192.Raw(secondDivisorTerm)),
            WideArithmetic.AddSigned192(
                Signed192.Raw(thirdDivisorTerm),
                Signed192.Raw(fourthDivisorTerm)));
        Signed576 numerator = WideArithmetic.MultiplySigned576(
            Signed576.ExtendValue(
                Signed320.ExtendValue(Signed192.Raw(first))),
            Signed192.Raw(second),
            Signed192.Raw(third),
            Signed192.Raw(fourth));
        Signed576 scaledDivisor = WideArithmetic.MultiplySigned576(
            Signed576.ExtendValue(Signed320.ExtendValue(divisor)),
            Signed192.One,
            Signed192.One);
        return TryGetSignedRawRatio(
            numerator,
            scaledDivisor,
            out result);
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
