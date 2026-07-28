//=======================================================================
// Fixed64.WideRawRatio.cs
//=======================================================================
// MIT License, Copyright (c) 2024–present David Oravsky (mrdav30)
// See LICENSE file in the project root for full license information.
//=======================================================================

using System;
using System.Runtime.CompilerServices;

namespace FixedMathSharp;

/// <content>
/// Provides wide-precision (128/192-bit) ratio-to-<see cref="Fixed64"/> conversion helpers,
/// used for computing exact division and unit-interval results without precision loss.
/// </content>
public partial struct Fixed64
{
    /// <summary>
    /// Converts an exact ratio proven by this method to lie in [0, 1] to Q32.32.
    /// </summary>
    internal static bool TryGetUnitIntervalRatio(
        Signed192 numerator,
        Signed192 denominator,
        out Fixed64 result)
    {
        int denominatorSign = denominator.Sign;
        int numeratorSign = numerator.Sign;
        if (denominatorSign == 0 || (numeratorSign != 0 && numeratorSign != denominatorSign))
        {
            result = default;
            return false;
        }

        if (numeratorSign == 0)
        {
            result = Zero;
            return true;
        }

        WideArithmetic.GetMagnitude(numerator, out ulong numeratorHigh, out ulong numeratorMiddle, out ulong numeratorLow);
        WideArithmetic.GetMagnitude(denominator, out ulong denominatorHigh, out ulong denominatorMiddle, out ulong denominatorLow);
        int comparison = WideArithmetic.CompareUnsigned(
            numeratorHigh,
            numeratorMiddle,
            numeratorLow,
            denominatorHigh,
            denominatorMiddle,
            denominatorLow);
        if (comparison > 0)
        {
            result = default;
            return false;
        }

        if (comparison == 0)
        {
            result = One;
            return true;
        }

        result = GetUnitIntervalRatio(
            numeratorHigh,
            numeratorMiddle,
            numeratorLow,
            denominatorHigh,
            denominatorMiddle,
            denominatorLow);
        return true;
    }

    /// <summary>
    /// Converts an exact five-word ratio proven by this method to lie in [0, 1]
    /// to Q32.32 using guard/sticky round-half-to-even state.
    /// </summary>
    internal static bool TryGetUnitIntervalRatio(
        Signed320 numerator,
        Signed320 denominator,
        out Fixed64 result)
    {
        if (Signed192.TryNarrowSigned(numerator, out Signed192 narrowNumerator)
            && Signed192.TryNarrowSigned(denominator, out Signed192 narrowDenominator))
        {
            return TryGetUnitIntervalRatio(narrowNumerator, narrowDenominator, out result);
        }

        int denominatorSign = denominator.Sign;
        int numeratorSign = numerator.Sign;
        if (denominatorSign == 0 || (numeratorSign != 0 && numeratorSign != denominatorSign))
        {
            result = default;
            return false;
        }

        if (numeratorSign == 0)
        {
            result = Zero;
            return true;
        }

        WideArithmetic.GetMagnitude(
            numerator,
            out ulong numeratorWord4,
            out ulong numeratorWord3,
            out ulong numeratorWord2,
            out ulong numeratorWord1,
            out ulong numeratorWord0);
        WideArithmetic.GetMagnitude(
            denominator,
            out ulong denominatorWord4,
            out ulong denominatorWord3,
            out ulong denominatorWord2,
            out ulong denominatorWord1,
            out ulong denominatorWord0);
        int comparison = WideArithmetic.CompareUnsigned(
            numeratorWord4,
            numeratorWord3,
            numeratorWord2,
            numeratorWord1,
            numeratorWord0,
            denominatorWord4,
            denominatorWord3,
            denominatorWord2,
            denominatorWord1,
            denominatorWord0);
        if (comparison > 0)
        {
            result = default;
            return false;
        }

        if (comparison == 0)
        {
            result = One;
            return true;
        }

        result = GetUnitIntervalRatio(
            numeratorWord4,
            numeratorWord3,
            numeratorWord2,
            numeratorWord1,
            numeratorWord0,
            denominatorWord4,
            denominatorWord3,
            denominatorWord2,
            denominatorWord1,
            denominatorWord0);
        return true;
    }

    private static Fixed64 GetUnitIntervalRatio(
        ulong numeratorWord4,
        ulong numeratorWord3,
        ulong numeratorWord2,
        ulong numeratorWord1,
        ulong numeratorWord0,
        ulong denominatorWord4,
        ulong denominatorWord3,
        ulong denominatorWord2,
        ulong denominatorWord1,
        ulong denominatorWord0)
    {
        ulong guardedQuotient = 0UL;
        for (int bit = FixedMath.SHIFT_AMOUNT_I; bit >= 0; bit--)
        {
            WideArithmetic.ShiftLeftOne(
                ref numeratorWord4,
                ref numeratorWord3,
                ref numeratorWord2,
                ref numeratorWord1,
                ref numeratorWord0);
            guardedQuotient <<= 1;
            if (WideArithmetic.CompareUnsigned(
                numeratorWord4,
                numeratorWord3,
                numeratorWord2,
                numeratorWord1,
                numeratorWord0,
                denominatorWord4,
                denominatorWord3,
                denominatorWord2,
                denominatorWord1,
                denominatorWord0) < 0)
            {
                continue;
            }

            WideArithmetic.SubtractUnsigned(
                ref numeratorWord4,
                ref numeratorWord3,
                ref numeratorWord2,
                ref numeratorWord1,
                ref numeratorWord0,
                denominatorWord4,
                denominatorWord3,
                denominatorWord2,
                denominatorWord1,
                denominatorWord0);
            guardedQuotient |= 1UL;
        }

        ulong rounded = RoundGuardedQuotientToEven(
            guardedQuotient,
            (numeratorWord4 | numeratorWord3 | numeratorWord2 | numeratorWord1 | numeratorWord0) != 0UL,
            out _);
        return new Fixed64((long)rounded);
    }

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    private static Fixed64 GetUnitIntervalRatio(
        ulong numeratorHigh,
        ulong numeratorMiddle,
        ulong numeratorLow,
        ulong denominatorHigh,
        ulong denominatorMiddle,
        ulong denominatorLow)
    {
        if ((numeratorHigh | denominatorHigh) == 0UL)
        {
            return GetUnitIntervalRatio128(
                numeratorMiddle,
                numeratorLow,
                denominatorMiddle,
                denominatorLow);
        }

        ulong guardedQuotient = 0UL;
        for (int bit = FixedMath.SHIFT_AMOUNT_I; bit >= 0; bit--)
        {
            WideArithmetic.ShiftLeftOne(ref numeratorHigh, ref numeratorMiddle, ref numeratorLow);
            guardedQuotient <<= 1;
            if (WideArithmetic.CompareUnsigned(
                numeratorHigh,
                numeratorMiddle,
                numeratorLow,
                denominatorHigh,
                denominatorMiddle,
                denominatorLow) < 0)
            {
                continue;
            }

            WideArithmetic.SubtractUnsigned(
                ref numeratorHigh,
                ref numeratorMiddle,
                ref numeratorLow,
                denominatorHigh,
                denominatorMiddle,
                denominatorLow);
            guardedQuotient |= 1UL;
        }

        ulong rounded = RoundGuardedQuotientToEven(
            guardedQuotient,
            (numeratorHigh | numeratorMiddle | numeratorLow) != 0UL,
            out _);
        return new Fixed64((long)rounded);
    }

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    private static Fixed64 GetUnitIntervalRatio128(
        ulong numeratorHigh,
        ulong numeratorLow,
        ulong denominatorHigh,
        ulong denominatorLow)
    {
        ulong guardedQuotient = 0UL;
        for (int bit = FixedMath.SHIFT_AMOUNT_I; bit >= 0; bit--)
        {
            bool overflow = (numeratorHigh & (1UL << 63)) != 0UL;
            numeratorHigh = (numeratorHigh << 1) | (numeratorLow >> 63);
            numeratorLow <<= 1;
            guardedQuotient <<= 1;

            if (!overflow
                && (numeratorHigh < denominatorHigh
                    || (numeratorHigh == denominatorHigh && numeratorLow < denominatorLow)))
            {
                continue;
            }

            ulong previousLow = numeratorLow;
            numeratorLow = unchecked(numeratorLow - denominatorLow);
            ulong borrow = previousLow < denominatorLow ? 1UL : 0UL;
            numeratorHigh = unchecked(numeratorHigh - denominatorHigh - borrow);
            guardedQuotient |= 1UL;
        }

        ulong rounded = RoundGuardedQuotientToEven(
            guardedQuotient,
            (numeratorHigh | numeratorLow) != 0UL,
            out _);
        return new Fixed64((long)rounded);
    }

    /// <summary>
    /// Computes the ratio of two non-negative wide integers, returning a Fixed64 value in the unit interval [0, 1].
    /// </summary>
    internal static Fixed64 GetUnitIntervalRatio(
        ReadOnlySpan<ulong> numerator,
        ReadOnlySpan<ulong> denominator)
    {
        int length = Math.Max(numerator.Length, denominator.Length) + 1;
        Span<ulong> numeratorMagnitude = stackalloc ulong[length];
        Span<ulong> remainder = stackalloc ulong[length];
        Span<ulong> denominatorMagnitude = stackalloc ulong[length];
        numeratorMagnitude.Clear();
        remainder.Clear();
        denominatorMagnitude.Clear();
        numerator.CopyTo(numeratorMagnitude);
        denominator.CopyTo(denominatorMagnitude);
        ShiftLeftMagnitude(
            numeratorMagnitude,
            FixedMath.SHIFT_AMOUNT_I,
            remainder);
        _ = TryGetSignedRawRatioCore(
            remainder,
            denominatorMagnitude,
            negative: false,
            roundToEven: true,
            out Fixed64 result);
        return result;
    }

    /// <summary>
    /// Converts an arbitrary nonzero signed wide ratio to Q32.32 with
    /// round-half-to-even and signed saturation.
    /// </summary>
    internal static Fixed64 GetSignedRatio(Signed192 numerator, Signed192 denominator)
    {
        int numeratorSign = numerator.Sign;
        int denominatorSign = denominator.Sign;
        if (numeratorSign == 0)
            return Zero;
        if (denominatorSign == 0)
            return numeratorSign < 0 ? MinValue : MaxValue;

        bool negative = numeratorSign != denominatorSign;
        WideArithmetic.GetMagnitude(numerator, out ulong numeratorHigh, out ulong numeratorMiddle, out ulong numeratorLow);
        WideArithmetic.GetMagnitude(denominator, out ulong denominatorHigh, out ulong denominatorMiddle, out ulong denominatorLow);

        if (!negative)
        {
            int comparison = WideArithmetic.CompareUnsigned(
                numeratorHigh, numeratorMiddle, numeratorLow,
                denominatorHigh, denominatorMiddle, denominatorLow);
            if (comparison <= 0)
            {
                return comparison == 0
                    ? One
                    : GetUnitIntervalRatio(
                        numeratorHigh, numeratorMiddle, numeratorLow,
                        denominatorHigh, denominatorMiddle, denominatorLow);
            }
        }

        int numeratorBits = WideArithmetic.GetBitLength(numeratorHigh, numeratorMiddle, numeratorLow);
        int denominatorBits = WideArithmetic.GetBitLength(denominatorHigh, denominatorMiddle, denominatorLow);
        int integerShift = numeratorBits - denominatorBits;
        if (integerShift >= 31)
        {
            if (integerShift > 31)
                return negative ? MinValue : MaxValue;

            WideArithmetic.ShiftLeft(
                denominatorHigh,
                denominatorMiddle,
                denominatorLow,
                31,
                out ulong limitHigh,
                out ulong limitMiddle,
                out ulong limitLow);
            int limitComparison = WideArithmetic.CompareUnsigned(
                numeratorHigh, numeratorMiddle, numeratorLow,
                limitHigh, limitMiddle, limitLow);
            if (limitComparison > 0 || (limitComparison == 0 && !negative))
                return negative ? MinValue : MaxValue;
            if (limitComparison == 0)
                return MinValue;
        }

        ulong guardedQuotient = 0UL;
        if (integerShift >= 0)
        {
            WideArithmetic.ShiftLeft(
                denominatorHigh,
                denominatorMiddle,
                denominatorLow,
                integerShift,
                out ulong shiftedHigh,
                out ulong shiftedMiddle,
                out ulong shiftedLow);
            for (int bit = integerShift; bit >= 0; bit--)
            {
                if (WideArithmetic.CompareUnsigned(
                    numeratorHigh, numeratorMiddle, numeratorLow,
                    shiftedHigh, shiftedMiddle, shiftedLow) >= 0)
                {
                    WideArithmetic.SubtractUnsigned(
                        ref numeratorHigh,
                        ref numeratorMiddle,
                        ref numeratorLow,
                        shiftedHigh,
                        shiftedMiddle,
                        shiftedLow);
                    guardedQuotient |= 1UL << bit;
                }

                WideArithmetic.ShiftRightOne(ref shiftedHigh, ref shiftedMiddle, ref shiftedLow);
            }
        }

        for (int bit = FixedMath.SHIFT_AMOUNT_I; bit >= 0; bit--)
        {
            WideArithmetic.ShiftLeftOne(ref numeratorHigh, ref numeratorMiddle, ref numeratorLow);
            guardedQuotient <<= 1;
            if (WideArithmetic.CompareUnsigned(
                numeratorHigh, numeratorMiddle, numeratorLow,
                denominatorHigh, denominatorMiddle, denominatorLow) < 0)
            {
                continue;
            }

            WideArithmetic.SubtractUnsigned(
                ref numeratorHigh,
                ref numeratorMiddle,
                ref numeratorLow,
                denominatorHigh,
                denominatorMiddle,
                denominatorLow);
            guardedQuotient |= 1UL;
        }

        ulong magnitude = RoundGuardedQuotientToEven(
            guardedQuotient,
            (numeratorHigh | numeratorMiddle | numeratorLow) != 0UL,
            out bool roundedOverflow);
        ulong limit = negative ? 1UL << 63 : (ulong)long.MaxValue;
        if (roundedOverflow || magnitude > limit)
            return negative ? MinValue : MaxValue;

        long raw = negative ? unchecked(-(long)magnitude) : (long)magnitude;
        return new Fixed64(raw);
    }

    /// <summary>
    /// Converts an arbitrary nonzero signed five-word ratio to Q32.32 with
    /// round-half-to-even and signed saturation.
    /// </summary>
    internal static Fixed64 GetSignedRatio(Signed320 numerator, Signed320 denominator)
    {
        if (Signed192.TryNarrowSigned(numerator, out Signed192 narrowNumerator)
            && Signed192.TryNarrowSigned(denominator, out Signed192 narrowDenominator))
        {
            return GetSignedRatio(narrowNumerator, narrowDenominator);
        }

        int numeratorSign = numerator.Sign;
        int denominatorSign = denominator.Sign;
        if (numeratorSign == 0)
            return Zero;
        if (denominatorSign == 0)
            return numeratorSign < 0 ? MinValue : MaxValue;

        bool negative = numeratorSign != denominatorSign;
        WideArithmetic.GetMagnitude(
            numerator,
            out ulong numeratorWord4,
            out ulong numeratorWord3,
            out ulong numeratorWord2,
            out ulong numeratorWord1,
            out ulong numeratorWord0);
        WideArithmetic.GetMagnitude(
            denominator,
            out ulong denominatorWord4,
            out ulong denominatorWord3,
            out ulong denominatorWord2,
            out ulong denominatorWord1,
            out ulong denominatorWord0);

        int comparison = WideArithmetic.CompareUnsigned(
            numeratorWord4,
            numeratorWord3,
            numeratorWord2,
            numeratorWord1,
            numeratorWord0,
            denominatorWord4,
            denominatorWord3,
            denominatorWord2,
            denominatorWord1,
            denominatorWord0);
        if (comparison <= 0)
        {
            if (comparison == 0)
                return negative ? -One : One;

            Fixed64 unitRatio = GetUnitIntervalRatio(
                numeratorWord4,
                numeratorWord3,
                numeratorWord2,
                numeratorWord1,
                numeratorWord0,
                denominatorWord4,
                denominatorWord3,
                denominatorWord2,
                denominatorWord1,
                denominatorWord0);
            return negative ? -unitRatio : unitRatio;
        }

        int numeratorBits = WideArithmetic.GetBitLength(
            numeratorWord4, numeratorWord3, numeratorWord2, numeratorWord1, numeratorWord0);
        int denominatorBits = WideArithmetic.GetBitLength(
            denominatorWord4, denominatorWord3, denominatorWord2, denominatorWord1, denominatorWord0);
        int integerShift = numeratorBits - denominatorBits;
        if (integerShift >= 31)
        {
            if (integerShift > 31)
                return negative ? MinValue : MaxValue;

            WideArithmetic.ShiftLeft(
                denominatorWord4,
                denominatorWord3,
                denominatorWord2,
                denominatorWord1,
                denominatorWord0,
                31,
                out ulong limitWord4,
                out ulong limitWord3,
                out ulong limitWord2,
                out ulong limitWord1,
                out ulong limitWord0);
            int limitComparison = WideArithmetic.CompareUnsigned(
                numeratorWord4,
                numeratorWord3,
                numeratorWord2,
                numeratorWord1,
                numeratorWord0,
                limitWord4,
                limitWord3,
                limitWord2,
                limitWord1,
                limitWord0);
            if (limitComparison > 0 || (limitComparison == 0 && !negative))
                return negative ? MinValue : MaxValue;
            if (limitComparison == 0)
                return MinValue;
        }

        ulong guardedQuotient = 0UL;
        WideArithmetic.ShiftLeft(
            denominatorWord4,
            denominatorWord3,
            denominatorWord2,
            denominatorWord1,
            denominatorWord0,
            integerShift,
            out ulong shiftedWord4,
            out ulong shiftedWord3,
            out ulong shiftedWord2,
            out ulong shiftedWord1,
            out ulong shiftedWord0);
        for (int bit = integerShift; bit >= 0; bit--)
        {
            if (WideArithmetic.CompareUnsigned(
                numeratorWord4,
                numeratorWord3,
                numeratorWord2,
                numeratorWord1,
                numeratorWord0,
                shiftedWord4,
                shiftedWord3,
                shiftedWord2,
                shiftedWord1,
                shiftedWord0) >= 0)
            {
                WideArithmetic.SubtractUnsigned(
                    ref numeratorWord4,
                    ref numeratorWord3,
                    ref numeratorWord2,
                    ref numeratorWord1,
                    ref numeratorWord0,
                    shiftedWord4,
                    shiftedWord3,
                    shiftedWord2,
                    shiftedWord1,
                    shiftedWord0);
                guardedQuotient |= 1UL << bit;
            }

            WideArithmetic.ShiftRightOne(
                ref shiftedWord4,
                ref shiftedWord3,
                ref shiftedWord2,
                ref shiftedWord1,
                ref shiftedWord0);
        }

        for (int bit = FixedMath.SHIFT_AMOUNT_I; bit >= 0; bit--)
        {
            WideArithmetic.ShiftLeftOne(
                ref numeratorWord4,
                ref numeratorWord3,
                ref numeratorWord2,
                ref numeratorWord1,
                ref numeratorWord0);
            guardedQuotient <<= 1;
            if (WideArithmetic.CompareUnsigned(
                numeratorWord4,
                numeratorWord3,
                numeratorWord2,
                numeratorWord1,
                numeratorWord0,
                denominatorWord4,
                denominatorWord3,
                denominatorWord2,
                denominatorWord1,
                denominatorWord0) < 0)
            {
                continue;
            }

            WideArithmetic.SubtractUnsigned(
                ref numeratorWord4,
                ref numeratorWord3,
                ref numeratorWord2,
                ref numeratorWord1,
                ref numeratorWord0,
                denominatorWord4,
                denominatorWord3,
                denominatorWord2,
                denominatorWord1,
                denominatorWord0);
            guardedQuotient |= 1UL;
        }

        ulong magnitude = RoundGuardedQuotientToEven(
            guardedQuotient,
            (numeratorWord4 | numeratorWord3 | numeratorWord2 | numeratorWord1 | numeratorWord0) != 0UL,
            out bool roundedOverflow);
        ulong limit = negative ? 1UL << 63 : (ulong)long.MaxValue;
        if (roundedOverflow || magnitude > limit)
            return negative ? MinValue : MaxValue;

        long raw = negative ? unchecked(-(long)magnitude) : (long)magnitude;
        return new Fixed64(raw);
    }

    /// <summary>
    /// Converts an arbitrary signed five-word ratio to Q32.32 when its final
    /// round-half-to-even result is representable.
    /// </summary>
    internal static bool TryGetSignedRatio(
        Signed320 numerator,
        Signed320 denominator,
        out Fixed64 result)
    {
        int numeratorSign = numerator.Sign;
        int denominatorSign = denominator.Sign;
        if (denominatorSign == 0)
        {
            result = default;
            return false;
        }
        if (numeratorSign == 0)
        {
            result = Zero;
            return true;
        }

        WideArithmetic.GetMagnitude(
            numerator,
            out ulong numeratorWord4,
            out ulong numeratorWord3,
            out ulong numeratorWord2,
            out ulong numeratorWord1,
            out ulong numeratorWord0);
        WideArithmetic.GetMagnitude(
            denominator,
            out ulong denominatorWord4,
            out ulong denominatorWord3,
            out ulong denominatorWord2,
            out ulong denominatorWord1,
            out ulong denominatorWord0);
        Signed320 numeratorMagnitude = new(
            numeratorWord4, numeratorWord3, numeratorWord2, numeratorWord1, numeratorWord0);
        Signed320 denominatorMagnitude = new(
            denominatorWord4, denominatorWord3, denominatorWord2, denominatorWord1, denominatorWord0);
        Signed320 twiceScale = Signed320.ExtendValue(
            Signed192.Signed(FixedMath.ONE_L * 2L));
        bool negative = numeratorSign != denominatorSign;
        Signed192 roundedLimit = negative
            ? new Signed192(0UL, 1UL, 1UL)             // 2 * 2^63 + 1
            : new Signed192(0UL, 0UL, ulong.MaxValue); // 2 * (2^63 - 1) + 1
        Signed576 scaledNumerator = GetNonNegativeProduct(numeratorMagnitude, twiceScale);
        Signed576 scaledLimit = GetNonNegativeProduct(
            denominatorMagnitude,
            Signed320.ExtendValue(roundedLimit));
        int comparison = WideArithmetic.CompareNonNegative(scaledNumerator, scaledLimit);
        if (comparison > 0 || (!negative && comparison == 0))
        {
            result = default;
            return false;
        }

        result = GetSignedRatio(numerator, denominator);
        return true;
    }

    private static Signed576 GetNonNegativeProduct(Signed320 leftMagnitude, Signed320 right)
    {
        Signed576 product = WideArithmetic.MultiplySigned320(leftMagnitude, right);
        return product.Sign < 0
            ? WideArithmetic.SubtractSigned576(default, product)
            : product;
    }

    /// <summary>
    /// Converts an exact signed five-word ratio with a positive single-word
    /// denominator to a raw integer with round-half-to-even.
    /// </summary>
    /// <remarks>
    /// The caller owns the invariant that the quotient is representable.
    /// </remarks>
    internal static Fixed64 GetSignedRawRatio(Signed320 numerator, Signed192 denominator)
    {
        int numeratorSign = numerator.Sign;
        if (numeratorSign == 0)
            return Zero;

        WideArithmetic.GetMagnitude(
            denominator,
            out _,
            out _,
            out ulong denominatorLow);
        WideArithmetic.GetMagnitude(
            numerator,
            out _,
            out _,
            out _,
            out ulong word1,
            out ulong word0);

        ulong quotient = Divide128By64(
            word1,
            word0,
            denominatorLow,
            out ulong remainder);
        int midpointComparison = remainder.CompareTo(denominatorLow - remainder);
        if (midpointComparison > 0 || (midpointComparison == 0 && (quotient & 1UL) != 0UL))
            quotient++;

        long raw = numeratorSign < 0 ? unchecked(-(long)quotient) : (long)quotient;
        return new Fixed64(raw);
    }

    /// <summary>
    /// Converts an exact signed nine-word ratio directly to a raw integer with
    /// round-half-to-even. Unlike <c>GetSignedRatio</c>, this method does not
    /// apply an additional Q32.32 scale.
    /// </summary>
    internal static bool TryGetSignedRawRatio(
        Signed576 numerator,
        Signed576 denominator,
        out Fixed64 result)
    {
        int numeratorSign = numerator.Sign;
        int denominatorSign = denominator.Sign;
        if (denominatorSign == 0)
        {
            result = default;
            return false;
        }
        if (numeratorSign == 0)
        {
            result = Zero;
            return true;
        }

        bool negative = numeratorSign != denominatorSign;
        if (Signed192.TryNarrowSigned(numerator, out Signed192 numerator192)
            && Signed192.TryNarrowSigned(denominator, out Signed192 denominator192))
        {
            Span<ulong> narrowRemainder = stackalloc ulong[3];
            Span<ulong> narrowDenominator = stackalloc ulong[3];
            WideArithmetic.GetMagnitude(
                numerator192,
                out narrowRemainder[2], out narrowRemainder[1], out narrowRemainder[0]);
            WideArithmetic.GetMagnitude(
                denominator192,
                out narrowDenominator[2], out narrowDenominator[1], out narrowDenominator[0]);
            return TryGetSignedRawRatioCore(
                narrowRemainder,
                narrowDenominator,
                negative,
                roundToEven: true,
                out result);
        }
        if (Signed320.TryNarrowSigned(numerator, out Signed320 numerator320)
            && Signed320.TryNarrowSigned(denominator, out Signed320 denominator320))
        {
            Span<ulong> mediumRemainder = stackalloc ulong[5];
            Span<ulong> mediumDenominator = stackalloc ulong[5];
            WideArithmetic.GetMagnitude(
                numerator320,
                out mediumRemainder[4], out mediumRemainder[3], out mediumRemainder[2],
                out mediumRemainder[1], out mediumRemainder[0]);
            WideArithmetic.GetMagnitude(
                denominator320,
                out mediumDenominator[4], out mediumDenominator[3], out mediumDenominator[2],
                out mediumDenominator[1], out mediumDenominator[0]);
            return TryGetSignedRawRatioCore(
                mediumRemainder,
                mediumDenominator,
                negative,
                roundToEven: true,
                out result);
        }

        Span<ulong> remainder = stackalloc ulong[9];
        Span<ulong> denominatorMagnitude = stackalloc ulong[9];
        WideArithmetic.GetMagnitude(numerator, remainder);
        WideArithmetic.GetMagnitude(denominator, denominatorMagnitude);
        return TryGetSignedRawRatioCore(
            remainder,
            denominatorMagnitude,
            negative,
            roundToEven: true,
            out result);
    }

    /// <summary>
    /// Converts an exact signed eleven-word ratio directly to a raw integer
    /// with round-half-to-even.
    /// </summary>
    internal static bool TryGetSignedRawRatio(
        Signed704 numerator,
        Signed704 denominator,
        out Fixed64 result)
    {
        int numeratorSign = numerator.Sign;
        int denominatorSign = denominator.Sign;
        if (denominatorSign == 0)
        {
            result = default;
            return false;
        }
        if (numeratorSign == 0)
        {
            result = Zero;
            return true;
        }

        Span<ulong> remainder = stackalloc ulong[11];
        Span<ulong> denominatorMagnitude = stackalloc ulong[11];
        WideArithmetic.GetMagnitude(numerator, remainder);
        WideArithmetic.GetMagnitude(denominator, denominatorMagnitude);
        return TryGetSignedRawRatioCore(
            remainder,
            denominatorMagnitude,
            numeratorSign != denominatorSign,
            roundToEven: true,
            out result);
    }

    internal static bool TryGetSignedRawRatio(
        Signed832 numerator,
        Signed832 denominator,
        int numeratorLeftShift,
        out Fixed64 result)
    {
        int numeratorSign = numerator.Sign;
        int denominatorSign = denominator.Sign;
        if (denominatorSign == 0)
        {
            result = default;
            return false;
        }
        if (numeratorSign == 0)
        {
            result = Zero;
            return true;
        }

        Span<ulong> numeratorMagnitude = stackalloc ulong[14];
        Span<ulong> denominatorMagnitude = stackalloc ulong[14];
        Span<ulong> shiftedNumerator = stackalloc ulong[14];
        numeratorMagnitude.Clear();
        denominatorMagnitude.Clear();
        shiftedNumerator.Clear();
        WideArithmetic.GetMagnitude(numerator, numeratorMagnitude);
        WideArithmetic.GetMagnitude(denominator, denominatorMagnitude);
        ShiftLeftMagnitude(
            numeratorMagnitude,
            numeratorLeftShift,
            shiftedNumerator);
        return TryGetSignedRawRatioCore(
            shiftedNumerator,
            denominatorMagnitude,
            numeratorSign != denominatorSign,
            roundToEven: true,
            out result);
    }

    internal static Fixed64 GetNonNegativeRawRatioFloor(
        Signed704 numerator,
        Signed704 denominator)
    {
        if (numerator.Sign <= 0)
            return Zero;

        Span<ulong> remainder = stackalloc ulong[11];
        Span<ulong> denominatorMagnitude = stackalloc ulong[11];
        WideArithmetic.GetMagnitude(numerator, remainder);
        WideArithmetic.GetMagnitude(denominator, denominatorMagnitude);
        return TryGetSignedRawRatioCore(
            remainder,
            denominatorMagnitude,
            negative: false,
            roundToEven: false,
            out Fixed64 result)
            ? result
            : MaxValue;
    }

    internal static bool TryGetSignedRawRatio(
        ReadOnlySpan<ulong> numeratorMagnitude,
        ReadOnlySpan<ulong> denominatorMagnitude,
        bool negative,
        out Fixed64 result)
    {
        int denominatorLength =
            GetActiveMagnitudeLength(denominatorMagnitude);
        if (denominatorLength == 1
            && denominatorMagnitude[0] == 0UL)
        {
            result = default;
            return false;
        }

        int numeratorLength =
            GetActiveMagnitudeLength(numeratorMagnitude);
        if (numeratorLength == 1
            && numeratorMagnitude[0] == 0UL)
        {
            result = Zero;
            return true;
        }

        int length = Math.Max(numeratorLength, denominatorLength);
        Span<ulong> remainder = stackalloc ulong[length];
        Span<ulong> denominator = stackalloc ulong[length];
        remainder.Clear();
        denominator.Clear();
        numeratorMagnitude[..numeratorLength].CopyTo(remainder);
        denominatorMagnitude[..denominatorLength].CopyTo(denominator);
        return TryGetSignedRawRatioCore(
            remainder,
            denominator,
            negative,
            roundToEven: true,
            out result);
    }

    private static bool TryGetSignedRawRatioCore(
        Span<ulong> remainder,
        Span<ulong> denominatorMagnitude,
        bool negative,
        bool roundToEven,
        out Fixed64 result)
    {
        int remainderLength = GetActiveMagnitudeLength(remainder);
        int denominatorLength = GetActiveMagnitudeLength(denominatorMagnitude);
        int activeLength = Math.Max(remainderLength, denominatorLength);
        Span<ulong> activeRemainder = remainder[..activeLength];
        Span<ulong> activeDenominator = denominatorMagnitude[..activeLength];
        int quotientBit = GetMagnitudeBitLength(remainder[..remainderLength])
            - GetMagnitudeBitLength(denominatorMagnitude[..denominatorLength]);
        if (quotientBit > 63)
        {
            result = default;
            return false;
        }

        ulong quotient = 0UL;
        if (quotientBit >= 0)
        {
            Span<ulong> shiftedDenominatorStorage =
                stackalloc ulong[denominatorMagnitude.Length];
            Span<ulong> shiftedDenominator = shiftedDenominatorStorage[..activeLength];
            ShiftLeftMagnitude(activeDenominator, quotientBit, shiftedDenominator);
            for (int bit = quotientBit; bit >= 0; bit--)
            {
                if (CompareMagnitude(activeRemainder, shiftedDenominator) >= 0)
                {
                    SubtractMagnitude(activeRemainder, shiftedDenominator);
                    quotient |= 1UL << bit;
                }

                ShiftRightOne(shiftedDenominator);
            }
        }

        int midpointComparison = -1;
        if (roundToEven)
        {
            Span<ulong> denominatorMinusRemainderStorage =
                stackalloc ulong[denominatorMagnitude.Length];
            Span<ulong> denominatorMinusRemainder =
                denominatorMinusRemainderStorage[..activeLength];
            activeDenominator.CopyTo(denominatorMinusRemainder);
            SubtractMagnitude(
                denominatorMinusRemainder,
                activeRemainder);
            midpointComparison = CompareMagnitude(
                activeRemainder,
                denominatorMinusRemainder);
        }
        return TryCreateRawRatioResult(
            quotient,
            midpointComparison,
            negative,
            out result);
    }

    private static bool TryCreateRawRatioResult(
        ulong quotient,
        int midpointComparison,
        bool negative,
        out Fixed64 result)
    {
        if (midpointComparison > 0 || (midpointComparison == 0 && (quotient & 1UL) != 0UL))
        {
            quotient++;
            if (quotient == 0UL)
            {
                result = default;
                return false;
            }
        }

        ulong limit = negative ? 1UL << 63 : (ulong)long.MaxValue;
        if (quotient > limit)
        {
            result = default;
            return false;
        }

        long raw = negative ? unchecked(-(long)quotient) : (long)quotient;
        result = new Fixed64(raw);
        return true;
    }

    private static int GetMagnitudeBitLength(ReadOnlySpan<ulong> value)
    {
        int index = value.Length - 1;
        return (index * 64) + 64 - CountLeadingZeroes(value[index]);
    }

    private static int GetActiveMagnitudeLength(ReadOnlySpan<ulong> value)
    {
        int length = value.Length;
        while (length > 1 && value[length - 1] == 0UL)
            length--;
        return length;
    }

    private static int CompareMagnitude(ReadOnlySpan<ulong> left, ReadOnlySpan<ulong> right)
    {
        for (int index = left.Length - 1; index >= 0; index--)
        {
            if (left[index] != right[index])
                return left[index] < right[index] ? -1 : 1;
        }

        return 0;
    }

    private static void SubtractMagnitude(Span<ulong> value, ReadOnlySpan<ulong> subtract)
    {
        ulong borrow = 0UL;
        for (int index = 0; index < value.Length; index++)
        {
            ulong subtrahend = unchecked(subtract[index] + borrow);
            ulong nextBorrow = subtrahend < subtract[index] || value[index] < subtrahend
                ? 1UL
                : 0UL;
            value[index] = unchecked(value[index] - subtrahend);
            borrow = nextBorrow;
        }
    }

    private static void ShiftLeftMagnitude(
        ReadOnlySpan<ulong> source,
        int bits,
        Span<ulong> destination)
    {
        if (bits == 0)
        {
            source.CopyTo(destination);
            return;
        }

        destination[0] = source[0] << bits;
        for (int index = 1; index < destination.Length; index++)
            destination[index] = (source[index] << bits) | (source[index - 1] >> (64 - bits));
    }

    private static void ShiftRightOne(Span<ulong> value)
    {
        ulong carry = 0UL;
        for (int index = value.Length - 1; index >= 0; index--)
        {
            ulong nextCarry = value[index] << 63;
            value[index] = (value[index] >> 1) | carry;
            carry = nextCarry;
        }
    }
}
