//=======================================================================
// Fixed64.WideConversion.cs
//=======================================================================
// MIT License, Copyright (c) 2024–present David Oravsky (mrdav30)
// See LICENSE file in the project root for full license information.
//=======================================================================

using System.Runtime.CompilerServices;

namespace FixedMathSharp;

public partial struct Fixed64
{
    /// <summary>
    /// Interpolates across the complete raw endpoint domain with one final
    /// round-half-to-even conversion.
    /// </summary>
    internal static Fixed64 LerpFullDomain(Fixed64 from, Fixed64 to, Fixed64 amount)
    {
        long amountRaw = amount.m_rawValue;
        if (amountRaw <= 0L)
            return from;
        if (amountRaw >= FixedMath.ONE_L)
            return to;

        long fromRaw = from.m_rawValue;
        long toRaw = to.m_rawValue;
        bool negativeDifference = toRaw < fromRaw;
        ulong differenceMagnitude = negativeDifference
            ? unchecked((ulong)fromRaw - (ulong)toRaw)
            : unchecked((ulong)toRaw - (ulong)fromRaw);

        Multiply64To128(
            differenceMagnitude,
            (ulong)amountRaw,
            out ulong productHigh,
            out ulong productLow);

        ulong interpolatedMagnitude = (productHigh << 32) | (productLow >> 32);
        ulong rawResult = negativeDifference
            ? unchecked((ulong)fromRaw - interpolatedMagnitude)
            : unchecked((ulong)fromRaw + interpolatedMagnitude);
        ulong remainder = productLow & uint.MaxValue;
        const ulong HalfRawUnit = 1UL << 31;
        if (remainder > HalfRawUnit || (remainder == HalfRawUnit && (rawResult & 1UL) != 0UL))
            rawResult = negativeDifference ? rawResult - 1UL : rawResult + 1UL;

        return new Fixed64(unchecked((long)rawResult));
    }

    /// <summary>
    /// Projects three component differences, clamps negative sums to zero, floors
    /// positive Q64.64 remainder, and saturates only the final Q32.32 result.
    /// </summary>
    internal static Fixed64 ProjectNonNegativeDifference(
        Fixed64 targetX,
        Fixed64 sourceX,
        Fixed64 directionX,
        Fixed64 targetY,
        Fixed64 sourceY,
        Fixed64 directionY,
        Fixed64 targetZ,
        Fixed64 sourceZ,
        Fixed64 directionZ)
    {
        WideGeometry.GetDifferenceProjectionWords(
            targetX,
            sourceX,
            directionX,
            targetY,
            sourceY,
            directionY,
            targetZ,
            sourceZ,
            directionZ,
            out ulong sumHigh,
            out ulong sumMiddle,
            out ulong sumLow);

        if ((sumHigh & (1UL << 63)) != 0UL || (sumHigh | sumMiddle | sumLow) == 0UL)
            return Zero;

        // After the Q64.64-to-Q32.32 shift, the low 32 bits of this word become
        // the result's high 32 bits. long.MaxValue >> 32 is therefore the largest
        // positive middle word that can still produce a representable raw result.
        ulong positiveRawHighLimit = (ulong)(long.MaxValue >> FixedMath.SHIFT_AMOUNT_I);
        if (sumHigh != 0UL || sumMiddle > positiveRawHighLimit)
            return MaxValue;

        long rawResult = unchecked((long)(
            (sumMiddle << FixedMath.SHIFT_AMOUNT_I)
            | (sumLow >> FixedMath.SHIFT_AMOUNT_I)));
        return new Fixed64(rawResult);
    }

    /// <summary>
    /// Converts an exact Q64.64 squared-distance sum to Q32.32 with one final
    /// round-half-to-even step and positive saturation.
    /// </summary>
    internal static Fixed64 RoundSquaredDistance(Signed192 value)
    {
        if (value.Sign <= 0)
            return Zero;

        ulong positiveRawHighLimit = (ulong)(long.MaxValue >> FixedMath.SHIFT_AMOUNT_I);
        if (value.High != 0UL || value.Middle > positiveRawHighLimit)
            return MaxValue;

        ulong raw = (value.Middle << FixedMath.SHIFT_AMOUNT_I)
            | (value.Low >> FixedMath.SHIFT_AMOUNT_I);
        if (raw >= long.MaxValue)
            return MaxValue;

        ulong remainder = value.Low & uint.MaxValue;
        const ulong HalfRawUnit = 1UL << (FixedMath.SHIFT_AMOUNT_I - 1);
        if (remainder > HalfRawUnit || (remainder == HalfRawUnit && (raw & 1UL) != 0UL))
            raw++;

        return new Fixed64((long)raw);
    }

    /// <summary>
    /// Evaluates one barycentric coordinate without saturating endpoint
    /// differences or weighted intermediates.
    /// </summary>
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    internal static Fixed64 BarycentricCoordinateFullDomain(
        Fixed64 coordA,
        Fixed64 coordB,
        Fixed64 coordC,
        Fixed64 weightB,
        Fixed64 weightC)
    {
        long baseRaw = coordA.m_rawValue;
        ulong extension = baseRaw < 0L ? ulong.MaxValue : 0UL;
        ulong high = extension;
        ulong middle = (extension << FixedMath.SHIFT_AMOUNT_I)
            | (unchecked((ulong)baseRaw) >> FixedMath.SHIFT_AMOUNT_I);
        ulong low = unchecked((ulong)baseRaw << FixedMath.SHIFT_AMOUNT_I);
        WideGeometry.AccumulateDifferenceProduct(
            coordB.m_rawValue,
            baseRaw,
            weightB.m_rawValue,
            0L,
            ref high,
            ref middle,
            ref low);
        WideGeometry.AccumulateDifferenceProduct(
            coordC.m_rawValue,
            baseRaw,
            weightC.m_rawValue,
            0L,
            ref high,
            ref middle,
            ref low);
        return RoundSignedToFixed(new Signed192(high, middle, low), FixedMath.SHIFT_AMOUNT_I);
    }

    /// <summary>
    /// Converts a signed wide value to Q32.32 with one final
    /// round-half-to-even step and signed saturation.
    /// </summary>
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    internal static Fixed64 RoundSignedToFixed(Signed192 value, int fractionalBits)
    {
        bool negative = value.Sign < 0;
        WideArithmetic.GetMagnitude(value, out ulong high, out ulong middle, out ulong low);
        if (high != 0UL || (middle >> fractionalBits) != 0UL)
            return negative ? MinValue : MaxValue;

        ulong magnitude = (middle << (64 - fractionalBits)) | (low >> fractionalBits);
        ulong limit = negative ? 1UL << 63 : (ulong)long.MaxValue;
        if (magnitude > limit)
            return negative ? MinValue : MaxValue;

        ulong remainderMask = (1UL << fractionalBits) - 1UL;
        ulong remainder = low & remainderMask;
        ulong half = 1UL << (fractionalBits - 1);
        if (remainder > half || (remainder == half && (magnitude & 1UL) != 0UL))
        {
            if (magnitude == limit)
                return negative ? MinValue : MaxValue;
            magnitude++;
        }

        long raw = negative ? unchecked(-(long)magnitude) : (long)magnitude;
        return new Fixed64(raw);
    }

    /// <summary>
    /// Converts an exact nonnegative square root to Q32.32 with one final
    /// round-half-to-even step and positive saturation.
    /// </summary>
    internal static Fixed64 RoundSquareRootToFixed(
        Signed192 root,
        Signed192 remainder,
        int fractionalBits)
    {
        WideArithmetic.GetMagnitude(root, out ulong high, out ulong middle, out ulong low);
        if (high != 0UL || (middle >> fractionalBits) != 0UL)
            return MaxValue;

        ulong magnitude = (middle << (64 - fractionalBits)) | (low >> fractionalBits);
        if (magnitude > (ulong)long.MaxValue)
            return MaxValue;

        ulong remainderMask = (1UL << fractionalBits) - 1UL;
        ulong discarded = low & remainderMask;
        ulong half = 1UL << (fractionalBits - 1);
        if (discarded > half
            || (discarded == half && (!remainder.IsZero || (magnitude & 1UL) != 0UL)))
        {
            if (magnitude == (ulong)long.MaxValue)
                return MaxValue;
            magnitude++;
        }

        return new Fixed64((long)magnitude);
    }

    /// <summary>
    /// Converts one exact cross component to a normalized Q32.32 component.
    /// </summary>
    /// <remarks>
    /// The caller supplies the component's exact square, the positive ceiling
    /// of the exact magnitude, and a nondegenerate squared magnitude produced
    /// from the same cross product.
    /// </remarks>
    internal static Fixed64 NormalizeWideComponent(
        Signed192 component,
        Signed320 componentSquare,
        Signed192 ceilingMagnitude,
        Signed320 squaredMagnitude)
    {
        int sign = component.Sign;
        if (sign == 0)
            return Zero;

        WideArithmetic.GetMagnitude(
            ceilingMagnitude,
            out ulong magnitudeHigh,
            out ulong magnitudeMiddle,
            out ulong magnitudeLow);
        int bitLength = WideArithmetic.GetBitLength(magnitudeHigh, magnitudeMiddle, magnitudeLow);
        int shift = bitLength > 63 ? bitLength - 63 : 0;
        ulong numerator = WideArithmetic.ShiftRightToUInt64(component, shift, out _);
        ulong denominator = WideArithmetic.ShiftRightToUInt64(ceilingMagnitude, shift, out bool discarded);
        if (discarded)
            denominator++;

        ulong candidate = (ulong)DivideMagnitude(numerator, denominator, false).m_rawValue;
        int midpointComparison = WideArithmetic.CompareNormalizedComponentToMidpoint(
            componentSquare,
            squaredMagnitude,
            candidate);
        if (midpointComparison > 0 || (midpointComparison == 0 && (candidate & 1UL) != 0UL))
            candidate++;

        long raw = (long)candidate;
        return new Fixed64(sign < 0 ? -raw : raw);
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
        if (WideArithmetic.TryNarrowSigned192(numerator, out Signed192 narrowNumerator)
            && WideArithmetic.TryNarrowSigned192(denominator, out Signed192 narrowDenominator))
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
        if (WideArithmetic.TryNarrowSigned192(numerator, out Signed192 narrowNumerator)
            && WideArithmetic.TryNarrowSigned192(denominator, out Signed192 narrowDenominator))
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
}
