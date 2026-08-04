//=======================================================================
// Fixed64.WideConversion.cs
//=======================================================================
// MIT License, Copyright (c) 2024–present David Oravsky (mrdav30)
// See LICENSE file in the project root for full license information.
//=======================================================================

using System.Runtime.CompilerServices;
using FixedMathSharp.Geometry;

namespace FixedMathSharp;

/// <content>
/// Wide (64x64-bit to 128-bit) arithmetic helpers for high-precision
/// interpolation and projection operations on <see cref="Fixed64"/> values.
/// </content>
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

    internal static bool TryAddProducts(
        Fixed64 firstLeft,
        Fixed64 firstRight,
        Fixed64 secondLeft,
        Fixed64 secondRight,
        out Fixed64 result) =>
        TryRoundProductCombination(
            GetExactRawProduct(firstLeft, firstRight),
            GetExactRawProduct(secondLeft, secondRight),
            subtract: false,
            out result);

    internal static bool TryAddProducts(
        Fixed64 firstLeft,
        Fixed64 firstRight,
        Fixed64 secondLeft,
        Fixed64 secondRight,
        Fixed64 thirdLeft,
        Fixed64 thirdRight,
        out Fixed64 result)
    {
        Signed192 numerator = WideArithmetic.AddSigned192(
            WideArithmetic.AddSigned192(
                GetExactRawProduct(firstLeft, firstRight),
                GetExactRawProduct(secondLeft, secondRight)),
            GetExactRawProduct(thirdLeft, thirdRight));
        return TryRoundProductCombination(
            numerator,
            default,
            subtract: false,
            out result);
    }

    internal static bool TryAddScaledProducts(
        Fixed64 firstLeft,
        Fixed64 firstRight,
        Fixed64 secondLeft,
        Fixed64 secondRight,
        Fixed64 thirdLeft,
        Fixed64 thirdRight,
        Fixed64 resultScale,
        out Fixed64 result)
    {
        Signed192 products = WideArithmetic.AddSigned192(
            WideArithmetic.AddSigned192(
                GetExactRawProduct(firstLeft, firstRight),
                GetExactRawProduct(secondLeft, secondRight)),
            GetExactRawProduct(thirdLeft, thirdRight));
        Signed320 scaled = WideArithmetic.MultiplySigned192(
            products,
            Signed192.Raw(resultScale));
        bool negative = scaled.Sign < 0;
        WideArithmetic.GetMagnitude(
            scaled,
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

        const ulong guardMask = 1UL << 63;
        result = RoundAndApplySign(
            word1,
            (word0 & guardMask) != 0UL,
            (word0 & (guardMask - 1UL)) != 0UL,
            negative,
            out bool representable);
        if (representable)
            return true;

        result = default;
        return false;
    }

    internal static bool TrySubtractProducts(
        Fixed64 firstLeft,
        Fixed64 firstRight,
        Fixed64 secondLeft,
        Fixed64 secondRight,
        out Fixed64 result) =>
        TryRoundProductCombination(
            GetExactRawProduct(firstLeft, firstRight),
            GetExactRawProduct(secondLeft, secondRight),
            subtract: true,
            out result);

    private static bool TryRoundProductCombination(
        Signed192 first,
        Signed192 second,
        bool subtract,
        out Fixed64 result)
    {
        Signed192 numerator = subtract
            ? WideArithmetic.SubtractSigned192(first, second)
            : WideArithmetic.AddSigned192(first, second);
        bool negative = numerator.Sign < 0;
        WideArithmetic.GetMagnitude(
            numerator,
            out _,
            out ulong middle,
            out ulong low);
        if ((middle >> FixedMath.SHIFT_AMOUNT_I) != 0UL)
        {
            result = default;
            return false;
        }

        ulong quotient = (middle << FixedMath.SHIFT_AMOUNT_I)
            | (low >> FixedMath.SHIFT_AMOUNT_I);
        ulong guardMask = 1UL << (FixedMath.SHIFT_AMOUNT_I - 1);
        bool guard = (low & guardMask) != 0UL;
        bool sticky = (low & (guardMask - 1UL)) != 0UL;
        result = RoundAndApplySign(
            quotient,
            guard,
            sticky,
            negative,
            out bool representable);
        if (representable)
            return true;

        result = default;
        return false;
    }
}
