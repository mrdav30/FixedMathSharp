//=======================================================================
// Fixed64.WideGeometry.cs
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
    /// Returns the exact dot product of two two-dimensional endpoint differences.
    /// </summary>
    internal static Signed192 GetDifferenceDotProduct2D(
        Fixed64 leftEndX,
        Fixed64 leftStartX,
        Fixed64 leftEndY,
        Fixed64 leftStartY,
        Fixed64 rightEndX,
        Fixed64 rightStartX,
        Fixed64 rightEndY,
        Fixed64 rightStartY)
    {
        ulong high = 0UL;
        ulong middle = 0UL;
        ulong low = 0UL;
        AccumulateDifferenceProduct(
            leftEndX.m_rawValue,
            leftStartX.m_rawValue,
            rightEndX.m_rawValue,
            rightStartX.m_rawValue,
            ref high,
            ref middle,
            ref low);
        AccumulateDifferenceProduct(
            leftEndY.m_rawValue,
            leftStartY.m_rawValue,
            rightEndY.m_rawValue,
            rightStartY.m_rawValue,
            ref high,
            ref middle,
            ref low);
        return new Signed192(high, middle, low);
    }

    /// <summary>
    /// Returns the exact 2D cross product of two endpoint differences.
    /// </summary>
    internal static Signed192 GetDifferenceCrossProduct2D(
        Fixed64 leftEndX,
        Fixed64 leftStartX,
        Fixed64 leftEndY,
        Fixed64 leftStartY,
        Fixed64 rightEndX,
        Fixed64 rightStartX,
        Fixed64 rightEndY,
        Fixed64 rightStartY)
    {
        ulong high = 0UL;
        ulong middle = 0UL;
        ulong low = 0UL;
        AccumulateDifferenceProduct(
            leftEndX.m_rawValue,
            leftStartX.m_rawValue,
            rightEndY.m_rawValue,
            rightStartY.m_rawValue,
            ref high,
            ref middle,
            ref low);
        AccumulateDifferenceProduct(
            leftEndY.m_rawValue,
            leftStartY.m_rawValue,
            rightStartX.m_rawValue,
            rightEndX.m_rawValue,
            ref high,
            ref middle,
            ref low);
        return new Signed192(high, middle, low);
    }

    /// <summary>
    /// Compares unsigned magnitudes of signed wide values.
    /// </summary>
    internal static int CompareMagnitude(Signed192 left, Signed192 right)
    {
        GetMagnitude(left, out ulong leftHigh, out ulong leftMiddle, out ulong leftLow);
        GetMagnitude(right, out ulong rightHigh, out ulong rightMiddle, out ulong rightLow);
        return CompareUnsigned(
            leftHigh,
            leftMiddle,
            leftLow,
            rightHigh,
            rightMiddle,
            rightLow);
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

        GetMagnitude(numerator, out ulong numeratorHigh, out ulong numeratorMiddle, out ulong numeratorLow);
        GetMagnitude(denominator, out ulong denominatorHigh, out ulong denominatorMiddle, out ulong denominatorLow);
        int comparison = CompareUnsigned(
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

        ulong guardedQuotient = 0UL;
        for (int bit = FixedMath.SHIFT_AMOUNT_I; bit >= 0; bit--)
        {
            ShiftLeftOne(ref numeratorHigh, ref numeratorMiddle, ref numeratorLow);
            guardedQuotient <<= 1;
            if (CompareUnsigned(
                numeratorHigh,
                numeratorMiddle,
                numeratorLow,
                denominatorHigh,
                denominatorMiddle,
                denominatorLow) < 0)
            {
                continue;
            }

            SubtractUnsigned(
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
        result = new Fixed64((long)rounded);
        return true;
    }

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    private static void GetMagnitude(
        Signed192 value,
        out ulong high,
        out ulong middle,
        out ulong low)
    {
        high = value.High;
        middle = value.Middle;
        low = value.Low;
        if (value.Sign >= 0)
            return;

        low = unchecked(~low + 1UL);
        middle = unchecked(~middle + (low == 0UL ? 1UL : 0UL));
        high = unchecked(~high + (middle == 0UL && low == 0UL ? 1UL : 0UL));
    }

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    private static int CompareUnsigned(
        ulong leftHigh,
        ulong leftMiddle,
        ulong leftLow,
        ulong rightHigh,
        ulong rightMiddle,
        ulong rightLow)
    {
        if (leftHigh != rightHigh)
            return leftHigh < rightHigh ? -1 : 1;
        if (leftMiddle != rightMiddle)
            return leftMiddle < rightMiddle ? -1 : 1;
        if (leftLow != rightLow)
            return leftLow < rightLow ? -1 : 1;
        return 0;
    }

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    private static void ShiftLeftOne(ref ulong high, ref ulong middle, ref ulong low)
    {
        high = (high << 1) | (middle >> 63);
        middle = (middle << 1) | (low >> 63);
        low <<= 1;
    }

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    private static void SubtractUnsigned(
        ref ulong high,
        ref ulong middle,
        ref ulong low,
        ulong subtractHigh,
        ulong subtractMiddle,
        ulong subtractLow)
    {
        ulong originalLow = low;
        low -= subtractLow;
        ulong borrow = originalLow < subtractLow ? 1UL : 0UL;

        ulong middleSubtrahend = subtractMiddle + borrow;
        ulong middleOverflow = middleSubtrahend < subtractMiddle ? 1UL : 0UL;
        ulong originalMiddle = middle;
        middle -= middleSubtrahend;
        borrow = middleOverflow | (originalMiddle < middleSubtrahend ? 1UL : 0UL);
        high -= subtractHigh + borrow;
    }
}
