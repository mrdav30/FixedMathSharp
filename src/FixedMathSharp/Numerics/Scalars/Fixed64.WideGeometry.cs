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
    /// Returns the exact dot product of two three-dimensional endpoint differences.
    /// </summary>
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    internal static Signed192 GetDifferenceDotProduct3D(
        Fixed64 leftEndX,
        Fixed64 leftStartX,
        Fixed64 leftEndY,
        Fixed64 leftStartY,
        Fixed64 leftEndZ,
        Fixed64 leftStartZ,
        Fixed64 rightEndX,
        Fixed64 rightStartX,
        Fixed64 rightEndY,
        Fixed64 rightStartY,
        Fixed64 rightEndZ,
        Fixed64 rightStartZ)
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
        AccumulateDifferenceProduct(
            leftEndZ.m_rawValue,
            leftStartZ.m_rawValue,
            rightEndZ.m_rawValue,
            rightStartZ.m_rawValue,
            ref high,
            ref middle,
            ref low);
        return new Signed192(high, middle, low);
    }

    /// <summary>
    /// Returns whether an exact squared direction rounds to zero in Q32.32.
    /// </summary>
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    internal static bool IsSquaredLengthDegenerate(Signed192 value) =>
        value.High == 0UL
        && value.Middle == 0UL
        && value.Low <= (1UL << (FixedMath.SHIFT_AMOUNT_I - 1));

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
    /// Adds exact three-word geometry values without scalar conversion.
    /// </summary>
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    internal static Signed192 AddSigned192(Signed192 left, Signed192 right)
    {
        ulong low = unchecked(left.Low + right.Low);
        ulong carry = low < left.Low ? 1UL : 0UL;
        ulong middle = unchecked(left.Middle + right.Middle + carry);
        carry = middle < left.Middle || (carry != 0UL && middle == left.Middle) ? 1UL : 0UL;
        return new Signed192(unchecked(left.High + right.High + carry), middle, low);
    }

    /// <summary>
    /// Subtracts exact three-word geometry values without scalar conversion.
    /// </summary>
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    internal static Signed192 SubtractSigned192(Signed192 left, Signed192 right)
    {
        ulong borrow = 0UL;
        ulong low = SubtractWord(left.Low, right.Low, ref borrow);
        ulong middle = SubtractWord(left.Middle, right.Middle, ref borrow);
        return new Signed192(unchecked(left.High - right.High - borrow), middle, low);
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

        if ((numeratorHigh | denominatorHigh) == 0UL)
        {
            result = GetUnitIntervalRatio128(
                numeratorMiddle,
                numeratorLow,
                denominatorMiddle,
                denominatorLow);
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
    /// Returns the exact signed result of <c>(first * second) - (third * fourth)</c>
    /// for values produced by three-component endpoint-difference dot products.
    /// </summary>
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    internal static Signed320 MultiplySubtract(
        Signed192 first,
        Signed192 second,
        Signed192 third,
        Signed192 fourth)
    {
        if (TryGetSigned95Magnitude(first, out ulong firstMiddle, out ulong firstLow)
            && TryGetSigned95Magnitude(second, out ulong secondMiddle, out ulong secondLow)
            && TryGetSigned95Magnitude(third, out ulong thirdMiddle, out ulong thirdLow)
            && TryGetSigned95Magnitude(fourth, out ulong fourthMiddle, out ulong fourthLow))
        {
            Signed192 narrowFirstProduct = MultiplySigned95(
                firstMiddle,
                firstLow,
                secondMiddle,
                secondLow,
                first.Sign * second.Sign < 0);
            Signed192 narrowSecondProduct = MultiplySigned95(
                thirdMiddle,
                thirdLow,
                fourthMiddle,
                fourthLow,
                third.Sign * fourth.Sign < 0);
            return ExtendToSigned320(SubtractSigned192(narrowFirstProduct, narrowSecondProduct));
        }

        Signed320 firstProduct = MultiplySigned192(first, second);
        Signed320 secondProduct = MultiplySigned192(third, fourth);

        ulong word0 = unchecked(firstProduct.Word0 - secondProduct.Word0);
        ulong borrow = firstProduct.Word0 < secondProduct.Word0 ? 1UL : 0UL;
        ulong word1 = SubtractWord(firstProduct.Word1, secondProduct.Word1, ref borrow);
        ulong word2 = SubtractWord(firstProduct.Word2, secondProduct.Word2, ref borrow);
        ulong word3 = SubtractWord(firstProduct.Word3, secondProduct.Word3, ref borrow);
        ulong word4 = unchecked(firstProduct.Word4 - secondProduct.Word4 - borrow);
        return new Signed320(word4, word3, word2, word1, word0);
    }

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    private static bool TryGetSigned95Magnitude(
        Signed192 value,
        out ulong middle,
        out ulong low)
    {
        GetMagnitude(value, out ulong high, out middle, out low);
        return high == 0UL && middle <= 0x7FFF_FFFFUL;
    }

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    private static Signed192 MultiplySigned95(
        ulong leftMiddle,
        ulong leftLow,
        ulong rightMiddle,
        ulong rightLow,
        bool negative)
    {
        MultiplyUnsigned96(
            (uint)leftMiddle,
            leftLow,
            (uint)rightMiddle,
            rightLow,
            out _,
            out ulong word2,
            out ulong word1,
            out ulong word0);

        if (negative)
        {
            word0 = unchecked(~word0 + 1UL);
            word1 = unchecked(~word1 + (word0 == 0UL ? 1UL : 0UL));
            word2 = unchecked(~word2 + (word1 == 0UL && word0 == 0UL ? 1UL : 0UL));
        }

        return new Signed192(word2, word1, word0);
    }

    /// <summary>
    /// Sign-extends an exact three-word geometry value to the segment solver width.
    /// </summary>
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    internal static Signed320 ExtendToSigned320(Signed192 value)
    {
        ulong extension = value.Sign < 0 ? ulong.MaxValue : 0UL;
        return new Signed320(extension, extension, value.High, value.Middle, value.Low);
    }

    /// <summary>
    /// Compares unsigned magnitudes of signed five-word values.
    /// </summary>
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    internal static int CompareMagnitude(Signed320 left, Signed320 right)
    {
        GetMagnitude(
            left,
            out ulong leftWord4,
            out ulong leftWord3,
            out ulong leftWord2,
            out ulong leftWord1,
            out ulong leftWord0);
        GetMagnitude(
            right,
            out ulong rightWord4,
            out ulong rightWord3,
            out ulong rightWord2,
            out ulong rightWord1,
            out ulong rightWord0);
        return CompareUnsigned(
            leftWord4,
            leftWord3,
            leftWord2,
            leftWord1,
            leftWord0,
            rightWord4,
            rightWord3,
            rightWord2,
            rightWord1,
            rightWord0);
    }

    /// <summary>
    /// Applies the public 3D segment near-parallel threshold to an exact Q128.128 determinant.
    /// </summary>
    internal static bool IsSegmentDeterminantNearParallel(Signed320 determinant)
    {
        GetMagnitude(
            determinant,
            out ulong word4,
            out ulong word3,
            out ulong word2,
            out ulong word1,
            out ulong word0);
        ulong epsilonRaw = (ulong)Epsilon.m_rawValue;
        return CompareUnsigned(
            word4,
            word3,
            word2,
            word1,
            word0,
            0UL,
            0UL,
            epsilonRaw >> 32,
            epsilonRaw << 32,
            0UL) < 0;
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
        if (TryNarrowSigned192(numerator, out Signed192 narrowNumerator)
            && TryNarrowSigned192(denominator, out Signed192 narrowDenominator))
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

        GetMagnitude(
            numerator,
            out ulong numeratorWord4,
            out ulong numeratorWord3,
            out ulong numeratorWord2,
            out ulong numeratorWord1,
            out ulong numeratorWord0);
        GetMagnitude(
            denominator,
            out ulong denominatorWord4,
            out ulong denominatorWord3,
            out ulong denominatorWord2,
            out ulong denominatorWord1,
            out ulong denominatorWord0);
        int comparison = CompareUnsigned(
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

        ulong guardedQuotient = 0UL;
        for (int bit = FixedMath.SHIFT_AMOUNT_I; bit >= 0; bit--)
        {
            ShiftLeftOne(
                ref numeratorWord4,
                ref numeratorWord3,
                ref numeratorWord2,
                ref numeratorWord1,
                ref numeratorWord0);
            guardedQuotient <<= 1;
            if (CompareUnsigned(
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

            SubtractUnsigned(
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
        result = new Fixed64((long)rounded);
        return true;
    }

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    private static bool TryNarrowSigned192(Signed320 value, out Signed192 result)
    {
        ulong extension = (value.Word2 & (1UL << 63)) != 0UL ? ulong.MaxValue : 0UL;
        if (value.Word4 != extension || value.Word3 != extension)
        {
            result = default;
            return false;
        }

        result = new Signed192(value.Word2, value.Word1, value.Word0);
        return true;
    }

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    private static Signed320 MultiplySigned192(Signed192 left, Signed192 right)
    {
        GetMagnitude(left, out ulong leftHigh, out ulong leftMiddle, out ulong leftLow);
        GetMagnitude(right, out ulong rightHigh, out ulong rightMiddle, out ulong rightLow);

        MultiplyUnsigned128(
            leftMiddle,
            leftLow,
            rightMiddle,
            rightLow,
            out ulong word3,
            out ulong word2,
            out ulong word1,
            out ulong word0);

        ulong word4 = 0UL;
        if ((leftHigh | rightHigh) != 0UL)
        {
            AddProductAt2(ref word4, ref word3, ref word2, leftLow, rightHigh);
            AddProductAt2(ref word4, ref word3, ref word2, leftHigh, rightLow);
            AddProductAt3(ref word4, ref word3, leftMiddle, rightHigh);
            AddProductAt3(ref word4, ref word3, leftHigh, rightMiddle);
            word4 = unchecked(word4 + (leftHigh * rightHigh));
        }

        if (left.Sign * right.Sign < 0)
        {
            word0 = unchecked(~word0 + 1UL);
            word1 = unchecked(~word1 + (word0 == 0UL ? 1UL : 0UL));
            word2 = unchecked(~word2 + (word1 == 0UL && word0 == 0UL ? 1UL : 0UL));
            word3 = unchecked(~word3 + (word2 == 0UL && word1 == 0UL && word0 == 0UL ? 1UL : 0UL));
            word4 = unchecked(~word4 + (word3 == 0UL && word2 == 0UL && word1 == 0UL && word0 == 0UL ? 1UL : 0UL));
        }

        return new Signed320(word4, word3, word2, word1, word0);
    }

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    private static void MultiplyUnsigned128(
        ulong leftHigh,
        ulong leftLow,
        ulong rightHigh,
        ulong rightLow,
        out ulong word3,
        out ulong word2,
        out ulong word1,
        out ulong word0)
    {
        if ((leftHigh | rightHigh) <= uint.MaxValue)
        {
            MultiplyUnsigned96(
                (uint)leftHigh,
                leftLow,
                (uint)rightHigh,
                rightLow,
                out word3,
                out word2,
                out word1,
                out word0);
            return;
        }

        Multiply64To128(leftLow, rightLow, out ulong lowHigh, out word0);
        Multiply64To128(leftLow, rightHigh, out ulong leftCrossHigh, out ulong leftCrossLow);
        Multiply64To128(leftHigh, rightLow, out ulong rightCrossHigh, out ulong rightCrossLow);
        Multiply64To128(leftHigh, rightHigh, out word3, out ulong highLow);

        word1 = lowHigh;
        ulong carry = 0UL;
        AccumulateWord(ref word1, leftCrossLow, ref carry);
        AccumulateWord(ref word1, rightCrossLow, ref carry);

        word2 = leftCrossHigh;
        ulong highCarry = 0UL;
        AccumulateWord(ref word2, rightCrossHigh, ref highCarry);
        AccumulateWord(ref word2, highLow, ref highCarry);
        AccumulateWord(ref word2, carry, ref highCarry);
        word3 = unchecked(word3 + highCarry);
    }

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    private static void MultiplyUnsigned96(
        uint leftHigh,
        ulong leftLow,
        uint rightHigh,
        ulong rightLow,
        out ulong word3,
        out ulong word2,
        out ulong word1,
        out ulong word0)
    {
        Multiply64To128(leftLow, rightLow, out ulong lowHigh, out word0);
        Multiply64By32(leftLow, rightHigh, out ulong leftCrossHigh, out ulong leftCrossLow);
        Multiply64By32(rightLow, leftHigh, out ulong rightCrossHigh, out ulong rightCrossLow);

        word1 = lowHigh;
        ulong carry = 0UL;
        AccumulateWord(ref word1, leftCrossLow, ref carry);
        AccumulateWord(ref word1, rightCrossLow, ref carry);

        word2 = leftCrossHigh;
        ulong highCarry = 0UL;
        AccumulateWord(ref word2, rightCrossHigh, ref highCarry);
        AccumulateWord(ref word2, (ulong)leftHigh * rightHigh, ref highCarry);
        AccumulateWord(ref word2, carry, ref highCarry);
        word3 = highCarry;
    }

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    private static void Multiply64By32(
        ulong left,
        uint right,
        out ulong high,
        out ulong low)
    {
        ulong lowProduct = (uint)left * (ulong)right;
        ulong highProduct = (left >> 32) * right;
        ulong middle = (lowProduct >> 32) + (uint)highProduct;
        low = (lowProduct & uint.MaxValue) | (middle << 32);
        high = (highProduct >> 32) + (middle >> 32);
    }

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    private static void AddProductAt2(
        ref ulong word4,
        ref ulong word3,
        ref ulong word2,
        ulong left,
        ulong right)
    {
        Multiply64To128(left, right, out ulong high, out ulong low);
        ulong previous = word2;
        word2 = unchecked(word2 + low);
        ulong carry = word2 < previous ? 1UL : 0UL;

        // A 64-by-64 product's high word is at most UInt64.MaxValue - 1,
        // so adding the one-bit low-word carry cannot overflow here.
        ulong highWithCarry = high + carry;
        previous = word3;
        word3 = unchecked(word3 + highWithCarry);
        if (word3 < previous)
            word4++;
    }

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    private static void AddProductAt3(
        ref ulong word4,
        ref ulong word3,
        ulong left,
        ulong right)
    {
        Multiply64To128(left, right, out ulong high, out ulong low);
        ulong previous = word3;
        word3 = unchecked(word3 + low);
        word4 = unchecked(word4 + high + (word3 < previous ? 1UL : 0UL));
    }

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    private static void AccumulateWord(ref ulong word, ulong add, ref ulong carry)
    {
        ulong previous = word;
        word = unchecked(word + add);
        if (word < previous)
            carry++;
    }

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    private static ulong SubtractWord(ulong value, ulong subtract, ref ulong borrow)
    {
        ulong subtrahend = unchecked(subtract + borrow);
        ulong overflow = subtrahend < subtract ? 1UL : 0UL;
        ulong result = unchecked(value - subtrahend);
        borrow = overflow | (value < subtrahend ? 1UL : 0UL);
        return result;
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
    private static void GetMagnitude(
        Signed320 value,
        out ulong word4,
        out ulong word3,
        out ulong word2,
        out ulong word1,
        out ulong word0)
    {
        word4 = value.Word4;
        word3 = value.Word3;
        word2 = value.Word2;
        word1 = value.Word1;
        word0 = value.Word0;
        if (value.Sign >= 0)
            return;

        word0 = unchecked(~word0 + 1UL);
        word1 = unchecked(~word1 + (word0 == 0UL ? 1UL : 0UL));
        word2 = unchecked(~word2 + (word1 == 0UL && word0 == 0UL ? 1UL : 0UL));
        word3 = unchecked(~word3 + (word2 == 0UL && word1 == 0UL && word0 == 0UL ? 1UL : 0UL));
        word4 = unchecked(~word4 + (word3 == 0UL && word2 == 0UL && word1 == 0UL && word0 == 0UL ? 1UL : 0UL));
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
    private static int CompareUnsigned(
        ulong leftWord4,
        ulong leftWord3,
        ulong leftWord2,
        ulong leftWord1,
        ulong leftWord0,
        ulong rightWord4,
        ulong rightWord3,
        ulong rightWord2,
        ulong rightWord1,
        ulong rightWord0)
    {
        if (leftWord4 != rightWord4)
            return leftWord4 < rightWord4 ? -1 : 1;
        if (leftWord3 != rightWord3)
            return leftWord3 < rightWord3 ? -1 : 1;
        if (leftWord2 != rightWord2)
            return leftWord2 < rightWord2 ? -1 : 1;
        if (leftWord1 != rightWord1)
            return leftWord1 < rightWord1 ? -1 : 1;
        if (leftWord0 != rightWord0)
            return leftWord0 < rightWord0 ? -1 : 1;
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
    private static void ShiftLeftOne(
        ref ulong word4,
        ref ulong word3,
        ref ulong word2,
        ref ulong word1,
        ref ulong word0)
    {
        word4 = (word4 << 1) | (word3 >> 63);
        word3 = (word3 << 1) | (word2 >> 63);
        word2 = (word2 << 1) | (word1 >> 63);
        word1 = (word1 << 1) | (word0 >> 63);
        word0 <<= 1;
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

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    private static void SubtractUnsigned(
        ref ulong word4,
        ref ulong word3,
        ref ulong word2,
        ref ulong word1,
        ref ulong word0,
        ulong subtractWord4,
        ulong subtractWord3,
        ulong subtractWord2,
        ulong subtractWord1,
        ulong subtractWord0)
    {
        ulong borrow = 0UL;
        word0 = SubtractWord(word0, subtractWord0, ref borrow);
        word1 = SubtractWord(word1, subtractWord1, ref borrow);
        word2 = SubtractWord(word2, subtractWord2, ref borrow);
        word3 = SubtractWord(word3, subtractWord3, ref borrow);
        word4 = unchecked(word4 - subtractWord4 - borrow);
    }
}
