//=======================================================================
// WideArithmetic.cs
//=======================================================================
// MIT License, Copyright (c) 2024–present David Oravsky (mrdav30)
// See LICENSE file in the project root for full license information.
//=======================================================================

using System.Runtime.CompilerServices;

namespace FixedMathSharp;

/// <summary>
/// Owns fixed-width limb arithmetic used by exact deterministic geometry.
/// </summary>
internal static class WideArithmetic
{
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
    /// Returns whether a wide magnitude is at most a positive raw threshold
    /// shifted into the value's scale.
    /// </summary>
    internal static bool IsMagnitudeAtMost(Signed192 value, ulong rawThreshold, int leftShift)
    {
        Signed192 threshold = new(
            0UL,
            rawThreshold >> (64 - leftShift),
            rawThreshold << leftShift);
        return CompareMagnitude(value, threshold) <= 0;
    }

    /// <summary>
    /// Adds exact three-word values without scalar conversion.
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
    /// Subtracts exact three-word values without scalar conversion.
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
    /// Returns the exact signed result of <c>(first * second) - (third * fourth)</c>.
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

    /// <summary>
    /// Sign-extends an exact three-word value to five words.
    /// </summary>
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    internal static Signed320 ExtendToSigned320(Signed192 value)
    {
        ulong extension = value.Sign < 0 ? ulong.MaxValue : 0UL;
        return new Signed320(extension, extension, value.High, value.Middle, value.Low);
    }

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    internal static Signed192 FromSignedRaw(long value)
    {
        ulong extension = value < 0L ? ulong.MaxValue : 0UL;
        return new Signed192(extension, extension, unchecked((ulong)value));
    }

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    internal static Signed320 AddSigned320(Signed320 left, Signed320 right)
    {
        ulong word0 = unchecked(left.Word0 + right.Word0);
        ulong carry = word0 < left.Word0 ? 1UL : 0UL;
        ulong word1 = unchecked(left.Word1 + right.Word1 + carry);
        carry = word1 < left.Word1 || (carry != 0UL && word1 == left.Word1) ? 1UL : 0UL;
        ulong word2 = unchecked(left.Word2 + right.Word2 + carry);
        carry = word2 < left.Word2 || (carry != 0UL && word2 == left.Word2) ? 1UL : 0UL;
        ulong word3 = unchecked(left.Word3 + right.Word3 + carry);
        // The high bit of the standard carry expression includes the incoming carry encoded in word3.
        carry = ((left.Word3 & right.Word3) | ((left.Word3 | right.Word3) & ~word3)) >> 63;
        return new Signed320(unchecked(left.Word4 + right.Word4 + carry), word3, word2, word1, word0);
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

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    internal static bool TryNarrowSigned192(Signed320 value, out Signed192 result)
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
    internal static Signed320 MultiplySigned192(Signed192 left, Signed192 right)
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
    internal static void GetMagnitude(
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
    internal static void GetMagnitude(
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
    internal static int CompareUnsigned(
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
    internal static int CompareUnsigned(
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
    internal static void ShiftLeftOne(ref ulong high, ref ulong middle, ref ulong low)
    {
        high = (high << 1) | (middle >> 63);
        middle = (middle << 1) | (low >> 63);
        low <<= 1;
    }

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    internal static void ShiftLeft(
        ulong high,
        ulong middle,
        ulong low,
        int bits,
        out ulong shiftedHigh,
        out ulong shiftedMiddle,
        out ulong shiftedLow)
    {
        if (bits == 0)
        {
            shiftedHigh = high;
            shiftedMiddle = middle;
            shiftedLow = low;
            return;
        }

        shiftedHigh = (high << bits) | (middle >> (64 - bits));
        shiftedMiddle = (middle << bits) | (low >> (64 - bits));
        shiftedLow = low << bits;
    }

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    internal static void ShiftRightOne(ref ulong high, ref ulong middle, ref ulong low)
    {
        low = (low >> 1) | (middle << 63);
        middle = (middle >> 1) | (high << 63);
        high >>= 1;
    }

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    internal static int GetBitLength(ulong high, ulong middle, ulong low)
    {
        if (high != 0UL)
            return 192 - Fixed64.CountLeadingZeroes(high);
        if (middle != 0UL)
            return 128 - Fixed64.CountLeadingZeroes(middle);
        return 64 - Fixed64.CountLeadingZeroes(low);
    }

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    internal static void ShiftLeftOne(
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
    internal static void SubtractUnsigned(
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
    internal static void SubtractUnsigned(
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

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    private static bool TryGetSigned95Magnitude(
        Signed192 value,
        out ulong middle,
        out ulong low)
    {
        GetMagnitude(value, out ulong high, out middle, out low);
        return high == 0UL && middle <= 0x7FFF_FFFFUL; // 2,147,483,647 (31 significant high bits).
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

        Fixed64.Multiply64To128(leftLow, rightLow, out ulong lowHigh, out word0);
        Fixed64.Multiply64To128(leftLow, rightHigh, out ulong leftCrossHigh, out ulong leftCrossLow);
        Fixed64.Multiply64To128(leftHigh, rightLow, out ulong rightCrossHigh, out ulong rightCrossLow);
        Fixed64.Multiply64To128(leftHigh, rightHigh, out word3, out ulong highLow);

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
        Fixed64.Multiply64To128(leftLow, rightLow, out ulong lowHigh, out word0);
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
        Fixed64.Multiply64To128(left, right, out ulong high, out ulong low);
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
        Fixed64.Multiply64To128(left, right, out ulong high, out ulong low);
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
}
