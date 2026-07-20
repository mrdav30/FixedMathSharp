//=======================================================================
// WideArithmetic.cs
//=======================================================================
// MIT License, Copyright (c) 2024–present David Oravsky (mrdav30)
// See LICENSE file in the project root for full license information.
//=======================================================================

using System;
using System.Runtime.CompilerServices;

namespace FixedMathSharp;

/// <summary>
/// Owns fixed-width limb arithmetic used by exact deterministic geometry.
/// </summary>
internal static partial class WideArithmetic
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
    /// Subtracts exact five-word values without scalar conversion.
    /// </summary>
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    internal static Signed320 SubtractSigned320(Signed320 left, Signed320 right)
    {
        ulong borrow = 0UL;
        ulong word0 = SubtractWord(left.Word0, right.Word0, ref borrow);
        ulong word1 = SubtractWord(left.Word1, right.Word1, ref borrow);
        ulong word2 = SubtractWord(left.Word2, right.Word2, ref borrow);
        ulong word3 = SubtractWord(left.Word3, right.Word3, ref borrow);
        return new Signed320(unchecked(left.Word4 - right.Word4 - borrow), word3, word2, word1, word0);
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
    internal static int GetBitLength(
        ulong word4,
        ulong word3,
        ulong word2,
        ulong word1,
        ulong word0)
    {
        if (word4 != 0UL)
            return 320 - Fixed64.CountLeadingZeroes(word4);
        if (word3 != 0UL)
            return 256 - Fixed64.CountLeadingZeroes(word3);
        if (word2 != 0UL)
            return 192 - Fixed64.CountLeadingZeroes(word2);
        if (word1 != 0UL)
            return 128 - Fixed64.CountLeadingZeroes(word1);
        return 64 - Fixed64.CountLeadingZeroes(word0);
    }

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    internal static void ShiftLeft(
        ulong word4,
        ulong word3,
        ulong word2,
        ulong word1,
        ulong word0,
        int bits,
        out ulong shiftedWord4,
        out ulong shiftedWord3,
        out ulong shiftedWord2,
        out ulong shiftedWord1,
        out ulong shiftedWord0)
    {
        if (bits == 0)
        {
            shiftedWord4 = word4;
            shiftedWord3 = word3;
            shiftedWord2 = word2;
            shiftedWord1 = word1;
            shiftedWord0 = word0;
            return;
        }

        shiftedWord4 = (word4 << bits) | (word3 >> (64 - bits));
        shiftedWord3 = (word3 << bits) | (word2 >> (64 - bits));
        shiftedWord2 = (word2 << bits) | (word1 >> (64 - bits));
        shiftedWord1 = (word1 << bits) | (word0 >> (64 - bits));
        shiftedWord0 = word0 << bits;
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
    internal static void ShiftRightOne(
        ref ulong word4,
        ref ulong word3,
        ref ulong word2,
        ref ulong word1,
        ref ulong word0)
    {
        word0 = (word0 >> 1) | (word1 << 63);
        word1 = (word1 >> 1) | (word2 << 63);
        word2 = (word2 >> 1) | (word3 << 63);
        word3 = (word3 >> 1) | (word4 << 63);
        word4 >>= 1;
    }

    /// <summary>
    /// Returns the floor square root and exact remainder of a nonnegative five-word value.
    /// </summary>
    internal static Signed192 GetFloorSquareRoot(Signed320 value, out Signed192 remainder)
    {
        if (value.IsZero)
        {
            remainder = default;
            return default;
        }

        if (value.Word4 == 0UL && value.Word3 == 0UL)
            return GetFloorSquareRoot192(value.Word2, value.Word1, value.Word0, out remainder);

        int pairIndex = (GetBitLength(value.Word4, value.Word3, value.Word2, value.Word1, value.Word0) - 1) >> 1;
        ulong rootHigh = 0UL;
        ulong rootMiddle = 0UL;
        ulong rootLow = 0UL;
        ulong remainderHigh = 0UL;
        ulong remainderMiddle = 0UL;
        ulong remainderLow = 0UL;

        for (; pairIndex >= 0; pairIndex--)
        {
            int pairShift = (pairIndex & 31) << 1;
            ulong pair = (pairIndex >> 5) switch
            {
                4 => (value.Word4 >> pairShift) & 3UL,
                3 => (value.Word3 >> pairShift) & 3UL,
                2 => (value.Word2 >> pairShift) & 3UL,
                1 => (value.Word1 >> pairShift) & 3UL,
                _ => (value.Word0 >> pairShift) & 3UL,
            };

            remainderHigh = (remainderHigh << 2) | (remainderMiddle >> 62);
            remainderMiddle = (remainderMiddle << 2) | (remainderLow >> 62);
            remainderLow = (remainderLow << 2) | pair;
            rootHigh = (rootHigh << 1) | (rootMiddle >> 63);
            rootMiddle = (rootMiddle << 1) | (rootLow >> 63);
            rootLow <<= 1;

            ulong candidateHigh = (rootHigh << 1) | (rootMiddle >> 63);
            ulong candidateMiddle = (rootMiddle << 1) | (rootLow >> 63);
            ulong candidateLow = (rootLow << 1) | 1UL;
            if (CompareUnsigned(
                remainderHigh,
                remainderMiddle,
                remainderLow,
                candidateHigh,
                candidateMiddle,
                candidateLow) < 0)
            {
                continue;
            }

            SubtractUnsigned(
                ref remainderHigh,
                ref remainderMiddle,
                ref remainderLow,
                candidateHigh,
                candidateMiddle,
                candidateLow);
            // The preceding shift leaves rootLow even, so this increment cannot overflow.
            rootLow++;
        }

        remainder = new Signed192(remainderHigh, remainderMiddle, remainderLow);
        return new Signed192(rootHigh, rootMiddle, rootLow);
    }

    /// <summary>
    /// Shifts a nonnegative three-word value into a 64-bit normalized prefix.
    /// </summary>
    /// <remarks>
    /// Triangle normalization calls this with 0 through 67 discarded bits,
    /// which guarantees that the returned prefix fits in 64 bits.
    /// </remarks>
    internal static ulong ShiftRightToUInt64(Signed192 value, int bits, out bool discarded)
    {
        GetMagnitude(value, out ulong high, out ulong middle, out ulong low);
        if (bits == 0)
        {
            discarded = false;
            return low;
        }

        if (bits < 64)
        {
            discarded = (low & ((1UL << bits) - 1UL)) != 0UL;
            return (middle << (64 - bits)) | (low >> bits);
        }

        int upperShift = bits - 64;
        discarded = low != 0UL
            || (upperShift != 0 && (middle & ((1UL << upperShift) - 1UL)) != 0UL);
        return upperShift == 0
            ? middle
            : (high << (64 - upperShift)) | (middle >> upperShift);
    }

    /// <summary>
    /// Compares a normalized component square with the squared midpoint between
    /// adjacent Q32.32 raw candidates.
    /// </summary>
    internal static int CompareNormalizedComponentToMidpoint(
        Signed320 componentSquare,
        Signed320 squaredMagnitude,
        ulong lowerCandidateRaw)
    {
        ulong doubledMidpoint = (lowerCandidateRaw << 1) + 1UL;
        Fixed64.Multiply64To128(
            doubledMidpoint,
            doubledMidpoint,
            out ulong multiplierHigh,
            out ulong multiplierLow);

        if (TryNarrowSigned192(squaredMagnitude, out Signed192 narrowSquaredMagnitude))
        {
            Signed320 rightNarrow = MultiplySigned192(
                narrowSquaredMagnitude,
                new Signed192(0UL, multiplierHigh, multiplierLow));
            return CompareUnsigned(
                (componentSquare.Word3 << 2) | (componentSquare.Word2 >> 62),
                (componentSquare.Word2 << 2) | (componentSquare.Word1 >> 62),
                (componentSquare.Word1 << 2) | (componentSquare.Word0 >> 62),
                componentSquare.Word0 << 2,
                0UL,
                rightNarrow.Word4,
                rightNarrow.Word3,
                rightNarrow.Word2,
                rightNarrow.Word1,
                rightNarrow.Word0);
        }

        Span<ulong> left = stackalloc ulong[6];
        left.Clear();
        left[1] = componentSquare.Word0 << 2;
        left[2] = (componentSquare.Word1 << 2) | (componentSquare.Word0 >> 62);
        left[3] = (componentSquare.Word2 << 2) | (componentSquare.Word1 >> 62);
        left[4] = (componentSquare.Word3 << 2) | (componentSquare.Word2 >> 62);
        left[5] = (componentSquare.Word4 << 2) | (componentSquare.Word3 >> 62);
        Span<ulong> right = stackalloc ulong[6];
        right.Clear();
        Span<ulong> magnitude = stackalloc ulong[5]
        {
            squaredMagnitude.Word0,
            squaredMagnitude.Word1,
            squaredMagnitude.Word2,
            squaredMagnitude.Word3,
            squaredMagnitude.Word4,
        };

        for (int index = 0; index < magnitude.Length; index++)
        {
            Fixed64.Multiply64To128(magnitude[index], multiplierLow, out ulong high, out ulong low);
            AddWord(right, index, low);
            AddWord(right, index + 1, high);
            if (multiplierHigh == 0UL)
                continue;

            Fixed64.Multiply64To128(magnitude[index], multiplierHigh, out high, out low);
            AddWord(right, index + 1, low);
            if (index + 2 < right.Length)
                AddWord(right, index + 2, high);
        }

        for (int index = left.Length - 1; index >= 0; index--)
        {
            if (left[index] != right[index])
                return left[index] < right[index] ? -1 : 1;
        }

        return 0;
    }

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    private static Signed192 GetFloorSquareRoot192(
        ulong valueHigh,
        ulong valueMiddle,
        ulong valueLow,
        out Signed192 remainder)
    {
        int pairIndex = (GetBitLength(valueHigh, valueMiddle, valueLow) - 1) >> 1;
        ulong rootHigh = 0UL;
        ulong rootLow = 0UL;
        ulong remainderHigh = 0UL;
        ulong remainderLow = 0UL;

        for (; pairIndex >= 0; pairIndex--)
        {
            int pairShift = (pairIndex & 31) << 1;
            ulong pair = pairIndex >= 64
                ? (valueHigh >> pairShift) & 3UL
                : pairIndex >= 32
                    ? (valueMiddle >> pairShift) & 3UL
                    : (valueLow >> pairShift) & 3UL;

            remainderHigh = (remainderHigh << 2) | (remainderLow >> 62);
            remainderLow = (remainderLow << 2) | pair;
            rootHigh = (rootHigh << 1) | (rootLow >> 63);
            rootLow <<= 1;

            ulong candidateHigh = (rootHigh << 1) | (rootLow >> 63);
            ulong candidateLow = (rootLow << 1) | 1UL;
            if (remainderHigh < candidateHigh
                || (remainderHigh == candidateHigh && remainderLow < candidateLow))
            {
                continue;
            }

            ulong originalRemainderLow = remainderLow;
            remainderLow -= candidateLow;
            remainderHigh -= candidateHigh + (originalRemainderLow < candidateLow ? 1UL : 0UL);
            rootLow++;
        }

        remainder = new Signed192(0UL, remainderHigh, remainderLow);
        return new Signed192(0UL, rootHigh, rootLow);
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

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    private static void AddWord(Span<ulong> words, int index, ulong value)
    {
        while (value != 0UL && index < words.Length)
        {
            ulong previous = words[index];
            words[index] = unchecked(previous + value);
            value = words[index] < previous ? 1UL : 0UL;
            index++;
        }
    }
}
