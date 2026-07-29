//=======================================================================
// WideArithmetic.Signed320.cs
//=======================================================================
// MIT License, Copyright (c) 2024–present David Oravsky (mrdav30)
// See LICENSE file in the project root for full license information.
//=======================================================================

using System;
using System.Runtime.CompilerServices;

namespace FixedMathSharp;

/// <content>
/// Arithmetic helpers for <see cref="Signed320"/> values, including magnitude extraction
/// and combined multiply-subtract operations built from <see cref="Signed192"/> operands.
/// </content>
internal static partial class WideArithmetic
{
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
            return Signed320.ExtendValue(SubtractSigned192(narrowFirstProduct, narrowSecondProduct));
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
    /// Returns the signed sum of two <see cref="Signed192"/> products.
    /// </summary>
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    internal static Signed320 AddProducts(
        Signed192 firstLeft,
        Signed192 firstRight,
        Signed192 secondLeft,
        Signed192 secondRight) =>
        AddSigned320(
            MultiplySigned192(firstLeft, firstRight),
            MultiplySigned192(secondLeft, secondRight));

    /// <summary>
    /// Returns the signed sum of three <see cref="Signed192"/> products.
    /// </summary>
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    internal static Signed320 AddProducts(
        Signed192 firstLeft,
        Signed192 firstRight,
        Signed192 secondLeft,
        Signed192 secondRight,
        Signed192 thirdLeft,
        Signed192 thirdRight) =>
        AddSigned320(
            AddProducts(
                firstLeft,
                firstRight,
                secondLeft,
                secondRight),
            MultiplySigned192(thirdLeft, thirdRight));

    /// <summary>
    /// Returns the signed dot product of two three-component
    /// <see cref="Signed192"/> values.
    /// </summary>
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    internal static Signed320 GetDotProduct3D(
        Signed192 leftX,
        Signed192 leftY,
        Signed192 leftZ,
        Signed192 rightX,
        Signed192 rightY,
        Signed192 rightZ) =>
        AddProducts(
            leftX,
            rightX,
            leftY,
            rightY,
            leftZ,
            rightZ);

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

        if (Signed192.TryNarrowSigned(squaredMagnitude, out Signed192 narrowSquaredMagnitude))
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
}
