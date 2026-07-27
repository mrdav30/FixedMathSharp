//=======================================================================
// WideArithmetic.Signed832.cs
//=======================================================================
// MIT License, Copyright (c) 2024–present David Oravsky (mrdav30)
// See LICENSE file in the project root for full license information.
//=======================================================================

using System;
using System.Runtime.CompilerServices;

namespace FixedMathSharp;

/// <content>
/// Signed 832-bit arithmetic helpers: addition, magnitude extraction, and squared-ratio comparison.
/// </content>
internal static partial class WideArithmetic
{
    internal static void GetMagnitude(Signed832 value, Span<ulong> magnitude)
    {
        magnitude.Clear();
        magnitude[0] = value.Word0;
        magnitude[1] = value.Word1;
        magnitude[2] = value.Word2;
        magnitude[3] = value.Word3;
        magnitude[4] = value.Word4;
        magnitude[5] = value.Word5;
        magnitude[6] = value.Word6;
        magnitude[7] = value.Word7;
        magnitude[8] = value.Word8;
        magnitude[9] = value.Word9;
        magnitude[10] = value.Word10;
        magnitude[11] = value.Word11;
        magnitude[12] = value.Word12;
        if (value.Sign >= 0)
            return;

        ulong carry = 1UL;
        for (int index = 0; index < 13; index++)
            magnitude[index] = AddSignedWord(~magnitude[index], 0UL, ref carry);
    }

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    internal static Signed832 AddSigned832(Signed832 left, Signed832 right)
    {
        ulong carry = 0UL;
        ulong word0 = AddSignedWord(left.Word0, right.Word0, ref carry);
        ulong word1 = AddSignedWord(left.Word1, right.Word1, ref carry);
        ulong word2 = AddSignedWord(left.Word2, right.Word2, ref carry);
        ulong word3 = AddSignedWord(left.Word3, right.Word3, ref carry);
        ulong word4 = AddSignedWord(left.Word4, right.Word4, ref carry);
        ulong word5 = AddSignedWord(left.Word5, right.Word5, ref carry);
        ulong word6 = AddSignedWord(left.Word6, right.Word6, ref carry);
        ulong word7 = AddSignedWord(left.Word7, right.Word7, ref carry);
        ulong word8 = AddSignedWord(left.Word8, right.Word8, ref carry);
        ulong word9 = AddSignedWord(left.Word9, right.Word9, ref carry);
        ulong word10 = AddSignedWord(left.Word10, right.Word10, ref carry);
        ulong word11 = AddSignedWord(left.Word11, right.Word11, ref carry);
        ulong word12 = unchecked(left.Word12 + right.Word12 + carry);
        return new Signed832(
            word12, word11, word10, word9, word8, word7, word6,
            word5, word4, word3, word2, word1, word0);
    }

    internal static int CompareNonNegativeSquaredRatios(
        Signed832 leftNumerator,
        Signed576 leftDenominator,
        Signed832 rightNumerator,
        Signed576 rightDenominator)
    {
        Span<ulong> leftNumeratorWords = stackalloc ulong[13]
        {
            leftNumerator.Word0, leftNumerator.Word1, leftNumerator.Word2,
            leftNumerator.Word3, leftNumerator.Word4, leftNumerator.Word5,
            leftNumerator.Word6, leftNumerator.Word7, leftNumerator.Word8,
            leftNumerator.Word9, leftNumerator.Word10, leftNumerator.Word11,
            leftNumerator.Word12,
        };
        Span<ulong> rightNumeratorWords = stackalloc ulong[13]
        {
            rightNumerator.Word0, rightNumerator.Word1, rightNumerator.Word2,
            rightNumerator.Word3, rightNumerator.Word4, rightNumerator.Word5,
            rightNumerator.Word6, rightNumerator.Word7, rightNumerator.Word8,
            rightNumerator.Word9, rightNumerator.Word10, rightNumerator.Word11,
            rightNumerator.Word12,
        };
        Span<ulong> leftDenominatorWords = stackalloc ulong[9];
        Span<ulong> rightDenominatorWords = stackalloc ulong[9];
        GetMagnitude(leftDenominator, leftDenominatorWords);
        GetMagnitude(rightDenominator, rightDenominatorWords);

        Span<ulong> leftOnce = stackalloc ulong[22];
        Span<ulong> rightOnce = stackalloc ulong[22];
        Span<ulong> leftScaled = stackalloc ulong[31];
        Span<ulong> rightScaled = stackalloc ulong[31];
        MultiplyMagnitudes(leftNumeratorWords, rightDenominatorWords, leftOnce);
        MultiplyMagnitudes(rightNumeratorWords, leftDenominatorWords, rightOnce);
        MultiplyMagnitudes(leftOnce, rightDenominatorWords, leftScaled);
        MultiplyMagnitudes(rightOnce, leftDenominatorWords, rightScaled);
        return CompareMagnitude(leftScaled, rightScaled);
    }

    internal static int CompareNonNegativeProducts(
        Signed832 firstLeft,
        Signed832 firstRight,
        Signed832 secondLeft,
        Signed832 secondRight)
    {
        Span<ulong> firstLeftWords = stackalloc ulong[13];
        Span<ulong> firstRightWords = stackalloc ulong[13];
        Span<ulong> secondLeftWords = stackalloc ulong[13];
        Span<ulong> secondRightWords = stackalloc ulong[13];
        GetMagnitude(firstLeft, firstLeftWords);
        GetMagnitude(firstRight, firstRightWords);
        GetMagnitude(secondLeft, secondLeftWords);
        GetMagnitude(secondRight, secondRightWords);
        Span<ulong> firstProduct = stackalloc ulong[26];
        Span<ulong> secondProduct = stackalloc ulong[26];
        MultiplyMagnitudes(firstLeftWords, firstRightWords, firstProduct);
        MultiplyMagnitudes(secondLeftWords, secondRightWords, secondProduct);
        return CompareMagnitude(firstProduct, secondProduct);
    }

    /// <summary>
    /// Multiplies signed nine-word conic coefficients whose proven product fits
    /// in thirteen words.
    /// </summary>
    internal static Signed832 MultiplySigned576ToSigned832(Signed576 left, Signed576 right)
    {
        Span<ulong> leftMagnitude = stackalloc ulong[9];
        Span<ulong> rightMagnitude = stackalloc ulong[9];
        GetMagnitude(left, leftMagnitude);
        GetMagnitude(right, rightMagnitude);
        Span<ulong> product = stackalloc ulong[18];
        MultiplyMagnitudes(leftMagnitude, rightMagnitude, product);

        ApplySigned832Sign(product, left.Sign * right.Sign < 0);

        return CreateSigned832(product);
    }

    /// <summary>
    /// Multiplies a signed nine-word conic coefficient by a signed five-word
    /// rational product whose proven result fits in thirteen words.
    /// </summary>
    internal static Signed832 MultiplySigned576ToSigned832(Signed576 left, Signed320 right)
    {
        Span<ulong> leftMagnitude = stackalloc ulong[9];
        Span<ulong> rightMagnitude = stackalloc ulong[5];
        GetMagnitude(left, leftMagnitude);
        CopyMagnitude(right, rightMagnitude);
        Span<ulong> product = stackalloc ulong[14];
        MultiplyMagnitudes(leftMagnitude, rightMagnitude, product);
        ApplySigned832Sign(product, left.Sign * right.Sign < 0);
        return CreateSigned832(product);
    }

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    internal static Signed832 SubtractSigned832(Signed832 left, Signed832 right)
    {
        ulong borrow = 0UL;
        ulong word0 = SubtractWord(left.Word0, right.Word0, ref borrow);
        ulong word1 = SubtractWord(left.Word1, right.Word1, ref borrow);
        ulong word2 = SubtractWord(left.Word2, right.Word2, ref borrow);
        ulong word3 = SubtractWord(left.Word3, right.Word3, ref borrow);
        ulong word4 = SubtractWord(left.Word4, right.Word4, ref borrow);
        ulong word5 = SubtractWord(left.Word5, right.Word5, ref borrow);
        ulong word6 = SubtractWord(left.Word6, right.Word6, ref borrow);
        ulong word7 = SubtractWord(left.Word7, right.Word7, ref borrow);
        ulong word8 = SubtractWord(left.Word8, right.Word8, ref borrow);
        ulong word9 = SubtractWord(left.Word9, right.Word9, ref borrow);
        ulong word10 = SubtractWord(left.Word10, right.Word10, ref borrow);
        ulong word11 = SubtractWord(left.Word11, right.Word11, ref borrow);
        ulong word12 = unchecked(left.Word12 - right.Word12 - borrow);
        return new Signed832(
            word12, word11, word10, word9, word8, word7, word6,
            word5, word4, word3, word2, word1, word0);
    }

    /// <summary>
    /// Returns the exact floor square root of a proven-fit nonnegative conic
    /// discriminant product. The wider radicand remains transient stack storage.
    /// </summary>
    internal static Signed576 GetFloorSquareRootOfProduct(
        Signed832 value,
        Signed192 nonNegativeFactor)
    {
        if (value.IsZero || nonNegativeFactor.IsZero)
            return default;

        Span<ulong> valueMagnitude = stackalloc ulong[13]
        {
            value.Word0, value.Word1, value.Word2, value.Word3, value.Word4,
            value.Word5, value.Word6, value.Word7, value.Word8, value.Word9,
            value.Word10, value.Word11, value.Word12,
        };
        Span<ulong> factorMagnitude = stackalloc ulong[3];
        GetMagnitude(
            nonNegativeFactor,
            out factorMagnitude[2],
            out factorMagnitude[1],
            out factorMagnitude[0]);
        Span<ulong> product = stackalloc ulong[16];
        MultiplyMagnitudes(valueMagnitude, factorMagnitude, product);

        int productBitLength = GetBitLength(product);
        int rootBitLength = (productBitLength + 1) >> 1;
        int activeWords = System.Math.Min(9, (rootBitLength + 64) >> 6);
        Span<ulong> rootStorage = stackalloc ulong[9];
        Span<ulong> remainderStorage = stackalloc ulong[9];
        Span<ulong> candidateStorage = stackalloc ulong[9];
        Span<ulong> root = rootStorage[..activeWords];
        Span<ulong> remainder = remainderStorage[..activeWords];
        Span<ulong> candidate = candidateStorage[..activeWords];
        rootStorage.Clear();
        remainder.Clear();
        for (int pairIndex = (productBitLength - 1) >> 1; pairIndex >= 0; pairIndex--)
        {
            ShiftLeftMagnitude(remainder, 2);
            remainder[0] |= GetBitPair(product, pairIndex);
            ShiftLeftMagnitude(root, 1);
            root.CopyTo(candidate);
            ShiftLeftMagnitude(candidate, 1);
            candidate[0] |= 1UL;
            if (CompareMagnitude(remainder, candidate) < 0)
                continue;

            SubtractMagnitude(remainder, candidate);
            root[0]++;
        }

        return new Signed576(
            rootStorage[8], rootStorage[7], rootStorage[6], rootStorage[5], rootStorage[4],
            rootStorage[3], rootStorage[2], rootStorage[1], rootStorage[0]);
    }

    private static Signed832 CreateSigned832(ReadOnlySpan<ulong> words) =>
        new(
            words[12], words[11], words[10], words[9], words[8], words[7],
            words[6], words[5], words[4], words[3], words[2], words[1], words[0]);

    private static void ApplySigned832Sign(Span<ulong> words, bool negative)
    {
        if (!negative)
            return;

        ulong carry = 1UL;
        for (int index = 0; index < 13; index++)
            words[index] = AddSignedWord(~words[index], 0UL, ref carry);
    }

    /// <summary>
    /// Multiplies nonnegative thirteen-word and five-word factors whose proven
    /// product fits in thirteen words.
    /// </summary>
    internal static Signed832 MultiplyNonNegativeToSigned832(
        Signed832 left,
        Signed320 right)
    {
        Span<ulong> leftWords = stackalloc ulong[13]
        {
            left.Word0, left.Word1, left.Word2, left.Word3, left.Word4,
            left.Word5, left.Word6, left.Word7, left.Word8, left.Word9,
            left.Word10, left.Word11, left.Word12,
        };
        Span<ulong> rightWords = stackalloc ulong[5];
        GetMagnitude(
            right,
            out rightWords[4],
            out rightWords[3],
            out rightWords[2],
            out rightWords[1],
            out rightWords[0]);
        Span<ulong> product = stackalloc ulong[18];
        MultiplyMagnitudes(leftWords, rightWords, product);
        return new Signed832(
            product[12], product[11], product[10], product[9], product[8],
            product[7], product[6], product[5], product[4], product[3],
            product[2], product[1], product[0]);
    }
}
