//=======================================================================
// WideArithmetic.Signed832.cs
//=======================================================================
// MIT License, Copyright (c) 2024–present David Oravsky (mrdav30)
// See LICENSE file in the project root for full license information.
//=======================================================================

using System;
using System.Runtime.CompilerServices;

namespace FixedMathSharp;

internal static partial class WideArithmetic
{
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
        product.Clear();
        MultiplyMagnitudes(leftMagnitude, rightMagnitude, product);

        if (left.Sign * right.Sign < 0)
        {
            ulong carry = 1UL;
            for (int index = 0; index < 13; index++)
                product[index] = AddSignedWord(~product[index], 0UL, ref carry);
        }

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
        product.Clear();
        MultiplyMagnitudes(valueMagnitude, factorMagnitude, product);

        Span<ulong> root = stackalloc ulong[9];
        Span<ulong> remainder = stackalloc ulong[9];
        Span<ulong> candidate = stackalloc ulong[9];
        root.Clear();
        remainder.Clear();
        for (int pairIndex = (GetBitLength(product) - 1) >> 1; pairIndex >= 0; pairIndex--)
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
            root[8], root[7], root[6], root[5], root[4],
            root[3], root[2], root[1], root[0]);
    }

    private static Signed832 CreateSigned832(ReadOnlySpan<ulong> words) =>
        new(
            words[12], words[11], words[10], words[9], words[8], words[7],
            words[6], words[5], words[4], words[3], words[2], words[1], words[0]);
}
