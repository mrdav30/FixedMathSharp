//=======================================================================
// WideArithmetic.Signed704.cs
//=======================================================================
// MIT License, Copyright (c) 2024–present David Oravsky (mrdav30)
// See LICENSE file in the project root for full license information.
//=======================================================================

using System;
using System.Runtime.CompilerServices;

namespace FixedMathSharp;

/// <content>
/// Arithmetic helpers for <see cref="Signed704"/>, including magnitude extraction,
/// signed multiplication producing an eleven-word result, and square root computation.
/// </content>
internal static partial class WideArithmetic
{
    internal static void GetMagnitude(Signed704 value, Span<ulong> magnitude)
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
        if (value.Sign >= 0)
            return;

        ulong carry = 1UL;
        for (int index = 0; index < 11; index++)
            magnitude[index] = AddSignedWord(~magnitude[index], 0UL, ref carry);
    }

    /// <summary>
    /// Multiplies a signed nine-word value by a signed five-word value whose
    /// proven product fits in eleven words.
    /// </summary>
    internal static Signed704 MultiplySigned576ToSigned704(Signed576 left, Signed320 right)
    {
        Span<ulong> leftMagnitude = stackalloc ulong[9];
        Span<ulong> rightMagnitude = stackalloc ulong[5];
        GetMagnitude(left, leftMagnitude);
        CopyMagnitude(right, rightMagnitude);
        Span<ulong> product = stackalloc ulong[14];
        MultiplyMagnitudes(leftMagnitude, rightMagnitude, product);
        ApplySigned704Sign(product, left.Sign * right.Sign < 0);
        return CreateSigned704(product);
    }

    /// <summary>
    /// Multiplies a nonnegative nine-word value by a nonnegative three-word
    /// value whose proven product fits in eleven words.
    /// </summary>
    internal static Signed704 MultiplyNonNegative(Signed576 left, Signed192 right)
    {
        Span<ulong> leftMagnitude = stackalloc ulong[9];
        Span<ulong> rightMagnitude = stackalloc ulong[3];
        GetMagnitude(left, leftMagnitude);
        GetMagnitude(right, out rightMagnitude[2], out rightMagnitude[1], out rightMagnitude[0]);
        Span<ulong> product = stackalloc ulong[12];
        MultiplyMagnitudes(leftMagnitude, rightMagnitude, product);
        return new Signed704(
            product[10], product[9], product[8], product[7], product[6], product[5],
            product[4], product[3], product[2], product[1], product[0]);
    }

    /// <summary>
    /// Returns the exact floor square root of a nonnegative eleven-word value.
    /// </summary>
    internal static Signed576 GetFloorSquareRoot(Signed704 value)
    {
        if (value.IsZero)
            return default;

        Span<ulong> root = stackalloc ulong[6];
        Span<ulong> remainder = stackalloc ulong[6];
        Span<ulong> candidate = stackalloc ulong[6];
        root.Clear();
        remainder.Clear();
        for (int pairIndex = (GetBitLength(value) - 1) >> 1; pairIndex >= 0; pairIndex--)
        {
            ShiftLeftMagnitude(remainder, 2);
            remainder[0] |= GetBitPair(value, pairIndex);
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
            0UL, 0UL, 0UL,
            root[5], root[4], root[3], root[2], root[1], root[0]);
    }

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    internal static Signed704 AddSigned704(Signed704 left, Signed704 right)
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
        ulong word10 = unchecked(left.Word10 + right.Word10 + carry);
        return new Signed704(word10, word9, word8, word7, word6, word5, word4, word3, word2, word1, word0);
    }

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    internal static Signed704 SubtractSigned704(Signed704 left, Signed704 right)
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
        ulong word10 = unchecked(left.Word10 - right.Word10 - borrow);
        return new Signed704(
            word10,
            word9,
            word8,
            word7,
            word6,
            word5,
            word4,
            word3,
            word2,
            word1,
            word0);
    }

    internal static int CompareNonNegative(
        Signed704 left,
        Signed704 right)
    {
        if (left.Word10 != right.Word10) return left.Word10 < right.Word10 ? -1 : 1;
        if (left.Word9 != right.Word9) return left.Word9 < right.Word9 ? -1 : 1;
        if (left.Word8 != right.Word8) return left.Word8 < right.Word8 ? -1 : 1;
        if (left.Word7 != right.Word7) return left.Word7 < right.Word7 ? -1 : 1;
        if (left.Word6 != right.Word6) return left.Word6 < right.Word6 ? -1 : 1;
        if (left.Word5 != right.Word5) return left.Word5 < right.Word5 ? -1 : 1;
        if (left.Word4 != right.Word4) return left.Word4 < right.Word4 ? -1 : 1;
        if (left.Word3 != right.Word3) return left.Word3 < right.Word3 ? -1 : 1;
        if (left.Word2 != right.Word2) return left.Word2 < right.Word2 ? -1 : 1;
        if (left.Word1 != right.Word1) return left.Word1 < right.Word1 ? -1 : 1;
        return left.Word0 == right.Word0
            ? 0
            : left.Word0 < right.Word0 ? -1 : 1;
    }

    /// <summary>
    /// Multiplies three five-word finite-axis factors whose proven combined
    /// magnitude is below 650 bits.
    /// </summary>
    internal static Signed704 MultiplySigned320(
        Signed320 first,
        Signed320 second,
        Signed320 third)
    {
        Span<ulong> firstMagnitude = stackalloc ulong[5];
        Span<ulong> secondMagnitude = stackalloc ulong[5];
        Span<ulong> thirdMagnitude = stackalloc ulong[5];
        CopyMagnitude(first, firstMagnitude);
        CopyMagnitude(second, secondMagnitude);
        CopyMagnitude(third, thirdMagnitude);

        Span<ulong> firstProduct = stackalloc ulong[10];
        MultiplyMagnitudes(firstMagnitude, secondMagnitude, firstProduct);
        Span<ulong> product = stackalloc ulong[15];
        MultiplyMagnitudes(firstProduct, thirdMagnitude, product);

        if (first.Sign * second.Sign * third.Sign < 0)
        {
            ulong carry = 1UL;
            for (int index = 0; index < 11; index++)
                product[index] = AddSignedWord(~product[index], 0UL, ref carry);
        }

        return new Signed704(
            product[10],
            product[9],
            product[8],
            product[7],
            product[6],
            product[5],
            product[4],
            product[3],
            product[2],
            product[1],
            product[0]);
    }

    private static void CopyMagnitude(Signed320 value, Span<ulong> destination)
    {
        GetMagnitude(value, out destination[4], out destination[3], out destination[2], out destination[1], out destination[0]);
    }

    private static Signed704 CreateSigned704(ReadOnlySpan<ulong> words) =>
        new(
            words[10], words[9], words[8], words[7], words[6], words[5],
            words[4], words[3], words[2], words[1], words[0]);

    private static void ApplySigned704Sign(Span<ulong> words, bool negative)
    {
        if (!negative)
            return;

        ulong carry = 1UL;
        for (int index = 0; index < 11; index++)
            words[index] = AddSignedWord(~words[index], 0UL, ref carry);
    }

    private static int GetBitLength(Signed704 value)
    {
        if (value.Word10 != 0UL) return 704 - Fixed64.CountLeadingZeroes(value.Word10);
        if (value.Word9 != 0UL) return 640 - Fixed64.CountLeadingZeroes(value.Word9);
        if (value.Word8 != 0UL) return 576 - Fixed64.CountLeadingZeroes(value.Word8);
        if (value.Word7 != 0UL) return 512 - Fixed64.CountLeadingZeroes(value.Word7);
        if (value.Word6 != 0UL) return 448 - Fixed64.CountLeadingZeroes(value.Word6);
        if (value.Word5 != 0UL) return 384 - Fixed64.CountLeadingZeroes(value.Word5);
        if (value.Word4 != 0UL) return 320 - Fixed64.CountLeadingZeroes(value.Word4);
        if (value.Word3 != 0UL) return 256 - Fixed64.CountLeadingZeroes(value.Word3);
        if (value.Word2 != 0UL) return 192 - Fixed64.CountLeadingZeroes(value.Word2);
        if (value.Word1 != 0UL) return 128 - Fixed64.CountLeadingZeroes(value.Word1);
        return 64 - Fixed64.CountLeadingZeroes(value.Word0);
    }

    private static int GetBitLength(ReadOnlySpan<ulong> value)
    {
        int index = value.Length - 1;
        while (value[index] == 0UL)
            index--;
        return (index * 64) + 64 - Fixed64.CountLeadingZeroes(value[index]);
    }

    private static ulong GetBitPair(Signed704 value, int pairIndex)
    {
        int bitIndex = pairIndex << 1;
        int shift = bitIndex & 63;
        ulong word = (bitIndex >> 6) switch
        {
            0 => value.Word0,
            1 => value.Word1,
            2 => value.Word2,
            3 => value.Word3,
            4 => value.Word4,
            5 => value.Word5,
            6 => value.Word6,
            7 => value.Word7,
            8 => value.Word8,
            9 => value.Word9,
            _ => value.Word10,
        };
        return (word >> shift) & 3UL;
    }

    private static ulong GetBitPair(ReadOnlySpan<ulong> value, int pairIndex)
    {
        int bitIndex = pairIndex << 1;
        return (value[bitIndex >> 6] >> (bitIndex & 63)) & 3UL;
    }

    private static void ShiftLeftMagnitude(Span<ulong> value, int bits)
    {
        ulong carry = 0UL;
        for (int index = 0; index < value.Length; index++)
        {
            ulong nextCarry = value[index] >> (64 - bits);
            value[index] = (value[index] << bits) | carry;
            carry = nextCarry;
        }
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
            value[index] = SubtractWord(value[index], subtract[index], ref borrow);
    }
}
