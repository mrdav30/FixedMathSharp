//=======================================================================
// WideArithmetic.Signed576.cs
//=======================================================================
// MIT License, Copyright (c) 2024–present David Oravsky (mrdav30)
// See LICENSE file in the project root for full license information.
//=======================================================================

using System;
using System.Runtime.CompilerServices;

namespace FixedMathSharp;

internal static partial class WideArithmetic
{
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    internal static Signed576 ExtendToSigned576(Signed320 value)
    {
        ulong extension = value.Sign < 0 ? ulong.MaxValue : 0UL;
        return new Signed576(
            extension, extension, extension, extension,
            value.Word4, value.Word3, value.Word2, value.Word1, value.Word0);
    }

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    internal static Signed576 ExtendToSigned576(Signed192 value) =>
        ExtendToSigned576(ExtendToSigned320(value));

    internal static void GetMagnitude(Signed576 value, Span<ulong> magnitude)
    {
        magnitude[0] = value.Word0;
        magnitude[1] = value.Word1;
        magnitude[2] = value.Word2;
        magnitude[3] = value.Word3;
        magnitude[4] = value.Word4;
        magnitude[5] = value.Word5;
        magnitude[6] = value.Word6;
        magnitude[7] = value.Word7;
        magnitude[8] = value.Word8;
        if (value.Sign >= 0)
            return;

        ulong carry = 1UL;
        for (int index = 0; index < magnitude.Length; index++)
            magnitude[index] = AddSignedWord(~magnitude[index], 0UL, ref carry);
    }

    /// <summary>
    /// Compares two nonnegative nine-word values.
    /// </summary>
    internal static int CompareNonNegative(Signed576 left, Signed576 right)
    {
        if (left.Word8 != right.Word8) return left.Word8 < right.Word8 ? -1 : 1;
        if (left.Word7 != right.Word7) return left.Word7 < right.Word7 ? -1 : 1;
        if (left.Word6 != right.Word6) return left.Word6 < right.Word6 ? -1 : 1;
        if (left.Word5 != right.Word5) return left.Word5 < right.Word5 ? -1 : 1;
        if (left.Word4 != right.Word4) return left.Word4 < right.Word4 ? -1 : 1;
        if (left.Word3 != right.Word3) return left.Word3 < right.Word3 ? -1 : 1;
        if (left.Word2 != right.Word2) return left.Word2 < right.Word2 ? -1 : 1;
        if (left.Word1 != right.Word1) return left.Word1 < right.Word1 ? -1 : 1;
        return left.Word0 == right.Word0 ? 0 : left.Word0 < right.Word0 ? -1 : 1;
    }

    /// <summary>
    /// Adds exact nine-word values without scalar conversion.
    /// </summary>
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    internal static Signed576 AddSigned576(Signed576 left, Signed576 right)
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
        ulong word8 = unchecked(left.Word8 + right.Word8 + carry);
        return new Signed576(word8, word7, word6, word5, word4, word3, word2, word1, word0);
    }

    /// <summary>
    /// Subtracts exact nine-word values without scalar conversion.
    /// </summary>
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    internal static Signed576 SubtractSigned576(Signed576 left, Signed576 right)
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
        ulong word8 = unchecked(left.Word8 - right.Word8 - borrow);
        return new Signed576(word8, word7, word6, word5, word4, word3, word2, word1, word0);
    }

    /// <summary>
    /// Multiplies signed five-word values in the finite-axis domain exactly.
    /// </summary>
    /// <remarks>
    /// Callers guarantee magnitudes below 262 bits, so the product is below
    /// 523 bits and cannot overflow the nine-word result.
    /// </remarks>
    internal static Signed576 MultiplySigned320(Signed320 left, Signed320 right)
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

        Span<ulong> leftMagnitude = stackalloc ulong[5]
        {
            leftWord0,
            leftWord1,
            leftWord2,
            leftWord3,
            leftWord4,
        };
        Span<ulong> rightMagnitude = stackalloc ulong[5]
        {
            rightWord0,
            rightWord1,
            rightWord2,
            rightWord3,
            rightWord4,
        };
        Span<ulong> product = stackalloc ulong[9];
        product.Clear();

        for (int leftIndex = 0; leftIndex < leftMagnitude.Length; leftIndex++)
        {
            for (int rightIndex = 0; rightIndex < rightMagnitude.Length; rightIndex++)
            {
                Fixed64.Multiply64To128(
                    leftMagnitude[leftIndex],
                    rightMagnitude[rightIndex],
                    out ulong high,
                    out ulong low);
                int productIndex = leftIndex + rightIndex;
                AddWord(product, productIndex, low);
                AddWord(product, productIndex + 1, high);
            }
        }

        if (left.Sign * right.Sign < 0)
        {
            ulong carry = 1UL;
            for (int index = 0; index < product.Length; index++)
                product[index] = AddSignedWord(~product[index], 0UL, ref carry);
        }

        return new Signed576(
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

    /// <summary>
    /// Returns the exact floor of the square root after applying the Q32.32
    /// parameter scale: <c>floor(sqrt(value * 2^64))</c>. The caller must
    /// prove <paramref name="value"/> is nonnegative and less than 2^523 so
    /// the scaled root and restoring remainder fit their five-word storage.
    /// </summary>
    internal static Signed320 GetFloorSquareRootScaledByFixed64(Signed576 value)
    {
        if (value.IsZero)
            return default;

        Span<ulong> root = stackalloc ulong[5];
        Span<ulong> remainder = stackalloc ulong[5];
        Span<ulong> candidate = stackalloc ulong[5];
        root.Clear();
        remainder.Clear();
        int pairIndex = (GetBitLength(value) - 1) >> 1;
        for (; pairIndex >= -FixedMath.SHIFT_AMOUNT_I; pairIndex--)
        {
            ShiftLeft(remainder, 2);
            if (pairIndex >= 0)
                remainder[0] |= GetBitPair(value, pairIndex);
            ShiftLeft(root, 1);

            root.CopyTo(candidate);
            ShiftLeft(candidate, 1);
            candidate[0] |= 1UL;
            if (CompareUnsigned(remainder, candidate) < 0)
                continue;

            SubtractUnsigned(remainder, candidate);
            root[0]++;
        }

        return new Signed320(root[4], root[3], root[2], root[1], root[0]);
    }

    private static int GetBitLength(Signed576 value)
    {
        if (value.Word8 != 0UL)
            return 576 - Fixed64.CountLeadingZeroes(value.Word8);
        if (value.Word7 != 0UL)
            return 512 - Fixed64.CountLeadingZeroes(value.Word7);
        if (value.Word6 != 0UL)
            return 448 - Fixed64.CountLeadingZeroes(value.Word6);
        if (value.Word5 != 0UL)
            return 384 - Fixed64.CountLeadingZeroes(value.Word5);
        if (value.Word4 != 0UL)
            return 320 - Fixed64.CountLeadingZeroes(value.Word4);
        if (value.Word3 != 0UL)
            return 256 - Fixed64.CountLeadingZeroes(value.Word3);
        if (value.Word2 != 0UL)
            return 192 - Fixed64.CountLeadingZeroes(value.Word2);
        if (value.Word1 != 0UL)
            return 128 - Fixed64.CountLeadingZeroes(value.Word1);
        return 64 - Fixed64.CountLeadingZeroes(value.Word0);
    }

    private static ulong GetBitPair(Signed576 value, int pairIndex)
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
            _ => value.Word8,
        };
        return (word >> shift) & 3UL;
    }

    private static void ShiftLeft(Span<ulong> value, int bits)
    {
        ulong carry = 0UL;
        for (int index = 0; index < value.Length; index++)
        {
            ulong nextCarry = value[index] >> (64 - bits);
            value[index] = (value[index] << bits) | carry;
            carry = nextCarry;
        }
    }

    private static int CompareUnsigned(ReadOnlySpan<ulong> left, ReadOnlySpan<ulong> right)
    {
        for (int index = left.Length - 1; index >= 0; index--)
        {
            if (left[index] != right[index])
                return left[index] < right[index] ? -1 : 1;
        }

        return 0;
    }

    private static void SubtractUnsigned(Span<ulong> value, ReadOnlySpan<ulong> subtract)
    {
        ulong borrow = 0UL;
        for (int index = 0; index < value.Length; index++)
            value[index] = SubtractWord(value[index], subtract[index], ref borrow);
    }

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    private static ulong AddSignedWord(ulong left, ulong right, ref ulong carry)
    {
        ulong sum = unchecked(left + right);
        ulong result = unchecked(sum + carry);
        carry = sum < left || result < sum ? 1UL : 0UL;
        return result;
    }
}
