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
        ulong extension = unchecked((ulong)((long)value.Word4 >> 63));
        return new Signed576(
            extension, extension, extension, extension,
            value.Word4, value.Word3, value.Word2, value.Word1, value.Word0);
    }

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    internal static bool TryNarrowSigned320(Signed576 value, out Signed320 result)
    {
        ulong extension = (value.Word4 & (1UL << 63)) != 0UL ? ulong.MaxValue : 0UL;
        if (value.Word8 != extension || value.Word7 != extension
            || value.Word6 != extension || value.Word5 != extension)
        {
            result = default;
            return false;
        }

        result = new Signed320(value.Word4, value.Word3, value.Word2, value.Word1, value.Word0);
        return true;
    }

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    internal static bool TryNarrowSigned192(Signed576 value, out Signed192 result)
    {
        ulong extension = (value.Word2 & (1UL << 63)) != 0UL ? ulong.MaxValue : 0UL;
        if (value.Word8 != extension || value.Word7 != extension || value.Word6 != extension
            || value.Word5 != extension || value.Word4 != extension || value.Word3 != extension)
        {
            result = default;
            return false;
        }

        result = new Signed192(value.Word2, value.Word1, value.Word0);
        return true;
    }

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
        if (TryNarrowSigned192(left, out Signed192 narrowLeft)
            && TryNarrowSigned192(right, out Signed192 narrowRight)
            && GetMagnitudeBitLength(narrowLeft) + GetMagnitudeBitLength(narrowRight) <= 319)
        {
            return ExtendToSigned576(MultiplySigned192(narrowLeft, narrowRight));
        }

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

        MultiplyMagnitudes(leftMagnitude, rightMagnitude, product);

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
    /// Multiplies a signed nine-word value by a signed three-word value whose
    /// proven product fits in nine words.
    /// </summary>
    internal static Signed576 MultiplySigned576(Signed576 left, Signed192 right)
    {
        if (TryNarrowSigned192(left, out Signed192 narrowLeft)
            && GetMagnitudeBitLength(narrowLeft) + GetMagnitudeBitLength(right) <= 319)
            return ExtendToSigned576(MultiplySigned192(narrowLeft, right));
        if (TryNarrowSigned320(left, out Signed320 mediumLeft))
            return MultiplySigned320(mediumLeft, ExtendToSigned320(right));

        Span<ulong> leftMagnitude = stackalloc ulong[9];
        Span<ulong> rightMagnitude = stackalloc ulong[3];
        GetMagnitude(left, leftMagnitude);
        GetMagnitude(right, out rightMagnitude[2], out rightMagnitude[1], out rightMagnitude[0]);
        Span<ulong> product = stackalloc ulong[12];
        product.Clear();
        MultiplyMagnitudes(leftMagnitude, rightMagnitude, product);

        if (left.Sign * right.Sign < 0)
        {
            ulong carry = 1UL;
            for (int index = 0; index < 9; index++)
                product[index] = AddSignedWord(~product[index], 0UL, ref carry);
        }

        return new Signed576(
            product[8], product[7], product[6], product[5], product[4],
            product[3], product[2], product[1], product[0]);
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

        if ((value.Word8 | value.Word7 | value.Word6 | value.Word5 | value.Word4) == 0UL
            && (value.Word3 & (1UL << 63)) == 0UL)
        {
            Signed320 shifted = new(value.Word3, value.Word2, value.Word1, value.Word0, 0UL);
            Signed192 narrowRoot = GetFloorSquareRoot(shifted, out _);
            return ExtendToSigned320(narrowRoot);
        }

        int bitLength = GetBitLength(value);
        int rootBitLength = ((bitLength + 1) >> 1) + FixedMath.SHIFT_AMOUNT_I;
        int activeWords = System.Math.Min(5, (rootBitLength + 64) >> 6);
        Span<ulong> rootStorage = stackalloc ulong[5];
        Span<ulong> remainderStorage = stackalloc ulong[5];
        Span<ulong> candidateStorage = stackalloc ulong[5];
        Span<ulong> root = rootStorage[..activeWords];
        Span<ulong> remainder = remainderStorage[..activeWords];
        Span<ulong> candidate = candidateStorage[..activeWords];
        rootStorage.Clear();
        remainder.Clear();
        int pairIndex = (bitLength - 1) >> 1;
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

        return new Signed320(rootStorage[4], rootStorage[3], rootStorage[2], rootStorage[1], rootStorage[0]);
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
        // Values below 2^255 use the Signed320 restoring-square-root path.
        // Reaching this wide path therefore proves Word3's high bit is set.
        return 256 - Fixed64.CountLeadingZeroes(value.Word3);
    }

    private static int GetMagnitudeBitLength(Signed192 value)
    {
        GetMagnitude(value, out ulong high, out ulong middle, out ulong low);
        if (high != 0UL)
            return 128 + 64 - Fixed64.CountLeadingZeroes(high);
        if (middle != 0UL)
            return 64 + 64 - Fixed64.CountLeadingZeroes(middle);
        return low == 0UL ? 0 : 64 - Fixed64.CountLeadingZeroes(low);
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
