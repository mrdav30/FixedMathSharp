//=======================================================================
// WideArithmetic.Signed576.cs
//=======================================================================
// MIT License, Copyright (c) 2024–present David Oravsky (mrdav30)
// See LICENSE file in the project root for full license information.
//=======================================================================

using System;
using System.Runtime.CompilerServices;

namespace FixedMathSharp;

/// <content>
/// Arithmetic helpers for <see cref="Signed576"/>: magnitude extraction, comparison,
/// and exact addition/subtraction operating directly on the nine-word representation.
/// </content>
internal static partial class WideArithmetic
{
    internal static void GetMagnitude(Signed576 value, Span<ulong> magnitude)
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
        if (value.Sign >= 0)
            return;

        ulong carry = 1UL;
        for (int index = 0; index < 9; index++)
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

    internal static Signed576 Double(Signed576 value) => AddSigned576(value, value);

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

    internal static Signed576 MultiplySigned320(Signed320 left, Signed192 right) =>
        MultiplySigned576(Signed576.ExtendValue(left), right);

    /// <summary>
    /// Multiplies signed five-word values in the finite-axis domain exactly.
    /// </summary>
    /// <remarks>
    /// Callers guarantee magnitudes below 262 bits, so the product is below
    /// 523 bits and cannot overflow the nine-word result.
    /// </remarks>
    internal static Signed576 MultiplySigned320(Signed320 left, Signed320 right)
    {
        if (Signed192.TryNarrowSigned(left, out Signed192 narrowLeft)
            && Signed192.TryNarrowSigned(right, out Signed192 narrowRight)
            && GetMagnitudeBitLength(narrowLeft) + GetMagnitudeBitLength(narrowRight) <= 319)
        {
            return Signed576.ExtendValue(MultiplySigned192(narrowLeft, narrowRight));
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
    /// Multiplies a signed nine-word value by a signed one-word value whose
    /// proven product fits in nine words.
    /// </summary>
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    internal static Signed576 MultiplySigned576(Signed576 left, long right)
    {
        ulong factor = Fixed64.AbsToUInt64(right);
        ulong carry = 0UL;
        ulong word0 = MultiplySigned576Word(left.Word0, factor, ref carry);
        ulong word1 = MultiplySigned576Word(left.Word1, factor, ref carry);
        ulong word2 = MultiplySigned576Word(left.Word2, factor, ref carry);
        ulong word3 = MultiplySigned576Word(left.Word3, factor, ref carry);
        ulong word4 = MultiplySigned576Word(left.Word4, factor, ref carry);
        ulong word5 = MultiplySigned576Word(left.Word5, factor, ref carry);
        ulong word6 = MultiplySigned576Word(left.Word6, factor, ref carry);
        ulong word7 = MultiplySigned576Word(left.Word7, factor, ref carry);
        ulong word8 = MultiplySigned576Word(left.Word8, factor, ref carry);
        Signed576 product = new(
            word8, word7, word6, word5, word4, word3, word2, word1, word0);
        return right < 0L
            ? SubtractSigned576(default, product)
            : product;
    }

    /// <summary>
    /// Multiplies a signed nine-word value by a signed three-word value whose
    /// proven product fits in nine words.
    /// </summary>
    internal static Signed576 MultiplySigned576(Signed576 left, Signed192 right)
    {
        if (Signed192.TryNarrowSigned(left, out Signed192 narrowLeft)
            && GetMagnitudeBitLength(narrowLeft) + GetMagnitudeBitLength(right) <= 319)
            return Signed576.ExtendValue(MultiplySigned192(narrowLeft, right));
        if (Signed320.TryNarrowSigned(left, out Signed320 mediumLeft))
            return MultiplySigned320(mediumLeft, Signed320.ExtendValue(right));

        Span<ulong> leftMagnitude = stackalloc ulong[9];
        Span<ulong> rightMagnitude = stackalloc ulong[3];
        GetMagnitude(left, leftMagnitude);
        GetMagnitude(right, out rightMagnitude[2], out rightMagnitude[1], out rightMagnitude[0]);
        Span<ulong> product = stackalloc ulong[12];
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

    internal static Signed576 MultiplySigned576(Signed576 value, Signed192 first, Signed192 second) =>
        MultiplySigned576(
            MultiplySigned576(value, first),
            second);

    internal static Signed576 MultiplySigned576(
        Signed576 value,
        Signed192 first,
        Signed192 second,
        Signed192 third) =>
        MultiplySigned576(
            MultiplySigned576(value, first, second),
            third);

    internal static Signed576 MultiplySigned576(
        Signed576 value,
        Signed192 first,
        Signed192 second,
        Signed192 third,
        Signed192 fourth) =>
        MultiplySigned576(
            MultiplySigned576(value, first, second, third),
            fourth);

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
            return Signed320.ExtendValue(narrowRoot);
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
            if (CompareMagnitudeEqualLength(remainder, candidate) < 0)
                continue;

            SubtractEqualMagnitudes(remainder, candidate, remainder);
            root[0]++;
        }

        return new Signed320(rootStorage[4], rootStorage[3], rootStorage[2], rootStorage[1], rootStorage[0]);
    }

    internal static Signed320 Negate(Signed320 value) => SubtractSigned320(default, value);

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    internal static Signed576 Absolute(Signed576 value) =>
        value.Sign < 0
            ? SubtractSigned576(default, value)
            : value;

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    internal static Signed576 ClampToNonNegative(Signed576 value)
    {
        // Two's-complement sign masking keeps this hot-path clamp branchless.
        ulong mask = ~unchecked((ulong)((long)value.Word8 >> 63));
        return new Signed576(
            value.Word8 & mask,
            value.Word7 & mask,
            value.Word6 & mask,
            value.Word5 & mask,
            value.Word4 & mask,
            value.Word3 & mask,
            value.Word2 & mask,
            value.Word1 & mask,
            value.Word0 & mask);
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

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    private static ulong MultiplySigned576Word(
        ulong value,
        ulong factor,
        ref ulong carry)
    {
        Fixed64.Multiply64To128(
            value,
            factor,
            out ulong high,
            out ulong low);
        ulong product = unchecked(low + carry);
        carry = unchecked(high + (product < low ? 1UL : 0UL));
        return product;
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

    private static Signed576 MultiplyNonNegativeToSigned576(
        Signed576 left,
        Signed576 right)
    {
        Span<ulong> leftWords = stackalloc ulong[9];
        Span<ulong> rightWords = stackalloc ulong[9];
        Span<ulong> product = stackalloc ulong[18];
        GetMagnitude(left, leftWords);
        GetMagnitude(right, rightWords);
        MultiplyMagnitudes(leftWords, rightWords, product);
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
}
