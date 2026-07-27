//=======================================================================
// WideArithmetic.Signed192.cs
//=======================================================================
// MIT License, Copyright (c) 2024–present David Oravsky (mrdav30)
// See LICENSE file in the project root for full license information.
//=======================================================================

using System.Runtime.CompilerServices;

namespace FixedMathSharp;

/// <content>
/// Arithmetic helpers for <see cref="Signed192"/>: magnitude/comparison,
/// exact add/subtract, and integer square root extraction.
/// </content>
internal static partial class WideArithmetic
{
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

    internal static Signed192 Double(Signed192 value) => AddSigned192(value, value);

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

    internal static Signed192 Difference(Fixed64 left, Fixed64 right) =>
        SubtractSigned192(Signed192.Signed(left.m_rawValue), Signed192.Signed(right.m_rawValue));

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

    internal static Signed192 Scale(Fixed64 value) =>
        Signed192.NarrowValue(
            MultiplySigned192(
                Signed192.Raw(value),
                Signed192.One));

    internal static Signed192 Negate(Signed192 value) => SubtractSigned192(default, value);

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    internal static Signed192 Absolute(Signed192 value)
    {
        if (value.Sign >= 0)
            return value;
        return SubtractSigned192(default, value);
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
    internal static void ShiftLeftOne(ref ulong high, ref ulong middle, ref ulong low)
    {
        high = (high << 1) | (middle >> 63);
        middle = (middle << 1) | (low >> 63);
        low <<= 1;
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
}
