//=======================================================================
// WideArithmetic.Magnitude.cs
//=======================================================================
// MIT License, Copyright (c) 2024–present David Oravsky (mrdav30)
// See LICENSE file in the project root for full license information.
//=======================================================================

using System;
using System.Runtime.CompilerServices;

namespace FixedMathSharp;

/// <content>
/// Policy-neutral unsigned magnitude-span arithmetic.
/// </content>
internal static partial class WideArithmetic
{
    internal static int GetActiveMagnitudeLength(
        ReadOnlySpan<ulong> value)
    {
        int length = value.Length;
        while (length > 0 && value[length - 1] == 0UL)
            length--;
        return length;
    }

    internal static int GetMagnitudeBitLength(
        ReadOnlySpan<ulong> value)
    {
        int length = GetActiveMagnitudeLength(value);
        return length == 0
            ? 0
            : ((length - 1) << 6)
                + 64
                - Fixed64.CountLeadingZeroes(value[length - 1]);
    }

    internal static void MultiplyMagnitudes(
        ReadOnlySpan<ulong> left,
        ReadOnlySpan<ulong> right,
        Span<ulong> product)
    {
        product.Clear();
        int leftLength = GetActiveMagnitudeLength(left);
        int rightLength = GetActiveMagnitudeLength(right);
        int retainedLeft = Math.Min(leftLength, product.Length);
        for (int leftIndex = 0; leftIndex < retainedLeft; leftIndex++)
        {
            ulong leftWord = left[leftIndex];
            if (leftWord == 0UL)
                continue;

            ulong carry = 0;
            int retainedRight = Math.Min(rightLength, product.Length - leftIndex);
            for (int rightIndex = 0; rightIndex < retainedRight; rightIndex++)
            {
                int productIndex = leftIndex + rightIndex;
                ulong rightWord = right[rightIndex];
                if (rightWord == 0)
                {
                    // A sparse zero limb still owns the preceding carry.
                    // Flush it once before skipping this multiplication.
                    AddWord(product, productIndex, carry);
                    carry = 0;
                    continue;
                }
                Fixed64.Multiply64To128(
                    leftWord,
                    rightWord,
                    out ulong high,
                    out ulong low);
                ulong sum = unchecked(low + product[productIndex]);
                high += sum < low ? 1UL : 0UL;
                ulong value = unchecked(sum + carry);
                carry = high + (value < sum ? 1UL : 0UL);
                product[productIndex] = value;
            }
            // One carry per row replaces two ripple additions per limb.
            // A limb product plus the old word and carry is at most 2^128-1,
            // so the next carry always fits. Preserve low-word truncation.
            AddWord(product, leftIndex + retainedRight, carry);
        }
    }

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    internal static void AddWord(
        Span<ulong> words,
        int index,
        ulong value)
    {
        while (value != 0UL && index < words.Length)
        {
            ulong previous = words[index];
            words[index] = unchecked(previous + value);
            value = words[index] < previous ? 1UL : 0UL;
            index++;
        }
    }

    internal static void AddEqualMagnitudes(
        ReadOnlySpan<ulong> left,
        ReadOnlySpan<ulong> right,
        Span<ulong> result)
    {
        ulong carry = 0UL;
        for (int index = 0; index < result.Length; index++)
            result[index] =
                AddSignedWord(left[index], right[index], ref carry);
    }

    internal static void AddMagnitudeInto(
        ReadOnlySpan<ulong> addend,
        Span<ulong> result)
    {
        ulong carry = 0UL;
        for (int index = 0; index < result.Length; index++)
        {
            ulong value = index < addend.Length
                ? addend[index]
                : 0UL;
            result[index] =
                AddSignedWord(result[index], value, ref carry);
        }
    }

    internal static void SubtractEqualMagnitudes(
        ReadOnlySpan<ulong> larger,
        ReadOnlySpan<ulong> smaller,
        Span<ulong> result)
    {
        ulong borrow = 0UL;
        for (int index = 0; index < result.Length; index++)
            result[index] =
                SubtractWord(larger[index], smaller[index], ref borrow);
    }

    internal static int CompareMagnitudeEqualLength(
        ReadOnlySpan<ulong> left,
        ReadOnlySpan<ulong> right)
    {
        for (int index = left.Length - 1; index >= 0; index--)
        {
            if (left[index] != right[index])
                return left[index] < right[index] ? -1 : 1;
        }

        return 0;
    }

    /// <summary>
    /// Accumulates a signed magnitude shifted left by a nonnegative bit count.
    /// Source and destination are disjoint; the destination has proven room
    /// for the complete result. No shifted copy or subtraction buffer is made.
    /// </summary>
    internal static void AddShiftedSignedMagnitude(ReadOnlySpan<ulong> addend,
        int addendSign, int shift, Span<ulong> result, ref int resultSign)
    {
        if (addendSign == 0)
            return;
        int wordShift = shift >> 6;
        int bitShift = shift & 63;
        if (resultSign == 0)
        {
            for (int index = 0; index < result.Length; index++)
                result[index] = GetShiftedMagnitudeWord(addend, index, wordShift, bitShift);
            resultSign = addendSign;
            return;
        }
        if (resultSign == addendSign)
        {
            ulong carry = 0;
            for (int index = 0; index < result.Length; index++)
                result[index] = AddSignedWord(result[index],
                    GetShiftedMagnitudeWord(addend, index, wordShift, bitShift), ref carry);
            return;
        }

        int comparison = 0;
        for (int index = result.Length - 1; index >= 0 && comparison == 0; index--)
            comparison = result[index].CompareTo(GetShiftedMagnitudeWord(addend, index, wordShift, bitShift));
        if (comparison == 0)
        {
            result.Clear();
            resultSign = 0;
            return;
        }
        ulong borrow = 0;
        for (int index = 0; index < result.Length; index++)
        {
            ulong shifted = GetShiftedMagnitudeWord(addend, index, wordShift, bitShift);
            result[index] = comparison > 0
                ? SubtractWord(result[index], shifted, ref borrow)
                : SubtractWord(shifted, result[index], ref borrow);
        }
        if (comparison < 0)
            resultSign = addendSign;
    }

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    private static ulong GetShiftedMagnitudeWord(ReadOnlySpan<ulong> source,
        int index, int wordShift, int bitShift)
    {
        int lowIndex = index - wordShift;
        ulong low = (uint)lowIndex < (uint)source.Length ? source[lowIndex] : 0UL;
        ulong high = bitShift != 0 && (uint)(lowIndex - 1) < (uint)source.Length
            ? source[lowIndex - 1] >> (64 - bitShift) : 0UL;
        return (low << bitShift) | high;
    }
}
