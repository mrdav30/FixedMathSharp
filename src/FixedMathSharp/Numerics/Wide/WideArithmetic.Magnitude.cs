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
        for (int leftIndex = 0; leftIndex < leftLength; leftIndex++)
        {
            ulong leftWord = left[leftIndex];
            if (leftWord == 0UL)
                continue;

            for (int rightIndex = 0; rightIndex < rightLength; rightIndex++)
            {
                ulong rightWord = right[rightIndex];
                if (rightWord == 0UL)
                    continue;

                Fixed64.Multiply64To128(
                    leftWord,
                    rightWord,
                    out ulong high,
                    out ulong low);
                int productIndex = leftIndex + rightIndex;
                AddWord(product, productIndex, low);
                AddWord(product, productIndex + 1, high);
            }
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
}
