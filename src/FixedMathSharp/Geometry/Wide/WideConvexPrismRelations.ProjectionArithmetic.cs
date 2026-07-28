//=======================================================================
// WideConvexPrismRelations.ProjectionArithmetic.cs
//=======================================================================
// MIT License, Copyright (c) 2024–present David Oravsky (mrdav30)
// See LICENSE file in the project root for full license information.
//=======================================================================

using System;

namespace FixedMathSharp.Geometry;

/// <content>
/// Wide-precision (multi-word) arithmetic helpers used to build and manipulate
/// magnitude spans for high-precision projection products, including
/// multiplication, addition, subtraction, shifting, and comparison of
/// arbitrary-length unsigned word arrays.
/// </content>
internal static partial class WideConvexPrismRelations
{
    private static void BuildProduct(
        Signed704 first,
        Signed704 second,
        Span<ulong> result)
    {
        Span<ulong> firstWords = stackalloc ulong[11];
        Span<ulong> secondWords = stackalloc ulong[11];
        WideArithmetic.GetMagnitude(first, firstWords);
        WideArithmetic.GetMagnitude(second, secondWords);
        MultiplyMagnitudes(firstWords, secondWords, result);
    }

    private static void BuildProduct(
        Signed704 first,
        Signed704 second,
        Signed320 third,
        Span<ulong> result)
    {
        BuildProduct(first, second, result);
        MultiplyBy(result, third);
    }

    private static void BuildProduct(
        Signed320 first,
        Signed320 second,
        Signed576 third,
        Span<ulong> result)
    {
        Span<ulong> firstWords = stackalloc ulong[5];
        Span<ulong> secondWords = stackalloc ulong[5];
        GetMagnitude(first, firstWords);
        GetMagnitude(second, secondWords);
        MultiplyMagnitudes(firstWords, secondWords, result);
        MultiplyBy(result, third);
    }

    private static void BuildProduct(
        Signed320 first,
        Signed320 second,
        Signed832 third,
        Span<ulong> result)
    {
        Span<ulong> firstWords = stackalloc ulong[5];
        Span<ulong> secondWords = stackalloc ulong[5];
        Span<ulong> thirdWords = stackalloc ulong[13];
        GetMagnitude(first, firstWords);
        GetMagnitude(second, secondWords);
        WideArithmetic.GetMagnitude(third, thirdWords);
        Span<ulong> intermediate = stackalloc ulong[40];
        MultiplyMagnitudes(firstWords, secondWords, intermediate);
        MultiplyMagnitudes(intermediate, thirdWords, result);
    }

    private static void BuildProduct(
        Signed320 first,
        Signed320 second,
        Signed320 third,
        Signed576 fourth,
        Span<ulong> result)
    {
        Span<ulong> firstWords = stackalloc ulong[5];
        Span<ulong> secondWords = stackalloc ulong[5];
        GetMagnitude(first, firstWords);
        GetMagnitude(second, secondWords);
        MultiplyMagnitudes(firstWords, secondWords, result);
        MultiplyBy(result, third);
        MultiplyBy(result, fourth);
    }

    private static void MultiplyBy(
        Span<ulong> value,
        Signed320 factor)
    {
        Span<ulong> words = stackalloc ulong[5];
        GetMagnitude(factor, words);
        MultiplyBy(value, words);
    }

    private static void MultiplyBy(
        Span<ulong> value,
        Signed576 factor)
    {
        Span<ulong> words = stackalloc ulong[9];
        WideArithmetic.GetMagnitude(factor, words);
        MultiplyBy(value, words);
    }

    private static void MultiplyBy(
        Span<ulong> value,
        ReadOnlySpan<ulong> factor)
    {
        Span<ulong> product = stackalloc ulong[value.Length];
        MultiplyMagnitudes(value, factor, product);
        product.CopyTo(value);
    }

    private static void GetMagnitude(
        Signed320 value,
        Span<ulong> result) =>
        WideArithmetic.GetMagnitude(
            value,
            out result[4],
            out result[3],
            out result[2],
            out result[1],
            out result[0]);

    private static void MultiplyMagnitudes(
        ReadOnlySpan<ulong> left,
        ReadOnlySpan<ulong> right,
        Span<ulong> product)
    {
        product.Clear();
        int leftLength = GetActiveLength(left);
        int rightLength = GetActiveLength(right);
        for (int leftIndex = 0; leftIndex < leftLength; leftIndex++)
        {
            ulong leftWord = left[leftIndex];
            if (leftWord == 0UL)
                continue;
            for (int rightIndex = 0;
                rightIndex < rightLength;
                rightIndex++)
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
                WideArithmetic.AddWord(product, productIndex, low);
                WideArithmetic.AddWord(product, productIndex + 1, high);
            }
        }
    }

    private static void AddMagnitudes(
        ReadOnlySpan<ulong> left,
        ReadOnlySpan<ulong> right,
        Span<ulong> result)
    {
        left.CopyTo(result);
        WideArithmetic.AddMagnitudeInto(right, result);
    }

    private static void SubtractMagnitudes(
        ReadOnlySpan<ulong> left,
        ReadOnlySpan<ulong> right,
        Span<ulong> result)
    {
        WideArithmetic.SubtractEqualMagnitudes(left, right, result);
    }

    private static void ShiftLeft(Span<ulong> value, int bits)
    {
        for (int iteration = 0; iteration < bits; iteration++)
        {
            ulong carry = 0UL;
            for (int index = 0; index < value.Length; index++)
            {
                ulong nextCarry = value[index] >> 63;
                value[index] = (value[index] << 1) | carry;
                carry = nextCarry;
            }
        }
    }

    private static int CompareMagnitude(
        ReadOnlySpan<ulong> left,
        ReadOnlySpan<ulong> right)
    {
        for (int index = left.Length - 1; index >= 0; index--)
        {
            ulong leftWord = left[index];
            ulong rightWord = right[index];
            if (leftWord == rightWord)
                continue;
            return leftWord < rightWord ? -1 : 1;
        }
        return 0;
    }

    private static int GetActiveLength(ReadOnlySpan<ulong> value)
    {
        int length = value.Length;
        while (length > 0 && value[length - 1] == 0UL)
            length--;
        return length;
    }

    private static bool IsZero(ReadOnlySpan<ulong> value) =>
        GetActiveLength(value) == 0;
}
