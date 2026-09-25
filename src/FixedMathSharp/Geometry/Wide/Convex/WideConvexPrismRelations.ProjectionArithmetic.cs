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
/// arbitrary-length unsigned word arrays and combining signed magnitudes.
/// </content>
internal static partial class WideConvexPrismRelations
{
    private static Signed576 GetMagnitude576(Signed576 value) =>
        value.Sign >= 0
            ? value
            : WideArithmetic.SubtractSigned576(default, value);

    private static void BuildProduct(
        Signed704 first,
        Signed704 second,
        Span<ulong> result)
    {
        Span<ulong> firstWords = stackalloc ulong[11];
        Span<ulong> secondWords = stackalloc ulong[11];
        WideArithmetic.GetMagnitude(first, firstWords);
        WideArithmetic.GetMagnitude(second, secondWords);
        WideArithmetic.MultiplyMagnitudes(firstWords, secondWords, result);
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
        WideArithmetic.MultiplyMagnitudes(firstWords, secondWords, result);
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
        WideArithmetic.MultiplyMagnitudes(firstWords, secondWords, intermediate);
        WideArithmetic.MultiplyMagnitudes(intermediate, thirdWords, result);
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
        WideArithmetic.MultiplyMagnitudes(firstWords, secondWords, result);
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
        WideArithmetic.MultiplyMagnitudes(value, factor, product);
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

    private static void CopyPositiveRadicand(
        ReadOnlySpan<ulong> radicand,
        Span<ulong> first,
        Span<ulong> second)
    {
        if (IsZero(first))
            radicand.CopyTo(first);
        else
            radicand.CopyTo(second);
    }

    private static void CombineWideSignedMagnitudes(
        ReadOnlySpan<ulong> left,
        int leftSign,
        ReadOnlySpan<ulong> right,
        int rightSign,
        Span<ulong> result,
        out int resultSign)
    {
        if (leftSign == 0 || IsZero(left))
        {
            right.CopyTo(result);
            resultSign = IsZero(right) ? 0 : rightSign;
            return;
        }
        if (rightSign == 0 || IsZero(right))
        {
            left.CopyTo(result);
            resultSign = leftSign;
            return;
        }
        if (leftSign == rightSign)
        {
            WideArithmetic.AddEqualMagnitudes(left, right, result);
            resultSign = leftSign;
            return;
        }

        int comparison = WideArithmetic.CompareMagnitudeEqualLength(left, right);
        if (comparison == 0)
        {
            result.Clear();
            resultSign = 0;
            return;
        }
        if (comparison > 0)
        {
            WideArithmetic.SubtractEqualMagnitudes(left, right, result);
            resultSign = leftSign;
        }
        else
        {
            WideArithmetic.SubtractEqualMagnitudes(right, left, result);
            resultSign = rightSign;
        }
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

    private static bool IsZero(ReadOnlySpan<ulong> value) =>
        WideArithmetic.GetActiveMagnitudeLength(value) == 0;
}
