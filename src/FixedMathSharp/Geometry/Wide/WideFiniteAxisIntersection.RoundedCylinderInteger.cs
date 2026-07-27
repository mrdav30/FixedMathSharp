//=======================================================================
// WideFiniteAxisIntersection.RoundedCylinderInteger.cs
//=======================================================================
// MIT License, Copyright (c) 2024–present David Oravsky (mrdav30)
// See LICENSE file in the project root for full license information.
//=======================================================================

using System;

namespace FixedMathSharp.Geometry;

/// <content>
/// Signed wide-integer arithmetic helpers (add, subtract, multiply, shift, compare)
/// used to evaluate rounded-cylinder intersection polynomial coefficients.
/// </content>
internal static partial class WideFiniteAxisIntersection
{
    private static void ImportRoundedCylinderCoefficient(
        Signed576 value,
        Span<ulong> coefficients,
        Span<sbyte> signs,
        int polynomialIndex,
        int coefficientIndex)
    {
        Span<ulong> destination = GetRoundedCylinderCoefficient(
            coefficients,
            polynomialIndex,
            coefficientIndex);
        WideArithmetic.GetMagnitude(value, destination[..9]);
        signs[GetRoundedCylinderCoefficientIndex(polynomialIndex, coefficientIndex)] = (sbyte)value.Sign;
    }

    private static Span<ulong> GetRoundedCylinderCoefficient(
        Span<ulong> coefficients,
        int polynomialIndex,
        int coefficientIndex)
    {
        int index = GetRoundedCylinderCoefficientIndex(polynomialIndex, coefficientIndex);
        return coefficients.Slice(
            index * RoundedCylinderWideLimbCount,
            RoundedCylinderWideLimbCount);
    }

    private static int GetRoundedCylinderCoefficientIndex(int polynomialIndex, int coefficientIndex) =>
        polynomialIndex * RoundedCylinderCoefficientCount + coefficientIndex;

    private static void AddRoundedCylinderSigned(
        ReadOnlySpan<ulong> left,
        sbyte leftSign,
        ReadOnlySpan<ulong> right,
        sbyte rightSign,
        Span<ulong> destination,
        out sbyte destinationSign)
    {
        if (leftSign == 0)
        {
            CopyRoundedCylinderWide(right, destination);
            destinationSign = rightSign;
            return;
        }
        if (rightSign == 0)
        {
            CopyRoundedCylinderWide(left, destination);
            destinationSign = leftSign;
            return;
        }
        if (leftSign != rightSign)
        {
            SubtractRoundedCylinderSigned(
                left,
                leftSign,
                right,
                (sbyte)-rightSign,
                destination,
                out destinationSign);
            return;
        }

        ulong carry = 0UL;
        for (int index = 0; index < destination.Length; index++)
        {
            ulong sum = unchecked(left[index] + right[index]);
            ulong result = unchecked(sum + carry);
            carry = sum < left[index] || result < sum ? 1UL : 0UL;
            destination[index] = result;
        }
        destinationSign = leftSign;
    }

    private static void SubtractRoundedCylinderSigned(
        ReadOnlySpan<ulong> left,
        sbyte leftSign,
        ReadOnlySpan<ulong> right,
        sbyte rightSign,
        Span<ulong> destination,
        out sbyte destinationSign)
    {
        if (leftSign == 0 || rightSign == 0)
        {
            bool useRight = leftSign == 0;
            CopyRoundedCylinderWide(useRight ? right : left, destination);
            destinationSign = useRight ? (sbyte)-rightSign : leftSign;
            return;
        }
        if (leftSign != rightSign)
        {
            AddRoundedCylinderSigned(
                left,
                leftSign,
                right,
                (sbyte)-rightSign,
                destination,
                out destinationSign);
            return;
        }

        int comparison = CompareRoundedCylinderWide(left, right);
        if (comparison == 0)
        {
            destination.Clear();
            destinationSign = 0;
            return;
        }

        ReadOnlySpan<ulong> larger = comparison > 0 ? left : right;
        ReadOnlySpan<ulong> smaller = comparison > 0 ? right : left;
        ulong borrow = 0UL;
        for (int index = 0; index < destination.Length; index++)
        {
            ulong difference = unchecked(larger[index] - smaller[index]);
            ulong result = unchecked(difference - borrow);
            borrow = larger[index] < smaller[index]
                || (borrow != 0UL && difference == 0UL)
                ? 1UL
                : 0UL;
            destination[index] = result;
        }
        destinationSign = comparison > 0 ? leftSign : (sbyte)-leftSign;
    }

    private static void MultiplyRoundedCylinderWide(
        ReadOnlySpan<ulong> left,
        ReadOnlySpan<ulong> right,
        Span<ulong> destination)
    {
        destination.Clear();
        int leftLength = GetRoundedCylinderWideLength(left);
        int rightLength = GetRoundedCylinderWideLength(right);
        for (int leftIndex = 0; leftIndex < leftLength; leftIndex++)
        {
            for (int rightIndex = 0; rightIndex < rightLength; rightIndex++)
            {
                Fixed64.Multiply64To128(
                    left[leftIndex],
                    right[rightIndex],
                    out ulong high,
                    out ulong low);
                AddRoundedCylinderWord(destination, leftIndex + rightIndex, low);
                AddRoundedCylinderWord(destination, leftIndex + rightIndex + 1, high);
            }
        }
    }

    private static void MultiplyRoundedCylinderWideByWord(
        ReadOnlySpan<ulong> value,
        ulong multiplier,
        Span<ulong> destination)
    {
        destination.Clear();
        if (multiplier == 0UL)
            return;
        int length = GetRoundedCylinderWideLength(value);
        for (int index = 0; index < length; index++)
        {
            Fixed64.Multiply64To128(value[index], multiplier, out ulong high, out ulong low);
            AddRoundedCylinderWord(destination, index, low);
            AddRoundedCylinderWord(destination, index + 1, high);
        }
    }

    private static void AddRoundedCylinderWord(Span<ulong> value, int index, ulong word)
    {
        while (word != 0UL)
        {
            ulong sum = unchecked(value[index] + word);
            word = sum < value[index] ? 1UL : 0UL;
            value[index] = sum;
            index++;
        }
    }

    private static sbyte MultiplyRoundedCylinderSigns(sbyte left, sbyte right) =>
        left == 0 || right == 0 ? (sbyte)0 : (sbyte)(left * right);

    private static int CompareRoundedCylinderWide(
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

    private static int GetRoundedCylinderWideLength(ReadOnlySpan<ulong> value)
    {
        int length = value.Length;
        while (length > 0 && value[length - 1] == 0UL)
            length--;
        return length;
    }

    private static void CopyRoundedCylinderWide(
        ReadOnlySpan<ulong> source,
        Span<ulong> destination) =>
        source.CopyTo(destination);

    private static int CountRoundedCylinderTrailingZeroes(ReadOnlySpan<ulong> value)
    {
        int wordIndex = 0;
        while (value[wordIndex] == 0UL)
            wordIndex++;
        ulong word = value[wordIndex];
        int bits = 0;
        while ((word & 1UL) == 0UL)
        {
            word >>= 1;
            bits++;
        }
        return wordIndex * 64 + bits;
    }

    private static void ShiftRoundedCylinderWideRight(Span<ulong> value, int bits)
    {
        int wordShift = bits >> 6;
        int bitShift = bits & 63;
        for (int destinationIndex = 0; destinationIndex < value.Length; destinationIndex++)
        {
            int sourceIndex = destinationIndex + wordShift;
            ulong lower = sourceIndex < value.Length ? value[sourceIndex] : 0UL;
            ulong upper = sourceIndex + 1 < value.Length ? value[sourceIndex + 1] : 0UL;
            uint nonzero = ((uint)bitShift | (0U - (uint)bitShift)) >> 31;
            ulong upperMask = 0UL - nonzero;
            value[destinationIndex] = (lower >> bitShift)
                | ((upper << ((64 - bitShift) & 63)) & upperMask);
        }
    }
}
