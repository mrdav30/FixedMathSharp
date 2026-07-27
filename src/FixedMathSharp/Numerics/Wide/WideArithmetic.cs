//=======================================================================
// WideArithmetic.cs
//=======================================================================
// MIT License, Copyright (c) 2024–present David Oravsky (mrdav30)
// See LICENSE file in the project root for full license information.
//=======================================================================

using System;
using System.Runtime.CompilerServices;

namespace FixedMathSharp;

/// <summary>
/// Owns fixed-width limb arithmetic used by exact deterministic geometry.
/// </summary>
internal static partial class WideArithmetic
{
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    internal static int CompareUnsigned(
        ulong leftHigh,
        ulong leftMiddle,
        ulong leftLow,
        ulong rightHigh,
        ulong rightMiddle,
        ulong rightLow)
    {
        if (leftHigh != rightHigh)
            return leftHigh < rightHigh ? -1 : 1;
        if (leftMiddle != rightMiddle)
            return leftMiddle < rightMiddle ? -1 : 1;
        if (leftLow != rightLow)
            return leftLow < rightLow ? -1 : 1;
        return 0;
    }

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    internal static int CompareUnsigned(
        ulong leftWord4,
        ulong leftWord3,
        ulong leftWord2,
        ulong leftWord1,
        ulong leftWord0,
        ulong rightWord4,
        ulong rightWord3,
        ulong rightWord2,
        ulong rightWord1,
        ulong rightWord0)
    {
        if (leftWord4 != rightWord4)
            return leftWord4 < rightWord4 ? -1 : 1;
        if (leftWord3 != rightWord3)
            return leftWord3 < rightWord3 ? -1 : 1;
        if (leftWord2 != rightWord2)
            return leftWord2 < rightWord2 ? -1 : 1;
        if (leftWord1 != rightWord1)
            return leftWord1 < rightWord1 ? -1 : 1;
        if (leftWord0 != rightWord0)
            return leftWord0 < rightWord0 ? -1 : 1;
        return 0;
    }

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    private static bool TryGetSigned95Magnitude(
        Signed192 value,
        out ulong middle,
        out ulong low)
    {
        GetMagnitude(value, out ulong high, out middle, out low);
        return high == 0UL && middle <= 0x7FFF_FFFFUL; // 2,147,483,647 (31 significant high bits).
    }

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    private static Signed192 MultiplySigned95(
        ulong leftMiddle,
        ulong leftLow,
        ulong rightMiddle,
        ulong rightLow,
        bool negative)
    {
        MultiplyUnsigned96(
            (uint)leftMiddle,
            leftLow,
            (uint)rightMiddle,
            rightLow,
            out _,
            out ulong word2,
            out ulong word1,
            out ulong word0);

        if (negative)
        {
            word0 = unchecked(~word0 + 1UL);
            word1 = unchecked(~word1 + (word0 == 0UL ? 1UL : 0UL));
            word2 = unchecked(~word2 + (word1 == 0UL && word0 == 0UL ? 1UL : 0UL));
        }

        return new Signed192(word2, word1, word0);
    }

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    private static void MultiplyUnsigned128(
        ulong leftHigh,
        ulong leftLow,
        ulong rightHigh,
        ulong rightLow,
        out ulong word3,
        out ulong word2,
        out ulong word1,
        out ulong word0)
    {
        if ((leftHigh | rightHigh) <= uint.MaxValue)
        {
            MultiplyUnsigned96(
                (uint)leftHigh,
                leftLow,
                (uint)rightHigh,
                rightLow,
                out word3,
                out word2,
                out word1,
                out word0);
            return;
        }

        Fixed64.Multiply64To128(leftLow, rightLow, out ulong lowHigh, out word0);
        Fixed64.Multiply64To128(leftLow, rightHigh, out ulong leftCrossHigh, out ulong leftCrossLow);
        Fixed64.Multiply64To128(leftHigh, rightLow, out ulong rightCrossHigh, out ulong rightCrossLow);
        Fixed64.Multiply64To128(leftHigh, rightHigh, out word3, out ulong highLow);

        word1 = lowHigh;
        ulong carry = 0UL;
        AccumulateWord(ref word1, leftCrossLow, ref carry);
        AccumulateWord(ref word1, rightCrossLow, ref carry);

        word2 = leftCrossHigh;
        ulong highCarry = 0UL;
        AccumulateWord(ref word2, rightCrossHigh, ref highCarry);
        AccumulateWord(ref word2, highLow, ref highCarry);
        AccumulateWord(ref word2, carry, ref highCarry);
        word3 = unchecked(word3 + highCarry);
    }

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    private static void MultiplyUnsigned96(
        uint leftHigh,
        ulong leftLow,
        uint rightHigh,
        ulong rightLow,
        out ulong word3,
        out ulong word2,
        out ulong word1,
        out ulong word0)
    {
        Fixed64.Multiply64To128(leftLow, rightLow, out ulong lowHigh, out word0);
        Multiply64By32(leftLow, rightHigh, out ulong leftCrossHigh, out ulong leftCrossLow);
        Multiply64By32(rightLow, leftHigh, out ulong rightCrossHigh, out ulong rightCrossLow);

        word1 = lowHigh;
        ulong carry = 0UL;
        AccumulateWord(ref word1, leftCrossLow, ref carry);
        AccumulateWord(ref word1, rightCrossLow, ref carry);

        word2 = leftCrossHigh;
        ulong highCarry = 0UL;
        AccumulateWord(ref word2, rightCrossHigh, ref highCarry);
        AccumulateWord(ref word2, (ulong)leftHigh * rightHigh, ref highCarry);
        AccumulateWord(ref word2, carry, ref highCarry);
        word3 = highCarry;
    }

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    private static void Multiply64By32(
        ulong left,
        uint right,
        out ulong high,
        out ulong low)
    {
        ulong lowProduct = (uint)left * (ulong)right;
        ulong highProduct = (left >> 32) * right;
        ulong middle = (lowProduct >> 32) + (uint)highProduct;
        low = (lowProduct & uint.MaxValue) | (middle << 32);
        high = (highProduct >> 32) + (middle >> 32);
    }

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    private static void AddProductAt2(
        ref ulong word4,
        ref ulong word3,
        ref ulong word2,
        ulong left,
        ulong right)
    {
        Fixed64.Multiply64To128(left, right, out ulong high, out ulong low);
        ulong previous = word2;
        word2 = unchecked(word2 + low);
        ulong carry = word2 < previous ? 1UL : 0UL;

        // A 64-by-64 product's high word is at most UInt64.MaxValue - 1,
        // so adding the one-bit low-word carry cannot overflow here.
        ulong highWithCarry = high + carry;
        previous = word3;
        word3 = unchecked(word3 + highWithCarry);
        if (word3 < previous)
            word4++;
    }

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    private static void AddProductAt3(
        ref ulong word4,
        ref ulong word3,
        ulong left,
        ulong right)
    {
        Fixed64.Multiply64To128(left, right, out ulong high, out ulong low);
        ulong previous = word3;
        word3 = unchecked(word3 + low);
        word4 = unchecked(word4 + high + (word3 < previous ? 1UL : 0UL));
    }

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    internal static void AccumulateWord(ref ulong word, ulong add, ref ulong carry)
    {
        ulong previous = word;
        word = unchecked(word + add);
        if (word < previous)
            carry++;
    }

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    internal static ulong SubtractWord(ulong value, ulong subtract, ref ulong borrow)
    {
        ulong subtrahend = unchecked(subtract + borrow);
        ulong overflow = subtrahend < subtract ? 1UL : 0UL;
        ulong result = unchecked(value - subtrahend);
        borrow = overflow | (value < subtrahend ? 1UL : 0UL);
        return result;
    }

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    internal static void AddWord(Span<ulong> words, int index, ulong value)
    {
        while (value != 0UL && index < words.Length)
        {
            ulong previous = words[index];
            words[index] = unchecked(previous + value);
            value = words[index] < previous ? 1UL : 0UL;
            index++;
        }
    }

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    internal static ulong AddSignedWord(ulong left, ulong right, ref ulong carry)
    {
        ulong sum = unchecked(left + right);
        ulong result = unchecked(sum + carry);
        carry = sum < left || result < sum ? 1UL : 0UL;
        return result;
    }

    internal static bool IsZeroMagnitude(ReadOnlySpan<ulong> value)
    {
        for (int index = 0; index < value.Length; index++)
        {
            if (value[index] != 0UL)
                return false;
        }

        return true;
    }

    #region Multiquadratic Comparison

    private const int MaximumLinearRadicalCount = 6;

    /// <summary>
    /// Gets the exact sign of a signed sum of nonnegative square roots.
    /// </summary>
    /// <remarks>
    /// Each radicand occupies <paramref name="radicandWordCount"/> words in
    /// little-endian order. Equal radicands are merged before the
    /// multiquadratic basis is built.
    /// </remarks>
    internal static int GetLinearRadicalSumSign(
        ReadOnlySpan<ulong> radicands,
        int radicandWordCount,
        ReadOnlySpan<int> signs)
    {
        int termCount = signs.Length;
        Span<ulong> uniqueRadicands = stackalloc ulong[
            MaximumLinearRadicalCount * radicandWordCount];
        Span<int> coefficients =
            stackalloc int[MaximumLinearRadicalCount];
        uniqueRadicands.Clear();
        coefficients.Clear();
        int uniqueCount = 0;
        for (int termIndex = 0; termIndex < termCount; termIndex++)
        {
            int sign = signs[termIndex];
            ReadOnlySpan<ulong> radicand = radicands.Slice(
                termIndex * radicandWordCount,
                radicandWordCount);
            if (sign == 0 || IsZeroMagnitude(radicand))
                continue;

            int existingIndex = -1;
            for (int candidateIndex = 0;
                 candidateIndex < uniqueCount;
                 candidateIndex++)
            {
                if (CompareMagnitudeEqualLength(
                        radicand,
                        uniqueRadicands.Slice(
                            candidateIndex * radicandWordCount,
                            radicandWordCount)) == 0)
                {
                    existingIndex = candidateIndex;
                    break;
                }
            }

            if (existingIndex >= 0)
            {
                coefficients[existingIndex] += sign;
                continue;
            }

            radicand.CopyTo(uniqueRadicands.Slice(
                uniqueCount * radicandWordCount,
                radicandWordCount));
            coefficients[uniqueCount] = sign;
            uniqueCount++;
        }

        int compactedCount = 0;
        for (int index = 0; index < uniqueCount; index++)
        {
            int coefficient = coefficients[index];
            if (coefficient == 0)
                continue;
            if (compactedCount != index)
            {
                uniqueRadicands.Slice(
                        index * radicandWordCount,
                        radicandWordCount)
                    .CopyTo(uniqueRadicands.Slice(
                        compactedCount * radicandWordCount,
                        radicandWordCount));
            }
            coefficients[compactedCount] = coefficient;
            compactedCount++;
        }
        uniqueCount = compactedCount;
        if (uniqueCount == 0)
            return 0;

        int basisCount = 1 << uniqueCount;
        Span<ulong> basisMagnitudes =
            stackalloc ulong[basisCount];
        Span<int> basisSigns =
            stackalloc int[basisCount];
        basisMagnitudes.Clear();
        basisSigns.Clear();
        for (int index = 0; index < uniqueCount; index++)
        {
            int coefficient = coefficients[index];
            int basisIndex = 1 << index;
            basisMagnitudes[basisIndex] =
                unchecked((ulong)Math.Abs(coefficient));
            basisSigns[basisIndex] = Math.Sign(coefficient);
        }

        return GetMultiquadraticSign(
            basisMagnitudes,
            basisSigns,
            coefficientWordCount: 1,
            uniqueRadicands,
            radicandWordCount,
            uniqueCount,
            uniqueCount);
    }

    private static int GetMultiquadraticSign(
        ReadOnlySpan<ulong> coefficientMagnitudes,
        ReadOnlySpan<int> coefficientSigns,
        int coefficientWordCount,
        ReadOnlySpan<ulong> radicands,
        int radicandWordCount,
        int radicalCount,
        int totalRadicalCount)
    {
        if (radicalCount == 0)
            return coefficientSigns[0];

        int halfBasisCount = 1 << (radicalCount - 1);
        int halfWordCount =
            halfBasisCount * coefficientWordCount;
        int firstSign = GetMultiquadraticSign(
            coefficientMagnitudes.Slice(0, halfWordCount),
            coefficientSigns.Slice(0, halfBasisCount),
            coefficientWordCount,
            radicands,
            radicandWordCount,
            radicalCount - 1,
            totalRadicalCount);
        int secondSign = GetMultiquadraticSign(
            coefficientMagnitudes.Slice(
                halfWordCount,
                halfWordCount),
            coefficientSigns.Slice(
                halfBasisCount,
                halfBasisCount),
            coefficientWordCount,
            radicands,
            radicandWordCount,
            radicalCount - 1,
            totalRadicalCount);
        if (firstSign == 0)
            return secondSign;
        if (secondSign == 0 || firstSign == secondSign)
            return firstSign;

        int eliminatedCount =
            totalRadicalCount - radicalCount + 1;
        int maximumRadicandDegree =
            1 << (eliminatedCount - 1);
        int nextCoefficientWordCount =
            maximumRadicandDegree * radicandWordCount
            + (maximumRadicandDegree << 2)
            + 1;
        Span<ulong> differenceMagnitudes = stackalloc ulong[
            halfBasisCount * nextCoefficientWordCount];
        Span<int> differenceSigns =
            stackalloc int[halfBasisCount];
        differenceMagnitudes.Clear();
        differenceSigns.Clear();
        BuildMultiquadraticSquaredDifference(
            coefficientMagnitudes.Slice(0, halfWordCount),
            coefficientSigns.Slice(0, halfBasisCount),
            coefficientMagnitudes.Slice(
                halfWordCount,
                halfWordCount),
            coefficientSigns.Slice(
                halfBasisCount,
                halfBasisCount),
            coefficientWordCount,
            radicands,
            radicandWordCount,
            radicalCount,
            differenceMagnitudes,
            differenceSigns,
            nextCoefficientWordCount);
        int squaredDifferenceSign = GetMultiquadraticSign(
            differenceMagnitudes,
            differenceSigns,
            nextCoefficientWordCount,
            radicands,
            radicandWordCount,
            radicalCount - 1,
            totalRadicalCount);
        if (squaredDifferenceSign == 0)
            return 0;
        return squaredDifferenceSign > 0
            ? firstSign
            : secondSign;
    }

    private static void BuildMultiquadraticSquaredDifference(
        ReadOnlySpan<ulong> firstMagnitudes,
        ReadOnlySpan<int> firstSigns,
        ReadOnlySpan<ulong> secondMagnitudes,
        ReadOnlySpan<int> secondSigns,
        int coefficientWordCount,
        ReadOnlySpan<ulong> radicands,
        int radicandWordCount,
        int radicalCount,
        Span<ulong> resultMagnitudes,
        Span<int> resultSigns,
        int resultWordCount)
    {
        AddMultiquadraticSquare(
            firstMagnitudes,
            firstSigns,
            coefficientWordCount,
            radicands,
            radicandWordCount,
            radicalCount - 1,
            extraRadicandIndex: -1,
            resultMagnitudes,
            resultSigns,
            resultWordCount,
            resultSign: 1);
        AddMultiquadraticSquare(
            secondMagnitudes,
            secondSigns,
            coefficientWordCount,
            radicands,
            radicandWordCount,
            radicalCount - 1,
            extraRadicandIndex: radicalCount - 1,
            resultMagnitudes,
            resultSigns,
            resultWordCount,
            resultSign: -1);
    }

    private static void AddMultiquadraticSquare(
        ReadOnlySpan<ulong> coefficientMagnitudes,
        ReadOnlySpan<int> coefficientSigns,
        int coefficientWordCount,
        ReadOnlySpan<ulong> radicands,
        int radicandWordCount,
        int remainingRadicalCount,
        int extraRadicandIndex,
        Span<ulong> resultMagnitudes,
        Span<int> resultSigns,
        int resultWordCount,
        int resultSign)
    {
        int basisCount = 1 << remainingRadicalCount;
        Span<ulong> firstProduct =
            stackalloc ulong[resultWordCount];
        Span<ulong> secondProduct =
            stackalloc ulong[resultWordCount];
        for (int firstIndex = 0;
             firstIndex < basisCount;
             firstIndex++)
        {
            int firstCoefficientSign =
                coefficientSigns[firstIndex];
            if (firstCoefficientSign == 0)
                continue;
            ReadOnlySpan<ulong> firstCoefficient =
                coefficientMagnitudes.Slice(
                    firstIndex * coefficientWordCount,
                    coefficientWordCount);
            for (int secondIndex = 0;
                 secondIndex < basisCount;
                 secondIndex++)
            {
                int secondCoefficientSign =
                    coefficientSigns[secondIndex];
                if (secondCoefficientSign == 0)
                    continue;
                ReadOnlySpan<ulong> secondCoefficient =
                    coefficientMagnitudes.Slice(
                        secondIndex * coefficientWordCount,
                        coefficientWordCount);
                MultiplyMagnitudes(
                    firstCoefficient,
                    secondCoefficient,
                    firstProduct);
                Span<ulong> product = firstProduct;
                Span<ulong> scratch = secondProduct;
                int commonRadicals =
                    firstIndex & secondIndex;
                for (int radicalIndex = 0;
                     radicalIndex < remainingRadicalCount;
                     radicalIndex++)
                {
                    if ((commonRadicals
                            & (1 << radicalIndex)) == 0)
                    {
                        continue;
                    }
                    MultiplyMagnitudes(
                        product,
                        radicands.Slice(
                            radicalIndex * radicandWordCount,
                            radicandWordCount),
                        scratch);
                    Span<ulong> temporary = product;
                    product = scratch;
                    scratch = temporary;
                }
                if (extraRadicandIndex >= 0)
                {
                    MultiplyMagnitudes(
                        product,
                        radicands.Slice(
                            extraRadicandIndex
                            * radicandWordCount,
                            radicandWordCount),
                        scratch);
                    Span<ulong> temporary = product;
                    product = scratch;
                    scratch = temporary;
                }

                int targetIndex =
                    firstIndex ^ secondIndex;
                AddSignedMagnitude(
                    product,
                    firstCoefficientSign
                    * secondCoefficientSign
                    * resultSign,
                    resultMagnitudes.Slice(
                        targetIndex * resultWordCount,
                        resultWordCount),
                    ref resultSigns[targetIndex]);
            }
        }
    }

    #endregion

    private static void AddSignedMagnitude(
        ReadOnlySpan<ulong> addend,
        int addendSign,
        Span<ulong> result,
        ref int resultSign)
    {
        // Compaction admits only nonzero coefficients and radicands, so their
        // exact fixed-width product is nonzero.
        if (resultSign == 0)
        {
            addend.CopyTo(result);
            resultSign = addendSign;
            return;
        }
        if (resultSign == addendSign)
        {
            AddMagnitudeInto(addend, result);
            return;
        }

        int comparison =
            CompareMagnitudeEqualLength(result, addend);
        if (comparison == 0)
        {
            result.Clear();
            resultSign = 0;
            return;
        }
        if (comparison > 0)
        {
            SubtractEqualMagnitudes(
                result,
                addend,
                result);
            return;
        }

        Span<ulong> difference =
            stackalloc ulong[result.Length];
        SubtractEqualMagnitudes(
            addend,
            result,
            difference);
        difference.CopyTo(result);
        resultSign = addendSign;
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

    private static void AddEqualMagnitudes(
        ReadOnlySpan<ulong> left,
        ReadOnlySpan<ulong> right,
        Span<ulong> result)
    {
        ulong carry = 0UL;
        for (int index = 0; index < result.Length; index++)
            result[index] =
                AddSignedWord(left[index], right[index], ref carry);
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
}
