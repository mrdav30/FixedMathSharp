//=======================================================================
// MIT License, Copyright (c) 2024-present David Oravsky (mrdav30)
// See LICENSE file in the project root for full license information.
//=======================================================================
using System;

namespace FixedMathSharp.Geometry;

/// <content>
/// Construction arithmetic for finite-axis signed magnitude polynomials.
/// Coefficients are in ascending degree order with caller-owned storage.
/// </content>
internal static partial class WideFiniteAxisIntersection
{
    /// <summary>
    /// Accumulates a polynomial multiplied by a signed integer and shifted by
    /// a nonnegative degree offset. Source and result have independent uniform
    /// strides, canonical signs, and disjoint storage. Every intermediate and
    /// nonzero source degree must fit the result. No scratch storage is needed.
    /// </summary>
    internal static void AddFiniteAxisPolynomial(
        ReadOnlySpan<ulong> source, ReadOnlySpan<sbyte> sourceSigns,
        Span<ulong> result, Span<sbyte> resultSigns,
        int multiplier, int degreeOffset = 0)
    {
        int sourceWords = source.Length / sourceSigns.Length;
        int resultWords = result.Length / resultSigns.Length;
        int factorSign = multiplier < 0 ? -1 : 1;
        uint magnitude = multiplier < 0 ? unchecked(0U - (uint)multiplier) : (uint)multiplier;
        for (int index = 0; index < sourceSigns.Length; index++)
        {
            if (sourceSigns[index] == 0)
                continue;
            int target = index + degreeOffset;
            int sign = resultSigns[target];
            uint factor = magnitude;
            int shift = 0;
            while (factor != 0)
            {
                if ((factor & 1) != 0)
                    WideArithmetic.AddShiftedSignedMagnitude(
                        source.Slice(index * sourceWords, sourceWords),
                        sourceSigns[index] * factorSign, shift,
                        result.Slice(target * resultWords, resultWords), ref sign);
                factor >>= 1;
                shift++;
            }
            resultSigns[target] = (sbyte)sign;
        }
    }

    /// <summary>
    /// Multiplies two polynomials, clearing the result unless accumulation is requested.
    /// Each magnitude span has its own uniform stride given by its sign count.
    /// </summary>
    /// <remarks>
    /// Sign spans are nonempty and magnitudes/signs are canonical. Result and
    /// product scratch must be disjoint from both inputs and each other. Scratch
    /// needs one result coefficient. Every intermediate magnitude and nonzero
    /// product degree must fit the result; padded zero input slots may exceed
    /// its degree capacity. No content normalization or rounding is performed.
    /// </remarks>
    internal static void MultiplyFiniteAxisPolynomials(
        ReadOnlySpan<ulong> left, ReadOnlySpan<sbyte> leftSigns,
        ReadOnlySpan<ulong> right, ReadOnlySpan<sbyte> rightSigns,
        Span<ulong> result, Span<sbyte> resultSigns, Span<ulong> product,
        bool accumulate = false)
    {
        int leftWords = left.Length / leftSigns.Length;
        int rightWords = right.Length / rightSigns.Length;
        int resultWords = result.Length / resultSigns.Length;
        product = product[..resultWords];
        if (!accumulate)
        {
            result.Clear();
            resultSigns.Clear();
        }
        for (int first = 0; first < leftSigns.Length; first++)
        {
            if (leftSigns[first] == 0)
                continue;
            for (int second = 0; second < rightSigns.Length; second++)
            {
                if (rightSigns[second] == 0)
                    continue;
                WideArithmetic.MultiplyMagnitudes(left.Slice(first * leftWords, leftWords),
                    right.Slice(second * rightWords, rightWords), product);
                int index = first + second;
                int sign = resultSigns[index];
                WideArithmetic.AddShiftedSignedMagnitude(product,
                    leftSigns[first] * rightSigns[second], 0,
                    result.Slice(index * resultWords, resultWords), ref sign);
                resultSigns[index] = (sbyte)sign;
            }
        }
    }

    /// <summary>
    /// Substitutes S=2^variableShift*t in place by shifting coefficient j by
    /// j*variableShift bits. The nonnegative shift must fit every coefficient;
    /// coefficientCount is positive and the separate signs remain unchanged.
    /// </summary>
    internal static void ScaleFiniteAxisPolynomialVariable(
        Span<ulong> coefficients, int coefficientCount, int variableShift)
    {
        System.Diagnostics.Debug.Assert(variableShift >= 0);
        int words = coefficients.Length / coefficientCount;
        for (int index = 1; index < coefficientCount; index++)
            ShiftFiniteRootLeft(coefficients.Slice(index * words, words), checked(index * variableShift));
    }

    /// <summary>
    /// Removes the common positive power of two from nonzero coefficients and
    /// returns its exponent. A zero polynomial returns zero. Signs are canonical,
    /// nonempty, and unchanged; a tuple may be normalized jointly to preserve ratios.
    /// </summary>
    internal static int NormalizeFiniteAxisPolynomialPowerOfTwo(
        Span<ulong> coefficients, ReadOnlySpan<sbyte> signs)
    {
        int words = coefficients.Length / signs.Length;
        int shift = int.MaxValue;
        for (int index = 0; index < signs.Length; index++)
        {
            if (signs[index] != 0)
                shift = Math.Min(shift, CountRoundedCylinderTrailingZeroes(coefficients.Slice(index * words, words)));
        }
        if (shift == int.MaxValue || shift == 0)
            return 0;
        for (int index = 0; index < signs.Length; index++)
        {
            if (signs[index] != 0)
                ShiftRoundedCylinderWideRight(coefficients.Slice(index * words, words), shift);
        }
        return shift;
    }

    /// <summary>
    /// Writes the ordinary derivative in the current variable, clearing padded
    /// output slots. Nonempty input/output sign spans determine their separate
    /// strides; storage is disjoint and every derivative coefficient must fit.
    /// </summary>
    internal static void DifferentiateFiniteAxisPolynomial(
        ReadOnlySpan<ulong> coefficients, ReadOnlySpan<sbyte> signs,
        Span<ulong> derivative, Span<sbyte> derivativeSigns)
    {
        int inputWords = coefficients.Length / signs.Length;
        int resultWords = derivative.Length / derivativeSigns.Length;
        derivative.Clear();
        derivativeSigns.Clear();
        for (int index = 1; index < signs.Length; index++)
        {
            if (signs[index] == 0)
                continue;
            MultiplyRoundedCylinderWideByWord(coefficients.Slice(index * inputWords, inputWords),
                (ulong)index, derivative.Slice((index - 1) * resultWords, resultWords));
            derivativeSigns[index - 1] = signs[index];
        }
    }
}
