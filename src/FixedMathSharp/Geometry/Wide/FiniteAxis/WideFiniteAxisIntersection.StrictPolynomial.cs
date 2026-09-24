//=======================================================================
// MIT License, Copyright (c) 2024-present David Oravsky (mrdav30)
// See LICENSE file in the project root for full license information.
//=======================================================================
using System;

namespace FixedMathSharp.Geometry;

/// <content>
/// Bounded exact polynomial signs for centered finite-shape overlap. No roots
/// are materialized or rounded: even-multiplicity tangencies do not penetrate.
/// </content>
internal static partial class WideFiniteAxisIntersection
{
    private const int StrictPolynomialInputWords = 16;
    private const int StrictPolynomialWorkWords = 128;

    /// <summary>
    /// Tests negativity somewhere on [0,1]. The five low-to-high coefficients
    /// each have sixteen magnitude words and a separate sign. Inputs are
    /// bounded by 1024 bits; the degree is at most four.
    /// </summary>
    internal static bool HasNegativeFiniteAxisPolynomial(
        ReadOnlySpan<ulong> input, ReadOnlySpan<sbyte> inputSigns)
    {
        // A B-bit quartic's factorized Sturm terms use at most 6B+32 bits.
        // The exceptional cubic/linear pseudo-remainder uses at most 7B+32;
        // lower-degree Euclidean chains have no larger transient. B<=1024
        // therefore fits 8192 bits, including endpoint sums and gcd tests.
        Span<ulong> coefficients = stackalloc ulong[25 * StrictPolynomialWorkWords];
        Span<sbyte> signs = stackalloc sbyte[25];
        Span<int> degrees = stackalloc int[5];
        coefficients.Clear();
        signs.Clear();
        degrees.Clear();
        for (int i = 0; i < 5; i++)
        {
            input.Slice(i * StrictPolynomialInputWords, StrictPolynomialInputWords)
                .CopyTo(GetRoundedCylinderCoefficient(coefficients, 0, i));
            signs[i] = inputSigns[i];
        }
        int degree = TrimRoundedCylinderPolynomial(signs, 0, 4);
        if (degree < 0)
            return false;
        degrees[0] = degree;
        if (GetStrictPolynomialEndpointSign(coefficients, signs, 0, degree, atOne: false) < 0
            || GetStrictPolynomialEndpointSign(coefficients, signs, 0, degree, atOne: true) < 0)
            return true;
        if (degree == 0)
            return false;

        int count = BuildFiniteAxisPolynomialSturmSequence(coefficients, signs, degrees);
        int roots = GetStrictPolynomialVariations(coefficients, signs, degrees, count, false)
            - GetStrictPolynomialVariations(coefficients, signs, degrees, count, true);
        if (roots == 0)
            return false;

        int gcdIndex = count - 1;
        int gcdDegree = degrees[gcdIndex];
        if (gcdDegree == 0)
            return true;
        if (gcdDegree == 1)
        {
            // The sole repeated root has multiplicity two. Remove it from
            // the distinct count only if it lies strictly inside (0,1).
            int start = signs[GetRoundedCylinderCoefficientIndex(gcdIndex, 0)];
            int end = GetStrictPolynomialValueAtOne(coefficients, signs, gcdIndex, 1, 0);
            int repeatedInside = start != 0 && end != 0 && start != end ? 1 : 0;
            return roots > repeatedInside;
        }
        if (gcdDegree == 2)
        {
            // A cubic with a quadratic gcd is a cube: an interior root would
            // already give a negative endpoint sign. Only quartics reach here.
            // Quartic: either two double roots (never a crossing), or one
            // triple and one simple root (both are crossings).
            Span<ulong> first = stackalloc ulong[StrictPolynomialWorkWords];
            Span<ulong> second = stackalloc ulong[StrictPolynomialWorkWords];
            Span<ulong> scaled = stackalloc ulong[StrictPolynomialWorkWords];
            Span<ulong> result = stackalloc ulong[StrictPolynomialWorkWords];
            ReadOnlySpan<ulong> b = GetRoundedCylinderCoefficient(coefficients, gcdIndex, 1);
            MultiplyRoundedCylinderWide(b, b, first);
            MultiplyRoundedCylinderWide(
                GetRoundedCylinderCoefficient(coefficients, gcdIndex, 2),
                GetRoundedCylinderCoefficient(coefficients, gcdIndex, 0), second);
            MultiplyRoundedCylinderWideByWord(second, 4, scaled);
            SubtractRoundedCylinderSigned(first,
                signs[GetRoundedCylinderCoefficientIndex(gcdIndex, 1)] == 0 ? (sbyte)0 : (sbyte)1,
                scaled, (sbyte)(signs[GetRoundedCylinderCoefficientIndex(gcdIndex, 2)]
                    * signs[GetRoundedCylinderCoefficientIndex(gcdIndex, 0)]), result, out sbyte sign);
            return sign == 0;
        }
        return false; // A quartic with a cubic gcd is a fourth power.
    }

    /// <summary>
    /// Builds an exact degree-at-most-four Sturm chain in five polynomial
    /// slots of five coefficients each. Word width is selected by the owning
    /// bounded geometry caller. Slot zero and degrees[0] contain the input;
    /// all remaining coefficient/sign slots must initially be zero.
    /// </summary>
    internal static int BuildFiniteAxisPolynomialSturmSequence(
        Span<ulong> coefficients, Span<sbyte> signs, Span<int> degrees)
    {
        int degree = degrees[0];
        if (degree <= 0)
            return degree + 1;
        if (signs[degree] < 0)
        {
            for (int i = 0; i <= degree; i++)
                signs[i] = (sbyte)-signs[i];
        }
        for (int i = 1; i <= degree; i++)
        {
            MultiplyRoundedCylinderWideByWord(GetRoundedCylinderCoefficient(coefficients, 0, i),
                (ulong)i, GetRoundedCylinderCoefficient(coefficients, 1, i - 1));
            signs[GetRoundedCylinderCoefficientIndex(1, i - 1)] = signs[i];
        }
        degrees[1] = degree - 1;
        if (degree == 4)
            return BuildRoundedCylinderQuarticSturmSequence(coefficients, signs, degrees);
        int count = 2;
        while (degrees[count - 1] > 0)
        {
            int remainder = BuildRoundedCylinderNegativeRemainder(coefficients, signs,
                count - 2, degrees[count - 2], count - 1, degrees[count - 1], count);
            if (remainder < 0)
                break;
            degrees[count++] = remainder;
        }
        return count;
    }

    private static int GetStrictPolynomialVariations(Span<ulong> coefficients,
        Span<sbyte> signs, Span<int> degrees, int count, bool atOne)
    {
        int previous = 0;
        int variations = 0;
        for (int i = 0; i < count; i++)
        {
            int sign = GetStrictPolynomialEndpointSign(coefficients, signs, i, degrees[i], atOne);
            if (previous != 0 && previous != sign)
                variations++;
            previous = sign;
        }
        return variations;
    }

    // These are inward one-sided signs, so endpoint roots are excluded from
    // the Sturm count and a negative interval touching an endpoint is kept.
    private static int GetStrictPolynomialEndpointSign(Span<ulong> coefficients,
        Span<sbyte> signs, int polynomial, int degree, bool atOne)
    {
        // Every sequence member has a nonzero leading coefficient. If all
        // lower derivatives vanish, the degree-th derivative has that sign.
        for (int order = 0; order < degree; order++)
        {
            int sign = atOne
                ? GetStrictPolynomialValueAtOne(coefficients, signs, polynomial, degree, order)
                : signs[GetRoundedCylinderCoefficientIndex(polynomial, order)];
            if (sign != 0)
                return atOne && (order & 1) != 0 ? -sign : sign;
        }
        int leading = signs[GetRoundedCylinderCoefficientIndex(polynomial, degree)];
        return atOne && (degree & 1) != 0 ? -leading : leading;
    }

    private static int GetStrictPolynomialValueAtOne(Span<ulong> coefficients,
        Span<sbyte> signs, int polynomial, int degree, int derivativeOrder)
    {
        int words = coefficients.Length / 25;
        Span<ulong> result = stackalloc ulong[words];
        Span<ulong> term = stackalloc ulong[words];
        Span<ulong> sum = stackalloc ulong[words];
        result.Clear();
        sbyte resultSign = 0;
        for (int i = derivativeOrder; i <= degree; i++)
        {
            ulong binomial = 1;
            for (int j = 1; j <= derivativeOrder; j++)
                binomial = binomial * (ulong)(i - j + 1) / (ulong)j;
            MultiplyRoundedCylinderWideByWord(GetRoundedCylinderCoefficient(coefficients, polynomial, i),
                binomial, term);
            AddRoundedCylinderSigned(result, resultSign, term,
                signs[GetRoundedCylinderCoefficientIndex(polynomial, i)], sum, out resultSign);
            sum.CopyTo(result);
        }
        return resultSign;
    }
}
