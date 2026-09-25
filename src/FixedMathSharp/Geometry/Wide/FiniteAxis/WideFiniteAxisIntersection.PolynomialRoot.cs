//=======================================================================
// MIT License, Copyright (c) 2024-present David Oravsky (mrdav30)
// See LICENSE file in the project root for full license information.
//=======================================================================
using System;

namespace FixedMathSharp.Geometry;

/// <content>
/// Retained positive real roots used by finite-axis contact features. All
/// coefficients use caller-sized, low-to-high sign/magnitude slots. No root
/// is replaced by a rounded scalar, and exact zero is not a tolerance test.
/// </content>
internal static partial class WideFiniteAxisIntersection
{
    /// <summary>
    /// Returns storage for an isolating dyadic cell of a degree-at-most-four
    /// integer polynomial. The input and cell must outlive the retained root.
    /// </summary>
    // Isolation needs fewer than 9B+64 numerator bits. Later refinements stop
    // at shift 8B+128; root < 2^(B+1) then needs at most 9B+129 bits.
    // Reserve at least 9B+192 bits, including the upper endpoint increment.
    internal static int GetFiniteAxisRootCellWords(ReadOnlySpan<ulong> coefficients,
        ReadOnlySpan<sbyte> signs) =>
        (9 * GetFiniteRootCoefficientBits(coefficients, signs.Length) + 255) / 64;

    internal static bool TryGetLargestPositiveFiniteAxisRoot(ReadOnlySpan<ulong> coefficients,
        ReadOnlySpan<sbyte> signs, Span<ulong> cell, out FiniteAxisPolynomialRoot root)
    {
        root = default;
        int words = coefficients.Length / signs.Length;
        int degree = signs.Length - 1;
        while (degree >= 0 && signs[degree] == 0)
            degree--;
        if (degree <= 0)
            return false;

        // Zero is excluded. Removing its factors also makes zero a valid
        // nonroot lower bound for the open isolation interval.
        int first = 0;
        while (signs[first] == 0)
            first++;
        degree -= first;
        if (degree == 0)
            return false;
        root.Coefficients = coefficients.Slice(first * words, (degree + 1) * words);
        root.Signs = signs.Slice(first, degree + 1);
        root.CoefficientBits = GetFiniteRootCoefficientBits(root.Coefficients, degree + 1);
        root.LowerNumerator = cell;
        cell.Clear();

        int sturmWords = (7 * root.CoefficientBits + 95) / 64;
        Span<ulong> sturm = stackalloc ulong[25 * sturmWords];
        Span<sbyte> sturmSigns = stackalloc sbyte[25];
        Span<int> degrees = stackalloc int[5];
        int count = BuildFiniteRootSturm(root, sturm, sturmSigns, degrees);
        int infinity = GetRoundedCylinderInfinityVariations(sturmSigns, degrees, count, false);
        int remaining = GetFiniteRootVariations(sturm, sturmSigns, degrees, count,
            cell, 0) - infinity;
        if (remaining == 0)
        {
            root = default;
            return false;
        }

        // Cauchy's bound is |root| < 1 + H/|leading| <= 1 + 2^B.
        // Doubling therefore takes at most B+1 steps, but ordinary roots
        // do not pay for unused high coefficient bits.
        cell[0] = 1;
        int upperShift = 0;
        while (GetFiniteRootVariations(sturm, sturmSigns, degrees, count,
                   cell, upperShift) != infinity)
            upperShift--;
        root.DenominatorShift = upperShift;
        if (EvaluateFiniteRootPolynomial(root.Coefficients, root.Signs, cell, upperShift, 0) == 0)
        {
            root.IsRational = true;
            return true;
        }
        cell.Clear();

        // The square-free part has height <= 2^4 sqrt(5) H (Mahler's
        // factor bound). Its nonzero integer discriminant, together with
        // |ri-rj| <= 4Hsf, gives separation > 2^(-8B-58). Starting from
        // width <=2^(B+1), fewer than 9B+64 bisections isolate a root.
        // This proves both termination and the caller cell's bit capacity.
        // A positive lower endpoint also makes this a relative-scale cell.
        // Reciprocal Cauchy bounds give root >2^(-B-1), so finding it takes
        // at most B+2 additional bisections and stays within the same bound.
        while (remaining != 1 || GetRoundedCylinderWideLength(cell) == 0
               || EvaluateFiniteRootPolynomial(root.Coefficients,
                   root.Signs, cell, root.DenominatorShift, 0) == 0)
        {
            remaining = RefineFiniteRoot(ref root, sturm, sturmSigns, degrees, count, infinity, remaining);
            if (root.IsRational)
                break;
        }
        if (!root.IsRational)
        {
            Span<ulong> upper = stackalloc ulong[cell.Length];
            cell.CopyTo(upper);
            AddRoundedCylinderWord(upper, 0, 1);
            int lowerSign = EvaluateFiniteRootPolynomial(root.Coefficients, root.Signs,
                cell, root.DenominatorShift, 0);
            int upperSign = EvaluateFiniteRootPolynomial(root.Coefficients, root.Signs,
                upper, root.DenominatorShift, 0);
            root.CrossingSign = lowerSign != upperSign ? (sbyte)lowerSign : (sbyte)0;
        }
        return true;
    }

    /// <summary>
    /// Evaluates the exact sign of a degree-at-most-eight integer polynomial
    /// at the retained root, including equality and repeated roots.
    /// </summary>
    internal static int GetSignAtFiniteAxisRoot(ref FiniteAxisPolynomialRoot root,
        scoped ReadOnlySpan<ulong> coefficients, scoped ReadOnlySpan<sbyte> signs)
    {
        int degree = signs.Length - 1;
        while (degree > 0 && signs[degree] == 0)
            degree--;
        if (degree == 0)
            return signs[0];
        int words = coefficients.Length / signs.Length;
        coefficients = coefficients[..((degree + 1) * words)];
        signs = signs[..(degree + 1)];
        if (root.IsRational)
            return EvaluateFiniteRootPolynomial(coefficients, signs,
                root.LowerNumerator, root.DenominatorShift, 0);
        int rootDegree = root.Signs.Length - 1;
        if (degree < rootDegree)
            return GetReducedSignAtFiniteAxisRoot(ref root, coefficients, signs);

        // Positive pseudo-division preserves Q(alpha)'s sign. Each eliminated
        // coefficient multiplies by c=|leading(P)| and subtracts one product,
        // growing the height by at most Bp+1 bits. There are <=qDegree-pDegree+1
        // eliminations (at most eight), not a general polynomial PRS. This
        // often proves equality or a constant sign before any root refinement.
        int remainderWords = (GetFiniteRootCoefficientBits(coefficients, signs.Length)
            + (degree - rootDegree + 1) * (root.CoefficientBits + 1) + 127) / 64;
        Span<ulong> remainder = stackalloc ulong[(degree + 1) * remainderWords];
        Span<sbyte> remainderSigns = stackalloc sbyte[degree + 1];
        remainder.Clear();
        for (int index = 0; index <= degree; index++)
        {
            CopyFiniteRootMagnitude(coefficients.Slice(index * words, words),
                remainder.Slice(index * remainderWords, remainderWords));
            remainderSigns[index] = signs[index];
        }
        int remainderDegree = ReduceFiniteRootQuery(root, remainder, remainderSigns, degree);
        if (remainderDegree < 0)
            return 0;
        if (remainderDegree == 0)
            return remainderSigns[0];
        return GetReducedSignAtFiniteAxisRoot(ref root,
            remainder[..((remainderDegree + 1) * remainderWords)], remainderSigns[..(remainderDegree + 1)]);
    }

    private static int GetReducedSignAtFiniteAxisRoot(ref FiniteAxisPolynomialRoot root,
        scoped ReadOnlySpan<ulong> coefficients, scoped ReadOnlySpan<sbyte> signs)
    {
        int certified = GetFiniteRootIntervalSign(root, coefficients, signs);
        if (certified != 0)
            return certified;

        if (signs.Length == 2)
        {
            // Linear interval evaluation is exact. An uncertified sign puts
            // its zero -q0/q1 inside this positive closed cell, so q0 and q1
            // are nonzero and opposite. The cell has exactly one distinct
            // P root and neither endpoint is a root. Evaluate P at that
            // positive fraction to prove equality, or locate a crossing root.
            int words = coefficients.Length / 2;
            int boundarySign = EvaluateFiniteRootAtPositiveFraction(root,
                coefficients[..words], coefficients[words..]);
            if (boundarySign == 0)
                return 0;
            if (root.CrossingSign != 0)
                return boundarySign == root.CrossingSign ? signs[1] : -signs[1];
        }

        // Sixty-four exact refinements are only a fast-path budget, never an
        // approximate decision. The Hermite query below handles every
        // remaining case, including shared roots and arbitrarily close values.
        if (root.DenominatorShift < 8 * root.CoefficientBits + 128)
        {
            int sturmWords = root.CrossingSign == 0 ? (7 * root.CoefficientBits + 95) / 64 : 0;
            Span<ulong> sturm = stackalloc ulong[25 * sturmWords];
            Span<sbyte> sturmSigns = stackalloc sbyte[25];
            Span<int> degrees = stackalloc int[5];
            int count = root.CrossingSign == 0 ? BuildFiniteRootSturm(root, sturm, sturmSigns, degrees) : 0;
            int infinity = root.CrossingSign == 0
                ? GetRoundedCylinderInfinityVariations(sturmSigns, degrees, count, false) : 0;
            for (int attempt = 0; attempt < 64
                 && root.DenominatorShift < 8 * root.CoefficientBits + 128; attempt++)
            {
                if (root.CrossingSign == 0)
                    RefineFiniteRoot(ref root, sturm, sturmSigns, degrees, count, infinity, 1);
                else
                    RefineFiniteCrossingRoot(ref root);
                if (root.IsRational)
                    return EvaluateFiniteRootPolynomial(coefficients, signs,
                        root.LowerNumerator, root.DenominatorShift, 0);
                certified = GetFiniteRootIntervalSign(root, coefficients, signs);
                if (certified != 0)
                    return certified;
            }
        }
        return GetFiniteRootHermiteIntervalSign(root, coefficients, signs);
    }

    private static int EvaluateFiniteRootAtPositiveFraction(FiniteAxisPolynomialRoot root,
        ReadOnlySpan<ulong> numerator, ReadOnlySpan<ulong> denominator)
    {
        int degree = root.Signs.Length - 1;
        int inputWords = root.Coefficients.Length / root.Signs.Length;
        // Homogeneous Horner evaluation computes denominator^degree * P(n/d).
        // At most five terms need Bp + degree*max(Bn,Bd) + 3 magnitude bits.
        int words = (root.CoefficientBits
            + degree * Math.Max(GetFiniteRootBits(numerator), GetFiniteRootBits(denominator)) + 66) / 64;
        Span<ulong> result = stackalloc ulong[words];
        Span<ulong> denominatorPower = stackalloc ulong[words];
        Span<ulong> first = stackalloc ulong[words];
        Span<ulong> second = stackalloc ulong[words];
        Span<ulong> sum = stackalloc ulong[words];
        CopyFiniteRootMagnitude(root.Coefficients.Slice(degree * inputWords, inputWords), result);
        sbyte resultSign = root.Signs[degree];
        denominatorPower.Clear();
        denominatorPower[0] = 1;
        for (int index = degree - 1; index >= 0; index--)
        {
            MultiplyRoundedCylinderWide(denominatorPower, denominator, first);
            first.CopyTo(denominatorPower);
            MultiplyRoundedCylinderWide(result, numerator, first);
            MultiplyRoundedCylinderWide(root.Coefficients.Slice(index * inputWords, inputWords),
                denominatorPower, second);
            AddRoundedCylinderSigned(first, resultSign, second, root.Signs[index], sum, out resultSign);
            sum.CopyTo(result);
        }
        return resultSign;
    }

    private static void RefineFiniteCrossingRoot(ref FiniteAxisPolynomialRoot root)
    {
        // Opposite endpoint signs and exactly one distinct enclosed root
        // permit ordinary sign bisection, even for an odd multiple root.
        // No Sturm sequence is needed again unless the root does not cross.
        ShiftFiniteRootLeft(root.LowerNumerator, 1);
        AddRoundedCylinderWord(root.LowerNumerator, 0, 1);
        root.DenominatorShift++;
        int sign = EvaluateFiniteRootPolynomial(root.Coefficients, root.Signs,
            root.LowerNumerator, root.DenominatorShift, 0);
        if (sign == 0)
            root.IsRational = true;
        else if (sign != root.CrossingSign)
            root.LowerNumerator[0]--;
    }

    internal static void NormalizeFiniteAxisPolynomialPowerOfTwo(
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
            return;
        for (int index = 0; index < signs.Length; index++)
        {
            if (signs[index] != 0)
                ShiftRoundedCylinderWideRight(coefficients.Slice(index * words, words), shift);
        }
    }

    private static int ReduceFiniteRootQuery(FiniteAxisPolynomialRoot root,
        Span<ulong> remainder, Span<sbyte> signs, int degree)
    {
        int words = remainder.Length / signs.Length;
        int rootDegree = root.Signs.Length - 1;
        int rootWords = root.Coefficients.Length / root.Signs.Length;
        ReadOnlySpan<ulong> leading = root.Coefficients.Slice(rootDegree * rootWords, rootWords);
        Span<ulong> factor = stackalloc ulong[words];
        Span<ulong> first = stackalloc ulong[words];
        Span<ulong> second = stackalloc ulong[words];
        while (degree >= rootDegree)
        {
            remainder.Slice(degree * words, words).CopyTo(factor);
            sbyte factorSign = (sbyte)(signs[degree] * root.Signs[rootDegree]);
            int offset = degree - rootDegree;
            for (int index = 0; index <= degree; index++)
            {
                Span<ulong> coefficient = remainder.Slice(index * words, words);
                MultiplyRoundedCylinderWide(leading, coefficient, first);
                sbyte secondSign = 0;
                second.Clear();
                if (index >= offset)
                {
                    MultiplyRoundedCylinderWide(factor,
                        root.Coefficients.Slice((index - offset) * rootWords, rootWords), second);
                    secondSign = (sbyte)(factorSign * root.Signs[index - offset]);
                }
                SubtractRoundedCylinderSigned(first, signs[index], second, secondSign,
                    coefficient, out signs[index]);
            }
            while (degree >= 0 && signs[degree] == 0)
                degree--;
            if (degree < 0)
                return -1;
            NormalizeFiniteAxisPolynomialPowerOfTwo(remainder, signs);
        }
        return degree;
    }

    private static int BuildFiniteRootSturm(FiniteAxisPolynomialRoot root,
        Span<ulong> sturm, Span<sbyte> signs, Span<int> degrees)
    {
        sturm.Clear();
        signs.Clear();
        degrees.Clear();
        int words = root.Coefficients.Length / root.Signs.Length;
        for (int index = 0; index < root.Signs.Length; index++)
        {
            CopyFiniteRootMagnitude(root.Coefficients.Slice(index * words, words), GetRoundedCylinderCoefficient(sturm, 0, index));
            signs[index] = root.Signs[index];
        }
        degrees[0] = root.Signs.Length - 1;
        return BuildFiniteAxisPolynomialSturmSequence(sturm, signs, degrees);
    }

    private static int RefineFiniteRoot(ref FiniteAxisPolynomialRoot root,
        scoped Span<ulong> sturm, scoped Span<sbyte> signs, scoped Span<int> degrees,
        int count, int infinity, int remaining)
    {
        ShiftFiniteRootLeft(root.LowerNumerator, 1);
        AddRoundedCylinderWord(root.LowerNumerator, 0, 1);
        root.DenominatorShift++;
        int right = GetFiniteRootVariations(sturm, signs, degrees, count,
            root.LowerNumerator, root.DenominatorShift) - infinity;
        if (right != 0)
            return right;
        if (EvaluateFiniteRootPolynomial(root.Coefficients, root.Signs,
                root.LowerNumerator, root.DenominatorShift, 0) == 0)
        {
            root.IsRational = true;
            return 1;
        }
        // The midpoint numerator is odd, so this subtraction cannot borrow.
        root.LowerNumerator[0]--;
        return remaining;
    }

    private static int GetFiniteRootVariations(Span<ulong> sturm, Span<sbyte> signs,
        Span<int> degrees, int count, ReadOnlySpan<ulong> numerator, int shift)
    {
        int previous = 0;
        int variations = 0;
        int words = sturm.Length / 25;
        for (int index = 0; index < count; index++)
        {
            int degree = degrees[index];
            int sign = 0;
            for (int order = 0; order <= degree && sign == 0; order++)
            {
                sign = EvaluateFiniteRootPolynomial(sturm.Slice(index * 5 * words, (degree + 1) * words),
                    signs.Slice(index * 5, degree + 1), numerator, shift, order);
            }
            if (previous != 0 && previous != sign)
                variations++;
            previous = sign;
        }
        return variations;
    }

    private static int EvaluateFiniteRootPolynomial(ReadOnlySpan<ulong> coefficients,
        ReadOnlySpan<sbyte> signs, ReadOnlySpan<ulong> numerator, int shift, int derivative)
    {
        int degree = signs.Length - 1;
        if (degree == derivative)
            return signs[degree];
        int inputWords = coefficients.Length / signs.Length;
        int boundBits = Math.Max(GetFiniteRootBits(numerator) + Math.Max(-shift, 0), Math.Max(shift, 0) + 1);
        int words = (GetFiniteRootCoefficientBits(coefficients, signs.Length)
            + (degree - derivative) * boundBits + 95) / 64;
        Span<ulong> n = stackalloc ulong[words];
        Span<ulong> result = stackalloc ulong[words];
        Span<ulong> product = stackalloc ulong[words];
        Span<ulong> term = stackalloc ulong[words];
        Span<ulong> sum = stackalloc ulong[words];
        n.Clear();
        numerator[..GetRoundedCylinderWideLength(numerator)].CopyTo(n);
        if (shift < 0)
            ShiftFiniteRootLeft(n, -shift);
        result.Clear();
        sbyte resultSign = 0;
        for (int index = degree; index >= derivative; index--)
        {
            MultiplyRoundedCylinderWide(result, n, product);
            ulong binomial = 1;
            for (int factor = 1; factor <= derivative; factor++)
                binomial = binomial * (ulong)(index - factor + 1) / (ulong)factor;
            MultiplyRoundedCylinderWideByWord(coefficients.Slice(index * inputWords, inputWords), binomial, term);
            if (shift > 0)
                ShiftFiniteRootLeft(term, (degree - index) * shift);
            AddRoundedCylinderSigned(product, resultSign, term, signs[index], sum, out resultSign);
            sum.CopyTo(result);
        }
        return resultSign;
    }

    private static int GetFiniteRootIntervalSign(FiniteAxisPolynomialRoot root,
        ReadOnlySpan<ulong> coefficients, ReadOnlySpan<sbyte> signs)
    {
        int degree = signs.Length - 1;
        int inputWords = coefficients.Length / signs.Length;
        int boundBits = Math.Max(GetFiniteRootBits(root.LowerNumerator) + 1
            + Math.Max(-root.DenominatorShift, 0), Math.Max(root.DenominatorShift, 0) + 1);
        int words = (GetFiniteRootCoefficientBits(coefficients, signs.Length) + degree * boundBits + 95) / 64;
        Span<ulong> lower = stackalloc ulong[words];
        Span<ulong> upper = stackalloc ulong[words];
        Span<ulong> minimum = stackalloc ulong[words];
        Span<ulong> maximum = stackalloc ulong[words];
        Span<ulong> product = stackalloc ulong[words];
        Span<ulong> term = stackalloc ulong[words];
        Span<ulong> sum = stackalloc ulong[words];
        lower.Clear();
        root.LowerNumerator[..GetRoundedCylinderWideLength(root.LowerNumerator)].CopyTo(lower);
        lower.CopyTo(upper);
        AddRoundedCylinderWord(upper, 0, 1);
        if (root.DenominatorShift < 0)
        {
            ShiftFiniteRootLeft(lower, -root.DenominatorShift);
            ShiftFiniteRootLeft(upper, -root.DenominatorShift);
        }
        minimum.Clear();
        maximum.Clear();
        sbyte minimumSign = 0;
        sbyte maximumSign = 0;
        for (int index = degree; index >= 0; index--)
        {
            term.Clear();
            CopyFiniteRootMagnitude(coefficients.Slice(index * inputWords, inputWords), term);
            if (root.DenominatorShift > 0)
                ShiftFiniteRootLeft(term, (degree - index) * root.DenominatorShift);
            MultiplyRoundedCylinderWide(minimum, minimumSign < 0 ? upper : lower, product);
            AddRoundedCylinderSigned(product, minimumSign, term, signs[index], sum, out minimumSign);
            sum.CopyTo(minimum);
            MultiplyRoundedCylinderWide(maximum, maximumSign < 0 ? lower : upper, product);
            AddRoundedCylinderSigned(product, maximumSign, term, signs[index], sum, out maximumSign);
            sum.CopyTo(maximum);
        }
        return minimumSign > 0 ? 1 : maximumSign < 0 ? -1 : 0;
    }

    private static int GetFiniteRootCoefficientBits(ReadOnlySpan<ulong> coefficients, int count)
    {
        int words = coefficients.Length / count;
        int bits = 0;
        for (int index = 0; index < count; index++)
            bits = Math.Max(bits, GetFiniteRootBits(coefficients.Slice(index * words, words)));
        return bits;
    }

    private static int GetFiniteRootBits(ReadOnlySpan<ulong> value)
    {
        int words = GetRoundedCylinderWideLength(value);
        return words == 0 ? 0 : words * 64 - Fixed64.CountLeadingZeroes(value[words - 1]);
    }

    private static void CopyFiniteRootMagnitude(ReadOnlySpan<ulong> source, Span<ulong> destination)
    {
        destination.Clear();
        source[..GetRoundedCylinderWideLength(source)].CopyTo(destination);
    }

    private static void ShiftFiniteRootLeft(Span<ulong> value, int bits)
    {
        int wordShift = bits >> 6;
        int bitShift = bits & 63;
        for (int index = value.Length - 1; index >= 0; index--)
        {
            int source = index - wordShift;
            ulong low = source >= 0 ? value[source] : 0;
            ulong high = source > 0 ? value[source - 1] : 0;
            value[index] = (low << bitShift) | (bitShift == 0 ? 0 : high >> (64 - bitShift));
        }
    }
}
