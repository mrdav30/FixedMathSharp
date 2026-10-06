//=======================================================================
// MIT License, Copyright (c) 2024-present David Oravsky (mrdav30)
// See LICENSE file in the project root for full license information.
//=======================================================================
using System;

namespace FixedMathSharp.Geometry;

/// <content>
/// Certified signs at retained value roots. A nonzero algebraic-value bound
/// makes equality terminate without rounding the root or adding a tolerance.
/// </content>
internal static partial class WideFiniteAxisIntersection
{
    /// <summary>
    /// Compares the selected root to a nonnegative dyadic threshold. Contact
    /// rounding uses at most 462 denominator bits; unlike a general sign
    /// query, counting roots at that threshold needs no root refinement.
    /// </summary>
    internal static int CompareFiniteValueRootToDyadic(FiniteAxisValueRoot root,
        ReadOnlySpan<ulong> numerator, int shift)
    {
        int numeratorBits = GetFiniteRootBits(numerator);
        if (numeratorBits == 0)
            return 1;
        if (numeratorBits > shift)
        {
            bool exactlyOne = numeratorBits == shift + 1
                && CountRoundedCylinderTrailingZeroes(numerator) == shift;
            return exactlyOne && root.IsRational && root.DenominatorShift == 0 ? 0 : -1;
        }
        // Reuse the certified cell before constructing a Sturm chain. With
        // zero radius, the shared mapped-endpoint comparison is just an exact
        // dyadic comparison. Nonrational cell endpoints are strictly excluded.
        int lowerComparison = CompareRadicalOffsetEndpoints(root.LowerNumerator, root.DenominatorShift, false,
            default, 0, 1, numerator, shift, false);
        if (root.IsRational)
            return lowerComparison;
        if (lowerComparison >= 0)
            return 1;
        if (CompareRadicalOffsetEndpoints(root.LowerNumerator, root.DenominatorShift, true,
                default, 0, 1, numerator, shift, false) <= 0)
            return -1;
        int bits = GetFiniteRootCoefficientBits(root.Coefficients, root.Signs.Length);
        Span<ulong> arena = stackalloc ulong[(880 * (bits + 64) + 63) / 64 + 512];
        Span<int> offsets = stackalloc int[9];
        Span<int> widths = stackalloc int[9];
        Span<int> degrees = stackalloc int[9];
        Span<sbyte> signs = stackalloc sbyte[81];
        int count = BuildFiniteValueSturm(root.Coefficients, root.Signs, arena,
            offsets, widths, degrees, signs, out int used);
        Span<ulong> chain = arena[..used];
        Span<ulong> evaluation = arena[used..];
        Span<ulong> zero = stackalloc ulong[1] { 0 };
        int throughThreshold = GetFiniteValueVariations(chain, offsets, widths, degrees,
            signs, count, zero, 0, evaluation) - GetFiniteValueVariations(chain, offsets, widths,
            degrees, signs, count, numerator, shift, evaluation);
        if (root.Ordinal >= throughThreshold)
            return 1;
        // The threshold is strictly inside this root's isolating cell. Once
        // it includes that root, it cannot include another root after it.
        return EvaluateFiniteRootPolynomial(root.Coefficients, root.Signs, numerator, shift, 0, evaluation) == 0
            ? 0 : -1;
    }

    internal static int CompareFiniteValueRoots(FiniteAxisValueRoot first, FiniteAxisValueRoot second)
    {
        if (first.Signs.SequenceEqual(second.Signs) && first.Coefficients.SequenceEqual(second.Coefficients))
            return first.Ordinal.CompareTo(second.Ordinal);
        int firstBits = GetFiniteRootCoefficientBits(first.Coefficients, first.Signs.Length);
        int secondBits = GetFiniteRootCoefficientBits(second.Coefficients, second.Signs.Length);
        // For unequal selected roots, the resultant/factor separation bound
        // is >2^(-8*(Bf+Bg+22)-89), including a shared irreducible factor.
        // Equal-width overlapping cells below that gap prove equality.
        int certifiedShift = 8 * (firstBits + secondBits + 22) + 91;
        int shift = Math.Max(first.DenominatorShift, second.DenominatorShift);
        certifiedShift = Math.Max(certifiedShift, shift);
        int words = (certifiedShift + 127) / 64;
        Span<ulong> firstCell = stackalloc ulong[words];
        Span<ulong> secondCell = stackalloc ulong[words];
        CopyFiniteRootMagnitude(first.LowerNumerator, firstCell);
        CopyFiniteRootMagnitude(second.LowerNumerator, secondCell);
        var left = new FiniteAxisValueRoot
        {
            Coefficients = first.Coefficients, Signs = first.Signs,
            LowerNumerator = firstCell, DenominatorShift = first.DenominatorShift,
            IsRational = first.IsRational, Ordinal = first.Ordinal
        };
        var right = new FiniteAxisValueRoot
        {
            Coefficients = second.Coefficients, Signs = second.Signs,
            LowerNumerator = secondCell, DenominatorShift = second.DenominatorShift,
            IsRational = second.IsRational, Ordinal = second.Ordinal
        };
        int comparison;
        bool finished;
        do
        {
            RefineFiniteValueRoot(ref left, shift);
            RefineFiniteValueRoot(ref right, shift);
            // A singleton can be expressed at any finer denominator without
            // changing its value. Ordinary cells have exactly this width.
            ShiftFiniteRootLeft(firstCell, shift - left.DenominatorShift);
            ShiftFiniteRootLeft(secondCell, shift - right.DenominatorShift);
            left.DenominatorShift = right.DenominatorShift = shift;
            comparison = WideArithmetic.CompareMagnitudeEqualLength(firstCell, secondCell);
            bool singleton = left.IsRational || right.IsRational;
            if (comparison == 0 && singleton)
                comparison = left.IsRational == right.IsRational ? 0 : left.IsRational ? -1 : 1;
            finished = comparison != 0 || singleton || shift == certifiedShift;
            shift = Math.Min(certifiedShift, Math.Max(32, 2 * shift));
        } while (!finished);
        return comparison;
    }

    internal static int GetSignAtFiniteValueRoot(FiniteAxisValueRoot root,
        scoped ReadOnlySpan<ulong> coefficients, scoped ReadOnlySpan<sbyte> signs) =>
        GetSignAtFiniteValueRootCore(ref root, coefficients, signs, retainRefinement: false);

    /// <summary>
    /// Evaluates a sign and retains certified cell refinement when the caller's
    /// numerator storage has room. The numerator and its metadata are updated
    /// together; refinements beyond that storage remain local to this query.
    /// A knownLowerSign of -1 or +1 certifies opposite nonzero defining-polynomial
    /// signs at this cell's endpoints and remains valid for its descendants;
    /// zero requests exact endpoint evaluation.
    /// </summary>
    internal static int GetSignAtFiniteValueRootAndRefine(scoped ref FiniteAxisValueRoot root,
        scoped ReadOnlySpan<ulong> coefficients, scoped ReadOnlySpan<sbyte> signs, int knownLowerSign = 0) =>
        GetSignAtFiniteValueRootCore(ref root, coefficients, signs, retainRefinement: true, knownLowerSign);

    private static int GetSignAtFiniteValueRootCore(scoped ref FiniteAxisValueRoot root,
        scoped ReadOnlySpan<ulong> coefficients, scoped ReadOnlySpan<sbyte> signs, bool retainRefinement, int knownLowerSign = 0)
    {
        int degree = signs.Length - 1;
        int inputWords = coefficients.Length / signs.Length;
        while (degree > 0 && signs[degree] == 0)
            degree--;
        if (degree == 0)
            return signs[0];
        coefficients = coefficients[..((degree + 1) * inputWords)];
        signs = signs[..(degree + 1)];
        if (root.IsRational)
            return EvaluateFiniteRootPolynomial(coefficients, signs,
                root.LowerNumerator, root.DenominatorShift, 0);

        int n = root.Signs.Length - 1;
        int queryBits = GetFiniteRootCoefficientBits(coefficients, degree + 1);
        int queryCountBits = GetFiniteValueCeilingLog2(degree + 1);
        // The entire cell satisfies alpha <= 2^-k: N+1 <= 2^bitLength(N).
        // Evaluate z=2^k*alpha in [0,1], with virtual integer coefficients
        // c[i]*2^((degree-i)*k). No coefficient or retained cell is rewritten.
        int variableShift = Math.Max(0, root.DenominatorShift - GetFiniteRootBits(root.LowerNumerator));
        int evaluationBits = 0;
        for (int index = 0; index <= degree; index++)
            if (signs[index] != 0)
                evaluationBits = Math.Max(evaluationBits,
                    GetFiniteRootBits(coefficients.Slice(index * inputWords, inputWords))
                    + (degree - index) * variableShift);
        int normalizationBits = evaluationBits - degree * variableShift;
        // Scaling also needs k extra cell bits. Use it only when the final
        // refinement/scratch bound does not exceed the original one.
        if (normalizationBits + variableShift > queryBits)
        {
            variableShift = 0;
            evaluationBits = normalizationBits = queryBits;
        }
        // Earlier queries may already have narrowed this exact cell. Reuse
        // its certified width before copying or refining it: the same error
        // proof requires effective shift >= precision + 2*ceilLog2(m+1).
        // A nonzero result proves the sign at the retained root; uncertainty
        // must still reach the full resultant/equality bound below.
        // 59 uses more of the proved cell while retaining the same two-word
        // result budget for production queries of degree at most sixteen.
        int retainedPrecision = Math.Min(59, root.DenominatorShift - variableShift - 2 * queryCountBits);
        if (retainedPrecision > 0)
        {
            int retainedSign = GetFiniteValueApproximateSign(root.LowerNumerator, root.DenominatorShift,
                coefficients, signs, retainedPrecision, evaluationBits, variableShift);
            if (retainedSign != 0)
                return retainedSign;
        }
        // If Q(alpha)!=0, the integer resultant of Q and alpha's primitive
        // minimal polynomial has magnitude >=1. Mahler measure of that
        // factor is <=M(F)<=sqrt(n+1)*height(F). Since alpha is in (0,1],
        // |Q(alpha)| > 2^-nonzeroBits. No irreducible factor is constructed.
        int rootBits = GetFiniteRootCoefficientBits(root.Coefficients, n + 1);
        int nonzeroBits = (n - 1) * (queryBits + queryCountBits)
            + degree * (rootBits + GetFiniteValueCeilingLog2(n + 1));
        // P(z)/2^evaluationBits = Q(alpha)/2^normalizationBits, so the same
        // resultant bound applies. normalizationBits may be negative. For an
        // integer defining polynomial alpha>2^(-rootBits-1), hence k<=rootBits;
        // its leading query term gives normalizationBits>=1-degree*k. The
        // resulting certified precision is still strictly positive.
        int certifiedPrecision = nonzeroBits + normalizationBits + queryCountBits + 4;
        int maximumShift = Math.Max(root.DenominatorShift,
            certifiedPrecision + 2 * queryCountBits + variableShift);
        Span<ulong> cell = stackalloc ulong[(maximumShift + 127) / 64];
        CopyFiniteRootMagnitude(root.LowerNumerator, cell);
        var refined = new FiniteAxisValueRoot
        {
            Coefficients = root.Coefficients,
            Signs = root.Signs,
            LowerNumerator = cell,
            DenominatorShift = root.DenominatorShift,
            Ordinal = root.Ordinal
        };
        int precision = Math.Min(32, certifiedPrecision);
        int result;
        bool finished;
        do
        {
            RefineFiniteValueRoot(ref refined, precision + 2 * queryCountBits + variableShift, knownLowerSign);
            if (retainRefinement)
            {
                int activeWords = GetRoundedCylinderWideLength(refined.LowerNumerator);
                bool fits = activeWords <= root.LowerNumerator.Length;
                if (fits && !refined.IsRational && activeWords == root.LowerNumerator.Length)
                {
                    // An open cell also owns its upper endpoint. An all-ones
                    // lower numerator fits, but incrementing it would not.
                    int word = 0;
                    while (word < activeWords && refined.LowerNumerator[word] == ulong.MaxValue)
                        word++;
                    fits = word < activeWords;
                }
                if (fits)
                {
                    // Reuse only a complete proved cell, never a truncated
                    // numerator. The local expanded cell still owns all work
                    // beyond caller capacity, preserving the same final bound.
                    refined.LowerNumerator[..activeWords].CopyTo(root.LowerNumerator);
                    root.LowerNumerator[activeWords..].Clear();
                    root.DenominatorShift = refined.DenominatorShift;
                    root.IsRational = refined.IsRational;
                }
            }
            if (refined.IsRational)
                return EvaluateFiniteRootPolynomial(coefficients, signs,
                    refined.LowerNumerator, refined.DenominatorShift, 0);
            result = GetFiniteValueApproximateSign(refined.LowerNumerator, refined.DenominatorShift,
                coefficients, signs, precision, evaluationBits, variableShift);
            finished = result != 0 || precision == certifiedPrecision;
            precision = Math.Min(certifiedPrecision, 2 * precision);
        } while (!finished);
        return result;
    }

    private static int GetFiniteValueApproximateSign(ReadOnlySpan<ulong> numerator, int shift,
        ReadOnlySpan<ulong> coefficients, ReadOnlySpan<sbyte> signs, int precision, int coefficientBits,
        int variableShift = 0) =>
        GetFiniteValueApproximateSign(numerator, shift, coefficients, signs, precision, coefficientBits,
            out _, out _, variableShift);

    private static int GetFiniteValueApproximateSign(ReadOnlySpan<ulong> numerator, int shift,
        ReadOnlySpan<ulong> coefficients, ReadOnlySpan<sbyte> signs, int precision, int coefficientBits,
        out uint leadingMagnitude, out int magnitudeBits, int variableShift = 0)
    {
        int degree = signs.Length - 1;
        int words = (precision + GetFiniteValueCeilingLog2(degree + 1) + 127) / 64;
        Span<ulong> result = stackalloc ulong[words];
        Span<ulong> product = stackalloc ulong[words + (shift - variableShift + 64) / 64];
        int sign = GetFiniteValueApproximateSignCore(numerator, shift, coefficients, signs, precision, coefficientBits,
            result, product, out int length, variableShift);
        // Only prediction callers need a magnitude hint. Sturm counting borrows
        // the same evaluator directly and consumes only its certified sign.
        magnitudeBits = length == 0 ? 0 : 64 * length - Fixed64.CountLeadingZeroes(result[length - 1]);
        if (magnitudeBits <= 32)
            leadingMagnitude = magnitudeBits == 0 ? 0U : (uint)result[0] << (32 - magnitudeBits);
        else
        {
            int discarded = magnitudeBits - 32;
            int word = discarded / 64;
            int offset = discarded % 64;
            ulong leading = result[word] >> offset;
            if (offset > 32)
                leading |= result[word + 1] << (64 - offset);
            leadingMagnitude = (uint)leading;
        }
        return sign;
    }

    /// <summary>
    /// Encloses a polynomial nonnegative at the retained unit-interval root,
    /// normalized by 2^coefficientBits, in units of 2^-precision. At most 17
    /// coefficients and precision in [1,59] keep both endpoints within ulong:
    /// 17*2^59 + 34 is less than 2^64.
    /// coefficientBits must cover every coefficient. An open cell requires
    /// shift >= precision + 2*ceilLog2(coefficientCount); a singleton needs no
    /// cell-width margin. Neither coefficients nor the root are changed.
    /// </summary>
    internal static void GetFiniteValueRootNonnegativeBounds(FiniteAxisValueRoot root,
        ReadOnlySpan<ulong> coefficients, ReadOnlySpan<sbyte> signs, int precision, int coefficientBits,
        out ulong minimum, out ulong maximum)
    {
        Span<ulong> value = stackalloc ulong[2];
        Span<ulong> product = stackalloc ulong[2 + (root.DenominatorShift + 64) / 64];
        int sign = GetFiniteValueApproximateSignCore(root.LowerNumerator, root.DenominatorShift,
            coefficients, signs, precision, coefficientBits, value, product, out _);
        // The existing compact Horner proof bounds quantization, products and
        // cell movement by <2*count units. Reuse its full compact magnitude:
        // count*2^precision <2^64 proves the second result word is zero.
        ulong error = 2UL * (ulong)signs.Length;
        // A certified nonzero sign must be positive under the precondition.
        // Its magnitude exceeds error; uncertainty retains the nonnegative zero.
        minimum = sign == 0 ? 0 : value[0] - error;
        maximum = value[0] + error;
    }

    private static int GetFiniteValueApproximateSignCore(ReadOnlySpan<ulong> numerator, int shift,
        ReadOnlySpan<ulong> coefficients, ReadOnlySpan<sbyte> signs, int precision, int coefficientBits,
        Span<ulong> result, Span<ulong> product, out int length, int variableShift = 0)
    {
        int degree = signs.Length - 1;
        int inputWords = coefficients.Length / signs.Length;
        int words = result.Length;
        shift -= variableShift;
        result.Clear();
        int resultSign = 0;
        // Evaluate the virtual polynomial P/2^coefficientBits at the supplied
        // dyadic point in [0,1]. Point-refinement callers use variableShift=0.
        // Coefficient quantization contributes <degree+1 units and product
        // truncation <degree units. A nonzero result certifies the point sign;
        // zero alone is uncertain, and midpoint refinement then evaluates exactly.
        // The root-query caller additionally enforces
        // shift>=precision+2*ceilLog2(m+1), so the normalized derivative
        // m(m+1)/2 bounds its cell error by <1/2 unit. Its total error remains
        // <2*(degree+1). At that caller's final precision any nonzero
        // normalized value exceeds 16*(degree+1) units: an uncertain result
        // there proves exact zero, just as in the unnormalized formulation.
        for (int index = degree; index >= 0; index--)
        {
            WideArithmetic.MultiplyMagnitudes(result, numerator, product);
            ShiftRoundedCylinderWideRight(product, shift);
            product[..words].CopyTo(result);
            if (GetRoundedCylinderWideLength(result) == 0)
                resultSign = 0;
            ReadOnlySpan<ulong> coefficient = coefficients.Slice(index * inputWords, inputWords);
            int retainedBits = precision - coefficientBits + (degree - index) * variableShift;
            if (retainedBits >= 0)
            {
                WideArithmetic.AddShiftedSignedMagnitude(coefficient, signs[index],
                    retainedBits, result, ref resultSign);
            }
            else
            {
                // Drop whole low words before copying, so even a huge input
                // coefficient fits the compact product buffer. Only 0..63
                // remaining low bits need shifting. Magnitude truncation
                // followed by the original sign is truncation toward zero.
                int discarded = -retainedBits;
                // Virtual normalization can discard the complete stored
                // coefficient; an empty source quantizes to zero exactly.
                CopyFiniteRootMagnitude(coefficient[Math.Min(discarded / 64, inputWords)..], product);
                ShiftRoundedCylinderWideRight(product, discarded % 64);
                int coefficientSign = GetRoundedCylinderWideLength(product) == 0 ? 0 : signs[index];
                WideArithmetic.AddShiftedSignedMagnitude(product, coefficientSign, 0, result, ref resultSign);
            }
        }
        length = GetRoundedCylinderWideLength(result);
        return length > 1 || (length == 1 && result[0] > 2UL * (ulong)(degree + 1)) ? resultSign : 0;
    }

    private static int GetFiniteValueCeilingLog2(int value) =>
        // Only nonconstant defining and query polynomials reach these callers.
        64 - Fixed64.CountLeadingZeroes((ulong)(value - 1));
}
