//=======================================================================
// MIT License, Copyright (c) 2024-present David Oravsky (mrdav30)
// See LICENSE file in the project root for full license information.
//=======================================================================
using System;
using System.Runtime.CompilerServices;

namespace FixedMathSharp.Geometry;

/// <summary>Exact comparison and materialization of retained convex contact value roots.</summary>
internal static class ConvexContactValueRoot
{
    /// <summary>Maps one admitted stationary parameter to its exact squared-gap value root.</summary>
    [MethodImpl(MethodImplOptions.NoInlining)]
    internal static FiniteAxisValueRoot MapSquaredValue(Signed192 rawScale, int valueShift,
        scoped ref FiniteAxisValueRoot parameter, scoped ReadOnlySpan<ulong> numerator,
        scoped ReadOnlySpan<sbyte> numeratorSigns, scoped ReadOnlySpan<ulong> denominator,
        scoped ReadOnlySpan<sbyte> denominatorSigns,
        ReadOnlySpan<ulong> values, ReadOnlySpan<sbyte> signs, Span<ulong> cell)
    {
        Span<ulong> batchCells = stackalloc ulong[64];
        Span<int> batchShifts = stackalloc int[8];
        FiniteAxisValueRoots roots = WideFiniteAxisIntersection.GetFiniteValueRoots(values, signs, batchCells, batchShifts);
        Span<ulong> upper = stackalloc ulong[cell.Length];
        for (int ordinal = 0; ordinal < roots.Count - 1; ordinal++)
        {
            FiniteAxisValueRoot root = FiniteAxisValueRoots.GetRoot(roots, ordinal, values, signs, cell);
            if (root.IsRational)
            {
                if (CompareSquaredValueEndpoint(rawScale, valueShift, ref parameter,
                        numerator, numeratorSigns, denominator, denominatorSigns, cell, root.DenominatorShift) == 0)
                    return root;
            }
            else
            {
                cell.CopyTo(upper);
                WideArithmetic.AddWord(upper, 0, 1);
                if (CompareSquaredValueEndpoint(rawScale, valueShift, ref parameter,
                        numerator, numeratorSigns, denominator, denominatorSigns, upper, root.DenominatorShift) < 0)
                    return root;
            }
        }
        System.Diagnostics.Debug.Assert(roots.Count > 0);
        return FiniteAxisValueRoots.GetRoot(roots, roots.Count - 1, values, signs, cell);
    }

    [MethodImpl(MethodImplOptions.NoInlining)]
    private static int CompareSquaredValueEndpoint(Signed192 rawScale, int valueShift,
        scoped ref FiniteAxisValueRoot parameter, scoped ReadOnlySpan<ulong> numerator,
        scoped ReadOnlySpan<sbyte> numeratorSigns, scoped ReadOnlySpan<ulong> denominator,
        scoped ReadOnlySpan<sbyte> denominatorSigns, scoped ReadOnlySpan<ulong> endpoint, int shift)
    {
        int sourceWords = numerator.Length / numeratorSigns.Length;
        int words = (shift + valueShift + sourceWords * 64 + 384 + 127) / 64;
        Span<ulong> query = stackalloc ulong[5 * words];
        Span<sbyte> signs = stackalloc sbyte[5];
        Span<ulong> product = stackalloc ulong[words];
        Span<ulong> scale = stackalloc ulong[3];
        Span<ulong> scaleSquared = stackalloc ulong[6];
        query.Clear(); signs.Clear();
        WideArithmetic.GetMagnitude(rawScale, out scale[2], out scale[1], out scale[0]);
        WideArithmetic.MultiplyMagnitudes(scale, scale, scaleSquared);
        for (int index = 0; index < 5; index++)
        {
            Span<ulong> target = query.Slice(index * words, words);
            int sign = 0;
            WideArithmetic.MultiplyMagnitudes(numerator.Slice(index * sourceWords, sourceWords), scaleSquared, product);
            WideArithmetic.AddShiftedSignedMagnitude(product, numeratorSigns[index], shift, target, ref sign);
            WideArithmetic.MultiplyMagnitudes(denominator.Slice(index * sourceWords, sourceWords), endpoint, product);
            WideArithmetic.AddShiftedSignedMagnitude(product, -denominatorSigns[index], valueShift, target, ref sign);
            signs[index] = (sbyte)sign;
        }
        return WideFiniteAxisIntersection.GetSignAtFiniteValueRootAndRefine(ref parameter, query, signs);
    }

    /// <summary>
    /// Compares S/RawScale², where S=2^ValueShift*t, to an analytic squared
    /// raw gap. Gap signs are deliberately excluded; the caller ranks them first.
    /// </summary>
    internal static int CompareRootSquared(Signed192 rawScale, int valueShift,
        FiniteAxisValueRoot root, ConvexContactCandidate candidate)
    {
        Span<ulong> scale = stackalloc ulong[3];
        WideArithmetic.GetMagnitude(rawScale, out scale[2], out scale[1], out scale[0]);
        Span<ulong> squaredScale = stackalloc ulong[6];
        WideArithmetic.MultiplyMagnitudes(scale, scale, squaredScale);
        int scaleWords = WideArithmetic.GetActiveMagnitudeLength(squaredScale);
        int linearWords = Math.Max(
            WideArithmetic.GetActiveMagnitudeLength(candidate.GapRational) + scaleWords,
            WideArithmetic.GetActiveMagnitudeLength(candidate.GapDenominator) + (valueShift + 63) / 64);
        int radicalWords = WideArithmetic.GetActiveMagnitudeLength(candidate.GapRadical) + scaleWords;
        int words = Math.Max(ConvexContactCandidate.Words, Math.Max(2 * linearWords + 1,
            2 * radicalWords + WideArithmetic.GetActiveMagnitudeLength(candidate.GapRadicand) + 1));
        Span<ulong> linear = stackalloc ulong[2 * words];
        Span<sbyte> linearSigns = stackalloc sbyte[2];
        // R=D*2^ValueShift*t-A*RawScale². Compare R with B*RawScale²*sqrt(C).
        WideArithmetic.MultiplyMagnitudes(candidate.GapRational, squaredScale, linear[..words]);
        linearSigns[0] = (sbyte)-candidate.GapRationalSign;
        linear[words..].Clear();
        candidate.GapDenominator.CopyTo(linear[words..]);
        linearSigns[1] = 1;
        WideFiniteAxisIntersection.ScaleFiniteAxisPolynomialVariable(linear, 2, valueShift);
        int rationalSign = WideFiniteAxisIntersection.GetSignAtFiniteValueRoot(root, linear, linearSigns);
        int radicalSign = WideArithmetic.GetActiveMagnitudeLength(candidate.GapRadicand) == 0
            ? 0 : candidate.GapRadicalSign;
        if (rationalSign != radicalSign)
            return rationalSign.CompareTo(radicalSign);
        if (rationalSign == 0)
            return 0;

        // Active-width bounds include scale multiplication, variable shifting,
        // both squares, and the radical product. This is independent of the
        // shape-specific candidate builder's coefficient-height proof.
        Span<ulong> query = stackalloc ulong[3 * words];
        Span<sbyte> querySigns = stackalloc sbyte[3];
        Span<ulong> product = stackalloc ulong[words];
        Span<ulong> temporary = stackalloc ulong[words];
        WideFiniteAxisIntersection.MultiplyFiniteAxisPolynomials(linear, linearSigns, linear, linearSigns,
            query, querySigns, product);
        WideArithmetic.MultiplyMagnitudes(candidate.GapRadical, squaredScale, temporary);
        WideArithmetic.MultiplyMagnitudes(temporary, temporary, product);
        WideArithmetic.MultiplyMagnitudes(product, candidate.GapRadicand, temporary);
        int constantSign = querySigns[0];
        WideArithmetic.AddShiftedSignedMagnitude(temporary, -1, 0, query[..words], ref constantSign);
        querySigns[0] = (sbyte)constantSign;
        WideFiniteAxisIntersection.NormalizeFiniteAxisPolynomialPowerOfTwo(query, querySigns);
        return rationalSign * WideFiniteAxisIntersection.GetSignAtFiniteValueRoot(root, query, querySigns);
    }

    internal static Vector3d GetNormalizedDirection(ref FiniteAxisValueRoot root,
        scoped ReadOnlySpan<ulong> gradient, scoped ReadOnlySpan<sbyte> gradientSigns, int orientation) =>
        GetScaledNormalizedDirection(ref root, gradient, gradientSigns, Fixed64.One, orientation);

    internal static Vector3d GetScaledNormalizedDirection(ref FiniteAxisValueRoot root,
        scoped ReadOnlySpan<ulong> gradient, scoped ReadOnlySpan<sbyte> gradientSigns, Fixed64 scale, int orientation)
    {
        int count = gradientSigns.Length / 3;
        int words = gradient.Length / gradientSigns.Length;
        int squareCount = 2 * count - 1;
        Span<ulong> squaredLength = stackalloc ulong[squareCount * 2 * words];
        Span<sbyte> squaredLengthSigns = stackalloc sbyte[squareCount];
        BuildSquaredLength(gradient, gradientSigns, squaredLength, squaredLengthSigns);
        return new Vector3d(
            GetRoundedNormalComponent(ref root, gradient[..(count * words)], gradientSigns[..count],
                squaredLength, squaredLengthSigns, scale, orientation),
            GetRoundedNormalComponent(ref root, gradient.Slice(count * words, count * words),
                gradientSigns.Slice(count, count), squaredLength, squaredLengthSigns, scale, orientation),
            GetRoundedNormalComponent(ref root, gradient[(2 * count * words)..], gradientSigns[(2 * count)..],
                squaredLength, squaredLengthSigns, scale, orientation));
    }

    internal static void GetRoundedDepth(Signed192 rawScale, int valueShift, ref FiniteAxisValueRoot root,
        out Fixed64 depth, out bool clamped)
    {
        Span<ulong> scale = stackalloc ulong[3];
        WideArithmetic.GetMagnitude(rawScale, out scale[2], out scale[1], out scale[0]);
        Span<ulong> squaredScale = stackalloc ulong[6];
        WideArithmetic.MultiplyMagnitudes(scale, scale, squaredScale);
        int maximum = CompareDepthToTwiceRaw(root, squaredScale, valueShift,
            unchecked((ulong)long.MaxValue << 1));
        if (maximum >= 0)
        {
            depth = Fixed64.MaxValue;
            clamped = maximum > 0;
            return;
        }
        // All rounding thresholds lie on this dyadic grid. Once the exact
        // selected root occupies one grid cell, none can be strictly inside
        // it: subsequent comparisons reuse the cell without rebuilding Sturm
        // chains. A root on a grid point is retained as an exact singleton.
        WideFiniteAxisIntersection.RefineFiniteValueRoot(ref root, valueShift + 2);
        ulong lower = 0;
        ulong upper = 1UL << 63;
        while (lower < upper)
        {
            ulong midpoint = lower + ((upper - lower) >> 1);
            if (CompareDepthToTwiceRaw(root, squaredScale, valueShift, midpoint << 1) >= 0)
                lower = midpoint + 1;
            else
                upper = midpoint;
        }
        ulong floor = lower - 1;
        int comparison = CompareDepthToTwiceRaw(root, squaredScale, valueShift, (floor << 1) | 1);
        depth = Fixed64.FromRaw((long)(floor + (comparison > 0 || (comparison == 0 && (floor & 1) != 0) ? 1UL : 0)));
        clamped = false;
    }

    private static int CompareDepthToTwiceRaw(FiniteAxisValueRoot root,
        ReadOnlySpan<ulong> squaredScale, int valueShift, ulong twiceRaw)
    {
        Fixed64.Multiply64To128(twiceRaw, twiceRaw, out ulong high, out ulong low);
        Span<ulong> squaredRaw = stackalloc ulong[2] { low, high };
        Span<ulong> threshold = stackalloc ulong[8];
        WideArithmetic.MultiplyMagnitudes(squaredScale, squaredRaw, threshold);
        return WideFiniteAxisIntersection.CompareFiniteValueRootToDyadic(root, threshold, valueShift + 2);
    }

    [MethodImpl(MethodImplOptions.NoInlining)]
    private static void BuildSquaredLength(ReadOnlySpan<ulong> gradient, ReadOnlySpan<sbyte> signs,
        Span<ulong> squaredLength, Span<sbyte> squaredLengthSigns)
    {
        int count = signs.Length / 3;
        int words = gradient.Length / signs.Length;
        Span<ulong> product = stackalloc ulong[2 * words];
        for (int axis = 0; axis < 3; axis++)
        {
            ReadOnlySpan<ulong> component = gradient.Slice(axis * count * words, count * words);
            ReadOnlySpan<sbyte> componentSigns = signs.Slice(axis * count, count);
            WideFiniteAxisIntersection.MultiplyFiniteAxisPolynomials(component, componentSigns,
                component, componentSigns, squaredLength, squaredLengthSigns, product, axis != 0);
        }
    }

    private static Fixed64 GetRoundedNormalComponent(ref FiniteAxisValueRoot root,
        scoped ReadOnlySpan<ulong> component, scoped ReadOnlySpan<sbyte> componentSigns,
        scoped ReadOnlySpan<ulong> squaredLength, scoped ReadOnlySpan<sbyte> squaredLengthSigns, Fixed64 scale, int orientation)
    {
        int sign = orientation * WideFiniteAxisIntersection.GetSignAtFiniteValueRootAndRefine(ref root, component, componentSigns);
        if (sign == 0 || scale == Fixed64.Zero)
            return Fixed64.Zero;
        int squareCount = squaredLengthSigns.Length;
        int queryWords = squaredLength.Length / squareCount + 2;
        Span<ulong> query = stackalloc ulong[squareCount * queryWords];
        Span<sbyte> querySigns = stackalloc sbyte[squareCount];
        ulong lower = 0;
        ulong upper = (ulong)scale.m_rawValue + 1;
        ulong previousTwiceRaw = 0;
        int removedShift = -1;
        while (lower < upper)
        {
            ulong midpoint = lower + ((upper - lower) >> 1);
            BuildNormalThreshold(component, componentSigns, squaredLength, squaredLengthSigns,
                midpoint << 1, (ulong)scale.m_rawValue, ref previousTwiceRaw, ref removedShift, query, querySigns);
            if (WideFiniteAxisIntersection.GetSignAtFiniteValueRootAndRefine(ref root, query, querySigns) >= 0)
                lower = midpoint + 1;
            else
                upper = midpoint;
        }
        ulong floor = lower - 1;
        BuildNormalThreshold(component, componentSigns, squaredLength, squaredLengthSigns,
            (floor << 1) | 1, (ulong)scale.m_rawValue, ref previousTwiceRaw, ref removedShift, query, querySigns);
        int comparison = WideFiniteAxisIntersection.GetSignAtFiniteValueRootAndRefine(ref root, query, querySigns);
        ulong rounded = floor + (comparison > 0 || (comparison == 0 && (floor & 1) != 0) ? 1UL : 0);
        return Fixed64.FromRaw(sign * (long)rounded);
    }

    [MethodImpl(MethodImplOptions.NoInlining)]
    private static void BuildNormalThreshold(ReadOnlySpan<ulong> component, ReadOnlySpan<sbyte> componentSigns,
        ReadOnlySpan<ulong> squaredLength, ReadOnlySpan<sbyte> squaredLengthSigns,
        ulong twiceRaw, ulong scale, ref ulong previousTwiceRaw, ref int removedShift,
        Span<ulong> query, Span<sbyte> querySigns)
    {
        int squareCount = squaredLengthSigns.Length;
        int squareWords = squaredLength.Length / squareCount;
        int queryWords = query.Length / querySigns.Length;
        Span<ulong> product = stackalloc ulong[queryWords];
        if (removedShift < 0)
        {
            WideFiniteAxisIntersection.MultiplyFiniteAxisPolynomials(component, componentSigns,
                component, componentSigns, query, querySigns, product);
            if (scale == (ulong)Fixed64.One.m_rawValue)
                removedShift = 66;
            else
            {
                Span<ulong> factor = stackalloc ulong[2];
                Fixed64.Multiply64To128(scale << 1, scale << 1, out factor[1], out factor[0]);
                for (int index = 0; index < squareCount; index++)
                {
                    Span<ulong> target = query.Slice(index * queryWords, queryWords);
                    WideArithmetic.MultiplyMagnitudes(target, factor, product);
                    product.CopyTo(target);
                }
                removedShift = 0;
            }
        }
        // Q_k=4*scale²*C²-k²*L. Restore the previous normalization and use
        // Q_new=Q_old+(old²-new²)*L, retaining C² without a separate buffer.
        // Both k values fit 64 unsigned bits, so the signed difference needs
        // at most 128 magnitude bits. The raw polynomial has the same height
        // bound; the existing two extra coefficient words remain sufficient.
        Fixed64.Multiply64To128(previousTwiceRaw, previousTwiceRaw, out ulong previousHigh, out ulong previousLow);
        Fixed64.Multiply64To128(twiceRaw, twiceRaw, out ulong high, out ulong low);
        Signed192 difference = WideArithmetic.SubtractSigned192(
            new Signed192(0, previousHigh, previousLow), new Signed192(0, high, low));
        Span<ulong> threshold = stackalloc ulong[2];
        WideArithmetic.GetMagnitude(difference, out _, out threshold[1], out threshold[0]);
        for (int index = 0; index < squareCount; index++)
        {
            Span<ulong> target = query.Slice(index * queryWords, queryWords);
            target.CopyTo(product);
            int shiftedSign = 0;
            target.Clear();
            WideArithmetic.AddShiftedSignedMagnitude(product, querySigns[index], removedShift, target, ref shiftedSign);
            WideArithmetic.MultiplyMagnitudes(squaredLength.Slice(index * squareWords, squareWords), threshold, product);
            WideArithmetic.AddShiftedSignedMagnitude(product, difference.Sign * squaredLengthSigns[index],
                0, target, ref shiftedSign);
            querySigns[index] = (sbyte)shiftedSign;
        }
        previousTwiceRaw = twiceRaw;
        removedShift = WideFiniteAxisIntersection.NormalizeFiniteAxisPolynomialPowerOfTwo(query, querySigns);
    }
}
