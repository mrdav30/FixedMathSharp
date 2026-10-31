//=======================================================================
// ConvexContactValueRoot.cs
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
    internal static FiniteAxisValueRoot MapSquaredValue(Signed192 rawScale, int valueShift,
        scoped ref FiniteAxisValueRoot parameter, scoped ReadOnlySpan<ulong> numerator,
        scoped ReadOnlySpan<sbyte> numeratorSigns, scoped ReadOnlySpan<ulong> denominator,
        scoped ReadOnlySpan<sbyte> denominatorSigns, ReadOnlySpan<ulong> values, ReadOnlySpan<sbyte> signs, Span<ulong> cell)
    {
        Span<ulong> scale = stackalloc ulong[3];
        WideArithmetic.GetMagnitude(rawScale, out scale[2], out scale[1], out scale[0]);
        return MapSquaredValue(scale, valueShift, ref parameter, numerator, numeratorSigns,
            denominator, denominatorSigns, values, signs, cell);
    }

    internal static FiniteAxisValueRoot MapSquaredValue(Signed320 rawScale, int valueShift,
        scoped ref FiniteAxisValueRoot parameter, scoped ReadOnlySpan<ulong> numerator,
        scoped ReadOnlySpan<sbyte> numeratorSigns, scoped ReadOnlySpan<ulong> denominator,
        scoped ReadOnlySpan<sbyte> denominatorSigns, ReadOnlySpan<ulong> values, ReadOnlySpan<sbyte> signs, Span<ulong> cell)
    {
        Span<ulong> scale = stackalloc ulong[5];
        WideArithmetic.GetMagnitude(rawScale, out scale[4], out scale[3], out scale[2], out scale[1], out scale[0]);
        return MapSquaredValue(scale, valueShift, ref parameter, numerator, numeratorSigns,
            denominator, denominatorSigns, values, signs, cell);
    }

    internal static int CompareRootSquared(Signed192 rawScale, int valueShift,
        scoped ref FiniteAxisValueRoot root, ConvexContactCandidate candidate)
    {
        Span<ulong> scale = stackalloc ulong[3];
        WideArithmetic.GetMagnitude(rawScale, out scale[2], out scale[1], out scale[0]);
        return CompareRootSquared(scale, valueShift, ref root, candidate);
    }

    internal static int CompareRootSquared(Signed320 rawScale, int valueShift,
        scoped ref FiniteAxisValueRoot root, ConvexContactCandidate candidate)
    {
        Span<ulong> scale = stackalloc ulong[5];
        WideArithmetic.GetMagnitude(rawScale, out scale[4], out scale[3], out scale[2], out scale[1], out scale[0]);
        return CompareRootSquared(scale, valueShift, ref root, candidate);
    }

    internal static bool GetRoundedDepth(Signed192 rawScale, int valueShift, ref FiniteAxisValueRoot root,
        out Fixed64 depth, out bool clamped, int gapSign = 1, Fixed64 radiusOffset = default)
    {
        Span<ulong> scale = stackalloc ulong[3];
        WideArithmetic.GetMagnitude(rawScale, out scale[2], out scale[1], out scale[0]);
        return GetRoundedDepth(scale, valueShift, ref root, out depth, out clamped, gapSign, radiusOffset);
    }

    internal static bool GetRoundedDepth(Signed320 rawScale, int valueShift, ref FiniteAxisValueRoot root,
        out Fixed64 depth, out bool clamped, int gapSign = 1, Fixed64 radiusOffset = default)
    {
        Span<ulong> scale = stackalloc ulong[5];
        WideArithmetic.GetMagnitude(rawScale, out scale[4], out scale[3], out scale[2], out scale[1], out scale[0]);
        return GetRoundedDepth(scale, valueShift, ref root, out depth, out clamped, gapSign, radiusOffset);
    }

    /// <summary>Compares a positive raw squared gap N/D to (twiceRaw/2)²; D must be positive at the admitted parameter.</summary>
    internal static int CompareSquaredGapToTwiceRaw(scoped ref FiniteAxisValueRoot parameter,
        scoped ReadOnlySpan<ulong> numerator, scoped ReadOnlySpan<sbyte> numeratorSigns,
        scoped ReadOnlySpan<ulong> denominator, scoped ReadOnlySpan<sbyte> denominatorSigns, ulong twiceRaw)
    {
        Span<ulong> endpoint = stackalloc ulong[2];
        Fixed64.Multiply64To128(twiceRaw, twiceRaw, out endpoint[1], out endpoint[0]);
        Span<ulong> scale = stackalloc ulong[1] { 1 };
        return CompareSquaredValueEndpoint(scale, 0, ref parameter,
            numerator, numeratorSigns, denominator, denominatorSigns, endpoint, 2);
    }

    /// <summary>Maps one admitted stationary parameter to its exact squared-gap value root.</summary>
    [MethodImpl(MethodImplOptions.NoInlining)]
    internal static FiniteAxisValueRoot MapSquaredValue(scoped ReadOnlySpan<ulong> scale, int valueShift,
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
                if (CompareSquaredValueEndpoint(scale, valueShift, ref parameter,
                        numerator, numeratorSigns, denominator, denominatorSigns, cell, root.DenominatorShift) == 0)
                    return root;
            }
            else
            {
                cell.CopyTo(upper);
                WideArithmetic.AddWord(upper, 0, 1);
                if (CompareSquaredValueEndpoint(scale, valueShift, ref parameter,
                        numerator, numeratorSigns, denominator, denominatorSigns, upper, root.DenominatorShift) < 0)
                    return root;
            }
        }
        System.Diagnostics.Debug.Assert(roots.Count > 0);
        return FiniteAxisValueRoots.GetRoot(roots, roots.Count - 1, values, signs, cell);
    }

    [MethodImpl(MethodImplOptions.NoInlining)]
    private static int CompareSquaredValueEndpoint(scoped ReadOnlySpan<ulong> scale, int valueShift,
        scoped ref FiniteAxisValueRoot parameter, scoped ReadOnlySpan<ulong> numerator,
        scoped ReadOnlySpan<sbyte> numeratorSigns, scoped ReadOnlySpan<ulong> denominator,
        scoped ReadOnlySpan<sbyte> denominatorSigns, scoped ReadOnlySpan<ulong> endpoint, int shift)
    {
        int sourceWords = numerator.Length / numeratorSigns.Length;
        int words = (shift + valueShift + sourceWords * 64 + scale.Length * 128 + 127) / 64;
        Span<ulong> query = stackalloc ulong[5 * words];
        Span<sbyte> signs = stackalloc sbyte[5];
        Span<ulong> product = stackalloc ulong[words];
        Span<ulong> scaleSquared = stackalloc ulong[2 * scale.Length];
        query.Clear(); signs.Clear();
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
    /// Retains certified cell refinement and its metadata when caller storage permits.
    /// </summary>
    internal static int CompareRootSquared(scoped ReadOnlySpan<ulong> scale, int valueShift,
        scoped ref FiniteAxisValueRoot root, ConvexContactCandidate candidate)
    {
        Span<ulong> squaredScale = stackalloc ulong[2 * scale.Length];
        WideArithmetic.MultiplyMagnitudes(scale, scale, squaredScale);
        int scaleWords = WideArithmetic.GetActiveMagnitudeLength(squaredScale);
        int denominatorWords = WideArithmetic.GetActiveMagnitudeLength(candidate.GapDenominator);
        int linearWords = Math.Max(
            WideArithmetic.GetActiveMagnitudeLength(candidate.GapRational) + scaleWords,
            denominatorWords + (valueShift + 63) / 64);
        int radicalWords = WideArithmetic.GetActiveMagnitudeLength(candidate.GapRadical) + scaleWords;
        int words = Math.Max(ConvexContactCandidate.Words, Math.Max(2 * linearWords + 1,
            2 * radicalWords + WideArithmetic.GetActiveMagnitudeLength(candidate.GapRadicand) + 1));
        Span<ulong> linear = stackalloc ulong[2 * words];
        Span<sbyte> linearSigns = stackalloc sbyte[2];
        // R=D*2^ValueShift*t-A*RawScale². Compare R with B*RawScale²*sqrt(C).
        WideArithmetic.MultiplyMagnitudes(candidate.GapRational, squaredScale, linear[..words]);
        linearSigns[0] = (sbyte)-candidate.GapRationalSign;
        linear[words..].Clear();
        // Query scratch follows active coefficient widths, while a retained
        // affine candidate can have a wider padded field. Copy only its value.
        candidate.GapDenominator[..denominatorWords].CopyTo(linear[words..]);
        linearSigns[1] = 1;
        WideFiniteAxisIntersection.ScaleFiniteAxisPolynomialVariable(linear, 2, valueShift);
        int rationalSign = WideFiniteAxisIntersection.GetSignAtFiniteValueRootAndRefine(ref root, linear, linearSigns);
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
        return rationalSign * WideFiniteAxisIntersection.GetSignAtFiniteValueRootAndRefine(ref root, query, querySigns);
    }

    /// <summary>Compares an exact squared value root with a positive-denominator quadratic ratio.</summary>
    internal static int CompareRootSquared(Signed192 rawScale, int valueShift, scoped ref FiniteAxisValueRoot root,
        scoped ContactQuadratic numerator, scoped ContactQuadratic denominator, scoped ReadOnlySpan<ulong> radical)
    {
        Span<ulong> scale = stackalloc ulong[3], square = stackalloc ulong[6];
        WideArithmetic.GetMagnitude(rawScale, out scale[2], out scale[1], out scale[0]);
        WideArithmetic.MultiplyMagnitudes(scale, scale, square);
        int words = Math.Max(ContactQuadratic.ActiveWords(numerator) + WideArithmetic.GetActiveMagnitudeLength(square),
            ContactQuadratic.ActiveWords(denominator) + (valueShift + 63) / 64) + 1;
        Span<ulong> a = stackalloc ulong[2 * words], b = stackalloc ulong[2 * words];
        Span<sbyte> aSigns = stackalloc sbyte[2], bSigns = stackalloc sbyte[2];
        WriteLinear(numerator.Rational, numerator.Signs[0], denominator.Rational, denominator.Signs[0], a, aSigns, square, words, valueShift);
        int firstSign = WideFiniteAxisIntersection.GetSignAtFiniteValueRootAndRefine(ref root, a, aSigns);
        if (WideArithmetic.GetActiveMagnitudeLength(radical) == 0) return firstSign;
        WriteLinear(numerator.Radical, numerator.Signs[1], denominator.Radical, denominator.Signs[1], b, bSigns, square, words, valueShift);
        int secondSign = WideFiniteAxisIntersection.GetSignAtFiniteValueRootAndRefine(ref root, b, bSigns);
        if (firstSign == 0) return secondSign;
        if (secondSign == 0 || firstSign == secondSign) return firstSign;
        // A(t)+B(t)*sqrt(K) has opposing terms. The original certified
        // value root also signs A(t)^2-K*B(t)^2, of degree only two.
        int squareWords = 2 * words + WideArithmetic.GetActiveMagnitudeLength(radical) + 1;
        Span<ulong> query = stackalloc ulong[3 * squareWords], secondSquare = stackalloc ulong[3 * squareWords];
        Span<sbyte> querySigns = stackalloc sbyte[3], secondSigns = stackalloc sbyte[3];
        Span<ulong> product = stackalloc ulong[squareWords];
        WideFiniteAxisIntersection.MultiplyFiniteAxisPolynomials(a, aSigns, a, aSigns, query, querySigns, product);
        WideFiniteAxisIntersection.MultiplyFiniteAxisPolynomials(b, bSigns, b, bSigns, secondSquare, secondSigns, product);
        for (int index = 0; index < 3; index++)
        {
            WideArithmetic.MultiplyMagnitudes(secondSquare.Slice(index * squareWords, squareWords), radical, product);
            int sign = querySigns[index];
            WideArithmetic.AddShiftedSignedMagnitude(product, -secondSigns[index], 0, query.Slice(index * squareWords, squareWords), ref sign);
            querySigns[index] = (sbyte)sign;
        }
        return firstSign * WideFiniteAxisIntersection.GetSignAtFiniteValueRootAndRefine(ref root, query, querySigns);

        static void WriteLinear(ReadOnlySpan<ulong> n, int nSign, ReadOnlySpan<ulong> d, int dSign, Span<ulong> values, Span<sbyte> signs,
            ReadOnlySpan<ulong> square, int words, int valueShift)
        {
            WideArithmetic.MultiplyMagnitudes(n, square, values[..words]); signs[0] = (sbyte)-nSign;
            values[words..].Clear(); d[..WideArithmetic.GetActiveMagnitudeLength(d)].CopyTo(values[words..]); signs[1] = (sbyte)dSign;
            WideFiniteAxisIntersection.ScaleFiniteAxisPolynomialVariable(values, 2, valueShift);
        }
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
        int activeWords = 0;
        for (int index = 0; index < gradientSigns.Length; index++)
            activeWords = Math.Max(activeWords,
                WideArithmetic.GetActiveMagnitudeLength(gradient.Slice(index * words, words)));
        // Each coefficient sums at most 3*count products; one extra word
        // covers that carry. Generated gradients also retain the existing
        // proof that their squared coefficients fit twice the input stride.
        // Taking the tighter bound avoids padding in every rounding threshold
        // without increasing the worst-case stack; a zero gradient needs one word.
        int squareWords = Math.Min(2 * words, 2 * activeWords + 1);
        Span<ulong> squaredLength = stackalloc ulong[squareCount * squareWords];
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

    /// <summary>Classifies and rounds radiusOffset + gapSign*sqrt(value) as one exact value.</summary>
    internal static bool GetRoundedDepth(scoped ReadOnlySpan<ulong> scale, int valueShift, ref FiniteAxisValueRoot root,
        out Fixed64 depth, out bool clamped, int gapSign = 1, Fixed64 radiusOffset = default)
    {
        depth = default; clamped = false;
        // Isolated squared-value roots are strictly positive. Exact zero-gap
        // features are retained separately by their geometry owner.
        System.Diagnostics.Debug.Assert(gapSign is -1 or 1);
        Span<ulong> squaredScale = stackalloc ulong[2 * scale.Length];
        WideArithmetic.MultiplyMagnitudes(scale, scale, squaredScale);
        if (gapSign < 0 && CompareDepthToTwiceRaw(root, squaredScale, valueShift, 0, gapSign, radiusOffset) < 0)
            return false;
        int maximum = CompareDepthToTwiceRaw(root, squaredScale, valueShift,
            unchecked((ulong)long.MaxValue << 1), gapSign, radiusOffset);
        if (maximum >= 0)
        {
            depth = Fixed64.MaxValue;
            clamped = maximum > 0;
            return true;
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
            if (CompareDepthToTwiceRaw(root, squaredScale, valueShift, midpoint << 1, gapSign, radiusOffset) >= 0)
                lower = midpoint + 1;
            else
                upper = midpoint;
        }
        ulong floor = lower - 1;
        int comparison = CompareDepthToTwiceRaw(root, squaredScale, valueShift, (floor << 1) | 1, gapSign, radiusOffset);
        depth = Fixed64.FromRaw((long)(floor + (comparison > 0 || (comparison == 0 && (floor & 1) != 0) ? 1UL : 0)));
        clamped = false;
        return true;
    }

    private static int CompareDepthToTwiceRaw(FiniteAxisValueRoot root,
        ReadOnlySpan<ulong> squaredScale, int valueShift, ulong twiceRaw, int gapSign, Fixed64 radiusOffset)
    {
        Signed192 difference = WideArithmetic.SubtractSigned192(new Signed192(0, 0, twiceRaw),
            WideArithmetic.AddSigned192(Signed192.Raw(radiusOffset), Signed192.Raw(radiusOffset)));
        if (gapSign != difference.Sign)
            return gapSign.CompareTo(difference.Sign);
        // Both the nonnegative output threshold and doubled radius fit 64
        // unsigned bits; their signed difference has the same magnitude bound.
        WideArithmetic.GetMagnitude(difference, out _, out _, out ulong magnitude);
        Fixed64.Multiply64To128(magnitude, magnitude, out ulong high, out ulong low);
        Span<ulong> squaredRaw = stackalloc ulong[2] { low, high };
        Span<ulong> threshold = stackalloc ulong[squaredScale.Length + 2];
        WideArithmetic.MultiplyMagnitudes(squaredScale, squaredRaw, threshold);
        return gapSign * WideFiniteAxisIntersection.CompareFiniteValueRootToDyadic(root, threshold, valueShift + 2);
    }

    [MethodImpl(MethodImplOptions.NoInlining)]
    internal static void BuildSquaredLength(ReadOnlySpan<ulong> gradient, ReadOnlySpan<sbyte> signs,
        Span<ulong> squaredLength, Span<sbyte> squaredLengthSigns)
    {
        int count = signs.Length / 3;
        int words = gradient.Length / signs.Length;
        Span<ulong> product = stackalloc ulong[squaredLength.Length / squaredLengthSigns.Length];
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
        int removedShift = PrepareNormalThreshold(root, component, componentSigns,
            squaredLength, squaredLengthSigns, (ulong)scale.m_rawValue, query, querySigns,
            out ulong lowerFloor, out ulong upperFloor);
        // The lower floor is already proven to pass its integer threshold.
        // Search for the first failing integer strictly above that floor.
        ulong lower = lowerFloor + 1;
        ulong upper = upperFloor + 1;
        ulong previousTwiceRaw = 0;
        while (lower < upper)
        {
            ulong midpoint = lower + ((upper - lower) >> 1);
            BuildNormalThreshold(squaredLength, squaredLengthSigns,
                midpoint << 1, ref previousTwiceRaw, ref removedShift, query, querySigns);
            if (WideFiniteAxisIntersection.GetSignAtFiniteValueRootAndRefine(ref root, query, querySigns) >= 0)
                lower = midpoint + 1;
            else
                upper = midpoint;
        }
        ulong floor = lower - 1;
        BuildNormalThreshold(squaredLength, squaredLengthSigns,
            (floor << 1) | 1, ref previousTwiceRaw, ref removedShift, query, querySigns);
        int comparison = WideFiniteAxisIntersection.GetSignAtFiniteValueRootAndRefine(ref root, query, querySigns);
        ulong rounded = floor + (comparison > 0 || (comparison == 0 && (floor & 1) != 0) ? 1UL : 0);
        return Fixed64.FromRaw(sign * (long)rounded);
    }

    [MethodImpl(MethodImplOptions.NoInlining)]
    private static int PrepareNormalThreshold(FiniteAxisValueRoot root,
        ReadOnlySpan<ulong> component, ReadOnlySpan<sbyte> componentSigns,
        ReadOnlySpan<ulong> squaredLength, ReadOnlySpan<sbyte> squaredLengthSigns, ulong scale,
        Span<ulong> query, Span<sbyte> querySigns, out ulong lower, out ulong upper)
    {
        int queryWords = query.Length / querySigns.Length;
        Span<ulong> product = stackalloc ulong[queryWords];
        WideFiniteAxisIntersection.MultiplyFiniteAxisPolynomials(component, componentSigns,
            component, componentSigns, query, querySigns, product);
        GetNormalFloorBounds(root, query, querySigns, squaredLength, squaredLengthSigns, scale, out lower, out upper);
        // Keep the same initialized threshold state as the original first
        // search iteration. Bounds consume the unscaled square first, so the
        // common normalization does not lose 64..126 useful precision bits.
        if (scale == (ulong)Fixed64.One.m_rawValue) return 66;
        Span<ulong> factor = stackalloc ulong[2];
        Fixed64.Multiply64To128(scale << 1, scale << 1, out factor[1], out factor[0]);
        for (int index = 0; index < querySigns.Length; index++)
        {
            Span<ulong> target = query.Slice(index * queryWords, queryWords);
            WideArithmetic.MultiplyMagnitudes(target, factor, product);
            product.CopyTo(target);
        }
        return 0;
    }

    private static void GetNormalFloorBounds(FiniteAxisValueRoot root,
        ReadOnlySpan<ulong> componentSquared, ReadOnlySpan<sbyte> componentSigns,
        ReadOnlySpan<ulong> squaredLength, ReadOnlySpan<sbyte> lengthSigns, ulong scale,
        out ulong lower, out ulong upper)
    {
        lower = 0; upper = scale;
        int count = lengthSigns.Length;
        int margin = 2 * (64 - Fixed64.CountLeadingZeroes((ulong)(count - 1)));
        // 59 is the largest fixed precision for which 17 normalized
        // coefficients plus the error allowance fit unsigned 64-bit endpoints.
        int precision = root.IsRational ? 59 : Math.Min(59, root.DenominatorShift - margin);
        if (precision <= 0) return;
        int bits = Math.Max(WideFiniteAxisIntersection.GetFiniteRootCoefficientBits(componentSquared, count),
            WideFiniteAxisIntersection.GetFiniteRootCoefficientBits(squaredLength, count));
        WideFiniteAxisIntersection.GetFiniteValueRootNonnegativeBounds(root, squaredLength, lengthSigns,
            precision, bits, out ulong lengthMinimum, out ulong lengthMaximum);
        if (lengthMinimum == 0) return;
        WideFiniteAxisIntersection.GetFiniteValueRootNonnegativeBounds(root, componentSquared, componentSigns,
            precision, bits, out ulong componentMinimum, out ulong componentMaximum);
        // C² and L share one normalization. Scale only the bounded numerator:
        // <=64-bit endpoints times <=126-bit scale² fit Signed320. Existing
        // clipped ratio/square-root division gives outward raw floor bounds;
        // exact integer and half-raw threshold signs remain the final authority.
        Fixed64.Multiply64To128(scale, scale, out ulong high, out ulong low);
        Signed192 factor = new(0, high, low);
        Signed320 minimum = WideArithmetic.MultiplySigned192(new Signed192(0, 0, componentMinimum), factor);
        Signed320 maximum = WideArithmetic.MultiplySigned192(new Signed192(0, 0, componentMaximum), factor);
        Span<ulong> bounds = stackalloc ulong[20];
        bounds.Clear();
        WideArithmetic.GetMagnitude(minimum, out bounds[4], out bounds[3], out bounds[2], out bounds[1], out bounds[0]);
        WideArithmetic.GetMagnitude(maximum, out bounds[9], out bounds[8], out bounds[7], out bounds[6], out bounds[5]);
        bounds[10] = lengthMinimum; bounds[15] = lengthMaximum;
        lower = WideArithmetic.GetRatioFloorSquareRoot(bounds[..5], bounds[15..], scale);
        upper = WideArithmetic.GetRatioFloorSquareRoot(bounds.Slice(5, 5), bounds.Slice(10, 5), scale);
    }

    [MethodImpl(MethodImplOptions.NoInlining)]
    private static void BuildNormalThreshold(
        ReadOnlySpan<ulong> squaredLength, ReadOnlySpan<sbyte> squaredLengthSigns,
        ulong twiceRaw, ref ulong previousTwiceRaw, ref int removedShift,
        Span<ulong> query, Span<sbyte> querySigns)
    {
        int squareCount = squaredLengthSigns.Length;
        int squareWords = squaredLength.Length / squareCount;
        int queryWords = query.Length / querySigns.Length;
        Span<ulong> product = stackalloc ulong[queryWords];
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
