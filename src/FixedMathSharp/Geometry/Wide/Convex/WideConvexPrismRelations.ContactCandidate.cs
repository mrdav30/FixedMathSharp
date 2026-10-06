//=======================================================================
// WideConvexPrismRelations.ContactCandidate.cs
//=======================================================================
// MIT License, Copyright (c) 2024-present David Oravsky (mrdav30)
// See LICENSE file in the project root for full license information.
//=======================================================================

using System;

namespace FixedMathSharp.Geometry;

/// <content>
/// Exact comparison and final rounding of analytic convex contact
/// candidates. Geometry and candidate-buffer ownership remain with the caller.
/// </content>
internal static partial class WideConvexPrismRelations
{
    // No fixed-width operation below truncates an input to fit a scratch span.
    // Multiplication reserves the sum of active operand lengths; a signed sum
    // reserves an additional word. For forty-word candidate fields, cross-
    // denominated A/B have at most 81/80 words. Their radical radicands have
    // at most 200 words, plus one guard word for CompareRadicalPairs' additions
    // and shifts. Its three-term reduction fits twice that scratch width.
    // Normal-square coefficients use at most 122 words (three 40-word factors
    // and two carry words). Integer midpoint factors add at most two words.
    // Actual active lengths, not those maxima, size every arithmetic scratch.
    internal static int CompareConvexContactCandidates(
        ConvexContactCandidate left,
        ConvexContactCandidate right)
    {
        if (left.GapSign != right.GapSign)
            return left.GapSign.CompareTo(right.GapSign);
        if (left.GapSign == 0)
            return 0;

        int words = Math.Max(
            ConvexContactCandidateProductWords(
                left.GapRational, right.GapDenominator),
            ConvexContactCandidateProductWords(
                right.GapRational, left.GapDenominator)) + 1;
        Span<ulong> first = stackalloc ulong[words];
        Span<ulong> second = stackalloc ulong[words];
        Span<ulong> rational = stackalloc ulong[words];
        WideArithmetic.MultiplyMagnitudes(
            left.GapRational, right.GapDenominator, first);
        WideArithmetic.MultiplyMagnitudes(
            right.GapRational, left.GapDenominator, second);
        CombineWideSignedMagnitudes(
            first, left.GapRationalSign,
            second, -right.GapRationalSign,
            rational, out int rationalSign);

        Span<ulong> leftRadical = stackalloc ulong[
            ConvexContactCandidateProductWords(
                left.GapRadical, right.GapDenominator)];
        Span<ulong> rightRadical = stackalloc ulong[
            ConvexContactCandidateProductWords(
                right.GapRadical, left.GapDenominator)];
        WideArithmetic.MultiplyMagnitudes(
            left.GapRadical, right.GapDenominator, leftRadical);
        WideArithmetic.MultiplyMagnitudes(
            right.GapRadical, left.GapDenominator, rightRadical);
        return left.GapSign * GetConvexContactCandidateThreeTermSign(
            rational, rationalSign,
            leftRadical, left.GapRadicalSign, left.GapRadicand,
            rightRadical, -right.GapRadicalSign, right.GapRadicand);
    }

    internal static int CompareConvexContactCandidateDepthToTwiceRaw(
        ConvexContactCandidate candidate,
        Fixed64 radiusOffset,
        Signed192 twiceRaw)
    {
        Signed192 target = WideArithmetic.SubtractSigned192(
            twiceRaw,
            new Signed192(0UL, 0UL,
                unchecked((ulong)radiusOffset.m_rawValue << 1)));
        if (candidate.GapSign != target.Sign)
            return candidate.GapSign.CompareTo(target.Sign);
        if (candidate.GapSign == 0)
            return 0;

        Span<ulong> targetMagnitude = stackalloc ulong[3];
        WideArithmetic.GetMagnitude(target,
            out targetMagnitude[2], out targetMagnitude[1],
            out targetMagnitude[0]);
        Span<ulong> targetSquare = stackalloc ulong[
            ConvexContactCandidateProductWords(
                targetMagnitude, targetMagnitude)];
        WideArithmetic.MultiplyMagnitudes(
            targetMagnitude, targetMagnitude, targetSquare);
        int words = Math.Max(
            ConvexContactCandidateLength(candidate.GapRational) + 1,
            ConvexContactCandidateProductWords(
                targetSquare, candidate.GapDenominator)) + 1;
        Span<ulong> rational = stackalloc ulong[words];
        Span<ulong> threshold = stackalloc ulong[words];
        Span<ulong> difference = stackalloc ulong[words];
        rational.Clear();
        candidate.GapRational.Slice(0,
            ConvexContactCandidateLength(candidate.GapRational))
            .CopyTo(rational);
        ShiftLeft(rational, 2);
        WideArithmetic.MultiplyMagnitudes(
            targetSquare, candidate.GapDenominator, threshold);
        CombineWideSignedMagnitudes(
            rational, candidate.GapRationalSign,
            threshold, -1, difference, out int differenceSign);
        Span<ulong> radical = stackalloc ulong[
            ConvexContactCandidateLength(candidate.GapRadical) + 1];
        radical.Clear();
        candidate.GapRadical.Slice(0,
            ConvexContactCandidateLength(candidate.GapRadical))
            .CopyTo(radical);
        ShiftLeft(radical, 2);
        return candidate.GapSign * GetConvexContactCandidateQuadraticSign(
            difference, differenceSign,
            radical, candidate.GapRadicalSign, candidate.GapRadicand);
    }

    /// <summary>
    /// Rounds an admitted nonnegative depth, including the supplied radius offset,
    /// once to nearest-even; clamping describes the exact value, not rounding.
    /// </summary>
    internal static void GetRoundedConvexContactCandidateDepth(
        ConvexContactCandidate candidate,
        Fixed64 radiusOffset,
        out Fixed64 rounded,
        out bool isClamped)
    {
        if (candidate.GapSign == 0)
        {
            rounded = radiusOffset;
            isClamped = false;
            return;
        }

        ulong radiusRaw = unchecked((ulong)radiusOffset.m_rawValue);
        GetConvexContactCandidateMagnitudeFloorBounds(candidate,
            candidate.GapSign < 0 ? radiusRaw : (ulong)long.MaxValue,
            out ulong lowerMagnitude, out ulong upperMagnitude);
        ulong low;
        ulong high;
        if (candidate.GapSign < 0)
        {
            // Magnitude bounds describe floors. Subtraction can cross one
            // additional raw unit when the magnitude is nonintegral; contact
            // admission proves its exact value is at most the radius.
            low = upperMagnitude >= radiusRaw ? 0 : radiusRaw - upperMagnitude - 1;
            high = radiusRaw - lowerMagnitude + 1;
        }
        else
        {
            // Clipped magnitude floors also cover conceptual overflow. Search
            // includes MaxValue itself; its exact comparison below still owns
            // clamping, even when both enclosure floors already equal the cap.
            low = Math.Min((ulong)long.MaxValue, radiusRaw + lowerMagnitude);
            high = Math.Min(1UL << 63, radiusRaw + upperMagnitude + 1);
        }
        while (low < high)
        {
            ulong midpoint = low + ((high - low) >> 1);
            int comparison = CompareConvexContactCandidateDepthToTwiceRaw(
                candidate, radiusOffset,
                new Signed192(0UL, 0UL, midpoint << 1));
            if (comparison >= 0)
                low = midpoint + 1UL;
            else
                high = midpoint;
        }

        ulong floor = low - 1UL;
        if (floor == unchecked((ulong)long.MaxValue))
        {
            rounded = Fixed64.MaxValue;
            isClamped = CompareConvexContactCandidateDepthToTwiceRaw(
                candidate, radiusOffset,
                new Signed192(0UL, 0UL, floor << 1)) > 0;
            return;
        }

        int midpointComparison =
            CompareConvexContactCandidateDepthToTwiceRaw(
                candidate, radiusOffset,
                new Signed192(0UL, 0UL, (floor << 1) | 1UL));
        rounded = Fixed64.FromRaw((long)(floor + GetNearestEvenIncrement(
            midpointComparison, floor)));
        isClamped = false;
    }

    internal static Vector3d GetConvexContactCandidateNormal(
        ConvexContactCandidate candidate) => GetConvexContactCandidateScaledNormal(candidate, Fixed64.One);

    internal static Vector3d GetConvexContactCandidateScaledNormal(
        ConvexContactCandidate candidate, Fixed64 scale)
    {
        if (scale == Fixed64.Zero)
            return Vector3d.Zero;
        int xSign = GetConvexContactCandidateQuadraticSign(candidate.NormalRational(0), candidate.Signs[0],
            candidate.NormalRadical(0), candidate.Signs[3], candidate.NormalRadicand);
        int ySign = GetConvexContactCandidateQuadraticSign(candidate.NormalRational(1), candidate.Signs[1],
            candidate.NormalRadical(1), candidate.Signs[4], candidate.NormalRadicand);
        int zSign = GetConvexContactCandidateQuadraticSign(candidate.NormalRational(2), candidate.Signs[2],
            candidate.NormalRadical(2), candidate.Signs[5], candidate.NormalRadicand);
        if ((xSign == 0 ? 0 : 1) + (ySign == 0 ? 0 : 1) + (zSign == 0 ? 0 : 1) == 1)
            return new Vector3d(xSign * scale, ySign * scale, zSign * scale);

        int rationalWords = 0;
        int radicalWords = 0;
        for (int component = 0; component < 3; component++)
        {
            rationalWords = Math.Max(rationalWords,
                ConvexContactCandidateLength(
                    candidate.NormalRational(component)));
            radicalWords = Math.Max(radicalWords,
                ConvexContactCandidateLength(
                    candidate.NormalRadical(component)));
        }

        int words = Math.Max(
            Math.Max(rationalWords * 2,
                radicalWords * 2 + ConvexContactCandidateLength(
                    candidate.NormalRadicand)),
            rationalWords + radicalWords + 1) + 2;
        // Slots 0..2 and 3..5 hold each component square's rational/radical
        // coefficient; slots 6 and 7 hold their sums (the normal norm square).
        Span<ulong> squares = stackalloc ulong[8 * words];
        Span<int> signs = stackalloc int[4];
        Span<ulong> sum = stackalloc ulong[words];
        squares.Clear();
        signs.Clear();
        Span<ulong> normRational = squares.Slice(6 * words, words);
        Span<ulong> normRadical = squares.Slice(7 * words, words);
        for (int component = 0; component < 3; component++)
        {
            Span<ulong> rational = squares.Slice(component * words, words);
            Span<ulong> radical = squares.Slice((component + 3) * words, words);
            BuildConvexContactCandidateNormalSquare(
                candidate, component, rational, radical,
                out signs[component]);
            WideArithmetic.AddMagnitudeInto(rational, normRational);
            CombineWideSignedMagnitudes(
                normRadical, signs[3], radical, signs[component],
                sum, out signs[3]);
            sum.CopyTo(normRadical);
        }

        WideArithmetic.GetMagnitudeSquareRootBounds(candidate.NormalRadicand,
            out Signed192 lowerRoot, out Signed192 upperRoot, out int rootShift);
        Span<ulong> roots = stackalloc ulong[4]
            { lowerRoot.Low, lowerRoot.Middle, upperRoot.Low, upperRoot.Middle };
        // A coefficient times the <=97-bit prefix root, shifted by k, needs
        // at most coefficientWords+2+ceil(k/64)+1 words. Use the scaled
        // component width too, so numerator and denominator bounds share one
        // padded division width. Forty-word fields give at most 147 words.
        int boundWords = words + 6 + ((rootShift + 63) >> 6);
        Span<ulong> denominatorBounds = stackalloc ulong[2 * boundWords];
        GetConvexContactCandidateQuadraticBounds(normRational, 1,
            normRadical, signs[3], roots, rootShift,
            denominatorBounds[..boundWords], denominatorBounds[boundWords..],
            out int denominatorSign);
        if (denominatorSign > 0)
        {
            // scaledSquares=(2S)^2*n_i^2, so its matching denominator is
            // 4*|n|^2. A nonpositive lower enclosure is uncertainty, not a
            // zero norm: retain the existing full search in that case.
            ShiftLeft(denominatorBounds[..boundWords], 2);
            ShiftLeft(denominatorBounds[boundWords..], 2);
        }
        ReadOnlySpan<ulong> positiveDenominatorBounds = denominatorSign > 0
            ? denominatorBounds : ReadOnlySpan<ulong>.Empty;
        return new Vector3d(
            GetRoundedConvexContactCandidateNormalComponent(
                candidate, 0, squares, signs, words, (ulong)scale.m_rawValue,
                roots, rootShift, positiveDenominatorBounds),
            GetRoundedConvexContactCandidateNormalComponent(
                candidate, 1, squares, signs, words, (ulong)scale.m_rawValue,
                roots, rootShift, positiveDenominatorBounds),
            GetRoundedConvexContactCandidateNormalComponent(
                candidate, 2, squares, signs, words, (ulong)scale.m_rawValue,
                roots, rootShift, positiveDenominatorBounds));
    }

    private static void BuildConvexContactCandidateNormalSquare(
        ConvexContactCandidate candidate,
        int component,
        Span<ulong> rational,
        Span<ulong> radical,
        out int radicalSign)
    {
        ReadOnlySpan<ulong> a = candidate.NormalRational(component);
        ReadOnlySpan<ulong> b = candidate.NormalRadical(component);
        Span<ulong> aSquared = stackalloc ulong[rational.Length];
        Span<ulong> bSquared = stackalloc ulong[rational.Length];
        Span<ulong> bSquaredK = stackalloc ulong[rational.Length];
        WideArithmetic.MultiplyMagnitudes(a, a, aSquared);
        WideArithmetic.MultiplyMagnitudes(b, b, bSquared);
        WideArithmetic.MultiplyMagnitudes(
            bSquared, candidate.NormalRadicand, bSquaredK);
        WideArithmetic.AddEqualMagnitudes(aSquared, bSquaredK, rational);
        WideArithmetic.MultiplyMagnitudes(a, b, radical);
        ShiftLeft(radical, 1);
        radicalSign = IsZero(radical) ? 0
            : candidate.Signs[component] * candidate.Signs[component + 3];
    }

    private static Fixed64 GetRoundedConvexContactCandidateNormalComponent(
        ConvexContactCandidate candidate,
        int component,
        ReadOnlySpan<ulong> squares,
        ReadOnlySpan<int> signs,
        int words,
        ulong scale,
        ReadOnlySpan<ulong> roots,
        int rootShift,
        ReadOnlySpan<ulong> denominatorBounds)
    {
        int sign = GetConvexContactCandidateQuadraticSign(
            candidate.NormalRational(component), candidate.Signs[component],
            candidate.NormalRadical(component), candidate.Signs[component + 3],
            candidate.NormalRadicand);
        if (sign == 0)
            return Fixed64.Zero;

        // These two products do not depend on the searched threshold. Keep
        // them in this returning component frame, including the final tie
        // comparison; do not enlarge or mutate the borrowed square slots.
        int productWords = words + 3;
        Span<ulong> scaledSquares = stackalloc ulong[2 * productWords];
        Span<ulong> scaleSquared = stackalloc ulong[2];
        Fixed64.Multiply64To128(scale << 1, scale << 1,
            out scaleSquared[1], out scaleSquared[0]);
        WideArithmetic.MultiplyMagnitudes(
            squares.Slice(component * words, words), scaleSquared, scaledSquares[..productWords]);
        WideArithmetic.MultiplyMagnitudes(
            squares.Slice((component + 3) * words, words), scaleSquared, scaledSquares[productWords..]);

        ulong low = 0UL;
        // Exact one-component normals already returned above. Every remaining
        // nonzero component is strictly below the norm, so its raw floor is
        // below scale; nearest-even rounding may still produce scale.
        ulong high = scale;
        if (!denominatorBounds.IsEmpty)
        {
            GetConvexContactCandidateNormalFloorBounds(scaledSquares,
                signs[component], roots, rootShift, denominatorBounds, scale,
                out low, out ulong upperFloor);
            high = Math.Min(high, upperFloor + 1);
        }
        while (low < high)
        {
            ulong midpoint = low + ((high - low) >> 1);
            int comparison =
                CompareConvexContactCandidateNormalComponentToTwiceRaw(
                    squares, signs, words, component,
                    candidate.NormalRadicand, midpoint << 1, scaledSquares);
            if (comparison >= 0)
                low = midpoint + 1UL;
            else
                high = midpoint;
        }

        ulong floor = low - 1UL;
        int midpointComparison =
            CompareConvexContactCandidateNormalComponentToTwiceRaw(
                squares, signs, words, component,
                candidate.NormalRadicand, (floor << 1) | 1UL, scaledSquares);
        floor += GetNearestEvenIncrement(midpointComparison, floor);
        return Fixed64.FromRaw(sign * (long)floor);
    }

    private static void GetConvexContactCandidateMagnitudeFloorBounds(
        ConvexContactCandidate candidate, ulong cap,
        out ulong lower, out ulong upper)
    {
        WideArithmetic.GetMagnitudeSquareRootBounds(candidate.GapRadicand,
            out Signed192 lowerRoot, out Signed192 upperRoot, out int shift);
        Span<ulong> roots = stackalloc ulong[4]
            { lowerRoot.Low, lowerRoot.Middle, upperRoot.Low, upperRoot.Middle };
        int words = Math.Max(2, Math.Max(
            Math.Max(ConvexContactCandidateLength(candidate.GapRational),
                ConvexContactCandidateLength(candidate.GapDenominator)),
            ConvexContactCandidateLength(candidate.GapRadical) + 2 + ((shift + 63) >> 6)) + 1);
        Span<ulong> bounds = stackalloc ulong[3 * words];
        Span<ulong> minimum = bounds[..words], maximum = bounds.Slice(words, words);
        Span<ulong> denominator = bounds[(2 * words)..];
        GetConvexContactCandidateQuadraticBounds(candidate.GapRational,
            candidate.GapRationalSign, candidate.GapRadical,
            candidate.GapRadicalSign, roots, shift, minimum, maximum,
            out int minimumSign);
        denominator.Clear();
        candidate.GapDenominator[..ConvexContactCandidateLength(candidate.GapDenominator)].CopyTo(denominator);
        // The true squared magnitude is nonnegative. A negative interval
        // minimum is cancellation in the enclosure, so its lower bound is 0.
        lower = minimumSign > 0
            ? WideArithmetic.GetRatioFloorSquareRoot(minimum, denominator, cap) : 0;
        upper = WideArithmetic.GetRatioFloorSquareRoot(maximum, denominator, cap);
    }

    private static void GetConvexContactCandidateNormalFloorBounds(
        ReadOnlySpan<ulong> scaledSquares, int radicalSign,
        ReadOnlySpan<ulong> roots, int shift,
        ReadOnlySpan<ulong> denominatorBounds, ulong cap,
        out ulong lower, out ulong upper)
    {
        // This frame returns before exact searches. Keep its numerator bounds
        // out of the live stack of the wide quadratic sign comparisons.
        int words = denominatorBounds.Length / 2;
        int coefficientWords = scaledSquares.Length / 2;
        Span<ulong> minimum = stackalloc ulong[words];
        Span<ulong> maximum = stackalloc ulong[words];
        GetConvexContactCandidateQuadraticBounds(scaledSquares[..coefficientWords], 1,
            scaledSquares[coefficientWords..], radicalSign, roots, shift,
            minimum, maximum, out int minimumSign);
        lower = minimumSign > 0
            ? WideArithmetic.GetRatioFloorSquareRoot(minimum, denominatorBounds[words..], cap) : 0;
        upper = WideArithmetic.GetRatioFloorSquareRoot(maximum, denominatorBounds[..words], cap);
    }

    private static void GetConvexContactCandidateQuadraticBounds(
        ReadOnlySpan<ulong> rational, int rationalSign,
        ReadOnlySpan<ulong> radical, int radicalSign,
        ReadOnlySpan<ulong> roots, int shift,
        Span<ulong> minimum, Span<ulong> maximum, out int minimumSign)
    {
        minimum.Clear();
        rational[..ConvexContactCandidateLength(rational)].CopyTo(minimum);
        minimum.CopyTo(maximum);
        minimumSign = IsZero(rational) ? 0 : rationalSign;
        int maximumSign = minimumSign;
        Span<ulong> product = stackalloc ulong[ConvexContactCandidateLength(radical) + 2];
        // Negative coefficients reverse the root endpoints. Accumulate the
        // factored root directly, including signed cancellation and the upper
        // carry at 2^96, without constructing a much wider shifted root copy.
        WideArithmetic.MultiplyMagnitudes(radical,
            radicalSign < 0 ? roots[2..] : roots[..2], product);
        WideArithmetic.AddShiftedSignedMagnitude(product, radicalSign, shift, minimum, ref minimumSign);
        WideArithmetic.MultiplyMagnitudes(radical,
            radicalSign < 0 ? roots[..2] : roots[2..], product);
        WideArithmetic.AddShiftedSignedMagnitude(product, radicalSign, shift, maximum, ref maximumSign);
        if (IsZero(minimum))
            minimumSign = 0;
    }

    private static int CompareConvexContactCandidateNormalComponentToTwiceRaw(
        ReadOnlySpan<ulong> squares,
        ReadOnlySpan<int> signs,
        int words,
        int component,
        ReadOnlySpan<ulong> radicand,
        ulong twiceRaw,
        ReadOnlySpan<ulong> scaledSquares)
    {
        // (2 S n_i)^2 - twiceRaw^2 |n|^2 has the sign of the unsquared
        // comparison, since both magnitudes and the norm are nonnegative.
        Span<ulong> thresholdSquared = stackalloc ulong[2];
        Fixed64.Multiply64To128(twiceRaw, twiceRaw,
            out thresholdSquared[1], out thresholdSquared[0]);
        int productWords = words + 3;
        Span<ulong> product = stackalloc ulong[productWords];
        Span<ulong> rational = stackalloc ulong[productWords];
        Span<ulong> radical = stackalloc ulong[productWords];
        WideArithmetic.MultiplyMagnitudes(
            squares.Slice(6 * words, words), thresholdSquared, product);
        CombineWideSignedMagnitudes(scaledSquares[..productWords], 1, product, -1,
            rational, out int rationalSign);
        WideArithmetic.MultiplyMagnitudes(
            squares.Slice(7 * words, words), thresholdSquared, product);
        CombineWideSignedMagnitudes(
            scaledSquares[productWords..], signs[component], product, -signs[3],
            radical, out int radicalSign);
        return GetConvexContactCandidateQuadraticSign(
            rational, rationalSign, radical, radicalSign, radicand);
    }

    internal static int GetConvexContactCandidateQuadraticSign(
        ReadOnlySpan<ulong> rational,
        int rationalSign,
        ReadOnlySpan<ulong> radical,
        int radicalSign,
        ReadOnlySpan<ulong> radicand)
    {
        if (IsZero(rational))
            rationalSign = 0;
        if (IsZero(radical) || IsZero(radicand))
            radicalSign = 0;
        if (rationalSign == 0)
            return radicalSign;
        if (radicalSign == 0 || radicalSign == rationalSign)
            return rationalSign;

        int words = Math.Max(
            ConvexContactCandidateLength(rational) * 2,
            ConvexContactCandidateLength(radical) * 2
                + ConvexContactCandidateLength(radicand));
        Span<ulong> rationalSquared = stackalloc ulong[words];
        Span<ulong> radicalSquared = stackalloc ulong[words];
        Span<ulong> radicalSquaredK = stackalloc ulong[words];
        WideArithmetic.MultiplyMagnitudes(rational, rational, rationalSquared);
        WideArithmetic.MultiplyMagnitudes(radical, radical, radicalSquared);
        WideArithmetic.MultiplyMagnitudes(
            radicalSquared, radicand, radicalSquaredK);
        return rationalSign * WideArithmetic.CompareMagnitudeEqualLength(
            rationalSquared, radicalSquaredK);
    }

    private static int GetConvexContactCandidateThreeTermSign(
        ReadOnlySpan<ulong> rational,
        int rationalSign,
        ReadOnlySpan<ulong> firstCoefficient,
        int firstSign,
        ReadOnlySpan<ulong> firstRadicand,
        ReadOnlySpan<ulong> secondCoefficient,
        int secondSign,
        ReadOnlySpan<ulong> secondRadicand)
    {
        if (IsZero(rational))
            rationalSign = 0;
        if (IsZero(firstCoefficient) || IsZero(firstRadicand))
            firstSign = 0;
        if (IsZero(secondCoefficient) || IsZero(secondRadicand))
            secondSign = 0;
        if (firstSign == 0)
            return GetConvexContactCandidateQuadraticSign(
                rational, rationalSign,
                secondCoefficient, secondSign, secondRadicand);
        if (secondSign == 0)
            return GetConvexContactCandidateQuadraticSign(
                rational, rationalSign,
                firstCoefficient, firstSign, firstRadicand);
        if (firstSign == secondSign && rationalSign != -firstSign)
            return firstSign;

        int words = Math.Max(
            ConvexContactCandidateLength(rational) * 2,
            Math.Max(
                ConvexContactCandidateLength(firstCoefficient) * 2
                    + ConvexContactCandidateLength(firstRadicand),
                ConvexContactCandidateLength(secondCoefficient) * 2
                    + ConvexContactCandidateLength(secondRadicand))) + 1;
        Span<ulong> radicands = stackalloc ulong[4 * words];
        Span<ulong> square = stackalloc ulong[words];
        Span<ulong> product = stackalloc ulong[words];
        radicands.Clear();
        WideArithmetic.MultiplyMagnitudes(rational, rational, square);
        CopyConvexContactCandidateSignedRadicand(
            square, rationalSign, radicands, words);
        WideArithmetic.MultiplyMagnitudes(
            firstCoefficient, firstCoefficient, square);
        WideArithmetic.MultiplyMagnitudes(square, firstRadicand, product);
        CopyConvexContactCandidateSignedRadicand(
            product, firstSign, radicands, words);
        WideArithmetic.MultiplyMagnitudes(
            secondCoefficient, secondCoefficient, square);
        WideArithmetic.MultiplyMagnitudes(square, secondRadicand, product);
        CopyConvexContactCandidateSignedRadicand(
            product, secondSign, radicands, words);
        return CompareRadicalPairs(
            radicands.Slice(0, words), radicands.Slice(words, words),
            radicands.Slice(2 * words, words), radicands.Slice(3 * words, words));
    }

    private static void CopyConvexContactCandidateSignedRadicand(
        ReadOnlySpan<ulong> value,
        int sign,
        Span<ulong> radicands,
        int words)
    {
        if (sign == 0)
            return;
        int offset = sign > 0 ? 0 : 2 * words;
        CopyPositiveRadicand(value,
            radicands.Slice(offset, words),
            radicands.Slice(offset + words, words));
    }

    private static int ConvexContactCandidateLength(
        ReadOnlySpan<ulong> value) =>
        WideArithmetic.GetActiveMagnitudeLength(value);

    private static int ConvexContactCandidateProductWords(
        ReadOnlySpan<ulong> left,
        ReadOnlySpan<ulong> right) => Math.Max(1,
            ConvexContactCandidateLength(left)
                + ConvexContactCandidateLength(right));
}
