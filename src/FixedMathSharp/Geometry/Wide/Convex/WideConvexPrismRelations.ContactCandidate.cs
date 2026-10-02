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

        ulong low = 0UL;
        ulong high = 1UL << 63;
        ulong radiusRaw = unchecked((ulong)radiusOffset.m_rawValue);
        if (candidate.GapSign < 0)
        {
            high = radiusRaw + 1UL;
        }
        else if (candidate.GapRadicalSign <= 0 || IsZero(candidate.GapRadicand))
        {
            // Here 0 < A+B sqrt(C) <= A. If A and D have a/d bits,
            // sqrt(A/D) < 2^ceil((a-d+1)/2). This is an exact upper
            // bound only; all floor, midpoint and clamp decisions below
            // still compare the complete candidate. Radius plus the bound
            // fits ulong even when it exceeds the Fixed64 search domain.
            int exponent = Math.Min(63, Math.Max(0,
                (WideArithmetic.GetMagnitudeBitLength(candidate.GapRational)
                    - WideArithmetic.GetMagnitudeBitLength(candidate.GapDenominator) + 2) / 2));
            high = Math.Min(high, radiusRaw + (1UL << exponent));
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

        return new Vector3d(
            GetRoundedConvexContactCandidateNormalComponent(
                candidate, 0, squares, signs, words, (ulong)scale.m_rawValue),
            GetRoundedConvexContactCandidateNormalComponent(
                candidate, 1, squares, signs, words, (ulong)scale.m_rawValue),
            GetRoundedConvexContactCandidateNormalComponent(
                candidate, 2, squares, signs, words, (ulong)scale.m_rawValue));
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
        ulong scale)
    {
        int sign = GetConvexContactCandidateQuadraticSign(
            candidate.NormalRational(component), candidate.Signs[component],
            candidate.NormalRadical(component), candidate.Signs[component + 3],
            candidate.NormalRadicand);
        if (sign == 0)
            return Fixed64.Zero;

        ulong low = 0UL;
        // Exact one-component normals already returned above. Every remaining
        // nonzero component is strictly below the norm, so its raw floor is
        // below scale; nearest-even rounding may still produce scale.
        ulong high = scale;
        while (low < high)
        {
            ulong midpoint = low + ((high - low) >> 1);
            int comparison =
                CompareConvexContactCandidateNormalComponentToTwiceRaw(
                    squares, signs, words, component,
                    candidate.NormalRadicand, midpoint << 1, scale);
            if (comparison >= 0)
                low = midpoint + 1UL;
            else
                high = midpoint;
        }

        ulong floor = low - 1UL;
        int midpointComparison =
            CompareConvexContactCandidateNormalComponentToTwiceRaw(
                squares, signs, words, component,
                candidate.NormalRadicand, (floor << 1) | 1UL, scale);
        floor += GetNearestEvenIncrement(midpointComparison, floor);
        return Fixed64.FromRaw(sign * (long)floor);
    }

    private static int CompareConvexContactCandidateNormalComponentToTwiceRaw(
        ReadOnlySpan<ulong> squares,
        ReadOnlySpan<int> signs,
        int words,
        int component,
        ReadOnlySpan<ulong> radicand,
        ulong twiceRaw,
        ulong scale)
    {
        // (2 S n_i)^2 - twiceRaw^2 |n|^2 has the sign of the unsquared
        // comparison, since both magnitudes and the norm are nonnegative.
        Span<ulong> scaleSquared = stackalloc ulong[2];
        Fixed64.Multiply64To128(scale << 1, scale << 1,
            out scaleSquared[1], out scaleSquared[0]);
        Span<ulong> thresholdSquared = stackalloc ulong[2];
        Fixed64.Multiply64To128(twiceRaw, twiceRaw,
            out thresholdSquared[1], out thresholdSquared[0]);
        int productWords = words + 3;
        Span<ulong> first = stackalloc ulong[productWords];
        Span<ulong> second = stackalloc ulong[productWords];
        Span<ulong> rational = stackalloc ulong[productWords];
        Span<ulong> radical = stackalloc ulong[productWords];
        WideArithmetic.MultiplyMagnitudes(
            squares.Slice(component * words, words), scaleSquared, first);
        WideArithmetic.MultiplyMagnitudes(
            squares.Slice(6 * words, words), thresholdSquared, second);
        CombineWideSignedMagnitudes(first, 1, second, -1,
            rational, out int rationalSign);
        WideArithmetic.MultiplyMagnitudes(
            squares.Slice((component + 3) * words, words), scaleSquared, first);
        WideArithmetic.MultiplyMagnitudes(
            squares.Slice(7 * words, words), thresholdSquared, second);
        CombineWideSignedMagnitudes(
            first, signs[component], second, -signs[3],
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
