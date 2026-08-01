//=======================================================================
// WideArithmetic.Comparison.cs
//=======================================================================
// MIT License, Copyright (c) 2024–present David Oravsky (mrdav30)
// See LICENSE file in the project root for full license information.
//=======================================================================

using System;

namespace FixedMathSharp;

/// <content>
/// Comparison helpers for wide fixed-point arithmetic, including normalized
/// depth rounding and comparisons against wide intermediate representations.
/// </content>
internal static partial class WideArithmetic
{
    #region Normalized Depth Comparison

    /// <summary>
    /// Rounds a nonnegative depth represented as
    /// <c>overlap / (commonDenominator * sqrt(squaredAxisLength))</c>
    /// to the nearest raw Q32.32 value with ties to even.
    /// </summary>
    /// <remarks>
    /// Squared axis lengths at least <c>(2^31)^2</c> use a constant-time floor
    /// root approximation whose error is less than one raw result unit, then
    /// one exact midpoint correction. Smaller nonzero axes use an exact binary
    /// search over the representable nonnegative raw domain. Both paths apply
    /// exact clamping classification and nearest-even midpoint admission.
    /// </remarks>
    internal static Fixed64 GetRoundedNonNegativeNormalizedDepth(
        Signed576 overlap,
        Signed576 squaredAxisLength,
        Signed320 commonDenominator,
        out bool isClamped)
    {
        Signed192 maximumTwiceRaw = AddSigned192(
            Signed192.Signed(long.MaxValue),
            Signed192.Signed(long.MaxValue));
        if (CompareNormalizedDepthToTwiceRaw(
                overlap,
                squaredAxisLength,
                commonDenominator,
                maximumTwiceRaw) > 0)
        {
            isClamped = true;
            return Fixed64.MaxValue;
        }

        Signed576 approximationThreshold = Signed576.ExtendValue(
            Signed320.ExtendValue(Signed192.Signed(1L << 62)));
        if (!squaredAxisLength.IsZero
            && CompareNonNegative(
                squaredAxisLength,
                approximationThreshold) < 0)
        {
            long low = 0L;
            long high = long.MaxValue;
            while (low < high)
            {
                long candidate = (long)(
                    ((ulong)low + (ulong)high + 1UL) >> 1);
                Signed192 candidateLowerMidpoint = SubtractSigned192(
                    AddSigned192(
                        Signed192.Signed(candidate),
                        Signed192.Signed(candidate)),
                    Signed192.Signed(1L));
                int candidateComparison = CompareNormalizedDepthToTwiceRaw(
                    overlap,
                    squaredAxisLength,
                    commonDenominator,
                    candidateLowerMidpoint);
                bool roundsAtLeastCandidate = candidateComparison > 0
                    || (candidateComparison == 0
                        && (candidate & 1L) == 0L);
                if (roundsAtLeastCandidate)
                    low = candidate;
                else
                    high = candidate - 1L;
            }

            isClamped = false;
            return Fixed64.FromRaw(low);
        }

        Signed320 scaledAxisLength =
            GetFloorSquareRootScaledByFixed64(squaredAxisLength);
        Signed576 scaledDenominator = MultiplySigned320(
            scaledAxisLength,
            commonDenominator);
        if (!Fixed64.TryGetSignedRawRatio(
                Signed832.ExtendValue(overlap),
                Signed832.ExtendValue(scaledDenominator),
                FixedMath.SHIFT_AMOUNT_I,
                out Fixed64 depth))
        {
            // The floor root can place a representable exact result one raw
            // unit above the scalar domain. The exact midpoint correction
            // below distinguishes MaxValue from MaxValue - one raw unit.
            depth = Fixed64.MaxValue;
        }
        isClamped = false;
        if (depth == Fixed64.Zero)
            return depth;

        Signed192 lowerMidpoint = SubtractSigned192(
            AddSigned192(
                Signed192.Signed(depth.m_rawValue),
                Signed192.Signed(depth.m_rawValue)),
            Signed192.Signed(1L));
        int comparison = CompareNormalizedDepthToTwiceRaw(
            overlap,
            squaredAxisLength,
            commonDenominator,
            lowerMidpoint);
        if (comparison < (depth.m_rawValue & 1L))
        {
            depth = Fixed64.FromRaw(depth.m_rawValue - 1L);
        }

        return depth;
    }

    /// <summary>
    /// Compares nonnegative depths represented as
    /// <c>overlap / (commonDenominator * sqrt(squaredAxisLength))</c>.
    /// </summary>
    internal static int CompareNonNegativeNormalizedDepths(
        Signed576 leftOverlap,
        Signed576 leftSquaredAxisLength,
        Signed320 leftCommonDenominator,
        Signed576 rightOverlap,
        Signed576 rightSquaredAxisLength,
        Signed320 rightCommonDenominator)
    {
        Span<ulong> leftOverlapWords = stackalloc ulong[9];
        Span<ulong> rightOverlapWords = stackalloc ulong[9];
        Span<ulong> leftCommonWords = stackalloc ulong[5];
        Span<ulong> rightCommonWords = stackalloc ulong[5];
        Span<ulong> leftAxisWords = stackalloc ulong[9];
        Span<ulong> rightAxisWords = stackalloc ulong[9];
        GetMagnitude(leftOverlap, leftOverlapWords);
        GetMagnitude(rightOverlap, rightOverlapWords);
        GetMagnitude(
            leftCommonDenominator,
            out leftCommonWords[4],
            out leftCommonWords[3],
            out leftCommonWords[2],
            out leftCommonWords[1],
            out leftCommonWords[0]);
        GetMagnitude(
            rightCommonDenominator,
            out rightCommonWords[4],
            out rightCommonWords[3],
            out rightCommonWords[2],
            out rightCommonWords[1],
            out rightCommonWords[0]);
        GetMagnitude(leftSquaredAxisLength, leftAxisWords);
        GetMagnitude(rightSquaredAxisLength, rightAxisWords);

        Span<ulong> leftOverlapSquared = stackalloc ulong[18];
        Span<ulong> rightOverlapSquared = stackalloc ulong[18];
        Span<ulong> leftCommonSquared = stackalloc ulong[10];
        Span<ulong> rightCommonSquared = stackalloc ulong[10];
        MultiplyMagnitudes(
            leftOverlapWords,
            leftOverlapWords,
            leftOverlapSquared);
        MultiplyMagnitudes(
            rightOverlapWords,
            rightOverlapWords,
            rightOverlapSquared);
        MultiplyMagnitudes(
            leftCommonWords,
            leftCommonWords,
            leftCommonSquared);
        MultiplyMagnitudes(
            rightCommonWords,
            rightCommonWords,
            rightCommonSquared);

        Span<ulong> leftScaledOnce = stackalloc ulong[28];
        Span<ulong> rightScaledOnce = stackalloc ulong[28];
        Span<ulong> leftScaled = stackalloc ulong[40];
        Span<ulong> rightScaled = stackalloc ulong[40];
        MultiplyMagnitudes(
            leftOverlapSquared,
            rightCommonSquared,
            leftScaledOnce);
        MultiplyMagnitudes(
            rightOverlapSquared,
            leftCommonSquared,
            rightScaledOnce);
        MultiplyMagnitudes(
            leftScaledOnce,
            rightAxisWords,
            leftScaled);
        MultiplyMagnitudes(
            rightScaledOnce,
            leftAxisWords,
            rightScaled);
        return CompareMagnitudeEqualLength(leftScaled, rightScaled);
    }

    private static int CompareNormalizedDepthToTwiceRaw(
        Signed576 overlap,
        Signed576 squaredAxisLength,
        Signed320 commonDenominator,
        Signed192 twiceRaw)
    {
        Span<ulong> overlapWords = stackalloc ulong[9];
        Span<ulong> axisWords = stackalloc ulong[9];
        Span<ulong> commonWords = stackalloc ulong[5];
        Span<ulong> twiceRawWords = stackalloc ulong[3];
        GetMagnitude(overlap, overlapWords);
        GetMagnitude(squaredAxisLength, axisWords);
        GetMagnitude(
            commonDenominator,
            out commonWords[4],
            out commonWords[3],
            out commonWords[2],
            out commonWords[1],
            out commonWords[0]);
        GetMagnitude(
            twiceRaw,
            out twiceRawWords[2],
            out twiceRawWords[1],
            out twiceRawWords[0]);

        Span<ulong> left = stackalloc ulong[25];
        MultiplyMagnitudes(overlapWords, overlapWords, left);
        ShiftLeftMagnitude(left, 2);

        Span<ulong> commonSquared = stackalloc ulong[10];
        Span<ulong> twiceRawSquared = stackalloc ulong[6];
        Span<ulong> thresholdSquared = stackalloc ulong[16];
        Span<ulong> right = stackalloc ulong[25];
        MultiplyMagnitudes(commonWords, commonWords, commonSquared);
        MultiplyMagnitudes(
            twiceRawWords,
            twiceRawWords,
            twiceRawSquared);
        MultiplyMagnitudes(
            commonSquared,
            twiceRawSquared,
            thresholdSquared);
        MultiplyMagnitudes(thresholdSquared, axisWords, right);
        return CompareMagnitudeEqualLength(left, right);
    }

    #endregion

    #region Normalized Magnitude Comparison

    /// <summary>
    /// Compares signed ratios of the form
    /// <c>numerator / sqrt(squaredAxisLength)</c>.
    /// </summary>
    internal static int CompareSignedNormalizedMagnitudes(
        ReadOnlySpan<ulong> leftNumerator,
        int leftSign,
        ReadOnlySpan<ulong> leftSquaredAxisLength,
        ReadOnlySpan<ulong> rightNumerator,
        int rightSign,
        ReadOnlySpan<ulong> rightSquaredAxisLength)
    {
        leftSign = leftSign == 0 || IsZeroMagnitude(leftNumerator)
            ? 0
            : leftSign;
        rightSign = rightSign == 0 || IsZeroMagnitude(rightNumerator)
            ? 0
            : rightSign;
        if (leftSign != rightSign)
            return leftSign.CompareTo(rightSign);
        if (leftSign == 0)
            return 0;

        int wordCount = Math.Max(
            Math.Max(leftNumerator.Length, rightNumerator.Length),
            Math.Max(
                leftSquaredAxisLength.Length,
                rightSquaredAxisLength.Length));
        int productWordCount = wordCount * 3;
        Span<ulong> products =
            stackalloc ulong[productWordCount * 2];
        Span<ulong> square = stackalloc ulong[wordCount * 2];
        Span<ulong> paddedNumerator = stackalloc ulong[wordCount];
        Span<ulong> paddedAxis = stackalloc ulong[wordCount];

        paddedNumerator.Clear();
        leftNumerator.CopyTo(paddedNumerator);
        MultiplyMagnitudes(
            paddedNumerator,
            paddedNumerator,
            square);
        paddedAxis.Clear();
        rightSquaredAxisLength.CopyTo(paddedAxis);
        MultiplyMagnitudes(
            square,
            paddedAxis,
            products.Slice(0, productWordCount));

        paddedNumerator.Clear();
        rightNumerator.CopyTo(paddedNumerator);
        MultiplyMagnitudes(
            paddedNumerator,
            paddedNumerator,
            square);
        paddedAxis.Clear();
        leftSquaredAxisLength.CopyTo(paddedAxis);
        MultiplyMagnitudes(
            square,
            paddedAxis,
            products.Slice(
                productWordCount,
                productWordCount));

        int comparison = CompareMagnitudeEqualLength(
            products.Slice(0, productWordCount),
            products.Slice(productWordCount, productWordCount));
        return leftSign > 0 ? comparison : -comparison;
    }

    /// <summary>
    /// Gets the exact sign of
    /// <c>rational + coefficient * sqrt(squaredRadicand)</c>.
    /// </summary>
    internal static int GetSignedMagnitudeAndSquareRootSign(
        ReadOnlySpan<ulong> rational,
        int rationalSign,
        ReadOnlySpan<ulong> coefficient,
        int coefficientSign,
        ReadOnlySpan<ulong> squaredRadicand)
    {
        rationalSign = rationalSign == 0 || IsZeroMagnitude(rational)
            ? 0
            : rationalSign;
        coefficientSign =
            coefficientSign == 0
            || IsZeroMagnitude(coefficient)
            || IsZeroMagnitude(squaredRadicand)
                ? 0
                : coefficientSign;
        if (rationalSign == 0)
            return coefficientSign;
        if (coefficientSign == 0 || rationalSign == coefficientSign)
            return rationalSign;

        int wordCount = Math.Max(
            Math.Max(rational.Length, coefficient.Length),
            squaredRadicand.Length);
        int productWordCount = wordCount * 3;
        Span<ulong> products =
            stackalloc ulong[productWordCount * 2];
        products.Clear();
        Span<ulong> padded = stackalloc ulong[wordCount];
        Span<ulong> square = stackalloc ulong[wordCount * 2];

        padded.Clear();
        rational.CopyTo(padded);
        MultiplyMagnitudes(padded, padded, square);
        square.CopyTo(products);

        padded.Clear();
        coefficient.CopyTo(padded);
        MultiplyMagnitudes(padded, padded, square);
        padded.Clear();
        squaredRadicand.CopyTo(padded);
        MultiplyMagnitudes(
            square,
            padded,
            products.Slice(
                productWordCount,
                productWordCount));

        int comparison = CompareMagnitudeEqualLength(
            products.Slice(0, productWordCount),
            products.Slice(productWordCount, productWordCount));
        if (comparison == 0)
            return 0;
        return comparison > 0
            ? rationalSign
            : coefficientSign;
    }

    #endregion

    #region Radial Projection Comparison

    private const int RadialProjectionWordCount = 96;

    /// <summary>
    /// Compares two nonnegative radial projection depths without materializing
    /// either radical.
    /// </summary>
    /// <remarks>
    /// Each depth has the form
    /// (rational + common * sqrt(radicandNumerator / radicandDenominator)) /
    /// (common * sqrt(axisSquared)). The comparison squares the nonnegative
    /// depths and reduces the result to at most two radicals on either side.
    /// </remarks>
    internal static int CompareRadialProjectionDepths(
        Signed576 leftRational,
        Signed832 leftRadicandNumerator,
        Signed576 leftRadicandDenominator,
        Signed576 leftAxisSquared,
        Signed576 rightRational,
        Signed832 rightRadicandNumerator,
        Signed576 rightRadicandDenominator,
        Signed576 rightAxisSquared,
        Signed192 common)
    {
        Signed320 radialCoefficient = Signed320.ExtendValue(common);
        return CompareRadialProjectionDepths(
            Signed704.ExtendValue(leftRational),
            radialCoefficient,
            leftRadicandNumerator,
            leftRadicandDenominator,
            leftAxisSquared,
            Signed704.ExtendValue(rightRational),
            radialCoefficient,
            rightRadicandNumerator,
            rightRadicandDenominator,
            rightAxisSquared);
    }

    /// <summary>
    /// Compares two nonnegative radial projection depths without materializing
    /// either radical.
    /// </summary>
    /// <remarks>
    /// Each depth has the form
    /// (rational + radialCoefficient *
    /// sqrt(radicandNumerator / radicandDenominator)) /
    /// sqrt(axisSquared). Any shared positive denominator has already been
    /// omitted because it cannot change the ordering.
    /// </remarks>
    internal static int CompareRadialProjectionDepths(
        Signed704 leftRational,
        Signed320 leftRadialCoefficient,
        Signed832 leftRadicandNumerator,
        Signed576 leftRadicandDenominator,
        Signed576 leftAxisSquared,
        Signed704 rightRational,
        Signed320 rightRadialCoefficient,
        Signed832 rightRadicandNumerator,
        Signed576 rightRadicandDenominator,
        Signed576 rightAxisSquared)
    {
        Span<ulong> leftBase =
            stackalloc ulong[RadialProjectionWordCount];
        Span<ulong> rightBase =
            stackalloc ulong[RadialProjectionWordCount];
        BuildSquaredRadialDepthNumerator(
            leftRational,
            leftRadialCoefficient,
            leftRadicandNumerator,
            leftRadicandDenominator,
            rightAxisSquared,
            rightRadicandDenominator,
            leftBase);
        BuildSquaredRadialDepthNumerator(
            rightRational,
            rightRadialCoefficient,
            rightRadicandNumerator,
            rightRadicandDenominator,
            leftAxisSquared,
            leftRadicandDenominator,
            rightBase);

        int baseComparison = CompareMagnitudeEqualLength(
            leftBase,
            rightBase);
        Span<ulong> baseMagnitude =
            stackalloc ulong[RadialProjectionWordCount];
        if (baseComparison >= 0)
        {
            SubtractEqualMagnitudes(
                leftBase,
                rightBase,
                baseMagnitude);
        }
        else
        {
            SubtractEqualMagnitudes(
                rightBase,
                leftBase,
                baseMagnitude);
        }

        Span<ulong> baseRadicand =
            stackalloc ulong[RadialProjectionWordCount];
        MultiplyMagnitudes(
            baseMagnitude,
            baseMagnitude,
            baseRadicand);
        Span<ulong> leftRadicand =
            stackalloc ulong[RadialProjectionWordCount];
        BuildRadialDepthCrossRadicand(
            leftRadialCoefficient,
            rightAxisSquared,
            rightRadicandDenominator,
            leftRational,
            leftRadicandNumerator,
            leftRadicandDenominator,
            leftRadicand);
        Span<ulong> rightRadicand =
            stackalloc ulong[RadialProjectionWordCount];
        BuildRadialDepthCrossRadicand(
            rightRadialCoefficient,
            leftAxisSquared,
            leftRadicandDenominator,
            rightRational,
            rightRadicandNumerator,
            rightRadicandDenominator,
            rightRadicand);

        int baseSign = IsZeroMagnitude(baseRadicand)
            ? 0
            : baseComparison;
        int leftSign = IsZeroMagnitude(leftRadicand)
            ? 0
            : leftRational.Sign;
        int rightSign = IsZeroMagnitude(rightRadicand)
            ? 0
            : -rightRational.Sign;
        int positiveCount =
            (baseSign > 0 ? 1 : 0)
            + (leftSign > 0 ? 1 : 0)
            + (rightSign > 0 ? 1 : 0);
        int negativeCount =
            (baseSign < 0 ? 1 : 0)
            + (leftSign < 0 ? 1 : 0)
            + (rightSign < 0 ? 1 : 0);
        if (positiveCount == 0)
            return negativeCount == 0 ? 0 : -1;
        if (negativeCount == 0)
            return 1;

        Span<ulong> positiveFirst =
            stackalloc ulong[RadialProjectionWordCount];
        Span<ulong> positiveSecond =
            stackalloc ulong[RadialProjectionWordCount];
        Span<ulong> negativeFirst =
            stackalloc ulong[RadialProjectionWordCount];
        Span<ulong> negativeSecond =
            stackalloc ulong[RadialProjectionWordCount];
        positiveFirst.Clear();
        positiveSecond.Clear();
        negativeFirst.Clear();
        negativeSecond.Clear();
        int positiveIndex = 0;
        int negativeIndex = 0;
        AddSignedRadicand(
            baseRadicand,
            baseSign,
            positiveFirst,
            positiveSecond,
            negativeFirst,
            negativeSecond,
            ref positiveIndex,
            ref negativeIndex);
        AddSignedRadicand(
            leftRadicand,
            leftSign,
            positiveFirst,
            positiveSecond,
            negativeFirst,
            negativeSecond,
            ref positiveIndex,
            ref negativeIndex);
        AddSignedRadicand(
            rightRadicand,
            rightSign,
            positiveFirst,
            positiveSecond,
            negativeFirst,
            negativeSecond,
            ref positiveIndex,
            ref negativeIndex);
        return CompareNonNegativeRadicalPairs(
            positiveFirst,
            positiveSecond,
            negativeFirst,
            negativeSecond);
    }

    private static void AddSignedRadicand(
        ReadOnlySpan<ulong> radicand,
        int sign,
        Span<ulong> positiveFirst,
        Span<ulong> positiveSecond,
        Span<ulong> negativeFirst,
        Span<ulong> negativeSecond,
        ref int positiveIndex,
        ref int negativeIndex)
    {
        if (sign == 0)
            return;

        if (sign > 0)
        {
            radicand.CopyTo(
                positiveIndex++ == 0
                    ? positiveFirst
                    : positiveSecond);
            return;
        }

        radicand.CopyTo(
            negativeIndex++ == 0
                ? negativeFirst
                : negativeSecond);
    }

    private static void BuildSquaredRadialDepthNumerator(
        Signed704 rational,
        Signed320 radialCoefficient,
        Signed832 radicandNumerator,
        Signed576 radicandDenominator,
        Signed576 otherAxisSquared,
        Signed576 otherRadicandDenominator,
        Span<ulong> result)
    {
        result.Clear();
        Span<ulong> rationalWords = stackalloc ulong[11];
        Span<ulong> radialCoefficientWords = stackalloc ulong[5];
        Span<ulong> numeratorWords = stackalloc ulong[13];
        Span<ulong> denominatorWords = stackalloc ulong[9];
        Span<ulong> axisWords = stackalloc ulong[9];
        Span<ulong> otherDenominatorWords = stackalloc ulong[9];
        GetMagnitude(rational, rationalWords);
        GetMagnitude(
            radialCoefficient,
            out radialCoefficientWords[4],
            out radialCoefficientWords[3],
            out radialCoefficientWords[2],
            out radialCoefficientWords[1],
            out radialCoefficientWords[0]);
        GetMagnitude(radicandNumerator, numeratorWords);
        GetMagnitude(radicandDenominator, denominatorWords);
        GetMagnitude(otherAxisSquared, axisWords);
        GetMagnitude(
            otherRadicandDenominator,
            otherDenominatorWords);

        Span<ulong> rationalSquared = stackalloc ulong[22];
        Span<ulong> rationalTerm = stackalloc ulong[31];
        MultiplyMagnitudes(
            rationalWords,
            rationalWords,
            rationalSquared);
        MultiplyMagnitudes(
            rationalSquared,
            denominatorWords,
            rationalTerm);

        Span<ulong> coefficientSquared = stackalloc ulong[10];
        Span<ulong> radialTerm = stackalloc ulong[23];
        MultiplyMagnitudes(
            radialCoefficientWords,
            radialCoefficientWords,
            coefficientSquared);
        MultiplyMagnitudes(
            coefficientSquared,
            numeratorWords,
            radialTerm);

        Span<ulong> numerator = stackalloc ulong[32];
        numerator.Clear();
        AddMagnitudeInto(rationalTerm, numerator);
        AddMagnitudeInto(radialTerm, numerator);
        Span<ulong> axisScaled = stackalloc ulong[41];
        Span<ulong> fullyScaled = stackalloc ulong[50];
        MultiplyMagnitudes(
            numerator,
            axisWords,
            axisScaled);
        MultiplyMagnitudes(
            axisScaled,
            otherDenominatorWords,
            fullyScaled);
        fullyScaled.CopyTo(result);
    }

    private static void BuildRadialDepthCrossRadicand(
        Signed320 radialCoefficient,
        Signed576 otherAxisSquared,
        Signed576 otherRadicandDenominator,
        Signed704 rational,
        Signed832 radicandNumerator,
        Signed576 radicandDenominator,
        Span<ulong> result)
    {
        if (radialCoefficient.IsZero
            || rational.IsZero
            || radicandNumerator.IsZero)
        {
            result.Clear();
            return;
        }

        Span<ulong> radialCoefficientWords = stackalloc ulong[5];
        Span<ulong> axisWords = stackalloc ulong[9];
        Span<ulong> otherDenominatorWords = stackalloc ulong[9];
        Span<ulong> rationalWords = stackalloc ulong[11];
        Span<ulong> numeratorWords = stackalloc ulong[13];
        Span<ulong> denominatorWords = stackalloc ulong[9];
        GetMagnitude(
            radialCoefficient,
            out radialCoefficientWords[4],
            out radialCoefficientWords[3],
            out radialCoefficientWords[2],
            out radialCoefficientWords[1],
            out radialCoefficientWords[0]);
        GetMagnitude(otherAxisSquared, axisWords);
        GetMagnitude(
            otherRadicandDenominator,
            otherDenominatorWords);
        GetMagnitude(rational, rationalWords);
        GetMagnitude(radicandNumerator, numeratorWords);
        GetMagnitude(radicandDenominator, denominatorWords);

        Span<ulong> coefficientAndAxis = stackalloc ulong[14];
        Span<ulong> withRational = stackalloc ulong[25];
        Span<ulong> completeCoefficient = stackalloc ulong[34];
        Span<ulong> coefficientSquared = stackalloc ulong[68];
        Span<ulong> withNumerator = stackalloc ulong[81];
        MultiplyMagnitudes(
            radialCoefficientWords,
            axisWords,
            coefficientAndAxis);
        MultiplyMagnitudes(
            coefficientAndAxis,
            rationalWords,
            withRational);
        MultiplyMagnitudes(
            withRational,
            otherDenominatorWords,
            completeCoefficient);
        MultiplyMagnitudes(
            completeCoefficient,
            completeCoefficient,
            coefficientSquared);
        MultiplyMagnitudes(
            coefficientSquared,
            numeratorWords,
            withNumerator);
        MultiplyMagnitudes(
            withNumerator,
            denominatorWords,
            result);
        ShiftLeftMagnitude(result, 2);
    }

    private static int CompareNonNegativeRadicalPairs(
        ReadOnlySpan<ulong> leftFirst,
        ReadOnlySpan<ulong> leftSecond,
        ReadOnlySpan<ulong> rightFirst,
        ReadOnlySpan<ulong> rightSecond)
    {
        Span<ulong> leftBase =
            stackalloc ulong[RadialProjectionWordCount];
        Span<ulong> rightBase =
            stackalloc ulong[RadialProjectionWordCount];
        AddEqualMagnitudes(leftFirst, leftSecond, leftBase);
        AddEqualMagnitudes(rightFirst, rightSecond, rightBase);
        int baseComparison = CompareMagnitudeEqualLength(
            leftBase,
            rightBase);
        Span<ulong> baseMagnitude =
            stackalloc ulong[RadialProjectionWordCount];
        if (baseComparison >= 0)
        {
            SubtractEqualMagnitudes(
                leftBase,
                rightBase,
                baseMagnitude);
        }
        else
        {
            SubtractEqualMagnitudes(
                rightBase,
                leftBase,
                baseMagnitude);
        }

        Span<ulong> leftProduct =
            stackalloc ulong[RadialProjectionWordCount * 2];
        Span<ulong> rightProduct =
            stackalloc ulong[RadialProjectionWordCount * 2];
        MultiplyMagnitudes(leftFirst, leftSecond, leftProduct);
        MultiplyMagnitudes(rightFirst, rightSecond, rightProduct);
        if (baseComparison >= 0)
        {
            return ComparePositiveRadicalPairDifference(
                baseMagnitude,
                leftProduct,
                rightProduct);
        }

        return -ComparePositiveRadicalPairDifference(
            baseMagnitude,
            rightProduct,
            leftProduct);
    }

    private static int ComparePositiveRadicalPairDifference(
        ReadOnlySpan<ulong> positiveBase,
        ReadOnlySpan<ulong> sameSideProduct,
        ReadOnlySpan<ulong> oppositeSideProduct)
    {
        Span<ulong> baseSquared =
            stackalloc ulong[RadialProjectionWordCount * 2];
        Span<ulong> fourSame =
            stackalloc ulong[RadialProjectionWordCount * 2];
        Span<ulong> fourOpposite =
            stackalloc ulong[RadialProjectionWordCount * 2];
        MultiplyMagnitudes(
            positiveBase,
            positiveBase,
            baseSquared);
        sameSideProduct.CopyTo(fourSame);
        oppositeSideProduct.CopyTo(fourOpposite);
        ShiftLeftMagnitude(fourSame, 2);
        ShiftLeftMagnitude(fourOpposite, 2);

        Span<ulong> knownLeft =
            stackalloc ulong[RadialProjectionWordCount * 2];
        AddEqualMagnitudes(baseSquared, fourSame, knownLeft);
        int knownComparison = CompareMagnitudeEqualLength(
            knownLeft,
            fourOpposite);
        if (knownComparison > 0)
            return 1;
        if (knownComparison == 0)
        {
            // This reducer partitions three signed radical terms. Therefore
            // at least one side has a zero product; equality of the known
            // squared terms can only occur when the remaining cross term is
            // also zero.
            return 0;
        }

        Span<ulong> remainder =
            stackalloc ulong[RadialProjectionWordCount * 2];
        SubtractEqualMagnitudes(
            fourOpposite,
            knownLeft,
            remainder);
        Span<ulong> crossSquared =
            stackalloc ulong[RadialProjectionWordCount * 4];
        Span<ulong> remainderSquared =
            stackalloc ulong[RadialProjectionWordCount * 4];
        MultiplyMagnitudes(
            baseSquared,
            sameSideProduct,
            crossSquared);
        ShiftLeftMagnitude(crossSquared, 4);
        MultiplyMagnitudes(
            remainder,
            remainder,
            remainderSquared);
        return CompareMagnitudeEqualLength(
            crossSquared,
            remainderSquared);
    }

    #endregion

    #region Radical Comparison


    /// <summary>
    /// Compares <c>sqrt(firstNumerator / firstDenominator) +
    /// sqrt(secondNumerator / secondDenominator)</c> with a nonnegative ratio.
    /// </summary>
    internal static int CompareNonNegativeRadicalSumToRatio(
        Signed576 firstNumerator,
        Signed192 firstDenominator,
        Signed576 secondNumerator,
        Signed192 secondDenominator,
        Signed320 ratioNumerator,
        Signed192 ratioDenominator)
    {
        bool firstZero = firstNumerator.IsZero;
        bool secondZero = secondNumerator.IsZero;
        if (firstZero && secondZero)
            return ratioNumerator.IsZero ? 0 : -1;
        if (firstZero)
        {
            return CompareNonNegativeRadicalToRatio(
                secondNumerator,
                secondDenominator,
                ratioNumerator,
                ratioDenominator);
        }
        if (secondZero)
        {
            return CompareNonNegativeRadicalToRatio(
                firstNumerator,
                firstDenominator,
                ratioNumerator,
                ratioDenominator);
        }

        Signed576 ratioSquared = MultiplySigned320(ratioNumerator, ratioNumerator);
        Signed576 denominatorProduct = MultiplySigned320(
            Signed320.ExtendValue(firstDenominator),
            Signed320.ExtendValue(secondDenominator));
        Signed576 squaredRatioTerm = MultiplyNonNegativeToSigned576(
            ratioSquared,
            denominatorProduct);
        Signed576 radicalSquares = AddSigned576(
            MultiplySigned576(firstNumerator, secondDenominator),
            MultiplySigned576(secondNumerator, firstDenominator));
        Signed320 ratioDenominatorSquared = MultiplySigned192(
            ratioDenominator,
            ratioDenominator);
        _ = Signed192.TryNarrowSigned(
            Signed576.ExtendValue(ratioDenominatorSquared),
            out Signed192 narrowRatioDenominatorSquared);
        Signed576 radicalSquareTerm = MultiplySigned576(
            radicalSquares,
            narrowRatioDenominatorSquared);
        Signed576 remainder = SubtractSigned576(
            squaredRatioTerm,
            radicalSquareTerm);
        if (remainder.Sign <= 0)
            return 1;

        return CompareRadicalCrossTerm(
            firstNumerator,
            firstDenominator,
            secondNumerator,
            secondDenominator,
            ratioDenominator,
            remainder);
    }

    /// <summary>
    /// Compares a nonnegative radical with a nonnegative ratio whose
    /// denominators require the full five-word geometry range.
    /// </summary>
    internal static int CompareNonNegativeRadicalToRatio(
        Signed576 numerator,
        Signed320 denominator,
        Signed320 ratioNumerator,
        Signed320 ratioDenominator)
    {
        Span<ulong> numeratorWords = stackalloc ulong[9];
        Span<ulong> denominatorWords = stackalloc ulong[5];
        Span<ulong> ratioNumeratorWords = stackalloc ulong[5];
        Span<ulong> ratioDenominatorWords = stackalloc ulong[5];
        GetMagnitude(numerator, numeratorWords);
        GetMagnitude(
            denominator,
            out denominatorWords[4],
            out denominatorWords[3],
            out denominatorWords[2],
            out denominatorWords[1],
            out denominatorWords[0]);
        GetMagnitude(
            ratioNumerator,
            out ratioNumeratorWords[4],
            out ratioNumeratorWords[3],
            out ratioNumeratorWords[2],
            out ratioNumeratorWords[1],
            out ratioNumeratorWords[0]);
        GetMagnitude(
            ratioDenominator,
            out ratioDenominatorWords[4],
            out ratioDenominatorWords[3],
            out ratioDenominatorWords[2],
            out ratioDenominatorWords[1],
            out ratioDenominatorWords[0]);

        Span<ulong> ratioNumeratorSquared = stackalloc ulong[10];
        Span<ulong> ratioDenominatorSquared = stackalloc ulong[10];
        Span<ulong> left = stackalloc ulong[19];
        Span<ulong> right = stackalloc ulong[19];
        MultiplyMagnitudes(
            ratioNumeratorWords,
            ratioNumeratorWords,
            ratioNumeratorSquared);
        MultiplyMagnitudes(
            ratioDenominatorWords,
            ratioDenominatorWords,
            ratioDenominatorSquared);
        MultiplyMagnitudes(
            numeratorWords,
            ratioDenominatorSquared,
            left);
        MultiplyMagnitudes(
            ratioNumeratorSquared,
            denominatorWords,
            right);
        return CompareMagnitudeEqualLength(left, right);
    }

    /// <summary>
    /// Returns the sign of a signed rational term plus a signed coefficient
    /// multiplied by one nonnegative radical.
    /// </summary>
    internal static int CompareSignedLinearRadicalToZero(
        Signed832 rational,
        Signed704 radicalCoefficient,
        Signed576 radicandNumerator,
        Signed320 radicandDenominator)
    {
        int rationalSign = rational.Sign;
        int coefficientSign = radicandNumerator.IsZero
            ? 0
            : radicalCoefficient.Sign;
        if (rationalSign == 0)
            return coefficientSign;
        if (coefficientSign == 0 || rationalSign == coefficientSign)
            return rationalSign;

        Span<ulong> rationalWords = stackalloc ulong[13];
        Span<ulong> coefficientWords = stackalloc ulong[11];
        Span<ulong> numeratorWords = stackalloc ulong[9];
        Span<ulong> denominatorWords = stackalloc ulong[5];
        GetMagnitude(rational, rationalWords);
        GetMagnitude(radicalCoefficient, coefficientWords);
        GetMagnitude(radicandNumerator, numeratorWords);
        GetMagnitude(
            radicandDenominator,
            out denominatorWords[4],
            out denominatorWords[3],
            out denominatorWords[2],
            out denominatorWords[1],
            out denominatorWords[0]);

        Span<ulong> rationalSquared = stackalloc ulong[26];
        Span<ulong> coefficientSquared = stackalloc ulong[22];
        Span<ulong> rationalScaled = stackalloc ulong[31];
        Span<ulong> coefficientScaled = stackalloc ulong[31];
        MultiplyMagnitudes(
            rationalWords,
            rationalWords,
            rationalSquared);
        MultiplyMagnitudes(
            coefficientWords,
            coefficientWords,
            coefficientSquared);
        MultiplyMagnitudes(
            rationalSquared,
            denominatorWords,
            rationalScaled);
        MultiplyMagnitudes(
            coefficientSquared,
            numeratorWords,
            coefficientScaled);
        int comparison = CompareMagnitudeEqualLength(
            rationalScaled,
            coefficientScaled);
        return comparison == 0
            ? 0
            : comparison > 0
                ? rationalSign
                : coefficientSign;
    }

    private static int CompareNonNegativeRadicalToRatio(
        Signed576 numerator,
        Signed192 denominator,
        Signed320 ratioNumerator,
        Signed192 ratioDenominator)
    {
        Signed320 ratioDenominatorSquared = MultiplySigned192(
            ratioDenominator,
            ratioDenominator);
        _ = Signed192.TryNarrowSigned(
            Signed576.ExtendValue(ratioDenominatorSquared),
            out Signed192 narrowRatioDenominatorSquared);
        Signed576 left = MultiplySigned576(
            numerator,
            narrowRatioDenominatorSquared);
        Signed576 right = MultiplySigned576(
            MultiplySigned320(ratioNumerator, ratioNumerator),
            denominator);
        return CompareNonNegative(left, right);
    }

    private static int CompareRadicalCrossTerm(
        Signed576 firstNumerator,
        Signed192 firstDenominator,
        Signed576 secondNumerator,
        Signed192 secondDenominator,
        Signed192 ratioDenominator,
        Signed576 remainder)
    {
        Span<ulong> firstNumeratorWords = stackalloc ulong[9];
        Span<ulong> secondNumeratorWords = stackalloc ulong[9];
        Span<ulong> firstDenominatorWords = stackalloc ulong[3];
        Span<ulong> secondDenominatorWords = stackalloc ulong[3];
        Span<ulong> ratioDenominatorWords = stackalloc ulong[3];
        Span<ulong> remainderWords = stackalloc ulong[9];
        GetMagnitude(firstNumerator, firstNumeratorWords);
        GetMagnitude(secondNumerator, secondNumeratorWords);
        GetMagnitude(
            firstDenominator,
            out firstDenominatorWords[2],
            out firstDenominatorWords[1],
            out firstDenominatorWords[0]);
        GetMagnitude(
            secondDenominator,
            out secondDenominatorWords[2],
            out secondDenominatorWords[1],
            out secondDenominatorWords[0]);
        GetMagnitude(
            ratioDenominator,
            out ratioDenominatorWords[2],
            out ratioDenominatorWords[1],
            out ratioDenominatorWords[0]);
        GetMagnitude(remainder, remainderWords);

        Span<ulong> numeratorProduct = stackalloc ulong[18];
        Span<ulong> ratioSquared = stackalloc ulong[6];
        Span<ulong> ratioFourth = stackalloc ulong[12];
        Span<ulong> denominatorProduct = stackalloc ulong[6];
        Span<ulong> numeratorAndRatio = stackalloc ulong[30];
        Span<ulong> left = stackalloc ulong[36];
        Span<ulong> right = stackalloc ulong[36];
        MultiplyMagnitudes(
            firstNumeratorWords,
            secondNumeratorWords,
            numeratorProduct);
        MultiplyMagnitudes(
            ratioDenominatorWords,
            ratioDenominatorWords,
            ratioSquared);
        MultiplyMagnitudes(ratioSquared, ratioSquared, ratioFourth);
        MultiplyMagnitudes(
            firstDenominatorWords,
            secondDenominatorWords,
            denominatorProduct);
        MultiplyMagnitudes(
            numeratorProduct,
            ratioFourth,
            numeratorAndRatio);
        MultiplyMagnitudes(
            numeratorAndRatio,
            denominatorProduct,
            left);
        ShiftLeftMagnitude(left, 2);
        MultiplyMagnitudes(remainderWords, remainderWords, right);

        int comparison = CompareMagnitudeEqualLength(left, right);
        return comparison == 0 ? 0 : comparison > 0 ? 1 : -1;
    }

    #endregion
}
