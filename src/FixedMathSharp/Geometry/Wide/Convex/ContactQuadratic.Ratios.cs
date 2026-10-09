//=======================================================================
// ContactQuadratic.Ratios.cs
//=======================================================================
// MIT License, Copyright (c) 2024-present David Oravsky (mrdav30)
// See LICENSE file in the project root for full license information.
//=======================================================================
using System;
using static FixedMathSharp.Geometry.CylinderContactAlgebra;

namespace FixedMathSharp.Geometry;

/// <content>Exact comparison of fractions in two possibly different quadratic fields.</content>
internal readonly ref partial struct ContactQuadratic
{
    /// <summary>Compares two exact quadratic fractions with nonzero denominators.</summary>
    internal static int CompareRatios(ContactQuadratic firstNumerator, ContactQuadratic firstDenominator,
        ReadOnlySpan<ulong> firstRoot, ContactQuadratic secondNumerator, ContactQuadratic secondDenominator,
        ReadOnlySpan<ulong> secondRoot)
    {
        if ((firstNumerator.Signs[1] | firstDenominator.Signs[1] | secondNumerator.Signs[1] | secondDenominator.Signs[1]) == 0)
        {
            // Rational events need only their two signed cross products. The
            // root banks and four-term quadratic field carry no contribution.
            int firstFractionSign = firstNumerator.Signs[0] * firstDenominator.Signs[0];
            int secondFractionSign = secondNumerator.Signs[0] * secondDenominator.Signs[0];
            System.Diagnostics.Debug.Assert(firstDenominator.Signs[0] != 0 && secondDenominator.Signs[0] != 0);
            if (firstFractionSign != secondFractionSign)
                return firstFractionSign.CompareTo(secondFractionSign);
            if (firstFractionSign == 0)
                return 0;
            int productWords = Math.Max(
                WideArithmetic.GetActiveMagnitudeLength(firstNumerator.Rational) + WideArithmetic.GetActiveMagnitudeLength(secondDenominator.Rational),
                WideArithmetic.GetActiveMagnitudeLength(secondNumerator.Rational) + WideArithmetic.GetActiveMagnitudeLength(firstDenominator.Rational));
            Span<ulong> products = stackalloc ulong[2 * productWords];
            Span<ulong> firstProduct = products[..productWords], secondProduct = products[productWords..];
            WideArithmetic.MultiplyMagnitudes(firstNumerator.Rational, secondDenominator.Rational, firstProduct);
            WideArithmetic.MultiplyMagnitudes(secondNumerator.Rational, firstDenominator.Rational, secondProduct);
            return firstFractionSign * WideArithmetic.CompareMagnitudeEqualLength(firstProduct, secondProduct);
        }
        int firstSign = firstDenominator.Sign(firstRoot), secondSign = secondDenominator.Sign(secondRoot);
        System.Diagnostics.Debug.Assert(firstSign != 0 && secondSign != 0);
        int words = 2 * Math.Max(Math.Max(ActiveWords(firstNumerator), ActiveWords(firstDenominator)),
            Math.Max(ActiveWords(secondNumerator), ActiveWords(secondDenominator))) + 1;
        Span<ulong> coefficients = stackalloc ulong[4 * words];
        Span<ulong> product = stackalloc ulong[words];
        Span<int> signs = stackalloc int[4];
        // Cross multiplication has four coefficients A+B*sqrt(K)+C*sqrt(L)+D*sqrt(KL).
        // Each is a difference of two complete products; one extra limb covers its carry.
        for (int left = 0; left < 2; left++)
        for (int right = 0; right < 2; right++)
        {
            int index = left + 2 * right;
            Span<ulong> coefficient = coefficients.Slice(index * words, words);
            WideArithmetic.MultiplyMagnitudes(Part(firstNumerator, left), Part(secondDenominator, right), coefficient);
            int sign = IsZero(coefficient) ? 0 : firstNumerator.Signs[left] * secondDenominator.Signs[right];
            WideArithmetic.MultiplyMagnitudes(Part(firstDenominator, left), Part(secondNumerator, right), product);
            CylinderContactAlgebra.Add(product, -firstDenominator.Signs[left] * secondNumerator.Signs[right], coefficient, ref sign);
            signs[index] = sign;
        }
        return firstSign * secondSign * FourTermSign(coefficients, signs, words, firstRoot, secondRoot);
    }

    private static int FourTermSign(Span<ulong> coefficients, Span<int> signs, int words,
        ReadOnlySpan<ulong> firstRoot, ReadOnlySpan<ulong> secondRoot)
    {
        ContactQuadratic first = At(coefficients, signs, 0, words);
        ContactQuadratic second = At(coefficients, signs, 1, words);
        int firstSign = first.Sign(firstRoot);
        if (IsZero(secondRoot))
            return firstSign;
        int secondSign = second.Sign(firstRoot);
        if (firstSign == 0)
            return secondSign;
        if (secondSign == 0 || firstSign == secondSign)
            return firstSign;

        // X and Y*sqrt(L) have opposite signs. The sign of X²-LY² chooses
        // the larger magnitude, with exact zero covering dependent-root ties.
        // Squaring needs 2W+|K| limbs; scaling adds |L|, then one carry limb.
        int squareWords = 2 * words + WideArithmetic.GetActiveMagnitudeLength(firstRoot)
            + WideArithmetic.GetActiveMagnitudeLength(secondRoot) + 1;
        Span<ulong> values = stackalloc ulong[6 * squareWords];
        Span<int> squareSigns = stackalloc int[6];
        ContactQuadratic firstSquare = At(values, squareSigns, 0, squareWords);
        ContactQuadratic secondSquare = At(values, squareSigns, 1, squareWords);
        ContactQuadratic scaled = At(values, squareSigns, 2, squareWords);
        Multiply(first, first, firstRoot, firstSquare);
        Multiply(second, second, firstRoot, secondSquare);
        Scale(secondSquare, secondRoot, 1, scaled);
        firstSquare.Add(scaled, -1);
        return firstSign * firstSquare.Sign(firstRoot);
    }

    internal static int ActiveWords(ContactQuadratic value) => Math.Max(
        WideArithmetic.GetActiveMagnitudeLength(value.Rational), WideArithmetic.GetActiveMagnitudeLength(value.Radical));

    private static ReadOnlySpan<ulong> Part(ContactQuadratic value, int index) =>
        index == 0 ? value.Rational : value.Radical;

    /// <summary>
    /// Rounds the nonnegative raw ratio N*sqrt(metric)/(D*scale) to nearest-even,
    /// returning false when that rounded raw value cannot fit Fixed64.
    /// </summary>
    /// <remarks>D and scale are positive. All comparisons retain the original field.</remarks>
    internal static bool TryRoundRootRatio(ContactQuadratic numerator, ContactQuadratic denominator,
        ReadOnlySpan<ulong> root, ReadOnlySpan<ulong> metric, ReadOnlySpan<ulong> scale, out Fixed64 result)
    {
        if (numerator.Signs[1] == 0 && denominator.Signs[1] == 0
            && WideArithmetic.GetActiveMagnitudeLength(metric) == 1 && metric[0] == 1UL)
        {
            // With a unit metric and rational fields the depth is exactly
            // N/(D*scale). Reuse raw nearest-even division, including its
            // overflow boundary, without squaring or searching a root. The
            // disjoint product reserves both complete active operand lengths.
            Span<ulong> divisor = stackalloc ulong[WideArithmetic.GetActiveMagnitudeLength(denominator.Rational)
                + WideArithmetic.GetActiveMagnitudeLength(scale)];
            WideArithmetic.MultiplyMagnitudes(denominator.Rational, scale, divisor);
            return Fixed64.TryGetSignedRawRatio(numerator.Rational, divisor, negative: false, out result);
        }
        // Each square needs 2W+|K| limbs. Metric/scale products and a
        // 65-bit doubled raw threshold add the bounded suffix below.
        int words = 2 * Math.Max(ActiveWords(numerator), ActiveWords(denominator))
            + WideArithmetic.GetActiveMagnitudeLength(root)
            + Math.Max(WideArithmetic.GetActiveMagnitudeLength(metric),
                2 * WideArithmetic.GetActiveMagnitudeLength(scale) + 3) + 2;
        Span<ulong> values = stackalloc ulong[6 * words];
        Span<int> signs = stackalloc int[6];
        ContactQuadratic square = At(values, signs, 0, words), left = At(values, signs, 1, words);
        ContactQuadratic right = At(values, signs, 2, words);
        Span<ulong> scaleSquare = stackalloc ulong[words];
        Multiply(numerator, numerator, root, square);
        Scale(square, metric, 1, left);
        left.Add(left); left.Add(left); // Compare doubled depth to raw half-unit thresholds.
        Multiply(denominator, denominator, root, square);
        WideArithmetic.MultiplyMagnitudes(scale, scale, scaleSquare);
        Scale(square, scaleSquare, 1, right);
        return TryRoundSquaredRatio(left, right, root, out result);
    }

    /// <summary>Rounds sqrt(N/D) in raw units, retaining the exact original field.</summary>
    /// <remarks>N is nonnegative and D is positive; false means rounded overflow.</remarks>
    internal static bool TryRoundSquareRootRatio(ContactQuadratic numerator, ContactQuadratic denominator,
        ReadOnlySpan<ulong> root, out Fixed64 result)
    {
        int words = numerator.FieldWords + 1;
        Span<ulong> values = stackalloc ulong[2 * words];
        Span<int> signs = stackalloc int[2];
        ContactQuadratic left = At(values, signs, 0, words);
        numerator.CopyTo(left); left.Add(left); left.Add(left);
        return TryRoundSquaredRatio(left, denominator, root, out result);
    }

    private static bool TryRoundSquaredRatio(ContactQuadratic left, ContactQuadratic right,
        ReadOnlySpan<ulong> root, out Fixed64 result)
    {
        // Complete threshold multiplication adds at most 130 bits. The third
        // extra limb also covers subtraction's carry; padded source tails do
        // not increase the transient buffer or the binary-search work.
        int words = Math.Max(ActiveWords(left), ActiveWords(right)) + 3;
        Span<ulong> values = stackalloc ulong[2 * words];
        Span<int> signs = stackalloc int[2];
        ContactQuadratic query = At(values, signs, 0, words);
        result = default;
        Signed192 upperHalf = WideArithmetic.AddSigned192(
            WideArithmetic.AddSigned192(Signed192.Signed(long.MaxValue), Signed192.Signed(long.MaxValue)), Signed192.Signed(1));
        // MaxValue is odd: its exact upper half rounds to the unrepresentable
        // next even integer. Values below that half still round to MaxValue.
        if (CompareDoubledRootRatio(left, right, root, upperHalf, query) >= 0)
            return false;
        long low = 0;
        if (left.Signs[1] == 0 && right.Signs[1] == 0)
        {
            // L/R is four times the raw depth squared. Reuse the exact
            // clipped integer root: floor(sqrt(4*d*d))/2 = floor(d).
            // The range check above proves the clipped result loses no bits.
            int width = Math.Max(2, Math.Max(ActiveWords(left), ActiveWords(right)));
            Span<ulong> rational = stackalloc ulong[2 * width];
            rational.Clear();
            left.Rational[..Math.Min(width, left.FieldWords)].CopyTo(rational[..width]);
            right.Rational[..Math.Min(width, right.FieldWords)].CopyTo(rational[width..]);
            low = (long)(WideArithmetic.GetRatioFloorSquareRoot(rational[..width], rational[width..], ulong.MaxValue) >> 1);
        }
        else
        {
            long high = long.MaxValue;
            while (low < high)
            {
                long midpoint = low + ((high - low) >> 1) + ((high - low) & 1);
                Signed192 threshold = WideArithmetic.AddSigned192(Signed192.Signed(midpoint), Signed192.Signed(midpoint));
                if (CompareDoubledRootRatio(left, right, root, threshold, query) >= 0)
                    low = midpoint;
                else
                    high = midpoint - 1;
            }
        }
        Signed192 half = WideArithmetic.AddSigned192(
            WideArithmetic.AddSigned192(Signed192.Signed(low), Signed192.Signed(low)), Signed192.Signed(1));
        int comparison = CompareDoubledRootRatio(left, right, root, half, query);
        result = Fixed64.FromRaw(low + (comparison > 0 || comparison == 0 && (low & 1) != 0 ? 1 : 0));
        return true;
    }

    private static int CompareDoubledRootRatio(ContactQuadratic left, ContactQuadratic right,
        ReadOnlySpan<ulong> root, Signed192 threshold, ContactQuadratic query)
    {
        Scale(right, Signed576.ExtendValue(WideArithmetic.MultiplySigned192(threshold, threshold)), query);
        query.MultiplySign(-1); query.Add(left);
        return query.Sign(root);
    }

}
