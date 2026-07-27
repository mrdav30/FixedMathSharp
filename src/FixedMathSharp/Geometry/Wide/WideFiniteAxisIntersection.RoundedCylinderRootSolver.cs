//=======================================================================
// WideFiniteAxisIntersection.RoundedCylinderRootSolver.cs
//=======================================================================
// MIT License, Copyright (c) 2024–present David Oravsky (mrdav30)
// See LICENSE file in the project root for full license information.
//=======================================================================

using System;

namespace FixedMathSharp.Bounds;

/// <content>
/// Implements a wide-precision root solver for rounded-cylinder (capsule-like) torus
/// intersection tests, using a Sturm sequence built over arbitrary-precision integer
/// coefficients to isolate and count roots within a bounded segment length.                                                                                                                                                                                                                                
/// </content>
internal static partial class WideFiniteAxisIntersection
{
    private const int RoundedCylinderPolynomialCount = 5;
    private const int RoundedCylinderCoefficientCount = 5;
    // The largest factorized quartic subresultant transient is below 2,772 bits.
    private const int RoundedCylinderWideLimbCount = 44;

    private static bool TryGetRoundedCylinderRimDistanceInterval(
        RoundedCylinderTorusPolynomial polynomial,
        Fixed64 segmentLength,
        bool startContained,
        bool endContained,
        bool calculateExit,
        out Fixed64 entry,
        out Fixed64 exit)
    {
        entry = default;
        exit = default;
        if (segmentLength == Fixed64.Zero)
            return startContained;

        Span<ulong> coefficients = stackalloc ulong[
            RoundedCylinderPolynomialCount
            * RoundedCylinderCoefficientCount
            * RoundedCylinderWideLimbCount];
        Span<sbyte> signs = stackalloc sbyte[
            RoundedCylinderPolynomialCount * RoundedCylinderCoefficientCount];
        Span<int> degrees = stackalloc int[RoundedCylinderPolynomialCount];
        coefficients.Clear();
        signs.Clear();
        degrees.Clear();

        ulong maximum = unchecked((ulong)segmentLength.m_rawValue);
        int sequenceCount = BuildRoundedCylinderSturmSequence(
            polynomial,
            coefficients,
            signs,
            degrees);
        if (sequenceCount == 0)
        {
            exit = segmentLength;
            return true;
        }
        HomogenizeRoundedCylinderSturmSequence(
            coefficients,
            degrees,
            sequenceCount,
            maximum);

        int lowerVariations = GetRoundedCylinderSignVariations(
            coefficients,
            signs,
            degrees,
            sequenceCount,
            0UL,
            1UL);
        int upperVariations = GetRoundedCylinderSignVariations(
            coefficients,
            signs,
            degrees,
            sequenceCount,
            maximum,
            1UL);
        int rootCount = lowerVariations - upperVariations;
        bool rootAtStart = polynomial.H0.IsZero;
        if (rootCount == 0)
        {
            if (!startContained && !rootAtStart)
                return false;

            entry = Fixed64.Zero;
            exit = endContained ? segmentLength : Fixed64.Zero;
            return true;
        }

        entry = startContained || rootAtStart
            ? Fixed64.Zero
            : FindRoundedCylinderRoot(
                coefficients,
                signs,
                degrees,
                sequenceCount,
                maximum,
                lowerVariations,
                upperVariations,
                rootCount,
                findFirst: true);
        if (!calculateExit)
        {
            exit = entry;
            return true;
        }
        exit = endContained
            ? segmentLength
            : FindRoundedCylinderRoot(
                coefficients,
                signs,
                degrees,
                sequenceCount,
                maximum,
                lowerVariations,
                upperVariations,
                rootCount,
                findFirst: false);
        return true;
    }

    private static Fixed64 FindRoundedCylinderRoot(
        Span<ulong> coefficients,
        Span<sbyte> signs,
        Span<int> degrees,
        int sequenceCount,
        ulong maximum,
        int lowerVariations,
        int upperVariations,
        int totalRootCount,
        bool findFirst)
    {
        ulong lower = 0UL;
        ulong upper = maximum;
        int intervalRootCount = totalRootCount;
        while (upper - lower > 1UL)
        {
            if (intervalRootCount == 1)
            {
                int lowerSign = EvaluateRoundedCylinderPolynomialSign(
                    coefficients,
                    signs,
                    degrees[0],
                    0,
                    lower,
                    1UL);
                // The global start root is handled before isolation; midpoint
                // roots remain on the upper boundary of the retained interval.
                int upperSign = EvaluateRoundedCylinderPolynomialSign(
                    coefficients,
                    signs,
                    degrees[0],
                    0,
                    upper,
                    1UL);
                if (upperSign == 0)
                    return Fixed64.FromRaw((long)upper);

                if (lowerSign != upperSign)
                {
                    return FindRoundedCylinderSignChangingRoot(
                        coefficients,
                        signs,
                        degrees[0],
                        maximum,
                        lower,
                        upper,
                        lowerSign);
                }
            }

            ulong midpoint = lower + ((upper - lower) >> 1);
            int midpointVariations = GetRoundedCylinderSignVariations(
                coefficients,
                signs,
                degrees,
                sequenceCount,
                midpoint,
                1UL);
            int leftRootCount = lowerVariations - midpointVariations;
            if (findFirst ? leftRootCount > 0 : leftRootCount == intervalRootCount)
            {
                upper = midpoint;
                upperVariations = midpointVariations;
                intervalRootCount = leftRootCount;
            }
            else
            {
                lower = midpoint;
                lowerVariations = midpointVariations;
                intervalRootCount -= leftRootCount;
            }
        }

        ulong midpointNumerator = (lower << 1) | 1UL;
        int halfVariations = GetRoundedCylinderSignVariations(
            coefficients,
            signs,
            degrees,
            sequenceCount,
            midpointNumerator,
            2UL);
        int leftHalfRootCount = lowerVariations - halfVariations;
        int midpointSign = EvaluateRoundedCylinderPolynomialSign(
            coefficients,
            signs,
            degrees[0],
            0,
            midpointNumerator,
            2UL);

        ulong rounded;
        if (findFirst)
        {
            rounded = leftHalfRootCount > 0 ? lower : upper;
            if ((midpointSign == 0) & (leftHalfRootCount == 1))
                rounded = lower + (lower & 1UL);
        }
        else
        {
            int rightHalfRootCount = intervalRootCount - leftHalfRootCount;
            rounded = rightHalfRootCount > 0 ? upper : lower;
            if ((midpointSign == 0) & (rightHalfRootCount == 0))
                rounded = lower + (lower & 1UL);
        }

        return Fixed64.FromRaw((long)rounded);
    }

    private static Fixed64 FindRoundedCylinderSignChangingRoot(
        Span<ulong> coefficients,
        Span<sbyte> signs,
        int degree,
        ulong maximum,
        ulong lower,
        ulong upper,
        int lowerSign)
    {
        while (upper - lower > 1UL)
        {
            ulong midpoint = lower + ((upper - lower) >> 1);
            int midpointSign = EvaluateRoundedCylinderPolynomialSign(
                coefficients,
                signs,
                degree,
                0,
                midpoint,
                1UL);
            if (midpointSign == 0)
                return Fixed64.FromRaw((long)midpoint);
            if (midpointSign == lowerSign)
                lower = midpoint;
            else
                upper = midpoint;
        }

        ulong midpointNumerator = (lower << 1) | 1UL;
        int halfSign = EvaluateRoundedCylinderPolynomialSign(
            coefficients,
            signs,
            degree,
            0,
            midpointNumerator,
            2UL);
        if (halfSign == 0)
            return Fixed64.FromRaw((long)(lower + (lower & 1UL)));
        return Fixed64.FromRaw((long)(halfSign == lowerSign ? upper : lower));
    }

    private static void HomogenizeRoundedCylinderSturmSequence(
        Span<ulong> coefficients,
        Span<int> degrees,
        int sequenceCount,
        ulong segmentLengthRaw)
    {
        Span<ulong> product = stackalloc ulong[RoundedCylinderWideLimbCount];
        for (int polynomialIndex = 0; polynomialIndex < sequenceCount; polynomialIndex++)
        {
            int degree = degrees[polynomialIndex];
            for (int coefficientIndex = 0; coefficientIndex < degree; coefficientIndex++)
            {
                Span<ulong> coefficient = GetRoundedCylinderCoefficient(
                    coefficients,
                    polynomialIndex,
                    coefficientIndex);
                int power = degree - coefficientIndex;
                for (int factor = 0; factor < power; factor++)
                {
                    MultiplyRoundedCylinderWideByWord(coefficient, segmentLengthRaw, product);
                    CopyRoundedCylinderWide(product, coefficient);
                }
            }
        }
    }

    private static bool IsRoundedCylinderEntryBeforeOrEqualToExit(
        RoundedCylinderQuadraticBound entry,
        RoundedCylinderQuadraticBound exit)
    {
        Signed576 cross = WideArithmetic.SubtractSigned576(
            WideArithmetic.MultiplySigned320(entry.Numerator, exit.Denominator),
            WideArithmetic.MultiplySigned320(exit.Numerator, entry.Denominator));
        bool hasEntryRadical = entry.RadicalSign < 0;
        bool hasExitRadical = exit.RadicalSign > 0;
        if (!hasEntryRadical && !hasExitRadical)
            return cross.Sign <= 0;
        if (cross.Sign <= 0)
            return true;

        Span<ulong> crossMagnitude = stackalloc ulong[RoundedCylinderWideLimbCount];
        Span<ulong> entryTerm = stackalloc ulong[RoundedCylinderWideLimbCount];
        Span<ulong> exitTerm = stackalloc ulong[RoundedCylinderWideLimbCount];
        Span<ulong> crossSquared = stackalloc ulong[RoundedCylinderWideLimbCount];
        Span<ulong> termSum = stackalloc ulong[RoundedCylinderWideLimbCount];
        Span<ulong> difference = stackalloc ulong[RoundedCylinderWideLimbCount];
        Span<ulong> differenceSquared = stackalloc ulong[RoundedCylinderWideLimbCount];
        Span<ulong> termProduct = stackalloc ulong[RoundedCylinderWideLimbCount];
        Span<ulong> right = stackalloc ulong[RoundedCylinderWideLimbCount];
        Span<ulong> first = stackalloc ulong[RoundedCylinderWideLimbCount];
        Span<ulong> second = stackalloc ulong[RoundedCylinderWideLimbCount];
        crossMagnitude.Clear();
        entryTerm.Clear();
        exitTerm.Clear();
        ImportRoundedCylinderWide(cross, crossMagnitude);
        if (hasEntryRadical)
        {
            GetRoundedCylinderRadicalSquare(
                exit.Denominator,
                entry.Discriminant,
                entryTerm,
                first,
                second);
        }
        if (hasExitRadical)
        {
            GetRoundedCylinderRadicalSquare(
                entry.Denominator,
                exit.Discriminant,
                exitTerm,
                first,
                second);
        }

        MultiplyRoundedCylinderWide(crossMagnitude, crossMagnitude, crossSquared);
        AddRoundedCylinderSigned(
            entryTerm,
            GetRoundedCylinderWideLength(entryTerm) == 0 ? (sbyte)0 : (sbyte)1,
            exitTerm,
            GetRoundedCylinderWideLength(exitTerm) == 0 ? (sbyte)0 : (sbyte)1,
            termSum,
            out _);
        SubtractRoundedCylinderSigned(
            crossSquared,
            1,
            termSum,
            1,
            difference,
            out sbyte differenceSign);
        if (differenceSign <= 0)
            return true;

        MultiplyRoundedCylinderWide(difference, difference, differenceSquared);
        MultiplyRoundedCylinderWide(entryTerm, exitTerm, termProduct);
        MultiplyRoundedCylinderWideByWord(termProduct, 4UL, right);
        return CompareRoundedCylinderWide(differenceSquared, right) <= 0;
    }

    private static void GetRoundedCylinderRadicalSquare(
        Signed320 coefficient,
        Signed576 discriminant,
        Span<ulong> destination,
        Span<ulong> first,
        Span<ulong> second)
    {
        ImportRoundedCylinderWide(coefficient, first);
        MultiplyRoundedCylinderWide(first, first, second);
        ImportRoundedCylinderWide(discriminant, first);
        MultiplyRoundedCylinderWide(second, first, destination);
    }

    private static void ImportRoundedCylinderWide(Signed320 value, Span<ulong> destination)
    {
        destination.Clear();
        WideArithmetic.GetMagnitude(
            value,
            out destination[4],
            out destination[3],
            out destination[2],
            out destination[1],
            out destination[0]);
    }

    private static void ImportRoundedCylinderWide(Signed576 value, Span<ulong> destination)
    {
        destination.Clear();
        WideArithmetic.GetMagnitude(value, destination[..9]);
    }

    private static int GetRoundedCylinderSignVariations(
        Span<ulong> coefficients,
        Span<sbyte> signs,
        Span<int> degrees,
        int sequenceCount,
        ulong numerator,
        ulong denominator)
    {
        int variations = 0;
        int previousSign = 0;
        for (int polynomialIndex = 0; polynomialIndex < sequenceCount; polynomialIndex++)
        {
            int sign = EvaluateRoundedCylinderPolynomialSign(
                coefficients,
                signs,
                degrees[polynomialIndex],
                polynomialIndex,
                numerator,
                denominator);
            if (sign == 0)
                continue;
            if (previousSign != 0 && sign != previousSign)
                variations++;
            previousSign = sign;
        }

        return variations;
    }

    private static int EvaluateRoundedCylinderPolynomialSign(
        Span<ulong> coefficients,
        Span<sbyte> signs,
        int degree,
        int polynomialIndex,
        ulong numerator,
        ulong denominator)
    {
        if (denominator == 1UL)
        {
            return EvaluateRoundedCylinderPolynomialSignAtInteger(
                coefficients,
                signs,
                degree,
                polynomialIndex,
                numerator);
        }

        Span<ulong> result = stackalloc ulong[RoundedCylinderWideLimbCount];
        Span<ulong> scale = stackalloc ulong[RoundedCylinderWideLimbCount];
        Span<ulong> first = stackalloc ulong[RoundedCylinderWideLimbCount];
        Span<ulong> second = stackalloc ulong[RoundedCylinderWideLimbCount];
        Span<ulong> sum = stackalloc ulong[RoundedCylinderWideLimbCount];
        result.Clear();
        scale.Clear();
        CopyRoundedCylinderWide(
            GetRoundedCylinderCoefficient(coefficients, polynomialIndex, degree),
            result);
        sbyte resultSign = signs[GetRoundedCylinderCoefficientIndex(polynomialIndex, degree)];
        scale[0] = denominator;

        for (int coefficientIndex = degree - 1; coefficientIndex >= 0; coefficientIndex--)
        {
            MultiplyRoundedCylinderWideByWord(result, numerator, first);
            sbyte firstSign = resultSign;
            MultiplyRoundedCylinderWide(
                GetRoundedCylinderCoefficient(coefficients, polynomialIndex, coefficientIndex),
                scale,
                second);
            sbyte secondSign = signs[GetRoundedCylinderCoefficientIndex(polynomialIndex, coefficientIndex)];
            AddRoundedCylinderSigned(
                first,
                firstSign,
                second,
                secondSign,
                sum,
                out resultSign);
            CopyRoundedCylinderWide(sum, result);
            MultiplyRoundedCylinderWideByWord(scale, denominator, first);
            CopyRoundedCylinderWide(first, scale);
        }

        return resultSign;
    }

    private static int EvaluateRoundedCylinderPolynomialSignAtInteger(
        Span<ulong> coefficients,
        Span<sbyte> signs,
        int degree,
        int polynomialIndex,
        ulong parameter)
    {
        Span<ulong> result = stackalloc ulong[RoundedCylinderWideLimbCount];
        Span<ulong> product = stackalloc ulong[RoundedCylinderWideLimbCount];
        Span<ulong> sum = stackalloc ulong[RoundedCylinderWideLimbCount];
        result.Clear();
        CopyRoundedCylinderWide(
            GetRoundedCylinderCoefficient(coefficients, polynomialIndex, degree),
            result);
        sbyte resultSign = signs[GetRoundedCylinderCoefficientIndex(polynomialIndex, degree)];

        for (int coefficientIndex = degree - 1; coefficientIndex >= 0; coefficientIndex--)
        {
            MultiplyRoundedCylinderWideByWord(result, parameter, product);
            AddRoundedCylinderSigned(
                product,
                resultSign,
                GetRoundedCylinderCoefficient(coefficients, polynomialIndex, coefficientIndex),
                signs[GetRoundedCylinderCoefficientIndex(polynomialIndex, coefficientIndex)],
                sum,
                out resultSign);
            CopyRoundedCylinderWide(sum, result);
        }

        return resultSign;
    }

    private static int BuildRoundedCylinderSturmSequence(
        RoundedCylinderTorusPolynomial polynomial,
        Span<ulong> coefficients,
        Span<sbyte> signs,
        Span<int> degrees)
    {
        ImportRoundedCylinderCoefficient(polynomial.H0, coefficients, signs, 0, 0);
        ImportRoundedCylinderCoefficient(polynomial.H1, coefficients, signs, 0, 1);
        ImportRoundedCylinderCoefficient(polynomial.H2, coefficients, signs, 0, 2);
        ImportRoundedCylinderCoefficient(polynomial.H3, coefficients, signs, 0, 3);
        ImportRoundedCylinderCoefficient(polynomial.H4, coefficients, signs, 0, 4);
        degrees[0] = TrimRoundedCylinderPolynomial(signs, 0, 4);
        if (degrees[0] < 0)
            return 0;
        if (degrees[0] == 0)
            return 1;

        for (int coefficientIndex = 1; coefficientIndex <= degrees[0]; coefficientIndex++)
        {
            int destinationIndex = coefficientIndex - 1;
            MultiplyRoundedCylinderWideByWord(
                GetRoundedCylinderCoefficient(coefficients, 0, coefficientIndex),
                (ulong)coefficientIndex,
                GetRoundedCylinderCoefficient(coefficients, 1, destinationIndex));
            signs[GetRoundedCylinderCoefficientIndex(1, destinationIndex)] =
                signs[GetRoundedCylinderCoefficientIndex(0, coefficientIndex)];
        }
        degrees[1] = degrees[0] - 1;

        // A nonstationary authored chord makes H4 strictly positive. A
        // stationary chord was already reduced to the constant cases above.
        return BuildRoundedCylinderQuarticSturmSequence(coefficients, signs, degrees);
    }

    private static int BuildRoundedCylinderQuarticSturmSequence(
        Span<ulong> coefficients,
        Span<sbyte> signs,
        Span<int> degrees)
    {
        Span<ulong> first = stackalloc ulong[RoundedCylinderWideLimbCount];
        Span<ulong> second = stackalloc ulong[RoundedCylinderWideLimbCount];
        Span<ulong> scratch = stackalloc ulong[RoundedCylinderWideLimbCount];
        Span<ulong> result = stackalloc ulong[RoundedCylinderWideLimbCount];

        ReadOnlySpan<ulong> a = GetRoundedCylinderCoefficient(coefficients, 0, 4);
        ReadOnlySpan<ulong> b = GetRoundedCylinderCoefficient(coefficients, 0, 3);
        ReadOnlySpan<ulong> c = GetRoundedCylinderCoefficient(coefficients, 0, 2);
        ReadOnlySpan<ulong> d = GetRoundedCylinderCoefficient(coefficients, 0, 1);
        ReadOnlySpan<ulong> e = GetRoundedCylinderCoefficient(coefficients, 0, 0);
        sbyte aSign = signs[GetRoundedCylinderCoefficientIndex(0, 4)];
        sbyte bSign = signs[GetRoundedCylinderCoefficientIndex(0, 3)];
        sbyte cSign = signs[GetRoundedCylinderCoefficientIndex(0, 2)];
        sbyte dSign = signs[GetRoundedCylinderCoefficientIndex(0, 1)];
        sbyte eSign = signs[GetRoundedCylinderCoefficientIndex(0, 0)];

        // Positive-scaled negative remainder of the quartic and its derivative.
        SetRoundedCylinderScaledProduct(b, bSign, b, bSign, 3UL, first, out sbyte firstSign, scratch);
        SetRoundedCylinderScaledProduct(a, aSign, c, cSign, 8UL, second, out sbyte secondSign, scratch);
        Span<ulong> u = GetRoundedCylinderCoefficient(coefficients, 2, 2);
        SubtractRoundedCylinderSigned(first, firstSign, second, secondSign, u, out sbyte uSign);
        signs[GetRoundedCylinderCoefficientIndex(2, 2)] = uSign;

        SetRoundedCylinderProduct(b, bSign, c, cSign, first, out firstSign);
        SetRoundedCylinderScaledProduct(a, aSign, d, dSign, 6UL, second, out secondSign, scratch);
        SubtractRoundedCylinderSigned(first, firstSign, second, secondSign, result, out sbyte resultSign);
        Span<ulong> v = GetRoundedCylinderCoefficient(coefficients, 2, 1);
        MultiplyRoundedCylinderWideByWord(result, 2UL, v);
        sbyte vSign = resultSign;
        signs[GetRoundedCylinderCoefficientIndex(2, 1)] = vSign;

        SetRoundedCylinderProduct(b, bSign, d, dSign, first, out firstSign);
        SetRoundedCylinderScaledProduct(a, aSign, e, eSign, 16UL, second, out secondSign, scratch);
        Span<ulong> w = GetRoundedCylinderCoefficient(coefficients, 2, 0);
        SubtractRoundedCylinderSigned(first, firstSign, second, secondSign, w, out sbyte wSign);
        signs[GetRoundedCylinderCoefficientIndex(2, 0)] = wSign;
        degrees[2] = TrimRoundedCylinderPolynomial(signs, 2, 2);
        if (degrees[2] < 0)
            return 2;
        if (degrees[2] == 0)
        {
            RemoveRoundedCylinderCommonPowerOfTwo(coefficients, signs, 2, 0);
            return 3;
        }
        if (degrees[2] == 1)
        {
            RemoveRoundedCylinderCommonPowerOfTwo(coefficients, signs, 2, 1);
            int finalDegree = BuildRoundedCylinderNegativeRemainder(
                coefficients,
                signs,
                1,
                3,
                2,
                1,
                3);
            if (finalDegree < 0)
                return 3;

            degrees[3] = finalDegree;
            return 4;
        }
        Span<ulong> l = GetRoundedCylinderCoefficient(coefficients, 3, 1);
        Span<ulong> m = GetRoundedCylinderCoefficient(coefficients, 3, 0);

        // This factorization divides the ordinary third Sturm polynomial by
        // positive 16a. Both two-bit shifts are algebraically exact.
        SetRoundedCylinderScaledProduct(c, cSign, c, cSign, 4UL, first, out firstSign, scratch);
        AddRoundedCylinderSigned(w, wSign, first, firstSign, result, out resultSign);
        SetRoundedCylinderScaledProduct(b, bSign, d, dSign, 9UL, first, out firstSign, scratch);
        SubtractRoundedCylinderSigned(result, resultSign, first, firstSign, l, out sbyte xSign);

        SetRoundedCylinderProduct(c, cSign, d, dSign, first, out firstSign);
        SetRoundedCylinderScaledProduct(b, bSign, e, eSign, 6UL, second, out secondSign, scratch);
        SubtractRoundedCylinderSigned(first, firstSign, second, secondSign, m, out sbyte ySign);

        SetRoundedCylinderProduct(u, uSign, l, xSign, first, out firstSign);
        SetRoundedCylinderProduct(v, vSign, v, vSign, second, out secondSign);
        SubtractRoundedCylinderSigned(first, firstSign, second, secondSign, result, out resultSign);
        ShiftRoundedCylinderWideRight(result, 2);
        CopyRoundedCylinderWide(result, l);
        sbyte lSign = resultSign;
        signs[GetRoundedCylinderCoefficientIndex(3, 1)] = lSign;

        SetRoundedCylinderProduct(u, uSign, m, ySign, first, out firstSign);
        MultiplyRoundedCylinderWideByWord(first, 2UL, second);
        secondSign = firstSign;
        SetRoundedCylinderProduct(v, vSign, w, wSign, first, out firstSign);
        SubtractRoundedCylinderSigned(second, secondSign, first, firstSign, result, out resultSign);
        ShiftRoundedCylinderWideRight(result, 2);
        CopyRoundedCylinderWide(result, m);
        sbyte mSign = resultSign;
        signs[GetRoundedCylinderCoefficientIndex(3, 0)] = mSign;

        RemoveRoundedCylinderCommonPowerOfTwo(coefficients, signs, 2, 2);
        uSign = signs[GetRoundedCylinderCoefficientIndex(2, 2)];
        vSign = signs[GetRoundedCylinderCoefficientIndex(2, 1)];
        wSign = signs[GetRoundedCylinderCoefficientIndex(2, 0)];
        degrees[3] = TrimRoundedCylinderPolynomial(signs, 3, 1);
        if (degrees[3] < 0)
            return 3;
        RemoveRoundedCylinderCommonPowerOfTwo(coefficients, signs, 3, degrees[3]);
        if (degrees[3] == 0)
            return 4;

        Span<ulong> k = GetRoundedCylinderCoefficient(coefficients, 4, 0);
        // The last remainder has the quartic discriminant's sign when u != 0.
        // Factorized invariants avoid materializing its much wider magnitude.
        SetRoundedCylinderProduct(c, cSign, c, cSign, first, out firstSign);
        SetRoundedCylinderScaledProduct(b, bSign, d, dSign, 3UL, second, out secondSign, scratch);
        SubtractRoundedCylinderSigned(first, firstSign, second, secondSign, result, out resultSign);
        SetRoundedCylinderScaledProduct(a, aSign, e, eSign, 12UL, first, out firstSign, scratch);
        AddRoundedCylinderSigned(result, resultSign, first, firstSign, k, out sbyte delta0Sign);

        SetRoundedCylinderTripleProduct(c, cSign, c, cSign, c, cSign, first, out firstSign, scratch);
        MultiplyRoundedCylinderWideByWord(first, 2UL, second);
        secondSign = firstSign;
        SetRoundedCylinderTripleProduct(b, bSign, c, cSign, d, dSign, first, out firstSign, scratch);
        MultiplyRoundedCylinderWideByWord(first, 9UL, result);
        SubtractRoundedCylinderSigned(second, secondSign, result, firstSign, scratch, out secondSign);
        CopyRoundedCylinderWide(scratch, second);
        SetRoundedCylinderTripleProduct(b, bSign, b, bSign, e, eSign, first, out firstSign, scratch);
        MultiplyRoundedCylinderWideByWord(first, 27UL, result);
        AddRoundedCylinderSigned(second, secondSign, result, firstSign, scratch, out secondSign);
        CopyRoundedCylinderWide(scratch, second);
        SetRoundedCylinderTripleProduct(a, aSign, d, dSign, d, dSign, first, out firstSign, scratch);
        MultiplyRoundedCylinderWideByWord(first, 27UL, result);
        AddRoundedCylinderSigned(second, secondSign, result, firstSign, scratch, out secondSign);
        CopyRoundedCylinderWide(scratch, second);
        SetRoundedCylinderTripleProduct(a, aSign, c, cSign, e, eSign, first, out firstSign, scratch);
        MultiplyRoundedCylinderWideByWord(first, 72UL, result);
        SubtractRoundedCylinderSigned(second, secondSign, result, firstSign, scratch, out secondSign);
        CopyRoundedCylinderWide(scratch, second);

        SetRoundedCylinderTripleProduct(k, delta0Sign, k, delta0Sign, k, delta0Sign, first, out firstSign, scratch);
        MultiplyRoundedCylinderWideByWord(first, 4UL, result);
        SetRoundedCylinderProduct(second, secondSign, second, secondSign, scratch, out sbyte delta1SquaredSign);
        SubtractRoundedCylinderSigned(result, firstSign, scratch, delta1SquaredSign, first, out sbyte kSign);
        k.Clear();
        if (kSign != 0)
            k[0] = 1UL;
        signs[GetRoundedCylinderCoefficientIndex(4, 0)] = kSign;
        if (kSign == 0)
            return 4;
        degrees[4] = 0;
        return 5;
    }

    private static void SetRoundedCylinderProduct(
        ReadOnlySpan<ulong> left,
        sbyte leftSign,
        ReadOnlySpan<ulong> right,
        sbyte rightSign,
        Span<ulong> destination,
        out sbyte destinationSign)
    {
        MultiplyRoundedCylinderWide(left, right, destination);
        destinationSign = MultiplyRoundedCylinderSigns(leftSign, rightSign);
    }

    private static void SetRoundedCylinderScaledProduct(
        ReadOnlySpan<ulong> left,
        sbyte leftSign,
        ReadOnlySpan<ulong> right,
        sbyte rightSign,
        ulong scale,
        Span<ulong> destination,
        out sbyte destinationSign,
        Span<ulong> scratch)
    {
        MultiplyRoundedCylinderWide(left, right, scratch);
        MultiplyRoundedCylinderWideByWord(scratch, scale, destination);
        destinationSign = MultiplyRoundedCylinderSigns(leftSign, rightSign);
    }

    private static void SetRoundedCylinderTripleProduct(
        ReadOnlySpan<ulong> first,
        sbyte firstSign,
        ReadOnlySpan<ulong> second,
        sbyte secondSign,
        ReadOnlySpan<ulong> third,
        sbyte thirdSign,
        Span<ulong> destination,
        out sbyte destinationSign,
        Span<ulong> scratch)
    {
        MultiplyRoundedCylinderWide(first, second, scratch);
        MultiplyRoundedCylinderWide(scratch, third, destination);
        destinationSign = MultiplyRoundedCylinderSigns(
            MultiplyRoundedCylinderSigns(firstSign, secondSign),
            thirdSign);
    }

    private static int BuildRoundedCylinderNegativeRemainder(
        Span<ulong> coefficients,
        Span<sbyte> signs,
        int dividendIndex,
        int dividendDegree,
        int divisorIndex,
        int divisorDegree,
        int destinationIndex)
    {
        for (int coefficientIndex = 0; coefficientIndex <= dividendDegree; coefficientIndex++)
        {
            CopyRoundedCylinderWide(
                GetRoundedCylinderCoefficient(coefficients, dividendIndex, coefficientIndex),
                GetRoundedCylinderCoefficient(coefficients, destinationIndex, coefficientIndex));
            signs[GetRoundedCylinderCoefficientIndex(destinationIndex, coefficientIndex)] =
                signs[GetRoundedCylinderCoefficientIndex(dividendIndex, coefficientIndex)];
        }

        Span<ulong> leadingRemainder = stackalloc ulong[RoundedCylinderWideLimbCount];
        Span<ulong> leadingDivisor = stackalloc ulong[RoundedCylinderWideLimbCount];
        Span<ulong> first = stackalloc ulong[RoundedCylinderWideLimbCount];
        Span<ulong> second = stackalloc ulong[RoundedCylinderWideLimbCount];
        Span<ulong> difference = stackalloc ulong[RoundedCylinderWideLimbCount];
        CopyRoundedCylinderWide(
            GetRoundedCylinderCoefficient(coefficients, divisorIndex, divisorDegree),
            leadingDivisor);
        sbyte leadingDivisorSign = signs[GetRoundedCylinderCoefficientIndex(divisorIndex, divisorDegree)];
        int remainderDegree = dividendDegree;
        int multiplicationCount = 0;
        while (remainderDegree >= divisorDegree)
        {
            CopyRoundedCylinderWide(
                GetRoundedCylinderCoefficient(coefficients, destinationIndex, remainderDegree),
                leadingRemainder);
            sbyte leadingRemainderSign = signs[
                GetRoundedCylinderCoefficientIndex(destinationIndex, remainderDegree)];
            int offset = remainderDegree - divisorDegree;

            for (int coefficientIndex = 0; coefficientIndex <= remainderDegree; coefficientIndex++)
            {
                Span<ulong> remainderCoefficient = GetRoundedCylinderCoefficient(
                    coefficients,
                    destinationIndex,
                    coefficientIndex);
                MultiplyRoundedCylinderWide(
                    leadingDivisor,
                    remainderCoefficient,
                    first);
                sbyte firstSign = MultiplyRoundedCylinderSigns(
                    leadingDivisorSign,
                    signs[GetRoundedCylinderCoefficientIndex(destinationIndex, coefficientIndex)]);

                int divisorCoefficientIndex = coefficientIndex - offset;
                sbyte secondSign;
                if (divisorCoefficientIndex >= 0 && divisorCoefficientIndex <= divisorDegree)
                {
                    MultiplyRoundedCylinderWide(
                        leadingRemainder,
                        GetRoundedCylinderCoefficient(coefficients, divisorIndex, divisorCoefficientIndex),
                        second);
                    secondSign = MultiplyRoundedCylinderSigns(
                        leadingRemainderSign,
                        signs[GetRoundedCylinderCoefficientIndex(divisorIndex, divisorCoefficientIndex)]);
                }
                else
                {
                    second.Clear();
                    secondSign = 0;
                }

                SubtractRoundedCylinderSigned(
                    first,
                    firstSign,
                    second,
                    secondSign,
                    difference,
                    out sbyte differenceSign);
                CopyRoundedCylinderWide(difference, remainderCoefficient);
                signs[GetRoundedCylinderCoefficientIndex(destinationIndex, coefficientIndex)] = differenceSign;
            }

            multiplicationCount++;
            remainderDegree = TrimRoundedCylinderPolynomial(
                signs,
                destinationIndex,
                remainderDegree - 1);
            if (remainderDegree < 0)
                return -1;
        }

        bool pseudoScaleIsPositive = leadingDivisorSign > 0 || (multiplicationCount & 1) == 0;
        if (pseudoScaleIsPositive)
        {
            for (int coefficientIndex = 0; coefficientIndex <= remainderDegree; coefficientIndex++)
            {
                int signIndex = GetRoundedCylinderCoefficientIndex(destinationIndex, coefficientIndex);
                signs[signIndex] = (sbyte)-signs[signIndex];
            }
        }

        RemoveRoundedCylinderCommonPowerOfTwo(
            coefficients,
            signs,
            destinationIndex,
            remainderDegree);
        return remainderDegree;
    }

    private static void RemoveRoundedCylinderCommonPowerOfTwo(
        Span<ulong> coefficients,
        Span<sbyte> signs,
        int polynomialIndex,
        int degree)
    {
        int commonShift = int.MaxValue;
        for (int coefficientIndex = 0; coefficientIndex <= degree; coefficientIndex++)
        {
            if (signs[GetRoundedCylinderCoefficientIndex(polynomialIndex, coefficientIndex)] == 0)
                continue;
            commonShift = Math.Min(
                commonShift,
                CountRoundedCylinderTrailingZeroes(
                    GetRoundedCylinderCoefficient(coefficients, polynomialIndex, coefficientIndex)));
        }

        for (int coefficientIndex = 0; coefficientIndex <= degree; coefficientIndex++)
        {
            if (signs[GetRoundedCylinderCoefficientIndex(polynomialIndex, coefficientIndex)] != 0)
            {
                ShiftRoundedCylinderWideRight(
                    GetRoundedCylinderCoefficient(coefficients, polynomialIndex, coefficientIndex),
                    commonShift);
            }
        }
    }

    private static int TrimRoundedCylinderPolynomial(
        Span<sbyte> signs,
        int polynomialIndex,
        int degree)
    {
        while (degree >= 0
            && signs[GetRoundedCylinderCoefficientIndex(polynomialIndex, degree)] == 0)
        {
            degree--;
        }
        return degree;
    }

}
