//=======================================================================
// WideFiniteAxisIntersection.StrictCirclePolynomial.cs
//=======================================================================
// MIT License, Copyright (c) 2024-present David Oravsky (mrdav30)
// See LICENSE file in the project root for full license information.
//=======================================================================

using System;

namespace FixedMathSharp.Geometry;

/// <content>
/// Classifies a rational circle against a strict quadratic solid and axial
/// strip. Algebraic arc endpoints remain exact; no root is rounded to Fixed64.
/// </content>
internal static partial class WideFiniteAxisIntersection
{
    // Circle coefficients are below 2^540 and strip coefficients below 2^204.
    // The shared Sturm transients need <7*540+32 bits. Homogeneous evaluation
    // at a quadratic endpoint adds at most 4*204+8 bits, below 96 words.
    private const int StrictCircleWords = 96;

    private readonly struct StrictCircleBound
    {
        internal readonly Signed320 Rational;
        internal readonly Signed320 Denominator;
        internal readonly Signed576 Radicand;
        internal readonly int RadicalSign;
        internal readonly int Infinity;

        internal StrictCircleBound(int infinity)
        {
            Rational = default;
            Denominator = default;
            Radicand = default;
            RadicalSign = 0;
            Infinity = infinity;
        }

        internal StrictCircleBound(Signed320 rational, Signed320 denominator,
            Signed576 radicand = default, int radicalSign = 0)
        {
            bool negate = denominator.Sign < 0;
            Rational = negate ? WideArithmetic.SubtractSigned320(default, rational) : rational;
            Denominator = negate ? WideArithmetic.SubtractSigned320(default, denominator) : denominator;
            Radicand = radicand;
            RadicalSign = radicand.IsZero ? 0 : negate ? -radicalSign : radicalSign;
            Infinity = 0;
        }
    }

    /// <summary>
    /// Tests whether a real projective circle parameter satisfies radial &lt; 0,
    /// lower &gt; 0, and upper &gt; 0 simultaneously. Coefficients are ascending.
    /// The caller separately tests the omitted projective point at infinity.
    /// </summary>
    internal static bool HasStrictCirclePolynomialPoint(
        ReadOnlySpan<Signed832> radial,
        ReadOnlySpan<Signed320> lower,
        ReadOnlySpan<Signed320> upper)
    {
        Span<StrictCircleBound> bounds = stackalloc StrictCircleBound[6];
        bounds[0] = new StrictCircleBound(-1);
        int boundCount = 1;
        AddStrictCircleBounds(lower, bounds, ref boundCount);
        AddStrictCircleBounds(upper, bounds, ref boundCount);
        bounds[boundCount++] = new StrictCircleBound(1);

        Span<ulong> sequence = stackalloc ulong[25 * StrictCircleWords];
        Span<sbyte> signs = stackalloc sbyte[25];
        Span<int> degrees = stackalloc int[5];
        sequence.Clear();
        signs.Clear();
        degrees.Clear();
        int degree = -1;
        for (int index = 0; index < 5; index++)
        {
            WideArithmetic.GetMagnitude(radial[index], StrictCircleCoefficient(sequence, 0, index));
            signs[index] = (sbyte)radial[index].Sign;
            if (signs[index] != 0)
                degree = index;
        }
        if (degree < 0)
            return false;
        // The shared builder normalizes only positive-degree polynomials.
        int originalLeadingSign = degree > 0 ? signs[degree] : 1;
        degrees[0] = degree;
        int sequenceCount = BuildFiniteAxisPolynomialSturmSequence(sequence, signs, degrees);

        for (int index = 0; index + 1 < boundCount; index++)
        {
            StrictCircleBound left = bounds[index];
            StrictCircleBound right = bounds[index + 1];
            if (GetStrictCircleQuadraticRightSign(lower, left) <= 0
                || GetStrictCircleQuadraticRightSign(upper, left) <= 0)
            {
                continue;
            }
            if (GetStrictCirclePolynomialRightSign(sequence, signs, degree, left)
                    * originalLeadingSign < 0)
            {
                return true;
            }
            if (CountStrictCircleOddRoots(sequence, signs, degrees, sequenceCount, left, right) > 0)
                return true;
        }
        return false;
    }

    private static void AddStrictCircleBounds(ReadOnlySpan<Signed320> polynomial,
        Span<StrictCircleBound> bounds, ref int count)
    {
        Signed320 constant = polynomial[0];
        Signed320 linear = polynomial[1];
        Signed320 quadratic = polynomial[2];
        if (quadratic.IsZero)
        {
            if (!linear.IsZero)
                InsertStrictCircleBound(new StrictCircleBound(
                    WideArithmetic.SubtractSigned320(default, constant), linear), bounds, ref count);
            return;
        }
        Signed576 discriminant = WideArithmetic.SubtractSigned576(
            WideArithmetic.MultiplySigned320(linear, linear),
            WideArithmetic.MultiplySigned576(WideArithmetic.MultiplySigned320(quadratic, constant), 4L));
        if (discriminant.Sign < 0)
            return;
        Signed320 numerator = WideArithmetic.SubtractSigned320(default, linear);
        Signed320 denominator = WideArithmetic.AddSigned320(quadratic, quadratic);
        InsertStrictCircleBound(new StrictCircleBound(numerator, denominator, discriminant, -1), bounds, ref count);
        if (!discriminant.IsZero)
            InsertStrictCircleBound(new StrictCircleBound(numerator, denominator, discriminant, 1), bounds, ref count);
    }

    private static void InsertStrictCircleBound(StrictCircleBound bound,
        Span<StrictCircleBound> bounds, ref int count)
    {
        int index = count;
        while (index > 1)
        {
            int comparison = CompareStrictCircleBounds(bound, bounds[index - 1]);
            if (comparison == 0)
                return;
            if (comparison > 0)
                break;
            index--;
        }
        // Detect an existing root before moving anything: otherwise an early
        // duplicate return can discard a later bound from the counted prefix.
        for (int shift = count; shift > index; shift--)
            bounds[shift] = bounds[shift - 1];
        bounds[index] = bound;
        count++;
    }

    private static int CompareStrictCircleBounds(StrictCircleBound left, StrictCircleBound right)
    {
        Signed576 rational = WideArithmetic.SubtractSigned576(
            WideArithmetic.MultiplySigned320(left.Rational, right.Denominator),
            WideArithmetic.MultiplySigned320(right.Rational, left.Denominator));
        Signed832 rationalSquared = WideArithmetic.MultiplySigned576ToSigned832(rational, rational);
        Signed832 leftRadical = WideArithmetic.MultiplySigned576ToSigned832(
            WideArithmetic.MultiplySigned320(right.Denominator, right.Denominator), left.Radicand);
        Signed832 rightRadical = WideArithmetic.MultiplySigned576ToSigned832(
            WideArithmetic.MultiplySigned320(left.Denominator, left.Denominator), right.Radicand);
        Span<ulong> radicands = stackalloc ulong[39];
        WideArithmetic.GetMagnitude(rationalSquared, radicands[..13]);
        WideArithmetic.GetMagnitude(leftRadical, radicands.Slice(13, 13));
        WideArithmetic.GetMagnitude(rightRadical, radicands.Slice(26, 13));
        Span<int> signs = stackalloc int[3] { rational.Sign, left.RadicalSign, -right.RadicalSign };
        return WideArithmetic.GetLinearRadicalSumSign(radicands, 13, signs);
    }

    private static int GetStrictCircleQuadraticRightSign(ReadOnlySpan<Signed320> polynomial,
        StrictCircleBound bound)
    {
        Span<ulong> coefficients = stackalloc ulong[25 * StrictCircleWords];
        Span<sbyte> signs = stackalloc sbyte[25];
        coefficients.Clear();
        signs.Clear();
        int degree = -1;
        for (int index = 0; index < 3; index++)
        {
            WideArithmetic.GetMagnitude(Signed576.ExtendValue(polynomial[index]),
                StrictCircleCoefficient(coefficients, 0, index));
            signs[index] = (sbyte)polynomial[index].Sign;
            if (signs[index] != 0)
                degree = index;
        }
        return degree < 0 ? 0 : GetStrictCirclePolynomialRightSign(coefficients, signs, degree, bound);
    }

    private static int GetStrictCirclePolynomialRightSign(ReadOnlySpan<ulong> coefficients,
        ReadOnlySpan<sbyte> signs, int degree, StrictCircleBound bound) =>
        GetStrictCirclePolynomialLimitSign(coefficients, signs, 0, degree, bound, 1);

    private static int GetStrictCirclePolynomialLimitSign(ReadOnlySpan<ulong> coefficients,
        ReadOnlySpan<sbyte> signs, int polynomial, int degree, StrictCircleBound bound, int direction)
    {
        if (bound.Infinity != 0)
            return signs[polynomial * 5 + degree] * (bound.Infinity < 0 && (degree & 1) != 0 ? -1 : 1);
        // The first nonzero derivative determines the right-hand sign at an
        // endpoint root; excluded tangent roots never manufacture penetration.
        for (int derivative = 0; derivative < degree; derivative++)
        {
            int sign = EvaluateStrictCirclePolynomial(coefficients, signs, polynomial, degree, bound, derivative);
            if (sign != 0)
                return direction < 0 && (derivative & 1) != 0 ? -sign : sign;
        }
        // Every caller supplies a trimmed, nonzero polynomial. Its highest
        // derivative has the leading coefficient's sign and cannot vanish.
        int leadingSign = signs[polynomial * 5 + degree];
        return direction < 0 && (degree & 1) != 0 ? -leadingSign : leadingSign;
    }

    private static int CountStrictCircleOddRoots(ReadOnlySpan<ulong> coefficients,
        ReadOnlySpan<sbyte> signs, ReadOnlySpan<int> degrees, int sequenceCount,
        StrictCircleBound left, StrictCircleBound right)
    {
        int degree = degrees[0];
        if (degree == 0)
            return 0;
        int distinct = GetStrictCircleVariations(coefficients, signs, degrees, sequenceCount, left, 1)
            - GetStrictCircleVariations(coefficients, signs, degrees, sequenceCount, right, -1);
        int last = sequenceCount - 1;
        int gcdDegree = degrees[last];
        if (gcdDegree == 0 || distinct == 0)
            return distinct;
        if (gcdDegree == 1)
        {
            // This gcd is linear. At infinity its sign is its leading sign
            // times the endpoint direction; finite endpoint zeros stay zero.
            int leftSign = left.Infinity != 0 ? signs[last * 5 + 1] * left.Infinity
                : EvaluateStrictCirclePolynomial(coefficients, signs, last, 1, left);
            int rightSign = right.Infinity != 0 ? signs[last * 5 + 1] * right.Infinity
                : EvaluateStrictCirclePolynomial(coefficients, signs, last, 1, right);
            return distinct - (leftSign != 0 && rightSign != 0 && leftSign != rightSign ? 1 : 0);
        }
        if (gcdDegree == 3)
            return 0; // A quartic with a cubic gcd is a fourth power.
        if (degree == 3)
            return distinct; // A cubic with a quadratic gcd is a third power.

        // A quartic quadratic gcd is either two double roots (no sign changes)
        // or one triple root plus one simple root (both change sign).
        Span<ulong> squared = stackalloc ulong[2 * StrictCircleWords];
        Span<ulong> product = stackalloc ulong[2 * StrictCircleWords];
        Span<ulong> scaled = stackalloc ulong[2 * StrictCircleWords];
        ReadOnlySpan<ulong> b = StrictCircleCoefficient(coefficients, last, 1);
        MultiplyRoundedCylinderWide(b, b, squared);
        MultiplyRoundedCylinderWide(StrictCircleCoefficient(coefficients, last, 0),
            StrictCircleCoefficient(coefficients, last, 2), product);
        MultiplyRoundedCylinderWideByWord(product, 4UL, scaled);
        bool nonnegativeProduct = signs[last * 5] == 0 || signs[last * 5] == signs[last * 5 + 2];
        return nonnegativeProduct && WideArithmetic.CompareMagnitudeEqualLength(squared, scaled) == 0 ? distinct : 0;
    }

    private static int GetStrictCircleVariations(ReadOnlySpan<ulong> coefficients,
        ReadOnlySpan<sbyte> signs, ReadOnlySpan<int> degrees, int count, StrictCircleBound bound, int direction)
    {
        int previous = 0;
        int variations = 0;
        for (int index = 0; index < count; index++)
        {
            int sign = GetStrictCirclePolynomialLimitSign(coefficients, signs, index, degrees[index], bound, direction);
            if (previous != 0 && previous != sign)
                variations++;
            previous = sign;
        }
        return variations;
    }

    private static int EvaluateStrictCirclePolynomial(ReadOnlySpan<ulong> coefficients,
        ReadOnlySpan<sbyte> signs, int polynomial, int degree, StrictCircleBound bound,
        int derivative = 0)
    {
        // Both callers handle infinity before entering this finite evaluator.
        Span<ulong> rational = stackalloc ulong[StrictCircleWords];
        Span<ulong> radical = stackalloc ulong[StrictCircleWords];
        Span<ulong> denominatorPower = stackalloc ulong[StrictCircleWords];
        Span<ulong> first = stackalloc ulong[StrictCircleWords];
        Span<ulong> second = stackalloc ulong[StrictCircleWords];
        Span<ulong> third = stackalloc ulong[StrictCircleWords];
        Span<ulong> nextRational = stackalloc ulong[StrictCircleWords];
        Span<ulong> nextRadical = stackalloc ulong[StrictCircleWords];
        Span<ulong> numerator = stackalloc ulong[9];
        Span<ulong> denominator = stackalloc ulong[9];
        Span<ulong> radicand = stackalloc ulong[9];
        WideArithmetic.GetMagnitude(Signed576.ExtendValue(bound.Rational), numerator);
        WideArithmetic.GetMagnitude(Signed576.ExtendValue(bound.Denominator), denominator);
        WideArithmetic.GetMagnitude(bound.Radicand, radicand);
        rational.Clear();
        radical.Clear();
        denominatorPower.Clear();
        denominatorPower[0] = 1UL;
        sbyte rationalSign = 0;
        sbyte radicalSign = 0;
        for (int index = degree; index >= derivative; index--)
        {
            MultiplyRoundedCylinderWide(rational, numerator, first);
            MultiplyRoundedCylinderWide(radical, radicand, second);
            AddRoundedCylinderSigned(first, (sbyte)(rationalSign * bound.Rational.Sign),
                second, (sbyte)(radicalSign * bound.RadicalSign), nextRational, out sbyte nextRationalSign);
            MultiplyRoundedCylinderWide(radical, numerator, first);
            AddRoundedCylinderSigned(first, (sbyte)(radicalSign * bound.Rational.Sign),
                rational, (sbyte)(rationalSign * bound.RadicalSign), nextRadical, out sbyte nextRadicalSign);
            if (index != degree)
            {
                MultiplyRoundedCylinderWide(denominatorPower, denominator, first);
                first.CopyTo(denominatorPower);
            }
            ulong multiplier = 1UL;
            for (int order = 0; order < derivative; order++)
                multiplier *= (ulong)(index - order);
            MultiplyRoundedCylinderWide(StrictCircleCoefficient(coefficients, polynomial, index),
                denominatorPower, first);
            MultiplyRoundedCylinderWideByWord(first, multiplier, second);
            AddRoundedCylinderSigned(nextRational, nextRationalSign, second,
                signs[polynomial * 5 + index], third, out rationalSign);
            third.CopyTo(rational);
            nextRadical.CopyTo(radical);
            radicalSign = nextRadicalSign;
        }
        if (rationalSign == 0)
            return radicalSign;
        if (radicalSign == 0 || rationalSign == radicalSign)
            return rationalSign;
        Span<ulong> rationalSquared = stackalloc ulong[2 * StrictCircleWords + 9];
        Span<ulong> radicalSquared = stackalloc ulong[2 * StrictCircleWords + 9];
        Span<ulong> scaledRadicalSquared = stackalloc ulong[2 * StrictCircleWords + 9];
        MultiplyRoundedCylinderWide(rational, rational, rationalSquared);
        MultiplyRoundedCylinderWide(radical, radical, radicalSquared);
        MultiplyRoundedCylinderWide(radicalSquared, radicand, scaledRadicalSquared);
        int comparison = WideArithmetic.CompareMagnitudeEqualLength(rationalSquared, scaledRadicalSquared);
        return comparison == 0 ? 0 : comparison > 0 ? rationalSign : radicalSign;
    }

    private static Span<ulong> StrictCircleCoefficient(Span<ulong> values, int polynomial, int coefficient) =>
        values.Slice((polynomial * 5 + coefficient) * StrictCircleWords, StrictCircleWords);

    private static ReadOnlySpan<ulong> StrictCircleCoefficient(ReadOnlySpan<ulong> values, int polynomial, int coefficient) =>
        values.Slice((polynomial * 5 + coefficient) * StrictCircleWords, StrictCircleWords);
}
