//=======================================================================
// WideFiniteAxisIntersection.DistanceSolver.cs
//=======================================================================
// MIT License, Copyright (c) 2024–present David Oravsky (mrdav30)
// See LICENSE file in the project root for full license information.
//=======================================================================

namespace FixedMathSharp.Geometry;

/// <content>
/// Solves bounded quadratic intersection problems and converts the resulting
/// parametric roots into actual distances along a segment of a given length.
/// </content>
internal static partial class WideFiniteAxisIntersection
{
    private static bool TrySolveUnitQuadraticAtDistance(
        Signed320 coefficient,
        Signed320 projection,
        Signed320 constant,
        Fixed64 segmentLength,
        out Fixed64 entryDistance,
        out Fixed64 exitDistance)
    {
        Signed320 one = Scale320;
        return TrySolveBoundedQuadraticAtDistance(
            coefficient,
            projection,
            constant,
            new RationalBound320(default, one),
            new RationalBound320(one, one),
            segmentLength,
            out entryDistance,
            out exitDistance,
            unitInterval: true);
    }

    private static bool TrySolveBoundedQuadraticAtDistance(
        Signed320 coefficient,
        Signed320 projection,
        Signed320 constant,
        RationalBound320 lower,
        RationalBound320 upper,
        Fixed64 segmentLength,
        out Fixed64 entryDistance,
        out Fixed64 exitDistance,
        bool unitInterval = false)
    {
        entryDistance = default;
        exitDistance = default;

        if (coefficient.IsZero)
        {
            if (constant.Sign > 0)
                return false;

            entryDistance = RoundAtDistance(lower, segmentLength);
            exitDistance = RoundAtDistance(upper, segmentLength);
            return true;
        }

        int lowerValueSign;
        int upperValueSign;
        int lowerDerivative;
        int upperDerivative;
        if (unitInterval)
        {
            // f(t) = A*t² + 2*B*t + C on [0, 1]. Endpoint signs need
            // only sums; the general rational evaluations multiply by positive
            // denominator powers and therefore have the same signs. Widen the
            // sums so even full-width coefficients cannot overflow here.
            Signed576 wideProjection = Signed576.ExtendValue(projection);
            Signed576 coefficientAndProjection = WideArithmetic.AddSigned576(
                Signed576.ExtendValue(coefficient), wideProjection);
            lowerValueSign = constant.Sign;
            upperValueSign = WideArithmetic.AddSigned576(
                WideArithmetic.AddSigned576(coefficientAndProjection, wideProjection),
                Signed576.ExtendValue(constant)).Sign;
            lowerDerivative = projection.Sign;
            upperDerivative = coefficientAndProjection.Sign;
        }
        else
        {
            lowerValueSign = EvaluatePolynomial(
                coefficient,
                projection,
                constant,
                lower.Numerator,
                lower.Denominator).Sign;
            upperValueSign = EvaluatePolynomial(
                coefficient,
                projection,
                constant,
                upper.Numerator,
                upper.Denominator).Sign;
            lowerDerivative = EvaluateDerivative(
                coefficient,
                projection,
                lower.Numerator,
                lower.Denominator).Sign;
            upperDerivative = EvaluateDerivative(
                coefficient,
                projection,
                upper.Numerator,
                upper.Denominator).Sign;
        }

        if ((lowerValueSign > 0 && lowerDerivative >= 0)
            || (upperValueSign > 0 && upperDerivative <= 0))
        {
            return false;
        }

        if (lowerValueSign <= 0 && upperValueSign <= 0)
        {
            entryDistance = RoundAtDistance(lower, segmentLength);
            exitDistance = RoundAtDistance(upper, segmentLength);
            return true;
        }

        Signed576 discriminant = WideArithmetic.SubtractSigned576(
            WideArithmetic.MultiplySigned320(projection, projection),
            WideArithmetic.MultiplySigned320(coefficient, constant));
        if (discriminant.Sign < 0)
            return false;

        Signed192 lengthSquared = SquareRaw(segmentLength.m_rawValue);
        Signed576 scaledSquareRoot = WideArithmetic.GetFloorSquareRoot(
            WideArithmetic.MultiplyNonNegative(discriminant, lengthSquared));
        entryDistance = lowerValueSign <= 0
            ? RoundAtDistance(lower, segmentLength)
            : RoundLowerRootAtDistance(
                coefficient,
                projection,
                constant,
                scaledSquareRoot,
                segmentLength);
        exitDistance = upperValueSign <= 0
            ? RoundAtDistance(upper, segmentLength)
            : RoundUpperRootAtDistance(
                coefficient,
                projection,
                constant,
                scaledSquareRoot,
                segmentLength);
        return true;
    }

    private static Fixed64 RoundLowerRootAtDistance(
        Signed320 coefficient,
        Signed320 projection,
        Signed320 constant,
        Signed576 scaledSquareRoot,
        Fixed64 segmentLength)
    {
        Signed320 lengthRaw = Signed320.ExtendValue(
            Signed192.Signed(segmentLength.m_rawValue));
        Signed576 negativeScaledProjection = WideArithmetic.SubtractSigned576(
            default,
            WideArithmetic.MultiplySigned320(projection, lengthRaw));
        Signed576 numerator = WideArithmetic.SubtractSigned576(
            negativeScaledProjection,
            scaledSquareRoot);
        Fixed64.TryGetSignedRawRatio(
            numerator,
            Signed576.ExtendValue(coefficient),
            out Fixed64 candidate);

        long upperRaw = candidate.m_rawValue;
        if (upperRaw == 0L)
            return Fixed64.Zero;

        Signed320 upper = Signed320.ExtendValue(
            Signed192.Signed(upperRaw));
        Signed320 midpoint = WideArithmetic.SubtractSigned320(
            WideArithmetic.AddSigned320(upper, upper),
            Scale320);
        Signed320 doubleLength = WideArithmetic.AddSigned320(lengthRaw, lengthRaw);
        Signed704 value = EvaluatePolynomial(
            coefficient,
            projection,
            constant,
            midpoint,
            doubleLength);
        if (value.IsZero)
            return candidate;

        return Fixed64.FromRaw(value.Sign > 0
            ? upperRaw
            : upperRaw - 1L);
    }

    private static Fixed64 RoundUpperRootAtDistance(
        Signed320 coefficient,
        Signed320 projection,
        Signed320 constant,
        Signed576 scaledSquareRoot,
        Fixed64 segmentLength)
    {
        Signed320 lengthRaw = Signed320.ExtendValue(
            Signed192.Signed(segmentLength.m_rawValue));
        Signed576 negativeScaledProjection = WideArithmetic.SubtractSigned576(
            default,
            WideArithmetic.MultiplySigned320(projection, lengthRaw));
        Signed576 numerator = WideArithmetic.AddSigned576(
            negativeScaledProjection,
            scaledSquareRoot);
        Fixed64.TryGetSignedRawRatio(
            numerator,
            Signed576.ExtendValue(coefficient),
            out Fixed64 candidate);

        long lowerRaw = candidate.m_rawValue;
        if (lowerRaw == segmentLength.m_rawValue)
            return segmentLength;

        Signed320 lower = Signed320.ExtendValue(
            Signed192.Signed(lowerRaw));
        Signed320 midpoint = WideArithmetic.AddSigned320(
            WideArithmetic.AddSigned320(lower, lower),
            Scale320);
        Signed320 doubleLength = WideArithmetic.AddSigned320(lengthRaw, lengthRaw);
        Signed704 value = EvaluatePolynomial(
            coefficient,
            projection,
            constant,
            midpoint,
            doubleLength);
        if (value.IsZero)
            return candidate;

        return Fixed64.FromRaw(value.Sign <= 0
            ? lowerRaw + 1L
            : lowerRaw);
    }

    private static Fixed64 RoundAtDistance(RationalBound320 value, Fixed64 segmentLength)
    {
        Signed320 lengthRaw = Signed320.ExtendValue(
            Signed192.Signed(segmentLength.m_rawValue));
        Fixed64.TryGetSignedRawRatio(
            WideArithmetic.MultiplySigned320(value.Numerator, lengthRaw),
            Signed576.ExtendValue(value.Denominator),
            out Fixed64 distance);
        return distance;
    }

    private static Signed192 SquareRaw(long raw)
    {
        ulong magnitude = (ulong)raw;
        Fixed64.Multiply64To128(magnitude, magnitude, out ulong high, out ulong low);
        return new Signed192(0UL, high, low);
    }
}
