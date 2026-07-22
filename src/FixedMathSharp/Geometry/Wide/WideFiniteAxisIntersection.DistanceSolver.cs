//=======================================================================
// WideFiniteAxisIntersection.DistanceSolver.cs
//=======================================================================
// MIT License, Copyright (c) 2024–present David Oravsky (mrdav30)
// See LICENSE file in the project root for full license information.
//=======================================================================

namespace FixedMathSharp.Bounds;

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
        Signed320 one = WideArithmetic.ExtendToSigned320(One);
        return TrySolveBoundedQuadraticAtDistance(
            coefficient,
            projection,
            constant,
            new RationalBound320(default, one),
            new RationalBound320(one, one),
            segmentLength,
            out entryDistance,
            out exitDistance);
    }

    private static bool TrySolveBoundedQuadraticAtDistance(
        Signed320 coefficient,
        Signed320 projection,
        Signed320 constant,
        RationalBound320 lower,
        RationalBound320 upper,
        Fixed64 segmentLength,
        out Fixed64 entryDistance,
        out Fixed64 exitDistance)
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

        Signed704 lowerValue = EvaluatePolynomial(
            coefficient,
            projection,
            constant,
            lower.Numerator,
            lower.Denominator);
        Signed704 upperValue = EvaluatePolynomial(
            coefficient,
            projection,
            constant,
            upper.Numerator,
            upper.Denominator);
        int lowerDerivative = EvaluateDerivative(
            coefficient,
            projection,
            lower.Numerator,
            lower.Denominator).Sign;
        int upperDerivative = EvaluateDerivative(
            coefficient,
            projection,
            upper.Numerator,
            upper.Denominator).Sign;

        if ((lowerValue.Sign > 0 && lowerDerivative >= 0)
            || (upperValue.Sign > 0 && upperDerivative <= 0))
        {
            return false;
        }

        if (lowerValue.Sign <= 0 && upperValue.Sign <= 0)
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
        entryDistance = lowerValue.Sign <= 0
            ? RoundAtDistance(lower, segmentLength)
            : RoundLowerRootAtDistance(
                coefficient,
                projection,
                constant,
                scaledSquareRoot,
                segmentLength);
        exitDistance = upperValue.Sign <= 0
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
        Signed320 lengthRaw = WideArithmetic.ExtendToSigned320(
            WideArithmetic.FromSignedRaw(segmentLength.m_rawValue));
        Signed576 negativeScaledProjection = WideArithmetic.SubtractSigned576(
            default,
            WideArithmetic.MultiplySigned320(projection, lengthRaw));
        Signed576 numerator = WideArithmetic.SubtractSigned576(
            negativeScaledProjection,
            scaledSquareRoot);
        Fixed64.TryGetSignedRawRatio(
            numerator,
            WideArithmetic.ExtendToSigned576(coefficient),
            out Fixed64 candidate);

        long upperRaw = candidate.m_rawValue;
        if (upperRaw == 0L)
            return Fixed64.Zero;

        Signed320 upper = WideArithmetic.ExtendToSigned320(
            WideArithmetic.FromSignedRaw(upperRaw));
        Signed320 midpoint = WideArithmetic.SubtractSigned320(
            WideArithmetic.AddSigned320(upper, upper),
            WideArithmetic.ExtendToSigned320(One));
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
        Signed320 lengthRaw = WideArithmetic.ExtendToSigned320(
            WideArithmetic.FromSignedRaw(segmentLength.m_rawValue));
        Signed576 negativeScaledProjection = WideArithmetic.SubtractSigned576(
            default,
            WideArithmetic.MultiplySigned320(projection, lengthRaw));
        Signed576 numerator = WideArithmetic.AddSigned576(
            negativeScaledProjection,
            scaledSquareRoot);
        Fixed64.TryGetSignedRawRatio(
            numerator,
            WideArithmetic.ExtendToSigned576(coefficient),
            out Fixed64 candidate);

        long lowerRaw = candidate.m_rawValue;
        if (lowerRaw == segmentLength.m_rawValue)
            return segmentLength;

        Signed320 lower = WideArithmetic.ExtendToSigned320(
            WideArithmetic.FromSignedRaw(lowerRaw));
        Signed320 midpoint = WideArithmetic.AddSigned320(
            WideArithmetic.AddSigned320(lower, lower),
            WideArithmetic.ExtendToSigned320(One));
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
        Signed320 lengthRaw = WideArithmetic.ExtendToSigned320(
            WideArithmetic.FromSignedRaw(segmentLength.m_rawValue));
        Fixed64.TryGetSignedRawRatio(
            WideArithmetic.MultiplySigned320(value.Numerator, lengthRaw),
            WideArithmetic.ExtendToSigned576(value.Denominator),
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
