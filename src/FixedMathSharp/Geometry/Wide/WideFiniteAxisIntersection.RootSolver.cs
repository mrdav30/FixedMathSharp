//=======================================================================
// WideFiniteAxisIntersection.RootSolver.cs
//=======================================================================
// MIT License, Copyright (c) 2024–present David Oravsky (mrdav30)
// See LICENSE file in the project root for full license information.
//=======================================================================

namespace FixedMathSharp.Geometry;

/// <content>
/// Root-solving helpers for the finite-axis intersection quadratic:
/// rounds candidate roots to exact fixed-point precision and evaluates
/// the polynomial/derivative using wide integer arithmetic to determine
/// correct rounding direction near the roots.
/// </content>
internal static partial class WideFiniteAxisIntersection
{
    private static Fixed64 RoundLowerRoot(
        Signed320 radialCoefficient,
        Signed320 radialProjection,
        Signed320 radialConstant,
        Signed320 scaledSquareRoot)
    {
        Signed320 scaledCoefficient = ShiftLeftByFixed64FractionalBits(radialCoefficient);
        Signed320 negativeScaledProjection = WideArithmetic.SubtractSigned320(
            default,
            ShiftLeftByFixed64FractionalBits(radialProjection));
        Signed320 numerator = WideArithmetic.SubtractSigned320(
            negativeScaledProjection,
            scaledSquareRoot);
        Fixed64 candidate = Fixed64.GetSignedRatio(numerator, scaledCoefficient);

        long upperRaw = candidate.m_rawValue;
        if (upperRaw == 0L)
            return Fixed64.Zero;

        long lowerRaw = upperRaw - 1L;
        Signed192 midpoint = Signed192.Signed((upperRaw * 2L) - 1L);
        Signed576 value = EvaluatePolynomial(
            radialCoefficient,
            radialProjection,
            radialConstant,
            midpoint,
            DoubleParameterScale);
        if (value.IsZero)
            return candidate;

        // The upward-biased lower seed differs from the exact root by less than
        // 1 / A, so this midpoint cannot cross the vertex before correction.
        return Fixed64.FromRaw(value.Sign > 0
            ? upperRaw
            : lowerRaw);
    }

    private static Fixed64 RoundUpperRoot(
        Signed320 radialCoefficient,
        Signed320 radialProjection,
        Signed320 radialConstant,
        Signed320 scaledSquareRoot,
        Fixed64 maxParameter)
    {
        Signed320 scaledCoefficient = ShiftLeftByFixed64FractionalBits(radialCoefficient);
        Signed320 negativeScaledProjection = WideArithmetic.SubtractSigned320(
            default,
            ShiftLeftByFixed64FractionalBits(radialProjection));
        Signed320 numerator = WideArithmetic.AddSigned320(
            negativeScaledProjection,
            scaledSquareRoot);
        Fixed64 candidate = Fixed64.GetSignedRatio(numerator, scaledCoefficient);

        long lowerRaw = candidate.m_rawValue;
        if (lowerRaw == maxParameter.m_rawValue)
            return maxParameter;

        long upperRaw = lowerRaw + 1L;
        Signed192 midpoint = Signed192.Signed((lowerRaw * 2L) + 1L);
        Signed576 value = EvaluatePolynomial(
            radialCoefficient,
            radialProjection,
            radialConstant,
            midpoint,
            DoubleParameterScale);
        if (value.IsZero)
            return candidate;

        // The downward-biased upper seed differs from the exact root by less
        // than 1 / A, so this midpoint cannot cross the vertex before correction.
        return Fixed64.FromRaw(value.Sign <= 0
            ? upperRaw
            : lowerRaw);
    }

    private static Signed320 ShiftLeftByFixed64FractionalBits(Signed320 value) =>
        new(
            (value.Word4 << FixedMath.SHIFT_AMOUNT_I) | (value.Word3 >> FixedMath.SHIFT_AMOUNT_I),
            (value.Word3 << FixedMath.SHIFT_AMOUNT_I) | (value.Word2 >> FixedMath.SHIFT_AMOUNT_I),
            (value.Word2 << FixedMath.SHIFT_AMOUNT_I) | (value.Word1 >> FixedMath.SHIFT_AMOUNT_I),
            (value.Word1 << FixedMath.SHIFT_AMOUNT_I) | (value.Word0 >> FixedMath.SHIFT_AMOUNT_I),
            value.Word0 << FixedMath.SHIFT_AMOUNT_I);

    private static Signed576 EvaluatePolynomial(
        Signed320 radialCoefficient,
        Signed320 radialProjection,
        Signed320 radialConstant,
        Signed192 numerator,
        Signed192 denominator)
    {
        Signed320 numeratorSquared = WideArithmetic.MultiplySigned192(numerator, numerator);
        Signed320 numeratorDenominator = WideArithmetic.MultiplySigned192(numerator, denominator);
        Signed320 denominatorSquared = WideArithmetic.MultiplySigned192(denominator, denominator);
        Signed576 first = WideArithmetic.MultiplySigned320(radialCoefficient, numeratorSquared);
        Signed576 second = WideArithmetic.MultiplySigned320(radialProjection, numeratorDenominator);
        Signed576 third = WideArithmetic.MultiplySigned320(radialConstant, denominatorSquared);
        return WideArithmetic.AddSigned576(
            WideArithmetic.AddSigned576(
                first,
                WideArithmetic.AddSigned576(second, second)),
            third);
    }

    private static Signed576 EvaluateDerivative(
        Signed320 radialCoefficient,
        Signed320 radialProjection,
        Signed192 numerator,
        Signed192 denominator) =>
        WideArithmetic.AddSigned576(
            WideArithmetic.MultiplySigned320(
                radialCoefficient,
                Signed320.ExtendValue(numerator)),
            WideArithmetic.MultiplySigned320(
                radialProjection,
                Signed320.ExtendValue(denominator)));

    private static Signed704 EvaluatePolynomial(
        Signed320 radialCoefficient,
        Signed320 radialProjection,
        Signed320 radialConstant,
        Signed320 numerator,
        Signed320 denominator)
    {
        Signed704 first = WideArithmetic.MultiplySigned320(radialCoefficient, numerator, numerator);
        Signed704 second = WideArithmetic.MultiplySigned320(radialProjection, numerator, denominator);
        Signed704 third = WideArithmetic.MultiplySigned320(radialConstant, denominator, denominator);
        return WideArithmetic.AddSigned704(
            WideArithmetic.AddSigned704(
                first,
                WideArithmetic.AddSigned704(second, second)),
            third);
    }

    private static Signed576 EvaluateDerivative(
        Signed320 radialCoefficient,
        Signed320 radialProjection,
        Signed320 numerator,
        Signed320 denominator) =>
        WideArithmetic.AddSigned576(
            WideArithmetic.MultiplySigned320(radialCoefficient, numerator),
            WideArithmetic.MultiplySigned320(radialProjection, denominator));
}
