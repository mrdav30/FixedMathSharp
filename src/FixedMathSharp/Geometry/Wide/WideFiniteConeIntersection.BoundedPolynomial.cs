//=======================================================================
// WideFiniteConeIntersection.BoundedPolynomial.cs
//=======================================================================
// MIT License, Copyright (c) 2024–present David Oravsky (mrdav30)
// See LICENSE file in the project root for full license information.
//=======================================================================

namespace FixedMathSharp.Geometry;

/// <content>
/// Bounded-unit-interval overloads for evaluating and solving the cone polynomial
/// and its derivative, wrapping the core solver with rational parameter bounds.
/// </content>
internal static partial class WideFiniteConeIntersection
{
    internal static int EvaluateBoundedUnitPolynomialSign(
        Signed576 coefficient,
        Signed576 projection,
        Signed576 constant,
        Signed192 numerator,
        Signed192 denominator) =>
        Evaluate(
            new ConeData(
                default,
                default,
                default,
                coefficient,
                projection,
                constant),
            numerator,
            denominator).Sign;

    internal static int EvaluateBoundedUnitPolynomialDerivativeSign(
        Signed576 coefficient,
        Signed576 projection,
        Signed192 numerator,
        Signed192 denominator) =>
        EvaluateDerivative(
            new ConeData(
                default,
                default,
                default,
                coefficient,
                projection,
                default),
            new RationalBound(numerator, denominator)).Sign;

    internal static bool TrySolveBoundedUnitPolynomial(
        Signed576 coefficient,
        Signed576 projection,
        Signed576 constant,
        Signed192 lowerNumerator,
        Signed192 lowerDenominator,
        Signed192 upperNumerator,
        Signed192 upperDenominator,
        Fixed64 outputScale,
        out Fixed64 entry,
        out Fixed64 exit) =>
        TrySolveBoundedPolynomial(
            new ConeData(
                default,
                default,
                default,
                coefficient,
                projection,
                constant),
            new RationalBound(lowerNumerator, lowerDenominator),
            new RationalBound(upperNumerator, upperDenominator),
            outputScale,
            out entry,
            out exit);
}
