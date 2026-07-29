//=======================================================================
// WideFiniteAxisIntersection.CenteredCapsuleDistance.cs
//=======================================================================
// MIT License, Copyright (c) 2024–present David Oravsky (mrdav30)
// See LICENSE file in the project root for full license information.
//=======================================================================

namespace FixedMathSharp.Geometry;

/// <content>
/// High-precision distance computations between a point and a centered capsule
/// (a finite axis segment with a radius), using wide integer arithmetic to
/// avoid overflow and rounding error in intermediate squared-distance terms.
/// </content>
internal static partial class WideFiniteAxisIntersection
{
    internal static bool TryGetDistanceToCenteredCapsule(
        Vector2d point,
        Vector2d center,
        Vector2d axisDirection,
        Fixed64 axisLength,
        Fixed64 radius,
        out Fixed64 distance)
    {
        GetClosestCenteredAxisRatio(
            point, center, axisDirection, axisLength, out Signed192 numerator, out Signed192 denominator);
        Signed320 x = GetCenteredAxisOffsetComponent(
            point.X, center.X, axisDirection.X, numerator, denominator);
        Signed320 y = GetCenteredAxisOffsetComponent(
            point.Y, center.Y, axisDirection.Y, numerator, denominator);
        return TryGetDistanceToCenteredCapsule(
            WideArithmetic.AddSigned576(
                WideArithmetic.MultiplySigned320(x, x),
                WideArithmetic.MultiplySigned320(y, y)),
            denominator,
            radius,
            out distance);
    }

    internal static bool TryGetDistanceToCenteredCapsule(
        Vector3d point,
        Vector3d center,
        Vector3d axisDirection,
        Fixed64 axisLength,
        Fixed64 radius,
        out Fixed64 distance)
    {
        GetClosestCenteredAxisRatio(
            point, center, axisDirection, axisLength, out Signed192 numerator, out Signed192 denominator);
        Signed320 x = GetCenteredAxisOffsetComponent(
            point.X, center.X, axisDirection.X, numerator, denominator);
        Signed320 y = GetCenteredAxisOffsetComponent(
            point.Y, center.Y, axisDirection.Y, numerator, denominator);
        Signed320 z = GetCenteredAxisOffsetComponent(
            point.Z, center.Z, axisDirection.Z, numerator, denominator);
        return TryGetDistanceToCenteredCapsule(
            WideArithmetic.AddSigned576(
                WideArithmetic.AddSigned576(
                    WideArithmetic.MultiplySigned320(x, x),
                    WideArithmetic.MultiplySigned320(y, y)),
                WideArithmetic.MultiplySigned320(z, z)),
            denominator,
            radius,
            out distance);
    }

    internal static bool TryGetDistanceToCenteredCapsule(
        Signed576 squaredAxisDistanceNumerator,
        Signed192 axisDenominator,
        Fixed64 radius,
        out Fixed64 distance)
    {
        Signed320 radiusAxis = WideArithmetic.MultiplySigned192(
            Signed192.Signed(radius.m_rawValue),
            axisDenominator);
        Signed576 radiusAxisSquared = WideArithmetic.MultiplySigned320(radiusAxis, radiusAxis);
        if (WideArithmetic.CompareNonNegative(
                squaredAxisDistanceNumerator,
                radiusAxisSquared) <= 0)
        {
            distance = Fixed64.Zero;
            return true;
        }

        Signed320 scaledAxisDistance = WideArithmetic.GetFloorSquareRootScaledByFixed64(
            squaredAxisDistanceNumerator);
        Signed320 radiusDistance = MultiplyThreeToSigned320(
            Signed192.Signed(radius.m_rawValue),
            axisDenominator,
            ParameterScale);
        Signed320 approximateSurfaceDistance = WideArithmetic.SubtractSigned320(
            scaledAxisDistance,
            radiusDistance);
        if (approximateSurfaceDistance.Sign <= 0)
        {
            distance = Fixed64.Zero;
            return true;
        }

        Signed320 scale = MultiplyThreeToSigned320(axisDenominator, ParameterScale, ParameterScale);
        if (!Fixed64.TryGetSignedRatio(approximateSurfaceDistance, scale, out distance))
        {
            distance = Fixed64.MaxValue;
            return false;
        }

        // A MaxValue candidate cannot require correction: every exact
        // half-raw midpoint maps to an integer restoring-root numerator. If
        // the floored numerator is below that midpoint, the exact root is not
        // above it; at or above it, TryGetSignedRatio already rejects the
        // positive overflow tie.
        if (distance == Fixed64.MaxValue)
            return true;

        // The restoring root is floor(sqrt(N) * S), so the approximate gap is
        // below the exact gap by less than one/(axisDenominator * S) raw unit.
        // Its half-even result can therefore only remain unchanged or advance
        // by one. Compare the exact squared distance with the midpoint above
        // the candidate; importantly, radius participates before the parity
        // decision because subtracting an odd raw radius reverses tie parity.
        ulong axisDistanceCandidateRaw = unchecked((ulong)radius.m_rawValue)
            + unchecked((ulong)distance.m_rawValue);
        Signed192 doubledMidpoint = new(
            0UL,
            axisDistanceCandidateRaw >> 63,
            (axisDistanceCandidateRaw << 1) | 1UL);
        Signed320 midpointNumerator = WideArithmetic.MultiplySigned192(
            axisDenominator,
            doubledMidpoint);
        Signed576 midpointSquared = WideArithmetic.MultiplySigned320(
            midpointNumerator,
            midpointNumerator);
        Signed576 fourSquaredDistance = WideArithmetic.AddSigned576(
            WideArithmetic.AddSigned576(
                squaredAxisDistanceNumerator,
                squaredAxisDistanceNumerator),
            WideArithmetic.AddSigned576(
                squaredAxisDistanceNumerator,
                squaredAxisDistanceNumerator));
        int midpointComparison = WideArithmetic.CompareNonNegative(
            fourSquaredDistance,
            midpointSquared);
        if (midpointComparison <= 0)
            return true;

        distance = Fixed64.FromRaw(distance.m_rawValue + 1L);
        return true;
    }
}
