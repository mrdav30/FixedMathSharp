//=======================================================================
// WideVector2dTransform.PointAnchorResponse.cs
//=======================================================================
// MIT License, Copyright (c) 2024–present David Oravsky (mrdav30)
// See LICENSE file in the project root for full license information.
//=======================================================================

using FixedMathSharp.Geometry;

namespace FixedMathSharp;

/// <content>
/// Exact 2D point-anchor response products.
/// </content>
internal static partial class WideVector2dTransform
{
    internal static FixedLever2d GetLever(
        in FixedPointAnchor2d first,
        in FixedPointAnchor2d second)
    {
        GetExactRelativeOffsetRatio(
            first.Origin,
            first.LocalPoint,
            first.LocalDisplacement,
            first.ExactLocalTerm,
            first.Rotation,
            second.Origin,
            second.LocalPoint,
            second.LocalDisplacement,
            second.ExactLocalTerm,
            second.Rotation,
            out Signed320 x,
            out Signed320 y,
            out Signed320 denominator);
        return new FixedLever2d(x, y, denominator);
    }

    internal static int CompareSquaredDistances(
        in FixedPointAnchor2d reference,
        in FixedPointAnchor2d first,
        in FixedPointAnchor2d second)
    {
        Signed576 firstSquaredDistance = GetSquaredDistanceNumerator(
            reference,
            first);
        Signed576 secondSquaredDistance = GetSquaredDistanceNumerator(
            reference,
            second);
        return WideArithmetic.CompareNonNegative(
            firstSquaredDistance,
            secondSquaredDistance);
    }

    internal static bool TryGetLeverVector(
        in FixedLever2d lever,
        out Vector2d vector)
    {
        Signed576 denominator = Signed576.ExtendValue(
            lever.Denominator);
        bool representable = Fixed64.TryGetSignedRawRatio(
                Signed576.ExtendValue(lever.XNumerator),
                denominator,
                out Fixed64 x)
            & Fixed64.TryGetSignedRawRatio(
                Signed576.ExtendValue(lever.YNumerator),
                denominator,
                out Fixed64 y);
        vector = representable ? new Vector2d(x, y) : default;
        return representable;
    }

    internal static bool TryGetScaledCrossProduct(
        in FixedLever2d lever,
        Vector2d vector,
        Fixed64 firstMultiplier,
        Fixed64 secondMultiplier,
        Fixed64 divisor,
        out Fixed64 crossProduct)
    {
        Signed576 numerator = GetCrossProductNumerator(
            lever.XNumerator,
            lever.YNumerator,
            vector);
        Signed576 denominator = WideArithmetic.MultiplySigned320(
            lever.Denominator,
            Signed320.ExtendValue(Signed192.One));
        Signed320 numeratorScale = WideArithmetic.MultiplySigned192(
            Signed192.Raw(firstMultiplier),
            Signed192.Raw(secondMultiplier));
        Signed320 denominatorScale = WideArithmetic.MultiplySigned192(
            Signed192.One,
            Signed192.Raw(divisor));
        return Fixed64.TryGetSignedRawRatio(
            WideArithmetic.MultiplySigned576ToSigned704(
                numerator,
                numeratorScale),
            WideArithmetic.MultiplySigned576ToSigned704(
                denominator,
                denominatorScale),
            out crossProduct);
    }

    internal static bool TryGetScaledSquaredCrossProduct(
        in FixedLever2d lever,
        Vector2d vector,
        Fixed64 scale,
        out Fixed64 squaredCrossProduct)
    {
        Signed576 crossNumerator = GetCrossProductNumerator(
            lever.XNumerator,
            lever.YNumerator,
            vector);
        Signed576 crossDenominator = WideArithmetic.MultiplySigned320(
            lever.Denominator,
            Signed320.ExtendValue(Signed192.One));
        Signed832 numerator = WideArithmetic.MultiplySigned832(
            WideArithmetic.MultiplySigned576ToSigned832(
                crossNumerator,
                crossNumerator),
            Signed192.Raw(scale));
        Signed832 denominator =
            WideArithmetic.MultiplySigned576ToSigned832(
                crossDenominator,
                crossDenominator);
        denominator = WideArithmetic.MultiplySigned832(
            WideArithmetic.MultiplySigned832(
                denominator,
                Signed192.One),
            Signed192.One);
        return Fixed64.TryGetSignedRawRatio(
            numerator,
            denominator,
            0,
            out squaredCrossProduct);
    }

    private static Signed576 GetCrossProductNumerator(
        Signed320 x,
        Signed320 y,
        Vector2d vector) =>
        WideArithmetic.SubtractSigned576(
            WideArithmetic.MultiplySigned320(
                x,
                Signed320.ExtendValue(
                    Signed192.Raw(vector.Y))),
            WideArithmetic.MultiplySigned320(
                y,
                Signed320.ExtendValue(
                    Signed192.Raw(vector.X))));

    private static Signed576 GetSquaredDistanceNumerator(
        in FixedPointAnchor2d reference,
        in FixedPointAnchor2d point)
    {
        GetExactRelativeOffsetRatio(
            point.Origin,
            point.LocalPoint,
            point.LocalDisplacement,
            point.ExactLocalTerm,
            point.Rotation,
            reference.Origin,
            reference.LocalPoint,
            reference.LocalDisplacement,
            reference.ExactLocalTerm,
            reference.Rotation,
            out Signed320 x,
            out Signed320 y,
            // Every point-anchor term uses the same fixed denominator, so the
            // squared coordinate numerators can be compared directly.
            out _);
        return WideArithmetic.AddSigned576(
            WideArithmetic.MultiplySigned320(x, x),
            WideArithmetic.MultiplySigned320(y, y));
    }

    private static void GetExactRelativeOffsetRatio(
        Vector2d firstOrigin,
        Vector2d firstLocalPoint,
        Vector2d firstLocalDisplacement,
        FixedPointAnchorTerm2d firstExactLocalTerm,
        Fixed64 firstAngleInRadians,
        Vector2d secondOrigin,
        Vector2d secondLocalPoint,
        Vector2d secondLocalDisplacement,
        FixedPointAnchorTerm2d secondExactLocalTerm,
        Fixed64 secondAngleInRadians,
        out Signed320 x,
        out Signed320 y,
        out Signed320 denominator)
    {
        GetExactCoordinates(
            firstOrigin,
            firstLocalPoint,
            firstLocalDisplacement,
            firstExactLocalTerm,
            FixedMath.Cos(firstAngleInRadians),
            FixedMath.Sin(firstAngleInRadians),
            out Signed320 firstX,
            out Signed320 firstY);
        GetExactCoordinates(
            secondOrigin,
            secondLocalPoint,
            secondLocalDisplacement,
            secondExactLocalTerm,
            FixedMath.Cos(secondAngleInRadians),
            FixedMath.Sin(secondAngleInRadians),
            out Signed320 secondX,
            out Signed320 secondY);
        x = WideArithmetic.SubtractSigned320(firstX, secondX);
        y = WideArithmetic.SubtractSigned320(firstY, secondY);
        denominator = GetExactAnchorDenominator();
    }
}
