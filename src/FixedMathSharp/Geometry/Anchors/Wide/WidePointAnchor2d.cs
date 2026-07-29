//=======================================================================
// WidePointAnchor2d.cs
//=======================================================================
// MIT License, Copyright (c) 2024–present David Oravsky (mrdav30)
// See LICENSE file in the project root for full license information.
//=======================================================================

namespace FixedMathSharp.Geometry;

/// <summary>
/// Provides methods for working with 2D point anchors that use wide fixed-point arithmetic.
/// </summary>
internal static class WidePointAnchor2d
{
    internal static bool TryGetPoint(
        Vector2d origin,
        Vector2d localPoint,
        Vector2d localDisplacement,
        FixedPointAnchorTerm2d exactLocalTerm,
        Fixed64 angleInRadians,
        out Vector2d result)
    {
        if (exactLocalTerm.IsZero)
        {
            return WideVector2dTransform.TryTransformCompositePoint(
                origin,
                localPoint,
                localDisplacement,
                angleInRadians,
                out result);
        }

        Fixed64 cosine = FixedMath.Cos(angleInRadians);
        Fixed64 sine = FixedMath.Sin(angleInRadians);
        Signed320 denominator = GetExactAnchorDenominator();
        GetExactCoordinates(
            origin,
            localPoint,
            localDisplacement,
            exactLocalTerm,
            cosine,
            sine,
            out Signed320 xNumerator,
            out Signed320 yNumerator);
        bool representable = Fixed64.TryGetSignedRawRatio(
                Signed576.ExtendValue(xNumerator),
                Signed576.ExtendValue(denominator),
                out Fixed64 x)
            & Fixed64.TryGetSignedRawRatio(
                Signed576.ExtendValue(yNumerator),
                Signed576.ExtendValue(denominator),
                out Fixed64 y);
        result = representable ? new Vector2d(x, y) : default;
        return representable;
    }

    internal static bool TryGetRelativeOffset(
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
        out Vector2d result)
    {
        if (firstExactLocalTerm.IsZero
            && secondExactLocalTerm.IsZero)
        {
            return WideVector2dTransform.TryGetRelativeOffset(
                firstOrigin,
                firstLocalPoint,
                firstLocalDisplacement,
                firstAngleInRadians,
                secondOrigin,
                secondLocalPoint,
                secondLocalDisplacement,
                secondAngleInRadians,
                out result);
        }

        GetExactRelativeOffsetRatio(
            firstOrigin,
            firstLocalPoint,
            firstLocalDisplacement,
            firstExactLocalTerm,
            firstAngleInRadians,
            secondOrigin,
            secondLocalPoint,
            secondLocalDisplacement,
            secondExactLocalTerm,
            secondAngleInRadians,
            out Signed320 xNumerator,
            out Signed320 yNumerator,
            out Signed320 coordinateDenominator);
        Signed576 denominator =
            Signed576.ExtendValue(coordinateDenominator);
        bool representable = Fixed64.TryGetSignedRawRatio(
                Signed576.ExtendValue(xNumerator),
                denominator,
                out Fixed64 x)
            & Fixed64.TryGetSignedRawRatio(
                Signed576.ExtendValue(yNumerator),
                denominator,
                out Fixed64 y);
        result = representable ? new Vector2d(x, y) : default;
        return representable;
    }

    internal static bool RepresentsSamePoint(
        in FixedPointAnchor2d first,
        in FixedPointAnchor2d second)
    {
        GetExactCoordinates(
            first.Origin,
            first.LocalPoint,
            first.LocalDisplacement,
            first.ExactLocalTerm,
            FixedMath.Cos(first.Rotation),
            FixedMath.Sin(first.Rotation),
            out Signed320 firstX,
            out Signed320 firstY);
        GetExactCoordinates(
            second.Origin,
            second.LocalPoint,
            second.LocalDisplacement,
            second.ExactLocalTerm,
            FixedMath.Cos(second.Rotation),
            FixedMath.Sin(second.Rotation),
            out Signed320 secondX,
            out Signed320 secondY);
        return firstX.Equals(secondX) && firstY.Equals(secondY);
    }

    internal static bool TryGetLocalPointIn(
        Vector2d pointOrigin,
        Vector2d pointLocalPoint,
        Vector2d pointLocalDisplacement,
        FixedPointAnchorTerm2d exactLocalTerm,
        Fixed64 pointAngleInRadians,
        Vector2d frameOrigin,
        Fixed64 frameAngleInRadians,
        out Vector2d localPoint)
    {
        if (exactLocalTerm.IsZero)
        {
            return WideVector2dTransform.TryGetLocalPointIn(
                pointOrigin,
                pointLocalPoint,
                pointLocalDisplacement,
                pointAngleInRadians,
                frameOrigin,
                frameAngleInRadians,
                out localPoint);
        }

        Fixed64 pointCosine = FixedMath.Cos(pointAngleInRadians);
        Fixed64 pointSine = FixedMath.Sin(pointAngleInRadians);
        Fixed64 frameCosine = FixedMath.Cos(frameAngleInRadians);
        Fixed64 frameSine = FixedMath.Sin(frameAngleInRadians);
        GetExactCoordinates(
            pointOrigin,
            pointLocalPoint,
            pointLocalDisplacement,
            exactLocalTerm,
            pointCosine,
            pointSine,
            out Signed320 pointX,
            out Signed320 pointY);
        Signed320 denominator = GetExactAnchorDenominator();
        Signed320 deltaX = WideArithmetic.SubtractSigned320(
            pointX,
            WideArithmetic.MultiplySigned192(
                Signed192.Raw(frameOrigin.X),
                Signed192.NarrowValue(denominator)));
        Signed320 deltaY = WideArithmetic.SubtractSigned320(
            pointY,
            WideArithmetic.MultiplySigned192(
                Signed192.Raw(frameOrigin.Y),
                Signed192.NarrowValue(denominator)));
        Signed576 xNumerator = WideArithmetic.AddSigned576(
            WideArithmetic.MultiplySigned576(
                Signed576.ExtendValue(deltaX),
                Signed192.Raw(frameCosine)),
            WideArithmetic.MultiplySigned576(
                Signed576.ExtendValue(deltaY),
                Signed192.Raw(frameSine)));
        Signed576 yNumerator = WideArithmetic.SubtractSigned576(
            WideArithmetic.MultiplySigned576(
                Signed576.ExtendValue(deltaY),
                Signed192.Raw(frameCosine)),
            WideArithmetic.MultiplySigned576(
                Signed576.ExtendValue(deltaX),
                Signed192.Raw(frameSine)));
        Signed576 inverseDenominator = WideArithmetic.MultiplySigned320(
            WideArithmetic.AddSigned320(
                WideArithmetic.MultiplySigned192(
                    Signed192.Raw(frameCosine),
                    Signed192.Raw(frameCosine)),
                WideArithmetic.MultiplySigned192(
                    Signed192.Raw(frameSine),
                    Signed192.Raw(frameSine))),
            Signed320.ExtendValue(
                FixedPointAnchorTerm3d.Denominator));
        bool representable = Fixed64.TryGetSignedRawRatio(
                xNumerator,
                inverseDenominator,
                out Fixed64 x)
            & Fixed64.TryGetSignedRawRatio(
                yNumerator,
                inverseDenominator,
                out Fixed64 y);
        localPoint = representable ? new Vector2d(x, y) : default;
        return representable;
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

    internal static void GetExactRelativeOffsetRatio(
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

    private static void GetExactCoordinates(
        Vector2d origin,
        Vector2d localPoint,
        Vector2d localDisplacement,
        FixedPointAnchorTerm2d exactLocalTerm,
        Fixed64 cosine,
        Fixed64 sine,
        out Signed320 x,
        out Signed320 y)
    {
        WideVector2dTransform.GetRotationNumerators(
            localPoint,
            localDisplacement,
            cosine,
            sine,
            out Signed320 roundedX,
            out Signed320 roundedY);
        Signed320 scaledRoundedX = Signed320.NarrowValue(
            WideArithmetic.MultiplySigned320(
                WideArithmetic.AddSigned320(
                    WideVector2dTransform.Scale(origin.X),
                    roundedX),
                Signed320.ExtendValue(
                    FixedPointAnchorTerm3d.Denominator)));
        Signed320 scaledRoundedY = Signed320.NarrowValue(
            WideArithmetic.MultiplySigned320(
                WideArithmetic.AddSigned320(
                    WideVector2dTransform.Scale(origin.Y),
                    roundedY),
                Signed320.ExtendValue(
                    FixedPointAnchorTerm3d.Denominator)));
        Signed320 residualX = WideArithmetic.SubtractSigned320(
            WideArithmetic.MultiplySigned192(
                Signed192.Signed(exactLocalTerm.X),
                Signed192.Raw(cosine)),
            WideArithmetic.MultiplySigned192(
                Signed192.Signed(exactLocalTerm.Y),
                Signed192.Raw(sine)));
        Signed320 residualY = WideArithmetic.AddSigned320(
            WideArithmetic.MultiplySigned192(
                Signed192.Signed(exactLocalTerm.X),
                Signed192.Raw(sine)),
            WideArithmetic.MultiplySigned192(
                Signed192.Signed(exactLocalTerm.Y),
                Signed192.Raw(cosine)));
        x = WideArithmetic.AddSigned320(scaledRoundedX, residualX);
        y = WideArithmetic.AddSigned320(scaledRoundedY, residualY);
    }

    private static Signed320 GetExactAnchorDenominator() =>
        WideArithmetic.MultiplySigned192(
            Signed192.One,
            FixedPointAnchorTerm3d.Denominator);
}
