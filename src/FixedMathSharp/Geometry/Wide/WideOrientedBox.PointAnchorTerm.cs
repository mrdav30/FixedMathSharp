//=======================================================================
// WideOrientedBox.PointAnchorTerm.cs
//=======================================================================
// MIT License, Copyright (c) 2024–present David Oravsky (mrdav30)
// See LICENSE file in the project root for full license information.
//=======================================================================

namespace FixedMathSharp.Geometry;

/// <content>
/// Wide-oriented box point anchor term operations.
/// </content>
internal static partial class WideOrientedBox
{
    internal static bool TryGetPoint(
        Vector3d origin,
        FixedQuaternion rotation,
        Vector3d localPoint,
        Vector3d localDisplacement,
        Vector3d localTranslation,
        FixedPointAnchorTerm3d exactLocalTerm,
        out Vector3d point)
    {
        if (exactLocalTerm.IsZero
            && localTranslation == Vector3d.Zero)
        {
            return TryGetPoint(
                origin,
                rotation,
                localPoint,
                localDisplacement,
                out point);
        }

        RationalBasis basis = new(rotation);
        Signed320 denominator = GetAnchorDenominator(basis);
        Signed576 denominatorWide =
            Signed576.ExtendValue(denominator);
        bool representable = Fixed64.TryGetSignedRawRatio(
                Signed576.ExtendValue(
                    GetAnchorCoordinateNumerator(
                        origin.X,
                        basis.Xx,
                        basis.Yx,
                        basis.Zx,
                        basis,
                        localPoint,
                        localDisplacement,
                        localTranslation,
                        exactLocalTerm)),
                denominatorWide,
                out Fixed64 x)
            & Fixed64.TryGetSignedRawRatio(
                Signed576.ExtendValue(
                    GetAnchorCoordinateNumerator(
                        origin.Y,
                        basis.Xy,
                        basis.Yy,
                        basis.Zy,
                        basis,
                        localPoint,
                        localDisplacement,
                        localTranslation,
                        exactLocalTerm)),
                denominatorWide,
                out Fixed64 y)
            & Fixed64.TryGetSignedRawRatio(
                Signed576.ExtendValue(
                    GetAnchorCoordinateNumerator(
                        origin.Z,
                        basis.Xz,
                        basis.Yz,
                        basis.Zz,
                        basis,
                        localPoint,
                        localDisplacement,
                        localTranslation,
                        exactLocalTerm)),
                denominatorWide,
                out Fixed64 z);
        point = representable ? new Vector3d(x, y, z) : default;
        return representable;
    }

    internal static bool TryGetRelativeOffset(
        Vector3d firstOrigin,
        FixedQuaternion firstRotation,
        Vector3d firstLocalPoint,
        Vector3d firstLocalDisplacement,
        Vector3d firstLocalTranslation,
        FixedPointAnchorTerm3d firstExactLocalTerm,
        Vector3d secondOrigin,
        FixedQuaternion secondRotation,
        Vector3d secondLocalPoint,
        Vector3d secondLocalDisplacement,
        Vector3d secondLocalTranslation,
        FixedPointAnchorTerm3d secondExactLocalTerm,
        Fixed64 scale,
        out Vector3d offset)
    {
        if (firstExactLocalTerm.IsZero
            && secondExactLocalTerm.IsZero
            && firstLocalTranslation == Vector3d.Zero
            && secondLocalTranslation == Vector3d.Zero)
        {
            return TryGetRelativeOffset(
                firstOrigin,
                firstRotation,
                firstLocalPoint,
                firstLocalDisplacement,
                secondOrigin,
                secondRotation,
                secondLocalPoint,
                secondLocalDisplacement,
                scale,
                out offset);
        }

        GetExactRelativeOffsetRatio(
            firstOrigin,
            firstRotation,
            firstLocalPoint,
            firstLocalDisplacement,
            firstLocalTranslation,
            firstExactLocalTerm,
            secondOrigin,
            secondRotation,
            secondLocalPoint,
            secondLocalDisplacement,
            secondLocalTranslation,
            secondExactLocalTerm,
            out Signed576 xNumerator,
            out Signed576 yNumerator,
            out Signed576 zNumerator,
            out Signed576 denominator);
        bool representable = TryGetScaledExactCoordinate(
                xNumerator,
                denominator,
                scale,
                out Fixed64 x)
            & TryGetScaledExactCoordinate(
                yNumerator,
                denominator,
                scale,
                out Fixed64 y)
            & TryGetScaledExactCoordinate(
                zNumerator,
                denominator,
                scale,
                out Fixed64 z);
        offset = representable ? new Vector3d(x, y, z) : default;
        return representable;
    }

    internal static bool RepresentsSamePoint(
        in FixedPointAnchor first,
        in FixedPointAnchor second)
    {
        GetExactRelativeOffsetRatio(
            first.Origin,
            first.Rotation,
            first.LocalPoint,
            first.LocalDisplacement,
            first.LocalTranslation,
            first.ExactLocalTerm,
            second.Origin,
            second.Rotation,
            second.LocalPoint,
            second.LocalDisplacement,
            second.LocalTranslation,
            second.ExactLocalTerm,
            out Signed576 x,
            out Signed576 y,
            out Signed576 z,
            out _);
        return x.IsZero && y.IsZero && z.IsZero;
    }

    internal static bool TryGetLocalPointIn(
        Vector3d pointOrigin,
        FixedQuaternion pointRotation,
        Vector3d pointLocalPoint,
        Vector3d pointLocalDisplacement,
        Vector3d pointLocalTranslation,
        FixedPointAnchorTerm3d exactLocalTerm,
        Vector3d frameOrigin,
        FixedQuaternion frameRotation,
        out Vector3d localPoint)
    {
        if (exactLocalTerm.IsZero
            && pointLocalTranslation == Vector3d.Zero)
        {
            return TryGetLocalPointIn(
                pointOrigin,
                pointRotation,
                pointLocalPoint,
                pointLocalDisplacement,
                frameOrigin,
                frameRotation,
                out localPoint);
        }

        RationalBasis pointBasis = new(pointRotation);
        RationalBasis frameBasis = new(frameRotation);
        Signed320 pointDenominator = GetAnchorDenominator(pointBasis);
        Signed320 deltaX = GetAnchorDeltaNumerator(
            pointOrigin.X,
            frameOrigin.X,
            pointBasis.Xx,
            pointBasis.Yx,
            pointBasis.Zx,
            pointBasis,
            pointLocalPoint,
            pointLocalDisplacement,
            pointLocalTranslation,
            exactLocalTerm,
            pointDenominator);
        Signed320 deltaY = GetAnchorDeltaNumerator(
            pointOrigin.Y,
            frameOrigin.Y,
            pointBasis.Xy,
            pointBasis.Yy,
            pointBasis.Zy,
            pointBasis,
            pointLocalPoint,
            pointLocalDisplacement,
            pointLocalTranslation,
            exactLocalTerm,
            pointDenominator);
        Signed320 deltaZ = GetAnchorDeltaNumerator(
            pointOrigin.Z,
            frameOrigin.Z,
            pointBasis.Xz,
            pointBasis.Yz,
            pointBasis.Zz,
            pointBasis,
            pointLocalPoint,
            pointLocalDisplacement,
            pointLocalTranslation,
            exactLocalTerm,
            pointDenominator);
        Signed576 denominator = WideArithmetic.MultiplySigned320(
            pointDenominator,
            Signed320.ExtendValue(frameBasis.Denominator));
        bool representable = Fixed64.TryGetSignedRawRatio(
                GetWideProjection(
                    deltaX,
                    deltaY,
                    deltaZ,
                    frameBasis.Xx,
                    frameBasis.Xy,
                    frameBasis.Xz),
                denominator,
                out Fixed64 x)
            & Fixed64.TryGetSignedRawRatio(
                GetWideProjection(
                    deltaX,
                    deltaY,
                    deltaZ,
                    frameBasis.Yx,
                    frameBasis.Yy,
                    frameBasis.Yz),
                denominator,
                out Fixed64 y)
            & Fixed64.TryGetSignedRawRatio(
                GetWideProjection(
                    deltaX,
                    deltaY,
                    deltaZ,
                    frameBasis.Zx,
                    frameBasis.Zy,
                    frameBasis.Zz),
                denominator,
                out Fixed64 z);
        localPoint = representable ? new Vector3d(x, y, z) : default;
        return representable;
    }

    internal static bool TryGetProjectedOffset(
        Vector3d firstOrigin,
        FixedQuaternion firstRotation,
        Vector3d firstLocalPoint,
        Vector3d firstLocalDisplacement,
        Vector3d firstLocalTranslation,
        FixedPointAnchorTerm3d firstExactLocalTerm,
        Vector3d secondOrigin,
        FixedQuaternion secondRotation,
        Vector3d secondLocalPoint,
        Vector3d secondLocalDisplacement,
        Vector3d secondLocalTranslation,
        FixedPointAnchorTerm3d secondExactLocalTerm,
        Vector3d direction,
        out Fixed64 projection)
    {
        if (firstExactLocalTerm.IsZero
            && secondExactLocalTerm.IsZero
            && firstLocalTranslation == Vector3d.Zero
            && secondLocalTranslation == Vector3d.Zero)
        {
            return TryGetProjectedOffset(
                firstOrigin,
                firstRotation,
                firstLocalPoint,
                firstLocalDisplacement,
                secondOrigin,
                secondRotation,
                secondLocalPoint,
                secondLocalDisplacement,
                direction,
                out projection);
        }

        GetExactProjectedOffsetRatio(
            firstOrigin,
            firstRotation,
            firstLocalPoint,
            firstLocalDisplacement,
            firstLocalTranslation,
            firstExactLocalTerm,
            secondOrigin,
            secondRotation,
            secondLocalPoint,
            secondLocalDisplacement,
            secondLocalTranslation,
            secondExactLocalTerm,
            direction,
            out Signed704 numerator,
            out Signed704 denominator);
        return Fixed64.TryGetSignedRawRatio(
            numerator,
            denominator,
            out projection);
    }

    internal static Fixed64 ProjectNonNegativeOffset(
        Vector3d firstOrigin,
        FixedQuaternion firstRotation,
        Vector3d firstLocalPoint,
        Vector3d firstLocalDisplacement,
        Vector3d firstLocalTranslation,
        FixedPointAnchorTerm3d firstExactLocalTerm,
        Vector3d secondOrigin,
        FixedQuaternion secondRotation,
        Vector3d secondLocalPoint,
        Vector3d secondLocalDisplacement,
        Vector3d secondLocalTranslation,
        FixedPointAnchorTerm3d secondExactLocalTerm,
        Vector3d direction)
    {
        if (firstExactLocalTerm.IsZero
            && secondExactLocalTerm.IsZero
            && firstLocalTranslation == Vector3d.Zero
            && secondLocalTranslation == Vector3d.Zero)
        {
            return ProjectNonNegativeOffset(
                firstOrigin,
                firstRotation,
                firstLocalPoint,
                firstLocalDisplacement,
                secondOrigin,
                secondRotation,
                secondLocalPoint,
                secondLocalDisplacement,
                direction);
        }

        GetExactProjectedOffsetRatio(
            firstOrigin,
            firstRotation,
            firstLocalPoint,
            firstLocalDisplacement,
            firstLocalTranslation,
            firstExactLocalTerm,
            secondOrigin,
            secondRotation,
            secondLocalPoint,
            secondLocalDisplacement,
            secondLocalTranslation,
            secondExactLocalTerm,
            direction,
            out Signed704 numerator,
            out Signed704 denominator);
        return Fixed64.GetNonNegativeRawRatioFloor(
            numerator,
            denominator);
    }

    private static void GetExactProjectedOffsetRatio(
        Vector3d firstOrigin,
        FixedQuaternion firstRotation,
        Vector3d firstLocalPoint,
        Vector3d firstLocalDisplacement,
        Vector3d firstLocalTranslation,
        FixedPointAnchorTerm3d firstExactLocalTerm,
        Vector3d secondOrigin,
        FixedQuaternion secondRotation,
        Vector3d secondLocalPoint,
        Vector3d secondLocalDisplacement,
        Vector3d secondLocalTranslation,
        FixedPointAnchorTerm3d secondExactLocalTerm,
        Vector3d direction,
        out Signed704 numerator,
        out Signed704 denominator)
    {
        GetExactRelativeOffsetRatio(
            firstOrigin,
            firstRotation,
            firstLocalPoint,
            firstLocalDisplacement,
            firstLocalTranslation,
            firstExactLocalTerm,
            secondOrigin,
            secondRotation,
            secondLocalPoint,
            secondLocalDisplacement,
            secondLocalTranslation,
            secondExactLocalTerm,
            out Signed576 x,
            out Signed576 y,
            out Signed576 z,
            out Signed576 coordinateDenominator);
        numerator = WideArithmetic.AddSigned704(
            WideArithmetic.AddSigned704(
                WideArithmetic.MultiplySigned576ToSigned704(
                    x,
                    Signed320.ExtendValue(Signed192.Raw(direction.X))),
                WideArithmetic.MultiplySigned576ToSigned704(
                    y,
                    Signed320.ExtendValue(Signed192.Raw(direction.Y)))),
            WideArithmetic.MultiplySigned576ToSigned704(
                z,
                Signed320.ExtendValue(Signed192.Raw(direction.Z))));
        denominator = WideArithmetic.MultiplySigned576ToSigned704(
            coordinateDenominator,
            Signed320.ExtendValue(Signed192.One));
    }

    private static void GetExactRelativeOffsetRatio(
        Vector3d firstOrigin,
        FixedQuaternion firstRotation,
        Vector3d firstLocalPoint,
        Vector3d firstLocalDisplacement,
        Vector3d firstLocalTranslation,
        FixedPointAnchorTerm3d firstExactLocalTerm,
        Vector3d secondOrigin,
        FixedQuaternion secondRotation,
        Vector3d secondLocalPoint,
        Vector3d secondLocalDisplacement,
        Vector3d secondLocalTranslation,
        FixedPointAnchorTerm3d secondExactLocalTerm,
        out Signed576 x,
        out Signed576 y,
        out Signed576 z,
        out Signed576 denominator)
    {
        RationalBasis firstBasis = new(firstRotation);
        RationalBasis secondBasis = new(secondRotation);
        Signed320 firstDenominator = GetAnchorDenominator(firstBasis);
        Signed320 secondDenominator = GetAnchorDenominator(secondBasis);
        denominator = WideArithmetic.MultiplySigned320(
            firstDenominator,
            secondDenominator);
        x = GetExactRelativeOffsetNumerator(
            firstOrigin.X,
            firstBasis.Xx,
            firstBasis.Yx,
            firstBasis.Zx,
            firstBasis,
            firstLocalPoint,
            firstLocalDisplacement,
            firstLocalTranslation,
            firstExactLocalTerm,
            firstDenominator,
            secondOrigin.X,
            secondBasis.Xx,
            secondBasis.Yx,
            secondBasis.Zx,
            secondBasis,
            secondLocalPoint,
            secondLocalDisplacement,
            secondLocalTranslation,
            secondExactLocalTerm,
            secondDenominator);
        y = GetExactRelativeOffsetNumerator(
            firstOrigin.Y,
            firstBasis.Xy,
            firstBasis.Yy,
            firstBasis.Zy,
            firstBasis,
            firstLocalPoint,
            firstLocalDisplacement,
            firstLocalTranslation,
            firstExactLocalTerm,
            firstDenominator,
            secondOrigin.Y,
            secondBasis.Xy,
            secondBasis.Yy,
            secondBasis.Zy,
            secondBasis,
            secondLocalPoint,
            secondLocalDisplacement,
            secondLocalTranslation,
            secondExactLocalTerm,
            secondDenominator);
        z = GetExactRelativeOffsetNumerator(
            firstOrigin.Z,
            firstBasis.Xz,
            firstBasis.Yz,
            firstBasis.Zz,
            firstBasis,
            firstLocalPoint,
            firstLocalDisplacement,
            firstLocalTranslation,
            firstExactLocalTerm,
            firstDenominator,
            secondOrigin.Z,
            secondBasis.Xz,
            secondBasis.Yz,
            secondBasis.Zz,
            secondBasis,
            secondLocalPoint,
            secondLocalDisplacement,
            secondLocalTranslation,
            secondExactLocalTerm,
            secondDenominator);
    }

    private static Signed576 GetExactRelativeOffsetNumerator(
        Fixed64 firstOrigin,
        Signed192 firstAxisX,
        Signed192 firstAxisY,
        Signed192 firstAxisZ,
        RationalBasis firstBasis,
        Vector3d firstLocalPoint,
        Vector3d firstLocalDisplacement,
        Vector3d firstLocalTranslation,
        FixedPointAnchorTerm3d firstExactLocalTerm,
        Signed320 firstDenominator,
        Fixed64 secondOrigin,
        Signed192 secondAxisX,
        Signed192 secondAxisY,
        Signed192 secondAxisZ,
        RationalBasis secondBasis,
        Vector3d secondLocalPoint,
        Vector3d secondLocalDisplacement,
        Vector3d secondLocalTranslation,
        FixedPointAnchorTerm3d secondExactLocalTerm,
        Signed320 secondDenominator) =>
        WideArithmetic.SubtractSigned576(
            WideArithmetic.MultiplySigned320(
                GetAnchorCoordinateNumerator(
                    firstOrigin,
                    firstAxisX,
                    firstAxisY,
                    firstAxisZ,
                    firstBasis,
                    firstLocalPoint,
                    firstLocalDisplacement,
                    firstLocalTranslation,
                    firstExactLocalTerm),
                secondDenominator),
            WideArithmetic.MultiplySigned320(
                GetAnchorCoordinateNumerator(
                    secondOrigin,
                    secondAxisX,
                    secondAxisY,
                    secondAxisZ,
                    secondBasis,
                    secondLocalPoint,
                    secondLocalDisplacement,
                    secondLocalTranslation,
                    secondExactLocalTerm),
                firstDenominator));

    private static Signed320 GetAnchorCoordinateNumerator(
        Fixed64 origin,
        Signed192 axisX,
        Signed192 axisY,
        Signed192 axisZ,
        RationalBasis basis,
        Vector3d localPoint,
        Vector3d localDisplacement,
        Vector3d localTranslation,
        FixedPointAnchorTerm3d exactLocalTerm)
    {
        Signed320 roundedNumerator = WideArithmetic.AddSigned320(
            WideArithmetic.MultiplySigned192(
                Signed192.Raw(origin),
                basis.Denominator),
            WideArithmetic.AddSigned320(
                WideArithmetic.AddSigned320(
                    GetProjection(
                        Signed192.Raw(localPoint.X),
                        Signed192.Raw(localPoint.Y),
                        Signed192.Raw(localPoint.Z),
                        axisX,
                        axisY,
                        axisZ),
                    GetProjection(
                        Signed192.Raw(localDisplacement.X),
                        Signed192.Raw(localDisplacement.Y),
                        Signed192.Raw(localDisplacement.Z),
                        axisX,
                        axisY,
                        axisZ)),
                GetProjection(
                    Signed192.Raw(localTranslation.X),
                    Signed192.Raw(localTranslation.Y),
                    Signed192.Raw(localTranslation.Z),
                    axisX,
                    axisY,
                    axisZ)));
        Signed576 scaledRounded = WideArithmetic.MultiplySigned320(
            roundedNumerator,
            Signed320.ExtendValue(
                FixedPointAnchorTerm3d.Denominator));
        Signed320 residualProjection = GetProjection(
            Signed192.Signed(exactLocalTerm.X),
            Signed192.Signed(exactLocalTerm.Y),
            Signed192.Signed(exactLocalTerm.Z),
            axisX,
            axisY,
            axisZ);
        return WideArithmetic.AddSigned320(
            Signed320.NarrowValue(scaledRounded),
            residualProjection);
    }

    private static Signed320 GetAnchorDenominator(RationalBasis basis) =>
        WideArithmetic.MultiplySigned192(
            basis.Denominator,
            FixedPointAnchorTerm3d.Denominator);

    private static Signed320 GetAnchorDeltaNumerator(
        Fixed64 pointOrigin,
        Fixed64 frameOrigin,
        Signed192 axisX,
        Signed192 axisY,
        Signed192 axisZ,
        RationalBasis pointBasis,
        Vector3d pointLocalPoint,
        Vector3d pointLocalDisplacement,
        Vector3d pointLocalTranslation,
        FixedPointAnchorTerm3d exactLocalTerm,
        Signed320 pointDenominator) =>
        WideArithmetic.SubtractSigned320(
            GetAnchorCoordinateNumerator(
                pointOrigin,
                axisX,
                axisY,
                axisZ,
                pointBasis,
                pointLocalPoint,
                pointLocalDisplacement,
                pointLocalTranslation,
                exactLocalTerm),
            WideArithmetic.MultiplySigned192(
                Signed192.Raw(frameOrigin),
                Signed192.NarrowValue(pointDenominator)));

    private static Signed576 GetWideProjection(
        Signed320 x,
        Signed320 y,
        Signed320 z,
        Signed192 axisX,
        Signed192 axisY,
        Signed192 axisZ) =>
        WideArithmetic.AddSigned576(
            WideArithmetic.AddSigned576(
                WideArithmetic.MultiplySigned320(
                    x,
                    Signed320.ExtendValue(axisX)),
                WideArithmetic.MultiplySigned320(
                    y,
                    Signed320.ExtendValue(axisY))),
            WideArithmetic.MultiplySigned320(
                z,
                Signed320.ExtendValue(axisZ)));

    private static bool TryGetScaledExactCoordinate(
        Signed576 numerator,
        Signed576 denominator,
        Fixed64 scale,
        out Fixed64 coordinate)
    {
        if (scale == Fixed64.One)
        {
            return Fixed64.TryGetSignedRawRatio(
                numerator,
                denominator,
                out coordinate);
        }
        if (scale == Fixed64.Zero)
        {
            coordinate = Fixed64.Zero;
            return true;
        }

        return Fixed64.TryGetSignedRawRatio(
            WideArithmetic.MultiplySigned576ToSigned704(
                numerator,
                Signed320.ExtendValue(Signed192.Raw(scale))),
            WideArithmetic.MultiplySigned576ToSigned704(
                denominator,
                Signed320.ExtendValue(Signed192.One)),
            out coordinate);
    }
}
