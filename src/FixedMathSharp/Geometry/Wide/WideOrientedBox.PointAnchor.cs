//=======================================================================
// WideOrientedBox.PointAnchor.cs
//=======================================================================
// MIT License, Copyright (c) 2024–present David Oravsky (mrdav30)
// See LICENSE file in the project root for full license information.
//=======================================================================

namespace FixedMathSharp.Bounds;

/// <content>
/// Wide-oriented box point anchor operations.
/// </content>
internal static partial class WideOrientedBox
{
    internal static bool TryGetPoint(
        Vector3d origin,
        FixedQuaternion rotation,
        Vector3d localPoint,
        Vector3d localDisplacement,
        out Vector3d point)
    {
        RationalBasis basis = new(rotation);
        Signed576 denominator = ToSigned576(basis.Denominator);
        bool representable = TryGetPointCoordinate(
            origin.X,
            basis.Xx,
            basis.Yx,
            basis.Zx,
            basis.Denominator,
            denominator,
            localPoint,
            localDisplacement,
            out Fixed64 x)
            & TryGetPointCoordinate(
                origin.Y,
                basis.Xy,
                basis.Yy,
                basis.Zy,
                basis.Denominator,
                denominator,
                localPoint,
                localDisplacement,
                out Fixed64 y)
            & TryGetPointCoordinate(
                origin.Z,
                basis.Xz,
                basis.Yz,
                basis.Zz,
                basis.Denominator,
                denominator,
                localPoint,
                localDisplacement,
                out Fixed64 z);
        if (!representable)
        {
            point = default;
            return false;
        }

        point = new Vector3d(x, y, z);
        return true;
    }

    internal static bool TryGetRelativeOffset(
        Vector3d firstOrigin,
        FixedQuaternion firstRotation,
        Vector3d firstLocalPoint,
        Vector3d firstLocalDisplacement,
        Vector3d secondOrigin,
        FixedQuaternion secondRotation,
        Vector3d secondLocalPoint,
        Vector3d secondLocalDisplacement,
        Fixed64 scale,
        out Vector3d offset)
    {
        RationalBasis firstBasis = new(firstRotation);
        RationalBasis secondBasis = new(secondRotation);
        Signed320 denominator = WideArithmetic.MultiplySigned192(
            firstBasis.Denominator,
            secondBasis.Denominator);
        Signed576 denominatorWide = Signed576.ExtendValue(denominator);
        bool representable = TryGetRelativeOffsetCoordinate(
            firstOrigin.X,
            firstBasis.Xx,
            firstBasis.Yx,
            firstBasis.Zx,
            firstBasis.Denominator,
            firstLocalPoint,
            firstLocalDisplacement,
            secondOrigin.X,
            secondBasis.Xx,
            secondBasis.Yx,
            secondBasis.Zx,
            secondBasis.Denominator,
            secondLocalPoint,
            secondLocalDisplacement,
            denominator,
            denominatorWide,
            scale,
            out Fixed64 x)
            & TryGetRelativeOffsetCoordinate(
                firstOrigin.Y,
                firstBasis.Xy,
                firstBasis.Yy,
                firstBasis.Zy,
                firstBasis.Denominator,
                firstLocalPoint,
                firstLocalDisplacement,
                secondOrigin.Y,
                secondBasis.Xy,
                secondBasis.Yy,
                secondBasis.Zy,
            secondBasis.Denominator,
            secondLocalPoint,
            secondLocalDisplacement,
            denominator,
            denominatorWide,
            scale,
            out Fixed64 y)
            & TryGetRelativeOffsetCoordinate(
                firstOrigin.Z,
                firstBasis.Xz,
                firstBasis.Yz,
                firstBasis.Zz,
                firstBasis.Denominator,
                firstLocalPoint,
                firstLocalDisplacement,
                secondOrigin.Z,
                secondBasis.Xz,
                secondBasis.Yz,
                secondBasis.Zz,
            secondBasis.Denominator,
            secondLocalPoint,
            secondLocalDisplacement,
            denominator,
            denominatorWide,
            scale,
            out Fixed64 z);
        if (!representable)
        {
            offset = default;
            return false;
        }

        offset = new Vector3d(x, y, z);
        return true;
    }

    internal static bool TryGetLocalPointIn(
        Vector3d pointOrigin,
        FixedQuaternion pointRotation,
        Vector3d pointLocalPoint,
        Vector3d pointLocalDisplacement,
        Vector3d frameOrigin,
        FixedQuaternion frameRotation,
        out Vector3d localPoint)
    {
        RationalBasis pointBasis = new(pointRotation);
        RationalBasis frameBasis = new(frameRotation);
        Signed320 denominator = WideArithmetic.MultiplySigned192(
            pointBasis.Denominator,
            frameBasis.Denominator);
        Signed576 denominatorWide = Signed576.ExtendValue(denominator);
        Signed192 originX = WideArithmetic.SubtractSigned192(
            Signed192.Raw(pointOrigin.X),
            Signed192.Raw(frameOrigin.X));
        Signed192 originY = WideArithmetic.SubtractSigned192(
            Signed192.Raw(pointOrigin.Y),
            Signed192.Raw(frameOrigin.Y));
        Signed192 originZ = WideArithmetic.SubtractSigned192(
            Signed192.Raw(pointOrigin.Z),
            Signed192.Raw(frameOrigin.Z));
        Signed320 rotatedX = GetProjection(
            Signed192.Raw(pointLocalPoint.X),
            Signed192.Raw(pointLocalPoint.Y),
            Signed192.Raw(pointLocalPoint.Z),
            pointBasis.Xx,
            pointBasis.Yx,
            pointBasis.Zx);
        rotatedX = WideArithmetic.AddSigned320(
            rotatedX,
            GetProjection(
                Signed192.Raw(pointLocalDisplacement.X),
                Signed192.Raw(pointLocalDisplacement.Y),
                Signed192.Raw(pointLocalDisplacement.Z),
                pointBasis.Xx,
                pointBasis.Yx,
                pointBasis.Zx));
        Signed320 rotatedY = GetProjection(
            Signed192.Raw(pointLocalPoint.X),
            Signed192.Raw(pointLocalPoint.Y),
            Signed192.Raw(pointLocalPoint.Z),
            pointBasis.Xy,
            pointBasis.Yy,
            pointBasis.Zy);
        rotatedY = WideArithmetic.AddSigned320(
            rotatedY,
            GetProjection(
                Signed192.Raw(pointLocalDisplacement.X),
                Signed192.Raw(pointLocalDisplacement.Y),
                Signed192.Raw(pointLocalDisplacement.Z),
                pointBasis.Xy,
                pointBasis.Yy,
                pointBasis.Zy));
        Signed320 rotatedZ = GetProjection(
            Signed192.Raw(pointLocalPoint.X),
            Signed192.Raw(pointLocalPoint.Y),
            Signed192.Raw(pointLocalPoint.Z),
            pointBasis.Xz,
            pointBasis.Yz,
            pointBasis.Zz);
        rotatedZ = WideArithmetic.AddSigned320(
            rotatedZ,
            GetProjection(
                Signed192.Raw(pointLocalDisplacement.X),
                Signed192.Raw(pointLocalDisplacement.Y),
                Signed192.Raw(pointLocalDisplacement.Z),
                pointBasis.Xz,
                pointBasis.Yz,
                pointBasis.Zz));

        bool representable = TryGetLocalPointCoordinate(
            originX,
            originY,
            originZ,
            rotatedX,
            rotatedY,
            rotatedZ,
            pointBasis.Denominator,
            frameBasis.Xx,
            frameBasis.Xy,
            frameBasis.Xz,
            denominatorWide,
            out Fixed64 x)
            & TryGetLocalPointCoordinate(
                originX,
                originY,
                originZ,
                rotatedX,
                rotatedY,
                rotatedZ,
                pointBasis.Denominator,
                frameBasis.Yx,
                frameBasis.Yy,
                frameBasis.Yz,
                denominatorWide,
                out Fixed64 y)
            & TryGetLocalPointCoordinate(
                originX,
                originY,
                originZ,
                rotatedX,
                rotatedY,
                rotatedZ,
                pointBasis.Denominator,
                frameBasis.Zx,
                frameBasis.Zy,
                frameBasis.Zz,
                denominatorWide,
                out Fixed64 z);
        if (!representable)
        {
            localPoint = default;
            return false;
        }

        localPoint = new Vector3d(x, y, z);
        return true;
    }

    internal static bool TryGetProjectedOffset(
        Vector3d firstOrigin,
        FixedQuaternion firstRotation,
        Vector3d firstLocalPoint,
        Vector3d firstLocalDisplacement,
        Vector3d secondOrigin,
        FixedQuaternion secondRotation,
        Vector3d secondLocalPoint,
        Vector3d secondLocalDisplacement,
        Vector3d direction,
        out Fixed64 projection)
    {
        GetProjectedOffsetRatio(
            firstOrigin,
            firstRotation,
            firstLocalPoint,
            firstLocalDisplacement,
            secondOrigin,
            secondRotation,
            secondLocalPoint,
            secondLocalDisplacement,
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
        Vector3d secondOrigin,
        FixedQuaternion secondRotation,
        Vector3d secondLocalPoint,
        Vector3d secondLocalDisplacement,
        Vector3d direction)
    {
        GetProjectedOffsetRatio(
            firstOrigin,
            firstRotation,
            firstLocalPoint,
            firstLocalDisplacement,
            secondOrigin,
            secondRotation,
            secondLocalPoint,
            secondLocalDisplacement,
            direction,
            out Signed704 numerator,
            out Signed704 denominator);
        return Fixed64.GetNonNegativeRawRatioFloor(
            numerator,
            denominator);
    }

    internal static void GetProjectedOffsetRatio(
        in FixedPointAnchor first,
        in FixedPointAnchor second,
        Vector3d direction,
        out Signed704 numerator,
        out Signed704 denominator)
    {
        if (!first.ExactLocalTerm.IsZero
            || first.LocalTranslation != Vector3d.Zero)
        {
            GetExactProjectedOffsetRatio(
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
                direction,
                out numerator,
                out denominator);
            return;
        }

        GetProjectedOffsetRatio(
            first.Origin,
            first.Rotation,
            first.LocalPoint,
            first.LocalDisplacement,
            second.Origin,
            second.Rotation,
            second.LocalPoint,
            second.LocalDisplacement,
            direction,
            out numerator,
            out denominator);
    }

    private static void GetProjectedOffsetRatio(
        Vector3d firstOrigin,
        FixedQuaternion firstRotation,
        Vector3d firstLocalPoint,
        Vector3d firstLocalDisplacement,
        Vector3d secondOrigin,
        FixedQuaternion secondRotation,
        Vector3d secondLocalPoint,
        Vector3d secondLocalDisplacement,
        Vector3d direction,
        out Signed704 numerator,
        out Signed704 denominator)
    {
        RationalBasis firstBasis = new(firstRotation);
        RationalBasis secondBasis = new(secondRotation);
        Signed320 coordinateDenominator =
            WideArithmetic.MultiplySigned192(
                firstBasis.Denominator,
                secondBasis.Denominator);
        Signed576 coordinateDenominatorWide =
            Signed576.ExtendValue(coordinateDenominator);
        Signed576 x = GetRelativeOffsetNumerator(
            firstOrigin.X,
            firstBasis.Xx,
            firstBasis.Yx,
            firstBasis.Zx,
            firstBasis.Denominator,
            firstLocalPoint,
            firstLocalDisplacement,
            secondOrigin.X,
            secondBasis.Xx,
            secondBasis.Yx,
            secondBasis.Zx,
            secondBasis.Denominator,
            secondLocalPoint,
            secondLocalDisplacement,
            coordinateDenominator);
        Signed576 y = GetRelativeOffsetNumerator(
            firstOrigin.Y,
            firstBasis.Xy,
            firstBasis.Yy,
            firstBasis.Zy,
            firstBasis.Denominator,
            firstLocalPoint,
            firstLocalDisplacement,
            secondOrigin.Y,
            secondBasis.Xy,
            secondBasis.Yy,
            secondBasis.Zy,
            secondBasis.Denominator,
            secondLocalPoint,
            secondLocalDisplacement,
            coordinateDenominator);
        Signed576 z = GetRelativeOffsetNumerator(
            firstOrigin.Z,
            firstBasis.Xz,
            firstBasis.Yz,
            firstBasis.Zz,
            firstBasis.Denominator,
            firstLocalPoint,
            firstLocalDisplacement,
            secondOrigin.Z,
            secondBasis.Xz,
            secondBasis.Yz,
            secondBasis.Zz,
            secondBasis.Denominator,
            secondLocalPoint,
            secondLocalDisplacement,
            coordinateDenominator);
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
            coordinateDenominatorWide,
            Signed320.ExtendValue(Signed192.One));
    }

    private static bool TryGetPointCoordinate(
        Fixed64 origin,
        Signed192 axisX,
        Signed192 axisY,
        Signed192 axisZ,
        Signed192 denominator,
        Signed576 denominatorWide,
        Vector3d localPoint,
        Vector3d localDisplacement,
        out Fixed64 coordinate)
    {
        Signed320 numerator = WideArithmetic.AddSigned320(
            WideArithmetic.MultiplySigned192(Signed192.Raw(origin), denominator),
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
                    axisZ)));
        return Fixed64.TryGetSignedRawRatio(
            Signed576.ExtendValue(numerator),
            denominatorWide,
            out coordinate);
    }

    private static bool TryGetRelativeOffsetCoordinate(
        Fixed64 firstOrigin,
        Signed192 firstAxisX,
        Signed192 firstAxisY,
        Signed192 firstAxisZ,
        Signed192 firstDenominator,
        Vector3d firstLocalPoint,
        Vector3d firstLocalDisplacement,
        Fixed64 secondOrigin,
        Signed192 secondAxisX,
        Signed192 secondAxisY,
        Signed192 secondAxisZ,
        Signed192 secondDenominator,
        Vector3d secondLocalPoint,
        Vector3d secondLocalDisplacement,
        Signed320 denominator,
        Signed576 denominatorWide,
        Fixed64 scale,
        out Fixed64 coordinate)
    {
        Signed576 numerator = GetRelativeOffsetNumerator(
            firstOrigin,
            firstAxisX,
            firstAxisY,
            firstAxisZ,
            firstDenominator,
            firstLocalPoint,
            firstLocalDisplacement,
            secondOrigin,
            secondAxisX,
            secondAxisY,
            secondAxisZ,
            secondDenominator,
            secondLocalPoint,
            secondLocalDisplacement,
            denominator);
        if (scale == Fixed64.One)
        {
            return Fixed64.TryGetSignedRawRatio(
                numerator,
                denominatorWide,
                out coordinate);
        }
        if (scale == Fixed64.Zero)
        {
            coordinate = Fixed64.Zero;
            return true;
        }

        Signed320 scaleRaw = Signed320.ExtendValue(Signed192.Raw(scale));
        Signed320 oneRaw = Signed320.ExtendValue(Signed192.One);
        return Fixed64.TryGetSignedRawRatio(
            WideArithmetic.MultiplySigned576ToSigned704(
                numerator,
                scaleRaw),
            WideArithmetic.MultiplySigned576ToSigned704(
                denominatorWide,
                oneRaw),
            out coordinate);
    }

    private static Signed576 GetRelativeOffsetNumerator(
        Fixed64 firstOrigin,
        Signed192 firstAxisX,
        Signed192 firstAxisY,
        Signed192 firstAxisZ,
        Signed192 firstDenominator,
        Vector3d firstLocalPoint,
        Vector3d firstLocalDisplacement,
        Fixed64 secondOrigin,
        Signed192 secondAxisX,
        Signed192 secondAxisY,
        Signed192 secondAxisZ,
        Signed192 secondDenominator,
        Vector3d secondLocalPoint,
        Vector3d secondLocalDisplacement,
        Signed320 denominator)
    {
        Signed320 firstRotated = GetProjection(
            Signed192.Raw(firstLocalPoint.X),
            Signed192.Raw(firstLocalPoint.Y),
            Signed192.Raw(firstLocalPoint.Z),
            firstAxisX,
            firstAxisY,
            firstAxisZ);
        firstRotated = WideArithmetic.AddSigned320(
            firstRotated,
            GetProjection(
                Signed192.Raw(firstLocalDisplacement.X),
                Signed192.Raw(firstLocalDisplacement.Y),
                Signed192.Raw(firstLocalDisplacement.Z),
                firstAxisX,
                firstAxisY,
                firstAxisZ));
        Signed320 secondRotated = GetProjection(
            Signed192.Raw(secondLocalPoint.X),
            Signed192.Raw(secondLocalPoint.Y),
            Signed192.Raw(secondLocalPoint.Z),
            secondAxisX,
            secondAxisY,
            secondAxisZ);
        secondRotated = WideArithmetic.AddSigned320(
            secondRotated,
            GetProjection(
                Signed192.Raw(secondLocalDisplacement.X),
                Signed192.Raw(secondLocalDisplacement.Y),
                Signed192.Raw(secondLocalDisplacement.Z),
                secondAxisX,
                secondAxisY,
                secondAxisZ));
        return WideArithmetic.SubtractSigned576(
            WideArithmetic.AddSigned576(
                WideArithmetic.MultiplySigned320(
                    Signed320.ExtendValue(
                        WideArithmetic.SubtractSigned192(
                            Signed192.Raw(firstOrigin),
                            Signed192.Raw(secondOrigin))),
                    denominator),
                WideArithmetic.MultiplySigned320(
                    firstRotated,
                    Signed320.ExtendValue(secondDenominator))),
            WideArithmetic.MultiplySigned320(
                secondRotated,
                Signed320.ExtendValue(firstDenominator)));
    }

    private static bool TryGetLocalPointCoordinate(
        Signed192 originX,
        Signed192 originY,
        Signed192 originZ,
        Signed320 rotatedX,
        Signed320 rotatedY,
        Signed320 rotatedZ,
        Signed192 pointDenominator,
        Signed192 frameAxisX,
        Signed192 frameAxisY,
        Signed192 frameAxisZ,
        Signed576 denominator,
        out Fixed64 coordinate)
    {
        Signed320 originProjection = GetProjection(
            originX,
            originY,
            originZ,
            frameAxisX,
            frameAxisY,
            frameAxisZ);
        Signed576 rotatedProjection = WideArithmetic.AddSigned576(
            WideArithmetic.AddSigned576(
                WideArithmetic.MultiplySigned320(
                    rotatedX,
                    Signed320.ExtendValue(frameAxisX)),
                WideArithmetic.MultiplySigned320(
                    rotatedY,
                    Signed320.ExtendValue(frameAxisY))),
            WideArithmetic.MultiplySigned320(
                rotatedZ,
                Signed320.ExtendValue(frameAxisZ)));
        Signed576 numerator = WideArithmetic.AddSigned576(
            WideArithmetic.MultiplySigned320(
                originProjection,
                Signed320.ExtendValue(pointDenominator)),
            rotatedProjection);
        return Fixed64.TryGetSignedRawRatio(
            numerator,
            denominator,
            out coordinate);
    }
}
