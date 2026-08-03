//=======================================================================
// WidePointAnchor3d.cs
//=======================================================================
// MIT License, Copyright (c) 2024–present David Oravsky (mrdav30)
// See LICENSE file in the project root for full license information.
//=======================================================================

namespace FixedMathSharp.Geometry;

/// <summary>
/// Provides high-precision (wide/extended) arithmetic helpers for resolving anchored 3D points
/// and their relative offsets under rotation, avoiding precision loss from repeated fixed-point operations.
/// </summary>
internal static partial class WidePointAnchor3d
{
    internal static bool TryGetPoint(
        Vector3d origin,
        FixedQuaternion rotation,
        Vector3d localPoint,
        Vector3d localDisplacement,
        out Vector3d point)
    {
        WideRationalBasis3d basis = new(rotation);
        Signed576 denominator =
            Signed576.ExtendValue(
                Signed320.ExtendValue(basis.Denominator));
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
        if (firstOrigin == secondOrigin
            && firstRotation == secondRotation)
        {
            bool localRepresentable;
            Vector3d localOffset;
            if (firstLocalDisplacement == Vector3d.Zero
                && secondLocalDisplacement == Vector3d.Zero)
            {
                localRepresentable = Vector3d.TrySubtract(
                    firstLocalPoint,
                    secondLocalPoint,
                    out localOffset);
            }
            else
            {
                localRepresentable = Vector3d.TrySubtractSums(
                    firstLocalPoint,
                    firstLocalDisplacement,
                    secondLocalPoint,
                    secondLocalDisplacement,
                    out localOffset);
            }

            if (localRepresentable)
            {
                if (firstRotation != FixedQuaternion.Identity)
                {
                    if (scale == Fixed64.One)
                        return firstRotation.TryRotate(localOffset, out offset);
                }
                else
                {
                    return TryScaleOffset(localOffset, scale, out offset);
                }
            }
        }

        if (firstRotation == FixedQuaternion.Identity
            && secondRotation == FixedQuaternion.Identity)
        {
            bool compactRepresentable = Vector3d.TryAddSubtract(
                firstOrigin,
                firstLocalPoint,
                secondOrigin,
                out Vector3d firstStep);
            compactRepresentable &= Vector3d.TryAddSubtract(
                firstStep,
                firstLocalDisplacement,
                secondLocalPoint,
                out Vector3d secondStep);
            compactRepresentable &= Vector3d.TrySubtract(
                secondStep,
                secondLocalDisplacement,
                out Vector3d compactOffset);
            if (compactRepresentable)
                return TryScaleOffset(compactOffset, scale, out offset);
        }

        WideRationalBasis3d firstBasis = new(firstRotation);
        WideRationalBasis3d secondBasis = new(secondRotation);
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

    private static bool TryScaleOffset(
        Vector3d value,
        Fixed64 scale,
        out Vector3d result)
    {
        if (scale == Fixed64.One)
        {
            result = value;
            return true;
        }

        bool representable = Fixed64.TryMultiplyDivide(
                value.X,
                scale,
                Fixed64.One,
                out Fixed64 x)
            & Fixed64.TryMultiplyDivide(
                value.Y,
                scale,
                Fixed64.One,
                out Fixed64 y)
            & Fixed64.TryMultiplyDivide(
                value.Z,
                scale,
                Fixed64.One,
                out Fixed64 z);
        result = representable ? new Vector3d(x, y, z) : default;
        return representable;
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
        WideRationalBasis3d pointBasis = new(pointRotation);
        WideRationalBasis3d frameBasis = new(frameRotation);
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
        Signed320 rotatedX = WideArithmetic.GetDotProduct3D(
            Signed192.Raw(pointLocalPoint.X),
            Signed192.Raw(pointLocalPoint.Y),
            Signed192.Raw(pointLocalPoint.Z),
            pointBasis.Xx,
            pointBasis.Yx,
            pointBasis.Zx);
        rotatedX = WideArithmetic.AddSigned320(
            rotatedX,
            WideArithmetic.GetDotProduct3D(
                Signed192.Raw(pointLocalDisplacement.X),
                Signed192.Raw(pointLocalDisplacement.Y),
                Signed192.Raw(pointLocalDisplacement.Z),
                pointBasis.Xx,
                pointBasis.Yx,
                pointBasis.Zx));
        Signed320 rotatedY = WideArithmetic.GetDotProduct3D(
            Signed192.Raw(pointLocalPoint.X),
            Signed192.Raw(pointLocalPoint.Y),
            Signed192.Raw(pointLocalPoint.Z),
            pointBasis.Xy,
            pointBasis.Yy,
            pointBasis.Zy);
        rotatedY = WideArithmetic.AddSigned320(
            rotatedY,
            WideArithmetic.GetDotProduct3D(
                Signed192.Raw(pointLocalDisplacement.X),
                Signed192.Raw(pointLocalDisplacement.Y),
                Signed192.Raw(pointLocalDisplacement.Z),
                pointBasis.Xy,
                pointBasis.Yy,
                pointBasis.Zy));
        Signed320 rotatedZ = WideArithmetic.GetDotProduct3D(
            Signed192.Raw(pointLocalPoint.X),
            Signed192.Raw(pointLocalPoint.Y),
            Signed192.Raw(pointLocalPoint.Z),
            pointBasis.Xz,
            pointBasis.Yz,
            pointBasis.Zz);
        rotatedZ = WideArithmetic.AddSigned320(
            rotatedZ,
            WideArithmetic.GetDotProduct3D(
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
        WideRationalBasis3d firstBasis = new(firstRotation);
        WideRationalBasis3d secondBasis = new(secondRotation);
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
                WideArithmetic.GetDotProduct3D(
                    Signed192.Raw(localPoint.X),
                    Signed192.Raw(localPoint.Y),
                    Signed192.Raw(localPoint.Z),
                    axisX,
                    axisY,
                    axisZ),
                WideArithmetic.GetDotProduct3D(
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
        Signed320 firstRotated = WideArithmetic.GetDotProduct3D(
            Signed192.Raw(firstLocalPoint.X),
            Signed192.Raw(firstLocalPoint.Y),
            Signed192.Raw(firstLocalPoint.Z),
            firstAxisX,
            firstAxisY,
            firstAxisZ);
        firstRotated = WideArithmetic.AddSigned320(
            firstRotated,
            WideArithmetic.GetDotProduct3D(
                Signed192.Raw(firstLocalDisplacement.X),
                Signed192.Raw(firstLocalDisplacement.Y),
                Signed192.Raw(firstLocalDisplacement.Z),
                firstAxisX,
                firstAxisY,
                firstAxisZ));
        Signed320 secondRotated = WideArithmetic.GetDotProduct3D(
            Signed192.Raw(secondLocalPoint.X),
            Signed192.Raw(secondLocalPoint.Y),
            Signed192.Raw(secondLocalPoint.Z),
            secondAxisX,
            secondAxisY,
            secondAxisZ);
        secondRotated = WideArithmetic.AddSigned320(
            secondRotated,
            WideArithmetic.GetDotProduct3D(
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
        Signed320 originProjection = WideArithmetic.GetDotProduct3D(
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
