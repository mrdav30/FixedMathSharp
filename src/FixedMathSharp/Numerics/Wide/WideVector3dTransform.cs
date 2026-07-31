//=======================================================================
// WideVector3dTransform.cs
//=======================================================================
// MIT License, Copyright (c) 2024–present David Oravsky (mrdav30)
// See LICENSE file in the project root for full license information.
//=======================================================================

using FixedMathSharp.Geometry;

namespace FixedMathSharp;

/// <summary>
/// Owns exact component-scaled three-dimensional point transforms.
/// </summary>
internal static class WideVector3dTransform
{
    internal static bool TryTransformScaledPoint(
        Vector3d origin,
        FixedQuaternion rotation,
        Vector3d localPoint,
        Vector3d scale,
        Vector3d localDisplacement,
        out Vector3d worldPoint)
    {
        WideRationalBasis3d basis = new(rotation);
        Signed320 scaledDenominator = WideArithmetic.MultiplySigned192(
            basis.Denominator,
            Signed192.One);
        Signed576 denominator = Signed576.ExtendValue(scaledDenominator);
        bool representable = TryTransformScaledCoordinate(
                origin.X,
                basis.Xx,
                basis.Yx,
                basis.Zx,
                scaledDenominator,
                denominator,
                localPoint,
                scale,
                localDisplacement,
                out Fixed64 x)
            & TryTransformScaledCoordinate(
                origin.Y,
                basis.Xy,
                basis.Yy,
                basis.Zy,
                scaledDenominator,
                denominator,
                localPoint,
                scale,
                localDisplacement,
                out Fixed64 y)
            & TryTransformScaledCoordinate(
                origin.Z,
                basis.Xz,
                basis.Yz,
                basis.Zz,
                scaledDenominator,
                denominator,
                localPoint,
                scale,
                localDisplacement,
                out Fixed64 z);
        if (!representable)
        {
            worldPoint = default;
            return false;
        }

        worldPoint = new Vector3d(x, y, z);
        return true;
    }

    internal static bool TryInverseTransformScaledPoint(
        Vector3d origin,
        FixedQuaternion rotation,
        Vector3d worldPoint,
        Vector3d scale,
        out Vector3d localPoint)
    {
        WideRationalBasis3d basis = new(rotation);
        Signed192 offsetX = WideArithmetic.SubtractSigned192(
            Signed192.Raw(worldPoint.X),
            Signed192.Raw(origin.X));
        Signed192 offsetY = WideArithmetic.SubtractSigned192(
            Signed192.Raw(worldPoint.Y),
            Signed192.Raw(origin.Y));
        Signed192 offsetZ = WideArithmetic.SubtractSigned192(
            Signed192.Raw(worldPoint.Z),
            Signed192.Raw(origin.Z));
        bool representable = TryInverseTransformScaledCoordinate(
                offsetX,
                offsetY,
                offsetZ,
                basis.Xx,
                basis.Xy,
                basis.Xz,
                basis.Denominator,
                scale.X,
                out Fixed64 x)
            & TryInverseTransformScaledCoordinate(
                offsetX,
                offsetY,
                offsetZ,
                basis.Yx,
                basis.Yy,
                basis.Yz,
                basis.Denominator,
                scale.Y,
                out Fixed64 y)
            & TryInverseTransformScaledCoordinate(
                offsetX,
                offsetY,
                offsetZ,
                basis.Zx,
                basis.Zy,
                basis.Zz,
                basis.Denominator,
                scale.Z,
                out Fixed64 z);
        if (!representable)
        {
            localPoint = default;
            return false;
        }

        localPoint = new Vector3d(x, y, z);
        return true;
    }

    internal static bool TryComposeScaledLocalPoints(
        Vector3d outerLocalPoint,
        Vector3d outerScale,
        Vector3d innerFrameOffset,
        Vector3d innerFrameScale,
        Vector3d innerLocalDisplacement,
        FixedQuaternion innerRotation,
        out Vector3d result)
    {
        WideRationalBasis3d basis = new(innerRotation);
        Signed320 scaledDenominator = WideArithmetic.MultiplySigned192(
            basis.Denominator,
            Signed192.One);
        Signed576 denominator = Signed576.ExtendValue(scaledDenominator);
        bool representable = TryComposeScaledLocalCoordinate(
                outerLocalPoint.X,
                outerScale.X,
                basis.Xx,
                basis.Yx,
                basis.Zx,
                basis.Denominator,
                denominator,
                innerFrameOffset.X,
                innerFrameScale.X,
                innerLocalDisplacement,
                out Fixed64 x)
            & TryComposeScaledLocalCoordinate(
                outerLocalPoint.Y,
                outerScale.Y,
                basis.Xy,
                basis.Yy,
                basis.Zy,
                basis.Denominator,
                denominator,
                innerFrameOffset.Y,
                innerFrameScale.Y,
                innerLocalDisplacement,
                out Fixed64 y)
            & TryComposeScaledLocalCoordinate(
                outerLocalPoint.Z,
                outerScale.Z,
                basis.Xz,
                basis.Yz,
                basis.Zz,
                basis.Denominator,
                denominator,
                innerFrameOffset.Z,
                innerFrameScale.Z,
                innerLocalDisplacement,
                out Fixed64 z);
        if (!representable)
        {
            result = default;
            return false;
        }

        result = new Vector3d(x, y, z);
        return true;
    }

    private static bool TryTransformScaledCoordinate(
        Fixed64 origin,
        Signed192 axisX,
        Signed192 axisY,
        Signed192 axisZ,
        Signed320 scaledDenominator,
        Signed576 denominator,
        Vector3d localPoint,
        Vector3d scale,
        Vector3d localDisplacement,
        out Fixed64 coordinate)
    {
        Signed576 scaledLocal = WideArithmetic.AddSigned576(
            WideArithmetic.AddSigned576(
                WideArithmetic.MultiplySigned320(
                    WideArithmetic.MultiplySigned192(
                        Signed192.Raw(localPoint.X),
                        Signed192.Raw(scale.X)),
                    Signed320.ExtendValue(axisX)),
                WideArithmetic.MultiplySigned320(
                    WideArithmetic.MultiplySigned192(
                        Signed192.Raw(localPoint.Y),
                        Signed192.Raw(scale.Y)),
                    Signed320.ExtendValue(axisY))),
            WideArithmetic.MultiplySigned320(
                WideArithmetic.MultiplySigned192(
                    Signed192.Raw(localPoint.Z),
                    Signed192.Raw(scale.Z)),
                Signed320.ExtendValue(axisZ)));
        Signed320 displacement = WideArithmetic.GetDotProduct3D(
            Signed192.Raw(localDisplacement.X),
            Signed192.Raw(localDisplacement.Y),
            Signed192.Raw(localDisplacement.Z),
            axisX,
            axisY,
            axisZ);
        Signed576 numerator = WideArithmetic.AddSigned576(
            WideArithmetic.AddSigned576(
                WideArithmetic.MultiplySigned320(
                    Signed320.ExtendValue(Signed192.Raw(origin)),
                    scaledDenominator),
                scaledLocal),
            WideArithmetic.MultiplySigned320(
                displacement,
                Signed320.ExtendValue(Signed192.One)));
        return Fixed64.TryGetSignedRawRatio(
            numerator,
            denominator,
            out coordinate);
    }

    private static bool TryInverseTransformScaledCoordinate(
        Signed192 offsetX,
        Signed192 offsetY,
        Signed192 offsetZ,
        Signed192 axisX,
        Signed192 axisY,
        Signed192 axisZ,
        Signed192 rotationDenominator,
        Fixed64 scale,
        out Fixed64 coordinate)
    {
        Signed320 projection = WideArithmetic.GetDotProduct3D(
            offsetX,
            offsetY,
            offsetZ,
            axisX,
            axisY,
            axisZ);
        Signed576 numerator = WideArithmetic.MultiplySigned320(
            projection,
            Signed320.ExtendValue(Signed192.One));
        Signed576 denominator = Signed576.ExtendValue(
            WideArithmetic.MultiplySigned192(
                rotationDenominator,
                Signed192.Raw(scale)));
        return Fixed64.TryGetSignedRawRatio(
            numerator,
            denominator,
            out coordinate);
    }

    private static bool TryComposeScaledLocalCoordinate(
        Fixed64 outerLocalPoint,
        Fixed64 outerScale,
        Signed192 axisX,
        Signed192 axisY,
        Signed192 axisZ,
        Signed192 rotationDenominator,
        Signed576 denominator,
        Fixed64 innerFrameOffset,
        Fixed64 innerFrameScale,
        Vector3d innerLocalDisplacement,
        out Fixed64 coordinate) =>
        Fixed64.TryGetSignedRawRatio(
            WideRationalBasis3d.GetComposedScaledCoordinateNumerator(
                outerLocalPoint,
                outerScale,
                axisX,
                axisY,
                axisZ,
                rotationDenominator,
                innerFrameOffset,
                innerFrameScale,
                innerLocalDisplacement),
            denominator,
            out coordinate);
}
