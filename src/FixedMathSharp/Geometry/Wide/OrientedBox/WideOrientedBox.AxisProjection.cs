//=======================================================================
// WideOrientedBox.AxisProjection.cs
//=======================================================================
// MIT License, Copyright (c) 2024–present David Oravsky (mrdav30)
// See LICENSE file in the project root for full license information.
//=======================================================================

namespace FixedMathSharp.Geometry;

/// <content>
/// Provides wide-precision axis projection utilities for oriented boxes, including
/// rotated axis sign/numerator computation and relative local point coordinate
/// numerators used for high-precision separating axis and containment tests.
/// </content>
internal static partial class WideOrientedBox
{
    private static Signed576 GetBoxProjectionRadiusNumerator(
        in WideAxis3 axis,
        Vector3d halfExtents,
        in WideRationalBasis3d basis) =>
        WideArithmetic.AddSigned576(
            WideArithmetic.AddSigned576(
                WideArithmetic.MultiplySigned576(
                    GetMagnitude(WideRigidProjection.GetBasisAxisProjection(
                        axis,
                        basis.Xx,
                        basis.Xy,
                        basis.Xz)),
                    Signed192.Raw(halfExtents.X)),
                WideArithmetic.MultiplySigned576(
                    GetMagnitude(WideRigidProjection.GetBasisAxisProjection(
                        axis,
                        basis.Yx,
                        basis.Yy,
                        basis.Yz)),
                    Signed192.Raw(halfExtents.Y))),
            WideArithmetic.MultiplySigned576(
                GetMagnitude(WideRigidProjection.GetBasisAxisProjection(
                    axis,
                    basis.Zx,
                    basis.Zy,
                    basis.Zz)),
                Signed192.Raw(halfExtents.Z)));

    private static Signed576 GetLocalBoxProjectionRadiusNumerator(
        in WideAxis3 axis,
        Vector3d halfExtents) =>
        WideArithmetic.AddSigned576(
            WideArithmetic.AddSigned576(
                WideArithmetic.MultiplySigned576(
                    Signed576.ExtendValue(GetMagnitude(axis.X)),
                    halfExtents.X.m_rawValue),
                WideArithmetic.MultiplySigned576(
                    Signed576.ExtendValue(GetMagnitude(axis.Y)),
                    halfExtents.Y.m_rawValue)),
            WideArithmetic.MultiplySigned576(
                Signed576.ExtendValue(GetMagnitude(axis.Z)),
                halfExtents.Z.m_rawValue));

    internal static int GetRotatedLocalProjectionSign(
        FixedQuaternion rotation,
        Vector3d localDirection,
        Vector3d worldDirection)
    {
        WideRationalBasis3d basis = new(rotation);
        WideAxis3 rotated = WideRigidProjection.TransformLocalAxis(
            basis,
            Signed192.Raw(localDirection.X),
            Signed192.Raw(localDirection.Y),
            Signed192.Raw(localDirection.Z));
        return WideArithmetic.AddSigned576(
            WideArithmetic.AddSigned576(
                WideArithmetic.MultiplySigned320(
                    rotated.X,
                    Signed320.ExtendValue(
                        Signed192.Raw(worldDirection.X))),
                WideArithmetic.MultiplySigned320(
                    rotated.Y,
                    Signed320.ExtendValue(
                        Signed192.Raw(worldDirection.Y)))),
            WideArithmetic.MultiplySigned320(
                rotated.Z,
                Signed320.ExtendValue(
                    Signed192.Raw(worldDirection.Z)))).Sign;
    }

    internal static void GetRotatedLocalAxisNumerators(
        FixedQuaternion rotation,
        Vector3d localDirection,
        out Signed192 x,
        out Signed192 y,
        out Signed192 z,
        out Signed192 rotationDenominator)
    {
        WideRationalBasis3d basis = new(rotation);
        WideAxis3 rotated = WideRigidProjection.TransformLocalAxis(
            basis,
            Signed192.Raw(localDirection.X),
            Signed192.Raw(localDirection.Y),
            Signed192.Raw(localDirection.Z));
        // A Q32.32 local component multiplied by a sum of quaternion products
        // remains strictly below the signed 192-bit range.
        _ = Signed192.TryNarrowSigned(rotated.X, out x);
        _ = Signed192.TryNarrowSigned(rotated.Y, out y);
        _ = Signed192.TryNarrowSigned(rotated.Z, out z);
        rotationDenominator = basis.Denominator;
    }

    internal static void GetRelativeLocalPointNumerators(
        Vector3d point,
        Vector3d origin,
        FixedQuaternion rotation,
        out Signed192 x,
        out Signed192 y,
        out Signed192 z,
        out Signed192 rotationDenominator)
    {
        WideRationalBasis3d basis = new(rotation);
        GetRelativeLocalPointNumerators(
            point,
            origin,
            basis,
            out x,
            out y,
            out z);
        rotationDenominator = basis.Denominator;
    }

    internal static void GetRelativeLocalPointNumerators(
        Vector3d point,
        Vector3d origin,
        in WideRationalBasis3d basis,
        out Signed192 x,
        out Signed192 y,
        out Signed192 z)
    {
        GetPointProjections(
            point,
            origin,
            basis,
            out Signed320 wideX,
            out Signed320 wideY,
            out Signed320 wideZ);
        // A full raw-domain point difference projected through a normalized
        // quaternion basis remains below the Signed192 limit.
        x = Signed192.NarrowProven(wideX);
        y = Signed192.NarrowProven(wideY);
        z = Signed192.NarrowProven(wideZ);
    }
}
