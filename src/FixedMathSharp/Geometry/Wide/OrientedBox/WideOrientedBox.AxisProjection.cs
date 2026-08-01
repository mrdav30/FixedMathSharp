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
        Signed192 differenceX = WideArithmetic.SubtractSigned192(
            Signed192.Raw(point.X),
            Signed192.Raw(origin.X));
        Signed192 differenceY = WideArithmetic.SubtractSigned192(
            Signed192.Raw(point.Y),
            Signed192.Raw(origin.Y));
        Signed192 differenceZ = WideArithmetic.SubtractSigned192(
            Signed192.Raw(point.Z),
            Signed192.Raw(origin.Z));
        x = GetRelativeLocalCoordinateNumerator(
            differenceX,
            differenceY,
            differenceZ,
            basis.Xx,
            basis.Xy,
            basis.Xz);
        y = GetRelativeLocalCoordinateNumerator(
            differenceX,
            differenceY,
            differenceZ,
            basis.Yx,
            basis.Yy,
            basis.Yz);
        z = GetRelativeLocalCoordinateNumerator(
            differenceX,
            differenceY,
            differenceZ,
            basis.Zx,
            basis.Zy,
            basis.Zz);
        rotationDenominator = basis.Denominator;
    }

    private static Signed192 GetRelativeLocalCoordinateNumerator(
        Signed192 differenceX,
        Signed192 differenceY,
        Signed192 differenceZ,
        Signed192 basisX,
        Signed192 basisY,
        Signed192 basisZ)
    {
        Signed320 numerator = WideArithmetic.AddSigned320(
            WideArithmetic.AddSigned320(
                WideArithmetic.MultiplySigned192(differenceX, basisX),
                WideArithmetic.MultiplySigned192(differenceY, basisY)),
            WideArithmetic.MultiplySigned192(differenceZ, basisZ));
        _ = Signed192.TryNarrowSigned(numerator, out Signed192 result);
        return result;
    }
}
