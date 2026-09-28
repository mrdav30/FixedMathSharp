//=======================================================================
// MIT License, Copyright (c) 2024-present David Oravsky (mrdav30)
// See LICENSE file in the project root for full license information.
//=======================================================================

namespace FixedMathSharp.Geometry;

/// <summary>Exact shared cylinder-local coordinates for polytope geometry.</summary>
internal readonly struct CylinderPolytopeFrame
{
    internal readonly WideRationalBasis3d Basis;
    internal readonly WideAxis3 Translation;
    internal readonly Signed192 Denominator;
    internal readonly Signed320 Cap;
    internal readonly Signed192 Radius;

    internal CylinderPolytopeFrame(Vector3d cylinderCenter, FixedQuaternion cylinderRotation,
        Fixed64 height, Fixed64 radius, Vector3d origin, FixedQuaternion rotation)
        : this(cylinderCenter, cylinderRotation, Signed192.Raw(height), radius, origin, rotation)
    {
    }

    internal CylinderPolytopeFrame(Vector3d cylinderCenter, FixedQuaternion cylinderRotation,
        Signed192 height, Fixed64 radius, Vector3d origin, FixedQuaternion rotation)
    {
        WideRationalBasis3d cylinderBasis = new(cylinderRotation);
        WideRationalBasis3d shapeBasis = new(rotation);
        Basis = WideRationalBasis3d.CreateRelative(cylinderBasis, shapeBasis);
        Denominator = Basis.Denominator;
        WideOrientedBox.GetRelativeLocalPointNumerators(origin, cylinderCenter, cylinderBasis,
            out Signed192 x, out Signed192 y, out Signed192 z);
        Translation = new WideAxis3(
            WideArithmetic.MultiplySigned192(x, shapeBasis.Denominator),
            WideArithmetic.MultiplySigned192(y, shapeBasis.Denominator),
            WideArithmetic.MultiplySigned192(z, shapeBasis.Denominator));
        // Work with doubled cylinder-local coordinates so odd raw full
        // heights need no rounded half-height. All vertices share D.
        Cap = WideArithmetic.MultiplySigned192(height, Denominator);
        Radius = WideArithmetic.AddSigned192(Signed192.Raw(radius), Signed192.Raw(radius));
    }

    internal WideAxis3 Transform(Vector3d point)
    {
        WideAxis3 local = WideRigidProjection.TransformLocalAxis(Basis,
            Signed192.Raw(point.X), Signed192.Raw(point.Y), Signed192.Raw(point.Z));
        Signed320 x = WideArithmetic.AddSigned320(local.X, Translation.X);
        Signed320 y = WideArithmetic.AddSigned320(local.Y, Translation.Y);
        Signed320 z = WideArithmetic.AddSigned320(local.Z, Translation.Z);
        return new WideAxis3(WideArithmetic.AddSigned320(x, x),
            WideArithmetic.AddSigned320(y, y), WideArithmetic.AddSigned320(z, z));
    }
}
