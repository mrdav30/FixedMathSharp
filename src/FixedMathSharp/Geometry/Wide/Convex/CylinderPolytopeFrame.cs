//=======================================================================
// CylinderPolytopeFrame.cs
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
    internal readonly Signed320 CoordinateDenominator;
    internal readonly bool HasExactScale;
    private readonly ScaledRigidFrame3d _sourceFrame;
    private readonly Vector3d _cylinderCenter;
    private readonly FixedQuaternion _cylinderRotation;
    internal ScaledRigidFrame3d SourceFrame => _sourceFrame;

    /// <summary>Exact capsule-local identity chart at a retained rational coordinate denominator.</summary>
    internal CylinderPolytopeFrame(Fixed64 height, Fixed64 radius, Signed320 coordinateDenominator)
    {
        this = default;
        CoordinateDenominator = coordinateDenominator;
        Cap = Signed320.NarrowValue(WideArithmetic.MultiplySigned320(coordinateDenominator, Signed320.ExtendValue(Signed192.Raw(height))));
        Radius = WideArithmetic.AddSigned192(Signed192.Raw(radius), Signed192.Raw(radius));
        HasExactScale = !Signed192.TryNarrowSigned(coordinateDenominator, out Signed192 denominator);
        if (HasExactScale)
        {
            Basis = new WideRationalBasis3d(FixedQuaternion.Identity);
            Denominator = Signed192.Signed(1);
            _sourceFrame = ScaledRigidFrame3d.Rigid(Vector3d.Zero, FixedQuaternion.Identity);
            _cylinderRotation = FixedQuaternion.Identity;
        }
        else
        {
            Denominator = denominator;
            Basis = WideRationalBasis3d.ScaledIdentity(denominator);
        }
    }

    internal CylinderPolytopeFrame(Vector3d cylinderCenter, FixedQuaternion cylinderRotation,
        Fixed64 height, Fixed64 radius, Vector3d origin, FixedQuaternion rotation)
        : this(cylinderCenter, cylinderRotation, Signed192.Raw(height), radius, origin, rotation)
    {
    }

    internal CylinderPolytopeFrame(Vector3d cylinderCenter, FixedQuaternion cylinderRotation,
        Signed192 height, Fixed64 radius, Vector3d origin, FixedQuaternion rotation)
    {
        this = default;
        WideRationalBasis3d cylinderBasis = new(cylinderRotation);
        WideRationalBasis3d shapeBasis = new(rotation);
        Basis = WideRationalBasis3d.CreateRelative(cylinderBasis, shapeBasis);
        Denominator = Basis.Denominator;
        CoordinateDenominator = Signed320.ExtendValue(Denominator);
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

    /// <summary>
    /// Retains an affine source for finite near-field geometry. Callers admit
    /// source/target reach at most 4096 and relative coordinates at most 16384.
    /// </summary>
    internal CylinderPolytopeFrame(in ScaledRigidFrame3d sourceFrame,
        Vector3d cylinderCenter, FixedQuaternion cylinderRotation, Signed192 height, Fixed64 radius)
    {
        // Finite-section and triangle owners dispatch rigid source frames
        // before reaching this affine arithmetic owner.
        this = default;
        _sourceFrame = sourceFrame;
        _cylinderCenter = cylinderCenter;
        _cylinderRotation = cylinderRotation;
        HasExactScale = true;
        WideRationalBasis3d cylinderBasis = new(cylinderRotation);
        Basis = WideRationalBasis3d.CreateRelative(cylinderBasis, sourceFrame.GeometryBasis);
        CoordinateDenominator = Signed320.NarrowValue(
            WideArithmetic.MultiplySigned320(sourceFrame.Denominator, cylinderBasis.Denominator));
        Cap = Signed320.NarrowValue(WideArithmetic.MultiplySigned320(CoordinateDenominator, height));
        Radius = WideArithmetic.AddSigned192(Signed192.Raw(radius), Signed192.Raw(radius));
    }

    internal WideAxis3 Transform(Vector3d point)
    {
        if (HasExactScale)
        {
            // Existing affine sources are nonrigid. The local identity chart
            // retains a wider rational denominator without changing its mapping.
            if (_sourceFrame.IsRigid)
            {
                Signed320 chartX = Signed320.NarrowValue(WideArithmetic.MultiplySigned320(CoordinateDenominator, Signed320.ExtendValue(Signed192.Raw(point.X))));
                Signed320 chartY = Signed320.NarrowValue(WideArithmetic.MultiplySigned320(CoordinateDenominator, Signed320.ExtendValue(Signed192.Raw(point.Y))));
                Signed320 chartZ = Signed320.NarrowValue(WideArithmetic.MultiplySigned320(CoordinateDenominator, Signed320.ExtendValue(Signed192.Raw(point.Z))));
                return new WideAxis3(WideArithmetic.AddSigned320(chartX, chartX), WideArithmetic.AddSigned320(chartY, chartY), WideArithmetic.AddSigned320(chartZ, chartZ));
            }
            _sourceFrame.GetPoint(point).GetLocalPointNumerators(_cylinderCenter, _cylinderRotation,
                out Signed576 px, out Signed576 py, out Signed576 pz, out _);
            // Near-field raw coordinates <2^46 and the common denominator
            // <2^259 leave doubled numerators below 306 magnitude bits.
            return new WideAxis3(Signed320.NarrowValue(WideArithmetic.AddSigned576(px, px)),
                Signed320.NarrowValue(WideArithmetic.AddSigned576(py, py)),
                Signed320.NarrowValue(WideArithmetic.AddSigned576(pz, pz)));
        }
        WideAxis3 local = WideRigidProjection.TransformLocalAxis(Basis,
            Signed192.Raw(point.X), Signed192.Raw(point.Y), Signed192.Raw(point.Z));
        Signed320 x = WideArithmetic.AddSigned320(local.X, Translation.X);
        Signed320 y = WideArithmetic.AddSigned320(local.Y, Translation.Y);
        Signed320 z = WideArithmetic.AddSigned320(local.Z, Translation.Z);
        return new WideAxis3(WideArithmetic.AddSigned320(x, x),
            WideArithmetic.AddSigned320(y, y), WideArithmetic.AddSigned320(z, z));
    }
}
