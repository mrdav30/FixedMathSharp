//=======================================================================
// RationalPoint3d.cs
//=======================================================================
// MIT License, Copyright (c) 2024-present David Oravsky (mrdav30)
// See LICENSE file in the project root for full license information.
//=======================================================================

using System;

namespace FixedMathSharp.Geometry;

/// <summary>
/// Retains exact world-space raw-coordinate numerators over one positive
/// denominator; materialization rounds each final coordinate once to nearest even.
/// </summary>
internal readonly struct RationalPoint3d
{
    internal readonly Signed320 X, Y, Z, Denominator;

    internal RationalPoint3d(Signed320 x, Signed320 y, Signed320 z, Signed320 denominator)
    {
        X = x; Y = y; Z = z; Denominator = denominator;
    }

    internal static RationalPoint3d FromScalarPoint(Vector3d point) =>
        new(ScaleScalar(point.X), ScaleScalar(point.Y), ScaleScalar(point.Z), new Signed320(0UL, 0UL, 0UL, 1UL, 0UL));

    /// <summary>Retains an existing anchor's exact feature term in the shared Q² point domain.</summary>
    internal static RationalPoint3d FromAnchor(in FixedPointAnchor anchor)
    {
        WideTriangleRelations.GetPointLocalNumerators(anchor, Vector3d.Zero, FixedQuaternion.Identity,
            out Signed576 x, out Signed576 y, out Signed576 z, out Signed320 denominator);
        // An anchor combines a 65-bit rigid basis with a 34-bit exact local
        // term denominator. Its world numerator plus this lift fits 229 bits.
        Signed192 qSquared = new(0UL, 1UL, 0UL);
        return new RationalPoint3d(
            Signed320.NarrowValue(WideArithmetic.MultiplySigned576(x, qSquared)),
            Signed320.NarrowValue(WideArithmetic.MultiplySigned576(y, qSquared)),
            Signed320.NarrowValue(WideArithmetic.MultiplySigned576(z, qSquared)),
            Signed320.NarrowValue(WideArithmetic.MultiplySigned320(denominator, qSquared)));
    }

    /// <summary>Embeds a complete planar anchor in X/Z with zero Y, preserving sub-lattice features.</summary>
    internal static RationalPoint3d FromAnchor(in FixedPointAnchor2d anchor)
    {
        WidePointAnchor2d.GetExactRelativeOffsetRatio(anchor.Origin, anchor.LocalPoint, anchor.LocalDisplacement,
            anchor.ExactLocalTerm, anchor.Rotation, Vector2d.Zero, Vector2d.Zero, Vector2d.Zero, default,
            Fixed64.Zero, out Signed320 x, out Signed320 z, out Signed320 rawDenominator);
        // Both owners retain raw-coordinate ratios. The anchor's 2 Q²
        // denominator already belongs to the shared Q² point domain.
        return new(x, default, z, rawDenominator);
    }

    /// <summary>Compares exact world projections without materializing either point.</summary>
    internal static int CompareProjection(in RationalPoint3d first, in RationalPoint3d second, Vector3d direction)
    {
        WideDirection3d delta = WideTriangleRelations.GetScaledPointDifference(first, second,
            WideArithmetic.TruncateSigned320By64(second.Denominator));
        var axis = new WideAxis3(Signed320.ExtendValue(Signed192.Raw(direction.X)),
            Signed320.ExtendValue(Signed192.Raw(direction.Y)), Signed320.ExtendValue(Signed192.Raw(direction.Z)));
        return WideDirection3d.Dot(delta, axis).Sign;
    }

    private static Signed320 ScaleScalar(Fixed64 coordinate)
    {
        ulong extension = coordinate.m_rawValue < 0L ? ulong.MaxValue : 0UL;
        return new Signed320(extension, extension, extension, unchecked((ulong)coordinate.m_rawValue), 0UL);
    }

    internal bool TryGetPoint(out Vector3d point) =>
        TryMaterialize(Signed576.ExtendValue(X), Signed576.ExtendValue(Y), Signed576.ExtendValue(Z),
            Signed576.ExtendValue(Denominator), out point);

    /// <summary>
    /// Transposes a normalized target rotation after exact origin subtraction.
    /// Source numerators are below 319 magnitude bits, so the three products
    /// and their sum are below 386 bits; 576-bit temporaries retain every term.
    /// </summary>
    internal void GetLocalPointNumerators(Vector3d targetOrigin, FixedQuaternion targetRotation,
        out Signed576 x, out Signed576 y, out Signed576 z, out Signed320 denominator)
    {
        if (!targetRotation.IsNormalized())
            throw new ArgumentException("The target rotation must be normalized.", nameof(targetRotation));
        if (targetRotation == FixedQuaternion.Identity)
        {
            x = SubtractOrigin(X, targetOrigin.X);
            y = SubtractOrigin(Y, targetOrigin.Y);
            z = SubtractOrigin(Z, targetOrigin.Z);
            denominator = Denominator;
            return;
        }

        WideRationalBasis3d basis = new(targetRotation);
        Signed576 relativeX = SubtractOrigin(X, targetOrigin.X);
        Signed576 relativeY = SubtractOrigin(Y, targetOrigin.Y);
        Signed576 relativeZ = SubtractOrigin(Z, targetOrigin.Z);
        x = Project(relativeX, relativeY, relativeZ, basis.Xx, basis.Xy, basis.Xz);
        y = Project(relativeX, relativeY, relativeZ, basis.Yx, basis.Yy, basis.Yz);
        z = Project(relativeX, relativeY, relativeZ, basis.Zx, basis.Zy, basis.Zz);
        // Q^2 and three normalized quaternion denominators stay below 259 bits.
        denominator = Signed320.NarrowValue(WideArithmetic.MultiplySigned320(Denominator, basis.Denominator));
    }

    internal bool TryGetLocalPointIn(Vector3d targetOrigin, FixedQuaternion targetRotation, out Vector3d point)
    {
        GetLocalPointNumerators(targetOrigin, targetRotation, out Signed576 x, out Signed576 y, out Signed576 z,
            out Signed320 denominator);
        return TryMaterialize(x, y, z, Signed576.ExtendValue(denominator), out point);
    }

    /// <summary>Converts only lattice-exact coordinates, without admitting rounded geometry.</summary>
    internal bool TryGetExactLocalPointIn(Vector3d targetOrigin, FixedQuaternion targetRotation, out Vector3d point)
    {
        GetLocalPointNumerators(targetOrigin, targetRotation, out Signed576 x, out Signed576 y, out Signed576 z,
            out Signed320 denominator);
        return TryMaterialize(x, y, z, Signed576.ExtendValue(denominator), out point)
            && x.Equals(WideArithmetic.MultiplySigned320(denominator, Signed192.Raw(point.X)))
            && y.Equals(WideArithmetic.MultiplySigned320(denominator, Signed192.Raw(point.Y)))
            && z.Equals(WideArithmetic.MultiplySigned320(denominator, Signed192.Raw(point.Z)));
    }
    private Signed576 SubtractOrigin(Signed320 coordinate, Fixed64 origin) =>
        WideArithmetic.SubtractSigned576(Signed576.ExtendValue(coordinate),
            WideArithmetic.MultiplySigned320(Denominator, Signed192.Raw(origin)));

    private static Signed576 Project(Signed576 x, Signed576 y, Signed576 z,
        Signed192 axisX, Signed192 axisY, Signed192 axisZ) =>
        WideArithmetic.AddSigned576(WideArithmetic.AddSigned576(
            WideArithmetic.MultiplySigned576(x, axisX), WideArithmetic.MultiplySigned576(y, axisY)),
            WideArithmetic.MultiplySigned576(z, axisZ));

    private static bool TryMaterialize(Signed576 x, Signed576 y, Signed576 z, Signed576 denominator, out Vector3d point)
    {
        bool representable = Fixed64.TryGetSignedRawRatio(x, denominator, out Fixed64 px)
            & Fixed64.TryGetSignedRawRatio(y, denominator, out Fixed64 py)
            & Fixed64.TryGetSignedRawRatio(z, denominator, out Fixed64 pz);
        point = representable ? new Vector3d(px, py, pz) : default;
        return representable;
    }
}
