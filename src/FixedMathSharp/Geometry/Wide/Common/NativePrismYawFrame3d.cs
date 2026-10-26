//=======================================================================
// MIT License, Copyright (c) 2024-present David Oravsky (mrdav30)
// See LICENSE file in the project root for full license information.
//=======================================================================

using System;

namespace FixedMathSharp.Geometry;

/// <summary>Forward exact geometry needed by hull projection and finite witnesses.</summary>
internal interface IForwardHullFrame3d
{
    Signed192 RotationDenominator { get; }
    Signed320 Denominator { get; }
    RationalPoint3d GetPoint(Vector3d point);
    WideDirection3d GetFaceNormal(in FixedTriangle triangle);
    WideDirection3d GetEdgeDirection(Vector3d first, Vector3d second);
    WideDirection3d GetInwardAxis(in FixedTriangle triangle, Vector3d first, Vector3d second);
    bool TryGetTriangleHalfPlanes(in FixedTriangle triangle, in RationalPoint3d point, Span<Signed832> values);
}

/// <summary>Preserves the scalar Sin/Cos prism chart without asserting a rigid inverse.</summary>
internal readonly struct NativePrismYawFrame3d : IForwardHullFrame3d
{
    private static readonly Signed192 Unit = Signed192.Raw(Fixed64.One);
    private static readonly Signed320 CoordinateDenominator = new(0UL, 0UL, 0UL, 1UL, 0UL);
    private readonly RationalPoint3d _origin;
    private readonly WideConvex2dRelations.RotationFrame2d _yaw;

    internal NativePrismYawFrame3d(Vector3d origin, Fixed64 yaw)
    {
        _origin = RationalPoint3d.FromScalarPoint(origin);
        _yaw = new(yaw);
    }

    public Signed192 RotationDenominator => Signed192.Signed(1);
    public Signed320 Denominator => CoordinateDenominator;

    public RationalPoint3d GetPoint(Vector3d point)
    {
        WideConvex2dRelations.GetRotatedOffset(_yaw, new(point.X, point.Z), out Signed192 x, out Signed192 z);
        RationalPoint3d scalar = RationalPoint3d.FromScalarPoint(point);
        // Native products have one Q denominator; lift them into the existing
        // Q² point domain before adding the absolute origin (under 129 bits).
        return new(
            WideArithmetic.AddSigned320(_origin.X, WideArithmetic.MultiplySigned192(x, Unit)),
            WideArithmetic.AddSigned320(_origin.Y, scalar.Y),
            WideArithmetic.AddSigned320(_origin.Z, WideArithmetic.MultiplySigned192(z, Unit)), Denominator);
    }

    public WideDirection3d GetEdgeDirection(Vector3d first, Vector3d second)
    {
        WideConvex2dRelations.GetRotatedOffset(_yaw, new(first.X, first.Z), out Signed192 ax, out Signed192 az);
        WideConvex2dRelations.GetRotatedOffset(_yaw, new(second.X, second.Z), out Signed192 bx, out Signed192 bz);
        Signed192 y = WideArithmetic.SubtractSigned192(Signed192.Raw(second.Y), Signed192.Raw(first.Y));
        return new(Signed576.ExtendValue(WideArithmetic.SubtractSigned192(bx, ax)),
            WideArithmetic.MultiplySigned320(Signed320.ExtendValue(y), Unit),
            Signed576.ExtendValue(WideArithmetic.SubtractSigned192(bz, az)));
    }

    public WideDirection3d GetFaceNormal(in FixedTriangle triangle)
    {
        triangle.GetExactNormal(out Signed192 x, out Signed192 y, out Signed192 z, out _);
        Signed320 nx = WideArithmetic.SubtractSigned320(WideArithmetic.MultiplySigned192(x, _yaw.Cosine),
            WideArithmetic.MultiplySigned192(z, _yaw.Sine));
        Signed320 nz = WideArithmetic.AddSigned320(WideArithmetic.MultiplySigned192(x, _yaw.Sine),
            WideArithmetic.MultiplySigned192(z, _yaw.Cosine));
        Signed320 radialMetric = WideArithmetic.AddSigned320(WideArithmetic.MultiplySigned192(_yaw.Cosine, _yaw.Cosine),
            WideArithmetic.MultiplySigned192(_yaw.Sine, _yaw.Sine));
        // Cofactors preserve the actual nonunit scalar matrix: c²+s² need
        // not equal Q². Raw face normals need <131 bits, so outputs need <197.
        return new(WideArithmetic.MultiplySigned320(nx, Unit),
            WideArithmetic.MultiplySigned320(radialMetric, y), WideArithmetic.MultiplySigned320(nz, Unit));
    }

    public WideDirection3d GetInwardAxis(in FixedTriangle triangle, Vector3d first, Vector3d second) =>
        WideDirection3d.Cross(GetFaceNormal(triangle), GetEdgeDirection(first, second));

    public bool TryGetTriangleHalfPlanes(in FixedTriangle triangle, in RationalPoint3d point, Span<Signed832> values)
    {
        WideDirection3d normal = GetFaceNormal(triangle);
        if (normal.IsZero) return false;
        Span<Vector3d> vertices = stackalloc Vector3d[3] { triangle.A, triangle.B, triangle.C };
        for (int edge = 0; edge < 3; edge++)
        {
            Vector3d first = vertices[edge], second = vertices[(edge + 1) % 3];
            WideDirection3d inward = WideDirection3d.Cross(normal, GetEdgeDirection(first, second));
            WideDirection3d delta = WideTriangleRelations.GetScaledPointDifference(point, GetPoint(first), RotationDenominator);
            // Native inward axes need <295 bits; even affine point differences
            // keep the complete dot below the existing 832-bit half-plane bank.
            values[edge] = WideDirection3d.Dot(inward, delta);
        }
        return true;
    }
}
