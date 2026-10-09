//=======================================================================
// WidePlaneMetrics.cs
//=======================================================================
// MIT License, Copyright (c) 2024-present David Oravsky (mrdav30)
// See LICENSE file in the project root for full license information.
//=======================================================================
using System;

namespace FixedMathSharp.Geometry;

/// <summary>Exact metrics of representable points projected onto an exact rigid plane.</summary>
internal readonly struct WidePlaneMetrics
{
    private readonly WideAxis3 normal;
    private readonly Signed576 metric;

    internal WidePlaneMetrics(FixedTriangle plane, FixedQuaternion rotation)
    {
        plane.GetExactNormal(out Signed192 x, out Signed192 y, out Signed192 z, out _);
        if (x.IsZero && y.IsZero && z.IsZero)
            throw new ArgumentException("The metric requires a nondegenerate plane.", nameof(plane));
        var basis = new WideRationalBasis3d(rotation);
        if (basis.Denominator.IsZero)
            throw new ArgumentException("The metric requires a nonzero rotation.", nameof(rotation));
        normal = WideRigidProjection.TransformLocalAxis(basis, x, y, z);
        metric = normal.SquaredLength;
    }

    // Raw endpoint differences <2^64, authored normal components <2^130
    // and arbitrary quaternion basis components <2^130 give N<2^262.
    // M=N.N<2^526; G(u,v)=M(u.v)-(N.u)(N.v)<2^657;
    // T(u,v)=N.(u cross v)<2^393 and T²<2^786. Signed832 retains
    // every metric. Ratio comparison uses the existing 26-word products.

    /// <summary>Returns M times squared projected distance in raw coordinate units.</summary>
    internal Signed832 GetSpanSquared(Vector3d first, Vector3d second)
    {
        WideAxis3 edge = Difference(second, first);
        return Gram(edge, edge);
    }

    /// <summary>Returns 4M times squared projected triangle area in raw coordinate units.</summary>
    internal Signed832 GetAreaSquared(Vector3d first, Vector3d second, Vector3d third)
    {
        Signed576 area = Triple(Difference(second, first), Difference(third, first));
        return WideArithmetic.MultiplySigned576ToSigned832(area, area);
    }

    /// <summary>
    /// Returns exact squared distance between the projected point and filled
    /// projected triangle as a nonnegative numerator over a positive denominator,
    /// in raw coordinate units squared. Degenerate projections reduce to edges.
    /// </summary>
    internal void GetTriangleDistanceSquared(Vector3d point, Vector3d first, Vector3d second, Vector3d third,
        out Signed832 numerator, out Signed832 denominator)
    {
        WideAxis3 edge = Difference(second, first), offset = Difference(point, first);
        int orientation = Triple(edge, Difference(third, first)).Sign;
        if (orientation != 0
            && orientation * Triple(edge, offset).Sign >= 0
            && orientation * Triple(Difference(third, second), Difference(point, second)).Sign >= 0
            && orientation * Triple(Difference(first, third), Difference(point, third)).Sign >= 0)
        {
            numerator = default;
            denominator = Signed832.ExtendValue(metric);
            return;
        }
        GetSegmentDistanceSquared(point, first, second, out numerator, out denominator);
        GetSegmentDistanceSquared(point, second, third, out Signed832 nextNumerator, out Signed832 nextDenominator);
        if (CompareRatios(nextNumerator, nextDenominator, numerator, denominator) < 0)
        {
            numerator = nextNumerator; denominator = nextDenominator;
        }
        GetSegmentDistanceSquared(point, third, first, out nextNumerator, out nextDenominator);
        if (CompareRatios(nextNumerator, nextDenominator, numerator, denominator) < 0)
        {
            numerator = nextNumerator; denominator = nextDenominator;
        }
    }

    /// <summary>Compares nonnegative metric ratios with positive denominators.</summary>
    internal static int CompareRatios(Signed832 firstNumerator, Signed832 firstDenominator,
        Signed832 secondNumerator, Signed832 secondDenominator) =>
        WideArithmetic.CompareNonNegativeProducts(firstNumerator, secondDenominator, secondNumerator, firstDenominator);

    private void GetSegmentDistanceSquared(Vector3d point, Vector3d first, Vector3d second,
        out Signed832 numerator, out Signed832 denominator)
    {
        WideAxis3 edge = Difference(second, first), offset = Difference(point, first);
        Signed832 length = Gram(edge, edge), projection = Gram(offset, edge);
        if (length.IsZero || projection.Sign <= 0)
        {
            numerator = Gram(offset, offset);
            denominator = Signed832.ExtendValue(metric);
        }
        else if (WideArithmetic.SubtractSigned832(projection, length).Sign >= 0)
        {
            offset = Difference(point, second);
            numerator = Gram(offset, offset);
            denominator = Signed832.ExtendValue(metric);
        }
        else
        {
            // G(v,v)G(e,e)-G(v,e)² = M*T(v,e)². Cancel M before
            // forming a numerator, avoiding an unnecessary >832-bit product.
            Signed576 area = Triple(offset, edge);
            numerator = WideArithmetic.MultiplySigned576ToSigned832(area, area);
            denominator = length;
        }
    }

    private Signed832 Gram(WideAxis3 first, WideAxis3 second) =>
        WideArithmetic.SubtractSigned832(
            WideArithmetic.MultiplySigned576ToSigned832(metric, WideAxis3.Dot(first, second)),
            WideArithmetic.MultiplySigned576ToSigned832(WideAxis3.Dot(normal, first), WideAxis3.Dot(normal, second)));

    private Signed576 Triple(WideAxis3 first, WideAxis3 second) => WideAxis3.Dot(normal, WideAxis3.Cross(first, second));

    private static WideAxis3 Difference(Vector3d end, Vector3d start) => new(
        Signed320.ExtendValue(WideArithmetic.SubtractSigned192(Signed192.Raw(end.X), Signed192.Raw(start.X))),
        Signed320.ExtendValue(WideArithmetic.SubtractSigned192(Signed192.Raw(end.Y), Signed192.Raw(start.Y))),
        Signed320.ExtendValue(WideArithmetic.SubtractSigned192(Signed192.Raw(end.Z), Signed192.Raw(start.Z))));
}
