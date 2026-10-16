//=======================================================================
// WideTriangleRelations.EdgeCoverage.cs
//=======================================================================
// MIT License, Copyright (c) 2024-present David Oravsky (mrdav30)
// See LICENSE file in the project root for full license information.
//=======================================================================
using System;

namespace FixedMathSharp.Geometry;

/// <content>Exact coplanar triangle coverage of an independently framed edge.</content>
internal static partial class WideTriangleRelations
{
    /// <summary>
    /// Clips source A/B against target, preserving the original unit parameter.
    /// Bits 1/2 denote coplanar target fill on source C's/opposite side;
    /// bit 4 denotes a folded target incident along the source edge.
    /// Requires normalized rigid rotations and raw authored triangle vertices.
    /// Degenerate, off-plane and disjoint edges return false; singleton
    /// intersections remain closed and do not imply positive-length coverage.
    /// </summary>
    internal static bool TryGetEdgeCoverage(in FixedTriangle source, Vector3d sourceOrigin,
        FixedQuaternion sourceRotation, in FixedTriangle target, Vector3d targetOrigin,
        FixedQuaternion targetRotation, out RationalSegmentInterval interval, out int sides)
    {
        interval = default; sides = 0;
        source.GetExactNormal(out _, out _, out _, out Signed320 sourceNormalSquared);
        target.GetExactNormal(out Signed192 nx, out Signed192 ny, out Signed192 nz, out Signed320 targetNormalSquared);
        if (sourceNormalSquared.IsZero || targetNormalSquared.IsZero) return false;
        int axis = !nx.IsZero ? 0 : !ny.IsZero ? 1 : 2;
        int u = (axis + 1) % 3, v = (axis + 2) % 3;
        Span<Vector3d> vertices = stackalloc Vector3d[3] { target.A, target.B, target.C };
        Span<Signed320> points = stackalloc Signed320[9];
        GetCoveragePoint(source.A, sourceOrigin, sourceRotation, targetOrigin, targetRotation, points, out Signed320 d);
        GetCoveragePoint(source.B, sourceOrigin, sourceRotation, targetOrigin, targetRotation, points.Slice(3), out _);
        GetCoveragePoint(source.C, sourceOrigin, sourceRotation, targetOrigin, targetRotation, points.Slice(6), out _);
        // A/B must lie in the target plane. C distinguishes a coplanar fill
        // from a folded incident sheet, which must not disappear at a seam.
        for (int point = 0; point < 3; point++)
        {
            Signed576 plane = WideArithmetic.AddSigned576(WideArithmetic.AddSigned576(
                WideArithmetic.MultiplySigned576(Signed576.ExtendValue(CoverageDifference(points[3 * point], target.A.X, d)), nx),
                WideArithmetic.MultiplySigned576(Signed576.ExtendValue(CoverageDifference(points[3 * point + 1], target.A.Y, d)), ny)),
                WideArithmetic.MultiplySigned576(Signed576.ExtendValue(CoverageDifference(points[3 * point + 2], target.A.Z, d)), nz));
            if (plane.IsZero) continue;
            if (point != 2) return false;
            sides = 4;
        }
        Signed320 dx = WideArithmetic.SubtractSigned320(points[3 + u], points[u]);
        Signed320 dy = WideArithmetic.SubtractSigned320(points[3 + v], points[v]);
        int inward = CoverageCross(dx, dy, WideArithmetic.SubtractSigned320(points[6 + u], points[u]),
            WideArithmetic.SubtractSigned320(points[6 + v], points[v])).Sign;
        int orientation = (axis == 0 ? nx : axis == 1 ? ny : nz).Sign;
        Signed320 lowerN = default, lowerD = Signed320.One, upperN = Signed320.One, upperD = Signed320.One;
        for (int edge = 0; edge < 3; edge++)
        {
            Vector3d a = vertices[edge], b = vertices[(edge + 1) % 3];
            Signed320 f0 = CoverageHalfPlane(a, b, points[u], points[v], d, u, v, orientation);
            Signed320 f1 = CoverageHalfPlane(a, b, points[3 + u], points[3 + v], d, u, v, orientation);
            if (!RationalSegmentInterval.TryClipHalfPlane(f0, f1, ref lowerN, ref lowerD, ref upperN, ref upperD)) return false;
            if (sides == 4) continue;
            int side = inward * CoverageCross(dx, dy,
                CoverageDifference(points[u], a[u], d), CoverageDifference(points[v], a[v], d)).Sign;
            // The differences above point FROM the vertex to source A.
            if (side < 0) sides |= 1;
            if (side > 0) sides |= 2;
        }
        interval = new RationalSegmentInterval(lowerN, lowerD, upperN, upperD);
        return true;
    }

    private static void GetCoveragePoint(Vector3d point, Vector3d origin, FixedQuaternion rotation,
        Vector3d targetOrigin, FixedQuaternion targetRotation, Span<Signed320> coordinates, out Signed320 d)
    {
        GetPointLocalNumerators(new FixedPointAnchor(origin, rotation, point), targetOrigin, targetRotation,
            out Signed576 x, out Signed576 y, out Signed576 z, out d);
        // Normalized raw-point frames: D < 132 bits, coordinates < 200 bits.
        coordinates[0] = Signed320.NarrowValue(x);
        coordinates[1] = Signed320.NarrowValue(y);
        coordinates[2] = Signed320.NarrowValue(z);
    }

    private static Signed320 CoverageDifference(Signed320 mapped, Fixed64 raw, Signed320 d) =>
        WideArithmetic.SubtractSigned320(mapped, Signed320.NarrowValue(
            WideArithmetic.MultiplySigned320(d, Signed320.ExtendValue(Signed192.Raw(raw)))));

    private static Signed576 CoverageCross(Signed320 dx, Signed320 dy, Signed320 x, Signed320 y) =>
        WideArithmetic.SubtractSigned576(WideArithmetic.MultiplySigned320(dx, y), WideArithmetic.MultiplySigned320(dy, x));

    private static Signed320 CoverageHalfPlane(Vector3d a, Vector3d b, Signed320 x, Signed320 y,
        Signed320 d, int u, int v, int orientation)
    {
        Signed192 dx = WideArithmetic.SubtractSigned192(Signed192.Raw(b[u]), Signed192.Raw(a[u]));
        Signed192 dy = WideArithmetic.SubtractSigned192(Signed192.Raw(b[v]), Signed192.Raw(a[v]));
        Signed576 value = WideArithmetic.SubtractSigned576(
            WideArithmetic.MultiplySigned576(Signed576.ExtendValue(CoverageDifference(y, a[v], d)), dx),
            WideArithmetic.MultiplySigned576(Signed576.ExtendValue(CoverageDifference(x, a[u], d)), dy));
        // Raw edges < 65 bits and mapped differences < 201: F and root N/D
        // stay below 268 bits. Side crosses stay below 403 bits (Signed576).
        return Signed320.NarrowValue(orientation < 0 ? WideArithmetic.SubtractSigned576(default, value) : value);
    }

}
