//=======================================================================
// WideConvex2dRelations.PlanarSweep.cs
//=======================================================================
// MIT License, Copyright (c) 2024-present David Oravsky (mrdav30)
// See LICENSE file in the project root for full license information.
//=======================================================================

using System;

namespace FixedMathSharp.Geometry;

/// <content>Exact continuous upright-capsule coverage in the plane.</content>
internal static partial class WideConvex2dRelations
{
    internal static bool IntersectsSweptUprightCapsule(
        Vector2d start, Vector2d end, Fixed64 axisLength, Fixed64 radius,
        Vector2d origin, ReadOnlySpan<Vector2d> offsets, bool strict)
    {
        int winding = GetPlanarSweepWinding(offsets);
        if (winding == 0)
            return false;

        // The swept axis is [start,end] + [-axis/2,+axis/2]Forward.
        // Doubled raw coordinates retain odd half-axis lengths exactly.
        // Relative coordinates/edge differences are below 2^67 in magnitude;
        // dot/cross products below 2^135 and squared comparisons below 2^271.
        // Signed192/320/576 therefore cover the entire public scalar domain.
        WideAxis2d first = new(
            WideArithmetic.Double(WideArithmetic.Difference(start.X, origin.X)),
            WideArithmetic.Double(WideArithmetic.Difference(start.Y, origin.Y)));
        WideAxis2d last = new(
            WideArithmetic.Double(WideArithmetic.Difference(end.X, origin.X)),
            WideArithmetic.Double(WideArithmetic.Difference(end.Y, origin.Y)));
        Signed192 length = Signed192.Raw(axisLength);
        Span<WideAxis2d> core = stackalloc WideAxis2d[4];
        core[0] = new(first.X, WideArithmetic.SubtractSigned192(first.Y, length));
        core[1] = new(last.X, WideArithmetic.SubtractSigned192(last.Y, length));
        core[2] = new(last.X, WideArithmetic.AddSigned192(last.Y, length));
        core[3] = new(first.X, WideArithmetic.AddSigned192(first.Y, length));

        if (HasPlanarSweepCoreOverlap(core, offsets, winding, strict: strict && radius == Fixed64.Zero))
            return true;
        if (radius == Fixed64.Zero)
            return false;

        Signed192 doubledRadius = WideArithmetic.Double(Signed192.Raw(radius));
        Signed320 radiusSquared = WideArithmetic.MultiplySigned192(doubledRadius, doubledRadius);
        int maximumDistanceSign = strict ? -1 : 0;
        // Disjoint convex sets have an endpoint-to-edge closest pair.
        // Four edges suffice even when the core degenerates to a segment/point.
        for (int i = 0; i < offsets.Length; i++)
        {
            WideAxis2d a = DoubleSweepOffset(offsets[i]);
            WideAxis2d b = DoubleSweepOffset(offsets[i + 1 == offsets.Length ? 0 : i + 1]);
            for (int j = 0; j < core.Length; j++)
            {
                if (IsSweepPointNearSegment(core[j], a, b, radiusSquared, maximumDistanceSign)
                    || IsSweepPointNearSegment(a, core[j], core[(j + 1) & 3], radiusSquared, maximumDistanceSign))
                    return true;
            }
        }
        return false;
    }

    private static bool HasPlanarSweepCoreOverlap(
        ReadOnlySpan<WideAxis2d> core, ReadOnlySpan<Vector2d> offsets, int winding, bool strict)
    {
        // Polygon edge halfspaces need only the four core projections: O(n),
        // not a full polygon projection for each of its own edge normals.
        for (int i = 0; i < offsets.Length; i++)
        {
            WideAxis2d a = DoubleSweepOffset(offsets[i]);
            WideAxis2d edge = SubtractSweepPoint(
                DoubleSweepOffset(offsets[i + 1 == offsets.Length ? 0 : i + 1]), a);
            if (edge.IsZero)
                continue;
            bool enters = false;
            for (int j = 0; j < core.Length; j++)
            {
                int side = SweepCross(edge, SubtractSweepPoint(core[j], a)).Sign * winding;
                if (strict ? side > 0 : side >= 0)
                {
                    enters = true;
                    break;
                }
            }
            if (!enters)
                return false;
        }

        // The other separating axes are the two distinct core edge normals.
        for (int edgeIndex = 0; edgeIndex < 2; edgeIndex++)
        {
            WideAxis2d edge = SubtractSweepPoint(core[edgeIndex + 1], core[edgeIndex]);
            if (edge.IsZero)
                continue;
            Signed320 coreMin = SweepCross(edge, core[0]);
            Signed320 coreMax = coreMin;
            for (int i = 1; i < core.Length; i++)
                IncludeSweepProjection(SweepCross(edge, core[i]), ref coreMin, ref coreMax);
            Signed320 polygonMin = SweepCross(edge, DoubleSweepOffset(offsets[0]));
            Signed320 polygonMax = polygonMin;
            for (int i = 1; i < offsets.Length; i++)
                IncludeSweepProjection(SweepCross(edge, DoubleSweepOffset(offsets[i])), ref polygonMin, ref polygonMax);
            int firstGap = WideArithmetic.SubtractSigned320(coreMax, polygonMin).Sign;
            int secondGap = WideArithmetic.SubtractSigned320(polygonMax, coreMin).Sign;
            if (strict ? firstGap <= 0 || secondGap <= 0 : firstGap < 0 || secondGap < 0)
                return false;
        }
        return true;
    }

    private static bool IsSweepPointNearSegment(
        WideAxis2d point, WideAxis2d start, WideAxis2d end, Signed320 radiusSquared, int maximumDistanceSign)
    {
        WideAxis2d relative = SubtractSweepPoint(point, start);
        WideAxis2d edge = SubtractSweepPoint(end, start);
        Signed320 projection = SweepDot(relative, edge);
        Signed320 lengthSquared = SweepDot(edge, edge);
        if (projection.Sign <= 0)
            return WideArithmetic.SubtractSigned320(SweepDot(relative, relative), radiusSquared).Sign <= maximumDistanceSign;
        if (WideArithmetic.SubtractSigned320(projection, lengthSquared).Sign >= 0)
        {
            relative = SubtractSweepPoint(point, end);
            return WideArithmetic.SubtractSigned320(SweepDot(relative, relative), radiusSquared).Sign <= maximumDistanceSign;
        }
        Signed320 cross = SweepCross(edge, relative);
        return WideArithmetic.SubtractSigned576(
            WideArithmetic.MultiplySigned320(cross, cross),
            WideArithmetic.MultiplySigned320(radiusSquared, lengthSquared)).Sign <= maximumDistanceSign;
    }

    private static int GetPlanarSweepWinding(ReadOnlySpan<Vector2d> offsets)
    {
        // A fan around the first vertex also tolerates repeated edge vertices.
        WideAxis2d first = DoubleSweepOffset(offsets[0]);
        for (int i = 1; i < offsets.Length - 1; i++)
        {
            int sign = SweepCross(
                SubtractSweepPoint(DoubleSweepOffset(offsets[i]), first),
                SubtractSweepPoint(DoubleSweepOffset(offsets[i + 1]), first)).Sign;
            if (sign != 0)
                return sign;
        }
        return 0;
    }

    private static void IncludeSweepProjection(Signed320 value, ref Signed320 minimum, ref Signed320 maximum)
    {
        if (WideArithmetic.SubtractSigned320(value, minimum).Sign < 0)
            minimum = value;
        if (WideArithmetic.SubtractSigned320(value, maximum).Sign > 0)
            maximum = value;
    }

    private static WideAxis2d DoubleSweepOffset(Vector2d value) => new(
        WideArithmetic.Double(Signed192.Raw(value.X)), WideArithmetic.Double(Signed192.Raw(value.Y)));

    private static WideAxis2d SubtractSweepPoint(WideAxis2d first, WideAxis2d second) => new(
        WideArithmetic.SubtractSigned192(first.X, second.X), WideArithmetic.SubtractSigned192(first.Y, second.Y));

    private static Signed320 SweepCross(WideAxis2d first, WideAxis2d second) =>
        WideArithmetic.MultiplySubtract(first.X, second.Y, first.Y, second.X);

    private static Signed320 SweepDot(WideAxis2d first, WideAxis2d second) =>
        WideArithmetic.AddProducts(first.X, second.X, first.Y, second.Y);
}
