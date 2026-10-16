//=======================================================================
// WideConvex2dRelations.EdgeCoverage.cs
//=======================================================================
// MIT License, Copyright (c) 2024-present David Oravsky (mrdav30)
// See LICENSE file in the project root for full license information.
//=======================================================================
using System;

namespace FixedMathSharp.Geometry;

/// <content>Exact independently framed convex coverage of an original planar edge.</content>
internal static partial class WideConvex2dRelations
{
    /// <summary>
    /// Clips the source edge against a convex polygon without materializing
    /// world vertices. Bits 1/2 denote target fill on inward's/opposite side.
    /// A singleton remains closed but does not fill an open neighborhood.
    /// Uses the same scalar Sin/Cos geometry as planar convex relations.
    /// </summary>
    internal static bool TryGetEdgeCoverage(Vector2d sourceOrigin, Fixed64 sourceRotation,
        FixedSegment2d source, Vector2d inward, Vector2d targetOrigin, Fixed64 targetRotation,
        ReadOnlySpan<Vector2d> targetOffsets, out RationalSegmentInterval interval, out int sides)
    {
        interval = default; sides = 0;
        if (targetOffsets.Length < 3) return false;
        int winding = GetPlanarSweepWinding(targetOffsets);
        if (winding == 0) return false;
        WideAxis2d a = CoveragePoint(sourceOrigin, sourceRotation, source.Start, sourceOrigin);
        WideAxis2d b = CoveragePoint(sourceOrigin, sourceRotation, source.End, sourceOrigin);
        WideAxis2d edge = SubtractSweepPoint(b, a);
        int inwardSide = SweepCross(edge, SubtractSweepPoint(
            CoveragePoint(sourceOrigin, sourceRotation, inward, sourceOrigin), a)).Sign;
        if (inwardSide == 0) return false;
        Signed320 lowerN = default, lowerD = Signed320.One, upperN = Signed320.One, upperD = Signed320.One;
        WideAxis2d first = CoveragePoint(targetOrigin, targetRotation, targetOffsets[0], sourceOrigin);
        for (int index = 0; index < targetOffsets.Length; index++)
        {
            WideAxis2d next = CoveragePoint(targetOrigin, targetRotation,
                targetOffsets[(index + 1) % targetOffsets.Length], sourceOrigin);
            WideAxis2d targetEdge = SubtractSweepPoint(next, first);
            Signed320 f0 = SweepCross(targetEdge, SubtractSweepPoint(a, first));
            Signed320 f1 = SweepCross(targetEdge, SubtractSweepPoint(b, first));
            if (winding < 0) { f0 = WideArithmetic.Negate(f0); f1 = WideArithmetic.Negate(f1); }
            if (!RationalSegmentInterval.TryClipHalfPlane(f0, f1,
                ref lowerN, ref lowerD, ref upperN, ref upperD)) return false;
            int side = inwardSide * SweepCross(edge, SubtractSweepPoint(first, a)).Sign;
            if (side > 0) sides |= 1;
            if (side < 0) sides |= 2;
            first = next;
        }
        // Shared S=2^32 coordinates require fewer than 99 bits for any raw
        // origin/offset. Their determinants and root differences fit 200 bits;
        // Signed320 retains the exact unit parameter without a rounded chart.
        interval = new RationalSegmentInterval(lowerN, lowerD, upperN, upperD);
        return true;
    }

    private static WideAxis2d CoveragePoint(Vector2d origin, Fixed64 rotation, Vector2d point, Vector2d reference)
    {
        GetRelativePointNumerators(origin, rotation, point, reference, out Signed192 x, out Signed192 y);
        return new WideAxis2d(x, y);
    }
}
