//=======================================================================
// WideConvex2dRelations.BoundaryFaces.cs
//=======================================================================
// MIT License, Copyright (c) 2024-present David Oravsky (mrdav30)
// See LICENSE file in the project root for full license information.
//=======================================================================

using System;

namespace FixedMathSharp.Geometry;

/// <content>Finite planar face witnesses clipped in their original exact edge parameter.</content>
internal static partial class WideConvex2dRelations
{
    private readonly struct BoundaryFacePoint
    {
        internal readonly Vector2d Start, End;
        internal readonly Signed576 N, D;
        internal readonly Signed832 Gap, Tangent;
        internal BoundaryFacePoint(Vector2d start, Vector2d end, Signed576 n, Signed576 d,
            Signed832 gap, Signed832 tangent)
        { Start = start; End = end; N = n; D = d; Gap = gap; Tangent = tangent; }
    }

    /// <summary>
    /// Returns the deepest one or two polygon witnesses against an oriented
    /// finite face. The retained face must actually intersect the polygon;
    /// this does not turn a remote supporting plane into a contact or recovery.
    /// A caller-proven crossing on the same connected cooriented supporting
    /// line may admit this supplier; finite tangent clipping still applies.
    /// </summary>
    internal static int GetBoundaryFaceContacts(Vector2d moverOrigin, Fixed64 moverRotation,
        ReadOnlySpan<Vector2d> moverOffsets, Vector2d targetOrigin, Fixed64 targetRotation,
        FixedSegment2d edge, Vector2d inward, in RationalSegmentInterval clip,
        Span<FixedContactAnchors2d> contacts, bool planeAdmitted = false)
    {
        if (contacts.Length < 2) throw new ArgumentException("Two contact slots are required.", nameof(contacts));
        if (!planeAdmitted && (!TryGetEdgeCoverage(targetOrigin, targetRotation, edge, inward,
            moverOrigin, moverRotation, moverOffsets, out RationalSegmentInterval overlap, out _)
            || !BoundaryIntervalsOverlap(clip, overlap))) return 0;

        RotationFrame2d frame = new(targetRotation);
        GetTransformedEdge(frame, edge.Start, edge.End, out Signed192 ex, out Signed192 ey);
        GetTransformedEdge(frame, edge.Start, inward, out Signed192 ix, out Signed192 iy);
        WideAxis2d outward = new(ey, WideArithmetic.Negate(ex));
        if (WideArithmetic.AddProducts(outward.X, ix, outward.Y, iy).Sign > 0) outward = -outward;
        Signed320 squared = WideArithmetic.AddProducts(ex, ex, ey, ey);
        if (squared.IsZero) return 0;
        Signed320 lowerN = clip.IsClipped ? clip.LowerNumerator : default;
        Signed320 lowerD = clip.IsClipped ? clip.LowerDenominator : Signed320.One;
        Signed320 upperN = clip.IsClipped ? clip.UpperNumerator : Signed320.One;
        Signed320 upperD = clip.IsClipped ? clip.UpperDenominator : Signed320.One;
        GetRelativePointNumerators(targetOrigin, targetRotation, edge.Start, targetOrigin,
            out Signed192 ax, out Signed192 ay);
        bool found = false;
        BoundaryFacePoint first = default, last = default;
        for (int index = 0; index < moverOffsets.Length; index++)
        {
            Vector2d start = moverOffsets[index], end = moverOffsets[(index + 1) % moverOffsets.Length];
            GetRelativePointNumerators(moverOrigin, moverRotation, start, targetOrigin, out Signed192 sx, out Signed192 sy);
            GetRelativePointNumerators(moverOrigin, moverRotation, end, targetOrigin, out Signed192 tx, out Signed192 ty);
            sx = WideArithmetic.SubtractSigned192(sx, ax); sy = WideArithmetic.SubtractSigned192(sy, ay);
            tx = WideArithmetic.SubtractSigned192(tx, ax); ty = WideArithmetic.SubtractSigned192(ty, ay);
            Signed320 u0 = WideArithmetic.AddProducts(sx, ex, sy, ey);
            Signed320 u1 = WideArithmetic.AddProducts(tx, ex, ty, ey);
            Signed576 ln = default, ld = Signed576.One, un = Signed576.One, ud = Signed576.One;
            if (!ClipBoundaryParameter(BoundaryWall(u0, squared, lowerN, lowerD), BoundaryWall(u1, squared, lowerN, lowerD),
                    ref ln, ref ld, ref un, ref ud)
                || !ClipBoundaryParameter(BoundaryWall(squared, u0, upperD, upperN), BoundaryWall(squared, u1, upperD, upperN),
                    ref ln, ref ld, ref un, ref ud)) continue;
            Signed320 g0 = WideArithmetic.Negate(WideArithmetic.AddProducts(sx, outward.X, sy, outward.Y));
            Signed320 g1 = WideArithmetic.Negate(WideArithmetic.AddProducts(tx, outward.X, ty, outward.Y));
            KeepBoundaryFacePoint(start, end, ln, ld, g0, g1, u0, u1, ref found, ref first, ref last);
            KeepBoundaryFacePoint(start, end, un, ud, g0, g1, u0, u1, ref found, ref first, ref last);
        }
        if (!found) return 0;
        Vector2d normal = WideNormalization.GetNormalized(WideArithmetic.Negate(outward.X), WideArithmetic.Negate(outward.Y));
        // Point/edge numerators share an S scale. Full Signed320 clip bounds
        // keep walls below 521 bits, point projections below 723 and depth
        // denominators below 554, fitting the existing Signed576/832 banks.
        Signed576 denominator = WideArithmetic.MultiplySigned576(first.D, Signed192.One);
        Fixed64 depth = WideArithmetic.GetRoundedNonNegativeNormalizedDepth(first.Gap,
            Signed832.ExtendValue(squared), denominator, out bool clamped);
        contacts[0] = MaterializeBoundaryFace(first, squared, moverOrigin, moverRotation,
            targetOrigin, targetRotation, edge, normal, depth, clamped);
        if (CompareBoundaryRatios(first.Tangent, first.D, last.Tangent, last.D) == 0) return 1;
        contacts[1] = MaterializeBoundaryFace(last, squared, moverOrigin, moverRotation,
            targetOrigin, targetRotation, edge, normal, depth, clamped);
        return 2;
    }

    private static bool BoundaryIntervalsOverlap(in RationalSegmentInterval first, in RationalSegmentInterval second)
    {
        if (!first.IsClipped || !second.IsClipped) return true;
        return RationalSegmentInterval.CompareParameters(first.LowerNumerator, first.LowerDenominator, second.UpperNumerator, second.UpperDenominator) <= 0
            && RationalSegmentInterval.CompareParameters(second.LowerNumerator, second.LowerDenominator, first.UpperNumerator, first.UpperDenominator) <= 0;
    }

    private static Signed576 BoundaryWall(Signed320 value, Signed320 plane, Signed320 n, Signed320 d) =>
        WideArithmetic.SubtractSigned576(WideArithmetic.MultiplySigned320(value, d), WideArithmetic.MultiplySigned320(plane, n));

    private static bool ClipBoundaryParameter(Signed576 first, Signed576 second,
        ref Signed576 ln, ref Signed576 ld, ref Signed576 un, ref Signed576 ud)
    {
        if (first.Sign >= 0 && second.Sign >= 0) return true;
        if (first.Sign < 0 && second.Sign < 0) return false;
        Signed576 n = first, d = WideArithmetic.SubtractSigned576(first, second);
        if (d.Sign < 0) { n = WideArithmetic.SubtractSigned576(default, n); d = WideArithmetic.SubtractSigned576(default, d); }
        // The ordered parallel lower/upper walls constrain opposite endpoints.
        if (first.Sign < 0) { ln = n; ld = d; }
        else { un = n; ud = d; }
        return CompareBoundaryRatios(Signed832.ExtendValue(ln), ld, Signed832.ExtendValue(un), ud) <= 0;
    }

    private static void KeepBoundaryFacePoint(Vector2d start, Vector2d end, Signed576 n, Signed576 d,
        Signed320 g0, Signed320 g1, Signed320 u0, Signed320 u1,
        ref bool found, ref BoundaryFacePoint first, ref BoundaryFacePoint last)
    {
        Signed832 gap = InterpolateBoundaryValue(g0, g1, n, d);
        if (gap.Sign < 0) return;
        Signed832 tangent = InterpolateBoundaryValue(u0, u1, n, d);
        var candidate = new BoundaryFacePoint(start, end, n, d, gap, tangent);
        int comparison = found ? CompareBoundaryRatios(gap, d, first.Gap, first.D) : 1;
        if (comparison > 0) { found = true; first = last = candidate; }
        else if (comparison == 0)
        {
            if (CompareBoundaryRatios(tangent, d, first.Tangent, first.D) < 0) first = candidate;
            if (CompareBoundaryRatios(tangent, d, last.Tangent, last.D) > 0) last = candidate;
        }
    }

    private static Signed832 InterpolateBoundaryValue(Signed320 start, Signed320 end, Signed576 n, Signed576 d) =>
        WideArithmetic.AddSigned832(WideArithmetic.MultiplySigned576ToSigned832(d, start),
            WideArithmetic.MultiplySigned576ToSigned832(n, WideArithmetic.SubtractSigned320(end, start)));

    private static int CompareBoundaryRatios(Signed832 n, Signed576 d, Signed832 otherN, Signed576 otherD) =>
        WideArithmetic.CompareNonNegativeProducts(n, Signed832.ExtendValue(otherD), otherN, Signed832.ExtendValue(d));

    private static FixedContactAnchors2d MaterializeBoundaryFace(in BoundaryFacePoint point, Signed320 squared,
        Vector2d moverOrigin, Fixed64 moverRotation, Vector2d targetOrigin, Fixed64 targetRotation,
        FixedSegment2d edge, Vector2d normal, Fixed64 depth, bool clamped)
    {
        Vector2d moving = new(GetSweepProjectedCoordinate(point.Start.X, point.End.X, point.N, point.D),
            GetSweepProjectedCoordinate(point.Start.Y, point.End.Y, point.N, point.D));
        Signed832 d = WideArithmetic.MultiplySigned576ToSigned832(point.D, squared);
        var parameter = new RationalSegmentInterval(point.Tangent, d, point.Tangent, d);
        return new FixedContactAnchors2d(new FixedPointAnchor2d(moverOrigin, moverRotation, moving),
            new FixedPointAnchor2d(targetOrigin, targetRotation, parameter.GetEndpoint(edge, false)), normal, depth, clamped);
    }
}
