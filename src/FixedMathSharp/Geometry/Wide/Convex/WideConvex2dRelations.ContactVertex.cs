//=======================================================================
// WideConvex2dRelations.ContactVertex.cs
//=======================================================================
// MIT License, Copyright (c) 2024-present David Oravsky (mrdav30)
// See LICENSE file in the project root for full license information.
//=======================================================================
using System;

namespace FixedMathSharp.Geometry;

/// <content>Exact normal admission against independently framed incident convex sectors.</content>
internal static partial class WideConvex2dRelations
{
    /// <summary>
    /// Returns a finite polygon contact with an original target vertex. All
    /// incident points are plain native vertex anchors; their exact inward
    /// rays restrict response normals before normalization or depth ranking.
    /// </summary>
    internal static bool TryGetBoundaryVertexContact(Vector2d moverOrigin, Fixed64 moverRotation,
        ReadOnlySpan<Vector2d> moverOffsets, in FixedPointAnchor2d vertex,
        ReadOnlySpan<FixedPointAnchor2d> incidentPoints, out FixedContactAnchors2d contact)
    {
        RotationFrame2d moverFrame = new(moverRotation), targetFrame = new(vertex.Rotation);
        PenetrationAxis best = default;
        Vector2d bestPoint = default;
        for (int edge = 0; edge < moverOffsets.Length; edge++)
        {
            GetTransformedEdge(moverFrame, moverOffsets[edge], moverOffsets[(edge + 1) % moverOffsets.Length],
                out Signed192 ex, out Signed192 ey);
            WideAxis2d axis = new(ey, WideArithmetic.Negate(ex));
            if (axis.IsZero) continue;
            GetProjectionRange(moverOrigin, moverFrame, moverOffsets, axis, out Signed320 minimum, out Signed320 maximum);
            Signed320 point = GetProjection(vertex.Origin, targetFrame, vertex.LocalPoint, axis);
            Signed320 positive = WideArithmetic.SubtractSigned320(maximum, point);
            Signed320 negative = WideArithmetic.SubtractSigned320(point, minimum);
            // A forbidden response direction remains a valid separation axis.
            if (positive.Sign < 0 || negative.Sign < 0) { contact = default; return false; }
            KeepBoundaryVertexAxis(moverOrigin, moverRotation, moverOffsets, vertex, incidentPoints,
                axis, positive, ref best, ref bestPoint);
            KeepBoundaryVertexAxis(moverOrigin, moverRotation, moverOffsets, vertex, incidentPoints,
                -axis, negative, ref best, ref bestPoint);
        }
        if (!best.HasValue) { contact = default; return false; }
        GetDepth(best.Overlap, best.AxisSquared, out Fixed64 depth, out bool clamped);
        contact = new FixedContactAnchors2d(new FixedPointAnchor2d(moverOrigin, moverRotation, bestPoint),
            vertex, WideNormalization.GetNormalized(best.Axis.X, best.Axis.Y), depth, clamped);
        return true;
    }

    private static void KeepBoundaryVertexAxis(Vector2d origin, Fixed64 rotation, ReadOnlySpan<Vector2d> offsets,
        in FixedPointAnchor2d vertex, ReadOnlySpan<FixedPointAnchor2d> incidentPoints,
        WideAxis2d axis, Signed320 overlap, ref PenetrationAxis best, ref Vector2d bestPoint)
    {
        if (!SupportsBoundaryFan(axis.X, axis.Y, vertex, incidentPoints)) return;
        GetSupportFeature(origin, new RotationFrame2d(rotation), offsets, axis, true, out Feature feature);
        if (!TryProjectAnchorOntoFeature(vertex, origin, rotation, feature.Start, feature.End,
            requireInteriorProjection: true, out Vector2d point)) return;
        Signed320 squared = WideArithmetic.AddProducts(axis.X, axis.X, axis.Y, axis.Y);
        Signed576 overlapSquared = WideArithmetic.MultiplySigned320(overlap, overlap);
        if (best.HasValue && WideArithmetic.SubtractSigned832(
            WideArithmetic.MultiplySigned576ToSigned832(overlapSquared, best.AxisSquared),
            WideArithmetic.MultiplySigned576ToSigned832(best.OverlapSquared, squared)).Sign >= 0) return;
        best = new PenetrationAxis(axis, overlap, overlapSquared, squared, false);
        bestPoint = point;
    }

    internal static bool SupportsBoundaryFan(Signed192 x, Signed192 y, in FixedPointAnchor2d vertex,
        ReadOnlySpan<FixedPointAnchor2d> incidentPoints)
    {
        GetRelativePointNumerators(vertex.Origin, vertex.Rotation, vertex.LocalPoint, vertex.Origin,
            out Signed192 vx, out Signed192 vy);
        foreach (ref readonly FixedPointAnchor2d point in incidentPoints)
        {
            GetRelativePointNumerators(point.Origin, point.Rotation, point.LocalPoint, vertex.Origin,
                out Signed192 px, out Signed192 py);
            if (WideArithmetic.AddProducts(x, WideArithmetic.SubtractSigned192(px, vx),
                y, WideArithmetic.SubtractSigned192(py, vy)).Sign < 0) return false;
        }
        return true;
    }

    /// <summary>Tests exact equality and inward orientation of two independently framed supporting lines.</summary>
    internal static bool SameBoundaryPlane(Vector2d origin, Fixed64 rotation, FixedSegment2d edge, Vector2d inward,
        Vector2d otherOrigin, Fixed64 otherRotation, FixedSegment2d otherEdge, Vector2d otherInward)
    {
        RotationFrame2d frame = new(rotation), otherFrame = new(otherRotation);
        GetTransformedEdge(frame, edge.Start, edge.End, out Signed192 ex, out Signed192 ey);
        GetTransformedEdge(otherFrame, otherEdge.Start, otherEdge.End, out Signed192 ox, out Signed192 oy);
        if ((ex.IsZero && ey.IsZero) || (ox.IsZero && oy.IsZero)
            || !WideArithmetic.MultiplySubtract(ex, oy, ey, ox).IsZero) return false;
        GetRotatedOffset(frame, edge.Start, out Signed192 ax, out Signed192 ay);
        GetRelativePointNumerators(otherOrigin, otherRotation, otherEdge.Start, origin, out Signed192 bx, out Signed192 by);
        if (!WideArithmetic.MultiplySubtract(ex, WideArithmetic.SubtractSigned192(by, ay),
            ey, WideArithmetic.SubtractSigned192(bx, ax)).IsZero) return false;
        GetTransformedEdge(frame, edge.Start, inward, out Signed192 ix, out Signed192 iy);
        GetTransformedEdge(otherFrame, otherEdge.Start, otherInward, out Signed192 jx, out Signed192 jy);
        int first = WideArithmetic.MultiplySubtract(ex, iy, ey, ix).Sign;
        return first != 0 && first == WideArithmetic.MultiplySubtract(ex, jy, ey, jx).Sign;
    }

    /// <summary>Returns -1 outside the retained span, 0 inside, 1 at its lower endpoint, or 2 at its upper endpoint.</summary>
    internal static int LocateBoundaryPoint(in FixedPointAnchor2d point, Vector2d origin, Fixed64 rotation,
        FixedSegment2d edge, in RationalSegmentInterval interval)
    {
        GetRelativePointNumerators(point.Origin, point.Rotation, point.LocalPoint, origin, out Signed192 px, out Signed192 py);
        GetRotatedOffset(new RotationFrame2d(rotation), edge.Start, out Signed192 ax, out Signed192 ay);
        GetTransformedEdge(new RotationFrame2d(rotation), edge.Start, edge.End, out Signed192 ex, out Signed192 ey);
        px = WideArithmetic.SubtractSigned192(px, ax); py = WideArithmetic.SubtractSigned192(py, ay);
        if (!WideArithmetic.MultiplySubtract(px, ey, py, ex).IsZero) return -1;
        Signed320 d = WideArithmetic.AddProducts(ex, ex, ey, ey);
        if (d.IsZero) return px.IsZero && py.IsZero ? 1 : -1;
        Signed320 n = WideArithmetic.AddProducts(px, ex, py, ey);
        if (n.Sign < 0 || WideArithmetic.SubtractSigned320(n, d).Sign > 0) return -1;
        int lower = interval.IsClipped ? RationalSegmentInterval.CompareParameters(Signed832.ExtendValue(n), Signed832.ExtendValue(d),
            interval.WideLowerNumerator, interval.WideLowerDenominator) : n.Sign;
        int upper = interval.IsClipped ? RationalSegmentInterval.CompareParameters(Signed832.ExtendValue(n), Signed832.ExtendValue(d),
            interval.WideUpperNumerator, interval.WideUpperDenominator) : WideArithmetic.SubtractSigned320(n, d).Sign;
        if (lower < 0 || upper > 0) return -1;
        return lower == 0 ? 1 : upper == 0 ? 2 : 0;
    }
}
