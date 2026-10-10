//=======================================================================
// ConePlaneRayEvents.cs
//=======================================================================
// MIT License, Copyright (c) 2024-present David Oravsky (mrdav30)
// See LICENSE file in the project root for full license information.
//=======================================================================
using System;

namespace FixedMathSharp.Geometry;

/// <summary>Exact critical events for a finite cone with a shared plane, finite segment.</summary>
/// <content>Scoped event dispatch and authored-edge endpoint, side and stationary candidates.</content>
internal static partial class ConePlaneRayEvents
{
    private const int Words = ConePlaneRayCharts.Words;
    private const int RootWords = ConePlaneRayCharts.RootWords;
    private static ContactQuadratic At(Span<ulong> values, Span<int> signs, int index) =>
        ContactQuadratic.At(values, signs, index, Words);
    private static Signed576 Extend(Signed320 value) => Signed576.ExtendValue(value);
    private static Signed576 Integer(long value) => Extend(Signed320.ExtendValue(Signed192.Signed(value)));

    // The fixed-width frame product is sufficient for two authored points,
    // but not for the larger canonical normal. Keep normal products in the
    // existing quadratic owner instead of narrowing their exact coefficients.
    private static void Product(WideAxis3 left, WideAxis3 right,
        in ConePlaneRayFrame frame, ContactQuadratic result)
    {
        Span<ulong> values = stackalloc ulong[8 * Words];
        Span<int> signs = stackalloc int[8];
        ContactQuadratic x = At(values, signs, 0), y = At(values, signs, 1);
        ContactQuadratic term = At(values, signs, 2), scaled = At(values, signs, 3);
        result.Clear();
        for (int axis = 0; axis < 3; axis++)
        {
            x.Set(Extend(axis == 0 ? left.X : axis == 1 ? left.Y : left.Z));
            y.Set(Extend(axis == 0 ? right.X : axis == 1 ? right.Y : right.Z));
            ContactQuadratic.Multiply(x, y, ReadOnlySpan<ulong>.Empty, term);
            Signed192 raw = Signed192.Raw(axis == 1 ? frame.Radius : frame.Height);
            ContactQuadratic.Scale(term, Extend(WideArithmetic.MultiplySigned192(raw, raw)), scaled);
            result.Add(scaled, axis == 1 ? -1 : 1);
        }
    }

    private static void SetAffinePoint(ConePlaneRayPoint point, WideAxis3 start, WideAxis3 delta,
        ContactQuadratic numerator, ContactQuadratic denominator)
    {
        Span<ulong> values = stackalloc ulong[2 * Words];
        Span<int> signs = stackalloc int[2];
        ContactQuadratic term = At(values, signs, 0);
        for (int axis = 0; axis < 3; axis++)
        {
            ContactQuadratic target = axis == 0 ? point.X : axis == 1 ? point.Y : point.Z;
            ContactQuadratic.Scale(denominator, Extend(axis == 0 ? start.X : axis == 1 ? start.Y : start.Z), target);
            ContactQuadratic.Scale(numerator, Extend(axis == 0 ? delta.X : axis == 1 ? delta.Y : delta.Z), term);
            target.Add(term);
        }
        denominator.CopyTo(point.Denominator);
    }

    private static bool Accumulate(in ConePlaneRayEventSource source, in ConePlaneRayFrame frame,
        scoped ref ConePlaneRaySelection positive, scoped ref ConePlaneRaySelection negative,
        scoped ref ConePlaneRayEventSink sink)
    {
        sink.HasPoint = false;
        if (frame.NormalSquared.IsZero)
            return false;
        bool found = false;
        if (source.Scope == ConePlaneRayEventScope.Segment && sink.Target.Feature == 0
            && (sink.Wants(ConePlaneRayEventKind.EdgeEndpoint) || sink.Wants(ConePlaneRayEventKind.EdgeSide)
                || sink.Wants(ConePlaneRayEventKind.EdgeStationary)))
            found |= AccumulateEdge(source, frame, ref positive, ref negative, ref sink);
        if (!sink.IsStreaming && source.Scope != ConePlaneRayEventScope.Segment
            && (sink.Wants(ConePlaneRayEventKind.Axis) || sink.Wants(ConePlaneRayEventKind.Apex)))
            found |= AccumulateAxisAndApex(source, frame, ref positive, ref negative, ref sink);
        if (sink.Wants(ConePlaneRayEventKind.LowerCircle) || sink.Wants(ConePlaneRayEventKind.UpperRim)
            || sink.Wants(ConePlaneRayEventKind.Generator))
            found |= AccumulateCircles(source, frame, ref positive, ref negative, ref sink);
        if (sink.IsStreaming && source.Scope != ConePlaneRayEventScope.Segment)
            found |= AccumulateAxisAndApex(source, frame, ref positive, ref negative, ref sink);
        // Radial base-stationary points/exits already belong to upper line 7
        // or its cardinal seams, with earlier canonical provenance.
        if (source.Scope != ConePlaneRayEventScope.Segment && sink.Wants(ConePlaneRayEventKind.BaseStationary)
            && (!sink.IsStreaming || !frame.Normal.Y.IsZero))
            found |= AccumulateBaseStationary(source, frame, ref positive, ref negative, ref sink);
        return found;
    }

    private static bool AccumulateEdge(in ConePlaneRayEventSource source, in ConePlaneRayFrame frame,
        scoped ref ConePlaneRaySelection positive, scoped ref ConePlaneRaySelection negative,
        scoped ref ConePlaneRayEventSink sink)
    {
        FixedSegment segment = new(source.A, source.B);
        WideAxis3 p = frame.Transform(segment.Start), end = frame.Transform(segment.End);
        WideAxis3 e = new(WideArithmetic.SubtractSigned320(end.X, p.X),
            WideArithmetic.SubtractSigned320(end.Y, p.Y), WideArithmetic.SubtractSigned320(end.Z, p.Z));
        if (!frame.Finite.TryGetAxialInterval(p.Y, e.Y, out Signed320 ln, out Signed320 ld,
            out Signed320 un, out Signed320 ud))
            return false;
        Span<ulong> values = stackalloc ulong[30 * Words];
        Span<int> signs = stackalloc int[30];
        Span<ulong> root = stackalloc ulong[RootWords];
        Span<ulong> pointValues = stackalloc ulong[ConePlaneRayPoint.StorageWords];
        Span<int> pointSigns = stackalloc int[8];
        var point = new ConePlaneRayPoint(pointValues, pointSigns);
        ContactQuadratic a = At(values, signs, 0), b = At(values, signs, 1), c = At(values, signs, 2);
        ContactQuadratic m = At(values, signs, 3), n = At(values, signs, 4), A = At(values, signs, 5);
        ContactQuadratic qa = At(values, signs, 6), qb = At(values, signs, 7), qc = At(values, signs, 8);
        ContactQuadratic t = At(values, signs, 9), d = At(values, signs, 10), s = At(values, signs, 11);
        ContactQuadratic sd = At(values, signs, 12), work = At(values, signs, 13), term = At(values, signs, 14);
        bool found = false;
        for (int endpoint = 0; endpoint < 2; endpoint++)
        {
            var descriptor = new ConePlaneRayEvent(ConePlaneRayEventKind.EdgeEndpoint, endpoint: endpoint);
            if (!sink.Wants(descriptor)) continue;
            s.Set(Extend(endpoint == 0 ? ln : un)); sd.Set(Extend(endpoint == 0 ? ld : ud));
            SetAffinePoint(point, p, e, s, sd);
            SetEvent(source, descriptor, ref positive, ref negative);
            bool admitted = ConePlaneRayPointExits.AccumulateRationalPoint(source, frame, point, ref positive, ref negative, sink.PointOnly, sink.RequestedOrientation);
            if (admitted) sink.Keep(descriptor, point, ReadOnlySpan<ulong>.Empty, positive, negative);
            found |= admitted;
        }
        if (!sink.Wants(ConePlaneRayEventKind.EdgeSide) && !sink.Wants(ConePlaneRayEventKind.EdgeStationary))
            return found;
        Product(e, e, frame, a); Product(p, e, frame, b); Product(p, p, frame, c);
        // Construct the stationary polynomial once, then emit side/stationary
        // events in each branch's inventory order. Targeted side replay avoids
        // these products entirely; streaming does not retain extra wide fields.
        bool stationary = a.Signs[0] != 0 && sink.Wants(ConePlaneRayEventKind.EdgeStationary);
        if (stationary)
        {
            Product(e, frame.Normal, frame, m); Product(p, frame.Normal, frame, n);
            Product(frame.Normal, frame.Normal, frame, A);
            ContactQuadratic.Multiply(a, A, ReadOnlySpan<ulong>.Empty, qa);
            ContactQuadratic.Multiply(m, m, ReadOnlySpan<ulong>.Empty, term); qa.Add(term, -1);
            ContactQuadratic.Multiply(a, n, ReadOnlySpan<ulong>.Empty, qb);
            ContactQuadratic.Multiply(b, m, ReadOnlySpan<ulong>.Empty, term); qb.Add(term, -1);
            ContactQuadratic.Multiply(a, c, ReadOnlySpan<ulong>.Empty, qc);
            ContactQuadratic.Multiply(b, b, ReadOnlySpan<ulong>.Empty, term); qc.Add(term, -1);
        }
        // a=0 has no isolated interior stationary point, or has a constant
        // interval already represented by its clipped endpoints and side/rim.
        for (int branch = -1; branch <= 1; branch += 2)
        {
            var descriptor = new ConePlaneRayEvent(ConePlaneRayEventKind.EdgeSide, branch: branch);
            if (sink.Wants(descriptor)
                && ConePlaneRayCharts.TryGetParameter(a, b, c, branch, root, s, sd)
                && ConePlaneRayCharts.IsUnitParameter(s, sd, root))
            {
                SetAffinePoint(point, p, e, s, sd);
                SetEvent(source, descriptor, ref positive, ref negative);
                bool admitted = ConePlaneRayPointExits.AccumulateSidePoint(source, frame, point, root, ref positive, ref negative, sink.PointOnly, sink.RequestedOrientation);
                if (admitted) sink.Keep(descriptor, point, root, positive, negative);
                found |= admitted;
            }
            descriptor = new ConePlaneRayEvent(ConePlaneRayEventKind.EdgeStationary, branch: branch);
            if (!stationary || !sink.Wants(descriptor)
                || !ConePlaneRayCharts.TryGetParameter(qa, qb, qc, branch, root, t, d))
                continue;
            ContactQuadratic.Multiply(b, d, root, s);
            ContactQuadratic.Multiply(m, t, root, term); s.Add(term); s.MultiplySign(-a.Signs[0]);
            ContactQuadratic.Multiply(a, d, root, sd); sd.MultiplySign(a.Signs[0]);
            if (!ConePlaneRayCharts.IsUnitParameter(s, sd, root))
                continue;
            SetAffinePoint(point, p, e, s, sd);
            int orientation = t.Sign(root) < 0 ? -1 : 1;
            descriptor = new ConePlaneRayEvent(descriptor.Kind, branch: branch, orientation: orientation);
            t.CopyTo(work, orientation);
            SetEvent(source, descriptor, ref positive, ref negative);
            bool stationaryAdmitted = ConePlaneRayPointExits.AccumulateStationarySideExit(source, frame, point, root,
                work, d, orientation, ref positive, ref negative, sink.PointOnly);
            if (stationaryAdmitted) sink.Keep(descriptor, point, root, positive, negative);
            found |= stationaryAdmitted;
        }
        return found;
    }
}
