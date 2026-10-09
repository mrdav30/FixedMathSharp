//=======================================================================
// ConePlaneRayEvents.Scopes.cs
//=======================================================================
// MIT License, Copyright (c) 2024-present David Oravsky (mrdav30)
// See LICENSE file in the project root for full license information.
//=======================================================================
using System;

namespace FixedMathSharp.Geometry;

/// <content>Shared-plane and finite-boundary construction cohorts, evaluated independently of mesh regions.</content>
internal static partial class ConePlaneRayEvents
{
    // Possible constructions, not admitted roots: circles 72, generator
    // endpoints 24, axis 3, apex 1, base stationary 2. A boundary needs its
    // six edge events and eight projected upper-rim wall chart branches.
    internal const int IntrinsicCapacity = 102;
    internal const int BoundaryCapacity = 14;

    internal static int GetIntrinsicEvents(in ConePlaneRayFrame frame, Span<ConePlaneRayEvent> events)
    {
        if (events.Length < IntrinsicCapacity) throw new ArgumentException("Intrinsic event storage is too small.", nameof(events));
        // For a nonzero axial normal every non-seam circle restriction is
        // C*(1+u*u): no real root, or a zero polynomial already rejected by
        // the chart owner. N.g=Ny*FullHeight*(d*d+n*n) cannot vanish because
        // height is positive, so no generator event survives. Base stationarity
        // also rejects the zero radial normal. Omitting those 90 constructions
        // preserves every admitted descriptor and both directional certificates,
        // including radius zero; the eight seams, three axes and apex remain.
        bool axial = frame.Normal.X.IsZero && frame.Normal.Z.IsZero && !frame.Normal.Y.IsZero;
        int count = 0;
        for (int upper = 0; upper < 2; upper++)
        for (int quadrant = 0; quadrant < 4; quadrant++)
        {
            ConePlaneRayEventKind kind = upper == 0 ? ConePlaneRayEventKind.LowerCircle : ConePlaneRayEventKind.UpperRim;
            for (int endpoint = 0; endpoint < 2; endpoint++)
            {
                if (endpoint == 0 && (quadrant & 2) != 0 || endpoint == 1 && (quadrant & 1) != 0) continue;
                events[count++] = new ConePlaneRayEvent(kind, 8, quadrant, endpoint, endpoint);
                if (upper == 0 && !axial)
                    for (int bound = 0; bound < 2; bound++)
                        events[count++] = new ConePlaneRayEvent(ConePlaneRayEventKind.Generator, 8, quadrant, endpoint, bound);
            }
            if (axial) continue;
            for (int line = 0; line < (upper == 0 ? 6 : 8); line++)
            {
                if (upper == 0 ? line >= 2 && line <= 4 : line < 3) continue;
                for (int branch = -1; branch <= 1; branch += 2)
                {
                    events[count++] = new ConePlaneRayEvent(kind, line, quadrant, branch);
                    if (upper == 0 && line == 0)
                        for (int bound = 0; bound < 2; bound++)
                            events[count++] = new ConePlaneRayEvent(ConePlaneRayEventKind.Generator, line, quadrant, branch, bound);
                }
            }
        }
        for (int endpoint = -1; endpoint < 2; endpoint++)
            events[count++] = new ConePlaneRayEvent(ConePlaneRayEventKind.Axis, endpoint: endpoint);
        events[count++] = new ConePlaneRayEvent(ConePlaneRayEventKind.Apex);
        if (!axial)
            for (int branch = -1; branch <= 1; branch += 2)
                events[count++] = new ConePlaneRayEvent(ConePlaneRayEventKind.BaseStationary, branch: branch);
        return count;
    }

    internal static int GetBoundaryEvents(Span<ConePlaneRayEvent> events)
    {
        if (events.Length < BoundaryCapacity) throw new ArgumentException("Boundary event storage is too small.", nameof(events));
        int count = 0;
        for (int endpoint = 0; endpoint < 2; endpoint++)
            events[count++] = new ConePlaneRayEvent(ConePlaneRayEventKind.EdgeEndpoint, endpoint: endpoint);
        for (int branch = -1; branch <= 1; branch += 2)
        {
            events[count++] = new ConePlaneRayEvent(ConePlaneRayEventKind.EdgeSide, branch: branch);
            events[count++] = new ConePlaneRayEvent(ConePlaneRayEventKind.EdgeStationary, branch: branch);
        }
        for (int quadrant = 0; quadrant < 4; quadrant++)
        for (int branch = -1; branch <= 1; branch += 2)
            events[count++] = new ConePlaneRayEvent(ConePlaneRayEventKind.UpperRim, quadrant: quadrant, branch: branch);
        // Lower-circle wall, generator-wall and finite-axis intersections are
        // already the segment's side roots, or endpoints when F is identically
        // zero on that segment. Only the upper-rim projection adds the switch
        // between its side and cap exits; no artificial triangle is required.
        return count;
    }

    internal static bool TryEvaluateEvent(in ConePlaneRayEventSource source, in ConePlaneRayFrame frame,
        ConePlaneRayEvent descriptor, scoped ConePlaneRayPoint point, scoped Span<ulong> root,
        scoped ref ConePlaneRaySelection positive, scoped ref ConePlaneRaySelection negative)
    {
        positive.HasValue = negative.HasValue = false;
        root.Clear();
        // Reborrow coefficient storage locally: the sink borrows this call's
        // output point/root, and no such reference may enter caller metadata.
        scoped ConePlaneRaySelection evaluatedPositive = new(positive.Values, positive.Signs);
        scoped ConePlaneRaySelection evaluatedNegative = new(negative.Values, negative.Signs);
        var sink = new ConePlaneRayEventSink(descriptor, point, root);
        Accumulate(source, frame, ref evaluatedPositive, ref evaluatedNegative, ref sink);
        if (!sink.HasPoint) return false;
        if (evaluatedPositive.HasValue)
            positive.Restore(frame, evaluatedPositive.MaximumSource, evaluatedPositive.MaximumEvent);
        if (evaluatedNegative.HasValue)
            negative.Restore(frame, evaluatedNegative.MaximumSource, evaluatedNegative.MaximumEvent);
        return true;
    }
}
