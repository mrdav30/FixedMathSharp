using System;
using System.Collections.Generic;
using FixedMathSharp.Geometry;
using Xunit;

namespace FixedMathSharp.Tests;

// Test composition of the production plane/segment cohorts. It performs no
// geometry beyond the actual exact triangle membership and maximum APIs.
internal static class ConePlaneRayTestQueries
{
    internal const int Capacity = ConePlaneRayEvents.IntrinsicCapacity + 3 * ConePlaneRayEvents.BoundaryCapacity;
    internal static bool AccumulateTriangle(FixedTriangle triangle, in ConePlaneRayFrame frame,
        scoped ref ConePlaneRaySelection positive, scoped ref ConePlaneRaySelection negative) =>
        CollectTriangle(triangle, frame, Span<ConePlaneRayTestEvent>.Empty, ref positive, ref negative) != 0;

    internal static int CollectTriangle(FixedTriangle triangle, in ConePlaneRayFrame frame, Span<ConePlaneRayTestEvent> admitted,
        scoped ref ConePlaneRaySelection positive, scoped ref ConePlaneRaySelection negative)
    {
        Span<ConePlaneRayEvent> events = stackalloc ConePlaneRayEvent[ConePlaneRayEvents.IntrinsicCapacity];
        Span<ulong> words = stackalloc ulong[ConePlaneRayPoint.StorageWords], root = stackalloc ulong[ConePlaneRaySelection.RootWords];
        Span<int> signs = stackalloc int[ConePlaneRayPoint.SignCount];
        var point = new ConePlaneRayPoint(words, signs);
        Span<ulong> pv = stackalloc ulong[ConePlaneRaySelection.StorageWords], nv = stackalloc ulong[ConePlaneRaySelection.StorageWords];
        Span<int> ps = stackalloc int[ConePlaneRaySelection.SignCount], ns = stackalloc int[ConePlaneRaySelection.SignCount];
        var p = new ConePlaneRaySelection(pv, ps); var n = new ConePlaneRaySelection(nv, ns);
        scoped ConePlaneRaySelection winnerPositive = new(positive.Values, positive.Signs);
        scoped ConePlaneRaySelection winnerNegative = new(negative.Values, negative.Signs);
        if (positive.HasValue) winnerPositive.Restore(frame, positive.MaximumSource, positive.MaximumEvent);
        if (negative.HasValue) winnerNegative.Restore(frame, negative.MaximumSource, negative.MaximumEvent);
        int count = 0;
        for (int domain = -1; domain < 3; domain++)
        {
            ConePlaneRayEventSource source = domain < 0 ? ConePlaneRayEventSource.Plane : new ConePlaneRayEventSource(triangle.GetEdge(domain));
            int possible = domain < 0 ? ConePlaneRayEvents.GetIntrinsicEvents(frame, events) : ConePlaneRayEvents.GetBoundaryEvents(events);
            foreach (ConePlaneRayEvent item in events[..possible])
            {
                if (!ConePlaneRayEvents.TryEvaluateEvent(source, frame, item, point, root, ref p, ref n)
                    || !ConePlaneRayPointExits.ContainsTrianglePoint(triangle, frame, point, root)) continue;
                if (!admitted.IsEmpty) admitted[count] = new ConePlaneRayTestEvent(source, item);
                count++;
                winnerPositive.KeepEvaluated(p, frame); winnerNegative.KeepEvaluated(n, frame);
            }
        }
        if (winnerPositive.HasValue) positive.Restore(frame, winnerPositive.MaximumSource, winnerPositive.MaximumEvent);
        if (winnerNegative.HasValue) negative.Restore(frame, winnerNegative.MaximumSource, winnerNegative.MaximumEvent);
        return count;
    }

    internal static bool Materialize(ConePlaneRayEventSource source, in ConePlaneRayFrame frame, ConePlaneRayEvent item,
        int orientation, out Vector3d lower, out Vector3d upper, out Fixed64 depth) =>
        Materialize(source, frame, item, orientation, out lower, out upper, out depth, out _);

    internal static bool Materialize(ConePlaneRayEventSource source, in ConePlaneRayFrame frame, ConePlaneRayEvent item,
        int orientation, out Vector3d lower, out Vector3d upper, out Fixed64 depth, out bool admitted)
    {
        Span<ulong> words = stackalloc ulong[ConePlaneRayPoint.StorageWords], root = stackalloc ulong[ConePlaneRaySelection.RootWords];
        Span<int> signs = stackalloc int[ConePlaneRayPoint.SignCount];
        var point = new ConePlaneRayPoint(words, signs);
        Span<ulong> pv = stackalloc ulong[ConePlaneRaySelection.StorageWords], nv = stackalloc ulong[ConePlaneRaySelection.StorageWords];
        Span<int> ps = stackalloc int[ConePlaneRaySelection.SignCount], ns = stackalloc int[ConePlaneRaySelection.SignCount];
        var p = new ConePlaneRaySelection(pv, ps); var n = new ConePlaneRaySelection(nv, ns);
        bool found = ConePlaneRayEvents.TryEvaluateEvent(source, frame, item, point, root, ref p, ref n);
        ConePlaneRaySelection selected = orientation > 0 ? p : n;
        admitted = found && selected.HasValue;
        return selected.TryMaterialize(frame, orientation, out lower, out upper, out depth);
    }
}
