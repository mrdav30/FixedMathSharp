//=======================================================================
// FixedTriangle.FiniteCone.PlaneSection.Scopes.Tests.cs
//=======================================================================
// MIT License, Copyright (c) 2024-present David Oravsky (mrdav30)
// See LICENSE file in the project root for full license information.
//=======================================================================
using System;
using FixedMathSharp.Geometry;
using Xunit;

namespace FixedMathSharp.Tests;

public sealed class FixedTriangleFiniteConePlaneSectionScopesTests
{
    [Fact]
    public void DegeneratePlane_RetainsGeneralInventoryButAdmitsNoPointOrDirectionalCertificate()
    {
        var frame = Frame(new FixedTriangle(Vector3d.Zero, Vector3d.Right, Vector3d.Right * 2));
        Assert.True(frame.Normal.IsZero);
        Span<ConePlaneRayEvent> events = stackalloc ConePlaneRayEvent[ConePlaneRayEvents.IntrinsicCapacity];
        int count = ConePlaneRayEvents.GetIntrinsicEvents(frame, events);
        Assert.Equal(102, count);
        Span<ulong> words = stackalloc ulong[ConePlaneRayPoint.StorageWords], root = stackalloc ulong[ConePlaneRaySelection.RootWords];
        Span<int> signs = stackalloc int[ConePlaneRayPoint.SignCount];
        var point = new ConePlaneRayPoint(words, signs);
        Span<ulong> pv = stackalloc ulong[ConePlaneRaySelection.StorageWords], nv = stackalloc ulong[ConePlaneRaySelection.StorageWords];
        Span<int> ps = stackalloc int[ConePlaneRaySelection.SignCount], ns = stackalloc int[ConePlaneRaySelection.SignCount];
        var positive = new ConePlaneRaySelection(pv, ps); var negative = new ConePlaneRaySelection(nv, ns);
        foreach (ConePlaneRayEvent descriptor in events[..count])
        {
            Assert.False(ConePlaneRayEvents.TryEvaluateEvent(ConePlaneRayEventSource.Plane, frame, descriptor,
                point, root, ref positive, ref negative));
            Assert.False(positive.HasValue); Assert.False(negative.HasValue);
        }
    }

    [Theory]
    [InlineData(-3, 0)]
    [InlineData(-2, 0)]
    [InlineData(0, 0)]
    [InlineData(2, 0)]
    [InlineData(3, 0)]
    [InlineData(-3, 2)]
    [InlineData(-2, 2)]
    [InlineData(0, 2)]
    [InlineData(2, 2)]
    [InlineData(3, 2)]
    public void AxialIntrinsicCohort_OmitsOnlyRejectedDescriptorsAndPreservesBothExits(int height, int radius)
    {
        var plane = new FixedTriangle(new Vector3d(-2, height, -2), new Vector3d(2, height, -2), new Vector3d(0, height, 2));
        var frame = new ConePlaneRayFrame(plane, Vector3d.Zero, FixedQuaternion.Identity,
            Vector3d.Zero, FixedQuaternion.Identity, (Fixed64)4, (Fixed64)radius);
        // This independent plane has X=0 but Z!=0 in its exact normal, so
        // it must retain the full inventory despite the first axial test.
        var general = Frame(new FixedTriangle(new Vector3d(-2, -2, 0), new Vector3d(2, -2, 0), new Vector3d(0, 2, 0)));
        Assert.True(general.Normal.X.IsZero); Assert.False(general.Normal.Z.IsZero);
        Span<ConePlaneRayEvent> complete = stackalloc ConePlaneRayEvent[ConePlaneRayEvents.IntrinsicCapacity];
        Span<ConePlaneRayEvent> compact = stackalloc ConePlaneRayEvent[ConePlaneRayEvents.IntrinsicCapacity];
        int fullCount = ConePlaneRayEvents.GetIntrinsicEvents(general, complete);
        int compactCount = ConePlaneRayEvents.GetIntrinsicEvents(frame, compact);
        Assert.Equal(102, fullCount); Assert.Equal(12, compactCount);
        Span<ulong> words = stackalloc ulong[ConePlaneRayPoint.StorageWords], root = stackalloc ulong[ConePlaneRaySelection.RootWords];
        Span<int> signs = stackalloc int[ConePlaneRayPoint.SignCount];
        var point = new ConePlaneRayPoint(words, signs);
        Span<ulong> pv = stackalloc ulong[ConePlaneRaySelection.StorageWords], nv = stackalloc ulong[ConePlaneRaySelection.StorageWords];
        Span<int> ps = stackalloc int[ConePlaneRaySelection.SignCount], ns = stackalloc int[ConePlaneRaySelection.SignCount];
        var positive = new ConePlaneRaySelection(pv, ps); var negative = new ConePlaneRaySelection(nv, ns);
        int omitted = 0, admittedAxis = 0;
        foreach (ConePlaneRayEvent descriptor in complete[..fullCount])
        {
            bool retained = false;
            foreach (ConePlaneRayEvent candidate in compact[..compactCount])
                retained |= descriptor.Matches(candidate);
            bool admitted = ConePlaneRayEvents.TryEvaluateEvent(ConePlaneRayEventSource.Plane, frame, descriptor,
                point, root, ref positive, ref negative);
            if (!retained)
            {
                omitted++;
                Assert.False(admitted); Assert.False(positive.HasValue); Assert.False(negative.HasValue);
            }
            if (!admitted || descriptor.Kind != ConePlaneRayEventKind.Axis) continue;
            admittedAxis++;
            Assert.True(retained);
            Assert.True(positive.TryMaterialize(frame, 1, out Vector3d p, out Vector3d q, out Fixed64 depth));
            Assert.Equal(new Vector3d(0, height, 0), p); Assert.Equal(Vector3d.Up * 2, q);
            Assert.Equal((Fixed64)(2 - height), depth);
            Assert.True(negative.TryMaterialize(frame, -1, out p, out q, out depth));
            Assert.Equal(new Vector3d(0, height, 0), p); Assert.Equal(Vector3d.Down * 2, q);
            Assert.Equal((Fixed64)(height + 2), depth);
        }
        Assert.Equal(90, omitted);
        Assert.Equal(height >= -2 && height <= 2 ? 1 : 0, admittedAxis);
    }

    [Fact]
    public void IntrinsicEvents_AdmitTheSharedPlaneWithoutClippingToItsSeedTriangle()
    {
        var seed = new FixedTriangle(new Vector3d(2, 0, 0), new Vector3d(3, 0, 0), new Vector3d(2, 0, 1));
        var containing = new FixedTriangle(new Vector3d(-2, 0, -2), new Vector3d(2, 0, -2), new Vector3d(0, 0, 2));
        var frame = Frame(seed);
        Span<ConePlaneRayEvent> events = stackalloc ConePlaneRayEvent[ConePlaneRayEvents.IntrinsicCapacity];
        int count = ConePlaneRayEvents.GetIntrinsicEvents(frame, events);
        Span<ulong> pointWords = stackalloc ulong[ConePlaneRayPoint.StorageWords], root = stackalloc ulong[ConePlaneRayCharts.RootWords];
        Span<int> pointSigns = stackalloc int[ConePlaneRayPoint.SignCount];
        var point = new ConePlaneRayPoint(pointWords, pointSigns);
        Span<ulong> pv = stackalloc ulong[ConePlaneRaySelection.StorageWords], nv = stackalloc ulong[ConePlaneRaySelection.StorageWords];
        Span<int> ps = stackalloc int[ConePlaneRaySelection.SignCount], ns = stackalloc int[ConePlaneRaySelection.SignCount];
        var positive = new ConePlaneRaySelection(pv, ps); var negative = new ConePlaneRaySelection(nv, ns);
        bool foundAxis = false;
        for (int index = 0; index < count; index++)
        {
            ConePlaneRayEvent item = events[index];
            if (item.Kind != ConePlaneRayEventKind.Axis || item.Endpoint != -1) continue;
            Assert.True(ConePlaneRayEvents.TryEvaluateEvent(ConePlaneRayEventSource.Plane, frame, item,
                point, root, ref positive, ref negative));
            Assert.False(ConePlaneRayPointExits.ContainsTrianglePoint(seed, frame, point, root));
            Assert.True(ConePlaneRayPointExits.ContainsTrianglePoint(containing, frame, point, root));
            Assert.True(positive.TryMaterialize(frame, 1, out Vector3d p, out Vector3d q, out Fixed64 depth));
            Assert.Equal(Vector3d.Zero, p); Assert.Equal(Vector3d.Up * 2, q); Assert.Equal(Fixed64.Two, depth);
            Assert.True(negative.TryMaterialize(frame, -1, out p, out q, out depth));
            Assert.Equal(Vector3d.Zero, p); Assert.Equal(Vector3d.Down * 2, q); Assert.Equal(Fixed64.Two, depth);
            foundAxis = true;
        }
        Assert.True(foundAxis);
    }

    [Fact]
    public void BoundaryEvents_RejectSideRootsBeyondBothFiniteEndpoints()
    {
        var seed = new FixedTriangle(new Vector3d(-2, 0, -2), new Vector3d(2, 0, -2), new Vector3d(0, 0, 2));
        var frame = Frame(seed);
        var shortSource = new ConePlaneRayEventSource(new FixedSegment(-Vector3d.Right * Fixed64.Half, Vector3d.Right * Fixed64.Half));
        var longSource = new ConePlaneRayEventSource(new FixedSegment(-Vector3d.Right * 2, Vector3d.Right * 2));
        Span<ConePlaneRayEvent> events = stackalloc ConePlaneRayEvent[ConePlaneRayEvents.BoundaryCapacity];
        int count = ConePlaneRayEvents.GetBoundaryEvents(events);
        Span<ulong> pointWords = stackalloc ulong[ConePlaneRayPoint.StorageWords], root = stackalloc ulong[ConePlaneRayCharts.RootWords];
        Span<int> pointSigns = stackalloc int[ConePlaneRayPoint.SignCount];
        var point = new ConePlaneRayPoint(pointWords, pointSigns);
        Span<ulong> pv = stackalloc ulong[ConePlaneRaySelection.StorageWords], nv = stackalloc ulong[ConePlaneRaySelection.StorageWords];
        Span<int> ps = stackalloc int[ConePlaneRaySelection.SignCount], ns = stackalloc int[ConePlaneRaySelection.SignCount];
        var positive = new ConePlaneRaySelection(pv, ps); var negative = new ConePlaneRaySelection(nv, ns);
        int sideRoots = 0;
        foreach (ConePlaneRayEvent item in events[..count])
        {
            if (item.Kind != ConePlaneRayEventKind.EdgeSide) continue;
            Assert.False(ConePlaneRayEvents.TryEvaluateEvent(shortSource, frame, item, point, root, ref positive, ref negative));
            Assert.False(positive.HasValue); Assert.False(negative.HasValue);
            Assert.True(ConePlaneRayEvents.TryEvaluateEvent(longSource, frame, item, point, root, ref positive, ref negative));
            Assert.True(negative.TryMaterialize(frame, -1, out Vector3d p, out Vector3d q, out Fixed64 depth));
            Assert.Equal(Fixed64.One, FixedMath.Abs(p.X)); Assert.Equal(Fixed64.Zero, p.Y); Assert.Equal(Fixed64.Zero, p.Z);
            Assert.Equal(new Vector3d(p.X, -(Fixed64)2, Fixed64.Zero), q); Assert.Equal(Fixed64.Two, depth);
            sideRoots++;
        }
        Assert.Equal(2, sideRoots);
    }

    [Fact]
    public void EvaluatedSelection_RetainsRationalPointUnderIrrationalDepthRootAndRestoresBorrowedStorage()
    {
        var seed = new FixedTriangle(new Vector3d(0, -2, -2), new Vector3d(0, 2, -2), new Vector3d(0, 0, 2));
        var frame = Frame(seed);
        var source = new ConePlaneRayEventSource(new FixedSegment(new Vector3d(Fixed64.Zero, Fixed64.Zero, Fixed64.Half),
            new Vector3d(Fixed64.Zero, Fixed64.One, Fixed64.Half)));
        var item = new ConePlaneRayEvent(ConePlaneRayEventKind.EdgeEndpoint, endpoint: 0);
        Span<ulong> pointWords = stackalloc ulong[ConePlaneRayPoint.StorageWords], root = stackalloc ulong[ConePlaneRayCharts.RootWords];
        Span<int> pointSigns = stackalloc int[ConePlaneRayPoint.SignCount];
        var point = new ConePlaneRayPoint(pointWords, pointSigns);
        Span<ulong> pv = stackalloc ulong[ConePlaneRaySelection.StorageWords], nv = stackalloc ulong[ConePlaneRaySelection.StorageWords];
        Span<int> ps = stackalloc int[ConePlaneRaySelection.SignCount], ns = stackalloc int[ConePlaneRaySelection.SignCount];
        var positive = new ConePlaneRaySelection(pv, ps); var negative = new ConePlaneRaySelection(nv, ns);
        Assert.True(ConePlaneRayEvents.TryEvaluateEvent(source, frame, item, point, root, ref positive, ref negative));
        Assert.Equal(0, WideArithmetic.GetActiveMagnitudeLength(root));
        Assert.True(WideArithmetic.GetActiveMagnitudeLength(positive.Root) > 0);
        Span<ulong> retainedWords = stackalloc ulong[ConePlaneRaySelection.StorageWords];
        Span<int> retainedSigns = stackalloc int[ConePlaneRaySelection.SignCount];
        var winner = new ConePlaneRaySelection(retainedWords, retainedSigns);
        Assert.True(winner.KeepEvaluated(positive, frame));
        Assert.False(winner.KeepEvaluated(positive, frame));
        var restored = new ConePlaneRaySelection(retainedWords, retainedSigns);
        restored.Restore(frame, winner.MaximumSource, winner.MaximumEvent);
        Assert.True(restored.TryMaterialize(frame, 1, out Vector3d p, out Vector3d q, out Fixed64 depth));
        Assert.Equal(new Vector3d(Fixed64.Zero, Fixed64.Zero, Fixed64.Half), p);
        Assert.Equal(new Vector3d(Fixed64.FromRaw(3719550787L), Fixed64.Zero, Fixed64.Half), q);
        Assert.Equal(Fixed64.FromRaw(3719550787L), depth);
    }

    [Theory]
    [InlineData(0, false)]
    [InlineData(1, false)]
    [InlineData(2, false)]
    [InlineData(3, true)]
    [InlineData(4, true)]
    [InlineData(5, true)]
    [InlineData(6, true)]
    [InlineData(7, false)]
    [InlineData(8, false)]
    public void SegmentDomain_RequiresTheExactFiniteLineOrPoint(int example, bool expected)
    {
        var seed = new FixedTriangle(new Vector3d(-2, 0, -2), new Vector3d(2, 0, -2), new Vector3d(0, 0, 2));
        var frame = Frame(seed);
        Vector3d quarter = Vector3d.Right * Fixed64.Quarter;
        FixedSegment segment = example >= 6 ? new FixedSegment(quarter, quarter)
            : new FixedSegment(-Vector3d.Right * Fixed64.Half, Vector3d.Right * Fixed64.Half);
        Vector3d query = example switch
        {
            0 => -Vector3d.Right * Fixed64.FromFraction(3, 4),
            1 => Vector3d.Right * Fixed64.FromFraction(3, 4),
            2 => Vector3d.Forward * Fixed64.Quarter,
            3 => segment.Start,
            4 => segment.End,
            6 => quarter,
            8 => quarter + Vector3d.Forward * Fixed64.Quarter,
            _ => Vector3d.Zero
        };
        Span<ulong> words = stackalloc ulong[ConePlaneRayPoint.StorageWords];
        Span<int> signs = stackalloc int[ConePlaneRayPoint.SignCount];
        var point = new ConePlaneRayPoint(words, signs); point.Set(frame.Transform(query));
        Assert.Equal(expected, ConePlaneRayPointExits.ContainsPoint(new ConePlaneRayEventSource(segment), frame, point, ReadOnlySpan<ulong>.Empty));
    }

    [Theory]
    [InlineData(0)]
    [InlineData(1)]
    [InlineData(2)]
    [InlineData(3)]
    public void ScopedSource_RejectsDescriptorsOutsideItsCohort(int example)
    {
        var frame = Frame(new FixedTriangle(new Vector3d(-2, 0, -2), new Vector3d(2, 0, -2), new Vector3d(0, 0, 2)));
        var source = example == 3 ? ConePlaneRayEventSource.Plane
            : new ConePlaneRayEventSource(new FixedSegment(Vector3d.Left, Vector3d.Right));
        ConePlaneRayEvent item = example switch
        {
            0 => new ConePlaneRayEvent(ConePlaneRayEventKind.LowerCircle, 8, 0, 0, 0),
            1 => new ConePlaneRayEvent(ConePlaneRayEventKind.UpperRim, 8, 0, 0, 0),
            2 => new ConePlaneRayEvent(ConePlaneRayEventKind.EdgeSide, feature: 1, branch: 1),
            _ => new ConePlaneRayEvent(ConePlaneRayEventKind.Apex, endpoint: 0)
        };
        if (example == 3)
            Assert.True(ConePlaneRayTestQueries.Materialize(source, frame, new ConePlaneRayEvent(ConePlaneRayEventKind.Apex), 1,
                out _, out _, out _));
        Assert.False(ConePlaneRayTestQueries.Materialize(source, frame, item, 1, out _, out _, out _));
    }

    [Fact]
    public void BoundaryRimAtZeroDepth_StillRequiresTheFiniteSegment()
    {
        var frame = Frame(new FixedTriangle(new Vector3d(-3, -2, -3), new Vector3d(3, -2, -3), new Vector3d(0, -2, 3)));
        var complete = new ConePlaneRayEventSource(new FixedSegment(new Vector3d(-2, -2, 1), new Vector3d(2, -2, 1)));
        var shortSegment = new ConePlaneRayEventSource(new FixedSegment(new Vector3d(-Fixed64.Half, -Fixed64.Two, Fixed64.One),
            new Vector3d(Fixed64.Half, -Fixed64.Two, Fixed64.One)));
        Span<ConePlaneRayEvent> events = stackalloc ConePlaneRayEvent[ConePlaneRayEvents.BoundaryCapacity];
        int count = ConePlaneRayEvents.GetBoundaryEvents(events), admitted = 0;
        foreach (ConePlaneRayEvent item in events[..count])
        {
            if (item.Kind != ConePlaneRayEventKind.UpperRim) continue;
            if (ConePlaneRayTestQueries.Materialize(complete, frame, item, 1, out Vector3d p, out Vector3d q, out Fixed64 depth))
            {
                admitted++;
                Assert.Equal(p, q); Assert.Equal(Fixed64.Zero, depth);
                Assert.Equal(-Fixed64.Two, p.Y); Assert.Equal(Fixed64.One, p.Z);
                Assert.Equal(7439101574L, FixedMath.Abs(p.X).m_rawValue);
            }
            Assert.False(ConePlaneRayTestQueries.Materialize(shortSegment, frame, item, 1, out _, out _, out _));
        }
        Assert.Equal(2, admitted);
    }

    [Fact]
    public void StationaryOppositeExits_CompareTheirExactSharedPointWithoutExitReplay()
    {
        var frame = Frame(new FixedTriangle(new Vector3d(0, -3, -3), new Vector3d(0, -3, 3), new Vector3d(0, 3, 0)));
        var source = new ConePlaneRayEventSource(new FixedSegment(-Vector3d.Forward * Fixed64.Half, Vector3d.Forward * Fixed64.Half));
        var first = new ConePlaneRayEvent(ConePlaneRayEventKind.EdgeStationary, branch: -1);
        var second = new ConePlaneRayEvent(ConePlaneRayEventKind.EdgeStationary, branch: 1);
        Assert.Equal(0, ConePlaneRayEvents.CompareEventAnchors(source, first, source, second, frame, includeCoincidentProvenance: false));
        Assert.True(ConePlaneRayEvents.CompareEventAnchors(source, first, source, second, frame) < 0);
        Assert.True(ConePlaneRayTestQueries.Materialize(source, frame, first, -1, out Vector3d p, out Vector3d q, out Fixed64 depth));
        Assert.Equal(Vector3d.Zero, p); Assert.Equal(Vector3d.Left, q); Assert.Equal(Fixed64.One, depth);
        Assert.True(ConePlaneRayTestQueries.Materialize(source, frame, second, 1, out p, out q, out depth));
        Assert.Equal(Vector3d.Zero, p); Assert.Equal(Vector3d.Right, q); Assert.Equal(Fixed64.One, depth);
    }

    private static ConePlaneRayFrame Frame(FixedTriangle triangle) => new(triangle, Vector3d.Zero, FixedQuaternion.Identity,
        Vector3d.Zero, FixedQuaternion.Identity, (Fixed64)4, Fixed64.Two);
}

internal readonly struct ConePlaneRayTestEvent
{
    internal readonly ConePlaneRayEventSource Source;
    internal readonly ConePlaneRayEvent Event;
    internal ConePlaneRayTestEvent(ConePlaneRayEventSource source, ConePlaneRayEvent item) { Source = source; Event = item; }
}

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
