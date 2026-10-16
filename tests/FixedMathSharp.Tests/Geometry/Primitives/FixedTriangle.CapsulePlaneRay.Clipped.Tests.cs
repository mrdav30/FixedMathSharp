using System;
using FixedMathSharp.Geometry;
using Xunit;

namespace FixedMathSharp.Tests.Bounds;

public sealed class FixedTriangleCapsulePlaneRayClippedTests
{
    private static Signed320 Integer(long value) => Signed320.ExtendValue(Signed192.Signed(value));

    [Theory]
    [InlineData(false, -1)]
    [InlineData(false, 1)]
    [InlineData(true, -1)]
    [InlineData(true, 1)]
    public void RationalClips_ShouldEmitTheirOwnEndpointsAndExcludeOutsideExtrema(bool capsule, int orientation)
    {
        var plane = new FixedTriangle(Vector3d.Zero, new Vector3d(2, 0, 0), new Vector3d(0, 0, 2));
        var finite = new CapsuleFiniteSectionFrame(Vector3d.Zero, FixedQuaternion.Identity,
            new Vector3d(Fixed64.Zero, Fixed64.Quarter, Fixed64.Zero), FixedQuaternion.Identity,
            capsule ? Fixed64.One : Fixed64.Zero, Fixed64.One);
        var frame = new CapsulePlaneRayFrame(plane, finite);
        var source = new CapsulePlaneRayEventSource(new FixedSegment(new(-2, 0, 0), new(2, 0, 0)));
        // Original segment endpoints lie outside the shape. The 3/5 endpoint
        // is inside and its authored X=2/5 cannot be represented exactly.
        var interval = new RationalSegmentInterval(Integer(3), Integer(5), Integer(4), Integer(5));
        int endpoints = 0, count = 0;
        Assert.True(CapsulePlaneRayEvents.VisitEvents(source, frame, orientation, interval,
            (in CapsulePlaneRayEventSource currentSource, in CapsulePlaneRayFrame current,
                CapsulePlaneRayEvent descriptor, int direction, scoped in CapsulePlaneRaySelection candidate) =>
            {
                Assert.True(candidate.TryGetAuthoredPoint(current, Vector3d.Zero, FixedQuaternion.Identity,
                    new Vector3d(Fixed64.Zero, Fixed64.Quarter, Fixed64.Zero), FixedQuaternion.Identity, out Vector3d point));
                Assert.InRange(point.X.m_rawValue, Fixed64.FromFraction(2, 5).m_rawValue, Fixed64.FromFraction(6, 5).m_rawValue);
                Span<ulong> values = stackalloc ulong[CapsulePlaneRaySelection.StorageWords];
                Span<int> signs = stackalloc int[CapsulePlaneRaySelection.SignCount];
                Span<sbyte> valueSigns = stackalloc sbyte[CapsulePlaneRaySelection.ValueSignCount];
                var replay = new CapsulePlaneRaySelection(values, signs, valueSigns);
                Assert.True(CapsulePlaneRayEvents.Reconstruct(currentSource, current, descriptor, direction, interval, ref replay));
                Assert.Equal(0, replay.ComparePoint(candidate, current));
                Assert.Equal(0, replay.CompareDepth(candidate));
                if (descriptor.Kind == CapsulePlaneRayEventKind.Endpoint)
                {
                    Assert.Equal(0, descriptor.Feature);
                    Assert.Equal(Fixed64.FromFraction(2, 5), point.X);
                    endpoints++;
                }
                count++;
            }));
        Assert.True(endpoints > 0);
        Assert.True(count > endpoints);
    }

    [Theory]
    [InlineData(0, 0, 1, 1)]
    [InlineData(0, 1, 1, 0)]
    [InlineData(-1, 1, 1, 1)]
    [InlineData(0, 1, -1, 1)]
    [InlineData(2, 1, 1, 1)]
    [InlineData(0, 1, 2, 1)]
    [InlineData(2, 3, 1, 3)]
    public void InvalidIntervals_ShouldBeRejectedBeforeEventGeneration(long ln, long ld, long un, long ud) =>
        Assert.ThrowsAny<ArgumentException>(() => new RationalSegmentInterval(Integer(ln), Integer(ld), Integer(un), Integer(ud)));

    [Fact]
    public void LargeOddClipDenominators_ShouldPreserveDistinctSubRawEndpointsAndDepths()
    {
        Signed320 d = new(1UL << 62, 0, 0, 0, 1); // 2^318+1.
        var interval = new RationalSegmentInterval(new(1UL << 61, 0, 0, 0, 1), d,
            new(1UL << 61, 0, 0, 0, 2), d);
        CheckEndpointPair(interval, expectedPointOrder: -1, expectedDepthOrder: 1);
    }

    [Fact]
    public void LargeOddClips_WithWideObliqueFrames_ShouldReconstructWithoutNarrowing()
    {
        Fixed64 Raw(long value) => Fixed64.FromRaw(value);
        var axis = new Vector3d(Raw(long.MaxValue - 1), Raw(long.MaxValue - 3), Raw(1));
        var plane = new FixedTriangle(Vector3d.Zero, axis,
            new Vector3d(Raw(-1), Raw(long.MaxValue - 5), Raw(long.MaxValue - 7)));
        FixedQuaternion member = FixedQuaternion.FromEulerAnglesInDegrees((Fixed64)17, (Fixed64)37, (Fixed64)11);
        FixedQuaternion shape = FixedQuaternion.FromEulerAnglesInDegrees((Fixed64)23, (Fixed64)31, (Fixed64)43);
        var frame = new CapsulePlaneRayFrame(plane, new CapsuleFiniteSectionFrame(Vector3d.Zero, member,
            Vector3d.Zero, shape, Fixed64.One, Fixed64.One));
        var source = new CapsulePlaneRayEventSource(new FixedSegment(-axis, axis));
        Signed320 d = new(1UL << 62, 0, 0, 0, 1);
        var interval = new RationalSegmentInterval(new(1UL << 61, 0, 0, 0, 1), d,
            new(1UL << 61, 0, 0, 0, 2), d);
        int endpoints = 0;
        foreach (int orientation in new[] { -1, 1 })
            Assert.True(CapsulePlaneRayEvents.VisitEvents(source, frame, orientation, interval,
                (in CapsulePlaneRayEventSource s, in CapsulePlaneRayFrame f, CapsulePlaneRayEvent descriptor, int o,
                    scoped in CapsulePlaneRaySelection candidate) =>
                {
                    Assert.Equal(CapsulePlaneRayEventKind.Endpoint, descriptor.Kind);
                    Assert.True(candidate.TryGetAuthoredPoint(f, Vector3d.Zero, member, Vector3d.Zero, shape, out Vector3d p));
                    Assert.Equal(Vector3d.Zero, p);
                    Span<ulong> values = stackalloc ulong[CapsulePlaneRaySelection.StorageWords];
                    Span<int> signs = stackalloc int[CapsulePlaneRaySelection.SignCount];
                    Span<sbyte> valueSigns = stackalloc sbyte[CapsulePlaneRaySelection.ValueSignCount];
                    var replay = new CapsulePlaneRaySelection(values, signs, valueSigns);
                    Assert.True(CapsulePlaneRayEvents.Reconstruct(s, f, descriptor, o, interval, ref replay));
                    Assert.Equal(0, replay.ComparePoint(candidate, f));
                    Assert.Equal(0, replay.CompareDepth(candidate));
                    endpoints++;
                }));
        Assert.Equal(4, endpoints);
    }

    [Fact]
    public void SingletonIntervals_ShouldRetainTheirExactClosedPoint()
    {
        var interval = new RationalSegmentInterval(Integer(1), Integer(3), Integer(1), Integer(3));
        CheckEndpointPair(interval, expectedPointOrder: 0, expectedDepthOrder: 0);
    }

    [Theory]
    [InlineData(false, -1)]
    [InlineData(false, 1)]
    [InlineData(true, -1)]
    [InlineData(true, 1)]
    public void ClippedReplay_ShouldRejectOutsideExtremaAndIncludeExactStationaryBounds(bool capsule, int orientation)
    {
        var plane = new FixedTriangle(Vector3d.Zero, new Vector3d(2, 0, 0), new Vector3d(0, 0, 2));
        var frame = new CapsulePlaneRayFrame(plane, new CapsuleFiniteSectionFrame(Vector3d.Zero, FixedQuaternion.Identity,
            new Vector3d(Fixed64.Zero, Fixed64.Quarter, Fixed64.Zero), FixedQuaternion.Identity,
            capsule ? Fixed64.One : Fixed64.Zero, Fixed64.One));
        var source = new CapsulePlaneRayEventSource(new FixedSegment(new(-2, 0, 0), new(2, 0, 0)));
        var left = new RationalSegmentInterval(Integer(0), Integer(1), Integer(1), Integer(5));
        var right = new RationalSegmentInterval(Integer(4), Integer(5), Integer(1), Integer(1));
        var fromMidpoint = new RationalSegmentInterval(Integer(1), Integer(2), Integer(3), Integer(4));
        var throughMidpoint = new RationalSegmentInterval(Integer(1), Integer(4), Integer(1), Integer(2));
        Signed320 odd = new(1UL << 62, 0, 0, 0, 1);
        var afterMidpoint = new RationalSegmentInterval(new(1UL << 61, 0, 0, 0, 1), odd, Integer(3), Integer(4));
        var beforeMidpoint = new RationalSegmentInterval(Integer(1), Integer(4), new(1UL << 61, 0, 0, 0, 0), odd);
        int sections = 0, stationary = 0;
        Assert.True(CapsulePlaneRayEvents.VisitEvents(source, frame, orientation,
            (in CapsulePlaneRayEventSource s, in CapsulePlaneRayFrame f, CapsulePlaneRayEvent descriptor, int o,
                scoped in CapsulePlaneRaySelection point) =>
            {
                if (descriptor.Kind is not (CapsulePlaneRayEventKind.Section or CapsulePlaneRayEventKind.Stationary)) return;
                Assert.True(point.TryGetAuthoredPoint(f, Vector3d.Zero, FixedQuaternion.Identity,
                    new Vector3d(Fixed64.Zero, Fixed64.Quarter, Fixed64.Zero), FixedQuaternion.Identity, out Vector3d p));
                Span<ulong> values = stackalloc ulong[CapsulePlaneRaySelection.StorageWords];
                Span<int> signs = stackalloc int[CapsulePlaneRaySelection.SignCount];
                Span<sbyte> valueSigns = stackalloc sbyte[CapsulePlaneRaySelection.ValueSignCount];
                var replay = new CapsulePlaneRaySelection(values, signs, valueSigns);
                Assert.False(CapsulePlaneRayEvents.Reconstruct(s, f, descriptor, o, left, ref replay));
                Assert.False(CapsulePlaneRayEvents.Reconstruct(s, f, descriptor, o, right, ref replay));
                if (descriptor.Kind == CapsulePlaneRayEventKind.Section) { sections++; return; }
                Assert.Equal(Fixed64.Zero, p.X);
                Assert.True(CapsulePlaneRayEvents.Reconstruct(s, f, descriptor, o, fromMidpoint, ref replay));
                Assert.Equal(0, replay.CompareDepth(point));
                Assert.True(CapsulePlaneRayEvents.Reconstruct(s, f, descriptor, o, throughMidpoint, ref replay));
                Assert.Equal(0, replay.CompareDepth(point));
                // The bounds differ from 1/2 by less than one Q32.32 step;
                // narrowing either bound would incorrectly retain this event.
                Assert.False(CapsulePlaneRayEvents.Reconstruct(s, f, descriptor, o, afterMidpoint, ref replay));
                Assert.False(CapsulePlaneRayEvents.Reconstruct(s, f, descriptor, o, beforeMidpoint, ref replay));
                stationary++;
            }));
        Assert.True(sections > 0);
        Assert.True(stationary > 0);
    }

    private static void CheckEndpointPair(RationalSegmentInterval interval, int expectedPointOrder, int expectedDepthOrder)
    {
        var plane = new FixedTriangle(Vector3d.Zero, new Vector3d(2, 0, 0), new Vector3d(0, 0, 2));
        var frame = new CapsulePlaneRayFrame(plane, new CapsuleFiniteSectionFrame(Vector3d.Zero, FixedQuaternion.Identity,
            new Vector3d(Fixed64.Zero, Fixed64.Quarter, Fixed64.Zero), FixedQuaternion.Identity, Fixed64.Zero, Fixed64.One));
        var source = new CapsulePlaneRayEventSource(new FixedSegment(new(-2, 0, 0), new(2, 0, 0)));
        ulong[][] values = { new ulong[CapsulePlaneRaySelection.StorageWords], new ulong[CapsulePlaneRaySelection.StorageWords] };
        int[][] signs = { new int[CapsulePlaneRaySelection.SignCount], new int[CapsulePlaneRaySelection.SignCount] };
        sbyte[][] valueSigns = { new sbyte[CapsulePlaneRaySelection.ValueSignCount], new sbyte[CapsulePlaneRaySelection.ValueSignCount] };
        var metadata = new CapsulePlaneRaySelectionMetadata[2];
        int count = 0;
        Assert.True(CapsulePlaneRayEvents.VisitEvents(source, frame, 1, interval,
            (in CapsulePlaneRayEventSource s, in CapsulePlaneRayFrame f, CapsulePlaneRayEvent e, int o,
                scoped in CapsulePlaneRaySelection point) =>
            {
                Assert.Equal(CapsulePlaneRayEventKind.Endpoint, e.Kind);
                Assert.Equal(count, e.Feature);
                point.Values.CopyTo(values[count]); point.Signs.CopyTo(signs[count]); point.ValueSigns.CopyTo(valueSigns[count]);
                metadata[count++] = point.Metadata;
            }));
        Assert.Equal(2, count);
        var first = new CapsulePlaneRaySelection(values[0], signs[0], valueSigns[0]); first.Restore(metadata[0]);
        var second = new CapsulePlaneRaySelection(values[1], signs[1], valueSigns[1]); second.Restore(metadata[1]);
        Assert.Equal(expectedPointOrder, first.ComparePoint(second, frame));
        Assert.Equal(expectedDepthOrder, first.CompareDepth(second));
    }

    [Fact]
    public void FullIntervals_ShouldUseOrdinaryEventsAndRejectInvalidClippedSources()
    {
        var full = new RationalSegmentInterval(Integer(0), Integer(7), Integer(5), Integer(5));
        Assert.False(full.IsClipped);
        var clip = new RationalSegmentInterval(Integer(1), Integer(3), Integer(2), Integer(3));
        var plane = new FixedTriangle(Vector3d.Zero, new Vector3d(2, 0, 0), new Vector3d(0, 0, 2));
        var frame = new CapsulePlaneRayFrame(plane, new CapsuleFiniteSectionFrame(Vector3d.Zero, FixedQuaternion.Identity,
            Vector3d.Zero, FixedQuaternion.Identity, Fixed64.Zero, Fixed64.One));
        int count = 0, ordinaryCount = 0;
        Assert.True(CapsulePlaneRayEvents.VisitEvents(CapsulePlaneRayEventSource.Plane, frame, 1, full,
            (in CapsulePlaneRayEventSource s, in CapsulePlaneRayFrame f, CapsulePlaneRayEvent e, int o,
                scoped in CapsulePlaneRaySelection point) => count++));
        Assert.True(CapsulePlaneRayEvents.VisitEvents(CapsulePlaneRayEventSource.Plane, frame, 1,
            (in CapsulePlaneRayEventSource s, in CapsulePlaneRayFrame f, CapsulePlaneRayEvent e, int o,
                scoped in CapsulePlaneRaySelection point) => ordinaryCount++));
        Assert.Equal(ordinaryCount, count);
        Assert.Throws<ArgumentNullException>(() => CapsulePlaneRayEvents.VisitEvents(CapsulePlaneRayEventSource.Plane, frame, 1, full, null!));
        Assert.Throws<ArgumentException>(() => CapsulePlaneRayEvents.VisitEvents(CapsulePlaneRayEventSource.Plane, frame, 1, clip,
            (in CapsulePlaneRayEventSource s, in CapsulePlaneRayFrame f, CapsulePlaneRayEvent e, int o,
                scoped in CapsulePlaneRaySelection point) => { }));
    }
}
