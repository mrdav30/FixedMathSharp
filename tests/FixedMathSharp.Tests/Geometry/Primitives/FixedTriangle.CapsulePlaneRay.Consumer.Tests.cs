using System;
using FixedMathSharp.Geometry;
using Xunit;

namespace FixedMathSharp.Tests.Bounds;

public sealed class FixedTriangleCapsulePlaneRayConsumerTests
{
    [Theory]
    [InlineData(0)]
    [InlineData(1)]
    [InlineData(2)]
    [InlineData(3)]
    [InlineData(4)]
    [InlineData(5)]
    [InlineData(6)]
    [InlineData(7)]
    [InlineData(8)]
    [InlineData(9)]
    public void FiniteEventReplay_RetainsExactPointDepthAndSplitExit(int fixture)
    {
        CreateFixture(fixture, out FixedTriangle triangle, out FixedSegment edge,
            out Vector3d center, out FixedQuaternion relativeRotation, out Fixed64 length);
        Vector3d origin = new(3, -2, 1);
        FixedQuaternion meshRotation = FixedQuaternion.FromAxisAngle(Vector3d.Up, Fixed64.PiOver4);
        center = origin + meshRotation.Rotate(center);
        FixedQuaternion rotation = meshRotation * relativeRotation;
        var finite = new CapsuleFiniteSectionFrame(origin, meshRotation, center, rotation, length, Fixed64.One);
        var frame = new CapsulePlaneRayFrame(triangle, finite);
        Span<ulong> expectedValues = stackalloc ulong[CapsulePlaneRaySelection.StorageWords];
        Span<int> expectedSigns = stackalloc int[CapsulePlaneRaySelection.SignCount];
        Span<sbyte> expectedValueSigns = stackalloc sbyte[CapsulePlaneRaySelection.ValueSignCount];
        foreach (CapsulePlaneRayEventSource source in new[]
            { CapsulePlaneRayEventSource.Plane, new CapsulePlaneRayEventSource(edge) })
        foreach (int orientation in new[] { -1, 1 })
        {
            ulong[] maximumValues = new ulong[CapsulePlaneRaySelection.StorageWords];
            int[] maximumSigns = new int[CapsulePlaneRaySelection.SignCount];
            sbyte[] maximumValueSigns = new sbyte[CapsulePlaneRaySelection.ValueSignCount];
            CapsulePlaneRaySelectionMetadata maximumMetadata = default;
            int count = 0;
            bool found = CapsulePlaneRayEvents.VisitEvents(source, frame, orientation,
                (in CapsulePlaneRayEventSource currentSource, in CapsulePlaneRayFrame current,
                    CapsulePlaneRayEvent descriptor, int direction, scoped in CapsulePlaneRaySelection candidate) =>
                {
                    Span<ulong> pointValues = stackalloc ulong[CapsulePlaneRaySelection.StorageWords];
                    Span<int> pointSigns = stackalloc int[CapsulePlaneRaySelection.SignCount];
                    Span<sbyte> pointValueSigns = stackalloc sbyte[CapsulePlaneRaySelection.ValueSignCount];
                    var replay = new CapsulePlaneRaySelection(pointValues, pointSigns, pointValueSigns);
                    Assert.True(CapsulePlaneRayEvents.Reconstruct(currentSource, current, descriptor, direction, ref replay));
                    Assert.Equal(0, replay.ComparePoint(candidate, current));
                    Assert.Equal(candidate.ContainsTrianglePoint(current, triangle), replay.ContainsTrianglePoint(current, triangle));
                    Assert.True(replay.TryGetAuthoredPoint(current, origin, meshRotation, center, rotation, out Vector3d authored));
                    Assert.True(candidate.TryGetAuthoredPoint(current, origin, meshRotation, center, rotation, out Vector3d original));
                    Assert.Equal(original, authored);

                    // A pool retains coefficient banks and restores their copied
                    // metadata; neither operation should resample or reselect q.
                    var maximum = new CapsulePlaneRaySelection(maximumValues, maximumSigns, maximumValueSigns);
                    maximum.Restore(maximumMetadata);
                    maximum.KeepEvaluated(candidate, current);
                    maximumMetadata = maximum.Metadata;

                    Assert.Equal(0, replay.CompareDepth(candidate));
                    Assert.True(replay.TryGetExitAnchor(current, direction, center, rotation,
                        out FixedPointAnchor exitAnchor, out Fixed64 depth, out bool clamped));
                    Assert.False(clamped);
                    Assert.True(exitAnchor.TryGetPoint(out Vector3d exit));
                    Vector3d expected = origin + meshRotation.Rotate(authored)
                        + current.GetWorldNormal(meshRotation) * depth * direction;
                    // The two published pose operations and normal/depth product
                    // round independently; these small fixtures bound their sum.
                    for (int axis = 0; axis < 3; axis++)
                        Assert.InRange(FixedMath.Abs(exit[axis] - expected[axis]).m_rawValue, 0L, 64L);

                    Span<ulong> oppositeValues = stackalloc ulong[CapsulePlaneRaySelection.StorageWords];
                    Span<int> oppositeSigns = stackalloc int[CapsulePlaneRaySelection.SignCount];
                    Span<sbyte> oppositeValueSigns = stackalloc sbyte[CapsulePlaneRaySelection.ValueSignCount];
                    var opposite = new CapsulePlaneRaySelection(oppositeValues, oppositeSigns, oppositeValueSigns);
                    Assert.True(CapsulePlaneRayEvents.Reconstruct(currentSource, current, descriptor, -direction, ref opposite));
                    Assert.Equal(0, opposite.ComparePoint(candidate, current));
                    Assert.Equal(-candidate.CompareDepth(opposite), opposite.CompareDepth(candidate));
                    count++;
                });
            Assert.Equal(count > 0, found);
            Assert.True(found);
            var expectedMaximum = new CapsulePlaneRaySelection(expectedValues, expectedSigns, expectedValueSigns);
            Assert.True(CapsulePlaneRayEvents.Accumulate(source, frame, orientation, ref expectedMaximum));
            var retainedMaximum = new CapsulePlaneRaySelection(maximumValues, maximumSigns, maximumValueSigns);
            retainedMaximum.Restore(maximumMetadata);
            Assert.Equal(0, retainedMaximum.CompareDepth(expectedMaximum));
            Assert.Equal(0, retainedMaximum.ComparePoint(expectedMaximum, frame));
        }
    }

    [Theory]
    [InlineData(0)]
    [InlineData(1)]
    [InlineData(2)]
    public void IncompleteWinnerBanks_ShouldBeRejected(int bank)
    {
        Assert.Throws<ArgumentException>(() =>
        {
            _ = new CapsulePlaneRaySelection(
                new ulong[CapsulePlaneRaySelection.StorageWords - (bank == 0 ? 1 : 0)],
                new int[CapsulePlaneRaySelection.SignCount - (bank == 1 ? 1 : 0)],
                new sbyte[CapsulePlaneRaySelection.ValueSignCount - (bank == 2 ? 1 : 0)]);
        });
    }

    [Fact]
    public void EmptyWinner_ShouldNotPublishPointDepthOrExit()
    {
        Span<ulong> values = stackalloc ulong[CapsulePlaneRaySelection.StorageWords];
        Span<int> signs = stackalloc int[CapsulePlaneRaySelection.SignCount];
        Span<sbyte> valueSigns = stackalloc sbyte[CapsulePlaneRaySelection.ValueSignCount];
        var selection = new CapsulePlaneRaySelection(values, signs, valueSigns);
        var empty = new CapsulePlaneRaySelection(new ulong[CapsulePlaneRaySelection.StorageWords],
            new int[CapsulePlaneRaySelection.SignCount], new sbyte[CapsulePlaneRaySelection.ValueSignCount]);
        selection.KeepEvaluated(empty, default);
        Assert.False(selection.HasValue);
        Assert.False(selection.ContainsTrianglePoint(default, default));
        Assert.False(selection.TryGetDepth(default, out Fixed64 depth, out bool clamped));
        Assert.Equal(Fixed64.Zero, depth); Assert.False(clamped);
        Assert.False(selection.TryGetAuthoredPoint(default, Vector3d.Zero, FixedQuaternion.Identity,
            Vector3d.Zero, FixedQuaternion.Identity, out Vector3d point));
        Assert.Equal(Vector3d.Zero, point);
        Assert.False(selection.TryGetExitAnchor(default, 1, Vector3d.Zero, FixedQuaternion.Identity,
            out _, out depth, out clamped));
        Assert.Equal(Fixed64.Zero, depth); Assert.False(clamped);
    }

    [Fact]
    public void EventVisitor_ShouldBeRequired()
    {
        Assert.Throws<ArgumentNullException>(() =>
            CapsulePlaneRayEvents.VisitEvents(CapsulePlaneRayEventSource.Plane, default, 1, null!));
    }

    [Theory]
    [InlineData(-1)]
    [InlineData(1)]
    public void OffPlaneFiniteSource_ShouldNotBorrowIntrinsicAdmission(int orientation)
    {
        var triangle = new FixedTriangle(new Vector3d(-8, 0, -8), new Vector3d(8, 0, -8), new Vector3d(0, 0, 8));
        var finite = new CapsuleFiniteSectionFrame(Vector3d.Zero, FixedQuaternion.Identity, Vector3d.Zero,
            FixedQuaternion.Identity, Fixed64.Two, Fixed64.One);
        var frame = new CapsulePlaneRayFrame(triangle, finite);
        var source = new CapsulePlaneRayEventSource(new FixedSegment(
            new Vector3d((Fixed64)(-2), Fixed64.Quarter, Fixed64.Zero),
            new Vector3d((Fixed64)2, Fixed64.Quarter, Fixed64.Zero)));
        Span<ulong> values = stackalloc ulong[CapsulePlaneRaySelection.StorageWords];
        Span<int> signs = stackalloc int[CapsulePlaneRaySelection.SignCount];
        Span<sbyte> valueSigns = stackalloc sbyte[CapsulePlaneRaySelection.ValueSignCount];
        var selection = new CapsulePlaneRaySelection(values, signs, valueSigns);
        Assert.False(CapsulePlaneRayEvents.Accumulate(source, frame, orientation, ref selection));
        Assert.False(CapsulePlaneRayEvents.Reconstruct(source, frame,
            new CapsulePlaneRayEvent(CapsulePlaneRayEventKind.Section, 0, 1), orientation, ref selection));
        Assert.False(selection.HasValue);

        // The intrinsic plane circle is still valid; only this off-plane
        // source loses the shared plane certificate.
        Assert.True(CapsulePlaneRayEvents.Reconstruct(CapsulePlaneRayEventSource.Plane, frame,
            new CapsulePlaneRayEvent(CapsulePlaneRayEventKind.Circle, 0, 0), orientation, ref selection));
        Assert.True(selection.TryGetAuthoredPoint(frame, Vector3d.Zero, FixedQuaternion.Identity,
            Vector3d.Zero, FixedQuaternion.Identity, out Vector3d point));
        Assert.Equal(Fixed64.Zero, point.Y);
        Assert.Equal(Fixed64.One, point.MagnitudeSquared);
    }

    [Theory]
    [InlineData(-1)]
    [InlineData(1)]
    public void TiltedFiniteSource_ShouldRetainItsClosedOnPlaneEndpoint(int orientation)
    {
        var triangle = new FixedTriangle(new Vector3d(-8, 0, -8), new Vector3d(8, 0, -8), new Vector3d(0, 0, 8));
        var finite = new CapsuleFiniteSectionFrame(Vector3d.Zero, FixedQuaternion.Identity, Vector3d.Zero,
            FixedQuaternion.Identity, Fixed64.Zero, Fixed64.One);
        var frame = new CapsulePlaneRayFrame(triangle, finite);
        Vector3d endpoint = new Vector3d(1, 0, 0);
        var source = new CapsulePlaneRayEventSource(new FixedSegment(endpoint, new Vector3d(2, 1, 0)));
        bool sawEndpoint = false, sawSection = false;
        // The source starts on the plane but immediately leaves both the
        // plane and sphere. Its closed endpoint remains an admitted event.
        Assert.True(CapsulePlaneRayEvents.VisitEvents(source, frame, orientation,
            (in CapsulePlaneRayEventSource currentSource, in CapsulePlaneRayFrame current,
                CapsulePlaneRayEvent descriptor, int direction, scoped in CapsulePlaneRaySelection candidate) =>
            {
                Assert.True(candidate.TryGetAuthoredPoint(current, Vector3d.Zero, FixedQuaternion.Identity,
                    Vector3d.Zero, FixedQuaternion.Identity, out Vector3d point));
                Assert.Equal(endpoint, point);
                Assert.True(candidate.TryGetExitAnchor(current, direction, Vector3d.Zero, FixedQuaternion.Identity,
                    out FixedPointAnchor anchor, out Fixed64 depth, out bool clamped));
                Assert.False(clamped);
                Assert.Equal(Fixed64.Zero, depth);
                Assert.True(anchor.TryGetPoint(out Vector3d exit));
                Assert.Equal(endpoint, exit);
                sawEndpoint |= descriptor.Kind == CapsulePlaneRayEventKind.Endpoint;
                sawSection |= descriptor.Kind == CapsulePlaneRayEventKind.Section;
            }));
        Assert.True(sawEndpoint);
        Assert.True(sawSection);
    }

    [Theory]
    [InlineData(-1, false)]
    [InlineData(1, false)]
    [InlineData(-1, true)]
    [InlineData(1, true)]
    public void ProjectedSupport_ShouldMatchAnalyticExitAndOppositeBodyReplay(int orientation, bool oblique)
    {
        var triangle = oblique
            ? new FixedTriangle(new Vector3d(-8, 6, -8), new Vector3d(8, -6, -8), new Vector3d(0, 0, 8))
            : new FixedTriangle(new Vector3d(-8, 0, -8), new Vector3d(8, 0, -8), new Vector3d(0, 0, 8));
        var finite = new CapsuleFiniteSectionFrame(Vector3d.Zero, FixedQuaternion.Identity, Vector3d.Zero,
            FixedQuaternion.Identity, Fixed64.Two, Fixed64.One);
        var frame = new CapsulePlaneRayFrame(triangle, finite);
        var descriptor = new CapsulePlaneRayEvent(CapsulePlaneRayEventKind.Support, orientation > 0 ? 1 : 0);
        Span<ulong> values = stackalloc ulong[CapsulePlaneRaySelection.StorageWords];
        Span<int> signs = stackalloc int[CapsulePlaneRaySelection.SignCount];
        Span<sbyte> valueSigns = stackalloc sbyte[CapsulePlaneRaySelection.ValueSignCount];
        var selection = new CapsulePlaneRaySelection(values, signs, valueSigns);
        Assert.True(CapsulePlaneRayEvents.Reconstruct(CapsulePlaneRayEventSource.Plane, frame,
            descriptor, orientation, ref selection));
        Assert.True(selection.TryGetExitAnchor(frame, orientation, Vector3d.Zero, FixedQuaternion.Identity,
            out FixedPointAnchor anchor, out Fixed64 depth, out bool clamped));
        Assert.False(clamped);
        Fixed64 expectedDepth = oblique ? (Fixed64)9 / 5 : Fixed64.Two;
        Assert.InRange(FixedMath.Abs(depth - expectedDepth).m_rawValue, 0L, 2L);
        Assert.True(anchor.TryGetPoint(out Vector3d exit));
        Vector3d expectedExit = oblique
            ? new Vector3d((Fixed64)(3 * orientation) / 5, (Fixed64)(9 * orientation) / 5, Fixed64.Zero)
            : Vector3d.Up * (Fixed64)(2 * orientation);
        Assert.InRange((exit - expectedExit).MagnitudeSquared.m_rawValue, 0L, 2L);

        // The same projected endpoint replayed toward the other hemisphere
        // can exit the body, rather than that endpoint's spherical support.
        selection = new CapsulePlaneRaySelection(values, signs, valueSigns);
        Assert.True(CapsulePlaneRayEvents.Reconstruct(CapsulePlaneRayEventSource.Plane, frame,
            descriptor, -orientation, ref selection));
        Assert.True(selection.TryGetExitAnchor(frame, -orientation, Vector3d.Zero, FixedQuaternion.Identity,
            out anchor, out depth, out clamped));
        Assert.False(clamped);
        expectedDepth = oblique ? (Fixed64)13 / 15 : Fixed64.Two;
        Assert.InRange(FixedMath.Abs(depth - expectedDepth).m_rawValue, 0L, 2L);
        Assert.True(anchor.TryGetPoint(out exit));
        expectedExit = oblique
            ? new Vector3d((Fixed64)(-orientation), (Fixed64)(-orientation) / 3, Fixed64.Zero)
            : Vector3d.Up * (Fixed64)(-2 * orientation);
        Assert.InRange((exit - expectedExit).MagnitudeSquared.m_rawValue, 0L, 2L);
    }

    [Theory]
    [InlineData(-1)]
    [InlineData(1)]
    public void SphereProjectedSupport_ShouldRetainSignedDepthInRotatedFrame(int orientation)
    {
        var triangle = new FixedTriangle(new Vector3d(-8, 0, -8), new Vector3d(8, 0, -8), new Vector3d(0, 0, 8));
        Vector3d center = Vector3d.Up * Fixed64.Quarter;
        FixedQuaternion rotation = FixedQuaternion.LookRotation(new Vector3d(1, 0, 1).Normalized,
            new Vector3d(1, 1, -1).Normalized);
        var finite = new CapsuleFiniteSectionFrame(Vector3d.Zero, FixedQuaternion.Identity, center, rotation,
            Fixed64.Zero, Fixed64.One);
        var frame = new CapsulePlaneRayFrame(triangle, finite);
        Span<ulong> values = stackalloc ulong[CapsulePlaneRaySelection.StorageWords];
        Span<int> signs = stackalloc int[CapsulePlaneRaySelection.SignCount];
        Span<sbyte> valueSigns = stackalloc sbyte[CapsulePlaneRaySelection.ValueSignCount];
        var selection = new CapsulePlaneRaySelection(values, signs, valueSigns);
        Assert.True(CapsulePlaneRayEvents.Reconstruct(CapsulePlaneRayEventSource.Plane, frame,
            new CapsulePlaneRayEvent(CapsulePlaneRayEventKind.Support), orientation, ref selection));
        Assert.True(selection.TryGetExitAnchor(frame, orientation, center, rotation,
            out FixedPointAnchor anchor, out Fixed64 depth, out bool clamped));
        Assert.False(clamped);
        Assert.Equal(orientation > 0 ? (Fixed64)5 / 4 : (Fixed64)3 / 4, depth);
        Assert.True(anchor.TryGetPoint(out Vector3d exit));
        Assert.InRange((exit - (center + Vector3d.Up * (Fixed64)orientation)).MagnitudeSquared.m_rawValue, 0L, 2L);
    }

    [Theory]
    [InlineData(-1)]
    [InlineData(1)]
    public void CoreParallelPlaneSupport_ShouldRetainClosedBodySeamTie(int orientation)
    {
        var triangle = new FixedTriangle(new Vector3d(0, -8, -8), new Vector3d(0, 8, -8), new Vector3d(0, 0, 8));
        var finite = new CapsuleFiniteSectionFrame(Vector3d.Zero, FixedQuaternion.Identity, Vector3d.Zero,
            FixedQuaternion.Identity, Fixed64.Two, Fixed64.One);
        var frame = new CapsulePlaneRayFrame(triangle, finite);
        Span<ulong> values = stackalloc ulong[CapsulePlaneRaySelection.StorageWords];
        Span<int> signs = stackalloc int[CapsulePlaneRaySelection.SignCount];
        Span<sbyte> valueSigns = stackalloc sbyte[CapsulePlaneRaySelection.ValueSignCount];
        for (int endpoint = 0; endpoint < 2; endpoint++)
        {
            var selection = new CapsulePlaneRaySelection(values, signs, valueSigns);
            Assert.True(CapsulePlaneRayEvents.Reconstruct(CapsulePlaneRayEventSource.Plane, frame,
                new CapsulePlaneRayEvent(CapsulePlaneRayEventKind.Support, endpoint), orientation, ref selection));
            // The body owns the same closed exit as the endpoint hemisphere.
            // Retaining that primitive preserves the exact seam tie convention.
            Assert.Equal(0, selection.Metadata.Primitive);
            Assert.True(selection.TryGetExitAnchor(frame, orientation, Vector3d.Zero, FixedQuaternion.Identity,
                out FixedPointAnchor anchor, out Fixed64 depth, out bool clamped));
            Assert.False(clamped);
            Assert.Equal(Fixed64.One, depth);
            Assert.True(anchor.TryGetPoint(out Vector3d exit));
            Assert.Equal(new Vector3d(orientation, endpoint == 0 ? -1 : 1, 0), exit);
        }
    }

    [Theory]
    [InlineData(-1, false)]
    [InlineData(1, false)]
    [InlineData(-1, true)]
    [InlineData(1, true)]
    public void CircleFamilies_ShouldRetainSignedSphereExitAndOddCoreSeam(int orientation, bool oddCore)
    {
        var triangle = new FixedTriangle(new Vector3d(-8, 0, -8), new Vector3d(8, 0, -8), new Vector3d(0, 0, 8));
        Vector3d center = oddCore ? Vector3d.Zero : Vector3d.Up * Fixed64.Quarter;
        FixedQuaternion rotation = oddCore ? FixedQuaternion.FromAxisAngle(Vector3d.Up, Fixed64.PiOver4)
            : FixedQuaternion.LookRotation(new Vector3d(1, 0, 1).Normalized, new Vector3d(1, 1, -1).Normalized);
        var finite = new CapsuleFiniteSectionFrame(Vector3d.Zero, FixedQuaternion.Identity, center, rotation,
            oddCore ? Fixed64.FromRaw(1) : Fixed64.Zero, Fixed64.One);
        var frame = new CapsulePlaneRayFrame(triangle, finite);
        int count = 0;
        Assert.True(CapsulePlaneRayEvents.VisitEvents(CapsulePlaneRayEventSource.Plane, frame, orientation,
            (in CapsulePlaneRayEventSource source, in CapsulePlaneRayFrame current, CapsulePlaneRayEvent descriptor,
                int direction, scoped in CapsulePlaneRaySelection candidate) =>
            {
                if (descriptor.Kind != CapsulePlaneRayEventKind.Circle) return;
                Assert.Equal(oddCore ? 0 : 2, descriptor.Feature);
                Assert.True(candidate.TryGetAuthoredPoint(current, Vector3d.Zero, FixedQuaternion.Identity,
                    center, rotation, out Vector3d point));
                Assert.Equal(Fixed64.Zero, point.Y);
                Assert.True(candidate.TryGetExitAnchor(current, direction, center, rotation,
                    out FixedPointAnchor anchor, out Fixed64 depth, out bool clamped));
                Assert.False(clamped);
                Assert.Equal(!oddCore && direction > 0 ? Fixed64.Half : Fixed64.Zero, depth);
                if (oddCore)
                {
                    // The signed half-raw seam rounds to zero, but remains
                    // distinct from an anchor containing only rounded offsets.
                    var rounded = new FixedPointAnchor(center, rotation, anchor.LocalPoint, anchor.LocalDisplacement);
                    Assert.NotEqual(rounded, anchor);
                    Assert.Equal(Fixed64.One, point.MagnitudeSquared);
                }
                else
                {
                    Assert.True(anchor.TryGetPoint(out Vector3d exit));
                    Assert.InRange(FixedMath.Abs((exit - center).MagnitudeSquared - Fixed64.One).m_rawValue, 0L, 32L);
                }
                count++;
            }));
        Assert.Equal(4, count);
    }

    [Theory]
    [InlineData(0)]
    [InlineData(-2)]
    [InlineData(2)]
    public void UnsupportedRayOrientation_ShouldBeRejectedBeforeAdmission(int orientation)
    {
        Assert.Throws<ArgumentOutOfRangeException>(() =>
        {
            var selection = new CapsulePlaneRaySelection(new ulong[CapsulePlaneRaySelection.StorageWords],
                new int[CapsulePlaneRaySelection.SignCount], new sbyte[CapsulePlaneRaySelection.ValueSignCount]);
            CapsulePlaneRayEvents.Accumulate(CapsulePlaneRayEventSource.Plane, default, orientation, ref selection);
        });
        Assert.Throws<ArgumentOutOfRangeException>(() =>
        {
            var selection = new CapsulePlaneRaySelection(new ulong[CapsulePlaneRaySelection.StorageWords],
                new int[CapsulePlaneRaySelection.SignCount], new sbyte[CapsulePlaneRaySelection.ValueSignCount]);
            selection.TryGetExitAnchor(default, orientation, Vector3d.Zero, FixedQuaternion.Identity,
                out _, out _, out _);
        });
    }

    [Theory]
    [InlineData(-1, false)]
    [InlineData(1, false)]
    [InlineData(-1, true)]
    [InlineData(1, true)]
    public void AxialCapOnlySection_ShouldRejectTheOtherFullBallHemisphere(int side, bool overlappingBalls)
    {
        var triangle = new FixedTriangle(new Vector3d(-8, 0, -8), new Vector3d(8, 0, -8), new Vector3d(0, 0, 8));
        Fixed64 length = overlappingBalls ? Fixed64.Half : Fixed64.Two;
        Vector3d center = Vector3d.Up * (overlappingBalls ? Fixed64.Half : (Fixed64)3 / 2) * side;
        var finite = new CapsuleFiniteSectionFrame(Vector3d.Zero, FixedQuaternion.Identity, center,
            FixedQuaternion.Identity, length, Fixed64.One);
        var frame = new CapsulePlaneRayFrame(triangle, finite);
        int circles = 0;
        Assert.True(CapsulePlaneRayEvents.VisitEvents(CapsulePlaneRayEventSource.Plane, frame, side,
            (in CapsulePlaneRayEventSource source, in CapsulePlaneRayFrame current,
                CapsulePlaneRayEvent descriptor, int direction, scoped in CapsulePlaneRaySelection candidate) =>
            {
                if (descriptor.Kind != CapsulePlaneRayEventKind.Circle) return;
                // Endpoint balls overlap for the short core, but only the
                // outward hemisphere belongs to the actual capsule surface.
                Assert.Equal(side > 0 ? 1 : 2, (int)descriptor.Feature);
                Assert.True(candidate.TryGetAuthoredPoint(current, Vector3d.Zero, FixedQuaternion.Identity,
                    center, FixedQuaternion.Identity, out Vector3d point));
                Assert.Equal(Fixed64.Zero, point.Y);
                Vector3d capCenter = center - Vector3d.Up * length * Fixed64.Half * side;
                Assert.InRange(FixedMath.Abs((point - capCenter).MagnitudeSquared - Fixed64.One).m_rawValue, 0L, 8L);
                Assert.True(candidate.TryGetExitAnchor(current, direction, center, FixedQuaternion.Identity,
                    out FixedPointAnchor anchor, out _, out bool clamped));
                Assert.False(clamped); Assert.True(anchor.TryGetPoint(out Vector3d exit));
                Vector3d exitCap = center + Vector3d.Up * length * Fixed64.Half * (exit.Y >= center.Y ? 1 : -1);
                Assert.InRange(FixedMath.Abs((exit - exitCap).MagnitudeSquared - Fixed64.One).m_rawValue, 0L, 8L);
                circles++;
            }));
        Assert.Equal(4, circles);
    }

    [Theory]
    [InlineData(-1)]
    [InlineData(1)]
    public void SphereZPlane_ShouldUseItsAuthoredCircularSection(int orientation)
    {
        var triangle = new FixedTriangle(new Vector3d(-8, -8, 0), new Vector3d(8, -8, 0), new Vector3d(0, 8, 0));
        Vector3d center = Vector3d.Forward * Fixed64.Quarter;
        var finite = new CapsuleFiniteSectionFrame(Vector3d.Zero, FixedQuaternion.Identity, center,
            FixedQuaternion.Identity, Fixed64.Zero, Fixed64.One);
        var frame = new CapsulePlaneRayFrame(triangle, finite);
        int circles = 0;
        Assert.True(CapsulePlaneRayEvents.VisitEvents(CapsulePlaneRayEventSource.Plane, frame, orientation,
            (in CapsulePlaneRayEventSource source, in CapsulePlaneRayFrame current,
                CapsulePlaneRayEvent descriptor, int direction, scoped in CapsulePlaneRaySelection candidate) =>
            {
                if (descriptor.Kind != CapsulePlaneRayEventKind.Circle) return;
                Assert.True(candidate.TryGetAuthoredPoint(current, Vector3d.Zero, FixedQuaternion.Identity,
                    center, FixedQuaternion.Identity, out Vector3d point));
                Assert.Equal(Fixed64.Zero, point.Z);
                Assert.InRange(FixedMath.Abs(point.MagnitudeSquared - (Fixed64)15 / 16).m_rawValue, 0L, 8L);
                Assert.True(candidate.TryGetExitAnchor(current, direction, center, FixedQuaternion.Identity,
                    out FixedPointAnchor anchor, out Fixed64 depth, out bool clamped));
                Assert.False(clamped); Assert.Equal(direction > 0 ? Fixed64.Half : Fixed64.Zero, depth);
                Assert.True(anchor.TryGetPoint(out Vector3d exit));
                Assert.Equal(point + Vector3d.Forward * depth * direction, exit);
                circles++;
            }));
        Assert.Equal(4, circles);
    }

    [Fact]
    public void AxialPlaneBeyondBothCaps_ShouldHaveNoFiniteEvents()
    {
        var triangle = new FixedTriangle(new Vector3d(-8, 0, -8), new Vector3d(8, 0, -8), new Vector3d(0, 0, 8));
        var finite = new CapsuleFiniteSectionFrame(Vector3d.Zero, FixedQuaternion.Identity, Vector3d.Up * 3,
            FixedQuaternion.Identity, Fixed64.Two, Fixed64.One);
        var frame = new CapsulePlaneRayFrame(triangle, finite);
        int count = 0;
        Assert.False(CapsulePlaneRayEvents.VisitEvents(CapsulePlaneRayEventSource.Plane, frame, 1,
            (in CapsulePlaneRayEventSource source, in CapsulePlaneRayFrame current,
                CapsulePlaneRayEvent descriptor, int direction, scoped in CapsulePlaneRaySelection candidate) => count++));
        Assert.Equal(0, count);
    }

    [Fact]
    public void InvertedCapsuleAxis_ShouldPreserveTheObliqueBodySectionMaximum()
    {
        var triangle = new FixedTriangle(new Vector3d(-8, 6, -8), new Vector3d(8, -6, -8), new Vector3d(0, 0, 8));
        var rotation = new FixedQuaternion(Fixed64.One, Fixed64.Zero, Fixed64.Zero, Fixed64.Zero);
        var finite = new CapsuleFiniteSectionFrame(Vector3d.Zero, FixedQuaternion.Identity, Vector3d.Zero,
            rotation, (Fixed64)8, Fixed64.One);
        var frame = new CapsulePlaneRayFrame(triangle, finite);
        Span<ulong> values = stackalloc ulong[CapsulePlaneRaySelection.StorageWords];
        Span<int> signs = stackalloc int[CapsulePlaneRaySelection.SignCount];
        Span<sbyte> valueSigns = stackalloc sbyte[CapsulePlaneRaySelection.ValueSignCount];
        var maximum = new CapsulePlaneRaySelection(values, signs, valueSigns);
        Assert.True(CapsulePlaneRayEvents.Accumulate(CapsulePlaneRayEventSource.Plane, frame, 1, ref maximum));
        Assert.True(maximum.TryGetDepth(frame, out Fixed64 depth, out bool clamped));
        Assert.False(clamped); Assert.InRange(FixedMath.Abs(depth - (Fixed64)10 / 3).m_rawValue, 0L, 2L);
        Assert.True(CapsulePlaneRayEvents.IntersectsTriangle(finite, triangle));
    }

    [Fact]
    public void RetainedDescriptor_ShouldRejectAnIncompatibleSourceOrChangedSection()
    {
        var triangle = new FixedTriangle(new Vector3d(-8, 0, -8), new Vector3d(8, 0, -8), new Vector3d(0, 0, 8));
        var bodyFinite = new CapsuleFiniteSectionFrame(Vector3d.Zero, FixedQuaternion.Identity, Vector3d.Zero,
            FixedQuaternion.Identity, Fixed64.Two, Fixed64.One);
        var body = new CapsulePlaneRayFrame(triangle, bodyFinite);
        var sphereFinite = new CapsuleFiniteSectionFrame(Vector3d.Zero, FixedQuaternion.Identity, Vector3d.Zero,
            FixedQuaternion.Identity, Fixed64.Zero, Fixed64.One);
        var sphere = new CapsulePlaneRayFrame(triangle, sphereFinite);
        var capFinite = new CapsuleFiniteSectionFrame(Vector3d.Zero, FixedQuaternion.Identity, Vector3d.Up * ((Fixed64)3 / 2),
            FixedQuaternion.Identity, Fixed64.Two, Fixed64.One);
        var cap = new CapsulePlaneRayFrame(triangle, capFinite);
        Span<ulong> values = stackalloc ulong[CapsulePlaneRaySelection.StorageWords];
        Span<int> signs = stackalloc int[CapsulePlaneRaySelection.SignCount];
        Span<sbyte> valueSigns = stackalloc sbyte[CapsulePlaneRaySelection.ValueSignCount];
        var point = new CapsulePlaneRaySelection(values, signs, valueSigns);
        var circle = new CapsulePlaneRayEvent(CapsulePlaneRayEventKind.Circle, 0, 0);
        Assert.True(CapsulePlaneRayEvents.Reconstruct(CapsulePlaneRayEventSource.Plane, body, circle, 1, ref point));
        Assert.False(CapsulePlaneRayEvents.Reconstruct(CapsulePlaneRayEventSource.Plane, cap, circle, 1, ref point));
        Assert.False(CapsulePlaneRayEvents.Reconstruct(new CapsulePlaneRayEventSource(triangle.GetEdge(0)), body, circle, 1, ref point));
        var angled = new FixedTriangle(new Vector3d(-8, 6, -8), new Vector3d(8, -6, -8), new Vector3d(0, 0, 8));
        var oblique = new CapsulePlaneRayFrame(angled, bodyFinite);
        var sagittal = new CapsulePlaneRayEvent(CapsulePlaneRayEventKind.Sagittal, 0, 1);
        Assert.True(CapsulePlaneRayEvents.Reconstruct(CapsulePlaneRayEventSource.Plane, oblique, sagittal, 1, ref point));
        Assert.False(CapsulePlaneRayEvents.Reconstruct(CapsulePlaneRayEventSource.Plane, sphere, sagittal, 1, ref point));
    }

    [Theory]
    [InlineData(0, false)]
    [InlineData(0, true)]
    [InlineData(1, false)]
    [InlineData(1, true)]
    [InlineData(2, false)]
    [InlineData(2, true)]
    [InlineData(3, false)]
    [InlineData(3, true)]
    [InlineData(4, false)]
    [InlineData(4, true)]
    [InlineData(5, false)]
    [InlineData(5, true)]
    [InlineData(6, false)]
    [InlineData(6, true)]
    [InlineData(7, false)]
    [InlineData(7, true)]
    public void CircularReplay_ShouldPreserveTheClosedSectionInventory(int scene, bool invertedAxis)
    {
        var triangle = new FixedTriangle(new Vector3d(-8, 0, -8), new Vector3d(8, 0, -8), new Vector3d(0, 0, 8));
        Fixed64 length = scene == 0 ? Fixed64.Zero : Fixed64.Two;
        Fixed64 height = scene switch
        {
            2 => -Fixed64.One,
            3 => Fixed64.One,
            4 => -(Fixed64)3 / 2,
            5 => (Fixed64)3 / 2,
            7 => (Fixed64)3,
            _ => Fixed64.Zero
        };
        Vector3d center = Vector3d.Up * height;
        FixedQuaternion rotation = invertedAxis
            ? new FixedQuaternion(Fixed64.One, Fixed64.Zero, Fixed64.Zero, Fixed64.Zero)
            : FixedQuaternion.Identity;
        if (scene == 6)
            rotation = new FixedQuaternion(Fixed64.Zero, Fixed64.Zero, Fixed64.One, Fixed64.One).Normalized;
        var finite = new CapsuleFiniteSectionFrame(Vector3d.Zero, FixedQuaternion.Identity, center,
            rotation, length, Fixed64.One);
        var frame = new CapsulePlaneRayFrame(triangle, finite);
        Span<ulong> values = stackalloc ulong[CapsulePlaneRaySelection.StorageWords];
        Span<int> signs = stackalloc int[CapsulePlaneRaySelection.SignCount];
        Span<sbyte> valueSigns = stackalloc sbyte[CapsulePlaneRaySelection.ValueSignCount];
        foreach (int orientation in new[] { -1, 1 })
        {
            int visitedFeatures = 0, circles = 0;
            CapsulePlaneRayEvents.VisitEvents(CapsulePlaneRayEventSource.Plane, frame, orientation,
                (in CapsulePlaneRayEventSource source, in CapsulePlaneRayFrame current,
                    CapsulePlaneRayEvent descriptor, int direction, scoped in CapsulePlaneRaySelection candidate) =>
                {
                    if (descriptor.Kind != CapsulePlaneRayEventKind.Circle) return;
                    visitedFeatures |= 1 << descriptor.Feature;
                    circles++;
                    Span<ulong> replayValues = stackalloc ulong[CapsulePlaneRaySelection.StorageWords];
                    Span<int> replaySigns = stackalloc int[CapsulePlaneRaySelection.SignCount];
                    Span<sbyte> replayValueSigns = stackalloc sbyte[CapsulePlaneRaySelection.ValueSignCount];
                    var replay = new CapsulePlaneRaySelection(replayValues, replaySigns, replayValueSigns);
                    Assert.True(CapsulePlaneRayEvents.Reconstruct(source, current, descriptor, direction, ref replay));
                    Assert.Equal(0, replay.ComparePoint(candidate, current));
                    Assert.Equal(0, replay.CompareDepth(candidate));
                    Assert.True(replay.TryGetAuthoredPoint(current, Vector3d.Zero, FixedQuaternion.Identity,
                        center, rotation, out Vector3d point));
                    Assert.Equal(Fixed64.Zero, point.Y);
                    // Every circle point lies on the capsule's closed boundary,
                    // regardless of which endpoint its authored axis calls lower.
                    Fixed64 coreY = FixedMath.Clamp(point.Y, height - length * Fixed64.Half, height + length * Fixed64.Half);
                    Assert.InRange(FixedMath.Abs((point - Vector3d.Up * coreY).MagnitudeSquared - Fixed64.One).m_rawValue, 0L, 8L);
                    Assert.True(replay.TryGetExitAnchor(current, direction, center, rotation,
                        out FixedPointAnchor anchor, out Fixed64 depth, out bool clamped));
                    Assert.False(clamped); Assert.True(anchor.TryGetPoint(out Vector3d exit));
                    Assert.Equal(point + Vector3d.Up * depth * direction, exit);
                });
            Assert.Equal(scene is 6 or 7 ? 0 : 4, circles);
            for (int feature = 0; feature < 4; feature++)
            {
                var replay = new CapsulePlaneRaySelection(values, signs, valueSigns);
                // Feature3 is invalid. At either shared seam only body0 is
                // replayable; the geometrically coincident cap circle is stale.
                Assert.Equal((visitedFeatures & (1 << feature)) != 0,
                    CapsulePlaneRayEvents.Reconstruct(CapsulePlaneRayEventSource.Plane, frame,
                        new CapsulePlaneRayEvent(CapsulePlaneRayEventKind.Circle, feature, 0), orientation, ref replay));
            }
        }
    }

    [Fact]
    public void LowerCapSectionRay_ShouldRejectAnUpperBallExitBelowItsHemisphere()
    {
        Vector3d basePoint = new((Fixed64)31 / 32, -(Fixed64)5 / 16, Fixed64.Zero);
        var triangle = new FixedTriangle(basePoint + Vector3d.Backward * 2, basePoint + Vector3d.Forward * 2,
            basePoint + new Vector3d(5, -12, 0));
        var finite = new CapsuleFiniteSectionFrame(Vector3d.Zero, FixedQuaternion.Identity, Vector3d.Zero,
            FixedQuaternion.Identity, Fixed64.Half, Fixed64.One);
        var frame = new CapsulePlaneRayFrame(triangle, finite);
        var source = new CapsulePlaneRayEventSource(new FixedSegment(basePoint + Vector3d.Backward, basePoint + Vector3d.Forward));
        Span<ulong> values = stackalloc ulong[CapsulePlaneRaySelection.StorageWords];
        Span<int> signs = stackalloc int[CapsulePlaneRaySelection.SignCount];
        Span<sbyte> valueSigns = stackalloc sbyte[CapsulePlaneRaySelection.ValueSignCount];
        var selection = new CapsulePlaneRaySelection(values, signs, valueSigns);
        Assert.True(CapsulePlaneRayEvents.Reconstruct(source, frame,
            new CapsulePlaneRayEvent(CapsulePlaneRayEventKind.Section, 1, 1), -1, ref selection));
        Assert.Equal(1, selection.Primitive);
        Assert.True(selection.TryGetAuthoredPoint(frame, Vector3d.Zero, FixedQuaternion.Identity,
            Vector3d.Zero, FixedQuaternion.Identity, out Vector3d point));
        Assert.Equal(basePoint.X, point.X); Assert.Equal(basePoint.Y, point.Y);
        Assert.InRange(FixedMath.Abs(point.Z * point.Z - (Fixed64)59 / 1024).m_rawValue, 0L, 2L);
        Assert.True(selection.TryGetExitAnchor(frame, -1, Vector3d.Zero, FixedQuaternion.Identity,
            out FixedPointAnchor anchor, out Fixed64 depth, out bool clamped));
        Assert.False(clamped); Assert.True(depth > Fixed64.Zero); Assert.True(anchor.TryGetPoint(out Vector3d exit));
        Assert.True(exit.Y < -Fixed64.Quarter);
        Assert.InRange(FixedMath.Abs((exit + Vector3d.Up * Fixed64.Quarter).MagnitudeSquared - Fixed64.One).m_rawValue, 0L, 8L);
    }

    [Theory]
    [InlineData(-1)]
    [InlineData(0)]
    public void RationalPoint_ShouldRejectNonpositiveDenominators(int denominator)
    {
        Assert.False(AccumulateInvalidPoint(1, denominator, false));
    }

    [Fact]
    public void DegeneratePlane_ShouldHaveNoDirectionalEvents()
    {
        Assert.False(AccumulateInvalidPoint(1, 1, true));
        var finite = new CapsuleFiniteSectionFrame(Vector3d.Zero, FixedQuaternion.Identity, Vector3d.Zero,
            FixedQuaternion.Identity, Fixed64.Two, Fixed64.One);
        var frame = new CapsulePlaneRayFrame(new FixedTriangle(Vector3d.Zero, Vector3d.Zero, Vector3d.Zero), finite);
        Assert.False(CapsulePlaneRayEvents.VisitEvents(CapsulePlaneRayEventSource.Plane, frame, 1,
            (in CapsulePlaneRayEventSource source, in CapsulePlaneRayFrame current,
                CapsulePlaneRayEvent descriptor, int direction, scoped in CapsulePlaneRaySelection candidate) => Assert.Fail("Degenerate plane emitted an event.")));
    }

    [Theory]
    [InlineData(0)]
    [InlineData(-2)]
    [InlineData(2)]
    public void RationalPoint_ShouldRequireAnExplicitSignedOrientation(int orientation)
    {
        Assert.Throws<ArgumentOutOfRangeException>(() => AccumulateInvalidPoint(orientation, 1, false));
    }

    [Fact]
    public void SphereStationaryPoint_ShouldReplayAcrossBothHemispheres()
    {
        var triangle = new FixedTriangle(new Vector3d(-8, 0, -8), new Vector3d(8, 0, -8), new Vector3d(0, 0, 8));
        var finite = new CapsuleFiniteSectionFrame(Vector3d.Zero, FixedQuaternion.Identity, Vector3d.Zero,
            FixedQuaternion.Identity, Fixed64.Zero, (Fixed64)5);
        var frame = new CapsulePlaneRayFrame(triangle, finite);
        var source = new CapsulePlaneRayEventSource(new FixedSegment(
            new Vector3d((Fixed64)2, (Fixed64)3 / 4, Fixed64.Zero),
            new Vector3d((Fixed64)4, -(Fixed64)3 / 4, Fixed64.Zero)));
        Span<ulong> values = stackalloc ulong[CapsulePlaneRaySelection.StorageWords];
        Span<int> signs = stackalloc int[CapsulePlaneRaySelection.SignCount];
        Span<sbyte> valueSigns = stackalloc sbyte[CapsulePlaneRaySelection.ValueSignCount];
        var selection = new CapsulePlaneRaySelection(values, signs, valueSigns);
        var descriptor = new CapsulePlaneRayEvent(CapsulePlaneRayEventKind.Stationary, 2, 1, 1);
        // The tilted source has its positive stationary ray at q=(3,0,0),
        // ending at (3,4,0). Replay keeps that same q; a sphere has no finite
        // core hemisphere restriction on its equally valid negative exit.
        foreach (int orientation in new[] { 1, -1 })
        {
            selection.HasValue = false;
            Assert.True(CapsulePlaneRayEvents.Reconstruct(source, frame, descriptor, orientation, ref selection));
            Assert.True(selection.TryGetAuthoredPoint(frame, Vector3d.Zero, FixedQuaternion.Identity,
                Vector3d.Zero, FixedQuaternion.Identity, out Vector3d point));
            Assert.Equal(new Vector3d(3, 0, 0), point);
            Assert.True(selection.TryGetExitAnchor(frame, orientation, Vector3d.Zero, FixedQuaternion.Identity,
                out FixedPointAnchor anchor, out Fixed64 depth, out bool clamped));
            Assert.False(clamped); Assert.Equal((Fixed64)4, depth);
            Assert.True(anchor.TryGetPoint(out Vector3d exit));
            Assert.Equal(new Vector3d(3, 4 * orientation, 0), exit);
        }
    }

    [Theory]
    [InlineData(-1)]
    [InlineData(1)]
    public void ZeroRadiusTransverseCapsule_ShouldReplayItsClosedEndpointEvents(int orientation)
    {
        var triangle = new FixedTriangle(new Vector3d(-8, 0, -8), new Vector3d(8, 0, -8), new Vector3d(0, 0, 8));
        FixedQuaternion rotation = new FixedQuaternion(Fixed64.Zero, Fixed64.Zero, Fixed64.One, Fixed64.One).Normalized;
        var finite = new CapsuleFiniteSectionFrame(Vector3d.Zero, FixedQuaternion.Identity, Vector3d.Zero,
            rotation, Fixed64.Two, Fixed64.Zero);
        var frame = new CapsulePlaneRayFrame(triangle, finite);
        int transverse = 0;
        Assert.True(CapsulePlaneRayEvents.VisitEvents(CapsulePlaneRayEventSource.Plane, frame, orientation,
            (in CapsulePlaneRayEventSource source, in CapsulePlaneRayFrame current,
                CapsulePlaneRayEvent descriptor, int direction, scoped in CapsulePlaneRaySelection candidate) =>
            {
                Assert.True(candidate.TryGetAuthoredPoint(current, Vector3d.Zero, FixedQuaternion.Identity,
                    Vector3d.Zero, rotation, out Vector3d point));
                Assert.Equal(Fixed64.Zero, point.Y); Assert.Equal(Fixed64.Zero, point.Z);
                Assert.InRange(point.X.m_rawValue, -Fixed64.One.m_rawValue, Fixed64.One.m_rawValue);
                Span<ulong> values = stackalloc ulong[CapsulePlaneRaySelection.StorageWords];
                Span<int> signs = stackalloc int[CapsulePlaneRaySelection.SignCount];
                Span<sbyte> valueSigns = stackalloc sbyte[CapsulePlaneRaySelection.ValueSignCount];
                var replay = new CapsulePlaneRaySelection(values, signs, valueSigns);
                Assert.True(CapsulePlaneRayEvents.Reconstruct(source, current, descriptor, direction, ref replay));
                Assert.Equal(0, replay.ComparePoint(candidate, current));
                Assert.True(replay.TryGetExitAnchor(current, direction, Vector3d.Zero, rotation,
                    out FixedPointAnchor anchor, out Fixed64 depth, out bool clamped));
                Assert.False(clamped); Assert.Equal(Fixed64.Zero, depth);
                Assert.True(anchor.TryGetPoint(out Vector3d exit)); Assert.Equal(point, exit);
                if (descriptor.Kind == CapsulePlaneRayEventKind.Sagittal && descriptor.Feature is 4 or 5) transverse++;
            }));
        // Both transverse cap representatives collapse to each finite core
        // endpoint. They remain valid closed contacts with zero normal depth.
        Assert.Equal(4, transverse);
    }

    private static bool AccumulateInvalidPoint(int orientation, int denominator, bool degenerate)
    {
        var triangle = degenerate ? new FixedTriangle(Vector3d.Zero, Vector3d.Zero, Vector3d.Zero)
            : new FixedTriangle(new Vector3d(-8, 0, -8), new Vector3d(8, 0, -8), new Vector3d(0, 0, 8));
        var finite = new CapsuleFiniteSectionFrame(Vector3d.Zero, FixedQuaternion.Identity, Vector3d.Zero,
            FixedQuaternion.Identity, Fixed64.Two, Fixed64.One);
        var frame = new CapsulePlaneRayFrame(triangle, finite);
        const int words = 9;
        Span<ulong> coefficients = stackalloc ulong[8 * words]; Span<int> coefficientSigns = stackalloc int[8];
        ContactQuadratic x = ContactQuadratic.At(coefficients, coefficientSigns, 0, words);
        ContactQuadratic y = ContactQuadratic.At(coefficients, coefficientSigns, 1, words);
        ContactQuadratic z = ContactQuadratic.At(coefficients, coefficientSigns, 2, words);
        ContactQuadratic d = ContactQuadratic.At(coefficients, coefficientSigns, 3, words);
        x.Clear(); y.Clear(); z.Clear(); d.Set(Signed576.ExtendValue(Signed320.ExtendValue(Signed192.Signed(denominator))));
        Span<ulong> values = stackalloc ulong[CapsulePlaneRaySelection.StorageWords];
        Span<int> signs = stackalloc int[CapsulePlaneRaySelection.SignCount];
        Span<sbyte> valueSigns = stackalloc sbyte[CapsulePlaneRaySelection.ValueSignCount];
        var selection = new CapsulePlaneRaySelection(values, signs, valueSigns);
        return CapsulePlaneRayEvents.AccumulateRationalPoint(frame, x, y, z, d, orientation, ref selection);
    }

    private static void CreateFixture(int fixture, out FixedTriangle triangle, out FixedSegment edge,
        out Vector3d center, out FixedQuaternion rotation, out Fixed64 length)
    {
        triangle = new FixedTriangle(new Vector3d(-8, 0, -8), new Vector3d(8, 0, -8), new Vector3d(0, 0, 8));
        edge = new FixedSegment(new Vector3d((Fixed64)(-2), Fixed64.Zero, Fixed64.Quarter),
            new Vector3d((Fixed64)2, Fixed64.Zero, Fixed64.Quarter));
        center = Vector3d.Zero; rotation = FixedQuaternion.Identity; length = Fixed64.Two;
        switch (fixture)
        {
            case 0: center = Vector3d.Up * Fixed64.Quarter; break;
            case 1:
            case 2:
                center = Vector3d.Up * ((Fixed64)3 / 2) * (fixture == 1 ? -1 : 1);
                rotation = FixedQuaternion.FromAxisAngle(Vector3d.Forward, Fixed64.PiOver4); break;
            case 3:
            case 4:
                triangle = new FixedTriangle(new Vector3d(0, -8, -8), new Vector3d(0, 8, -8), new Vector3d(0, 0, 8));
                edge = new FixedSegment(new Vector3d(Fixed64.Zero, (Fixed64)(-2), Fixed64.Quarter),
                    new Vector3d(Fixed64.Zero, (Fixed64)2, Fixed64.Quarter));
                center = Vector3d.Right * Fixed64.Quarter;
                if (fixture == 4) { length = Fixed64.Zero; rotation = FixedQuaternion.FromAxisAngle(Vector3d.Right, Fixed64.PiOver4); }
                break;
            case 5:
                center = new Vector3d(Fixed64.Quarter, -Fixed64.Quarter, Fixed64.Quarter);
                rotation = FixedQuaternion.LookRotation(new Vector3d(1, 0, 1).Normalized,
                    new Vector3d(1, 1, -1).Normalized); break;
            case 6: rotation = FixedQuaternion.FromAxisAngle(Vector3d.Forward, Fixed64.Pi); break;
            case 7:
                triangle = new FixedTriangle(-Vector3d.Right * Fixed64.Quarter, Vector3d.Right * Fixed64.Quarter,
                    Vector3d.Forward * Fixed64.Quarter);
                edge = triangle.GetEdge(0); break;
            case 8:
                triangle = new FixedTriangle(new Vector3d(-4, 3, -4), new Vector3d(4, -3, -4), new Vector3d(0, 0, 4));
                edge = new FixedSegment(new Vector3d(-Fixed64.Half, (Fixed64)3 / 8, (Fixed64)31 / 32),
                    new Vector3d(-Fixed64.One, (Fixed64)3 / 4, (Fixed64)11 / 32)); break;
            case 9:
                Vector3d start = new(-Fixed64.FromFraction(3, 4), -(Fixed64)3 / 2, Fixed64.Zero);
                triangle = new FixedTriangle(start + Vector3d.Backward * 2, start + Vector3d.Forward * 2,
                    start + new Vector3d((Fixed64)3, -(Fixed64)5 / 4, Fixed64.Zero));
                edge = new FixedSegment(start + Vector3d.Backward, start + Vector3d.Forward); break;
        }
    }
}
