using System;
using FixedMathSharp.Geometry;
using Xunit;

namespace FixedMathSharp.Tests.Bounds;

public sealed class FixedTriangleCapsulePlaneRayTests
{
    [Theory]
    [InlineData(1)]
    [InlineData(-1)]
    public void ObliqueLongBody_ProducesTheWholeSectionMaximum(int orientation)
    {
        FixedQuaternion rotation = FixedQuaternion.LookRotation(new Vector3d(1, 0, 1).Normalized,
            new Vector3d(1, 1, -1).Normalized);
        var triangle = new FixedTriangle(new Vector3d(-8, 0, -8), new Vector3d(8, 0, -8), new Vector3d(0, 0, 8));
        var finite = new CapsuleFiniteSectionFrame(Vector3d.Zero, FixedQuaternion.Identity,
            Vector3d.Zero, rotation, (Fixed64)8, Fixed64.Half);
        var frame = new CapsulePlaneRayFrame(triangle, finite);
        Span<ulong> values = stackalloc ulong[CapsulePlaneRaySelection.StorageWords];
        Span<int> signs = stackalloc int[CapsulePlaneRaySelection.SignCount];
        Span<sbyte> valueSigns = stackalloc sbyte[CapsulePlaneRaySelection.ValueSignCount];
        var selection = new CapsulePlaneRaySelection(values, signs, valueSigns);
        Assert.True(CapsulePlaneRayEvents.Accumulate(CapsulePlaneRayEventSource.Plane, frame, orientation, ref selection));
        Assert.True(selection.TryGetDepth(frame, out Fixed64 depth));
        Fixed64 axisY = rotation.Rotate(Vector3d.Up).Y;
        Fixed64 expected = Fixed64.One / FixedMath.Sqrt(Fixed64.One - axisY * axisY);
        Assert.InRange(FixedMath.Abs(depth - expected).m_rawValue, 0L, 64L);
        Assert.True(selection.TryGetExitAnchor(frame, orientation, Vector3d.Zero, rotation,
            out _, out Fixed64 exitDepth, out bool clamped));
        Assert.Equal(depth, exitDepth);
        Assert.False(clamped);
    }

    [Theory]
    [InlineData(1)]
    [InlineData(-1)]
    public void ObliqueBodySection_ProducesNoncollinearFiniteCoverage(int orientation)
    {
        FixedQuaternion rotation = FixedQuaternion.FromAxisAngle(Vector3d.Forward, Fixed64.PiOver4);
        var triangle = new FixedTriangle(new Vector3d(-8, 0, -8), new Vector3d(8, 0, -8), new Vector3d(0, 0, 8));
        var finite = new CapsuleFiniteSectionFrame(Vector3d.Zero, FixedQuaternion.Identity,
            Vector3d.Zero, rotation, (Fixed64)8, Fixed64.Half);
        var frame = new CapsulePlaneRayFrame(triangle, finite);
        Vector3d[] transverse = new Vector3d[2];
        Vector3d[] sagittal = new Vector3d[2];
        int transverseCount = 0, sagittalCount = 0;
        CapsulePlaneRayEvents.VisitEvents(CapsulePlaneRayEventSource.Plane, frame, orientation,
            (in CapsulePlaneRayEventSource source, in CapsulePlaneRayFrame current, CapsulePlaneRayEvent descriptor,
                int direction, scoped in CapsulePlaneRaySelection candidate) =>
            {
                if (descriptor.Kind != CapsulePlaneRayEventKind.Sagittal ||
                    (descriptor.Feature != 0 && descriptor.Feature != 3)) return;
                Assert.True(candidate.TryGetAuthoredPoint(current, Vector3d.Zero, FixedQuaternion.Identity,
                    Vector3d.Zero, rotation, out Vector3d point));
                Assert.Equal(Fixed64.Zero, point.Y);
                if (descriptor.Feature == 0) { sagittal[sagittalCount++] = point; return; }
                Assert.True(candidate.TryGetDepth(current, out Fixed64 depth));
                Assert.Equal(Fixed64.Zero, depth);
                transverse[transverseCount++] = point;
            });
        Assert.Equal(2, transverseCount);
        Assert.Equal(2, sagittalCount);
        Assert.Contains(Vector3d.Forward * Fixed64.Half, transverse);
        Assert.Contains(Vector3d.Backward * Fixed64.Half, transverse);
        // Both ray maxima lie on the sagittal diameter; the transverse diameter
        // gives the finite sample pool area rather than a collinear pair.
        Assert.True(Vector3d.Cross(sagittal[1] - sagittal[0], transverse[1] - transverse[0]).MagnitudeSquared > Fixed64.Zero);
    }

    [Theory]
    [InlineData(1)]
    [InlineData(-1)]
    public void ObliqueSectionWhollyInsideOneCap_ProducesNoncollinearFiniteCoverage(int orientation)
    {
        FixedQuaternion rotation = FixedQuaternion.FromAxisAngle(Vector3d.Forward, Fixed64.PiOver4);
        Vector3d center = new(Fixed64.Zero, -(Fixed64)3 / 2, Fixed64.Zero);
        var triangle = new FixedTriangle(new Vector3d(-8, 0, -8), new Vector3d(8, 0, -8), new Vector3d(0, 0, 8));
        var finite = new CapsuleFiniteSectionFrame(Vector3d.Zero, FixedQuaternion.Identity,
            center, rotation, Fixed64.Two, Fixed64.One);
        var frame = new CapsulePlaneRayFrame(triangle, finite);
        Vector3d[] rim = new Vector3d[4];
        Vector3d capCenter = center + rotation.Rotate(Vector3d.Up);
        int count = 0;
        CapsulePlaneRayEvents.VisitEvents(CapsulePlaneRayEventSource.Plane, frame, orientation,
            (in CapsulePlaneRayEventSource source, in CapsulePlaneRayFrame current, CapsulePlaneRayEvent descriptor,
                int direction, scoped in CapsulePlaneRaySelection candidate) =>
            {
                if (descriptor.Kind != CapsulePlaneRayEventKind.Sagittal ||
                    (descriptor.Feature != 2 && descriptor.Feature != 5)) return;
                Assert.True(candidate.TryGetAuthoredPoint(current, Vector3d.Zero, FixedQuaternion.Identity,
                    center, rotation, out Vector3d point));
                Assert.Equal(Fixed64.Zero, point.Y);
                Assert.InRange(FixedMath.Abs((point - capCenter).MagnitudeSquared - Fixed64.One).m_rawValue, 0L, 64L);
                Assert.True(candidate.TryGetDepth(current, out Fixed64 depth));
                if (orientation > 0) Assert.Equal(Fixed64.Zero, depth);
                rim[count++] = point;
            });
        Assert.Equal(4, count);
        // This section has no body or seam crossing. Both independent rim
        // diameters must come from its cap rather than mesh triangulation.
        Assert.True(Vector3d.Cross(rim[1] - rim[0], rim[3] - rim[2]).MagnitudeSquared > Fixed64.Zero);
    }

    [Theory]
    [InlineData(1)]
    [InlineData(-1)]
    public void AxialPlaneInsideBody_ProducesFourBoundaryFamilyRepresentatives(int orientation)
    {
        var triangle = new FixedTriangle(new Vector3d(-8, 0, -8), new Vector3d(8, 0, -8), new Vector3d(0, 0, 8));
        var frame = new CapsulePlaneRayFrame(triangle, Frame(Vector3d.Zero, Fixed64.Two, Fixed64.One));
        int rimCount = 0;
        CapsulePlaneRayEvents.VisitEvents(CapsulePlaneRayEventSource.Plane, frame, orientation,
            (in CapsulePlaneRayEventSource source, in CapsulePlaneRayFrame current, CapsulePlaneRayEvent descriptor,
                int direction, scoped in CapsulePlaneRaySelection candidate) =>
            {
                if (descriptor.Kind != CapsulePlaneRayEventKind.Circle || descriptor.Feature != 0) return;
                Assert.True(candidate.TryGetAuthoredPoint(current, Vector3d.Zero, FixedQuaternion.Identity,
                    Vector3d.Zero, FixedQuaternion.Identity, out Vector3d point));
                Assert.Equal(Fixed64.Zero, point.Y);
                Assert.Equal(Fixed64.One, Vector3d.Dot(point, point));
                Assert.True(candidate.TryGetDepth(current, out Fixed64 depth));
                Assert.Equal(Fixed64.One, depth);
                rimCount++;
            });
        Assert.Equal(4, rimCount);
    }

    [Theory]
    [InlineData(false)]
    [InlineData(true)]
    public void CircularFamily_ShouldIgnoreCapsuleYawAndZeroCoreAxis(bool sphere)
    {
        var triangle = new FixedTriangle(new Vector3d(-8, 0, -8), new Vector3d(8, 0, -8), new Vector3d(0, 0, 8));
        FixedQuaternion changed = FixedQuaternion.FromAxisAngle(sphere ? Vector3d.Right : Vector3d.Up, Fixed64.PiOver4);
        Vector3d[] original = GetCircularPoints(triangle, sphere, FixedQuaternion.Identity);
        Assert.Equal(new[] { Vector3d.Forward, Vector3d.Backward, Vector3d.Right, Vector3d.Left }, original);
        Assert.Equal(original, GetCircularPoints(triangle, sphere, changed));
    }

    [Theory]
    [InlineData(1)]
    [InlineData(-1)]
    public void OddCoreAndMaximumRadius_RetainSplitAnchorAndExplicitDepthClamp(int orientation)
    {
        var triangle = new FixedTriangle(new Vector3d(-8, 0, -8), new Vector3d(8, 0, -8), new Vector3d(0, 0, 8));
        var frame = new CapsulePlaneRayFrame(triangle, Frame(Vector3d.Zero, Fixed64.FromRaw(1), Fixed64.MaxValue));
        Span<ulong> values = stackalloc ulong[CapsulePlaneRaySelection.StorageWords];
        Span<int> signs = stackalloc int[CapsulePlaneRaySelection.SignCount];
        Span<sbyte> valueSigns = stackalloc sbyte[CapsulePlaneRaySelection.ValueSignCount];
        var selection = new CapsulePlaneRaySelection(values, signs, valueSigns);
        Assert.True(AccumulatePoint(frame, Vector3d.Zero, orientation, ref selection));
        Assert.True(selection.TryGetDepth(frame, out Fixed64 depth, out bool clamped));
        Assert.True(clamped);
        Assert.Equal(Fixed64.MaxValue, depth);
        Assert.True(selection.TryGetExitAnchor(frame, orientation, Vector3d.Zero, FixedQuaternion.Identity,
            out FixedPointAnchor anchor, out Fixed64 exitDepth, out bool exitClamped));
        Assert.Equal(depth, exitDepth);
        Assert.True(exitClamped);
        Assert.Equal(Vector3d.Zero, anchor.LocalPoint);
        Assert.Equal(new Vector3d(Fixed64.Zero, Fixed64.MaxValue * orientation, Fixed64.Zero), anchor.LocalDisplacement);
        Assert.Equal(orientation < 0, anchor.TryGetPoint(out _));
    }

    [Fact]
    public void FiniteEdgeSectionEndpoint_CanSelectAGenuineQuarticMaximum()
    {
        var triangle = new FixedTriangle(new Vector3d(-4, 3, -4), new Vector3d(4, -3, -4), new Vector3d(0, 0, 4));
        var frame = new CapsulePlaneRayFrame(triangle, Frame(Vector3d.Zero, Fixed64.Two, Fixed64.One));
        var source = new CapsulePlaneRayEventSource(new FixedSegment(
            new Vector3d(-Fixed64.Half, (Fixed64)3 / 8, (Fixed64)31 / 32),
            new Vector3d(-Fixed64.One, (Fixed64)3 / 4, (Fixed64)11 / 32)));
        Span<ulong> values = stackalloc ulong[CapsulePlaneRaySelection.StorageWords];
        Span<int> signs = stackalloc int[CapsulePlaneRaySelection.SignCount];
        Span<sbyte> valueSigns = stackalloc sbyte[CapsulePlaneRaySelection.ValueSignCount];
        var selection = new CapsulePlaneRaySelection(values, signs, valueSigns);
        Assert.True(CapsulePlaneRayEvents.Accumulate(source, frame, 1, ref selection));
        Assert.True(selection.Metadata.UsesValueRoot);
        Assert.Equal(CapsulePlaneRayEventKind.Section, selection.MaximumEvent.Kind);
        Assert.True(selection.TryGetDepth(frame, out Fixed64 depth));
        // x=(1275+20*sqrt(23))/1312, t=4/5+sqrt(16/25-(3x/5-1)^2).
        // The outer radical is not in Q(sqrt(23)); this finite endpoint wins.
        Assert.Equal(6475490689L, depth.m_rawValue);
        Assert.True(selection.TryGetAuthoredPoint(frame, Vector3d.Zero, FixedQuaternion.Identity,
            Vector3d.Zero, FixedQuaternion.Identity, out Vector3d point));
        Assert.Equal(new Vector3d(Fixed64.FromRaw(-3590269569), Fixed64.FromRaw(2692702177),
            Fixed64.FromRaw(2357267167)), point);
        Assert.True(selection.TryGetExitAnchor(frame, 1, Vector3d.Zero, FixedQuaternion.Identity,
            out FixedPointAnchor anchor, out Fixed64 exitDepth, out bool clamped));
        Assert.Equal(depth, exitDepth);
        Assert.False(clamped);
        Assert.True(anchor.TryGetPoint(out Vector3d exit));
        Vector3d expectedExit = point + frame.GetWorldNormal(FixedQuaternion.Identity) * depth;
        for (int axis = 0; axis < 3; axis++)
            Assert.InRange(FixedMath.Abs(exit[axis] - expectedExit[axis]).m_rawValue, 0L, 8L);
    }

    [Fact]
    public void SectionEndpoint_WithIndependentPointAndDepthRadicals_KeepsBothAnchors()
    {
        Vector3d basePoint = new(-Fixed64.FromFraction(3, 4), -(Fixed64)3 / 2, Fixed64.Zero);
        var triangle = new FixedTriangle(basePoint + Vector3d.Backward * 2, basePoint + Vector3d.Forward * 2,
            basePoint + new Vector3d((Fixed64)3, -(Fixed64)5 / 4, Fixed64.Zero));
        var frame = new CapsulePlaneRayFrame(triangle, Frame(Vector3d.Zero, Fixed64.Two, Fixed64.One));
        var source = new CapsulePlaneRayEventSource(new FixedSegment(basePoint + Vector3d.Backward, basePoint + Vector3d.Forward));
        Span<ulong> values = stackalloc ulong[CapsulePlaneRaySelection.StorageWords];
        Span<int> signs = stackalloc int[CapsulePlaneRaySelection.SignCount];
        Span<sbyte> valueSigns = stackalloc sbyte[CapsulePlaneRaySelection.ValueSignCount];
        var selection = new CapsulePlaneRaySelection(values, signs, valueSigns);
        Assert.True(CapsulePlaneRayEvents.Reconstruct(source, frame,
            new CapsulePlaneRayEvent(CapsulePlaneRayEventKind.Section, 1, 1), 1, ref selection));
        Assert.True(selection.TryGetDepth(frame, out Fixed64 depth));
        Assert.Equal(14845103363L, depth.m_rawValue); // (135+sqrt(2001))/52.
        Assert.True(selection.TryGetAuthoredPoint(frame, Vector3d.Zero, FixedQuaternion.Identity,
            Vector3d.Zero, FixedQuaternion.Identity, out Vector3d point));
        Assert.True(selection.TryGetExitAnchor(frame, 1, Vector3d.Zero, FixedQuaternion.Identity,
            out FixedPointAnchor anchor, out Fixed64 exitDepth, out bool clamped));
        Assert.Equal(depth, exitDepth);
        Assert.False(clamped);
        Assert.Equal(Fixed64.FromRaw(1859775393), point.Z); // sqrt(3)/4.
        Assert.Equal(point.Z, anchor.LocalDisplacement.Z);
    }

    [Fact]
    public void OppositeStationaryRayReconstruction_PreservesItsOriginalBasePoint()
    {
        var triangle = new FixedTriangle(new Vector3d(-4, 3, -4), new Vector3d(4, -3, -4), new Vector3d(0, 0, 4));
        var frame = new CapsulePlaneRayFrame(triangle, Frame(Vector3d.Zero, (Fixed64)8, Fixed64.One));
        Vector3d middle = new(Fixed64.Quarter, -(Fixed64)3 / 16, Fixed64.Zero);
        var source = new CapsulePlaneRayEventSource(new FixedSegment(middle + Vector3d.Backward, middle + Vector3d.Forward));
        CapsulePlaneRayEvent stationary = default;
        bool found = false;
        CapsulePlaneRayEvents.VisitEvents(source, frame, 1,
            (in CapsulePlaneRayEventSource _, in CapsulePlaneRayFrame current, CapsulePlaneRayEvent descriptor,
                int direction, scoped in CapsulePlaneRaySelection candidate) =>
            {
                if (descriptor.Kind != CapsulePlaneRayEventKind.Stationary || descriptor.Feature != 0 || descriptor.Orientation != 1) return;
                stationary = descriptor; found = true;
                Assert.True(candidate.TryGetAuthoredPoint(current, Vector3d.Zero, FixedQuaternion.Identity,
                    Vector3d.Zero, FixedQuaternion.Identity, out Vector3d point));
                Assert.Equal(middle, point);
            });
        Assert.True(found);
        Span<ulong> values = stackalloc ulong[CapsulePlaneRaySelection.StorageWords];
        Span<int> signs = stackalloc int[CapsulePlaneRaySelection.SignCount];
        Span<sbyte> valueSigns = stackalloc sbyte[CapsulePlaneRaySelection.ValueSignCount];
        var selection = new CapsulePlaneRaySelection(values, signs, valueSigns);
        Assert.True(CapsulePlaneRayEvents.Reconstruct(source, frame, stationary, -1, ref selection));
        Assert.True(selection.TryGetAuthoredPoint(frame, Vector3d.Zero, FixedQuaternion.Identity,
            Vector3d.Zero, FixedQuaternion.Identity, out Vector3d restored));
        Assert.Equal(middle, restored);
        Assert.True(selection.TryGetDepth(frame, out Fixed64 depth));
        Assert.InRange(FixedMath.Abs(depth - (Fixed64)25 / 12).m_rawValue, 0L, 1L);
    }

    private static Vector3d[] GetCircularPoints(FixedTriangle triangle, bool sphere, FixedQuaternion rotation)
    {
        var finite = new CapsuleFiniteSectionFrame(Vector3d.Zero, FixedQuaternion.Identity, Vector3d.Zero,
            rotation, sphere ? Fixed64.Zero : Fixed64.Two, Fixed64.One);
        var frame = new CapsulePlaneRayFrame(triangle, finite);
        var points = new Vector3d[4];
        CapsulePlaneRayEvents.VisitEvents(CapsulePlaneRayEventSource.Plane, frame, 1,
            (in CapsulePlaneRayEventSource _, in CapsulePlaneRayFrame current, CapsulePlaneRayEvent descriptor,
                int direction, scoped in CapsulePlaneRaySelection candidate) =>
            {
                if (descriptor.Kind != CapsulePlaneRayEventKind.Circle) return;
                Assert.True(candidate.TryGetAuthoredPoint(current, Vector3d.Zero, FixedQuaternion.Identity,
                    Vector3d.Zero, rotation, out points[descriptor.Branch]));
            });
        return points;
    }

    [Theory]
    [InlineData(1, 3)]
    [InlineData(-1, -5)]
    public void RationalCrossingCoreRay_ShouldReachTheActualEndpointCap(int orientation, int exitQuarter)
    {
        Vector3d center = new(Fixed64.Zero, -(Fixed64)1 / 4, Fixed64.Zero);
        var triangle = new FixedTriangle(new Vector3d(-8, 0, -8), new Vector3d(8, 0, -8), new Vector3d(0, 0, 8));
        var frame = new CapsulePlaneRayFrame(triangle, Frame(center, Fixed64.One, (Fixed64)1 / 2));
        Span<ulong> values = stackalloc ulong[CapsulePlaneRaySelection.StorageWords];
        Span<int> signs = stackalloc int[CapsulePlaneRaySelection.SignCount];
        Span<sbyte> valueSigns = stackalloc sbyte[CapsulePlaneRaySelection.ValueSignCount];
        var selection = new CapsulePlaneRaySelection(values, signs, valueSigns);
        Assert.True(AccumulatePoint(frame, Vector3d.Zero, orientation, ref selection));
        Assert.True(selection.TryGetAuthoredPoint(frame, Vector3d.Zero, FixedQuaternion.Identity,
            center, FixedQuaternion.Identity, out Vector3d lower));
        Assert.True(selection.TryGetExitAnchor(frame, orientation, center, FixedQuaternion.Identity,
            out FixedPointAnchor anchor, out Fixed64 depth, out bool clamped));
        Assert.True(anchor.TryGetPoint(out Vector3d upper));
        Assert.Equal(Vector3d.Zero, lower);
        Assert.Equal(new Vector3d(Fixed64.Zero, (Fixed64)exitQuarter / 4, Fixed64.Zero), upper);
        Assert.Equal(FixedMath.Abs((Fixed64)exitQuarter / 4), depth);
        Assert.False(clamped);
        Assert.Equal(Vector3d.Up, frame.GetWorldNormal(FixedQuaternion.Identity));
    }

    [Theory]
    [InlineData(1, 5)]
    [InlineData(-1, -3)]
    public void ZeroCoreRay_ShouldRetainSphereBoundaryDepth(int orientation, int exitQuarter)
    {
        Vector3d center = new(Fixed64.Zero, (Fixed64)1 / 4, Fixed64.Zero);
        var triangle = new FixedTriangle(new Vector3d(-8, 0, -8), new Vector3d(8, 0, -8), new Vector3d(0, 0, 8));
        var frame = new CapsulePlaneRayFrame(triangle, Frame(center, Fixed64.Zero, Fixed64.One));
        Span<ulong> values = stackalloc ulong[CapsulePlaneRaySelection.StorageWords];
        Span<int> signs = stackalloc int[CapsulePlaneRaySelection.SignCount];
        Span<sbyte> valueSigns = stackalloc sbyte[CapsulePlaneRaySelection.ValueSignCount];
        var selection = new CapsulePlaneRaySelection(values, signs, valueSigns);
        Assert.True(AccumulatePoint(frame, Vector3d.Zero, orientation, ref selection));
        Assert.True(selection.TryGetAuthoredPoint(frame, Vector3d.Zero, FixedQuaternion.Identity,
            center, FixedQuaternion.Identity, out Vector3d lower));
        Assert.True(selection.TryGetExitAnchor(frame, orientation, center, FixedQuaternion.Identity,
            out FixedPointAnchor anchor, out Fixed64 depth, out bool clamped));
        Assert.True(anchor.TryGetPoint(out Vector3d upper));
        Assert.Equal(Vector3d.Zero, lower);
        Assert.Equal(new Vector3d(Fixed64.Zero, (Fixed64)exitQuarter / 4, Fixed64.Zero), upper);
        Assert.Equal(FixedMath.Abs((Fixed64)exitQuarter / 4), depth);
        Assert.False(clamped);
    }

    [Theory]
    [InlineData(false)]
    [InlineData(true)]
    public void SectionWhollyInsideTriangle_ShouldRemainAdmitted(bool reverse)
    {
        var triangle = new FixedTriangle(new Vector3d(-8, 0, -8), new Vector3d(8, 0, -8), new Vector3d(0, 0, 8));
        if (reverse) triangle = new FixedTriangle(triangle.A, triangle.C, triangle.B);
        var finite = Frame(Vector3d.Zero, Fixed64.One, (Fixed64)1 / 2);
        Assert.False(finite.ContainsPoint(triangle.A));
        Assert.False(finite.ContainsPoint(triangle.B));
        Assert.False(finite.ContainsPoint(triangle.C));
        for (int edge = 0; edge < 3; edge++) Assert.False(finite.IntersectsSegment(triangle.GetEdge(edge)));
        Assert.True(CapsulePlaneRayEvents.IntersectsTriangle(finite, triangle));
    }

    [Fact]
    public void ParallelCoreWhollyProjectedInsideTriangle_ShouldRemainAdmitted()
    {
        var triangle = new FixedTriangle(new Vector3d(0, -8, -8), new Vector3d(0, 8, -8), new Vector3d(0, 0, 8));
        Assert.True(CapsulePlaneRayEvents.IntersectsTriangle(Frame(new Vector3d((Fixed64)1 / 4, Fixed64.Zero, Fixed64.Zero), Fixed64.One, (Fixed64)1 / 2), triangle));
        Assert.False(CapsulePlaneRayEvents.IntersectsTriangle(Frame(new Vector3d(1, 0, 0), Fixed64.One, (Fixed64)1 / 2), triangle));
    }

    [Theory]
    [InlineData(-1, true)]
    [InlineData(0, true)]
    [InlineData(1, false)]
    public void EndpointProjection_ShouldUseExactOneRawTouchAdmission(long offset, bool expected)
    {
        var triangle = new FixedTriangle(new Vector3d(-8, 0, -8), new Vector3d(8, 0, -8), new Vector3d(0, 0, 8));
        Fixed64 centerHeight = (Fixed64)3 / 2 + Fixed64.FromRaw(offset);
        Assert.Equal(expected, CapsulePlaneRayEvents.IntersectsTriangle(Frame(new Vector3d(Fixed64.Zero, centerHeight, Fixed64.Zero), Fixed64.One, Fixed64.One), triangle));
    }

    [Fact]
    public void FiniteTriangleOutsideAdmittedPlaneSection_ShouldRejectProjectionCertificate()
    {
        var triangle = new FixedTriangle(new Vector3d(4, 0, 4), new Vector3d(5, 0, 4), new Vector3d(4, 0, 5));
        Assert.False(CapsulePlaneRayEvents.IntersectsTriangle(Frame(Vector3d.Zero, Fixed64.One, Fixed64.One), triangle));
    }

    [Fact]
    public void TriangleEdgeAndVertex_ShouldSupplyClosedAdmissionBeforeInteriorCertificate()
    {
        var finite = Frame(Vector3d.Zero, Fixed64.One, Fixed64.One);
        var vertex = new FixedTriangle(Vector3d.Zero, new Vector3d(4, 0, 0), new Vector3d(0, 0, 4));
        var edge = new FixedTriangle(new Vector3d(-4, 0, 0), new Vector3d(4, 0, 0), new Vector3d(0, 0, 4));
        Assert.True(CapsulePlaneRayEvents.IntersectsTriangle(finite, vertex));
        Assert.True(CapsulePlaneRayEvents.IntersectsTriangle(finite, edge));
        var outsidePoint = new FixedTriangle(new Vector3d(4, 0, 4), new Vector3d(4, 0, 4), new Vector3d(4, 0, 4));
        Assert.False(CapsulePlaneRayEvents.IntersectsTriangle(finite, outsidePoint));
    }

    [Fact]
    public void ZeroRadiusOddCore_ShouldNotRoundItsHalfLengthIntoFalseAdmission()
    {
        var triangle = new FixedTriangle(new Vector3d(-8, 0, -8), new Vector3d(8, 0, -8), new Vector3d(0, 0, 8));
        Assert.True(CapsulePlaneRayEvents.IntersectsTriangle(Frame(Vector3d.Zero, Fixed64.FromRaw(1), Fixed64.Zero), triangle));
        Assert.False(CapsulePlaneRayEvents.IntersectsTriangle(Frame(new Vector3d(Fixed64.Zero, Fixed64.FromRaw(1), Fixed64.Zero), Fixed64.FromRaw(1), Fixed64.Zero), triangle));
    }

    [Theory]
    [InlineData(false)]
    [InlineData(true)]
    public void TriangleSingleAdmittedVertexAndCapInterior_ShouldSupplyClosedAdmission(bool upperCap)
    {
        var vertex = upperCap
            ? new FixedTriangle(new Vector3d(4, 0, 0), Vector3d.Zero, new Vector3d(0, 0, 4))
            : new FixedTriangle(new Vector3d(4, 0, 0), new Vector3d(0, 0, 4), Vector3d.Zero);
        Assert.True(CapsulePlaneRayEvents.IntersectsTriangle(Frame(Vector3d.Zero, Fixed64.Two, Fixed64.One), vertex));
        var enclosing = new FixedTriangle(new Vector3d(-8, 0, -8), new Vector3d(8, 0, -8), new Vector3d(0, 0, 8));
        Vector3d center = Vector3d.Up * ((Fixed64)3 / 2) * (upperCap ? -1 : 1);
        Assert.True(CapsulePlaneRayEvents.IntersectsTriangle(Frame(center, Fixed64.Two, Fixed64.One), enclosing));
    }

    private static CapsuleFiniteSectionFrame Frame(Vector3d center, Fixed64 length, Fixed64 radius) =>
        new(Vector3d.Zero, FixedQuaternion.Identity, center, FixedQuaternion.Identity, length, radius);

    private static bool AccumulatePoint(in CapsulePlaneRayFrame frame, Vector3d point, int orientation,
        ref CapsulePlaneRaySelection selection)
    {
        const int words = CapsulePlaneRaySelection.FieldWords;
        Span<ulong> values = stackalloc ulong[8 * words]; Span<int> signs = stackalloc int[8];
        ContactQuadratic x = ContactQuadratic.At(values, signs, 0, words);
        ContactQuadratic y = ContactQuadratic.At(values, signs, 1, words);
        ContactQuadratic z = ContactQuadratic.At(values, signs, 2, words);
        ContactQuadratic d = ContactQuadratic.At(values, signs, 3, words);
        WideAxis3 transformed = frame.Finite.Transform(point);
        x.Set(Signed576.ExtendValue(transformed.X)); y.Set(Signed576.ExtendValue(transformed.Y)); z.Set(Signed576.ExtendValue(transformed.Z));
        d.Set(Signed576.ExtendValue(Signed320.ExtendValue(Signed192.Signed(1))));
        return CapsulePlaneRayEvents.AccumulateRationalPoint(frame, x, y, z, d, orientation, ref selection);
    }
}
