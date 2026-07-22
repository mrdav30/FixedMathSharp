using System;
using FixedMathSharp.Bounds;
using Xunit;

namespace FixedMathSharp.Tests.Bounds;

public sealed partial class FiniteAxisIntersectionTests
{
    [Fact]
    public void BoundedRayCollapsedCapsules_PreservePhysicalRawEntryDistance()
    {
        var ray2d = new FixedRay2d(Vector2d.Zero, Vector2d.Right);
        var ray3d = new FixedRay(Vector3d.Zero, Vector3d.Right);
        Fixed64 maxParameter = (Fixed64)1_000_000;
        Fixed64 radius = Fixed64.One;

        foreach (long centerOffsetRaw in new[] { 0L, 1L })
        {
            Fixed64 centerX = (Fixed64)101 + Fixed64.FromRaw(centerOffsetRaw);
            var axis2d = new FixedSegment2d(
                new Vector2d(centerX, Fixed64.Zero),
                new Vector2d(centerX, Fixed64.Zero));
            var axis3d = new FixedSegment(
                new Vector3d(centerX, Fixed64.Zero, Fixed64.Zero),
                new Vector3d(centerX, Fixed64.Zero, Fixed64.Zero));
            Fixed64 expectedEntry = (Fixed64)100 + Fixed64.FromRaw(centerOffsetRaw);

            Assert.True(ray2d.TryGetCapsuleIntersectionInterval(
                axis2d,
                radius,
                maxParameter,
                out Fixed64 entry2d,
                out Fixed64 exit2d));
            Assert.True(ray3d.TryGetCapsuleIntersectionInterval(
                axis3d,
                radius,
                maxParameter,
                out Fixed64 entry3d,
                out Fixed64 exit3d));
            Assert.True(ray2d.TryGetCapsuleIntersectionInterval(
                axis2d.Start,
                Vector2d.Forward,
                Fixed64.Zero,
                radius,
                maxParameter,
                out Fixed64 centeredEntry2d,
                out Fixed64 centeredExit2d));
            Assert.True(ray3d.TryGetCapsuleIntersectionInterval(
                axis3d.Start,
                Vector3d.Up,
                Fixed64.Zero,
                radius,
                maxParameter,
                out Fixed64 centeredEntry3d,
                out Fixed64 centeredExit3d));

            Assert.Equal(expectedEntry, entry2d);
            Assert.Equal(expectedEntry, entry3d);
            Assert.Equal(expectedEntry, centeredEntry2d);
            Assert.Equal(expectedEntry, centeredEntry3d);
            Assert.Equal((Fixed64)102 + Fixed64.FromRaw(centerOffsetRaw), exit2d);
            Assert.Equal(exit2d, exit3d);
            Assert.Equal(exit2d, centeredExit2d);
            Assert.Equal(exit2d, centeredExit3d);
        }
    }

    [Fact]
    public void BoundedRayCapsule_NonNormalizedDirectionReturnsRayParameter()
    {
        var ray = new FixedRay2d(Vector2d.Zero, Vector2d.Right * Fixed64.Two);
        Fixed64 centerX = (Fixed64)101 + Fixed64.FromRaw(1L);
        var axis = new FixedSegment2d(
            new Vector2d(centerX, Fixed64.Zero),
            new Vector2d(centerX, Fixed64.Zero));

        Assert.True(ray.TryGetCapsuleIntersectionInterval(
            axis,
            Fixed64.One,
            (Fixed64)1_000_000,
            out Fixed64 entry,
            out Fixed64 exit));

        Assert.Equal((Fixed64)50, entry);
        Assert.Equal((Fixed64)51, exit);
    }

    [Fact]
    public void BoundedRayCapsule_OrdinaryUnitParameterMatchesSegmentFamily()
    {
        var start = new Vector3d((Fixed64)(-4), Fixed64.Half, Fixed64.Zero);
        var end = new Vector3d((Fixed64)6, Fixed64.Half, Fixed64.Zero);
        var segment = new FixedSegment(start, end);
        var ray = new FixedRay(start, end - start);
        var axis = new FixedSegment(
            new Vector3d(Fixed64.Zero, -Fixed64.One, Fixed64.Zero),
            new Vector3d(Fixed64.Zero, Fixed64.One, Fixed64.Zero));

        Assert.True(segment.TryGetCapsuleIntersectionInterval(
            axis,
            Fixed64.One,
            Fixed64.Half,
            out Fixed64 segmentEntry,
            out Fixed64 segmentExit,
            out bool segmentStartContained,
            out bool segmentEndContainedStrict));
        Assert.True(ray.TryGetCapsuleIntersectionInterval(
            axis,
            Fixed64.One,
            Fixed64.Half,
            Fixed64.One,
            out Fixed64 rayEntry,
            out Fixed64 rayExit,
            out bool rayOriginContained,
            out bool rayMaximumContainedStrict));

        Assert.Equal(segmentEntry, rayEntry);
        Assert.Equal(segmentExit, rayExit);
        Assert.Equal(segmentStartContained, rayOriginContained);
        Assert.Equal(segmentEndContainedStrict, rayMaximumContainedStrict);
    }

    [Fact]
    public void BoundedRayCollapsedCapsule_ExtremeOriginsPreservePhysicalDistance()
    {
        var minimumRay = new FixedRay2d(
            new Vector2d(Fixed64.MinValue, Fixed64.Zero),
            Vector2d.Right);
        var maximumRay = new FixedRay2d(
            new Vector2d(Fixed64.MaxValue, Fixed64.Zero),
            Vector2d.Left);
        var minimumAxis = new FixedSegment2d(
            new Vector2d(Fixed64.MinValue + (Fixed64)101, Fixed64.Zero),
            new Vector2d(Fixed64.MinValue + (Fixed64)101, Fixed64.Zero));
        var maximumAxis = new FixedSegment2d(
            new Vector2d(Fixed64.MaxValue - (Fixed64)101, Fixed64.Zero),
            new Vector2d(Fixed64.MaxValue - (Fixed64)101, Fixed64.Zero));

        Assert.True(minimumRay.TryGetCapsuleIntersectionInterval(
            minimumAxis, Fixed64.One, (Fixed64)1_000_000, out Fixed64 minimumEntry, out _));
        Assert.True(maximumRay.TryGetCapsuleIntersectionInterval(
            maximumAxis, Fixed64.One, (Fixed64)1_000_000, out Fixed64 maximumEntry, out _));
        Assert.Equal((Fixed64)100, minimumEntry);
        Assert.Equal(minimumEntry, maximumEntry);
    }

    [Fact]
    public void BoundedRayCapsule_ClipsAtMaximumAndReportsExactContainment()
    {
        var ray = new FixedRay2d(Vector2d.Zero, Vector2d.Right);
        var axis = new FixedSegment2d(new Vector2d((Fixed64)2, Fixed64.Zero), new Vector2d((Fixed64)4, Fixed64.Zero));

        Assert.True(ray.TryGetCapsuleIntersectionInterval(
            axis,
            Fixed64.One,
            Fixed64.Zero,
            (Fixed64)3,
            out Fixed64 entry,
            out Fixed64 exit,
            out bool originContained,
            out bool maximumContainedStrict));

        Assert.Equal(Fixed64.One, entry);
        Assert.Equal((Fixed64)3, exit);
        Assert.False(originContained);
        Assert.True(maximumContainedStrict);
    }

    [Fact]
    public void BoundedRayCapsule_MaximumBeforeEntryRejectsAndZeroMaximumQueriesOrigin()
    {
        var ray = new FixedRay2d(Vector2d.Zero, Vector2d.Right);
        var axis = new FixedSegment2d(
            new Vector2d((Fixed64)2, Fixed64.Zero),
            new Vector2d((Fixed64)4, Fixed64.Zero));

        Assert.False(ray.TryGetCapsuleIntersectionInterval(
            axis, Fixed64.One, Fixed64.Half, out _, out _));

        var boundaryRay = new FixedRay2d(new Vector2d(Fixed64.One, Fixed64.Zero), Vector2d.Right);
        Assert.True(boundaryRay.TryGetCapsuleIntersectionInterval(
            axis,
            Fixed64.One,
            Fixed64.Zero,
            Fixed64.Zero,
            out Fixed64 entry,
            out Fixed64 exit,
            out bool originContained,
            out bool maximumContainedStrict));
        Assert.Equal(Fixed64.Zero, entry);
        Assert.Equal(Fixed64.Zero, exit);
        Assert.True(originContained);
        Assert.False(maximumContainedStrict);
    }

    [Fact]
    public void BoundedRayCenteredCapsule_EndBoundaryIsNotStrictlyContained()
    {
        var ray = new FixedRay(Vector3d.Zero, Vector3d.Right);

        Assert.True(ray.TryGetCapsuleIntersectionInterval(
            new Vector3d((Fixed64)2, Fixed64.Zero, Fixed64.Zero),
            Vector3d.Up,
            Fixed64.One,
            Fixed64.One,
            Fixed64.Zero,
            Fixed64.One,
            out Fixed64 entry,
            out Fixed64 exit,
            out bool originContained,
            out bool maximumContainedStrict));

        Assert.Equal(Fixed64.One, entry);
        Assert.Equal(Fixed64.One, exit);
        Assert.False(originContained);
        Assert.False(maximumContainedStrict);
    }

    [Fact]
    public void BoundedRayFiniteCylinder_EndpointCenteredAndAffineFamiliesAgree()
    {
        var ray = new FixedRay(
            new Vector3d((Fixed64)(-3), Fixed64.Zero, Fixed64.Zero),
            Vector3d.Right);
        var axis = new FixedSegment(
            new Vector3d(Fixed64.Zero, -Fixed64.One, Fixed64.Zero),
            new Vector3d(Fixed64.Zero, Fixed64.One, Fixed64.Zero));

        Assert.True(ray.TryGetFiniteCylinderIntersectionInterval(
            axis,
            Fixed64.One,
            Fixed64.Zero,
            (Fixed64)6,
            out Fixed64 endpointEntry,
            out Fixed64 endpointExit,
            out bool originContained,
            out bool maximumContainedStrict));
        Assert.True(ray.TryGetFiniteCylinderIntersectionInterval(
            Vector3d.Zero,
            Vector3d.Up,
            Fixed64.One,
            Fixed64.One,
            Fixed64.Zero,
            Fixed64.Zero,
            (Fixed64)6,
            out Fixed64 centeredEntry,
            out Fixed64 centeredExit,
            out _,
            out _));
        Assert.True(ray.TryGetFiniteCylinderIntersectionInterval(
            axis,
            Fixed64.One,
            Fixed64.One,
            Fixed64.Zero,
            Fixed64.Zero,
            (Fixed64)6,
            out Fixed64 affineEntry,
            out Fixed64 affineExit));

        Assert.Equal((Fixed64)2, endpointEntry);
        Assert.Equal((Fixed64)4, endpointExit);
        Assert.Equal(endpointEntry, centeredEntry);
        Assert.Equal(endpointExit, centeredExit);
        Assert.Equal(endpointEntry, affineEntry);
        Assert.Equal(endpointExit, affineExit);
        Assert.False(originContained);
        Assert.False(maximumContainedStrict);
    }

    [Fact]
    public void BoundedRayContainment_ClassifiesCapsSidesAndFlatCapBoundaries()
    {
        var axis2d = new FixedSegment2d(-Vector2d.Forward, Vector2d.Forward);
        var axis3d = new FixedSegment(-Vector3d.Up, Vector3d.Up);
        var axialRay2d = new FixedRay2d(new Vector2d(Fixed64.Zero, (Fixed64)(-3)), Vector2d.Forward);
        var axialRay3d = new FixedRay(new Vector3d(Fixed64.Zero, (Fixed64)(-3), Fixed64.Zero), Vector3d.Up);

        Assert.True(axialRay2d.TryGetCapsuleIntersectionInterval(
            axis2d, Fixed64.One, Fixed64.Zero, (Fixed64)6,
            out _, out _, out _, out bool maximumInPositiveCap2d));
        Assert.True(axialRay3d.TryGetCapsuleIntersectionInterval(
            axis3d, Fixed64.One, Fixed64.Zero, (Fixed64)6,
            out _, out _, out _, out bool maximumInPositiveCap3d));
        Assert.False(maximumInPositiveCap2d);
        Assert.False(maximumInPositiveCap3d);

        Assert.False(axialRay2d.TryGetCapsuleIntersectionInterval(
            axis2d, Fixed64.One, Fixed64.Zero, Fixed64.Half,
            out _, out _, out _, out bool maximumInNegativeCap2d));
        Assert.False(axialRay3d.TryGetCapsuleIntersectionInterval(
            axis3d, Fixed64.One, Fixed64.Zero, Fixed64.Half,
            out _, out _, out _, out bool maximumInNegativeCap3d));
        Assert.False(maximumInNegativeCap2d);
        Assert.False(maximumInNegativeCap3d);

        var sideRay2d = new FixedRay2d(new Vector2d((Fixed64)(-2), Fixed64.Zero), Vector2d.Right);
        var sideRay3d = new FixedRay(new Vector3d((Fixed64)(-2), Fixed64.Zero, Fixed64.Zero), Vector3d.Right);
        Assert.True(sideRay2d.TryGetCapsuleIntersectionInterval(
            Vector2d.Zero, Vector2d.Forward, Fixed64.One, Fixed64.One,
            Fixed64.Zero, (Fixed64)2 + Fixed64.Half,
            out _, out _, out _, out bool centeredSide2d));
        Assert.True(sideRay3d.TryGetCapsuleIntersectionInterval(
            Vector3d.Zero, Vector3d.Up, Fixed64.One, Fixed64.One,
            Fixed64.Zero, (Fixed64)2 + Fixed64.Half,
            out _, out _, out _, out bool centeredSide3d));
        Assert.True(centeredSide2d);
        Assert.True(centeredSide3d);

        var capBoundaryRay = new FixedRay(-Vector3d.Up, Vector3d.Up);
        Assert.True(capBoundaryRay.TryGetFiniteCylinderIntersectionInterval(
            axis3d, Fixed64.One, Fixed64.Zero, Fixed64.Two,
            out _, out _, out bool originOnCap, out bool maximumOnCapStrict));
        Assert.True(originOnCap);
        Assert.False(maximumOnCapStrict);
        Assert.True(capBoundaryRay.TryGetFiniteCylinderIntersectionInterval(
            Vector3d.Zero, Vector3d.Up, Fixed64.One, Fixed64.One,
            Fixed64.Zero, Fixed64.Zero, Fixed64.Two,
            out _, out _, out bool centeredOriginOnCap, out bool centeredMaximumOnCapStrict));
        Assert.True(centeredOriginOnCap);
        Assert.False(centeredMaximumOnCapStrict);

        var lowerCapArrivalRay = new FixedRay(-Vector3d.Up * Fixed64.Two, Vector3d.Up);
        Assert.True(lowerCapArrivalRay.TryGetFiniteCylinderIntersectionInterval(
            axis3d, Fixed64.One, Fixed64.Zero, Fixed64.One,
            out Fixed64 lowerCapEntry, out Fixed64 lowerCapExit,
            out bool lowerCapOriginContained, out bool lowerCapMaximumContainedStrict));
        Assert.Equal(Fixed64.One, lowerCapEntry);
        Assert.Equal(Fixed64.One, lowerCapExit);
        Assert.False(lowerCapOriginContained);
        Assert.False(lowerCapMaximumContainedStrict);

        Assert.True(lowerCapArrivalRay.TryGetFiniteCylinderIntersectionInterval(
            Vector3d.Zero, Vector3d.Up, Fixed64.One, Fixed64.One,
            Fixed64.Zero, Fixed64.Zero, Fixed64.One,
            out Fixed64 centeredLowerCapEntry, out Fixed64 centeredLowerCapExit,
            out bool centeredLowerCapOriginContained, out bool centeredLowerCapMaximumContainedStrict));
        Assert.Equal(Fixed64.One, centeredLowerCapEntry);
        Assert.Equal(Fixed64.One, centeredLowerCapExit);
        Assert.False(centeredLowerCapOriginContained);
        Assert.False(centeredLowerCapMaximumContainedStrict);
    }

    [Fact]
    public void BoundedRayFiniteAxisIntervals_RejectPathsBeyondConceptualCaps()
    {
        var ray2d = new FixedRay2d(new Vector2d(Fixed64.Zero, (Fixed64)3), Vector2d.Forward);
        var ray3d = new FixedRay(new Vector3d((Fixed64)(-3), (Fixed64)2, Fixed64.Zero), Vector3d.Right);
        var axis = new FixedSegment(-Vector3d.Up, Vector3d.Up);

        Assert.False(ray2d.TryGetCapsuleIntersectionInterval(
            Vector2d.Zero, Vector2d.Forward, Fixed64.One, Fixed64.Half,
            Fixed64.One, out _, out _));
        Assert.False(ray3d.TryGetCapsuleIntersectionInterval(
            Vector3d.Zero, Vector3d.Up, Fixed64.One, Fixed64.Half,
            Fixed64.One, out _, out _));
        Assert.False(ray3d.TryGetFiniteCylinderIntersectionInterval(
            axis, Fixed64.One, Fixed64.One, Fixed64.Zero,
            Fixed64.Zero, (Fixed64)6, out _, out _));
        Assert.False(ray3d.TryGetFiniteCylinderIntersectionInterval(
            Vector3d.Zero, Vector3d.Up, Fixed64.One, Fixed64.One,
            Fixed64.Zero, Fixed64.Zero, (Fixed64)6,
            out _, out _, out _, out _));
    }

    [Fact]
    public void BoundedRayFiniteAxisContracts_RejectInvalidShapeParameters()
    {
        var ray2d = new FixedRay2d(Vector2d.Zero, Vector2d.Right);
        var ray3d = new FixedRay(Vector3d.Zero, Vector3d.Right);
        var collapsed2d = new FixedSegment2d(Vector2d.Zero, Vector2d.Zero);
        var collapsed3d = new FixedSegment(Vector3d.Zero, Vector3d.Zero);
        var axis2d = new FixedSegment2d(Vector2d.Zero, Vector2d.Forward);
        var axis3d = new FixedSegment(Vector3d.Zero, Vector3d.Up);

        Assert.Throws<ArgumentOutOfRangeException>(() => ray2d.TryGetCapsuleIntersectionInterval(
            collapsed2d, -Fixed64.One, Fixed64.One, out _, out _));
        Assert.Throws<ArgumentOutOfRangeException>(() => ray2d.TryGetCapsuleIntersectionInterval(
            axis2d, Fixed64.One, -Fixed64.One, Fixed64.One, out _, out _, out _, out _));
        Assert.False(ray2d.TryGetCapsuleIntersectionInterval(
            axis2d, Fixed64.One, Fixed64.Zero, -Fixed64.One,
            out Fixed64 entry2d, out Fixed64 exit2d,
            out bool origin2d, out bool maximum2d));
        Assert.Equal(Fixed64.Zero, entry2d);
        Assert.Equal(Fixed64.Zero, exit2d);
        Assert.False(origin2d);
        Assert.False(maximum2d);
        Assert.Throws<ArgumentException>(() => ray2d.TryGetCapsuleIntersectionInterval(
            Vector2d.Zero, Vector2d.One, Fixed64.One, Fixed64.One, Fixed64.One, out _, out _));
        Assert.Throws<ArgumentOutOfRangeException>(() => ray2d.TryGetCapsuleIntersectionInterval(
            Vector2d.Zero, Vector2d.Forward, -Fixed64.One, Fixed64.One,
            Fixed64.Zero, Fixed64.One, out _, out _, out _, out _));
        Assert.Throws<ArgumentOutOfRangeException>(() => ray2d.TryGetCapsuleIntersectionInterval(
            Vector2d.Zero, Vector2d.Forward, Fixed64.One, -Fixed64.One,
            Fixed64.Zero, Fixed64.One, out _, out _, out _, out _));
        Assert.Throws<ArgumentOutOfRangeException>(() => ray2d.TryGetCapsuleIntersectionInterval(
            Vector2d.Zero, Vector2d.Forward, Fixed64.One, Fixed64.One,
            -Fixed64.One, Fixed64.One, out _, out _, out _, out _));
        Assert.False(ray2d.TryGetCapsuleIntersectionInterval(
            Vector2d.Zero, Vector2d.Forward, Fixed64.One, Fixed64.One,
            Fixed64.Zero, -Fixed64.One, out _, out _, out _, out _));

        Assert.Throws<ArgumentOutOfRangeException>(() => ray3d.TryGetCapsuleIntersectionInterval(
            axis3d, -Fixed64.One, Fixed64.Zero, Fixed64.One, out _, out _, out _, out _));
        Assert.Throws<ArgumentOutOfRangeException>(() => ray3d.TryGetCapsuleIntersectionInterval(
            axis3d, Fixed64.One, -Fixed64.One, Fixed64.One, out _, out _, out _, out _));
        Assert.False(ray3d.TryGetCapsuleIntersectionInterval(
            axis3d, Fixed64.One, Fixed64.Zero, -Fixed64.One, out _, out _, out _, out _));
        Assert.Throws<ArgumentException>(() => ray3d.TryGetCapsuleIntersectionInterval(
            Vector3d.Zero, Vector3d.One, Fixed64.One, Fixed64.One,
            Fixed64.Zero, Fixed64.One, out _, out _, out _, out _));
        Assert.Throws<ArgumentOutOfRangeException>(() => ray3d.TryGetCapsuleIntersectionInterval(
            Vector3d.Zero, Vector3d.Up, -Fixed64.One, Fixed64.One,
            Fixed64.Zero, Fixed64.One, out _, out _, out _, out _));
        Assert.Throws<ArgumentOutOfRangeException>(() => ray3d.TryGetCapsuleIntersectionInterval(
            Vector3d.Zero, Vector3d.Up, Fixed64.One, -Fixed64.One,
            Fixed64.Zero, Fixed64.One, out _, out _, out _, out _));
        Assert.Throws<ArgumentOutOfRangeException>(() => ray3d.TryGetCapsuleIntersectionInterval(
            Vector3d.Zero, Vector3d.Up, Fixed64.One, Fixed64.One,
            -Fixed64.One, Fixed64.One, out _, out _, out _, out _));
        Assert.False(ray3d.TryGetCapsuleIntersectionInterval(
            Vector3d.Zero, Vector3d.Up, Fixed64.One, Fixed64.One,
            Fixed64.Zero, -Fixed64.One, out _, out _, out _, out _));

        Assert.Throws<ArgumentException>(() => ray3d.TryGetFiniteCylinderIntersectionInterval(
            collapsed3d, Fixed64.One, Fixed64.One, out _, out _));
        Assert.Throws<ArgumentOutOfRangeException>(() => ray3d.TryGetFiniteCylinderIntersectionInterval(
            axis3d, -Fixed64.One, Fixed64.Zero, Fixed64.One, out _, out _, out _, out _));
        Assert.Throws<ArgumentOutOfRangeException>(() => ray3d.TryGetFiniteCylinderIntersectionInterval(
            axis3d, Fixed64.One, -Fixed64.One, Fixed64.One, out _, out _, out _, out _));
        Assert.False(ray3d.TryGetFiniteCylinderIntersectionInterval(
            axis3d, Fixed64.One, Fixed64.Zero, -Fixed64.One, out _, out _, out _, out _));

        Assert.Throws<ArgumentException>(() => ray3d.TryGetFiniteCylinderIntersectionInterval(
            collapsed3d, Fixed64.One, Fixed64.One, Fixed64.Zero,
            Fixed64.Zero, Fixed64.One, out _, out _));
        Assert.Throws<ArgumentOutOfRangeException>(() => ray3d.TryGetFiniteCylinderIntersectionInterval(
            axis3d, Fixed64.Zero, Fixed64.One, Fixed64.Zero,
            Fixed64.Zero, Fixed64.One, out _, out _));
        Assert.Throws<ArgumentOutOfRangeException>(() => ray3d.TryGetFiniteCylinderIntersectionInterval(
            axis3d, Fixed64.One, -Fixed64.One, Fixed64.Zero,
            Fixed64.Zero, Fixed64.One, out _, out _));
        Assert.Throws<ArgumentOutOfRangeException>(() => ray3d.TryGetFiniteCylinderIntersectionInterval(
            axis3d, Fixed64.One, Fixed64.One, -Fixed64.One,
            Fixed64.Zero, Fixed64.One, out _, out _));
        Assert.Throws<ArgumentOutOfRangeException>(() => ray3d.TryGetFiniteCylinderIntersectionInterval(
            axis3d, Fixed64.One, Fixed64.One, Fixed64.Zero,
            -Fixed64.One, Fixed64.One, out _, out _));
        Assert.False(ray3d.TryGetFiniteCylinderIntersectionInterval(
            axis3d, Fixed64.One, Fixed64.One, Fixed64.Zero,
            Fixed64.Zero, -Fixed64.One, out _, out _));

        Assert.Throws<ArgumentException>(() => ray3d.TryGetFiniteCylinderIntersectionInterval(
            Vector3d.Zero, Vector3d.One, Fixed64.One, Fixed64.One,
            Fixed64.Zero, Fixed64.Zero, Fixed64.One, out _, out _, out _, out _));
        Assert.Throws<ArgumentOutOfRangeException>(() => ray3d.TryGetFiniteCylinderIntersectionInterval(
            Vector3d.Zero,
            Vector3d.Up,
            Fixed64.Zero,
            Fixed64.One,
            Fixed64.Zero,
            Fixed64.Zero,
            Fixed64.One,
            out _,
            out _));
        Assert.Throws<ArgumentOutOfRangeException>(() => ray3d.TryGetFiniteCylinderIntersectionInterval(
            Vector3d.Zero, Vector3d.Up, Fixed64.One, -Fixed64.One,
            Fixed64.Zero, Fixed64.Zero, Fixed64.One, out _, out _, out _, out _));
        Assert.Throws<ArgumentOutOfRangeException>(() => ray3d.TryGetFiniteCylinderIntersectionInterval(
            Vector3d.Zero, Vector3d.Up, Fixed64.One, Fixed64.One,
            -Fixed64.One, Fixed64.Zero, Fixed64.One, out _, out _, out _, out _));
        Assert.Throws<ArgumentOutOfRangeException>(() => ray3d.TryGetFiniteCylinderIntersectionInterval(
            Vector3d.Zero, Vector3d.Up, Fixed64.One, Fixed64.One,
            Fixed64.Zero, -Fixed64.One, Fixed64.One, out _, out _, out _, out _));
        Assert.False(ray3d.TryGetFiniteCylinderIntersectionInterval(
            Vector3d.Zero, Vector3d.Up, Fixed64.One, Fixed64.One,
            Fixed64.Zero, Fixed64.Zero, -Fixed64.One, out _, out _, out _, out _));

        Assert.False(ray3d.TryGetCapsuleIntersectionInterval(
            collapsed3d, Fixed64.One, -Fixed64.One, out _, out _));
    }

    [Fact]
    public void GetPoint_FusesMultiplyAddAcrossSaturatingAndHalfwayIntermediates()
    {
        Fixed64 position = Fixed64.MaxValue - Fixed64.One;
        Fixed64 direction = Fixed64.MaxValue;
        Fixed64 parameter = Fixed64.FromRaw(-1L);
        Fixed64 expected = Fixed64.FromRaw(
            (Fixed64.MaxValue - Fixed64.One).m_rawValue - Fixed64.Half.m_rawValue);
        var ray2d = new FixedRay2d(
            new Vector2d(position, Fixed64.FromRaw(1L)),
            new Vector2d(direction, Fixed64.Half));
        var ray3d = new FixedRay(
            new Vector3d(position, Fixed64.FromRaw(1L), -position),
            new Vector3d(direction, Fixed64.Half, -direction));

        Assert.True(Fixed64.TryMultiplyAdd(direction, parameter, position, out Fixed64 scalar));
        Assert.Equal(expected, scalar);
        Assert.Equal(new Vector2d(expected, Fixed64.Zero), ray2d.GetPoint(parameter));
        Assert.Equal(
            new Vector3d(expected, Fixed64.Zero, -expected),
            ray3d.GetPoint(parameter));
        Assert.True(ray2d.TryGetPoint(parameter, out Vector2d point2d));
        Assert.Equal(new Vector2d(expected, Fixed64.Zero), point2d);
        Assert.True(ray3d.TryGetPoint(parameter, out Vector3d point3d));
        Assert.Equal(new Vector3d(expected, Fixed64.Zero, -expected), point3d);

        Assert.False(new FixedRay2d(Vector2d.Zero, new Vector2d(Fixed64.MaxValue, Fixed64.Zero))
            .TryGetPoint(Fixed64.Two, out point2d));
        Assert.Equal(default, point2d);
        Assert.False(new FixedRay(Vector3d.Zero, new Vector3d(Fixed64.MaxValue, Fixed64.Zero, Fixed64.Zero))
            .TryGetPoint(Fixed64.Two, out point3d));
        Assert.Equal(default, point3d);
        Assert.False(new FixedRay(Vector3d.Zero, new Vector3d(Fixed64.Zero, Fixed64.MaxValue, Fixed64.Zero))
            .TryGetPoint(Fixed64.Two, out point3d));
        Assert.Equal(default, point3d);
        Assert.False(new FixedRay(Vector3d.Zero, new Vector3d(Fixed64.Zero, Fixed64.Zero, Fixed64.MaxValue))
            .TryGetPoint(Fixed64.Two, out point3d));
        Assert.Equal(default, point3d);
    }
}
