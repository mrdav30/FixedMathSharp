using FixedMathSharp.Geometry;
using System;
using Xunit;

namespace FixedMathSharp.Tests.Bounds;

public sealed class FixedSegmentRoundedBoxTests
{
    [Fact]
    public void RoundedBox_FaceEntry_ReturnsPhysicalDistance()
    {
        var query = new FixedSegment(
            new Vector3d((Fixed64)2, Fixed64.Zero, Fixed64.Zero),
            Vector3d.Zero);
        FixedBoundBox box = FixedBoundBox.FromCenterAndSize(Vector3d.Zero, Vector3d.One);

        bool hit = query.TryGetSweptSphereBoxIntersectionDistance(
            box,
            Fixed64.Half,
            Fixed64.Two,
            out Fixed64 distance);

        Assert.True(hit);
        Assert.Equal(Fixed64.One, distance);
    }

    [Fact]
    public void RoundedBox_DiagonalEdgeSeparation_RejectsSharpExpansionHit()
    {
        var query = new FixedSegment(
            new Vector3d((Fixed64)2, Fixed64.FromFraction(9, 10), Fixed64.Zero),
            new Vector3d(Fixed64.FromFraction(9, 10), Fixed64.FromFraction(9, 10), Fixed64.Zero));
        FixedBoundBox box = FixedBoundBox.FromCenterAndSize(Vector3d.Zero, Vector3d.One);
        Vector3d.TryGetDistance(query.Start, query.End, out Fixed64 totalDistance);

        bool hit = query.TryGetSweptSphereBoxIntersectionDistance(
            box,
            Fixed64.Half,
            totalDistance,
            out Fixed64 distance);

        Assert.False(hit);
        Assert.Equal(Fixed64.Zero, distance);
    }

    [Fact]
    public void RoundedBox_EdgeTangentAtEndpoint_ReturnsTotalDistance()
    {
        var query = new FixedSegment(
            new Vector3d((Fixed64)2, Fixed64.One, Fixed64.Zero),
            new Vector3d(Fixed64.FromFraction(7, 8), Fixed64.One, Fixed64.Zero));
        FixedBoundBox box = FixedBoundBox.FromCenterAndSize(Vector3d.Zero, Vector3d.One);
        Vector3d.TryGetDistance(query.Start, query.End, out Fixed64 totalDistance);

        bool hit = query.TryGetSweptSphereBoxIntersectionDistance(
            box,
            Fixed64.FromFraction(5, 8),
            totalDistance,
            out Fixed64 distance);

        Assert.True(hit);
        Assert.Equal(totalDistance, distance);
    }

    [Fact]
    public void RoundedBox_CornerTangentAtEndpoint_ReturnsTotalDistance()
    {
        var query = new FixedSegment(
            new Vector3d((Fixed64)2, Fixed64.FromFraction(7, 8), Fixed64.One),
            new Vector3d(Fixed64.Half, Fixed64.FromFraction(7, 8), Fixed64.One));
        FixedBoundBox box = FixedBoundBox.FromCenterAndSize(Vector3d.Zero, Vector3d.One);

        bool hit = query.TryGetSweptSphereBoxIntersectionDistance(
            box,
            Fixed64.FromFraction(5, 8),
            Fixed64.FromFraction(3, 2),
            out Fixed64 distance);

        Assert.True(hit);
        Assert.Equal(Fixed64.FromFraction(3, 2), distance);
    }

    [Fact]
    public void RoundedBox_StartWithinRoundedEdge_ReturnsZero()
    {
        var query = new FixedSegment(
            new Vector3d(Fixed64.FromFraction(4, 5), Fixed64.FromFraction(4, 5), Fixed64.Zero),
            new Vector3d((Fixed64)2, (Fixed64)2, Fixed64.Zero));
        FixedBoundBox box = FixedBoundBox.FromCenterAndSize(Vector3d.Zero, Vector3d.One);

        bool hit = query.TryGetSweptSphereBoxIntersectionDistance(
            box,
            Fixed64.Half,
            Fixed64.One,
            out Fixed64 distance);

        Assert.True(hit);
        Assert.Equal(Fixed64.Zero, distance);
    }

    [Fact]
    public void RoundedBox_PointOutsideDilation_ReturnsFalse()
    {
        var query = new FixedSegment(Vector3d.One, Vector3d.One);
        FixedBoundBox box = FixedBoundBox.FromCenterAndSize(Vector3d.Zero, Vector3d.One);

        bool hit = query.TryGetSweptSphereBoxIntersectionDistance(
            box,
            Fixed64.Half,
            Fixed64.Zero,
            out Fixed64 distance);

        Assert.False(hit);
        Assert.Equal(Fixed64.Zero, distance);
    }

    [Fact]
    public void RoundedBox_StartOnFaceMovingOutward_DoesNotRetainFaceState()
    {
        var query = new FixedSegment(
            new Vector3d(Fixed64.Half, (Fixed64)2, Fixed64.Zero),
            new Vector3d(Fixed64.One, Fixed64.One, Fixed64.Zero));
        FixedBoundBox box = FixedBoundBox.FromCenterAndSize(Vector3d.Zero, Vector3d.One);

        bool hit = query.TryGetSweptSphereBoxIntersectionDistance(
            box,
            Fixed64.Half,
            Fixed64.One,
            out Fixed64 distance);

        Assert.False(hit);
        Assert.Equal(Fixed64.Zero, distance);
    }

    [Fact]
    public void RoundedBox_AfterCrossingFaceCore_ReturnsLaterTangent()
    {
        var query = new FixedSegment(
            new Vector3d((Fixed64)2, (Fixed64)2, Fixed64.Zero),
            new Vector3d(Fixed64.Zero, Fixed64.FromFraction(3, 4), Fixed64.Zero));
        FixedBoundBox box = FixedBoundBox.FromCenterAndSize(Vector3d.Zero, Vector3d.One);

        bool hit = query.TryGetSweptSphereBoxIntersectionDistance(
            box,
            Fixed64.FromFraction(1, 4),
            Fixed64.One,
            out Fixed64 distance);

        Assert.True(hit);
        Assert.Equal(Fixed64.One, distance);
    }

    [Fact]
    public void RoundedBox_InterleavedAxisBreakpoints_ReturnsEarliestFeatureInterval()
    {
        var query = new FixedSegment(
            new Vector3d(Fixed64.FromFraction(3, 2), Fixed64.One, Fixed64.Zero),
            new Vector3d(-Fixed64.Half, -Fixed64.One, Fixed64.Zero));
        FixedBoundBox box = FixedBoundBox.FromCenterAndSize(Vector3d.Zero, Vector3d.One);

        bool hit = query.TryGetSweptSphereBoxIntersectionDistance(
            box,
            Fixed64.Zero,
            Fixed64.One,
            out Fixed64 distance);

        Assert.True(hit);
        Assert.Equal(Fixed64.Half, distance);
    }

    [Fact]
    public void RoundedBox_FullDomainChord_PreservesFirstDistance()
    {
        var query = new FixedSegment(
            new Vector3d(Fixed64.MinValue, Fixed64.Zero, Fixed64.Zero),
            new Vector3d(Fixed64.MaxValue, Fixed64.Zero, Fixed64.Zero));
        FixedBoundBox box = FixedBoundBox.FromMinMax(Vector3d.Zero, Vector3d.Zero);

        bool hit = query.TryGetSweptSphereBoxIntersectionDistance(
            box,
            Fixed64.One,
            Fixed64.One,
            out Fixed64 distance);

        Assert.True(hit);
        Assert.Equal(Fixed64.FromRaw(Fixed64.Half.m_rawValue - 1L), distance);
    }

    [Fact]
    public void RoundedBox_InvalidArguments_Throw()
    {
        var query = new FixedSegment(Vector3d.Zero, Vector3d.One);
        FixedBoundBox box = FixedBoundBox.FromMinMax(Vector3d.Zero, Vector3d.One);

        Assert.Throws<ArgumentOutOfRangeException>(() => query.TryGetSweptSphereBoxIntersectionDistance(
            box, -Fixed64.One, Fixed64.One, out _));
        Assert.Throws<ArgumentOutOfRangeException>(() => query.TryGetSweptSphereBoxIntersectionDistance(
            box, Fixed64.Zero, -Fixed64.One, out _));
        Assert.Throws<ArgumentException>(() => query.TryGetSweptSphereBoxIntersectionDistance(
            box, Fixed64.Zero, Fixed64.Zero, out _));
    }
}
