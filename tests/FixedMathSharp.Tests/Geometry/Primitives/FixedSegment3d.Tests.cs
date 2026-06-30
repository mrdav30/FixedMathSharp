using FixedMathSharp.Bounds;
using MemoryPack;
using System.Text.Json;
using Xunit;

namespace FixedMathSharp.Tests.Bounds;

public class FixedSegment3dTests
{
    [Fact]
    public void Constructor_AssignsEndpointsAndDerivedValues()
    {
        var segment = new FixedSegment(new Vector3d(1, 2, 3), new Vector3d(3, 8, 6));

        Assert.Equal(new Vector3d(1, 2, 3), segment.Start);
        Assert.Equal(new Vector3d(3, 8, 6), segment.End);
        Assert.Equal(new Vector3d(2, 6, 3), segment.Delta);
        Assert.Equal(new Fixed64(7), segment.Length);
        Assert.Equal(new Fixed64(49), segment.LengthSquared);
        Assert.Equal(
            FixedBoundBox.FromMinMax(new Vector3d(1, 2, 3), new Vector3d(3, 8, 6)),
            segment.Bounds);
    }

    [Fact]
    public void Bounds_NormalizesReversedEndpoints()
    {
        var segment = new FixedSegment(new Vector3d(5, -2, 7), new Vector3d(-1, 3, -4));

        Assert.Equal(
            FixedBoundBox.FromMinMax(new Vector3d(-1, -2, -4), new Vector3d(5, 3, 7)),
            segment.Bounds);
    }

    [Fact]
    public void ClosestPoint_ProjectsInsideHorizontalVerticalDepthAndDiagonalSegments()
    {
        var horizontal = new FixedSegment(new Vector3d(0, 0, 0), new Vector3d(8, 0, 0));
        var vertical = new FixedSegment(new Vector3d(3, -4, 2), new Vector3d(3, 4, 2));
        var depth = new FixedSegment(new Vector3d(1, 2, -8), new Vector3d(1, 2, 8));
        var diagonal = new FixedSegment(new Vector3d(0, 0, 0), new Vector3d(8, 8, 8));

        Assert.Equal(new Vector3d(4, 0, 0), horizontal.ClosestPoint(new Vector3d(4, 3, 2)));
        Assert.Equal(new Vector3d(3, -2, 2), vertical.ClosestPoint(new Vector3d(7, -2, 5)));
        Assert.Equal(new Vector3d(1, 2, 4), depth.ClosestPoint(new Vector3d(5, 6, 4)));
        Assert.Equal(new Vector3d(4, 4, 4), diagonal.ClosestPoint(new Vector3d(8, 4, 0)));
    }

    [Fact]
    public void ClosestPoint_ClampsToEndpointsForOutsideOrReversedSegments()
    {
        var segment = new FixedSegment(new Vector3d(10, 0, 0), new Vector3d(0, 0, 0));

        Assert.Equal(new Vector3d(10, 0, 0), segment.ClosestPoint(new Vector3d(20, 5, 1)));
        Assert.Equal(new Vector3d(0, 0, 0), segment.ClosestPoint(new Vector3d(-3, -2, -1)));
    }

    [Fact]
    public void ZeroLengthSegment_ReturnsStartForClosestPointAndDistance()
    {
        var segment = new FixedSegment(new Vector3d(2, 3, 4), new Vector3d(2, 3, 4));

        Assert.Equal(Vector3d.Zero, segment.Delta);
        Assert.Equal(Fixed64.Zero, segment.Length);
        Assert.Equal(Fixed64.Zero, segment.LengthSquared);
        Assert.Equal(new Vector3d(2, 3, 4), segment.ClosestPoint(new Vector3d(9, 9, 9)));
        Assert.Equal(new Fixed64(110), segment.DistanceSquared(new Vector3d(9, 9, 9)));
        Assert.Equal(
            FixedBoundBox.FromMinMax(new Vector3d(2, 3, 4), new Vector3d(2, 3, 4)),
            segment.Bounds);
    }

    [Fact]
    public void DistanceSquared_UsesClosestPointWithoutSquareRoot()
    {
        var segment = new FixedSegment(new Vector3d(0, 0, 0), new Vector3d(8, 0, 0));

        Assert.Equal(new Fixed64(13), segment.DistanceSquared(new Vector3d(4, 3, 2)));
        Assert.Equal(new Fixed64(25), segment.DistanceSquared(new Vector3d(13, 0, 0)));
    }

    [Fact]
    public void EqualityDeconstructAndHashCode_UseOrderedEndpoints()
    {
        var segment = new FixedSegment(new Vector3d(1, 2, 3), new Vector3d(4, 5, 6));
        var same = new FixedSegment(new Vector3d(1, 2, 3), new Vector3d(4, 5, 6));
        var reversed = new FixedSegment(new Vector3d(4, 5, 6), new Vector3d(1, 2, 3));

        segment.Deconstruct(out Vector3d start, out Vector3d end);

        Assert.Equal(new Vector3d(1, 2, 3), start);
        Assert.Equal(new Vector3d(4, 5, 6), end);
        Assert.True(segment == same);
        Assert.False(segment != same);
        Assert.Equal(segment.GetHashCode(), same.GetHashCode());
        Assert.Equal(segment.Bounds, reversed.Bounds);
        Assert.NotEqual(segment, reversed);
        Assert.False(segment == reversed);
        Assert.True(segment != reversed);
        Assert.False(segment.Equals("not a segment"));
    }

    [Fact]
    public void JsonSerialization_RoundTripsState()
    {
        var segment = new FixedSegment(new Vector3d(1, 2, 3), new Vector3d(4, 5, 6));

        byte[] json = JsonSerializer.SerializeToUtf8Bytes(segment);
        var roundTrip = JsonSerializer.Deserialize<FixedSegment>(json);

        Assert.Equal(segment, roundTrip);
    }

#if !FIXEDMATHSHARP_DISABLE_MEMORYPACK
    [Fact]
    public void MemoryPackSerialization_RoundTripsState()
    {
        var segment = new FixedSegment(new Vector3d(1, 2, 3), new Vector3d(4, 5, 6));

        byte[] bytes = MemoryPackSerializer.Serialize(segment);
        var roundTrip = MemoryPackSerializer.Deserialize<FixedSegment>(bytes);

        Assert.Equal(segment, roundTrip);
    }
#endif
}
