using FixedMathSharp.Bounds;
using MemoryPack;
using System.Text.Json;
using Xunit;

namespace FixedMathSharp.Tests.Bounds;

public class FixedSegment2dTests
{
    [Fact]
    public void Constructor_AssignsEndpointsAndDerivedValues()
    {
        var segment = new FixedSegment2d(new Vector2d(1, 2), new Vector2d(4, 6));

        Assert.Equal(new Vector2d(1, 2), segment.Start);
        Assert.Equal(new Vector2d(4, 6), segment.End);
        Assert.Equal(new Vector2d(3, 4), segment.Delta);
        Assert.Equal(new Fixed64(5), segment.Length);
        Assert.Equal(new Fixed64(25), segment.LengthSquared);
        Assert.Equal(FixedBoundArea.FromMinMax(new Vector2d(1, 2), new Vector2d(4, 6)), segment.Bounds);
    }

    [Fact]
    public void Bounds_NormalizesReversedEndpoints()
    {
        var segment = new FixedSegment2d(new Vector2d(5, -2), new Vector2d(-1, 3));

        Assert.Equal(FixedBoundArea.FromMinMax(new Vector2d(-1, -2), new Vector2d(5, 3)), segment.Bounds);
    }

    [Fact]
    public void ClosestPoint_ProjectsInsideHorizontalVerticalAndDiagonalSegments()
    {
        var horizontal = new FixedSegment2d(new Vector2d(0, 0), new Vector2d(8, 0));
        var vertical = new FixedSegment2d(new Vector2d(3, -4), new Vector2d(3, 4));
        var diagonal = new FixedSegment2d(new Vector2d(0, 0), new Vector2d(8, 8));

        Assert.Equal(new Vector2d(4, 0), horizontal.ClosestPoint(new Vector2d(4, 3)));
        Assert.Equal(new Vector2d(3, -2), vertical.ClosestPoint(new Vector2d(7, -2)));
        Assert.Equal(new Vector2d(4, 4), diagonal.ClosestPoint(new Vector2d(6, 2)));
    }

    [Fact]
    public void ClosestPoint_ClampsToEndpointsForOutsideOrReversedSegments()
    {
        var segment = new FixedSegment2d(new Vector2d(10, 0), new Vector2d(0, 0));

        Assert.Equal(new Vector2d(10, 0), segment.ClosestPoint(new Vector2d(20, 5)));
        Assert.Equal(new Vector2d(0, 0), segment.ClosestPoint(new Vector2d(-3, -2)));
    }

    [Fact]
    public void ZeroLengthSegment_ReturnsStartForClosestPointAndDistance()
    {
        var segment = new FixedSegment2d(new Vector2d(2, 3), new Vector2d(2, 3));

        Assert.Equal(Vector2d.Zero, segment.Delta);
        Assert.Equal(Fixed64.Zero, segment.Length);
        Assert.Equal(Fixed64.Zero, segment.LengthSquared);
        Assert.Equal(new Vector2d(2, 3), segment.ClosestPoint(new Vector2d(9, 9)));
        Assert.Equal(new Fixed64(85), segment.DistanceSquared(new Vector2d(9, 9)));
        Assert.Equal(FixedBoundArea.FromMinMax(new Vector2d(2, 3), new Vector2d(2, 3)), segment.Bounds);
    }

    [Fact]
    public void DistanceSquared_UsesClosestPointWithoutSquareRoot()
    {
        var segment = new FixedSegment2d(new Vector2d(0, 0), new Vector2d(8, 0));

        Assert.Equal(new Fixed64(9), segment.DistanceSquared(new Vector2d(4, 3)));
        Assert.Equal(new Fixed64(25), segment.DistanceSquared(new Vector2d(13, 0)));
    }

    [Fact]
    public void EqualityDeconstructAndHashCode_UseEndpoints()
    {
        var segment = new FixedSegment2d(new Vector2d(1, 2), new Vector2d(3, 4));
        var same = new FixedSegment2d(new Vector2d(1, 2), new Vector2d(3, 4));
        var reversed = new FixedSegment2d(new Vector2d(3, 4), new Vector2d(1, 2));

        segment.Deconstruct(out Vector2d start, out Vector2d end);

        Assert.Equal(new Vector2d(1, 2), start);
        Assert.Equal(new Vector2d(3, 4), end);
        Assert.True(segment == same);
        Assert.False(segment != same);
        Assert.Equal(segment.GetHashCode(), same.GetHashCode());
        Assert.NotEqual(segment, reversed);
        Assert.False(segment == reversed);
        Assert.True(segment != reversed);
        Assert.False(segment.Equals("not a segment"));
    }

    [Fact]
    public void JsonSerialization_RoundTripsState()
    {
        var segment = new FixedSegment2d(new Vector2d(1, 2), new Vector2d(3, 4));

        byte[] json = JsonSerializer.SerializeToUtf8Bytes(segment);
        var roundTrip = JsonSerializer.Deserialize<FixedSegment2d>(json);

        Assert.Equal(segment, roundTrip);
    }

#if !FIXEDMATHSHARP_DISABLE_MEMORYPACK
    [Fact]
    public void MemoryPackSerialization_RoundTripsState()
    {
        var segment = new FixedSegment2d(new Vector2d(1, 2), new Vector2d(3, 4));

        byte[] bytes = MemoryPackSerializer.Serialize(segment);
        var roundTrip = MemoryPackSerializer.Deserialize<FixedSegment2d>(bytes);

        Assert.Equal(segment, roundTrip);
    }
#endif
}
