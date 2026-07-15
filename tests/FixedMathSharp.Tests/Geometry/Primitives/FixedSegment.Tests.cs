using FixedMathSharp.Bounds;
using MemoryPack;
using System.Text.Json;
using Xunit;

namespace FixedMathSharp.Tests.Bounds;

public class FixedSegmentTests
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
    public void GetClosestPoints_NonParallelSegments_ReturnsIntersection()
    {
        var first = new FixedSegment(new Vector3d(0, 0, 0), new Vector3d(1, 1, 0));
        var second = new FixedSegment(new Vector3d(1, 0, 0), new Vector3d(0, 1, 0));

        var result = first.GetClosestPoints(second);
        var expectedIntersection = Vector3d.FromDouble(0.5, 0.5, 0);

        Assert.Equal(expectedIntersection, result.ThisPoint);
        Assert.Equal(expectedIntersection, result.OtherPoint);
    }

    [Fact]
    public void GetClosestPoints_ParallelSegments_PreservesExistingParameterPolicy()
    {
        var parallel = new FixedSegment(new Vector3d(0, 0, 0), new Vector3d(1, 1, 0));
        var offset = new FixedSegment(new Vector3d(0, 0, 1), new Vector3d(1, 1, 1));
        var longerFirst = new FixedSegment(new Vector3d(0, 0, 0), new Vector3d(0, 2, 0));
        var shorterSecond = new FixedSegment(new Vector3d(0, 0, 0), new Vector3d(0, 1, 0));

        Assert.Equal((parallel.Start, offset.Start), parallel.GetClosestPoints(offset));
        Assert.Equal((Vector3d.Zero, Vector3d.Zero), longerFirst.GetClosestPoints(shorterSecond));
    }

    [Fact]
    public void GetClosestPoints_ClampsFirstSegmentEndpoints()
    {
        var first = new FixedSegment(new Vector3d(0, 0, 0), new Vector3d(1, 0, 0));
        var beyondEnd = new FixedSegment(new Vector3d(2, -1, 0), new Vector3d(2, 1, 0));
        var beforeStart = new FixedSegment(new Vector3d(-1, -1, 0), new Vector3d(-1, 1, 0));

        Assert.Equal(
            (new Vector3d(1, 0, 0), new Vector3d(2, 0, 0)),
            first.GetClosestPoints(beyondEnd));
        Assert.Equal(
            (new Vector3d(0, 0, 0), new Vector3d(-1, 0, 0)),
            first.GetClosestPoints(beforeStart));
    }

    [Fact]
    public void GetClosestPoints_ClampsSecondSegmentEndpoints()
    {
        var horizontal = new FixedSegment(new Vector3d(0, 0, 0), new Vector3d(10, 0, 0));

        Assert.Equal(
            (Vector3d.Zero, new Vector3d(0, 2, 0)),
            horizontal.GetClosestPoints(new FixedSegment(new Vector3d(0, 2, 0), new Vector3d(0, 3, 0))));
        Assert.Equal(
            (Vector3d.Zero, new Vector3d(0, -2, 0)),
            horizontal.GetClosestPoints(new FixedSegment(new Vector3d(0, -3, 0), new Vector3d(0, -2, 0))));
        Assert.Equal(
            (Vector3d.Up, new Vector3d(0, 2, 0)),
            new FixedSegment(Vector3d.Zero, Vector3d.Up)
                .GetClosestPoints(new FixedSegment(new Vector3d(0, 2, 0), new Vector3d(1, 3, 0))));
        Assert.Equal(
            (Vector3d.Up, new Vector3d(0, 2, 0)),
            new FixedSegment(Vector3d.Zero, Vector3d.Up)
                .GetClosestPoints(new FixedSegment(new Vector3d(0, 2, 0), new Vector3d(0, 3, 0))));
    }

    [Fact]
    public void GetClosestPoints_PointSegments_AreHandledSymmetrically()
    {
        var identicalPoint = new FixedSegment(new Vector3d(2, 3, 4), new Vector3d(2, 3, 4));
        var distinctPoint = new FixedSegment(new Vector3d(5, 7, 9), new Vector3d(5, 7, 9));
        var line = new FixedSegment(new Vector3d(0, 0, 0), new Vector3d(4, 0, 0));
        var offLinePoint = new FixedSegment(new Vector3d(2, 3, 0), new Vector3d(2, 3, 0));

        Assert.Equal((identicalPoint.Start, identicalPoint.Start), identicalPoint.GetClosestPoints(identicalPoint));
        Assert.Equal((identicalPoint.Start, distinctPoint.Start), identicalPoint.GetClosestPoints(distinctPoint));
        Assert.Equal((offLinePoint.Start, new Vector3d(2, 0, 0)), offLinePoint.GetClosestPoints(line));
        Assert.Equal((new Vector3d(2, 0, 0), offLinePoint.Start), line.GetClosestPoints(offLinePoint));
    }

    [Fact]
    public void GetClosestPoints_SubSquareResolutionSegments_ArePointsAtTheirStarts()
    {
        var point = new Vector3d(2, 3, 0);
        var oneRawEnd = new Vector3d(Fixed64.FromRaw(point.X.m_rawValue + 1), point.Y, point.Z);
        var tiny = new FixedSegment(point, oneRawEnd);
        var line = new FixedSegment(new Vector3d(0, 0, 0), new Vector3d(4, 0, 0));

        Assert.Equal(Fixed64.Zero, tiny.LengthSquared);
        Assert.Equal((point, new Vector3d(2, 0, 0)), tiny.GetClosestPoints(line));
        Assert.Equal((new Vector3d(2, 0, 0), point), line.GetClosestPoints(tiny));
    }

    [Fact]
    public void GetClosestPoints_SwappedSegments_ReturnSwappedPoints()
    {
        FixedSegment[] firstSegments =
        {
            new(new Vector3d(0, 0, 0), new Vector3d(1, 0, 0)),
            new(new Vector3d(2, 3, 0), new Vector3d(2, 3, 0)),
            new(new Vector3d(0, 0, 0), new Vector3d(0, 4, 0)),
        };
        FixedSegment[] secondSegments =
        {
            new(new Vector3d(2, -1, 0), new Vector3d(2, 1, 0)),
            new(new Vector3d(0, 0, 0), new Vector3d(4, 0, 0)),
            new(new Vector3d(3, 2, 0), new Vector3d(5, 2, 0)),
        };

        for (int i = 0; i < firstSegments.Length; i++)
        {
            var forward = firstSegments[i].GetClosestPoints(secondSegments[i]);
            var reverse = secondSegments[i].GetClosestPoints(firstSegments[i]);

            Assert.Equal(forward.ThisPoint, reverse.OtherPoint);
            Assert.Equal(forward.OtherPoint, reverse.ThisPoint);
        }
    }

    [Fact]
    public void GetClosestPoints_ReversedEndpoints_PreserveClosestLocations()
    {
        var first = new FixedSegment(new Vector3d(0, 0, 0), new Vector3d(2, 0, 0));
        var second = new FixedSegment(new Vector3d(5, 3, 0), new Vector3d(5, 5, 0));
        var expected = (new Vector3d(2, 0, 0), new Vector3d(5, 3, 0));

        Assert.Equal(expected, first.GetClosestPoints(second));
        Assert.Equal(expected,
            new FixedSegment(first.End, first.Start)
                .GetClosestPoints(new FixedSegment(second.End, second.Start)));
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
