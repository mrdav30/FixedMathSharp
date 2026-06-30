using FixedMathSharp.Bounds;
using MemoryPack;
using System;
using System.Text.Json;
using Xunit;

namespace FixedMathSharp.Tests.Bounds;

public class FixedTriangle2dTests
{
    [Fact]
    public void Constructor_AssignsVerticesAndDerivedValues()
    {
        var triangle = new FixedTriangle2d(
            new Vector2d(0, 0),
            new Vector2d(6, 0),
            new Vector2d(0, 6));

        Assert.Equal(new Vector2d(0, 0), triangle.A);
        Assert.Equal(new Vector2d(6, 0), triangle.B);
        Assert.Equal(new Vector2d(0, 6), triangle.C);
        Assert.Equal(new Fixed64(18), triangle.SignedArea);
        Assert.Equal(new Fixed64(18), triangle.Area);
        Assert.Equal(new Vector2d(2, 2), triangle.Centroid);
        Assert.False(triangle.IsDegenerate);
        Assert.Equal(FixedBoundArea.FromMinMax(new Vector2d(0, 0), new Vector2d(6, 6)), triangle.Bounds);
    }

    [Fact]
    public void SignedArea_PreservesOrientation()
    {
        var counterClockwise = new FixedTriangle2d(
            new Vector2d(0, 0),
            new Vector2d(6, 0),
            new Vector2d(0, 6));
        var clockwise = new FixedTriangle2d(
            new Vector2d(0, 0),
            new Vector2d(0, 6),
            new Vector2d(6, 0));

        Assert.Equal(new Fixed64(18), counterClockwise.SignedArea);
        Assert.Equal(new Fixed64(-18), clockwise.SignedArea);
        Assert.Equal(counterClockwise.Area, clockwise.Area);
    }

    [Fact]
    public void Bounds_NormalizesAllVertices()
    {
        var triangle = new FixedTriangle2d(
            new Vector2d(5, -2),
            new Vector2d(-1, 3),
            new Vector2d(2, -7));

        Assert.Equal(FixedBoundArea.FromMinMax(new Vector2d(-1, -7), new Vector2d(5, 3)), triangle.Bounds);
    }

    [Fact]
    public void GetVertexAndGetEdge_ReturnStableOrderedGeometry()
    {
        var triangle = new FixedTriangle2d(
            new Vector2d(1, 2),
            new Vector2d(5, 2),
            new Vector2d(1, 7));

        Assert.Equal(FixedTriangle2d.VertexCount, 3);
        Assert.Equal(FixedTriangle2d.EdgeCount, 3);
        Assert.Equal(new Vector2d(1, 2), triangle.GetVertex(0));
        Assert.Equal(new Vector2d(5, 2), triangle.GetVertex(1));
        Assert.Equal(new Vector2d(1, 7), triangle.GetVertex(2));
        Assert.Equal(new FixedSegment2d(new Vector2d(1, 2), new Vector2d(5, 2)), triangle.GetEdge(0));
        Assert.Equal(new FixedSegment2d(new Vector2d(5, 2), new Vector2d(1, 7)), triangle.GetEdge(1));
        Assert.Equal(new FixedSegment2d(new Vector2d(1, 7), new Vector2d(1, 2)), triangle.GetEdge(2));
        Assert.Throws<ArgumentOutOfRangeException>(() => { _ = triangle.GetVertex(-1); });
        Assert.Throws<ArgumentOutOfRangeException>(() => { _ = triangle.GetVertex(3); });
        Assert.Throws<ArgumentOutOfRangeException>(() => { _ = triangle.GetEdge(-1); });
        Assert.Throws<ArgumentOutOfRangeException>(() => { _ = triangle.GetEdge(3); });
    }

    [Fact]
    public void GetPoint_InterpolatesFromBarycentricWeights()
    {
        var triangle = new FixedTriangle2d(
            new Vector2d(2, 4),
            new Vector2d(10, 4),
            new Vector2d(2, 12));

        Assert.Equal(new Vector2d(2, 4), triangle.GetPoint(Fixed64.Zero, Fixed64.Zero));
        Assert.Equal(new Vector2d(10, 4), triangle.GetPoint(Fixed64.One, Fixed64.Zero));
        Assert.Equal(new Vector2d(2, 12), triangle.GetPoint(Fixed64.Zero, Fixed64.One));
        Assert.Equal(
            new Vector2d(4, 8),
            triangle.GetPoint(Fixed64.FromDouble(0.25), Fixed64.Half));
    }

    [Fact]
    public void TryGetBarycentricWeights_ReturnsWeightsForPointAndPreservesWinding()
    {
        var counterClockwise = new FixedTriangle2d(
            new Vector2d(2, 4),
            new Vector2d(10, 4),
            new Vector2d(2, 12));
        var clockwise = new FixedTriangle2d(
            new Vector2d(2, 4),
            new Vector2d(2, 12),
            new Vector2d(10, 4));
        var point = new Vector2d(4, 8);

        Assert.True(counterClockwise.TryGetBarycentricWeights(point, out Fixed64 weightA, out Fixed64 weightB, out Fixed64 weightC));
        AssertNearlyEqual(Fixed64.FromDouble(0.25), weightA);
        AssertNearlyEqual(Fixed64.FromDouble(0.25), weightB);
        AssertNearlyEqual(Fixed64.Half, weightC);
        AssertNearlyEqual(point, counterClockwise.GetPoint(weightB, weightC));

        Assert.True(clockwise.TryGetBarycentricWeights(point, out weightA, out weightB, out weightC));
        AssertNearlyEqual(Fixed64.FromDouble(0.25), weightA);
        AssertNearlyEqual(Fixed64.Half, weightB);
        AssertNearlyEqual(Fixed64.FromDouble(0.25), weightC);
        AssertNearlyEqual(point, clockwise.GetPoint(weightB, weightC));
    }

    [Fact]
    public void TryGetBarycentricWeights_ReturnsFalseForDegenerateTriangle()
    {
        var triangle = new FixedTriangle2d(
            new Vector2d(0, 0),
            new Vector2d(4, 0),
            new Vector2d(8, 0));

        Assert.False(triangle.TryGetBarycentricWeights(new Vector2d(2, 0), out Fixed64 weightA, out Fixed64 weightB, out Fixed64 weightC));
        Assert.Equal(Fixed64.Zero, weightA);
        Assert.Equal(Fixed64.Zero, weightB);
        Assert.Equal(Fixed64.Zero, weightC);
    }

    [Fact]
    public void Contains_IsBoundaryInclusiveForBothOrientations()
    {
        var counterClockwise = new FixedTriangle2d(
            new Vector2d(0, 0),
            new Vector2d(6, 0),
            new Vector2d(0, 6));
        var clockwise = new FixedTriangle2d(
            new Vector2d(0, 0),
            new Vector2d(0, 6),
            new Vector2d(6, 0));

        Assert.True(counterClockwise.Contains(new Vector2d(1, 1)));
        Assert.True(counterClockwise.Contains(new Vector2d(3, 0)));
        Assert.True(counterClockwise.Contains(new Vector2d(0, 0)));
        Assert.False(counterClockwise.Contains(new Vector2d(4, 4)));

        Assert.True(clockwise.Contains(new Vector2d(1, 1)));
        Assert.True(clockwise.Contains(new Vector2d(0, 3)));
        Assert.True(clockwise.Contains(new Vector2d(6, 0)));
        Assert.False(clockwise.Contains(new Vector2d(4, 4)));
    }

    [Fact]
    public void DegenerateTriangle_ContainsOnlyCollapsedEdges()
    {
        var line = new FixedTriangle2d(
            new Vector2d(0, 0),
            new Vector2d(4, 0),
            new Vector2d(8, 0));
        var point = new FixedTriangle2d(
            new Vector2d(2, 2),
            new Vector2d(2, 2),
            new Vector2d(2, 2));

        Assert.True(line.IsDegenerate);
        Assert.Equal(Fixed64.Zero, line.Area);
        Assert.True(line.Contains(new Vector2d(2, 0)));
        Assert.True(line.Contains(new Vector2d(6, 0)));
        Assert.False(line.Contains(new Vector2d(5, 1)));

        Assert.True(point.IsDegenerate);
        Assert.True(point.Contains(new Vector2d(2, 2)));
        Assert.False(point.Contains(new Vector2d(2, 3)));
    }

    [Fact]
    public void ClosestPoint_ReturnsInteriorPointOrNearestEdgePoint()
    {
        var triangle = new FixedTriangle2d(
            new Vector2d(0, 0),
            new Vector2d(6, 0),
            new Vector2d(0, 6));

        AssertNearlyEqual(new Vector2d(1, 1), triangle.ClosestPoint(new Vector2d(1, 1)));
        AssertNearlyEqual(new Vector2d(3, 3), triangle.ClosestPoint(new Vector2d(4, 4)));
        AssertNearlyEqual(new Vector2d(2, 0), triangle.ClosestPoint(new Vector2d(2, -3)));
        AssertNearlyEqual(new Vector2d(0, 0), triangle.ClosestPoint(new Vector2d(-2, -1)));
    }

    [Fact]
    public void ClosestPoint_DegenerateTriangle_UsesClosestCollapsedEdge()
    {
        var line = new FixedTriangle2d(
            new Vector2d(0, 0),
            new Vector2d(4, 0),
            new Vector2d(8, 0));
        var point = new FixedTriangle2d(
            new Vector2d(2, 2),
            new Vector2d(2, 2),
            new Vector2d(2, 2));

        AssertNearlyEqual(new Vector2d(5, 0), line.ClosestPoint(new Vector2d(5, 3)));
        AssertNearlyEqual(new Vector2d(2, 2), point.ClosestPoint(new Vector2d(5, 6)));
    }

    [Fact]
    public void DistanceSquared_UsesClosestPointWithoutSquareRoot()
    {
        var triangle = new FixedTriangle2d(
            new Vector2d(0, 0),
            new Vector2d(6, 0),
            new Vector2d(0, 6));

        AssertNearlyEqual(Fixed64.Zero, triangle.DistanceSquared(new Vector2d(1, 1)));
        AssertNearlyEqual(new Fixed64(2), triangle.DistanceSquared(new Vector2d(4, 4)));
        AssertNearlyEqual(new Fixed64(9), triangle.DistanceSquared(new Vector2d(2, -3)));
    }

    [Fact]
    public void EqualityDeconstructAndHashCode_UseOrderedVertices()
    {
        var triangle = new FixedTriangle2d(new Vector2d(1, 2), new Vector2d(3, 4), new Vector2d(5, 6));
        var same = new FixedTriangle2d(new Vector2d(1, 2), new Vector2d(3, 4), new Vector2d(5, 6));
        var reordered = new FixedTriangle2d(new Vector2d(3, 4), new Vector2d(5, 6), new Vector2d(1, 2));

        triangle.Deconstruct(out Vector2d a, out Vector2d b, out Vector2d c);

        Assert.Equal(new Vector2d(1, 2), a);
        Assert.Equal(new Vector2d(3, 4), b);
        Assert.Equal(new Vector2d(5, 6), c);
        Assert.True(triangle == same);
        Assert.False(triangle != same);
        Assert.Equal(triangle.GetHashCode(), same.GetHashCode());
        Assert.NotEqual(triangle, reordered);
        Assert.False(triangle == reordered);
        Assert.True(triangle != reordered);
        Assert.False(triangle.Equals("not a triangle"));
    }

    [Fact]
    public void JsonSerialization_RoundTripsState()
    {
        var triangle = new FixedTriangle2d(new Vector2d(1, 2), new Vector2d(3, 4), new Vector2d(5, 6));

        byte[] json = JsonSerializer.SerializeToUtf8Bytes(triangle);
        var roundTrip = JsonSerializer.Deserialize<FixedTriangle2d>(json);

        Assert.Equal(triangle, roundTrip);
    }

#if !FIXEDMATHSHARP_DISABLE_MEMORYPACK
    [Fact]
    public void MemoryPackSerialization_RoundTripsState()
    {
        var triangle = new FixedTriangle2d(new Vector2d(1, 2), new Vector2d(3, 4), new Vector2d(5, 6));

        byte[] bytes = MemoryPackSerializer.Serialize(triangle);
        var roundTrip = MemoryPackSerializer.Deserialize<FixedTriangle2d>(bytes);

        Assert.Equal(triangle, roundTrip);
    }
#endif

    private static void AssertNearlyEqual(Vector2d expected, Vector2d actual)
    {
        Assert.True(
            Vector2d.DistanceSquared(expected, actual) <= Fixed64.Epsilon,
            $"Expected {expected}, actual {actual}.");
    }

    private static void AssertNearlyEqual(Fixed64 expected, Fixed64 actual)
    {
        Fixed64 difference = FixedMath.Abs(expected - actual);
        Assert.True(difference <= Fixed64.Epsilon, $"Expected {expected}, actual {actual}.");
    }
}
