using FixedMathSharp.Bounds;
using MemoryPack;
using System;
using System.Text.Json;
using Xunit;

namespace FixedMathSharp.Tests.Bounds;

public class FixedTriangleTests
{
    [Fact]
    public void Constructor_AssignsVerticesAndDerivedValues()
    {
        var triangle = new FixedTriangle(
            new Vector3d(0, 0, 0),
            new Vector3d(6, 0, 0),
            new Vector3d(0, 6, 0));

        Assert.Equal(new Vector3d(0, 0, 0), triangle.A);
        Assert.Equal(new Vector3d(6, 0, 0), triangle.B);
        Assert.Equal(new Vector3d(0, 6, 0), triangle.C);
        Assert.Equal(new Vector3d(0, 0, 36), triangle.UnnormalizedNormal);
        Assert.Equal(Vector3d.Forward, triangle.Normal);
        Assert.Equal(new Fixed64(18), triangle.Area);
        Assert.Equal(new Vector3d(2, 2, 0), triangle.Centroid);
        Assert.False(triangle.IsDegenerate);
        Assert.Equal(
            FixedBoundBox.FromMinMax(new Vector3d(0, 0, 0), new Vector3d(6, 6, 0)),
            triangle.Bounds);
    }

    [Fact]
    public void Normal_PreservesVertexWinding()
    {
        var counterClockwise = new FixedTriangle(
            new Vector3d(0, 0, 0),
            new Vector3d(6, 0, 0),
            new Vector3d(0, 6, 0));
        var clockwise = new FixedTriangle(
            new Vector3d(0, 0, 0),
            new Vector3d(0, 6, 0),
            new Vector3d(6, 0, 0));

        Assert.Equal(Vector3d.Forward, counterClockwise.Normal);
        Assert.Equal(Vector3d.Backward, clockwise.Normal);
        Assert.Equal(counterClockwise.Area, clockwise.Area);
    }

    [Fact]
    public void Bounds_NormalizesAllVertices()
    {
        var triangle = new FixedTriangle(
            new Vector3d(5, -2, 7),
            new Vector3d(-1, 3, -4),
            new Vector3d(2, -7, 9));

        Assert.Equal(
            FixedBoundBox.FromMinMax(new Vector3d(-1, -7, -4), new Vector3d(5, 3, 9)),
            triangle.Bounds);
    }

    [Fact]
    public void GetVertexAndGetEdge_ReturnStableOrderedGeometry()
    {
        var triangle = new FixedTriangle(
            new Vector3d(1, 2, 3),
            new Vector3d(5, 2, 3),
            new Vector3d(1, 7, 3));

        Assert.Equal(FixedTriangle.VertexCount, 3);
        Assert.Equal(FixedTriangle.EdgeCount, 3);
        Assert.Equal(new Vector3d(1, 2, 3), triangle.GetVertex(0));
        Assert.Equal(new Vector3d(5, 2, 3), triangle.GetVertex(1));
        Assert.Equal(new Vector3d(1, 7, 3), triangle.GetVertex(2));
        Assert.Equal(new FixedSegment(new Vector3d(1, 2, 3), new Vector3d(5, 2, 3)), triangle.GetEdge(0));
        Assert.Equal(new FixedSegment(new Vector3d(5, 2, 3), new Vector3d(1, 7, 3)), triangle.GetEdge(1));
        Assert.Equal(new FixedSegment(new Vector3d(1, 7, 3), new Vector3d(1, 2, 3)), triangle.GetEdge(2));
        Assert.Throws<ArgumentOutOfRangeException>(() => { _ = triangle.GetVertex(-1); });
        Assert.Throws<ArgumentOutOfRangeException>(() => { _ = triangle.GetVertex(3); });
        Assert.Throws<ArgumentOutOfRangeException>(() => { _ = triangle.GetEdge(-1); });
        Assert.Throws<ArgumentOutOfRangeException>(() => { _ = triangle.GetEdge(3); });
    }

    [Fact]
    public void GetPoint_InterpolatesFromBarycentricWeights()
    {
        var triangle = new FixedTriangle(
            new Vector3d(1, 2, 3),
            new Vector3d(9, 2, 3),
            new Vector3d(1, 10, 3));

        Assert.Equal(new Vector3d(1, 2, 3), triangle.GetPoint(Fixed64.Zero, Fixed64.Zero));
        Assert.Equal(new Vector3d(9, 2, 3), triangle.GetPoint(Fixed64.One, Fixed64.Zero));
        Assert.Equal(new Vector3d(1, 10, 3), triangle.GetPoint(Fixed64.Zero, Fixed64.One));
        Assert.Equal(
            new Vector3d(3, 6, 3),
            triangle.GetPoint(Fixed64.FromDouble(0.25), Fixed64.Half));
    }

    [Fact]
    public void TryGetProjectedBarycentricWeights_ReturnsProjectedWeightsForOffPlanePoint()
    {
        var triangle = new FixedTriangle(
            new Vector3d(1, 2, 3),
            new Vector3d(9, 2, 3),
            new Vector3d(1, 10, 3));

        Assert.True(triangle.TryGetProjectedBarycentricWeights(new Vector3d(3, 6, 13), out Fixed64 weightA, out Fixed64 weightB, out Fixed64 weightC));
        AssertNearlyEqual(Fixed64.FromDouble(0.25), weightA);
        AssertNearlyEqual(Fixed64.FromDouble(0.25), weightB);
        AssertNearlyEqual(Fixed64.Half, weightC);
        AssertNearlyEqual(new Vector3d(3, 6, 3), triangle.GetPoint(weightB, weightC));
    }

    [Fact]
    public void TryGetProjectedBarycentricWeights_ReturnsFalseForDegenerateTriangle()
    {
        var triangle = new FixedTriangle(
            new Vector3d(0, 0, 0),
            new Vector3d(4, 0, 0),
            new Vector3d(8, 0, 0));

        Assert.False(triangle.TryGetProjectedBarycentricWeights(new Vector3d(2, 0, 0), out Fixed64 weightA, out Fixed64 weightB, out Fixed64 weightC));
        Assert.Equal(Fixed64.Zero, weightA);
        Assert.Equal(Fixed64.Zero, weightB);
        Assert.Equal(Fixed64.Zero, weightC);
    }

    [Fact]
    public void Contains_IsBoundaryInclusiveAndPlaneAware()
    {
        var triangle = new FixedTriangle(
            new Vector3d(0, 0, 0),
            new Vector3d(6, 0, 0),
            new Vector3d(0, 6, 0));

        Assert.True(triangle.Contains(new Vector3d(1, 1, 0)));
        Assert.True(triangle.Contains(new Vector3d(3, 0, 0)));
        Assert.True(triangle.Contains(new Vector3d(0, 0, 0)));
        Assert.False(triangle.Contains(new Vector3d(4, 4, 0)));
        Assert.False(triangle.Contains(new Vector3d(1, 1, 1)));
    }

    [Fact]
    public void DegenerateTriangle_ContainsOnlyCollapsedEdges()
    {
        var line = new FixedTriangle(
            new Vector3d(0, 0, 0),
            new Vector3d(4, 0, 0),
            new Vector3d(8, 0, 0));
        var point = new FixedTriangle(
            new Vector3d(2, 2, 2),
            new Vector3d(2, 2, 2),
            new Vector3d(2, 2, 2));

        Assert.True(line.IsDegenerate);
        Assert.Equal(Fixed64.Zero, line.Area);
        Assert.Equal(Vector3d.Zero, line.Normal);
        Assert.True(line.Contains(new Vector3d(2, 0, 0)));
        Assert.True(line.Contains(new Vector3d(6, 0, 0)));
        Assert.False(line.Contains(new Vector3d(5, 1, 0)));

        Assert.True(point.IsDegenerate);
        Assert.True(point.Contains(new Vector3d(2, 2, 2)));
        Assert.False(point.Contains(new Vector3d(2, 3, 2)));
    }

    [Fact]
    public void ClosestPoint_ReturnsInteriorProjectionOrNearestEdgePoint()
    {
        var triangle = new FixedTriangle(
            new Vector3d(0, 0, 0),
            new Vector3d(6, 0, 0),
            new Vector3d(0, 6, 0));

        AssertNearlyEqual(new Vector3d(1, 1, 0), triangle.ClosestPoint(new Vector3d(1, 1, 0)));
        AssertNearlyEqual(new Vector3d(1, 1, 0), triangle.ClosestPoint(new Vector3d(1, 1, 5)));
        AssertNearlyEqual(new Vector3d(3, 3, 0), triangle.ClosestPoint(new Vector3d(4, 4, 0)));
        AssertNearlyEqual(new Vector3d(2, 0, 0), triangle.ClosestPoint(new Vector3d(2, -3, 0)));
        AssertNearlyEqual(new Vector3d(0, 0, 0), triangle.ClosestPoint(new Vector3d(-2, -1, 0)));
    }

    [Fact]
    public void ClosestPoint_DegenerateTriangle_UsesClosestCollapsedEdge()
    {
        var line = new FixedTriangle(
            new Vector3d(0, 0, 0),
            new Vector3d(4, 0, 0),
            new Vector3d(8, 0, 0));
        var point = new FixedTriangle(
            new Vector3d(2, 2, 2),
            new Vector3d(2, 2, 2),
            new Vector3d(2, 2, 2));

        AssertNearlyEqual(new Vector3d(5, 0, 0), line.ClosestPoint(new Vector3d(5, 3, 2)));
        AssertNearlyEqual(new Vector3d(2, 2, 2), point.ClosestPoint(new Vector3d(5, 6, 7)));
    }

    [Fact]
    public void DistanceSquared_UsesClosestPointWithoutSquareRoot()
    {
        var triangle = new FixedTriangle(
            new Vector3d(0, 0, 0),
            new Vector3d(6, 0, 0),
            new Vector3d(0, 6, 0));

        AssertNearlyEqual(Fixed64.Zero, triangle.DistanceSquared(new Vector3d(1, 1, 0)));
        AssertNearlyEqual(new Fixed64(25), triangle.DistanceSquared(new Vector3d(1, 1, 5)));
        AssertNearlyEqual(new Fixed64(2), triangle.DistanceSquared(new Vector3d(4, 4, 0)));
        AssertNearlyEqual(new Fixed64(9), triangle.DistanceSquared(new Vector3d(2, -3, 0)));
    }

    [Fact]
    public void EqualityDeconstructAndHashCode_UseOrderedVertices()
    {
        var triangle = new FixedTriangle(new Vector3d(1, 2, 3), new Vector3d(4, 5, 6), new Vector3d(7, 8, 9));
        var same = new FixedTriangle(new Vector3d(1, 2, 3), new Vector3d(4, 5, 6), new Vector3d(7, 8, 9));
        var reordered = new FixedTriangle(new Vector3d(4, 5, 6), new Vector3d(7, 8, 9), new Vector3d(1, 2, 3));

        triangle.Deconstruct(out Vector3d a, out Vector3d b, out Vector3d c);

        Assert.Equal(new Vector3d(1, 2, 3), a);
        Assert.Equal(new Vector3d(4, 5, 6), b);
        Assert.Equal(new Vector3d(7, 8, 9), c);
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
        var triangle = new FixedTriangle(new Vector3d(1, 2, 3), new Vector3d(4, 5, 6), new Vector3d(7, 8, 9));

        byte[] json = JsonSerializer.SerializeToUtf8Bytes(triangle);
        var roundTrip = JsonSerializer.Deserialize<FixedTriangle>(json);

        Assert.Equal(triangle, roundTrip);
    }

#if !FIXEDMATHSHARP_DISABLE_MEMORYPACK
    [Fact]
    public void MemoryPackSerialization_RoundTripsState()
    {
        var triangle = new FixedTriangle(new Vector3d(1, 2, 3), new Vector3d(4, 5, 6), new Vector3d(7, 8, 9));

        byte[] bytes = MemoryPackSerializer.Serialize(triangle);
        var roundTrip = MemoryPackSerializer.Deserialize<FixedTriangle>(bytes);

        Assert.Equal(triangle, roundTrip);
    }
#endif

    private static void AssertNearlyEqual(Vector3d expected, Vector3d actual)
    {
        Assert.True(
            Vector3d.DistanceSquared(expected, actual) <= Fixed64.Epsilon,
            $"Expected {expected}, actual {actual}.");
    }

    private static void AssertNearlyEqual(Fixed64 expected, Fixed64 actual)
    {
        Fixed64 difference = FixedMath.Abs(expected - actual);
        Assert.True(difference <= Fixed64.Epsilon, $"Expected {expected}, actual {actual}.");
    }
}
