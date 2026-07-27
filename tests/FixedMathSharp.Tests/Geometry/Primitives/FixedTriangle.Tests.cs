using FixedMathSharp.Geometry;
using MemoryPack;
using System;
using System.Numerics;
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
    public void Centroid_FullRawDomain_UsesIndependentThreeValueAverages()
    {
        Vector3d[] vertices =
        {
            RawVector(long.MinValue, long.MaxValue, -1L),
            RawVector(long.MaxValue, long.MinValue, 0L),
            RawVector(long.MaxValue, long.MaxValue, 2L),
        };

        for (int offset = 0; offset < vertices.Length; offset++)
        {
            var triangle = new FixedTriangle(
                vertices[offset],
                vertices[(offset + 1) % vertices.Length],
                vertices[(offset + 2) % vertices.Length]);

            AssertRawEqual(
                RawVector(
                    AverageRaw(triangle.A.X, triangle.B.X, triangle.C.X),
                    AverageRaw(triangle.A.Y, triangle.B.Y, triangle.C.Y),
                    AverageRaw(triangle.A.Z, triangle.B.Z, triangle.C.Z)),
                triangle.Centroid);
        }
    }

    [Fact]
    public void UnnormalizedNormal_FullRawDomain_MatchesExactCrossOracleForBothWindings()
    {
        const long Scale = 1L << 30;
        FixedTriangle[] triangles =
        {
            new(
                RawVector(long.MinValue, long.MinValue, long.MinValue),
                RawVector(long.MaxValue, long.MaxValue, long.MinValue),
                RawVector(long.MaxValue, long.MinValue, long.MaxValue)),
            new(
                Vector3d.Zero,
                RawVector(long.MaxValue, long.MaxValue - Scale, long.MaxValue - (2 * Scale)),
                RawVector(long.MaxValue - (3 * Scale), long.MaxValue - (4 * Scale), long.MaxValue - (5 * Scale))),
            TriangleFromAxisCross(BigInteger.One << 31),
            TriangleFromAxisCross(3 * (BigInteger.One << 31)),
        };

        foreach (FixedTriangle triangle in triangles)
        {
            AssertRawEqual(UnnormalizedNormalOracle(triangle), triangle.UnnormalizedNormal);
            var reversed = new FixedTriangle(triangle.A, triangle.C, triangle.B);
            AssertRawEqual(UnnormalizedNormalOracle(reversed), reversed.UnnormalizedNormal);
        }
    }

    [Fact]
    public void Area_FullRawDomain_MatchesExactSquareRootAndRoundingOracle()
    {
        BigInteger half = BigInteger.One << 32;
        FixedTriangle[] triangles =
        {
            new(new Vector3d(-3, 5, 7), new Vector3d(11, -2, 13), new Vector3d(4, 17, -19)),
            new(
                RawVector(long.MinValue, long.MinValue, long.MinValue),
                RawVector(long.MaxValue, long.MaxValue, long.MinValue),
                RawVector(long.MaxValue, long.MinValue, long.MaxValue)),
            TriangleFromAxisCross((2 * (BigInteger.One << 33)) + half - 1),
            TriangleFromAxisCross((2 * (BigInteger.One << 33)) + half),
            TriangleFromAxisCross((3 * (BigInteger.One << 33)) + half),
            TriangleFromAxisCross((3 * (BigInteger.One << 33)) + half + 1),
        };

        foreach (FixedTriangle triangle in triangles)
        {
            AssertRawEqual(AreaOracle(triangle), triangle.Area);
            var reversed = new FixedTriangle(triangle.A, triangle.C, triangle.B);
            AssertRawEqual(AreaOracle(reversed), reversed.Area);
        }
    }

    [Fact]
    public void NormalAndDegeneracy_FullRawDomain_MatchExactSquaredCrossOracle()
    {
        BigInteger boundaryComponent = BigInteger.One << 52;
        FixedTriangle[] triangles =
        {
            new(new Vector3d(-3, 5, 7), new Vector3d(11, -2, 13), new Vector3d(4, 17, -19)),
            new(
                RawVector(long.MinValue, long.MinValue, long.MinValue),
                RawVector(long.MaxValue, long.MaxValue, long.MinValue),
                RawVector(long.MaxValue, long.MinValue, long.MaxValue)),
            TriangleFromCrossYZ(boundaryComponent, boundaryComponent - 1),
            TriangleFromCrossYZ(boundaryComponent, boundaryComponent),
            new(
                Vector3d.Zero,
                RawVector((long)boundaryComponent, -1L, 0L),
                RawVector((long)boundaryComponent, 0L, 1L)),
            new(
                RawVector(long.MinValue, long.MinValue, long.MinValue),
                Vector3d.Zero,
                RawVector(long.MaxValue, long.MaxValue, long.MaxValue)),
            new(Vector3d.One, Vector3d.One, Vector3d.One),
        };

        foreach (FixedTriangle triangle in triangles)
        {
            Assert.Equal(IsDegenerateOracle(triangle), triangle.IsDegenerate);
            AssertRawEqual(NormalOracle(triangle), triangle.Normal);

            var reversed = new FixedTriangle(triangle.A, triangle.C, triangle.B);
            Assert.Equal(IsDegenerateOracle(reversed), reversed.IsDegenerate);
            AssertRawEqual(NormalOracle(reversed), reversed.Normal);
        }
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
    public void ContainsProjection_ClassifiesOffPlaneInteriorBoundaryAndExteriorPoints()
    {
        var triangle = new FixedTriangle(
            new Vector3d(1, 2, 3),
            new Vector3d(9, 2, 3),
            new Vector3d(1, 10, 3));

        Assert.True(triangle.ContainsProjection(new Vector3d(3, 6, 13)));
        Assert.True(triangle.ContainsProjection(new Vector3d(5, 6, -20)));
        Assert.False(triangle.ContainsProjection(new Vector3d(0, 3, 3)));
        Assert.False(triangle.ContainsProjection(new Vector3d(9, 10, 3)));
    }

    [Fact]
    public void ContainsProjection_FullRawDomainRetainsExactBarycentricSigns()
    {
        var triangle = new FixedTriangle(
            RawVector(long.MinValue, long.MinValue, 0L),
            RawVector(long.MaxValue, long.MinValue, 0L),
            RawVector(long.MinValue, long.MaxValue, 0L));
        FixedTriangle reversed = new(triangle.A, triangle.C, triangle.B);
        Vector3d interior = RawVector(long.MinValue + 1L, long.MinValue + 1L, long.MaxValue);
        Vector3d exterior = RawVector(long.MaxValue, long.MaxValue, long.MinValue);

        Assert.True(triangle.ContainsProjection(interior));
        Assert.True(reversed.ContainsProjection(interior));
        Assert.False(triangle.ContainsProjection(exterior));
        Assert.False(reversed.ContainsProjection(exterior));
    }

    [Fact]
    public void ContainsProjection_DegenerateTriangleHasNoProjectedFace()
    {
        var line = new FixedTriangle(
            new Vector3d(0, 0, 0),
            new Vector3d(4, 0, 0),
            new Vector3d(8, 0, 0));

        Assert.False(line.ContainsProjection(new Vector3d(2, 10, 0)));
    }

    [Fact]
    public void TryGetProjectedBarycentricWeights_FullRawDomain_MatchesExactGramRatioOracles()
    {
        var extreme = new FixedTriangle(
            RawVector(long.MinValue, long.MinValue, 0L),
            RawVector(long.MaxValue, long.MinValue, 0L),
            RawVector(long.MinValue, long.MaxValue, 0L));
        FixedTriangle reversed = new(extreme.A, extreme.C, extreme.B);
        Vector3d[] extremePoints =
        {
            extreme.A,
            extreme.B,
            extreme.C,
            RawVector(0L, 0L, long.MinValue),
            RawVector(0L, 0L, long.MaxValue),
            RawVector(long.MaxValue, long.MaxValue, 0L),
        };

        foreach (Vector3d point in extremePoints)
        {
            AssertProjectedBarycentricMatchesOracle(extreme, point);
            AssertProjectedBarycentricMatchesOracle(reversed, point);
        }

        var oblique = new FixedTriangle(
            RawVector(long.MinValue + 100, long.MaxValue - 100, -7_000_000_000_000_000_000L),
            RawVector(long.MaxValue - 100, long.MinValue + 100, 6_000_000_000_000_000_000L),
            RawVector(123_456_789L, 4_000_000_000_000_000_000L, long.MaxValue - 123));
        Vector3d obliquePoint = RawVector(
            -5_000_000_000_000_000_000L,
            2_000_000_000_000_000_000L,
            -1_000_000_000_000_000_000L);
        AssertProjectedBarycentricMatchesOracle(oblique, obliquePoint);
        AssertProjectedBarycentricMatchesOracle(new FixedTriangle(oblique.A, oblique.C, oblique.B), obliquePoint);

        long tieDenominator = 1L << 41;
        var tieTriangle = new FixedTriangle(
            Vector3d.Zero,
            RawVector(tieDenominator, 0L, 0L),
            RawVector(0L, 1L, 0L));
        AssertProjectedBarycentricMatchesOracle(tieTriangle, RawVector(256L, 0L, long.MaxValue));
        AssertProjectedBarycentricMatchesOracle(tieTriangle, RawVector(768L, 0L, long.MinValue));

        long shortEdge = 1L << 27;
        var saturating = new FixedTriangle(
            Vector3d.Zero,
            RawVector(shortEdge, 0L, 0L),
            RawVector(0L, shortEdge, 0L));
        AssertProjectedBarycentricMatchesOracle(
            saturating,
            RawVector(long.MaxValue, long.MinValue, long.MaxValue));

        BigInteger boundaryComponent = BigInteger.One << 52;
        FixedTriangle[] boundaryTriangles =
        {
            TriangleFromCrossYZ(boundaryComponent, boundaryComponent - 1),
            TriangleFromCrossYZ(boundaryComponent, boundaryComponent),
            new(
                Vector3d.Zero,
                RawVector((long)boundaryComponent, -1L, 0L),
                RawVector((long)boundaryComponent, 0L, 1L)),
        };
        foreach (FixedTriangle triangle in boundaryTriangles)
            AssertProjectedBarycentricMatchesOracle(triangle, triangle.A);
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
    public void ClosestPoint_FullRawDomain_MatchesExactVoronoiRegionOracle()
    {
        var ordinary = new FixedTriangle(
            new Vector3d(0, 0, 0),
            new Vector3d(6, 0, 0),
            new Vector3d(0, 6, 0));
        Vector3d[] ordinaryPoints =
        {
            new(-2, -1, 0),
            new(8, -1, 0),
            new(-1, 8, 0),
            new(2, -3, 0),
            new(-3, 2, 0),
            new(4, 4, 0),
            new(1, 1, 5),
        };
        foreach (Vector3d point in ordinaryPoints)
            AssertRawEqual(ClosestPointOracle(ordinary, point), ordinary.ClosestPoint(point));

        var extreme = new FixedTriangle(
            RawVector(long.MinValue, long.MinValue, long.MinValue),
            RawVector(long.MaxValue, long.MinValue, long.MinValue),
            RawVector(long.MinValue, long.MaxValue, long.MinValue));
        Vector3d[] extremePoints =
        {
            Vector3d.Zero,
            RawVector(long.MaxValue, long.MaxValue, long.MaxValue),
            RawVector(long.MinValue, 0L, 0L),
        };
        foreach (Vector3d point in extremePoints)
            AssertRawEqual(ClosestPointOracle(extreme, point), extreme.ClosestPoint(point));

        var cancelling = new FixedTriangle(
            RawVector(3076609537861764145L, 7668618164354562626L, -5519951239265643117L),
            RawVector(4259172488818480412L, 3150526339421503291L, 7924857658844154386L),
            RawVector(2890439961318813495L, 6665335116053339516L, -389207196658545195L));
        Vector3d cancellingPoint = RawVector(
            1042687612658448709L,
            4779396958478887438L,
            6457292184727341251L);
        AssertRawEqual(ClosestPointOracle(cancelling, cancellingPoint), cancelling.ClosestPoint(cancellingPoint));
    }

    [Fact]
    public void ClosestPoint_DegenerateFullDomain_UsesExactDistanceOrderingAndStableEdgeTies()
    {
        Fixed64 min = Fixed64.MinValue;
        var saturated = new FixedTriangle(
            new Vector3d(min, min, min),
            new Vector3d(Fixed64.FromRaw(long.MinValue + 1), min, min),
            new Vector3d(min, Fixed64.FromRaw(long.MinValue + 2), min));
        Vector3d query = new(Fixed64.MaxValue, Fixed64.MaxValue, Fixed64.MaxValue);
        Vector3d ab = saturated.GetEdge(0).ClosestPoint(query);
        Vector3d bc = saturated.GetEdge(1).ClosestPoint(query);
        Vector3d ca = saturated.GetEdge(2).ClosestPoint(query);
        Assert.Equal(Fixed64.MaxValue, Vector3d.DistanceSquared(query, ab));
        Assert.Equal(Fixed64.MaxValue, Vector3d.DistanceSquared(query, bc));
        Assert.Equal(Fixed64.MaxValue, Vector3d.DistanceSquared(query, ca));
        Assert.True(DistanceSquaredRaw(query, bc) < DistanceSquaredRaw(query, ab));
        AssertRawEqual(ClosestPointOracle(saturated, query), saturated.ClosestPoint(query));

        var tied = new FixedTriangle(
            Vector3d.Zero,
            RawVector(100_000L, 0L, 0L),
            RawVector(0L, 100_000L, 0L));
        Vector3d tieQuery = RawVector(10_000L, 10_000L, 0L);
        Vector3d first = tied.GetEdge(0).ClosestPoint(tieQuery);
        Vector3d last = tied.GetEdge(2).ClosestPoint(tieQuery);
        Assert.NotEqual(first, last);
        Assert.Equal(DistanceSquaredRaw(tieQuery, first), DistanceSquaredRaw(tieQuery, last));
        Assert.Equal(first, tied.ClosestPoint(tieQuery));
    }

    [Fact]
    public void ContainsAndDistanceSquared_FullRawDomain_MatchExactClosestDistanceOracle()
    {
        var extreme = new FixedTriangle(
            RawVector(long.MinValue, long.MinValue, 0L),
            RawVector(long.MaxValue, long.MinValue, 0L),
            RawVector(long.MinValue, long.MaxValue, 0L));
        Vector3d[] points =
        {
            extreme.GetPoint(Fixed64.Quarter, Fixed64.Quarter),
            RawVector(0L, 0L, long.MaxValue),
            RawVector(long.MaxValue, long.MaxValue, 0L),
        };
        foreach (Vector3d point in points)
        {
            Assert.Equal(DistanceSquaredOracle(extreme, point), extreme.DistanceSquared(point));
            Assert.Equal(ContainsOracle(extreme, point), extreme.Contains(point));
        }

        var unit = new FixedTriangle(Vector3d.Zero, Vector3d.Right, Vector3d.Up);
        long boundary = MaximumRawDistanceAtOrBelow(Fixed64.Epsilon.m_rawValue);
        Vector3d inside = RawVector(Fixed64.Half.m_rawValue, Fixed64.Half.m_rawValue, boundary);
        Vector3d outside = RawVector(Fixed64.Half.m_rawValue, Fixed64.Half.m_rawValue, boundary + 1);
        Assert.Equal(Fixed64.Epsilon, DistanceSquaredOracle(unit, inside));
        Assert.True(unit.Contains(inside));
        Assert.False(unit.Contains(outside));

        var line = new FixedTriangle(
            RawVector(long.MinValue, 0L, 0L),
            RawVector(long.MaxValue, 0L, 0L),
            Vector3d.Zero);
        var pointTriangle = new FixedTriangle(Vector3d.Zero, Vector3d.Zero, Vector3d.Zero);
        Vector3d splitRoundingBoundary = RawVector(46_341L, 46_341L, 1_047_551L);
        Assert.Equal(Fixed64.Epsilon, DistanceSquaredOracle(pointTriangle, splitRoundingBoundary));
        Assert.Equal(
            DistanceSquaredOracle(pointTriangle, splitRoundingBoundary),
            pointTriangle.DistanceSquared(splitRoundingBoundary));
        Assert.True(pointTriangle.Contains(splitRoundingBoundary));
        Assert.Equal(ContainsOracle(line, Vector3d.Zero), line.Contains(Vector3d.Zero));
        Assert.Equal(ContainsOracle(line, Vector3d.Up), line.Contains(Vector3d.Up));
        Assert.Equal(ContainsOracle(pointTriangle, Vector3d.Zero), pointTriangle.Contains(Vector3d.Zero));
        Assert.Equal(ContainsOracle(pointTriangle, Vector3d.One), pointTriangle.Contains(Vector3d.One));
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
        Assert.True(triangle.Equals((object)same));
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

    private static void AssertRawEqual(Vector3d expected, Vector3d actual)
    {
        Assert.True(
            expected == actual,
            $"Expected raw ({expected.X.m_rawValue}, {expected.Y.m_rawValue}, {expected.Z.m_rawValue}), "
            + $"actual ({actual.X.m_rawValue}, {actual.Y.m_rawValue}, {actual.Z.m_rawValue}).");
    }

    private static void AssertRawEqual(Fixed64 expected, Fixed64 actual)
    {
        Assert.True(
            expected == actual,
            $"Expected raw {expected.m_rawValue}, actual {actual.m_rawValue}.");
    }

    private static void AssertProjectedBarycentricMatchesOracle(FixedTriangle triangle, Vector3d point)
    {
        (bool expected, Fixed64 expectedA, Fixed64 expectedB, Fixed64 expectedC) = BarycentricOracle(triangle, point);
        bool actual = triangle.TryGetProjectedBarycentricWeights(
            point,
            out Fixed64 actualA,
            out Fixed64 actualB,
            out Fixed64 actualC);

        Assert.Equal(expected, actual);
        AssertRawEqual(expectedA, actualA);
        AssertRawEqual(expectedB, actualB);
        AssertRawEqual(expectedC, actualC);
    }

    private static (bool Success, Fixed64 A, Fixed64 B, Fixed64 C) BarycentricOracle(
        FixedTriangle triangle,
        Vector3d point)
    {
        (BigInteger X, BigInteger Y, BigInteger Z) ab = Difference(triangle.B, triangle.A);
        (BigInteger X, BigInteger Y, BigInteger Z) ac = Difference(triangle.C, triangle.A);
        (BigInteger X, BigInteger Y, BigInteger Z) ap = Difference(point, triangle.A);
        BigInteger abAb = Dot(ab, ab);
        BigInteger abAc = Dot(ab, ac);
        BigInteger acAc = Dot(ac, ac);
        BigInteger apAb = Dot(ap, ab);
        BigInteger apAc = Dot(ap, ac);
        BigInteger denominator = (abAb * acAc) - (abAc * abAc);
        BigInteger threshold = (BigInteger)Fixed64.Epsilon.m_rawValue << 96;
        if (BigInteger.Abs(denominator) <= threshold)
            return (false, Fixed64.Zero, Fixed64.Zero, Fixed64.Zero);

        BigInteger numeratorB = (acAc * apAb) - (abAc * apAc);
        BigInteger numeratorC = (abAb * apAc) - (abAc * apAb);
        BigInteger numeratorA = denominator - numeratorB - numeratorC;
        return (
            true,
            RoundRatioToFixed(numeratorA, denominator),
            RoundRatioToFixed(numeratorB, denominator),
            RoundRatioToFixed(numeratorC, denominator));
    }

    private static Vector3d ClosestPointOracle(FixedTriangle triangle, Vector3d point)
    {
        if (IsDegenerateOracle(triangle))
            return ClosestPointOnEdgesOracle(triangle, point);

        (BigInteger X, BigInteger Y, BigInteger Z) ab = Difference(triangle.B, triangle.A);
        (BigInteger X, BigInteger Y, BigInteger Z) ac = Difference(triangle.C, triangle.A);
        BigInteger d1 = Dot(ab, Difference(point, triangle.A));
        BigInteger d2 = Dot(ac, Difference(point, triangle.A));
        if (d1 <= 0 && d2 <= 0)
            return triangle.A;

        BigInteger d3 = Dot(ab, Difference(point, triangle.B));
        BigInteger d4 = Dot(ac, Difference(point, triangle.B));
        if (d3 >= 0 && d4 <= d3)
            return triangle.B;

        BigInteger vc = (d1 * d4) - (d3 * d2);
        if (vc <= 0 && d1 >= 0 && d3 <= 0)
            return InterpolateOracle(triangle.A, triangle.B, RoundUnitRatioRawToEven(d1, d1 - d3));

        BigInteger d5 = Dot(ab, Difference(point, triangle.C));
        BigInteger d6 = Dot(ac, Difference(point, triangle.C));
        if (d6 >= 0 && d5 <= d6)
            return triangle.C;

        BigInteger vb = (d5 * d2) - (d1 * d6);
        if (vb <= 0 && d2 >= 0 && d6 <= 0)
            return InterpolateOracle(triangle.A, triangle.C, RoundUnitRatioRawToEven(d2, d2 - d6));

        BigInteger va = (d3 * d6) - (d5 * d4);
        BigInteger bcD4 = d4 - d3;
        BigInteger bcD5 = d5 - d6;
        if (va <= 0 && bcD4 >= 0 && bcD5 >= 0)
            return InterpolateOracle(triangle.B, triangle.C, RoundUnitRatioRawToEven(bcD4, bcD4 + bcD5));

        BigInteger denominator = va + vb + vc;
        if (denominator <= 0)
            return ClosestPointOnEdgesOracle(triangle, point);

        return BarycentricInterpolateOracle(
            triangle,
            RoundUnitRatioRawToEven(vb, denominator),
            RoundUnitRatioRawToEven(vc, denominator));
    }

    private static Vector3d ClosestPointOnEdgesOracle(FixedTriangle triangle, Vector3d point)
    {
        Vector3d best = ClosestPointOnSegmentOracle(triangle.A, triangle.B, point);
        BigInteger bestDistance = DistanceSquaredRaw(point, best);
        Vector3d candidate = ClosestPointOnSegmentOracle(triangle.B, triangle.C, point);
        BigInteger candidateDistance = DistanceSquaredRaw(point, candidate);
        if (candidateDistance < bestDistance)
        {
            best = candidate;
            bestDistance = candidateDistance;
        }

        candidate = ClosestPointOnSegmentOracle(triangle.C, triangle.A, point);
        if (DistanceSquaredRaw(point, candidate) < bestDistance)
            best = candidate;
        return best;
    }

    private static Vector3d ClosestPointOnSegmentOracle(Vector3d start, Vector3d end, Vector3d point)
    {
        (BigInteger X, BigInteger Y, BigInteger Z) direction = Difference(end, start);
        BigInteger denominator = Dot(direction, direction);
        if (denominator <= (BigInteger.One << 31))
            return start;

        BigInteger numerator = Dot(Difference(point, start), direction);
        if (numerator <= 0)
            return start;
        if (numerator >= denominator)
            return end;
        return InterpolateOracle(start, end, RoundUnitRatioRawToEven(numerator, denominator));
    }

    private static Vector3d BarycentricInterpolateOracle(FixedTriangle triangle, long weightBRaw, long weightCRaw) =>
        RawVector(
            BarycentricRaw(triangle.A.X, triangle.B.X, triangle.C.X, weightBRaw, weightCRaw),
            BarycentricRaw(triangle.A.Y, triangle.B.Y, triangle.C.Y, weightBRaw, weightCRaw),
            BarycentricRaw(triangle.A.Z, triangle.B.Z, triangle.C.Z, weightBRaw, weightCRaw));

    private static Vector3d InterpolateOracle(Vector3d start, Vector3d end, long amountRaw) =>
        RawVector(
            InterpolateRaw(start.X, end.X, amountRaw),
            InterpolateRaw(start.Y, end.Y, amountRaw),
            InterpolateRaw(start.Z, end.Z, amountRaw));

    private static long BarycentricRaw(
        Fixed64 a,
        Fixed64 b,
        Fixed64 c,
        long weightBRaw,
        long weightCRaw) =>
        RoundSignedRawToLong(
            ((BigInteger)a.m_rawValue << 32)
            + (((BigInteger)b.m_rawValue - a.m_rawValue) * weightBRaw)
            + (((BigInteger)c.m_rawValue - a.m_rawValue) * weightCRaw),
            32);

    private static long InterpolateRaw(Fixed64 start, Fixed64 end, long amountRaw) =>
        RoundSignedRawToLong(
            ((BigInteger)start.m_rawValue << 32)
            + (((BigInteger)end.m_rawValue - start.m_rawValue) * amountRaw),
            32);

    private static long RoundUnitRatioRawToEven(BigInteger numerator, BigInteger denominator) =>
        RoundRatioToFixed(numerator, denominator).m_rawValue;

    private static Fixed64 RoundRatioToFixed(BigInteger numerator, BigInteger denominator)
    {
        BigInteger quotient = BigInteger.DivRem(
            BigInteger.Abs(numerator) << FixedMath.SHIFT_AMOUNT_I,
            BigInteger.Abs(denominator),
            out BigInteger remainder);
        int midpointComparison = (remainder << 1).CompareTo(BigInteger.Abs(denominator));
        if (midpointComparison > 0 || (midpointComparison == 0 && !quotient.IsEven))
            quotient++;
        if (numerator.Sign != denominator.Sign && numerator.Sign != 0)
            quotient = -quotient;
        if (quotient > long.MaxValue)
            return Fixed64.MaxValue;
        if (quotient < long.MinValue)
            return Fixed64.MinValue;
        return Fixed64.FromRaw((long)quotient);
    }

    private static Vector3d UnnormalizedNormalOracle(FixedTriangle triangle)
    {
        (BigInteger X, BigInteger Y, BigInteger Z) cross = CrossRaw(triangle);
        return new Vector3d(
            RoundSignedRawToFixed(cross.X, 32),
            RoundSignedRawToFixed(cross.Y, 32),
            RoundSignedRawToFixed(cross.Z, 32));
    }

    private static Fixed64 AreaOracle(FixedTriangle triangle)
    {
        BigInteger squaredMagnitude = CrossSquaredMagnitudeRaw(triangle);
        BigInteger root = IntegerSquareRoot(squaredMagnitude);
        BigInteger quotient = root >> 33;
        BigInteger midpoint = (quotient << 33) + (BigInteger.One << 32);
        int midpointComparison = squaredMagnitude.CompareTo(midpoint * midpoint);
        if (midpointComparison > 0 || (midpointComparison == 0 && !quotient.IsEven))
            quotient++;
        return quotient > long.MaxValue ? Fixed64.MaxValue : Fixed64.FromRaw((long)quotient);
    }

    private static Vector3d NormalOracle(FixedTriangle triangle)
    {
        BigInteger squaredMagnitude = CrossSquaredMagnitudeRaw(triangle);
        if (squaredMagnitude <= ((BigInteger)Fixed64.Epsilon.m_rawValue << 96))
            return Vector3d.Zero;

        (BigInteger X, BigInteger Y, BigInteger Z) cross = CrossRaw(triangle);
        return RawVector(
            NormalizedComponentRaw(cross.X, squaredMagnitude),
            NormalizedComponentRaw(cross.Y, squaredMagnitude),
            NormalizedComponentRaw(cross.Z, squaredMagnitude));
    }

    private static long NormalizedComponentRaw(BigInteger component, BigInteger squaredMagnitude)
    {
        if (component.IsZero)
            return 0L;

        BigInteger target = component * component << 64;
        BigInteger low = BigInteger.Zero;
        BigInteger high = BigInteger.One << 32;
        while (low < high)
        {
            BigInteger middle = (low + high + 1) >> 1;
            if ((middle * middle * squaredMagnitude) <= target)
                low = middle;
            else
                high = middle - 1;
        }

        BigInteger doubledMidpoint = (low << 1) + 1;
        int midpointComparison = ((doubledMidpoint * doubledMidpoint) * squaredMagnitude)
            .CompareTo(target << 2);
        if (midpointComparison < 0 || (midpointComparison == 0 && !low.IsEven))
            low++;
        long raw = (long)low;
        return component.Sign < 0 ? -raw : raw;
    }

    private static bool IsDegenerateOracle(FixedTriangle triangle) =>
        CrossSquaredMagnitudeRaw(triangle) <= ((BigInteger)Fixed64.Epsilon.m_rawValue << 96);

    private static BigInteger CrossSquaredMagnitudeRaw(FixedTriangle triangle)
    {
        (BigInteger X, BigInteger Y, BigInteger Z) cross = CrossRaw(triangle);
        return (cross.X * cross.X) + (cross.Y * cross.Y) + (cross.Z * cross.Z);
    }

    private static (BigInteger X, BigInteger Y, BigInteger Z) CrossRaw(FixedTriangle triangle)
    {
        (BigInteger X, BigInteger Y, BigInteger Z) ab = Difference(triangle.B, triangle.A);
        (BigInteger X, BigInteger Y, BigInteger Z) ac = Difference(triangle.C, triangle.A);
        return (
            (ab.Y * ac.Z) - (ab.Z * ac.Y),
            (ab.Z * ac.X) - (ab.X * ac.Z),
            (ab.X * ac.Y) - (ab.Y * ac.X));
    }

    private static (BigInteger X, BigInteger Y, BigInteger Z) Difference(Vector3d left, Vector3d right) =>
        (
            (BigInteger)left.X.m_rawValue - right.X.m_rawValue,
            (BigInteger)left.Y.m_rawValue - right.Y.m_rawValue,
            (BigInteger)left.Z.m_rawValue - right.Z.m_rawValue);

    private static BigInteger Dot(
        (BigInteger X, BigInteger Y, BigInteger Z) left,
        (BigInteger X, BigInteger Y, BigInteger Z) right) =>
        (left.X * right.X) + (left.Y * right.Y) + (left.Z * right.Z);

    private static BigInteger DistanceSquaredRaw(Vector3d first, Vector3d second) =>
        Dot(Difference(first, second), Difference(first, second));

    private static Fixed64 DistanceSquaredOracle(FixedTriangle triangle, Vector3d point) =>
        RoundPositiveSquaredDistance(DistanceSquaredRaw(point, ClosestPointOracle(triangle, point)));

    private static bool ContainsOracle(FixedTriangle triangle, Vector3d point) =>
        DistanceSquaredOracle(triangle, point) <= Fixed64.Epsilon;

    private static Fixed64 RoundPositiveSquaredDistance(BigInteger squaredRaw)
    {
        BigInteger raw = RoundUnsignedRaw(squaredRaw, 32);
        return raw > long.MaxValue ? Fixed64.MaxValue : Fixed64.FromRaw((long)raw);
    }

    private static Fixed64 RoundSignedRawToFixed(BigInteger value, int fractionalBits) =>
        Fixed64.FromRaw(RoundSignedRawToLong(value, fractionalBits));

    private static long RoundSignedRawToLong(BigInteger value, int fractionalBits)
    {
        BigInteger magnitude = RoundUnsignedRaw(BigInteger.Abs(value), fractionalBits);
        if (value.Sign < 0)
            magnitude = -magnitude;
        if (magnitude > long.MaxValue)
            return long.MaxValue;
        if (magnitude < long.MinValue)
            return long.MinValue;
        return (long)magnitude;
    }

    private static BigInteger RoundUnsignedRaw(BigInteger value, int fractionalBits)
    {
        BigInteger divisor = BigInteger.One << fractionalBits;
        BigInteger quotient = BigInteger.DivRem(value, divisor, out BigInteger remainder);
        int midpointComparison = (remainder << 1).CompareTo(divisor);
        if (midpointComparison > 0 || (midpointComparison == 0 && !quotient.IsEven))
            quotient++;
        return quotient;
    }

    private static BigInteger IntegerSquareRoot(BigInteger value)
    {
        if (value.IsZero)
            return BigInteger.Zero;

        BigInteger current = BigInteger.One << (int)((value.GetBitLength() + 1) / 2);
        while (true)
        {
            BigInteger next = (current + (value / current)) >> 1;
            if (next >= current)
                return current;
            current = next;
        }
    }

    private static long MaximumRawDistanceAtOrBelow(long squaredDistanceRaw)
    {
        BigInteger maximumSquared = (((BigInteger)squaredDistanceRaw << 1) + 1) << 31;
        long result = (long)IntegerSquareRoot(maximumSquared);
        while (RoundPositiveSquaredDistance((BigInteger)result * result).m_rawValue > squaredDistanceRaw)
            result--;
        while (RoundPositiveSquaredDistance((BigInteger)(result + 1) * (result + 1)).m_rawValue <= squaredDistanceRaw)
            result++;
        return result;
    }

    private static long AverageRaw(Fixed64 first, Fixed64 second, Fixed64 third)
    {
        BigInteger sum = (BigInteger)first.m_rawValue + second.m_rawValue + third.m_rawValue;
        BigInteger quotient = BigInteger.DivRem(BigInteger.Abs(sum), 3, out BigInteger remainder);
        if (remainder >= 2)
            quotient++;
        return (long)(sum.Sign < 0 ? -quotient : quotient);
    }

    private static FixedTriangle TriangleFromAxisCross(BigInteger cross)
    {
        Assert.InRange(cross, long.MinValue, long.MaxValue);
        return new FixedTriangle(
            Vector3d.Zero,
            RawVector((long)cross, 0L, 0L),
            RawVector(0L, 1L, 0L));
    }

    private static FixedTriangle TriangleFromCrossYZ(BigInteger y, BigInteger z)
    {
        Assert.InRange(y, long.MinValue, long.MaxValue);
        Assert.InRange(z, long.MinValue, long.MaxValue);
        return new FixedTriangle(
            Vector3d.Zero,
            RawVector(1L, 0L, 0L),
            RawVector(0L, (long)z, (long)-y));
    }

    private static Vector3d RawVector(long x, long y, long z) =>
        new(Fixed64.FromRaw(x), Fixed64.FromRaw(y), Fixed64.FromRaw(z));
}
