using System;
using System.Numerics;
using System.Text.Json;
using FixedMathSharp.Geometry;
using MemoryPack;
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
    public void Centroid_FullRawDomain_UsesIndependentThreeValueAverages()
    {
        Vector2d[] vertices =
        {
            new(Fixed64.MinValue, Fixed64.MaxValue),
            new(Fixed64.MaxValue, Fixed64.MinValue),
            new(Fixed64.MaxValue, Fixed64.MaxValue),
        };

        for (int offset = 0; offset < vertices.Length; offset++)
        {
            var triangle = new FixedTriangle2d(
                vertices[offset],
                vertices[(offset + 1) % vertices.Length],
                vertices[(offset + 2) % vertices.Length]);

            Assert.Equal(
                new Vector2d(
                    Fixed64.FromRaw(AverageRaw(triangle.A.X, triangle.B.X, triangle.C.X)),
                    Fixed64.FromRaw(AverageRaw(triangle.A.Y, triangle.B.Y, triangle.C.Y))),
                triangle.Centroid);
        }
    }

    [Fact]
    public void SignedArea_HighWordAndRoundingOverflow_SaturatesByWinding()
    {
        var triangle = new FixedTriangle2d(
            RawVector(0L, 0L),
            RawVector(long.MaxValue, -12884901888L),
            RawVector(1L, 17179869184L));
        var reversed = new FixedTriangle2d(triangle.A, triangle.C, triangle.B);

        Assert.Equal(((BigInteger.One << 64) - 1) * (BigInteger.One << 33) + (BigInteger.One << 32), CrossRaw(triangle.A, triangle.B, triangle.C));
        Assert.Equal(Fixed64.MaxValue, triangle.SignedArea);
        Assert.Equal(Fixed64.MaxValue, triangle.Area);
        Assert.Equal(Fixed64.MinValue, reversed.SignedArea);
        Assert.Equal(Fixed64.MaxValue, reversed.Area);
    }

    [Fact]
    public void SignedAreaAndDegeneracy_FullRawDomain_MatchExactCrossOracle()
    {
        var extreme = new FixedTriangle2d(
            new Vector2d(Fixed64.MinValue, Fixed64.MinValue),
            new Vector2d(Fixed64.MaxValue, Fixed64.MinValue),
            new Vector2d(Fixed64.MinValue, Fixed64.MaxValue));
        var reversed = new FixedTriangle2d(extreme.A, extreme.C, extreme.B);
        Assert.Equal(Fixed64.MaxValue, extreme.SignedArea);
        Assert.Equal(Fixed64.MinValue, reversed.SignedArea);
        Assert.Equal(Fixed64.MaxValue, extreme.Area);
        Assert.Equal(Fixed64.MaxValue, reversed.Area);

        var cancelling = new FixedTriangle2d(
            new Vector2d(Fixed64.MinValue, Fixed64.MinValue),
            new Vector2d(Fixed64.MaxValue, Fixed64.FromRaw(long.MaxValue - 1)),
            new Vector2d(Fixed64.FromRaw(long.MaxValue - 1), Fixed64.FromRaw(long.MaxValue - 2)));
        Assert.Equal(BigInteger.MinusOne, CrossRaw(cancelling.A, cancelling.B, cancelling.C));
        Assert.Equal(Fixed64.Zero, cancelling.SignedArea);

        Assert.Equal(Fixed64.Zero, TriangleFromCross(BigInteger.One << 32).SignedArea);
        Assert.Equal(Fixed64.FromRaw(2), TriangleFromCross(3 * (BigInteger.One << 32)).SignedArea);
        Assert.Equal(Fixed64.Zero, TriangleFromCross(-(BigInteger.One << 32)).SignedArea);
        Assert.Equal(Fixed64.FromRaw(-2), TriangleFromCross(-3 * (BigInteger.One << 32)).SignedArea);

        for (long areaRaw = FixedMath.DEFAULT_TOLERANCE_L - 1;
             areaRaw <= FixedMath.DEFAULT_TOLERANCE_L + 1;
             areaRaw++)
        {
            FixedTriangle2d triangle = TriangleFromCross((BigInteger)areaRaw << 33);
            Assert.Equal(Fixed64.FromRaw(areaRaw), triangle.SignedArea);
            Assert.Equal(areaRaw <= FixedMath.DEFAULT_TOLERANCE_L, triangle.IsDegenerate);
        }
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
    public void TryGetBarycentricWeights_FullRawDomain_MatchesIndependentRatioOracles()
    {
        var extreme = new FixedTriangle2d(
            new Vector2d(Fixed64.MinValue, Fixed64.MinValue),
            new Vector2d(Fixed64.MaxValue, Fixed64.MinValue),
            new Vector2d(Fixed64.MinValue, Fixed64.MaxValue));
        AssertBarycentricMatchesOracle(extreme, Vector2d.Zero);
        AssertBarycentricMatchesOracle(new FixedTriangle2d(extreme.A, extreme.C, extreme.B), Vector2d.Zero);
        AssertBarycentricMatchesOracle(extreme, new Vector2d(Fixed64.MaxValue, Fixed64.MaxValue));

        long exactTieDenominator = 1L << 41;
        var tieTriangle = new FixedTriangle2d(
            Vector2d.Zero,
            new Vector2d(Fixed64.FromRaw(exactTieDenominator), Fixed64.Zero),
            new Vector2d(Fixed64.Zero, Fixed64.MinIncrement));
        AssertBarycentricMatchesOracle(tieTriangle, new Vector2d(Fixed64.FromRaw(256), Fixed64.Zero));
        AssertBarycentricMatchesOracle(tieTriangle, new Vector2d(Fixed64.FromRaw(768), Fixed64.Zero));

        BigInteger threshold = (BigInteger)Fixed64.Epsilon.m_rawValue << FixedMath.SHIFT_AMOUNT_I;
        foreach (BigInteger denominator in new[] { threshold - 1, threshold, threshold + 1 })
        {
            FixedTriangle2d triangle = TriangleFromCross(denominator);
            bool actual = triangle.TryGetBarycentricWeights(
                triangle.A,
                out Fixed64 weightA,
                out Fixed64 weightB,
                out Fixed64 weightC);
            bool expected = denominator > threshold;
            Assert.Equal(expected, actual);
            if (!expected)
            {
                Assert.Equal(Fixed64.Zero, weightA);
                Assert.Equal(Fixed64.Zero, weightB);
                Assert.Equal(Fixed64.Zero, weightC);
            }
        }
    }

    [Fact]
    public void TryGetBarycentricWeights_PositiveRoundingCarry_SaturatesWeight()
    {
        var triangle = new FixedTriangle2d(
            RawVector(0L, 0L),
            RawVector(4294967296L, 0L),
            RawVector(1L, 4294967296L));
        Vector2d point = RawVector(long.MaxValue, -2147483648L);

        Assert.True(triangle.TryGetBarycentricWeights(point, out _, out Fixed64 weightB, out _));
        Assert.Equal(Fixed64.MaxValue, weightB);
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
    public void Contains_FullRawDomain_PreservesWindingBoundariesAndCollapsedGeometry()
    {
        var extreme = new FixedTriangle2d(
            new Vector2d(Fixed64.MinValue, Fixed64.MinValue),
            new Vector2d(Fixed64.MaxValue, Fixed64.MinValue),
            new Vector2d(Fixed64.MinValue, Fixed64.MaxValue));
        Vector2d interior = new(Fixed64.FromRaw(-1), Fixed64.FromRaw(-1));
        Assert.True(extreme.Contains(interior));
        Assert.True(new FixedTriangle2d(extreme.A, extreme.C, extreme.B).Contains(interior));
        Assert.False(extreme.Contains(new Vector2d(Fixed64.MaxValue, Fixed64.MaxValue)));

        var unit = new FixedTriangle2d(Vector2d.Zero, Vector2d.Right, Vector2d.Forward);
        Assert.True(unit.Contains(new Vector2d(Fixed64.Zero, -Fixed64.Epsilon)));
        Assert.False(unit.Contains(new Vector2d(Fixed64.Zero, -Fixed64.Epsilon - Fixed64.MinIncrement)));

        var line = new FixedTriangle2d(
            new Vector2d(Fixed64.MinValue, Fixed64.Zero),
            new Vector2d(Fixed64.MaxValue, Fixed64.Zero),
            Vector2d.Zero);
        Assert.True(line.Contains(Vector2d.Zero));
        Assert.False(line.Contains(new Vector2d(Fixed64.Zero, Fixed64.One)));
        var point = new FixedTriangle2d(Vector2d.Zero, Vector2d.Zero, Vector2d.Zero);
        Assert.True(point.Contains(Vector2d.Zero));
        Assert.False(point.Contains(Vector2d.One));
    }

    [Fact]
    public void Contains_FullRawDomain_MatchesExactOrientationOracle()
    {
        var triangle = new FixedTriangle2d(
            RawVector(3076609537861764145L, 7668618164354562626L),
            RawVector(4259172488818480412L, 3150526339421503291L),
            RawVector(2890439961318813495L, 6665335116053339516L));
        Vector2d point = RawVector(1042687612658448709L, 4779396958478887438L);

        Assert.False(ContainsOracle(triangle, point));
        Assert.False(triangle.Contains(point));
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
    public void ClosestPoint_UsesExactDistanceOrderingAndKeepsFirstTie()
    {
        Fixed64 min = Fixed64.MinValue;
        var saturated = new FixedTriangle2d(
            new Vector2d(min, min),
            new Vector2d(Fixed64.FromRaw(long.MinValue + 1), min),
            new Vector2d(min, Fixed64.FromRaw(long.MinValue + 2)));
        Vector2d query = new(Fixed64.MaxValue, Fixed64.MaxValue);
        Vector2d ab = saturated.GetEdge(0).ClosestPoint(query);
        Vector2d bc = saturated.GetEdge(1).ClosestPoint(query);
        Vector2d ca = saturated.GetEdge(2).ClosestPoint(query);
        Assert.Equal(Fixed64.MaxValue, Vector2d.DistanceSquared(query, ab));
        Assert.Equal(Fixed64.MaxValue, Vector2d.DistanceSquared(query, bc));
        Assert.Equal(Fixed64.MaxValue, Vector2d.DistanceSquared(query, ca));
        Assert.True(DistanceSquaredRaw(query, bc) < DistanceSquaredRaw(query, ab));
        Assert.Equal(bc, saturated.ClosestPoint(query));

        var tied = new FixedTriangle2d(Vector2d.Zero, new Vector2d(2, 0), new Vector2d(0, 2));
        Vector2d tieQuery = new(-1, -1);
        Vector2d first = tied.GetEdge(0).ClosestPoint(tieQuery);
        Assert.Equal(DistanceSquaredRaw(tieQuery, first), DistanceSquaredRaw(tieQuery, tied.GetEdge(2).ClosestPoint(tieQuery)));
        Assert.Equal(first, tied.ClosestPoint(tieQuery));
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

    private static FixedTriangle2d TriangleFromCross(BigInteger cross)
    {
        Assert.InRange(cross, long.MinValue, long.MaxValue);
        return new FixedTriangle2d(
            Vector2d.Zero,
            new Vector2d(Fixed64.FromRaw((long)cross), Fixed64.Zero),
            new Vector2d(Fixed64.Zero, Fixed64.MinIncrement));
    }

    private static void AssertBarycentricMatchesOracle(FixedTriangle2d triangle, Vector2d point)
    {
        BigInteger denominator = CrossRaw(triangle.A, triangle.B, triangle.C);
        Assert.True(BigInteger.Abs(denominator) > ((BigInteger)Fixed64.Epsilon.m_rawValue << FixedMath.SHIFT_AMOUNT_I));
        Assert.True(triangle.TryGetBarycentricWeights(point, out Fixed64 actualA, out Fixed64 actualB, out Fixed64 actualC));
        Assert.Equal(RoundRatioToFixed(CrossRaw(point, triangle.B, triangle.C), denominator), actualA);
        Assert.Equal(RoundRatioToFixed(CrossRaw(point, triangle.C, triangle.A), denominator), actualB);
        Assert.Equal(RoundRatioToFixed(CrossRaw(point, triangle.A, triangle.B), denominator), actualC);
    }

    private static BigInteger CrossRaw(Vector2d origin, Vector2d first, Vector2d second)
    {
        BigInteger firstX = (BigInteger)first.X.m_rawValue - origin.X.m_rawValue;
        BigInteger firstY = (BigInteger)first.Y.m_rawValue - origin.Y.m_rawValue;
        BigInteger secondX = (BigInteger)second.X.m_rawValue - origin.X.m_rawValue;
        BigInteger secondY = (BigInteger)second.Y.m_rawValue - origin.Y.m_rawValue;
        return (firstX * secondY) - (firstY * secondX);
    }

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

    private static BigInteger DistanceSquaredRaw(Vector2d first, Vector2d second)
    {
        BigInteger x = (BigInteger)first.X.m_rawValue - second.X.m_rawValue;
        BigInteger y = (BigInteger)first.Y.m_rawValue - second.Y.m_rawValue;
        return (x * x) + (y * y);
    }

    private static long AverageRaw(Fixed64 first, Fixed64 second, Fixed64 third)
    {
        BigInteger sum = (BigInteger)first.m_rawValue + second.m_rawValue + third.m_rawValue;
        BigInteger quotient = BigInteger.DivRem(BigInteger.Abs(sum), 3, out BigInteger remainder);
        if (remainder >= 2)
            quotient++;
        return (long)(sum.Sign < 0 ? -quotient : quotient);
    }

    private static bool ContainsOracle(FixedTriangle2d triangle, Vector2d point)
    {
        BigInteger area = CrossRaw(triangle.A, triangle.B, triangle.C);
        BigInteger areaThreshold = (BigInteger)Fixed64.Epsilon.m_rawValue << 33;
        if (BigInteger.Abs(area) <= areaThreshold)
        {
            return triangle.GetEdge(0).DistanceSquared(point) <= Fixed64.Epsilon
                || triangle.GetEdge(1).DistanceSquared(point) <= Fixed64.Epsilon
                || triangle.GetEdge(2).DistanceSquared(point) <= Fixed64.Epsilon;
        }

        BigInteger edgeThreshold = (BigInteger)Fixed64.Epsilon.m_rawValue << FixedMath.SHIFT_AMOUNT_I;
        BigInteger ab = CrossRaw(triangle.A, triangle.B, point);
        BigInteger bc = CrossRaw(triangle.B, triangle.C, point);
        BigInteger ca = CrossRaw(triangle.C, triangle.A, point);
        return area.Sign > 0
            ? ab >= -edgeThreshold && bc >= -edgeThreshold && ca >= -edgeThreshold
            : ab <= edgeThreshold && bc <= edgeThreshold && ca <= edgeThreshold;
    }

    private static Vector2d RawVector(long x, long y) =>
        new(Fixed64.FromRaw(x), Fixed64.FromRaw(y));
}
