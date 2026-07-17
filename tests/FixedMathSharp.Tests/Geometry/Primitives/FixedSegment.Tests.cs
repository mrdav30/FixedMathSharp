using FixedMathSharp.Bounds;
using MemoryPack;
using System.Numerics;
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
    public void DistanceSquared_SumsExactComponentSquaresBeforeNearestEvenRounding()
    {
        var point = new FixedSegment(Vector3d.Zero, Vector3d.Zero);
        var evenTie = new Vector3d(Fixed64.FromRaw(32_768), Fixed64.FromRaw(32_768), Fixed64.Zero);
        var aboveTie = new Vector3d(Fixed64.FromRaw(32_768), Fixed64.FromRaw(32_768), Fixed64.MinIncrement);
        var oddTie = new Vector3d(Fixed64.FromRaw(65_536), Fixed64.FromRaw(32_768), Fixed64.FromRaw(32_768));

        Assert.Equal(Fixed64.Zero, point.DistanceSquared(evenTie));
        Assert.Equal(Fixed64.MinIncrement, point.DistanceSquared(aboveTie));
        Assert.Equal(Fixed64.FromRaw(2), point.DistanceSquared(oddTie));
    }

    [Fact]
    public void ClosestPointAndDistance_FullDomainCasesMatchBigIntegerOracle()
    {
        long one = Fixed64.One.m_rawValue;
        var fullSpan = new FixedSegment(
            RawVector(long.MinValue, 0, 0),
            RawVector(long.MaxValue, 0, 0));
        var roundedTie = new FixedSegment(Vector3d.Zero, RawVector(1L << 33, 0, 0));
        var degenerateTieStart = RawVector(123, 456, 789);
        var degenerateTie = new FixedSegment(
            degenerateTieStart,
            RawVector(123 + 32_768, 456 + 32_768, 789));

        (FixedSegment Segment, Vector3d Point)[] cases =
        {
            (fullSpan, new Vector3d(Fixed64.Zero, new Fixed64(3), new Fixed64(4))),
            (fullSpan, RawVector(long.MaxValue - 2, 3 * one, 4 * one)),
            (new FixedSegment(RawVector(long.MinValue + 4_096, 0, 0), RawVector(long.MaxValue - 4_096, 0, 0)), RawVector(long.MinValue, 0, 0)),
            (new FixedSegment(RawVector(long.MinValue + 4_096, 0, 0), RawVector(long.MaxValue - 4_096, 0, 0)), RawVector(long.MaxValue, 0, 0)),
            (roundedTie, RawVector(1, one, 0)),
            (roundedTie, RawVector(3, one, 0)),
            (degenerateTie, RawVector(one, 2 * one, 0)),
            (new FixedSegment(Vector3d.Zero, RawVector(32_768, 32_768, 1)), RawVector(32_768, 32_768, 1)),
            (new FixedSegment(RawVector(long.MinValue, long.MinValue, long.MinValue), RawVector(long.MinValue, long.MinValue, long.MinValue)), RawVector(long.MaxValue, long.MaxValue, long.MaxValue)),
        };

        foreach ((FixedSegment segment, Vector3d point) in cases)
        {
            Vector3d expectedPoint = OracleClosestPoint(point, segment);
            Assert.Equal(expectedPoint, segment.ClosestPoint(point));
            Assert.Equal(OracleDistanceSquared(point, expectedPoint), segment.DistanceSquared(point));
        }
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
    public void GetClosestPoints_FullRawSpanCrossing_PreservesExactDeterminant()
    {
        var horizontal = new FixedSegment(
            new Vector3d(Fixed64.MinValue, Fixed64.Zero, Fixed64.Zero),
            new Vector3d(Fixed64.MaxValue, Fixed64.Zero, Fixed64.Zero));
        var vertical = new FixedSegment(
            new Vector3d(Fixed64.Zero, Fixed64.MinValue, Fixed64.Zero),
            new Vector3d(Fixed64.Zero, Fixed64.MaxValue, Fixed64.Zero));
        var intersection = new Vector3d(Fixed64.Zero, Fixed64.Zero, Fixed64.Zero);

        Assert.Equal((intersection, intersection), horizontal.GetClosestPoints(vertical));
    }

    [Fact]
    public void GetClosestPoints_EndpointOnInterior_ReturnsExistingEndpointExactly()
    {
        var contact = new Vector3d(1, 1, 1);
        var endingAtContact = new FixedSegment(new Vector3d(1, 2, 1), contact);
        var containing = new FixedSegment(Vector3d.Zero, new Vector3d(3, 3, 3));

        Assert.Equal((contact, contact), endingAtContact.GetClosestPoints(containing));
        Assert.Equal((contact, contact), containing.GetClosestPoints(endingAtContact));
    }

    [Fact]
    public void GetClosestPoints_FullDomainCasesMatchBigIntegerOracle()
    {
        long one = Fixed64.One.m_rawValue;
        long longDirection = 1L << 36;
        var contact = new Vector3d(1, 1, 1);
        var tinyPointEnd = RawVector(
            contact.X.m_rawValue + 32_768,
            contact.Y.m_rawValue + 32_768,
            contact.Z.m_rawValue);

        (FixedSegment First, FixedSegment Second)[] cases =
        {
            (
                new FixedSegment(RawVector(long.MinValue, 0, 0), RawVector(long.MaxValue, 0, 0)),
                new FixedSegment(RawVector(0, long.MinValue, 0), RawVector(0, long.MaxValue, 0))),
            (
                new FixedSegment(new Vector3d(-4, 0, 0), new Vector3d(4, 0, 0)),
                new FixedSegment(new Vector3d(0, -4, 1), new Vector3d(0, 4, 1))),
            (
                new FixedSegment(Vector3d.Zero, new Vector3d(4, 4, 0)),
                new FixedSegment(new Vector3d(0, 0, 1), new Vector3d(4, 4, 1))),
            (
                new FixedSegment(Vector3d.Zero, RawVector(longDirection, 0, 0)),
                new FixedSegment(RawVector(0, 0, one), RawVector(longDirection, (1L << 16) - 1, one))),
            (
                new FixedSegment(Vector3d.Zero, RawVector(longDirection, 0, 0)),
                new FixedSegment(RawVector(0, 0, one), RawVector(longDirection, 1L << 16, one))),
            (
                new FixedSegment(RawVector(long.MinValue, 0, 0), RawVector(long.MaxValue, 0, 0)),
                new FixedSegment(RawVector(long.MinValue + 4_096, 0, 0), RawVector(long.MaxValue - 4_096, 0, 0))),
            (
                new FixedSegment(Vector3d.Zero, new Vector3d(1, 0, 0)),
                new FixedSegment(new Vector3d(2, -1, 0), new Vector3d(2, 1, 0))),
            (
                new FixedSegment(new Vector3d(2, 0, 0), Vector3d.Zero),
                new FixedSegment(new Vector3d(5, 5, 0), new Vector3d(5, 3, 0))),
            (
                new FixedSegment(contact, contact),
                new FixedSegment(Vector3d.Zero, new Vector3d(3, 3, 3))),
            (
                new FixedSegment(contact, tinyPointEnd),
                new FixedSegment(Vector3d.Zero, new Vector3d(3, 3, 3))),
            (
                new FixedSegment(Vector3d.Zero, RawVector(1L << 33, 0, 0)),
                new FixedSegment(RawVector(1, -one, 0), RawVector(1, one, 0))),
            (
                new FixedSegment(Vector3d.Zero, RawVector(1L << 33, 0, 0)),
                new FixedSegment(RawVector(3, -one, 0), RawVector(3, one, 0))),
            (
                new FixedSegment(new Vector3d(1, 2, 1), contact),
                new FixedSegment(Vector3d.Zero, new Vector3d(3, 3, 3))),
            (
                new FixedSegment(Vector3d.Zero, Vector3d.Right),
                new FixedSegment(new Vector3d(1, 1, 0), new Vector3d(0, 1, 0))),
            (
                new FixedSegment(Vector3d.Zero, Vector3d.Right),
                new FixedSegment(new Vector3d(-3, 1, 0), new Vector3d(-2, 1, 0))),
        };

        foreach ((FixedSegment first, FixedSegment second) in cases)
        {
            AssertClosestPairMatchesOracle(first, second);
            AssertClosestPairMatchesOracle(second, first);
            AssertClosestPairMatchesOracle(
                new FixedSegment(first.End, first.Start),
                new FixedSegment(second.End, second.Start));
        }
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
        Assert.True(segment.Equals((object)same));
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

    private static void AssertClosestPairMatchesOracle(FixedSegment first, FixedSegment second)
    {
        (Vector3d ThisPoint, Vector3d OtherPoint) expected = OracleClosestPoints(first, second);
        (Vector3d ThisPoint, Vector3d OtherPoint) actual = first.GetClosestPoints(second);

        Assert.Equal(expected, actual);
        Assert.Equal(
            ExactSquaredDistance(expected.ThisPoint, expected.OtherPoint),
            ExactSquaredDistance(actual.ThisPoint, actual.OtherPoint));
    }

    private static Vector3d OracleClosestPoint(Vector3d point, FixedSegment segment)
    {
        RawVector3 direction = Difference(segment.End, segment.Start);
        BigInteger denominator = Dot(direction, direction);
        if (denominator <= (BigInteger.One << 31))
            return segment.Start;

        BigInteger numerator = Dot(Difference(point, segment.Start), direction);
        if (numerator <= BigInteger.Zero)
            return segment.Start;
        if (numerator >= denominator)
            return segment.End;

        return InterpolateOracle(segment, RoundUnitRatioRawToEven(numerator, denominator));
    }

    private static (Vector3d ThisPoint, Vector3d OtherPoint) OracleClosestPoints(
        FixedSegment first,
        FixedSegment second)
    {
        RawVector3 firstDirection = Difference(first.End, first.Start);
        RawVector3 secondDirection = Difference(second.End, second.Start);
        BigInteger firstLengthSquared = Dot(firstDirection, firstDirection);
        BigInteger secondLengthSquared = Dot(secondDirection, secondDirection);
        BigInteger degenerateThreshold = BigInteger.One << 31;

        if (firstLengthSquared <= degenerateThreshold)
        {
            if (secondLengthSquared <= degenerateThreshold)
                return (first.Start, second.Start);
            if (PointOnSegmentOracle(first.Start, second))
                return (first.Start, first.Start);
            return (first.Start, OracleClosestPoint(first.Start, second));
        }

        if (secondLengthSquared <= degenerateThreshold)
        {
            if (PointOnSegmentOracle(second.Start, first))
                return (second.Start, second.Start);
            return (OracleClosestPoint(second.Start, first), second.Start);
        }

        RawVector3 startDifference = Difference(first.Start, second.Start);
        BigInteger directionsDot = Dot(firstDirection, secondDirection);
        BigInteger firstDirectionDotDifference = Dot(firstDirection, startDifference);
        BigInteger secondDirectionDotDifference = Dot(secondDirection, startDifference);
        BigInteger determinant = (firstLengthSquared * secondLengthSquared)
            - (directionsDot * directionsDot);
        BigInteger firstNumerator = BigInteger.Zero;
        BigInteger secondNumerator;
        BigInteger secondDenominator;
        long firstParameterRaw;
        bool firstParameterIsClamped;

        if (BigInteger.Abs(determinant) < ((BigInteger)Fixed64.Epsilon.m_rawValue << 96))
        {
            firstParameterRaw = 0L;
            firstParameterIsClamped = true;
            bool useDirectionsDot = directionsDot > secondLengthSquared;
            secondNumerator = useDirectionsDot
                ? firstDirectionDotDifference
                : secondDirectionDotDifference;
            secondDenominator = useDirectionsDot ? directionsDot : secondLengthSquared;
        }
        else
        {
            firstNumerator = (directionsDot * secondDirectionDotDifference)
                - (secondLengthSquared * firstDirectionDotDifference);
            secondNumerator = (firstLengthSquared * secondDirectionDotDifference)
                - (directionsDot * firstDirectionDotDifference);
            secondDenominator = determinant;

            if (firstNumerator < BigInteger.Zero)
            {
                firstParameterRaw = 0L;
                firstParameterIsClamped = true;
                secondNumerator = secondDirectionDotDifference;
                secondDenominator = secondLengthSquared;
            }
            else if (firstNumerator > determinant)
            {
                firstParameterRaw = Fixed64.One.m_rawValue;
                firstParameterIsClamped = true;
                secondNumerator = secondDirectionDotDifference + directionsDot;
                secondDenominator = secondLengthSquared;
            }
            else
            {
                firstParameterRaw = 0L;
                firstParameterIsClamped = false;
            }
        }

        long secondParameterRaw;
        if (secondNumerator < BigInteger.Zero)
        {
            secondParameterRaw = 0L;
            firstParameterRaw = ClampUnitRatioRawToEven(
                -firstDirectionDotDifference,
                firstLengthSquared);
        }
        else if (secondNumerator > secondDenominator)
        {
            secondParameterRaw = Fixed64.One.m_rawValue;
            firstParameterRaw = ClampUnitRatioRawToEven(
                directionsDot - firstDirectionDotDifference,
                firstLengthSquared);
        }
        else
        {
            secondParameterRaw = RoundUnitRatioRawToEven(secondNumerator, secondDenominator);
            if (!firstParameterIsClamped)
                firstParameterRaw = RoundUnitRatioRawToEven(firstNumerator, determinant);
        }

        if (firstParameterRaw == 0L && PointOnSegmentOracle(first.Start, second))
            return (first.Start, first.Start);
        if (firstParameterRaw == Fixed64.One.m_rawValue && PointOnSegmentOracle(first.End, second))
            return (first.End, first.End);
        if (secondParameterRaw == 0L && PointOnSegmentOracle(second.Start, first))
            return (second.Start, second.Start);
        if (secondParameterRaw == Fixed64.One.m_rawValue && PointOnSegmentOracle(second.End, first))
            return (second.End, second.End);

        return (
            InterpolateOracle(first, firstParameterRaw),
            InterpolateOracle(second, secondParameterRaw));
    }

    private static bool PointOnSegmentOracle(Vector3d point, FixedSegment segment)
    {
        RawVector3 direction = Difference(segment.End, segment.Start);
        RawVector3 pointDifference = Difference(point, segment.Start);
        BigInteger lengthSquared = Dot(direction, direction);
        if (lengthSquared.IsZero)
            return Dot(pointDifference, pointDifference).IsZero;

        BigInteger projection = Dot(pointDifference, direction);
        return projection >= BigInteger.Zero
            && projection <= lengthSquared
            && (lengthSquared * Dot(pointDifference, pointDifference))
                - (projection * projection) == BigInteger.Zero;
    }

    private static long ClampUnitRatioRawToEven(BigInteger numerator, BigInteger denominator)
    {
        if (numerator <= BigInteger.Zero)
            return 0L;
        if (numerator >= denominator)
            return Fixed64.One.m_rawValue;
        return RoundUnitRatioRawToEven(numerator, denominator);
    }

    private static long RoundUnitRatioRawToEven(BigInteger numerator, BigInteger denominator)
    {
        if (denominator.Sign < 0)
        {
            numerator = -numerator;
            denominator = -denominator;
        }

        BigInteger quotient = BigInteger.DivRem(
            numerator << FixedMath.SHIFT_AMOUNT_I,
            denominator,
            out BigInteger remainder);
        BigInteger twiceRemainder = remainder << 1;
        if (twiceRemainder > denominator
            || (twiceRemainder == denominator && !quotient.IsEven))
        {
            quotient++;
        }

        return (long)quotient;
    }

    private static Vector3d InterpolateOracle(FixedSegment segment, long parameterRaw)
    {
        return RawVector(
            InterpolateRaw(segment.Start.X.m_rawValue, segment.End.X.m_rawValue, parameterRaw),
            InterpolateRaw(segment.Start.Y.m_rawValue, segment.End.Y.m_rawValue, parameterRaw),
            InterpolateRaw(segment.Start.Z.m_rawValue, segment.End.Z.m_rawValue, parameterRaw));
    }

    private static long InterpolateRaw(long from, long to, long parameterRaw)
    {
        if (parameterRaw <= 0L)
            return from;
        if (parameterRaw >= Fixed64.One.m_rawValue)
            return to;

        BigInteger difference = (BigInteger)to - from;
        BigInteger quotient = BigInteger.DivRem(
            BigInteger.Abs(difference) * parameterRaw,
            BigInteger.One << FixedMath.SHIFT_AMOUNT_I,
            out BigInteger remainder);
        BigInteger raw = difference.Sign < 0 ? from - quotient : from + quotient;
        BigInteger half = BigInteger.One << (FixedMath.SHIFT_AMOUNT_I - 1);
        if (remainder > half || (remainder == half && !raw.IsEven))
            raw += difference.Sign;

        return (long)raw;
    }

    private static Fixed64 OracleDistanceSquared(Vector3d first, Vector3d second)
    {
        BigInteger quotient = BigInteger.DivRem(
            ExactSquaredDistance(first, second),
            BigInteger.One << FixedMath.SHIFT_AMOUNT_I,
            out BigInteger remainder);
        BigInteger half = BigInteger.One << (FixedMath.SHIFT_AMOUNT_I - 1);
        if (remainder > half || (remainder == half && !quotient.IsEven))
            quotient++;

        return quotient > long.MaxValue
            ? Fixed64.MaxValue
            : Fixed64.FromRaw((long)quotient);
    }

    private static BigInteger ExactSquaredDistance(Vector3d first, Vector3d second)
    {
        RawVector3 difference = Difference(first, second);
        return Dot(difference, difference);
    }

    private static RawVector3 Difference(Vector3d end, Vector3d start)
    {
        return new RawVector3(
            (BigInteger)end.X.m_rawValue - start.X.m_rawValue,
            (BigInteger)end.Y.m_rawValue - start.Y.m_rawValue,
            (BigInteger)end.Z.m_rawValue - start.Z.m_rawValue);
    }

    private static BigInteger Dot(RawVector3 left, RawVector3 right)
    {
        return (left.X * right.X) + (left.Y * right.Y) + (left.Z * right.Z);
    }

    private static Vector3d RawVector(long x, long y, long z)
    {
        return new Vector3d(Fixed64.FromRaw(x), Fixed64.FromRaw(y), Fixed64.FromRaw(z));
    }

    private readonly struct RawVector3
    {
        internal readonly BigInteger X;
        internal readonly BigInteger Y;
        internal readonly BigInteger Z;

        internal RawVector3(BigInteger x, BigInteger y, BigInteger z)
        {
            X = x;
            Y = y;
            Z = z;
        }
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
