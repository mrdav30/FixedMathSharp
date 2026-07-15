using FixedMathSharp.Bounds;
using MemoryPack;
using System.Numerics;
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
    public void ClosestPoint_FullDomainProjection_MatchesBigIntegerOracle()
    {
        var segment = new FixedSegment2d(
            new Vector2d(Fixed64.MinValue, Fixed64.Zero),
            new Vector2d(Fixed64.MaxValue, Fixed64.Zero));
        var point = new Vector2d(Fixed64.Zero, Fixed64.MaxValue);

        Assert.Equal(GetClosestPointOracle(segment, point), segment.ClosestPoint(point));
        Assert.Equal(new Vector2d(Fixed64.Zero, Fixed64.Zero), segment.ClosestPoint(point));
    }

    [Fact]
    public void TryGetUniqueIntersection_InteriorCrossing_IsEndpointOrderIndependent()
    {
        var horizontal = new FixedSegment2d(new Vector2d(0, 0), new Vector2d(10, 0));
        var horizontalReversed = new FixedSegment2d(horizontal.End, horizontal.Start);
        var vertical = new FixedSegment2d(new Vector2d(5, -5), new Vector2d(5, 5));
        var verticalReversed = new FixedSegment2d(vertical.End, vertical.Start);

        AssertUniqueIntersection(horizontal, vertical, Fixed64.Half);
        AssertUniqueIntersection(horizontal, verticalReversed, Fixed64.Half);
        AssertUniqueIntersection(horizontalReversed, vertical, Fixed64.Half);
        AssertUniqueIntersection(horizontalReversed, verticalReversed, Fixed64.Half);
    }

    [Fact]
    public void TryGetUniqueIntersection_EndpointTouchAndNonparallelDisjoint_UseClosedSegments()
    {
        var segment = new FixedSegment2d(new Vector2d(0, 0), new Vector2d(4, 0));

        AssertUniqueIntersection(
            segment,
            new FixedSegment2d(new Vector2d(4, 0), new Vector2d(4, 3)),
            Fixed64.One);
        AssertUniqueIntersection(
            new FixedSegment2d(segment.End, segment.Start),
            new FixedSegment2d(new Vector2d(4, 0), new Vector2d(4, 3)),
            Fixed64.Zero);
        AssertNoUniqueIntersection(
            segment,
            new FixedSegment2d(new Vector2d(5, -1), new Vector2d(5, 1)));
    }

    [Fact]
    public void TryGetUniqueIntersection_ParallelAndCollinearCases_DistinguishUniqueTouch()
    {
        var segment = new FixedSegment2d(new Vector2d(0, 0), new Vector2d(4, 0));

        AssertNoUniqueIntersection(
            segment,
            new FixedSegment2d(new Vector2d(0, 1), new Vector2d(4, 1)));
        AssertNoUniqueIntersection(
            segment,
            new FixedSegment2d(new Vector2d(5, 0), new Vector2d(8, 0)));
        AssertUniqueIntersection(
            segment,
            new FixedSegment2d(new Vector2d(4, 0), new Vector2d(8, 0)),
            Fixed64.One);
        AssertNoUniqueIntersection(
            segment,
            new FixedSegment2d(new Vector2d(2, 0), new Vector2d(8, 0)));
        AssertNoUniqueIntersection(segment, segment);
    }

    [Fact]
    public void TryGetUniqueIntersection_PointSegments_FollowExplicitPointSemantics()
    {
        var point = new FixedSegment2d(new Vector2d(2, 0), new Vector2d(2, 0));
        var samePoint = new FixedSegment2d(point.Start, point.Start);
        var distinctPoint = new FixedSegment2d(new Vector2d(3, 0), new Vector2d(3, 0));
        var containing = new FixedSegment2d(new Vector2d(0, 0), new Vector2d(4, 0));

        AssertUniqueIntersection(point, samePoint, Fixed64.Zero);
        AssertNoUniqueIntersection(point, distinctPoint);
        AssertUniqueIntersection(point, containing, Fixed64.Zero);
        AssertUniqueIntersection(containing, point, Fixed64.Half);
        AssertNoUniqueIntersection(
            new FixedSegment2d(new Vector2d(2, 1), new Vector2d(2, 1)),
            containing);
    }

    [Fact]
    public void TryGetUniqueIntersection_CancellingProducts_PreserveSmallestNonzeroDeterminant()
    {
        long n = long.MaxValue;
        var first = new FixedSegment2d(
            Vector2d.Zero,
            new Vector2d(Fixed64.FromRaw(n), Fixed64.FromRaw(n - 1)));
        var second = new FixedSegment2d(
            Vector2d.Zero,
            new Vector2d(Fixed64.FromRaw(n - 1), Fixed64.FromRaw(n - 2)));

        Assert.Equal(BigInteger.MinusOne, CrossRaw(first.Delta, second.Delta));
        AssertUniqueIntersection(first, second, Fixed64.Zero);

        var oneRawHorizontal = new FixedSegment2d(
            Vector2d.Zero,
            new Vector2d(Fixed64.MinIncrement, Fixed64.Zero));
        var oneRawVertical = new FixedSegment2d(
            oneRawHorizontal.End,
            new Vector2d(Fixed64.MinIncrement, Fixed64.MinIncrement));
        Assert.Equal(BigInteger.One, CrossRaw(oneRawHorizontal.Delta, oneRawVertical.Delta));
        AssertUniqueIntersection(oneRawHorizontal, oneRawVertical, Fixed64.One);
    }

    [Fact]
    public void TryGetUniqueIntersection_FullDomainCrossing_MatchesBigIntegerRatioOracle()
    {
        var horizontal = new FixedSegment2d(
            new Vector2d(Fixed64.MinValue, Fixed64.Zero),
            new Vector2d(Fixed64.MaxValue, Fixed64.Zero));
        var vertical = new FixedSegment2d(
            new Vector2d(Fixed64.Zero, Fixed64.MinValue),
            new Vector2d(Fixed64.Zero, Fixed64.MaxValue));
        Fixed64 expected = GetIntersectionParameterOracle(horizontal, vertical);

        Assert.Equal(Fixed64.Half, expected);
        AssertUniqueIntersection(horizontal, vertical, expected);
    }

    [Fact]
    public void GetClosestPoints_CrossingAndCollinearOverlap_ReturnPointsOnBothSegments()
    {
        var horizontal = new FixedSegment2d(new Vector2d(0, 0), new Vector2d(10, 0));
        var vertical = new FixedSegment2d(new Vector2d(5, -5), new Vector2d(5, 5));
        var overlap = new FixedSegment2d(new Vector2d(4, 0), new Vector2d(8, 0));

        Assert.Equal(
            (new Vector2d(5, 0), new Vector2d(5, 0)),
            horizontal.GetClosestPoints(vertical));
        Assert.Equal(
            (new Vector2d(4, 0), new Vector2d(4, 0)),
            horizontal.GetClosestPoints(overlap));
    }

    [Fact]
    public void GetClosestPoints_CollinearOverlap_KeepsFirstSharedEndpointInCandidateOrder()
    {
        var firstStartShared = new FixedSegment2d(new Vector2d(4, 0), new Vector2d(10, 0));
        var firstEndShared = new FixedSegment2d(new Vector2d(0, 0), new Vector2d(6, 0));
        var containingFromLeft = new FixedSegment2d(new Vector2d(0, 0), new Vector2d(8, 0));
        var containingFromRight = new FixedSegment2d(new Vector2d(4, 0), new Vector2d(10, 0));

        Assert.Equal(
            (firstStartShared.Start, firstStartShared.Start),
            firstStartShared.GetClosestPoints(containingFromLeft));
        Assert.Equal(
            (firstEndShared.End, firstEndShared.End),
            firstEndShared.GetClosestPoints(containingFromRight));
    }

    [Fact]
    public void GetClosestPoints_ParallelEqualDistanceTie_KeepsFirstEndpointProjection()
    {
        var first = new FixedSegment2d(new Vector2d(0, 0), new Vector2d(4, 0));
        var firstReversed = new FixedSegment2d(first.End, first.Start);
        var second = new FixedSegment2d(new Vector2d(0, 2), new Vector2d(4, 2));

        Assert.Equal(
            (new Vector2d(0, 0), new Vector2d(0, 2)),
            first.GetClosestPoints(second));
        Assert.Equal(
            (new Vector2d(4, 0), new Vector2d(4, 2)),
            firstReversed.GetClosestPoints(second));
    }

    [Fact]
    public void GetClosestPoints_PointOnSegment_ReturnsExactPointBeforeParameterQuantization()
    {
        var point = new Vector2d(2, 0);
        var pointSegment = new FixedSegment2d(point, point);
        var line = new FixedSegment2d(new Vector2d(0, 0), new Vector2d(6, 0));

        (Vector2d pointResult, Vector2d lineResult) = pointSegment.GetClosestPoints(line);

        Assert.Equal(point.X.m_rawValue, pointResult.X.m_rawValue);
        Assert.Equal(point.X.m_rawValue, lineResult.X.m_rawValue);
        Assert.Equal((point, point), (pointResult, lineResult));
    }

    [Fact]
    public void GetClosestPoints_NonparallelEndpointOnInterior_ReturnsExactContact()
    {
        var contact = new Vector2d(2, 0);
        var endingAtContact = new FixedSegment2d(new Vector2d(2, -2), contact);
        var containing = new FixedSegment2d(new Vector2d(0, 0), new Vector2d(6, 0));

        (Vector2d endpointResult, Vector2d interiorResult) = endingAtContact.GetClosestPoints(containing);

        Assert.Equal(contact.X.m_rawValue, endpointResult.X.m_rawValue);
        Assert.Equal(contact.X.m_rawValue, interiorResult.X.m_rawValue);
        Assert.Equal((contact, contact), (endpointResult, interiorResult));
        Assert.Equal(
            (contact, contact),
            containing.GetClosestPoints(endingAtContact));
    }

    [Fact]
    public void GetClosestPoints_DegenerateSegments_ArePointsSymmetrically()
    {
        var firstPoint = new FixedSegment2d(new Vector2d(2, 3), new Vector2d(2, 3));
        var secondPoint = new FixedSegment2d(new Vector2d(5, 7), new Vector2d(5, 7));
        var horizontal = new FixedSegment2d(new Vector2d(0, 0), new Vector2d(4, 0));

        Assert.Equal((firstPoint.Start, secondPoint.Start), firstPoint.GetClosestPoints(secondPoint));
        Assert.Equal((firstPoint.Start, new Vector2d(2, 0)), firstPoint.GetClosestPoints(horizontal));
        Assert.Equal((new Vector2d(2, 0), firstPoint.Start), horizontal.GetClosestPoints(firstPoint));
    }

    [Fact]
    public void GetClosestPoints_EndpointClamps_AreStableUnderReversedEndpoints()
    {
        var first = new FixedSegment2d(new Vector2d(0, 0), new Vector2d(2, 0));
        var second = new FixedSegment2d(new Vector2d(5, 3), new Vector2d(5, 5));
        var expected = (new Vector2d(2, 0), new Vector2d(5, 3));

        Assert.Equal(expected, first.GetClosestPoints(second));
        Assert.Equal(expected,
            new FixedSegment2d(first.End, first.Start)
                .GetClosestPoints(new FixedSegment2d(second.End, second.Start)));
    }

    [Fact]
    public void GetClosestPoints_ExactDistanceOrdering_DoesNotUseSaturatedPublicDistance()
    {
        var first = new FixedSegment2d(
            new Vector2d(Fixed64.MinValue, Fixed64.Zero),
            new Vector2d(Fixed64.FromRaw(long.MinValue + 100), Fixed64.Zero));
        var second = new FixedSegment2d(
            new Vector2d(Fixed64.MaxValue, Fixed64.Zero),
            new Vector2d(Fixed64.MaxValue, Fixed64.FromRaw(100)));

        Assert.Equal(Fixed64.MaxValue, Vector2d.DistanceSquared(first.Start, second.Start));
        Assert.Equal(Fixed64.MaxValue, Vector2d.DistanceSquared(first.End, second.Start));
        Assert.True(DistanceSquaredRaw(first.End, second.Start) < DistanceSquaredRaw(first.Start, second.Start));
        Assert.Equal((first.End, second.Start), first.GetClosestPoints(second));
    }

    private static void AssertUniqueIntersection(
        FixedSegment2d first,
        FixedSegment2d second,
        Fixed64 expectedParameter)
    {
        Assert.True(first.TryGetUniqueIntersection(second, out Fixed64 parameter));
        Assert.Equal(expectedParameter, parameter);
    }

    private static void AssertNoUniqueIntersection(FixedSegment2d first, FixedSegment2d second)
    {
        Assert.False(first.TryGetUniqueIntersection(second, out Fixed64 parameter));
        Assert.Equal(default, parameter);
    }

    private static Vector2d GetClosestPointOracle(FixedSegment2d segment, Vector2d point)
    {
        BigInteger deltaX = (BigInteger)segment.End.X.m_rawValue - segment.Start.X.m_rawValue;
        BigInteger deltaY = (BigInteger)segment.End.Y.m_rawValue - segment.Start.Y.m_rawValue;
        BigInteger denominator = (deltaX * deltaX) + (deltaY * deltaY);
        if (denominator.IsZero)
            return segment.Start;

        BigInteger pointX = (BigInteger)point.X.m_rawValue - segment.Start.X.m_rawValue;
        BigInteger pointY = (BigInteger)point.Y.m_rawValue - segment.Start.Y.m_rawValue;
        BigInteger numerator = (pointX * deltaX) + (pointY * deltaY);
        if (numerator.Sign <= 0)
            return segment.Start;
        if (numerator >= denominator)
            return segment.End;

        long parameterRaw = RoundRatioToRaw(numerator, denominator);
        return new Vector2d(
            Fixed64.FromRaw(LerpRawToEven(segment.Start.X.m_rawValue, segment.End.X.m_rawValue, parameterRaw)),
            Fixed64.FromRaw(LerpRawToEven(segment.Start.Y.m_rawValue, segment.End.Y.m_rawValue, parameterRaw)));
    }

    private static Fixed64 GetIntersectionParameterOracle(FixedSegment2d first, FixedSegment2d second)
    {
        BigInteger firstX = (BigInteger)first.End.X.m_rawValue - first.Start.X.m_rawValue;
        BigInteger firstY = (BigInteger)first.End.Y.m_rawValue - first.Start.Y.m_rawValue;
        BigInteger secondX = (BigInteger)second.End.X.m_rawValue - second.Start.X.m_rawValue;
        BigInteger secondY = (BigInteger)second.End.Y.m_rawValue - second.Start.Y.m_rawValue;
        BigInteger startX = (BigInteger)second.Start.X.m_rawValue - first.Start.X.m_rawValue;
        BigInteger startY = (BigInteger)second.Start.Y.m_rawValue - first.Start.Y.m_rawValue;
        BigInteger numerator = (startX * secondY) - (startY * secondX);
        BigInteger denominator = (firstX * secondY) - (firstY * secondX);
        return Fixed64.FromRaw(RoundRatioToRaw(numerator, denominator));
    }

    private static BigInteger CrossRaw(Vector2d left, Vector2d right) =>
        ((BigInteger)left.X.m_rawValue * right.Y.m_rawValue)
        - ((BigInteger)left.Y.m_rawValue * right.X.m_rawValue);

    private static BigInteger DistanceSquaredRaw(Vector2d left, Vector2d right)
    {
        BigInteger x = (BigInteger)left.X.m_rawValue - right.X.m_rawValue;
        BigInteger y = (BigInteger)left.Y.m_rawValue - right.Y.m_rawValue;
        return (x * x) + (y * y);
    }

    private static long RoundRatioToRaw(BigInteger numerator, BigInteger denominator)
    {
        BigInteger scaledNumerator = BigInteger.Abs(numerator) << FixedMath.SHIFT_AMOUNT_I;
        BigInteger denominatorMagnitude = BigInteger.Abs(denominator);
        BigInteger quotient = BigInteger.DivRem(scaledNumerator, denominatorMagnitude, out BigInteger remainder);
        int midpointComparison = (remainder << 1).CompareTo(denominatorMagnitude);
        if (midpointComparison > 0 || (midpointComparison == 0 && !quotient.IsEven))
            quotient++;

        return (long)((numerator.Sign < 0) != (denominator.Sign < 0) ? -quotient : quotient);
    }

    private static long LerpRawToEven(long fromRaw, long toRaw, long amountRaw)
    {
        BigInteger numerator = ((BigInteger)fromRaw << FixedMath.SHIFT_AMOUNT_I)
            + ((BigInteger)toRaw - fromRaw) * amountRaw;
        BigInteger denominator = BigInteger.One << FixedMath.SHIFT_AMOUNT_I;
        BigInteger quotient = BigInteger.DivRem(BigInteger.Abs(numerator), denominator, out BigInteger remainder);
        int midpointComparison = (remainder << 1).CompareTo(denominator);
        if (midpointComparison > 0 || (midpointComparison == 0 && !quotient.IsEven))
            quotient++;

        return (long)(numerator.Sign < 0 ? -quotient : quotient);
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
