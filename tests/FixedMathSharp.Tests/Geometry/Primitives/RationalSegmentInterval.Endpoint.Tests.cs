using FixedMathSharp.Geometry;
using System;
using System.Numerics;
using Xunit;

namespace FixedMathSharp.Tests.Bounds;

public sealed class RationalSegmentIntervalEndpointTests
{
    [Fact]
    public void CompleteInterval_ShouldReturnTheOriginalEndpoints()
    {
        var edge = new FixedSegment2d(new(Fixed64.MinValue, Fixed64.MaxValue), new(Fixed64.MaxValue, Fixed64.MinValue));
        RationalSegmentInterval interval = default;
        Assert.Equal(edge.Start, interval.GetEndpoint(edge, upper: false));
        Assert.Equal(edge.End, interval.GetEndpoint(edge, upper: true));
    }

    [Theory]
    [InlineData(0L, 1L, 0L)]
    [InlineData(1L, 2L, 2L)]
    [InlineData(-1L, 0L, 0L)]
    [InlineData(-2L, -1L, -2L)]
    [InlineData(long.MinValue, long.MaxValue, 0L)]
    [InlineData(long.MaxValue, long.MaxValue, long.MaxValue)]
    [InlineData(long.MinValue, long.MinValue, long.MinValue)]
    public void HalfParameter_ShouldRoundTheFinalCoordinateToEvenWithoutRoundingTheParameter(
        long firstRaw, long secondRaw, long expectedRaw)
    {
        var interval = new RationalSegmentInterval(Integer(1), Integer(2), Integer(1), Integer(2));
        var edge = new FixedSegment2d(new(Fixed64.FromRaw(firstRaw), Fixed64.FromRaw(secondRaw)),
            new(Fixed64.FromRaw(secondRaw), Fixed64.FromRaw(firstRaw)));
        Vector2d expected = new(Fixed64.FromRaw(expectedRaw), Fixed64.FromRaw(expectedRaw));
        Assert.Equal(expected, interval.GetEndpoint(edge, upper: false));
        Assert.Equal(expected, interval.GetEndpoint(edge, upper: true));
    }

    [Fact]
    public void LargeOddDenominators_ShouldMaterializeOnceWithoutCollapsingTheAuthoritativeInterval()
    {
        Signed320 denominator = new(1UL << 62, 0, 0, 0, 1);
        Signed320 lower = new(1UL << 61, 0, 0, 0, 1), upper = new(1UL << 61, 0, 0, 0, 2);
        var interval = new RationalSegmentInterval(lower, denominator, upper, denominator);
        var edge = new FixedSegment2d(new(Fixed64.MinValue, Fixed64.MaxValue), new(Fixed64.MaxValue, Fixed64.MinValue));
        Assert.True(RationalSegmentInterval.CompareParameters(interval.LowerNumerator, interval.LowerDenominator,
            interval.UpperNumerator, interval.UpperDenominator) < 0);
        Assert.Equal(new Vector2d(Fixed64.Zero, Fixed64.FromRaw(-1)), interval.GetEndpoint(edge, upper: false));
        Assert.Equal(interval.GetEndpoint(edge, upper: false), interval.GetEndpoint(edge, upper: true));

        BigInteger d = (BigInteger.One << 318) + 1;
        foreach (bool useUpper in new[] { false, true })
        {
            BigInteger n = (BigInteger.One << 317) + (useUpper ? 2 : 1);
            Vector2d point = interval.GetEndpoint(edge, useUpper);
            Assert.Equal(RoundEven((BigInteger)long.MinValue * d + ((BigInteger)long.MaxValue - long.MinValue) * n, d), point.X.m_rawValue);
            Assert.Equal(RoundEven((BigInteger)long.MaxValue * d + ((BigInteger)long.MinValue - long.MaxValue) * n, d), point.Y.m_rawValue);
        }
    }

    [Fact]
    public void FinalLocalEndpoint_ShouldNotRequireARepresentableWorldPoint()
    {
        var interval = new RationalSegmentInterval(Integer(3), Integer(5), Integer(4), Integer(5));
        Vector2d point = interval.GetEndpoint(new FixedSegment2d(Vector2d.Zero, new(2, 0)), upper: false);
        Assert.Equal(new Vector2d(Fixed64.FromFraction(6, 5), Fixed64.Zero), point);
        var anchor = new FixedPointAnchor2d(new(Fixed64.MaxValue, Fixed64.Zero), Fixed64.Zero, point);
        Assert.False(anchor.TryGetPoint(out _));
        Assert.Equal(point, anchor.LocalPoint);
    }

    [Theory]
    [InlineData(false, false, 0)]
    [InlineData(false, true, 0)]
    [InlineData(true, false, 0)]
    [InlineData(true, true, 0)]
    [InlineData(false, false, 1)]
    [InlineData(false, true, 1)]
    [InlineData(true, false, 1)]
    [InlineData(true, true, 1)]
    [InlineData(false, false, 2)]
    [InlineData(false, true, 2)]
    [InlineData(true, false, 2)]
    [InlineData(true, true, 2)]
    public void ClippedInterval_ShouldMapOntoACollinearSubedgeBeforeRounding(bool reverseSource, bool reverseTarget, int axis)
    {
        // The physical retained interval is raw coordinates [3,5]. Its target
        // parameters are not representable in Q32.32 and must stay exact.
        var interval = new RationalSegmentInterval(Integer(reverseSource ? 2 : 3), Integer(7),
            Integer(reverseSource ? 4 : 5), Integer(7));
        var source = new FixedSegment(Point(reverseSource ? 7 : 0), Point(reverseSource ? 0 : 7));
        var target = new FixedSegment(Point(reverseTarget ? 6 : 1), Point(reverseTarget ? 1 : 6));
        Assert.True(interval.TryMapToSegment(source, target, out var mapped));
        Assert.True(mapped.IsClipped);
        Assert.Equal(0, RationalSegmentInterval.CompareParameters(mapped.LowerNumerator, mapped.LowerDenominator,
            Integer(reverseTarget ? 1 : 2), Integer(5)));
        Assert.Equal(0, RationalSegmentInterval.CompareParameters(mapped.UpperNumerator, mapped.UpperDenominator,
            Integer(reverseTarget ? 3 : 4), Integer(5)));

        Vector3d Point(long raw)
        {
            Vector3d point = new(11, -13, 17);
            point[axis] = Fixed64.FromRaw(raw);
            return point;
        }
    }

    [Theory]
    [InlineData(-2, 2, 1, 2, 1, 1)]
    [InlineData(4, 8, 0, 1, 0, 1)]
    [InlineData(-4, 0, 1, 1, 1, 1)]
    [InlineData(1, 3, 0, 1, 1, 1)]
    [InlineData(4, 0, 0, 1, 1, 1)]
    public void CompleteInterval_ShouldMapItsClosedOverlapIncludingPointTouches(int start, int end,
        int lowerN, int lowerD, int upperN, int upperD)
    {
        RationalSegmentInterval interval = default;
        Assert.True(interval.TryMapToSegment(new(Vector3d.Zero, new(4, 0, 0)),
            new(new(start, 0, 0), new(end, 0, 0)), out var mapped));
        Assert.Equal(0, RationalSegmentInterval.CompareParameters(mapped.IsClipped ? mapped.LowerNumerator : default,
            mapped.IsClipped ? mapped.LowerDenominator : Signed320.One, Integer(lowerN), Integer(lowerD)));
        Assert.Equal(0, RationalSegmentInterval.CompareParameters(mapped.IsClipped ? mapped.UpperNumerator : Signed320.One,
            mapped.IsClipped ? mapped.UpperDenominator : Signed320.One, Integer(upperN), Integer(upperD)));
    }

    [Theory]
    [InlineData(0, 1, 0, 2, 0)]
    [InlineData(4, 2, 0, 2, 0)]
    [InlineData(4, 1, 1, 2, 0)]
    [InlineData(4, 1, 0, 2, 1)]
    [InlineData(4, 1, 1, 2, 1)]
    [InlineData(4, 5, 0, 6, 0)]
    [InlineData(4, -2, 0, -1, 0)]
    public void Mapping_ShouldRejectDegenerateOffLineAndDisjointSegments(int sourceEnd, int startX, int startY, int endX, int endY)
    {
        RationalSegmentInterval interval = default;
        Assert.False(interval.TryMapToSegment(new(Vector3d.Zero, new(sourceEnd, 0, 0)),
            new(new(startX, startY, 0), new(endX, endY, 0)), out var mapped));
        Assert.False(mapped.IsClipped);
    }

    [Fact]
    public void FullRawRangeMapping_ShouldRetainBothOneRawEndpointsExactly()
    {
        RationalSegmentInterval interval = default;
        var source = new FixedSegment(new(Fixed64.FromRaw(-1), Fixed64.Zero, Fixed64.Zero),
            new(Fixed64.FromRaw(1), Fixed64.Zero, Fixed64.Zero));
        var target = new FixedSegment(new(Fixed64.MinValue, Fixed64.Zero, Fixed64.Zero),
            new(Fixed64.MaxValue, Fixed64.Zero, Fixed64.Zero));
        Assert.True(interval.TryMapToSegment(source, target, out var mapped));
        var denominator = new Signed320(0, 0, 0, 0, ulong.MaxValue);
        Assert.Equal(0, RationalSegmentInterval.CompareParameters(mapped.LowerNumerator, mapped.LowerDenominator,
            Integer(long.MaxValue), denominator));
        Assert.Equal(0, RationalSegmentInterval.CompareParameters(mapped.UpperNumerator, mapped.UpperDenominator,
            new Signed320(0, 0, 0, 0, (1UL << 63) + 1), denominator));
        var target2D = new FixedSegment2d(new(target.Start.X, Fixed64.Zero), new(target.End.X, Fixed64.Zero));
        Assert.Equal(new Vector2d(Fixed64.FromRaw(-1), Fixed64.Zero), mapped.GetEndpoint(target2D, false));
        Assert.Equal(new Vector2d(Fixed64.FromRaw(1), Fixed64.Zero), mapped.GetEndpoint(target2D, true));
        Assert.True(interval.TryMapToSegment(target, source, out var contained));
        Assert.False(contained.IsClipped);
    }

    [Theory]
    [InlineData(0, 1)]
    [InlineData(5, 7)]
    public void ClippedInterval_ShouldRejectACollinearTargetInsideOnlyTheOriginalSegment(int start, int end)
    {
        var interval = new RationalSegmentInterval(Integer(1), Integer(4), Integer(1), Integer(2));
        Assert.False(interval.TryMapToSegment(new(Vector3d.Zero, new(8, 0, 0)),
            new(new(start, 0, 0), new(end, 0, 0)), out _));
    }

    [Fact]
    public void WideMappedCoefficientsWithinTheClippingBank_ShouldRemainExact()
    {
        var denominator = new Signed320(1UL << 62, 0, 0, 0, 1);
        var upper = new Signed320(1UL << 62, 0, 0, 0, 0);
        var interval = new RationalSegmentInterval(Integer(1), denominator, upper, denominator);
        var edge = new FixedSegment(Vector3d.Zero, new(Fixed64.FromRaw(1), Fixed64.Zero, Fixed64.Zero));
        Assert.True(interval.TryMapToSegment(edge, edge, out var mapped));
        Assert.Equal(0, RationalSegmentInterval.CompareParameters(mapped.LowerNumerator, mapped.LowerDenominator, Integer(1), denominator));
        Assert.Equal(0, RationalSegmentInterval.CompareParameters(mapped.UpperNumerator, mapped.UpperDenominator, upper, denominator));
    }

    [Theory]
    [InlineData(2L, 3L)]
    [InlineData(0L, 2L)]
    [InlineData(-1L, 1L)]
    public void MappingBeyondTheClippingBank_ShouldThrowInsteadOfTruncating(long start, long end)
    {
        var denominator = new Signed320(1UL << 62, 0, 0, 0, 1);
        var interval = new RationalSegmentInterval(Integer(1), denominator, new(1UL << 62, 0, 0, 0, 0), denominator);
        var source = new FixedSegment(Vector3d.Zero, new(Fixed64.FromRaw(1), Fixed64.Zero, Fixed64.Zero));
        var target = new FixedSegment(new(Fixed64.FromRaw(start), Fixed64.Zero, Fixed64.Zero),
            new(Fixed64.FromRaw(end), Fixed64.Zero, Fixed64.Zero));
        Assert.Throws<OverflowException>(() => interval.TryMapToSegment(source, target, out _));
    }

    [Fact]
    public void MappedDenominatorAtTheSignedMinimum_ShouldNotOverflowWhenMadePositive()
    {
        var denominator = new Signed320(1UL << 62, 0, 0, 0, 0);
        var upper = new Signed320((1UL << 62) - 1, ulong.MaxValue, ulong.MaxValue, ulong.MaxValue, ulong.MaxValue);
        var interval = new RationalSegmentInterval(Integer(1), denominator, upper, denominator);
        var source = new FixedSegment(Vector3d.Zero, new(Fixed64.FromRaw(1), Fixed64.Zero, Fixed64.Zero));
        var target = new FixedSegment(new(Fixed64.FromRaw(-1), Fixed64.Zero, Fixed64.Zero),
            new(Fixed64.FromRaw(1), Fixed64.Zero, Fixed64.Zero));
        Assert.Throws<OverflowException>(() => interval.TryMapToSegment(source, target, out _));
    }

    private static Signed320 Integer(long value) => Signed320.ExtendValue(Signed192.Signed(value));

    private static long RoundEven(BigInteger numerator, BigInteger denominator)
    {
        BigInteger quotient = BigInteger.DivRem(BigInteger.Abs(numerator), denominator, out BigInteger remainder);
        if (2 * remainder > denominator || 2 * remainder == denominator && !quotient.IsEven) quotient++;
        return (long)(numerator.Sign < 0 ? -quotient : quotient);
    }
}
