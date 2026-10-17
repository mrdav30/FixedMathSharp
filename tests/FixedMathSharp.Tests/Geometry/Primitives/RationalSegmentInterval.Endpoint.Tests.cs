using FixedMathSharp.Geometry;
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

    private static Signed320 Integer(long value) => Signed320.ExtendValue(Signed192.Signed(value));

    private static long RoundEven(BigInteger numerator, BigInteger denominator)
    {
        BigInteger quotient = BigInteger.DivRem(BigInteger.Abs(numerator), denominator, out BigInteger remainder);
        if (2 * remainder > denominator || 2 * remainder == denominator && !quotient.IsEven) quotient++;
        return (long)(numerator.Sign < 0 ? -quotient : quotient);
    }
}
