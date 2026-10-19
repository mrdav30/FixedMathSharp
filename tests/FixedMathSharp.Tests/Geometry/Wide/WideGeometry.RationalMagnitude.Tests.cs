using System;
using FixedMathSharp.Geometry;
using Xunit;

namespace FixedMathSharp.Tests.Geometry.Wide;

public sealed class RationalMagnitudeTests
{
    [Theory]
    [InlineData(0, 0, 0, 1, 0)]
    [InlineData(3, 4, 0, 1, 5)]
    [InlineData(-3, 4, 0, 2, 3)]
    [InlineData(1, 1, 1, 1, 2)]
    [InlineData(0, 0, 1, 2, 1)]
    [InlineData(long.MaxValue, 0, 0, 1, long.MaxValue)]
    public void ExactRationalNorm_ShouldRoundOutward(long x, long y, long z, long divisor, long expected)
    {
        Assert.True(WideGeometry.TryGetMagnitudeCeiling(Signed192.Signed(x), Signed192.Signed(y),
            Signed192.Signed(z), Signed192.Signed(divisor), out Fixed64 result));
        Assert.Equal(expected, result.m_rawValue);
    }

    [Fact]
    public void Norm_ShouldRetainCompleteProductsAndRejectRoundedOverflow()
    {
        Signed192 large = new(1UL << 60, 0, 17);
        Signed192 divisor = new(1UL << 61, 0, 0);
        Assert.True(WideGeometry.TryGetMagnitudeCeiling(large, large, large, divisor, out Fixed64 tiny));
        Assert.Equal(1, tiny.m_rawValue);
        Assert.False(WideGeometry.TryGetMagnitudeCeiling(Signed192.Signed(long.MaxValue), Signed192.Signed(1),
            default, Signed192.Signed(1), out _));
        Assert.False(WideGeometry.TryGetMagnitudeCeiling(large, default, default, Signed192.Signed(1), out _));
        Assert.Throws<ArgumentOutOfRangeException>(() => WideGeometry.TryGetMagnitudeCeiling(default, default,
            default, default, out _));
        Assert.Throws<ArgumentOutOfRangeException>(() => WideGeometry.TryGetMagnitudeCeiling(default, default,
            default, Signed192.Signed(-1), out _));
    }
}
