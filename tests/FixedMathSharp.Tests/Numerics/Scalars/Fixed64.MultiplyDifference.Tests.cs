using Xunit;

namespace FixedMathSharp.Tests;

public sealed class Fixed64MultiplyDifferenceTests
{
    [Fact]
    public void TryMultiplyDifference_RescuesOppositeDomainEndpointsAfterDownscale()
    {
        Assert.True(Fixed64.TryMultiplyDifference(
            Fixed64.MaxValue,
            Fixed64.MinValue,
            Fixed64.Half,
            Fixed64.Half,
            out Fixed64 positive));
        Assert.True(Fixed64.TryMultiplyDifference(
            Fixed64.MinValue,
            Fixed64.MaxValue,
            Fixed64.Half,
            Fixed64.Half,
            out Fixed64 negative));

        Assert.Equal(Fixed64.FromRaw(4611686018427387904L), positive);
        Assert.Equal(Fixed64.FromRaw(-4611686018427387904L), negative);
    }

    [Theory]
    [InlineData(2L, 0L)]
    [InlineData(3L, 1L)]
    [InlineData(6L, 2L)]
    public void TryMultiplyDifference_RoundsExactProductOnceToEven(
        long differenceRaw,
        long expectedRaw)
    {
        Assert.True(Fixed64.TryMultiplyDifference(
            Fixed64.FromRaw(differenceRaw),
            Fixed64.Zero,
            Fixed64.FromRaw(1L),
            Fixed64.FromRaw(1L << 62),
            out Fixed64 result));

        Assert.Equal(expectedRaw, result.m_rawValue);
    }

    [Fact]
    public void TryMultiplyDifference_FinalOverflowReturnsFalseAndDefault()
    {
        Assert.False(Fixed64.TryMultiplyDifference(
            Fixed64.MaxValue,
            Fixed64.MinValue,
            Fixed64.One,
            Fixed64.One,
            out Fixed64 result));

        Assert.Equal(default, result);
    }

    [Fact]
    public void TryMultiplyDifference_SignedResultOverflowReturnsFalseAndDefault()
    {
        Assert.False(Fixed64.TryMultiplyDifference(
            Fixed64.MaxValue,
            Fixed64.Zero,
            Fixed64.One,
            Fixed64.Two,
            out Fixed64 result));

        Assert.Equal(default, result);
    }

    [Fact]
    public void TryMultiplyDifference_MiddleWordCarryStillRejectsExactOverflow()
    {
        Fixed64 value = Fixed64.MaxValue;
        Fixed64 subtrahend = Fixed64.MinValue;
        Fixed64 first = Fixed64.FromRaw(long.MaxValue / 16);
        Fixed64 second = Fixed64.MaxValue;

        Assert.False(Fixed64.TryMultiplyDifference(
            value,
            subtrahend,
            first,
            second,
            out Fixed64 result));
        Assert.Equal(default, result);
    }

    [Theory]
    [InlineData(false, false, false)]
    [InlineData(true, false, true)]
    [InlineData(false, true, true)]
    [InlineData(true, true, false)]
    public void TryMultiplyDifference_PreservesCombinedSign(
        bool reverseDifference,
        bool negateFirst,
        bool expectedNegative)
    {
        Fixed64 value = reverseDifference ? Fixed64.One : Fixed64.Two;
        Fixed64 subtrahend = reverseDifference ? Fixed64.Two : Fixed64.One;
        Fixed64 first = negateFirst ? -Fixed64.Two : Fixed64.Two;

        Assert.True(Fixed64.TryMultiplyDifference(
            value,
            subtrahend,
            first,
            Fixed64.Half,
            out Fixed64 result));

        Assert.Equal(expectedNegative ? -Fixed64.One : Fixed64.One, result);
    }
}
