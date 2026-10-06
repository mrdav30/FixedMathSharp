using System;
using System.Numerics;
using Xunit;

namespace FixedMathSharp.Tests;

public sealed class WideSquareRootArithmeticTests
{
    [Fact]
    public void SquareRootBounds_EmptyInputReturnsZero()
    {
        WideArithmetic.GetMagnitudeSquareRootBounds(ReadOnlySpan<ulong>.Empty,
            out Signed192 lower, out Signed192 upper, out int shift);
        Assert.True(lower.IsZero);
        Assert.True(upper.IsZero);
        Assert.Equal(0, shift);
    }

    [Theory]
    [InlineData(0, 0, true)]
    [InlineData(1, 0, true)]
    [InlineData(2, 0, false)]
    [InlineData(3, 0, true)]
    [InlineData(4, 0, false)]
    [InlineData(5, 1, true)]
    [InlineData(6, 1, false)]
    [InlineData(7, 0, true)]
    [InlineData(8, 1, false)]
    [InlineData(9, 1, false)]
    [InlineData(10, 32, true)]
    [InlineData(11, 32, false)]
    [InlineData(12, 33, false)]
    [InlineData(13, 33, false)]
    [InlineData(14, 33, false)]
    [InlineData(15, 33, false)]
    [InlineData(16, 1184, false)]
    [InlineData(17, 1184, true)]
    [InlineData(18, 1184, false)]
    [InlineData(19, 1184, false)]
    [InlineData(20, 31, true)]
    [InlineData(21, 31, false)]
    public void SquareRootBounds_EncloseExactValueAndCollapseOnlyCertifiedSquares(
        int fixture, int expectedShift, bool exact)
    {
        BigInteger value = fixture switch
        {
            0 => BigInteger.Zero,
            1 => BigInteger.One,
            2 => 2,
            3 => 4,
            4 => (BigInteger.One << 192) - 1,
            5 => BigInteger.One << 192,
            6 => BigInteger.One << 193,
            7 => BigInteger.Pow((BigInteger.One << 96) - 1, 2),
            8 => (BigInteger.One << 192) + 1,
            9 => (BigInteger.One << 192) + 2,
            10 => BigInteger.One << 254,
            11 => (BigInteger.One << 254) + 1,
            12 => (BigInteger.One << 256) + (BigInteger.One << 64),
            13 => (BigInteger.One << 256) + 1,
            14 => (BigInteger.One << 256) + (BigInteger.One << 65),
            15 => (BigInteger.One << 256) + (BigInteger.One << 66),
            16 => (BigInteger.One << 2560) - 1,
            17 => BigInteger.One << 2558,
            18 => (BigInteger.One << 2558) + 1,
            19 => (BigInteger.One << 2560) - (BigInteger.One << 2368),
            20 => BigInteger.One << 252,
            _ => (BigInteger.One << 252) + (BigInteger.One << 61)
        };
        int words = Math.Max(1, (BitLength(value) + 63) / 64);
        // The first shifted prefix uses exactly four input words, with no
        // padded tail; its final cross-word read must still be in range.
        ulong[] magnitude = Encode(value, words + (fixture == 5 ? 0 : fixture % 3));
        ulong[] original = (ulong[])magnitude.Clone();
        WideArithmetic.GetMagnitudeSquareRootBounds(magnitude,
            out Signed192 lower, out Signed192 upper, out int shift);
        Assert.Equal(expectedShift, shift);
        Assert.Equal(0UL, lower.High);
        Assert.Equal(0UL, upper.High);
        BigInteger low = Integer(lower), high = Integer(upper);
        Assert.InRange(high, BigInteger.Zero, BigInteger.One << 96);
        Assert.Equal(exact ? BigInteger.Zero : BigInteger.One, high - low);
        BigInteger scaledLow = low << shift, scaledHigh = high << shift;
        Assert.True(scaledLow * scaledLow <= value);
        Assert.True(scaledHigh * scaledHigh >= value);
        if (exact)
            Assert.Equal(value, scaledLow * scaledLow);
        else
            Assert.True(scaledHigh * scaledHigh > value);
        Assert.Equal(original, magnitude);
    }

    [Theory]
    [InlineData(0)]
    [InlineData(1)]
    [InlineData(2)]
    [InlineData(3)]
    [InlineData(4)]
    [InlineData(5)]
    [InlineData(6)]
    [InlineData(7)]
    [InlineData(8)]
    [InlineData(9)]
    [InlineData(10)]
    [InlineData(11)]
    [InlineData(12)]
    [InlineData(13)]
    [InlineData(14)]
    [InlineData(15)]
    public void RatioFloorSquareRoot_MatchesIndependentIntegerSquareWitnesses(int fixture)
    {
        BigInteger maximum = ulong.MaxValue;
        BigInteger wide = (BigInteger.One << 9000) + 3;
        (BigInteger, BigInteger, ulong, ulong, int) data = fixture switch
        {
            0 => (0, 1, ulong.MaxValue, 0, 2),
            1 => (1, 1, 0, 0, 2),
            2 => (1, 1, 1, 1, 2),
            3 => (3, 2, ulong.MaxValue, 1, 2),
            4 => (8, 2, ulong.MaxValue, 2, 2),
            5 => (17, 4, ulong.MaxValue, 2, 2),
            6 => (17 * 17 * 7 - 1, 7, 17, 16, 2),
            7 => (17 * 17 * 7, 7, 17, 17, 2),
            8 => (17 * 17 * 7 + 1, 7, 17, 17, 2),
            9 => (3 * (maximum - 1) * (maximum - 1) + 2, 3, ulong.MaxValue, ulong.MaxValue - 1, 3),
            10 => (3 * maximum * maximum - 1, 3, ulong.MaxValue, ulong.MaxValue - 1, 3),
            11 => (3 * maximum * maximum, 3, ulong.MaxValue, ulong.MaxValue, 3),
            12 => (wide * (maximum - 1) * (maximum - 1) + wide - 1,
                wide, ulong.MaxValue, ulong.MaxValue - 1, 147),
            13 => ((BigInteger.One << 9407) - 1, 1, ulong.MaxValue, ulong.MaxValue, 147),
            14 => (BigInteger.One << 9000, BigInteger.One << 9064, ulong.MaxValue, 0, 147),
            _ => ((BigInteger.One << 9408) - 1, (BigInteger.One << 9408) - 1, ulong.MaxValue, 1, 147)
        };
        var (numerator, denominator, cap, expected, words) = data;
        ulong[] n = Encode(numerator, words), d = Encode(denominator, words);
        ulong[] originalN = (ulong[])n.Clone(), originalD = (ulong[])d.Clone();
        ulong actual = WideArithmetic.GetRatioFloorSquareRoot(n, d, cap);
        Assert.Equal(expected, actual);
        // Independent witnesses establish the clipped floor without another
        // production division or square-root implementation as the oracle.
        BigInteger square = (BigInteger)actual * actual;
        Assert.True(square * denominator <= numerator);
        if (actual < cap)
            Assert.True((square + 2 * (BigInteger)actual + 1) * denominator > numerator);
        Assert.Equal(originalN, n);
        Assert.Equal(originalD, d);
    }

    private static int BitLength(BigInteger value)
    {
        int bits = 0;
        while (value != 0) { value >>= 1; bits++; }
        return bits;
    }

    private static BigInteger Integer(Signed192 value) =>
        ((BigInteger)value.High << 128) | ((BigInteger)value.Middle << 64) | value.Low;

    private static ulong[] Encode(BigInteger value, int words)
    {
        ulong[] result = new ulong[words];
        for (int index = 0; index < words; index++)
        {
            result[index] = (ulong)(value & ulong.MaxValue);
            value >>= 64;
        }
        Assert.Equal(BigInteger.Zero, value);
        return result;
    }
}
