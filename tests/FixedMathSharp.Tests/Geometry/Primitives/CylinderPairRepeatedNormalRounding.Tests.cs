using System;
using System.Numerics;
using FixedMathSharp.Geometry;
using Xunit;

namespace FixedMathSharp.Tests;

public sealed class CylinderPairRepeatedNormalRoundingTests
{
    [Theory]
    [InlineData(1, 1, 1, 1, 0L)]
    [InlineData(3, 1, 1, 1, 2L)]
    [InlineData(1, -1, 1, 1, 0L)]
    [InlineData(3, -1, 1, 1, -2L)]
    [InlineData(1, 1, -1, -1, 0L)]
    [InlineData(3, 1, -1, -1, -2L)]
    [InlineData(1, -1, -1, -1, 0L)]
    [InlineData(3, -1, -1, -1, 2L)]
    public void NestedRadicalNormal_ExactHalfRawRoundsToEven(
        int odd, int xSign, int ySign, int orientation, long expectedX)
    {
        // (odd*sqrt(2), sqrt(2)*sqrt(D), 0) has norm 2^33*sqrt(2)
        // when D=2^66-odd^2. Thus normalized X is exactly odd/2 raw units.
        BigInteger d = (BigInteger.One << 66) - odd * odd;
        Assert.Equal(BigInteger.One << 66, d + odd * odd);
        AssertNormal(odd, d, xSign, ySign, orientation, expectedX);
    }

    [Theory]
    [InlineData(1, -1, 1, 1L)]
    [InlineData(1, 1, 1, 0L)]
    [InlineData(3, -1, 1, 2L)]
    [InlineData(3, 1, 1, 1L)]
    [InlineData(1, -1, -1, -1L)]
    [InlineData(1, 1, -1, 0L)]
    [InlineData(3, -1, -1, -2L)]
    [InlineData(3, 1, -1, -1L)]
    public void NestedRadicalNormal_AdjacentRadicandsStayOnCorrectSideOfHalfRaw(
        int odd, int offset, int orientation, long expectedX)
    {
        BigInteger d = (BigInteger.One << 66) - odd * odd + offset;
        // Compare normalized X to odd/2 raw units by squaring positive
        // quantities: the exact residual is -odd^2*offset, not a tolerance.
        BigInteger residual = ((BigInteger)odd * odd << 66) - odd * odd * (d + odd * odd);
        Assert.Equal(-offset, residual.Sign);
        AssertNormal(odd, d, 1, 1, orientation, expectedX);
    }

    private static void AssertNormal(int odd, BigInteger d, int xSign, int ySign,
        int orientation, long expectedX)
    {
        const int words = 2;
        Span<ulong> coordinates = stackalloc ulong[12 * words];
        Span<sbyte> coordinateSigns = stackalloc sbyte[12];
        Span<ulong> delta = stackalloc ulong[2 * words];
        Span<sbyte> deltaSigns = stackalloc sbyte[2];
        Span<ulong> e = stackalloc ulong[words];
        coordinates.Clear(); coordinateSigns.Clear(); delta.Clear(); deltaSigns.Clear(); e.Clear();

        // Each coordinate is an ordinary pair plus a pair times sqrt(delta);
        // each pair is low+high*sqrt(E). Keep E=2 and delta=D independent.
        coordinates[words] = (ulong)odd;
        coordinateSigns[1] = (sbyte)xSign;
        coordinates[7 * words] = 1;
        coordinateSigns[7] = (sbyte)ySign;
        delta[0] = (ulong)(d & ulong.MaxValue);
        delta[1] = (ulong)(d >> 64);
        deltaSigns[0] = 1;
        e[0] = 2;

        // Independent integer certificates bound both nonzero rounded
        // components, including the carry to exactly one for Y.
        BigInteger normSquaredWithoutTwo = odd * odd + d;
        BigInteger xSquaredFourRaw = (BigInteger)odd * odd << 66;
        long magnitudeX = Math.Abs(expectedX);
        BigInteger upper = (2 * magnitudeX + 1) * (2 * magnitudeX + 1) * normSquaredWithoutTwo;
        Assert.True(xSquaredFourRaw <= upper);
        if (magnitudeX > 0)
        {
            BigInteger lower = (2 * magnitudeX - 1) * (2 * magnitudeX - 1) * normSquaredWithoutTwo;
            Assert.True(xSquaredFourRaw >= lower);
        }
        BigInteger yLowerTwiceRaw = (BigInteger.One << 33) - 1;
        Assert.True((d << 66) > yLowerTwiceRaw * yLowerTwiceRaw * normSquaredWithoutTwo);

        ulong[] originalCoordinates = coordinates.ToArray(), originalDelta = delta.ToArray(), originalE = e.ToArray();
        sbyte[] originalCoordinateSigns = coordinateSigns.ToArray(), originalDeltaSigns = deltaSigns.ToArray();
        Vector3d actual = CylinderPairRepeatedRim.RoundNormal(
            coordinates, coordinateSigns, delta, deltaSigns, e, orientation);
        Assert.Equal(expectedX, actual.X.m_rawValue);
        Assert.Equal(ySign * orientation * (1L << 32), actual.Y.m_rawValue);
        Assert.Equal(0L, actual.Z.m_rawValue);
        Assert.True(coordinates.SequenceEqual(originalCoordinates));
        Assert.True(coordinateSigns.SequenceEqual(originalCoordinateSigns));
        Assert.True(delta.SequenceEqual(originalDelta));
        Assert.True(deltaSigns.SequenceEqual(originalDeltaSigns));
        Assert.True(e.SequenceEqual(originalE));
    }
}
