using System;
using System.Numerics;
using Xunit;

namespace FixedMathSharp.Tests;

public sealed class WideNormalizedDepthRoundingTests
{
    private static readonly BigInteger AxisScale = BigInteger.One << 32;

    [Theory]
    [InlineData(12L, 9L, 5L, 8L, 4L, 5L, 0)]
    [InlineData(12L, 16L, 3L, 10L, 1L, 5L, -1)]
    public void Comparison_PreservesEqualDenominatorCancellationAndUnequalFallback(
        long leftOverlap,
        long leftSquaredAxis,
        long leftCommon,
        long rightOverlap,
        long rightSquaredAxis,
        long rightCommon,
        int expected)
    {
        int actual = WideArithmetic.CompareNonNegativeNormalizedDepths(
            ToSigned576(leftOverlap),
            ToSigned576(leftSquaredAxis),
            ToSigned320(leftCommon),
            ToSigned576(rightOverlap),
            ToSigned576(rightSquaredAxis),
            ToSigned320(rightCommon));

        Assert.Equal(expected, Math.Sign(actual));
    }

    [Fact]
    public void Rounding_TinyNonSquareAxisCannotUseSingleApproximationCorrection()
    {
        BigInteger overlap = new BigInteger(long.MaxValue);
        BigInteger squaredAxis = new BigInteger(2);

        Fixed64 actual = WideArithmetic.GetRoundedNonNegativeNormalizedDepth(
            ToSigned576(overlap),
            ToSigned576(squaredAxis),
            ToSigned320(BigInteger.One),
            out bool actualClamped);
        (long expectedRaw, bool expectedClamped) =
            GetExpected(overlap, squaredAxis, BigInteger.One);

        Assert.Equal(6_521_908_912_666_391_105L, expectedRaw);
        Assert.Equal(expectedRaw, actual.m_rawValue);
        Assert.Equal(expectedClamped, actualClamped);
    }

    [Fact]
    public void Rounding_PathThresholdCannotSkipTheLastTinyAxisMagnitude()
    {
        BigInteger[] axisMagnitudes =
        {
            BigInteger.One,
            (BigInteger.One << 31) - BigInteger.One,
            BigInteger.One << 31,
        };

        foreach (BigInteger axisMagnitude in axisMagnitudes)
        {
            BigInteger squaredAxis = axisMagnitude * axisMagnitude;
            Fixed64 actual = WideArithmetic.GetRoundedNonNegativeNormalizedDepth(
                ToSigned576(axisMagnitude),
                ToSigned576(squaredAxis),
                ToSigned320(BigInteger.One),
                out bool actualClamped);
            (long expectedRaw, bool expectedClamped) =
                GetExpected(axisMagnitude, squaredAxis, BigInteger.One);

            Assert.Equal(1L, expectedRaw);
            Assert.Equal(expectedRaw, actual.m_rawValue);
            Assert.Equal(expectedClamped, actualClamped);
        }
    }

    [Theory]
    [InlineData(5L, 2L)]
    [InlineData(7L, 4L)]
    public void Rounding_LowerMidpointCannotIgnoreCandidateParity(
        long overlapRaw,
        long expectedRaw)
    {
        BigInteger overlap = new BigInteger(overlapRaw);
        BigInteger squaredAxis = BigInteger.One;
        BigInteger common = new BigInteger(2);

        Fixed64 actual = WideArithmetic.GetRoundedNonNegativeNormalizedDepth(
            ToSigned576(overlap),
            ToSigned576(squaredAxis),
            ToSigned320(common),
            out bool actualClamped);
        (long oracleRaw, bool oracleClamped) =
            GetExpected(overlap, squaredAxis, common);

        Assert.Equal(expectedRaw, oracleRaw);
        Assert.Equal(oracleRaw, actual.m_rawValue);
        Assert.Equal(oracleClamped, actualClamped);
    }

    [Fact]
    public void Rounding_MaximumComparisonCannotClampARepresentableDepth()
    {
        BigInteger[] overlaps =
        {
            new BigInteger(long.MaxValue),
            new BigInteger(long.MaxValue) + BigInteger.One,
        };
        bool[] expectedClamp = { false, true };

        for (int index = 0; index < overlaps.Length; index++)
        {
            Fixed64 actual = WideArithmetic.GetRoundedNonNegativeNormalizedDepth(
                ToSigned576(overlaps[index]),
                ToSigned576(BigInteger.One),
                ToSigned320(BigInteger.One),
                out bool actualClamped);
            (long expectedRaw, bool oracleClamped) =
                GetExpected(overlaps[index], BigInteger.One, BigInteger.One);

            Assert.Equal(long.MaxValue, expectedRaw);
            Assert.Equal(expectedRaw, actual.m_rawValue);
            Assert.Equal(expectedClamp[index], oracleClamped);
            Assert.Equal(oracleClamped, actualClamped);
        }
    }

    [Fact]
    public void Rounding_FullDomainProductsCannotTruncateAboveSigned832()
    {
        BigInteger overlap = new BigInteger(long.MaxValue) << 400;
        BigInteger squaredAxis = BigInteger.One << 400;
        BigInteger common = BigInteger.One << 200;

        Fixed64 actual = WideArithmetic.GetRoundedNonNegativeNormalizedDepth(
            ToSigned576(overlap),
            ToSigned576(squaredAxis),
            ToSigned320(common),
            out bool actualClamped);
        (long expectedRaw, bool expectedClamped) =
            GetExpected(overlap, squaredAxis, common);

        Assert.Equal(long.MaxValue, expectedRaw);
        Assert.False(expectedClamped);
        Assert.Equal(expectedRaw, actual.m_rawValue);
        Assert.Equal(expectedClamped, actualClamped);
    }

    [Fact]
    public void Rounding_MatchesExactBigIntegerOracleAcrossMidpointsAndFullDomain()
    {
        (BigInteger Overlap, BigInteger SquaredAxis, BigInteger Common)[] cases =
        {
            (BigInteger.Zero, AxisScale * AxisScale, BigInteger.One),
            ((BigInteger.One << 32) + (BigInteger.One << 31) - 1,
                AxisScale * AxisScale,
                BigInteger.One),
            ((BigInteger.One << 32) + (BigInteger.One << 31),
                AxisScale * AxisScale,
                BigInteger.One),
            ((BigInteger.One << 32) + (BigInteger.One << 31) + 1,
                AxisScale * AxisScale,
                BigInteger.One),
            ((BigInteger.One << 33) + (BigInteger.One << 31),
                AxisScale * AxisScale,
                BigInteger.One),
            ((BigInteger.One << 32) + (BigInteger.One << 31),
                (AxisScale * AxisScale) + BigInteger.One,
                BigInteger.One),
            (new BigInteger(long.MaxValue) * AxisScale
                + (BigInteger.One << 31),
                (AxisScale * AxisScale) + BigInteger.One,
                BigInteger.One),
            (new BigInteger(long.MaxValue) * AxisScale
                + BigInteger.One,
                (AxisScale * AxisScale) + BigInteger.One,
                BigInteger.One),
            (new BigInteger(long.MaxValue) * (AxisScale >> 1)
                + (AxisScale >> 2),
                ((AxisScale >> 1) * (AxisScale >> 1))
                    + BigInteger.One,
                BigInteger.One),
            (new BigInteger(long.MaxValue) * AxisScale,
                AxisScale * AxisScale,
                BigInteger.One),
            (new BigInteger(long.MaxValue) * AxisScale
                + BigInteger.One,
                AxisScale * AxisScale,
                BigInteger.One),
            ((BigInteger.One << 63) * AxisScale,
                AxisScale * AxisScale,
                BigInteger.One),
            (((BigInteger.One << 190) + (BigInteger.One << 127) + 1)
                * ((BigInteger.One << 96) + 1),
                ((BigInteger.One << 190) + (BigInteger.One << 127) + 1)
                * ((BigInteger.One << 190) + (BigInteger.One << 127) + 1),
                (BigInteger.One << 96) + 1),
        };

        foreach ((BigInteger overlap, BigInteger squaredAxis, BigInteger common) in cases)
        {
            Fixed64 actual = WideArithmetic.GetRoundedNonNegativeNormalizedDepth(
                ToSigned576(overlap),
                ToSigned576(squaredAxis),
                ToSigned320(common),
                out bool actualClamped);
            (long expectedRaw, bool expectedClamped) =
                GetExpected(overlap, squaredAxis, common);

            Assert.Equal(expectedRaw, actual.m_rawValue);
            Assert.Equal(expectedClamped, actualClamped);
        }
    }

    [Fact]
    public void Rounding_WarmedFastAndTinyPathsCannotAllocate()
    {
        Signed576 fastOverlap = ToSigned576(
            (BigInteger.One << 32) + (BigInteger.One << 31));
        Signed576 fastSquaredAxis = ToSigned576(
            (AxisScale * AxisScale) + BigInteger.One);
        Signed576 tinyOverlap = ToSigned576(new BigInteger(long.MaxValue));
        Signed576 tinySquaredAxis = ToSigned576(new BigInteger(2));
        Signed576 fullDomainOverlap = ToSigned576(
            new BigInteger(long.MaxValue) << 400);
        Signed576 fullDomainSquaredAxis = ToSigned576(
            BigInteger.One << 400);
        Signed320 common = ToSigned320(BigInteger.One);
        Signed320 fullDomainCommon = ToSigned320(BigInteger.One << 200);

        _ = WideArithmetic.GetRoundedNonNegativeNormalizedDepth(
            fastOverlap,
            fastSquaredAxis,
            common,
            out _);
        _ = WideArithmetic.GetRoundedNonNegativeNormalizedDepth(
            tinyOverlap,
            tinySquaredAxis,
            common,
            out _);
        _ = WideArithmetic.GetRoundedNonNegativeNormalizedDepth(
            fullDomainOverlap,
            fullDomainSquaredAxis,
            fullDomainCommon,
            out _);
        long before = GC.GetAllocatedBytesForCurrentThread();
        Fixed64 fastResult = default;
        Fixed64 tinyResult = default;
        Fixed64 fullDomainResult = default;
        bool fastClamped = true;
        bool tinyClamped = true;
        bool fullDomainClamped = true;
        Span<ulong> dirty = stackalloc ulong[16];
        for (int index = 0; index < 32; index++)
        {
            dirty.Fill(unchecked((ulong)index * 0x9E37_79B9_7F4A_7C15UL));
            fastResult = WideArithmetic.GetRoundedNonNegativeNormalizedDepth(
                fastOverlap,
                fastSquaredAxis,
                common,
                out fastClamped);
            tinyResult = WideArithmetic.GetRoundedNonNegativeNormalizedDepth(
                tinyOverlap,
                tinySquaredAxis,
                common,
                out tinyClamped);
            fullDomainResult =
                WideArithmetic.GetRoundedNonNegativeNormalizedDepth(
                    fullDomainOverlap,
                    fullDomainSquaredAxis,
                    fullDomainCommon,
                    out fullDomainClamped);
        }
        long allocated = GC.GetAllocatedBytesForCurrentThread() - before;

        Assert.Equal(1L, fastResult.m_rawValue);
        Assert.Equal(6_521_908_912_666_391_105L, tinyResult.m_rawValue);
        Assert.Equal(long.MaxValue, fullDomainResult.m_rawValue);
        Assert.False(fastClamped);
        Assert.False(tinyClamped);
        Assert.False(fullDomainClamped);
        Assert.Equal(0L, allocated);
    }

    private static (long Raw, bool Clamped) GetExpected(
        BigInteger overlap,
        BigInteger squaredAxis,
        BigInteger common)
    {
        BigInteger maximum = long.MaxValue;
        if (CompareToTwiceRaw(
                overlap,
                squaredAxis,
                common,
                maximum << 1) > 0)
        {
            return (long.MaxValue, true);
        }

        BigInteger low = BigInteger.Zero;
        BigInteger high = maximum;
        while (low < high)
        {
            BigInteger candidate = (low + high + 1) >> 1;
            int comparison = CompareToLowerMidpoint(
                overlap,
                squaredAxis,
                common,
                candidate);
            bool roundsAtLeastCandidate = comparison > 0
                || (comparison == 0 && candidate.IsEven);
            if (roundsAtLeastCandidate)
                low = candidate;
            else
                high = candidate - 1;
        }

        return ((long)low, false);
    }

    private static int CompareToLowerMidpoint(
        BigInteger overlap,
        BigInteger squaredAxis,
        BigInteger common,
        BigInteger candidate)
    {
        BigInteger lowerMidpoint = (candidate << 1) - BigInteger.One;
        return CompareToTwiceRaw(
            overlap,
            squaredAxis,
            common,
            lowerMidpoint);
    }

    private static int CompareToTwiceRaw(
        BigInteger overlap,
        BigInteger squaredAxis,
        BigInteger common,
        BigInteger twiceRaw)
    {
        BigInteger twiceOverlap = overlap << 1;
        return (twiceOverlap * twiceOverlap).CompareTo(
            common * common
            * twiceRaw * twiceRaw
            * squaredAxis);
    }

    private static Signed320 ToSigned320(BigInteger value)
    {
        BigInteger encoded = value.Sign < 0
            ? (BigInteger.One << 320) + value
            : value;
        ulong word0 = (ulong)(encoded & ulong.MaxValue);
        ulong word1 = (ulong)((encoded >> 64) & ulong.MaxValue);
        ulong word2 = (ulong)((encoded >> 128) & ulong.MaxValue);
        ulong word3 = (ulong)((encoded >> 192) & ulong.MaxValue);
        ulong word4 = (ulong)((encoded >> 256) & ulong.MaxValue);
        return new Signed320(word4, word3, word2, word1, word0);
    }

    private static Signed576 ToSigned576(BigInteger value)
    {
        BigInteger encoded = value.Sign < 0
            ? (BigInteger.One << 576) + value
            : value;
        ulong word0 = (ulong)(encoded & ulong.MaxValue);
        ulong word1 = (ulong)((encoded >> 64) & ulong.MaxValue);
        ulong word2 = (ulong)((encoded >> 128) & ulong.MaxValue);
        ulong word3 = (ulong)((encoded >> 192) & ulong.MaxValue);
        ulong word4 = (ulong)((encoded >> 256) & ulong.MaxValue);
        ulong word5 = (ulong)((encoded >> 320) & ulong.MaxValue);
        ulong word6 = (ulong)((encoded >> 384) & ulong.MaxValue);
        ulong word7 = (ulong)((encoded >> 448) & ulong.MaxValue);
        ulong word8 = (ulong)((encoded >> 512) & ulong.MaxValue);
        return new Signed576(
            word8,
            word7,
            word6,
            word5,
            word4,
            word3,
            word2,
            word1,
            word0);
    }
}
