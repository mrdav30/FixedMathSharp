using System;
using System.Numerics;
using Xunit;

namespace FixedMathSharp.Tests;

public sealed class WideNormalizedDepthRoundingTests
{
    private static readonly BigInteger AxisScale = BigInteger.One << 32;

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
    public void Rounding_DirtyStackAndWarmedExecutionRemainAllocationFree()
    {
        Signed576 overlap = ToSigned576(
            (BigInteger.One << 32) + (BigInteger.One << 31));
        Signed576 squaredAxis = ToSigned576(
            (AxisScale * AxisScale) + BigInteger.One);
        Signed320 common = ToSigned320(BigInteger.One);

        _ = WideArithmetic.GetRoundedNonNegativeNormalizedDepth(
            overlap,
            squaredAxis,
            common,
            out _);
        long before = GC.GetAllocatedBytesForCurrentThread();
        Fixed64 result = default;
        bool clamped = true;
        Span<ulong> dirty = stackalloc ulong[16];
        for (int index = 0; index < 32; index++)
        {
            dirty.Fill(unchecked((ulong)index * 0x9E37_79B9_7F4A_7C15UL));
            result = WideArithmetic.GetRoundedNonNegativeNormalizedDepth(
                overlap,
                squaredAxis,
                common,
                out clamped);
        }
        long allocated = GC.GetAllocatedBytesForCurrentThread() - before;

        Assert.Equal(1L, result.m_rawValue);
        Assert.False(clamped);
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
