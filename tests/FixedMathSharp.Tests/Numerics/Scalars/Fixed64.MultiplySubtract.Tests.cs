using System;
using System.Numerics;
using Xunit;

namespace FixedMathSharp.Tests;

public sealed class Fixed64MultiplySubtractTests
{
    [Fact]
    public void TryMultiplySubtractClamped_DirectCapsuleFactors_RetainsRepresentableDifference()
    {
        Assert.True(Fixed64.TryMultiplySubtractClamped(
            minuendFirst: Fixed64.MaxValue,
            minuendSecond: Fixed64.Two,
            minuendThird: Fixed64.One,
            subtrahendFirst: Fixed64.MaxValue,
            subtrahendSecond: Fixed64.Two - Fixed64.MinIncrement,
            subtrahendThird: Fixed64.One,
            out Fixed64 result));

        Assert.Equal(Fixed64.Half, result);
    }

    [Fact]
    public void TryMultiplySubtractClamped_CompoundCapsuleFactors_RetainsRepresentableDifference()
    {
        Assert.True(Fixed64.TryMultiplySubtractClamped(
            minuendFirst: Fixed64.MaxValue,
            minuendSecond: Fixed64.Two,
            minuendThird: Fixed64.One,
            minuendFourth: Fixed64.One,
            subtrahendFirst: Fixed64.MaxValue,
            subtrahendSecond: Fixed64.Two - Fixed64.MinIncrement,
            subtrahendThird: Fixed64.One,
            subtrahendFourth: Fixed64.One,
            out Fixed64 result));

        Assert.Equal(Fixed64.Half, result);
    }

    [Fact]
    public void TryMultiplySubtractClamped_RawDomainTuples_MatchBigIntegerOracle()
    {
        long[] boundaries =
        {
            long.MinValue,
            long.MinValue + 1L,
            -FixedMath.ONE_L,
            -3L,
            -1L,
            0L,
            1L,
            3L,
            FixedMath.ONE_L,
            long.MaxValue - 1L,
            long.MaxValue,
        };

        for (int index = 0; index < boundaries.Length; index++)
        {
            long first = boundaries[index];
            long second = boundaries[(index * 3 + 1) % boundaries.Length];
            long third = boundaries[(index * 5 + 2) % boundaries.Length];
            long fourth = boundaries[(index * 7 + 3) % boundaries.Length];

            AssertTwoMatchesOracle(first, second, third, fourth);
            AssertThreeMatchesOracle(first, second, third, fourth, first, second);
            AssertFourMatchesOracle(first, second, third, fourth, first, second, third, fourth);
        }

        ulong state = 0xD1B54A32D192ED03UL;
        for (int index = 0; index < 64; index++)
        {
            long first = NextRaw(ref state);
            long second = NextRaw(ref state);
            long third = NextRaw(ref state);
            long fourth = NextRaw(ref state);
            long fifth = NextRaw(ref state);
            long sixth = NextRaw(ref state);
            long seventh = NextRaw(ref state);
            long eighth = NextRaw(ref state);

            AssertTwoMatchesOracle(first, second, third, fourth);
            AssertThreeMatchesOracle(first, second, third, fourth, fifth, sixth);
            AssertFourMatchesOracle(first, second, third, fourth, fifth, sixth, seventh, eighth);
        }
    }

    [Fact]
    public void TryMultiplySubtractClamped_ExactZeroAndNegativeProductSubtraction_MatchOracle()
    {
        AssertTwoMatchesOracle(long.MinValue, FixedMath.ONE_L, long.MinValue, FixedMath.ONE_L);
        AssertThreeMatchesOracle(long.MinValue, FixedMath.ONE_L, FixedMath.ONE_L, long.MinValue, FixedMath.ONE_L, FixedMath.ONE_L);
        AssertFourMatchesOracle(long.MinValue, FixedMath.ONE_L, FixedMath.ONE_L, FixedMath.ONE_L, long.MinValue, FixedMath.ONE_L, FixedMath.ONE_L, FixedMath.ONE_L);

        AssertTwoMatchesOracle(1L, FixedMath.ONE_L, -1L, FixedMath.ONE_L);
        AssertThreeMatchesOracle(1L, FixedMath.ONE_L, FixedMath.ONE_L, -1L, FixedMath.ONE_L, FixedMath.ONE_L);
        AssertFourMatchesOracle(1L, FixedMath.ONE_L, FixedMath.ONE_L, FixedMath.ONE_L, -1L, FixedMath.ONE_L, FixedMath.ONE_L, FixedMath.ONE_L);
    }

    [Fact]
    public void TryMultiplySubtractClamped_SignPermutations_MatchBigIntegerOracle()
    {
        for (int mask = 0; mask < 16; mask++)
        {
            AssertTwoMatchesOracle(
                WithSign(3L, mask, 0),
                WithSign(5L, mask, 1),
                WithSign(7L, mask, 2),
                WithSign(11L, mask, 3));
        }

        for (int mask = 0; mask < 64; mask++)
        {
            AssertThreeMatchesOracle(
                WithSign(3L, mask, 0),
                WithSign(5L, mask, 1),
                WithSign(7L, mask, 2),
                WithSign(11L, mask, 3),
                WithSign(13L, mask, 4),
                WithSign(17L, mask, 5));
        }

        for (int mask = 0; mask < 256; mask++)
        {
            AssertFourMatchesOracle(
                WithSign(3L, mask, 0),
                WithSign(5L, mask, 1),
                WithSign(7L, mask, 2),
                WithSign(11L, mask, 3),
                WithSign(13L, mask, 4),
                WithSign(17L, mask, 5),
                WithSign(19L, mask, 6),
                WithSign(23L, mask, 7));
        }
    }

    [Fact]
    public void TryMultiplySubtractClamped_RoundingAndRepresentabilityBoundaries_MatchOracle()
    {
        AssertTwoMatchesOracle(1L, 1L << 31, 0L, FixedMath.ONE_L);
        AssertTwoMatchesOracle(1L, 3L << 31, 0L, FixedMath.ONE_L);
        AssertThreeMatchesOracle(1L, FixedMath.ONE_L >> 1, FixedMath.ONE_L, 0L, FixedMath.ONE_L, FixedMath.ONE_L);
        AssertThreeMatchesOracle(1L, FixedMath.ONE_L >> 1, FixedMath.ONE_L * 3L, 0L, FixedMath.ONE_L, FixedMath.ONE_L);
        AssertFourMatchesOracle(1L, FixedMath.ONE_L >> 1, FixedMath.ONE_L, FixedMath.ONE_L, 0L, FixedMath.ONE_L, FixedMath.ONE_L, FixedMath.ONE_L);
        AssertFourMatchesOracle(1L, FixedMath.ONE_L >> 1, FixedMath.ONE_L, FixedMath.ONE_L * 3L, 0L, FixedMath.ONE_L, FixedMath.ONE_L, FixedMath.ONE_L);

        AssertTwoMatchesOracle(long.MaxValue, FixedMath.ONE_L, 0L, FixedMath.ONE_L);
        AssertThreeMatchesOracle(long.MaxValue, FixedMath.ONE_L, FixedMath.ONE_L, 0L, FixedMath.ONE_L, FixedMath.ONE_L);
        AssertFourMatchesOracle(long.MaxValue, FixedMath.ONE_L, FixedMath.ONE_L, FixedMath.ONE_L, 0L, FixedMath.ONE_L, FixedMath.ONE_L, FixedMath.ONE_L);
        AssertTwoMatchesOracle(long.MaxValue, FixedMath.ONE_L, -1L, FixedMath.ONE_L >> 1);
        AssertThreeMatchesOracle(long.MaxValue, FixedMath.ONE_L, FixedMath.ONE_L, -1L, FixedMath.ONE_L >> 1, FixedMath.ONE_L);
        AssertFourMatchesOracle(long.MaxValue, FixedMath.ONE_L, FixedMath.ONE_L, FixedMath.ONE_L, -1L, FixedMath.ONE_L >> 1, FixedMath.ONE_L, FixedMath.ONE_L);
    }

    [Fact]
    public void TryMultiplySubtractClamped_WarmedCallsAllocateNothing()
    {
        for (int index = 0; index < 8; index++)
            InvokeAllOverloads();

        long before = GC.GetAllocatedBytesForCurrentThread();
        for (int index = 0; index < 64; index++)
            InvokeAllOverloads();

        Assert.Equal(0L, GC.GetAllocatedBytesForCurrentThread() - before);
    }

    private static void AssertTwoMatchesOracle(long first, long second, long third, long fourth)
    {
        (bool expectedSuccess, long expectedRaw) = GetOracle(2, first, second, third, fourth);
        bool actualSuccess = Fixed64.TryMultiplySubtractClamped(
            Fixed64.FromRaw(first),
            Fixed64.FromRaw(second),
            Fixed64.FromRaw(third),
            Fixed64.FromRaw(fourth),
            out Fixed64 actual);

        Assert.Equal(expectedSuccess, actualSuccess);
        Assert.Equal(expectedSuccess ? Fixed64.FromRaw(expectedRaw) : default, actual);
    }

    private static void AssertThreeMatchesOracle(long first, long second, long third, long fourth, long fifth, long sixth)
    {
        (bool expectedSuccess, long expectedRaw) = GetOracle(3, first, second, third, fourth, fifth, sixth);
        bool actualSuccess = Fixed64.TryMultiplySubtractClamped(
            Fixed64.FromRaw(first),
            Fixed64.FromRaw(second),
            Fixed64.FromRaw(third),
            Fixed64.FromRaw(fourth),
            Fixed64.FromRaw(fifth),
            Fixed64.FromRaw(sixth),
            out Fixed64 actual);

        Assert.Equal(expectedSuccess, actualSuccess);
        Assert.Equal(expectedSuccess ? Fixed64.FromRaw(expectedRaw) : default, actual);
    }

    private static void AssertFourMatchesOracle(
        long first,
        long second,
        long third,
        long fourth,
        long fifth,
        long sixth,
        long seventh,
        long eighth)
    {
        (bool expectedSuccess, long expectedRaw) = GetOracle(4, first, second, third, fourth, fifth, sixth, seventh, eighth);
        bool actualSuccess = Fixed64.TryMultiplySubtractClamped(
            Fixed64.FromRaw(first),
            Fixed64.FromRaw(second),
            Fixed64.FromRaw(third),
            Fixed64.FromRaw(fourth),
            Fixed64.FromRaw(fifth),
            Fixed64.FromRaw(sixth),
            Fixed64.FromRaw(seventh),
            Fixed64.FromRaw(eighth),
            out Fixed64 actual);

        Assert.Equal(expectedSuccess, actualSuccess);
        Assert.Equal(expectedSuccess ? Fixed64.FromRaw(expectedRaw) : default, actual);
    }

    private static (bool Success, long Raw) GetOracle(int factorsPerProduct, params long[] factors)
    {
        BigInteger minuend = BigInteger.One;
        BigInteger subtrahend = BigInteger.One;
        for (int index = 0; index < factorsPerProduct; index++)
        {
            minuend *= factors[index];
            subtrahend *= factors[index + factorsPerProduct];
        }

        BigInteger difference = minuend - subtrahend;
        if (difference <= BigInteger.Zero)
            return (true, 0L);

        BigInteger denominator = BigInteger.One << ((factorsPerProduct - 1) * FixedMath.SHIFT_AMOUNT_I);
        BigInteger quotient = BigInteger.DivRem(difference, denominator, out BigInteger remainder);
        int midpointComparison = (remainder << 1).CompareTo(denominator);
        if (midpointComparison > 0 || (midpointComparison == 0 && !quotient.IsEven))
            quotient++;

        return quotient > long.MaxValue ? (false, default) : (true, (long)quotient);
    }

    private static long NextRaw(ref ulong state)
    {
        state ^= state << 7;
        state ^= state >> 9;
        state ^= state << 8;
        return unchecked((long)state);
    }

    private static long WithSign(long magnitude, int mask, int bit) =>
        (mask & (1 << bit)) == 0 ? magnitude : -magnitude;

    private static void InvokeAllOverloads()
    {
        _ = Fixed64.TryMultiplySubtractClamped(Fixed64.MaxValue, Fixed64.Two, Fixed64.Zero, Fixed64.One, out _);
        _ = Fixed64.TryMultiplySubtractClamped(Fixed64.MaxValue, Fixed64.Two, Fixed64.One, Fixed64.Zero, Fixed64.One, Fixed64.One, out _);
        _ = Fixed64.TryMultiplySubtractClamped(Fixed64.MaxValue, Fixed64.Two, Fixed64.One, Fixed64.One, Fixed64.Zero, Fixed64.One, Fixed64.One, Fixed64.One, out _);
    }
}
