using System;
using System.Numerics;
using Xunit;

namespace FixedMathSharp.Tests;

public sealed class Fixed64MultiplyAddTests
{
    [Theory]
    [InlineData(2, 3, 4, 10)]
    [InlineData(-2, 3, 4, -2)]
    [InlineData(-2, -3, -4, 2)]
    public void MultiplyAdd_OrdinarySignedValues_ReturnExactResult(
        int left,
        int right,
        int addend,
        int expected)
    {
        Assert.Equal(
            (Fixed64)expected,
            Fixed64.MultiplyAdd((Fixed64)left, (Fixed64)right, (Fixed64)addend));
        Assert.True(Fixed64.TryMultiplyAdd(
            (Fixed64)left,
            (Fixed64)right,
            (Fixed64)addend,
            out Fixed64 result));
        Assert.Equal((Fixed64)expected, result);
    }

    [Theory]
    [InlineData(1L, 0L)]
    [InlineData(3L, 2L)]
    [InlineData(-1L, 0L)]
    [InlineData(-3L, -2L)]
    public void MultiplyAdd_HalfwayProducts_RoundToEven(long leftRaw, long expectedRaw)
    {
        Assert.Equal(
            Fixed64.FromRaw(expectedRaw),
            Fixed64.MultiplyAdd(
                Fixed64.FromRaw(leftRaw),
                Fixed64.Half,
                Fixed64.Zero));
    }

    [Fact]
    public void MultiplyAdd_ExactFinalResult_RescuesSaturatedProduct()
    {
        Assert.True(Fixed64.TryMultiplyAdd(
            Fixed64.MaxValue,
            Fixed64.Two,
            -Fixed64.MaxValue,
            out Fixed64 result));
        Assert.Equal(Fixed64.MaxValue, result);
    }

    [Fact]
    public void MultiplyAdd_UnrepresentableFinalResult_UsesExplicitContracts()
    {
        Assert.False(Fixed64.TryMultiplyAdd(
            Fixed64.MaxValue,
            Fixed64.Two,
            Fixed64.Zero,
            out Fixed64 result));
        Assert.Equal(default, result);
        Assert.Equal(
            Fixed64.MaxValue,
            Fixed64.MultiplyAdd(Fixed64.MaxValue, Fixed64.Two, Fixed64.Zero));
        Assert.Equal(
            Fixed64.MinValue,
            Fixed64.MultiplyAdd(Fixed64.MinValue, Fixed64.Two, Fixed64.Zero));
    }

    [Fact]
    public void MultiplyAdd_RawBoundaryCrossProduct_MatchesBigIntegerOracle()
    {
        long[] values =
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

        foreach (long leftRaw in values)
        foreach (long rightRaw in values)
        foreach (long addendRaw in values)
        {
            (bool expectedSuccess, long expectedRaw) = MultiplyAddOracle(
                leftRaw,
                rightRaw,
                addendRaw);
            bool actualSuccess = Fixed64.TryMultiplyAdd(
                Fixed64.FromRaw(leftRaw),
                Fixed64.FromRaw(rightRaw),
                Fixed64.FromRaw(addendRaw),
                out Fixed64 actual);

            Assert.Equal(expectedSuccess, actualSuccess);
            Assert.Equal(
                expectedSuccess ? Fixed64.FromRaw(expectedRaw) : default,
                actual);
        }
    }

    private static (bool Success, long Raw) MultiplyAddOracle(
        long leftRaw,
        long rightRaw,
        long addendRaw)
    {
        BigInteger numerator = (BigInteger)leftRaw * rightRaw
            + (BigInteger)addendRaw * FixedMath.ONE_L;
        bool negative = numerator.Sign < 0;
        BigInteger magnitude = BigInteger.Abs(numerator);
        BigInteger quotient = BigInteger.DivRem(
            magnitude,
            FixedMath.ONE_L,
            out BigInteger remainder);
        int midpointComparison = (remainder * 2).CompareTo(FixedMath.ONE_L);
        if (midpointComparison > 0 || (midpointComparison == 0 && !quotient.IsEven))
            quotient++;
        if (negative)
            quotient = -quotient;

        if (quotient < long.MinValue || quotient > long.MaxValue)
            return (false, default);
        return (true, (long)quotient);
    }
}
