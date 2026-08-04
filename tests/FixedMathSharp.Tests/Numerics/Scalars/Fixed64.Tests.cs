using System;
using System.Globalization;
using System.Numerics;
using System.Text.Json;
using System.Text.Json.Serialization;
using FixedMathSharp.Geometry;
using MemoryPack;
using Xunit;

namespace FixedMathSharp.Tests;

public class Fixed64Tests
{
    #region Test: Basic Arithmetic Operations (+, -, *, /)

    [Fact]
    public void Add_Fixed64Values_ReturnsCorrectSum()
    {
        var a = new Fixed64(2);
        var b = new Fixed64(3);
        var result = a + b;
        Assert.Equal(new Fixed64(5), result);
    }

    [Fact]
    public void Subtract_Fixed64Values_ReturnsCorrectDifference()
    {
        var a = new Fixed64(5);
        var b = new Fixed64(3);
        var result = a - b;
        Assert.Equal(new Fixed64(2), result);
    }

    [Fact]
    public void Multiply_Fixed64Values_ReturnsCorrectProduct()
    {
        var a = new Fixed64(2);
        var b = new Fixed64(3);
        var result = a * b;
        Assert.Equal(new Fixed64(6), result);
    }

    [Fact]
    public void Divide_Fixed64Values_ReturnsCorrectQuotient()
    {
        var a = new Fixed64(6);
        var b = new Fixed64(2);
        var result = a / b;
        Assert.Equal(new Fixed64(3), result);
    }

    [Fact]
    public void Divide_ValidDivisor_DoesNotAllocate()
    {
        var a = new Fixed64(6);
        var b = new Fixed64(2);
        var expected = new Fixed64(3);

        _ = a / b;

        long before = GC.GetAllocatedBytesForCurrentThread();
        var result = a / b;
        long allocated = GC.GetAllocatedBytesForCurrentThread() - before;

        Assert.Equal(expected, result);
        Assert.Equal(0, allocated);
    }

    [Fact]
    public void Divide_ByZero_ThrowsException()
    {
        var a = new Fixed64(6);
        Assert.Throws<DivideByZeroException>(() => { var result = a / Fixed64.Zero; });
    }

    [Theory]
    [InlineData(1L, 0L)]
    [InlineData(3L, 2L)]
    [InlineData(5L, 2L)]
    [InlineData(-1L, 0L)]
    [InlineData(-3L, -2L)]
    [InlineData(-5L, -2L)]
    public void Divide_ByTwo_MidpointsRoundToEven(long inputRaw, long expectedRaw)
    {
        Fixed64 result = Fixed64.FromRaw(inputRaw) / Fixed64.Two;

        Assert.Equal(Fixed64.FromRaw(expectedRaw), result);
    }

    [Fact]
    public void Divide_ByBinaryPowers_MatchesMultiplicationByExactReciprocal()
    {
        long[] rawValues =
        {
            long.MinValue,
            long.MaxValue,
            -9L,
            -8L,
            -7L,
            -6L,
            -5L,
            -4L,
            -3L,
            -2L,
            -1L,
            0L,
            1L,
            2L,
            3L,
            4L,
            5L,
            6L,
            7L,
            8L,
            9L,
        };

        foreach (long rawValue in rawValues)
        {
            Fixed64 value = Fixed64.FromRaw(rawValue);

            Assert.Equal(value * Fixed64.Half, value / Fixed64.Two);
            Assert.Equal(value * Fixed64.Quarter, value / new Fixed64(4));
            Assert.Equal(value * Fixed64.Eighth, value / new Fixed64(8));
        }
    }

    [Fact]
    public void Divide_AroundMidpoint_RoundsBelowAtAndAboveToNearestEven()
    {
        Fixed64 positive = Fixed64.MinIncrement;
        Fixed64 negative = -Fixed64.MinIncrement;
        (Fixed64 Divisor, long PositiveRaw, long NegativeRaw)[] cases =
        {
            (Fixed64.Two + Fixed64.MinIncrement, 0L, 0L),
            (Fixed64.Two, 0L, 0L),
            (Fixed64.Two - Fixed64.MinIncrement, 1L, -1L),
        };

        foreach ((Fixed64 divisor, long positiveRaw, long negativeRaw) in cases)
        {
            Assert.Equal(Fixed64.FromRaw(positiveRaw), positive / divisor);
            Assert.Equal(Fixed64.FromRaw(negativeRaw), negative / divisor);
        }
    }

    [Fact]
    public void Divide_RawDomain_MatchesBigIntegerOracle()
    {
        (long Dividend, long Divisor)[] boundaryPairs =
        {
            (0L, 1L),
            (1L, Fixed64.Two.m_rawValue),
            (3L, Fixed64.Two.m_rawValue),
            (5L, Fixed64.Two.m_rawValue),
            (-1L, Fixed64.Two.m_rawValue),
            (-3L, Fixed64.Two.m_rawValue),
            (-5L, Fixed64.Two.m_rawValue),
            (long.MinValue, long.MinValue),
            (long.MinValue, -Fixed64.One.m_rawValue),
            (long.MinValue, Fixed64.One.m_rawValue),
            (long.MinValue, -1L),
            (long.MinValue, 1L),
            (long.MinValue, long.MaxValue),
            (long.MaxValue, long.MinValue),
            (long.MaxValue, -1L),
            (long.MaxValue, 1L),
            (long.MaxValue, long.MaxValue),
        };

        foreach ((long dividend, long divisor) in boundaryPairs)
            AssertDivisionMatchesOracle(dividend, divisor);

        var random = new System.Random(0xD1A1DE);
        for (int i = 0; i < 2_048; i++)
        {
            long dividend = random.NextInt64();
            if ((i & 1) != 0)
                dividend = ~dividend;

            long divisor = random.NextInt64(1L, long.MaxValue);
            if ((i & 2) != 0)
                divisor = -divisor;

            AssertDivisionMatchesOracle(dividend, divisor);
        }
    }

    [Theory]
    [InlineData(6L, 1L, 2L, 3L)]
    [InlineData(-6L, 1L, 2L, -3L)]
    [InlineData(6L, -1L, 2L, -3L)]
    [InlineData(6L, 1L, -2L, -3L)]
    [InlineData(-6L, -1L, 2L, 3L)]
    [InlineData(-6L, 1L, -2L, 3L)]
    [InlineData(6L, -1L, -2L, 3L)]
    [InlineData(-6L, -1L, -2L, -3L)]
    [InlineData(0L, long.MinValue, 1L, 0L)]
    [InlineData(long.MaxValue, 1L, 1L, long.MaxValue)]
    [InlineData(long.MinValue, 1L, 1L, long.MinValue)]
    public void TryMultiplyDivide_TwoFactors_SignAndBoundaryCases_ReturnExactResult(
        long leftRaw,
        long rightRaw,
        long divisorRaw,
        long expectedRaw)
    {
        bool succeeded = Fixed64.TryMultiplyDivide(
            Fixed64.FromRaw(leftRaw),
            Fixed64.FromRaw(rightRaw),
            Fixed64.FromRaw(divisorRaw),
            out Fixed64 result);

        Assert.True(succeeded);
        Assert.Equal(expectedRaw, result.m_rawValue);
    }

    [Fact]
    public void TryMultiplyDivide_TwoFactors_DivisorZeroOrOverflow_ReturnsFalseAndDefault()
    {
        Assert.False(Fixed64.TryMultiplyDivide(
            Fixed64.One,
            Fixed64.One,
            Fixed64.Zero,
            out Fixed64 zeroDivisorResult));
        Assert.Equal(default, zeroDivisorResult);

        Assert.False(Fixed64.TryMultiplyDivide(
            Fixed64.MaxValue,
            Fixed64.FromRaw(2),
            Fixed64.MinIncrement,
            out Fixed64 positiveOverflowResult));
        Assert.Equal(default, positiveOverflowResult);

        Assert.False(Fixed64.TryMultiplyDivide(
            Fixed64.MinValue,
            Fixed64.FromRaw(2),
            Fixed64.MinIncrement,
            out Fixed64 negativeOverflowResult));
        Assert.Equal(default, negativeOverflowResult);
    }

    [Theory]
    [InlineData(1L, 0L)]
    [InlineData(3L, 2L)]
    [InlineData(-1L, 0L)]
    [InlineData(-3L, -2L)]
    public void TryMultiplyDivide_TwoFactors_MidpointsRoundToEven(long leftRaw, long expectedRaw)
    {
        bool succeeded = Fixed64.TryMultiplyDivide(
            Fixed64.FromRaw(leftRaw),
            Fixed64.MinIncrement,
            Fixed64.FromRaw(2),
            out Fixed64 result);

        Assert.True(succeeded);
        Assert.Equal(expectedRaw, result.m_rawValue);
    }

    [Fact]
    public void TryMultiplyDivide_TwoFactors_RescuesSaturatedIntermediate()
    {
        var value = new Fixed64(65_536);

        Assert.NotEqual(value, (value * value) / value);
        Assert.True(Fixed64.TryMultiplyDivide(value, value, value, out Fixed64 result));
        Assert.Equal(value, result);
    }

    [Fact]
    public void TryMultiplyDivide_TwoFactors_RescuesRoundedToZeroIntermediate()
    {
        Assert.Equal(Fixed64.Zero, (Fixed64.MinIncrement * Fixed64.Half) / Fixed64.Half);
        Assert.True(Fixed64.TryMultiplyDivide(
            Fixed64.MinIncrement,
            Fixed64.Half,
            Fixed64.Half,
            out Fixed64 result));
        Assert.Equal(Fixed64.MinIncrement, result);
    }

    [Theory]
    [InlineData(6L, 1L, 4_294_967_296L, 2L, 3L)]
    [InlineData(-6L, 1L, 4_294_967_296L, 2L, -3L)]
    [InlineData(6L, -1L, 4_294_967_296L, 2L, -3L)]
    [InlineData(6L, 1L, -4_294_967_296L, 2L, -3L)]
    [InlineData(6L, 1L, 4_294_967_296L, -2L, -3L)]
    [InlineData(-6L, -1L, 4_294_967_296L, 2L, 3L)]
    [InlineData(-6L, 1L, -4_294_967_296L, 2L, 3L)]
    [InlineData(-6L, 1L, 4_294_967_296L, -2L, 3L)]
    [InlineData(6L, -1L, -4_294_967_296L, -2L, -3L)]
    [InlineData(0L, long.MinValue, long.MinValue, 1L, 0L)]
    [InlineData(long.MaxValue, 4_294_967_296L, 4_294_967_296L, 4_294_967_296L, long.MaxValue)]
    [InlineData(long.MinValue, 4_294_967_296L, 4_294_967_296L, 4_294_967_296L, long.MinValue)]
    public void TryMultiplyDivide_ThreeFactors_SignAndBoundaryCases_ReturnExactResult(
        long firstRaw,
        long secondRaw,
        long thirdRaw,
        long divisorRaw,
        long expectedRaw)
    {
        bool succeeded = Fixed64.TryMultiplyDivide(
            Fixed64.FromRaw(firstRaw),
            Fixed64.FromRaw(secondRaw),
            Fixed64.FromRaw(thirdRaw),
            Fixed64.FromRaw(divisorRaw),
            out Fixed64 result);

        Assert.True(succeeded);
        Assert.Equal(expectedRaw, result.m_rawValue);
    }

    [Fact]
    public void TryMultiplyDivide_ThreeFactors_DivisorZeroOrOverflow_ReturnsFalseAndDefault()
    {
        Assert.False(Fixed64.TryMultiplyDivide(
            Fixed64.One,
            Fixed64.One,
            Fixed64.One,
            Fixed64.Zero,
            out Fixed64 zeroDivisorResult));
        Assert.Equal(default, zeroDivisorResult);

        Assert.False(Fixed64.TryMultiplyDivide(
            Fixed64.MaxValue,
            Fixed64.FromRaw(2),
            Fixed64.One,
            Fixed64.MinIncrement,
            out Fixed64 positiveOverflowResult));
        Assert.Equal(default, positiveOverflowResult);

        Assert.False(Fixed64.TryMultiplyDivide(
            Fixed64.MinValue,
            Fixed64.FromRaw(2),
            Fixed64.One,
            Fixed64.MinIncrement,
            out Fixed64 negativeOverflowResult));
        Assert.Equal(default, negativeOverflowResult);
    }

    [Theory]
    [InlineData(1L, 0L)]
    [InlineData(3L, 2L)]
    [InlineData(-1L, 0L)]
    [InlineData(-3L, -2L)]
    public void TryMultiplyDivide_ThreeFactors_MidpointsRoundToEven(long firstRaw, long expectedRaw)
    {
        bool succeeded = Fixed64.TryMultiplyDivide(
            Fixed64.FromRaw(firstRaw),
            Fixed64.MinIncrement,
            Fixed64.One,
            Fixed64.FromRaw(2),
            out Fixed64 result);

        Assert.True(succeeded);
        Assert.Equal(expectedRaw, result.m_rawValue);
    }

    [Fact]
    public void TryMultiplyDivide_ThreeFactors_CcdRawRegressionPreservesRepresentableResult()
    {
        bool succeeded = Fixed64.TryMultiplyDivide(
            Fixed64.FromRaw(1L << 16),
            Fixed64.One,
            Fixed64.One,
            Fixed64.MinIncrement,
            out Fixed64 result);

        Assert.True(succeeded);
        Assert.Equal(1L << 48, result.m_rawValue);
    }

    [Fact]
    public void TryMultiplyDivide_ThreeFactors_RescuesSaturatedIntermediate()
    {
        var value = new Fixed64(65_536);

        Assert.NotEqual(value, ((value * value) * Fixed64.One) / value);
        Assert.True(Fixed64.TryMultiplyDivide(
            value,
            value,
            Fixed64.One,
            value,
            out Fixed64 result));
        Assert.Equal(value, result);
    }

    [Fact]
    public void TryMultiplyDivide_ThreeFactors_RescuesRoundedToZeroIntermediate()
    {
        Assert.Equal(
            Fixed64.Zero,
            ((Fixed64.MinIncrement * Fixed64.Half) * Fixed64.One) / Fixed64.Half);
        Assert.True(Fixed64.TryMultiplyDivide(
            Fixed64.MinIncrement,
            Fixed64.Half,
            Fixed64.One,
            Fixed64.Half,
            out Fixed64 result));
        Assert.Equal(Fixed64.MinIncrement, result);
    }

    [Fact]
    public void TryMultiplyDivideBySum_NarrowsOnlyTheFinalRatio()
    {
        Assert.True(Fixed64.TryMultiplyDivideBySum(
            Fixed64.MaxValue,
            Fixed64.One,
            Fixed64.One,
            Fixed64.One,
            Fixed64.MaxValue,
            Fixed64.MaxValue,
            Fixed64.Zero,
            Fixed64.Zero,
            out Fixed64 recovered));
        Assert.Equal(Fixed64.Half, recovered);
        Assert.True(Fixed64.TryMultiplyDivideBySum(
            (Fixed64)3,
            (Fixed64)4,
            (Fixed64)5,
            Fixed64.Half,
            (Fixed64)2,
            (Fixed64)3,
            (Fixed64)4,
            Fixed64.One,
            out Fixed64 ordinary));
        Assert.Equal((Fixed64)3, ordinary);

        Assert.False(Fixed64.TryMultiplyDivideBySum(
            Fixed64.One,
            Fixed64.One,
            Fixed64.One,
            Fixed64.One,
            Fixed64.One,
            -Fixed64.One,
            Fixed64.Zero,
            Fixed64.Zero,
            out Fixed64 zeroDivisor));
        Assert.Equal(default, zeroDivisor);
        Assert.False(Fixed64.TryMultiplyDivideBySum(
            Fixed64.MaxValue,
            Fixed64.MaxValue,
            Fixed64.MaxValue,
            Fixed64.MaxValue,
            Fixed64.One,
            Fixed64.Zero,
            Fixed64.Zero,
            Fixed64.Zero,
            out Fixed64 overflow));
        Assert.Equal(default, overflow);
    }

    [Fact]
    public void TryMultiplyDivide_RawDomain_MatchesBigIntegerOracle()
    {
        (long Left, long Right, long Divisor)[] twoFactorBoundaries =
        {
            (0L, long.MinValue, 1L),
            (1L, 1L, 2L),
            (3L, 1L, 2L),
            (-3L, 1L, 2L),
            (long.MaxValue, 1L, 1L),
            (long.MinValue, 1L, 1L),
            (long.MaxValue, 2L, 1L),
            (long.MinValue, 2L, 1L),
            (1L << 48, 1L << 48, 1L << 48),
            (1L, 1L << 31, 1L << 31),
            // Exercises the small-divisor 128-by-64 path with a nonzero high word.
            (FixedMath.ONE_L, FixedMath.ONE_L, 3L),
            // The exact quotient is 2^63 plus a remainder above one half.
            (long.MaxValue, 5L << 60, (5L << 60) - 1L),
            // Exercises the 2^63 divisor magnitude without cancellation and with
            // the largest possible remainder.
            (long.MaxValue, 1L, long.MinValue),
            // Exercises the maximum 63-bit common-power cancellation.
            (long.MinValue, 1L, long.MinValue),
            (1L, 1L, 0L),
        };
        foreach ((long left, long right, long divisor) in twoFactorBoundaries)
            AssertMultiplyDivideMatchesOracle(left, right, divisor);

        (long First, long Second, long Third, long Divisor)[] threeFactorBoundaries =
        {
            (0L, long.MinValue, long.MinValue, 1L),
            (1L, 1L, FixedMath.ONE_L, 2L),
            (3L, 1L, FixedMath.ONE_L, 2L),
            (-3L, 1L, FixedMath.ONE_L, 2L),
            (long.MaxValue, FixedMath.ONE_L, FixedMath.ONE_L, FixedMath.ONE_L),
            (long.MinValue, FixedMath.ONE_L, FixedMath.ONE_L, FixedMath.ONE_L),
            (long.MaxValue, 2L, FixedMath.ONE_L, 1L),
            (long.MinValue, 2L, FixedMath.ONE_L, 1L),
            (1L << 16, FixedMath.ONE_L, FixedMath.ONE_L, 1L),
            (1L << 48, 1L << 48, FixedMath.ONE_L, 1L << 48),
            (1L, 1L << 31, FixedMath.ONE_L, 1L << 31),
            // The discarded quotient bits are zero; only the division remainder is sticky.
            (1L, 1L, (1L << 33) + 1L, 2L),
            // The two partial middle words carry into the high word while the
            // final rounded result remains representable.
            (4_416_396_496_686_455L, 2_551_152_366_865_965L,
                1_530_502_775_920_113L, 1_796_020_480_733_518_619L),
            // Exercises a nonzero 2^63 divisor magnitude and maximum remainder.
            (long.MaxValue, 1L, 1L, long.MinValue),
            // Exercises the maximum 63-bit common-power cancellation.
            (long.MinValue, FixedMath.ONE_L, FixedMath.ONE_L, long.MinValue),
            // Exercises long.MinValue as the third-factor magnitude.
            (1L, FixedMath.ONE_L, long.MinValue, FixedMath.ONE_L),
            (1L, 1L, FixedMath.ONE_L, 0L),
        };
        foreach ((long first, long second, long third, long divisor) in threeFactorBoundaries)
            AssertMultiplyDivideMatchesOracle(first, second, third, divisor);

        var random = new System.Random(0x4D17D1);
        for (int i = 0; i < 2_048; i++)
        {
            long first = random.NextInt64();
            long second = random.NextInt64();
            long third = random.NextInt64();
            long divisor = random.NextInt64(1L, long.MaxValue);
            if ((i & 1) != 0)
                first = ~first;
            if ((i & 2) != 0)
                second = ~second;
            if ((i & 4) != 0)
                third = ~third;
            if ((i & 8) != 0)
                divisor = -divisor;

            AssertMultiplyDivideMatchesOracle(first, second, divisor);
            AssertMultiplyDivideMatchesOracle(first, second, third, divisor);
        }

        (long FactorLimit, long DivisorMinimum, long DivisorLimit)[] representableTiers =
        {
            (1L << 16, 1L, 1L << 8),
            (1L << 36, 1L << 31, 1L << 37),
            (1L << 52, 1L << 61, long.MaxValue),
        };
        foreach ((long factorLimit, long divisorMinimum, long divisorLimit) in representableTiers)
        {
            for (int i = 0; i < 256; i++)
            {
                long first = random.NextInt64(1L, factorLimit);
                long second = random.NextInt64(1L, factorLimit);
                long third = random.NextInt64(1L, factorLimit);
                long divisor = random.NextInt64(divisorMinimum, divisorLimit);
                if ((i & 1) != 0)
                    first = -first;
                if ((i & 2) != 0)
                    second = -second;
                if ((i & 4) != 0)
                    third = -third;
                if ((i & 8) != 0)
                    divisor = -divisor;

                AssertMultiplyDivideMatchesOracle(
                    first,
                    second,
                    third,
                    divisor,
                    requireRepresentable: true);
            }
        }
    }

    [Fact]
    public void MultiplyDivide_InternalCore_PreservesSaturatedFallback()
    {
        Fixed64 twoFactorPositive = Fixed64.MultiplyDivide(
            Fixed64.MaxValue,
            Fixed64.FromRaw(2),
            Fixed64.MinIncrement,
            out bool twoFactorRepresentable);
        Fixed64 threeFactorNegative = Fixed64.MultiplyDivide(
            Fixed64.MinValue,
            Fixed64.FromRaw(2),
            Fixed64.One,
            Fixed64.MinIncrement,
            out bool threeFactorRepresentable);

        Assert.False(twoFactorRepresentable);
        Assert.Equal(Fixed64.MaxValue, twoFactorPositive);
        Assert.False(threeFactorRepresentable);
        Assert.Equal(Fixed64.MinValue, threeFactorNegative);
    }

    [Fact]
    public void RoundGuardedQuotientToEven_UsesGuardStickyParityAndCarry()
    {
        AssertRoundGuardedQuotient(4UL, false, 2UL, false);
        AssertRoundGuardedQuotient(5UL, false, 2UL, false);
        AssertRoundGuardedQuotient(7UL, false, 4UL, false);
        AssertRoundGuardedQuotient(5UL, true, 3UL, false);
        AssertRoundGuardedQuotient(ulong.MaxValue, false, 1UL << 63, true);
    }

    [Theory]
    [InlineData(false, long.MaxValue)]
    [InlineData(true, long.MinValue)]
    public void DivideMagnitude_RoundedCarrySaturatesToSignedLimit(
        bool negative,
        long expectedRaw)
    {
        // A full-width dividend reaches final rounding carry while the divisor remains
        // inside the signed-raw magnitude domain required by DivideMagnitude.
        Fixed64 result = Fixed64.DivideMagnitude(
            ulong.MaxValue,
            1UL << (FixedMath.SHIFT_AMOUNT_I + 1),
            negative);

        Assert.Equal(expectedRaw, result.m_rawValue);
    }

    [Fact]
    public void WideDifferenceProducts_MatchBigIntegerAcrossRawDomain()
    {
        var random = new System.Random(0x6E9D21);
        for (int i = 0; i < 2_048; i++)
        {
            long leftEndX = random.NextInt64();
            long leftStartX = random.NextInt64();
            long leftEndY = random.NextInt64();
            long leftStartY = random.NextInt64();
            long rightEndX = random.NextInt64();
            long rightStartX = random.NextInt64();
            long rightEndY = random.NextInt64();
            long rightStartY = random.NextInt64();
            long leftEndZ = random.NextInt64();
            long leftStartZ = random.NextInt64();
            long rightEndZ = random.NextInt64();
            long rightStartZ = random.NextInt64();

            BigInteger leftX = (BigInteger)leftEndX - leftStartX;
            BigInteger leftY = (BigInteger)leftEndY - leftStartY;
            BigInteger rightX = (BigInteger)rightEndX - rightStartX;
            BigInteger rightY = (BigInteger)rightEndY - rightStartY;
            BigInteger leftZ = (BigInteger)leftEndZ - leftStartZ;
            BigInteger rightZ = (BigInteger)rightEndZ - rightStartZ;

            Signed192 dot = WideGeometry.GetDifferenceDotProduct2D(
                Fixed64.FromRaw(leftEndX), Fixed64.FromRaw(leftStartX),
                Fixed64.FromRaw(leftEndY), Fixed64.FromRaw(leftStartY),
                Fixed64.FromRaw(rightEndX), Fixed64.FromRaw(rightStartX),
                Fixed64.FromRaw(rightEndY), Fixed64.FromRaw(rightStartY));
            Signed192 cross = WideGeometry.GetDifferenceCrossProduct2D(
                Fixed64.FromRaw(leftEndX), Fixed64.FromRaw(leftStartX),
                Fixed64.FromRaw(leftEndY), Fixed64.FromRaw(leftStartY),
                Fixed64.FromRaw(rightEndX), Fixed64.FromRaw(rightStartX),
                Fixed64.FromRaw(rightEndY), Fixed64.FromRaw(rightStartY));
            Signed192 dot3D = WideGeometry.GetDifferenceDotProduct3D(
                Fixed64.FromRaw(leftEndX), Fixed64.FromRaw(leftStartX),
                Fixed64.FromRaw(leftEndY), Fixed64.FromRaw(leftStartY),
                Fixed64.FromRaw(leftEndZ), Fixed64.FromRaw(leftStartZ),
                Fixed64.FromRaw(rightEndX), Fixed64.FromRaw(rightStartX),
                Fixed64.FromRaw(rightEndY), Fixed64.FromRaw(rightStartY),
                Fixed64.FromRaw(rightEndZ), Fixed64.FromRaw(rightStartZ));

            Assert.Equal((leftX * rightX) + (leftY * rightY), ToBigInteger(dot));
            Assert.Equal((leftX * rightY) - (leftY * rightX), ToBigInteger(cross));
            Assert.Equal(
                (leftX * rightX) + (leftY * rightY) + (leftZ * rightZ),
                ToBigInteger(dot3D));
        }
    }

    [Fact]
    public void WideSignedConversionAndBarycentricAccumulation_MatchBigIntegerOracle()
    {
        BigInteger weighted = (((BigInteger)long.MinValue - long.MaxValue) * Fixed64.Half.m_rawValue) * 2;
        Signed192 weightedActual = WideGeometry.GetDifferenceDotProduct2D(
            Fixed64.MinValue, Fixed64.MaxValue, Fixed64.MinValue, Fixed64.MaxValue,
            Fixed64.Half, Fixed64.Zero, Fixed64.Half, Fixed64.Zero);

        Assert.Equal(weighted, ToBigInteger(weightedActual));
        Assert.Equal(
            Fixed64.MinValue,
            Fixed64.BarycentricCoordinateFullDomain(
                Fixed64.MaxValue,
                Fixed64.MinValue,
                Fixed64.MinValue,
                Fixed64.Half,
                Fixed64.Half));

        (BigInteger Value, int FractionalBits)[] conversions =
        {
            (BigInteger.One << 31, 32),
            (3 * (BigInteger.One << 31), 32),
            (-(BigInteger.One << 31), 32),
            (-3 * (BigInteger.One << 31), 32),
            ((BigInteger)long.MaxValue << 32, 32),
            ((BigInteger)long.MinValue << 32, 32),
            ((BigInteger)long.MaxValue << 33, 33),
            ((BigInteger)long.MinValue << 33, 33),
            (BigInteger.One << 128, 32),
            (-(BigInteger.One << 128), 33),
            (BigInteger.One << 160, 32),
            (-(BigInteger.One << 159), 33),
            ((BigInteger)long.MaxValue << 32 | ((BigInteger.One << 31) + 1), 32),
            (-(((BigInteger.One << 63) << 32) + (BigInteger.One << 31) + 1), 32),
            (((BigInteger)ulong.MaxValue << 32) + (BigInteger.One << 31) + 1, 32),
            (-(((BigInteger)ulong.MaxValue << 32) + (BigInteger.One << 31) + 1), 32),
        };

        foreach ((BigInteger value, int fractionalBits) in conversions)
        {
            Assert.Equal(
                RoundWideRawToFixed(value, fractionalBits),
                Fixed64.RoundSignedToFixed(ToSigned192(value), fractionalBits));
        }
    }

    [Fact]
    public void WideGeneralSignedRatiosAndThresholds_MatchBigIntegerOracle()
    {
        (BigInteger Numerator, BigInteger Denominator)[] ratios =
        {
            (BigInteger.Zero, 3),
            (1, 2),
            (3, 2),
            (-3, 2),
            (3, -2),
            (-3, -2),
            (1, BigInteger.One << 33),
            (3, BigInteger.One << 33),
            ((BigInteger)long.MaxValue << 17, BigInteger.One << 49),
            ((BigInteger)long.MinValue << 17, BigInteger.One << 49),
            (BigInteger.One << 160, BigInteger.One),
            (-(BigInteger.One << 160), BigInteger.One),
            (long.MaxValue, BigInteger.One << 32),
            ((BigInteger.One << 64) - 1, BigInteger.One << 33),
            (BigInteger.One << 64, BigInteger.One << 33),
            (-((BigInteger.One << 64) - 1), BigInteger.One << 33),
            (-(BigInteger.One << 64), BigInteger.One << 33),
            (-(BigInteger.One << 64), (BigInteger.One << 33) + 1),
            (-((BigInteger.One << 64) + 1), BigInteger.One << 33),
        };

        foreach ((BigInteger numerator, BigInteger denominator) in ratios)
        {
            Assert.Equal(
                RoundWideRatioToFixed(numerator, denominator),
                Fixed64.GetSignedRatio(ToSigned192(numerator), ToSigned192(denominator)));
        }

        Assert.Equal(Fixed64.MaxValue, Fixed64.GetSignedRatio(ToSigned192(1), default));
        Assert.Equal(Fixed64.MinValue, Fixed64.GetSignedRatio(ToSigned192(-1), default));

        BigInteger threshold = (BigInteger)Fixed64.Epsilon.m_rawValue << 33;
        Assert.True(WideArithmetic.IsMagnitudeAtMost(ToSigned192(threshold - 1), (ulong)Fixed64.Epsilon.m_rawValue, 33));
        Assert.True(WideArithmetic.IsMagnitudeAtMost(ToSigned192(-threshold), (ulong)Fixed64.Epsilon.m_rawValue, 33));
        Assert.False(WideArithmetic.IsMagnitudeAtMost(ToSigned192(threshold + 1), (ulong)Fixed64.Epsilon.m_rawValue, 33));

        var random = new System.Random(0x51A9_77);
        var bytes = new byte[17];
        for (int i = 0; i < 512; i++)
        {
            BigInteger numerator = NextSignedGeometryDot(random, bytes);
            BigInteger denominator;
            do
            {
                denominator = NextSignedGeometryDot(random, bytes);
            }
            while (denominator.IsZero);

            Assert.Equal(
                RoundWideRatioToFixed(numerator, denominator),
                Fixed64.GetSignedRatio(ToSigned192(numerator), ToSigned192(denominator)));
        }
    }

    [Fact]
    public void WideSegmentProductsThresholdsAndRatios_MatchBigIntegerOracle()
    {
        BigInteger maximumDot = (BigInteger.One << 130) - 1;
        (BigInteger First, BigInteger Second, BigInteger Third, BigInteger Fourth)[] products =
        {
            (maximumDot, maximumDot - 17, maximumDot - 1, maximumDot - 18),
            (-maximumDot, maximumDot - 31, maximumDot - 7, -maximumDot + 13),
            (BigInteger.One << 129, -(BigInteger.One << 128), -(BigInteger.One << 127), BigInteger.One << 130),
            (maximumDot, maximumDot, maximumDot, maximumDot),
            ((BigInteger.One << 94) - 1, -((BigInteger.One << 94) - 3), -((BigInteger.One << 94) - 5), (BigInteger.One << 94) - 7),
            (-(BigInteger.One << 64), BigInteger.One << 64, BigInteger.Zero, BigInteger.Zero),
            (-(BigInteger.One << 64) + 1, (BigInteger.One << 64) + 1, BigInteger.Zero, BigInteger.Zero),
            (-(BigInteger.One << 96), BigInteger.One << 32, BigInteger.Zero, BigInteger.Zero),
            (-(BigInteger.One << 96), BigInteger.One << 96, BigInteger.Zero, BigInteger.Zero),
            (-((BigInteger.One << 128) - 1), (BigInteger.One << 128) + 1, BigInteger.Zero, BigInteger.Zero),
            (-((BigInteger.One << 128) - (BigInteger.One << 32)), (BigInteger.One << 128) + (BigInteger.One << 32), BigInteger.Zero, BigInteger.Zero),
            (-(BigInteger.One << 128), (BigInteger.One << 128) - 1, BigInteger.Zero, BigInteger.Zero),
            (-(BigInteger.One << 128), (BigInteger.One << 128) - (BigInteger.One << 64), BigInteger.Zero, BigInteger.Zero),
            (-(BigInteger.One << 128), BigInteger.One << 128, BigInteger.Zero, BigInteger.Zero),
            (BigInteger.Zero, BigInteger.One << 96, -BigInteger.One, BigInteger.One),
        };

        foreach ((BigInteger first, BigInteger second, BigInteger third, BigInteger fourth) in products)
        {
            Signed320 actual = WideArithmetic.MultiplySubtract(
                ToSigned192(first),
                ToSigned192(second),
                ToSigned192(third),
                ToSigned192(fourth));

            Assert.Equal((first * second) - (third * fourth), ToBigInteger(actual));
        }

        var random = new System.Random(0x73E6_03D);
        var bytes = new byte[17];
        for (int i = 0; i < 256; i++)
        {
            BigInteger first = NextSignedGeometryDot(random, bytes);
            BigInteger second = NextSignedGeometryDot(random, bytes);
            BigInteger third = NextSignedGeometryDot(random, bytes);
            BigInteger fourth = NextSignedGeometryDot(random, bytes);

            Assert.Equal(
                (first * second) - (third * fourth),
                ToBigInteger(WideArithmetic.MultiplySubtract(
                    ToSigned192(first),
                    ToSigned192(second),
                    ToSigned192(third),
                    ToSigned192(fourth))));
            Assert.Equal(
                first + second,
                ToBigInteger(WideArithmetic.AddSigned192(ToSigned192(first), ToSigned192(second))));
            Assert.Equal(
                first - second,
                ToBigInteger(WideArithmetic.SubtractSigned192(ToSigned192(first), ToSigned192(second))));
        }

        BigInteger determinantThreshold = (BigInteger)Fixed64.Epsilon.m_rawValue << 96;
        Assert.True(WideGeometry.IsSegmentDeterminantNearParallel(ToSigned320(determinantThreshold - 1)));
        Assert.True(WideGeometry.IsSegmentDeterminantNearParallel(ToSigned320(-determinantThreshold + 1)));
        Assert.False(WideGeometry.IsSegmentDeterminantNearParallel(ToSigned320(determinantThreshold)));
        Assert.False(WideGeometry.IsSegmentDeterminantNearParallel(ToSigned320(-determinantThreshold)));

        (BigInteger Numerator, BigInteger Denominator)[] ratios =
        {
            (BigInteger.Zero, maximumDot * maximumDot),
            (BigInteger.One, BigInteger.One << 33),
            (3, BigInteger.One << 33),
            (-3, -(BigInteger.One << 33)),
            ((BigInteger.One << 258) - 17, (BigInteger.One << 259) + 91),
            (-((BigInteger.One << 258) - 17), -((BigInteger.One << 259) + 91)),
        };

        foreach ((BigInteger numerator, BigInteger denominator) in ratios)
        {
            Assert.True(Fixed64.TryGetUnitIntervalRatio(
                ToSigned320(numerator),
                ToSigned320(denominator),
                out Fixed64 result));
            Assert.Equal(RoundUnitRatioRawToEven(numerator, denominator), result.m_rawValue);
        }

        Assert.False(Fixed64.TryGetUnitIntervalRatio(
            ToSigned320(BigInteger.One << 259),
            ToSigned320(BigInteger.One << 258),
            out _));
        Assert.False(Fixed64.TryGetUnitIntervalRatio(
            ToSigned320(-BigInteger.One),
            ToSigned320(BigInteger.One << 258),
            out _));
        Assert.False(Fixed64.TryGetUnitIntervalRatio(
            ToSigned320(BigInteger.One),
            ToSigned320(BigInteger.Zero),
            out _));
        Assert.False(Fixed64.TryGetUnitIntervalRatio(
            ToSigned320(BigInteger.One << 200),
            ToSigned320(BigInteger.Zero),
            out _));
        Assert.False(Fixed64.TryGetUnitIntervalRatio(
            ToSigned320(BigInteger.One << 200),
            ToSigned320(-(BigInteger.One << 201)),
            out _));
        Assert.False(Fixed64.TryGetUnitIntervalRatio(
            ToSigned320(-(BigInteger.One << 200)),
            ToSigned320(BigInteger.One << 201),
            out _));
        Assert.True(Fixed64.TryGetUnitIntervalRatio(
            ToSigned320(-(BigInteger.One << 259)),
            ToSigned320(-(BigInteger.One << 259)),
            out Fixed64 negativeOne));
        Assert.Equal(Fixed64.One, negativeOne);

        BigInteger word0Base = BigInteger.One << 200;
        Assert.True(WideArithmetic.CompareMagnitude(
            ToSigned320(word0Base + 1),
            ToSigned320(word0Base + 2)) < 0);
        Assert.True(WideArithmetic.CompareMagnitude(
            ToSigned320(word0Base + 2),
            ToSigned320(word0Base + 1)) > 0);
        Assert.True(WideArithmetic.CompareMagnitude(
            ToSigned320(BigInteger.One << 64),
            ToSigned320(BigInteger.One << 65)) < 0);
        Assert.True(WideArithmetic.CompareMagnitude(
            ToSigned320(BigInteger.One << 65),
            ToSigned320(BigInteger.One << 64)) > 0);

        BigInteger[] negativeWordBoundaries =
        {
            -BigInteger.One,
            -(BigInteger.One << 64),
            -(BigInteger.One << 128),
            -(BigInteger.One << 192),
            -(BigInteger.One << 256),
        };
        foreach (BigInteger value in negativeWordBoundaries)
        {
            Assert.True(WideArithmetic.CompareMagnitude(
                ToSigned320(value),
                ToSigned320(BigInteger.Zero)) > 0);
        }
    }

    [Fact]
    public void WideTriangleRootsAndSignedRatios_MatchBigIntegerOracle()
    {
        BigInteger[] squaredMagnitudes =
        {
            BigInteger.Zero,
            BigInteger.One,
            ((BigInteger.One << 65) - 1) * ((BigInteger.One << 65) - 1),
            (BigInteger.One << 128) - 1,
            BigInteger.One << 128,
            BigInteger.One << 200,
            ((BigInteger.One << 129) - 1) * ((BigInteger.One << 129) - 1),
            (BigInteger.One << 260) + (BigInteger.One << 129) + 17,
            (BigInteger.One << 263) - 1,
        };

        foreach (BigInteger squaredMagnitude in squaredMagnitudes)
        {
            Signed192 root = WideArithmetic.GetFloorSquareRoot(
                ToSigned320(squaredMagnitude),
                out Signed192 remainder);
            BigInteger expectedRoot = IntegerSquareRoot(squaredMagnitude);
            Assert.Equal(expectedRoot, ToBigInteger(root));
            Assert.Equal(squaredMagnitude - (expectedRoot * expectedRoot), ToBigInteger(remainder));
        }

        Assert.Equal(257, WideArithmetic.GetBitLength(1UL, 0UL, 0UL, 0UL, 0UL));
        Assert.Equal(193, WideArithmetic.GetBitLength(0UL, 1UL, 0UL, 0UL, 0UL));
        Assert.Equal(129, WideArithmetic.GetBitLength(0UL, 0UL, 1UL, 0UL, 0UL));
        Assert.Equal(65, WideArithmetic.GetBitLength(0UL, 0UL, 0UL, 1UL, 0UL));
        Assert.Equal(1, WideArithmetic.GetBitLength(0UL, 0UL, 0UL, 0UL, 1UL));

        (BigInteger Numerator, BigInteger Denominator)[] ratios =
        {
            (BigInteger.Zero, BigInteger.One << 258),
            (BigInteger.One << 258, BigInteger.Zero),
            (-(BigInteger.One << 258), BigInteger.Zero),
            ((BigInteger.One << 258) + 17, (BigInteger.One << 259) + 31),
            (-((BigInteger.One << 258) + 17), (BigInteger.One << 259) + 31),
            ((BigInteger.One << 258) + 17, -((BigInteger.One << 259) + 31)),
            (3 * (BigInteger.One << 250), 2 * (BigInteger.One << 250)),
            (-3 * (BigInteger.One << 250), 2 * (BigInteger.One << 250)),
            (2 * (BigInteger.One << 250), 3 * (BigInteger.One << 250)),
            ((BigInteger.One << 260) + 51, (BigInteger.One << 258) + 7),
            (BigInteger.One << 300, BigInteger.One << 269),
            (-(BigInteger.One << 300), BigInteger.One << 269),
            (BigInteger.One << 310, BigInteger.One << 270),
            (-(BigInteger.One << 310), BigInteger.One << 270),
            ((BigInteger.One << 31) * (BigInteger.One << 260), BigInteger.One << 260),
            (-((BigInteger.One << 31) * (BigInteger.One << 260)), BigInteger.One << 260),
            (((BigInteger.One << 31) * (BigInteger.One << 260)) - 1, BigInteger.One << 260),
            (-(((BigInteger.One << 31) * (BigInteger.One << 260)) - 1), BigInteger.One << 260),
            ((((BigInteger.One << 260) + 1) << 31) - 1, (BigInteger.One << 260) + 1),
            ((((BigInteger.One << 260) + 1) << 31) + 1, (BigInteger.One << 260) + 1),
            (-((((BigInteger.One << 260) + 1) << 31) + 1), (BigInteger.One << 260) + 1),
        };

        foreach ((BigInteger numerator, BigInteger denominator) in ratios)
        {
            Assert.Equal(
                RoundWideRatioToFixed(numerator, denominator),
                Fixed64.GetSignedRatio(ToSigned320(numerator), ToSigned320(denominator)));
        }

        BigInteger left = (BigInteger.One << 258) + 17;
        BigInteger right = -((BigInteger.One << 257) + 91);
        Assert.Equal(
            left - right,
            ToBigInteger(WideArithmetic.SubtractSigned320(ToSigned320(left), ToSigned320(right))));

        int fractionalBits = FixedMath.SHIFT_AMOUNT_I + 1;
        BigInteger half = BigInteger.One << (fractionalBits - 1);
        Assert.Equal(
            Fixed64.MaxValue,
            Fixed64.RoundSquareRootToFixed(ToSigned192(BigInteger.One << 128), default, fractionalBits));
        Assert.Equal(
            Fixed64.MaxValue,
            Fixed64.RoundSquareRootToFixed(
                ToSigned192((BigInteger.One << 63) << fractionalBits),
                default,
                fractionalBits));
        Assert.Equal(
            Fixed64.MaxValue,
            Fixed64.RoundSquareRootToFixed(
                ToSigned192(((BigInteger)long.MaxValue << fractionalBits) + half),
                ToSigned192(BigInteger.One),
                fractionalBits));

        Assert.Equal(
            0,
            WideArithmetic.CompareNormalizedComponentToMidpoint(
                ToSigned320(BigInteger.One),
                ToSigned320(BigInteger.One << 66),
                0UL));
        Assert.Equal(
            -1,
            WideArithmetic.CompareNormalizedComponentToMidpoint(
                ToSigned320(BigInteger.One),
                ToSigned320(BigInteger.One << 200),
                0UL));
        Assert.Equal(
            0,
            WideArithmetic.CompareNormalizedComponentToMidpoint(
                ToSigned320(BigInteger.One << 134),
                ToSigned320(BigInteger.One << 200),
                0UL));
        Assert.Equal(
            1,
            WideArithmetic.CompareNormalizedComponentToMidpoint(
                ToSigned320(BigInteger.One << 135),
                ToSigned320(BigInteger.One << 200),
                0UL));
        Assert.Equal(
            -1,
            WideArithmetic.CompareNormalizedComponentToMidpoint(
                ToSigned320(BigInteger.One),
                ToSigned320(BigInteger.One << 200),
                uint.MaxValue));
        Assert.Equal(
            2,
            Fixed64.NormalizeWideComponent(
                ToSigned192(3),
                ToSigned320(9),
                ToSigned192(BigInteger.One << 33),
                ToSigned320(BigInteger.One << 66)).m_rawValue);
        Assert.Equal(
            0,
            Fixed64.NormalizeWideComponent(
                ToSigned192(BigInteger.One),
                ToSigned320(BigInteger.One),
                ToSigned192(BigInteger.One << 33),
                ToSigned320(BigInteger.One << 66)).m_rawValue);
        BigInteger normalizedComponent = BigInteger.Parse("2501680466859034032");
        BigInteger normalizedMagnitude = BigInteger.Parse("11460232859870921725");
        BigInteger normalizedSquaredMagnitude = normalizedMagnitude * normalizedMagnitude;
        Assert.Equal(
            NormalizedComponentRaw(normalizedComponent, normalizedSquaredMagnitude),
            Fixed64.NormalizeWideComponent(
                ToSigned192(normalizedComponent),
                ToSigned320(normalizedComponent * normalizedComponent),
                ToSigned192(normalizedMagnitude),
                ToSigned320(normalizedSquaredMagnitude)).m_rawValue);

        AssertShiftedPrefix(123, 0);
        AssertShiftedPrefix(BigInteger.One << 65, 2);
        AssertShiftedPrefix((BigInteger.One << 65) + 3, 2);
        AssertShiftedPrefix((BigInteger.One << 127) + (BigInteger.One << 64), 64);
        AssertShiftedPrefix((BigInteger.One << 128) + (BigInteger.One << 64), 67);
        AssertShiftedPrefix((BigInteger.One << 129) + (BigInteger.One << 65) + 3, 67);

        Assert.Equal(
            BigInteger.One,
            ToBigInteger(WideGeometry.GetDifferenceDotProduct3D(
                Fixed64.FromRaw(2), Fixed64.Zero,
                Fixed64.FromRaw(1), Fixed64.Zero,
                Fixed64.Zero, Fixed64.Zero,
                Fixed64.FromRaw(2), Fixed64.Zero,
                Fixed64.FromRaw(-3), Fixed64.Zero,
                Fixed64.Zero, Fixed64.Zero)));
        Assert.Equal(
            BigInteger.One,
            ToBigInteger(WideGeometry.GetDifferenceDotProduct3D(
                Fixed64.FromRaw(1), Fixed64.Zero,
                Fixed64.FromRaw(2), Fixed64.Zero,
                Fixed64.Zero, Fixed64.Zero,
                Fixed64.FromRaw(-3), Fixed64.Zero,
                Fixed64.FromRaw(2), Fixed64.Zero,
                Fixed64.Zero, Fixed64.Zero)));
    }

    [Fact]
    public void TryGetSignedRatio_ReportsFinalHalfEvenRepresentabilityAcrossSigned320Domain()
    {
        BigInteger scale = BigInteger.One << FixedMath.SHIFT_AMOUNT_I;
        BigInteger denominator = scale * 2;
        BigInteger positiveLimit = long.MaxValue;
        BigInteger negativeLimitMagnitude = BigInteger.One << 63;
        BigInteger signed320Minimum = -(BigInteger.One << 319);

        Assert.True(Fixed64.TryGetSignedRatio(
            ToSigned320(BigInteger.Zero), ToSigned320(denominator), out Fixed64 zero));
        Assert.Equal(Fixed64.Zero, zero);
        Assert.False(Fixed64.TryGetSignedRatio(ToSigned320(BigInteger.One), default, out _));

        Assert.True(Fixed64.TryGetSignedRatio(
            ToSigned320(positiveLimit * 2), ToSigned320(denominator), out Fixed64 maximum));
        Assert.Equal(Fixed64.MaxValue, maximum);
        Assert.False(Fixed64.TryGetSignedRatio(
            ToSigned320(positiveLimit * 2 + 1), ToSigned320(denominator), out _));

        Assert.True(Fixed64.TryGetSignedRatio(
            ToSigned320(-(negativeLimitMagnitude * 2 + 1)),
            ToSigned320(denominator),
            out Fixed64 minimum));
        Assert.Equal(Fixed64.MinValue, minimum);
        Assert.False(Fixed64.TryGetSignedRatio(
            ToSigned320(-(negativeLimitMagnitude * 2 + 2)),
            ToSigned320(denominator),
            out _));

        Assert.True(Fixed64.TryGetSignedRatio(
            ToSigned320(BigInteger.One), ToSigned320(signed320Minimum), out Fixed64 tiny));
        Assert.Equal(Fixed64.Zero, tiny);
        Assert.True(Fixed64.TryGetSignedRatio(
            ToSigned320(signed320Minimum), ToSigned320(signed320Minimum), out Fixed64 one));
        Assert.Equal(Fixed64.One, one);
    }

    [Fact]
    public void WideSquaredDistanceConversion_MatchesNearestEvenAndSaturationOracle()
    {
        BigInteger[] values =
        {
            BigInteger.Zero,
            BigInteger.One << 31,
            (BigInteger.One << 31) + 1,
            (BigInteger.One << 32) + (BigInteger.One << 31),
            ((BigInteger)long.MaxValue << 32) - 1,
            ((BigInteger)long.MaxValue << 32) + (BigInteger.One << 31),
            BigInteger.One << 129,
        };

        foreach (BigInteger value in values)
        {
            Assert.Equal(
                RoundSquaredDistanceRawToFixed(value),
                Fixed64.RoundSquaredDistance(ToSigned192(value)));
        }

        Assert.True(WideGeometry.IsSquaredLengthDegenerate(ToSigned192(BigInteger.One << 31)));
        Assert.False(WideGeometry.IsSquaredLengthDegenerate(ToSigned192((BigInteger.One << 31) + 1)));
        Assert.Equal(Fixed64.Zero, Fixed64.RoundSquaredDistance(ToSigned192(-BigInteger.One)));
    }

    [Fact]
    public void TryGetUnitIntervalRatio_UsesWideSignRangeAndNearestEvenRounding()
    {
        AssertWideRatioRejected(BigInteger.Zero, BigInteger.Zero);
        AssertWideRatioRejected(BigInteger.One, -2);
        AssertWideRatioRejected(-BigInteger.One, 2);
        AssertWideRatioRejected(3, 2);

        (BigInteger Numerator, BigInteger Denominator)[] cases =
        {
            (BigInteger.Zero, 7),
            (7, 7),
            (1, 2),
            (1, BigInteger.One << 33),
            (3, BigInteger.One << 33),
            (1, 3),
            (2, 3),
            (-((BigInteger.One << 129) + 17), -((BigInteger.One << 130) + 91)),
            ((BigInteger.One << 190) - 13, (BigInteger.One << 190) - 1),
        };

        foreach ((BigInteger numerator, BigInteger denominator) in cases)
        {
            Assert.True(Fixed64.TryGetUnitIntervalRatio(
                ToSigned192(numerator),
                ToSigned192(denominator),
                out Fixed64 result));
            Assert.Equal(RoundUnitRatioRawToEven(numerator, denominator), result.m_rawValue);
        }

        Assert.True(WideArithmetic.CompareMagnitude(ToSigned192(-9), ToSigned192(8)) > 0);
        Assert.True(WideArithmetic.CompareMagnitude(ToSigned192(-8), ToSigned192(9)) < 0);
        Assert.Equal(0, WideArithmetic.CompareMagnitude(ToSigned192(-9), ToSigned192(9)));
        Assert.Equal(
            0,
            WideArithmetic.CompareMagnitude(
                ToSigned192(-(BigInteger.One << 65)),
                ToSigned192(BigInteger.One << 65)));
        Assert.Equal(
            0,
            WideArithmetic.CompareMagnitude(
                ToSigned192(-(BigInteger.One << 128)),
                ToSigned192(BigInteger.One << 128)));
    }

    [Fact]
    public void TryGetUnitIntervalRatio_TwoWordMagnitudesMatchBigIntegerOracle()
    {
        var random = new System.Random(0x1280_0033);
        var bytes = new byte[16];
        BigInteger maximum = (BigInteger.One << 127) - 1;

        for (int i = 0; i < 512; i++)
        {
            random.NextBytes(bytes);
            BigInteger denominator = new BigInteger(bytes, isUnsigned: true, isBigEndian: false)
                & maximum;
            denominator |= BigInteger.One << 64;

            random.NextBytes(bytes);
            BigInteger numerator = new BigInteger(bytes, isUnsigned: true, isBigEndian: false)
                % denominator;
            if ((i & 1) != 0)
            {
                numerator = -numerator;
                denominator = -denominator;
            }

            Assert.True(Fixed64.TryGetUnitIntervalRatio(
                ToSigned192(numerator),
                ToSigned192(denominator),
                out Fixed64 result));
            Assert.Equal(RoundUnitRatioRawToEven(numerator, denominator), result.m_rawValue);
        }
    }

    private static void AssertWideRatioRejected(BigInteger numerator, BigInteger denominator)
    {
        Assert.False(Fixed64.TryGetUnitIntervalRatio(
            ToSigned192(numerator),
            ToSigned192(denominator),
            out Fixed64 result));
        Assert.Equal(default, result);
    }

    private static Signed192 ToSigned192(BigInteger value)
    {
        BigInteger modulus = BigInteger.One << 192;
        BigInteger encoded = value.Sign < 0 ? modulus + value : value;
        BigInteger wordMask = ulong.MaxValue;
        return new Signed192(
            (ulong)((encoded >> 128) & wordMask),
            (ulong)((encoded >> 64) & wordMask),
            (ulong)(encoded & wordMask));
    }

    private static BigInteger ToBigInteger(Signed192 value)
    {
        BigInteger encoded = ((BigInteger)value.High << 128)
            | ((BigInteger)value.Middle << 64)
            | value.Low;
        return value.Sign < 0 ? encoded - (BigInteger.One << 192) : encoded;
    }

    private static Signed320 ToSigned320(BigInteger value)
    {
        BigInteger modulus = BigInteger.One << 320;
        BigInteger encoded = value.Sign < 0 ? modulus + value : value;
        BigInteger wordMask = ulong.MaxValue;
        return new Signed320(
            (ulong)((encoded >> 256) & wordMask),
            (ulong)((encoded >> 192) & wordMask),
            (ulong)((encoded >> 128) & wordMask),
            (ulong)((encoded >> 64) & wordMask),
            (ulong)(encoded & wordMask));
    }

    private static BigInteger ToBigInteger(Signed320 value)
    {
        BigInteger encoded = ((BigInteger)value.Word4 << 256)
            | ((BigInteger)value.Word3 << 192)
            | ((BigInteger)value.Word2 << 128)
            | ((BigInteger)value.Word1 << 64)
            | value.Word0;
        return value.Sign < 0 ? encoded - (BigInteger.One << 320) : encoded;
    }

    private static long RoundUnitRatioRawToEven(BigInteger numerator, BigInteger denominator)
    {
        BigInteger quotient = BigInteger.DivRem(
            BigInteger.Abs(numerator) << FixedMath.SHIFT_AMOUNT_I,
            BigInteger.Abs(denominator),
            out BigInteger remainder);
        int midpointComparison = (remainder << 1).CompareTo(BigInteger.Abs(denominator));
        if (midpointComparison > 0 || (midpointComparison == 0 && !quotient.IsEven))
            quotient++;
        return (long)quotient;
    }

    private static Fixed64 RoundSquaredDistanceRawToFixed(BigInteger value)
    {
        BigInteger quotient = BigInteger.DivRem(
            value,
            BigInteger.One << FixedMath.SHIFT_AMOUNT_I,
            out BigInteger remainder);
        BigInteger half = BigInteger.One << (FixedMath.SHIFT_AMOUNT_I - 1);
        if (remainder > half || (remainder == half && !quotient.IsEven))
            quotient++;

        return quotient > long.MaxValue ? Fixed64.MaxValue : Fixed64.FromRaw((long)quotient);
    }

    private static Fixed64 RoundWideRawToFixed(BigInteger value, int fractionalBits)
    {
        BigInteger denominator = BigInteger.One << fractionalBits;
        BigInteger quotient = BigInteger.DivRem(BigInteger.Abs(value), denominator, out BigInteger remainder);
        int midpointComparison = (remainder << 1).CompareTo(denominator);
        if (midpointComparison > 0 || (midpointComparison == 0 && !quotient.IsEven))
            quotient++;
        if (value.Sign < 0)
            quotient = -quotient;
        if (quotient > long.MaxValue)
            return Fixed64.MaxValue;
        if (quotient < long.MinValue)
            return Fixed64.MinValue;
        return Fixed64.FromRaw((long)quotient);
    }

    private static Fixed64 RoundWideRatioToFixed(BigInteger numerator, BigInteger denominator)
    {
        if (numerator.IsZero)
            return Fixed64.Zero;
        if (denominator.IsZero)
            return numerator.Sign < 0 ? Fixed64.MinValue : Fixed64.MaxValue;

        BigInteger quotient = BigInteger.DivRem(
            BigInteger.Abs(numerator) << FixedMath.SHIFT_AMOUNT_I,
            BigInteger.Abs(denominator),
            out BigInteger remainder);
        int midpointComparison = (remainder << 1).CompareTo(BigInteger.Abs(denominator));
        if (midpointComparison > 0 || (midpointComparison == 0 && !quotient.IsEven))
            quotient++;
        if (numerator.Sign != denominator.Sign)
            quotient = -quotient;
        if (quotient > long.MaxValue)
            return Fixed64.MaxValue;
        if (quotient < long.MinValue)
            return Fixed64.MinValue;
        return Fixed64.FromRaw((long)quotient);
    }

    private static BigInteger NextSignedGeometryDot(System.Random random, byte[] bytes)
    {
        random.NextBytes(bytes);
        bytes[16] &= 0x03;
        BigInteger value = new BigInteger(bytes, isUnsigned: true, isBigEndian: false);
        return random.Next(2) == 0 ? value : -value;
    }

    private static BigInteger IntegerSquareRoot(BigInteger value)
    {
        if (value.IsZero)
            return BigInteger.Zero;

        BigInteger current = BigInteger.One << (int)((value.GetBitLength() + 1) / 2);
        while (true)
        {
            BigInteger next = (current + (value / current)) >> 1;
            if (next >= current)
                return current;
            current = next;
        }
    }

    private static long NormalizedComponentRaw(BigInteger component, BigInteger squaredMagnitude)
    {
        BigInteger target = component * component << 64;
        BigInteger low = BigInteger.Zero;
        BigInteger high = BigInteger.One << 32;
        while (low < high)
        {
            BigInteger middle = (low + high + 1) >> 1;
            if ((middle * middle * squaredMagnitude) <= target)
                low = middle;
            else
                high = middle - 1;
        }

        BigInteger midpoint = (low << 1) + 1;
        int comparison = (target << 2).CompareTo(midpoint * midpoint * squaredMagnitude);
        if (comparison > 0 || (comparison == 0 && !low.IsEven))
            low++;
        return component.Sign < 0 ? -(long)low : (long)low;
    }

    private static void AssertShiftedPrefix(BigInteger value, int bits)
    {
        ulong actual = WideArithmetic.ShiftRightToUInt64(ToSigned192(value), bits, out bool discarded);
        BigInteger mask = bits == 0 ? BigInteger.Zero : (BigInteger.One << bits) - 1;
        Assert.Equal((ulong)(value >> bits), actual);
        Assert.Equal((value & mask) != 0, discarded);
    }

    private static void AssertRoundGuardedQuotient(
        ulong guardedQuotient,
        bool hasTrailingRemainder,
        ulong expected,
        bool expectedOverflow)
    {
        ulong result = Fixed64.RoundGuardedQuotientToEven(
            guardedQuotient,
            hasTrailingRemainder,
            out bool overflowed);

        Assert.Equal(expected, result);
        Assert.Equal(expectedOverflow, overflowed);
    }

    private static void AssertDivisionMatchesOracle(long dividendRaw, long divisorRaw)
    {
        long expectedRaw = DivideRawToEven(dividendRaw, divisorRaw);
        Fixed64 dividend = Fixed64.FromRaw(dividendRaw);
        Fixed64 divisor = Fixed64.FromRaw(divisorRaw);

        Assert.Equal(expectedRaw, (dividend / divisor).m_rawValue);
        if (divisorRaw > 0)
            Assert.Equal(expectedRaw, FixedMath.FastDiv(dividend, divisor).m_rawValue);
    }

    private static long DivideRawToEven(long dividendRaw, long divisorRaw)
    {
        BigInteger divisorMagnitude = BigInteger.Abs(new BigInteger(divisorRaw));
        BigInteger quotient = BigInteger.DivRem(
            BigInteger.Abs(new BigInteger(dividendRaw)) << FixedMath.SHIFT_AMOUNT_I,
            divisorMagnitude,
            out BigInteger remainder);

        int midpointComparison = (remainder << 1).CompareTo(divisorMagnitude);
        if (midpointComparison > 0 || (midpointComparison == 0 && !quotient.IsEven))
            quotient++;

        bool negative = (dividendRaw < 0) ^ (divisorRaw < 0);
        BigInteger limit = negative ? BigInteger.One << 63 : long.MaxValue;
        if (quotient > limit)
            return negative ? long.MinValue : long.MaxValue;
        if (negative && quotient == limit)
            return long.MinValue;

        long result = (long)quotient;
        return negative ? -result : result;
    }

    private static void AssertMultiplyDivideMatchesOracle(
        long leftRaw,
        long rightRaw,
        long divisorRaw)
    {
        bool expectedSuccess = TryRoundRationalToInt64(
            (BigInteger)leftRaw * rightRaw,
            divisorRaw,
            out long expectedRaw);
        bool actualSuccess = Fixed64.TryMultiplyDivide(
            Fixed64.FromRaw(leftRaw),
            Fixed64.FromRaw(rightRaw),
            Fixed64.FromRaw(divisorRaw),
            out Fixed64 actual);

        Assert.Equal(expectedSuccess, actualSuccess);
        Assert.Equal(expectedSuccess ? expectedRaw : 0L, actual.m_rawValue);
    }

    private static void AssertMultiplyDivideMatchesOracle(
        long firstRaw,
        long secondRaw,
        long thirdRaw,
        long divisorRaw,
        bool requireRepresentable = false)
    {
        bool expectedSuccess = TryRoundRationalToInt64(
            (BigInteger)firstRaw * secondRaw * thirdRaw,
            (BigInteger)divisorRaw << FixedMath.SHIFT_AMOUNT_I,
            out long expectedRaw);
        if (requireRepresentable)
            Assert.True(expectedSuccess);

        bool actualSuccess = Fixed64.TryMultiplyDivide(
            Fixed64.FromRaw(firstRaw),
            Fixed64.FromRaw(secondRaw),
            Fixed64.FromRaw(thirdRaw),
            Fixed64.FromRaw(divisorRaw),
            out Fixed64 actual);

        Assert.Equal(expectedSuccess, actualSuccess);
        Assert.Equal(expectedSuccess ? expectedRaw : 0L, actual.m_rawValue);
    }

    private static bool TryRoundRationalToInt64(
        BigInteger numerator,
        BigInteger denominator,
        out long result)
    {
        if (denominator.IsZero)
        {
            result = default;
            return false;
        }

        BigInteger quotient = BigInteger.DivRem(
            BigInteger.Abs(numerator),
            BigInteger.Abs(denominator),
            out BigInteger remainder);
        int midpointComparison = (remainder << 1).CompareTo(BigInteger.Abs(denominator));
        if (midpointComparison > 0 || (midpointComparison == 0 && !quotient.IsEven))
            quotient++;

        if ((numerator.Sign < 0) != (denominator.Sign < 0))
            quotient = -quotient;
        if (quotient < long.MinValue || quotient > long.MaxValue)
        {
            result = default;
            return false;
        }

        result = (long)quotient;
        return true;
    }

    #endregion

    #region Test: Comparison Operators (<, <=, >, >=, ==, !=)

    [Fact]
    public void GreaterThan_Fixed64Values_ReturnsTrue()
    {
        var a = new Fixed64(5);
        var b = new Fixed64(3);
        Assert.True(a > b);
    }

    [Fact]
    public void LessThanOrEqual_Fixed64Values_ReturnsTrue()
    {
        var a = new Fixed64(3);
        var b = new Fixed64(5);
        Assert.True(a <= b);
    }

    [Fact]
    public void Equality_Fixed64Values_ReturnsTrue()
    {
        var a = new Fixed64(5);
        var b = new Fixed64(5);
        Assert.True(a == b);
    }

    [Fact]
    public void NotEquality_Fixed64Values_ReturnsTrue()
    {
        var a = new Fixed64(5);
        var b = new Fixed64(3);
        Assert.True(a != b);
    }

    #endregion

    #region Test: Implicit and Explicit Conversions

    [Fact]
    public void Convert_FromInteger_ReturnsCorrectFixed64()
    {
        Fixed64 result = (Fixed64)5;
        Assert.Equal(new Fixed64(5), result);
    }

    [Fact]
    public void Convert_FromDouble_ReturnsCorrectFixed64()
    {
        Fixed64 result = (Fixed64)5.5f;
        Assert.Equal(Fixed64.FromDouble(5.5f), result);
    }

    [Fact]
    public void Convert_FromFloatingPoint_RejectsNonFiniteAndOutOfRangeValues()
    {
        Assert.Equal(new Fixed64(2147483647), Fixed64.FromDouble(2147483647d));
        Assert.Equal(Fixed64.MinValue, Fixed64.FromDouble(-2147483648d));

        Assert.Throws<ArgumentOutOfRangeException>(() => Fixed64.FromDouble(double.NaN));
        Assert.Throws<ArgumentOutOfRangeException>(() => Fixed64.FromDouble(double.PositiveInfinity));
        Assert.Throws<ArgumentOutOfRangeException>(() => Fixed64.FromDouble(double.NegativeInfinity));
        Assert.Throws<ArgumentOutOfRangeException>(() => (Fixed64)float.NaN);

        Assert.Throws<OverflowException>(() => Fixed64.FromDouble(2147483648d));
        Assert.Throws<OverflowException>(() => Fixed64.FromDouble(-2147483649d));
        Assert.Throws<OverflowException>(() => (Fixed64)float.MaxValue);
    }

    [Fact]
    public void Convert_ToDouble_ReturnsCorrectDouble()
    {
        var fixedValue = Fixed64.FromDouble(5.5f);
        double result = (double)fixedValue;
        Assert.Equal(5.5, result);
    }

    #endregion

    #region Test: Fraction Method

    [Fact]
    public void Fraction_CreatesCorrectFixed64Value()
    {
        var result = Fixed64.FromFraction(1, 2);
        Assert.Equal(Fixed64.FromDouble(0.5f), result);
    }

    [Fact]
    public void Fraction_RejectsNonFiniteResults()
    {
        Assert.Throws<ArgumentOutOfRangeException>(() => Fixed64.FromFraction(1, 0));
        Assert.Throws<ArgumentOutOfRangeException>(() => Fixed64.FromFraction(0, 0));
    }

    #endregion

    #region Test: Arithmetic Overflow Protection

    [Fact]
    public void Add_OverflowProtection_ReturnsMaxValue()
    {
        var a = Fixed64.MaxValue;
        var b = new Fixed64(1);
        var result = a + b;
        Assert.Equal(Fixed64.MaxValue, result);
    }

    [Fact]
    public void Add_NegativeOverflowProtection_ReturnsMinValue()
    {
        var result = Fixed64.MinValue + -Fixed64.One;

        Assert.Equal(Fixed64.MinValue, result);
    }

    [Fact]
    public void Subtract_OverflowProtection_ReturnsMinValue()
    {
        var a = Fixed64.MinValue;
        var b = new Fixed64(1);
        var result = a - b;
        Assert.Equal(Fixed64.MinValue, result);
    }

    [Fact]
    public void Subtract_PositiveOverflowProtection_ReturnsMaxValue()
    {
        var result = Fixed64.MaxValue - -Fixed64.One;

        Assert.Equal(Fixed64.MaxValue, result);
    }

    [Fact]
    public void TryAdd_ExactResultsIncludingRepresentableLimits_Succeeds()
    {
        Assert.True(Fixed64.TryAdd(Fixed64.One, Fixed64.Two, out Fixed64 ordinaryResult));
        Assert.Equal(Fixed64.Three, ordinaryResult);

        Assert.True(Fixed64.TryAdd(
            Fixed64.FromRaw(long.MaxValue - 1),
            Fixed64.MinIncrement,
            out Fixed64 maximumResult));
        Assert.Equal(Fixed64.MaxValue, maximumResult);

        Assert.True(Fixed64.TryAdd(
            Fixed64.FromRaw(long.MinValue + 1),
            Fixed64.FromRaw(-1),
            out Fixed64 minimumResult));
        Assert.Equal(Fixed64.MinValue, minimumResult);
    }

    [Fact]
    public void TryAdd_Overflow_ReturnsFalseAndDefaultWithoutChangingOperatorSaturation()
    {
        Assert.False(Fixed64.TryAdd(
            Fixed64.MaxValue,
            Fixed64.MinIncrement,
            out Fixed64 positiveResult));
        Assert.Equal(default, positiveResult);
        Assert.Equal(Fixed64.MaxValue, Fixed64.MaxValue + Fixed64.MinIncrement);

        Fixed64 negativeIncrement = Fixed64.FromRaw(-1);
        Assert.False(Fixed64.TryAdd(
            Fixed64.MinValue,
            negativeIncrement,
            out Fixed64 negativeResult));
        Assert.Equal(default, negativeResult);
        Assert.Equal(Fixed64.MinValue, Fixed64.MinValue + negativeIncrement);
    }

    [Fact]
    public void TrySubtract_ExactResultsIncludingRepresentableLimits_Succeeds()
    {
        Assert.True(Fixed64.TrySubtract(Fixed64.Three, Fixed64.One, out Fixed64 ordinaryResult));
        Assert.Equal(Fixed64.Two, ordinaryResult);

        Assert.True(Fixed64.TrySubtract(
            Fixed64.FromRaw(long.MaxValue - 1),
            Fixed64.FromRaw(-1),
            out Fixed64 maximumResult));
        Assert.Equal(Fixed64.MaxValue, maximumResult);

        Assert.True(Fixed64.TrySubtract(
            Fixed64.FromRaw(long.MinValue + 1),
            Fixed64.MinIncrement,
            out Fixed64 minimumResult));
        Assert.Equal(Fixed64.MinValue, minimumResult);
    }

    [Fact]
    public void TrySubtract_Overflow_ReturnsFalseAndDefaultWithoutChangingOperatorSaturation()
    {
        Fixed64 negativeIncrement = Fixed64.FromRaw(-1);
        Assert.False(Fixed64.TrySubtract(
            Fixed64.MaxValue,
            negativeIncrement,
            out Fixed64 positiveResult));
        Assert.Equal(default, positiveResult);
        Assert.Equal(Fixed64.MaxValue, Fixed64.MaxValue - negativeIncrement);

        Assert.False(Fixed64.TrySubtract(
            Fixed64.MinValue,
            Fixed64.MinIncrement,
            out Fixed64 negativeResult));
        Assert.Equal(default, negativeResult);
        Assert.Equal(Fixed64.MinValue, Fixed64.MinValue - Fixed64.MinIncrement);
    }

    [Fact]
    public void TryAddSubtract_ExactResultsIgnoreIntermediateOverflow()
    {
        Assert.True(Fixed64.TryAddSubtract(
            Fixed64.Three,
            Fixed64.Two,
            Fixed64.One,
            out Fixed64 ordinaryResult));
        Assert.Equal((Fixed64)4, ordinaryResult);

        Assert.True(Fixed64.TryAddSubtract(
            Fixed64.MaxValue,
            Fixed64.One,
            Fixed64.One,
            out Fixed64 positiveCancellation));
        Assert.Equal(Fixed64.MaxValue, positiveCancellation);

        Assert.True(Fixed64.TryAddSubtract(
            Fixed64.MinValue,
            -Fixed64.One,
            -Fixed64.One,
            out Fixed64 negativeCancellation));
        Assert.Equal(Fixed64.MinValue, negativeCancellation);
    }

    [Fact]
    public void TryAddSubtract_FinalOverflow_ReturnsFalseAndDefault()
    {
        Assert.False(Fixed64.TryAddSubtract(
            Fixed64.MaxValue,
            Fixed64.One,
            Fixed64.Zero,
            out Fixed64 positiveResult));
        Assert.Equal(default, positiveResult);

        Assert.False(Fixed64.TryAddSubtract(
            Fixed64.MinValue,
            -Fixed64.One,
            Fixed64.Zero,
            out Fixed64 negativeResult));
        Assert.Equal(default, negativeResult);
    }

    [Fact]
    public void TryAddSubtract_RawDomainBoundariesMatchBigIntegerOracle()
    {
        long[] rawValues =
        {
            long.MinValue,
            long.MinValue + 1,
            -FixedMath.ONE_L,
            -1,
            0,
            1,
            FixedMath.ONE_L,
            long.MaxValue - 1,
            long.MaxValue
        };

        foreach (long firstRaw in rawValues)
        {
            foreach (long secondRaw in rawValues)
            {
                foreach (long subtrahendRaw in rawValues)
                {
                    BigInteger expected = (BigInteger)firstRaw + secondRaw - subtrahendRaw;
                    bool representable = expected >= long.MinValue && expected <= long.MaxValue;
                    bool succeeded = Fixed64.TryAddSubtract(
                        Fixed64.FromRaw(firstRaw),
                        Fixed64.FromRaw(secondRaw),
                        Fixed64.FromRaw(subtrahendRaw),
                        out Fixed64 result);

                    Assert.Equal(representable, succeeded);
                    Assert.Equal(
                        representable ? Fixed64.FromRaw((long)expected) : default,
                        result);
                }
            }
        }
    }

    [Fact]
    public void TrySubtractSums_IgnoresIntermediateOverflowAndRejectsFinalOverflow()
    {
        Assert.True(Fixed64.TrySubtractSums(
            Fixed64.MaxValue,
            Fixed64.MaxValue,
            Fixed64.MaxValue,
            Fixed64.MaxValue,
            out Fixed64 cancelled));
        Assert.Equal(Fixed64.Zero, cancelled);

        Assert.True(Fixed64.TrySubtractSums(
            Fixed64.MaxValue,
            Fixed64.One,
            Fixed64.MaxValue,
            Fixed64.Zero,
            out Fixed64 one));
        Assert.Equal(Fixed64.One, one);

        Assert.False(Fixed64.TrySubtractSums(
            Fixed64.MaxValue,
            Fixed64.MaxValue,
            Fixed64.MinValue,
            Fixed64.MinValue,
            out Fixed64 overflow));
        Assert.Equal(default, overflow);
    }

    [Fact]
    public void Multiply_PositiveOverflowProtection_ReturnsMaxValue()
    {
        Assert.Equal(Fixed64.MaxValue, Fixed64.MaxValue * Fixed64.Two);
        Assert.Equal(Fixed64.MaxValue, Fixed64.MaxValue * Fixed64.MaxValue);
        Assert.Equal(Fixed64.MaxValue, Fixed64.FromRaw(1L << 47) * Fixed64.FromRaw(1L << 48));
    }

    [Fact]
    public void Multiply_NegativeOverflowProtection_ReturnsMinValue()
    {
        Assert.Equal(Fixed64.MinValue, Fixed64.MinValue * Fixed64.Two);
    }

    [Fact]
    public void Modulus_MinRawByNegativeOne_ReturnsZero()
    {
        Assert.Equal(Fixed64.Zero, Fixed64.FromRaw(long.MinValue) % Fixed64.FromRaw(-1));
    }

    #endregion

    #region Test: Operations

    [Fact]
    public void IsInteger_PositiveInteger_ReturnsTrue()
    {
        var a = new Fixed64(42);
        Assert.True(a.IsInteger());
    }

    [Fact]
    public void IsInteger_NegativeInteger_ReturnsTrue()
    {
        var a = new Fixed64(-42);
        Assert.True(a.IsInteger());
    }

    [Fact]
    public void IsInteger_WhenZero_ReturnsTrue()
    {
        var a = Fixed64.Zero;
        Assert.True(a.IsInteger());
    }

    [Fact]
    public void IsInteger_PositiveDecimal_ReturnsFalse()
    {
        var a = Fixed64.FromDouble(4.2f);
        Assert.False(a.IsInteger());
    }

    [Fact]
    public void IsInteger_NegativeDecimal_ReturnsFalse()
    {
        var a = Fixed64.FromDouble(-4.2f);
        Assert.False(a.IsInteger());
    }

    [Fact]
    public void PostIncrementAndPostDecrement_ReturnOriginalValuesAndMutateTarget()
    {
        var value = Fixed64.FromDouble(2);

        var postIncrement = value++;
        Assert.Equal(Fixed64.FromDouble(2), postIncrement);
        Assert.Equal(Fixed64.FromDouble(3), value);

        var postDecrement = value--;
        Assert.Equal(Fixed64.FromDouble(3), postDecrement);
        Assert.Equal(Fixed64.FromDouble(2), value);
    }

    [Fact]
    public void PreIncrementAndPreDecrement_WorkCorrectly()
    {
        var value = new Fixed64(2);

        var incremented = ++value;
        Assert.Equal(new Fixed64(3), incremented);
        Assert.Equal(new Fixed64(3), value);

        var decremented = --value;
        Assert.Equal(new Fixed64(2), decremented);
        Assert.Equal(new Fixed64(2), value);
    }

    [Fact]
    public void Sign_ReturnsExpectedValue()
    {
        Assert.Equal(-1, Fixed64.Sign(new Fixed64(-2)));
        Assert.Equal(0, Fixed64.Sign(Fixed64.Zero));
        Assert.Equal(1, Fixed64.Sign(new Fixed64(2)));
    }

    [Fact]
    public void ExplicitConversionsAndRawConverters_WorkCorrectly()
    {
        Fixed64 fromLong = (Fixed64)5L;
        Assert.Equal(new Fixed64(5), fromLong);
        Assert.Equal(new Fixed64(-5), (Fixed64)(-5L));
        Assert.Equal(5L, (long)Fixed64.FromDouble(5.75));
        Assert.Equal(-5L, (long)Fixed64.FromDouble(-5.75));

        Fixed64 fromInt = (Fixed64)6;
        Assert.Equal(new Fixed64(6), fromInt);
        Assert.Equal(6, (int)Fixed64.FromDouble(6.75));

        Fixed64 fromFloat = (Fixed64)5.5f;
        Assert.Equal(Fixed64.FromDouble(5.5), fromFloat);
        Assert.Equal(5.5f, (float)fromFloat);

        Fixed64 fromDouble = (Fixed64)7.25;
        Assert.Equal(Fixed64.FromDouble(7.25), fromDouble);
        Assert.Equal(7.25, (double)fromDouble);

        Fixed64 fromDecimal = (Fixed64)8.5m;
        Assert.Equal(Fixed64.FromDouble(8.5), fromDecimal);
        Assert.True(Math.Abs((decimal)fromDecimal - 8.5m) < 0.000001m);

        Assert.Equal(1.0, Fixed64.ToDouble(Fixed64.One.m_rawValue));
        Assert.Equal(1.0f, Fixed64.ToFloat(Fixed64.One.m_rawValue));
        Assert.True(Math.Abs(Fixed64.ToDecimal(Fixed64.One.m_rawValue) - 1.0m) < 0.000001m);

        Assert.Equal(5L, Fixed64.FromRaw(5).m_rawValue);
        Assert.NotEqual(Fixed64.FromRaw(5), fromLong);
        Assert.Equal(Fixed64.MaxValue, (Fixed64)((long)int.MaxValue + 1L));
        Assert.Equal(Fixed64.MinValue, (Fixed64)((long)int.MinValue - 1L));
        Assert.Equal(Fixed64.MaxValue, (Fixed64)long.MaxValue);
        Assert.Equal(Fixed64.MinValue, (Fixed64)long.MinValue);
    }

    [Fact]
    public void MixedOperators_WorkCorrectly()
    {
        var value = Fixed64.FromDouble(1.5f);

        Assert.Equal(Fixed64.FromDouble(3.5f), value + 2);
        Assert.Equal(Fixed64.FromDouble(3.5f), 2 + value);

        Assert.Equal(Fixed64.FromDouble(0.5f), value - 1);
        Assert.Equal(Fixed64.FromDouble(0.5f), 2 - value);

        Assert.Equal(new Fixed64(3), value * 2);
        Assert.Equal(new Fixed64(3), 2 * value);
        Assert.Equal(Fixed64.FromDouble(0.75), value / 2);
        Assert.Equal(Fixed64.FromDouble(1.3333333333), 2 / value);

        Assert.Equal(Fixed64.One, new Fixed64(7) % 3);
        Assert.Equal(Fixed64.One, 7 % new Fixed64(3));
        Assert.True(value > 1);
        Assert.True(2 > value);
        Assert.True(value < 2);
        Assert.True(1 < value);
        Assert.True(value >= 1);
        Assert.True(2 >= value);
        Assert.True(value <= 2);
        Assert.True(1 <= value);
        Assert.True(new Fixed64(2) == 2);
        Assert.True(2 == new Fixed64(2));
        Assert.True(value != 2);
        Assert.True(2 != value);
    }

    [Fact]
    public void ConstantsOffsetAndRawFormatting_ReturnExpectedValues()
    {
        Assert.Equal(-Fixed64.One, Fixed64.NegOne);
        Assert.Equal(Fixed64.FromDouble(0.25), Fixed64.Quarter);
        Assert.Equal(Fixed64.FromDouble(0.125), Fixed64.Eighth);
        FixedMathTestHelper.AssertWithinRelativeTolerance(Fixed64.One, Fixed64.InvertedPi * Fixed64.Pi);
        FixedMathTestHelper.AssertWithinRelativeTolerance(Fixed64.One, Fixed64.Rad2Deg * Fixed64.Deg2Rad);
        Assert.Equal(new Fixed64(-64), Fixed64.Log2Min);

        Fixed64 value = Fixed64.FromDouble(1.5);

        Assert.Equal(Fixed64.FromDouble(4.5), value.Offset(3));
        Assert.Equal(value.m_rawValue.ToString(CultureInfo.InvariantCulture), value.ToRawString());
        Assert.Equal(1, Fixed64.ToInt(value));
        Assert.False(value.Equals("not-fixed"));
    }

    [Fact]
    public void MultiplyOverflowProtection_ReturnsSaturatedBounds()
    {
        Assert.Equal(Fixed64.MaxValue, Fixed64.MaxValue * new Fixed64(2));
        Assert.Equal(Fixed64.MinValue, Fixed64.MaxValue * new Fixed64(-2));
        Assert.Equal(Fixed64.MaxValue, Fixed64.MinValue * -Fixed64.One);
        Assert.Equal(Fixed64.MinValue, Fixed64.MinValue * Fixed64.One);
    }

    [Fact]
    public void DivideOverflowProtection_ReturnsSaturatedBounds()
    {
        Assert.Equal(Fixed64.MaxValue, Fixed64.MaxValue / Fixed64.Epsilon);
        Assert.Equal(Fixed64.MinValue, new Fixed64(-1) * Fixed64.MaxValue / Fixed64.Epsilon);
    }

    [Fact]
    public void EdgeOperators_HandleSpecialCases()
    {
        Assert.Equal(Fixed64.Zero, Fixed64.MinValue % (-Fixed64.Epsilon));
        Assert.Equal(Fixed64.MaxValue, -Fixed64.MinValue);
        Assert.Equal(new Fixed64(2), Fixed64.One << 1);
        Assert.Equal(new Fixed64(2), new Fixed64(4) >> 1);
    }

    [Fact]
    public void Parsing_DecimalStrings_WorksCorrectly()
    {
        Assert.Equal(Fixed64.FromDouble(1.25), Fixed64.Parse("1.25"));
        Assert.Equal(Fixed64.FromDouble(1.25), Fixed64.Parse("1.25", null));
        Assert.Equal(Fixed64.FromDouble(-2.5), Fixed64.Parse("-2.5"));
        Assert.Equal(Fixed64.FromDouble(1234.5), Fixed64.Parse("1,234.5", CultureInfo.InvariantCulture));
        Assert.Throws<ArgumentNullException>(() => Fixed64.Parse(null!));
        Assert.Throws<FormatException>(() => Fixed64.Parse(string.Empty));
        Assert.Throws<FormatException>(() => Fixed64.Parse("abc"));
        Assert.Throws<FormatException>(() => Fixed64.Parse("NaN"));
        Assert.Throws<OverflowException>(() => Fixed64.Parse("2147483648", CultureInfo.InvariantCulture));

        Assert.True(Fixed64.TryParse("1.25", out var parsedPositive));
        Assert.Equal(Fixed64.FromDouble(1.25), parsedPositive);

        Assert.True(Fixed64.TryParse("1.25", null, out var parsedWithNullProvider));
        Assert.Equal(Fixed64.FromDouble(1.25), parsedWithNullProvider);

        Assert.True(Fixed64.TryParse("-2.5", out var parsedNegative));
        Assert.Equal(Fixed64.FromDouble(-2.5), parsedNegative);

        Assert.False(Fixed64.TryParse(string.Empty, out var emptyResult));
        Assert.Equal(Fixed64.Zero, emptyResult);

        Assert.False(Fixed64.TryParse("abc", out var invalidResult));
        Assert.Equal(Fixed64.Zero, invalidResult);

        Assert.False(Fixed64.TryParse("NaN", out var nonFiniteResult));
        Assert.Equal(Fixed64.Zero, nonFiniteResult);

        Assert.False(Fixed64.TryParse("2147483648", CultureInfo.InvariantCulture, out var overflowResult));
        Assert.Equal(Fixed64.Zero, overflowResult);
    }

    [Fact]
    public void Parsing_RawStrings_UsesExplicitRawApis()
    {
        var rawValue = Fixed64.One.m_rawValue.ToString();
        var negativeRawValue = (-Fixed64.One.m_rawValue).ToString();

        Assert.Equal(Fixed64.One, Fixed64.ParseRaw(rawValue));
        Assert.Equal(-Fixed64.One, Fixed64.ParseRaw(negativeRawValue));
        Assert.Throws<ArgumentNullException>(() => Fixed64.ParseRaw(null!));
        Assert.Throws<FormatException>(() => Fixed64.ParseRaw(string.Empty));
        Assert.Throws<FormatException>(() => Fixed64.ParseRaw("abc"));

        Assert.True(Fixed64.TryParseRaw(rawValue, out var parsedPositive));
        Assert.Equal(Fixed64.One, parsedPositive);

        Assert.True(Fixed64.TryParseRaw(negativeRawValue, out var parsedNegative));
        Assert.Equal(-Fixed64.One, parsedNegative);

        Assert.False(Fixed64.TryParseRaw(string.Empty, out var emptyResult));
        Assert.Equal(Fixed64.Zero, emptyResult);

        Assert.False(Fixed64.TryParseRaw("abc", out var invalidResult));
        Assert.Equal(Fixed64.Zero, invalidResult);
    }

    [Fact]
    public void DecimalConversion_UsesDecimalMathAndRejectsOutOfRangeValues()
    {
        Assert.Equal(Fixed64.FromDouble(1.25), Fixed64.FromDecimal(1.25m));
        Assert.Equal(Fixed64.MinIncrement, Fixed64.FromDecimal(FixedMath.SCALE_FACTOR_M));
        Assert.Equal(Fixed64.MinValue, Fixed64.FromDecimal(-2147483648m));
        Assert.Throws<OverflowException>(() => Fixed64.FromDecimal(2147483648m));
        Assert.Throws<OverflowException>(() => Fixed64.FromDecimal(-2147483649m));
        Assert.Equal(Fixed64.FromDecimal(1.25m), (Fixed64)1.25m);
    }

    [Fact]
    public void FormattingAndComparerMembers_WorkCorrectly()
    {
        var value = Fixed64.FromDouble(1.2345f);

        Assert.Equal("1.23", value.ToString("0.00"));
        Assert.Equal("1.23", value.ToString("0.00", null));
        Assert.Equal(0, value.CompareTo(Fixed64.FromDouble(1.2345f)));
        Assert.True(value.CompareTo(Fixed64.FromDouble(1.2f)) > 0);
        Assert.True(value.CompareTo(Fixed64.FromDouble(1.3f)) < 0);

        Fixed64 comparer = new();
        Assert.True(comparer.Equals(new Fixed64(2), new Fixed64(2)));
        Assert.False(comparer.Equals(new Fixed64(2), new Fixed64(3)));
        Assert.Equal(new Fixed64(2).GetHashCode(), comparer.GetHashCode(new Fixed64(2)));
    }

    [Fact]
    public void Fixed64Extensions_WrapperMethods_MatchUnderlyingImplementations()
    {
        var value = Fixed64.FromDouble(-1.75);
        var positive = Fixed64.FromDouble(1.75);

        Assert.Equal(Fixed64.Sign(value), value.Sign());
        Assert.Equal(Fixed64.IsInteger(positive), positive.IsInteger());
        Assert.Equal(FixedMath.Squared(positive), positive.Squared());
        Assert.Equal(FixedMath.Round(positive), positive.Round());
        Assert.Equal(FixedMath.Round(positive, MidpointRounding.AwayFromZero), positive.Round(MidpointRounding.AwayFromZero));
        Assert.Equal(FixedMath.RoundToPrecision(positive, 1), positive.RoundToPrecision(1));
        Assert.Equal(FixedMath.ClampOne(new Fixed64(3)), new Fixed64(3).ClampOne());
        Assert.Equal(FixedMath.Clamp01(new Fixed64(-1)), new Fixed64(-1).Clamp01());
        Assert.Equal(FixedMath.Abs(value), value.Abs());
        Assert.True(Fixed64.FromDouble(0.5).AbsLessThan(Fixed64.One));
        Assert.Equal(FixedMath.FastAdd(Fixed64.One, Fixed64.Two), Fixed64.One.FastAdd(Fixed64.Two));
        Assert.Equal(FixedMath.FastSub(Fixed64.Three, Fixed64.One), Fixed64.Three.FastSub(Fixed64.One));
        Assert.Equal(FixedMath.FastMul(Fixed64.Two, Fixed64.Three), Fixed64.Two.FastMul(Fixed64.Three));
        Assert.Equal(FixedMath.FastDiv(Fixed64.Three, Fixed64.Two), Fixed64.Three.FastDiv(Fixed64.Two));
        Assert.Equal(FixedMath.FastMod(new Fixed64(7), new Fixed64(3)), new Fixed64(7).FastMod(new Fixed64(3)));
        Assert.Equal(FixedMath.Lerp(Fixed64.Zero, Fixed64.Two, Fixed64.Half), Fixed64.Zero.Lerp(Fixed64.Two, Fixed64.Half));
        Assert.Equal(FixedMath.SmoothStep(Fixed64.Zero, new Fixed64(10), Fixed64.Half), Fixed64.Zero.SmoothStep(new Fixed64(10), Fixed64.Half));
        Assert.Equal(FixedMath.CubicInterpolate(Fixed64.Zero, new Fixed64(10), Fixed64.Two, Fixed64.Two, Fixed64.Half), Fixed64.Zero.CubicInterpolate(new Fixed64(10), Fixed64.Two, Fixed64.Two, Fixed64.Half));
        Assert.Equal(FixedMath.Floor(value), value.Floor());
        Assert.Equal(FixedMath.Ceil(value), value.Ceil());
        Assert.Equal((int)FixedMath.Round(positive), positive.RoundToInt());
        Assert.Equal((int)FixedMath.Ceil(value), value.CeilToInt());
        Assert.Equal((int)FixedMath.Floor(value), value.FloorToInt());
        Assert.Equal((long)FixedMath.Round(positive), positive.RoundToLong());
        Assert.Equal((long)FixedMath.Ceil(value), value.CeilToLong());
        Assert.Equal((long)FixedMath.Floor(value), value.FloorToLong());
        Assert.Equal(FixedMath.Clamp(value, Fixed64.Zero, Fixed64.One), value.Clamp(Fixed64.Zero, Fixed64.One));
        Assert.Equal(FixedMath.Sqrt(Fixed64.Two), Fixed64.Two.Sqrt());
    }

    [Fact]
    public void Fixed64Extensions_AngleHelpers_WorkCorrectly()
    {
        Assert.Equal(FixedMath.DegToRad(new Fixed64(90)), new Fixed64(90).ToRadians());
        Assert.Equal(FixedMath.RadToDeg(Fixed64.HalfPi), Fixed64.HalfPi.ToDegrees());
        Assert.Equal(FixedMath.Sin(Fixed64.HalfPi), Fixed64.HalfPi.Sin());
        Assert.Equal(FixedMath.Cos(Fixed64.HalfPi), Fixed64.HalfPi.Cos());
        Assert.Equal(FixedMath.Tan(Fixed64.PiOver4), Fixed64.PiOver4.Tan());
        Assert.Equal(FixedMath.Acos(Fixed64.Half), Fixed64.Half.Acos());
        Assert.Equal(FixedMath.Asin(Fixed64.Half), Fixed64.Half.Asin());
        Assert.Equal(FixedMath.Atan(Fixed64.One), Fixed64.One.Atan());
        Assert.Equal(FixedMath.Atan2(Fixed64.One, Fixed64.One), Fixed64.One.Atan2(Fixed64.One));
        Assert.Equal(FixedMath.GetHypotenuse(new Fixed64(3), new Fixed64(4)), new Fixed64(3).Hypot(new Fixed64(4)));
        Assert.Equal(FixedMath.Log2(Fixed64.Two), Fixed64.Two.Log2());
        Assert.Equal(FixedMath.Ln(Fixed64.Two), Fixed64.Two.Ln());
        Assert.Equal(FixedMath.Pow(Fixed64.Two, Fixed64.Three), Fixed64.Two.Pow(Fixed64.Three));
        Assert.Equal(FixedMath.Pow2(Fixed64.Three), Fixed64.Three.Pow2());
    }

    [Fact]
    public void Fixed64Extensions_EpsilonAndFuzzyHelpers_WorkCorrectly()
    {
        Assert.True((Fixed64.Epsilon + Fixed64.One).MoreThanEpsilon());
        Assert.False(Fixed64.Epsilon.MoreThanEpsilon());
        Assert.True(Fixed64.Zero.LessThanEpsilon());
        Assert.False(Fixed64.Epsilon.LessThanEpsilon());

        Assert.True(Fixed64.Zero.FuzzyComponentEqual(Fixed64.Epsilon, Fixed64.Epsilon));
        Assert.True(new Fixed64(100).FuzzyComponentEqual(new Fixed64(101), Fixed64.FromDouble(0.02)));
        Assert.False(new Fixed64(100).FuzzyComponentEqual(new Fixed64(103), Fixed64.FromDouble(0.02)));
    }

    #endregion

    #region Test: Serialization

    [Fact]
    public void Fixed64_NetSerialization_RoundTripMaintainsData()
    {
        var originalValue = Fixed64.Pi;

        var jsonOptions = new JsonSerializerOptions
        {
            DefaultIgnoreCondition = JsonIgnoreCondition.WhenWritingNull,
            ReferenceHandler = ReferenceHandler.IgnoreCycles
        };
        var json = JsonSerializer.SerializeToUtf8Bytes(originalValue, jsonOptions);
        var deserializedValue = JsonSerializer.Deserialize<Fixed64>(json, jsonOptions);

        // Check that deserialized values match the original
        Assert.Equal(originalValue, deserializedValue);
    }

#if !FIXEDMATHSHARP_DISABLE_MEMORYPACK
    [Fact]
    public void Fixed64_MemoryPackSerialization_RoundTripMaintainsData()
    {
        Fixed64 originalValue = Fixed64.Pi;

        byte[] bytes = MemoryPackSerializer.Serialize(originalValue);
        Fixed64 deserializedValue = MemoryPackSerializer.Deserialize<Fixed64>(bytes);

        // Check that deserialized values match the original
        Assert.Equal(originalValue, deserializedValue);
    }
#endif

    #endregion
}
