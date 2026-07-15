using MemoryPack;
using System;
using System.Globalization;
using System.Numerics;
using System.Text.Json;
using System.Text.Json.Serialization;
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

            BigInteger leftX = (BigInteger)leftEndX - leftStartX;
            BigInteger leftY = (BigInteger)leftEndY - leftStartY;
            BigInteger rightX = (BigInteger)rightEndX - rightStartX;
            BigInteger rightY = (BigInteger)rightEndY - rightStartY;

            Fixed64.Signed192 dot = Fixed64.GetDifferenceDotProduct2D(
                Fixed64.FromRaw(leftEndX), Fixed64.FromRaw(leftStartX),
                Fixed64.FromRaw(leftEndY), Fixed64.FromRaw(leftStartY),
                Fixed64.FromRaw(rightEndX), Fixed64.FromRaw(rightStartX),
                Fixed64.FromRaw(rightEndY), Fixed64.FromRaw(rightStartY));
            Fixed64.Signed192 cross = Fixed64.GetDifferenceCrossProduct2D(
                Fixed64.FromRaw(leftEndX), Fixed64.FromRaw(leftStartX),
                Fixed64.FromRaw(leftEndY), Fixed64.FromRaw(leftStartY),
                Fixed64.FromRaw(rightEndX), Fixed64.FromRaw(rightStartX),
                Fixed64.FromRaw(rightEndY), Fixed64.FromRaw(rightStartY));

            Assert.Equal((leftX * rightX) + (leftY * rightY), ToBigInteger(dot));
            Assert.Equal((leftX * rightY) - (leftY * rightX), ToBigInteger(cross));
        }
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

        Assert.True(Fixed64.CompareMagnitude(ToSigned192(-9), ToSigned192(8)) > 0);
        Assert.True(Fixed64.CompareMagnitude(ToSigned192(-8), ToSigned192(9)) < 0);
        Assert.Equal(0, Fixed64.CompareMagnitude(ToSigned192(-9), ToSigned192(9)));
        Assert.Equal(
            0,
            Fixed64.CompareMagnitude(
                ToSigned192(-(BigInteger.One << 65)),
                ToSigned192(BigInteger.One << 65)));
        Assert.Equal(
            0,
            Fixed64.CompareMagnitude(
                ToSigned192(-(BigInteger.One << 128)),
                ToSigned192(BigInteger.One << 128)));
    }

    private static void AssertWideRatioRejected(BigInteger numerator, BigInteger denominator)
    {
        Assert.False(Fixed64.TryGetUnitIntervalRatio(
            ToSigned192(numerator),
            ToSigned192(denominator),
            out Fixed64 result));
        Assert.Equal(default, result);
    }

    private static Fixed64.Signed192 ToSigned192(BigInteger value)
    {
        BigInteger modulus = BigInteger.One << 192;
        BigInteger encoded = value.Sign < 0 ? modulus + value : value;
        BigInteger wordMask = ulong.MaxValue;
        return new Fixed64.Signed192(
            (ulong)((encoded >> 128) & wordMask),
            (ulong)((encoded >> 64) & wordMask),
            (ulong)(encoded & wordMask));
    }

    private static BigInteger ToBigInteger(Fixed64.Signed192 value)
    {
        BigInteger encoded = ((BigInteger)value.High << 128)
            | ((BigInteger)value.Middle << 64)
            | value.Low;
        return value.Sign < 0 ? encoded - (BigInteger.One << 192) : encoded;
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
    public void CountLeadingZeroes_InternalHelper_ReturnsExpectedCounts()
    {
        var method = typeof(Fixed64).GetMethod(
            "CountLeadingZeroes",
            System.Reflection.BindingFlags.Static | System.Reflection.BindingFlags.NonPublic);

        Assert.NotNull(method);
        Assert.Equal(63, (int)method!.Invoke(null, new object[] { 1UL })!);
        Assert.Equal(0, (int)method.Invoke(null, new object[] { 0x8000000000000000UL })!);
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
