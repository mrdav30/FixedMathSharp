using System;
using Xunit;
using System.Numerics;

namespace FixedMathSharp.Tests;

public class FixedMathTests
{
    #region Test: CopySign Method

    [Fact]
    public void CopySign_PositiveToPositive_ReturnsPositive()
    {
        var result = FixedMath.CopySign(new Fixed64(5), new Fixed64(3));
        Assert.Equal(new Fixed64(5), result);
    }

    [Fact]
    public void CopySign_PositiveToNegative_ReturnsNegative()
    {
        var result = FixedMath.CopySign(new Fixed64(5), new Fixed64(-3));
        Assert.Equal(new Fixed64(-5), result);
    }

    #endregion

    #region Test: Clamp01 Method

    [Fact]
    public void Clamp01_ValueLessThanZero_ReturnsZero()
    {
        var result = FixedMath.Clamp01(new Fixed64(-1));
        Assert.Equal(Fixed64.Zero, result);
    }

    [Fact]
    public void Clamp01_ValueGreaterThanOne_ReturnsOne()
    {
        var result = FixedMath.Clamp01(new Fixed64(2));
        Assert.Equal(Fixed64.One, result);
    }

    [Fact]
    public void Clamp01_ValueInRange_ReturnsValue()
    {
        var result = FixedMath.Clamp01(Fixed64.FromDouble(0.5f));
        Assert.Equal(Fixed64.FromDouble(0.5f), result);
    }

    [Fact]
    public void ClampOne_ClampsToNegativeOneOneRange()
    {
        Assert.Equal(-Fixed64.One, FixedMath.ClampOne(Fixed64.FromDouble(-2)));
        Assert.Equal(Fixed64.FromDouble(0.5f), FixedMath.ClampOne(Fixed64.FromDouble(0.5f)));
        Assert.Equal(Fixed64.One, FixedMath.ClampOne(Fixed64.FromDouble(2)));
    }

    #endregion

    #region Test: FastAbs Method

    [Fact]
    public void FastAbs_PositiveValue_ReturnsSameValue()
    {
        var result = FixedMath.Abs(new Fixed64(10));
        Assert.Equal(new Fixed64(10), result);
    }

    [Fact]
    public void FastAbs_NegativeValue_ReturnsPositiveValue()
    {
        var result = FixedMath.Abs(new Fixed64(-10));
        Assert.Equal(new Fixed64(10), result);
    }

    [Fact]
    public void FastAbs_MinValue_ReturnsMaxValue()
    {
        Assert.Equal(Fixed64.MaxValue, FixedMath.Abs(Fixed64.MinValue));
    }

    #endregion

    #region Test: Ceiling Method

    [Fact]
    public void Ceiling_WithFraction_ReturnsNextInteger()
    {
        var result = FixedMath.Ceil(Fixed64.FromDouble(1.5));
        Assert.Equal(new Fixed64(2), result);
    }

    [Fact]
    public void Ceiling_ExactInteger_ReturnsSameInteger()
    {
        var result = FixedMath.Ceil(Fixed64.FromDouble(3.0));
        var test = new Fixed64(3);
        Assert.Equal(test, result);
    }

    #endregion

    #region Test: Max Method

    [Fact]
    public void Max_FirstValueLarger_ReturnsFirstValue()
    {
        var result = FixedMath.Max(new Fixed64(5), new Fixed64(3));
        Assert.Equal(new Fixed64(5), result);
    }

    [Fact]
    public void Max_SecondValueLarger_ReturnsSecondValue()
    {
        var result = FixedMath.Max(new Fixed64(3), new Fixed64(5));
        Assert.Equal(new Fixed64(5), result);
    }

    [Fact]
    public void Max_EqualValues_ReturnsEitherValue()
    {
        var result = FixedMath.Max(new Fixed64(5), new Fixed64(5));
        Assert.Equal(new Fixed64(5), result);
    }

    #endregion

    #region Test: Min Method

    [Fact]
    public void Min_FirstValueSmaller_ReturnsFirstValue()
    {
        var result = FixedMath.Min(new Fixed64(3), new Fixed64(5));
        Assert.Equal(new Fixed64(3), result);
    }

    [Fact]
    public void Min_SecondValueSmaller_ReturnsSecondValue()
    {
        var result = FixedMath.Min(new Fixed64(5), new Fixed64(3));
        Assert.Equal(new Fixed64(3), result);
    }

    [Fact]
    public void Min_EqualValues_ReturnsEitherValue()
    {
        var result = FixedMath.Min(new Fixed64(5), new Fixed64(5));
        Assert.Equal(new Fixed64(5), result);
    }

    #endregion

    #region Test: Average Method

    [Theory]
    [InlineData(long.MaxValue, long.MaxValue, long.MaxValue, long.MaxValue)]
    [InlineData(long.MinValue, long.MinValue, long.MinValue, long.MinValue)]
    [InlineData(long.MinValue, long.MaxValue, 0L, 0L)]
    [InlineData(long.MaxValue, long.MaxValue, long.MinValue, 3074457345618258602L)]
    [InlineData(0L, 0L, 1L, 0L)]
    [InlineData(0L, 0L, 2L, 1L)]
    [InlineData(0L, 0L, -1L, 0L)]
    [InlineData(0L, 0L, -2L, -1L)]
    public void Average_ExtremesAndRawRemainders_ReturnsNearest(
        long firstRaw,
        long secondRaw,
        long thirdRaw,
        long expectedRaw)
    {
        Fixed64 first = Fixed64.FromRaw(firstRaw);
        Fixed64 second = Fixed64.FromRaw(secondRaw);
        Fixed64 third = Fixed64.FromRaw(thirdRaw);

        Fixed64 result = FixedMath.Average(first, second, third);

        Assert.Equal(Fixed64.FromRaw(expectedRaw), result);
        Assert.Equal(result, FixedMath.Average(third, first, second));
        Assert.Equal(result, FixedMath.Average(second, third, first));
    }

    #endregion

    #region Test: Midpoint Method

    [Theory]
    [InlineData(long.MaxValue, long.MaxValue, long.MaxValue)]
    [InlineData(long.MinValue, long.MinValue, long.MinValue)]
    [InlineData(long.MinValue, long.MaxValue, 0L)]
    [InlineData(1L, 2L, 2L)]
    [InlineData(2L, 3L, 2L)]
    [InlineData(-2L, -1L, -2L)]
    [InlineData(-3L, -2L, -2L)]
    public void Midpoint_ExtremesAndRawTies_ReturnsNearestEven(long leftRaw, long rightRaw, long expectedRaw)
    {
        Fixed64 result = FixedMath.Midpoint(Fixed64.FromRaw(leftRaw), Fixed64.FromRaw(rightRaw));

        Assert.Equal(Fixed64.FromRaw(expectedRaw), result);
        Assert.Equal(result, FixedMath.Midpoint(Fixed64.FromRaw(rightRaw), Fixed64.FromRaw(leftRaw)));
    }

    #endregion

    #region Test: Round Method (Without Decimal Places)

    [Fact]
    public void Round_ToEven_RoundsToNearestEven()
    {
        var result = FixedMath.Round(Fixed64.FromDouble(2.5));
        Assert.Equal(new Fixed64(2), result);
    }

    [Fact]
    public void Round_AwayFromZero_RoundsUp()
    {
        var result = FixedMath.Round(Fixed64.FromDouble(2.5), MidpointRounding.AwayFromZero);
        Assert.Equal(new Fixed64(3), result);
    }

    [Fact]
    public void Round_ToEven_NegativeNumber_RoundsToNearestEven()
    {
        var result = FixedMath.Round(Fixed64.FromDouble(-2.5));
        Assert.Equal(new Fixed64(-2), result);
    }

    [Fact]
    public void Round_AwayFromZero_NegativeHalf_RoundsDown()
    {
        var result = FixedMath.Round(Fixed64.FromDouble(-2.5), MidpointRounding.AwayFromZero);
        Assert.Equal(new Fixed64(-3), result);
    }

    #endregion

    #region Test: Round Method (With Decimal Places)

    [Fact]
    public void Round_WithDecimalPlaces_RoundsToTwoDecimalPlaces()
    {
        var result = FixedMath.RoundToPrecision(Fixed64.FromDouble(2.556f), 2, MidpointRounding.AwayFromZero);
        Assert.Equal(Fixed64.FromDouble(2.56), result);
    }

    [Fact]
    public void Round_WithDecimalPlaces_RoundsToZeroDecimalPlaces_ToEven()
    {
        var result = FixedMath.RoundToPrecision(Fixed64.FromDouble(2.5), 0);
        Assert.Equal(new Fixed64(2), result);
    }

    [Fact]
    public void Round_WithDecimalPlaces_RoundsToZeroDecimalPlaces_AwayFromZero()
    {
        var result = FixedMath.RoundToPrecision(Fixed64.FromDouble(2.5), 0, MidpointRounding.AwayFromZero);
        Assert.Equal(new Fixed64(3), result);
    }

    [Fact]
    public void Round_WithDecimalPlaces_ThrowsWhenPrecisionIsOutOfRange()
    {
        Assert.Throws<ArgumentOutOfRangeException>(() => FixedMath.RoundToPrecision(Fixed64.FromDouble(1.23), -1));
        Assert.Throws<ArgumentOutOfRangeException>(() => FixedMath.RoundToPrecision(Fixed64.FromDouble(1.23), FixedMath.Pow10Lookup.Length));
    }

    [Fact]
    public void Pow10Lookup_Access_DoesNotAllocate()
    {
        _ = FixedMath.Pow10Lookup.Length;

        long before = GC.GetAllocatedBytesForCurrentThread();
        int length = FixedMath.Pow10Lookup.Length;
        long allocated = GC.GetAllocatedBytesForCurrentThread() - before;

        Assert.Equal(10, length);
        Assert.Equal(0, allocated);
    }

    #endregion

    #region Test: FastAdd Method

    [Fact]
    public void FastAdd_AddsTwoPositiveValues()
    {
        var a = new Fixed64(2);
        var b = new Fixed64(3);
        var result = FixedMath.FastAdd(a, b);
        Assert.Equal(new Fixed64(5), result);
    }

    [Fact]
    public void FastAdd_AddsNegativeAndPositiveValue()
    {
        var a = new Fixed64(-2);
        var b = new Fixed64(3);
        var result = FixedMath.FastAdd(a, b);
        Assert.Equal(new Fixed64(1), result);
    }

    [Fact]
    public void FastAdd_AddsTwoNegativeValues()
    {
        var a = new Fixed64(-5);
        var b = new Fixed64(-3);
        var result = FixedMath.FastAdd(a, b);
        Assert.Equal(new Fixed64(-8), result);
    }

    #endregion

    #region Test: FastSub Method

    [Fact]
    public void FastSub_SubtractsTwoPositiveValues()
    {
        var a = new Fixed64(5);
        var b = new Fixed64(3);
        var result = FixedMath.FastSub(a, b);
        Assert.Equal(new Fixed64(2), result);
    }

    [Fact]
    public void FastSub_SubtractsNegativeFromPositive()
    {
        var a = new Fixed64(5);
        var b = new Fixed64(-3);
        var result = FixedMath.FastSub(a, b);
        Assert.Equal(new Fixed64(8), result);
    }

    [Fact]
    public void FastSub_SubtractsPositiveFromNegative()
    {
        var a = new Fixed64(-5);
        var b = new Fixed64(3);
        var result = FixedMath.FastSub(a, b);
        Assert.Equal(new Fixed64(-8), result);
    }

    #endregion

    #region Test: FastMul Method

    [Fact]
    public void FastMul_MultipliesTwoPositiveValues()
    {
        var a = new Fixed64(2);
        var b = new Fixed64(3);
        var result = FixedMath.FastMul(a, b);
        Assert.Equal(new Fixed64(6), result);
    }

    [Fact]
    public void FastMul_MultipliesPositiveAndNegativeValue()
    {
        var a = new Fixed64(2);
        var b = new Fixed64(-3);
        var result = FixedMath.FastMul(a, b);
        Assert.Equal(new Fixed64(-6), result);
    }

    [Fact]
    public void FastMul_MultipliesWithZero()
    {
        var a = new Fixed64(0);
        var b = new Fixed64(3);
        var result = FixedMath.FastMul(a, b);
        Assert.Equal(Fixed64.Zero, result);
    }

    [Fact]
    public void FastMul_FractionalRemainder_TruncatesInsteadOfOperatorRounding()
    {
        Fixed64 x = Fixed64.FromRaw(1);
        Fixed64 y = Fixed64.FromRaw((1L << 31) + 1);

        Assert.Equal(Fixed64.FromRaw(1), x * y);
        Assert.Equal(Fixed64.Zero, FixedMath.FastMul(x, y));
    }

    [Fact]
    public void FastMul_IntegerLeftOperand_PreservesUncheckedRawSemantics()
    {
        Fixed64 x = new Fixed64(-7);
        Fixed64 y = Fixed64.FromDouble(3.25);

        Assert.Equal(Fixed64.FromRaw(-7 * y.m_rawValue), FixedMath.FastMul(x, y));
    }

    [Fact]
    public void FastMul_IntegerRightOperand_PreservesUncheckedRawSemantics()
    {
        Fixed64 x = Fixed64.FromDouble(-3.25);
        Fixed64 y = new Fixed64(7);

        Assert.Equal(Fixed64.FromRaw(7 * x.m_rawValue), FixedMath.FastMul(x, y));
    }

    #endregion

    #region Test: Clamp Method

    [Fact]
    public void Clamp_ValueWithinRange_ReturnsSameValue()
    {
        var value = new Fixed64(5);
        var min = new Fixed64(3);
        var result = FixedMath.Clamp(value, min, Fixed64.MaxValue);
        Assert.Equal(new Fixed64(5), result);
    }

    [Fact]
    public void Clamp_ValueWithinRange_ReturnsValue()
    {
        var result = FixedMath.Clamp(new Fixed64(3), new Fixed64(2), new Fixed64(5));
        Assert.Equal(new Fixed64(3), result);
    }

    [Fact]
    public void Clamp_ValueBelowMin_ReturnsMin()
    {
        var result = FixedMath.Clamp(new Fixed64(1), new Fixed64(2), new Fixed64(5));
        Assert.Equal(new Fixed64(2), result);
    }

    [Fact]
    public void Clamp_ValueAboveMax_ReturnsMax()
    {
        var result = FixedMath.Clamp(new Fixed64(6), new Fixed64(2), new Fixed64(5));
        Assert.Equal(new Fixed64(5), result);
    }

    [Fact]
    public void Clamp_GenericComparable_ClampsAtBothEndsAndPassesThrough()
    {
        Assert.Equal(5, FixedMath.Clamp(9, 1, 5));
        Assert.Equal(1, FixedMath.Clamp(-3, 1, 5));
        Assert.Equal(3, FixedMath.Clamp(3, 1, 5));
    }

    #endregion

    #region Test: Interpolation Methods

    [Fact]
    public void Lerp_TAtZero_ReturnsFromValue()
    {
        var result = FixedMath.Lerp(new Fixed64(3), new Fixed64(5), Fixed64.Zero);
        Assert.Equal(new Fixed64(3), result);
    }

    [Fact]
    public void Lerp_TAtOne_ReturnsToValue()
    {
        var result = FixedMath.Lerp(new Fixed64(3), new Fixed64(5), Fixed64.One);
        Assert.Equal(new Fixed64(5), result);
    }

    [Fact]
    public void Lerp_TAtHalf_ReturnsMidpoint()
    {
        var result = FixedMath.Lerp(new Fixed64(3), new Fixed64(5), Fixed64.Half);
        Assert.Equal(new Fixed64(4), result);
    }

    [Fact]
    public void Lerp_FullRawDomain_MatchesBigIntegerOracle()
    {
        (Fixed64 From, Fixed64 To, Fixed64 Amount)[] cases =
        {
            (Fixed64.MinValue, Fixed64.MaxValue, Fixed64.Zero),
            (Fixed64.MinValue, Fixed64.MaxValue, Fixed64.MinIncrement),
            (Fixed64.MinValue, Fixed64.MaxValue, Fixed64.Half),
            (Fixed64.MinValue, Fixed64.MaxValue, Fixed64.One - Fixed64.MinIncrement),
            (Fixed64.MinValue, Fixed64.MaxValue, Fixed64.One),
            (Fixed64.MaxValue, Fixed64.MinValue, Fixed64.Half),
            (Fixed64.FromRaw(long.MinValue + 1), Fixed64.MaxValue, Fixed64.Half),
            (Fixed64.FromRaw(1), Fixed64.Zero, Fixed64.FromFraction(3, 4)),
        };

        foreach ((Fixed64 from, Fixed64 to, Fixed64 amount) in cases)
        {
            Fixed64 expected = Fixed64.FromRaw(LerpRawToEven(from.m_rawValue, to.m_rawValue, amount.m_rawValue));
            Fixed64 actual = FixedMath.Lerp(from, to, amount);
            Assert.True(
                expected == actual,
                $"from={from.m_rawValue}, to={to.m_rawValue}, amount={amount.m_rawValue}, expected={expected.m_rawValue}, actual={actual.m_rawValue}");
        }
    }

    private static long LerpRawToEven(long fromRaw, long toRaw, long amountRaw)
    {
        if (amountRaw <= 0L)
            return fromRaw;
        if (amountRaw >= FixedMath.ONE_L)
            return toRaw;

        BigInteger numerator = ((BigInteger)fromRaw << FixedMath.SHIFT_AMOUNT_I)
            + ((BigInteger)toRaw - fromRaw) * amountRaw;
        BigInteger denominator = BigInteger.One << FixedMath.SHIFT_AMOUNT_I;
        BigInteger quotient = BigInteger.DivRem(BigInteger.Abs(numerator), denominator, out BigInteger remainder);
        int midpointComparison = (remainder << 1).CompareTo(denominator);
        if (midpointComparison > 0 || (midpointComparison == 0 && !quotient.IsEven))
            quotient++;

        return (long)(numerator.Sign < 0 ? -quotient : quotient);
    }

    [Fact]
    public void SmoothStep_TAtHalf_ReturnsSmoothedMidpoint()
    {
        var result = FixedMath.SmoothStep(Fixed64.Zero, new Fixed64(10), Fixed64.Half);
        Assert.Equal(new Fixed64(5), result);
    }

    [Fact]
    public void SmoothStep_TOutsideUnitRange_ReturnsClampedEndpoints()
    {
        Assert.Equal(new Fixed64(3), FixedMath.SmoothStep(new Fixed64(3), new Fixed64(5), -Fixed64.Epsilon));
        Assert.Equal(new Fixed64(5), FixedMath.SmoothStep(new Fixed64(3), new Fixed64(5), Fixed64.One + Fixed64.Epsilon));
    }

    [Fact]
    public void CubicInterpolate_TAtEndpoints_ReturnsEndpointValues()
    {
        var p0 = new Fixed64(3);
        var p1 = new Fixed64(5);

        Assert.Equal(p0, FixedMath.CubicInterpolate(p0, p1, Fixed64.One, Fixed64.One, Fixed64.Zero));
        Assert.Equal(p1, FixedMath.CubicInterpolate(p0, p1, Fixed64.One, Fixed64.One, Fixed64.One));
    }

    [Fact]
    public void CatmullRomAndHermite_ReturnExpectedValues()
    {
        Assert.Equal(new Fixed64(15), FixedMath.CatmullRom(new Fixed64(0), new Fixed64(10), new Fixed64(20), new Fixed64(30), Fixed64.Half));
        Assert.Equal(Fixed64.Zero, FixedMath.HermiteSpline(Fixed64.Zero, Fixed64.One, new Fixed64(10), Fixed64.One, Fixed64.Zero));
        Assert.Equal(new Fixed64(5), FixedMath.HermiteSpline(Fixed64.Zero, Fixed64.Zero, new Fixed64(10), Fixed64.Zero, Fixed64.Half));
        Assert.Equal(new Fixed64(10), FixedMath.HermiteSpline(Fixed64.Zero, Fixed64.One, new Fixed64(10), Fixed64.One, Fixed64.One));
    }

    [Fact]
    public void BarycentricCoordinate_WeightsSecondAndThirdValues()
    {
        var value1 = new Fixed64(10);
        var value2 = new Fixed64(20);
        var value3 = new Fixed64(30);

        Assert.Equal(value1, FixedMath.BarycentricCoordinate(value1, value2, value3, Fixed64.Zero, Fixed64.Zero));
        Assert.Equal(value2, FixedMath.BarycentricCoordinate(value1, value2, value3, Fixed64.One, Fixed64.Zero));
        Assert.Equal(value3, FixedMath.BarycentricCoordinate(value1, value2, value3, Fixed64.Zero, Fixed64.One));
        Assert.Equal(new Fixed64(25), FixedMath.BarycentricCoordinate(new Fixed64(10), new Fixed64(20), new Fixed64(30), Fixed64.Half, Fixed64.Half));
    }

    [Fact]
    public void BarycentricCoordinate_FullRawDomain_MatchesSingleRoundingOracle()
    {
        (long A, long B, long C, long WeightB, long WeightC)[] cases =
        {
            (long.MinValue, long.MaxValue, long.MaxValue, Fixed64.Half.m_rawValue, Fixed64.Half.m_rawValue),
            (long.MaxValue, long.MinValue, long.MinValue, Fixed64.Half.m_rawValue, Fixed64.Half.m_rawValue),
            (long.MaxValue, long.MinValue, 0L, long.MinValue, 0L),
            (long.MinValue, long.MaxValue, 0L, long.MinValue, 0L),
            (0L, 1L, 0L, Fixed64.Half.m_rawValue, 0L),
            (1L, 2L, 0L, Fixed64.Half.m_rawValue, 0L),
        };

        foreach ((long a, long b, long c, long weightB, long weightC) in cases)
        {
            Fixed64 expected = BarycentricRawToEvenSaturating(a, b, c, weightB, weightC);
            Fixed64 actual = FixedMath.BarycentricCoordinate(
                Fixed64.FromRaw(a),
                Fixed64.FromRaw(b),
                Fixed64.FromRaw(c),
                Fixed64.FromRaw(weightB),
                Fixed64.FromRaw(weightC));

            Assert.True(
                expected == actual,
                $"a={a}, b={b}, c={c}, weightB={weightB}, weightC={weightC}, expected={expected.m_rawValue}, actual={actual.m_rawValue}");
        }
    }

    private static Fixed64 BarycentricRawToEvenSaturating(
        long a,
        long b,
        long c,
        long weightB,
        long weightC)
    {
        BigInteger numerator = ((BigInteger)a << FixedMath.SHIFT_AMOUNT_I)
            + (((BigInteger)b - a) * weightB)
            + (((BigInteger)c - a) * weightC);
        BigInteger denominator = BigInteger.One << FixedMath.SHIFT_AMOUNT_I;
        BigInteger quotient = BigInteger.DivRem(BigInteger.Abs(numerator), denominator, out BigInteger remainder);
        int midpointComparison = (remainder << 1).CompareTo(denominator);
        if (midpointComparison > 0 || (midpointComparison == 0 && !quotient.IsEven))
            quotient++;

        if (numerator.Sign < 0)
            quotient = -quotient;
        if (quotient > long.MaxValue)
            return Fixed64.MaxValue;
        if (quotient < long.MinValue)
            return Fixed64.MinValue;
        return Fixed64.FromRaw((long)quotient);
    }

    [Fact]
    public void SumSquaredBarycentricProducts_ReturnsSecondOrderSimplexProductSum()
    {
        var result = FixedMath.SumSquaredBarycentricProducts(new Fixed64(2), new Fixed64(3), new Fixed64(5));

        Assert.Equal(new Fixed64(69), result);
    }

    [Fact]
    public void SumBarycentricProducts_ReturnsCrossSimplexProductSum()
    {
        var result = FixedMath.SumBarycentricProducts(
            new Fixed64(1),
            new Fixed64(2),
            new Fixed64(3),
            new Fixed64(4),
            new Fixed64(5),
            new Fixed64(6));

        Assert.Equal(new Fixed64(122), result);
    }

    #endregion

    #region Test: MoveTowards Method

    [Fact]
    public void MoveTowards_ValueMovesUpWithoutOvershoot()
    {
        var result = FixedMath.MoveTowards(new Fixed64(3), new Fixed64(5), new Fixed64(1));
        Assert.Equal(new Fixed64(4), result);
    }

    [Fact]
    public void MoveTowards_ValueMovesDownWithoutOvershoot()
    {
        var result = FixedMath.MoveTowards(new Fixed64(5), new Fixed64(3), new Fixed64(1));
        Assert.Equal(new Fixed64(4), result);
    }

    [Fact]
    public void MoveTowards_ValueOvershoot_ReturnsTarget()
    {
        var result = FixedMath.MoveTowards(new Fixed64(3), new Fixed64(5), new Fixed64(3));
        Assert.Equal(new Fixed64(5), result);  // It overshoots, so it should return 5
    }

    [Fact]
    public void MoveTowards_ValueMovesDownWithOvershoot_ReturnsTarget()
    {
        var result = FixedMath.MoveTowards(new Fixed64(5), new Fixed64(3), new Fixed64(3));
        Assert.Equal(new Fixed64(3), result);
    }

    [Fact]
    public void MoveTowards_ValueAlreadyAtTarget_ReturnsTarget()
    {
        var result = FixedMath.MoveTowards(new Fixed64(5), new Fixed64(5), new Fixed64(3));
        Assert.Equal(new Fixed64(5), result);
    }

    #endregion

    #region Test: FastDiv Method

    [Fact]
    public void FastDiv_DividesTwoPositiveValues()
    {
        var result = FixedMath.FastDiv(new Fixed64(10), new Fixed64(2));
        Assert.Equal(new Fixed64(5), result);
    }

    [Fact]
    public void FastDiv_DividesNegativeByPositiveValue()
    {
        var result = FixedMath.FastDiv(new Fixed64(-10), new Fixed64(2));
        Assert.Equal(new Fixed64(-5), result);
    }

    [Fact]
    public void FastDiv_SafeFractionalValues_MatchDivisionOperator()
    {
        Fixed64 dividend = Fixed64.FromDouble(7.5);
        Fixed64 divisor = Fixed64.FromDouble(1.25);

        Assert.Equal(dividend / divisor, FixedMath.FastDiv(dividend, divisor));
    }

    [Fact]
    public void FastDiv_NonPositiveDivisor_MatchesDivisionOperator()
    {
        Fixed64 dividend = Fixed64.FromDouble(7.5);
        Fixed64 divisor = Fixed64.FromDouble(-1.25);

        Assert.Equal(dividend / divisor, FixedMath.FastDiv(dividend, divisor));
        Assert.Throws<DivideByZeroException>(() => FixedMath.FastDiv(dividend, Fixed64.Zero));
    }

    [Fact]
    public void FastDiv_OverflowingPositiveDivisorPath_Saturates()
    {
        Assert.Equal(Fixed64.MaxValue, FixedMath.FastDiv(Fixed64.MaxValue, Fixed64.MinIncrement));
        Assert.Equal(Fixed64.MinValue, FixedMath.FastDiv(Fixed64.MinValue, Fixed64.MinIncrement));
    }

    [Fact]
    public void FastDiv_PositiveDivisors_MatchesDivisionAtRoundingBoundaries()
    {
        Fixed64 positive = Fixed64.MinIncrement;
        Fixed64 negative = -Fixed64.MinIncrement;
        Fixed64[] divisors =
        {
            Fixed64.Two + Fixed64.MinIncrement,
            Fixed64.Two,
            Fixed64.Two - Fixed64.MinIncrement,
        };

        foreach (Fixed64 divisor in divisors)
        {
            Assert.Equal(positive / divisor, FixedMath.FastDiv(positive, divisor));
            Assert.Equal(negative / divisor, FixedMath.FastDiv(negative, divisor));
        }
    }

    #endregion

    #region Test: FastMod Method (Edge Case)

    [Fact]
    public void FastMod_PositiveValues_ReturnsCorrectRemainder()
    {
        var result = FixedMath.FastMod(new Fixed64(10), new Fixed64(3));
        Assert.Equal(new Fixed64(1), result);
    }

    [Fact]
    public void FastMod_NegativeDividend_ReturnsCorrectRemainder()
    {
        var result = FixedMath.FastMod(new Fixed64(-10), new Fixed64(3));
        Assert.Equal(new Fixed64(-1), result);  // Check for correct handling of negative numbers
    }

    [Fact]
    public void FastMod_ZeroDivisor_ThrowsException()
    {
        Assert.Throws<DivideByZeroException>(() => FixedMath.FastMod(new Fixed64(10), Fixed64.Zero));
    }

    #endregion

}
