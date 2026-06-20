using System;
using Xunit;

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

    #region Test: AddOverflowHelper Method

    [Fact]
    public void AddOverflowHelper_NoOverflow_ReturnsCorrectSum()
    {
        bool overflow = false;
        long x = 10;
        long y = 20;
        var result = FixedMath.AddOverflowHelper(x, y, ref overflow);
        Assert.Equal(30, result);
        Assert.False(overflow);
    }

    [Fact]
    public void AddOverflowHelper_PositiveOverflow_SetsOverflowFlag()
    {
        bool overflow = false;
        long x = long.MaxValue;
        long y = 1;
        _ = FixedMath.AddOverflowHelper(x, y, ref overflow);
        Assert.True(overflow);
    }

    [Fact]
    public void AddOverflowHelper_NegativeOverflow_SetsOverflowFlag()
    {
        bool overflow = false;
        long x = long.MinValue;
        long y = -1;
        _ = FixedMath.AddOverflowHelper(x, y, ref overflow);
        Assert.True(overflow);
    }

    #endregion
}
