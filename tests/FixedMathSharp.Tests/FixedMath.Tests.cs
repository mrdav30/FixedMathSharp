using System;
using Xunit;

namespace FixedMathSharp.Tests
{
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
            var result = FixedMath.Clamp01(new Fixed64(0.5f));
            Assert.Equal(new Fixed64(0.5f), result);
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

        #endregion

        #region Test: Ceiling Method

        [Fact]
        public void Ceiling_WithFraction_ReturnsNextInteger()
        {
            var result = FixedMath.Ceiling(new Fixed64(1.5));
            Assert.Equal(new Fixed64(2), result);
        }

        [Fact]
        public void Ceiling_ExactInteger_ReturnsSameInteger()
        {
            var result = FixedMath.Ceiling(new Fixed64(3.0));
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
            var result = FixedMath.Round(new Fixed64(2.5));
            Assert.Equal(new Fixed64(2), result);
        }

        [Fact]
        public void Round_AwayFromZero_RoundsUp()
        {
            var result = FixedMath.Round(new Fixed64(2.5), System.MidpointRounding.AwayFromZero);
            Assert.Equal(new Fixed64(3), result);
        }

        [Fact]
        public void Round_ToEven_NegativeNumber_RoundsToNearestEven()
        {
            var result = FixedMath.Round(new Fixed64(-2.5));
            Assert.Equal(new Fixed64(-2), result);
        }

        #endregion

        #region Test: Round Method (With Decimal Places)

        [Fact]
        public void Round_WithDecimalPlaces_RoundsToTwoDecimalPlaces()
        {
            var result = FixedMath.RoundToPrecision(new Fixed64(2.556f), 2, MidpointRounding.AwayFromZero);
            Assert.Equal(2.56f, result.ToFormattedFloat());
        }

        [Fact]
        public void Round_WithDecimalPlaces_RoundsToZeroDecimalPlaces_ToEven()
        {
            var result = FixedMath.RoundToPrecision(new Fixed64(2.5), 0);
            Assert.Equal(new Fixed64(2), result);
        }

        [Fact]
        public void Round_WithDecimalPlaces_RoundsToZeroDecimalPlaces_AwayFromZero()
        {
            var result = FixedMath.RoundToPrecision(new Fixed64(2.5), 0, MidpointRounding.AwayFromZero);
            Assert.Equal(new Fixed64(3), result);
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

        #endregion

        #region Test: Interpolation Methods

        [Fact]
        public void LinearInterpolate_TAtZero_ReturnsFromValue()
        {
            var result = FixedMath.LinearInterpolate(new Fixed64(3), new Fixed64(5), new Fixed64(0));
            Assert.Equal(new Fixed64(3), result);
        }

        [Fact]
        public void LinearInterpolate_TAtOne_ReturnsToValue()
        {
            var result = FixedMath.LinearInterpolate(new Fixed64(3), new Fixed64(5), new Fixed64(1));
            Assert.Equal(new Fixed64(5), result);
        }

        [Fact]
        public void LinearInterpolate_TAtHalf_ReturnsMidpoint()
        {
            var result = FixedMath.LinearInterpolate(new Fixed64(3), new Fixed64(5), new Fixed64(0.5));
            Assert.Equal(new Fixed64(4), result);  // Midpoint should be 4
        }

        [Fact]
        public void SmoothStep_TAtZero_ReturnsFromValue()
        {
            var result = FixedMath.SmoothStep(new Fixed64(3), new Fixed64(5), new Fixed64(0));
            Assert.Equal(new Fixed64(3), result);
        }

        [Fact]
        public void SmoothStep_TAtOne_ReturnsToValue()
        {
            var result = FixedMath.SmoothStep(new Fixed64(3), new Fixed64(5), new Fixed64(1));
            Assert.Equal(new Fixed64(5), result);
        }

        [Fact]
        public void SmoothStep_TAtHalf_ReturnsSmoothedMidpoint()
        {
            var result = FixedMath.SmoothStep(new Fixed64(0), new Fixed64(10), new Fixed64(0.5));
            Assert.Equal(new Fixed64(5), result); // Should be near 5 with smoothing effect
        }

        [Fact]
        public void CubicInterpolate_TAtZero_ReturnsP0()
        {
            var result = FixedMath.CubicInterpolate(new Fixed64(3), new Fixed64(5), new Fixed64(1), new Fixed64(1), new Fixed64(0));
            Assert.Equal(new Fixed64(3), result);
        }

        [Fact]
        public void CubicInterpolate_TAtOne_ReturnsP1()
        {
            var result = FixedMath.CubicInterpolate(new Fixed64(3), new Fixed64(5), new Fixed64(1), new Fixed64(1), new Fixed64(1));
            Assert.Equal(new Fixed64(5), result);
        }

        [Fact]
        public void CubicInterpolate_TAtHalf_ReturnsSmoothCurveValue()
        {
            var result = FixedMath.CubicInterpolate(new Fixed64(0), new Fixed64(10), new Fixed64(2), new Fixed64(2), new Fixed64(0.5));
            Assert.Equal(new Fixed64(5), result); // Expected to be near midpoint but with cubic smoothing
        }

        #endregion

        #region Test: Clamp Method

        [Fact]
        public void Clamp_ValueWithinRange_ReturnsSameValue()
        {
            var value = new Fixed64(5);
            var min = new Fixed64(3);
            var result = FixedMath.Clamp(value, min, Fixed64.MAX_VALUE);
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
}