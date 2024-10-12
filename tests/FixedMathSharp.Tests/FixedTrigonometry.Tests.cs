using System;
using System.Security.Cryptography;
using Xunit;

namespace FixedMathSharp.Tests
{
    public class FixedTrigonometryTests
    {
        #region Test: Pow Method

        [Fact]
        public void Pow_RaisesValueToPositiveExponent()
        {
            var baseValue = new Fixed64(2);
            var exponent = new Fixed64(3);
            var result = FixedMath.Pow(baseValue, exponent);
            Assert.Equal(new Fixed64(8), result); // 2^3 = 8
        }

        [Fact]
        public void Pow_RaisesValueToZeroExponent_ReturnsOne()
        {
            var baseValue = new Fixed64(5);
            var exponent = FixedMath.Zero;
            var result = FixedMath.Pow(baseValue, exponent);
            Assert.Equal(FixedMath.One, result); // Any number raised to 0 is 1
        }

        [Fact]
        public void Pow_RaisesToNegativeExponent()
        {
            var baseValue = new Fixed64(2);
            var exponent = new Fixed64(-1);
            var result = FixedMath.Pow(baseValue, exponent);
            Assert.Equal(new Fixed64(0.5), result); // 2^-1 = 0.5
        }

        [Fact]
        public void Pow_ZeroToNegativeExponent_ThrowsException()
        {
            var baseValue = FixedMath.Zero;
            var exponent = new Fixed64(-1);
            Assert.Throws<DivideByZeroException>(() => FixedMath.Pow(baseValue, exponent));
        }

        #endregion

        #region Test: Sqrt Method

        [Fact]
        public void Sqrt_PositiveValue_ReturnsSquareRoot()
        {
            var value = new Fixed64(4);
            var result = FixedMath.Sqrt(value);
            Assert.Equal(new Fixed64(2), result); // sqrt(4) = 2
        }

        [Fact]
        public void Sqrt_NegativeValue_ThrowsException()
        {
            var value = new Fixed64(-4);
            Assert.Throws<ArgumentOutOfRangeException>(() => FixedMath.Sqrt(value));
        }

        [Fact]
        public void Sqrt_Zero_ReturnsZero()
        {
            var value = FixedMath.Zero;
            var result = FixedMath.Sqrt(value);
            Assert.Equal(FixedMath.Zero, result);
        }

        #endregion

        #region Test: Log2 and Ln Methods

        [Fact]
        public void Value_ReturnsCorrectLog()
        {
            var value = new Fixed64(8);
            var result = FixedMath.Log2(value);
            Assert.Equal(new Fixed64(3), result); // log2(8) = 3
        }

        [Fact]
        public void Log2_Zero_ThrowsException()
        {
            var value = FixedMath.Zero;
            Assert.Throws<ArgumentOutOfRangeException>(() => FixedMath.Log2(value));
        }

        [Fact]
        public void Ln_PositiveValue_ReturnsCorrectLog()
        {
            var value = new Fixed64(Math.E);
            var result = FixedMath.Ln(value);
            Assert.Equal(FixedMath.One, result); // ln(e) = 1
        }

        [Fact]
        public void Ln_NegativeValue_ThrowsException()
        {
            var value = new Fixed64(-1);
            Assert.Throws<ArgumentOutOfRangeException>(() => FixedMath.Ln(value));
        }

        #endregion

        #region Test: Rad2Deg and Deg2Rad Methods

        [Fact]
        public void Rad2Deg_ConvertsRadiansToDegrees()
        {
            var radians = FixedMath.PiOver2; // π/2 radians
            var result = FixedMath.RadToDeg(radians);
            var expected = new Fixed64(90);
            FixedMathTestHelper.AssertWithinRelativeTolerance(expected, result);
        }

        [Fact]
        public void Deg2Rad_ConvertsDegreesToRadians()
        {
            var degrees = (Fixed64)180;
            var result = FixedMath.DegToRad(degrees);
            var expected = FixedMath.PI; // 180 degrees = π radians
            FixedMathTestHelper.AssertWithinRelativeTolerance(expected, result);
        }

        #endregion

        #region Test: Sin and Cos Methods

        [Fact]
        public void Sin_ReturnsSineOfAngle()
        {
            var angle = FixedMath.PiOver2; // π/2 radians = 90 degrees
            var result = FixedMath.Sin(angle);
            Assert.Equal(FixedMath.One, result); // sin(90°) = 1
        }

        [Fact]
        public void Cos_ReturnsCosineOfAngle()
        {
            var angle = FixedMath.PI; // π radians = 180 degrees
            var result = FixedMath.Cos(angle);
            Assert.Equal(Fixed64.FromRaw(-1 * FixedMath.ONE_L), result); // cos(180°) = -1
        }

        #endregion

        #region Test: Tan Method

        [Fact]
        public void Tan_ReturnsTangentOfAngle()
        {
            var angle = new Fixed64(45); // 45 degrees in radians
            var result = FixedMath.Tan(FixedMath.DegToRad(angle));
            var expected = FixedMath.One; // tan(45°) ≈ 1.0
            FixedMathTestHelper.AssertWithinRelativeTolerance(expected, result);
        }

        #endregion

        #region Test: Asin Method

        [Fact]
        public void Asin_ReturnsArcsine()
        {
            var value = FixedMath.One; // sin(90°) = 1
            var result = FixedMath.Asin(value);
            Assert.Equal(FixedMath.PiOver2, result); // asin(1) = π/2
        }

        [Fact]
        public void Asin_ReturnsNegativePiOver2()
        {
            var value = -FixedMath.One; // sin(-90°) = -1
            var result = FixedMath.Asin(value);
            Assert.Equal(-FixedMath.PiOver2, result); // asin(-1) = -π/2
        }

        [Fact]
        public void Asin_ReturnsAsinOfHalf()
        {
            var value = new Fixed64(0.5);
            var result = FixedMath.Asin(value);
            var expected = FixedMath.PiOver6;  // asin(0.5) ≈ π/6 ≈ 0.5236 radians
            FixedMathTestHelper.AssertWithinRelativeTolerance(expected, result);
        }

        [Fact]
        public void Asin_ReturnsAsinOfNegativeHalf()
        {
            var value = new Fixed64(-0.5);
            var result = FixedMath.Asin(value);
            var expected = -FixedMath.PiOver6;  // asin(-0.5) ≈ -π/6 ≈ -0.5236 radians
            FixedMathTestHelper.AssertWithinRelativeTolerance(expected, result);
        }

        [Fact]
        public void Asin_ThrowsForOutOfDomain()
        {
            var value = new Fixed64(2); // Value greater than 1
            Assert.Throws<ArithmeticException>(() => FixedMath.Asin(value));
        }

        [Fact]
        public void Asin_ReturnsZeroForZero()
        {
            var value = FixedMath.Zero;
            var result = FixedMath.Asin(value);
            Assert.Equal(FixedMath.Zero, result); // asin(0) = 0
        }

        #endregion

        #region Test: Acos Method

        [Fact]
        public void Acos_ReturnsArccosine()
        {
            var value = FixedMath.Zero; // cos(90°) = 0
            var result = FixedMath.Acos(value);
            Assert.Equal(FixedMath.PiOver2, result); // acos(0) = π/2
        }

        [Fact]
        public void Acos_ReturnsZeroForOne()
        {
            var value = FixedMath.One; // cos(0°) = 1
            var result = FixedMath.Acos(value);
            Assert.Equal(FixedMath.Zero, result); // acos(1) = 0
        }

        [Fact]
        public void Acos_ReturnsPiForNegativeOne()
        {
            var value = -FixedMath.One; // cos(180°) = -1
            var result = FixedMath.Acos(value);
            Assert.Equal(FixedMath.PI, result); // acos(-1) = π
        }

        [Fact]
        public void Acos_ReturnsPiOver3ForHalf()
        {
            var value = new Fixed64(0.5); // cos(60°) = 0.5
            var result = FixedMath.Acos(value);
            var expected = FixedMath.PiOver3; // acos(0.5) ≈ π/3 ≈ 1.0472 radians
            FixedMathTestHelper.AssertWithinRelativeTolerance(expected, result);
        }

        [Fact]
        public void Acos_Returns2PiOver3ForNegativeHalf()
        {
            var value = new Fixed64(-0.5); // cos(120°) = -0.5
            var result = FixedMath.Acos(value);
            var expected = FixedMath.TwoPI / 3; // acos(-0.5) ≈ 2π/3 ≈ 2.0944 radians
            FixedMathTestHelper.AssertWithinRelativeTolerance(expected, result);
        }

        [Fact]
        public void Acos_ThrowsOutOfRangeForGreaterThanOne()
        {
            var value = new Fixed64(1.1);
            Assert.Throws<ArgumentOutOfRangeException>(() => FixedMath.Acos(value));
        }

        [Fact]
        public void Acos_ThrowsOutOfRangeForLessThanNegativeOne()
        {
            var value = new Fixed64(-1.1);
            Assert.Throws<ArgumentOutOfRangeException>(() => FixedMath.Acos(value));
        }

        #endregion

        #region Test: Atan

        [Fact]
        public void Atan_ReturnsArctangent()
        {
            var value = FixedMath.One; // tan(45°) = 1
            var result = FixedMath.Atan(value);
            Assert.Equal(FixedMath.PiOver4, result); // atan(1) = π/4
        }

        [Fact]
        public void Atan_Symmetry()
        {
            var value = new Fixed64(0.5);
            var positiveResult = FixedMath.Atan(value);
            var negativeResult = FixedMath.Atan(-value);
            Assert.Equal(positiveResult, -negativeResult);
        }

        [Fact]
        public void Atan_NearZero()
        {
            var value = new Fixed64(0.00001);
            var result = FixedMath.Atan(value);
            FixedMathTestHelper.AssertWithinRelativeTolerance(value, result); // Should be close to zero
        }

        [Fact]
        public void Atan_NearOne()
        {
            var value = new Fixed64(0.9999);
            var result = FixedMath.Atan(value);
            var expected = FixedMath.PiOver4; // atan(1) = π/4
            FixedMathTestHelper.AssertWithinRelativeTolerance(expected, result);
        }

        [Fact]
        public void Atan_TanInverse()
        {
            var values = new[] { new Fixed64(0.1), new Fixed64(0.5), new Fixed64(1), new Fixed64(2) };
            foreach (var value in values)
            {
                var atanResult = FixedMath.Atan(value);
                var tanResult = FixedMath.Tan(atanResult);
                FixedMathTestHelper.AssertWithinRelativeTolerance(value, tanResult, message: $"Relative error exceeds tolerance for value {value}");
            }
        }

        [Fact]
        public void Atan_RangeLimits()
        {
            var values = new[] { new Fixed64(-10), new Fixed64(-1), new Fixed64(0), new Fixed64(1), new Fixed64(10) };
            foreach (var value in values)
            {
                var result = FixedMath.Atan(value);
                FixedMathTestHelper.AssertWithinRange(result, -FixedMath.PiOver2, FixedMath.PiOver2, $"Result {result} is out of range for input {value}");
            }
        }

        [Fact]
        public void Atan_LargeInputs()
        {
            var values = new[] { new Fixed64(1000), new Fixed64(-1000), new Fixed64(1000000), new Fixed64(-1000000) };
            foreach (var value in values)
            {
                var result = FixedMath.Atan(value);
                FixedMathTestHelper.AssertWithinRange(result, -FixedMath.PiOver2, FixedMath.PiOver2, $"Result {result} is out of range for input {value}");
            }
        }

        #endregion

        #region Test: Atan2 Method

        [Fact]
        public void Atan2_ReturnsArctangentOfQuotient()
        {
            var y = FixedMath.One;
            var x = FixedMath.One;
            var result = FixedMath.Atan2(y, x);
            Assert.Equal(FixedMath.PiOver4, result); // atan2(1, 1) = π/4
        }

        [Fact]
        public void Atan2_Symmetry()
        {
            var y = FixedMath.One;
            var x = FixedMath.One;
            var positiveResult = FixedMath.Atan2(y, x);
            var negativeResult = FixedMath.Atan2(-y, -x);
            Assert.Equal(positiveResult - FixedMath.PI, negativeResult);
        }

        [Fact]
        public void Atan2_ZeroY()
        {
            var x = FixedMath.One;
            var result = FixedMath.Atan2(FixedMath.Zero, x);
            Assert.Equal(FixedMath.Zero, result); // atan2(0, x) should be 0 for positive x

            result = FixedMath.Atan2(FixedMath.Zero, -x);
            Assert.Equal(FixedMath.PI, result); // atan2(0, -x) should be π for negative x
        }

        [Fact]
        public void Atan2_ZeroX()
        {
            var y = FixedMath.One;
            var result = FixedMath.Atan2(y, FixedMath.Zero);
            Assert.Equal(FixedMath.PiOver2, result); // atan2(y, 0) should be π/2 for positive y

            result = FixedMath.Atan2(-y, FixedMath.Zero);
            Assert.Equal(-FixedMath.PiOver2, result); // atan2(-y, 0) should be -π/2 for negative y
        }

        [Fact]
        public void Atan2_Quadrants()
        {
            var result = FixedMath.Atan2(FixedMath.One, FixedMath.One); // Q1
            FixedMathTestHelper.AssertWithinRange(result, FixedMath.Zero, FixedMath.PiOver2);

            result = FixedMath.Atan2(FixedMath.One, -FixedMath.One); // Q2
            FixedMathTestHelper.AssertWithinRange(result, FixedMath.PiOver2, FixedMath.PI);

            result = FixedMath.Atan2(-FixedMath.One, -FixedMath.One); // Q3
            FixedMathTestHelper.AssertWithinRange(result, -FixedMath.PI, -FixedMath.PiOver2);

            result = FixedMath.Atan2(-FixedMath.One, FixedMath.One); // Q4
            FixedMathTestHelper.AssertWithinRange(result, -FixedMath.PiOver2, -FixedMath.Zero);
        }

        [Fact]
        public void Atan2_SpecificAngles()
        {
            FixedMathTestHelper.AssertWithinRelativeTolerance(FixedMath.PiOver4, FixedMath.Atan2(FixedMath.One, FixedMath.One)); // 45 degrees
            FixedMathTestHelper.AssertWithinRelativeTolerance(-FixedMath.PiOver4, FixedMath.Atan2(-FixedMath.One, FixedMath.One)); // -45 degrees
            FixedMathTestHelper.AssertWithinRelativeTolerance(3 * FixedMath.PiOver4, FixedMath.Atan2(FixedMath.One, -FixedMath.One)); // 135 degrees
            FixedMathTestHelper.AssertWithinRelativeTolerance(-3 * FixedMath.PiOver4, FixedMath.Atan2(-FixedMath.One, -FixedMath.One)); // -135 degrees          
        }

        [Fact]
        public void Atan2_LargeValues()
        {
            var largeValue = new Fixed64(1000000);
            var result = FixedMath.Atan2(largeValue, largeValue);
            FixedMathTestHelper.AssertWithinRelativeTolerance(FixedMath.PiOver4, result);  // atan2(1000000, 1000000) should be π/4

            result = FixedMath.Atan2(-largeValue, -largeValue);
            FixedMathTestHelper.AssertWithinRelativeTolerance(-3 * FixedMath.PiOver4, result);  // atan2(-1000000, -1000000) should be -3π/4
        }

        #endregion

        #region Test: SinToCos Method

        [Fact]
        public void SinToCos_ReturnsCosine()
        {
            var sinValue = FixedMath.One; // sin(90°) = 1
            var result = FixedMath.SinToCos(sinValue);
            Assert.Equal(FixedMath.Zero, result); // cos(90°) = 0
        }

        #endregion
    }
}