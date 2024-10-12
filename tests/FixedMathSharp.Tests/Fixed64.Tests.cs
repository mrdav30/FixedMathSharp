using System;
using Xunit;

namespace FixedMathSharp.Tests
{
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
        public void Divide_ByZero_ThrowsException()
        {
            var a = new Fixed64(6);
            Assert.Throws<DivideByZeroException>(() => { var result = a / FixedMath.Zero; });
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
        public void Convert_FromFloat_ReturnsCorrectFixed64()
        {
            Fixed64 result = (Fixed64)5.5f;
            Assert.Equal(new Fixed64(5.5f), result);
        }

        [Fact]
        public void Convert_ToDouble_ReturnsCorrectDouble()
        {
            var fixedValue = new Fixed64(5.5f);
            double result = (double)fixedValue;
            Assert.Equal(5.5, result);
        }

        #endregion

        #region Test: Fraction Method

        [Fact]
        public void Fraction_CreatesCorrectFixed64Value()
        {
            var result = Fixed64.Fraction(1, 2);
            Assert.Equal(new Fixed64(0.5f), result);
        }

        #endregion

        #region Test: Arithmetic Overflow Protection

        [Fact]
        public void Add_OverflowProtection_ReturnsMaxValue()
        {
            var a = FixedMath.MaxValue;
            var b = new Fixed64(1);
            var result = a + b;
            Assert.Equal(FixedMath.MaxValue, result);
        }

        [Fact]
        public void Subtract_OverflowProtection_ReturnsMinValue()
        {
            var a = FixedMath.MinValue;
            var b = new Fixed64(1);
            var result = a - b;
            Assert.Equal(FixedMath.MinValue, result);
        }

        #endregion
    }
}