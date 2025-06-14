using System;

#if NET48_OR_GREATER
using System.IO;
using System.Runtime.Serialization.Formatters.Binary;
#endif

#if NET8_0_OR_GREATER
using System.Text.Json;
using System.Text.Json.Serialization;
#endif

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
            Assert.Throws<DivideByZeroException>(() => { var result = a / Fixed64.Zero; });
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

        [Fact]
        public void Fixed64_Serialization_RoundTripMaintainsData()
        {
            var originalValue = FixedMath.PI;

            // Serialize the Fixed64 object
#if NET48_OR_GREATER
            var formatter = new BinaryFormatter();
            using var stream = new MemoryStream();
            formatter.Serialize(stream, originalValue);

            // Reset stream position and deserialize
            stream.Seek(0, SeekOrigin.Begin);
            var deserializedValue = (Fixed64)formatter.Deserialize(stream);
#endif

#if NET8_0_OR_GREATER
            var jsonOptions = new JsonSerializerOptions
            {
                DefaultIgnoreCondition = JsonIgnoreCondition.WhenWritingNull,
                ReferenceHandler = ReferenceHandler.IgnoreCycles,
                IncludeFields = true,
                IgnoreReadOnlyProperties = true
            };
            var json = JsonSerializer.SerializeToUtf8Bytes(originalValue, jsonOptions);
            var deserializedValue = JsonSerializer.Deserialize<Fixed64>(json, jsonOptions);
#endif

            // Check that deserialized values match the original
            Assert.Equal(originalValue, deserializedValue);
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
            var a = Fixed64.MAX_VALUE;
            var b = new Fixed64(1);
            var result = a + b;
            Assert.Equal(Fixed64.MAX_VALUE, result);
        }

        [Fact]
        public void Subtract_OverflowProtection_ReturnsMinValue()
        {
            var a = Fixed64.MIN_VALUE;
            var b = new Fixed64(1);
            var result = a - b;
            Assert.Equal(Fixed64.MIN_VALUE, result);
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
            var a = new Fixed64(4.2);
            Assert.False(a.IsInteger());
        }

        [Fact]
        public void IsInteger_NegativeDecimal_ReturnsFalse()
        {
            var a = new Fixed64(-4.2);
            Assert.False(a.IsInteger());
        }

        #endregion
    }
}