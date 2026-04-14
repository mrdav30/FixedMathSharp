using MemoryPack;
using System;
using System.Collections.Generic;
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

    [Fact]
    public void OffsetAndRawHelpers_WorkCorrectly()
    {
        var value = new Fixed64(2.5);
        value.Offset(2);

        Assert.Equal(new Fixed64(4.5), value);
        Assert.Equal(value.m_rawValue.ToString(), value.RawToString());
    }

    [Fact]
    public void PostIncrementAndPostDecrement_ReturnOriginalValuesAndMutateTarget()
    {
        var value = new Fixed64(2);

        var postIncrement = Fixed64.PostIncrement(ref value);
        Assert.Equal(new Fixed64(2), postIncrement);
        Assert.Equal(new Fixed64(3), value);

        var postDecrement = Fixed64.PostDecrement(ref value);
        Assert.Equal(new Fixed64(3), postDecrement);
        Assert.Equal(new Fixed64(2), value);
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
        Assert.Equal(5L, (long)new Fixed64(5.75));

        Fixed64 fromInt = (Fixed64)6;
        Assert.Equal(new Fixed64(6), fromInt);
        Assert.Equal(6, (int)new Fixed64(6.75));

        Fixed64 fromFloat = (Fixed64)5.5f;
        Assert.Equal(new Fixed64(5.5), fromFloat);
        Assert.Equal(5.5f, (float)fromFloat);

        Fixed64 fromDouble = (Fixed64)7.25;
        Assert.Equal(new Fixed64(7.25), fromDouble);
        Assert.Equal(7.25, (double)fromDouble);

        Fixed64 fromDecimal = (Fixed64)8.5m;
        Assert.Equal(new Fixed64(8.5), fromDecimal);
        Assert.True(Math.Abs((decimal)fromDecimal - 8.5m) < 0.000001m);

        Assert.Equal(1.0, Fixed64.ToDouble(Fixed64.One.m_rawValue));
        Assert.Equal(1.0f, Fixed64.ToFloat(Fixed64.One.m_rawValue));
        Assert.True(Math.Abs(Fixed64.ToDecimal(Fixed64.One.m_rawValue) - 1.0m) < 0.000001m);
    }

    [Fact]
    public void MixedOperators_WorkCorrectly()
    {
        var value = new Fixed64(1.5);

        Assert.Equal(new Fixed64(3.5), value + 2);
        Assert.Equal(new Fixed64(3.5), 2 + value);

        Assert.Equal(new Fixed64(0.5), value - 1);
        Assert.Equal(new Fixed64(0.5), 2 - value);

        Assert.Equal(new Fixed64(3), value * 2);
        Assert.Equal(new Fixed64(3), 2 * value);
        Assert.Equal(new Fixed64(0.75), value / 2);
        Assert.Equal(new Fixed64(1.3333333333), 2 / value);
    }

    [Fact]
    public void MultiplyOverflowProtection_ReturnsSaturatedBounds()
    {
        Assert.Equal(Fixed64.MAX_VALUE, Fixed64.MAX_VALUE * new Fixed64(2));
        Assert.Equal(Fixed64.MIN_VALUE, Fixed64.MAX_VALUE * new Fixed64(-2));
    }

    [Fact]
    public void DivideOverflowProtection_ReturnsSaturatedBounds()
    {
        Assert.Equal(Fixed64.MAX_VALUE, Fixed64.MAX_VALUE / Fixed64.Epsilon);
        Assert.Equal(Fixed64.MIN_VALUE, new Fixed64(-1) * Fixed64.MAX_VALUE / Fixed64.Epsilon);
    }

    [Fact]
    public void EdgeOperators_HandleSpecialCases()
    {
        Assert.Equal(Fixed64.Zero, Fixed64.MIN_VALUE % (-Fixed64.Epsilon));
        Assert.Equal(Fixed64.MAX_VALUE, -Fixed64.MIN_VALUE);
        Assert.Equal(new Fixed64(2), Fixed64.One << 1);
        Assert.Equal(new Fixed64(2), new Fixed64(4) >> 1);
    }

    [Fact]
    public void Parsing_RawStrings_WorksCorrectly()
    {
        var rawValue = Fixed64.One.m_rawValue.ToString();
        var negativeRawValue = (-Fixed64.One.m_rawValue).ToString();

        Assert.Equal(Fixed64.One, Fixed64.Parse(rawValue));
        Assert.Equal(-Fixed64.One, Fixed64.Parse(negativeRawValue));
        Assert.Throws<ArgumentNullException>(() => Fixed64.Parse(null!));
        Assert.Throws<ArgumentNullException>(() => Fixed64.Parse(string.Empty));
        Assert.Throws<FormatException>(() => Fixed64.Parse("abc"));

        Assert.True(Fixed64.TryParse(rawValue, out var parsedPositive));
        Assert.Equal(Fixed64.One, parsedPositive);

        Assert.True(Fixed64.TryParse(negativeRawValue, out var parsedNegative));
        Assert.Equal(-Fixed64.One, parsedNegative);

        Assert.False(Fixed64.TryParse(string.Empty, out var emptyResult));
        Assert.Equal(Fixed64.Zero, emptyResult);

        Assert.False(Fixed64.TryParse("abc", out var invalidResult));
        Assert.Equal(Fixed64.Zero, invalidResult);
    }

    [Fact]
    public void FormattingAndComparerMembers_WorkCorrectly()
    {
        var value = new Fixed64(1.2345);

        Assert.Equal("1.23", value.ToString("0.00"));
        Assert.Equal(0, value.CompareTo(new Fixed64(1.2345)));
        Assert.True(value.CompareTo(new Fixed64(1.2)) > 0);
        Assert.True(value.CompareTo(new Fixed64(1.3)) < 0);

        IEqualityComparer<Fixed64> comparer = new Fixed64();
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
        var value = new Fixed64(-1.75);
        var positive = new Fixed64(1.75);

        Assert.Equal(Fixed64.Sign(value), value.Sign());
        Assert.Equal(Fixed64.IsInteger(positive), positive.IsInteger());
        Assert.Equal(FixedMath.Squared(positive), positive.Squared());
        Assert.Equal(FixedMath.Round(positive), positive.Round());
        Assert.Equal(FixedMath.Round(positive, MidpointRounding.AwayFromZero), positive.Round(MidpointRounding.AwayFromZero));
        Assert.Equal(FixedMath.RoundToPrecision(positive, 1), positive.RoundToPrecision(1));
        Assert.Equal(FixedMath.ClampOne(new Fixed64(3)), new Fixed64(3).ClampOne());
        Assert.Equal(FixedMath.Clamp01(new Fixed64(-1)), new Fixed64(-1).Clamp01());
        Assert.Equal(FixedMath.Abs(value), value.Abs());
        Assert.True(new Fixed64(0.5).AbsLessThan(Fixed64.One));
        Assert.Equal(FixedMath.FastAdd(Fixed64.One, Fixed64.Two), Fixed64.One.FastAdd(Fixed64.Two));
        Assert.Equal(FixedMath.FastSub(Fixed64.Three, Fixed64.One), Fixed64.Three.FastSub(Fixed64.One));
        Assert.Equal(FixedMath.FastMul(Fixed64.Two, Fixed64.Three), Fixed64.Two.FastMul(Fixed64.Three));
        Assert.Equal(FixedMath.FastMod(new Fixed64(7), new Fixed64(3)), new Fixed64(7).FastMod(new Fixed64(3)));
        Assert.Equal(FixedMath.Floor(value), value.Floor());
        Assert.Equal(FixedMath.Ceiling(value), value.Ceiling());
        Assert.Equal((int)FixedMath.Round(positive), positive.RoundToInt());
        Assert.Equal((int)FixedMath.Ceiling(value), value.CeilToInt());
        Assert.Equal((int)FixedMath.Floor(value), value.FloorToInt());
    }

    [Fact]
    public void Fixed64Extensions_FormattingAndAngleHelpers_WorkCorrectly()
    {
        var value = new Fixed64(1.2345);

        Assert.Equal(1.23, value.ToFormattedDouble());
        Assert.Equal(1.23f, value.ToFormattedFloat());
        Assert.Equal((float)(double)value, value.ToPreciseFloat());

        var formatted = value.ToFormattedString();
        Assert.True(formatted is "1.23" or "1,23");

        Assert.Equal(FixedMath.DegToRad(new Fixed64(90)), new Fixed64(90).ToRadians());
        Assert.Equal(FixedMath.RadToDeg(FixedMath.PiOver2), FixedMath.PiOver2.ToDegree());
    }

    [Fact]
    public void Fixed64Extensions_EpsilonAndFuzzyHelpers_WorkCorrectly()
    {
        Assert.True((Fixed64.Epsilon + Fixed64.One).MoreThanEpsilon());
        Assert.False(Fixed64.Epsilon.MoreThanEpsilon());
        Assert.True(Fixed64.Zero.LessThanEpsilon());
        Assert.False(Fixed64.Epsilon.LessThanEpsilon());

        Assert.True(Fixed64.Zero.FuzzyComponentEqual(Fixed64.Epsilon, Fixed64.Epsilon));
        Assert.True(new Fixed64(100).FuzzyComponentEqual(new Fixed64(101), new Fixed64(0.02)));
        Assert.False(new Fixed64(100).FuzzyComponentEqual(new Fixed64(103), new Fixed64(0.02)));
    }

    #endregion

    #region Test: Serialization

    [Fact]
    public void Fixed64_NetSerialization_RoundTripMaintainsData()
    {
        var originalValue = FixedMath.PI;

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

    [Fact]
    public void Fixed64_MemoryPackSerialization_RoundTripMaintainsData()
    {
        Fixed64 originalValue = FixedMath.PI;

        byte[] bytes = MemoryPackSerializer.Serialize(originalValue);
        Fixed64 deserializedValue = MemoryPackSerializer.Deserialize<Fixed64>(bytes);

        // Check that deserialized values match the original
        Assert.Equal(originalValue, deserializedValue);
    }

    #endregion
}
