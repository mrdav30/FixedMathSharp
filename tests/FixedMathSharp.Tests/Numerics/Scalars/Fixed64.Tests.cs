using MemoryPack;
using System;
using System.Globalization;
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
    }

    [Fact]
    public void LongArithmeticOperators_AreNotPublicApi()
    {
        foreach (var method in typeof(Fixed64).GetMethods(System.Reflection.BindingFlags.Public | System.Reflection.BindingFlags.Static))
        {
            if (!IsArithmeticOperatorName(method.Name))
                continue;

            var parameters = method.GetParameters();
            if (parameters.Length != 2 || method.ReturnType != typeof(Fixed64))
                continue;

            bool usesLong = parameters[0].ParameterType == typeof(long) || parameters[1].ParameterType == typeof(long);
            bool usesFixed64 = parameters[0].ParameterType == typeof(Fixed64) || parameters[1].ParameterType == typeof(Fixed64);

            Assert.False(usesLong && usesFixed64, $"{method.Name} should require explicit Fixed64 conversion or Fixed64.FromRaw for long operands.");
        }
    }

    [Fact]
    public void Fixed64_DoesNotExposeScalarAlgorithmMethods()
    {
        string[] movedMethodNames =
        {
            "Lerp",
            "SmoothStep",
            "CubicInterpolate",
            "CatmullRom",
            "HermiteSpline",
            "BarycentricCoordinate"
        };

        foreach (string methodName in movedMethodNames)
        {
            Assert.Empty(typeof(Fixed64).GetMember(methodName, System.Reflection.BindingFlags.Public | System.Reflection.BindingFlags.Static));
        }
    }

    private static bool IsArithmeticOperatorName(string name)
    {
        switch (name)
        {
            case "op_Addition":
            case "op_Subtraction":
            case "op_Multiply":
            case "op_Division":
            case "op_Modulus":
                return true;
            default:
                return false;
        }
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
        Assert.Equal(Fixed64.FromDouble(-2.5), Fixed64.Parse("-2.5"));
        Assert.Equal(Fixed64.FromDouble(1234.5), Fixed64.Parse("1,234.5", CultureInfo.InvariantCulture));
        Assert.Throws<ArgumentNullException>(() => Fixed64.Parse(null!));
        Assert.Throws<FormatException>(() => Fixed64.Parse(string.Empty));
        Assert.Throws<FormatException>(() => Fixed64.Parse("abc"));
        Assert.Throws<FormatException>(() => Fixed64.Parse("NaN"));
        Assert.Throws<OverflowException>(() => Fixed64.Parse("2147483648", CultureInfo.InvariantCulture));

        Assert.True(Fixed64.TryParse("1.25", out var parsedPositive));
        Assert.Equal(Fixed64.FromDouble(1.25), parsedPositive);

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
        Assert.Equal(Fixed64.FromDecimal(1.25m), (Fixed64)1.25m);
    }

    [Fact]
    public void FormattingAndComparerMembers_WorkCorrectly()
    {
        var value = Fixed64.FromDouble(1.2345f);

        Assert.Equal("1.23", value.ToString("0.00"));
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
    }

    [Fact]
    public void Fixed64Extensions_AngleHelpers_WorkCorrectly()
    {
        Assert.Equal(FixedMath.DegToRad(new Fixed64(90)), new Fixed64(90).ToRadians());
        Assert.Equal(FixedMath.RadToDeg(Fixed64.HalfPi), Fixed64.HalfPi.ToDegrees());
        Assert.Equal(FixedMath.Sin(Fixed64.HalfPi), Fixed64.HalfPi.Sin());
        Assert.Equal(FixedMath.Cos(Fixed64.HalfPi), Fixed64.HalfPi.Cos());
        Assert.Equal(FixedMath.Tan(Fixed64.PiOver4), Fixed64.PiOver4.Tan());
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
