using MemoryPack;
using System.Text.Json;
using System.Text.Json.Serialization;
using Xunit;

namespace FixedMathSharp.Tests;

public class FixedCurveTests
{
    [Fact]
    public void Evaluate_LinearInterpolation_ShouldInterpolateCorrectly()
    {
        FixedCurve curve = new FixedCurve(FixedCurveMode.Linear,
            new FixedCurveKey(0, 0),
            new FixedCurveKey(10, 100));

        Assert.Equal(Fixed64.Zero, curve.Evaluate(Fixed64.Zero));
        Assert.Equal((Fixed64)50, curve.Evaluate((Fixed64)5)); // Midpoint
        Assert.Equal((Fixed64)100, curve.Evaluate((Fixed64)10));
    }

    [Fact]
    public void Evaluate_StepInterpolation_ShouldJumpToNearestKeyframe()
    {
        FixedCurve curve = new FixedCurve(FixedCurveMode.Step,
            new FixedCurveKey(0, 10),
            new FixedCurveKey(5, 50),
            new FixedCurveKey(10, 100));

        Assert.Equal((Fixed64)10, curve.Evaluate(Fixed64.Zero));
        Assert.Equal((Fixed64)10, curve.Evaluate((Fixed64)4.99)); // Should not interpolate
        Assert.Equal((Fixed64)50, curve.Evaluate((Fixed64)5));
        Assert.Equal((Fixed64)50, curve.Evaluate((Fixed64)9.99));
        Assert.Equal((Fixed64)100, curve.Evaluate((Fixed64)10));
    }

    [Fact]
    public void Evaluate_SmoothInterpolation_ShouldSmoothlyTransition()
    {
        FixedCurve curve = new FixedCurve(FixedCurveMode.Smooth,
            new FixedCurveKey(0, 0),
            new FixedCurveKey(10, 100));

        Fixed64 result = curve.Evaluate((Fixed64)5);
        Assert.True(result > (Fixed64)45 && result < (Fixed64)55, $"Unexpected smooth interpolation result: {result}");
    }

    [Fact]
    public void Evaluate_CubicInterpolation_ShouldUseTangentsCorrectly()
    {
        FixedCurve curve = new FixedCurve(FixedCurveMode.Cubic,
            new FixedCurveKey(0, 0, 10, 10),
            new FixedCurveKey(10, 100, -10, -10));

        Fixed64 result = curve.Evaluate((Fixed64)5);
        Assert.True(result > (Fixed64)45 && result < (Fixed64)55, $"Unexpected cubic interpolation result: {result}");
    }

    [Fact]
    public void Evaluate_TimeBeforeFirstKeyframe_ShouldReturnFirstValue()
    {
        FixedCurve curve = new FixedCurve(FixedCurveMode.Linear,
            new FixedCurveKey(5, 50),
            new FixedCurveKey(10, 100));

        Assert.Equal((Fixed64)50, curve.Evaluate(Fixed64.Zero));
    }

    [Fact]
    public void Evaluate_TimeAfterLastKeyframe_ShouldReturnLastValue()
    {
        FixedCurve curve = new FixedCurve(FixedCurveMode.Linear,
            new FixedCurveKey(5, 50),
            new FixedCurveKey(10, 100));

        Assert.Equal((Fixed64)100, curve.Evaluate((Fixed64)15));
    }

    [Fact]
    public void Evaluate_SingleKeyframe_ShouldAlwaysReturnSameValue()
    {
        FixedCurve curve = new FixedCurve(FixedCurveMode.Linear,
            new FixedCurveKey(5, 50));

        Assert.Equal((Fixed64)50, curve.Evaluate(Fixed64.Zero));
        Assert.Equal((Fixed64)50, curve.Evaluate((Fixed64)10));
    }

    [Fact]
    public void Evaluate_DuplicateKeyframes_ShouldHandleGracefully()
    {
        FixedCurve curve = new FixedCurve(FixedCurveMode.Linear,
            new FixedCurveKey(5, 50),
            new FixedCurveKey(5, 50), // Duplicate
            new FixedCurveKey(10, 100));

        Assert.Equal((Fixed64)50, curve.Evaluate((Fixed64)5));
        Assert.Equal((Fixed64)100, curve.Evaluate((Fixed64)10));
    }

    [Fact]
    public void Evaluate_NegativeValues_ShouldInterpolateCorrectly()
    {
        FixedCurve curve = new FixedCurve(FixedCurveMode.Linear,
            new FixedCurveKey(-10, -100),
            new FixedCurveKey(0, 0),
            new FixedCurveKey(10, 100));

        Assert.Equal((Fixed64)(-100), curve.Evaluate(-(Fixed64)10));
        Assert.Equal((Fixed64)(0), curve.Evaluate(Fixed64.Zero));
        Assert.Equal((Fixed64)(100), curve.Evaluate((Fixed64)10));
    }

    [Fact]
    public void Evaluate_ExtremeValues_ShouldHandleCorrectly()
    {
        FixedCurve curve = new FixedCurve(FixedCurveMode.Linear,
            new FixedCurveKey(Fixed64.MIN_VALUE, -(Fixed64)10000),
            new FixedCurveKey(Fixed64.MAX_VALUE, (Fixed64)10000));

        Assert.Equal((Fixed64)(-10000), curve.Evaluate(Fixed64.MIN_VALUE));
        Assert.Equal((Fixed64)(10000), curve.Evaluate(Fixed64.MAX_VALUE));
    }

    [Fact]
    public void Constructor_SortsKeyframesAndClonesInputArray()
    {
        FixedCurveKey[] keyframes =
        {
            new FixedCurveKey(10, 100),
            new FixedCurveKey(0, 0)
        };

        FixedCurve curve = new FixedCurve(FixedCurveMode.Linear, keyframes);
        keyframes[0] = new FixedCurveKey(20, 200);

        Assert.Equal(Fixed64.Zero, curve.Keyframes[0].Time);
        Assert.Equal(new Fixed64(10), curve.Keyframes[1].Time);
    }

    [Fact]
    public void Constructor_WithNullKeyframes_UsesEmptyArray()
    {
        FixedCurve curve = new FixedCurve(FixedCurveMode.Linear, null!);

        Assert.Empty(curve.Keyframes);
    }

    [Fact]
    public void Evaluate_NoKeyframes_ReturnsOne()
    {
        FixedCurve curve = new FixedCurve(FixedCurveMode.Linear, System.Array.Empty<FixedCurveKey>());

        Assert.Equal(Fixed64.One, curve.Evaluate(Fixed64.Zero));
    }

    [Fact]
    public void FixedCurve_EqualityOperatorsAndHashCode_WorkCorrectly()
    {
        FixedCurve curve = new FixedCurve(
            new FixedCurveKey(0, 0),
            new FixedCurveKey(10, 10));
        FixedCurve same = new FixedCurve(
            new FixedCurveKey(0, 0),
            new FixedCurveKey(10, 10));
        FixedCurve other = new FixedCurve(
            new FixedCurveKey(0, 0),
            new FixedCurveKey(10, 5));
        FixedCurve? nullCurve = null;

        Assert.True(curve == same);
        Assert.False(curve != same);
        Assert.True(curve != other);
        Assert.True(curve.Equals((object)same));
        Assert.False(curve.Equals(new object()));
        Assert.False(curve == nullCurve);
        Assert.True(nullCurve == null);
        Assert.Equal(curve.GetHashCode(), same.GetHashCode());
    }

    [Fact]
    public void FixedCurveKey_ConstructorsEqualityAndHashCode_WorkCorrectly()
    {
        FixedCurveKey fromDouble = new FixedCurveKey(1.5, 2.5, 3.5, 4.5);
        FixedCurveKey same = new FixedCurveKey(new Fixed64(1.5), new Fixed64(2.5), new Fixed64(3.5), new Fixed64(4.5));
        FixedCurveKey simple = new FixedCurveKey(1.5, 2.5);

        Assert.Equal(new Fixed64(1.5), fromDouble.Time);
        Assert.Equal(new Fixed64(2.5), fromDouble.Value);
        Assert.Equal(new Fixed64(3.5), fromDouble.InTangent);
        Assert.Equal(new Fixed64(4.5), fromDouble.OutTangent);
        Assert.Equal(Fixed64.Zero, simple.InTangent);
        Assert.Equal(Fixed64.Zero, simple.OutTangent);
        Assert.True(fromDouble == same);
        Assert.False(fromDouble != same);
        Assert.True(fromDouble.Equals((object)same));
        Assert.False(fromDouble.Equals(new object()));
        Assert.Equal(fromDouble.GetHashCode(), same.GetHashCode());
    }

    #region Test: Serialization

    [Fact]
    public void FixedCurve_NetSerialization_RoundTripMaintainsData()
    {
        var originalCurve = new FixedCurve(
            new FixedCurveKey(-10, -100),
            new FixedCurveKey(0, 0),
            new FixedCurveKey(10, 100));

        var jsonOptions = new JsonSerializerOptions
        {
            DefaultIgnoreCondition = JsonIgnoreCondition.WhenWritingNull,
            ReferenceHandler = ReferenceHandler.IgnoreCycles,
        };
        var json = JsonSerializer.SerializeToUtf8Bytes(originalCurve, jsonOptions);
        var deserializedCurve = JsonSerializer.Deserialize<FixedCurve>(json, jsonOptions);
        Assert.NotNull(deserializedCurve);

        // Check that deserialized values match the original
        Assert.Equal(originalCurve, deserializedCurve);
    }

    [Fact]
    public void FixedCurve_MemoryPackSerialization_RoundTripMaintainsData()
    {
        FixedCurve originalValue = new FixedCurve(
            new FixedCurveKey(-10, -100),
            new FixedCurveKey(0, 0),
            new FixedCurveKey(10, 100));

        byte[] bytes = MemoryPackSerializer.Serialize(originalValue);
        FixedCurve deserializedValue = MemoryPackSerializer.Deserialize<FixedCurve>(bytes)!;

        // Check that deserialized values match the original
        Assert.Equal(originalValue, deserializedValue);
    }

    #endregion
}
