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