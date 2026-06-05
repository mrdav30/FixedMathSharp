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
        FixedCurve curve = new(FixedCurveMode.Linear,
            new FixedCurveKey(0, 0),
            new FixedCurveKey(10, 100));

        Assert.Equal(Fixed64.Zero, curve.Evaluate(Fixed64.Zero));
        Assert.Equal((Fixed64)50, curve.Evaluate((Fixed64)5)); // Midpoint
        Assert.Equal((Fixed64)100, curve.Evaluate((Fixed64)10));
    }

    [Fact]
    public void Evaluate_StepInterpolation_ShouldJumpToNearestKeyframe()
    {
        FixedCurve curve = new(FixedCurveMode.Step,
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
        FixedCurve curve = new(FixedCurveMode.Smooth,
            new FixedCurveKey(0, 0),
            new FixedCurveKey(10, 100));

        Fixed64 result = curve.Evaluate((Fixed64)5);
        Assert.True(result > (Fixed64)45 && result < (Fixed64)55, $"Unexpected smooth interpolation result: {result}");
    }

    [Fact]
    public void Evaluate_CubicInterpolation_ShouldUseTangentsCorrectly()
    {
        FixedCurve curve = new(FixedCurveMode.Cubic,
            new FixedCurveKey(0, 0, 10, 10),
            new FixedCurveKey(10, 100, -10, -10));

        Fixed64 result = curve.Evaluate((Fixed64)5);
        Assert.True(result > (Fixed64)45 && result < (Fixed64)55, $"Unexpected cubic interpolation result: {result}");
    }

    [Fact]
    public void Evaluate_TimeBeforeFirstKeyframe_ShouldReturnFirstValue()
    {
        FixedCurve curve = new(FixedCurveMode.Linear,
            new FixedCurveKey(5, 50),
            new FixedCurveKey(10, 100));

        Assert.Equal((Fixed64)50, curve.Evaluate(Fixed64.Zero));
    }

    [Fact]
    public void Evaluate_TimeAfterLastKeyframe_ShouldReturnLastValue()
    {
        FixedCurve curve = new(FixedCurveMode.Linear,
            new FixedCurveKey(5, 50),
            new FixedCurveKey(10, 100));

        Assert.Equal((Fixed64)100, curve.Evaluate((Fixed64)15));
    }

    [Fact]
    public void Evaluate_SingleKeyframe_ShouldAlwaysReturnSameValue()
    {
        FixedCurve curve = new(FixedCurveMode.Linear,
            new FixedCurveKey(5, 50));

        Assert.Equal((Fixed64)50, curve.Evaluate(Fixed64.Zero));
        Assert.Equal((Fixed64)50, curve.Evaluate((Fixed64)10));
    }

    [Fact]
    public void Evaluate_DuplicateKeyframes_ShouldHandleGracefully()
    {
        FixedCurve curve = new(FixedCurveMode.Linear,
            new FixedCurveKey(5, 50),
            new FixedCurveKey(5, 50), // Duplicate
            new FixedCurveKey(10, 100));

        Assert.Equal((Fixed64)50, curve.Evaluate((Fixed64)5));
        Assert.Equal((Fixed64)100, curve.Evaluate((Fixed64)10));
    }

    [Fact]
    public void Evaluate_NegativeValues_ShouldInterpolateCorrectly()
    {
        FixedCurve curve = new(FixedCurveMode.Linear,
            new FixedCurveKey(-10, -100),
            new FixedCurveKey(0, 0),
            new FixedCurveKey(10, 100));

        Assert.Equal((Fixed64)(-100), curve.Evaluate(-(Fixed64)10));
        Assert.Equal((Fixed64)(0), curve.Evaluate(Fixed64.Zero));
        Assert.Equal((Fixed64)(100), curve.Evaluate((Fixed64)10));
    }

    [Fact]
    public void Evaluate_TimeWithinSecondSegment_SkipsEarlierSegment()
    {
        FixedCurve curve = new(FixedCurveMode.Linear,
            new FixedCurveKey(0, 0),
            new FixedCurveKey(10, 100),
            new FixedCurveKey(20, 200));

        Assert.Equal(new Fixed64(150), curve.Evaluate(new Fixed64(15)));
    }

    [Fact]
    public void Evaluate_ExtremeValues_ShouldHandleCorrectly()
    {
        FixedCurve curve = new(FixedCurveMode.Linear,
            new FixedCurveKey(Fixed64.MinValue, -(Fixed64)10000),
            new FixedCurveKey(Fixed64.MaxValue, (Fixed64)10000));

        Assert.Equal((Fixed64)(-10000), curve.Evaluate(Fixed64.MinValue));
        Assert.Equal((Fixed64)(10000), curve.Evaluate(Fixed64.MaxValue));
    }

    [Fact]
    public void Constructor_SortsKeyframesInPlace()
    {
        FixedCurveKey[] keyframes =
        {
            new(10, 100),
            new(0, 0)
        };

        FixedCurve curve = new(FixedCurveMode.Linear, keyframes);

        Assert.Equal(Fixed64.Zero, curve.Keyframes[0].Time);
        Assert.Equal(new Fixed64(10), curve.Keyframes[1].Time);
        Assert.Same(keyframes, curve.Keyframes);
        Assert.Equal(Fixed64.Zero, keyframes[0].Time);
        Assert.Equal(new Fixed64(10), keyframes[1].Time);
    }

    [Fact]
    public void Constructor_WithNullKeyframes_UsesEmptyArray()
    {
        FixedCurve curve = new(FixedCurveMode.Linear, null!);

        Assert.Empty(curve.Keyframes);
    }

    [Fact]
    public void Evaluate_NoKeyframes_ReturnsOne()
    {
        FixedCurve curve = new(FixedCurveMode.Linear, System.Array.Empty<FixedCurveKey>());

        Assert.Equal(Fixed64.One, curve.Evaluate(Fixed64.Zero));
    }

    [Fact]
    public void FixedCurve_EqualityOperatorsAndHashCode_WorkCorrectly()
    {
        FixedCurve curve = new(
            new FixedCurveKey(0, 0),
            new FixedCurveKey(10, 10));
        FixedCurve same = new(
            new FixedCurveKey(0, 0),
            new FixedCurveKey(10, 10));
        FixedCurve other = new(
            new FixedCurveKey(0, 0),
            new FixedCurveKey(10, 5));
        FixedCurve? nullCurve = null;

        Assert.True(curve == same);
        Assert.False(curve != same);
        Assert.True(curve != other);
        Assert.True(curve.Equals(curve));
        Assert.False(curve.Equals((FixedCurve?)null));
        Assert.True(curve.Equals((object)same));
        Assert.False(curve.Equals(new object()));
        Assert.False(curve.Equals(new FixedCurve(
            FixedCurveMode.Step,
            new FixedCurveKey(0, 0),
            new FixedCurveKey(10, 10))));
        Assert.False(curve == nullCurve);
        Assert.Null(nullCurve);
        Assert.Equal(curve.GetHashCode(), same.GetHashCode());
    }

    [Fact]
    public void FixedCurveKey_ConstructorsEqualityAndHashCode_WorkCorrectly()
    {
        FixedCurveKey fromDouble = FixedCurveKey.FromFloatPoint(1.5, 2.5, 3.5, 4.5);
        FixedCurveKey same = new(Fixed64.FromFloatPoint(1.5), Fixed64.FromFloatPoint(2.5), Fixed64.FromFloatPoint(3.5), Fixed64.FromFloatPoint(4.5));
        FixedCurveKey simple = FixedCurveKey.FromFloat(1.5, 2.5);

        Assert.Equal(Fixed64.FromFloatPoint(1.5), fromDouble.Time);
        Assert.Equal(Fixed64.FromFloatPoint(2.5), fromDouble.Value);
        Assert.Equal(Fixed64.FromFloatPoint(3.5), fromDouble.InTangent);
        Assert.Equal(Fixed64.FromFloatPoint(4.5), fromDouble.OutTangent);
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

        // Check that deserialized values match the original
        Assert.Equal(originalCurve, deserializedCurve);
    }

#if !FIXEDMATHSHARP_DISABLE_MEMORYPACK
    [Fact]
    public void FixedCurve_MemoryPackSerialization_RoundTripMaintainsData()
    {
        FixedCurve originalValue = new(
            new FixedCurveKey(-10, -100),
            new FixedCurveKey(0, 0),
            new FixedCurveKey(10, 100));

        byte[] bytes = MemoryPackSerializer.Serialize(originalValue);
        FixedCurve deserializedValue = MemoryPackSerializer.Deserialize<FixedCurve>(bytes)!;

        // Check that deserialized values match the original
        Assert.Equal(originalValue, deserializedValue);
    }
#endif

    #endregion
}
