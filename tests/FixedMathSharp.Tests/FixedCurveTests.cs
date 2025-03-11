#if NET48_OR_GREATER
using System.IO;
using System.Runtime.Serialization.Formatters.Binary;
using System.Security.Cryptography;

#endif

#if NET8_0_OR_GREATER
using System.Text.Json;
using System.Text.Json.Serialization;
#endif

using Xunit;

namespace FixedMathSharp.Tests
{
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
        public void FixedCurve_Serialization_RoundTripMaintainsData()
        {
            var originalCurve = new FixedCurve(
                new FixedCurveKey(-10, -100),
                new FixedCurveKey(0, 0),
                new FixedCurveKey(10, 100));

            // Serialize the Fixed3x3 object
#if NET48_OR_GREATER
            var formatter = new BinaryFormatter();
            using var stream = new MemoryStream();
            formatter.Serialize(stream, originalCurve);

            // Reset stream position and deserialize
            stream.Seek(0, SeekOrigin.Begin);
            var deserializedCurve = (FixedCurve)formatter.Deserialize(stream);
#endif

#if NET8_0_OR_GREATER
            var jsonOptions = new JsonSerializerOptions
            {
                DefaultIgnoreCondition = JsonIgnoreCondition.WhenWritingNull,
                ReferenceHandler = ReferenceHandler.IgnoreCycles,
                IncludeFields = true,
                IgnoreReadOnlyProperties = true
            };
            var json = JsonSerializer.SerializeToUtf8Bytes(originalCurve, jsonOptions);
            var deserializedCurve = JsonSerializer.Deserialize<FixedCurve>(json, jsonOptions);
#endif

            // Check that deserialized values match the original
            Assert.Equal(originalCurve, deserializedCurve);
        }
    }
}