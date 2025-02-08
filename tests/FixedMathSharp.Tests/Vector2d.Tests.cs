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
    public class Vector2dTests
    {
        [Fact]
        public void RotatedRight_Rotates90DegreesClockwise()
        {
            var vector = new Vector2d(1, 0);
            var result = vector.RotatedRight;

            Assert.Equal(new Vector2d(0, -1), result); // (1, 0) rotated 90° clockwise becomes (0, -1)
        }

        [Fact]
        public void RotatedLeft_Rotates90DegreesCounterclockwise()
        {
            var vector = new Vector2d(1, 0);
            var result = vector.RotatedLeft;

            Assert.Equal(new Vector2d(0, 1), result); // (1, 0) rotated 90° counterclockwise becomes (0, 1)
        }

        [Fact]
        public void RightHandNormal_ReturnsCorrectNormalVector()
        {
            var vector = new Vector2d(1, 0);
            var result = vector.RightHandNormal;

            Assert.Equal(new Vector2d(0, 1), result); // The right-hand normal of (1, 0) is (0, 1)
        }

        [Fact]
        public void LeftHandNormal_ReturnsCorrectNormalVector()
        {
            var vector = new Vector2d(1, 0);
            var result = vector.LeftHandNormal;

            Assert.Equal(new Vector2d(0, -1), result); // The left-hand normal of (1, 0) is (0, -1)
        }

        [Fact]
        public void MyMagnitude_CalculatesCorrectMagnitude()
        {
            var vector = new Vector2d(3, 4);
            var result = vector.Magnitude;

            Assert.Equal(new Fixed64(5), result); // The magnitude of (3, 4) is 5 (3-4-5 triangle)
        }

        [Fact]
        public void SqrMagnitude_CalculatesCorrectSquareMagnitude()
        {
            var vector = new Vector2d(3, 4);
            var result = vector.SqrMagnitude;

            Assert.Equal(new Fixed64(25), result); // The squared magnitude of (3, 4) is 25 (3^2 + 4^2)
        }


        [Fact]
        public void NormalizeInPlace_NormalizesVectorCorrectly()
        {
            var vector = new Vector2d(3, 4);
            vector.Normalize();

            var expected = new Vector2d(new Fixed64(0.6), new Fixed64(0.8)); // Normalized vector (0.6, 0.8)
            Assert.True(vector.FuzzyEqual(expected, new Fixed64(0.0001)));
        }

        [Fact]
        public void LerpInPlace_InterpolatesBetweenVectorsCorrectly()
        {
            var start = new Vector2d(0, 0);
            var end = new Vector2d(10, 10);
            var amount = new Fixed64(0.5); // 50% interpolation

            start.LerpInPlace(end, amount);
            Assert.Equal(new Vector2d(5, 5), start); // Should be halfway between (0, 0) and (10, 10)
        }

        [Fact]
        public void RotateInPlace_RotatesVectorCorrectly()
        {
            var vector = new Vector2d(1, 0);
            var cos = FixedMath.Cos(FixedMath.PiOver2); // 90° cosine
            var sin = FixedMath.Sin(FixedMath.PiOver2); // 90° sine

            vector.RotateInPlace(cos, sin);
            Assert.True(vector.FuzzyEqual(new Vector2d(0, 1), new Fixed64(0.0001))); // (1, 0) rotated 90° becomes (0, 1)
        }

        [Fact]
        public void RotateInverse_RotatesVectorInOppositeDirection()
        {
            var vector = new Vector2d(1, 0);
            var cos = FixedMath.Cos(FixedMath.PiOver2); // 90° cosine
            var sin = FixedMath.Sin(FixedMath.PiOver2); // 90° sine

            vector.RotateInverse(cos, sin);
            Assert.True(vector.FuzzyEqual(new Vector2d(0, -1), new Fixed64(0.0001))); // Should rotate -90° to (0, -1)
        }

        [Fact]
        public void RotateRightInPlace_RotatesVector90DegreesClockwise()
        {
            var vector = new Vector2d(1, 0);
            vector.RotateRightInPlace();

            Assert.Equal(new Vector2d(0, -1), vector); // (1, 0) rotated 90° clockwise becomes (0, -1)
        }

        [Fact]
        public void RotateLeftInPlace_RotatesVector90DegreesCounterclockwise()
        {
            var vector = new Vector2d(1, 0);
            vector.RotateLeftInPlace();

            Assert.Equal(new Vector2d(0, 1), vector); // (1, 0) rotated 90° counterclockwise becomes (0, 1)
        }

        [Fact]
        public void ScaleInPlace_ScalesVectorCorrectly()
        {
            var vector = new Vector2d(2, 3);
            var scaleFactor = new Fixed64(2);
            vector.ScaleInPlace(scaleFactor);

            Assert.Equal(new Vector2d(4, 6), vector); // (2, 3) scaled by 2 becomes (4, 6)
        }

        [Fact]
        public void Dot_ComputesDotProductCorrectly()
        {
            var vector1 = new Vector2d(1, 2);
            var vector2 = new Vector2d(3, 4);
            var result = vector1.Dot(vector2);

            Assert.Equal(new Fixed64(11), result); // Dot product of (1, 2) and (3, 4) is 1*3 + 2*4 = 11
        }

        [Fact]
        public void Cross_ComputesCrossProductCorrectly()
        {
            var vector1 = new Vector2d(1, 2);
            var vector2 = new Vector2d(3, 4);
            var result = vector1.CrossProduct(vector2);

            Assert.Equal(new Fixed64(-2), result); // Cross product of (1, 2) and (3, 4) is 1*4 - 2*3 = -2
        }

        [Fact]
        public void Distance_ComputesDistanceCorrectly()
        {
            var vector1 = new Vector2d(1, 1);
            var vector2 = new Vector2d(4, 5);
            var result = vector1.Distance(vector2);

            Assert.Equal(new Fixed64(5), result); // Distance between (1, 1) and (4, 5) is 5 (3-4-5 triangle)
        }

        [Fact]
        public void SqrDistance_ComputesSquareDistanceCorrectly()
        {
            var vector1 = new Vector2d(1, 1);
            var vector2 = new Vector2d(4, 5);
            var result = vector1.SqrDistance(vector2);

            Assert.Equal(new Fixed64(25), result); // Squared distance between (1, 1) and (4, 5) is 25
        }

        [Fact]
        public void ReflectInPlace_ReflectsVectorCorrectly()
        {
            var vector = new Vector2d(1, 1);
            var axisX = new Fixed64(0);
            var axisY = new Fixed64(1); // Reflect across the Y-axis

            vector.ReflectInPlace(axisX, axisY);
            Assert.Equal(new Vector2d(-1, 1), vector); // Reflecting (1, 1) across Y-axis becomes (-1, 1)
        }

        [Fact]
        public void AddInPlace_AddsToVectorCorrectly()
        {
            var vector = new Vector2d(1, 1);
            var amount = new Fixed64(2);
            vector.AddInPlace(amount);

            Assert.Equal(new Vector2d(3, 3), vector); // (1, 1) + 2 becomes (3, 3)
        }

        [Fact]
        public void SubtractInPlace_SubtractsFromVectorCorrectly()
        {
            var vector = new Vector2d(3, 3);
            var amount = new Fixed64(1);
            vector.SubtractInPlace(amount);

            Assert.Equal(new Vector2d(2, 2), vector); // (3, 3) - 1 becomes (2, 2)
        }

        [Fact]
        public void NormalizeInPlace_DoesNothingForZeroVector()
        {
            var vector = new Vector2d(0, 0);
            vector.Normalize();

            Assert.Equal(new Vector2d(0, 0), vector); // A zero vector remains zero after normalization
        }

        [Fact]
        public void V2ClampOneInPlace_ClampsCorrectly()
        {
            var vector = new Vector2d(2, -3);
            var result = vector.ClampOneInPlace();

            Assert.Equal(new Vector2d(1, -1), result); // Clamps x and y to [-1, 1]
        }

        [Fact]
        public void V2ToDegrees_ConvertsCorrectly()
        {
            var radians = new Vector2d(FixedMath.PiOver2, FixedMath.PI); // (90°, 180°)
            var result = radians.ToDegrees();

            Assert.True(result.FuzzyEqual(new Vector2d(90, 180))); // Converts radians to degrees
        }

        [Fact]
        public void V2ToRadians_ConvertsCorrectly()
        {
            var degrees = new Vector2d(90, 180);
            var result = degrees.ToRadians();

            Assert.True(result.FuzzyEqual(new Vector2d(FixedMath.PiOver2, FixedMath.PI))); // Converts degrees to radians
        }

        [Fact]
        public void V2FuzzyEqualAbsolute_ComparesCorrectly_WithAllowedDifference()
        {
            var vector1 = new Vector2d(2, 2);
            var vector2 = new Vector2d(2.1, 2.1);
            var allowedDifference = new Fixed64(0.15);

            Assert.True(vector1.FuzzyEqualAbsolute(vector2, allowedDifference)); // Approximate equality with a 0.15 difference
        }

        [Fact]
        public void V2FuzzyEqual_ComparesCorrectly_WithDefaultTolerance()
        {
            var vector1 = new Vector2d(100, 100);
            var vector2 = new Vector2d(100.0000008537, 100.0000008537); // Small difference

            // Use FuzzyEqual with the default tolerance, which is small (e.g., 0.01% difference)
            Assert.True(vector1.FuzzyEqual(vector2)); // The difference should be within the default tolerance

            vector2 = new Vector2d(100.0001, 100.0001); // big difference
            Assert.False(vector1.FuzzyEqual(vector2)); // The difference should be outside the default tolerance
        }


        [Fact]
        public void V2FuzzyEqual_ComparesCorrectly_WithCustomPercentage()
        {
            var vector1 = new Vector2d(100, 100);
            var vector2 = new Vector2d(102, 102);
            var percentage = new Fixed64(0.02); // Allow a 2% difference

            Assert.True(vector1.FuzzyEqual(vector2, percentage)); // Should be approximately equal within 2% difference
        }

        [Fact]
        public void V2CheckDistance_VerifiesDistanceCorrectly()
        {
            var vector1 = new Vector2d(0, 0);
            var vector2 = new Vector2d(3, 4); // Distance is 5 (3-4-5 triangle)
            var factor = new Fixed64(5);

            Assert.True(vector1.CheckDistance(vector2, factor)); // Distance is 5, so should return true
        }

        [Fact]
        public void V2SqrDistance_CalculatesCorrectly()
        {
            var vector1 = new Vector2d(0, 0);
            var vector2 = new Vector2d(3, 4); // Squared distance should be 25
            var result = vector1.SqrDistance(vector2);

            Assert.Equal(new Fixed64(25), result); // 3^2 + 4^2 = 25
        }

        [Fact]
        public void V2Rotate_RotatesVectorCorrectly()
        {
            var vector = new Vector2d(1, 0);
            var angle = FixedMath.PiOver2; // Rotate by 90° (π/2 radians)
            var result = vector.Rotate(angle);

            Assert.True(result.FuzzyEqual(new Vector2d(0, 1), new Fixed64(0.0001))); // Should rotate to (0, 1)
        }

        [Fact]
        public void Vector2d_Serialization_RoundTripMaintainsData()
        {
            var originalValue = new Vector2d(FixedMath.PI, FixedMath.PiOver2);

            // Serialize the Vector3d object
#if NET48_OR_GREATER
            var formatter = new BinaryFormatter();
            using var stream = new MemoryStream();
            formatter.Serialize(stream, originalValue);

            // Reset stream position and deserialize
            stream.Seek(0, SeekOrigin.Begin);
            var deserializedValue = (Vector2d)formatter.Deserialize(stream);
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
            var deserializedValue = JsonSerializer.Deserialize<Vector2d>(json, jsonOptions);
#endif

            // Check that deserialized values match the original
            Assert.Equal(originalValue, deserializedValue);
        }
    }
}
