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
    public class Vector3dTests
    {
        #region Test: Constructors

        [Fact]
        public void Constructor_Default_InitializesToZero()
        {
            var vector = new Vector3d();
            Assert.Equal(Fixed64.Zero, vector.x);
            Assert.Equal(Fixed64.Zero, vector.y);
            Assert.Equal(Fixed64.Zero, vector.z);
        }

        [Fact]
        public void Constructor_Fixed64_InitializesComponents()
        {
            var x = new Fixed64(1);
            var y = new Fixed64(2);
            var z = new Fixed64(3);
            var vector = new Vector3d(x, y, z);
            Assert.Equal(x, vector.x);
            Assert.Equal(y, vector.y);
            Assert.Equal(z, vector.z);
        }

        [Fact]
        public void Constructor_Float_InitializesComponents()
        {
            var vector = new Vector3d(1.5f, -2.5f, 3.0f);
            Assert.Equal(new Fixed64(1.5), vector.x);
            Assert.Equal(new Fixed64(-2.5), vector.y);
            Assert.Equal(new Fixed64(3.0), vector.z);
        }

        #endregion

        #region Test: Magnitude and Normalization

        [Fact]
        public void MyMagnitude_CalculatesCorrectly()
        {
            var vector = new Vector3d(3, 4, 0); // Magnitude should be 5 (3-4-5 triangle).
            Assert.Equal(new Fixed64(5), vector.Magnitude);
        }

        [Fact]
        public void Normalize_ProducesUnitVector()
        {
            var vector = new Vector3d(3, 4, 0);
            var normalized = vector.Normal;
            FixedMathTestHelper.AssertWithinRelativeTolerance(Fixed64.One, normalized.Magnitude);
        }

        [Fact]
        public void NormalizeInPlace_ProducesUnitVector()
        {
            var vector = new Vector3d(0, 0, 5);
            vector.Normalize();
            FixedMathTestHelper.AssertWithinRelativeTolerance(Fixed64.One, vector.Magnitude);
        }

        [Fact]
        public void IsNormalized_IdentifiesNormalizedVector()
        {
            var normalizedVector = new Vector3d(1, 0, 0).Normalize();
            Assert.True(normalizedVector.IsNormalized());
        }

        [Fact]
        public void IsNormalized_IdentifiesNonNormalizedVector()
        {
            var nonNormalizedVector = new Vector3d(10, 0, 0);
            Assert.False(nonNormalizedVector.IsNormalized());
        }

        [Fact]
        public void IsNormalized_ZeroVector_ReturnsFalse()
        {
            var zeroVector = new Vector3d(0, 0, 0);
            Assert.False(zeroVector.IsNormalized());
        }

        #endregion

        #region Test: Arithmetic 

        [Fact]
        public void Addition_AddsComponentsCorrectly()
        {
            var v1 = new Vector3d(1, 2, 3);
            var v2 = new Vector3d(4, 5, 6);
            var result = v1 + v2;
            Assert.Equal(new Vector3d(5, 7, 9), result);
        }

        [Fact]
        public void Subtraction_SubtractsComponentsCorrectly()
        {
            var v1 = new Vector3d(5, 7, 9);
            var v2 = new Vector3d(1, 2, 3);
            var result = v1 - v2;
            Assert.Equal(new Vector3d(4, 5, 6), result);
        }

        [Fact]
        public void Scaling_ScalesComponentsCorrectly()
        {
            var vector = new Vector3d(1, -2, 3);
            var result = vector * new Fixed64(2);
            Assert.Equal(new Vector3d(2, -4, 6), result);
        }

        [Fact]
        public void Division_DividesComponentsCorrectly()
        {
            var vector = new Vector3d(2, -4, 6);
            var result = vector / new Fixed64(2);
            Assert.Equal(new Vector3d(1, -2, 3), result);
        }

        #endregion

        #region Test: Dot and Cross Product

        [Fact]
        public void Dot_ProductCalculatesCorrectly()
        {
            var v1 = new Vector3d(1, 0, 0);
            var v2 = new Vector3d(0, 1, 0);
            Assert.Equal(Fixed64.Zero, Vector3d.Dot(v1, v2)); // Perpendicular vectors, dot product should be zero.
        }

        [Fact]
        public void Cross_ProductCalculatesCorrectly()
        {
            var v1 = new Vector3d(1, 0, 0);
            var v2 = new Vector3d(0, 1, 0);
            var result = Vector3d.Cross(v1, v2);
            Assert.Equal(new Vector3d(0, 0, 1), result); // Standard right-hand rule cross product.
        }

        #endregion

        #region Test: Distance and SqrDistance

        [Fact]
        public void Distance_CalculatesCorrectly()
        {
            var v1 = new Vector3d(0, 0, 0);
            var v2 = new Vector3d(0, 3, 4);
            Assert.Equal(new Fixed64(5), Vector3d.Distance(v1, v2)); // 3-4-5 triangle.
        }

        [Fact]
        public void SqrDistance_CalculatesCorrectly()
        {
            var v1 = new Vector3d(0, 0, 0);
            var v2 = new Vector3d(0, 3, 4);
            Assert.Equal(new Fixed64(25), Vector3d.SqrDistance(v1, v2)); // 3^2 + 4^2 = 25.
        }

        #endregion

        #region Test: Projection 

        [Fact]
        public void Project_ReturnsCorrectProjection()
        {
            var v1 = new Vector3d(2, 3, 4);
            var v2 = new Vector3d(1, 0, 0); // Project onto the X-axis.
            var result = Vector3d.Project(v1, v2);
            Assert.Equal(new Vector3d(2, 0, 0), result);
        }

        [Fact]
        public void ProjectOnPlane_ReturnsCorrectProjection()
        {
            var v1 = new Vector3d(1, 1, 1);
            var planeNormal = new Vector3d(0, 1, 0); // Y-plane.
            var result = Vector3d.ProjectOnPlane(v1, planeNormal);
            Assert.Equal(new Vector3d(1, 0, 1), result);
        }

        #endregion

        #region Test: Rotation 

        [Fact]
        public void RightHandNormal_RotatesCorrectly()
        {
            var vector = new Vector3d(1, 0, 0);
            var result = vector.RightHandNormal;
            Assert.Equal(new Vector3d(0, 0, -1), result);
        }

        [Fact]
        public void LeftHandNormal_RotatesCorrectly()
        {
            var vector = new Vector3d(1, 0, 0);
            var result = vector.LeftHandNormal;
            Assert.Equal(new Vector3d(0, 0, 1), result);
        }

        #endregion

        #region Test: Equality 

        [Fact]
        public void Equality_TwoIdenticalVectors_AreEqual()
        {
            var v1 = new Vector3d(1, 2, 3);
            var v2 = new Vector3d(1, 2, 3);
            Assert.True(v1 == v2);
        }

        [Fact]
        public void Equality_TwoDifferentVectors_AreNotEqual()
        {
            var v1 = new Vector3d(1, 2, 3);
            var v2 = new Vector3d(3, 2, 1);
            Assert.True(v1 != v2);
        }

        [Fact]
        public void Vector3d_Serialization_RoundTripMaintainsData()
        {
            var originalValue = new Vector3d(FixedMath.PI, FixedMath.PiOver2, FixedMath.TwoPI);

            // Serialize the Vector3d object
#if NET48_OR_GREATER
            var formatter = new BinaryFormatter();
            using var stream = new MemoryStream();
            formatter.Serialize(stream, originalValue);

            // Reset stream position and deserialize
            stream.Seek(0, SeekOrigin.Begin);
            var deserializedValue = (Vector3d)formatter.Deserialize(stream);
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
            var deserializedValue = JsonSerializer.Deserialize<Vector3d>(json, jsonOptions);
#endif

            // Check that deserialized values match the original
            Assert.Equal(originalValue, deserializedValue);
        }

#endregion

        #region Test: Static Math

        [Fact]
        public void Lerp_ReturnsCorrectInterpolation()
        {
            var start = new Vector3d(1, 1, 1);
            var end = new Vector3d(3, 3, 3);

            // Lerp at t = 0 (should return the start vector)
            var result = Vector3d.Lerp(start, end, Fixed64.Zero);
            Assert.Equal(start, result);

            // Lerp at t = 1 (should return the end vector)
            result = Vector3d.Lerp(start, end, Fixed64.One);
            Assert.Equal(end, result);

            // Lerp at t = 0.5 (should be the midpoint)
            var midpoint = new Vector3d(2, 2, 2);
            result = Vector3d.Lerp(start, end, new Fixed64(0.5));
            Assert.Equal(midpoint, result);
        }

        [Fact]
        public void UnclampedLerp_AllowsInterpolationOutsideRange()
        {
            var start = new Vector3d(1, 1, 1);
            var end = new Vector3d(3, 3, 3);

            // Unclamped Lerp at t = -1 (should go past start)
            var result = Vector3d.UnclampedLerp(start, end, new Fixed64(-1));
            var expected = new Vector3d(-1, -1, -1);
            Assert.Equal(expected, result);

            // Unclamped Lerp at t = 2 (should go past end)
            result = Vector3d.UnclampedLerp(start, end, new Fixed64(2));
            expected = new Vector3d(5, 5, 5);
            Assert.Equal(expected, result);
        }

        [Fact]
        public void SpeedLerp_DoesNotOvershoot()
        {
            var start = new Vector3d(0, 0, 0);
            var end = new Vector3d(10, 0, 0);
            var speed = new Fixed64(5);
            var dt = Fixed64.One; // Simulating 1 second of delta time

            // Speed lerp should move the vector but not overshoot the target
            var result = Vector3d.SpeedLerp(start, end, speed, dt);
            var expected = new Vector3d(5, 0, 0);
            Assert.Equal(expected, result);

            // Test where it shouldn't overshoot
            result = Vector3d.SpeedLerp(result, end, speed, dt);
            Assert.Equal(end, result);
        }

        [Fact]
        public void Slerp_InterpolatesOnSphere()
        {
            var start = new Vector3d(1, 0, 0);
            var end = new Vector3d(0, 1, 0);

            // Slerp at t = 0 (should return the start vector)
            var result = Vector3d.Slerp(start, end, Fixed64.Zero);
            Assert.Equal(start, result);

            // Slerp at t = 1 (should return the end vector)
            result = Vector3d.Slerp(start, end, Fixed64.One);
            Assert.Equal(end, result);

            // Slerp at t = 0.5 (should return a midpoint vector on the unit circle)
            result = Vector3d.Slerp(start, end, new Fixed64(0.5));
            // Since the midpoint on a unit circle between (1, 0, 0) and (0, 1, 0) should be (0.707, 0.707, 0)
            var expected = new Vector3d(new Fixed64(0.70710678118), new Fixed64(0.70710678118), Fixed64.Zero);
            Assert.True(Vector3d.AreAlmostParallel(result, expected, new Fixed64(0.999)));
        }

        [Fact]
        public void StaticMagnitude_CalculatesCorrectly()
        {
            var vector = new Vector3d(3, 4, 0); // 3-4-5 triangle
            var result = Vector3d.GetMagnitude(vector);
            Assert.Equal(new Fixed64(5), result);
        }

        [Fact]
        public void Dot_CalculatesCorrectly()
        {
            var v1 = new Vector3d(1, 0, 0);
            var v2 = new Vector3d(0, 1, 0);

            // Perpendicular vectors should have dot product of 0
            var result = Vector3d.Dot(v1, v2);
            Assert.Equal(Fixed64.Zero, result);

            // Parallel vectors (should be 1 * 1)
            v2 = new Vector3d(1, 0, 0);
            result = Vector3d.Dot(v1, v2);
            Assert.Equal(Fixed64.One, result);
        }

        [Fact]
        public void StaticDistance_CalculatesCorrectly()
        {
            var v1 = new Vector3d(0, 0, 0);
            var v2 = new Vector3d(3, 4, 0); // Distance should be 5 (3-4-5 triangle)
            var result = Vector3d.Distance(v1, v2);
            Assert.Equal(new Fixed64(5), result);
        }

        [Fact]
        public void StaticSqrDistance_CalculatesCorrectly()
        {
            var v1 = new Vector3d(0, 0, 0);
            var v2 = new Vector3d(3, 4, 0); // Squared distance should be 25
            var result = Vector3d.SqrDistance(v1, v2);
            Assert.Equal(new Fixed64(25), result);
        }

        [Fact]
        public void Project_CalculatesCorrectly()
        {
            var vector = new Vector3d(2, 2, 0);
            var onNormal = new Vector3d(1, 0, 0); // Project onto X-axis
            var result = Vector3d.Project(vector, onNormal);
            var expected = new Vector3d(2, 0, 0);
            Assert.Equal(expected, result);
        }

        [Fact]
        public void Clamp_CalculatesCorrectly()
        {
            var value = new Vector3d(2, 2, 2);
            var min = new Vector3d(1, 1, 1);
            var max = new Vector3d(3, 3, 3);
            var result = Vector3d.Clamp(value, min, max);
            Assert.Equal(value, result);

            // Test clamping below the min
            value = new Vector3d(0, 0, 0);
            result = Vector3d.Clamp(value, min, max);
            Assert.Equal(min, result);

            // Test clamping above the max
            value = new Vector3d(4, 4, 4);
            result = Vector3d.Clamp(value, min, max);
            Assert.Equal(max, result);
        }

        [Fact]
        public void Angle_CalculatesCorrectly()
        {
            var v1 = new Vector3d(1, 0, 0);
            var v2 = new Vector3d(0, 1, 0);

            // The angle between (1, 0, 0) and (0, 1, 0) should be 90 degrees
            var result = Vector3d.Angle(v1, v2);
            var expected = FixedMath.RadToDeg(FixedMath.PiOver2);
            FixedMathTestHelper.AssertWithinRelativeTolerance(expected, result);
        }

        [Fact]
        public void Midpoint_CalculatesCorrectly()
        {
            var v1 = new Vector3d(1, 1, 1);
            var v2 = new Vector3d(3, 3, 3);
            var result = Vector3d.Midpoint(v1, v2);
            Assert.Equal(new Vector3d(2, 2, 2), result);
        }

        [Fact]
        public void Midpoint_WithNegativeValues_CalculatesCorrectly()
        {
            var v1 = new Vector3d(-1, -1, -1);
            var v2 = new Vector3d(1, 1, 1);
            var result = Vector3d.Midpoint(v1, v2);
            Assert.Equal(new Vector3d(0, 0, 0), result);
        }

        [Fact]
        public void ClosestPointsOnTwoLines_NonParallelSegments()
        {
            var line1Start = new Vector3d(0, 0, 0);
            var line1End = new Vector3d(1, 1, 0);
            var line2Start = new Vector3d(1, 0, 0);
            var line2End = new Vector3d(0, 1, 0);

            var (pointOnLine1, pointOnLine2) = Vector3d.ClosestPointsOnTwoLines(line1Start, line1End, line2Start, line2End);

            // These points should be the same since the lines intersect at (0.5, 0.5, 0)
            var expectedIntersection = new Vector3d(0.5, 0.5, 0);
            Assert.Equal(expectedIntersection, pointOnLine1);
            Assert.Equal(expectedIntersection, pointOnLine2);
        }


        [Fact]
        public void ClosestPointsOnTwoLines_ParallelSegments()
        {
            var line1Start = new Vector3d(0, 0, 0);
            var line1End = new Vector3d(1, 1, 0);
            var line2Start = new Vector3d(0, 0, 1);
            var line2End = new Vector3d(1, 1, 1);
            var (pointOnLine1, pointOnLine2) = Vector3d.ClosestPointsOnTwoLines(line1Start, line1End, line2Start, line2End);

            // Points on the same relative positions on the parallel lines
            Assert.Equal(new Vector3d(0, 0, 0), pointOnLine1);
            Assert.Equal(new Vector3d(0, 0, 1), pointOnLine2);
        }

        [Fact]
        public void ClosestPointOnLineSegment_InsideSegment()
        {
            var a = new Vector3d(0, 0, 0);
            var b = new Vector3d(2, 0, 0);
            var p = new Vector3d(1, 0, 1); // Above the midpoint
            var result = Vector3d.ClosestPointOnLineSegment(a, b, p);
            Assert.Equal(new Vector3d(1, 0, 0), result); // Should project to (1, 0, 0)
        }

        [Fact]
        public void ClosestPointOnLineSegment_OutsideSegment()
        {
            var a = new Vector3d(0, 0, 0);
            var b = new Vector3d(2, 0, 0);
            var p = new Vector3d(3, 0, 0); // Outside segment
            var result = Vector3d.ClosestPointOnLineSegment(a, b, p);
            Assert.Equal(b, result); // Should clamp to the endpoint b
        }

        [Fact]
        public void Max_CalculatesCorrectly()
        {
            var v1 = new Vector3d(1, 5, 3);
            var v2 = new Vector3d(2, 4, 3);
            var result = Vector3d.Max(v1, v2);
            Assert.Equal(new Vector3d(2, 5, 3), result);
        }

        [Fact]
        public void Min_CalculatesCorrectly()
        {
            var v1 = new Vector3d(1, 5, 3);
            var v2 = new Vector3d(2, 4, 3);
            var result = Vector3d.Min(v1, v2);
            Assert.Equal(new Vector3d(1, 4, 3), result);
        }

        #endregion

        #region Test: Extensions

        [Fact]
        public void V3ClampOneInPlace_ClampsCorrectly()
        {
            var vector = new Vector3d(2, -3, 0.5);
            var result = vector.ClampOneInPlace();

            Assert.Equal(new Vector3d(1, -1, 0.5), result); // Clamps x and y, z stays the same
        }

        [Fact]
        public void V3ToDegrees_ConvertsCorrectly_WithTolerance()
        {
            var radians = new Vector3d(FixedMath.PiOver2, FixedMath.PI, Fixed64.Zero);
            var result = radians.ToDegrees();
            var expected = new Vector3d(90, 180, 0);

            // Use FuzzyEqual with a small allowed difference
            Assert.True(result.FuzzyEqualAbsolute(expected, new Fixed64(0.0001)));
        }

        [Fact]
        public void V3ToRadians_ConvertsCorrectly_WithTolerance()
        {
            var degrees = new Vector3d(90, 180, 0);
            var result = degrees.ToRadians();
            var expected = new Vector3d(FixedMath.PiOver2, FixedMath.PI, Fixed64.Zero);

            // Use FuzzyEqual with a small allowed difference
            Assert.True(result.FuzzyEqualAbsolute(expected, new Fixed64(0.0001)));
        }

        [Fact]
        public void V3FuzzyEqualAbsolute_ComparesCorrectly_WithAllowedDifference()
        {
            var vector1 = new Vector3d(2, 2, 2);
            var vector2 = new Vector3d(2.1, 2.1, 2.1);
            var allowedDifference = new Fixed64(0.15);

            // Should be approximately equal given the allowed difference of 0.15
            Assert.True(vector1.FuzzyEqualAbsolute(vector2, allowedDifference));
        }

        [Fact]
        public void V3FuzzyEqual_ComparesCorrectly_WithPercentage()
        {
            var vector1 = new Vector3d(100, 100, 100);
            var vector2 = new Vector3d(101, 101, 101);
            var percentage = new Fixed64(0.02); // Allow a 2% difference

            // Should be approximately equal within the percentage
            Assert.True(vector1.FuzzyEqual(vector2, percentage));
        }

        [Fact]
        public void V3CheckDistance_VerifiesDistanceCorrectly()
        {
            var vector1 = new Vector3d(0, 0, 0);
            var vector2 = new Vector3d(3, 4, 0); // Distance is 5 (3-4-5 triangle)
            var factor = new Fixed64(5);

            Assert.True(vector1.CheckDistance(vector2, factor)); // Distance is 5, so should return true
        }

        [Fact]
        public void V3SqrDistance_CalculatesCorrectly()
        {
            var vector1 = new Vector3d(0, 0, 0);
            var vector2 = new Vector3d(3, 4, 0); // Squared distance should be 25
            var result = Vector3d.SqrDistance(vector1, vector2);

            Assert.Equal(new Fixed64(25), result); // 3^2 + 4^2 = 25
        }

        [Fact]
        public void V3RotateVector_RotatesCorrectly_WithFuzzyEqual()
        {
            var vector = new Vector3d(1, 0, 0);
            var position = new Vector3d(0, 0, 0);
            var quaternion = FixedQuaternion.FromEulerAngles(new Fixed64(0), new Fixed64(0), FixedMath.PiOver2); // 90° rotation around Z-axis

            var result = vector.Rotate(position, quaternion);
            Assert.True(result.FuzzyEqual(new Vector3d(0, 1, 0), new Fixed64(0.0001))); // Allow small error tolerance
        }

        [Fact]
        public void V3InverseRotateVector_AppliesInverseRotationCorrectly_WithFuzzyEqual()
        {
            var vector = new Vector3d(0, 1, 0);
            var position = new Vector3d(0, 0, 0);
            var quaternion = FixedQuaternion.FromEulerAngles(new Fixed64(0), new Fixed64(0), FixedMath.PiOver2); // 90° rotation around Z-axis

            var result = vector.InverseRotate(position, quaternion);
            Assert.True(result.FuzzyEqual(new Vector3d(1, 0, 0), new Fixed64(0.0001))); // Allow small error tolerance
        }

        #endregion
    }
}
