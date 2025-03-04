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
    public class FixedQuanternionTests
    {
        #region Test: Initialization and Identity

        [Fact]
        public void FixedQuaternion_InitializesCorrectly()
        {
            var quaternion = new FixedQuaternion(Fixed64.One, Fixed64.Zero, Fixed64.Zero, Fixed64.One);
            Assert.Equal(Fixed64.One, quaternion.x);
            Assert.Equal(Fixed64.Zero, quaternion.y);
            Assert.Equal(Fixed64.Zero, quaternion.z);
            Assert.Equal(Fixed64.One, quaternion.w);
        }

        [Fact]
        public void FixedQuaternion_Identity_IsCorrect()
        {
            var identity = FixedQuaternion.Identity;
            Assert.Equal(Fixed64.Zero, identity.x);
            Assert.Equal(Fixed64.Zero, identity.y);
            Assert.Equal(Fixed64.Zero, identity.z);
            Assert.Equal(Fixed64.One, identity.w);
        }

        #endregion

        #region Test: Normalization

        [Fact]
        public void FixedQuaternion_Normalize_WorksCorrectly()
        {
            var quaternion = new FixedQuaternion(new Fixed64(1), new Fixed64(1), new Fixed64(1), new Fixed64(1));
            quaternion.Normalize();

            var magnitudeSqr = quaternion.x * quaternion.x + quaternion.y * quaternion.y + quaternion.z * quaternion.z + quaternion.w * quaternion.w;
            Assert.Equal(Fixed64.One, magnitudeSqr, new Fixed64(0.0001)); // Ensure magnitude squared is 1
        }

        [Fact]
        public void FixedQuaternion_NormalizeStatic_WorksCorrectly()
        {
            var quaternion = new FixedQuaternion(new Fixed64(2), new Fixed64(2), new Fixed64(2), new Fixed64(2));
            var normalized = FixedQuaternion.GetNormalized(quaternion);

            var magnitudeSqr = normalized.x * normalized.x + normalized.y * normalized.y + normalized.z * normalized.z + normalized.w * normalized.w;
            Assert.Equal(Fixed64.One, magnitudeSqr, new Fixed64(0.0001));
        }

        #endregion

        #region Test: Conjugate and Inverse

        [Fact]
        public void FixedQuaternion_Conjugate_WorksCorrectly()
        {
            var quaternion = new FixedQuaternion(new Fixed64(1), new Fixed64(2), new Fixed64(3), new Fixed64(4));
            var conjugate = quaternion.Conjugate();

            Assert.Equal(new Fixed64(-1), conjugate.x);
            Assert.Equal(new Fixed64(-2), conjugate.y);
            Assert.Equal(new Fixed64(-3), conjugate.z);
            Assert.Equal(new Fixed64(4), conjugate.w);
        }

        [Fact]
        public void FixedQuaternion_Inverse_WorksCorrectly()
        {
            var quaternion = new FixedQuaternion(new Fixed64(1), new Fixed64(2), new Fixed64(3), new Fixed64(4));
            var inverse = quaternion.Inverse();

            var multiplied = quaternion * inverse;
            Assert.True(multiplied.FuzzyEqual(FixedQuaternion.Identity)); // quaternion * inverse = identity
        }

        #endregion

        #region Test: Conversion

        [Fact]
        public void FixedQuaternion_FromEulerAnglesInDegrees_WorksCorrectly()
        {
            var quaternion = FixedQuaternion.FromEulerAnglesInDegrees(new Fixed64(90), new Fixed64(0), new Fixed64(0));
            var eulerAngles = quaternion.EulerAngles;

            Assert.True(eulerAngles.FuzzyEqual(new Vector3d(90, 0, 0), new Fixed64(0.0001))); // Expected result: (90, 0, 0) degrees
        }

        [Fact]
        public void FixedQuaternion_FromEulerAnglesInDegrees_MatchesExpected()
        {
            var quaternion = FixedQuaternion.FromEulerAnglesInDegrees(new Fixed64(0), new Fixed64(0), new Fixed64(90));
            var expected = new FixedQuaternion(Fixed64.Zero, Fixed64.Zero, FixedMath.Sqrt(Fixed64.Half), FixedMath.Sqrt(Fixed64.Half));

            Assert.True(quaternion.FuzzyEqual(expected, new Fixed64(0.0001)),
                        $"Expected: {expected}, Actual: {quaternion}");
        }

        [Fact]
        public void FixedQuaternion_ToEulerAngles_WorksCorrectly()
        {
            var quaternion = FixedQuaternion.FromEulerAnglesInDegrees(new Fixed64(90), new Fixed64(0), new Fixed64(0));
            var eulerAngles = quaternion.ToEulerAngles();

            Assert.True(eulerAngles.FuzzyEqual(new Vector3d(90, 0, 0), new Fixed64(0.0001)));
        }

        [Fact]
        public void FixedQuaternion_FromMatrix_WorksCorrectly()
        {
            var matrix = new Fixed3x3(
                Fixed64.One, Fixed64.Zero, Fixed64.Zero,
                Fixed64.Zero, Fixed64.One, Fixed64.Zero,
                Fixed64.Zero, Fixed64.Zero, Fixed64.One
            );

            var result = FixedQuaternion.FromMatrix(matrix);
            Assert.True(result.FuzzyEqual(FixedQuaternion.Identity), $"FromMatrix returned {result}, expected Identity.");
        }

        [Fact]
        public void FixedQuaternion_FromDirection_WorksCorrectly()
        {
            var direction = Vector3d.Right; // X-axis direction
            var result = FixedQuaternion.FromDirection(direction);

            // Expect a 90-degree rotation around the Y-axis
            var expected = FixedQuaternion.FromAxisAngle(Vector3d.Up, FixedMath.PiOver2);

            Assert.True(result.FuzzyEqual(expected), $"FromDirection returned {result}, expected {expected}.");
        }

        [Fact]
        public void FixedQuaternion_FromAxisAngle_WorksCorrectly()
        {
            var axis = Vector3d.Up;
            var angle = FixedMath.PiOver2;  // 90 degrees

            var result = FixedQuaternion.FromAxisAngle(axis, angle);
            var expected = new FixedQuaternion(Fixed64.Zero, FixedMath.Sin(FixedMath.PiOver4), Fixed64.Zero, FixedMath.Cos(FixedMath.PiOver4));

            Assert.True(result.FuzzyEqual(expected), $"FromAxisAngle returned {result}, expected {expected}.");

            axis = Vector3d.Forward;  

            result = FixedQuaternion.FromAxisAngle(axis, angle);
            expected = new FixedQuaternion(Fixed64.Zero, Fixed64.Zero, FixedMath.Sin(FixedMath.PiOver4), FixedMath.Cos(FixedMath.PiOver4));

            Assert.True(result.FuzzyEqual(expected), $"FromAxisAngle returned {result}, expected {expected}.");

            axis = Vector3d.Right;

            result = FixedQuaternion.FromAxisAngle(axis, angle);
            expected = new FixedQuaternion(FixedMath.Sin(FixedMath.PiOver4), Fixed64.Zero, Fixed64.Zero, FixedMath.Cos(FixedMath.PiOver4));

            Assert.True(result.FuzzyEqual(expected), $"FromAxisAngle returned {result}, expected {expected}.");

        }

        [Fact]
        public void FixedQuaternion_ToDirection_WorksCorrectly()
        {
            var quaternion = FixedQuaternion.Identity;
            var result = quaternion.ToDirection();

            var expected = new Vector3d(0, 0, 1);  // Default forward direction

            Assert.True(result.FuzzyEqual(expected), $"ToDirection returned {result}, expected {expected}.");
        }

        [Fact]
        public void FixedQuaternion_ToMatrix_WorksCorrectly()
        {
            var quaternion = FixedQuaternion.Identity;
            var result = quaternion.ToMatrix3x3();

            var expected = new Fixed3x3(
                Fixed64.One, Fixed64.Zero, Fixed64.Zero,
                Fixed64.Zero, Fixed64.One, Fixed64.Zero,
                Fixed64.Zero, Fixed64.Zero, Fixed64.One
            );

            Assert.True(result == expected, $"ToMatrix returned {result}, expected Identity matrix.");
        }

        [Fact]
        public void FixedQuaternion_ToAngularVelocity_WorksCorrectly()
        {
            var prevRotation = FixedQuaternion.Identity;
            var currentRotation = FixedQuaternion.FromAxisAngle(Vector3d.Up, FixedMath.PiOver4); // Rotated 45 degrees around Y-axis
            var deltaTime = new Fixed64(2); // Assume 2 seconds elapsed

            var angularVelocity = FixedQuaternion.ToAngularVelocity(currentRotation, prevRotation, deltaTime);

            var expected = new Vector3d(Fixed64.Zero, FixedMath.PiOver4 / deltaTime, Fixed64.Zero); // Expect ω = θ / dt
            Assert.True(angularVelocity.FuzzyEqual(expected, new Fixed64(0.0001)),
                $"ToAngularVelocity returned {angularVelocity}, expected {expected}");
        }

        [Fact]
        public void FixedQuaternion_ToAngularVelocity_ZeroForNoRotation()
        {
            var prevRotation = FixedQuaternion.Identity;
            var currentRotation = FixedQuaternion.Identity;
            var deltaTime = Fixed64.One;

            var angularVelocity = FixedQuaternion.ToAngularVelocity(currentRotation, prevRotation, deltaTime);

            Assert.True(angularVelocity.FuzzyEqual(Vector3d.Zero),
                $"ToAngularVelocity should return zero for no rotation, but got {angularVelocity}");
        }

        [Fact]
        public void FixedQuanternion_Serialization_RoundTripMaintainsData()
        {
            var quaternion = FixedQuaternion.Identity;
            var sin = FixedMath.Sin(FixedMath.PiOver4);  // 45° rotation
            var cos = FixedMath.Cos(FixedMath.PiOver4);

            var originalRotation = quaternion.Rotated(sin, cos);

            // Serialize the FixedQuaternion object
#if NET48_OR_GREATER
            var formatter = new BinaryFormatter();
            using var stream = new MemoryStream();
            formatter.Serialize(stream, originalRotation);

            // Reset stream position and deserialize
            stream.Seek(0, SeekOrigin.Begin);
            var deserializedRotation = (FixedQuaternion)formatter.Deserialize(stream);
#endif

#if NET8_0_OR_GREATER
            var jsonOptions = new JsonSerializerOptions
            {
                DefaultIgnoreCondition = JsonIgnoreCondition.WhenWritingNull,
                ReferenceHandler = ReferenceHandler.IgnoreCycles,
                IncludeFields = true,
                IgnoreReadOnlyProperties = true
            };
            var json = JsonSerializer.SerializeToUtf8Bytes(originalRotation, jsonOptions);
            var deserializedRotation = JsonSerializer.Deserialize<FixedQuaternion>(json, jsonOptions);
#endif

            // Check that deserialized values match the original
            Assert.Equal(originalRotation, deserializedRotation);
        }

#endregion

        #region Test: Lerp and Slerp

        [Fact]
        public void FixedQuaternion_Lerp_WorksCorrectly()
        {
            var q1 = FixedQuaternion.FromEulerAnglesInDegrees(new Fixed64(0), new Fixed64(0), new Fixed64(0));
            var q2 = FixedQuaternion.FromEulerAnglesInDegrees(new Fixed64(90), new Fixed64(0), new Fixed64(0));

            var result = FixedQuaternion.Lerp(q1, q2, new Fixed64(0.5)); // Halfway between q1 and q2
            var expected = FixedQuaternion.FromEulerAnglesInDegrees(new Fixed64(45), new Fixed64(0), new Fixed64(0));

            Assert.True(result.FuzzyEqual(expected, new Fixed64(0.0001)));
        }

        [Fact]
        public void FixedQuaternion_Slerp_WorksCorrectly()
        {
            var q1 = FixedQuaternion.FromEulerAnglesInDegrees(new Fixed64(0), new Fixed64(0), new Fixed64(0));
            var q2 = FixedQuaternion.FromEulerAnglesInDegrees(new Fixed64(90), new Fixed64(0), new Fixed64(0));

            var result = FixedQuaternion.Slerp(q1, q2, new Fixed64(0.5)); // Halfway between q1 and q2
            var expected = FixedQuaternion.FromEulerAnglesInDegrees(new Fixed64(45), new Fixed64(0), new Fixed64(0));

            Assert.True(result.FuzzyEqual(expected, new Fixed64(0.0001)));
        }


        #endregion

        #region Test: Dot Product and Angle

        [Fact]
        public void FixedQuaternion_Dot_WorksCorrectly()
        {
            var q1 = FixedQuaternion.FromEulerAnglesInDegrees(new Fixed64(0), new Fixed64(0), new Fixed64(0));
            var q2 = FixedQuaternion.FromEulerAnglesInDegrees(new Fixed64(90), new Fixed64(0), new Fixed64(0));

            var dotProduct = FixedQuaternion.Dot(q1, q2);
            Assert.True(dotProduct > Fixed64.Zero); // Should be positive for non-opposite quaternions
        }

        [Fact]
        public void FixedQuaternion_AngleBetween_WorksCorrectly()
        {
            var q1 = FixedQuaternion.FromEulerAnglesInDegrees(new Fixed64(0), new Fixed64(0), new Fixed64(0));
            var q2 = FixedQuaternion.FromEulerAnglesInDegrees(new Fixed64(90), new Fixed64(0), new Fixed64(0));

            var angle = FixedQuaternion.Angle(q1, q2);
            FixedMathTestHelper.AssertWithinRelativeTolerance(new Fixed64(45), angle); // 45 degrees between these quaternions
        }

        [Fact]
        public void FixedQuaternion_AngleAxis_WorksCorrectly()
        {
            var angle = new Fixed64(90);  // 90 degrees
            var axis = Vector3d.Up;

            var result = FixedQuaternion.AngleAxis(angle, axis);
            var expected = FixedQuaternion.FromAxisAngle(axis, FixedMath.PiOver2);

            Assert.True(result.FuzzyEqual(expected), $"AngleAxis returned {result}, expected {expected}.");
        }

        #endregion

        #region Test: Rotation

        [Fact]
        public void FixedQuaternion_RotateVector_WorksCorrectly()
        {
            var quaternion = FixedQuaternion.FromEulerAnglesInDegrees(new Fixed64(0), new Fixed64(0), new Fixed64(90)); // 90° around Z-axis
            var vector = new Vector3d(1, 0, 0);

            var result = quaternion.Rotate(vector);
            Assert.True(result.FuzzyEqual(new Vector3d(0, 1, 0))); // Expect (0, 1, 0) after rotation
        }

        [Fact]
        public void FixedQuaternion_Rotated_WorksCorrectly()
        {
            var quaternion = FixedQuaternion.Identity;
            var sin = FixedMath.Sin(FixedMath.PiOver4);  // 45° rotation
            var cos = FixedMath.Cos(FixedMath.PiOver4);

            var result = quaternion.Rotated(sin, cos);
            var expected = FixedQuaternion.FromAxisAngle(Vector3d.Up, FixedMath.PiOver4);

            Assert.True(result.FuzzyEqual(expected), $"Rotated quaternion was {result}, expected {expected}.");
        }

        [Fact]
        public void FixedQuaternion_LookRotation_WorksCorrectly()
        {
            var forward = new Vector3d(0, 0, 1);
            var result = FixedQuaternion.LookRotation(forward);

            var expected = FixedQuaternion.Identity;  // No rotation needed along Z-axis

            Assert.True(result.FuzzyEqual(expected), $"Look rotation returned {result}, expected {expected}.");
        }

        [Fact]
        public void FixedQuaternion_QuaternionLog_WorksCorrectly()
        {
            var quaternion = FixedQuaternion.FromAxisAngle(Vector3d.Up, FixedMath.PiOver4); // 45-degree rotation around Y-axis
            var logResult = FixedQuaternion.QuaternionLog(quaternion);

            var expected = new Vector3d(Fixed64.Zero, FixedMath.PiOver4, Fixed64.Zero); // Expect log(q) = θ * axis
            Assert.True(logResult.FuzzyEqual(expected, new Fixed64(0.0001)),
                $"QuaternionLog returned {logResult}, expected {expected}");
        }

        [Fact]
        public void FixedQuaternion_QuaternionLog_ReturnsZeroForIdentity()
        {
            var identity = FixedQuaternion.Identity;
            var logResult = FixedQuaternion.QuaternionLog(identity);

            Assert.True(logResult.FuzzyEqual(Vector3d.Zero),
                $"QuaternionLog of Identity should be (0,0,0), but got {logResult}");
        }

        #endregion

        #region Test: Operators

        [Fact]
        public void FixedQuaternion_Multiplication_WorksCorrectly()
        {
            var q1 = FixedQuaternion.FromEulerAnglesInDegrees(new Fixed64(0), new Fixed64(0), new Fixed64(90));
            var q2 = FixedQuaternion.FromEulerAnglesInDegrees(new Fixed64(90), new Fixed64(0), new Fixed64(0));

            var result = q1 * q2;
            var expected = FixedQuaternion.FromEulerAnglesInDegrees(new Fixed64(90), new Fixed64(0), new Fixed64(90));

            Assert.True(result.FuzzyEqual(expected, new Fixed64(0.0001)));
        }

        #endregion
    }
}
