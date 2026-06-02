using MemoryPack;
using System;
using System.Text.Json;
using System.Text.Json.Serialization;
using Xunit;

namespace FixedMathSharp.Tests;

public class FixedQuaternionTests
{
    private static void AssertRepresentsSameRotation(FixedQuaternion actual, FixedQuaternion expected, Fixed64? tolerance = null)
    {
        Fixed64 limit = tolerance ?? Fixed64.FromFloatPoint(0.0001);
        Assert.True(
            actual.FuzzyEqual(expected, limit) || actual.FuzzyEqual(expected * -Fixed64.One, limit),
            $"Expected {actual} to represent the same rotation as {expected}.");
    }

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

    [Fact]
    public void FixedQuaternion_Indexer_GetSetAndInvalidIndexBehaveCorrectly()
    {
        var quaternion = new FixedQuaternion(Fixed64.One, Fixed64.Zero, new Fixed64(2), new Fixed64(3));

        Assert.Equal(Fixed64.One, quaternion[0]);
        Assert.Equal(Fixed64.Zero, quaternion[1]);
        Assert.Equal(new Fixed64(2), quaternion[2]);
        Assert.Equal(new Fixed64(3), quaternion[3]);

        quaternion[0] = new Fixed64(7);
        quaternion[1] = new Fixed64(8);
        quaternion[2] = new Fixed64(9);
        quaternion[3] = new Fixed64(10);

        Assert.Equal(new Fixed64(7), quaternion.x);
        Assert.Equal(new Fixed64(8), quaternion.y);
        Assert.Equal(new Fixed64(9), quaternion.z);
        Assert.Equal(new Fixed64(10), quaternion.w);
        Assert.Throws<IndexOutOfRangeException>(() => _ = quaternion[4]);
        Assert.Throws<IndexOutOfRangeException>(() => quaternion[-1] = Fixed64.Zero);
    }

    [Fact]
    public void FixedQuaternion_Set_UpdatesAllComponents()
    {
        var quaternion = FixedQuaternion.Zero;

        quaternion.Set(new Fixed64(1), new Fixed64(2), new Fixed64(3), new Fixed64(4));

        Assert.Equal(new FixedQuaternion(new Fixed64(1), new Fixed64(2), new Fixed64(3), new Fixed64(4)), quaternion);
    }

    #endregion

    #region Test: Normalization

    [Fact]
    public void FixedQuaternion_Normalize_WorksCorrectly()
    {
        var quaternion = new FixedQuaternion(new Fixed64(1), new Fixed64(1), new Fixed64(1), new Fixed64(1));
        quaternion.Normalize();

        var magnitudeSqr = quaternion.x * quaternion.x + quaternion.y * quaternion.y + quaternion.z * quaternion.z + quaternion.w * quaternion.w;
        Assert.Equal(Fixed64.One, magnitudeSqr, Fixed64.FromFloatPoint(0.0001)); // Ensure magnitude squared is 1
    }

    [Fact]
    public void FixedQuaternion_NormalizeStatic_WorksCorrectly()
    {
        var quaternion = new FixedQuaternion(new Fixed64(2), new Fixed64(2), new Fixed64(2), new Fixed64(2));
        var normalized = FixedQuaternion.GetNormalized(quaternion);

        var magnitudeSqr = normalized.x * normalized.x + normalized.y * normalized.y + normalized.z * normalized.z + normalized.w * normalized.w;
        Assert.Equal(Fixed64.One, magnitudeSqr, Fixed64.FromFloatPoint(0.0001));
    }

    [Fact]
    public void FixedQuaternion_NormalProperty_ReturnsNormalizedCopyWithoutMutatingSource()
    {
        var quaternion = new FixedQuaternion(new Fixed64(2), Fixed64.Zero, Fixed64.Zero, Fixed64.Zero);

        var normal = quaternion.Normal;

        Assert.True(normal.IsNormalized());
        Assert.Equal(new Fixed64(2), quaternion.x);
        Assert.Equal(Fixed64.Zero, quaternion.y);
        Assert.Equal(Fixed64.Zero, quaternion.z);
        Assert.Equal(Fixed64.Zero, quaternion.w);
    }

    [Fact]
    public void FixedQuaternion_IsNormalized_DetectsUnitAndNonUnitQuaternions()
    {
        Assert.True(FixedQuaternion.Identity.IsNormalized());
        Assert.False(new FixedQuaternion(new Fixed64(2), Fixed64.Zero, Fixed64.Zero, Fixed64.Zero).IsNormalized());
    }

    [Fact]
    public void FixedQuaternion_GetMagnitude_And_GetNormalized_HandleZeroQuaternion()
    {
        Assert.Equal(Fixed64.Zero, FixedQuaternion.GetMagnitude(FixedQuaternion.Zero));
        Assert.Equal(FixedQuaternion.Identity, FixedQuaternion.GetNormalized(FixedQuaternion.Zero));
    }

    [Fact]
    public void FixedQuaternion_MagnitudeProperties_ReturnExpectedValues()
    {
        var quaternion = new FixedQuaternion(Fixed64.One, new Fixed64(2), new Fixed64(2), new Fixed64(4));

        Assert.Equal(new Fixed64(25), quaternion.SqrMagnitude);
        Assert.Equal(new Fixed64(5), quaternion.Magnitude);
        Assert.Equal(FixedQuaternion.GetMagnitude(quaternion), quaternion.Magnitude);
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

    [Fact]
    public void FixedQuaternion_Inverse_HandlesIdentityAndZero()
    {
        Assert.Equal(FixedQuaternion.Identity, FixedQuaternion.Identity.Inverse());
        Assert.Equal(FixedQuaternion.Zero, FixedQuaternion.Zero.Inverse());
    }

    [Fact]
    public void FixedQuaternion_Divide_MultipliesByInverseDivisor()
    {
        var dividend = FixedQuaternion.FromAxisAngle(Vector3d.Up, Fixed64.HalfPi);
        var divisor = FixedQuaternion.FromAxisAngle(Vector3d.Up, Fixed64.PiOver4);

        var result = FixedQuaternion.Divide(dividend, divisor);

        AssertRepresentsSameRotation(result, FixedQuaternion.FromAxisAngle(Vector3d.Up, Fixed64.PiOver4));
    }

    [Fact]
    public void FixedQuaternion_Divide_ZeroDivisorThrows()
    {
        Assert.Throws<InvalidOperationException>(() => FixedQuaternion.Divide(FixedQuaternion.Identity, FixedQuaternion.Zero));
    }

    #endregion

    #region Test: Conversion

    [Fact]
    public void FixedQuaternion_FromEulerAnglesInDegrees_WorksCorrectly()
    {
        var quaternion = FixedQuaternion.FromEulerAnglesInDegrees(new Fixed64(90), new Fixed64(0), new Fixed64(0));
        var eulerAngles = quaternion.EulerAngles;

        Assert.True(eulerAngles.FuzzyEqual(new Vector3d(90, 0, 0), Fixed64.FromFloatPoint(0.0001)));
    }

    [Fact]
    public void FixedQuaternion_FromEulerAnglesInDegrees_MatchesExpected()
    {
        var quaternion = FixedQuaternion.FromEulerAnglesInDegrees(new Fixed64(0), new Fixed64(0), new Fixed64(90));
        var expected = new FixedQuaternion(Fixed64.Zero, Fixed64.Zero, FixedMath.Sqrt(Fixed64.Half), FixedMath.Sqrt(Fixed64.Half));

        Assert.True(quaternion.FuzzyEqual(expected, Fixed64.FromFloatPoint(0.0001)),
                    $"Expected: {expected}, Actual: {quaternion}");
    }

    [Fact]
    public void FixedQuaternion_ToEulerAngles_WorksCorrectly()
    {
        var quaternion = FixedQuaternion.FromEulerAnglesInDegrees(new Fixed64(90), new Fixed64(0), new Fixed64(0));
        var eulerAngles = quaternion.ToEulerAngles();

        Assert.True(eulerAngles.FuzzyEqual(new Vector3d(90, 0, 0), Fixed64.FromFloatPoint(0.0001)));
    }

    [Fact]
    public void FixedQuaternion_EulerAngles_SetterUpdatesQuaternion()
    {
        var quaternion = FixedQuaternion.Identity;

        quaternion.EulerAngles = new Vector3d(0, 90, 0);

        Assert.True(quaternion.FuzzyEqual(FixedQuaternion.FromEulerAnglesInDegrees(Fixed64.Zero, new Fixed64(90), Fixed64.Zero), Fixed64.FromFloatPoint(0.0001)));
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
    public void FixedQuaternion_FromMatrix4x4_WorksCorrectly()
    {
        var rotation = FixedQuaternion.FromAxisAngle(Vector3d.Up, Fixed64.HalfPi);
        var matrix = Fixed4x4.CreateTransform(Vector3d.Zero, rotation, Vector3d.One);

        var result = FixedQuaternion.FromMatrix(matrix);

        Assert.True(result.FuzzyEqual(rotation, Fixed64.FromFloatPoint(0.0001)));
    }

    [Theory]
    [InlineData(0)]
    [InlineData(1)]
    [InlineData(2)]
    public void FixedQuaternion_FromMatrix_HandlesNegativeTraceBranches(int branch)
    {
        FixedQuaternion expected = branch switch
        {
            0 => FixedQuaternion.FromAxisAngle(Vector3d.Right, Fixed64.Pi),
            1 => FixedQuaternion.FromAxisAngle(Vector3d.Up, Fixed64.Pi),
            _ => FixedQuaternion.FromAxisAngle(Vector3d.Forward, Fixed64.Pi),
        };

        FixedQuaternion result = FixedQuaternion.FromMatrix(expected.ToMatrix3x3());

        AssertRepresentsSameRotation(result, expected);
    }

    [Fact]
    public void FixedQuaternion_FromDirection_WorksCorrectly()
    {
        var direction = Vector3d.Right; // X-axis direction
        var result = FixedQuaternion.FromDirection(direction);

        // Expect a 90-degree rotation around the Y-axis
        var expected = FixedQuaternion.FromAxisAngle(Vector3d.Up, Fixed64.HalfPi);

        Assert.True(result.FuzzyEqual(expected), $"FromDirection returned {result}, expected {expected}.");
    }

    [Fact]
    public void FixedQuaternion_FromDirection_ForwardReturnsIdentity()
    {
        Assert.Equal(FixedQuaternion.Identity, FixedQuaternion.FromDirection(Vector3d.Forward));
    }

    [Fact]
    public void FixedQuaternion_FromAxisAngle_WorksCorrectly()
    {
        var axis = Vector3d.Up;
        var angle = Fixed64.HalfPi;  // 90 degrees

        var result = FixedQuaternion.FromAxisAngle(axis, angle);
        var expected = new FixedQuaternion(Fixed64.Zero, FixedMath.Sin(Fixed64.PiOver4), Fixed64.Zero, FixedMath.Cos(Fixed64.PiOver4));

        Assert.True(result.FuzzyEqual(expected), $"FromAxisAngle returned {result}, expected {expected}.");

        axis = Vector3d.Forward;

        result = FixedQuaternion.FromAxisAngle(axis, angle);
        expected = new FixedQuaternion(Fixed64.Zero, Fixed64.Zero, FixedMath.Sin(Fixed64.PiOver4), FixedMath.Cos(Fixed64.PiOver4));

        Assert.True(result.FuzzyEqual(expected), $"FromAxisAngle returned {result}, expected {expected}.");

        axis = Vector3d.Right;

        result = FixedQuaternion.FromAxisAngle(axis, angle);
        expected = new FixedQuaternion(FixedMath.Sin(Fixed64.PiOver4), Fixed64.Zero, Fixed64.Zero, FixedMath.Cos(Fixed64.PiOver4));

        Assert.True(result.FuzzyEqual(expected), $"FromAxisAngle returned {result}, expected {expected}.");

    }

    [Fact]
    public void FixedQuaternion_FromAxisAngle_NormalizesAxisInput()
    {
        var result = FixedQuaternion.FromAxisAngle(new Vector3d(0, 2, 0), Fixed64.HalfPi);
        var expected = FixedQuaternion.FromAxisAngle(Vector3d.Up, Fixed64.HalfPi);

        Assert.True(result.FuzzyEqual(expected, Fixed64.FromFloatPoint(0.0001)));
    }

    [Fact]
    public void FixedQuaternion_FromAxisAngle_ThrowsWhenAngleIsOutOfRange()
    {
        Assert.Throws<ArgumentOutOfRangeException>(() => FixedQuaternion.FromAxisAngle(Vector3d.Up, Fixed64.Pi + Fixed64.One));
    }

    [Theory]
    [InlineData("pitch", 4, 0, 0)]
    [InlineData("yaw", 0, 4, 0)]
    [InlineData("roll", 0, 0, 4)]
    public void FixedQuaternion_FromEulerAngles_ThrowsWhenComponentIsOutOfRange(string parameterName, int pitch, int yaw, int roll)
    {
        var exception = Assert.Throws<ArgumentOutOfRangeException>(() => FixedQuaternion.FromEulerAngles(new Fixed64(pitch), new Fixed64(yaw), new Fixed64(roll)));

        Assert.Equal(parameterName, exception.ParamName);
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
    public void FixedQuaternion_ToEulerAngles_HandlesGimbalLockPitch()
    {
        var quaternion = FixedQuaternion.FromAxisAngle(Vector3d.Up, Fixed64.HalfPi);
        var eulerAngles = quaternion.ToEulerAngles();

        Assert.True(eulerAngles.FuzzyEqual(new Vector3d(0, 90, 0)));
    }

    [Fact]
    public void FixedQuaternion_FromAxisAngle_Equals_FromEulerAngles()
    {
        var quaternion = FixedQuaternion.FromAxisAngle(Vector3d.Up, Fixed64.HalfPi);
        var expectedQuaternion = FixedQuaternion.FromEulerAnglesInDegrees(new Fixed64(0), new Fixed64(90), new Fixed64(0));

        Assert.Equal(Fixed64.HalfPi, FixedMath.DegToRad(new Fixed64(90)));

        Assert.True(quaternion.Equals(expectedQuaternion));
    }

    [Fact]
    public void FixedQuaternion_ToAngularVelocity_WorksCorrectly()
    {
        var prevRotation = FixedQuaternion.Identity;
        var currentRotation = FixedQuaternion.FromAxisAngle(Vector3d.Up, Fixed64.PiOver4); // Rotated 45 degrees around Y-axis
        var deltaTime = new Fixed64(2); // Assume 2 seconds elapsed

        var angularVelocity = FixedQuaternion.ToAngularVelocity(currentRotation, prevRotation, deltaTime);

        var expected = new Vector3d(Fixed64.Zero, Fixed64.PiOver4 / deltaTime, Fixed64.Zero); // Expect ω = θ / dt
        Assert.True(angularVelocity.FuzzyEqual(expected, Fixed64.FromFloatPoint(0.0001)),
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

    #endregion

    #region Test: Lerp and Slerp

    [Fact]
    public void FixedQuaternion_Lerp_WorksCorrectly()
    {
        var q1 = FixedQuaternion.FromEulerAnglesInDegrees(new Fixed64(0), new Fixed64(0), new Fixed64(0));
        var q2 = FixedQuaternion.FromEulerAnglesInDegrees(new Fixed64(90), new Fixed64(0), new Fixed64(0));

        var result = FixedQuaternion.Lerp(q1, q2, Fixed64.FromFloatPoint(0.5)); // Halfway between q1 and q2
        var expected = FixedQuaternion.FromEulerAnglesInDegrees(new Fixed64(45), new Fixed64(0), new Fixed64(0));

        Assert.True(result.FuzzyEqual(expected, Fixed64.FromFloatPoint(0.0001)));
    }

    [Fact]
    public void FixedQuaternion_Lerp_ClampsInterpolationFactor()
    {
        var q1 = FixedQuaternion.Identity;
        var q2 = FixedQuaternion.FromEulerAnglesInDegrees(new Fixed64(90), new Fixed64(0), new Fixed64(0));

        var result = FixedQuaternion.Lerp(q1, q2, new Fixed64(2));

        Assert.True(result.FuzzyEqual(q2, Fixed64.FromFloatPoint(0.0001)));
    }

    [Fact]
    public void FixedQuaternion_Lerp_UsesShortestPathForNegatedQuaternion()
    {
        var rotation = FixedQuaternion.FromAxisAngle(Vector3d.Right, Fixed64.HalfPi);
        var negatedRotation = rotation * -Fixed64.One;

        var result = FixedQuaternion.Lerp(rotation, negatedRotation, Fixed64.Half);

        AssertRepresentsSameRotation(result, rotation);
    }

    [Fact]
    public void FixedQuaternion_Slerp_WorksCorrectly()
    {
        var q1 = FixedQuaternion.FromEulerAnglesInDegrees(new Fixed64(0), new Fixed64(0), new Fixed64(0));
        var q2 = FixedQuaternion.FromEulerAnglesInDegrees(new Fixed64(90), new Fixed64(0), new Fixed64(0));

        var result = FixedQuaternion.Slerp(q1, q2, Fixed64.FromFloatPoint(0.5)); // Halfway between q1 and q2
        var expected = FixedQuaternion.FromEulerAnglesInDegrees(new Fixed64(45), new Fixed64(0), new Fixed64(0));

        Assert.True(result.FuzzyEqual(expected, Fixed64.FromFloatPoint(0.0001)));
    }

    [Fact]
    public void FixedQuaternion_Slerp_UsesShortestPathForNegatedQuaternion()
    {
        var result = FixedQuaternion.Slerp(FixedQuaternion.Identity, FixedQuaternion.Identity * -Fixed64.One, Fixed64.Half);

        Assert.True(result.FuzzyEqual(FixedQuaternion.Identity, Fixed64.FromFloatPoint(0.0001)));
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
        var expected = FixedQuaternion.FromAxisAngle(axis, Fixed64.HalfPi);

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
        var sin = FixedMath.Sin(Fixed64.PiOver4);  // 45° rotation
        var cos = FixedMath.Cos(Fixed64.PiOver4);

        var result = quaternion.Rotated(sin, cos);
        var expected = FixedQuaternion.FromAxisAngle(Vector3d.Up, Fixed64.PiOver4);

        Assert.True(result.FuzzyEqual(expected), $"Rotated quaternion was {result}, expected {expected}.");
    }

    [Fact]
    public void FixedQuaternion_Rotated_UsesCustomAxis()
    {
        var quaternion = FixedQuaternion.Identity;
        var sin = FixedMath.Sin(Fixed64.PiOver4);
        var cos = FixedMath.Cos(Fixed64.PiOver4);

        var result = quaternion.Rotated(sin, cos, Vector3d.Right);

        Assert.True(result.FuzzyEqual(FixedQuaternion.FromAxisAngle(Vector3d.Right, Fixed64.PiOver4), Fixed64.FromFloatPoint(0.0001)));
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
    public void FixedQuaternion_LookRotation_WithCustomUpVector_WorksCorrectly()
    {
        var result = FixedQuaternion.LookRotation(Vector3d.Right, Vector3d.Up);

        Assert.True(result.FuzzyEqual(FixedQuaternion.FromAxisAngle(Vector3d.Up, Fixed64.HalfPi), Fixed64.FromFloatPoint(0.0001)));
    }

    [Fact]
    public void FixedQuaternion_QuaternionLog_WorksCorrectly()
    {
        var quaternion = FixedQuaternion.FromAxisAngle(Vector3d.Up, Fixed64.PiOver4); // 45-degree rotation around Y-axis
        var logResult = FixedQuaternion.QuaternionLog(quaternion);

        var expected = new Vector3d(Fixed64.Zero, Fixed64.PiOver4, Fixed64.Zero); // Expect log(q) = θ * axis
        Assert.True(logResult.FuzzyEqual(expected, Fixed64.FromFloatPoint(0.0001)),
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

    [Fact]
    public void FixedQuaternion_QuaternionLog_ReturnsZeroForVerySmallRotation()
    {
        var tinyRotation = new FixedQuaternion(Fixed64.FromRaw(1), Fixed64.Zero, Fixed64.Zero, Fixed64.One).Normalize();

        Assert.True(FixedQuaternion.QuaternionLog(tinyRotation).FuzzyEqual(Vector3d.Zero));
    }

    #endregion

    #region Test: Operators

    [Fact]
    public void FixedQuaternion_Multiplication_WorksCorrectly()
    {
        var q1 = FixedQuaternion.FromEulerAnglesInDegrees(new Fixed64(0), new Fixed64(0), new Fixed64(90));
        var q2 = FixedQuaternion.FromEulerAnglesInDegrees(new Fixed64(90), new Fixed64(0), new Fixed64(0));

        var result = q2 * q1; // quaternion multiplication is non-commutative
        var expected = FixedQuaternion.FromEulerAnglesInDegrees(new Fixed64(90), new Fixed64(0), new Fixed64(90));

        Assert.True(result.FuzzyEqual(expected, Fixed64.FromFloatPoint(0.0001)));
    }

    [Fact]
    public void FixedQuaternion_ScalarArithmeticEqualityAndFormatting_WorkCorrectly()
    {
        var quaternion = new FixedQuaternion(new Fixed64(1), new Fixed64(2), new Fixed64(3), new Fixed64(4));
        var same = new FixedQuaternion(new Fixed64(1), new Fixed64(2), new Fixed64(3), new Fixed64(4));

        Assert.Equal(new FixedQuaternion(new Fixed64(2), new Fixed64(4), new Fixed64(6), new Fixed64(8)), quaternion * new Fixed64(2));
        Assert.Equal(new FixedQuaternion(new Fixed64(2), new Fixed64(4), new Fixed64(6), new Fixed64(8)), new Fixed64(2) * quaternion);
        Assert.Equal(new FixedQuaternion(Fixed64.FromFloatPoint(0.5), Fixed64.One, Fixed64.FromFloatPoint(1.5), new Fixed64(2)), quaternion / new Fixed64(2));
        Assert.Equal(new FixedQuaternion(new Fixed64(1), new Fixed64(2), new Fixed64(3), new Fixed64(5)), quaternion + FixedQuaternion.Identity);
        Assert.True(quaternion == same);
        Assert.False(quaternion != same);
        Assert.False(quaternion.Equals("not-a-quaternion"));
        Assert.Equal(quaternion.GetHashCode(), same.GetHashCode());
        Assert.Equal("(1, 2, 3, 4)", quaternion.ToString());
    }

    [Fact]
    public void FixedQuaternion_SubtractionNegationAndDeconstruction_WorkCorrectly()
    {
        var quaternion = new FixedQuaternion(new Fixed64(1), new Fixed64(2), new Fixed64(3), new Fixed64(4));
        var other = new FixedQuaternion(Fixed64.One, Fixed64.One, Fixed64.One, Fixed64.One);

        Assert.Equal(new FixedQuaternion(Fixed64.Zero, Fixed64.One, new Fixed64(2), new Fixed64(3)), quaternion - other);
        Assert.Equal(new FixedQuaternion(new Fixed64(-1), new Fixed64(-2), new Fixed64(-3), new Fixed64(-4)), -quaternion);

        quaternion.Deconstruct(out Fixed64 x, out Fixed64 y, out Fixed64 z, out Fixed64 w);

        Assert.Equal(new Fixed64(1), x);
        Assert.Equal(new Fixed64(2), y);
        Assert.Equal(new Fixed64(3), z);
        Assert.Equal(new Fixed64(4), w);
    }

    [Fact]
    public void FixedQuaternion_ExtensionMethods_DelegateAndCompareCorrectly()
    {
        var currentRotation = FixedQuaternion.FromAxisAngle(Vector3d.Up, Fixed64.PiOver4);
        var previousRotation = FixedQuaternion.Identity;
        var nearRotation = new FixedQuaternion(Fixed64.FromRaw(1), Fixed64.Zero, Fixed64.Zero, Fixed64.One);

        var angularVelocity = currentRotation.ToAngularVelocity(previousRotation, new Fixed64(2));

        Assert.True(angularVelocity.FuzzyEqual(FixedQuaternion.ToAngularVelocity(currentRotation, previousRotation, new Fixed64(2)), Fixed64.FromFloatPoint(0.0001)));
        Assert.True(FixedQuaternion.Identity.FuzzyEqualAbsolute(nearRotation, Fixed64.FromRaw(1)));
        Assert.True(FixedQuaternion.Identity.FuzzyEqual(nearRotation, Fixed64.FromFloatPoint(0.01)));
        Assert.False(FixedQuaternion.Identity.FuzzyEqualAbsolute(FixedQuaternion.Zero, Fixed64.FromFloatPoint(0.1)));
        Assert.False(FixedQuaternion.Identity.FuzzyEqual(FixedQuaternion.Zero, Fixed64.FromFloatPoint(0.1)));
    }

    [Fact]
    public void FixedQuaternion_FromAxisAngleAndEulerAngles_RejectAnglesOutsidePiRange()
    {
        Fixed64 tooHigh = Fixed64.Pi + Fixed64.Epsilon;
        Fixed64 tooLow = -Fixed64.Pi - Fixed64.Epsilon;

        Assert.Throws<ArgumentOutOfRangeException>(() => FixedQuaternion.FromAxisAngle(Vector3d.Up, tooHigh));
        Assert.Throws<ArgumentOutOfRangeException>(() => FixedQuaternion.FromAxisAngle(Vector3d.Up, tooLow));
        Assert.Throws<ArgumentOutOfRangeException>(() => FixedQuaternion.FromEulerAngles(tooHigh, Fixed64.Zero, Fixed64.Zero));
        Assert.Throws<ArgumentOutOfRangeException>(() => FixedQuaternion.FromEulerAngles(tooLow, Fixed64.Zero, Fixed64.Zero));
        Assert.Throws<ArgumentOutOfRangeException>(() => FixedQuaternion.FromEulerAngles(Fixed64.Zero, tooHigh, Fixed64.Zero));
        Assert.Throws<ArgumentOutOfRangeException>(() => FixedQuaternion.FromEulerAngles(Fixed64.Zero, tooLow, Fixed64.Zero));
        Assert.Throws<ArgumentOutOfRangeException>(() => FixedQuaternion.FromEulerAngles(Fixed64.Zero, Fixed64.Zero, tooHigh));
        Assert.Throws<ArgumentOutOfRangeException>(() => FixedQuaternion.FromEulerAngles(Fixed64.Zero, Fixed64.Zero, tooLow));
    }

    [Theory]
    [InlineData(0)]
    [InlineData(1)]
    [InlineData(2)]
    [InlineData(3)]
    public void FixedQuaternion_FuzzyEqualAbsolute_ReturnsFalse_WhenAnyComponentExceedsTolerance(int componentIndex)
    {
        var baseline = new FixedQuaternion(new Fixed64(1), new Fixed64(2), new Fixed64(3), new Fixed64(4));
        var changed = OffsetQuaternionComponent(baseline, componentIndex, Fixed64.FromFloatPoint(0.2));

        Assert.False(baseline.FuzzyEqualAbsolute(changed, Fixed64.FromFloatPoint(0.1)));
    }

    [Theory]
    [InlineData(0)]
    [InlineData(1)]
    [InlineData(2)]
    [InlineData(3)]
    public void FixedQuaternion_FuzzyEqual_ReturnsFalse_WhenAnyComponentExceedsPercentage(int componentIndex)
    {
        var baseline = new FixedQuaternion(new Fixed64(100), new Fixed64(100), new Fixed64(100), new Fixed64(100));
        var changed = OffsetQuaternionComponent(baseline, componentIndex, new Fixed64(50));

        Assert.False(baseline.FuzzyEqual(changed, Fixed64.FromFloatPoint(0.01)));
    }

    #endregion

    #region Test: Serialization


    [Fact]
    public void FixedQuanternion_NetSerialization_RoundTripMaintainsData()
    {
        var quaternion = FixedQuaternion.Identity;
        var sin = FixedMath.Sin(Fixed64.PiOver4);  // 45° rotation
        var cos = FixedMath.Cos(Fixed64.PiOver4);

        var originalRotation = quaternion.Rotated(sin, cos);

        var jsonOptions = new JsonSerializerOptions
        {
            DefaultIgnoreCondition = JsonIgnoreCondition.WhenWritingNull,
            ReferenceHandler = ReferenceHandler.IgnoreCycles
        };
        var json = JsonSerializer.SerializeToUtf8Bytes(originalRotation, jsonOptions);
        var deserializedRotation = JsonSerializer.Deserialize<FixedQuaternion>(json, jsonOptions);

        // Check that deserialized values match the original
        Assert.Equal(originalRotation, deserializedRotation);
    }

#if !FIXEDMATHSHARP_DISABLE_MEMORYPACK
    [Fact]
    public void FixedQuanternion_MemoryPackSerialization_RoundTripMaintainsData()
    {
        var quaternion = FixedQuaternion.Identity;
        var sin = FixedMath.Sin(Fixed64.PiOver4);  // 45° rotation
        var cos = FixedMath.Cos(Fixed64.PiOver4);

        FixedQuaternion originalValue = quaternion.Rotated(sin, cos);

        byte[] bytes = MemoryPackSerializer.Serialize(originalValue);
        FixedQuaternion deserializedValue = MemoryPackSerializer.Deserialize<FixedQuaternion>(bytes);

        // Check that deserialized values match the original
        Assert.Equal(originalValue, deserializedValue);
    }
#endif

    #endregion

    private static FixedQuaternion OffsetQuaternionComponent(FixedQuaternion quaternion, int componentIndex, Fixed64 offset)
    {
        switch (componentIndex)
        {
            case 0:
                quaternion.x += offset;
                break;
            case 1:
                quaternion.y += offset;
                break;
            case 2:
                quaternion.z += offset;
                break;
            case 3:
                quaternion.w += offset;
                break;
            default:
                throw new ArgumentOutOfRangeException(nameof(componentIndex));
        }

        return quaternion;
    }
}
