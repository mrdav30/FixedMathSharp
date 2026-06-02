using FixedMathSharp.Assertions;
using System;
using Xunit;

namespace FixedMathSharp.Tests;

public class FixedAssertionsTests
{
    private static void AssertAssertionPasses(Action assertion)
    {
        Assert.Null(Record.Exception(assertion));
    }

    private static void AssertAssertionFails(Action assertion)
    {
        Assert.NotNull(Record.Exception(assertion));
    }

    [Fact]
    public void Fixed64Assertions_SupportApproximateEqualityAndRawValueChecks()
    {
        Fixed64 value = Fixed64.FromFloatPoint(1.25);
        Fixed64 farValue = new(2);

        AssertAssertionPasses(() =>
        {
            value.Should().Be(value);
            value.Should().NotBe(farValue);
            value.Should().BeApproximately(Fixed64.FromFloatPoint(1.25));
            value.Should().NotBeApproximately(farValue, Fixed64.FromFloatPoint(0.1));
            value.Should().HaveRawValue(value.m_rawValue);
        });
    }

    [Fact]
    public void Fixed64Assertions_FailForApproximateAndRawValueMismatches()
    {
        Fixed64 value = Fixed64.FromFloatPoint(1.25);

        AssertAssertionFails(() => value.Should().Be(Fixed64.FromFloatPoint(2)));
        AssertAssertionFails(() => value.Should().NotBe(value));
        AssertAssertionFails(() => value.Should().BeApproximately(new Fixed64(2), Fixed64.FromFloatPoint(0.1)));
        AssertAssertionFails(() => value.Should().NotBeApproximately(Fixed64.FromFloatPoint(1.3), Fixed64.FromFloatPoint(0.1)));
        AssertAssertionFails(() => value.Should().HaveRawValue(value.m_rawValue + 1));
    }

    [Fact]
    public void Vector2dAssertions_SupportApproximateMagnitudeAndNormalizationChecks()
    {
        Vector2d actual = new(2.001, 3.001);
        Vector2d expected = new(2, 3);
        Vector2d normalized = new Vector2d(3, 4).Normalize();

        AssertAssertionPasses(() =>
        {
            actual.Should().Be(actual);
            actual.Should().NotBe(new Vector2d(9, 9));
            actual.Should().BeApproximately(expected, Fixed64.FromFloatPoint(0.01));
            actual.Should().NotBeApproximately(new Vector2d(5, 7), Fixed64.FromFloatPoint(0.1));
            actual.Should().HaveComponentApproximately(expected, Fixed64.FromFloatPoint(0.01));
            actual.Should().HaveMagnitudeApproximately(actual.Magnitude, Fixed64.Epsilon);
            normalized.Should().BeNormalized(Fixed64.FromFloatPoint(0.0001));
        });
    }

    [Fact]
    public void Vector2dAssertions_FailWhenExpectedValuesDoNotMatch()
    {
        Vector2d actual = new(2, 3);

        AssertAssertionFails(() => actual.Should().Be(new Vector2d(2, 4)));
        AssertAssertionFails(() => actual.Should().NotBe(actual));
        AssertAssertionFails(() => actual.Should().BeApproximately(new Vector2d(2, 4), Fixed64.Epsilon));
        AssertAssertionFails(() => actual.Should().NotBeApproximately(new Vector2d(2.001, 3.001), Fixed64.FromFloatPoint(0.01)));
        AssertAssertionFails(() => actual.Should().HaveComponentApproximately(new Vector2d(2, 4), Fixed64.Epsilon));
        AssertAssertionFails(() => actual.Should().HaveMagnitudeApproximately(Fixed64.Zero, Fixed64.Epsilon));
        AssertAssertionFails(() => actual.Should().BeNormalized(Fixed64.FromFloatPoint(0.0001)));
    }

    [Fact]
    public void Vector3dAssertions_SupportApproximateMagnitudeAndNormalizationChecks()
    {
        Vector3d actual = new(1.001, 2.001, 3.001);
        Vector3d expected = new(1, 2, 3);
        Vector3d normalized = new Vector3d(3, 4, 0).Normalize();

        AssertAssertionPasses(() =>
        {
            actual.Should().Be(actual);
            actual.Should().NotBe(new Vector3d(9, 9, 9));
            actual.Should().BeApproximately(expected, Fixed64.FromFloatPoint(0.01));
            actual.Should().NotBeApproximately(new Vector3d(8, 8, 8), Fixed64.FromFloatPoint(0.1));
            actual.Should().HaveComponentApproximately(expected, Fixed64.FromFloatPoint(0.01));
            actual.Should().HaveMagnitudeApproximately(actual.Magnitude, Fixed64.Epsilon);
            normalized.Should().BeNormalized(Fixed64.FromFloatPoint(0.0001));
        });
    }

    [Fact]
    public void Vector3dAssertions_FailWhenExpectedValuesDoNotMatch()
    {
        Vector3d actual = new(1, 2, 3);

        AssertAssertionFails(() => actual.Should().Be(new Vector3d(1, 2, 4)));
        AssertAssertionFails(() => actual.Should().NotBe(actual));
        AssertAssertionFails(() => actual.Should().BeApproximately(new Vector3d(1, 2, 4), Fixed64.Epsilon));
        AssertAssertionFails(() => actual.Should().NotBeApproximately(new Vector3d(1.001, 2.001, 3.001), Fixed64.FromFloatPoint(0.01)));
        AssertAssertionFails(() => actual.Should().HaveComponentApproximately(new Vector3d(1, 2, 4), Fixed64.Epsilon));
        AssertAssertionFails(() => actual.Should().HaveMagnitudeApproximately(Fixed64.Zero, Fixed64.Epsilon));
        AssertAssertionFails(() => actual.Should().BeNormalized(Fixed64.FromFloatPoint(0.0001)));
    }

    [Fact]
    public void FixedQuaternionAssertions_SupportEqualityApproximateAndIdentityChecks()
    {
        FixedQuaternion actual = FixedQuaternion.FromAxisAngle(Vector3d.Up, Fixed64.HalfPi);
        FixedQuaternion negated = actual * -Fixed64.One;
        FixedQuaternion wOffset = new(actual.X, actual.Y, actual.Z, actual.W + Fixed64.FromFloatPoint(0.2));

        AssertAssertionPasses(() =>
        {
            actual.Should().Be(actual);
            actual.Should().NotBe(FixedQuaternion.Identity);
            actual.Should().BeApproximately(actual, Fixed64.Epsilon);
            actual.Should().NotBeApproximately(wOffset, Fixed64.FromFloatPoint(0.01));
            actual.Should().HaveComponentApproximately(actual, Fixed64.Epsilon);
            actual.Should().RepresentSameRotationAs(negated, Fixed64.FromFloatPoint(0.0001));
            actual.Should().BeNormalized(Fixed64.FromFloatPoint(0.0001));
            FixedQuaternion.Identity.Should().BeIdentity();
        });
    }

    [Fact]
    public void FixedQuaternionAssertions_FailForDifferentRotationsAndNormalization()
    {
        FixedQuaternion actual = FixedQuaternion.FromAxisAngle(Vector3d.Up, Fixed64.HalfPi);
        FixedQuaternion differentRotation = FixedQuaternion.FromAxisAngle(Vector3d.Right, Fixed64.HalfPi);
        FixedQuaternion scaled = actual * new Fixed64(2);
        FixedQuaternion wOffset = new(actual.X, actual.Y, actual.Z, actual.W + Fixed64.FromFloatPoint(0.2));

        AssertAssertionFails(() => actual.Should().Be(FixedQuaternion.Identity));
        AssertAssertionFails(() => FixedQuaternion.Identity.Should().NotBe(FixedQuaternion.Identity));
        AssertAssertionFails(() => actual.Should().BeApproximately(wOffset, Fixed64.FromFloatPoint(0.01)));
        AssertAssertionFails(() => actual.Should().NotBeApproximately(actual, Fixed64.Epsilon));
        AssertAssertionFails(() => actual.Should().RepresentSameRotationAs(differentRotation, Fixed64.FromFloatPoint(0.0001)));
        AssertAssertionFails(() => scaled.Should().BeNormalized(Fixed64.FromFloatPoint(0.0001)));
        AssertAssertionFails(() => actual.Should().BeIdentity());
    }

    [Fact]
    public void Fixed3x3Assertions_SupportIdentityScaleAndAxisAssertions()
    {
        Vector3d scale = new(2, 3, 4);
        Fixed3x3 rotation = Fixed3x3.CreateRotationY(Fixed64.HalfPi);
        Fixed3x3 scaleMatrix = Fixed3x3.CreateScale(scale);
        Fixed3x3 mismatch = new(
            rotation.M11, rotation.M12, rotation.M13,
            rotation.M21, rotation.M22, rotation.M23,
            rotation.M31, rotation.M32, rotation.M33 + Fixed64.FromFloatPoint(0.2));

        AssertAssertionPasses(() =>
        {
            Fixed3x3.Identity.Should().BeIdentity();
            rotation.Should().NotBe(Fixed3x3.Zero);
            rotation.Should().BeApproximately(rotation, Fixed64.Epsilon);
            rotation.Should().NotBeApproximately(mismatch, Fixed64.FromFloatPoint(0.01));
            scaleMatrix.Should().HaveScaleApproximately(scale, Fixed64.Epsilon);
            rotation.Should().HaveNormalizedAxes(Fixed64.Epsilon);
        });
    }

    [Fact]
    public void Fixed3x3Assertions_FailForIdentityScaleAndAxisMismatches()
    {
        Fixed3x3 rotation = Fixed3x3.CreateRotationY(Fixed64.HalfPi);
        Fixed3x3 mismatch = new(
            rotation.M11, rotation.M12, rotation.M13,
            rotation.M21, rotation.M22, rotation.M23,
            rotation.M31, rotation.M32, rotation.M33 + Fixed64.FromFloatPoint(0.2));
        Fixed3x3 unnormalizedAxes = Fixed3x3.CreateScale(new Vector3d(1, 1, 2));

        AssertAssertionFails(() => rotation.Should().BeIdentity());
        AssertAssertionFails(() => Fixed3x3.Identity.Should().NotBe(Fixed3x3.Identity));
        AssertAssertionFails(() => rotation.Should().BeApproximately(mismatch, Fixed64.FromFloatPoint(0.01)));
        AssertAssertionFails(() => rotation.Should().HaveScaleApproximately(new Vector3d(1, 1, 2), Fixed64.Epsilon));
        AssertAssertionFails(() => unnormalizedAxes.Should().HaveNormalizedAxes(Fixed64.Epsilon));
    }

    [Fact]
    public void Fixed4x4Assertions_SupportIdentityAffineAndTransformAssertions()
    {
        Vector3d translation = new(1, 2, 3);
        FixedQuaternion rotation = FixedQuaternion.FromAxisAngle(Vector3d.Up, Fixed64.HalfPi);
        Vector3d scale = new(2, 2, 2);
        Fixed4x4 transform = Fixed4x4.ScaleRotateTranslate(translation, rotation, scale);
        Fixed4x4 rotationOnly = Fixed4x4.CreateRotation(rotation);
        Fixed4x4 mismatch = new(
            transform.M11, transform.M12, transform.M13, transform.M14,
            transform.M21, transform.M22, transform.M23, transform.M24,
            transform.M31, transform.M32, transform.M33, transform.M34,
            transform.M41, transform.M42, transform.M43, transform.M44 + Fixed64.FromFloatPoint(0.2));

        AssertAssertionPasses(() =>
        {
            Fixed4x4.Identity.Should().BeIdentity();
            transform.Should().NotBe(Fixed4x4.Identity);
            transform.Should().BeApproximately(transform, Fixed64.Epsilon);
            transform.Should().NotBeApproximately(mismatch, Fixed64.FromFloatPoint(0.01));
            transform.Should().BeAffine();
            transform.Should().HaveTranslationApproximately(translation, Fixed64.FromFloatPoint(0.0001));
            transform.Should().HaveRotationApproximately(rotation, Fixed64.FromFloatPoint(0.0001));
            transform.Should().HaveScaleApproximately(scale, Fixed64.FromFloatPoint(0.0001));
            rotationOnly.Should().HaveNormalizedRotationBasis(Fixed64.Epsilon);
        });
    }

    [Fact]
    public void Fixed4x4Assertions_FailForIdentityAffineAndTransformMismatches()
    {
        Vector3d translation = new(1, 2, 3);
        FixedQuaternion rotation = FixedQuaternion.FromAxisAngle(Vector3d.Up, Fixed64.HalfPi);
        FixedQuaternion differentRotation = FixedQuaternion.FromAxisAngle(Vector3d.Right, Fixed64.HalfPi);
        Vector3d scale = new(2, 2, 2);
        Fixed4x4 transform = Fixed4x4.ScaleRotateTranslate(translation, rotation, scale);
        Fixed4x4 mismatch = new(
            transform.M11, transform.M12, transform.M13, transform.M14,
            transform.M21, transform.M22, transform.M23, transform.M24,
            transform.M31, transform.M32, transform.M33, transform.M34,
            transform.M41, transform.M42, transform.M43, transform.M44 + Fixed64.FromFloatPoint(0.2));
        Fixed4x4 nonAffine = transform;
        nonAffine.M14 = Fixed64.One;
        Fixed4x4 unnormalizedBasis = Fixed4x4.CreateScale(new Vector3d(1, 1, 2));

        AssertAssertionFails(() => transform.Should().BeIdentity());
        AssertAssertionFails(() => Fixed4x4.Identity.Should().NotBe(Fixed4x4.Identity));
        AssertAssertionFails(() => transform.Should().BeApproximately(mismatch, Fixed64.FromFloatPoint(0.01)));
        AssertAssertionFails(() => Fixed4x4.Identity.Should().NotBeApproximately(Fixed4x4.Identity, Fixed64.Epsilon));
        AssertAssertionFails(() => nonAffine.Should().BeAffine());
        AssertAssertionFails(() => transform.Should().HaveTranslationApproximately(new Vector3d(1, 2, 4), Fixed64.FromFloatPoint(0.0001)));
        AssertAssertionFails(() => transform.Should().HaveScaleApproximately(new Vector3d(2, 2, 3), Fixed64.FromFloatPoint(0.0001)));
        AssertAssertionFails(() => transform.Should().HaveRotationApproximately(differentRotation, Fixed64.FromFloatPoint(0.0001)));
        AssertAssertionFails(() => unnormalizedBasis.Should().HaveNormalizedRotationBasis(Fixed64.Epsilon));
    }

    [Theory]
    [InlineData(0)]
    [InlineData(1)]
    public void Vector2dAssertions_ComponentApproximationFailsForEachComponent(int componentIndex)
    {
        var expected = new Vector2d(10, 10);
        var actual = expected;
        actual[componentIndex] += Fixed64.FromFloatPoint(0.2);

        AssertAssertionFails(() => actual.Should().HaveComponentApproximately(expected, Fixed64.FromFloatPoint(0.1)));
    }

    [Theory]
    [InlineData(0)]
    [InlineData(1)]
    [InlineData(2)]
    public void Vector3dAssertions_ComponentApproximationFailsForEachComponent(int componentIndex)
    {
        var expected = new Vector3d(10, 10, 10);
        var actual = expected;
        actual[componentIndex] += Fixed64.FromFloatPoint(0.2);

        AssertAssertionFails(() => actual.Should().HaveComponentApproximately(expected, Fixed64.FromFloatPoint(0.1)));
    }

    [Theory]
    [InlineData(0)]
    [InlineData(1)]
    [InlineData(2)]
    [InlineData(3)]
    public void FixedQuaternionAssertions_ComponentApproximationFailsForEachComponent(int componentIndex)
    {
        var expected = new FixedQuaternion(new Fixed64(10), new Fixed64(10), new Fixed64(10), new Fixed64(10));
        var actual = OffsetQuaternionComponent(expected, componentIndex, Fixed64.FromFloatPoint(0.2));

        AssertAssertionFails(() => actual.Should().BeApproximately(expected, Fixed64.FromFloatPoint(0.1)));
    }

    [Theory]
    [InlineData(0)]
    [InlineData(1)]
    [InlineData(2)]
    [InlineData(3)]
    [InlineData(4)]
    [InlineData(5)]
    [InlineData(6)]
    [InlineData(7)]
    [InlineData(8)]
    public void Fixed3x3Assertions_ComponentApproximationFailsForEachComponent(int componentIndex)
    {
        var expected = CreateSequentialMatrix3x3();
        var actual = OffsetMatrixComponent(expected, componentIndex, Fixed64.FromFloatPoint(0.2));

        AssertAssertionFails(() => actual.Should().BeApproximately(expected, Fixed64.FromFloatPoint(0.1)));
    }

    [Theory]
    [InlineData(0)]
    [InlineData(1)]
    [InlineData(2)]
    [InlineData(3)]
    [InlineData(4)]
    [InlineData(5)]
    [InlineData(6)]
    [InlineData(7)]
    [InlineData(8)]
    [InlineData(9)]
    [InlineData(10)]
    [InlineData(11)]
    [InlineData(12)]
    [InlineData(13)]
    [InlineData(14)]
    [InlineData(15)]
    public void Fixed4x4Assertions_ComponentApproximationFailsForEachComponent(int componentIndex)
    {
        var expected = CreateSequentialMatrix4x4();
        var actual = OffsetMatrixComponent(expected, componentIndex, Fixed64.FromFloatPoint(0.2));

        AssertAssertionFails(() => actual.Should().BeApproximately(expected, Fixed64.FromFloatPoint(0.1)));
    }

    [Theory]
    [InlineData(0)]
    [InlineData(1)]
    public void Fixed3x3Assertions_NormalizedAxesFailWhenEarlyAxisIsNotUnitLength(int axisIndex)
    {
        Vector3d scale = axisIndex == 0
            ? new Vector3d(2, 1, 1)
            : new Vector3d(1, 2, 1);

        AssertAssertionFails(() => Fixed3x3.CreateScale(scale).Should().HaveNormalizedAxes(Fixed64.Epsilon));
    }

    [Theory]
    [InlineData(0)]
    [InlineData(1)]
    public void Fixed4x4Assertions_NormalizedRotationBasisFailsWhenEarlyAxisIsNotUnitLength(int axisIndex)
    {
        Vector3d scale = axisIndex == 0
            ? new Vector3d(2, 1, 1)
            : new Vector3d(1, 2, 1);

        AssertAssertionFails(() => Fixed4x4.CreateScale(scale).Should().HaveNormalizedRotationBasis(Fixed64.Epsilon));
    }

    private static FixedQuaternion OffsetQuaternionComponent(FixedQuaternion quaternion, int componentIndex, Fixed64 offset)
    {
        switch (componentIndex)
        {
            case 0:
                quaternion.X += offset;
                break;
            case 1:
                quaternion.Y += offset;
                break;
            case 2:
                quaternion.Z += offset;
                break;
            case 3:
                quaternion.W += offset;
                break;
            default:
                throw new ArgumentOutOfRangeException(nameof(componentIndex));
        }

        return quaternion;
    }

    private static Fixed3x3 CreateSequentialMatrix3x3()
    {
        return new Fixed3x3(
            new Fixed64(1), new Fixed64(2), new Fixed64(3),
            new Fixed64(4), new Fixed64(5), new Fixed64(6),
            new Fixed64(7), new Fixed64(8), new Fixed64(9));
    }

    private static Fixed3x3 OffsetMatrixComponent(Fixed3x3 matrix, int componentIndex, Fixed64 offset)
    {
        switch (componentIndex)
        {
            case 0:
                matrix.M11 += offset;
                break;
            case 1:
                matrix.M12 += offset;
                break;
            case 2:
                matrix.M13 += offset;
                break;
            case 3:
                matrix.M21 += offset;
                break;
            case 4:
                matrix.M22 += offset;
                break;
            case 5:
                matrix.M23 += offset;
                break;
            case 6:
                matrix.M31 += offset;
                break;
            case 7:
                matrix.M32 += offset;
                break;
            case 8:
                matrix.M33 += offset;
                break;
            default:
                throw new ArgumentOutOfRangeException(nameof(componentIndex));
        }

        return matrix;
    }

    private static Fixed4x4 CreateSequentialMatrix4x4()
    {
        return new Fixed4x4(
            new Fixed64(1), new Fixed64(2), new Fixed64(3), new Fixed64(4),
            new Fixed64(5), new Fixed64(6), new Fixed64(7), new Fixed64(8),
            new Fixed64(9), new Fixed64(10), new Fixed64(11), new Fixed64(12),
            new Fixed64(13), new Fixed64(14), new Fixed64(15), new Fixed64(16));
    }

    private static Fixed4x4 OffsetMatrixComponent(Fixed4x4 matrix, int componentIndex, Fixed64 offset)
    {
        switch (componentIndex)
        {
            case 0:
                matrix.M11 += offset;
                break;
            case 1:
                matrix.M12 += offset;
                break;
            case 2:
                matrix.M13 += offset;
                break;
            case 3:
                matrix.M14 += offset;
                break;
            case 4:
                matrix.M21 += offset;
                break;
            case 5:
                matrix.M22 += offset;
                break;
            case 6:
                matrix.M23 += offset;
                break;
            case 7:
                matrix.M24 += offset;
                break;
            case 8:
                matrix.M31 += offset;
                break;
            case 9:
                matrix.M32 += offset;
                break;
            case 10:
                matrix.M33 += offset;
                break;
            case 11:
                matrix.M34 += offset;
                break;
            case 12:
                matrix.M41 += offset;
                break;
            case 13:
                matrix.M42 += offset;
                break;
            case 14:
                matrix.M43 += offset;
                break;
            case 15:
                matrix.M44 += offset;
                break;
            default:
                throw new ArgumentOutOfRangeException(nameof(componentIndex));
        }

        return matrix;
    }
}
