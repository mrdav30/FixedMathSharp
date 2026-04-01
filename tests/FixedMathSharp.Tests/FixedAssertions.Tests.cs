using System;
using FixedMathSharp.Assertions;
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
        Fixed64 value = new Fixed64(1.25);
        Fixed64 farValue = new Fixed64(2);

        AssertAssertionPasses(() =>
        {
            value.Should().Be(value);
            value.Should().NotBe(farValue);
            value.Should().BeApproximately(new Fixed64(1.25));
            value.Should().NotBeApproximately(farValue, new Fixed64(0.1));
            value.Should().HaveRawValue(value.m_rawValue);
        });
    }

    [Fact]
    public void Fixed64Assertions_FailForApproximateAndRawValueMismatches()
    {
        Fixed64 value = new Fixed64(1.25);

        AssertAssertionFails(() => value.Should().Be(new Fixed64(2)));
        AssertAssertionFails(() => value.Should().NotBe(value));
        AssertAssertionFails(() => value.Should().BeApproximately(new Fixed64(2), new Fixed64(0.1)));
        AssertAssertionFails(() => value.Should().NotBeApproximately(new Fixed64(1.3), new Fixed64(0.1)));
        AssertAssertionFails(() => value.Should().HaveRawValue(value.m_rawValue + 1));
    }

    [Fact]
    public void Vector2dAssertions_SupportApproximateMagnitudeAndNormalizationChecks()
    {
        Vector2d actual = new Vector2d(2.001, 3.001);
        Vector2d expected = new Vector2d(2, 3);
        Vector2d normalized = new Vector2d(3, 4).Normalize();

        AssertAssertionPasses(() =>
        {
            actual.Should().Be(actual);
            actual.Should().NotBe(new Vector2d(9, 9));
            actual.Should().BeApproximately(expected, new Fixed64(0.01));
            actual.Should().NotBeApproximately(new Vector2d(5, 7), new Fixed64(0.1));
            actual.Should().HaveComponentApproximately(expected, new Fixed64(0.01));
            actual.Should().HaveMagnitudeApproximately(actual.Magnitude, Fixed64.Epsilon);
            normalized.Should().BeNormalized(new Fixed64(0.0001));
        });
    }

    [Fact]
    public void Vector2dAssertions_FailWhenExpectedValuesDoNotMatch()
    {
        Vector2d actual = new Vector2d(2, 3);

        AssertAssertionFails(() => actual.Should().Be(new Vector2d(2, 4)));
        AssertAssertionFails(() => actual.Should().NotBe(actual));
        AssertAssertionFails(() => actual.Should().BeApproximately(new Vector2d(2, 4), Fixed64.Epsilon));
        AssertAssertionFails(() => actual.Should().NotBeApproximately(new Vector2d(2.001, 3.001), new Fixed64(0.01)));
        AssertAssertionFails(() => actual.Should().HaveComponentApproximately(new Vector2d(2, 4), Fixed64.Epsilon));
        AssertAssertionFails(() => actual.Should().HaveMagnitudeApproximately(Fixed64.Zero, Fixed64.Epsilon));
        AssertAssertionFails(() => actual.Should().BeNormalized(new Fixed64(0.0001)));
    }

    [Fact]
    public void Vector3dAssertions_SupportApproximateMagnitudeAndNormalizationChecks()
    {
        Vector3d actual = new Vector3d(1.001, 2.001, 3.001);
        Vector3d expected = new Vector3d(1, 2, 3);
        Vector3d normalized = new Vector3d(3, 4, 0).Normalize();

        AssertAssertionPasses(() =>
        {
            actual.Should().Be(actual);
            actual.Should().NotBe(new Vector3d(9, 9, 9));
            actual.Should().BeApproximately(expected, new Fixed64(0.01));
            actual.Should().NotBeApproximately(new Vector3d(8, 8, 8), new Fixed64(0.1));
            actual.Should().HaveComponentApproximately(expected, new Fixed64(0.01));
            actual.Should().HaveMagnitudeApproximately(actual.Magnitude, Fixed64.Epsilon);
            normalized.Should().BeNormalized(new Fixed64(0.0001));
        });
    }

    [Fact]
    public void Vector3dAssertions_FailWhenExpectedValuesDoNotMatch()
    {
        Vector3d actual = new Vector3d(1, 2, 3);

        AssertAssertionFails(() => actual.Should().Be(new Vector3d(1, 2, 4)));
        AssertAssertionFails(() => actual.Should().NotBe(actual));
        AssertAssertionFails(() => actual.Should().BeApproximately(new Vector3d(1, 2, 4), Fixed64.Epsilon));
        AssertAssertionFails(() => actual.Should().NotBeApproximately(new Vector3d(1.001, 2.001, 3.001), new Fixed64(0.01)));
        AssertAssertionFails(() => actual.Should().HaveComponentApproximately(new Vector3d(1, 2, 4), Fixed64.Epsilon));
        AssertAssertionFails(() => actual.Should().HaveMagnitudeApproximately(Fixed64.Zero, Fixed64.Epsilon));
        AssertAssertionFails(() => actual.Should().BeNormalized(new Fixed64(0.0001)));
    }

    [Fact]
    public void FixedQuaternionAssertions_SupportEqualityApproximateAndIdentityChecks()
    {
        FixedQuaternion actual = FixedQuaternion.FromAxisAngle(Vector3d.Up, FixedMath.PiOver2);
        FixedQuaternion negated = actual * -Fixed64.One;
        FixedQuaternion wOffset = new FixedQuaternion(actual.x, actual.y, actual.z, actual.w + new Fixed64(0.2));

        AssertAssertionPasses(() =>
        {
            actual.Should().Be(actual);
            actual.Should().NotBe(FixedQuaternion.Identity);
            actual.Should().BeApproximately(actual, Fixed64.Epsilon);
            actual.Should().NotBeApproximately(wOffset, new Fixed64(0.01));
            actual.Should().HaveComponentApproximately(actual, Fixed64.Epsilon);
            actual.Should().RepresentSameRotationAs(negated, new Fixed64(0.0001));
            actual.Should().BeNormalized(new Fixed64(0.0001));
            FixedQuaternion.Identity.Should().BeIdentity();
        });
    }

    [Fact]
    public void FixedQuaternionAssertions_FailForDifferentRotationsAndNormalization()
    {
        FixedQuaternion actual = FixedQuaternion.FromAxisAngle(Vector3d.Up, FixedMath.PiOver2);
        FixedQuaternion differentRotation = FixedQuaternion.FromAxisAngle(Vector3d.Right, FixedMath.PiOver2);
        FixedQuaternion scaled = actual * new Fixed64(2);
        FixedQuaternion wOffset = new FixedQuaternion(actual.x, actual.y, actual.z, actual.w + new Fixed64(0.2));

        AssertAssertionFails(() => actual.Should().Be(FixedQuaternion.Identity));
        AssertAssertionFails(() => FixedQuaternion.Identity.Should().NotBe(FixedQuaternion.Identity));
        AssertAssertionFails(() => actual.Should().BeApproximately(wOffset, new Fixed64(0.01)));
        AssertAssertionFails(() => actual.Should().NotBeApproximately(actual, Fixed64.Epsilon));
        AssertAssertionFails(() => actual.Should().RepresentSameRotationAs(differentRotation, new Fixed64(0.0001)));
        AssertAssertionFails(() => scaled.Should().BeNormalized(new Fixed64(0.0001)));
        AssertAssertionFails(() => actual.Should().BeIdentity());
    }

    [Fact]
    public void Fixed3x3Assertions_SupportIdentityScaleAndAxisAssertions()
    {
        Vector3d scale = new Vector3d(2, 3, 4);
        Fixed3x3 rotation = Fixed3x3.CreateRotationY(FixedMath.PiOver2);
        Fixed3x3 scaleMatrix = Fixed3x3.CreateScale(scale);
        Fixed3x3 mismatch = new Fixed3x3(
            rotation.m00, rotation.m01, rotation.m02,
            rotation.m10, rotation.m11, rotation.m12,
            rotation.m20, rotation.m21, rotation.m22 + new Fixed64(0.2));

        AssertAssertionPasses(() =>
        {
            Fixed3x3.Identity.Should().BeIdentity();
            rotation.Should().NotBe(Fixed3x3.Zero);
            rotation.Should().BeApproximately(rotation, Fixed64.Epsilon);
            rotation.Should().NotBeApproximately(mismatch, new Fixed64(0.01));
            scaleMatrix.Should().HaveScaleApproximately(scale, Fixed64.Epsilon);
            rotation.Should().HaveNormalizedAxes(Fixed64.Epsilon);
        });
    }

    [Fact]
    public void Fixed3x3Assertions_FailForIdentityScaleAndAxisMismatches()
    {
        Fixed3x3 rotation = Fixed3x3.CreateRotationY(FixedMath.PiOver2);
        Fixed3x3 mismatch = new Fixed3x3(
            rotation.m00, rotation.m01, rotation.m02,
            rotation.m10, rotation.m11, rotation.m12,
            rotation.m20, rotation.m21, rotation.m22 + new Fixed64(0.2));
        Fixed3x3 unnormalizedAxes = Fixed3x3.CreateScale(new Vector3d(1, 1, 2));

        AssertAssertionFails(() => rotation.Should().BeIdentity());
        AssertAssertionFails(() => Fixed3x3.Identity.Should().NotBe(Fixed3x3.Identity));
        AssertAssertionFails(() => rotation.Should().BeApproximately(mismatch, new Fixed64(0.01)));
        AssertAssertionFails(() => rotation.Should().HaveScaleApproximately(new Vector3d(1, 1, 2), Fixed64.Epsilon));
        AssertAssertionFails(() => unnormalizedAxes.Should().HaveNormalizedAxes(Fixed64.Epsilon));
    }

    [Fact]
    public void Fixed4x4Assertions_SupportIdentityAffineAndTransformAssertions()
    {
        Vector3d translation = new Vector3d(1, 2, 3);
        FixedQuaternion rotation = FixedQuaternion.FromAxisAngle(Vector3d.Up, FixedMath.PiOver2);
        Vector3d scale = new Vector3d(2, 2, 2);
        Fixed4x4 transform = Fixed4x4.ScaleRotateTranslate(translation, rotation, scale);
        Fixed4x4 rotationOnly = Fixed4x4.CreateRotation(rotation);
        Fixed4x4 mismatch = new Fixed4x4(
            transform.m00, transform.m01, transform.m02, transform.m03,
            transform.m10, transform.m11, transform.m12, transform.m13,
            transform.m20, transform.m21, transform.m22, transform.m23,
            transform.m30, transform.m31, transform.m32, transform.m33 + new Fixed64(0.2));

        AssertAssertionPasses(() =>
        {
            Fixed4x4.Identity.Should().BeIdentity();
            transform.Should().NotBe(Fixed4x4.Identity);
            transform.Should().BeApproximately(transform, Fixed64.Epsilon);
            transform.Should().NotBeApproximately(mismatch, new Fixed64(0.01));
            transform.Should().BeAffine();
            transform.Should().HaveTranslationApproximately(translation, new Fixed64(0.0001));
            transform.Should().HaveRotationApproximately(rotation, new Fixed64(0.0001));
            transform.Should().HaveScaleApproximately(scale, new Fixed64(0.0001));
            rotationOnly.Should().HaveNormalizedRotationBasis(Fixed64.Epsilon);
        });
    }

    [Fact]
    public void Fixed4x4Assertions_FailForIdentityAffineAndTransformMismatches()
    {
        Vector3d translation = new Vector3d(1, 2, 3);
        FixedQuaternion rotation = FixedQuaternion.FromAxisAngle(Vector3d.Up, FixedMath.PiOver2);
        FixedQuaternion differentRotation = FixedQuaternion.FromAxisAngle(Vector3d.Right, FixedMath.PiOver2);
        Vector3d scale = new Vector3d(2, 2, 2);
        Fixed4x4 transform = Fixed4x4.ScaleRotateTranslate(translation, rotation, scale);
        Fixed4x4 mismatch = new Fixed4x4(
            transform.m00, transform.m01, transform.m02, transform.m03,
            transform.m10, transform.m11, transform.m12, transform.m13,
            transform.m20, transform.m21, transform.m22, transform.m23,
            transform.m30, transform.m31, transform.m32, transform.m33 + new Fixed64(0.2));
        Fixed4x4 nonAffine = transform;
        nonAffine.m03 = Fixed64.One;
        Fixed4x4 unnormalizedBasis = Fixed4x4.CreateScale(new Vector3d(1, 1, 2));

        AssertAssertionFails(() => transform.Should().BeIdentity());
        AssertAssertionFails(() => Fixed4x4.Identity.Should().NotBe(Fixed4x4.Identity));
        AssertAssertionFails(() => transform.Should().BeApproximately(mismatch, new Fixed64(0.01)));
        AssertAssertionFails(() => Fixed4x4.Identity.Should().NotBeApproximately(Fixed4x4.Identity, Fixed64.Epsilon));
        AssertAssertionFails(() => nonAffine.Should().BeAffine());
        AssertAssertionFails(() => transform.Should().HaveTranslationApproximately(new Vector3d(1, 2, 4), new Fixed64(0.0001)));
        AssertAssertionFails(() => transform.Should().HaveScaleApproximately(new Vector3d(2, 2, 3), new Fixed64(0.0001)));
        AssertAssertionFails(() => transform.Should().HaveRotationApproximately(differentRotation, new Fixed64(0.0001)));
        AssertAssertionFails(() => unnormalizedBasis.Should().HaveNormalizedRotationBasis(Fixed64.Epsilon));
    }
}
