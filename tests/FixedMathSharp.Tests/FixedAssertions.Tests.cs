using System;
using FixedMathSharp.Assertions;
using Xunit;

namespace FixedMathSharp.Tests;

public class FixedAssertionsTests
{
    [Fact]
    public void Fixed64Assertions_BeApproximately_And_HaveRawValue_DoNotThrow()
    {
        Fixed64 value = new Fixed64(1.25);

        Exception? exception = Record.Exception(() =>
        {
            value.Should().BeApproximately(new Fixed64(1.25));
            value.Should().HaveRawValue(value.m_rawValue);
        });

        Assert.Null(exception);
    }

    [Fact]
    public void Vector2dAssertions_HaveComponentApproximately_DoesNotThrow()
    {
        Vector2d actual = new Vector2d(2.001, 3.001);
        Vector2d expected = new Vector2d(2, 3);

        Exception? exception = Record.Exception(() =>
        {
            actual.Should().HaveComponentApproximately(expected, new Fixed64(0.01));
            actual.Should().HaveMagnitudeApproximately(actual.Magnitude, Fixed64.Epsilon);
        });

        Assert.Null(exception);
    }

    [Fact]
    public void Vector3dAssertions_BeNormalized_DoesNotThrow()
    {
        Vector3d actual = new Vector3d(3, 4, 0).Normalize();

        Exception? exception = Record.Exception(() =>
        {
            actual.Should().BeNormalized(new Fixed64(0.0001));
            actual.Should().BeApproximately(new Vector3d(new Fixed64(0.6), new Fixed64(0.8), Fixed64.Zero), new Fixed64(0.0001));
        });

        Assert.Null(exception);
    }

    [Fact]
    public void FixedQuaternionAssertions_RepresentSameRotationAs_AcceptsNegatedQuaternion()
    {
        FixedQuaternion actual = FixedQuaternion.FromAxisAngle(Vector3d.Up, FixedMath.PiOver2);
        FixedQuaternion negated = actual * -Fixed64.One;

        Exception? exception = Record.Exception(() =>
        {
            actual.Should().RepresentSameRotationAs(negated, new Fixed64(0.0001));
            actual.Should().BeNormalized(new Fixed64(0.0001));
        });

        Assert.Null(exception);
    }

    [Fact]
    public void Fixed3x3Assertions_BeApproximately_And_HaveNormalizedAxes_DoNotThrow()
    {
        Fixed3x3 actual = Fixed3x3.CreateRotationY(FixedMath.PiOver2);
        Fixed3x3 expected = Fixed3x3.CreateRotationY(FixedMath.PiOver2);

        Exception? exception = Record.Exception(() =>
        {
            actual.Should().BeApproximately(expected, Fixed64.Epsilon);
            actual.Should().HaveNormalizedAxes(Fixed64.Epsilon);
        });

        Assert.Null(exception);
    }

    [Fact]
    public void Fixed4x4Assertions_TransformAssertions_DoNotThrow()
    {
        Vector3d translation = new Vector3d(1, 2, 3);
        FixedQuaternion rotation = FixedQuaternion.FromAxisAngle(Vector3d.Up, FixedMath.PiOver2);
        Vector3d scale = new Vector3d(2, 2, 2);
        Fixed4x4 matrix = Fixed4x4.ScaleRotateTranslate(translation, rotation, scale);

        Exception? exception = Record.Exception(() =>
        {
            matrix.Should().BeAffine();
            matrix.Should().HaveTranslationApproximately(translation, new Fixed64(0.0001));
            matrix.Should().HaveRotationApproximately(rotation, new Fixed64(0.0001));
            matrix.Should().HaveScaleApproximately(scale, new Fixed64(0.0001));
        });

        Assert.Null(exception);
    }

    [Fact]
    public void Fixed4x4Assertions_BeApproximately_ThrowsForClearlyDifferentMatrices()
    {
        Exception? exception = Record.Exception(() =>
        {
            Fixed4x4.Identity.Should().BeApproximately(Fixed4x4.Zero, Fixed64.Epsilon);
        });

        Assert.NotNull(exception);
    }
}
