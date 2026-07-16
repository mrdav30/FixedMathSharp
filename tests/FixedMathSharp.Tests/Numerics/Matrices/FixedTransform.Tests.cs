using Xunit;

namespace FixedMathSharp.Tests;

public sealed class FixedTransformTests
{
    private static readonly Fixed64 Tolerance = Fixed64.FromDouble(0.0001);

    [Fact]
    public void FixedTransform_FromComponents_PreservesComponentsAndNormalizesRotation()
    {
        Vector3d position = new((Fixed64)1, (Fixed64)2, (Fixed64)3);
        FixedQuaternion rotation = new(Fixed64.Zero, Fixed64.Zero, Fixed64.Zero, (Fixed64)2);
        Vector3d scale = new((Fixed64)(-2), Fixed64.Zero, (Fixed64)4);

        var transform = new FixedTransform(position, rotation, scale);

        Assert.Equal(position, transform.Position);
        Assert.Equal(FixedQuaternion.Identity, transform.Rotation);
        Assert.Equal(scale, transform.Scale);

        Vector3d updatedPosition = new((Fixed64)5, (Fixed64)6, (Fixed64)7);
        Vector3d updatedScale = new(Fixed64.Zero, (Fixed64)(-3), (Fixed64)8);
        transform.Position = updatedPosition;
        transform.Scale = updatedScale;
        transform.Rotation = new FixedQuaternion(Fixed64.Zero, Fixed64.Zero, Fixed64.Zero, (Fixed64)3);

        Assert.Equal(updatedPosition, transform.Position);
        Assert.Equal(updatedScale, transform.Scale);
        Assert.Equal(FixedQuaternion.Identity, transform.Rotation);
    }

    [Fact]
    public void FixedTransform_PlanarConstructor_UsesZeroElevationUnitYScaleAndPreservesParent()
    {
        var parent = new FixedTransform(Vector3d.Zero, FixedQuaternion.Identity, Vector3d.One);
        Vector2d position = new((Fixed64)2, (Fixed64)3);
        Vector2d scale = new((Fixed64)(-4), Fixed64.Zero);

        var transform = new FixedTransform(position, Fixed64.PiOver4, scale, parent);

        Assert.Equal(new Vector3d(position.X, Fixed64.Zero, position.Y), transform.Position);
        Assert.Equal(new Vector3d(scale.X, Fixed64.One, scale.Y), transform.Scale);
        Assert.Same(parent, transform.Parent);
        AssertAnglesEquivalent(Fixed64.PiOver4, transform.RotationXZRadians);
    }

    [Fact]
    public void FixedTransform_PlanarPositionAndScaleSetters_PreserveYComponents()
    {
        var transform = new FixedTransform(
            new Vector3d((Fixed64)1, (Fixed64)7, (Fixed64)2),
            FixedQuaternion.Identity,
            new Vector3d((Fixed64)3, (Fixed64)5, (Fixed64)4));

        transform.PositionXZ = new Vector2d((Fixed64)(-1), (Fixed64)(-2));
        transform.ScaleXZ = new Vector2d((Fixed64)(-3), Fixed64.Zero);

        Assert.Equal(new Vector3d((Fixed64)(-1), (Fixed64)7, (Fixed64)(-2)), transform.Position);
        Assert.Equal(new Vector3d((Fixed64)(-3), (Fixed64)5, Fixed64.Zero), transform.Scale);
        Assert.Equal(transform.Position.ToVector2d(), transform.PositionXZ);
        Assert.Equal(transform.Scale.ToVector2d(), transform.ScaleXZ);
    }

    [Fact]
    public void FixedTransform_RotationXZRadiansSetter_ReplacesPitchAndRoll()
    {
        var transform = new FixedTransform(
            Vector3d.Zero,
            FixedQuaternion.FromEulerAngles(Fixed64.PiOver6, Fixed64.PiOver4, Fixed64.PiOver3),
            Vector3d.One);

        transform.RotationXZRadians = Fixed64.PiOver3;

        FixedQuaternion expected = FixedQuaternion.FromAxisAngle(Vector3d.Up, -Fixed64.PiOver3).Normalized;
        Assert.Equal(expected, transform.Rotation);
        AssertAnglesEquivalent(Fixed64.PiOver3, transform.RotationXZRadians);
    }

    [Fact]
    public void FixedTransform_PlanarRotation_MatchesVector2dRotateBasis()
    {
        Fixed64[] angles = { Fixed64.Zero, Fixed64.HalfPi, -Fixed64.HalfPi };
        Vector2d[] expectedAxes = { Vector2d.Right, Vector2d.Forward, -Vector2d.Forward };

        for (int i = 0; i < angles.Length; i++)
        {
            var transform = new FixedTransform(Vector2d.Zero, angles[i], Vector2d.One);
            Vector2d actual = transform.Rotation.Rotate(Vector3d.Right).ToVector2d();
            Vector2d expected = Vector2d.Rotate(Vector2d.Right, angles[i]);

            Assert.True(actual.FuzzyEqual(expected, Tolerance), $"Expected {expected}, got {actual}.");
            Assert.True(actual.FuzzyEqual(expectedAxes[i], Tolerance), $"Expected {expectedAxes[i]}, got {actual}.");
        }
    }

    [Fact]
    public void FixedTransform_RotationXZRadians_RoundTripsModuloTwoPiIndependentOfScale()
    {
        Fixed64[] angles =
        {
            Fixed64.PiOver4,
            -Fixed64.PiOver3,
            Fixed64.Pi,
            -Fixed64.Pi,
            Fixed64.TwoPi,
            -Fixed64.TwoPi,
            Fixed64.PiOver6 + (Fixed64.TwoPi * (Fixed64)7),
            -Fixed64.PiOver4 - (Fixed64.TwoPi * (Fixed64)6),
        };

        foreach (Fixed64 angle in angles)
        {
            var transform = new FixedTransform(
                new Vector2d((Fixed64)2, (Fixed64)3),
                angle,
                new Vector2d((Fixed64)(-2), Fixed64.Zero));

            AssertAnglesEquivalent(angle, transform.RotationXZRadians);
            Assert.Equal(new Vector2d((Fixed64)(-2), Fixed64.Zero), transform.ScaleXZ);
        }
    }

    [Fact]
    public void FixedTransform_RotationXZRadians_ProjectsLocalRightIndependentOfScale()
    {
        FixedQuaternion rotation = FixedQuaternion.FromEulerAngles(
            Fixed64.PiOver6,
            Fixed64.PiOver4,
            Fixed64.PiOver3).Normalized;
        Vector3d localRight = rotation.Rotate(Vector3d.Right);
        Fixed64 expected = FixedMath.Atan2(localRight.Z, localRight.X);
        var first = new FixedTransform(Vector3d.Zero, rotation, new Vector3d((Fixed64)(-2), Fixed64.Zero, (Fixed64)3));
        var second = new FixedTransform(Vector3d.Zero, rotation, new Vector3d((Fixed64)9, (Fixed64)8, (Fixed64)(-7)));

        Assert.Equal(expected, first.RotationXZRadians);
        Assert.Equal(expected, second.RotationXZRadians);
    }

    [Fact]
    public void FixedTransform_RotationXZRadians_ZeroProjectionReturnsZero()
    {
        var transform = new FixedTransform(
            Vector3d.Zero,
            new FixedQuaternion(Fixed64.Zero, Fixed64.Zero, Fixed64.One, Fixed64.One),
            Vector3d.One);
        Vector3d localRight = transform.Rotation.Rotate(Vector3d.Right);

        Assert.Equal(Fixed64.Zero, localRight.X);
        Assert.Equal(Fixed64.Zero, localRight.Z);
        Assert.Equal(Fixed64.Zero, transform.RotationXZRadians);
    }

    [Fact]
    public void FixedTransform_FromMatrix_PreservesSupportedDecomposedComponents()
    {
        Vector3d position = new((Fixed64)(-5), (Fixed64)6, (Fixed64)7);
        FixedQuaternion rotation = FixedQuaternion.FromEulerAnglesInDegrees((Fixed64)(-20), (Fixed64)35, (Fixed64)50);
        Vector3d[] scales =
        {
            new Vector3d((Fixed64)3, (Fixed64)4, (Fixed64)5),
            new Vector3d((Fixed64)(-3), (Fixed64)4, (Fixed64)5),
        };

        foreach (Vector3d scale in scales)
        {
            var transform = new FixedTransform(Fixed4x4.CreateTransform(position, rotation, scale));

            Assert.Equal(position, transform.Position);
            Assert.True(transform.Scale.FuzzyEqual(scale, Tolerance), $"Expected {scale}, got {transform.Scale}.");
            AssertRepresentsSameRotation(transform.Rotation, rotation);
        }
    }

    [Fact]
    public void FixedTransform_FromMatrix_MatchesEstablishedAmbiguousDecomposition()
    {
        Fixed4x4[] matrices =
        {
            Fixed4x4.CreateTransform(
                new Vector3d((Fixed64)1, (Fixed64)2, (Fixed64)3),
                FixedQuaternion.FromEulerAngles(Fixed64.PiOver6, Fixed64.PiOver4, Fixed64.PiOver3),
                new Vector3d((Fixed64)(-2), (Fixed64)(-3), (Fixed64)4)),
            Fixed4x4.CreateScale(Vector3d.Zero),
        };

        foreach (Fixed4x4 matrix in matrices)
        {
            Assert.True(Fixed4x4.Decompose(matrix, out Vector3d position, out FixedQuaternion rotation, out Vector3d scale));

            var transform = new FixedTransform(matrix);

            Assert.Equal(position, transform.Position);
            Assert.Equal(scale, transform.Scale);
            AssertRepresentsSameRotation(transform.Rotation, rotation.Normalized);
        }
    }

    [Fact]
    public void FixedTransform_EulerAngles_SetterUsesDegreeBasedQuaternion()
    {
        var transform = new FixedTransform(Vector3d.Zero, FixedQuaternion.Identity, Vector3d.One);
        Vector3d eulerAngles = new((Fixed64)90, Fixed64.Zero, Fixed64.Zero);

        transform.EulerAngles = eulerAngles;

        Assert.True(transform.EulerAngles.FuzzyEqual(eulerAngles, Tolerance), $"Expected {eulerAngles}, got {transform.EulerAngles}.");
        AssertRepresentsSameRotation(transform.Rotation, FixedQuaternion.FromEulerAnglesInDegrees(eulerAngles.X, eulerAngles.Y, eulerAngles.Z));
    }

    [Fact]
    public void FixedTransform_Parent_CanBeAssignedAndCleared()
    {
        var parent = new FixedTransform(Vector3d.Zero, FixedQuaternion.Identity, Vector3d.One);
        var child = new FixedTransform(Vector3d.Zero, FixedQuaternion.Identity, Vector3d.One, parent);

        Assert.Same(parent, child.Parent);

        child.Parent = null;

        Assert.Null(child.Parent);
    }

    [Fact]
    public void FixedTransform_DoesNotExposeLossyScaleAlias()
    {
        Assert.Null(typeof(FixedTransform).GetProperty("LossyScale"));
    }

    private static void AssertAnglesEquivalent(Fixed64 expected, Fixed64 actual)
    {
        Fixed64 difference = (actual - expected) % Fixed64.TwoPi;
        if (difference > Fixed64.Pi)
            difference -= Fixed64.TwoPi;
        else if (difference < -Fixed64.Pi)
            difference += Fixed64.TwoPi;

        Assert.True(FixedMath.Abs(difference) <= Tolerance, $"Expected {expected} modulo TwoPi, got {actual}.");
    }

    private static void AssertRepresentsSameRotation(FixedQuaternion actual, FixedQuaternion expected)
    {
        Assert.True(
            actual.FuzzyEqual(expected, Tolerance) || actual.FuzzyEqual(expected * -Fixed64.One, Tolerance),
            $"Expected {actual} to represent the same rotation as {expected}.");
    }
}
