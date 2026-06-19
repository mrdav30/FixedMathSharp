using Xunit;

namespace FixedMathSharp.Tests;

public sealed class FixedTransformTests
{
    private static readonly Fixed64 Tolerance = Fixed64.FromDouble(0.0001);

    [Fact]
    public void FixedTransform_FromComponents_ExposesMatrixBackedComponents()
    {
        Vector3d position = new((Fixed64)1, (Fixed64)2, (Fixed64)3);
        FixedQuaternion rotation = FixedQuaternion.FromEulerAnglesInDegrees((Fixed64)15, (Fixed64)30, (Fixed64)45);
        Vector3d scale = new((Fixed64)2, (Fixed64)3, (Fixed64)4);

        var transform = new FixedTransform(position, rotation, scale);

        Assert.Equal(position, transform.Position);
        AssertRepresentsSameRotation(transform.Rotation, rotation);
        Assert.True(transform.Scale.FuzzyEqual(scale, Tolerance), $"Expected {scale}, got {transform.Scale}.");
        Assert.True(transform.LossyScale.FuzzyEqual(scale, Tolerance), $"Expected {scale}, got {transform.LossyScale}.");
    }

    [Fact]
    public void FixedTransform_FromMatrix_ExposesMatrixBackedComponents()
    {
        Vector3d position = new((Fixed64)(-5), (Fixed64)6, (Fixed64)7);
        FixedQuaternion rotation = FixedQuaternion.FromEulerAnglesInDegrees((Fixed64)(-20), (Fixed64)35, (Fixed64)50);
        Vector3d scale = new((Fixed64)3, (Fixed64)4, (Fixed64)5);
        Fixed4x4 matrix = Fixed4x4.CreateTransform(position, rotation, scale);

        var transform = new FixedTransform(matrix);

        Assert.Equal(position, transform.Position);
        AssertRepresentsSameRotation(transform.Rotation, rotation);
        Assert.True(transform.Scale.FuzzyEqual(scale, Tolerance), $"Expected {scale}, got {transform.Scale}.");
    }

    [Fact]
    public void FixedTransform_MutatingComponents_UpdatesMatrixBackedState()
    {
        var transform = new FixedTransform(Vector3d.Zero, FixedQuaternion.Identity, Vector3d.One);
        Vector3d position = new((Fixed64)8, (Fixed64)9, (Fixed64)10);
        FixedQuaternion rotation = FixedQuaternion.FromEulerAnglesInDegrees((Fixed64)25, (Fixed64)(-15), (Fixed64)40);
        Vector3d scale = new((Fixed64)2, (Fixed64)5, (Fixed64)7);

        transform.Position = position;
        transform.Scale = scale;
        transform.Rotation = rotation;

        Assert.Equal(position, transform.Position);
        AssertRepresentsSameRotation(transform.Rotation, rotation);
        Assert.True(transform.Scale.FuzzyEqual(scale, Tolerance), $"Expected {scale}, got {transform.Scale}.");
        Assert.True(transform.LossyScale.FuzzyEqual(scale, Tolerance), $"Expected {scale}, got {transform.LossyScale}.");
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
        var child = new FixedTransform(new Vector3d((Fixed64)1, Fixed64.Zero, Fixed64.Zero), FixedQuaternion.Identity, Vector3d.One);

        Assert.Null(child.Parent);

        child.Parent = parent;

        Assert.Same(parent, child.Parent);

        child.Parent = null;

        Assert.Null(child.Parent);
    }

    private static void AssertRepresentsSameRotation(FixedQuaternion actual, FixedQuaternion expected)
    {
        Assert.True(
            actual.FuzzyEqual(expected, Tolerance) || actual.FuzzyEqual(expected * -Fixed64.One, Tolerance),
            $"Expected {actual} to represent the same rotation as {expected}.");
    }
}
