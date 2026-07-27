using FixedMathSharp.Bounds;
using Xunit;

namespace FixedMathSharp.Tests.Geometry.Primitives;

public sealed class CanonicalAxisFrameTests
{
    [Fact]
    public void CanonicalAxisRotation_MapsUpToAxis()
    {
        Vector3d axis = new Vector3d(1, 2, -3).Normalized;

        FixedQuaternion rotation =
            WideGeometry.GetCanonicalAxisRotation(axis);

        Assert.True(rotation.IsNormalized());
        Assert.True(
            rotation.Rotate(Vector3d.Up).FuzzyEqual(
                axis,
                Fixed64.FromRaw(8)));
    }

    [Fact]
    public void CanonicalAxisRotation_UsesStableUpAndDownTies()
    {
        Assert.Equal(
            FixedQuaternion.Identity,
            WideGeometry.GetCanonicalAxisRotation(Vector3d.Up));

        FixedQuaternion down =
            WideGeometry.GetCanonicalAxisRotation(Vector3d.Down);
        Assert.True(down.IsNormalized());
        Assert.True(
            down.Rotate(Vector3d.Up).FuzzyEqual(
                Vector3d.Down,
                Fixed64.FromRaw(8)));
        Assert.True(
            down.Rotate(Vector3d.Right).FuzzyEqual(
                Vector3d.Right,
                Fixed64.FromRaw(8)));
    }

    [Fact]
    public void CanonicalAxisRotation_MirrorsWithoutChangingReferenceRoll()
    {
        Vector3d firstAxis = new Vector3d(1, 2, 3).Normalized;
        Vector3d mirroredAxis = new Vector3d(-1, 2, 3).Normalized;

        FixedQuaternion first =
            WideGeometry.GetCanonicalAxisRotation(firstAxis);
        FixedQuaternion mirrored =
            WideGeometry.GetCanonicalAxisRotation(mirroredAxis);

        Assert.True(first.Rotate(Vector3d.Up).FuzzyEqual(
            firstAxis,
            Fixed64.FromRaw(8)));
        Assert.True(mirrored.Rotate(Vector3d.Up).FuzzyEqual(
            mirroredAxis,
            Fixed64.FromRaw(8)));
        Assert.Equal(first.W, mirrored.W);
        Assert.Equal(first.X, mirrored.X);
        Assert.Equal(-first.Z, mirrored.Z);
    }
}
