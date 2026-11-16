using Xunit;

namespace FixedMathSharp.Tests;

public class MatrixNormalizationAndComparisonTests
{
    [Fact]
    public void Fixed3x3_Normalization_PreservesSkewAndRebuildsThirdRow()
    {
        Vector3d skewedUp = new(1, 1, 0);
        Fixed3x3 matrix = new(new Vector3d(2, 0, 0), skewedUp, new Vector3d(0, 0, -4));
        Fixed3x3 normalized = Fixed3x3.GetNormalized(matrix);

        Assert.Equal(new Fixed3x3(Vector3d.Right, skewedUp.Normalized, Vector3d.Forward), normalized);
        Assert.Equal(new Fixed64(-4), matrix.M33);
        Assert.Equal(normalized, matrix.NormalizeInPlace());
        Assert.Equal(normalized, matrix);
    }

    [Fact]
    public void Fixed4x4_Normalization_PreservesSkewAndThirdRowDirection()
    {
        Fixed4x4 matrix = Fixed4x4.FromRows(
            new Vector4d(2, 0, 0, 0),
            new Vector4d(1, 1, 0, 0),
            new Vector4d(0, 0, -4, 0),
            new Vector4d(5, 6, 7, 1));
        Assert.True(matrix.IsAffine);
        matrix.M14 = Fixed64.One;
        Assert.False(matrix.IsAffine);

        Fixed4x4 normalized = Fixed4x4.NormalizeRotationMatrix(matrix);
        Vector3d up = new Vector3d(1, 1, 0).Normalized;
        Assert.Equal(Fixed4x4.FromRows(
            new Vector4d(1, 0, 0, 0),
            new Vector4d(up.X, up.Y, up.Z, Fixed64.Zero),
            new Vector4d(0, 0, -1, 0),
            new Vector4d(0, 0, 0, 1)), normalized);
        Assert.False(Fixed4x4.Decompose(normalized, out _, out _, out _));
        Assert.Equal(normalized, matrix.NormalizeRotationMatrix());
        Assert.Equal(normalized, matrix);
    }

    [Fact]
    public void Fixed3x3_FuzzyEqual_UsesEpsilonAndTheFirstOperandAsReference()
    {
        Fixed3x3 expected = Fixed3x3.Identity;
        Fixed3x3 actual = expected;
        actual.M11 += Fixed64.Epsilon;
        Assert.True(expected.FuzzyEqual(actual));

        actual.M11 += Fixed64.MinIncrement;
        Assert.False(expected.FuzzyEqual(actual));
        Assert.True(expected.FuzzyEqual(actual, Fixed64.FromDecimal(0.01m)));

        actual.M11 = Fixed64.Two;
        Assert.False(expected.FuzzyEqual(actual, Fixed64.Half));
        Assert.True(actual.FuzzyEqual(expected, Fixed64.Half));
    }

    [Fact]
    public void Fixed4x4_FuzzyEqual_UsesEpsilonAndTheFirstOperandAsReference()
    {
        Fixed4x4 expected = Fixed4x4.Identity;
        Fixed4x4 actual = expected;
        actual.M11 += Fixed64.Epsilon;
        Assert.True(expected.FuzzyEqual(actual));

        actual.M11 += Fixed64.MinIncrement;
        Assert.False(expected.FuzzyEqual(actual));
        Assert.True(expected.FuzzyEqual(actual, Fixed64.FromDecimal(0.01m)));

        actual.M11 = Fixed64.Two;
        Assert.False(expected.FuzzyEqual(actual, Fixed64.Half));
        Assert.True(actual.FuzzyEqual(expected, Fixed64.Half));
    }

    [Fact]
    public void FixedTransform_PlanarInverse_AcceptsZeroYScale()
    {
        FixedTransform transform = new(new Vector3d(3, 4, 5), FixedQuaternion.Identity, new Vector3d(1, 0, 1));

        Assert.False(transform.TryGetWorldToLocalMatrix(out _));
        Assert.True(transform.TryInverseTransformPointXZ(new Vector2d(4, 6), out Vector2d local));
        Assert.Equal(Vector2d.One, local);
    }
}
