using Xunit;

namespace FixedMathSharp.Tests;

public class Fixed4x4TryOperationsTests
{
    [Fact]
    public void TryTransformAffinePoint_MatchesRepresentableRowVectorTransform()
    {
        Fixed4x4 matrix = Fixed4x4.CreateTransform(
            new Vector3d(7, -11, 13),
            FixedQuaternion.FromEulerAnglesInDegrees((Fixed64)17, (Fixed64)(-29), (Fixed64)43),
            new Vector3d(2, 3, 4));
        Vector3d point = new(5, -7, 11);

        Assert.True(Fixed4x4.TryTransformAffinePoint(matrix, point, out Vector3d result));
        Assert.Equal(Fixed4x4.TransformPoint(matrix, point), result);
    }

    [Fact]
    public void TryTransformAffinePoint_RetainsCancellationAcrossOverflowingProducts()
    {
        Fixed4x4 matrix = new(
            Fixed64.Two, Fixed64.Zero, Fixed64.Zero, Fixed64.Zero,
            -Fixed64.Two, Fixed64.One, Fixed64.Zero, Fixed64.Zero,
            Fixed64.Zero, Fixed64.Zero, Fixed64.One, Fixed64.Zero,
            Fixed64.One, Fixed64.Zero, Fixed64.Zero, Fixed64.One);
        Vector3d point = new(Fixed64.MaxValue, Fixed64.MaxValue, Fixed64.Zero);

        Assert.True(Fixed4x4.TryTransformAffinePoint(matrix, point, out Vector3d result));
        Assert.Equal(new Vector3d(Fixed64.One, Fixed64.MaxValue, Fixed64.Zero), result);
    }

    [Theory]
    [InlineData(0)]
    [InlineData(1)]
    [InlineData(2)]
    public void TryTransformAffinePoint_UnrepresentableCoordinateFailsAtomically(int coordinate)
    {
        Fixed4x4 matrix = Fixed4x4.Identity;
        matrix[12 + coordinate] = Fixed64.MaxValue;
        Vector3d point = coordinate switch
        {
            0 => Vector3d.Right,
            1 => Vector3d.Up,
            _ => Vector3d.Forward,
        };

        Assert.False(Fixed4x4.TryTransformAffinePoint(matrix, point, out Vector3d result));
        Assert.Equal(Vector3d.Zero, result);
    }

    [Fact]
    public void TryTransformAffinePoint_RejectsNonAffineMatrices()
    {
        Fixed4x4 matrix = Fixed4x4.Identity;
        matrix.M14 = Fixed64.One;

        Assert.False(Fixed4x4.TryTransformAffinePoint(matrix, Vector3d.One, out Vector3d result));
        Assert.Equal(Vector3d.Zero, result);
    }

    [Fact]
    public void TryTransformAffinePoint_RoundsHalfEvenOnlyAfterTheCompleteCoordinate()
    {
        Fixed4x4 matrix = new(
            Fixed64.Half, Fixed64.Zero, Fixed64.Zero, Fixed64.Zero,
            Fixed64.Zero, Fixed64.One, Fixed64.Zero, Fixed64.Zero,
            Fixed64.Zero, Fixed64.Zero, Fixed64.One, Fixed64.Zero,
            Fixed64.Zero, Fixed64.Zero, Fixed64.Zero, Fixed64.One);

        Assert.True(Fixed4x4.TryTransformAffinePoint(
            matrix,
            new Vector3d(Fixed64.FromRaw(1), Fixed64.Zero, Fixed64.Zero),
            out Vector3d even));
        Assert.Equal(Fixed64.Zero, even.X);

        Assert.True(Fixed4x4.TryTransformAffinePoint(
            matrix,
            new Vector3d(Fixed64.FromRaw(3), Fixed64.Zero, Fixed64.Zero),
            out Vector3d odd));
        Assert.Equal(Fixed64.FromRaw(2), odd.X);
    }

    [Fact]
    public void TryTransformAffinePoint_AcceptsMinimumAndRejectsRoundingPastMaximum()
    {
        Fixed4x4 minimum = Fixed4x4.Identity;
        minimum.M41 = Fixed64.MinValue;
        Assert.True(Fixed4x4.TryTransformAffinePoint(minimum, Vector3d.Zero, out Vector3d exactMinimum));
        Assert.Equal(Fixed64.MinValue, exactMinimum.X);

        Fixed4x4 aboveMaximum = Fixed4x4.Identity;
        aboveMaximum.M11 = Fixed64.Half;
        aboveMaximum.M41 = Fixed64.MaxValue;
        Assert.False(Fixed4x4.TryTransformAffinePoint(
            aboveMaximum,
            new Vector3d(Fixed64.FromRaw(1), Fixed64.Zero, Fixed64.Zero),
            out Vector3d rejected));
        Assert.Equal(Vector3d.Zero, rejected);
    }

    [Fact]
    public void TryExtractLossyScale_PreservesExistingMagnitudeAndReflectionContract()
    {
        Fixed4x4 rightHanded = Fixed4x4.CreateScale(new Vector3d(2, 3, 4));
        Fixed4x4 reflected = Fixed4x4.CreateScale(new Vector3d(2, -3, 4));
        Fixed4x4 singular = Fixed4x4.CreateScale(
            new Vector3d(Fixed64.Zero, (Fixed64)3, (Fixed64)4));
        Fixed4x4 nonAffine = rightHanded;
        nonAffine.M14 = Fixed64.One;

        Assert.True(Fixed4x4.TryExtractLossyScale(rightHanded, out Vector3d rightHandedScale));
        Assert.Equal(new Vector3d(2, 3, 4), rightHandedScale);
        Assert.True(Fixed4x4.TryExtractLossyScale(reflected, out Vector3d reflectedScale));
        Assert.Equal(new Vector3d(-2, 3, 4), reflectedScale);
        Assert.True(Fixed4x4.TryExtractLossyScale(singular, out Vector3d singularScale));
        Assert.Equal(
            new Vector3d(Fixed64.Zero, (Fixed64)3, (Fixed64)4),
            singularScale);
        Assert.True(Fixed4x4.TryExtractLossyScale(nonAffine, out Vector3d nonAffineScale));
        Assert.Equal(new Vector3d(2, 3, 4), nonAffineScale);
    }

    [Fact]
    public void TryExtractLossyScale_AcceptsRepresentableCardinalLimit()
    {
        Fixed64 negativeLimit = Fixed64.FromRaw(-long.MaxValue);
        Fixed4x4 matrix = Fixed4x4.CreateScale(
            new Vector3d(negativeLimit, Fixed64.MaxValue, Fixed64.One));

        Assert.True(Fixed4x4.TryExtractLossyScale(matrix, out Vector3d scale));
        Assert.Equal(
            new Vector3d(-Fixed64.MaxValue, Fixed64.MaxValue, Fixed64.One),
            scale);
    }

    [Theory]
    [InlineData(0)]
    [InlineData(1)]
    [InlineData(2)]
    public void TryExtractLossyScale_UnrepresentableBasisMagnitudeFailsAtomically(int row)
    {
        Fixed4x4 matrix = Fixed4x4.Identity;
        matrix[row * 4] = Fixed64.MaxValue;
        matrix[(row * 4) + 1] = Fixed64.MaxValue;

        Assert.False(Fixed4x4.TryExtractLossyScale(matrix, out Vector3d scale));
        Assert.Equal(Vector3d.Zero, scale);
    }
}
