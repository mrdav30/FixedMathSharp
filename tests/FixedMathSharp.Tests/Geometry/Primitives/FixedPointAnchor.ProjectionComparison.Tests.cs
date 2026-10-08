using FixedMathSharp.Geometry;
using Xunit;

namespace FixedMathSharp.Tests.Geometry.Primitives;

public sealed class FixedPointAnchorProjectionComparisonTests
{
    [Theory]
    [InlineData(2, 1, 1)]
    [InlineData(1, 2, -1)]
    [InlineData(1, 1, 0)]
    [InlineData(-2, -1, -1)]
    [InlineData(-1, -2, 1)]
    [InlineData(-1, -1, 0)]
    [InlineData(-1, 1, -1)]
    [InlineData(1, -1, 1)]
    [InlineData(0, 0, 0)]
    [InlineData(0, 1, -1)]
    [InlineData(1, 0, 1)]
    public void ProjectedOffsets_PreserveSignedOrdering(int left, int right, int expected)
    {
        FixedPointAnchor zero = Point(Fixed64.Zero);
        Assert.Equal(expected, WidePointAnchor3d.CompareProjectedOffsets(
            Point((Fixed64)left), zero, Point((Fixed64)right), zero, Vector3d.Right));
    }

    [Fact]
    public void ProjectedOffsets_RankBeyondScalarRangeAndRetainOneRawDifferences()
    {
        FixedPointAnchor high = Point(Fixed64.MaxValue);
        FixedPointAnchor low = Point(Fixed64.MinValue);
        FixedPointAnchor aboveLow = Point(Fixed64.FromRaw(long.MinValue + 1));
        Assert.Equal(1, WidePointAnchor3d.CompareProjectedOffsets(
            high, low, high, aboveLow, Vector3d.Right));
        Assert.Equal(-1, WidePointAnchor3d.CompareProjectedOffsets(
            low, high, aboveLow, high, Vector3d.Right));
        Assert.Equal(0, WidePointAnchor3d.CompareProjectedOffsets(
            high, low, low, high, Vector3d.Zero));
    }

    [Fact]
    public void ProjectedOffsets_IncludeSecondAnchorExactTermAndLocalTranslation()
    {
        FixedPointAnchor zero = Point(Fixed64.Zero);
        FixedPointAnchor exact = new FixedPointAnchor(
            Vector3d.Zero, FixedQuaternion.Identity, Vector3d.Zero, Vector3d.Zero,
            FixedPointAnchorTerm3d.CreateRadialSupport(
                new Vector3d(Fixed64.FromRaw(1), Fixed64.Zero, Fixed64.Zero),
                Fixed64.Half, Vector3d.Zero))
            .WithLocalTranslation(Vector3d.Right);
        Assert.Equal(-1, WidePointAnchor3d.CompareProjectedOffsets(
            zero, exact, zero, Point(Fixed64.One), Vector3d.Right));
        Assert.Equal(1, WidePointAnchor3d.CompareProjectedOffsets(
            exact, zero, Point(Fixed64.One), zero, Vector3d.Right));
    }

    [Fact]
    public void ProjectedOffsets_RetainSubRawRotationAcrossDifferentDenominators()
    {
        FixedPointAnchor zero = Point(Fixed64.Zero);
        FixedQuaternion rotation = new FixedQuaternion(
            Fixed64.Zero, Fixed64.Zero, (Fixed64)3, (Fixed64)4).Normalized;
        FixedPointAnchor rotated = new(Vector3d.Zero, rotation,
            new Vector3d(Fixed64.FromRaw(1), Fixed64.Zero, Fixed64.Zero));
        Assert.True(rotated.TryGetProjectedOffsetFrom(zero, Vector3d.Right, out Fixed64 rounded));
        Assert.Equal(Fixed64.Zero, rounded);
        Assert.Equal(1, WidePointAnchor3d.CompareProjectedOffsets(
            rotated, zero, zero, zero, Vector3d.Right));
        Assert.Equal(-1, WidePointAnchor3d.CompareProjectedOffsets(
            zero, rotated, zero, zero, Vector3d.Right));
    }

    private static FixedPointAnchor Point(Fixed64 x) =>
        new(new Vector3d(x, Fixed64.Zero, Fixed64.Zero), FixedQuaternion.Identity, Vector3d.Zero);

    [Fact]
    public void ProjectedOffsets_CompareDifferentAxesWithoutMixingDirections()
    {
        FixedPointAnchor zero = Point(Fixed64.Zero);
        FixedPointAnchor horizontal = Point(Fixed64.One);
        FixedPointAnchor vertical = new(new Vector3d(0, 2, 0),
            FixedQuaternion.Identity, Vector3d.Zero);
        Assert.Equal(-1, WidePointAnchor3d.CompareProjectedOffsets(
            horizontal, zero, vertical, zero, Vector3d.Right, Vector3d.Up));
        Assert.Equal(1, WidePointAnchor3d.CompareProjectedOffsets(
            vertical, zero, horizontal, zero, Vector3d.Up, Vector3d.Right));
    }
}
