using System;
using Xunit;

namespace FixedMathSharp.Tests;

public class FixedMathGeometryTests
{
    [Theory]
    [InlineData(3)]
    [InlineData(-3)]
    public void TryGetCircleCrossSectionRadius_ReturnsRightTriangleLeg(int offset)
    {
        bool found = FixedMath.TryGetCircleCrossSectionRadius(
            (Fixed64)5,
            (Fixed64)offset,
            out Fixed64 crossSectionRadius);

        Assert.True(found);
        Assert.Equal((Fixed64)4, crossSectionRadius);
    }

    [Fact]
    public void TryGetCircleCrossSectionRadius_DistinguishesTangentOutsideAndInvalidRadius()
    {
        Assert.True(FixedMath.TryGetCircleCrossSectionRadius(
            (Fixed64)5,
            (Fixed64)5,
            out Fixed64 tangentRadius));
        Assert.Equal(Fixed64.Zero, tangentRadius);

        Assert.False(FixedMath.TryGetCircleCrossSectionRadius(
            (Fixed64)5,
            (Fixed64)5 + Fixed64.FromRaw(1L),
            out Fixed64 outsideRadius));
        Assert.Equal(Fixed64.Zero, outsideRadius);

        Assert.Throws<ArgumentOutOfRangeException>(() =>
            FixedMath.TryGetCircleCrossSectionRadius(-Fixed64.One, Fixed64.Zero, out _));
    }

    [Fact]
    public void TryGetCircleCrossSectionRadius_PreservesExtremeDifferenceOfSquares()
    {
        Assert.True(FixedMath.TryGetCircleCrossSectionRadius(
            (Fixed64)100_000,
            (Fixed64)60_000,
            out Fixed64 ordinaryExtreme));
        Assert.Equal((Fixed64)80_000, ordinaryExtreme);

        Assert.True(FixedMath.TryGetCircleCrossSectionRadius(
            Fixed64.MaxValue,
            Fixed64.FromRaw(long.MaxValue - 1L),
            out Fixed64 maximumRaw));
        Assert.Equal(Fixed64.One, maximumRaw);

        Assert.False(FixedMath.TryGetCircleCrossSectionRadius(
            Fixed64.MaxValue,
            Fixed64.MinValue,
            out Fixed64 minimumOffset));
        Assert.Equal(Fixed64.Zero, minimumOffset);
    }

    [Fact]
    public void TryGetSphereSlabCrossSectionRadius_UsesNearestPlaneInsideSlab()
    {
        Assert.True(FixedMath.TryGetSphereSlabCrossSectionRadius(
            (Fixed64)4,
            (Fixed64)5,
            Fixed64.Zero,
            Fixed64.One,
            out Fixed64 projectedRadius));
        Assert.Equal((Fixed64)4, projectedRadius);

        Assert.True(FixedMath.TryGetSphereSlabCrossSectionRadius(
            (Fixed64)(-4),
            (Fixed64)5,
            Fixed64.Zero,
            Fixed64.One,
            out Fixed64 mirroredProjectedRadius));
        Assert.Equal((Fixed64)4, mirroredProjectedRadius);

        Assert.True(FixedMath.TryGetSphereSlabCrossSectionRadius(
            Fixed64.One,
            (Fixed64)5,
            Fixed64.Zero,
            Fixed64.One,
            out Fixed64 insideRadius));
        Assert.Equal((Fixed64)5, insideRadius);
    }

    [Fact]
    public void TryGetSphereSlabCrossSectionRadius_PreservesOppositeDomainSeparation()
    {
        Assert.False(FixedMath.TryGetSphereSlabCrossSectionRadius(
            Fixed64.MaxValue,
            Fixed64.MaxValue,
            Fixed64.MinValue,
            Fixed64.One,
            out Fixed64 missedRadius));
        Assert.Equal(Fixed64.Zero, missedRadius);

        Assert.True(FixedMath.TryGetSphereSlabCrossSectionRadius(
            (Fixed64)6,
            (Fixed64)5,
            Fixed64.Zero,
            Fixed64.One,
            out Fixed64 tangentRadius));
        Assert.Equal(Fixed64.Zero, tangentRadius);
    }

    [Fact]
    public void TryGetSphereSlabCrossSectionRadius_RejectsNegativeDimensions()
    {
        Assert.Throws<ArgumentOutOfRangeException>(() =>
            FixedMath.TryGetSphereSlabCrossSectionRadius(
                Fixed64.Zero,
                -Fixed64.One,
                Fixed64.Zero,
                Fixed64.One,
                out _));
        Assert.Throws<ArgumentOutOfRangeException>(() =>
            FixedMath.TryGetSphereSlabCrossSectionRadius(
                Fixed64.Zero,
                Fixed64.One,
                Fixed64.Zero,
                -Fixed64.One,
                out _));
    }
}
