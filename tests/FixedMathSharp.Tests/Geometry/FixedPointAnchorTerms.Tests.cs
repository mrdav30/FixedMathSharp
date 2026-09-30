using FixedMathSharp.Geometry;
using Xunit;

namespace FixedMathSharp.Tests.Geometry;

public sealed class FixedPointAnchorTermsTests
{
    // Pin both tie signs and cancellation of products and sums beyond a
    // signed machine word; noncardinal axes can also incur double rounding.
    [Theory]
    [InlineData(1L, 1L, 0L, 0L, 1, 8589934592L)]
    [InlineData(1L, 1L, 0L, 0L, -1, -8589934592L)]
    [InlineData(3L, 3L, 2L, 2L, 1, -8589934592L)]
    [InlineData(3L, 3L, -2L, -2L, -1, 8589934592L)]
    [InlineData(long.MaxValue, long.MaxValue, 4611686018427387904L, 4611686018427387904L, 1, -8589934592L)]
    [InlineData(long.MaxValue, long.MaxValue, -4611686018427387904L, -4611686018427387904L, -1, 8589934592L)]
    public void CenteredSupport_RetainsExactResidualAfterWideCancellation(
        long length, long radius, long axialOffset, long radialOffset, int sign, long expected)
    {
        Vector2d axis = new((Fixed64)sign, Fixed64.Zero);
        Vector2d radial = new(Fixed64.Half * sign, Fixed64.Zero);
        var axial = new Vector2d(Fixed64.FromRaw(axialOffset), Fixed64.Zero);
        var roundedRadial = new Vector2d(Fixed64.FromRaw(radialOffset), Fixed64.Zero);
        FixedPointAnchorTerm2d planar = FixedPointAnchorTerm2d.CreateCenteredAxisSupport(
            axis, Fixed64.FromRaw(length), radial, Fixed64.FromRaw(radius), axial, roundedRadial);
        FixedPointAnchorTerm3d spatial = FixedPointAnchorTerm3d.CreateCenteredAxisSupport(
            new Vector3d(axis.X, Fixed64.Zero, Fixed64.Zero), Fixed64.FromRaw(length),
            new Vector3d(radial.X, Fixed64.Zero, Fixed64.Zero), Fixed64.FromRaw(radius),
            new Vector3d(axial.X, Fixed64.Zero, Fixed64.Zero),
            new Vector3d(roundedRadial.X, Fixed64.Zero, Fixed64.Zero));
        Assert.Equal(expected, planar.X);
        Assert.Equal(0L, planar.Y);
        Assert.Equal(expected, spatial.X);
        Assert.Equal(0L, spatial.Y);
        Assert.Equal(0L, spatial.Z);
    }

    [Fact]
    public void CenteredSupport_RetainsNoncardinalAxialDoubleRounding()
    {
        Vector2d axis = new(Fixed64.FromFraction(4, 5), Fixed64.FromFraction(3, 5));
        FixedPointAnchor2d planar = FixedSegment2d.GetCenteredCapsuleSupportAnchor(
            Vector2d.Zero, Fixed64.Zero, axis, Fixed64.FromRaw(3L), Fixed64.FromRaw(2L), axis);
        Vector3d spatialAxis = new(axis.X, axis.Y, Fixed64.Zero);
        FixedPointAnchor spatial = WideGeometry.GetCenteredCapsuleSupportAnchor(
            Vector3d.Zero, FixedQuaternion.Identity, spatialAxis,
            Fixed64.FromRaw(3L), Fixed64.FromRaw(2L), spatialAxis);

        // X: 7*3435973837 - 8*4294967296; Y: 7*2576980378 - 4*4294967296.
        Assert.Equal(-10307921509L, planar.ExactLocalTerm.X);
        Assert.Equal(858993462L, planar.ExactLocalTerm.Y);
        Assert.Equal(planar.ExactLocalTerm.X, spatial.ExactLocalTerm.X);
        Assert.Equal(planar.ExactLocalTerm.Y, spatial.ExactLocalTerm.Y);
        Assert.Equal(0L, spatial.ExactLocalTerm.Z);
        Assert.True(planar.TryGetPoint(out Vector2d planarPoint));
        Assert.Equal(new Vector2d(Fixed64.FromRaw(3L), Fixed64.FromRaw(2L)), planarPoint);
        Assert.True(spatial.TryGetPoint(out Vector3d spatialPoint));
        Assert.Equal(new Vector3d(planarPoint.X, planarPoint.Y, Fixed64.Zero), spatialPoint);
    }
}
