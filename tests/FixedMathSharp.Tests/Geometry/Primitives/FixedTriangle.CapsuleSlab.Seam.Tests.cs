//=======================================================================
// MIT License, Copyright (c) 2024-present David Oravsky (mrdav30)
// See LICENSE file in the project root for full license information.
//=======================================================================
using FixedMathSharp.Geometry;
using Xunit;

namespace FixedMathSharp.Tests;

public sealed class FixedTriangleCapsuleSlabSeamTests
{
    [Theory]
    [InlineData(-1L)]
    [InlineData(1L)]
    public void CapTouch_DiagonalCoreDoesNotRoundProjectionAndResidualSeparately(long rawX)
    {
        // At A the exact core projection is (rawX/2,rawX/2), while
        // its radial residual is (rawX/2,-rawX/2). Rounding both first
        // erases one raw unit even though their sum A is representable.
        var point = new Vector3d(Fixed64.FromRaw(rawX), Fixed64.One, Fixed64.Zero);
        var triangle = new FixedTriangle(point, new Vector3d(1, 2, 0), new Vector3d(0, 2, 1));
        Fixed64 diagonal = Fixed64.FromRaw(3037000500L);
        Assert.True(triangle.TryGetCenteredCapsuleSlabContact(Vector3d.Zero, FixedQuaternion.Identity,
            Vector3d.Zero, Fixed64.Zero, new Vector2d(diagonal, diagonal), Fixed64.Two,
            Fixed64.One, Fixed64.One, out FixedContactAnchors contact));
        Assert.Equal(Vector3d.Down, contact.Normal);
        Assert.Equal(Fixed64.Zero, contact.Depth);
        Assert.False(contact.DepthIsClamped);
        Assert.True(contact.FirstAnchor.TryGetPoint(out Vector3d first));
        Assert.True(contact.SecondAnchor.TryGetPoint(out Vector3d second));
        Assert.Equal(point, first);
        Assert.Equal(first, second);
    }

    [Theory]
    [InlineData(0L)]
    [InlineData(1L)]
    public void StraightSideTouch_DoesNotRoundCoreAndRadialCoordinatesSeparately(long coreShift)
    {
        // The plane 4X-3Z=-5 raw is tangent to the radius-one-raw stadium.
        // Its first axial slice crosses AB at q=(-.5,-1,1) raw. Independent
        // rounding of core(.3,.4) and radial(-.8,.6) moves the second X by
        // one raw unit. The long odd edge keeps this triangle nondegenerate.
        const long scale = 1L << 32;
        const long odd = scale + 1L;
        var a = new Vector3d(Fixed64.FromRaw((-1L - 3L * odd) / 2L),
            Fixed64.FromRaw(-1L - 15L * odd), Fixed64.FromRaw(1L - 2L * odd));
        var b = new Vector3d(Fixed64.FromRaw((-1L + 3L * odd) / 2L),
            Fixed64.FromRaw(-1L + 15L * odd), Fixed64.FromRaw(1L + 2L * odd));
        var offset = new Vector3d(Fixed64.FromRaw(3L * coreShift), Fixed64.Zero, Fixed64.FromRaw(4L * coreShift));
        var triangle = new FixedTriangle(a + offset, b + offset, a - Vector3d.Up + offset);
        Assert.False(triangle.IsDegenerate);
        const long fifth = 858993459L;
        var axis = new Vector2d(Fixed64.FromRaw(3L * fifth), Fixed64.FromRaw(4L * fifth));
        Assert.True(triangle.TryGetCenteredCapsuleSlabContact(Vector3d.Zero, FixedQuaternion.Identity,
            Vector3d.Zero, Fixed64.Zero, axis, Fixed64.Two, Fixed64.MinIncrement,
            Fixed64.MinIncrement, out FixedContactAnchors contact));
        Assert.Equal(new Vector3d(Fixed64.FromFraction(4, 5), Fixed64.Zero, -Fixed64.FromFraction(3, 5)), contact.Normal);
        Assert.Equal(Fixed64.Zero, contact.Depth);
        Assert.False(contact.DepthIsClamped);
        Assert.True(contact.FirstAnchor.TryGetPoint(out Vector3d first));
        Assert.True(contact.SecondAnchor.TryGetPoint(out Vector3d second));
        Assert.Equal(first, second);
        FixedMathTestHelper.AssertWithinRange(first.Y, -Fixed64.MinIncrement, Fixed64.MinIncrement);
    }

    [Theory]
    [InlineData(-1)]
    [InlineData(1)]
    public void StraightRim_FaceSliceClipsBothOutsideCoreEndpoints(int side)
    {
        // The triangle/embedded-box SAT minimum is 5/16 on this face.
        // Its matched support slice has X endpoints +/-20/3, so neither
        // slice endpoint belongs to the core interval [-2,2].
        var point = new Vector3d(Fixed64.Zero, side * Fixed64.FromFraction(13, 16), side * Fixed64.FromFraction(3, 4));
        var triangle = new FixedTriangle(point + new Vector3d(-10, -side * 4, side * 3),
            point + new Vector3d(10, -side * 4, side * 3), point + new Vector3d(0, side * 8, -side * 6));
        Assert.True(triangle.TryGetCenteredCapsuleSlabContact(Vector3d.Zero, FixedQuaternion.Identity,
            Vector3d.Zero, Fixed64.Zero, Vector2d.Right, (Fixed64)4, Fixed64.One, Fixed64.One,
            out FixedContactAnchors contact));
        Assert.Equal(new Vector3d(Fixed64.Zero, -side * Fixed64.FromFraction(3, 5), -side * Fixed64.FromFraction(4, 5)), contact.Normal);
        Assert.Equal(Fixed64.FromFraction(5, 16), contact.Depth);
        Assert.True(contact.FirstAnchor.TryGetPoint(out Vector3d first));
        Assert.True(contact.SecondAnchor.TryGetPoint(out Vector3d second));
        Assert.Equal(point.Y, first.Y);
        Assert.Equal(point.Z, first.Z);
        Assert.Equal(first.X, second.X);
        Assert.Equal((Fixed64)side, second.Y);
        Assert.Equal((Fixed64)side, second.Z);
        FixedMathTestHelper.AssertWithinRange(first.X, (Fixed64)(-2), Fixed64.Two);
    }

    [Theory]
    [InlineData(-1)]
    [InlineData(1)]
    public void CapFace_UsesInteriorEdgeWitnessWhenVerticesAndCoreLineMiss(int side)
    {
        // Every vertex is outside the stadium. The core line meets the
        // triangle only beyond its endpoint; the nearest cap witness lies
        // inside AB. A radius-1/4 ball centered at (side*1.5,.75,0) lies
        // in the prism and its center lies in the triangle, proving depth.
        Fixed64 x = side * Fixed64.FromFraction(3, 2);
        Fixed64 y = Fixed64.FromFraction(3, 4);
        var triangle = new FixedTriangle(new Vector3d(x, y, (Fixed64)(-2)), new Vector3d(x, y, Fixed64.Two),
            new Vector3d((Fixed64)(side * 3), y, Fixed64.Zero));
        Assert.True(triangle.TryGetCenteredCapsuleSlabContact(Vector3d.Zero, FixedQuaternion.Identity,
            Vector3d.Zero, Fixed64.Zero, Vector2d.Right, Fixed64.Two, Fixed64.One, Fixed64.One,
            out FixedContactAnchors contact));
        Assert.Equal(Vector3d.Down, contact.Normal);
        Assert.Equal(Fixed64.FromFraction(1, 4), contact.Depth);
        Assert.True(contact.FirstAnchor.TryGetPoint(out Vector3d first));
        Assert.True(contact.SecondAnchor.TryGetPoint(out Vector3d second));
        Assert.Equal(y, first.Y);
        Assert.Equal(Fixed64.One, second.Y);
        Assert.Equal(first.X, second.X);
        Assert.Equal(first.Z, second.Z);
        Fixed64 outward = side * first.X;
        FixedMathTestHelper.AssertWithinRange(outward, Fixed64.FromFraction(3, 2), (Fixed64)3);
        Assert.True((Fixed64)3 * FixedMath.Abs(first.Z) <= (Fixed64)4 * ((Fixed64)3 - outward));
        Fixed64 radialX = outward - Fixed64.One;
        Assert.True(radialX * radialX + first.Z * first.Z <= Fixed64.One);
    }

    [Theory]
    [InlineData(-1)]
    [InlineData(1)]
    public void CapTouch_ClampsFreeCoreProjectionToItsEndpoint(int side)
    {
        Vector3d point = new(side * 3, 1, 0);
        var triangle = new FixedTriangle(point, new Vector3d(side * 4, 2, 0), new Vector3d(side * 3, 2, 1));
        Assert.True(triangle.TryGetCenteredCapsuleSlabContact(Vector3d.Zero, FixedQuaternion.Identity,
            Vector3d.Zero, Fixed64.Zero, Vector2d.Right, Fixed64.Two, Fixed64.Two, Fixed64.One,
            out FixedContactAnchors contact));
        Assert.Equal(Vector3d.Down, contact.Normal);
        Assert.Equal(Fixed64.Zero, contact.Depth);
        Assert.True(contact.FirstAnchor.TryGetPoint(out Vector3d first));
        Assert.True(contact.SecondAnchor.TryGetPoint(out Vector3d second));
        Assert.Equal(point, first);
        Assert.Equal(first, second);
    }

    [Theory]
    [InlineData(-1)]
    [InlineData(1)]
    public void StraightSide_ClipsCoreBoundaryWhenNoAxialBoundaryMeetsFeature(int side)
    {
        // AB is parallel to the core and strictly between both axial
        // boundaries. Its vertices are beyond both ends of the core.
        var triangle = new FixedTriangle(new Vector3d((Fixed64)(-3), Fixed64.Zero, side * Fixed64.Half),
            new Vector3d((Fixed64)3, Fixed64.Zero, side * Fixed64.Half), new Vector3d(0, 0, side * 3));
        Assert.True(triangle.TryGetCenteredCapsuleSlabContact(Vector3d.Zero, FixedQuaternion.Identity,
            Vector3d.Zero, Fixed64.Zero, Vector2d.Right, (Fixed64)4, Fixed64.One, Fixed64.One,
            out FixedContactAnchors contact));
        Assert.Equal(new Vector3d(0, 0, -side), contact.Normal);
        Assert.Equal(Fixed64.Half, contact.Depth);
        Assert.True(contact.FirstAnchor.TryGetPoint(out Vector3d first));
        Assert.True(contact.SecondAnchor.TryGetPoint(out Vector3d second));
        Assert.Equal(first.X, second.X);
        Assert.Equal(Fixed64.Zero, first.Y);
        Assert.Equal(first.Y, second.Y);
        Assert.Equal(side * Fixed64.Half, first.Z);
        Assert.Equal((Fixed64)side, second.Z);
        FixedMathTestHelper.AssertWithinRange(first.X, (Fixed64)(-2), Fixed64.Two);
    }

    [Fact]
    public void ZeroRadius_CoreRectangleRetainsClosedPairedContact()
    {
        var triangle = new FixedTriangle(new Vector3d(-3, -2, 0), new Vector3d(3, -2, 0), new Vector3d(0, 2, 0));
        Assert.True(triangle.TryGetCenteredCapsuleSlabContact(Vector3d.Zero, FixedQuaternion.Identity,
            Vector3d.Zero, Fixed64.Zero, Vector2d.Right, (Fixed64)4, Fixed64.Zero, Fixed64.One,
            out FixedContactAnchors contact));
        Assert.Equal(Fixed64.Zero, contact.Depth);
        Assert.False(contact.DepthIsClamped);
        Assert.Equal(Fixed64.One, FixedMath.Abs(contact.Normal.Z));
        Assert.True(contact.FirstAnchor.TryGetPoint(out Vector3d first));
        Assert.True(contact.SecondAnchor.TryGetPoint(out Vector3d second));
        Assert.Equal(first, second);
        Assert.Equal(Fixed64.Zero, first.Z);
        FixedMathTestHelper.AssertWithinRange(first.X, (Fixed64)(-2), Fixed64.Two);
        FixedMathTestHelper.AssertWithinRange(first.Y, -Fixed64.One, Fixed64.One);
        Assert.True((Fixed64)4 * FixedMath.Abs(first.X) <= (Fixed64)3 * (Fixed64.Two - first.Y));
    }

    [Fact]
    public void StraightRim_TouchAtSingleTriangleVertexRetainsThatWitness()
    {
        // In the Y/Z rectangle, only C touches the corner (1,1). The AB
        // seam axis supports C strictly, so a singleton support feature
        // must not be treated as an edge interpolation.
        Vector3d point = new(0, 1, 1);
        var triangle = new FixedTriangle(new Vector3d(0, 2, 0), new Vector3d(0, 0, 3), point);
        Assert.True(triangle.TryGetCenteredCapsuleSlabContact(Vector3d.Zero, FixedQuaternion.Identity,
            Vector3d.Zero, Fixed64.Zero, Vector2d.Right, (Fixed64)4, Fixed64.One, Fixed64.One,
            out FixedContactAnchors contact));
        Assert.Equal(Fixed64.Zero, contact.Depth);
        Assert.Equal(Fixed64.Zero, contact.Normal.X);
        Assert.True(contact.Normal.Y < Fixed64.Zero);
        Assert.True(contact.Normal.Z < Fixed64.Zero);
        Assert.True(contact.FirstAnchor.TryGetPoint(out Vector3d first));
        Assert.True(contact.SecondAnchor.TryGetPoint(out Vector3d second));
        Assert.Equal(point, first);
        Assert.Equal(first, second);
    }

    [Fact]
    public void CapFace_UsesOppositeEndpointAfterFirstProjectedWitnessMissesStadium()
    {
        // AB is Z=.5X+1. Projecting core endpoint +1 gives (.4,1.2),
        // outside the stadium; endpoint -1 gives (-1.2,.4), inside it.
        // A radius-.25 ball centered at the latter cap-plane point fits
        // the prism, proving the selected cap depth is globally minimal.
        Fixed64 y = Fixed64.FromFraction(3, 4);
        var triangle = new FixedTriangle(new Vector3d((Fixed64)(-4), y, -Fixed64.One),
            new Vector3d((Fixed64)4, y, (Fixed64)3), new Vector3d((Fixed64)3, y, (Fixed64)5));
        Assert.True(triangle.TryGetCenteredCapsuleSlabContact(Vector3d.Zero, FixedQuaternion.Identity,
            Vector3d.Zero, Fixed64.Zero, Vector2d.Right, Fixed64.Two, Fixed64.One, Fixed64.One,
            out FixedContactAnchors contact));
        Assert.Equal(Vector3d.Down, contact.Normal);
        Assert.Equal(Fixed64.FromFraction(1, 4), contact.Depth);
        Assert.True(contact.FirstAnchor.TryGetPoint(out Vector3d first));
        Assert.True(contact.SecondAnchor.TryGetPoint(out Vector3d second));
        Assert.Equal(y, first.Y);
        Assert.Equal(Fixed64.One, second.Y);
        Assert.Equal(first.X, second.X);
        Assert.Equal(first.Z, second.Z);
        Fixed64 tolerance = Fixed64.FromRaw(8);
        Assert.True(Fixed64.Two * first.Z - first.X - Fixed64.Two >= -tolerance);
        Assert.True((Fixed64)11 - Fixed64.Two * first.X - first.Z >= -tolerance);
        Assert.True((Fixed64)6 * first.X - (Fixed64)7 * first.Z + (Fixed64)17 >= -tolerance);
        Fixed64 radialX = first.X < -Fixed64.One ? first.X + Fixed64.One
            : first.X > Fixed64.One ? first.X - Fixed64.One : Fixed64.Zero;
        Assert.True(radialX * radialX + first.Z * first.Z <= Fixed64.One);
    }

    [Theory]
    [InlineData(-1)]
    [InlineData(1)]
    public void CapTouch_ZeroRadiusRetainsExactOddCoreEndpointIdentity(int side)
    {
        // AB is 2X+Z=one raw; the triangle lies on its positive side.
        // At Z=0 it meets the half-raw core only at one endpoint; mirroring
        // verifies the residual's sign as well as its retained identity.
        // Both displayed points round X to zero, but the exact endpoint
        // must remain distinguishable from a rounded-only anchor.
        Fixed64 epsilon = Fixed64.MinIncrement;
        var triangle = new FixedTriangle(new Vector3d((Fixed64)side, Fixed64.One, side * (epsilon - Fixed64.Two)),
            new Vector3d((Fixed64)(-side), Fixed64.One, side * (epsilon + Fixed64.Two)), new Vector3d(side, 1, side));
        Assert.True(triangle.TryGetCenteredCapsuleSlabContact(Vector3d.Zero, FixedQuaternion.Identity,
            Vector3d.Zero, Fixed64.Zero, Vector2d.Right, epsilon, Fixed64.Zero, Fixed64.One,
            out FixedContactAnchors contact));
        Assert.Equal(Vector3d.Down, contact.Normal);
        Assert.Equal(Fixed64.Zero, contact.Depth);
        Assert.True(contact.FirstAnchor.TryGetPoint(out Vector3d first));
        Assert.True(contact.SecondAnchor.TryGetPoint(out Vector3d second));
        Assert.Equal(Vector3d.Up, first);
        Assert.Equal(first, second);
        FixedPointAnchor anchor = contact.SecondAnchor;
        var roundedOnly = new FixedPointAnchor(anchor.Origin, anchor.Rotation, anchor.LocalPoint, anchor.LocalDisplacement);
        Assert.NotEqual(0, anchor.CompareLocalFeature(roundedOnly));
        Assert.True(anchor.TryGetScaledOffsetFrom(roundedOnly, Fixed64.Two, out Vector3d residual));
        Assert.Equal(new Vector3d(side * epsilon, Fixed64.Zero, Fixed64.Zero), residual);
    }

    [Fact]
    public void CapTouch_RetainsOnlyIntegralRadialCoordinatesAtCoreEndpoint()
    {
        // The positive core endpoint is C=(644245094.25,858993459) raw.
        // AB is 2X+Z=2147483650 raw, and its closest point to C is
        // q=C+(1,.5) raw. Retain C's quarter-raw X term with integral
        // radial X=1; radial Z=.5 instead uses the combined tie parity.
        const long fifth = 858993459L;
        Fixed64 line = Fixed64.FromRaw(2147483650L);
        var triangle = new FixedTriangle(new Vector3d(-Fixed64.One, Fixed64.One, line + Fixed64.Two),
            new Vector3d(Fixed64.One, Fixed64.One, line - Fixed64.Two),
            new Vector3d((Fixed64)3, Fixed64.One, line + (Fixed64)4));
        var axis = new Vector2d(Fixed64.FromRaw(3L * fifth), Fixed64.FromRaw(4L * fifth));
        Assert.True(triangle.TryGetCenteredCapsuleSlabContact(Vector3d.Zero, FixedQuaternion.Identity,
            Vector3d.Zero, Fixed64.Zero, axis, Fixed64.Half, Fixed64.One, Fixed64.One,
            out FixedContactAnchors contact));
        Assert.Equal(Vector3d.Down, contact.Normal);
        Assert.Equal(Fixed64.Zero, contact.Depth);
        Assert.True(contact.FirstAnchor.TryGetPoint(out Vector3d first));
        Assert.True(contact.SecondAnchor.TryGetPoint(out Vector3d second));
        Assert.Equal(new Vector3d(Fixed64.FromRaw(644245095L), Fixed64.One, Fixed64.FromRaw(858993460L)), first);
        Assert.Equal(first, second);
        FixedPointAnchor anchor = contact.SecondAnchor;
        var roundedOnly = new FixedPointAnchor(anchor.Origin, anchor.Rotation, anchor.LocalPoint, anchor.LocalDisplacement);
        Assert.True(anchor.TryGetScaledOffsetFrom(roundedOnly, (Fixed64)4, out Vector3d residual));
        Assert.Equal(new Vector3d(Fixed64.MinIncrement, Fixed64.Zero, Fixed64.Zero), residual);
    }
}
