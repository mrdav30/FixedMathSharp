//=======================================================================
// MIT License, Copyright (c) 2024-present David Oravsky (mrdav30)
// See LICENSE file in the project root for full license information.
//=======================================================================
using FixedMathSharp.Geometry;
using Xunit;

namespace FixedMathSharp.Tests;

public sealed class FixedTriangleCapsuleSlabFeatureTests
{
    [Theory]
    [InlineData(17179869184L)]
    [InlineData(long.MaxValue)]
    public void StraightRim_ObliqueTriangleEdge_SelectsFreeCoreCoordinate(long coreRaw)
    {
        // In Y/Z, the edge direction (4,-3) crosses the unit square's top
        // corner. q=(1,13/16,3/4) and p=(1,1,1) differ by (0,3,4)/16.
        // The centered box [-2,2]x[-1,1]x[-1,1] lies inside the slab and has
        // this same minimum 5/16, so no rounded-end direction can improve it.
        // Lengthening the core preserves this attained lower bound while
        // forcing endpoint arithmetic far beyond the centered fixture scale.
        var point = new Vector3d(Fixed64.One, Fixed64.FromFraction(13, 16), Fixed64.FromFraction(3, 4));
        Vector3d edge = new(1, 4, -3);
        var triangle = new FixedTriangle(point - edge, point + edge, point + new Vector3d(0, 3, 4));
        Assert.True(triangle.TryGetCenteredCapsuleSlabContact(Vector3d.Zero, FixedQuaternion.Identity,
            Vector3d.Zero, Fixed64.Zero, Vector2d.Right, Fixed64.FromRaw(coreRaw), Fixed64.One, Fixed64.One,
            out FixedContactAnchors contact));
        Assert.Equal(new Vector3d(Fixed64.Zero, -Fixed64.FromFraction(3, 5), -Fixed64.FromFraction(4, 5)), contact.Normal);
        Assert.Equal(Fixed64.FromFraction(5, 16), contact.Depth);
        Assert.True(contact.FirstAnchor.TryGetPoint(out Vector3d first));
        Assert.True(contact.SecondAnchor.TryGetPoint(out Vector3d second));
        Assert.Equal(point, first);
        Assert.Equal(new Vector3d(1, 1, 1), second);
    }

    [Theory]
    [InlineData(-1L, true)]
    [InlineData(0L, true)]
    [InlineData(1L, false)]
    public void StraightRim_TouchDistinguishesOneRawAxialGap(long rawY, bool overlaps)
    {
        Vector3d point = new(1, 1, 1), edge = new(1, 4, -3);
        var triangle = new FixedTriangle(point - edge, point + edge, point + new Vector3d(0, 3, 4));
        Assert.Equal(overlaps, triangle.TryGetCenteredCapsuleSlabContact(
            new Vector3d(Fixed64.Zero, Fixed64.FromRaw(rawY), Fixed64.Zero), FixedQuaternion.Identity,
            Vector3d.Zero, Fixed64.Zero, Vector2d.Right, (Fixed64)4, Fixed64.One, Fixed64.One,
            out FixedContactAnchors contact));
        if (rawY == 0)
        {
            Assert.Equal(Fixed64.Zero, contact.Depth);
            Assert.True(contact.FirstAnchor.TryGetPoint(out Vector3d first));
            Assert.True(contact.SecondAnchor.TryGetPoint(out Vector3d second));
            Assert.Equal(point, first);
            Assert.Equal(first, second);
        }
    }

    [Theory]
    [InlineData(-1, 0, false)]
    [InlineData(-1, 1, false)]
    [InlineData(-1, 2, false)]
    [InlineData(-1, 0, true)]
    [InlineData(-1, 1, true)]
    [InlineData(-1, 2, true)]
    [InlineData(1, 0, false)]
    [InlineData(1, 1, false)]
    [InlineData(1, 2, false)]
    [InlineData(1, 0, true)]
    [InlineData(1, 1, true)]
    [InlineData(1, 2, true)]
    public void RoundedEnd_InteriorEdgeRimRoot_RetainsExactDepthAndPairedWitnesses(int side, int firstVertex, bool reverse)
    {
        // A cylinder centered at (+/-1,0,0) is contained in this stadium prism.
        // Its known minimum supports the same endpoint of the whole prism.
        // Winding and cyclic reindexing must not change this unique contact.
        FixedTriangle source = InteriorRimTriangle();
        Vector3d a = firstVertex == 0 ? source.A : firstVertex == 1 ? source.B : source.C;
        Vector3d b = firstVertex == 0 ? source.B : firstVertex == 1 ? source.C : source.A;
        Vector3d c = firstVertex == 0 ? source.C : firstVertex == 1 ? source.A : source.B;
        a.X *= side; b.X *= side; c.X *= side;
        var triangle = reverse ? new FixedTriangle(a, c, b) : new FixedTriangle(a, b, c);
        Assert.True(triangle.TryGetCenteredCapsuleSlabContact(
            new Vector3d(side, 0, 0), FixedQuaternion.Identity, Vector3d.Zero, Fixed64.Zero,
            Vector2d.Right, Fixed64.Two, (Fixed64)5, (Fixed64)5, out FixedContactAnchors contact));

        Assert.Equal(new Vector3d(-side * Fixed64.FromFraction(3, 13), -Fixed64.FromFraction(12, 13),
            -Fixed64.FromFraction(4, 13)), contact.Normal);
        Assert.Equal(Fixed64.FromFraction(13, 256), contact.Depth);
        Assert.False(contact.DepthIsClamped);
        Assert.True(contact.FirstAnchor.TryGetPoint(out Vector3d first));
        Assert.True(contact.SecondAnchor.TryGetPoint(out Vector3d second));
        Assert.Equal(new Vector3d(side * Fixed64.FromFraction(1021, 256), Fixed64.FromFraction(317, 64),
            Fixed64.FromFraction(255, 64)), first);
        Assert.Equal(new Vector3d(side * 4, 5, 4), second);
    }

    [Theory]
    [InlineData(-1L, true)]
    [InlineData(0L, true)]
    [InlineData(1L, false)]
    public void RoundedEnd_RimTouch_DistinguishesOneRawAxialDisplacement(long rawY, bool overlaps)
    {
        var origin = new Vector3d(Fixed64.One + Fixed64.FromFraction(3, 256),
            Fixed64.FromFraction(12, 256) + Fixed64.FromRaw(rawY), Fixed64.FromFraction(4, 256));
        Assert.Equal(overlaps, InteriorRimTriangle().TryGetCenteredCapsuleSlabContact(
            origin, FixedQuaternion.Identity, Vector3d.Zero, Fixed64.Zero,
            Vector2d.Right, Fixed64.Two, (Fixed64)5, (Fixed64)5, out FixedContactAnchors contact));
        if (rawY == 0)
        {
            Assert.Equal(Fixed64.Zero, contact.Depth);
            Assert.True(contact.FirstAnchor.TryGetPoint(out Vector3d first));
            Assert.True(contact.SecondAnchor.TryGetPoint(out Vector3d second));
            Assert.Equal(new Vector3d(4, 5, 4), first);
            Assert.Equal(first, second);
        }
    }

    [Fact]
    public void StraightSide_UsesFreeCoreAndAxialCoordinatesForPairedWitnesses()
    {
        // The supporting triangle is offset along the core. Its contact is
        // on the straight side, not an arbitrarily selected rounded endpoint.
        var triangle = new FixedTriangle(new Vector3d(Fixed64.One, (Fixed64)(-2), Fixed64.Half),
            new Vector3d((Fixed64)3, (Fixed64)(-2), Fixed64.Half),
            new Vector3d(Fixed64.Two, Fixed64.Two, Fixed64.Half));
        Assert.True(triangle.TryGetCenteredCapsuleSlabContact(Vector3d.Zero, FixedQuaternion.Identity,
            Vector3d.Zero, Fixed64.Zero, Vector2d.Right, (Fixed64)4, Fixed64.One, Fixed64.One,
            out FixedContactAnchors contact));
        Assert.Equal(Vector3d.Backward, contact.Normal);
        Assert.Equal(Fixed64.Half, contact.Depth);
        Assert.True(contact.FirstAnchor.TryGetPoint(out Vector3d first));
        Assert.True(contact.SecondAnchor.TryGetPoint(out Vector3d second));
        Assert.Equal(Fixed64.Half, first.Z);
        Assert.Equal(Fixed64.One, second.Z);
        Assert.Equal(first.X, second.X);
        Assert.Equal(first.Y, second.Y);
        FixedMathTestHelper.AssertWithinRange(first.X, Fixed64.One, Fixed64.Two);
        FixedMathTestHelper.AssertWithinRange(first.Y, -Fixed64.One, Fixed64.One);
        // Left and right edges are x=1+(y+2)/4 and x=3-(y+2)/4.
        Assert.True(first.X >= Fixed64.One + (first.Y + Fixed64.Two) / (Fixed64)4);
        Assert.True(first.X <= (Fixed64)3 - (first.Y + Fixed64.Two) / (Fixed64)4);
    }

    [Fact]
    public void CapInteriorBetweenEndDisks_RetainsMatchedPlanarCoordinates()
    {
        // This triangle lies over the middle of a long core, outside both
        // endpoint disks. Independent end-cylinder contacts cannot own it.
        var triangle = new FixedTriangle(new Vector3d(Fixed64.Two, Fixed64.Half, -Fixed64.Half),
            new Vector3d((Fixed64)3, Fixed64.Half, -Fixed64.Half),
            new Vector3d(Fixed64.Two, Fixed64.Half, Fixed64.Half));
        Assert.True(triangle.TryGetCenteredCapsuleSlabContact(Vector3d.Zero, FixedQuaternion.Identity,
            Vector3d.Zero, Fixed64.Zero, Vector2d.Right, (Fixed64)10, Fixed64.One, Fixed64.One,
            out FixedContactAnchors contact));
        Assert.Equal(Vector3d.Down, contact.Normal);
        Assert.Equal(Fixed64.Half, contact.Depth);
        Assert.True(contact.FirstAnchor.TryGetPoint(out Vector3d first));
        Assert.True(contact.SecondAnchor.TryGetPoint(out Vector3d second));
        Assert.Equal(Fixed64.Half, first.Y);
        Assert.Equal(Fixed64.One, second.Y);
        Assert.Equal(first.X, second.X);
        Assert.Equal(first.Z, second.Z);
        FixedMathTestHelper.AssertWithinRange(first.X, Fixed64.Two, (Fixed64)3);
        FixedMathTestHelper.AssertWithinRange(first.Z, -Fixed64.Half, Fixed64.Half);
        Assert.True(first.X + first.Z <= Fixed64.FromFraction(5, 2));
    }

    private static FixedTriangle InteriorRimTriangle() => new(
        new Vector3d(Fixed64.FromFraction(1021, 256), Fixed64.FromFraction(309, 64), Fixed64.FromFraction(231, 64)),
        new Vector3d(Fixed64.FromFraction(509, 256), Fixed64.FromFraction(325, 64), Fixed64.FromFraction(279, 64)),
        new Vector3d(Fixed64.FromFraction(813, 256), Fixed64.FromFraction(365, 64), Fixed64.FromFraction(271, 64)));

    [Theory]
    [InlineData(-1, 1L)]
    [InlineData(-1, 3L)]
    [InlineData(-1, 5L)]
    [InlineData(-1, 7L)]
    [InlineData(1, 1L)]
    [InlineData(1, 3L)]
    [InlineData(1, 5L)]
    [InlineData(1, 7L)]
    public void RoundedEndTouch_OddRawCoreRoundsCombinedCurvedSupportOnce(int side, long coreRaw)
    {
        // The plane 4X+3Z=7 raw supports the endpoint disk: 4*(1/2)+5=7.
        // Its exact support (1.3 raw, Y, .6 raw) rounds to (1 raw,Y,1 raw).
        // Rounding the radial X first produces 1+.5 raw, incorrectly rounding
        // the final anchor to 2 raw despite an exactly touching triangle.
        // Translating to subsequent odd half-core endpoints exercises both
        // tie parities, on both mirrored rounded ends.
        Fixed64 raw = Fixed64.MinIncrement;
        Fixed64 x = Fixed64.FromRaw(side * ((coreRaw + 1) / 2));
        var triangle = new FixedTriangle(new Vector3d(x, (Fixed64)(-2), raw),
            new Vector3d(x, Fixed64.Two, raw),
            new Vector3d((Fixed64)(side * 3) + x, Fixed64.Zero, (Fixed64)(-4) + raw));
        Assert.False(triangle.IsDegenerate);
        Assert.True(triangle.TryGetCenteredCapsuleSlabContact(Vector3d.Zero, FixedQuaternion.Identity,
            Vector3d.Zero, Fixed64.Zero, Vector2d.Right, Fixed64.FromRaw(coreRaw), raw, Fixed64.One,
            out FixedContactAnchors contact));
        Assert.Equal(Fixed64.Zero, contact.Depth);
        Assert.Equal(new Vector3d(-side * Fixed64.FromFraction(4, 5), Fixed64.Zero,
            -Fixed64.FromFraction(3, 5)), contact.Normal);
        Assert.True(contact.FirstAnchor.TryGetPoint(out Vector3d first));
        Assert.True(contact.SecondAnchor.TryGetPoint(out Vector3d second));
        Assert.Equal(first, second);
        Assert.Equal(x, second.X);
        Assert.Equal(raw, second.Z);
    }

    [Theory]
    [InlineData(-1, 1L, false)]
    [InlineData(1, 1L, false)]
    [InlineData(-1, 3L, false)]
    [InlineData(1, 3L, false)]
    [InlineData(-1, 1L, true)]
    [InlineData(1, 1L, true)]
    public void RoundedRimRootTouch_RoundsCoreAndCurvedSupportTogether(int side, long coreRaw, bool roundUp)
    {
        Fixed64 raw = Fixed64.MinIncrement;
        Vector3d outward = roundUp ? new Vector3d(side * 10, 7, 24) : new Vector3d(side * 6, 7, 8);
        Vector3d edge = roundUp ? new Vector3d(side * 23, -26, -2) : new Vector3d(side, -10, 8);
        var center = new Vector3d(roundUp ? Fixed64.Zero : Fixed64.FromRaw(side * ((coreRaw + 1) / 2)),
            Fixed64.One + raw, roundUp ? raw : Fixed64.Zero);
        var triangle = new FixedTriangle(center - edge, center + edge, center + outward);
        // outward·edge=0. The endpoint's exact rim support lies on AB:
        // for (6,7,8), p=((core/2+.6)raw,1,.8raw);
        // for (10,7,24), p=((.5+5/13)raw,1,12/13raw).
        // Thus this is a nonmeridional edge/rim touch, not a face shortcut.
        Assert.False(triangle.IsDegenerate);
        Assert.True(triangle.TryGetCenteredCapsuleSlabContact(Vector3d.Zero, FixedQuaternion.Identity,
            Vector3d.Zero, Fixed64.Zero, Vector2d.Right, Fixed64.FromRaw(coreRaw), raw, Fixed64.One,
            out FixedContactAnchors contact));
        Assert.Equal(Fixed64.Zero, contact.Depth);
        Assert.False(contact.DepthIsClamped);
        Assert.True(contact.FirstAnchor.TryGetPoint(out Vector3d first));
        Assert.True(contact.SecondAnchor.TryGetPoint(out Vector3d second));
        Assert.Equal(first, second);
        Assert.Equal(new Vector3d(Fixed64.FromRaw(side * (roundUp ? 1 : (coreRaw + 1) / 2)),
            Fixed64.One, raw), second);
        // The public normalization rounds its square root before division;
        // the contact rounds each exact algebraic component only once.
        Vector3d expected = (-outward).Normalized;
        FixedMathTestHelper.AssertWithinRange(contact.Normal.X, expected.X - raw, expected.X + raw);
        FixedMathTestHelper.AssertWithinRange(contact.Normal.Y, expected.Y - raw, expected.Y + raw);
        FixedMathTestHelper.AssertWithinRange(contact.Normal.Z, expected.Z - raw, expected.Z + raw);
    }

    [Theory]
    [InlineData(-1)]
    [InlineData(1)]
    public void RoundedRimRootTouch_ChartEndingOnSeamRetainsInteriorRoot(int side)
    {
        Fixed64 raw = Fixed64.MinIncrement;
        Vector3d outward = new(side * 6, 7, 8), edge = new(-side, -6, 6);
        var center = new Vector3d(side * 4 * raw, Fixed64.One + 3 * raw, raw);
        var triangle = new FixedTriangle(center - edge, center + edge, center + outward);
        // outward·edge=0. Exact rim support (side*3.5 raw,1,4 raw)
        // lies at center+edge*raw/2 and rounds both anchors to an even X.
        // The admitted root is t=7/8, although its chart's core projection
        // is proportional to 1-t and reaches the seam at t=1.
        Assert.True(triangle.TryGetCenteredCapsuleSlabContact(Vector3d.Zero, FixedQuaternion.Identity,
            Vector3d.Zero, Fixed64.Zero, Vector2d.Right, raw, 5 * raw, Fixed64.One,
            out FixedContactAnchors contact));
        Assert.Equal(Fixed64.Zero, contact.Depth);
        Assert.False(contact.DepthIsClamped);
        Assert.True(contact.FirstAnchor.TryGetPoint(out Vector3d first));
        Assert.True(contact.SecondAnchor.TryGetPoint(out Vector3d second));
        Assert.Equal(new Vector3d(side * 4 * raw, Fixed64.One, 4 * raw), second);
        Assert.Equal(first, second);
        Vector3d expected = (-outward).Normalized;
        FixedMathTestHelper.AssertWithinRange(contact.Normal.X, expected.X - raw, expected.X + raw);
        FixedMathTestHelper.AssertWithinRange(contact.Normal.Y, expected.Y - raw, expected.Y + raw);
        FixedMathTestHelper.AssertWithinRange(contact.Normal.Z, expected.Z - raw, expected.Z + raw);
    }

    [Fact]
    public void RoundedRimRootTouch_PreservesExactlyZeroRadialComponent()
    {
        Vector3d point = new(0, 1, 2), edge = new(1, 4, -3);
        var triangle = new FixedTriangle(point - edge, point + edge, point + new Vector3d(0, 3, 4));
        Assert.True(triangle.TryGetCenteredCapsuleSlabContact(Vector3d.Zero, FixedQuaternion.Identity,
            Vector3d.Zero, Fixed64.Zero, Vector2d.Forward, Fixed64.Two, Fixed64.One, Fixed64.One,
            out FixedContactAnchors contact));
        Assert.Equal(Fixed64.Zero, contact.Depth);
        Assert.Equal(new Vector3d(Fixed64.Zero, -Fixed64.FromFraction(3, 5), -Fixed64.FromFraction(4, 5)), contact.Normal);
        Assert.True(contact.FirstAnchor.TryGetPoint(out Vector3d first));
        Assert.True(contact.SecondAnchor.TryGetPoint(out Vector3d second));
        Assert.Equal(point, first);
        Assert.Equal(first, second);
    }

    [Theory]
    [InlineData(-1, 2L)]
    [InlineData(1, 2L)]
    [InlineData(-1, 4L)]
    [InlineData(1, 4L)]
    public void RoundedRimRoot_ExactHalfTieUsesCombinedCoordinateParity(int side, long coreRaw)
    {
        Fixed64 raw = Fixed64.MinIncrement;
        var center = new Vector3d(Fixed64.FromRaw(side * (coreRaw / 2 + 2)), -Fixed64.One, Fixed64.Zero);
        Vector3d edge = new(0, 1, 2);
        var triangle = new FixedTriangle(center - edge, center + edge, center + side * Vector3d.Right);
        // The exact root has outward n=(side/4,-sqrt(3)/2,sqrt(3)/4).
        // Its endpoint-disk support X is core/2+2.5 raw: the final integer
        // parity, not the radial component's parity, determines the tie.
        Assert.True(triangle.TryGetCenteredCapsuleSlabContact(Vector3d.Zero, FixedQuaternion.Identity,
            Vector3d.Zero, Fixed64.Zero, Vector2d.Right, Fixed64.FromRaw(coreRaw),
            Fixed64.FromRaw(5), Fixed64.One, out FixedContactAnchors contact));
        Assert.Equal(Fixed64.FromRaw(2), contact.Depth);
        Assert.Equal(new Vector3d(Fixed64.FromRaw(-side * 1073741824L),
            Fixed64.FromRaw(3719550787L), Fixed64.FromRaw(-1859775393L)), contact.Normal);
        Assert.True(contact.FirstAnchor.TryGetPoint(out Vector3d first));
        Assert.True(contact.SecondAnchor.TryGetPoint(out Vector3d second));
        Assert.Equal(new Vector3d(Fixed64.FromRaw(side * (coreRaw / 2 + 2)),
            -Fixed64.One + raw * 2, raw * 3), first);
        Assert.Equal(new Vector3d(Fixed64.FromRaw(side * 4), -Fixed64.One, raw * 4), second);
    }

    [Fact]
    public void ZeroRadiusCoreRectangle_HasPositiveDepthAgainstTransverseTriangle()
    {
        // The Y/Z triangle contains a radius>1 disk around (1,0,0).
        // Its difference with the X/Y core rectangle therefore contains a
        // radius1 ball around zero, attained by the rectangle's X endpoint.
        var triangle = new FixedTriangle(new Vector3d(1, -4, -4),
            new Vector3d(1, 4, -4), new Vector3d(1, 0, 4));
        Assert.True(triangle.TryGetCenteredCapsuleSlabContact(Vector3d.Zero, FixedQuaternion.Identity,
            Vector3d.Zero, Fixed64.Zero, Vector2d.Right, (Fixed64)4, Fixed64.Zero, Fixed64.One,
            out FixedContactAnchors contact));
        Assert.Equal(Vector3d.Left, contact.Normal);
        Assert.Equal(Fixed64.One, contact.Depth);
        Assert.True(contact.FirstAnchor.TryGetPoint(out Vector3d first));
        Assert.True(contact.SecondAnchor.TryGetPoint(out Vector3d second));
        Assert.Equal(Fixed64.One, first.X);
        Assert.Equal(Fixed64.Two, second.X);
        Assert.Equal(first.Y, second.Y);
        Assert.Equal(Fixed64.Zero, first.Z);
        Assert.Equal(first.Z, second.Z);
        FixedMathTestHelper.AssertWithinRange(first.Y, -Fixed64.One, Fixed64.One);
    }
}
