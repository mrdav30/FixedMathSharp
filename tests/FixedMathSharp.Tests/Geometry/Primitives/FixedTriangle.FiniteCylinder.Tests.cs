//=======================================================================
// MIT License, Copyright (c) 2024-present David Oravsky (mrdav30)
// See LICENSE file in the project root for full license information.
//=======================================================================
using System;
using FixedMathSharp.Geometry;
using Xunit;

namespace FixedMathSharp.Tests;

public sealed partial class FixedTriangleFiniteCylinderTests
{
    [Fact]
    public void Cylinder_SideEdge_UsesWholeSupportingAxialSegment()
    {
        // The unit ball centered on q=(4,0,0) is inside the cylinder;
        // the left-facing support plane attains depth one. Projection onto
        // either cap endpoint would invent a tangential witness displacement.
        var triangle = new FixedTriangle(new Vector3d(4, -4, -4),
            new Vector3d(4, 4, 4), new Vector3d(8, 0, 0));

        Assert.True(triangle.TryGetCenteredFiniteCylinderContact(
            Vector3d.Zero, FixedQuaternion.Identity, Vector3d.Zero,
            FixedQuaternion.Identity, (Fixed64)10, (Fixed64)5, out FixedContactAnchors contact));

        Assert.Equal(Vector3d.Left, contact.Normal);
        Assert.Equal(Fixed64.One, contact.Depth);
        Assert.True(contact.FirstAnchor.TryGetPoint(out Vector3d first));
        Assert.True(contact.SecondAnchor.TryGetPoint(out Vector3d second));
        Assert.Equal(new Vector3d(4, 0, 0), first);
        Assert.Equal(new Vector3d(5, 0, 0), second);
    }

    [Fact]
    public void Cylinder_RimWithSubRawNormalComponent_RetainsRadialWitness()
    {
        Fixed64 epsilon = Fixed64.MinIncrement;
        var triangle = new FixedTriangle(new Vector3d((Fixed64)9, (Fixed64)5 - epsilon, (Fixed64)4),
            new Vector3d(Fixed64.One, (Fixed64)5 + epsilon, (Fixed64)(-4)), new Vector3d(5, 6, 0));

        Assert.True(triangle.TryGetCenteredFiniteCylinderContact(
            Vector3d.Zero, FixedQuaternion.Identity, Vector3d.Zero,
            FixedQuaternion.Identity, (Fixed64)10, (Fixed64)5,
            out FixedContactAnchors contact, out bool isCapFaceContact));

        Assert.Equal(Vector3d.Down, contact.Normal);
        Assert.Equal(Fixed64.Zero, contact.Depth);
        Assert.False(isCapFaceContact);
        Assert.True(contact.FirstAnchor.TryGetPoint(out Vector3d first));
        Assert.True(contact.SecondAnchor.TryGetPoint(out Vector3d second));
        Assert.Equal(new Vector3d(5, 5, 0), first);
        Assert.Equal(first, second);
    }

    [Fact]
    public void Cylinder_SubRawRimTangency_RoundsMatchedWitnessOnlyAtFinalCoordinates()
    {
        Fixed64 epsilon = Fixed64.MinIncrement;
        Vector3d point = new(Fixed64.FromRaw(5), (Fixed64)5 + epsilon, Fixed64.FromRaw(3));
        Vector3d edge = new(20, 13, -69);
        var triangle = new FixedTriangle(point + edge, point - edge, point + new Vector3d(5, 56, 12));
        Assert.True(triangle.TryGetCenteredFiniteCylinderContact(Vector3d.Zero, FixedQuaternion.Identity,
            Vector3d.Zero, FixedQuaternion.Identity, (Fixed64)10, Fixed64.FromRaw(9), out FixedContactAnchors contact));
        Assert.Equal(Fixed64.Zero, contact.Depth);
        Assert.True(contact.FirstAnchor.TryGetPoint(out Vector3d first));
        Assert.True(contact.SecondAnchor.TryGetPoint(out Vector3d second));
        // The sole exact touch is p=(45/13 raw,5,108/13 raw). Rounding p
        // before edge projection moves X across a second rounding threshold.
        var expected = new Vector3d(Fixed64.FromRaw(3), (Fixed64)5, Fixed64.FromRaw(8));
        Assert.Equal(expected, first); Assert.Equal(expected, second);
    }

    [Fact]
    public void Cylinder_SubRawRimToFaceProjection_RoundsEachWitnessOnlyOnce()
    {
        Vector3d point = new(Fixed64.FromRaw(3), (Fixed64)5 + Fixed64.FromRaw(178), Fixed64.FromRaw(10));
        Vector3d e = new(1, -100, 0), f = new(0, -300, 1);
        var triangle = new FixedTriangle(point + e + f, point - e + f, point - f * Fixed64.Two);
        Assert.True(triangle.TryGetCenteredFiniteCylinderContact(Vector3d.Zero, FixedQuaternion.Identity,
            Vector3d.Zero, FixedQuaternion.Identity, (Fixed64)10, Fixed64.FromRaw(11), out FixedContactAnchors contact));
        // Exact outward normal (100,1,300), plane constant 5+3478 raw.
        // Support excess (1100*sqrt(10)-3478) raw is positive, but depth
        // rounds to zero. The exact projected face foot rounds to Z=10 raw;
        // projecting the rounded cylinder support instead would give Z=11.
        Assert.Equal(Fixed64.Zero, contact.Depth);
        Assert.True(contact.FirstAnchor.TryGetPoint(out Vector3d first));
        Assert.True(contact.SecondAnchor.TryGetPoint(out Vector3d second));
        var expected = new Vector3d(Fixed64.FromRaw(3), (Fixed64)5, Fixed64.FromRaw(10));
        Assert.Equal(expected, first); Assert.Equal(expected, second);
    }

    [Fact]
    public void Cylinder_InteriorEdgeRimStationaryRoot_ReturnsExactPairedFeature()
    {
        // q=(765/256,317/64,255/64), p=(3,5,4). The edge direction
        // (8,-1,-3) admits the exact chart root t=1/3 with K!=0, rather
        // than a chart endpoint or rational K=0 principal direction.
        FixedTriangle triangle = CreateInteriorRimTriangle();
        Assert.True(triangle.TryGetCenteredFiniteCylinderContact(Vector3d.Zero, FixedQuaternion.Identity,
            Vector3d.Zero, FixedQuaternion.Identity, (Fixed64)10, (Fixed64)5, out FixedContactAnchors contact));
        Assert.Equal(new Vector3d(-Fixed64.FromFraction(3, 13), -Fixed64.FromFraction(12, 13), -Fixed64.FromFraction(4, 13)), contact.Normal);
        Assert.Equal(Fixed64.FromFraction(13, 256), contact.Depth);
        Assert.True(contact.FirstAnchor.TryGetPoint(out Vector3d first));
        Assert.True(contact.SecondAnchor.TryGetPoint(out Vector3d second));
        Assert.Equal(new Vector3d(Fixed64.FromFraction(765, 256), Fixed64.FromFraction(317, 64), Fixed64.FromFraction(255, 64)), first);
        Assert.Equal(new Vector3d(3, 5, 4), second);
    }

    [Theory]
    [InlineData(-1L, true)]
    [InlineData(0L, true)]
    [InlineData(1L, false)]
    public void Cylinder_InteriorRimRootTouch_DistinguishesOneRawAxialDisplacement(long rawY, bool overlaps)
    {
        FixedTriangle triangle = CreateInteriorRimTriangle();
        var translation = new Vector3d(Fixed64.FromFraction(3, 256),
            Fixed64.FromFraction(12, 256) + Fixed64.FromRaw(rawY), Fixed64.FromFraction(4, 256));
        Assert.Equal(overlaps, triangle.TryGetCenteredFiniteCylinderContact(translation, FixedQuaternion.Identity,
            Vector3d.Zero, FixedQuaternion.Identity, (Fixed64)10, (Fixed64)5, out FixedContactAnchors contact));
        if (rawY == 0)
        {
            Assert.Equal(Fixed64.Zero, contact.Depth);
            Assert.True(contact.FirstAnchor.TryGetPoint(out Vector3d first));
            Assert.True(contact.SecondAnchor.TryGetPoint(out Vector3d second));
            Assert.Equal(new Vector3d(3, 5, 4), first);
            Assert.Equal(first, second);
        }
    }

    [Fact]
    public void CircleSlab_ObliqueEdgeRimMinimum_ReturnsPairedWitnesses()
    {
        // The edge midpoint q=(77/16,19/4,0) is 5/16 from the rim
        // p=(5,5,0), along (-3/5,-4/5,0). The third vertex lies strictly
        // behind that support edge. Neither a face normal nor an edge/side
        // cross normal is this minimum; its edge/rim stationary direction is needed.
        FixedTriangle triangle = CreateObliqueRimTriangle();

        Assert.True(triangle.TryGetCircleSlabContact(
            Vector3d.Zero, FixedQuaternion.Identity, Vector3d.Zero,
            Fixed64.Zero, (Fixed64)5, (Fixed64)5, out FixedContactAnchors contact));

        Assert.Equal(new Vector3d(-Fixed64.FromFraction(3, 5),
            -Fixed64.FromFraction(4, 5), Fixed64.Zero), contact.Normal);
        Assert.Equal(Fixed64.FromFraction(5, 16), contact.Depth);
        Assert.False(contact.DepthIsClamped);
        Assert.True(contact.FirstAnchor.TryGetPoint(out Vector3d trianglePoint));
        Assert.True(contact.SecondAnchor.TryGetPoint(out Vector3d cylinderPoint));
        Assert.Equal(new Vector3d(Fixed64.FromFraction(77, 16),
            Fixed64.FromFraction(19, 4), Fixed64.Zero), trianglePoint);
        Assert.Equal(new Vector3d(5, 5, 0), cylinderPoint);
    }

    [Fact]
    public void CircleSlab_ObliqueEdgeRimGap_RejectsSeparatedTriangle()
    {
        // Translating the triangle by (3/8,1/2,0) moves it 5/8 outward
        // along the same certified support direction, leaving a 5/16 gap.
        FixedTriangle triangle = CreateObliqueRimTriangle();

        Assert.False(triangle.TryGetCircleSlabContact(
            new Vector3d(Fixed64.FromFraction(3, 8), Fixed64.Half, Fixed64.Zero),
            FixedQuaternion.Identity, Vector3d.Zero,
            Fixed64.Zero, (Fixed64)5, (Fixed64)5, out _));
    }

    [Theory]
    [InlineData(-1, true)]
    [InlineData(0, true)]
    [InlineData(1, false)]
    public void CircleSlab_ObliqueRimTouch_DistinguishesOneRawDisplacement(long rawX, bool overlaps)
    {
        var triangle = new FixedTriangle(
            new Vector3d((Fixed64)6, Fixed64.FromFraction(17, 4), Fixed64.FromFraction(5, 4)),
            new Vector3d((Fixed64)4, Fixed64.FromFraction(23, 4), -Fixed64.FromFraction(5, 4)),
            new Vector3d(8, 9, 0));

        Assert.Equal(overlaps, triangle.TryGetCircleSlabContact(
            new Vector3d(Fixed64.FromRaw(rawX), Fixed64.Zero, Fixed64.Zero),
            FixedQuaternion.Identity, Vector3d.Zero,
            Fixed64.Zero, (Fixed64)5, (Fixed64)5, out FixedContactAnchors contact));
        if (rawX == 0)
        {
            Assert.Equal(Fixed64.Zero, contact.Depth);
            Assert.True(contact.FirstAnchor.TryGetPoint(out Vector3d first));
            Assert.True(contact.SecondAnchor.TryGetPoint(out Vector3d second));
            Assert.Equal(new Vector3d(5, 5, 0), first);
            Assert.Equal(first, second);
        }
    }

    [Fact]
    public void CircleSlab_FullDomainHalfThickness_RetainsUnmaterializedCap()
    {
        Fixed64 capY = Fixed64.MaxValue - Fixed64.FromFraction(1, 4);
        var triangle = new FixedTriangle(
            new Vector3d((Fixed64)(-16), capY, (Fixed64)(-16)),
            new Vector3d((Fixed64)16, capY, (Fixed64)(-16)),
            new Vector3d(Fixed64.Zero, capY, (Fixed64)16));
        var origin = new Vector3d(Fixed64.Zero, Fixed64.MaxValue, Fixed64.Zero);

        Assert.True(triangle.TryGetCircleSlabContact(
            origin, FixedQuaternion.Identity, origin,
            Fixed64.Zero, Fixed64.MaxValue, (Fixed64)5, out FixedContactAnchors contact));

        Assert.Equal(Vector3d.Down, contact.Normal);
        Assert.Equal(Fixed64.FromFraction(1, 4), contact.Depth);
        Assert.False(contact.DepthIsClamped);
        Assert.Equal(origin, contact.FirstAnchor.Origin);
        Assert.Equal(origin, contact.SecondAnchor.Origin);
        Assert.False(contact.FirstAnchor.TryGetPoint(out _));
        Assert.False(contact.SecondAnchor.TryGetPoint(out _));
    }

    private static FixedTriangle CreateObliqueRimTriangle() => new(
        new Vector3d(Fixed64.FromFraction(93, 16), (Fixed64)4, Fixed64.FromFraction(5, 4)),
        new Vector3d(Fixed64.FromFraction(61, 16), Fixed64.FromFraction(11, 2), -Fixed64.FromFraction(5, 4)),
        new Vector3d(Fixed64.FromFraction(125, 16), Fixed64.FromFraction(35, 4), Fixed64.Zero));

    private static FixedTriangle CreateInteriorRimTriangle() => new(
        new Vector3d(Fixed64.FromFraction(1021, 256), Fixed64.FromFraction(309, 64), Fixed64.FromFraction(231, 64)),
        new Vector3d(Fixed64.FromFraction(509, 256), Fixed64.FromFraction(325, 64), Fixed64.FromFraction(279, 64)),
        new Vector3d(Fixed64.FromFraction(813, 256), Fixed64.FromFraction(365, 64), Fixed64.FromFraction(271, 64)));
}
