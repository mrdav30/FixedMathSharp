using System;
using System.Numerics;
using FixedMathSharp.Geometry;
using Xunit;

namespace FixedMathSharp.Tests.Bounds;

public sealed class FixedConvexPrismCurvedDepthTests
{
    private static readonly Vector3d ConeNormal = new(
        Fixed64.FromRaw(3_841_535_534L), Fixed64.FromRaw(1_920_767_767L), Fixed64.Zero);

    [Theory]
    [InlineData(-1L, false, false)]
    [InlineData(0L, false, false)]
    [InlineData(1L, false, false)]
    [InlineData(0L, false, true)]
    [InlineData(0L, true, false)]
    [InlineData(0L, true, true)]
    public void ConeSide_UsesTheSmoothSupportingPlaneAtThePrismEdge(long gapRaw, bool collapsed, bool reverse)
    {
        Vector2d[] footprint = ConeFootprint(collapsed, reverse);
        Vector3d prismOrigin = new(Fixed64.Half + Fixed64.FromRaw(gapRaw), Fixed64.One, Fixed64.Zero);
        bool hit = FixedConvexPrismRelations.TryGetCenteredConeContact(
            Vector3d.Zero, FixedQuaternion.Identity, Vector3d.Up, Fixed64.Two, Fixed64.One,
            prismOrigin, Fixed64.Zero, footprint, Fixed64.One, out FixedContactAnchors contact);

        Assert.Equal(gapRaw <= 0, hit);
        if (!hit)
        {
            Assert.Equal(default, contact);
            return;
        }

        // The cone satisfies 2x+y<=1; the prism has x>=1/2+gap and y>=0.
        // At gap=0 their only common point is (1/2,0,0), whose unique
        // smooth cone normal is (2,1,0)/sqrt(5). A one-raw inward shift
        // gives depth 2/sqrt(5) raw, rounding to one raw. Its exact cone
        // point (1/2-1/5 raw,2/5 raw,0) rounds back to the touching point.
        Assert.False(contact.DepthIsClamped);
        Assert.Equal(Fixed64.FromRaw(gapRaw < 0 ? 1 : 0), contact.Depth);
        Assert.Equal(ConeNormal, contact.Normal);
        AssertPoint(new Vector3d(Fixed64.Half, Fixed64.Zero, Fixed64.Zero), contact.FirstAnchor);
        AssertPoint(new Vector3d(prismOrigin.X, Fixed64.Zero, Fixed64.Zero), contact.SecondAnchor);
    }

    [Theory]
    [InlineData(false)]
    [InlineData(true)]
    public void ConeSide_InwardNormalTranslationHasAnIndependentMinimumAndClearsTheWholeShape(bool collapsed)
    {
        Fixed64 delta = Fixed64.FromFraction(1, 64);
        Vector3d prismOrigin = new(Fixed64.Half - delta * Fixed64.Two,
            Fixed64.One - delta, Fixed64.Zero);
        Vector2d[] footprint = ConeFootprint(collapsed, false);
        Assert.True(FixedConvexPrismRelations.TryGetCenteredConeContact(
            Vector3d.Zero, FixedQuaternion.Identity, Vector3d.Up, Fixed64.Two, Fixed64.One,
            prismOrigin, Fixed64.Zero, footprint, Fixed64.One, out FixedContactAnchors contact));

        // The cone's z=0 triangular section minus the prism's x=1/2
        // vertical segment, extruded through |z|<=1, is contained in the
        // complete Minkowski difference. At -delta*(2,1,0), its only nearby
        // face is 2x+y=0; all other faces are over 1/2 away. Thus it contains
        // a ball of radius sqrt(5)/64, while that supporting plane bounds
        // the complete difference by the same distance.
        const long expectedDepthRaw = 150_059_982L;
        BigInteger squaredRawDepth = (BigInteger)5 << 52;
        Assert.True(BigInteger.Pow(2 * (BigInteger)expectedDepthRaw - 1, 2) < 4 * squaredRawDepth);
        Assert.True(BigInteger.Pow(2 * (BigInteger)expectedDepthRaw + 1, 2) > 4 * squaredRawDepth);
        Assert.Equal(Fixed64.FromRaw(expectedDepthRaw), contact.Depth);
        Assert.False(contact.DepthIsClamped);
        Assert.Equal(ConeNormal, contact.Normal);
        AssertPoint(new Vector3d(Fixed64.Half, Fixed64.Zero, Fixed64.Zero), contact.FirstAnchor);
        AssertPoint(new Vector3d(prismOrigin.X, -delta, Fixed64.Zero), contact.SecondAnchor);

        Vector3d clearedOrigin = prismOrigin + contact.Normal * (contact.Depth + Fixed64.FromRaw(8));
        // This scalar inequality separates every cone point from every
        // translated prism point, independently of the contact predicate.
        Assert.True(clearedOrigin.X * Fixed64.Two + clearedOrigin.Y - Fixed64.One > Fixed64.One);
        Assert.False(FixedConvexPrismRelations.TryGetCenteredConeContact(
            Vector3d.Zero, FixedQuaternion.Identity, Vector3d.Up, Fixed64.Two, Fixed64.One,
            clearedOrigin, Fixed64.Zero, footprint, Fixed64.One, out _));
    }

    [Theory]
    [InlineData(false)]
    [InlineData(true)]
    public void ConeSide_QuarterTurnAndFullDomainTranslationPreserveTheContact(bool largeOrigin)
    {
        Vector3d translation = largeOrigin
            ? new Vector3d(Fixed64.MaxValue - (Fixed64)4, Fixed64.MinValue + (Fixed64)4,
                Fixed64.MaxValue - (Fixed64)4)
            : Vector3d.Zero;
        FixedQuaternion rotation = FixedQuaternion.FromAxisAngle(Vector3d.Up, -Fixed64.HalfPi);
        Vector3d prismOrigin = translation + new Vector3d(Fixed64.Zero, Fixed64.One, Fixed64.Half);
        Assert.True(FixedConvexPrismRelations.TryGetCenteredConeContact(
            translation, rotation, Vector3d.Up, Fixed64.Two, Fixed64.One,
            prismOrigin, Fixed64.HalfPi, ConeFootprint(false, false), Fixed64.One,
            out FixedContactAnchors contact));

        // The quarter turn sends local +X to world +Z. Relative support
        // is therefore 2z+y<=1 regardless of the common large translation.
        Assert.Equal(Fixed64.Zero, contact.Depth);
        Assert.False(contact.DepthIsClamped);
        Assert.Equal(new Vector3d(Fixed64.Zero, ConeNormal.Y, ConeNormal.X), contact.Normal);
        FixedPointAnchor origin = new(translation, FixedQuaternion.Identity, Vector3d.Zero);
        Assert.True(contact.FirstAnchor.TryGetOffsetFrom(origin, out Vector3d first));
        Assert.True(contact.SecondAnchor.TryGetOffsetFrom(origin, out Vector3d second));
        Assert.Equal(new Vector3d(Fixed64.Zero, Fixed64.Zero, Fixed64.Half), first);
        Assert.Equal(first, second);
        Assert.Equal(translation, contact.FirstAnchor.Origin);
        Assert.Equal(rotation, contact.FirstAnchor.Rotation);
        Assert.Equal(prismOrigin, contact.SecondAnchor.Origin);
    }

    [Theory]
    [InlineData(-1, false)]
    [InlineData(0, false)]
    [InlineData(1, false)]
    [InlineData(-1, true)]
    public void CylinderRim_MatchesTheCompleteBoxContactAndHasPairedSurfaceWitnesses(int displacement, bool reverse)
    {
        Vector2d[] footprint = { new((Fixed64)(-10), -Fixed64.Half), new((Fixed64)10, -Fixed64.Half),
            new((Fixed64)10, Fixed64.Half), new((Fixed64)(-10), Fixed64.Half) };
        if (reverse)
            Array.Reverse(footprint);
        const long quaternionScale = 1_920_767_767L;
        FixedQuaternion rotation = new(Fixed64.Zero, Fixed64.Zero,
            Fixed64.FromRaw(-quaternionScale), Fixed64.FromRaw(2 * quaternionScale));
        Vector3d shift = new(Fixed64.Zero, Fixed64.FromFraction(15 * displacement, 1024),
            Fixed64.FromFraction(16 * displacement, 1024));
        Vector3d center = new Vector3d((Fixed64)(-5), Fixed64.FromFraction(31, 2),
            Fixed64.FromFraction(41, 2)) + shift;
        var box = new FixedOrientedBox(Vector3d.Zero, FixedQuaternion.Identity,
            new Vector3d((Fixed64)10, Fixed64.Half, Fixed64.Half));
        bool boxHit = box.TryGetCenteredCylinderContact(center, rotation, Vector3d.Up,
            (Fixed64)10, (Fixed64)25, out FixedContactAnchors boxContact);
        bool prismHit = FixedConvexPrismRelations.TryGetCenteredCylinderContact(
            center, rotation, Vector3d.Up, (Fixed64)10, (Fixed64)25,
            Vector3d.Zero, Fixed64.Zero, footprint, Fixed64.Half, out FixedContactAnchors contact);
        Assert.Equal(displacement <= 0, boxHit);
        Assert.Equal(boxHit, prismHit);
        if (!prismHit)
        {
            Assert.Equal(default, contact);
            return;
        }

        // Exact axis a=(4,3,0)/5 and radial u=(-9,12,20)/25 give
        // center-5a-25u=(0,1/2,1/2)+shift. The supporting normal is
        // (0,15,16)/sqrt(481). The inward displacement remains inside
        // the edge and is below the projected ellipse curvature radius16.
        Assert.Equal(Fixed64.FromRaw(displacement < 0 ? 91_988_268L : 0), contact.Depth);
        Assert.Equal(boxContact.Depth, contact.Depth);
        Assert.Equal(-boxContact.Normal, contact.Normal);
        Assert.False(contact.DepthIsClamped);
        Vector3d targetPoint = new(Fixed64.Zero, Fixed64.Half, Fixed64.Half);
        AssertPoint(targetPoint + shift, contact.FirstAnchor);
        AssertPoint(targetPoint, contact.SecondAnchor);
        Assert.Equal(center, contact.FirstAnchor.Origin);
        Assert.Equal(rotation, contact.FirstAnchor.Rotation);
        Assert.Equal(Vector3d.Zero, contact.SecondAnchor.Origin);
    }

    private static Vector2d[] ConeFootprint(bool collapsed, bool reverse)
    {
        Vector2d[] footprint = collapsed
            ? new Vector2d[] { new(0, -1), new(0, 1), new(0, 1) }
            : new Vector2d[] { new(0, -1), new(1, -1), new(1, 1), new(0, 1) };
        if (reverse)
            Array.Reverse(footprint);
        return footprint;
    }

    private static void AssertPoint(Vector3d expected, FixedPointAnchor anchor)
    {
        Assert.True(anchor.TryGetPoint(out Vector3d point));
        Assert.Equal(expected, point);
    }
}
