using System;
using FixedMathSharp.Geometry;
using Xunit;

namespace FixedMathSharp.Tests.Bounds;

public sealed class FixedOrientedBoxCylinderContactRegressionTests
{
    [Fact]
    public void Contact_RejectsCapClippedRadialSeparation()
    {
        FixedOrientedBox box = CreateCapClippedSeparatedBox();

        // FMS-Issue-025: the authored cap-clipped minimum radial distance
        // squared is 2530093/1922000 > 1, with ample margin for input rounding.
        // The legacy contact incorrectly returned depth raw 160035987.
        Assert.False(box.TryGetCenteredCylinderContact(
            Vector3d.Zero, FixedQuaternion.Identity, Vector3d.Up,
            Fixed64.Two, Fixed64.One, out FixedContactAnchors contact));
        Assert.Equal(default, contact);
    }

    [Fact]
    public void Manifold_RejectsCapClippedRadialSeparation()
    {
        FixedOrientedBox box = CreateCapClippedSeparatedBox();
        Span<FixedContactLocalPoints> contacts = stackalloc FixedContactLocalPoints[4];

        Assert.False(box.TryGetCenteredCylinderContact(
            Vector3d.Zero, FixedQuaternion.Identity, Vector3d.Up,
            Fixed64.Two, Fixed64.One, contacts,
            out FixedContactAnchors contact, out int count));
        Assert.Equal(default, contact);
        Assert.Equal(0, count);
    }

    [Theory]
    [InlineData(-1L, false)]
    [InlineData(0L, true)]
    [InlineData(1L, true)]
    [InlineData(1_073_741_824L, true)]
    public void RadialCorner_UsesExactMinimumDepthAndClosedBoundary(
        long radiusOffsetRaw, bool expectedContact)
    {
        // The box projects to [3,4] x [4,5] in XZ. Its closest radial
        // point is (3,4), exactly distance 5. The cap interval [-2,2]
        // contains the box's [-1/2,1/2], so the minimum escape distance
        // is radius - 5 for these radii, not a cap or box-face depth.
        var box = new FixedOrientedBox(
            new Vector3d(Fixed64.FromFraction(7, 2), Fixed64.Zero, Fixed64.FromFraction(9, 2)),
            FixedQuaternion.Identity,
            new Vector3d(Fixed64.Half, Fixed64.Half, Fixed64.Half));
        Fixed64 radius = (Fixed64)5 + Fixed64.FromRaw(radiusOffsetRaw);

        bool actual = box.TryGetCenteredCylinderContact(
            Vector3d.Zero, FixedQuaternion.Identity, Vector3d.Up,
            (Fixed64)4, radius, out FixedContactAnchors primary);
        Assert.Equal(expectedContact, actual);

        Span<FixedContactLocalPoints> contacts = stackalloc FixedContactLocalPoints[4];
        bool manifoldActual = box.TryGetCenteredCylinderContact(
            Vector3d.Zero, FixedQuaternion.Identity, Vector3d.Up,
            (Fixed64)4, radius, contacts, out FixedContactAnchors manifold, out int count);
        Assert.Equal(expectedContact, manifoldActual);
        Assert.Equal(primary, manifold);
        Assert.Equal(0, count);

        if (!expectedContact)
        {
            Assert.Equal(default, primary);
            return;
        }

        Assert.Equal(Fixed64.FromRaw(radiusOffsetRaw), primary.Depth);
        Assert.False(primary.DepthIsClamped);
        Assert.Equal(Fixed64.Zero, primary.Normal.Y);
        FixedMathTestHelper.AssertWithinRange(primary.Normal.X,
            Fixed64.FromFraction(-3, 5) - Fixed64.FromRaw(1),
            Fixed64.FromFraction(-3, 5) + Fixed64.FromRaw(1));
        FixedMathTestHelper.AssertWithinRange(primary.Normal.Z,
            Fixed64.FromFraction(-4, 5) - Fixed64.FromRaw(1),
            Fixed64.FromFraction(-4, 5) + Fixed64.FromRaw(1));
    }

    [Theory]
    [InlineData(0, false)]
    [InlineData(1, false)]
    [InlineData(-1, false)]
    [InlineData(0, true)]
    [InlineData(1, true)]
    [InlineData(-1, true)]
    public void RadialCorner_PreservesDepthUnderAxisReversalRigidHalfTurnAndExtremeTranslation(
        int translationSign, bool halfTurn)
    {
        Vector3d translation = translationSign == 0 ? Vector3d.Zero : new Vector3d(
            translationSign > 0 ? Fixed64.MaxValue - (Fixed64)16 : Fixed64.MinValue + (Fixed64)16,
            translationSign > 0 ? Fixed64.MinValue + (Fixed64)16 : Fixed64.MaxValue - (Fixed64)16,
            translationSign > 0 ? Fixed64.MaxValue - (Fixed64)16 : Fixed64.MinValue + (Fixed64)16);
        // This shared half-turn has exactly representable transformed centers
        // and an exact rational frame, so no rounded rotation contaminates depth.
        FixedQuaternion rotation = halfTurn
            ? new FixedQuaternion(Fixed64.Zero, Fixed64.One, Fixed64.Zero, Fixed64.Zero)
            : FixedQuaternion.Identity;
        int radialSign = halfTurn ? -1 : 1;
        var box = new FixedOrientedBox(translation + new Vector3d(
            (Fixed64)radialSign * Fixed64.FromFraction(7, 2), Fixed64.Zero,
            (Fixed64)radialSign * Fixed64.FromFraction(9, 2)), rotation,
            new Vector3d(Fixed64.Half, Fixed64.Half, Fixed64.Half));

        Assert.True(box.TryGetCenteredCylinderContact(translation, rotation, Vector3d.Up,
            (Fixed64)4, Fixed64.FromFraction(21, 4), out FixedContactAnchors forward));
        Assert.True(box.TryGetCenteredCylinderContact(translation, rotation, -Vector3d.Up,
            (Fixed64)4, Fixed64.FromFraction(21, 4), out FixedContactAnchors reversed));
        Assert.Equal(Fixed64.Quarter, forward.Depth);
        Assert.Equal(forward.Depth, reversed.Depth);
        Assert.Equal(forward.Normal, reversed.Normal);
        Assert.Equal(box.Center, forward.FirstAnchor.Origin);
        Assert.Equal(translation, forward.SecondAnchor.Origin);
        Assert.Equal(Fixed64.Zero, forward.Normal.Y);
        Assert.Equal(-radialSign, Fixed64.Sign(forward.Normal.X));
        Assert.Equal(-radialSign, Fixed64.Sign(forward.Normal.Z));
    }

    [Theory]
    [InlineData(-1, -1)]
    [InlineData(-1, 1)]
    [InlineData(1, -1)]
    [InlineData(1, 1)]
    public void ZeroRadiusSegment_PreservesCapDepthAndManifoldWhenLocalAxisReverses(
        int capSign, int axisSign)
    {
        var box = new FixedOrientedBox(Vector3d.Zero, FixedQuaternion.Identity,
            new Vector3d(Fixed64.Half, Fixed64.Half, Fixed64.Half));
        Vector3d center = new(Fixed64.Zero,
            (Fixed64)capSign * Fixed64.FromFraction(3, 4), Fixed64.Zero);
        Span<FixedContactLocalPoints> contacts = stackalloc FixedContactLocalPoints[4];
        Assert.True(box.TryGetCenteredCylinderContact(center, FixedQuaternion.Identity,
            Vector3d.Up * (Fixed64)axisSign, Fixed64.One, Fixed64.Zero,
            contacts, out FixedContactAnchors contact, out int count));
        Assert.Equal(Fixed64.Quarter, contact.Depth);
        Assert.Equal(Vector3d.Up * (Fixed64)capSign, contact.Normal);
        Assert.Equal(1, count);
        Assert.Equal(new Vector3d(Fixed64.Zero, (Fixed64)capSign * Fixed64.Half, Fixed64.Zero),
            contacts[0].FirstLocalPoint);
        Assert.Equal(new Vector3d(Fixed64.Zero, (Fixed64)(-capSign) * Fixed64.Half, Fixed64.Zero),
            contacts[0].SecondLocalPoint);
    }

    [Theory]
    [InlineData(-1, false)]
    [InlineData(0, true)]
    [InlineData(1, false)]
    public void ObliqueFixtures_AgreeWithIndependentCapClippingAwayFromBoundaries(
        int centerSign, bool expected)
    {
        FixedOrientedBox source = CreateCapClippedSeparatedBox();
        var box = new FixedOrientedBox(source.Center * (Fixed64)centerSign,
            source.Orientation, source.HalfExtents);
        // Centered solids have a common interior; the two reflected separated
        // fixtures retain the recorded >1 squared-radial certificate by symmetry.
        bool strict = WideOrientedBox.DoesCenteredCylinderPenetrateBox(
            Vector3d.Zero, FixedQuaternion.Identity, Fixed64.Two, Fixed64.One,
            box.Center, box.Orientation, box.HalfExtents);
        Assert.Equal(expected, strict);
        Assert.Equal(strict, box.TryGetCenteredCylinderContact(
            Vector3d.Zero, FixedQuaternion.Identity, Vector3d.Up,
            Fixed64.Two, Fixed64.One, out FixedContactAnchors primary));
        Span<FixedContactLocalPoints> contacts = stackalloc FixedContactLocalPoints[4];
        Assert.Equal(strict, box.TryGetCenteredCylinderContact(
            Vector3d.Zero, FixedQuaternion.Identity, Vector3d.Up,
            Fixed64.Two, Fixed64.One, contacts, out FixedContactAnchors manifold, out int count));
        Assert.Equal(primary, manifold);
        if (expected)
            Assert.True(primary.Depth > Fixed64.Zero);
        else
        {
            Assert.Equal(default, primary);
            Assert.Equal(0, count);
        }
    }

    [Theory]
    [InlineData(-1L, false, 0L)]
    [InlineData(0L, true, 0L)]
    [InlineData(1L, true, 1L)]
    public void ObliqueEdgeRim_PreservesExactTouchAndRawRadiusNeighbors(
        long radiusOffsetRaw, bool expected, long expectedDepthRaw)
    {
        // The integer quaternion ratio gives exact axis a=(4,3,0)/5.
        // H=(4,3,0), R*u=(-9,12,20), and box edge point v=(0,1/2,1/2)
        // give v+H+R*u=(-5,31/2,41/2), exactly the cylinder center.
        // u is perpendicular to a and has unit length. Its support normal
        // is n=(0,15,16)/sqrt(481), inside the positive-cap/box-edge cone.
        // Thus touch is exact and needs an edge/rim stationary direction,
        // not a box face, cylinder pole, side plane or projected-c direction.
        // At radius+epsilon, depth changes by (20/sqrt(481))*epsilon
        // to first order: one raw radius step rounds to one raw depth step.
        const long quaternionScale = 1_920_767_767L;
        FixedQuaternion rotation = new(Fixed64.Zero, Fixed64.Zero,
            Fixed64.FromRaw(-quaternionScale), Fixed64.FromRaw(2 * quaternionScale));
        var box = new FixedOrientedBox(Vector3d.Zero, FixedQuaternion.Identity,
            new Vector3d((Fixed64)10, Fixed64.Half, Fixed64.Half));
        Vector3d center = new((Fixed64)(-5), Fixed64.FromFraction(31, 2), Fixed64.FromFraction(41, 2));
        Fixed64 radius = (Fixed64)25 + Fixed64.FromRaw(radiusOffsetRaw);
        Assert.Equal(expected, box.TryGetCenteredCylinderContact(center, rotation, Vector3d.Up,
            (Fixed64)10, radius, out FixedContactAnchors primary));
        Span<FixedContactLocalPoints> contacts = stackalloc FixedContactLocalPoints[4];
        Assert.Equal(expected, box.TryGetCenteredCylinderContact(center, rotation, Vector3d.Up,
            (Fixed64)10, radius, contacts, out FixedContactAnchors manifold, out int count));
        Assert.Equal(primary, manifold);
        Assert.Equal(0, count);
        if (!expected)
        {
            Assert.Equal(default, primary);
            return;
        }
        Assert.Equal(Fixed64.FromRaw(expectedDepthRaw), primary.Depth);
        Assert.False(primary.DepthIsClamped);
        Assert.Equal(Fixed64.Zero, primary.Normal.X);
        Assert.True(primary.Normal.Y > Fixed64.Zero);
        Assert.True(primary.Normal.Z > Fixed64.Zero);
        if (radiusOffsetRaw == 0)
        {
            Assert.Equal(Fixed64.FromRaw(2_937_504_781L), primary.Normal.Y);
            Assert.Equal(Fixed64.FromRaw(3_133_338_433L), primary.Normal.Z);
        }
    }

    private static FixedOrientedBox CreateCapClippedSeparatedBox() => new(
        new Vector3d(Fixed64.FromFraction(126, 100),
            Fixed64.FromFraction(-165, 100), Fixed64.FromFraction(-92, 100)),
        new FixedQuaternion((Fixed64)(-1), (Fixed64)(-9), (Fixed64)7, Fixed64.Zero).Normalized,
        new Vector3d(Fixed64.FromFraction(83, 100),
            Fixed64.FromFraction(75, 100), Fixed64.FromFraction(52, 100)));
}
