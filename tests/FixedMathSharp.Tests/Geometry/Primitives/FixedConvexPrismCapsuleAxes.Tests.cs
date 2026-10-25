using System;
using FixedMathSharp.Geometry;
using Xunit;

namespace FixedMathSharp.Tests.Bounds;

public sealed class FixedConvexPrismCapsuleAxisTests
{
    [Theory]
    [InlineData(false, 0, -1)]
    [InlineData(false, 0, 1)]
    [InlineData(false, 1, -1)]
    [InlineData(false, 1, 1)]
    [InlineData(false, 2, -1)]
    [InlineData(false, 2, 1)]
    [InlineData(true, 0, -1)]
    [InlineData(true, 0, 1)]
    [InlineData(true, 1, -1)]
    [InlineData(true, 1, 1)]
    [InlineData(true, 2, -1)]
    [InlineData(true, 2, 1)]
    public void ContainedRoundShape_SelectsTheSignedExitAndPairedSupports(bool capsule, int component, int sign)
    {
        Vector2d[] footprint = { new(-1, -1), new(1, -1), new(1, 1), new(-1, 1) };
        if (sign < 0) Array.Reverse(footprint);
        Vector3d outward = (component == 0 ? Vector3d.Up : component == 1 ? Vector3d.Right : Vector3d.Forward) * (Fixed64)sign;
        Vector3d coreAxis = component == 1 ? Vector3d.Forward : Vector3d.Right;
        // The selected half-extent is 1/4. A center offset of 1/2 leaves
        // an exit of 3/4; every other coordinate needs at least 5/4.
        AssertExit(capsule, outward * Fixed64.Half, coreAxis, footprint,
            -outward, Fixed64.FromFraction(3, 4), Fixed64.Quarter);
    }

    [Theory]
    [InlineData(-1, false)]
    [InlineData(-1, true)]
    [InlineData(1, false)]
    [InlineData(1, true)]
    public void ContainedAxialCapsule_SelectsTheFacingEndCap(int sign, bool reverseAxis)
    {
        Vector2d[] footprint = { new(-2, -2), new(2, -2), new(2, 2), new(-2, 2) };
        Vector3d outward = Vector3d.Up * (Fixed64)sign;
        // The complete capsule is inside Y=[-1,1]. Its far end is 3/4
        // from its center, so the nearest whole-shape exit is 1-1/8+3/4.
        AssertExit(true, outward * Fixed64.FromFraction(1, 8), reverseAxis ? Vector3d.Down : Vector3d.Up,
            footprint, -outward, Fixed64.FromFraction(13, 8), Fixed64.FromFraction(3, 4));
    }

    [Theory]
    [InlineData(false, false)]
    [InlineData(false, true)]
    [InlineData(true, false)]
    [InlineData(true, true)]
    public void CenteredProjectionTie_PreservesTheFirstAxisAndZeroEndpointPolicy(bool capsule, bool collapsed)
    {
        Vector2d[] footprint = collapsed
            ? new Vector2d[] { new(0, -1), new(0, 1), new(0, 1) }
            : new Vector2d[] { new(-1, -1), new(1, -1), new(1, 1), new(-1, 1) };
        // A straddling equal-depth interval keeps its negative orientation.
        // A collapsed X interval has min=max=0, so the earlier nonnegative
        // minimum branch owns +X instead; neither policy depends on rounding.
        AssertExit(capsule, Vector3d.Zero, collapsed ? Vector3d.Up : Vector3d.Right,
            footprint, collapsed ? Vector3d.Right : Vector3d.Down,
            collapsed ? Fixed64.Quarter : Fixed64.FromFraction(5, 4), Fixed64.Quarter);
    }

    [Theory]
    [InlineData(false)]
    [InlineData(true)]
    public void ContainedRoundShape_RetainsNativeYawAndAnchorsBeyondTheScalarFace(bool capsule)
    {
        Vector2d[] footprint = { new(-2, -2), new(2, -2), new(2, 2), new(-2, 2) };
        Vector3d center = new(Fixed64.MaxValue, Fixed64.MaxValue, Fixed64.MaxValue);
        Vector3d origin = center - Vector3d.Up * Fixed64.Half;
        FixedQuaternion rotation = new(Fixed64.Half, Fixed64.Half, Fixed64.Half, Fixed64.Half);
        Fixed64 yaw = Fixed64.FromFraction(1, 3);
        // The exact cyclic rotation sends the capsule core to world Z.
        // Its Y exit is still 3/4; the opposing prism face is outside
        // the scalar domain, so compare the retained relative anchors.
        bool hit = capsule
            ? FixedConvexPrismRelations.TryGetCenteredCapsuleContact(center, rotation, Vector3d.Up,
                Fixed64.One, Fixed64.Quarter, origin, yaw, footprint, Fixed64.One, out FixedContactAnchors contact)
            : FixedConvexPrismRelations.TryGetSphereContact(center, Fixed64.Quarter,
                origin, yaw, footprint, Fixed64.One, out contact);
        Assert.True(hit);
        Assert.Equal(Vector3d.Down, contact.Normal);
        Assert.Equal(Fixed64.FromFraction(3, 4), contact.Depth);
        Assert.False(contact.DepthIsClamped);
        Assert.Equal(center, contact.FirstAnchor.Origin);
        Assert.Equal(capsule ? rotation : FixedQuaternion.Identity, contact.FirstAnchor.Rotation);
        Assert.Equal(origin, contact.SecondAnchor.Origin);
        Assert.Equal(FixedQuaternion.FromAxisAngle(Vector3d.Up, -yaw), contact.SecondAnchor.Rotation);
        Assert.False(contact.SecondAnchor.TryGetPoint(out _));
        Assert.True(contact.FirstAnchor.TryGetOffsetFrom(contact.SecondAnchor, out Vector3d separation));
        Assert.Equal(contact.Normal * contact.Depth, separation);
    }

    [Fact]
    public void CapsuleContact_FarPrismOriginRetainsFiniteSupportAnchors()
    {
        Vector2d[] footprint =
        {
            new(-Fixed64.MaxValue + (Fixed64)3, -Fixed64.One),
            new(-Fixed64.MaxValue + (Fixed64)4, -Fixed64.One),
            new(-Fixed64.MaxValue + (Fixed64)4, Fixed64.One),
            new(-Fixed64.MaxValue + (Fixed64)3, Fixed64.One)
        };
        Vector3d center = new(-Fixed64.One, Fixed64.Half, Fixed64.Zero);
        Vector3d origin = new(Fixed64.MaxValue, Fixed64.Zero, Fixed64.Zero);
        // The core crosses the prism's physical X=[3,4]. Its midpoint cannot
        // fit the prism's local scalar frame; each support still needs its
        // own finite anchor, without materializing that overflowing conversion.
        Assert.True(FixedConvexPrismRelations.TryGetCenteredCapsuleContact(center, FixedQuaternion.Identity,
            Vector3d.Right, (Fixed64)10, Fixed64.Quarter, origin, Fixed64.Zero,
            footprint, Fixed64.One, out FixedContactAnchors contact));
        Assert.Equal(center, contact.FirstAnchor.Origin);
        Assert.Equal(origin, contact.SecondAnchor.Origin);
        Assert.Equal(Vector3d.Down, contact.Normal);
        Assert.Equal(Fixed64.FromFraction(3, 4), contact.Depth);
        Assert.False(contact.DepthIsClamped);
        Assert.True(contact.FirstAnchor.TryGetPoint(out Vector3d first));
        Assert.True(contact.SecondAnchor.TryGetPoint(out Vector3d second));
        Assert.Equal(Fixed64.Quarter, first.Y);
        Assert.Equal(Fixed64.Zero, first.Z);
        FixedMathTestHelper.AssertWithinRange(first.X, (Fixed64)(-6), (Fixed64)4);
        Assert.Equal(Fixed64.One, second.Y);
        FixedMathTestHelper.AssertWithinRange(second.X, (Fixed64)3, (Fixed64)4);
        FixedMathTestHelper.AssertWithinRange(second.Z, -Fixed64.One, Fixed64.One);
    }

    [Fact]
    public void ContainedRoundContacts_WarmedPathsDoNotAllocate()
    {
        Vector2d[] footprint = { new(-1, -1), new(1, -1), new(1, 1), new(-1, 1) };
        long allocated = FixedMathTestHelper.MeasureWarmedAllocations(() =>
        {
            Assert.True(Query(false, Vector3d.Up * Fixed64.Half, Vector3d.Right, footprint, out _));
            Assert.True(Query(true, Vector3d.Up * Fixed64.Half, Vector3d.Right, footprint, out _));
        });
        Assert.Equal(0, allocated);
    }

    [Theory]
    [InlineData(0, -1)]
    [InlineData(0, 0)]
    [InlineData(0, 1)]
    [InlineData(1, -1)]
    [InlineData(1, 0)]
    [InlineData(1, 1)]
    [InlineData(2, -1)]
    [InlineData(2, 0)]
    [InlineData(2, 1)]
    public void InitialCapsuleAxes_PreserveOneRawClosedContact(int axis, int gapRaw)
    {
        Vector2d[] footprint = { new(-1, -1), new(1, -1), new(1, 1), new(-1, 1) };
        Vector3d direction = axis == 0 ? Vector3d.Up : axis == 1 ? Vector3d.Right : Vector3d.Forward;
        // A unit-long +X capsule with radius 1/4 has exact support
        // extents (3/4,1/4,1/4). Against the unit prism, these placements
        // independently isolate Up, the capsule axis, and their cross axis.
        Fixed64 tangentDistance = Fixed64.One + Fixed64.FromFraction(axis == 1 ? 3 : 1, 4);
        Vector3d center = direction * (tangentDistance + Fixed64.FromRaw(gapRaw));
        bool hit = FixedConvexPrismRelations.TryGetCenteredCapsuleContact(
            center, FixedQuaternion.Identity, Vector3d.Right, Fixed64.One, Fixed64.FromFraction(1, 4),
            Vector3d.Zero, Fixed64.Zero, footprint, Fixed64.One, out FixedContactAnchors contact);

        Assert.Equal(gapRaw <= 0, hit);
        if (!hit)
        {
            Assert.Equal(default, contact);
            return;
        }
        Assert.Equal(-direction, contact.Normal);
        Assert.Equal(Fixed64.FromRaw(-gapRaw), contact.Depth);
        Assert.False(contact.DepthIsClamped);
    }

    private static void AssertExit(bool capsule, Vector3d center, Vector3d coreAxis,
        ReadOnlySpan<Vector2d> footprint, Vector3d normal, Fixed64 depth, Fixed64 supportDistance)
    {
        Assert.True(Query(capsule, center, coreAxis, footprint, out FixedContactAnchors contact));
        Assert.Equal(normal, contact.Normal);
        Assert.Equal(depth, contact.Depth);
        Assert.False(contact.DepthIsClamped);
        Assert.True(contact.FirstAnchor.TryGetPoint(out Vector3d first));
        Assert.True(contact.SecondAnchor.TryGetPoint(out Vector3d second));
        Assert.Equal(center + normal * supportDistance, first);
        Assert.Equal(normal * depth, first - second);
        if (!capsule)
        {
            Assert.True(FixedConvexPrismRelations.TryGetCenteredCapsuleContact(center, FixedQuaternion.Identity,
                coreAxis, Fixed64.Zero, Fixed64.Quarter, Vector3d.Zero, Fixed64.Zero,
                footprint, Fixed64.One, out FixedContactAnchors zeroCore));
            Assert.Equal(contact, zeroCore);
        }

        Vector3d touchingCenter = center - normal * depth;
        Assert.True(Query(capsule, touchingCenter, coreAxis, footprint, out FixedContactAnchors touching));
        Assert.Equal(Fixed64.Zero, touching.Depth);
        // A translated support coordinate equals the complete prism face at
        // the returned depth, then strictly exceeds it after one raw unit.
        Assert.False(Query(capsule, touchingCenter - normal * Fixed64.MinIncrement, coreAxis, footprint, out _));
    }

    private static bool Query(bool capsule, Vector3d center, Vector3d coreAxis,
        ReadOnlySpan<Vector2d> footprint, out FixedContactAnchors contact) => capsule
        ? FixedConvexPrismRelations.TryGetCenteredCapsuleContact(center, FixedQuaternion.Identity,
            coreAxis, Fixed64.One, Fixed64.Quarter, Vector3d.Zero, Fixed64.Zero, footprint, Fixed64.One, out contact)
        : FixedConvexPrismRelations.TryGetSphereContact(center, Fixed64.Quarter,
            Vector3d.Zero, Fixed64.Zero, footprint, Fixed64.One, out contact);
}
