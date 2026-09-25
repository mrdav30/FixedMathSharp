using FixedMathSharp.Geometry;
using Xunit;

namespace FixedMathSharp.Tests;

public sealed class FiniteDiskStrictOverlapTests
{
    [Theory]
    [InlineData(-1L, true)]
    [InlineData(0L, false)]
    [InlineData(1L, false)]
    public void CapDisk_ClassifiesPerpendicularRimBoundary(long rawOffset, bool expected)
    {
        Fixed64 q = Fixed64.FromRaw(3_037_000_500L);
        FixedQuaternion quarterTurn = new(Fixed64.Zero, Fixed64.Zero, -q, q);
        Assert.Equal(expected, WideFiniteAxisIntersection.DoesFiniteCapDiskEnterSolid(
            Vector3d.Zero, FixedQuaternion.Identity, Fixed64.Two, (Fixed64)5 / 4, 1,
            new Vector3d((Fixed64)7 / 4, (Fixed64)7 / 4, Fixed64.Two + Fixed64.FromRaw(rawOffset)),
            quarterTurn, Fixed64.Two, (Fixed64)5 / 4, false));
    }

    [Fact]
    public void CapDisk_RejectsSeparatedPerpendicularRims()
    {
        Fixed64 q = Fixed64.FromRaw(3_037_000_500L);
        FixedQuaternion quarterTurn = new(Fixed64.Zero, Fixed64.Zero, -q, q);
        Assert.False(WideFiniteAxisIntersection.DoesFiniteCapDiskEnterSolid(
            Vector3d.Zero, FixedQuaternion.Identity, Fixed64.Two, Fixed64.One, 1,
            new Vector3d((Fixed64)7 / 4, (Fixed64)7 / 4, (Fixed64)11 / 8),
            quarterTurn, Fixed64.Two, Fixed64.One, false));
    }

    [Theory]
    [InlineData(false)]
    [InlineData(true)]
    public void CapDisk_DetectsEnclosedTargetSection(bool cone)
    {
        // The cap center and every cap-rim point miss the target; its small
        // plane section is entirely enclosed by the radius-10 cap disk.
        Assert.True(WideFiniteAxisIntersection.DoesFiniteCapDiskEnterSolid(
            new Vector3d(3, 0, 0), FixedQuaternion.Identity, Fixed64.Two, (Fixed64)10, 1,
            Vector3d.Zero, FixedQuaternion.Identity, (Fixed64)4, Fixed64.One, cone));
    }

    [Theory]
    [InlineData(false)]
    [InlineData(true)]
    public void CapDisk_DetectsObliqueTargetSection(bool cone)
    {
        FixedQuaternion rotation = new(Fixed64.Zero, Fixed64.Zero, (Fixed64)3 / 5, (Fixed64)4 / 5);
        Assert.True(WideFiniteAxisIntersection.DoesFiniteCapDiskEnterSolid(
            new Vector3d(5, 0, 0), rotation, Fixed64.Two, (Fixed64)20, 1,
            Vector3d.Zero, FixedQuaternion.Identity, (Fixed64)40, Fixed64.One, cone));
    }

    [Fact]
    public void CapDisk_DetectsEnclosedCylinderSectionEnteringAtBase()
    {
        Fixed64 q = Fixed64.FromRaw(1_358_187_913L);
        FixedQuaternion rotation = new(Fixed64.Zero, Fixed64.Zero, -q, 3 * q);
        // The cap plane is 3x+4y=-7. Its cylinder section contains (0,-7/4,0),
        // but not the target axial midpoint; the disk center and rim miss it.
        Assert.True(WideFiniteAxisIntersection.DoesFiniteCapDiskEnterSolid(
            new Vector3d(0, -3, 5), rotation, Fixed64.Two, (Fixed64)20, 1,
            Vector3d.Zero, FixedQuaternion.Identity, (Fixed64)4, Fixed64.One, false));
    }

    [Fact]
    public void CapDisk_DetectsEnclosedCylinderSectionOnlyNearUpperCap()
    {
        Fixed64 q = Fixed64.FromRaw(1_358_187_913L);
        FixedQuaternion rotation = new(Fixed64.Zero, Fixed64.Zero, -q, 3 * q);
        // Plane 3x+4y=9 meets the cylinder only near y=2; its unconstrained
        // radial minimum y=9/4 is outside the finite target axial interval.
        Assert.True(WideFiniteAxisIntersection.DoesFiniteCapDiskEnterSolid(
            new Vector3d(0, 1, 5), rotation, Fixed64.Two, (Fixed64)20, 1,
            Vector3d.Zero, FixedQuaternion.Identity, (Fixed64)4, Fixed64.One, false));
    }

    [Fact]
    public void CapDisk_DetectsEnclosedConeSectionOnlyNearBase()
    {
        Fixed64 q = Fixed64.FromRaw(3_037_000_500L);
        FixedQuaternion rotation = new(Fixed64.Zero, Fixed64.Zero, -q, q);
        // Plane x=5/2 crosses the H=4,R=3 cone only below y=-4/3.
        Assert.True(WideFiniteAxisIntersection.DoesFiniteCapDiskEnterSolid(
            new Vector3d((Fixed64)3 / 2, Fixed64.Zero, (Fixed64)5), rotation,
            Fixed64.Two, (Fixed64)20, 1,
            Vector3d.Zero, FixedQuaternion.Identity, (Fixed64)4, (Fixed64)3, true));
    }

    [Theory]
    [InlineData(-1L, false)]
    [InlineData(0L, false)]
    [InlineData(1L, true)]
    public void CapDisk_PreservesRigidFrameSubrawAxialBoundary(long rawOffset, bool expected)
    {
        FixedQuaternion rotation = FixedQuaternion.FromEulerAnglesInDegrees((Fixed64)13, (Fixed64)29, (Fixed64)41);
        Vector3d center = new(Fixed64.MaxValue, Fixed64.MinValue, Fixed64.MaxValue);
        Assert.Equal(expected, WideFiniteAxisIntersection.DoesFiniteCapDiskEnterSolid(
            center, rotation, Fixed64.Two, Fixed64.One, 1,
            center, rotation, Fixed64.Two + Fixed64.FromRaw(rawOffset), Fixed64.One, false));
    }

    [Fact]
    public void CapDisk_RejectsConeApexOnlyTangency() =>
        Assert.False(WideFiniteAxisIntersection.DoesFiniteCapDiskEnterSolid(
            Vector3d.Zero, FixedQuaternion.Identity, Fixed64.Two, Fixed64.One, 1,
            Vector3d.Zero, FixedQuaternion.Identity, Fixed64.Two, Fixed64.One, true));
}
