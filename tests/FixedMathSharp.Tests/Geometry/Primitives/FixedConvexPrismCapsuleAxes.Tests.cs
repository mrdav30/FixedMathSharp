using FixedMathSharp.Geometry;
using Xunit;

namespace FixedMathSharp.Tests.Bounds;

public sealed class FixedConvexPrismCapsuleAxisTests
{
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
}
