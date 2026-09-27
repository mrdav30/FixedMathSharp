using System;
using FixedMathSharp.Geometry;
using Xunit;

namespace FixedMathSharp.Tests.Bounds;

public sealed class FixedOrientedBoxCylinderVertexRimTests
{
    [Theory]
    [InlineData(-1)]
    [InlineData(1)]
    public void VertexRim_RejectsMixedNormalSeparationInsideCapsuleProxy(int reflection)
    {
        var box = new FixedOrientedBox(Vector3d.Zero, FixedQuaternion.Identity, Vector3d.One);
        FixedQuaternion rotation = CylinderRotation();
        Fixed64 delta = Fixed64.FromFraction(1, 64);
        Vector3d center = new Vector3d(-(Fixed64)2 - delta, (Fixed64)2 + delta,
            (Fixed64)5 + (Fixed64)4 * delta) * (Fixed64)reflection;

        // Exact axis a=(1,2,2)/3 and disk direction u=(-2,-1,2)/3
        // are perpendicular unit vectors. At delta=0 the cylinder touches
        // box vertex v=(-1,1,1): v+3a+3u=(-2,2,5).
        // Moving by 3*delta*(a+u) separates along (-1,1,4), whose box
        // support signs differ from both the cap pole and side direction.
        // Reflection gives the opposite vertex and cap with the same proof.
        // The capsule proxy still overlaps: point (-1,delta,1) lies inside
        // the box and has endpoint-distance squared
        // (2+delta)^2+(2+4*delta)^2 = 8+20*delta+17*delta^2 < 9.
        Assert.True(box.TryGetCenteredCapsuleContact(center, rotation, Vector3d.Up,
            (Fixed64)6, (Fixed64)3, out _));
        Assert.False(box.TryGetCenteredCylinderContact(center, rotation, Vector3d.Up,
            (Fixed64)6, (Fixed64)3, out FixedContactAnchors primary));
        Assert.Equal(default, primary);
        Span<FixedContactLocalPoints> contacts = stackalloc FixedContactLocalPoints[4];
        Assert.False(box.TryGetCenteredCylinderContact(center, rotation, Vector3d.Up,
            (Fixed64)6, (Fixed64)3, contacts, out FixedContactAnchors manifold, out int count));
        Assert.Equal(default, manifold);
        Assert.Equal(0, count);
    }

    [Fact]
    public void VertexRim_InwardMovementProducesPositiveContact()
    {
        var box = new FixedOrientedBox(Vector3d.Zero, FixedQuaternion.Identity, Vector3d.One);
        Fixed64 delta = Fixed64.FromFraction(-1, 64);
        Vector3d center = new(-(Fixed64)2 - delta, (Fixed64)2 + delta,
            (Fixed64)5 + (Fixed64)4 * delta);
        // Relative to this cylinder, v=(-1,1,1) has axial coordinate
        // -3*(1+delta) and radial distance 3*(1+delta), both strictly
        // inside. A neighborhood of v therefore gives genuine overlap.
        Assert.True(box.TryGetCenteredCylinderContact(center, CylinderRotation(), Vector3d.Up,
            (Fixed64)6, (Fixed64)3, out FixedContactAnchors primary));
        Assert.True(primary.Depth > Fixed64.Zero);
        Assert.False(primary.DepthIsClamped);
        Span<FixedContactLocalPoints> contacts = stackalloc FixedContactLocalPoints[4];
        Assert.True(box.TryGetCenteredCylinderContact(center, CylinderRotation(), Vector3d.Up,
            (Fixed64)6, (Fixed64)3, contacts, out FixedContactAnchors manifold, out int count));
        Assert.Equal(primary, manifold);
        Assert.Equal(0, count);
    }

    [Fact]
    public void AxisAlignedBoxVertices_DoNotBecomeFalseRimSeparation()
    {
        var box = new FixedOrientedBox(Vector3d.Zero, FixedQuaternion.Identity, Vector3d.One);
        Vector3d center = new(1, 0, 1);
        // The vertical box edge through (1,0,1) lies on the cylinder axis.
        // Its cap/vertex residual has zero radial projection; this is not
        // a separating rim. The XZ square extends left and down from the
        // disk center, so the exact minimum escape is one unit along +X/+Z.
        Assert.True(box.TryGetCenteredCylinderContact(center, FixedQuaternion.Identity, Vector3d.Up,
            Fixed64.Two, Fixed64.One, out FixedContactAnchors primary));
        Assert.Equal(Fixed64.One, primary.Depth);
        Assert.Equal(Vector3d.Right, primary.Normal);
        Span<FixedContactLocalPoints> contacts = stackalloc FixedContactLocalPoints[4];
        Assert.True(box.TryGetCenteredCylinderContact(center, FixedQuaternion.Identity, Vector3d.Up,
            Fixed64.Two, Fixed64.One, contacts, out FixedContactAnchors manifold, out int count));
        Assert.Equal(primary, manifold);
        Assert.Equal(0, count);
    }

    private static FixedQuaternion CylinderRotation()
    {
        // Preserve the integer quaternion ratio exactly; the rational basis
        // rotates local +Y to (1,2,2)/3 without rounded axis components.
        const long scale = 784_150_157L;
        return new FixedQuaternion(Fixed64.FromRaw(2 * scale), Fixed64.Zero,
            Fixed64.FromRaw(-scale), Fixed64.FromRaw(5 * scale));
    }
}
