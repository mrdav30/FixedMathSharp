using System;
using FixedMathSharp.Geometry;
using Xunit;

namespace FixedMathSharp.Tests.Bounds;

public sealed class FixedConvexPrismCanonicalContactsTests
{
    [Theory]
    [InlineData(0, false, false)]
    [InlineData(0, true, false)]
    [InlineData(1, false, false)]
    [InlineData(1, true, false)]
    [InlineData(2, false, false)]
    [InlineData(2, true, false)]
    [InlineData(0, false, true)]
    [InlineData(1, false, true)]
    [InlineData(2, false, true)]
    public void EqualCubeExits_RetainTheFirstAxisAndBothPoleAnchors(int axisIndex, bool subdivided, bool reverse)
    {
        Vector3d axis = axisIndex == 0 ? Vector3d.Up : axisIndex == 1 ? Vector3d.Right : Vector3d.Forward;
        Vector2d[] footprint = subdivided
            ? new Vector2d[] { new(-1, -1), new(0, -1), new(1, -1), new(1, 0), new(1, 1), new(0, 1), new(-1, 1), new(-1, 0) }
            : new Vector2d[] { new(-1, -1), new(1, -1), new(1, 1), new(-1, 1) };
        if (reverse) Array.Reverse(footprint);
        Assert.True(FixedConvexPrismRelations.TryGetCenteredCylinderContact(Vector3d.Zero,
            FixedQuaternion.Identity, axis, Fixed64.Two, Fixed64.One, Vector3d.Zero,
            Fixed64.Zero, footprint, Fixed64.One, out var contact));
        // The cube adds at least |axial|+|radial| to the cylinder support;
        // every gap is >=2, and all coordinate exits attain 2. The authored
        // positive axis is first and its center witnesses are exactly paired.
        Assert.Equal(Fixed64.Two, contact.Depth);
        Assert.False(contact.DepthIsClamped);
        Assert.Equal(axis, contact.Normal);
        Assert.Equal(new FixedPointAnchor(Vector3d.Zero, FixedQuaternion.Identity, axis), contact.FirstAnchor);
        Assert.Equal(new FixedPointAnchor(Vector3d.Zero, FixedQuaternion.Identity, -axis), contact.SecondAnchor);
        Assert.True(FixedConvexPrismRelations.TryGetCenteredCylinderContact(-axis * Fixed64.Two,
            FixedQuaternion.Identity, axis, Fixed64.Two, Fixed64.One, Vector3d.Zero,
            Fixed64.Zero, footprint, Fixed64.One, out var touching));
        Assert.Equal(Fixed64.Zero, touching.Depth);
        Assert.False(FixedConvexPrismRelations.TryGetCenteredCylinderContact(
            -axis * (Fixed64.Two + Fixed64.FromRaw(1)), FixedQuaternion.Identity, axis,
            Fixed64.Two, Fixed64.One, Vector3d.Zero, Fixed64.Zero, footprint, Fixed64.One, out _));
    }

    [Theory]
    [InlineData(-1L, false, false)]
    [InlineData(0L, false, false)]
    [InlineData(1L, false, false)]
    [InlineData(67108864L, false, false)]
    [InlineData(-1L, true, false)]
    [InlineData(0L, true, false)]
    [InlineData(1L, true, false)]
    [InlineData(67108864L, true, false)]
    [InlineData(-1L, false, true)]
    [InlineData(0L, false, true)]
    [InlineData(1L, false, true)]
    [InlineData(67108864L, false, true)]
    public void UniqueSideCrossDirection_RetainsTheMinimumAndExactPairedWitnesses(long shiftRaw, bool reverse, bool largeOrigin)
    {
        const long k = 858993459;
        Vector3d axis = new(Fixed64.FromRaw(3 * k), Fixed64.FromRaw(4 * k), Fixed64.Zero);
        Vector2d[] footprint = { new(0, -2), new(2, -2), new(2, 2), new(0, 2) };
        if (reverse) Array.Reverse(footprint);
        Vector3d origin = largeOrigin ? new Vector3d(Fixed64.MaxValue - 32, Fixed64.MinValue + 32, Fixed64.MaxValue - 32) : Vector3d.Zero;
        Fixed64 shift = Fixed64.FromRaw(shiftRaw);
        Vector3d displacement = new((Fixed64)4 * shift, (Fixed64)(-3) * shift, Fixed64.Zero);
        Vector3d center = origin + new Vector3d((Fixed64)(-4), Fixed64.FromFraction(7, 2), Fixed64.Zero) + displacement;
        bool hit = FixedConvexPrismRelations.TryGetCenteredCylinderContact(center, FixedQuaternion.Identity,
            axis, Fixed64.Two, (Fixed64)5, origin, Fixed64.Zero, footprint, Fixed64.Half, out var contact);
        Assert.Equal(shiftRaw >= 0, hit);
        if (!hit) { Assert.Equal(default, contact); return; }
        // Axis (3,4,0)/5 and edge Z have unique side normal (4,-3,0)/5.
        // Radius5 gives the exact radial offset (4,-3,0). The prism edge
        // point P=(0,1/2,0) lies inside the cylinder by 5*shift; a ball of
        // that radius fits there, while the supporting plane bounds the exit.
        Assert.False(contact.DepthIsClamped);
        Assert.Equal((Fixed64)5 * shift, contact.Depth);
        Assert.Equal(new Vector3d(Fixed64.FromFraction(4, 5), -Fixed64.FromFraction(3, 5), Fixed64.Zero), contact.Normal);
        Assert.Equal(new FixedPointAnchor(center, FixedQuaternion.Identity, new Vector3d(4, -3, 0)), contact.FirstAnchor);
        Assert.Equal(new FixedPointAnchor(origin, FixedQuaternion.Identity, new Vector3d(Fixed64.Zero, Fixed64.Half, Fixed64.Zero)), contact.SecondAnchor);
        Assert.True(FixedConvexPrismRelations.TryGetCenteredCylinderContact(center - displacement,
            FixedQuaternion.Identity, axis, Fixed64.Two, (Fixed64)5, origin, Fixed64.Zero,
            footprint, Fixed64.Half, out var touching));
        Assert.Equal(Fixed64.Zero, touching.Depth);
        Vector3d step = new(Fixed64.FromRaw(4), Fixed64.FromRaw(-3), Fixed64.Zero);
        Assert.False(FixedConvexPrismRelations.TryGetCenteredCylinderContact(center - displacement - step,
            FixedQuaternion.Identity, axis, Fixed64.Two, (Fixed64)5, origin, Fixed64.Zero,
            footprint, Fixed64.Half, out _));
    }

    [Fact]
    public void OffsetDirectionOwnsTheEarlierEqualXExitForANondegenerateEllipse()
    {
        const long k = 1_920_767_767;
        FixedQuaternion rotation = new(Fixed64.Zero, Fixed64.FromRaw(k), Fixed64.Zero, Fixed64.FromRaw(2 * k));
        Fixed64 a = Fixed64.FromRaw((1L << 32) - 1), b = Fixed64.FromRaw(1);
        Vector3d axis = new(-a, b, Fixed64.Zero);
        Vector3d center = new((Fixed64)21, Fixed64.Zero, (Fixed64)72 * a);
        Vector2d[] footprint = { new(0, 0), new(42, 0), new(42, 200), new(0, 200) };
        Fixed64 expectedDepth = Fixed64.FromRaw((155L << 32) - 54);

        Assert.True(FixedConvexPrismRelations.TryGetCenteredCylinderContact(center, rotation, axis,
            (Fixed64)180, (Fixed64)100, Vector3d.Zero, Fixed64.Zero, footprint, (Fixed64)1000, out var contact));
        // The two X gaps tie exactly because both X centers are21. Their
        // common raw value is 155*Q-54 plus a positive correction <1e-8raw.
        // The first vertical edge's negative-cap offset supplies +X before
        // the -X wall. Both edges are genuinely oblique to the authored axis.
        Assert.Equal(expectedDepth, contact.Depth);
        Assert.False(contact.DepthIsClamped);
        Assert.Equal(Vector3d.Right, contact.Normal);
        // Exact rim ratios round to these bounded primitive-frame coordinates.
        // The rational yaw sends (-48,0,36) to the prism point (0,0,60).
        Assert.Equal(new FixedPointAnchor(center, rotation,
            new Vector3d((Fixed64)90 * a, Fixed64.FromRaw(-15), (Fixed64)100)), contact.FirstAnchor);
        Assert.Equal(new FixedPointAnchor(Vector3d.Zero, rotation,
            new Vector3d((Fixed64)(-48), Fixed64.FromRaw(-15), (Fixed64)36)), contact.SecondAnchor);

        Vector3d exited = center - Vector3d.Right * expectedDepth;
        Assert.True(FixedConvexPrismRelations.TryGetCenteredCylinderContact(exited, rotation, axis,
            (Fixed64)180, (Fixed64)100, Vector3d.Zero, Fixed64.Zero, footprint, (Fixed64)1000, out var touching));
        // Rounded exit leaves less than1e-8raw positive gap; it reports zero.
        Assert.Equal(Fixed64.Zero, touching.Depth);
        Assert.False(FixedConvexPrismRelations.TryGetCenteredCylinderContact(
            exited - new Vector3d(Fixed64.FromRaw(1), Fixed64.Zero, Fixed64.Zero), rotation, axis,
            (Fixed64)180, (Fixed64)100, Vector3d.Zero, Fixed64.Zero, footprint, (Fixed64)1000, out _));
    }

    [Theory]
    [InlineData(false, false)]
    [InlineData(false, true)]
    [InlineData(true, false)]
    [InlineData(true, true)]
    public void VerticalEdgeConcavityKeepsTheExactMinimumAndWitnesses(bool cone, bool reverse)
    {
        const long k = 858_993_459;
        Vector3d axis = new(Fixed64.FromRaw(3 * k), Fixed64.FromRaw(4 * k), Fixed64.Zero);
        Vector3d center = new(cone ? axis.X : -axis.X, Fixed64.Zero, Fixed64.Zero);
        Vector2d[] footprint = { new(Fixed64.FromFraction(-3, 2), (Fixed64)(-1)), new(10, -1), new(10, 10), new(Fixed64.FromFraction(-3, 2), (Fixed64)10) };
        if (reverse) Array.Reverse(footprint);
        bool Query(Vector3d position, out FixedContactAnchors result) => cone
            ? FixedConvexPrismRelations.TryGetCenteredConeContact(position,
                FixedQuaternion.Identity, axis, Fixed64.Two, Fixed64.Two,
                Vector3d.Zero, Fixed64.Zero, footprint, (Fixed64)10, out result)
            : FixedConvexPrismRelations.TryGetCenteredCylinderContact(position,
                FixedQuaternion.Identity, axis, Fixed64.Two, Fixed64.Two,
                Vector3d.Zero, Fixed64.Zero, footprint, (Fixed64)10, out result);
        Assert.True(Query(center, out var contact));
        Assert.Equal((Fixed64)3, contact.Depth);
        Assert.Equal(Vector3d.Forward, contact.Normal);
        Assert.False(contact.DepthIsClamped);
        Assert.Equal(new FixedPointAnchor(center, FixedQuaternion.Identity,
            -axis + new Vector3d(0, 0, 2)), contact.FirstAnchor);
        Assert.Equal(new FixedPointAnchor(Vector3d.Zero, FixedQuaternion.Identity,
            new Vector3d(cone ? Fixed64.Zero : -Fixed64.Two * axis.X, -axis.Y, (Fixed64)(-1))), contact.SecondAnchor);
        Vector3d touchingCenter = center - new Vector3d(0, 0, 3);
        Assert.True(Query(touchingCenter, out var touching));
        Assert.Equal(Fixed64.Zero, touching.Depth);
        Assert.False(Query(touchingCenter - new Vector3d(Fixed64.Zero, Fixed64.Zero, Fixed64.FromRaw(1)), out _));
    }
}
