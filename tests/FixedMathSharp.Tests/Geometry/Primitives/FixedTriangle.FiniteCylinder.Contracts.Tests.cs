//=======================================================================
// MIT License, Copyright (c) 2024-present David Oravsky (mrdav30)
// See LICENSE file in the project root for full license information.
//=======================================================================
using System;
using FixedMathSharp.Geometry;
using FixedMathSharp.Random;
using Xunit;

namespace FixedMathSharp.Tests;

public sealed partial class FixedTriangleFiniteCylinderTests
{
    [Fact]
    public void Cylinder_SubRawRimMinima_RankBothStationaryValuesBeforeRounding()
    {
        Fixed64 raw = Fixed64.MinIncrement;
        var a = new Vector3d(3 * raw, -2 * raw, Fixed64.Zero);
        // Long edges preserve the sub-raw support chart without falling below
        // the public triangle-area degeneracy threshold.
        var triangle = new FixedTriangle(a, a + new Vector3d(-1, 7, 20), a + new Vector3d(40, -30, 0));
        const long k = 1920767767L;
        var rotation = new FixedQuaternion(Fixed64.Zero, Fixed64.Zero,
            Fixed64.FromRaw(k), Fixed64.FromRaw(2 * k));
        // The exact cylinder frame rotates XY by [3,4;-4,3]/5.
        // P=A+(B-A)*raw/50=(.3,-3.5,.4) raw is inside AB and centers a
        // half-raw-radius ball in the cylinder. Its only boundary normals
        // have nonzero projection on AB, making penetration strictly >.5 raw.
        // The negative-X support gap is .8 raw, so nearest-even depth is one
        // raw unit. Two admitted roots lie below the analytic half-raw bound;
        // both must be ranked exactly even though their rounded depths agree.
        Assert.True(triangle.TryGetCenteredFiniteCylinderContact(Vector3d.Zero, FixedQuaternion.Identity,
            Vector3d.Zero, rotation, 8 * raw, raw, out FixedContactAnchors contact));
        Assert.Equal(raw, contact.Depth);
        Assert.False(contact.DepthIsClamped);
        // The minimum's parameter is between 221/1000 and 222/1000.
        // These world-normal bounds distinguish it from the later maximum
        // and analytic candidates, even though their public depths round alike.
        FixedMathTestHelper.AssertWithinRange(contact.Normal.X, -Fixed64.FromFraction(17, 20), -Fixed64.FromFraction(21, 25));
        FixedMathTestHelper.AssertWithinRange(contact.Normal.Y, Fixed64.FromFraction(49, 100), Fixed64.Half);
        FixedMathTestHelper.AssertWithinRange(contact.Normal.Z, -Fixed64.FromFraction(11, 50), -Fixed64.FromFraction(21, 100));
        Assert.True(contact.FirstAnchor.TryGetPoint(out _));
        Assert.True(contact.SecondAnchor.TryGetPoint(out _));
    }

    [Fact]
    public void CapsuleSlab_ZeroCore_RejectsCertifiedObliqueCircleGap()
    {
        // Along n=(3/5,4/5,0), the minimum triangle support is
        // 5/16 beyond the cylinder rim. A zero core is precisely the circle.
        Assert.False(CreateObliqueRimTriangle().TryGetCenteredCapsuleSlabContact(
            new Vector3d(Fixed64.FromFraction(3, 8), Fixed64.Half, Fixed64.Zero),
            FixedQuaternion.Identity, Vector3d.Zero, Fixed64.Zero, Vector2d.Right,
            Fixed64.Zero, (Fixed64)5, (Fixed64)5, out _));
    }

    [Theory]
    [InlineData(false)]
    [InlineData(true)]
    public void CapsuleSlab_ZeroCore_HasIdenticalCircleAnchorsAndAuthoredYaw(bool yawed)
    {
        FixedTriangle triangle = CreateObliqueRimTriangle();
        Fixed64 yaw = yawed ? Fixed64.PiOver4 : Fixed64.Zero;
        Assert.True(triangle.TryGetCircleSlabContact(Vector3d.Zero, FixedQuaternion.Identity,
            Vector3d.Zero, yaw, (Fixed64)5, (Fixed64)5, out FixedContactAnchors circle));
        Assert.True(triangle.TryGetCenteredCapsuleSlabContact(Vector3d.Zero, FixedQuaternion.Identity,
            Vector3d.Zero, yaw, Vector2d.Forward, Fixed64.Zero, (Fixed64)5, (Fixed64)5,
            out FixedContactAnchors capsule));
        Assert.Equal(Fixed64.FromFraction(5, 16), capsule.Depth);
        Assert.Equal(circle, capsule);
    }

    [Fact]
    public void Cylinder_InvalidDimensionsAndFrames_RejectBeforeGeometry()
    {
        FixedTriangle triangle = CreateObliqueRimTriangle();
        Assert.Throws<ArgumentException>(() => triangle.TryGetCenteredFiniteCylinderContact(
            Vector3d.Zero, default, Vector3d.Zero, FixedQuaternion.Identity, Fixed64.One, Fixed64.One, out _));
        Assert.Throws<ArgumentException>(() => triangle.TryGetCenteredFiniteCylinderContact(
            Vector3d.Zero, FixedQuaternion.Identity, Vector3d.Zero, default, Fixed64.One, Fixed64.One, out _));
        Assert.Throws<ArgumentOutOfRangeException>(() => triangle.TryGetCenteredFiniteCylinderContact(
            Vector3d.Zero, FixedQuaternion.Identity, Vector3d.Zero, FixedQuaternion.Identity, Fixed64.Zero, Fixed64.One, out _));
        Assert.Throws<ArgumentOutOfRangeException>(() => triangle.TryGetCenteredFiniteCylinderContact(
            Vector3d.Zero, FixedQuaternion.Identity, Vector3d.Zero, FixedQuaternion.Identity, -Fixed64.One, Fixed64.One, out _));
        Assert.Throws<ArgumentOutOfRangeException>(() => triangle.TryGetCenteredFiniteCylinderContact(
            Vector3d.Zero, FixedQuaternion.Identity, Vector3d.Zero, FixedQuaternion.Identity, Fixed64.One, -Fixed64.MinIncrement, out _));
        Assert.False(new FixedTriangle(Vector3d.Zero, Vector3d.Up, Vector3d.Up)
            .TryGetCenteredFiniteCylinderContact(Vector3d.Zero, FixedQuaternion.Identity,
                Vector3d.Zero, FixedQuaternion.Identity, Fixed64.One, Fixed64.One, out _));
    }

    [Theory]
    [InlineData(-1)]
    [InlineData(1)]
    public void Cylinder_CapFaceAndSmallCapFace_RetainPairedCoordinates(int sign)
    {
        Fixed64 y = sign * Fixed64.FromFraction(19, 4);
        var triangle = new FixedTriangle(new Vector3d((Fixed64)2, y, (Fixed64)2),
            new Vector3d((Fixed64)3, y, (Fixed64)2), new Vector3d((Fixed64)2, y, (Fixed64)3));
        Assert.True(triangle.TryGetCenteredFiniteCylinderContact(Vector3d.Zero, FixedQuaternion.Identity,
            Vector3d.Zero, FixedQuaternion.Identity, (Fixed64)10, (Fixed64)5,
            out FixedContactAnchors contact, out bool capFace));
        Assert.True(capFace);
        Assert.Equal(new Vector3d(Fixed64.Zero, (Fixed64)(-sign), Fixed64.Zero), contact.Normal);
        Assert.Equal(Fixed64.FromFraction(1, 4), contact.Depth);
        Assert.True(contact.FirstAnchor.TryGetPoint(out Vector3d first));
        Assert.True(contact.SecondAnchor.TryGetPoint(out Vector3d second));
        Assert.Equal((Fixed64)2, first.X); Assert.Equal((Fixed64)2, first.Z);
        Assert.Equal(first.X, second.X); Assert.Equal(first.Z, second.Z);
        Assert.Equal(sign * (Fixed64)5, second.Y);
    }

    [Theory]
    [InlineData(1L, 0L)]
    [InlineData(3L, 2L)]
    public void Cylinder_OddRawHeight_RetainsExactHalfHeight(long rawHeight, long roundedDepth)
    {
        var triangle = new FixedTriangle(new Vector3d(-2, 0, -2), new Vector3d(2, 0, -2), new Vector3d(0, 0, 2));
        Assert.True(triangle.TryGetCenteredFiniteCylinderContact(Vector3d.Zero, FixedQuaternion.Identity,
            Vector3d.Zero, FixedQuaternion.Identity, Fixed64.FromRaw(rawHeight), Fixed64.One, out FixedContactAnchors contact));
        Assert.Equal(Fixed64.FromRaw(roundedDepth), contact.Depth);
        var roundedOnly = new FixedPointAnchor(contact.SecondAnchor.Origin, contact.SecondAnchor.Rotation,
            contact.SecondAnchor.LocalPoint, contact.SecondAnchor.LocalDisplacement);
        Assert.NotEqual(0, contact.SecondAnchor.CompareLocalFeature(roundedOnly));
    }

    [Fact]
    public void Cylinder_ZeroRadiusCore_CrossesTriangleAndRejectsSideGap()
    {
        var triangle = new FixedTriangle(new Vector3d(-16, 0, -16), new Vector3d(16, 0, -16), new Vector3d(0, 0, 16));
        Assert.True(triangle.TryGetCenteredFiniteCylinderContact(Vector3d.Zero, FixedQuaternion.Identity,
            Vector3d.Zero, FixedQuaternion.Identity, Fixed64.Two, Fixed64.Zero, out FixedContactAnchors contact));
        Assert.Equal(Fixed64.One, contact.Depth);
        Assert.True(contact.FirstAnchor.TryGetPoint(out Vector3d first));
        Assert.Equal(Vector3d.Zero, first);
        Assert.False(triangle.TryGetCenteredFiniteCylinderContact(Vector3d.Zero, FixedQuaternion.Identity,
            new Vector3d(30, 0, 0), FixedQuaternion.Identity, Fixed64.Two, Fixed64.Zero, out _));
    }

    [Fact]
    public void Cylinder_ZeroRadiusCoreWithinVerticalTriangle_ReturnsPairedSideWitnesses()
    {
        // The entire axial core lies in the triangle plane. Its zero radial
        // support wins over the positive cap gap, so this is a genuine side
        // contact whose selected normal must not create a nonzero witness.
        var triangle = new FixedTriangle(new Vector3d(0, -16, -16),
            new Vector3d(0, 16, -16), new Vector3d(0, 0, 16));
        Assert.True(triangle.TryGetCenteredFiniteCylinderContact(Vector3d.Zero, FixedQuaternion.Identity,
            Vector3d.Zero, FixedQuaternion.Identity, Fixed64.Two, Fixed64.Zero,
            out FixedContactAnchors contact, out bool capFace));
        Assert.Equal(Vector3d.Right, contact.Normal);
        Assert.Equal(Fixed64.Zero, contact.Depth);
        Assert.False(contact.DepthIsClamped);
        Assert.False(capFace);
        Assert.True(contact.FirstAnchor.TryGetPoint(out Vector3d first));
        Assert.True(contact.SecondAnchor.TryGetPoint(out Vector3d second));
        Assert.Equal(Vector3d.Zero, first);
        Assert.Equal(Vector3d.Zero, second);
    }

    [Fact]
    public void Cylinder_FullDomainRadius_RoundsRadialSupportOnlyOnce()
    {
        Fixed64 radius = (Fixed64)2000000000;
        var triangle = new FixedTriangle(new Vector3d(1199999996, -16, 1600000003),
            new Vector3d(1199999996, 16, 1600000003), new Vector3d(1200000004, 0, 1599999997));
        Assert.True(triangle.TryGetCenteredFiniteCylinderContact(Vector3d.Zero, FixedQuaternion.Identity,
            Vector3d.Zero, FixedQuaternion.Identity, (Fixed64)10, radius, out FixedContactAnchors contact));
        Assert.Equal(Fixed64.Zero, contact.Depth);
        Assert.Equal(new Vector3d(-Fixed64.FromFraction(3, 5), Fixed64.Zero, -Fixed64.FromFraction(4, 5)), contact.Normal);
        Assert.True(contact.FirstAnchor.TryGetPoint(out Vector3d first));
        Assert.True(contact.SecondAnchor.TryGetPoint(out Vector3d second));
        Assert.Equal(new Vector3d(1200000000, 0, 1600000000), first);
        Assert.Equal(first, second);
    }

    [Fact]
    public void Cylinder_FullDomainCapRimTouch_DoesNotRenormalizeRoundedWitness()
    {
        const long k = 1844674407370955161;
        var point = new Vector3d(Fixed64.FromRaw(3 * k + 2), (Fixed64)5, Fixed64.FromRaw(4 * k + 1));
        var triangle = new FixedTriangle(point + new Vector3d(4, 0, -3),
            point - new Vector3d(4, 0, -3), point + new Vector3d(3, 0, 4));
        Assert.True(triangle.TryGetCenteredFiniteCylinderContact(Vector3d.Zero, FixedQuaternion.Identity,
            Vector3d.Zero, FixedQuaternion.Identity, (Fixed64)10, Fixed64.MaxValue, out FixedContactAnchors contact));
        Assert.Equal(Vector3d.Down, contact.Normal);
        Assert.Equal(Fixed64.Zero, contact.Depth);
        Assert.True(contact.FirstAnchor.TryGetPoint(out Vector3d first));
        Assert.True(contact.SecondAnchor.TryGetPoint(out Vector3d second));
        var expected = new Vector3d(Fixed64.FromRaw(3 * k + 1), (Fixed64)5, Fixed64.FromRaw(4 * k + 2));
        Assert.Equal(expected, first); Assert.Equal(expected, second);
    }

    [Theory]
    [InlineData(false)]
    [InlineData(true)]
    public void CircleSlab_FractionalRawDepthAtMaximum_ClampsBeforeRounding(bool aboveMaximum)
    {
        // This accepted normalized quaternion represents the exact rational
        // XY rotation [3,-4;4,3]/5. The small authored offset has world Y=2/5
        // raw, so both exact depths round to Max but only Max+2/5 is clamped.
        const long k = 1920767767L;
        var rotation = new FixedQuaternion(Fixed64.Zero, Fixed64.Zero,
            Fixed64.FromRaw(k), Fixed64.FromRaw(2 * k));
        Vector3d offset = new(-Fixed64.MinIncrement, Fixed64.FromRaw(2), Fixed64.Zero);
        Vector3d edge = new(3, -4, 0), transverse = new(0, 0, 2);
        Vector3d edgeCenter = aboveMaximum ? -offset : offset;
        var triangle = new FixedTriangle(edgeCenter - edge - transverse,
            edgeCenter + edge - transverse, offset + transverse);
        Assert.True(triangle.TryGetCircleSlabContact(Vector3d.Zero, rotation, Vector3d.Zero,
            Fixed64.Zero, Fixed64.MaxValue, Fixed64.MaxValue, out FixedContactAnchors contact));
        Assert.Equal(Fixed64.MaxValue, contact.Depth);
        Assert.Equal(aboveMaximum, contact.DepthIsClamped);
        Assert.Equal(aboveMaximum ? Vector3d.Up : Vector3d.Down, contact.Normal);
    }

    [Fact]
    public void Cylinder_LongSupportEdge_DoesNotRoundBarycentricParameterFirst()
    {
        Vector3d q = new(Fixed64.FromFraction(77, 16), Fixed64.FromFraction(19, 4), Fixed64.Zero);
        Vector3d offset = new(536870912, -402653184, 671088640);
        var triangle = new FixedTriangle(q - offset, q + offset * Fixed64.Two, q + new Vector3d(3, 4, 0));
        Assert.True(triangle.TryGetCenteredFiniteCylinderContact(Vector3d.Zero, FixedQuaternion.Identity,
            Vector3d.Zero, FixedQuaternion.Identity, (Fixed64)10, (Fixed64)5, out FixedContactAnchors contact));
        Assert.Equal(Fixed64.FromFraction(5, 16), contact.Depth);
        Assert.True(contact.FirstAnchor.TryGetPoint(out Vector3d first));
        Assert.True(contact.SecondAnchor.TryGetPoint(out Vector3d second));
        Assert.Equal(q, first); Assert.Equal(new Vector3d(5, 5, 0), second);
    }

    [Theory]
    [InlineData(false, -1, -1)]
    [InlineData(false, -1, 1)]
    [InlineData(false, 1, -1)]
    [InlineData(false, 1, 1)]
    [InlineData(true, -1, -1)]
    [InlineData(true, -1, 1)]
    [InlineData(true, 1, -1)]
    [InlineData(true, 1, 1)]
    public void Cylinder_ObliqueRim_EnumeratesBothChartsAndSupportCones(bool turn, int mirrorX, int mirrorY)
    {
        FixedTriangle source = CreateObliqueRimTriangle();
        var triangle = new FixedTriangle(Turn(source.A), Turn(source.B), Turn(source.C));
        Vector3d expectedNormal = Turn(new Vector3d(-Fixed64.FromFraction(3, 5), -Fixed64.FromFraction(4, 5), Fixed64.Zero));
        Vector3d expectedFirst = Turn(new Vector3d(Fixed64.FromFraction(77, 16), Fixed64.FromFraction(19, 4), Fixed64.Zero));
        Vector3d expectedSecond = Turn(new Vector3d(5, 5, 0));
        for (int order = 0; order < 3; order++)
        {
            Assert.True(triangle.TryGetCenteredFiniteCylinderContact(Vector3d.Zero, FixedQuaternion.Identity,
                Vector3d.Zero, FixedQuaternion.Identity, (Fixed64)10, (Fixed64)5, out FixedContactAnchors contact));
            Assert.Equal(expectedNormal, contact.Normal);
            Assert.Equal(Fixed64.FromFraction(5, 16), contact.Depth);
            Assert.True(contact.FirstAnchor.TryGetPoint(out Vector3d first));
            Assert.True(contact.SecondAnchor.TryGetPoint(out Vector3d second));
            Assert.Equal(expectedFirst, first); Assert.Equal(expectedSecond, second);
            triangle = new FixedTriangle(triangle.B, triangle.C, triangle.A);
        }
        Vector3d Turn(Vector3d point) => turn
            ? new Vector3d(point.Z, mirrorY * point.Y, -mirrorX * point.X)
            : new Vector3d(mirrorX * point.X, mirrorY * point.Y, point.Z);
    }

    [Fact]
    public void Cylinder_SharedArbitraryRigidFrame_PreservesAuthoredWitnesses()
    {
        var rotation = new FixedQuaternion(Fixed64.One, Fixed64.Two, (Fixed64)3, (Fixed64)4).Normalized;
        var origin = new Vector3d(Fixed64.MaxValue, Fixed64.MinValue, Fixed64.MaxValue);
        FixedTriangle triangle = CreateObliqueRimTriangle();
        Assert.True(triangle.TryGetCenteredFiniteCylinderContact(origin, rotation, origin, rotation,
            (Fixed64)10, (Fixed64)5, out FixedContactAnchors contact));
        Assert.Equal(Fixed64.FromFraction(5, 16), contact.Depth);
        Assert.Equal(origin, contact.FirstAnchor.Origin); Assert.Equal(origin, contact.SecondAnchor.Origin);
        Assert.Equal(rotation, contact.FirstAnchor.Rotation); Assert.Equal(rotation, contact.SecondAnchor.Rotation);
        Assert.Equal(new Vector3d(Fixed64.FromFraction(77, 16), Fixed64.FromFraction(19, 4), Fixed64.Zero), contact.FirstAnchor.LocalPoint);
        Assert.Equal(new Vector3d(5, 5, 0), contact.SecondAnchor.LocalPoint + contact.SecondAnchor.LocalDisplacement);
    }

    [Fact]
    public void CircleSlab_YawFrame_PreservesCanonicalWorldContactAndAuthoredRotation()
    {
        FixedTriangle triangle = CreateObliqueRimTriangle();
        Fixed64 yaw = Fixed64.PiOver4;
        Assert.True(triangle.TryGetCircleSlabContact(Vector3d.Zero, FixedQuaternion.Identity,
            Vector3d.Zero, yaw, (Fixed64)5, (Fixed64)5, out FixedContactAnchors contact));
        Assert.Equal(Fixed64.FromFraction(5, 16), contact.Depth);
        Assert.Equal(FixedQuaternion.FromAxisAngle(Vector3d.Up, -yaw), contact.SecondAnchor.Rotation);
        Assert.True(contact.SecondAnchor.TryGetPoint(out Vector3d second));
        Assert.InRange(FixedMath.Abs(second.X - (Fixed64)5).m_rawValue, 0L, 2L);
        Assert.InRange(FixedMath.Abs(second.Z).m_rawValue, 0L, 2L);
    }

    [Fact]
    public void Cylinder_DeterministicFixtures_AgreeWithIndependentClippedIntrusionPredicate()
    {
        var random = new DeterministicRandom(0x082C71UL);
        for (int index = 0; index < 48; index++)
        {
            var triangle = new FixedTriangle(Point(), Point(), Point());
            if (triangle.IsDegenerate)
                continue;
            bool overlap = triangle.TryGetCenteredFiniteCylinderContact(Vector3d.Zero, FixedQuaternion.Identity,
                Vector3d.Zero, FixedQuaternion.Identity, (Fixed64)8, (Fixed64)3, out FixedContactAnchors contact);
            bool strict = WideOrientedBox.DoesCenteredCylinderPenetrateTriangle(Vector3d.Zero,
                FixedQuaternion.Identity, (Fixed64)8, (Fixed64)3, triangle, Vector3d.Zero, FixedQuaternion.Identity);
            if (strict)
                Assert.True(overlap, $"Strict fixture {index} was missed: {triangle.A}; {triangle.B}; {triangle.C}");
            else if (overlap)
                Assert.Equal(Fixed64.Zero, contact.Depth);
            if (overlap)
                Assert.True(contact.Normal.IsNormalized());
        }
        Vector3d Point() => new(random.Next(-9, 10), random.Next(-9, 10), random.Next(-9, 10));
    }

    [Fact]
    public void Cylinder_WarmedObliqueAndSideContacts_DoNotAllocate()
    {
        FixedTriangle triangle = CreateObliqueRimTriangle();
        FixedTriangle interiorRoot = CreateInteriorRimTriangle();
        var side = new FixedTriangle(new Vector3d(4, -4, -4), new Vector3d(4, 4, 4), new Vector3d(8, 0, 0));
        bool succeeded = true;
        long allocation = FixedMathTestHelper.MeasureWarmedAllocations(() =>
        {
            succeeded &= triangle.TryGetCenteredFiniteCylinderContact(Vector3d.Zero, FixedQuaternion.Identity,
                Vector3d.Zero, FixedQuaternion.Identity, (Fixed64)10, (Fixed64)5, out _);
            succeeded &= side.TryGetCenteredFiniteCylinderContact(Vector3d.Zero, FixedQuaternion.Identity,
                Vector3d.Zero, FixedQuaternion.Identity, (Fixed64)10, (Fixed64)5, out _);
            succeeded &= interiorRoot.TryGetCenteredFiniteCylinderContact(Vector3d.Zero, FixedQuaternion.Identity,
                Vector3d.Zero, FixedQuaternion.Identity, (Fixed64)10, (Fixed64)5, out _);
        });
        Assert.True(succeeded); Assert.Equal(0, allocation);
    }
}
