using FixedMathSharp.Geometry;
using System.Numerics;
using Xunit;

namespace FixedMathSharp.Tests;

public sealed class CenteredCylinderPairStrictRegressionTests
{
    [Theory]
    [InlineData(false)]
    [InlineData(true)]
    public void Contact_CommonPerpendicularTouchKeepsTheEarlierAnalyticNormal(bool reverse)
    {
        // The supporting plane z=1 meets A only at y=0 and B only at x=1.
        // Their unique common point (1,0,1) lies on both finite cap rims;
        // the earlier common-perpendicular candidate already owns depth0.
        Vector3d second = new(1, 1, 2);
        bool hit = reverse
            ? FixedSegment.TryGetCenteredFiniteCylindersContact(
                second, FixedQuaternion.Identity, Vector3d.Up, Fixed64.Two, Fixed64.One,
                Vector3d.Zero, FixedQuaternion.Identity, Vector3d.Right, Fixed64.Two, Fixed64.One,
                out FixedContactAnchors contact)
            : FixedSegment.TryGetCenteredFiniteCylindersContact(
                Vector3d.Zero, FixedQuaternion.Identity, Vector3d.Right, Fixed64.Two, Fixed64.One,
                second, FixedQuaternion.Identity, Vector3d.Up, Fixed64.Two, Fixed64.One, out contact);
        Assert.True(hit);
        Assert.Equal(Fixed64.Zero, contact.Depth);
        Assert.Equal(reverse ? -Vector3d.Forward : Vector3d.Forward, contact.Normal);
        Assert.False(contact.DepthIsClamped);
    }

    [Theory]
    [InlineData(-1L, false)]
    [InlineData(0L, false)]
    [InlineData(1L, false)]
    [InlineData(-1L, true)]
    [InlineData(0L, true)]
    [InlineData(1L, true)]
    public void Contact_SideRimTangencyPreservesOneRawNeighborsAndSwap(long offset, bool reverse)
    {
        // In the XZ plane the first cylinder reaches the corner (1,1).
        // The second disk is centered at (4,5+offset raw), with radius5.
        // At offset0 their common point is (1,0,1), with supporting normal
        // (3,0,4)/5. Squared separation is9+(4+offset raw)^2, so the
        // two raw neighbors are independently strictly inside/outside.
        Vector3d second = new((Fixed64)4, Fixed64.Zero, (Fixed64)5 + Fixed64.FromRaw(offset));
        bool hit = reverse
            ? FixedSegment.TryGetCenteredFiniteCylindersContact(
                second, FixedQuaternion.Identity, Vector3d.Up, Fixed64.Two, (Fixed64)5,
                Vector3d.Zero, FixedQuaternion.Identity, Vector3d.Right, Fixed64.Two, Fixed64.One,
                out FixedContactAnchors contact)
            : FixedSegment.TryGetCenteredFiniteCylindersContact(
                Vector3d.Zero, FixedQuaternion.Identity, Vector3d.Right, Fixed64.Two, Fixed64.One,
                second, FixedQuaternion.Identity, Vector3d.Up, Fixed64.Two, (Fixed64)5, out contact);
        Assert.Equal(offset <= 0, hit);
        if (!hit)
        {
            Assert.Equal(default, contact);
            return;
        }
        Assert.Equal(Fixed64.FromRaw(-offset), contact.Depth);
        Assert.False(contact.DepthIsClamped);
        if (offset == 0)
        {
            Vector3d normal = new(Fixed64.FromFraction(3, 5), Fixed64.Zero, Fixed64.FromFraction(4, 5));
            Assert.Equal(reverse ? -normal : normal, contact.Normal);
        }
    }

    [Fact]
    public void Contact_RejectsSeparatedPerpendicularRims()
    {
        // A requires x^2 + z^2 <= 1 and y <= 1. B requires x >= 3/4
        // and (y - 7/4)^2 + (z - 11/8)^2 <= 1. Their possible z
        // intervals are disjoint: (11/8)^2 > (sqrt(7)/2)^2.
        // Both axes, their cross product, and the closest-core direction
        // nevertheless have positive projection overlap.
        Assert.False(TryGetContact(
            Fixed64.One,
            (Fixed64)11 / 8,
            out _));
    }

    [Fact]
    public void Contact_OverlappingPerpendicularRimsDoNotExceedASeparatingTranslation()
    {
        // The same two z intervals overlap because (5/4)^2 < 7/4.
        Assert.True(TryGetContact(
            Fixed64.One,
            (Fixed64)5 / 4,
            out FixedContactAnchors contact));
        Assert.True(contact.Depth > Fixed64.Zero);
        // Direction (1,1,1) has normalized support gap
        // (2*sqrt(2)-11/4)/sqrt(3). Since sqrt(2)<99/70 and
        // sqrt(3)>5/3, that gap is below 33/700<1/20. A global
        // minimum cannot exceed this explicit separating translation.
        Assert.True(contact.Depth < Fixed64.FromFraction(1, 20));
    }

    [Theory]
    [InlineData(-1L, true)]
    [InlineData(1L, false)]
    public void Contact_DistinguishesOneRawRimOffsets(long offset, bool expected)
    {
        // Radius 5/4 and horizontal offset 3/4 leave exactly one unit
        // of z reach per disk: (5/4)^2 - (3/4)^2 = 1.
        Assert.Equal(expected, TryGetContact(
            (Fixed64)5 / 4,
            Fixed64.Two + Fixed64.FromRaw(offset),
            out _));
    }

    [Fact]
    public void Contact_ReportsZeroDepthAtExactRimTangency()
    {
        // The only common point is (3/4, 1, 1), on both cap rims.
        Assert.True(TryGetContact(
            (Fixed64)5 / 4,
            Fixed64.Two,
            out FixedContactAnchors contact));
        Assert.Equal(Fixed64.Zero, contact.Depth);
    }

    [Theory]
    [InlineData(false)]
    [InlineData(true)]
    public void Contact_RepeatedRimValueRetainsTheUniqueAdmissibleMinimum(bool reverse)
    {
        // Component ratio 1:2 makes the exact rational rotations independent
        // of the common dyadic scale: the two axes are (+/-3,0,4)/5.
        Fixed64 component = Fixed64.FromRaw(1_920_767_767L);
        FixedQuaternion firstRotation = new(Fixed64.Zero, -component,
            Fixed64.Zero, component + component);
        FixedQuaternion secondRotation = new(Fixed64.Zero, component,
            Fixed64.Zero, component + component);
        Vector3d secondCenter = new(0, 1, 8);

        // The ++ cap pair has two rim stationary normals with the same
        // squared gap 351/400. Its pencil is rank one, not a constant-distance
        // family. Only the positive-Z normal has admissible finite-cap signs.
        // The unique global normal is (0,15,sqrt(399))/(4*sqrt(39)); the
        // minimum gap is 3*sqrt(39)/20. See the exact certificate in #024's plan.
        const long depthRaw = 4_023_309_325L;
        const long yRaw = 2_579_044_439L;
        const long zRaw = 3_434_424_822L;
        AssertRoundedSquareRoot(depthRaw, 351, 400);
        AssertRoundedSquareRoot(yRaw, 75, 208);
        AssertRoundedSquareRoot(zRaw, 133, 208);

        bool hit = reverse
            ? FixedSegment.TryGetCenteredFiniteCylindersContact(
                secondCenter, secondRotation, Vector3d.Left, (Fixed64)10, Fixed64.One,
                Vector3d.Zero, firstRotation, Vector3d.Right, (Fixed64)10, Fixed64.One,
                out FixedContactAnchors contact)
            : FixedSegment.TryGetCenteredFiniteCylindersContact(
                Vector3d.Zero, firstRotation, Vector3d.Right, (Fixed64)10, Fixed64.One,
                secondCenter, secondRotation, Vector3d.Left, (Fixed64)10, Fixed64.One,
                out contact);
        Assert.True(hit);
        Assert.Equal(Fixed64.FromRaw(depthRaw), contact.Depth);
        Vector3d expectedNormal = new(Fixed64.Zero,
            Fixed64.FromRaw(yRaw), Fixed64.FromRaw(zRaw));
        Assert.Equal(reverse ? -expectedNormal : expectedNormal, contact.Normal);
        Assert.False(contact.DepthIsClamped);
        Assert.Equal(reverse ? secondCenter : Vector3d.Zero, contact.FirstAnchor.Origin);
        Assert.Equal(reverse ? Vector3d.Zero : secondCenter, contact.SecondAnchor.Origin);
    }

    [Fact]
    public void Contact_PenetratingRimBatchIsAllocationFreeAndKeepsMinimumDepth()
    {
        // Match the frozen benchmark geometry and its 64-call measured batch.
        // This guard measures the calling thread, not benchmark infrastructure.
        int hits = 0;
        FixedContactAnchors contact = default;
        long allocated = FixedMathTestHelper.MeasureWarmedAllocations(() =>
        {
            hits = 0;
            for (int index = 0; index < 64; index++)
                if (TryGetContact(Fixed64.One, Fixed64.FromFraction(5, 4), out contact))
                    hits++;
        });
        Assert.Equal(64, hits);
        Assert.Equal(0, allocated);
        Assert.True(contact.Depth > Fixed64.Zero);
        Assert.True(contact.Depth < Fixed64.FromFraction(1, 20));
        Assert.False(contact.DepthIsClamped);
    }

    private static void AssertRoundedSquareRoot(long raw, int numerator, int denominator)
    {
        // Strict midpoint bounds independently certify the expected raw
        // constant without using the production square-root/rounding helpers.
        BigInteger fourScaledSquare = (BigInteger)numerator << 66;
        BigInteger twiceRaw = (BigInteger)raw * 2;
        Assert.True((BigInteger)denominator * (twiceRaw - 1) * (twiceRaw - 1) < fourScaledSquare);
        Assert.True(fourScaledSquare < (BigInteger)denominator * (twiceRaw + 1) * (twiceRaw + 1));
    }

    private static bool TryGetContact(
        Fixed64 radius,
        Fixed64 secondZ,
        out FixedContactAnchors contact) =>
        FixedSegment.TryGetCenteredFiniteCylindersContact(
            Vector3d.Zero,
            FixedQuaternion.Identity,
            Vector3d.Up,
            Fixed64.Two,
            radius,
            new Vector3d((Fixed64)7 / 4, (Fixed64)7 / 4, secondZ),
            FixedQuaternion.Identity,
            Vector3d.Right,
            Fixed64.Two,
            radius,
            out contact);
}
