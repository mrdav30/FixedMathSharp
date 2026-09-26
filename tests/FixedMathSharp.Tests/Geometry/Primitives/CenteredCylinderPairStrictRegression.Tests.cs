using FixedMathSharp.Geometry;
using Xunit;

namespace FixedMathSharp.Tests;

public sealed class CenteredCylinderPairStrictRegressionTests
{
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
