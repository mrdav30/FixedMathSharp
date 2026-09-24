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
    public void Contact_AcceptsOverlappingPerpendicularRims()
    {
        // The same two z intervals overlap because (5/4)^2 < 7/4.
        Assert.True(TryGetContact(
            Fixed64.One,
            (Fixed64)5 / 4,
            out FixedContactAnchors contact));
        Assert.True(contact.Depth > Fixed64.Zero);
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
