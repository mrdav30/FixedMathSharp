using System.Numerics;
using FixedMathSharp.Geometry;
using Xunit;

namespace FixedMathSharp.Tests;

public sealed class CenteredCylinderPairParallelTests
{
    [Theory]
    [InlineData(0, 2)]
    [InlineData(3, 2)]
    [InlineData(9, 1)]
    [InlineData(10, 0)]
    public void Contact_CoaxialPairsChooseTheSmallerRadialOrAxialTranslation(int centerHeight, int expectedDepth)
    {
        // The difference body is a cylinder of radius 2 and half-length 10.
        // Along its axis, the nearest boundary is min(2, 10-centerHeight).
        Assert.True(FixedSegment.TryGetCenteredFiniteCylindersContact(
            Vector3d.Zero, FixedQuaternion.Identity, Vector3d.Up, (Fixed64)10, Fixed64.One,
            new Vector3d(0, centerHeight, 0), FixedQuaternion.Identity, Vector3d.Up,
            (Fixed64)10, Fixed64.One, out FixedContactAnchors contact));
        Assert.Equal((Fixed64)expectedDepth, contact.Depth);
        Assert.False(contact.DepthIsClamped);
        Assert.Equal(expectedDepth == 2 ? Vector3d.Right : Vector3d.Up, contact.Normal);
    }

    [Fact]
    public void Contact_RotatedAntiparallelCoaxialPairsRetainTheAuthoredRadialRepresentative()
    {
        // This authored quaternion rotates local right to (3/5,0,-4/5).
        // The centers differ by exactly five units along that common axis;
        // radial translation 2 is shorter than axial translation 5.
        Fixed64 s = Fixed64.FromRaw(1920767767);
        var rotation = new FixedQuaternion(Fixed64.Zero, s, Fixed64.Zero, 2 * s);
        Assert.True(FixedSegment.TryGetCenteredFiniteCylindersContact(
            Vector3d.Zero, rotation, Vector3d.Right, (Fixed64)10, Fixed64.One,
            new Vector3d(3, 0, -4), rotation, Vector3d.Left, (Fixed64)10, Fixed64.One,
            out FixedContactAnchors contact));
        Assert.Equal(Fixed64.Two, contact.Depth);
        Assert.Equal(Vector3d.Down, contact.Normal);
        Assert.False(contact.DepthIsClamped);
    }

    [Fact]
    public void Contact_NearUnitAxialDepthRoundsBelowTheExactMidpoint()
    {
        const long q = 1L << 32;
        const long lengthRaw = (1L << 62) - 3 * (1L << 29);
        const long expectedRaw = (1L << 62) - 5 * (1L << 29);
        Vector3d axis = new(Fixed64.FromRaw(q - 1), Fixed64.MinIncrement, Fixed64.Zero);
        // At coincident centers, axial depth is L*sqrt((Q-1)^2+1)/Q.
        // It is the smaller translation, and lies strictly inside the
        // rounding cell below the midpoint that the old estimate crosses.
        BigInteger axisSquare = (BigInteger)(q - 1) * (q - 1) + 1;
        BigInteger scaledDepthSquare = (BigInteger)lengthRaw * lengthRaw * axisSquare;
        BigInteger qSquare = (BigInteger)q * q;
        BigInteger twice = 2 * (BigInteger)expectedRaw;
        Assert.True(qSquare * (twice - 1) * (twice - 1) < 4 * scaledDepthSquare);
        Assert.True(4 * scaledDepthSquare < qSquare * (twice + 1) * (twice + 1));
        Assert.True(scaledDepthSquare < 4 * (BigInteger)long.MaxValue * long.MaxValue * qSquare);

        AssertCoincidentContact(axis, Fixed64.FromRaw(lengthRaw), Fixed64.MaxValue, Fixed64.MaxValue,
            expectedRaw, clamped: false);
    }

    [Fact]
    public void Contact_NearUnitAxesPreserveTheExactPerpendicularDepthInBothOrders()
    {
        const long q = 1L << 32;
        Vector3d axis = new(Fixed64.FromRaw(q - 1), Fixed64.MinIncrement, Fixed64.Zero);
        Assert.True(axis.IsNormalized());
        // Z is exactly perpendicular to the authored axis. The difference
        // body has radial depth 1+1-1=1; its axial half-length exceeds 3.
        Assert.True(16 * ((BigInteger)(q - 1) * (q - 1) + 1) > 9 * (BigInteger)q * q);
        Assert.True(FixedSegment.TryGetCenteredFiniteCylindersContact(
            Vector3d.Zero, FixedQuaternion.Identity, axis, (Fixed64)4, Fixed64.One,
            Vector3d.Forward, FixedQuaternion.Identity, axis, (Fixed64)4, Fixed64.One,
            out FixedContactAnchors forward));
        Assert.True(FixedSegment.TryGetCenteredFiniteCylindersContact(
            Vector3d.Forward, FixedQuaternion.Identity, axis, (Fixed64)4, Fixed64.One,
            Vector3d.Zero, FixedQuaternion.Identity, axis, (Fixed64)4, Fixed64.One,
            out FixedContactAnchors reverse));
        Assert.Equal(Fixed64.One, forward.Depth);
        Assert.Equal(forward.Depth, reverse.Depth);
        Assert.False(forward.DepthIsClamped);
        Assert.False(reverse.DepthIsClamped);
        Assert.Equal(Vector3d.Forward, forward.Normal);
        Assert.Equal(-forward.Normal, reverse.Normal);
        Assert.Equal(Vector3d.Zero, forward.FirstAnchor.Origin);
        Assert.Equal(Vector3d.Forward, forward.SecondAnchor.Origin);
        Assert.Equal(forward.SecondAnchor.Origin, reverse.FirstAnchor.Origin);
        Assert.Equal(forward.FirstAnchor.Origin, reverse.SecondAnchor.Origin);
    }

    [Fact]
    public void Contact_OverflowingRadiusSumCanBeTheMinimumParallelTranslation()
    {
        const long q = 1L << 32;
        Vector3d axis = new(Fixed64.FromRaw(q + 1), Fixed64.Zero, Fixed64.Zero);
        // The radial translation is exactly M+1 raw units. The slightly
        // longer authored axis makes the cap depth M*(Q+1)/Q still larger.
        BigInteger maximum = long.MaxValue;
        Assert.True(maximum * (q + 1) > (maximum + 1) * q);
        AssertCoincidentContact(axis, Fixed64.MaxValue, Fixed64.MaxValue, Fixed64.MinIncrement,
            long.MaxValue, clamped: true);
        AssertCoincidentContact(axis, Fixed64.MaxValue, Fixed64.MinIncrement, Fixed64.MaxValue,
            long.MaxValue, clamped: true);
    }

    [Fact]
    public void Contact_SubHalfRawExcessAboveMaximumStillReportsConceptualClamping()
    {
        const long q = 1L << 32;
        Vector3d axis = new(Fixed64.One, Fixed64.MinIncrement, Fixed64.Zero);
        // The axial minimum M*sqrt(Q²+1)/Q is strictly above M but
        // below M+1/2. Rounding to M must not erase conceptual clamping.
        BigInteger maximum = long.MaxValue;
        BigInteger qSquare = (BigInteger)q * q;
        BigInteger scaledDepthSquare = maximum * maximum * (qSquare + 1);
        Assert.True(scaledDepthSquare > maximum * maximum * qSquare);
        Assert.True(4 * scaledDepthSquare < (2 * maximum + 1) * (2 * maximum + 1) * qSquare);
        Assert.True(scaledDepthSquare < 4 * maximum * maximum * qSquare);
        AssertCoincidentContact(axis, Fixed64.MaxValue, Fixed64.MaxValue, Fixed64.MaxValue,
            long.MaxValue, clamped: true);
    }

    [Fact]
    public void Contact_PositiveParallelRadiiPreserveExactMaximumWithoutClamping()
    {
        // Both radii are positive: this is the parallel-cylinder authority,
        // not the segment reduction. Cap depth M is below radial depth 2M.
        AssertCoincidentContact(Vector3d.Right, Fixed64.MaxValue, Fixed64.MaxValue, Fixed64.MaxValue,
            long.MaxValue, clamped: false);
    }

    private static void AssertCoincidentContact(Vector3d axis, Fixed64 length,
        Fixed64 firstRadius, Fixed64 secondRadius, long expectedRaw, bool clamped)
    {
        Assert.True(axis.IsNormalized());
        Assert.True(FixedSegment.TryGetCenteredFiniteCylindersContact(
            Vector3d.Zero, FixedQuaternion.Identity, axis, length, firstRadius,
            Vector3d.Zero, FixedQuaternion.Identity, axis, length, secondRadius,
            out FixedContactAnchors contact));
        Assert.Equal(expectedRaw, contact.Depth.m_rawValue);
        Assert.Equal(clamped, contact.DepthIsClamped);
        Assert.True(contact.Normal.IsNormalized());
        Assert.Equal(Vector3d.Zero, contact.FirstAnchor.Origin);
        Assert.Equal(Vector3d.Zero, contact.SecondAnchor.Origin);
    }
}
