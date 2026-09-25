using System.Numerics;
using FixedMathSharp.Geometry;
using Xunit;

namespace FixedMathSharp.Tests;

public sealed class CenteredCylinderPairDepthRoundingTests
{
    [Theory]
    [InlineData(3L, 4L, 8_589_934_587L)]
    [InlineData(4_294_967_297L, 4_294_967_297L, 2_515_933_591L)]
    [InlineData(4_294_967_300L, 4_294_967_300L, 2_515_933_586L)]
    public void Contact_PreservesExactAndOneRawCorrectionResults(long xRaw, long zRaw, long expectedRaw)
    {
        // Parallel cores select 2-sqrt(x*x+z*z). The retained estimates need
        // zero, one upward, and one downward raw correction respectively.
        BigInteger squared = (BigInteger)xRaw * xRaw + (BigInteger)zRaw * zRaw;
        BigInteger twiceDistance = 2 * ((BigInteger.One << 33) - expectedRaw);
        Assert.True(4 * squared > (twiceDistance - 1) * (twiceDistance - 1));
        Assert.True(4 * squared < (twiceDistance + 1) * (twiceDistance + 1));

        Assert.True(FixedSegment.TryGetCenteredFiniteCylindersContact(
            Vector3d.Zero, FixedQuaternion.Identity, Vector3d.Up, (Fixed64)4, Fixed64.One,
            new Vector3d(Fixed64.FromRaw(xRaw), Fixed64.Zero, Fixed64.FromRaw(zRaw)),
            FixedQuaternion.Identity, Vector3d.Up, (Fixed64)4, Fixed64.One,
            out FixedContactAnchors contact));
        Assert.Equal(expectedRaw, contact.Depth.m_rawValue);
        Assert.False(contact.DepthIsClamped);
    }

    [Theory]
    [InlineData(9, 8, 117, 153, 870_953_460L)]
    [InlineData(19, 16, 505, 649, 738_755_925L)]
    [InlineData(5, 4, 34, 43, 597_275_436L)]
    public void Contact_RoundsReducedCoreAxisProjectionExactly(
        int zNumerator, int denominator, int radialSquared, int axisSquared, long expectedRaw)
    {
        // The retained closest-core direction is (3d/4,3d/4,zNumerator).
        // Its selected projection is (2*sqrt(radialSquared)-axisSquared/d)
        // /sqrt(axisSquared), below the two 1/4-unit axial candidates.
        // These are penetrating controls, not a claim that this finite set of
        // contact axes completely classifies every cylinder pair (FMS-024).
        Assert.True(CompareSelectedProjectionToTwiceRaw(
            radialSquared, axisSquared, denominator, 2 * (BigInteger)expectedRaw - 1) > 0);
        Assert.True(CompareSelectedProjectionToTwiceRaw(
            radialSquared, axisSquared, denominator, 2 * (BigInteger)expectedRaw + 1) < 0);
        Vector3d secondCenter = new((Fixed64)7 / 4, (Fixed64)7 / 4, (Fixed64)zNumerator / denominator);
        Assert.True(Contact(secondCenter, false, out FixedContactAnchors forward));
        Assert.True(Contact(secondCenter, true, out FixedContactAnchors reverse));
        Assert.Equal(expectedRaw, forward.Depth.m_rawValue);
        Assert.Equal(expectedRaw, reverse.Depth.m_rawValue);
        Assert.False(forward.DepthIsClamped);
        Assert.False(reverse.DepthIsClamped);
        Assert.Equal(-forward.Normal, reverse.Normal);
    }

    [Theory]
    [InlineData(3L, 2L)]
    [InlineData(5L, 4L)]
    public void Contact_PreservesNearestEvenHalfRawDepth(long secondHeightRaw, long expectedRaw)
    {
        Assert.True(FixedSegment.TryGetCenteredFiniteCylindersContact(
            Vector3d.Zero, FixedQuaternion.Identity, Vector3d.Up, Fixed64.FromRaw(2), Fixed64.One,
            Vector3d.Zero, FixedQuaternion.Identity, Vector3d.Up, Fixed64.FromRaw(secondHeightRaw), Fixed64.One,
            out FixedContactAnchors contact));
        Assert.Equal(expectedRaw, contact.Depth.m_rawValue);
        Assert.False(contact.DepthIsClamped);
    }

    private static int CompareSelectedProjectionToTwiceRaw(
        BigInteger radialSquared, BigInteger axisSquared, BigInteger denominator, BigInteger twiceRaw)
    {
        BigInteger scale = BigInteger.One << 32;
        // 4*d*scale*sqrt(A) ? 2*scale*S + d*twiceRaw*sqrt(S).
        // Both sides are positive. Square once, check the remaining rational
        // sign, then square against the one positive radical. No production
        // wide-arithmetic or depth-comparison helper participates in this proof.
        BigInteger residual = 16 * denominator * denominator * scale * scale * radialSquared
            - 4 * scale * scale * axisSquared * axisSquared
            - denominator * denominator * twiceRaw * twiceRaw * axisSquared;
        if (residual.Sign < 0)
            return -1;
        return (residual * residual).CompareTo(
            16 * axisSquared * axisSquared * denominator * denominator * scale * scale * twiceRaw * twiceRaw * axisSquared);
    }

    private static bool Contact(Vector3d center, bool reverse, out FixedContactAnchors contact) => reverse
        ? FixedSegment.TryGetCenteredFiniteCylindersContact(
            center, FixedQuaternion.Identity, Vector3d.Right, Fixed64.Two, Fixed64.One,
            Vector3d.Zero, FixedQuaternion.Identity, Vector3d.Up, Fixed64.Two, Fixed64.One, out contact)
        : FixedSegment.TryGetCenteredFiniteCylindersContact(
            Vector3d.Zero, FixedQuaternion.Identity, Vector3d.Up, Fixed64.Two, Fixed64.One,
            center, FixedQuaternion.Identity, Vector3d.Right, Fixed64.Two, Fixed64.One, out contact);
}
