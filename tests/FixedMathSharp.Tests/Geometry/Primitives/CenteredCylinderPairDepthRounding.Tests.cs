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
    [InlineData(9, 8, 1, 8)]
    [InlineData(19, 16, 3, 32)]
    [InlineData(5, 4, 1, 20)]
    public void Contact_RimMinimumRespectsAnIndependentSeparatingTranslationInBothOrders(
        int zNumerator, int denominator, int boundNumerator, int boundDenominator)
    {
        // Direction (1,1,1) gives support gap (2*sqrt(2)-3/2-z)/sqrt(3).
        // sqrt(2)<99/70 and sqrt(3)>5/3 certify the rational bound below.
        // The old closest-core projection exceeded this bound in every case;
        // its correctly rounded value was not the solids' minimum translation.
        Assert.True(3 * (93 * denominator - 70 * zNumerator) * boundDenominator
            < 350 * denominator * boundNumerator);
        long upperRaw = Fixed64.FromFraction(boundNumerator, boundDenominator).m_rawValue;
        Vector3d secondCenter = new((Fixed64)7 / 4, (Fixed64)7 / 4, (Fixed64)zNumerator / denominator);
        Assert.True(Contact(secondCenter, false, out FixedContactAnchors forward));
        Assert.True(Contact(secondCenter, true, out FixedContactAnchors reverse));
        Assert.InRange(forward.Depth.m_rawValue, 1, upperRaw - 1);
        Assert.Equal(forward.Depth, reverse.Depth);
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

    private static bool Contact(Vector3d center, bool reverse, out FixedContactAnchors contact) => reverse
        ? FixedSegment.TryGetCenteredFiniteCylindersContact(
            center, FixedQuaternion.Identity, Vector3d.Right, Fixed64.Two, Fixed64.One,
            Vector3d.Zero, FixedQuaternion.Identity, Vector3d.Up, Fixed64.Two, Fixed64.One, out contact)
        : FixedSegment.TryGetCenteredFiniteCylindersContact(
            Vector3d.Zero, FixedQuaternion.Identity, Vector3d.Up, Fixed64.Two, Fixed64.One,
            center, FixedQuaternion.Identity, Vector3d.Right, Fixed64.Two, Fixed64.One, out contact);
}
