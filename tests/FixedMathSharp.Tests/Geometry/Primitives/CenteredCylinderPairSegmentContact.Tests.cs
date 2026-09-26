using FixedMathSharp.Geometry;
using Xunit;

namespace FixedMathSharp.Tests;

public sealed class CenteredCylinderPairSegmentContactTests
{
    private static FixedQuaternion SegmentRotation => new(
        Fixed64.FromRaw(5 * 607_400_100L), Fixed64.Zero,
        Fixed64.FromRaw(-3 * 607_400_100L), Fixed64.FromRaw(4 * 607_400_100L));

    [Theory]
    [InlineData(false)]
    [InlineData(true)]
    public void ZeroRadius_ObliqueSegmentUsesTheUniqueSideRimMinimum(bool fullDomainTranslation)
    {
        // The exact segment direction is (12,-9,20)/25. Projecting the
        // cylinder onto its normal plane gives ellipse semiaxes 625,225.
        // At this center the minor offset is zero and the major offset is
        // 80*sqrt(34); the nearest boundary distance is exactly
        // 225*sqrt(1-80^2*34/(625^2-225^2))=135.
        Vector3d origin = fullDomainTranslation
            ? new Vector3d(Fixed64.MaxValue - (Fixed64)600,
                Fixed64.MaxValue - (Fixed64)500, Fixed64.MinValue)
            : Vector3d.Zero;
        Vector3d center = origin + new Vector3d(544, 392, 0);
        Assert.True(Contact(origin, center, false, out FixedContactAnchors forward));
        Assert.True(Contact(origin, center, true, out FixedContactAnchors reverse));
        Assert.Equal((Fixed64)135, forward.Depth);
        Assert.Equal(forward.Depth, reverse.Depth);
        Assert.Equal(new Vector3d(Fixed64.FromFraction(3, 5),
            Fixed64.FromFraction(4, 5), Fixed64.Zero), forward.Normal);
        Assert.Equal(-forward.Normal, reverse.Normal);
        Assert.False(forward.DepthIsClamped);
        Assert.False(reverse.DepthIsClamped);
        Assert.Equal(origin, forward.FirstAnchor.Origin);
        Assert.Equal(center, forward.SecondAnchor.Origin);
        Assert.Equal(forward.FirstAnchor, reverse.SecondAnchor);
        Assert.Equal(forward.SecondAnchor, reverse.FirstAnchor);
    }

    [Theory]
    [InlineData(-1L, true, 1L)]
    [InlineData(0L, true, 0L)]
    [InlineData(1L, false, 0L)]
    public void ZeroRadius_ParallelSegmentDistinguishesSideBoundary(long offset, bool expected, long depthRaw)
    {
        Vector3d center = new(Fixed64.One + Fixed64.FromRaw(offset), Fixed64.Zero, Fixed64.Zero);
        bool hit = FixedSegment.TryGetCenteredFiniteCylindersContact(
            Vector3d.Zero, FixedQuaternion.Identity, Vector3d.Up, Fixed64.Two, Fixed64.One,
            center, FixedQuaternion.Identity, Vector3d.Up, Fixed64.Two, Fixed64.Zero,
            out FixedContactAnchors contact);
        Assert.Equal(expected, hit);
        if (expected)
        {
            Assert.Equal(Fixed64.FromRaw(depthRaw), contact.Depth);
            Assert.Equal(Vector3d.Right, contact.Normal);
            Assert.False(contact.DepthIsClamped);
        }
        else
        {
            Assert.Equal(default, contact);
        }
    }

    [Theory]
    [InlineData(0L, true)]
    [InlineData(1L, true)]
    [InlineData(2L, false)]
    public void BothZeroRadii_PreserveClosedContactOfHalfRawEndpoints(long centerRaw, bool expected)
    {
        // Endpoints are +/- half a raw unit, not rounded scalar positions.
        bool hit = FixedSegment.TryGetCenteredFiniteCylindersContact(
            Vector3d.Zero, FixedQuaternion.Identity, Vector3d.Up, Fixed64.MinIncrement, Fixed64.Zero,
            new Vector3d(Fixed64.Zero, Fixed64.FromRaw(centerRaw), Fixed64.Zero),
            FixedQuaternion.Identity, Vector3d.Up, Fixed64.MinIncrement, Fixed64.Zero,
            out FixedContactAnchors contact);
        Assert.Equal(expected, hit);
        if (expected)
        {
            Assert.Equal(Fixed64.Zero, contact.Depth);
            Assert.False(contact.DepthIsClamped);
        }
        else
        {
            Assert.Equal(default, contact);
        }
    }

    [Fact]
    public void ZeroRadius_ReversedSeparatedSegmentReturnsDefaultContact()
    {
        // Same exact normal as the inward fixture, but q=p+135*n.
        Assert.False(Contact(Vector3d.Zero, new Vector3d(706, 608, 0), true,
            out FixedContactAnchors contact));
        Assert.Equal(default, contact);
    }

    [Fact]
    public void ZeroRadius_OppositeLimitOriginsRejectWithoutSaturating()
    {
        Assert.False(Contact(new Vector3d(Fixed64.MinValue, Fixed64.Zero, Fixed64.Zero),
            new Vector3d(Fixed64.MaxValue, Fixed64.Zero, Fixed64.Zero), false,
            out FixedContactAnchors contact));
        Assert.Equal(default, contact);
    }

    [Fact]
    public void ZeroRadius_ObliqueContactDoesNotAllocate()
    {
        FixedContactAnchors contact = default;
        long allocated = FixedMathTestHelper.MeasureWarmedAllocations(() =>
        {
            _ = Contact(Vector3d.Zero, new Vector3d(544, 392, 0), false, out contact);
            _ = Contact(Vector3d.Zero, new Vector3d(544, 392, 0), true, out contact);
        });
        Assert.Equal(0, allocated);
        Assert.Equal((Fixed64)135, contact.Depth);
    }

    private static bool Contact(Vector3d cylinderCenter, Vector3d segmentCenter,
        bool reverse, out FixedContactAnchors contact) => reverse
        ? FixedSegment.TryGetCenteredFiniteCylindersContact(
            segmentCenter, SegmentRotation, Vector3d.Up, (Fixed64)2000, Fixed64.Zero,
            cylinderCenter, FixedQuaternion.Identity, Vector3d.Up, (Fixed64)1000, (Fixed64)625,
            out contact)
        : FixedSegment.TryGetCenteredFiniteCylindersContact(
            cylinderCenter, FixedQuaternion.Identity, Vector3d.Up, (Fixed64)1000, (Fixed64)625,
            segmentCenter, SegmentRotation, Vector3d.Up, (Fixed64)2000, Fixed64.Zero,
            out contact);
}
