using FixedMathSharp.Geometry;
using Xunit;

namespace FixedMathSharp.Tests;

public sealed class FiniteSlabClippedSectionTests
{
    [Theory]
    [InlineData(0, false)] [InlineData(0, true)]
    [InlineData(2, false)] [InlineData(2, true)]
    public void RetainedSegment_ShouldKeepClosedCurvedAdmissionWithoutRoundingItsCut(int length, bool affine)
    {
        var source = affine
            ? new ScaledRigidFrame3d(Vector3d.Zero, FixedQuaternion.Identity, Vector3d.Zero,
                Vector3d.One * Fixed64.Half, FixedQuaternion.Identity, Vector3d.Zero, Vector3d.One)
            : ScaledRigidFrame3d.Rigid(Vector3d.Zero, FixedQuaternion.Identity);
        var frame = new FiniteSlabSectionFrame(source, Vector3d.Zero, FixedQuaternion.Identity,
            (Fixed64)length, Fixed64.One, Fixed64.One);
        FixedSegment crossing = Segment(Fixed64.Zero, Fixed64.Zero);
        Assert.True(frame.IntersectsSegment(crossing, default(RationalSegmentInterval)));
        Assert.False(frame.IntersectsSegment(crossing, Clip(0, 1, 8)));
        Assert.True(frame.IntersectsSegment(crossing, Clip(0, 2, 8)));
        Assert.True(frame.IntersectsSegment(crossing, Clip(6, 8, 8)));
        Assert.False(frame.IntersectsSegment(crossing, Clip(7, 8, 8)));
        // This endpoint lies just outside X=-1 by less than one Q32 unit.
        // Materializing the rational cut first would turn the miss into a hit.
        long denominator = 1L << 42;
        Assert.False(frame.IntersectsSegment(crossing, Clip(0, (denominator >> 2) - 1, denominator)));
        Assert.True(frame.IntersectsSegment(crossing, Clip(0, denominator >> 2, denominator)));
        foreach (int side in new[] { -1, 1 })
        {
            FixedSegment cap = Segment(Fixed64.Zero, (Fixed64)(side * 2));
            Assert.Equal(length != 0, frame.IntersectsSegment(cap, Clip(1, 1, 2)));
            Assert.False(frame.IntersectsSegment(cap, Clip(0, 1, 4)));
        }
        Assert.False(frame.IntersectsSegment(Segment(Fixed64.One + Fixed64.MinIncrement, Fixed64.Zero), Clip(0, 1, 1)));

        FixedSegment Segment(Fixed64 y, Fixed64 z)
        {
            Fixed64 scale = affine ? Fixed64.Two : Fixed64.One;
            return new(new Vector3d(-(Fixed64)2, y, z) * scale, new Vector3d((Fixed64)2, y, z) * scale);
        }
    }

    private static RationalSegmentInterval Clip(long lower, long upper, long denominator) => new(
        Signed320.ExtendValue(Signed192.Signed(lower)), Signed320.ExtendValue(Signed192.Signed(denominator)),
        Signed320.ExtendValue(Signed192.Signed(upper)), Signed320.ExtendValue(Signed192.Signed(denominator)));
}
