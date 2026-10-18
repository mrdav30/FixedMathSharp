using FixedMathSharp.Geometry;
using System.Numerics;
using Xunit;

namespace FixedMathSharp.Tests;

public sealed class FixedTriangleClippedCapsuleTests
{
    [Theory]
    [InlineData(0, 1, -1)]
    [InlineData(2, 2, 0)]
    [InlineData(4, 3, 1)]
    public void ClippedEdge_ShouldSelectTheConstrainedClosestWitness(int center, int expected, int endpoint)
    {
        var triangle = new FixedTriangle(Vector3d.Zero, new Vector3d(4, 0, 0), new Vector3d(4, 0, 0));
        var interval = Interval(1, 4, 3, 4);
        Assert.True(triangle.TryGetClippedCenteredCapsuleContact(Vector3d.Zero, FixedQuaternion.Identity,
            new Vector3d((Fixed64)center, Fixed64.Quarter, Fixed64.Zero), FixedQuaternion.Identity, Fixed64.Zero,
            Fixed64.Two, Vector3d.Up, interval, out var contact, out _, out int selected));
        Assert.Equal(new Vector3d(expected, 0, 0), contact.FirstAnchor.LocalPoint);
        Assert.Equal(endpoint, selected);
    }

    [Theory]
    [InlineData(false)]
    [InlineData(true)]
    public void ParallelFamily_ShouldKeepAStableInteriorWitness(bool narrow)
    {
        BigInteger scale = BigInteger.One << (narrow ? 200 : 160);
        var intervals = new[]
        {
            Interval(3, 5, narrow ? 3 * scale + 5 : 4, narrow ? 5 * scale : 5),
            Interval(3 * scale + (narrow ? 0 : 5), 5 * scale,
                narrow ? 3 * scale + 5 : 4, narrow ? 5 * scale : 5)
        };
        foreach (var interval in intervals)
        foreach (bool reverse in new[] { false, true })
        {
            var triangle = new FixedTriangle(reverse ? Vector3d.Up : Vector3d.Zero,
                reverse ? Vector3d.Zero : Vector3d.Up, reverse ? Vector3d.Zero : Vector3d.Up);
            var clip = reverse ? Interval(ToBig(interval.UpperDenominator) - ToBig(interval.UpperNumerator),
                ToBig(interval.UpperDenominator), ToBig(interval.LowerDenominator) - ToBig(interval.LowerNumerator),
                ToBig(interval.LowerDenominator)) : interval;
            Assert.True(triangle.TryGetClippedCenteredCapsuleContact(Vector3d.Zero, FixedQuaternion.Identity,
                Vector3d.Zero, FixedQuaternion.Identity, Fixed64.Two, Fixed64.Half, Vector3d.Right,
                clip, out var contact, out var feature, out int endpoint));
            Assert.Equal(Fixed64.FromFraction(narrow ? 3 : 7, narrow ? 5 : 10), contact.FirstAnchor.LocalPoint.Y);
            Assert.Equal(contact.FirstAnchor.LocalPoint.Y, contact.SecondAnchor.LocalPoint.Y);
            Assert.Equal(Vector3d.Right, contact.Normal);
            Assert.Equal(Fixed64.Half, contact.Depth);
            Assert.True(feature.IsZeroSeparation);
            Assert.Equal(0, endpoint);
        }
    }

    [Fact]
    public void ClippedPoint_ShouldReprojectOntoTheFiniteCoreCapAndRejectMisses()
    {
        var triangle = new FixedTriangle(new Vector3d(0, 4, 0), new Vector3d(4, 4, 0), new Vector3d(4, 4, 0));
        var point = Interval(1, 2, 1, 2);
        Assert.True(triangle.TryGetClippedCenteredCapsuleContact(Vector3d.Zero, FixedQuaternion.Identity,
            Vector3d.Zero, FixedQuaternion.Identity, Fixed64.One, (Fixed64)8, Vector3d.Up, point,
            out var contact, out _, out _));
        Assert.Equal(new Vector3d(2, 4, 0), contact.FirstAnchor.LocalPoint);
        Assert.Equal(Fixed64.Half, contact.SecondAnchor.LocalPoint.Y);
        Assert.False(triangle.TryGetClippedCenteredCapsuleContact(Vector3d.Zero, FixedQuaternion.Identity,
            Vector3d.Zero, FixedQuaternion.Identity, Fixed64.Zero, Fixed64.Half, Vector3d.Up, point,
            out _, out _, out _));
    }

    private static RationalSegmentInterval Interval(BigInteger ln, BigInteger ld, BigInteger un, BigInteger ud) =>
        new(Signed(ln), Signed(ld), Signed(un), Signed(ud));

    [Theory]
    [InlineData(2, 6, 4)]
    [InlineData(5, 6, 11)]
    public void ParallelFamily_ShouldPreserveNativeMidpointOrSelectInteriorDyadicMidpoint(int low, int high, int expected)
    {
        var triangle = new FixedTriangle(Vector3d.Zero, Vector3d.Up, Vector3d.Up);
        BigInteger scale = BigInteger.One << 263;
        var interval = Interval(low * scale, 8 * scale, high * scale, 8 * scale);
        Assert.True(triangle.TryGetClippedCenteredCapsuleContact(Vector3d.Zero, FixedQuaternion.Identity,
            Vector3d.Zero, FixedQuaternion.Identity, Fixed64.Two, Fixed64.One, Vector3d.Right,
            interval, out var contact, out _, out int endpoint));
        Assert.Equal(Fixed64.FromFraction(expected, low == 2 ? 8 : 16), contact.FirstAnchor.LocalPoint.Y);
        Assert.Equal(0, endpoint);
    }

    [Theory]
    [InlineData(false, false)]
    [InlineData(false, true)]
    [InlineData(true, false)]
    [InlineData(true, true)]
    public void FiniteSection_ShouldIntersectOnlyTheRetainedSubsegment(bool capsule, bool vertical)
    {
        var frame = new CapsuleFiniteSectionFrame(Vector3d.Zero, FixedQuaternion.Identity, Vector3d.Zero,
            FixedQuaternion.Identity, capsule ? Fixed64.Two : Fixed64.Zero, Fixed64.One);
        Vector3d axis = vertical ? Vector3d.Up : Vector3d.Right;
        var segment = new FixedSegment(-axis * (Fixed64)4, axis * (Fixed64)4);
        Assert.True(frame.IntersectsSegment(segment, default));
        for (int lo = 0; lo < 8; lo++)
        for (int hi = lo; hi <= 8; hi++)
        {
            var interval = Interval(lo, 8, hi, 8);
            bool expected = lo <= (capsule && vertical ? 6 : 5) && hi >= (capsule && vertical ? 2 : 3);
            Assert.Equal(expected, frame.IntersectsSegment(segment, interval));
        }
    }

    private static Signed320 Signed(BigInteger n)
    {
        BigInteger mask = ulong.MaxValue;
        return new((ulong)((n >> 256) & mask), (ulong)((n >> 192) & mask), (ulong)((n >> 128) & mask),
            (ulong)((n >> 64) & mask), (ulong)(n & mask));
    }

    private static BigInteger ToBig(Signed320 n) => ((BigInteger)n.Word4 << 256) | ((BigInteger)n.Word3 << 192)
        | ((BigInteger)n.Word2 << 128) | ((BigInteger)n.Word1 << 64) | n.Word0;
}
