using FixedMathSharp.Geometry;
using Xunit;

namespace FixedMathSharp.Tests.Bounds;

public sealed class WideEdgePointLocationTests
{
    [Theory]
    [InlineData(0, -1)]
    [InlineData(1, 0)]
    [InlineData(2, 1)]
    [InlineData(3, 2)]
    [InlineData(4, -1)]
    public void RationalPoint_ShouldRespectBothBoundedEndpoints(int numerator, int expected)
    {
        var source = new FixedSegment(Vector3d.Zero, new Vector3d(4, 0, 0));
        var target = new FixedSegment(new Vector3d(1, 0, 0), new Vector3d(3, 0, 0));
        Assert.Equal(expected, Locate(source, numerator, 4, target));
        int reversed = expected == 0 ? 2 : expected == 2 ? 0 : expected;
        Assert.Equal(reversed, Locate(source, numerator, 4, new FixedSegment(target.End, target.Start)));
    }

    [Theory]
    [InlineData(0)]
    [InlineData(1)]
    [InlineData(2)]
    public void Incidence_ShouldRejectOffLinePointsInEveryProjection(int axis)
    {
        Vector3d p = axis == 0 ? new(0, 1, 0) : axis == 1 ? new(0, 0, 1) : new(1, 0, 0);
        Vector3d q = axis == 0 ? new(1, 0, 0) : axis == 1 ? new(0, 1, 0) : new(0, 0, 1);
        Assert.Equal(-1, Locate(new FixedSegment(p, p), 0, 1, new FixedSegment(Vector3d.Zero, q)));
    }

    [Fact]
    public void SubRawPoint_ShouldRemainDistinctFromItsRoundedEndpoint()
    {
        var source = new FixedSegment(Vector3d.Zero, new Vector3d(Fixed64.FromRaw(1), Fixed64.Zero, Fixed64.Zero));
        Assert.Equal(1, Locate(source, 1, 3, source));
        Assert.Equal(-1, Locate(source, 1, 3, new FixedSegment(Vector3d.Zero, Vector3d.Zero)));
        Assert.Equal(0, Locate(source, 0, 1, new FixedSegment(Vector3d.Zero, Vector3d.Zero)));
    }

    [Fact]
    public void IndependentFrames_ShouldPreserveExactIncidenceNearWorldLimits()
    {
        var origin = new Vector3d(Fixed64.MaxValue - (Fixed64)16, Fixed64.MinValue + (Fixed64)16, Fixed64.Zero);
        var rotation = new FixedQuaternion(Fixed64.Zero, Fixed64.One, Fixed64.Zero, Fixed64.Zero);
        Assert.Equal(0, WideTriangleRelations.LocateEdgePoint(new FixedSegment(Vector3d.Zero, new Vector3d(4, 0, 0)),
            origin, FixedQuaternion.Identity, I(1), I(4), new FixedSegment(new Vector3d(-1, 0, 0), new Vector3d(-3, 0, 0)), origin, rotation));
    }

    private static Signed320 I(long value) => Signed320.ExtendValue(Signed192.Signed(value));

    [Fact]
    public void FullWidthBounds_ShouldRetainCancellationAcrossIndependentObliqueFrames()
    {
        Signed320 d = new(1UL << 62, 0, 0, 0, 0), half = new(1UL << 61, 0, 0, 0, 0);
        Signed320 lower = WideArithmetic.SubtractSigned320(half, I(1));
        Signed320 upper = WideArithmetic.AddSigned320(half, I(1));
        var clip = new RationalSegmentInterval(lower, d, upper, d);
        var edge = new FixedSegment(new Vector3d(-Fixed64.MaxValue, Fixed64.Zero, Fixed64.Zero),
            new Vector3d(Fixed64.MaxValue, Fixed64.Zero, Fixed64.Zero));
        FixedQuaternion source = FixedQuaternion.FromEulerAnglesInDegrees((Fixed64)17, (Fixed64)37, (Fixed64)11);
        FixedQuaternion target = FixedQuaternion.FromEulerAnglesInDegrees((Fixed64)13, (Fixed64)(-23), (Fixed64)31);
        Assert.Equal(1, WideTriangleRelations.LocateEdgePoint(edge, Vector3d.Zero, source, half, d,
            edge, Vector3d.Zero, target, clip));
        // An adjacent sub-raw point leaves the other framed line; rounding
        // either point to the common midpoint would incorrectly admit it.
        Assert.Equal(-1, WideTriangleRelations.LocateEdgePoint(edge, Vector3d.Zero, source, upper, d,
            edge, Vector3d.Zero, target, clip));
    }

    [Theory]
    [InlineData(0, -1)]
    [InlineData(1, 1)]
    [InlineData(2, 1)]
    [InlineData(3, 1)]
    [InlineData(4, -1)]
    public void BoundedIncidence_ShouldRespectClosedPartialSpans(int n, int expected)
    {
        var edge = new FixedSegment(Vector3d.Zero, new Vector3d(4, 0, 0));
        var clip = new RationalSegmentInterval(I(1), I(4), I(3), I(4));
        Assert.Equal(expected, WideTriangleRelations.LocateEdgePoint(edge, Vector3d.Zero, FixedQuaternion.Identity,
            I(n), I(4), edge, Vector3d.Zero, FixedQuaternion.Identity, clip));
        Assert.Equal(expected, WideTriangleRelations.LocateEdgePoint(edge, Vector3d.Zero, FixedQuaternion.Identity,
            I(n), I(4), new FixedSegment(edge.End, edge.Start), Vector3d.Zero, FixedQuaternion.Identity, clip));
    }
    private static int Locate(FixedSegment source, long n, long d, FixedSegment target) =>
        WideTriangleRelations.LocateEdgePoint(source, Vector3d.Zero, FixedQuaternion.Identity, I(n), I(d),
            target, Vector3d.Zero, FixedQuaternion.Identity);
}
