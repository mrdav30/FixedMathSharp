using FixedMathSharp.Geometry;
using Xunit;

namespace FixedMathSharp.Tests.Geometry.Wide;

public sealed class WideRigidProjectionTests
{
    [Theory]
    [InlineData(0)]
    [InlineData(1)]
    [InlineData(2)]
    public void FullWidthAxes_ShouldRetainEachNonzeroCrossComponent(int component)
    {
        Signed320 large = new(1UL << 44, 0, 0, 0, 0); // 2^300; each nonzero product is 2^600.
        var first = component == 0 ? new WideAxis3(default, large, default)
            : component == 1 ? new WideAxis3(default, default, large) : new WideAxis3(large, default, default);
        var second = component == 0 ? new WideAxis3(default, default, large)
            : component == 1 ? new WideAxis3(large, default, default) : new WideAxis3(default, large, default);
        Assert.False(WideAxis3.AreParallel(first, second));
        Assert.False(WideAxis3.AreParallel(second, first));
        Assert.False(WideAxis3.AreParallel(-first, second));
    }

    [Fact]
    public void FullWidthParallelAndZeroAxes_ShouldRemainExactlyParallel()
    {
        Signed320 large = new(1UL << 44, 0, 0, 0, 0), doubled = new(1UL << 45, 0, 0, 0, 0);
        var first = new WideAxis3(large, large, large);
        var second = new WideAxis3(doubled, doubled, doubled);
        Assert.True(WideAxis3.AreParallel(first, second));
        Assert.True(WideAxis3.AreParallel(second, first));
        Assert.True(WideAxis3.AreParallel(first, -second));
        Assert.True(WideAxis3.AreParallel(default, first));
        Assert.True(WideAxis3.AreParallel(first, default));
        Assert.True(WideAxis3.AreParallel(default, default));
    }

    [Fact]
    public void PreparedLocalAxis_MatchesRotatedPointProjection()
    {
        var basis = new WideRationalBasis3d(
            FixedQuaternion.FromEulerAnglesInDegrees(
                (Fixed64)11,
                (Fixed64)(-23),
                (Fixed64)37));
        var axis = new WideAxis3(
            Signed320.ExtendValue(Signed192.Raw((Fixed64)7)),
            Signed320.ExtendValue(Signed192.Raw((Fixed64)(-11))),
            Signed320.ExtendValue(Signed192.Raw((Fixed64)13)));
        var point = new Vector3d(3, -2, 5);

        WideRigidProjection.GetLocalAxisProjections(
            axis,
            basis,
            out Signed576 x,
            out Signed576 y,
            out Signed576 z);
        Signed576 projection = WideRigidProjection.GetLocalOffsetProjection(
            point,
            x,
            y,
            z);
        WideAxis3 transformedPoint = WideRigidProjection.TransformLocalAxis(
            basis,
            Signed192.Raw(point.X),
            Signed192.Raw(point.Y),
            Signed192.Raw(point.Z));

        Assert.Equal(WideAxis3.Dot(axis, transformedPoint), projection);
    }
}
