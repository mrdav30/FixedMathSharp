using System;
using FixedMathSharp.Geometry;
using Xunit;

namespace FixedMathSharp.Tests.Bounds;

public sealed class CapsulePlaneRayJoinedAdmissionTests
{
    [Fact]
    public void EmptySelection_ShouldDeclineCrossFrameAdmission()
    {
        Span<ulong> values = stackalloc ulong[CapsulePlaneRaySelection.StorageWords];
        Span<int> signs = stackalloc int[CapsulePlaneRaySelection.SignCount];
        Span<sbyte> valueSigns = stackalloc sbyte[CapsulePlaneRaySelection.ValueSignCount];
        var selection = new CapsulePlaneRaySelection(values, signs, valueSigns);
        Assert.False(selection.ContainsTrianglePoint(default, default, default));
    }

    [Theory]
    [InlineData(false)]
    [InlineData(true)]
    public void Admission_ShouldCompareTheSamePhysicalPointAcrossNativeCoordinateScales(bool different)
    {
        var triangle = new FixedTriangle(new(-8, 0, -8), new(8, 0, -8), new(0, 0, 8));
        var source = new CapsulePlaneRayFrame(triangle, new CapsuleFiniteSectionFrame(Vector3d.Zero,
            FixedQuaternion.Identity, Vector3d.Zero, FixedQuaternion.Identity, Fixed64.One, Fixed64.One));
        FixedQuaternion pose = different
            ? FixedQuaternion.FromEulerAnglesInDegrees(Fixed64.Zero, (Fixed64)37, Fixed64.Zero)
            : FixedQuaternion.Identity;
        var target = new CapsulePlaneRayFrame(triangle, new CapsuleFiniteSectionFrame(Vector3d.Zero,
            pose, Vector3d.Zero, FixedQuaternion.Identity, Fixed64.One, Fixed64.One));
        var outside = new FixedTriangle(new(100, 0, 0), new(101, 0, 0), new(100, 0, 1));
        int count = 0;
        CapsulePlaneRayEvents.VisitEvents(CapsulePlaneRayEventSource.Plane, source, 1,
            (in CapsulePlaneRayEventSource _, in CapsulePlaneRayFrame current,
                CapsulePlaneRayEvent descriptor, int orientation, scoped in CapsulePlaneRaySelection point) =>
            {
                Assert.True(point.ContainsTrianglePoint(target, triangle, current));
                Assert.False(point.ContainsTrianglePoint(target, outside, current));
                count++;
            });
        Assert.True(count > 0);
    }

    [Fact]
    public void CommonChartMetrics_ShouldIgnoreNormalDisplacement()
    {
        var normal = new WideAxis3(default, Signed320.ExtendValue(Signed192.Signed(1)), default);
        var metrics = new WidePlaneMetrics(normal);
        Signed832 actual = metrics.GetSpanSquared(Vector3d.Zero, new Vector3d(3, 100, 4));
        Signed320 expected = WideArithmetic.MultiplySigned192(Signed192.Raw((Fixed64)5), Signed192.Raw((Fixed64)5));
        Assert.True(WideArithmetic.SubtractSigned832(actual, Signed832.ExtendValue(Signed576.ExtendValue(expected))).IsZero);
        Assert.Throws<ArgumentException>(() => new WidePlaneMetrics(default(WideAxis3)));
    }
}
