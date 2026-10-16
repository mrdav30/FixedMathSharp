using System;
using FixedMathSharp.Geometry;
using Xunit;

namespace FixedMathSharp.Tests.Bounds;

public sealed class WideTriangleRelationsEdgeCoverageTests
{
    [Theory]
    [InlineData(0)]
    [InlineData(1)]
    [InlineData(2)]
    public void EdgeCoverage_ShouldHandleEveryProjectionAxisAndClosedIncidence(int axis)
    {
        Vector3d P(int x, int y, int z = 0) => axis == 0 ? new(z, x, y) : axis == 1 ? new(y, z, x) : new(x, y, z);
        var source = new FixedTriangle(P(0, 0), P(4, 0), P(0, 2));
        Check(new(P(1, 0), P(3, 0), P(1, 1)), true, 1, 1, 4, 3, 4);
        Check(new(P(-1, -1), P(5, -1), P(2, 3)), true, 3, 0, 1, 1, 1);
        Check(new(P(4, 0), P(6, 0), P(4, -1)), true, 2, 1, 1, 1, 1);
        Check(new(P(-2, 0), P(6, 0), P(-2, -1)), true, 2, 0, 1, 1, 1);
        Check(new(P(2, -1), P(2, 1), P(5, 0)), true, 3, 1, 2, 1, 1);
        Check(new(P(0, -1), P(1, 1), P(3, 4)), true, 3, 1, 8, 3, 20);
        Check(new(P(4, -1), P(3, 1), P(1, 4)), true, 3, 17, 20, 7, 8);
        Check(new(P(1, 0), P(3, 0), P(1, 0, 1)), true, 4, 1, 4, 3, 4);
        Check(new(P(1, 1), P(3, 1), P(2, 2)), false);
        Check(new(P(5, 0), P(6, 0), P(5, -1)), false);
        Check(new(P(-3, 0), P(-1, 0), P(-3, -1)), false);
        Check(new(P(1, -1), P(2, -1), P(1, -2)), false);
        Check(new(P(0, 0), P(1, 0), P(2, 0)), false);

        void Check(FixedTriangle target, bool expected, int sides = 0, int ln = 0, int ld = 1, int un = 1, int ud = 1)
        {
            for (int reverse = 0; reverse < 2; reverse++)
            {
                if (reverse == 1) target = new FixedTriangle(target.B, target.A, target.C);
                bool found = WideTriangleRelations.TryGetEdgeCoverage(source, Vector3d.Zero,
                    FixedQuaternion.Identity, target, Vector3d.Zero, FixedQuaternion.Identity,
                    out RationalSegmentInterval interval, out int actualSides);
                Assert.Equal(expected, found);
                if (!found) continue;
                Assert.Equal(sides, actualSides);
                AssertBounds(interval, ln, ld, un, ud);
            }
        }
    }

    [Theory]
    [InlineData(0)]
    [InlineData(1)]
    [InlineData(2)]
    public void OffPlaneOrDegenerateSource_ShouldNotSupplyCoverage(int variant)
    {
        var target = new FixedTriangle(new(0, 0, 0), new(4, 0, 0), new(0, 2, 0));
        var source = variant == 0 ? new FixedTriangle(new(0, 0, 1), new(4, 0, 0), new(0, -2, 0))
            : variant == 1 ? new FixedTriangle(new(0, 0, 0), new(4, 0, 1), new(0, -2, 0))
            : new FixedTriangle(Vector3d.Zero, Vector3d.Zero, Vector3d.One);
        Assert.False(WideTriangleRelations.TryGetEdgeCoverage(source, Vector3d.Zero,
            FixedQuaternion.Identity, target, Vector3d.Zero, FixedQuaternion.Identity, out _, out _));
    }

    [Theory]
    [InlineData(false)]
    [InlineData(true)]
    public void IndependentObliqueFrames_ShouldKeepSubintervalAtWorldLimits(bool limits)
    {
        Vector3d origin = limits ? new(Fixed64.MaxValue - (Fixed64)16, Fixed64.MinValue + (Fixed64)16,
            Fixed64.MaxValue - (Fixed64)16) : Vector3d.Zero;
        FixedQuaternion rotation = new FixedQuaternion(Fixed64.Zero, Fixed64.Zero, (Fixed64)3, (Fixed64)4).Normalized;
        FixedQuaternion otherRotation = new(Fixed64.Zero, Fixed64.Zero, rotation.W, -rotation.Z);
        var source = new FixedTriangle(new(0, 0, 0), new(4, 0, 0), new(0, 2, 0));
        var target = new FixedTriangle(new(-1, 0, -1), new(-3, 0, -1), new(-1, 1, -1));
        Assert.True(WideTriangleRelations.TryGetEdgeCoverage(source, origin, rotation, target,
            origin + Vector3d.Forward, otherRotation, out RationalSegmentInterval interval, out int sides));
        Assert.Equal(2, sides);
        AssertBounds(interval, 1, 4, 3, 4);
        Assert.True(WideTriangleRelations.TryGetEdgeCoverage(new(source.B, source.A, source.C), origin,
            rotation, target, origin + Vector3d.Forward, otherRotation, out interval, out sides));
        Assert.Equal(2, sides);
        AssertBounds(interval, 1, 4, 3, 4);
    }

    [Fact]
    public void FullDomainSeparatedOrigins_ShouldNotOverflowIntoCoincidence()
    {
        var triangle = new FixedTriangle(new(0, 0, 0), new(4, 0, 0), new(0, 2, 0));
        Assert.False(WideTriangleRelations.TryGetEdgeCoverage(triangle,
            new(Fixed64.MinValue + (Fixed64)16, Fixed64.Zero, Fixed64.Zero), FixedQuaternion.Identity,
            triangle, new(Fixed64.MaxValue - (Fixed64)16, Fixed64.Zero, Fixed64.Zero),
            FixedQuaternion.Identity, out _, out _));
    }

    [Fact]
    public void ProducedInterval_ShouldReconstructCapsuleWitnessesWithoutRoundingItsEndpoints()
    {
        var plane = new FixedTriangle(new(0, 0, 0), new(4, 0, 0), new(0, 2, 0));
        var target = new FixedTriangle(new(0, -1, 0), new(1, 2, 0), new(4, -1, 0));
        Assert.True(WideTriangleRelations.TryGetEdgeCoverage(plane, Vector3d.Zero, FixedQuaternion.Identity,
            target, Vector3d.Zero, FixedQuaternion.Identity, out RationalSegmentInterval interval, out int sides));
        Assert.Equal(3, sides);
        AssertBounds(interval, 1, 12, 3, 4);
        var frame = new CapsulePlaneRayFrame(plane, new CapsuleFiniteSectionFrame(Vector3d.Zero,
            FixedQuaternion.Identity, Vector3d.Zero, FixedQuaternion.Identity, Fixed64.Zero, (Fixed64)4));
        var source = new CapsulePlaneRayEventSource(new FixedSegment(plane.A, plane.B));
        int endpoints = 0;
        Assert.True(CapsulePlaneRayEvents.VisitEvents(source, frame, 1, interval,
            (in CapsulePlaneRayEventSource edge, in CapsulePlaneRayFrame current,
                CapsulePlaneRayEvent descriptor, int orientation, scoped in CapsulePlaneRaySelection candidate) =>
            {
                Span<ulong> values = stackalloc ulong[CapsulePlaneRaySelection.StorageWords];
                Span<int> signs = stackalloc int[CapsulePlaneRaySelection.SignCount];
                Span<sbyte> valueSigns = stackalloc sbyte[CapsulePlaneRaySelection.ValueSignCount];
                var reconstructed = new CapsulePlaneRaySelection(values, signs, valueSigns);
                Assert.True(CapsulePlaneRayEvents.Reconstruct(edge, current, descriptor, orientation, interval, ref reconstructed));
                Assert.Equal(0, reconstructed.ComparePoint(candidate, current));
                Assert.Equal(0, reconstructed.CompareDepth(candidate));
                if (descriptor.Kind != CapsulePlaneRayEventKind.Endpoint) return;
                Assert.True(candidate.TryGetAuthoredPoint(current, Vector3d.Zero, FixedQuaternion.Identity,
                    Vector3d.Zero, FixedQuaternion.Identity, out Vector3d point));
                Assert.Equal(Fixed64.FromFraction(descriptor.Feature == 0 ? 1 : 9, 3), point.X);
                endpoints++;
            }));
        Assert.Equal(2, endpoints);
    }

    [Fact]
    public void RepeatedCoverage_ShouldAllocateNothing()
    {
        var source = new FixedTriangle(new(0, 0, 0), new(4, 0, 0), new(0, 2, 0));
        var target = new FixedTriangle(new(1, 0, 0), new(3, 0, 0), new(1, -1, 0));
        for (int i = 0; i < 32; i++) WideTriangleRelations.TryGetEdgeCoverage(source, Vector3d.Zero,
            FixedQuaternion.Identity, target, Vector3d.Zero, FixedQuaternion.Identity, out _, out _);
        long before = GC.GetAllocatedBytesForCurrentThread();
        int found = 0;
        for (int i = 0; i < 128; i++)
            if (WideTriangleRelations.TryGetEdgeCoverage(source, Vector3d.Zero, FixedQuaternion.Identity,
                target, Vector3d.Zero, FixedQuaternion.Identity, out _, out _)) found++;
        Assert.Equal(0, GC.GetAllocatedBytesForCurrentThread() - before);
        Assert.Equal(128, found);
    }

    [Fact]
    public void FullRawVerticesAndObliqueFrames_ShouldRetainOneRawEndpointDifferences()
    {
        Fixed64 min = Fixed64.MinValue, max = Fixed64.MaxValue, raw = Fixed64.FromRaw(1);
        var source = new FixedTriangle(new(min, Fixed64.Zero, Fixed64.Zero), new(max, Fixed64.Zero, Fixed64.Zero),
            new(min, max, Fixed64.Zero));
        var target = new FixedTriangle(new(min + raw, Fixed64.Zero, Fixed64.Zero), new(max - raw, Fixed64.Zero, Fixed64.Zero),
            new(min, min, Fixed64.Zero));
        var rotation = new FixedQuaternion((Fixed64)1, (Fixed64)2, (Fixed64)3, (Fixed64)4).Normalized;
        Assert.True(WideTriangleRelations.TryGetEdgeCoverage(source, new(min, max, min), rotation, target,
            new(min, max, min), rotation, out RationalSegmentInterval interval, out int sides));
        Assert.Equal(2, sides);
        Signed320 width = Signed320.ExtendValue(WideArithmetic.SubtractSigned192(Signed192.Raw(max), Signed192.Raw(min)));
        Signed320 rawUnit = Signed320.ExtendValue(Signed192.Signed(1));
        Assert.Equal(0, RationalSegmentInterval.CompareParameters(interval.LowerNumerator, interval.LowerDenominator,
            rawUnit, width));
        Assert.Equal(0, RationalSegmentInterval.CompareParameters(interval.UpperNumerator, interval.UpperDenominator,
            WideArithmetic.SubtractSigned320(width, rawUnit), width));
    }

    [Theory]
    [InlineData(false)]
    [InlineData(true)]
    public void CoplanarEdgeCoverage_ShouldRetainExactPartialIntervalAndFilledSide(bool reverse)
    {
        var source = new FixedTriangle(new(0, 0, 0), new(4, 0, 0), new(0, 2, 0));
        var other = reverse
            ? new FixedTriangle(new(3, 0, 0), new(1, 0, 0), new(1, -1, 0))
            : new FixedTriangle(new(1, 0, 0), new(3, 0, 0), new(1, -1, 0));
        Assert.True(WideTriangleRelations.TryGetEdgeCoverage(source, Vector3d.Zero,
            FixedQuaternion.Identity, other, Vector3d.Zero, FixedQuaternion.Identity,
            out RationalSegmentInterval interval, out int sides));
        Assert.Equal(2, sides);
        Assert.True(interval.IsClipped);
        Assert.Equal(0, Compare(interval.LowerNumerator, interval.LowerDenominator, 1, 4));
        Assert.Equal(0, Compare(interval.UpperNumerator, interval.UpperDenominator, 3, 4));
    }

    private static int Compare(Signed320 n, Signed320 d, long numerator, long denominator) =>
        WideArithmetic.SubtractSigned576(WideArithmetic.MultiplySigned576(Signed576.ExtendValue(n), denominator),
            WideArithmetic.MultiplySigned576(Signed576.ExtendValue(d), numerator)).Sign;

    private static void AssertBounds(in RationalSegmentInterval interval, int ln, int ld, int un, int ud)
    {
        if (!interval.IsClipped) { Assert.Equal(0, ln); Assert.Equal(un, ud); return; }
        Assert.Equal(0, Compare(interval.LowerNumerator, interval.LowerDenominator, ln, ld));
        Assert.Equal(0, Compare(interval.UpperNumerator, interval.UpperDenominator, un, ud));
    }
}
