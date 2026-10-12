using System;
using System.Collections.Generic;
using FixedMathSharp.Geometry;
using Xunit;

namespace FixedMathSharp.Tests.Geometry;

public sealed class FiniteSlabRaySupportTests
{
    private static FixedTriangle Plane(int kind) => kind switch
    {
        0 => new(new Vector3d(-5, 0, -5), new Vector3d(5, 0, -5), new Vector3d(0, 0, 5)),
        1 => new(new Vector3d(0, -5, -5), new Vector3d(0, 5, -5), new Vector3d(0, 0, 5)),
        2 => new(new Vector3d(-5, -5, 0), new Vector3d(5, -5, 0), new Vector3d(0, 5, 0)),
        3 => new(new Vector3d(-5, 5, -5), new Vector3d(5, -5, -5), new Vector3d(0, 0, 5)),
        _ => new(new Vector3d(-5, 5, -5), new Vector3d(5, 5, -5), new Vector3d(0, -5, 5))
    };

    public static IEnumerable<object[]> IntrinsicSupportCases()
    {
        for (int plane = 1; plane <= 6; plane++)
            for (int shape = 0; shape < 5; shape++)
                for (int orientation = -1; orientation <= 1; orientation += 2)
                    yield return new object[] { plane, shape, orientation };
    }

    [Theory]
    [MemberData(nameof(IntrinsicSupportCases))]
    public void IntrinsicPlaneSupportMatchesTheContainingFiniteDomainAndRetainsPhysicalPoles(
        int planeKind, int shape, int orientation)
    {
        FixedTriangle small = planeKind == 6
            ? new(new Vector3d(-10, -5, 5), new Vector3d(10, -5, -5), new Vector3d(0, 5, 0))
            : planeKind == 5
            ? new(new Vector3d(-5, -5, 5), new Vector3d(5, -5, -5), new Vector3d(0, 5, 0))
            : Plane(planeKind);
        FixedTriangle plane = new(small.A * (Fixed64)100, small.B * (Fixed64)100, small.C * (Fixed64)100);
        // Shape four has duplicate and collinear authored vertices: its true
        // support corners cannot rely on independent adjacent wall equations.
        Vector2d[] offsets = shape == 2 ? new Vector2d[] { new(-1, -1), new(1, 0), new(-1, 1) }
            : shape == 4 ? new Vector2d[] { new(-1, -1), new(-1, -1), new(-1, 0), new(-1, 1), new(1, 0) }
            : Array.Empty<Vector2d>();
        FiniteSlabRayFrame frame = new(plane, Vector3d.Zero, FixedQuaternion.Identity,
            Vector3d.Zero, FixedQuaternion.Identity, shape == 1 || shape == 3 ? Fixed64.Two : Fixed64.Zero,
            shape == 3 ? Fixed64.Zero : Fixed64.One, Fixed64.One);
        if (planeKind == 1 || planeKind == 3) Assert.Equal(1, frame.Plane.Normal.X.Sign);
        Span<ulong> values = stackalloc ulong[2 * FiniteSlabRaySelection.StorageWords];
        Span<sbyte> signs = stackalloc sbyte[2 * FiniteSlabRaySelection.SignCount];
        FiniteSlabRaySelection intrinsic = new(values, signs);
        FiniteSlabRaySelection finite = new(values[FiniteSlabRaySelection.StorageWords..], signs[FiniteSlabRaySelection.SignCount..]);
        FiniteSlabRayExtrema.AccumulatePlane(frame, offsets, orientation, ref intrinsic);
        FiniteSlabRayExtrema.AccumulateTriangle(frame, plane, offsets, orientation, ref finite);
        Assert.Equal(0, intrinsic.CompareDepth(finite));

        ulong[] retainedValues = values[..FiniteSlabRaySelection.StorageWords].ToArray();
        sbyte[] retainedSigns = signs[..FiniteSlabRaySelection.SignCount].ToArray();
        FiniteSlabRaySelectionMetadata metadata = intrinsic.Metadata;
        HashSet<Vector3d> poles = new();
        void Visit(scoped in FiniteSlabRaySelection candidate)
        {
            FiniteSlabRaySelection peak = new(retainedValues, retainedSigns); peak.Restore(metadata);
            if (candidate.CompareDepth(peak) != 0) return;
            Assert.True(candidate.TryGetContactAnchor(frame, out _, out bool clamped, out _, out FixedPointAnchor exitAnchor));
            Assert.True(exitAnchor.TryGetPoint(out Vector3d exit));
            Assert.False(clamped);
            Assert.Equal(Fixed64.One, exit.Y.Abs());
            if (shape == 0)
            {
                Fixed64 squaredRadius = exit.X * exit.X + exit.Z * exit.Z;
                Assert.True(squaredRadius >= Fixed64.One - Fixed64.FromRaw(4)
                    && squaredRadius <= Fixed64.One + Fixed64.FromRaw(4));
            }
            else if (shape == 1)
            {
                Fixed64 radialZ = FixedMath.Max(Fixed64.Zero, exit.Z.Abs() - Fixed64.One);
                Fixed64 squaredRadius = exit.X * exit.X + radialZ * radialZ;
                Assert.True(squaredRadius >= Fixed64.One - Fixed64.FromRaw(4)
                    && squaredRadius <= Fixed64.One + Fixed64.FromRaw(4));
            }
            else if (shape == 3)
            {
                Assert.Equal(Fixed64.Zero, exit.X);
                Assert.Equal(Fixed64.One, exit.Z.Abs());
            }
            else Assert.Contains(new Vector2d(exit.X, exit.Z), offsets);
            poles.Add(exit);
        }
        intrinsic = new(values, signs);
        FiniteSlabRayExtrema.AccumulatePlane(frame, offsets, orientation, ref intrinsic, Visit);
        int height = planeKind == 1 || planeKind == 2 || planeKind == 5 || planeKind == 6 ? 2 : 1;
        int footprint = shape == 1 || shape == 3
            ? planeKind == 1 || planeKind == 3 ? 2 : 1
            : shape == 2 || shape == 4
                ? (planeKind == 1 || planeKind == 3) && orientation < 0
                    || planeKind == 6 && orientation > 0 ? 2 : 1 : 1;
        Assert.Equal((height == 2 ? 1 : 0) + (footprint == 2 ? 1 : 0),
            frame.GetSupportDimension(offsets, orientation));
        Assert.Equal(height * footprint, poles.Count);
        if (offsets.Length != 0)
        {
            Array.Reverse(offsets); poles.Clear(); intrinsic = new(values, signs);
            FiniteSlabRayExtrema.AccumulatePlane(frame, offsets, orientation, ref intrinsic, Visit);
            Assert.Equal(height * footprint, poles.Count);
            FiniteSlabRaySelection peak = new(retainedValues, retainedSigns); peak.Restore(metadata);
            Assert.Equal(0, intrinsic.CompareDepth(peak));
        }
    }

    [Theory]
    [InlineData(0, 0, 1, 2)]
    [InlineData(1, 0, 1, 1)]
    [InlineData(3, 0, 1, 0)]
    [InlineData(1, 2, 1, 2)]
    [InlineData(2, 2, 1, 1)]
    [InlineData(0, 2, 1, 2)]
    [InlineData(3, 2, 1, 1)]
    [InlineData(4, 2, 1, 0)]
    [InlineData(1, 0, 0, 1)]
    [InlineData(0, 0, 0, 0)]
    [InlineData(0, 2, 0, 1)]
    [InlineData(1, 2, 0, 2)]
    [InlineData(2, 2, 0, 1)]
    public void SupportDimensionCountsOnlyDirectionsFreeUnderTheExactPlaneFunctional(
        int plane, int length, int radius, int dimension)
    {
        FiniteSlabRayFrame frame = new(Plane(plane), Vector3d.Zero, FixedQuaternion.Identity,
            Vector3d.Zero, FixedQuaternion.Identity, (Fixed64)length, (Fixed64)radius, Fixed64.One);
        Assert.Equal(dimension, frame.GetSupportDimension(ReadOnlySpan<Vector2d>.Empty, 1));
        Assert.Equal(dimension, frame.GetSupportDimension(ReadOnlySpan<Vector2d>.Empty, -1));
    }

    [Fact]
    public void SupportDimensionRejectsAnInvalidOrientationAndKeepsAnEmptyFrameCollapsed()
    {
        FiniteSlabRayFrame frame = default;
        Assert.Equal(0, frame.GetSupportDimension(ReadOnlySpan<Vector2d>.Empty, 1));
        Assert.Throws<ArgumentOutOfRangeException>(() => frame.GetSupportDimension(Array.Empty<Vector2d>(), 0));
    }

    [Theory]
    [InlineData(1, 1, 1)]
    [InlineData(1, -1, 2)]
    [InlineData(3, 1, 0)]
    [InlineData(3, -1, 1)]
    [InlineData(0, 1, 2)]
    public void PrismSupportDimensionUsesOrientedExactVertexTies(int plane, int orientation, int dimension)
    {
        FiniteSlabRayFrame frame = new(Plane(plane), Vector3d.Zero, FixedQuaternion.Identity,
            Vector3d.Zero, FixedQuaternion.Identity, Fixed64.Zero, Fixed64.Zero, Fixed64.One);
        Span<Vector2d> offsets = stackalloc Vector2d[3] { new(-1, -1), new(1, 0), new(-1, 1) };
        Assert.Equal(dimension, frame.GetSupportDimension(offsets, orientation));
        Span<Vector2d> reversed = stackalloc Vector2d[3] { offsets[2], offsets[1], offsets[0] };
        Assert.Equal(dimension, frame.GetSupportDimension(reversed, orientation));
    }

    [Theory]
    [InlineData(-1, false)]
    [InlineData(1, false)]
    [InlineData(-1, true)]
    [InlineData(1, true)]
    public void DegenerateTriangleRetainsBothClosedBoundarySupportEndpointsAndTangency(int orientation, bool tangent)
    {
        FixedTriangle plane = Plane(0);
        FiniteSlabRayFrame frame = new(plane, Vector3d.Zero, FixedQuaternion.Identity,
            Vector3d.Zero, FixedQuaternion.Identity, Fixed64.Zero, Fixed64.One, Fixed64.One);
        Span<ulong> peakValues = stackalloc ulong[FiniteSlabRaySelection.StorageWords];
        Span<sbyte> peakSigns = stackalloc sbyte[FiniteSlabRaySelection.SignCount];
        FiniteSlabRaySelection peak = new(peakValues, peakSigns);
        FiniteSlabRayExtrema.AccumulatePlane(frame, ReadOnlySpan<Vector2d>.Empty, orientation, ref peak);
        ulong[] retainedValues = peakValues.ToArray(); sbyte[] retainedSigns = peakSigns.ToArray();
        FiniteSlabRaySelectionMetadata metadata = peak.Metadata;
        HashSet<Vector3d> points = new();
        void Visit(scoped in FiniteSlabRaySelection candidate)
        {
            FiniteSlabRaySelection planePeak = new(retainedValues, retainedSigns); planePeak.Restore(metadata);
            if (candidate.CompareDepth(planePeak) != 0) return;
            Assert.True(candidate.TryGetContactAnchor(frame, out _, out _, out Vector3d point, out _));
            points.Add(point);
        }
        Vector3d a = new(-Fixed64.Two, Fixed64.Zero, tangent ? Fixed64.One : Fixed64.Zero);
        Vector3d b = new(Fixed64.Two, Fixed64.Zero, tangent ? Fixed64.One : Fixed64.Zero);
        Span<ulong> values = stackalloc ulong[FiniteSlabRaySelection.StorageWords];
        Span<sbyte> signs = stackalloc sbyte[FiniteSlabRaySelection.SignCount];
        FiniteSlabRaySelection maximum = new(values, signs);
        FiniteSlabRayExtrema.AccumulateTriangle(frame, new(a, b, b), ReadOnlySpan<Vector2d>.Empty, orientation, ref maximum, Visit);
        Assert.Equal(0, maximum.CompareDepth(peak));
        if (tangent) Assert.Equal(new Vector3d(0, 0, 1), Assert.Single(points));
        else
        {
            Assert.Equal(2, points.Count);
            Assert.Contains(new Vector3d(-1, 0, 0), points);
            Assert.Contains(new Vector3d(1, 0, 0), points);
        }
        points.Clear(); maximum = new(values, signs);
        FiniteSlabRayExtrema.AccumulateTriangle(frame, new(b, a, a), ReadOnlySpan<Vector2d>.Empty, orientation, ref maximum, Visit);
        Assert.Equal(tangent ? 1 : 2, points.Count);
        Assert.Equal(0, maximum.CompareDepth(peak));
    }

    [Theory]
    [InlineData(-1)]
    [InlineData(1)]
    public void PointDomainUsesTheCanonicalPlaneMetricAndMatchesItsPeak(int orientation)
    {
        FixedTriangle plane = Plane(0);
        FiniteSlabRayFrame frame = new(plane, Vector3d.Zero, FixedQuaternion.Identity,
            Vector3d.Zero, FixedQuaternion.Identity, Fixed64.Zero, Fixed64.One, Fixed64.One);
        Span<ulong> values = stackalloc ulong[2 * FiniteSlabRaySelection.StorageWords];
        Span<sbyte> signs = stackalloc sbyte[2 * FiniteSlabRaySelection.SignCount];
        FiniteSlabRaySelection peak = new(values, signs);
        FiniteSlabRaySelection point = new(values[FiniteSlabRaySelection.StorageWords..], signs[FiniteSlabRaySelection.SignCount..]);
        FiniteSlabRayExtrema.AccumulatePlane(frame, ReadOnlySpan<Vector2d>.Empty, orientation, ref peak);
        FiniteSlabRayExtrema.AccumulateTriangle(frame, new(Vector3d.Zero, Vector3d.Zero, Vector3d.Zero),
            ReadOnlySpan<Vector2d>.Empty, orientation, ref point);
        Assert.Equal(0, point.CompareDepth(peak));
        Assert.True(point.TryGetContactAnchor(frame, out Fixed64 depth, out _, out Vector3d source, out FixedPointAnchor exitAnchor));
        Assert.True(exitAnchor.TryGetPoint(out Vector3d exit));
        Assert.Equal(Fixed64.One, depth);
        Assert.Equal(Vector3d.Zero, source);
        Assert.Equal(new Vector3d(0, orientation, 0), exit);
    }
}
