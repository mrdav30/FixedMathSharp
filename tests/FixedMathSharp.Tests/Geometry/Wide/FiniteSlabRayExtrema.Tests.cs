//=======================================================================
// MIT License, Copyright (c) 2024-present David Oravsky (mrdav30)
// See LICENSE file in the project root for full license information.
//=======================================================================
using System;
using System.Collections.Generic;
using Xunit;
using FixedMathSharp.Geometry;

namespace FixedMathSharp.Tests.Geometry;

public class FiniteSlabRayExtremaTests
{
    [Fact]
    public void TranslatedStadiumExitRetainsAnEndpointBeyondTheLocalScalarRange()
    {
        Fixed64 length = Fixed64.MaxValue, radius = Fixed64.MaxValue - Fixed64.One;
        FixedTriangle plane = new(new Vector3d(Fixed64.MinValue, Fixed64.Zero, Fixed64.MinValue),
            new Vector3d(Fixed64.MaxValue, Fixed64.Zero, Fixed64.MinValue),
            new Vector3d(Fixed64.Zero, Fixed64.Zero, Fixed64.MaxValue));
        Vector3d center = new(Fixed64.Zero, Fixed64.Zero, -(length / Fixed64.Two));
        FiniteSlabRayFrame frame = new(plane, Vector3d.Zero, FixedQuaternion.Identity,
            center, FixedQuaternion.Identity, length, radius, Fixed64.One);
        Span<ulong> values = stackalloc ulong[FiniteSlabRaySelection.StorageWords];
        Span<sbyte> signs = stackalloc sbyte[FiniteSlabRaySelection.SignCount];
        FiniteSlabRaySelection maximum = new(values, signs);
        bool retained = false;
        void Visit(scoped in FiniteSlabRaySelection candidate)
        {
            if (!candidate.TryGetContactAnchor(frame, out Fixed64 depth, out bool clamped,
                out Vector3d source, out FixedPointAnchor exit)
                || source.X != Fixed64.Zero || source.Z <= radius - Fixed64.One) return;
            Assert.Equal(Fixed64.One, depth);
            Assert.False(clamped);
            Assert.True(exit.TryGetPoint(out Vector3d world));
            Assert.Equal(source.Z, world.Z);
            Assert.Equal(Fixed64.One, world.Y);
            Assert.True(exit.LocalPoint.Z > Fixed64.Zero && exit.LocalDisplacement.Z > Fixed64.Zero);
            Assert.True(exit.LocalPoint.Z > Fixed64.MaxValue - exit.LocalDisplacement.Z);
            retained = true;
        }
        FiniteSlabRayExtrema.AccumulateTriangle(frame, plane, ReadOnlySpan<Vector2d>.Empty, 1, ref maximum, Visit);
        Assert.True(retained);
    }

    [Fact]
    public void OddHalfCoreNonintegralRadialExitRoundsTheTotalOnce()
    {
        Fixed64 oneRaw = Fixed64.FromRaw(1), twoRaw = Fixed64.FromRaw(2), threeRaw = Fixed64.FromRaw(3);
        FixedTriangle triangle = new(new Vector3d(-oneRaw, Fixed64.Zero, twoRaw),
            new Vector3d(oneRaw, Fixed64.Zero, twoRaw), new Vector3d(Fixed64.Zero, Fixed64.Zero, threeRaw));
        FiniteSlabRayFrame frame = new(triangle, Vector3d.Zero, FixedQuaternion.Identity,
            Vector3d.Zero, FixedQuaternion.Identity, threeRaw, twoRaw, Fixed64.One);
        Span<ulong> values = stackalloc ulong[FiniteSlabRaySelection.StorageWords];
        Span<sbyte> signs = stackalloc sbyte[FiniteSlabRaySelection.SignCount];
        FiniteSlabRaySelection maximum = new(values, signs);
        bool retained = false;
        void Visit(scoped in FiniteSlabRaySelection candidate)
        {
            if (!candidate.TryGetContactAnchor(frame, out Fixed64 depth, out bool clamped,
                out Vector3d source, out FixedPointAnchor exit) || source.Z != threeRaw) return;
            Assert.True(exit.TryGetPoint(out Vector3d world));
            Assert.Equal(source.Z, world.Z);
            Assert.Equal(Fixed64.One, world.Y);
            Assert.Equal(Fixed64.One, depth);
            Assert.False(clamped);
            retained = true;
        }
        FiniteSlabRayExtrema.AccumulateTriangle(frame, triangle, ReadOnlySpan<Vector2d>.Empty, 1, ref maximum, Visit);
        Assert.True(retained);
    }

    [Fact]
    public void StadiumCentralCapRayUsesItsFiniteVerticalDepth()
    {
        FixedTriangle triangle = new(new Vector3d((Fixed64)(-.5), (Fixed64).5, Fixed64.Two),
            new Vector3d((Fixed64)(-.5), (Fixed64).5, (Fixed64)3),
            new Vector3d((Fixed64).5, (Fixed64).5, Fixed64.Two));
        FiniteSlabRayFrame frame = new(triangle, Vector3d.Zero, FixedQuaternion.Identity,
            Vector3d.Zero, FixedQuaternion.Identity, (Fixed64)10, Fixed64.One, Fixed64.One);
        Span<ulong> values = stackalloc ulong[FiniteSlabRaySelection.StorageWords];
        Span<sbyte> signs = stackalloc sbyte[FiniteSlabRaySelection.SignCount];
        FiniteSlabRaySelection maximum = new(values, signs);
        FiniteSlabRayExtrema.AccumulateTriangle(frame, triangle, ReadOnlySpan<Vector2d>.Empty, 1, ref maximum);
        Assert.True(maximum.TryGetContactAnchor(frame, out Fixed64 depth, out bool clamped,
            out _, out FixedPointAnchor exit));
        Assert.Equal((Fixed64).5, depth);
        Assert.False(clamped);
        Assert.True(exit.TryGetPoint(out Vector3d point));
        Assert.Equal(Fixed64.One, point.Y);
        Assert.Equal(Vector3d.Down, frame.GetNormal(1));
    }

    [Theory]
    [InlineData(-1, false)]
    [InlineData(1, false)]
    [InlineData(-1, true)]
    [InlineData(1, true)]
    public void TallCylinderClearanceIncludesSourcePointsOutsideItsInitialSection(int orientation, bool unboundedPlane)
    {
        // The owner is x+y=0. Clearing the complete cylinder requires its
        // support height (100+1)/sqrt(2), even though the corresponding
        // projected owner point lies outside the original radius-one section.
        FixedTriangle plane = new(new Vector3d(-200, 200, -200),
            new Vector3d(200, -200, -200), new Vector3d(0, 0, 200));
        FiniteSlabRayFrame frame = new(plane, Vector3d.Zero, FixedQuaternion.Identity,
            Vector3d.Zero, FixedQuaternion.Identity, Fixed64.Zero, Fixed64.One, (Fixed64)100);
        Span<ulong> values = stackalloc ulong[FiniteSlabRaySelection.StorageWords];
        Span<sbyte> signs = stackalloc sbyte[FiniteSlabRaySelection.SignCount];
        FiniteSlabRaySelection maximum = new(values, signs);
        if (unboundedPlane)
            FiniteSlabRayExtrema.AccumulatePlane(frame, ReadOnlySpan<Vector2d>.Empty, orientation, ref maximum);
        else
            FiniteSlabRayExtrema.AccumulateTriangle(frame, plane, ReadOnlySpan<Vector2d>.Empty, orientation, ref maximum);
        Assert.True(maximum.TryGetContactAnchor(frame, out Fixed64 depth, out bool clamped,
            out Vector3d source, out FixedPointAnchor exitAnchor));
        Assert.True(exitAnchor.TryGetPoint(out Vector3d exit));
        Assert.False(clamped);
        double expected = 101 / Math.Sqrt(2);
        Assert.InRange((double)depth, expected - 0.00000001, expected + 0.00000001);
        Assert.True(source.X.Abs() > Fixed64.One);
        Assert.Equal(Fixed64.One, exit.X.Abs());
        Assert.Equal((Fixed64)100, exit.Y.Abs());
    }

    [Theory]
    [InlineData(-1)]
    [InlineData(1)]
    public void TallCylinderClearanceRemainsClippedToTheFiniteOwner(int orientation)
    {
        FixedTriangle first = new(new Vector3d(-2, 2, -2),
            new Vector3d(2, -2, -2), new Vector3d(-2, 2, 2));
        FixedTriangle second = new(first.B, new Vector3d(2, -2, 2), first.C);
        FiniteSlabRayFrame frame = new(first, Vector3d.Zero, FixedQuaternion.Identity,
            Vector3d.Zero, FixedQuaternion.Identity, Fixed64.Zero, Fixed64.One, (Fixed64)100);
        Span<ulong> values = stackalloc ulong[FiniteSlabRaySelection.StorageWords];
        Span<sbyte> signs = stackalloc sbyte[FiniteSlabRaySelection.SignCount];
        FiniteSlabRaySelection maximum = new(values, signs);
        FiniteSlabRayExtrema.AccumulateTriangle(frame, first, ReadOnlySpan<Vector2d>.Empty, orientation, ref maximum);
        FiniteSlabRayExtrema.AccumulateTriangle(frame, second, ReadOnlySpan<Vector2d>.Empty, orientation, ref maximum);
        Assert.True(maximum.TryGetContactAnchor(frame, out Fixed64 depth, out bool clamped,
            out Vector3d source, out FixedPointAnchor exitAnchor));
        Assert.True(exitAnchor.TryGetPoint(out Vector3d exit));
        Assert.False(clamped);
        double expected = 3 * Math.Sqrt(2);
        Assert.InRange((double)depth, expected - 0.00000001, expected + 0.00000001);
        Assert.Equal(Fixed64.Two, source.X.Abs());
        Assert.Equal(Fixed64.One, exit.X.Abs());
        Assert.Equal((Fixed64)5, exit.Y.Abs());
    }

    [Fact]
    public void IntrinsicPlaneFamilyHasNoAuthoredTriangleBoundsOrProvenance()
    {
        FixedTriangle plane = new(Vector3d.Zero, new Vector3d((Fixed64).1, Fixed64.Zero, Fixed64.Zero),
            new Vector3d(Fixed64.Zero, Fixed64.Zero, (Fixed64).1));
        FiniteSlabRayFrame frame = new(plane, Vector3d.Zero, FixedQuaternion.Identity,
            Vector3d.Zero, FixedQuaternion.Identity, Fixed64.Zero, Fixed64.One, Fixed64.One);
        Span<ulong> values = stackalloc ulong[FiniteSlabRaySelection.StorageWords];
        Span<sbyte> signs = stackalloc sbyte[FiniteSlabRaySelection.SignCount];
        FiniteSlabRaySelection maximum = new(values, signs);
        List<Vector3d> points = new();
        void Visit(scoped in FiniteSlabRaySelection candidate)
        {
            Assert.Equal(0, candidate.Metadata.DomainEdgeMask);
            Assert.True(candidate.TryGetContactAnchor(frame, out _, out _, out Vector3d point, out FixedPointAnchor candidateExit));
            Assert.True(candidateExit.TryGetPoint(out _));
            points.Add(point);
        }
        FiniteSlabRayExtrema.AccumulatePlane(frame, ReadOnlySpan<Vector2d>.Empty, 1, ref maximum, Visit);
        Assert.Contains(new Vector3d(1, 0, 0), points);
        Assert.Contains(new Vector3d(-1, 0, 0), points);
        Assert.Contains(new Vector3d(0, 0, 1), points);
        Assert.Contains(new Vector3d(0, 0, -1), points);
        Assert.True(maximum.TryGetContactAnchor(frame, out Fixed64 depth, out bool clamped, out _, out FixedPointAnchor exit));
        Assert.True(exit.TryGetPoint(out _));
        Assert.Equal(Fixed64.One, depth);
        Assert.False(clamped);
    }

    [Fact]
    public void ObliqueStadiumClearanceUsesWholeShapeSupportBeyondTheInitialSection()
    {
        // x+y+2z=-3.5. Whole-shape support is sqrt(5)+2+2;
        // the projected source lies beyond the original stadium side.
        FixedTriangle plane = new(new Vector3d((Fixed64)(-10), (Fixed64)26.5, (Fixed64)(-10)),
            new Vector3d((Fixed64)10, (Fixed64)6.5, (Fixed64)(-10)),
            new Vector3d(Fixed64.Zero, (Fixed64)(-23.5), (Fixed64)10));
        FiniteSlabRayFrame frame = new(plane, Vector3d.Zero, FixedQuaternion.Identity,
            Vector3d.Zero, FixedQuaternion.Identity, Fixed64.Two, Fixed64.One, Fixed64.Two);
        Span<ulong> values = stackalloc ulong[FiniteSlabRaySelection.StorageWords];
        Span<sbyte> signs = stackalloc sbyte[FiniteSlabRaySelection.SignCount];
        FiniteSlabRaySelection maximum = new(values, signs);
        FiniteSlabRayExtrema.AccumulateTriangle(frame, plane, ReadOnlySpan<Vector2d>.Empty, 1, ref maximum);
        Assert.True(maximum.HasValue);
        Assert.True(maximum.TryGetContactAnchor(frame, out Fixed64 depth, out bool clamped,
            out Vector3d source, out FixedPointAnchor exitAnchor));
        Assert.True(exitAnchor.TryGetPoint(out Vector3d exit));
        Assert.False(clamped);
        double expected = (7.5 + Math.Sqrt(5)) / Math.Sqrt(6);
        Assert.InRange((double)depth, expected - 0.00000001, expected + 0.00000001);
        Assert.False(frame.Section.ContainsPoint(source));
        Assert.InRange((double)source.Z, -1.36, -1.34);
        Assert.InRange((double)exit.Z, 1.89, 1.90);
        Assert.Equal(Fixed64.Two, exit.Y);
    }

    [Fact]
    public void CylinderSectionWithFreeAxialCoordinateUsesARealBoundaryWitness()
    {
        FixedTriangle plane = new(new Vector3d((Fixed64).8, (Fixed64)(-3), (Fixed64)(-3)),
            new Vector3d((Fixed64).8, (Fixed64)3, (Fixed64)(-3)),
            new Vector3d((Fixed64).8, Fixed64.Zero, (Fixed64)3));
        FiniteSlabRayFrame frame = new(plane, Vector3d.Zero, FixedQuaternion.Identity,
            Vector3d.Zero, FixedQuaternion.Identity, Fixed64.Zero, Fixed64.One, Fixed64.One);
        Span<ulong> values = stackalloc ulong[FiniteSlabRaySelection.StorageWords];
        Span<sbyte> signs = stackalloc sbyte[FiniteSlabRaySelection.SignCount];
        FiniteSlabRaySelection maximum = new(values, signs);
        FiniteSlabRayExtrema.AccumulateTriangle(frame, plane, ReadOnlySpan<Vector2d>.Empty, 1, ref maximum);
        Assert.True(maximum.TryGetContactAnchor(frame, out Fixed64 depth, out bool clamped,
            out Vector3d source, out FixedPointAnchor exitAnchor));
        Assert.True(exitAnchor.TryGetPoint(out Vector3d exit));
        Assert.False(clamped);
        Assert.Equal(Fixed64.One - (Fixed64).8, depth);
        Assert.Equal(Fixed64.Zero, source.Z);
        Assert.Equal(Fixed64.One, exit.X);
        Assert.True(source.Y >= -Fixed64.One && source.Y <= Fixed64.One);
    }

    [Theory]
    [InlineData(false)]
    [InlineData(true)]
    public void VisitorReportsWholeSlabExitsAndPreservesIntrinsicCapCardinals(bool stadium)
    {
        FixedTriangle plane = new(new Vector3d(-5, 0, -5), new Vector3d(5, 0, -5), new Vector3d(0, 0, 5));
        FiniteSlabRayFrame frame = new(plane, Vector3d.Zero, FixedQuaternion.Identity,
            Vector3d.Zero, FixedQuaternion.Identity, stadium ? Fixed64.Two : Fixed64.Zero, Fixed64.One, Fixed64.One);
        Span<ulong> values = stackalloc ulong[FiniteSlabRaySelection.StorageWords];
        Span<sbyte> signs = stackalloc sbyte[FiniteSlabRaySelection.SignCount];
        FiniteSlabRaySelection maximum = new(values, signs);
        List<Vector3d> intrinsic = new();
        int callbacks = 0;
        void Visit(scoped in FiniteSlabRaySelection candidate)
        {
            Assert.True(candidate.TryGetContactAnchor(frame, out Fixed64 depth, out bool clamped,
                out Vector3d source, out FixedPointAnchor exitAnchor));
            Assert.True(exitAnchor.TryGetPoint(out Vector3d exit));
            Assert.False(clamped);
            Assert.Equal(Fixed64.One, depth);
            Assert.Equal(Fixed64.One, exit.Y);
            Assert.Equal(Fixed64.Zero, source.Y);
            callbacks++;
            if (candidate.Metadata.DomainEdgeMask == 0) intrinsic.Add(source);
        }
        FiniteSlabRayExtrema.AccumulateTriangle(frame, plane, ReadOnlySpan<Vector2d>.Empty, 1, ref maximum, Visit);
        Assert.True(callbacks >= 4);
        Assert.Contains(new Vector3d(1, 0, stadium ? 1 : 0), intrinsic);
        Assert.Contains(new Vector3d(-1, 0, stadium ? 1 : 0), intrinsic);
        Assert.Contains(new Vector3d(0, 0, stadium ? 2 : 1), intrinsic);
        Assert.Contains(new Vector3d(0, 0, stadium ? -2 : -1), intrinsic);
    }

    [Fact]
    public void ZeroRadiusAndConvexPrismUseTheSameBoundedLinearOwner()
    {
        FixedTriangle plane = new(new Vector3d(-3, 0, -3), new Vector3d(3, 0, -3), new Vector3d(0, 0, 3));
        Span<Vector2d> square = stackalloc Vector2d[4]
        { new(-1, -1), new(1, -1), new(1, 1), new(-1, 1) };
        Span<ulong> values = stackalloc ulong[FiniteSlabRaySelection.StorageWords];
        Span<sbyte> signs = stackalloc sbyte[FiniteSlabRaySelection.SignCount];
        for (int shape = 0; shape < 2; shape++)
        {
            FiniteSlabRayFrame frame = new(plane, Vector3d.Zero, FixedQuaternion.Identity,
                Vector3d.Zero, FixedQuaternion.Identity, Fixed64.Two, Fixed64.Zero, Fixed64.One);
            FiniteSlabRaySelection maximum = new(values, signs);
            FiniteSlabRayExtrema.AccumulateTriangle(frame, plane, shape == 0 ? ReadOnlySpan<Vector2d>.Empty : square, -1, ref maximum);
            Assert.True(maximum.TryGetContactAnchor(frame, out Fixed64 depth, out bool clamped,
                out _, out FixedPointAnchor exitAnchor));
            Assert.True(exitAnchor.TryGetPoint(out Vector3d exit));
            Assert.False(clamped);
            Assert.Equal(Fixed64.One, depth);
            Assert.Equal(-Fixed64.One, exit.Y);
        }
    }

    [Fact]
    public void ReversedDomainWindingAndCanonicalOwnerMetricGiveEqualDepths()
    {
        FixedTriangle owner = new(new Vector3d(-3, 0, -3), new Vector3d(3, 0, -3), new Vector3d(0, 0, 3));
        FixedTriangle reverse = new(owner.C, owner.B, owner.A);
        FiniteSlabRayFrame frame = new(owner, Vector3d.Zero, FixedQuaternion.Identity,
            Vector3d.Zero, FixedQuaternion.Identity, Fixed64.Zero, Fixed64.One, Fixed64.One);
        Span<ulong> values = stackalloc ulong[2 * FiniteSlabRaySelection.StorageWords];
        Span<sbyte> signs = stackalloc sbyte[2 * FiniteSlabRaySelection.SignCount];
        FiniteSlabRaySelection first = new(values[..FiniteSlabRaySelection.StorageWords], signs[..FiniteSlabRaySelection.SignCount]);
        FiniteSlabRaySelection second = new(values[FiniteSlabRaySelection.StorageWords..], signs[FiniteSlabRaySelection.SignCount..]);
        FiniteSlabRayExtrema.AccumulateTriangle(frame, owner, ReadOnlySpan<Vector2d>.Empty, 1, ref first);
        FiniteSlabRayExtrema.AccumulateTriangle(frame, reverse, ReadOnlySpan<Vector2d>.Empty, 1, ref second);
        Assert.Equal(0, first.CompareDepth(second));
    }
}
