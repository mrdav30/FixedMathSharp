using System;
using System.Collections.Generic;
using FixedMathSharp.Geometry;
using Xunit;

namespace FixedMathSharp.Tests.Geometry;

public sealed class FiniteSlabRayBoundaryTests
{
    [Theory]
    [InlineData(-1)]
    [InlineData(1)]
    public void ForwardCircleTraversalRetainsExitPolesAndSuppressesItsPositiveDepthEntryRoots(int orientation)
    {
        Fixed64 sourceX = (Fixed64)(-2 * orientation);
        FixedTriangle triangle = new(new Vector3d(sourceX, (Fixed64)(-5), (Fixed64)(-5)),
            new Vector3d(sourceX, (Fixed64)5, (Fixed64)(-5)),
            new Vector3d(sourceX, Fixed64.Zero, (Fixed64)5));
        FiniteSlabRayFrame frame = new(triangle, Vector3d.Zero, FixedQuaternion.Identity, Vector3d.Zero,
            FixedQuaternion.Identity, Fixed64.Zero, Fixed64.One, Fixed64.One);
        HashSet<Vector3d> exits = new();
        void Visit(scoped in FiniteSlabRaySelection candidate)
        {
            Assert.True(candidate.TryGetContactAnchor(frame, out Fixed64 depth, out bool clamped,
                out Vector3d source, out FixedPointAnchor anchor));
            Assert.False(clamped);
            Assert.Equal((Fixed64)3, depth);
            Assert.Equal(sourceX, source.X);
            Assert.True(anchor.TryGetPoint(out Vector3d exit));
            Assert.Equal((Fixed64)orientation, exit.X);
            Assert.Equal(source.Y, exit.Y);
            Assert.Equal(Fixed64.Zero, source.Z);
            Assert.Equal(Fixed64.Zero, exit.Z);
            exits.Add(exit);
        }
        Span<ulong> values = stackalloc ulong[FiniteSlabRaySelection.StorageWords];
        Span<sbyte> signs = stackalloc sbyte[FiniteSlabRaySelection.SignCount];
        FiniteSlabRaySelection maximum = new(values, signs);
        FiniteSlabRayExtrema.AccumulateTriangle(frame, triangle, ReadOnlySpan<Vector2d>.Empty,
            orientation, ref maximum, Visit);
        Assert.Equal(2, exits.Count);
        Assert.Contains(new Vector3d(orientation, -1, 0), exits);
        Assert.Contains(new Vector3d(orientation, 1, 0), exits);
        Assert.True(maximum.TryGetContactAnchor(frame, out Fixed64 maximumDepth, out _, out _, out _));
        Assert.Equal((Fixed64)3, maximumDepth);
    }

    [Theory]
    [InlineData(false, -1)]
    [InlineData(false, 1)]
    [InlineData(true, -1)]
    [InlineData(true, 1)]
    public void FiniteSourceDomainsBeyondTheSlabRejectBackwardRadialAndCapRoots(bool axial, int orientation)
    {
        Fixed64 direction = (Fixed64)orientation;
        Fixed64 coordinate = (axial ? Fixed64.Two : Fixed64.FromRaw(3 * Fixed64.One.m_rawValue / 2)) * direction;
        FixedTriangle triangle = axial
            ? new(new Vector3d(-Fixed64.Two, coordinate, -Fixed64.Two),
                new Vector3d(Fixed64.Two, coordinate, -Fixed64.Two),
                new Vector3d(Fixed64.Zero, coordinate, Fixed64.Two))
            : new(new Vector3d(coordinate, -Fixed64.Two, -Fixed64.Two),
                new Vector3d(coordinate, Fixed64.Two, -Fixed64.Two),
                new Vector3d(coordinate, Fixed64.Zero, Fixed64.Two));
        FiniteSlabRayFrame frame = new(triangle, Vector3d.Zero, FixedQuaternion.Identity, Vector3d.Zero,
            FixedQuaternion.Identity, Fixed64.Zero, Fixed64.One, Fixed64.One);
        int callbacks = 0;
        void Visit(scoped in FiniteSlabRaySelection candidate) => callbacks++;
        Span<ulong> values = stackalloc ulong[FiniteSlabRaySelection.StorageWords];
        Span<sbyte> signs = stackalloc sbyte[FiniteSlabRaySelection.SignCount];
        FiniteSlabRaySelection maximum = new(values, signs);
        FiniteSlabRayExtrema.AccumulateTriangle(frame, triangle, ReadOnlySpan<Vector2d>.Empty,
            orientation, ref maximum, Visit);
        Assert.False(maximum.HasValue);
        Assert.Equal(0, callbacks);
    }

    [Theory]
    [InlineData(-1)]
    [InlineData(1)]
    public void RationalAxialClearanceClampsDepthAtOppositeScalarLimitsWithoutLosingAnchors(int orientation)
    {
        Fixed64 sourceY = orientation > 0 ? Fixed64.MinValue : Fixed64.MaxValue;
        Fixed64 exitY = orientation > 0 ? Fixed64.MaxValue : Fixed64.MinValue;
        FixedTriangle plane = new(new Vector3d(-Fixed64.One, sourceY, -Fixed64.One), new Vector3d(Fixed64.One, sourceY, -Fixed64.One),
            new Vector3d(Fixed64.Zero, sourceY, Fixed64.One));
        FiniteSlabRayFrame frame = new(plane, Vector3d.Zero, FixedQuaternion.Identity,
            new Vector3d(Fixed64.Zero, exitY - (Fixed64)orientation, Fixed64.Zero), FixedQuaternion.Identity,
            Fixed64.Zero, Fixed64.One, Fixed64.One);
        Span<ulong> values = stackalloc ulong[FiniteSlabRaySelection.StorageWords];
        Span<sbyte> signs = stackalloc sbyte[FiniteSlabRaySelection.SignCount];
        FiniteSlabRaySelection maximum = new(values, signs);
        FiniteSlabRayExtrema.AccumulatePlane(frame, ReadOnlySpan<Vector2d>.Empty, orientation, ref maximum);
        Assert.True(maximum.TryGetContactAnchor(frame, out Fixed64 depth, out bool clamped,
            out Vector3d source, out FixedPointAnchor anchor));
        Assert.True(clamped);
        Assert.Equal(Fixed64.MaxValue, depth);
        Assert.Equal(sourceY, source.Y);
        Assert.True(anchor.TryGetPoint(out Vector3d exit));
        Assert.Equal(new Vector3d(source.X, exitY, source.Z), exit);
    }

    [Theory]
    [InlineData(-1)]
    [InlineData(1)]
    public void FinitePrismVisitorsSuppressInteriorSourceWitnessesAndRetainOnlyOutwardFaces(int orientation)
    {
        FixedTriangle plane = new(new Vector3d(0, -5, -5), new Vector3d(0, 5, -5), new Vector3d(0, 0, 5));
        Vector2d[] offsets = { new(-1, -1), new(1, -1), new(1, 1), new(-1, 1) };
        FiniteSlabRayFrame frame = new(plane, Vector3d.Zero, FixedQuaternion.Identity, Vector3d.Zero,
            FixedQuaternion.Identity, Fixed64.Zero, Fixed64.One, Fixed64.One);
        HashSet<Vector3d> exits = new();
        void Visit(scoped in FiniteSlabRaySelection candidate)
        {
            Assert.True(candidate.TryGetContactAnchor(frame, out Fixed64 depth, out bool clamped,
                out Vector3d source, out FixedPointAnchor anchor));
            Assert.False(clamped);
            Assert.Equal(Fixed64.One, depth);
            Assert.Equal(Fixed64.Zero, source.X);
            Assert.True(anchor.TryGetPoint(out Vector3d exit));
            Assert.Equal((Fixed64)orientation, exit.X);
            Assert.Equal(source.Y, exit.Y);
            Assert.Equal(source.Z, exit.Z);
            exits.Add(exit);
        }
        Span<ulong> values = stackalloc ulong[FiniteSlabRaySelection.StorageWords];
        Span<sbyte> signs = stackalloc sbyte[FiniteSlabRaySelection.SignCount];
        FiniteSlabRaySelection maximum = new(values, signs);
        FiniteSlabRayExtrema.AccumulateTriangle(frame, plane, offsets, orientation, ref maximum, Visit);
        Assert.Equal(4, exits.Count);
        foreach (int y in new[] { -1, 1 })
            foreach (int z in new[] { -1, 1 }) Assert.Contains(new Vector3d(orientation, y, z), exits);
    }

    [Theory]
    [InlineData(-1)]
    [InlineData(1)]
    public void ClosedSourceVertexOnAnOutwardCapRemainsAnExitWithoutAnExplicitActiveCap(int orientation)
    {
        Vector3d tangent = new(-orientation, orientation, 0);
        FixedTriangle triangle = new(tangent, new Vector3d(-2 * orientation, 2 * orientation, -2),
            new Vector3d(-2 * orientation, 2 * orientation, 2));
        FiniteSlabRayFrame frame = new(triangle, Vector3d.Zero, FixedQuaternion.Identity, Vector3d.Zero,
            FixedQuaternion.Identity, Fixed64.Zero, Fixed64.One, Fixed64.One);
        HashSet<Vector3d> exits = new();
        void Visit(scoped in FiniteSlabRaySelection candidate)
        {
            Assert.True(candidate.TryGetContactAnchor(frame, out Fixed64 depth, out bool clamped,
                out Vector3d source, out FixedPointAnchor anchor));
            Assert.False(clamped);
            Assert.Equal(Fixed64.Zero, depth);
            Assert.Equal(tangent, source);
            Assert.True(anchor.TryGetPoint(out Vector3d exit));
            Assert.Equal(tangent, exit);
            exits.Add(exit);
        }
        Span<ulong> values = stackalloc ulong[FiniteSlabRaySelection.StorageWords];
        Span<sbyte> signs = stackalloc sbyte[FiniteSlabRaySelection.SignCount];
        FiniteSlabRaySelection maximum = new(values, signs);
        FiniteSlabRayExtrema.AccumulateTriangle(frame, triangle, ReadOnlySpan<Vector2d>.Empty,
            orientation, ref maximum, Visit);
        Assert.Equal(tangent, Assert.Single(exits));
    }

    [Theory]
    [InlineData(5, 2, -1)]
    [InlineData(5, 2, 1)]
    [InlineData(15, 6, -1)]
    [InlineData(15, 6, 1)]
    public void OddCoreQuadraticChartDepthUsesNearestEvenForBothHalfwayParities(
        long lengthRaw, long expectedDepthRaw, int orientation)
    {
        FixedTriangle plane = new(new Vector3d(-3, 0, 4), new Vector3d(3, 0, -4), new Vector3d(0, 5, 0));
        FiniteSlabRayFrame frame = new(plane, Vector3d.Zero, FixedQuaternion.Identity, Vector3d.Zero,
            FixedQuaternion.Identity, Fixed64.FromRaw(lengthRaw), Fixed64.FromRaw(1), Fixed64.FromRaw(1));
        Span<ulong> values = stackalloc ulong[FiniteSlabRaySelection.StorageWords];
        Span<sbyte> signs = stackalloc sbyte[FiniteSlabRaySelection.SignCount];
        FiniteSlabRaySelection maximum = new(values, signs);
        FiniteSlabRayExtrema.AccumulatePlane(frame, ReadOnlySpan<Vector2d>.Empty, orientation, ref maximum);
        Assert.True(maximum.TryGetContactAnchor(frame, out Fixed64 depth, out bool clamped,
            out _, out FixedPointAnchor anchor));
        Assert.False(clamped);
        // The (4,0,3) normal gives radius + 3*length/10:
        // 2.5 and 5.5 raw units round to the neighboring even integers.
        Assert.Equal(Fixed64.FromRaw(expectedDepthRaw), depth);
        Assert.True(anchor.TryGetPoint(out Vector3d exit));
        Assert.Equal(Fixed64.FromRaw(orientation), exit.X);
        Assert.Equal(Fixed64.FromRaw(-1), exit.Y);
        Assert.Equal(Fixed64.FromRaw(orientation * (lengthRaw / 2 + 1)), exit.Z);
    }

    [Theory]
    [InlineData(-1)]
    [InlineData(1)]
    public void SlabSupportDepthMayExceedScalarRangeWhileEachPhysicalComponentRemainsRepresentable(int orientation)
    {
        FixedTriangle plane = new(new Vector3d(-5, 5, -5), new Vector3d(5, -5, -5), new Vector3d(0, 0, 5));
        FiniteSlabRayFrame frame = new(plane, Vector3d.Zero, FixedQuaternion.Identity, Vector3d.Zero,
            FixedQuaternion.Identity, Fixed64.Zero, Fixed64.MaxValue, Fixed64.MaxValue);
        Span<ulong> values = stackalloc ulong[FiniteSlabRaySelection.StorageWords];
        Span<sbyte> signs = stackalloc sbyte[FiniteSlabRaySelection.SignCount];
        FiniteSlabRaySelection maximum = new(values, signs);
        FiniteSlabRayExtrema.AccumulatePlane(frame, ReadOnlySpan<Vector2d>.Empty, orientation, ref maximum);
        Assert.True(maximum.TryGetContactAnchor(frame, out Fixed64 depth, out bool clamped,
            out Vector3d source, out FixedPointAnchor anchor));
        Assert.True(clamped);
        Assert.Equal(Fixed64.MaxValue, depth);
        Assert.Equal(Vector3d.Zero, source);
        Assert.True(anchor.TryGetPoint(out Vector3d exit));
        Fixed64 component = orientation > 0 ? Fixed64.MaxValue : -Fixed64.MaxValue;
        Assert.Equal(new Vector3d(component, component, Fixed64.Zero), exit);
    }

    [Theory]
    [InlineData(-1)]
    [InlineData(1)]
    public void FullRangeIrrationalClearanceClampsOnlyDepthAndPreservesFiniteAnchors(int orientation)
    {
        Fixed64 oneRaw = Fixed64.FromRaw(1);
        Fixed64 edge = orientation > 0 ? Fixed64.MinValue : Fixed64.MaxValue;
        Fixed64 step = orientation > 0 ? oneRaw : -oneRaw;
        Fixed64 centerEdge = (orientation > 0 ? Fixed64.MaxValue : Fixed64.MinValue) - step;
        FixedTriangle plane = new(new Vector3d(edge, Fixed64.Zero, edge + step * (Fixed64)4),
            new Vector3d(edge + step * (Fixed64)4, Fixed64.Zero, edge),
            new Vector3d(edge + step * Fixed64.Two, oneRaw, edge + step * Fixed64.Two));
        FiniteSlabRayFrame frame = new(plane, Vector3d.Zero, FixedQuaternion.Identity,
            new Vector3d(centerEdge, Fixed64.Zero, centerEdge), FixedQuaternion.Identity,
            Fixed64.Zero, oneRaw, oneRaw);
        Span<ulong> values = stackalloc ulong[FiniteSlabRaySelection.StorageWords];
        Span<sbyte> signs = stackalloc sbyte[FiniteSlabRaySelection.SignCount];
        FiniteSlabRaySelection maximum = new(values, signs);
        FiniteSlabRayExtrema.AccumulatePlane(frame, ReadOnlySpan<Vector2d>.Empty, orientation, ref maximum);
        Assert.True(maximum.TryGetContactAnchor(frame, out Fixed64 depth, out bool clamped,
            out Vector3d source, out FixedPointAnchor anchor));
        Assert.True(clamped);
        Assert.Equal(Fixed64.MaxValue, depth);
        Assert.Equal(new Vector3d(edge + step * Fixed64.Two, -oneRaw, edge + step * Fixed64.Two), source);
        Assert.True(anchor.TryGetPoint(out Vector3d exit));
        Assert.Equal(orientation > 0 ? Fixed64.MaxValue : Fixed64.MinValue, exit.X);
        Assert.Equal(exit.X, exit.Z);
        Assert.Equal(-oneRaw, exit.Y);
    }

    [Theory]
    [InlineData(false, -1, true)]
    [InlineData(false, 1, false)]
    [InlineData(true, -1, false)]
    [InlineData(true, 1, true)]
    public void IrrationalPlaneSupportPreservesAuthoredSourceRangeChecksAtBothScalarLimits(
        bool minimum, int orientation, bool representable)
    {
        Fixed64 edge = minimum ? Fixed64.MinValue : Fixed64.MaxValue;
        Fixed64 direction = minimum ? Fixed64.One : -Fixed64.One;
        Vector3d center = new(edge, Fixed64.Zero, Fixed64.Zero);
        FixedTriangle plane = new(center, new Vector3d(edge + direction, -Fixed64.Two * direction, Fixed64.Zero),
            new Vector3d(edge + direction, Fixed64.Zero, -direction));
        // Normal (2,1,2) projects the radial support a positive distance
        // past the center in X; the outward limit must remain a checked miss.
        FiniteSlabRayFrame frame = new(plane, Vector3d.Zero, FixedQuaternion.Identity, center,
            FixedQuaternion.Identity, Fixed64.Zero, Fixed64.One, Fixed64.FromRaw(1));
        Span<ulong> values = stackalloc ulong[FiniteSlabRaySelection.StorageWords];
        Span<sbyte> signs = stackalloc sbyte[FiniteSlabRaySelection.SignCount];
        FiniteSlabRaySelection maximum = new(values, signs);
        FiniteSlabRayExtrema.AccumulatePlane(frame, ReadOnlySpan<Vector2d>.Empty, orientation, ref maximum);
        Assert.True(maximum.HasValue);
        Assert.Equal(representable, maximum.TryGetContactAnchor(frame, out _, out bool clamped,
            out Vector3d source, out FixedPointAnchor anchor));
        if (representable)
        {
            Assert.False(clamped);
            Assert.True(anchor.TryGetPoint(out Vector3d exit));
            Assert.True(minimum ? source.X > edge && exit.X > edge : source.X < edge && exit.X < edge);
        }
    }

    [Fact]
    public void RetainedAlgebraicDepthReconstructsCellsBeyondTheCompactPrecisionAndKeepsExactOrdering()
    {
        const int words = FiniteSlabPolynomial.Words;
        Span<ulong> equationValues = stackalloc ulong[3 * words];
        Span<sbyte> equationSigns = stackalloc sbyte[3];
        equationValues.Clear(); equationSigns.Clear();
        // 2^1024(u-1)^2-2=0 has one root in [0,1):
        // 1-sqrt(2)/2^512, strictly below the rational comparison root.
        equationValues[0] = ulong.MaxValue - 1;
        for (int index = 1; index < 16; index++) equationValues[index] = ulong.MaxValue;
        equationValues[words + 16] = 2;
        equationValues[2 * words + 16] = 1;
        equationSigns[0] = 1; equationSigns[1] = -1; equationSigns[2] = 1;
        Span<ulong> cells = stackalloc ulong[64]; Span<int> shifts = stackalloc int[8];
        FiniteAxisValueRoots roots = WideFiniteAxisIntersection.GetFiniteValueRoots(equationValues, equationSigns, cells, shifts);
        Assert.Equal(1, roots.Count);
        Span<ulong> rootCell = stackalloc ulong[WideFiniteAxisIntersection.GetFiniteValueRootCellWords(equationValues, equationSigns)];
        FiniteAxisValueRoot root = FiniteAxisValueRoots.GetRoot(roots, 0, equationValues, equationSigns, rootCell);
        WideFiniteAxisIntersection.RefineFiniteValueRoot(ref root, 600);
        Assert.True(root.DenominatorShift >= 512);

        Span<ulong> chartValues = stackalloc ulong[6 * words]; Span<sbyte> chartSigns = stackalloc sbyte[6];
        chartValues.Clear(); chartSigns.Clear();
        FiniteSlabPolynomial zero = FiniteSlabPolynomial.At(chartValues, chartSigns, 0, 2);
        FiniteSlabPolynomial t = FiniteSlabPolynomial.At(chartValues, chartSigns, 2, 2);
        FiniteSlabPolynomial denominator = FiniteSlabPolynomial.At(chartValues, chartSigns, 4, 2);
        t.Set(-1); t.Set(2, 1); denominator.Set(1); denominator.Set(-1, 1);
        ulong[] values = new ulong[FiniteSlabRaySelection.StorageWords];
        sbyte[] signs = new sbyte[FiniteSlabRaySelection.SignCount];
        FiniteSlabRaySelection retained = new(values, signs);
        retained.Keep(root, roots.Count, zero, zero, t, denominator, true,
            zero.Slice(0, 1), zero.Slice(0, 1), default, 1, 0);
        FiniteSlabRaySelectionMetadata metadata = retained.Metadata;
        Assert.False(metadata.HasCompactCell);
        FiniteSlabRaySelection restored = new((ulong[])values.Clone(), (sbyte[])signs.Clone());
        restored.Restore(metadata);
        Assert.Equal(0, retained.CompareDepth(restored));

        equationValues.Clear(); equationSigns.Clear();
        // 2^512 u-(2^512-1)=0.
        for (int index = 0; index < 8; index++) equationValues[index] = ulong.MaxValue;
        equationValues[words + 8] = 1;
        equationSigns[0] = -1; equationSigns[1] = 1;
        roots = WideFiniteAxisIntersection.GetFiniteValueRoots(equationValues[..(2 * words)], equationSigns[..2], cells, shifts);
        Span<ulong> rationalCell = stackalloc ulong[WideFiniteAxisIntersection.GetFiniteValueRootCellWords(equationValues[..(2 * words)], equationSigns[..2])];
        root = FiniteAxisValueRoots.GetRoot(roots, 0, equationValues[..(2 * words)], equationSigns[..2], rationalCell);
        Span<ulong> otherValues = stackalloc ulong[FiniteSlabRaySelection.StorageWords];
        Span<sbyte> otherSigns = stackalloc sbyte[FiniteSlabRaySelection.SignCount];
        FiniteSlabRaySelection other = new(otherValues, otherSigns);
        other.Keep(root, roots.Count, zero, zero, t, denominator, true,
            zero.Slice(0, 1), zero.Slice(0, 1), default, 1, 0);
        Assert.True(retained.CompareDepth(other) < 0);
        Assert.True(other.CompareDepth(restored) > 0);
        retained.Keep(root, roots.Count, zero, zero, t, denominator, true,
            zero.Slice(0, 1), zero.Slice(0, 1), default, 1, 0);
        Assert.Equal(0, retained.CompareDepth(other));
        Assert.True(retained.CompareDepth(restored) > 0);
    }

    [Theory]
    [InlineData(0, false, -1)]
    [InlineData(0, false, 1)]
    [InlineData(0, true, -1)]
    [InlineData(0, true, 1)]
    [InlineData(1, false, -1)]
    [InlineData(1, false, 1)]
    [InlineData(1, true, -1)]
    [InlineData(1, true, 1)]
    [InlineData(2, false, -1)]
    [InlineData(2, false, 1)]
    [InlineData(2, true, -1)]
    [InlineData(2, true, 1)]
    public void VerticalDegenerateDomainsRetainClosedRadialExitsWithoutInventingCaps(
        int shape, bool tangent, int orientation)
    {
        FixedTriangle plane = new(new Vector3d(0, -5, -5), new Vector3d(0, 5, -5), new Vector3d(0, 0, 5));
        Fixed64 radius = shape == 2 ? Fixed64.Zero : Fixed64.One;
        FiniteSlabRayFrame frame = new(plane, Vector3d.Zero, FixedQuaternion.Identity, Vector3d.Zero,
            FixedQuaternion.Identity, shape == 0 ? Fixed64.Zero : Fixed64.Two, radius, Fixed64.One);
        Fixed64 tip = radius + (shape == 0 ? Fixed64.Zero : Fixed64.One);
        Vector3d first = tangent ? new(Fixed64.Zero, Fixed64.Zero, tip) : new(0, 0, -3);
        Vector3d last = tangent ? first : new(0, 0, 3);
        int exits = 0;
        void Visit(scoped in FiniteSlabRaySelection candidate)
        {
            Assert.True(candidate.TryGetContactAnchor(frame, out _, out bool clamped,
                out Vector3d source, out FixedPointAnchor anchor));
            Assert.False(clamped);
            Assert.True(anchor.TryGetPoint(out Vector3d exit));
            Assert.Equal(Fixed64.Zero, exit.Y);
            Assert.Equal(source.Z, exit.Z);
            if (tangent) Assert.Equal(first, exit);
            exits++;
        }
        Span<ulong> values = stackalloc ulong[FiniteSlabRaySelection.StorageWords];
        Span<sbyte> signs = stackalloc sbyte[FiniteSlabRaySelection.SignCount];
        FiniteSlabRaySelection maximum = new(values, signs);
        FiniteSlabRayExtrema.AccumulateTriangle(frame, new(first, last, last), ReadOnlySpan<Vector2d>.Empty,
            orientation, ref maximum, Visit);
        Assert.True(maximum.TryGetContactAnchor(frame, out Fixed64 depth, out bool clamped,
            out _, out FixedPointAnchor best));
        Assert.False(clamped);
        Assert.Equal(tangent ? Fixed64.Zero : radius, depth);
        Assert.True(best.TryGetPoint(out Vector3d point));
        Assert.Equal(tangent ? Fixed64.Zero : radius * (Fixed64)orientation, point.X);
        Assert.True(exits > 0);
    }

    [Theory]
    [InlineData(true)]
    [InlineData(false)]
    public void RetainedSelectionRejectsUndersizedValueOrSignStorage(bool shortValues)
    {
        ulong[] values = new ulong[FiniteSlabRaySelection.StorageWords - (shortValues ? 1 : 0)];
        sbyte[] signs = new sbyte[FiniteSlabRaySelection.SignCount - (shortValues ? 0 : 1)];
        Assert.Throws<ArgumentException>(() => { _ = new FiniteSlabRaySelection(values, signs); });
    }

    [Theory]
    [InlineData(-2)]
    [InlineData(0)]
    [InlineData(2)]
    public void PlaneClearanceRejectsInvalidOrientationBeforeAcceptingAnEmptyFrame(int orientation)
    {
        Assert.Throws<ArgumentOutOfRangeException>(() =>
        {
            FiniteSlabRaySelection maximum = new(new ulong[FiniteSlabRaySelection.StorageWords],
                new sbyte[FiniteSlabRaySelection.SignCount]);
            FiniteSlabRayExtrema.AccumulatePlane(default, Array.Empty<Vector2d>(), orientation, ref maximum);
        });
    }

    [Fact]
    public void DegeneratePlaneRetainsNoContactAndDoesNotInventAnchors()
    {
        Span<ulong> values = stackalloc ulong[FiniteSlabRaySelection.StorageWords];
        Span<sbyte> signs = stackalloc sbyte[FiniteSlabRaySelection.SignCount];
        FiniteSlabRaySelection maximum = new(values, signs);
        FiniteSlabRayExtrema.AccumulatePlane(default, ReadOnlySpan<Vector2d>.Empty, 1, ref maximum);
        Assert.False(maximum.HasValue);
        Assert.False(maximum.TryGetContactAnchor(default, out Fixed64 depth, out bool clamped,
            out Vector3d source, out FixedPointAnchor anchor));
        Assert.Equal(Fixed64.Zero, depth);
        Assert.False(clamped);
        Assert.Equal(Vector3d.Zero, source);
        Assert.Equal(default, anchor);
    }

    [Theory]
    [InlineData(0, -1, 5, true)]
    [InlineData(0, 1, 5, true)]
    [InlineData(1, -1, 7, true)]
    [InlineData(1, 1, 7, true)]
    [InlineData(2, -1, 4, true)]
    [InlineData(2, 1, 4, true)]
    [InlineData(3, -1, 1, true)]
    [InlineData(3, 1, 1, true)]
    [InlineData(0, -1, 4, false)]
    [InlineData(0, 1, 4, false)]
    [InlineData(1, -1, 6, false)]
    [InlineData(1, 1, 6, false)]
    [InlineData(2, 1, 4, false)]
    [InlineData(3, 1, 1, false)]
    public void HorizontalPlaneRetainsTheFullPhysicalCapFootprint(int shape, int orientation, int expectedPoints, bool includeInteriorSample)
    {
        FixedTriangle plane = new(new Vector3d(-5, 0, -5), new Vector3d(5, 0, -5), new Vector3d(0, 0, 5));
        Vector2d[] offsets = shape == 2
            ? new Vector2d[] { new(-1, 0), new(0, 1), new(1, 0), new(0, -1) } : Array.Empty<Vector2d>();
        FiniteSlabRayFrame frame = new(plane, Vector3d.Zero, FixedQuaternion.Identity, Vector3d.Zero,
            FixedQuaternion.Identity, shape == 1 ? Fixed64.Two : Fixed64.Zero,
            shape == 3 ? Fixed64.Zero : Fixed64.One, Fixed64.One);
        HashSet<Vector3d> points = new();
        void Visit(scoped in FiniteSlabRaySelection candidate)
        {
            Assert.True(candidate.TryGetContactAnchor(frame, out Fixed64 depth, out bool clamped,
                out Vector3d source, out FixedPointAnchor anchor));
            Assert.Equal(Fixed64.One, depth);
            Assert.False(clamped);
            Assert.Equal(Fixed64.Zero, source.Y);
            Assert.True(anchor.TryGetPoint(out Vector3d exit));
            Assert.Equal(new Vector3d(source.X, (Fixed64)orientation, source.Z), exit);
            points.Add(exit);
        }
        Span<ulong> values = stackalloc ulong[FiniteSlabRaySelection.StorageWords];
        Span<sbyte> signs = stackalloc sbyte[FiniteSlabRaySelection.SignCount];
        FiniteSlabRaySelection maximum = new(values, signs);
        FiniteSlabRayExtrema.AccumulatePlane(frame, offsets, orientation, ref maximum, Visit, includeInteriorSample);
        Assert.Equal(expectedPoints, points.Count);
    }

    [Fact]
    public void UnboundedPlaneRejectsAnUnrepresentableProjectedSourceWhileRetainingRepresentablePoles()
    {
        FixedTriangle plane = new(new Vector3d(-5, 0, -5), new Vector3d(5, 0, -5), new Vector3d(0, 0, 5));
        FiniteSlabRayFrame frame = new(plane, Vector3d.Zero, FixedQuaternion.Identity,
            new Vector3d(Fixed64.MaxValue, Fixed64.Zero, Fixed64.Zero), FixedQuaternion.Identity,
            Fixed64.Zero, Fixed64.One, Fixed64.One);
        int rejected = 0, retained = 0;
        void Visit(scoped in FiniteSlabRaySelection candidate)
        {
            if (!candidate.TryGetContactAnchor(frame, out _, out _, out Vector3d source, out FixedPointAnchor anchor))
            { rejected++; return; }
            Assert.True(anchor.TryGetPoint(out Vector3d exit));
            Assert.Equal(source.X, exit.X);
            Assert.Equal(Fixed64.One, exit.Y);
            retained++;
        }
        Span<ulong> values = stackalloc ulong[FiniteSlabRaySelection.StorageWords];
        Span<sbyte> signs = stackalloc sbyte[FiniteSlabRaySelection.SignCount];
        FiniteSlabRaySelection maximum = new(values, signs);
        FiniteSlabRayExtrema.AccumulatePlane(frame, ReadOnlySpan<Vector2d>.Empty, 1, ref maximum, Visit);
        Assert.Equal(1, rejected);
        Assert.Equal(4, retained);
    }

    [Theory]
    [InlineData(false, false, -1)]
    [InlineData(false, false, 1)]
    [InlineData(true, false, -1)]
    [InlineData(true, false, 1)]
    [InlineData(true, true, -1)]
    [InlineData(true, true, 1)]
    public void PhysicalSupportLineClipsItsMiddleAndClosedTipIndependentlyOfWindingAndSubdivision(
        bool prism, bool reversed, int orientation)
    {
        FixedTriangle plane = new(new Vector3d(-5, 5, -5), new Vector3d(5, -5, -5), new Vector3d(0, 0, 5));
        Vector2d[] offsets = prism
            ? new Vector2d[] { new(-1, -2), new(1, -2), new(1, 2), new(-1, 2) } : Array.Empty<Vector2d>();
        if (reversed) Array.Reverse(offsets);
        FiniteSlabRayFrame frame = new(plane, Vector3d.Zero, FixedQuaternion.Identity,
            Vector3d.Zero, FixedQuaternion.Identity, prism ? Fixed64.Zero : (Fixed64)4, Fixed64.One, Fixed64.One);
        Assert.Equal(1, frame.GetSupportDimension(offsets, orientation));
        ulong[] peakValues = new ulong[FiniteSlabRaySelection.StorageWords];
        sbyte[] peakSigns = new sbyte[FiniteSlabRaySelection.SignCount];
        FiniteSlabRaySelection peak = new(peakValues, peakSigns);
        FiniteSlabRayExtrema.AccumulatePlane(frame, offsets, orientation, ref peak);
        FiniteSlabRaySelectionMetadata metadata = peak.Metadata;
        HashSet<Vector3d> sources = new();
        void Visit(scoped in FiniteSlabRaySelection candidate)
        {
            FiniteSlabRaySelection planePeak = new(peakValues, peakSigns); planePeak.Restore(metadata);
            if (candidate.CompareDepth(planePeak) != 0) return;
            Assert.True(candidate.TryGetContactAnchor(frame, out _, out bool clamped, out Vector3d source, out FixedPointAnchor anchor));
            Assert.False(clamped);
            Assert.True(anchor.TryGetPoint(out Vector3d exit));
            Assert.Equal(new Vector3d((Fixed64)orientation, (Fixed64)orientation, source.Z), exit);
            Assert.Equal(Fixed64.Zero, source.X);
            Assert.Equal(Fixed64.Zero, source.Y);
            sources.Add(source);
        }
        Span<ulong> values = stackalloc ulong[FiniteSlabRaySelection.StorageWords];
        Span<sbyte> signs = stackalloc sbyte[FiniteSlabRaySelection.SignCount];
        for (int domain = 0; domain < 3; domain++)
        {
            FixedTriangle triangle = domain switch
            {
                0 => new(new Vector3d(-1, 1, -1), new Vector3d(1, -1, -1), new Vector3d(0, 0, 1)),
                1 => new(new Vector3d(0, 0, 2), new Vector3d(1, -1, 3), new Vector3d(-1, 1, 3)),
                _ => new(new Vector3d(0, 0, 3), new Vector3d(1, -1, 4), new Vector3d(-1, 1, 4))
            };
            sources.Clear(); FiniteSlabRaySelection clipped = new(values, signs);
            FiniteSlabRayExtrema.AccumulatePhysicalSupportTriangle(frame, triangle, offsets, orientation, ref clipped, Visit);
            if (domain == 0)
            {
                Assert.Equal(2, sources.Count);
                Assert.Contains(new Vector3d(0, 0, -1), sources);
                Assert.Contains(new Vector3d(0, 0, 1), sources);
                sources.Clear();
                Vector3d middle = (triangle.A + triangle.B) / Fixed64.Two;
                clipped = new(values, signs);
                FiniteSlabRayExtrema.AccumulatePhysicalSupportTriangle(frame, new(triangle.A, middle, triangle.C),
                    offsets, orientation, ref clipped, Visit);
                FiniteSlabRayExtrema.AccumulatePhysicalSupportTriangle(frame, new(middle, triangle.B, triangle.C),
                    offsets, orientation, ref clipped, Visit);
                Assert.Equal(2, sources.Count);
                Assert.Contains(new Vector3d(0, 0, -1), sources);
                Assert.Contains(new Vector3d(0, 0, 1), sources);
            }
            else if (domain == 1) Assert.Equal(new Vector3d(0, 0, 2), Assert.Single(sources));
            else Assert.Empty(sources);
        }
    }

    [Theory]
    [InlineData(0, false, -1)]
    [InlineData(0, false, 1)]
    [InlineData(1, false, -1)]
    [InlineData(1, false, 1)]
    [InlineData(2, false, -1)]
    [InlineData(2, false, 1)]
    [InlineData(0, true, -1)]
    [InlineData(0, true, 1)]
    [InlineData(1, true, -1)]
    [InlineData(1, true, 1)]
    [InlineData(2, true, -1)]
    [InlineData(2, true, 1)]
    public void PointSupportClippingIncludesInteriorAndClosedBoundaryButRejectsOutside(
        int domain, bool prism, int orientation)
    {
        FixedTriangle plane = new(new Vector3d(-5, 5, -5), new Vector3d(5, -5, -5), new Vector3d(0, 0, 5));
        FixedTriangle triangle = domain switch
        {
            0 => new(new Vector3d(-2, 2, -2), new Vector3d(2, -2, -2), new Vector3d(0, 0, 2)),
            1 => new(Vector3d.Zero, new Vector3d(2, -2, -2), new Vector3d(2, -2, 2)),
            _ => new(new Vector3d(-1, 1, -2), new Vector3d(-1, 1, 2), new Vector3d(-3, 3, 0))
        };
        Vector2d[] offsets = prism
            ? new Vector2d[] { new(-1, 0), new(0, 1), new(1, 0), new(0, -1) } : Array.Empty<Vector2d>();
        FiniteSlabRayFrame frame = new(plane, Vector3d.Zero, FixedQuaternion.Identity,
            Vector3d.Zero, FixedQuaternion.Identity, Fixed64.Zero, Fixed64.One, Fixed64.One);
        Assert.Equal(0, frame.GetSupportDimension(offsets, orientation));
        Span<ulong> values = stackalloc ulong[2 * FiniteSlabRaySelection.StorageWords];
        Span<sbyte> signs = stackalloc sbyte[2 * FiniteSlabRaySelection.SignCount];
        FiniteSlabRaySelection peak = new(values, signs);
        FiniteSlabRayExtrema.AccumulatePlane(frame, offsets, orientation, ref peak);
        ulong[] retainedValues = values[..FiniteSlabRaySelection.StorageWords].ToArray();
        sbyte[] retainedSigns = signs[..FiniteSlabRaySelection.SignCount].ToArray();
        FiniteSlabRaySelectionMetadata metadata = peak.Metadata;
        int admitted = 0;
        void Visit(scoped in FiniteSlabRaySelection candidate)
        {
            FiniteSlabRaySelection planePeak = new(retainedValues, retainedSigns); planePeak.Restore(metadata);
            if (candidate.CompareDepth(planePeak) != 0) return;
            Assert.True(candidate.TryGetContactAnchor(frame, out _, out bool clamped, out Vector3d source, out FixedPointAnchor anchor));
            Assert.False(clamped);
            Assert.Equal(Vector3d.Zero, source);
            Assert.True(anchor.TryGetPoint(out Vector3d exit));
            Assert.Equal(new Vector3d(orientation, orientation, 0), exit);
            admitted++;
        }
        FiniteSlabRaySelection clipped = new(values[FiniteSlabRaySelection.StorageWords..], signs[FiniteSlabRaySelection.SignCount..]);
        FiniteSlabRayExtrema.AccumulatePhysicalSupportTriangle(frame, triangle, offsets, orientation, ref clipped, Visit);
        if (domain == 2)
        {
            Assert.Equal(0, admitted);
            Assert.True(!clipped.HasValue || clipped.CompareDepth(peak) < 0);
        }
        else
        {
            Assert.Equal(1, admitted);
            Assert.Equal(0, clipped.CompareDepth(peak));
        }
    }
}
