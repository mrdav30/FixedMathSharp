//=======================================================================
// FixedTriangle.ProjectedCircleSweep.Tests.cs
//=======================================================================
// MIT License, Copyright (c) 2024-present David Oravsky (mrdav30)
// See LICENSE file in the project root for full license information.
//=======================================================================

using System;
using FixedMathSharp.Geometry;
using Xunit;

namespace FixedMathSharp.Tests;

public sealed class FixedTriangleProjectedCircleSweepTests
{
    [Theory]
    [InlineData(false, false, false)]
    [InlineData(false, false, true)]
    [InlineData(false, true, false)]
    [InlineData(false, true, true)]
    [InlineData(true, false, false)]
    [InlineData(true, false, true)]
    [InlineData(true, true, false)]
    [InlineData(true, true, true)]
    public void PositiveThicknessClip_RetainsExactBoundaryDistanceAndAnchor(bool reverse, bool reversedWinding, bool authored)
    {
        Vector3d a = new(0, -2, 0), b = new(4, 2, 0), c = new(0, -2, 4);
        var triangle = reversedWinding ? new FixedTriangle(c, b, a) : new FixedTriangle(a, b, c);
        var origin = new Vector3d(7, 11, -13);
        Vector2d start = new(reverse ? 13 : 5, -13), end = new(reverse ? 5 : 13, -13);
        Fixed64 distance;
        FixedPointAnchor contact;
        // No vertex survives the slab: its projected boundary is x=3/2 or 5/2,
        // whereas projecting the unclipped triangle would report an earlier hit.
        bool hit = authored
            ? triangle.TryGetFiniteSlabProjectedCircleSweep(origin, FixedQuaternion.Identity,
                new FixedSegment2d(start, end), (Fixed64)8, Fixed64.Half, (Fixed64)11, Fixed64.Half, out distance, out contact)
            : triangle.TryGetFiniteSlabProjectedCircleSweep(origin, FixedQuaternion.Identity,
                start, reverse ? Vector2d.Left : Vector2d.Right, (Fixed64)8, Fixed64.Half, (Fixed64)11, Fixed64.Half, out distance, out contact);
        Assert.True(hit);
        Assert.Equal((Fixed64)3, distance);
        Assert.Equal(origin, contact.Origin);
        Assert.Equal(FixedQuaternion.Identity, contact.Rotation);
        Assert.Equal(new Vector3d(Fixed64.FromFraction(reverse ? 5 : 3, 2), reverse ? Fixed64.Half : -Fixed64.Half, Fixed64.Zero), contact.LocalPoint);
        Vector2d direction = reverse ? Vector2d.Left : Vector2d.Right;
        Fixed64 before = distance - Fixed64.MinIncrement;
        Assert.False(authored
            ? triangle.TryGetFiniteSlabProjectedCircleSweep(origin, FixedQuaternion.Identity,
                new FixedSegment2d(start, start + direction * before), before, Fixed64.Half,
                (Fixed64)11, Fixed64.Half, out _, out _)
            : triangle.TryGetFiniteSlabProjectedCircleSweep(origin, FixedQuaternion.Identity,
                start, direction, before, Fixed64.Half, (Fixed64)11, Fixed64.Half, out _, out _));
        Assert.True(authored
            ? triangle.TryGetFiniteSlabProjectedCircleSweep(origin, FixedQuaternion.Identity,
                new FixedSegment2d(start, start + direction * distance), distance, Fixed64.Half,
                (Fixed64)11, Fixed64.Half, out _, out _)
            : triangle.TryGetFiniteSlabProjectedCircleSweep(origin, FixedQuaternion.Identity,
                start, direction, distance, Fixed64.Half, (Fixed64)11, Fixed64.Half, out _, out _));
        Span<Vector2d> projection = stackalloc Vector2d[4]
        {
            new(Fixed64.FromFraction(3, 2), Fixed64.Zero), new(Fixed64.FromFraction(5, 2), Fixed64.Zero),
            new(Fixed64.FromFraction(5, 2), Fixed64.FromFraction(3, 2)), new(Fixed64.FromFraction(3, 2), Fixed64.FromFraction(5, 2)),
        };
        Assert.True(FixedConvex2dRelations.TryGetSweptCircleFirstDistance(start, Fixed64.Half,
            reverse ? Vector2d.Left : Vector2d.Right, (Fixed64)8, new Vector2d(7, -13), projection,
            out Fixed64 projectedDistance, out _, out _));
        Assert.Equal(distance, projectedDistance);
    }

    [Fact]
    public void PositiveThicknessClip_QuarterTurnRetainsAuthoredAnchor()
    {
        var triangle = new FixedTriangle(new Vector3d(0, -2, 0), new Vector3d(4, 2, 0), new Vector3d(0, -2, 4));
        var origin = new Vector3d(7, 11, -13);
        var rotation = new FixedQuaternion(Fixed64.Zero, Fixed64.One, Fixed64.Zero, Fixed64.One).Normalized;
        Assert.True(triangle.TryGetFiniteSlabProjectedCircleSweep(origin, rotation,
            new Vector2d(7, -11), new Vector2d(0, -1), (Fixed64)8, Fixed64.Half,
            (Fixed64)11, Fixed64.Half, out Fixed64 distance, out FixedPointAnchor contact));
        Assert.Equal((Fixed64)3, distance);
        Assert.Equal(origin, contact.Origin);
        Assert.Equal(rotation, contact.Rotation);
        Assert.Equal(new Vector3d(Fixed64.FromFraction(3, 2), -Fixed64.Half, Fixed64.Zero), contact.LocalPoint);
        Assert.True(contact.TryGetPoint(out Vector3d world));
        Assert.Equal(new Vector3d((Fixed64)7, Fixed64.FromFraction(21, 2), Fixed64.FromFraction(-29, 2)), world);
    }

    [Fact]
    public void PositiveThicknessClip_PreservesUnmaterializedScalarFaceAnchor()
    {
        var triangle = new FixedTriangle(new Vector3d(0, -2, 0), new Vector3d(4, 2, 0), new Vector3d(0, -2, 4));
        var origin = new Vector3d(Fixed64.MaxValue, Fixed64.Zero, Fixed64.Zero);
        Assert.True(triangle.TryGetFiniteSlabProjectedCircleSweep(origin, FixedQuaternion.Identity,
            new Vector2d(Fixed64.MaxValue - Fixed64.Two, Fixed64.Zero), Vector2d.Right, (Fixed64)6,
            Fixed64.Half, Fixed64.Zero, Fixed64.Half, out Fixed64 distance, out FixedPointAnchor contact));
        Assert.Equal((Fixed64)3, distance);
        Assert.Equal(origin, contact.Origin);
        Assert.Equal(new Vector3d(Fixed64.FromFraction(3, 2), -Fixed64.Half, Fixed64.Zero), contact.LocalPoint);
        Assert.False(contact.TryGetPoint(out _));
    }

    [Theory]
    [InlineData(3, 4, 5, false)]
    [InlineData(3, 4, 5, true)]
    [InlineData(5, 12, 13, false)]
    [InlineData(5, 12, 13, true)]
    [InlineData(7, 24, 25, false)]
    [InlineData(7, 24, 25, true)]
    public void AuthoredPythagoreanTangent_ReturnsExactMidpointInBothDirections(int a, int b, int r, bool reverse)
    {
        var triangle = new FixedTriangle(new Vector3d(0, -2, 0), new Vector3d(0, 2, 0), new Vector3d(0, -2, -100));
        var tangent = new Vector2d(-b, a);
        var travel = new Vector2d(a, b);
        var segment = reverse
            ? new FixedSegment2d(tangent + travel, tangent - travel)
            : new FixedSegment2d(tangent - travel, tangent + travel);
        Fixed64 totalDistance = (Fixed64)(2 * r);

        Assert.True(triangle.TryGetFiniteSlabProjectedCircleSweep(Vector3d.Zero, FixedQuaternion.Identity, segment, totalDistance, (Fixed64)r, Fixed64.Zero, Fixed64.Half, out Fixed64 distance, out _));
        Assert.Equal((Fixed64)r, distance);
    }

    [Theory]
    [InlineData(1, 0)]
    [InlineData(2, 1)]
    [InlineData(3, 2)]
    [InlineData(5, 2)]
    [InlineData(7, 4)]
    public void AuthoredSegment_RoundsPhysicalHalfTicksToEven(long totalRaw, long expectedRaw)
    {
        var target = new FixedTriangle(new Vector3d(0, -2, 0), new Vector3d(0, 2, 0), new Vector3d(0, -2, -100));
        var segment = new FixedSegment2d(new Vector2d(-1, -1), new Vector2d(1, -1));

        Assert.True(target.TryGetFiniteSlabProjectedCircleSweep(Vector3d.Zero, FixedQuaternion.Identity, segment, Fixed64.FromRaw(totalRaw), Fixed64.Zero, Fixed64.Zero, Fixed64.Half, out Fixed64 distance, out _));
        Assert.Equal(Fixed64.FromRaw(expectedRaw), distance);
    }

    [Theory]
    [InlineData(false)]
    [InlineData(true)]
    public void AuthoredFullDomainSegment_PreservesWideEndpointDifference(bool reverse)
    {
        var triangle = new FixedTriangle(new Vector3d(0, -2, 0), new Vector3d(0, 2, 0), new Vector3d(0, -2, -100));
        Vector2d first = new(Fixed64.MinValue, -Fixed64.One);
        Vector2d second = new(Fixed64.MaxValue, -Fixed64.One);
        var segment = reverse ? new FixedSegment2d(second, first) : new FixedSegment2d(first, second);

        Assert.True(triangle.TryGetFiniteSlabProjectedCircleSweep(
            Vector3d.Zero, FixedQuaternion.Identity, segment,
            Fixed64.MaxValue, Fixed64.Zero, Fixed64.Zero, Fixed64.Half,
            out Fixed64 distance, out _));
        Assert.Equal(Fixed64.FromRaw((1L << 62) - (reverse ? 1L : 0L)), distance);
    }

    [Theory]
    [InlineData(1, 1)]
    [InlineData(3, 2)]
    public void AuthoredFullDomainSegment_CanMapToTinyPhysicalDistance(long totalRaw, long expectedRaw)
    {
        var triangle = new FixedTriangle(new Vector3d(0, -2, 0), new Vector3d(0, 2, 0), new Vector3d(0, -2, -100));
        var segment = new FixedSegment2d(new Vector2d(Fixed64.MinValue, -Fixed64.One), new Vector2d(Fixed64.MaxValue, -Fixed64.One));

        Assert.True(triangle.TryGetFiniteSlabProjectedCircleSweep(
            Vector3d.Zero, FixedQuaternion.Identity, segment,
            Fixed64.FromRaw(totalRaw), Fixed64.Zero, Fixed64.Zero, Fixed64.Half,
            out Fixed64 distance, out _));
        Assert.Equal(Fixed64.FromRaw(expectedRaw), distance);
    }

    [Theory]
    [InlineData(1, 1)]
    [InlineData(3, 2)]
    public void AuthoredFullDomainTangentPoint_CanMapToTinyPhysicalDistance(long totalRaw, long expectedRaw)
    {
        var triangle = new FixedTriangle(Vector3d.Zero, Vector3d.Zero, Vector3d.Zero);
        var segment = new FixedSegment2d(new Vector2d(Fixed64.MinValue, Fixed64.One), new Vector2d(Fixed64.MaxValue, Fixed64.One));

        Assert.True(triangle.TryGetFiniteSlabProjectedCircleSweep(
            Vector3d.Zero, FixedQuaternion.Identity, segment,
            Fixed64.FromRaw(totalRaw), Fixed64.One, Fixed64.Zero, Fixed64.Half,
            out Fixed64 distance, out _));
        Assert.Equal(Fixed64.FromRaw(expectedRaw), distance);
    }

    [Fact]
    public void AuthoredTinyDistanceMapping_RetainsExactFiniteEdgeRootAndWitness()
    {
        var triangle = new FixedTriangle(Vector3d.Zero, Vector3d.Right, Vector3d.Forward);
        var segment = new FixedSegment2d(new Vector2d(-1, -1), new Vector2d(2, 1));

        Assert.True(triangle.TryGetFiniteSlabProjectedCircleSweep(
            Vector3d.Zero, FixedQuaternion.Identity, segment,
            Fixed64.MinIncrement, Fixed64.Zero, Fixed64.Zero, Fixed64.Zero,
            out Fixed64 distance, out FixedPointAnchor contact));
        Assert.Equal(Fixed64.Zero, distance);
        Assert.Equal(new Vector3d(Fixed64.Half, Fixed64.Zero, Fixed64.Zero), contact.LocalPoint);
    }

    [Fact]
    public void AuthoredSegment_MapsLongPhysicalDistanceWithOneFinalRounding()
    {
        var target = new FixedTriangle(new Vector3d(0, -2, 0), new Vector3d(0, 2, 0), new Vector3d(0, -2, -100));
        var segment = new FixedSegment2d(new Vector2d(-1, -1), new Vector2d(2, -1));
        Fixed64 totalDistance = Fixed64.MaxValue;
        Fixed64 radius = Fixed64.Zero;

        Assert.True(target.TryGetFiniteSlabProjectedCircleSweep(Vector3d.Zero, FixedQuaternion.Identity, segment, totalDistance, radius, Fixed64.Zero, Fixed64.Half, out Fixed64 distance, out _));
        Assert.Equal(Fixed64.FromRaw(Fixed64.MaxValue.m_rawValue / 3L), distance);
    }

    [Fact]
    public void AuthoredSegment_TangentWithLongMappedDistanceRoundsHalfToEven()
    {
        var target = new FixedTriangle(new Vector3d(0, -2, 0), new Vector3d(0, 2, 0), new Vector3d(0, -2, -100));
        var segment = new FixedSegment2d(new Vector2d(-7, -1), new Vector2d(-1, 7));
        Fixed64 totalDistance = Fixed64.MaxValue;
        Fixed64 radius = (Fixed64)5;

        Assert.True(target.TryGetFiniteSlabProjectedCircleSweep(Vector3d.Zero, FixedQuaternion.Identity, segment, totalDistance, radius, Fixed64.Zero, Fixed64.Half, out Fixed64 distance, out _));
        Assert.Equal(Fixed64.FromRaw(1L << 62), distance);
    }

    [Theory]
    [InlineData(true)]
    [InlineData(false)]
    public void AuthoredPointSegment_UsesInclusiveOverlap(bool overlaps)
    {
        var target = new FixedTriangle(new Vector3d(0, -2, 0), new Vector3d(0, 2, 0), new Vector3d(0, -2, -100));
        Vector2d point = overlaps ? Vector2d.Zero : new Vector2d(-5, 5);
        var segment = new FixedSegment2d(point, point);
        Fixed64 totalDistance = Fixed64.Zero;
        Fixed64 radius = Fixed64.Zero;

        Assert.Equal(overlaps, target.TryGetFiniteSlabProjectedCircleSweep(Vector3d.Zero, FixedQuaternion.Identity, segment, totalDistance, radius, Fixed64.Zero, Fixed64.Half, out Fixed64 distance, out _));
        Assert.Equal(Fixed64.Zero, distance);
    }

    [Theory]
    [InlineData(0)]
    [InlineData(1)]
    [InlineData(2)]
    [InlineData(3)]
    public void AuthoredSegment_ValidatesPhysicalMappingAndDimensions(int invalid)
    {
        var target = new FixedTriangle(new Vector3d(0, -2, 0), new Vector3d(0, 2, 0), new Vector3d(0, -2, -100));
        var segment = new FixedSegment2d(new Vector2d(-1, -1), new Vector2d(2, -1));
        Fixed64 totalDistance = invalid == 0 ? -Fixed64.One : invalid == 1 ? Fixed64.Zero : Fixed64.One;
        Fixed64 radius = invalid == 2 ? -Fixed64.One : Fixed64.Zero;
        Fixed64 halfThickness = invalid == 3 ? -Fixed64.One : Fixed64.Half;

        Assert.ThrowsAny<ArgumentException>(() => target.TryGetFiniteSlabProjectedCircleSweep(Vector3d.Zero, FixedQuaternion.Identity, segment, totalDistance, radius, Fixed64.Zero, halfThickness, out _, out _));
    }

    [Fact]
    public void OddRawSlabClip_RoundsOnlyTheFinalSweepDistance()
    {
        Fixed64 raw = Fixed64.MinIncrement;
        var triangle = new FixedTriangle(
            new Vector3d(Fixed64.Zero, -raw, Fixed64.Zero),
            new Vector3d(raw, raw, Fixed64.Zero),
            new Vector3d(Fixed64.Zero, raw, raw));

        Assert.True(triangle.TryGetFiniteSlabProjectedCircleSweep(
            Vector3d.Zero,
            FixedQuaternion.Identity,
            new Vector2d(-raw, Fixed64.Zero),
            Vector2d.Right,
            Fixed64.FromRaw(4),
            Fixed64.Zero,
            Fixed64.Zero,
            Fixed64.Zero,
            out Fixed64 distance,
            out _));

        Assert.Equal(Fixed64.FromRaw(2), distance);
    }

    [Fact]
    public void CoplanarTriangle_ReturnsFirstExpandedEdgeDistance()
    {
        var triangle = new FixedTriangle(
            new Vector3d(Fixed64.Zero, Fixed64.Zero, -Fixed64.One),
            new Vector3d(Fixed64.Zero, Fixed64.Zero, Fixed64.One),
            new Vector3d(Fixed64.Two, Fixed64.Zero, Fixed64.Zero));

        Assert.True(triangle.TryGetFiniteSlabProjectedCircleSweep(
            Vector3d.Zero,
            FixedQuaternion.Identity,
            new Vector2d((Fixed64)(-2), Fixed64.Zero),
            Vector2d.Right,
            (Fixed64)5,
            Fixed64.Half,
            Fixed64.Zero,
            Fixed64.Zero,
            out Fixed64 distance,
            out FixedPointAnchor contact));

        Assert.Equal(Fixed64.FromFraction(3, 2), distance);
        Assert.Equal(Vector3d.Zero, contact.Origin);
        Assert.Equal(FixedQuaternion.Identity, contact.Rotation);
    }

    [Fact]
    public void CollapsedProjection_SweepsTheSingleRetainedPoint()
    {
        var triangle = new FixedTriangle(
            Vector3d.Zero,
            Vector3d.Zero,
            Vector3d.Zero);

        Assert.True(triangle.TryGetFiniteSlabProjectedCircleSweep(
            Vector3d.Zero,
            FixedQuaternion.Identity,
            new Vector2d(-Fixed64.Two, Fixed64.Zero),
            Vector2d.Right,
            (Fixed64)3,
            Fixed64.Half,
            Fixed64.Zero,
            Fixed64.Zero,
            out Fixed64 distance,
            out FixedPointAnchor contact));

        Assert.Equal(Fixed64.FromFraction(3, 2), distance);
        Assert.Equal(Vector3d.Zero, contact.LocalPoint);
    }

    [Fact]
    public void SweepMissingFiniteSlabProjection_ReturnsFalse()
    {
        var triangle = new FixedTriangle(
            new Vector3d(Fixed64.Zero, (Fixed64)2, -Fixed64.One),
            new Vector3d(Fixed64.Zero, (Fixed64)2, Fixed64.One),
            new Vector3d(Fixed64.Two, (Fixed64)2, Fixed64.Zero));

        Assert.False(triangle.TryGetFiniteSlabProjectedCircleSweep(
            Vector3d.Zero,
            FixedQuaternion.Identity,
            new Vector2d((Fixed64)(-2), Fixed64.Zero),
            Vector2d.Right,
            (Fixed64)5,
            Fixed64.Half,
            Fixed64.Zero,
            Fixed64.Zero,
            out _,
            out _));
    }

    [Theory]
    [InlineData(-2)]
    [InlineData(2)]
    public void ProjectedInteriorOutsideSlab_ReturnsFalse(int height)
    {
        var triangle = new FixedTriangle(
            new Vector3d(-Fixed64.One, (Fixed64)height, -Fixed64.One),
            new Vector3d(Fixed64.One, (Fixed64)height, -Fixed64.One),
            new Vector3d(Fixed64.Zero, (Fixed64)height, Fixed64.One));

        Assert.False(triangle.TryGetFiniteSlabProjectedCircleContact(
            Vector3d.Zero,
            FixedQuaternion.Identity,
            Vector2d.Zero,
            Fixed64.Zero,
            Fixed64.Zero,
            Fixed64.One,
            out _));
    }

    [Fact]
    public void SweepEndingBeforeProjectedBoundary_ReturnsFalse()
    {
        FixedTriangle triangle = CreateCoplanarProjectedTriangle();

        Assert.False(triangle.TryGetFiniteSlabProjectedCircleSweep(
            Vector3d.Zero,
            FixedQuaternion.Identity,
            new Vector2d((Fixed64)(-2), Fixed64.Zero),
            Vector2d.Right,
            Fixed64.One,
            Fixed64.Half,
            Fixed64.Zero,
            Fixed64.Zero,
            out _,
            out _));
    }

    [Fact]
    public void ZeroLengthSweepOutsideProjection_ReturnsFalse()
    {
        FixedTriangle triangle = CreateCoplanarProjectedTriangle();

        Assert.False(triangle.TryGetFiniteSlabProjectedCircleSweep(
            Vector3d.Zero,
            FixedQuaternion.Identity,
            new Vector2d((Fixed64)(-2), Fixed64.Zero),
            Vector2d.Right,
            Fixed64.Zero,
            Fixed64.Half,
            Fixed64.Zero,
            Fixed64.Zero,
            out _,
            out _));
    }

    [Fact]
    public void SweepCanHitBeforeMaximumWhoseClosestPointLiesBeyondIt()
    {
        FixedTriangle triangle = CreateCoplanarProjectedTriangle();

        Assert.True(triangle.TryGetFiniteSlabProjectedCircleSweep(
            Vector3d.Zero,
            FixedQuaternion.Identity,
            new Vector2d((Fixed64)(-2), Fixed64.Zero),
            Vector2d.Right,
            Fixed64.FromFraction(7, 4),
            Fixed64.Half,
            Fixed64.Zero,
            Fixed64.Zero,
            out Fixed64 distance,
            out _));

        Assert.Equal(Fixed64.FromFraction(3, 2), distance);
    }

    [Fact]
    public void SkewProjectedEdge_ReturnsExactDistance()
    {
        var triangle = new FixedTriangle(
            Vector3d.Zero,
            new Vector3d(Fixed64.One, Fixed64.Zero, Fixed64.Two),
            new Vector3d(Fixed64.Two, Fixed64.Zero, Fixed64.Zero));

        Assert.True(triangle.TryGetFiniteSlabProjectedCircleSweep(
            Vector3d.Zero,
            FixedQuaternion.Identity,
            new Vector2d(-Fixed64.One, Fixed64.Half),
            Vector2d.Right,
            (Fixed64)5,
            Fixed64.Zero,
            Fixed64.Zero,
            Fixed64.Zero,
            out Fixed64 distance,
            out _));

        Assert.Equal(Fixed64.FromFraction(5, 4), distance);
    }

    [Fact]
    public void SkewProjectedEdge_WithDominantSecondCrossTermReturnsExactDistance()
    {
        var triangle = new FixedTriangle(
            Vector3d.Zero,
            new Vector3d(Fixed64.One, Fixed64.Zero, Fixed64.Two),
            new Vector3d((Fixed64)(-2), Fixed64.Zero, Fixed64.Zero));

        Assert.True(triangle.TryGetFiniteSlabProjectedCircleSweep(
            Vector3d.Zero,
            FixedQuaternion.Identity,
            new Vector2d(Fixed64.Two, Fixed64.One),
            Vector2d.Left,
            (Fixed64)5,
            Fixed64.Zero,
            Fixed64.Zero,
            Fixed64.Zero,
            out Fixed64 distance,
            out _));

        Assert.Equal(Fixed64.FromFraction(3, 2), distance);
    }

    [Fact]
    public void SweepStartingOnSupportingLineBeyondSegment_ReturnsFalse()
    {
        FixedTriangle triangle = CreateCoplanarProjectedTriangle();

        Assert.False(triangle.TryGetFiniteSlabProjectedCircleSweep(
            Vector3d.Zero,
            FixedQuaternion.Identity,
            new Vector2d(Fixed64.Zero, Fixed64.Two),
            Vector2d.Right,
            Fixed64.One,
            Fixed64.Zero,
            Fixed64.Zero,
            Fixed64.Zero,
            out _,
            out _));
    }

    [Fact]
    public void EvenLowerMidpointTie_RoundsDownToEvenDistance()
    {
        Fixed64 raw = Fixed64.MinIncrement;
        var triangle = new FixedTriangle(
            new Vector3d(Fixed64.Zero, -raw, Fixed64.Zero),
            new Vector3d(raw, raw, Fixed64.Zero),
            new Vector3d(Fixed64.Zero, raw, raw));

        Assert.True(triangle.TryGetFiniteSlabProjectedCircleSweep(
            Vector3d.Zero,
            FixedQuaternion.Identity,
            new Vector2d(Fixed64.FromRaw(-2), Fixed64.Zero),
            Vector2d.Right,
            Fixed64.FromRaw(5),
            Fixed64.Zero,
            Fixed64.Zero,
            Fixed64.Zero,
            out Fixed64 distance,
            out _));

        Assert.Equal(Fixed64.FromRaw(2), distance);
    }

    [Fact]
    public void SlabPlaneThroughSecondEdgeVertex_RetainsPointContact()
    {
        var triangle = new FixedTriangle(
            new Vector3d(Fixed64.Zero, Fixed64.One, -Fixed64.One),
            new Vector3d(Fixed64.Zero, Fixed64.Zero, Fixed64.One),
            new Vector3d(Fixed64.Two, Fixed64.One, Fixed64.Zero));

        Assert.True(triangle.TryGetFiniteSlabProjectedCircleSweep(
            Vector3d.Zero,
            FixedQuaternion.Identity,
            new Vector2d((Fixed64)(-2), Fixed64.One),
            Vector2d.Right,
            (Fixed64)5,
            Fixed64.Half,
            Fixed64.Zero,
            Fixed64.Zero,
            out Fixed64 distance,
            out FixedPointAnchor contact));

        Assert.Equal(Fixed64.FromFraction(3, 2), distance);
        Assert.Equal(
            new Vector3d(Fixed64.Zero, Fixed64.Zero, Fixed64.One),
            contact.LocalPoint);
    }

    [Fact]
    public void CircleCenterInsideProjectedTriangle_ReturnsInteriorContact()
    {
        var triangle = new FixedTriangle(
            new Vector3d(-Fixed64.One, Fixed64.Zero, -Fixed64.One),
            new Vector3d(Fixed64.One, Fixed64.Zero, -Fixed64.One),
            new Vector3d(Fixed64.Zero, Fixed64.Zero, Fixed64.One));

        Assert.True(triangle.TryGetFiniteSlabProjectedCircleContact(
            Vector3d.Zero,
            FixedQuaternion.Identity,
            Vector2d.Zero,
            Fixed64.Zero,
            Fixed64.Zero,
            Fixed64.Zero,
            out FixedPointAnchor contact));
        Assert.True(contact.TryGetPoint(out Vector3d point));
        Assert.Equal(Vector3d.Zero, point);
    }

    [Fact]
    public void ScalarFaceContact_RetainsUnmaterializedTriangleWitness()
    {
        var triangle = new FixedTriangle(
            new Vector3d(Fixed64.One, Fixed64.Zero, -Fixed64.One),
            new Vector3d(Fixed64.One, Fixed64.Zero, Fixed64.One),
            new Vector3d(Fixed64.Two, Fixed64.Zero, Fixed64.Zero));

        Assert.True(triangle.TryGetFiniteSlabProjectedCircleContact(
            new Vector3d(Fixed64.MaxValue, Fixed64.Zero, Fixed64.Zero),
            FixedQuaternion.Identity,
            new Vector2d(Fixed64.MaxValue, Fixed64.Zero),
            Fixed64.One,
            Fixed64.Zero,
            Fixed64.Zero,
            out FixedPointAnchor contact));
        Assert.False(contact.TryGetPoint(out _));
        Assert.Equal(Fixed64.One, contact.LocalPoint.X);
    }

    [Fact]
    public void VertexOverlapMovingAway_ReturnsZeroDistance()
    {
        var triangle = new FixedTriangle(
            Vector3d.Zero,
            Vector3d.Right,
            Vector3d.Forward);

        Assert.True(triangle.TryGetFiniteSlabProjectedCircleSweep(
            Vector3d.Zero,
            FixedQuaternion.Identity,
            new Vector2d(
                -Fixed64.FromFraction(1, 4),
                -Fixed64.FromFraction(1, 4)),
            Vector2d.Left,
            Fixed64.One,
            Fixed64.Half,
            Fixed64.Zero,
            Fixed64.Zero,
            out Fixed64 distance,
            out FixedPointAnchor contact));

        Assert.Equal(Fixed64.Zero, distance);
        Assert.Equal(Vector3d.Zero, contact.LocalPoint);
    }

    [Fact]
    public void MirroredScalarFaceSweep_MatchesOriginCenteredDistance()
    {
        var triangle = new FixedTriangle(
            new Vector3d(Fixed64.Zero, Fixed64.Zero, -Fixed64.One),
            new Vector3d(Fixed64.Zero, Fixed64.Zero, Fixed64.One),
            new Vector3d(Fixed64.One, Fixed64.Zero, Fixed64.Zero));

        Assert.True(triangle.TryGetFiniteSlabProjectedCircleSweep(
            Vector3d.Zero,
            FixedQuaternion.Identity,
            new Vector2d((Fixed64)(-2), Fixed64.Zero),
            Vector2d.Right,
            (Fixed64)4,
            Fixed64.Half,
            Fixed64.Zero,
            Fixed64.Zero,
            out Fixed64 baseline,
            out _));
        Assert.True(triangle.TryGetFiniteSlabProjectedCircleSweep(
            new Vector3d(Fixed64.MaxValue, Fixed64.Zero, Fixed64.Zero),
            FixedQuaternion.Identity,
            new Vector2d(
                Fixed64.MaxValue - Fixed64.Two,
                Fixed64.Zero),
            Vector2d.Right,
            (Fixed64)4,
            Fixed64.Half,
            Fixed64.Zero,
            Fixed64.Zero,
            out Fixed64 scalarFace,
            out _));

        Assert.Equal(Fixed64.FromFraction(3, 2), baseline);
        Assert.Equal(baseline, scalarFace);
    }

    [Fact]
    public void FullDomainEdgeSweep_PreservesRationalSegmentDistance()
    {
        var triangle = new FixedTriangle(
            new Vector3d(Fixed64.MinValue, Fixed64.Zero, Fixed64.Zero),
            new Vector3d(Fixed64.MaxValue, Fixed64.Zero, Fixed64.Zero),
            new Vector3d(Fixed64.MinValue, Fixed64.Zero, Fixed64.One));

        Assert.True(triangle.TryGetFiniteSlabProjectedCircleSweep(
            Vector3d.Zero,
            FixedQuaternion.Identity,
            new Vector2d(Fixed64.Zero, -Fixed64.Two),
            Vector2d.Forward,
            Fixed64.Two,
            Fixed64.Half,
            Fixed64.Zero,
            Fixed64.Zero,
            out Fixed64 distance,
            out _));

        Assert.Equal(Fixed64.FromFraction(3, 2), distance);
    }

    [Fact]
    public void ProjectedVerticalEdge_UsesNearestBoundaryWitnessAcrossWinding()
    {
        Fixed64 originY = Fixed64.MaxValue - (Fixed64)4;
        Fixed64 quarter = Fixed64.FromFraction(1, 4);
        int[,] permutations =
        {
            { 0, 1, 2 },
            { 0, 2, 1 },
            { 1, 0, 2 },
            { 1, 2, 0 },
            { 2, 0, 1 },
            { 2, 1, 0 },
        };
        foreach (bool mirrored in new[] { false, true })
        {
            Fixed64 collapsedY = mirrored ? Fixed64.One : -Fixed64.One;
            Fixed64 boundaryY = mirrored ? -quarter : quarter;
            Vector3d[] vertices =
            {
                new(Fixed64.Zero, collapsedY, Fixed64.Zero),
                new(Fixed64.Zero, boundaryY, Fixed64.Zero),
                new(Fixed64.One, boundaryY, Fixed64.One),
            };
            for (int index = 0; index < permutations.GetLength(0); index++)
            {
                var triangle = new FixedTriangle(
                    vertices[permutations[index, 0]],
                    vertices[permutations[index, 1]],
                    vertices[permutations[index, 2]]);

                Assert.True(triangle.TryGetFiniteSlabProjectedCircleSweep(
                    new Vector3d(Fixed64.Zero, originY, Fixed64.Zero),
                    FixedQuaternion.Identity,
                    new Vector2d((Fixed64)(-3), Fixed64.Zero),
                    Vector2d.Right,
                    (Fixed64)6,
                    Fixed64.Half,
                    originY,
                    Fixed64.One,
                    out Fixed64 distance,
                    out FixedPointAnchor contact));

                Assert.Equal(Fixed64.FromFraction(5, 2), distance);
                Assert.Equal(boundaryY, contact.LocalPoint.Y);
            }
        }
    }

    [Fact]
    public void WarmedProjectedCircleSweep_DoesNotAllocate()
    {
        var triangle = new FixedTriangle(
            new Vector3d(Fixed64.Zero, Fixed64.Zero, -Fixed64.One),
            new Vector3d(Fixed64.Zero, Fixed64.Zero, Fixed64.One),
            new Vector3d(Fixed64.Two, Fixed64.Zero, Fixed64.Zero));
        _ = triangle.TryGetFiniteSlabProjectedCircleSweep(
            Vector3d.Zero,
            FixedQuaternion.Identity,
            new Vector2d((Fixed64)(-2), Fixed64.Zero),
            Vector2d.Right,
            (Fixed64)5,
            Fixed64.Half,
            Fixed64.Zero,
            Fixed64.Zero,
            out _,
            out _);

        bool allSucceeded = true;
        long allocated = FixedMathTestHelper.MeasureWarmedAllocations(() =>
        {
            for (int iteration = 0; iteration < 32; iteration++)
            {
                allSucceeded &= triangle.TryGetFiniteSlabProjectedCircleSweep(
                    Vector3d.Zero, FixedQuaternion.Identity,
                    new Vector2d((Fixed64)(-2), Fixed64.Zero), Vector2d.Right,
                    (Fixed64)5, Fixed64.Half, Fixed64.Zero, Fixed64.Zero, out _, out _);
                allSucceeded &= triangle.TryGetFiniteSlabProjectedCircleSweep(
                    Vector3d.Zero, FixedQuaternion.Identity,
                    new FixedSegment2d(new Vector2d(-2, 0), new Vector2d(3, 0)),
                    (Fixed64)5, Fixed64.Half, Fixed64.Zero, Fixed64.Zero, out _, out _);
            }
        });

        Assert.True(allSucceeded);
        Assert.Equal(0L, allocated);
    }

    [Fact]
    public void ProjectedCircleContracts_RejectInvalidGeometry()
    {
        var triangle = new FixedTriangle(
            Vector3d.Zero,
            Vector3d.Right,
            Vector3d.Forward);
        FixedQuaternion nonUnitRotation = new(
            Fixed64.Two,
            Fixed64.Zero,
            Fixed64.Zero,
            Fixed64.Zero);

        Assert.Throws<ArgumentException>(() =>
            triangle.TryGetFiniteSlabProjectedCircleContact(
                Vector3d.Zero,
                nonUnitRotation,
                Vector2d.Zero,
                Fixed64.Zero,
                Fixed64.Zero,
                Fixed64.Zero,
                out _));
        Assert.Throws<ArgumentOutOfRangeException>(() =>
            triangle.TryGetFiniteSlabProjectedCircleContact(
                Vector3d.Zero,
                FixedQuaternion.Identity,
                Vector2d.Zero,
                -Fixed64.MinIncrement,
                Fixed64.Zero,
                Fixed64.Zero,
                out _));
        Assert.Throws<ArgumentOutOfRangeException>(() =>
            triangle.TryGetFiniteSlabProjectedCircleContact(
                Vector3d.Zero,
                FixedQuaternion.Identity,
                Vector2d.Zero,
                Fixed64.Zero,
                Fixed64.Zero,
                -Fixed64.MinIncrement,
                out _));
        Assert.Throws<ArgumentException>(() =>
            triangle.TryGetFiniteSlabProjectedCircleSweep(
                Vector3d.Zero,
                FixedQuaternion.Identity,
                Vector2d.Zero,
                Vector2d.Zero,
                Fixed64.Zero,
                Fixed64.Zero,
                Fixed64.Zero,
                Fixed64.Zero,
                out _,
                out _));
        Assert.Throws<ArgumentOutOfRangeException>(() =>
            triangle.TryGetFiniteSlabProjectedCircleSweep(
                Vector3d.Zero,
                FixedQuaternion.Identity,
                Vector2d.Zero,
                Vector2d.Right,
                -Fixed64.MinIncrement,
                Fixed64.Zero,
                Fixed64.Zero,
                Fixed64.Zero,
                out _,
                out _));
        Assert.Throws<ArgumentOutOfRangeException>(() =>
            triangle.TryGetFiniteSlabProjectedCircleSweep(
                Vector3d.Zero,
                FixedQuaternion.Identity,
                Vector2d.Zero,
                Vector2d.Right,
                Fixed64.Zero,
                -Fixed64.MinIncrement,
                Fixed64.Zero,
                Fixed64.Zero,
                out _,
                out _));
        Assert.Throws<ArgumentOutOfRangeException>(() =>
            triangle.TryGetFiniteSlabProjectedCircleSweep(
                Vector3d.Zero,
                FixedQuaternion.Identity,
                Vector2d.Zero,
                Vector2d.Right,
                Fixed64.Zero,
                Fixed64.Zero,
                Fixed64.Zero,
                -Fixed64.MinIncrement,
                out _,
                out _));
    }

    [Fact]
    public void DegenerateTriangle_FallsBackToItsBoundaryFeature()
    {
        var triangle = new FixedTriangle(
            Vector3d.Zero,
            Vector3d.Right,
            Vector3d.Right);

        Assert.True(triangle.TryGetFiniteSlabProjectedCircleContact(
            Vector3d.Zero,
            FixedQuaternion.Identity,
            Vector2d.Zero,
            Fixed64.One,
            Fixed64.Zero,
            Fixed64.One,
            out FixedPointAnchor contact));
        Assert.Equal(Vector3d.Zero, contact.LocalPoint);
    }

    private static FixedTriangle CreateCoplanarProjectedTriangle() =>
        new(
            new Vector3d(Fixed64.Zero, Fixed64.Zero, -Fixed64.One),
            new Vector3d(Fixed64.Zero, Fixed64.Zero, Fixed64.One),
            new Vector3d(Fixed64.Two, Fixed64.Zero, Fixed64.Zero));
}
