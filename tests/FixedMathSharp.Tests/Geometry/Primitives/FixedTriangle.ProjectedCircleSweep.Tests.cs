//=======================================================================
// FixedTriangle.ProjectedCircleSweep.Tests.cs
//=======================================================================
// MIT License, Copyright (c) 2024-present David Oravsky (mrdav30)
// See LICENSE file in the project root for full license information.
//=======================================================================

using FixedMathSharp.Bounds;
using System;
using Xunit;

namespace FixedMathSharp.Tests;

public sealed class FixedTriangleProjectedCircleSweepTests
{
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

        long before = GC.GetAllocatedBytesForCurrentThread();
        bool allSucceeded = true;
        for (int iteration = 0; iteration < 32; iteration++)
        {
            allSucceeded &= triangle.TryGetFiniteSlabProjectedCircleSweep(
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
        }
        long after = GC.GetAllocatedBytesForCurrentThread();

        Assert.True(allSucceeded);
        Assert.Equal(before, after);
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
