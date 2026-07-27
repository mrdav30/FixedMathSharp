//=======================================================================
// CenteredFiniteSurfaceEdgeContracts.Tests.cs
//=======================================================================
// MIT License, Copyright (c) 2024–present David Oravsky (mrdav30)
// See LICENSE file in the project root for full license information.
//=======================================================================

using FixedMathSharp.Bounds;
using System;
using Xunit;

namespace FixedMathSharp.Tests;

public sealed class CenteredFiniteSurfaceEdgeContractsTests
{
    // These consecutive raw values straddle sqrt(2^63), the last radial
    // component boundary around a vector whose first component is MaxValue.
    private const long BelowScalarLimitMidpointRaw = 3_037_000_499L;
    private const long AboveScalarLimitMidpointRaw = 3_037_000_500L;

    [Fact]
    public void CapsuleSurfaceAnchor_ValidatesItsRigidFrameAndRadialPlane()
    {
        ArgumentException rotation = Assert.Throws<ArgumentException>(() =>
            FixedSegment.TryGetClosestCenteredCapsuleSurfaceAnchor(
                Vector3d.Zero,
                Vector3d.Zero,
                FixedQuaternion.Zero,
                Vector3d.Up,
                Fixed64.Two,
                Fixed64.One,
                Vector3d.Right,
                out _,
                out _,
                out _));
        Assert.Equal("frameRotation", rotation.ParamName);

        ArgumentException radial = Assert.Throws<ArgumentException>(() =>
            FixedSegment.TryGetClosestCenteredCapsuleSurfaceAnchor(
                Vector3d.Zero,
                Vector3d.Zero,
                FixedQuaternion.Identity,
                Vector3d.Up,
                Fixed64.Two,
                Fixed64.One,
                Vector3d.Up,
                out _,
                out _,
                out _));
        Assert.Equal("localFallbackRadialDirection", radial.ParamName);
    }

    [Fact]
    public void CapsuleSurfaceAnchor_RejectsOnlyUnrepresentableFinalResults()
    {
        Assert.False(
            FixedSegment.TryGetClosestCenteredCapsuleSurfaceAnchor(
                new Vector3d(
                    Fixed64.MinValue,
                    Fixed64.Zero,
                    Fixed64.Zero),
                new Vector3d(
                    Fixed64.MaxValue,
                    Fixed64.Zero,
                    Fixed64.Zero),
                FixedQuaternion.Identity,
                Vector3d.Up,
                Fixed64.Two,
                Fixed64.One,
                Vector3d.Right,
                out _,
                out _,
                out _));

        Assert.False(
            FixedSegment.TryGetClosestCenteredCapsuleSurfaceAnchor(
                new Vector3d(
                    Fixed64.MaxValue,
                    Fixed64.MaxValue,
                    Fixed64.Zero),
                Vector3d.Zero,
                FixedQuaternion.Identity,
                Vector3d.Up,
                Fixed64.Two,
                Fixed64.Zero,
                Vector3d.Right,
                out _,
                out _,
                out _));
    }

    [Fact]
    public void CapsuleSurfaceAnchors_PreserveTheNegativeCapFeature()
    {
        Assert.True(
            FixedSegment.TryGetClosestCenteredCapsuleSurfaceAnchor(
                new Vector3d(0, -3, 0),
                Vector3d.Zero,
                FixedQuaternion.Identity,
                Vector3d.Up,
                Fixed64.Two,
                Fixed64.One,
                Vector3d.Right,
                out FixedPointAnchor closest,
                out Vector3d normal,
                out Fixed64 signedDistance));
        Assert.True(closest.TryGetPoint(out Vector3d closestPoint));
        Assert.Equal(new Vector3d(0, -2, 0), closestPoint);
        Assert.Equal(Vector3d.Down, normal);
        Assert.Equal(Fixed64.One, signedDistance);

        FixedPointAnchor selected =
            FixedSegment.GetSurfaceAnchorOnCenteredCapsule(
                new Vector3d(0, -3, 0),
                Vector3d.Zero,
                FixedQuaternion.Identity,
                Vector3d.Up,
                Fixed64.Two,
                Fixed64.One,
                Vector3d.Down);
        Assert.True(selected.TryGetPoint(out Vector3d selectedPoint));
        Assert.Equal(new Vector3d(0, -2, 0), selectedPoint);
    }

    [Fact]
    public void CylinderSurfaceAnchor_DistinguishesContainedSideCapAndExteriorRim()
    {
        Assert.True(
            FixedSegment.TryGetClosestCenteredFiniteCylinderSurfaceAnchor(
                Vector3d.Zero,
                Vector3d.Zero,
                FixedQuaternion.Identity,
                Vector3d.Up,
                (Fixed64)4,
                Fixed64.One,
                Vector3d.Right,
                out FixedPointAnchor side,
                out Vector3d sideNormal,
                out Fixed64 sideDistance));
        Assert.True(side.TryGetPoint(out Vector3d sidePoint));
        Assert.Equal(Vector3d.Right, sidePoint);
        Assert.Equal(Vector3d.Right, sideNormal);
        Assert.Equal(-Fixed64.One, sideDistance);

        Vector3d nearCap = new(
            Fixed64.Zero,
            Fixed64.FromFraction(7, 4),
            Fixed64.Zero);
        Assert.True(
            FixedSegment.TryGetClosestCenteredFiniteCylinderSurfaceAnchor(
                nearCap,
                Vector3d.Zero,
                FixedQuaternion.Identity,
                Vector3d.Up,
                (Fixed64)4,
                Fixed64.One,
                Vector3d.Right,
                out FixedPointAnchor cap,
                out Vector3d capNormal,
                out Fixed64 capDistance));
        Assert.True(cap.TryGetPoint(out Vector3d capPoint));
        Assert.Equal(new Vector3d(0, 2, 0), capPoint);
        Assert.Equal(Vector3d.Up, capNormal);
        Assert.Equal(-Fixed64.FromFraction(1, 4), capDistance);

        Assert.True(
            FixedSegment.TryGetClosestCenteredFiniteCylinderSurfaceAnchor(
                new Vector3d(2, 3, 0),
                Vector3d.Zero,
                FixedQuaternion.Identity,
                Vector3d.Up,
                (Fixed64)4,
                Fixed64.One,
                Vector3d.Right,
                out FixedPointAnchor rim,
                out Vector3d rimNormal,
                out Fixed64 rimDistance));
        Assert.True(rim.TryGetPoint(out Vector3d rimPoint));
        Assert.Equal(new Vector3d(1, 2, 0), rimPoint);
        Assert.Equal(
            new Vector3d(
                FixedMath.Sqrt(Fixed64.Half),
                FixedMath.Sqrt(Fixed64.Half),
                Fixed64.Zero),
            rimNormal);
        Assert.Equal(FixedMath.Sqrt(Fixed64.Two), rimDistance);

        Assert.True(
            FixedSegment.TryGetClosestCenteredFiniteCylinderSurfaceAnchor(
                new Vector3d(
                    Fixed64.Zero,
                    -Fixed64.FromFraction(7, 4),
                    Fixed64.Zero),
                Vector3d.Zero,
                FixedQuaternion.Identity,
                Vector3d.Up,
                (Fixed64)4,
                Fixed64.One,
                Vector3d.Right,
                out FixedPointAnchor negativeCap,
                out Vector3d negativeCapNormal,
                out Fixed64 negativeCapDistance));
        Assert.True(negativeCap.TryGetPoint(out Vector3d negativeCapPoint));
        Assert.Equal(new Vector3d(0, -2, 0), negativeCapPoint);
        Assert.Equal(Vector3d.Down, negativeCapNormal);
        Assert.Equal(-Fixed64.FromFraction(1, 4), negativeCapDistance);

        Assert.True(
            FixedSegment.TryGetClosestCenteredFiniteCylinderSurfaceAnchor(
                new Vector3d(2, -3, 0),
                Vector3d.Zero,
                FixedQuaternion.Identity,
                Vector3d.Up,
                (Fixed64)4,
                Fixed64.One,
                Vector3d.Right,
                out FixedPointAnchor negativeRim,
                out Vector3d negativeRimNormal,
                out Fixed64 negativeRimDistance));
        Assert.True(negativeRim.TryGetPoint(out Vector3d negativeRimPoint));
        Assert.Equal(new Vector3d(1, -2, 0), negativeRimPoint);
        Assert.Equal(
            new Vector3d(
                FixedMath.Sqrt(Fixed64.Half),
                -FixedMath.Sqrt(Fixed64.Half),
                Fixed64.Zero),
            negativeRimNormal);
        Assert.Equal(FixedMath.Sqrt(Fixed64.Two), negativeRimDistance);
    }

    [Fact]
    public void CylinderSurfaceAnchor_RejectsUnrepresentableRadialAndFinalDistances()
    {
        Assert.False(
            FixedSegment.TryGetClosestCenteredFiniteCylinderSurfaceAnchor(
                new Vector3d(
                    Fixed64.MaxValue,
                    Fixed64.Zero,
                    Fixed64.MaxValue),
                Vector3d.Zero,
                FixedQuaternion.Identity,
                Vector3d.Up,
                Fixed64.Two,
                Fixed64.Zero,
                Vector3d.Right,
                out _,
                out _,
                out _));

        Assert.False(
            FixedSegment.TryGetClosestCenteredFiniteCylinderSurfaceAnchor(
                new Vector3d(
                    Fixed64.MaxValue,
                    Fixed64.MaxValue,
                    Fixed64.Zero),
                Vector3d.Zero,
                FixedQuaternion.Identity,
                Vector3d.Up,
                Fixed64.One,
                Fixed64.Zero,
                Vector3d.Right,
                out _,
                out _,
                out _));
    }

    [Theory]
    [InlineData(false)]
    [InlineData(true)]
    public void ConeSurfaceAnchor_SelectsTheApexPastTheSideMeridian(
        bool outsideBaseRadius)
    {
        Vector3d point = new(
            outsideBaseRadius
                ? (Fixed64)3
                : Fixed64.FromFraction(1, 10),
            outsideBaseRadius
                ? (Fixed64)10
                : (Fixed64)3,
            Fixed64.Zero);

        Assert.True(
            FixedSegment.TryGetClosestCenteredFiniteConeSurfaceAnchor(
                point,
                Vector3d.Zero,
                FixedQuaternion.Identity,
                Vector3d.Up,
                (Fixed64)4,
                (Fixed64)2,
                Vector3d.Right,
                out FixedPointAnchor anchor,
                out Vector3d normal,
                out Fixed64 signedDistance));
        Assert.True(anchor.TryGetPoint(out Vector3d surface));
        Assert.Equal(new Vector3d(0, 2, 0), surface);
        Assert.True(normal.IsNormalized());
        Assert.True(signedDistance > Fixed64.Zero);
    }

    [Fact]
    public void ConeSurfaceAnchor_RejectsUnrepresentableMeridianAndFinalDistances()
    {
        Assert.False(
            FixedSegment.TryGetClosestCenteredFiniteConeSurfaceAnchor(
                new Vector3d(
                    Fixed64.MaxValue,
                    Fixed64.MaxValue,
                    Fixed64.Zero),
                Vector3d.Zero,
                FixedQuaternion.Identity,
                Vector3d.Up,
                (Fixed64)4,
                (Fixed64)2,
                Vector3d.Right,
                out _,
                out _,
                out _));

        Assert.False(
            FixedSegment.TryGetClosestCenteredFiniteConeSurfaceAnchor(
                new Vector3d(
                    Fixed64.MaxValue,
                    Fixed64.MaxValue - Fixed64.Two,
                    Fixed64.Zero),
                Vector3d.Zero,
                FixedQuaternion.Identity,
                Vector3d.Up,
                (Fixed64)4,
                (Fixed64)2,
                Vector3d.Right,
                out _,
                out _,
                out _));
    }

    [Fact]
    public void ConeSurfaceOffset_RejectsAnUnrepresentableObliqueBaseWitness()
    {
        Fixed64 diagonal = FixedMath.Sqrt(Fixed64.Half);
        Vector3d axis = new(diagonal, diagonal, Fixed64.Zero);
        Vector3d radial = new(diagonal, -diagonal, Fixed64.Zero);
        Fixed64 axialDistance =
            Fixed64.MaxValue * -Fixed64.FromFraction(12, 25);
        Fixed64 radialDistance =
            Fixed64.MaxValue * Fixed64.FromFraction(93, 100);
        Vector3d point =
            (axis * axialDistance) + (radial * radialDistance);

        Assert.False(FixedSegment.TryGetClosestCenteredFiniteConeSurfaceOffset(
            point,
            Vector3d.Zero,
            axis,
            Fixed64.MaxValue,
            Fixed64.MaxValue,
            radial,
            out Vector3d surfaceOffset,
            out Vector3d normal,
            out Fixed64 signedDistance));
        Assert.Equal(Vector3d.Zero, surfaceOffset);
        Assert.Equal(Vector3d.Zero, normal);
        Assert.Equal(Fixed64.Zero, signedDistance);
    }

    [Fact]
    public void CenteredAxisTriangle_ProjectsAnEdgeEndpointBackOntoTheAxis()
    {
        FixedTriangle triangle = new(
            new Vector3d(0, 2, 0),
            new Vector3d(0, 3, 0),
            new Vector3d(0, 2, 1));

        Assert.True(triangle.TryGetClosestPointsToCenteredAxis(
            Vector3d.Zero,
            Vector3d.Right,
            (Fixed64)4,
            out Vector3d pointOnTriangle,
            out Vector3d pointOnAxis));
        Assert.Equal(new Vector3d(0, 2, 0), pointOnTriangle);
        Assert.Equal(Vector3d.Zero, pointOnAxis);
    }

    [Fact]
    public void CenteredAxisTriangle_ClampsTheAxisAfterClampingAnEdgeEndpoint()
    {
        FixedTriangle triangle = new(
            new Vector3d(2, 1, 0),
            new Vector3d(3, 2, 0),
            new Vector3d(2, 1, 1));

        Assert.True(triangle.TryGetClosestPointsToCenteredAxis(
            Vector3d.Zero,
            Vector3d.Right,
            Fixed64.Two,
            out Vector3d pointOnTriangle,
            out Vector3d pointOnAxis));
        Assert.Equal(new Vector3d(2, 1, 0), pointOnTriangle);
        Assert.Equal(Vector3d.Right, pointOnAxis);
    }

    [Fact]
    public void CenteredAxisTriangle_RejectsAnUnrepresentableObliqueAxisWitness()
    {
        Vector3d scalarCorner = new(
            Fixed64.MaxValue,
            Fixed64.MaxValue,
            Fixed64.Zero);
        FixedTriangle triangle = new(
            scalarCorner,
            scalarCorner,
            scalarCorner);
        Fixed64 diagonal = FixedMath.Sqrt(Fixed64.Half);

        Assert.False(triangle.TryGetClosestPointsToCenteredAxis(
            new Vector3d(
                Fixed64.MaxValue,
                Fixed64.Zero,
                Fixed64.Zero),
            new Vector3d(diagonal, diagonal, Fixed64.Zero),
            Fixed64.MaxValue,
            out Vector3d pointOnTriangle,
            out Vector3d pointOnAxis));
        Assert.Equal(Vector3d.Zero, pointOnTriangle);
        Assert.Equal(Vector3d.Zero, pointOnAxis);
    }

    [Fact]
    public void CenteredAxesDistance_HonorsRoundToEvenAtTheScalarLimit()
    {
        Assert.True(FixedSegment.TryGetDistanceBetweenCenteredAxes(
            Vector3d.Zero,
            Vector3d.Right,
            Fixed64.Zero,
            new Vector3d(
                Fixed64.MaxValue,
                Fixed64.FromRaw(BelowScalarLimitMidpointRaw),
                Fixed64.Zero),
            Vector3d.Right,
            Fixed64.Zero,
            out Fixed64 limitDistance));
        Assert.Equal(Fixed64.MaxValue, limitDistance);

        Assert.False(FixedSegment.TryGetDistanceBetweenCenteredAxes(
            Vector3d.Zero,
            Vector3d.Right,
            Fixed64.Zero,
            new Vector3d(
                Fixed64.MaxValue,
                Fixed64.FromRaw(AboveScalarLimitMidpointRaw),
                Fixed64.Zero),
            Vector3d.Right,
            Fixed64.Zero,
            out Fixed64 distance));
        Assert.Equal(Fixed64.MaxValue, distance);
    }
}
