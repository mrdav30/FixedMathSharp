//=======================================================================
// CenteredFiniteSurface.Tests.cs
//=======================================================================
// MIT License, Copyright (c) 2024–present David Oravsky (mrdav30)
// See LICENSE file in the project root for full license information.
//=======================================================================

using System;
using FixedMathSharp.Geometry;
using Xunit;

namespace FixedMathSharp.Tests;

public sealed partial class FiniteAxisIntersectionTests
{
    [Fact]
    public void CenteredCylinderSurface_PreservesScalarFaceGeometry()
    {
        Vector3d center = new(Fixed64.MinValue, Fixed64.Zero, Fixed64.Zero);

        Assert.True(FixedSegment.TryGetClosestPointOnCenteredFiniteCylinderSurface(
            new Vector3d(Fixed64.MinValue, Fixed64.Two, Fixed64.Zero),
            center,
            Vector3d.Right,
            Fixed64.Two,
            Fixed64.One,
            Vector3d.Up,
            out Vector3d surface,
            out Vector3d normal,
            out Fixed64 signedDistance));
        Assert.Equal(
            new Vector3d(Fixed64.MinValue, Fixed64.One, Fixed64.Zero),
            surface);
        Assert.Equal(Vector3d.Up, normal);
        Assert.Equal(Fixed64.One, signedDistance);
    }

    [Fact]
    public void CenteredCylinderSurfaceOffset_RemainsAuthoritativePastScalarFace()
    {
        Vector3d center = new(Fixed64.MaxValue, Fixed64.Zero, Fixed64.Zero);
        Vector3d point = center;

        Assert.False(FixedSegment.TryGetClosestPointOnCenteredFiniteCylinderSurface(
            point,
            center,
            Vector3d.Up,
            (Fixed64)10,
            Fixed64.Two,
            Vector3d.Right,
            out _,
            out _,
            out _));
        Assert.True(FixedSegment.TryGetClosestCenteredFiniteCylinderSurfaceOffset(
            point,
            center,
            Vector3d.Up,
            (Fixed64)10,
            Fixed64.Two,
            Vector3d.Right,
            out Vector3d surfaceOffset,
            out Vector3d normal,
            out Fixed64 signedDistance));
        Assert.Equal(new Vector3d(Fixed64.Two, Fixed64.Zero, Fixed64.Zero), surfaceOffset);
        Assert.Equal(Vector3d.Right, normal);
        Assert.Equal(-Fixed64.Two, signedDistance);
    }

    [Fact]
    public void CenteredCylinderSurface_ReturnsStableInsideSideCapAndRimFeatures()
    {
        Assert.True(FixedSegment.TryGetClosestPointOnCenteredFiniteCylinderSurface(
            Vector3d.Zero,
            Vector3d.Zero,
            Vector3d.Up,
            (Fixed64)4,
            Fixed64.One,
            Vector3d.Right,
            out Vector3d side,
            out Vector3d sideNormal,
            out Fixed64 sideDistance));
        Assert.Equal(Vector3d.Right, side);
        Assert.Equal(Vector3d.Right, sideNormal);
        Assert.Equal(-Fixed64.One, sideDistance);

        Assert.True(FixedSegment.TryGetClosestPointOnCenteredFiniteCylinderSurface(
            new Vector3d(
                Fixed64.Zero,
                Fixed64.FromFraction(7, 4),
                Fixed64.Zero),
            Vector3d.Zero,
            Vector3d.Up,
            (Fixed64)4,
            Fixed64.One,
            Vector3d.Right,
            out Vector3d cap,
            out Vector3d capNormal,
            out Fixed64 capDistance));
        Assert.Equal(new Vector3d(0, 2, 0), cap);
        Assert.Equal(Vector3d.Up, capNormal);
        Assert.Equal(-Fixed64.FromFraction(1, 4), capDistance);

        Assert.True(FixedSegment.TryGetClosestPointOnCenteredFiniteCylinderSurface(
            new Vector3d(2, 3, 0),
            Vector3d.Zero,
            Vector3d.Up,
            (Fixed64)4,
            Fixed64.One,
            Vector3d.Right,
            out Vector3d rim,
            out Vector3d rimNormal,
            out Fixed64 rimDistance));
        Assert.Equal(new Vector3d(1, 2, 0), rim);
        Fixed64 sqrtHalf = FixedMath.Sqrt(Fixed64.Half);
        Assert.Equal(
            new Vector3d(sqrtHalf, sqrtHalf, Fixed64.Zero),
            rimNormal);
        Assert.Equal(FixedMath.Sqrt(Fixed64.Two), rimDistance);
    }

    [Fact]
    public void CenteredCylinderContainment_ExpandsAxialAndRadialLimitsIndependently()
    {
        Assert.False(FixedSegment.ContainsPointInCenteredFiniteCylinder(
            new Vector3d(0, 3, 0),
            Vector3d.Zero,
            Vector3d.Up,
            (Fixed64)4,
            Fixed64.One));
        Assert.True(FixedSegment.ContainsPointInCenteredFiniteCylinder(
            new Vector3d(0, 3, 0),
            Vector3d.Zero,
            Vector3d.Up,
            (Fixed64)4,
            Fixed64.One,
            Fixed64.One,
            Fixed64.Zero));
        Assert.False(FixedSegment.ContainsPointInCenteredFiniteCylinder(
            new Vector3d(2, 0, 0),
            Vector3d.Zero,
            Vector3d.Up,
            (Fixed64)4,
            Fixed64.One,
            Fixed64.One,
            Fixed64.Zero));
        Assert.True(FixedSegment.ContainsPointInCenteredFiniteCylinder(
            new Vector3d(2, 0, 0),
            Vector3d.Zero,
            Vector3d.Up,
            (Fixed64)4,
            Fixed64.One,
            Fixed64.Zero,
            Fixed64.One));
    }

    [Fact]
    public void CenteredCylinderSphereOverlap_ClassifiesRoundedRimWithoutWitnesses()
    {
        Assert.True(FixedSegment.DoesCenteredFiniteCylinderOverlapSphere(
            Vector3d.Zero,
            Vector3d.Up,
            (Fixed64)4,
            Fixed64.One,
            new Vector3d(2, 3, 0),
            FixedMath.Sqrt(Fixed64.Two)));
        Assert.False(FixedSegment.DoesCenteredFiniteCylinderOverlapSphere(
            Vector3d.Zero,
            Vector3d.Up,
            (Fixed64)4,
            Fixed64.One,
            new Vector3d(2, 3, 0),
            Fixed64.One));
    }

    [Fact]
    public void CenteredConeSurface_ReturnsSideBaseAndSignedDistance()
    {
        Assert.True(FixedSegment.TryGetClosestPointOnCenteredFiniteConeSurface(
            new Vector3d(2, 0, 0),
            Vector3d.Zero,
            Vector3d.Up,
            (Fixed64)4,
            (Fixed64)2,
            Vector3d.Right,
            out Vector3d side,
            out Vector3d sideNormal,
            out Fixed64 sideDistance));
        Assert.Equal(new Vector3d(
            Fixed64.FromFraction(6, 5),
            -Fixed64.FromFraction(2, 5),
            Fixed64.Zero), side);
        Assert.Equal(new Vector3d(
            Fixed64.FromFraction(2, 1) / FixedMath.Sqrt((Fixed64)5),
            Fixed64.One / FixedMath.Sqrt((Fixed64)5),
            Fixed64.Zero), sideNormal);
        Assert.True(sideDistance > Fixed64.Zero);

        Assert.True(FixedSegment.TryGetClosestPointOnCenteredFiniteConeSurface(
            new Vector3d(1, -3, 0),
            Vector3d.Zero,
            Vector3d.Up,
            (Fixed64)4,
            (Fixed64)2,
            Vector3d.Right,
            out Vector3d basePoint,
            out Vector3d baseNormal,
            out Fixed64 baseDistance));
        Assert.Equal(new Vector3d(1, -2, 0), basePoint);
        Assert.Equal(Vector3d.Down, baseNormal);
        Assert.Equal(Fixed64.One, baseDistance);
    }

    [Fact]
    public void CenteredConeSurfaceOffset_RemainsAuthoritativePastScalarFace()
    {
        Vector3d center = new(Fixed64.MaxValue, Fixed64.Zero, Fixed64.Zero);

        Assert.False(FixedSegment.TryGetClosestPointOnCenteredFiniteConeSurface(
            center,
            center,
            Vector3d.Up,
            (Fixed64)4,
            (Fixed64)2,
            Vector3d.Right,
            out _,
            out _,
            out _));
        Assert.True(FixedSegment.TryGetClosestCenteredFiniteConeSurfaceOffset(
            center,
            center,
            Vector3d.Up,
            (Fixed64)4,
            (Fixed64)2,
            Vector3d.Right,
            out Vector3d surfaceOffset,
            out Vector3d normal,
            out Fixed64 signedDistance));
        Assert.Equal(
            new Vector3d(
                Fixed64.FromFraction(4, 5),
                Fixed64.FromFraction(2, 5),
                Fixed64.Zero),
            surfaceOffset);
        Assert.True(normal.X > Fixed64.Zero);
        Assert.True(normal.Y > Fixed64.Zero);
        Assert.True(signedDistance < Fixed64.Zero);

        Fixed64 diagonal = FixedMath.Sqrt(Fixed64.Half);
        Assert.False(FixedSegment.TryGetClosestCenteredFiniteConeSurfaceOffset(
            new Vector3d(Fixed64.MinValue, Fixed64.Zero, Fixed64.Zero),
            center,
            new Vector3d(diagonal, diagonal, Fixed64.Zero),
            Fixed64.MaxValue,
            Fixed64.MaxValue,
            new Vector3d(diagonal, -diagonal, Fixed64.Zero),
            out Vector3d unrepresentableOffset,
            out Vector3d unrepresentableNormal,
            out Fixed64 unrepresentableDistance));
        Assert.Equal(Vector3d.Zero, unrepresentableOffset);
        Assert.Equal(Vector3d.Zero, unrepresentableNormal);
        Assert.Equal(Fixed64.Zero, unrepresentableDistance);
    }

    [Fact]
    public void CenteredConeSurface_HandlesContainedAxisApexAndUnrepresentableWitnesses()
    {
        Assert.True(FixedSegment.TryGetClosestPointOnCenteredFiniteConeSurface(
            Vector3d.Zero,
            Vector3d.Zero,
            Vector3d.Up,
            (Fixed64)4,
            (Fixed64)2,
            Vector3d.Right,
            out Vector3d containedSurface,
            out Vector3d containedNormal,
            out Fixed64 containedDistance));
        Assert.Equal(
            new Vector3d(
                Fixed64.FromFraction(4, 5),
                Fixed64.FromFraction(2, 5),
                Fixed64.Zero),
            containedSurface);
        Assert.True(containedNormal.X > Fixed64.Zero);
        Assert.True(containedNormal.Y > Fixed64.Zero);
        Assert.True(containedDistance < Fixed64.Zero);

        Assert.True(FixedSegment.TryGetClosestPointOnCenteredFiniteConeSurface(
            new Vector3d(0, 2, 0),
            Vector3d.Zero,
            Vector3d.Up,
            (Fixed64)4,
            (Fixed64)2,
            Vector3d.Right,
            out Vector3d apex,
            out Vector3d apexNormal,
            out Fixed64 apexDistance));
        Assert.Equal(new Vector3d(0, 2, 0), apex);
        Assert.True(apexNormal.X > Fixed64.Zero);
        Assert.Equal(Fixed64.Zero, apexDistance);

        Assert.False(FixedSegment.TryGetClosestPointOnCenteredFiniteConeSurface(
            new Vector3d(Fixed64.MaxValue, Fixed64.Zero, Fixed64.Zero),
            new Vector3d(Fixed64.MaxValue, Fixed64.Zero, Fixed64.Zero),
            Vector3d.Up,
            Fixed64.Two,
            Fixed64.One,
            Vector3d.Right,
            out _,
            out _,
            out _));
        Assert.False(FixedSegment.TryGetClosestPointOnCenteredFiniteConeSurface(
            new Vector3d(Fixed64.MinValue, Fixed64.MaxValue, Fixed64.Zero),
            new Vector3d(Fixed64.MaxValue, Fixed64.Zero, Fixed64.Zero),
            Vector3d.Up,
            Fixed64.Two,
            Fixed64.One,
            Vector3d.Right,
            out _,
            out _,
            out _));
        Assert.False(FixedSegment.TryGetClosestPointOnCenteredFiniteConeSurface(
            new Vector3d(Fixed64.Zero, Fixed64.MinValue, Fixed64.Zero),
            new Vector3d(Fixed64.Zero, Fixed64.MaxValue, Fixed64.Zero),
            Vector3d.Up,
            Fixed64.Two,
            Fixed64.One,
            Vector3d.Right,
            out _,
            out _,
            out _));
        Assert.True(FixedSegment.TryGetClosestPointOnCenteredFiniteConeSurface(
            new Vector3d(Fixed64.Zero, Fixed64.MaxValue, Fixed64.Zero),
            Vector3d.Zero,
            Vector3d.Up,
            Fixed64.Two,
            Fixed64.One,
            Vector3d.Right,
            out Vector3d maximumApex,
            out _,
            out Fixed64 maximumApexDistance));
        Assert.Equal(Vector3d.Up, maximumApex);
        Assert.Equal(
            Fixed64.MaxValue - Fixed64.One,
            maximumApexDistance);

        Fixed64 halfMaximum = Fixed64.FromRaw(Fixed64.MaxValue.m_rawValue / 2L);
        Assert.True(FixedSegment.TryGetClosestPointOnCenteredFiniteConeSurface(
            new Vector3d(Fixed64.One, halfMaximum, Fixed64.Zero),
            Vector3d.Zero,
            Vector3d.Up,
            Fixed64.MaxValue,
            Fixed64.MaxValue,
            Vector3d.Right,
            out _,
            out _,
            out Fixed64 largeConeDistance));
        Assert.True(largeConeDistance < Fixed64.Two);

        Fixed64 threeQuarterMaximum = Fixed64.FromRaw(
            (Fixed64.MaxValue.m_rawValue / 4L) * 3L);
        Assert.False(FixedSegment.TryGetClosestPointOnCenteredFiniteConeSurface(
            new Vector3d(
                threeQuarterMaximum,
                threeQuarterMaximum,
                Fixed64.Zero),
            Vector3d.Zero,
            Vector3d.Up,
            Fixed64.Two,
            Fixed64.One,
            Vector3d.Right,
            out _,
            out _,
            out _));
    }

    [Fact]
    public void CenteredConeSphereOverlap_ClassifiesWithoutWorldWitness()
    {
        Assert.True(FixedSegment.DoesCenteredFiniteConeOverlapSphere(
            Vector3d.Zero,
            Vector3d.Up,
            (Fixed64)4,
            (Fixed64)2,
            Vector3d.Zero,
            Fixed64.Zero));
        Assert.True(FixedSegment.DoesCenteredFiniteConeOverlapSphere(
            Vector3d.Zero,
            Vector3d.Up,
            (Fixed64)4,
            (Fixed64)2,
            new Vector3d(1, -3, 0),
            Fixed64.One));
        Assert.False(FixedSegment.DoesCenteredFiniteConeOverlapSphere(
            Vector3d.Zero,
            Vector3d.Up,
            (Fixed64)4,
            (Fixed64)2,
            new Vector3d(1, -3, 0),
            Fixed64.FromRaw(Fixed64.One.m_rawValue - 1L)));
        Assert.False(FixedSegment.DoesCenteredFiniteConeOverlapSphere(
            new Vector3d(Fixed64.MaxValue, Fixed64.Zero, Fixed64.Zero),
            Vector3d.Up,
            Fixed64.Two,
            Fixed64.One,
            new Vector3d(Fixed64.MinValue, Fixed64.MaxValue, Fixed64.Zero),
            Fixed64.One));
    }

    [Fact]
    public void CenteredCylinderCapsuleAxisPenetration_KeepsRelativeProjectionExact()
    {
        Assert.True(FixedSegment.DoCenteredFiniteCylinderAndCapsuleOverlapOnAxis(
            Vector3d.Right,
            Vector3d.Zero,
            Vector3d.Up,
            Fixed64.Two,
            Fixed64.One,
            new Vector3d(
                Fixed64.FromFraction(3, 2),
                Fixed64.Zero,
                Fixed64.Zero),
            Vector3d.Up,
            Fixed64.Two,
            Fixed64.One));
        Assert.True(FixedSegment.TryGetCenteredFiniteCylinderCapsuleAxisPenetration(
            Vector3d.Right,
            Vector3d.Zero,
            Vector3d.Up,
            Fixed64.Two,
            Fixed64.One,
            new Vector3d(
                Fixed64.FromFraction(3, 2),
                Fixed64.Zero,
                Fixed64.Zero),
            Vector3d.Up,
            Fixed64.Two,
            Fixed64.One,
            out Vector3d axis,
            out Fixed64 depth));
        Assert.Equal(Vector3d.Right, axis);
        Assert.Equal(Fixed64.Half, depth);

        Assert.False(FixedSegment.DoCenteredFiniteCylinderAndCapsuleOverlapOnAxis(
            Vector3d.Right,
            Vector3d.Zero,
            Vector3d.Up,
            Fixed64.Two,
            Fixed64.One,
            new Vector3d(
                Fixed64.FromRaw((Fixed64.Two.m_rawValue) + 1L),
                Fixed64.Zero,
                Fixed64.Zero),
            Vector3d.Up,
            Fixed64.Two,
            Fixed64.One));
    }

    [Fact]
    public void CenteredCylinderAxisPenetration_CombinesBothRadicalsOnce()
    {
        Vector3d diagonal = new(
            FixedMath.Sqrt(Fixed64.Half),
            FixedMath.Sqrt(Fixed64.Half),
            Fixed64.Zero);
        Assert.True(FixedSegment.TryGetCenteredFiniteCylindersAxisPenetration(
            diagonal,
            Vector3d.Zero,
            Vector3d.Up,
            Fixed64.Two,
            Fixed64.One,
            new Vector3d(Fixed64.One, Fixed64.One, Fixed64.Zero),
            Vector3d.Right,
            Fixed64.Two,
            Fixed64.One,
            out Vector3d axis,
            out Fixed64 depth));
        Assert.Equal(diagonal, axis);
        Assert.True(depth > Fixed64.One);
    }

    [Fact]
    public void CenteredCylinderSurface_HandlesExteriorCapsAndUnrepresentableFeatures()
    {
        Assert.True(FixedSegment.TryGetClosestPointOnCenteredFiniteCylinderSurface(
            new Vector3d(0, 3, 0),
            Vector3d.Zero,
            Vector3d.Up,
            (Fixed64)4,
            Fixed64.One,
            Vector3d.Right,
            out Vector3d cap,
            out Vector3d normal,
            out Fixed64 distance));
        Assert.Equal(new Vector3d(0, 2, 0), cap);
        Assert.Equal(Vector3d.Up, normal);
        Assert.Equal(Fixed64.One, distance);

        Assert.True(FixedSegment.TryGetClosestPointOnCenteredFiniteCylinderSurface(
            new Vector3d(0, -3, 0),
            Vector3d.Zero,
            Vector3d.Up,
            (Fixed64)4,
            Fixed64.One,
            Vector3d.Right,
            out cap,
            out normal,
            out distance));
        Assert.Equal(new Vector3d(0, -2, 0), cap);
        Assert.Equal(Vector3d.Down, normal);
        Assert.Equal(Fixed64.One, distance);

        Assert.True(FixedSegment.TryGetClosestPointOnCenteredFiniteCylinderSurface(
            new Vector3d(2, 0, 0),
            Vector3d.Zero,
            Vector3d.Up,
            (Fixed64)4,
            Fixed64.One,
            Vector3d.Right,
            out Vector3d side,
            out normal,
            out distance));
        Assert.Equal(Vector3d.Right, side);
        Assert.Equal(Vector3d.Right, normal);
        Assert.Equal(Fixed64.One, distance);

        Fixed64 justOutside = Fixed64.FromRaw(Fixed64.One.m_rawValue + 1L);
        Assert.True(FixedSegment.TryGetClosestPointOnCenteredFiniteCylinderSurface(
            new Vector3d(justOutside, Fixed64.FromRaw(Fixed64.Two.m_rawValue + 1L), Fixed64.Zero),
            Vector3d.Zero,
            Vector3d.Up,
            (Fixed64)4,
            Fixed64.One,
            Vector3d.Right,
            out _,
            out normal,
            out distance));
        Fixed64 sqrtHalf = FixedMath.Sqrt(Fixed64.Half);
        Assert.Equal(new Vector3d(sqrtHalf, sqrtHalf, Fixed64.Zero), normal);
        Assert.True(distance >= Fixed64.Zero);

        Vector3d scalarFace = new(Fixed64.MaxValue, Fixed64.Zero, Fixed64.Zero);
        Assert.False(FixedSegment.TryGetClosestPointOnCenteredFiniteCylinderSurface(
            scalarFace,
            scalarFace,
            Vector3d.Up,
            Fixed64.Two,
            Fixed64.One,
            Vector3d.Right,
            out _,
            out _,
            out _));
        Assert.True(FixedSegment.TryGetClosestPointOnCenteredFiniteCylinderSurface(
            scalarFace,
            scalarFace,
            Vector3d.Right,
            Fixed64.Two,
            Fixed64.Zero,
            Vector3d.Up,
            out Vector3d degenerateSurface,
            out normal,
            out distance));
        Assert.Equal(scalarFace, degenerateSurface);
        Assert.Equal(Vector3d.Up, normal);
        Assert.Equal(Fixed64.Zero, distance);

        Vector3d verticalScalarFace =
            new(Fixed64.Zero, Fixed64.MaxValue, Fixed64.Zero);
        Assert.True(FixedSegment.TryGetClosestPointOnCenteredFiniteCylinderSurface(
            verticalScalarFace,
            verticalScalarFace,
            Vector3d.Up,
            Fixed64.Two,
            Fixed64.Zero,
            Vector3d.Right,
            out degenerateSurface,
            out normal,
            out distance));
        Assert.Equal(verticalScalarFace, degenerateSurface);
        Assert.Equal(Vector3d.Right, normal);
        Assert.Equal(Fixed64.Zero, distance);
        Assert.False(FixedSegment.TryGetClosestPointOnCenteredFiniteCylinderSurface(
            new Vector3d(Fixed64.MinValue, Fixed64.Zero, Fixed64.Zero),
            new Vector3d(Fixed64.MaxValue, Fixed64.Zero, Fixed64.Zero),
            Vector3d.Up,
            Fixed64.Two,
            Fixed64.Zero,
            Vector3d.Right,
            out _,
            out _,
            out _));
    }

    [Fact]
    public void CenteredCylinderSurface_RejectsUnrepresentableExteriorCornerOffset()
    {
        Fixed64 diagonal = FixedMath.Sqrt(Fixed64.Half);
        Vector3d axis = new(diagonal, diagonal, Fixed64.Zero);

        bool result = FixedSegment.TryGetClosestCenteredFiniteCylinderSurfaceOffset(
            new Vector3d(Fixed64.MinValue, Fixed64.Zero, Fixed64.Zero),
            new Vector3d(Fixed64.MaxValue, Fixed64.Zero, Fixed64.Zero),
            axis,
            Fixed64.MaxValue,
            Fixed64.MaxValue,
            new Vector3d(diagonal, -diagonal, Fixed64.Zero),
            out Vector3d surfaceOffset,
            out Vector3d normal,
            out Fixed64 distance);
        Assert.False(
            result,
            $"Unexpected offset {surfaceOffset}, normal {normal}, distance {distance}.");
        Assert.Equal(Vector3d.Zero, surfaceOffset);
        Assert.Equal(Vector3d.Zero, normal);
        Assert.Equal(Fixed64.Zero, distance);

        Fixed64 containedRadius =
            Fixed64.MaxValue * Fixed64.FromFraction(99, 100);
        Vector3d containedPoint = new(
            diagonal * containedRadius,
            -diagonal * containedRadius,
            Fixed64.Zero);
        bool containedResult =
            FixedSegment.TryGetClosestCenteredFiniteCylinderSurfaceOffset(
            containedPoint,
            Vector3d.Zero,
            axis,
            Fixed64.MaxValue,
            Fixed64.MaxValue,
            new Vector3d(diagonal, -diagonal, Fixed64.Zero),
            out surfaceOffset,
            out normal,
            out distance);
        Assert.True(containedResult);
        Vector3d expectedSurfaceOffset =
            new Vector3d(diagonal, -diagonal, Fixed64.Zero)
            * Fixed64.MaxValue;
        Assert.Equal(expectedSurfaceOffset, surfaceOffset);
        Assert.True(distance < Fixed64.Zero);
        Assert.Equal(
            new Vector3d(diagonal, -diagonal, Fixed64.Zero),
            normal);
    }

    [Fact]
    public void CenteredCylinderProjection_HandlesZeroRadicalsOrientationAndDepthLimits()
    {
        Assert.True(FixedSegment.TryGetCenteredFiniteCylindersAxisPenetration(
            Vector3d.Up,
            Vector3d.Zero,
            Vector3d.Up,
            Fixed64.Two,
            Fixed64.Zero,
            Vector3d.Zero,
            Vector3d.Up,
            Fixed64.Two,
            Fixed64.Zero,
            out Vector3d tieAxis,
            out Fixed64 coincidentDepth));
        Assert.Equal(Vector3d.Up, tieAxis);
        Assert.Equal(Fixed64.Two, coincidentDepth);

        Assert.True(FixedSegment.TryGetCenteredFiniteCylindersAxisPenetration(
            Vector3d.Up,
            Vector3d.Zero,
            Vector3d.Up,
            Fixed64.Two,
            Fixed64.Zero,
            -Vector3d.Up,
            Vector3d.Right,
            Fixed64.Two,
            Fixed64.One,
            out Vector3d orientedAxis,
            out Fixed64 depth));
        Assert.Equal(Vector3d.Down, orientedAxis);
        Assert.Equal(Fixed64.One, depth);

        Assert.False(FixedSegment.TryGetCenteredFiniteCylindersAxisPenetration(
            Vector3d.Right,
            Vector3d.Zero,
            Vector3d.Up,
            Fixed64.Two,
            Fixed64.One,
            new Vector3d(3, 0, 0),
            Vector3d.Up,
            Fixed64.Two,
            Fixed64.One,
            out _,
            out _));
        Assert.True(FixedSegment.TryGetCenteredFiniteCylindersAxisPenetration(
            Vector3d.Right,
            Vector3d.Zero,
            Vector3d.Up,
            Fixed64.MaxValue,
            Fixed64.MaxValue,
            Vector3d.Zero,
            Vector3d.Up,
            Fixed64.MaxValue,
            Fixed64.MaxValue,
            out Vector3d clampedAxis,
            out Fixed64 clampedDepth,
            out bool depthIsClamped));
        Assert.Equal(Vector3d.Right, clampedAxis);
        Assert.Equal(Fixed64.MaxValue, clampedDepth);
        Assert.True(depthIsClamped);

        Assert.True(FixedSegment.TryGetCenteredFiniteCylinderCapsuleAxisPenetration(
            Vector3d.Right,
            Vector3d.Zero,
            Vector3d.Up,
            Fixed64.MaxValue,
            Fixed64.MaxValue,
            Vector3d.Zero,
            Vector3d.Up,
            Fixed64.MaxValue,
            Fixed64.MaxValue,
            out clampedAxis,
            out clampedDepth,
            out depthIsClamped));
        Assert.Equal(Vector3d.Right, clampedAxis);
        Assert.Equal(Fixed64.MaxValue, clampedDepth);
        Assert.True(depthIsClamped);
        Assert.False(FixedSegment.TryGetCenteredFiniteCylinderCapsuleAxisPenetration(
            Vector3d.Right,
            Vector3d.Zero,
            Vector3d.Up,
            Fixed64.MaxValue,
            Fixed64.MaxValue,
            Vector3d.Zero,
            Vector3d.Up,
            Fixed64.MaxValue,
            Fixed64.MaxValue,
            out _,
            out _));

        Assert.True(FixedSegment.TryGetCenteredFiniteCylindersAxisPenetration(
            Vector3d.Right,
            Vector3d.Zero,
            Vector3d.Up,
            Fixed64.Two,
            Fixed64.One,
            new Vector3d(2, 0, 0),
            Vector3d.Up,
            Fixed64.Two,
            Fixed64.One,
            out _,
            out Fixed64 tangentDepth));
        Assert.Equal(Fixed64.Zero, tangentDepth);

        Assert.True(FixedSegment.DoCenteredFiniteCylindersOverlapOnAxis(
            Vector3d.Right,
            Vector3d.Zero,
            Vector3d.Up,
            Fixed64.MaxValue,
            Fixed64.MaxValue,
            Vector3d.Zero,
            Vector3d.Up,
            Fixed64.MaxValue,
            Fixed64.MaxValue));
        Assert.False(FixedSegment.TryGetCenteredFiniteCylindersAxisPenetration(
            Vector3d.Right,
            Vector3d.Zero,
            Vector3d.Up,
            Fixed64.MaxValue,
            Fixed64.MaxValue,
            Vector3d.Zero,
            Vector3d.Up,
            Fixed64.MaxValue,
            Fixed64.MaxValue,
            out _,
            out _));

        Assert.True(FixedSegment.TryGetCenteredFiniteCylindersAxisPenetration(
            Vector3d.Right,
            Vector3d.Zero,
            Vector3d.Up,
            Fixed64.One,
            Fixed64.MaxValue,
            Vector3d.Zero,
            Vector3d.Up,
            Fixed64.One,
            Fixed64.Zero,
            out _,
            out Fixed64 maximumDepth));
        Assert.Equal(Fixed64.MaxValue, maximumDepth);

        Vector3d slightlyLongAxis = new(
            Fixed64.FromRaw(Fixed64.One.m_rawValue + 1L),
            Fixed64.Zero,
            Fixed64.Zero);
        Assert.True(slightlyLongAxis.IsNormalized());
        Assert.True(FixedSegment.TryGetCenteredFiniteCylindersAxisPenetration(
            slightlyLongAxis,
            Vector3d.Zero,
            Vector3d.Up,
            Fixed64.One,
            Fixed64.MaxValue,
            new Vector3d(Fixed64.MaxValue, Fixed64.Zero, Fixed64.Zero),
            Vector3d.Right,
            (Fixed64)4,
            Fixed64.Zero,
            out _,
            out Fixed64 boundaryDepth));
        Assert.True(boundaryDepth > Fixed64.Zero);
    }

    [Fact]
    public void CenteredCylinderProjection_IsSymmetricAcrossExactRationalAxes()
    {
        Vector3d[] axes =
        {
            Vector3d.Right,
            Vector3d.Up,
            new(
                Fixed64.FromFraction(3, 5),
                Fixed64.FromFraction(4, 5),
                Fixed64.Zero)
        };
        Fixed64[] separations =
        {
            -Fixed64.FromFraction(7, 4),
            Fixed64.Zero,
            Fixed64.FromFraction(3, 4),
            Fixed64.FromFraction(9, 4)
        };

        foreach (Vector3d projectionAxis in axes)
        {
            foreach (Vector3d firstAxis in axes)
            {
                foreach (Vector3d secondAxis in axes)
                {
                    foreach (Fixed64 separation in separations)
                    {
                        Vector3d secondCenter = projectionAxis * separation;
                        bool forward = FixedSegment.DoCenteredFiniteCylindersOverlapOnAxis(
                            projectionAxis,
                            Vector3d.Zero,
                            firstAxis,
                            Fixed64.FromFraction(3, 2),
                            Fixed64.FromFraction(2, 3),
                            secondCenter,
                            secondAxis,
                            Fixed64.FromFraction(5, 2),
                            Fixed64.FromFraction(4, 5));
                        bool reverse = FixedSegment.DoCenteredFiniteCylindersOverlapOnAxis(
                            projectionAxis,
                            secondCenter,
                            secondAxis,
                            Fixed64.FromFraction(5, 2),
                            Fixed64.FromFraction(4, 5),
                            Vector3d.Zero,
                            firstAxis,
                            Fixed64.FromFraction(3, 2),
                            Fixed64.FromFraction(2, 3));
                        Assert.Equal(forward, reverse);

                        bool materialized =
                            FixedSegment.TryGetCenteredFiniteCylindersAxisPenetration(
                                projectionAxis,
                                Vector3d.Zero,
                                firstAxis,
                                Fixed64.FromFraction(3, 2),
                                Fixed64.FromFraction(2, 3),
                                secondCenter,
                                secondAxis,
                                Fixed64.FromFraction(5, 2),
                                Fixed64.FromFraction(4, 5),
                                out _,
                                out Fixed64 depth);
                        Assert.Equal(forward, materialized);
                        if (materialized)
                            Assert.True(depth >= Fixed64.Zero);
                    }
                }
            }
        }
    }

    [Fact]
    public void CenteredSurfaceRelations_RejectInvalidAuthoredGeometry()
    {
        Assert.Throws<ArgumentException>(() =>
            FixedSegment.DoesCenteredFiniteCylinderOverlapSphere(
                Vector3d.Zero,
                Vector3d.Zero,
                Fixed64.One,
                Fixed64.One,
                Vector3d.Zero,
                Fixed64.One));
        Assert.Throws<ArgumentOutOfRangeException>(() =>
            FixedSegment.DoesCenteredFiniteCylinderOverlapSphere(
                Vector3d.Zero,
                Vector3d.Up,
                Fixed64.Zero,
                Fixed64.One,
                Vector3d.Zero,
                Fixed64.One));
        Assert.Throws<ArgumentOutOfRangeException>(() =>
            FixedSegment.DoesCenteredFiniteCylinderOverlapSphere(
                Vector3d.Zero,
                Vector3d.Up,
                Fixed64.One,
                -Fixed64.One,
                Vector3d.Zero,
                Fixed64.One));
        Assert.Throws<ArgumentOutOfRangeException>(() =>
            FixedSegment.DoesCenteredFiniteCylinderOverlapSphere(
                Vector3d.Zero,
                Vector3d.Up,
                Fixed64.One,
                Fixed64.One,
                Vector3d.Zero,
                -Fixed64.One));

        Assert.Throws<ArgumentException>(() =>
            FixedSegment.DoesCenteredFiniteConeOverlapSphere(
                Vector3d.Zero,
                Vector3d.Zero,
                Fixed64.One,
                Fixed64.One,
                Vector3d.Zero,
                Fixed64.One));
        Assert.Throws<ArgumentOutOfRangeException>(() =>
            FixedSegment.DoesCenteredFiniteConeOverlapSphere(
                Vector3d.Zero,
                Vector3d.Up,
                Fixed64.Zero,
                Fixed64.One,
                Vector3d.Zero,
                Fixed64.One));
        Assert.Throws<ArgumentOutOfRangeException>(() =>
            FixedSegment.DoesCenteredFiniteConeOverlapSphere(
                Vector3d.Zero,
                Vector3d.Up,
                Fixed64.One,
                -Fixed64.One,
                Vector3d.Zero,
                Fixed64.One));
        Assert.Throws<ArgumentOutOfRangeException>(() =>
            FixedSegment.DoesCenteredFiniteConeOverlapSphere(
                Vector3d.Zero,
                Vector3d.Up,
                Fixed64.One,
                Fixed64.One,
                Vector3d.Zero,
                -Fixed64.One));

        Assert.Throws<ArgumentException>(() =>
            FixedSegment.ContainsPointInCenteredFiniteCylinder(
                Vector3d.Zero,
                Vector3d.Zero,
                Vector3d.Zero,
                Fixed64.One,
                Fixed64.One,
                Fixed64.Zero,
                Fixed64.Zero));
        Assert.Throws<ArgumentOutOfRangeException>(() =>
            FixedSegment.ContainsPointInCenteredFiniteCylinder(
                Vector3d.Zero,
                Vector3d.Zero,
                Vector3d.Up,
                Fixed64.Zero,
                Fixed64.One,
                Fixed64.Zero,
                Fixed64.Zero));
        Assert.Throws<ArgumentOutOfRangeException>(() =>
            FixedSegment.ContainsPointInCenteredFiniteCylinder(
                Vector3d.Zero,
                Vector3d.Zero,
                Vector3d.Up,
                Fixed64.One,
                -Fixed64.One,
                Fixed64.Zero,
                Fixed64.Zero));
        Assert.Throws<ArgumentOutOfRangeException>(() =>
            FixedSegment.ContainsPointInCenteredFiniteCylinder(
                Vector3d.Zero,
                Vector3d.Zero,
                Vector3d.Up,
                Fixed64.One,
                Fixed64.One,
                -Fixed64.One,
                Fixed64.Zero));
        Assert.Throws<ArgumentOutOfRangeException>(() =>
            FixedSegment.ContainsPointInCenteredFiniteCylinder(
                Vector3d.Zero,
                Vector3d.Zero,
                Vector3d.Up,
                Fixed64.One,
                Fixed64.One,
                Fixed64.Zero,
                -Fixed64.One));
    }

    [Fact]
    public void CenteredProjectionAndWitnessRelations_RejectInvalidAxesAndDimensions()
    {
        static bool CylinderCapsule(
            Vector3d projectionAxis,
            Vector3d cylinderAxis,
            Fixed64 cylinderLength,
            Fixed64 cylinderRadius,
            Vector3d capsuleAxis,
            Fixed64 capsuleLength,
            Fixed64 capsuleRadius) =>
            FixedSegment.DoCenteredFiniteCylinderAndCapsuleOverlapOnAxis(
                projectionAxis,
                Vector3d.Zero,
                cylinderAxis,
                cylinderLength,
                cylinderRadius,
                Vector3d.Zero,
                capsuleAxis,
                capsuleLength,
                capsuleRadius);

        Assert.Throws<ArgumentException>(() =>
            CylinderCapsule(Vector3d.Zero, Vector3d.Up, Fixed64.One, Fixed64.One, Vector3d.Up, Fixed64.One, Fixed64.One));
        Assert.Throws<ArgumentException>(() =>
            CylinderCapsule(Vector3d.Right, Vector3d.Zero, Fixed64.One, Fixed64.One, Vector3d.Up, Fixed64.One, Fixed64.One));
        Assert.Throws<ArgumentOutOfRangeException>(() =>
            CylinderCapsule(Vector3d.Right, Vector3d.Up, Fixed64.Zero, Fixed64.One, Vector3d.Up, Fixed64.One, Fixed64.One));
        Assert.Throws<ArgumentOutOfRangeException>(() =>
            CylinderCapsule(Vector3d.Right, Vector3d.Up, Fixed64.One, -Fixed64.One, Vector3d.Up, Fixed64.One, Fixed64.One));
        Assert.Throws<ArgumentException>(() =>
            CylinderCapsule(Vector3d.Right, Vector3d.Up, Fixed64.One, Fixed64.One, Vector3d.Zero, Fixed64.One, Fixed64.One));
        Assert.Throws<ArgumentOutOfRangeException>(() =>
            CylinderCapsule(Vector3d.Right, Vector3d.Up, Fixed64.One, Fixed64.One, Vector3d.Up, -Fixed64.One, Fixed64.One));
        Assert.Throws<ArgumentOutOfRangeException>(() =>
            CylinderCapsule(Vector3d.Right, Vector3d.Up, Fixed64.One, Fixed64.One, Vector3d.Up, Fixed64.One, -Fixed64.One));

        static bool CylinderSurface(
            Vector3d axis,
            Fixed64 length,
            Fixed64 radius,
            Vector3d fallback) =>
            FixedSegment.TryGetClosestPointOnCenteredFiniteCylinderSurface(
                Vector3d.Zero,
                Vector3d.Zero,
                axis,
                length,
                radius,
                fallback,
                out _,
                out _,
                out _);

        Assert.Throws<ArgumentException>(() =>
            CylinderSurface(Vector3d.Zero, Fixed64.One, Fixed64.One, Vector3d.Right));
        Assert.Throws<ArgumentOutOfRangeException>(() =>
            CylinderSurface(Vector3d.Up, Fixed64.Zero, Fixed64.One, Vector3d.Right));
        Assert.Throws<ArgumentOutOfRangeException>(() =>
            CylinderSurface(Vector3d.Up, Fixed64.One, -Fixed64.One, Vector3d.Right));
        Assert.Throws<ArgumentException>(() =>
            CylinderSurface(Vector3d.Up, Fixed64.One, Fixed64.One, Vector3d.Zero));
        Assert.Throws<ArgumentException>(() =>
            CylinderSurface(Vector3d.Up, Fixed64.One, Fixed64.One, Vector3d.Up));
    }
}
