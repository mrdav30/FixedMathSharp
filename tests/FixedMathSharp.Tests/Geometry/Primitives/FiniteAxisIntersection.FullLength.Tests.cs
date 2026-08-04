//=======================================================================
// FiniteAxisIntersection.FullLength.Tests.cs
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
    [Theory]
    [InlineData(false)]
    [InlineData(true)]
    public void CenteredFiniteAxes_TreatOddRawInputAsFullLengthAtMirroredScalarFaces(bool maximumFace)
    {
        Fixed64 centerX = maximumFace ? Fixed64.MaxValue : Fixed64.MinValue;
        Fixed64 inwardOffset = Fixed64.FromRaw(maximumFace ? -2L : 2L);
        Vector3d center = new(centerX, Fixed64.Zero, Fixed64.Zero);
        Vector3d point = new(centerX + inwardOffset, Fixed64.Zero, Fixed64.Zero);
        Fixed64 axisLength = Fixed64.FromRaw(3L);

        Assert.False(FixedSegment.ContainsPointInCenteredCapsule(
            point, center, Vector3d.Right, axisLength, Fixed64.Zero));
        Assert.False(FixedSegment.ContainsPointInCenteredFiniteCylinder(
            point, center, Vector3d.Right, axisLength, Fixed64.Zero));
    }

    [Fact]
    public void CenteredCapsule_OddRawFullLengthPreservesDistanceAndSurfaceWitness()
    {
        Fixed64 axisLength = Fixed64.FromRaw(3L);
        Vector3d point = new(Fixed64.FromRaw(3L), Fixed64.Zero, Fixed64.Zero);

        Assert.Equal(
            Fixed64.FromRaw(2L),
            FixedSegment.GetDistanceToCenteredCapsule(
                point, Vector3d.Zero, Vector3d.Right, axisLength, Fixed64.Zero));
        Assert.True(FixedSegment.TryGetSurfacePointOnCenteredCapsule(
            point,
            Vector3d.Zero,
            Vector3d.Right,
            axisLength,
            Fixed64.Zero,
            Vector3d.Right,
            out Vector3d surface));
        Assert.Equal(new Vector3d(Fixed64.FromRaw(2L), Fixed64.Zero, Fixed64.Zero), surface);
    }

    [Fact]
    public void CenteredFiniteAxisQueries_PreserveOddRawFullLengthParity()
    {
        Fixed64 axisLength = Fixed64.FromRaw(3L);
        Vector3d point = new(Fixed64.FromRaw(2L), Fixed64.Zero, Fixed64.Zero);
        FixedSegment pointSegment = new(point, point);
        FixedRay pointRay = new(point, Vector3d.Zero);

        Assert.False(pointSegment.TryGetCapsuleIntersectionInterval(
            Vector3d.Zero,
            Vector3d.Right,
            axisLength,
            Fixed64.Zero,
            out _,
            out _));
        Assert.False(pointRay.TryGetFiniteCylinderIntersectionInterval(
            Vector3d.Zero,
            Vector3d.Right,
            axisLength,
            Fixed64.Zero,
            Fixed64.Zero,
            Fixed64.Zero,
            Fixed64.Zero,
            out _,
            out _));
        Assert.False(pointSegment.TryGetSweptSphereFiniteCylinderIntersectionDistance(
            Vector3d.Zero,
            Vector3d.Right,
            axisLength,
            Fixed64.Zero,
            Fixed64.Zero,
            Fixed64.Zero,
            out _));
    }

    [Fact]
    public void SlabSupport_PreservesOddRawFullLengthParity()
    {
        Fixed64 axisLength = Fixed64.FromRaw(3L);
        FixedRange slab = new((Fixed64)(-1), Fixed64.One);

        Assert.True(FixedSlabProjection.TryGetCapsuleSupport(
            Vector3d.Zero,
            Vector3d.Right,
            axisLength,
            Fixed64.Zero,
            slab,
            Vector2d.Right,
            out Vector2d capsuleSupport));
        Assert.Equal(new Vector2d(Fixed64.FromRaw(2L), Fixed64.Zero), capsuleSupport);

        Assert.True(FixedSlabProjection.TryGetCylinderSupport(
            Vector3d.Zero,
            Vector3d.Right,
            axisLength,
            Fixed64.Zero,
            slab,
            Vector2d.Right,
            out Vector2d cylinderSupport));
        Assert.Equal(capsuleSupport, cylinderSupport);
    }

    [Theory]
    [InlineData(false)]
    [InlineData(true)]
    public void SlabSupport_RejectsUnrepresentableOddLengthWinnerAtMirroredScalarFaces(bool maximumFace)
    {
        Fixed64 face = maximumFace ? Fixed64.MaxValue : Fixed64.MinValue;
        Vector3d center = new(face, Fixed64.Zero, Fixed64.Zero);
        Vector2d direction = maximumFace ? Vector2d.Right : Vector2d.Left;

        Assert.False(FixedSlabProjection.TryGetCapsuleSupport(
            center,
            Vector3d.Right,
            Fixed64.FromRaw(3L),
            Fixed64.Zero,
            new FixedRange(-Fixed64.One, Fixed64.One),
            direction,
            out Vector2d capsuleSupport));
        Assert.Equal(Vector2d.Zero, capsuleSupport);

        Assert.False(FixedSlabProjection.TryGetCylinderSupport(
            center,
            Vector3d.Right,
            Fixed64.FromRaw(3L),
            Fixed64.Zero,
            new FixedRange(-Fixed64.One, Fixed64.One),
            direction,
            out Vector2d cylinderSupport));
        Assert.Equal(Vector2d.Zero, cylinderSupport);
    }

    [Theory]
    [InlineData(false)]
    [InlineData(true)]
    public void CenteredAxes_ClosestPointsPreserveOddLengthsAtMirroredScalarFaces(bool maximumFace)
    {
        Fixed64 face = maximumFace ? Fixed64.MaxValue : Fixed64.MinValue;
        Fixed64 inward = Fixed64.FromRaw(maximumFace ? -1L : 1L);
        Vector3d expected = new(face + inward, Fixed64.Zero, Fixed64.Zero);

        Assert.True(FixedSegment.TryGetClosestPointsBetweenCenteredAxes(
            new Vector3d(face, Fixed64.Zero, Fixed64.Zero),
            Vector3d.Right,
            Fixed64.FromRaw(3L),
            new Vector3d(face + inward, Fixed64.Zero, Fixed64.Zero),
            Vector3d.Up,
            Fixed64.FromRaw(3L),
            out Vector3d first,
            out Vector3d second));
        Assert.Equal(expected, first);
        Assert.Equal(expected, second);
    }

    [Fact]
    public void CenteredAxes2d_ClosestPointsPreserveOddLengths()
    {
        Vector2d expected = new(Fixed64.FromRaw(1L), Fixed64.Zero);

        Assert.True(FixedSegment2d.TryGetClosestPointsBetweenCenteredAxes(
            Vector2d.Zero,
            Vector2d.Right,
            Fixed64.FromRaw(3L),
            new Vector2d(Fixed64.FromRaw(1L), Fixed64.Zero),
            new Vector2d(Fixed64.Zero, Fixed64.One),
            Fixed64.FromRaw(3L),
            out Vector2d first,
            out Vector2d second));
        Assert.Equal(expected, first);
        Assert.Equal(expected, second);
    }

    [Fact]
    public void CenteredAxes2d_ClosestPointsRejectUnrepresentableFinalWitness()
    {
        Vector2d maximumCenter = new(Fixed64.MaxValue, Fixed64.Zero);

        Assert.False(FixedSegment2d.TryGetClosestPointsBetweenCenteredAxes(
            maximumCenter,
            Vector2d.Left,
            Fixed64.Two,
            maximumCenter + Vector2d.Forward,
            Vector2d.Left,
            Fixed64.Two,
            out Vector2d first,
            out Vector2d second));
        Assert.Equal(Vector2d.Zero, first);
        Assert.Equal(Vector2d.Zero, second);
    }

    [Fact]
    public void CenteredAxes_ClosestPointsCoverInteriorAndClampedFeatures()
    {
        Assert.True(FixedSegment.TryGetClosestPointsBetweenCenteredAxes(
            Vector3d.Zero,
            Vector3d.Right,
            (Fixed64)4,
            new Vector3d(Fixed64.One, Fixed64.One, Fixed64.One),
            Vector3d.Up,
            (Fixed64)4,
            out Vector3d interiorFirst,
            out Vector3d interiorSecond));
        Assert.Equal(new Vector3d(Fixed64.One, Fixed64.Zero, Fixed64.Zero), interiorFirst);
        Assert.Equal(new Vector3d(Fixed64.One, Fixed64.Zero, Fixed64.One), interiorSecond);

        Assert.True(FixedSegment.TryGetClosestPointsBetweenCenteredAxes(
            Vector3d.Zero,
            Vector3d.Right,
            Fixed64.Two,
            new Vector3d((Fixed64)3, (Fixed64)3, Fixed64.Zero),
            Vector3d.Up,
            Fixed64.Two,
            out Vector3d clampedFirst,
            out Vector3d clampedSecond));
        Assert.Equal(Vector3d.Right, clampedFirst);
        Assert.Equal(new Vector3d((Fixed64)3, (Fixed64)2, Fixed64.Zero), clampedSecond);
    }

    [Fact]
    public void CenteredAxes_ClosestPointsKeepParallelTieWideUntilFinalWitness()
    {
        Assert.True(FixedSegment.TryGetClosestPointsBetweenCenteredAxes(
            Vector3d.Zero,
            Vector3d.Right,
            Fixed64.Two,
            Vector3d.Up,
            Vector3d.Right,
            Fixed64.Two,
            out Vector3d first,
            out Vector3d second));
        Assert.Equal(Vector3d.Left, first);
        Assert.Equal(new Vector3d(-Fixed64.One, Fixed64.One, Fixed64.Zero), second);

        Vector3d maximumCenter = new(Fixed64.MaxValue, Fixed64.Zero, Fixed64.Zero);
        Assert.False(FixedSegment.TryGetClosestPointsBetweenCenteredAxes(
            maximumCenter,
            Vector3d.Left,
            Fixed64.Two,
            maximumCenter + Vector3d.Up,
            Vector3d.Left,
            Fixed64.Two,
            out first,
            out second));
        Assert.Equal(Vector3d.Zero, first);
        Assert.Equal(Vector3d.Zero, second);
    }

    [Fact]
    public void CenteredAxes_ClosestOffsetsRemainAvailablePastScalarFace()
    {
        Vector3d maximumCenter = new(Fixed64.MaxValue, Fixed64.Zero, Fixed64.Zero);
        Vector3d secondCenter = maximumCenter + Vector3d.Up;

        Assert.False(FixedSegment.TryGetClosestPointsBetweenCenteredAxes(
            maximumCenter,
            Vector3d.Left,
            Fixed64.Two,
            secondCenter,
            Vector3d.Left,
            Fixed64.Two,
            out _,
            out _));
        Assert.True(FixedSegment.TryGetClosestOffsetsBetweenCenteredAxes(
            maximumCenter,
            Vector3d.Left,
            Fixed64.Two,
            secondCenter,
            Vector3d.Left,
            Fixed64.Two,
            out Vector3d firstOffset,
            out Vector3d secondOffset));
        Assert.Equal(Vector3d.Right, firstOffset);
        Assert.Equal(Vector3d.Right, secondOffset);
        Assert.Equal(
            Vector3d.Up,
            FixedSegment.GetClosestDirectionBetweenCenteredAxes(
                maximumCenter,
                Vector3d.Left,
                Fixed64.Two,
                secondCenter,
                Vector3d.Left,
                Fixed64.Two));
    }

    [Fact]
    public void CenteredAxes_ClosestPointsValidateBothAuthoredAxes()
    {
        ArgumentException firstDirection = Assert.Throws<ArgumentException>(() =>
            FixedSegment.TryGetClosestPointsBetweenCenteredAxes(
                Vector3d.Zero, Vector3d.Zero, Fixed64.One,
                Vector3d.Zero, Vector3d.Up, Fixed64.One,
                out _, out _));
        Assert.Equal("firstAxisDirection", firstDirection.ParamName);

        ArgumentOutOfRangeException secondLength = Assert.Throws<ArgumentOutOfRangeException>(() =>
            FixedSegment.TryGetClosestPointsBetweenCenteredAxes(
                Vector3d.Zero, Vector3d.Up, Fixed64.One,
                Vector3d.Zero, Vector3d.Right, -Fixed64.One,
                out _, out _));
        Assert.Equal("secondAxisLength", secondLength.ParamName);
    }

    [Theory]
    [InlineData(false)]
    [InlineData(true)]
    public void CenteredFiniteAxisBounds_ClipOddLengthsAtMirroredScalarFaces(bool maximumFace)
    {
        Fixed64 face = maximumFace ? Fixed64.MaxValue : Fixed64.MinValue;
        Fixed64 inwardTwo = Fixed64.FromRaw(maximumFace ? -2L : 2L);
        Vector3d center = new(face, Fixed64.Zero, Fixed64.Zero);
        Vector3d expectedMin = maximumFace
            ? new Vector3d(face + inwardTwo, Fixed64.Zero, Fixed64.Zero)
            : center;
        Vector3d expectedMax = maximumFace
            ? center
            : new Vector3d(face + inwardTwo, Fixed64.Zero, Fixed64.Zero);

        FixedBoundBox capsule = FixedBoundBox.FromCenteredCapsuleClippedToDomain(
            center,
            Vector3d.Right,
            Fixed64.FromRaw(3L),
            Fixed64.Zero);
        FixedBoundBox cylinder = FixedBoundBox.FromCenteredFiniteCylinderClippedToDomain(
            center,
            Vector3d.Right,
            Fixed64.FromRaw(3L),
            Fixed64.Zero);

        Assert.Equal(expectedMin, capsule.Min);
        Assert.Equal(expectedMax, capsule.Max);
        Assert.Equal(capsule, cylinder);
    }

    [Fact]
    public void CenteredFiniteAxisBounds_AreTightForOrdinaryCardinalShapes()
    {
        Vector3d center = new(Fixed64.One, Fixed64.Two, (Fixed64)3);

        Assert.Equal(
            FixedBoundBox.FromMinMax(
                new Vector3d((Fixed64)(-2), Fixed64.One, Fixed64.Two),
                new Vector3d((Fixed64)4, (Fixed64)3, (Fixed64)4)),
            FixedBoundBox.FromCenteredCapsuleClippedToDomain(
                center, Vector3d.Right, (Fixed64)4, Fixed64.One));
        Assert.Equal(
            FixedBoundBox.FromMinMax(
                new Vector3d(-Fixed64.One, Fixed64.One, Fixed64.Two),
                new Vector3d((Fixed64)3, (Fixed64)3, (Fixed64)4)),
            FixedBoundBox.FromCenteredFiniteCylinderClippedToDomain(
                center, Vector3d.Right, (Fixed64)4, Fixed64.One));
    }

    [Fact]
    public void CenteredFiniteCylinderBounds_CombineRotatedOddRawExtentsBeforeRounding()
    {
        Fixed64 diagonal = FixedMath.Sqrt(Fixed64.Half);

        FixedBoundBox bounds = FixedBoundBox.FromCenteredFiniteCylinderClippedToDomain(
            Vector3d.Zero,
            new Vector3d(diagonal, diagonal, Fixed64.Zero),
            Fixed64.FromRaw(1L),
            Fixed64.FromRaw(2L));

        Assert.Equal(
            FixedBoundBox.FromMinMax(
                new Vector3d(Fixed64.FromRaw(-2L), Fixed64.FromRaw(-2L), Fixed64.FromRaw(-2L)),
                new Vector3d(Fixed64.FromRaw(2L), Fixed64.FromRaw(2L), Fixed64.FromRaw(2L))),
            bounds);
    }

    [Fact]
    public void CenteredFiniteCylinderBounds_RoundCombinedExtentOutwardWhenFractionsCarry()
    {
        Fixed64 diagonal = FixedMath.Sqrt(Fixed64.Half);

        FixedBoundBox bounds = FixedBoundBox.FromCenteredFiniteCylinderClippedToDomain(
            Vector3d.Zero,
            new Vector3d(diagonal, diagonal, Fixed64.Zero),
            Fixed64.FromRaw(2L),
            Fixed64.FromRaw(2L));

        Assert.Equal(
            FixedBoundBox.FromMinMax(
                new Vector3d(Fixed64.FromRaw(-3L), Fixed64.FromRaw(-3L), Fixed64.FromRaw(-2L)),
                new Vector3d(Fixed64.FromRaw(3L), Fixed64.FromRaw(3L), Fixed64.FromRaw(2L))),
            bounds);
    }

    [Fact]
    public void CenteredCapsuleBounds_RoundOddRawHalfExtentOutwardOnBothFaces()
    {
        FixedBoundBox bounds = FixedBoundBox.FromCenteredCapsuleClippedToDomain(
            Vector3d.Zero,
            Vector3d.Right,
            Fixed64.FromRaw(1L),
            Fixed64.Zero);

        Assert.Equal(
            FixedBoundBox.FromMinMax(
                new Vector3d(Fixed64.FromRaw(-1L), Fixed64.Zero, Fixed64.Zero),
                new Vector3d(Fixed64.FromRaw(1L), Fixed64.Zero, Fixed64.Zero)),
            bounds);
    }

    [Theory]
    [InlineData(0, false)]
    [InlineData(0, true)]
    [InlineData(1, false)]
    [InlineData(1, true)]
    [InlineData(2, false)]
    [InlineData(2, true)]
    public void CenteredCapsuleBounds_ClipHalfRawEndpointBeyondScalarDomain(
        int axisIndex,
        bool maximumFace)
    {
        Fixed64 centerCoordinate = maximumFace ? Fixed64.MaxValue : Fixed64.MinValue;
        Fixed64 innerCoordinate = Fixed64.FromRaw(
            centerCoordinate.m_rawValue + (maximumFace ? -1L : 1L));
        Vector3d axis = CreateAxisVector(axisIndex, Fixed64.One);
        Vector3d center = CreateAxisVector(axisIndex, centerCoordinate);

        FixedBoundBox bounds = FixedBoundBox.FromCenteredCapsuleClippedToDomain(
            center,
            axis,
            Fixed64.FromRaw(1L),
            Fixed64.Zero);

        Assert.Equal(
            FixedBoundBox.FromMinMax(
                CreateAxisVector(
                    axisIndex,
                    maximumFace ? innerCoordinate : Fixed64.MinValue),
                CreateAxisVector(
                    axisIndex,
                    maximumFace ? Fixed64.MaxValue : innerCoordinate)),
            bounds);
    }

    [Fact]
    public void CenteredCapsuleBounds_ClipRoundedObliqueEndpointBeyondMaximum()
    {
        Vector3d axis = new(
            Fixed64.FromFraction(3, 5),
            Fixed64.FromFraction(4, 5),
            Fixed64.Zero);

        FixedBoundBox bounds = FixedBoundBox.FromCenteredCapsuleClippedToDomain(
            new Vector3d(Fixed64.MaxValue, Fixed64.Zero, Fixed64.Zero),
            axis,
            Fixed64.FromRaw(1L),
            Fixed64.Zero);

        Assert.Equal(
            FixedBoundBox.FromMinMax(
                new Vector3d(
                    Fixed64.FromRaw(Fixed64.MaxValue.m_rawValue - 1L),
                    Fixed64.FromRaw(-1L),
                    Fixed64.Zero),
                new Vector3d(
                    Fixed64.MaxValue,
                    Fixed64.FromRaw(1L),
                    Fixed64.Zero)),
            bounds);
    }

    [Fact]
    public void CenteredCapsule_ExtremeObliqueCrossingRetainsWideRadialRoots()
    {
        Vector2d axis = new(Fixed64.FromFraction(3, 5), Fixed64.FromFraction(4, 5));
        Vector2d start = new(
            -(Fixed64.MaxValue * axis.Y),
            Fixed64.MaxValue * axis.X);
        var query = new FixedSegment2d(start, -start);

        Assert.True(query.TryGetCapsuleIntersectionInterval(
            Vector2d.Zero,
            axis,
            Fixed64.Two,
            Fixed64.One,
            out Fixed64 entry,
            out Fixed64 exit));
        Assert.True(entry < Fixed64.Half);
        Assert.True(exit > Fixed64.Half);
        Assert.False(FixedSegment2d.ContainsPointInCenteredCapsule(
            query.Start, Vector2d.Zero, axis, Fixed64.Two, Fixed64.One));
        Assert.True(FixedSegment2d.ContainsPointInCenteredCapsule(
            Vector2d.Zero, Vector2d.Zero, axis, Fixed64.Two, Fixed64.One));
        Assert.False(FixedSegment2d.ContainsPointInCenteredCapsule(
            query.End, Vector2d.Zero, axis, Fixed64.Two, Fixed64.One));
    }

    [Fact]
    public void CenteredCapsule_ExtremeObliqueOneSidedCrossingsRetainContainedBoundary()
    {
        Vector2d axis = new(Fixed64.FromFraction(3, 5), Fixed64.FromFraction(4, 5));
        Vector2d outside = new(
            -(Fixed64.MaxValue * axis.Y),
            Fixed64.MaxValue * axis.X);
        var exiting = new FixedSegment2d(Vector2d.Zero, outside);
        var entering = new FixedSegment2d(outside, Vector2d.Zero);

        Assert.True(exiting.TryGetCapsuleIntersectionInterval(
            Vector2d.Zero,
            axis,
            Fixed64.Two,
            Fixed64.One,
            out Fixed64 exitingEntry,
            out Fixed64 exitingExit));
        Assert.Equal(Fixed64.Zero, exitingEntry);
        Assert.True(exitingExit < Fixed64.One);

        Assert.True(entering.TryGetCapsuleIntersectionInterval(
            Vector2d.Zero,
            axis,
            Fixed64.Two,
            Fixed64.One,
            out Fixed64 enteringEntry,
            out Fixed64 enteringExit));
        Assert.True(enteringEntry > Fixed64.Zero);
        Assert.Equal(Fixed64.One, enteringExit);
    }

    [Fact]
    public void CenteredCapsule_ExtremeObliqueSweepCanRemainRadiallySeparated()
    {
        Vector3d axis = new(
            Fixed64.FromFraction(3, 5),
            Fixed64.FromFraction(4, 5),
            Fixed64.Zero);
        Vector3d start = new(
            -(Fixed64.MaxValue * axis.Y),
            Fixed64.MaxValue * axis.X,
            Fixed64.Two);
        var query = new FixedSegment(start, new Vector3d(-start.X, -start.Y, start.Z));

        Assert.False(query.TryGetCapsuleIntersectionInterval(
            Vector3d.Zero,
            axis,
            Fixed64.Two,
            Fixed64.One,
            out _,
            out _));
    }

    [Fact]
    public void CenteredFiniteAxisBounds_ValidateFullLengthContracts()
    {
        Assert.Throws<ArgumentException>(() =>
            FixedBoundBox.FromCenteredCapsuleClippedToDomain(
                Vector3d.Zero, Vector3d.Zero, Fixed64.One, Fixed64.One));
        Assert.Throws<ArgumentOutOfRangeException>(() =>
            FixedBoundBox.FromCenteredCapsuleClippedToDomain(
                Vector3d.Zero, Vector3d.Up, -Fixed64.One, Fixed64.One));
        Assert.Throws<ArgumentOutOfRangeException>(() =>
            FixedBoundBox.FromCenteredFiniteCylinderClippedToDomain(
                Vector3d.Zero, Vector3d.Up, Fixed64.Zero, Fixed64.One));
        Assert.Throws<ArgumentOutOfRangeException>(() =>
            FixedBoundBox.FromCenteredFiniteCylinderClippedToDomain(
                Vector3d.Zero, Vector3d.Up, Fixed64.One, -Fixed64.One));
    }

    [Fact]
    public void FullLengthCenteredAxisOperations_DoNotAllocate()
    {
        FixedRange slab = new(-Fixed64.One, Fixed64.One);
        Fixed64 diagonal = FixedMath.Sqrt(Fixed64.Half);
        Vector3d cylinderAxis = new(diagonal, diagonal, Fixed64.Zero);

        ExerciseFullLengthCenteredAxisOperations(slab, cylinderAxis);

        long before = GC.GetAllocatedBytesForCurrentThread();
        for (int i = 0; i < 128; i++)
            ExerciseFullLengthCenteredAxisOperations(slab, cylinderAxis);

        Assert.Equal(0L, GC.GetAllocatedBytesForCurrentThread() - before);
    }

    private static void ExerciseFullLengthCenteredAxisOperations(
        FixedRange slab,
        Vector3d cylinderAxis)
    {
        _ = FixedSegment.TryGetClosestPointsBetweenCenteredAxes(
            Vector3d.Zero,
            Vector3d.Right,
            Fixed64.Two,
            Vector3d.Up,
            Vector3d.Right,
            Fixed64.Two,
            out _,
            out _);
        _ = FixedBoundBox.FromCenteredCapsuleClippedToDomain(
            Vector3d.Zero, Vector3d.Right, Fixed64.Two, Fixed64.One);
        _ = FixedBoundBox.FromCenteredFiniteCylinderClippedToDomain(
            Vector3d.Zero, cylinderAxis, Fixed64.FromRaw(1L), Fixed64.FromRaw(2L));
        _ = FixedSlabProjection.TryGetCapsuleSupport(
            Vector3d.Zero,
            Vector3d.Right,
            Fixed64.Two,
            Fixed64.One,
            slab,
            Vector2d.Right,
            out _);
        Vector2d obliqueAxis = new(Fixed64.FromFraction(3, 5), Fixed64.FromFraction(4, 5));
        Vector2d outside = new(
            -(Fixed64.MaxValue * obliqueAxis.Y),
            Fixed64.MaxValue * obliqueAxis.X);
        _ = new FixedSegment2d(outside, -outside).TryGetCapsuleIntersectionInterval(
            Vector2d.Zero,
            obliqueAxis,
            Fixed64.Two,
            Fixed64.One,
            out _,
            out _);
    }

    private static Vector3d CreateAxisVector(int axisIndex, Fixed64 coordinate) =>
        axisIndex switch
        {
            0 => new Vector3d(coordinate, Fixed64.Zero, Fixed64.Zero),
            1 => new Vector3d(Fixed64.Zero, coordinate, Fixed64.Zero),
            _ => new Vector3d(Fixed64.Zero, Fixed64.Zero, coordinate)
        };
}
