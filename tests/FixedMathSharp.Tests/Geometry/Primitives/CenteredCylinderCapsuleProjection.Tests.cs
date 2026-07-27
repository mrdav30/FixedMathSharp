using FixedMathSharp.Geometry;
using System;
using Xunit;

namespace FixedMathSharp.Tests;

public sealed class CenteredCylinderCapsuleProjectionTests
{
    [Fact]
    public void AxisPenetration_CancelsUnrepresentableRadicalBeforeRounding()
    {
        long scale = Fixed64.One.m_rawValue;
        Vector3d projectionAxis = new(
            Fixed64.FromRaw(scale + 64L),
            Fixed64.Zero,
            Fixed64.Zero);
        Vector3d capsuleCenter = new(
            Fixed64.MaxValue - Fixed64.One,
            Fixed64.Zero,
            Fixed64.Zero);
        Fixed64 expectedDepth = Fixed64.FromRaw(scale + 64L);

        Assert.True(FixedSegment.TryGetCenteredFiniteCylinderCapsuleAxisPenetration(
            projectionAxis,
            Vector3d.Zero,
            Vector3d.Up,
            Fixed64.One,
            Fixed64.Zero,
            capsuleCenter,
            Vector3d.Up,
            Fixed64.Zero,
            Fixed64.MaxValue,
            out Vector3d clampedAxis,
            out Fixed64 clampedDepth,
            out bool depthIsClamped));
        Assert.Equal(projectionAxis, clampedAxis);
        Assert.Equal(expectedDepth, clampedDepth);
        Assert.False(depthIsClamped);

        long before = GC.GetAllocatedBytesForCurrentThread();
        bool result = FixedSegment.TryGetCenteredFiniteCylinderCapsuleAxisPenetration(
            projectionAxis,
            Vector3d.Zero,
            Vector3d.Up,
            Fixed64.One,
            Fixed64.Zero,
            capsuleCenter,
            Vector3d.Up,
            Fixed64.Zero,
            Fixed64.MaxValue,
            out Vector3d axis,
            out Fixed64 depth);
        long allocated = GC.GetAllocatedBytesForCurrentThread() - before;

        Assert.True(result);
        Assert.Equal(clampedAxis, axis);
        Assert.Equal(clampedDepth, depth);
        Assert.Equal(0L, allocated);
    }

    [Fact]
    public void AxisPenetration_ClampsOnlyAboveMaximumDepth()
    {
        Assert.True(FixedSegment.TryGetCenteredFiniteCylinderCapsuleAxisPenetration(
            Vector3d.Right,
            Vector3d.Zero,
            Vector3d.Up,
            Fixed64.One,
            Fixed64.Zero,
            Vector3d.Zero,
            Vector3d.Up,
            Fixed64.Zero,
            Fixed64.MaxValue,
            out Vector3d maximumAxis,
            out Fixed64 maximumDepth,
            out bool maximumIsClamped));
        Assert.Equal(Vector3d.Right, maximumAxis);
        Assert.Equal(Fixed64.MaxValue, maximumDepth);
        Assert.False(maximumIsClamped);
        Assert.True(FixedSegment.TryGetCenteredFiniteCylinderCapsuleAxisPenetration(
            Vector3d.Right,
            Vector3d.Zero,
            Vector3d.Up,
            Fixed64.One,
            Fixed64.Zero,
            Vector3d.Zero,
            Vector3d.Up,
            Fixed64.Zero,
            Fixed64.MaxValue,
            out Vector3d exactAxis,
            out Fixed64 exactDepth));
        Assert.Equal(maximumAxis, exactAxis);
        Assert.Equal(maximumDepth, exactDepth);

        long scale = Fixed64.One.m_rawValue;
        Vector3d projectionAxis = new(
            Fixed64.FromRaw(scale + 64L),
            Fixed64.Zero,
            Fixed64.Zero);
        Vector3d capsuleCenter = new(
            Fixed64.FromRaw(137_438_951_424L),
            Fixed64.Zero,
            Fixed64.Zero);

        Assert.True(FixedSegment.TryGetCenteredFiniteCylinderCapsuleAxisPenetration(
            projectionAxis,
            Vector3d.Zero,
            Vector3d.Up,
            Fixed64.One,
            Fixed64.Zero,
            capsuleCenter,
            Vector3d.Up,
            Fixed64.Zero,
            Fixed64.MaxValue,
            out Vector3d clampedAxis,
            out Fixed64 clampedDepth,
            out bool depthIsClamped));
        Assert.Equal(projectionAxis, clampedAxis);
        Assert.Equal(Fixed64.MaxValue, clampedDepth);
        Assert.True(depthIsClamped);

        Assert.False(FixedSegment.TryGetCenteredFiniteCylinderCapsuleAxisPenetration(
            projectionAxis,
            Vector3d.Zero,
            Vector3d.Up,
            Fixed64.One,
            Fixed64.Zero,
            capsuleCenter,
            Vector3d.Up,
            Fixed64.Zero,
            Fixed64.MaxValue,
            out _,
            out _));
    }

    [Fact]
    public void AxisPenetration_UnclampedOverloadRejectsUnrepresentableDepth()
    {
        Assert.True(FixedSegment
            .DoCenteredFiniteCylinderAndCapsuleOverlapOnAxis(
                Vector3d.Up,
                Vector3d.Zero,
                Vector3d.Up,
                Fixed64.MaxValue,
                Fixed64.MaxValue,
                Vector3d.Zero,
                Vector3d.Up,
                Fixed64.MaxValue,
                Fixed64.MaxValue));

        Assert.False(FixedSegment
            .TryGetCenteredFiniteCylinderCapsuleAxisPenetration(
                Vector3d.Up,
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
    }

    [Fact]
    public void AxisPenetration_AtPositiveScalarFace_DoesNotMaterializeRanges()
    {
        Vector3d cylinderCenter = new(
            Fixed64.MaxValue,
            Fixed64.Zero,
            Fixed64.Zero);
        Vector3d capsuleCenter = new(
            Fixed64.MaxValue - Fixed64.FromFraction(1, 4),
            Fixed64.Zero,
            Fixed64.Zero);

        Assert.True(FixedSegment.TryGetCenteredFiniteCylinderCapsuleAxisPenetration(
            Vector3d.Up,
            cylinderCenter,
            Vector3d.Up,
            Fixed64.One,
            Fixed64.Half,
            capsuleCenter,
            Vector3d.Up,
            Fixed64.Two,
            Fixed64.Half,
            out Vector3d cylinderToCapsuleAxis,
            out Fixed64 depth,
            out bool depthIsClamped));
        Assert.Equal(Vector3d.Up, cylinderToCapsuleAxis);
        Assert.Equal(Fixed64.Two, depth);
        Assert.False(depthIsClamped);

        Assert.True(FixedSegment.TryGetCenteredFiniteCylinderCapsuleAxisPenetration(
            Vector3d.Up,
            cylinderCenter,
            Vector3d.Up,
            Fixed64.One,
            Fixed64.Half,
            capsuleCenter,
            Vector3d.Up,
            Fixed64.Two,
            Fixed64.Half,
            out cylinderToCapsuleAxis,
            out depth));
        Assert.Equal(Vector3d.Up, cylinderToCapsuleAxis);
        Assert.Equal(Fixed64.Two, depth);

        Assert.True(FixedSegment.TryGetCenteredFiniteCylinderCapsuleAxisPenetration(
            Vector3d.Right,
            cylinderCenter,
            Vector3d.Up,
            Fixed64.One,
            Fixed64.Half,
            capsuleCenter,
            Vector3d.Up,
            Fixed64.Two,
            Fixed64.Half,
            out cylinderToCapsuleAxis,
            out depth,
            out depthIsClamped));
        Assert.Equal(Vector3d.Left, cylinderToCapsuleAxis);
        Assert.Equal(Fixed64.FromFraction(3, 4), depth);
        Assert.False(depthIsClamped);

        Assert.True(FixedSegment.TryGetClosestPointsBetweenCenteredAxes(
            capsuleCenter,
            Vector3d.Up,
            Fixed64.Two,
            cylinderCenter,
            Vector3d.Up,
            Fixed64.One,
            out Vector3d capsulePoint,
            out Vector3d cylinderPoint));
        Assert.Equal(capsuleCenter.X, capsulePoint.X);
        Assert.Equal(cylinderCenter.X, cylinderPoint.X);
        Assert.Equal(capsulePoint.Y, cylinderPoint.Y);
        Assert.Equal(Fixed64.Zero, capsulePoint.Z);
        Assert.Equal(Fixed64.Zero, cylinderPoint.Z);
    }
}
