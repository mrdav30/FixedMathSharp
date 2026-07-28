//=======================================================================
// ScaledCompositeTransform.Tests.cs
//=======================================================================
// MIT License, Copyright (c) 2024–present David Oravsky (mrdav30)
// See LICENSE file in the project root for full license information.
//=======================================================================

using System;
using Xunit;

namespace FixedMathSharp.Tests;

public sealed class ScaledCompositeTransformTests
{
    [Fact]
    public void QuaternionTransform_AdmitsFinalCancellationAndCompositeDisplacement()
    {
        var halfTurnY = new FixedQuaternion(
            Fixed64.Zero,
            Fixed64.One,
            Fixed64.Zero,
            Fixed64.Zero);
        Vector3d extreme = new(
            Fixed64.MaxValue,
            Fixed64.Zero,
            Fixed64.Zero);

        Assert.True(halfTurnY.TryTransformScaledPoint(
            extreme,
            extreme,
            new Vector3d(2, 1, 1),
            out Vector3d cancelled));
        Assert.Equal(
            new Vector3d(-Fixed64.MaxValue, Fixed64.Zero, Fixed64.Zero),
            cancelled);

        Assert.True(FixedQuaternion.Identity.TryTransformScaledPoint(
            new Vector3d(10, -20, 30),
            new Vector3d(2, 3, 4),
            new Vector3d(5, 6, 7),
            new Vector3d(1, 2, 3),
            out Vector3d composite));
        Assert.Equal(new Vector3d(21, 0, 61), composite);
    }

    [Fact]
    public void QuaternionTransform_IsAtomicAndPreservesZeroQuaternionContract()
    {
        Vector3d extreme = new(
            Fixed64.MaxValue,
            Fixed64.Zero,
            Fixed64.Zero);

        Assert.False(FixedQuaternion.Identity.TryTransformScaledPoint(
            extreme,
            extreme,
            new Vector3d(2, 1, 1),
            out Vector3d rejected));
        Assert.Equal(default, rejected);

        Assert.True(FixedQuaternion.Zero.TryTransformScaledPoint(
            extreme,
            extreme,
            new Vector3d(2, 1, 1),
            Vector3d.One,
            out Vector3d zeroRotation));
        Assert.Equal(extreme, zeroRotation);
    }

    [Fact]
    public void PlanarTransform_AdmitsFinalCancellationAndRotatesCompositeLocalPoint()
    {
        Vector2d extreme = new(
            Fixed64.MaxValue,
            Fixed64.Zero);

        Assert.True(Vector2d.TryTransformScaledPoint(
            -extreme,
            extreme,
            new Vector2d(2, 1),
            Fixed64.Zero,
            out Vector2d cancelled));
        Assert.Equal(extreme, cancelled);

        Assert.True(Vector2d.TryTransformScaledPoint(
            new Vector2d(3, 4),
            new Vector2d(1, 0),
            new Vector2d(2, 1),
            new Vector2d(0, 1),
            Fixed64.HalfPi,
            out Vector2d rotated));
        Assert.Equal(new Vector2d(2, 6), rotated);
    }

    [Fact]
    public void PlanarTransform_IsAtomicWhenFinalCoordinateIsUnrepresentable()
    {
        Vector2d extreme = new(
            Fixed64.MaxValue,
            Fixed64.Zero);

        Assert.False(Vector2d.TryTransformScaledPoint(
            extreme,
            extreme,
            new Vector2d(2, 1),
            Fixed64.Zero,
            out Vector2d rejected));
        Assert.Equal(default, rejected);
    }

    [Fact]
    public void PlanarLocalComposition_AdmitsCancellationAcrossBothScaledFrames()
    {
        Vector2d extreme = new(
            Fixed64.MaxValue,
            Fixed64.Zero);

        Assert.True(Vector2d.TryComposeScaledLocalPoints(
            extreme,
            new Vector2d(2, 1),
            -extreme,
            new Vector2d(2, 1),
            Vector2d.Zero,
            Fixed64.Pi,
            out Vector2d cancelled));
        Assert.Equal(Vector2d.Zero, cancelled);

        Assert.True(Vector2d.TryComposeScaledLocalPoints(
            new Vector2d(2, 3),
            new Vector2d(4, 5),
            new Vector2d(1, 2),
            new Vector2d(6, 7),
            new Vector2d(1, 1),
            Fixed64.HalfPi,
            out Vector2d composed));
        Assert.Equal(new Vector2d(13, 30), composed);

        Assert.False(Vector2d.TryComposeScaledLocalPoints(
            extreme,
            new Vector2d(2, 1),
            Vector2d.Zero,
            Vector2d.One,
            Vector2d.Zero,
            Fixed64.Zero,
            out Vector2d rejected));
        Assert.Equal(default, rejected);
    }

    [Fact]
    public void SpatialLocalComposition_AdmitsCancellationAcrossBothScaledFrames()
    {
        Vector3d extreme = new(
            Fixed64.MaxValue,
            Fixed64.Zero,
            Fixed64.Zero);
        var halfTurnY = new FixedQuaternion(
            Fixed64.Zero,
            Fixed64.One,
            Fixed64.Zero,
            Fixed64.Zero);

        Assert.True(Vector3d.TryComposeScaledLocalPoints(
            extreme,
            new Vector3d(2, 1, 1),
            -extreme,
            new Vector3d(2, 1, 1),
            Vector3d.Zero,
            halfTurnY,
            out Vector3d cancelled));
        Assert.Equal(Vector3d.Zero, cancelled);

        Assert.False(Vector3d.TryComposeScaledLocalPoints(
            extreme,
            new Vector3d(2, 1, 1),
            Vector3d.Zero,
            Vector3d.One,
            Vector3d.Zero,
            FixedQuaternion.Identity,
            out Vector3d rejected));
        Assert.Equal(default, rejected);
    }

    [Fact]
    public void ScaledCompositeTransforms_DoNotAllocateAfterWarmup()
    {
        var rotation = FixedQuaternion.FromEulerAnglesInDegrees(
            (Fixed64)17,
            (Fixed64)(-23),
            (Fixed64)31);
        _ = rotation.TryTransformScaledPoint(
            new Vector3d(3, 4, 5),
            new Vector3d(6, 7, 8),
            new Vector3d(2, 3, 4),
            Vector3d.One,
            out _);
        _ = Vector2d.TryTransformScaledPoint(
            new Vector2d(3, 4),
            new Vector2d(6, 7),
            new Vector2d(2, 3),
            Vector2d.One,
            Fixed64.FromFraction(2, 3),
            out _);
        _ = Vector2d.TryComposeScaledLocalPoints(
            new Vector2d(3, 4),
            new Vector2d(2, 3),
            new Vector2d(6, 7),
            new Vector2d(4, 5),
            Vector2d.One,
            Fixed64.FromFraction(2, 3),
            out _);
        _ = Vector3d.TryComposeScaledLocalPoints(
            new Vector3d(3, 4, 5),
            new Vector3d(2, 3, 4),
            new Vector3d(6, 7, 8),
            new Vector3d(4, 5, 6),
            Vector3d.One,
            rotation,
            out _);
        _ = Vector3d.TryCross(Vector3d.Right, Vector3d.Up, out _);
        _ = Vector3d.TryDot(Vector3d.Right, Vector3d.Up, out _);
        _ = Fixed3x3.TryTransformDirection(
            Fixed3x3.Identity,
            Vector3d.One,
            out _);
        _ = Vector3d.TryLinearCombination(
            Vector3d.Right,
            Fixed64.One,
            Vector3d.Up,
            Fixed64.One,
            Vector3d.Forward,
            Fixed64.One,
            out _);
        long before = GC.GetAllocatedBytesForCurrentThread();

        for (int iteration = 0; iteration < 64; iteration++)
        {
            _ = rotation.TryTransformScaledPoint(
                new Vector3d(3, 4, 5),
                new Vector3d(6, 7, 8),
                new Vector3d(2, 3, 4),
                Vector3d.One,
                out _);
            _ = Vector2d.TryTransformScaledPoint(
                new Vector2d(3, 4),
                new Vector2d(6, 7),
                new Vector2d(2, 3),
                Vector2d.One,
                Fixed64.FromFraction(2, 3),
                out _);
            _ = Vector2d.TryComposeScaledLocalPoints(
                new Vector2d(3, 4),
                new Vector2d(2, 3),
                new Vector2d(6, 7),
                new Vector2d(4, 5),
                Vector2d.One,
                Fixed64.FromFraction(2, 3),
                out _);
            _ = Vector3d.TryComposeScaledLocalPoints(
                new Vector3d(3, 4, 5),
                new Vector3d(2, 3, 4),
                new Vector3d(6, 7, 8),
                new Vector3d(4, 5, 6),
                Vector3d.One,
                rotation,
                out _);
            _ = Vector3d.TryCross(
                Vector3d.Right,
                Vector3d.Up,
                out _);
            _ = Vector3d.TryDot(
                Vector3d.Right,
                Vector3d.Up,
                out _);
            _ = Fixed3x3.TryTransformDirection(
                Fixed3x3.Identity,
                Vector3d.One,
                out _);
            _ = Vector3d.TryLinearCombination(
                Vector3d.Right,
                Fixed64.One,
                Vector3d.Up,
                Fixed64.One,
                Vector3d.Forward,
                Fixed64.One,
                out _);
        }

        Assert.Equal(before, GC.GetAllocatedBytesForCurrentThread());
    }
}
