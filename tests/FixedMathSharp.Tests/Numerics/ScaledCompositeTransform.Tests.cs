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
    public void PlanarInverseTransform_RoundTripsAnisotropicMirroredScale()
    {
        Vector2d origin = new(10, 20);
        Vector2d localPoint = new(3, -4);
        Vector2d scale = new(-2, 3);

        Assert.True(Vector2d.TryTransformScaledPoint(
            origin,
            localPoint,
            scale,
            Fixed64.HalfPi,
            out Vector2d worldPoint));
        Assert.Equal(new Vector2d(22, 14), worldPoint);
        Assert.True(Vector2d.TryInverseTransformScaledPoint(
            origin,
            worldPoint,
            scale,
            Fixed64.HalfPi,
            out Vector2d roundTrip));
        Assert.Equal(localPoint, roundTrip);
    }

    [Fact]
    public void PlanarInverseTransform_RetainsFullDomainSubtractionAndHalfEvenRounding()
    {
        Vector2d origin = new(Fixed64.MinValue, Fixed64.Zero);
        Vector2d worldPoint = new(Fixed64.MaxValue, Fixed64.Zero);
        Vector2d scale = new((Fixed64)3, Fixed64.One);
        Fixed64 expectedX = Fixed64.FromRaw(6148914691236517205L);

        Assert.True(Vector2d.TryInverseTransformScaledPoint(
            origin,
            worldPoint,
            scale,
            Fixed64.Zero,
            out Vector2d fullDomain));
        Assert.Equal(new Vector2d(expectedX, Fixed64.Zero), fullDomain);

        Assert.True(Vector2d.TryInverseTransformScaledPoint(
            Vector2d.Zero,
            new Vector2d(Fixed64.FromRaw(1), Fixed64.Zero),
            new Vector2d(Fixed64.Two, Fixed64.One),
            Fixed64.Zero,
            out Vector2d even));
        Assert.Equal(Vector2d.Zero, even);

        Assert.True(Vector2d.TryInverseTransformScaledPoint(
            Vector2d.Zero,
            new Vector2d(Fixed64.FromRaw(3), Fixed64.Zero),
            new Vector2d(Fixed64.Two, Fixed64.One),
            Fixed64.Zero,
            out Vector2d odd));
        Assert.Equal(new Vector2d(Fixed64.FromRaw(2), Fixed64.Zero), odd);
    }

    [Fact]
    public void PlanarInverseTransform_RejectsSingularOrUnrepresentableResultsAtomically()
    {
        Assert.False(Vector2d.TryInverseTransformScaledPoint(
            Vector2d.Zero,
            Vector2d.One,
            new Vector2d(Fixed64.Zero, Fixed64.One),
            Fixed64.PiOver4,
            out Vector2d singular));
        Assert.Equal(default, singular);

        Assert.False(Vector2d.TryInverseTransformScaledPoint(
            new Vector2d(Fixed64.MinValue, Fixed64.Zero),
            new Vector2d(Fixed64.MaxValue, Fixed64.Zero),
            new Vector2d(Fixed64.MinIncrement, Fixed64.One),
            Fixed64.Zero,
            out Vector2d overflow));
        Assert.Equal(default, overflow);
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
    public void QuaternionInverseTransform_RetainsSubtractionAndDivisionUntilFinalCoordinate()
    {
        Vector3d scale = new((Fixed64)3, Fixed64.One, Fixed64.One);
        Vector3d origin = new(Fixed64.MinValue, Fixed64.Zero, Fixed64.Zero);
        Vector3d worldPoint = new(Fixed64.MaxValue, Fixed64.Zero, Fixed64.Zero);
        Fixed64 expectedX = Fixed64.FromRaw(6148914691236517205L);

        Assert.True(FixedQuaternion.Identity.TryInverseTransformScaledPoint(
            origin,
            worldPoint,
            scale,
            out Vector3d localPoint));
        Assert.Equal(new Vector3d(expectedX, Fixed64.Zero, Fixed64.Zero), localPoint);
        Assert.True(FixedQuaternion.Identity.TryTransformScaledPoint(
            origin,
            localPoint,
            scale,
            out Vector3d roundTrip));
        Assert.Equal(worldPoint, roundTrip);

        Assert.True(FixedQuaternion.Identity.TryInverseTransformScaledPoint(
            -origin,
            -worldPoint,
            scale,
            out Vector3d mirrored));
        Assert.Equal(new Vector3d(-expectedX, Fixed64.Zero, Fixed64.Zero), mirrored);

        var halfTurnY = new FixedQuaternion(
            Fixed64.Zero,
            Fixed64.One,
            Fixed64.Zero,
            Fixed64.Zero);
        Vector3d authoredLocal = new((Fixed64)5, (Fixed64)(-6), (Fixed64)7);
        Vector3d mirroredScale = new((Fixed64)(-3), Fixed64.Two, (Fixed64)4);
        Assert.True(halfTurnY.TryTransformScaledPoint(
            new Vector3d(10, 20, 30),
            authoredLocal,
            mirroredScale,
            out Vector3d mirroredWorld));
        Assert.True(halfTurnY.TryInverseTransformScaledPoint(
            new Vector3d(10, 20, 30),
            mirroredWorld,
            mirroredScale,
            out Vector3d mirroredRoundTrip));
        Assert.Equal(authoredLocal, mirroredRoundTrip);
    }

    [Fact]
    public void QuaternionInverseTransform_RetainsRotationProjectionThroughScaleDivision()
    {
        var rotation = new FixedQuaternion(
            Fixed64.Zero,
            Fixed64.Zero,
            Fixed64.One,
            Fixed64.Two);
        Vector3d worldPoint = new(
            Fixed64.MaxValue,
            Fixed64.MaxValue,
            Fixed64.Zero);
        Assert.True(Fixed64.TryMultiplyDivide(
            Fixed64.MaxValue,
            (Fixed64)7,
            (Fixed64)10,
            out Fixed64 expectedX));
        Assert.True(Fixed64.TryMultiplyDivide(
            Fixed64.MaxValue,
            -Fixed64.One,
            (Fixed64)5,
            out Fixed64 expectedY));

        Assert.True(rotation.TryInverseTransformScaledPoint(
            Vector3d.Zero,
            worldPoint,
            new Vector3d(Fixed64.Two, Fixed64.One, Fixed64.One),
            out Vector3d localPoint));
        Assert.Equal(
            new Vector3d(expectedX, expectedY, Fixed64.Zero),
            localPoint);
    }

    [Fact]
    public void QuaternionInverseTransform_UsesFinalHalfEvenRounding()
    {
        Vector3d scale = new(Fixed64.Two, Fixed64.One, Fixed64.One);

        Assert.True(FixedQuaternion.Identity.TryInverseTransformScaledPoint(
            Vector3d.Zero,
            new Vector3d(Fixed64.FromRaw(1), Fixed64.Zero, Fixed64.Zero),
            scale,
            out Vector3d even));
        Assert.Equal(Vector3d.Zero, even);

        Assert.True(FixedQuaternion.Identity.TryInverseTransformScaledPoint(
            Vector3d.Zero,
            new Vector3d(Fixed64.FromRaw(3), Fixed64.Zero, Fixed64.Zero),
            scale,
            out Vector3d odd));
        Assert.Equal(
            new Vector3d(Fixed64.FromRaw(2), Fixed64.Zero, Fixed64.Zero),
            odd);
    }

    [Fact]
    public void QuaternionInverseTransform_RejectsSingularOrUnrepresentableResultsAtomically()
    {
        Vector3d maximum = new(
            Fixed64.MaxValue,
            Fixed64.Zero,
            Fixed64.Zero);
        Vector3d minimum = new(
            Fixed64.MinValue,
            Fixed64.Zero,
            Fixed64.Zero);

        Assert.False(FixedQuaternion.Identity.TryInverseTransformScaledPoint(
            Vector3d.Zero,
            Vector3d.One,
            new Vector3d(Fixed64.One, Fixed64.Zero, Fixed64.One),
            out Vector3d singular));
        Assert.Equal(default, singular);
        Assert.False(FixedQuaternion.Identity.TryInverseTransformScaledPoint(
            Vector3d.Zero,
            Vector3d.One,
            new Vector3d(Fixed64.Zero, Fixed64.One, Fixed64.One),
            out singular));
        Assert.False(FixedQuaternion.Identity.TryInverseTransformScaledPoint(
            Vector3d.Zero,
            Vector3d.One,
            new Vector3d(Fixed64.One, Fixed64.One, Fixed64.Zero),
            out singular));

        Assert.False(FixedQuaternion.Zero.TryInverseTransformScaledPoint(
            Vector3d.Zero,
            Vector3d.One,
            Vector3d.One,
            out Vector3d zeroRotation));
        Assert.Equal(default, zeroRotation);

        Assert.False(FixedQuaternion.Identity.TryInverseTransformScaledPoint(
            minimum,
            maximum,
            Vector3d.One,
            out Vector3d overflow));
        Assert.Equal(default, overflow);
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
        _ = rotation.TryInverseTransformScaledPoint(
            new Vector3d(3, 4, 5),
            new Vector3d(6, 7, 8),
            new Vector3d(2, 3, 4),
            out _);
        for (int iteration = 0; iteration < 32; iteration++)
        {
            _ = rotation.TryInverseTransformScaledPoint(
                new Vector3d(3, 4, 5),
                new Vector3d(6, 7, 8),
                new Vector3d(2, 3, 4),
                out _);
        }
        _ = Vector2d.TryTransformScaledPoint(
            new Vector2d(3, 4),
            new Vector2d(6, 7),
            new Vector2d(2, 3),
            Vector2d.One,
            Fixed64.FromFraction(2, 3),
            out _);
        _ = Vector2d.TryInverseTransformScaledPoint(
            new Vector2d(3, 4),
            new Vector2d(6, 7),
            new Vector2d(2, 3),
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
            _ = rotation.TryInverseTransformScaledPoint(
                new Vector3d(3, 4, 5),
                new Vector3d(6, 7, 8),
                new Vector3d(2, 3, 4),
                out _);
            _ = Vector2d.TryTransformScaledPoint(
                new Vector2d(3, 4),
                new Vector2d(6, 7),
                new Vector2d(2, 3),
                Vector2d.One,
                Fixed64.FromFraction(2, 3),
                out _);
            _ = Vector2d.TryInverseTransformScaledPoint(
                new Vector2d(3, 4),
                new Vector2d(6, 7),
                new Vector2d(2, 3),
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
