//=======================================================================
// FixedOrientedBox.CircleSlabSweep.Tests.cs
//=======================================================================
// MIT License, Copyright (c) 2024–present David Oravsky (mrdav30)
// See LICENSE file in the project root for full license information.
//=======================================================================

using FixedMathSharp.Bounds;
using System;
using Xunit;

namespace FixedMathSharp.Tests.Bounds;

public sealed class FixedOrientedBoxCircleSlabSweepTests
{
    [Fact]
    public void SeparationLowerBound_UsesStrongestProjectedFaceGap()
    {
        var box = new FixedOrientedBox(
            Vector3d.Zero,
            FixedQuaternion.Identity,
            Vector3d.One);

        Assert.Equal(
            (Fixed64)3,
            box.GetCircleSlabSeparationLowerBound(
                new Vector3d(5, 0, 0),
                Fixed64.One,
                Fixed64.One));
        Fixed64 cornerGap = box.GetCircleSlabSeparationLowerBound(
            new Vector3d(3, 0, 3),
            Fixed64.One,
            Fixed64.One);
        Assert.True(cornerGap > Fixed64.One, $"Gap: {cornerGap}");
        Assert.True(cornerGap <= FixedMath.Sqrt((Fixed64)8) - Fixed64.One);
        Assert.Equal(
            Fixed64.Zero,
            box.GetCircleSlabSeparationLowerBound(
                new Vector3d(3, 0, 3),
                Fixed64.One,
                (Fixed64)3));
        Assert.Equal(
            Fixed64.Zero,
            box.GetCircleSlabSeparationLowerBound(
                Vector3d.Zero,
                Fixed64.One,
                Fixed64.One));
        Assert.Equal(
            Fixed64.FromFraction(7, 2),
            box.GetCircleSlabSeparationLowerBound(
                new Vector3d(0, 5, 0),
                Fixed64.Half,
                Fixed64.One));
    }

    [Fact]
    public void SeparationLowerBound_LargeRotatedCornerNearScalarFacesMatchesOriginGeometry()
    {
        FixedQuaternion orientation =
            FixedQuaternion.FromEulerAnglesInDegrees(
                Fixed64.Zero,
                (Fixed64)29,
                Fixed64.Zero);
        Vector3d halfExtents = new(
            (Fixed64)50_000_000,
            Fixed64.One,
            Fixed64.One);
        Vector3d localCircleCenter = new(
            halfExtents.X + (Fixed64)3,
            Fixed64.Zero,
            halfExtents.Z + (Fixed64)3);
        Vector3d relativeCircleCenter = orientation * localCircleCenter;
        var originBox = new FixedOrientedBox(
            Vector3d.Zero,
            orientation,
            halfExtents);
        Fixed64 originGap = originBox.GetCircleSlabSeparationLowerBound(
            relativeCircleCenter,
            Fixed64.One,
            Fixed64.One);
        Vector3d translatedCenter = new(
            Fixed64.MaxValue - (Fixed64)100_000_000,
            Fixed64.Zero,
            Fixed64.MinValue + (Fixed64)100_000_000);
        var translatedBox = new FixedOrientedBox(
            translatedCenter,
            orientation,
            halfExtents);

        Fixed64 translatedGap =
            translatedBox.GetCircleSlabSeparationLowerBound(
                translatedCenter + relativeCircleCenter,
                Fixed64.One,
                Fixed64.One);

        Assert.True(originGap > Fixed64.Two, $"Gap: {originGap}");
        Assert.Equal(originGap, translatedGap);
    }

    [Fact]
    public void Sweep_FindsFirstExpandedFaceEntry()
    {
        var box = new FixedOrientedBox(
            Vector3d.Zero,
            FixedQuaternion.Identity,
            Vector3d.One);

        Assert.True(box.TryGetCircleSlabSweepDistance(
            new Vector3d(-5, 0, 0),
            Vector2d.Right,
            (Fixed64)10,
            Fixed64.One,
            Fixed64.Half,
            out Fixed64 distance));
        Assert.Equal(Fixed64.FromFraction(7, 2), distance);
    }

    [Fact]
    public void Sweep_IncludesCornerTangencyAndRejectsSeparation()
    {
        var box = new FixedOrientedBox(
            Vector3d.Zero,
            FixedQuaternion.Identity,
            Vector3d.One);

        Assert.True(box.TryGetCircleSlabSweepDistance(
            new Vector3d(-5, 0, 2),
            Vector2d.Right,
            (Fixed64)10,
            Fixed64.One,
            Fixed64.One,
            out Fixed64 tangent));
        Assert.Equal((Fixed64)4, tangent);
        Assert.False(box.TryGetCircleSlabSweepDistance(
            new Vector3d(
                (Fixed64)(-5),
                Fixed64.Zero,
                Fixed64.FromRaw((2L << 32) + 1L)),
            Vector2d.Right,
            (Fixed64)10,
            Fixed64.One,
            Fixed64.One,
            out _));
    }

    [Fact]
    public void Sweep_ReturnsZeroForStartOverlapAndRejectsDisjointSlab()
    {
        var box = new FixedOrientedBox(
            Vector3d.Zero,
            FixedQuaternion.Identity,
            Vector3d.One);

        Assert.True(box.TryGetCircleSlabSweepDistance(
            Vector3d.Zero,
            Vector2d.Right,
            Fixed64.One,
            Fixed64.One,
            Fixed64.Zero,
            out Fixed64 overlapping));
        Assert.Equal(Fixed64.Zero, overlapping);
        Assert.False(box.TryGetCircleSlabSweepDistance(
            new Vector3d(-5, 5, 0),
            Vector2d.Right,
            (Fixed64)10,
            Fixed64.Half,
            Fixed64.One,
            out _));
    }

    [Fact]
    public void ZeroLengthSweepOutsideProjection_ReturnsFalse()
    {
        var box = new FixedOrientedBox(
            Vector3d.Zero,
            FixedQuaternion.Identity,
            Vector3d.One);

        Assert.False(box.TryGetCircleSlabSweepDistance(
            new Vector3d((Fixed64)(-5), Fixed64.Zero, Fixed64.Zero),
            Vector2d.Right,
            Fixed64.Zero,
            Fixed64.One,
            Fixed64.One,
            out _));
    }

    [Fact]
    public void SeparationLowerBound_IsSymmetricAcrossVerticalAndRadialSides()
    {
        var box = new FixedOrientedBox(
            Vector3d.Zero,
            FixedQuaternion.Identity,
            Vector3d.One);

        Assert.Equal(
            Fixed64.Zero,
            box.GetCircleSlabSeparationLowerBound(
                new Vector3d(
                    Fixed64.FromFraction(3, 2),
                    Fixed64.Zero,
                    Fixed64.Zero),
                Fixed64.One,
                Fixed64.One));
        Assert.Equal(
            Fixed64.FromFraction(7, 2),
            box.GetCircleSlabSeparationLowerBound(
                new Vector3d(
                    Fixed64.Zero,
                    (Fixed64)(-5),
                    Fixed64.Zero),
                Fixed64.Half,
                Fixed64.One));
    }

    [Fact]
    public void Sweep_TouchingExpandedFaceExactlyAtMaximumReturnsMaximum()
    {
        var box = new FixedOrientedBox(
            Vector3d.Zero,
            FixedQuaternion.Identity,
            Vector3d.One);

        Assert.True(box.TryGetCircleSlabSweepDistance(
            new Vector3d((Fixed64)(-5), Fixed64.Zero, Fixed64.Zero),
            Vector2d.Right,
            (Fixed64)4,
            Fixed64.One,
            Fixed64.Zero,
            out Fixed64 distance));

        Assert.Equal((Fixed64)4, distance);
    }

    [Fact]
    public void Sweep_TouchingRoundedCornerExactlyAtMaximumReturnsMaximum()
    {
        var box = new FixedOrientedBox(
            Vector3d.Zero,
            FixedQuaternion.Identity,
            Vector3d.One);

        Assert.True(box.TryGetCircleSlabSweepDistance(
            new Vector3d((Fixed64)(-5), Fixed64.Zero, Fixed64.Two),
            Vector2d.Right,
            Fixed64.FromFraction(13, 4),
            Fixed64.One,
            Fixed64.FromFraction(5, 4),
            out Fixed64 distance));

        Assert.Equal(Fixed64.FromFraction(13, 4), distance);
    }

    [Fact]
    public void Sweep_RoundedCornerEntryBeforeClosestApproachRoundsExactly()
    {
        var box = new FixedOrientedBox(
            Vector3d.Zero,
            FixedQuaternion.Identity,
            Vector3d.One);

        Assert.True(box.TryGetCircleSlabSweepDistance(
            new Vector3d((Fixed64)(-5), Fixed64.Zero, Fixed64.Two),
            Vector2d.Right,
            Fixed64.FromFraction(7, 2),
            Fixed64.One,
            Fixed64.FromFraction(5, 4),
            out Fixed64 distance));
        Assert.Equal(Fixed64.FromFraction(13, 4), distance);

        Assert.False(box.TryGetCircleSlabSweepDistance(
            new Vector3d((Fixed64)(-5), Fixed64.Zero, Fixed64.Two),
            Vector2d.Left,
            Fixed64.One,
            Fixed64.One,
            Fixed64.FromFraction(5, 4),
            out _));
    }

    [Fact]
    public void Sweep_InvertedLocalYMatchesIdentityProjection()
    {
        var box = new FixedOrientedBox(
            Vector3d.Zero,
            FixedQuaternion.FromAxisAngle(
                Vector3d.Forward,
                Fixed64.Pi),
            Vector3d.One);

        Assert.True(box.TryGetCircleSlabSweepDistance(
            new Vector3d((Fixed64)(-5), Fixed64.Zero, Fixed64.Zero),
            Vector2d.Right,
            (Fixed64)10,
            Fixed64.One,
            Fixed64.Half,
            out Fixed64 distance));

        Assert.Equal(Fixed64.FromFraction(7, 2), distance);
    }

    [Fact]
    public void Sweep_TiltedProjectedFaceUsesItsExactPlanarLength()
    {
        FixedQuaternion orientation =
            FixedQuaternion.FromEulerAnglesInDegrees(
                (Fixed64)45,
                (Fixed64)30,
                Fixed64.Zero);
        var box = new FixedOrientedBox(
            Vector3d.Zero,
            orientation,
            Vector3d.One);

        Vector3d projectedAxis3 = orientation * Vector3d.Down;
        var projectedAxis = new Vector2d(
            projectedAxis3.X,
            projectedAxis3.Z).Normalized;
        Assert.True(box.TryGetCircleSlabSweepDistance(
            new Vector3d(
                -projectedAxis.X * (Fixed64)5,
                Fixed64.Zero,
                -projectedAxis.Y * (Fixed64)5),
            projectedAxis,
            (Fixed64)10,
            (Fixed64)2,
            Fixed64.Half,
            out Fixed64 distance));
        Assert.True(distance > Fixed64.Zero);
        Assert.True(distance < (Fixed64)10);
    }

    [Fact]
    public void Sweep_UsesWideSlabIntersectionsWhenNoBoxCornerIsInside()
    {
        var box = new FixedOrientedBox(
            Vector3d.Zero,
            FixedQuaternion.Identity,
            new Vector3d(1, 2, 1));

        Assert.True(box.TryGetCircleSlabSweepDistance(
            new Vector3d(-5, 0, 0),
            Vector2d.Right,
            (Fixed64)10,
            Fixed64.Half,
            Fixed64.Zero,
            out Fixed64 distance));
        Assert.Equal((Fixed64)4, distance);
    }

    [Fact]
    public void Sweep_PreservesScalarFaceGeometryWithoutMaterializingCorners()
    {
        var box = new FixedOrientedBox(
            new Vector3d(
                Fixed64.MaxValue,
                Fixed64.Zero,
                Fixed64.Zero),
            FixedQuaternion.Identity,
            Vector3d.One);
        Vector3d start = new(
            Fixed64.MaxValue - (Fixed64)3,
            Fixed64.Zero,
            Fixed64.Zero);

        Assert.True(box.TryGetCircleSlabSweepDistance(
            start,
            Vector2d.Right,
            (Fixed64)4,
            Fixed64.One,
            Fixed64.Zero,
            out Fixed64 distance));
        Assert.Equal(Fixed64.Two, distance);
    }

    [Fact]
    public void Sweep_ValidatesTheSemanticDistanceContract()
    {
        var box = new FixedOrientedBox(
            Vector3d.Zero,
            FixedQuaternion.Identity,
            Vector3d.One);

        Assert.Throws<ArgumentException>(() =>
            box.TryGetCircleSlabSweepDistance(
                new Vector3d(-5, 0, 0),
                Vector2d.One,
                Fixed64.One,
                Fixed64.One,
                Fixed64.One,
                out _));
        Assert.Throws<ArgumentOutOfRangeException>(() =>
            box.TryGetCircleSlabSweepDistance(
                new Vector3d(-5, 0, 0),
                Vector2d.Right,
                -Fixed64.One,
                Fixed64.One,
                Fixed64.One,
                out _));
        Assert.Throws<ArgumentOutOfRangeException>(() =>
            box.TryGetCircleSlabSweepDistance(
                new Vector3d(-5, 0, 0),
                Vector2d.Right,
                Fixed64.One,
                Fixed64.Zero,
                Fixed64.One,
                out _));
        Assert.Throws<ArgumentOutOfRangeException>(() =>
            box.TryGetCircleSlabSweepDistance(
                new Vector3d(-5, 0, 0),
                Vector2d.Right,
                Fixed64.One,
                Fixed64.One,
                -Fixed64.One,
                out _));
        Assert.Throws<ArgumentOutOfRangeException>(() =>
            box.GetCircleSlabSeparationLowerBound(
                Vector3d.Zero,
                Fixed64.Zero,
                Fixed64.One));
        Assert.Throws<ArgumentOutOfRangeException>(() =>
            box.GetCircleSlabSeparationLowerBound(
                Vector3d.Zero,
                Fixed64.One,
                -Fixed64.One));
    }

    [Fact]
    public void Sweep_WarmedPathDoesNotAllocate()
    {
        var box = new FixedOrientedBox(
            Vector3d.Zero,
            FixedQuaternion.FromEulerAnglesInDegrees(
                (Fixed64)13,
                (Fixed64)(-29),
                (Fixed64)7),
            new Vector3d(2, 3, 4));
        _ = box.TryGetCircleSlabSweepDistance(
            new Vector3d(-10, 0, 0),
            Vector2d.Right,
            (Fixed64)20,
            Fixed64.One,
            Fixed64.Half,
            out _);
        long before = GC.GetAllocatedBytesForCurrentThread();

        for (int iteration = 0; iteration < 64; iteration++)
        {
            _ = box.TryGetCircleSlabSweepDistance(
                new Vector3d(-10, 0, 0),
                Vector2d.Right,
                (Fixed64)20,
                Fixed64.One,
                Fixed64.Half,
                out _);
        }

        Assert.Equal(before, GC.GetAllocatedBytesForCurrentThread());
    }
}
