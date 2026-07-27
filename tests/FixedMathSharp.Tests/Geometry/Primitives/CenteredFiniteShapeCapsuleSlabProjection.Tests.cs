//=======================================================================
// CenteredFiniteShapeCapsuleSlabProjection.Tests.cs
//=======================================================================
// MIT License, Copyright (c) 2024-present David Oravsky (mrdav30)
// See LICENSE file in the project root for full license information.
//=======================================================================

using FixedMathSharp.Geometry;
using System;
using Xunit;

namespace FixedMathSharp.Tests;

public sealed class CenteredFiniteShapeCapsuleSlabProjectionTests
{
    [Fact]
    public void OrdinaryPlanarAxis_ShouldMatchAllFiniteShapeProjections()
    {
        Vector3d slabCenter =
            Vector3d.Right * Fixed64.FromFraction(3, 4);

        Assert.True(FixedSegment.TryGetCenteredCapsuleCapsuleSlabAxisPenetration(
            Vector3d.Right,
            Vector3d.Zero,
            Vector3d.Up,
            Fixed64.Two,
            Fixed64.Half,
            slabCenter,
            Vector2d.Forward,
            Fixed64.One,
            Fixed64.Half,
            Fixed64.Half,
            out Vector3d capsuleAxis,
            out Fixed64 capsuleDepth,
            out bool capsuleDepthIsClamped));
        Assert.True(FixedSegment.TryGetCenteredFiniteCylinderCapsuleSlabAxisPenetration(
            Vector3d.Right,
            Vector3d.Zero,
            Vector3d.Up,
            Fixed64.Two,
            Fixed64.Half,
            slabCenter,
            Vector2d.Forward,
            Fixed64.One,
            Fixed64.Half,
            Fixed64.Half,
            out Vector3d cylinderAxis,
            out Fixed64 cylinderDepth,
            out bool cylinderDepthIsClamped));
        Assert.True(FixedSegment.TryGetCenteredFiniteConeCapsuleSlabAxisPenetration(
            Vector3d.Right,
            Vector3d.Zero,
            Vector3d.Up,
            Fixed64.Two,
            Fixed64.Half,
            slabCenter,
            Vector2d.Forward,
            Fixed64.One,
            Fixed64.Half,
            Fixed64.Half,
            out Vector3d coneAxis,
            out Fixed64 coneDepth,
            out bool coneDepthIsClamped));

        Assert.Equal(Vector3d.Right, capsuleAxis);
        Assert.Equal(Vector3d.Right, cylinderAxis);
        Assert.Equal(Vector3d.Right, coneAxis);
        Assert.Equal(Fixed64.FromFraction(1, 4), capsuleDepth);
        Assert.Equal(capsuleDepth, cylinderDepth);
        Assert.Equal(capsuleDepth, coneDepth);
        Assert.False(capsuleDepthIsClamped);
        Assert.False(cylinderDepthIsClamped);
        Assert.False(coneDepthIsClamped);
    }

    [Theory]
    [InlineData(true)]
    [InlineData(false)]
    public void MirroredScalarFace_ShouldRetainCanonicalOverlap(bool positive)
    {
        Fixed64 shapeCoordinate = positive
            ? Fixed64.MaxValue - Fixed64.FromFraction(1, 4)
            : Fixed64.MinValue + Fixed64.FromFraction(1, 4);
        Fixed64 slabCoordinate =
            positive ? Fixed64.MaxValue : Fixed64.MinValue;
        Vector3d shapeCenter =
            Vector3d.Right * shapeCoordinate;
        Vector3d slabCenter =
            Vector3d.Right * slabCoordinate;
        Vector3d expectedAxis =
            positive ? Vector3d.Right : Vector3d.Left;
        Fixed64 expectedDepth = Fixed64.FromFraction(3, 4);

        Assert.True(FixedSegment.TryGetCenteredCapsuleCapsuleSlabAxisPenetration(
            Vector3d.Right,
            shapeCenter,
            Vector3d.Up,
            Fixed64.Two,
            Fixed64.Half,
            slabCenter,
            Vector2d.Forward,
            Fixed64.One,
            Fixed64.Half,
            Fixed64.Half,
            out Vector3d capsuleAxis,
            out Fixed64 capsuleDepth,
            out _));
        Assert.True(FixedSegment.TryGetCenteredFiniteCylinderCapsuleSlabAxisPenetration(
            Vector3d.Right,
            shapeCenter,
            Vector3d.Up,
            Fixed64.Two,
            Fixed64.Half,
            slabCenter,
            Vector2d.Forward,
            Fixed64.One,
            Fixed64.Half,
            Fixed64.Half,
            out Vector3d cylinderAxis,
            out Fixed64 cylinderDepth,
            out _));
        Assert.True(FixedSegment.TryGetCenteredFiniteConeCapsuleSlabAxisPenetration(
            Vector3d.Right,
            shapeCenter,
            Vector3d.Up,
            Fixed64.Two,
            Fixed64.Half,
            slabCenter,
            Vector2d.Forward,
            Fixed64.One,
            Fixed64.Half,
            Fixed64.Half,
            out Vector3d coneAxis,
            out Fixed64 coneDepth,
            out _));

        Assert.Equal(expectedAxis, capsuleAxis);
        Assert.Equal(expectedAxis, cylinderAxis);
        Assert.Equal(expectedAxis, coneAxis);
        Assert.Equal(expectedDepth, capsuleDepth);
        Assert.Equal(expectedDepth, cylinderDepth);
        Assert.Equal(expectedDepth, coneDepth);
    }

    [Fact]
    public void AxialConeProjection_ShouldSelectApexAndBaseFeatures()
    {
        Assert.True(FixedSegment.TryGetCenteredFiniteConeCapsuleSlabAxisPenetration(
            Vector3d.Up,
            Vector3d.Zero,
            Vector3d.Up,
            Fixed64.Two,
            Fixed64.Half,
            Vector3d.Up * Fixed64.FromFraction(5, 4),
            Vector2d.Forward,
            Fixed64.One,
            Fixed64.Half,
            Fixed64.Half,
            out Vector3d positiveAxis,
            out Fixed64 positiveDepth,
            out _));
        Assert.True(FixedSegment.TryGetCenteredFiniteConeCapsuleSlabAxisPenetration(
            Vector3d.Up,
            Vector3d.Zero,
            Vector3d.Up,
            Fixed64.Two,
            Fixed64.Half,
            -Vector3d.Up * Fixed64.FromFraction(5, 4),
            Vector2d.Forward,
            Fixed64.One,
            Fixed64.Half,
            Fixed64.Half,
            out Vector3d negativeAxis,
            out Fixed64 negativeDepth,
            out _));

        Assert.Equal(Vector3d.Up, positiveAxis);
        Assert.Equal(Vector3d.Down, negativeAxis);
        Assert.Equal(Fixed64.FromFraction(1, 4), positiveDepth);
        Assert.Equal(positiveDepth, negativeDepth);
    }

    [Fact]
    public void ReversedAndObliqueConeAxes_ShouldKeepExactSupportFeatures()
    {
        Vector3d upwardDiagonal =
            new Vector3d(Fixed64.One, Fixed64.One, Fixed64.Zero).Normalized;
        Vector3d downwardDiagonal =
            new Vector3d(Fixed64.One, -Fixed64.One, Fixed64.Zero).Normalized;

        Assert.True(FixedSegment.TryGetCenteredFiniteConeCapsuleSlabAxisPenetration(
            Vector3d.Down,
            Vector3d.Zero,
            Vector3d.Up,
            Fixed64.Two,
            Fixed64.Half,
            Vector3d.Zero,
            Vector2d.Forward,
            Fixed64.One,
            Fixed64.Half,
            Fixed64.Half,
            out _,
            out _,
            out _));
        Assert.True(FixedSegment.TryGetCenteredFiniteConeCapsuleSlabAxisPenetration(
            upwardDiagonal,
            Vector3d.Zero,
            Vector3d.Up,
            Fixed64.Half,
            Fixed64.Two,
            Vector3d.Zero,
            Vector2d.Forward,
            Fixed64.One,
            Fixed64.Half,
            Fixed64.Half,
            out _,
            out _,
            out _));
        Assert.True(FixedSegment.TryGetCenteredFiniteConeCapsuleSlabAxisPenetration(
            downwardDiagonal,
            Vector3d.Zero,
            Vector3d.Up,
            Fixed64.Half,
            Fixed64.Two,
            Vector3d.Zero,
            Vector2d.Forward,
            Fixed64.One,
            Fixed64.Half,
            Fixed64.Half,
            out Vector3d downwardAxis,
            out _,
            out _));

        Assert.Equal(-downwardDiagonal, downwardAxis);
    }

    [Fact]
    public void SeparatedProjection_ShouldReturnNoPenetration()
    {
        Assert.False(FixedSegment.TryGetCenteredCapsuleCapsuleSlabAxisPenetration(
            Vector3d.Right,
            Vector3d.Zero,
            Vector3d.Up,
            Fixed64.One,
            Fixed64.Half,
            Vector3d.Right * Fixed64.Two,
            Vector2d.Forward,
            Fixed64.One,
            Fixed64.Half,
            Fixed64.Half,
            out _,
            out _,
            out _));
        Assert.False(FixedSegment.TryGetCenteredFiniteConeCapsuleSlabAxisPenetration(
            Vector3d.Right,
            Vector3d.Zero,
            Vector3d.Up,
            Fixed64.One,
            Fixed64.Half,
            Vector3d.Right * Fixed64.Two,
            Vector2d.Forward,
            Fixed64.One,
            Fixed64.Half,
            Fixed64.Half,
            out _,
            out _,
            out _));
        Assert.False(FixedSegment.TryGetCenteredFiniteConeCapsuleSlabAxisPenetration(
            Vector3d.Right,
            Vector3d.Zero,
            Vector3d.Up,
            Fixed64.One,
            Fixed64.Half,
            -Vector3d.Right * Fixed64.Two,
            Vector2d.Forward,
            Fixed64.One,
            Fixed64.Half,
            Fixed64.Half,
            out _,
            out _,
            out _));
    }

    [Fact]
    public void InvalidGeometry_ShouldBeRejectedBeforeProjection()
    {
        Assert.Throws<ArgumentException>(() => CallCapsule(
            Vector3d.Zero,
            Vector3d.Up,
            Fixed64.One,
            Fixed64.Half,
            Vector2d.Forward,
            Fixed64.One,
            Fixed64.Half,
            Fixed64.Half));
        Assert.Throws<ArgumentException>(() => CallCapsule(
            Vector3d.Right,
            Vector3d.Zero,
            Fixed64.One,
            Fixed64.Half,
            Vector2d.Forward,
            Fixed64.One,
            Fixed64.Half,
            Fixed64.Half));
        Assert.Throws<ArgumentOutOfRangeException>(() => CallCapsule(
            Vector3d.Right,
            Vector3d.Up,
            -Fixed64.One,
            Fixed64.Half,
            Vector2d.Forward,
            Fixed64.One,
            Fixed64.Half,
            Fixed64.Half));
        Assert.Throws<ArgumentOutOfRangeException>(() =>
            FixedSegment.TryGetCenteredFiniteCylinderCapsuleSlabAxisPenetration(
                Vector3d.Right,
                Vector3d.Zero,
                Vector3d.Up,
                Fixed64.Zero,
                Fixed64.Half,
                Vector3d.Zero,
                Vector2d.Forward,
                Fixed64.One,
                Fixed64.Half,
                Fixed64.Half,
                out _,
                out _,
                out _));
        Assert.Throws<ArgumentOutOfRangeException>(() => CallCapsule(
            Vector3d.Right,
            Vector3d.Up,
            Fixed64.One,
            -Fixed64.One,
            Vector2d.Forward,
            Fixed64.One,
            Fixed64.Half,
            Fixed64.Half));
        Assert.Throws<ArgumentException>(() => CallCapsule(
            Vector3d.Right,
            Vector3d.Up,
            Fixed64.One,
            Fixed64.Half,
            Vector2d.Zero,
            Fixed64.One,
            Fixed64.Half,
            Fixed64.Half));
        Assert.Throws<ArgumentOutOfRangeException>(() => CallCapsule(
            Vector3d.Right,
            Vector3d.Up,
            Fixed64.One,
            Fixed64.Half,
            Vector2d.Forward,
            -Fixed64.One,
            Fixed64.Half,
            Fixed64.Half));
        Assert.Throws<ArgumentOutOfRangeException>(() => CallCapsule(
            Vector3d.Right,
            Vector3d.Up,
            Fixed64.One,
            Fixed64.Half,
            Vector2d.Forward,
            Fixed64.One,
            -Fixed64.One,
            Fixed64.Half));
        Assert.Throws<ArgumentOutOfRangeException>(() => CallCapsule(
            Vector3d.Right,
            Vector3d.Up,
            Fixed64.One,
            Fixed64.Half,
            Vector2d.Forward,
            Fixed64.One,
            Fixed64.Half,
            Fixed64.Zero));
    }

    private static void CallCapsule(
        Vector3d projectionAxis,
        Vector3d capsuleAxis,
        Fixed64 capsuleLength,
        Fixed64 capsuleRadius,
        Vector2d slabAxis,
        Fixed64 slabLength,
        Fixed64 slabRadius,
        Fixed64 slabHalfThickness) =>
        FixedSegment.TryGetCenteredCapsuleCapsuleSlabAxisPenetration(
            projectionAxis,
            Vector3d.Zero,
            capsuleAxis,
            capsuleLength,
            capsuleRadius,
            Vector3d.Zero,
            slabAxis,
            slabLength,
            slabRadius,
            slabHalfThickness,
            out _,
            out _,
            out _);
}
