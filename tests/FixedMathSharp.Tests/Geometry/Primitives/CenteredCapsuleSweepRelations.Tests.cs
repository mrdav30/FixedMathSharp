//=======================================================================
// CenteredCapsuleSweepRelations.Tests.cs
//=======================================================================
// MIT License, Copyright (c) 2024–present David Oravsky (mrdav30)
// See LICENSE file in the project root for full license information.
//=======================================================================

using FixedMathSharp.Bounds;
using System;
using Xunit;

namespace FixedMathSharp.Tests;

public sealed class CenteredCapsuleSweepRelationsTests
{
    [Fact]
    public void SegmentSweep_ReturnsEarliestSideDistanceAndNormal()
    {
        Assert.True(FixedSegment2d.TryGetSweptCenteredCapsuleSegmentFirstDistance(
            new Vector2d(-5, 0),
            Vector2d.Forward,
            Fixed64.Two,
            Fixed64.One,
            Vector2d.Right,
            (Fixed64)10,
            new FixedSegment2d(new Vector2d(0, -3), new Vector2d(0, 3)),
            Fixed64.Zero,
            out Fixed64 distance,
            out Vector2d normal));
        Assert.Equal((Fixed64)4, distance);
        Assert.Equal(Vector2d.Right, normal);
    }

    [Theory]
    [InlineData(false)]
    [InlineData(true)]
    public void SegmentSweep_IgnoresUnrepresentableMoverEndpointAtScalarFace(bool maximumFace)
    {
        Fixed64 face = maximumFace ? Fixed64.MaxValue : Fixed64.MinValue;
        Fixed64 inward = maximumFace ? -Fixed64.One : Fixed64.One;
        Vector2d direction = maximumFace ? Vector2d.Left : Vector2d.Right;
        Fixed64 targetX = face + inward * (Fixed64)3;

        Assert.True(FixedSegment2d.TryGetSweptCenteredCapsuleSegmentFirstDistance(
            new Vector2d(face, Fixed64.Zero),
            direction,
            Fixed64.Two,
            Fixed64.One,
            direction,
            Fixed64.Two,
            new FixedSegment2d(
                new Vector2d(targetX, -Fixed64.One),
                new Vector2d(targetX, Fixed64.One)),
            Fixed64.Zero,
            out Fixed64 distance,
            out Vector2d normal));
        Assert.Equal(Fixed64.One, distance);
        Assert.Equal(direction, normal);
    }

    [Fact]
    public void CapsuleSweep_ReturnsEarliestCombinedRadiusDistance()
    {
        Assert.True(FixedSegment2d.TryGetSweptCenteredCapsulesFirstDistance(
            new Vector2d(-5, 0),
            Vector2d.Forward,
            (Fixed64)4,
            Fixed64.One,
            Vector2d.Right,
            (Fixed64)10,
            Vector2d.Zero,
            Vector2d.Forward,
            (Fixed64)4,
            Fixed64.One,
            out Fixed64 distance,
            out Vector2d normal));
        Assert.Equal((Fixed64)3, distance);
        Assert.Equal(Vector2d.Right, normal);
    }

    [Fact]
    public void CapsuleSweep_PreservesOddRawAxisLengthsAndStableNegativeEndpointOrder()
    {
        Fixed64 odd = Fixed64.FromRaw(3L);

        Assert.True(FixedSegment2d.TryGetSweptCenteredCapsulesFirstDistance(
            new Vector2d(Fixed64.FromRaw(-10L), Fixed64.Zero),
            Vector2d.Forward,
            odd,
            Fixed64.FromRaw(1L),
            Vector2d.Right,
            Fixed64.FromRaw(20L),
            new Vector2d(Fixed64.FromRaw(2L), Fixed64.Zero),
            Vector2d.Forward,
            odd,
            Fixed64.FromRaw(1L),
            out Fixed64 distance,
            out Vector2d normal));
        Assert.Equal(Fixed64.FromRaw(10L), distance);
        Assert.Equal(Vector2d.Right, normal);
    }

    [Fact]
    public void CapsuleSweep_ReturnsFalseForMissAndRejectsInvalidDirection()
    {
        Assert.False(FixedSegment2d.TryGetSweptCenteredCapsulesFirstDistance(
            Vector2d.Zero,
            Vector2d.Forward,
            Fixed64.Two,
            Fixed64.One,
            Vector2d.Right,
            Fixed64.One,
            new Vector2d(0, 5),
            Vector2d.Forward,
            Fixed64.Two,
            Fixed64.One,
            out Fixed64 distance,
            out Vector2d normal));
        Assert.Equal(Fixed64.Zero, distance);
        Assert.Equal(Vector2d.Zero, normal);

        Assert.Equal(
            "direction",
            Assert.Throws<ArgumentException>(() =>
                FixedSegment2d.TryGetSweptCenteredCapsulesFirstDistance(
                    Vector2d.Zero,
                    Vector2d.Forward,
                    Fixed64.Two,
                    Fixed64.One,
                    Vector2d.One,
                    Fixed64.One,
                    new Vector2d(4, 0),
                    Vector2d.Forward,
                    Fixed64.Two,
                    Fixed64.One,
                    out _,
                    out _)).ParamName);
    }

    [Fact]
    public void CapsuleSweep_RejectsInvalidRadiiAndDistance()
    {
        FixedSegment2d target = new(Vector2d.Zero, Vector2d.Forward);

        Assert.Equal(
            "radius",
            Assert.Throws<ArgumentOutOfRangeException>(() =>
                FixedSegment2d
                    .TryGetSweptCenteredCapsuleSegmentFirstDistance(
                        Vector2d.Zero,
                        Vector2d.Forward,
                        Fixed64.One,
                        -Fixed64.One,
                        Vector2d.Right,
                        Fixed64.One,
                        target,
                        Fixed64.Zero,
                        out _,
                        out _)).ParamName);
        Assert.Equal(
            "maximumDistance",
            Assert.Throws<ArgumentOutOfRangeException>(() =>
                FixedSegment2d
                    .TryGetSweptCenteredCapsuleSegmentFirstDistance(
                        Vector2d.Zero,
                        Vector2d.Forward,
                        Fixed64.One,
                        Fixed64.One,
                        Vector2d.Right,
                        -Fixed64.One,
                        target,
                        Fixed64.Zero,
                        out _,
                        out _)).ParamName);
        Assert.Equal(
            "targetRadius",
            Assert.Throws<ArgumentOutOfRangeException>(() =>
                FixedSegment2d
                    .TryGetSweptCenteredCapsuleSegmentFirstDistance(
                        Vector2d.Zero,
                        Vector2d.Forward,
                        Fixed64.One,
                        Fixed64.One,
                        Vector2d.Right,
                        Fixed64.One,
                        target,
                        -Fixed64.One,
                        out _,
                        out _)).ParamName);
    }

    [Fact]
    public void ZeroDistanceSweep_ReportsCrossingAxesAsInitialOverlap()
    {
        Assert.True(FixedSegment2d.TryGetSweptCenteredCapsulesFirstDistance(
            Vector2d.Zero,
            Vector2d.Right,
            Fixed64.Two,
            Fixed64.Zero,
            Vector2d.Right,
            Fixed64.Zero,
            Vector2d.Zero,
            Vector2d.Forward,
            Fixed64.Two,
            Fixed64.Zero,
            out Fixed64 distance,
            out Vector2d normal));

        Assert.Equal(Fixed64.Zero, distance);
        Assert.Equal(Vector2d.Forward, normal);
    }

    [Fact]
    public void CapsuleSweep_DetectsTargetEndpointEnteringMoverSide()
    {
        Assert.True(FixedSegment2d.TryGetSweptCenteredCapsulesFirstDistance(
            new Vector2d(Fixed64.Zero, (Fixed64)(-5)),
            Vector2d.Right,
            (Fixed64)10,
            Fixed64.Zero,
            Vector2d.Forward,
            (Fixed64)10,
            Vector2d.Zero,
            Vector2d.Forward,
            Fixed64.Two,
            Fixed64.Zero,
            out Fixed64 distance,
            out Vector2d normal));

        Assert.Equal((Fixed64)4, distance);
        Assert.Equal(Vector2d.Forward, normal);
    }

    [Fact]
    public void SegmentSweep_UsesTravelNormalForDegenerateTargetAxis()
    {
        Assert.True(
            FixedSegment2d.TryGetSweptCenteredCapsuleSegmentFirstDistance(
                new Vector2d((Fixed64)(-5), Fixed64.Zero),
                Vector2d.Forward,
                Fixed64.Two,
                Fixed64.One,
                Vector2d.Right,
                (Fixed64)10,
                new FixedSegment2d(Vector2d.Zero, Vector2d.Zero),
                Fixed64.Zero,
                out Fixed64 distance,
                out Vector2d normal));

        Assert.Equal((Fixed64)4, distance);
        Assert.Equal(Vector2d.Right, normal);
    }

    [Fact]
    public void PointCapsuleSweep_UsesTravelNormalAtCoincidentImpact()
    {
        Assert.True(FixedSegment2d.TryGetSweptCenteredCapsulesFirstDistance(
            new Vector2d(-5, 0),
            Vector2d.Forward,
            Fixed64.Zero,
            Fixed64.Zero,
            Vector2d.Right,
            (Fixed64)10,
            Vector2d.Zero,
            Vector2d.Forward,
            Fixed64.Zero,
            Fixed64.Zero,
            out Fixed64 distance,
            out Vector2d normal));

        Assert.Equal((Fixed64)5, distance);
        Assert.Equal(Vector2d.Right, normal);
    }

    [Fact]
    public void ZeroDistanceSweep_RecognizesEveryCollinearEndpointOrdering()
    {
        AssertInitialAxisIntersection(
            Vector2d.Zero,
            Vector2d.Right,
            Fixed64.Two,
            new Vector2d(Fixed64.Zero, Fixed64.Half),
            Vector2d.Forward,
            Fixed64.One);
        AssertInitialAxisIntersection(
            Vector2d.Zero,
            Vector2d.Right,
            Fixed64.Two,
            new Vector2d(Fixed64.Zero, -Fixed64.Half),
            Vector2d.Forward,
            Fixed64.One);
        AssertInitialAxisIntersection(
            Vector2d.Zero,
            Vector2d.Right,
            Fixed64.Two,
            new Vector2d(-Fixed64.One, Fixed64.Zero),
            Vector2d.Forward,
            Fixed64.Two);
        AssertInitialAxisIntersection(
            Vector2d.Zero,
            Vector2d.Right,
            Fixed64.Two,
            new Vector2d(Fixed64.One, Fixed64.Zero),
            Vector2d.Forward,
            Fixed64.Two);
    }

    private static void AssertInitialAxisIntersection(
        Vector2d firstCenter,
        Vector2d firstAxis,
        Fixed64 firstLength,
        Vector2d secondCenter,
        Vector2d secondAxis,
        Fixed64 secondLength)
    {
        Assert.True(FixedSegment2d.TryGetSweptCenteredCapsulesFirstDistance(
            firstCenter,
            firstAxis,
            firstLength,
            Fixed64.Zero,
            Vector2d.Right,
            Fixed64.Zero,
            secondCenter,
            secondAxis,
            secondLength,
            Fixed64.Zero,
            out Fixed64 distance,
            out _));
        Assert.Equal(Fixed64.Zero, distance);
    }
}
