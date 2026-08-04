//=======================================================================
// FixedConvex2dSweepContracts.Tests.cs
//=======================================================================
// MIT License, Copyright (c) 2024-present David Oravsky (mrdav30)
// See LICENSE file in the project root for full license information.
//=======================================================================

using System;
using FixedMathSharp.Geometry;
using Xunit;

namespace FixedMathSharp.Tests;

public sealed class FixedConvex2dSweepContractsTests
{
    private static readonly Vector2d[] UnitBox =
    {
        new(-Fixed64.One, -Fixed64.One),
        new(Fixed64.One, -Fixed64.One),
        new(Fixed64.One, Fixed64.One),
        new(-Fixed64.One, Fixed64.One),
    };

    [Fact]
    public void SweepFamilies_ReportInitialOverlapAtZeroDistance()
    {
        Assert.True(FixedConvex2dRelations
            .TryGetSegmentFirstIntersectionDistance(
                Vector2d.Zero,
                Vector2d.Right,
                Fixed64.One,
                Vector2d.Zero,
                UnitBox,
                out Fixed64 segmentDistance,
                out Vector2d segmentNormal,
                out _));
        Assert.True(FixedConvex2dRelations
            .TryGetSegmentFirstIntersectionDistance(
                Vector2d.Zero,
                Vector2d.Right,
                Fixed64.One,
                Vector2d.Zero,
                Fixed64.PiOver4,
                UnitBox,
                out Fixed64 rotatedSegmentDistance,
                out Vector2d rotatedSegmentNormal,
                out _));
        Assert.True(FixedConvex2dRelations.TryGetSweptCircleFirstDistance(
            Vector2d.Zero,
            Fixed64.Half,
            Vector2d.Right,
            Fixed64.One,
            Vector2d.Zero,
            UnitBox,
            out Fixed64 circleDistance,
            out _,
            out _));
        Assert.True(FixedConvex2dRelations.TryGetSweptCircleFirstDistance(
            Vector2d.Zero,
            Fixed64.Half,
            Vector2d.Right,
            Fixed64.One,
            Vector2d.Zero,
            Fixed64.PiOver4,
            UnitBox,
            out Fixed64 rotatedCircleDistance,
            out _,
            out _));
        Assert.True(FixedConvex2dRelations
            .TryGetSweptCenteredCapsuleFirstDistance(
                Vector2d.Zero,
                Vector2d.Forward,
                Fixed64.One,
                Fixed64.Half,
                Vector2d.Right,
                Fixed64.One,
                Vector2d.Zero,
                UnitBox,
                out Fixed64 capsuleDistance,
                out _,
                out _));
        Assert.True(FixedConvex2dRelations
            .TryGetSweptCenteredCapsuleFirstDistance(
                Vector2d.Zero,
                Vector2d.Forward,
                Fixed64.One,
                Fixed64.Half,
                Vector2d.Right,
                Fixed64.One,
                Vector2d.Zero,
                Fixed64.PiOver4,
                UnitBox,
                out Fixed64 rotatedCapsuleDistance,
                out _,
                out _));
        Assert.True(FixedConvex2dRelations.TryGetSweptConvexFirstDistance(
            Vector2d.Zero,
            UnitBox,
            Vector2d.Right,
            Fixed64.One,
            Vector2d.Right * Fixed64.Half,
            UnitBox,
            out Fixed64 convexDistance,
            out _,
            out _));
        Assert.True(FixedConvex2dRelations.TryGetSweptConvexFirstDistance(
            Vector2d.Zero,
            Fixed64.PiOver4,
            UnitBox,
            Vector2d.Right,
            Fixed64.One,
            Vector2d.Right * Fixed64.Half,
            Fixed64.PiOver4,
            UnitBox,
            out Fixed64 rotatedConvexDistance,
            out _,
            out _));

        Assert.Equal(Fixed64.Zero, segmentDistance);
        Assert.Equal(-Vector2d.Right, segmentNormal);
        Assert.Equal(Fixed64.Zero, rotatedSegmentDistance);
        Assert.Equal(-Vector2d.Right, rotatedSegmentNormal);
        Assert.Equal(Fixed64.Zero, circleDistance);
        Assert.Equal(Fixed64.Zero, rotatedCircleDistance);
        Assert.Equal(Fixed64.Zero, capsuleDistance);
        Assert.Equal(Fixed64.Zero, rotatedCapsuleDistance);
        Assert.Equal(Fixed64.Zero, convexDistance);
        Assert.Equal(Fixed64.Zero, rotatedConvexDistance);
    }

    [Fact]
    public void RotatedSweepFamilies_ReturnFalseWhenMovingAway()
    {
        Vector2d farLeft = new((Fixed64)(-5), Fixed64.Zero);

        Assert.False(FixedConvex2dRelations
            .TryGetSegmentFirstIntersectionDistance(
                farLeft,
                Vector2d.Left,
                Fixed64.One,
                Vector2d.Zero,
                Fixed64.PiOver4,
                UnitBox,
                out _,
                out _,
                out _));
        Assert.False(FixedConvex2dRelations.TryGetSweptCircleFirstDistance(
            farLeft,
            Fixed64.Half,
            Vector2d.Left,
            Fixed64.One,
            Vector2d.Zero,
            Fixed64.PiOver4,
            UnitBox,
            out _,
            out _,
            out _));
        Assert.False(FixedConvex2dRelations
            .TryGetSweptCenteredCapsuleFirstDistance(
                farLeft,
                Vector2d.Forward,
                Fixed64.One,
                Fixed64.Half,
                Vector2d.Left,
                Fixed64.One,
                Vector2d.Zero,
                Fixed64.PiOver4,
                UnitBox,
                out _,
                out _,
                out _));
        Assert.False(FixedConvex2dRelations.TryGetSweptConvexFirstDistance(
            farLeft,
            Fixed64.PiOver4,
            UnitBox,
            Vector2d.Left,
            Fixed64.One,
            Vector2d.Zero,
            Fixed64.PiOver4,
            UnitBox,
            out _,
            out _,
            out _));
    }

    [Fact]
    public void SweepFamilies_RejectInvalidGeometryAndTravelContracts()
    {
        Assert.Equal(
            "circleRadius",
            Assert.Throws<ArgumentOutOfRangeException>(() =>
                FixedConvex2dRelations.TryGetSweptCircleFirstDistance(
                    Vector2d.Zero,
                    -Fixed64.One,
                    Vector2d.Right,
                    Fixed64.One,
                    Vector2d.Zero,
                    UnitBox,
                    out _,
                    out _,
                    out _)).ParamName);
        Assert.Equal(
            "circleRadius",
            Assert.Throws<ArgumentOutOfRangeException>(() =>
                FixedConvex2dRelations.TryGetSweptCircleFirstDistance(
                    Vector2d.Zero,
                    -Fixed64.One,
                    Vector2d.Right,
                    Fixed64.One,
                    Vector2d.Zero,
                    Fixed64.PiOver4,
                    UnitBox,
                    out _,
                    out _,
                    out _)).ParamName);
        Assert.Equal(
            "capsuleRadius",
            Assert.Throws<ArgumentOutOfRangeException>(() =>
                FixedConvex2dRelations
                    .TryGetSweptCenteredCapsuleFirstDistance(
                        Vector2d.Zero,
                        Vector2d.Forward,
                        Fixed64.One,
                        -Fixed64.One,
                        Vector2d.Right,
                        Fixed64.One,
                        Vector2d.Zero,
                        UnitBox,
                        out _,
                        out _,
                        out _)).ParamName);
        Assert.Equal(
            "capsuleRadius",
            Assert.Throws<ArgumentOutOfRangeException>(() =>
                FixedConvex2dRelations
                    .TryGetSweptCenteredCapsuleFirstDistance(
                        Vector2d.Zero,
                        Vector2d.Forward,
                        Fixed64.One,
                        -Fixed64.One,
                        Vector2d.Right,
                        Fixed64.One,
                        Vector2d.Zero,
                        Fixed64.PiOver4,
                        UnitBox,
                        out _,
                        out _,
                        out _)).ParamName);
        Assert.Equal(
            "direction",
            Assert.Throws<ArgumentException>(() =>
                FixedConvex2dRelations.TryGetSweptConvexFirstDistance(
                    Vector2d.Zero,
                    UnitBox,
                    Vector2d.One,
                    Fixed64.One,
                    Vector2d.Right * (Fixed64)4,
                    UnitBox,
                    out _,
                    out _,
                    out _)).ParamName);
        Assert.Equal(
            "maximumDistance",
            Assert.Throws<ArgumentOutOfRangeException>(() =>
                FixedConvex2dRelations.TryGetSweptConvexFirstDistance(
                    Vector2d.Zero,
                    UnitBox,
                    Vector2d.Right,
                    -Fixed64.One,
                    Vector2d.Right * (Fixed64)4,
                    UnitBox,
                    out _,
                    out _,
                    out _)).ParamName);
        Assert.Equal(
            "axisDirection",
            Assert.Throws<ArgumentException>(() =>
                FixedConvex2dRelations
                    .TryGetSweptCenteredCapsuleFirstDistance(
                        Vector2d.Zero,
                        Vector2d.One,
                        Fixed64.One,
                        Fixed64.One,
                        Vector2d.Right,
                        Fixed64.One,
                        Vector2d.Right * (Fixed64)4,
                        UnitBox,
                        out _,
                        out _,
                        out _)).ParamName);
        Assert.Equal(
            "axisLength",
            Assert.Throws<ArgumentOutOfRangeException>(() =>
                FixedConvex2dRelations
                    .TryGetSweptCenteredCapsuleFirstDistance(
                        Vector2d.Zero,
                        Vector2d.Forward,
                        -Fixed64.One,
                        Fixed64.One,
                        Vector2d.Right,
                        Fixed64.One,
                        Vector2d.Right * (Fixed64)4,
                        UnitBox,
                        out _,
                        out _,
                        out _)).ParamName);
    }
}
