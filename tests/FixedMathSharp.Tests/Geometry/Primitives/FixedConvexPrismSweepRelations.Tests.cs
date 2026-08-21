//=======================================================================
// FixedConvexPrismSweepRelations.Tests.cs
//=======================================================================
// MIT License, Copyright (c) 2024-present David Oravsky (mrdav30)
// See LICENSE file in the project root for full license information.
//=======================================================================

using System;
using Xunit;

namespace FixedMathSharp.Tests.Bounds;

public sealed class FixedConvexPrismSweepRelationsTests
{
    private static readonly Vector2d[] UnitSquare =
    {
        new(-Fixed64.Half, -Fixed64.Half),
        new(Fixed64.Half, -Fixed64.Half),
        new(Fixed64.Half, Fixed64.Half),
        new(-Fixed64.Half, Fixed64.Half),
    };

    [Fact]
    public void JointBoundaryOnlyContact_RequiresOneSharedStrictParameter()
    {
        Vector3d start = new(
            Fixed64.Zero,
            Fixed64.FromFraction(1, 4),
            Fixed64.Zero);
        Vector3d end = new(
            Fixed64.One,
            Fixed64.FromFraction(5, 4),
            Fixed64.Zero);
        Vector3d prismOrigin = Vector3d.Right;
        Fixed64 radius = Fixed64.FromFraction(1, 4);

        AssertSweepResult(
            expected: false,
            start,
            end,
            radius,
            Fixed64.One,
            prismOrigin,
            UnitSquare);
        AssertSweepResult(
            expected: true,
            start,
            end,
            radius + Fixed64.MinIncrement,
            Fixed64.One,
            prismOrigin,
            UnitSquare);
        AssertSweepResult(
            expected: true,
            start - Vector3d.Up * Fixed64.MinIncrement,
            end - Vector3d.Up * Fixed64.MinIncrement,
            radius,
            Fixed64.One,
            prismOrigin,
            UnitSquare);
    }

    [Fact]
    public void StationaryCylinder_RequiresStrictPlanarAndVerticalOverlap()
    {
        Vector3d overlappingBottom =
            -Vector3d.Up * Fixed64.FromFraction(1, 4);

        Assert.True(FixedConvexPrismRelations
            .IntersectsSweptUprightCylinderStrict(
                overlappingBottom,
                overlappingBottom,
                Fixed64.FromFraction(1, 4),
                Fixed64.One,
                Vector3d.Zero,
                Fixed64.Zero,
                UnitSquare,
                Fixed64.Half));
        Assert.False(FixedConvexPrismRelations
            .IntersectsSweptUprightCylinderStrict(
                new Vector3d(
                    Fixed64.FromFraction(3, 4),
                    overlappingBottom.Y,
                    Fixed64.Zero),
                new Vector3d(
                    Fixed64.FromFraction(3, 4),
                    overlappingBottom.Y,
                    Fixed64.Zero),
                Fixed64.FromFraction(1, 4),
                Fixed64.One,
                Vector3d.Zero,
                Fixed64.Zero,
                UnitSquare,
                Fixed64.Half));
        Assert.False(FixedConvexPrismRelations
            .IntersectsSweptUprightCylinderStrict(
                Vector3d.Up * Fixed64.Half,
                Vector3d.Up * Fixed64.Half,
                Fixed64.FromFraction(1, 4),
                Fixed64.One,
                Vector3d.Zero,
                Fixed64.Zero,
                UnitSquare,
                Fixed64.Half));
    }

    [Fact]
    public void ZeroRadius_RequiresTheAxisPointToEnterTheStrictFootprint()
    {
        Vector3d bottom = -Vector3d.Up * Fixed64.FromFraction(1, 4);
        var boundaryStart = new Vector3d(
            Fixed64.Half,
            bottom.Y,
            -Fixed64.FromFraction(1, 4));
        var boundaryEnd = new Vector3d(
            Fixed64.Half,
            bottom.Y,
            Fixed64.FromFraction(1, 4));

        Assert.False(FixedConvexPrismRelations
            .IntersectsSweptUprightCylinderStrict(
                boundaryStart,
                boundaryEnd,
                Fixed64.Zero,
                Fixed64.One,
                Vector3d.Zero,
                Fixed64.Zero,
                UnitSquare,
                Fixed64.Half));
        Assert.True(FixedConvexPrismRelations
            .IntersectsSweptUprightCylinderStrict(
                boundaryStart,
                new Vector3d(Fixed64.Zero, bottom.Y, Fixed64.Zero),
                Fixed64.Zero,
                Fixed64.One,
                Vector3d.Zero,
                Fixed64.Zero,
                UnitSquare,
                Fixed64.Half));
    }

    [Fact]
    public void OddRawHeight_PreservesTheAuthoredTopEndpoint()
    {
        Vector3d bottom =
            -Vector3d.Up * Fixed64.FromFraction(3, 2);

        Assert.False(FixedConvexPrismRelations
            .IntersectsSweptUprightCylinderStrict(
                bottom,
                bottom,
                Fixed64.Zero,
                Fixed64.One,
                Vector3d.Zero,
                Fixed64.Zero,
                UnitSquare,
                Fixed64.Half));
        Assert.True(FixedConvexPrismRelations
            .IntersectsSweptUprightCylinderStrict(
                bottom,
                bottom,
                Fixed64.Zero,
                Fixed64.One + Fixed64.MinIncrement,
                Vector3d.Zero,
                Fixed64.Zero,
                UnitSquare,
                Fixed64.Half));
    }

    [Fact]
    public void OrderedWindingAndRotatedFourAndSixVertexPrisms_Agree()
    {
        Vector2d[] rectangle =
        {
            new(-2, -1),
            new(2, -1),
            new(2, 1),
            new(-2, 1),
        };
        Vector2d[] reversedRectangle = Reverse(rectangle);
        Vector2d[] hexagon =
        {
            new(-2, 0),
            new(-1, -1),
            new(1, -1),
            new(2, 0),
            new(1, 1),
            new(-1, 1),
        };
        Vector2d[] reversedHexagon = Reverse(hexagon);
        Vector3d prismOrigin = new(3, 2, -5);
        Vector3d bottom = new(
            prismOrigin.X,
            prismOrigin.Y - Fixed64.Half,
            prismOrigin.Z);

        Assert.True(Intersects(
            bottom,
            bottom,
            Fixed64.Zero,
            Fixed64.One,
            prismOrigin,
            Fixed64.PiOver6,
            rectangle));
        Assert.True(Intersects(
            bottom,
            bottom,
            Fixed64.Zero,
            Fixed64.One,
            prismOrigin,
            Fixed64.PiOver6,
            reversedRectangle));
        Assert.True(Intersects(
            bottom,
            bottom,
            Fixed64.Zero,
            Fixed64.One,
            prismOrigin,
            -Fixed64.PiOver4,
            hexagon));
        Assert.True(Intersects(
            bottom,
            bottom,
            Fixed64.Zero,
            Fixed64.One,
            prismOrigin,
            -Fixed64.PiOver4,
            reversedHexagon));
    }

    [Fact]
    public void OffCenterAnisotropicFootprint_UsesTheAuthoredRotationForBothWindings()
    {
        Vector2d[] offCenterRectangle =
        {
            new(Fixed64.Zero, -Fixed64.FromFraction(1, 4)),
            new((Fixed64)4, -Fixed64.FromFraction(1, 4)),
            new((Fixed64)4, Fixed64.FromFraction(1, 4)),
            new(Fixed64.Zero, Fixed64.FromFraction(1, 4)),
        };
        Vector2d[] reversed = Reverse(offCenterRectangle);
        Vector3d bottom = new(
            Fixed64.Zero,
            -Fixed64.FromFraction(1, 4),
            Fixed64.Two);

        Assert.True(Intersects(
            bottom,
            bottom,
            Fixed64.Zero,
            Fixed64.One,
            Vector3d.Zero,
            Fixed64.HalfPi,
            offCenterRectangle));
        Assert.True(Intersects(
            bottom,
            bottom,
            Fixed64.Zero,
            Fixed64.One,
            Vector3d.Zero,
            Fixed64.HalfPi,
            reversed));
        Assert.False(Intersects(
            bottom,
            bottom,
            Fixed64.Zero,
            Fixed64.One,
            Vector3d.Zero,
            Fixed64.Zero,
            offCenterRectangle));
        Assert.False(Intersects(
            bottom,
            bottom,
            Fixed64.Zero,
            Fixed64.One,
            Vector3d.Zero,
            -Fixed64.HalfPi,
            offCenterRectangle));
        Assert.False(Intersects(
            bottom,
            bottom,
            Fixed64.Zero,
            Fixed64.One,
            Vector3d.Zero,
            Fixed64.Zero,
            reversed));
        Assert.False(Intersects(
            bottom,
            bottom,
            Fixed64.Zero,
            Fixed64.One,
            Vector3d.Zero,
            -Fixed64.HalfPi,
            reversed));
    }

    [Fact]
    public void VertexOnlyOverlap_DistinguishesRawTangencyAndFullDomainMotion()
    {
        Vector3d tangentBottom = new(
            Fixed64.Half + Fixed64.FromRaw(3),
            -Fixed64.FromFraction(1, 4),
            Fixed64.Half + Fixed64.FromRaw(4));

        Assert.False(Intersects(
            tangentBottom,
            tangentBottom,
            Fixed64.FromRaw(5),
            Fixed64.One,
            Vector3d.Zero,
            Fixed64.Zero,
            UnitSquare));
        Assert.True(Intersects(
            tangentBottom,
            tangentBottom,
            Fixed64.FromRaw(6),
            Fixed64.One,
            Vector3d.Zero,
            Fixed64.Zero,
            UnitSquare));

        Fixed64 nearMinimum = Fixed64.MinValue
            + Fixed64.One
            + Fixed64.FromRaw(6);
        Vector3d fullDomainStart = new(
            nearMinimum,
            tangentBottom.Y,
            Fixed64.MaxValue);
        Vector3d fullDomainEnd = new(
            Fixed64.MaxValue,
            tangentBottom.Y,
            nearMinimum);
        AssertSweepResult(
            expected: true,
            fullDomainStart,
            fullDomainEnd,
            Fixed64.FromRaw(4),
            Fixed64.One,
            Vector3d.Zero,
            UnitSquare);
    }

    [Fact]
    public void FullRawDomainAndExtremeOrigins_PreserveStrictIntersection()
    {
        Vector3d fullDomainStart = new(
            Fixed64.MinValue,
            -Fixed64.FromFraction(1, 4),
            Fixed64.Zero);
        Vector3d fullDomainEnd = new(
            Fixed64.MaxValue,
            -Fixed64.FromFraction(1, 4),
            Fixed64.Zero);

        AssertSweepResult(
            expected: true,
            fullDomainStart,
            fullDomainEnd,
            Fixed64.Zero,
            Fixed64.One,
            Vector3d.Zero,
            UnitSquare);

        Vector3d extremeOrigin = new(
            Fixed64.MaxValue,
            Fixed64.Zero,
            Fixed64.MinValue);
        Vector3d extremeBottom = new(
            extremeOrigin.X,
            -Fixed64.FromFraction(1, 4),
            extremeOrigin.Z);
        Assert.True(Intersects(
            extremeBottom,
            extremeBottom,
            Fixed64.Zero,
            Fixed64.One,
            extremeOrigin,
            Fixed64.PiOver6,
            UnitSquare));
    }

    [Fact]
    public void InvalidDimensionsAndPrismShape_Throw()
    {
        Assert.Equal(
            "radius",
            Assert.Throws<ArgumentOutOfRangeException>(() => Intersects(
                Vector3d.Zero,
                Vector3d.Zero,
                -Fixed64.MinIncrement,
                Fixed64.One,
                Vector3d.Zero,
                Fixed64.Zero,
                UnitSquare)).ParamName);
        Assert.Equal(
            "height",
            Assert.Throws<ArgumentOutOfRangeException>(() => Intersects(
                Vector3d.Zero,
                Vector3d.Zero,
                Fixed64.Zero,
                Fixed64.Zero,
                Vector3d.Zero,
                Fixed64.Zero,
                UnitSquare)).ParamName);
        Assert.Equal(
            "prismLocalOffsets",
            Assert.Throws<ArgumentException>(() => Intersects(
                Vector3d.Zero,
                Vector3d.Zero,
                Fixed64.Zero,
                Fixed64.One,
                Vector3d.Zero,
                Fixed64.Zero,
                UnitSquare.AsSpan(0, 2))).ParamName);
        Assert.Equal(
            "prismHalfThickness",
            Assert.Throws<ArgumentOutOfRangeException>(() =>
                FixedConvexPrismRelations
                    .IntersectsSweptUprightCylinderStrict(
                        Vector3d.Zero,
                        Vector3d.Zero,
                        Fixed64.Zero,
                        Fixed64.One,
                        Vector3d.Zero,
                        Fixed64.Zero,
                        UnitSquare,
                        Fixed64.Zero)).ParamName);
    }

    [Fact]
    public void WarmedSweep_AllocatesNoManagedMemory()
    {
        Vector3d start = new(
            Fixed64.Zero,
            Fixed64.FromFraction(1, 4),
            Fixed64.Zero);
        Vector3d end = new(
            Fixed64.One,
            Fixed64.FromFraction(5, 4),
            Fixed64.Zero);
        bool allIntersect = false;

        long allocated = FixedMathTestHelper.MeasureWarmedAllocations(() =>
        {
            allIntersect = true;
            for (int iteration = 0; iteration < 64; iteration++)
            {
                allIntersect &= Intersects(
                    start,
                    end,
                    Fixed64.FromFraction(1, 4)
                        + Fixed64.MinIncrement,
                    Fixed64.One,
                    Vector3d.Right,
                    Fixed64.Zero,
                    UnitSquare);
            }
        });

        Assert.True(allIntersect);
        Assert.Equal(0L, allocated);
    }

    private static void AssertSweepResult(
        bool expected,
        Vector3d start,
        Vector3d end,
        Fixed64 radius,
        Fixed64 height,
        Vector3d prismOrigin,
        ReadOnlySpan<Vector2d> prismOffsets)
    {
        Assert.Equal(
            expected,
            Intersects(
                start,
                end,
                radius,
                height,
                prismOrigin,
                Fixed64.Zero,
                prismOffsets));
        Assert.Equal(
            expected,
            Intersects(
                end,
                start,
                radius,
                height,
                prismOrigin,
                Fixed64.Zero,
                prismOffsets));
    }

    private static bool Intersects(
        Vector3d start,
        Vector3d end,
        Fixed64 radius,
        Fixed64 height,
        Vector3d prismOrigin,
        Fixed64 prismRotation,
        ReadOnlySpan<Vector2d> prismOffsets) =>
        FixedConvexPrismRelations
            .IntersectsSweptUprightCylinderStrict(
                start,
                end,
                radius,
                height,
                prismOrigin,
                prismRotation,
                prismOffsets,
                Fixed64.Half);

    private static Vector2d[] Reverse(ReadOnlySpan<Vector2d> offsets)
    {
        var reversed = new Vector2d[offsets.Length];
        for (int index = 0; index < offsets.Length; index++)
            reversed[index] = offsets[offsets.Length - index - 1];
        return reversed;
    }
}
