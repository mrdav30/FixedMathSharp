//=======================================================================
// FixedConvex2dPlanarSweep.Tests.cs
//=======================================================================
// MIT License, Copyright (c) 2024-present David Oravsky (mrdav30)
// See LICENSE file in the project root for full license information.
//=======================================================================

using System;
using FixedMathSharp.Geometry;
using System.Numerics;
using Xunit;

namespace FixedMathSharp.Tests.Bounds;

public sealed class FixedConvex2dPlanarSweepTests
{
    private static readonly Vector2d[] Square =
    {
        new(-Fixed64.Half, -Fixed64.Half), new(Fixed64.Half, -Fixed64.Half),
        new(Fixed64.Half, Fixed64.Half), new(-Fixed64.Half, Fixed64.Half),
    };

    [Theory]
    [InlineData(0, 0, 0, 0, 0, 0, true)]
    [InlineData(2, 0, 2, 0, 0, 0, false)]
    [InlineData(-2, 0, 2, 0, 0, 0, true)]
    [InlineData(0, -2, 0, 2, 0, 0, true)]
    [InlineData(-2, -2, 2, 2, 0, 0, true)]
    [InlineData(-2, 2, 2, 2, 2, 0, false)]
    [InlineData(-2, 0, 2, 0, 4, 0, true)]
    [InlineData(0, 0, 0, 0, 2, 1, true)]
    [InlineData(3, 3, 3, 3, 2, 1, false)]
    public void Sweep_HandlesContainmentCrossingAndSeparation(
        int sx, int sy, int ex, int ey, int length, int radius, bool expected)
    {
        AssertSweep(expected, new(sx, sy), new(ex, ey), (Fixed64)length, (Fixed64)radius);
    }

    [Theory]
    [InlineData(-1, true)]
    [InlineData(0, false)]
    [InlineData(1, false)]
    public void SideContact_DistinguishesOneRawPenetrationFromTangency(long adjustment, bool expected)
    {
        Fixed64 x = Fixed64.One + Fixed64.FromRaw(adjustment);
        AssertSweep(expected, new(x, (Fixed64)(-2)), new(x, (Fixed64)2), Fixed64.One, Fixed64.Half);
    }

    [Fact]
    public void RoundedCorner_IsNotAnExpandedBox()
    {
        Vector2d[] box = { new(-1, -1), new(0, -1), new(0, 0), new(-1, 0) };
        Vector2d center = new(3, 4);
        AssertSweep(false, center, center, Fixed64.Zero, (Fixed64)5, box);
        AssertSweep(true, center, center, Fixed64.Zero, (Fixed64)5 + Fixed64.MinIncrement, box);
        AssertSweep(false, new(1, 1), new(1, 1), Fixed64.Zero, Fixed64.One, box);
    }

    [Fact]
    public void CapsuleMiddle_IsIncludedWhenBothEndCirclesMiss()
    {
        Vector2d[] smallBox =
        {
            new(-Fixed64.One / 8, -Fixed64.One / 8), new(Fixed64.One / 8, -Fixed64.One / 8),
            new(Fixed64.One / 8, Fixed64.One / 8), new(-Fixed64.One / 8, Fixed64.One / 8),
        };
        AssertSweep(true, new(-2, 0), new(2, 0), (Fixed64)2, Fixed64.Half, smallBox);
        AssertSweep(false, new(-2, 1), new(2, 1), Fixed64.Zero, Fixed64.Half, smallBox);
        AssertSweep(false, new(-2, -1), new(2, -1), Fixed64.Zero, Fixed64.Half, smallBox);
    }

    [Fact]
    public void ZeroRadius_RequiresStrictPolygonInterior()
    {
        AssertSweep(false, new(Fixed64.Half, -Fixed64.One), new(Fixed64.Half, Fixed64.One), Fixed64.One, Fixed64.Zero);
        AssertSweep(true, new(Fixed64.Half - Fixed64.MinIncrement, -Fixed64.One),
            new(Fixed64.Half - Fixed64.MinIncrement, Fixed64.One), Fixed64.One, Fixed64.Zero);
        AssertSweep(false, new(Fixed64.Half, Fixed64.Half), new(Fixed64.Half, Fixed64.Half), Fixed64.Zero, Fixed64.Zero);
    }

    [Fact]
    public void OddRawAxisLength_PreservesConceptualHalfRawEnds()
    {
        Vector2d center = new(Fixed64.Zero, Fixed64.Half + Fixed64.MinIncrement);
        AssertSweep(false, center, center, Fixed64.FromRaw(1), Fixed64.Zero);
        AssertSweep(false, center, center, Fixed64.FromRaw(2), Fixed64.Zero);
        AssertSweep(true, center, center, Fixed64.FromRaw(3), Fixed64.Zero);
        AssertSweep(true, center, center, Fixed64.FromRaw(1), Fixed64.MinIncrement);
    }

    [Fact]
    public void FullDomainChordAndConceptualPolygon_DoNotSaturate()
    {
        AssertSweep(true, new(Fixed64.MinValue, Fixed64.MinValue),
            new(Fixed64.MaxValue, Fixed64.MaxValue), Fixed64.MaxValue, Fixed64.MinIncrement);
        Vector2d center = new(Fixed64.MaxValue, Fixed64.MaxValue);
        Vector2d[] beyondDomain = { new(1, 1), new(2, 1), new(2, 2), new(1, 2) };
        Assert.False(FixedConvex2dRelations.IntersectsSweptUprightCapsuleStrict(
            center, center, Fixed64.Zero, Fixed64.One, center, beyondDomain));
        Assert.True(FixedConvex2dRelations.IntersectsSweptUprightCapsuleStrict(
            center, center, Fixed64.Zero, (Fixed64)2, center, beyondDomain));
    }

    [Fact]
    public void RepeatedAndCollinearEdges_PreserveConvexInterior()
    {
        Vector2d[] redundant = { new(0, 0), new(1, 0), new(2, 0), new(2, 0), new(2, 2), new(0, 2) };
        AssertSweep(true, new(1, 1), new(1, 1), Fixed64.Zero, Fixed64.Zero, redundant);
        AssertSweep(false, new(-1, 1), new(-1, 1), Fixed64.Zero, Fixed64.One, redundant);
        Vector2d[] collinear = { new(-1, 0), new(0, 0), new(1, 0) };
        AssertSweep(false, Vector2d.Zero, Vector2d.Zero, Fixed64.One, Fixed64.One, collinear);
    }

    [Fact]
    public void DiagonalCore_RequiresItsOwnSeparatingAxes()
    {
        // y=x-2 passes each square face's supporting line but misses the square.
        AssertSweep(false, new(-2, -4), new(4, 2), Fixed64.Zero, Fixed64.Zero);
        AssertSweep(false, new(-2, -4), new(4, 2), Fixed64.Zero, Fixed64.Half);
        AssertSweep(true, new(-2, -4), new(4, 2), Fixed64.Zero, Fixed64.One);
        // Rounded upper cap: bottom of radius-one cap exactly touches y=1/2.
        AssertSweep(false, new(0, 2), new(0, 2), Fixed64.One, Fixed64.One);
        AssertSweep(true, new(Fixed64.Zero, (Fixed64)2 - Fixed64.MinIncrement),
            new(Fixed64.Zero, (Fixed64)2 - Fixed64.MinIncrement), Fixed64.One, Fixed64.One);
    }

    [Theory]
    [InlineData(-1, true)]
    [InlineData(0, true)]
    [InlineData(1, false)]
    public void ClosedCapsuleContact_AdmitsExactThreeFourFiveCorner(long radiusAdjustment, bool expected)
    {
        Vector2d[] box = { new(-1, -1), new(0, -1), new(0, 0), new(-1, 0) };
        Fixed64 radius = (Fixed64)5 - Fixed64.FromRaw(radiusAdjustment);
        Assert.Equal(expected, FixedConvex2dRelations.IntersectsUprightCapsule(
            new(3, 4), Fixed64.Zero, radius, Vector2d.Zero, box));
    }

    [Fact]
    public void ClosedCapsuleContact_PreservesZeroRadiusAndOddAxis()
    {
        Assert.True(FixedConvex2dRelations.IntersectsUprightCapsule(
            new(Fixed64.Half, Fixed64.Half), Fixed64.Zero, Fixed64.Zero, Vector2d.Zero, Square));
        Vector2d center = new(Fixed64.Zero, Fixed64.Half + Fixed64.MinIncrement);
        Assert.False(FixedConvex2dRelations.IntersectsUprightCapsule(
            center, Fixed64.FromRaw(1), Fixed64.Zero, Vector2d.Zero, Square));
        Assert.True(FixedConvex2dRelations.IntersectsUprightCapsule(
            center, Fixed64.FromRaw(2), Fixed64.Zero, Vector2d.Zero, Square));
        Assert.True(FixedConvex2dRelations.IntersectsUprightCapsule(
            center, Fixed64.FromRaw(3), Fixed64.Zero, Vector2d.Zero, Square));
    }

    [Fact]
    public void StationaryCapsules_MatchIndependentExactRectangleDistance()
    {
        long unit = Fixed64.One.m_rawValue;
        long[] coordinates = { long.MinValue, -unit, -1, 0, 1, unit, long.MaxValue };
        long[] dimensions = { 0, 1, 3, unit, long.MaxValue };
        foreach (long x in coordinates)
        foreach (long y in coordinates)
        foreach (long length in dimensions)
        foreach (long radius in dimensions)
        {
            // A stationary upright axis against a unit square has independent
            // horizontal and vertical gaps. BigInteger is test-only oracle math.
            BigInteger horizontal = BigInteger.Abs((BigInteger)x * 2) - unit;
            BigInteger vertical = BigInteger.Abs((BigInteger)y * 2) - unit - length;
            bool expected = radius == 0
                ? horizontal < 0 && vertical < 0
                : BigInteger.Pow(BigInteger.Max(horizontal, 0), 2)
                    + BigInteger.Pow(BigInteger.Max(vertical, 0), 2)
                    < BigInteger.Pow((BigInteger)radius * 2, 2);
            Vector2d center = new(Fixed64.FromRaw(x), Fixed64.FromRaw(y));
            bool actual = FixedConvex2dRelations.IntersectsSweptUprightCapsuleStrict(
                center, center, Fixed64.FromRaw(length), Fixed64.FromRaw(radius), Vector2d.Zero, Square);
            Assert.Equal(expected, actual);
        }
    }

    [Fact]
    public void RepeatedSweep_AllocatesNothingAndPreservesClassification()
    {
        int hits = 0;
        long allocated = FixedMathTestHelper.MeasureWarmedAllocations(() =>
        {
            hits = 0;
            for (int i = 0; i < 128; i++)
            {
                if (FixedConvex2dRelations.IntersectsSweptUprightCapsuleStrict(
                    new(-2, 0), new(2, 0), Fixed64.One, Fixed64.Half, Vector2d.Zero, Square))
                    hits++;
                if (FixedConvex2dRelations.IntersectsSweptUprightCapsuleStrict(
                    new(-2, -4), new(4, 2), Fixed64.Zero, Fixed64.Half, Vector2d.Zero, Square))
                    hits++;
            }
        });
        Assert.Equal(128, hits);
        Assert.Equal(0, allocated);
    }

    [Fact]
    public void InvalidDimensionsAndTooFewVertices_Reject()
    {
        Assert.Throws<ArgumentOutOfRangeException>(() => FixedConvex2dRelations.IntersectsSweptUprightCapsuleStrict(
            Vector2d.Zero, Vector2d.Zero, -Fixed64.One, Fixed64.One, Vector2d.Zero, Square));
        Assert.Throws<ArgumentOutOfRangeException>(() => FixedConvex2dRelations.IntersectsSweptUprightCapsuleStrict(
            Vector2d.Zero, Vector2d.Zero, Fixed64.One, -Fixed64.One, Vector2d.Zero, Square));
        Assert.Throws<ArgumentException>(() => FixedConvex2dRelations.IntersectsSweptUprightCapsuleStrict(
            Vector2d.Zero, Vector2d.Zero, Fixed64.One, Fixed64.One, Vector2d.Zero, Array.Empty<Vector2d>()));
    }

    private static void AssertSweep(bool expected, Vector2d start, Vector2d end,
        Fixed64 axisLength, Fixed64 radius, Vector2d[]? offsets = null)
    {
        Vector2d[] polygon = (Vector2d[])(offsets ?? Square).Clone();
        for (int winding = 0; winding < 2; winding++)
        {
            Assert.Equal(expected, FixedConvex2dRelations.IntersectsSweptUprightCapsuleStrict(
                start, end, axisLength, radius, Vector2d.Zero, polygon));
            Assert.Equal(expected, FixedConvex2dRelations.IntersectsSweptUprightCapsuleStrict(
                end, start, axisLength, radius, Vector2d.Zero, polygon));
            Array.Reverse(polygon);
        }
    }
}
