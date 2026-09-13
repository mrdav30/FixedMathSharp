//=======================================================================
// FixedSegment2d.Separation.Tests.cs
//=======================================================================
// MIT License, Copyright (c) 2024-present David Oravsky (mrdav30)
// See LICENSE file in the project root for full license information.
//=======================================================================

using System;
using FixedMathSharp.Geometry;
using Xunit;

namespace FixedMathSharp.Tests;

public sealed class FixedSegment2dSeparationTests
{
    [Fact]
    public void TryGetCapsuleIntersectionParameterEnclosure_RoundsBothBoundsOutward()
    {
        FixedSegment2d query = new(
            Vector2d.Zero,
            new Vector2d(new Fixed64(3), Fixed64.Zero));
        FixedSegment2d capsuleAxis = new(
            new Vector2d(new Fixed64(3) / new Fixed64(2), Fixed64.Zero),
            new Vector2d(new Fixed64(3) / new Fixed64(2), Fixed64.Zero));

        Assert.True(query.TryGetCapsuleIntersectionParameterEnclosure(
            capsuleAxis,
            Fixed64.One,
            out Fixed64 entry,
            out Fixed64 exit));

        long scale = FixedMath.ONE_L;
        Assert.True(entry.m_rawValue * 6L <= scale);
        Assert.True((entry.m_rawValue + 1L) * 6L > scale);
        Assert.True(exit.m_rawValue * 6L >= 5L * scale);
        Assert.True((exit.m_rawValue - 1L) * 6L < 5L * scale);

        Assert.True(query.TryGetCapsuleIntersectionParameterEnclosure(
            query,
            Fixed64.One,
            out entry,
            out exit));
        Assert.Equal(Fixed64.Zero, entry);
        Assert.Equal(Fixed64.One, exit);

        FixedSegment2d far = new(
            new Vector2d(new Fixed64(10), Fixed64.Zero),
            new Vector2d(new Fixed64(10), Fixed64.Zero));
        Assert.False(query.TryGetCapsuleIntersectionParameterEnclosure(
            far,
            Fixed64.One,
            out _,
            out _));
        Assert.Throws<ArgumentOutOfRangeException>(() =>
            query.TryGetCapsuleIntersectionParameterEnclosure(
                capsuleAxis,
                -Fixed64.MinIncrement,
                out _,
                out _));

        bool allFound = true;
        long allocated = FixedMathTestHelper.MeasureWarmedAllocations(() =>
        {
            allFound = true;
            for (int i = 0; i < 256; i++)
            {
                allFound &= query.TryGetCapsuleIntersectionParameterEnclosure(
                    capsuleAxis,
                    Fixed64.One,
                    out _,
                    out _);
            }
        });

        Assert.True(allFound);
        Assert.Equal(0, allocated);
    }

    [Theory]
    [InlineData(0L, true)]
    [InlineData(1L, false)]
    public void IsDistanceAtLeast_AcceptsEqualityAndRejectsOneRawOutside(
        long thresholdRawOffset,
        bool expected)
    {
        FixedSegment2d path = new(
            new Vector2d(-Fixed64.One, Fixed64.One),
            new Vector2d(Fixed64.One, Fixed64.One));
        FixedSegment2d wall = new(
            new Vector2d(-Fixed64.One, Fixed64.Zero),
            new Vector2d(Fixed64.One, Fixed64.Zero));
        Fixed64 threshold = Fixed64.FromRaw(
            Fixed64.One.m_rawValue + thresholdRawOffset);

        Assert.Equal(expected, path.IsDistanceAtLeast(wall, threshold));
        Assert.Equal(expected, wall.IsDistanceAtLeast(path, threshold));
    }

    [Fact]
    public void IsDistanceAtLeast_HandlesCrossingTouchingAndCollinearOverlap()
    {
        Fixed64 positive = Fixed64.FromRaw(1L);
        FixedSegment2d horizontal = new(
            new Vector2d(-Fixed64.One, Fixed64.Zero),
            new Vector2d(Fixed64.One, Fixed64.Zero));
        FixedSegment2d crossing = new(
            new Vector2d(Fixed64.Zero, -Fixed64.One),
            new Vector2d(Fixed64.Zero, Fixed64.One));
        FixedSegment2d touching = new(
            new Vector2d(Fixed64.One, Fixed64.Zero),
            new Vector2d(Fixed64.Two, Fixed64.One));
        FixedSegment2d overlapping = new(
            new Vector2d(Fixed64.Zero, Fixed64.Zero),
            new Vector2d(Fixed64.Two, Fixed64.Zero));

        Assert.True(horizontal.IsDistanceAtLeast(crossing, Fixed64.Zero));
        Assert.False(horizontal.IsDistanceAtLeast(crossing, positive));
        Assert.False(horizontal.IsDistanceAtLeast(touching, positive));
        Assert.False(horizontal.IsDistanceAtLeast(overlapping, positive));
    }

    [Fact]
    public void IsDistanceAtLeast_HandlesPointSegmentAndTwoPointsExactly()
    {
        FixedSegment2d segment = new(
            new Vector2d(-Fixed64.One, Fixed64.Zero),
            new Vector2d(Fixed64.One, Fixed64.Zero));
        FixedSegment2d point = new(
            new Vector2d(Fixed64.Zero, Fixed64.Two),
            new Vector2d(Fixed64.Zero, Fixed64.Two));
        FixedSegment2d origin = new(Vector2d.Zero, Vector2d.Zero);
        FixedSegment2d threeFour = new(
            new Vector2d((Fixed64)3, (Fixed64)4),
            new Vector2d((Fixed64)3, (Fixed64)4));

        Assert.True(point.IsDistanceAtLeast(segment, Fixed64.Two));
        Assert.False(point.IsDistanceAtLeast(
            segment,
            Fixed64.FromRaw(Fixed64.Two.m_rawValue + 1L)));
        Assert.True(origin.IsDistanceAtLeast(threeFour, (Fixed64)5));
        Assert.False(origin.IsDistanceAtLeast(
            threeFour,
            Fixed64.FromRaw(((Fixed64)5).m_rawValue + 1L)));
    }

    [Fact]
    public void IsDistanceAtLeast_HandlesObliqueParallelSegmentsExactly()
    {
        FixedSegment2d first = new(
            Vector2d.Zero,
            new Vector2d((Fixed64)3, (Fixed64)4));
        FixedSegment2d second = new(
            new Vector2d((Fixed64)(-4), (Fixed64)3),
            new Vector2d((Fixed64)(-1), (Fixed64)7));

        Assert.True(first.IsDistanceAtLeast(second, (Fixed64)5));
        Assert.False(first.IsDistanceAtLeast(
            second,
            Fixed64.FromRaw(((Fixed64)5).m_rawValue + 1L)));
        Assert.Equal(
            first.IsDistanceAtLeast(second, (Fixed64)5),
            new FixedSegment2d(first.End, first.Start).IsDistanceAtLeast(
                new FixedSegment2d(second.End, second.Start),
                (Fixed64)5));
    }

    [Theory]
    [InlineData(false)]
    [InlineData(true)]
    public void IsDistanceAtLeast_PointAgainstFullDomainSegmentUsesInteriorProjection(bool reversed)
    {
        Vector2d start = new(Fixed64.MinValue, Fixed64.Zero);
        Vector2d end = new(Fixed64.MaxValue, Fixed64.Zero);
        FixedSegment2d segment = reversed ? new(end, start) : new(start, end);
        Vector2d position = new(Fixed64.Zero, Fixed64.One);
        FixedSegment2d point = new(position, position);

        // Both wall endpoints are far away, but the exact interior projection is one unit below.
        Assert.True(point.IsDistanceAtLeast(segment, Fixed64.One));
        Assert.True(segment.IsDistanceAtLeast(point, Fixed64.One));
        Assert.False(point.IsDistanceAtLeast(segment, Fixed64.One + Fixed64.MinIncrement));
        Assert.False(segment.IsDistanceAtLeast(point, Fixed64.One + Fixed64.MinIncrement));
    }

    [Theory]
    [InlineData(false)]
    [InlineData(true)]
    public void IsDistanceAtLeast_PointBeyondFiniteEndpointUsesEndpointDistance(bool reversed)
    {
        Vector2d start = new(Fixed64.MaxValue - (Fixed64)4, Fixed64.Zero);
        Vector2d end = new(Fixed64.MaxValue - (Fixed64)3, Fixed64.Zero);
        FixedSegment2d segment = reversed ? new(end, start) : new(start, end);
        Vector2d position = new(Fixed64.MaxValue, (Fixed64)4);
        FixedSegment2d point = new(position, position);

        // The supporting line is four units away; the finite endpoint is exactly five (3-4-5).
        Assert.True(point.IsDistanceAtLeast(segment, (Fixed64)5));
        Assert.True(segment.IsDistanceAtLeast(point, (Fixed64)5));
        Assert.False(point.IsDistanceAtLeast(segment, (Fixed64)5 + Fixed64.MinIncrement));
        Assert.False(segment.IsDistanceAtLeast(point, (Fixed64)5 + Fixed64.MinIncrement));
    }

    [Fact]
    public void IsDistanceAtLeast_HandlesExtremeRawCoordinatesWithoutNarrowing()
    {
        FixedSegment2d minimum = new(
            new Vector2d(Fixed64.MinValue, Fixed64.MinValue),
            new Vector2d(Fixed64.MinValue, Fixed64.MinValue));
        FixedSegment2d maximum = new(
            new Vector2d(Fixed64.MaxValue, Fixed64.MaxValue),
            new Vector2d(Fixed64.MaxValue, Fixed64.MaxValue));

        Assert.True(minimum.IsDistanceAtLeast(maximum, Fixed64.MaxValue));
        Assert.True(maximum.IsDistanceAtLeast(minimum, Fixed64.MaxValue));
    }

    [Theory]
    [InlineData(false, false)]
    [InlineData(false, true)]
    [InlineData(true, false)]
    [InlineData(true, true)]
    public void IsDistanceAtLeast_SkewSegmentsKeepTheNearestEndpointProjection(
        bool reverseFirst,
        bool reverseSecond)
    {
        FixedSegment2d first = new(new Vector2d(0, 0), new Vector2d(6, 8));
        FixedSegment2d second = new(new Vector2d(-1, 7), new Vector2d(-4, 11));
        if (reverseFirst)
            first = new(first.End, first.Start);
        if (reverseSecond)
            second = new(second.End, second.Start);

        // (-1, 7) projects to (3, 4): the perpendicular offset (-4, 3) has length five.
        // Both endpoints of the first segment are farther from the second segment.
        Assert.True(first.IsDistanceAtLeast(second, (Fixed64)5));
        Assert.True(second.IsDistanceAtLeast(first, (Fixed64)5));
        Assert.False(first.IsDistanceAtLeast(second, (Fixed64)5 + Fixed64.MinIncrement));
        Assert.False(second.IsDistanceAtLeast(first, (Fixed64)5 + Fixed64.MinIncrement));
    }

    [Theory]
    [InlineData(1L, false, false)]
    [InlineData(1L, false, true)]
    [InlineData(2L, true, false)]
    [InlineData(2L, true, true)]
    public void IsDistanceAtLeast_FullDomainDiagonalRetainsSubRawRadialSeparation(
        long pointYRaw,
        bool expected,
        bool reversed)
    {
        Vector2d start = new(Fixed64.MinValue, Fixed64.MinValue);
        Vector2d end = new(Fixed64.MaxValue, Fixed64.MaxValue);
        FixedSegment2d diagonal = reversed ? new(end, start) : new(start, end);
        Vector2d position = new(Fixed64.Zero, Fixed64.FromRaw(pointYRaw));
        FixedSegment2d point = new(position, position);

        // Distance to y=x is pointYRaw / sqrt(2) raw units: one is below one, two is above.
        Assert.Equal(expected, point.IsDistanceAtLeast(diagonal, Fixed64.MinIncrement));
        Assert.Equal(expected, diagonal.IsDistanceAtLeast(point, Fixed64.MinIncrement));
    }

    [Fact]
    public void IsDistanceAtLeast_RejectsNegativeMinimumDistance()
    {
        FixedSegment2d segment = new(Vector2d.Zero, Vector2d.One);

        Assert.Throws<ArgumentOutOfRangeException>(() =>
            segment.IsDistanceAtLeast(segment, Fixed64.FromRaw(-1L)));
    }

    [Fact]
    public void IsDistanceAtLeast_WarmedBatchAllocatesZero()
    {
        FixedSegment2d first = new(
            Vector2d.Zero,
            new Vector2d((Fixed64)3, (Fixed64)4));
        FixedSegment2d second = new(
            new Vector2d((Fixed64)(-4), (Fixed64)3),
            new Vector2d((Fixed64)(-1), (Fixed64)7));
        bool separated = false;

        long allocated = FixedMathTestHelper.MeasureWarmedAllocations(() =>
        {
            for (int i = 0; i < 256; i++)
                separated = first.IsDistanceAtLeast(second, (Fixed64)5);
        });

        Assert.True(separated);
        Assert.Equal(0L, allocated);
    }
}
