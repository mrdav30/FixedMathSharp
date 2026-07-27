//=======================================================================
// FixedConvex2dMassProperties.Tests.cs
//=======================================================================
// MIT License, Copyright (c) 2024-present David Oravsky (mrdav30)
// See LICENSE file in the project root for full license information.
//=======================================================================

using FixedMathSharp.Bounds;
using System;
using Xunit;

namespace FixedMathSharp.Tests.Bounds;

public sealed class FixedConvex2dMassPropertiesTests
{
    [Fact]
    public void AreaAndCentroid_PreserveExtremeSymmetricPolygon()
    {
        Fixed64 extent = Fixed64.MaxValue * Fixed64.FromFraction(3, 4);
        Vector2d[] vertices =
        {
            new(-extent, -extent),
            new(extent, -extent),
            new(extent, extent),
            new(-extent, extent),
        };

        Assert.True(FixedConvex2dRelations.TryGetAreaAndCentroid(
            vertices,
            out Fixed64 area,
            out Vector2d centroid));
        Assert.Equal(Fixed64.MaxValue, area);
        Assert.Equal(Vector2d.Zero, centroid);
    }

    [Fact]
    public void AreaAndCentroid_AreWindingIndependent()
    {
        Vector2d[] counterClockwise =
        {
            Vector2d.Zero,
            new((Fixed64)2, Fixed64.Zero),
            new((Fixed64)2, Fixed64.One),
            new(Fixed64.Zero, Fixed64.One),
        };
        Vector2d[] clockwise =
        {
            Vector2d.Zero,
            new(Fixed64.Zero, Fixed64.One),
            new((Fixed64)2, Fixed64.One),
            new((Fixed64)2, Fixed64.Zero),
        };

        Assert.True(FixedConvex2dRelations.TryGetAreaAndCentroid(
            counterClockwise,
            out Fixed64 firstArea,
            out Vector2d firstCentroid));
        Assert.True(FixedConvex2dRelations.TryGetAreaAndCentroid(
            clockwise,
            out Fixed64 secondArea,
            out Vector2d secondCentroid));
        Assert.Equal((Fixed64)2, firstArea);
        Assert.Equal(firstArea, secondArea);
        Assert.Equal(new Vector2d(Fixed64.One, Fixed64.Half), firstCentroid);
        Assert.Equal(firstCentroid, secondCentroid);
    }

    [Fact]
    public void AreaAndCentroid_RejectExactZeroArea()
    {
        Vector2d[] vertices =
        {
            Vector2d.Zero,
            Vector2d.Right,
            Vector2d.Right * Fixed64.Two,
        };

        Assert.False(FixedConvex2dRelations.TryGetAreaAndCentroid(
            vertices,
            out Fixed64 area,
            out Vector2d centroid));
        Assert.Equal(Fixed64.Zero, area);
        Assert.Equal(Vector2d.Zero, centroid);
    }

    [Fact]
    public void AreaAndCentroid_WarmedPathDoesNotAllocate()
    {
        Vector2d[] vertices =
        {
            Vector2d.Zero,
            Vector2d.Right,
            Vector2d.One,
        };
        _ = FixedConvex2dRelations.TryGetAreaAndCentroid(
            vertices,
            out _,
            out _);

        long before = GC.GetAllocatedBytesForCurrentThread();
        for (int i = 0; i < 128; i++)
        {
            _ = FixedConvex2dRelations.TryGetAreaAndCentroid(
                vertices,
                out _,
                out _);
        }
        long after = GC.GetAllocatedBytesForCurrentThread();

        Assert.Equal(0L, after - before);
    }
}
