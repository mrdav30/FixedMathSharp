using System;
using Xunit;

namespace FixedMathSharp.Tests.Numerics.Vectors;

public sealed class Vector2dAverageTests
{
    [Fact]
    public void GetAverage_WithScalarFaceValues_ShouldAccumulateWithoutSaturation()
    {
        Fixed64 centerX = Fixed64.MaxValue - Fixed64.Two;
        Vector2d[] values =
        {
            new(centerX - Fixed64.One, -Fixed64.One),
            new(centerX + Fixed64.One, -Fixed64.One),
            new(centerX + Fixed64.One, Fixed64.One),
            new(centerX - Fixed64.One, Fixed64.One)
        };

        Vector2d average = Vector2d.GetAverage(values);

        Assert.Equal(new Vector2d(centerX, Fixed64.Zero), average);
    }

    [Fact]
    public void GetAverage_WithEmptyValues_ShouldRejectUndefinedMean()
    {
        Assert.Throws<ArgumentException>(
            () => Vector2d.GetAverage(ReadOnlySpan<Vector2d>.Empty));
    }

    [Fact]
    public void GetAverage_ShouldNotAllocateAfterWarmup()
    {
        Vector2d[] values =
        {
            new(-Fixed64.One, Fixed64.One),
            new(Fixed64.One, -Fixed64.One)
        };
        _ = Vector2d.GetAverage(values);

        long before = GC.GetAllocatedBytesForCurrentThread();
        for (int i = 0; i < 100; i++)
            _ = Vector2d.GetAverage(values);

        Assert.Equal(
            0L,
            GC.GetAllocatedBytesForCurrentThread() - before);
    }
}
