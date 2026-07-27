//=======================================================================
// VectorWeightedAverage.Tests.cs
//=======================================================================
// MIT License, Copyright (c) 2024-present David Oravsky (mrdav30)
// See LICENSE file in the project root for full license information.
//=======================================================================

using System;
using Xunit;

namespace FixedMathSharp.Tests;

public sealed class VectorWeightedAverageTests
{
    [Fact]
    public void TryGetWeightedAverage_ShouldRetainFullDomainProductsAndCancellation()
    {
        Fixed64 positive = Fixed64.MaxValue - Fixed64.One;
        Fixed64 negative = Fixed64.MinValue + Fixed64.One;
        Fixed64 largeWeight = Fixed64.MaxValue - (Fixed64)2;
        Vector2d[] values2D =
        {
            new(positive, positive),
            new(negative, negative)
        };
        Vector3d[] values3D =
        {
            new(positive, positive, positive),
            new(negative, negative, negative)
        };
        Fixed64[] weights = { largeWeight, largeWeight };

        Assert.True(Vector2d.TryGetWeightedAverage(
            values2D,
            weights,
            out Vector2d average2D));
        Assert.True(Vector3d.TryGetWeightedAverage(
            values3D,
            weights,
            out Vector3d average3D));
        Assert.Equal(Vector2d.Zero, average2D);
        Assert.Equal(Vector3d.Zero, average3D);
    }

    [Fact]
    public void TryGetWeightedAverage_ShouldRoundFinalHalfTieToEven()
    {
        Fixed64 oneRaw = Fixed64.FromRaw(1);
        Fixed64 twoRaw = Fixed64.FromRaw(2);
        Fixed64[] weights = { Fixed64.One, Fixed64.One };

        Assert.True(Vector2d.TryGetWeightedAverage(
            new[] { Vector2d.Zero, new Vector2d(oneRaw, twoRaw) },
            weights,
            out Vector2d lowerEven));
        Assert.True(Vector3d.TryGetWeightedAverage(
            new[]
            {
                new Vector3d(oneRaw, oneRaw, oneRaw),
                new Vector3d(twoRaw, twoRaw, twoRaw)
            },
            weights,
            out Vector3d upperEven));

        Assert.Equal(new Vector2d(Fixed64.Zero, oneRaw), lowerEven);
        Assert.Equal(new Vector3d(twoRaw, twoRaw, twoRaw), upperEven);
    }

    [Fact]
    public void TryGetWeightedAverage_ShouldRejectMissingOrInvalidWeights()
    {
        Assert.False(Vector2d.TryGetWeightedAverage(
            new[] { Vector2d.One },
            new[] { Fixed64.Zero },
            out Vector2d average2D));
        Assert.Equal(Vector2d.Zero, average2D);
        Assert.False(Vector3d.TryGetWeightedAverage(
            Array.Empty<Vector3d>(),
            Array.Empty<Fixed64>(),
            out Vector3d average3D));
        Assert.Equal(Vector3d.Zero, average3D);

        Assert.Throws<ArgumentException>(() =>
            Vector2d.TryGetWeightedAverage(
                new[] { Vector2d.Zero },
                Array.Empty<Fixed64>(),
                out _));
        Assert.Throws<ArgumentException>(() =>
            Vector3d.TryGetWeightedAverage(
                new[] { Vector3d.Zero },
                Array.Empty<Fixed64>(),
                out _));
        Assert.Throws<ArgumentOutOfRangeException>(() =>
            Vector2d.TryGetWeightedAverage(
                new[] { Vector2d.Zero },
                new[] { -Fixed64.One },
                out _));
        Assert.Throws<ArgumentOutOfRangeException>(() =>
            Vector3d.TryGetWeightedAverage(
                new[] { Vector3d.Zero },
                new[] { -Fixed64.One },
                out _));
    }
}
