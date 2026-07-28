//=======================================================================
// FixedMassProperties.Tests.cs
//=======================================================================
// MIT License, Copyright (c) 2024-present David Oravsky (mrdav30)
// See LICENSE file in the project root for full license information.
//=======================================================================

using FixedMathSharp.Geometry;
using System;
using Xunit;

namespace FixedMathSharp.Tests.Geometry;

public sealed class FixedMassPropertiesTests
{
    [Fact]
    public void UniformTriangleShell_RetainsExactCombinedFirstMoment()
    {
        Vector3d[] vertices =
        {
            new(Fixed64.FromRaw(-4), Fixed64.Zero, Fixed64.Zero),
            new(Fixed64.FromRaw(-3), Fixed64.One, Fixed64.Zero),
            new(Fixed64.FromRaw(-3), Fixed64.Zero, Fixed64.One),
            new(Fixed64.FromRaw(-3), Fixed64.Zero, Fixed64.Zero),
            new(Fixed64.FromRaw(-2), Fixed64.One, Fixed64.Zero),
            new(Fixed64.FromRaw(-2), Fixed64.Zero, Fixed64.One)
        };
        int[] indices = { 0, 1, 2, 3, 4, 5 };
        var first = new FixedTriangle(
            vertices[0],
            vertices[1],
            vertices[2]);
        var second = new FixedTriangle(
            vertices[3],
            vertices[4],
            vertices[5]);

        Assert.True(FixedTriangle.TryGetUniformShellMassProperties(
            vertices,
            indices,
            out FixedMassWeight surfaceWeight,
            out Vector3d center,
            out Fixed3x3 tensor));
        Assert.True(first.AreaWeight.Add(second.AreaWeight).TryGetMeasure(
            out Fixed64 expectedArea));
        Assert.True(surfaceWeight.TryGetMeasure(out Fixed64 area));
        Assert.Equal(expectedArea, area);
        Assert.Equal(Fixed64.FromRaw(-3), center.X);
        Assert.Equal(Fixed64.FromFraction(1, 3), center.Y);
        Assert.Equal(Fixed64.FromFraction(1, 3), center.Z);
        Assert.True(tensor.M11 > Fixed64.Zero);
        Assert.Equal(tensor.M12, tensor.M21);
        Assert.Equal(tensor.M13, tensor.M31);
        Assert.Equal(tensor.M23, tensor.M32);
    }

    [Fact]
    public void UniformTriangleShell_MatchesAnalyticRightTriangleAndTranslation()
    {
        Vector3d[] vertices =
        {
            Vector3d.Zero,
            Vector3d.Right * Fixed64.Two,
            Vector3d.Up * Fixed64.Two
        };
        Vector3d translation = new(
            (Fixed64)100,
            (Fixed64)(-50),
            (Fixed64)25);
        Vector3d[] translatedVertices =
        {
            vertices[0] + translation,
            vertices[1] + translation,
            vertices[2] + translation
        };
        int[] indices = { 0, 1, 2 };

        Assert.True(FixedTriangle.TryGetUniformShellMassProperties(
            vertices,
            indices,
            out _,
            out Vector3d center,
            out Fixed3x3 tensor));
        Assert.True(FixedTriangle.TryGetUniformShellMassProperties(
            translatedVertices,
            indices,
            out _,
            out Vector3d translatedCenter,
            out Fixed3x3 translatedTensor));

        Assert.Equal(new Vector3d(
            Fixed64.FromFraction(2, 3),
            Fixed64.FromFraction(2, 3),
            Fixed64.Zero), center);
        Assert.Equal(center + translation, translatedCenter);
        Assert.Equal(tensor, translatedTensor);
        AssertRawNear(tensor.M11, Fixed64.FromFraction(2, 9));
        AssertRawNear(tensor.M22, Fixed64.FromFraction(2, 9));
        AssertRawNear(tensor.M33, Fixed64.FromFraction(4, 9));
        AssertRawNear(tensor.M12, Fixed64.FromFraction(1, 9));
        Assert.Equal(Fixed64.Zero, tensor.M13);
        Assert.Equal(Fixed64.Zero, tensor.M23);
    }

    [Fact]
    public void UniformTriangleShell_ValidatesTopologyAndRejectsMissingSurface()
    {
        Vector3d[] vertices =
        {
            Vector3d.Zero,
            Vector3d.Right,
            Vector3d.Up
        };

        Assert.Throws<ArgumentException>(
            () => FixedTriangle.TryGetUniformShellMassProperties(
                vertices,
                new[] { 0, 1 },
                out _,
                out _,
                out _));
        Assert.Throws<ArgumentOutOfRangeException>(
            () => FixedTriangle.TryGetUniformShellMassProperties(
                vertices,
                new[] { 3, 1, 2 },
                out _,
                out _,
                out _));
        Assert.Throws<ArgumentOutOfRangeException>(
            () => FixedTriangle.TryGetUniformShellMassProperties(
                vertices,
                new[] { 0, 3, 2 },
                out _,
                out _,
                out _));
        Assert.Throws<ArgumentOutOfRangeException>(
            () => FixedTriangle.TryGetUniformShellMassProperties(
                vertices,
                new[] { 0, 1, 3 },
                out _,
                out _,
                out _));
        Assert.False(FixedTriangle.TryGetUniformShellMassProperties(
            vertices,
            Array.Empty<int>(),
            out FixedMassWeight surfaceWeight,
            out Vector3d center,
            out Fixed3x3 tensor));
        Assert.Equal(default, surfaceWeight);
        Assert.Equal(default, center);
        Assert.Equal(default, tensor);
    }

    [Fact]
    public void UniformTriangleShell_RejectsOnlyFinalTensorOverflow()
    {
        Vector3d[] vertices =
        {
            new((Fixed64)(-50001), Fixed64.Zero, Fixed64.Zero),
            new((Fixed64)(-50000), Fixed64.Zero, Fixed64.Zero),
            new((Fixed64)(-50001), Fixed64.One, Fixed64.Zero),
            new((Fixed64)50000, Fixed64.Zero, Fixed64.Zero),
            new((Fixed64)50001, Fixed64.Zero, Fixed64.Zero),
            new((Fixed64)50000, Fixed64.One, Fixed64.Zero)
        };

        Assert.False(FixedTriangle.TryGetUniformShellMassProperties(
            vertices,
            new[] { 0, 1, 2, 3, 4, 5 },
            out FixedMassWeight surfaceWeight,
            out Vector3d center,
            out Fixed3x3 tensor));
        Assert.Equal(default, surfaceWeight);
        Assert.Equal(default, center);
        Assert.Equal(default, tensor);
    }

    [Fact]
    public void WideWeights_PreserveUnrepresentableRelativeMeasures()
    {
        Fixed64 extent = (Fixed64)1_500_000;
        FixedMassWeight first = FixedMassWeight.FromProduct(
            extent,
            extent,
            extent);
        FixedMassWeight second = FixedMassWeight.FromProduct(
            extent,
            extent,
            extent,
            Fixed64.Two);
        FixedMassWeight total = first.Add(second);

        Assert.False(first.TryGetMeasure(out _));
        Assert.True(first.TryGetProportionalShare(
            (Fixed64)3,
            total,
            out Fixed64 firstShare));
        Assert.True(second.TryGetProportionalShare(
            (Fixed64)3,
            total,
            out Fixed64 secondShare));
        Assert.Equal(Fixed64.One, firstShare);
        Assert.Equal(Fixed64.Two, secondShare);
    }

    [Fact]
    public void TriangleAreaWeights_PreserveUnrepresentableRelativeMeasures()
    {
        Fixed64 extent = (Fixed64)100_000;
        FixedMassWeight first = new FixedTriangle(
            Vector3d.Zero,
            new Vector3d(extent, Fixed64.Zero, Fixed64.Zero),
            new Vector3d(Fixed64.Zero, extent, Fixed64.Zero))
            .AreaWeight;
        FixedMassWeight second = new FixedTriangle(
            Vector3d.Zero,
            new Vector3d(extent * Fixed64.Two, Fixed64.Zero, Fixed64.Zero),
            new Vector3d(Fixed64.Zero, extent, Fixed64.Zero))
            .AreaWeight;
        FixedMassWeight total = first.Add(second);

        Assert.False(first.TryGetMeasure(out _));
        Assert.True(first.TryGetProportionalShare(
            (Fixed64)3,
            total,
            out Fixed64 firstShare));
        Assert.True(second.TryGetProportionalShare(
            (Fixed64)3,
            total,
            out Fixed64 secondShare));
        Assert.Equal(Fixed64.One, firstShare);
        Assert.Equal(Fixed64.Two, secondShare);
    }

    [Fact]
    public void WeightedScalars_RoundOnceAfterWideWeighting()
    {
        Fixed64[] values =
        {
            Fixed64.One,
            (Fixed64)3,
        };
        FixedMassWeight[] weights =
        {
            FixedMassWeight.One,
            FixedMassWeight.One.Add(FixedMassWeight.One),
        };

        Assert.True(FixedMassWeight.TryGetWeightedAverage(
            values,
            weights,
            out Fixed64 average));
        Assert.Equal(Fixed64.FromFraction(7, 3), average);
    }

    [Fact]
    public void WeightedMassPoints3d_CancelOutsideTheScalarDomain()
    {
        FixedMassPoint negative = FixedMassPoint.CreateScaledLocalComposition(
            new Vector3d(Fixed64.MinValue, Fixed64.Zero, Fixed64.Zero),
            new Vector3d(Fixed64.Two, Fixed64.One, Fixed64.One),
            Vector3d.Zero,
            Vector3d.One,
            Vector3d.Zero,
            FixedQuaternion.Identity);
        FixedMassPoint positive = FixedMassPoint.CreateScaledLocalComposition(
            new Vector3d(Fixed64.MaxValue, Fixed64.Zero, Fixed64.Zero),
            new Vector3d(Fixed64.Two, Fixed64.One, Fixed64.One),
            Vector3d.Zero,
            Vector3d.One,
            Vector3d.Zero,
            FixedQuaternion.Identity);
        FixedMassPoint[] points = { negative, positive };
        FixedMassWeight[] weights =
        {
            FixedMassWeight.One,
            FixedMassWeight.One,
        };

        Assert.False(negative.TryGetPoint(out _));
        Assert.False(positive.TryGetPoint(out _));
        Assert.True(FixedMassPoint.TryGetWeightedAverage(
            points,
            weights,
            out Vector3d average));
        Assert.Equal(
            new Vector3d(Fixed64.FromRaw(-1), Fixed64.Zero, Fixed64.Zero),
            average);
    }

    [Fact]
    public void WeightedMassPoints2d_CancelOutsideTheScalarDomain()
    {
        FixedMassPoint2d negative = FixedMassPoint2d.CreateScaledLocalComposition(
            new Vector2d(Fixed64.MinValue, Fixed64.Zero),
            new Vector2d(Fixed64.Two, Fixed64.One),
            Vector2d.Zero,
            Vector2d.One,
            Vector2d.Zero,
            Fixed64.Zero);
        FixedMassPoint2d positive = FixedMassPoint2d.CreateScaledLocalComposition(
            new Vector2d(Fixed64.MaxValue, Fixed64.Zero),
            new Vector2d(Fixed64.Two, Fixed64.One),
            Vector2d.Zero,
            Vector2d.One,
            Vector2d.Zero,
            Fixed64.Zero);
        FixedMassPoint2d[] points = { negative, positive };
        FixedMassWeight[] weights =
        {
            FixedMassWeight.One,
            FixedMassWeight.One,
        };

        Assert.False(negative.TryGetPoint(out _));
        Assert.False(positive.TryGetPoint(out _));
        Assert.True(FixedMassPoint2d.TryGetWeightedAverage(
            points,
            weights,
            out Vector2d average));
        Assert.Equal(
            new Vector2d(Fixed64.FromRaw(-1), Fixed64.Zero),
            average);
    }

    [Fact]
    public void ParallelAxis3d_RetainsWidePointUntilTheFinalTensor()
    {
        FixedMassPoint point = FixedMassPoint.CreateScaledLocalComposition(
            new Vector3d((Fixed64)1_500_000_000, Fixed64.Zero, Fixed64.Zero),
            new Vector3d(Fixed64.Two, Fixed64.One, Fixed64.One),
            Vector3d.Zero,
            Vector3d.One,
            Vector3d.Zero,
            FixedQuaternion.Identity);

        Assert.False(point.TryGetPoint(out _));
        Assert.True(point.TryAddParallelAxisTensor(
            Fixed3x3.Zero,
            Fixed64.FromRaw(1),
            Vector3d.Zero,
            out Fixed3x3 tensor));
        Assert.Equal(Fixed64.Zero, tensor.M11);
        Assert.Equal(
            Fixed64.FromRaw(9_000_000_000_000_000_000L),
            tensor.M22);
        Assert.Equal(tensor.M22, tensor.M33);

        Assert.False(point.TryAddParallelAxisTensor(
            Fixed3x3.Zero,
            Fixed64.FromRaw(2),
            Vector3d.Zero,
            out _));
    }

    [Fact]
    public void ParallelAxis2d_RetainsWidePointUntilTheFinalMoment()
    {
        FixedMassPoint2d point = FixedMassPoint2d.CreateScaledLocalComposition(
            new Vector2d((Fixed64)1_500_000_000, Fixed64.Zero),
            new Vector2d(Fixed64.Two, Fixed64.One),
            Vector2d.Zero,
            Vector2d.One,
            Vector2d.Zero,
            Fixed64.Zero);

        Assert.False(point.TryGetPoint(out _));
        Assert.True(point.TryAddParallelAxisMoment(
            Fixed64.Zero,
            Fixed64.FromRaw(1),
            Vector2d.Zero,
            out Fixed64 moment));
        Assert.Equal(
            Fixed64.FromRaw(9_000_000_000_000_000_000L),
            moment);

        Assert.False(point.TryAddParallelAxisMoment(
            Fixed64.Zero,
            Fixed64.FromRaw(2),
            Vector2d.Zero,
            out _));
    }

    [Fact]
    public void SemanticMassInputs_RejectInvalidContracts()
    {
        Assert.Throws<ArgumentOutOfRangeException>(
            () => FixedMassWeight.FromMeasure(-Fixed64.One));
        Assert.Throws<ArgumentException>(
            () => FixedMassPoint.CreateScaledLocalComposition(
                Vector3d.Zero,
                Vector3d.One,
                Vector3d.Zero,
                Vector3d.One,
                Vector3d.Zero,
                default));

        FixedMassPoint[] points = { FixedMassPoint.FromPoint(Vector3d.Zero) };
        Assert.Throws<ArgumentException>(
            () => FixedMassPoint.TryGetWeightedAverage(
                points,
                Array.Empty<FixedMassWeight>(),
                out _));
        Assert.False(FixedMassPoint.TryGetWeightedAverage(
            points,
            new[] { FixedMassWeight.Zero },
            out _));
    }

    [Fact]
    public void MassPoint3d_MatchesScaleInvariantQuaternionComposition()
    {
        FixedQuaternion rotation =
            FixedQuaternion.FromEulerAnglesInDegrees(
                (Fixed64)17,
                (Fixed64)29,
                (Fixed64)43);
        Vector3d displacement = new(
            (Fixed64)1_000_000_000,
            (Fixed64)(-500_000_000),
            (Fixed64)250_000_000);

        Assert.NotEqual(Fixed64.One, rotation.MagnitudeSquared);
        Assert.True(Vector3d.TryComposeScaledLocalPoints(
            Vector3d.Zero,
            Vector3d.One,
            Vector3d.Zero,
            Vector3d.One,
            displacement,
            rotation,
            out Vector3d expected));
        FixedMassPoint point =
            FixedMassPoint.CreateScaledLocalComposition(
                Vector3d.Zero,
                Vector3d.One,
                Vector3d.Zero,
                Vector3d.One,
                displacement,
                rotation);

        Assert.True(point.TryGetPoint(out Vector3d actual));
        Assert.Equal(expected, actual);
    }

    [Fact]
    public void WideWeightAddition_RejectsSemanticOverflow()
    {
        FixedMassWeight weight = FixedMassWeight.One;
        bool overflowed = false;
        for (int i = 0; i < 256; i++)
        {
            if (weight.TryAdd(weight, out FixedMassWeight doubled))
            {
                weight = doubled;
                continue;
            }

            overflowed = true;
            Assert.Throws<OverflowException>(() => weight.Add(weight));
            break;
        }

        Assert.True(overflowed);
    }

    [Fact]
    public void WeightShares_RejectInvalidTotalsAndOversubscribedWeights()
    {
        FixedMassWeight weight =
            FixedMassWeight.FromProduct(Fixed64.Two, Fixed64.Two);
        FixedMassWeight smallerTotal = FixedMassWeight.One;

        Assert.True(FixedMassWeight.Zero.IsZero);
        Assert.False(weight.TryGetProportionalShare(
            -Fixed64.One,
            weight,
            out _));
        Assert.False(weight.TryGetProportionalShare(
            Fixed64.One,
            FixedMassWeight.Zero,
            out _));
        Assert.False(weight.TryGetProportionalShare(
            Fixed64.One,
            smallerTotal,
            out _));
    }

    [Fact]
    public void WeightedAverages_RejectZeroWeightAndUnrepresentableResults()
    {
        FixedMassWeight[] zeroWeight = { FixedMassWeight.Zero };
        Assert.False(FixedMassWeight.TryGetWeightedAverage(
            new[] { Fixed64.One },
            zeroWeight,
            out _));
        FixedMassPoint2d representablePoint =
            FixedMassPoint2d.FromPoint(Vector2d.One);
        Assert.True(representablePoint.TryGetPoint(out Vector2d materialized));
        Assert.Equal(Vector2d.One, materialized);
        Assert.False(FixedMassPoint2d.TryGetWeightedAverage(
            new[] { representablePoint },
            zeroWeight,
            out _));

        FixedMassPoint2d point2d =
            FixedMassPoint2d.CreateScaledLocalComposition(
                new Vector2d(Fixed64.MaxValue, Fixed64.Zero),
                new Vector2d(Fixed64.Two, Fixed64.One),
                Vector2d.Zero,
                Vector2d.One,
                Vector2d.Zero,
                Fixed64.Zero);
        Assert.False(point2d.TryGetPoint(out _));
        Assert.False(FixedMassPoint2d.TryGetWeightedAverage(
            new[] { point2d },
            new[] { FixedMassWeight.One },
            out _));

        FixedMassPoint point3d =
            FixedMassPoint.CreateScaledLocalComposition(
                new Vector3d(
                    Fixed64.MaxValue,
                    Fixed64.Zero,
                    Fixed64.Zero),
                new Vector3d(
                    Fixed64.Two,
                    Fixed64.One,
                    Fixed64.One),
                Vector3d.Zero,
                Vector3d.One,
                Vector3d.Zero,
                FixedQuaternion.Identity);
        Assert.False(FixedMassPoint.TryGetWeightedAverage(
            new[] { point3d },
            new[] { FixedMassWeight.One },
            out _));
    }

    [Fact]
    public void ParallelAxis3d_HandlesNonPositiveMassAndFinalAdditionOverflow()
    {
        FixedMassPoint point = FixedMassPoint.FromPoint(Vector3d.Right);
        Fixed3x3 centerTensor = new(
            Fixed64.Zero, Fixed64.Zero, Fixed64.Zero,
            Fixed64.Zero, Fixed64.MaxValue, Fixed64.Zero,
            Fixed64.Zero, Fixed64.Zero, Fixed64.Zero);

        Assert.False(point.TryAddParallelAxisTensor(
            Fixed3x3.Zero,
            -Fixed64.One,
            Vector3d.Zero,
            out _));
        Assert.True(point.TryAddParallelAxisTensor(
            centerTensor,
            Fixed64.Zero,
            Vector3d.Zero,
            out Fixed3x3 zeroMassTensor));
        Assert.Equal(centerTensor, zeroMassTensor);
        Assert.False(point.TryAddParallelAxisTensor(
            centerTensor,
            Fixed64.One,
            Vector3d.Zero,
            out _));
    }

    [Fact]
    public void ParallelAxis2d_HandlesNonPositiveMassAndFinalAdditionOverflow()
    {
        FixedMassPoint2d point =
            FixedMassPoint2d.FromPoint(Vector2d.Right);

        Assert.False(point.TryAddParallelAxisMoment(
            Fixed64.Zero,
            -Fixed64.One,
            Vector2d.Zero,
            out _));
        Assert.True(point.TryAddParallelAxisMoment(
            Fixed64.MaxValue,
            Fixed64.Zero,
            Vector2d.Zero,
            out Fixed64 zeroMassMoment));
        Assert.Equal(Fixed64.MaxValue, zeroMassMoment);
        Assert.False(point.TryAddParallelAxisMoment(
            Fixed64.MaxValue,
            Fixed64.One,
            Vector2d.Zero,
            out _));
    }

    private static void AssertRawNear(
        Fixed64 actual,
        Fixed64 expected)
    {
        const long RawTolerance = 2;
        Assert.InRange(
            actual.m_rawValue,
            expected.m_rawValue - RawTolerance,
            expected.m_rawValue + RawTolerance);
    }
}
