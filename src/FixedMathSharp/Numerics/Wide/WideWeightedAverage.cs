//=======================================================================
// WideWeightedAverage.cs
//=======================================================================
// MIT License, Copyright (c) 2024-present David Oravsky (mrdav30)
// See LICENSE file in the project root for full license information.
//=======================================================================

using System;

namespace FixedMathSharp;

/// <summary>
/// Provides methods for computing the weighted average of two-dimensional and 
/// three-dimensional vectors using wide arithmetic to avoid overflow and maintain precision.
/// </summary>
internal static class WideWeightedAverage
{
    // A maximum-length span needs at most 159 signed bits for weighted
    // component products and 95 for the total Q32.32 weight.

    internal static Vector2d GetAverage(ReadOnlySpan<Vector2d> values)
    {
        Signed192 totalX = default;
        Signed192 totalY = default;
        for (int i = 0; i < values.Length; i++)
        {
            totalX = WideArithmetic.AddSigned192(
                totalX,
                Signed192.Raw(values[i].X));
            totalY = WideArithmetic.AddSigned192(
                totalY,
                Signed192.Raw(values[i].Y));
        }

        Signed576 denominator = Signed576.ExtendValue(
            Signed320.ExtendValue(
                Signed192.Signed(values.Length)));
        return new Vector2d(
            GetComponent(totalX, denominator),
            GetComponent(totalY, denominator));
    }

    internal static bool TryGet(
        ReadOnlySpan<Vector2d> values,
        ReadOnlySpan<Fixed64> weights,
        out Vector2d average)
    {
        Signed192 totalWeight = default;
        Signed320 weightedX = default;
        Signed320 weightedY = default;
        for (int i = 0; i < values.Length; i++)
        {
            Signed192 weight = Signed192.Raw(weights[i]);
            totalWeight = WideArithmetic.AddSigned192(
                totalWeight,
                weight);
            weightedX = WideArithmetic.AddSigned320(
                weightedX,
                WideArithmetic.MultiplySigned192(
                    Signed192.Raw(values[i].X),
                    weight));
            weightedY = WideArithmetic.AddSigned320(
                weightedY,
                WideArithmetic.MultiplySigned192(
                    Signed192.Raw(values[i].Y),
                    weight));
        }

        if (totalWeight.Sign == 0)
        {
            average = default;
            return false;
        }

        Signed576 denominator = Signed576.ExtendValue(
            Signed320.ExtendValue(totalWeight));
        Fixed64 x = GetComponent(
                weightedX,
                denominator);
        Fixed64 y = GetComponent(
                weightedY,
                denominator);
        average = new Vector2d(x, y);
        return true;
    }

    internal static bool TryGet(
        ReadOnlySpan<Vector3d> values,
        ReadOnlySpan<Fixed64> weights,
        out Vector3d average)
    {
        Signed192 totalWeight = default;
        Signed320 weightedX = default;
        Signed320 weightedY = default;
        Signed320 weightedZ = default;
        for (int i = 0; i < values.Length; i++)
        {
            Signed192 weight = Signed192.Raw(weights[i]);
            totalWeight = WideArithmetic.AddSigned192(
                totalWeight,
                weight);
            weightedX = WideArithmetic.AddSigned320(
                weightedX,
                WideArithmetic.MultiplySigned192(
                    Signed192.Raw(values[i].X),
                    weight));
            weightedY = WideArithmetic.AddSigned320(
                weightedY,
                WideArithmetic.MultiplySigned192(
                    Signed192.Raw(values[i].Y),
                    weight));
            weightedZ = WideArithmetic.AddSigned320(
                weightedZ,
                WideArithmetic.MultiplySigned192(
                    Signed192.Raw(values[i].Z),
                    weight));
        }

        if (totalWeight.Sign == 0)
        {
            average = default;
            return false;
        }

        Signed576 denominator = Signed576.ExtendValue(
            Signed320.ExtendValue(totalWeight));
        Fixed64 x = GetComponent(
                weightedX,
                denominator);
        Fixed64 y = GetComponent(
                weightedY,
                denominator);
        Fixed64 z = GetComponent(
                weightedZ,
                denominator);
        average = new Vector3d(x, y, z);
        return true;
    }

    private static Fixed64 GetComponent(
        Signed320 numerator,
        Signed576 denominator)
    {
        // A non-negative weighted average is bounded by its representable
        // inputs, so only the generic ratio helper's representable result is
        // reachable here.
        Fixed64.TryGetSignedRawRatio(
            Signed576.ExtendValue(numerator),
            denominator,
            out Fixed64 component);
        return component;
    }

    private static Fixed64 GetComponent(
        Signed192 numerator,
        Signed576 denominator)
    {
        // An arithmetic mean is bounded by its representable inputs.
        Fixed64.TryGetSignedRawRatio(
            Signed576.ExtendValue(
                Signed320.ExtendValue(numerator)),
            denominator,
            out Fixed64 component);
        return component;
    }

    internal static void ValidateInputs(
        int valueCount,
        ReadOnlySpan<Fixed64> weights)
    {
        if (valueCount != weights.Length)
        {
            throw new ArgumentException(
                "Values and weights must have the same length.",
                nameof(weights));
        }

        for (int i = 0; i < weights.Length; i++)
        {
            if (weights[i] < Fixed64.Zero)
            {
                throw new ArgumentOutOfRangeException(
                    nameof(weights),
                    "Weights cannot be negative.");
            }
        }
    }
}
