//=======================================================================
// FixedMassWeight.cs
//=======================================================================
// MIT License, Copyright (c) 2024-present David Oravsky (mrdav30)
// See LICENSE file in the project root for full license information.
//=======================================================================

using System;

namespace FixedMathSharp.Geometry;

/// <summary>
/// Represents a nonnegative relative mass-property measure without narrowing
/// products or aggregate sums to <see cref="Fixed64"/>.
/// </summary>
public readonly struct FixedMassWeight
{
    internal readonly Signed320 Numerator;

    internal FixedMassWeight(Signed320 numerator)
    {
        Numerator = numerator;
    }

    /// <summary>Gets the zero relative weight.</summary>
    public static FixedMassWeight Zero => default;

    /// <summary>Gets a unit relative weight.</summary>
    public static FixedMassWeight One =>
        WideMassProperties.CreateWeight(Fixed64.One);

    /// <summary>Gets whether this weight is exactly zero.</summary>
    public bool IsZero => Numerator.IsZero;

    /// <summary>Creates a weight from one nonnegative measure.</summary>
    public static FixedMassWeight FromMeasure(Fixed64 measure) =>
        WideMassProperties.CreateWeight(measure);

    /// <summary>Creates a weight from an exact product of nonnegative factors.</summary>
    public static FixedMassWeight FromProduct(
        Fixed64 first,
        Fixed64 second) =>
        WideMassProperties.CreateWeight(first, second);

    /// <summary>Creates a weight from an exact product of nonnegative factors.</summary>
    public static FixedMassWeight FromProduct(
        Fixed64 first,
        Fixed64 second,
        Fixed64 third) =>
        WideMassProperties.CreateWeight(first, second, third);

    /// <summary>Creates a weight from an exact product of nonnegative factors.</summary>
    public static FixedMassWeight FromProduct(
        Fixed64 first,
        Fixed64 second,
        Fixed64 third,
        Fixed64 fourth) =>
        WideMassProperties.CreateWeight(first, second, third, fourth);

    /// <summary>Adds another nonnegative weight without scalar narrowing.</summary>
    /// <exception cref="OverflowException">
    /// The exact aggregate exceeds the semantic weight domain.
    /// </exception>
    public FixedMassWeight Add(FixedMassWeight other)
    {
        if (!TryAdd(other, out FixedMassWeight result))
        {
            throw new OverflowException(
                "The aggregate mass-property weight is outside the semantic weight domain.");
        }

        return result;
    }

    /// <summary>
    /// Attempts to add another nonnegative weight without scalar narrowing.
    /// </summary>
    public bool TryAdd(
        FixedMassWeight other,
        out FixedMassWeight result) =>
        WideMassProperties.TryAdd(this, other, out result);

    /// <summary>
    /// Attempts to materialize the represented measure as <see cref="Fixed64"/>.
    /// </summary>
    public bool TryGetMeasure(out Fixed64 measure) =>
        WideMassProperties.TryGetMeasure(this, out measure);

    /// <summary>
    /// Attempts to distribute <paramref name="total"/> by this weight's exact
    /// share of <paramref name="totalWeight"/>.
    /// </summary>
    public bool TryGetProportionalShare(
        Fixed64 total,
        FixedMassWeight totalWeight,
        out Fixed64 share) =>
        WideMassProperties.TryGetProportionalShare(
            this,
            total,
            totalWeight,
            out share);

    /// <summary>
    /// Attempts to obtain the positive-weight average of scalar values with
    /// one final round-half-to-even conversion.
    /// </summary>
    public static bool TryGetWeightedAverage(
        ReadOnlySpan<Fixed64> values,
        ReadOnlySpan<FixedMassWeight> weights,
        out Fixed64 average) =>
        WideMassProperties.TryGetWeightedAverage(
            values,
            weights,
            out average);
}
