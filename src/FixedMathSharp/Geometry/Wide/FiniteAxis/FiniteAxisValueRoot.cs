//=======================================================================
// MIT License, Copyright (c) 2024-present David Oravsky (mrdav30)
// See LICENSE file in the project root for full license information.
//=======================================================================
using System;

namespace FixedMathSharp.Geometry;

/// <summary>
/// One ordinal-selected root in (0,1] of a degree-at-most-eight integer value
/// polynomial. Borrowed coefficients and cell storage outlive this view.
/// </summary>
internal ref struct FiniteAxisValueRoot
{
    internal ReadOnlySpan<ulong> Coefficients;
    internal ReadOnlySpan<sbyte> Signs;
    internal Span<ulong> LowerNumerator;
    internal int DenominatorShift;
    internal int Ordinal;
    internal bool IsRational;

    /// <summary>Copies a root into independent storage and returns a view borrowing only the destinations.</summary>
    /// <remarks>
    /// The source must be a valid root. Destinations must be mutually disjoint
    /// and disjoint from source storage. Coefficient and sign capacities must
    /// hold the source polynomial spans; cell capacity must hold the active
    /// numerator and, for a nonrational root, its excluded upper endpoint.
    /// Polynomial destination tails are untouched; the cell tail is cleared.
    /// </remarks>
    internal static FiniteAxisValueRoot CopyTo(scoped FiniteAxisValueRoot source,
        Span<ulong> coefficients, Span<sbyte> signs, Span<ulong> cell)
    {
        source.Coefficients.CopyTo(coefficients);
        source.Signs.CopyTo(signs);
        int words = WideArithmetic.GetActiveMagnitudeLength(source.LowerNumerator);
        source.LowerNumerator[..words].CopyTo(cell);
        cell[words..].Clear();
        return new FiniteAxisValueRoot
        {
            Coefficients = coefficients[..source.Coefficients.Length], Signs = signs[..source.Signs.Length],
            LowerNumerator = cell, DenominatorShift = source.DenominatorShift,
            Ordinal = source.Ordinal, IsRational = source.IsRational
        };
    }
}
