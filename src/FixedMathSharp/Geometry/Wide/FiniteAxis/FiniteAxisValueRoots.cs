//=======================================================================
// MIT License, Copyright (c) 2024-present David Oravsky (mrdav30)
// See LICENSE file in the project root for full license information.
//=======================================================================
using System;

namespace FixedMathSharp.Geometry;

/// <summary>
/// Authoritative distinct-root count and optional compact cells for one fixed
/// polynomial on (0,1]. A failed compact batch still owns the complete count;
/// its partial cells are never consumed. Storage is borrowed and not mutated.
/// </summary>
internal readonly ref struct FiniteAxisValueRoots
{
    private readonly ReadOnlySpan<ulong> _cells;
    private readonly ReadOnlySpan<int> _shifts;
    private readonly int _rationalMask;
    internal int Count { get; }
    internal bool HasCompactCells { get; }
    internal bool HasRepeatedRoots { get; }

    internal FiniteAxisValueRoots(ReadOnlySpan<ulong> cells, ReadOnlySpan<int> shifts,
        int count, int rationalMask, bool compact, bool repeated)
    {
        _cells = cells;
        _shifts = shifts;
        _rationalMask = rationalMask;
        Count = count;
        HasCompactCells = compact;
        HasRepeatedRoots = repeated;
    }

    /// <summary>
    /// Materializes ordinal in [0,Count) for the unchanged polynomial used to
    /// create this view. The result borrows only the explicit polynomial and
    /// destination spans, not the compact cache. Fallback requires the full
    /// GetFiniteValueRootCellWords capacity; compact copying needs its active
    /// numerator words plus capacity for the nonrational upper endpoint.
    /// </summary>
    internal static FiniteAxisValueRoot GetRoot(scoped FiniteAxisValueRoots roots,
        int ordinal, ReadOnlySpan<ulong> coefficients,
        ReadOnlySpan<sbyte> signs, Span<ulong> cell)
    {
        System.Diagnostics.Debug.Assert(ordinal >= 0 && ordinal < roots.Count);
        int words = coefficients.Length / signs.Length;
        int degree = signs.Length - 1;
        // A valid ordinal proves that this unchanged polynomial has a root,
        // so trimming cannot reach a constant or zero polynomial.
        while (signs[degree] == 0)
            degree--;
        coefficients = coefficients[..((degree + 1) * words)];
        signs = signs[..(degree + 1)];
        if (!roots.HasCompactCells)
            return WideFiniteAxisIntersection.GetKnownFiniteValueRoot(coefficients, signs, ordinal, roots.Count, cell);
        int cellWords = roots._cells.Length / 8;
        ReadOnlySpan<ulong> source = roots._cells.Slice(ordinal * cellWords, cellWords);
        cell.Clear();
        source[..WideArithmetic.GetActiveMagnitudeLength(source)].CopyTo(cell);
        return new FiniteAxisValueRoot
        {
            Coefficients = coefficients, Signs = signs, LowerNumerator = cell,
            DenominatorShift = roots._shifts[ordinal], Ordinal = ordinal,
            IsRational = (roots._rationalMask & (1 << ordinal)) != 0
        };
    }
}
