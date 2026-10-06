//=======================================================================
// MIT License, Copyright (c) 2024-present David Oravsky (mrdav30)
// See LICENSE file in the project root for full license information.
//=======================================================================
using System;
using System.Runtime.CompilerServices;

namespace FixedMathSharp.Geometry;

/// <content>
/// Shared exact value-root isolation for reciprocal polynomial charts.
/// </content>
internal static partial class WideFiniteAxisIntersection
{
    /// <summary>
    /// Isolates F(t) and its nominal-degree reversal G(s)=s^n F(1/s) on (0,1].
    /// Writes G into equally strided caller storage and returns independent
    /// compact views. All output spans must be disjoint from each other and F.
    /// </summary>
    [MethodImpl(MethodImplOptions.NoInlining)] // Release the shared chain before either consumer's returning scratch.
    internal static void GetFiniteValueReciprocalRoots(ReadOnlySpan<ulong> coefficients,
        ReadOnlySpan<sbyte> signs, Span<ulong> reciprocalCoefficients, Span<sbyte> reciprocalSigns,
        Span<ulong> cells, Span<int> shifts, Span<ulong> reciprocalCells, Span<int> reciprocalShifts,
        out FiniteAxisValueRoots roots, out FiniteAxisValueRoots reciprocalRoots)
    {
        int degree = signs.Length - 1;
        int words = coefficients.Length / signs.Length;
        for (int index = 0; index <= degree; index++)
        {
            coefficients.Slice(index * words, words)
                .CopyTo(reciprocalCoefficients.Slice((degree - index) * words, words));
            reciprocalSigns[degree - index] = signs[index];
        }
        // Nominal leading zeros create a zero factor in G; zero factors in F
        // disappear at infinity. Keep their global repeated-root metadata with
        // the existing complete owners rather than complicating the shared chain.
        if (degree <= 0 || signs[0] == 0 || signs[degree] == 0)
        {
            roots = GetFiniteValueRoots(coefficients, signs, cells, shifts);
            reciprocalRoots = GetFiniteValueRoots(reciprocalCoefficients, reciprocalSigns,
                reciprocalCells, reciprocalShifts);
            return;
        }
        cells.Clear(); shifts.Clear(); reciprocalCells.Clear(); reciprocalShifts.Clear();
        bool compact = TryGetFiniteValueRootsBernstein(coefficients, signs, cells, shifts, out int count);
        bool reciprocalCompact = TryGetFiniteValueRootsBernstein(reciprocalCoefficients, reciprocalSigns,
            reciprocalCells, reciprocalShifts, out int reciprocalCount);
        int rationalMask = 0, reciprocalMask = 0;
        bool repeated = false;
        if (!compact || !reciprocalCompact)
        {
            int bits = GetFiniteRootCoefficientBits(coefficients, signs.Length);
            Span<ulong> arena = stackalloc ulong[(880 * (bits + 64) + 63) / 64 + 512];
            Span<int> offsets = stackalloc int[9];
            Span<int> widths = stackalloc int[9];
            Span<int> degrees = stackalloc int[9];
            Span<sbyte> sturmSigns = stackalloc sbyte[81];
            int chainCount = BuildFiniteValueSturm(coefficients, signs, arena,
                offsets, widths, degrees, sturmSigns, out int used);
            repeated = degrees[chainCount - 1] > 0;
            Span<ulong> chain = arena[..used];
            Span<ulong> evaluation = arena[used..];
            if (!compact)
            {
                count = GetFiniteValueRootCount(cells[..(cells.Length / 8)], chain, offsets, widths,
                    degrees, sturmSigns, chainCount, evaluation, out int lowerVariations);
                compact = TryIsolateFiniteValueRootCells(coefficients, signs, cells, shifts, count,
                    lowerVariations, chain, offsets, widths, degrees, sturmSigns, chainCount,
                    evaluation, out rationalMask);
            }
            if (!reciprocalCompact)
            {
                ReverseFiniteValueSturm(chain, offsets, widths, degrees, sturmSigns, chainCount);
                reciprocalCount = GetFiniteValueRootCount(reciprocalCells[..(reciprocalCells.Length / 8)],
                    chain, offsets, widths, degrees, sturmSigns, chainCount, evaluation, out int lowerVariations);
                reciprocalCompact = TryIsolateFiniteValueRootCells(reciprocalCoefficients, reciprocalSigns,
                    reciprocalCells, reciprocalShifts, reciprocalCount, lowerVariations, chain, offsets,
                    widths, degrees, sturmSigns, chainCount, evaluation, out reciprocalMask);
            }
        }
        roots = new FiniteAxisValueRoots(cells, shifts, count, rationalMask, compact, repeated);
        reciprocalRoots = new FiniteAxisValueRoots(reciprocalCells, reciprocalShifts,
            reciprocalCount, reciprocalMask, reciprocalCompact, repeated);
    }

    private static void ReverseFiniteValueSturm(Span<ulong> chain, ReadOnlySpan<int> offsets,
        ReadOnlySpan<int> widths, Span<int> degrees, Span<sbyte> signs, int count)
    {
        // For oriented rows F_i, R_i(s)=(-1)^i s^degree(F_i) F_i(1/s).
        // Neighbor signs stay opposite at an internal row zero. At a defining
        // root, R_1=s G' (up to the original positive derivative scale).
        // Thus variations count distinct roots on positive s, even with abnormal
        // degree drops/repetition. Evaluate NEW right limits: reciprocity swaps sides.
        for (int index = 0; index < count; index++)
        {
            int degree = degrees[index], words = widths[index];
            Span<ulong> row = chain.Slice(offsets[index], (degree + 1) * words);
            Span<sbyte> rowSigns = signs.Slice(index * 9, degree + 1);
            for (int coefficient = 0; coefficient < (degree + 1) / 2; coefficient++)
            {
                int other = degree - coefficient;
                (rowSigns[coefficient], rowSigns[other]) = (rowSigns[other], rowSigns[coefficient]);
                for (int word = 0; word < words; word++)
                    (row[coefficient * words + word], row[other * words + word]) =
                        (row[other * words + word], row[coefficient * words + word]);
            }
            if ((index & 1) != 0)
                for (int coefficient = 0; coefficient <= degree; coefficient++)
                    rowSigns[coefficient] = (sbyte)-rowSigns[coefficient];
            while (rowSigns[degree] == 0)
                degree--;
            degrees[index] = degree;
        }
    }
}
