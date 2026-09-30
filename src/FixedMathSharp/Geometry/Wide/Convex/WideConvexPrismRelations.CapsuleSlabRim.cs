//=======================================================================
// MIT License, Copyright (c) 2024-present David Oravsky (mrdav30)
// See LICENSE file in the project root for full license information.
//=======================================================================
using System;
using System.Runtime.CompilerServices;
using static FixedMathSharp.Geometry.CylinderContactAlgebra;
using static FixedMathSharp.Geometry.CircularRimContactAlgebra;

namespace FixedMathSharp.Geometry;

internal static partial class WideConvexPrismRelations
{
    private const int CapsuleSlabParameterSlots = 26;
    // The reviewed value-polynomial height is <2800 bits, including the
    // common variable shift. Do not size retained cells from the field width.
    private const int CapsuleSlabValueCellWords = (16 * (2800 + 11) + 255) / 64;

    private struct CapsuleSlabRimSelection
    {
        internal bool HasValue, IsRational;
        internal WideAxis3 First, Second;
        internal int Cap, End, GapSign, ParameterOrdinal, ValueOrdinal, ValueShift;
    }

    [MethodImpl(MethodImplOptions.NoInlining)]
    private static bool TryImproveCapsuleSlabRim(in CapsuleSlabGeometry geometry, ConvexContactCandidate analytic,
        Fixed64 capsuleRadius, out bool intersects, out Vector3d normal, out Fixed64 depth, out bool clamped)
    {
        intersects = false; normal = default; depth = default; clamped = false;
        Span<ulong> bestValues = stackalloc ulong[5 * ValueWords];
        Span<sbyte> bestSigns = stackalloc sbyte[5];
        Span<ulong> bestCell = stackalloc ulong[CapsuleSlabValueCellWords];
        CapsuleSlabRimSelection best = default;
        GetBasis(geometry.CapsuleAxis, out WideAxis3 u, out WideAxis3 v);
        // Every quadrant and reciprocal chart is retained. Reflection and
        // largest-positive-root selection are not valid inside an end region.
        for (int cap = -1; cap <= 1; cap += 2)
            for (int end = -1; end <= 1; end += 2)
                for (int firstSign = -1; firstSign <= 1; firstSign += 2)
                    for (int secondSign = -1; secondSign <= 1; secondSign += 2)
                        for (int chart = 0; chart < 2; chart++)
                        {
                            WideAxis3 first = firstSign > 0 ? u : -u, second = secondSign > 0 ? v : -v;
                            if (chart != 0) (first, second) = (second, first);
                            KeepCapsuleSlabRimChart(geometry, cap, end, first, second, analytic,
                                bestValues, bestSigns, bestCell, ref best);
                        }
        if (!best.HasValue) return false;
        if (best.GapSign == 0)
        {
            intersects = true; depth = capsuleRadius;
        }
        else
        {
            FiniteAxisValueRoot value = RestoreCapsuleSlabValue(bestValues, bestSigns, bestCell, best);
            intersects = ConvexContactValueRoot.GetRoundedDepth(geometry.RawScale, geometry.ValueShift,
                ref value, out depth, out clamped, best.GapSign, capsuleRadius);
        }
        if (intersects) normal = GetCapsuleSlabRimNormal(geometry, best);
        return true;
    }

    [MethodImpl(MethodImplOptions.NoInlining)]
    private static void KeepCapsuleSlabRimChart(in CapsuleSlabGeometry geometry, int cap, int end,
        WideAxis3 first, WideAxis3 second, scoped ConvexContactCandidate analytic,
        scoped Span<ulong> bestValues, scoped Span<sbyte> bestSigns, scoped Span<ulong> bestCell,
        ref CapsuleSlabRimSelection best)
    {
        // GetBasis puts each Y/Z coordinate in exactly one of the two
        // directions. Its sign is therefore constant on the open chart
        // (0,1], even after swapping/sign changes. Admit the whole chart here;
        // no per-root cap/end polynomials or sign queries are needed.
        if (first.Y.Sign * cap < 0 || second.Y.Sign * cap < 0
            || first.Z.Sign * end < 0 || second.Z.Sign * end < 0)
            return;
        Span<ulong> data = stackalloc ulong[CapsuleSlabParameterSlots * Words];
        Span<sbyte> signs = stackalloc sbyte[CapsuleSlabParameterSlots];
        BuildParameter(geometry.CapOffset(cap, end), geometry.Radius, geometry.RawScale, first, second, data, signs);
        Span<ulong> batchCells = stackalloc ulong[64];
        Span<int> batchShifts = stackalloc int[8];
        FiniteAxisValueRoots roots = WideFiniteAxisIntersection.GetFiniteValueRoots(data[..(5 * Words)], signs[..5],
            batchCells, batchShifts);
        Span<ulong> cell = stackalloc ulong[WideFiniteAxisIntersection.GetFiniteValueRootCellWords(data[..(5 * Words)], signs[..5])];
        Span<ulong> values = stackalloc ulong[5 * ValueWords];
        Span<sbyte> valueSigns = stackalloc sbyte[5];
        Span<ulong> valueCell = stackalloc ulong[CapsuleSlabValueCellWords];
        bool valuesReady = false;
        for (int ordinal = 0; ordinal < roots.Count; ordinal++)
        {
            scoped FiniteAxisValueRoot root = FiniteAxisValueRoots.GetRoot(roots, ordinal, data[..(5 * Words)], signs[..5], cell);
            // Squaring the stationary equation introduces opposite-sign roots.
            // Simultaneous K=L=0 belongs to the earlier principal-axis fan.
            int kSign = Sign(ref root, data, signs, 8, 2);
            if (kSign == 0 || Sign(ref root, data, signs, 10, 3) != -kSign) continue;
            int gapSign = Sign(ref root, data, signs, 13, 3) * kSign;
            int incumbentSign = best.HasValue ? best.GapSign : analytic.GapSign;
            if (gapSign > incumbentSign || gapSign == 0 && incumbentSign == 0) continue;
            var candidate = new CapsuleSlabRimSelection
            {
                HasValue = true, First = first, Second = second, Cap = cap, End = end,
                GapSign = gapSign, ParameterOrdinal = ordinal
            };
            if (gapSign == 0)
            {
                best = candidate;
                continue;
            }
            if (!valuesReady)
            {
                BuildValues(geometry.CapsuleAxis, geometry.CapOffset(cap, end), geometry.Radius,
                    geometry.ValueShift, values, valueSigns);
                valuesReady = true;
            }
            scoped FiniteAxisValueRoot value = ConvexContactValueRoot.MapSquaredValue(geometry.RawScale, geometry.ValueShift,
                ref root, data.Slice(16 * Words, 5 * Words), signs.Slice(16, 5),
                data.Slice(21 * Words, 5 * Words), signs.Slice(21, 5), values, valueSigns, valueCell);
            if (gapSign == incumbentSign)
            {
                int squaredComparison = best.HasValue
                    ? WideFiniteAxisIntersection.CompareFiniteValueRoots(value, RestoreCapsuleSlabValue(bestValues, bestSigns, bestCell, best))
                    : ConvexContactValueRoot.CompareRootSquared(geometry.RawScale, geometry.ValueShift, value, analytic);
                if (gapSign * squaredComparison >= 0) continue;
            }
            values.CopyTo(bestValues); valueSigns.CopyTo(bestSigns); valueCell.CopyTo(bestCell);
            candidate.ValueOrdinal = value.Ordinal; candidate.ValueShift = value.DenominatorShift;
            candidate.IsRational = value.IsRational; best = candidate;
        }
    }

    [MethodImpl(MethodImplOptions.NoInlining)]
    private static Vector3d GetCapsuleSlabRimNormal(in CapsuleSlabGeometry geometry, CapsuleSlabRimSelection best)
    {
        Span<ulong> data = stackalloc ulong[CapsuleSlabParameterSlots * Words];
        Span<sbyte> signs = stackalloc sbyte[CapsuleSlabParameterSlots];
        BuildParameter(geometry.CapOffset(best.Cap, best.End), geometry.Radius, geometry.RawScale, best.First, best.Second, data, signs);
        Span<ulong> cell = stackalloc ulong[WideFiniteAxisIntersection.GetFiniteValueRootCellWords(data[..(5 * Words)], signs[..5])];
        bool found = WideFiniteAxisIntersection.TryGetFiniteValueRoot(data[..(5 * Words)], signs[..5],
            best.ParameterOrdinal, cell, out FiniteAxisValueRoot root);
        System.Diagnostics.Debug.Assert(found);
        Span<ulong> gradient = stackalloc ulong[30];
        Span<sbyte> gradientSigns = stackalloc sbyte[6];
        WriteGradient(Transform(geometry.WorldBasis, best.First), Transform(geometry.WorldBasis, best.Second), gradient, gradientSigns);
        return ConvexContactValueRoot.GetNormalizedDirection(ref root, gradient, gradientSigns, 1);
    }

    private static FiniteAxisValueRoot RestoreCapsuleSlabValue(ReadOnlySpan<ulong> values, ReadOnlySpan<sbyte> signs,
        Span<ulong> cell, CapsuleSlabRimSelection selection) => new()
    {
        Coefficients = values, Signs = signs, LowerNumerator = cell, DenominatorShift = selection.ValueShift,
        Ordinal = selection.ValueOrdinal, IsRational = selection.IsRational
    };
}
