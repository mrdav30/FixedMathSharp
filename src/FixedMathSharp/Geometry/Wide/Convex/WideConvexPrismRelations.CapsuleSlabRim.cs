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
    // The unshifted stationary quartic W has coefficient height <2520 bits.
    private const int CapsuleSlabParameterCellWords = (16 * (2520 + 11) + 255) / 64;

    private ref struct CapsuleSlabRimSelection
    {
        internal bool HasValue;
        internal WideAxis3 First, Second;
        internal int GapSign;
        internal ulong AnalyticUpperTwiceRaw;
        internal FiniteAxisValueRoot Value, Parameter;
        internal Span<ulong> Values, Cell, ParameterValues, ParameterCell;
        internal Span<sbyte> Signs, ParameterSigns;

        internal void KeepParameter(scoped FiniteAxisValueRoot root, WideAxis3 first, WideAxis3 second, int gapSign)
        {
            Parameter = FiniteAxisValueRoot.CopyTo(root, ParameterValues, ParameterSigns, ParameterCell);
            First = first; Second = second; GapSign = gapSign; HasValue = true;
        }
    }

    [MethodImpl(MethodImplOptions.NoInlining)]
    private static bool TryImproveCapsuleSlabRim(in CapsuleSlabGeometry geometry, ConvexContactCandidate analytic,
        Fixed64 capsuleRadius, out bool intersects, out Vector3d normal, out Fixed64 depth, out bool clamped)
    {
        intersects = false; normal = default; depth = default; clamped = false;
        var best = new CapsuleSlabRimSelection
        {
            Values = stackalloc ulong[5 * ValueWords], Signs = stackalloc sbyte[5],
            Cell = stackalloc ulong[CapsuleSlabValueCellWords],
            ParameterValues = stackalloc ulong[5 * Words], ParameterSigns = stackalloc sbyte[5],
            ParameterCell = stackalloc ulong[CapsuleSlabParameterCellWords]
        };
        GetBasis(geometry.CapsuleAxis, out WideAxis3 u, out WideAxis3 v);
        if (analytic.GapSign > 0)
        {
            GetRoundedConvexContactCandidateDepth(analytic, Fixed64.Zero, out Fixed64 magnitude, out bool magnitudeClamped);
            if (!magnitudeClamped)
                best.AnalyticUpperTwiceRaw = ((ulong)magnitude.m_rawValue << 1) | 1UL;
        }
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
                            KeepCapsuleSlabRimChart(geometry, cap, end, first, second, analytic, ref best);
                        }
        if (!best.HasValue) return false;
        if (best.GapSign == 0)
        {
            intersects = true; depth = capsuleRadius;
        }
        else
        {
            intersects = ConvexContactValueRoot.GetRoundedDepth(geometry.RawScale, geometry.ValueShift,
                ref best.Value, out depth, out clamped, best.GapSign, capsuleRadius);
        }
        if (intersects) normal = GetCapsuleSlabRimNormal(geometry, ref best);
        return true;
    }

    [MethodImpl(MethodImplOptions.NoInlining)]
    private static void KeepCapsuleSlabRimChart(in CapsuleSlabGeometry geometry, int cap, int end,
        WideAxis3 first, WideAxis3 second, scoped ConvexContactCandidate analytic,
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
            if (gapSign == 0)
            {
                best.KeepParameter(root, first, second, gapSign);
                continue;
            }
            // A positive analytic gap rounded to d is at most d+1/2 raw.
            // A root at or above that exact bound cannot win (ties stay earlier).
            // Reuse the triangle/box certificate before value mapping; negative
            // gaps retain exact signed ranking, and clamped gaps supply no bound.
            if (best.AnalyticUpperTwiceRaw != 0 && gapSign > 0
                && ConvexContactValueRoot.CompareSquaredGapToTwiceRaw(ref root,
                    data.Slice(16 * Words, 5 * Words), signs.Slice(16, 5),
                    data.Slice(21 * Words, 5 * Words), signs.Slice(21, 5), best.AnalyticUpperTwiceRaw) >= 0)
                continue;
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
                    ? WideFiniteAxisIntersection.CompareFiniteValueRoots(value, best.Value)
                    : ConvexContactValueRoot.CompareRootSquared(geometry.RawScale, geometry.ValueShift, ref value, analytic);
                if (gapSign * squaredComparison >= 0) continue;
            }
            best.Value = FiniteAxisValueRoot.CopyTo(value, best.Values, best.Signs, best.Cell);
            best.KeepParameter(root, first, second, gapSign);
        }
    }

    [MethodImpl(MethodImplOptions.NoInlining)]
    private static Vector3d GetCapsuleSlabRimNormal(in CapsuleSlabGeometry geometry, scoped ref CapsuleSlabRimSelection best)
    {
        // Admission and value mapping already refined this parameter. Retain
        // that exact cell and its metadata rather than rebuilding its quartic
        // and isolating the same root again for final normal rounding.
        Span<ulong> gradient = stackalloc ulong[30];
        Span<sbyte> gradientSigns = stackalloc sbyte[6];
        WriteGradient(Transform(geometry.WorldBasis, best.First), Transform(geometry.WorldBasis, best.Second), gradient, gradientSigns);
        return ConvexContactValueRoot.GetNormalizedDirection(ref best.Parameter, gradient, gradientSigns, 1);
    }
}
