//=======================================================================
// TriangleConeRimContacts.cs
//=======================================================================
// MIT License, Copyright (c) 2024-present David Oravsky (mrdav30)
// See LICENSE file in the project root for full license information.
//=======================================================================
using System;
using System.Runtime.CompilerServices;
using static FixedMathSharp.Geometry.CylinderContactAlgebra;
using static FixedMathSharp.Geometry.CircularRimContactAlgebra;

namespace FixedMathSharp.Geometry;

/// <summary>Admitted stationary triangle-edge/base-rim contacts of a finite cone.</summary>
internal static class TriangleConeRimContacts
{
    private const int ParameterSlots = 33;
    internal const int ValueCellWords = (16 * (ValueWords * 64 + 11) + 255) / 64;

    private struct Selection
    {
        internal bool HasValue, IsZero, IsRational;
        internal WideAxis3 First, Second;
        internal int Edge, ParameterOrdinal, ValueOrdinal, ValueShift;
    }

    internal static bool TryGetContact(in TriangleCircularGeometry geometry, FixedTriangle triangle,
        Fixed64 height, Fixed64 radius, scoped ConvexContactCandidate analytic, Fixed64 analyticDepth,
        out bool hasBetter, out Vector3d normal, out Vector3d radialPoint,
        out Vector3d trianglePoint, out Fixed64 depth, out bool clamped)
    {
        hasBetter = false; normal = radialPoint = trianglePoint = default;
        depth = analyticDepth; clamped = false;
        Span<ulong> bestValues = stackalloc ulong[5 * ValueWords];
        Span<sbyte> bestSigns = stackalloc sbyte[5];
        Span<ulong> bestCell = stackalloc ulong[ValueCellWords];
        Selection best = default;
        if (!TryKeepContacts(geometry, height, radius, analytic, analyticDepth,
                bestValues, bestSigns, bestCell, ref best))
            return false;
        if (!best.HasValue)
            return true;
        hasBetter = true;
        GetContactMaterials(geometry, triangle, height, radius, bestValues, bestSigns, bestCell, best,
            out normal, out radialPoint, out trianglePoint, out depth, out clamped);
        return true;
    }

    private static bool TryKeepContacts(in TriangleCircularGeometry geometry,
        Fixed64 height, Fixed64 radius, scoped ConvexContactCandidate analytic, Fixed64 analyticDepth,
        scoped Span<ulong> bestValues, scoped Span<sbyte> bestSigns, scoped Span<ulong> bestCell,
        ref Selection best)
    {
        // The caller's exact face-width proof excludes a clamped analytic gap.
        // Its rounded value d bounds the original gap by d+1/2 raw, including Max.
        ulong upperTwiceRaw = ((ulong)analyticDepth.m_rawValue << 1) | 1UL;
        for (int edge = 0; edge < 3; edge++)
        {
            WideAxis3 e = geometry.Edge(edge);
            // Horizontal edges are meridional; vertical edges are circular.
            // Their complete boundaries/principal radial directions are analytic.
            if (e.Y.IsZero || e.X.IsZero && e.Z.IsZero)
                continue;
            GetBasis(e, out WideAxis3 u, out WideAxis3 v);
            for (int firstSign = -1; firstSign <= 1; firstSign += 2)
                for (int secondSign = -1; secondSign <= 1; secondSign += 2)
                    for (int chart = 0; chart < 2; chart++)
                    {
                        WideAxis3 first = firstSign > 0 ? u : -u;
                        WideAxis3 second = secondSign > 0 ? v : -v;
                        if (chart != 0) (first, second) = (second, first);
                        if (!TryChart(geometry, height, radius, edge, first, second,
                                analytic, upperTwiceRaw, bestValues, bestSigns, bestCell, ref best))
                            return false;
                    }
        }
        return true;
    }

    private static void GetContactMaterials(in TriangleCircularGeometry geometry, FixedTriangle triangle,
        Fixed64 height, Fixed64 radius, ReadOnlySpan<ulong> bestValues, ReadOnlySpan<sbyte> bestSigns,
        Span<ulong> bestCell, Selection best, out Vector3d normal, out Vector3d radialPoint,
        out Vector3d trianglePoint, out Fixed64 depth, out bool clamped)
    {
        clamped = false;
        GetMaterials(geometry, triangle, height, radius, best, out normal, out radialPoint, out trianglePoint);
        if (best.IsZero)
            depth = Fixed64.Zero;
        else
        {
            FiniteAxisValueRoot root = Restore(bestValues, bestSigns, bestCell, best);
            ConvexContactValueRoot.GetRoundedDepth(geometry.RawScale, geometry.ValueShift,
                ref root, out depth, out clamped);
        }
    }

    [MethodImpl(MethodImplOptions.NoInlining)]
    private static bool TryChart(in TriangleCircularGeometry geometry, Fixed64 height, Fixed64 radius,
        int edge, WideAxis3 first, WideAxis3 second, scoped ConvexContactCandidate analytic, ulong upperTwiceRaw,
        scoped Span<ulong> bestValues, scoped Span<sbyte> bestSigns, scoped Span<ulong> bestCell,
        ref Selection best)
    {
        WideAxis3 outward = geometry.EdgeFromTo((edge + 2) % 3, edge);
        Signed576 a = WideAxis3.Dot(first, outward), b = WideAxis3.Dot(second, outward);
        // Each root needs positive outward projection a+t*b on t in (0,1].
        // Nonpositive endpoints exclude the complete affine chart before construction.
        if (a.Sign <= 0 && WideArithmetic.AddSigned576(a, b).Sign <= 0)
            return true;
        Span<ulong> data = stackalloc ulong[ParameterSlots * Words];
        Span<sbyte> signs = stackalloc sbyte[ParameterSlots];
        BuildConeParameter(geometry, height, radius, edge, first, second, data, signs);
        Span<ulong> batchCells = stackalloc ulong[64];
        Span<int> batchShifts = stackalloc int[8];
        FiniteAxisValueRoots roots = WideFiniteAxisIntersection.GetFiniteValueRoots(
            data[..(5 * Words)], signs[..5], batchCells, batchShifts);
        Span<ulong> cell = stackalloc ulong[WideFiniteAxisIntersection.GetFiniteValueRootCellWords(
            data[..(5 * Words)], signs[..5])];
        Span<ulong> values = stackalloc ulong[5 * ValueWords];
        Span<sbyte> valueSigns = stackalloc sbyte[5];
        Span<ulong> valueCell = stackalloc ulong[ValueCellWords];
        for (int ordinal = 0; ordinal < roots.Count; ordinal++)
        {
            scoped FiniteAxisValueRoot root = FiniteAxisValueRoots.GetRoot(roots, ordinal,
                data[..(5 * Words)], signs[..5], cell);
            if (Sign(ref root, data, signs, 31, 2) <= 0)
                continue;
            // For negative n.Y, squaring is equivalent only with this sign
            // guard. Equality belongs to the earlier lateral-generator fan.
            if (Sign(ref root, data, signs, 26, 2) < 0
                && Sign(ref root, data, signs, 28, 3) <= 0)
                continue;
            int kSign = Sign(ref root, data, signs, 8, 2);
            if (kSign == 0 || Sign(ref root, data, signs, 10, 3) != -kSign)
                continue;
            int gapSign = Sign(ref root, data, signs, 13, 3) * kSign;
            if (gapSign < 0)
                return false;
            if (best.IsZero || analytic.GapSign == 0)
                continue;
            var candidate = new Selection
            {
                HasValue = true, First = first, Second = second, Edge = edge,
                ParameterOrdinal = ordinal
            };
            if (gapSign == 0)
            {
                candidate.IsZero = true; best = candidate;
                continue;
            }
            // Positive roots at or above the analytic half-raw upper bound
            // cannot win. Negative/zero gaps have already retained classification.
            if (ConvexContactValueRoot.CompareSquaredGapToTwiceRaw(ref root,
                    data.Slice(16 * Words, 5 * Words), signs.Slice(16, 5),
                    data.Slice(21 * Words, 5 * Words), signs.Slice(21, 5), upperTwiceRaw) >= 0)
                continue;
            // Materialize only roots that survive feature admission and the
            // analytic bound. Rebuilding is valid for any number of survivors.
            BuildValues(geometry.Edge(edge), geometry.CapOffset(edge, 1), geometry.Radius, geometry.ValueShift, values, valueSigns);
            scoped FiniteAxisValueRoot value = ConvexContactValueRoot.MapSquaredValue(
                geometry.RawScale, geometry.ValueShift, ref root,
                data.Slice(16 * Words, 5 * Words), signs.Slice(16, 5),
                data.Slice(21 * Words, 5 * Words), signs.Slice(21, 5), values, valueSigns, valueCell);
            int comparison = best.HasValue
                ? WideFiniteAxisIntersection.CompareFiniteValueRoots(value, Restore(bestValues, bestSigns, bestCell, best))
                : ConvexContactValueRoot.CompareRootSquared(geometry.RawScale, geometry.ValueShift, ref value, analytic);
            if (comparison >= 0)
                continue;
            values.CopyTo(bestValues); valueSigns.CopyTo(bestSigns); valueCell.CopyTo(bestCell);
            candidate.ValueOrdinal = value.Ordinal; candidate.ValueShift = value.DenominatorShift;
            candidate.IsRational = value.IsRational; best = candidate;
        }
        return true;
    }

    [MethodImpl(MethodImplOptions.NoInlining)]
    private static void BuildConeParameter(in TriangleCircularGeometry geometry, Fixed64 height, Fixed64 radius,
        int edge, WideAxis3 first, WideAxis3 second, Span<ulong> data, Span<sbyte> signs)
    {
        BuildConeParameter(geometry.CapOffset(edge, 1), geometry.Radius, geometry.RawScale,
            height, radius, first, second, data, signs);
        WideAxis3 outward = geometry.EdgeFromTo((edge + 2) % 3, edge);
        Write(data, signs, 31, WideAxis3.Dot(outward, first));
        Write(data, signs, 32, WideAxis3.Dot(outward, second));
    }

    // Writes the shared rim stationary chart and cone support switch into
    // slots 0..30. Triangle-only support walls occupy the following two slots.
    internal static void BuildConeParameter(WideAxis3 offset, Signed320 scaledRadius, Signed192 rawScale,
        Fixed64 height, Fixed64 radius, WideAxis3 first, WideAxis3 second,
        Span<ulong> data, Span<sbyte> signs)
    {
        BuildParameter(offset, scaledRadius, rawScale, first, second, data, signs);
        Write(data, signs, 26, Signed576.ExtendValue(first.Y));
        Write(data, signs, 27, Signed576.ExtendValue(second.Y));
        Signed192 h = Signed192.Raw(height), r = Signed192.Raw(radius);
        Signed320 hh = WideArithmetic.MultiplySigned192(h, h), rr = WideArithmetic.MultiplySigned192(r, r);
        Span<ulong> work = stackalloc ulong[3 * Words];
        for (int index = 0; index < 3; index++)
        {
            WideAxis3 left = index < 2 ? first : second, right = index == 0 ? first : second;
            // Triangle axes <198 bits; segment affine-chart axes <200 bits.
            // Raw dimensions <63 give region coefficients <532 bits. The
            // forty-word magnitude owner retains both without narrowing.
            Signed576 radial = RadialDot(left, right), axial = WideArithmetic.MultiplySigned320(left.Y, right.Y);
            Import(radial, Slot(work, 0)); Import(rr, Slot(work, 1));
            WideArithmetic.MultiplyMagnitudes(Slot(work, 0), Slot(work, 1), Slot(data, 28 + index));
            int sign = radial.Sign;
            Import(axial, Slot(work, 0)); Import(hh, Slot(work, 1));
            WideArithmetic.MultiplyMagnitudes(Slot(work, 0), Slot(work, 1), Slot(work, 2));
            Add(Slot(work, 2), -axial.Sign, Slot(data, 28 + index), ref sign);
            if (index == 1) WideArithmetic.AddMagnitudeInto(Slot(data, 29), Slot(data, 29));
            signs[28 + index] = (sbyte)sign;
        }
    }

    [MethodImpl(MethodImplOptions.NoInlining)]
    private static void GetMaterials(in TriangleCircularGeometry geometry, FixedTriangle triangle,
        Fixed64 height, Fixed64 radius, Selection best,
        out Vector3d normal, out Vector3d radialPoint, out Vector3d trianglePoint)
    {
        Span<ulong> data = stackalloc ulong[ParameterSlots * Words];
        Span<sbyte> signs = stackalloc sbyte[ParameterSlots];
        BuildConeParameter(geometry, height, radius, best.Edge, best.First, best.Second, data, signs);
        Span<ulong> cell = stackalloc ulong[WideFiniteAxisIntersection.GetFiniteValueRootCellWords(data[..(5 * Words)], signs[..5])];
        bool found = WideFiniteAxisIntersection.TryGetFiniteValueRoot(data[..(5 * Words)], signs[..5],
            best.ParameterOrdinal, cell, out FiniteAxisValueRoot root);
        System.Diagnostics.Debug.Assert(found);
        trianglePoint = TriangleCylinderRimWitnesses.GetRootPoint(geometry, triangle, best.Edge, best.First, best.Second, ref root);
        Span<ulong> gradient = stackalloc ulong[30];
        Span<sbyte> gradientSigns = stackalloc sbyte[6];
        WriteGradient(best.First, best.Second, gradient, gradientSigns);
        gradient.Slice(10, 10).Clear(); gradientSigns.Slice(2, 2).Clear();
        radialPoint = ConvexContactValueRoot.GetScaledNormalizedDirection(ref root, gradient, gradientSigns, radius, -1);
        WriteGradient(Transform(geometry.WorldBasis, best.First), Transform(geometry.WorldBasis, best.Second), gradient, gradientSigns);
        normal = ConvexContactValueRoot.GetNormalizedDirection(ref root, gradient, gradientSigns, 1);
    }

    private static FiniteAxisValueRoot Restore(ReadOnlySpan<ulong> values, ReadOnlySpan<sbyte> signs,
        Span<ulong> cell, Selection selection) => new()
    {
        Coefficients = values, Signs = signs, LowerNumerator = cell,
        DenominatorShift = selection.ValueShift, Ordinal = selection.ValueOrdinal, IsRational = selection.IsRational
    };
}
