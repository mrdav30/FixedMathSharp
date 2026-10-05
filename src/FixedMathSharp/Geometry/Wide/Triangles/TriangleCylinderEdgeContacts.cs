//=======================================================================
// MIT License, Copyright (c) 2024-present David Oravsky (mrdav30)
// See LICENSE file in the project root for full license information.
//=======================================================================
using System;
using System.Runtime.CompilerServices;
using static FixedMathSharp.Geometry.CylinderContactAlgebra;
using static FixedMathSharp.Geometry.CircularRimContactAlgebra;

namespace FixedMathSharp.Geometry;

/// <summary>Every admitted triangle-edge/cylinder-rim stationary support root.</summary>
internal static class TriangleCylinderEdgeContacts
{
    private const int ParameterSlots = 30;
    private const int ValueCellWords = (16 * (ValueWords * 64 + 11) + 255) / 64;

    private struct Selection
    {
        internal bool HasValue, IsZero, IsRational;
        internal WideAxis3 First, Second;
        internal int Edge, Cap, CoreSign, ParameterOrdinal, ValueOrdinal, ValueShift;
    }

    internal static bool TryGetContact(in TriangleCircularGeometry geometry, WideAxis3 coreOffset, WideAxis3 coreAxis,
        Vector2d coreDirection, Fixed64 coreLength, FixedTriangle triangle,
        in ConvexContactCandidate analytic, Fixed64 analyticDepth, bool analyticClamped, Fixed64 radius, out bool hasBetter, out Vector3d normal,
        out Vector3d radialPoint, out Vector3d trianglePoint, out Fixed64 depth, out bool clamped,
        out int mask, out int cap, out int coreSign, out int integralRadialMask)
    {
        hasBetter = false; normal = radialPoint = trianglePoint = default; depth = default;
        clamped = false; mask = cap = coreSign = 0;
        integralRadialMask = 5;
        Span<ulong> bestValues = stackalloc ulong[5 * ValueWords];
        Span<sbyte> bestSigns = stackalloc sbyte[5];
        Span<ulong> bestCell = stackalloc ulong[ValueCellWords];
        Selection best = default;
        int regionStart = coreAxis.IsZero ? 0 : -1;
        for (int region = regionStart; region <= (coreAxis.IsZero ? 0 : 1); region += 2)
        {
            TriangleCircularGeometry endpoint = region == 0 ? geometry : geometry.AtCoreRegion(coreOffset, region);
            for (int edge = 0; edge < 3; edge++)
            {
                WideAxis3 e = endpoint.Edge(edge);
                // Parallel edges project a circle; perpendicular edges project a
                // line segment. Their extrema are already analytic fan boundaries.
                if (e.Y.IsZero || e.X.IsZero && e.Z.IsZero)
                    continue;
                GetBasis(e, out WideAxis3 u, out WideAxis3 v);
                for (int firstSign = -1; firstSign <= 1; firstSign += 2)
                    for (int secondSign = -1; secondSign <= 1; secondSign += 2)
                        for (int capSign = -1; capSign <= 1; capSign += 2)
                            for (int chart = 0; chart < 2; chart++)
                            {
                                WideAxis3 first = firstSign > 0 ? u : -u;
                                WideAxis3 second = secondSign > 0 ? v : -v;
                                if (chart != 0)
                                    (first, second) = (second, first);
                                if (!TryChart(endpoint, coreAxis, region, edge, first, second, capSign, analytic,
                                        ((ulong)analyticDepth.m_rawValue << 1) | 1UL, !analyticClamped,
                                        bestValues, bestSigns, bestCell, ref best))
                                    return false;
                            }
            }
        }
        if (!best.HasValue)
            return true;
        hasBetter = true; mask = (1 << best.Edge) | (1 << ((best.Edge + 1) % 3)); cap = best.Cap; coreSign = best.CoreSign;
        TriangleCircularGeometry winner = coreSign == 0 ? geometry : geometry.AtCoreRegion(coreOffset, coreSign);
        GetMaterials(winner, coreAxis, coreDirection, coreLength, triangle, best, radius,
            out normal, out radialPoint, out trianglePoint, out integralRadialMask);
        if (!best.IsZero)
        {
            FiniteAxisValueRoot root = Restore(bestValues, bestSigns, bestCell, best);
            ConvexContactValueRoot.GetRoundedDepth(geometry.RawScale, geometry.ValueShift,
                ref root, out depth, out clamped);
        }
        return true;
    }

    [MethodImpl(MethodImplOptions.NoInlining)]
    private static bool TryChart(in TriangleCircularGeometry geometry, WideAxis3 coreAxis, int region, int edge,
        WideAxis3 first, WideAxis3 second, int cap, scoped ConvexContactCandidate analytic,
        ulong upperTwiceRaw, bool hasUpperBound,
        scoped Span<ulong> bestValues, scoped Span<sbyte> bestSigns, scoped Span<ulong> bestCell,
        ref Selection best)
    {
        // Each admission projection is affine on t in (0,1] and must be
        // positive. Nonpositive endpoints exclude every root before construction.
        WideAxis3 outward = TriangleCircularGeometry.Subtract(geometry.Vertex(edge), geometry.Vertex((edge + 2) % 3));
        // GetBasis gives exactly one nonzero Y component. Its cap projection
        // cannot change sign inside this chart, so this check fully admits it.
        if (first.Y.Sign * cap <= 0 && WideArithmetic.AddSigned320(first.Y, second.Y).Sign * cap <= 0)
            return true;
        Signed576 firstCone = WideAxis3.Dot(first, outward), secondCone = WideAxis3.Dot(second, outward);
        if (firstCone.Sign <= 0 && WideArithmetic.AddSigned576(firstCone, secondCone).Sign <= 0)
            return true;
        if (!coreAxis.IsZero)
        {
            Signed576 firstCore = WideAxis3.Dot(coreAxis, first), secondCore = WideAxis3.Dot(coreAxis, second);
            if (firstCore.Sign * region <= 0 && WideArithmetic.AddSigned576(firstCore, secondCore).Sign * region <= 0)
                return true;
        }

        Span<ulong> data = stackalloc ulong[ParameterSlots * Words];
        Span<sbyte> signs = stackalloc sbyte[ParameterSlots];
        BuildParameter(geometry, coreAxis, region, edge, first, second, cap, data, signs);
        Span<ulong> batchCells = stackalloc ulong[64];
        Span<int> batchShifts = stackalloc int[8];
        FiniteAxisValueRoots roots = WideFiniteAxisIntersection.GetFiniteValueRoots(
            data[..(5 * Words)], signs[..5], batchCells, batchShifts);
        Span<ulong> cell = stackalloc ulong[WideFiniteAxisIntersection.GetFiniteValueRootCellWords(
            data[..(5 * Words)], signs[..5])];
        Span<ulong> values = stackalloc ulong[5 * ValueWords];
        Span<sbyte> valueSigns = stackalloc sbyte[5];
        Span<ulong> valueCell = stackalloc ulong[ValueCellWords];
        bool valuesReady = false;
        for (int ordinal = 0; ordinal < roots.Count; ordinal++)
        {
            scoped FiniteAxisValueRoot root = FiniteAxisValueRoots.GetRoot(roots, ordinal,
                data[..(5 * Words)], signs[..5], cell);
            // Zero third-vertex alignment is the already-ranked face normal;
            // this boundary cannot improve that earlier canonical feature.
            if (Sign(ref root, data, signs, 26, 2) <= 0)
                continue;
            // Boundary roots belong to the explicit seam fan. Reject the
            // opposite endpoint's hemisphere before sign rejection or ranking.
            if (!coreAxis.IsZero && Sign(ref root, data, signs, 28, 2) <= 0)
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
                Cap = cap, CoreSign = region, ParameterOrdinal = ordinal
            };
            if (gapSign == 0)
            {
                candidate.IsZero = true; best = candidate;
                continue;
            }
            // The unclamped analytic depth d bounds its exact gap by d+1/2 raw.
            // Reject positive roots at or above that bound before value mapping;
            // admitted negative and zero gaps have already retained their semantics.
            if (hasUpperBound && ConvexContactValueRoot.CompareSquaredGapToTwiceRaw(ref root,
                    data.Slice(16 * Words, 5 * Words), signs.Slice(16, 5),
                    data.Slice(21 * Words, 5 * Words), signs.Slice(21, 5), upperTwiceRaw) >= 0)
                continue;
            if (!valuesReady)
            {
                BuildValues(geometry.Edge(edge), geometry.CapOffset(edge, cap), geometry.Radius, geometry.ValueShift, values, valueSigns);
                valuesReady = true;
            }
            scoped FiniteAxisValueRoot value = ConvexContactValueRoot.MapSquaredValue(
                geometry.RawScale, geometry.ValueShift, ref root,
                data.Slice(16 * Words, 5 * Words), signs.Slice(16, 5),
                data.Slice(21 * Words, 5 * Words), signs.Slice(21, 5),
                values, valueSigns, valueCell);
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

    private static void BuildParameter(in TriangleCircularGeometry geometry, WideAxis3 coreAxis, int region, int edge,
        WideAxis3 first, WideAxis3 second, int cap, Span<ulong> data, Span<sbyte> signs)
    {
        CircularRimContactAlgebra.BuildParameter(geometry.CapOffset(edge, cap), geometry.Radius, geometry.RawScale,
            first, second, data, signs);
        WideAxis3 outward = TriangleCircularGeometry.Subtract(geometry.Vertex(edge), geometry.Vertex((edge + 2) % 3));
        Write(data, signs, 26, WideAxis3.Dot(outward, first));
        Write(data, signs, 27, WideAxis3.Dot(outward, second));
        if (!coreAxis.IsZero)
        {
            Signed576 firstCore = WideAxis3.Dot(coreAxis, first), secondCore = WideAxis3.Dot(coreAxis, second);
            Write(data, signs, 28, region < 0 ? WideArithmetic.SubtractSigned576(default, firstCore) : firstCore);
            Write(data, signs, 29, region < 0 ? WideArithmetic.SubtractSigned576(default, secondCore) : secondCore);
        }
    }

    [MethodImpl(MethodImplOptions.NoInlining)]
    private static void GetMaterials(in TriangleCircularGeometry geometry, WideAxis3 coreAxis,
        Vector2d coreDirection, Fixed64 coreLength, FixedTriangle triangle, Selection best, Fixed64 radius,
        out Vector3d normal, out Vector3d radialPoint, out Vector3d trianglePoint, out int integralRadialMask)
    {
        Span<ulong> data = stackalloc ulong[ParameterSlots * Words];
        Span<sbyte> signs = stackalloc sbyte[ParameterSlots];
        BuildParameter(geometry, coreAxis, best.CoreSign, best.Edge, best.First, best.Second, best.Cap, data, signs);
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
        integralRadialMask = 5;
        if (best.CoreSign != 0)
        {
            Vector3d core = TriangleCapsuleSlabEndpointWitnesses.GetCore(coreDirection, coreLength, best.CoreSign,
                out FixedPointAnchorTerm3d term);
            radialPoint = TriangleCapsuleSlabEndpointWitnesses.AdjustRoot(ref root, gradient, gradientSigns,
                radius, core, term, radialPoint, out integralRadialMask);
        }
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
