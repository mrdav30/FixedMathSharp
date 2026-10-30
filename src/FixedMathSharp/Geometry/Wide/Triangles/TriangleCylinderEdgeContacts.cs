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

    internal struct Selection
    {
        internal bool HasValue, IsZero, IsRational;
        internal WideDirection3d First, Second;
        internal int Edge, Cap, CoreSign, ParameterOrdinal, ValueOrdinal, ValueShift, Triangle;
    }

    internal static bool TryGetContact(in TriangleCircularGeometry geometry, in WideDirection3d coreOffset, in WideDirection3d coreAxis,
        Vector2d coreDirection, Fixed64 coreLength, FixedTriangle triangle,
        in ConvexContactCandidate analytic, Fixed64 analyticDepth, bool analyticClamped, Fixed64 radius, out bool hasBetter, out Vector3d normal,
        out Vector3d radialPoint, out Vector3d trianglePoint, out Fixed64 depth, out bool clamped,
        out int mask, out int cap, out int coreSign, out int integralRadialMask, out bool strictOverlap, bool materialize = true)
    {
        int ValueWords = geometry.ValueWords;
        int ValueCellWords = (16 * (geometry.ValueWords * 64 + 11) + 255) / 64;
        hasBetter = false; normal = radialPoint = trianglePoint = default; depth = default;
        clamped = false; mask = cap = coreSign = 0;
        integralRadialMask = 5; strictOverlap = analytic.GapSign > 0;
        Span<ulong> bestValues = stackalloc ulong[5 * ValueWords];
        Span<sbyte> bestSigns = stackalloc sbyte[5];
        Span<ulong> bestCell = stackalloc ulong[ValueCellWords];
        Selection best = default;
        if (!TryKeepContacts(geometry, coreOffset, coreAxis, analytic, analyticDepth, analyticClamped,
                bestValues, bestSigns, bestCell, ref best)) return false;
        if (!best.HasValue)
            return true;
        strictOverlap = !best.IsZero;
        if (!materialize) return true;
        hasBetter = true; mask = (1 << best.Edge) | (1 << ((best.Edge + 1) % 3)); cap = best.Cap; coreSign = best.CoreSign;
        TriangleCircularGeometry winnerStorage;
        scoped ref readonly TriangleCircularGeometry winner = ref geometry;
        if (coreSign != 0)
        {
            winnerStorage = geometry.AtCoreRegion(coreOffset, coreSign);
            winner = ref winnerStorage;
        }
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

    internal static bool TryKeepContacts(in TriangleCircularGeometry geometry, in WideDirection3d coreOffset, in WideDirection3d coreAxis,
        in ConvexContactCandidate analytic, Fixed64 analyticDepth, bool analyticClamped,
        scoped Span<ulong> bestValues, scoped Span<sbyte> bestSigns, scoped Span<ulong> bestCell, ref Selection best,
        in CylinderHullSupport hull = default, int triangleIndex = 0)
    {
        int regionStart = coreAxis.IsZero ? 0 : -1;
        for (int region = regionStart; region <= (coreAxis.IsZero ? 0 : 1); region += 2)
        {
            TriangleCircularGeometry endpointStorage;
            scoped ref readonly TriangleCircularGeometry endpoint = ref geometry;
            if (region != 0)
            {
                endpointStorage = geometry.AtCoreRegion(coreOffset, region);
                endpoint = ref endpointStorage;
            }
            for (int edge = 0; edge < 3; edge++)
            {
                WideDirection3d e = endpoint.Edge(edge);
                // Parallel edges project a circle; perpendicular edges project a
                // line segment. Their extrema are already analytic fan boundaries.
                if (e.Y.IsZero || e.X.IsZero && e.Z.IsZero)
                    continue;
                GetBasis(e, out WideDirection3d u, out WideDirection3d v);
                for (int firstSign = -1; firstSign <= 1; firstSign += 2)
                    for (int secondSign = -1; secondSign <= 1; secondSign += 2)
                        for (int capSign = -1; capSign <= 1; capSign += 2)
                        {
                            WideDirection3d first = firstSign > 0 ? u : -u;
                            WideDirection3d second = secondSign > 0 ? v : -v;
                            if (!TryCharts(endpoint, coreAxis, region, edge, first, second, capSign, analytic,
                                    ((ulong)analyticDepth.m_rawValue << 1) | 1UL, !analyticClamped,
                                    bestValues, bestSigns, bestCell, ref best, hull, triangleIndex))
                                return false;
                        }
            }
        }
        return true;
    }

    [MethodImpl(MethodImplOptions.NoInlining)]
    private static bool TryCharts(in TriangleCircularGeometry geometry, in WideDirection3d coreAxis, int region, int edge,
        in WideDirection3d originalFirst, in WideDirection3d originalSecond, int cap, scoped ConvexContactCandidate analytic,
        ulong upperTwiceRaw, bool hasUpperBound,
        scoped Span<ulong> bestValues, scoped Span<sbyte> bestSigns, scoped Span<ulong> bestCell,
        ref Selection best, in CylinderHullSupport hull, int triangleIndex)
    {
        int Words = geometry.ParameterWords;
        scoped ref readonly WideDirection3d first = ref originalFirst;
        scoped ref readonly WideDirection3d second = ref originalSecond;
        bool firstAdmitted = IsChartAdmitted(geometry, coreAxis, region, edge, first, second, cap);
        bool secondAdmitted = IsChartAdmitted(geometry, coreAxis, region, edge, second, first, cap);
        if (!firstAdmitted && !secondAdmitted) return true;
        if (!firstAdmitted)
        {
            first = ref originalSecond;
            second = ref originalFirst;
        }
        Span<ulong> data = stackalloc ulong[ParameterSlots * Words];
        Span<sbyte> signs = stackalloc sbyte[ParameterSlots];
        BuildParameter(geometry, coreAxis, region, edge, first, second, cap, data, signs);
        if (!TryChart(geometry, coreAxis, region, edge, first, second, cap, analytic, upperTwiceRaw, hasUpperBound,
                data, signs, bestValues, bestSigns, bestCell, ref best, hull, triangleIndex)) return false;
        if (!firstAdmitted || !secondAdmitted) return true;
        // Finish the first chart before mutation; winners already own copied
        // value algebra. Preserve its earlier canonical rank when depths tie.
        ReverseParameterChart(data, signs);
        WriteAdmission(geometry, coreAxis, region, edge, second, first, data, signs);
        return TryChart(geometry, coreAxis, region, edge, second, first, cap, analytic, upperTwiceRaw, hasUpperBound,
            data, signs, bestValues, bestSigns, bestCell, ref best, hull, triangleIndex);
    }

    private static bool IsChartAdmitted(in TriangleCircularGeometry geometry, in WideDirection3d coreAxis, int region, int edge,
        in WideDirection3d first, in WideDirection3d second, int cap)
    {

        // Each admission projection is affine on t in (0,1] and must be
        // positive. Nonpositive endpoints exclude every root before construction.
        // GetBasis gives exactly one nonzero Y component. Its cap projection
        // cannot change sign inside this chart, so this check fully admits it.
        if (first.Y.Sign * cap <= 0 && WideArithmetic.AddSigned576(first.Y, second.Y).Sign * cap <= 0)
            return false;
        WideDirection3d outward = TriangleCircularGeometry.Subtract(geometry.Vertex(edge), geometry.Vertex((edge + 2) % 3));
        Signed832 firstCone = WideDirection3d.Dot(first, outward), secondCone = WideDirection3d.Dot(second, outward);
        if (firstCone.Sign <= 0 && WideArithmetic.AddSigned832(firstCone, secondCone).Sign <= 0)
            return false;
        if (!coreAxis.IsZero)
        {
            Signed832 firstCore = WideDirection3d.Dot(coreAxis, first), secondCore = WideDirection3d.Dot(coreAxis, second);
            if (firstCore.Sign * region <= 0 && WideArithmetic.AddSigned832(firstCore, secondCore).Sign * region <= 0)
                return false;
        }
        return true;
    }

    [MethodImpl(MethodImplOptions.NoInlining)]
    private static bool TryChart(in TriangleCircularGeometry geometry, in WideDirection3d coreAxis, int region, int edge,
        in WideDirection3d first, in WideDirection3d second, int cap, scoped ConvexContactCandidate analytic,
        ulong upperTwiceRaw, bool hasUpperBound, scoped Span<ulong> data, scoped Span<sbyte> signs,
        scoped Span<ulong> bestValues, scoped Span<sbyte> bestSigns, scoped Span<ulong> bestCell,
        ref Selection best, in CylinderHullSupport hull, int triangleIndex)
    {
        int Words = geometry.ParameterWords;
        int ValueWords = geometry.ValueWords;
        int ValueCellWords = (16 * (geometry.ValueWords * 64 + 11) + 255) / 64;
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
            if (!hull.IsEmpty && !SupportsHull(hull, geometry.Vertex(edge), first, second, ref root))
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
            if (!hull.IsEmpty && !TriangleCylinderRimWitnesses.IsRootPointOnEdge(geometry, edge, first, second, ref root))
                continue;
            if (best.IsZero || analytic.GapSign == 0)
                continue;
            var candidate = new Selection
            {
                HasValue = true, First = first, Second = second, Edge = edge,
                Cap = cap, CoreSign = region, ParameterOrdinal = ordinal, Triangle = triangleIndex
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
                BuildValues(geometry.Edge(edge), geometry.CapOffset(edge, cap), geometry.Radius, geometry.ValueShift, values, valueSigns, useRigidBounds: !geometry.IsAffine);
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

    private static void BuildParameter(in TriangleCircularGeometry geometry, in WideDirection3d coreAxis, int region, int edge,
        in WideDirection3d first, in WideDirection3d second, int cap, Span<ulong> data, Span<sbyte> signs)
    {

        CircularRimContactAlgebra.BuildParameter(geometry.CapOffset(edge, cap), geometry.Radius, geometry.RawScale,
            first, second, data, signs, useRigidBounds: !geometry.IsAffine);
        WriteAdmission(geometry, coreAxis, region, edge, first, second, data, signs);
    }

    private static void WriteAdmission(in TriangleCircularGeometry geometry, in WideDirection3d coreAxis, int region, int edge,
        in WideDirection3d first, in WideDirection3d second, Span<ulong> data, Span<sbyte> signs)
    {

        WideDirection3d outward = TriangleCircularGeometry.Subtract(geometry.Vertex(edge), geometry.Vertex((edge + 2) % 3));
        Write(data, signs, 26, WideDirection3d.Dot(outward, first));
        Write(data, signs, 27, WideDirection3d.Dot(outward, second));
        if (!coreAxis.IsZero)
        {
            Signed832 firstCore = WideDirection3d.Dot(coreAxis, first), secondCore = WideDirection3d.Dot(coreAxis, second);
            Write(data, signs, 28, region < 0 ? WideArithmetic.SubtractSigned832(default, firstCore) : firstCore);
            Write(data, signs, 29, region < 0 ? WideArithmetic.SubtractSigned832(default, secondCore) : secondCore);
        }
    }

    [MethodImpl(MethodImplOptions.NoInlining)]
    internal static void GetMaterials(in TriangleCircularGeometry geometry, in WideDirection3d coreAxis,
        Vector2d coreDirection, Fixed64 coreLength, FixedTriangle triangle, in Selection best, Fixed64 radius,
        out Vector3d normal, out Vector3d radialPoint, out Vector3d trianglePoint, out int integralRadialMask)
    {
        int Words = geometry.ParameterWords;
        Span<ulong> data = stackalloc ulong[ParameterSlots * Words];
        Span<sbyte> signs = stackalloc sbyte[ParameterSlots];
        BuildParameter(geometry, coreAxis, best.CoreSign, best.Edge, best.First, best.Second, best.Cap, data, signs);
        Span<ulong> cell = stackalloc ulong[WideFiniteAxisIntersection.GetFiniteValueRootCellWords(data[..(5 * Words)], signs[..5])];
        bool found = WideFiniteAxisIntersection.TryGetFiniteValueRoot(data[..(5 * Words)], signs[..5],
            best.ParameterOrdinal, cell, out FiniteAxisValueRoot root);
        System.Diagnostics.Debug.Assert(found);
        trianglePoint = TriangleCylinderRimWitnesses.GetRootPoint(geometry, triangle, best.Edge, best.First, best.Second, ref root);
        Span<ulong> gradient = stackalloc ulong[54];
        Span<sbyte> gradientSigns = stackalloc sbyte[6];
        WriteGradient(best.First, best.Second, gradient, gradientSigns);
        gradient.Slice(18, 18).Clear(); gradientSigns.Slice(2, 2).Clear();
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

    internal static FiniteAxisValueRoot Restore(ReadOnlySpan<ulong> values, ReadOnlySpan<sbyte> signs,
        Span<ulong> cell, in Selection selection) => new()
    {
        Coefficients = values, Signs = signs, LowerNumerator = cell,
        DenominatorShift = selection.ValueShift, Ordinal = selection.ValueOrdinal, IsRational = selection.IsRational
    };

    private static bool SupportsHull(in CylinderHullSupport hull, in WideDirection3d point,
        in WideDirection3d first, in WideDirection3d second, ref FiniteAxisValueRoot root)
    {
        Span<ulong> query = stackalloc ulong[26]; Span<sbyte> signs = stackalloc sbyte[2];
        for (int i = 0; i < hull.Count; i++)
        {
            WideDirection3d difference = TriangleCircularGeometry.Subtract(point, hull.Vertex(i));
            Signed832 a = WideDirection3d.Dot(difference, first), b = WideDirection3d.Dot(difference, second);
            WideArithmetic.GetMagnitude(a, query[..13]); signs[0] = (sbyte)a.Sign;
            WideArithmetic.GetMagnitude(b, query[13..]); signs[1] = (sbyte)b.Sign;
            if (WideFiniteAxisIntersection.GetSignAtFiniteValueRootAndRefine(ref root, query, signs) < 0) return false;
        }
        return true;
    }
}
