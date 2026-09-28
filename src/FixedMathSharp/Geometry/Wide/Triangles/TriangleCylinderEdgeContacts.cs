//=======================================================================
// MIT License, Copyright (c) 2024-present David Oravsky (mrdav30)
// See LICENSE file in the project root for full license information.
//=======================================================================
using System;
using System.Runtime.CompilerServices;
using static FixedMathSharp.Geometry.CylinderContactAlgebra;

namespace FixedMathSharp.Geometry;

/// <summary>Every admitted triangle-edge/cylinder-rim stationary support root.</summary>
internal static class TriangleCylinderEdgeContacts
{
    private const int ValueWords = 56;
    private const int ParameterSlots = 30;
    private const int ValueCellWords = (16 * (ValueWords * 64 + 11) + 255) / 64;

    private struct Selection
    {
        internal bool HasValue, IsZero, IsRational;
        internal WideAxis3 First, Second;
        internal int Edge, Cap, ParameterOrdinal, ValueOrdinal, ValueShift;
    }

    internal static bool TryGetContact(in TriangleCylinderGeometry geometry, FixedTriangle triangle,
        in ConvexContactCandidate analytic, Fixed64 radius, out bool hasBetter, out Vector3d normal,
        out Vector3d radialPoint, out Vector3d trianglePoint, out Fixed64 depth, out bool clamped, out int mask, out int cap)
    {
        hasBetter = false; normal = radialPoint = trianglePoint = default; depth = default;
        clamped = false; mask = cap = 0;
        Span<ulong> bestValues = stackalloc ulong[5 * ValueWords];
        Span<sbyte> bestSigns = stackalloc sbyte[5];
        Span<ulong> bestCell = stackalloc ulong[ValueCellWords];
        Selection best = default;
        for (int edge = 0; edge < 3; edge++)
        {
            WideAxis3 e = geometry.Edge(edge);
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
                            if (!TryChart(geometry, edge, first, second, capSign, analytic,
                                    bestValues, bestSigns, bestCell, ref best))
                                return false;
                        }
        }
        if (!best.HasValue)
            return true;
        hasBetter = true; mask = (1 << best.Edge) | (1 << ((best.Edge + 1) % 3)); cap = best.Cap;
        GetMaterials(geometry, triangle, best, radius, out normal, out radialPoint, out trianglePoint);
        if (!best.IsZero)
        {
            FiniteAxisValueRoot root = Restore(bestValues, bestSigns, bestCell, best);
            ConvexContactValueRoot.GetRoundedDepth(geometry.RawScale, geometry.ValueShift,
                ref root, out depth, out clamped);
        }
        return true;
    }

    [MethodImpl(MethodImplOptions.NoInlining)]
    private static bool TryChart(in TriangleCylinderGeometry geometry, int edge,
        WideAxis3 first, WideAxis3 second, int cap, scoped ConvexContactCandidate analytic,
        scoped Span<ulong> bestValues, scoped Span<sbyte> bestSigns, scoped Span<ulong> bestCell,
        ref Selection best)
    {
        WideAxis3 outward = TriangleCylinderGeometry.Subtract(geometry.Vertex(edge), geometry.Vertex((edge + 2) % 3));
        if (first.Y.Sign * cap < 0 && WideArithmetic.AddSigned320(first.Y, second.Y).Sign * cap <= 0)
            return true;
        Signed576 firstCone = WideAxis3.Dot(first, outward), secondCone = WideAxis3.Dot(second, outward);
        if (firstCone.Sign < 0 && WideArithmetic.AddSigned576(firstCone, secondCone).Sign <= 0)
            return true;

        Span<ulong> data = stackalloc ulong[ParameterSlots * Words];
        Span<sbyte> signs = stackalloc sbyte[ParameterSlots];
        BuildParameter(geometry, edge, first, second, cap, data, signs);
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
            // Zero cap alignment is the already-ranked edge/side axis;
            // zero third-vertex alignment is the already-ranked face normal.
            // Neither boundary can improve that earlier canonical feature.
            if (Sign(ref root, data, signs, 26, 2) <= 0)
                continue;
            if (Sign(ref root, data, signs, 28, 2) <= 0)
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
                Cap = cap, ParameterOrdinal = ordinal
            };
            if (gapSign == 0)
            {
                candidate.IsZero = true; best = candidate;
                continue;
            }
            if (!valuesReady)
            {
                BuildValues(geometry, edge, cap, values, valueSigns);
                valuesReady = true;
            }
            scoped FiniteAxisValueRoot value = ConvexContactValueRoot.MapSquaredValue(
                geometry.RawScale, geometry.ValueShift, ref root,
                data.Slice(16 * Words, 5 * Words), signs.Slice(16, 5),
                data.Slice(21 * Words, 5 * Words), signs.Slice(21, 5),
                values, valueSigns, valueCell);
            int comparison = best.HasValue
                ? WideFiniteAxisIntersection.CompareFiniteValueRoots(value, Restore(bestValues, bestSigns, bestCell, best))
                : ConvexContactValueRoot.CompareRootSquared(geometry.RawScale, geometry.ValueShift, value, analytic);
            if (comparison >= 0)
                continue;
            values.CopyTo(bestValues); valueSigns.CopyTo(bestSigns); valueCell.CopyTo(bestCell);
            candidate.ValueOrdinal = value.Ordinal; candidate.ValueShift = value.DenominatorShift;
            candidate.IsRational = value.IsRational; best = candidate;
        }
        return true;
    }

    [MethodImpl(MethodImplOptions.NoInlining)]
    private static void BuildParameter(in TriangleCylinderGeometry geometry, int edge,
        WideAxis3 first, WideAxis3 second, int cap, Span<ulong> data, Span<sbyte> signs)
    {
        // Coordinates and differences are <2^198; chart directions reuse edge
        // components (no cross-cross growth). M<2^398, P<2^399, Q<2^797,
        // K<2^800, L<2^1198, and W,N,D<2^2403. Forty words give 2560 bits.
        Span<ulong> inputs = stackalloc ulong[11 * Words];
        Span<sbyte> inputSigns = stackalloc sbyte[9];
        inputs.Clear(); inputSigns.Clear();
        Span<ulong> p = inputs[..(2 * Words)], q = inputs.Slice(2 * Words, 3 * Words);
        Span<ulong> metric = inputs.Slice(5 * Words, 3 * Words), scale = Slot(inputs, 8);
        Span<ulong> radiusSquared = Slot(inputs, 9), product = Slot(inputs, 10);
        WideAxis3 c = geometry.CapOffset(edge, cap);
        Write(p, inputSigns[..2], 0, WideAxis3.Dot(c, first));
        Write(p, inputSigns[..2], 1, WideAxis3.Dot(c, second));
        Write(metric, inputSigns.Slice(5, 3), 0, first.SquaredLength);
        Write(metric, inputSigns.Slice(5, 3), 1, Twice(WideAxis3.Dot(first, second)));
        Write(metric, inputSigns.Slice(5, 3), 2, second.SquaredLength);
        Write(q, inputSigns.Slice(2, 3), 0, RadialDot(first, first));
        Write(q, inputSigns.Slice(2, 3), 1, Twice(RadialDot(first, second)));
        Write(q, inputSigns.Slice(2, 3), 2, RadialDot(second, second));
        Import(WideArithmetic.MultiplySigned320(geometry.Radius, geometry.Radius), radiusSquared);
        for (int index = 0; index < 3; index++)
        {
            WideArithmetic.MultiplyMagnitudes(Slot(q, index), radiusSquared, product);
            product.CopyTo(Slot(q, index));
        }
        Import(Signed320.ExtendValue(geometry.RawScale), scale);
        CylinderEdgeContactPolynomial.Build(p, inputSigns[..2], q, inputSigns.Slice(2, 3),
            metric, inputSigns.Slice(5, 3), scale, data[..(26 * Words)], signs[..26]);
        Write(data, signs, 26, Signed576.ExtendValue(cap > 0 ? first.Y : WideArithmetic.Negate(first.Y)));
        Write(data, signs, 27, Signed576.ExtendValue(cap > 0 ? second.Y : WideArithmetic.Negate(second.Y)));
        WideAxis3 outward = TriangleCylinderGeometry.Subtract(geometry.Vertex(edge), geometry.Vertex((edge + 2) % 3));
        Write(data, signs, 28, WideAxis3.Dot(outward, first));
        Write(data, signs, 29, WideAxis3.Dot(outward, second));
    }

    [MethodImpl(MethodImplOptions.NoInlining)]
    private static void BuildValues(in TriangleCylinderGeometry geometry, int edge, int cap,
        Span<ulong> values, Span<sbyte> signs)
    {
        WideAxis3 e = geometry.Edge(edge), c = geometry.CapOffset(edge, cap);
        Span<ulong> invariants = stackalloc ulong[8 * ValueWords];
        Span<sbyte> invariantSigns = stackalloc sbyte[8];
        invariants.Clear(); invariantSigns.Clear();
        Write(invariants, invariantSigns, 0, Signed576.ExtendValue(Signed320.ExtendValue(Signed192.Signed(1))), ValueWords);
        Write(invariants, invariantSigns, 1, e.SquaredLength, ValueWords);
        Write(invariants, invariantSigns, 2, Signed576.ExtendValue(e.Y), ValueWords);
        Write(invariants, invariantSigns, 3, c.SquaredLength, ValueWords);
        Write(invariants, invariantSigns, 4, Signed576.ExtendValue(c.Y), ValueWords);
        Write(invariants, invariantSigns, 5, WideAxis3.Dot(e, c), ValueWords);
        Write(invariants, invariantSigns, 6, WideArithmetic.MultiplySigned320(geometry.Radius, geometry.Radius), ValueWords);
        CylinderPairSideValuePolynomial.BuildUnshifted(invariants, invariantSigns, ValueWords, values, signs);
        // U=1; B,C²,q,y,rho have at most 400 bits. The three linear
        // invariants have weighted height <805 after z=2^ValueShift*t,
        // ValueShift<=402. The quartic summands and their carry fit 3240 bits,
        // below the shared 56-word (3584-bit) value-polynomial storage.
        WideFiniteAxisIntersection.ScaleFiniteAxisPolynomialVariable(values, 5, geometry.ValueShift);
        WideFiniteAxisIntersection.NormalizeFiniteAxisPolynomialPowerOfTwo(values, signs);
    }

    [MethodImpl(MethodImplOptions.NoInlining)]
    private static void GetMaterials(in TriangleCylinderGeometry geometry, FixedTriangle triangle, Selection best, Fixed64 radius,
        out Vector3d normal, out Vector3d radialPoint, out Vector3d trianglePoint)
    {
        Span<ulong> data = stackalloc ulong[ParameterSlots * Words];
        Span<sbyte> signs = stackalloc sbyte[ParameterSlots];
        BuildParameter(geometry, best.Edge, best.First, best.Second, best.Cap, data, signs);
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

    internal static void GetBasis(WideAxis3 edge, out WideAxis3 first, out WideAxis3 second)
    {
        if (!edge.X.IsZero)
        {
            first = new WideAxis3(WideArithmetic.Negate(edge.Y), edge.X, default);
            second = new WideAxis3(WideArithmetic.Negate(edge.Z), default, edge.X);
        }
        else
        {
            first = new WideAxis3(default, WideArithmetic.Negate(edge.Z), edge.Y);
            second = new WideAxis3(edge.Y, default, default);
        }
    }

    private static void WriteGradient(WideAxis3 first, WideAxis3 second, Span<ulong> values, Span<sbyte> signs)
    {
        values.Clear();
        for (int component = 0; component < 3; component++)
        {
            Signed320 a = Component(first, component), b = Component(second, component);
            Span<ulong> firstSlot = values.Slice(component * 10, 5), secondSlot = values.Slice(component * 10 + 5, 5);
            WideArithmetic.GetMagnitude(a, out firstSlot[4], out firstSlot[3], out firstSlot[2], out firstSlot[1], out firstSlot[0]);
            WideArithmetic.GetMagnitude(b, out secondSlot[4], out secondSlot[3], out secondSlot[2], out secondSlot[1], out secondSlot[0]);
            signs[2 * component] = (sbyte)a.Sign; signs[2 * component + 1] = (sbyte)b.Sign;
        }
    }
    private static WideAxis3 Transform(WideRationalBasis3d basis, WideAxis3 axis) => new(
        TransformComponent(basis.Xx, basis.Yx, basis.Zx, axis),
        TransformComponent(basis.Xy, basis.Yy, basis.Zy, axis),
        TransformComponent(basis.Xz, basis.Yz, basis.Zz, axis));
    private static Signed320 TransformComponent(Signed192 x, Signed192 y, Signed192 z, WideAxis3 axis) =>
        Signed320.NarrowValue(WideAxis3.Dot(new WideAxis3(Signed320.ExtendValue(x), Signed320.ExtendValue(y), Signed320.ExtendValue(z)), axis));
    private static Signed576 RadialDot(WideAxis3 a, WideAxis3 b) => WideArithmetic.AddSigned576(
        WideArithmetic.MultiplySigned320(a.X, b.X), WideArithmetic.MultiplySigned320(a.Z, b.Z));
    private static Signed576 Twice(Signed576 value) => WideArithmetic.AddSigned576(value, value);
    private static int Sign(ref FiniteAxisValueRoot root, ReadOnlySpan<ulong> data, ReadOnlySpan<sbyte> signs, int start, int count) =>
        WideFiniteAxisIntersection.GetSignAtFiniteValueRootAndRefine(ref root, data.Slice(start * Words, count * Words), signs.Slice(start, count));
    private static void Write(Span<ulong> values, Span<sbyte> signs, int index, Signed576 value, int words = Words)
    {
        Span<ulong> target = values.Slice(index * words, words);
        target.Clear(); WideArithmetic.GetMagnitude(value, target[..9]); signs[index] = (sbyte)value.Sign;
    }
    private static FiniteAxisValueRoot Restore(ReadOnlySpan<ulong> values, ReadOnlySpan<sbyte> signs,
        Span<ulong> cell, Selection selection) => new()
    {
        Coefficients = values, Signs = signs, LowerNumerator = cell,
        DenominatorShift = selection.ValueShift, Ordinal = selection.ValueOrdinal, IsRational = selection.IsRational
    };
}
