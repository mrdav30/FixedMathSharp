//=======================================================================
// MIT License, Copyright (c) 2024-present David Oravsky (mrdav30)
// See LICENSE file in the project root for full license information.
//=======================================================================
using System;
using System.Runtime.CompilerServices;

namespace FixedMathSharp.Geometry;

/// <summary>Exactly ranked stationary real-prism-edge/circular-rim support gaps.</summary>
internal static class CurvedPrismRimContacts
{
    private const int Words = 40;
    private const int Slots = 31;
    private const int ValueWords = 56;
    private const int ValueCellWords = (16 * (ValueWords * 64 + 11) + 255) / 64;

    internal struct Selection
    {
        internal bool HasValue, IsZero;
        internal int Edge, CapSign, ParameterOrdinal;
        internal WideDirection3d First, Second;
        internal bool IsRational;
        internal int ValueOrdinal, ValueShift;
    }

    /// <summary>
    /// Retains stationary minima after complete closed intersection admission
    /// and the rational, vertex and generator boundaries. Exact ties retain
    /// those earlier features; every admitted support gap is nonnegative.
    /// </summary>
    internal static void GetContact(in CurvedPrismContactGeometry geometry,
        scoped ConvexContactCandidate analytic, Fixed64 analyticDepth, bool analyticClamped,
        out Selection best, out Fixed64 depth, out bool clamped)
    {
        best = default; depth = analyticDepth; clamped = analyticClamped;
        // A radius-zero primitive is a finite segment, owned by the rational fan.
        if (geometry.Radius.IsZero) return;
        Span<ulong> bestValues = stackalloc ulong[5 * ValueWords];
        Span<sbyte> bestSigns = stackalloc sbyte[5];
        Span<ulong> bestCell = stackalloc ulong[ValueCellWords];
        WideDirection3d axis = new(geometry.Axis);
        for (int edgeIndex = 0; edgeIndex < geometry.EdgeCount; edgeIndex++)
        {
            geometry.Edge(edgeIndex, out int start, out int end);
            WideDirection3d edge = CurvedPrismContact.Difference(geometry.Vertex(end), geometry.Vertex(start));
            // Parallel projection is circular; perpendicular projection is a
            // segment. Their complete boundaries and principal directions are
            // in the earlier rational/generator fan, not this ellipse quartic.
            if (edge.IsZero || WideDirection3d.Dot(edge, axis).IsZero
                || WideDirection3d.AreParallel(edge, axis))
                continue;
            edge = ReduceEdgeDirection(edge);
            CircularRimContactAlgebra.GetBasis(edge, out WideDirection3d u, out WideDirection3d v);
            for (int cap = -1; cap <= (geometry.IsCone ? -1 : 1); cap += 2)
                for (int firstSign = -1; firstSign <= 1; firstSign += 2)
                    for (int secondSign = -1; secondSign <= 1; secondSign += 2)
                        for (int chart = 0; chart < 2; chart++)
                        {
                            WideDirection3d first = firstSign < 0 ? -u : u;
                            WideDirection3d second = secondSign < 0 ? -v : v;
                            if (chart != 0) (first, second) = (second, first);
                            KeepChart(geometry, edgeIndex, edge, first, second, cap, analytic,
                                ((ulong)analyticDepth.m_rawValue << 1) | 1UL, !analyticClamped,
                                bestValues, bestSigns, bestCell, ref best);
                            if (best.IsZero)
                            {
                                depth = Fixed64.Zero; clamped = false; return;
                            }
                        }
        }
        if (!best.HasValue) return;
        FiniteAxisValueRoot root = RestoreValue(bestValues, bestSigns, bestCell, best);
        ConvexContactValueRoot.GetRoundedDepth(geometry.RawScale, geometry.ValueShift,
            ref root, out depth, out clamped);
    }

    private static WideDirection3d ReduceEdgeDirection(in WideDirection3d edge)
    {
        // Real edge components are below 166 bits. One positive content scales
        // both linear chart basis vectors equally, preserving t and its root
        // ordinal. N and D scale by the same eighth power; the normal and
        // witness ratios are unchanged. BuildValues keeps the original edge.
        Signed192 x = Signed192.NarrowValue(edge.X), y = Signed192.NarrowValue(edge.Y), z = Signed192.NarrowValue(edge.Z);
        Signed192 divisor = WideArithmetic.GetGreatestCommonDivisor(
            WideArithmetic.GetGreatestCommonDivisor(x, y), z);
        return new WideDirection3d(
            Signed576.ExtendValue(WideArithmetic.DivideExactSigned192(x, divisor)),
            Signed576.ExtendValue(WideArithmetic.DivideExactSigned192(y, divisor)),
            Signed576.ExtendValue(WideArithmetic.DivideExactSigned192(z, divisor)));
    }

    [MethodImpl(MethodImplOptions.NoInlining)]
    private static void KeepChart(in CurvedPrismContactGeometry geometry, int edge,
        in WideDirection3d edgeDirection, in WideDirection3d first, in WideDirection3d second, int cap,
        scoped ConvexContactCandidate analytic, ulong upperTwiceRaw, bool hasUpperBound,
        scoped Span<ulong> bestValues, scoped Span<sbyte> bestSigns, scoped Span<ulong> bestCell,
        ref Selection best)
    {
        if (!geometry.IsCone)
        {
            Signed832 axial = WideDirection3d.Dot(first, geometry.Axis);
            Signed832 slope = WideDirection3d.Dot(second, geometry.Axis);
            // Nonpositive endpoints leave only an analytic side-boundary
            // endpoint. Identically zero would mean the excluded parallel edge.
            if (axial.Sign * cap <= 0 && WideArithmetic.AddSigned832(axial, slope).Sign * cap <= 0) return;
        }
        geometry.Edge(edge, out int start, out _);
        WideDirection3d vertex = geometry.Vertex(start);
        // One strictly negative support-wall value at both chart endpoints
        // excludes this entire affine chart before constructing its quartic.
        for (int index = 0; index < geometry.VertexCount; index++)
        {
            WideDirection3d delta = CurvedPrismContact.Difference(geometry.Vertex(index), vertex);
            Signed832 a = WideDirection3d.Dot(delta, first), b = WideDirection3d.Dot(delta, second);
            if (a.Sign < 0 && WideArithmetic.AddSigned832(a, b).Sign < 0) return;
        }
        WideDirection3d offset = CurvedPrismContact.Difference(cap < 0 ? -geometry.Half : geometry.Half, vertex);
        if (IsChartDominated(geometry, edgeDirection, offset, first, second, analytic)) return;
        Span<ulong> data = stackalloc ulong[Slots * Words];
        Span<sbyte> signs = stackalloc sbyte[Slots];
        BuildParameter(geometry, edge, first, second, cap, data, signs);
        Polynomial storage = new(data, signs, Words);
        Polynomial stationary = storage.Slice(0, 5), k = storage.Slice(8, 2), l = storage.Slice(10, 3);
        Polynomial gap = storage.Slice(13, 3), numerator = storage.Slice(16, 5), denominator = storage.Slice(21, 5);
        Span<ulong> batchCells = stackalloc ulong[64];
        Span<int> batchShifts = stackalloc int[8];
        FiniteAxisValueRoots roots = WideFiniteAxisIntersection.GetFiniteValueRoots(
            stationary.Values, stationary.Signs, batchCells, batchShifts);
        Span<ulong> cell = stackalloc ulong[WideFiniteAxisIntersection.GetFiniteValueRootCellWords(
            stationary.Values, stationary.Signs)];
        Span<ulong> values = stackalloc ulong[5 * ValueWords];
        Span<sbyte> valueSigns = stackalloc sbyte[5];
        Span<ulong> valueCell = stackalloc ulong[ValueCellWords];
        bool valuesReady = false;
        for (int ordinal = 0; ordinal < roots.Count; ordinal++)
        {
            scoped FiniteAxisValueRoot root = FiniteAxisValueRoots.GetRoot(
                roots, ordinal, stationary.Values, stationary.Signs, cell);
            int axialSign = Sign(ref root, storage.Slice(26, 2));
            if (geometry.IsCone
                ? axialSign > 0 && Sign(ref root, storage.Slice(28, 3)) <= 0
                : axialSign < 0)
                continue;
            if (!HasPrismSupport(geometry, vertex, first, second, ref root)) continue;
            int kSign = Sign(ref root, k);
            // Q is positive on a nondegenerate ellipse. K=0 is its principal
            // direction, already present in the rational feature fan.
            if (kSign == 0 || Sign(ref root, l) != -kSign) continue;
            int gapSign = Sign(ref root, gap) * kSign;
            System.Diagnostics.Debug.Assert(gapSign >= 0);
            Selection candidate = new()
            {
                HasValue = true, Edge = edge, CapSign = cap, First = first, Second = second,
                ParameterOrdinal = ordinal
            };
            if (gapSign == 0)
            {
                candidate.IsZero = true; best = candidate; return;
            }
            if (hasUpperBound && ConvexContactValueRoot.CompareSquaredGapToTwiceRaw(ref root,
                    numerator.Values, numerator.Signs, denominator.Values, denominator.Signs, upperTwiceRaw) >= 0)
                continue;
            if (!valuesReady)
            {
                BuildValues(geometry, edge, cap, values, valueSigns); valuesReady = true;
            }
            scoped FiniteAxisValueRoot value = ConvexContactValueRoot.MapSquaredValue(
                geometry.RawScale, geometry.ValueShift, ref root, numerator.Values, numerator.Signs,
                denominator.Values, denominator.Signs, values, valueSigns, valueCell);
            int comparison = best.HasValue
                ? WideFiniteAxisIntersection.CompareFiniteValueRoots(value, RestoreValue(bestValues, bestSigns, bestCell, best))
                : ConvexContactValueRoot.CompareRootSquared(geometry.RawScale, geometry.ValueShift, ref value, analytic);
            if (comparison >= 0) continue;
            values.CopyTo(bestValues); valueSigns.CopyTo(bestSigns); valueCell.CopyTo(bestCell);
            candidate.ValueOrdinal = value.Ordinal; candidate.ValueShift = value.DenominatorShift;
            candidate.IsRational = value.IsRational; best = candidate;
        }
    }

    [MethodImpl(MethodImplOptions.NoInlining)]
    private static bool IsChartDominated(in CurvedPrismContactGeometry geometry,
        in WideDirection3d edge, in WideDirection3d offset, in WideDirection3d first,
        in WideDirection3d second, scoped ConvexContactCandidate analytic)
    {
        WideDirection3d last = new(WideArithmetic.AddSigned576(first.X, second.X),
            WideArithmetic.AddSigned576(first.Y, second.Y), WideArithmetic.AddSigned576(first.Z, second.Z));
        Signed832 a = WideDirection3d.Dot(offset, first), b = WideDirection3d.Dot(offset, last);
        if (a.Sign < 0 || b.Sign < 0) return false;
        Signed832 firstMetric = first.SquaredLength, lastMetric = last.SquaredLength;
        Signed832 aSquared = WideArithmetic.MultiplySigned576ToSigned832(Signed576.NarrowValue(a), Signed576.NarrowValue(a));
        Signed832 bSquared = WideArithmetic.MultiplySigned576ToSigned832(Signed576.NarrowValue(b), Signed576.NarrowValue(b));
        bool useLast = WideArithmetic.CompareNonNegativeProducts(aSquared, lastMetric, bSquared, firstMetric) > 0;
        Signed832 alphaNumerator = useLast ? b : a;
        Signed832 alphaSquared = useLast ? bSquared : aSquared;
        Signed832 alphaMetric = useLast ? lastMetric : firstMetric;

        // n(t)=(1-t)*first+t*last. Nonnegative endpoint projections and
        // the triangle inequality give offset.n/|n| >= alpha, the smaller
        // endpoint projection divided by its exact Euclidean norm.
        // On edge-perpendicular normals, the projected disk support is at
        // least beta=R*|Axis.edge|/sqrt(q*edge²). Thus (alpha+beta)/S bounds
        // every gap from below; beta>0 follows from the outer ellipse dispatch.
        Span<ulong> values = stackalloc ulong[ConvexContactCandidate.Slots * Words];
        Span<int> signs = stackalloc int[ConvexContactCandidate.Slots];
        values.Clear(); signs.Clear(); values[0] = 1; signs[0] = 1;
        Span<ulong> work = stackalloc ulong[6 * Words];
        Span<ulong> projection = work[..Words], radial = work.Slice(Words, Words);
        Span<ulong> metric = work.Slice(2 * Words, Words), k = work.Slice(3 * Words, Words);
        Span<ulong> temporary = work.Slice(4 * Words, Words), product = work.Slice(5 * Words, Words);
        Span<ulong> rational = values.Slice(7 * Words, Words), radical = values.Slice(8 * Words, Words);
        Span<ulong> root = values.Slice(9 * Words, Words), denominator = values.Slice(10 * Words, Words);
        CylinderContactAlgebra.Import(alphaNumerator, projection);
        CylinderContactAlgebra.Import(alphaMetric, metric);
        CylinderContactAlgebra.Import(WideArithmetic.MultiplySigned832(
            WideDirection3d.Dot(edge, geometry.Axis), geometry.Radius), radial);
        CylinderContactAlgebra.Import(WideArithmetic.MultiplySigned832(edge.SquaredLength, geometry.SquaredAxis), k);
        CylinderContactAlgebra.Import(alphaSquared, temporary);
        WideArithmetic.MultiplyMagnitudes(temporary, k, rational);
        WideArithmetic.MultiplyMagnitudes(radial, radial, temporary);
        WideArithmetic.MultiplyMagnitudes(temporary, metric, product);
        WideArithmetic.AddMagnitudeInto(product, rational);
        WideArithmetic.MultiplyMagnitudes(projection, radial, radical);
        WideArithmetic.AddMagnitudeInto(radical, radical);
        WideArithmetic.MultiplyMagnitudes(k, metric, root);
        CylinderContactAlgebra.Import(Signed576.ExtendValue(geometry.RawScale), projection);
        WideArithmetic.MultiplyMagnitudes(projection, projection, temporary);
        WideArithmetic.MultiplyMagnitudes(temporary, root, denominator);
        signs[7] = 1; signs[8] = alphaNumerator.Sign;
        // With alpha=a/sqrt(M), h=R*|Axis.edge| and K=q*edge², the
        // squared bound is (a²K+h²M+2ah*sqrt(KM))/(KM*S²).
        // a<2^338, M<2^340, h<2^366 and K<2^400; a² fits Signed832.
        // Every squared-candidate field, including sum carries, is <1100 bits.
        // The existing forty-word quadratic candidate owner retains them all.
        return WideConvexPrismRelations.CompareConvexContactCandidates(
            new ConvexContactCandidate(values, signs, 1), analytic) >= 0;
    }

    private static bool HasPrismSupport(in CurvedPrismContactGeometry geometry, in WideDirection3d vertex,
        in WideDirection3d first, in WideDirection3d second, scoped ref FiniteAxisValueRoot root)
    {
        Span<ulong> query = stackalloc ulong[26];
        Span<sbyte> signs = stackalloc sbyte[2];
        for (int index = 0; index < geometry.VertexCount; index++)
        {
            WideDirection3d delta = CurvedPrismContact.Difference(geometry.Vertex(index), vertex);
            Signed832 a = WideDirection3d.Dot(delta, first), b = WideDirection3d.Dot(delta, second);
            // An affine constraint nonnegative at both endpoints holds on the whole chart.
            if (a.Sign >= 0 && WideArithmetic.AddSigned832(a, b).Sign >= 0) continue;
            WideArithmetic.GetMagnitude(a, query[..13]);
            WideArithmetic.GetMagnitude(b, query[13..]);
            signs[0] = (sbyte)a.Sign; signs[1] = (sbyte)b.Sign;
            if (WideFiniteAxisIntersection.GetSignAtFiniteValueRootAndRefine(ref root, query, signs) < 0) return false;
        }
        return true;
    }

    /// <summary>
    /// Writes W,Q,K,L,J,N,D into 0..25; 26..27 are the oriented cap axial
    /// projection (unoriented for a cone), and 28..30 the cone's strict base
    /// switch. All coefficients use forty-word fields.
    /// </summary>
    [MethodImpl(MethodImplOptions.NoInlining)]
    private static void BuildParameter(in CurvedPrismContactGeometry geometry, int edge,
        in WideDirection3d first, in WideDirection3d second, int cap,
        Span<ulong> data, Span<sbyte> signs)
    {
        // |u|<2^33, q<2^65, |e|,|first|,|second|<2^166,
        // |c|<2^167, R<2^163 and S<2^99 give M<2^335,P<2^401,
        // Q<2^792,K<2^740,L<2^1130,W,N<2^2290,D<2^2150.
        // Forty words retain all convolution carries without scalar narrowing.
        Span<ulong> scratch = stackalloc ulong[22 * Words];
        Span<sbyte> scratchSigns = stackalloc sbyte[21];
        scratch.Clear(); scratchSigns.Clear(); data.Clear(); signs.Clear();
        Polynomial work = new(scratch[..(21 * Words)], scratchSigns, Words);
        Polynomial p = work.Slice(0, 2), q = work.Slice(2, 1), radiusSquared = work.Slice(3, 1);
        Polynomial factor = work.Slice(4, 1), axial = work.Slice(5, 2), axialSquared = work.Slice(7, 3);
        Polynomial metric = work.Slice(10, 3), radial = work.Slice(13, 3), radical = work.Slice(16, 3);
        Polynomial scale = work.Slice(19, 1), scaledQ = work.Slice(20, 1);
        Span<ulong> product = scratch.Slice(21 * Words, Words);
        WideDirection3d offset = GetOffset(geometry, edge, cap);
        Write(q, 0, Signed832.ExtendValue(geometry.SquaredAxis));
        Write(radiusSquared, 0, Signed832.ExtendValue(WideArithmetic.MultiplySigned320(geometry.Radius, geometry.Radius)));
        Write(axial, 0, WideDirection3d.Dot(first, geometry.Axis));
        Write(axial, 1, WideDirection3d.Dot(second, geometry.Axis));
        Write(metric, 0, first.SquaredLength);
        Write(metric, 1, Twice(WideDirection3d.Dot(first, second)));
        Write(metric, 2, second.SquaredLength);
        Multiply(radial, q, metric, product);
        Multiply(axialSquared, axial, axial, product); Add(radial, axialSquared, -1);
        Multiply(factor, radiusSquared, q, product);
        Multiply(radical, factor, radial, product);
        Write(p, 0, WideArithmetic.MultiplySigned832(WideDirection3d.Dot(offset, first), geometry.SquaredAxis));
        Write(p, 1, WideArithmetic.MultiplySigned832(WideDirection3d.Dot(offset, second), geometry.SquaredAxis));
        Write(scale, 0, Signed832.ExtendValue(geometry.RawScale));
        Multiply(scaledQ, q, scale, product);
        CylinderEdgeContactPolynomial.Build(p.Values, p.Signs, radical.Values, radical.Signs,
            metric.Values, metric.Signs, scaledQ.Values, data[..(26 * Words)], signs[..26]);
        Polynomial storage = new(data, signs, Words);
        Add(storage.Slice(26, 2), axial, geometry.IsCone ? 1 : cap);
        if (!geometry.IsCone) return;
        Polynomial region = storage.Slice(28, 3);
        Multiply(region, radiusSquared, radial, product);
        // T may grow when Axis is made primitive; T²*q itself is unchanged.
        Write(scale, 0, Signed832.ExtendValue(WideArithmetic.MultiplySigned320(
            Signed320.ExtendValue(geometry.HalfFactor), geometry.HalfFactor)));
        Multiply(scaledQ, q, scale, product);
        Multiply(radical, scaledQ, axialSquared, product); Add(region, radical, -4);
    }

    [MethodImpl(MethodImplOptions.NoInlining)]
    private static void BuildValues(in CurvedPrismContactGeometry geometry, int edge, int cap,
        Span<ulong> values, Span<sbyte> signs)
    {
        geometry.Edge(edge, out int start, out int end);
        WideDirection3d e = CurvedPrismContact.Difference(geometry.Vertex(end), geometry.Vertex(start));
        WideDirection3d c = GetOffset(geometry, edge, cap);
        Span<ulong> invariants = stackalloc ulong[8 * ValueWords];
        Span<sbyte> invariantSigns = stackalloc sbyte[8];
        invariants.Clear(); invariantSigns.Clear();
        Polynomial scalars = new(invariants, invariantSigns, ValueWords);
        Write(scalars, 0, Signed832.ExtendValue(geometry.SquaredAxis));
        Write(scalars, 1, e.SquaredLength);
        Write(scalars, 2, WideDirection3d.Dot(e, geometry.Axis));
        Write(scalars, 3, c.SquaredLength);
        Write(scalars, 4, WideDirection3d.Dot(c, geometry.Axis));
        Write(scalars, 5, WideDirection3d.Dot(e, c));
        Write(scalars, 6, Signed832.ExtendValue(WideArithmetic.MultiplySigned320(geometry.Radius, geometry.Radius)));
        CylinderPairSideValuePolynomial.BuildUnshifted(invariants, invariantSigns, ValueWords, values, signs);
        // Under the parameter bounds, weighted l,m,v heights are <750
        // after z=2^ValueShift*t (ValueShift<=338). Quartic discriminant
        // summands and their carries stay below 3100 bits, within 56 words.
        WideFiniteAxisIntersection.ScaleFiniteAxisPolynomialVariable(values, 5, geometry.ValueShift);
        WideFiniteAxisIntersection.NormalizeFiniteAxisPolynomialPowerOfTwo(values, signs);
    }

    /// <summary>Restores one retained parameter and rounds its paired physical witnesses once.</summary>
    [MethodImpl(MethodImplOptions.NoInlining)]
    internal static void GetMaterials(in CurvedPrismContactGeometry geometry, in Selection selection,
        Vector3d center, FixedQuaternion rotation, Vector3d prismOrigin,
        out Vector3d normal, out FixedPointAnchor shapeAnchor, out FixedPointAnchor prismAnchor)
    {
        Span<ulong> data = stackalloc ulong[Slots * Words];
        Span<sbyte> signs = stackalloc sbyte[Slots];
        BuildParameter(geometry, selection.Edge, selection.First, selection.Second, selection.CapSign, data, signs);
        Span<ulong> cell = stackalloc ulong[WideFiniteAxisIntersection.GetFiniteValueRootCellWords(
            data[..(5 * Words)], signs[..5])];
        bool found = WideFiniteAxisIntersection.TryGetFiniteValueRoot(data[..(5 * Words)], signs[..5],
            selection.ParameterOrdinal, cell, out FiniteAxisValueRoot root);
        System.Diagnostics.Debug.Assert(found);
        Span<ulong> gradient = stackalloc ulong[54];
        Span<sbyte> gradientSigns = stackalloc sbyte[6];
        CircularRimContactAlgebra.WriteGradient(selection.First.Transform(geometry.WorldBasis),
            selection.Second.Transform(geometry.WorldBasis), gradient, gradientSigns);
        normal = ConvexContactValueRoot.GetNormalizedDirection(ref root, gradient, gradientSigns, 1);

        Span<ulong> points = stackalloc ulong[36 * Words];
        Span<sbyte> pointSigns = stackalloc sbyte[36];
        points.Clear(); pointSigns.Clear();
        Polynomial storage = new(points, pointSigns, Words);
        Polynomial shape = storage.Slice(0, 9), prism = storage.Slice(9, 18);
        Polynomial shapeDenominator = storage.Slice(27, 3), prismDenominator = storage.Slice(30, 6);
        BuildPointPolynomials(geometry, selection, data, signs, shape, prism, shapeDenominator, prismDenominator);
        shapeAnchor = CreateRootAnchor(geometry, center, rotation, center, shape, shapeDenominator, ref root);
        prismAnchor = CreateRootAnchor(geometry, center, rotation, prismOrigin, prism, prismDenominator, ref root);
    }

    [MethodImpl(MethodImplOptions.NoInlining)]
    private static void BuildPointPolynomials(in CurvedPrismContactGeometry geometry, in Selection selection,
        Span<ulong> data, Span<sbyte> signs, Polynomial shape, Polynomial prism,
        Polynomial shapeDenominator, Polynomial prismDenominator)
    {
        Polynomial parameter = new(data, signs, Words);
        Polynomial k = parameter.Slice(8, 2), l = parameter.Slice(10, 3), j = parameter.Slice(13, 3);
        l.Values.CopyTo(shapeDenominator.Values); l.Signs.CopyTo(shapeDenominator.Signs);
        Span<ulong> work = stackalloc ulong[25 * Words];
        Span<sbyte> workSigns = stackalloc sbyte[24];
        work.Clear(); workSigns.Clear();
        Polynomial scratch = new(work[..(24 * Words)], workSigns, Words);
        Polynomial q = scratch.Slice(0, 1), radiusSquared = scratch.Slice(1, 1), metric = scratch.Slice(2, 3);
        Polynomial qk = scratch.Slice(5, 2), qkm = scratch.Slice(7, 4), n = scratch.Slice(11, 2);
        Polynomial axial = scratch.Slice(13, 2), radial = scratch.Slice(15, 2), scalar = scratch.Slice(17, 1);
        Polynomial term = scratch.Slice(18, 6);
        Span<ulong> product = work.Slice(24 * Words, Words);
        Span<ulong> separation = stackalloc ulong[6 * Words];
        Span<sbyte> separationSigns = stackalloc sbyte[6];
        Polynomial separationPolynomial = new(separation, separationSigns, Words);
        Write(q, 0, Signed832.ExtendValue(geometry.SquaredAxis));
        Write(radiusSquared, 0, Signed832.ExtendValue(WideArithmetic.MultiplySigned320(geometry.Radius, geometry.Radius)));
        Write(metric, 0, selection.First.SquaredLength);
        Write(metric, 1, Twice(WideDirection3d.Dot(selection.First, selection.Second)));
        Write(metric, 2, selection.Second.SquaredLength);
        Write(axial, 0, WideDirection3d.Dot(selection.First, geometry.Axis));
        Write(axial, 1, WideDirection3d.Dot(selection.Second, geometry.Axis));
        Multiply(qk, q, k, product); Multiply(qkm, qk, metric, product);
        Multiply(prismDenominator, l, qkm, product);
        WideDirection3d half = selection.CapSign < 0 ? -geometry.Half : geometry.Half;
        for (int coordinate = 0; coordinate < 3; coordinate++)
        {
            Write(n, 0, Signed832.ExtendValue(Component(selection.First, coordinate)));
            Write(n, 1, Signed832.ExtendValue(Component(selection.Second, coordinate)));
            Multiply(radial, q, n, product);
            Write(scalar, 0, Signed832.ExtendValue(Component(geometry.Axis, coordinate)));
            Multiply(term.Slice(0, 2), axial, scalar, product); Add(radial, term.Slice(0, 2), -1);
            // sqrt(Q)=-L/K, so the exact rim point is
            // capH-R²*K*(q*n-(u.n)*u)/L, with no new radical.
            Multiply(term.Slice(0, 3), k, radial, product);
            Multiply(shape.Slice(3 * coordinate, 3), term.Slice(0, 3), radiusSquared, product);
            for (int coefficient = 0; coefficient < 3; coefficient++)
                shape.Signs[3 * coordinate + coefficient] *= -1;
            Write(scalar, 0, Signed832.ExtendValue(Component(half, coordinate)));
            Multiply(term.Slice(0, 3), scalar, l, product);
            Add(shape.Slice(3 * coordinate, 3), term.Slice(0, 3), 1);
            // Subtract the exact normal gap J*n/(q*K*M) from that point.
            Multiply(prism.Slice(6 * coordinate, 6), shape.Slice(3 * coordinate, 3), qkm, product);
            Multiply(term.Slice(0, 4), j, n, product);
            Multiply(separationPolynomial, term.Slice(0, 4), l, product);
            Add(prism.Slice(6 * coordinate, 6), separationPolynomial, -1);
        }
    }

    private static FixedPointAnchor CreateRootAnchor(in CurvedPrismContactGeometry geometry,
        Vector3d center, FixedQuaternion rotation, Vector3d targetOrigin,
        scoped Polynomial point, scoped Polynomial denominator, scoped ref FiniteAxisValueRoot root)
    {
        int count = denominator.Signs.Length;
        Span<ulong> work = stackalloc ulong[(3 * count + 2) * Words];
        Span<sbyte> workSigns = stackalloc sbyte[3 * count + 2];
        work.Clear(); workSigns.Clear();
        Polynomial storage = new(work, workSigns, Words);
        Polynomial scaledDenominator = storage.Slice(0, count), finalDenominator = storage.Slice(count, count);
        Polynomial numerator = storage.Slice(2 * count, count), scalar = storage.Slice(3 * count, 1);
        Span<ulong> product = work.Slice((3 * count + 1) * Words, Words);
        Write(scalar, 0, Signed832.ExtendValue(geometry.RawScale));
        Multiply(scaledDenominator, denominator, scalar, product);
        Write(scalar, 0, Signed832.ExtendValue(geometry.WorldBasis.Denominator));
        Multiply(finalDenominator, scaledDenominator, scalar, product);
        int denominatorSign = Sign(ref root, finalDenominator);
        System.Diagnostics.Debug.Assert(denominatorSign != 0);
        WideOrientedBox.GetRelativeLocalPointNumerators(center, targetOrigin, geometry.WorldBasis,
            out Signed192 dx, out Signed192 dy, out Signed192 dz);
        Vector3d first = default, second = default;
        for (int coordinate = 0; coordinate < 3; coordinate++)
        {
            // Convert P/(S*den) into the target origin's exact primitive-local
            // frame: (D*P+delta*den*S)/(D*den*S). Coordinate queries, including
            // full raw-domain midpoint products, remain below 2520 bits.
            Write(scalar, 0, Signed832.ExtendValue(geometry.WorldBasis.Denominator));
            Multiply(numerator, point.Slice(coordinate * count, count), scalar, product);
            Write(scalar, 0, Signed832.ExtendValue(coordinate == 0 ? dx : coordinate == 1 ? dy : dz));
            AddProduct(numerator, scaledDenominator, scalar, 1, product);
            long leading = CompareRatio(numerator, finalDenominator, Signed192.Signed(long.MaxValue), denominatorSign, ref root) > 0
                ? long.MaxValue
                : CompareRatio(numerator, finalDenominator, Signed192.Signed(long.MinValue), denominatorSign, ref root) < 0
                    ? long.MinValue : 0;
            if (leading != 0)
            {
                Write(scalar, 0, Signed832.ExtendValue(Signed192.Signed(leading)));
                AddProduct(numerator, finalDenominator, scalar, -1, product);
            }
            Fixed64 rounded = RoundRatio(numerator, finalDenominator, denominatorSign, leading, ref root);
            first[coordinate] = leading == 0 ? rounded : Fixed64.FromRaw(leading);
            second[coordinate] = leading == 0 ? Fixed64.Zero : rounded;
        }
        // Primitive support coordinates are below 3/2 Max; a prism local
        // coordinate is below sqrt(3) Max. Two additive terms preserve both.
        return new FixedPointAnchor(targetOrigin, rotation, first, second);
    }

    private static Fixed64 RoundRatio(scoped Polynomial numerator, scoped Polynomial denominator, int denominatorSign,
        long leading, scoped ref FiniteAxisValueRoot root)
    {
        long low = long.MinValue, high = long.MaxValue;
        while (low < high)
        {
            ulong span = unchecked((ulong)high - (ulong)low);
            long middle = unchecked((long)((ulong)low + (span >> 1) + (span & 1)));
            if (CompareRatio(numerator, denominator, Signed192.Signed(middle), denominatorSign, ref root) >= 0) low = middle;
            else high = middle - 1;
        }
        Signed192 half = WideArithmetic.AddSigned192(
            WideArithmetic.AddSigned192(Signed192.Signed(low), Signed192.Signed(low)), Signed192.Signed(1));
        int comparison = CompareRatio(numerator, denominator, half, denominatorSign, ref root, twiceNumerator: true);
        // Ties belong to the complete coordinate, including an odd split term.
        return Fixed64.FromRaw(low + (comparison > 0 || comparison == 0 && ((low ^ leading) & 1) != 0 ? 1 : 0));
    }

    private static int CompareRatio(scoped Polynomial numerator, scoped Polynomial denominator, Signed192 threshold,
        int denominatorSign, scoped ref FiniteAxisValueRoot root, bool twiceNumerator = false)
    {
        int count = denominator.Signs.Length;
        Span<ulong> data = stackalloc ulong[(count + 2) * Words];
        Span<sbyte> signs = stackalloc sbyte[count + 1];
        data.Clear(); signs.Clear();
        Polynomial storage = new(data[..((count + 1) * Words)], signs, Words);
        Polynomial query = storage.Slice(0, count), scalar = storage.Slice(count, 1);
        Span<ulong> product = data.Slice((count + 1) * Words, Words);
        Write(scalar, 0, Signed832.ExtendValue(threshold));
        Multiply(query, denominator, scalar, product);
        for (int index = 0; index < count; index++) query.Signs[index] *= -1;
        Add(query, numerator, twiceNumerator ? 2 : 1);
        return Sign(ref root, query) * denominatorSign;
    }

    private static void AddProduct(Polynomial target, Polynomial first, Polynomial second, int multiplier,
        Span<ulong> product)
    {
        Span<ulong> values = stackalloc ulong[target.Values.Length];
        Span<sbyte> signs = stackalloc sbyte[target.Signs.Length];
        Polynomial term = new(values, signs, target.Words);
        Multiply(term, first, second, product); Add(target, term, multiplier);
    }

    private static FiniteAxisValueRoot RestoreValue(ReadOnlySpan<ulong> values,
        ReadOnlySpan<sbyte> signs, Span<ulong> cell, in Selection selection) => new()
    {
        Coefficients = values, Signs = signs, LowerNumerator = cell,
        DenominatorShift = selection.ValueShift, Ordinal = selection.ValueOrdinal,
        IsRational = selection.IsRational
    };

    private static WideDirection3d GetOffset(in CurvedPrismContactGeometry geometry, int edge, int cap)
    {
        geometry.Edge(edge, out int start, out _);
        return CurvedPrismContact.Difference(cap < 0 ? -geometry.Half : geometry.Half, geometry.Vertex(start));
    }
    private static Signed832 Twice(Signed832 value) => WideArithmetic.AddSigned832(value, value);
    private static Signed576 Component(in WideDirection3d value, int coordinate) =>
        coordinate == 0 ? value.X : coordinate == 1 ? value.Y : value.Z;
    private static Signed320 Component(in WideAxis3 value, int coordinate) =>
        coordinate == 0 ? value.X : coordinate == 1 ? value.Y : value.Z;
    private static int Sign(scoped ref FiniteAxisValueRoot root, scoped Polynomial polynomial) =>
        WideFiniteAxisIntersection.GetSignAtFiniteValueRootAndRefine(ref root, polynomial.Values, polynomial.Signs);
    private static void Write(Polynomial polynomial, int index, Signed832 value)
    {
        Span<ulong> target = polynomial.Values.Slice(index * polynomial.Words, polynomial.Words);
        target.Clear(); WideArithmetic.GetMagnitude(value, target[..13]); polynomial.Signs[index] = (sbyte)value.Sign;
    }
    private static void Multiply(Polynomial result, Polynomial first, Polynomial second, Span<ulong> product) =>
        WideFiniteAxisIntersection.MultiplyFiniteAxisPolynomials(first.Values, first.Signs,
            second.Values, second.Signs, result.Values, result.Signs, product);
    private static void Add(Polynomial result, Polynomial source, int multiplier) =>
        WideFiniteAxisIntersection.AddFiniteAxisPolynomial(source.Values, source.Signs,
            result.Values, result.Signs, multiplier);
    private readonly ref struct Polynomial
    {
        internal readonly Span<ulong> Values;
        internal readonly Span<sbyte> Signs;
        internal readonly int Words;
        internal Polynomial(Span<ulong> values, Span<sbyte> signs, int words)
        { Values = values; Signs = signs; Words = words; }
        internal Polynomial Slice(int offset, int count) =>
            new(Values.Slice(offset * Words, count * Words), Signs.Slice(offset, count), Words);
    }
}
