//=======================================================================
// MIT License, Copyright (c) 2024-present David Oravsky (mrdav30)
// See LICENSE file in the project root for full license information.
//=======================================================================
using System;
using System.Runtime.CompilerServices;

namespace FixedMathSharp.Geometry;

/// <summary>
/// Exact box-edge/cylinder-rim support minima. Signed box-coordinate charts
/// preserve each edge's normal cone instead of reflecting a projected minimum
/// into a different box feature.
/// </summary>
internal static class BoxCylinderEdgeContacts
{
    private const int Words = 40;
    private const int ValueWords = 56;
    private const int ParameterSlots = 28;
    private const int ValueCellWords = (16 * (ValueWords * 64 + 11) + 255) / 64;

    private struct Selection
    {
        internal bool HasValue, IsZero, IsRational;
        internal int Edge, First, Second, FirstSign, SecondSign, CapSign, ParameterOrdinal;
        internal int ValueOrdinal, ValueShift;
    }

    /// <summary>
    /// AnalyticBest already has a nonnegative exact gap. Returns false on a
    /// certified separating root. Exact ties retain the earlier analytic or
    /// edge candidate. Principal and zero-radial directions belong to the caller.
    /// </summary>
    internal static bool TryGetContact(in BoxCylinderGeometry geometry,
        in ConvexContactCandidate analyticBest, out bool hasBetter,
        out Vector3d normal, out Fixed64 depth, out bool depthIsClamped,
        out Vector3d localSupportSigns)
    {
        hasBetter = false;
        normal = localSupportSigns = default;
        depth = default;
        depthIsClamped = false;
        Span<ulong> bestValues = stackalloc ulong[5 * ValueWords];
        Span<sbyte> bestSigns = stackalloc sbyte[5];
        Span<ulong> bestCell = stackalloc ulong[ValueCellWords];
        Selection best = default;
        for (int edge = 0; edge < 3; edge++)
        {
            int first = (edge + 1) % 3, second = (edge + 2) % 3;
            // A cylinder parallel to this edge projects to a circle. Its
            // principal radial direction is already an analytic candidate.
            if (Component(geometry.Axis, first).IsZero && Component(geometry.Axis, second).IsZero)
                continue;
            // A perpendicular cylinder projects to a rectangle, not an
            // ellipse. Together with the projected box its support is linear
            // between box-face, cap-pole and side-cross normals. A positive
            // minimum lies at those boundaries; a negative interior value
            // implies a negative boundary value. The analytic owner tests all.
            if (Component(geometry.Axis, edge).IsZero)
                continue;
            for (int firstSign = -1; firstSign <= 1; firstSign += 2)
                for (int secondSign = -1; secondSign <= 1; secondSign += 2)
                    for (int cap = -1; cap <= 1; cap += 2)
                    {
                        if (BoxCylinderAnalyticFeatures.IsEdgeConeDominated(geometry,
                                edge, first, second, firstSign, secondSign, cap, analyticBest))
                            continue;
                        for (int chart = 0; chart < 2; chart++)
                        {
                            int u = chart == 0 ? first : second;
                            int v = chart == 0 ? second : first;
                            int su = chart == 0 ? firstSign : secondSign;
                            int sv = chart == 0 ? secondSign : firstSign;
                            if (!TryChart(geometry, edge, u, v, su, sv, cap, analyticBest,
                                    bestValues, bestSigns, bestCell, ref best))
                                return false;
                        }
                    }
        }
        if (!best.HasValue)
            return true;
        hasBetter = true;
        normal = GetNormal(geometry, best);
        localSupportSigns = new Vector3d(
            SupportSign(best, 0), SupportSign(best, 1), SupportSign(best, 2));
        if (!best.IsZero)
        {
            FiniteAxisValueRoot root = RestoreValueRoot(bestValues, bestSigns, bestCell, best);
            ConvexContactValueRoot.GetRoundedDepth(geometry.RawScale, geometry.ValueShift,
                ref root, out depth, out depthIsClamped);
        }
        return true;
    }

    [MethodImpl(MethodImplOptions.NoInlining)]
    private static bool TryChart(in BoxCylinderGeometry geometry, int edge, int first, int second,
        int firstSign, int secondSign, int capSign, scoped ConvexContactCandidate analytic,
        scoped Span<ulong> bestValues, scoped Span<sbyte> bestSigns, scoped Span<ulong> bestCell,
        ref Selection best)
    {
        Signed320 au = Signed(Component(geometry.Axis, first), firstSign);
        Signed320 av = Signed(Component(geometry.Axis, second), secondSign);
        if (au.Sign * capSign < 0
            && WideArithmetic.AddSigned320(au, av).Sign * capSign <= 0)
            return true;

        Span<ulong> data = stackalloc ulong[ParameterSlots * Words];
        Span<sbyte> signs = stackalloc sbyte[ParameterSlots];
        BuildParameter(geometry, edge, first, second, firstSign, secondSign, capSign, data, signs);
        Polynomial storage = new(data, signs, Words);
        Polynomial stationary = storage.Slice(0, 5);
        Polynomial k = storage.Slice(8, 2);
        Polynomial l = storage.Slice(10, 3), signedGap = storage.Slice(13, 3);
        Polynomial numerator = storage.Slice(16, 5), denominator = storage.Slice(21, 5);
        Polynomial cap = storage.Slice(26, 2);
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
            if (Sign(ref root, cap) < 0)
                continue;
            int kSign = Sign(ref root, k);
            // K=0 has the rational direction (p,q), already enumerated by
            // the analytic owner. The nonparallel/nonperpendicular dispatch
            // above proves this projected ellipse has Q>0 on the whole chart.
            if (kSign == 0 || Sign(ref root, l) != -kSign)
                continue;
            int gapSign = Sign(ref root, signedGap) * kSign;
            if (gapSign < 0)
                return false;
            if (best.IsZero || analytic.GapSign == 0)
                continue;
            if (gapSign == 0)
            {
                best = CreateSelection(edge, first, second, firstSign, secondSign, capSign, ordinal);
                best.IsZero = true;
                continue;
            }
            if (!valuesReady)
            {
                BuildValues(geometry, edge, first, second, firstSign, secondSign, capSign, values, valueSigns);
                valuesReady = true;
            }
            scoped FiniteAxisValueRoot value = MapValue(geometry, ref root, numerator, denominator,
                values, valueSigns, valueCell);
            int comparison = best.HasValue
                ? WideFiniteAxisIntersection.CompareFiniteValueRoots(value,
                    RestoreValueRoot(bestValues, bestSigns, bestCell, best))
                : ConvexContactValueRoot.CompareRootSquared(geometry.RawScale, geometry.ValueShift, value, analytic);
            if (comparison >= 0)
                continue;
            values.CopyTo(bestValues);
            valueSigns.CopyTo(bestSigns);
            valueCell.CopyTo(bestCell);
            best = CreateSelection(edge, first, second, firstSign, secondSign, capSign, ordinal);
            best.ValueOrdinal = value.Ordinal;
            best.ValueShift = value.DenominatorShift;
            best.IsRational = value.IsRational;
        }
        return true;
    }

    private static Selection CreateSelection(int edge, int first, int second,
        int firstSign, int secondSign, int capSign, int ordinal) => new()
    {
        HasValue = true, Edge = edge, First = first, Second = second,
        FirstSign = firstSign, SecondSign = secondSign, CapSign = capSign, ParameterOrdinal = ordinal
    };

    private static FiniteAxisValueRoot RestoreValueRoot(ReadOnlySpan<ulong> values,
        ReadOnlySpan<sbyte> signs, Span<ulong> cell, Selection selection) => new()
    {
        Coefficients = values, Signs = signs, LowerNumerator = cell,
        DenominatorShift = selection.ValueShift, Ordinal = selection.ValueOrdinal,
        IsRational = selection.IsRational
    };

    /// <summary>
    /// W=L²-K²Q, with P=p+qt, Q=A+2Bt+Ct², K=q-pt and
    /// L=B+(C-A)t-Bt². At an admitted K!=0 root, sqrt(Q)=-L/K,
    /// so the signed gap is (PK-L)/(D K sqrt(1+t²)).
    /// </summary>
    [MethodImpl(MethodImplOptions.NoInlining)]
    private static void BuildParameter(in BoxCylinderGeometry geometry, int edge, int first, int second,
        int firstSign, int secondSign, int capSign, Span<ulong> data, Span<sbyte> signs)
    {
        // |a_i|<2^170, |c_i|<2^237, R<2^235, S<2^170 imply
        // U<2^342, |p|,|q|<2^579. Q<2^1157 and W,N<2^2340;
        // the denominator coefficients are <2^2190. Forty words (2560 bits)
        // include every coefficient-sum carry, before positive normalization.
        data.Clear(); signs.Clear();
        Polynomial storage = new(data, signs, Words);
        Polynomial w = storage.Slice(0, 5), radial = storage.Slice(5, 3);
        Polynomial k = storage.Slice(8, 2), l = storage.Slice(10, 3), j = storage.Slice(13, 3);
        Polynomial numerator = storage.Slice(16, 5), denominator = storage.Slice(21, 5);
        Polynomial cap = storage.Slice(26, 2);
        Span<ulong> scratch = stackalloc ulong[24 * Words];
        Span<sbyte> scratchSigns = stackalloc sbyte[24];
        scratch.Clear(); scratchSigns.Clear();
        Polynomial temp = new(scratch, scratchSigns, Words);
        Polynomial p = temp.Slice(0, 2), u = temp.Slice(2, 1), radius = temp.Slice(3, 1);
        Polynomial factor = temp.Slice(4, 1), axis = temp.Slice(5, 2), axisSquared = temp.Slice(7, 3);
        Polynomial one = temp.Slice(10, 3), temporary = temp.Slice(13, 5), kSquared = temp.Slice(18, 3);
        Polynomial scalar = temp.Slice(21, 1), scale = temp.Slice(22, 1);
        Span<ulong> product = scratch.Slice(23 * Words, Words);
        Signed320 au = Signed(Component(geometry.Axis, first), firstSign);
        Signed320 av = Signed(Component(geometry.Axis, second), secondSign);
        WideAxis3 offset = GetOffset(geometry, edge, first, second, firstSign, secondSign, capSign);
        Write(u, 0, geometry.Axis.SquaredLength);
        Write(radius, 0, WideArithmetic.MultiplySigned320(geometry.Radius, geometry.Radius));
        Multiply(factor, radius, u, product);
        Write(axis, 0, Signed576.ExtendValue(au));
        Write(axis, 1, Signed576.ExtendValue(av));
        Add(cap, axis, capSign);
        one.Values[0] = one.Values[2 * Words] = 1;
        one.Signs[0] = one.Signs[2] = 1;
        Multiply(radial, u, one, product);
        Multiply(axisSquared, axis, axis, product);
        Add(radial, axisSquared, -1);
        Multiply(temporary.Slice(0, 3), radial, factor, product);
        Copy(radial, temporary.Slice(0, 3));
        // B=-R² U au av. Build it directly rather than divide Q's middle
        // coefficient, keeping the construction entirely in shared arithmetic.
        Write(scalar, 0, WideArithmetic.MultiplySigned320(au, av));
        Multiply(temporary.Slice(0, 1), factor, scalar, product);
        Add(l.Slice(0, 1), temporary.Slice(0, 1), -1);
        Add(l.Slice(2, 1), temporary.Slice(0, 1), 1);
        Add(l.Slice(1, 1), radial.Slice(2, 1), 1);
        Add(l.Slice(1, 1), radial.Slice(0, 1), -1);
        Write(axis, 0, Signed576.ExtendValue(Signed(Component(offset, first), firstSign)));
        Write(axis, 1, Signed576.ExtendValue(Signed(Component(offset, second), secondSign)));
        Multiply(p, u, axis, product);
        Add(k.Slice(0, 1), p.Slice(1, 1), 1);
        Add(k.Slice(1, 1), p.Slice(0, 1), -1);
        Multiply(w, l, l, product);
        Multiply(kSquared, k, k, product);
        Multiply(temporary, kSquared, radial, product);
        Add(w, temporary, -1);
        Multiply(j, p, k, product);
        Add(j, l, -1);
        Multiply(numerator, j, j, product);
        Write(scale, 0, Signed576.ExtendValue(Signed320.ExtendValue(geometry.RawScale)));
        Multiply(scalar, u, scale, product);
        Multiply(temporary.Slice(0, 1), scalar, scalar, product);
        Copy(scalar, temporary.Slice(0, 1));
        Multiply(temporary, kSquared, one, product);
        Multiply(denominator, temporary, scalar, product);
        WideFiniteAxisIntersection.NormalizeFiniteAxisPolynomialPowerOfTwo(w.Values, w.Signs);
        // N and D share one scale. The other polynomials are sign queries only.
        Polynomial ratio = storage.Slice(16, 10);
        WideFiniteAxisIntersection.NormalizeFiniteAxisPolynomialPowerOfTwo(ratio.Values, ratio.Signs);
    }

    [MethodImpl(MethodImplOptions.NoInlining)]
    private static void BuildValues(in BoxCylinderGeometry geometry, int edge, int first, int second,
        int firstSign, int secondSign, int capSign, Span<ulong> values, Span<sbyte> signs)
    {
        WideAxis3 c = GetOffset(geometry, edge, first, second, firstSign, secondSign, capSign);
        Span<ulong> invariants = stackalloc ulong[8 * ValueWords];
        Span<sbyte> invariantSigns = stackalloc sbyte[8];
        invariants.Clear(); invariantSigns.Clear();
        Polynomial scalars = new(invariants, invariantSigns, ValueWords);
        Write(scalars, 0, geometry.Axis.SquaredLength);
        Write(scalars, 1, Signed576.ExtendValue(Signed320.ExtendValue(Signed192.Signed(1))));
        Write(scalars, 2, Signed576.ExtendValue(Component(geometry.Axis, edge)));
        Write(scalars, 3, c.SquaredLength);
        Write(scalars, 4, WideAxis3.Dot(geometry.Axis, c));
        // c's edge coordinate is zero: y=b.c=0, and tau=0.
        Write(scalars, 6, WideArithmetic.MultiplySigned320(geometry.Radius, geometry.Radius));
        CylinderPairSideValuePolynomial.BuildUnshifted(invariants, invariantSigns, ValueWords, values, signs);
        // Weighted height after z=2^ValueShift*t is <3310 bits, including
        // ValueShift<=480 and |c_i|<2^237. Fifty-six words provide 3584 bits.
        WideFiniteAxisIntersection.ScaleFiniteAxisPolynomialVariable(values, 5, geometry.ValueShift);
        WideFiniteAxisIntersection.NormalizeFiniteAxisPolynomialPowerOfTwo(values, signs);
    }

    [MethodImpl(MethodImplOptions.NoInlining)]
    private static FiniteAxisValueRoot MapValue(in BoxCylinderGeometry geometry,
        scoped ref FiniteAxisValueRoot parameter, scoped Polynomial numerator, scoped Polynomial denominator,
        ReadOnlySpan<ulong> values, ReadOnlySpan<sbyte> signs, Span<ulong> cell)
    {
        Span<ulong> batchCells = stackalloc ulong[64];
        Span<int> batchShifts = stackalloc int[8];
        FiniteAxisValueRoots roots = WideFiniteAxisIntersection.GetFiniteValueRoots(values, signs, batchCells, batchShifts);
        Span<ulong> upper = stackalloc ulong[cell.Length];
        for (int ordinal = 0; ordinal < roots.Count - 1; ordinal++)
        {
            FiniteAxisValueRoot root = FiniteAxisValueRoots.GetRoot(roots, ordinal, values, signs, cell);
            if (root.IsRational)
            {
                if (CompareValueEndpoint(geometry, ref parameter, numerator, denominator,
                        cell, root.DenominatorShift) == 0)
                    return root;
            }
            else
            {
                cell.CopyTo(upper);
                WideArithmetic.AddWord(upper, 0, 1);
                // Membership is known from the stationary equation. Earlier
                // value roots were excluded, so only this upper bound matters.
                if (CompareValueEndpoint(geometry, ref parameter, numerator, denominator,
                        upper, root.DenominatorShift) < 0)
                    return root;
            }
        }
        System.Diagnostics.Debug.Assert(roots.Count > 0);
        return FiniteAxisValueRoots.GetRoot(roots, roots.Count - 1, values, signs, cell);
    }

    [MethodImpl(MethodImplOptions.NoInlining)]
    private static int CompareValueEndpoint(in BoxCylinderGeometry geometry,
        scoped ref FiniteAxisValueRoot parameter, scoped Polynomial numerator, scoped Polynomial denominator,
        scoped ReadOnlySpan<ulong> endpoint, int shift)
    {
        int words = (shift + geometry.ValueShift + Words * 64 + 384 + 127) / 64;
        Span<ulong> query = stackalloc ulong[5 * words];
        Span<sbyte> signs = stackalloc sbyte[5];
        Span<ulong> product = stackalloc ulong[words];
        Span<ulong> scale = stackalloc ulong[3];
        Span<ulong> scaleSquared = stackalloc ulong[6];
        query.Clear(); signs.Clear();
        WideArithmetic.GetMagnitude(geometry.RawScale, out scale[2], out scale[1], out scale[0]);
        WideArithmetic.MultiplyMagnitudes(scale, scale, scaleSquared);
        for (int index = 0; index < 5; index++)
        {
            Span<ulong> target = query.Slice(index * words, words);
            int sign = 0;
            WideArithmetic.MultiplyMagnitudes(numerator.Values.Slice(index * Words, Words), scaleSquared, product);
            WideArithmetic.AddShiftedSignedMagnitude(product, numerator.Signs[index], shift, target, ref sign);
            WideArithmetic.MultiplyMagnitudes(denominator.Values.Slice(index * Words, Words), endpoint, product);
            WideArithmetic.AddShiftedSignedMagnitude(product, -denominator.Signs[index], geometry.ValueShift, target, ref sign);
            signs[index] = (sbyte)sign;
        }
        return WideFiniteAxisIntersection.GetSignAtFiniteValueRootAndRefine(ref parameter, query, signs);
    }

    [MethodImpl(MethodImplOptions.NoInlining)]
    private static Vector3d GetNormal(in BoxCylinderGeometry geometry, Selection selection)
    {
        Span<ulong> data = stackalloc ulong[ParameterSlots * Words];
        Span<sbyte> signs = stackalloc sbyte[ParameterSlots];
        BuildParameter(geometry, selection.Edge, selection.First, selection.Second,
            selection.FirstSign, selection.SecondSign, selection.CapSign, data, signs);
        Span<ulong> cell = stackalloc ulong[WideFiniteAxisIntersection.GetFiniteValueRootCellWords(
            data[..(5 * Words)], signs[..5])];
        bool found = WideFiniteAxisIntersection.TryGetFiniteValueRoot(data[..(5 * Words)], signs[..5],
            selection.ParameterOrdinal, cell, out FiniteAxisValueRoot root);
        System.Diagnostics.Debug.Assert(found);
        const int directionWords = 3;
        Span<ulong> direction = stackalloc ulong[6 * directionWords];
        Span<sbyte> directionSigns = stackalloc sbyte[6];
        direction.Clear(); directionSigns.Clear();
        WideAxis3 first = geometry.WorldBasis.GetAxis(selection.First);
        WideAxis3 second = geometry.WorldBasis.GetAxis(selection.Second);
        for (int component = 0; component < 3; component++)
        {
            WriteDirection(Component(first, component), selection.FirstSign,
                direction.Slice(component * 2 * directionWords, directionWords), out directionSigns[2 * component]);
            WriteDirection(Component(second, component), selection.SecondSign,
                direction.Slice((component * 2 + 1) * directionWords, directionWords), out directionSigns[2 * component + 1]);
        }
        return ConvexContactValueRoot.GetNormalizedDirection(ref root, direction, directionSigns, 1);
    }

    private static void WriteDirection(Signed320 value, int orientation, Span<ulong> magnitude, out sbyte sign)
    {
        Signed192 narrow = Signed192.NarrowProven(value);
        WideArithmetic.GetMagnitude(narrow, out magnitude[2], out magnitude[1], out magnitude[0]);
        sign = (sbyte)(value.Sign * orientation);
    }

    private static WideAxis3 GetOffset(in BoxCylinderGeometry geometry, int edge, int first, int second,
        int firstSign, int secondSign, int capSign)
    {
        int corner = (firstSign > 0 ? 1 << first : 0) | (secondSign > 0 ? 1 << second : 0);
        WideAxis3 offset = geometry.GetCapVertexOffset(corner, capSign);
        // Projection onto the edge-normal plane discards this coordinate.
        return new WideAxis3(edge == 0 ? default : offset.X,
            edge == 1 ? default : offset.Y, edge == 2 ? default : offset.Z);
    }

    private static Fixed64 SupportSign(Selection selection, int axis) => axis == selection.Edge
        ? Fixed64.Zero : (axis == selection.First ? selection.FirstSign : selection.SecondSign) > 0
            ? Fixed64.One : -Fixed64.One;
    private static Signed320 Component(in WideAxis3 value, int axis) => axis == 0 ? value.X : axis == 1 ? value.Y : value.Z;
    private static Signed320 Signed(Signed320 value, int sign) => sign < 0 ? WideArithmetic.Negate(value) : value;
    private static int Sign(scoped ref FiniteAxisValueRoot root, scoped Polynomial polynomial) =>
        WideFiniteAxisIntersection.GetSignAtFiniteValueRootAndRefine(ref root, polynomial.Values, polynomial.Signs);
    private static void Write(Polynomial polynomial, int index, Signed576 value)
    {
        Span<ulong> target = polynomial.Values.Slice(index * polynomial.Words, polynomial.Words);
        target.Clear();
        WideArithmetic.GetMagnitude(value, target[..9]);
        polynomial.Signs[index] = (sbyte)value.Sign;
    }
    private static void Multiply(Polynomial result, Polynomial first, Polynomial second, Span<ulong> product) =>
        WideFiniteAxisIntersection.MultiplyFiniteAxisPolynomials(first.Values, first.Signs,
            second.Values, second.Signs, result.Values, result.Signs, product);
    private static void Add(Polynomial result, Polynomial source, int multiplier) =>
        WideFiniteAxisIntersection.AddFiniteAxisPolynomial(source.Values, source.Signs,
            result.Values, result.Signs, multiplier);
    private static void Copy(Polynomial result, Polynomial source)
    {
        source.Values.CopyTo(result.Values);
        source.Signs.CopyTo(result.Signs);
    }

    private readonly ref struct Polynomial
    {
        internal readonly Span<ulong> Values;
        internal readonly Span<sbyte> Signs;
        internal readonly int Words;
        internal Polynomial(Span<ulong> values, Span<sbyte> signs, int words)
        {
            Values = values; Signs = signs; Words = words;
        }
        internal Polynomial Slice(int offset, int count) =>
            new(Values.Slice(offset * Words, count * Words), Signs.Slice(offset, count), Words);
    }
}
