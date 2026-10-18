//=======================================================================
// MIT License, Copyright (c) 2024-present David Oravsky (mrdav30)
// See LICENSE file in the project root for full license information.
//=======================================================================
using System;
using static FixedMathSharp.Geometry.CylinderContactAlgebra;

namespace FixedMathSharp.Geometry;

/// <content>Finite planar strata and exact oblique circular-rim root selection.</content>
internal static partial class SegmentSlabSurfaceContact
{
    private static void AccumulatePrism(scoped Input input, scoped ref Selection selection)
    {
        int winding = FiniteSlabSectionFrame.GetPrismWinding(input.Offsets);
        for (int cap = -1; cap <= 1; cap += 2)
            AccumulatePlane(input, new(default, cap < 0 ? WideArithmetic.Negate(input.Cap) : input.Cap, default),
                Axis(0, cap, 0), false, ref selection);
        Span<WideAxis3> cone = stackalloc WideAxis3[5];
        for (int index = 0; index < input.Offsets.Length; index++)
        {
            WideAxis3 a = PrismVertex(input, index), b = PrismVertex(input, (index + 1) % input.Offsets.Length);
            WideAxis3 edge = TriangleCircularGeometry.Subtract(b, a);
            if (edge.IsZero) continue;
            int previous = (index + input.Offsets.Length - 1) % input.Offsets.Length;
            WideAxis3 inward = TriangleCircularGeometry.Subtract(PrismVertex(input, previous), a);
            while (inward.IsZero)
            {
                previous = (previous + input.Offsets.Length - 1) % input.Offsets.Length;
                inward = TriangleCircularGeometry.Subtract(PrismVertex(input, previous), a);
            }
            WideAxis3 outward = new(winding > 0 ? edge.Z : WideArithmetic.Negate(edge.Z), default,
                winding > 0 ? WideArithmetic.Negate(edge.X) : edge.X);
            bool straight = WideArithmetic.SubtractSigned576(WideArithmetic.MultiplySigned320(inward.X, edge.Z),
                WideArithmetic.MultiplySigned320(inward.Z, edge.X)).IsZero;
            AccumulatePlane(input, a, outward, false, ref selection);
            cone[0] = inward; cone[1] = edge;
            cone[2] = Axis(0, 1, 0); cone[3] = Axis(0, -1, 0);
            // Opposite incident rays at a redundant straight vertex also need the wall's outward orientation.
            if (straight) cone[4] = -outward;
            WideAxis3 bottom = new(a.X, WideArithmetic.Negate(input.Cap), a.Z);
            WideAxis3 top = new(a.X, input.Cap, a.Z);
            AccumulateLine(input, bottom, top, cone[..(straight ? 5 : 4)], ref selection);
            for (int cap = -1; cap <= 1; cap += 2)
            {
                WideAxis3 first = new(a.X, cap < 0 ? WideArithmetic.Negate(input.Cap) : input.Cap, a.Z);
                WideAxis3 second = new(b.X, first.Y, b.Z);
                cone[0] = -outward; cone[1] = Axis(0, -cap, 0); cone[2] = edge; cone[3] = -edge;
                AccumulateLine(input, first, second, cone[..4], ref selection);
                cone[0] = inward; cone[1] = edge; cone[2] = Axis(0, -cap, 0);
                if (straight) cone[3] = -outward;
                AccumulateVertex(input, first, cone[..(straight ? 4 : 3)], ref selection);
            }
        }
    }

    private static WideAxis3 PrismVertex(scoped Input input, int index)
    {
        Vector2d vertex = input.Offsets[index];
        Signed192 scale = Signed192.NarrowProven(input.Scale);
        return new(WideArithmetic.MultiplySigned192(Signed192.Raw(vertex.X), scale), default,
            WideArithmetic.MultiplySigned192(Signed192.Raw(vertex.Y), scale));
    }

    private static WideAxis3 Axis(long x, long y, long z) => new(
        Signed320.ExtendValue(Signed192.Signed(x)), Signed320.ExtendValue(Signed192.Signed(y)),
        Signed320.ExtendValue(Signed192.Signed(z)));

    private static void AccumulateStraight(scoped Input input, scoped ref Selection selection)
    {
        Span<WideAxis3> cone = stackalloc WideAxis3[4];
        for (int cap = -1; cap <= 1; cap += 2)
            AccumulatePlane(input, new(default, cap < 0 ? WideArithmetic.Negate(input.Cap) : input.Cap, default),
                Axis(0, cap, 0), true, ref selection);
        for (int side = -1; side <= 1; side += 2)
        {
            Signed320 x = side < 0 ? WideArithmetic.Negate(input.Radial) : input.Radial;
            AccumulatePlane(input, new(x, default, default), Axis(side, 0, 0), true, ref selection);
            for (int cap = -1; cap <= 1; cap += 2)
            {
                Signed320 y = cap < 0 ? WideArithmetic.Negate(input.Cap) : input.Cap;
                cone[0] = Axis(-side, 0, 0); cone[1] = Axis(0, -cap, 0);
                cone[2] = Axis(0, 0, 1); cone[3] = Axis(0, 0, -1);
                AccumulateLine(input, new(x, y, WideArithmetic.Negate(input.Core)),
                    new(x, y, input.Core), cone, ref selection);
            }
        }
    }

    private static bool Seed(scoped Input input, scoped Candidate candidate, int seed)
    {
        candidate.Clear();
        if (!input.Clip.IsEmpty && seed < 3)
        {
            ref readonly RationalSegmentInterval clip = ref input.Clip[0];
            if (seed < 2)
            {
                candidate.Parameter.Set(Extend(seed == 0 ? clip.LowerNumerator : clip.UpperNumerator));
                candidate.ParameterDenominator.Set(Extend(seed == 0 ? clip.LowerDenominator : clip.UpperDenominator));
            }
            else
            {
                // The mediant lies strictly inside distinct positive-denominator
                // bounds without the 640-bit denominator of their arithmetic mean.
                candidate.Parameter.Set(WideArithmetic.AddSigned576(Extend(clip.LowerNumerator), Extend(clip.UpperNumerator)));
                candidate.ParameterDenominator.Set(WideArithmetic.AddSigned576(Extend(clip.LowerDenominator), Extend(clip.UpperDenominator)));
            }
        }
        else if (seed == 1) candidate.Parameter.Set(One);
        else if (seed == 2)
        { candidate.Parameter.Set(One); candidate.ParameterDenominator.Set(Extend(Signed320.ExtendValue(Signed192.Signed(2)))); }
        else if (seed >= 3)
        {
            FixedSegment segment = seed == 3 ? input.Segment : new(input.Segment.End, input.Segment.Start);
            bool admitted = input.RequireSourceInside
                ? input.Frame.TryGetFirstSegmentParameter(segment, input.Offsets,
                    candidate.Parameter, candidate.ParameterDenominator, candidate.Root)
                : input.Frame.TryGetFirstProjectedSegmentParameter(segment, input.Offsets,
                    candidate.Parameter, candidate.ParameterDenominator, candidate.Root);
            if (!admitted) return false;
            if (seed == 4)
            { candidate.Parameter.MultiplySign(-1); candidate.Parameter.Add(candidate.ParameterDenominator); }
        }
        SetPoint(input, candidate); return true;
    }

    private static void AccumulatePlane(scoped Input input, WideAxis3 origin, WideAxis3 outward,
        bool central, scoped ref Selection selection)
    {
        Span<ulong> values = stackalloc ulong[StorageWords], root = stackalloc ulong[Words];
        Span<int> signs = stackalloc int[2 * Fields];
        var candidate = new Candidate(values, signs, root);
        Span<ulong> work = stackalloc ulong[8 * Words]; Span<int> workSigns = stackalloc int[8];
        ContactQuadratic offset = ContactQuadratic.At(work, workSigns, 0), term = ContactQuadratic.At(work, workSigns, 1),
            product = ContactQuadratic.At(work, workSigns, 2), bound = ContactQuadratic.At(work, workSigns, 3);
        Signed576 square = outward.SquaredLength;
        // Point seeds coincide; keep the first tie and omit segment-only stationary candidates.
        int seedCount = input.Edge.IsZero ? 1 : 6;
        for (int seed = 0; seed < seedCount; seed++)
        {
            if (seed == 5)
            {
                Signed576 velocity = WideAxis3.Dot(input.Edge, outward);
                if (velocity.IsZero) continue;
                candidate.Clear();
                candidate.Parameter.Set(WideAxis3.Dot(TriangleCircularGeometry.Subtract(origin, input.First), outward));
                candidate.ParameterDenominator.Set(velocity);
                if (velocity.Sign < 0)
                { candidate.Parameter.MultiplySign(-1); candidate.ParameterDenominator.MultiplySign(-1); }
                SetPoint(input, candidate);
            }
            else if (!Seed(input, candidate, seed)) continue;
            offset.Clear();
            for (int axis = 0; axis < 3; axis++)
            {
                ContactQuadratic.Scale(candidate.Denominator, Extend(Component(origin, axis)), term);
                term.Add(candidate.Point(axis), -1);
                ContactQuadratic.Scale(term, Extend(Component(outward, axis)), product); offset.Add(product);
            }
            if (offset.Sign(candidate.Root) < 0) continue;
            term.Set(square); ScaleCoordinates(candidate, term);
            for (int axis = 0; axis < 3; axis++)
            {
                ContactQuadratic.Scale(offset, Extend(Component(outward, axis)), term);
                candidate.Boundary(axis).Add(term);
                candidate.Normal(axis).Set(Extend(Component(outward, axis)));
            }
            if (central)
            {
                ContactQuadratic.Scale(candidate.Denominator, Extend(input.Core), bound);
                bound.CopyTo(term); term.Add(candidate.Boundary(2), -1);
                if (term.Sign(candidate.Root) < 0) continue;
                bound.CopyTo(term); term.Add(candidate.Boundary(2));
                if (term.Sign(candidate.Root) < 0) continue;
            }
            Keep(input, candidate, ref selection);
        }
    }

    private static void AccumulateLine(scoped Input input, WideAxis3 first, WideAxis3 second,
        scoped ReadOnlySpan<WideAxis3> cone, scoped ref Selection selection)
    {
        WideAxis3 axis = TriangleCircularGeometry.Subtract(second, first);
        // Positive thickness/core length and distinct prism edges keep every line finite and nonzero.
        System.Diagnostics.Debug.Assert(!axis.IsZero);
        WideAxis3 offset = TriangleCircularGeometry.Subtract(input.First, first);
        Signed576 ee = input.Edge.SquaredLength, ff = axis.SquaredLength,
            ef = WideAxis3.Dot(input.Edge, axis), ew = WideAxis3.Dot(input.Edge, offset), fw = WideAxis3.Dot(axis, offset);
        Span<ulong> work = stackalloc ulong[16 * Words]; Span<int> workSigns = stackalloc int[16];
        ContactQuadratic a = ContactQuadratic.At(work, workSigns, 0), b = ContactQuadratic.At(work, workSigns, 1),
            determinant = ContactQuadratic.At(work, workSigns, 2), term = ContactQuadratic.At(work, workSigns, 3),
            upper = ContactQuadratic.At(work, workSigns, 5),
            projection = ContactQuadratic.At(work, workSigns, 6), divisor = ContactQuadratic.At(work, workSigns, 7);
        a.Set(ee); b.Set(ff); ContactQuadratic.Multiply(a, b, ReadOnlySpan<ulong>.Empty, determinant);
        a.Set(ef); ContactQuadratic.Multiply(a, a, ReadOnlySpan<ulong>.Empty, term); determinant.Add(term, -1);
        Span<ulong> values = stackalloc ulong[StorageWords], root = stackalloc ulong[Words];
        Span<int> signs = stackalloc int[2 * Fields]; var candidate = new Candidate(values, signs, root);
        int seedCount = input.Edge.IsZero ? 1 : 8;
        for (int seed = 0; seed < seedCount; seed++)
        {
            if (seed == 5)
            {
                if (determinant.Signs[0] == 0) continue;
                candidate.Clear(); a.Set(ef); b.Set(fw); ContactQuadratic.Multiply(a, b, root, candidate.Parameter);
                a.Set(ff); b.Set(ew); ContactQuadratic.Multiply(a, b, root, term); candidate.Parameter.Add(term, -1);
                determinant.CopyTo(candidate.ParameterDenominator); SetPoint(input, candidate);
            }
            else if (seed >= 6)
            {
                if (ef.IsZero) continue;
                candidate.Clear(); candidate.Parameter.Set(WideArithmetic.SubtractSigned576(seed == 6 ? default : ff, fw));
                candidate.ParameterDenominator.Set(ef);
                if (ef.Sign < 0) { candidate.Parameter.MultiplySign(-1); candidate.ParameterDenominator.MultiplySign(-1); }
                SetPoint(input, candidate);
            }
            else if (!Seed(input, candidate, seed)) continue;
            if (seed == 5)
            {
                a.Set(ee); b.Set(fw); ContactQuadratic.Multiply(a, b, root, projection);
                a.Set(ef); b.Set(ew); ContactQuadratic.Multiply(a, b, root, term); projection.Add(term, -1);
                candidate.Denominator.CopyTo(divisor);
            }
            else
            {
                projection.Clear();
                for (int component = 0; component < 3; component++)
                {
                    ContactQuadratic.Scale(candidate.Denominator, Extend(Component(first, component)), term);
                    candidate.Point(component).CopyTo(a); a.Add(term, -1);
                    ContactQuadratic.Scale(a, Extend(Component(axis, component)), term); projection.Add(term);
                }
                ContactQuadratic.Scale(candidate.Denominator, ff, divisor);
            }
            divisor.CopyTo(upper); upper.Add(projection, -1);
            if (projection.Sign(candidate.Root) < 0 || upper.Sign(candidate.Root) < 0) continue;
            if (seed != 5) { a.Set(ff); ScaleCoordinates(candidate, a); }
            for (int component = 0; component < 3; component++)
            {
                ContactQuadratic.Scale(candidate.Denominator, Extend(Component(first, component)), candidate.Boundary(component));
                ContactQuadratic.Scale(projection, Extend(Component(axis, component)), term); candidate.Boundary(component).Add(term);
            }
            KeepPlanar(input, candidate, cone, ref selection);
        }
    }

    private static void AccumulateVertex(scoped Input input, WideAxis3 point,
        scoped ReadOnlySpan<WideAxis3> cone, scoped ref Selection selection)
    {
        Span<ulong> values = stackalloc ulong[StorageWords], root = stackalloc ulong[Words];
        Span<int> signs = stackalloc int[2 * Fields]; var candidate = new Candidate(values, signs, root);
        int seedCount = input.Edge.IsZero ? 1 : 4;
        for (int seed = 0; seed < seedCount; seed++)
        {
            if (seed == 3)
            {
                candidate.Clear(); candidate.Parameter.Set(WideAxis3.Dot(
                    TriangleCircularGeometry.Subtract(point, input.First), input.Edge));
                candidate.ParameterDenominator.Set(input.Edge.SquaredLength); SetPoint(input, candidate);
            }
            else Seed(input, candidate, seed);
            for (int axis = 0; axis < 3; axis++)
                ContactQuadratic.Scale(candidate.Denominator, Extend(Component(point, axis)), candidate.Boundary(axis));
            KeepPlanar(input, candidate, cone, ref selection);
        }
    }

    private static void KeepPlanar(scoped Input input, scoped Candidate candidate,
        scoped ReadOnlySpan<WideAxis3> cone, scoped ref Selection selection)
    {
        Span<ulong> values = stackalloc ulong[8 * Words]; Span<int> signs = stackalloc int[8];
        ContactQuadratic sum = ContactQuadratic.At(values, signs, 0), term = ContactQuadratic.At(values, signs, 1),
            a = ContactQuadratic.At(values, signs, 2), b = ContactQuadratic.At(values, signs, 3);
        bool nonzero = false;
        for (int axis = 0; axis < 3; axis++)
        {
            candidate.Boundary(axis).CopyTo(candidate.Normal(axis)); candidate.Normal(axis).Add(candidate.Point(axis), -1);
            nonzero |= candidate.Normal(axis).Sign(candidate.Root) != 0;
        }
        if (nonzero)
        {
            foreach (WideAxis3 wall in cone)
            { Dot(candidate, wall, sum, term); if (sum.Sign(candidate.Root) > 0) return; }
            Keep(input, candidate, ref selection); return;
        }
        Span<WideAxis3> walls = stackalloc WideAxis3[input.Fan.Length + cone.Length];
        input.Fan.CopyTo(walls); cone.CopyTo(walls[input.Fan.Length..]);
        // A nonempty pointed cone in three dimensions contains an axis or
        // an extreme ray at two wall planes. Degenerate lines keep both signs.
        for (int axis = 0; axis < 3; axis++)
            for (int branch = -1; branch <= 1; branch += 2)
            {
                for (int component = 0; component < 3; component++)
                    candidate.Normal(component).Set(component == axis ? Extend(Signed320.ExtendValue(Signed192.Signed(branch))) : default);
                KeepPlanarNormal(input, candidate, cone, ref selection);
            }
        for (int first = 0; first < walls.Length; first++)
            for (int second = first + 1; second < walls.Length; second++)
            {
                for (int axis = 0; axis < 3; axis++)
                {
                    int next = (axis + 1) % 3, last = (axis + 2) % 3;
                    a.Set(Extend(Component(walls[first], next))); b.Set(Extend(Component(walls[second], last)));
                    ContactQuadratic.Multiply(a, b, candidate.Root, candidate.Normal(axis));
                    a.Set(Extend(Component(walls[first], last))); b.Set(Extend(Component(walls[second], next)));
                    ContactQuadratic.Multiply(a, b, candidate.Root, term); candidate.Normal(axis).Add(term, -1);
                }
                KeepPlanarNormal(input, candidate, cone, ref selection);
                for (int axis = 0; axis < 3; axis++) candidate.Normal(axis).MultiplySign(-1);
                KeepPlanarNormal(input, candidate, cone, ref selection);
            }
    }

    private static void KeepPlanarNormal(scoped Input input, scoped Candidate candidate,
        scoped ReadOnlySpan<WideAxis3> cone, scoped ref Selection selection)
    {
        Span<ulong> values = stackalloc ulong[4 * Words]; Span<int> signs = stackalloc int[4];
        var sum = ContactQuadratic.At(values, signs, 0); var term = ContactQuadratic.At(values, signs, 1);
        foreach (WideAxis3 wall in cone)
        { Dot(candidate, wall, sum, term); if (sum.Sign(candidate.Root) > 0) return; }
        Keep(input, candidate, ref selection);
    }
    private const int ObliqueValueCellWords = (16 * (CircularRimContactAlgebra.ValueWords * 64 + 11) + 255) / 64;

    private struct ObliqueSelection
    {
        internal bool HasValue, IsZero, IsRational;
        internal WideAxis3 First, Second;
        internal int Cap, Region, ParameterOrdinal, ValueOrdinal, ValueShift;
    }

    private static bool TryImproveObliqueRims(scoped Input input, scoped Selection analytic, out FixedContactAnchors contact)
    {
        contact = default;
        if (input.Edge.Y.IsZero || input.Edge.X.IsZero && input.Edge.Z.IsZero || input.Radius == Fixed64.Zero
            || analytic.HasValue && analytic.MetricSigns[10] == 0) return false;
        Span<ulong> bestValues = stackalloc ulong[5 * CircularRimContactAlgebra.ValueWords];
        Span<sbyte> bestSigns = stackalloc sbyte[5];
        Span<ulong> bestCell = stackalloc ulong[ObliqueValueCellWords];
        ObliqueSelection best = default;
        CircularRimContactAlgebra.GetBasis(input.Edge, out WideAxis3 u, out WideAxis3 v);
        int firstRegion = input.Length == Fixed64.Zero ? 0 : -1;
        Span<Signed320> bounds = stackalloc Signed320[9]
        { input.First.X, input.First.Y, input.First.Z, input.Edge.X, input.Edge.Y, input.Edge.Z, input.Cap, input.Core, input.Radial };
        int valueShift = TriangleCircularGeometry.GetValueShift(bounds, 5);
        for (int region = firstRegion; region <= (input.Length == Fixed64.Zero ? 0 : 1); region += 2)
            for (int cap = -1; cap <= 1; cap += 2)
                for (int firstSign = -1; firstSign <= 1; firstSign += 2)
                    for (int secondSign = -1; secondSign <= 1; secondSign += 2)
                    {
                        WideAxis3 first = firstSign < 0 ? -u : u, second = secondSign < 0 ? -v : v;
                        AccumulateObliqueChart(input, analytic, first, second, region, cap, valueShift,
                            bestValues, bestSigns, bestCell, ref best);
                        AccumulateObliqueChart(input, analytic, second, first, region, cap, valueShift,
                            bestValues, bestSigns, bestCell, ref best);
                    }
        if (!best.HasValue) return false;
        contact = MaterializeOblique(input, best, valueShift, bestValues, bestSigns, bestCell);
        return true;
    }

    private static WideAxis3 RimOffset(scoped Input input, int region, int cap) => new(input.First.X,
        WideArithmetic.SubtractSigned320(input.First.Y, cap < 0 ? WideArithmetic.Negate(input.Cap) : input.Cap),
        WideArithmetic.SubtractSigned320(input.First.Z, region < 0 ? WideArithmetic.Negate(input.Core) : region > 0 ? input.Core : default));

    private static void AccumulateObliqueChart(scoped Input input, scoped Selection analytic,
        WideAxis3 first, WideAxis3 second, int region, int cap, int valueShift,
        scoped Span<ulong> bestValues, scoped Span<sbyte> bestSigns, scoped Span<ulong> bestCell, ref ObliqueSelection best)
    {
        // The cap generator and stadium hemisphere must agree with the exit
        // ray -n. Reject an entirely unpermitted chart before its quartic.
        if (first.Y.Sign * cap > 0 && WideArithmetic.AddSigned320(first.Y, second.Y).Sign * cap > 0) return;
        if (region != 0 && first.Z.Sign * region > 0
            && WideArithmetic.AddSigned320(first.Z, second.Z).Sign * region > 0) return;
        Span<ulong> data = stackalloc ulong[26 * Words]; Span<sbyte> signs = stackalloc sbyte[26];
        WideAxis3 offset = RimOffset(input, region, cap);
        CircularRimContactAlgebra.BuildParameter(offset, input.Radial, Signed192.NarrowProven(input.Scale),
            first, second, data, signs);
        Span<ulong> batchCells = stackalloc ulong[64]; Span<int> batchShifts = stackalloc int[8];
        FiniteAxisValueRoots roots = WideFiniteAxisIntersection.GetFiniteValueRoots(data[..(5 * Words)], signs[..5], batchCells, batchShifts);
        Span<ulong> cell = stackalloc ulong[WideFiniteAxisIntersection.GetFiniteValueRootCellWords(data[..(5 * Words)], signs[..5])];
        Span<ulong> values = stackalloc ulong[5 * CircularRimContactAlgebra.ValueWords]; Span<sbyte> valueSigns = stackalloc sbyte[5];
        Span<ulong> valueCell = stackalloc ulong[ObliqueValueCellWords]; bool valuesReady = false;
        for (int ordinal = 0; ordinal < roots.Count; ordinal++)
        {
            FiniteAxisValueRoot root = FiniteAxisValueRoots.GetRoot(roots, ordinal, data[..(5 * Words)], signs[..5], cell);
            int kSign = CircularRimContactAlgebra.Sign(ref root, data, signs, 8, 2);
            if (kSign == 0 || CircularRimContactAlgebra.Sign(ref root, data, signs, 10, 3) != -kSign) continue;
            int gapSign = CircularRimContactAlgebra.Sign(ref root, data, signs, 13, 3) * kSign;
            if (gapSign < 0 || !AdmitObliqueFoot(input, first, second, region, cap, ref root)) continue;
            var candidate = new ObliqueSelection
            { HasValue = true, First = first, Second = second, Region = region, Cap = cap, ParameterOrdinal = ordinal };
            if (gapSign == 0)
            { if (!best.IsZero) { candidate.IsZero = true; best = candidate; } continue; }
            if (best.IsZero) continue;
            if (!valuesReady)
            { CircularRimContactAlgebra.BuildValues(input.Edge, offset, input.Radial, valueShift, values, valueSigns); valuesReady = true; }
            FiniteAxisValueRoot value = ConvexContactValueRoot.MapSquaredValue(Signed192.NarrowProven(input.Scale), valueShift,
                ref root, data.Slice(16 * Words, 5 * Words), signs.Slice(16, 5), data.Slice(21 * Words, 5 * Words), signs.Slice(21, 5),
                values, valueSigns, valueCell);
            int comparison = best.HasValue ? WideFiniteAxisIntersection.CompareFiniteValueRoots(value,
                RestoreObliqueValue(bestValues, bestSigns, bestCell, best)) : analytic.HasValue
                    ? ConvexContactValueRoot.CompareRootSquared(Signed192.NarrowProven(input.Scale), valueShift, ref value,
                        new ConvexContactCandidate(analytic.Metric, analytic.MetricSigns, analytic.MetricSigns[10])) : -1;
            if (comparison >= 0) continue;
            values.CopyTo(bestValues); valueSigns.CopyTo(bestSigns); valueCell.CopyTo(bestCell);
            candidate.ValueOrdinal = value.Ordinal; candidate.ValueShift = value.DenominatorShift;
            candidate.IsRational = value.IsRational; best = candidate;
        }
    }

    private static FiniteAxisValueRoot RestoreObliqueValue(ReadOnlySpan<ulong> values, ReadOnlySpan<sbyte> signs,
        Span<ulong> cell, ObliqueSelection selection) => new()
    { Coefficients = values, Signs = signs, LowerNumerator = cell, Ordinal = selection.ValueOrdinal,
        DenominatorShift = selection.ValueShift, IsRational = selection.IsRational };

    private static void RimParameter(scoped Input input, WideAxis3 first, WideAxis3 second, int region,
        out Signed576 a0, out Signed576 a1, out Signed576 b0, out Signed576 b1, out Signed576 d0, out Signed576 d1)
    {
        WideAxis3 offset = RimOffset(input, region, 1);
        WideAxis3 end = new(WideArithmetic.AddSigned320(offset.X, input.Edge.X),
            WideArithmetic.AddSigned320(offset.Y, input.Edge.Y), WideArithmetic.AddSigned320(offset.Z, input.Edge.Z));
        WideAxis3 tangent0 = new(WideArithmetic.Negate(first.Z), default, first.X),
            tangent1 = new(WideArithmetic.Negate(second.Z), default, second.X);
        a0 = WideAxis3.Dot(offset, tangent0); a1 = WideAxis3.Dot(offset, tangent1);
        b0 = WideAxis3.Dot(end, tangent0); b1 = WideAxis3.Dot(end, tangent1);
        d0 = WideArithmetic.SubtractSigned576(b0, a0); d1 = WideArithmetic.SubtractSigned576(b1, a1);
    }

    private static int RootSign(scoped ref FiniteAxisValueRoot root, Signed576 first, Signed576 second)
    {
        Span<ulong> values = stackalloc ulong[18]; Span<sbyte> signs = stackalloc sbyte[2];
        WideArithmetic.GetMagnitude(first, values[..9]); WideArithmetic.GetMagnitude(second, values[9..]);
        signs[0] = (sbyte)first.Sign; signs[1] = (sbyte)second.Sign;
        return WideFiniteAxisIntersection.GetSignAtFiniteValueRootAndRefine(ref root, values, signs);
    }

    private static bool AdmitObliqueFoot(scoped Input input, WideAxis3 first, WideAxis3 second,
        int region, int cap, scoped ref FiniteAxisValueRoot root)
    {
        if (RootSign(ref root, Extend(first.Y), Extend(second.Y)) * cap > 0
            || region != 0 && RootSign(ref root, Extend(first.Z), Extend(second.Z)) * region > 0) return false;
        foreach (WideAxis3 wall in input.Fan)
            if (RootSign(ref root, WideAxis3.Dot(first, wall), WideAxis3.Dot(second, wall)) < 0) return false;
        RimParameter(input, first, second, region, out Signed576 a0, out Signed576 a1,
            out Signed576 b0, out Signed576 b1, out Signed576 d0, out Signed576 d1);
        int denominator = RootSign(ref root, d0, d1);
        // A zero tangent divisor is meridional: L=0 then W=0 forces K=0, rejected by chart admission.
        System.Diagnostics.Debug.Assert(denominator != 0);
        int start, end;
        if (input.Clip.IsEmpty)
        {
            start = -RootSign(ref root, a0, a1) * denominator;
            end = RootSign(ref root, b0, b1) * denominator;
        }
        else
        {
            ref readonly RationalSegmentInterval clip = ref input.Clip[0];
            start = CompareObliqueParameter(ref root, a0, a1, d0, d1,
                clip.LowerNumerator, clip.LowerDenominator) * denominator;
            end = -CompareObliqueParameter(ref root, a0, a1, d0, d1,
                clip.UpperNumerator, clip.UpperDenominator) * denominator;
        }
        if (start < 0 || end < 0 || input.InteriorOnly && (start == 0 || end == 0)) return false;
        if (!input.RequireSourceInside) return true;
        // The tangent-plane ratio is the matched finite foot. Check the full
        // slab before ranking its exact gap; outer opposite feet cannot hide
        // behind the preflight's unrelated intersecting segment parameter.
        Span<ulong> coordinates = stackalloc ulong[8 * Words]; Span<sbyte> coordinateSigns = stackalloc sbyte[8];
        for (int axis = 0; axis < 3; axis++)
        {
            WriteRootCoordinate(input, axis, a0, a1, d0, d1, coordinates.Slice(axis * 2 * Words, 2 * Words),
                coordinateSigns.Slice(axis * 2, 2));
            if (denominator < 0)
            { coordinateSigns[2 * axis] = (sbyte)-coordinateSigns[2 * axis]; coordinateSigns[2 * axis + 1] = (sbyte)-coordinateSigns[2 * axis + 1]; }
        }
        CircularRimContactAlgebra.Write(coordinates, coordinateSigns, 6, denominator < 0 ? WideArithmetic.SubtractSigned576(default, d0) : d0);
        CircularRimContactAlgebra.Write(coordinates, coordinateSigns, 7, denominator < 0 ? WideArithmetic.SubtractSigned576(default, d1) : d1);
        return ContainsObliqueFoot(input, coordinates, coordinateSigns, ref root);
    }

    private static int CompareObliqueParameter(scoped ref FiniteAxisValueRoot root,
        Signed576 a0, Signed576 a1, Signed576 d0, Signed576 d1, Signed320 numerator, Signed320 denominator)
    {
        // The foot is -a(u)/d(u). A retained bound adds at most 319
        // bits to the <398-bit linear coefficients; do not narrow to Signed576.
        Span<ulong> values = stackalloc ulong[2 * Words], factor = stackalloc ulong[9],
            coefficient = stackalloc ulong[9], product = stackalloc ulong[Words];
        Span<sbyte> signs = stackalloc sbyte[2];
        for (int index = 0; index < 2; index++)
        {
            Signed576 a = index == 0 ? a0 : a1, d = index == 0 ? d0 : d1;
            Import(denominator, factor); Import(a, coefficient);
            WideArithmetic.MultiplyMagnitudes(coefficient, factor, Slot(values, index));
            int sign = -a.Sign;
            Import(numerator, factor); Import(d, coefficient);
            WideArithmetic.MultiplyMagnitudes(coefficient, factor, product);
            Add(product, -d.Sign * numerator.Sign, Slot(values, index), ref sign); signs[index] = (sbyte)sign;
        }
        return WideFiniteAxisIntersection.GetSignAtFiniteValueRootAndRefine(ref root, values, signs);
    }

    private static void WriteRootCoordinate(scoped Input input, int axis, Signed576 a0, Signed576 a1,
        Signed576 d0, Signed576 d1, Span<ulong> values, Span<sbyte> signs)
    {
        // Coordinate numerators need the complete 198+398-bit product; the
        // existing forty-word polynomial owner retains it without narrowing.
        Span<ulong> factor = stackalloc ulong[9], first = stackalloc ulong[Words], second = stackalloc ulong[Words], product = stackalloc ulong[Words];
        Import(Component(input.First, axis), factor);
        for (int index = 0; index < 2; index++)
        {
            Signed576 d = index == 0 ? d0 : d1, a = index == 0 ? a0 : a1;
            Import(d, first); WideArithmetic.MultiplyMagnitudes(first, factor, values.Slice(index * Words, Words));
            int sign = d.Sign * Component(input.First, axis).Sign;
            Import(a, second); Import(Component(input.Edge, axis), first);
            WideArithmetic.MultiplyMagnitudes(first, second, product);
            Add(product, -a.Sign * Component(input.Edge, axis).Sign, values.Slice(index * Words, Words), ref sign);
            signs[index] = (sbyte)sign;
        }
    }

    private static bool ContainsObliqueFoot(scoped Input input, scoped ReadOnlySpan<ulong> coordinates,
        scoped ReadOnlySpan<sbyte> coordinateSigns, scoped ref FiniteAxisValueRoot root)
    {
        Span<ulong> work = stackalloc ulong[13 * Words]; Span<sbyte> signs = stackalloc sbyte[13];
        Span<ulong> query = work[..(2 * Words)], z = work.Slice(2 * Words, 2 * Words),
            denominator = work.Slice(4 * Words, 2 * Words), square = work.Slice(6 * Words, 3 * Words),
            term = work.Slice(9 * Words, 3 * Words), product = Slot(work, 12);
        Span<ulong> scalar = stackalloc ulong[9];
        ReadOnlySpan<ulong> d = coordinates.Slice(6 * Words, 2 * Words); ReadOnlySpan<sbyte> dSigns = coordinateSigns.Slice(6, 2);
        Import(input.Cap, scalar);
        for (int cap = -1; cap <= 1; cap += 2)
        {
            for (int index = 0; index < 2; index++)
            {
                WideArithmetic.MultiplyMagnitudes(Slot(d, index), scalar, Slot(query, index)); int sign = dSigns[index];
                Add(Slot(coordinates, index + 2), cap * coordinateSigns[index + 2], Slot(query, index), ref sign); signs[index] = (sbyte)sign;
            }
            if (WideFiniteAxisIntersection.GetSignAtFiniteValueRootAndRefine(ref root, query, signs[..2]) < 0) return false;
        }
        coordinates.Slice(4 * Words, 2 * Words).CopyTo(z); coordinateSigns.Slice(4, 2).CopyTo(signs.Slice(2, 2));
        Import(input.Core, scalar);
        for (int region = -1; region <= 1; region += 2)
        {
            for (int index = 0; index < 2; index++)
            {
                WideArithmetic.MultiplyMagnitudes(Slot(d, index), scalar, Slot(query, index)); int sign = -region * dSigns[index];
                Add(Slot(coordinates, index + 4), coordinateSigns[index + 4], Slot(query, index), ref sign); signs[index] = (sbyte)sign;
            }
            int side = WideFiniteAxisIntersection.GetSignAtFiniteValueRootAndRefine(ref root, query, signs[..2]);
            if (side * region >= 0) { query.CopyTo(z); signs[..2].CopyTo(signs.Slice(2, 2)); break; }
            if (region == 1) { z.Clear(); signs.Slice(2, 2).Clear(); }
        }
        WideFiniteAxisIntersection.MultiplyFiniteAxisPolynomials(coordinates[..(2 * Words)], coordinateSigns[..2],
            coordinates[..(2 * Words)], coordinateSigns[..2], square, signs.Slice(6, 3), product);
        WideFiniteAxisIntersection.MultiplyFiniteAxisPolynomials(z, signs.Slice(2, 2), z, signs.Slice(2, 2), term, signs.Slice(9, 3), product);
        for (int index = 0; index < 3; index++)
        { int sign = signs[index + 6]; Add(Slot(term, index), signs[index + 9], Slot(square, index), ref sign); signs[index + 6] = (sbyte)sign; }
        Import(input.Radial, scalar);
        for (int index = 0; index < 2; index++)
        { WideArithmetic.MultiplyMagnitudes(Slot(d, index), scalar, Slot(denominator, index)); signs[index + 4] = dSigns[index]; }
        WideFiniteAxisIntersection.MultiplyFiniteAxisPolynomials(denominator, signs.Slice(4, 2), denominator, signs.Slice(4, 2), term, signs.Slice(9, 3), product);
        for (int index = 0; index < 3; index++)
        { int sign = signs[index + 9]; Add(Slot(square, index), -signs[index + 6], Slot(term, index), ref sign); signs[index + 9] = (sbyte)sign; }
        return WideFiniteAxisIntersection.GetSignAtFiniteValueRootAndRefine(ref root, term, signs.Slice(9, 3)) >= 0;
    }

    private static FixedContactAnchors MaterializeOblique(scoped Input input, ObliqueSelection best, int valueShift,
        scoped ReadOnlySpan<ulong> bestValues, scoped ReadOnlySpan<sbyte> bestSigns, scoped Span<ulong> bestCell)
    {
        Span<ulong> data = stackalloc ulong[26 * Words]; Span<sbyte> signs = stackalloc sbyte[26];
        CircularRimContactAlgebra.BuildParameter(RimOffset(input, best.Region, best.Cap), input.Radial,
            Signed192.NarrowProven(input.Scale), best.First, best.Second, data, signs);
        Span<ulong> cell = stackalloc ulong[WideFiniteAxisIntersection.GetFiniteValueRootCellWords(data[..(5 * Words)], signs[..5])];
        bool found = WideFiniteAxisIntersection.TryGetFiniteValueRoot(data[..(5 * Words)], signs[..5], best.ParameterOrdinal, cell, out FiniteAxisValueRoot root);
        System.Diagnostics.Debug.Assert(found);
        RimParameter(input, best.First, best.Second, best.Region, out Signed576 a0, out Signed576 a1,
            out Signed576 b0, out Signed576 b1, out Signed576 d0, out Signed576 d1);
        int denominator = RootSign(ref root, d0, d1); Vector3d point = default;
        for (int axis = 0; axis < 3; axis++)
            point[axis] = TriangleCylinderRimWitnesses.RoundRootCoordinate(input.Segment.Start[axis], input.Segment.End[axis],
                a0, a1, b0, b1, d0, d1, denominator, ref root);
        Span<ulong> gradient = stackalloc ulong[30]; Span<sbyte> gradientSigns = stackalloc sbyte[6];
        CircularRimContactAlgebra.WriteGradient(best.First, best.Second, gradient, gradientSigns);
        gradient.Slice(10, 10).Clear(); gradientSigns.Slice(2, 2).Clear();
        Vector3d local = ConvexContactValueRoot.GetScaledNormalizedDirection(ref root, gradient, gradientSigns, input.Radius, -1);
        Vector3d core = default; FixedPointAnchorTerm3d exactTerm = default;
        if (best.Region != 0)
        {
            core = TriangleCapsuleSlabEndpointWitnesses.GetCore(Vector2d.Forward, input.Length, -best.Region, out exactTerm);
            local = TriangleCapsuleSlabEndpointWitnesses.AdjustRoot(ref root, gradient, gradientSigns,
                input.Radius, core, exactTerm, local, out int integralMask);
            exactTerm = TriangleCapsuleSlabEndpointWitnesses.CreateTerm(Vector2d.Forward,
                best.Region < 0 ? -input.Length : input.Length, core, integralMask);
        }
        Fixed64 halfThickness = input.FullHeight == Fixed64.Zero ? input.HalfThickness : input.FullHeight * Fixed64.Half;
        local.Y = best.Cap < 0 ? -halfThickness : halfThickness;
        if ((input.FullHeight.m_rawValue & 1L) != 0)
            exactTerm = new(exactTerm.X, FixedPointAnchorTerm3d.GetResidual(Fixed64.One,
                best.Cap < 0 ? -input.FullHeight : input.FullHeight, Fixed64.Zero, Fixed64.Zero,
                core.Y, local.Y), exactTerm.Z);
        var basis = new WideRationalBasis3d(input.SlabRotation);
        CircularRimContactAlgebra.WriteGradient(CircularRimContactAlgebra.Transform(basis, best.First),
            CircularRimContactAlgebra.Transform(basis, best.Second), gradient, gradientSigns);
        Vector3d normal = ConvexContactValueRoot.GetNormalizedDirection(ref root, gradient, gradientSigns, 1);
        Fixed64 depth = default; bool clamped = false;
        if (!best.IsZero)
        {
            FiniteAxisValueRoot value = RestoreObliqueValue(bestValues, bestSigns, bestCell, best);
            ConvexContactValueRoot.GetRoundedDepth(Signed192.NarrowProven(input.Scale), valueShift, ref value, out depth, out clamped);
        }
        return new(new FixedPointAnchor(input.Origin, input.Rotation, point),
            new FixedPointAnchor(input.Center, input.SlabRotation, core, local, exactTerm), normal, depth, clamped);
    }
}
