//=======================================================================
// CurvedPrismCapWitness.cs
//=======================================================================
// MIT License, Copyright (c) 2024-present David Oravsky (mrdav30)
// See LICENSE file in the project root for full license information.
//=======================================================================
using System;
using static FixedMathSharp.Geometry.CylinderContactAlgebra;

namespace FixedMathSharp.Geometry;

/// <summary>Matched rational prism witnesses for an admitted primitive cap pole.</summary>
internal static class CurvedPrismCapWitness
{
    /// <summary>
    /// Normal is a nonzero rational local pole, parallel to the primitive axis.
    /// Returns a point on its minimum prism feature within the projected cap
    /// disk. Caller output fields have denominator.Length words (112 words
    /// cover the complete Signed576 pole bound)
    /// and are disjoint from the three forty-word input normal fields.
    /// </summary>
    internal static bool TryGetPoint(in CurvedPrismContactGeometry geometry,
        ReadOnlySpan<ulong> normal, ReadOnlySpan<int> normalSigns, int minimumVertex, int capSign,
        Span<ulong> point, Span<int> pointSigns, Span<ulong> denominator, bool initialSection = false)
    {
        int words = denominator.Length;
        Span<ulong> storage = stackalloc ulong[28 * words];
        Span<int> signs = stackalloc int[28];
        ContactQuadratic metric = ContactQuadratic.At(storage, signs, 3, words);
        ContactQuadratic minimum = ContactQuadratic.At(storage, signs, 4, words);
        ContactQuadratic divisor = ContactQuadratic.At(storage, signs, 8, words);
        ContactQuadratic t = ContactQuadratic.At(storage, signs, 9, words);
        ContactQuadratic tDivisor = ContactQuadratic.At(storage, signs, 10, words);
        ContactQuadratic work = ContactQuadratic.At(storage, signs, 11, words);
        ContactQuadratic term = ContactQuadratic.At(storage, signs, 12, words);
        ContactQuadratic sum = ContactQuadratic.At(storage, signs, 13, words);
        Span<ulong> magnitude = stackalloc ulong[Words];
        SumSquares(normal, magnitude); SetMagnitude(magnitude, 1, metric);
        Dot(geometry.Vertex(minimumVertex), normal, normalSigns, magnitude, out int minimumSign);
        SetMagnitude(magnitude, minimumSign, minimum);
        for (int axis = 0; axis < 3; axis++)
        {
            SetMagnitude(Slot(normal, axis), normalSigns[axis], term);
            ContactQuadratic.Multiply(term, minimum, ReadOnlySpan<ulong>.Empty,
                ContactQuadratic.At(storage, signs, axis, words));
        }
        Signed832 radiusSquared = initialSection && !(geometry.IsCone && capSign > 0)
            ? Signed832.ExtendValue(WideArithmetic.MultiplySigned320(geometry.Radius, geometry.Radius)) : default;
        Signed576 radiusDenominator = default;
        if (initialSection && !TryGetInitialPoleRadius(geometry, minimumVertex, capSign, ref radiusSquared, out radiusDenominator)) return false;
        // C=(minP/n²)*n is the disk center in the prism support plane.
        // An admitted pole guarantees a finite common slice. If C is outside
        // that convex feature, its nearest point is on a real boundary edge.
        if (Contains(geometry, storage[..(6 * words)], signs[..6], metric, words))
        {
            CopyPoint(storage, signs, 0, metric, point, pointSigns, denominator, words);
            return true;
        }
        if (!initialSection && !(geometry.IsCone && capSign > 0))
            radiusSquared = Signed832.ExtendValue(WideArithmetic.MultiplySigned320(geometry.Radius, geometry.Radius));
        divisor.Set(Signed576.ExtendValue(Signed192.Signed(1)));
        for (int vertex = 0; vertex < geometry.VertexCount; vertex++)
        {
            WideDirection3d candidate = geometry.Vertex(vertex);
            if (!IsMinimum(candidate, normal, normalSigns, minimum, work, magnitude)) continue;
            for (int axis = 0; axis < 3; axis++)
                ContactQuadratic.At(storage, signs, 5 + axis, words).Set(Component(candidate, axis));
            if (!WithinDisk(storage, signs, divisor, metric, radiusSquared, words, radiusDenominator)) continue;
            CopyPoint(storage, signs, 5, divisor, point, pointSigns, denominator, words);
            return true;
        }
        for (int edge = 0; edge < geometry.EdgeCount; edge++)
        {
            geometry.Edge(edge, out int first, out int second);
            WideDirection3d start = geometry.Vertex(first), end = geometry.Vertex(second);
            if (!IsMinimum(start, normal, normalSigns, minimum, work, magnitude)
                || !IsMinimum(end, normal, normalSigns, minimum, work, magnitude)) continue;
            WideDirection3d delta = new(WideArithmetic.SubtractSigned576(end.X, start.X),
                WideArithmetic.SubtractSigned576(end.Y, start.Y), WideArithmetic.SubtractSigned576(end.Z, start.Z));
            if (delta.IsZero) continue;
            t.Clear();
            for (int axis = 0; axis < 3; axis++)
            {
                ContactQuadratic.Scale(metric, Signed832.ExtendValue(Component(start, axis)), work);
                ContactQuadratic.At(storage, signs, axis, words).CopyTo(sum); sum.Add(work, -1);
                ContactQuadratic.Scale(sum, Signed832.ExtendValue(Component(delta, axis)), term); t.Add(term);
            }
            ContactQuadratic.Scale(metric, delta.SquaredLength, tDivisor);
            // Endpoint candidates were already rejected above; only the open
            // edge can provide a new nearest point inside the disk.
            if (t.Signs[0] <= 0) continue;
            t.CopyTo(work); work.Add(tDivisor, -1);
            if (work.Signs[0] >= 0) continue;
            for (int axis = 0; axis < 3; axis++)
            {
                ContactQuadratic destination = ContactQuadratic.At(storage, signs, 5 + axis, words);
                ContactQuadratic.Scale(tDivisor, Signed832.ExtendValue(Component(start, axis)), destination);
                ContactQuadratic.Scale(t, Signed832.ExtendValue(Component(delta, axis)), term); destination.Add(term);
            }
            if (!WithinDisk(storage, signs, tDivisor, metric, radiusSquared, words, radiusDenominator)) continue;
            CopyPoint(storage, signs, 5, tDivisor, point, pointSigns, denominator, words);
            return true;
        }
        return false;
    }

    private static bool TryGetInitialPoleRadius(in CurvedPrismContactGeometry geometry, int minimumVertex,
        int capSign, ref Signed832 radiusSquared, out Signed576 radiusDenominator)
    {
        radiusDenominator = default;
        // Reduced-pole projections need <232 bits. Seek an initial-body
        // point before falling back to the larger translated support disk.
        Signed832 axialProjection = WideDirection3d.Dot(new WideDirection3d(geometry.Axis), geometry.Vertex(minimumVertex));
        Signed832 half = Signed832.ExtendValue(WideArithmetic.MultiplySigned320(geometry.SquaredAxis, geometry.HalfFactor));
        Signed832 height = WideArithmetic.SubtractSigned832(half, axialProjection);
        if (height.Sign < 0 || WideArithmetic.AddSigned832(half, axialProjection).Sign < 0) return false;
        if (geometry.IsCone && capSign < 0)
        {
            var h = Signed320.NarrowValue(Signed576.NarrowValue(height));
            radiusSquared = WideArithmetic.MultiplySigned576(Signed576.NarrowValue(radiusSquared), WideArithmetic.MultiplySigned320(h, h));
            var t2 = WideArithmetic.MultiplySigned320(Signed320.ExtendValue(geometry.HalfFactor), geometry.HalfFactor);
            radiusDenominator = Signed576.NarrowValue(WideArithmetic.MultiplySigned576(t2,
                WideArithmetic.MultiplySigned320(geometry.SquaredAxis, geometry.SquaredAxis)));
            radiusDenominator = WideArithmetic.AddSigned576(radiusDenominator, radiusDenominator);
            radiusDenominator = WideArithmetic.AddSigned576(radiusDenominator, radiusDenominator);
        }
        return true;
    }

    // A pole's disk is free in two dimensions. Native side rectangles have
    // orthogonal edge/up axes even with the exact nonunit scalar-yaw metric.
    // Reduce the pole to the authored axis before projecting: its components
    // need <65 bits, so clipped rational points and disk tests fit 112 words.
    internal static bool TryGetWindowPoint(in CurvedPrismContactGeometry geometry, int minimumVertex, int capSign,
        int face, int sourceEdge, in RationalSegmentInterval clip, bool open,
        Fixed64 lower, Fixed64 upper, Span<ulong> point, Span<int> pointSigns, Span<ulong> denominator, bool initialSection = false)
    {
        int words = denominator.Length;
        Span<ulong> fields = stackalloc ulong[34 * words]; Span<int> signs = stackalloc int[34];
        fields.Clear(); signs.Clear();
        var metric = ContactQuadratic.At(fields, signs, 3, words);
        var candidateD = ContactQuadratic.At(fields, signs, 8, words);
        var n = ContactQuadratic.At(fields, signs, 9, words); var d = ContactQuadratic.At(fields, signs, 10, words);
        var un = ContactQuadratic.At(fields, signs, 11, words); var ud = ContactQuadratic.At(fields, signs, 12, words);
        var work = ContactQuadratic.At(fields, signs, 13, words); var term = ContactQuadratic.At(fields, signs, 14, words);
        var axis = new WideDirection3d(geometry.Axis); if (capSign < 0) axis = -axis;
        metric.Set(axis.SquaredLength);
        Signed832 minimum = WideDirection3d.Dot(axis, geometry.Vertex(minimumVertex));
        for (int coordinate = 0; coordinate < 3; coordinate++)
            ContactQuadratic.At(fields, signs, coordinate, words).Set(WideArithmetic.MultiplySigned832(minimum,
                Signed320.NarrowValue(Component(axis, coordinate))));
        int edgeIndex = sourceEdge >= 0 ? sourceEdge : 3 * face;
        geometry.Edge(edgeIndex, out int first, out int last);
        WideDirection3d a = geometry.Vertex(first), e = CurvedPrismContact.Difference(geometry.Vertex(last), a);
        Project(a, e, fields, signs, metric, n, d, words);
        Clamp(n, d, clip.IsClipped ? clip.LowerNumerator : default, clip.IsClipped ? clip.LowerDenominator : Signed320.One,
            clip.IsClipped ? clip.UpperNumerator : Signed320.One, clip.IsClipped ? clip.UpperDenominator : Signed320.One, work, term);
        WideDirection3d up = default;
        if (sourceEdge < 0)
        {
            up = CurvedPrismContact.Difference(geometry.Vertex(first + 1), a);
            Project(a, up, fields, signs, metric, un, ud, words);
            Signed320 height = Signed320.ExtendValue(WideArithmetic.AddSigned192(Signed192.Raw(geometry.PrismHalfThickness), Signed192.Raw(geometry.PrismHalfThickness)));
            Clamp(un, ud, Signed320.ExtendValue(WideArithmetic.AddSigned192(Signed192.Raw(lower), Signed192.Raw(geometry.PrismHalfThickness))), height,
                Signed320.ExtendValue(WideArithmetic.AddSigned192(Signed192.Raw(upper), Signed192.Raw(geometry.PrismHalfThickness))), height, work, term);
        }
        else { un.Clear(); ud.Set(Signed576.ExtendValue(Signed192.Signed(1))); }
        ContactQuadratic.Multiply(d, ud, ReadOnlySpan<ulong>.Empty, candidateD);
        for (int coordinate = 0; coordinate < 3; coordinate++)
        {
            var value = ContactQuadratic.At(fields, signs, 5 + coordinate, words);
            ContactQuadratic.Scale(candidateD, Component(a, coordinate), value);
            ContactQuadratic.Multiply(n, ud, ReadOnlySpan<ulong>.Empty, work);
            ContactQuadratic.Scale(work, Component(e, coordinate), term); value.Add(term);
            if (!up.IsZero)
            {
                ContactQuadratic.Multiply(un, d, ReadOnlySpan<ulong>.Empty, work);
                ContactQuadratic.Scale(work, Component(up, coordinate), term); value.Add(term);
            }
        }
        Signed832 radiusSquared = geometry.IsCone && capSign > 0 ? default
            : Signed832.ExtendValue(WideArithmetic.MultiplySigned320(geometry.Radius, geometry.Radius));
        Signed576 radiusDenominator = default;
        if (initialSection && !TryGetInitialPoleRadius(geometry, minimumVertex, capSign, ref radiusSquared, out radiusDenominator)) return false;
        if (!WithinDisk(fields, signs, candidateD, metric, radiusSquared, words, radiusDenominator)) return false;
        if (open && !StrictlyInside(n, d, clip, work, term))
        {
            if (!MoveInsideOpenEdge(a, e, clip, fields, signs, metric, radiusSquared, radiusDenominator, candidateD, words)) return false;
        }
        CopyPoint(fields, signs, 5, candidateD, point, pointSigns, denominator, words); return true;
    }

    private static void Project(in WideDirection3d a, in WideDirection3d edge, Span<ulong> fields, Span<int> signs,
        ContactQuadratic metric, ContactQuadratic n, ContactQuadratic d, int words)
    {
        var work = ContactQuadratic.At(fields, signs, 15, words); var term = ContactQuadratic.At(fields, signs, 16, words);
        n.Clear();
        for (int axis = 0; axis < 3; axis++)
        {
            ContactQuadratic.Scale(metric, Component(a, axis), work);
            work.MultiplySign(-1); work.Add(ContactQuadratic.At(fields, signs, axis, words));
            ContactQuadratic.Scale(work, Component(edge, axis), term); n.Add(term);
        }
        ContactQuadratic.Scale(metric, edge.SquaredLength, d);
    }

    private static void Clamp(ContactQuadratic n, ContactQuadratic d, Signed320 ln, Signed320 ld, Signed320 un, Signed320 ud,
        ContactQuadratic work, ContactQuadratic term)
    {
        ContactQuadratic.Scale(n, Signed576.ExtendValue(ld), work); ContactQuadratic.Scale(d, Signed576.ExtendValue(ln), term); work.Add(term, -1);
        if (work.Signs[0] < 0) { n.Set(Signed576.ExtendValue(ln)); d.Set(Signed576.ExtendValue(ld)); }
        ContactQuadratic.Scale(n, Signed576.ExtendValue(ud), work); ContactQuadratic.Scale(d, Signed576.ExtendValue(un), term); work.Add(term, -1);
        if (work.Signs[0] > 0) { n.Set(Signed576.ExtendValue(un)); d.Set(Signed576.ExtendValue(ud)); }
    }

    private static bool StrictlyInside(ContactQuadratic n, ContactQuadratic d, in RationalSegmentInterval clip,
        ContactQuadratic work, ContactQuadratic term)
    {
        ContactQuadratic.Scale(n, Signed576.ExtendValue(clip.IsClipped ? clip.LowerDenominator : Signed320.One), work);
        ContactQuadratic.Scale(d, Signed576.ExtendValue(clip.IsClipped ? clip.LowerNumerator : default), term); work.Add(term, -1);
        if (work.Signs[0] <= 0) return false;
        ContactQuadratic.Scale(n, Signed576.ExtendValue(clip.IsClipped ? clip.UpperDenominator : Signed320.One), work);
        ContactQuadratic.Scale(d, Signed576.ExtendValue(clip.IsClipped ? clip.UpperNumerator : Signed320.One), term); work.Add(term, -1);
        return work.Signs[0] < 0;
    }

    private static bool MoveInsideOpenEdge(in WideDirection3d a, in WideDirection3d edge, in RationalSegmentInterval clip,
        Span<ulong> fields, Span<int> signs, ContactQuadratic metric, Signed832 radiusSquared, Signed576 radiusDenominator, ContactQuadratic denominator, int words)
    {
        // Q is the true nearest closed-window point. With positive disk slack,
        // blend toward the interval mediant F by t=g/(g+2|d.v|+v*v). Then
        // 0<t<=1 and the whole exact point remains strictly inside the disk.
        Span<ulong> scratch = stackalloc ulong[30 * words]; Span<int> scratchSigns = stackalloc int[30];
        var fd = ContactQuadratic.At(scratch, scratchSigns, 0, words); var fn = ContactQuadratic.At(scratch, scratchSigns, 1, words);
        var g = ContactQuadratic.At(scratch, scratchSigns, 2, words); var k = ContactQuadratic.At(scratch, scratchSigns, 3, words);
        var work = ContactQuadratic.At(scratch, scratchSigns, 4, words); var term = ContactQuadratic.At(scratch, scratchSigns, 5, words);
        var deltaD = ContactQuadratic.At(scratch, scratchSigns, 6, words); var qd = ContactQuadratic.At(scratch, scratchSigns, 7, words);
        fn.Set(Signed576.ExtendValue(WideArithmetic.AddSigned320(clip.IsClipped ? clip.LowerNumerator : default, clip.IsClipped ? clip.UpperNumerator : Signed320.One)));
        fd.Set(Signed576.ExtendValue(WideArithmetic.AddSigned320(clip.IsClipped ? clip.LowerDenominator : Signed320.One, clip.IsClipped ? clip.UpperDenominator : Signed320.One)));
        ContactQuadratic.Multiply(denominator, fd, ReadOnlySpan<ulong>.Empty, deltaD);
        ContactQuadratic.Multiply(deltaD, metric, ReadOnlySpan<ulong>.Empty, qd);
        ContactQuadratic.Multiply(qd, qd, ReadOnlySpan<ulong>.Empty, work); ContactQuadratic.Scale(work, radiusSquared, g); k.Clear();
        for (int axis = 0; axis < 3; axis++)
        {
            var q = ContactQuadratic.At(scratch, scratchSigns, 8 + axis, words); var v = ContactQuadratic.At(scratch, scratchSigns, 11 + axis, words);
            ContactQuadratic.Scale(fd, Component(a, axis), work); ContactQuadratic.Scale(fn, Component(edge, axis), term); work.Add(term);
            ContactQuadratic.Multiply(work, denominator, ReadOnlySpan<ulong>.Empty, v);
            ContactQuadratic.Multiply(ContactQuadratic.At(fields, signs, 5 + axis, words), fd, ReadOnlySpan<ulong>.Empty, q);
            v.Add(q, -1); q.CopyTo(work); ContactQuadratic.Multiply(work, metric, ReadOnlySpan<ulong>.Empty, q);
            ContactQuadratic.Multiply(ContactQuadratic.At(fields, signs, axis, words), deltaD, ReadOnlySpan<ulong>.Empty, term); q.Add(term, -1);
            v.CopyTo(work); ContactQuadratic.Multiply(work, metric, ReadOnlySpan<ulong>.Empty, v);
            ContactQuadratic.Multiply(q, q, ReadOnlySpan<ulong>.Empty, work);
            if (radiusDenominator.IsZero) g.Add(work, -1);
            else { ContactQuadratic.Scale(work, radiusDenominator, term); g.Add(term, -1); }
        }
        if (g.Signs[0] <= 0) return false; // Tangency at an excluded endpoint admits no open point.
        var dot = ContactQuadratic.At(scratch, scratchSigns, 14, words); dot.Clear();
        for (int axis = 0; axis < 3; axis++)
        {
            var q = ContactQuadratic.At(scratch, scratchSigns, 8 + axis, words); var v = ContactQuadratic.At(scratch, scratchSigns, 11 + axis, words);
            ContactQuadratic.Multiply(q, v, ReadOnlySpan<ulong>.Empty, work); dot.Add(work);
            ContactQuadratic.Multiply(v, v, ReadOnlySpan<ulong>.Empty, work); k.Add(work);
        }
        // Q is the exact projection onto the same interval containing F,
        // so (Q-C).(F-Q)>=0; no absolute-value branch is needed.
        System.Diagnostics.Debug.Assert(dot.Signs[0] >= 0);
        k.Add(dot); k.Add(dot);
        if (!radiusDenominator.IsZero) { ContactQuadratic.Scale(k, radiusDenominator, work); work.CopyTo(k); }
        k.Add(g);
        for (int axis = 0; axis < 3; axis++)
        {
            var value = ContactQuadratic.At(fields, signs, 5 + axis, words);
            ContactQuadratic.Multiply(value, fd, ReadOnlySpan<ulong>.Empty, work);
            ContactQuadratic.Multiply(work, k, ReadOnlySpan<ulong>.Empty, ContactQuadratic.At(scratch, scratchSigns, 8 + axis, words));
            // v includes metric; divide it out by multiplying the other terms.
            ContactQuadratic.Multiply(ContactQuadratic.At(scratch, scratchSigns, 8 + axis, words), metric, ReadOnlySpan<ulong>.Empty, work);
            ContactQuadratic.Multiply(ContactQuadratic.At(scratch, scratchSigns, 11 + axis, words), g, ReadOnlySpan<ulong>.Empty, term); work.Add(term); work.CopyTo(value);
        }
        ContactQuadratic.Multiply(deltaD, k, ReadOnlySpan<ulong>.Empty, work);
        ContactQuadratic.Multiply(work, metric, ReadOnlySpan<ulong>.Empty, denominator); return true;
    }

    private static bool Contains(in CurvedPrismContactGeometry geometry,
        Span<ulong> point, Span<int> pointSigns, ContactQuadratic denominator, int words)
    {
        Span<ulong> storage = stackalloc ulong[6 * words];
        Span<int> signs = stackalloc int[6];
        ContactQuadratic sum = ContactQuadratic.At(storage, signs, 0, words);
        ContactQuadratic term = ContactQuadratic.At(storage, signs, 1, words);
        ContactQuadratic bound = ContactQuadratic.At(storage, signs, 2, words);
        for (int plane = 0; plane < geometry.PlaneCount; plane++)
        {
            WideDirection3d normal = geometry.Plane(plane);
            if (normal.IsZero) continue;
            for (int orientation = 0; orientation < 2; orientation++)
            {
                bound.Set(geometry.ProjectionMinimum(normal));
                ContactQuadratic.Multiply(bound, denominator, ReadOnlySpan<ulong>.Empty, sum);
                sum.MultiplySign(-1);
                for (int axis = 0; axis < 3; axis++)
                {
                    ContactQuadratic.Scale(ContactQuadratic.At(point, pointSigns, axis, words),
                        Signed832.ExtendValue(Component(normal, axis)), term); sum.Add(term);
                }
                if (sum.Signs[0] < 0) return false;
                normal = -normal;
            }
        }
        return true;
    }

    private static bool WithinDisk(Span<ulong> values, Span<int> valueSigns,
        ContactQuadratic denominator, ContactQuadratic metric, Signed832 radiusSquared, int words, Signed576 radiusDenominator = default)
    {
        // Coordinates <165 bits and pole components <576 bits give metric
        // <1154, center numerators <1320 and edge point numerators <1656 bits.
        // Complete squared-distance products are <5700 bits, within the
        // caller's 112-word banks, including every signed-sum carry.
        Span<ulong> storage = stackalloc ulong[10 * words];
        Span<int> signs = stackalloc int[10];
        ContactQuadratic difference = ContactQuadratic.At(storage, signs, 0, words);
        ContactQuadratic product = ContactQuadratic.At(storage, signs, 1, words);
        ContactQuadratic sum = ContactQuadratic.At(storage, signs, 2, words);
        ContactQuadratic divisor = ContactQuadratic.At(storage, signs, 3, words);
        ContactQuadratic limit = ContactQuadratic.At(storage, signs, 4, words);
        sum.Clear();
        for (int axis = 0; axis < 3; axis++)
        {
            ContactQuadratic.Multiply(ContactQuadratic.At(values, valueSigns, 5 + axis, words),
                metric, ReadOnlySpan<ulong>.Empty, difference);
            ContactQuadratic.Multiply(ContactQuadratic.At(values, valueSigns, axis, words),
                denominator, ReadOnlySpan<ulong>.Empty, product); difference.Add(product, -1);
            ContactQuadratic.Multiply(difference, difference, ReadOnlySpan<ulong>.Empty, product); sum.Add(product);
        }
        if (!radiusDenominator.IsZero)
        { ContactQuadratic.Scale(sum, radiusDenominator, product); product.CopyTo(sum); }
        ContactQuadratic.Multiply(denominator, metric, ReadOnlySpan<ulong>.Empty, divisor);
        ContactQuadratic.Multiply(divisor, divisor, ReadOnlySpan<ulong>.Empty, product);
        ContactQuadratic.Scale(product, radiusSquared, limit); sum.Add(limit, -1);
        return sum.Signs[0] <= 0;
    }

    private static bool IsMinimum(in WideDirection3d point, ReadOnlySpan<ulong> normal,
        ReadOnlySpan<int> normalSigns, ContactQuadratic minimum, ContactQuadratic work, Span<ulong> magnitude)
    {
        Dot(point, normal, normalSigns, magnitude, out int sign);
        SetMagnitude(magnitude, sign, work); work.Add(minimum, -1);
        return work.Signs[0] == 0;
    }

    private static void SetMagnitude(ReadOnlySpan<ulong> magnitude, int sign, ContactQuadratic result)
    {
        magnitude.CopyTo(result.Rational); result.Rational[magnitude.Length..].Clear(); result.Radical.Clear();
        result.Signs[0] = sign; result.Signs[1] = 0;
    }

    private static void CopyPoint(Span<ulong> values, Span<int> signs, int first,
        ContactQuadratic divisor, Span<ulong> point, Span<int> pointSigns, Span<ulong> denominator, int words)
    {
        for (int axis = 0; axis < 3; axis++)
        {
            ContactQuadratic coordinate = ContactQuadratic.At(values, signs, first + axis, words);
            coordinate.Rational.CopyTo(point.Slice(axis * words, words)); pointSigns[axis] = coordinate.Signs[0];
        }
        divisor.Rational.CopyTo(denominator);
    }
}
