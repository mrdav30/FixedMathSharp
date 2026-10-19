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
        Span<ulong> point, Span<int> pointSigns, Span<ulong> denominator)
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
        // C=(minP/n²)*n is the disk center in the prism support plane.
        // An admitted pole guarantees a finite common slice. If C is outside
        // that convex feature, its nearest point is on a real boundary edge.
        if (Contains(geometry, storage[..(6 * words)], signs[..6], metric, words))
        {
            CopyPoint(storage, signs, 0, metric, point, pointSigns, denominator, words);
            return true;
        }
        Signed832 radiusSquared = geometry.IsCone && capSign > 0 ? default
            : Signed832.ExtendValue(WideArithmetic.MultiplySigned320(geometry.Radius, geometry.Radius));
        divisor.Set(Signed576.ExtendValue(Signed192.Signed(1)));
        for (int vertex = 0; vertex < geometry.VertexCount; vertex++)
        {
            WideDirection3d candidate = geometry.Vertex(vertex);
            if (!IsMinimum(candidate, normal, normalSigns, minimum, work, magnitude)) continue;
            for (int axis = 0; axis < 3; axis++)
                ContactQuadratic.At(storage, signs, 5 + axis, words).Set(Component(candidate, axis));
            if (!WithinDisk(storage, signs, divisor, metric, radiusSquared, words)) continue;
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
            if (!WithinDisk(storage, signs, tDivisor, metric, radiusSquared, words)) continue;
            CopyPoint(storage, signs, 5, tDivisor, point, pointSigns, denominator, words);
            return true;
        }
        return false;
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
        ContactQuadratic denominator, ContactQuadratic metric, Signed832 radiusSquared, int words)
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
