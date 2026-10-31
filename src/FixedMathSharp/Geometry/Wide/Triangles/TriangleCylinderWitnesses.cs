//=======================================================================
// TriangleCylinderWitnesses.cs
//=======================================================================
// MIT License, Copyright (c) 2024-present David Oravsky (mrdav30)
// See LICENSE file in the project root for full license information.
//=======================================================================

using System;
using static FixedMathSharp.Geometry.CylinderContactAlgebra;

namespace FixedMathSharp.Geometry;

/// <summary>Selected triangle-feature witnesses, with one final coordinate rounding.</summary>
internal static class TriangleCylinderWitnesses
{
    /// <summary>
    /// Intersects the selected support feature with the radial tangent plane,
    /// then with the finite axial interval. No cap endpoint is used as a proxy
    /// for the free side segment, and barycentric parameters never round first.
    /// Shifted core-region coordinates are below 232 bits and analytic
    /// directions below 630. Tangent weights stay below 864 bits, axial
    /// numerators below 1098, and blended weights below 1964. Their coordinate
    /// products remain below 2200 bits, inside forty-word candidate storage.
    /// </summary>
    internal static bool TryGetSidePoint(in TriangleCircularGeometry geometry, FixedTriangle triangle,
        int mask, ReadOnlySpan<ulong> normal, ReadOnlySpan<int> normalSigns,
        out Vector3d point, out Fixed64 axial, out int cap)
    {
        point = default; axial = default; cap = 0;
        Span<ulong> tangent = stackalloc ulong[3 * Words];
        Span<int> tangentSigns = stackalloc int[3];
        tangent.Clear(); tangentSigns.Clear();
        Slot(normal, 2).CopyTo(Slot(tangent, 0)); tangentSigns[0] = -normalSigns[2];
        Slot(normal, 0).CopyTo(Slot(tangent, 2)); tangentSigns[2] = normalSigns[0];
        Span<ulong> distances = stackalloc ulong[3 * Words];
        Span<int> distanceSigns = stackalloc int[3];
        Span<ulong> candidates = stackalloc ulong[6 * Words];
        Span<ulong> axialValues = stackalloc ulong[2 * Words];
        Span<int> axialSigns = stackalloc int[2];
        candidates.Clear();
        for (int vertex = 0; vertex < 3; vertex++)
            Dot(geometry.Vertex(vertex), tangent, tangentSigns, Slot(distances, vertex), out distanceSigns[vertex]);
        int count = 0;
        for (int vertex = 0; vertex < 3; vertex++)
        {
            if ((mask & (1 << vertex)) != 0 && distanceSigns[vertex] == 0)
                candidates[(count++ * 3 + vertex) * Words] = 1;
        }
        for (int edge = 0; edge < 3; edge++)
        {
            int next = (edge + 1) % 3;
            if ((mask & (1 << edge)) == 0 || (mask & (1 << next)) == 0
                || distanceSigns[edge] * distanceSigns[next] >= 0)
                continue;
            Slot(distances, next).CopyTo(Slot(candidates, 3 * count + edge));
            Slot(distances, edge).CopyTo(Slot(candidates, 3 * count + next));
            count++;
        }
        if (count == 0) return false;
        System.Diagnostics.Debug.Assert(count <= 2);
        Span<ulong> weights = stackalloc ulong[3 * Words];
        Span<ulong> denominator = stackalloc ulong[Words];
        Span<ulong> bound = stackalloc ulong[Words];
        Span<ulong> half = stackalloc ulong[Words];
        Import(geometry.HalfHeight, half);
        int selected = -1;
        cap = 0;
        for (int index = 0; index < count; index++)
        {
            ReadOnlySpan<ulong> candidate = candidates.Slice(index * 3 * Words, 3 * Words);
            SumWeights(candidate, denominator);
            SumCoordinates(geometry, candidate, 1, Slot(axialValues, index), out axialSigns[index]);
            WideArithmetic.MultiplyMagnitudes(denominator, half, bound);
            int comparison = WideArithmetic.CompareMagnitudeEqualLength(Slot(axialValues, index), bound);
            if (comparison <= 0)
            {
                selected = index;
                if (comparison == 0)
                    cap = -axialSigns[index];
                break;
            }
        }
        if (selected >= 0)
            candidates.Slice(selected * 3 * Words, 3 * Words).CopyTo(weights);
        else
        {
            // The minimizing side support proves an admitted axial point.
            // If neither endpoint is in the interval, they straddle it. Blend
            // directly at Y=0: w=w0*|Y1 numerator|+w1*|Y0 numerator|.
            if (count != 2 || axialSigns[0] * axialSigns[1] >= 0) return false;
            Span<ulong> product = stackalloc ulong[Words];
            for (int vertex = 0; vertex < 3; vertex++)
            {
                WideArithmetic.MultiplyMagnitudes(Slot(candidates, vertex), Slot(axialValues, 1), Slot(weights, vertex));
                WideArithmetic.MultiplyMagnitudes(Slot(candidates, vertex + 3), Slot(axialValues, 0), product);
                WideArithmetic.AddMagnitudeInto(product, Slot(weights, vertex));
            }
        }
        SumWeights(weights, denominator);
        SumCoordinates(geometry, weights, 1, bound, out int axialSign);
        Import(Signed576.ExtendValue(geometry.RawScale), half);
        Span<ulong> scaledDenominator = stackalloc ulong[Words];
        WideArithmetic.MultiplyMagnitudes(denominator, half, scaledDenominator);
        bool represented = Fixed64.TryGetSignedRawRatio(bound, scaledDenominator, axialSign < 0, out axial);
        System.Diagnostics.Debug.Assert(represented);
        point = RoundTriangle(geometry, triangle, weights, denominator);
        return true;
    }

    /// <summary>
    /// Projects the origin onto the selected cap-pole feature, retaining its exact barycentric
    /// weights until both authored and cylinder-local coordinates round.
    /// A cap-pole minimum proves the projected radial point is in the disk;
    /// its final-rounded coordinates must not be normalized a second time.
    /// </summary>
    internal static Vector3d GetCapPoint(in TriangleCircularGeometry geometry, FixedTriangle triangle,
        int mask, out Vector3d projectedRadial)
    {
        bool admitted = TryGetCapPoint(geometry, triangle, mask, out Vector3d point, out projectedRadial, false);
        System.Diagnostics.Debug.Assert(admitted);
        return point;
    }

    internal static bool TryGetCapPoint(in TriangleCircularGeometry geometry, FixedTriangle triangle,
        int mask, out Vector3d point, out Vector3d projectedRadial, bool requireAdmission = true)
    {
        point = projectedRadial = default;
        Span<ulong> weights = stackalloc ulong[3 * Words];
        Span<ulong> denominator = stackalloc ulong[Words];
        GetCapWeights(geometry, mask, weights);
        SumWeights(weights, denominator);
        if (requireAdmission)
        {
            Span<ulong> work = stackalloc ulong[3 * Words];
            SumCoordinates(geometry, weights, 0, Slot(work, 0), out _);
            SumCoordinates(geometry, weights, 2, Slot(work, 1), out _);
            Span<ulong> radial = stackalloc ulong[Words];
            Import(geometry.Radius, radial);
            WideArithmetic.MultiplyMagnitudes(denominator, radial, Slot(work, 2));
            // Affine barycentric numerators fit the existing bank; their
            // squares need complete products plus the radial-sum carry.
            int squareWords = 2 * Math.Max(WideArithmetic.GetActiveMagnitudeLength(Slot(work, 0)),
                Math.Max(WideArithmetic.GetActiveMagnitudeLength(Slot(work, 1)), WideArithmetic.GetActiveMagnitudeLength(Slot(work, 2)))) + 1;
            Span<ulong> squares = stackalloc ulong[3 * squareWords];
            Span<ulong> sum = squares[..squareWords], term = squares.Slice(squareWords, squareWords), limit = squares.Slice(2 * squareWords, squareWords);
            WideArithmetic.MultiplyMagnitudes(Slot(work, 0), Slot(work, 0), sum);
            WideArithmetic.MultiplyMagnitudes(Slot(work, 1), Slot(work, 1), term);
            WideArithmetic.AddMagnitudeInto(term, sum);
            WideArithmetic.MultiplyMagnitudes(Slot(work, 2), Slot(work, 2), limit);
            if (WideArithmetic.CompareMagnitudeEqualLength(sum, limit) > 0) return false;
        }
        point = RoundTriangle(geometry, triangle, weights, denominator);
        Span<ulong> scale = stackalloc ulong[Words];
        Span<ulong> scaledDenominator = stackalloc ulong[Words];
        Span<ulong> coordinate = stackalloc ulong[Words];
        Import(Signed576.ExtendValue(geometry.RawScale), scale);
        WideArithmetic.MultiplyMagnitudes(denominator, scale, scaledDenominator);
        SumCoordinates(geometry, weights, 0, coordinate, out int sign);
        bool representedX = Fixed64.TryGetSignedRawRatio(coordinate, scaledDenominator, sign < 0, out Fixed64 x);
        SumCoordinates(geometry, weights, 2, coordinate, out sign);
        bool representedZ = Fixed64.TryGetSignedRawRatio(coordinate, scaledDenominator, sign < 0, out Fixed64 z);
        System.Diagnostics.Debug.Assert(representedX && representedZ); // Both bounded by the authored radius.
        projectedRadial = new Vector3d(x, Fixed64.Zero, z);
        return true;
    }

    internal static void GetCapWeights(in TriangleCircularGeometry geometry, int mask, Span<ulong> weights)
    {
        weights.Clear();
        if (mask == 7)
            GetFaceWeights(geometry, weights);
        else if ((mask & (mask - 1)) == 0)
            weights[((mask & 1) != 0 ? 0 : (mask & 2) != 0 ? 1 : 2) * Words] = 1;
        else
            GetEdgeWeights(geometry, mask, weights);
    }

    private static void GetEdgeWeights(in TriangleCircularGeometry geometry, int mask, Span<ulong> weights)
    {
        int first = (mask & 1) != 0 ? 0 : 1;
        int second = (mask & 4) != 0 ? 2 : 1;
        WideDirection3d edge = geometry.EdgeFromTo(first, second);
        // A pole's selected feature has constant local Y. Projecting the
        // origin is identical to projecting either cap center onto it.
        Signed832 parameter = WideDirection3d.Dot(TriangleCircularGeometry.Subtract(default, geometry.Vertex(first)), edge);
        Signed832 denominatorValue = WideArithmetic.MultiplySigned832(edge.SquaredLength, geometry.EdgeScale);
        if (parameter.Sign <= 0)
        {
            weights[first * Words] = 1;
            return;
        }
        Signed832 remainder = WideArithmetic.SubtractSigned832(denominatorValue, parameter);
        if (remainder.Sign <= 0)
        {
            weights[second * Words] = 1;
            return;
        }
        Import(remainder, Slot(weights, first)); Import(parameter, Slot(weights, second));
    }

    private static void GetFaceWeights(in TriangleCircularGeometry geometry, Span<ulong> weights)
    {
        WideDirection3d e = geometry.Edge(0), f = -geometry.Edge(2);
        WideDirection3d offset = TriangleCircularGeometry.Subtract(default, geometry.A);
        Signed832 d1 = WideDirection3d.Dot(e, offset), d2 = WideDirection3d.Dot(f, offset);
        if (d1.Sign <= 0 && d2.Sign <= 0)
        {
            weights[0] = 1;
            return;
        }
        Signed832 ee = e.SquaredLength, ff = f.SquaredLength, ef = WideDirection3d.Dot(e, f);
        Signed832 d3 = WideArithmetic.SubtractSigned832(d1, WideArithmetic.MultiplySigned832(ee, geometry.EdgeScale));
        Signed832 d4 = WideArithmetic.SubtractSigned832(d2, WideArithmetic.MultiplySigned832(ef, geometry.EdgeScale));
        if (d3.Sign >= 0 && WideArithmetic.SubtractSigned832(d4, d3).Sign <= 0)
        {
            weights[Words] = 1;
            return;
        }
        // Retained edges are <196 bits even for shifted core regions. Gram
        // dots are <394, offset dots <430; cancelled barycentric minors are
        // <825 and the EdgeScale-adjusted alpha <827, within Signed832.
        Span<ulong> minors = stackalloc ulong[4 * Words];
        Span<ulong> vc = Slot(minors, 0), vb = Slot(minors, 1), va = Slot(minors, 2), determinant = Slot(minors, 3);
        DifferenceOfProducts(ee, d2, ef, d1, vc, out int vcSign);
        if (vcSign <= 0 && d1.Sign >= 0 && d3.Sign <= 0)
        {
            SetEdgeWeights(0, 1, WideArithmetic.SubtractSigned832(default, d3), d1, weights);
            return;
        }
        Signed832 d5 = WideArithmetic.SubtractSigned832(d1, WideArithmetic.MultiplySigned832(ef, geometry.EdgeScale));
        Signed832 d6 = WideArithmetic.SubtractSigned832(d2, WideArithmetic.MultiplySigned832(ff, geometry.EdgeScale));
        if (d6.Sign >= 0 && WideArithmetic.SubtractSigned832(d5, d6).Sign <= 0)
        {
            weights[2 * Words] = 1;
            return;
        }
        DifferenceOfProducts(ff, d1, ef, d2, vb, out int vbSign);
        // Once A/B/AB are excluded, vb<=0 proves d2>=0: the opposite
        // signs would force the earlier AB or B Voronoi region (Gram det>0).
        if (vbSign <= 0 && d6.Sign <= 0)
        {
            SetEdgeWeights(0, 2, WideArithmetic.SubtractSigned832(default, d6), d2, weights);
            return;
        }
        DifferenceOfProducts(ee, ff, ef, ef, determinant, out int vaSign);
        Span<ulong> factor = stackalloc ulong[Words];
        Import(Signed576.ExtendValue(geometry.EdgeScale), factor);
        WideArithmetic.MultiplyMagnitudes(determinant, factor, va);
        Add(vb, -vbSign, va, ref vaSign); Add(vc, -vcSign, va, ref vaSign);
        Signed832 first = WideArithmetic.SubtractSigned832(d4, d3), second = WideArithmetic.SubtractSigned832(d5, d6);
        // The preceding vertex and edge regions already prove both BC
        // weights nonnegative whenever va<=0 for this nondegenerate face.
        if (vaSign <= 0)
        {
            SetEdgeWeights(1, 2, second, first, weights);
            return;
        }
        va.CopyTo(Slot(weights, 0)); vb.CopyTo(Slot(weights, 1)); vc.CopyTo(Slot(weights, 2));
    }

    private static void SetEdgeWeights(int first, int second,
        Signed832 firstWeight, Signed832 secondWeight, Span<ulong> weights)
    {
        Import(firstWeight, Slot(weights, first)); Import(secondWeight, Slot(weights, second));
    }

    private static Vector3d RoundTriangle(in TriangleCircularGeometry geometry, FixedTriangle triangle, ReadOnlySpan<ulong> weights,
        ReadOnlySpan<ulong> denominator) => new(
        RoundComponent(geometry, triangle, 0, weights, denominator),
        RoundComponent(geometry, triangle, 1, weights, denominator),
        RoundComponent(geometry, triangle, 2, weights, denominator));

    private static Fixed64 RoundComponent(in TriangleCircularGeometry geometry, FixedTriangle triangle, int component,
        ReadOnlySpan<ulong> weights, ReadOnlySpan<ulong> denominator)
    {
        Span<ulong> sum = stackalloc ulong[Words];
        Span<ulong> value = stackalloc ulong[Words];
        Span<ulong> product = stackalloc ulong[Words];
        sum.Clear(); int sign = 0;
        for (int vertex = 0; vertex < 3; vertex++)
        {
            Signed576 coordinate = geometry.WitnessCoordinate(triangle, vertex, component);
            Import(coordinate, value);
            WideArithmetic.MultiplyMagnitudes(value, Slot(weights, vertex), product);
            Add(product, coordinate.Sign, sum, ref sign);
        }
        Span<ulong> scaledDenominator = stackalloc ulong[geometry.IsAffine ? Words : 0];
        scoped ReadOnlySpan<ulong> witnessDenominator = denominator;
        if (geometry.IsAffine)
        {
            Span<ulong> scale = stackalloc ulong[5];
            WideArithmetic.GetMagnitude(geometry.RawScale, out scale[4], out scale[3], out scale[2], out scale[1], out scale[0]);
            WideArithmetic.MultiplyMagnitudes(denominator, scale, scaledDenominator);
            witnessDenominator = scaledDenominator;
        }
        bool represented = Fixed64.TryGetSignedRawRatio(sum, witnessDenominator, sign < 0, out Fixed64 result);
        System.Diagnostics.Debug.Assert(represented); // A convex combination of representable authored vertices.
        return result;
    }

    private static void SumWeights(ReadOnlySpan<ulong> weights, Span<ulong> denominator)
    {
        WideArithmetic.AddEqualMagnitudes(Slot(weights, 0), Slot(weights, 1), denominator);
        WideArithmetic.AddMagnitudeInto(Slot(weights, 2), denominator);
    }

    private static void SumCoordinates(in TriangleCircularGeometry geometry, ReadOnlySpan<ulong> weights,
        int component, Span<ulong> result, out int sign)
    {
        Span<ulong> coordinate = stackalloc ulong[Words];
        Span<ulong> product = stackalloc ulong[Words];
        result.Clear(); sign = 0;
        for (int vertex = 0; vertex < 3; vertex++)
        {
            Signed576 value = Component(geometry.Vertex(vertex), component);
            Import(value, coordinate);
            WideArithmetic.MultiplyMagnitudes(coordinate, Slot(weights, vertex), product);
            Add(product, value.Sign, result, ref sign);
        }
    }
}
