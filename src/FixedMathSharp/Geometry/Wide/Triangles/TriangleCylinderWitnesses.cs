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
    /// Geometry coordinates are below 198 bits and analytic directions below
    /// 598. Tangent weights are below 798 bits, axial numerators below 998,
    /// and blended weights below 1798. Their coordinate products remain below
    /// 2000 bits, inside the existing forty-word candidate arithmetic storage.
    /// </summary>
    internal static Vector3d GetSidePoint(in TriangleCylinderGeometry geometry, FixedTriangle triangle,
        int mask, ReadOnlySpan<ulong> normal, ReadOnlySpan<int> normalSigns,
        out Fixed64 axial, out int cap)
    {
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
        System.Diagnostics.Debug.Assert(count > 0 && count <= 2);
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
            System.Diagnostics.Debug.Assert(count == 2 && axialSigns[0] * axialSigns[1] < 0);
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
        Import(Signed320.ExtendValue(geometry.RawScale), half);
        Span<ulong> scaledDenominator = stackalloc ulong[Words];
        WideArithmetic.MultiplyMagnitudes(denominator, half, scaledDenominator);
        bool represented = Fixed64.TryGetSignedRawRatio(bound, scaledDenominator, axialSign < 0, out axial);
        System.Diagnostics.Debug.Assert(represented);
        return RoundTriangle(triangle, weights, denominator);
    }

    /// <summary>
    /// Projects the origin onto the selected cap-pole feature, retaining its exact barycentric
    /// weights until both authored and cylinder-local coordinates round.
    /// A cap-pole minimum proves the projected radial point is in the disk;
    /// its final-rounded coordinates must not be normalized a second time.
    /// </summary>
    internal static Vector3d GetCapPoint(in TriangleCylinderGeometry geometry, FixedTriangle triangle,
        int mask, out Vector3d projectedRadial)
    {
        Span<ulong> weights = stackalloc ulong[3 * Words];
        Span<ulong> denominator = stackalloc ulong[Words];
        weights.Clear();
        if (mask == 7)
            GetFaceWeights(geometry, weights);
        else if ((mask & (mask - 1)) == 0)
            weights[((mask & 1) != 0 ? 0 : (mask & 2) != 0 ? 1 : 2) * Words] = 1;
        else
            GetEdgeWeights(geometry, mask, weights);
        SumWeights(weights, denominator);
        Vector3d point = RoundTriangle(triangle, weights, denominator);
        Span<ulong> scale = stackalloc ulong[Words];
        Span<ulong> scaledDenominator = stackalloc ulong[Words];
        Span<ulong> coordinate = stackalloc ulong[Words];
        Import(Signed320.ExtendValue(geometry.RawScale), scale);
        WideArithmetic.MultiplyMagnitudes(denominator, scale, scaledDenominator);
        SumCoordinates(geometry, weights, 0, coordinate, out int sign);
        bool representedX = Fixed64.TryGetSignedRawRatio(coordinate, scaledDenominator, sign < 0, out Fixed64 x);
        SumCoordinates(geometry, weights, 2, coordinate, out sign);
        bool representedZ = Fixed64.TryGetSignedRawRatio(coordinate, scaledDenominator, sign < 0, out Fixed64 z);
        System.Diagnostics.Debug.Assert(representedX && representedZ); // Both bounded by the authored radius.
        projectedRadial = new Vector3d(x, Fixed64.Zero, z);
        return point;
    }

    private static void GetEdgeWeights(in TriangleCylinderGeometry geometry, int mask, Span<ulong> weights)
    {
        int first = (mask & 1) != 0 ? 0 : 1;
        int second = (mask & 4) != 0 ? 2 : 1;
        WideAxis3 edge = TriangleCylinderGeometry.Subtract(geometry.Vertex(second), geometry.Vertex(first));
        // A pole's selected feature has constant local Y. Projecting the
        // origin is identical to projecting either cap center onto it.
        Signed576 parameter = WideAxis3.Dot(TriangleCylinderGeometry.Subtract(default, geometry.Vertex(first)), edge);
        Signed576 denominatorValue = edge.SquaredLength;
        if (parameter.Sign <= 0)
        {
            weights[first * Words] = 1;
            return;
        }
        Signed576 remainder = WideArithmetic.SubtractSigned576(denominatorValue, parameter);
        if (remainder.Sign <= 0)
        {
            weights[second * Words] = 1;
            return;
        }
        Import(remainder, Slot(weights, first)); Import(parameter, Slot(weights, second));
    }

    private static void GetFaceWeights(in TriangleCylinderGeometry geometry, Span<ulong> weights)
    {
        WideAxis3 e = TriangleCylinderGeometry.Subtract(geometry.B, geometry.A);
        WideAxis3 f = TriangleCylinderGeometry.Subtract(geometry.C, geometry.A);
        WideAxis3 offset = TriangleCylinderGeometry.Subtract(default, geometry.A);
        Signed576 d1 = WideAxis3.Dot(e, offset), d2 = WideAxis3.Dot(f, offset);
        if (d1.Sign <= 0 && d2.Sign <= 0)
        {
            weights[0] = 1;
            return;
        }
        Signed576 ee = e.SquaredLength, ff = f.SquaredLength, ef = WideAxis3.Dot(e, f);
        Signed576 d3 = WideArithmetic.SubtractSigned576(d1, ee), d4 = WideArithmetic.SubtractSigned576(d2, ef);
        if (d3.Sign >= 0 && WideArithmetic.SubtractSigned576(d4, d3).Sign <= 0)
        {
            weights[Words] = 1;
            return;
        }
        // Dot products use fewer than 400 bits; the face-region minors use
        // fewer than 802, within Signed832. Weighted source coordinates stay
        // in the shared magnitude arena until their single final rounding.
        Signed832 vc = Cross(d1, d4, d3, d2);
        if (vc.Sign <= 0 && d1.Sign >= 0 && d3.Sign <= 0)
        {
            SetEdgeWeights(0, 1, WideArithmetic.SubtractSigned576(default, d3), d1, weights);
            return;
        }
        Signed576 d5 = WideArithmetic.SubtractSigned576(d1, ef), d6 = WideArithmetic.SubtractSigned576(d2, ff);
        if (d6.Sign >= 0 && WideArithmetic.SubtractSigned576(d5, d6).Sign <= 0)
        {
            weights[2 * Words] = 1;
            return;
        }
        Signed832 vb = Cross(d5, d2, d1, d6);
        // Once A/B/AB are excluded, vb<=0 proves d2>=0: the opposite
        // signs would force the earlier AB or B Voronoi region (Gram det>0).
        if (vb.Sign <= 0 && d6.Sign <= 0)
        {
            SetEdgeWeights(0, 2, WideArithmetic.SubtractSigned576(default, d6), d2, weights);
            return;
        }
        Signed832 va = Cross(d3, d6, d5, d4);
        Signed576 first = WideArithmetic.SubtractSigned576(d4, d3), second = WideArithmetic.SubtractSigned576(d5, d6);
        // The preceding vertex and edge regions already prove both BC
        // weights nonnegative whenever va<=0 for this nondegenerate face.
        if (va.Sign <= 0)
        {
            SetEdgeWeights(1, 2, second, first, weights);
            return;
        }
        WideArithmetic.GetMagnitude(va, weights[..13]);
        WideArithmetic.GetMagnitude(vb, weights.Slice(Words, 13));
        WideArithmetic.GetMagnitude(vc, weights.Slice(2 * Words, 13));
    }

    private static void SetEdgeWeights(int first, int second,
        Signed576 firstWeight, Signed576 secondWeight, Span<ulong> weights)
    {
        Import(firstWeight, Slot(weights, first)); Import(secondWeight, Slot(weights, second));
    }

    private static Signed832 Cross(Signed576 a, Signed576 b, Signed576 c, Signed576 d) =>
        WideArithmetic.SubtractSigned832(WideArithmetic.MultiplySigned576ToSigned832(a, b),
            WideArithmetic.MultiplySigned576ToSigned832(c, d));

    private static Vector3d RoundTriangle(FixedTriangle triangle, ReadOnlySpan<ulong> weights,
        ReadOnlySpan<ulong> denominator) => new(
        RoundComponent(triangle.A.X, triangle.B.X, triangle.C.X, weights, denominator),
        RoundComponent(triangle.A.Y, triangle.B.Y, triangle.C.Y, weights, denominator),
        RoundComponent(triangle.A.Z, triangle.B.Z, triangle.C.Z, weights, denominator));

    private static Fixed64 RoundComponent(Fixed64 a, Fixed64 b, Fixed64 c,
        ReadOnlySpan<ulong> weights, ReadOnlySpan<ulong> denominator)
    {
        Span<ulong> sum = stackalloc ulong[Words];
        Span<ulong> value = stackalloc ulong[Words];
        Span<ulong> product = stackalloc ulong[Words];
        sum.Clear(); int sign = 0;
        for (int vertex = 0; vertex < 3; vertex++)
        {
            Signed320 coordinate = Signed320.ExtendValue(Signed192.Raw(vertex == 0 ? a : vertex == 1 ? b : c));
            Import(coordinate, value);
            WideArithmetic.MultiplyMagnitudes(value, Slot(weights, vertex), product);
            Add(product, coordinate.Sign, sum, ref sign);
        }
        bool represented = Fixed64.TryGetSignedRawRatio(sum, denominator, sign < 0, out Fixed64 result);
        System.Diagnostics.Debug.Assert(represented); // A convex combination of representable authored vertices.
        return result;
    }

    private static void SumWeights(ReadOnlySpan<ulong> weights, Span<ulong> denominator)
    {
        WideArithmetic.AddEqualMagnitudes(Slot(weights, 0), Slot(weights, 1), denominator);
        WideArithmetic.AddMagnitudeInto(Slot(weights, 2), denominator);
    }

    private static void SumCoordinates(in TriangleCylinderGeometry geometry, ReadOnlySpan<ulong> weights,
        int component, Span<ulong> result, out int sign)
    {
        Span<ulong> coordinate = stackalloc ulong[Words];
        Span<ulong> product = stackalloc ulong[Words];
        result.Clear(); sign = 0;
        for (int vertex = 0; vertex < 3; vertex++)
        {
            Signed320 value = Component(geometry.Vertex(vertex), component);
            Import(value, coordinate);
            WideArithmetic.MultiplyMagnitudes(coordinate, Slot(weights, vertex), product);
            Add(product, value.Sign, result, ref sign);
        }
    }
}
