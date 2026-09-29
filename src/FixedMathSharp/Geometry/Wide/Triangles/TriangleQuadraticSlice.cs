//=======================================================================
// MIT License, Copyright (c) 2024-present David Oravsky (mrdav30)
// See LICENSE file in the project root for full license information.
//=======================================================================
using System;
using static FixedMathSharp.Geometry.CylinderContactAlgebra;
using static FixedMathSharp.Geometry.ContactQuadratic;

namespace FixedMathSharp.Geometry;

/// <summary>Exact triangle-feature plane slicing and closed quadratic interval admission.</summary>
internal static class TriangleQuadraticSlice
{
    /// <summary>
    /// Slices the selected feature by one exact plane, then admits its free
    /// interval coordinate. Quadratic weights retain A+B*sqrt(K) throughout;
    /// no intersection parameter or support point rounds before projection.
    /// </summary>
    internal static bool TryGetWeights(int mask, Span<ulong> distances, Span<int> distanceSigns,
        Span<ulong> projections, Span<int> projectionSigns, ContactQuadratic bound, ReadOnlySpan<ulong> root,
        Span<ulong> weights, Span<int> weightSigns, out bool centered)
    {
        centered = false;
        Span<int> planeSigns = stackalloc int[3];
        Span<ulong> candidates = stackalloc ulong[12 * Words];
        Span<int> candidateSigns = stackalloc int[12];
        candidates.Clear(); candidateSigns.Clear();
        int count = 0;
        for (int vertex = 0; vertex < 3; vertex++)
        {
            planeSigns[vertex] = At(distances, distanceSigns, vertex).Sign(root);
            if ((mask & (1 << vertex)) != 0 && planeSigns[vertex] == 0)
                At(candidates, candidateSigns, 3 * count++ + vertex).Set(Signed576.One);
        }
        for (int edge = 0; edge < 3; edge++)
        {
            int next = (edge + 1) % 3;
            if ((mask & (1 << edge)) == 0 || (mask & (1 << next)) == 0
                || planeSigns[edge] * planeSigns[next] >= 0)
                continue;
            At(distances, distanceSigns, next).CopyTo(At(candidates, candidateSigns, 3 * count + edge), planeSigns[next]);
            At(distances, distanceSigns, edge).CopyTo(At(candidates, candidateSigns, 3 * count + next), planeSigns[edge]);
            count++;
        }
        System.Diagnostics.Debug.Assert(count <= 2);
        Span<ulong> values = stackalloc ulong[12 * Words];
        Span<int> valueSigns = stackalloc int[12];
        ContactQuadratic denominator = At(values, valueSigns, 0), limit = At(values, valueSigns, 1);
        ContactQuadratic comparison = At(values, valueSigns, 2), first = At(values, valueSigns, 4), second = At(values, valueSigns, 5);
        Span<int> endpointSigns = stackalloc int[2];
        for (int index = 0; index < count; index++)
        {
            Span<ulong> candidate = candidates.Slice(index * 6 * Words, 6 * Words);
            Span<int> signs = candidateSigns.Slice(index * 6, 6);
            SumWeights(candidate, signs, denominator);
            ContactQuadratic projection = At(values, valueSigns, 4 + index);
            SumWeighted(projections, projectionSigns, candidate, signs, root, projection);
            endpointSigns[index] = projection.Sign(root);
            Multiply(denominator, bound, root, limit);
            projection.CopyTo(comparison); comparison.Add(limit, -1);
            if (comparison.Sign(root) > 0)
                continue;
            projection.CopyTo(comparison); comparison.Add(limit);
            if (comparison.Sign(root) < 0)
                continue;
            candidate.CopyTo(weights); signs.CopyTo(weightSigns);
            return true;
        }
        if (count != 2 || endpointSigns[0] * endpointSigns[1] >= 0)
            return false;
        // Neither endpoint is admitted, but they straddle the interval. A
        // direct blend at coordinate zero keeps one quadratic field and
        // avoids nested rounded ratios (or a growing clipping polygon).
        centered = true;
        ContactQuadratic product = At(values, valueSigns, 3);
        for (int vertex = 0; vertex < 3; vertex++)
        {
            ContactQuadratic result = At(weights, weightSigns, vertex);
            Multiply(At(candidates, candidateSigns, vertex), second, root, result);
            result.MultiplySign(endpointSigns[1]);
            Multiply(At(candidates, candidateSigns, vertex + 3), first, root, product);
            result.Add(product, endpointSigns[0]);
        }
        return true;
    }

    internal static void SumWeights(Span<ulong> weights, Span<int> signs, ContactQuadratic result)
    {
        result.Clear();
        for (int vertex = 0; vertex < 3; vertex++)
            result.Add(At(weights, signs, vertex));
    }

    internal static void SumWeighted(Span<ulong> coordinates, Span<int> coordinateSigns,
        Span<ulong> weights, Span<int> weightSigns, ReadOnlySpan<ulong> root, ContactQuadratic result)
    {
        Span<ulong> work = stackalloc ulong[2 * Words];
        Span<int> signs = stackalloc int[2];
        ContactQuadratic product = At(work, signs, 0);
        result.Clear();
        for (int vertex = 0; vertex < 3; vertex++)
        {
            Multiply(At(coordinates, coordinateSigns, vertex), At(weights, weightSigns, vertex), root, product);
            result.Add(product);
        }
    }
    internal static Fixed64 RoundTriangleCoordinate(Fixed64 a, Fixed64 b, Fixed64 c,
        Span<ulong> weights, Span<int> signs, ContactQuadratic denominator, ReadOnlySpan<ulong> root)
    {
        Span<Signed576> coordinates = stackalloc Signed576[3]
        {
            Signed576.ExtendValue(Signed320.ExtendValue(Signed192.Raw(a))),
            Signed576.ExtendValue(Signed320.ExtendValue(Signed192.Raw(b))),
            Signed576.ExtendValue(Signed320.ExtendValue(Signed192.Raw(c)))
        };
        Span<ulong> values = stackalloc ulong[2 * Words];
        Span<int> valueSigns = stackalloc int[2];
        ContactQuadratic numerator = At(values, valueSigns, 0);
        Sum(coordinates, weights, signs, numerator);
        return RoundRatio(numerator, denominator, root,
            Math.Min(a.m_rawValue, Math.Min(b.m_rawValue, c.m_rawValue)),
            Math.Max(a.m_rawValue, Math.Max(b.m_rawValue, c.m_rawValue)));
    }

    internal static void Sum(ReadOnlySpan<Signed576> coordinates, Span<ulong> weights, Span<int> signs, ContactQuadratic result)
    {
        Span<ulong> values = stackalloc ulong[2 * Words];
        Span<int> productSigns = stackalloc int[2];
        ContactQuadratic product = At(values, productSigns, 0);
        result.Clear();
        for (int vertex = 0; vertex < 3; vertex++)
        {
            Scale(At(weights, signs, vertex), coordinates[vertex], product);
            result.Add(product);
        }
    }

}
