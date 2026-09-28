//=======================================================================
// MIT License, Copyright (c) 2024-present David Oravsky (mrdav30)
// See LICENSE file in the project root for full license information.
//=======================================================================
using System;

namespace FixedMathSharp.Geometry;

/// <summary>The shared stationary quartic on a cylinder's edge-normal chart.</summary>
internal static class CylinderEdgeContactPolynomial
{
    internal const int Words = 40;
    internal const int Slots = 26;

    /// <summary>
    /// For g=(P+sqrt(Q))/(scale*sqrt(M)), writes W=L²-K²Q,
    /// Q,K,L,J=PK-L,N=J²,D=scale²K²M into slots 0,5,8,10,13,16,21.
    /// K=2P'M-PM' and L=Q'M-QM'. M is a positive quadratic metric.
    /// Root admission must retain L=-K sqrt(Q), cap and polytope cones.
    /// All buffers are disjoint, and each coefficient must fit forty words.
    /// </summary>
    internal static void Build(ReadOnlySpan<ulong> p, ReadOnlySpan<sbyte> ps,
        ReadOnlySpan<ulong> q, ReadOnlySpan<sbyte> qs,
        ReadOnlySpan<ulong> metric, ReadOnlySpan<sbyte> ms,
        ReadOnlySpan<ulong> scale, Span<ulong> data, Span<sbyte> signs)
    {
        data.Clear(); signs.Clear();
        q.CopyTo(data.Slice(5 * Words, 3 * Words)); qs.CopyTo(signs.Slice(5, 3));
        Span<ulong> scratch = stackalloc ulong[9 * Words];
        Span<sbyte> scratchSigns = stackalloc sbyte[8];
        scratch.Clear(); scratchSigns.Clear();
        Span<ulong> product = scratch.Slice(8 * Words, Words);
        Span<ulong> k = data.Slice(8 * Words, 2 * Words), l = data.Slice(10 * Words, 3 * Words);
        Span<sbyte> ks = signs.Slice(8, 2), ls = signs.Slice(10, 3);
        Term(p, ps, 1, metric, ms, 0, k, ks, 0, 2, product);
        Term(p, ps, 0, metric, ms, 1, k, ks, 0, -1, product);
        Term(p, ps, 1, metric, ms, 1, k, ks, 1, 1, product);
        Term(p, ps, 0, metric, ms, 2, k, ks, 1, -2, product);
        Term(q, qs, 1, metric, ms, 0, l, ls, 0, 1, product);
        Term(q, qs, 0, metric, ms, 1, l, ls, 0, -1, product);
        Term(q, qs, 2, metric, ms, 0, l, ls, 1, 2, product);
        Term(q, qs, 0, metric, ms, 2, l, ls, 1, -2, product);
        Term(q, qs, 2, metric, ms, 1, l, ls, 2, 1, product);
        Term(q, qs, 1, metric, ms, 2, l, ls, 2, -1, product);
        Span<ulong> kSquared = scratch[..(3 * Words)], temporary = scratch.Slice(3 * Words, 5 * Words);
        Span<sbyte> kSquaredSigns = scratchSigns[..3], temporarySigns = scratchSigns.Slice(3, 5);
        Multiply(l, ls, l, ls, data[..(5 * Words)], signs[..5], product);
        Multiply(k, ks, k, ks, kSquared, kSquaredSigns, product);
        Multiply(kSquared, kSquaredSigns, q, qs, temporary, temporarySigns, product);
        Add(temporary, temporarySigns, data[..(5 * Words)], signs[..5], -1);
        Span<ulong> j = data.Slice(13 * Words, 3 * Words);
        Span<sbyte> js = signs.Slice(13, 3);
        Multiply(p, ps, k, ks, j, js, product);
        Add(l, ls, j, js, -1);
        Multiply(j, js, j, js, data.Slice(16 * Words, 5 * Words), signs.Slice(16, 5), product);
        Multiply(kSquared, kSquaredSigns, metric, ms, temporary, temporarySigns, product);
        WideArithmetic.MultiplyMagnitudes(scale, scale, kSquared[..Words]);
        kSquaredSigns[0] = 1;
        Multiply(temporary, temporarySigns, kSquared[..Words], kSquaredSigns[..1],
            data.Slice(21 * Words, 5 * Words), signs.Slice(21, 5), product);
        WideFiniteAxisIntersection.NormalizeFiniteAxisPolynomialPowerOfTwo(data[..(5 * Words)], signs[..5]);
        WideFiniteAxisIntersection.NormalizeFiniteAxisPolynomialPowerOfTwo(data.Slice(16 * Words, 10 * Words), signs.Slice(16, 10));
    }

    private static void Term(ReadOnlySpan<ulong> a, ReadOnlySpan<sbyte> aSigns, int ai,
        ReadOnlySpan<ulong> b, ReadOnlySpan<sbyte> bSigns, int bi,
        Span<ulong> result, Span<sbyte> resultSigns, int ri, int multiplier, Span<ulong> product)
    {
        WideArithmetic.MultiplyMagnitudes(a.Slice(ai * Words, Words), b.Slice(bi * Words, Words), product);
        int sign = resultSigns[ri];
        WideArithmetic.AddShiftedSignedMagnitude(product,
            aSigns[ai] * bSigns[bi] * Math.Sign(multiplier), Math.Abs(multiplier) == 2 ? 1 : 0,
            result.Slice(ri * Words, Words), ref sign);
        resultSigns[ri] = (sbyte)sign;
    }
    private static void Multiply(ReadOnlySpan<ulong> a, ReadOnlySpan<sbyte> aSigns,
        ReadOnlySpan<ulong> b, ReadOnlySpan<sbyte> bSigns, Span<ulong> result,
        Span<sbyte> resultSigns, Span<ulong> product) =>
        WideFiniteAxisIntersection.MultiplyFiniteAxisPolynomials(a, aSigns, b, bSigns, result, resultSigns, product);
    private static void Add(ReadOnlySpan<ulong> source, ReadOnlySpan<sbyte> sourceSigns,
        Span<ulong> result, Span<sbyte> resultSigns, int multiplier) =>
        WideFiniteAxisIntersection.AddFiniteAxisPolynomial(source, sourceSigns, result, resultSigns, multiplier);
}
