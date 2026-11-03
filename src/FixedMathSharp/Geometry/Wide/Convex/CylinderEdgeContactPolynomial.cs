//=======================================================================
// CylinderEdgeContactPolynomial.cs
//=======================================================================
// MIT License, Copyright (c) 2024-present David Oravsky (mrdav30)
// See LICENSE file in the project root for full license information.
//=======================================================================

using System;

namespace FixedMathSharp.Geometry;

/// <summary>The shared stationary quartic on a cylinder's edge-normal chart.</summary>
internal static class CylinderEdgeContactPolynomial
{
    internal const int Slots = 26;

    /// <summary>
    /// For g=(P+sqrt(Q))/(scale*sqrt(M)), writes W=L²-K²Q,
    /// Q,K,L,J=PK-L,N=J²,D=scale²K²M into slots 0,5,8,10,13,16,21.
    /// K=2P'M-PM' and L=Q'M-QM'. M is a positive quadratic metric.
    /// Root admission must retain L=-K sqrt(Q), cap and polytope cones.
    /// All buffers are disjoint, and each coefficient must fit its supplied word stride.
    /// </summary>
    internal static void Build(ReadOnlySpan<ulong> p, ReadOnlySpan<sbyte> ps,
        ReadOnlySpan<ulong> q, ReadOnlySpan<sbyte> qs,
        ReadOnlySpan<ulong> metric, ReadOnlySpan<sbyte> ms,
        ReadOnlySpan<ulong> scale, Span<ulong> data, Span<sbyte> signs)
    {
        BuildStationary(p, ps, q, qs, metric, ms, scale, data, signs);
        CompleteGap(data, signs);
    }

    /// <summary>
    /// Builds W,Q,K,L for root isolation/admission. Slots 13..14 borrow P,
    /// 16..18 borrow M, 19 borrows scale, and 21..23 borrow K squared until
    /// completion replaces 13..25 by J,N,D. Call CompleteGap at most once.
    /// </summary>
    internal static void BuildStationary(ReadOnlySpan<ulong> p, ReadOnlySpan<sbyte> ps,
        ReadOnlySpan<ulong> q, ReadOnlySpan<sbyte> qs,
        ReadOnlySpan<ulong> metric, ReadOnlySpan<sbyte> ms,
        ReadOnlySpan<ulong> scale, Span<ulong> data, Span<sbyte> signs)
    {
        int words = data.Length / signs.Length;
        data.Clear(); signs.Clear();
        q.CopyTo(data.Slice(5 * words, 3 * words)); qs.CopyTo(signs.Slice(5, 3));
        Span<ulong> scratch = stackalloc ulong[9 * words];
        Span<sbyte> scratchSigns = stackalloc sbyte[8];
        scratch.Clear(); scratchSigns.Clear();
        Span<ulong> product = scratch.Slice(8 * words, words);
        Span<ulong> k = data.Slice(8 * words, 2 * words), l = data.Slice(10 * words, 3 * words);
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
        Span<ulong> kSquared = scratch[..(3 * words)], temporary = scratch.Slice(3 * words, 5 * words);
        Span<sbyte> kSquaredSigns = scratchSigns[..3], temporarySigns = scratchSigns.Slice(3, 5);
        Multiply(l, ls, l, ls, data[..(5 * words)], signs[..5], product);
        Multiply(k, ks, k, ks, kSquared, kSquaredSigns, product);
        Multiply(kSquared, kSquaredSigns, q, qs, temporary, temporarySigns, product);
        Add(temporary, temporarySigns, data[..(5 * words)], signs[..5], -1);
        p.CopyTo(data.Slice(13 * words, 2 * words));
        ps.CopyTo(signs.Slice(13, 2));
        metric.CopyTo(data.Slice(16 * words, 3 * words));
        ms.CopyTo(signs.Slice(16, 3));
        scale.CopyTo(data.Slice(19 * words, words));
        kSquared.CopyTo(data.Slice(21 * words, 3 * words));
        kSquaredSigns.CopyTo(signs.Slice(21, 3));
        WideFiniteAxisIntersection.NormalizeFiniteAxisPolynomialPowerOfTwo(data[..(5 * words)], signs[..5]);
    }

    /// <summary>
    /// Completes J=PK-L, N=J squared and D=scale squared*K squared*M once.
    /// Scratch consumes staged P before J overwrites it; D is written before
    /// N so staged M/scale can share the future N slots.
    /// </summary>
    internal static void CompleteGap(Span<ulong> data, Span<sbyte> signs)
    {
        int words = data.Length / signs.Length;
        Span<ulong> scratch = stackalloc ulong[7 * words];
        Span<sbyte> scratchSigns = stackalloc sbyte[6];
        scratch.Clear(); scratchSigns.Clear();
        Span<ulong> temporary = scratch[..(5 * words)], scaleSquared = scratch.Slice(5 * words, words);
        Span<sbyte> temporarySigns = scratchSigns[..5], scaleSquaredSigns = scratchSigns.Slice(5, 1);
        Span<ulong> product = scratch.Slice(6 * words, words);
        Multiply(data.Slice(13 * words, 2 * words), signs.Slice(13, 2),
            data.Slice(8 * words, 2 * words), signs.Slice(8, 2),
            temporary[..(3 * words)], temporarySigns[..3], product);
        Add(data.Slice(10 * words, 3 * words), signs.Slice(10, 3),
            temporary[..(3 * words)], temporarySigns[..3], -1);
        temporary[..(3 * words)].CopyTo(data.Slice(13 * words, 3 * words));
        temporarySigns[..3].CopyTo(signs.Slice(13, 3));
        // Consume staged K squared before D overwrites its slots.
        Multiply(data.Slice(21 * words, 3 * words), signs.Slice(21, 3),
            data.Slice(16 * words, 3 * words), signs.Slice(16, 3), temporary, temporarySigns, product);
        ReadOnlySpan<ulong> scale = data.Slice(19 * words, words);
        WideArithmetic.MultiplyMagnitudes(scale, scale, scaleSquared);
        scaleSquaredSigns[0] = 1;
        Multiply(temporary, temporarySigns, scaleSquared, scaleSquaredSigns,
            data.Slice(21 * words, 5 * words), signs.Slice(21, 5), product);
        // D no longer borrows M/scale; N can overwrite those slots.
        Multiply(data.Slice(13 * words, 3 * words), signs.Slice(13, 3),
            data.Slice(13 * words, 3 * words), signs.Slice(13, 3),
            data.Slice(16 * words, 5 * words), signs.Slice(16, 5), product);
        WideFiniteAxisIntersection.NormalizeFiniteAxisPolynomialPowerOfTwo(data.Slice(16 * words, 10 * words), signs.Slice(16, 10));
    }

    private static void Term(ReadOnlySpan<ulong> a, ReadOnlySpan<sbyte> aSigns, int ai,
        ReadOnlySpan<ulong> b, ReadOnlySpan<sbyte> bSigns, int bi,
        Span<ulong> result, Span<sbyte> resultSigns, int ri, int multiplier, Span<ulong> product)
    {
        int words = product.Length;
        WideArithmetic.MultiplyMagnitudes(a.Slice(ai * words, words), b.Slice(bi * words, words), product);
        int sign = resultSigns[ri];
        WideArithmetic.AddShiftedSignedMagnitude(product,
            aSigns[ai] * bSigns[bi] * Math.Sign(multiplier), Math.Abs(multiplier) == 2 ? 1 : 0,
            result.Slice(ri * words, words), ref sign);
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
