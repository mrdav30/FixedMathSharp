using System;
using System.Linq;
using System.Numerics;
using FixedMathSharp.Geometry;
using Xunit;

namespace FixedMathSharp.Tests.Geometry;

public sealed class CylinderEdgeContactPolynomialTests
{
    [Theory]
    [InlineData(4)]
    [InlineData(16)]
    [InlineData(40)]
    public void DeferredGap_PreservesTheExactStationaryAndSquaredGapEquations(int words)
    {
        // Independent integer polynomials check the shared eager caller and
        // the staged curved-prism caller, including cancellation and zero K.
        foreach ((long[] p, long[] q, long[] m, long scale) in new[]
        {
            (new long[] { 3, -5 }, new long[] { 7, 2, 9 }, new long[] { 5, -2, 3 }, 6L),
            (new long[] { -8, 0 }, new long[] { 12, 0, 4 }, new long[] { 4, 0, 4 }, 8L),
            (new long[] { 0, 0 }, new long[] { 4, 0, 4 }, new long[] { 4, 0, 4 }, 2L),
            (new long[] { 0, 7 }, new long[] { 0, 0, 0 }, new long[] { 9, 6, 2 }, 1L)
        })
        {
            BigInteger content = BigInteger.One << (words == 4 ? 32 : 70);
            BigInteger[] bp = p.Select(x => content * x).ToArray();
            BigInteger[] bq = q.Select(x => content * x).ToArray();
            BigInteger[] bm = m.Select(x => content * x).ToArray();
            BigInteger bs = content * scale;
            BigInteger[] fullK = Difference(Product(Derivative(bp), bm), Product(bp, Derivative(bm)), 2);
            BigInteger[] fullL = Difference(Product(Derivative(bq), bm), Product(bq, Derivative(bm)));
            Assert.True(fullK[2].IsZero);
            Assert.True(fullL[3].IsZero);
            BigInteger[] k = fullK[..2], l = fullL[..3];
            BigInteger[] w = Difference(Product(l, l), Product(Product(k, k), bq));
            BigInteger[] j = Difference(Product(bp, k), l);
            BigInteger[] n = Product(j, j);
            BigInteger[] d = Product(Product(k, k), bm).Select(x => x * bs * bs).ToArray();
            Normalize(w);
            BigInteger[] fraction = n.Concat(d).ToArray();
            Normalize(fraction);

            ulong[] eager = new ulong[CylinderEdgeContactPolynomial.Slots * words];
            sbyte[] eagerSigns = new sbyte[CylinderEdgeContactPolynomial.Slots];
            ulong[] staged = new ulong[eager.Length];
            sbyte[] stagedSigns = new sbyte[eagerSigns.Length];
            ulong[] pv = Encode(bp, words), qv = Encode(bq, words), mv = Encode(bm, words);
            sbyte[] ps = Signs(bp), qs = Signs(bq), ms = Signs(bm);
            ulong[] sv = Encode(new[] { bs }, words);
            CylinderEdgeContactPolynomial.Build(pv, ps, qv, qs, mv, ms, sv, eager, eagerSigns);
            CylinderEdgeContactPolynomial.BuildStationary(pv, ps, qv, qs, mv, ms, sv, staged, stagedSigns);
            ulong[] stationary = staged[..(13 * words)];
            sbyte[] stationarySigns = stagedSigns[..13];
            CylinderEdgeContactPolynomial.CompleteGap(staged, stagedSigns);

            Assert.Equal(eager, staged);
            Assert.Equal(eagerSigns, stagedSigns);
            Assert.Equal(stationary, staged[..(13 * words)]);
            Assert.Equal(stationarySigns, stagedSigns[..13]);
            BigInteger[] expected = w.Take(5).Concat(bq).Concat(k.Take(2)).Concat(l.Take(3))
                .Concat(j.Take(3)).Concat(fraction).ToArray();
            Assert.Equal(CylinderEdgeContactPolynomial.Slots, expected.Length);
            for (int i = 0; i < expected.Length; i++)
                Assert.Equal(expected[i], Decode(staged.AsSpan(i * words, words), stagedSigns[i]));
        }
    }

    private static BigInteger[] Derivative(BigInteger[] p) =>
        p.Skip(1).Select((x, i) => x * (i + 1)).ToArray();

    private static BigInteger[] Product(BigInteger[] a, BigInteger[] b)
    {
        BigInteger[] result = new BigInteger[a.Length + b.Length - 1];
        for (int i = 0; i < a.Length; i++)
            for (int j = 0; j < b.Length; j++) result[i + j] += a[i] * b[j];
        return result;
    }

    private static BigInteger[] Difference(BigInteger[] a, BigInteger[] b, int firstScale = 1)
    {
        BigInteger[] result = new BigInteger[Math.Max(a.Length, b.Length)];
        for (int i = 0; i < a.Length; i++) result[i] += firstScale * a[i];
        for (int i = 0; i < b.Length; i++) result[i] -= b[i];
        return result;
    }

    private static void Normalize(BigInteger[] values)
    {
        if (values.All(x => x.IsZero)) return;
        while (values.All(x => x.IsEven))
            for (int i = 0; i < values.Length; i++) values[i] >>= 1;
    }

    private static sbyte[] Signs(BigInteger[] values) => values.Select(x => (sbyte)x.Sign).ToArray();

    private static ulong[] Encode(BigInteger[] values, int words)
    {
        ulong[] result = new ulong[values.Length * words];
        for (int i = 0; i < values.Length; i++)
        {
            BigInteger value = BigInteger.Abs(values[i]);
            for (int word = 0; word < words; word++, value >>= 64)
                result[i * words + word] = (ulong)(value & ulong.MaxValue);
            Assert.True(value.IsZero);
        }
        return result;
    }

    private static BigInteger Decode(ReadOnlySpan<ulong> value, int sign)
    {
        BigInteger magnitude = BigInteger.Zero;
        for (int word = value.Length - 1; word >= 0; word--) magnitude = (magnitude << 64) | value[word];
        return sign * magnitude;
    }
}
