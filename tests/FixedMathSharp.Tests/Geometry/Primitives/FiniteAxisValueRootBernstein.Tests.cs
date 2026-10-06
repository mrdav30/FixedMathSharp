using System;
using System.Numerics;
using FixedMathSharp.Geometry;
using Xunit;

namespace FixedMathSharp.Tests;

public sealed class FiniteAxisValueRootBernsteinTests
{
    [Theory]
    [InlineData(1, 63)]
    [InlineData(-1, 63)]
    [InlineData(1, 64)]
    [InlineData(-1, 64)]
    public void SturmClusterAtCompactLimit_PreservesCanonicalDyadicsAndFailureMetadata(int sign, int bits)
    {
        BigInteger scale = BigInteger.One << bits;
        // The early quarter root completes before the deep cluster. The half
        // root is repeated and is recognized only after separating its neighbor.
        // One-word cells support q<=63: the next bit must fail without a partial mask.
        BigInteger[] polynomial = { sign * 65537 };
        foreach (var factor in new[] { (-(BigInteger)1, (BigInteger)4),
            (1 - scale / 2, scale), (-(BigInteger)1, (BigInteger)2), (-(BigInteger)1, (BigInteger)2) })
        {
            var product = new BigInteger[polynomial.Length + 1];
            for (int index = 0; index < polynomial.Length; index++)
            {
                product[index] += factor.Item1 * polynomial[index];
                product[index + 1] += factor.Item2 * polynomial[index];
            }
            polynomial = product;
        }
        Encode(polynomial, out ulong[] coefficients, out sbyte[] signs);
        Span<ulong> cells = stackalloc ulong[8];
        Span<int> shifts = stackalloc int[8];
        cells.Fill(ulong.MaxValue);
        shifts.Fill(-1);
        Assert.Equal(bits == 63, WideFiniteAxisIntersection.TryGetFiniteValueRoots(coefficients, signs,
            cells, shifts, out int count, out int rationalMask, out bool repeated));
        Assert.Equal(3, count);
        Assert.True(repeated);
        Assert.Equal(bits == 63 ? 7 : 0, rationalMask);
        if (bits == 63)
        {
            Assert.Equal(new[] { 2, 63, 1 }, shifts[..3].ToArray());
            Assert.Equal(new[] { 1UL, (1UL << 62) - 1, 1UL }, cells[..3].ToArray());
        }
        else
        {
            // The shared multi-root node exhausted its compact cell, but its
            // authoritative count still materializes every root independently.
            FiniteAxisValueRoots roots = WideFiniteAxisIntersection.GetFiniteValueRoots(coefficients, signs, cells, shifts);
            Assert.False(roots.HasCompactCells);
            Assert.Equal(3, roots.Count);
            Assert.True(roots.HasRepeatedRoots);
            var expected = new[] { Rational(1, 4), (scale / 2 - 1, scale), Rational(1, 2) };
            var fullCell = new ulong[WideFiniteAxisIntersection.GetFiniteValueRootCellWords(coefficients, signs)];
            for (int ordinal = 0; ordinal < roots.Count; ordinal++)
            {
                fullCell.AsSpan().Fill(ulong.MaxValue);
                FiniteAxisValueRoot root = FiniteAxisValueRoots.GetRoot(roots, ordinal, coefficients, signs, fullCell);
                Assert.Equal(ordinal, root.Ordinal);
                Assert.True(root.IsRational);
                Assert.Equal(expected[ordinal].Item1 << root.DenominatorShift,
                    Decode(fullCell) * expected[ordinal].Item2);
            }
        }
    }

    [Theory]
    [InlineData(1, false)]
    [InlineData(-1, false)]
    [InlineData(1, true)]
    [InlineData(-1, true)]
    public void SturmSingletonAtCompactLimit_PreservesCountAndFallbackAfterEndpointCleanupFails(int sign, bool earlierRoot)
    {
        BigInteger scale = BigInteger.One << 100;
        BigInteger numerator = earlierRoot ? scale / 4 + 1 : BigInteger.One;
        // A singleton near zero cannot obtain a positive lower endpoint in
        // q<=63. With an earlier quarter root, subdivision instead seeds the
        // singleton immediately above that excluded root; its lower endpoint
        // stays a root until q=100. Both must fail in singleton isolation, not
        // in shared multi-root subdivision. The repeated root at two keeps
        // global repetition meaningful even though all physical roots are simple.
        BigInteger[] polynomial = { -sign * 65537 * numerator, sign * 65537 * scale };
        var factors = earlierRoot ? new[] { (-1, 4), (-2, 1), (-2, 1) } : new[] { (-2, 1), (-2, 1) };
        foreach (var factor in factors)
        {
            var product = new BigInteger[polynomial.Length + 1];
            for (int index = 0; index < polynomial.Length; index++)
            {
                product[index] += factor.Item1 * polynomial[index];
                product[index + 1] += factor.Item2 * polynomial[index];
            }
            polynomial = product;
        }
        Encode(polynomial, out ulong[] coefficients, out sbyte[] signs);
        ulong[] original = (ulong[])coefficients.Clone();
        sbyte[] originalSigns = (sbyte[])signs.Clone();
        Span<ulong> cells = stackalloc ulong[8];
        Span<int> shifts = stackalloc int[8];
        cells.Fill(ulong.MaxValue);
        shifts.Fill(-1);
        Assert.False(WideFiniteAxisIntersection.TryGetFiniteValueRoots(coefficients, signs,
            cells, shifts, out int count, out int rationalMask, out bool repeated));
        Assert.Equal(earlierRoot ? 2 : 1, count);
        Assert.True(repeated);
        Assert.Equal(0, rationalMask);
        if (earlierRoot)
        {
            Assert.Equal(1UL, cells[0]);
            Assert.Equal(2, shifts[0]);
        }

        FiniteAxisValueRoots roots = WideFiniteAxisIntersection.GetFiniteValueRoots(coefficients, signs, cells, shifts);
        Assert.False(roots.HasCompactCells);
        Assert.Equal(count, roots.Count);
        Assert.True(roots.HasRepeatedRoots);
        var expected = earlierRoot ? new[] { Rational(1, 4), (numerator, scale) } : new[] { (numerator, scale) };
        var fullCell = new ulong[WideFiniteAxisIntersection.GetFiniteValueRootCellWords(coefficients, signs)];
        for (int ordinal = 0; ordinal < roots.Count; ordinal++)
        {
            fullCell.AsSpan().Fill(ulong.MaxValue);
            FiniteAxisValueRoot root = FiniteAxisValueRoots.GetRoot(roots, ordinal, coefficients, signs, fullCell);
            Assert.Equal(ordinal, root.Ordinal);
            Assert.True(root.IsRational);
            Assert.Equal(earlierRoot && ordinal == 0 ? 2 : 100, root.DenominatorShift);
            Assert.Equal(expected[ordinal].Item1 << root.DenominatorShift,
                Decode(fullCell) * expected[ordinal].Item2);
        }
        Assert.Equal(original, coefficients);
        Assert.Equal(originalSigns, signs);
    }

    [Theory]
    [InlineData(1, 0)]
    [InlineData(-1, 0)]
    [InlineData(1, 1)]
    [InlineData(-1, 1)]
    [InlineData(1, 2)]
    [InlineData(-1, 2)]
    public void SturmBatchCells_MatchIndependentOrdinalIsolation(int sign, int geometry)
    {
        // Force Sturm, then compare batch cells with the single-ordinal owner.
        // Include repeated midpoints, excluded zero and the closed endpoint one.
        var factors = geometry == 0
            ? new[] { (-1, 8), (-1, 4), (-1, 2), (-3, 4), (-1, 1) }
            : geometry == 1
                ? new[] { (0, 1), (0, 1), (-1, 4), (-1, 4), (-1, 2), (-1, 2), (-3, 4), (-1, 1) }
                : new[] { (0, 1), (-1, 3), (-2, 3), (-1, 1) };
        BigInteger[] polynomial = { sign * 65537 };
        foreach (var factor in factors)
        {
            var product = new BigInteger[polynomial.Length + 1];
            for (int index = 0; index < polynomial.Length; index++)
            {
                product[index] += factor.Item1 * polynomial[index];
                product[index + 1] += factor.Item2 * polynomial[index];
            }
            polynomial = product;
        }
        Encode(polynomial, out ulong[] coefficients, out sbyte[] signs);
        Span<ulong> cells = stackalloc ulong[64];
        Span<int> shifts = stackalloc int[8];
        Assert.True(WideFiniteAxisIntersection.TryGetFiniteValueRoots(coefficients, signs,
            cells, shifts, out int count, out int rationalMask, out bool repeated));
        Assert.Equal(geometry == 0 ? 5 : geometry == 1 ? 4 : 3, count);
        Assert.Equal(geometry == 1, repeated);
        var singleCell = new ulong[WideFiniteAxisIntersection.GetFiniteValueRootCellWords(coefficients, signs)];
        for (int ordinal = 0; ordinal < count; ordinal++)
        {
            Assert.True(WideFiniteAxisIntersection.TryGetFiniteValueRoot(coefficients, signs,
                ordinal, singleCell, out FiniteAxisValueRoot root));
            Assert.Equal(ordinal, root.Ordinal);
            Assert.Equal(root.DenominatorShift, shifts[ordinal]);
            Assert.Equal(root.IsRational, (rationalMask & (1 << ordinal)) != 0);
            Assert.Equal(Decode(singleCell), Decode(cells.Slice(ordinal * 8, 8)));
        }
    }

    [Theory]
    [InlineData(1)]
    [InlineData(-1)]
    public void SturmMidpointRows_PreserveRightLimitSignsAndRepeatedEndpointRoots(int sign)
    {
        // x²(2x-1)²(3x-1)(3x-2)(x-1)(x+1). At one half,
        // F and F' both vanish: counting must use the first nonzero
        // derivative's right-limit sign. Zero is excluded, one included.
        BigInteger[] polynomial = { sign * 65537 };
        foreach (var factor in new[] { (0, 1), (0, 1), (-1, 2), (-1, 2), (-1, 3), (-2, 3), (-1, 1), (1, 1) })
        {
            var product = new BigInteger[polynomial.Length + 1];
            for (int index = 0; index < polynomial.Length; index++)
            {
                product[index] += factor.Item1 * polynomial[index];
                product[index + 1] += factor.Item2 * polynomial[index];
            }
            polynomial = product;
        }
        AssertRationalRoots(polynomial,
            new[] { Rational(1, 3), Rational(1, 2), Rational(2, 3), Rational(1, 1) }, true);
    }

    [Theory]
    [InlineData(1, -1)]
    [InlineData(-1, -1)]
    [InlineData(1, 1)]
    [InlineData(-1, 1)]
    public void SturmUncertainMidpoint_DistinguishesTinyRealSplitsFromComplexRoots(int sign, int offset)
    {
        BigInteger scale = BigInteger.One << 256;
        BigInteger center = scale / 2;
        // A common factor of 65537 forces exact Sturm isolation. Multiplying
        // by (3x-1) guarantees physical isolation even for the complex pair,
        // so the tiny midpoint residual must be classified exactly.
        BigInteger[] quadratic = { sign * 65537 * (center * center + offset),
            -sign * 65537 * 2 * scale * center, sign * 65537 * scale * scale };
        var polynomial = new BigInteger[4];
        for (int index = 0; index < quadratic.Length; index++)
        {
            polynomial[index] -= quadratic[index];
            polynomial[index + 1] += 3 * quadratic[index];
        }
        AssertRationalRoots(polynomial, offset > 0
            ? new[] { Rational(1, 3) }
            : new[] { Rational(1, 3), (center - 1, scale), (center + 1, scale) });
    }

    [Theory]
    [InlineData(1, 0)]
    [InlineData(-1, 0)]
    [InlineData(1, 7000)]
    public void EightSimpleRationalRoots_HaveExactOrderedEnclosures(int sign, int contentBits)
    {
        BigInteger content = contentBits == 0 ? BigInteger.One : (BigInteger.One << contentBits) + 1;
        BigInteger[] polynomial = { sign * content };
        var roots = new (BigInteger Numerator, BigInteger Denominator)[8];
        for (int ordinal = 0; ordinal < 8; ordinal++)
        {
            int numerator = 2 * ordinal + 1;
            var product = new BigInteger[polynomial.Length + 1];
            for (int index = 0; index < polynomial.Length; index++)
            {
                product[index] -= numerator * polynomial[index];
                product[index + 1] += 19 * polynomial[index];
            }
            polynomial = product;
            roots[ordinal] = (numerator, 19);
        }
        AssertRationalRoots(polynomial, roots);
    }

    [Theory]
    [InlineData(1)]
    [InlineData(-1)]
    public void SimpleIrrationalRoots_AreBracketedByIndependentSquareComparisons(int sign)
    {
        // (3x^2-1)(2x^2-1) has precisely two positive roots,
        // 1/sqrt(3) followed by 1/sqrt(2).
        Encode(new BigInteger[] { sign, 0, -5 * sign, 0, 6 * sign },
            out ulong[] coefficients, out sbyte[] signs);
        Span<ulong> cells = stackalloc ulong[64];
        Span<int> shifts = stackalloc int[8];
        Assert.True(WideFiniteAxisIntersection.TryGetFiniteValueRoots(coefficients, signs,
            cells, shifts, out int count, out int rationalMask, out bool repeated));
        Assert.Equal(2, count);
        Assert.Equal(0, rationalMask);
        Assert.False(repeated);
        for (int ordinal = 0; ordinal < 2; ordinal++)
        {
            BigInteger lower = Decode(cells.Slice(ordinal * 8, 8));
            BigInteger unit = BigInteger.One << (2 * shifts[ordinal]);
            int denominator = 3 - ordinal;
            Assert.True(lower > 0);
            Assert.True(denominator * lower * lower < unit);
            Assert.True(unit < denominator * (lower + 1) * (lower + 1));
        }
    }

    [Theory]
    [InlineData(0)]
    [InlineData(1)]
    [InlineData(2)]
    public void ExactBoundaryOrMidpointRoot_PreservesPhysicalEndpointSemantics(int boundary)
    {
        if (boundary == 0)
            AssertRationalRoots(new BigInteger[] { 0, -1, 3 }, new[] { Rational(1, 3) });
        else if (boundary == 1)
            AssertRationalRoots(new BigInteger[] { 1, -4, 3 }, new[] { Rational(1, 3), Rational(1, 1) });
        else
            AssertRationalRoots(new BigInteger[] { 1, -5, 6 }, new[] { Rational(1, 3), Rational(1, 2) });
    }

    [Theory]
    [InlineData(0)]
    [InlineData(1)]
    [InlineData(2)]
    public void InconclusivePrimeCertificate_DoesNotChangeTheExactAnswer(int cause)
    {
        const int prime = 65537;
        if (cause == 0)
            // The leading coefficient vanishes modulo the chosen prime.
            AssertRationalRoots(new BigInteger[] { 1, -(prime + 3), 3 * prime },
                new[] { Rational(1, prime), Rational(1, 3) });
        else if (cause == 1)
            // All coefficients vanish, but primitive F is square-free.
            AssertRationalRoots(new BigInteger[] { 2 * prime, -11 * prime, 15 * prime },
                new[] { Rational(1, 3), Rational(2, 5) });
        else
            // Distinct rational factors coincide only in the modular image.
            AssertRationalRoots(new BigInteger[] { prime + 1, -3 * (prime + 2), 9 },
                new[] { Rational(1, 3) });
    }

    [Fact]
    public void RepeatedFactorOutsidePhysicalInterval_PreservesGlobalMetadata()
    {
        // (x-2)^2(3x-1), with only a simple physical root.
        AssertRationalRoots(new BigInteger[] { -4, 16, -13, 3 },
            new[] { Rational(1, 3) }, true);
    }

    [Fact]
    public void DeepSimpleCluster_RemainsCompleteBeyondTheSubdivisionBudget()
    {
        BigInteger scale = BigInteger.One << 80;
        BigInteger lower = 3 * (BigInteger.One << 78) + 1;
        BigInteger upper = lower + 2;
        AssertRationalRoots(new[] { lower * upper, -scale * (lower + upper), scale * scale },
            new[] { (lower, scale), (upper, scale) });
    }

    [Fact]
    public void NearRealComplexPair_DoesNotBecomeAFalsePhysicalRoot()
    {
        BigInteger scale = BigInteger.One << 80;
        BigInteger center = scale / 3;
        // (scale*x-center)^2+1 is strictly positive on the entire real line,
        // even though coarse Bernstein cells around one third are ambiguous.
        AssertRationalRoots(new[] { center * center + 1, -2 * scale * center, scale * scale },
            Array.Empty<(BigInteger Numerator, BigInteger Denominator)>());
    }

    private static (BigInteger Numerator, BigInteger Denominator) Rational(int numerator, int denominator)
        => (numerator, denominator);

    private static void AssertRationalRoots(BigInteger[] polynomial,
        (BigInteger Numerator, BigInteger Denominator)[] expected, bool expectedRepeated = false)
    {
        Encode(polynomial, out ulong[] coefficients, out sbyte[] signs);
        ulong[] original = (ulong[])coefficients.Clone();
        sbyte[] originalSigns = (sbyte[])signs.Clone();
        Span<ulong> cells = stackalloc ulong[64];
        Span<int> shifts = stackalloc int[8];
        cells.Fill(ulong.MaxValue);
        Assert.True(WideFiniteAxisIntersection.TryGetFiniteValueRoots(coefficients, signs,
            cells, shifts, out int count, out int rationalMask, out bool repeated));
        Assert.Equal(expected.Length, count);
        Assert.Equal(expectedRepeated, repeated);
        for (int ordinal = 0; ordinal < count; ordinal++)
        {
            BigInteger lower = Decode(cells.Slice(ordinal * 8, 8));
            BigInteger expectedNumerator = expected[ordinal].Numerator << shifts[ordinal];
            BigInteger scaledLower = lower * expected[ordinal].Denominator;
            if ((rationalMask & (1 << ordinal)) != 0)
                Assert.Equal(expectedNumerator, scaledLower);
            else
            {
                Assert.True(lower > 0);
                Assert.True(scaledLower < expectedNumerator);
                Assert.True(expectedNumerator < scaledLower + expected[ordinal].Denominator);
            }
        }
        Assert.Equal(original, coefficients);
        Assert.Equal(originalSigns, signs);
    }

    private static BigInteger Decode(ReadOnlySpan<ulong> value)
    {
        BigInteger result = 0;
        for (int index = value.Length - 1; index >= 0; index--)
            result = (result << 64) + value[index];
        return result;
    }

    private static void Encode(BigInteger[] input, out ulong[] values, out sbyte[] signs)
    {
        int words = 1;
        foreach (BigInteger coefficient in input)
            words = Math.Max(words, (BigInteger.Abs(coefficient).GetByteCount() + 7) / 8);
        values = new ulong[input.Length * words];
        signs = new sbyte[input.Length];
        for (int index = 0; index < input.Length; index++)
        {
            signs[index] = (sbyte)input[index].Sign;
            BigInteger magnitude = BigInteger.Abs(input[index]);
            for (int word = 0; word < words; word++, magnitude >>= 64)
                values[index * words + word] = (ulong)(magnitude & ulong.MaxValue);
        }
    }
}
