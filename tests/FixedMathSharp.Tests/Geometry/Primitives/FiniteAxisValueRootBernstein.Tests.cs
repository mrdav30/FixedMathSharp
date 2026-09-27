using System;
using System.Numerics;
using FixedMathSharp.Geometry;
using Xunit;

namespace FixedMathSharp.Tests;

public sealed class FiniteAxisValueRootBernsteinTests
{
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
