using System;
using System.Numerics;
using FixedMathSharp.Geometry;
using Xunit;

namespace FixedMathSharp.Tests;

public sealed class FiniteAxisValueRootSubresultantTests
{
    [Theory]
    [InlineData(1)]
    [InlineData(-1)]
    public void NegativeIntermediateTwoExponent_IsCanceledBeforeRetainingTheNextRow(int sign)
    {
        // (3x-1)(5x-2)(7x-1)(9x-1). F and F' are primitive,
        // v2(lc(F'))=2, while the first degree-two remainder has odd
        // leading coefficient: a*(8*a*c-3*b*b), with a,b odd.
        // Thus the following step has q=0+2*0-4=-4. Its quotient
        // supplies the missing power of two; q itself must not be unsigned.
        BigInteger[] polynomial = { 2 * sign, -43 * sign, 317 * sign, -933 * sign, 945 * sign };
        Encode(polynomial, out ulong[] coefficients, out sbyte[] signs);
        AssertRationalRoot(coefficients, signs, 0, 1, 9);
        AssertRationalRoot(coefficients, signs, 1, 1, 7);
        AssertRationalRoot(coefficients, signs, 2, 1, 3);
        AssertRationalRoot(coefficients, signs, 3, 2, 5);
        AssertNoRoot(coefficients, signs, 4);
    }

    [Theory]
    [InlineData(1)]
    [InlineData(-1)]
    public void DyadicallyScaledDenseFactors_PreserveAllRootsWithOddScalarDivisors(int sign)
    {
        BigInteger denominator = 3 * (BigInteger.One << 37);
        BigInteger[] polynomial = { sign };
        // Eight distinct rational factors make a dense primitive polynomial.
        // The variable's dyadic scale creates large PRS two-valuations;
        // nontrivial odd parts of leading terms and root differences remain.
        for (int root = 1; root <= 8; root++)
        {
            var product = new BigInteger[polynomial.Length + 1];
            for (int index = 0; index < polynomial.Length; index++)
            {
                product[index] -= (2 * root - 1) * polynomial[index];
                product[index + 1] += denominator * polynomial[index];
            }
            polynomial = product;
        }
        Encode(polynomial, out ulong[] coefficients, out sbyte[] signs);
        for (int ordinal = 0; ordinal < 8; ordinal++)
            AssertRationalRoot(coefficients, signs, ordinal, 2 * ordinal + 1, denominator);
        AssertNoRoot(coefficients, signs, 8);
    }

    [Theory]
    [InlineData(1)]
    [InlineData(-1)]
    public void DyadicallyScaledAbnormalDrop_PreservesRawBrownScalarSigns(int sign)
    {
        // (16t^4-1)(81t^4-1), t=2^65*x. The degree7-to4 drop
        // exercises scalar powers and exact odd division with tracked scales.
        BigInteger[] polynomial =
        {
            sign, 0, 0, 0, -97 * sign * (BigInteger.One << 260), 0, 0, 0,
            1296 * sign * (BigInteger.One << 520)
        };
        Encode(polynomial, out ulong[] coefficients, out sbyte[] signs);
        AssertRationalRoot(coefficients, signs, 0, 1, 3 * (BigInteger.One << 65));
        AssertRationalRoot(coefficients, signs, 1, 1, BigInteger.One << 66);
        AssertNoRoot(coefficients, signs, 2);
    }

    [Theory]
    [InlineData(1)]
    [InlineData(-1)]
    public void DyadicallyScaledRepeatedFactor_ExportsTheSamePrimitiveRadical(int sign)
    {
        // (16t^4-1)^2, t=2^37*x, retains one degree-four factor.
        BigInteger[] polynomial =
        {
            sign, 0, 0, 0, -32 * sign * (BigInteger.One << 148), 0, 0, 0,
            256 * sign * (BigInteger.One << 296)
        };
        Encode(polynomial, out ulong[] coefficients, out sbyte[] signs);
        AssertRationalRoot(coefficients, signs, 0, 1, BigInteger.One << 38);
        AssertNoRoot(coefficients, signs, 1);
        Encode(new BigInteger[] { -1, 0, 0, 0, BigInteger.One << 152 },
            out ulong[] expectedFactor, out sbyte[] expectedSigns);
        ulong[] factor = new ulong[expectedFactor.Length];
        sbyte[] factorSigns = new sbyte[5];
        Assert.Equal(4, WideFiniteAxisIntersection.GetFiniteValueRepeatedRootFactor(
            coefficients, signs, factor, factorSigns));
        Assert.Equal(expectedFactor, factor);
        Assert.Equal(expectedSigns, factorSigns);
    }

    [Theory]
    [InlineData(1)]
    [InlineData(-1)]
    public void SparseDegreeDrops_PreserveBothPositiveRootsAndPolynomialSign(int sign)
    {
        // (16x^4-1)(81x^4-1) has positive roots1/2 and1/3. Its
        // derivative PRS drops from degree7 to4, with skipped zero terms
        // inside pseudo-division. Initial non-power-of-two content is harmless.
        BigInteger scale = sign * ((BigInteger.One << 129) + 17);
        BigInteger[] polynomial = { scale, 0, 0, 0, -97 * scale, 0, 0, 0, 1296 * scale };
        Encode(polynomial, out ulong[] coefficients, out sbyte[] signs);
        AssertRationalRoot(coefficients, signs, 0, 1, 3);
        AssertRationalRoot(coefficients, signs, 1, 1, 2);
        AssertNoRoot(coefficients, signs, 2);
    }

    [Theory]
    [InlineData(1)]
    [InlineData(-1)]
    public void EvenDegreeDrop_PreservesTheOddPseudoLeadingPowerSign(int sign)
    {
        // F=5x^8-3x^5-1 has exactly one positive root by Descartes,
        // and F(0)<0<F(1). The first remainder has degree5, so the next
        // pseudo-division uses a leading-coefficient power of three.
        BigInteger[] polynomial = { -sign, 0, 0, 0, 0, -3 * sign, 0, 0, 5 * sign };
        Encode(polynomial, out ulong[] coefficients, out sbyte[] signs);
        Span<ulong> cell = stackalloc ulong[3];
        Assert.True(WideFiniteAxisIntersection.TryGetFiniteValueRoot(coefficients, signs,
            0, cell, out FiniteAxisValueRoot root));
        WideFiniteAxisIntersection.RefineFiniteValueRoot(ref root, 80);
        Assert.False(root.IsRational);
        BigInteger numerator = Decode(root.LowerNumerator);
        Assert.True(sign * Evaluate(polynomial, numerator, root.DenominatorShift) < 0);
        Assert.True(sign * Evaluate(polynomial, numerator + 1, root.DenominatorShift) > 0);
        AssertNoRoot(coefficients, signs, 1);
    }

    [Theory]
    [InlineData(1)]
    [InlineData(-1)]
    public void RepeatedSparseFactor_IsExportedPrimitiveAfterRawSubresultants(int sign)
    {
        // (16x^4-1)^2. The last raw subresultant contains content which
        // must be removed before exporting into factor-sized storage.
        BigInteger[] polynomial = { sign, 0, 0, 0, -32 * sign, 0, 0, 0, 256 * sign };
        Encode(polynomial, out ulong[] coefficients, out sbyte[] signs);
        AssertRationalRoot(coefficients, signs, 0, 1, 2);
        AssertNoRoot(coefficients, signs, 1);
        Span<ulong> factor = stackalloc ulong[5];
        Span<sbyte> factorSigns = stackalloc sbyte[5];
        Assert.Equal(4, WideFiniteAxisIntersection.GetFiniteValueRepeatedRootFactor(
            coefficients, signs, factor, factorSigns));
        Assert.True(factor.SequenceEqual(new ulong[] { 1, 0, 0, 0, 16 }));
        Assert.True(factorSigns.SequenceEqual(new sbyte[] { -1, 0, 0, 0, 1 }));
    }

    private static void AssertRationalRoot(ulong[] coefficients, sbyte[] signs, int ordinal,
        BigInteger numerator, BigInteger denominator)
    {
        ulong[] cell = new ulong[WideFiniteAxisIntersection.GetFiniteValueRootCellWords(coefficients, signs)];
        Assert.True(WideFiniteAxisIntersection.TryGetFiniteValueRoot(coefficients, signs,
            ordinal, cell, out FiniteAxisValueRoot root));
        WideFiniteAxisIntersection.RefineFiniteValueRoot(ref root, 80);
        BigInteger lower = Decode(root.LowerNumerator) * denominator;
        BigInteger expected = (BigInteger)numerator << root.DenominatorShift;
        if (root.IsRational)
            Assert.Equal(expected, lower);
        else
        {
            Assert.True(lower < expected);
            Assert.True(expected < lower + denominator);
        }
        Assert.Equal(ordinal, root.Ordinal);
    }

    private static void AssertNoRoot(ulong[] coefficients, sbyte[] signs, int ordinal)
    {
        ulong[] cell = new ulong[WideFiniteAxisIntersection.GetFiniteValueRootCellWords(coefficients, signs)];
        Assert.False(WideFiniteAxisIntersection.TryGetFiniteValueRoot(coefficients, signs, ordinal, cell, out _));
    }

    private static BigInteger Evaluate(BigInteger[] polynomial, BigInteger numerator, int shift)
    {
        BigInteger value = 0;
        for (int index = polynomial.Length - 1; index >= 0; index--)
            value = value * numerator + (polynomial[index] << ((polynomial.Length - 1 - index) * shift));
        return value;
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
            Assert.Equal(BigInteger.Zero, magnitude);
        }
    }
}
