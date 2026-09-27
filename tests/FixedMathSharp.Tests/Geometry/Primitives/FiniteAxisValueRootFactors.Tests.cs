using System;
using System.Numerics;
using FixedMathSharp.Geometry;
using Xunit;

namespace FixedMathSharp.Tests;

public sealed class FiniteAxisValueRootFactorsTests
{
    [Fact]
    public void RepeatedFactor_ExcludesSimpleFactorsAndRemovesEveryMultiplicity()
    {
        BigInteger[] first = { -1, 3 };
        BigInteger[] second = { -2, 3 };
        BigInteger[] polynomial = Multiply(Power(first, 3), Power(second, 2));
        polynomial = Multiply(polynomial, new BigInteger[] { 1, 1 });
        AssertFactor(polynomial, new BigInteger[] { 2, -9, 9 });
    }

    [Fact]
    public void RepeatedFactor_ReducesASeventhDegreeDerivativeGcdToOneLinearFactor() =>
        AssertFactor(Power(new BigInteger[] { -1, 3 }, 8), new BigInteger[] { -1, 3 });

    [Fact]
    public void RepeatedFactor_PreservesTheFullQuarticRadical()
    {
        // Four different roots, each repeated twice; expected expansion is
        // (3x-1)(3x-2)(x+1)(x-2), independently specified below.
        BigInteger[] radical = { -4, 16, -7, -18, 9 };
        AssertFactor(Multiply(radical, radical), radical);
    }

    [Fact]
    public void RepeatedFactor_KeepsIrreducibleAndZeroRootFactors()
    {
        BigInteger[] quadratic = { -1, 0, 2 };
        BigInteger[] polynomial = Multiply(Power(quadratic, 3), new BigInteger[] { 0, 0, 1 });
        AssertFactor(polynomial, new BigInteger[] { 0, -1, 0, 2 });
    }

    [Fact]
    public void RepeatedFactor_RemovesNegativeContentAndPreservesInput()
    {
        BigInteger[] factor = { -1, 0, 2 };
        BigInteger[] polynomial = Multiply(Power(factor, 3), new BigInteger[] { 1, 1 });
        BigInteger content = -((BigInteger.One << 300) + 21);
        for (int index = 0; index < polynomial.Length; index++)
            polynomial[index] *= content;
        AssertFactor(polynomial, factor, paddedWords: 116);
    }

    [Fact]
    public void RepeatedFactor_DividesNonMonicMultiwordFactorsExactly()
    {
        BigInteger leading = (BigInteger.One << 130) + 3;
        BigInteger[] repeated = { -5, leading };
        BigInteger[] polynomial = Multiply(Power(repeated, 5), new BigInteger[] { 7, 0, 1 });
        AssertFactor(polynomial, repeated);
    }

    [Fact]
    public void RepeatedFactor_HandlesDegreeDropAndClearsUnusedOutput() =>
        AssertFactor(new BigInteger[] { 1, -4, 4, 0, 0, 0, 0, 0, 0 }, new BigInteger[] { -1, 2 });

    [Fact]
    public void RepeatedFactor_ReturnsOneForConstantsAndSquareFreePolynomials()
    {
        AssertFactor(new BigInteger[] { -37 }, new BigInteger[] { 1 });
        AssertFactor(new BigInteger[] { -1, 3 }, new BigInteger[] { 1 });
        AssertFactor(new BigInteger[] { -2, -1, 1 }, new BigInteger[] { 1 });
    }

    [Fact]
    public void RepeatedFactor_RejectsTheZeroPolynomialWithClearedOutput() =>
        AssertFactor(new BigInteger[] { 0, 0, 0 }, Array.Empty<BigInteger>());

    [Fact]
    public void RepeatedFactor_UsesNoWarmedAllocations()
    {
        Encode(Power(new BigInteger[] { -1, 0, 2 }, 4), 1,
            out ulong[] coefficients, out sbyte[] signs);
        ulong[] output = new ulong[10];
        sbyte[] outputSigns = new sbyte[5];
        int degree = -1;
        long allocated = FixedMathTestHelper.MeasureWarmedAllocations(() =>
            degree = WideFiniteAxisIntersection.GetFiniteValueRepeatedRootFactor(
                coefficients, signs, output, outputSigns));
        Assert.Equal(2, degree);
        Assert.Equal(0, allocated);
        Assert.Equal(new sbyte[] { -1, 0, 1, 0, 0 }, outputSigns);
        Assert.Equal(1UL, output[0]);
        Assert.Equal(2UL, output[4]);
    }

    private static void AssertFactor(BigInteger[] polynomial, BigInteger[] expected, int paddedWords = 1)
    {
        Encode(polynomial, paddedWords, out ulong[] coefficients, out sbyte[] signs);
        ulong[] original = (ulong[])coefficients.Clone();
        sbyte[] originalSigns = (sbyte[])signs.Clone();
        int bits = 0;
        foreach (BigInteger value in polynomial)
            bits = Math.Max(bits, (int)BigInteger.Abs(value).GetBitLength());
        int outputWords = Math.Max(1, ((bits + 1) / 2 + 5 + 63) / 64);
        ulong[] output = new ulong[5 * outputWords];
        sbyte[] outputSigns = new sbyte[5];
        Array.Fill(output, ulong.MaxValue);
        Array.Fill(outputSigns, (sbyte)-1);
        int degree = WideFiniteAxisIntersection.GetFiniteValueRepeatedRootFactor(
            coefficients, signs, output, outputSigns);
        Assert.Equal(expected.Length - 1, degree);
        for (int index = 0; index < 5; index++)
        {
            BigInteger actual = 0;
            for (int word = outputWords - 1; word >= 0; word--)
                actual = (actual << 64) | output[index * outputWords + word];
            BigInteger wanted = index < expected.Length ? expected[index] : 0;
            Assert.Equal(wanted.Sign, outputSigns[index]);
            Assert.Equal(BigInteger.Abs(wanted), actual);
        }
        Assert.Equal(original, coefficients);
        Assert.Equal(originalSigns, signs);
    }

    private static BigInteger[] Power(BigInteger[] factor, int exponent)
    {
        BigInteger[] result = { 1 };
        for (int index = 0; index < exponent; index++)
            result = Multiply(result, factor);
        return result;
    }

    private static BigInteger[] Multiply(BigInteger[] left, BigInteger[] right)
    {
        var result = new BigInteger[left.Length + right.Length - 1];
        for (int i = 0; i < left.Length; i++)
        for (int j = 0; j < right.Length; j++)
            result[i + j] += left[i] * right[j];
        return result;
    }

    private static void Encode(BigInteger[] polynomial, int paddedWords,
        out ulong[] coefficients, out sbyte[] signs)
    {
        int bits = 0;
        foreach (BigInteger value in polynomial)
            bits = Math.Max(bits, (int)BigInteger.Abs(value).GetBitLength());
        int words = Math.Max(paddedWords, (bits + 63) / 64);
        coefficients = new ulong[words * polynomial.Length];
        signs = new sbyte[polynomial.Length];
        for (int index = 0; index < polynomial.Length; index++)
        {
            BigInteger value = BigInteger.Abs(polynomial[index]);
            signs[index] = (sbyte)polynomial[index].Sign;
            for (int word = 0; word < words; word++)
            {
                coefficients[index * words + word] = (ulong)(value & ulong.MaxValue);
                value >>= 64;
            }
        }
    }
}
