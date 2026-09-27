using System;
using System.Numerics;
using FixedMathSharp.Geometry;
using Xunit;

namespace FixedMathSharp.Tests;

public sealed class FiniteAxisPolynomialArithmeticTests
{
    [Theory]
    [InlineData(0)]
    [InlineData(1)]
    [InlineData(64)]
    [InlineData(65)]
    public void ScaleVariable_PreservesSignsAndCrossesWholeWordBoundaries(int shift)
    {
        BigInteger[] source = { -3, 0, 5, -2 };
        ulong[] coefficients = Encode(source, 5, out sbyte[] signs);
        WideFiniteAxisIntersection.ScaleFiniteAxisPolynomialVariable(coefficients, source.Length, shift);
        for (int index = 0; index < source.Length; index++)
            Assert.Equal(source[index] << (index * shift), Decode(coefficients, signs, index));
    }

    [Theory]
    [InlineData(0)]
    [InlineData(64)]
    [InlineData(130)]
    public void RemovePowerOfTwo_ReturnsTheJointScaleWithoutChangingSigns(int shift)
    {
        BigInteger[] expected = { -3, 0, 5, -2 };
        BigInteger[] source = { -3 * (BigInteger.One << shift), 0,
            5 * (BigInteger.One << shift), -2 * (BigInteger.One << shift) };
        ulong[] coefficients = Encode(source, 4, out sbyte[] signs);
        Assert.Equal(shift, WideFiniteAxisIntersection.NormalizeFiniteAxisPolynomialPowerOfTwo(coefficients, signs));
        for (int index = 0; index < source.Length; index++)
            Assert.Equal(expected[index], Decode(coefficients, signs, index));
    }

    [Fact]
    public void RemovePowerOfTwo_ZeroPolynomialHasNoRemovedScale()
    {
        ulong[] coefficients = Encode(new BigInteger[] { 0, 0 }, 2, out sbyte[] signs);
        Assert.Equal(0, WideFiniteAxisIntersection.NormalizeFiniteAxisPolynomialPowerOfTwo(coefficients, signs));
        Assert.Equal(BigInteger.Zero, Decode(coefficients, signs, 0));
        Assert.Equal(BigInteger.Zero, Decode(coefficients, signs, 1));
    }

    [Fact]
    public void Differentiate_PreservesCarriesWithDifferentOutputStrideAndClearsPadding()
    {
        BigInteger[] source = { -7, 0, ulong.MaxValue, -(BigInteger.One << 70), 0 };
        ulong[] coefficients = Encode(source, 2, out sbyte[] signs);
        var derivative = new ulong[6 * 3];
        var derivativeSigns = new sbyte[6];
        Array.Fill(derivative, ulong.MaxValue);
        Array.Fill(derivativeSigns, (sbyte)-1);
        WideFiniteAxisIntersection.DifferentiateFiniteAxisPolynomial(
            coefficients, signs, derivative, derivativeSigns);
        BigInteger[] expected = { 0, 2 * (BigInteger)ulong.MaxValue, -3 * (BigInteger.One << 70), 0, 0, 0 };
        for (int index = 0; index < expected.Length; index++)
            Assert.Equal(expected[index], Decode(derivative, derivativeSigns, index));
    }

    [Fact]
    public void Differentiate_ConstantProducesCanonicalZero()
    {
        ulong[] coefficients = Encode(new BigInteger[] { -23 }, 1, out sbyte[] signs);
        ulong[] derivative = { ulong.MaxValue };
        sbyte[] derivativeSigns = { -1 };
        WideFiniteAxisIntersection.DifferentiateFiniteAxisPolynomial(
            coefficients, signs, derivative, derivativeSigns);
        Assert.Equal(BigInteger.Zero, Decode(derivative, derivativeSigns, 0));
    }

    [Theory]
    [InlineData(-3)]
    [InlineData(0)]
    [InlineData(3)]
    [InlineData(int.MinValue)]
    public void Add_AppliesSignedMultiplierAndDegreeOffsetAcrossStrides(int multiplier)
    {
        ulong[] source = Encode(new BigInteger[] { ulong.MaxValue, -7, 0 }, 1, out sbyte[] sourceSigns);
        ulong[] result = Encode(new BigInteger[] { 2, 0, 21, 0, 0 }, 2, out sbyte[] resultSigns);
        WideFiniteAxisIntersection.AddFiniteAxisPolynomial(
            source, sourceSigns, result, resultSigns, multiplier, degreeOffset: 1);
        BigInteger[] expected = { 2, (BigInteger)ulong.MaxValue * multiplier,
            21 - 7 * (BigInteger)multiplier, 0, 0 };
        for (int index = 0; index < expected.Length; index++)
            Assert.Equal(expected[index], Decode(result, resultSigns, index));
    }

    [Fact]
    public void Multiply_DifferentStridesClearAndAccumulateExactSignedProducts()
    {
        ulong[] left = Encode(new BigInteger[] { ulong.MaxValue, -1, 0 }, 1, out sbyte[] leftSigns);
        ulong[] right = Encode(new BigInteger[] { BigInteger.One << 70, 3 }, 2, out sbyte[] rightSigns);
        var result = new ulong[5 * 3];
        var resultSigns = new sbyte[5];
        var scratch = new ulong[3];
        Array.Fill(result, ulong.MaxValue);
        Array.Fill(resultSigns, (sbyte)-1);
        Array.Fill(scratch, ulong.MaxValue);
        WideFiniteAxisIntersection.MultiplyFiniteAxisPolynomials(
            left, leftSigns, right, rightSigns, result, resultSigns, scratch);
        BigInteger[] expected =
        {
            (BigInteger)ulong.MaxValue << 70,
            3 * (BigInteger)ulong.MaxValue - (BigInteger.One << 70), -3, 0, 0
        };
        for (int index = 0; index < expected.Length; index++)
            Assert.Equal(expected[index], Decode(result, resultSigns, index));

        sbyte[] negatedSigns = { -1, 1, 0 };
        WideFiniteAxisIntersection.MultiplyFiniteAxisPolynomials(
            left, negatedSigns, right, rightSigns, result, resultSigns, scratch, accumulate: true);
        for (int index = 0; index < resultSigns.Length; index++)
            Assert.Equal(BigInteger.Zero, Decode(result, resultSigns, index));
    }

    private static ulong[] Encode(BigInteger[] values, int words, out sbyte[] signs)
    {
        var coefficients = new ulong[values.Length * words];
        signs = new sbyte[values.Length];
        for (int index = 0; index < values.Length; index++)
        {
            signs[index] = (sbyte)values[index].Sign;
            BigInteger magnitude = BigInteger.Abs(values[index]);
            for (int word = 0; word < words; word++)
            {
                coefficients[index * words + word] = (ulong)(magnitude & ulong.MaxValue);
                magnitude >>= 64;
            }
            Assert.Equal(BigInteger.Zero, magnitude);
        }
        return coefficients;
    }

    private static BigInteger Decode(ulong[] coefficients, sbyte[] signs, int index)
    {
        int words = coefficients.Length / signs.Length;
        BigInteger magnitude = 0;
        for (int word = words - 1; word >= 0; word--)
            magnitude = (magnitude << 64) | coefficients[index * words + word];
        Assert.Equal(magnitude.IsZero, signs[index] == 0);
        return magnitude * signs[index];
    }
}
