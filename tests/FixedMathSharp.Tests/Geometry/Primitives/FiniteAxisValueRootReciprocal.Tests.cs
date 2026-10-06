using System;
using System.Numerics;
using FixedMathSharp.Geometry;
using Xunit;

namespace FixedMathSharp.Tests;

public sealed class FiniteAxisValueRootReciprocalTests
{
    [Theory]
    [InlineData(0)]
    [InlineData(1)]
    [InlineData(2)]
    [InlineData(3)]
    [InlineData(4)]
    [InlineData(5)]
    [InlineData(6)]
    [InlineData(7)]
    [InlineData(8)]
    [InlineData(9)]
    [InlineData(10)]
    [InlineData(11)]
    public void ReciprocalCells_MatchIndependentOwnersAcrossSignsEndpointsAndCapacity(int geometry)
    {
        BigInteger[] polynomial = CreatePolynomial(geometry);
        foreach (int orientation in new[] { 1, -1 })
        {
            var input = new BigInteger[polynomial.Length];
            for (int index = 0; index < input.Length; index++)
                input[index] = polynomial[index] * orientation;
            Encode(input, out ulong[] coefficients, out sbyte[] signs);
            var original = (ulong[])coefficients.Clone();
            var originalSigns = (sbyte[])signs.Clone();
            var reversed = (BigInteger[])input.Clone();
            Array.Reverse(reversed);
            Encode(reversed, out ulong[] expectedReciprocal, out sbyte[] expectedReciprocalSigns);
            var reciprocal = new ulong[coefficients.Length];
            var reciprocalSigns = new sbyte[signs.Length];
            Array.Fill(reciprocal, ulong.MaxValue);
            int words = geometry is 7 or 8 ? 1 : 8;
            var cells = new ulong[8 * words];
            var shifts = new int[8];
            var reciprocalCells = new ulong[8 * words];
            var reciprocalShifts = new int[8];
            WideFiniteAxisIntersection.GetFiniteValueReciprocalRoots(coefficients, signs,
                reciprocal, reciprocalSigns, cells, shifts, reciprocalCells, reciprocalShifts,
                out FiniteAxisValueRoots roots, out FiniteAxisValueRoots reciprocalRoots);
            Assert.Equal(expectedReciprocal, reciprocal);
            Assert.Equal(expectedReciprocalSigns, reciprocalSigns);
            Assert.Equal(original, coefficients);
            Assert.Equal(originalSigns, signs);
            AssertMatchesIndependent(roots, coefficients, signs, words);
            AssertMatchesIndependent(reciprocalRoots, reciprocal, reciprocalSigns, words);
        }
    }

    private static void AssertMatchesIndependent(FiniteAxisValueRoots actual,
        ReadOnlySpan<ulong> coefficients, ReadOnlySpan<sbyte> signs, int compactWords)
    {
        Span<ulong> cells = stackalloc ulong[8 * compactWords];
        Span<int> shifts = stackalloc int[8];
        FiniteAxisValueRoots expected = WideFiniteAxisIntersection.GetFiniteValueRoots(coefficients, signs, cells, shifts);
        Assert.Equal(expected.Count, actual.Count);
        Assert.Equal(expected.HasRepeatedRoots, actual.HasRepeatedRoots);
        Assert.Equal(expected.HasCompactCells, actual.HasCompactCells);
        var expectedCell = new ulong[WideFiniteAxisIntersection.GetFiniteValueRootCellWords(coefficients, signs)];
        var actualCell = new ulong[expectedCell.Length];
        for (int ordinal = 0; ordinal < expected.Count; ordinal++)
        {
            FiniteAxisValueRoot first = FiniteAxisValueRoots.GetRoot(expected, ordinal, coefficients, signs, expectedCell);
            FiniteAxisValueRoot second = FiniteAxisValueRoots.GetRoot(actual, ordinal, coefficients, signs, actualCell);
            Assert.Equal(first.Ordinal, second.Ordinal);
            Assert.Equal(first.DenominatorShift, second.DenominatorShift);
            Assert.Equal(first.IsRational, second.IsRational);
            Assert.Equal(expectedCell, actualCell);
        }
    }

    private static BigInteger[] CreatePolynomial(int geometry)
    {
        BigInteger scale = BigInteger.One << 80;
        if (geometry is 7 or 8)
        {
            BigInteger center = geometry == 7 ? scale / 2 : 2 * scale;
            return new[] { 65537 * (center * center - 1), -65537 * 2 * scale * center, 65537 * scale * scale };
        }
        if (geometry == 9) return new BigInteger[] { 0 };
        if (geometry == 10) return new BigInteger[] { 7 };
        if (geometry == 11) return new BigInteger[] { 65537, 0, -5 * 65537, 0, 6 * 65537 };
        var factors = geometry switch
        {
            0 => new[] { (-1, 3), (-1, 2), (-1, 1), (-2, 1), (-3, 1), (1, 1) },
            1 => new[] { (-1, 3), (-1, 3), (-2, 1), (-2, 1) },
            2 => new[] { (0, 1), (0, 1), (-1, 3) },
            3 => new[] { (-1, 3), (-2, 1) },
            4 => new[] { (-1, 3), (-3, 2), (-7, 3) },
            5 => new[] { (-1, 2), (-3, 2) },
            _ => new[] { (-1, 3), (-2, 1) }
        };
        // Content 65537 forces Sturm; cases4-6 independently exercise both,
        // only reciprocal, and only primary Bernstein success respectively.
        BigInteger[] polynomial = { geometry is 4 or 5 or 6 ? 1 : 65537 };
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
        if (geometry == 3) Array.Resize(ref polynomial, polynomial.Length + 2);
        return polynomial;
    }

    private static void Encode(BigInteger[] polynomial, out ulong[] coefficients, out sbyte[] signs)
    {
        int words = 1;
        foreach (BigInteger coefficient in polynomial)
            words = Math.Max(words, (BigInteger.Abs(coefficient).GetByteCount() + 7) / 8);
        coefficients = new ulong[words * polynomial.Length];
        signs = new sbyte[polynomial.Length];
        for (int index = 0; index < polynomial.Length; index++)
        {
            signs[index] = (sbyte)polynomial[index].Sign;
            BigInteger magnitude = BigInteger.Abs(polynomial[index]);
            for (int word = 0; word < words; word++, magnitude >>= 64)
                coefficients[index * words + word] = (ulong)(magnitude & ulong.MaxValue);
        }
    }
}
