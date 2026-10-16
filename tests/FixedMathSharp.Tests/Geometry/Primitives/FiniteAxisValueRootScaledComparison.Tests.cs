using System;
using System.Numerics;
using FixedMathSharp.Geometry;
using Xunit;

namespace FixedMathSharp.Tests;

public sealed class FiniteAxisValueRootScaledComparisonTests
{
    [Theory]
    [InlineData(1, 2, 3, 1, 6, 1, 0)]
    [InlineData(3, 4, 3, 1, 2, 1, -1)]
    [InlineData(1, 2, 1, 3, 4, 3, 1)]
    [InlineData(1, 2, 7, 1, 2, 7, 0)]
    [InlineData(1, 1, 3, 1, 3, 1, 0)]
    [InlineData(1, 2, 3, 1, 2, 1, -1)]
    public void RationalRoots_CompareAfterDivisionAndPreserveInputs(
        int firstNumerator, int firstDenominator, int firstDivisor,
        int secondNumerator, int secondDenominator, int secondDivisor, int expected) =>
        AssertComparison(new BigInteger[] { -firstNumerator, firstDenominator }, firstDivisor,
            new BigInteger[] { -secondNumerator, secondDenominator }, secondDivisor, expected);

    [Fact]
    public void IrrationalRoots_FromDifferentPolynomialsCanHaveEqualScaledValues()
    {
        // sqrt(1/2)/3 = sqrt(1/18), including different polynomial contents.
        AssertComparison(new BigInteger[] { -1, 0, 2 }, 3,
            new BigInteger[] { -7, 0, 126 }, 1, 0);
    }

    [Theory]
    [InlineData(-1)]
    [InlineData(1)]
    public void UnequalDeepRoots_RemainOrderedAfterLargeNonbinaryDivision(int offset)
    {
        BigInteger scale = BigInteger.One << 600;
        BigInteger divisor = (BigInteger.One << 131) + 3;
        // The second quotient differs by offset/(3*scale*divisor).
        AssertComparison(new BigInteger[] { -1, 3 }, divisor,
            new[] { -2 * (scale + offset), 3 * scale }, 2 * divisor, -offset);
    }

    [Fact]
    public void CoincidentLowerEndpoints_DoNotConfuseAnOpenCellWithASingleton()
    {
        // sqrt(1/2)/3 is strictly below 1/4; equality of coarse lower
        // endpoints cannot certify a tie between differently scaled cells.
        AssertComparison(new BigInteger[] { -1, 0, 2 }, 3,
            new BigInteger[] { -1, 4 }, 1, -1);
    }

    [Theory]
    [InlineData(0, 0)]
    [InlineData(1, 1)]
    public void ProportionalScaledPolynomials_ShouldPreserveOrdinalsAcrossDifferentRootCounts(int ordinal, int expected) =>
        // P=x*(16*x*x-16*x+3), Q=-x*(4*x*x-8*x+3).
        // Q(2*x)=-2*P(x), but Q's second positive root exceeds one.
        AssertComparison(new BigInteger[] { 0, 3, -16, 16 }, 1,
            new BigInteger[] { 0, -3, 8, -4 }, 2, expected, ordinal);

    [Fact]
    public void DifferentDegreePolynomials_ShouldRetainExactSharedRootComparison() =>
        AssertComparison(new BigInteger[] { -1, 3 }, 1,
            new BigInteger[] { -4, 0, 9 }, 2, 0);

    [Fact]
    public void EqualDivisors_ShouldCancelBeforeProportionalContentComparison() =>
        AssertComparison(new BigInteger[] { -1, 3 }, 7,
            new BigInteger[] { -7, 21 }, 7, 0);

    [Fact]
    public void DifferentZeroPatterns_ShouldRetainExactOrdering() =>
        AssertComparison(new BigInteger[] { -1, 2, 1 }, 1,
            new BigInteger[] { 0, -1, 3 }, 2, 1);

    [Fact]
    public void DifferentCoefficientSigns_ShouldRetainExactOrdering() =>
        AssertComparison(new BigInteger[] { -1, 0, 2 }, 3,
            new BigInteger[] { -1, 1, 2 }, 1, -1);

    private static void AssertComparison(BigInteger[] firstPolynomial, BigInteger firstDivisor,
        BigInteger[] secondPolynomial, BigInteger secondDivisor, int expected, int firstOrdinal = 0)
    {
        CreatePolynomial(firstPolynomial, out ulong[] firstValues, out sbyte[] firstSigns);
        CreatePolynomial(secondPolynomial, out ulong[] secondValues, out sbyte[] secondSigns);
        ulong[] firstCell = new ulong[WideFiniteAxisIntersection.GetFiniteValueRootCellWords(firstValues, firstSigns)];
        ulong[] secondCell = new ulong[WideFiniteAxisIntersection.GetFiniteValueRootCellWords(secondValues, secondSigns)];
        Assert.True(WideFiniteAxisIntersection.TryGetFiniteValueRoot(firstValues, firstSigns, firstOrdinal, firstCell, out FiniteAxisValueRoot first));
        Assert.True(WideFiniteAxisIntersection.TryGetFiniteValueRoot(secondValues, secondSigns, 0, secondCell, out FiniteAxisValueRoot second));
        ulong[] originalFirst = (ulong[])firstCell.Clone(), originalSecond = (ulong[])secondCell.Clone();
        ulong[] firstScale = Magnitude(firstDivisor), secondScale = Magnitude(secondDivisor);
        Assert.Equal(expected, WideFiniteAxisIntersection.CompareFiniteValueRoots(first, firstScale, second, secondScale));
        Assert.Equal(-expected, WideFiniteAxisIntersection.CompareFiniteValueRoots(second, secondScale, first, firstScale));
        Assert.Equal(originalFirst, firstCell);
        Assert.Equal(originalSecond, secondCell);
    }

    private static void CreatePolynomial(BigInteger[] coefficients, out ulong[] values, out sbyte[] signs)
    {
        int words = 1;
        foreach (BigInteger coefficient in coefficients)
            words = Math.Max(words, (BigInteger.Abs(coefficient).GetByteCount() + 7) / 8);
        values = new ulong[coefficients.Length * words];
        signs = new sbyte[coefficients.Length];
        for (int i = 0; i < coefficients.Length; i++)
        {
            signs[i] = (sbyte)coefficients[i].Sign;
            Magnitude(BigInteger.Abs(coefficients[i])).CopyTo(values, i * words);
        }
    }

    private static ulong[] Magnitude(BigInteger value)
    {
        var words = new ulong[Math.Max(1, (value.GetByteCount() + 7) / 8)];
        for (int i = 0; i < words.Length; i++)
        {
            words[i] = (ulong)(value & ulong.MaxValue);
            value >>= 64;
        }
        return words;
    }
}
