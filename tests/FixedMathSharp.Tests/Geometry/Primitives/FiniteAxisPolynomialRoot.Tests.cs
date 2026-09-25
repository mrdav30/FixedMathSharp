using System;
using System.Numerics;
using FixedMathSharp.Geometry;
using Xunit;

namespace FixedMathSharp.Tests;

public sealed class FiniteAxisPolynomialRootTests
{
    [Theory]
    [InlineData(0)]
    [InlineData(1)]
    [InlineData(-1)]
    public void LargestPositiveRoot_RejectsConstants(int constant) =>
        Assert.False(HasPositiveRoot(new BigInteger[] { constant }));

    [Theory]
    [InlineData(1)]
    [InlineData(2)]
    [InlineData(3)]
    [InlineData(4)]
    public void LargestPositiveRoot_RejectsPolynomialWithOnlyZeroRoots(int degree)
    {
        var polynomial = new BigInteger[degree + 1];
        polynomial[degree] = 1;
        Assert.False(HasPositiveRoot(polynomial));
    }

    [Fact]
    public void LargestPositiveRoot_RejectsZeroAndNegativeRoots() =>
        Assert.False(HasPositiveRoot(new BigInteger[] { 0, 2, 3, 1 }));

    [Fact]
    public void LargestPositiveRoot_RejectsComplexRoots() =>
        Assert.False(HasPositiveRoot(new BigInteger[] { 1, 0, 1 }));

    [Theory]
    [InlineData(1)]
    [InlineData(-1)]
    public void SignAtRoot_SelectsLargestRootAndPreservesExactEquality(int leading)
    {
        BigInteger[] polynomial = { 24 * leading, -50 * leading, 35 * leading, -10 * leading, leading };
        AssertSign(polynomial, new BigInteger[] { -3, 1 }, 1);
        AssertSign(polynomial, new BigInteger[] { -4, 1 }, 0);
        AssertSign(polynomial, new BigInteger[] { -5, 1 }, -1);
    }

    [Theory]
    [InlineData(1)]
    [InlineData(2)]
    [InlineData(3)]
    [InlineData(4)]
    public void SignAtRoot_HandlesEveryMultiplicity(int multiplicity)
    {
        BigInteger[] polynomial = { 1 };
        for (int index = 0; index < multiplicity; index++)
            polynomial = Multiply(polynomial, new BigInteger[] { -3, 2 });
        AssertSign(polynomial, new BigInteger[] { -1, 1 }, 1);
        AssertSign(polynomial, new BigInteger[] { -3, 2 }, 0);
        AssertSign(polynomial, new BigInteger[] { -2, 1 }, -1);
    }

    [Theory]
    [InlineData(1)]
    [InlineData(2)]
    [InlineData(3)]
    [InlineData(4)]
    public void SignAtRoot_PreservesNonDyadicMultipleRootEquality(int multiplicity)
    {
        BigInteger[] polynomial = { 1 };
        for (int index = 0; index < multiplicity; index++)
            polynomial = Multiply(polynomial, new BigInteger[] { -1, 3 });
        AssertSign(polynomial, new BigInteger[] { -1, 3 }, 0);
        AssertSign(polynomial, new BigInteger[] { -1, 4 }, 1);
        AssertSign(polynomial, new BigInteger[] { -1, 2 }, -1);
    }

    [Fact]
    public void NormalizeScale_RemovesOnlyTheCommonPositivePowerOfTwo()
    {
        BigInteger scale = BigInteger.One << 130;
        Encode(new BigInteger[] { -12 * scale, 0, 28 * scale }, out ulong[] coefficients, out sbyte[] signs);
        WideFiniteAxisIntersection.NormalizeFiniteAxisPolynomialPowerOfTwo(coefficients, signs);
        int words = coefficients.Length / signs.Length;
        Assert.Equal((ulong)3, coefficients[0]);
        Assert.Equal((ulong)7, coefficients[2 * words]);
        Assert.Equal(new sbyte[] { -1, 0, 1 }, signs);
        for (int index = 0; index < coefficients.Length; index++)
        {
            if (index != 0 && index != 2 * words)
                Assert.Equal(0UL, coefficients[index]);
        }
        ulong[] normalized = (ulong[])coefficients.Clone();
        WideFiniteAxisIntersection.NormalizeFiniteAxisPolynomialPowerOfTwo(coefficients, signs);
        Assert.Equal(normalized, coefficients);
    }

    [Fact]
    public void NormalizeScale_PreservesTheZeroPolynomial()
    {
        var coefficients = new ulong[9];
        var signs = new sbyte[3];
        WideFiniteAxisIntersection.NormalizeFiniteAxisPolynomialPowerOfTwo(coefficients, signs);
        Assert.Equal(new ulong[9], coefficients);
        Assert.Equal(new sbyte[3], signs);
    }

    [Fact]
    public void SignAtRoot_HandlesIrrationalRootWithRepeatedFactor()
    {
        BigInteger[] polynomial = { 4, 0, -4, 0, 1 }; // (x²-2)².
        AssertSign(polynomial, new BigInteger[] { -2, 0, 1 }, 0);
        AssertSign(polynomial, new BigInteger[] { -1, 1 }, 1);
        AssertSign(polynomial, new BigInteger[] { -3, 2 }, -1);
    }

    [Theory]
    [InlineData(1, 1)]
    [InlineData(1, -1)]
    [InlineData(-1, 1)]
    [InlineData(-1, -1)]
    public void SignAtRoot_PreservesNonDyadicLinearBoundariesAndLeadingSigns(
        int polynomialSign, int querySign)
    {
        BigInteger scale = (BigInteger.One << 130) + 1;
        for (int multiplicity = 1; multiplicity <= 4; multiplicity++)
        {
            BigInteger[] polynomial = { polynomialSign };
            for (int factor = 0; factor < 4; factor++)
                polynomial = Multiply(polynomial, factor < multiplicity
                    ? new BigInteger[] { -1, 3 } : new BigInteger[] { 2, 1 });
            // The largest positive root is exactly 1/3 for every multiplicity.
            // The query at that root is querySign*offset, even though each
            // adjacent rational threshold differs by much less than 2^-64.
            for (int offset = -1; offset <= 1; offset++)
                AssertSign(polynomial,
                    new BigInteger[] { querySign * (-scale + offset), querySign * 3 * scale },
                    querySign * offset);
        }
    }

    [Fact]
    public void SignAtRoot_PreservesFullWidthLinearBoundary()
    {
        BigInteger stationaryScale = (BigInteger.One << 2040) + 1;
        BigInteger queryScale = (BigInteger.One << 12900) + 3;
        // (15x-1)(x+1)(x²+1) has the sole positive root 1/15.
        BigInteger[] polynomial = Multiply(
            Multiply(new BigInteger[] { -stationaryScale, 15 * stationaryScale },
                new BigInteger[] { 1, 1 }), new BigInteger[] { 1, 0, 1 });
        for (int offset = -1; offset <= 1; offset++)
            AssertSign(polynomial, new BigInteger[] { -queryScale + offset, 15 * queryScale }, offset);
    }

    [Theory]
    [InlineData(1)]
    [InlineData(-1)]
    public void SignAtRoot_PreservesDyadicEqualityForNonlinearQueries(int querySign)
    {
        // P=(2x-3)(x²+1) has sole positive root 3/2. The quadratic
        // query 4x²-9 shares that root without being a multiple of P.
        BigInteger[] polynomial = { -3, 2, -3, 2 };
        for (int offset = -1; offset <= 1; offset++)
            AssertSign(polynomial,
                new BigInteger[] { querySign * (-9 + offset), 0, querySign * 4 },
                querySign * offset);
    }

    [Theory]
    [InlineData(64)]
    [InlineData(80)]
    public void SignAtRoot_PreservesLargeIrrationalEqualityAndUnitDifferences(int scaleExponent)
    {
        BigInteger radicand = BigInteger.One << (2 * scaleExponent + 1);
        // P=(x²-radicand)² selects 2^scaleExponent * sqrt(2). Its
        // quadratic factor is exactly zero there, independently of scale.
        // The nearby queries differ by one integer, not a tolerance.
        BigInteger[] polynomial = { radicand * radicand, 0, -2 * radicand, 0, 1 };
        AssertSign(polynomial, new BigInteger[] { -radicand, 0, 1 }, 0);
        AssertSign(polynomial, new BigInteger[] { -radicand + 1, 0, 1 }, 1);
        AssertSign(polynomial, new BigInteger[] { -radicand - 1, 0, 1 }, -1);
    }

    [Fact]
    public void SignAtRoot_RepeatedExactTiesKeepTheRetainedCellBoundedAndUsable()
    {
        Encode(new BigInteger[] { 4, 0, -4, 0, 1 }, out ulong[] polynomial, out sbyte[] signs);
        Encode(new BigInteger[] { -2, 0, 1 }, out ulong[] query, out sbyte[] querySigns);
        Span<ulong> cell = stackalloc ulong[WideFiniteAxisIntersection.GetFiniteAxisRootCellWords(polynomial, signs)];
        Assert.True(WideFiniteAxisIntersection.TryGetLargestPositiveFiniteAxisRoot(polynomial, signs, cell, out var root));
        // Equality cannot become a certified nonzero interval sign. Repeated
        // queries must stop refining at the capacity bound and remain exact.
        for (int repeat = 0; repeat < 8; repeat++)
            Assert.Equal(0, WideFiniteAxisIntersection.GetSignAtFiniteAxisRoot(ref root, query, querySigns));
        Encode(new BigInteger[] { -1, 1 }, out query, out querySigns);
        Assert.Equal(1, WideFiniteAxisIntersection.GetSignAtFiniteAxisRoot(ref root, query, querySigns));
        Encode(new BigInteger[] { -3, 2 }, out query, out querySigns);
        Assert.Equal(-1, WideFiniteAxisIntersection.GetSignAtFiniteAxisRoot(ref root, query, querySigns));
    }

    [Fact]
    public void SignAtRoot_ExcludesZeroFactorsAndComplexConjugates()
    {
        BigInteger[] polynomial = { 0, -2, 1, -2, 1 }; // x(x-2)(x²+1).
        AssertSign(polynomial, new BigInteger[] { -2, 1 }, 0);
        AssertSign(polynomial, new BigInteger[] { 1, 0, 1 }, 1);
    }

    [Fact]
    public void SignAtRoot_HandlesDegreeEightExactZeroAndAdjacentValues()
    {
        BigInteger[] polynomial = { -2, 0, 0, 0, 1 };
        AssertSign(polynomial, new BigInteger[] { -4, 0, 0, 0, 0, 0, 0, 0, 1 }, 0);
        AssertSign(polynomial, new BigInteger[] { -3, 0, 0, 0, 0, 0, 0, 0, 1 }, 1);
        AssertSign(polynomial, new BigInteger[] { -5, 0, 0, 0, 0, 0, 0, 0, 1 }, -1);
    }

    [Fact]
    public void SignAtRoot_DistinguishesVeryCloseRootsWithoutTolerance()
    {
        BigInteger scale = BigInteger.One << 200;
        // Roots 1 and 1+2^-200, independently known from the original factors.
        BigInteger[] polynomial = Multiply(new BigInteger[] { -1, 1 }, new BigInteger[] { -scale - 1, scale });
        AssertSign(polynomial, new BigInteger[] { -1, 1 }, 1);
        AssertSign(polynomial, new BigInteger[] { -scale - 1, scale }, 0);
        AssertSign(polynomial, new BigInteger[] { -2 * scale - 1, 2 * scale }, 1);
    }

    [Fact]
    public void SignAtRoot_HandlesLargeAndSmallPositiveRoots()
    {
        BigInteger scale = BigInteger.One << 130;
        AssertSign(new BigInteger[] { -scale, 1 }, new BigInteger[] { -scale, 1 }, 0);
        AssertSign(new BigInteger[] { -1, scale }, new BigInteger[] { -1, 2 * scale }, 1);
    }

    [Fact]
    public void SignAtRoot_PreservesFullStationaryCoefficientWidth()
    {
        BigInteger scale = (BigInteger.One << 2040) + 1;
        BigInteger[] polynomial = { -2 * scale, 0, scale };
        AssertSign(polynomial, new BigInteger[] { -2, 0, 1 }, 0);
        AssertSign(polynomial, new BigInteger[] { -1, 1 }, 1);
    }

    [Fact]
    public void SignAtRoot_CombinesFullQuarticAndAnalyticComparisonWidths()
    {
        BigInteger stationaryScale = (BigInteger.One << 2040) + 1;
        BigInteger queryScale = (BigInteger.One << 12900) + 3;
        BigInteger[] polynomial = { 6 * stationaryScale, 0, -5 * stationaryScale, 0, stationaryScale };
        // P=(x²-2)(x²-3) selects sqrt(3). Q=(x²-3)^4 shares that root
        // but is not a multiple of P: exact equality still needs root identity
        // after pseudo-division, at the largest admitted coefficient widths.
        AssertSign(polynomial,
            new BigInteger[] { 81 * queryScale, 0, -108 * queryScale, 0, 54 * queryScale, 0, -12 * queryScale, 0, queryScale }, 0);
    }

    [Fact]
    public void SignAtRoot_MatchesIndependentQuadraticFieldOracleAcrossRetainedQueries()
    {
        var random = new System.Random(236104);
        int sample = 0;
        foreach (int first in new[] { -2, 0, 2, 3, 5 })
        foreach (int second in new[] { -1, 2, 3, 7 })
        foreach (int leading in new[] { -1, 1 })
        {
            int largestSquared = Math.Max(first, second);
            if (largestSquared <= 0)
                continue;
            BigInteger[] polynomial = Multiply(
                new BigInteger[] { -first * leading, 0, leading },
                new BigInteger[] { -second, 0, 1 });
            Encode(polynomial, out ulong[] coefficients, out sbyte[] signs);
            var cell = new ulong[WideFiniteAxisIntersection.GetFiniteAxisRootCellWords(coefficients, signs)];
            Assert.True(WideFiniteAxisIntersection.TryGetLargestPositiveFiniteAxisRoot(coefficients, signs, cell, out var root));
            for (int degree = 0; degree <= 8; degree++)
            {
                var query = new BigInteger[degree + 1];
                BigInteger rational = 0;
                BigInteger radical = 0;
                for (int index = 0; index <= degree; index++)
                {
                    query[index] = random.Next(-4, 5);
                    BigInteger term = query[index] * BigInteger.Pow(largestSquared, index / 2);
                    if ((index & 1) == 0)
                        rational += term;
                    else
                        radical += term;
                }
                int expected = GetQuadraticFieldSign(rational, radical, largestSquared);
                Encode(query, out ulong[] queryCoefficients, out sbyte[] querySigns);
                int actual = WideFiniteAxisIntersection.GetSignAtFiniteAxisRoot(ref root, queryCoefficients, querySigns);
                Assert.True(expected == actual,
                    $"Quadratic-field sample {sample++}, factors {first}/{second}, degree {degree}: expected {expected}, actual {actual}.");
            }
        }
    }

    private static int GetQuadraticFieldSign(BigInteger rational, BigInteger radical, int radicand)
    {
        if (rational.IsZero)
            return radical.Sign;
        if (radical.IsZero || rational.Sign == radical.Sign)
            return rational.Sign;
        int comparison = (rational * rational).CompareTo(radical * radical * radicand);
        return comparison == 0 ? 0 : comparison > 0 ? rational.Sign : radical.Sign;
    }

    private static bool HasPositiveRoot(BigInteger[] polynomial)
    {
        Encode(polynomial, out ulong[] coefficients, out sbyte[] signs);
        Span<ulong> cell = stackalloc ulong[WideFiniteAxisIntersection.GetFiniteAxisRootCellWords(coefficients, signs)];
        return WideFiniteAxisIntersection.TryGetLargestPositiveFiniteAxisRoot(coefficients, signs, cell, out _);
    }

    private static void AssertSign(BigInteger[] polynomial, BigInteger[] query, int expected)
    {
        Encode(polynomial, out ulong[] coefficients, out sbyte[] signs);
        Encode(query, out ulong[] queryCoefficients, out sbyte[] querySigns);
        Span<ulong> cell = stackalloc ulong[WideFiniteAxisIntersection.GetFiniteAxisRootCellWords(coefficients, signs)];
        Assert.True(WideFiniteAxisIntersection.TryGetLargestPositiveFiniteAxisRoot(coefficients, signs, cell, out var root));
        Assert.Equal(expected, WideFiniteAxisIntersection.GetSignAtFiniteAxisRoot(ref root, queryCoefficients, querySigns));
    }

    private static void Encode(BigInteger[] polynomial, out ulong[] coefficients, out sbyte[] signs)
    {
        int words = 1;
        foreach (BigInteger coefficient in polynomial)
            words = Math.Max(words, (BigInteger.Abs(coefficient).GetByteCount(isUnsigned: true) + 7) / 8);
        coefficients = new ulong[words * polynomial.Length];
        signs = new sbyte[polynomial.Length];
        for (int index = 0; index < polynomial.Length; index++)
        {
            BigInteger magnitude = BigInteger.Abs(polynomial[index]);
            signs[index] = (sbyte)polynomial[index].Sign;
            for (int word = 0; word < words; word++)
            {
                coefficients[index * words + word] = (ulong)(magnitude & ulong.MaxValue);
                magnitude >>= 64;
            }
        }
    }

    private static BigInteger[] Multiply(BigInteger[] left, BigInteger[] right)
    {
        var result = new BigInteger[left.Length + right.Length - 1];
        for (int i = 0; i < left.Length; i++)
        for (int j = 0; j < right.Length; j++)
            result[i + j] += left[i] * right[j];
        return result;
    }
}
