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
    [InlineData(12)]
    [InlineData(13)]
    [InlineData(14)]
    public void SquareRootBounds_EncloseExactFloorsWithoutMutatingBorrowedInputs(int geometry)
    {
        BigInteger scale = geometry == 9 ? BigInteger.One << 80 : BigInteger.One;
        BigInteger[] polynomial = geometry == 0 ? new BigInteger[] { -1, 1 }
            : geometry == 8 ? new BigInteger[] { -1, 0, 8 }
            : geometry == 14 ? new BigInteger[] { 4, 0, -4, 0, 1 }
            : new BigInteger[] { -2 * scale * scale, 0, 1 };
        BigInteger[] numerator = { 1, 0, 0 }, denominator = { 1, 0, 0 };
        ulong cap = 10, expectedLower = 1, expectedUpper = 1;
        switch (geometry)
        {
            case 1: numerator = new BigInteger[] { 0, 0, 1 }; expectedUpper = 2; break;
            // D(alpha)>0, but D's cell minimum is zero: retain the full range.
            case 2: denominator = new BigInteger[] { -1, 1, 0 }; expectedLower = 0; expectedUpper = cap; break;
            case 3: numerator = new BigInteger[] { -1, 1, 0 }; expectedLower = 0; break;
            case 4: numerator[0] = 4; cap = expectedLower = expectedUpper = 2; break;
            case 5: numerator = new BigInteger[] { 0, 0, 1 }; cap = expectedUpper = 2; break;
            case 6:
                // A two-word quotient after division of much wider operands.
                // Its exact square root is below the full unsigned cap.
                BigInteger content = (BigInteger.One << 240) + 3;
                numerator[0] = content * BigInteger.Pow((BigInteger)ulong.MaxValue - 1, 2);
                denominator[0] = content;
                cap = ulong.MaxValue; expectedLower = expectedUpper = cap - 1;
                break;
            case 7: numerator[0] = 0; expectedLower = expectedUpper = 0; break;
            case 9:
                // Negative denominator shift: alpha=2^80*sqrt(2)>1.
                numerator = new BigInteger[] { 0, 0, 1 }; denominator[0] = scale * scale;
                expectedUpper = 2; break;
            // (alpha-1)^2>=0, but dependency gives a negative interval minimum.
            case 10: numerator = new BigInteger[] { 1, -2, 1 }; expectedLower = 0; break;
            case 11: denominator = new BigInteger[] { 1, -2, 1 }; expectedLower = 0; expectedUpper = cap; break;
            case 12: numerator[0] = 0; cap = expectedLower = expectedUpper = 0; break;
            case 13:
            case 14:
                // Unpadded constants do not need the retained root's coordinates,
                // including a numerator wider than the entire constant buffer.
                numerator = denominator = new BigInteger[] { 1 }; break;
        }
        Encode(polynomial, out ulong[] coefficients, out sbyte[] signs);
        Encode(numerator, out ulong[] n, out sbyte[] ns);
        Encode(denominator, out ulong[] d, out sbyte[] ds);
        var cell = new ulong[WideFiniteAxisIntersection.GetFiniteAxisRootCellWords(coefficients, signs)];
        Assert.True(WideFiniteAxisIntersection.TryGetLargestPositiveFiniteAxisRoot(coefficients, signs, cell, out var root));
        if (geometry == 14)
        {
            // P=(x^2-2)^2 and Q=2^400*(x^2-2)+1 share no root;
            // Q(alpha)=1, while its nearby zero forces repeated refinement.
            BigInteger content = BigInteger.One << 400;
            Encode(new BigInteger[] { 1 - 2 * content, 0, content }, out ulong[] query, out sbyte[] querySigns);
            for (int attempt = 0; attempt < 3; attempt++)
                Assert.Equal(1, WideFiniteAxisIntersection.GetSignAtFiniteAxisRoot(ref root, query, querySigns));
            Assert.True(root.DenominatorShift > 128);
        }
        ulong[] originalCell = (ulong[])cell.Clone(), originalN = (ulong[])n.Clone(), originalD = (ulong[])d.Clone();
        int shift = root.DenominatorShift;
        WideFiniteAxisIntersection.GetFiniteAxisRootSquareRootBounds(root, n, ns, d, ds,
            cap, out ulong lower, out ulong upper);
        Assert.Equal(expectedLower, lower);
        Assert.Equal(expectedUpper, upper);
        Assert.Equal(shift, root.DenominatorShift);
        Assert.Equal(originalCell, cell);
        Assert.Equal(originalN, n);
        Assert.Equal(originalD, d);
    }

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
    public void ReducedRatio_PreservesSharedScaleAndQuadraticFieldValue(int geometry)
    {
        BigInteger[] polynomial = { -6, 0, -3, 0, 3 }; // 3(x²-2)(x²+1).
        BigInteger[] numerator = { 1, 0, 0, 0, 1 }, denominator = { 2, 0, 0, 0, 1 };
        if (geometry == 1)
            for (int index = 0; index < polynomial.Length; index++) polynomial[index] = -polynomial[index];
        if (geometry == 2) numerator = new BigInteger[] { 1, 0, 0, 0, 0 };
        if (geometry == 3)
        {
            polynomial = new BigInteger[] { -6, 0, 3 };
            numerator = new BigInteger[] { 0, 0, 5, 0, 0 };
            denominator = new BigInteger[] { 0, 0, 1, 0, 2 };
        }
        if (geometry == 4) numerator = new BigInteger[5];
        if (geometry == 5)
        {
            numerator = new BigInteger[] { 8, 8, 0, 0, 0 };
            denominator = new BigInteger[] { 16, 8, 0, 0, 0 };
        }
        if (geometry == 6)
        {
            polynomial = new BigInteger[] { -4, 0, 2 };
            numerator = new BigInteger[] { 0, 0, 0, 0, 1, 0, 1 };
            denominator = new BigInteger[] { 1, 0, 0, 0, 0, 0, 1 };
        }
        if (geometry == 7) polynomial = new BigInteger[] { 4, 0, -4, 0, 1 };
        if (geometry == 8)
        {
            // Different input widths plus a wide leading scale exercise the
            // paired carry budget without depending on the production geometry.
            for (int index = 0; index < polynomial.Length; index++) polynomial[index] *= (BigInteger.One << 160) + 1;
            for (int index = 0; index < numerator.Length; index++) numerator[index] *= (BigInteger.One << 240) + 3;
            for (int index = 0; index < denominator.Length; index++) denominator[index] *= (BigInteger.One << 128) + 1;
        }
        if (geometry == 9) { numerator = new BigInteger[] { 8 }; denominator = new BigInteger[] { 16 }; }
        Encode(polynomial, out ulong[] p, out sbyte[] ps);
        Encode(numerator, out ulong[] n, out sbyte[] ns);
        Encode(denominator, out ulong[] d, out sbyte[] ds);
        var cell = new ulong[WideFiniteAxisIntersection.GetFiniteAxisRootCellWords(p, ps)];
        Assert.True(WideFiniteAxisIntersection.TryGetLargestPositiveFiniteAxisRoot(p, ps, cell, out var root));
        ulong[] oldCell = (ulong[])cell.Clone(), oldN = (ulong[])n.Clone(), oldD = (ulong[])d.Clone();
        int words = WideFiniteAxisIntersection.GetFiniteAxisRootRatioWords(root, n, ns, d, ds,
            out int first, out int count);
        var pair = new ulong[2 * count * words];
        var signs = new sbyte[2 * count];
        Array.Fill(pair, ulong.MaxValue);
        Array.Fill(signs, (sbyte)99);
        int reducedCount = WideFiniteAxisIntersection.ReduceFiniteAxisRootRatio(root,
            n.AsSpan(first * (n.Length / ns.Length), count * (n.Length / ns.Length)), ns.AsSpan(first, count),
            d.AsSpan(first * (d.Length / ds.Length), count * (d.Length / ds.Length)), ds.AsSpan(first, count), pair, signs);
        Assert.InRange(reducedCount, 1, root.Signs.Length - 1);
        (BigInteger nr, BigInteger nx) = EvaluateQuadraticField(numerator);
        (BigInteger dr, BigInteger dx) = EvaluateQuadraticField(denominator);
        var reducedN = new BigInteger[reducedCount];
        var reducedD = new BigInteger[reducedCount];
        for (int index = 0; index < count; index++)
        {
            BigInteger a = 0, b = 0;
            for (int word = words - 1; word >= 0; word--)
            {
                a = (a << 64) + pair[index * words + word];
                b = (b << 64) + pair[(count + index) * words + word];
            }
            if (index < reducedCount)
            {
                reducedN[index] = a * signs[index];
                reducedD[index] = b * signs[count + index];
            }
            else { Assert.Equal(BigInteger.Zero, a); Assert.Equal(BigInteger.Zero, b); }
        }
        (BigInteger rr, BigInteger rx) = EvaluateQuadraticField(reducedN);
        (BigInteger sr, BigInteger sx) = EvaluateQuadraticField(reducedD);
        Assert.Equal(nr * sr + 2 * nx * sx, dr * rr + 2 * dx * rx);
        Assert.Equal(nr * sx + nx * sr, dr * rx + dx * rr);
        Assert.Equal(1, GetQuadraticFieldSign(sr, sx, 2));
        Assert.Equal(oldCell, cell); Assert.Equal(oldN, n); Assert.Equal(oldD, d);
    }

    private static (BigInteger Rational, BigInteger Radical) EvaluateQuadraticField(BigInteger[] coefficients)
    {
        BigInteger rational = 0, radical = 0;
        for (int index = 0; index < coefficients.Length; index++)
        {
            BigInteger term = coefficients[index] * BigInteger.Pow(2, index / 2);
            if ((index & 1) == 0) rational += term;
            else radical += term;
        }
        return (rational, radical);
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
