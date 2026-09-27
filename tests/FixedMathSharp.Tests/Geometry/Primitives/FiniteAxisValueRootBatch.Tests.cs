using System;
using System.Numerics;
using FixedMathSharp.Geometry;
using Xunit;

namespace FixedMathSharp.Tests;

public sealed class FiniteAxisValueRootBatchTests
{
    [Fact]
    public void EightRoots_UseEveryBatchSlotInExactOrdinalOrder()
    {
        BigInteger[] polynomial = { 1 };
        var expected = new (int Numerator, int Denominator)[8];
        for (int root = 1; root <= 8; root++)
        {
            var product = new BigInteger[polynomial.Length + 1];
            for (int index = 0; index < polynomial.Length; index++)
            {
                product[index] -= root * polynomial[index];
                product[index + 1] += 9 * polynomial[index];
            }
            polynomial = product;
            expected[root - 1] = (root, 9);
        }
        AssertBatchMatchesSingles(polynomial, 8, expected);
    }

    [Theory]
    [InlineData(1)]
    [InlineData(-1)]
    public void Batch_PreservesDistinctOrdinalCellsIncludingRepeatedAndEndpointRoots(int sign)
    {
        // x(3x-1)^2(2x-1)(x-1)(x-2), padded to degree eight.
        BigInteger[] polynomial = { 0, -2, 19, -67, 107, -75, 18, 0, 0 };
        for (int index = 0; index < polynomial.Length; index++)
            polynomial[index] *= sign;
        AssertBatchMatchesSingles(polynomial, 8, new[] { (1, 3), (1, 2), (1, 1) }, true);
    }

    [Fact]
    public void ClusteredRoots_RejectSmallBatchButExactFallbackAndLargerBatchAgree()
    {
        BigInteger scale = BigInteger.One << 100;
        BigInteger middle = scale >> 1;
        // Both roots lie above one half: straddling one half would let its
        // coarse dyadic boundary isolate them despite their tiny separation.
        BigInteger[] polynomial = { (middle + 1) * (middle + 3), -scale * (2 * middle + 4), scale * scale };
        Encode(polynomial, out ulong[] coefficients, out sbyte[] signs);
        Span<ulong> smallCells = stackalloc ulong[8];
        Span<int> shifts = stackalloc int[8];
        smallCells.Fill(ulong.MaxValue);
        Assert.False(WideFiniteAxisIntersection.TryGetFiniteValueRoots(coefficients, signs,
            smallCells, shifts, out int count, out _, out bool hasRepeatedRoots));
        Assert.Equal(2, count);
        Assert.False(hasRepeatedRoots);
        AssertBatchMatchesSingles(polynomial, 3, new[] { (middle + 1, scale), (middle + 3, scale) });
    }

    [Fact]
    public void CapacityFailure_AfterRationalRootRetainsTotalCountAndDiscardsPartialMask()
    {
        BigInteger scale = BigInteger.One << 100;
        BigInteger lower = 3 * (BigInteger.One << 98) + 1;
        BigInteger upper = lower + 2;
        BigInteger constant = lower * upper;
        BigInteger linear = -scale * (lower + upper);
        BigInteger quadratic = scale * scale;
        // (4x-1)(scale*x-lower)(scale*x-upper). The first root is
        // an exact singleton before the later pair exhausts compact storage.
        BigInteger[] polynomial =
        {
            -constant, 4 * constant - linear, 4 * linear - quadratic, 4 * quadratic
        };
        Encode(polynomial, out ulong[] coefficients, out sbyte[] signs);
        Span<ulong> cells = stackalloc ulong[8];
        Span<int> shifts = stackalloc int[8];
        Assert.False(WideFiniteAxisIntersection.TryGetFiniteValueRoots(coefficients, signs,
            cells, shifts, out int count, out int rationalMask, out bool hasRepeatedRoots));
        Assert.Equal(3, count);
        Assert.Equal(0, rationalMask);
        Assert.False(hasRepeatedRoots);
        AssertBatchMatchesSingles(polynomial, 3, new[]
        {
            (BigInteger.One, new BigInteger(4)), (lower, scale), (upper, scale)
        });
    }

    [Fact]
    public void PrecisionLimit_PreservesUpperEndpointCapacity()
    {
        BigInteger scale = BigInteger.One << 64;
        // The second root at one keeps the upper endpoint on a root until
        // the lower root is found exactly at shift64. One-word batching
        // deliberately stops before consuming its upper-endpoint carry bit.
        BigInteger[] polynomial = { scale - 1, 1 - 2 * scale, scale };
        Encode(polynomial, out ulong[] coefficients, out sbyte[] signs);
        Span<ulong> cells = stackalloc ulong[8];
        Span<int> shifts = stackalloc int[8];
        Assert.False(WideFiniteAxisIntersection.TryGetFiniteValueRoots(coefficients, signs,
            cells, shifts, out int count, out _, out bool hasRepeatedRoots));
        Assert.Equal(2, count);
        Assert.False(hasRepeatedRoots);
        AssertBatchMatchesSingles(polynomial, 2, new[] { (scale - 1, scale), (BigInteger.One, BigInteger.One) });
    }

    [Theory]
    [InlineData(0)]
    [InlineData(1)]
    [InlineData(-1)]
    public void ConstantPolynomial_HasSuccessfulEmptyBatch(int constant)
    {
        Encode(new BigInteger[] { constant, 0, 0 }, out ulong[] coefficients, out sbyte[] signs);
        Span<ulong> cells = stackalloc ulong[8];
        Span<int> shifts = stackalloc int[8];
        Assert.True(WideFiniteAxisIntersection.TryGetFiniteValueRoots(coefficients, signs,
            cells, shifts, out int count, out int rationalMask, out bool hasRepeatedRoots));
        Assert.Equal(0, count);
        Assert.Equal(0, rationalMask);
        Assert.False(hasRepeatedRoots);
    }

    [Fact]
    public void LinearPolynomial_HasNoRepeatedRoots()
    {
        AssertBatchMatchesSingles(new BigInteger[] { -1, 2, 0 }, 1, new[] { (1, 2) });
    }

    [Fact]
    public void RepeatedRootOutsidePhysicalInterval_IsReportedGlobally()
    {
        // (x-2)^2(2x-1): the only physical root is simple.
        AssertBatchMatchesSingles(new BigInteger[] { -4, 12, -9, 2 }, 1, new[] { (1, 2) }, true);
        // (x-2)^2: repeated-root metadata remains meaningful with no roots
        // at all in the physical interval (0,1].
        AssertBatchMatchesSingles(new BigInteger[] { 4, -4, 1 }, 1,
            Array.Empty<(int Numerator, int Denominator)>(), true);
    }

    [Fact]
    public void CapacityFailure_PreservesGlobalRepeatedRootFlag()
    {
        BigInteger scale = BigInteger.One << 100;
        BigInteger lower = (scale >> 1) + 1;
        BigInteger upper = lower + 2;
        BigInteger constant = lower * upper;
        BigInteger linear = -scale * (lower + upper);
        BigInteger quadratic = scale * scale;
        // The clustered physical roots are simple, but the additional
        // factor (x-2)^2 requires global repeated-root handling on fallback.
        BigInteger[] polynomial =
        {
            4 * constant, 4 * linear - 4 * constant,
            4 * quadratic - 4 * linear + constant, linear - 4 * quadratic, quadratic
        };
        Encode(polynomial, out ulong[] coefficients, out sbyte[] signs);
        Span<ulong> cells = stackalloc ulong[8];
        Span<int> shifts = stackalloc int[8];
        Assert.False(WideFiniteAxisIntersection.TryGetFiniteValueRoots(coefficients, signs,
            cells, shifts, out int count, out int rationalMask, out bool hasRepeatedRoots));
        Assert.Equal(2, count);
        Assert.Equal(0, rationalMask);
        Assert.True(hasRepeatedRoots);
        AssertBatchMatchesSingles(polynomial, 3, new[] { (lower, scale), (upper, scale) }, true);
    }

    [Fact]
    public void RootOwner_MaterializesTrimmedIndependentCellsAndClearsDirtyTail()
    {
        // (3x-1)(2x-1), with authored degree padding.
        Encode(new BigInteger[] { 1, -5, 6, 0, 0 }, out ulong[] coefficients, out sbyte[] signs);
        ulong[] cells = new ulong[64];
        int[] shifts = new int[8];
        FiniteAxisValueRoots roots = WideFiniteAxisIntersection.GetFiniteValueRoots(coefficients, signs, cells, shifts);
        Assert.Equal(2, roots.Count);
        Assert.True(roots.HasCompactCells);
        Assert.False(roots.HasRepeatedRoots);
        ulong[] savedCells = (ulong[])cells.Clone();
        int[] savedShifts = (int[])shifts.Clone();
        ulong[] fullCell = new ulong[WideFiniteAxisIntersection.GetFiniteValueRootCellWords(coefficients, signs)];
        Array.Fill(fullCell, ulong.MaxValue);
        FiniteAxisValueRoot first = FiniteAxisValueRoots.GetRoot(roots, 0, coefficients, signs, fullCell);
        Assert.Equal(3, first.Signs.Length);
        Assert.Equal(3, first.Coefficients.Length);
        Assert.Equal(0, first.Ordinal);
        AssertEnclosure(Decode(first.LowerNumerator), first.DenominatorShift, first.IsRational, (1, 3));
        WideFiniteAxisIntersection.RefineFiniteValueRoot(ref first, 80);
        AssertEnclosure(Decode(first.LowerNumerator), first.DenominatorShift, first.IsRational, (1, 3));
        Assert.Equal(savedCells, cells);
        Assert.Equal(savedShifts, shifts);
        Array.Fill(fullCell, ulong.MaxValue);
        FiniteAxisValueRoot second = FiniteAxisValueRoots.GetRoot(roots, 1, coefficients, signs, fullCell);
        Assert.Equal(1, second.Ordinal);
        AssertEnclosure(Decode(second.LowerNumerator), second.DenominatorShift, second.IsRational, (1, 2));
        // Active-word copy also permits a destination shorter than the eight-word cache slot.
        Span<ulong> oneWord = stackalloc ulong[1];
        FiniteAxisValueRoot small = FiniteAxisValueRoots.GetRoot(roots, 0, coefficients, signs, oneWord);
        AssertEnclosure(Decode(small.LowerNumerator), small.DenominatorShift, small.IsRational, (1, 3));
    }

    [Fact]
    public void RootOwner_CapacityFallbackConstructsEveryAuthoritativeOrdinal()
    {
        BigInteger scale = BigInteger.One << 100;
        BigInteger lower = 3 * (BigInteger.One << 98) + 1;
        BigInteger upper = lower + 2;
        BigInteger constant = lower * upper;
        BigInteger linear = -scale * (lower + upper);
        BigInteger quadratic = scale * scale;
        Encode(new[] { -constant, 4 * constant - linear, 4 * linear - quadratic, 4 * quadratic },
            out ulong[] coefficients, out sbyte[] signs);
        Span<ulong> cells = stackalloc ulong[8];
        Span<int> shifts = stackalloc int[8];
        FiniteAxisValueRoots roots = WideFiniteAxisIntersection.GetFiniteValueRoots(coefficients, signs, cells, shifts);
        Assert.Equal(3, roots.Count);
        Assert.False(roots.HasCompactCells);
        Assert.False(roots.HasRepeatedRoots);
        ulong[] fullCell = new ulong[WideFiniteAxisIntersection.GetFiniteValueRootCellWords(coefficients, signs)];
        var expected = new[] { (BigInteger.One, new BigInteger(4)), (lower, scale), (upper, scale) };
        for (int ordinal = 0; ordinal < roots.Count; ordinal++)
        {
            Array.Fill(fullCell, ulong.MaxValue);
            FiniteAxisValueRoot root = FiniteAxisValueRoots.GetRoot(roots, ordinal, coefficients, signs, fullCell);
            Assert.Equal(ordinal, root.Ordinal);
            AssertEnclosure(Decode(root.LowerNumerator), root.DenominatorShift, root.IsRational, expected[ordinal]);
        }
    }

    private static void AssertBatchMatchesSingles(BigInteger[] polynomial, int words,
        (int Numerator, int Denominator)[] expected, bool expectedRepeatedRoots = false)
    {
        var converted = new (BigInteger Numerator, BigInteger Denominator)[expected.Length];
        for (int index = 0; index < expected.Length; index++)
            converted[index] = expected[index];
        AssertBatchMatchesSingles(polynomial, words, converted, expectedRepeatedRoots);
    }

    private static void AssertBatchMatchesSingles(BigInteger[] polynomial, int words,
        (BigInteger Numerator, BigInteger Denominator)[] expected, bool expectedRepeatedRoots = false)
    {
        Encode(polynomial, out ulong[] coefficients, out sbyte[] signs);
        ulong[] originalCoefficients = (ulong[])coefficients.Clone();
        sbyte[] originalSigns = (sbyte[])signs.Clone();
        ulong[] cells = new ulong[8 * words];
        int[] shifts = new int[8];
        Array.Fill(cells, ulong.MaxValue);
        Assert.True(WideFiniteAxisIntersection.TryGetFiniteValueRoots(coefficients, signs,
            cells, shifts, out int count, out int rationalMask, out bool hasRepeatedRoots));
        Assert.Equal(expected.Length, count);
        Assert.Equal(expectedRepeatedRoots, hasRepeatedRoots);
        ulong[] singleCell = new ulong[WideFiniteAxisIntersection.GetFiniteValueRootCellWords(coefficients, signs)];
        for (int ordinal = 0; ordinal < count; ordinal++)
        {
            Assert.True(WideFiniteAxisIntersection.TryGetFiniteValueRoot(coefficients, signs,
                ordinal, singleCell, out FiniteAxisValueRoot single));
            BigInteger lower = Decode(cells.AsSpan(ordinal * words, words));
            bool rational = (rationalMask & (1 << ordinal)) != 0;
            AssertEnclosure(lower, shifts[ordinal], rational, expected[ordinal]);
            AssertEnclosure(Decode(single.LowerNumerator), single.DenominatorShift,
                single.IsRational, expected[ordinal]);
            if (!rational)
                Assert.True(lower + 1 < (BigInteger.One << (words * 64)));
        }
        Assert.False(WideFiniteAxisIntersection.TryGetFiniteValueRoot(coefficients, signs,
            count, singleCell, out _));
        Assert.Equal(originalCoefficients, coefficients);
        Assert.Equal(originalSigns, signs);
    }

    private static void AssertEnclosure(BigInteger lower, int shift, bool rational,
        (BigInteger Numerator, BigInteger Denominator) expected)
    {
        BigInteger scaledExpected = expected.Numerator << shift;
        BigInteger scaledLower = lower * expected.Denominator;
        if (rational)
            Assert.Equal(scaledExpected, scaledLower);
        else
        {
            Assert.True(lower > 0);
            Assert.True(scaledLower < scaledExpected);
            Assert.True(scaledExpected < scaledLower + expected.Denominator);
        }
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
