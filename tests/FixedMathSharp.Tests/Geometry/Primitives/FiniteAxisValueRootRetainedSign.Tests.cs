using System;
using System.Numerics;
using FixedMathSharp.Geometry;
using Xunit;

namespace FixedMathSharp.Tests;

public sealed class FiniteAxisValueRootRetainedSignTests
{
    [Fact]
    public void PreservingSign_LeavesTheCellAndItsMetadataUnchanged()
    {
        ulong[] cell = new ulong[16];
        Assert.True(WideFiniteAxisIntersection.TryGetFiniteValueRoot(new ulong[] { 3, 8 },
            new sbyte[] { -1, 1 }, 0, cell, out FiniteAxisValueRoot root));
        Assert.False(root.IsRational);
        ulong[] original = (ulong[])cell.Clone();
        int shift = root.DenominatorShift;
        Assert.Equal(0, WideFiniteAxisIntersection.GetSignAtFiniteValueRoot(root,
            new ulong[] { 3, 8 }, new sbyte[] { -1, 1 }));
        Assert.Equal(original, cell);
        Assert.Equal(shift, root.DenominatorShift);
        Assert.False(root.IsRational);
    }

    [Fact]
    public void MutableSigns_RetainCertifiedCellsAcrossAQuerySequence()
    {
        ulong[] defining = { 1, 0, 2 };
        sbyte[] definingSigns = { -1, 0, 1 };
        ulong[] cell = new ulong[16];
        Assert.True(WideFiniteAxisIntersection.TryGetFiniteValueRoot(defining, definingSigns,
            0, cell, out FiniteAxisValueRoot root));
        int initialShift = root.DenominatorShift;
        BigInteger[][] queries = { new BigInteger[] { -3, 4 }, new BigInteger[] { -2, 3 },
            new BigInteger[] { -1, 0, 2 } };
        int[] expected = { -1, 1, 0 };
        for (int index = 0; index < queries.Length; index++)
        {
            Encode(queries[index], out ulong[] values, out sbyte[] signs);
            int previousShift = root.DenominatorShift;
            Assert.Equal(expected[index], WideFiniteAxisIntersection.GetSignAtFiniteValueRoot(root, values, signs));
            Assert.Equal(expected[index], WideFiniteAxisIntersection.GetSignAtFiniteValueRootAndRefine(
                ref root, values, signs));
            Assert.True(root.DenominatorShift >= previousShift);
            AssertSquareRootCell(root);
        }
        Assert.True(root.DenominatorShift > initialShift);
        Assert.Equal(0, root.Ordinal);
        Assert.True(root.Coefficients.SequenceEqual(defining));
        Assert.True(root.Signs.SequenceEqual(definingSigns));
    }

    [Fact]
    public void MutableSign_RetainsAnExactlyDiscoveredDyadicRoot()
    {
        ulong[] cell = new ulong[1];
        Assert.True(WideFiniteAxisIntersection.TryGetFiniteValueRoot(new ulong[] { 3, 8 },
            new sbyte[] { -1, 1 }, 0, cell, out FiniteAxisValueRoot root));
        Assert.False(root.IsRational);
        Assert.Equal(0, WideFiniteAxisIntersection.GetSignAtFiniteValueRootAndRefine(ref root,
            new ulong[] { 3, 8 }, new sbyte[] { -1, 1 }));
        Assert.True(root.IsRational);
        Assert.Equal(3, root.DenominatorShift);
        Assert.Equal(3UL, cell[0]);
        Assert.Equal(-1, WideFiniteAxisIntersection.GetSignAtFiniteValueRootAndRefine(ref root,
            new ulong[] { 1, 2 }, new sbyte[] { -1, 1 }));
    }

    [Theory]
    [InlineData(1, 1, 1)]
    [InlineData(-1, 1, 1)]
    [InlineData(1, 16, 1)]
    [InlineData(-1, 16, 1)]
    [InlineData(1, 1, 129)]
    [InlineData(-1, 16, 129)]
    public void MutableSign_KnownCrossingPreservesExactQueriesAndRetainedCells(int orientation, int words, int power)
    {
        Encode(new[] { new BigInteger(-orientation), BigInteger.Zero, orientation * (BigInteger.One << power) },
            out ulong[] defining, out sbyte[] definingSigns);
        var ordinaryCell = new ulong[words];
        var knownCell = new ulong[words];
        Assert.True(WideFiniteAxisIntersection.TryGetFiniteValueRoot(defining, definingSigns,
            0, ordinaryCell, out FiniteAxisValueRoot ordinary));
        Assert.True(WideFiniteAxisIntersection.TryGetFiniteValueRoot(defining, definingSigns,
            0, knownCell, out FiniteAxisValueRoot known));
        BigInteger scale = (BigInteger.One << 256) + 17;
        for (int offset = -1; offset <= 1; offset++)
        {
            // At sqrt(2^-power), each query is exactly offset. The tiny
            // nonzero cases force refinement beyond a small caller cell.
            Encode(new[] { -scale + offset, BigInteger.Zero, scale << power },
                out ulong[] values, out sbyte[] signs);
            Assert.Equal(offset, WideFiniteAxisIntersection.GetSignAtFiniteValueRoot(known, values, signs));
            Assert.Equal(offset, WideFiniteAxisIntersection.GetSignAtFiniteValueRootAndRefine(ref ordinary, values, signs));
            Assert.Equal(offset, WideFiniteAxisIntersection.GetSignAtFiniteValueRootAndRefine(
                ref known, values, signs, -orientation));
            Assert.Equal(ordinaryCell, knownCell);
            Assert.Equal(ordinary.DenominatorShift, known.DenominatorShift);
            Assert.Equal(ordinary.IsRational, known.IsRational);
            Assert.Equal(ordinary.Ordinal, known.Ordinal);
            AssertSquareRootCell(known, power);
        }
    }

    [Theory]
    [InlineData(1)]
    [InlineData(-1)]
    public void MutableSign_KnownCrossingExcludesZeroAndOtherRootEndpoints(int orientation)
    {
        // F=x(2x-1)(3x-2): zero is excluded, 1/2 is a different root,
        // and the selected 2/3 cell must have two nonzero endpoints.
        Encode(new BigInteger[] { 0, 2 * orientation, -7 * orientation, 6 * orientation },
            out ulong[] defining, out sbyte[] definingSigns);
        var ordinaryCell = new ulong[16];
        var knownCell = new ulong[16];
        Assert.True(WideFiniteAxisIntersection.TryGetFiniteValueRoot(defining, definingSigns,
            1, ordinaryCell, out FiniteAxisValueRoot ordinary));
        Assert.True(WideFiniteAxisIntersection.TryGetFiniteValueRoot(defining, definingSigns,
            1, knownCell, out FiniteAxisValueRoot known));
        Assert.False(known.IsRational);
        Assert.Equal(-orientation, WideFiniteAxisIntersection.EvaluateFiniteRootPolynomial(
            defining, definingSigns, known.LowerNumerator, known.DenominatorShift, 0));
        ulong[] upper = (ulong[])knownCell.Clone();
        WideArithmetic.AddWord(upper, 0, 1);
        Assert.Equal(orientation, WideFiniteAxisIntersection.EvaluateFiniteRootPolynomial(
            defining, definingSigns, upper, known.DenominatorShift, 0));
        BigInteger scale = (BigInteger.One << 256) + 17;
        for (int offset = -1; offset <= 1; offset++)
        {
            Encode(new[] { -2 * scale + offset, 3 * scale }, out ulong[] values, out sbyte[] signs);
            Assert.Equal(offset, WideFiniteAxisIntersection.GetSignAtFiniteValueRootAndRefine(ref ordinary, values, signs));
            Assert.Equal(offset, WideFiniteAxisIntersection.GetSignAtFiniteValueRootAndRefine(ref known, values, signs, -orientation));
            Assert.Equal(ordinaryCell, knownCell);
            Assert.Equal(ordinary.DenominatorShift, known.DenominatorShift);
            Assert.Equal(ordinary.IsRational, known.IsRational);
            Assert.Equal(1, known.Ordinal);
        }
    }

    [Theory]
    [InlineData(1)]
    [InlineData(-1)]
    public void MutableSign_KnownCrossingRetainsDyadicDiscoveryAndSubsequentQueries(int orientation)
    {
        var cell = new ulong[1];
        ulong[] defining = { 3, 8 };
        sbyte[] definingSigns = { (sbyte)-orientation, (sbyte)orientation };
        Assert.True(WideFiniteAxisIntersection.TryGetFiniteValueRoot(defining, definingSigns,
            0, cell, out FiniteAxisValueRoot root));
        Assert.False(root.IsRational);
        Assert.Equal(0, WideFiniteAxisIntersection.GetSignAtFiniteValueRootAndRefine(
            ref root, defining, definingSigns, -orientation));
        Assert.True(root.IsRational);
        Assert.Equal(3, root.DenominatorShift);
        Assert.Equal(3UL, cell[0]);
        Assert.Equal(-1, WideFiniteAxisIntersection.GetSignAtFiniteValueRootAndRefine(ref root,
            new ulong[] { 1, 2 }, new sbyte[] { -1, 1 }, -orientation));
    }

    [Theory]
    [InlineData(-1)]
    [InlineData(0)]
    [InlineData(1)]
    public void MutableSign_UsesLocalOverflowPrecisionWhenTheCallerCellIsSmall(int offset)
    {
        ulong[] cell = new ulong[1];
        Assert.True(WideFiniteAxisIntersection.TryGetFiniteValueRoot(new ulong[] { 1, 0, 2 },
            new sbyte[] { -1, 0, 1 }, 0, cell, out FiniteAxisValueRoot root));
        int initialShift = root.DenominatorShift;
        BigInteger scale = (BigInteger.One << 256) + 17;
        Encode(new[] { -scale + offset, BigInteger.Zero, 2 * scale }, out ulong[] values, out sbyte[] signs);
        Assert.Equal(offset, WideFiniteAxisIntersection.GetSignAtFiniteValueRootAndRefine(ref root, values, signs));
        Assert.InRange(root.DenominatorShift, initialShift + 1, 64);
        AssertSquareRootCell(root);
        Assert.Equal(offset, WideFiniteAxisIntersection.GetSignAtFiniteValueRoot(root, values, signs));
        Assert.Equal(-1, WideFiniteAxisIntersection.GetSignAtFiniteValueRootAndRefine(ref root,
            new ulong[] { 3, 4 }, new sbyte[] { -1, 1 }));
        AssertSquareRootCell(root);
    }

    [Fact]
    public void MutableSign_PreservesTinyRootCellsWhoseShiftExceedsNumeratorCapacity()
    {
        // alpha=2^(-129/2) lies strictly between2^-65 and2^-64.
        // Its initial lower numerator is one even though the shift is65.
        Encode(new[] { -BigInteger.One, BigInteger.Zero, BigInteger.One << 129 },
            out ulong[] defining, out sbyte[] definingSigns);
        ulong[] cell = new ulong[1];
        Assert.True(WideFiniteAxisIntersection.TryGetFiniteValueRoot(defining, definingSigns,
            0, cell, out FiniteAxisValueRoot root));
        Assert.Equal(65, root.DenominatorShift);
        Assert.Equal(1UL, cell[0]);
        AssertSquareRootCell(root, 129);

        // 2^65*alpha-1=sqrt(2)-1>0. Resolving the normalized query
        // needs more precision than the initial tiny cell, not more storage
        // merely because the denominator shift already exceeds64.
        Encode(new[] { -BigInteger.One, BigInteger.One << 65 }, out ulong[] values, out sbyte[] signs);
        Assert.Equal(1, WideFiniteAxisIntersection.GetSignAtFiniteValueRootAndRefine(ref root, values, signs));
        Assert.True(root.DenominatorShift > 65);
        AssertSquareRootCell(root, 129);
        Assert.Equal(0, root.Ordinal);
        Assert.True(root.Coefficients.SequenceEqual(defining));
        Assert.True(root.Signs.SequenceEqual(definingSigns));
        int retainedShift = root.DenominatorShift;
        ulong retainedNumerator = cell[0];
        Assert.Equal(1, WideFiniteAxisIntersection.GetSignAtFiniteValueRoot(root, values, signs));
        Assert.Equal(retainedShift, root.DenominatorShift);
        Assert.Equal(retainedNumerator, cell[0]);
    }

    [Theory]
    [InlineData(65, false)]
    [InlineData(257, false)]
    [InlineData(65, true)]
    public void TinyRootSeparatedSign_UsesRelativeCellPrecision(int initialShift, bool zeroLower)
    {
        // alpha=2^(-(2q-1)/2), so 2^q*alpha-1=sqrt(2)-1>0.
        // Scaling the same separated geometry must not require q additional
        // bisections just because the linear coefficient has q more bits.
        int power = 2 * initialShift - 1;
        Encode(new[] { -BigInteger.One, BigInteger.Zero, BigInteger.One << power },
            out ulong[] defining, out sbyte[] definingSigns);
        ulong[] cell = new ulong[16];
        Assert.True(WideFiniteAxisIntersection.TryGetFiniteValueRoot(defining, definingSigns,
            0, cell, out FiniteAxisValueRoot root));
        Assert.Equal(initialShift, root.DenominatorShift);
        if (zeroLower)
        {
            // The wider cell (0,2^-(q-1)) still isolates this positive root.
            // Its zero numerator must not be mistaken for a zero root.
            cell.AsSpan().Clear();
            root.DenominatorShift--;
            AssertSquareRootCell(root, power);
        }
        Encode(new[] { -BigInteger.One, BigInteger.One << initialShift },
            out ulong[] query, out sbyte[] querySigns);
        Assert.Equal(1, WideFiniteAxisIntersection.GetSignAtFiniteValueRootAndRefine(ref root, query, querySigns));
        Assert.InRange(root.DenominatorShift, initialShift + 1, initialShift + 40);
        AssertSquareRootCell(root, power);
    }

    [Theory]
    [InlineData(-1)]
    [InlineData(0)]
    [InlineData(1)]
    public void TinyRootCancellation_PreservesLowCoefficientBits(int offset)
    {
        const int power = 129;
        Encode(new[] { -BigInteger.One, BigInteger.Zero, BigInteger.One << power },
            out ulong[] defining, out sbyte[] definingSigns);
        ulong[] cell = new ulong[16];
        Assert.True(WideFiniteAxisIntersection.TryGetFiniteValueRoot(defining, definingSigns,
            0, cell, out FiniteAxisValueRoot root));
        BigInteger scale = (BigInteger.One << 193) + 17;
        // The large terms cancel exactly at alpha. Neither discarded low
        // bits nor a negative effective normalization may turn uncertainty
        // into equality before the certified nonzero bound is reached.
        Encode(new[] { -scale + offset, BigInteger.Zero, scale << power },
            out ulong[] query, out sbyte[] querySigns);
        ulong[] originalCell = (ulong[])cell.Clone();
        ulong[] originalQuery = (ulong[])query.Clone();
        int initialShift = root.DenominatorShift;
        Assert.Equal(offset, WideFiniteAxisIntersection.GetSignAtFiniteValueRoot(root, query, querySigns));
        Assert.Equal(originalCell, cell);
        Assert.Equal(initialShift, root.DenominatorShift);
        Assert.Equal(offset, WideFiniteAxisIntersection.GetSignAtFiniteValueRootAndRefine(ref root, query, querySigns));
        Assert.Equal(originalQuery, query);
        AssertSquareRootCell(root, power);
    }

    [Fact]
    public void TinyRootQueries_HandleNegativeHeightAndCompletelyDiscardedCoefficients()
    {
        const int power = 129;
        Encode(new[] { -BigInteger.One, BigInteger.Zero, BigInteger.One << power },
            out ulong[] defining, out sbyte[] definingSigns);
        ulong[] cell = new ulong[16];
        Assert.True(WideFiniteAxisIntersection.TryGetFiniteValueRoot(defining, definingSigns,
            0, cell, out FiniteAxisValueRoot root));
        BigInteger[][] queries =
        {
            new BigInteger[] { 0, 0, 1 }, // Q(alpha)=2^-129, with negative effective height.
            new[] { BigInteger.One, BigInteger.Zero, BigInteger.Zero, BigInteger.Zero, BigInteger.One << power },
            new BigInteger[] { 1, 0, 1 } // Constant-dominated query keeps the original normalization.
        };
        foreach (BigInteger[] query in queries)
        {
            Encode(query, out ulong[] values, out sbyte[] signs);
            Assert.Equal(1, WideFiniteAxisIntersection.GetSignAtFiniteValueRootAndRefine(ref root, values, signs));
            AssertSquareRootCell(root, power);
        }
    }

    private static void AssertSquareRootCell(FiniteAxisValueRoot root, int definingPower = 1)
    {
        Assert.False(root.IsRational);
        BigInteger numerator = 0;
        for (int index = root.LowerNumerator.Length - 1; index >= 0; index--)
            numerator = (numerator << 64) + root.LowerNumerator[index];
        BigInteger scaleSquared = BigInteger.One << (2 * root.DenominatorShift);
        Assert.True((numerator * numerator << definingPower) < scaleSquared);
        Assert.True(scaleSquared < ((numerator + 1) * (numerator + 1) << definingPower));
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
