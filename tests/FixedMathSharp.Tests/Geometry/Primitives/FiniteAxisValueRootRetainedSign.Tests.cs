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
