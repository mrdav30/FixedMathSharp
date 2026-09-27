using System;
using System.Numerics;
using FixedMathSharp.Geometry;
using Xunit;

namespace FixedMathSharp.Tests;

public sealed class FiniteAxisValueRootRadicalOffsetTests
{
    [Theory]
    [InlineData(1, 1)]
    [InlineData(-1, 0)]
    public void Map_SelectsTheSignedBranchBetweenRationalRoots(int sign, int expectedOrdinal) =>
        AssertMapping(new BigInteger[] { -1, 64 }, sign, 1, 4,
            new BigInteger[] { 9, -640, 4096 }, expectedOrdinal);

    [Theory]
    [InlineData(1, 1)]
    [InlineData(-1, 0)]
    public void Map_IdentifiesTheIrrationalOffsetOfASingletonSource(int sign, int expectedOrdinal) =>
        // alpha=1/16 and tau=1/8 give beta=3/16 +/- sqrt(2)/8.
        // Both are roots of 256*beta²-96*beta+1; alpha<tau admits either sign.
        AssertMapping(new BigInteger[] { -1, 16 }, sign, 1, 3,
            new BigInteger[] { 1, -96, 256 }, expectedOrdinal);

    [Theory]
    [InlineData(1, 1)]
    [InlineData(-1, 0)]
    public void Map_MapsNonDyadicInputToTheCorrectIrrationalConjugate(int sign, int expectedOrdinal) =>
        AssertMapping(new BigInteger[] { -1, 48 }, sign, 1, 4,
            new BigInteger[] { 1, -96, 576 }, expectedOrdinal);

    [Theory]
    [InlineData(1, 1)]
    [InlineData(-1, 0)]
    public void Map_MapsAnIrrationalSourceRootWithoutFormingAHighDegreeSignQuery(int sign, int expectedOrdinal) =>
        AssertMapping(new BigInteger[] { -1, 0, 128 }, sign, 1, 2,
            new BigInteger[] { 49, -1408, 5888, -16384, 16384 }, expectedOrdinal);

    [Fact]
    public void Map_PreservesTheSeparationFallbackForAnIrrationalSourceAndRationalTarget() =>
        // alpha=(3-2*sqrt(2))/8, radius²=1/8: the positive offset is 1/4.
        // The target (4t-1)(2t-1) puts 1/4 before a later root, so the
        // mapper must compare cells rather than return the last ordinal.
        // Its singleton cannot contain the irrational source's mapped interval.
        AssertMapping(new BigInteger[] { 1, -48, 64 }, 1, 1, 3,
            new BigInteger[] { 1, -6, 8 }, 0);

    [Fact]
    public void Map_ClipsANegativeBranchCellAtItsMinimum() =>
        // The initial source cell (1/8,1/4) ends at tau. Only the positive
        // full-gap branch (1/2-sqrt(1/5))² is admitted by this contract.
        AssertMapping(new BigInteger[] { -1, 5 }, -1, 1, 2,
            new BigInteger[] { 1, -360, 400 }, 0);

    [Fact]
    public void Map_PreservesVerySmallPositiveSourceAndTargetValues()
    {
        BigInteger denominator = BigInteger.One << 120;
        // alpha=2^-120, tau=4*2^-120. Negative branch beta=2^-120.
        AssertMapping(new BigInteger[] { -1, denominator }, -1, 4, 120,
            new BigInteger[] { 9, -10 * denominator, denominator * denominator }, 0);
    }

    [Fact]
    public void Map_PreservesAZeroRadiusOffset() =>
        AssertMapping(new BigInteger[] { -1, 3 }, 1, 0, 7,
            new BigInteger[] { 1, -6, 9 }, 0, 32);

    [Theory]
    [InlineData(1, 1)]
    [InlineData(-1, 0)]
    public void Map_CertifiesAnOrdinaryIrrationalRootWithoutSeparationPrecision(int sign, int expectedOrdinal)
    {
        Encode(new BigInteger[] { -1, 48 }, out ulong[] source, out sbyte[] sourceSigns);
        Encode(new BigInteger[] { 1, -96, 576 }, out ulong[] target, out sbyte[] targetSigns);
        ulong[] sourceCell = new ulong[WideFiniteAxisIntersection.GetFiniteValueRootCellWords(source, sourceSigns)];
        ulong[] targetCell = new ulong[WideFiniteAxisIntersection.GetFiniteValueRootCellWords(target, targetSigns)];
        Assert.True(WideFiniteAxisIntersection.TryGetFiniteValueRoot(source, sourceSigns, 0, sourceCell, out FiniteAxisValueRoot root));
        FiniteAxisValueRoot mapped = WideFiniteAxisIntersection.MapKnownFiniteValueRootRadicalOffset(root, sign, new ulong[] { 1 }, 4,
            target, targetSigns, targetCell);
        Assert.Equal(expectedOrdinal, mapped.Ordinal);
        Assert.False(mapped.IsRational);
        // These well-separated roots require only containment, not the
        // hundreds of bits needed by the worst-case root-separation bound.
        Assert.InRange(mapped.DenominatorShift, 0, 32);
    }

    [Fact]
    public void Map_RefinesOverlappingCellsToRejectAVeryCloseWrongOrdinal()
    {
        BigInteger scale = BigInteger.One << 60;
        AssertMapping(new BigInteger[] { -1, 3 }, 1, 0, 7,
            new BigInteger[] { scale - 1, -3 * (2 * scale - 1), 9 * scale }, 1);
    }

    [Fact]
    public void Map_PreservesExactMembershipBeyondCompactBatchPrecision()
    {
        BigInteger scale = BigInteger.One << 600;
        // (3x-1)(scale*(3x-1)-1): both roots require more than the
        // compact batch's 511 bits to isolate. The zero-radius map is the
        // identity, so its exact target is the lower root at one third.
        AssertMapping(new BigInteger[] { -1, 3 }, 1, 0, 0,
            new BigInteger[] { scale + 1, -6 * scale - 3, 9 * scale }, 0);
    }

    [Fact]
    public void Map_SelectsKnownNineOverSixtyFourBesideOneSeventh()
    {
        // alpha=1/16, radius=1/8: beta=(1/4+1/8)^2=9/64.
        // Target (64x-9)(7x-1) also contains the nearby root1/7.
        // Membership is a precondition, not something this mapper validates.
        Encode(new BigInteger[] { -1, 16 }, out ulong[] source, out sbyte[] sourceSigns);
        Encode(new BigInteger[] { 9, -127, 448 }, out ulong[] target, out sbyte[] targetSigns);
        ulong[] sourceCell = new ulong[WideFiniteAxisIntersection.GetFiniteValueRootCellWords(source, sourceSigns)];
        ulong[] targetCell = new ulong[WideFiniteAxisIntersection.GetFiniteValueRootCellWords(target, targetSigns)];
        Assert.True(WideFiniteAxisIntersection.TryGetFiniteValueRoot(source, sourceSigns, 0, sourceCell, out FiniteAxisValueRoot root));
        FiniteAxisValueRoot mapped = WideFiniteAxisIntersection.MapKnownFiniteValueRootRadicalOffset(root, 1, new ulong[] { 1 }, 6,
            target, targetSigns, targetCell);
        Assert.Equal(0, mapped.Ordinal);
        BigInteger lower = 0;
        for (int index = mapped.LowerNumerator.Length - 1; index >= 0; index--)
            lower = (lower << 64) + mapped.LowerNumerator[index];
        BigInteger expected = new BigInteger(9) << mapped.DenominatorShift;
        if (mapped.IsRational)
            Assert.Equal(expected, 64 * lower);
        else
        {
            Assert.True(64 * lower < expected);
            Assert.True(expected < 64 * (lower + 1));
        }
    }

    [Fact]
    public void Map_AllocatesNothingWithCallerOwnedRootsAndStorage()
    {
        Encode(new BigInteger[] { -1, 0, 128 }, out ulong[] source, out sbyte[] sourceSigns);
        Encode(new BigInteger[] { 49, -1408, 5888, -16384, 16384 }, out ulong[] target, out sbyte[] targetSigns);
        ulong[] sourceCell = new ulong[WideFiniteAxisIntersection.GetFiniteValueRootCellWords(source, sourceSigns)];
        ulong[] targetCell = new ulong[WideFiniteAxisIntersection.GetFiniteValueRootCellWords(target, targetSigns)];
        ulong[] radius = { 1 };
        int ordinal = -1;
        long allocated = FixedMathTestHelper.MeasureWarmedAllocations(() =>
        {
            WideFiniteAxisIntersection.TryGetFiniteValueRoot(source, sourceSigns, 0, sourceCell, out FiniteAxisValueRoot root);
            FiniteAxisValueRoot mapped = WideFiniteAxisIntersection.MapKnownFiniteValueRootRadicalOffset(root, -1, radius, 2,
                target, targetSigns, targetCell);
            ordinal = mapped.Ordinal;
        });
        Assert.Equal(0, ordinal);
        Assert.Equal(0, allocated);
    }

    private static void AssertMapping(BigInteger[] sourcePolynomial, int sign,
        ulong radiusSquared, int valueShift, BigInteger[] targetPolynomial, int expectedOrdinal,
        int maximumTargetShift = int.MaxValue)
    {
        Encode(sourcePolynomial, out ulong[] source, out sbyte[] sourceSigns);
        Encode(targetPolynomial, out ulong[] target, out sbyte[] targetSigns);
        ulong[] sourceCell = new ulong[WideFiniteAxisIntersection.GetFiniteValueRootCellWords(source, sourceSigns)];
        ulong[] sourceSnapshot = new ulong[sourceCell.Length];
        ulong[] targetCell = new ulong[WideFiniteAxisIntersection.GetFiniteValueRootCellWords(target, targetSigns)];
        Assert.True(WideFiniteAxisIntersection.TryGetFiniteValueRoot(source, sourceSigns, 0, sourceCell, out FiniteAxisValueRoot root));
        sourceCell.CopyTo(sourceSnapshot, 0);
        FiniteAxisValueRoot mapped = WideFiniteAxisIntersection.MapKnownFiniteValueRootRadicalOffset(root, sign, new[] { radiusSquared }, valueShift,
            target, targetSigns, targetCell);
        Assert.Equal(expectedOrdinal, mapped.Ordinal);
        Assert.InRange(mapped.DenominatorShift, 0, maximumTargetShift);
        Assert.Equal(sourceSnapshot, sourceCell);
        Assert.Equal(targetSigns, mapped.Signs.ToArray());
    }

    private static void Encode(BigInteger[] polynomial, out ulong[] values, out sbyte[] signs)
    {
        int words = 1;
        foreach (BigInteger coefficient in polynomial)
        {
            BigInteger remaining = BigInteger.Abs(coefficient) >> 64;
            int count = 1;
            while (!remaining.IsZero) { count++; remaining >>= 64; }
            words = Math.Max(words, count);
        }
        values = new ulong[words * polynomial.Length];
        signs = new sbyte[polynomial.Length];
        for (int index = 0; index < polynomial.Length; index++)
        {
            BigInteger value = BigInteger.Abs(polynomial[index]);
            signs[index] = (sbyte)polynomial[index].Sign;
            for (int word = 0; word < words; word++)
            {
                values[index * words + word] = (ulong)(value & ulong.MaxValue);
                value >>= 64;
            }
        }
    }
}
