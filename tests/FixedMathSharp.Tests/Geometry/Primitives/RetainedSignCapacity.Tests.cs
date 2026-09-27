using System.Numerics;
using FixedMathSharp.Geometry;
using Xunit;

namespace FixedMathSharp.Tests;

public sealed class RetainedSignCapacityTests
{
    [Fact]
    public void Sign_DoesNotRetainALowerNumeratorWhoseUpperEndpointOverflows()
    {
        // alpha=2^100/[4(2^100+1)] lies just below 1/4. At shift66 its
        // lower numerator is ulong.MaxValue, but its upper endpoint needs
        // a second word. The original coarse isolating cell needs only one.
        ulong[] coefficients = { 0, 1UL << 36, 4, 1UL << 38 };
        sbyte[] signs = { -1, 1 };
        ulong[] cell = new ulong[1];
        Assert.True(WideFiniteAxisIntersection.TryGetFiniteValueRoot(coefficients, signs, 0, cell, out var root));
        BigInteger numerator = BigInteger.One << 100;
        BigInteger denominator = 4 * (numerator + 1);
        int expected = (4 * numerator - denominator).Sign;
        Assert.Equal(-1, expected);
        Assert.Equal(expected, WideFiniteAxisIntersection.GetSignAtFiniteValueRootAndRefine(
            ref root, new ulong[] { 1, 4 }, signs));
        AssertCell(root, numerator, denominator);
    }

    [Fact]
    public void Sign_RetainedCompactCellRemainsValidForAnotherQueryAndRefinement()
    {
        ulong[] coefficients = { 0, 1UL << 36, 4, 1UL << 38 };
        sbyte[] signs = { -1, 1 };
        ulong[] cell = new ulong[1];
        Assert.True(WideFiniteAxisIntersection.TryGetFiniteValueRoot(coefficients, signs, 0, cell, out var root));
        BigInteger numerator = BigInteger.One << 100;
        BigInteger denominator = 4 * (numerator + 1);
        Assert.Equal((4 * numerator - denominator).Sign,
            WideFiniteAxisIntersection.GetSignAtFiniteValueRootAndRefine(ref root, new ulong[] { 1, 4 }, signs));
        Assert.Equal((8 * numerator - denominator).Sign,
            WideFiniteAxisIntersection.GetSignAtFiniteValueRootAndRefine(ref root, new ulong[] { 1, 8 }, signs));
        // This requested grid and its upper endpoint fit the original cell.
        WideFiniteAxisIntersection.RefineFiniteValueRoot(ref root, 60);
        AssertCell(root, numerator, denominator);
    }

    private static void AssertCell(FiniteAxisValueRoot root, BigInteger numerator, BigInteger denominator)
    {
        Assert.False(root.IsRational);
        Assert.Equal(0, root.Ordinal);
        Assert.Equal(1, root.LowerNumerator.Length);
        BigInteger lower = root.LowerNumerator[0];
        Assert.True(lower + 1 <= ulong.MaxValue);
        // Independent exact rational containment checks both endpoints and
        // the retained denominator metadata, not only the selected sign.
        BigInteger scaledNumerator = numerator << root.DenominatorShift;
        Assert.True(lower * denominator < scaledNumerator);
        Assert.True(scaledNumerator < (lower + 1) * denominator);
    }
}
