using System;
using FixedMathSharp.Geometry;
using Xunit;

namespace FixedMathSharp.Tests;

public sealed class CylinderPairValueSlopeTests
{
    [Theory]
    [InlineData(0, 1, -1)]
    [InlineData(1, 1, 1)]
    [InlineData(0, -1, 1)]
    [InlineData(1, -1, -1)]
    public void SimpleSlope_UsesTheSelectedNonDyadicCrossing(int ordinal, int leadingSign, int expected)
    {
        // (3t-1)(3t-2): simple roots 1/3 and 2/3 have opposite slopes.
        AssertSlope(new ulong[] { 2, 9, 9 }, new sbyte[]
            { (sbyte)leadingSign, (sbyte)-leadingSign, (sbyte)leadingSign }, ordinal, false, expected);
    }

    [Theory]
    [InlineData(-1)]
    [InlineData(1)]
    public void SimpleSlope_EvaluatesTheDerivativeAtAnExactDyadicRoot(int leadingSign) =>
        AssertSlope(new ulong[] { 1, 2 }, new sbyte[] { (sbyte)-leadingSign, (sbyte)leadingSign },
            0, true, leadingSign);

    [Theory]
    [InlineData(1)]
    [InlineData(-1)]
    public void SimpleSlope_PreservesAnIrrationalCrossingWithEitherLeadingSign(int leadingSign) =>
        AssertSlope(new ulong[] { 1, 0, 2 }, new sbyte[] { (sbyte)-leadingSign, 0, (sbyte)leadingSign },
            0, false, leadingSign);

    [Fact]
    public void SimpleSlope_HandlesTheUpperDomainEndpoint() =>
        AssertSlope(new ulong[] { 1, 1 }, new sbyte[] { -1, 1 }, 0, true, 1);

    [Fact]
    public void SimpleSlope_AllocatesNothingAndPreservesTheIsolatingCell()
    {
        ulong[] coefficients = { 2, 9, 9 };
        sbyte[] signs = { 1, -1, 1 };
        ulong[] cell = new ulong[WideFiniteAxisIntersection.GetFiniteValueRootCellWords(coefficients, signs)];
        int slope = 0;
        long allocated = FixedMathTestHelper.MeasureWarmedAllocations(() =>
        {
            WideFiniteAxisIntersection.TryGetFiniteValueRoot(coefficients, signs, 0, cell, out FiniteAxisValueRoot root);
            slope = CylinderPairRimFeatures.GetSimpleValueSlopeSign(root);
        });
        Assert.Equal(-1, slope);
        Assert.Equal(0, allocated);
    }

    private static void AssertSlope(ulong[] coefficients, sbyte[] signs, int ordinal, bool rational, int expected)
    {
        ulong[] cell = new ulong[WideFiniteAxisIntersection.GetFiniteValueRootCellWords(coefficients, signs)];
        Assert.True(WideFiniteAxisIntersection.TryGetFiniteValueRoot(coefficients, signs, ordinal, cell, out FiniteAxisValueRoot root));
        Assert.Equal(rational, root.IsRational);
        ulong[] original = (ulong[])cell.Clone();
        Assert.Equal(expected, CylinderPairRimFeatures.GetSimpleValueSlopeSign(root));
        Assert.Equal(original, cell);
    }
}
