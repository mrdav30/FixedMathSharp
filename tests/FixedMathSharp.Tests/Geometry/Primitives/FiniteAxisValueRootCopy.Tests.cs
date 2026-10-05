using System;
using FixedMathSharp.Geometry;
using Xunit;

namespace FixedMathSharp.Tests;

public sealed class FiniteAxisValueRootCopyTests
{
    [Theory]
    [InlineData(false)]
    [InlineData(true)]
    public void CopyTo_ShouldRetainIndependentActivePolynomialCellAndMetadata(bool rational)
    {
        // The rational case selects ordinal 1, the root 3/4 of
        // (4x-1)(4x-3). The irrational case selects sqrt(1/2).
        ulong[] coefficients = rational ? new ulong[] { 3, 16, 16, 0, 0 } : new ulong[] { 1, 0, 2, 0, 0 };
        sbyte[] signs = rational ? new sbyte[] { 1, -1, 1, 0, 0 } : new sbyte[] { -1, 0, 1, 0, 0 };
        ulong[] cell = new ulong[16];
        Assert.True(WideFiniteAxisIntersection.TryGetFiniteValueRoot(coefficients, signs,
            rational ? 1 : 0, cell, out FiniteAxisValueRoot source));
        Assert.Equal(0, WideFiniteAxisIntersection.GetSignAtFiniteValueRootAndRefine(
            ref source, source.Coefficients, source.Signs));
        Assert.Equal(rational, source.IsRational);
        ulong[] expectedCoefficients = source.Coefficients.ToArray();
        sbyte[] expectedSigns = source.Signs.ToArray();
        int expectedShift = source.DenominatorShift;
        int expectedOrdinal = source.Ordinal;
        ulong[] destinationCoefficients = new ulong[9];
        sbyte[] destinationSigns = new sbyte[9];
        // The destination is smaller than the source's scratch allocation,
        // but holds its numerator and the excluded upper endpoint exactly.
        ulong[] destinationCell = new ulong[4];
        Array.Fill(destinationCoefficients, ulong.MaxValue);
        Array.Fill(destinationSigns, (sbyte)2);
        Array.Fill(destinationCell, ulong.MaxValue);

        FiniteAxisValueRoot copy = FiniteAxisValueRoot.CopyTo(source, destinationCoefficients, destinationSigns, destinationCell);

        Assert.True(copy.Coefficients.SequenceEqual(expectedCoefficients));
        Assert.True(copy.Signs.SequenceEqual(expectedSigns));
        Assert.Equal(expectedShift, copy.DenominatorShift);
        Assert.Equal(expectedOrdinal, copy.Ordinal);
        Assert.Equal(rational, copy.IsRational);
        for (int index = 0; index < destinationCell.Length; index++)
            Assert.Equal(cell[index], destinationCell[index]);
        for (int index = expectedCoefficients.Length; index < destinationCoefficients.Length; index++)
            Assert.Equal(ulong.MaxValue, destinationCoefficients[index]);
        for (int index = expectedSigns.Length; index < destinationSigns.Length; index++)
            Assert.Equal((sbyte)2, destinationSigns[index]);

        // Reusing every source buffer must not change the copied root.
        Array.Fill(coefficients, ulong.MaxValue);
        Array.Fill(signs, (sbyte)0);
        Array.Fill(cell, ulong.MaxValue);
        Assert.True(copy.Coefficients.SequenceEqual(expectedCoefficients));
        Assert.True(copy.Signs.SequenceEqual(expectedSigns));
        Assert.Equal(expectedShift, copy.DenominatorShift);
        Assert.Equal(expectedOrdinal, copy.Ordinal);
        Assert.Equal(rational, copy.IsRational);
        Assert.Equal(0, WideFiniteAxisIntersection.GetSignAtFiniteValueRoot(copy, copy.Coefficients, copy.Signs));
    }
}
