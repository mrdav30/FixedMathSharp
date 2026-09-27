using System;
using System.Numerics;
using FixedMathSharp.Geometry;
using Xunit;

namespace FixedMathSharp.Tests;

public sealed class FiniteAxisValueRootRefinementTests
{
    [Theory]
    [InlineData(320, 1, 1)]
    [InlineData(512, -1, 1)]
    [InlineData(384, 1, 0)]
    [InlineData(384, -1, 0)]
    public void Refine_PreservesTheSelectedRootOfAnIllConditionedQuadratic(int shift, int sign, int ordinal)
    {
        // alpha=1/2+sqrt(2)/2^200. The two simple roots are only
        // 2*sqrt(2)/2^200 apart, so a coarse sign certificate must defer.
        BigInteger k = BigInteger.One << 199;
        BigInteger scale = BigInteger.One << 200;
        Encode(new[] { sign * (k * k - 2), -sign * 2 * k * scale, sign * scale * scale },
            out ulong[] coefficients, out sbyte[] signs);
        Span<ulong> cell = stackalloc ulong[10];
        Assert.True(WideFiniteAxisIntersection.TryGetFiniteValueRoot(coefficients, signs,
            ordinal, cell, out FiniteAxisValueRoot root));
        WideFiniteAxisIntersection.RefineFiniteValueRoot(ref root, shift);
        Assert.False(root.IsRational);
        Assert.Equal(shift, root.DenominatorShift);
        Assert.Equal(ordinal, root.Ordinal);
        BigInteger numerator = 0;
        for (int word = cell.Length - 1; word >= 0; word--)
            numerator = (numerator << 64) + cell[word];
        BigInteger lower = ordinal == 1
            ? numerator - (BigInteger.One << (shift - 1))
            : (BigInteger.One << (shift - 1)) - numerator - 1;
        BigInteger exactSquare = BigInteger.One << (2 * (shift - 200) + 1);
        Assert.True(lower > 0);
        Assert.True(lower * lower < exactSquare);
        Assert.True(exactSquare < (lower + 1) * (lower + 1));
    }

    [Theory]
    [InlineData(1, 1, 1)]
    [InlineData(1, 3, -1)]
    [InlineData(129, 1, 1)]
    [InlineData(129, 3, -1)]
    public void Refine_PreservesTheRootAcrossByteSizedAndPartialRefinementTargets(
        int definingPower, int multiplicity, int sign)
    {
        BigInteger[] polynomial = { sign };
        for (int power = 0; power < multiplicity; power++)
            polynomial = Multiply(polynomial,
                new[] { -BigInteger.One, BigInteger.Zero, BigInteger.One << definingPower });
        Encode(polynomial, out ulong[] coefficients, out sbyte[] signs);
        ulong[] originalCoefficients = (ulong[])coefficients.Clone();
        sbyte[] originalSigns = (sbyte[])signs.Clone();
        Span<ulong> cell = stackalloc ulong[8];
        Assert.True(WideFiniteAxisIntersection.TryGetFiniteValueRoot(coefficients, signs,
            0, cell, out FiniteAxisValueRoot root));

        int firstTarget = Math.Max(96, root.DenominatorShift + 24);
        foreach (int target in new[] { firstTarget, firstTarget + 8, firstTarget + 25 })
        {
            WideFiniteAxisIntersection.RefineFiniteValueRoot(ref root, target);
            Assert.Equal(target, root.DenominatorShift);
            Assert.Equal(0, root.Ordinal);
            AssertCell(root, definingPower);
        }
        Assert.Equal(originalCoefficients, coefficients);
        Assert.Equal(originalSigns, signs);
    }

    [Theory]
    [InlineData(3, 3)]
    [InlineData(257, 16)]
    [InlineData(4097, 20)]
    public void Refine_DiscoversDyadicEndpointsWithTheirMinimalDenominator(int numerator, int shift)
    {
        // The latter roots lie on a fine byte-grid endpoint but not on the
        // initial cell boundary. A multi-bit discovery must remove its extra
        // denominator powers of two, just as successive bisections would.
        Encode(new[] { -(BigInteger)numerator, BigInteger.One << shift },
            out ulong[] coefficients, out sbyte[] signs);
        Span<ulong> cell = stackalloc ulong[5];
        Assert.True(WideFiniteAxisIntersection.TryGetFiniteValueRoot(coefficients, signs,
            0, cell, out FiniteAxisValueRoot root));
        Assert.False(root.IsRational);

        WideFiniteAxisIntersection.RefineFiniteValueRoot(ref root, 256);

        Assert.True(root.IsRational);
        Assert.Equal(shift, root.DenominatorShift);
        Assert.Equal((ulong)numerator, cell[0]);
        for (int word = 1; word < cell.Length; word++)
            Assert.Equal(0UL, cell[word]);
        Assert.Equal(0, root.Ordinal);
    }

    [Theory]
    [InlineData(0, 1)]
    [InlineData(0, -1)]
    [InlineData(1, 1)]
    [InlineData(1, -1)]
    public void Refine_PreservesNonDyadicRationalsOnEitherSideOfAByteBoundary(int offset, int sign)
    {
        // alpha=(3*2^197+offset)/(2^200+1) lies immediately below/above
        // 3/8. Coarse endpoint-value hints can land on that grid boundary;
        // any rejected proposal must restore the complete original cell.
        BigInteger numerator = (3 * (BigInteger.One << 197)) + offset;
        BigInteger denominator = (BigInteger.One << 200) + 1;
        Encode(new[] { -sign * numerator, sign * denominator },
            out ulong[] coefficients, out sbyte[] signs);
        ulong[] originalCoefficients = (ulong[])coefficients.Clone();
        sbyte[] originalSigns = (sbyte[])signs.Clone();
        Span<ulong> cell = stackalloc ulong[7];
        Assert.True(WideFiniteAxisIntersection.TryGetFiniteValueRoot(coefficients, signs,
            0, cell, out FiniteAxisValueRoot root));

        const int target = 384;
        WideFiniteAxisIntersection.RefineFiniteValueRoot(ref root, target);

        Assert.False(root.IsRational);
        Assert.Equal(target, root.DenominatorShift);
        Assert.Equal(0, root.Ordinal);
        BigInteger lower = 0;
        for (int word = cell.Length - 1; word >= 0; word--)
            lower = (lower << 64) + cell[word];
        BigInteger scaledNumerator = numerator << target;
        Assert.True(lower * denominator < scaledNumerator);
        Assert.True(scaledNumerator < (lower + 1) * denominator);
        Assert.Equal(originalCoefficients, coefficients);
        Assert.Equal(originalSigns, signs);
    }

    [Theory]
    [InlineData(1, 1)]
    [InlineData(1, -1)]
    [InlineData(3, 1)]
    [InlineData(2, 1)]
    [InlineData(4, 1)]
    public void Refine_PreservesTheSelectedIrrationalRootAcrossMultiplicities(int multiplicity, int sign)
    {
        BigInteger[] polynomial = { sign };
        for (int power = 0; power < multiplicity; power++)
            polynomial = Multiply(polynomial, new BigInteger[] { -1, 0, 2 });
        Encode(polynomial, out ulong[] coefficients, out sbyte[] signs);
        ulong[] original = (ulong[])coefficients.Clone();
        Span<ulong> cell = stackalloc ulong[4];
        Assert.True(WideFiniteAxisIntersection.TryGetFiniteValueRoot(coefficients, signs,
            0, cell, out FiniteAxisValueRoot root));

        WideFiniteAxisIntersection.RefineFiniteValueRoot(ref root, 128);
        Assert.Equal(128, root.DenominatorShift);
        AssertCell(root, 1);
        ulong[] retained = cell.ToArray();
        WideFiniteAxisIntersection.RefineFiniteValueRoot(ref root, 96);
        Assert.Equal(128, root.DenominatorShift);
        Assert.True(cell.SequenceEqual(retained));
        WideFiniteAxisIntersection.RefineFiniteValueRoot(ref root, 192);
        Assert.Equal(192, root.DenominatorShift);
        AssertCell(root, 1);
        Assert.Equal(0, root.Ordinal);
        Assert.Equal(original, coefficients);
        Assert.True(root.Coefficients.SequenceEqual(coefficients));
        Assert.True(root.Signs.SequenceEqual(signs));
    }

    [Fact]
    public void Refine_StopsAtAnExactlyDiscoveredDyadicRoot()
    {
        ulong[] coefficients = { 3, 8 };
        sbyte[] signs = { -1, 1 };
        Span<ulong> cell = stackalloc ulong[2];
        Assert.True(WideFiniteAxisIntersection.TryGetFiniteValueRoot(coefficients, signs,
            0, cell, out FiniteAxisValueRoot root));
        Assert.False(root.IsRational);
        WideFiniteAxisIntersection.RefineFiniteValueRoot(ref root, 80);
        Assert.True(root.IsRational);
        Assert.Equal(3, root.DenominatorShift);
        Assert.Equal(3UL, cell[0]);
        Assert.Equal(0UL, cell[1]);
        WideFiniteAxisIntersection.RefineFiniteValueRoot(ref root, 100);
        Assert.Equal(3, root.DenominatorShift);
        Assert.Equal(3UL, cell[0]);
    }

    [Fact]
    public void Refine_PreservesATinyRootWithAnInitiallySmallNumerator()
    {
        Encode(new[] { -BigInteger.One, BigInteger.Zero, BigInteger.One << 129 },
            out ulong[] coefficients, out sbyte[] signs);
        Span<ulong> cell = stackalloc ulong[3];
        Assert.True(WideFiniteAxisIntersection.TryGetFiniteValueRoot(coefficients, signs,
            0, cell, out FiniteAxisValueRoot root));
        Assert.Equal(65, root.DenominatorShift);
        Assert.Equal(1UL, cell[0]);
        WideFiniteAxisIntersection.RefineFiniteValueRoot(ref root, 160);
        Assert.Equal(160, root.DenominatorShift);
        AssertCell(root, 129);
    }

    private static void AssertCell(FiniteAxisValueRoot root, int definingPower)
    {
        Assert.False(root.IsRational);
        BigInteger lower = 0;
        for (int index = root.LowerNumerator.Length - 1; index >= 0; index--)
            lower = (lower << 64) + root.LowerNumerator[index];
        BigInteger scaleSquared = BigInteger.One << (2 * root.DenominatorShift);
        Assert.True((lower * lower << definingPower) < scaleSquared);
        Assert.True(scaleSquared < ((lower + 1) * (lower + 1) << definingPower));
    }

    private static BigInteger[] Multiply(BigInteger[] left, BigInteger[] right)
    {
        var result = new BigInteger[left.Length + right.Length - 1];
        for (int first = 0; first < left.Length; first++)
            for (int second = 0; second < right.Length; second++)
                result[first + second] += left[first] * right[second];
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
            Assert.Equal(BigInteger.Zero, magnitude);
        }
    }
}
