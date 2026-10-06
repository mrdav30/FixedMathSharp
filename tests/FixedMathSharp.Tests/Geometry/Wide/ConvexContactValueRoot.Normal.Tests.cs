using System;
using System.Numerics;
using FixedMathSharp.Geometry;
using Xunit;

namespace FixedMathSharp.Tests.Geometry.Wide;

public sealed class ConvexContactValueRootNormalTests
{
    [Theory]
    [InlineData(0L)]
    [InlineData(1L << 32)]
    [InlineData(long.MaxValue)]
    public void ConstantPythagoreanDirection_PreservesScaleOrientationAndZeroComponent(long scaleRaw)
    {
        ulong[] gradient = { 3, 4, 0 };
        sbyte[] signs = { 1, -1, 0 };
        foreach (int orientation in new[] { -1, 1 })
        {
            var root = ThirdRoot(0);
            Vector3d expected = new(Fixed64.FromRaw(orientation * RoundRatio(3 * (BigInteger)scaleRaw, 5)),
                Fixed64.FromRaw(-orientation * RoundRatio(4 * (BigInteger)scaleRaw, 5)), Fixed64.Zero);
            Assert.Equal(expected, ConvexContactValueRoot.GetScaledNormalizedDirection(
                ref root, gradient, signs, Fixed64.FromRaw(scaleRaw), orientation));
            // The normalization entry point supplies exactly Q32.32 one.
            Assert.Equal(new Vector3d(Fixed64.FromRaw(orientation * 2576980378L),
                Fixed64.FromRaw(-orientation * 3435973837L), Fixed64.Zero),
                ConvexContactValueRoot.GetNormalizedDirection(ref root, gradient, signs, orientation));
        }
        Assert.Equal(new ulong[] { 3, 4, 0 }, gradient);
        Assert.Equal(new sbyte[] { 1, -1, 0 }, signs);
    }

    [Theory]
    [InlineData(0L)]
    [InlineData(1L << 32)]
    [InlineData(long.MaxValue)]
    public void ZeroGradient_MaterializesZeroForEitherOrientation(long scaleRaw)
    {
        foreach (int orientation in new[] { -1, 1 })
        {
            var root = ThirdRoot(0);
            Assert.Equal(Vector3d.Zero, ConvexContactValueRoot.GetScaledNormalizedDirection(
                ref root, new ulong[3], new sbyte[3], Fixed64.FromRaw(scaleRaw), orientation));
        }
    }

    [Theory]
    [InlineData(1L)]
    [InlineData(3L)]
    [InlineData(5L)]
    [InlineData(7L)]
    [InlineData(1L << 32)]
    [InlineData(long.MaxValue)]
    public void AlgebraicDirection_RoundsExactHalfRawTiesAndIrrationalComponent(long scaleRaw)
    {
        // alpha=sqrt(3)/2 is the sole root in (0,1] of 4t²-3.
        // (1,2alpha,0) has length exactly two: X=scale/2 and Y=scale*sqrt(3)/2.
        ulong[] gradient = { 1, 0, 0, 2, 0, 0 };
        sbyte[] signs = { 1, 0, 0, 1, 0, 0 };
        foreach (int shift in new[] { 0, 96 })
        {
            foreach (int orientation in new[] { -1, 1 })
            {
                var root = SqrtThreeQuarterRoot(shift);
                Vector3d expected = new(Fixed64.FromRaw(orientation * RoundRatio(scaleRaw, 2)),
                    Fixed64.FromRaw(orientation * RoundSquareRootRatio(3 * (BigInteger)scaleRaw * scaleRaw, 4)), Fixed64.Zero);
                Assert.Equal(expected, ConvexContactValueRoot.GetScaledNormalizedDirection(
                    ref root, gradient, signs, Fixed64.FromRaw(scaleRaw), orientation));
            }
        }
    }

    [Theory]
    [InlineData(0)]
    [InlineData(96)]
    [InlineData(128)]
    public void NonDyadicDirection_PreservesIndependentRadicalRoundingAcrossRootCells(int shift)
    {
        // At alpha=1/3, (1,-2alpha,0)/length is exactly (3,-2,0)/sqrt(13).
        ulong[] gradient = { 1, 0, 0, 2, 0, 0 };
        sbyte[] signs = { 1, 0, 0, -1, 0, 0 };
        foreach (long scaleRaw in new[] { 1L << 32, long.MaxValue })
        {
            foreach (int orientation in new[] { -1, 1 })
            {
                var root = ThirdRoot(shift);
                BigInteger square = (BigInteger)scaleRaw * scaleRaw;
                Vector3d expected = new(Fixed64.FromRaw(orientation * RoundSquareRootRatio(9 * square, 13)),
                    Fixed64.FromRaw(-orientation * RoundSquareRootRatio(4 * square, 13)), Fixed64.Zero);
                Assert.Equal(expected, ConvexContactValueRoot.GetScaledNormalizedDirection(
                    ref root, gradient, signs, Fixed64.FromRaw(scaleRaw), orientation));
            }
        }
    }

    [Theory]
    [InlineData(1)]
    [InlineData(63)]
    [InlineData(128)]
    public void DyadicRoot_ExactComponentCancellationPreservesAxisDirection(int shift)
    {
        // At alpha=1/2 both 2alpha-1 and 1-2alpha vanish exactly.
        ulong[] gradient = { 1, 2, 1, 0, 1, 2 };
        sbyte[] signs = { -1, 1, 1, 0, 1, -1 };
        foreach (int orientation in new[] { -1, 1 })
        {
            var root = Root(new ulong[] { 1, 2 }, new sbyte[] { -1, 1 },
                BigInteger.One << (shift - 1), shift, true);
            Assert.Equal(new Vector3d(Fixed64.Zero, Fixed64.FromRaw(orientation * long.MaxValue), Fixed64.Zero),
                ConvexContactValueRoot.GetScaledNormalizedDirection(ref root, gradient, signs, Fixed64.MaxValue, orientation));
        }
    }

    [Theory]
    [InlineData(2, 1, 0)]
    [InlineData(2, 1, 96)]
    [InlineData(2, ConvexContactCandidate.Words, 0)]
    [InlineData(2, ConvexContactCandidate.Words, 96)]
    [InlineData(8, 1, 0)]
    [InlineData(8, 1, 96)]
    [InlineData(8, ConvexContactCandidate.Words, 0)]
    [InlineData(8, ConvexContactCandidate.Words, 96)]
    public void WideCommonFactor_PreservesDirectionThroughCancellationAndDegreeSixteenSquares(int degree, int words, int shift)
    {
        // f(t)=1+A*(3t-1)^degree is positive and f(1/3)=1. Thus the direction
        // (3f,-4f,0) is exactly (3,-4,0)/5, independently of coefficient width.
        // Shallow Horner cells can have a nonpositive lower length bound despite
        // a positive exact length. Degree eight also exercises squared degree sixteen.
        int count = degree + 1;
        var gradient = new ulong[3 * count * words];
        var signs = new sbyte[3 * count];
        BigInteger amplitude = BigInteger.One << (64 * (words - 1));
        BigInteger binomial = BigInteger.One;
        for (int index = 0; index <= degree; index++)
        {
            BigInteger coefficient = amplitude * binomial * BigInteger.Pow(3, index);
            if (((degree - index) & 1) != 0) coefficient = -coefficient;
            if (index == 0) coefficient++;
            Write(3 * coefficient, gradient.AsSpan(index * words, words), out signs[index]);
            Write(-4 * coefficient, gradient.AsSpan((count + index) * words, words), out signs[count + index]);
            if (index < degree) binomial = binomial * (degree - index) / (index + 1);
        }
        ulong[] original = (ulong[])gradient.Clone();
        sbyte[] originalSigns = (sbyte[])signs.Clone();
        foreach (int orientation in new[] { -1, 1 })
        {
            var root = ThirdRoot(shift);
            Assert.Equal(new Vector3d(Fixed64.FromRaw(orientation * RoundRatio(3 * (BigInteger)long.MaxValue, 5)),
                Fixed64.FromRaw(-orientation * RoundRatio(4 * (BigInteger)long.MaxValue, 5)), Fixed64.Zero),
                ConvexContactValueRoot.GetScaledNormalizedDirection(ref root, gradient, signs, Fixed64.MaxValue, orientation));
        }
        Assert.Equal(original, gradient);
        Assert.Equal(originalSigns, signs);
    }

    [Theory]
    [InlineData(1, 1, 1L << 32)]
    [InlineData(1, 1, long.MaxValue)]
    [InlineData(1, ConvexContactCandidate.Words, 1L << 32)]
    [InlineData(1, ConvexContactCandidate.Words, long.MaxValue)]
    [InlineData(8, 1, 1L << 32)]
    [InlineData(8, 1, long.MaxValue)]
    [InlineData(8, ConvexContactCandidate.Words, 1L << 32)]
    [InlineData(8, ConvexContactCandidate.Words, long.MaxValue)]
    public void PositiveCommonFactor_PreservesDirectionWhenCoefficientSumsExceedNormalization(int degree, int words, long scaleRaw)
    {
        // At the exact endpoint t=1, f=A*(1+t)^degree=A*2^degree.
        // Its positive coefficient sum exceeds the largest coefficient's power
        // normalization; the length sum also exercises compact values wider than 32 bits.
        // The common factor cancels, leaving the independent (3,-4,0)/5 identity.
        int count = degree + 1;
        var gradient = new ulong[3 * count * words];
        var signs = new sbyte[3 * count];
        BigInteger coefficient = BigInteger.One << (64 * (words - 1));
        for (int index = 0; index <= degree; index++)
        {
            Write(3 * coefficient, gradient.AsSpan(index * words, words), out signs[index]);
            Write(-4 * coefficient, gradient.AsSpan((count + index) * words, words), out signs[count + index]);
            if (index < degree) coefficient = coefficient * (degree - index) / (index + 1);
        }
        foreach (int orientation in new[] { -1, 1 })
        {
            var root = Root(new ulong[] { 1, 1 }, new sbyte[] { -1, 1 }, BigInteger.One, 0, true);
            Assert.Equal(new Vector3d(Fixed64.FromRaw(orientation * RoundRatio(3 * (BigInteger)scaleRaw, 5)),
                Fixed64.FromRaw(-orientation * RoundRatio(4 * (BigInteger)scaleRaw, 5)), Fixed64.Zero),
                ConvexContactValueRoot.GetScaledNormalizedDirection(ref root, gradient, signs, Fixed64.FromRaw(scaleRaw), orientation));
        }
    }

    private static FiniteAxisValueRoot ThirdRoot(int shift) => Root(new ulong[] { 1, 3 }, new sbyte[] { -1, 1 },
        (BigInteger.One << shift) / 3, shift, false);

    private static FiniteAxisValueRoot SqrtThreeQuarterRoot(int shift) => Root(new ulong[] { 3, 0, 4 },
        new sbyte[] { -1, 0, 1 }, IntegerSquareRoot((3 * (BigInteger.One << (2 * shift))) / 4), shift, false);

    private static FiniteAxisValueRoot Root(ulong[] coefficients, sbyte[] signs, BigInteger lower, int shift, bool singleton)
    {
        var cell = new ulong[WideFiniteAxisIntersection.GetFiniteValueRootCellWords(coefficients, signs)];
        Write(lower, cell, out _);
        return new FiniteAxisValueRoot
        {
            Coefficients = coefficients, Signs = signs, LowerNumerator = cell,
            DenominatorShift = shift, IsRational = singleton
        };
    }

    private static void Write(BigInteger value, Span<ulong> destination, out sbyte sign)
    {
        sign = (sbyte)value.Sign;
        value = BigInteger.Abs(value);
        for (int index = 0; index < destination.Length; index++)
        {
            destination[index] = (ulong)(value & ulong.MaxValue);
            value >>= 64;
        }
        Assert.Equal(BigInteger.Zero, value);
    }

    private static long RoundRatio(BigInteger numerator, BigInteger denominator)
    {
        BigInteger floor = BigInteger.DivRem(numerator, denominator, out BigInteger remainder);
        int midpoint = (2 * remainder).CompareTo(denominator);
        return (long)(floor + (midpoint > 0 || midpoint == 0 && !floor.IsEven ? 1 : 0));
    }

    private static long RoundSquareRootRatio(BigInteger numerator, BigInteger denominator)
    {
        BigInteger floor = IntegerSquareRoot(numerator / denominator);
        BigInteger half = 2 * floor + 1;
        int midpoint = (4 * numerator).CompareTo(half * half * denominator);
        return (long)(floor + (midpoint > 0 || midpoint == 0 && !floor.IsEven ? 1 : 0));
    }

    private static BigInteger IntegerSquareRoot(BigInteger value)
    {
        if (value.IsZero) return BigInteger.Zero;
        BigInteger root = BigInteger.One << (int)((value.GetBitLength() + 1) / 2);
        while (true)
        {
            BigInteger next = (root + value / root) / 2;
            if (next >= root) return root;
            root = next;
        }
    }
}
