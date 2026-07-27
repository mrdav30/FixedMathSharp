//=======================================================================
// WideRadialProjectionComparison.Tests.cs
//=======================================================================
// MIT License, Copyright (c) 2024–present David Oravsky (mrdav30)
// See LICENSE file in the project root for full license information.
//=======================================================================

using System;
using System.Numerics;
using System.Runtime.CompilerServices;
using System.Threading;
using Xunit;

namespace FixedMathSharp.Tests;

public sealed class WideRadialProjectionComparisonTests
{
    private static ulong s_stackSink;

    [Fact]
    public void Comparison_ShouldOrderDistinctDepthsInsideOneRawOutputBin()
    {
        const long scale = 1L << 40;
        Signed576 full = ToSigned576(scale);
        Signed576 almostFull = ToSigned576(scale - 1L);
        Signed576 denominator = ToSigned576(1L);
        Signed192 common = Signed192.Signed(1L);

        int greater = WideArithmetic.CompareRadialProjectionDepths(
            default,
            Signed832.ExtendValue(full),
            denominator,
            full,
            default,
            Signed832.ExtendValue(almostFull),
            denominator,
            full,
            common);
        int lesser = WideArithmetic.CompareRadialProjectionDepths(
            default,
            Signed832.ExtendValue(almostFull),
            denominator,
            full,
            default,
            Signed832.ExtendValue(full),
            denominator,
            full,
            common);
        int equal = WideArithmetic.CompareRadialProjectionDepths(
            default,
            Signed832.ExtendValue(full),
            denominator,
            full,
            default,
            Signed832.ExtendValue(full),
            denominator,
            full,
            common);

        Assert.True(greater > 0);
        Assert.True(lesser < 0);
        Assert.Equal(0, equal);
    }

    [Fact]
    public void Comparison_PerfectSquareMatrix_MatchesBigIntegerOracle()
    {
        ProjectionDescriptor[] descriptors =
        {
            new(0L, 3L, 2L, 5L),
            new(4L, 1L, 3L, 2L),
            new(-2L, 5L, 2L, 3L),
            new(7L, 0L, 1L, 4L),
            new(-7L, 2L, 1L, 1L),
            new(0L, 6L, 4L, 5L),
            new(-7L, 1L, 1L, 1L),
            new(1L << 50, 1L << 49, 1L, 1L),
        };

        for (int leftIndex = 0; leftIndex < descriptors.Length; leftIndex++)
        {
            for (int rightIndex = 0; rightIndex < descriptors.Length; rightIndex++)
            {
                ProjectionDescriptor left = descriptors[leftIndex];
                ProjectionDescriptor right = descriptors[rightIndex];
                int expected = CompareOracle(left, right);
                int actual = Compare(left, right);
                int reversed = Compare(right, left);

                Assert.Equal(expected, actual);
                Assert.Equal(-expected, reversed);
            }
        }
    }

    [Fact]
    public void Comparison_NonSquareCrossRadicalMatrix_MatchesHighPrecisionOracle()
    {
        RadicalProjectionDescriptor[] descriptors =
        {
            new(3L, 2L, 3L, 5L),
            new(4L, 7L, 11L, 3L),
            new(-2L, 19L, 2L, 7L),
            new(0L, 5L, 13L, 2L),
            new(10L, 3L, 17L, 11L),
            new(-1L, 29L, 5L, 13L),
        };

        for (int leftIndex = 0; leftIndex < descriptors.Length; leftIndex++)
        {
            for (int rightIndex = 0; rightIndex < descriptors.Length; rightIndex++)
            {
                RadicalProjectionDescriptor left = descriptors[leftIndex];
                RadicalProjectionDescriptor right = descriptors[rightIndex];
                int expected = CompareHighPrecisionOracle(left, right);
                int actual = Compare(left, right);
                int reversed = Compare(right, left);

                Assert.Equal(expected, actual);
                Assert.Equal(-expected, reversed);
            }
        }
    }

    [Theory]
    [InlineData(-2L, 19L, 2L, 7L, 4L, 7L, 11L, 3L)]
    [InlineData(3L, 2L, 3L, 5L, -1L, 29L, 5L, 13L)]
    [InlineData(0L, 5L, 13L, 2L, 10L, 3L, 17L, 11L)]
    public void Comparison_NonSquareCrossRadicalCancellation_MatchesHighPrecisionOracle(
        long leftRational,
        long leftNumerator,
        long leftDenominator,
        long leftAxisSquared,
        long rightRational,
        long rightNumerator,
        long rightDenominator,
        long rightAxisSquared)
    {
        var left = new RadicalProjectionDescriptor(
            leftRational,
            leftNumerator,
            leftDenominator,
            leftAxisSquared);
        var right = new RadicalProjectionDescriptor(
            rightRational,
            rightNumerator,
            rightDenominator,
            rightAxisSquared);

        Assert.Equal(
            CompareHighPrecisionOracle(left, right),
            Compare(left, right));
    }

    [Fact]
    public void Comparison_EquivalentScaledDescriptors_AreEqual()
    {
        var first = new ProjectionDescriptor(0L, 3L, 2L, 5L);
        var second = new ProjectionDescriptor(0L, 6L, 4L, 5L);

        Assert.Equal(0, Compare(first, second));
    }

    [Fact]
    public void Comparison_DistinctRadicalDecompositionsOfSameDepth_AreEqual()
    {
        var first = new RadicalProjectionDescriptor(-20L, 9L, 1L, 1L);
        var second = new RadicalProjectionDescriptor(-19L, 9L, 1L, 4L);

        Assert.Equal(
            CompareHighPrecisionOracle(first, second),
            Compare(first, second));
        Assert.Equal(0, Compare(first, second));
    }

    [Fact]
    public void Comparison_OrdersDescriptorsTransitively()
    {
        var lower = new ProjectionDescriptor(0L, 3L, 2L, 5L);
        var middle = new ProjectionDescriptor(4L, 1L, 3L, 2L);
        var upper = new ProjectionDescriptor(-2L, 5L, 2L, 3L);

        Assert.True(Compare(lower, middle) < 0);
        Assert.True(Compare(middle, upper) < 0);
        Assert.True(Compare(lower, upper) < 0);
    }

    [Fact]
    public void Comparison_DirtyStackAndZeroTerms_RemainDeterministic()
    {
        var zero = new ProjectionDescriptor(-7L, 1L, 1L, 1L);
        var large = new ProjectionDescriptor(
            1L << 50,
            1L << 49,
            1L,
            1L);

        for (int iteration = 0; iteration < 128; iteration++)
        {
            PolluteStack(unchecked((ulong)iteration + 1UL));
            Assert.True(Compare(large, zero) > 0);
            PolluteStack(unchecked((ulong)iteration + 257UL));
            Assert.Equal(0, Compare(zero, zero));
        }
    }

    [Fact]
    public void Comparison_WarmedExecution_DoesNotAllocate()
    {
        var left = new ProjectionDescriptor(-2L, 5L, 2L, 3L);
        var right = new ProjectionDescriptor(4L, 1L, 3L, 2L);
        for (int iteration = 0; iteration < 32; iteration++)
            _ = Compare(left, right);

        int accumulator = 0;
        long before = GC.GetAllocatedBytesForCurrentThread();
        for (int iteration = 0; iteration < 64; iteration++)
            accumulator += Compare(left, right);
        long after = GC.GetAllocatedBytesForCurrentThread();

        Assert.Equal(64, accumulator);
        Assert.Equal(before, after);
    }

    [Fact]
    public void GeneralizedComparison_RadicalMatrixMatchesHighPrecisionOracle()
    {
        GeneralizedProjectionDescriptor[] descriptors =
        {
            new(3L, 2L, 3L, 5L, 7L),
            new(-2L, 5L, 19L, 2L, 7L),
            new(4L, 3L, 7L, 11L, 3L),
            new(0L, 0L, 0L, 1L, 5L),
            new(-5L, 2L, 9L, 1L, 1L),
            new(-4L, 3L, 4L, 1L, 4L),
        };

        for (int leftIndex = 0; leftIndex < descriptors.Length; leftIndex++)
        {
            for (int rightIndex = 0; rightIndex < descriptors.Length; rightIndex++)
            {
                GeneralizedProjectionDescriptor left = descriptors[leftIndex];
                GeneralizedProjectionDescriptor right = descriptors[rightIndex];
                int expected = CompareHighPrecisionOracle(left, right);
                int actual = Compare(left, right);
                int reversed = Compare(right, left);

                Assert.Equal(expected, actual);
                Assert.Equal(-expected, reversed);
            }
        }
    }

    [Fact]
    public void GeneralizedComparison_OrdersFullSigned704RationalRange()
    {
        Signed704 larger = CreateSigned704Word(8, 1UL);
        Signed704 smaller = CreateSigned704Word(7, ulong.MaxValue);
        Signed576 unit = ToSigned576(1L);

        int comparison = WideArithmetic.CompareRadialProjectionDepths(
            larger,
            default,
            default,
            unit,
            unit,
            smaller,
            default,
            default,
            unit,
            unit);
        int reversed = WideArithmetic.CompareRadialProjectionDepths(
            smaller,
            default,
            default,
            unit,
            unit,
            larger,
            default,
            default,
            unit,
            unit);

        Assert.True(comparison > 0);
        Assert.True(reversed < 0);
    }

    [Fact]
    public void GeneralizedComparison_DirtyStackAndWarmedExecutionRemainAllocationFree()
    {
        var left = new GeneralizedProjectionDescriptor(
            -2L,
            5L,
            19L,
            2L,
            7L);
        var right = new GeneralizedProjectionDescriptor(
            4L,
            3L,
            7L,
            11L,
            3L);
        for (int iteration = 0; iteration < 32; iteration++)
            _ = Compare(left, right);

        int accumulator = 0;
        long before = GC.GetAllocatedBytesForCurrentThread();
        for (int iteration = 0; iteration < 64; iteration++)
        {
            PolluteStack(unchecked((ulong)iteration + 1UL));
            accumulator += Compare(left, right);
            PolluteStack(unchecked((ulong)iteration + 257UL));
            accumulator -= Compare(right, left);
        }
        long after = GC.GetAllocatedBytesForCurrentThread();

        Assert.Equal(128, accumulator);
        Assert.Equal(before, after);
    }

    [Fact]
    public void Signed704Comparison_OrdersEveryMagnitudeWord()
    {
        Signed704 zero = default;
        for (int wordIndex = 0; wordIndex <= 10; wordIndex++)
        {
            Signed704 value = CreateSigned704Word(wordIndex, 1UL);

            Assert.True(WideArithmetic.CompareNonNegative(value, zero) > 0);
            Assert.True(WideArithmetic.CompareNonNegative(zero, value) < 0);
            Assert.Equal(0, WideArithmetic.CompareNonNegative(value, value));
        }
    }

    private static int Compare(
        ProjectionDescriptor left,
        ProjectionDescriptor right)
    {
        Signed192 common = Signed192.Signed(7L);
        return WideArithmetic.CompareRadialProjectionDepths(
            ToSigned576(left.Rational),
            SquareToSigned832(left.RadicandRootNumerator),
            SquareToSigned576(left.RadicandRootDenominator),
            SquareToSigned576(left.AxisRoot),
            ToSigned576(right.Rational),
            SquareToSigned832(right.RadicandRootNumerator),
            SquareToSigned576(right.RadicandRootDenominator),
            SquareToSigned576(right.AxisRoot),
            common);
    }

    private static int Compare(
        RadicalProjectionDescriptor left,
        RadicalProjectionDescriptor right)
    {
        Signed192 common = Signed192.Signed(7L);
        return WideArithmetic.CompareRadialProjectionDepths(
            ToSigned576(left.Rational),
            ToSigned832(left.RadicandNumerator),
            ToSigned576(left.RadicandDenominator),
            ToSigned576(left.AxisSquared),
            ToSigned576(right.Rational),
            ToSigned832(right.RadicandNumerator),
            ToSigned576(right.RadicandDenominator),
            ToSigned576(right.AxisSquared),
            common);
    }

    private static int Compare(
        GeneralizedProjectionDescriptor left,
        GeneralizedProjectionDescriptor right) =>
        WideArithmetic.CompareRadialProjectionDepths(
            ToSigned704(left.Rational),
            ToSigned320(left.RadialCoefficient),
            ToSigned832(left.RadicandNumerator),
            ToSigned576(left.RadicandDenominator),
            ToSigned576(left.AxisSquared),
            ToSigned704(right.Rational),
            ToSigned320(right.RadialCoefficient),
            ToSigned832(right.RadicandNumerator),
            ToSigned576(right.RadicandDenominator),
            ToSigned576(right.AxisSquared));

    private static int CompareOracle(
        ProjectionDescriptor left,
        ProjectionDescriptor right)
    {
        BigInteger leftNumerator =
            ((BigInteger)left.Rational * left.RadicandRootDenominator)
            + ((BigInteger)7 * left.RadicandRootNumerator);
        BigInteger leftDenominator =
            (BigInteger)7
            * left.RadicandRootDenominator
            * left.AxisRoot;
        BigInteger rightNumerator =
            ((BigInteger)right.Rational * right.RadicandRootDenominator)
            + ((BigInteger)7 * right.RadicandRootNumerator);
        BigInteger rightDenominator =
            (BigInteger)7
            * right.RadicandRootDenominator
            * right.AxisRoot;
        return (leftNumerator * rightDenominator)
            .CompareTo(rightNumerator * leftDenominator);
    }

    private static int CompareHighPrecisionOracle(
        RadicalProjectionDescriptor left,
        RadicalProjectionDescriptor right)
    {
        const int precisionBits = 512;
        BigInteger scale = BigInteger.One << precisionBits;
        BigInteger scaledSquare = scale * scale;
        BigInteger leftRadical = IntegerSquareRoot(
            (new BigInteger(left.RadicandNumerator) * scaledSquare)
            / left.RadicandDenominator);
        BigInteger rightRadical = IntegerSquareRoot(
            (new BigInteger(right.RadicandNumerator) * scaledSquare)
            / right.RadicandDenominator);
        BigInteger leftAxis = IntegerSquareRoot(
            new BigInteger(left.AxisSquared) * scaledSquare);
        BigInteger rightAxis = IntegerSquareRoot(
            new BigInteger(right.AxisSquared) * scaledSquare);
        BigInteger leftDepthNumerator =
            (new BigInteger(left.Rational) * scale)
            + (7 * leftRadical);
        BigInteger rightDepthNumerator =
            (new BigInteger(right.Rational) * scale)
            + (7 * rightRadical);

        Assert.True(leftDepthNumerator >= BigInteger.Zero);
        Assert.True(rightDepthNumerator >= BigInteger.Zero);
        return (leftDepthNumerator * rightAxis)
            .CompareTo(rightDepthNumerator * leftAxis);
    }

    private static int CompareHighPrecisionOracle(
        GeneralizedProjectionDescriptor left,
        GeneralizedProjectionDescriptor right)
    {
        const int precisionBits = 512;
        BigInteger scale = BigInteger.One << precisionBits;
        BigInteger scaledSquare = scale * scale;
        BigInteger leftRadical = IntegerSquareRoot(
            (new BigInteger(left.RadicandNumerator) * scaledSquare)
            / left.RadicandDenominator);
        BigInteger rightRadical = IntegerSquareRoot(
            (new BigInteger(right.RadicandNumerator) * scaledSquare)
            / right.RadicandDenominator);
        BigInteger leftAxis = IntegerSquareRoot(
            new BigInteger(left.AxisSquared) * scaledSquare);
        BigInteger rightAxis = IntegerSquareRoot(
            new BigInteger(right.AxisSquared) * scaledSquare);
        BigInteger leftDepthNumerator =
            (new BigInteger(left.Rational) * scale)
            + (left.RadialCoefficient * leftRadical);
        BigInteger rightDepthNumerator =
            (new BigInteger(right.Rational) * scale)
            + (right.RadialCoefficient * rightRadical);

        Assert.True(leftDepthNumerator >= BigInteger.Zero);
        Assert.True(rightDepthNumerator >= BigInteger.Zero);
        return (leftDepthNumerator * rightAxis)
            .CompareTo(rightDepthNumerator * leftAxis);
    }

    private static BigInteger IntegerSquareRoot(BigInteger value)
    {
        Assert.True(value >= BigInteger.Zero);
        if (value <= BigInteger.One)
            return value;

        int bitLength = checked((int)value.GetBitLength());
        BigInteger estimate =
            BigInteger.One << ((bitLength + 1) / 2);
        while (true)
        {
            BigInteger next = (estimate + (value / estimate)) >> 1;
            if (next >= estimate)
                return estimate;

            estimate = next;
        }
    }

    [MethodImpl(MethodImplOptions.NoInlining)]
    private static void PolluteStack(ulong seed)
    {
        Span<ulong> words = stackalloc ulong[512];
        for (int index = 0; index < words.Length; index++)
            words[index] = seed + unchecked((ulong)index * 0x9E37_79B9UL);
        Volatile.Write(
            ref s_stackSink,
            words[seed.GetHashCode() & 511]);
    }

    private static Signed832 SquareToSigned832(long value) =>
        Signed832.ExtendValue(
            SquareToSigned576(value));

    private static Signed832 ToSigned832(long value) =>
        Signed832.ExtendValue(ToSigned576(value));

    private static Signed576 SquareToSigned576(long value)
    {
        Signed192 wide = Signed192.Signed(value);
        return Signed576.ExtendValue(
            WideArithmetic.MultiplySigned192(wide, wide));
    }

    private static Signed576 ToSigned576(long value) =>
        Signed576.ExtendValue(
            ToSigned320(value));

    private static Signed320 ToSigned320(long value) =>
        Signed320.ExtendValue(
            Signed192.Signed(value));

    private static Signed704 ToSigned704(long value) =>
        Signed704.ExtendValue(ToSigned576(value));

    private static Signed704 CreateSigned704Word(
        int wordIndex,
        ulong value) =>
        wordIndex switch
        {
            10 => new(value, 0UL, 0UL, 0UL, 0UL, 0UL, 0UL, 0UL, 0UL, 0UL, 0UL),
            9 => new(0UL, value, 0UL, 0UL, 0UL, 0UL, 0UL, 0UL, 0UL, 0UL, 0UL),
            8 => new(0UL, 0UL, value, 0UL, 0UL, 0UL, 0UL, 0UL, 0UL, 0UL, 0UL),
            7 => new(0UL, 0UL, 0UL, value, 0UL, 0UL, 0UL, 0UL, 0UL, 0UL, 0UL),
            6 => new(0UL, 0UL, 0UL, 0UL, value, 0UL, 0UL, 0UL, 0UL, 0UL, 0UL),
            5 => new(0UL, 0UL, 0UL, 0UL, 0UL, value, 0UL, 0UL, 0UL, 0UL, 0UL),
            4 => new(0UL, 0UL, 0UL, 0UL, 0UL, 0UL, value, 0UL, 0UL, 0UL, 0UL),
            3 => new(0UL, 0UL, 0UL, 0UL, 0UL, 0UL, 0UL, value, 0UL, 0UL, 0UL),
            2 => new(0UL, 0UL, 0UL, 0UL, 0UL, 0UL, 0UL, 0UL, value, 0UL, 0UL),
            1 => new(0UL, 0UL, 0UL, 0UL, 0UL, 0UL, 0UL, 0UL, 0UL, value, 0UL),
            0 => new(0UL, 0UL, 0UL, 0UL, 0UL, 0UL, 0UL, 0UL, 0UL, 0UL, value),
            _ => throw new ArgumentOutOfRangeException(nameof(wordIndex)),
        };

    private readonly record struct ProjectionDescriptor(
        long Rational,
        long RadicandRootNumerator,
        long RadicandRootDenominator,
        long AxisRoot);

    private readonly record struct RadicalProjectionDescriptor(
        long Rational,
        long RadicandNumerator,
        long RadicandDenominator,
        long AxisSquared);

    private readonly record struct GeneralizedProjectionDescriptor(
        long Rational,
        long RadialCoefficient,
        long RadicandNumerator,
        long RadicandDenominator,
        long AxisSquared);
}
