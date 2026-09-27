using System;
using System.Numerics;
using System.Runtime.CompilerServices;
using System.Threading;
using FixedMathSharp.Geometry;
using Xunit;

namespace FixedMathSharp.Tests;

public sealed class FiniteAxisValueRootTests
{
    [Fact]
    public void Isolate_EnumeratesAllEightPositiveRootsInOrder()
    {
        BigInteger[] polynomial = { 1 };
        for (int root = 1; root <= 8; root++)
            polynomial = Multiply(polynomial, new BigInteger[] { -root, 9 });
        for (int ordinal = 0; ordinal < 8; ordinal++)
            AssertRoot(polynomial, ordinal, ordinal + 1, 9);
        AssertNoRoot(polynomial, 8);
    }

    [Fact]
    public void Isolate_CountsRepeatedRootsOnceAndExcludesZero()
    {
        BigInteger[] polynomial = { 0, 0, 1 };
        for (int repeat = 0; repeat < 3; repeat++)
            polynomial = Multiply(polynomial, new BigInteger[] { -1, 3 });
        for (int repeat = 0; repeat < 2; repeat++)
            polynomial = Multiply(polynomial, new BigInteger[] { -2, 3 });
        polynomial = Multiply(polynomial, new BigInteger[] { -1, 1 });
        AssertRoot(polynomial, 0, 1, 3);
        AssertRoot(polynomial, 1, 2, 3);
        AssertRoot(polynomial, 2, 1, 1);
        AssertNoRoot(polynomial, 3);
    }

    [Fact]
    public void Isolate_PreservesDyadicRootsAndNegativeLeadingCoefficient()
    {
        BigInteger[] polynomial = { -1 };
        for (int root = 1; root <= 8; root++)
            polynomial = Multiply(polynomial, new BigInteger[] { -root, 8 });
        for (int ordinal = 0; ordinal < 8; ordinal++)
            AssertRoot(polynomial, ordinal, ordinal + 1, 8);
    }

    [Fact]
    public void Isolate_RejectsConstantsAndRootsOutsideThePhysicalInterval()
    {
        AssertNoRoot(new BigInteger[] { -1, 3 }, -1);
        AssertNoRoot(new BigInteger[] { 0 }, 0);
        AssertNoRoot(new BigInteger[] { -1 }, 0);
        AssertNoRoot(new BigInteger[] { 0, 0, 1 }, 0);
        AssertNoRoot(new BigInteger[] { -2, -1, 1 }, 0); // -1 and +2.
    }

    [Theory]
    [InlineData(16)]
    [InlineData(116)]
    [InlineData(1024)]
    public void Isolate_SizesScratchFromValuesRatherThanPaddedInputSlots(int words) =>
        AssertRoot(new BigInteger[] { -1, 3 }, 0, 1, 3, words);

    [Fact]
    public void Sign_IdentifiesTheSelectedRootRatherThanAnySharedFactor()
    {
        BigInteger[] polynomial = Multiply(new BigInteger[] { -1, 3 },
            new BigInteger[] { -2, 3 });
        polynomial = Multiply(polynomial, polynomial);
        AssertSign(polynomial, 0, new BigInteger[] { -1, 3 }, 0);
        AssertSign(polynomial, 1, new BigInteger[] { -1, 3 }, 1);
        AssertSign(polynomial, 0, new BigInteger[] { -2, 3 }, -1);
        AssertSign(polynomial, 1, new BigInteger[] { -2, 3 }, 0);
    }

    [Fact]
    public void Sign_CertifiesEqualityAtAnIrrationalRepeatedRoot()
    {
        BigInteger[] factor = { -1, 0, 2 };
        BigInteger[] polynomial = Multiply(Multiply(factor, factor), new BigInteger[] { 1, 1 });
        AssertSign(polynomial, 0, factor, 0);
        AssertSign(polynomial, 0, Multiply(factor, new BigInteger[] { -2, 7, 3 }), 0);
        AssertSign(polynomial, 0, new BigInteger[] { -1, 1 }, -1);
        AssertSign(polynomial, 0, new BigInteger[] { -1, 2 }, 1);
    }

    [Fact]
    public void Sign_DistinguishesTinyNonzeroValuesFromExactZero()
    {
        BigInteger scale = BigInteger.One << 200;
        BigInteger[] polynomial = { -1, 3 };
        AssertSign(polynomial, 0, new BigInteger[] { -scale, 3 * scale }, 0);
        AssertSign(polynomial, 0, new BigInteger[] { -scale - 1, 3 * scale }, -1);
        AssertSign(polynomial, 0, new BigInteger[] { -scale + 1, 3 * scale }, 1);
    }

    [Fact]
    public void Sign_HandlesDyadicRootsConstantsAndZeroQueries()
    {
        BigInteger[] polynomial = { -1, 2 };
        AssertSign(polynomial, 0, new BigInteger[] { 1 }, 1);
        AssertSign(polynomial, 0, new BigInteger[] { -1 }, -1);
        AssertSign(polynomial, 0, new BigInteger[] { 0, 0 }, 0);
        AssertSign(polynomial, 0, new BigInteger[] { -1, 0, 4 }, 0);
    }

    [Fact]
    public void Compare_OrdersSelectedRootsAndProvesSharedValuesEqual()
    {
        AssertComparison(new BigInteger[] { -1, 3 }, 0, new BigInteger[] { 1, -3 }, 0, 0);
        BigInteger[] first = Multiply(new BigInteger[] { -1, 3 }, new BigInteger[] { -2, 3 });
        BigInteger[] second = Multiply(first, new BigInteger[] { 1, 1 });
        AssertComparison(first, 0, second, 0, 0);
        AssertComparison(first, 0, second, 1, -1);
        AssertComparison(first, 1, second, 0, 1);
        AssertComparison(first, 0, first, 1, -1);
        AssertComparison(new BigInteger[] { -1, 0, 2 }, 0,
            new BigInteger[] { 1, 0, -4, 0, 4 }, 0, 0);
    }

    [Fact]
    public void Compare_HandlesRationalSingletonsAndVeryCloseDistinctRoots()
    {
        // The irrational root lies just above 3/4, inside the first refined
        // cell whose lower endpoint is that rational singleton.
        BigInteger[] justAbove = { -(9 * (BigInteger.One << 126) + 1), 0, BigInteger.One << 130 };
        AssertComparison(justAbove, 0, new BigInteger[] { -3, 4 }, 0, 1);
        AssertComparison(new BigInteger[] { -3, 4 }, 0, justAbove, 0, -1);
        BigInteger scale = BigInteger.One << 100;
        AssertComparison(new BigInteger[] { -1, 2 }, 0, new BigInteger[] { -1, 2 }, 0, 0);
        AssertComparison(new BigInteger[] { -1, 2 }, 0, new BigInteger[] { 1, -2 }, 0, 0);
        AssertComparison(new BigInteger[] { -1, 2 }, 0, new BigInteger[] { -scale - 1, 2 * scale }, 0, -1);
        AssertComparison(new BigInteger[] { -scale - 1, 2 * scale }, 0, new BigInteger[] { -1, 2 }, 0, 1);
        AssertComparison(new BigInteger[] { -scale, 3 * scale }, 0,
            new BigInteger[] { -scale + 1, 3 * scale }, 0, 1);
    }

    [Fact]
    public void CompareDyadic_CountsTheSelectedRootAtAndAroundAnExactThreshold()
    {
        BigInteger[] polynomial = Multiply(new BigInteger[] { -3, 8 }, new BigInteger[] { -7, 8 });
        polynomial = Multiply(polynomial, polynomial);
        AssertDyadicComparison(polynomial, 0, 3, 3, 0);
        AssertDyadicComparison(polynomial, 1, 3, 3, 1);
        AssertDyadicComparison(polynomial, 0, 7, 3, -1);
        AssertDyadicComparison(polynomial, 1, 7, 3, 0);
        AssertDyadicComparison(polynomial, 0, 2, 3, 1);
        AssertDyadicComparison(polynomial, 0, 4, 3, -1);
        AssertDyadicComparison(polynomial, 0, 0, 0, 1);
        AssertDyadicComparison(polynomial, 0, 1, 0, -1);
        AssertDyadicComparison(new BigInteger[] { -1, 1 }, 0, 1, 0, 0);
        AssertDyadicComparison(new BigInteger[] { -1, 1 }, 0, 3, 1, -1);
        AssertDyadicComparison(new BigInteger[] { -1, 1 }, 0, 2, 0, -1);
    }

    [Fact]
    public void CompareDyadic_PreservesOpenCellEndpointsAndAlignedSingletons()
    {
        BigInteger[] polynomial = { -1, 0, 2 }; // sqrt(1/2) in (1/2,1).
        AssertDyadicComparison(polynomial, 0, 1, 1, 1);
        AssertDyadicComparison(polynomial, 0, 1, 0, -1);
        AssertDyadicComparison(polynomial, 0, 3, 2, -1);
        AssertDyadicComparison(new BigInteger[] { -1, 2 }, 0, 8, 4, 0);
        AssertDyadicComparison(new BigInteger[] { -1, 2 }, 0, 7, 4, 1);
        AssertDyadicComparison(new BigInteger[] { -1, 2 }, 0, 9, 4, -1);
    }

    private static void AssertDyadicComparison(BigInteger[] polynomial, int ordinal,
        ulong numerator, int shift, int expected)
    {
        Encode(polynomial, out ulong[] coefficients, out sbyte[] signs);
        ulong[] cell = new ulong[WideFiniteAxisIntersection.GetFiniteValueRootCellWords(coefficients, signs)];
        Assert.True(WideFiniteAxisIntersection.TryGetFiniteValueRoot(coefficients, signs,
            ordinal, cell, out FiniteAxisValueRoot root));
        Assert.Equal(expected, WideFiniteAxisIntersection.CompareFiniteValueRootToDyadic(root,
            new[] { numerator }, shift));
    }

    [Fact]
    public void IsolateAndSign_RespectOneMiBWorkerStackAtTheCoefficientHeightCeiling()
    {
        BigInteger[] polynomial = { 1 };
        for (int index = 1; index <= 8; index++)
            polynomial = Multiply(polynomial, new BigInteger[] { -index, 9 });
        int height = 0;
        foreach (BigInteger coefficient in polynomial)
            height = Math.Max(height, (int)BigInteger.Abs(coefficient).GetBitLength());
        for (int index = 0; index < polynomial.Length; index++)
            polynomial[index] <<= 7416 - height;
        Encode(polynomial, out ulong[] coefficients, out sbyte[] signs);
        ulong[] cell = new ulong[WideFiniteAxisIntersection.GetFiniteValueRootCellWords(coefficients, signs)];
        Exception? failure = null;
        bool found = false;
        int querySign = 0;
        var worker = new Thread(() =>
        {
            try
            {
                found = WideFiniteAxisIntersection.TryGetFiniteValueRoot(coefficients, signs, 0, cell,
                    out FiniteAxisValueRoot root);
                querySign = WideFiniteAxisIntersection.GetSignAtFiniteValueRoot(root,
                    new ulong[] { 1, 10 }, new sbyte[] { -1, 1 });
            }
            catch (Exception exception)
            {
                failure = exception;
            }
        }, 1024 * 1024);
        worker.Start();
        worker.Join();
        Assert.Null(failure);
        Assert.True(found);
        Assert.Equal(1, querySign); // 10/9-1 = 1/9.
    }

    [Fact]
    public void Isolate_PrimitiveHighCoefficientsPreserveCallerHeadroomOnOneMiBStack()
    {
        // The roots k/16+2^-900 are distinct and well separated, so this
        // exercises high primitive PRS coefficients, not 900 bisections.
        // The leading coefficient is 2^7200 and the constant is odd:
        // primitive normalization cannot remove the high coefficient size.
        BigInteger denominator = BigInteger.One << 900;
        BigInteger spacing = BigInteger.One << 896;
        BigInteger[] polynomial = { 1 };
        for (int index = 1; index <= 8; index++)
            polynomial = Multiply(polynomial, new[] { -(index * spacing + 1), denominator });
        BigInteger content = BigInteger.Zero;
        int height = 0;
        foreach (BigInteger coefficient in polynomial)
        {
            content = BigInteger.GreatestCommonDivisor(content, coefficient);
            height = Math.Max(height, (int)BigInteger.Abs(coefficient).GetBitLength());
        }
        Assert.Equal(BigInteger.One, content);
        Assert.InRange(height, 7201, 7440);
        Encode(polynomial, out ulong[] coefficients, out sbyte[] signs);
        ulong[] cell = new ulong[WideFiniteAxisIntersection.GetFiniteValueRootCellWords(coefficients, signs)];
        Exception? failure = null;
        bool found = false, rational = false;
        BigInteger lower = BigInteger.Zero;
        int shift = 0, ordinal = -1;
        var worker = new Thread(() =>
        {
            try
            {
                found = IsolateWithLiveCallerBuffer(coefficients, signs, cell, out FiniteAxisValueRoot root);
                if (found)
                {
                    lower = ToInteger(root.LowerNumerator);
                    shift = root.DenominatorShift;
                    ordinal = root.Ordinal;
                    rational = root.IsRational;
                }
            }
            catch (Exception exception)
            {
                failure = exception;
            }
        }, 1024 * 1024);
        worker.Start();
        worker.Join();
        Assert.Null(failure);
        Assert.True(found);
        Assert.Equal(0, ordinal);
        BigInteger expected = (spacing + 1) << shift;
        if (rational)
            Assert.Equal(expected, lower * denominator);
        else
        {
            Assert.True(lower * denominator < expected);
            Assert.True(expected < (lower + 1) * denominator);
            Assert.True((lower + 1) * denominator <= (2 * spacing + 1) << shift);
        }
    }

    [MethodImpl(MethodImplOptions.NoInlining)]
    private static bool IsolateWithLiveCallerBuffer(ulong[] coefficients, sbyte[] signs,
        ulong[] cell, out FiniteAxisValueRoot root)
    {
        Span<ulong> caller = stackalloc ulong[8192];
        for (int index = 0; index < caller.Length; index++)
            caller[index] = 0xA55A_0FF0_1234_5678UL ^ (ulong)index;
        bool found = WideFiniteAxisIntersection.TryGetFiniteValueRoot(coefficients, signs, 0, cell, out root);
        // Consume every word after isolation, keeping all 64 KiB live in
        // the containing frame throughout the primitive PRS computation.
        for (int index = 0; index < caller.Length; index++)
            if (caller[index] != (0xA55A_0FF0_1234_5678UL ^ (ulong)index))
                throw new InvalidOperationException("Root isolation changed its caller's stack buffer.");
        return found;
    }

    [Fact]
    public void IsolateAndSign_AllocateNothingAndLeaveBorrowedInputUnchanged()
    {
        Encode(new BigInteger[] { 1, 0, -4, 0, 4 }, out ulong[] coefficients, out sbyte[] signs);
        Encode(new BigInteger[] { -1, 0, 2 }, out ulong[] query, out sbyte[] querySigns);
        ulong[] original = (ulong[])coefficients.Clone();
        ulong[] cell = new ulong[WideFiniteAxisIntersection.GetFiniteValueRootCellWords(coefficients, signs)];
        bool found = false;
        int result = 7;
        long allocated = FixedMathTestHelper.MeasureWarmedAllocations(() =>
        {
            found = WideFiniteAxisIntersection.TryGetFiniteValueRoot(coefficients, signs, 0, cell,
                out FiniteAxisValueRoot root);
            result = WideFiniteAxisIntersection.GetSignAtFiniteValueRoot(root, query, querySigns);
        });
        Assert.True(found);
        Assert.Equal(0, result);
        Assert.Equal(0, allocated);
        Assert.Equal(original, coefficients);
    }

    private static void AssertComparison(BigInteger[] first, int firstOrdinal,
        BigInteger[] second, int secondOrdinal, int expected)
    {
        Encode(first, out ulong[] firstCoefficients, out sbyte[] firstSigns);
        Encode(second, out ulong[] secondCoefficients, out sbyte[] secondSigns);
        ulong[] firstCell = new ulong[WideFiniteAxisIntersection.GetFiniteValueRootCellWords(firstCoefficients, firstSigns)];
        ulong[] secondCell = new ulong[WideFiniteAxisIntersection.GetFiniteValueRootCellWords(secondCoefficients, secondSigns)];
        Assert.True(WideFiniteAxisIntersection.TryGetFiniteValueRoot(firstCoefficients, firstSigns,
            firstOrdinal, firstCell, out FiniteAxisValueRoot firstRoot));
        Assert.True(WideFiniteAxisIntersection.TryGetFiniteValueRoot(secondCoefficients, secondSigns,
            secondOrdinal, secondCell, out FiniteAxisValueRoot secondRoot));
        Assert.Equal(expected, WideFiniteAxisIntersection.CompareFiniteValueRoots(firstRoot, secondRoot));
    }

    private static void AssertSign(BigInteger[] polynomial, int ordinal, BigInteger[] query, int expected)
    {
        Encode(polynomial, out ulong[] coefficients, out sbyte[] signs);
        ulong[] cell = new ulong[WideFiniteAxisIntersection.GetFiniteValueRootCellWords(coefficients, signs)];
        Assert.True(WideFiniteAxisIntersection.TryGetFiniteValueRoot(coefficients, signs,
            ordinal, cell, out FiniteAxisValueRoot root));
        Encode(query, out ulong[] queryCoefficients, out sbyte[] querySigns);
        Assert.Equal(expected, WideFiniteAxisIntersection.GetSignAtFiniteValueRoot(root,
            queryCoefficients, querySigns));
    }

    private static void AssertRoot(BigInteger[] polynomial, int ordinal, int numerator, int denominator,
        int paddedWords = 1)
    {
        Encode(polynomial, out ulong[] coefficients, out sbyte[] signs, paddedWords);
        ulong[] cell = new ulong[WideFiniteAxisIntersection.GetFiniteValueRootCellWords(coefficients, signs)];
        Assert.True(WideFiniteAxisIntersection.TryGetFiniteValueRoot(coefficients, signs,
            ordinal, cell, out FiniteAxisValueRoot root));
        BigInteger lower = ToInteger(root.LowerNumerator);
        BigInteger target = (BigInteger)numerator << root.DenominatorShift;
        if (root.IsRational)
            Assert.Equal(target, lower * denominator);
        else
        {
            Assert.True(lower * denominator < target);
            Assert.True(target < (lower + 1) * denominator);
        }
        Assert.Equal(ordinal, root.Ordinal);
    }

    private static void AssertNoRoot(BigInteger[] polynomial, int ordinal)
    {
        Encode(polynomial, out ulong[] coefficients, out sbyte[] signs);
        ulong[] cell = new ulong[WideFiniteAxisIntersection.GetFiniteValueRootCellWords(coefficients, signs)];
        Assert.False(WideFiniteAxisIntersection.TryGetFiniteValueRoot(coefficients, signs,
            ordinal, cell, out _));
    }

    private static BigInteger[] Multiply(BigInteger[] left, BigInteger[] right)
    {
        var result = new BigInteger[left.Length + right.Length - 1];
        for (int i = 0; i < left.Length; i++)
        for (int j = 0; j < right.Length; j++)
            result[i + j] += left[i] * right[j];
        return result;
    }

    private static void Encode(BigInteger[] polynomial, out ulong[] coefficients, out sbyte[] signs,
        int paddedWords = 1)
    {
        int bits = 0;
        foreach (BigInteger value in polynomial)
            bits = Math.Max(bits, (int)BigInteger.Abs(value).GetBitLength());
        int words = Math.Max(paddedWords, (bits + 63) / 64);
        coefficients = new ulong[words * polynomial.Length];
        signs = new sbyte[polynomial.Length];
        for (int index = 0; index < polynomial.Length; index++)
        {
            BigInteger value = BigInteger.Abs(polynomial[index]);
            signs[index] = (sbyte)polynomial[index].Sign;
            for (int word = 0; word < words; word++)
            {
                coefficients[index * words + word] = (ulong)(value & ulong.MaxValue);
                value >>= 64;
            }
        }
    }

    private static BigInteger ToInteger(ReadOnlySpan<ulong> words)
    {
        BigInteger result = 0;
        for (int index = words.Length - 1; index >= 0; index--)
            result = (result << 64) | words[index];
        return result;
    }
}
