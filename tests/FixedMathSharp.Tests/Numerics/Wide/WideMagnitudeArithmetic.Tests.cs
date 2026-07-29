using System;
using System.Numerics;
using Xunit;

namespace FixedMathSharp.Tests;

public sealed class WideMagnitudeArithmeticTests
{
    [Fact]
    public void LengthAndBitLength_EmptyZeroAndHighestWord_AreExact()
    {
        Assert.Equal(0, WideArithmetic.GetActiveMagnitudeLength(ReadOnlySpan<ulong>.Empty));
        Assert.Equal(0, WideArithmetic.GetMagnitudeBitLength(ReadOnlySpan<ulong>.Empty));

        ulong[] zero = { 0UL, 0UL, 0UL };
        Assert.Equal(0, WideArithmetic.GetActiveMagnitudeLength(zero));
        Assert.Equal(0, WideArithmetic.GetMagnitudeBitLength(zero));

        ulong[] oneWord = { 1UL, 0UL, 0UL };
        Assert.Equal(1, WideArithmetic.GetActiveMagnitudeLength(oneWord));
        Assert.Equal(1, WideArithmetic.GetMagnitudeBitLength(oneWord));

        ulong[] highestWord = { 0UL, 0UL, 1UL << 63 };
        Assert.Equal(3, WideArithmetic.GetActiveMagnitudeLength(highestWord));
        Assert.Equal(192, WideArithmetic.GetMagnitudeBitLength(highestWord));
    }

    [Fact]
    public void AddWord_FullCarryAndTruncatedOverflow_AreStable()
    {
        ulong[] words = { ulong.MaxValue, ulong.MaxValue, 123UL };
        WideArithmetic.AddWord(words, 0, 1UL);
        Assert.Equal(new ulong[] { 0UL, 0UL, 124UL }, words);

        ulong[] truncated = { ulong.MaxValue, ulong.MaxValue };
        WideArithmetic.AddWord(truncated, 0, 1UL);
        Assert.Equal(new ulong[] { 0UL, 0UL }, truncated);
    }

    [Fact]
    public void AddSubtractAndCompare_EqualWidthUnequalActiveLengths_ClearDirtyDestination()
    {
        ulong[] first = { ulong.MaxValue, ulong.MaxValue, 0UL };
        ulong[] second = { 1UL, 0UL, 0UL };
        ulong[] sum = { 9UL, 9UL, 9UL };
        WideArithmetic.AddEqualMagnitudes(first, second, sum);
        Assert.Equal(new ulong[] { 0UL, 0UL, 1UL }, sum);

        ulong[] difference = { 9UL, 9UL, 9UL };
        WideArithmetic.SubtractEqualMagnitudes(sum, second, difference);
        Assert.Equal(first, difference);
        Assert.True(WideArithmetic.CompareMagnitudeEqualLength(sum, difference) > 0);
        Assert.Equal(0, WideArithmetic.CompareMagnitudeEqualLength(sum, sum));
    }

    [Fact]
    public void MultiplyMagnitudes_MaximumWorkspace_MatchesBigIntegerAndClearsDestination()
    {
        ulong[] emptyProduct = { 9UL, 9UL };
        WideArithmetic.MultiplyMagnitudes(
            ReadOnlySpan<ulong>.Empty,
            new ulong[] { ulong.MaxValue },
            emptyProduct);
        Assert.Equal(new ulong[] { 0UL, 0UL }, emptyProduct);

        ulong[] oneWordProduct = { 9UL, 9UL };
        WideArithmetic.MultiplyMagnitudes(
            new ulong[] { ulong.MaxValue },
            new ulong[] { ulong.MaxValue },
            oneWordProduct);
        Assert.Equal(
            new BigInteger(ulong.MaxValue) * ulong.MaxValue,
            ToBigInteger(oneWordProduct));

        ulong[] left = new ulong[320];
        ulong[] right = new ulong[320];
        Array.Fill(left, ulong.MaxValue);
        Array.Fill(right, ulong.MaxValue);
        ulong[] product = new ulong[640];
        Array.Fill(product, 0xDEAD_BEEF_DEAD_BEEFUL);

        WideArithmetic.MultiplyMagnitudes(left, right, product);

        BigInteger expected = ToBigInteger(left) * ToBigInteger(right);
        Assert.Equal(expected, ToBigInteger(product));
    }

    private static BigInteger ToBigInteger(ReadOnlySpan<ulong> words)
    {
        BigInteger value = BigInteger.Zero;
        for (int index = words.Length - 1; index >= 0; index--)
            value = (value << 64) | words[index];
        return value;
    }
}
