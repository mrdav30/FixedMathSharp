using System;
using System.Numerics;
using System.Runtime.CompilerServices;
using Xunit;

namespace FixedMathSharp.Tests;

public sealed class WideMultiquadraticComparisonTests
{
    [Theory]
    [InlineData(new ulong[] { 4, 9, 25 }, new int[] { 1, 1, -1 }, 0)]
    [InlineData(new ulong[] { 2, 8, 18 }, new int[] { 1, 1, -1 }, 0)]
    [InlineData(new ulong[] { 7, 7, 0 }, new int[] { 1, -1, 1 }, 0)]
    [InlineData(new ulong[] { 7, 7, 9 }, new int[] { 1, -1, 1 }, 1)]
    [InlineData(new ulong[] { 7, 9 }, new int[] { 0, 1 }, 1)]
    [InlineData(new ulong[] { 9, 4 }, new int[] { 1, 1 }, 1)]
    [InlineData(new ulong[] { 9, 4 }, new int[] { -1, -1 }, -1)]
    [InlineData(new ulong[] { 2, 3 }, new int[] { 1, -1 }, -1)]
    [InlineData(new ulong[] { 3, 2 }, new int[] { 1, -1 }, 1)]
    public void Sign_HandlesPerfectSquaresDependenciesAndZeroTerms(
        ulong[] radicands,
        int[] signs,
        int expected)
    {
        Assert.Equal(
            expected,
            WideArithmetic.GetLinearRadicalSumSign(
                radicands,
                radicandWordCount: 1,
                signs));
    }

    [Fact]
    public void Sign_IsPermutationInvariantAndAntisymmetric()
    {
        ulong[] radicands = { 2, 3, 5, 7, 11 };
        int[] signs = { 1, -1, 1, -1, 1 };
        int expected = WideArithmetic.GetLinearRadicalSumSign(
            radicands,
            radicandWordCount: 1,
            signs);
        Assert.NotEqual(0, expected);

        ulong[] reversedRadicands = { 11, 7, 5, 3, 2 };
        int[] reversedSigns = { 1, -1, 1, -1, 1 };
        Assert.Equal(
            expected,
            WideArithmetic.GetLinearRadicalSumSign(
                reversedRadicands,
                radicandWordCount: 1,
                reversedSigns));

        for (int index = 0; index < signs.Length; index++)
            signs[index] = -signs[index];
        Assert.Equal(
            -expected,
            WideArithmetic.GetLinearRadicalSumSign(
                radicands,
                radicandWordCount: 1,
            signs));
    }

    [Fact]
    public void Sign_PreservesTransitiveOrderingAcrossRadicalSums()
    {
        int firstBeforeSecond =
            WideArithmetic.GetLinearRadicalSumSign(
                new ulong[] { 2, 11, 3, 10 },
                radicandWordCount: 1,
                new[] { 1, 1, -1, -1 });
        int secondBeforeThird =
            WideArithmetic.GetLinearRadicalSumSign(
                new ulong[] { 3, 10, 5, 8 },
                radicandWordCount: 1,
                new[] { 1, 1, -1, -1 });
        int firstBeforeThird =
            WideArithmetic.GetLinearRadicalSumSign(
                new ulong[] { 2, 11, 5, 8 },
                radicandWordCount: 1,
                new[] { 1, 1, -1, -1 });

        Assert.True(firstBeforeSecond < 0);
        Assert.True(secondBeforeThird < 0);
        Assert.True(firstBeforeThird < 0);
    }

    [Fact]
    public void NormalizedComparisons_PreserveSignedAndDegenerateTerms()
    {
        Assert.Equal(
            0,
            WideArithmetic.CompareSignedNormalizedMagnitudes(
                new ulong[] { 0UL },
                1,
                new ulong[] { 3UL },
                new ulong[] { 0UL },
                -1,
                new ulong[] { 5UL }));
        Assert.True(
            WideArithmetic.CompareSignedNormalizedMagnitudes(
                new ulong[] { 2UL },
                1,
                new ulong[] { 4UL },
                new ulong[] { 3UL },
                1,
                new ulong[] { 16UL }) > 0);
        Assert.True(
            WideArithmetic.CompareSignedNormalizedMagnitudes(
                new ulong[] { 2UL },
                -1,
                new ulong[] { 4UL },
                new ulong[] { 3UL },
                -1,
                new ulong[] { 16UL }) < 0);
        Assert.True(
            WideArithmetic.CompareSignedNormalizedMagnitudes(
                new ulong[] { 1UL },
                -1,
                new ulong[] { 1UL },
                new ulong[] { 1UL },
                1,
                new ulong[] { 1UL }) < 0);

        Assert.Equal(
            1,
            WideArithmetic.GetSignedMagnitudeAndSquareRootSign(
                new ulong[] { 5UL },
                1,
                new ulong[] { 0UL },
                1,
                new ulong[] { 7UL }));
        Assert.Equal(
            1,
            WideArithmetic.GetSignedMagnitudeAndSquareRootSign(
                new ulong[] { 5UL },
                1,
                new ulong[] { 2UL },
                1,
                new ulong[] { 3UL }));
        Assert.Equal(
            -1,
            WideArithmetic.GetSignedMagnitudeAndSquareRootSign(
                new ulong[] { 0UL },
                1,
                new ulong[] { 2UL },
                -1,
                new ulong[] { 3UL }));
        Assert.Equal(
            0,
            WideArithmetic.GetSignedMagnitudeAndSquareRootSign(
                new ulong[] { 2UL },
                1,
                new ulong[] { 1UL },
                -1,
                new ulong[] { 4UL }));
        Assert.Equal(
            -1,
            WideArithmetic.GetSignedMagnitudeAndSquareRootSign(
                new ulong[] { 1UL },
                1,
                new ulong[] { 1UL },
                -1,
                new ulong[] { 4UL }));
    }

    [Fact]
    public void Sign_AgreesWithHighPrecisionOracleMatrix()
    {
        var random = new System.Random(0x5EED);
        for (int sample = 0; sample < 128; sample++)
        {
            int count = 2 + sample % 5;
            ulong[] radicands = new ulong[count * 2];
            int[] signs = new int[count];
            for (int index = 0; index < count; index++)
            {
                radicands[index * 2] =
                    unchecked((ulong)random.NextInt64(1, long.MaxValue));
                radicands[index * 2 + 1] =
                    unchecked((ulong)random.NextInt64(0, 1L << 24));
                signs[index] = random.Next(2) == 0 ? -1 : 1;
            }

            int expected = GetOracleSign(
                radicands,
                radicandWordCount: 2,
                signs);
            int actual =
                WideArithmetic.GetLinearRadicalSumSign(
                    radicands,
                    radicandWordCount: 2,
                    signs);
            Assert.Equal(expected, actual);
        }
    }

    [Fact]
    public void Sign_IsDirtyStackDeterministicAndDoesNotAllocateAfterWarmup()
    {
        ulong[] radicands =
        {
            0xFFFF_FFFF_FFFF_FFC5UL, 0x0000_0000_0000_0011UL,
            0xFFFF_FFFF_FFFF_FF6DUL, 0x0000_0000_0000_0013UL,
            0xFFFF_FFFF_FFFF_FEFFUL, 0x0000_0000_0000_0017UL,
            0xFFFF_FFFF_FFFF_FEA9UL, 0x0000_0000_0000_001DUL,
            0xFFFF_FFFF_FFFF_FE8BUL, 0x0000_0000_0000_001FUL,
        };
        int[] signs = { 1, -1, 1, -1, 1 };
        int expected = WideArithmetic.GetLinearRadicalSumSign(
            radicands,
            radicandWordCount: 2,
            signs);

        for (int iteration = 0; iteration < 16; iteration++)
        {
            PolluteStack(unchecked((ulong)iteration + 1UL));
            Assert.Equal(
                expected,
                WideArithmetic.GetLinearRadicalSumSign(
                    radicands,
                    radicandWordCount: 2,
                    signs));
        }

        _ = WideArithmetic.GetLinearRadicalSumSign(
            radicands,
            radicandWordCount: 2,
            signs);
        long before =
            GC.GetAllocatedBytesForCurrentThread();
        int checksum = 0;
        for (int iteration = 0; iteration < 16; iteration++)
        {
            checksum +=
                WideArithmetic.GetLinearRadicalSumSign(
                    radicands,
                    radicandWordCount: 2,
                    signs);
        }
        long after =
            GC.GetAllocatedBytesForCurrentThread();

        Assert.Equal(expected * 16, checksum);
        Assert.Equal(before, after);
    }

    private static int GetOracleSign(
        ReadOnlySpan<ulong> radicands,
        int radicandWordCount,
        ReadOnlySpan<int> signs)
    {
        const int precision = 512;
        BigInteger total = BigInteger.Zero;
        for (int index = 0; index < signs.Length; index++)
        {
            BigInteger radicand = BigInteger.Zero;
            for (int word = radicandWordCount - 1;
                 word >= 0;
                 word--)
            {
                radicand <<= 64;
                radicand += radicands[
                    index * radicandWordCount + word];
            }
            BigInteger root =
                IntegerSquareRoot(radicand << (precision * 2));
            total += signs[index] * root;
        }
        return total.Sign;
    }

    private static BigInteger IntegerSquareRoot(
        BigInteger value)
    {
        if (value.IsZero)
            return BigInteger.Zero;
        BigInteger estimate =
            BigInteger.One
            << ((GetBitLength(value) + 1) >> 1);
        while (true)
        {
            BigInteger next =
                (estimate + value / estimate) >> 1;
            if (next >= estimate)
                return estimate;
            estimate = next;
        }
    }

    private static int GetBitLength(BigInteger value)
    {
        byte[] bytes = value.ToByteArray();
        int highest = bytes[^1];
        int bits = (bytes.Length - 1) * 8;
        while (highest != 0)
        {
            bits++;
            highest >>= 1;
        }
        return bits;
    }

    [MethodImpl(MethodImplOptions.NoInlining)]
    private static void PolluteStack(ulong seed)
    {
        Span<ulong> words = stackalloc ulong[2_048];
        for (int index = 0; index < words.Length; index++)
        {
            words[index] =
                seed
                + unchecked((ulong)index * 0x9E37_79B9UL);
        }
        GC.KeepAlive(words[
            unchecked((int)seed) & 2_047]);
    }
}
