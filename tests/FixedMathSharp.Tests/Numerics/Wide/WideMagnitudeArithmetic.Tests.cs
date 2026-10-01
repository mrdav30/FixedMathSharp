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

    [Theory]
    [InlineData(0)]
    [InlineData(1)]
    [InlineData(3)]
    [InlineData(7)]
    [InlineData(12)]
    [InlineData(16)]
    public void MultiplyMagnitudes_DenseAndSparseRowsPreserveCarriesAndTruncation(int outputWords)
    {
        ulong state = 0x7913_C658_FFAA_4242UL;
        BigInteger mask = (BigInteger.One << (64 * outputWords)) - 1;
        for (int sample = 0; sample < 48; sample++)
        {
            ulong[] left = new ulong[8];
            ulong[] right = new ulong[8];
            for (int index = 0; index < 8; index++)
            {
                state ^= state << 13;
                state ^= state >> 7;
                state ^= state << 17;
                left[index] = sample % 3 == 0 && index % 2 == 0 ? 0 : state;
                right[index] = sample % 3 == 1 && index % 2 != 0 ? 0 : ~state;
            }
            if (sample == 0)
            {
                Array.Fill(left, ulong.MaxValue);
                Array.Fill(right, ulong.MaxValue);
            }
            ulong[] originalLeft = (ulong[])left.Clone();
            ulong[] originalRight = (ulong[])right.Clone();
            ulong[] product = new ulong[outputWords];
            Array.Fill(product, ulong.MaxValue);
            WideArithmetic.MultiplyMagnitudes(left, right, product);
            Assert.Equal((ToBigInteger(left) * ToBigInteger(right)) & mask, ToBigInteger(product));
            Assert.Equal(originalLeft, left);
            Assert.Equal(originalRight, right);
        }
    }

    [Fact]
    public void DivideMagnitudes_ProducesExactQuotientAndRemainderAcrossWordBoundaries()
    {
        BigInteger[] numerators =
        {
            0, 1, ulong.MaxValue, BigInteger.One << 64,
            (BigInteger.One << 128) - 1, (BigInteger.One << 191) + 37,
            (BigInteger.One << 256) - (BigInteger.One << 129) + 7
        };
        BigInteger[] divisors =
        {
            1, 3, uint.MaxValue, (BigInteger)uint.MaxValue + 1,
            ulong.MaxValue, (BigInteger.One << 64) + 1,
            (BigInteger.One << 127) + 3, (BigInteger.One << 192) - 1
        };
        foreach (BigInteger numerator in numerators)
        foreach (BigInteger divisor in divisors)
            AssertDivision(numerator, divisor);
    }

    [Fact]
    public void DivideMagnitudes_SeededDenseWordsMatchBigInteger()
    {
        ulong state = 0xDA12_E592_738A_456BUL;
        for (int sample = 0; sample < 96; sample++)
        {
            BigInteger numerator = 0;
            BigInteger divisor = 0;
            for (int word = 0; word < 8; word++)
            {
                state ^= state << 13;
                state ^= state >> 7;
                state ^= state << 17;
                numerator = (numerator << 64) | state;
                if (word < sample % 8 + 1)
                    divisor = (divisor << 64) | (state ^ 0xA571_2513_390B_CDEFUL);
            }
            AssertDivision(numerator, divisor);
        }
        // Top-word estimation needs two corrections in the first pair. The
        // second pair needs a full subtraction/add-back after its top two
        // words alone permit a quotient one above the correct value.
        BigInteger radix = BigInteger.One << 64;
        BigInteger twoWordDivisor = (radix / 2) * radix + radix - 1;
        AssertDivision((radix - 2) * twoWordDivisor - 1, twoWordDivisor);
        BigInteger threeWordDivisor = (BigInteger.One << 191) + 1;
        AssertDivision(2 * threeWordDivisor - 1, threeWordDivisor);
        AssertDivision((radix / 2 - 1) * radix * radix, twoWordDivisor);
        AssertDivision(twoWordDivisor * radix - 1, twoWordDivisor);
        BigInteger smallLowWord = (radix / 2) * radix + 1;
        AssertDivision(smallLowWord * radix - 1, smallLowWord);
    }

    [Fact]
    public void MagnitudeGcd_PreservesOddContentAndZeroOperands()
    {
        BigInteger common = ((BigInteger.One << 191) + 27) * 15;
        foreach ((BigInteger first, BigInteger second) in new[]
        {
            (BigInteger.Zero, BigInteger.Zero), (common, BigInteger.Zero),
            (BigInteger.Zero, common), (common * 21, common * 35),
            (common * 21 + 1, common * 35),
            ((BigInteger.One << 255) - 1, (BigInteger.One << 192) - 1)
        })
        {
            ulong[] left = EncodeMagnitude(first, 8);
            ulong[] right = EncodeMagnitude(second, 8);
            ulong[] scratch = new ulong[17];
            WideArithmetic.GetMagnitudeGreatestCommonDivisor(left, right, scratch);
            Assert.Equal(BigInteger.GreatestCommonDivisor(first, second), ToBigInteger(left));
        }
    }

    [Theory]
    [InlineData(0)]
    [InlineData(1)]
    [InlineData(63)]
    [InlineData(64)]
    [InlineData(65)]
    [InlineData(129)]
    public void ShiftedSignedAddition_MatchesExactIntegerArithmetic(int shift)
    {
        BigInteger[] values = { 0, 1, -1, ulong.MaxValue, -((BigInteger.One << 129) + 7) };
        foreach (BigInteger initial in values)
        foreach (BigInteger addend in values)
        {
            ulong[] destination = EncodeMagnitude(BigInteger.Abs(initial), 8);
            ulong[] source = EncodeMagnitude(BigInteger.Abs(addend), 3);
            int sign = initial.Sign;
            WideArithmetic.AddShiftedSignedMagnitude(source, addend.Sign, shift, destination, ref sign);
            BigInteger expected = initial + (addend << shift);
            Assert.Equal(expected.Sign, sign);
            Assert.Equal(BigInteger.Abs(expected), ToBigInteger(destination));
        }
        // Equality and a borrow extending across whole words.
        BigInteger equal = (BigInteger.One << (shift + 65)) - (BigInteger.One << shift);
        ulong[] magnitude = EncodeMagnitude(equal, 8);
        int equalSign = 1;
        WideArithmetic.AddShiftedSignedMagnitude(EncodeMagnitude((BigInteger.One << 65) - 1, 2),
            -1, shift, magnitude, ref equalSign);
        Assert.Equal(0, equalSign);
        Assert.Equal(BigInteger.Zero, ToBigInteger(magnitude));
    }

    [Theory]
    [InlineData(0)]
    [InlineData(63)]
    [InlineData(64)]
    [InlineData(65)]
    [InlineData(129)]
    public void ShiftedSignedAddition_PropagatesCarryBeyondPaddedSourceSupport(int shift)
    {
        // Carry must cross limbs beyond the shifted source, while the full
        // destination and its untouched low/high padding remain exact.
        BigInteger initial = (BigInteger.One << 320) - 1;
        BigInteger addend = ulong.MaxValue;
        ulong[] source = EncodeMagnitude(addend, 3);
        ulong[] original = (ulong[])source.Clone();
        foreach (int sign in new[] { -1, 1 })
        {
            ulong[] destination = EncodeMagnitude(initial, 8);
            int actualSign = sign;
            WideArithmetic.AddShiftedSignedMagnitude(source, sign, shift, destination, ref actualSign);
            Assert.Equal(initial + (addend << shift), ToBigInteger(destination));
            Assert.Equal(sign, actualSign);
            Assert.Equal(original, source);
        }
    }

    [Theory]
    [InlineData(0, 0UL)]
    [InlineData(64, ulong.MaxValue)]
    [InlineData(int.MaxValue, ulong.MaxValue)]
    public void ShiftedSignedAddition_PreservesTruncationAtTheDestinationBoundary(int shift, ulong expected)
    {
        // Internal limb truncation does not independently normalize the sign.
        ulong[] destination = { ulong.MaxValue };
        int sign = 1;
        WideArithmetic.AddShiftedSignedMagnitude(new ulong[] { 1 }, 1, shift, destination, ref sign);
        Assert.Equal(expected, destination[0]);
        Assert.Equal(1, sign);
    }

    private static void AssertDivision(BigInteger numerator, BigInteger divisor)
    {
        ulong[] left = EncodeMagnitude(numerator, 9);
        ulong[] right = EncodeMagnitude(divisor, 9);
        ulong[] quotient = new ulong[9];
        ulong[] remainder = new ulong[9];
        ulong[] scratch = new ulong[19];
        Array.Fill(quotient, ulong.MaxValue);
        Array.Fill(remainder, ulong.MaxValue);
        Array.Fill(scratch, ulong.MaxValue);
        WideArithmetic.DivideMagnitudes(left, right, quotient, remainder, scratch);
        Assert.Equal(numerator / divisor, ToBigInteger(quotient));
        Assert.Equal(numerator % divisor, ToBigInteger(remainder));
        Assert.Equal(numerator, ToBigInteger(left));
        Assert.Equal(divisor, ToBigInteger(right));

        WideArithmetic.DivideMagnitudes(left, right, left, remainder, scratch);
        Assert.Equal(numerator / divisor, ToBigInteger(left));
        Assert.Equal(numerator % divisor, ToBigInteger(remainder));
        left = EncodeMagnitude(numerator, 9);

        // Remainder may overwrite the original numerator; omitting the
        // quotient is the Euclidean content-reduction operation.
        WideArithmetic.DivideMagnitudes(left, right, Span<ulong>.Empty, left, scratch);
        Assert.Equal(numerator % divisor, ToBigInteger(left));
    }

    private static ulong[] EncodeMagnitude(BigInteger value, int words)
    {
        var result = new ulong[words];
        for (int index = 0; index < words; index++)
        {
            result[index] = (ulong)(value & ulong.MaxValue);
            value >>= 64;
        }
        Assert.Equal(BigInteger.Zero, value);
        return result;
    }
}
