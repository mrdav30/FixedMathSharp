using System.Numerics;
using Xunit;

namespace FixedMathSharp.Tests;

public sealed class Fixed64Divide128Tests
{
    [Theory]
    [InlineData(1UL, 0UL, 0x8000000000000000UL, 2UL, 0UL, 0, 0)]
    [InlineData(0x8000000000000000UL, 0UL, 0x8000000000000001UL,
        0xFFFFFFFFFFFFFFFEUL, 2UL, 1, 0)]
    [InlineData(0x7FFFFFFFFFFFFFFDUL, 0x0000000100000000UL, 0x80000000FFFFFFFFUL,
        0xFFFFFFFDFFFFFFFFUL, 0x7FFFFFFFFFFFFFFFUL, 2, 2)]
    [InlineData(0x0000000080000000UL, 0xFFFFFFFF00000000UL, 0x80000000FFFFFFFFUL,
        0x0000000100000000UL, 0UL, 0, 0)]
    [InlineData(0x0000000100000000UL, 0x0000000100000000UL, 0x8000000000000001UL,
        0x00000001FFFFFFFFUL, 0x7FFFFFFF00000001UL, 1, 1)]
    public void Divide128By64_NormalizedTrialBoundariesAreExact(
        ulong high, ulong low, ulong divisor, ulong expectedQuotient, ulong expectedRemainder,
        int firstCorrectionCount, int secondCorrectionCount)
    {
        ulong quotient = Fixed64.Divide128By64(high, low, divisor, out ulong remainder);
        Assert.Equal(expectedQuotient, quotient);
        Assert.Equal(expectedRemainder, remainder);
        AssertDivision(high, low, divisor);

        // Independently certify which Algorithm D correction paths these
        // vectors exercise, using full integer division instead of its loop.
        BigInteger radix = BigInteger.One << 32;
        BigInteger first = (BigInteger)high * radix + (low >> 32);
        BigInteger firstDigit = BigInteger.DivRem(first, divisor, out BigInteger firstRemainder);
        BigInteger secondDigit = (firstRemainder * radix + (uint)low) / divisor;
        ulong divisorHigh = divisor >> 32;
        Assert.Equal((BigInteger)firstCorrectionCount, high / divisorHigh - firstDigit);
        Assert.Equal((BigInteger)secondCorrectionCount, firstRemainder / divisorHigh - secondDigit);
    }

    [Fact]
    public void Divide128By64_FastPathsAndQuotientLimitsMatchBigInteger()
    {
        ulong[] divisors =
        {
            1, 2, 3, uint.MaxValue - 1UL, uint.MaxValue, (ulong)uint.MaxValue + 1,
            (ulong)uint.MaxValue + 2, (1UL << 63) - 1, 1UL << 63,
            (1UL << 63) + 1, ulong.MaxValue - 1, ulong.MaxValue
        };
        ulong[] lows = { 0, 1, uint.MaxValue, 1UL << 32, 1UL << 63, ulong.MaxValue - 1, ulong.MaxValue };
        foreach (ulong divisor in divisors)
        foreach (ulong high in new[] { 0UL, divisor / 2, divisor - 1 })
        foreach (ulong low in lows)
            AssertDivision(high, low, divisor);
    }

    [Fact]
    public void Divide128By64_EveryNormalizationShiftPreservesCarriesAndRemainders()
    {
        for (int topBit = 32; topBit < 64; topBit++)
        {
            ulong divisor = (1UL << topBit) | 3UL;
            AssertDivision(divisor - 1, ulong.MaxValue, divisor);
            AssertDivision(divisor / 2, 1UL, divisor);
            AssertDivision(1UL, 0UL, divisor);
        }
    }

    [Fact]
    public void Divide128By64_DeterministicDenseInputsMatchBigInteger()
    {
        ulong state = 0xE21A9F07D53CB641UL;
        for (int sample = 0; sample < 256; sample++)
        {
            ulong divisor = NextWord(ref state) | 1UL;
            ulong high = NextWord(ref state) % divisor;
            ulong low = NextWord(ref state);
            AssertDivision(high, low, divisor);
        }
    }

    private static void AssertDivision(ulong high, ulong low, ulong divisor)
    {
        BigInteger numerator = ((BigInteger)high << 64) | low;
        BigInteger expected = BigInteger.DivRem(numerator, divisor, out BigInteger expectedRemainder);
        ulong quotient = Fixed64.Divide128By64(high, low, divisor, out ulong remainder);
        Assert.Equal((ulong)expected, quotient);
        Assert.Equal((ulong)expectedRemainder, remainder);
    }

    private static ulong NextWord(ref ulong state)
    {
        state ^= state << 13;
        state ^= state >> 7;
        state ^= state << 17;
        return state;
    }
}
