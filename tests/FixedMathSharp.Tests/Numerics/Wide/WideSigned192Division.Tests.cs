using System.Numerics;
using Xunit;

namespace FixedMathSharp.Tests;

public sealed class WideSigned192DivisionTests
{
    [Theory]
    [InlineData(1)]
    [InlineData(63)]
    [InlineData(64)]
    [InlineData(65)]
    [InlineData(127)]
    [InlineData(128)]
    [InlineData(129)]
    [InlineData(162)]
    [InlineData(190)]
    public void DivideExact_PreservesBitsAcrossAllThreeWords(int bit)
    {
        BigInteger numerator = BigInteger.One << bit;
        foreach (int sign in new[] { -1, 1 })
        {
            Signed192 result = WideArithmetic.DivideExactSigned192(
                Encode(sign * numerator), Encode(2));
            Assert.Equal(sign * (numerator >> 1), Decode(result));
        }
    }

    [Theory]
    [InlineData(64)]
    [InlineData(128)]
    [InlineData(162)]
    public void DivideExact_DistinguishesOneFromAHighWordDivisor(int bit)
    {
        // The old ellipse-only shortcut inspected Middle and Low but not High.
        BigInteger divisor = (BigInteger.One << bit) + 1;
        Assert.Equal(new BigInteger(3), Decode(WideArithmetic.DivideExactSigned192(
            Encode(3 * divisor), Encode(divisor))));
        Assert.Equal(new BigInteger(-3), Decode(WideArithmetic.DivideExactSigned192(
            Encode(-3 * divisor), Encode(divisor))));
    }

    [Fact]
    public void DivideExact_ZeroOneAndMaximumAdmittedMagnitude_AreExact()
    {
        BigInteger maximum = (BigInteger.One << 191) - 1;
        Assert.Equal(BigInteger.Zero, Decode(WideArithmetic.DivideExactSigned192(
            default, Encode(maximum))));
        foreach (int sign in new[] { -1, 1 })
        {
            Assert.Equal(sign * maximum, Decode(WideArithmetic.DivideExactSigned192(
                Encode(sign * maximum), Encode(1))));
            Assert.Equal(new BigInteger(sign), Decode(WideArithmetic.DivideExactSigned192(
                Encode(sign * maximum), Encode(maximum))));
            BigInteger quotient = maximum / 3;
            Assert.Equal(sign * quotient, Decode(WideArithmetic.DivideExactSigned192(
                Encode(sign * quotient * 3), Encode(3))));
        }
    }

    [Fact]
    public void GreatestCommonDivisor_PreservesSignedOddContentAndZeroOperands()
    {
        BigInteger odd = (BigInteger.One << 162) + 27;
        BigInteger maximum = (BigInteger.One << 191) - 1;
        foreach ((BigInteger first, BigInteger second) in new[]
        {
            (BigInteger.Zero, BigInteger.Zero), (odd, BigInteger.Zero),
            (BigInteger.Zero, -odd), (-odd * 21, odd * 35),
            (odd * 35, -odd * 21), (maximum, maximum - 1),
            (-maximum, maximum), (odd * 21 + 1, odd * 35),
            (2 * odd, 3 * odd)
        })
        {
            Assert.Equal(BigInteger.GreatestCommonDivisor(first, second),
                Decode(WideArithmetic.GetGreatestCommonDivisor(Encode(first), Encode(second))));
        }
    }

    [Fact]
    public void IntegerContentReduction_DoesNotAllocate()
    {
        BigInteger common = (BigInteger.One << 162) + 27;
        Signed192 first = Encode(-21 * common);
        Signed192 second = Encode(35 * common);
        Signed192 gcd = default;
        Signed192 quotient = default;
        long allocated = FixedMathTestHelper.MeasureWarmedAllocations(() =>
        {
            gcd = WideArithmetic.GetGreatestCommonDivisor(first, second);
            quotient = WideArithmetic.DivideExactSigned192(first, gcd);
        });
        Assert.Equal(0, allocated);
        Assert.Equal(7 * common, Decode(gcd));
        Assert.Equal(new BigInteger(-3), Decode(quotient));
    }

    [Theory]
    [InlineData(0)]
    [InlineData(1)]
    [InlineData(31)]
    [InlineData(32)]
    [InlineData(63)]
    [InlineData(64)]
    [InlineData(65)]
    [InlineData(95)]
    [InlineData(96)]
    [InlineData(127)]
    [InlineData(128)]
    [InlineData(129)]
    [InlineData(162)]
    [InlineData(187)]
    [InlineData(188)]
    [InlineData(189)]
    [InlineData(190)]
    public void GreatestCommonDivisor_BulkPowerBoundaries_MatchOracle(int shift)
    {
        BigInteger common = BigInteger.One << shift;
        BigInteger first = (shift <= 187 ? 9 : 1) * common;
        BigInteger second = (shift <= 187 ? 15 : shift < 190 ? 3 : 1) * common;
        AssertGcdSignsAndOrders(first, second);
    }

    [Fact]
    public void GreatestCommonDivisor_NormalizedUnitAgainstLargeOddContent_MatchesOracle()
    {
        BigInteger maximum = (BigInteger.One << 191) - 1;
        foreach (int shift in new[] { 0, 32, 63, 64, 95, 127, 128, 162, 190 })
        {
            BigInteger common = BigInteger.One << shift;
            BigInteger other = (maximum >> shift) << shift;
            AssertGcdSignsAndOrders(common, other);
        }
    }

    [Fact]
    public void GreatestCommonDivisor_AsymmetricThreeWordCarries_MatchOracle()
    {
        BigInteger maximum = (BigInteger.One << 191) - 1;
        foreach ((BigInteger first, BigInteger second) in new[]
        {
            ((BigInteger.One << 190) + (BigInteger.One << 130) + (BigInteger.One << 64),
             (BigInteger.One << 189) + (BigInteger.One << 128) + (BigInteger.One << 63)),
            ((BigInteger.One << 190) + (BigInteger.One << 128) + (BigInteger.One << 127),
             (BigInteger.One << 189) + (BigInteger.One << 65) + (BigInteger.One << 64)),
            ((BigInteger.One << 190) + (BigInteger.One << 129) + (BigInteger.One << 128),
             (BigInteger.One << 188) + (BigInteger.One << 127) + (BigInteger.One << 126)),
            // Restore a multiword odd gcd across partial/whole word shifts.
            (3 * ((BigInteger.One << 125) + (BigInteger.One << 63) + 1) * (BigInteger.One << 63),
             5 * ((BigInteger.One << 125) + (BigInteger.One << 63) + 1) * (BigInteger.One << 63)),
            (3 * ((BigInteger.One << 120) + (BigInteger.One << 64) + 1) * (BigInteger.One << 64),
             5 * ((BigInteger.One << 120) + (BigInteger.One << 64) + 1) * (BigInteger.One << 64)),
            // Low==1 alone is not integer one; these operands share odd content3.
            ((BigInteger.One << 128) + (BigInteger.One << 64) + 1,
             3 * ((BigInteger.One << 128) + 3)),
            (maximum, maximum - 2),
            (maximum, (BigInteger.One << 190) + 1)
        })
            AssertGcdSignsAndOrders(first, second);

        for (int index = 1; index <= 12; index++)
        {
            // Fixed integer patterns vary every word and subtraction carry.
            BigInteger first = (BigInteger.One << 190) + ((BigInteger)index << 128)
                + ((BigInteger)(index * index + 1) << 64) + 2 * index + 1;
            BigInteger second = (BigInteger.One << 189) + ((BigInteger)(31 - index) << 129)
                + ((BigInteger)(index * 17 + 3) << 63) + 4 * index + 1;
            AssertGcdSignsAndOrders(first, second);
        }
    }

    [Theory]
    [InlineData(0)]
    [InlineData(1)]
    [InlineData(31)]
    [InlineData(32)]
    [InlineData(63)]
    [InlineData(64)]
    [InlineData(65)]
    [InlineData(95)]
    [InlineData(96)]
    [InlineData(127)]
    [InlineData(128)]
    [InlineData(129)]
    [InlineData(162)]
    [InlineData(189)]
    [InlineData(190)]
    public void DivideExact_PowerDivisorsAcrossEveryWord_MatchOracle(int shift)
    {
        BigInteger maximum = (BigInteger.One << 191) - 1;
        BigInteger divisor = BigInteger.One << shift;
        BigInteger expected = maximum >> shift;
        BigInteger numerator = expected << shift;
        foreach (int sign in new[] { -1, 1 })
            Assert.Equal(sign * expected, Decode(WideArithmetic.DivideExactSigned192(
                Encode(sign * numerator), Encode(divisor))));
    }

    [Fact]
    public void DivideExact_NonPowerDivisorsWithTrailingZeroesAndHighWords_MatchOracle()
    {
        BigInteger maximum = (BigInteger.One << 191) - 1;
        foreach (BigInteger divisor in new[]
        {
            new BigInteger(3), 3 * (BigInteger.One << 32),
            (BigInteger.One << 64) + 1,
            (BigInteger.One << 64) + (BigInteger.One << 32),
            (BigInteger.One << 128) + 1,
            (BigInteger.One << 128) + (BigInteger.One << 64),
            (BigInteger.One << 162) + 1,
            ((BigInteger.One << 160) + 1) << 20,
            (BigInteger.One << 190) - 1
        })
        {
            BigInteger expected = maximum / divisor;
            BigInteger numerator = expected * divisor;
            foreach (int sign in new[] { -1, 1 })
                Assert.Equal(sign * expected, Decode(WideArithmetic.DivideExactSigned192(
                    Encode(sign * numerator), Encode(divisor))));
        }
    }

    [Fact]
    public void IntegerContentReduction_CommonBinaryScale_DoesNotAllocate()
    {
        BigInteger common = BigInteger.One << 64;
        BigInteger expected = -((BigInteger.One << 120) + 1);
        Signed192 first = Encode(expected * common), second = Encode(common);
        Signed192 gcd = default, quotient = default;
        long allocated = FixedMathTestHelper.MeasureWarmedAllocations(() =>
        {
            gcd = WideArithmetic.GetGreatestCommonDivisor(first, second);
            quotient = WideArithmetic.DivideExactSigned192(first, gcd);
        });
        Assert.Equal(0, allocated);
        Assert.Equal(common, Decode(gcd));
        Assert.Equal(expected, Decode(quotient));
    }

    private static void AssertGcdSignsAndOrders(BigInteger first, BigInteger second)
    {
        BigInteger expected = BigInteger.GreatestCommonDivisor(first, second);
        foreach (int firstSign in new[] { -1, 1 })
            foreach (int secondSign in new[] { -1, 1 })
            {
                Signed192 left = Encode(firstSign * first), right = Encode(secondSign * second);
                Assert.Equal(expected, Decode(WideArithmetic.GetGreatestCommonDivisor(left, right)));
                Assert.Equal(expected, Decode(WideArithmetic.GetGreatestCommonDivisor(right, left)));
            }
    }

    private static Signed192 Encode(BigInteger value)
    {
        Assert.True(BigInteger.Abs(value) < (BigInteger.One << 191));
        if (value.Sign < 0)
            value += BigInteger.One << 192;
        return new Signed192((ulong)(value >> 128),
            (ulong)((value >> 64) & ulong.MaxValue), (ulong)(value & ulong.MaxValue));
    }

    private static BigInteger Decode(Signed192 value)
    {
        BigInteger result = ((BigInteger)value.High << 128)
            | ((BigInteger)value.Middle << 64) | value.Low;
        return (value.High & (1UL << 63)) == 0 ? result : result - (BigInteger.One << 192);
    }
}
