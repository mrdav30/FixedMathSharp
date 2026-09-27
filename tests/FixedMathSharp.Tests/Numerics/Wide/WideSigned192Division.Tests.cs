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

    [Theory]
    [InlineData(1)]
    [InlineData(64)]
    [InlineData(128)]
    [InlineData(187)]
    public void GreatestCommonDivisor_RestoresCommonPowersOfTwo(int shift)
    {
        BigInteger common = BigInteger.One << shift;
        Assert.Equal(3 * common, Decode(WideArithmetic.GetGreatestCommonDivisor(
            Encode(9 * common), Encode(15 * common))));
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
