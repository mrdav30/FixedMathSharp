using System.Numerics;
using Xunit;

namespace FixedMathSharp.Tests;

public sealed class Fixed64RawRatioTests
{
    [Fact]
    public void ThreeWordRawRatio_ShouldMatchExactIntegerRoundingAcrossItsSignedDomain()
    {
        BigInteger[] values = { 0, 1, -1, 5, -5, 7, -7, long.MaxValue, long.MinValue,
            BigInteger.One << 64, -(BigInteger.One << 64),
            (BigInteger.One << 64) + 1, (BigInteger.One << 128) + 3,
            15 * (BigInteger.One << 63), 21 * (BigInteger.One << 63),
            15 * (BigInteger.One << 63) - 1, 15 * (BigInteger.One << 63) + 1,
            -(15 * (BigInteger.One << 63)), -(21 * (BigInteger.One << 63)),
            (BigInteger.One << 191) - 1, -(BigInteger.One << 191) };
        BigInteger[] divisors = { 0, 1, -1, 2, -2, 3 * (BigInteger.One << 64),
            -3 * (BigInteger.One << 64), BigInteger.One << 64,
            2 * (BigInteger.One << 64), (BigInteger)ulong.MaxValue << 64, -((BigInteger)ulong.MaxValue << 64),
            (BigInteger.One << 128) + 1, (BigInteger.One << 191) - 1, -(BigInteger.One << 191) };
        foreach (BigInteger numerator in values)
        foreach (BigInteger denominator in divisors)
        {
            BigInteger raw = 0;
            if (denominator != 0)
            {
                raw = BigInteger.DivRem(BigInteger.Abs(numerator), BigInteger.Abs(denominator), out BigInteger remainder);
                int half = (2 * remainder).CompareTo(BigInteger.Abs(denominator));
                if (half > 0 || half == 0 && !raw.IsEven) raw++;
                if (numerator.Sign != denominator.Sign) raw = -raw;
            }
            bool representable = denominator != 0 && raw >= long.MinValue && raw <= long.MaxValue;
            Assert.Equal(representable, Fixed64.TryGetSignedRawRatio(ToSigned(numerator), ToSigned(denominator), out Fixed64 actual));
            Assert.Equal(representable ? (long)raw : 0, actual.m_rawValue);
        }
    }

    private static Signed192 ToSigned(BigInteger value)
    {
        BigInteger bits = value & ((BigInteger.One << 192) - 1);
        BigInteger mask = ulong.MaxValue;
        return new Signed192((ulong)((bits >> 128) & mask), (ulong)((bits >> 64) & mask), (ulong)(bits & mask));
    }
}
