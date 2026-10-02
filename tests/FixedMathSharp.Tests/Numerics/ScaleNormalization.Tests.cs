using FixedMathSharp.Random;
using System.Numerics;
using Xunit;

namespace FixedMathSharp.Tests;

public sealed class ScaleNormalizationTests
{
    [Fact]
    public void ScaleNormalization_PreservesPreviousRoundingAcrossRawEdges()
    {
        long unit = Fixed64.One.m_rawValue;
        long[] edges = { long.MinValue, long.MinValue + 1, long.MaxValue, long.MaxValue - 1,
            -(1L << 62), 1L << 62, -2 * unit, 2 * unit,
            -unit - 1, -unit, -unit + 1, unit - 1, unit, unit + 1, -1, 0, 1 };
        foreach (long x in edges)
        foreach (long y in edges)
        foreach (long z in edges)
            AssertPreviousRounding(x, y, z);
    }

    [Fact]
    public void ScaleNormalization_PreservesPreviousRoundingAcrossDeterministicRawCorpus()
    {
        var random = new DeterministicRandom(0x022UL);
        for (int i = 0; i < 2048; i++)
        {
            // Include both full-domain and tiny values, with deliberately unequal
            // component scales so sub-raw ratios and dominant-axis ties are exercised.
            int shift = i % 64;
            AssertPreviousRounding(unchecked((long)random.NextU64()) >> shift,
                unchecked((long)random.NextU64()) >> ((shift + 21) % 64),
                unchecked((long)random.NextU64()) >> ((shift + 42) % 64));
        }
    }

    private static void AssertPreviousRounding(long x, long y, long z)
    {
        var planar = new Vector2d(Fixed64.FromRaw(x), Fixed64.FromRaw(y));
        if (x != 0 || y != 0)
        {
            Fixed64 scale = FixedMath.Max(planar.X.Abs(), planar.Y.Abs());
            Vector2d scaled = planar / scale;
            AssertScaleInvariant(scaled.X, scaled.Y, Fixed64.Zero, scale, x, y, 0);
            Fixed64 previousMagnitude = FixedMath.GetScaledMagnitude(
                scaled.X, scaled.Y, Fixed64.Zero, Fixed64.Zero);
            Assert.Equal(scaled / previousMagnitude, Vector2d.GetScaleNormalized(planar));
        }

        var spatial = new Vector3d(planar.X, planar.Y, Fixed64.FromRaw(z));
        if (x != 0 || y != 0 || z != 0)
        {
            Fixed64 scale = FixedMath.Max(spatial.X.Abs(), FixedMath.Max(spatial.Y.Abs(), spatial.Z.Abs()));
            Vector3d scaled = spatial / scale;
            AssertScaleInvariant(scaled.X, scaled.Y, scaled.Z, scale, x, y, z);
            Fixed64 previousMagnitude = FixedMath.GetScaledMagnitude(
                scaled.X, scaled.Y, scaled.Z, Fixed64.Zero);
            Assert.Equal(scaled / previousMagnitude, Vector3d.GetScaleNormalized(spatial));
        }
    }

    private static void AssertScaleInvariant(
        Fixed64 x, Fixed64 y, Fixed64 z, Fixed64 scale, long rawX, long rawY, long rawZ)
    {
        Assert.Equal(Fixed64.One, FixedMath.Max(x.Abs(), FixedMath.Max(y.Abs(), z.Abs())));
        Assert.Equal(RoundRatio(rawX, scale.m_rawValue), x.m_rawValue);
        Assert.Equal(RoundRatio(rawY, scale.m_rawValue), y.m_rawValue);
        Assert.Equal(RoundRatio(rawZ, scale.m_rawValue), z.m_rawValue);
    }

    private static long RoundRatio(long raw, long denominator)
    {
        BigInteger quotient = BigInteger.DivRem(
            BigInteger.Abs(new BigInteger(raw)) * Fixed64.One.m_rawValue,
            denominator, out BigInteger remainder);
        int midpoint = (2 * remainder).CompareTo(denominator);
        if (midpoint > 0 || (midpoint == 0 && !quotient.IsEven))
            quotient++;
        return (long)(raw < 0 ? -quotient : quotient);
    }
}
