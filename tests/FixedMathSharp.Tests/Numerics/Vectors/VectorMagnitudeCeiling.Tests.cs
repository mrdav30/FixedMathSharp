using System.Numerics;
using FixedMathSharp.Geometry;
using Xunit;

namespace FixedMathSharp.Tests;

public sealed class VectorMagnitudeCeilingTests
{
    [Fact]
    public void MagnitudeCeiling_RoundsIrrationalLowRawLengthsOutward()
    {
        var planar = new Vector2d(
            Fixed64.MinIncrement,
            Fixed64.MinIncrement);
        var spatial = new Vector3d(
            Fixed64.MinIncrement,
            Fixed64.MinIncrement,
            Fixed64.MinIncrement);

        Assert.True(planar.TryGetMagnitudeCeiling(out Fixed64 planarCeiling));
        Assert.True(spatial.TryGetMagnitudeCeiling(out Fixed64 spatialCeiling));
        Assert.Equal(Fixed64.FromRaw(2L), planarCeiling);
        Assert.Equal(Fixed64.FromRaw(2L), spatialCeiling);
    }

    [Fact]
    public void MagnitudeCeiling_PreservesExactRootsAndRejectsOnlyFinalOverflow()
    {
        Assert.True(
            new Vector2d(3, 4).TryGetMagnitudeCeiling(
                out Fixed64 exact));
        Assert.Equal((Fixed64)5, exact);
        Assert.True(
            Vector3d.Zero.TryGetMagnitudeCeiling(
                out Fixed64 zero));
        Assert.Equal(Fixed64.Zero, zero);

        Assert.False(
            new Vector3d(
                Fixed64.MaxValue,
                Fixed64.MaxValue,
                Fixed64.Zero)
            .TryGetMagnitudeCeiling(out Fixed64 rejected));
        Assert.Equal(default, rejected);
    }

    [Fact]
    public void ScaledMagnitudeCeiling_RetainsProductsUntilOutwardRounding()
    {
        Assert.True(WideGeometry.TryGetScaledMagnitudeCeiling(
            new Vector3d(Fixed64.MinIncrement, Fixed64.Zero, Fixed64.Zero),
            new Vector3d(Fixed64.Half, Fixed64.One, Fixed64.One),
            out Fixed64 halfRaw));
        Assert.Equal(Fixed64.MinIncrement, halfRaw);

        Assert.True(WideGeometry.TryGetScaledMagnitudeCeiling(
            new Vector3d(Fixed64.One, Fixed64.MinIncrement, Fixed64.Zero),
            Vector3d.One,
            out Fixed64 irrational));
        Assert.Equal(Fixed64.FromRaw((1L << 32) + 1), irrational);

        // Large authored coordinates can be compensated by a small scale.
        Assert.True(WideGeometry.TryGetScaledMagnitudeCeiling(
            new Vector3d(Fixed64.FromRaw(3L << 60), Fixed64.FromRaw(-(1L << 62)), Fixed64.Zero),
            new Vector3d(Fixed64.FromRaw(1L << 6), Fixed64.FromRaw(1L << 6), Fixed64.FromRaw(1L << 6)),
            out Fixed64 compensated));
        Assert.Equal((Fixed64)20, compensated);
    }

    [Fact]
    public void ScaledMagnitudeCeiling_AppliesTheFinalScalarBoundaryExactly()
    {
        Assert.True(WideGeometry.TryGetScaledMagnitudeCeiling(
            Vector3d.Zero, new Vector3d(Fixed64.MinValue, Fixed64.MinValue, Fixed64.MinValue), out Fixed64 zero));
        Assert.Equal(Fixed64.Zero, zero);
        Assert.True(WideGeometry.TryGetScaledMagnitudeCeiling(
            new Vector3d(Fixed64.MaxValue, Fixed64.Zero, Fixed64.Zero),
            Vector3d.One,
            out Fixed64 maximum));
        Assert.Equal(Fixed64.MaxValue, maximum);

        CheckScaledMagnitude(new Vector3d(Fixed64.MaxValue, Fixed64.MinIncrement, Fixed64.Zero), Vector3d.One);
        CheckScaledMagnitude(new Vector3d(Fixed64.MinValue, Fixed64.Zero, Fixed64.Zero), Vector3d.One);
        var minima = new Vector3d(Fixed64.MinValue, Fixed64.MinValue, Fixed64.MinValue);
        CheckScaledMagnitude(minima, minima);
        CheckScaledMagnitude(minima, new Vector3d(Fixed64.Half, Fixed64.Half, Fixed64.Half));
    }

    [Fact]
    public void ScaledMagnitudeCeiling_MatchesIndependentIntegerEnclosureOracle()
    {
        long[] raw = { 0L, 1L, -1L, (1L << 31), (1L << 32) - 1, 1L << 32,
            (1L << 32) + 1, -(1L << 55), long.MaxValue, long.MinValue };
        for (int i = 0; i < raw.Length; i++)
        {
            for (int j = 0; j < raw.Length; j++)
            {
                var vector = new Vector3d(Fixed64.FromRaw(raw[i]),
                    Fixed64.FromRaw(raw[(i + j) % raw.Length]), Fixed64.FromRaw(raw[j]));
                var scale = new Vector3d(Fixed64.FromRaw(raw[j]),
                    Fixed64.FromRaw(raw[(i + 3 * j) % raw.Length]),
                    Fixed64.FromRaw(raw[(i + 2 * j + 1) % raw.Length]));
                CheckScaledMagnitude(vector, scale);
            }
        }
    }

    private static void CheckScaledMagnitude(Vector3d vector, Vector3d scale)
    {
        BigInteger x = (BigInteger)vector.X.m_rawValue * scale.X.m_rawValue;
        BigInteger y = (BigInteger)vector.Y.m_rawValue * scale.Y.m_rawValue;
        BigInteger z = (BigInteger)vector.Z.m_rawValue * scale.Z.m_rawValue;
        BigInteger squared = x * x + y * y + z * z;
        BigInteger maximum = (BigInteger)long.MaxValue << 32;
        bool expected = squared <= maximum * maximum;
        bool actual = WideGeometry.TryGetScaledMagnitudeCeiling(vector, scale, out Fixed64 result);
        Assert.Equal(expected, actual);
        if (!expected)
        {
            Assert.Equal(default, result);
            return;
        }

        // Search the result domain directly, independently of the production square root.
        long lower = 0, upper = long.MaxValue;
        while (lower < upper)
        {
            long middle = lower + ((upper - lower) >> 1);
            BigInteger enclosing = (BigInteger)middle << 32;
            if (enclosing * enclosing >= squared)
                upper = middle;
            else
                lower = middle + 1;
        }
        Assert.Equal(lower, result.m_rawValue);
    }
}
