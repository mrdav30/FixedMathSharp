using System;
using System.Numerics;
using FixedMathSharp.Geometry;
using Xunit;

namespace FixedMathSharp.Tests;

public sealed class CylinderPairNormalRoundingTests
{
    [Theory]
    [InlineData(1UL, 1, 0L)]
    [InlineData(3UL, 1, 2L)]
    [InlineData(1UL, -1, 0L)]
    [InlineData(3UL, -1, -2L)]
    public void AlgebraicNormal_RoundsExactHalfRawComponentsToEven(ulong odd, int orientation, long expectedRaw)
    {
        // alpha = sqrt(2^66-odd^2)/2^33. The direction (odd,2^33*alpha,0)
        // has length exactly 2^33, so X is exactly odd/2 raw units. Y is
        // strictly within half a raw unit of one for both authored odd values.
        ulong square = odd * odd;
        ulong[] defining = { unchecked(0UL - square), 3UL, 0UL, 0UL, 0UL, 4UL };
        sbyte[] definingSigns = { -1, 0, 1 };
        Span<ulong> cell = stackalloc ulong[32];
        Assert.True(WideFiniteAxisIntersection.TryGetFiniteValueRoot(defining, definingSigns,
            0, cell, out FiniteAxisValueRoot root));
        ulong[] gradient = { odd, 0UL, 0UL, 1UL << 33, 0UL, 0UL };
        sbyte[] signs = { 1, 0, 0, 1, 0, 0 };
        Vector3d normal = ConvexContactValueRoot.GetNormalizedDirection(ref root, gradient, signs, orientation);
        Assert.Equal(expectedRaw, normal.X.m_rawValue);
        Assert.Equal(orientation * (1L << 32), normal.Y.m_rawValue);
        Assert.Equal(Fixed64.Zero, normal.Z);
    }

    [Fact]
    public void SquaredLength_PreservesTheCarryAcrossThreeFullWidthComponents()
    {
        ulong[] gradient = { ulong.MaxValue, ulong.MaxValue, ulong.MaxValue };
        sbyte[] signs = { 1, -1, 1 };
        ulong[] squaredLength = new ulong[3];
        sbyte[] squaredSigns = new sbyte[1];
        ConvexContactValueRoot.BuildSquaredLength(gradient, signs, squaredLength, squaredSigns);
        BigInteger actual = ((BigInteger)squaredLength[2] << 128)
            | ((BigInteger)squaredLength[1] << 64) | squaredLength[0];
        Assert.Equal(3 * BigInteger.Pow(ulong.MaxValue, 2), actual);
        Assert.Equal(1, squaredSigns[0]);
    }

    [Theory]
    [InlineData(2)]
    [InlineData(CylinderPairRimFeatures.CoefficientWords)]
    public void AlgebraicNormal_PreservesPaddedCoefficientsAndSummationCarry(int words)
    {
        // alpha=sqrt(1/2). A shared factor (2^64-1)*(1-alpha) cancels
        // exactly, leaving the direction (3,4,0)/5. Its squared polynomial
        // crosses the second limb; padding must not determine rounding.
        ulong[] defining = { 1UL, 0UL, 2UL };
        sbyte[] definingSigns = { -1, 0, 1 };
        Span<ulong> cell = stackalloc ulong[32];
        Assert.True(WideFiniteAxisIntersection.TryGetFiniteValueRoot(defining, definingSigns,
            0, cell, out FiniteAxisValueRoot root));
        ulong[] gradient = new ulong[6 * words];
        sbyte[] signs = { 1, -1, 1, -1, 0, 0 };
        for (int index = 0; index < 4; index++)
        {
            BigInteger value = (index < 2 ? 3 : 4) * (BigInteger)ulong.MaxValue;
            gradient[index * words] = (ulong)(value & ulong.MaxValue);
            gradient[index * words + 1] = (ulong)(value >> 64);
        }
        ulong[] original = (ulong[])gradient.Clone();
        Vector3d normal = ConvexContactValueRoot.GetNormalizedDirection(ref root, gradient, signs, 1);
        Assert.Equal(Fixed64.FromRaw(2576980378L), normal.X);
        Assert.Equal(Fixed64.FromRaw(3435973837L), normal.Y);
        Assert.Equal(Fixed64.Zero, normal.Z);
        Assert.Equal(original, gradient);
        Assert.Equal(new sbyte[] { 1, -1, 1, -1, 0, 0 }, signs);
    }
}
