using System;
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
}
