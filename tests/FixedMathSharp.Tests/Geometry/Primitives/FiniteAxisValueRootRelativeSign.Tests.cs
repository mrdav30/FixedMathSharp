using System;
using System.Numerics;
using FixedMathSharp.Geometry;
using Xunit;

namespace FixedMathSharp.Tests;

public sealed class FiniteAxisValueRootRelativeSignTests
{
    [Theory]
    [InlineData(1, -1)]
    [InlineData(-1, 1)]
    public void LargeCommonHeight_DoesNotChangeSeparatedLinearSign(int scaleSign, int expected)
    {
        // alpha=sqrt(1/2)<3/4, since1/2<9/16. The non-power-of-two
        // content prevents a trailing-zero normalization from doing this work.
        BigInteger scale = ((BigInteger.One << 4096) + 3) * scaleSign;
        AssertSign(new[] { -3 * scale, 4 * scale }, expected);
    }

    [Theory]
    [InlineData(-1)]
    [InlineData(0)]
    [InlineData(1)]
    public void DiscardedLowCoefficientBits_RemainDecisiveAfterCancellation(int offset)
    {
        // Q=K*(2x²-1)+offset. Early relative-precision passes discard
        // offset, but Q(alpha)=offset exactly; uncertainty must not mean zero.
        BigInteger scale = (BigInteger.One << 256) + 17;
        AssertSign(new[] { -scale + offset, BigInteger.Zero, 2 * scale }, offset);
    }

    [Fact]
    public void NegativeCoefficientTruncation_IsTowardZeroNotFloor()
    {
        BigInteger scale = BigInteger.One << 193;
        // Q(alpha)=(-scale+1)/2+scale/2=1/2.
        AssertSign(new[] { scale / 2, BigInteger.Zero, -scale + 1 }, 1);
    }

    private static void AssertSign(BigInteger[] query, int expected)
    {
        ulong[] defining = { 1, 0, 2 };
        sbyte[] definingSigns = { -1, 0, 1 };
        var cell = new ulong[WideFiniteAxisIntersection.GetFiniteValueRootCellWords(defining, definingSigns)];
        Assert.True(WideFiniteAxisIntersection.TryGetFiniteValueRoot(defining, definingSigns, 0, cell,
            out FiniteAxisValueRoot root));
        ulong[] originalCell = (ulong[])cell.Clone();
        int originalShift = root.DenominatorShift;
        int words = 1;
        foreach (BigInteger coefficient in query)
            words = Math.Max(words, (BigInteger.Abs(coefficient).GetByteCount() + 7) / 8);
        var values = new ulong[query.Length * words];
        var signs = new sbyte[query.Length];
        for (int i = 0; i < query.Length; i++)
        {
            signs[i] = (sbyte)query[i].Sign;
            BigInteger magnitude = BigInteger.Abs(query[i]);
            for (int j = 0; j < words; j++)
            {
                values[i * words + j] = (ulong)(magnitude & ulong.MaxValue);
                magnitude >>= 64;
            }
            Assert.Equal(BigInteger.Zero, magnitude);
        }
        ulong[] originalQuery = (ulong[])values.Clone();
        Assert.Equal(expected, WideFiniteAxisIntersection.GetSignAtFiniteValueRoot(root, values, signs));
        Assert.Equal(originalCell, cell);
        Assert.Equal(originalShift, root.DenominatorShift);
        Assert.Equal(originalQuery, values);
    }
}
