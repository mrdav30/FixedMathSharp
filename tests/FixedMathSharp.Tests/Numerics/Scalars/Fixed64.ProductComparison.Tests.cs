using Xunit;

namespace FixedMathSharp.Tests;

public sealed class Fixed64ProductComparisonTests
{
    [Fact]
    public void CompareProducts_OrdersExactProductsWithoutRoundingOrSaturation()
    {
        Fixed64 aboveHalfDomain = Fixed64.FromRaw(
            (Fixed64.MaxValue.m_rawValue / 2L) + 1L);

        Assert.Equal(
            0,
            Fixed64.CompareProducts(
                (Fixed64)2,
                (Fixed64)3,
                (Fixed64)1,
                (Fixed64)6));
        Assert.True(
            Fixed64.CompareProducts(
                Fixed64.MaxValue,
                Fixed64.Two,
                Fixed64.MaxValue,
                Fixed64.One) > 0);
        Assert.True(
            Fixed64.CompareProducts(
                Fixed64.MaxValue,
                Fixed64.One,
                aboveHalfDomain,
                Fixed64.Two) < 0);
    }

    [Theory]
    [InlineData(-2, 3, -1, 3, -1)]
    [InlineData(-1, -6, 2, 3, 0)]
    [InlineData(-1, 6, -2, 3, 0)]
    [InlineData(0, -6, -2, 3, 1)]
    public void CompareProducts_OrdersSignedAndZeroProducts(
        int leftFirst,
        int leftSecond,
        int rightFirst,
        int rightSecond,
        int expectedSign)
    {
        int actual = Fixed64.CompareProducts(
            (Fixed64)leftFirst,
            (Fixed64)leftSecond,
            (Fixed64)rightFirst,
            (Fixed64)rightSecond);

        Assert.Equal(expectedSign, actual);
        Assert.Equal(
            -expectedSign,
            Fixed64.CompareProducts(
                (Fixed64)rightFirst,
                (Fixed64)rightSecond,
                (Fixed64)leftFirst,
                (Fixed64)leftSecond));
    }

    [Fact]
    public void CompareProducts_FourFactors_OrdersExactScaledProducts()
    {
        Fixed64 aboveHalfDomain = Fixed64.FromRaw(
            (Fixed64.MaxValue.m_rawValue / 2L) + 1L);

        Assert.True(
            Fixed64.CompareProducts(
                Fixed64.MaxValue,
                Fixed64.One,
                Fixed64.One,
                Fixed64.One,
                aboveHalfDomain,
                Fixed64.Two,
                Fixed64.One,
                Fixed64.One) < 0);
        Assert.Equal(
            0,
            Fixed64.CompareProducts(
                (Fixed64)2,
                (Fixed64)3,
                (Fixed64)5,
                (Fixed64)7,
                (Fixed64)1,
                (Fixed64)6,
                (Fixed64)7,
                (Fixed64)5));
        Assert.True(
            Fixed64.CompareProducts(
                -Fixed64.Two,
                (Fixed64)3,
                (Fixed64)5,
                (Fixed64)7,
                -Fixed64.One,
                (Fixed64)3,
                (Fixed64)5,
                (Fixed64)7) < 0);
    }

    [Fact]
    public void CompareProducts_WarmedExecution_DoesNotAllocate()
    {
        int accumulatedSign = 0;
        long allocated = FixedMathTestHelper.MeasureWarmedAllocations(() =>
        {
            accumulatedSign = 0;
            for (int index = 0; index < 64; index++)
            {
                accumulatedSign += Fixed64.CompareProducts(
                    Fixed64.MaxValue,
                    Fixed64.Two,
                    Fixed64.MaxValue,
                    Fixed64.One);
                accumulatedSign += Fixed64.CompareProducts(
                    Fixed64.MaxValue,
                    Fixed64.One,
                    Fixed64.One,
                    Fixed64.One,
                    Fixed64.MaxValue,
                    Fixed64.Two,
                    Fixed64.One,
                    Fixed64.One);
            }
        });

        Assert.Equal(0, accumulatedSign);
        Assert.Equal(0L, allocated);
    }
}
