using System;
using FixedMathSharp.Geometry;
using Xunit;

namespace FixedMathSharp.Tests.Geometry.Wide;

public sealed class ContactQuadraticRatioTests
{
    [Theory]
    [InlineData(1, 1, 3, 2, 2, -2, 1, 2, 0, 8, 0)]
    [InlineData(2, 1, 3, 1, 2, 2, 1, 3, 1, 3, -1)]
    [InlineData(-2, -1, 3, 1, 2, -2, -1, 3, 1, 3, 1)]
    [InlineData(2, 1, -3, -1, 2, -2, -1, 3, 1, 3, 1)]
    [InlineData(1, 0, 1, 0, 0, 1, 0, 1, 0, 0, 0)]
    [InlineData(0, 1, 1, 0, 2, 0, 1, 1, 0, 3, -1)]
    public void IndependentQuadraticRatios_PreserveSignsAndExactEquality(
        int a, int b, int c, int d, int k, int e, int f, int g, int h, int l, int expected)
    {
        Assert.Equal(expected, Compare(a, b, c, d, k, e, f, g, h, l));
        Assert.Equal(-expected, Compare(e, f, g, h, l, a, b, c, d, k));
    }

    [Fact]
    public void RationalizedFields_AgreeWithIndependentIntegerRatioOracle()
    {
        // Perfect-square roots make all four signed coefficients independent,
        // while the expected result needs only ordinary integer arithmetic.
        for (int a = -2; a <= 2; a++)
        for (int b = -2; b <= 2; b++)
        for (int c = -2; c <= 2; c++)
        for (int d = -2; d <= 2; d++)
        {
            int left = a + 2 * b, right = c + 3 * d;
            int firstDenominator = 7 + 2 * d, secondDenominator = -8 + 3 * b;
            int expected = Math.Sign((left * secondDenominator - right * firstDenominator)
                * Math.Sign(firstDenominator * secondDenominator));
            Assert.Equal(expected, Compare(a, b, 7, d, 4, c, d, -8, b, 9));
        }
    }

    [Fact]
    public void DifferentRadicands_CompareOneIntegerNeighborOfAnExactTie()
    {
        // sqrt(8)/2 = sqrt(2), independent of the root representation.
        Assert.Equal(0, Compare(0, 1, 1, 0, 2, 0, 1, 2, 0, 8));
        Assert.Equal(-1, Compare(0, 1, 1, 0, 2, 1, 1, 2, 0, 8));
        Assert.Equal(1, Compare(0, 1, 1, 0, 2, -1, 1, 2, 0, 8));
    }

    [Theory]
    [InlineData(0, 1, 1, 1, 0)]
    [InlineData(1, 2, 1, 1, 0)]
    [InlineData(3, 2, 1, 1, 2)]
    [InlineData(5, 2, 1, 1, 2)]
    [InlineData(7, 2, 1, 1, 4)]
    [InlineData(1, 1, 2, 1, 1)]
    [InlineData(3, 1, 2, 1, 4)]
    [InlineData(1, 1, 0, 1, 0)]
    public void RootRatioMaterialization_RoundsRawUnitsOnlyAtFinalNearestEven(
        int numerator, int denominator, int metric, int scale, long expectedRaw)
    {
        Span<ulong> values = stackalloc ulong[4 * ConvexContactCandidate.Words];
        Span<int> signs = stackalloc int[4];
        ContactQuadratic n = ContactQuadratic.At(values, signs, 0), d = ContactQuadratic.At(values, signs, 1);
        Set(n, numerator, 0); Set(d, denominator, 0);
        Assert.True(ContactQuadratic.TryRoundRootRatio(n, d, new ulong[] { 0 },
            new ulong[] { (ulong)metric }, new ulong[] { (ulong)scale }, out Fixed64 result));
        Assert.Equal(expectedRaw, result.m_rawValue);
    }

    [Fact]
    public void WideBorrowedFields_PreserveHighLimbsAndClearCopiedTails()
    {
        const int words = 48;
        Span<ulong> values = stackalloc ulong[8 * words];
        Span<int> signs = stackalloc int[8];
        ContactQuadratic n = ContactQuadratic.At(values, signs, 0, words), d = ContactQuadratic.At(values, signs, 1, words);
        ContactQuadratic otherN = ContactQuadratic.At(values, signs, 2, words), otherD = ContactQuadratic.At(values, signs, 3, words);
        Set(n, 0, 0); Set(d, 0, 0); Set(otherN, 0, 0); Set(otherD, 0, 0);
        n.Rational[43] = 3; d.Rational[43] = 2; n.Signs[0] = d.Signs[0] = 1;
        Set(otherN, 1, 0); Set(otherD, 1, 0);
        Assert.Equal(1, ContactQuadratic.CompareRatios(n, d, new ulong[] { 2 }, otherN, otherD, new ulong[] { 3 }));
        Assert.Equal(2, ContactQuadratic.RoundRatio(n, d, new ulong[] { 2 }).m_rawValue);
        Assert.True(ContactQuadratic.TryRoundRootRatio(n, d, new ulong[] { 2 }, new ulong[] { 1 }, new ulong[] { 1 }, out Fixed64 result));
        Assert.Equal(2, result.m_rawValue);
        otherN.Values.Fill(ulong.MaxValue);
        Span<ulong> smallValues = stackalloc ulong[2 * ConvexContactCandidate.Words];
        Span<int> smallSigns = stackalloc int[2];
        var small = new ContactQuadratic(smallValues, smallSigns); Set(small, 2, -3);
        small.CopyTo(otherN);
        Assert.Equal(2UL, otherN.Rational[0]); Assert.Equal(3UL, otherN.Radical[0]);
        Assert.Equal(0UL, otherN.Rational[47]); Assert.Equal(0UL, otherN.Radical[47]);
        Assert.Equal(-1, otherN.Signs[1]);
    }

    [Theory]
    [InlineData(0)]
    [InlineData(1)]
    [InlineData(2)]
    [InlineData(3)]
    public void RootRatioMaterialization_HandlesUpperHalfWithoutOverflow(int boundary)
    {
        Span<ulong> values = stackalloc ulong[4 * ConvexContactCandidate.Words];
        Span<int> signs = stackalloc int[4];
        ContactQuadratic n = ContactQuadratic.At(values, signs, 0), d = ContactQuadratic.At(values, signs, 1);
        Set(n, 0, 0); Set(d, 1, 0); n.Signs[0] = 1;
        if (boundary == 0) n.Rational[0] = (ulong)long.MaxValue;
        else if (boundary == 1) n.Rational[0] = (ulong)long.MaxValue + 1;
        else if (boundary == 2) { n.Rational[0] = ulong.MaxValue; d.Rational[0] = 2; }
        else { n.Rational[0] = ulong.MaxValue - 2; n.Rational[1] = 1; d.Rational[0] = 4; }
        bool represented = ContactQuadratic.TryRoundRootRatio(n, d, new ulong[] { 0 },
            new ulong[] { 1 }, new ulong[] { 1 }, out Fixed64 result);
        Assert.Equal(boundary == 0 || boundary == 3, represented);
        Assert.Equal(represented ? long.MaxValue : 0, result.m_rawValue);
    }

    [Theory]
    [InlineData(0, 0, 1, 0, 2, 0)]
    [InlineData(1, 0, 4, 0, 2, 0)]
    [InlineData(9, 0, 4, 0, 2, 2)]
    [InlineData(25, 0, 4, 0, 2, 2)]
    [InlineData(49, 0, 4, 0, 2, 4)]
    [InlineData(2, 0, 1, 0, 2, 1)]
    [InlineData(3, 2, 1, 0, 2, 2)]
    [InlineData(1, 1, 3, 2, 2, 1)]
    public void SquareRootRatioMaterialization_PreservesFieldAndNearestEven(int a, int b, int c, int d, int k, long expectedRaw)
    {
        Span<ulong> values = stackalloc ulong[4 * ConvexContactCandidate.Words];
        Span<int> signs = stackalloc int[4];
        ContactQuadratic n = ContactQuadratic.At(values, signs, 0), denominator = ContactQuadratic.At(values, signs, 1);
        Set(n, a, b); Set(denominator, c, d);
        Assert.True(ContactQuadratic.TryRoundSquareRootRatio(n, denominator, new ulong[] { (ulong)k }, out Fixed64 result));
        Assert.Equal(expectedRaw, result.m_rawValue);
    }

    private static int Compare(int a, int b, int c, int d, int k,
        int e, int f, int g, int h, int l)
    {
        const int words = ConvexContactCandidate.Words;
        Span<ulong> values = stackalloc ulong[8 * words];
        Span<int> signs = stackalloc int[8];
        Span<ulong> firstRoot = stackalloc ulong[words];
        Span<ulong> secondRoot = stackalloc ulong[words];
        firstRoot.Clear(); secondRoot.Clear(); firstRoot[0] = (ulong)k; secondRoot[0] = (ulong)l;
        ContactQuadratic firstNumerator = ContactQuadratic.At(values, signs, 0);
        ContactQuadratic firstDenominator = ContactQuadratic.At(values, signs, 1);
        ContactQuadratic secondNumerator = ContactQuadratic.At(values, signs, 2);
        ContactQuadratic secondDenominator = ContactQuadratic.At(values, signs, 3);
        Set(firstNumerator, a, b); Set(firstDenominator, c, d);
        Set(secondNumerator, e, f); Set(secondDenominator, g, h);
        return ContactQuadratic.CompareRatios(firstNumerator, firstDenominator, firstRoot,
            secondNumerator, secondDenominator, secondRoot);
    }

    private static void Set(ContactQuadratic value, int rational, int radical)
    {
        value.Clear(); value.Rational[0] = (ulong)Math.Abs(rational);
        value.Radical[0] = (ulong)Math.Abs(radical);
        value.Signs[0] = Math.Sign(rational); value.Signs[1] = Math.Sign(radical);
    }
}
