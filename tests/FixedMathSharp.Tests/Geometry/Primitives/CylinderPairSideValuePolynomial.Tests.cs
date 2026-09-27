using System;
using System.Numerics;
using FixedMathSharp.Geometry;
using Xunit;

namespace FixedMathSharp.Tests;

public sealed class CylinderPairSideValuePolynomialTests
{
    [Theory]
    [InlineData(3, -2, 5, 11)]
    [InlineData(-4, 3, -1, 11)]
    [InlineData(3, -2, 5, 0)]
    [InlineData(0, 0, 0, 3)]
    public void Build_MatchesIndependentConicAndSylvesterResultant(int x, int y, int z, int radius)
    {
        AssertOracle(new BigInteger[] { 1, 2, 2 }, new BigInteger[] { 2, -3, 4 },
            new BigInteger[] { x, y, z }, 7, radius, 16);
    }

    [Fact]
    public void Build_HandlesFullGeometryExponentRangeWithoutTruncation()
    {
        BigInteger a = BigInteger.One << 31, b = BigInteger.One << 161;
        BigInteger c = BigInteger.One << 227, r = BigInteger.One << 224;
        AssertOracle(new[] { a + 1, a - 1, -a }, new[] { b, -b + 1, b - 3 },
            new[] { -c + 5, c, c - 1 }, r + 1, r - 3, 116);
    }

    [Fact]
    public void Build_IsInvariantUnderNegatingEitherAxis()
    {
        BigInteger[] v = { 9, 29, 4, 38, 9, 32, 49, 121 };
        BigInteger[] expected = Construct(v, 16, out _);
        v[2] = -v[2]; v[4] = -v[4];
        Assert.Equal(expected, Construct(v, 16, out _));
        v[2] = -v[2]; v[5] = -v[5];
        Assert.Equal(expected, Construct(v, 16, out _));
    }

    [Fact]
    public void Build_WarmedConstructionDoesNotAllocate()
    {
        const int words = 16;
        var values = new ulong[8 * words];
        var inputSigns = new sbyte[8];
        ulong[] scalars = { 9, 29, 4, 38, 9, 32, 49, 121 };
        for (int i = 0; i < 8; i++) { values[i * words] = scalars[i]; inputSigns[i] = 1; }
        var output = new ulong[9 * words]; var signs = new sbyte[9];
        Assert.Equal(8, CylinderPairSideValuePolynomial.Build(values, inputSigns, words, output, signs));
        ulong[] expected = (ulong[])output.Clone();
        sbyte[] expectedSigns = (sbyte[])signs.Clone();
        int degree = 0;
        long allocated = FixedMathTestHelper.MeasureWarmedAllocations(() =>
        {
            for (int i = 0; i < 8; i++)
                degree = CylinderPairSideValuePolynomial.Build(values, inputSigns, words, output, signs);
        });
        Assert.Equal(0, allocated);
        Assert.Equal(8, degree);
        Assert.Equal(expected, output);
        Assert.Equal(expectedSigns, signs);
    }

    [Fact]
    public void Build_ExposesUnshiftedQuarticWithoutChangingItsExactScale()
    {
        const int words = 16;
        var input = new ulong[8 * words]; var inputSigns = new sbyte[8];
        ulong[] values = { 9, 29, 4, 38, 9, 32, 49, 0 };
        for (int i = 0; i < values.Length; i++) { input[i * words] = values[i]; inputSigns[i] = values[i] == 0 ? (sbyte)0 : (sbyte)1; }
        var output = new ulong[9 * words]; var signs = new sbyte[9];
        var f = new ulong[5 * words]; var fSigns = new sbyte[5];
        CylinderPairSideValuePolynomial.Build(input, inputSigns, words, output, signs, f, fSigns);
        BigInteger[] quartic = Decode(f, fSigns, words);
        Assert.True(quartic[4] > 0);
        Assert.Equal(Decode(output, signs, words), Mul(quartic, quartic));
    }

    [Theory]
    [InlineData(false, 1)]
    [InlineData(false, -1)]
    [InlineData(true, 1)]
    [InlineData(true, -1)]
    public void Build_GeometryWrapperChoosesSideCapAndRadius(bool firstSide, int capSign)
    {
        Fixed64 s = Fixed64.FromRaw(1920767767);
        var geometry = new CylinderPairGeometry(Vector3d.Zero,
            new FixedQuaternion(Fixed64.Zero, -s, Fixed64.Zero, 2 * s), Vector3d.Right,
            Signed192.Raw((Fixed64)10), Fixed64.One, new Vector3d(0, 1, 8),
            new FixedQuaternion(Fixed64.Zero, s, Fixed64.Zero, 2 * s), Vector3d.Left,
            Signed192.Raw((Fixed64)10), (Fixed64)2);
        BigInteger raw = BigInteger.One << 32;
        BigInteger[] a = firstSide ? new BigInteger[] { 7, 0, 24 } : new BigInteger[] { 1, 0, 0 };
        BigInteger[] b = firstSide ? new BigInteger[] { 1, 0, 0 } : new BigInteger[] { 7, 0, 24 };
        BigInteger[] c = firstSide
            ? new[] { (7 * capSign - 32) * raw, -5 * raw, (24 * capSign - 24) * raw }
            : new[] { (25 * capSign - 32) * raw, -5 * raw, -24 * raw };
        BigInteger ra = (firstSide ? 10 : 5) * raw, rb = (firstSide ? 5 : 10) * raw;
        BigInteger[] expected = Construct(new[] { Dot(a, a), Dot(b, b), Dot(a, b),
            Dot(c, c), Dot(a, c), Dot(b, c), ra * ra, rb * rb }, 32, out _);
        var output = new ulong[9 * 32]; var signs = new sbyte[9];
        Assert.Equal(8, CylinderPairSideValuePolynomial.Build(geometry, firstSide, capSign, output, signs));
        Assert.Equal(expected, Decode(output, signs, 32));
    }

    private static void AssertOracle(BigInteger[] a, BigInteger[] b, BigInteger[] c,
        BigInteger ra, BigInteger rb, int words)
    {
        BigInteger u = Dot(a, a), v = Dot(b, b), ab = Dot(a, b);
        BigInteger q = Dot(c, c), x = Dot(a, c), y = Dot(b, c);
        BigInteger rho = ra * ra, tau = rb * rb, h = u * v, e = h - ab * ab;
        BigInteger tripleSquared = e * q - v * x * x + 2 * ab * x * y - u * y * y;
        BigInteger minorProjection = v * x - ab * y;
        // Principal-frame ellipse, with all four rational invariants put over
        // one denominator. This deliberately retains E, which runtime cancels.
        BigInteger d = h * e, major = rho * d, minor = rho * ab * ab * e;
        BigInteger centerMajor = tripleSquared * h;
        BigInteger centerMinor = minorProjection * minorProjection * u;
        BigInteger[] l = { major + minor - centerMajor - centerMinor, d };
        BigInteger[] m = { -major * minor + minor * centerMajor + major * centerMinor,
            -(major + minor) * d };
        BigInteger[] t = { 0, major * minor * d };
        BigInteger[] f = Add(Add(Mul(Mul(m, m), Mul(l, l)), Scale(Mul(Mul(m, m), m), 4)),
            Add(Scale(Mul(t, Mul(Mul(l, l), l)), -4),
                Add(Scale(Mul(t, Mul(m, l)), -18), Scale(Mul(t, t), -27))));
        BigInteger divisor = h * h * BigInteger.Pow(e, 6) * rho * rho;
        for (int i = 0; i < f.Length; i++)
        {
            f[i] = BigInteger.DivRem(f[i], divisor, out BigInteger remainder);
            Assert.Equal(BigInteger.Zero, remainder);
        }
        // Eliminate z between F(z) and z²-2(S+tau)z+(S-tau)².
        // The 6x6 Sylvester determinant is independent of runtime's conjugate.
        var matrix = new BigInteger[6, 6][];
        for (int row = 0; row < 6; row++)
            for (int col = 0; col < 6; col++) matrix[row, col] = new BigInteger[] { 0 };
        for (int row = 0; row < 2; row++)
            for (int j = 0; j < 5; j++) matrix[row, row + j] = new[] { j < f.Length ? f[4 - j] : BigInteger.Zero };
        BigInteger[][] quadratic = { new BigInteger[] { 1 }, new[] { -2 * tau, (BigInteger)(-2) },
            new[] { tau * tau, -2 * tau, BigInteger.One } };
        for (int row = 0; row < 4; row++)
            for (int j = 0; j < 3; j++) matrix[row + 2, row + j] = quadratic[j];
        BigInteger[] expected = new BigInteger[9];
        Determinant(matrix, 0, 0, 1, new BigInteger[] { 1 }, expected);
        BigInteger[] actual = Construct(new[] { u, v, ab, q, x, y, rho, tau }, words, out int degree);
        Assert.Equal(expected, actual);
        Assert.Equal(8, degree);
    }

    private static void Determinant(BigInteger[,][] matrix, int row, int used, int sign,
        BigInteger[] product, BigInteger[] result)
    {
        if (row == 6)
        {
            for (int i = 0; i < product.Length; i++) result[i] += sign * product[i];
            return;
        }
        for (int col = 0; col < 6; col++)
        {
            if ((used & (1 << col)) != 0) continue;
            int inversions = 0;
            for (int prior = col + 1; prior < 6; prior++)
                if ((used & (1 << prior)) != 0) inversions++;
            Determinant(matrix, row + 1, used | (1 << col),
                (inversions & 1) == 0 ? sign : -sign, Mul(product, matrix[row, col]), result);
        }
    }

    private static BigInteger[] Construct(BigInteger[] values, int words, out int degree)
    {
        var input = new ulong[8 * words]; var inputSigns = new sbyte[8];
        for (int i = 0; i < 8; i++)
        {
            inputSigns[i] = (sbyte)values[i].Sign;
            BigInteger magnitude = BigInteger.Abs(values[i]);
            for (int j = 0; j < words; j++) { input[i * words + j] = (ulong)(magnitude & ulong.MaxValue); magnitude >>= 64; }
            Assert.Equal(BigInteger.Zero, magnitude);
        }
        var output = new ulong[9 * words]; var signs = new sbyte[9];
        Array.Fill(output, ulong.MaxValue); Array.Fill(signs, (sbyte)-1);
        degree = CylinderPairSideValuePolynomial.Build(input, inputSigns, words, output, signs);
        return Decode(output, signs, words);
    }

    private static BigInteger[] Decode(ulong[] output, sbyte[] signs, int words)
    {
        var result = new BigInteger[signs.Length];
        for (int i = 0; i < signs.Length; i++)
        {
            for (int j = words - 1; j >= 0; j--) result[i] = (result[i] << 64) + output[i * words + j];
            result[i] *= signs[i];
        }
        return result;
    }

    private static BigInteger Dot(BigInteger[] a, BigInteger[] b) => a[0] * b[0] + a[1] * b[1] + a[2] * b[2];
    private static BigInteger[] Add(BigInteger[] a, BigInteger[] b)
    {
        var r = new BigInteger[Math.Max(a.Length, b.Length)];
        for (int i = 0; i < a.Length; i++) r[i] += a[i];
        for (int i = 0; i < b.Length; i++) r[i] += b[i];
        return r;
    }
    private static BigInteger[] Scale(BigInteger[] a, int scale)
    {
        var r = new BigInteger[a.Length]; for (int i = 0; i < a.Length; i++) r[i] = a[i] * scale; return r;
    }
    private static BigInteger[] Mul(BigInteger[] a, BigInteger[] b)
    {
        var r = new BigInteger[a.Length + b.Length - 1];
        for (int i = 0; i < a.Length; i++) for (int j = 0; j < b.Length; j++) r[i + j] += a[i] * b[j];
        return r;
    }
}
