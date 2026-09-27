using System;
using System.Numerics;
using FixedMathSharp.Geometry;
using Xunit;

namespace FixedMathSharp.Tests;

public sealed class CylinderPairSideFeaturesTests
{
    [Theory]
    [InlineData(135)]
    [InlineData(180)]
    [InlineData(-135)]
    [InlineData(0)]
    [InlineData(-200)]
    [InlineData(-205)]
    public void ObliquePlane_PreservesExistingSelectedEllipseAndFullRadius(int clearance)
    {
        const long s = 607400100;
        FixedQuaternion rotation = new(Fixed64.FromRaw(5 * s), Fixed64.Zero,
            Fixed64.FromRaw(-3 * s), Fixed64.FromRaw(4 * s));
        Vector3d center = new(625 - 3 * clearance / 5, 500 - 4 * clearance / 5, 0);
        var geometry = new CylinderPairGeometry(Vector3d.Zero, FixedQuaternion.Identity, Vector3d.Up,
            Signed192.Raw((Fixed64)1000), (Fixed64)625, center, rotation, Vector3d.Up, Signed192.Raw((Fixed64)2000), (Fixed64)200);
        Span<ulong> polynomial = stackalloc ulong[9 * 116]; Span<sbyte> signs = stackalloc sbyte[9];
        Span<ulong> cell = stackalloc ulong[1900];
        Assert.True(WideConvexPrismRelations.TryGetCylinderPairSideFeature(Vector3d.Zero, FixedQuaternion.Identity,
            Vector3d.Up, Signed192.Raw((Fixed64)1000), (Fixed64)625, center, rotation, Vector3d.Up, Signed192.Raw((Fixed64)2000), (Fixed64)200,
            geometry, false, polynomial, signs, cell, out FiniteAxisValueRoot root, out Vector3d normal, out int gapSign));
        Assert.Equal(Math.Sign(clearance + 200), gapSign);
        Assert.Equal(new Vector3d(Fixed64.FromFraction(3, 5), Fixed64.FromFraction(4, 5), Fixed64.Zero), normal);
        if (gapSign > 0)
            AssertExactRawSquared(geometry, root, (BigInteger)(clearance + 200) << 32);
        else
            Assert.True(root.Signs.IsEmpty);
    }

    [Theory]
    [InlineData(false, 3, 1)]
    [InlineData(true, 3, 1)]
    [InlineData(false, 1, -1)]
    public void PerpendicularCorner_RetainsNegativeUnshiftedGapAndPairOrientation(bool swap, int radius, int expectedSign)
    {
        Vector3d first = swap ? new Vector3d(2, 0, 2) : Vector3d.Zero;
        Vector3d second = swap ? Vector3d.Zero : new Vector3d(2, 0, 2);
        Vector3d a = swap ? Vector3d.Up : Vector3d.Right;
        Vector3d b = swap ? Vector3d.Right : Vector3d.Up;
        Fixed64 ra = swap ? (Fixed64)radius : Fixed64.One, rb = swap ? Fixed64.One : (Fixed64)radius;
        var geometry = new CylinderPairGeometry(first, FixedQuaternion.Identity, a, Signed192.Raw((Fixed64)2), ra,
            second, FixedQuaternion.Identity, b, Signed192.Raw((Fixed64)2), rb);
        Span<ulong> polynomial = stackalloc ulong[9 * 116]; Span<sbyte> signs = stackalloc sbyte[9];
        Span<ulong> cell = stackalloc ulong[1900];
        Assert.True(WideConvexPrismRelations.TryGetCylinderPairSideFeature(first, FixedQuaternion.Identity, a,
            Signed192.Raw((Fixed64)2), ra, second, FixedQuaternion.Identity, b, Signed192.Raw((Fixed64)2), rb, geometry, swap,
            polynomial, signs, cell, out FiniteAxisValueRoot root, out Vector3d normal, out int gapSign));
        Assert.Equal(expectedSign, gapSign);
        Fixed64 component = Fixed64.FromRaw(3037000500);
        Assert.Equal(new Vector3d(swap ? -component : component, Fixed64.Zero, swap ? -component : component), normal);
        if (gapSign > 0)
        {
            BigInteger scale = Integer(geometry.RawScale), raw = BigInteger.One << 32;
            BigInteger unit = raw * raw * scale * scale;
            // (3-sqrt(2))² = 11-6sqrt(2): selected value satisfies x²-22x+49.
            AssertRootQuery(root, new[] { 49 * unit * unit,
                (-22 * unit) << geometry.ValueShift, BigInteger.One << (2 * geometry.ValueShift) });
            AssertRootQuery(root, new[] { -9 * unit, BigInteger.One << geometry.ValueShift }, -1);
        }
    }

    [Fact]
    public void PerpendicularCorner_ExactFullTouchHasNoPositiveValueRoot()
    {
        Vector3d center = new(4, 0, 5);
        var geometry = new CylinderPairGeometry(Vector3d.Zero, FixedQuaternion.Identity, Vector3d.Right,
            Signed192.Raw((Fixed64)2), Fixed64.One, center, FixedQuaternion.Identity, Vector3d.Up, Signed192.Raw((Fixed64)2), (Fixed64)5);
        Span<ulong> polynomial = stackalloc ulong[9 * 116]; Span<sbyte> signs = stackalloc sbyte[9];
        Span<ulong> cell = stackalloc ulong[1900];
        Assert.True(WideConvexPrismRelations.TryGetCylinderPairSideFeature(Vector3d.Zero, FixedQuaternion.Identity,
            Vector3d.Right, Signed192.Raw((Fixed64)2), Fixed64.One, center, FixedQuaternion.Identity, Vector3d.Up,
            Signed192.Raw((Fixed64)2), (Fixed64)5, geometry, false, polynomial, signs, cell,
            out FiniteAxisValueRoot root, out Vector3d normal, out int gapSign));
        Assert.Equal(0, gapSign);
        Assert.True(root.Signs.IsEmpty);
        Assert.Equal(new Vector3d(Fixed64.FromFraction(3, 5), Fixed64.Zero, Fixed64.FromFraction(4, 5)), normal);
    }

    [Fact]
    public void CommonPerpendicularWinner_IsLeftToEarlierGlobalCandidate()
    {
        Vector3d center = new(0, 0, 3);
        var geometry = new CylinderPairGeometry(Vector3d.Zero, FixedQuaternion.Identity, Vector3d.Right,
            Signed192.Raw((Fixed64)2), Fixed64.One, center, FixedQuaternion.Identity, Vector3d.Up, Signed192.Raw((Fixed64)2), (Fixed64)3);
        Span<ulong> polynomial = stackalloc ulong[9 * 116]; Span<sbyte> signs = stackalloc sbyte[9];
        Span<ulong> cell = stackalloc ulong[1900];
        Assert.False(WideConvexPrismRelations.TryGetCylinderPairSideFeature(Vector3d.Zero, FixedQuaternion.Identity,
            Vector3d.Right, Signed192.Raw((Fixed64)2), Fixed64.One, center, FixedQuaternion.Identity, Vector3d.Up,
            Signed192.Raw((Fixed64)2), (Fixed64)3, geometry, false, polynomial, signs, cell, out _, out _, out _));
    }

    [Fact]
    public void CommonPerpendicularWinner_BeatsAnExistingNonprincipalStationaryMinimum()
    {
        // Projected cap ellipse semiaxes are5 and3. After the cap offset,
        // its quadrant gap is sqrt(25-16s²)-sqrt(1-s²)+3s/2, 0<=s<=1.
        // sqrt(25-16s²)>=5-2s² and sqrt(1-s²)<=1-s²/2 prove gap>=4.
        // The derivative is positive at both endpoints and negative at45°,
        // so a genuine nonprincipal local minimum exists but cannot win.
        Fixed64 s = Fixed64.FromRaw(1920767767);
        FixedQuaternion rotation = new(Fixed64.Zero, Fixed64.Zero, s, 2 * s);
        Vector3d center = new(Fixed64.FromFraction(3, 2), (Fixed64)2, Fixed64.One);
        var geometry = new CylinderPairGeometry(Vector3d.Zero, FixedQuaternion.Identity, Vector3d.Up,
            Signed192.Raw((Fixed64)10), (Fixed64)5, center, rotation, Vector3d.Up, Signed192.Raw(Fixed64.Two), Fixed64.One);
        Span<ulong> polynomial = stackalloc ulong[9 * 116]; Span<sbyte> signs = stackalloc sbyte[9];
        Span<ulong> cell = stackalloc ulong[1900];
        Assert.False(WideConvexPrismRelations.TryGetCylinderPairSideFeature(Vector3d.Zero, FixedQuaternion.Identity,
            Vector3d.Up, Signed192.Raw((Fixed64)10), (Fixed64)5, center, rotation, Vector3d.Up,
            Signed192.Raw(Fixed64.Two), Fixed64.One, geometry, false, polynomial, signs, cell, out _, out _, out _));
    }

    [Fact]
    public void PrincipalMinor_WhenMajorProjectionIsZero_MapsItsSmoothCriticalValue()
    {
        Fixed64 s = Fixed64.FromRaw(1920767767);
        FixedQuaternion rotation = new(Fixed64.Zero, Fixed64.Zero, s, 2 * s);
        Vector3d center = new(4, -3, 0);
        var geometry = new CylinderPairGeometry(Vector3d.Zero, FixedQuaternion.Identity, Vector3d.Right,
            Signed192.Raw((Fixed64)2), Fixed64.One, center, rotation, Vector3d.Right, Signed192.Raw((Fixed64)2), (Fixed64)4);
        Span<ulong> polynomial = stackalloc ulong[9 * 116]; Span<sbyte> signs = stackalloc sbyte[9];
        Span<ulong> cell = stackalloc ulong[1900];
        Assert.True(WideConvexPrismRelations.TryGetCylinderPairSideFeature(Vector3d.Zero, FixedQuaternion.Identity,
            Vector3d.Right, Signed192.Raw((Fixed64)2), Fixed64.One, center, rotation, Vector3d.Right,
            Signed192.Raw((Fixed64)2), (Fixed64)4, geometry, false, polynomial, signs, cell,
            out FiniteAxisValueRoot root, out Vector3d normal, out int gapSign));
        Assert.Equal(1, gapSign);
        Assert.Equal(new Vector3d(Fixed64.FromFraction(4, 5), -Fixed64.FromFraction(3, 5), Fixed64.Zero), normal);
        Assert.Equal((BigInteger)5, Integer(geometry.RawScale));
        AssertRootQuery(root, new[] { -(BigInteger.One << 66), BigInteger.One << geometry.ValueShift });
    }

    private static void AssertExactRawSquared(CylinderPairGeometry geometry, FiniteAxisValueRoot root, BigInteger raw)
    {
        BigInteger scaled = raw * Integer(geometry.RawScale);
        AssertRootQuery(root, new[] { -scaled * scaled, BigInteger.One << geometry.ValueShift });
    }
    private static void AssertRootQuery(FiniteAxisValueRoot root, BigInteger[] values, int expectedSign = 0)
    {
        const int words = 116;
        var coefficients = new ulong[values.Length * words]; var signs = new sbyte[values.Length];
        for (int i = 0; i < values.Length; i++)
        {
            signs[i] = (sbyte)values[i].Sign; BigInteger magnitude = BigInteger.Abs(values[i]);
            for (int j = 0; j < words; j++) { coefficients[i * words + j] = (ulong)(magnitude & ulong.MaxValue); magnitude >>= 64; }
            Assert.Equal(BigInteger.Zero, magnitude);
        }
        Assert.Equal(expectedSign, WideFiniteAxisIntersection.GetSignAtFiniteValueRoot(root, coefficients, signs));
    }
    private static BigInteger Integer(Signed192 value) =>
        ((BigInteger)(long)value.High << 128) | ((BigInteger)value.Middle << 64) | value.Low;
}
