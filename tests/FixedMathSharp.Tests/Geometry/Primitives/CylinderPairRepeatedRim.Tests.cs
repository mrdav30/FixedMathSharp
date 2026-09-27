using System;
using System.Collections.Generic;
using System.Numerics;
using FixedMathSharp.Geometry;
using Xunit;

namespace FixedMathSharp.Tests;

public sealed class CylinderPairRepeatedRimTests
{
    [Theory]
    [InlineData(0, 1)]
    [InlineData(1, -1)]
    [InlineData(-1, 1)]
    public void Contact_FirstCapBeatsTheNonorthogonalHZeroStationaryValue(
        int axisRawOffset, int secondAxisSign)
    {
        // a=X, b=(3,0,4)/5, c=(3,-3,0), radii5 has H=0
        // stationary points with S=50. The +X support gap is exactly7,
        // strictly below sqrt50. Adjusting the center with the first half
        // preserves c for near-unit authored axes.
        Fixed64 s = Fixed64.FromRaw(1920767767);
        var secondRotation = new FixedQuaternion(Fixed64.Zero, -s, Fixed64.Zero, 2 * s);
        Vector3d firstAxis = new(Fixed64.FromRaw((1L << 32) + axisRawOffset), Fixed64.Zero, Fixed64.Zero);
        Vector3d secondAxis = secondAxisSign > 0 ? Vector3d.Right : Vector3d.Left;
        Fixed64 half = Fixed64.FromRaw(5 * ((1L << 32) + axisRawOffset));
        Vector3d center = new(half, (Fixed64)3, (Fixed64)4);
        // The point (3,3,4) belongs to both cylinders.
        Assert.True(FixedSegment.TryGetCenteredFiniteCylindersContact(
            Vector3d.Zero, FixedQuaternion.Identity, firstAxis, (Fixed64)10, (Fixed64)5,
            center, secondRotation, secondAxis, (Fixed64)10, (Fixed64)5, out FixedContactAnchors contact));
        Assert.InRange(contact.Depth.m_rawValue, 0L, 7L << 32);
        Assert.False(contact.DepthIsClamped);
        Assert.NotEqual(Vector3d.Zero, contact.Normal);
        Assert.Equal(Vector3d.Zero, contact.FirstAnchor.Origin);
        Assert.Equal(center, contact.SecondAnchor.Origin);
        Assert.True(FixedSegment.TryGetCenteredFiniteCylindersContact(
            center, secondRotation, secondAxis, (Fixed64)10, (Fixed64)5,
            Vector3d.Zero, FixedQuaternion.Identity, firstAxis, (Fixed64)10, (Fixed64)5,
            out FixedContactAnchors reversed));
        Assert.Equal(contact.Depth, reversed.Depth);
        Assert.False(reversed.DepthIsClamped);
        Assert.Equal(center, reversed.FirstAnchor.Origin);
        Assert.Equal(Vector3d.Zero, reversed.SecondAnchor.Origin);
    }

    [Fact]
    public void RankOne_MatchesTheSelectedIrrationalValueAndRoundsItsRadicalNormal()
    {
        // c=(0,0,1), radii2/4. The larger stationary value is 21+4sqrt17,
        // with minimal polynomial S²-42S+169 and normals (0,+-4,1)/sqrt17.
        var geometry = new CylinderPairGeometry(Vector3d.Zero, FixedQuaternion.Identity, Vector3d.Right,
            Signed192.Raw((Fixed64)10), (Fixed64)2, new Vector3d(5, 0, 4), FixedQuaternion.Identity, Vector3d.Forward,
            Signed192.Raw((Fixed64)10), (Fixed64)4);
        BigInteger unit = Integer(geometry.FirstRadius) / 2;
        BigInteger square = unit * unit;
        BigInteger[] exact = { 169 * square * square, -(42 * square << geometry.ValueShift),
            BigInteger.One << (2 * geometry.ValueShift) };
        BigInteger gcd = BigInteger.GreatestCommonDivisor(exact[0],
            BigInteger.GreatestCommonDivisor(exact[1], exact[2]));
        const int words = 12;
        ulong[] polynomial = new ulong[3 * words];
        sbyte[] signs = new sbyte[3];
        for (int i = 0; i < exact.Length; i++)
        {
            exact[i] /= gcd;
            Write(BigInteger.Abs(exact[i]), polynomial.AsSpan(i * words, words));
            signs[i] = (sbyte)exact[i].Sign;
        }
        ulong[] cell = new ulong[WideFiniteAxisIntersection.GetFiniteValueRootCellWords(polynomial, signs)];
        Assert.True(WideFiniteAxisIntersection.TryGetFiniteValueRoot(polynomial, signs, 1, cell, out var root));
        Assert.False(root.IsRational);
        long y = RoundRootRatio(16, 17), z = RoundRootRatio(1, 17);
        Vector3d[] expected = { new(Fixed64.Zero, Fixed64.FromRaw(y), Fixed64.FromRaw(z)),
            new(Fixed64.Zero, Fixed64.FromRaw(-y), Fixed64.FromRaw(z)) };
        var actual = new HashSet<Vector3d>();
        int accepted = 0;
        for (int branch = 0; branch < 8; branch++)
        {
            if (!CylinderPairRepeatedRim.TryGetRankOneNormal(geometry, 1, 1, root, branch,
                out Vector3d normal)) continue;
            accepted++;
            actual.Add(normal);
        }
        Assert.Equal(2, accepted);
        Assert.True(actual.SetEquals(expected));
    }

    [Fact]
    public void RankOne_CoplanarDependentProjectionsRetainTheZeroEigenvalueBranch()
    {
        // c=(0,0,3), p=(0,+-8/5,6/5), q=(0,+-4,0).
        // S=49 has two admitting rank-one stationary normals. The projected
        // c and b are dependent, so their zero-eigenvalue compatibility is
        // identically zero rather than a linear equation selecting S.
        var geometry = new CylinderPairGeometry(Vector3d.Zero, FixedQuaternion.Identity, Vector3d.Right,
            Signed192.Raw((Fixed64)10), (Fixed64)2, new Vector3d(5, 0, 2), FixedQuaternion.Identity, Vector3d.Forward,
            Signed192.Raw((Fixed64)10), (Fixed64)4);
        BigInteger radius = Integer(geometry.FirstRadius);
        BigInteger constant = 49 * radius * radius;
        BigInteger leading = (BigInteger)4 << geometry.ValueShift;
        const int words = 10;
        ulong[] polynomial = new ulong[2 * words];
        Write(constant, polynomial.AsSpan(0, words)); Write(leading, polynomial.AsSpan(words, words));
        sbyte[] signs = { -1, 1 };
        ulong[] cell = new ulong[WideFiniteAxisIntersection.GetFiniteValueRootCellWords(polynomial, signs)];
        Assert.True(WideFiniteAxisIntersection.TryGetFiniteValueRoot(polynomial, signs, 0, cell, out var root));
        long y = RoundRootRatio(16, 25), z = RoundRootRatio(9, 25);
        Vector3d[] expected = { new(Fixed64.Zero, Fixed64.FromRaw(y), Fixed64.FromRaw(z)),
            new(Fixed64.Zero, Fixed64.FromRaw(-y), Fixed64.FromRaw(z)) };
        var actual = new HashSet<Vector3d>();
        int accepted = 0;
        for (int branch = 0; branch < 8; branch++)
        {
            if (!CylinderPairRepeatedRim.TryGetRankOneNormal(geometry, 1, 1, root, branch,
                out Vector3d normal)) continue;
            accepted++;
            actual.Add(normal);
        }
        Assert.Equal(2, accepted);
        Assert.True(actual.SetEquals(expected));
    }

    [Theory]
    [InlineData(4, 50, 1)]
    [InlineData(4, 50, -1)]
    [InlineData(5, 41, 1)]
    [InlineData(5, 41, -1)]
    public void RankOne_RejectsHZeroValuesDominatedByThePublicCapMinimum(
        int centerZ, int squaredValue, int secondAxisSign)
    {
        // p=(0,3,4), c=(0,-3,1), t=5, q=(5*sqrt(7)/4,15/4,0).
        // v=t*b+q has S=50 and positive radial/cap signs.
        var geometry = new CylinderPairGeometry(Vector3d.Zero, FixedQuaternion.Identity, Vector3d.Right,
            Signed192.Raw((Fixed64)10), (Fixed64)5, new Vector3d(5, 3, centerZ), FixedQuaternion.Identity,
            secondAxisSign > 0 ? Vector3d.Forward : Vector3d.Backward,
            Signed192.Raw((Fixed64)10), (Fixed64)5);
        BigInteger radius = Integer(geometry.FirstRadius);
        BigInteger constant = squaredValue * radius * radius;
        BigInteger leading = (BigInteger)25 << geometry.ValueShift;
        const int words = 10;
        ulong[] polynomial = new ulong[2 * words];
        Write(constant, polynomial.AsSpan(0, words)); Write(leading, polynomial.AsSpan(words, words));
        sbyte[] signs = { -1, 1 };
        ulong[] cell = new ulong[WideFiniteAxisIntersection.GetFiniteValueRootCellWords(polynomial, signs)];
        Assert.True(WideFiniteAxisIntersection.TryGetFiniteValueRoot(polynomial, signs, 0, cell, out var root));
        // For c=(0,-3,1), the projected c and b are independent with a
        // nonzero dot. Rank-one eigenvalue compatibility requires
        // (S-25)^2=325, whereas the independently known H=0 value S=50
        // gives 625. Neither eigenvalue can supply a rank-one normal.
        // For c=(0,-3,0), S=41 does have the rank-one pencil
        // -64*(y-3)^2, but its recovered points have H=0 and cannot beat
        // earlier cap/side minima. Reversing b and its cap sign changes neither.
        for (int branch = 0; branch < 8; branch++)
        {
            Assert.False(CylinderPairRepeatedRim.TryGetRankOneNormal(geometry, 1, secondAxisSign,
                root, branch, out Vector3d rejectedNormal));
            Assert.Equal(Vector3d.Zero, rejectedNormal);
        }
        // The +X gap is5, below both sqrt41 and sqrt50. The point
        // (4,2,3) is strictly inside both cylinders, proving positive contact.
        Assert.True(FixedSegment.TryGetCenteredFiniteCylindersContact(
            Vector3d.Zero, FixedQuaternion.Identity, Vector3d.Right, (Fixed64)10, (Fixed64)5,
            new Vector3d(5, 3, centerZ), FixedQuaternion.Identity,
            secondAxisSign > 0 ? Vector3d.Forward : Vector3d.Backward,
            (Fixed64)10, (Fixed64)5, out FixedContactAnchors contact));
        Assert.InRange(contact.Depth.m_rawValue, 1L, 5L << 32);
        Assert.False(contact.DepthIsClamped);
    }

    [Fact]
    public void Contact_CapBeatsTheTangentHZeroSecondCircle()
    {
        // a=X, b=Z, c=(0,-3,1), p=(0,3,4), q=(0,15/4,0).
        // The tangent H=0 witness has depth25/4, but the +X cap has
        // gap15/4. The point (4,2,3) is strictly inside both cylinders.
        Assert.True(FixedSegment.TryGetCenteredFiniteCylindersContact(
            Vector3d.Zero, FixedQuaternion.Identity, Vector3d.Right, (Fixed64)10, (Fixed64)5,
            new Vector3d(5, 3, 4), FixedQuaternion.Identity, Vector3d.Forward,
            (Fixed64)10, Fixed64.FromRaw(15L << 30), out FixedContactAnchors contact));
        Assert.InRange(contact.Depth.m_rawValue, 1L, 15L << 30);
        Assert.False(contact.DepthIsClamped);
    }

    [Fact]
    public void Contact_SidePlaneRetainsTheHZeroNormalContinuumInBothOrders()
    {
        // c=(0,-5,0), p=(0,5,0), t=0. The H=0 continuum has S=25.
        // For unit n=(x,y,z) in the positive octant the gap is
        // 5*(sqrt(x²+y²)+sqrt(y²+z²)-y)>=5, with equality on either
        // side plane. Squaring the nonnegative terms reduces the inequality
        // to x²*z²>=0. Other octants cannot lower the center projection.
        Vector3d center = new(5, 5, 5);
        Assert.True(FixedSegment.TryGetCenteredFiniteCylindersContact(
            Vector3d.Zero, FixedQuaternion.Identity, Vector3d.Right, (Fixed64)10, (Fixed64)5,
            center, FixedQuaternion.Identity, Vector3d.Forward, (Fixed64)10, (Fixed64)5,
            out FixedContactAnchors contact));
        Assert.Equal((Fixed64)5, contact.Depth);
        Assert.Equal(Vector3d.Right, contact.Normal);
        Assert.False(contact.DepthIsClamped);
        Assert.True(FixedSegment.TryGetCenteredFiniteCylindersContact(
            center, FixedQuaternion.Identity, Vector3d.Forward, (Fixed64)10, (Fixed64)5,
            Vector3d.Zero, FixedQuaternion.Identity, Vector3d.Right, (Fixed64)10, (Fixed64)5,
            out FixedContactAnchors reversed));
        Assert.Equal(contact.Depth, reversed.Depth);
        Assert.Equal(Vector3d.Backward, reversed.Normal);
        Assert.False(reversed.DepthIsClamped);
    }

    [Fact]
    public void Zero_TangentCircleIntersectionLeavesTheBoundaryNormalToTheSideAndCap()
    {
        // p=(0,0,5), q=(3,4,0), c=-p-q. The first circle is tangent
        // to b.p=5, and the tangent-cross normal is Z. Its second radial
        // dot is zero, so this exact touch belongs to side/cap admission.
        var geometry = new CylinderPairGeometry(Vector3d.Zero, FixedQuaternion.Identity, Vector3d.Right,
            Signed192.Raw((Fixed64)10), (Fixed64)5, new Vector3d(8, 4, 10),
            FixedQuaternion.Identity, Vector3d.Forward, Signed192.Raw((Fixed64)10), (Fixed64)5);
        for (int branch = 0; branch < 2; branch++)
        {
            Assert.False(CylinderPairRepeatedRim.TryGetZeroNormal(geometry, 1, 1, branch, out Vector3d normal));
            Assert.Equal(Vector3d.Zero, normal);
        }
    }

    [Theory]
    [InlineData(1)]
    [InlineData(-1)]
    public void Zero_RecoversTheCommonSupportNormalFromTheTwoRimTangents(int direction)
    {
        // p=(0,3,4), q=(3,4,0), c=-p-q. The admitting tangent-cross
        // orientation is (9,12,16), so the gap is exactly zero.
        var geometry = new CylinderPairGeometry(Vector3d.Zero, FixedQuaternion.Identity, Vector3d.Right,
            Signed192.Raw((Fixed64)10), (Fixed64)5, new Vector3d(8 * direction, 7 * direction, 9 * direction),
            FixedQuaternion.Identity, Vector3d.Forward,
            Signed192.Raw((Fixed64)10), (Fixed64)5);
        Vector3d expected = new(Fixed64.FromRaw(direction * RoundRootRatio(81, 481)),
            Fixed64.FromRaw(direction * RoundRootRatio(144, 481)), Fixed64.FromRaw(direction * RoundRootRatio(256, 481)));
        int accepted = 0;
        for (int branch = 0; branch < 2; branch++)
        {
            if (!CylinderPairRepeatedRim.TryGetZeroNormal(geometry, direction, direction, branch, out Vector3d normal)) continue;
            accepted++;
            Assert.Equal(expected, normal);
        }
        Assert.Equal(1, accepted);
    }

    [Theory]
    [InlineData(1, 1, 1)]
    [InlineData(-1, -1, -1)]
    [InlineData(1, 1, -1)]
    public void RankOne_AdmitsAndRoundsTheUniqueInteriorNormalOnlyForMatchingCaps(
        int direction, int firstCapSign, int secondCapSign)
    {
        // Radius one, length ten, exact authored axes (3,0,4)/5 and
        // (-3,0,4)/5. The ++ cap offset is (0,-1,0). The selected minimum
        // has S=351/400 and n=(0,15,sqrt(399))/(4*sqrt(39)). Its pencil is
        // rank one, and this S is a repeated discriminant root.
        Fixed64 s = Fixed64.FromRaw(1920767767);
        var geometry = new CylinderPairGeometry(Vector3d.Zero,
            new FixedQuaternion(Fixed64.Zero, -s, Fixed64.Zero, 2 * s), Vector3d.Right,
            Signed192.Raw((Fixed64)10), Fixed64.One,
            new Vector3d(3 * (firstCapSign - secondCapSign), direction, 4 * (firstCapSign + secondCapSign)),
            new FixedQuaternion(Fixed64.Zero, s, Fixed64.Zero, 2 * s), Vector3d.Left,
            Signed192.Raw((Fixed64)10), Fixed64.One);
        BigInteger radius = Integer(geometry.FirstRadius);
        BigInteger constant = 351 * radius * radius;
        BigInteger leading = (BigInteger)400 << geometry.ValueShift;
        BigInteger gcd = BigInteger.GreatestCommonDivisor(constant, leading);
        constant /= gcd;
        leading /= gcd;
        const int words = 10;
        ulong[] polynomial = new ulong[2 * words];
        Write(constant, polynomial.AsSpan(0, words));
        Write(leading, polynomial.AsSpan(words, words));
        sbyte[] signs = { -1, 1 };
        ulong[] cell = new ulong[WideFiniteAxisIntersection.GetFiniteValueRootCellWords(polynomial, signs)];
        Assert.True(WideFiniteAxisIntersection.TryGetFiniteValueRoot(polynomial, signs, 0, cell, out var root));
        Vector3d expected = new(Fixed64.Zero,
            Fixed64.FromRaw(direction * RoundRootRatio(75, 208)),
            Fixed64.FromRaw(direction * RoundRootRatio(133, 208)));
        int accepted = 0;
        for (int branch = 0; branch < 8; branch++)
        {
            if (!CylinderPairRepeatedRim.TryGetRankOneNormal(geometry, firstCapSign, secondCapSign,
                root, branch, out Vector3d normal))
            {
                Assert.Equal(Vector3d.Zero, normal);
                continue;
            }
            accepted++;
            Assert.Equal(expected, normal);
        }
        // Changing cap signs and moving the center preserves c and S.
        // Both axis dots have the sign of n.Z, so opposite caps cannot
        // admit either repeated normal; the positive-Z point reaches the
        // second-cap rejection after passing the first-cap predicate.
        Assert.Equal(firstCapSign == secondCapSign ? 1 : 0, accepted);
    }

    [Fact]
    public void Zero_RejectsAnIntersectionWhoseRadialSupportSignsDisagree()
    {
        // p=(0,3,4), q=(3,-4,0), c=(-3,1,-4) give an exact rim
        // intersection. The rim-tangent cross is (-9,12,16), with
        // p.dot=100 and q.dot=-75, so neither orientation supports both.
        var geometry = new CylinderPairGeometry(Vector3d.Zero, FixedQuaternion.Identity,
            Vector3d.Right, Signed192.Raw((Fixed64)10), (Fixed64)5,
            new Vector3d(8, -1, 9), FixedQuaternion.Identity, Vector3d.Forward,
            Signed192.Raw((Fixed64)10), (Fixed64)5);
        for (int branch = 0; branch < 2; branch++)
        {
            Assert.False(CylinderPairRepeatedRim.TryGetZeroNormal(geometry, 1, 1,
                branch, out Vector3d normal));
            Assert.Equal(Vector3d.Zero, normal);
        }
    }

    [Fact]
    public void RankOne_LeavesARankZeroStationaryContinuumToTheSidePlane()
    {
        // a=X, b=Z, c=(0,-4,0), ra=2, rb=4. Every first-rim
        // p=(0,y,z), y²+z²=4, with q=(0,4,0) gives v=p and S=4.
        // T=64*C at this value: both spatial eigenvalues coincide and
        // the whole pencil matrix vanishes, not an isolated rank-one line.
        var geometry = new CylinderPairGeometry(Vector3d.Zero, FixedQuaternion.Identity,
            Vector3d.Right, Signed192.Raw((Fixed64)10), (Fixed64)2,
            new Vector3d(5, 4, 5), FixedQuaternion.Identity, Vector3d.Forward,
            Signed192.Raw((Fixed64)10), (Fixed64)4);
        BigInteger radius = Integer(geometry.FirstRadius);
        const int words = 10;
        ulong[] polynomial = new ulong[2 * words];
        Write(radius * radius, polynomial.AsSpan(0, words));
        Write(BigInteger.One << geometry.ValueShift, polynomial.AsSpan(words, words));
        sbyte[] signs = { -1, 1 };
        ulong[] cell = new ulong[WideFiniteAxisIntersection.GetFiniteValueRootCellWords(polynomial, signs)];
        Assert.True(WideFiniteAxisIntersection.TryGetFiniteValueRoot(polynomial, signs, 0, cell, out var root));
        for (int branch = 0; branch < 8; branch++)
        {
            Assert.False(CylinderPairRepeatedRim.TryGetRankOneNormal(geometry, 1, 1,
                root, branch, out Vector3d normal));
            Assert.Equal(Vector3d.Zero, normal);
        }
    }

    [Fact]
    public void RankOne_RejectsTheIncompatibleEigenvalueAndTheRemainingNonminimumPoints()
    {
        // a=Z, b=(3,0,4)/5, c=(0,1,1), ra=5/4, rb=1.
        // At S=41/16, T-4*C=-(8*x-6)^2/25. Its line x=3/4
        // meets the first circle at y=+-1. The negative point has H=0;
        // the positive point gives q=-Y, v=(3/4,1,1), and q.v=-1.
        // Neither can supply a common outward normal. The other spatial
        // eigenvalue has zero linear compatibility coefficient but a nonzero
        // constant: projected c and b are orthogonal, while b.c=4/5.
        Fixed64 s = Fixed64.FromRaw(1920767767);
        var geometry = new CylinderPairGeometry(Vector3d.Zero, FixedQuaternion.Identity,
            Vector3d.Forward, Signed192.Raw((Fixed64)10), Fixed64.FromRaw(5L << 30),
            new Vector3d(3, -1, 8),
            new FixedQuaternion(Fixed64.Zero, -s, Fixed64.Zero, 2 * s), Vector3d.Right,
            Signed192.Raw((Fixed64)10), Fixed64.One);
        BigInteger radius = Integer(geometry.FirstRadius);
        AssertNormalsAtCertifiedRepeatedValue(geometry, 1, 1,
            41 * radius * radius, (BigInteger)25 << geometry.ValueShift);
    }

    [Fact]
    public void RankOne_RejectsTheNonrealEigenvaluePairAtAnExactSideBoundary()
    {
        // a=Z, b=(3,0,4)/5, c=(0,1,0), ra=1, rb=2.
        // p=-Y and q=-2Y give c+p=0, S=4 and positive radial dots.
        // Both cap dots vanish, so this H=0 point belongs to the side plane.
        // The quadratic rank-one discriminant has the sign of
        // (1-4*9/25)*(1-9/25)<0; the other eigenvalue's remaining
        // minor is proportional to (4-1)*(4-1), and also cannot give rank one.
        Fixed64 s = Fixed64.FromRaw(1920767767);
        var geometry = new CylinderPairGeometry(Vector3d.Zero, FixedQuaternion.Identity,
            Vector3d.Forward, Signed192.Raw((Fixed64)10), Fixed64.One,
            new Vector3d(3, -1, 9),
            new FixedQuaternion(Fixed64.Zero, -s, Fixed64.Zero, 2 * s), Vector3d.Right,
            Signed192.Raw((Fixed64)10), (Fixed64)2);
        BigInteger radius = Integer(geometry.FirstRadius);
        AssertNormalsAtCertifiedRepeatedValue(geometry, 1, 1,
            4 * radius * radius, BigInteger.One << geometry.ValueShift);
    }

    [Fact]
    public void RankOne_RejectsTheZeroFirstRadialPointAndAdmitsItsCompanion()
    {
        // a=X, b=Z, c=(-1,0,-2), ra=2, rb=4, S=25.
        // T-80*C=-80*y². Its two points are p=2Z, q=-4X,
        // v=-5X (first radial dot zero), and p=-2Z, q=4X,
        // v=(3,0,-4) (radial dots8 and12). Only the second admits (+,-).
        var geometry = new CylinderPairGeometry(Vector3d.Zero, FixedQuaternion.Identity,
            Vector3d.Right, Signed192.Raw((Fixed64)10), (Fixed64)2,
            new Vector3d(6, 0, -3), FixedQuaternion.Identity, Vector3d.Forward,
            Signed192.Raw((Fixed64)10), (Fixed64)4);
        BigInteger unit = Integer(geometry.FirstRadius) / 2;
        Vector3d expected = new(Fixed64.FromRaw(RoundRootRatio(9, 25)), Fixed64.Zero,
            Fixed64.FromRaw(-RoundRootRatio(16, 25)));
        AssertNormalsAtCertifiedRepeatedValue(geometry, 1, -1,
            25 * unit * unit, BigInteger.One << geometry.ValueShift, expected);
    }

    private static void AssertNormalsAtCertifiedRepeatedValue(in CylinderPairGeometry geometry,
        int firstCapSign, int secondCapSign, BigInteger numerator, BigInteger denominator,
        params Vector3d[] expected)
    {
        BigInteger gcd = BigInteger.GreatestCommonDivisor(numerator, denominator);
        numerator /= gcd;
        denominator /= gcd;
        const int words = 10;
        ulong[] polynomial = new ulong[2 * words];
        Write(numerator, polynomial.AsSpan(0, words));
        Write(denominator, polynomial.AsSpan(words, words));
        sbyte[] signs = { -1, 1 };
        ulong[] cell = new ulong[WideFiniteAxisIntersection.GetFiniteValueRootCellWords(polynomial, signs)];
        Assert.True(WideFiniteAxisIntersection.TryGetFiniteValueRoot(polynomial, signs, 0, cell, out var root));

        // Independently certify that this rational value is a repeated root
        // of the geometry's discriminant, not merely an arbitrary test root.
        const int valueWords = CylinderPairRimFeatures.CoefficientWords;
        ulong[] values = new ulong[9 * valueWords];
        sbyte[] valueSigns = new sbyte[9];
        CylinderPairRimFeatures.BuildValues(geometry, firstCapSign, secondCapSign, values, valueSigns);
        BigInteger value = BigInteger.Zero, derivative = BigInteger.Zero;
        for (int power = 0; power < 9; power++)
        {
            BigInteger coefficient = BigInteger.Zero;
            for (int word = valueWords - 1; word >= 0; word--)
                coefficient = (coefficient << 64) | values[power * valueWords + word];
            coefficient *= valueSigns[power];
            value += coefficient * BigInteger.Pow(numerator, power) * BigInteger.Pow(denominator, 8 - power);
            if (power != 0)
                derivative += power * coefficient * BigInteger.Pow(numerator, power - 1)
                    * BigInteger.Pow(denominator, 8 - power);
        }
        Assert.Equal(BigInteger.Zero, value);
        Assert.Equal(BigInteger.Zero, derivative);
        var actual = new HashSet<Vector3d>();
        int accepted = 0;
        for (int branch = 0; branch < 8; branch++)
        {
            if (CylinderPairRepeatedRim.TryGetRankOneNormal(geometry, firstCapSign, secondCapSign,
                    root, branch, out Vector3d normal))
            {
                accepted++;
                actual.Add(normal);
            }
            else
                Assert.Equal(Vector3d.Zero, normal);
        }
        Assert.Equal(expected.Length, accepted);
        Assert.True(actual.SetEquals(expected));
    }

    [Fact]
    public void RankOne_TangentLineAdmitsTheMergedCardinalNormalOnlyOnce()
    {
        // a=(3,0,4)/5, b=(-3,0,4)/5, c=(0,-32,0), radii25.
        // The symmetric rank-one family merges at p=q=(0,25,0):
        // v=(0,18,0), S=324, H=350. Its first-circle line is tangent,
        // not a pair of distinct points, and both cap dots are exactly zero.
        Fixed64 s = Fixed64.FromRaw(1920767767);
        var geometry = new CylinderPairGeometry(Vector3d.Zero,
            new FixedQuaternion(Fixed64.Zero, -s, Fixed64.Zero, 2 * s), Vector3d.Right,
            Signed192.Raw((Fixed64)10), (Fixed64)25, new Vector3d(0, 32, 8),
            new FixedQuaternion(Fixed64.Zero, s, Fixed64.Zero, 2 * s), Vector3d.Left,
            Signed192.Raw((Fixed64)10), (Fixed64)25);
        BigInteger unit = Integer(geometry.FirstRadius) / 25;
        const int words = 10;
        ulong[] polynomial = new ulong[2 * words];
        Write(324 * unit * unit, polynomial.AsSpan(0, words));
        Write(BigInteger.One << geometry.ValueShift, polynomial.AsSpan(words, words));
        sbyte[] signs = { -1, 1 };
        ulong[] cell = new ulong[WideFiniteAxisIntersection.GetFiniteValueRootCellWords(polynomial, signs)];
        Assert.True(WideFiniteAxisIntersection.TryGetFiniteValueRoot(polynomial, signs, 0, cell, out var root));
        int accepted = 0;
        for (int branch = 0; branch < 8; branch++)
        {
            if (CylinderPairRepeatedRim.TryGetRankOneNormal(geometry, 1, 1, root,
                    branch, out Vector3d normal))
            {
                accepted++;
                Assert.Equal(Vector3d.Up, normal);
            }
            else
            {
                Assert.Equal(Vector3d.Zero, normal);
            }
        }
        Assert.Equal(1, accepted);
    }

    private static long RoundRootRatio(BigInteger numerator, BigInteger denominator)
    {
        BigInteger target = (BigInteger)numerator << 64;
        long lower = 0;
        long upper = 1L << 32;
        while (lower < upper)
        {
            long middle = lower + (upper - lower + 1) / 2;
            if ((BigInteger)middle * middle * denominator <= target)
                lower = middle;
            else
                upper = middle - 1;
        }
        BigInteger twice = 2 * (BigInteger)lower + 1;
        int midpoint = (4 * target).CompareTo(twice * twice * denominator);
        return lower + (midpoint > 0 || (midpoint == 0 && (lower & 1) != 0) ? 1 : 0);
    }

    private static BigInteger Integer(Signed320 value) =>
        ((BigInteger)(long)value.Word4 << 256) | ((BigInteger)value.Word3 << 192)
        | ((BigInteger)value.Word2 << 128) | ((BigInteger)value.Word1 << 64) | value.Word0;

    private static void Write(BigInteger value, Span<ulong> words)
    {
        words.Clear();
        for (int index = 0; index < words.Length; index++)
        {
            words[index] = (ulong)(value & ulong.MaxValue);
            value >>= 64;
        }
        Assert.Equal(BigInteger.Zero, value);
    }
}
