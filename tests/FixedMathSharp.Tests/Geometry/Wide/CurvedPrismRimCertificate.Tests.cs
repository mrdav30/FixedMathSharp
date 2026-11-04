using System;
using System.Numerics;
using System.Reflection;
using FixedMathSharp.Geometry;
using Xunit;

namespace FixedMathSharp.Tests.Geometry;

public sealed class CurvedPrismRimCertificateTests
{
    private delegate bool EllipseCertificate(in CurvedPrismContactGeometry geometry,
        in WideDirection3d edge, in WideDirection3d offset);

    private delegate bool PhysicalConcavity(in CurvedPrismContactGeometry geometry,
        in WideDirection3d edge, in WideDirection3d offset, in WideDirection3d first,
        in WideDirection3d second, int cap, int firstAxialSign);

    private static readonly Vector2d[] Footprint =
        { new(-Fixed64.One, -Fixed64.One), new(Fixed64.One, -Fixed64.One),
          new(Fixed64.One, Fixed64.One), new(-Fixed64.One, Fixed64.One) };

    [Fact]
    public void EllipseCertificate_RetainsBothExactEqualityBoundaries()
    {
        EllipseCertificate excludes = Certificate<EllipseCertificate>("HasNoNonnegativeEllipseMinimum");
        var geometry = Geometry(Vector3d.Right, (Fixed64)15);
        BigInteger radius = Number(Signed832.ExtendValue(geometry.Radius));
        Assert.Equal(BigInteger.Zero, radius % 15);
        foreach (int delta in new[] { -1, 0, 1 })
        {
            // b=R/sqrt(2): the major-radius bound is the larger one.
            CheckEllipse(excludes, geometry, Vector(1, 1, 0), Vector(0, 0, radius + delta), radius, delta > 0);
            // b=3R/5: the evolute radius is 16R/15, strictly above R.
            CheckEllipse(excludes, geometry, Vector(3, 4, 0), Vector(0, 0, 16 * radius / 15 + delta), radius, delta > 0);
        }
    }

    [Fact]
    public void EllipseCertificate_PreservesCompleteProductsNearTheCoordinateBounds()
    {
        EllipseCertificate excludes = Certificate<EllipseCertificate>("HasNoNonnegativeEllipseMinimum");
        var geometry = Geometry(new Vector3d(Fixed64.FromRaw((1L << 32) - 1), Fixed64.FromRaw(1), Fixed64.Zero), Fixed64.One);
        BigInteger[] axis = Axis(geometry);
        BigInteger radius = Number(Signed832.ExtendValue(geometry.Radius));
        BigInteger s = (BigInteger.One << 165) - 1;
        BigInteger[] edge = Vector(s, s - 1, s - 2), offset = Vector(s - 3, 4 - s, s - 5);
        foreach (int sign in new[] { -1, 1 })
        {
            BigInteger[] e = Scale(edge, sign);
            WideDirection3d wideEdge = Wide(e), wideOffset = Wide(offset);
            Assert.True(EllipseOracle(e, offset, axis, radius));
            Assert.True(excludes(geometry, wideEdge, wideOffset));
        }
    }

    [Fact]
    public void PhysicalConcavity_PreservesExactEqualityBothCapsAndEquatorOrientations()
    {
        PhysicalConcavity excludes = Certificate<PhysicalConcavity>("IsPhysicalArcConcave");
        var geometry = Geometry(Vector3d.Right, Fixed64.One);
        BigInteger[] edge = Vector(1, 4, 8), first = Vector(8, 2, -2), last = Vector(-4, 11, -5);
        // E²=81, F²=72, U=8: rp²Ke=8²*81=72²=d²Kp² exactly.
        // For F'=10F+L the radial ratio is1586/7362>1/9; for
        // F'=10F-L it is306/7362<1/9. This brackets the exact boundary.
        foreach (int perturbation in new[] { -1, 0, 1 })
        {
            BigInteger[] f = perturbation == 0 ? first : Add(Scale(first, 10), Scale(last, perturbation));
            BigInteger[] l = perturbation == 0 ? last : Cross(edge, f);
            foreach (int cap in new[] { -1, 1 })
                foreach (int edgeSign in new[] { -1, 1 })
                    foreach (bool reverse in new[] { false, true })
                    {
                        BigInteger[] a = Scale(f, cap), b = Scale(l, cap);
                        if (reverse) (a, b) = (b, a);
                        CheckPhysical(excludes, geometry, Scale(edge, edgeSign), Vector(0, 0, 0), a, b, cap, perturbation >= 0);
                    }
        }
        foreach (int cap in new[] { -1, 1 })
        {
            BigInteger[] f = Scale(first, cap), l = Scale(last, cap);
            CheckPhysical(excludes, geometry, edge, Add(f, l), f, l, cap, true);
            CheckPhysical(excludes, geometry, edge, Scale(f, -1), f, l, cap, false);
            // c.F=0 but c.equator<0 exercises the second offset guard.
            CheckPhysical(excludes, geometry, edge, Scale(l, -1), f, l, cap, false);
        }
    }

    [Fact]
    public void PhysicalConcavity_PreservesCompleteWideProductsAgainstBigInteger()
    {
        PhysicalConcavity excludes = Certificate<PhysicalConcavity>("IsPhysicalArcConcave");
        var geometry = Geometry(new Vector3d(Fixed64.FromRaw((1L << 32) - 1), Fixed64.FromRaw(1), Fixed64.Zero), Fixed64.One);
        BigInteger[] e = Vector(1, 4, 8);
        BigInteger[] f = Add(Scale(Vector(8, 2, -2), (BigInteger.One << 157) - 1), Vector(-4, 11, -5));
        BigInteger[] l = Cross(e, f);
        e = Scale(e, (BigInteger.One << 161) - 1);
        CurvatureProducts(e, f, Axis(geometry), out BigInteger left, out BigInteger right);
        Assert.True(BigInteger.Max(left, right).GetBitLength() > 1100);
        CheckPhysical(excludes, geometry, e, Vector(0, 0, 0), f, l, 1, left >= right);
    }

    private static void CheckEllipse(EllipseCertificate excludes, in CurvedPrismContactGeometry geometry,
        BigInteger[] edge, BigInteger[] offset, BigInteger radius, bool expected)
    {
        Assert.Equal(expected, EllipseOracle(edge, offset, Axis(geometry), radius));
        WideDirection3d e = Wide(edge), c = Wide(offset);
        Assert.Equal(expected, excludes(geometry, e, c));
    }

    private static void CheckPhysical(PhysicalConcavity excludes, in CurvedPrismContactGeometry geometry,
        BigInteger[] edge, BigInteger[] offset, BigInteger[] first, BigInteger[] last, int cap, bool expected)
    {
        BigInteger[] axis = Axis(geometry);
        BigInteger uf = Dot(axis, first), ul = Dot(axis, last);
        Assert.True(uf * ul < 0);
        Assert.Equal(BigInteger.Zero, Dot(first, last));
        Assert.Equal(BigInteger.Zero, Dot(edge, first));
        Assert.Equal(BigInteger.Zero, Dot(edge, last));
        // Independent cleared positive affine equator: no cross-ray scale
        // or orientation shortcut from the runtime certificate is reused.
        BigInteger[] equator = Scale(Subtract(Scale(first, ul), Scale(last, uf)), (ul - uf).Sign);
        BigInteger[] physical = cap * uf > 0 ? first : last;
        CurvatureProducts(edge, physical, axis, out BigInteger left, out BigInteger right);
        bool oracle = Dot(offset, physical) >= 0 && Dot(offset, equator) >= 0 && left >= right;
        Assert.Equal(expected, oracle);
        WideDirection3d e = Wide(edge), c = Wide(offset), f = Wide(first), second = Wide(Subtract(last, first));
        Assert.Equal(expected, excludes(geometry, e, c, f, second, cap, uf.Sign));
    }

    private static void CurvatureProducts(BigInteger[] edge, BigInteger[] physical, BigInteger[] axis,
        out BigInteger left, out BigInteger right)
    {
        BigInteger q = Dot(axis, axis), k = q * Dot(physical, physical);
        BigInteger radial = k - BigInteger.Pow(Dot(axis, physical), 2);
        left = radial * radial * q * Dot(edge, edge);
        right = BigInteger.Pow(Dot(axis, edge), 2) * k * k;
    }

    private static bool EllipseOracle(BigInteger[] edge, BigInteger[] offset, BigInteger[] axis, BigInteger radius)
    {
        BigInteger e2 = Dot(edge, edge), d = Dot(edge, axis), k = e2 * Dot(axis, axis);
        BigInteger c = e2 * Dot(offset, offset) - BigInteger.Pow(Dot(edge, offset), 2);
        return c > radius * radius * e2
            && c * k * d * d > radius * radius * e2 * BigInteger.Pow(k - d * d, 2);
    }

    private static CurvedPrismContactGeometry Geometry(Vector3d axis, Fixed64 radius) =>
        new(Vector3d.Zero, FixedQuaternion.Identity, axis, Fixed64.Two, radius,
            Vector3d.Zero, Fixed64.Zero, Footprint, Fixed64.One, false, Span<WideDirection3d>.Empty);

    private static T Certificate<T>(string name) where T : Delegate =>
        typeof(CurvedPrismRimContacts).GetMethod(name, BindingFlags.NonPublic | BindingFlags.Static)!.CreateDelegate<T>();

    private static BigInteger[] Axis(in CurvedPrismContactGeometry geometry) =>
        Vector(Number(Signed832.ExtendValue(geometry.Axis.X)), Number(Signed832.ExtendValue(geometry.Axis.Y)), Number(Signed832.ExtendValue(geometry.Axis.Z)));

    private static BigInteger[] Vector(BigInteger x, BigInteger y, BigInteger z) => new[] { x, y, z };
    private static BigInteger Dot(BigInteger[] a, BigInteger[] b) => a[0] * b[0] + a[1] * b[1] + a[2] * b[2];
    private static BigInteger[] Scale(BigInteger[] v, BigInteger s) => Vector(v[0] * s, v[1] * s, v[2] * s);
    private static BigInteger[] Add(BigInteger[] a, BigInteger[] b) => Vector(a[0] + b[0], a[1] + b[1], a[2] + b[2]);
    private static BigInteger[] Subtract(BigInteger[] a, BigInteger[] b) => Vector(a[0] - b[0], a[1] - b[1], a[2] - b[2]);
    private static BigInteger[] Cross(BigInteger[] a, BigInteger[] b) =>
        Vector(a[1] * b[2] - a[2] * b[1], a[2] * b[0] - a[0] * b[2], a[0] * b[1] - a[1] * b[0]);
    private static WideDirection3d Wide(BigInteger[] v) => new(Encode(v[0]), Encode(v[1]), Encode(v[2]));

    private static Signed576 Encode(BigInteger value)
    {
        Assert.InRange(value, -(BigInteger.One << 575), (BigInteger.One << 575) - 1);
        if (value.Sign < 0) value += BigInteger.One << 576;
        ulong[] words = new ulong[9];
        for (int i = 0; i < words.Length; i++) { words[i] = (ulong)(value & ulong.MaxValue); value >>= 64; }
        return new(words[8], words[7], words[6], words[5], words[4], words[3], words[2], words[1], words[0]);
    }

    private static BigInteger Number(Signed832 value)
    {
        Span<ulong> words = stackalloc ulong[13];
        WideArithmetic.GetMagnitude(value, words);
        BigInteger result = 0;
        for (int i = words.Length - 1; i >= 0; i--) result = (result << 64) + words[i];
        return value.Sign * result;
    }
}
