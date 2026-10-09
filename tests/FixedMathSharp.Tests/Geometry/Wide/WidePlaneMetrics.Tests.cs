//=======================================================================
// WidePlaneMetrics.Tests.cs
//=======================================================================
// MIT License, Copyright (c) 2024-present David Oravsky (mrdav30)
// See LICENSE file in the project root for full license information.
//=======================================================================
using System;
using System.Numerics;
using FixedMathSharp.Geometry;
using Xunit;

namespace FixedMathSharp.Tests.Geometry.Wide;

public sealed class WidePlaneMetricsTests
{
    [Theory]
    [InlineData(0)]
    [InlineData(1)]
    [InlineData(2)]
    public void SpanAndArea_AgreeWithIntegerProjectionOracle(int frame)
    {
        GetFrame(frame, out FixedTriangle plane, out FixedQuaternion rotation);
        var metrics = new WidePlaneMetrics(plane, rotation);
        BigInteger[] normal = Normal(plane, rotation);
        BigInteger m = Dot(normal, normal);
        Vector3d[] points = Points();
        for (int i = 0; i < points.Length; i++)
        {
            Vector3d a = points[i], b = points[(i + 1) % points.Length], c = points[(i + 3) % points.Length];
            BigInteger[] u = Sub(Project(b, normal, m), Project(a, normal, m));
            BigInteger[] v = Sub(Project(c, normal, m), Project(a, normal, m));
            BigInteger[] cross = Cross(u, v);
            Assert.Equal(Dot(u, u), Integer(metrics.GetSpanSquared(a, b)) * m);
            Assert.Equal(Dot(cross, cross), Integer(metrics.GetAreaSquared(a, b, c)) * m * m * m);
            Assert.Equal(Integer(metrics.GetSpanSquared(a, b)), Integer(metrics.GetSpanSquared(b, a)));
            Assert.Equal(Integer(metrics.GetAreaSquared(a, b, c)), Integer(metrics.GetAreaSquared(c, b, a)));
        }
    }

    [Theory]
    [InlineData(0)]
    [InlineData(1)]
    [InlineData(2)]
    public void FilledTriangleDistance_AgreesWithIndependentProjectedGramOracle(int frame)
    {
        GetFrame(frame, out FixedTriangle plane, out FixedQuaternion rotation);
        var metrics = new WidePlaneMetrics(plane, rotation);
        BigInteger[] normal = Normal(plane, rotation);
        BigInteger m = Dot(normal, normal);
        Vector3d[] points = Points();
        for (int i = 0; i < points.Length; i++)
        for (int j = 0; j < points.Length; j++)
        {
            Vector3d a = points[i], b = points[(i + 1) % points.Length];
            Vector3d c = points[(i + 2) % points.Length], p = points[j];
            var expected = Distance(Project(p, normal, m), Project(a, normal, m),
                Project(b, normal, m), Project(c, normal, m));
            metrics.GetTriangleDistanceSquared(p, a, b, c, out Signed832 n, out Signed832 d);
            Assert.True(d.Sign > 0);
            Assert.True(n.Sign >= 0);
            Assert.Equal(expected.N * Integer(d), Integer(n) * expected.D * m * m);
            metrics.GetTriangleDistanceSquared(p, c, b, a, out Signed832 rn, out Signed832 rd);
            Assert.Equal(0, WidePlaneMetrics.CompareRatios(n, d, rn, rd));
        }
    }

    [Theory]
    [InlineData(1, 7, 1, 0, 1)] // interior projection ignores its normal offset
    [InlineData(2, -9, 0, 0, 1)] // closed edge
    [InlineData(3, 0, 3, 2, 1)] // nearest point is inside the hypotenuse
    [InlineData(-1, 0, -1, 2, 1)] // vertex region
    [InlineData(5, 0, 0, 1, 1)] // beyond an edge endpoint
    public void FilledTriangleDistance_PreservesClosedFeatureRegions(long x, long y, long z, long n, long d)
    {
        var plane = new FixedTriangle(Raw(0, 0, 0), Raw(4, 0, 0), Raw(0, 0, 4));
        var metrics = new WidePlaneMetrics(plane, FixedQuaternion.Identity);
        metrics.GetTriangleDistanceSquared(Raw(x, y, z), plane.A, plane.B, plane.C,
            out Signed832 actualN, out Signed832 actualD);
        Assert.Equal(n * Integer(actualD), d * Integer(actualN));
    }

    [Fact]
    public void DegenerateProjectedTriangles_ReduceToTheirSegmentOrPoint()
    {
        var plane = new FixedTriangle(Raw(0, 0, 0), Raw(4, 0, 0), Raw(0, 0, 4));
        var metrics = new WidePlaneMetrics(plane, FixedQuaternion.Identity);
        metrics.GetTriangleDistanceSquared(Raw(2, 50, 3), Raw(0, 0, 0), Raw(4, 1, 0), Raw(2, 2, 0),
            out Signed832 n, out Signed832 d);
        Assert.Equal(9 * Integer(d), Integer(n));
        metrics.GetTriangleDistanceSquared(Raw(2, 50, 3), Raw(0, 0, 0), Raw(0, 1, 0), Raw(0, 2, 0), out n, out d);
        Assert.Equal(13 * Integer(d), Integer(n));
    }

    [Fact]
    public void RationalDistances_CompareBeyondFixedCarrierProductWidth()
    {
        GetFrame(2, out FixedTriangle plane, out FixedQuaternion rotation);
        var metrics = new WidePlaneMetrics(plane, rotation);
        Vector3d[] points = Points();
        metrics.GetTriangleDistanceSquared(points[0], points[1], points[2], points[3], out Signed832 a, out Signed832 b);
        metrics.GetTriangleDistanceSquared(points[4], points[1], points[2], points[3], out Signed832 c, out Signed832 d);
        BigInteger left = Integer(a) * Integer(d), right = Integer(c) * Integer(b);
        Assert.True(left.GetBitLength() > 832 || right.GetBitLength() > 832);
        int expected = left.CompareTo(right);
        Assert.Equal(expected, WidePlaneMetrics.CompareRatios(a, b, c, d));
        Assert.Equal(-expected, WidePlaneMetrics.CompareRatios(c, d, a, b));
        Assert.Equal(0, WidePlaneMetrics.CompareRatios(a, b, a, b));
    }

    [Fact]
    public void Construction_RequiresAnActualPlaneAndRotation()
    {
        var plane = new FixedTriangle(Vector3d.Zero, Vector3d.Right, Vector3d.Forward);
        Assert.Throws<ArgumentException>(() => new WidePlaneMetrics(default, FixedQuaternion.Identity));
        Assert.Throws<ArgumentException>(() => new WidePlaneMetrics(plane, default));
    }

    private static void GetFrame(int index, out FixedTriangle plane, out FixedQuaternion rotation)
    {
        plane = index == 2
            ? new FixedTriangle(Raw(long.MinValue, long.MaxValue, -1), Raw(long.MaxValue, -3, long.MinValue), Raw(5, long.MinValue, long.MaxValue))
            : new FixedTriangle(Raw(0, 0, 0), Raw(5, 2, -1), Raw(-2, 3, 7));
        rotation = index == 0 ? FixedQuaternion.Identity
            : index == 1 ? new FixedQuaternion(Fixed64.FromRaw(2), Fixed64.FromRaw(-3), Fixed64.FromRaw(5), Fixed64.FromRaw(7))
            : new FixedQuaternion(Fixed64.MinValue, Fixed64.MaxValue, Fixed64.FromRaw(long.MaxValue - 1), Fixed64.FromRaw(long.MinValue + 1));
    }

    private static Vector3d[] Points() => new[]
    {
        Raw(0, 0, 0), Raw(3, -7, 11), Raw(-13, 17, -19), Raw(23, 29, -31),
        Raw(long.MinValue, long.MaxValue, -1), Raw(long.MaxValue, long.MinValue, 1),
        Raw(long.MaxValue - 1, long.MaxValue, long.MinValue)
    };

    private static Vector3d Raw(long x, long y, long z) => new(Fixed64.FromRaw(x), Fixed64.FromRaw(y), Fixed64.FromRaw(z));
    private static BigInteger[] V(Vector3d p) => new BigInteger[] { p.X.m_rawValue, p.Y.m_rawValue, p.Z.m_rawValue };
    private static BigInteger[] Sub(BigInteger[] a, BigInteger[] b) => new[] { a[0] - b[0], a[1] - b[1], a[2] - b[2] };
    private static BigInteger Dot(BigInteger[] a, BigInteger[] b) => a[0] * b[0] + a[1] * b[1] + a[2] * b[2];
    private static BigInteger[] Cross(BigInteger[] a, BigInteger[] b) => new[]
        { a[1] * b[2] - a[2] * b[1], a[2] * b[0] - a[0] * b[2], a[0] * b[1] - a[1] * b[0] };

    private static BigInteger[] Normal(FixedTriangle plane, FixedQuaternion rotation)
    {
        BigInteger[] n = Cross(Sub(V(plane.B), V(plane.A)), Sub(V(plane.C), V(plane.A)));
        BigInteger x = rotation.X.m_rawValue, y = rotation.Y.m_rawValue, z = rotation.Z.m_rawValue, w = rotation.W.m_rawValue;
        return new[]
        {
            (x*x-y*y-z*z+w*w)*n[0] + 2*(x*y-z*w)*n[1] + 2*(x*z+y*w)*n[2],
            2*(x*y+z*w)*n[0] + (-x*x+y*y-z*z+w*w)*n[1] + 2*(y*z-x*w)*n[2],
            2*(x*z-y*w)*n[0] + 2*(y*z+x*w)*n[1] + (-x*x-y*y+z*z+w*w)*n[2]
        };
    }

    private static BigInteger[] Project(Vector3d point, BigInteger[] normal, BigInteger m)
    {
        BigInteger[] p = V(point);
        BigInteger along = Dot(p, normal);
        return new[] { m*p[0]-along*normal[0], m*p[1]-along*normal[1], m*p[2]-along*normal[2] };
    }

    private static (BigInteger N, BigInteger D) Distance(BigInteger[] p, BigInteger[] a, BigInteger[] b, BigInteger[] c)
    {
        BigInteger[] u = Sub(b, a), v = Sub(c, a), w = Sub(p, a);
        BigInteger uu = Dot(u, u), uv = Dot(u, v), vv = Dot(v, v), wu = Dot(w, u), wv = Dot(w, v);
        BigInteger determinant = uu*vv-uv*uv, s = wu*vv-wv*uv, t = wv*uu-wu*uv;
        if (determinant > 0 && s >= 0 && t >= 0 && s+t <= determinant) return (0, 1);
        var best = Segment(p, a, b);
        var next = Segment(p, b, c);
        if (next.N*best.D < best.N*next.D) best = next;
        next = Segment(p, c, a);
        return next.N*best.D < best.N*next.D ? next : best;
    }

    private static (BigInteger N, BigInteger D) Segment(BigInteger[] p, BigInteger[] a, BigInteger[] b)
    {
        BigInteger[] v = Sub(p, a), e = Sub(b, a);
        BigInteger vv = Dot(v, v), ee = Dot(e, e), ve = Dot(v, e);
        if (ee == 0 || ve <= 0) return (vv, 1);
        if (ve >= ee) { v = Sub(p, b); return (Dot(v, v), 1); }
        return (vv*ee-ve*ve, ee);
    }

    private static BigInteger Integer(Signed832 value)
    {
        Span<ulong> words = stackalloc ulong[13];
        WideArithmetic.GetMagnitude(value, words);
        BigInteger result = 0;
        for (int i = words.Length - 1; i >= 0; i--) result = (result << 64) + words[i];
        return value.Sign < 0 ? -result : result;
    }
}
