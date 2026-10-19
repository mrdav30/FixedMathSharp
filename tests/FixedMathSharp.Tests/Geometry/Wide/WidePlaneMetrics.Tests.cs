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
    [InlineData(false)]
    [InlineData(true)]
    public void AffineAxisMetrics_ShouldMatchIndependentFullDomainProjection(bool oblique)
    {
        Signed576 unit = new(0, 0, 1UL << 16, 0, 0, 0, 0, 0, 0);
        var axis = new WideDirection3d(oblique ? unit : default, WideArithmetic.MultiplySigned576(unit, -3),
            oblique ? WideArithmetic.MultiplySigned576(unit, 2) : default);
        BigInteger scale = BigInteger.One << 400;
        BigInteger[] normal = { oblique ? scale : 0, -3 * scale, oblique ? 2 * scale : 0 };
        BigInteger metric = Dot(normal, normal);
        const int words = 32;
        Span<ulong> values = stackalloc ulong[4 * words]; Span<int> signs = stackalloc int[4];
        var numerator = ContactQuadratic.At(values, signs, 0, words);
        var denominator = ContactQuadratic.At(values, signs, 1, words);
        Vector3d[] points = Points();
        for (int i = 0; i < points.Length; i++)
        {
            Vector3d a = points[i], b = points[(i + 1) % points.Length], c = points[(i + 2) % points.Length];
            BigInteger[] u = Sub(Project(b, normal, metric), Project(a, normal, metric));
            BigInteger[] v = Sub(Project(c, normal, metric), Project(a, normal, metric));
            WidePlaneMetrics.GetSpanSquared(axis, a, b, numerator);
            Assert.Equal(Dot(u, u), Integer(numerator) * metric);
            WidePlaneMetrics.GetAreaSquared(axis, a, b, c, numerator);
            Assert.Equal(Dot(Cross(u, v), Cross(u, v)), Integer(numerator) * metric * metric * metric);
            Assert.Equal(Dot(normal, Cross(Sub(V(b), V(a)), Sub(V(c), V(a)))).Sign,
                WidePlaneMetrics.GetAreaSign(axis, a, b, c));
            foreach (Vector3d point in points)
            {
                var expected = Distance(Project(point, normal, metric), Project(a, normal, metric),
                    Project(b, normal, metric), Project(c, normal, metric));
                WidePlaneMetrics.GetTriangleDistanceSquared(axis, point, a, b, c, numerator, denominator);
                Assert.True(Integer(numerator) >= 0 && Integer(denominator) > 0);
                Assert.Equal(expected.N * Integer(denominator), Integer(numerator) * expected.D * metric * metric);
            }
        }
        // A collapsed projected triangle still has a finite point-distance metric.
        WidePlaneMetrics.GetTriangleDistanceSquared(axis, Vector3d.Right, Vector3d.Zero,
            Vector3d.Zero, Vector3d.Zero, numerator, denominator);
        var projected = Project(Vector3d.Right, normal, metric);
        Assert.Equal(Dot(projected, projected) * Integer(denominator), Integer(numerator) * metric * metric);
    }

    private static BigInteger Integer(ContactQuadratic value)
    {
        BigInteger result = 0;
        for (int index = value.Rational.Length - 1; index >= 0; index--) result = (result << 64) + value.Rational[index];
        return value.Signs[0] < 0 ? -result : result;
    }

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

    [Fact]
    public void ScaledArea_WithRigidFrame_ShouldRetainMetricScaleAfterNormalCancellation()
    {
        var triangle = new FixedTriangle(Vector3d.Zero, Vector3d.Forward, Vector3d.Right);
        WideTriangleRelations.GetScaledAreaAndNormal(triangle, ScaledRigidFrame3d.Rigid(Vector3d.One, FixedQuaternion.Identity),
            out Fixed64 area, out Vector3d normal);
        Assert.Equal(Fixed64.Half, area);
        Assert.Equal(Vector3d.Up, normal);
    }

    [Theory]
    [InlineData(0)]
    [InlineData(37)]
    public void ScaledPlaneMetrics_ShouldMatchIndependentUnroundedAffineNormal(int degrees)
    {
        var plane = new FixedTriangle(new Vector3d(-2, 1, 0), new Vector3d(3, -1, 2), new Vector3d(1, 2, 4));
        var rotation = FixedQuaternion.FromEulerAnglesInDegrees((Fixed64)degrees, (Fixed64)degrees, (Fixed64)degrees);
        Vector3d scale = new(Fixed64.FromFraction(2, 3), Fixed64.FromFraction(3, 5), Fixed64.FromFraction(5, 7));
        Vector3d part = new(Fixed64.FromFraction(7, 9), Fixed64.One, Fixed64.FromFraction(11, 13));
        var frame = new ScaledRigidFrame3d(Vector3d.One, rotation, Vector3d.Zero, scale, rotation, Vector3d.Zero, part);
        var metrics = new WidePlaneMetrics(plane, frame);
        BigInteger[] firstEdge = Sub(V(plane.B), V(plane.A)), secondEdge = Sub(V(plane.C), V(plane.A));
        for (int axis = 0; axis < 3; axis++)
        {
            BigInteger factor = (BigInteger)scale[axis].m_rawValue * part[axis].m_rawValue;
            firstEdge[axis] *= factor; secondEdge[axis] *= factor;
        }
        BigInteger[] local = Cross(firstEdge, secondEdge), normal = local;
        if (degrees != 0)
        {
            BigInteger x = rotation.X.m_rawValue, y = rotation.Y.m_rawValue, z = rotation.Z.m_rawValue, w = rotation.W.m_rawValue;
            normal = new[] { (w*w+x*x-y*y-z*z)*local[0]+2*(x*y-z*w)*local[1]+2*(x*z+y*w)*local[2],
                2*(x*y+z*w)*local[0]+(w*w-x*x+y*y-z*z)*local[1]+2*(y*z-x*w)*local[2],
                2*(x*z-y*w)*local[0]+2*(y*z+x*w)*local[1]+(w*w-x*x-y*y+z*z)*local[2] };
        }
        Vector3d a = Raw(-7, 11, -13), b = Raw(17, -19, 23), c = Raw(29, 31, -37);
        BigInteger[] edge = Sub(V(b), V(a));
        Assert.Equal(Dot(normal, normal)*Dot(edge, edge)-BigInteger.Pow(Dot(normal, edge), 2), Integer(metrics.GetSpanSquared(a,b)));
        BigInteger area = Dot(normal, Cross(edge, Sub(V(c), V(a))));
        Assert.Equal(area*area, Integer(metrics.GetAreaSquared(a,b,c)));
        Assert.Throws<ArgumentException>(() => new WidePlaneMetrics(default(FixedTriangle), frame));
        var rigid = new WidePlaneMetrics(plane, ScaledRigidFrame3d.Rigid(Vector3d.One, rotation));
        Assert.Equal(Integer(new WidePlaneMetrics(plane, rotation).GetSpanSquared(a,b)), Integer(rigid.GetSpanSquared(a,b)));
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
        // Identity's exact rational basis has unit scale, so its metric normal
        // is the authored integer cross product without a quaternion scale.
        if (rotation == FixedQuaternion.Identity) return n;
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
