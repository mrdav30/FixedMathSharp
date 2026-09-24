using System;
using System.Collections.Generic;
using System.Numerics;
using FixedMathSharp.Geometry;
using Xunit;

namespace FixedMathSharp.Tests;

public sealed class CenteredCylinderPolytopeStrictOverlapTests
{
    [Fact]
    public void CylinderBox_RejectsSeparationMissedByFiniteContactAxisSet()
    {
        Vector3d center = new(Fixed64.FromFraction(126, 100),
            Fixed64.FromFraction(-165, 100), Fixed64.FromFraction(-92, 100));
        Vector3d extents = new(Fixed64.FromFraction(83, 100),
            Fixed64.FromFraction(75, 100), Fixed64.FromFraction(52, 100));
        FixedQuaternion rotation = new FixedQuaternion((Fixed64)(-1), (Fixed64)(-9),
            (Fixed64)7, Fixed64.Zero).Normalized;

        // For the exact proportional quaternion and authored fractions, cap
        // clipping gives minimum radial squared 2530093/1922000 > 1. The
        // admitted fixed representations remain far from the separating rim.
        Assert.False(Box(Vector3d.Zero, FixedQuaternion.Identity, Fixed64.Two,
            Fixed64.One, center, rotation, extents));
    }

    [Fact]
    public void CylinderTriangle_FindsInteriorBeyondClosestPointOutsideCap()
    {
        var triangle = new FixedTriangle(new Vector3d(0, 7, -5),
            new Vector3d(10, 1, -5), new Vector3d(5, 4, 5));
        // Plane 3*x+5*y=35: closest to the center has y=175/34 > 5,
        // but (4,23/5,0) is in both the triangle and cylinder interior.
        Assert.True(Triangle(Vector3d.Zero, FixedQuaternion.Identity,
            (Fixed64)10, (Fixed64)5, triangle, Vector3d.Zero, FixedQuaternion.Identity));
    }

    [Theory]
    [InlineData(-1L, true)]
    [InlineData(0L, false)]
    [InlineData(1L, false)]
    public void CylinderTriangle_PreservesStrictCapBoundary(long rawStep, bool expected)
    {
        Fixed64 y = Fixed64.One + Fixed64.FromRaw(rawStep);
        var triangle = new FixedTriangle(new Vector3d((Fixed64)(-2), y, (Fixed64)(-2)),
            new Vector3d((Fixed64)2, y, (Fixed64)(-2)), new Vector3d(Fixed64.Zero, y, (Fixed64)2));
        Assert.Equal(expected, Triangle(Vector3d.Zero, FixedQuaternion.Identity,
            Fixed64.Two, Fixed64.One, triangle, Vector3d.Zero, FixedQuaternion.Identity));
    }

    [Theory]
    [InlineData(-1L, true)]
    [InlineData(0L, false)]
    [InlineData(1L, false)]
    public void CylinderTriangle_PreservesStrictSideBoundary(long rawStep, bool expected)
    {
        Fixed64 x = Fixed64.One + Fixed64.FromRaw(rawStep);
        var triangle = new FixedTriangle(new Vector3d(x, (Fixed64)(-2), (Fixed64)(-2)),
            new Vector3d(x, (Fixed64)2, (Fixed64)(-2)), new Vector3d(x, Fixed64.Zero, (Fixed64)2));
        Assert.Equal(expected, Triangle(Vector3d.Zero, FixedQuaternion.Identity,
            Fixed64.Two, Fixed64.One, triangle, Vector3d.Zero, FixedQuaternion.Identity));
    }

    [Fact]
    public void CylinderTriangle_PreservesSubRawInteriorAndOddHeight()
    {
        // Plane 2*x+z=9 has radial distance 9/sqrt(5) > 4; shifting
        // to 2*x+z=8 gives 8/sqrt(5) < 4, despite integer coordinates.
        var inside = new FixedTriangle(Point(5, -2, -2), Point(3, 2, 2), Point(4, 0, 0));
        var outside = new FixedTriangle(Point(5, -2, -1), Point(3, 2, 3), Point(4, 0, 1));
        Assert.True(Triangle(Vector3d.Zero, FixedQuaternion.Identity, Raw(3), Raw(4),
            inside, Vector3d.Zero, FixedQuaternion.Identity));
        Assert.False(Triangle(Vector3d.Zero, FixedQuaternion.Identity, Raw(3), Raw(4),
            outside, Vector3d.Zero, FixedQuaternion.Identity));
        // y=1 lies inside the exact [-1.5,1.5] slab, not on a rounded cap.
        var point = new FixedTriangle(Point(0, 1, 0), Point(0, 1, 0), Point(0, 1, 0));
        Assert.True(Triangle(Vector3d.Zero, FixedQuaternion.Identity, Raw(3), Raw(1),
            point, Vector3d.Zero, FixedQuaternion.Identity));
    }

    [Fact]
    public void CylinderTriangle_PreservesDegenerateSegmentAndPointMeaning()
    {
        Assert.True(Triangle(Vector3d.Zero, FixedQuaternion.Identity, Raw(4), Raw(1),
            new FixedTriangle(Point(-2, 0, 0), Point(2, 0, 0), Point(-2, 0, 0)),
            Vector3d.Zero, FixedQuaternion.Identity));
        Assert.False(Triangle(Vector3d.Zero, FixedQuaternion.Identity, Raw(4), Raw(1),
            new FixedTriangle(Point(1, 0, -2), Point(1, 0, 2), Point(1, 0, -2)),
            Vector3d.Zero, FixedQuaternion.Identity));
        foreach (long radial in new[] { 0L, 1L, 2L })
        {
            Vector3d p = Point(radial, 0, 0);
            Assert.Equal(radial == 0, Triangle(Vector3d.Zero, FixedQuaternion.Identity,
                Raw(4), Raw(1), new FixedTriangle(p, p, p), Vector3d.Zero, FixedQuaternion.Identity));
        }
    }

    [Fact]
    public void CylinderTriangle_HandlesCapCoincidencesAndReversedWinding()
    {
        FixedTriangle[] triangles =
        {
            // Entire edge on the cap, with the third vertex below it.
            new(Point(-4, 2, 0), Point(4, 2, 0), Point(0, 0, 5)),
            // One cap vertex, the other vertices on opposite sides.
            new(Point(0, 2, 0), Point(-4, 4, 3), Point(4, 0, 3)),
            // All vertices on the cap: closed clipping alone must not admit it.
            new(Point(-4, 2, 0), Point(4, 2, 0), Point(0, 2, 5)),
            // Projected area zero, but the axial/radial segment enters the solid.
            new(Point(-4, -4, 0), Point(4, 4, 0), Point(0, 4, 0))
        };
        bool[] expected = { true, true, false, true };
        for (int index = 0; index < triangles.Length; index++)
        {
            FixedTriangle t = triangles[index];
            Assert.Equal(expected[index], Triangle(Vector3d.Zero, FixedQuaternion.Identity,
                Raw(4), Raw(1), t, Vector3d.Zero, FixedQuaternion.Identity));
            Assert.Equal(expected[index], Triangle(Vector3d.Zero, FixedQuaternion.Identity,
                Raw(4), Raw(1), new FixedTriangle(t.C, t.B, t.A), Vector3d.Zero, FixedQuaternion.Identity));
        }
    }

    [Fact]
    public void CylinderTriangle_PreservesTangencyInCommonArbitraryFrame()
    {
        FixedQuaternion rotation = new FixedQuaternion((Fixed64)2, (Fixed64)3, (Fixed64)5, (Fixed64)7).Normalized;
        Vector3d origin = Point(long.MinValue, long.MaxValue, long.MinValue);
        foreach (long step in new[] { -1L, 0L, 1L })
        {
            var triangle = new FixedTriangle(Point(4+step, -4, -4),
                Point(4+step, 4, -4), Point(4+step, 0, 4));
            Assert.Equal(step < 0, Triangle(origin, rotation, Raw(8), Raw(4),
                triangle, origin, rotation));
        }
    }

    [Fact]
    public void CylinderHull_DistinguishesClosedEnclosureFromOpenSurfaces()
    {
        Vector3d[] vertices = CubeVertices(new Vector3d(8, 8, 8));
        int[] indices = CubeTriangles();
        Assert.True(Hull(Vector3d.Zero, FixedQuaternion.Identity, Fixed64.Two, Fixed64.One,
            Vector3d.Zero, FixedQuaternion.Identity, vertices, indices, true));
        Assert.False(Hull(Vector3d.Zero, FixedQuaternion.Identity, Fixed64.Two, Fixed64.One,
            Vector3d.Zero, FixedQuaternion.Identity, vertices, indices, false));
        // The single distant open face must never manufacture enclosure.
        Assert.False(Hull(Vector3d.Zero, FixedQuaternion.Identity, Fixed64.Two, Fixed64.One,
            Vector3d.Zero, FixedQuaternion.Identity, vertices, new[] { 0, 1, 3, 0, 3, 2 }, false));
        Array.Reverse(indices);
        Assert.True(Hull(Vector3d.Zero, FixedQuaternion.Identity, Fixed64.Two, Fixed64.One,
            Vector3d.Zero, FixedQuaternion.Identity, vertices, indices, true));
    }

    [Fact]
    public void CylinderBoxAndHull_PreserveBoundaryAndBothEnclosures()
    {
        Vector3d extents = new(1, 1, 1);
        Vector3d[] vertices = CubeVertices(extents);
        int[] indices = CubeTriangles();
        foreach (long step in new[] { -1L, 0L, 1L })
        {
            Vector3d origin = new(Fixed64.Two + Raw(step), Fixed64.Zero, Fixed64.Zero);
            bool expected = step < 0;
            Assert.Equal(expected, Box(Vector3d.Zero, FixedQuaternion.Identity, Fixed64.Two,
                Fixed64.One, origin, FixedQuaternion.Identity, extents));
            Assert.Equal(expected, Hull(Vector3d.Zero, FixedQuaternion.Identity, Fixed64.Two,
                Fixed64.One, origin, FixedQuaternion.Identity, vertices, indices, true));
        }
        Assert.True(Box(Vector3d.Zero, FixedQuaternion.Identity, Fixed64.One,
            Fixed64.One, Vector3d.Zero, FixedQuaternion.Identity, new Vector3d(4, 4, 4)));
        Assert.True(Box(Vector3d.Zero, FixedQuaternion.Identity, (Fixed64)10,
            (Fixed64)10, Vector3d.Zero, FixedQuaternion.Identity, extents));
        Assert.True(Hull(Vector3d.Zero, FixedQuaternion.Identity, (Fixed64)10,
            (Fixed64)10, Vector3d.Zero, FixedQuaternion.Identity, vertices, indices, true));
    }

    [Fact]
    public void CylinderTriangle_MatchesIndependentClippedPolygonRationalOracle()
    {
        FixedQuaternion cylinderRotation = new FixedQuaternion((Fixed64)2, (Fixed64)3, (Fixed64)5, (Fixed64)7).Normalized;
        FixedQuaternion shapeRotation = new FixedQuaternion((Fixed64)(-3), (Fixed64)5, (Fixed64)2, Fixed64.One).Normalized;
        uint state = 127U;
        for (int index = 0; index < 128; index++)
        {
            Vector3d NextPoint() => Point(NextRaw(), NextRaw(), NextRaw());
            long NextRaw()
            {
                state = unchecked(state * 1664525U + 1013904223U);
                return (long)(state % 41U) - 20;
            }
            var triangle = new FixedTriangle(NextPoint(), NextPoint(), NextPoint());
            AssertOracle(NextPoint(), cylinderRotation, Raw(13), Raw(7), triangle,
                NextPoint(), shapeRotation);
        }

        long[] limits = { long.MinValue, -1L, 0L, 1L, long.MaxValue };
        foreach (long coordinate in limits)
        foreach (long translation in limits)
        {
            AssertOracle(Point(long.MinValue, translation, long.MaxValue), cylinderRotation,
                Fixed64.MaxValue, Fixed64.MaxValue,
                new FixedTriangle(Point(coordinate, long.MaxValue, long.MinValue),
                    Point(long.MaxValue, long.MinValue, coordinate), Point(long.MinValue, coordinate, long.MaxValue)),
                Point(translation, long.MaxValue, long.MinValue), shapeRotation);
        }
    }

    [Fact]
    public void CylinderPolytope_AllocatesNothingAfterWarmup()
    {
        FixedQuaternion rotation = new FixedQuaternion((Fixed64)2, (Fixed64)3, (Fixed64)5, (Fixed64)7).Normalized;
        Vector3d[] vertices = CubeVertices(new Vector3d(2, 2, 2));
        int[] indices = CubeTriangles();
        var triangle = new FixedTriangle(new Vector3d(0, 7, -5),
            new Vector3d(10, 1, -5), new Vector3d(5, 4, 5));
        int overlaps = 0;
        Action queries = () =>
        {
            for (int index = 0; index < 8; index++)
            {
                if (Box(new Vector3d(3, 0, 0), rotation, (Fixed64)4, Fixed64.One,
                    Vector3d.Zero, rotation, new Vector3d(2, 2, 2))) overlaps++;
                if (Hull(new Vector3d(3, 0, 0), rotation, (Fixed64)4, Fixed64.One,
                    Vector3d.Zero, rotation, vertices, indices, true)) overlaps++;
                if (Triangle(Vector3d.Zero, FixedQuaternion.Identity, (Fixed64)10, (Fixed64)5,
                    triangle, Vector3d.Zero, FixedQuaternion.Identity)) overlaps++;
            }
        };
        Assert.Equal(0L, FixedMathTestHelper.MeasureWarmedAllocations(queries));
        Assert.True(overlaps > 0);
    }

    private static Vector3d[] CubeVertices(Vector3d extents)
    {
        var vertices = new Vector3d[8];
        for (int index = 0; index < vertices.Length; index++)
            vertices[index] = new Vector3d((index & 1) == 0 ? -extents.X : extents.X,
                (index & 2) == 0 ? -extents.Y : extents.Y, (index & 4) == 0 ? -extents.Z : extents.Z);
        return vertices;
    }

    private static int[] CubeTriangles() => new[]
    {
        0, 1, 3, 0, 3, 2, 4, 6, 7, 4, 7, 5,
        0, 4, 5, 0, 5, 1, 2, 3, 7, 2, 7, 6,
        0, 2, 6, 0, 6, 4, 1, 5, 7, 1, 7, 3
    };

    // Test-only arbitrary-precision rational oracle deliberately uses general
    // polygon clipping, unlike production's bounded original-edge/cap features.
    private readonly struct Rational
    {
        internal readonly BigInteger N;
        internal readonly BigInteger D;
        internal Rational(BigInteger n, BigInteger d)
        {
            if (d.Sign < 0) { n = -n; d = -d; }
            BigInteger gcd = BigInteger.GreatestCommonDivisor(n, d);
            N = n / gcd;
            D = d / gcd;
        }
        internal int Sign => N.Sign;
        public static implicit operator Rational(long n) => new(n, BigInteger.One);
        public static Rational operator +(Rational a, Rational b) => new(a.N*b.D+b.N*a.D, a.D*b.D);
        public static Rational operator -(Rational a, Rational b) => new(a.N*b.D-b.N*a.D, a.D*b.D);
        public static Rational operator *(Rational a, Rational b) => new(a.N*b.N, a.D*b.D);
        public static Rational operator /(Rational a, Rational b) => new(a.N*b.D, a.D*b.N);
    }

    private readonly record struct OraclePoint(Rational X, Rational Y, Rational Z)
    {
        internal static OraclePoint From(Vector3d value) => new(value.X.m_rawValue, value.Y.m_rawValue, value.Z.m_rawValue);
        public static OraclePoint operator +(OraclePoint a, OraclePoint b) => new(a.X+b.X, a.Y+b.Y, a.Z+b.Z);
        public static OraclePoint operator -(OraclePoint a, OraclePoint b) => new(a.X-b.X, a.Y-b.Y, a.Z-b.Z);
        public static OraclePoint operator *(OraclePoint a, Rational t) => new(a.X*t, a.Y*t, a.Z*t);
    }

    private static OraclePoint OracleRotate(OraclePoint p, FixedQuaternion q, bool inverse)
    {
        BigInteger x = q.X.m_rawValue, y = q.Y.m_rawValue, z = q.Z.m_rawValue, w = q.W.m_rawValue;
        if (inverse) { x = -x; y = -y; z = -z; }
        BigInteger d = x*x+y*y+z*z+w*w;
        return new OraclePoint(
            p.X*new Rational(x*x-y*y-z*z+w*w,d)+p.Y*new Rational(2*(x*y-z*w),d)+p.Z*new Rational(2*(x*z+y*w),d),
            p.X*new Rational(2*(x*y+z*w),d)+p.Y*new Rational(y*y-x*x-z*z+w*w,d)+p.Z*new Rational(2*(y*z-x*w),d),
            p.X*new Rational(2*(x*z-y*w),d)+p.Y*new Rational(2*(y*z+x*w),d)+p.Z*new Rational(z*z-x*x-y*y+w*w,d));
    }

    private static void AssertOracle(Vector3d center, FixedQuaternion rotation, Fixed64 height,
        Fixed64 radius, FixedTriangle triangle, Vector3d origin, FixedQuaternion shapeRotation)
    {
        OraclePoint Transform(Vector3d p) => OracleRotate(OracleRotate(OraclePoint.From(p), shapeRotation, false)
            + OraclePoint.From(origin) - OraclePoint.From(center), rotation, true);
        var points = new List<OraclePoint> { Transform(triangle.A), Transform(triangle.B), Transform(triangle.C) };
        Rational halfHeight = new(height.m_rawValue, 2);
        bool axialInterior = points.Exists(p => (p.Y + halfHeight).Sign > 0)
            && points.Exists(p => (p.Y - halfHeight).Sign < 0);
        points = Clip(Clip(points, halfHeight, true), (Rational)0-halfHeight, false);
        bool expected = false;
        if (axialInterior && points.Count > 0)
        {
            Rational radiusSquared = (Rational)radius.m_rawValue * radius.m_rawValue;
            int orientation = 0;
            bool originInside = true;
            for (int index = 0; index < points.Count; index++)
            {
                OraclePoint a = points[index], b = points[(index+1)%points.Count];
                OraclePoint edge = b-a;
                Rational q = edge.X*edge.X+edge.Z*edge.Z;
                Rational parameter = q.Sign == 0 ? 0 : ((Rational)0-a.X*edge.X-a.Z*edge.Z)/q;
                if (parameter.Sign < 0) parameter = 0;
                if ((parameter-(Rational)1).Sign > 0) parameter = 1;
                OraclePoint closest = a+edge*parameter;
                expected |= (closest.X*closest.X+closest.Z*closest.Z-radiusSquared).Sign < 0;
                int cross = (a.X*b.Z-a.Z*b.X).Sign;
                if (cross != 0)
                {
                    if (orientation != 0 && orientation != cross) originInside = false;
                    orientation = cross;
                }
            }
            expected |= originInside && orientation != 0;
        }
        bool actual = Triangle(center, rotation, height, radius, triangle, origin, shapeRotation);
        Assert.True(expected == actual, $"Expected {expected}, actual {actual}; center={center}; origin={origin}; triangle={triangle}");
    }

    private static List<OraclePoint> Clip(List<OraclePoint> points, Rational plane, bool upper)
    {
        var result = new List<OraclePoint>();
        for (int index = 0; index < points.Count; index++)
        {
            OraclePoint a = points[index], b = points[(index+1)%points.Count];
            bool aInside = upper ? (a.Y-plane).Sign <= 0 : (a.Y-plane).Sign >= 0;
            bool bInside = upper ? (b.Y-plane).Sign <= 0 : (b.Y-plane).Sign >= 0;
            if (aInside) result.Add(a);
            if (aInside != bInside) result.Add(a+(b-a)*((plane-a.Y)/(b.Y-a.Y)));
        }
        return result;
    }

    private static Fixed64 Raw(long value) => Fixed64.FromRaw(value);
    private static Vector3d Point(long x, long y, long z) => new(Raw(x), Raw(y), Raw(z));
    private static bool Hull(Vector3d center, FixedQuaternion rotation, Fixed64 height, Fixed64 radius,
        Vector3d origin, FixedQuaternion shapeRotation, Vector3d[] points, int[] indices, bool closed) =>
        WideOrientedBox.DoesCenteredCylinderPenetrateConvexHull(center, rotation, height, radius,
            origin, shapeRotation, points, indices, closed);

    private static bool Box(Vector3d cylinderCenter, FixedQuaternion cylinderRotation,
        Fixed64 height, Fixed64 radius, Vector3d boxCenter, FixedQuaternion boxRotation,
        Vector3d halfExtents) => WideOrientedBox.DoesCenteredCylinderPenetrateBox(
            cylinderCenter, cylinderRotation, height, radius, boxCenter, boxRotation, halfExtents);

    private static bool Triangle(Vector3d cylinderCenter, FixedQuaternion cylinderRotation,
        Fixed64 height, Fixed64 radius, FixedTriangle triangle, Vector3d triangleOrigin,
        FixedQuaternion triangleRotation)
        => WideOrientedBox.DoesCenteredCylinderPenetrateTriangle(cylinderCenter, cylinderRotation,
            height, radius, triangle, triangleOrigin, triangleRotation);
}
