//=======================================================================
// CenteredFiniteSphereStrictOverlap.Tests.cs
//=======================================================================
// MIT License, Copyright (c) 2024-present David Oravsky (mrdav30)
// See LICENSE file in the project root for full license information.
//=======================================================================

using System;
using System.Numerics;
using FixedMathSharp.Geometry;
using Xunit;

namespace FixedMathSharp.Tests;

public sealed class CenteredFiniteSphereStrictOverlapTests
{
    [Fact]
    public void RimDistance_PreservesRadicalWhenRationalTermsCancelExactly()
    {
        // Q=8, r=1, axial gap=4, s=5: Q+r*r+gap*gap-s*s=0.
        // The remaining -2*r*sqrt(Q) term proves strict penetration.
        Assert.True(Cylinder(Vector3d.Zero, FixedQuaternion.Identity,
            Raw(2), Raw(1), Point(2, 5, 2), Raw(5)));
        Assert.True(Cone(Vector3d.Zero, FixedQuaternion.Identity,
            Raw(2), Raw(1), Point(2, -5, 2), Raw(5)));
    }

    [Fact]
    public void CylinderStrictOverlap_PreservesSubRawSidePenetration()
    {
        // sqrt(4^2 + 2^2) - 1 < 4 raw units. Rounding the surface
        // direction and the offset before taking its length loses this overlap.
        Assert.True(Cylinder(Vector3d.Zero, FixedQuaternion.Identity,
            Fixed64.Two, Raw(1), Point(4, 0, 2), Raw(4)));
    }

    [Fact]
    public void ConeStrictOverlap_PreservesSubRawSidePenetration()
    {
        // The meridian segment is (2,-3) -> (0,3). The distance from
        // (3,0) to its interior is 12/sqrt(40), strictly below 2 raw units.
        Assert.True(Cone(Vector3d.Zero, FixedQuaternion.Identity,
            Raw(6), Raw(2), Point(3, 0, 0), Raw(2)));
    }

    [Theory]
    [InlineData(3, 0, 0, 2, false)]
    [InlineData(3, 0, 0, 3, true)]
    [InlineData(0, 3, 0, 2, false)]
    [InlineData(0, 3, 0, 3, true)]
    [InlineData(4, 5, 0, 5, false)]
    [InlineData(4, 5, 0, 6, true)]
    [InlineData(0, 0, 0, 0, true)]
    [InlineData(1, 0, 0, 0, false)]
    [InlineData(0, 1, 0, 0, false)]
    public void CylinderStrictOverlap_SeparatesSideCapRimAndInterior(
        long x, long y, long z, long radius, bool expected)
    {
        Assert.Equal(expected, Cylinder(Vector3d.Zero, FixedQuaternion.Identity,
            Raw(2), Raw(1), Point(x, y, z), Raw(radius)));
    }

    [Theory]
    [InlineData(0, -5, 2, false)]
    [InlineData(0, -5, 3, true)]
    [InlineData(5, -7, 5, false)]
    [InlineData(5, -7, 6, true)]
    [InlineData(0, 5, 2, false)]
    [InlineData(0, 5, 3, true)]
    [InlineData(0, 0, 0, true)]
    [InlineData(1, 0, 0, false)]
    [InlineData(0, -3, 0, false)]
    [InlineData(0, 3, 0, false)]
    [InlineData(10, -2, 1, false)]
    public void ConeStrictOverlap_SeparatesBaseRimApexAndInterior(
        long x, long y, long radius, bool expected)
    {
        Assert.Equal(expected, Cone(Vector3d.Zero, FixedQuaternion.Identity,
            Raw(6), Raw(2), Point(x, y, 0), Raw(radius)));
    }

    [Fact]
    public void StrictOverlap_PreservesOddRawCapsAndZeroRadiusSegments()
    {
        // The centered length-one segment ends at y=1/2, not y=0.
        Assert.True(Cylinder(Vector3d.Zero, FixedQuaternion.Identity,
            Raw(1), Fixed64.Zero, Point(0, 1, 0), Raw(1)));
        Assert.True(Cone(Vector3d.Zero, FixedQuaternion.Identity,
            Raw(1), Fixed64.Zero, Point(0, 1, 0), Raw(1)));
        Assert.False(Cylinder(Vector3d.Zero, FixedQuaternion.Identity,
            Raw(1), Fixed64.Zero, Vector3d.Zero, Fixed64.Zero));
        Assert.False(Cone(Vector3d.Zero, FixedQuaternion.Identity,
            Raw(1), Fixed64.Zero, Vector3d.Zero, Fixed64.Zero));
    }

    [Fact]
    public void StrictOverlap_DoesNotSaturateFullDomainSeparation()
    {
        Vector3d center = new(Fixed64.MinValue, Fixed64.Zero, Fixed64.Zero);
        Vector3d point = new(Fixed64.MaxValue, Fixed64.Zero, Fixed64.Zero);
        Assert.False(Cylinder(center, FixedQuaternion.Identity,
            Fixed64.MaxValue, Fixed64.MaxValue, point, Fixed64.MaxValue));
        Assert.False(Cone(center, FixedQuaternion.Identity,
            Fixed64.MaxValue, Fixed64.MaxValue, point, Fixed64.MaxValue));
    }

    [Fact]
    public void StrictOverlap_PreservesTinyRimGapAfterFullDomainCancellation()
    {
        Vector3d center = new(Fixed64.MinValue, Fixed64.Zero, Fixed64.Zero);
        Vector3d point = Point(0, -(1L << 62), 0);
        // Both shapes share this base rim. The radial gap is exactly one raw
        // unit and the axial gap is half a raw unit: sqrt(5)/2 lies in (1,2).
        Assert.False(Cylinder(center, FixedQuaternion.Identity,
            Fixed64.MaxValue, Fixed64.MaxValue, point, Raw(1)));
        Assert.True(Cylinder(center, FixedQuaternion.Identity,
            Fixed64.MaxValue, Fixed64.MaxValue, point, Raw(2)));
        Assert.False(Cone(center, FixedQuaternion.Identity,
            Fixed64.MaxValue, Fixed64.MaxValue, point, Raw(1)));
        Assert.True(Cone(center, FixedQuaternion.Identity,
            Fixed64.MaxValue, Fixed64.MaxValue, point, Raw(2)));
    }

    [Fact]
    public void StrictOverlap_PreservesTangencyInAnArbitraryRigidFrame()
    {
        FixedQuaternion rotation = FixedQuaternion.FromAxisAngle(
            Vector3d.Forward, Fixed64.Pi / (Fixed64)7);
        // Rotation about Z preserves these radial Z points without rounding.
        Assert.False(Cylinder(Vector3d.Zero, rotation, Raw(4), Raw(3),
            Point(0, 0, 5), Raw(2)));
        Assert.True(Cylinder(Vector3d.Zero, rotation, Raw(4), Raw(3),
            Point(0, 0, 5), Raw(3)));
        // Cone height 4, radius 3 has a 3-4-5 side. At axial height 0,
        // radial 4 the side distance is (4*4 - 3*2)/5 = 2 exactly.
        Assert.False(Cone(Vector3d.Zero, rotation, Raw(4), Raw(3),
            Point(0, 0, 4), Raw(2)));
        Assert.True(Cone(Vector3d.Zero, rotation, Raw(4), Raw(3),
            Point(0, 0, 4), Raw(3)));
    }

    [Fact]
    public void StrictOverlap_MatchesIndependentUnboundedRigidFrameOracle()
    {
        FixedQuaternion rotation = FixedQuaternion.FromAxisAngle(
            new Vector3d(2, 3, 5).Normalized, Fixed64.Pi / (Fixed64)7);
        for (int x = -9; x <= 9; x += 3)
        for (int y = -9; y <= 9; y += 3)
        {
            AssertOracle(Vector3d.Zero, rotation, Raw(9), Raw(4),
                Point(x, y, 2), Raw(3));
        }

        // The differences must remain 65-bit, including the antipodal pair.
        long[] coordinates = { long.MinValue, -1L, 0L, 1L, long.MaxValue };
        foreach (long x in coordinates)
        foreach (long y in coordinates)
        {
            AssertOracle(Point(long.MinValue, y, long.MaxValue), rotation,
                Fixed64.MaxValue, Fixed64.MaxValue,
                Point(x, long.MaxValue, long.MinValue), Fixed64.MaxValue);
        }
    }

    [Fact]
    public void StrictOverlap_AllocatesNothingAfterWarmup()
    {
        FixedQuaternion rotation = FixedQuaternion.FromAxisAngle(
            new Vector3d(2, 3, 5).Normalized, Fixed64.Pi / (Fixed64)7);
        int overlaps = 0;
        Action query = () =>
        {
            for (int index = 0; index < 32; index++)
            {
                if (Cylinder(Vector3d.Zero, rotation, Raw(9), Raw(4),
                        Point(index % 9, index % 7, 2), Raw(3)))
                    overlaps++;
                if (Cone(Vector3d.Zero, rotation, Raw(9), Raw(4),
                        Point(index % 9, index % 7, 2), Raw(3)))
                    overlaps++;
            }
        };
        Assert.Equal(0L, FixedMathTestHelper.MeasureWarmedAllocations(query));
        Assert.True(overlaps > 0);
    }

    private static void AssertOracle(Vector3d center, FixedQuaternion rotation,
        Fixed64 height, Fixed64 radius, Vector3d sphereCenter, Fixed64 sphereRadius)
    {
        GetOraclePoint(center, rotation, sphereCenter,
            out BigInteger x, out BigInteger y, out BigInteger z, out BigInteger d);
        // Work in the common 2D-denominator scale: every coordinate, radius,
        // and half-height is an integer; no production wide arithmetic is used.
        BigInteger radialSquared = 4 * (x * x + z * z);
        y *= 2;
        BigInteger halfHeight = height.m_rawValue * d;
        BigInteger r = (BigInteger)2 * radius.m_rawValue * d;
        BigInteger s = (BigInteger)2 * sphereRadius.m_rawValue * d;
        BigInteger gap = BigInteger.Max(BigInteger.Abs(y) - halfHeight, 0);
        bool cylinderInside = BigInteger.Abs(y) < halfHeight && radialSquared < r * r;
        bool cylinder = cylinderInside || OracleDiskDistanceLess(radialSquared, r, gap, s);
        Assert.Equal(cylinder, Cylinder(center, rotation, height, radius, sphereCenter, sphereRadius));

        BigInteger fullHeight = 2 * halfHeight;
        BigInteger aboveBase = y + halfHeight;
        BigInteger belowApex = halfHeight - y;
        bool coneInside = aboveBase > 0 && belowApex > 0
            && fullHeight * fullHeight * radialSquared < r * r * belowApex * belowApex;
        bool cone = coneInside || OracleDiskDistanceLess(radialSquared, r, aboveBase, s);
        BigInteger sideSquared = r * r + fullHeight * fullHeight;
        BigInteger parameter = fullHeight * aboveBase + r * r;
        if (OracleRadicalSign(parameter, -r, radialSquared) > 0)
        {
            if (OracleRadicalSign(parameter - sideSquared, -r, radialSquared) >= 0)
            {
                cone |= radialSquared + belowApex * belowApex < s * s;
            }
            else
            {
                BigInteger offset = -r * belowApex;
                cone |= OracleRadicalSign(
                    fullHeight * fullHeight * radialSquared + offset * offset - s * s * sideSquared,
                    2 * fullHeight * offset, radialSquared) < 0;
            }
        }
        Assert.Equal(cone, Cone(center, rotation, height, radius, sphereCenter, sphereRadius));
    }

    private static bool OracleDiskDistanceLess(BigInteger q, BigInteger r,
        BigInteger axial, BigInteger s) => q <= r * r
        ? axial * axial < s * s
        : OracleRadicalSign(q + r * r + axial * axial - s * s, -2 * r, q) < 0;

    private static int OracleRadicalSign(BigInteger a, BigInteger b, BigInteger q)
    {
        int bSign = q.IsZero ? 0 : b.Sign;
        if (a.IsZero) return bSign;
        if (bSign == 0 || a.Sign == bSign) return a.Sign;
        int comparison = (a * a).CompareTo(b * b * q);
        return comparison == 0 ? 0 : comparison > 0 ? a.Sign : bSign;
    }

    private static void GetOraclePoint(Vector3d center, FixedQuaternion rotation,
        Vector3d point, out BigInteger localX, out BigInteger localY,
        out BigInteger localZ, out BigInteger denominator)
    {
        BigInteger x = rotation.X.m_rawValue;
        BigInteger y = rotation.Y.m_rawValue;
        BigInteger z = rotation.Z.m_rawValue;
        BigInteger w = rotation.W.m_rawValue;
        BigInteger dx = (BigInteger)point.X.m_rawValue - center.X.m_rawValue;
        BigInteger dy = (BigInteger)point.Y.m_rawValue - center.Y.m_rawValue;
        BigInteger dz = (BigInteger)point.Z.m_rawValue - center.Z.m_rawValue;
        denominator = x*x + y*y + z*z + w*w;
        localX = dx*(x*x-y*y-z*z+w*w) + dy*2*(x*y+z*w) + dz*2*(x*z-y*w);
        localY = dx*2*(x*y-z*w) + dy*(y*y-x*x-z*z+w*w) + dz*2*(y*z+x*w);
        localZ = dx*2*(x*z+y*w) + dy*2*(y*z-x*w) + dz*(z*z-x*x-y*y+w*w);
    }

    private static bool Cylinder(Vector3d center, FixedQuaternion rotation,
        Fixed64 height, Fixed64 radius, Vector3d sphereCenter, Fixed64 sphereRadius) =>
        WideFiniteAxisIntersection.DoesCenteredFiniteCylinderPenetrateSphere(
            center, rotation, height, radius, sphereCenter, sphereRadius);

    private static bool Cone(Vector3d center, FixedQuaternion rotation,
        Fixed64 height, Fixed64 radius, Vector3d sphereCenter, Fixed64 sphereRadius) =>
        WideFiniteAxisIntersection.DoesCenteredFiniteConePenetrateSphere(
            center, rotation, height, radius, sphereCenter, sphereRadius);

    private static Fixed64 Raw(long value) => Fixed64.FromRaw(value);

    private static Vector3d Point(long x, long y, long z) => new(Raw(x), Raw(y), Raw(z));
}
