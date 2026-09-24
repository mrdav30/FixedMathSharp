using System;
using System.Numerics;
using FixedMathSharp.Geometry;
using Xunit;

namespace FixedMathSharp.Tests;

public sealed class CenteredFiniteCapsuleStrictOverlapTests
{
    [Theory]
    [InlineData(1, -4, 4, 0, 0, false)] // (2t-1)^2
    [InlineData(1, -8, 24, -32, 16, false)] // (2t-1)^4
    [InlineData(0, -1, 1, 0, 0, true)] // t(t-1)
    [InlineData(0, 0, 1, -2, 1, false)] // t^2(t-1)^2
    [InlineData(4, -65, 375, -875, 625, true)] // (5t-1)^3(5t-4)
    [InlineData(16, -200, 825, -1250, 625, false)] // (5t-1)^2(5t-4)^2
    [InlineData(12, -155, 675, -1125, 625, true)] // (5t-1)^2(5t-3)(5t-4)
    [InlineData(6, -29, 13, 32, -16, true)] // negative leading coefficient
    [InlineData(1, -4, 0, 0, 4, true)] // quartic/derivative has a linear remainder; P(1/2)<0
    [InlineData(3, -8, 0, 0, 16, false)] // (2t-1)^2(4t^2+4t+3)
    [InlineData(1, 1, 0, 0, 1, false)] // negative-leading linear Sturm divisor
    [InlineData(1, 0, 0, 0, 1, false)] // constant second Sturm remainder
    [InlineData(10, 4, 2, 0, 1, false)] // constant third Sturm remainder
    [InlineData(0, 0, 0, 0, 0, false)]
    [InlineData(-1, 0, 0, 0, 0, true)]
    [InlineData(1, 0, 0, 0, 0, false)]
    public void StrictPolynomial_DistinguishesCrossingsAndRepeatedTangency(
        long c0, long c1, long c2, long c3, long c4, bool expected)
    {
        Assert.Equal(expected, HasNegative(c0, c1, c2, c3, c4));
    }

    [Fact]
    public void StrictPolynomial_DetectsNegativeIntervalMuchNarrowerThanOneRawUnit()
    {
        BigInteger square = BigInteger.One << 800;
        // (2^400*(2t-1))^2 - 1 is negative only within 2^-401 of 1/2.
        Assert.True(HasNegative(square - 1, -4 * square, 4 * square, 0, 0));
        Assert.False(HasNegative(square, -4 * square, 4 * square, 0, 0));
    }

    [Fact]
    public void StrictPolynomial_MatchesIndependentFactoredRootSigns()
    {
        // Enumerating known rational roots covers every quartic multiplicity
        // partition. Expectations use factor signs, not polynomial/root helpers.
        int[] roots = { -1, 0, 1, 2, 3, 4, 5 };
        for (int a = 0; a < roots.Length; a++)
        for (int b = a; b < roots.Length; b++)
        for (int c = b; c < roots.Length; c++)
        for (int d = c; d < roots.Length; d++)
        foreach (int leading in new[] { -1, 1 })
        {
            int[] factors = { roots[a], roots[b], roots[c], roots[d] };
            BigInteger[] coefficients = { leading, 0, 0, 0, 0 };
            int degree = 0;
            foreach (int root in factors)
            {
                for (int i = degree + 1; i >= 0; i--)
                    coefficients[i] = (i > 0 ? 4 * coefficients[i - 1] : 0)
                        - (i <= degree ? root * coefficients[i] : 0);
                degree++;
            }
            bool expected = false;
            for (int sample = 0; sample <= 8; sample++)
            {
                int sign = leading;
                foreach (int root in factors)
                    sign *= Math.Sign(sample - 2 * root);
                expected |= sign < 0;
            }
            Assert.Equal(expected, HasNegative(coefficients));
        }
        BigInteger k = BigInteger.One << 240;
        Assert.False(HasNegative(4, -12*k, 13*k*k, -6*k*k*k, k*k*k*k));
    }

    [Theory]
    [InlineData(-1L, true)]
    [InlineData(0L, false)]
    [InlineData(1L, false)]
    public void CylinderCapsule_ClassifiesRimBoundaryBeforeDepthRounding(long rawStep, bool expected)
    {
        var capsuleCenter = new Vector3d((Fixed64)83 / 4,
            Fixed64.Two + Fixed64.FromRaw(rawStep), Fixed64.Zero);
        Assert.Equal(expected, Cylinder(capsuleCenter, Vector3d.Right, (Fixed64)20, (Fixed64)5 / 4));
    }

    [Fact]
    public void CylinderCapsule_RejectsKnownMissingRimAxisFalseOverlap()
    {
        Assert.False(Cylinder(new Vector3d((Fixed64)83 / 4, (Fixed64)7 / 4, Fixed64.Zero),
            Vector3d.Right, (Fixed64)20, Fixed64.One));
    }

    [Theory]
    [InlineData(-1L, true)]
    [InlineData(0L, false)]
    [InlineData(1L, false)]
    public void CylinderCapsule_ClassifiesInteriorCoreToRimMinimum(long rawStep, bool expected)
    {
        Assert.Equal(expected, Cylinder(new Vector3d((Fixed64)43 / 4,
            Fixed64.Two + Fixed64.FromRaw(rawStep), Fixed64.Zero),
            Vector3d.Forward, (Fixed64)20, (Fixed64)5 / 4));
    }

    [Theory]
    [InlineData(-1L, true)]
    [InlineData(0L, false)]
    [InlineData(1L, false)]
    public void ConeCapsule_ClassifiesBaseRimBoundary(long rawStep, bool expected)
    {
        // Moving farther below the base increases the 3-4-5 rim gap.
        var capsuleCenter = new Vector3d((Fixed64)83 / 4,
            -Fixed64.Two - Fixed64.FromRaw(rawStep), Fixed64.Zero);
        Assert.Equal(expected, WideFiniteAxisIntersection.DoesCenteredFiniteConePenetrateCapsule(
            Vector3d.Zero, FixedQuaternion.Identity, Fixed64.Two, (Fixed64)10,
            capsuleCenter, FixedQuaternion.Identity, Vector3d.Right, (Fixed64)20, (Fixed64)5 / 4));
    }

    [Theory]
    [InlineData(-1L, true)]
    [InlineData(0L, false)]
    [InlineData(1L, false)]
    public void ConeCapsule_ClassifiesInteriorSideMinimum(long rawStep, bool expected)
    {
        // The 3-4-5 cone side is exactly two units from (4,0,0).
        Assert.Equal(expected, WideFiniteAxisIntersection.DoesCenteredFiniteConePenetrateCapsule(
            Vector3d.Zero, FixedQuaternion.Identity, (Fixed64)4, (Fixed64)3,
            new Vector3d((Fixed64)4 + Fixed64.FromRaw(rawStep), Fixed64.Zero, Fixed64.Zero),
            FixedQuaternion.Identity, Vector3d.Forward, Fixed64.Two, Fixed64.Two));
    }

    [Theory]
    [InlineData(0L, false)]
    [InlineData(1L, false)]
    [InlineData(2L, true)]
    public void CylinderCapsule_PreservesFullDomainRadiusCancellation(long inward, bool expected)
    {
        Assert.Equal(expected, WideFiniteAxisIntersection.DoesCenteredFiniteCylinderPenetrateCapsule(
            new Vector3d(Fixed64.MinValue, Fixed64.Zero, Fixed64.Zero), FixedQuaternion.Identity,
            Fixed64.MaxValue, Fixed64.MaxValue,
            new Vector3d(Fixed64.MaxValue - Fixed64.FromRaw(inward), Fixed64.Zero, Fixed64.Zero),
            FixedQuaternion.Identity, Vector3d.Up, Fixed64.MaxValue, Fixed64.MaxValue));
    }

    [Fact]
    public void CenteredCapsule_PreservesOddRawEndpointPastCap()
    {
        Assert.True(WideFiniteAxisIntersection.DoesCenteredFiniteCylinderPenetrateCapsule(
            Vector3d.Zero, FixedQuaternion.Identity, Fixed64.FromRaw(1), Fixed64.One,
            new Vector3d(Fixed64.Zero, Fixed64.FromRaw(1), Fixed64.Zero), FixedQuaternion.Identity,
            Vector3d.Up, Fixed64.FromRaw(1), Fixed64.FromRaw(1)));
    }

    [Theory]
    [InlineData(0L, false)]
    [InlineData(1L, true)]
    public void ZeroRadiusCore_RequiresPositiveEntryPastCapOrApex(long extraLength, bool expected)
    {
        Fixed64 length = Fixed64.Two + Fixed64.FromRaw(extraLength);
        Assert.Equal(expected, WideFiniteAxisIntersection.DoesCenteredFiniteCylinderPenetrateCapsule(
            Vector3d.Zero, FixedQuaternion.Identity, Fixed64.Two, Fixed64.One,
            new Vector3d(0, 2, 0), FixedQuaternion.Identity, Vector3d.Up, length, Fixed64.Zero));
        Assert.Equal(expected, WideFiniteAxisIntersection.DoesCenteredFiniteConePenetrateCapsule(
            Vector3d.Zero, FixedQuaternion.Identity, Fixed64.Two, Fixed64.One,
            new Vector3d(0, 2, 0), FixedQuaternion.Identity, Vector3d.Up, length, Fixed64.Zero));
        Assert.Equal(expected, WideFiniteAxisIntersection.DoesCenteredFiniteConePenetrateCapsule(
            Vector3d.Zero, FixedQuaternion.Identity, Fixed64.Two, Fixed64.One,
            new Vector3d(0, -2, 0), FixedQuaternion.Identity, Vector3d.Up, length, Fixed64.Zero));
    }

    [Theory]
    [InlineData(6, false)]
    [InlineData(8, true)]
    public void ZeroRadiusSolids_UseExactRigidCoreDistance(int length, bool expected)
    {
        Assert.Equal(expected, WideFiniteAxisIntersection.DoesCenteredFiniteCylinderPenetrateCapsule(
            Vector3d.Zero, FixedQuaternion.Identity, Fixed64.Two, Fixed64.Zero,
            new Vector3d(4, 0, 0), FixedQuaternion.Identity, Vector3d.Right, (Fixed64)length, Fixed64.One));
        Assert.Equal(expected, WideFiniteAxisIntersection.DoesCenteredFiniteConePenetrateCapsule(
            Vector3d.Zero, FixedQuaternion.Identity, Fixed64.Two, Fixed64.Zero,
            new Vector3d(4, 0, 0), FixedQuaternion.Identity, Vector3d.Right, (Fixed64)length, Fixed64.One));
    }

    [Fact]
    public void CapsuleCore_EntersExpandedCapDiskWithoutCenterSphereEntry()
    {
        // The center is outside the rounded cylinder, but the left core end
        // (5,3,0) is two units above the interior of the upper disk.
        Assert.True(Cylinder(new Vector3d(20, 3, 0), Vector3d.Right, (Fixed64)30, (Fixed64)3));
        // Here rho is always at least 3/2, outside the radius-one disk core.
        // At z=0, N=rho^2+axial^2+R^2-s^2=-7/2, so the full ring lies
        // inside the radius-three expansion without squaring away its sign.
        Assert.True(WideFiniteAxisIntersection.DoesCenteredFiniteCylinderPenetrateCapsule(
            Vector3d.Zero, FixedQuaternion.Identity, Fixed64.Two, Fixed64.One,
            new Vector3d((Fixed64)3/2, (Fixed64)5/2, (Fixed64)4), FixedQuaternion.Identity,
            Vector3d.Forward, (Fixed64)12, (Fixed64)3));
    }

    [Fact]
    public void CapsuleCore_EntersApexSphereAwayFromItsCenter()
    {
        // The left core end (0,3,0) is one unit above the apex; the core
        // midpoint is outside, and the base disk is five units below it.
        Assert.True(WideFiniteAxisIntersection.DoesCenteredFiniteConePenetrateCapsule(
            Vector3d.Zero, FixedQuaternion.Identity, (Fixed64)4, (Fixed64)3,
            new Vector3d(4, 3, 0), FixedQuaternion.Identity, Vector3d.Right, (Fixed64)8, Fixed64.Two));
    }

    [Theory]
    [InlineData(-1L, true)]
    [InlineData(0L, false)]
    [InlineData(1L, false)]
    public void CylinderCapsule_PreservesRimSignUnderRigidQuarterTurn(long rawStep, bool expected)
    {
        var rotation = new FixedQuaternion(Fixed64.Zero, Fixed64.Zero, Fixed64.One, Fixed64.One).Normalized;
        var point = new Vector3d(-Fixed64.Two-Fixed64.FromRaw(rawStep), (Fixed64)83/4, Fixed64.Zero);
        Assert.Equal(expected, WideFiniteAxisIntersection.DoesCenteredFiniteCylinderPenetrateCapsule(
            Vector3d.Zero, rotation, Fixed64.Two, (Fixed64)10, point, rotation,
            Vector3d.Right, (Fixed64)20, (Fixed64)5/4));
        Assert.Equal(expected, WideFiniteAxisIntersection.DoesCenteredFiniteCylinderPenetrateCapsule(
            Vector3d.Zero, rotation, Fixed64.Two, (Fixed64)10, point, rotation,
            -Vector3d.Right, (Fixed64)20, (Fixed64)5/4));
    }

    [Fact]
    public void StrictCapsuleQueries_AllocateNothingAfterWarmup()
    {
        int penetrations = 0;
        Action queries = () =>
        {
            if (Cylinder(new Vector3d((Fixed64)43/4, Fixed64.Two, Fixed64.Zero),
                    Vector3d.Forward, (Fixed64)20, (Fixed64)5/4))
                penetrations++;
            if (WideFiniteAxisIntersection.DoesCenteredFiniteConePenetrateCapsule(
                    Vector3d.Zero, FixedQuaternion.Identity, (Fixed64)4, (Fixed64)3,
                    new Vector3d(4, 0, 0), FixedQuaternion.Identity,
                    Vector3d.Forward, Fixed64.Two, Fixed64.Two))
                penetrations++;
        };
        Assert.Equal(0L, FixedMathTestHelper.MeasureWarmedAllocations(queries));
        Assert.Equal(0, penetrations);
    }

    [Fact]
    public void ParallelCapsules_MatchIndependentUnboundedRigidFrameOracles()
    {
        var rotation = new FixedQuaternion((Fixed64)1, (Fixed64)(-2), (Fixed64)3, (Fixed64)4).Normalized;
        Vector3d[] origins = { Vector3d.Zero, new(Fixed64.MinValue, Fixed64.MaxValue, Fixed64.MinValue) };
        Vector3d[] points = { Vector3d.Zero, new(1, 2, 3), new(20, 30, 40),
            new(Fixed64.MaxValue, Fixed64.MinValue, Fixed64.MaxValue),
            new(Fixed64.MinValue, Fixed64.MaxValue, Fixed64.MinValue) };
        Fixed64[] heights = { Fixed64.FromRaw(1), Fixed64.Two, Fixed64.MaxValue };
        Fixed64[] radii = { Fixed64.FromRaw(1), (Fixed64)10, Fixed64.MaxValue };
        Fixed64[] lengths = { Fixed64.FromRaw(1), (Fixed64)100, Fixed64.MaxValue };
        Fixed64[] expansions = { Fixed64.Zero, Fixed64.FromRaw(1), Fixed64.Two, Fixed64.MaxValue };
        foreach (Vector3d origin in origins)
        foreach (Vector3d point in points)
        for (int size = 0; size < heights.Length; size++)
        foreach (Fixed64 expansion in expansions)
        {
            GetLocalPoint(origin, point, rotation, out BigInteger x, out BigInteger y,
                out BigInteger z, out BigInteger denominator);
            BigInteger q = 4 * (x*x + z*z);
            BigInteger r = 2 * (BigInteger)radii[size].m_rawValue * denominator;
            BigInteger s = 2 * (BigInteger)expansion.m_rawValue * denominator;
            BigInteger halfHeight = heights[size].m_rawValue * denominator;
            BigInteger halfLength = lengths[size].m_rawValue * denominator;
            BigInteger lower = 2*y - halfLength, upper = 2*y + halfLength;
            BigInteger gap = BigInteger.Max(BigInteger.Abs(2*y) - halfHeight - halfLength, 0);
            bool cylinder = (BigInteger.Abs(2*y) < halfHeight + halfLength && q < r*r)
                || DiskDistanceLess(q, r, gap, s);
            Assert.Equal(cylinder, WideFiniteAxisIntersection.DoesCenteredFiniteCylinderPenetrateCapsule(
                origin, rotation, heights[size], radii[size], point, rotation,
                Vector3d.Up, lengths[size], expansion));

            // With constant rho, distance to a cone is minimized by the core
            // point nearest its base plane. Interior crossing is separate for s=0.
            BigInteger nearestY = BigInteger.Min(BigInteger.Max(-halfHeight, lower), upper);
            bool coneInterior = upper > -halfHeight && lower < halfHeight
                && (lower <= -halfHeight ? q < r*r
                    : 4*halfHeight*halfHeight*q < r*r*(halfHeight-lower)*(halfHeight-lower));
            bool cone = coneInterior || ConePointDistanceLess(q, nearestY, halfHeight, r, s);
            Assert.Equal(cone, WideFiniteAxisIntersection.DoesCenteredFiniteConePenetrateCapsule(
                origin, rotation, heights[size], radii[size], point, rotation,
                Vector3d.Up, lengths[size], expansion));
        }
    }

    private static bool ConePointDistanceLess(BigInteger q, BigInteger y, BigInteger halfHeight,
        BigInteger r, BigInteger s)
    {
        BigInteger h = 2*halfHeight, aboveBase = y+halfHeight, belowApex = halfHeight-y;
        if (DiskDistanceLess(q, r, aboveBase, s)) return true;
        BigInteger sideSquared = h*h+r*r;
        BigInteger parameter = h*aboveBase+r*r;
        if (RadicalSign(parameter, -r, q) <= 0) return false;
        if (RadicalSign(parameter-sideSquared, -r, q) >= 0)
            return q+belowApex*belowApex < s*s;
        BigInteger offset = -r*belowApex;
        return RadicalSign(h*h*q+offset*offset-s*s*sideSquared, 2*h*offset, q) < 0;
    }

    [Fact]
    public void ObliqueZeroRadiusCores_MatchIndependentUnboundedRigidFrameOracle()
    {
        var bodyRotation = new FixedQuaternion((Fixed64)1, (Fixed64)2, (Fixed64)3, (Fixed64)4).Normalized;
        var capsuleRotation = new FixedQuaternion((Fixed64)3, (Fixed64)1, (Fixed64)(-1), (Fixed64)2).Normalized;
        Vector3d[] origins = { Vector3d.Zero, new(Fixed64.MinValue, Fixed64.MaxValue, Fixed64.MinValue) };
        Vector3d[] centers = { Vector3d.Zero, new(1, 2, 3), new(20, 30, 40),
            new(Fixed64.MaxValue, Fixed64.MinValue, Fixed64.MaxValue),
            new(Fixed64.MinValue, Fixed64.MaxValue, Fixed64.MinValue) };
        Fixed64[] heights = { Fixed64.FromRaw(1), (Fixed64)5, Fixed64.MaxValue };
        Fixed64[] radii = { Fixed64.FromRaw(1), (Fixed64)3, Fixed64.MaxValue };
        Fixed64[] lengths = { Fixed64.FromRaw(1), (Fixed64)101, Fixed64.MaxValue };
        foreach (Vector3d origin in origins)
        foreach (Vector3d center in centers)
        for (int size = 0; size < heights.Length; size++)
        foreach (int direction in new[] { -1, 1 })
        {
            GetLocalCore(origin, bodyRotation, center, capsuleRotation, lengths[size],
                out var start, out var end, out BigInteger denominator);
            Assert.NotEqual(start.X, end.X);
            Assert.NotEqual(start.Y, end.Y);
            Assert.NotEqual(start.Z, end.Z);
            bool cylinder = CoreEntersSolid(start, end, denominator, heights[size], radii[size], false);
            bool cone = CoreEntersSolid(start, end, denominator, heights[size], radii[size], true);
            Vector3d axis = direction > 0 ? Vector3d.Up : -Vector3d.Up;
            Assert.Equal(cylinder, WideFiniteAxisIntersection.DoesCenteredFiniteCylinderPenetrateCapsule(
                origin, bodyRotation, heights[size], radii[size], center, capsuleRotation,
                axis, lengths[size], Fixed64.Zero));
            Assert.Equal(cone, WideFiniteAxisIntersection.DoesCenteredFiniteConePenetrateCapsule(
                origin, bodyRotation, heights[size], radii[size], center, capsuleRotation,
                axis, lengths[size], Fixed64.Zero));
        }
    }

    [Fact]
    public void ObliqueConeCapsule_EntersIrrationalOffsetSideOnlyAtInteriorVertex()
    {
        var bodyRotation = new FixedQuaternion((Fixed64)1, (Fixed64)2, (Fixed64)3, (Fixed64)4).Normalized;
        var capsuleRotation = new FixedQuaternion((Fixed64)3, (Fixed64)1, (Fixed64)(-1), (Fixed64)2).Normalized;
        var center = new Vector3d((Fixed64)11/4, Fixed64.Half, Fixed64.Two);
        Fixed64 height = (Fixed64)5, radius = (Fixed64)3, length = (Fixed64)12, expansion = (Fixed64)1/5;
        GetLocalCore(Vector3d.Zero, bodyRotation, center, capsuleRotation, length,
            out var start, out var end, out BigInteger denominator);
        BigInteger halfHeight = height.m_rawValue*denominator;
        BigInteger r = 2*(BigInteger)radius.m_rawValue*denominator;
        BigInteger s = 2*(BigInteger)expansion.m_rawValue*denominator;
        // Ideal frames give relative Up=(-4,-1,8)/9 and the t=1/4 core
        // witness (3/2,0,3/4). All assertions below instead use the actual
        // normalized raw quaternions, with independent unbounded arithmetic.
        foreach (int numerator in new[] { 0, 2, 4 })
        {
            BigInteger x = 2*((4-numerator)*start.X+numerator*end.X);
            BigInteger y = 2*((4-numerator)*start.Y+numerator*end.Y);
            BigInteger z = 2*((4-numerator)*start.Z+numerator*end.Z);
            Assert.False(ConePointDistanceLess(x*x+z*z, y, 4*halfHeight, 4*r, 4*s));
            // The entire chord lies well inside the offset-side axial slab,
            // and cannot reach either the base disk or the apex sphere.
            Assert.True(BigInteger.Abs(y) < 4*(halfHeight-s));
        }
        BigInteger wx = 2*(3*start.X+end.X), wy = 2*(3*start.Y+end.Y), wz = 2*(3*start.Z+end.Z);
        Assert.True(ConePointDistanceLess(wx*wx+wz*wz, wy, 4*halfHeight, 4*r, 4*s));
        foreach (Vector3d axis in new[] { Vector3d.Up, -Vector3d.Up })
            Assert.True(WideFiniteAxisIntersection.DoesCenteredFiniteConePenetrateCapsule(
                Vector3d.Zero, bodyRotation, height, radius, center, capsuleRotation, axis, length, expansion));
    }

    private static bool CoreEntersSolid((BigInteger X, BigInteger Y, BigInteger Z) start,
        (BigInteger X, BigInteger Y, BigInteger Z) end, BigInteger denominator,
        Fixed64 height, Fixed64 radius, bool cone)
    {
        BigInteger x = 2*start.X, y = 2*start.Y, z = 2*start.Z;
        BigInteger dx = 2*(end.X-start.X), dy = 2*(end.Y-start.Y), dz = 2*(end.Z-start.Z);
        BigInteger halfHeight = height.m_rawValue*denominator;
        BigInteger r = 2*(BigInteger)radius.m_rawValue*denominator;
        BigInteger lowN = 0, lowD = 1, highN = 1, highD = 1;
        if (dy.IsZero)
        {
            if (BigInteger.Abs(y) >= halfHeight) return false;
        }
        else
        {
            BigInteger boundD = BigInteger.Abs(dy);
            BigInteger first = dy.Sign > 0 ? -halfHeight-y : y-halfHeight;
            BigInteger last = dy.Sign > 0 ? halfHeight-y : y+halfHeight;
            if (first*lowD > lowN*boundD) { lowN = first; lowD = boundD; }
            if (last*highD < highN*boundD) { highN = last; highD = boundD; }
            if (lowN*highD >= highN*lowD) return false;
        }
        BigInteger a = dx*dx+dz*dz, b = 2*(x*dx+z*dz), c = x*x+z*z;
        if (cone)
        {
            BigInteger h2 = 4*halfHeight*halfHeight, r2 = r*r, belowApex = halfHeight-y;
            a = h2*a-r2*dy*dy;
            b = h2*b+2*r2*belowApex*dy;
            c = h2*c-r2*belowApex*belowApex;
        }
        else c -= r*r;
        return Evaluate(lowN, lowD) < 0 || Evaluate(highN, highD) < 0
            || (a > 0 && -b*lowD > 2*a*lowN && -b*highD < 2*a*highN && 4*a*c < b*b);

        BigInteger Evaluate(BigInteger n, BigInteger d) => a*n*n+b*n*d+c*d*d;
    }

    private static void GetLocalCore(Vector3d origin, FixedQuaternion bodyRotation,
        Vector3d center, FixedQuaternion capsuleRotation, Fixed64 length,
        out (BigInteger X, BigInteger Y, BigInteger Z) start,
        out (BigInteger X, BigInteger Y, BigInteger Z) end, out BigInteger denominator)
    {
        GetLocalPoint(origin, center, bodyRotation, out BigInteger x, out BigInteger y,
            out BigInteger z, out BigInteger bodyDenominator);
        BigInteger cx = capsuleRotation.X.m_rawValue, cy = capsuleRotation.Y.m_rawValue;
        BigInteger cz = capsuleRotation.Z.m_rawValue, cw = capsuleRotation.W.m_rawValue;
        BigInteger capsuleDenominator = cx*cx+cy*cy+cz*cz+cw*cw;
        RotateToLocal(2*(cx*cy-cz*cw), cw*cw-cx*cx+cy*cy-cz*cz, 2*(cy*cz+cx*cw),
            bodyRotation, out BigInteger ax, out BigInteger ay, out BigInteger az, out _);
        BigInteger l = length.m_rawValue;
        denominator = 2*bodyDenominator*capsuleDenominator;
        start = (2*capsuleDenominator*x-l*ax, 2*capsuleDenominator*y-l*ay, 2*capsuleDenominator*z-l*az);
        end = (2*capsuleDenominator*x+l*ax, 2*capsuleDenominator*y+l*ay, 2*capsuleDenominator*z+l*az);
    }

    private static bool DiskDistanceLess(BigInteger q, BigInteger r, BigInteger axial, BigInteger s) =>
        q <= r*r ? axial*axial < s*s : RadicalSign(q+r*r+axial*axial-s*s, -2*r, q) < 0;

    private static int RadicalSign(BigInteger a, BigInteger b, BigInteger q)
    {
        int sign = q.IsZero ? 0 : b.Sign;
        if (a.IsZero) return sign;
        if (sign == 0 || sign == a.Sign) return a.Sign;
        int comparison = (a*a).CompareTo(b*b*q);
        return comparison == 0 ? 0 : comparison > 0 ? a.Sign : sign;
    }

    private static void GetLocalPoint(Vector3d origin, Vector3d point, FixedQuaternion rotation,
        out BigInteger localX, out BigInteger localY, out BigInteger localZ, out BigInteger denominator)
    {
        BigInteger dx = (BigInteger)point.X.m_rawValue-origin.X.m_rawValue;
        BigInteger dy = (BigInteger)point.Y.m_rawValue-origin.Y.m_rawValue;
        BigInteger dz = (BigInteger)point.Z.m_rawValue-origin.Z.m_rawValue;
        RotateToLocal(dx, dy, dz, rotation, out localX, out localY, out localZ, out denominator);
    }

    private static void RotateToLocal(BigInteger dx, BigInteger dy, BigInteger dz, FixedQuaternion rotation,
        out BigInteger localX, out BigInteger localY, out BigInteger localZ, out BigInteger denominator)
    {
        BigInteger x = rotation.X.m_rawValue, y = rotation.Y.m_rawValue;
        BigInteger z = rotation.Z.m_rawValue, w = rotation.W.m_rawValue;
        denominator = x*x+y*y+z*z+w*w;
        localX = dx*(x*x-y*y-z*z+w*w)+dy*2*(x*y+z*w)+dz*2*(x*z-y*w);
        localY = dx*2*(x*y-z*w)+dy*(y*y-x*x-z*z+w*w)+dz*2*(y*z+x*w);
        localZ = dx*2*(x*z+y*w)+dy*2*(y*z-x*w)+dz*(z*z-x*x-y*y+w*w);
    }

    private static bool Cylinder(Vector3d center, Vector3d axis, Fixed64 length, Fixed64 radius) =>
        WideFiniteAxisIntersection.DoesCenteredFiniteCylinderPenetrateCapsule(
            Vector3d.Zero, FixedQuaternion.Identity, Fixed64.Two, (Fixed64)10,
            center, FixedQuaternion.Identity, axis, length, radius);

    private static bool HasNegative(params BigInteger[] coefficients)
    {
        Span<ulong> magnitudes = stackalloc ulong[5 * 16];
        Span<sbyte> signs = stackalloc sbyte[5];
        magnitudes.Clear();
        for (int i = 0; i < 5; i++)
        {
            signs[i] = (sbyte)coefficients[i].Sign;
            BigInteger magnitude = BigInteger.Abs(coefficients[i]);
            for (int word = 0; word < 16; word++)
            {
                magnitudes[i * 16 + word] = (ulong)(magnitude & ulong.MaxValue);
                magnitude >>= 64;
            }
            Assert.True(magnitude.IsZero);
        }
        return WideFiniteAxisIntersection.HasNegativeFiniteAxisPolynomial(magnitudes, signs);
    }
}
