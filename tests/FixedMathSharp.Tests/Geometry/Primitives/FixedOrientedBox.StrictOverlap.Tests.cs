using System;
using System.Numerics;
using FixedMathSharp.Geometry;
using Xunit;

namespace FixedMathSharp.Tests.Bounds;

public sealed class FixedOrientedBoxStrictOverlapTests
{
    [Theory]
    [InlineData(-1, false)]
    [InlineData(0, false)]
    [InlineData(1, true)]
    public void SphereCorner_PreservesStrictSignBeforeDepthRounding(long rawOffset, bool penetrates)
    {
        Fixed64 offset = Fixed64.FromRaw(rawOffset);
        Vector3d center = new((Fixed64)3 + offset, Fixed64.Zero, (Fixed64)4 - offset);
        Assert.Equal(penetrates, WideOrientedBox.DoesSpherePenetrate(
            new Vector3d(-1, 0, -1), FixedQuaternion.Identity, Vector3d.One,
            center, (Fixed64)5));
    }

    [Theory]
    [InlineData(-1, 0, false)]
    [InlineData(0, 0, false)]
    [InlineData(1, 0, true)]
    [InlineData(-1, 2, false)]
    [InlineData(0, 2, false)]
    [InlineData(1, 2, true)]
    public void CapsuleEdge_PreservesStrictSignBeforeDepthRounding(long rawOffset, int length, bool penetrates)
    {
        Fixed64 offset = Fixed64.FromRaw(rawOffset);
        Vector3d center = new((Fixed64)3 + offset, Fixed64.Zero, (Fixed64)4 - offset);
        Assert.Equal(penetrates, WideOrientedBox.DoesCenteredCapsulePenetrate(
            new Vector3d(-1, 0, -1), FixedQuaternion.Identity, Vector3d.One,
            center, FixedQuaternion.Identity, Vector3d.Up, (Fixed64)length, (Fixed64)5));
    }

    [Theory]
    [InlineData(0, true)]
    [InlineData(1, false)]
    [InlineData(2, false)]
    public void ZeroRadiusPoint_RequiresBoxInterior(int x, bool penetrates)
    {
        Vector3d point = new(x, 0, 0);
        Assert.Equal(penetrates, WideOrientedBox.DoesSpherePenetrate(
            Vector3d.Zero, FixedQuaternion.Identity, Vector3d.One, point, Fixed64.Zero));
        Assert.Equal(penetrates, WideOrientedBox.DoesCenteredCapsulePenetrate(
            Vector3d.Zero, FixedQuaternion.Identity, Vector3d.One,
            point, FixedQuaternion.Identity, Vector3d.Up, Fixed64.Zero, Fixed64.Zero));
        Assert.Equal(penetrates, WideOrientedBox.DoesCenteredCapsulePenetrate(
            Vector3d.Zero, FixedQuaternion.Identity, Vector3d.One,
            point, FixedQuaternion.Identity, Vector3d.Up, Fixed64.One, Fixed64.Zero));
    }

    [Theory]
    [InlineData(0, false)]
    [InlineData(1, false)]
    [InlineData(2, true)]
    public void FullDomainCancellation_PreservesLastRawUnit(long inwardOffset, bool penetrates)
    {
        Vector3d boxCenter = new(Fixed64.MinValue, Fixed64.Zero, Fixed64.Zero);
        Vector3d extents = new(Fixed64.MaxValue, Fixed64.One, Fixed64.One);
        Vector3d sphereCenter = new(Fixed64.MaxValue - Fixed64.FromRaw(inwardOffset), Fixed64.Zero, Fixed64.Zero);
        // The center difference exceeds Fixed64. At inwardOffset=1 it is
        // exactly twice MaxValue: the extent and radius meet without overlap.
        Assert.Equal(penetrates, WideOrientedBox.DoesSpherePenetrate(
            boxCenter, FixedQuaternion.Identity, extents, sphereCenter, Fixed64.MaxValue));
        Assert.Equal(penetrates, WideOrientedBox.DoesCenteredCapsulePenetrate(
            boxCenter, FixedQuaternion.Identity, extents,
            sphereCenter, FixedQuaternion.Identity, Vector3d.Up, Fixed64.Two, Fixed64.MaxValue));
    }

    [Theory]
    [InlineData(-1, false)]
    [InlineData(0, false)]
    [InlineData(1, true)]
    public void QuarterTurn_PreservesExactFaceBoundary(long inwardOffset, bool penetrates)
    {
        // Rational basis for this quaternion is exactly a quarter-turn, even
        // though its normalized quaternion components are rounded.
        var rotation = new FixedQuaternion(Fixed64.Zero, Fixed64.Zero, Fixed64.One, Fixed64.One).Normalized;
        Vector3d point = new(Fixed64.Zero, (Fixed64)3 - Fixed64.FromRaw(inwardOffset), Fixed64.Zero);
        Vector3d extents = new(2, 1, 1);
        Assert.Equal(penetrates, WideOrientedBox.DoesSpherePenetrate(
            Vector3d.Zero, rotation, extents, point, Fixed64.One));
        Assert.Equal(penetrates, WideOrientedBox.DoesCenteredCapsulePenetrate(
            Vector3d.Zero, rotation, extents, point, rotation,
            Vector3d.Up, Fixed64.One, Fixed64.One));
    }

    [Fact]
    public void SphereRigidFrame_MatchesIndependentUnboundedOracle()
    {
        var rotation = new FixedQuaternion((Fixed64)1, (Fixed64)(-2), (Fixed64)3, (Fixed64)4).Normalized;
        Vector3d[] centers = { Vector3d.Zero,
            new(Fixed64.MinValue, Fixed64.MaxValue, Fixed64.MinValue) };
        Vector3d[] extents = { Vector3d.One,
            new(Fixed64.MaxValue, Fixed64.MaxValue, Fixed64.MaxValue) };
        Vector3d[] points = { Vector3d.Zero, new(1, 2, 3), new(-3, 0, 2),
            new(Fixed64.MaxValue, Fixed64.MinValue, Fixed64.MaxValue),
            new(Fixed64.MinValue, Fixed64.MaxValue, Fixed64.MinValue) };
        foreach (Vector3d center in centers)
        foreach (Vector3d extent in extents)
        foreach (Vector3d point in points)
        foreach (Fixed64 radius in new[] { Fixed64.Zero, Fixed64.MinIncrement, Fixed64.One, Fixed64.MaxValue })
        {
            BigInteger x = rotation.X.m_rawValue, y = rotation.Y.m_rawValue;
            BigInteger z = rotation.Z.m_rawValue, w = rotation.W.m_rawValue;
            BigInteger dx = (BigInteger)point.X.m_rawValue - center.X.m_rawValue;
            BigInteger dy = (BigInteger)point.Y.m_rawValue - center.Y.m_rawValue;
            BigInteger dz = (BigInteger)point.Z.m_rawValue - center.Z.m_rawValue;
            BigInteger d = x*x + y*y + z*z + w*w;
            BigInteger localX = dx*(x*x-y*y-z*z+w*w) + dy*2*(x*y+z*w) + dz*2*(x*z-y*w);
            BigInteger localY = dx*2*(x*y-z*w) + dy*(y*y-x*x-z*z+w*w) + dz*2*(y*z+x*w);
            BigInteger localZ = dx*2*(x*z+y*w) + dy*2*(y*z-x*w) + dz*(z*z-x*x-y*y+w*w);
            BigInteger gapX = BigInteger.Abs(localX) - extent.X.m_rawValue*d;
            BigInteger gapY = BigInteger.Abs(localY) - extent.Y.m_rawValue*d;
            BigInteger gapZ = BigInteger.Abs(localZ) - extent.Z.m_rawValue*d;
            bool inside = gapX < 0 && gapY < 0 && gapZ < 0;
            gapX = BigInteger.Max(gapX, 0);
            gapY = BigInteger.Max(gapY, 0);
            gapZ = BigInteger.Max(gapZ, 0);
            BigInteger scaledRadius = radius.m_rawValue*d;
            bool expected = inside || gapX*gapX + gapY*gapY + gapZ*gapZ < scaledRadius*scaledRadius;
            Assert.Equal(expected, WideOrientedBox.DoesSpherePenetrate(center, rotation, extent, point, radius));
            Assert.Equal(expected, WideOrientedBox.DoesCenteredCapsulePenetrate(center, rotation, extent,
                point, rotation, Vector3d.Up, Fixed64.Zero, radius));
        }
    }

    [Fact]
    public void StrictClassifiers_DoNotAllocateAfterWarmup()
    {
        var point = new Vector3d(1, 0, 0);
        for (int i = 0; i < 8; i++)
        {
            _ = WideOrientedBox.DoesSpherePenetrate(Vector3d.Zero, FixedQuaternion.Identity,
                Vector3d.One, point, Fixed64.One);
            _ = WideOrientedBox.DoesCenteredCapsulePenetrate(Vector3d.Zero, FixedQuaternion.Identity,
                Vector3d.One, point, FixedQuaternion.Identity, Vector3d.Up, Fixed64.Two, Fixed64.One);
        }
        long before = GC.GetAllocatedBytesForCurrentThread();
        bool result = true;
        for (int i = 0; i < 8; i++)
        {
            result &= WideOrientedBox.DoesSpherePenetrate(Vector3d.Zero, FixedQuaternion.Identity,
                Vector3d.One, point, Fixed64.One);
            result &= WideOrientedBox.DoesCenteredCapsulePenetrate(Vector3d.Zero, FixedQuaternion.Identity,
                Vector3d.One, point, FixedQuaternion.Identity, Vector3d.Up, Fixed64.Two, Fixed64.One);
        }
        long allocated = GC.GetAllocatedBytesForCurrentThread() - before;
        Assert.True(result);
        Assert.Equal(0, allocated);
    }
}
