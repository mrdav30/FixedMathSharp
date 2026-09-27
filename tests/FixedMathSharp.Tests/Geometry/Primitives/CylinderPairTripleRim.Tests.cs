using System.Numerics;
using FixedMathSharp.Geometry;
using Xunit;

namespace FixedMathSharp.Tests;

public sealed class CylinderPairTripleRimTests
{
    [Theory]
    [InlineData(1521, 1, 1)]
    [InlineData(2929, 1, 1)]
    [InlineData(1521, -1, 1)]
    [InlineData(1521, 1, -1)]
    [InlineData(2929, -1, 1)]
    [InlineData(2929, 1, -1)]
    public void Contact_GenuineRankTwoTripleDoesNotReplaceASmallerSupportBound(
        int distanceScale, int firstCapSign, int secondCapSign)
    {
        // Euler-brick direction (44,117,240) has radial lengths125,267.
        // The two curvature eigenvalues give these exact triple witnesses;
        // the independent integer pencil check below proves multiplicity3.
        const int scale = 2225;
        const int length = 200;
        const int firstRadius = 125 * scale, secondRadius = 267 * scale;
        int cx = (distanceScale - scale) * 44;
        int cy = (distanceScale - 2 * scale) * 117;
        int cz = (distanceScale - scale) * 240;
        int x = length / 2 * secondCapSign - cx;
        int y = -cy;
        int z = length / 2 * firstCapSign - cz;
        BigInteger tripleSquared = (BigInteger)distanceScale * distanceScale * 73225;
        AssertRankTwoTriple(cx, cy, cz, firstRadius, secondRadius,
            scale * 44, scale * 117, tripleSquared);

        // P=(x,0,0) lies strictly inside both cylinders, proving overlap.
        Assert.True((BigInteger)x * x < (BigInteger)firstRadius * firstRadius);
        Assert.True((BigInteger)y * y + (BigInteger)z * z < (BigInteger)secondRadius * secondRadius);
        long upper = firstRadius + length / 2 - System.Math.Abs(x); // X support gap.
        Assert.True(upper > 0 && (BigInteger)upper * upper < tripleSquared);
        Assert.True(FixedSegment.TryGetCenteredFiniteCylindersContact(
            Vector3d.Zero, FixedQuaternion.Identity, Vector3d.Forward, (Fixed64)length, (Fixed64)firstRadius,
            new Vector3d(x, y, z), FixedQuaternion.Identity, Vector3d.Right, (Fixed64)length, (Fixed64)secondRadius,
            out FixedContactAnchors contact));
        Assert.True(contact.Depth > Fixed64.Zero);
        Assert.True(contact.Depth <= (Fixed64)upper);
        Assert.False(contact.DepthIsClamped);
    }

    [Fact]
    public void Contact_HZeroTripleRetainsTheSmallerCapSupportBound()
    {
        // a=Z,b=(3,0,4)/5,p=(0,25,0),q=(-12,0,9),c=(12,-25,16).
        // u=20b,v=(0,0,25): H=0. At p, the translated T spatial
        // block is [[0,-300],[-300,400]] up to a nonzero scale:
        // rank2 with zero circle-tangent curvature, hence a triple pencil.
        // P=(-9,20,-4) belongs strictly to both cylinders:
        // first radial square481<625; second axial12/5<5 and
        // radial square34-144/25<225. The Z support gap is11<25.
        Fixed64 s = Fixed64.FromRaw(1920767767);
        Assert.True(FixedSegment.TryGetCenteredFiniteCylindersContact(
            Vector3d.Zero, FixedQuaternion.Identity, Vector3d.Forward, (Fixed64)10, (Fixed64)25,
            new Vector3d(-9, 25, -7), new FixedQuaternion(Fixed64.Zero, -s, Fixed64.Zero, 2 * s),
            Vector3d.Right, (Fixed64)10, (Fixed64)15, out FixedContactAnchors contact));
        Assert.True(contact.Depth > Fixed64.Zero);
        Assert.True(contact.Depth <= (Fixed64)11);
        Assert.False(contact.DepthIsClamped);
    }

    [Fact]
    public void Contact_ZeroRadialTripleRetainsTheSmallerCapSupportBound()
    {
        // p=(25,0,0),q=(0,3,4),c=(-16,-3,-4),v=(9,0,0).
        // q.v=0. The independent pencil still has a finite rank2 triple.
        AssertRankTwoTriple(-16, -3, -4, 25, 5, 25, 0, 81);
        // P=(17,3,1/2) is strictly inside both cylinders. Their Z
        // support gap is1, strictly less than the triple distance9.
        Assert.True(FixedSegment.TryGetCenteredFiniteCylindersContact(
            Vector3d.Zero, FixedQuaternion.Identity, Vector3d.Forward, Fixed64.Two, (Fixed64)25,
            new Vector3d(17, 3, 5), FixedQuaternion.Identity, Vector3d.Right, Fixed64.Two, (Fixed64)5,
            out FixedContactAnchors contact));
        Assert.True(contact.Depth > Fixed64.Zero);
        Assert.True(contact.Depth <= Fixed64.One);
        Assert.False(contact.DepthIsClamped);
    }

    [Fact]
    public void Contact_OppositeRadialTripleCannotOverrideASeparatingAxis()
    {
        // These signed radial scales make the stationary Hessian singular,
        // but p.v>0,q.v<0 do not describe one common support direction.
        const long a = 4950625, d = 4702817;
        Assert.Equal(BigInteger.Zero, (BigInteger)d * d - (a - d) * (BigInteger)d
            - (BigInteger)d * (a - 704 * 704));
        Fixed64 cx = Fixed64.FromRaw((d - a) * 44 << 31);
        Fixed64 cy = Fixed64.FromRaw((2 * d - a) * 117 << 31);
        Fixed64 cz = (Fixed64)(d * 240);
        Fixed64 secondRadius = Fixed64.FromRaw(d * 267 << 31);
        Vector3d second = new(Fixed64.One - cx, -cy, Fixed64.One - cz);
        Assert.True(Fixed64.One + secondRadius < -second.Z); // Strict Z separation.
        Assert.False(FixedSegment.TryGetCenteredFiniteCylindersContact(
            Vector3d.Zero, FixedQuaternion.Identity, Vector3d.Forward, Fixed64.Two,
            Fixed64.FromRaw(a * 125 << 31), second, FixedQuaternion.Identity,
            Vector3d.Right, Fixed64.Two, secondRadius, out FixedContactAnchors contact));
        Assert.Equal(default, contact);
    }

    private static void AssertRankTwoTriple(BigInteger cx, BigInteger cy, BigInteger cz,
        BigInteger ra, BigInteger rb, BigInteger px, BigInteger py, BigInteger squaredValue)
    {
        // Independent physical-unit pencil for first axis Z, second axis X.
        BigInteger rho = ra * ra, tau = rb * rb;
        BigInteger k = cx * cx + cy * cy + cz * cz + rho - tau - squaredValue;
        BigInteger xx = 4 * (cx * cx + tau), xy = 4 * cx * cy, yy = 4 * cy * cy;
        BigInteger lx = 2 * k * cx + 4 * tau * cx, ly = 2 * k * cy;
        BigInteger constant = k * k - 4 * tau * squaredValue + 4 * tau * cx * cx;
        BigInteger lambda = BigInteger.DivRem(xx * px + xy * py + lx, px, out BigInteger remainder);
        Assert.Equal(BigInteger.Zero, remainder);
        Assert.Equal(rho, px * px + py * py);
        Assert.Equal(BigInteger.Zero, xy * px + (yy - lambda) * py + ly);
        Assert.Equal(BigInteger.Zero, lx * px + ly * py + constant + lambda * rho);
        BigInteger a = rho;
        BigInteger b = constant - rho * (xx + yy);
        BigInteger c = rho * (xx * yy - xy * xy) - constant * (xx + yy) + lx * lx + ly * ly;
        BigInteger d = constant * (xx * yy - xy * xy) + 2 * xy * lx * ly - xx * ly * ly - yy * lx * lx;
        Assert.Equal(BigInteger.Zero, ((a * lambda + b) * lambda + c) * lambda + d);
        Assert.Equal(BigInteger.Zero, 3 * a * lambda * lambda + 2 * b * lambda + c);
        Assert.Equal(BigInteger.Zero, 6 * a * lambda + 2 * b);
        Assert.NotEqual(BigInteger.Zero, (xx - lambda) * (yy - lambda) - xy * xy);
    }
}
