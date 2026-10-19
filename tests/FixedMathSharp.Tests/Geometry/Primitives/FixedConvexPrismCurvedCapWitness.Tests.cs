using System;
using System.Numerics;
using FixedMathSharp.Geometry;
using Xunit;

namespace FixedMathSharp.Tests.Bounds;

public sealed class FixedConvexPrismCurvedCapWitnessTests
{
    [Theory]
    [InlineData("cylinder top", false)]
    [InlineData("cylinder top", true)]
    [InlineData("cylinder bottom", false)]
    [InlineData("cylinder bottom", true)]
    [InlineData("cone base", false)]
    [InlineData("cone base", true)]
    public void OffAxisCap_SelectsARealEdgePointWhenEveryVertexMissesTheDisk(string feature, bool duplicate)
    {
        bool cone = feature == "cone base";
        int sign = feature == "cylinder top" ? 1 : -1;
        Vector2d[] footprint = duplicate
            ? new Vector2d[] { new(Fixed64.Half, (Fixed64)(-2)), new(Fixed64.Half, (Fixed64)(-2)),
                new(Fixed64.FromFraction(3, 2), (Fixed64)(-2)), new(Fixed64.FromFraction(3, 2), (Fixed64)2),
                new(Fixed64.Half, (Fixed64)2) }
            : new Vector2d[] { new(Fixed64.Half, (Fixed64)(-2)), new(Fixed64.FromFraction(3, 2), (Fixed64)(-2)),
                new(Fixed64.FromFraction(3, 2), (Fixed64)2), new(Fixed64.Half, (Fixed64)2) };
        Vector3d origin = new(Fixed64.Zero, (Fixed64)sign * Fixed64.FromFraction(5, 4), Fixed64.Zero);

        Assert.True(TryContact(cone, Vector3d.Zero, origin, footprint, out FixedContactAnchors contact));

        // The near plane gives depth<=1/4. The prism contains p=(1/2,+/-3/4,0),
        // and the cylinder contains ball(p,1/4). For the cone base, that ball
        // also clears its side: 1-(2*p.X+p.Y)=3/4 > sqrt(5)/4.
        // This matching inner-ball bound proves the complete minimum is 1/4.
        Assert.Equal(Fixed64.Quarter, contact.Depth);
        Assert.False(contact.DepthIsClamped);
        Assert.Equal(Vector3d.Up * (Fixed64)sign, contact.Normal);
        AssertPoint(new Vector3d(Fixed64.Half, (Fixed64)sign, Fixed64.Zero), contact.FirstAnchor);
        AssertPoint(new Vector3d(Fixed64.Half, (Fixed64)sign * Fixed64.FromFraction(3, 4), Fixed64.Zero), contact.SecondAnchor);
        // The axis disk center is outside x>=1/2, every corner has |z|=2,
        // and the only admitted finite disk foot is on the left real edge.
        AssertClears(cone, origin, footprint, contact);
    }

    [Theory]
    [InlineData(false)]
    [InlineData(true)]
    public void OffAxisCap_RetainsTheFirstAdmittedFiniteVertex(bool cone)
    {
        int sign = cone ? -1 : 1;
        Fixed64 eighth = Fixed64.FromFraction(1, 8);
        Vector2d[] footprint = { new(Fixed64.Half, -eighth), new(Fixed64.FromFraction(3, 4), -eighth),
            new(Fixed64.FromFraction(3, 4), eighth), new(Fixed64.Half, eighth) };
        Vector3d origin = new(Fixed64.Zero, (Fixed64)sign * Fixed64.FromFraction(5, 4), Fixed64.Zero);

        Assert.True(TryContact(cone, Vector3d.Zero, origin, footprint, out FixedContactAnchors contact));

        // The first near-plane vertex has radial distance sqrt(17)/8<9/16.
        // A radius1/4 ball there fits the cylinder; for the cone base its side
        // clearance is at least(5/8)/sqrt(5)>1/4. The pole again proves both
        // depth bounds without relying on the contact query's selected value.
        Assert.Equal(Fixed64.Quarter, contact.Depth);
        Assert.False(contact.DepthIsClamped);
        Assert.Equal(Vector3d.Up * (Fixed64)sign, contact.Normal);
        AssertPoint(new Vector3d(Fixed64.Half, (Fixed64)sign, -eighth), contact.FirstAnchor);
        AssertPoint(new Vector3d(Fixed64.Half, (Fixed64)sign * Fixed64.FromFraction(3, 4), -eighth), contact.SecondAnchor);
        AssertClears(cone, origin, footprint, contact);
    }

    [Theory]
    [InlineData(false)]
    [InlineData(true)]
    public void NativeYawCap_RetainsTheExactAffineVertexInThePrimitiveFrame(bool cone)
    {
        int sign = cone ? -1 : 1;
        Fixed64 eighth = Fixed64.FromFraction(1, 8), yaw = Fixed64.FromFraction(1, 3);
        Vector2d[] footprint = { new(Fixed64.Half, -eighth), new(Fixed64.FromFraction(3, 4), -eighth),
            new(Fixed64.FromFraction(3, 4), eighth), new(Fixed64.Half, eighth) };
        Vector3d origin = new(Fixed64.Zero, (Fixed64)sign * Fixed64.FromFraction(5, 4), Fixed64.Zero);
        BigInteger scale = BigInteger.One << 32;
        BigInteger cosine = FixedMath.Cos(yaw).m_rawValue, sine = FixedMath.Sin(yaw).m_rawValue;
        BigInteger yawNormSquared = cosine * cosine + sine * sine;
        Assert.True(yawNormSquared > 0);
        Assert.True(68 * yawNormSquared < 81 * scale * scale);
        Assert.True(37 * yawNormSquared < 64 * scale * scale);
        Fixed64 x = RoundNativeCoordinate(footprint[0].X.m_rawValue * cosine - footprint[0].Y.m_rawValue * sine);
        Fixed64 z = RoundNativeCoordinate(footprint[0].X.m_rawValue * sine + footprint[0].Y.m_rawValue * cosine);

        Assert.True(TryContact(cone, Vector3d.Zero, origin, footprint, out FixedContactAnchors contact, yaw));

        // Native yaw is the exact affine map from public sine/cosine raw
        // values, not a yaw quaternion. Its nonzero determinant leaves the
        // axis center outside local x>=1/2; every vertex is inside the disk.
        // The inequalities above prove first-vertex radius<9/16 and maximum
        // vertex radius<1. Thus ball(first near-plane vertex,1/4) fits the
        // cylinder and cone base, as in the unrotated inner-ball proof.
        // The first minimum is bottom index0 for Up, top index1 for Down.
        Assert.Equal(Fixed64.Quarter, contact.Depth);
        Assert.False(contact.DepthIsClamped);
        Assert.Equal(Vector3d.Up * (Fixed64)sign, contact.Normal);
        Assert.Equal(Vector3d.Zero, contact.FirstAnchor.Origin);
        Assert.Equal(origin, contact.SecondAnchor.Origin);
        Assert.Equal(FixedQuaternion.Identity, contact.FirstAnchor.Rotation);
        Assert.Equal(FixedQuaternion.Identity, contact.SecondAnchor.Rotation);
        AssertPoint(new Vector3d(x, (Fixed64)sign, z), contact.FirstAnchor);
        AssertPoint(new Vector3d(x, (Fixed64)sign * Fixed64.FromFraction(3, 4), z), contact.SecondAnchor);
        AssertClears(cone, origin, footprint, contact, yaw);
    }

    [Fact]
    public void NonwinningApexPole_RejectsAProjectedFeatureThatCannotContainTheApex()
    {
        Fixed64 eighth = Fixed64.FromFraction(1, 8);
        Vector2d[] footprint = { new(eighth, -Fixed64.Quarter), new(Fixed64.FromFraction(3, 8), -Fixed64.Quarter),
            new(Fixed64.FromFraction(3, 8), Fixed64.Quarter), new(eighth, Fixed64.Quarter) };
        Vector3d origin = new(Fixed64.Zero, Fixed64.FromFraction(5, 4), Fixed64.Zero);
        // 2x+y<=1 supports the whole cone and the prism lies on its other
        // side. Their unique common point is(1/8,3/4,0); the Up pole's positive
        // overlap is not the global minimum and cannot supply matched anchors.
        Assert.True(TryContact(true, Vector3d.Zero, origin, footprint, out FixedContactAnchors contact));
        Assert.Equal(Fixed64.Zero, contact.Depth);
        Vector3d touch = new(eighth, Fixed64.FromFraction(3, 4), Fixed64.Zero);
        AssertPoint(touch, contact.FirstAnchor); AssertPoint(touch, contact.SecondAnchor);

        Span<WideDirection3d> vertices = stackalloc WideDirection3d[32];
        var geometry = new CurvedPrismContactGeometry(Vector3d.Zero, FixedQuaternion.Identity, Vector3d.Up,
            Fixed64.Two, Fixed64.One, origin, Fixed64.Zero, footprint, Fixed64.Half, true, vertices);
        Span<ulong> normal = stackalloc ulong[3 * ConvexContactCandidate.Words];
        Span<int> normalSigns = stackalloc int[3];
        CylinderContactAlgebra.WriteDirection(geometry.Axis, normal, normalSigns);
        const int words = 112;
        Span<ulong> point = stackalloc ulong[3 * words], denominator = stackalloc ulong[words];
        Span<int> pointSigns = stackalloc int[3];
        // This valid candidate asks whether the minimum prism feature meets
        // the apex's radius-zero projected disk. Every prism point has x>=1/8.
        Assert.False(CurvedPrismCapWitness.TryGetPoint(geometry, normal, normalSigns,
            0, 1, point, pointSigns, denominator));
    }

    private static bool TryContact(bool cone, Vector3d center, Vector3d origin, Vector2d[] footprint,
        out FixedContactAnchors contact, Fixed64 yaw = default) => cone
        ? FixedConvexPrismRelations.TryGetCenteredConeContact(center, FixedQuaternion.Identity, Vector3d.Up,
            Fixed64.Two, Fixed64.One, origin, yaw, footprint, Fixed64.Half, out contact)
        : FixedConvexPrismRelations.TryGetCenteredCylinderContact(center, FixedQuaternion.Identity, Vector3d.Up,
            Fixed64.Two, Fixed64.One, origin, yaw, footprint, Fixed64.Half, out contact);

    private static void AssertClears(bool cone, Vector3d origin, Vector2d[] footprint, FixedContactAnchors contact,
        Fixed64 yaw = default)
    {
        Vector3d center = -contact.Normal * (contact.Depth + Fixed64.FromRaw(8));
        Fixed64 primitiveMaximum = Vector3d.Dot(center, contact.Normal) + Fixed64.One;
        Fixed64 prismMinimum = origin.Y * contact.Normal.Y - Fixed64.Half;
        Assert.True(primitiveMaximum < prismMinimum);
        Assert.False(TryContact(cone, center, origin, footprint, out FixedContactAnchors cleared, yaw));
        Assert.Equal(default, cleared);
    }

    private static Fixed64 RoundNativeCoordinate(BigInteger numerator)
    {
        BigInteger denominator = BigInteger.One << 32;
        BigInteger rounded = BigInteger.DivRem(BigInteger.Abs(numerator), denominator, out BigInteger remainder);
        int halfComparison = (2 * remainder).CompareTo(denominator);
        if (halfComparison > 0 || (halfComparison == 0 && !rounded.IsEven)) rounded++;
        return Fixed64.FromRaw((long)(numerator.Sign * rounded));
    }

    private static void AssertPoint(Vector3d expected, FixedPointAnchor anchor)
    {
        Assert.True(anchor.TryGetPoint(out Vector3d point));
        Assert.Equal(expected, point);
    }
}
