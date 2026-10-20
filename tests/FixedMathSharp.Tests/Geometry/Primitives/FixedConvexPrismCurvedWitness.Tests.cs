using System;
using System.Numerics;
using FixedMathSharp.Geometry;
using Xunit;

namespace FixedMathSharp.Tests.Bounds;

public sealed class FixedConvexPrismCurvedWitnessTests
{
    [Theory]
    [InlineData(true, -1L, false)]
    [InlineData(true, 0L, false)]
    [InlineData(true, 1L, false)]
    [InlineData(false, -1L, false)]
    [InlineData(false, 0L, false)]
    [InlineData(false, 1L, false)]
    [InlineData(true, 0L, true)]
    [InlineData(false, 0L, true)]
    public void InitialZeroCertificate_PreservesCompleteAdmissionAndFiniteWitnesses(bool cone, long gapRaw, bool exterior)
    {
        const long k = 1_920_767_767L;
        FixedQuaternion rotation = cone ? FixedQuaternion.Identity
            : new(Fixed64.Zero, Fixed64.Zero, Fixed64.FromRaw(-k), Fixed64.FromRaw(2 * k));
        Vector3d center = Vector3d.Up * (cone ? Fixed64.FromFraction(-3, 2) : (Fixed64)8);
        Vector3d normal = cone ? Vector3d.Up : Vector3d.Down;
        Vector3d origin = normal * Fixed64.FromRaw(gapRaw);
        // An exterior footprint still has zero vertical gap: complete admission
        // must reject it before that projection can certify a global minimum.
        if (exterior) origin += Vector3d.Forward * (Fixed64)20;
        Vector2d[] footprint = Square((Fixed64)10);
        bool hit = cone
            ? FixedConvexPrismRelations.TryGetCenteredConeContact(center, rotation, Vector3d.Up,
                Fixed64.One, Fixed64.Half, origin, Fixed64.Zero, footprint, Fixed64.One, out FixedContactAnchors contact)
            : FixedConvexPrismRelations.TryGetCenteredCylinderContact(center, rotation, Vector3d.Up,
                (Fixed64)10, (Fixed64)5, origin, Fixed64.Zero, footprint, Fixed64.One, out contact);
        Assert.Equal(gapRaw <= 0 && !exterior, hit);
        if (!hit)
        {
            Assert.Equal(default, contact);
            return;
        }
        // The cone apex is (0,-1,0). The cylinder's exact authored axis
        // (4,3,0)/5 gives its lowest rim point (-1,1,0). The broad prism face
        // has the same supporting plane at zero and separates after one raw.
        Assert.Equal(normal, contact.Normal);
        Assert.Equal(Fixed64.FromRaw(-gapRaw), contact.Depth);
        Assert.False(contact.DepthIsClamped);
        Vector3d first = Offset(contact.FirstAnchor, Vector3d.Zero);
        Vector3d second = Offset(contact.SecondAnchor, Vector3d.Zero);
        Assert.Equal(cone ? new Vector3d(0, -1, 0) : new Vector3d(-1, 1, 0), first);
        Assert.Equal(first + origin, second);
        Assert.Equal(center, contact.FirstAnchor.Origin);
        Assert.Equal(rotation, contact.FirstAnchor.Rotation);
        Assert.Equal(origin, contact.SecondAnchor.Origin);
        AssertPaired(first, second, contact);
    }

    [Theory]
    [InlineData(-1L)]
    [InlineData(0L)]
    [InlineData(1L)]
    public void CylinderSideVertex_RetainsTheFirstExactZeroAndRadialNeighbors(long gapRaw)
    {
        const long k = 1_920_767_767L;
        FixedQuaternion rotation = new(Fixed64.Zero, Fixed64.Zero,
            Fixed64.FromRaw(-k), Fixed64.FromRaw(2 * k));
        Vector3d point = new(3, -4, 12);
        Vector3d shift = point * Fixed64.FromRaw(gapRaw);
        Vector3d origin = new Vector3d(3, -5, 12) + shift;
        Vector2d[] footprint = { new(0, 0), new(2, 0), new(2, 2), new(0, 2) };
        bool hit = FixedConvexPrismRelations.TryGetCenteredCylinderContact(
            Vector3d.Zero, rotation, Vector3d.Up, (Fixed64)10, (Fixed64)13,
            origin, Fixed64.Zero, footprint, Fixed64.One, out FixedContactAnchors contact);
        Assert.Equal(gapRaw <= 0, hit);
        if (!hit)
        {
            Assert.Equal(default, contact);
            return;
        }
        // Axis=(4,3,0)/5 is perpendicular to P=(3,-4,12), whose length
        // is 13. The cylinder has 3X-4Y+12Z<=169 and this prism has >=169.
        // Their unique common point P lies inside the finite cylinder side
        // and strictly inside the prism vertex's normal cone. A signed raw
        // multiple of P moves that vertex radially by exactly 13 raw units.
        // Inward, the cylinder contains a ball of that radius centered at the
        // prism vertex (cap margin five), proving the matching minimum depth.
        Assert.Equal(new Vector3d(Fixed64.FromFraction(3, 13), Fixed64.FromFraction(-4, 13),
            Fixed64.FromFraction(12, 13)), contact.Normal);
        Assert.Equal(Fixed64.FromRaw(-13 * gapRaw), contact.Depth);
        Assert.False(contact.DepthIsClamped);
        Assert.Equal(point, Offset(contact.FirstAnchor, Vector3d.Zero));
        Assert.Equal(point + shift, Offset(contact.SecondAnchor, Vector3d.Zero));
        AssertPaired(point, point + shift, contact);
    }

    [Theory]
    [InlineData(false, -1L)]
    [InlineData(false, 0L)]
    [InlineData(false, 1L)]
    [InlineData(true, -1L)]
    [InlineData(true, 0L)]
    [InlineData(true, 1L)]
    public void RepeatedBoundaryVertices_PreserveClosedContactAndPhysicalWitnesses(bool cone, long gapRaw)
    {
        Vector2d[] footprint = { new(0, -1), new(1, -1), new(1, 1), new(0, 1) };
        var repeated = new Vector2d[20];
        for (int corner = 0; corner < footprint.Length; corner++)
            for (int copy = 0; copy < 5; copy++)
                repeated[corner * 5 + copy] = footprint[corner];
        Vector3d origin = new((cone ? Fixed64.Half : Fixed64.One) + Fixed64.FromRaw(gapRaw),
            cone ? Fixed64.One : Fixed64.Zero, Fixed64.Zero);

        bool expected = Query(cone, Vector3d.Zero, Vector3d.Up, Fixed64.Two, Fixed64.One,
            origin, footprint, Fixed64.One, out FixedContactAnchors original);
        bool actual = Query(cone, Vector3d.Zero, Vector3d.Up, Fixed64.Two, Fixed64.One,
            origin, repeated, Fixed64.One, out FixedContactAnchors expanded);
        // Consecutive duplicate corners describe precisely the same finite
        // rectangle. The cone support 2x+y<=1 and cylinder support x<=1
        // independently put the three translations on either side of touch.
        Assert.Equal(gapRaw <= 0, expected);
        Assert.Equal(expected, actual);
        if (!actual)
        {
            Assert.Equal(default, original);
            Assert.Equal(default, expanded);
            return;
        }
        Assert.Equal(original.Normal, expanded.Normal);
        Assert.Equal(original.Depth, expanded.Depth);
        Assert.Equal(original.DepthIsClamped, expanded.DepthIsClamped);
        Assert.Equal(Offset(original.FirstAnchor, Vector3d.Zero), Offset(expanded.FirstAnchor, Vector3d.Zero));
        Assert.Equal(Offset(original.SecondAnchor, Vector3d.Zero), Offset(expanded.SecondAnchor, Vector3d.Zero));
    }

    [Theory]
    [InlineData(0)]
    [InlineData(1)]
    [InlineData(2)]
    [InlineData(3)]
    [InlineData(4)]
    [InlineData(5)]
    public void Containment_RetainsFinitePairedSupportsAndClearsTheWholeShape(int fixture)
    {
        bool cone = fixture == 1 || fixture == 5;
        Fixed64 length = fixture < 3 ? Fixed64.Two : fixture == 4 ? (Fixed64)6 : (Fixed64)4;
        Fixed64 radius = fixture < 3 ? Fixed64.One : fixture == 3 ? (Fixed64)3 : Fixed64.Two;
        Fixed64 horizontal = fixture < 2 ? (Fixed64)5 : fixture == 2 ? (Fixed64)3 : Fixed64.FromFraction(1, 4);
        Fixed64 vertical = fixture < 2 ? (Fixed64)3 : fixture == 2 ? (Fixed64)5 : Fixed64.FromFraction(1, 4);
        Vector3d origin = fixture == 5 ? -Vector3d.Up : Vector3d.Zero;
        Vector2d[] footprint = Square(horizontal);
        Assert.True(Query(cone, Vector3d.Zero, Vector3d.Up, length, radius, origin, footprint, vertical,
            out FixedContactAnchors contact));

        // The inner shape is separated most cheaply through a cap or a
        // cardinal side of the enclosing box. In the reversed containment
        // rows, the small box exits the cylinder or the cone's flat base.
        Fixed64 expectedDepth = fixture < 3 ? (Fixed64)4 : fixture == 5
            ? Fixed64.FromFraction(5, 4) : Fixed64.FromFraction(9, 4);
        Assert.Equal(expectedDepth, contact.Depth);
        Assert.False(contact.DepthIsClamped);
        if (fixture == 2 || fixture == 4)
        {
            Assert.Equal(Fixed64.Zero, contact.Normal.Y);
            Assert.Equal(Fixed64.One, FixedMath.Abs(contact.Normal.X) + FixedMath.Abs(contact.Normal.Z));
        }
        else
            Assert.Equal(fixture == 5 ? -Vector3d.Up : Vector3d.Up, contact.Normal);

        Vector3d first = Offset(contact.FirstAnchor, Vector3d.Zero);
        Vector3d second = Offset(contact.SecondAnchor, Vector3d.Zero);
        AssertPaired(first, second, contact);
        Assert.InRange(first.Y.m_rawValue, (-length / Fixed64.Two).m_rawValue, (length / Fixed64.Two).m_rawValue);
        Assert.InRange(second.X.m_rawValue, (-horizontal).m_rawValue, horizontal.m_rawValue);
        Assert.InRange(second.Z.m_rawValue, (-horizontal).m_rawValue, horizontal.m_rawValue);
        Assert.InRange(second.Y.m_rawValue, (origin.Y - vertical).m_rawValue, (origin.Y + vertical).m_rawValue);
        if (contact.Normal.Y != Fixed64.Zero)
        {
            Assert.Equal(contact.Normal.Y * length / Fixed64.Two, first.Y);
            Assert.Equal(origin.Y - contact.Normal.Y * vertical, second.Y);
        }
        else
        {
            Assert.Equal(radius, Vector3d.Dot(first, contact.Normal));
            Assert.Equal(-horizontal, Vector3d.Dot(second, contact.Normal));
        }

        Vector3d cleared = origin + contact.Normal * (contact.Depth + Fixed64.FromRaw(8));
        // These cardinal support inequalities separate every point of both
        // complete shapes; the query is an additional regression assertion.
        Fixed64 shapeMaximum = contact.Normal.Y != Fixed64.Zero ? length / Fixed64.Two : radius;
        Fixed64 prismMinimum = Vector3d.Dot(cleared, contact.Normal)
            - (contact.Normal.Y != Fixed64.Zero ? vertical : horizontal);
        Assert.True(prismMinimum > shapeMaximum);
        Assert.False(Query(cone, Vector3d.Zero, Vector3d.Up, length, radius, cleared, footprint, vertical, out _));
    }

    [Theory]
    [InlineData(false)]
    [InlineData(true)]
    public void CylinderSide_ZeroProjectionPatchRetainsTheEarlierEdgeMinimum(bool reverse)
    {
        Vector2d[] footprint = { new(0, 0), new(0, 1), new(1, 1) };
        if (reverse) Array.Reverse(footprint);
        Assert.True(Query(false, Vector3d.Zero, Vector3d.Up, (Fixed64)10, Fixed64.One,
            Vector3d.Zero, footprint, Fixed64.One, out FixedContactAnchors contact));

        // The prism contains the origin and the cylinder contains the unit
        // ball, so the complete Minkowski difference contains that ball.
        // Either cardinal support plane attains depth one. The nonzero
        // diagonal vertex direction has zero minimum projection and the same
        // depth throughout its patch; the earlier real edge owns this tie.
        Vector3d normal = reverse ? Vector3d.Forward : Vector3d.Right;
        Assert.Equal(normal, contact.Normal);
        Assert.Equal(Fixed64.One, contact.Depth);
        Assert.False(contact.DepthIsClamped);
        Vector3d first = Offset(contact.FirstAnchor, Vector3d.Zero);
        Vector3d second = Offset(contact.SecondAnchor, Vector3d.Zero);
        AssertPaired(first, second, contact);
        Assert.Equal(normal.X, first.X);
        Assert.Equal(normal.Z, first.Z);
        Assert.Equal(Fixed64.Zero, second.X);
        Assert.Equal(Fixed64.Zero, second.Z);
        Assert.InRange(first.Y.m_rawValue, (-Fixed64.One).m_rawValue, Fixed64.One.m_rawValue);
        Assert.Equal(first.Y, second.Y);

        Assert.True(Query(false, Vector3d.Zero, Vector3d.Up, (Fixed64)10, Fixed64.One,
            normal, footprint, Fixed64.One, out FixedContactAnchors touching));
        Assert.Equal(Fixed64.Zero, touching.Depth);
        Vector3d cleared = normal * (Fixed64.One + Fixed64.FromRaw(1));
        // Every translated prism point now projects beyond the cylinder's
        // exact cardinal support, independently proving whole-shape clearance.
        Assert.True(Vector3d.Dot(cleared, normal) > Fixed64.One);
        Assert.False(Query(false, Vector3d.Zero, Vector3d.Up, (Fixed64)10, Fixed64.One,
            cleared, footprint, Fixed64.One, out _));
    }

    [Theory]
    [InlineData(false, 1)]
    [InlineData(false, -1)]
    [InlineData(true, 1)]
    [InlineData(true, -1)]
    public void OddDimensions_UseTheAcceptedAuthoredAxisNormForCapWitnesses(bool cone, int orientation)
    {
        const long scale = 1L << 32;
        Vector3d axis = new(Fixed64.Zero, Fixed64.FromRaw(orientation * (scale + 2)), Fixed64.Zero);
        Assert.True(axis.IsNormalized());
        Fixed64 length = Fixed64.FromRaw(2 * scale + 1);
        Fixed64 radius = Fixed64.FromRaw(scale + 3);
        Vector3d origin = Vector3d.Up * (Fixed64.FromFraction(3, 2) * (Fixed64)orientation);
        Vector2d[] footprint = Square(Fixed64.FromFraction(1, 4));
        Assert.True(Query(cone, Vector3d.Zero, axis, length, radius, origin, footprint, Fixed64.Half,
            out FixedContactAnchors contact));

        // The cap is L*|axis|/2, not L/2. The exact overlap in raw units
        // is ((2S+1)(S+2)-2S*S)/(2S), strictly between 2.5 and 3.
        BigInteger numerator = (BigInteger)(2 * scale + 1) * (scale + 2) - 2 * (BigInteger)scale * scale;
        Assert.True(2 * numerator > 5 * (BigInteger)(2 * scale));
        Assert.True(numerator < 3 * (BigInteger)(2 * scale));
        Assert.Equal(Fixed64.FromRaw(3), contact.Depth);
        Assert.False(contact.DepthIsClamped);
        Assert.Equal(Vector3d.Up * (Fixed64)orientation, contact.Normal);
        Assert.Equal(new Vector3d(Fixed64.Zero, Fixed64.FromRaw(orientation * (scale + 3)), Fixed64.Zero),
            Offset(contact.FirstAnchor, Vector3d.Zero));
        Assert.Equal(Vector3d.Up * (Fixed64)orientation, Offset(contact.SecondAnchor, Vector3d.Zero));
        Vector3d cleared = origin + contact.Normal * Fixed64.FromRaw(5);
        Assert.False(Query(cone, Vector3d.Zero, axis, length, radius, cleared, footprint, Fixed64.Half, out _));
    }

    [Theory]
    [InlineData(false)]
    [InlineData(true)]
    public void CollapsedFootprint_RetainsTheNearestFiniteSidePair(bool line)
    {
        Vector2d[] footprint = line
            ? new Vector2d[] { new(Fixed64.Zero, -Fixed64.Half), new(Fixed64.Zero, Fixed64.Half), new(Fixed64.Zero, Fixed64.Half) }
            : new Vector2d[] { Vector2d.Zero, Vector2d.Zero, Vector2d.Zero };
        Vector3d origin = new(Fixed64.FromFraction(1, 4), Fixed64.Zero, Fixed64.Zero);
        Assert.True(Query(false, Vector3d.Zero, Vector3d.Up, Fixed64.Two, Fixed64.One,
            origin, footprint, Fixed64.FromFraction(1, 4), out FixedContactAnchors contact));
        Assert.Equal(Fixed64.FromFraction(3, 4), contact.Depth);
        Assert.Equal(Vector3d.Right, contact.Normal);
        Vector3d first = Offset(contact.FirstAnchor, Vector3d.Zero);
        Vector3d second = Offset(contact.SecondAnchor, Vector3d.Zero);
        Assert.Equal(Fixed64.One, first.X);
        Assert.Equal(origin.X, second.X);
        Assert.Equal(Fixed64.Zero, first.Z);
        Assert.Equal(Fixed64.Zero, second.Z);
        Assert.InRange(first.Y.m_rawValue, (-Fixed64.FromFraction(1, 4)).m_rawValue, Fixed64.FromFraction(1, 4).m_rawValue);
        AssertPaired(first, second, contact);
        Assert.False(Query(false, Vector3d.Zero, Vector3d.Up, Fixed64.Two, Fixed64.One,
            origin + contact.Normal * (contact.Depth + Fixed64.FromRaw(2)), footprint,
            Fixed64.FromFraction(1, 4), out _));
    }

    [Theory]
    [InlineData(false)]
    [InlineData(true)]
    public void ZeroRadius_HasFinitePairedSegmentSupports(bool cone)
    {
        Vector2d[] footprint = Square(Fixed64.Half);
        Assert.True(Query(cone, Vector3d.Zero, Vector3d.Up, Fixed64.Two, Fixed64.Zero,
            Vector3d.Zero, footprint, Fixed64.Half, out FixedContactAnchors contact));
        Assert.Equal(Fixed64.Half, contact.Depth);
        Assert.False(contact.DepthIsClamped);
        Vector3d first = Offset(contact.FirstAnchor, Vector3d.Zero);
        Vector3d second = Offset(contact.SecondAnchor, Vector3d.Zero);
        AssertPaired(first, second, contact);
        Assert.InRange(first.Y.m_rawValue, (-Fixed64.One).m_rawValue, Fixed64.One.m_rawValue);
        Assert.Equal(Fixed64.Zero, first.X);
        Assert.Equal(Fixed64.Zero, first.Z);
        Vector3d cleared = contact.Normal * (contact.Depth + Fixed64.FromRaw(4));
        // The chosen normal is perpendicular to the segment, so its
        // entire support is zero and the box minimum is now positive.
        Assert.True(Vector3d.Dot(cleared, contact.Normal) > Fixed64.Half);
        Assert.False(Query(cone, Vector3d.Zero, Vector3d.Up, Fixed64.Two, Fixed64.Zero,
            cleared, footprint, Fixed64.Half, out _));
    }

    [Theory]
    [InlineData(false)]
    [InlineData(true)]
    public void ZeroLength_RetainsThePublicPositiveLengthContract(bool cone)
    {
        Vector2d[] footprint = Square(Fixed64.Half);
        var exception = Assert.Throws<ArgumentOutOfRangeException>(() => Query(cone, Vector3d.Zero,
            Vector3d.Up, Fixed64.Zero, Fixed64.One, Vector3d.Zero, footprint, Fixed64.Half, out _));
        Assert.Equal(cone ? "coneHeight" : "cylinderAxisLength", exception.ParamName);
    }

    [Theory]
    [InlineData(false)]
    [InlineData(true)]
    public void BeyondWorldRangeCap_RetainsOwnerRelativeWitnesses(bool cone)
    {
        Vector3d center = new(Fixed64.MaxValue - Fixed64.One,
            Fixed64.MaxValue - Fixed64.FromFraction(1, 4), Fixed64.MinValue + Fixed64.One);
        Vector3d origin = center + Vector3d.Up * Fixed64.FromFraction(1, 8);
        Vector2d[] footprint = Square(Fixed64.FromFraction(1, 4));
        Fixed64 halfThickness = Fixed64.FromRaw(1);
        // The very thin target lies above the center. Its finite horizontal
        // extent keeps the apex/cap pole closer than the sloping cone sides.
        Assert.True(Query(cone, center, Vector3d.Up, Fixed64.Two, (Fixed64)4,
            origin, footprint, halfThickness, out FixedContactAnchors contact));
        Assert.Equal(Vector3d.Up, contact.Normal);
        Assert.Equal(Fixed64.FromFraction(7, 8) + halfThickness, contact.Depth);
        Assert.False(contact.FirstAnchor.TryGetPoint(out _));
        Vector3d first = Offset(contact.FirstAnchor, center);
        Vector3d second = Offset(contact.SecondAnchor, center);
        Assert.Equal(Fixed64.One, first.Y);
        Assert.Equal(Fixed64.FromFraction(1, 8) - halfThickness, second.Y);
        AssertPaired(first, second, contact);
        Assert.Equal(center, contact.FirstAnchor.Origin);
        Assert.Equal(origin, contact.SecondAnchor.Origin);
    }

    private static bool Query(bool cone, Vector3d center, Vector3d axis, Fixed64 length, Fixed64 radius,
        Vector3d origin, Vector2d[] footprint, Fixed64 halfThickness, out FixedContactAnchors contact) => cone
        ? FixedConvexPrismRelations.TryGetCenteredConeContact(center, FixedQuaternion.Identity, axis,
            length, radius, origin, Fixed64.Zero, footprint, halfThickness, out contact)
        : FixedConvexPrismRelations.TryGetCenteredCylinderContact(center, FixedQuaternion.Identity, axis,
            length, radius, origin, Fixed64.Zero, footprint, halfThickness, out contact);

    private static Vector2d[] Square(Fixed64 half) => new Vector2d[]
    {
        new(-half, -half), new(half, -half), new(half, half), new(-half, half)
    };

    private static Vector3d Offset(FixedPointAnchor anchor, Vector3d origin)
    {
        Assert.True(anchor.TryGetOffsetFrom(new FixedPointAnchor(origin, FixedQuaternion.Identity, Vector3d.Zero),
            out Vector3d offset));
        return offset;
    }

    private static void AssertPaired(Vector3d first, Vector3d second, FixedContactAnchors contact)
    {
        Vector3d expected = contact.Normal * contact.Depth;
        for (int component = 0; component < 3; component++)
            Assert.True(BigInteger.Abs((BigInteger)first[component].m_rawValue - second[component].m_rawValue
                - expected[component].m_rawValue) <= 4);
    }
}
