using System;
using System.Numerics;
using FixedMathSharp.Geometry;
using Xunit;

namespace FixedMathSharp.Tests.Bounds;

public sealed class FixedConvexPrismCurvedWitnessTests
{
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
