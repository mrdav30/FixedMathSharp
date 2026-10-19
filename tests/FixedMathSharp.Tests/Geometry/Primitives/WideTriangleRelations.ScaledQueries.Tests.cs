using System;
using System.Numerics;
using FixedMathSharp.Geometry;
using Xunit;

namespace FixedMathSharp.Tests.Geometry;

public sealed class ScaledTriangleQueryTests
{
    [Theory]
    [InlineData(0)]
    [InlineData(1)]
    [InlineData(2)]
    [InlineData(3)]
    public void ClosestPointMetric_ShouldRetainExactFaceEdgeAndCollapsedPointDistances(int fixture)
    {
        var frame = new ScaledRigidFrame3d(Vector3d.Zero, FixedQuaternion.Identity, Vector3d.Zero,
            Vector3d.One * Fixed64.Quarter, FixedQuaternion.Identity, Vector3d.Zero, Vector3d.One);
        Fixed64 fiveRaw = Fixed64.FromRaw(5);
        var triangle = fixture == 0
            ? new FixedTriangle(new Vector3d(-(Fixed64)2, -Fixed64.One + fiveRaw, -(Fixed64)2),
                new Vector3d((Fixed64)2, Fixed64.One + fiveRaw, -(Fixed64)2), new Vector3d(Fixed64.Zero, fiveRaw, (Fixed64)2))
            : fixture == 2
                ? new FixedTriangle(Vector3d.Right * fiveRaw, Vector3d.Right * fiveRaw, Vector3d.Right * fiveRaw)
                : new FixedTriangle(Vector3d.Zero, new Vector3d(4, 0, 0), new Vector3d(0, 0, 4));
        Vector3d query = fixture == 1 ? Vector3d.One : Vector3d.Zero;
        const int words = 36;
        Span<ulong> values = stackalloc ulong[4 * words]; Span<int> signs = stackalloc int[4];
        var numerator = ContactQuadratic.At(values, signs, 0, words);
        var denominator = ContactQuadratic.At(values, signs, 1, words);
        Assert.True(WideTriangleRelations.TryGetClosestPointAnchor(triangle, frame,
            RationalPoint3d.FromScalarPoint(query), Vector3d.Zero, numerator, denominator, out var closest));
        BigInteger expectedN = fixture switch
        {
            0 => 5,
            1 => (BigInteger)3 << 64,
            2 => 25,
            _ => 0
        };
        int expectedD = fixture switch { 0 => 4, 1 => 2, 2 => 16, _ => 1 };
        Assert.Equal(expectedN * Magnitude(denominator.Rational), Magnitude(numerator.Rational) * expectedD);
        Assert.True(denominator.Signs[0] > 0);
        Assert.Equal(0, numerator.Signs[1]); Assert.Equal(0, denominator.Signs[1]);
        Vector3d expectedPoint = fixture switch
        {
            0 => Vector3d.Up * Fixed64.FromRaw(1),
            1 => new Vector3d(Fixed64.Half, Fixed64.Zero, Fixed64.Half),
            2 => Vector3d.Right * Fixed64.FromRaw(1),
            _ => Vector3d.Zero
        };
        Assert.True(closest.TryGetPoint(out Vector3d point));
        Assert.Equal(expectedPoint, point);
    }

    [Fact]
    public void ClosestPointMetric_ShouldRetainFullDomainRationalQueryDistance()
    {
        var rotation = FixedQuaternion.FromEulerAnglesInDegrees(Fixed64.Zero, Fixed64.Zero, (Fixed64)37);
        var sourceRotation = FixedQuaternion.FromEulerAnglesInDegrees(Fixed64.Zero, (Fixed64)23, Fixed64.Zero);
        var frame = new ScaledRigidFrame3d(Vector3d.Zero, rotation, Vector3d.Zero,
            Vector3d.One * Fixed64.Quarter, sourceRotation, Vector3d.Zero, Vector3d.One);
        var triangle = new FixedTriangle(new Vector3d(-16, 0, -16), new Vector3d(16, 0, -16), new Vector3d(0, 0, 16));
        var query = new FixedPointAnchor(new Vector3d(Fixed64.Zero, Fixed64.MaxValue - Fixed64.One, Fixed64.Zero),
            rotation, Vector3d.Up);
        const int words = 36;
        Span<ulong> values = stackalloc ulong[4 * words]; Span<int> signs = stackalloc int[4];
        var numerator = ContactQuadratic.At(values, signs, 0, words);
        var denominator = ContactQuadratic.At(values, signs, 1, words);
        Assert.True(WideTriangleRelations.TryGetClosestPointAnchor(triangle, frame,
            RationalPoint3d.FromAnchor(query), Vector3d.Zero, numerator, denominator, out var closest));
        BigInteger w = rotation.W.m_rawValue, z = rotation.Z.m_rawValue;
        BigInteger rotationD = w * w + z * z;
        BigInteger y = ((BigInteger)long.MaxValue - (BigInteger.One << 32)) * rotationD
            + (w * w - z * z) * (BigInteger.One << 32);
        Assert.Equal(y * y * Magnitude(denominator.Rational),
            Magnitude(numerator.Rational) * rotationD * rotationD);
        Assert.Equal(Fixed64.Zero, closest.LocalPoint.Y);
    }

    private static BigInteger Magnitude(ReadOnlySpan<ulong> words)
    {
        BigInteger value = BigInteger.Zero;
        for (int index = words.Length - 1; index >= 0; index--) value = (value << 64) + words[index];
        return value;
    }

    [Fact]
    public void ScaledAreaAndSupport_ShouldUseTheFinitePlaneBeforeRounding()
    {
        Vector3d scale = new(Fixed64.FromFraction(2, 3), Fixed64.One, Fixed64.FromFraction(3, 5));
        var frame = new ScaledRigidFrame3d(Vector3d.Zero, FixedQuaternion.Identity, Vector3d.Zero,
            scale, FixedQuaternion.Identity, Vector3d.Zero, Vector3d.One);
        var triangle = new FixedTriangle(Vector3d.Zero, new Vector3d(0, 0, 3), new Vector3d(3, 0, 0));
        WideTriangleRelations.GetScaledAreaAndNormal(triangle, frame, out Fixed64 area, out Vector3d normal);
        System.Numerics.BigInteger numerator = (System.Numerics.BigInteger)9 * scale.X.m_rawValue * scale.Z.m_rawValue;
        System.Numerics.BigInteger denominator = System.Numerics.BigInteger.One << 33;
        var rounded = System.Numerics.BigInteger.DivRem(numerator, denominator, out var remainder);
        if (2 * remainder > denominator || 2 * remainder == denominator && !rounded.IsEven) rounded++;
        Assert.Equal((long)rounded, area.m_rawValue);
        Assert.Equal(Vector3d.Up, normal);

        var support = new FixedPointAnchor(new Vector3d(Fixed64.Half, -Fixed64.One, Fixed64.Half),
            FixedQuaternion.Identity, Vector3d.Zero);
        Assert.True(WideTriangleRelations.TryGetProjectedSupportContact(triangle, frame, support,
            Vector3d.Up, Vector3d.Zero, out FixedContactAnchors contact));
        Assert.Equal(Fixed64.One, contact.Depth);
        Assert.True(contact.FirstAnchor.TryGetPoint(out Vector3d projected));
        Assert.Equal(new Vector3d(Fixed64.Half, Fixed64.Zero, Fixed64.Half), projected);
        Assert.False(WideTriangleRelations.TryGetProjectedSupportContact(triangle, frame, support,
            Vector3d.Right, Vector3d.Zero, out _));
        Assert.False(WideTriangleRelations.TryGetProjectedSupportContact(triangle, frame,
            new FixedPointAnchor(Vector3d.Up, FixedQuaternion.Identity, Vector3d.Zero), Vector3d.Up, Vector3d.Zero, out _));
        Assert.False(WideTriangleRelations.TryGetProjectedSupportContact(triangle, frame,
            new FixedPointAnchor(new Vector3d(3, -1, 3), FixedQuaternion.Identity, Vector3d.Zero), Vector3d.Up, Vector3d.Zero, out _));
        Assert.False(WideTriangleRelations.TryGetSegmentIntersection(new FixedTriangle(Vector3d.Zero, Vector3d.Zero, Vector3d.Zero),
            frame, new FixedSegment(Vector3d.Down, Vector3d.Up), out _, out _));
    }

    [Theory]
    [InlineData(0)]
    [InlineData(45)]
    public void CrossingSegment_ShouldIntersectTheExactScaledPlaneAtItsMidpoint(int angle)
    {
        var rotation = FixedQuaternion.FromEulerAnglesInDegrees(Fixed64.Zero, Fixed64.Zero, (Fixed64)angle);
        Vector3d origin = new(3, 2, -1);
        var frame = new ScaledRigidFrame3d(origin, rotation, Vector3d.Zero,
            new Vector3d(Fixed64.FromFraction(2, 3), Fixed64.One, Fixed64.FromFraction(3, 5)),
            rotation, Vector3d.Zero, new Vector3d(Fixed64.FromFraction(7, 9), Fixed64.One, Fixed64.One));
        var triangle = new FixedTriangle(new Vector3d(-4, 0, -4), new Vector3d(4, 0, -4), new Vector3d(0, 0, 4));
        Vector3d normal = rotation.Rotate(Vector3d.Up);
        Assert.True(WideTriangleRelations.TryGetSegmentIntersection(triangle, frame,
            new FixedSegment(origin - normal, origin + normal), out Vector3d first, out Vector3d second));
        Assert.Equal(origin, first);
        Assert.Equal(first, second);
        Assert.False(WideTriangleRelations.TryGetSegmentIntersection(triangle, frame,
            new FixedSegment(origin + normal, origin + normal * Fixed64.Two), out _, out _));
    }

    [Fact]
    public void CoplanarCrossing_ShouldClipToFiniteScaledEdges()
    {
        Vector3d origin = new(3, 2, -1);
        Vector3d scale = new(Fixed64.FromFraction(2, 3), Fixed64.One, Fixed64.One);
        var frame = new ScaledRigidFrame3d(origin, FixedQuaternion.Identity, Vector3d.Zero,
            scale, FixedQuaternion.Identity, Vector3d.Zero, Vector3d.One);
        var triangle = new FixedTriangle(new Vector3d(-4, 0, -4), new Vector3d(4, 0, -4), new Vector3d(0, 0, 4));
        Assert.True(WideTriangleRelations.TryGetSegmentIntersection(triangle, frame,
            new FixedSegment(origin + new Vector3d(-10, 0, 0), origin + new Vector3d(10, 0, 0)),
            out Vector3d first, out Vector3d second));
        Assert.Equal(origin + new Vector3d(-scale.X * Fixed64.Two, Fixed64.Zero, Fixed64.Zero), first);
        Assert.Equal(origin + new Vector3d(scale.X * Fixed64.Two, Fixed64.Zero, Fixed64.Zero), second);
        Assert.False(WideTriangleRelations.TryGetSegmentIntersection(triangle, frame,
            new FixedSegment(origin + new Vector3d(-10, 1, 0), origin + new Vector3d(10, 1, 0)), out _, out _));
    }

    [Theory]
    [InlineData(0)]
    [InlineData(45)]
    public void ClosestPoint_ShouldRankFiniteFeaturesBeforeFinalMaterialization(int angle)
    {
        var rotation = FixedQuaternion.FromEulerAnglesInDegrees(Fixed64.Zero, Fixed64.Zero, (Fixed64)angle);
        Vector3d origin = new(3, 2, -1);
        var frame = new ScaledRigidFrame3d(origin, rotation, Vector3d.Zero,
            new Vector3d(Fixed64.FromFraction(2, 3), Fixed64.One, Fixed64.FromFraction(3, 5)),
            rotation, Vector3d.Zero, Vector3d.One);
        var triangle = new FixedTriangle(new Vector3d(-4, 0, -4), new Vector3d(4, 0, -4), new Vector3d(0, 0, 4));
        Vector3d query = origin + rotation.Rotate(Vector3d.Up);
        Assert.True(WideTriangleRelations.TryGetClosestPointAnchor(triangle, frame, query, origin, out FixedPointAnchor point));
        Assert.True(point.TryGetPoint(out Vector3d world));
        for (int axis = 0; axis < 3; axis++)
            Assert.InRange(Math.Abs(world[axis].m_rawValue - origin[axis].m_rawValue), 0L, 1L);

        // The far point is closest to the apex, rather than to the infinite face plane.
        query = origin + rotation.Rotate(new Vector3d(0, 1, 10));
        Assert.True(WideTriangleRelations.TryGetClosestPointAnchor(triangle, frame, query, origin, out point));
        Assert.True(point.TryGetPoint(out world));
        Assert.True(frame.TryGetPoint(triangle.C, out Vector3d expected));
        Assert.Equal(expected, world);
    }
}
