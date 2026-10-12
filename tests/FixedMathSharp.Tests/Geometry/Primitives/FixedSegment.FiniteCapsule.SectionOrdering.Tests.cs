using System;
using FixedMathSharp.Geometry;
using Xunit;

namespace FixedMathSharp.Tests.Geometry.Primitives;

public sealed class FixedSegmentFiniteCapsuleSectionOrderingTests
{
    [Theory]
    [InlineData(0, 0, true)]
    [InlineData(1, 0, true)]
    [InlineData(0, 2, true)]
    [InlineData(0, -2, true)]
    [InlineData(1, 2, false)]
    [InlineData(0, 3, false)]
    public void ClosedCapsule_AdmitsFiniteSideAndEndCaps(int x, int y, bool expected)
    {
        var frame = Frame(Fixed64.Two, Fixed64.One);
        Assert.Equal(expected, frame.ContainsPoint(new Vector3d(x, y, 0)));
    }

    [Theory]
    [InlineData(false)]
    [InlineData(true)]
    public void OneRawOutsideSideOrCap_IsSeparated(bool cap)
    {
        var frame = Frame(Fixed64.Two, Fixed64.One);
        Vector3d boundary = cap ? Vector3d.Up * Fixed64.Two : Vector3d.Right;
        Vector3d outward = cap ? Vector3d.Up : Vector3d.Right;
        Assert.True(frame.ContainsPoint(boundary));
        Assert.False(frame.ContainsPoint(boundary + outward * Fixed64.MinIncrement));
        Assert.False(frame.IntersectsSegment(new FixedSegment(boundary + outward * Fixed64.MinIncrement,
            boundary + outward * Fixed64.Half)));
    }

    [Theory]
    [InlineData(false)]
    [InlineData(true)]
    public void FirstSectionPoint_IsIndependentOfDirectionAndSubdivision(bool endpointCap)
    {
        var frame = Frame(Fixed64.Two, Fixed64.One);
        Fixed64 y = endpointCap ? Fixed64.One + Fixed64.Half : Fixed64.Zero;
        Fixed64 z = endpointCap ? Fixed64.Zero : Fixed64.Half;
        var full = new FixedSegment(new Vector3d(-Fixed64.Two, y, z), new Vector3d(Fixed64.Two, y, z));
        var piece = new FixedSegment(new Vector3d(-Fixed64.One, y, z), new Vector3d(Fixed64.Zero, y, z));
        Span<ulong> values = stackalloc ulong[2 * FiniteSectionPoint.StorageWords];
        Span<int> signs = stackalloc int[2 * FiniteSectionPoint.SignCount];
        var first = new FiniteSectionPoint(values[..FiniteSectionPoint.StorageWords], signs[..FiniteSectionPoint.SignCount]);
        var second = new FiniteSectionPoint(values[FiniteSectionPoint.StorageWords..], signs[FiniteSectionPoint.SignCount..]);
        Assert.True(FiniteSectionPoint.TryGetFirstSegmentPoint(full, frame, first));
        Assert.True(FiniteSectionPoint.TryGetFirstSegmentPoint(new FixedSegment(piece.End, piece.Start), frame, second));
        Assert.Equal(0, FiniteSectionPoint.CompareLexicographic(first, second));
        Fixed64 x = ContactQuadratic.RoundRatio(first.Coordinate(0), first.Denominator, first.Root);
        Fixed64 expected = -FixedMath.Sqrt(Fixed64.FromFraction(3, 4));
        Assert.InRange(x.m_rawValue, expected.m_rawValue - 1, expected.m_rawValue + 1);
    }

    [Theory]
    [InlineData(0)]
    [InlineData(2)]
    public void RepeatedRootTouch_IsAClosedZeroDimensionalSection(int length)
    {
        var frame = Frame((Fixed64)length, Fixed64.One);
        Fixed64 y = Fixed64.One + (Fixed64)length / 2;
        var segment = new FixedSegment(new Vector3d(-Fixed64.One, y, Fixed64.Zero), new Vector3d(Fixed64.One, y, Fixed64.Zero));
        Span<ulong> values = stackalloc ulong[FiniteSectionPoint.StorageWords];
        Span<int> signs = stackalloc int[FiniteSectionPoint.SignCount];
        var point = new FiniteSectionPoint(values, signs);
        Assert.True(frame.IntersectsSegment(segment));
        Assert.True(FiniteSectionPoint.TryGetFirstSegmentPoint(segment, frame, point));
        Assert.Equal(Fixed64.Zero, ContactQuadratic.RoundRatio(point.Coordinate(0), point.Denominator, point.Root));
        Assert.Equal(y, ContactQuadratic.RoundRatio(point.Coordinate(1), point.Denominator, point.Root));
    }

    [Fact]
    public void OddRawCore_UsesItsExactHalfLength()
    {
        var frame = Frame(Fixed64.FromRaw(5), Fixed64.Zero);
        Assert.True(frame.ContainsPoint(Vector3d.Up * Fixed64.FromRaw(2)));
        Assert.False(frame.ContainsPoint(Vector3d.Up * Fixed64.FromRaw(3)));
        Assert.True(frame.IntersectsSegment(new FixedSegment(Vector3d.Up * Fixed64.FromRaw(2), Vector3d.Up * Fixed64.FromRaw(3))));
        Assert.False(frame.IntersectsSegment(new FixedSegment(Vector3d.Up * Fixed64.FromRaw(3), Vector3d.Up * Fixed64.FromRaw(4))));
    }

    [Theory]
    [InlineData(false, -1)]
    [InlineData(false, 1)]
    [InlineData(true, -1)]
    [InlineData(true, 1)]
    public void SharedRigidFrame_PreservesExactTouchNearBothScalarLimits(bool high, int capSign)
    {
        Fixed64 coordinate = high ? Fixed64.MaxValue - (Fixed64)4 : Fixed64.MinValue + (Fixed64)4;
        Vector3d origin = new(coordinate, coordinate, coordinate);
        FixedQuaternion rotation = new FixedQuaternion(Fixed64.One, Fixed64.Two, (Fixed64)3, (Fixed64)4).Normalized;
        var frame = new CapsuleFiniteSectionFrame(origin, rotation, origin, rotation, Fixed64.Two, Fixed64.One);
        Assert.True(frame.ContainsPoint(Vector3d.Up * (Fixed64.Two * capSign)));
        Assert.False(frame.ContainsPoint(Vector3d.Up * ((Fixed64.Two + Fixed64.MinIncrement) * capSign)));
        Assert.True(frame.IntersectsSegment(new FixedSegment(new Vector3d(-1, 2 * capSign, 0), new Vector3d(1, 2 * capSign, 0))));
    }

    [Theory]
    [InlineData(-3, 3)]
    [InlineData(3, -3)]
    public void AxialCrossing_RetainsFirstCapInInputOrder(int startY, int endY)
    {
        var frame = Frame(Fixed64.Two, Fixed64.One);
        var segment = new FixedSegment(new Vector3d(0, startY, 0), new Vector3d(0, endY, 0));
        Span<ulong> values = stackalloc ulong[6 * FiniteSectionPoint.Words];
        Span<int> signs = stackalloc int[6];
        Span<ulong> root = stackalloc ulong[FiniteSectionPoint.RootWords];
        ContactQuadratic n = ContactQuadratic.At(values, signs, 0, FiniteSectionPoint.Words);
        ContactQuadratic d = ContactQuadratic.At(values, signs, 1, FiniteSectionPoint.Words);
        ContactQuadratic comparison = ContactQuadratic.At(values, signs, 2, FiniteSectionPoint.Words);
        Assert.True(frame.IntersectsSegment(segment));
        Assert.True(frame.TryGetFirstSegmentParameter(segment, n, d, root));
        // Each direction enters at its near cap after 1/6 of the segment.
        // The finite cylinder's later entry cannot replace that first point.
        ContactQuadratic.Scale(n, Signed576.ExtendValue(Signed320.ExtendValue(Signed192.Signed(6))), comparison);
        comparison.Add(d, -1);
        Assert.Equal(0, comparison.Sign(root));
    }

    [Theory]
    [InlineData(-4, 4, 4, 0, 0, 2, 1)]
    [InlineData(-4, 0, 4, 4, -4, 8, 5)]
    public void BodyStationaryPointOutsideAxialClip_StillRetainsTheFiniteCapSection(
        int startX, int startY, int endX, int endY, int pointX, int pointY, int divisor)
    {
        var frame = Frame(Fixed64.Two, Fixed64.One);
        var segment = new FixedSegment(new Vector3d(startX, startY, 0), new Vector3d(endX, endY, 0));
        Span<ulong> values = stackalloc ulong[FiniteSectionPoint.StorageWords];
        Span<int> signs = stackalloc int[FiniteSectionPoint.SignCount];
        var point = new FiniteSectionPoint(values, signs);
        Assert.True(frame.IntersectsSegment(segment));
        Assert.True(FiniteSectionPoint.TryGetFirstSegmentPoint(segment, frame, point));
        Fixed64 x = ContactQuadratic.RoundRatio(point.Coordinate(0), point.Denominator, point.Root);
        Fixed64 y = ContactQuadratic.RoundRatio(point.Coordinate(1), point.Denominator, point.Root);
        // Exact intersections are (0,2) and (-4/5,8/5), respectively.
        Assert.InRange(x.m_rawValue, Fixed64.FromFraction(pointX, divisor).m_rawValue - 1,
            Fixed64.FromFraction(pointX, divisor).m_rawValue + 1);
        Assert.InRange(y.m_rawValue, Fixed64.FromFraction(pointY, divisor).m_rawValue - 1,
            Fixed64.FromFraction(pointY, divisor).m_rawValue + 1);
    }

    [Theory]
    [InlineData(2, -3, 2, 3)]
    [InlineData(2, 0, 3, 0)]
    [InlineData(0, 3, 0, 4)]
    [InlineData(0, -4, 0, -3)]
    [InlineData(-4, 5, 4, 1)]
    [InlineData(-4, -5, 4, -1)]
    public void DisjointFiniteSegments_HaveNoAdmittedSection(int startX, int startY, int endX, int endY)
    {
        var frame = Frame(Fixed64.Two, Fixed64.One);
        var segment = new FixedSegment(new Vector3d(startX, startY, 0), new Vector3d(endX, endY, 0));
        Span<ulong> values = stackalloc ulong[FiniteSectionPoint.StorageWords];
        Span<int> signs = stackalloc int[FiniteSectionPoint.SignCount];
        var point = new FiniteSectionPoint(values, signs);
        Assert.False(frame.IntersectsSegment(segment));
        Assert.False(FiniteSectionPoint.TryGetFirstSegmentPoint(segment, frame, point));
    }

    [Fact]
    public void SideTouchAtSegmentEnd_IsClosedAndAnInteriorStartRemainsFirst()
    {
        var frame = Frame(Fixed64.Two, Fixed64.One);
        Assert.True(frame.IntersectsSegment(new FixedSegment(new Vector3d(2, 0, 0), Vector3d.Right)));
        Span<ulong> values = stackalloc ulong[FiniteSectionPoint.StorageWords];
        Span<int> signs = stackalloc int[FiniteSectionPoint.SignCount];
        var point = new FiniteSectionPoint(values, signs);
        Assert.True(FiniteSectionPoint.TryGetFirstSegmentPoint(new FixedSegment(Vector3d.Up * (Fixed64)3, Vector3d.Zero), frame, point));
        for (int axis = 0; axis < 3; axis++)
            Assert.Equal(Fixed64.Zero, ContactQuadratic.RoundRatio(point.Coordinate(axis), point.Denominator, point.Root));
    }

    [Theory]
    [InlineData(0)]
    [InlineData(1)]
    [InlineData(2)]
    [InlineData(3)]
    public void InvalidGeometry_RejectsTheOwningParameter(int invalid)
    {
        ArgumentException exception = Assert.ThrowsAny<ArgumentException>(() =>
            new CapsuleFiniteSectionFrame(Vector3d.Zero, invalid == 2 ? default : FixedQuaternion.Identity,
                Vector3d.Zero, invalid == 3 ? default : FixedQuaternion.Identity,
                invalid == 0 ? -Fixed64.One : Fixed64.Two, invalid == 1 ? -Fixed64.One : Fixed64.One));
        Assert.Equal(new[] { "length", "radius", "rotation", "capsuleRotation" }[invalid], exception.ParamName);
    }

    private static CapsuleFiniteSectionFrame Frame(Fixed64 length, Fixed64 radius) =>
        new(Vector3d.Zero, FixedQuaternion.Identity, Vector3d.Zero, FixedQuaternion.Identity, length, radius);
}
