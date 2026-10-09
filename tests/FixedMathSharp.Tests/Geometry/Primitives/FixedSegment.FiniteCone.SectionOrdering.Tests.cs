using System;
using FixedMathSharp.Geometry;
using Xunit;

namespace FixedMathSharp.Tests.Geometry;

public sealed class FixedSegmentFiniteConeSectionOrderingTests
{
    [Theory]
    [InlineData(false)]
    [InlineData(true)]
    public void SectionEndpoint_RemainsExactAcrossSegmentReversalAndSubdivision(bool reverse)
    {
        var frame = Frame();
        var full = new FixedSegment(new Vector3d(-2, 0, 0), new Vector3d(2, 0, 0));
        var piece = new FixedSegment(new Vector3d(-2, 0, 0), new Vector3d(-Fixed64.Half, Fixed64.Zero, Fixed64.Zero));
        if (reverse) full = new FixedSegment(full.End, full.Start);
        Span<ulong> values = stackalloc ulong[2 * ConeSectionPoint.StorageWords];
        Span<int> signs = stackalloc int[2 * ConeSectionPoint.SignCount];
        var first = new ConeSectionPoint(values[..ConeSectionPoint.StorageWords], signs[..ConeSectionPoint.SignCount]);
        var second = new ConeSectionPoint(values[ConeSectionPoint.StorageWords..], signs[ConeSectionPoint.SignCount..]);
        Assert.True(ConeSectionPoint.TryGetFirstSegmentPoint(full, frame, first));
        Assert.True(ConeSectionPoint.TryGetFirstSegmentPoint(piece, frame, second));
        Assert.Equal(0, ConeSectionPoint.CompareLexicographic(first, second));
        Assert.Equal(-Fixed64.One.m_rawValue, ContactQuadratic.RoundRatio(first.Coordinate(0), first.Denominator, first.Root).m_rawValue);
    }

    [Fact]
    public void SectionEndpoint_OrdersIndependentIrrationalBoundariesBeforeRounding()
    {
        var frame = Frame();
        var diagonal = new FixedSegment(new Vector3d(-2, 0, -2), new Vector3d(2, 0, 2));
        var axis = new FixedSegment(new Vector3d(-2, 0, 0), new Vector3d(2, 0, 0));
        Span<ulong> values = stackalloc ulong[2 * ConeSectionPoint.StorageWords];
        Span<int> signs = stackalloc int[2 * ConeSectionPoint.SignCount];
        var first = new ConeSectionPoint(values[..ConeSectionPoint.StorageWords], signs[..ConeSectionPoint.SignCount]);
        var second = new ConeSectionPoint(values[ConeSectionPoint.StorageWords..], signs[ConeSectionPoint.SignCount..]);
        Assert.True(ConeSectionPoint.TryGetFirstSegmentPoint(diagonal, frame, first));
        Assert.True(ConeSectionPoint.TryGetFirstSegmentPoint(axis, frame, second));
        Assert.Equal(1, ConeSectionPoint.CompareLexicographic(first, second));
        Assert.Equal(-1, ConeSectionPoint.CompareLexicographic(second, first));
    }

    [Theory]
    [InlineData(0L, true)]
    [InlineData(1L, false)]
    public void SectionEndpoint_RetainsExactTangencyAndRejectsRawNeighbor(long gap, bool expected)
    {
        Fixed64 x = Fixed64.One + Fixed64.FromRaw(gap);
        var segment = new FixedSegment(new Vector3d(x, Fixed64.Zero, -Fixed64.One), new Vector3d(x, Fixed64.Zero, Fixed64.One));
        Span<ulong> values = stackalloc ulong[ConeSectionPoint.StorageWords];
        Span<int> signs = stackalloc int[ConeSectionPoint.SignCount];
        var point = new ConeSectionPoint(values, signs);
        Assert.Equal(expected, ConeSectionPoint.TryGetFirstSegmentPoint(segment, Frame(), point));
        if (expected)
        {
            Assert.True(point.Denominator.Sign(point.Root) > 0);
            Assert.Equal(Fixed64.One, ContactQuadratic.RoundRatio(point.Coordinate(0), point.Denominator, point.Root));
            Assert.Equal(Fixed64.Zero, ContactQuadratic.RoundRatio(point.Coordinate(2), point.Denominator, point.Root));
            Span<ulong> copiedValues = stackalloc ulong[ConeSectionPoint.StorageWords];
            Span<int> copiedSigns = stackalloc int[ConeSectionPoint.SignCount];
            var copy = new ConeSectionPoint(copiedValues, copiedSigns);
            point.CopyTo(copy);
            Assert.Equal(0, ConeSectionPoint.CompareLexicographic(point, copy));
        }
    }

    private static ConeFiniteSectionFrame Frame() => new(Vector3d.Zero, FixedQuaternion.Identity,
        Vector3d.Zero, FixedQuaternion.Identity, (Fixed64)4, (Fixed64)2);

    [Theory]
    [InlineData(false)]
    [InlineData(true)]
    public void SectionEndpoint_ClipsLinearAndConcaveLateralCrossings(bool linear)
    {
        var segment = linear
            ? new FixedSegment(new Vector3d(-1, 3, 0), new Vector3d(Fixed64.Zero, Fixed64.One, Fixed64.Zero))
            : new FixedSegment(new Vector3d(-Fixed64.Half, (Fixed64)3, Fixed64.Zero), new Vector3d(Fixed64.Half, Fixed64.Zero, Fixed64.Zero));
        Span<ulong> values = stackalloc ulong[ConeSectionPoint.StorageWords];
        Span<int> signs = stackalloc int[ConeSectionPoint.SignCount];
        var point = new ConeSectionPoint(values, signs);
        Assert.True(ConeSectionPoint.TryGetFirstSegmentPoint(segment, Frame(), point));
        Fixed64 x = ContactQuadratic.RoundRatio(point.Coordinate(0), point.Denominator, point.Root);
        Fixed64 y = ContactQuadratic.RoundRatio(point.Coordinate(1), point.Denominator, point.Root);
        Assert.Equal(linear ? -(Fixed64)1 / 4 : -(Fixed64)1 / 10, x);
        Assert.Equal(linear ? (Fixed64)3 / 2 : (Fixed64)9 / 5, y);
    }

    [Theory]
    [InlineData(1)]
    [InlineData(2)]
    public void SectionEndpoint_UsesAllAuthoredCoordinateAxes(int axis)
    {
        Vector3d low = Vector3d.Zero, high = Vector3d.Zero;
        low[axis] = -Fixed64.Half; high[axis] = Fixed64.Half;
        Span<ulong> values = stackalloc ulong[2 * ConeSectionPoint.StorageWords];
        Span<int> signs = stackalloc int[2 * ConeSectionPoint.SignCount];
        var first = new ConeSectionPoint(values[..ConeSectionPoint.StorageWords], signs[..ConeSectionPoint.SignCount]);
        var second = new ConeSectionPoint(values[ConeSectionPoint.StorageWords..], signs[ConeSectionPoint.SignCount..]);
        Assert.True(ConeSectionPoint.TryGetFirstSegmentPoint(new FixedSegment(high, low), Frame(), first));
        Assert.True(ConeSectionPoint.TryGetFirstSegmentPoint(new FixedSegment(high, high), Frame(), second));
        Assert.Equal(-1, ConeSectionPoint.CompareLexicographic(first, second));
        Assert.Equal(-Fixed64.Half, ContactQuadratic.RoundRatio(first.Coordinate(axis), first.Denominator, first.Root));
    }

    [Fact]
    public void SectionEndpoint_ClipsAxialExtentOfZeroRadiusCone()
    {
        var frame = new ConeFiniteSectionFrame(Vector3d.Zero, FixedQuaternion.Identity,
            Vector3d.Zero, FixedQuaternion.Identity, (Fixed64)4, Fixed64.Zero);
        Span<ulong> values = stackalloc ulong[ConeSectionPoint.StorageWords];
        Span<int> signs = stackalloc int[ConeSectionPoint.SignCount];
        var point = new ConeSectionPoint(values, signs);
        Assert.True(ConeSectionPoint.TryGetFirstSegmentPoint(new FixedSegment(new Vector3d(0, -3, 0), new Vector3d(0, 3, 0)), frame, point));
        Assert.Equal(-(Fixed64)2, ContactQuadratic.RoundRatio(point.Coordinate(1), point.Denominator, point.Root));
    }

    [Fact]
    public void SectionFrame_RejectsNegativeRadiusAndUnnormalizedRigidFrames()
    {
        Assert.Throws<ArgumentOutOfRangeException>(() => new ConeFiniteSectionFrame(Vector3d.Zero,
            FixedQuaternion.Identity, Vector3d.Zero, FixedQuaternion.Identity, (Fixed64)4, -Fixed64.One));
        Assert.Throws<ArgumentException>(() => new ConeFiniteSectionFrame(Vector3d.Zero,
            default, Vector3d.Zero, FixedQuaternion.Identity, (Fixed64)4, Fixed64.Two));
        Assert.Throws<ArgumentException>(() => new ConeFiniteSectionFrame(Vector3d.Zero,
            FixedQuaternion.Identity, Vector3d.Zero, default, (Fixed64)4, Fixed64.Two));
    }

    [Theory]
    [InlineData(-3)]
    [InlineData(3)]
    public void SectionParameter_RejectsStationaryPointOutsideClosedAxialClip(int endX)
    {
        // Only t=1 reaches the base plane, where |X|=3 exceeds radius 2.
        // The convex radial stationary point lies above or below that clip;
        // using it without finite-height admission would create a false hit.
        var segment = new FixedSegment(new Vector3d(-5, -5, 0), new Vector3d(endX, -2, 0));
        Span<ulong> values = stackalloc ulong[4 * ConeSectionPoint.Words];
        Span<int> signs = stackalloc int[4];
        Span<ulong> root = stackalloc ulong[ConeSectionPoint.RootWords];
        Assert.False(ConeSectionPoint.TryGetSegmentParameter(segment, Frame(), false,
            ContactQuadratic.At(values, signs, 0, ConeSectionPoint.Words),
            ContactQuadratic.At(values, signs, 1, ConeSectionPoint.Words), root));
    }

    [Theory]
    [InlineData(false, false)]
    [InlineData(false, true)]
    [InlineData(true, false)]
    [InlineData(true, true)]
    public void SectionParameter_RetainsNondyadicSingletonInEitherInputOrder(bool upper, bool reverse)
    {
        var segment = new FixedSegment(new Vector3d(-1,0,1), new Vector3d(2,0,1));
        if (reverse) segment = new FixedSegment(segment.End, segment.Start);
        Span<ulong> values = stackalloc ulong[6 * ConeSectionPoint.Words];
        Span<int> signs = stackalloc int[6];
        Span<ulong> root = stackalloc ulong[ConeSectionPoint.RootWords];
        ContactQuadratic n = ContactQuadratic.At(values, signs, 0, ConeSectionPoint.Words);
        ContactQuadratic d = ContactQuadratic.At(values, signs, 1, ConeSectionPoint.Words);
        ContactQuadratic query = ContactQuadratic.At(values, signs, 2, ConeSectionPoint.Words);
        Assert.True(ConeSectionPoint.TryGetSegmentParameter(segment, Frame(), upper, n, d, root));
        ContactQuadratic.Scale(n, Signed576.ExtendValue(Signed320.ExtendValue(Signed192.Signed(3))), query);
        query.Add(d, -1);
        if (reverse) query.Add(d, -1);
        Assert.Equal(0, query.Sign(root));
        Assert.True(d.Sign(root) > 0);
    }

    [Theory]
    [InlineData(false)]
    [InlineData(true)]
    public void SectionParameter_RetainsBothIrrationalEndpointsOfClosedSection(bool upper)
    {
        var segment = new FixedSegment(new Vector3d(-2,0,-2), new Vector3d(2,0,2));
        Span<ulong> values = stackalloc ulong[10 * ConeSectionPoint.Words];
        Span<int> signs = stackalloc int[10];
        Span<ulong> root = stackalloc ulong[ConeSectionPoint.RootWords];
        ContactQuadratic n = ContactQuadratic.At(values, signs, 0, ConeSectionPoint.Words), d = ContactQuadratic.At(values, signs, 1, ConeSectionPoint.Words);
        ContactQuadratic term = ContactQuadratic.At(values, signs, 2, ConeSectionPoint.Words), query = ContactQuadratic.At(values, signs, 3, ConeSectionPoint.Words);
        ContactQuadratic scaled = ContactQuadratic.At(values, signs, 4, ConeSectionPoint.Words);
        Assert.True(ConeSectionPoint.TryGetSegmentParameter(segment, Frame(), upper, n, d, root));
        // The independent authored-section equation is 32t²-32t+7=0.
        ContactQuadratic.Multiply(n, n, root, term); ContactQuadratic.Scale(term, Signed576.ExtendValue(Signed320.ExtendValue(Signed192.Signed(32))), query);
        ContactQuadratic.Multiply(n, d, root, term); ContactQuadratic.Scale(term, Signed576.ExtendValue(Signed320.ExtendValue(Signed192.Signed(32))), scaled); query.Add(scaled, -1);
        ContactQuadratic.Multiply(d, d, root, term); ContactQuadratic.Scale(term, Signed576.ExtendValue(Signed320.ExtendValue(Signed192.Signed(7))), scaled); query.Add(scaled);
        Assert.Equal(0, query.Sign(root));
        n.CopyTo(query); query.Add(n); query.Add(d, -1);
        Assert.Equal(upper ? 1 : -1, query.Sign(root));
    }

    [Theory]
    [InlineData(false)]
    [InlineData(true)]
    public void SectionParameter_UpperExitSupportsConcaveAndLinearCharts(bool linear)
    {
        var segment = linear
            ? new FixedSegment(new Vector3d(0,1,0), new Vector3d(-1,3,0))
            : new FixedSegment(new Vector3d(Fixed64.Half, Fixed64.Zero, Fixed64.Zero), new Vector3d(-Fixed64.Half, (Fixed64)3, Fixed64.Zero));
        Span<ulong> values = stackalloc ulong[6 * ConeSectionPoint.Words];
        Span<int> signs = stackalloc int[6];
        Span<ulong> root = stackalloc ulong[ConeSectionPoint.RootWords];
        ContactQuadratic n = ContactQuadratic.At(values, signs, 0, ConeSectionPoint.Words), d = ContactQuadratic.At(values, signs, 1, ConeSectionPoint.Words);
        ContactQuadratic query = ContactQuadratic.At(values, signs, 2, ConeSectionPoint.Words);
        Assert.True(ConeSectionPoint.TryGetSegmentParameter(segment, Frame(), true, n, d, root));
        ContactQuadratic.Scale(n, Signed576.ExtendValue(Signed320.ExtendValue(Signed192.Signed(linear ? 4 : 5))), query);
        for (int i = 0; i < (linear ? 1 : 3); i++) query.Add(d, -1);
        Assert.Equal(0, query.Sign(root));
    }
}
