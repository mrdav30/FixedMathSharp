using System;
using FixedMathSharp.Geometry;
using Xunit;

namespace FixedMathSharp.Tests.Geometry;

public class CurvedPrismBoundaryLineTests
{
    [Theory]
    [InlineData(false)]
    [InlineData(true)]
    public void ConeSourceClipCannotAdmitASegmentOutsideItsFiniteHeight(bool above)
    {
        var cone = new ConeFiniteSectionFrame(Fixed64.Two, Fixed64.Half, Signed320.ExtendValue(Signed192.Signed(1)));
        WideAxis3 start = new(default, Signed320.ExtendValue(Signed192.Raw((Fixed64)(above ? 5 : -1))), default);
        WideAxis3 end = new(default, Signed320.ExtendValue(Signed192.Raw((Fixed64)(above ? 6 : -2))), default);
        Assert.False(cone.IntersectsTransformedSegment(start, end, Interval(1, 4, 3, 4)));
    }

    [Theory]
    [InlineData(false)]
    [InlineData(true)]
    public void AxialSectionAdmissionIntersectsBothShapeCapsWithTheOriginalSourceClip(bool cone)
    {
        Signed320 one = Signed320.ExtendValue(Signed192.Signed(1));
        // These owners use doubled local coordinates: a height-two cylinder
        // spans [-2,2], while the cone's downward apex chart spans [0,4].
        WideAxis3 start = new(default, Raw(cone ? -2 : -4), default);
        WideAxis3 end = new(default, Raw(cone ? 6 : 4), default);
        var cylinder = new FiniteSlabSectionFrame(Fixed64.Two, Fixed64.Half, one);
        var finiteCone = new ConeFiniteSectionFrame(Fixed64.Two, Fixed64.Half, one);
        Assert.True(Intersects(Interval(1, 8, 7, 8)));
        Assert.True(Intersects(Interval(3, 8, 5, 8)));
        Assert.False(Intersects(Interval(0, 1, 1, 8)));
        Assert.False(Intersects(Interval(7, 8, 1, 1)));

        bool Intersects(RationalSegmentInterval clip) => cone
            ? finiteCone.IntersectsTransformedSegment(start, end, clip)
            : cylinder.IntersectsTransformedSegment(start, end, clip);
        static Signed320 Raw(int value) => Signed320.ExtendValue(Signed192.Raw((Fixed64)value));
    }

    [Theory]
    [InlineData(false)]
    [InlineData(true)]
    public void RealPrismEdges_RequireIntersectionWithTheRetainedFiniteBody(bool cone)
    {
        Vector2d[] footprint = { new(-2, -2), new(2, -2), new(2, 0), new(-2, 0) };
        Span<WideDirection3d> vertices = stackalloc WideDirection3d[8];
        var geometry = new CurvedPrismContactGeometry(new(Fixed64.Zero, Fixed64.Zero, Fixed64.Quarter),
            FixedQuaternion.Identity, Vector3d.Up, Fixed64.Two, Fixed64.Half,
            Vector3d.Zero, Fixed64.Zero, footprint, Fixed64.One, cone, vertices);
        // Original edge 6 is the bottom front edge at y=-1. Both shapes
        // include its x=1/4 section, but its right-hand retained tail is disjoint.
        Assert.True(CurvedPrismContact.IntersectsRealEdge(geometry, 6, default));
        Assert.True(CurvedPrismContact.IntersectsRealEdge(geometry, 6, Interval(13, 32, 7, 16)));
        Assert.False(CurvedPrismContact.IntersectsRealEdge(geometry, 6, Interval(1, 4, 5, 16)));
        // The cylinder retains the corresponding top section; the cone apex
        // is at z=1/4 and therefore does not touch the front edge at z=0.
        Assert.Equal(!cone, CurvedPrismContact.IntersectsRealEdge(geometry, 7, default));
        Assert.False(CurvedPrismContact.IntersectsRealEdge(geometry, 8, default));
    }

    [Theory]
    [InlineData(false)]
    [InlineData(true)]
    public void VerticalSourceClip_IntersectsHeightBeforeTheRadialQuadratic(bool cone)
    {
        Vector2d[] footprint = { new(-2, -2), new(2, -2), new(2, 0), new(-2, 0) };
        Span<WideDirection3d> vertices = stackalloc WideDirection3d[8];
        var geometry = new CurvedPrismContactGeometry(new(Fixed64.Zero, Fixed64.Zero, Fixed64.Quarter),
            FixedQuaternion.Identity, Vector3d.Up, Fixed64.Two, Fixed64.Half,
            Vector3d.Zero, Fixed64.Zero, footprint, Fixed64.Two, cone, vertices);
        // A vertical boundary through x=1/4,z=0 has finite body height [-1,1].
        // Retain a slice wholly before/after that band, then a slice which
        // encompasses the full band. Source clipping must precede admission.
        var cut = Interval(7, 16, 7, 16);
        Assert.True(CurvedPrismContact.IntersectsBoundaryLine(geometry, 2, cut, -Fixed64.Two, Fixed64.Two));
        Assert.False(CurvedPrismContact.IntersectsBoundaryLine(geometry, 2, cut, -Fixed64.Two, -Fixed64.One - Fixed64.Quarter));
        Assert.False(CurvedPrismContact.IntersectsBoundaryLine(geometry, 2, cut, Fixed64.One + Fixed64.Quarter, Fixed64.Two));
    }
    [Theory]
    [InlineData(false)]
    [InlineData(true)]
    public void HorizontalBoundaryLine_ClipsTheOriginalSourceParameter(bool cone)
    {
        Vector2d[] footprint = { new(-2, -2), new(2, -2), new(2, 0), new(-2, 0) };
        Span<WideDirection3d> vertices = stackalloc WideDirection3d[8];
        var geometry = new CurvedPrismContactGeometry(new(Fixed64.Zero, Fixed64.Zero, Fixed64.Quarter),
            FixedQuaternion.Identity, Vector3d.Up, Fixed64.Two, Fixed64.Half,
            Vector3d.Zero, Fixed64.Zero, footprint, Fixed64.Two, cone, vertices);
        // At y=-1, both primitives have a radius-1/2 section. The retained
        // x=[1/4,3/8] line crosses it; x=[3/4,1] does not.
        Assert.True(CurvedPrismContact.IntersectsBoundaryLine(geometry, 2, Interval(13, 32, 7, 16), -Fixed64.One, -Fixed64.One));
        Assert.False(CurvedPrismContact.IntersectsBoundaryLine(geometry, 2, Interval(1, 4, 5, 16), -Fixed64.One, -Fixed64.One));
        Assert.True(CurvedPrismContact.IntersectsBoundaryLine(geometry, 2, default, -Fixed64.One, -Fixed64.One));
        Assert.False(CurvedPrismContact.IntersectsBoundaryLine(geometry, 2, default, -Fixed64.Two, -Fixed64.Two));
    }

    [Theory]
    [InlineData(false)]
    [InlineData(true)]
    public void VerticalBoundaryLine_UsesTheFiniteBodyAtTheExactTangentCut(bool cone)
    {
        Vector2d[] footprint = { new(-2, -2), new(2, -2), new(2, 0), new(-2, 0) };
        Span<WideDirection3d> vertices = stackalloc WideDirection3d[8];
        var geometry = new CurvedPrismContactGeometry(new(Fixed64.Zero, Fixed64.Zero, Fixed64.Quarter),
            FixedQuaternion.Identity, Vector3d.Up, Fixed64.Two, Fixed64.Half,
            Vector3d.Zero, Fixed64.Zero, footprint, Fixed64.Two, cone, vertices);
        RationalSegmentInterval cut = Interval(7, 16, 7, 16); // x=1/4, z=0.
        Assert.True(CurvedPrismContact.IntersectsBoundaryLine(geometry, 2, cut, -Fixed64.Half, -Fixed64.Quarter));
        Assert.Equal(!cone, CurvedPrismContact.IntersectsBoundaryLine(geometry, 2, cut, Fixed64.Half, 3 * Fixed64.Quarter));
        Assert.False(CurvedPrismContact.IntersectsBoundaryLine(geometry, 2, cut, Fixed64.Two, (Fixed64)3));
    }

    [Theory]
    [InlineData(false)]
    [InlineData(true)]
    public void RationalVerticalBoundary_PreservesTheExactNativeChart(bool cone)
    {
        Vector2d[] footprint = { new(-2, -2), new(2, -2), new(2, 0), new(-2, 0) };
        FixedQuaternion rotation = new FixedQuaternion(Fixed64.One, Fixed64.Zero, Fixed64.Zero,
            Fixed64.Two + Fixed64.FromRaw(1)).Normalized;
        Span<WideDirection3d> vertices = stackalloc WideDirection3d[8];
        var geometry = new CurvedPrismContactGeometry(new(Fixed64.Zero, Fixed64.Zero, Fixed64.Quarter),
            rotation, Vector3d.Up, Fixed64.Two, Fixed64.Half,
            Vector3d.Zero, Fixed64.Zero, footprint, Fixed64.Two, cone, vertices);
        // The irreducible cut is 7/16 + 1/(16*2^128). No fixed lattice
        // point represents it; the quaternion chart must retain it through
        // exact finite-section admission.
        Signed320 n = new(0, 0, 7, 0, 1), d = new(0, 0, 16, 0, 0);
        var cut = new RationalSegmentInterval(n, d, n, d);
        Assert.True(CurvedPrismContact.IntersectsBoundaryLine(geometry, 2, cut, -Fixed64.Half, -Fixed64.Quarter));
        Assert.False(CurvedPrismContact.IntersectsBoundaryLine(geometry, 2, cut, Fixed64.Two, (Fixed64)3));
    }

    [Theory]
    [InlineData(false)]
    [InlineData(true)]
    public void NativeBoundaryIntervals_MapOppositeClippedEdgesWithoutRounding(bool reverse)
    {
        Fixed64 yaw = Fixed64.FromFraction(1, 7); Vector2d origin = new(3, -2);
        var first = reverse ? new FixedSegment2d(new(2, 0), new(-2, 0)) : new FixedSegment2d(new(-2, 0), new(2, 0));
        var second = reverse ? new FixedSegment2d(new(-1, 0), new(1, 0)) : new FixedSegment2d(new(1, 0), new(-1, 0));
        Assert.True(WideConvex2dRelations.TryIntersectBoundarySegments(origin, yaw, first, default, origin, yaw,
            second, Interval(1, 4, 3, 4), out var retained));
        Assert.Equal(0, RationalSegmentInterval.CompareParameters(retained.WideLowerNumerator, retained.WideLowerDenominator,
            Signed832.ExtendValue(Signed192.Signed(3)), Signed832.ExtendValue(Signed192.Signed(8))));
        Assert.Equal(0, RationalSegmentInterval.CompareParameters(retained.WideUpperNumerator, retained.WideUpperDenominator,
            Signed832.ExtendValue(Signed192.Signed(5)), Signed832.ExtendValue(Signed192.Signed(8))));
    }

    [Fact]
    public void NativeBoundaryIntervals_QualifyCrossingsAndRejectSeparatedOrClippedSources()
    {
        var source = new FixedSegment2d(new(-2, 0), new(2, 0));
        var crossing = new FixedSegment2d(new(0, -1), new(0, 1));
        Assert.True(WideConvex2dRelations.TryIntersectBoundarySegments(Vector2d.Zero, Fixed64.Zero, source, default,
            Vector2d.Zero, Fixed64.Zero, crossing, default, out var retained));
        Assert.Equal(0, RationalSegmentInterval.CompareParameters(retained.WideLowerNumerator, retained.WideLowerDenominator,
            Signed832.ExtendValue(Signed192.Signed(1)), Signed832.ExtendValue(Signed192.Signed(2))));
        Assert.Equal(0, RationalSegmentInterval.CompareParameters(retained.WideLowerNumerator, retained.WideLowerDenominator,
            retained.WideUpperNumerator, retained.WideUpperDenominator));
        Assert.False(WideConvex2dRelations.TryIntersectBoundarySegments(Vector2d.Zero, Fixed64.Zero, source, default,
            Vector2d.Zero, Fixed64.Zero, crossing, Interval(0, 1, 1, 4), out _));
        Assert.False(WideConvex2dRelations.TryIntersectBoundarySegments(Vector2d.Zero, Fixed64.Zero, source, default,
            Vector2d.Zero, Fixed64.Zero, new(new(-1, 1), new(1, 1)), default, out _));
        Assert.False(WideConvex2dRelations.TryIntersectBoundarySegments(Vector2d.Zero, Fixed64.Zero, source, default,
            Vector2d.Zero, Fixed64.Zero, new(new(3, 0), new(4, 0)), default, out _));
    }

    [Fact]
    public void NativeBoundaryIntervals_PreserveAnAliasedSourceClipForCollinearEdges()
    {
        var edge = new FixedSegment2d(new(-2, 0), new(2, 0));
        var retained = Interval(1, 4, 3, 4);
        Assert.True(WideConvex2dRelations.TryIntersectBoundarySegments(Vector2d.Zero, Fixed64.Zero, edge, retained,
            Vector2d.Zero, Fixed64.Zero, edge, default, out retained));
        Assert.Equal(0, RationalSegmentInterval.CompareParameters(retained.WideLowerNumerator, retained.WideLowerDenominator,
            Signed832.ExtendValue(Signed192.Signed(1)), Signed832.ExtendValue(Signed192.Signed(4))));
        Assert.Equal(0, RationalSegmentInterval.CompareParameters(retained.WideUpperNumerator, retained.WideUpperDenominator,
            Signed832.ExtendValue(Signed192.Signed(3)), Signed832.ExtendValue(Signed192.Signed(4))));
    }

    [Theory]
    [InlineData(false)]
    [InlineData(true)]
    public void NativeBoundaryIntervals_RejectCrossingsExcludedByAnAliasedClip(bool target)
    {
        var edge = new FixedSegment2d(new(-2, 0), new(2, 0));
        var crossing = new FixedSegment2d(new(0, -1), new(0, 1));
        var retained = Interval(0, 1, 1, 4);
        bool hit = target
            ? WideConvex2dRelations.TryIntersectBoundarySegments(Vector2d.Zero, Fixed64.Zero, edge, default,
                Vector2d.Zero, Fixed64.Zero, crossing, retained, out retained)
            : WideConvex2dRelations.TryIntersectBoundarySegments(Vector2d.Zero, Fixed64.Zero, edge, retained,
                Vector2d.Zero, Fixed64.Zero, crossing, default, out retained);
        Assert.False(hit); Assert.Equal(default, retained);
    }

    [Theory]
    [InlineData(false)]
    [InlineData(true)]
    public void NativeBoundaryIntervals_HandleReversedCrossingsAndEndpointRejection(bool reverse)
    {
        var source = new FixedSegment2d(new(-2, 0), new(2, 0));
        var crossing = reverse ? new FixedSegment2d(new(0, 1), new(0, -1)) : new FixedSegment2d(new(0, -1), new(0, 1));
        Assert.True(WideConvex2dRelations.TryIntersectBoundarySegments(Vector2d.Zero, Fixed64.Zero, source, default,
            Vector2d.Zero, Fixed64.Zero, crossing, Interval(1, 4, 3, 4), out var retained));
        Assert.Equal(0, RationalSegmentInterval.CompareParameters(retained.WideLowerNumerator, retained.WideLowerDenominator,
            Signed832.ExtendValue(Signed192.Signed(1)), Signed832.ExtendValue(Signed192.Signed(2))));
        // Both a crossing beyond the source endpoint and a crossing below the
        // target's first endpoint are outside the actual finite segments.
        Assert.False(WideConvex2dRelations.TryIntersectBoundarySegments(Vector2d.Zero, Fixed64.Zero, source, default,
            Vector2d.Zero, Fixed64.Zero, new(new(3, -1), new(3, 1)), default, out _));
        Assert.False(WideConvex2dRelations.TryIntersectBoundarySegments(Vector2d.Zero, Fixed64.Zero, source, default,
            Vector2d.Zero, Fixed64.Zero, new(new(0, 1), new(0, 2)), default, out _));
        Assert.False(WideConvex2dRelations.TryIntersectBoundarySegments(Vector2d.Zero, Fixed64.Zero, source, Interval(3, 4, 1, 1),
            Vector2d.Zero, Fixed64.Zero, crossing, default, out _));
        Assert.False(WideConvex2dRelations.TryIntersectBoundarySegments(Vector2d.Zero, Fixed64.Zero, source, default,
            Vector2d.Zero, Fixed64.Zero, crossing, Interval(3, 4, 1, 1), out _));
    }

    [Fact]
    public void NativeBoundaryIntervals_MapVerticalCollinearEdgesAndRetainTheSmallerWindow()
    {
        var source = new FixedSegment2d(new(0, 2), new(0, -2));
        var other = new FixedSegment2d(new(0, 1), new(0, -1));
        Assert.True(WideConvex2dRelations.TryIntersectBoundarySegments(Vector2d.Zero, Fixed64.Zero, source, Interval(1, 4, 3, 4),
            Vector2d.Zero, Fixed64.Zero, other, default, out var retained));
        Assert.Equal(0, RationalSegmentInterval.CompareParameters(retained.WideLowerNumerator, retained.WideLowerDenominator,
            Signed832.ExtendValue(Signed192.Signed(1)), Signed832.ExtendValue(Signed192.Signed(4))));
        Assert.Equal(0, RationalSegmentInterval.CompareParameters(retained.WideUpperNumerator, retained.WideUpperDenominator,
            Signed832.ExtendValue(Signed192.Signed(3)), Signed832.ExtendValue(Signed192.Signed(4))));
    }
    [Theory]
    [InlineData(false)]
    [InlineData(true)]
    public void NativeBoundaryIntervals_KeepTheSharedEndpointOfTranslatedCollinearEdges(bool reverse)
    {
        var edge = reverse ? new FixedSegment2d(new(-1, 1), new(1, 1)) : new FixedSegment2d(new(1, 1), new(-1, 1));
        Assert.True(WideConvex2dRelations.TryIntersectBoundarySegments(new(-1, -1), Fixed64.Zero, edge, default,
            new(1, -1), Fixed64.Zero, edge, default, out var retained));
        Signed832 expected = reverse ? Signed832.ExtendValue(Signed192.Signed(1)) : default;
        Signed832 one = Signed832.ExtendValue(Signed192.Signed(1));
        Assert.Equal(0, RationalSegmentInterval.CompareParameters(retained.WideLowerNumerator, retained.WideLowerDenominator, expected, one));
        Assert.Equal(0, RationalSegmentInterval.CompareParameters(retained.WideUpperNumerator, retained.WideUpperDenominator, expected, one));
        // A gap to the left or right must remain separated. In particular the
        // mapped interval entirely below zero cannot enter a nonnegative-only
        // unit-parameter comparison as though it were a positive magnitude.
        Assert.False(WideConvex2dRelations.TryIntersectBoundarySegments(new(-1, -1), Fixed64.Zero, edge, default,
            new(2, -1), Fixed64.Zero, edge, default, out _));
        Assert.False(WideConvex2dRelations.TryIntersectBoundarySegments(new(2, -1), Fixed64.Zero, edge, default,
            new(-1, -1), Fixed64.Zero, edge, default, out _));
    }
    private static RationalSegmentInterval Interval(int ln, int ld, int un, int ud) => new(
        Signed320.ExtendValue(Signed192.Signed(ln)), Signed320.ExtendValue(Signed192.Signed(ld)),
        Signed320.ExtendValue(Signed192.Signed(un)), Signed320.ExtendValue(Signed192.Signed(ud)));
}

