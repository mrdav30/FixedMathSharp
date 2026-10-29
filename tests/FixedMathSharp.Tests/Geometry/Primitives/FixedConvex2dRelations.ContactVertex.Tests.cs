using FixedMathSharp.Geometry;
using System;
using Xunit;

namespace FixedMathSharp.Tests.Bounds;

public sealed class FixedConvex2dContactVertexTests
{
    [Fact]
    public void CompositeBoundaryWitness_ShouldPreserveDisplacementAndSubRawTerms()
    {
        var composite = new FixedPointAnchor2d(Vector2d.Zero, Fixed64.Quarter,
            Vector2d.Right, Vector2d.Right);
        Assert.Equal(0, WideConvex2dRelations.LocateBoundaryPoint(composite, Vector2d.Zero,
            Fixed64.Quarter, new(Vector2d.Zero, new(4, 0)), default));
        Assert.Equal(2, WideConvex2dRelations.LocateBoundaryPoint(composite, Vector2d.Zero,
            Fixed64.Quarter, new(Vector2d.Zero, new(2, 0)), default));
        Assert.Equal(1, WideConvex2dRelations.LocateBoundaryPoint(composite, Vector2d.Zero,
            Fixed64.Quarter, new(new(2, 0), new(4, 0)), default));
        var exact = WideFiniteAxisIntersection.GetCenteredCapsuleSupportAnchor(Vector2d.Zero,
            Fixed64.Zero, Vector2d.Right, Fixed64.FromRaw(3), Fixed64.Half, Vector2d.Right);
        Fixed64 rounded = Fixed64.Half + Fixed64.FromRaw(2);
        Assert.Equal(-1, WideConvex2dRelations.LocateBoundaryPoint(exact, Vector2d.Zero,
            Fixed64.Zero, new(new(rounded, Fixed64.Zero), new(rounded + Fixed64.One, Fixed64.Zero)), default));
        RationalPoint3d spatial = RationalPoint3d.FromAnchor(composite);
        var frame = new NativePrismYawFrame3d(Vector3d.Zero, Fixed64.Quarter);
        Assert.Equal(0, RationalPoint3d.CompareProjection(spatial, frame.GetPoint(new(2, 0, 0)), Vector3d.Right));
        Assert.Equal(0, RationalPoint3d.CompareProjection(spatial, frame.GetPoint(new(2, 0, 0)), Vector3d.Forward));
    }

    [Theory]
    [InlineData(0, true)] [InlineData(1, false)] [InlineData(2, false)] [InlineData(-2, true)]
    public void ExactFanBoundary_ShouldRetainTheOriginalAxisSign(int rawX, bool admitted)
    {
        var witness = Anchor(Vector2d.Zero);
        FixedPointAnchor2d[] fan = { Anchor(new(1, -1)) };
        WideDirection3d direction = new(Signed320.ExtendValue(Signed192.Signed(rawX)), default(Signed320), default(Signed320));
        Assert.Equal(admitted, WideConvex2dRelations.SupportsBoundaryFan(direction, witness, fan));
    }

    [Theory]
    [InlineData(-2, 0, 2, 0, 2)] [InlineData(2, 0, -2, 0, 2)]
    [InlineData(2, 3, -2, -2, 2)] [InlineData(2, -3, -2, 2, 2)]
    [InlineData(0, 0, 0, 0, 2)] [InlineData(2, 3, -2, -3, 0)]
    public void ClosestFiniteCoreAxis_ShouldRemainExactAcrossEndpointsAndInterior(int x, int y, int dx, int dy, int length)
    {
        Vector2d origin = new(7, -3);
        var point = new FixedPointAnchor2d(origin, Fixed64.Quarter, new(x, y));
        WideDirection3d actual = WideCenteredCapsule2dRelations.GetPointToCoreDirection(origin,
            Fixed64.Quarter, (Fixed64)length, point);
        var frame = new NativePrismYawFrame3d(new(7, 0, -3), Fixed64.Quarter);
        WideDirection3d expected = frame.GetEdgeDirection(Vector3d.Zero, new(dx, 0, dy));
        Assert.True(WideDirection3d.Cross(actual, expected).IsZero);
        Assert.Equal(expected.IsZero, actual.IsZero);
        if (!actual.IsZero) Assert.True(WideDirection3d.Dot(actual, expected).Sign > 0);
    }

    [Fact]
    public void ExactFlatFan_ShouldRejectATangentialAxisThatNormalizesToZero()
    {
        Vector2d center = new(Fixed64.FromRaw(1), (Fixed64)1_000_000_000);
        Fixed64 radius = (Fixed64)1_000_000_001;
        var vertex = Anchor(Vector2d.Zero);
        FixedPointAnchor2d[] fan = { Anchor(Vector2d.Right), Anchor(-Vector2d.Right), Anchor(-Vector2d.Forward) };
        Span<FixedPointAnchor2d> first = stackalloc FixedPointAnchor2d[2], second = stackalloc FixedPointAnchor2d[2];
        Assert.True(WideCenteredCapsule2dRelations.TryGetContacts(center, Fixed64.Zero, Vector2d.Forward,
            Fixed64.Zero, radius, Vector2d.Zero, Fixed64.Zero, new[] { Vector2d.Zero }, first, second,
            out _, out Vector2d rounded, out _, out _));
        Assert.Equal(Fixed64.Zero, rounded.X);
        Assert.False(WideCenteredCapsule2dRelations.TryGetBoundaryVertexContact(center, Fixed64.Zero,
            Fixed64.Zero, radius, vertex, fan, out _));
    }

    [Theory]
    [InlineData(false)]
    [InlineData(true)]
    public void TrueConvexCorner_ShouldRetainCircleAndCapsuleWitnesses(bool core)
    {
        var vertex = Anchor(Vector2d.Zero);
        FixedPointAnchor2d[] fan = { Anchor(Vector2d.Right), Anchor(Vector2d.Forward) };
        Vector2d center = new(-Fixed64.Quarter, -Fixed64.Quarter);
        Assert.True(WideCenteredCapsule2dRelations.TryGetBoundaryVertexContact(center, Fixed64.Zero,
            core ? Fixed64.One : Fixed64.Zero, Fixed64.Half, vertex, fan, out var contact));
        Assert.True(contact.Normal.X > Fixed64.Zero);
        Assert.True(contact.Normal.Y >= Fixed64.Zero);
        Assert.True(contact.Depth > Fixed64.Zero);
        Assert.Equal(vertex, contact.SecondAnchor);
        Assert.True(contact.FirstAnchor.TryGetPoint(out Vector2d point));
        Assert.True(point.X >= center.X);
    }

    [Fact]
    public void ConcaveFan_ShouldNotManufactureAVertexNormal()
    {
        FixedPointAnchor2d[] fan = { Anchor(Vector2d.Right), Anchor(-Vector2d.Right), Anchor(Vector2d.Forward), Anchor(-Vector2d.Forward) };
        Assert.False(WideCenteredCapsule2dRelations.TryGetBoundaryVertexContact(new(Fixed64.Quarter, Fixed64.Quarter),
            Fixed64.Zero, Fixed64.Zero, Fixed64.One, Anchor(Vector2d.Zero), fan, out _));
    }

    [Fact]
    public void TiedCapsuleSide_ShouldUseTheAdmissibleOppositeDirection()
    {
        FixedPointAnchor2d[] fan = { Anchor(Vector2d.Right), Anchor(Vector2d.Forward), Anchor(-Vector2d.Forward) };
        Assert.True(WideCenteredCapsule2dRelations.TryGetBoundaryVertexContact(Vector2d.Zero,
            Fixed64.Zero, Fixed64.Two, Fixed64.Half, Anchor(Vector2d.Zero), fan, out var contact));
        Assert.Equal(Vector2d.Right, contact.Normal);
        Assert.Equal(Fixed64.Half, contact.Depth);
        Assert.True(contact.FirstAnchor.TryGetPoint(out Vector2d point));
        Assert.Equal(new Vector2d(Fixed64.Half, Fixed64.Zero), point);
    }

    [Fact]
    public void CapsuleCapVertex_ShouldRetainItsFiniteCapAnchor()
    {
        var vertex = Anchor(new(Fixed64.Zero, (Fixed64)1.25));
        FixedPointAnchor2d[] fan = { Anchor(new(0, 2)), Anchor(new(-1, 2)), Anchor(new(1, 2)) };
        Assert.True(WideCenteredCapsule2dRelations.TryGetBoundaryVertexContact(Vector2d.Zero, Fixed64.Zero,
            Fixed64.Two, Fixed64.Half, vertex, fan, out var contact));
        Assert.Equal(Vector2d.Forward, contact.Normal);
        Assert.Equal(Fixed64.Quarter, contact.Depth);
        Assert.True(contact.FirstAnchor.TryGetPoint(out var point));
        Assert.Equal(new Vector2d(Fixed64.Zero, (Fixed64)1.5), point);
    }

    [Fact]
    public void Polygon_ShouldRankTheAdmittedAxisInsteadOfRejectingItsOrdinaryWinner()
    {
        Vector2d[] polygon = { new(-Fixed64.Half, -Fixed64.One), new(Fixed64.Half, -Fixed64.One),
            new(Fixed64.Half, Fixed64.One), new(-Fixed64.Half, Fixed64.One) };
        FixedPointAnchor2d[] fan = { Anchor(Vector2d.Right), Anchor(-Vector2d.Right), Anchor(-Vector2d.Forward) };
        Assert.True(WideConvex2dRelations.TryGetBoundaryVertexContact(new(Fixed64.Quarter, Fixed64.Half),
            Fixed64.Zero, polygon, Anchor(Vector2d.Zero), fan, out var contact));
        Assert.Equal(-Vector2d.Forward, contact.Normal);
        Assert.Equal(Fixed64.Half, contact.Depth);
        Assert.True(contact.FirstAnchor.TryGetPoint(out Vector2d point));
        Assert.Equal(new Vector2d(Fixed64.Zero, -Fixed64.Half), point);
    }

    [Theory]
    [InlineData(-1)] [InlineData(0)] [InlineData(1)]
    public void MissingOverlapOrAdmissibleFan_ShouldRemainWithoutContact(int side)
    {
        Vector2d[] polygon = { new(-1, -1), new(1, -1), new(1, 1), new(-1, 1) };
        FixedPointAnchor2d[] fan = { Anchor(Vector2d.Right), Anchor(-Vector2d.Right), Anchor(Vector2d.Forward), Anchor(-Vector2d.Forward) };
        Assert.False(WideConvex2dRelations.TryGetBoundaryVertexContact(new(side * 3, 0),
            Fixed64.Zero, polygon, Anchor(Vector2d.Zero), fan, out _));
        Assert.False(WideCenteredCapsule2dRelations.TryGetBoundaryVertexContact(new(side * 3, 0),
            Fixed64.Zero, Fixed64.Zero, Fixed64.One, Anchor(Vector2d.Zero), fan, out _));
    }

    [Theory]
    [InlineData(-1, 0, -1)]
    [InlineData(0, 0, 1)]
    [InlineData(1, 0, 2)]
    [InlineData(0.5, 0, 0)]
    [InlineData(0.5, 1, -1)]
    [InlineData(2, 0, -1)]
    public void PointLocation_ShouldRetainExactNativeClipBounds(double x, double y, int expected)
    {
        var clip = new RationalSegmentInterval(Signed320.ExtendValue(Signed192.Signed(1)), Signed320.ExtendValue(Signed192.Signed(2)),
            Signed320.One, Signed320.One);
        Assert.Equal(expected, WideConvex2dRelations.LocateBoundaryPoint(Anchor(new((Fixed64)x, (Fixed64)y)),
            Vector2d.Zero, Fixed64.Zero, new(new(-1, 0), new(1, 0)), clip));
    }

    [Theory]
    [InlineData(false)]
    [InlineData(true)]
    public void CollapsedEdge_ShouldOnlyContainItsPoint(bool same)
    {
        Assert.Equal(same ? 1 : -1, WideConvex2dRelations.LocateBoundaryPoint(Anchor(same ? Vector2d.Zero : Vector2d.Right),
            Vector2d.Zero, Fixed64.Zero, new(Vector2d.Zero, Vector2d.Zero), default));
    }

    [Fact]
    public void ObtuseIncidentEdgeProjection_ShouldNotSupplyAnUnboundedWitness()
    {
        Vector2d[] triangle = { new(0, 0), new(4, 0), new(5, 1), new(5, 1) };
        var vertex = Anchor(new((Fixed64)0.1, (Fixed64)0.01));
        Assert.True(WideConvex2dRelations.TryGetBoundaryVertexContact(Vector2d.Zero, Fixed64.Zero,
            triangle, vertex, Array.Empty<FixedPointAnchor2d>(), out var contact));
        // The sloped y=x/5 edge is slightly nearer than y=0; the short
        // obtuse edge still has an inadmissible unbounded projection.
        Assert.True(contact.Normal.X < Fixed64.Zero && contact.Normal.Y > Fixed64.Half);
        Assert.InRange(5 * contact.Normal.X.m_rawValue + contact.Normal.Y.m_rawValue, -3L, 3L);
        Vector2d point = contact.FirstAnchor.LocalPoint;
        Assert.True(point.X >= Fixed64.Zero && point.X <= (Fixed64)5);
        Assert.True(point.Y >= Fixed64.Zero && point.Y <= Fixed64.One);
        Assert.InRange(point.X.m_rawValue - 5 * point.Y.m_rawValue, -3L, 3L);
        long tangent = 5 * vertex.LocalPoint.X.m_rawValue + vertex.LocalPoint.Y.m_rawValue;
        Assert.InRange(5 * point.X.m_rawValue + point.Y.m_rawValue - tangent, -3L, 3L);
        long gap = vertex.LocalPoint.X.m_rawValue - 5 * vertex.LocalPoint.Y.m_rawValue;
        long depth = contact.Depth.m_rawValue;
        Assert.True(26 * (2 * depth - 1) * (2 * depth - 1) <= 4 * gap * gap);
        Assert.True(4 * gap * gap <= 26 * (2 * depth + 1) * (2 * depth + 1));
        Assert.Equal(vertex, contact.SecondAnchor);
    }

    [Theory]
    [InlineData(-2, -1)] [InlineData(-1, 1)] [InlineData(0, 0)] [InlineData(1, 2)]
    public void FullSpanPointLocation_ShouldRetainBothOriginalEndpoints(int x, int expected)
    {
        Assert.Equal(expected, WideConvex2dRelations.LocateBoundaryPoint(Anchor(new(x, 0)), Vector2d.Zero,
            Fixed64.Zero, new(new(-1, 0), new(1, 0)), default));
        Assert.Equal(-1, WideConvex2dRelations.LocateBoundaryPoint(Anchor(Vector2d.Forward), Vector2d.Zero,
            Fixed64.Zero, new(Vector2d.Zero, Vector2d.Zero), default));
    }

    [Fact]
    public void PointInsideOriginalEdgeButBeyondRetainedUpperBound_ShouldRemainOutside()
    {
        var interval = new RationalSegmentInterval(Signed320.ExtendValue(Signed192.Signed(1)), Signed320.ExtendValue(Signed192.Signed(4)),
            Signed320.ExtendValue(Signed192.Signed(3)), Signed320.ExtendValue(Signed192.Signed(4)));
        Assert.Equal(-1, WideConvex2dRelations.LocateBoundaryPoint(Anchor(new((Fixed64)0.75, Fixed64.Zero)), Vector2d.Zero,
            Fixed64.Zero, new(new(-1, 0), new(1, 0)), interval));
    }

    [Theory]
    [InlineData(0)] [InlineData(1)] [InlineData(2)] [InlineData(3)] [InlineData(4)]
    public void SupportingLineDegeneracyAndNonparallelFrames_ShouldRemainDistinct(int kind)
    {
        FixedSegment2d first = kind == 1 ? new(Vector2d.Zero, Vector2d.Zero) : kind == 0
            ? new(Vector2d.Zero, Vector2d.Forward) : new(Vector2d.Zero, Vector2d.Right);
        FixedSegment2d second = kind == 2 ? new(Vector2d.Zero, Vector2d.Zero)
            : kind is 0 or 3 ? new(Vector2d.Zero, Vector2d.Forward) : new(Vector2d.Zero, Vector2d.Right);
        Vector2d inward = kind == 0 ? Vector2d.Right : kind == 4 ? Vector2d.Right : Vector2d.Forward;
        Assert.Equal(kind == 0, WideConvex2dRelations.SameBoundaryPlane(Vector2d.Zero, Fixed64.Zero, first, inward,
            Vector2d.Zero, Fixed64.Zero, second, inward));
    }

    [Theory]
    [InlineData(0, false, true)]
    [InlineData(0, true, false)]
    [InlineData(1, false, false)]
    public void SupportingLines_ShouldRequireCoincidenceAndMatchingInwardSides(int height, bool opposite, bool same)
    {
        Assert.Equal(same, WideConvex2dRelations.SameBoundaryPlane(Vector2d.Zero, Fixed64.Zero,
            new(Vector2d.Zero, Vector2d.Right), -Vector2d.Forward, new(3, height), Fixed64.Zero,
            new(Vector2d.Right, Vector2d.Zero), opposite ? Vector2d.Forward : -Vector2d.Forward));
    }

    [Theory]
    [InlineData(-1, true)] [InlineData(0, true)] [InlineData(1, true)]
    [InlineData(2, false)] [InlineData(3, false)]
    public void StrictConvexClassification_ShouldDistinguishPenetrationFromTouchWithoutRoundedDepth(int offset, bool penetrates)
    {
        Vector2d[] square = { new(-1, -1), new(1, -1), new(1, 1), new(-1, 1) };
        Assert.Equal(penetrates, WideConvex2dRelations.DoConvexesOverlap(Vector2d.Zero, Fixed64.Zero, square,
            new(offset, 0), Fixed64.Zero, square));
        Assert.False(WideConvex2dRelations.DoConvexesOverlap(Vector2d.Zero, Fixed64.Zero,
            new[] { Vector2d.Zero, Vector2d.Zero, Vector2d.Zero }, Vector2d.Zero, Fixed64.Zero,
            new[] { Vector2d.Zero, Vector2d.Zero, Vector2d.Zero }));
    }

    [Theory]
    [InlineData(-1, true, true)] [InlineData(0, false, true)] [InlineData(1, false, false)]
    public void CapsuleClassification_ShouldKeepClosedTouchSeparateFromStrictPenetration(int offset, bool strict, bool closed)
    {
        Vector2d[] square = { new(-1, -1), new(1, -1), new(1, 1), new(-1, 1) };
        Vector2d center = new((Fixed64)1.5 + Fixed64.FromRaw(offset), Fixed64.Zero);
        foreach (Fixed64 length in new[] { Fixed64.Zero, Fixed64.One })
        {
            Assert.Equal(strict, WideCenteredCapsule2dRelations.DoesCapsuleOverlapConvex(center, Vector2d.Forward,
                length, Fixed64.Half, Vector2d.Zero, Fixed64.Zero, square));
            Assert.Equal(closed, WideCenteredCapsule2dRelations.DoesCapsuleOverlapConvex(center, Vector2d.Forward,
                length, Fixed64.Half, Vector2d.Zero, Fixed64.Zero, square, strict: false));
        }
    }

    [Fact]
    public void OddRawCapsuleCore_ShouldRetainStrictPenetrationWhenContactDepthRoundsToZero()
    {
        Vector2d[] square = { new(-1, -1), new(1, -1), new(1, 1), new(-1, 1) };
        Vector2d center = new(Fixed64.Zero, (Fixed64)1.5);
        Fixed64 length = Fixed64.FromRaw(1);
        Assert.True(WideCenteredCapsule2dRelations.DoesCapsuleOverlapConvex(center, Vector2d.Forward,
            length, Fixed64.Half, Vector2d.Zero, Fixed64.Zero, square));
        Span<FixedPointAnchor2d> first = stackalloc FixedPointAnchor2d[2], second = stackalloc FixedPointAnchor2d[2];
        Assert.True(WideCenteredCapsule2dRelations.TryGetContacts(center, Fixed64.Zero, Vector2d.Forward,
            length, Fixed64.Half, Vector2d.Zero, Fixed64.Zero, square, first, second, out _, out _, out Fixed64 depth, out _));
        Assert.Equal(Fixed64.Zero, depth);
    }

    [Theory]
    [InlineData(-1, false, false)] [InlineData(0, false, true)] [InlineData(1, true, true)]
    public void ObliqueConvexClassification_ShouldRetainSubrawPenetration(int offset, bool strict, bool closed)
    {
        Vector2d[] first = { new(-2, 1), new(0, 0), new(2, 4), new(0, 5) };
        Vector2d[] second = { new(-2, 1), new(0, 0), new(-2, -4), new(-4, -3) };
        Vector2d origin = new(Fixed64.FromRaw(offset), Fixed64.Zero);
        Assert.Equal(strict, WideConvex2dRelations.DoConvexesOverlap(Vector2d.Zero, Fixed64.Zero, first,
            origin, Fixed64.Zero, second));
        Assert.Equal(closed, WideConvex2dRelations.DoConvexesOverlap(Vector2d.Zero, Fixed64.Zero, first,
            origin, Fixed64.Zero, second, strict: false));
        if (strict)
        {
            Span<Vector2d> a = stackalloc Vector2d[2], b = stackalloc Vector2d[2];
            Assert.True(WideConvex2dRelations.TryGetContactOffsets(Vector2d.Zero, Fixed64.Zero, first,
                origin, Fixed64.Zero, second, a, b, out _, out _, out Fixed64 depth, out _));
            Assert.Equal(Fixed64.Zero, depth);
        }
    }

    [Theory]
    [InlineData(0, 0, true)] [InlineData(1, 0, false)] [InlineData(0, 1, false)]
    public void ClosedCollapsedConvexes_ShouldRetainFiniteCoordinateExtent(int x, int y, bool closed)
    {
        Vector2d[] point = { Vector2d.Zero, Vector2d.Zero, Vector2d.Zero };
        Assert.Equal(closed, WideConvex2dRelations.DoConvexesOverlap(Vector2d.Zero, Fixed64.Zero, point,
            new(x, y), Fixed64.Zero, point, strict: false));
        Assert.False(WideConvex2dRelations.DoConvexesOverlap(Vector2d.Zero, Fixed64.Zero, point,
            new(x, y), Fixed64.Zero, point));
    }

    private static FixedPointAnchor2d Anchor(Vector2d point) => new(Vector2d.Zero, Fixed64.Zero, point);
}
