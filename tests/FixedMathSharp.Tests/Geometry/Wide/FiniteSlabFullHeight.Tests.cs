using System;
using FixedMathSharp.Geometry;
using Xunit;

namespace FixedMathSharp.Tests.Geometry;

public sealed class FiniteSlabFullHeightTests
{
    [Theory]
    [InlineData(1)]
    [InlineData(3)]
    public void FullHeightAdmission_ShouldPreserveAnOddRawCapInsteadOfRoundingTheHalf(long heightRaw)
    {
        Fixed64 height = Fixed64.FromRaw(heightRaw);
        FiniteSlabSectionFrame full = new(Vector3d.Zero, FixedQuaternion.Identity, Vector3d.Zero,
            FixedQuaternion.Identity, Fixed64.Zero, Fixed64.One, height, fullHeight: true);
        FiniteSlabSectionFrame mixed = new(Vector3d.Zero, FixedQuaternion.Identity, Vector3d.Zero,
            FixedQuaternion.Identity, Fixed64.Zero, Fixed64.One, height);
        Vector3d outside = new(Fixed64.Zero, Fixed64.FromRaw(heightRaw / 2 + 1), Fixed64.Zero);
        Assert.True(full.ContainsPoint(Vector3d.Zero)); Assert.False(full.ContainsPoint(outside));
        Assert.True(mixed.ContainsPoint(outside));
        Assert.False(full.IntersectsSegment(new(outside, outside)));
        Assert.True(mixed.IntersectsSegment(new(outside, outside)));
    }

    [Theory]
    [InlineData(1, -1, false)]
    [InlineData(1, 1, false)]
    [InlineData(3, -1, false)]
    [InlineData(3, 1, false)]
    [InlineData(1, -1, true)]
    [InlineData(1, 1, true)]
    [InlineData(3, -1, true)]
    [InlineData(3, 1, true)]
    public void SegmentCapAnchor_ShouldRetainTheOddHalfThroughTranslationAndRotation(long heightRaw, int side, bool rotate)
    {
        Fixed64 height = Fixed64.FromRaw(heightRaw);
        Vector3d center = new((Fixed64)10, Fixed64.FromRaw(3), (Fixed64)(-7));
        FixedQuaternion rotation = rotate ? new FixedQuaternion(Fixed64.Zero, Fixed64.Zero, Fixed64.One, Fixed64.Zero) : FixedQuaternion.Identity;
        WideAxis3[] fan = { Axis(1, 0, 0), Axis(-1, 0, 0), Axis(0, -side, 0), Axis(0, 0, 1), Axis(0, 0, -1) };
        Assert.True(SegmentSlabSurfaceContact.TryGetContact(new(Vector3d.Zero, Vector3d.Zero), center, rotation,
            center, rotation, Fixed64.Zero, Fixed64.One, height, Array.Empty<Vector2d>(), fan,
            interiorOnly: false, out var contact, fullHeight: true));
        Assert.True(contact.SecondAnchor.TryGetPoint(out Vector3d point));
        int worldSide = rotate ? -side : side;
        long doubledWorldY = 6 + worldSide * heightRaw;
        long roundedY = RoundHalfEven(doubledWorldY);
        Assert.Equal(new Vector3d(center.X, Fixed64.FromRaw(roundedY), center.Z), point);
        Assert.NotEqual(0L, contact.SecondAnchor.ExactLocalTerm.Y);
        Assert.Equal(0L, contact.SecondAnchor.ExactLocalTerm.X); Assert.Equal(0L, contact.SecondAnchor.ExactLocalTerm.Z);
        Assert.Equal(Fixed64.FromRaw(heightRaw == 1 ? 0 : 2), contact.Depth);
    }

    [Theory]
    [InlineData(1, -1)]
    [InlineData(1, 1)]
    [InlineData(3, -1)]
    [InlineData(3, 1)]
    public void PlaneRayCapAnchor_ShouldRetainTheSameOddHalfAsFiniteAdmission(long heightRaw, int orientation)
    {
        FixedTriangle plane = new(new Vector3d(-2, 0, -2), new Vector3d(2, 0, -2), new Vector3d(0, 0, 2));
        Vector3d center = new(Fixed64.Zero, Fixed64.FromRaw(3), Fixed64.Zero);
        FiniteSlabRayFrame frame = new(plane, center, FixedQuaternion.Identity, center, FixedQuaternion.Identity,
            Fixed64.Zero, Fixed64.One, Fixed64.FromRaw(heightRaw), fullHeight: true);
        Assert.Equal(Fixed64.FromRaw(heightRaw), frame.FullHeight);
        Assert.Equal(1, frame.Plane.Normal.Y.Sign);
        Span<ulong> values = stackalloc ulong[FiniteSlabRaySelection.StorageWords];
        Span<sbyte> signs = stackalloc sbyte[FiniteSlabRaySelection.SignCount];
        FiniteSlabRaySelection selection = new(values, signs);
        FiniteSlabRayExtrema.AccumulatePlane(frame, Array.Empty<Vector2d>(), orientation, ref selection);
        Assert.True(selection.TryGetContactAnchor(frame, out Fixed64 depth, out bool clamped,
            out _, out FixedPointAnchor anchor));
        Assert.False(clamped); Assert.Equal(Fixed64.FromRaw(heightRaw == 1 ? 0 : 2), depth);
        Assert.NotEqual(0L, anchor.ExactLocalTerm.Y);
        Assert.True(anchor.TryGetPoint(out Vector3d point));
        Assert.Equal(Fixed64.FromRaw(RoundHalfEven(6 + orientation * heightRaw)), point.Y);
    }

    [Theory]
    [InlineData(-1)]
    [InlineData(1)]
    public void ObliqueRimAnchor_ShouldPreserveItsOddFullHeightCap(int side)
    {
        Vector3d a = new(5 * Fixed64.One / 4, side * Fixed64.One / 4, 9 * Fixed64.One / 4);
        Vector3d b = new(7 * Fixed64.One / 4, side * 3 * Fixed64.One / 4, 7 * Fixed64.One / 4);
        Vector3d center = new(Fixed64.Zero, Fixed64.FromRaw(3), Fixed64.Zero);
        WideAxis3[] fan = { Axis(1, -4 * side, 0), Axis(-1, 2 * side, 0),
            Axis(1, side, -1), Axis(-1, -side, 1) };
        Fixed64 height = Fixed64.FromRaw(Fixed64.Two.m_rawValue + 1);
        Assert.True(SegmentSlabSurfaceContact.TryGetContact(new(a, b), center, FixedQuaternion.Identity,
            center, FixedQuaternion.Identity, Fixed64.Zero, (Fixed64)5, height, Array.Empty<Vector2d>(), fan,
            interiorOnly: true, out var contact, fullHeight: true));
        Assert.NotEqual(0L, contact.SecondAnchor.ExactLocalTerm.Y);
        Assert.True(contact.SecondAnchor.TryGetPoint(out Vector3d point));
        Assert.Equal(Fixed64.FromRaw(RoundHalfEven(6 + side * height.m_rawValue)), point.Y);
        Assert.InRange(point.X.m_rawValue, ((Fixed64)3).m_rawValue - 3, ((Fixed64)3).m_rawValue + 3);
        Assert.InRange(point.Z.m_rawValue, ((Fixed64)4).m_rawValue - 3, ((Fixed64)4).m_rawValue + 3);
    }

    [Fact]
    public void OddHeightSideContact_ShouldKeepItsActualInteriorYInsteadOfAddingACapResidual()
    {
        Vector3d p = new(Fixed64.Half, Fixed64.Zero, Fixed64.Zero);
        WideAxis3[] fan = { Axis(-1, 0, 0), Axis(0, 1, 0), Axis(0, -1, 0), Axis(0, 0, 1), Axis(0, 0, -1) };
        Assert.True(SegmentSlabSurfaceContact.TryGetContact(new(p, p), Vector3d.Zero, FixedQuaternion.Identity,
            Vector3d.Zero, FixedQuaternion.Identity, Fixed64.Zero, Fixed64.One, Fixed64.FromRaw(3),
            Array.Empty<Vector2d>(), fan, false, out var contact, fullHeight: true));
        Assert.Equal(Fixed64.Half, contact.Depth); Assert.Equal(Vector3d.Left, contact.Normal);
        Assert.Equal(0L, contact.SecondAnchor.ExactLocalTerm.Y);
        Assert.True(contact.SecondAnchor.TryGetPoint(out Vector3d point)); Assert.Equal(Vector3d.Right, point);
    }

    [Fact]
    public void OddHeightFiniteSideRay_ShouldNotInventACapResidual()
    {
        FixedTriangle plane = new(new Vector3d(0, 0, -2), new Vector3d(0, 0, 2),
            new Vector3d(Fixed64.Zero, Fixed64.FromRaw(1), Fixed64.Zero));
        FiniteSlabRayFrame frame = new(plane, Vector3d.Zero, FixedQuaternion.Identity, Vector3d.Zero,
            FixedQuaternion.Identity, Fixed64.Zero, Fixed64.One, Fixed64.FromRaw(3), fullHeight: true);
        Span<ulong> values = stackalloc ulong[FiniteSlabRaySelection.StorageWords];
        Span<sbyte> signs = stackalloc sbyte[FiniteSlabRaySelection.SignCount];
        FiniteSlabRaySelection selection = new(values, signs);
        FiniteSlabRayExtrema.AccumulateTriangle(frame, plane, Array.Empty<Vector2d>(), 1, ref selection);
        Assert.True(selection.TryGetContactAnchor(frame, out Fixed64 depth, out bool clamped,
            out _, out FixedPointAnchor anchor));
        Assert.False(clamped); Assert.Equal(Fixed64.One, depth); Assert.Equal(0L, anchor.ExactLocalTerm.Y);
        Assert.True(anchor.TryGetPoint(out Vector3d point));
        Assert.Equal(Fixed64.One, point.X); Assert.InRange(point.Y.m_rawValue, 0, 1);
    }

    [Fact]
    public void OddHeightCap_ShouldComposeWithTheExistingOddHalfCoreResidual()
    {
        Vector3d p = new(Fixed64.Zero, Fixed64.Zero, Fixed64.Half + Fixed64.FromRaw(2));
        Vector3d center = new(Fixed64.Zero, Fixed64.FromRaw(3), Fixed64.FromRaw(3));
        WideAxis3[] fan = { Axis(1, 0, 0), Axis(-1, 0, 0), Axis(0, 1, -3), Axis(0, -1, 1) };
        Fixed64 height = Fixed64.FromRaw(Fixed64.Two.m_rawValue + 1);
        Assert.True(SegmentSlabSurfaceContact.TryGetContact(new(p, p), center, FixedQuaternion.Identity,
            center, FixedQuaternion.Identity, Fixed64.FromRaw(3), Fixed64.One, height,
            Array.Empty<Vector2d>(), fan, false, out var contact, fullHeight: true));
        Assert.NotEqual(0L, contact.SecondAnchor.ExactLocalTerm.Y);
        Assert.NotEqual(0L, contact.SecondAnchor.ExactLocalTerm.Z);
        Assert.True(contact.SecondAnchor.TryGetPoint(out Vector3d point));
        Assert.Equal(Fixed64.FromRaw(Fixed64.One.m_rawValue + 4), point.Y);
        Assert.Equal(Fixed64.FromRaw(Fixed64.One.m_rawValue + 4), point.Z);
    }

    private static long RoundHalfEven(long twiceRaw)
    {
        long lower = twiceRaw >> 1;
        return (twiceRaw & 1L) != 0 && (lower & 1L) != 0 ? lower + 1 : lower;
    }

    private static WideAxis3 Axis(long x, long y, long z) => new(Signed320.ExtendValue(Signed192.Signed(x)),
        Signed320.ExtendValue(Signed192.Signed(y)), Signed320.ExtendValue(Signed192.Signed(z)));
}
