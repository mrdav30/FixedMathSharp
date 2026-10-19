using System;
using FixedMathSharp.Geometry;
using Xunit;

namespace FixedMathSharp.Tests;

public sealed class SegmentSlabSurfaceContactClippingTests
{
    [Theory]
    [InlineData(false, false)]
    [InlineData(false, true)]
    [InlineData(true, false)]
    [InlineData(true, true)]
    public void AffineObliqueRim_ShouldMaterializeTheFinitePhysicalStationaryFoot(bool boundary, bool reverse)
    {
        var source = new ScaledRigidFrame3d(Vector3d.Zero, FixedQuaternion.Identity, Vector3d.Zero,
            Vector3d.One * Fixed64.Half, FixedQuaternion.Identity, Vector3d.Zero, Vector3d.One);
        Vector3d a = new(5 * Fixed64.One / 2, Fixed64.Half, 9 * Fixed64.One / 2),
            b = new(7 * Fixed64.One / 2, 3 * Fixed64.One / 2, 7 * Fixed64.One / 2);
        RationalSegmentInterval clip = boundary ? Interval(1, 2, 3, 4, reverse) : Interval(1, 4, 3, 4, reverse);
        if (reverse) (a, b) = (b, a);
        WideAxis3[] fan = { Axis(1, -4, 0), Axis(-1, 2, 0), Axis(1, 1, -1), Axis(-1, -1, 1) };
        Assert.True(SegmentSlabSurfaceContact.TryGetContact(new(a, b), source, Vector3d.Zero,
            FixedQuaternion.Identity, Fixed64.Zero, (Fixed64)5, Fixed64.One, Array.Empty<Vector2d>(),
            fan, !boundary, clip, out FixedContactAnchors contact));
        Assert.True(contact.FirstAnchor.TryGetPoint(out Vector3d first));
        Assert.True(contact.SecondAnchor.TryGetPoint(out Vector3d second));
        Vector3d expectedFirst = new(3 * Fixed64.One / 2, Fixed64.Half, Fixed64.Two), expectedSecond = new(3, 1, 4);
        for (int axis = 0; axis < 3; axis++)
        {
            Assert.InRange(first[axis].m_rawValue, expectedFirst[axis].m_rawValue - 1, expectedFirst[axis].m_rawValue + 1);
            Assert.InRange(second[axis].m_rawValue, expectedSecond[axis].m_rawValue - 1, expectedSecond[axis].m_rawValue + 1);
        }
    }

    [Theory]
    [InlineData(false, false)]
    [InlineData(false, true)]
    [InlineData(true, false)]
    [InlineData(true, true)]
    public void SourceFrames_ShouldRetainWorldFanAndClippedFiniteSideWitnesses(bool rigid, bool prism)
    {
        var source = rigid ? ScaledRigidFrame3d.Rigid(Vector3d.Zero, FixedQuaternion.Identity)
            : new ScaledRigidFrame3d(Vector3d.Zero, FixedQuaternion.Identity, Vector3d.Zero,
                Vector3d.One * Fixed64.Half, FixedQuaternion.Identity, Vector3d.Zero, Vector3d.One);
        Fixed64 authored = rigid ? Fixed64.One : Fixed64.Two;
        var segment = new FixedSegment(new Vector3d(Fixed64.Half, Fixed64.Zero, Fixed64.Zero) * authored,
            new Vector3d(Fixed64.Half, Fixed64.Half, Fixed64.Zero) * authored);
        WideAxis3[] fan = { Axis(-1, 0, 0), Axis(0, 1, 0), Axis(0, -1, 0), Axis(0, 0, 1), Axis(0, 0, -1) };
        Vector2d[] offsets = Offsets(prism);
        Assert.True(SegmentSlabSurfaceContact.TryGetContact(segment, source, Vector3d.Zero,
            FixedQuaternion.Identity, Fixed64.Zero, Fixed64.One, Fixed64.One, offsets, fan, true,
            default(RationalSegmentInterval), out FixedContactAnchors native));
        RationalSegmentInterval clip = Interval(0, 1, 1, 4, false);
        Assert.True(SegmentSlabSurfaceContact.TryGetContact(segment, source, Vector3d.Zero,
            FixedQuaternion.Identity, Fixed64.Zero, Fixed64.One, Fixed64.One, offsets, fan, true, clip,
            out FixedContactAnchors clipped));
        Assert.Equal(Fixed64.Half, native.Depth); Assert.Equal(native.Depth, clipped.Depth);
        Assert.Equal(Vector3d.Left, clipped.Normal);
        Assert.True(clipped.FirstAnchor.TryGetPoint(out Vector3d point));
        Assert.Equal(Fixed64.Half, point.X); Assert.True(point.Y > Fixed64.Zero && point.Y < Fixed64.One / 8);
        Assert.False(SegmentSlabSurfaceContact.TryGetContact(new(segment.Start, segment.Start), source,
            Vector3d.Zero, FixedQuaternion.Identity, Fixed64.Zero, Fixed64.One, Fixed64.One,
            offsets, fan, true, out _));
        Assert.False(SegmentSlabSurfaceContact.TryGetContact(new(new(20, 0, 0), new(21, 0, 0)), source,
            Vector3d.Zero, FixedQuaternion.Identity, Fixed64.Zero, Fixed64.One, Fixed64.One,
            offsets, Array.Empty<WideAxis3>(), false, out _));
        var far = new FixedSegment(new Vector3d(3, 0, 0) * authored, new Vector3d(4, 0, 0) * authored);
        WideAxis3[] farFan = { Axis(1, 0, 0), Axis(0, 1, 0), Axis(0, -1, 0), Axis(0, 0, 1), Axis(0, 0, -1) };
        Assert.True(SegmentSlabSurfaceContact.TryGetFeaturePenetration(far, source, Vector3d.Zero,
            FixedQuaternion.Identity, Fixed64.Zero, Fixed64.One, Fixed64.One, offsets, farFan, false,
            out FixedContactAnchors feature));
        Assert.Equal((Fixed64)4, feature.Depth); Assert.Equal(Vector3d.Right, feature.Normal);
    }

    [Theory]
    [InlineData(false, false)]
    [InlineData(false, true)]
    [InlineData(true, false)]
    [InlineData(true, true)]
    public void ParallelSideFamily_ShouldSelectInsideTheRetainedEnd(bool prism, bool reverse)
    {
        var segment = new FixedSegment(new Vector3d(Fixed64.Half, -Fixed64.One, Fixed64.Zero),
            new Vector3d(Fixed64.Half, Fixed64.One, Fixed64.Zero));
        RationalSegmentInterval clip = Interval(0, 1, 1, 4, reverse);
        if (reverse) segment = new(segment.End, segment.Start);
        WideAxis3[] fan = { Axis(-1, 0, 0), Axis(0, 1, 0), Axis(0, -1, 0), Axis(0, 0, 1), Axis(0, 0, -1) };
        Assert.True(Contact(segment, prism, fan, true, clip, out FixedContactAnchors contact));
        Assert.True(contact.FirstAnchor.TryGetPoint(out Vector3d first));
        Assert.True(contact.SecondAnchor.TryGetPoint(out Vector3d second));
        Assert.True(first.Y > -Fixed64.One && first.Y < -Fixed64.Half);
        Assert.Equal(Fixed64.Half, first.X); Assert.Equal(Fixed64.Zero, first.Z);
        Assert.Equal(new Vector3d(Fixed64.One, first.Y, Fixed64.Zero), second);
        Assert.Equal(Fixed64.Half, contact.Depth); Assert.Equal(Vector3d.Left, contact.Normal);
    }

    [Theory]
    [InlineData(false, false)]
    [InlineData(false, true)]
    [InlineData(true, false)]
    [InlineData(true, true)]
    public void SourceOverlapOutsideRetainedInterval_ShouldNotAdmitAContact(bool prism, bool reverse)
    {
        var segment = new FixedSegment(new Vector3d(Fixed64.Half, Fixed64.Zero, Fixed64.Zero),
            new Vector3d(Fixed64.Half, (Fixed64)4, Fixed64.Zero));
        RationalSegmentInterval clip = Interval(1, 2, 3, 4, reverse);
        if (reverse) segment = new(segment.End, segment.Start);
        Assert.False(Contact(segment, prism, Array.Empty<WideAxis3>(), false, clip, out _));
    }

    [Theory]
    [InlineData(false, false)]
    [InlineData(false, true)]
    [InlineData(true, false)]
    [InlineData(true, true)]
    public void RetainedEndpointTouch_ShouldRemainClosedAndExcludeTheStrictInterior(bool prism, bool reverse)
    {
        var segment = new FixedSegment(new Vector3d(Fixed64.Half, Fixed64.Zero, Fixed64.Zero),
            new Vector3d(Fixed64.One + Fixed64.Half, Fixed64.Zero, Fixed64.Zero));
        RationalSegmentInterval clip = Interval(0, 1, 1, 2, reverse);
        if (reverse) segment = new(segment.End, segment.Start);
        WideAxis3[] fan = { Axis(-1, 0, 0), Axis(0, 1, 0), Axis(0, -1, 0), Axis(0, 0, 1), Axis(0, 0, -1) };
        Assert.True(Contact(segment, prism, fan, false, clip, out FixedContactAnchors contact));
        Assert.Equal(Fixed64.Zero, contact.Depth); Assert.Equal(Vector3d.Left, contact.Normal);
        Assert.True(contact.FirstAnchor.TryGetPoint(out Vector3d first)); Assert.Equal(Vector3d.Right, first);
        Assert.True(contact.SecondAnchor.TryGetPoint(out Vector3d second)); Assert.Equal(first, second);
        Assert.False(Contact(segment, prism, fan, true, clip, out _));
    }

    [Theory]
    [InlineData(false, false)]
    [InlineData(false, true)]
    [InlineData(true, false)]
    [InlineData(true, true)]
    public void RetainedPiece_ShouldRankItsOwnMinimumInsteadOfRejectingTheOriginalWinner(bool prism, bool reverse)
    {
        var segment = new FixedSegment(Vector3d.Zero, new Vector3d(Fixed64.One + Fixed64.Half, Fixed64.Zero, Fixed64.Zero));
        RationalSegmentInterval clip = Interval(1, 4, 1, 2, reverse);
        if (reverse) segment = new(segment.End, segment.Start);
        WideAxis3[] fan = { Axis(-1, 0, 0), Axis(0, 1, 0), Axis(0, -1, 0), Axis(0, 0, 1), Axis(0, 0, -1) };
        Assert.True(Contact(segment, prism, fan, false, clip, out FixedContactAnchors contact));
        Assert.Equal(Fixed64.One / 4, contact.Depth); Assert.Equal(Vector3d.Left, contact.Normal);
        Assert.True(contact.FirstAnchor.TryGetPoint(out Vector3d first));
        Assert.Equal(new Vector3d(3 * Fixed64.One / 4, Fixed64.Zero, Fixed64.Zero), first);
        Assert.True(contact.SecondAnchor.TryGetPoint(out Vector3d second)); Assert.Equal(Vector3d.Right, second);
    }

    [Theory]
    [InlineData(false)]
    [InlineData(true)]
    public void DefaultInterval_ShouldPreserveTheNativeContact(bool prism)
    {
        var segment = new FixedSegment(new Vector3d(Fixed64.Half, Fixed64.Zero, Fixed64.Zero),
            new Vector3d(Fixed64.Half, Fixed64.Half, Fixed64.Zero));
        Assert.True(Contact(segment, prism, Array.Empty<WideAxis3>(), false, default, out var clipped));
        Vector2d[] offsets = Offsets(prism);
        Assert.True(SegmentSlabSurfaceContact.TryGetContact(segment, Vector3d.Zero, FixedQuaternion.Identity,
            Vector3d.Zero, FixedQuaternion.Identity, Fixed64.Zero, Fixed64.One, Fixed64.One, offsets,
            Array.Empty<WideAxis3>(), false, out var native));
        Assert.Equal(native.Depth, clipped.Depth); Assert.Equal(native.Normal, clipped.Normal);
        Assert.Equal(native.FirstAnchor, clipped.FirstAnchor); Assert.Equal(native.SecondAnchor, clipped.SecondAnchor);
    }

    [Theory]
    [InlineData(false, false)]
    [InlineData(false, true)]
    [InlineData(true, false)]
    [InlineData(true, true)]
    public void ObliqueRimStationaryFoot_ShouldUseItsExactOriginalSegmentParameter(bool boundary, bool reverse)
    {
        Vector3d a = new(5 * Fixed64.One / 4, Fixed64.One / 4, 9 * Fixed64.One / 4);
        Vector3d b = new(7 * Fixed64.One / 4, 3 * Fixed64.One / 4, 7 * Fixed64.One / 4);
        RationalSegmentInterval clip = boundary ? Interval(1, 2, 3, 4, reverse) : Interval(1, 4, 3, 4, reverse);
        if (reverse) (a, b) = (b, a);
        // Keep the stationary stratum fixed when testing a closed retained
        // endpoint; disabling interiorOnly must not introduce other exit rays.
        WideAxis3[] fan = { Axis(1, -4, 0), Axis(-1, 2, 0), Axis(1, 1, -1), Axis(-1, -1, 1) };
        Assert.True(SegmentSlabSurfaceContact.TryGetContact(new(a, b), Vector3d.Zero, FixedQuaternion.Identity,
            Vector3d.Zero, FixedQuaternion.Identity, Fixed64.Zero, (Fixed64)5, Fixed64.One,
            Array.Empty<Vector2d>(), fan, interiorOnly: !boundary, clip, out var contact));
        Assert.True(contact.FirstAnchor.TryGetPoint(out Vector3d first));
        Assert.True(contact.SecondAnchor.TryGetPoint(out Vector3d second));
        Vector3d expectedFirst = new(3 * Fixed64.One / 2, Fixed64.Half, Fixed64.Two);
        Vector3d expectedSecond = new(3, 1, 4);
        for (int axis = 0; axis < 3; axis++)
        {
            Assert.InRange(first[axis].m_rawValue, expectedFirst[axis].m_rawValue - 1, expectedFirst[axis].m_rawValue + 1);
            Assert.InRange(second[axis].m_rawValue, expectedSecond[axis].m_rawValue - 1, expectedSecond[axis].m_rawValue + 1);
        }
        Fixed64 expectedDepth = FixedMath.Sqrt(13 * Fixed64.One / 2);
        Assert.InRange(contact.Depth.m_rawValue, expectedDepth.m_rawValue - 1, expectedDepth.m_rawValue + 1);
        if (boundary)
            Assert.False(SegmentSlabSurfaceContact.TryGetContact(new(a, b), Vector3d.Zero, FixedQuaternion.Identity,
                Vector3d.Zero, FixedQuaternion.Identity, Fixed64.Zero, (Fixed64)5, Fixed64.One,
                Array.Empty<Vector2d>(), fan, interiorOnly: true, clip, out _));
    }

    [Theory]
    [InlineData(false, false)]
    [InlineData(false, true)]
    [InlineData(true, false)]
    [InlineData(true, true)]
    public void FullWidthClip_ShouldKeepTheStrictInteriorOfAParallelFamily(bool prism, bool reverse)
    {
        var segment = new FixedSegment(new Vector3d(Fixed64.Half, -Fixed64.One, Fixed64.Zero),
            new Vector3d(Fixed64.Half, Fixed64.One, Fixed64.Zero));
        if (reverse) segment = new(segment.End, segment.Start);
        // These coprime 319-bit bounds straddle one half by less than one
        // world raw unit. Their mediant is exactly the admitted interior foot.
        RationalSegmentInterval clip = Geometry.PlaneRayClippedBoundaryTests.FullWidthMiddleInterval();
        WideAxis3[] fan = { Axis(-1, 0, 0), Axis(0, 1, 0), Axis(0, -1, 0), Axis(0, 0, 1), Axis(0, 0, -1) };
        Assert.True(Contact(segment, prism, fan, true, clip, out FixedContactAnchors contact));
        Assert.True(contact.FirstAnchor.TryGetPoint(out Vector3d first));
        Assert.True(contact.SecondAnchor.TryGetPoint(out Vector3d second));
        Assert.Equal(new Vector3d(Fixed64.Half, Fixed64.Zero, Fixed64.Zero), first);
        Assert.Equal(Vector3d.Right, second);
        Assert.Equal(Fixed64.Half, contact.Depth); Assert.Equal(Vector3d.Left, contact.Normal);
    }

    [Theory]
    [InlineData(false)]
    [InlineData(true)]
    public void FullWidthClip_ShouldAdmitTheExactObliqueStationaryFoot(bool reverse)
    {
        Vector3d a = new(5 * Fixed64.One / 4, Fixed64.One / 4, 9 * Fixed64.One / 4);
        Vector3d b = new(7 * Fixed64.One / 4, 3 * Fixed64.One / 4, 7 * Fixed64.One / 4);
        if (reverse) (a, b) = (b, a);
        RationalSegmentInterval clip = Geometry.PlaneRayClippedBoundaryTests.FullWidthMiddleInterval();
        WideAxis3[] fan = { Axis(1, -4, 0), Axis(-1, 2, 0), Axis(1, 1, -1), Axis(-1, -1, 1) };
        Assert.True(SegmentSlabSurfaceContact.TryGetContact(new(a, b), Vector3d.Zero, FixedQuaternion.Identity,
            Vector3d.Zero, FixedQuaternion.Identity, Fixed64.Zero, (Fixed64)5, Fixed64.One,
            Array.Empty<Vector2d>(), fan, interiorOnly: true, clip, out var contact));
        Assert.True(contact.FirstAnchor.TryGetPoint(out Vector3d first));
        Assert.True(contact.SecondAnchor.TryGetPoint(out Vector3d second));
        Vector3d expectedFirst = new(3 * Fixed64.One / 2, Fixed64.Half, Fixed64.Two);
        Vector3d expectedSecond = new(3, 1, 4);
        for (int axis = 0; axis < 3; axis++)
        {
            Assert.InRange(first[axis].m_rawValue, expectedFirst[axis].m_rawValue - 1, expectedFirst[axis].m_rawValue + 1);
            Assert.InRange(second[axis].m_rawValue, expectedSecond[axis].m_rawValue - 1, expectedSecond[axis].m_rawValue + 1);
        }
        Fixed64 expectedDepth = FixedMath.Sqrt(13 * Fixed64.One / 2);
        Assert.InRange(contact.Depth.m_rawValue, expectedDepth.m_rawValue - 1, expectedDepth.m_rawValue + 1);
    }

    private static bool Contact(FixedSegment segment, bool prism, ReadOnlySpan<WideAxis3> fan,
        bool interiorOnly, in RationalSegmentInterval clip, out FixedContactAnchors contact)
        => SegmentSlabSurfaceContact.TryGetContact(segment, Vector3d.Zero, FixedQuaternion.Identity,
            Vector3d.Zero, FixedQuaternion.Identity, Fixed64.Zero, Fixed64.One, Fixed64.One,
            Offsets(prism), fan, interiorOnly, clip, out contact);

    private static Vector2d[] Offsets(bool prism) => prism
        ? new[] { new Vector2d(-1, -1), new Vector2d(1, -1), new Vector2d(1, 1), new Vector2d(-1, 1) }
        : Array.Empty<Vector2d>();

    private static RationalSegmentInterval Interval(long ln, long ld, long un, long ud, bool reverse)
        => reverse ? new(Number(ud - un), Number(ud), Number(ld - ln), Number(ld))
            : new(Number(ln), Number(ld), Number(un), Number(ud));

    private static Signed320 Number(long value) => Signed320.ExtendValue(Signed192.Signed(value));
    private static WideAxis3 Axis(long x, long y, long z) => new(Number(x), Number(y), Number(z));
}
