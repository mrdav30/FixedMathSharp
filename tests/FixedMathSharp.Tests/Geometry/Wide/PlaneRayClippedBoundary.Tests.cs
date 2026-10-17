using System;
using FixedMathSharp.Geometry;
using Xunit;

namespace FixedMathSharp.Tests.Geometry;

public class PlaneRayClippedBoundaryTests
{
    private static readonly FixedQuaternion NegativeIdentity = new(Fixed64.Zero, Fixed64.Zero, Fixed64.Zero, -Fixed64.One);

    [Theory]
    [InlineData(false)]
    [InlineData(true)]
    public void ConeCutsMustCreateAndReplayNewRationalEndpoints(bool fractionalCut)
    {
        FixedTriangle plane = new(new Vector3d(-4, 0, -4), new(4, 0, -4), new(0, 0, 4));
        ConePlaneRayFrame frame = new(plane, Vector3d.Zero, FixedQuaternion.Identity,
            Vector3d.Zero, FixedQuaternion.Identity, (Fixed64)4, Fixed64.Two);
        FixedSegment segment = new(new Vector3d(fractionalCut ? -Fixed64.One : (Fixed64)(-2), Fixed64.Zero, Fixed64.Zero),
            new Vector3d(fractionalCut ? Fixed64.One : Fixed64.Two, Fixed64.Zero, Fixed64.Zero));
        var source = new ConePlaneRayEventSource(segment);
        RationalSegmentInterval clip = fractionalCut ? Interval(1, 3, 2, 5) : Interval(5, 8, 3, 4);
        int endpointCount = 0;
        void Visit(in ConePlaneRayEventSource visitedSource, in ConePlaneRayFrame visitedFrame,
            ConePlaneRayEvent descriptor, scoped ConePlaneRayPoint point, scoped ReadOnlySpan<ulong> root,
            scoped in ConePlaneRaySelection positive, scoped in ConePlaneRaySelection negative)
        {
            Assert.Equal(segment.Start, visitedSource.A); Assert.Equal(segment.End, visitedSource.B);
            Span<ulong> pv = stackalloc ulong[ConePlaneRaySelection.StorageWords], nv = stackalloc ulong[ConePlaneRaySelection.StorageWords];
            Span<int> ps = stackalloc int[ConePlaneRaySelection.SignCount], ns = stackalloc int[ConePlaneRaySelection.SignCount];
            Span<ulong> pointValues = stackalloc ulong[ConePlaneRayPoint.StorageWords], replayRoot = stackalloc ulong[ConePlaneRaySelection.RootWords];
            Span<int> pointSigns = stackalloc int[ConePlaneRayPoint.SignCount];
            ConePlaneRaySelection p = new(pv, ps), n = new(nv, ns);
            ConePlaneRayPoint replayPoint = new(pointValues, pointSigns);
            Assert.True(ConePlaneRayEvents.TryEvaluateEvent(source, frame, descriptor, clip, replayPoint,
                replayRoot, ref p, ref n));
            Assert.Equal(0, point.CompareTo(replayPoint, root, replayRoot));
            if (positive.HasValue) Assert.Equal(0, p.CompareDepth(positive));
            if (negative.HasValue) Assert.Equal(0, n.CompareDepth(negative));
            if (descriptor.Kind != ConePlaneRayEventKind.EdgeEndpoint) return;
            endpointCount++;
            if (descriptor.Endpoint != (fractionalCut ? 1 : 0)) return;
            Assert.True(positive.HasValue);
            // x=1/2 gives upper cone exit y=1; x=-1/5 gives y=8/5.
            Assert.Equal(fractionalCut ? Fixed64.FromFraction(8, 5) : Fixed64.One, positive.GetRoundedMaximumDepth());
            Span<ulong> expectedValues = stackalloc ulong[4 * ConePlaneRaySelection.FieldWords]; Span<int> expectedSigns = stackalloc int[4];
            ContactQuadratic expectedN = ContactQuadratic.At(expectedValues, expectedSigns, 0, ConePlaneRaySelection.FieldWords);
            ContactQuadratic expectedD = ContactQuadratic.At(expectedValues, expectedSigns, 1, ConePlaneRaySelection.FieldWords);
            Signed320 scale = WideArithmetic.MultiplySigned192(frame.Finite.ShapeFrame.Denominator, Signed192.Raw(Fixed64.Two));
            expectedN.Set(Signed576.ExtendValue(fractionalCut ? WideArithmetic.Negate(scale) : scale));
            expectedD.Set(Signed576.ExtendValue(Signed320.ExtendValue(Signed192.Signed(fractionalCut ? 5 : 2))));
            Assert.Equal(0, ContactQuadratic.CompareRatios(point.X, point.Denominator, root,
                expectedN, expectedD, ReadOnlySpan<ulong>.Empty));
        }
        Assert.True(ConePlaneRayEvents.VisitEvents(source, frame, clip, Visit));
        Assert.Equal(2, endpointCount);
    }

    [Theory]
    [InlineData(0)]
    [InlineData(1)]
    [InlineData(2)]
    public void SlabSegmentBoundsMustConstrainTheOriginalBarycentricParameter(int shape)
    {
        FixedTriangle plane = new(new Vector3d(-4, 4, -4), new(4, -4, -4), new(0, 0, 4));
        FiniteSlabRayFrame frame = new(plane, Vector3d.Zero, FixedQuaternion.Identity,
            Vector3d.Zero, FixedQuaternion.Identity, shape == 1 ? Fixed64.Two : Fixed64.Zero, Fixed64.Two, Fixed64.One);
        Vector2d[] offsets = shape == 2 ? new[] { new Vector2d(-2, -2), new(2, -2), new(2, 2), new(-2, 2) } : Array.Empty<Vector2d>();
        FixedSegment segment = new(new Vector3d(-2, 2, 0), new(2, -2, 0));
        Span<ulong> values = stackalloc ulong[FiniteSlabRaySelection.StorageWords]; Span<sbyte> signs = stackalloc sbyte[FiniteSlabRaySelection.SignCount];
        FiniteSlabRaySelection maximum = new(values, signs);
        RationalSegmentInterval clip = Interval(0, 1, 1, 2);
        FiniteSlabRayExtrema.AccumulateSegment(frame, segment, clip, offsets, 1, ref maximum);
        Assert.True(maximum.HasValue);
        Assert.Equal(segment.Start, maximum.Metadata.Triangle.A); Assert.Equal(segment.End, maximum.Metadata.Triangle.B);
        Assert.True(maximum.TryGetContactAnchor(frame, out _, out bool clamped, out Vector3d point, out FixedPointAnchor exit));
        Assert.False(clamped);
        Assert.Equal(Vector3d.Zero, point);
        Assert.True(exit.TryGetPoint(out Vector3d endpoint));
        Assert.Equal(new Vector3d(1, 1, 0), endpoint);
    }

    [Fact]
    public void ConeClipMustAlsoRejectProjectedRimEventsOutsideItsOriginalParameterRange()
    {
        FixedTriangle plane = new(new Vector3d(-4, -2, -4), new(4, -2, -4), new(0, -2, 4));
        ConePlaneRayFrame frame = new(plane, Vector3d.Zero, FixedQuaternion.Identity,
            Vector3d.Zero, FixedQuaternion.Identity, (Fixed64)4, Fixed64.Two);
        var source = new ConePlaneRayEventSource(new FixedSegment(new(-2, -2, 1), new(2, -2, 1)));
        int originalRims = 0, clippedRims = 0, clippedPoints = 0;
        void Original(in ConePlaneRayEventSource s, in ConePlaneRayFrame f, ConePlaneRayEvent e,
            scoped ConePlaneRayPoint p, scoped ReadOnlySpan<ulong> r,
            scoped in ConePlaneRaySelection positive, scoped in ConePlaneRaySelection negative)
        { if (e.Kind == ConePlaneRayEventKind.UpperRim) originalRims++; }
        void Clipped(in ConePlaneRayEventSource s, in ConePlaneRayFrame f, ConePlaneRayEvent e,
            scoped ConePlaneRayPoint p, scoped ReadOnlySpan<ulong> r,
            scoped in ConePlaneRaySelection positive, scoped in ConePlaneRaySelection negative)
        {
            clippedPoints++;
            if (e.Kind == ConePlaneRayEventKind.UpperRim) clippedRims++;
            Span<ulong> values = stackalloc ulong[4 * ConePlaneRaySelection.FieldWords]; Span<int> signs = stackalloc int[4];
            ContactQuadratic n = ContactQuadratic.At(values, signs, 0, ConePlaneRaySelection.FieldWords);
            ContactQuadratic d = ContactQuadratic.At(values, signs, 1, ConePlaneRaySelection.FieldWords);
            d.Set(Signed576.ExtendValue(Signed320.ExtendValue(Signed192.Signed(1))));
            n.Set(Signed576.ExtendValue(Signed320.ExtendValue(Signed192.Raw(-Fixed64.One))));
            Assert.True(ContactQuadratic.CompareRatios(p.X, p.Denominator, r, n, d, ReadOnlySpan<ulong>.Empty) >= 0);
            n.Set(Signed576.ExtendValue(Signed320.ExtendValue(Signed192.Raw(Fixed64.One))));
            Assert.True(ContactQuadratic.CompareRatios(p.X, p.Denominator, r, n, d, ReadOnlySpan<ulong>.Empty) <= 0);
        }
        Assert.True(ConePlaneRayEvents.VisitEvents(source, frame, Original));
        Assert.True(originalRims > 0);
        Assert.True(ConePlaneRayEvents.VisitEvents(source, frame, Interval(3, 8, 5, 8), Clipped));
        Assert.True(clippedPoints > 0);
        Assert.Equal(0, clippedRims);
    }

    [Theory]
    [InlineData(false)]
    [InlineData(true)]
    public void FullWidthCoprimeSlabCutsMustSurviveTallIndependentFrameConditioning(bool curvedExit)
    {
        Fixed64 extent = Fixed64.MaxValue;
        FixedTriangle plane = new(new Vector3d(-extent, extent, -extent), new(extent, -extent, -extent), new(Fixed64.Zero, Fixed64.Zero, extent));
        Vector3d middle = curvedExit ? Vector3d.Forward : Vector3d.Zero;
        FixedSegment segment = new(new Vector3d(-extent, extent, middle.Z), new(extent, -extent, middle.Z));
        Fixed64 height = curvedExit ? Fixed64.MaxValue : Fixed64.FromRaw(1L << 58);
        Fixed64 radius = curvedExit ? Fixed64.Two : Fixed64.MaxValue;
        FiniteSlabRayFrame firstFrame = new(plane, Vector3d.Zero, FixedQuaternion.Identity,
            Vector3d.Zero, NegativeIdentity, Fixed64.Zero, radius, height);
        FiniteSlabRayFrame secondFrame = new(plane, Vector3d.Zero, NegativeIdentity,
            Vector3d.Zero, NegativeIdentity, Fixed64.Zero, radius, height);
        RationalSegmentInterval clip = FullWidthMiddleInterval();
        Span<ulong> av = stackalloc ulong[FiniteSlabRaySelection.StorageWords], bv = stackalloc ulong[FiniteSlabRaySelection.StorageWords];
        Span<ulong> zv = stackalloc ulong[FiniteSlabRaySelection.StorageWords];
        Span<sbyte> aSign = stackalloc sbyte[FiniteSlabRaySelection.SignCount], bSign = stackalloc sbyte[FiniteSlabRaySelection.SignCount];
        Span<sbyte> zSign = stackalloc sbyte[FiniteSlabRaySelection.SignCount];
        FiniteSlabRaySelection a = new(av, aSign), b = new(bv, bSign), zero = new(zv, zSign);
        FiniteSlabRayExtrema.AccumulateSegment(firstFrame, segment, clip, ReadOnlySpan<Vector2d>.Empty, 1, ref a);
        FiniteSlabRayExtrema.AccumulateSegment(secondFrame, segment, clip, ReadOnlySpan<Vector2d>.Empty, 1, ref b);
        FiniteSlabRayExtrema.AccumulateTriangle(firstFrame, new FixedTriangle(middle, middle, middle),
            ReadOnlySpan<Vector2d>.Empty, 1, ref zero);
        Assert.True(a.HasValue && b.HasValue && zero.HasValue);
        Assert.Equal(0, a.CompareDepthAcrossFrames(b, firstFrame, secondFrame));
        // The cap selects the upper cut; the curved cylinder selects the lower.
        // Both exact rays exceed the middle ray despite the same rounded source.
        Assert.True(a.CompareDepth(zero) > 0);
        Assert.True(a.TryGetContactAnchor(firstFrame, out _, out bool clamped, out Vector3d point, out _));
        Assert.False(clamped); Assert.Equal(middle, point);
    }

    [Fact]
    public void FullWidthConeCutsMustRetainDistinctDepthsEvenWhenBothRoundToTheAxis()
    {
        Fixed64 extent = Fixed64.MaxValue;
        FixedTriangle plane = new(new Vector3d(-extent, Fixed64.Zero, -extent),
            new(extent, Fixed64.Zero, -extent), new(Fixed64.Zero, Fixed64.Zero, extent));
        ConePlaneRayFrame frame = new(plane, Vector3d.Zero, NegativeIdentity,
            Vector3d.Zero, NegativeIdentity, (Fixed64)4, Fixed64.Two);
        var source = new ConePlaneRayEventSource(new FixedSegment(-Vector3d.Right * extent, Vector3d.Right * extent));
        int endpoints = 0;
        void Check(in ConePlaneRayEventSource s, in ConePlaneRayFrame f, ConePlaneRayEvent e,
            scoped ConePlaneRayPoint p, scoped ReadOnlySpan<ulong> r,
            scoped in ConePlaneRaySelection positive, scoped in ConePlaneRaySelection negative)
        {
            if (e.Kind != ConePlaneRayEventKind.EdgeEndpoint) return;
            endpoints++;
            Span<ulong> pv = stackalloc ulong[ConePlaneRaySelection.StorageWords], nv = stackalloc ulong[ConePlaneRaySelection.StorageWords];
            Span<int> ps = stackalloc int[ConePlaneRaySelection.SignCount], ns = stackalloc int[ConePlaneRaySelection.SignCount];
            Span<ulong> pointValues = stackalloc ulong[ConePlaneRayPoint.StorageWords], root = stackalloc ulong[ConePlaneRaySelection.RootWords];
            Span<int> pointSigns = stackalloc int[ConePlaneRayPoint.SignCount];
            ConePlaneRaySelection axis = new(pv, ps), opposite = new(nv, ns);
            ConePlaneRayPoint point = new(pointValues, pointSigns);
            var axisSource = new ConePlaneRayEventSource(new FixedSegment(Vector3d.Zero, Vector3d.Zero));
            Assert.True(ConePlaneRayEvents.TryEvaluateEvent(axisSource, frame,
                new ConePlaneRayEvent(ConePlaneRayEventKind.EdgeEndpoint, endpoint: 0), point, root, ref axis, ref opposite));
            Assert.True(axis.CompareDepth(positive) > 0);
            Assert.Equal(Fixed64.Two, axis.GetRoundedMaximumDepth());
            Assert.Equal(Fixed64.Two, positive.GetRoundedMaximumDepth());
        }
        Assert.True(ConePlaneRayEvents.VisitEvents(source, frame, FullWidthMiddleInterval(), Check));
        Assert.Equal(2, endpoints);
    }

    [Fact]
    public void DefaultIntervalsMustPreserveExistingConeEventsAndSlabDomains()
    {
        FixedTriangle plane = new(new Vector3d(-4, 4, -4), new(4, -4, -4), new(0, 0, 4));
        FixedSegment segment = new(new Vector3d(-2, 2, 0), new(2, -2, 0));
        ConePlaneRayFrame cone = new(plane, Vector3d.Zero, FixedQuaternion.Identity,
            Vector3d.Zero, FixedQuaternion.Identity, (Fixed64)4, Fixed64.Two);
        var source = new ConePlaneRayEventSource(segment);
        int events = 0;
        void Check(in ConePlaneRayEventSource s, in ConePlaneRayFrame f, ConePlaneRayEvent e,
            scoped ConePlaneRayPoint point, scoped ReadOnlySpan<ulong> root,
            scoped in ConePlaneRaySelection positive, scoped in ConePlaneRaySelection negative)
        {
            events++;
            Span<ulong> pv = stackalloc ulong[ConePlaneRaySelection.StorageWords], nv = stackalloc ulong[ConePlaneRaySelection.StorageWords];
            Span<int> ps = stackalloc int[ConePlaneRaySelection.SignCount], ns = stackalloc int[ConePlaneRaySelection.SignCount];
            Span<ulong> pointValues = stackalloc ulong[ConePlaneRayPoint.StorageWords], replayRoot = stackalloc ulong[ConePlaneRaySelection.RootWords];
            Span<int> pointSigns = stackalloc int[ConePlaneRayPoint.SignCount];
            ConePlaneRaySelection p = new(pv, ps), n = new(nv, ns);
            ConePlaneRayPoint replay = new(pointValues, pointSigns);
            Assert.True(ConePlaneRayEvents.TryEvaluateEvent(source, cone, e, default, replay, replayRoot, ref p, ref n));
            Assert.Equal(0, point.CompareTo(replay, root, replayRoot));
            if (positive.HasValue) Assert.Equal(0, p.CompareDepth(positive));
            if (negative.HasValue) Assert.Equal(0, n.CompareDepth(negative));
        }
        Assert.True(ConePlaneRayEvents.VisitEvents(source, cone, default, Check)); Assert.True(events > 0);
        FiniteSlabRayFrame slab = new(plane, Vector3d.Zero, FixedQuaternion.Identity,
            Vector3d.Zero, FixedQuaternion.Identity, Fixed64.Two, Fixed64.Two, Fixed64.One);
        Span<ulong> av = stackalloc ulong[FiniteSlabRaySelection.StorageWords], bv = stackalloc ulong[FiniteSlabRaySelection.StorageWords];
        Span<sbyte> sa = stackalloc sbyte[FiniteSlabRaySelection.SignCount], sb = stackalloc sbyte[FiniteSlabRaySelection.SignCount];
        FiniteSlabRaySelection a = new(av, sa), b = new(bv, sb);
        FiniteSlabRayExtrema.AccumulateSegment(slab, segment, default, ReadOnlySpan<Vector2d>.Empty, 1, ref a);
        FiniteSlabRayExtrema.AccumulateTriangle(slab, new FixedTriangle(segment.Start, segment.End, segment.End),
            ReadOnlySpan<Vector2d>.Empty, 1, ref b);
        Assert.True(a.HasValue && b.HasValue); Assert.Equal(0, a.CompareDepth(b));
    }

    [Theory]
    [InlineData(0)]
    [InlineData(1)]
    [InlineData(2)]
    public void ConeClipsMustPreserveAxialBoundsOtherSegmentAxesAndSingletons(int geometry)
    {
        bool vertical = geometry == 0;
        FixedTriangle plane = vertical
            ? new(new Vector3d(0, -4, -4), new(0, 4, -4), new(0, 0, 4))
            : new(new Vector3d(-4, 0, -4), new(4, 0, -4), new(0, 0, 4));
        ConePlaneRayFrame frame = new(plane, Vector3d.Zero, FixedQuaternion.Identity,
            Vector3d.Zero, FixedQuaternion.Identity, (Fixed64)4, Fixed64.Two);
        FixedSegment segment = vertical ? new(-Vector3d.Up * (Fixed64)4, Vector3d.Up * (Fixed64)4)
            : geometry == 1 ? new(-Vector3d.Forward * Fixed64.Two, Vector3d.Forward * Fixed64.Two)
            : new(Vector3d.Zero, Vector3d.Zero);
        var source = new ConePlaneRayEventSource(segment);
        RationalSegmentInterval clip = vertical ? Interval(1, 8, 7, 8) : Interval(1, 4, 3, 4);
        int requested = vertical ? 1 : geometry == 1 ? -1 : 0;
        int endpoints = 0;
        void Check(in ConePlaneRayEventSource s, in ConePlaneRayFrame f, ConePlaneRayEvent e,
            scoped ConePlaneRayPoint point, scoped ReadOnlySpan<ulong> root,
            scoped in ConePlaneRaySelection positive, scoped in ConePlaneRaySelection negative)
        {
            if (e.Kind != ConePlaneRayEventKind.EdgeEndpoint) return;
            endpoints++;
            Vector3d expected = vertical ? Vector3d.Up * (e.Endpoint == 0 ? -Fixed64.Two : Fixed64.Two)
                : geometry == 1 ? Vector3d.Forward * (e.Endpoint == 0 ? -Fixed64.One : Fixed64.One) : Vector3d.Zero;
            WideAxis3 native = frame.Transform(expected);
            Span<ulong> expectedValues = stackalloc ulong[4 * ConePlaneRaySelection.FieldWords]; Span<int> expectedSigns = stackalloc int[4];
            ContactQuadratic n = ContactQuadratic.At(expectedValues, expectedSigns, 0, ConePlaneRaySelection.FieldWords);
            ContactQuadratic d = ContactQuadratic.At(expectedValues, expectedSigns, 1, ConePlaneRaySelection.FieldWords);
            d.Set(Signed576.ExtendValue(Signed320.ExtendValue(Signed192.Signed(1))));
            for (int axis = 0; axis < 3; axis++)
            {
                n.Set(Signed576.ExtendValue(axis == 0 ? native.X : axis == 1 ? native.Y : native.Z));
                Assert.Equal(0, ContactQuadratic.CompareRatios(axis == 0 ? point.X : axis == 1 ? point.Y : point.Z,
                    point.Denominator, root, n, d, ReadOnlySpan<ulong>.Empty));
            }
            Span<ulong> pv = stackalloc ulong[ConePlaneRaySelection.StorageWords], nv = stackalloc ulong[ConePlaneRaySelection.StorageWords];
            Span<int> ps = stackalloc int[ConePlaneRaySelection.SignCount], ns = stackalloc int[ConePlaneRaySelection.SignCount];
            Span<ulong> pointValues = stackalloc ulong[ConePlaneRayPoint.StorageWords], replayRoot = stackalloc ulong[ConePlaneRaySelection.RootWords];
            Span<int> pointSigns = stackalloc int[ConePlaneRayPoint.SignCount];
            ConePlaneRaySelection p = new(pv, ps), opposite = new(nv, ns);
            ConePlaneRayPoint replay = new(pointValues, pointSigns);
            Assert.True(ConePlaneRayEvents.TryEvaluateEvent(source, frame, e, clip, replay, replayRoot, ref p, ref opposite, requested));
            Assert.Equal(0, point.CompareTo(replay, root, replayRoot));
            Assert.Equal(requested >= 0, p.HasValue); Assert.Equal(requested <= 0, opposite.HasValue);
            if (p.HasValue) Assert.Equal(0, p.CompareDepth(positive));
            if (opposite.HasValue) Assert.Equal(0, opposite.CompareDepth(negative));
        }
        Assert.True(ConePlaneRayEvents.VisitEvents(source, frame, clip, Check));
        Assert.Equal(2, endpoints);
    }

    [Fact]
    public void ConeClipsMustRejectDisjointAxialRangesAndInvalidReplayArguments()
    {
        FixedTriangle plane = new(new Vector3d(0, -4, -4), new(0, 4, -4), new(0, 0, 4));
        ConePlaneRayFrame frame = new(plane, Vector3d.Zero, FixedQuaternion.Identity,
            Vector3d.Zero, FixedQuaternion.Identity, (Fixed64)4, Fixed64.Two);
        var source = new ConePlaneRayEventSource(new FixedSegment(-Vector3d.Up * (Fixed64)4, Vector3d.Up * (Fixed64)4));
        RationalSegmentInterval clip = Interval(0, 1, 1, 8);
        int events = 0;
        void Visit(in ConePlaneRayEventSource s, in ConePlaneRayFrame f, ConePlaneRayEvent e,
            scoped ConePlaneRayPoint p, scoped ReadOnlySpan<ulong> r,
            scoped in ConePlaneRaySelection positive, scoped in ConePlaneRaySelection negative) => events++;
        bool Replay(ConePlaneRayEventSource input, int orientation)
        {
            Span<ulong> pv = stackalloc ulong[ConePlaneRaySelection.StorageWords], nv = stackalloc ulong[ConePlaneRaySelection.StorageWords];
            Span<int> ps = stackalloc int[ConePlaneRaySelection.SignCount], ns = stackalloc int[ConePlaneRaySelection.SignCount];
            Span<ulong> pointValues = stackalloc ulong[ConePlaneRayPoint.StorageWords], root = stackalloc ulong[ConePlaneRaySelection.RootWords];
            Span<int> pointSigns = stackalloc int[ConePlaneRayPoint.SignCount];
            ConePlaneRaySelection p = new(pv, ps), n = new(nv, ns); ConePlaneRayPoint point = new(pointValues, pointSigns);
            return ConePlaneRayEvents.TryEvaluateEvent(input, frame,
                new ConePlaneRayEvent(ConePlaneRayEventKind.EdgeEndpoint, endpoint: 0), clip, point, root, ref p, ref n, orientation);
        }
        Assert.False(ConePlaneRayEvents.VisitEvents(source, frame, clip, Visit)); Assert.Equal(0, events);
        Assert.False(Replay(source, 0));
        Assert.Throws<ArgumentException>(() => ConePlaneRayEvents.VisitEvents(ConePlaneRayEventSource.Plane, frame, clip, Visit));
        Assert.Throws<ArgumentException>(() => Replay(ConePlaneRayEventSource.Plane, 0));
        Assert.Throws<ArgumentOutOfRangeException>(() => Replay(source, -2));
        Assert.Throws<ArgumentOutOfRangeException>(() => Replay(source, 2));
    }

    internal static RationalSegmentInterval FullWidthMiddleInterval()
    {
        Signed320 d = new(0x7fffffffffffffff, ulong.MaxValue, ulong.MaxValue, ulong.MaxValue, ulong.MaxValue);
        Signed320 lower = new(0x3fffffffffffffff, ulong.MaxValue, ulong.MaxValue, ulong.MaxValue, ulong.MaxValue);
        Signed320 upper = new(1UL << 62, 0, 0, 0, 0);
        return new(lower, d, upper, d);
    }

    private static RationalSegmentInterval Interval(long ln, long ld, long un, long ud) => new(
        Signed320.ExtendValue(Signed192.Signed(ln)), Signed320.ExtendValue(Signed192.Signed(ld)),
        Signed320.ExtendValue(Signed192.Signed(un)), Signed320.ExtendValue(Signed192.Signed(ud)));
}
