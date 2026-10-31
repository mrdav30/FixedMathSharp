using System;
using FixedMathSharp.Geometry;
using Xunit;

namespace FixedMathSharp.Tests.Geometry;

public class ConePrismBoundaryTests
{
    [Fact]
    public void NativeBoundaryRejectsAnExactClearanceBeyondTheScalarDepthRange()
    {
        FixedQuaternion rotation = new FixedQuaternion(Fixed64.Zero, Fixed64.Zero, -Fixed64.One, Fixed64.One).Normalized;
        Vector3d origin = new(Fixed64.Zero, (Fixed64)(-999_999_999), Fixed64.Zero);
        Vector2d[] footprint = { new(-Fixed64.One, -Fixed64.Half), new(-Fixed64.Half, -Fixed64.Half),
            new(-Fixed64.Half, Fixed64.Half), new(-Fixed64.One, Fixed64.Half) };
        Span<WideDirection3d> vertices = stackalloc WideDirection3d[8];
        var geometry = new CurvedPrismContactGeometry(Vector3d.Zero, rotation, Vector3d.Up,
            Fixed64.Two, Fixed64.MaxValue, origin, Fixed64.Zero, footprint, Fixed64.One, true, vertices);
        // The base is x=-1. Its z=0 point is inside the original base disk,
        // but the upward ray from y=-1e9 exits after R+1e9, beyond Fixed64.
        Assert.False(new ConePrismBoundaryContacts().TryGetContact(geometry, -1, default, -Fixed64.One, -Fixed64.One,
            Vector3d.Zero, rotation, origin, out var contact, boundaryEdge: 3));
        Assert.Equal(default, contact);
    }

    [Fact]
    public void NativeBoundaryRequiresABoundedSourceMaterializationChart()
    {
        Fixed64 authored = (Fixed64)1_800_000_000, yaw = Fixed64.PiOver4;
        Vector3d origin = new(Fixed64.Zero, -Fixed64.One, (Fixed64)(-2_000_000_000));
        var frame = new NativePrismYawFrame3d(origin, yaw);
        Assert.True(frame.GetPoint(new(authored, Fixed64.One, authored)).TryGetPoint(out Vector3d center));
        Vector2d[] footprint = { new(authored - Fixed64.Quarter, authored - Fixed64.Quarter), new(authored + Fixed64.Quarter, authored - Fixed64.Quarter),
            new(authored + Fixed64.Quarter, authored + Fixed64.Quarter), new(authored - Fixed64.Quarter, authored + Fixed64.Quarter) };
        Span<WideDirection3d> vertices = stackalloc WideDirection3d[8];
        var geometry = new CurvedPrismContactGeometry(center, FixedQuaternion.Identity, Vector3d.Up,
            Fixed64.Two, Fixed64.One, origin, yaw, footprint, Fixed64.Half, true, vertices);
        // The physical line is near the small cone around world z=.545e9.
        // Its relative z from this unbounded authored origin is about 2.545e9:
        // the exact contact exists but cannot become one local scalar point.
        Assert.False(new ConePrismBoundaryContacts().TryGetContact(geometry, -2, default, Fixed64.Half, Fixed64.Half,
            center, FixedQuaternion.Identity, origin, out var contact, boundaryEdge: 0));
        Assert.Equal(default, contact);
    }

    [Fact]
    public void NativeEventsKeepWideSourceCutsBeforeChoosingTheFiniteConeExit()
    {
        Signed320 one = Signed320.ExtendValue(Signed192.Signed(1));
        Signed832 wide = new(0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0);
        Signed832 denominator = WideArithmetic.MultiplySigned832(wide, Signed192.Signed(3));
        var clip = new RationalSegmentInterval(WideArithmetic.AddSigned832(wide, Signed832.ExtendValue(Signed192.Signed(1))), denominator,
            WideArithmetic.SubtractSigned832(WideArithmetic.AddSigned832(wide, wide), Signed832.ExtendValue(Signed192.Signed(1))), denominator);
        Assert.True(clip.HasWideBounds);
        var normal = new WideDirection3d(default, default, Signed576.ExtendValue(Signed192.Signed(1)));
        var frame = new ConePlaneRayFrame(Fixed64.Two, Fixed64.Half, one, normal, default);
        Assert.False(frame.UsesWideBanks);
        WideAxis3 start = new(Signed320.ExtendValue(Signed192.Raw(-Fixed64.One)), Signed320.ExtendValue(Signed192.Raw(Fixed64.Two)), default);
        WideAxis3 end = new(Signed320.ExtendValue(Signed192.Raw(Fixed64.One)), start.Y, default);
        Fixed64 maximum = Fixed64.Zero; bool visited = false;
        Assert.True(ConePlaneRayEvents.VisitNativeSegment(frame, start, end, clip, Visit));
        Assert.True(visited); Assert.Equal(Fixed64.Quarter, maximum);

        void Visit(in ConePlaneRayEventSource source, in ConePlaneRayFrame nativeFrame, ConePlaneRayEvent descriptor,
            scoped ConePlaneRayPoint point, scoped ReadOnlySpan<ulong> root,
            scoped in ConePlaneRaySelection positive, scoped in ConePlaneRaySelection negative)
        {
            Assert.True(nativeFrame.UsesWideBanks); visited = true;
            if (positive.HasValue) maximum = FixedMath.Max(maximum, positive.GetRoundedMaximumDepth());
        }
    }

    [Theory]
    [InlineData(0, false)]
    [InlineData(0, true)]
    [InlineData(1, false)]
    [InlineData(1, true)]
    [InlineData(2, false)]
    [InlineData(2, true)]
    public void NativeSegmentAdmissionChecksEveryCoordinateAndTheRetainedParameter(int axis, bool reverse)
    {
        WideAxis3 start = Point(0), end = Point(4);
        if (reverse) (start, end) = (end, start);
        Assert.True(Contains(Point(2)));
        Assert.True(Contains(start)); Assert.True(Contains(end));
        Assert.False(Contains(Point(-1))); Assert.False(Contains(Point(5)));
        var clip = new RationalSegmentInterval(Signed320.ExtendValue(Signed192.Signed(1)), Signed320.ExtendValue(Signed192.Signed(4)),
            Signed320.ExtendValue(Signed192.Signed(3)), Signed320.ExtendValue(Signed192.Signed(4)));
        Assert.True(Contains(Point(2), clip)); Assert.False(Contains(start, clip));
        // A point inside the scalar parameter interval can still miss the line.
        var offLine = axis == 0 ? new WideAxis3(Value(2), Value(1), default)
            : axis == 1 ? new WideAxis3(default, Value(2), Value(1)) : new WideAxis3(Value(1), default, Value(2));
        Assert.False(Contains(offLine));

        WideAxis3 Point(int coordinate) => axis == 0 ? new(Value(coordinate), default, default)
            : axis == 1 ? new(default, Value(coordinate), default) : new(default, default, Value(coordinate));
        static Signed320 Value(int value) => Signed320.ExtendValue(Signed192.Signed(value));
        bool Contains(WideAxis3 position, RationalSegmentInterval interval = default)
        {
            Span<ulong> values = stackalloc ulong[ConePlaneRayPoint.StorageWords];
            Span<int> signs = stackalloc int[ConePlaneRayPoint.SignCount];
            Span<ulong> root = stackalloc ulong[1]; root.Clear();
            var point = new ConePlaneRayPoint(values, signs); point.Set(position);
            return ConePlaneRayEvents.ContainsNativeSegmentPoint(start, end, point, root, interval);
        }
    }

    [Fact]
    public void NativeSingletonAdmissionRequiresTheSameExactPoint()
    {
        Span<ulong> values = stackalloc ulong[ConePlaneRayPoint.StorageWords];
        Span<int> signs = stackalloc int[ConePlaneRayPoint.SignCount];
        Span<ulong> root = stackalloc ulong[1]; root.Clear();
        var point = new ConePlaneRayPoint(values, signs); point.Set(default(WideAxis3));
        Assert.True(ConePlaneRayEvents.ContainsNativeSegmentPoint(default, default, point, root, default));
        point.Set(new WideAxis3(default, Signed320.ExtendValue(Signed192.Signed(1)), default));
        Assert.False(ConePlaneRayEvents.ContainsNativeSegmentPoint(default, default, point, root, default));
    }

    [Theory]
    [InlineData(false)]
    [InlineData(true)]
    public void CutSideWindowRetainsTheExactBaseRimBeforeSelectingItsExit(bool translated)
    {
        Vector3d origin = translated ? new(Fixed64.MaxValue - (Fixed64)4, Fixed64.Zero, Fixed64.Zero) : Vector3d.Zero;
        Vector3d center = origin + new Vector3d(Fixed64.Zero, Fixed64.Zero, Fixed64.Quarter);
        Vector2d[] footprint = { new(-2, -2), new(2, -2), new(2, 0), new(-2, 0) };
        Span<WideDirection3d> vertices = stackalloc WideDirection3d[8];
        var geometry = new CurvedPrismContactGeometry(center, FixedQuaternion.Identity, Vector3d.Up,
            Fixed64.Two, Fixed64.Half, origin, Fixed64.Zero, footprint, Fixed64.Two, true, vertices);
        var clip = new RationalSegmentInterval(Signed320.ExtendValue(Signed192.Signed(13)), Signed320.ExtendValue(Signed192.Signed(32)),
            Signed320.ExtendValue(Signed192.Signed(7)), Signed320.ExtendValue(Signed192.Signed(16)));
        Assert.False(CurvedPrismContact.TryGetFaceSupportContact(geometry, 2, clip, -Fixed64.One, -Fixed64.One,
            center, FixedQuaternion.Identity, origin, false, out _, out _));
        var worker = new ConePrismBoundaryContacts();
        Assert.True(worker.TryGetContact(geometry, 2, clip, -Fixed64.One, -Fixed64.One,
            center, FixedQuaternion.Identity, origin, out FixedContactAnchors contact));
        Assert.True(contact.SecondAnchor.TryGetPoint(out Vector3d source));
        Assert.True(contact.FirstAnchor.TryGetPoint(out Vector3d exit));
        Assert.Equal(origin.X + Fixed64.Quarter, source.X); Assert.Equal(-Fixed64.One, source.Y);
        Assert.Equal(Fixed64.Zero, source.Z); Assert.Equal(source.X, exit.X); Assert.Equal(source.Y, exit.Y);
        Fixed64 expected = FixedMath.Sqrt((Fixed64)3) / (Fixed64)4 - Fixed64.Quarter;
        Assert.True(FixedMath.Abs(contact.Depth - expected) <= Fixed64.FromRaw(3));
        Assert.True(FixedMath.Abs(exit.Z + expected) <= Fixed64.FromRaw(3));
        Assert.Equal(Vector3d.Backward, contact.Normal); Assert.False(contact.DepthIsClamped);
    }

    [Fact]
    public void CutSideWindowOutsideTheOriginalConeDoesNotCreateAResponse()
    {
        Vector3d center = new(Fixed64.Zero, Fixed64.Zero, Fixed64.Quarter);
        Vector2d[] footprint = { new(-2, -2), new(2, -2), new(2, 0), new(-2, 0) };
        Span<WideDirection3d> vertices = stackalloc WideDirection3d[8];
        var geometry = new CurvedPrismContactGeometry(center, FixedQuaternion.Identity, Vector3d.Up,
            Fixed64.Two, Fixed64.Half, Vector3d.Zero, Fixed64.Zero, footprint, Fixed64.Two, true, vertices);
        var clip = new RationalSegmentInterval(Signed320.ExtendValue(Signed192.Signed(1)), Signed320.ExtendValue(Signed192.Signed(4)),
            Signed320.ExtendValue(Signed192.Signed(5)), Signed320.ExtendValue(Signed192.Signed(16)));
        Assert.False(new ConePrismBoundaryContacts().TryGetContact(geometry, 2, clip, -Fixed64.One, -Fixed64.One,
            center, FixedQuaternion.Identity, Vector3d.Zero, out FixedContactAnchors contact));
        Assert.Equal(default, contact);
    }
    [Fact]
    public void CapBoundaryLineRetainsItsStationaryConeSideExit()
    {
        Vector3d center = new(Fixed64.Zero, Fixed64.Quarter, Fixed64.Zero);
        Vector3d origin = new(Fixed64.Zero, Fixed64.Two, Fixed64.Zero);
        Vector2d[] footprint = { new(Fixed64.Quarter, -Fixed64.Quarter), new(Fixed64.FromFraction(3, 8), -Fixed64.Quarter),
            new(Fixed64.FromFraction(3, 8), Fixed64.Quarter), new(Fixed64.Quarter, Fixed64.Quarter) };
        Span<WideDirection3d> vertices = stackalloc WideDirection3d[8];
        var geometry = new CurvedPrismContactGeometry(center, FixedQuaternion.Identity, Vector3d.Up,
            Fixed64.Two, Fixed64.Half, origin, Fixed64.Zero, footprint, Fixed64.Two, true, vertices);
        Assert.False(CurvedPrismContact.TryGetFaceSupportContact(geometry, -1, default, -Fixed64.Two, Fixed64.Two,
            center, FixedQuaternion.Identity, origin, false, out _, out _));
        Assert.True(new ConePrismBoundaryContacts().TryGetContact(geometry, -1, default, -Fixed64.Two, Fixed64.Two,
            center, FixedQuaternion.Identity, origin, out FixedContactAnchors contact, boundaryEdge: 3));
        Assert.True(contact.SecondAnchor.TryGetPoint(out Vector3d source));
        Assert.True(contact.FirstAnchor.TryGetPoint(out Vector3d exit));
        Assert.Equal(new Vector3d(Fixed64.Quarter, Fixed64.Zero, Fixed64.Zero), source);
        Assert.Equal(new Vector3d(Fixed64.Quarter, Fixed64.Quarter, Fixed64.Zero), exit);
        Assert.Equal(Fixed64.Quarter, contact.Depth); Assert.Equal(Vector3d.Up, contact.Normal);
    }
}
