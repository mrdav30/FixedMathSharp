using System;
using FixedMathSharp.Geometry;
using Xunit;

namespace FixedMathSharp.Tests;

public sealed class FixedSegmentFiniteConeSurfaceClippingTests
{
    [Theory]
    [InlineData(false)]
    [InlineData(true)]
    public void ClippedBaseFamily_SelectsTheClippedMidpointBeforeMaterialization(bool reverse)
    {
        var segment = new FixedSegment(new Vector3d(-1, -1, 0), new Vector3d(1, -1, 0));
        if (reverse) segment = new FixedSegment(segment.End, segment.Start);
        SegmentConeSurfaceCandidate candidate = Find(segment, ConeSurfaceFeature.Base, ConeSurfaceFamily.SegmentInterval);
        var clip = Interval(1, 3, 1, 2);
        Span<ConeSurfaceFamilyEvent> storage = stackalloc ConeSurfaceFamilyEvent[3];
        var selected = new ConeSurfaceFamilySelection(storage);
        Assert.True(candidate.AccumulateFamilyEvents(ReadOnlySpan<WideAxis3>.Empty,
            ConeSurfacePointLocation.Interior, clip, ref selected));
        Assert.Equal(1, selected.Count);
        FixedContactAnchors contact = selected[0].GetContact(candidate, ReadOnlySpan<WideAxis3>.Empty, clip);
        Assert.True(selected[0].TryGetWorldAnchors(candidate, ReadOnlySpan<WideAxis3>.Empty, clip, out Vector3d first, out Vector3d second));
        Fixed64 x = Fixed64.FromFraction(reverse ? 1 : -1, 6);
        Assert.Equal(new Vector3d(x, -Fixed64.One, Fixed64.Zero), first);
        Assert.Equal(new Vector3d(x, -Fixed64.Two, Fixed64.Zero), second);
        Assert.Equal(first, contact.FirstAnchor.LocalPoint);
        Assert.Equal(second, contact.SecondAnchor.LocalPoint);
        Assert.Equal(Fixed64.One, contact.Depth);
        Assert.Equal(Vector3d.Up, contact.Normal);
    }

    [Theory]
    [InlineData(false)]
    [InlineData(true)]
    public void ClippedParallelSideFamily_UsesTheRetainedFiniteInterval(bool reverse)
    {
        var segment = new FixedSegment(new Vector3d(Fixed64.Zero, Fixed64.FromFraction(3, 4), Fixed64.Zero),
            new Vector3d(Fixed64.Half, -Fixed64.Quarter, Fixed64.Zero));
        if (reverse) segment = new FixedSegment(segment.End, segment.Start);
        SegmentConeSurfaceCandidate candidate = Find(segment, ConeSurfaceFeature.Side, ConeSurfaceFamily.SegmentInterval);
        var clip = Interval(1, 4, 3, 4);
        Span<ConeSurfaceFamilyEvent> storage = stackalloc ConeSurfaceFamilyEvent[3];
        var selected = new ConeSurfaceFamilySelection(storage);
        Assert.True(candidate.AccumulateFamilyEvents(ReadOnlySpan<WideAxis3>.Empty,
            ConeSurfacePointLocation.Interior, clip, ref selected));
        Assert.Equal(1, selected.Count);
        FixedContactAnchors contact = selected[0].GetContact(candidate, ReadOnlySpan<WideAxis3>.Empty, clip);
        Assert.Equal(new Vector3d(Fixed64.Quarter, Fixed64.Quarter, Fixed64.Zero), contact.FirstAnchor.LocalPoint);
        Assert.Equal(new Vector3d(Fixed64.FromFraction(3, 4), Fixed64.Half, Fixed64.Zero), contact.SecondAnchor.LocalPoint);
        Assert.Equal(Fixed64.FromRaw(2400959709L), contact.Depth);
    }

    [Theory]
    [InlineData(false)]
    [InlineData(true)]
    public void ClippedFamilies_KeepClosedEndpointsButDoNotPromoteThemToEdgeInteriors(bool upper)
    {
        SegmentConeSurfaceCandidate candidate = Find(new FixedSegment(new Vector3d(-1, -1, 0), new Vector3d(1, -1, 0)),
            ConeSurfaceFeature.Base, ConeSurfaceFamily.SegmentInterval);
        var clip = Interval(1, 4, 3, 4);
        Span<ConeSurfaceFamilyEvent> storage = stackalloc ConeSurfaceFamilyEvent[3];
        var selected = new ConeSurfaceFamilySelection(storage);
        Assert.True(candidate.AccumulateFamilyEvents(ReadOnlySpan<WideAxis3>.Empty,
            upper ? ConeSurfacePointLocation.End : ConeSurfacePointLocation.Start, clip, ref selected));
        Assert.Equal(1, selected.Count);
        Assert.Equal(new Vector3d(upper ? Fixed64.Half : -Fixed64.Half, -Fixed64.One, Fixed64.Zero),
            selected[0].GetContact(candidate, ReadOnlySpan<WideAxis3>.Empty, clip).FirstAnchor.LocalPoint);
        Assert.Throws<ArgumentException>(() => candidate.IsInInterval(clip));
    }

    [Fact]
    public void EmptyAndSingletonFamilyInteriors_AreRejectedBeforeSelection()
    {
        SegmentConeSurfaceCandidate candidate = Find(new FixedSegment(new Vector3d(-4, -1, 0), new Vector3d(4, -1, 0)),
            ConeSurfaceFeature.Base, ConeSurfaceFamily.SegmentInterval);
        Span<ConeSurfaceFamilyEvent> storage = stackalloc ConeSurfaceFamilyEvent[3];
        var selected = new ConeSurfaceFamilySelection(storage);
        Assert.False(candidate.AccumulateFamilyEvents(ReadOnlySpan<WideAxis3>.Empty,
            ConeSurfacePointLocation.Interior, Interval(0, 1, 1, 8), ref selected));
        Assert.Equal(0, selected.Count);
        Assert.False(candidate.AccumulateFamilyEvents(ReadOnlySpan<WideAxis3>.Empty,
            ConeSurfacePointLocation.Interior, Interval(1, 2, 1, 2), ref selected));
        Assert.Equal(0, selected.Count);
    }

    [Fact]
    public void UniqueNormalFamily_ClipsItsExactPointBeforeNormalSelectionAndReplay()
    {
        Span<SegmentConeSurfaceCandidate> storage = stackalloc SegmentConeSurfaceCandidate[SegmentConeSurfaceCandidates.MaximumCandidates];
        var candidates = new ConeSurfaceSelection(storage);
        Assert.True(SegmentConeSurfaceCandidates.AccumulateSegmentConeSurfaceCandidates(
            new FixedSegment(-Vector3d.Right, Vector3d.Right), Vector3d.Zero, FixedQuaternion.Identity,
            Vector3d.Zero, FixedQuaternion.Identity, (Fixed64)4, Fixed64.Zero, ref candidates));
        bool found = false;
        for (int index = 0; index < candidates.Count; index++)
        {
            SegmentConeSurfaceCandidate candidate = candidates[index];
            if (candidate.Feature != ConeSurfaceFeature.Side || candidate.Family != ConeSurfaceFamily.NormalCone) continue;
            found = true;
            var events = new ConeSurfaceFamilyEvent[candidate.GetMaximumFamilyEventCount(0)];
            var selected = new ConeSurfaceFamilySelection(events);
            var clip = Interval(1, 4, 3, 4);
            Assert.True(candidate.AccumulateFamilyEvents(ReadOnlySpan<WideAxis3>.Empty,
                ConeSurfacePointLocation.Interior, clip, ref selected));
            Assert.True(selected.Count > 0);
            ConeSurfaceFamilyEvent descriptor = selected[0];
            FixedContactAnchors contact = descriptor.GetContact(candidate, ReadOnlySpan<WideAxis3>.Empty, clip);
            Assert.Equal(Vector3d.Zero, contact.FirstAnchor.LocalPoint);
            Assert.Equal(Vector3d.Zero, contact.SecondAnchor.LocalPoint);
            Assert.Equal(Fixed64.Zero, contact.Depth);
            var outside = Interval(0, 1, 1, 4);
            Assert.False(descriptor.TryGetWorldAnchors(candidate, ReadOnlySpan<WideAxis3>.Empty, outside, out _, out _));
            Assert.Throws<InvalidOperationException>(() => descriptor.GetContact(candidate, ReadOnlySpan<WideAxis3>.Empty, outside));
            selected = new ConeSurfaceFamilySelection(events);
            Assert.False(candidate.AccumulateFamilyEvents(ReadOnlySpan<WideAxis3>.Empty,
                ConeSurfacePointLocation.Interior, outside, ref selected));
        }
        Assert.True(found);
    }

    [Theory]
    [InlineData(false)]
    [InlineData(true)]
    public void UniqueNormalFamily_OriginalInteriorBecomesTheClippedEndpoint(bool upper)
    {
        Span<SegmentConeSurfaceCandidate> storage = stackalloc SegmentConeSurfaceCandidate[SegmentConeSurfaceCandidates.MaximumCandidates];
        var candidates = new ConeSurfaceSelection(storage);
        Assert.True(SegmentConeSurfaceCandidates.AccumulateSegmentConeSurfaceCandidates(
            new FixedSegment(-Vector3d.Right, Vector3d.Right), Vector3d.Zero, FixedQuaternion.Identity,
            Vector3d.Zero, FixedQuaternion.Identity, (Fixed64)4, Fixed64.Zero, ref candidates));
        var clip = upper ? Interval(1, 4, 1, 2) : Interval(1, 2, 3, 4);
        ConeSurfacePointLocation endpoint = upper ? ConeSurfacePointLocation.End : ConeSurfacePointLocation.Start;
        bool found = false;
        for (int index = 0; index < candidates.Count; index++)
        {
            SegmentConeSurfaceCandidate candidate = candidates[index];
            if (candidate.Feature != ConeSurfaceFeature.Side || candidate.Family != ConeSurfaceFamily.NormalCone) continue;
            found = true;
            Assert.Equal(ConeSurfacePointLocation.Interior, candidate.PointLocation);
            var events = new ConeSurfaceFamilyEvent[candidate.GetMaximumFamilyEventCount(0)];
            var selected = new ConeSurfaceFamilySelection(events);
            Assert.False(candidate.AccumulateFamilyEvents(ReadOnlySpan<WideAxis3>.Empty, endpoint, ref selected));
            Assert.False(candidate.AccumulateFamilyEvents(ReadOnlySpan<WideAxis3>.Empty,
                ConeSurfacePointLocation.Interior, clip, ref selected));
            Assert.True(candidate.AccumulateFamilyEvents(ReadOnlySpan<WideAxis3>.Empty, endpoint, clip, ref selected));
            bool retainsEndpointDirection = false;
            for (int item = 0; item < selected.Count; item++)
            {
                FixedContactAnchors contact = selected[item].GetContact(candidate, ReadOnlySpan<WideAxis3>.Empty, clip);
                Assert.Equal(Vector3d.Zero, contact.FirstAnchor.LocalPoint);
                Assert.Equal(Vector3d.Zero, contact.SecondAnchor.LocalPoint);
                Assert.Equal(Fixed64.Zero, contact.Depth);
                Assert.True(upper ? contact.Normal.X >= Fixed64.Zero : contact.Normal.X <= Fixed64.Zero);
                retainsEndpointDirection |= contact.Normal == (upper ? Vector3d.Right : -Vector3d.Right);
                Assert.True(selected[item].TryGetWorldAnchors(candidate, ReadOnlySpan<WideAxis3>.Empty,
                    clip, out Vector3d first, out Vector3d second));
                Assert.Equal(Vector3d.Zero, first); Assert.Equal(Vector3d.Zero, second);
            }
            // A real endpoint permits normals excluded by the original interior's tangent constraint.
            Assert.True(retainsEndpointDirection);
        }
        Assert.True(found);
    }

    [Theory]
    [InlineData(false)]
    [InlineData(true)]
    public void QuarticRimClipping_PreservesExactMidpointIncidence(bool reverse)
    {
        var segment = new FixedSegment(
            new Vector3d(Fixed64.FromFraction(188, 64), Fixed64.FromFraction(-1281, 256), Fixed64.FromFraction(249, 64)),
            new Vector3d(Fixed64.FromFraction(190, 64), Fixed64.FromFraction(-1249, 256), Fixed64.FromFraction(255, 64)));
        if (reverse) segment = new FixedSegment(segment.End, segment.Start);
        Span<SegmentConeSurfaceCandidate> storage = stackalloc SegmentConeSurfaceCandidate[SegmentConeSurfaceCandidates.MaximumCandidates];
        var selected = new ConeSurfaceSelection(storage);
        Assert.True(SegmentConeSurfaceCandidates.AccumulateSegmentConeSurfaceCandidates(segment,
            Vector3d.Zero, FixedQuaternion.Identity, Vector3d.Zero, FixedQuaternion.Identity, (Fixed64)10, (Fixed64)5, ref selected));
        bool found = false;
        for (int index = 0; index < selected.Count; index++)
        {
            SegmentConeSurfaceCandidate candidate = selected[index];
            if (candidate.Feature != ConeSurfaceFeature.Rim || candidate.Chart < 0 || !candidate.IsSegmentInterior
                || candidate.GetContact().Depth != Fixed64.FromFraction(25, 256)) continue;
            found = true;
            Assert.True(candidate.IsInInterval(Interval(0, 1, 1, 2)));
            Assert.False(candidate.IsInInterval(Interval(0, 1, 1, 2), interiorOnly: true));
            Assert.True(candidate.IsInInterval(Interval(1, 2, 1, 1)));
            Assert.False(candidate.IsInInterval(Interval(1, 2, 1, 1), interiorOnly: true));
            Assert.True(candidate.IsInInterval(Interval(1, 4, 3, 4), interiorOnly: true));
            Assert.False(candidate.IsInInterval(Interval(0, 1, 1, 4)));
            Assert.False(candidate.IsInInterval(Interval(3, 4, 1, 1)));
        }
        Assert.True(found);
    }

    [Theory]
    [InlineData(0)]
    [InlineData(1)]
    [InlineData(2)]
    [InlineData(3)]
    public void AnalyticClipping_UsesTheOriginalExactMidpoint(int kind)
    {
        FixedSegment segment = kind switch
        {
            0 => new FixedSegment(new Vector3d(-1, 0, 0), new Vector3d(1, 0, 0)),
            1 => new FixedSegment(new Vector3d(Fixed64.Half, -Fixed64.Quarter, -Fixed64.Half),
                new Vector3d(Fixed64.Half, -Fixed64.Quarter, Fixed64.Half)),
            2 => new FixedSegment(new Vector3d(-1, -1, 0), new Vector3d(1, 1, 0)),
            _ => new FixedSegment(new Vector3d(Fixed64.Quarter, -Fixed64.Three, Fixed64.Zero),
                new Vector3d(Fixed64.Quarter, -Fixed64.One, Fixed64.Zero))
        };
        Span<SegmentConeSurfaceCandidate> storage = stackalloc SegmentConeSurfaceCandidate[SegmentConeSurfaceCandidates.MaximumCandidates];
        var selected = new ConeSurfaceSelection(storage);
        Assert.True(SegmentConeSurfaceCandidates.AccumulateSegmentConeSurfaceCandidates(segment,
            Vector3d.Zero, FixedQuaternion.Identity, Vector3d.Zero, FixedQuaternion.Identity, (Fixed64)4,
            kind == 0 ? Fixed64.Zero : Fixed64.Two, ref selected));
        bool found = false;
        for (int index = 0; index < selected.Count; index++)
        {
            SegmentConeSurfaceCandidate candidate = selected[index];
            if (candidate.Family == ConeSurfaceFamily.SegmentInterval) continue;
            Assert.True(candidate.IsInInterval(default));
            Assert.Equal(candidate.IsSegmentInterior, candidate.IsInInterval(default, interiorOnly: true));
            if (!candidate.IsSegmentInterior)
            {
                Assert.False(candidate.IsInInterval(Interval(1, 4, 3, 4)));
                Assert.Equal(candidate.PointLocation == ConeSurfacePointLocation.Start,
                    candidate.IsInInterval(Interval(0, 1, 1, 2)));
                Assert.Equal(candidate.PointLocation == ConeSurfacePointLocation.End,
                    candidate.IsInInterval(Interval(1, 2, 1, 1)));
                Assert.False(candidate.IsInInterval(Interval(0, 1, 1, 2), interiorOnly: true));
                Assert.False(candidate.IsInInterval(Interval(1, 2, 1, 1), interiorOnly: true));
                continue;
            }
            if (candidate.Family != ConeSurfaceFamily.None && !(kind == 0 && candidate.Feature == ConeSurfaceFeature.Side)) continue;
            if (candidate.Family == ConeSurfaceFamily.None
                && candidate.GetContact().FirstAnchor.LocalPoint != (segment.Start + segment.End) * Fixed64.Half) continue;
            found = true;
            Assert.True(candidate.IsInInterval(Interval(0, 1, 1, 2)));
            Assert.False(candidate.IsInInterval(Interval(0, 1, 1, 2), interiorOnly: true));
            Assert.True(candidate.IsInInterval(Interval(1, 2, 1, 1)));
            Assert.False(candidate.IsInInterval(Interval(1, 2, 1, 1), interiorOnly: true));
            Assert.True(candidate.IsInInterval(Interval(1, 4, 3, 4), interiorOnly: true));
            Assert.False(candidate.IsInInterval(Interval(0, 1, 1, 4)));
            Assert.False(candidate.IsInInterval(Interval(3, 4, 1, 1)));
        }
        // The zero-radius crossing is a normal family rather than an isolated point.
        Assert.True(kind == 0 || found);
    }

    private static SegmentConeSurfaceCandidate Find(FixedSegment segment, ConeSurfaceFeature feature, ConeSurfaceFamily family)
    {
        Span<SegmentConeSurfaceCandidate> storage = stackalloc SegmentConeSurfaceCandidate[SegmentConeSurfaceCandidates.MaximumCandidates];
        var selected = new ConeSurfaceSelection(storage);
        Assert.True(SegmentConeSurfaceCandidates.AccumulateSegmentConeSurfaceCandidates(segment,
            Vector3d.Zero, FixedQuaternion.Identity, Vector3d.Zero, FixedQuaternion.Identity, (Fixed64)4, Fixed64.Two, ref selected));
        for (int index = 0; index < selected.Count; index++)
            if (selected[index].Feature == feature && selected[index].Family == family) return selected[index];
        throw new InvalidOperationException("The fixture did not retain the expected finite cone stratum.");
    }

    private static RationalSegmentInterval Interval(long lowerN, long lowerD, long upperN, long upperD) =>
        new(Signed320.ExtendValue(Signed192.Signed(lowerN)), Signed320.ExtendValue(Signed192.Signed(lowerD)),
            Signed320.ExtendValue(Signed192.Signed(upperN)), Signed320.ExtendValue(Signed192.Signed(upperD)));
}
