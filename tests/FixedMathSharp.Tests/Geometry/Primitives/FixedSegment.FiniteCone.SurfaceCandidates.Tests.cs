using System;
using FixedMathSharp.Geometry;
using Xunit;

namespace FixedMathSharp.Tests;

public sealed class FixedSegmentFiniteConeSurfaceCandidatesTests
{
    [Theory]
    [InlineData(-2, false, false)]
    [InlineData(-1, false, false)]
    [InlineData(0, false, false)]
    [InlineData(2, false, false)]
    [InlineData(0, true, false)]
    [InlineData(-1, false, true)]
    public void SurfaceCandidateDepths_AxisFamiliesFollowIndependentSquaredDistanceOracle(int y, bool zeroRadius, bool moved)
    {
        Vector3d point = Vector3d.Up * y, translation = moved ? new Vector3d(7, -3, 2) : Vector3d.Zero;
        FixedQuaternion rotation = moved ? new FixedQuaternion(Fixed64.Zero, Fixed64.One, Fixed64.Zero, Fixed64.Zero) : FixedQuaternion.Identity;
        Span<SegmentConeSurfaceCandidate> storage = stackalloc SegmentConeSurfaceCandidate[SegmentConeSurfaceCandidates.MaximumCandidates];
        var selection = new ConeSurfaceSelection(storage);
        Assert.True(SegmentConeSurfaceCandidates.AccumulateSegmentConeSurfaceCandidates(new FixedSegment(point, point),
            translation, rotation, translation, rotation, (Fixed64)4, zeroRadius ? Fixed64.Zero : Fixed64.Two, ref selection));
        for (int first = 0; first < selection.Count; first++)
        for (int second = 0; second < selection.Count; second++)
        {
            (int a, int b) = AxisSquaredDepth(selection[first].Feature, y, zeroRadius);
            (int c, int d) = AxisSquaredDepth(selection[second].Feature, y, zeroRadius);
            int expected = Math.Sign(a * d - c * b);
            Assert.Equal(expected, SegmentConeSurfaceCandidates.CompareDepths(selection[first], selection[second]));
        }
    }

    [Theory]
    [InlineData(false)]
    [InlineData(true)]
    public void SurfaceCandidateDepths_RimChartAndAnalyticEndpointRetainAnExactTie(bool reverse)
    {
        Vector3d p = new(1, -1, 0), q = new(2, -2, 0);
        Vector3d other = p + new Vector3d(Fixed64.Quarter, Fixed64.Quarter, Fixed64.Quarter);
        Span<SegmentConeSurfaceCandidate> storage = stackalloc SegmentConeSurfaceCandidate[SegmentConeSurfaceCandidates.MaximumCandidates];
        var selection = new ConeSurfaceSelection(storage);
        Assert.True(Accumulate(reverse ? new FixedSegment(other, p) : new FixedSegment(p, other), Vector3d.Zero, ref selection));
        int analytic = -1, quartic = -1;
        for (int index = 0; index < selection.Count; index++)
        {
            SegmentConeSurfaceCandidate candidate = selection[index];
            if (candidate.Feature != ConeSurfaceFeature.Rim || candidate.Family != ConeSurfaceFamily.None) continue;
            FixedContactAnchors contact = candidate.GetContact();
            Assert.True(contact.FirstAnchor.TryGetPoint(out Vector3d a)); Assert.True(contact.SecondAnchor.TryGetPoint(out Vector3d b));
            if (a != p || b != q) continue;
            if (candidate.Chart >= 0) quartic = index;
            else analytic = index;
        }
        Assert.True(analytic >= 0 && quartic >= 0);
        Assert.Equal(0, SegmentConeSurfaceCandidates.CompareDepths(selection[analytic], selection[quartic]));
        Assert.Equal(0, SegmentConeSurfaceCandidates.CompareDepths(selection[quartic], selection[analytic]));
        Assert.Equal(0, SegmentConeSurfaceCandidates.CompareDepths(selection[quartic], selection[quartic]));
    }

    [Theory]
    [InlineData(false)]
    [InlineData(true)]
    public void SurfaceCandidateDepths_QuarticAndMeridionalInventoryPreserveExactOrderAndSymmetry(bool meridional)
    {
        Vector3d a = new(-1, -1, 0), b = new(1, 1, meridional ? 0 : 1);
        Span<SegmentConeSurfaceCandidate> storage = stackalloc SegmentConeSurfaceCandidate[SegmentConeSurfaceCandidates.MaximumCandidates];
        var selection = new ConeSurfaceSelection(storage);
        Assert.True(Accumulate(new FixedSegment(a, b), Vector3d.Zero, ref selection));
        int distinct = 0, rim = 0;
        for (int first = 0; first < selection.Count; first++)
        {
            SegmentConeSurfaceCandidate candidate = selection[first];
            Assert.Equal(0, SegmentConeSurfaceCandidates.CompareDepths(candidate, candidate));
            if (candidate.Family != ConeSurfaceFamily.None) continue;
            if (candidate.Feature == ConeSurfaceFeature.Rim) rim++;
            Fixed64 depth = candidate.GetContact().Depth;
            for (int second = first + 1; second < selection.Count; second++)
            {
                SegmentConeSurfaceCandidate other = selection[second];
                int comparison = SegmentConeSurfaceCandidates.CompareDepths(candidate, other);
                Assert.Equal(-comparison, SegmentConeSurfaceCandidates.CompareDepths(other, candidate));
                if (other.Family != ConeSurfaceFamily.None) continue;
                Fixed64 otherDepth = other.GetContact().Depth;
                // Distinct rounded outputs cannot reverse an exact ordering.
                // Exact-equality obligations are asserted separately above.
                if (depth != otherDepth) { Assert.Equal(Math.Sign(depth.CompareTo(otherDepth)), comparison); distinct++; }
            }
        }
        Assert.True(rim > 0 && distinct > 0);
    }

    private static (int Numerator, int Denominator) AxisSquaredDepth(ConeSurfaceFeature feature, int y, bool zeroRadius) => feature switch
    {
        ConeSurfaceFeature.Apex => ((2 - y) * (2 - y), 1),
        ConeSurfaceFeature.Base => ((2 + y) * (2 + y), 1),
        ConeSurfaceFeature.Side => zeroRadius ? (0, 1) : ((2 - y) * (2 - y), 5),
        _ => (4 + (2 + y) * (2 + y), 1)
    };

    [Theory]
    [InlineData(false)]
    [InlineData(true)]
    public void SurfaceCandidateDepths_AxialEndpointsUseTheAdmittedSourcePoint(bool reverse)
    {
        Vector3d low = -Vector3d.Up, high = Vector3d.Zero;
        var segment = reverse ? new FixedSegment(high, low) : new FixedSegment(low, high);
        Span<SegmentConeSurfaceCandidate> storage = stackalloc SegmentConeSurfaceCandidate[SegmentConeSurfaceCandidates.MaximumCandidates];
        var selection = new ConeSurfaceSelection(storage);
        Assert.True(Accumulate(segment, Vector3d.Zero, ref selection));
        for (int first = 0; first < selection.Count; first++)
        for (int second = 0; second < selection.Count; second++)
        {
            SegmentConeSurfaceCandidate a = selection[first], b = selection[second];
            int aY = (int)(a.RootOrdinal == 1 ? segment.End.Y : segment.Start.Y);
            int bY = (int)(b.RootOrdinal == 1 ? segment.End.Y : segment.Start.Y);
            (int an, int ad) = AxisSquaredDepth(a.Feature, aY, false);
            (int bn, int bd) = AxisSquaredDepth(b.Feature, bY, false);
            Assert.Equal(Math.Sign(an * bd - bn * ad), SegmentConeSurfaceCandidates.CompareDepths(a, b));
        }
    }

    [Theory]
    [InlineData(false)]
    [InlineData(true)]
    public void SurfaceCandidateDepths_ExactQuarticTouchOrdersBeforeEveryPositiveExit(bool reverse)
    {
        Vector3d rim = new(2, -2, 0), outside = rim + new Vector3d(Fixed64.Quarter, Fixed64.Quarter, Fixed64.Quarter);
        var segment = reverse ? new FixedSegment(outside, rim) : new FixedSegment(rim, outside);
        Span<SegmentConeSurfaceCandidate> storage = stackalloc SegmentConeSurfaceCandidate[SegmentConeSurfaceCandidates.MaximumCandidates];
        var selection = new ConeSurfaceSelection(storage);
        Assert.True(Accumulate(segment, Vector3d.Zero, ref selection));
        int zeroRoot = -1, positive = -1;
        for (int index = 0; index < selection.Count; index++)
        {
            SegmentConeSurfaceCandidate candidate = selection[index];
            if (candidate.Family != ConeSurfaceFamily.None) continue;
            Fixed64 depth = candidate.GetContact().Depth;
            if (candidate.Feature == ConeSurfaceFeature.Rim && candidate.Chart >= 0 && depth == Fixed64.Zero) zeroRoot = index;
            if (depth > Fixed64.Zero && candidate.Chart < 0) positive = index;
        }
        Assert.True(zeroRoot >= 0 && positive >= 0);
        Assert.Equal(0, SegmentConeSurfaceCandidates.CompareDepths(selection[zeroRoot], selection[zeroRoot]));
        Assert.Equal(-1, SegmentConeSurfaceCandidates.CompareDepths(selection[zeroRoot], selection[positive]));
        Assert.Equal(1, SegmentConeSurfaceCandidates.CompareDepths(selection[positive], selection[zeroRoot]));
    }

    [Theory]
    [InlineData(false, false)]
    [InlineData(true, false)]
    [InlineData(false, true)]
    [InlineData(true, true)]
    public void SurfaceCandidates_PointLocation_FixedFamiliesRetainTheirEndpoint(bool reverse, bool zeroRadius)
    {
        var other = new Vector3d(1, 1, 0);
        var segment = reverse ? new FixedSegment(other, Vector3d.Zero) : new FixedSegment(Vector3d.Zero, other);
        Span<SegmentConeSurfaceCandidate> storage = stackalloc SegmentConeSurfaceCandidate[SegmentConeSurfaceCandidates.MaximumCandidates];
        var selection = new ConeSurfaceSelection(storage);
        Assert.True(SegmentConeSurfaceCandidates.AccumulateSegmentConeSurfaceCandidates(segment,
            Vector3d.Zero, FixedQuaternion.Identity, Vector3d.Zero, FixedQuaternion.Identity, (Fixed64)4,
            zeroRadius ? Fixed64.Zero : Fixed64.Two, ref selection));
        int found = 0;
        for (int index = 0; index < selection.Count; index++)
        {
            SegmentConeSurfaceCandidate candidate = selection[index];
            if (candidate.Family != (zeroRadius ? ConeSurfaceFamily.NormalCone : ConeSurfaceFamily.RotationCircle))
                continue;
            Assert.Equal(reverse ? ConeSurfacePointLocation.End : ConeSurfacePointLocation.Start, candidate.PointLocation);
            Assert.False(candidate.IsSegmentInterior);
            found++;
        }
        Assert.True(found > 0);
    }

    [Fact]
    public void SurfaceCandidates_PointLocation_DegeneratePointIsStartAndIntervalsRemainUnspecified()
    {
        Span<SegmentConeSurfaceCandidate> storage = stackalloc SegmentConeSurfaceCandidate[SegmentConeSurfaceCandidates.MaximumCandidates];
        var selection = new ConeSurfaceSelection(storage);
        Assert.True(Accumulate(new FixedSegment(Vector3d.Zero, Vector3d.Zero), Vector3d.Zero, ref selection));
        bool interval = false, fixedPoint = false;
        for (int index = 0; index < selection.Count; index++)
        {
            SegmentConeSurfaceCandidate candidate = selection[index];
            if (candidate.Family == ConeSurfaceFamily.SegmentInterval)
            {
                interval = true;
                Assert.Equal(ConeSurfacePointLocation.SegmentIntervalUnspecified, candidate.PointLocation);
            }
            else
            {
                fixedPoint = true;
                Assert.Equal(ConeSurfacePointLocation.Start, candidate.PointLocation);
            }
            Assert.False(candidate.IsSegmentInterior);
        }
        Assert.True(interval);
        Assert.True(fixedPoint);
    }

    [Theory]
    [InlineData(false, false)]
    [InlineData(true, false)]
    [InlineData(false, true)]
    [InlineData(true, true)]
    public void SurfaceCandidates_PointLocation_RetainsRimEndpointAcrossReversal(bool reverse, bool meridional)
    {
        var p = new Vector3d(1, -1, 0);
        var other = new Vector3d(Fixed64.FromFraction(5, 4), Fixed64.FromFraction(-3, 4), meridional ? Fixed64.Zero : Fixed64.FromFraction(1, 4));
        Span<SegmentConeSurfaceCandidate> storage = stackalloc SegmentConeSurfaceCandidate[SegmentConeSurfaceCandidates.MaximumCandidates];
        var selection = new ConeSurfaceSelection(storage);
        Assert.True(Accumulate(reverse ? new FixedSegment(other, p) : new FixedSegment(p, other), Vector3d.Zero, ref selection));
        bool found = false;
        for (int index = 0; index < selection.Count; index++)
        {
            SegmentConeSurfaceCandidate candidate = selection[index];
            if (candidate.Feature != ConeSurfaceFeature.Rim || candidate.Family != ConeSurfaceFamily.None)
                continue;
            FixedContactAnchors contact = candidate.GetContact();
            Assert.True(contact.FirstAnchor.TryGetPoint(out Vector3d actualP));
            Assert.True(contact.SecondAnchor.TryGetPoint(out Vector3d q));
            if (actualP != p || q != new Vector3d(2, -2, 0))
                continue;
            Assert.Equal(reverse ? ConeSurfacePointLocation.End : ConeSurfacePointLocation.Start, candidate.PointLocation);
            Assert.False(candidate.IsSegmentInterior);
            found = true;
        }
        Assert.True(found);
    }

    [Theory]
    [InlineData(false)]
    [InlineData(true)]
    public void SurfaceCandidates_PointLocation_NondyadicAxisCrossingIsInterior(bool reverse)
    {
        var a = new Vector3d(-1, -1, 0);
        var b = new Vector3d(2, 2, 0);
        Span<SegmentConeSurfaceCandidate> storage = stackalloc SegmentConeSurfaceCandidate[SegmentConeSurfaceCandidates.MaximumCandidates];
        var selection = new ConeSurfaceSelection(storage);
        Assert.True(SegmentConeSurfaceCandidates.AccumulateSegmentConeSurfaceCandidates(reverse ? new FixedSegment(b, a) : new FixedSegment(a, b),
            Vector3d.Zero, FixedQuaternion.Identity, Vector3d.Zero, FixedQuaternion.Identity, (Fixed64)4, Fixed64.Zero, ref selection));
        bool found = false;
        for (int index = 0; index < selection.Count; index++)
        {
            SegmentConeSurfaceCandidate candidate = selection[index];
            if (candidate.Feature != ConeSurfaceFeature.Side || candidate.Family != ConeSurfaceFamily.NormalCone)
                continue;
            found = true;
            Assert.Equal(ConeSurfacePointLocation.Interior, candidate.PointLocation);
            Assert.True(candidate.IsSegmentInterior);
        }
        Assert.True(found);
    }

    [Theory]
    [InlineData(false)]
    [InlineData(true)]
    public void SurfaceCandidates_InteriorRimTouch_AdmitsTangentGeneratorAndBaseNormals(bool horizontal)
    {
        var q = new Vector3d(2, -2, 0);
        var offset = horizontal ? new Vector3d(Fixed64.Zero, Fixed64.Zero, Fixed64.FromFraction(1, 4))
            : new Vector3d(Fixed64.FromFraction(-1, 4), Fixed64.Half, Fixed64.Zero);
        var segment = new FixedSegment(q - offset, q + offset);
        Span<SegmentConeSurfaceCandidate> storage = stackalloc SegmentConeSurfaceCandidate[SegmentConeSurfaceCandidates.MaximumCandidates];
        var selection = new ConeSurfaceSelection(storage);
        Assert.True(Accumulate(segment, Vector3d.Zero, ref selection));
        bool found = false;
        for (int index = 0; index < selection.Count; index++)
        {
            SegmentConeSurfaceCandidate candidate = selection[index];
            if (candidate.Feature != ConeSurfaceFeature.Rim || candidate.Family != ConeSurfaceFamily.NormalCone || !candidate.IsSegmentInterior)
                continue;
            found = true;
            FixedContactAnchors contact = FamilyContacts(candidate, NormalRayHalfspaces(Covector(horizontal ? Vector3d.Up : new Vector3d(-2, -1, 0))))[0];
            Assert.True(contact.FirstAnchor.TryGetPoint(out Vector3d p));
            Assert.Equal(q, p);
            Assert.Equal(Fixed64.Zero, contact.Depth);
        }
        Assert.True(found);
    }

    [Theory]
    [InlineData(false)]
    [InlineData(true)]
    public void SurfaceCandidates_SideStationaryEndpoint_PreservesFiniteEndpointProvenance(bool reverse)
    {
        var p = new Vector3d(Fixed64.Half, Fixed64.FromFraction(-1, 4), Fixed64.Zero);
        var other = p - Vector3d.Forward;
        var segment = reverse ? new FixedSegment(p, other) : new FixedSegment(other, p);
        Span<SegmentConeSurfaceCandidate> storage = stackalloc SegmentConeSurfaceCandidate[SegmentConeSurfaceCandidates.MaximumCandidates];
        var selection = new ConeSurfaceSelection(storage);
        Assert.True(Accumulate(segment, Vector3d.Zero, ref selection));
        bool found = false;
        for (int index = 0; index < selection.Count; index++)
        {
            SegmentConeSurfaceCandidate candidate = selection[index];
            if (candidate.Feature != ConeSurfaceFeature.Side || candidate.Family != ConeSurfaceFamily.None)
                continue;
            FixedContactAnchors contact = candidate.GetContact();
            if (contact.Depth != Fixed64.FromRaw(2400959709L))
                continue;
            Assert.True(contact.FirstAnchor.TryGetPoint(out Vector3d actualP));
            Assert.Equal(p, actualP);
            Assert.False(candidate.IsSegmentInterior);
            found = true;
        }
        Assert.True(found);
    }

    [Theory]
    [InlineData(false)]
    [InlineData(true)]
    public void SurfaceCandidates_RimChartEndpoint_PreservesFiniteEndpointProvenance(bool reverse)
    {
        var p = new Vector3d(1, -1, 0);
        var other = new Vector3d(Fixed64.FromFraction(5, 4), Fixed64.FromFraction(-3, 4), Fixed64.FromFraction(1, 4));
        var segment = reverse ? new FixedSegment(other, p) : new FixedSegment(p, other);
        Span<SegmentConeSurfaceCandidate> storage = stackalloc SegmentConeSurfaceCandidate[SegmentConeSurfaceCandidates.MaximumCandidates];
        var selection = new ConeSurfaceSelection(storage);
        Assert.True(Accumulate(segment, Vector3d.Zero, ref selection));
        bool found = false;
        for (int index = 0; index < selection.Count; index++)
        {
            SegmentConeSurfaceCandidate candidate = selection[index];
            if (candidate.Feature != ConeSurfaceFeature.Rim || candidate.Family != ConeSurfaceFamily.None)
                continue;
            Assert.Empty(FamilyContacts(candidate, NormalRayHalfspaces(Covector(Vector3d.Up))));

            if (candidate.Chart < 0)
                continue;
            FixedContactAnchors contact = candidate.GetContact();
            Assert.True(contact.FirstAnchor.TryGetPoint(out Vector3d actualP));
            Assert.True(contact.SecondAnchor.TryGetPoint(out Vector3d q));
            if (actualP != p || q != new Vector3d(2, -2, 0))
                continue;
            Assert.False(candidate.IsSegmentInterior);
            Assert.Equal(Fixed64.FromRaw(6074001000L), contact.Depth);
            found = true;
        }
        Assert.True(found);
    }

    [Theory]
    [InlineData(false, -1)]
    [InlineData(true, -1)]
    [InlineData(false, 1)]
    [InlineData(true, 1)]
    public void SurfaceCandidates_RimChartAxis_RetainsTheStationaryBasisDirection(bool reverse, int zDirection)
    {
        var p = new Vector3d(1, -1, 0);
        var offset = new Vector3d(Fixed64.FromFraction(1, 4), Fixed64.FromFraction(1, 4), Fixed64.FromFraction(zDirection, 4));
        var segment = reverse ? new FixedSegment(p + offset, p - offset) : new FixedSegment(p - offset, p + offset);
        Span<SegmentConeSurfaceCandidate> storage = stackalloc SegmentConeSurfaceCandidate[SegmentConeSurfaceCandidates.MaximumCandidates];
        var selection = new ConeSurfaceSelection(storage);
        Assert.True(Accumulate(segment, Vector3d.Zero, ref selection));
        bool found = false;
        for (int index = 0; index < selection.Count; index++)
        {
            SegmentConeSurfaceCandidate candidate = selection[index];
            if (candidate.Feature != ConeSurfaceFeature.Rim || candidate.Family != ConeSurfaceFamily.None || !candidate.IsSegmentInterior)
                continue;
            FixedContactAnchors contact = candidate.GetContact();
            Assert.True(contact.FirstAnchor.TryGetPoint(out Vector3d actualP));
            Assert.True(contact.SecondAnchor.TryGetPoint(out Vector3d q));
            if (actualP != p || q != new Vector3d(2, -2, 0))
                continue;
            Assert.Equal(Fixed64.FromRaw(6074001000L), contact.Depth);
            Assert.True(candidate.TryGetNormalDotSign(Covector(1, 1, 0), out int sign));
            Assert.Equal(0, sign);
            found = true;
        }
        Assert.True(found);
    }

    [Fact]
    public void SurfaceCandidates_SideStationaryFoot_RejectsPointAboveApex()
    {
        var segment = new FixedSegment(new Vector3d(Fixed64.Zero, Fixed64.One, Fixed64.FromFraction(-1, 4)),
            new Vector3d(Fixed64.Two, (Fixed64)5, Fixed64.FromFraction(1, 4)));
        Span<SegmentConeSurfaceCandidate> storage = stackalloc SegmentConeSurfaceCandidate[SegmentConeSurfaceCandidates.MaximumCandidates];
        var selection = new ConeSurfaceSelection(storage);
        Assert.True(Accumulate(segment, Vector3d.Zero, ref selection));
        bool found = false;
        for (int index = 0; index < selection.Count; index++)
        {
            SegmentConeSurfaceCandidate candidate = selection[index];
            if (candidate.Family != ConeSurfaceFamily.None)
                continue;
            Assert.True(candidate.GetContact().FirstAnchor.TryGetPoint(out Vector3d p));
            Assert.InRange(p.Y.m_rawValue, -Fixed64.Two.m_rawValue, Fixed64.Two.m_rawValue);
            found = true;
        }
        Assert.True(found);
    }

    [Fact]
    public void SurfaceCandidates_AxialBaseCrossing_RetainsTheWholeRimCircle()
    {
        Span<SegmentConeSurfaceCandidate> storage = stackalloc SegmentConeSurfaceCandidate[SegmentConeSurfaceCandidates.MaximumCandidates];
        var selection = new ConeSurfaceSelection(storage);
        Assert.True(Accumulate(new FixedSegment(new Vector3d(0, -3, 0), new Vector3d(0, -1, 0)), Vector3d.Zero, ref selection));
        bool found = false;
        for (int index = 0; index < selection.Count; index++)
        {
            SegmentConeSurfaceCandidate candidate = selection[index];

            if (candidate.Feature != ConeSurfaceFeature.Rim || candidate.Family != ConeSurfaceFamily.RotationCircle || !candidate.IsSegmentInterior)
                continue;
            found = true;
            FixedContactAnchors contact = FamilyContacts(candidate, NormalRayHalfspaces(Covector(0, 0, -1)))[0];
            Assert.True(contact.FirstAnchor.TryGetPoint(out Vector3d p));
            Assert.True(contact.SecondAnchor.TryGetPoint(out Vector3d q));
            Assert.Equal(new Vector3d(0, -2, 0), p);
            Assert.Equal(new Vector3d(0, -2, 2), q);
            Assert.Equal(Fixed64.Two, contact.Depth);

        }
        Assert.True(found);
    }

    [Fact]
    public void SurfaceCandidates_ParallelGeneratorInterval_ClipsTheEntryThroughTheOppositeSide()
    {
        Span<SegmentConeSurfaceCandidate> storage = stackalloc SegmentConeSurfaceCandidate[SegmentConeSurfaceCandidates.MaximumCandidates];
        var selection = new ConeSurfaceSelection(storage);
        Assert.True(Accumulate(new FixedSegment(new Vector3d(-2, 1, 0), new Vector3d(0, -3, 0)), Vector3d.Zero, ref selection));
        bool found = false;
        for (int index = 0; index < selection.Count; index++)
        {
            SegmentConeSurfaceCandidate candidate = selection[index];
            if (candidate.Feature != ConeSurfaceFeature.Side || candidate.Family != ConeSurfaceFamily.SegmentInterval)
                continue;
            found = true;
            FixedContactAnchors lower = FamilyContacts(candidate)[0];
            Assert.True(lower.FirstAnchor.TryGetPoint(out Vector3d p));
            Assert.Equal(new Vector3d(Fixed64.FromFraction(-5, 4), -Fixed64.Half, Fixed64.Zero), p);

            Assert.Empty(FamilyContacts(candidate, NormalRayHalfspaces(Covector(Vector3d.Forward))));

        }
        Assert.True(found);
    }

    [Theory]
    [InlineData(false)]
    [InlineData(true)]
    public void SurfaceCandidates_RimStationaryFoot_RejectsPointOutsideAxialBounds(bool aboveApex)
    {
        Vector3d q = aboveApex ? new Vector3d(3, -1, 4) : new Vector3d(3, -5, 4);
        Vector3d p = aboveApex ? new Vector3d(Fixed64.Zero, Fixed64.FromFraction(5, 4), Fixed64.FromFraction(1, 4))
            : new Vector3d(Fixed64.FromFraction(21, 8), Fixed64.FromFraction(-21, 4), Fixed64.FromFraction(7, 2));
        Vector3d offset = aboveApex ? new Vector3d(Fixed64.FromFraction(3, 2), Fixed64.Two, Fixed64.Zero)
            : new Vector3d(Fixed64.Half, Fixed64.FromFraction(-3, 4), Fixed64.Zero);
        var segment = new FixedSegment(p - offset, p + offset);
        Span<SegmentConeSurfaceCandidate> storage = stackalloc SegmentConeSurfaceCandidate[SegmentConeSurfaceCandidates.MaximumCandidates];
        var selection = new ConeSurfaceSelection(storage);
        Assert.True(SegmentConeSurfaceCandidates.AccumulateSegmentConeSurfaceCandidates(segment,
            Vector3d.Zero, FixedQuaternion.Identity, Vector3d.Zero, FixedQuaternion.Identity,
            (Fixed64)(aboveApex ? 2 : 10), (Fixed64)5, ref selection));
        bool witness = false;
        for (int index = 0; index < selection.Count; index++)
        {
            SegmentConeSurfaceCandidate candidate = selection[index];
            if (candidate.Family != ConeSurfaceFamily.None)
                continue;
            FixedContactAnchors contact = candidate.GetContact();
            Assert.True(contact.FirstAnchor.TryGetPoint(out Vector3d actualP));
            Assert.True(contact.SecondAnchor.TryGetPoint(out Vector3d actualQ));
            Assert.InRange(actualP.Y.m_rawValue, ((Fixed64)(aboveApex ? -1 : -5)).m_rawValue, ((Fixed64)(aboveApex ? 1 : 5)).m_rawValue);
            Assert.False(actualP == p && actualQ == q);
            witness = true;
        }
        Assert.True(witness);
    }

    [Fact]
    public void SurfaceCandidates_GenericRimTouch_RetainsZeroDepthStationaryRoot()
    {
        var q = new Vector3d(3, -5, 4);
        var offset = new Vector3d(Fixed64.FromFraction(1, 4), Fixed64.FromFraction(1, 4), Fixed64.FromFraction(1, 4));
        Span<SegmentConeSurfaceCandidate> storage = stackalloc SegmentConeSurfaceCandidate[SegmentConeSurfaceCandidates.MaximumCandidates];
        var selection = new ConeSurfaceSelection(storage);
        Assert.True(SegmentConeSurfaceCandidates.AccumulateSegmentConeSurfaceCandidates(new FixedSegment(q - offset, q + offset),
            Vector3d.Zero, FixedQuaternion.Identity, Vector3d.Zero, FixedQuaternion.Identity, (Fixed64)10, (Fixed64)5, ref selection));
        bool found = false;
        for (int index = 0; index < selection.Count; index++)
        {
            SegmentConeSurfaceCandidate candidate = selection[index];
            if (candidate.Feature != ConeSurfaceFeature.Rim || candidate.Family != ConeSurfaceFamily.None || candidate.Chart < 0)
                continue;
            FixedContactAnchors contact = candidate.GetContact();
            Assert.Equal(Fixed64.Zero, contact.Depth);
            Assert.True(contact.FirstAnchor.TryGetPoint(out Vector3d actualP));
            Assert.True(contact.SecondAnchor.TryGetPoint(out Vector3d actualQ));
            Assert.Equal(q, actualP);
            Assert.Equal(q, actualQ);
            found = true;
        }
        Assert.True(found);
    }

    [Theory]
    [InlineData(false)]
    [InlineData(true)]
    public void SurfaceCandidates_LargeSideAndMeridionalDepth_PreserveClamping(bool meridional)
    {
        Fixed64 size = Fixed64.FromRaw(long.MaxValue - 1);
        Fixed64 half = Fixed64.FromRaw(size.m_rawValue / 2);
        Vector3d p = new(-size, -half, Fixed64.Zero);
        var offset = new Vector3d(Fixed64.FromRaw(size.m_rawValue / 16), Fixed64.FromRaw(2 * (size.m_rawValue / 16)), Fixed64.Zero);
        var segment = meridional ? new FixedSegment(-offset, offset) : new FixedSegment(p, p);
        Span<SegmentConeSurfaceCandidate> storage = stackalloc SegmentConeSurfaceCandidate[SegmentConeSurfaceCandidates.MaximumCandidates];
        var selection = new ConeSurfaceSelection(storage);
        Assert.True(SegmentConeSurfaceCandidates.AccumulateSegmentConeSurfaceCandidates(segment,
            Vector3d.Zero, FixedQuaternion.Identity, Vector3d.Zero, FixedQuaternion.Identity, size, size, ref selection));
        bool found = false;
        for (int index = 0; index < selection.Count; index++)
        {
            SegmentConeSurfaceCandidate candidate = selection[index];
            if (candidate.Feature != (meridional ? ConeSurfaceFeature.Rim : ConeSurfaceFeature.Side) || candidate.Family != ConeSurfaceFamily.None)
                continue;
            FixedContactAnchors contact = candidate.GetContact();
            if (!contact.DepthIsClamped)
                continue;
            Assert.Equal(Fixed64.MaxValue, contact.Depth);
            found = true;
        }
        Assert.True(found);
    }

    [Theory]
    [InlineData(false)]
    [InlineData(true)]
    public void SurfaceCandidates_MeridionalEndpointTouch_UsesEndpointNormalHalfspace(bool reverse)
    {
        var q = new Vector3d(2, -2, 0);
        var outside = new Vector3d(3, -1, 0);
        var segment = reverse ? new FixedSegment(outside, q) : new FixedSegment(q, outside);
        Span<SegmentConeSurfaceCandidate> storage = stackalloc SegmentConeSurfaceCandidate[SegmentConeSurfaceCandidates.MaximumCandidates];
        var selection = new ConeSurfaceSelection(storage);
        Assert.True(Accumulate(segment, Vector3d.Zero, ref selection));
        bool found = false;
        for (int index = 0; index < selection.Count; index++)
        {
            SegmentConeSurfaceCandidate candidate = selection[index];
            if (candidate.Feature != ConeSurfaceFeature.Rim || candidate.Family != ConeSurfaceFamily.NormalCone || candidate.RootOrdinal > -3)
                continue;
            found = true;
            Assert.False(candidate.IsSegmentInterior);
            FixedContactAnchors contact = FamilyContacts(candidate, NormalRayHalfspaces(Covector(new Vector3d(-1, 1, 0))))[0];
            Assert.Equal(Fixed64.Zero, contact.Depth);
            Assert.Empty(FamilyContacts(candidate, NormalRayHalfspaces(Covector(Vector3d.Up))));
        }
        Assert.True(found);
    }

    [Theory]
    [InlineData(-1L)]
    [InlineData(1L)]
    public void SurfaceCandidates_SlantedHorizontalRim_RawHeightMissHasNoHorizontalNormal(long rawHeight)
    {
        var p = new Vector3d(Fixed64.One, -Fixed64.Two + Fixed64.FromRaw(rawHeight), Fixed64.Zero);
        var offset = new Vector3d(Fixed64.Zero, Fixed64.FromFraction(1, 4), Fixed64.FromFraction(1, 4));
        var segment = new FixedSegment(p - offset, p + offset);
        Span<SegmentConeSurfaceCandidate> storage = stackalloc SegmentConeSurfaceCandidate[SegmentConeSurfaceCandidates.MaximumCandidates];
        var selection = new ConeSurfaceSelection(storage);
        Assert.True(Accumulate(segment, Vector3d.Zero, ref selection));
        for (int index = 0; index < selection.Count; index++)
        {
            SegmentConeSurfaceCandidate candidate = selection[index];
            if (candidate.Feature != ConeSurfaceFeature.Rim || !candidate.IsSegmentInterior)
                continue;
            Assert.True(candidate.TryGetNormalDotSign(Covector(0, 1, 0), out int sign));
            Assert.NotEqual(0, sign);
        }
    }

    [Theory]
    [InlineData(false, false)]
    [InlineData(true, false)]
    [InlineData(false, true)]
    [InlineData(true, true)]
    public void SurfaceCandidates_SlantedHorizontalRim_RetainsBothPrincipalFeet(bool reverse, bool touch)
    {
        var p = new Vector3d(touch ? 2 : 1, -2, 0);
        var offset = new Vector3d(Fixed64.Zero, Fixed64.FromFraction(1, 4), Fixed64.FromFraction(1, 4));
        var segment = reverse ? new FixedSegment(p + offset, p - offset) : new FixedSegment(p - offset, p + offset);
        Span<SegmentConeSurfaceCandidate> storage = stackalloc SegmentConeSurfaceCandidate[SegmentConeSurfaceCandidates.MaximumCandidates];
        var selection = new ConeSurfaceSelection(storage);
        Assert.True(Accumulate(segment, Vector3d.Zero, ref selection));
        bool near = false, far = false;
        for (int index = 0; index < selection.Count; index++)
        {
            SegmentConeSurfaceCandidate candidate = selection[index];
            if (candidate.Feature != ConeSurfaceFeature.Rim || !candidate.IsSegmentInterior)
                continue;
            FixedContactAnchors contact;
            if (candidate.Family == ConeSurfaceFamily.NormalCone)
            {
                FixedContactAnchors[] familyContacts = FamilyContacts(candidate, NormalRayHalfspaces(Covector(-Vector3d.Right)));
                if (familyContacts.Length == 0) continue;
                contact = familyContacts[0];
            }
            else if (candidate.Family == ConeSurfaceFamily.None)
                contact = candidate.GetContact();
            else
                continue;
            Assert.True(contact.FirstAnchor.TryGetPoint(out Vector3d actualP));
            Assert.True(contact.SecondAnchor.TryGetPoint(out Vector3d q));
            if (actualP != p)
                continue;
            if (q == new Vector3d(2, -2, 0))
            {
                near = true;
                Assert.Equal(touch ? Fixed64.Zero : Fixed64.One, contact.Depth);
                Assert.Equal(-Vector3d.Right, contact.Normal);
            }
            if (q == new Vector3d(-2, -2, 0))
            {
                far = true;
                Assert.Equal((Fixed64)(touch ? 4 : 3), contact.Depth);
                Assert.Equal(Vector3d.Right, contact.Normal);
            }
        }
        Assert.True(near);
        Assert.True(far);
    }

    [Fact]
    public void SurfaceCandidates_WideNormal_UniqueAxisCrossingRetainsItsExactParameter()
    {
        var segment = new FixedSegment(new Vector3d(-1, -1, 0), new Vector3d(2, 2, 0));
        Signed320 unit = new(1UL << 54, 0, 0, 0, 0);
        var normal = new WideAxis3(default, default, unit);
        Span<SegmentConeSurfaceCandidate> storage = stackalloc SegmentConeSurfaceCandidate[SegmentConeSurfaceCandidates.MaximumCandidates];
        var selection = new ConeSurfaceSelection(storage);
        Assert.True(SegmentConeSurfaceCandidates.AccumulateSegmentConeSurfaceCandidates(segment,
            Vector3d.Zero, FixedQuaternion.Identity, Vector3d.Zero, FixedQuaternion.Identity, (Fixed64)4, Fixed64.Zero, ref selection));
        bool found = false;
        for (int index = 0; index < selection.Count; index++)
        {
            SegmentConeSurfaceCandidate candidate = selection[index];
            if (candidate.Feature != ConeSurfaceFeature.Side || candidate.Family != ConeSurfaceFamily.NormalCone)
                continue;
            found = true;
            FixedContactAnchors contact = FamilyContacts(candidate, NormalRayHalfspaces(normal))[0];
            Assert.True(contact.FirstAnchor.TryGetPoint(out Vector3d p));
            Assert.Equal(Vector3d.Zero, p);
            Assert.Equal(Vector3d.Forward, contact.Normal);

            Assert.Empty(FamilyContacts(candidate, NormalRayHalfspaces(Covector(1, 0, 0))));

        }
        Assert.True(found);
    }

    [Theory]
    [InlineData(0)]
    [InlineData(1)]
    public void SurfaceCandidates_SideInterior_RequiresRealNormalAndGeneratorTangency(int example)
    {
        // First edge is steeper than every generator; the second is parallel
        // to a generator but its offset lies outside that generator's plane.
        var segment = example == 0
            ? new FixedSegment(new Vector3d(Fixed64.FromFraction(1, 4), -Fixed64.One, Fixed64.Zero), new Vector3d(Fixed64.Half, Fixed64.One, Fixed64.Zero))
            : new FixedSegment(new Vector3d(Fixed64.Zero, Fixed64.Zero, Fixed64.FromFraction(1, 4)), new Vector3d(Fixed64.Half, -Fixed64.One, Fixed64.FromFraction(1, 4)));
        Span<SegmentConeSurfaceCandidate> storage = stackalloc SegmentConeSurfaceCandidate[SegmentConeSurfaceCandidates.MaximumCandidates];
        var selection = new ConeSurfaceSelection(storage);
        Assert.True(Accumulate(segment, Vector3d.Zero, ref selection));
        Assert.True(selection.Count > 0);
        for (int index = 0; index < selection.Count; index++)
        {
            SegmentConeSurfaceCandidate candidate = selection[index];
            Assert.False(candidate.Feature == ConeSurfaceFeature.Side
                && (candidate.IsSegmentInterior || candidate.Family == ConeSurfaceFamily.SegmentInterval));

        }
    }

    [Theory]
    [InlineData(0)]
    [InlineData(1)]
    [InlineData(2)]
    public void SurfaceCandidates_WideAuthoredNormal_PreservesRelativeFrameAndExactDomain(int example)
    {
        Vector3d p = example == 0 ? new Vector3d(0, 2, 0) : new Vector3d(2, -2, 0);
        var delta = example == 2 ? new Vector3d(Fixed64.FromFraction(1, 4), Fixed64.FromFraction(1, 4), Fixed64.Zero) : Vector3d.Zero;
        var segment = new FixedSegment(p - delta, p + delta);
        FixedQuaternion coneRotation = FixedQuaternion.FromAxisAngle(Vector3d.Up, Fixed64.PiOver6);
        Signed320 unit = new(1UL << 54, 0, 0, 0, 0); // 2^310: no Fixed64 narrowing can preserve this input.
        Signed320 twice = WideArithmetic.AddSigned320(unit, unit);
        var normal = new WideAxis3(WideArithmetic.Negate(example == 2 ? unit : twice),
            example == 2 ? unit : WideArithmetic.Negate(unit), default);
        Span<SegmentConeSurfaceCandidate> storage = stackalloc SegmentConeSurfaceCandidate[SegmentConeSurfaceCandidates.MaximumCandidates];
        var selection = new ConeSurfaceSelection(storage);
        Assert.True(SegmentConeSurfaceCandidates.AccumulateSegmentConeSurfaceCandidates(segment,
            Vector3d.Zero, FixedQuaternion.Identity, Vector3d.Zero, coneRotation, (Fixed64)4, Fixed64.Two, ref selection));
        bool found = false;
        for (int index = 0; index < selection.Count; index++)
        {
            SegmentConeSurfaceCandidate candidate = selection[index];
            if (candidate.Feature != (example == 0 ? ConeSurfaceFeature.Apex : ConeSurfaceFeature.Rim) || candidate.Family != ConeSurfaceFamily.NormalCone)
                continue;
            found = true;
            FixedContactAnchors contact = FamilyContacts(candidate, NormalRayHalfspaces(normal))[0];
            Assert.Equal(Fixed64.Zero, contact.Depth);
            Assert.True(contact.Normal.X < Fixed64.Zero);
            Assert.Equal(Fixed64.Zero, contact.Normal.Z);

        }
        Assert.True(found);
    }

    [Fact]
    public void SurfaceCandidates_NontrivialRelativeRotation_PreservesExactSideNormalProjection()
    {
        FixedSegment segment = ExampleSegment(1);
        Span<SegmentConeSurfaceCandidate> storage = stackalloc SegmentConeSurfaceCandidate[SegmentConeSurfaceCandidates.MaximumCandidates];
        var selection = new ConeSurfaceSelection(storage);
        Assert.True(SegmentConeSurfaceCandidates.AccumulateSegmentConeSurfaceCandidates(segment,
            Vector3d.Zero, FixedQuaternion.Identity, Vector3d.Zero,
            FixedQuaternion.FromAxisAngle(Vector3d.Up, Fixed64.PiOver6), (Fixed64)4, Fixed64.Two, ref selection));
        bool found = false;
        for (int index = 0; index < selection.Count; index++)
        {
            SegmentConeSurfaceCandidate candidate = selection[index];
            if (candidate.Feature != ConeSurfaceFeature.Side || candidate.Family != ConeSurfaceFamily.None
                || candidate.GetContact().Depth != Fixed64.FromRaw(2400959709L))
                continue;
            found = true;
            Assert.True(candidate.TryGetNormalDotSign(Covector(1, -2, 0), out int sign));
            Assert.Equal(0, sign);
        }
        Assert.True(found);
    }

    [Theory]
    [InlineData(false)]
    [InlineData(true)]
    public void SurfaceCandidates_WideConeParallelSlice_ClipsItsFiniteGenerator(bool reachesGenerator)
    {
        var segment = new FixedSegment(new Vector3d(-Fixed64.Half, Fixed64.FromFraction(1, 4), Fixed64.Zero),
            reachesGenerator ? new Vector3d(Fixed64.Half, Fixed64.Zero, Fixed64.Zero)
                : new Vector3d(-Fixed64.FromFraction(1, 4), Fixed64.FromFraction(3, 16), Fixed64.Zero));
        Span<SegmentConeSurfaceCandidate> storage = stackalloc SegmentConeSurfaceCandidate[SegmentConeSurfaceCandidates.MaximumCandidates];
        var selection = new ConeSurfaceSelection(storage);
        Assert.True(SegmentConeSurfaceCandidates.AccumulateSegmentConeSurfaceCandidates(segment,
            Vector3d.Zero, FixedQuaternion.Identity, Vector3d.Zero, FixedQuaternion.Identity, Fixed64.One, (Fixed64)4, ref selection));
        bool found = false;
        for (int index = 0; index < selection.Count; index++)
        {
            SegmentConeSurfaceCandidate candidate = selection[index];
            if (candidate.Feature != ConeSurfaceFeature.Side || candidate.Family != ConeSurfaceFamily.SegmentInterval)
                continue;
            found = true;

            FixedContactAnchors lower = FamilyContacts(candidate)[0];
            Assert.Equal(new Vector3d(Fixed64.Zero, Fixed64.Half, Fixed64.Zero), lower.SecondAnchor.LocalPoint);
        }
        Assert.Equal(reachesGenerator, found);
    }

    [Theory]
    [InlineData(true, false)]
    [InlineData(false, false)]
    [InlineData(true, true)]
    [InlineData(false, true)]
    public void SurfaceCandidates_ZeroDepthFamily_RequiresNonemptyEndpointNormalDomain(bool apex, bool reverse)
    {
        var segment = apex ? new FixedSegment(new Vector3d(0, 2, 0), new Vector3d(0, 1, 0))
            : new FixedSegment(new Vector3d(2, -2, 0), new Vector3d(1, -1, 0));
        if (reverse)
            segment = new FixedSegment(segment.End, segment.Start);
        Span<SegmentConeSurfaceCandidate> storage = stackalloc SegmentConeSurfaceCandidate[SegmentConeSurfaceCandidates.MaximumCandidates];
        var selection = new ConeSurfaceSelection(storage);
        Assert.True(Accumulate(segment, Vector3d.Zero, ref selection));
        for (int index = 0; index < selection.Count; index++)
            Assert.False(selection[index].Feature == (apex ? ConeSurfaceFeature.Apex : ConeSurfaceFeature.Rim)
                && selection[index].Family == ConeSurfaceFamily.NormalCone);
    }

    [Theory]
    [InlineData(false, false)]
    [InlineData(true, false)]
    [InlineData(false, true)]
    [InlineData(true, true)]
    public void SurfaceCandidates_MeridionalRim_RetainsPrincipalStationaryFoot(bool touch, bool reverse)
    {
        Vector3d point = touch ? new Vector3d(2, -2, 0) : new Vector3d(1, -1, 0);
        var offset = new Vector3d(Fixed64.FromFraction(1, 4), Fixed64.FromFraction(1, 4), Fixed64.Zero);
        var segment = reverse ? new FixedSegment(point + offset, point - offset) : new FixedSegment(point - offset, point + offset);
        Span<SegmentConeSurfaceCandidate> storage = stackalloc SegmentConeSurfaceCandidate[SegmentConeSurfaceCandidates.MaximumCandidates];
        var selection = new ConeSurfaceSelection(storage);
        Assert.True(Accumulate(segment, Vector3d.Zero, ref selection));
        bool found = false;
        for (int index = 0; index < selection.Count; index++)
        {
            SegmentConeSurfaceCandidate candidate = selection[index];
            if (candidate.Feature != ConeSurfaceFeature.Rim || !candidate.IsSegmentInterior)
                continue;
            FixedContactAnchors contact;
            if (touch)
            {
                FixedContactAnchors[] familyContacts = FamilyContacts(candidate, NormalRayHalfspaces(Covector(new Vector3d(-1, 1, 0))));
                if (familyContacts.Length == 0) continue;
                contact = familyContacts[0];
            }
            else
            {
                if (candidate.Family != ConeSurfaceFamily.None)
                    continue;
                contact = candidate.GetContact();
            }
            if (contact.Depth != (touch ? Fixed64.Zero : Fixed64.FromRaw(6074001000L)))
                continue;
            found = true;
            Assert.True(contact.FirstAnchor.TryGetPoint(out Vector3d p));
            Assert.True(contact.SecondAnchor.TryGetPoint(out Vector3d q));
            Assert.Equal(point, p);
            Assert.Equal(new Vector3d(2, -2, 0), q);
        }
        Assert.True(found);
    }

    [Fact]
    public void SurfaceCandidates_MeridionalBaseCrossing_RetainsBothTransverseRimFeet()
    {
        var segment = new FixedSegment(new Vector3d(-Fixed64.FromFraction(1, 4), -Fixed64.FromFraction(9, 4), Fixed64.Zero),
            new Vector3d(Fixed64.FromFraction(1, 4), -Fixed64.FromFraction(7, 4), Fixed64.Zero));
        Span<SegmentConeSurfaceCandidate> storage = stackalloc SegmentConeSurfaceCandidate[SegmentConeSurfaceCandidates.MaximumCandidates];
        var selection = new ConeSurfaceSelection(storage);
        Assert.True(Accumulate(segment, Vector3d.Zero, ref selection));
        bool positive = false, negative = false;
        for (int index = 0; index < selection.Count; index++)
        {
            SegmentConeSurfaceCandidate candidate = selection[index];
            if (candidate.Feature != ConeSurfaceFeature.Rim || candidate.Family != ConeSurfaceFamily.None)
                continue;
            FixedContactAnchors contact = candidate.GetContact();
            if (contact.Depth != Fixed64.Two)
                continue;
            positive |= contact.Normal.Z == Fixed64.One;
            negative |= contact.Normal.Z == -Fixed64.One;
            Assert.True(candidate.TryGetNormalDotSign(Covector(1, 0, 0), out int sign));
            Assert.Equal(0, sign);
        }
        Assert.True(positive && negative);
    }

    [Fact]
    public void SurfaceCandidates_BaseInterval_ExposesIrrationalEndpoints()
    {
        var segment = new FixedSegment(new Vector3d(-Fixed64.Two, Fixed64.Zero, Fixed64.Half),
            new Vector3d(Fixed64.Two, Fixed64.Zero, Fixed64.Half));
        Span<SegmentConeSurfaceCandidate> storage = stackalloc SegmentConeSurfaceCandidate[SegmentConeSurfaceCandidates.MaximumCandidates];
        var selection = new ConeSurfaceSelection(storage);
        Assert.True(Accumulate(segment, Vector3d.Zero, ref selection));
        bool found = false;
        for (int index = 0; index < selection.Count; index++)
        {
            SegmentConeSurfaceCandidate candidate = selection[index];
            if (candidate.Feature != ConeSurfaceFeature.Base || candidate.Family != ConeSurfaceFamily.SegmentInterval)
                continue;
            found = true;
            FixedContactAnchors lower = FamilyContacts(candidate)[0];
            FixedContactAnchors upper = FamilyContacts(candidate)[^1];
            Assert.Equal(Fixed64.FromRaw(-3719550787L), lower.FirstAnchor.LocalPoint.X);
            Assert.Equal(Fixed64.FromRaw(3719550787L), upper.FirstAnchor.LocalPoint.X);
            Assert.Equal(Fixed64.Two, lower.Depth);
        }
        Assert.True(found);
    }

    [Fact]
    public void SurfaceCandidates_BaseTangent_ExposesExactNondyadicEndpoint()
    {
        var segment = new FixedSegment(new Vector3d(-1, 0, 1), new Vector3d(2, 0, 1));
        Span<SegmentConeSurfaceCandidate> storage = stackalloc SegmentConeSurfaceCandidate[SegmentConeSurfaceCandidates.MaximumCandidates];
        var selection = new ConeSurfaceSelection(storage);
        Assert.True(Accumulate(segment, Vector3d.Zero, ref selection));
        bool found = false;
        for (int index = 0; index < selection.Count; index++)
        {
            SegmentConeSurfaceCandidate candidate = selection[index];
            if (candidate.Feature != ConeSurfaceFeature.Base || candidate.Family != ConeSurfaceFamily.SegmentInterval)
                continue;
            found = true;

            FixedContactAnchors lower = FamilyContacts(candidate)[0];
            FixedContactAnchors upper = FamilyContacts(candidate)[^1];
            Assert.Equal(Fixed64.Two, lower.Depth);
            Assert.True(lower.FirstAnchor.TryGetPoint(out Vector3d p));
            Assert.True(lower.SecondAnchor.TryGetPoint(out Vector3d q));
            Assert.Equal(new Vector3d(0, 0, 1), p);
            Assert.Equal(new Vector3d(0, -2, 1), q);
            Assert.Equal(lower.FirstAnchor, upper.FirstAnchor);
        }
        Assert.True(found);
    }

    [Fact]
    public void SurfaceCandidates_ZeroRadiusCrossing_RetainsIsolatedNormalDomain()
    {
        var segment = new FixedSegment(-Vector3d.Right, Vector3d.Right);
        Span<SegmentConeSurfaceCandidate> storage = stackalloc SegmentConeSurfaceCandidate[64];
        var selection = new ConeSurfaceSelection(storage);
        Assert.True(SegmentConeSurfaceCandidates.AccumulateSegmentConeSurfaceCandidates(segment,
            Vector3d.Zero, FixedQuaternion.Identity, Vector3d.Zero, FixedQuaternion.Identity,
            (Fixed64)4, Fixed64.Zero, ref selection));
        bool found = false;
        for (int index = 0; index < selection.Count; index++)
        {
            SegmentConeSurfaceCandidate candidate = selection[index];
            if (candidate.Feature != ConeSurfaceFeature.Side || candidate.Family != ConeSurfaceFamily.NormalCone)
                continue;
            found = true;
            Assert.True(candidate.IsSegmentInterior);
            FixedContactAnchors contact = FamilyContacts(candidate, NormalRayHalfspaces(Covector(new Vector3d(0, 0, 1))))[0];
            Assert.Equal(Fixed64.Zero, contact.Depth);
            Assert.Empty(FamilyContacts(candidate, NormalRayHalfspaces(Covector(Vector3d.Right))));
            Assert.Empty(FamilyContacts(candidate, NormalRayHalfspaces(Covector(Vector3d.Up))));

        }
        Assert.True(found);
    }

    [Fact]
    public void SurfaceCandidates_ObliqueRim_RetainsFiniteInteriorRoot()
    {
        var segment = new FixedSegment(
            new Vector3d(Fixed64.FromFraction(188, 64), Fixed64.FromFraction(-1281, 256), Fixed64.FromFraction(249, 64)),
            new Vector3d(Fixed64.FromFraction(190, 64), Fixed64.FromFraction(-1249, 256), Fixed64.FromFraction(255, 64)));
        Span<SegmentConeSurfaceCandidate> storage = stackalloc SegmentConeSurfaceCandidate[64];
        var selection = new ConeSurfaceSelection(storage);

        Assert.True(SegmentConeSurfaceCandidates.AccumulateSegmentConeSurfaceCandidates(segment,
            Vector3d.Zero, FixedQuaternion.Identity, Vector3d.Zero, FixedQuaternion.Identity,
            (Fixed64)10, (Fixed64)5, ref selection));
        bool found = false;
        for (int index = 0; index < selection.Count; index++)
            if (selection[index].Feature == ConeSurfaceFeature.Rim && selection[index].IsSegmentInterior
                && selection[index].GetContact().Depth == Fixed64.FromFraction(25, 256))
                found = true;
        Assert.True(found);
    }

    [Fact]
    public void SurfaceCandidates_VerticalSegment_RetainsBothAdmittedRimBranches()
    {
        var segment = new FixedSegment(new Vector3d(0, -3, 0), new Vector3d(0, 3, 0));
        Span<SegmentConeSurfaceCandidate> storage = stackalloc SegmentConeSurfaceCandidate[64];
        var selection = new ConeSurfaceSelection(storage);

        Assert.True(Accumulate(segment, new Vector3d(Fixed64.FromFraction(1, 4), Fixed64.Zero, Fixed64.Zero), ref selection));
        bool positive = false, negative = false;
        for (int index = 0; index < selection.Count; index++)
        {
            SegmentConeSurfaceCandidate candidate = selection[index];
            if (candidate.Feature != ConeSurfaceFeature.Rim || candidate.Family != ConeSurfaceFamily.None)
                continue;
            Assert.True(candidate.IsSegmentInterior);
            FixedContactAnchors contact = candidate.GetContact();
            if (contact.Normal == Vector3d.Right)
            {
                positive = true;
                Assert.Equal(Fixed64.FromFraction(7, 4), contact.Depth);
            }
            if (contact.Normal == -Vector3d.Right)
            {
                negative = true;
                Assert.Equal(Fixed64.FromFraction(9, 4), contact.Depth);
            }
        }
        Assert.True(positive);
        Assert.True(negative);
    }

    [Fact]
    public void SurfaceCandidates_RimFootOutsideFiniteSegment_IsNotAnInteriorCandidate()
    {
        var segment = new FixedSegment(new Vector3d(0, -1, 0), new Vector3d(0, 1, 0));
        Span<SegmentConeSurfaceCandidate> storage = stackalloc SegmentConeSurfaceCandidate[64];
        var selection = new ConeSurfaceSelection(storage);

        Assert.True(Accumulate(segment, new Vector3d(Fixed64.FromFraction(1, 4), Fixed64.Zero, Fixed64.Zero), ref selection));
        for (int index = 0; index < selection.Count; index++)
            Assert.False(selection[index].Feature == ConeSurfaceFeature.Rim && selection[index].IsSegmentInterior);
    }

    [Fact]
    public void SurfaceCandidates_HorizontalInteriorSegment_RetainsBaseInterval()
    {
        var segment = new FixedSegment(new Vector3d(-Fixed64.Half, Fixed64.Zero, Fixed64.Zero),
            new Vector3d(Fixed64.Half, Fixed64.Zero, Fixed64.Zero));
        Span<SegmentConeSurfaceCandidate> storage = stackalloc SegmentConeSurfaceCandidate[64];
        var selection = new ConeSurfaceSelection(storage);

        Assert.True(Accumulate(segment, Vector3d.Zero, ref selection));
        Assert.True(HasFamily(selection, ConeSurfaceFeature.Base, ConeSurfaceFamily.SegmentInterval));
    }

    [Fact]
    public void SurfaceCandidates_BaseInterval_AdmitsExactRepresentativeBeforeRounding()
    {
        var segment = new FixedSegment(new Vector3d(-2, 0, 0), new Vector3d(2, 0, 0));
        Span<SegmentConeSurfaceCandidate> storage = stackalloc SegmentConeSurfaceCandidate[64];
        var selection = new ConeSurfaceSelection(storage);
        Assert.True(Accumulate(segment, Vector3d.Zero, ref selection));
        bool found = false;
        for (int index = 0; index < selection.Count; index++)
        {
            SegmentConeSurfaceCandidate candidate = selection[index];
            if (candidate.Feature != ConeSurfaceFeature.Base || candidate.Family != ConeSurfaceFamily.SegmentInterval)
                continue;
            found = true;
            FixedContactAnchors middle = FamilyContacts(candidate)[1];
            Assert.Equal(Vector3d.Up, middle.Normal);
            Assert.Equal(Fixed64.Two, middle.Depth);

            FixedContactAnchors lower = FamilyContacts(candidate)[0];
            FixedContactAnchors upper = FamilyContacts(candidate)[^1];
            Assert.Equal(-Vector3d.Right, lower.FirstAnchor.LocalPoint);
            Assert.Equal(Vector3d.Right, upper.FirstAnchor.LocalPoint);
        }
        Assert.True(found);
    }

    [Fact]
    public void SurfaceCandidates_BaseEndpoint_UsesFiniteEndpointNormalDomain()
    {
        var segment = new FixedSegment(Vector3d.Zero, Vector3d.Up);
        Span<SegmentConeSurfaceCandidate> storage = stackalloc SegmentConeSurfaceCandidate[64];
        var selection = new ConeSurfaceSelection(storage);
        Assert.True(Accumulate(segment, Vector3d.Zero, ref selection));
        int count = 0;
        for (int index = 0; index < selection.Count; index++)
        {
            SegmentConeSurfaceCandidate candidate = selection[index];
            if (candidate.Feature != ConeSurfaceFeature.Base)
                continue;
            count++;
            Assert.Equal(ConeSurfaceFamily.None, candidate.Family);
            FixedContactAnchors contact = candidate.GetContact();
            Assert.Equal(Vector3d.Up, contact.Normal);
            Assert.Equal((Fixed64)3, contact.Depth);
            Assert.True(contact.FirstAnchor.TryGetPoint(out Vector3d first));
            Assert.Equal(Vector3d.Up, first);
        }
        Assert.Equal(1, count);
    }

    [Fact]
    public void SurfaceCandidates_ParallelGenerator_RetainsFiniteSliceFamily()
    {
        // H=4,R=2, generator (2,-4,0), inward offset -(4,2,0)/10.
        var segment = new FixedSegment(
            new Vector3d(Fixed64.FromFraction(1, 10), Fixed64.FromFraction(4, 5), Fixed64.Zero),
            new Vector3d(Fixed64.FromFraction(3, 5), -Fixed64.FromFraction(1, 5), Fixed64.Zero));
        Span<SegmentConeSurfaceCandidate> storage = stackalloc SegmentConeSurfaceCandidate[64];
        var selection = new ConeSurfaceSelection(storage);

        Assert.True(Accumulate(segment, Vector3d.Zero, ref selection));
        Assert.True(HasFamily(selection, ConeSurfaceFeature.Side, ConeSurfaceFamily.SegmentInterval));
    }

    [Fact]
    public void SurfaceCandidates_SideInterior_RetainsMatchedFiniteGeneratorFoot()
    {
        var segment = new FixedSegment(new Vector3d(Fixed64.Half, -Fixed64.FromFraction(1, 4), -Fixed64.Half),
            new Vector3d(Fixed64.Half, -Fixed64.FromFraction(1, 4), Fixed64.Half));
        Span<SegmentConeSurfaceCandidate> storage = stackalloc SegmentConeSurfaceCandidate[64];
        var selection = new ConeSurfaceSelection(storage);
        Assert.True(Accumulate(segment, Vector3d.Zero, ref selection));
        bool found = false;
        for (int index = 0; index < selection.Count; index++)
        {
            SegmentConeSurfaceCandidate candidate = selection[index];
            if (candidate.Feature != ConeSurfaceFeature.Side || candidate.Family != ConeSurfaceFamily.None || !candidate.IsSegmentInterior)
                continue;
            FixedContactAnchors contact = candidate.GetContact();
            if (contact.Depth != Fixed64.FromRaw(2400959709L))
                continue;
            found = true;
            Assert.True(contact.FirstAnchor.TryGetPoint(out Vector3d first));
            Assert.True(contact.SecondAnchor.TryGetPoint(out Vector3d second));
            Assert.Equal(new Vector3d(Fixed64.Half, -Fixed64.FromFraction(1, 4), Fixed64.Zero), first);
            Assert.Equal(Vector3d.Right, second);
        }
        Assert.True(found);
    }

    [Fact]
    public void SurfaceCandidates_ParallelFamily_MaterializesAdmittedSlice()
    {
        var segment = new FixedSegment(new Vector3d(Fixed64.Zero, Fixed64.FromFraction(3, 4), Fixed64.Zero),
            new Vector3d(Fixed64.Half, -Fixed64.FromFraction(1, 4), Fixed64.Zero));
        Span<SegmentConeSurfaceCandidate> storage = stackalloc SegmentConeSurfaceCandidate[64];
        var selection = new ConeSurfaceSelection(storage);
        Assert.True(Accumulate(segment, Vector3d.Zero, ref selection));
        bool found = false;
        for (int index = 0; index < selection.Count; index++)
        {
            SegmentConeSurfaceCandidate candidate = selection[index];
            if (candidate.Feature != ConeSurfaceFeature.Side || candidate.Family != ConeSurfaceFamily.SegmentInterval)
                continue;
            found = true;
            FixedContactAnchors contact = FamilyContacts(candidate)[1];
            Assert.Equal(Fixed64.FromRaw(2400959709L), contact.Depth);
            Assert.True(contact.SecondAnchor.TryGetPoint(out Vector3d second));
            Assert.Equal(new Vector3d(Fixed64.FromFraction(3, 4), Fixed64.Half, Fixed64.Zero), second);

            FixedContactAnchors lower = FamilyContacts(candidate)[0];
            FixedContactAnchors upper = FamilyContacts(candidate)[^1];
            Assert.Equal(new Vector3d(Fixed64.Half, Fixed64.One, Fixed64.Zero), lower.SecondAnchor.LocalPoint);
            Assert.Equal(Vector3d.Right, upper.SecondAnchor.LocalPoint);
        }
        Assert.True(found);
    }

    [Fact]
    public void SurfaceCandidates_PointOnConeAxis_RetainsRotationalFamilies()
    {
        var segment = new FixedSegment(Vector3d.Zero, Vector3d.Zero);
        Span<SegmentConeSurfaceCandidate> storage = stackalloc SegmentConeSurfaceCandidate[64];
        var selection = new ConeSurfaceSelection(storage);

        Assert.True(Accumulate(segment, Vector3d.Zero, ref selection));
        Assert.True(HasFamily(selection, ConeSurfaceFeature.Side, ConeSurfaceFamily.RotationCircle));
        Assert.True(HasFamily(selection, ConeSurfaceFeature.Rim, ConeSurfaceFamily.RotationCircle));
        for (int index = 0; index < selection.Count; index++)
        {
            SegmentConeSurfaceCandidate candidate = selection[index];
            if (candidate.Feature != ConeSurfaceFeature.Side || candidate.Family != ConeSurfaceFamily.RotationCircle)
                continue;
            FixedContactAnchors contact = FamilyContacts(candidate, NormalRayHalfspaces(Covector(candidate.Feature == ConeSurfaceFeature.Side ? -2 : -1, candidate.Feature == ConeSurfaceFeature.Side ? -1 : 1, 0)))[0];
            Assert.Equal(Fixed64.FromRaw(3841535534L), contact.Depth);
            Assert.True(contact.SecondAnchor.TryGetPoint(out Vector3d second));
            Assert.Equal(new Vector3d(Fixed64.FromFraction(4, 5), Fixed64.FromFraction(2, 5), Fixed64.Zero), second);

        }
    }

    [Fact]
    public void SurfaceCandidates_HorizontalRimFoot_RetainsItsMatchedPointAndDepth()
    {
        var segment = new FixedSegment(new Vector3d(Fixed64.Half, Fixed64.Zero, -Fixed64.One),
            new Vector3d(Fixed64.Half, Fixed64.Zero, Fixed64.One));
        Span<SegmentConeSurfaceCandidate> storage = stackalloc SegmentConeSurfaceCandidate[64];
        var selection = new ConeSurfaceSelection(storage);
        Assert.True(Accumulate(segment, Vector3d.Zero, ref selection));
        bool found = false;
        for (int index = 0; index < selection.Count; index++)
        {
            SegmentConeSurfaceCandidate candidate = selection[index];
            if (candidate.Feature != ConeSurfaceFeature.Rim || candidate.Family != ConeSurfaceFamily.None || !candidate.IsSegmentInterior)
                continue;
            FixedContactAnchors contact = candidate.GetContact();
            if (contact.Depth != Fixed64.FromFraction(5, 2))
                continue;
            found = true;
            Assert.True(contact.FirstAnchor.TryGetPoint(out Vector3d first));
            Assert.True(contact.SecondAnchor.TryGetPoint(out Vector3d second));
            Assert.Equal(new Vector3d(Fixed64.Half, Fixed64.Zero, Fixed64.Zero), first);
            Assert.Equal(new Vector3d(2, -2, 0), second);
        }
        Assert.True(found);
    }

    [Fact]
    public void SurfaceCandidates_RimCircle_UsesExactEndpointNormalDomain()
    {
        var segment = new FixedSegment(Vector3d.Zero, Vector3d.Right);
        Span<SegmentConeSurfaceCandidate> storage = stackalloc SegmentConeSurfaceCandidate[64];
        var selection = new ConeSurfaceSelection(storage);
        Assert.True(Accumulate(segment, Vector3d.Zero, ref selection));
        bool found = false;
        for (int index = 0; index < selection.Count; index++)
        {
            SegmentConeSurfaceCandidate candidate = selection[index];
            if (candidate.Feature != ConeSurfaceFeature.Rim || candidate.Family != ConeSurfaceFamily.RotationCircle)
                continue;
            found = true;
            Assert.Throws<InvalidOperationException>(() => candidate.GetContact());
            FixedContactAnchors contact = FamilyContacts(candidate, NormalRayHalfspaces(Covector(candidate.Feature == ConeSurfaceFeature.Side ? -2 : -1, candidate.Feature == ConeSurfaceFeature.Side ? -1 : 1, 0)))[0];
            Assert.Equal(Fixed64.FromRaw(12148002000L), contact.Depth);
            Assert.True(contact.SecondAnchor.TryGetPoint(out Vector3d second));
            Assert.Equal(new Vector3d(2, -2, 0), second);
            Assert.Empty(FamilyContacts(candidate, NormalRayHalfspaces(Covector(1, 1, 0))));

        }
        Assert.True(found);
    }

    [Theory]
    [InlineData(0L, 8589934592L)]
    [InlineData(4294967296L, 4294967296L)]
    public void SurfaceCandidates_ApexDepth_PreservesRawUnits(long pointY, long expectedDepth)
    {
        var point = new Vector3d(Fixed64.Zero, Fixed64.FromRaw(pointY), Fixed64.Zero);
        var segment = new FixedSegment(point, point);
        Span<SegmentConeSurfaceCandidate> storage = stackalloc SegmentConeSurfaceCandidate[64];
        var selection = new ConeSurfaceSelection(storage);
        Assert.True(Accumulate(segment, Vector3d.Zero, ref selection));
        bool found = false;
        for (int index = 0; index < selection.Count; index++)
        {
            SegmentConeSurfaceCandidate candidate = selection[index];
            if (candidate.Feature != ConeSurfaceFeature.Apex || candidate.Family != ConeSurfaceFamily.None)
                continue;
            found = true;
            FixedContactAnchors contact = candidate.GetContact();
            Assert.Equal(Fixed64.FromRaw(expectedDepth), contact.Depth);
            Assert.Equal(-Vector3d.Up, contact.Normal);
        }
        Assert.True(found);
    }

    [Theory]
    [InlineData(0L, true)]
    [InlineData(1L, false)]
    public void SurfaceCandidates_ApexTouchAndRawGap_UseExactMembership(long gap, bool admitted)
    {
        Fixed64 y = Fixed64.Two + Fixed64.FromRaw(gap);
        var segment = new FixedSegment(new Vector3d(-Fixed64.One, y, Fixed64.Zero),
            new Vector3d(Fixed64.One, y, Fixed64.Zero));
        Span<SegmentConeSurfaceCandidate> storage = stackalloc SegmentConeSurfaceCandidate[64];
        var selection = new ConeSurfaceSelection(storage);

        Assert.Equal(admitted, Accumulate(segment, Vector3d.Zero, ref selection));
        if (admitted)
        {
            Assert.True(HasFamily(selection, ConeSurfaceFeature.Apex, ConeSurfaceFamily.NormalCone));
            for (int index = 0; index < selection.Count; index++)
            {
                SegmentConeSurfaceCandidate candidate = selection[index];
                if (candidate.Feature != ConeSurfaceFeature.Apex || candidate.Family != ConeSurfaceFamily.NormalCone)
                    continue;
                FixedContactAnchors touch = FamilyContacts(candidate, NormalRayHalfspaces(Covector(-Vector3d.Up)))[0];
                Assert.Equal(Fixed64.Zero, touch.Depth);
                Assert.Equal(-Vector3d.Up, touch.Normal);
                Assert.Empty(FamilyContacts(candidate, NormalRayHalfspaces(Covector(Vector3d.Up))));

            }
        }
        else
            Assert.Equal(0, selection.Count);
    }

    private static bool Accumulate(FixedSegment segment, Vector3d center, ref ConeSurfaceSelection selection) =>
        SegmentConeSurfaceCandidates.AccumulateSegmentConeSurfaceCandidates(segment,
            Vector3d.Zero, FixedQuaternion.Identity, center, FixedQuaternion.Identity,
            (Fixed64)4, Fixed64.Two, ref selection);

    [Fact]
    public void SurfaceCandidates_ExactNormalQueries_RetainZeroBeforeRounding()
    {
        var segment = new FixedSegment(new Vector3d(Fixed64.Half, -Fixed64.FromFraction(1, 4), -Fixed64.Half),
            new Vector3d(Fixed64.Half, -Fixed64.FromFraction(1, 4), Fixed64.Half));
        Span<SegmentConeSurfaceCandidate> storage = stackalloc SegmentConeSurfaceCandidate[SegmentConeSurfaceCandidates.MaximumCandidates];
        var selection = new ConeSurfaceSelection(storage);
        Assert.True(Accumulate(segment, Vector3d.Zero, ref selection));
        bool found = false;
        for (int index = 0; index < selection.Count; index++)
        {
            SegmentConeSurfaceCandidate candidate = selection[index];
            if (candidate.Feature != ConeSurfaceFeature.Side || candidate.Family != ConeSurfaceFamily.None || !candidate.IsSegmentInterior
                || candidate.GetContact().Depth != Fixed64.FromRaw(2400959709L))
                continue;
            found = true;
            Assert.True(candidate.TryGetNormalDotSign(Covector(1, -2, 0), out int zero));
            Assert.Equal(0, zero);
            Assert.True(candidate.TryGetNormalDotSign(Covector(1, 0, 0), out int negative));
            Assert.Equal(-1, negative);
            Assert.True(candidate.TryGetNormalDotSign(Covector(-1, 0, 0), out int positive));
            Assert.Equal(1, positive);
        }
        Assert.True(found);
    }

    [Fact]
    public void SurfaceCandidates_QuarticNormalQuery_RetainsItsOwnRoot()
    {
        var segment = new FixedSegment(
            new Vector3d(Fixed64.FromFraction(188, 64), Fixed64.FromFraction(-1281, 256), Fixed64.FromFraction(249, 64)),
            new Vector3d(Fixed64.FromFraction(190, 64), Fixed64.FromFraction(-1249, 256), Fixed64.FromFraction(255, 64)));
        Span<SegmentConeSurfaceCandidate> storage = stackalloc SegmentConeSurfaceCandidate[SegmentConeSurfaceCandidates.MaximumCandidates];
        var selection = new ConeSurfaceSelection(storage);
        Assert.True(SegmentConeSurfaceCandidates.AccumulateSegmentConeSurfaceCandidates(segment,
            Vector3d.Zero, FixedQuaternion.Identity, Vector3d.Zero, FixedQuaternion.Identity, (Fixed64)10, (Fixed64)5, ref selection));
        bool found = false;
        for (int index = 0; index < selection.Count; index++)
        {
            SegmentConeSurfaceCandidate candidate = selection[index];
            if (candidate.Feature != ConeSurfaceFeature.Rim || candidate.Family != ConeSurfaceFamily.None || !candidate.IsSegmentInterior
                || candidate.GetContact().Depth != Fixed64.FromFraction(25, 256))
                continue;
            found = true;
            Assert.True(candidate.TryGetNormalDotSign(Covector(5, 4, 0), out int zero));
            Assert.Equal(0, zero);
            Assert.True(candidate.TryGetNormalDotSign(Covector(0, 1, 0), out int positive));
            Assert.Equal(1, positive);
        }
        Assert.True(found);
    }

    [Theory]
    [InlineData(0)]
    [InlineData(1)]
    [InlineData(2)]
    [InlineData(3)]
    public void SurfaceCandidates_ReversedEndpoints_RetainTheSameWitnessSet(int example)
    {
        FixedSegment segment = ExampleSegment(example);
        Span<SegmentConeSurfaceCandidate> forwardStorage = stackalloc SegmentConeSurfaceCandidate[SegmentConeSurfaceCandidates.MaximumCandidates];
        Span<SegmentConeSurfaceCandidate> reverseStorage = stackalloc SegmentConeSurfaceCandidate[SegmentConeSurfaceCandidates.MaximumCandidates];
        var forward = new ConeSurfaceSelection(forwardStorage);
        var reverse = new ConeSurfaceSelection(reverseStorage);
        Assert.True(Accumulate(segment, Vector3d.Zero, ref forward));
        Assert.True(Accumulate(new FixedSegment(segment.End, segment.Start), Vector3d.Zero, ref reverse));
        Assert.Equal(forward.Count, reverse.Count);
        for (int index = 0; index < forward.Count; index++)
        {
            SegmentConeSurfaceCandidate candidate = forward[index];
            bool found = false;
            for (int other = 0; other < reverse.Count && !found; other++)
            {
                SegmentConeSurfaceCandidate counterpart = reverse[other];
                if (candidate.Feature != counterpart.Feature || candidate.Family != counterpart.Family)
                    continue;
                if (candidate.Family != ConeSurfaceFamily.None)
                {
                    found = true;
                    continue;
                }
                FixedContactAnchors first = candidate.GetContact(), second = counterpart.GetContact();
                Assert.True(first.FirstAnchor.TryGetPoint(out Vector3d p));
                Assert.True(second.FirstAnchor.TryGetPoint(out Vector3d otherP));
                Assert.True(first.SecondAnchor.TryGetPoint(out Vector3d q));
                Assert.True(second.SecondAnchor.TryGetPoint(out Vector3d otherQ));
                found = first.Depth == second.Depth && first.Normal == second.Normal && p == otherP && q == otherQ;
            }
            Assert.True(found);
        }
    }

    [Fact]
    public void SurfaceCandidates_RigidFrameAndLargeTranslation_PreserveLocalCertificates()
    {
        FixedSegment segment = ExampleSegment(2);
        FixedQuaternion rotation = FixedQuaternion.FromAxisAngle(Vector3d.Forward, Fixed64.PiOver6);
        var origin = new Vector3d(Fixed64.MaxValue - (Fixed64)16, Fixed64.MinValue + (Fixed64)16, Fixed64.MaxValue - (Fixed64)32);
        Span<SegmentConeSurfaceCandidate> localStorage = stackalloc SegmentConeSurfaceCandidate[SegmentConeSurfaceCandidates.MaximumCandidates];
        Span<SegmentConeSurfaceCandidate> movedStorage = stackalloc SegmentConeSurfaceCandidate[SegmentConeSurfaceCandidates.MaximumCandidates];
        var local = new ConeSurfaceSelection(localStorage);
        var moved = new ConeSurfaceSelection(movedStorage);
        Assert.True(Accumulate(segment, Vector3d.Zero, ref local));
        Assert.True(SegmentConeSurfaceCandidates.AccumulateSegmentConeSurfaceCandidates(segment,
            origin, rotation, origin, rotation, (Fixed64)4, Fixed64.Two, ref moved));
        Assert.Equal(local.Count, moved.Count);
        for (int index = 0; index < local.Count; index++)
        {
            Assert.Equal(local[index].Feature, moved[index].Feature);
            Assert.Equal(local[index].Family, moved[index].Family);
            if (local[index].Family == ConeSurfaceFamily.None)
            {
                Assert.Equal(local[index].GetContact().Depth, moved[index].GetContact().Depth);
                Assert.Equal(local[index].GetContact().FirstAnchor.LocalPoint, moved[index].GetContact().FirstAnchor.LocalPoint);
                Assert.Equal(local[index].GetContact().SecondAnchor.LocalPoint, moved[index].GetContact().SecondAnchor.LocalPoint);
            }
            if (local[index].TryGetNormalDotSign(Covector(1, -2, 3), out int expected))
            {
                Assert.True(moved[index].TryGetNormalDotSign(Covector(1, -2, 3), out int actual));
                Assert.Equal(expected, actual);
            }
        }
    }

    [Fact]
    public void SurfaceCandidates_CompleteWarmedConstructionAndQueries_AllocateZeroBytes()
    {
        long checksum = ExerciseSurfaceCandidates();
        checksum ^= ExerciseSurfaceCandidates();
        long before = GC.GetAllocatedBytesForCurrentThread();
        for (int iteration = 0; iteration < 4; iteration++)
            checksum = unchecked(checksum + ExerciseSurfaceCandidates());
        long allocated = GC.GetAllocatedBytesForCurrentThread() - before;
        Assert.Equal(0, allocated);
        Assert.NotEqual(0, checksum);
    }

    private static long ExerciseSurfaceCandidates()
    {
        Span<SegmentConeSurfaceCandidate> storage = stackalloc SegmentConeSurfaceCandidate[SegmentConeSurfaceCandidates.MaximumCandidates];
        Span<ConeSurfaceFamilyEvent> familyStorage = stackalloc ConeSurfaceFamilyEvent[128];
        long checksum = 0;
        for (int example = 0; example < 4; example++)
        {
            var selection = new ConeSurfaceSelection(storage);
            Accumulate(ExampleSegment(example), Vector3d.Zero, ref selection);
            checksum += selection.Count;
            for (int index = 0; index < selection.Count; index++)
            {
                SegmentConeSurfaceCandidate candidate = selection[index];
                if (candidate.Family == ConeSurfaceFamily.None)
                    checksum = unchecked(checksum + candidate.GetContact().Depth.m_rawValue);
                else
                {
                    var families = new ConeSurfaceFamilySelection(familyStorage);
                    candidate.AccumulateFamilyEvents(ReadOnlySpan<WideAxis3>.Empty,
                        candidate.Family == ConeSurfaceFamily.SegmentInterval ? ConeSurfacePointLocation.Interior : candidate.PointLocation, ref families);
                    for (int family = 0; family < families.Count; family++)
                        checksum = unchecked(checksum + families[family].GetContact(candidate, ReadOnlySpan<WideAxis3>.Empty).Depth.m_rawValue);
                }
                if (candidate.TryGetNormalDotSign(Covector(1, -2, 3), out int sign))
                    checksum += sign;
            }
        }
        return checksum;
    }

    private static FixedSegment ExampleSegment(int example) => example switch
    {
        0 => new FixedSegment(new Vector3d(Fixed64.Half, -Fixed64.One, -Fixed64.One), new Vector3d(Fixed64.Half, Fixed64.One, Fixed64.One)),
        1 => new FixedSegment(new Vector3d(Fixed64.Half, -Fixed64.FromFraction(1, 4), -Fixed64.Half), new Vector3d(Fixed64.Half, -Fixed64.FromFraction(1, 4), Fixed64.Half)),
        2 => new FixedSegment(new Vector3d(Fixed64.Zero, Fixed64.FromFraction(3, 4), Fixed64.Zero), new Vector3d(Fixed64.Half, -Fixed64.FromFraction(1, 4), Fixed64.Zero)),
        _ => new FixedSegment(Vector3d.Zero, Vector3d.Zero)
    };

    private static FixedContactAnchors[] FamilyContacts(SegmentConeSurfaceCandidate candidate,
        ReadOnlySpan<WideAxis3> halfspaces = default)
    {
        var storage = new ConeSurfaceFamilyEvent[candidate.GetMaximumFamilyEventCount(halfspaces.Length)];
        var contacts = new System.Collections.Generic.List<FixedContactAnchors>();
        for (int stratum = 0; stratum < 3; stratum++)
        {
            var events = new ConeSurfaceFamilySelection(storage);
            candidate.AccumulateFamilyEvents(halfspaces, stratum == 0 ? ConeSurfacePointLocation.Start
                : stratum == 1 ? ConeSurfacePointLocation.Interior : ConeSurfacePointLocation.End, ref events);
            for (int index = 0; index < events.Count; index++) contacts.Add(events[index].GetContact(candidate, halfspaces));
        }
        return contacts.ToArray();
    }

    private static WideAxis3[] NormalRayHalfspaces(WideAxis3 normal)
    {
        // n is parallel to normal iff it is perpendicular to its three
        // cross-axis directions; -normal selects the positive ray.
        WideAxis3 a = new(default, normal.Z, WideArithmetic.Negate(normal.Y));
        WideAxis3 b = new(WideArithmetic.Negate(normal.Z), default, normal.X);
        WideAxis3 c = new(normal.Y, WideArithmetic.Negate(normal.X), default);
        return new[] { a, Negate(a), b, Negate(b), c, Negate(c), Negate(normal) };
    }

    private static WideAxis3 Negate(WideAxis3 value) => new(WideArithmetic.Negate(value.X),
        WideArithmetic.Negate(value.Y), WideArithmetic.Negate(value.Z));

    // These fixtures share identity authored/cone frames. Import the exact
    // raw direction without normalization or a rounded quaternion transform.
    private static WideAxis3 Covector(Vector3d direction) => new(
        Signed320.ExtendValue(Signed192.Raw(direction.X)), Signed320.ExtendValue(Signed192.Raw(direction.Y)),
        Signed320.ExtendValue(Signed192.Raw(direction.Z)));

    private static WideAxis3 Covector(long x, long y, long z) => new(
        Signed320.ExtendValue(Signed192.Signed(x)), Signed320.ExtendValue(Signed192.Signed(y)), Signed320.ExtendValue(Signed192.Signed(z)));

    private static bool HasFamily(ConeSurfaceSelection selection, ConeSurfaceFeature feature, ConeSurfaceFamily family)
    {
        for (int index = 0; index < selection.Count; index++)
            if (selection[index].Feature == feature && selection[index].Family == family)
                return true;
        return false;
    }
}
