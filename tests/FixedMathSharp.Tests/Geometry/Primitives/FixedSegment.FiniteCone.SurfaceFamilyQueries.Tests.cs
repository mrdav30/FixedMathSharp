using System;
using FixedMathSharp.Geometry;
using Xunit;

namespace FixedMathSharp.Tests;

public sealed class FixedSegmentFiniteConeSurfaceFamilyQueriesTests
{
    [Theory]
    [InlineData(false)]
    [InlineData(true)]
    public void ApexEndpointNormal_IntersectsConeAndSegmentHalfspacesBeforeRounding(bool reverse)
    {
        var apex = new Vector3d(0, 2, 0);
        FixedSegment segment = Ordered(apex, apex + Vector3d.Right, reverse);
        SegmentConeSurfaceCandidate candidate = Find(segment, ConeSurfaceFeature.Apex, ConeSurfaceFamily.NormalCone);
        var boundary = new Vector3d(-2, -1, 0);

        Assert.True(candidate.TryGetContactForAuthoredNormal(Covector(boundary), out FixedContactAnchors contact));
        AssertTouch(contact, apex);
        Assert.True(candidate.TryGetNormalDotSignForAuthoredNormal(Covector(boundary), Covector(1, -2, 0), out int sign));
        Assert.Equal(0, sign);
        // Both directions lie in the apex normal cone; only the negative-X
        // direction also satisfies this finite segment's endpoint halfspace.
        Assert.False(candidate.TryGetContactForAuthoredNormal(Covector(new Vector3d(2, -1, 0)), out _));
        Assert.False(candidate.TryGetContactForAuthoredNormal(Covector(new Vector3d(-(Fixed64)2 - Fixed64.MinIncrement, -Fixed64.One, Fixed64.Zero)), out _));
        Assert.False(candidate.TryGetNormalDotSignForAuthoredNormal(Covector(Vector3d.Up), Covector(0, 1, 0), out _));
    }

    [Fact]
    public void ApexInteriorNormal_RequiresExactTangencyToTheSegment()
    {
        var apex = new Vector3d(0, 2, 0);
        SegmentConeSurfaceCandidate candidate = Find(new FixedSegment(apex - Vector3d.Right, apex + Vector3d.Right),
            ConeSurfaceFeature.Apex, ConeSurfaceFamily.NormalCone);
        Assert.True(candidate.IsSegmentInterior);
        Assert.True(candidate.TryGetContactForAuthoredNormal(Covector(new Vector3d(0, -1, 2)), out FixedContactAnchors contact));
        AssertTouch(contact, apex);
        Assert.False(candidate.TryGetContactForAuthoredNormal(Covector(new Vector3d(Fixed64.MinIncrement, -Fixed64.One, Fixed64.Zero)), out _));
    }

    [Fact]
    public void RimPointNormal_AdmitsCapAndSideBoundaryButRejectsAngularRawNeighbor()
    {
        var rim = new Vector3d(2, -2, 0);
        SegmentConeSurfaceCandidate candidate = Find(new FixedSegment(rim, rim), ConeSurfaceFeature.Rim, ConeSurfaceFamily.NormalCone);
        var side = new Vector3d(-2, -1, 0);
        Assert.True(candidate.TryGetContactForAuthoredNormal(Covector(side), out FixedContactAnchors sideContact));
        AssertTouch(sideContact, rim);
        Assert.True(candidate.TryGetContactForAuthoredNormal(Covector(Vector3d.Up), out FixedContactAnchors capContact));
        AssertTouch(capContact, rim);
        Assert.Equal(Vector3d.Up, capContact.Normal);
        Assert.True(candidate.TryGetNormalDotSignForAuthoredNormal(Covector(side), Covector(1, -2, 0), out int sign));
        Assert.Equal(0, sign);
        Assert.False(candidate.TryGetContactForAuthoredNormal(Covector(new Vector3d(-(Fixed64)2, -Fixed64.One - Fixed64.MinIncrement, Fixed64.Zero)), out _));
        Assert.False(candidate.TryGetContactForAuthoredNormal(Covector(new Vector3d(-Fixed64.One, Fixed64.Zero, Fixed64.MinIncrement)), out _));
        Assert.False(candidate.TryGetContactForAuthoredNormal(Covector(Vector3d.Right), out _));
        Assert.False(candidate.TryGetContactForAuthoredNormal(Covector(Vector3d.Zero), out _));
        Assert.False(candidate.TryGetNormalDotSignForAuthoredNormal(Covector(Vector3d.Right), Covector(1, 0, 0), out _));
    }

    [Theory]
    [InlineData(false)]
    [InlineData(true)]
    public void RimEndpointNormal_RetainsOnlyTheFiniteSegmentHalfspace(bool reverse)
    {
        var rim = new Vector3d(2, -2, 0);
        SegmentConeSurfaceCandidate candidate = Find(Ordered(rim, rim - Vector3d.Right, reverse),
            ConeSurfaceFeature.Rim, ConeSurfaceFamily.NormalCone);
        Assert.True(candidate.TryGetContactForAuthoredNormal(Covector(Vector3d.Up), out FixedContactAnchors contact));
        AssertTouch(contact, rim);
        Assert.False(candidate.TryGetContactForAuthoredNormal(Covector(new Vector3d(-2, -1, 0)), out _));
    }

    [Theory]
    [InlineData(-2, 1)]
    [InlineData(0, 0)]
    [InlineData(2, -1)]
    public void ZeroRadiusPoint_NormalDomainChangesAtTheFiniteAxisCaps(int y, int axialDirection)
    {
        var point = new Vector3d(0, y, 0);
        SegmentConeSurfaceCandidate candidate = Find(new FixedSegment(point, point), ConeSurfaceFeature.Side,
            ConeSurfaceFamily.SegmentInterval, Fixed64.Zero);
        Assert.True(candidate.TryGetContactAtParameterAndAuthoredNormal(Fixed64.Half, Covector(Vector3d.Right), out FixedContactAnchors radial));
        AssertTouch(radial, point);
        if (axialDirection != 0)
        {
            Vector3d allowed = Vector3d.Up * (Fixed64)axialDirection;
            Assert.True(candidate.TryGetContactAtParameterAndAuthoredNormal(Fixed64.Half, Covector(allowed), out FixedContactAnchors axial));
            AssertTouch(axial, point);
            Assert.Equal(allowed, axial.Normal);
            Assert.False(candidate.TryGetContactAtParameterAndAuthoredNormal(Fixed64.Half, Covector(-allowed), out _));
        }
        else
        {
            Assert.False(candidate.TryGetContactAtParameterAndAuthoredNormal(Fixed64.Half, Covector(Vector3d.Up), out _));
            Assert.False(candidate.TryGetContactAtParameterAndAuthoredNormal(Fixed64.Half, Covector(-Vector3d.Up), out _));
        }
        Assert.False(candidate.TryGetContactAtParameterAndAuthoredNormal(Fixed64.Half, Covector(Vector3d.Zero), out _));
        Assert.False(candidate.TryGetContactAtParameter(Fixed64.Half, out _));
        Assert.False(candidate.TryGetContactAtIntervalEndpoint(false, out _));
    }

    [Theory]
    [InlineData(false)]
    [InlineData(true)]
    public void ZeroRadiusCrossing_UsesTheCorrectEndpointNormalHalfspace(bool reverse)
    {
        SegmentConeSurfaceCandidate candidate = Find(Ordered(Vector3d.Zero, Vector3d.Right, reverse),
            ConeSurfaceFeature.Side, ConeSurfaceFamily.NormalCone, Fixed64.Zero);
        Assert.False(candidate.IsSegmentInterior);
        Assert.True(candidate.TryGetContactForAuthoredNormal(Covector(-Vector3d.Right), out FixedContactAnchors contact));
        AssertTouch(contact, Vector3d.Zero);
        Assert.False(candidate.TryGetContactForAuthoredNormal(Covector(Vector3d.Right), out _));
        Assert.False(candidate.TryGetContactForAuthoredNormal(Covector(Vector3d.Up), out _));
        Assert.True(candidate.TryGetNormalDotSignForAuthoredNormal(Covector(-Vector3d.Right), Covector(1, 0, 0), out int sign));
        Assert.Equal(-1, sign);
    }

    [Fact]
    public void ZeroRadiusInterval_RejectsRawNeighborsBeyondBothClippedCaps()
    {
        SegmentConeSurfaceCandidate candidate = Find(new FixedSegment(new Vector3d(0, -4, 0), new Vector3d(0, 4, 0)),
            ConeSurfaceFeature.Side, ConeSurfaceFamily.SegmentInterval, Fixed64.Zero);
        Fixed64 lower = Fixed64.Quarter, upper = (Fixed64)3 / 4;
        Assert.True(candidate.TryGetContactAtParameterAndAuthoredNormal(lower, Covector(Vector3d.Right), out FixedContactAnchors first));
        Assert.True(candidate.TryGetContactAtParameterAndAuthoredNormal(upper, Covector(Vector3d.Right), out FixedContactAnchors last));
        AssertTouch(first, new Vector3d(0, -2, 0));
        AssertTouch(last, new Vector3d(0, 2, 0));
        Assert.False(candidate.TryGetContactAtParameterAndAuthoredNormal(lower - Fixed64.MinIncrement, Covector(Vector3d.Right), out _));
        Assert.False(candidate.TryGetContactAtParameterAndAuthoredNormal(upper + Fixed64.MinIncrement, Covector(Vector3d.Right), out _));
        // Cap normals still must be perpendicular to this segment because
        // both clipped cap points are interior to the authored segment.
        Assert.False(candidate.TryGetContactAtIntervalEndpointForAuthoredNormal(false, Covector(Vector3d.Up), out _));
        Assert.False(candidate.TryGetContactAtIntervalEndpointForAuthoredNormal(true, Covector(-Vector3d.Up), out _));
    }

    [Theory]
    [InlineData(false)]
    [InlineData(true)]
    public void SideRotationCircle_RejectsDirectionsOutsideItsEndpointDomain(bool reverse)
    {
        SegmentConeSurfaceCandidate candidate = Find(Ordered(Vector3d.Zero, Vector3d.Right, reverse),
            ConeSurfaceFeature.Side, ConeSurfaceFamily.RotationCircle);
        var right = new Vector2d(1, 0);
        Assert.True(candidate.TryGetContactForRadialDirection(right, out FixedContactAnchors contact));
        Assert.Equal(Fixed64.FromRaw(3841535534L), contact.Depth);
        AssertPoint(contact.FirstAnchor, Vector3d.Zero);
        AssertPoint(contact.SecondAnchor, new Vector3d((Fixed64)4 / 5, (Fixed64)2 / 5, Fixed64.Zero));
        Assert.True(candidate.TryGetNormalDotSignForRadialDirection(right, Covector(1, 0, 0), out int sign));
        Assert.Equal(-1, sign);
        Assert.False(candidate.TryGetContactForRadialDirection(new Vector2d(-1, 0), out _));
        Assert.False(candidate.TryGetNormalDotSignForRadialDirection(new Vector2d(-1, 0), Covector(1, 0, 0), out _));
        Assert.False(candidate.TryGetContactForRadialDirection(Vector2d.Zero, out _));
    }

    [Fact]
    public void BaseInterval_RejectsQueriesForOtherFamilyDomains()
    {
        SegmentConeSurfaceCandidate candidate = Find(new FixedSegment(-Vector3d.Right, Vector3d.Right),
            ConeSurfaceFeature.Base, ConeSurfaceFamily.SegmentInterval);
        Assert.True(candidate.TryGetContactAtParameter(Fixed64.Half, out FixedContactAnchors contact));
        Assert.Equal(Fixed64.Two, contact.Depth);
        Assert.True(candidate.TryGetNormalDotSign(Covector(0, -1, 0), out int sign));
        Assert.Equal(-1, sign);
        Assert.False(candidate.TryGetContactForAuthoredNormal(Covector(Vector3d.Up), out _));
        Assert.False(candidate.TryGetContactForRadialDirection(new Vector2d(1, 0), out _));
        Assert.False(candidate.TryGetNormalDotSignForRadialDirection(new Vector2d(1, 0), Covector(1, 0, 0), out _));
        Assert.False(candidate.TryGetContactAtParameterAndAuthoredNormal(Fixed64.Half, Covector(Vector3d.Right), out _));
        Assert.False(candidate.TryGetContactAtIntervalEndpointForAuthoredNormal(false, Covector(Vector3d.Right), out _));
        Assert.Throws<InvalidOperationException>(() => candidate.GetContact());
    }

    [Fact]
    public void UniqueApexWitness_RejectsFamilyQueriesWithoutLosingItsContact()
    {
        SegmentConeSurfaceCandidate candidate = Find(new FixedSegment(Vector3d.Zero, Vector3d.Zero),
            ConeSurfaceFeature.Apex, ConeSurfaceFamily.None);
        Assert.Equal(Fixed64.Two, candidate.GetContact().Depth);
        Assert.False(candidate.TryGetContactAtParameter(Fixed64.Half, out _));
        Assert.False(candidate.TryGetContactAtIntervalEndpoint(false, out _));
        Assert.False(candidate.TryGetContactForAuthoredNormal(Covector(-Vector3d.Up), out _));
        Assert.False(candidate.TryGetContactForRadialDirection(new Vector2d(1, 0), out _));
        Assert.False(candidate.TryGetNormalDotSignForAuthoredNormal(Covector(-Vector3d.Up), Covector(0, 1, 0), out _));
    }

    [Fact]
    public void EmptyCandidateStorage_RejectsAdmittedGeometryWithoutDroppingWitnesses() =>
        Assert.Throws<InvalidOperationException>(AccumulateWithoutStorage);

    [Fact]
    public void LargeCone_ClampsUnrepresentableDepthWhilePreservingFiniteAnchors()
    {
        Fixed64 dimension = Fixed64.MaxValue - Fixed64.MinIncrement;
        Fixed64 half = Fixed64.FromRaw(dimension.m_rawValue / 2);
        var point = new Vector3d(dimension, -half, Fixed64.Zero);
        Span<SegmentConeSurfaceCandidate> storage = stackalloc SegmentConeSurfaceCandidate[SegmentConeSurfaceCandidates.MaximumCandidates];
        var selection = new ConeSurfaceSelection(storage);
        Assert.True(SegmentConeSurfaceCandidates.AccumulateSegmentConeSurfaceCandidates(new FixedSegment(point, point),
            Vector3d.Zero, FixedQuaternion.Identity, Vector3d.Zero, FixedQuaternion.Identity, dimension, dimension, ref selection));
        bool apex = false, oppositeRim = false;
        for (int index = 0; index < selection.Count; index++)
        {
            SegmentConeSurfaceCandidate candidate = selection[index];
            if (candidate.Family != ConeSurfaceFamily.None
                || candidate.Feature != ConeSurfaceFeature.Apex && candidate.Feature != ConeSurfaceFeature.Rim) continue;
            FixedContactAnchors contact = candidate.GetContact();
            // Distances are sqrt(2)*dimension to the apex and 2*dimension
            // to the opposite rim; both exceed MaxValue without anchor overflow.
            Assert.Equal(Fixed64.MaxValue, contact.Depth);
            Assert.True(contact.DepthIsClamped);
            AssertPoint(contact.FirstAnchor, point);
            if (candidate.Feature == ConeSurfaceFeature.Apex)
            {
                apex = true;
                AssertPoint(contact.SecondAnchor, new Vector3d(Fixed64.Zero, half, Fixed64.Zero));
            }
            else
            {
                oppositeRim = true;
                AssertPoint(contact.SecondAnchor, new Vector3d(-dimension, -half, Fixed64.Zero));
            }
        }
        Assert.True(apex && oppositeRim);
    }

    private static SegmentConeSurfaceCandidate Find(FixedSegment segment, ConeSurfaceFeature feature,
        ConeSurfaceFamily family, Fixed64? radius = null)
    {
        Span<SegmentConeSurfaceCandidate> storage = stackalloc SegmentConeSurfaceCandidate[SegmentConeSurfaceCandidates.MaximumCandidates];
        var selection = new ConeSurfaceSelection(storage);
        Assert.True(SegmentConeSurfaceCandidates.AccumulateSegmentConeSurfaceCandidates(segment,
            Vector3d.Zero, FixedQuaternion.Identity, Vector3d.Zero, FixedQuaternion.Identity,
            (Fixed64)4, radius ?? Fixed64.Two, ref selection));
        for (int index = 0; index < selection.Count; index++)
            if (selection[index].Feature == feature && selection[index].Family == family) return selection[index];
        throw new InvalidOperationException($"Expected admitted {feature}/{family} descriptor.");
    }

    private static void AccumulateWithoutStorage()
    {
        var selection = new ConeSurfaceSelection(Span<SegmentConeSurfaceCandidate>.Empty);
        SegmentConeSurfaceCandidates.AccumulateSegmentConeSurfaceCandidates(new FixedSegment(Vector3d.Zero, Vector3d.Zero),
            Vector3d.Zero, FixedQuaternion.Identity, Vector3d.Zero, FixedQuaternion.Identity, (Fixed64)4, Fixed64.Two, ref selection);
    }

    private static FixedSegment Ordered(Vector3d first, Vector3d second, bool reverse) =>
        reverse ? new FixedSegment(second, first) : new FixedSegment(first, second);

    // These fixtures share identity authored/cone frames. Import the exact
    // raw direction without normalization or a rounded quaternion transform.
    private static WideAxis3 Covector(Vector3d direction) => new(
        Signed320.ExtendValue(Signed192.Raw(direction.X)), Signed320.ExtendValue(Signed192.Raw(direction.Y)),
        Signed320.ExtendValue(Signed192.Raw(direction.Z)));

    private static WideAxis3 Covector(long x, long y, long z) => new(
        Signed320.ExtendValue(Signed192.Signed(x)), Signed320.ExtendValue(Signed192.Signed(y)), Signed320.ExtendValue(Signed192.Signed(z)));

    private static void AssertTouch(FixedContactAnchors contact, Vector3d point)
    {
        Assert.Equal(Fixed64.Zero, contact.Depth);
        Assert.False(contact.DepthIsClamped);
        AssertPoint(contact.FirstAnchor, point);
        AssertPoint(contact.SecondAnchor, point);
    }

    private static void AssertPoint(FixedPointAnchor anchor, Vector3d expected)
    {
        Assert.True(anchor.TryGetPoint(out Vector3d point));
        Assert.Equal(expected, point);
    }
}
