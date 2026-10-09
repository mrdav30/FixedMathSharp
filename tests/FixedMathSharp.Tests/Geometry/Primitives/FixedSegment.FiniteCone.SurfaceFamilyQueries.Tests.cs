using System;
using FixedMathSharp.Geometry;
using Xunit;

namespace FixedMathSharp.Tests;

public sealed class FixedSegmentFiniteConeSurfaceFamilyQueriesTests
{
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

    private static void AccumulateWithoutStorage()
    {
        var selection = new ConeSurfaceSelection(Span<SegmentConeSurfaceCandidate>.Empty);
        SegmentConeSurfaceCandidates.AccumulateSegmentConeSurfaceCandidates(new FixedSegment(Vector3d.Zero, Vector3d.Zero),
            Vector3d.Zero, FixedQuaternion.Identity, Vector3d.Zero, FixedQuaternion.Identity, (Fixed64)4, Fixed64.Two, ref selection);
    }

    private static void AssertPoint(FixedPointAnchor anchor, Vector3d expected)
    {
        Assert.True(anchor.TryGetPoint(out Vector3d point));
        Assert.Equal(expected, point);
    }
}
