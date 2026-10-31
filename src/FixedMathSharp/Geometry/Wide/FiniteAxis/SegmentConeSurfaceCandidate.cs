//=======================================================================
// SegmentConeSurfaceCandidate.cs
//=======================================================================
// MIT License, Copyright (c) 2024-present David Oravsky (mrdav30)
// See LICENSE file in the project root for full license information.
//=======================================================================

using System;
using static FixedMathSharp.Geometry.CylinderContactAlgebra;
using static FixedMathSharp.Geometry.ContactQuadratic;

namespace FixedMathSharp.Geometry;

/// <summary>One exact chart branch or a retained degenerate family.</summary>
internal readonly struct SegmentConeSurfaceCandidate
{
    /// <summary>Tests a unique retained parameter against the original segment's exact clip.</summary>
    /// <remarks>Interval families must instead be clipped before their family events are selected.</remarks>
    internal bool IsInInterval(in RationalSegmentInterval clip, bool interiorOnly = false) =>
        SegmentConeSurfaceCandidates.IsInInterval(this, clip, interiorOnly);

    internal int GetMaximumFamilyEventCount(int halfspaceCount) =>
        SegmentConeSurfaceCandidates.GetMaximumFamilyEventCount(this, halfspaceCount);

    internal bool AccumulateFamilyEvents(scoped ReadOnlySpan<WideAxis3> authoredHalfspaces,
        ConeSurfacePointLocation location, scoped ref ConeSurfaceFamilySelection selection) =>
        SegmentConeSurfaceCandidates.AccumulateFamilyEvents(this, authoredHalfspaces, location, ref selection);

    internal bool AccumulateFamilyEvents(scoped ReadOnlySpan<WideAxis3> authoredHalfspaces,
        ConeSurfacePointLocation location, in RationalSegmentInterval clip, scoped ref ConeSurfaceFamilySelection selection) =>
        SegmentConeSurfaceCandidates.AccumulateFamilyEvents(this, authoredHalfspaces, location, ref selection, clip);

    internal ConeSurfaceFeature Feature { get; }
    internal ConeSurfaceFamily Family { get; }
    internal ConeSurfacePointLocation PointLocation { get; }
    /// <summary>Whether a unique retained segment point is strictly interior; interval families have no unique point.</summary>
    internal bool IsSegmentInterior => PointLocation == ConeSurfacePointLocation.Interior;
    internal readonly SegmentConeSurfaceInput Input;
    internal readonly int Chart, RootOrdinal;

    internal SegmentConeSurfaceCandidate(in SegmentConeSurfaceInput input, int chart, int rootOrdinal, ConeSurfacePointLocation location)
    {
        Input = input; Chart = chart; RootOrdinal = rootOrdinal;
        Feature = ConeSurfaceFeature.Rim; Family = ConeSurfaceFamily.None; PointLocation = location;
    }

    internal SegmentConeSurfaceCandidate(in SegmentConeSurfaceInput input, ConeSurfaceFeature feature, ConeSurfaceFamily family)
    {
        Input = input; Feature = feature; Family = family;
        PointLocation = ConeSurfacePointLocation.SegmentIntervalUnspecified; Chart = RootOrdinal = 0;
    }

    internal SegmentConeSurfaceCandidate(in SegmentConeSurfaceInput input, ConeSurfaceFeature feature,
        ConeSurfaceFamily family, int chart, int ordinal, ConeSurfacePointLocation location)
    {
        Input = input; Feature = feature; Family = family; Chart = chart;
        RootOrdinal = ordinal; PointLocation = location;
    }

    internal FixedContactAnchors GetContact() => Family == ConeSurfaceFamily.None
        ? Feature == ConeSurfaceFeature.Apex ? SegmentConeSurfaceCandidates.GetApexContact(this)
            : Feature == ConeSurfaceFeature.Base ? SegmentConeSurfaceCandidates.GetBaseContact(this)
            : Feature == ConeSurfaceFeature.Side ? SegmentConeSurfaceCandidates.GetSideContact(this)
            : Chart >= 0 ? SegmentConeSurfaceCandidates.GetRimRootContact(this)
                : SegmentConeSurfaceCandidates.GetRimPointContact(this)
        : throw new InvalidOperationException("A family requires an admitted exact parameter before materialization.");

    internal bool TryGetNormalDotSign(WideAxis3 authoredCovector, out int sign) =>
        SegmentConeSurfaceCandidates.TryGetNormalDotSign(this, authoredCovector, out sign);

}
