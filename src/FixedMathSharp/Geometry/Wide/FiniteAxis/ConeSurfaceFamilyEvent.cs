//=======================================================================
// ConeSurfaceFamilyEvent.cs
//=======================================================================
// MIT License, Copyright (c) 2024-present David Oravsky (mrdav30)
// See LICENSE file in the project root for full license information.
//=======================================================================

using System;
using static FixedMathSharp.Geometry.ContactQuadratic;

namespace FixedMathSharp.Geometry;

/// <summary>Compact exact construction provenance; replay uses the original candidate and halfspace order.</summary>
internal readonly struct ConeSurfaceFamilyEvent
{
    internal readonly ConeSurfaceFamilyEventKind Kind;
    internal readonly int First, Second, Branch, Parameter;
    internal ConeSurfacePointLocation PointLocation { get; }
    internal ConeSurfaceFamilyEvent(ConeSurfaceFamilyEventKind kind, int first, int second, int branch, int parameter, ConeSurfacePointLocation location)
    { Kind = kind; First = first; Second = second; Branch = branch; Parameter = parameter; PointLocation = location; }
    internal FixedContactAnchors GetContact(in SegmentConeSurfaceCandidate candidate, ReadOnlySpan<WideAxis3> halfspaces) =>
        SegmentConeSurfaceCandidates.GetFamilyContact(candidate, halfspaces, this);
    internal FixedContactAnchors GetContact(in SegmentConeSurfaceCandidate candidate, ReadOnlySpan<WideAxis3> halfspaces,
        in RationalSegmentInterval clip) => SegmentConeSurfaceCandidates.GetFamilyContact(candidate, halfspaces, this, clip);
    internal bool TryGetWorldAnchors(in SegmentConeSurfaceCandidate candidate, ReadOnlySpan<WideAxis3> halfspaces, out Vector3d first, out Vector3d second) =>
        SegmentConeSurfaceCandidates.TryGetFamilyWorldAnchors(candidate, halfspaces, this, out first, out second);
    internal bool TryGetWorldAnchors(in SegmentConeSurfaceCandidate candidate, ReadOnlySpan<WideAxis3> halfspaces,
        in RationalSegmentInterval clip, out Vector3d first, out Vector3d second) =>
        SegmentConeSurfaceCandidates.TryGetFamilyWorldAnchors(candidate, halfspaces, this, out first, out second, clip);
}
