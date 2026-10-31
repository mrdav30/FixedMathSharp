//=======================================================================
// IPrismCapsuleContactDomain.cs
//=======================================================================
// MIT License, Copyright (c) 2024-present David Oravsky (mrdav30)
// See LICENSE file in the project root for full license information.
//=======================================================================

using System;

namespace FixedMathSharp.Geometry;

/// <summary>Supplies retained support intervals and consumes finite capsule contacts.</summary>
internal interface IPrismCapsuleContactDomain
{
    bool IncludeRecovery { get; }
    ReadOnlySpan<RationalSegmentInterval> GetSupportIntervals(in WideDirection3d normal,
        ReadOnlySpan<Signed832> projected, Signed832 denominator, in RationalSegmentInterval solidClip);
    void AddSupportContact(in FixedContactAnchors contact, scoped in CapsulePrismContactDepth native, in WideDirection3d normal,
        in WideConvexPrismRelations.ProjectionDepth depth, bool physical, int interval);
    ReadOnlySpan<RationalSegmentInterval> GetEdgeIntervals(int vertex, int kind);
    bool AdmitEdgePoint(int vertex, int kind, in Signed832 edgeN, in Signed832 edgeD, in WideDirection3d normal);
    void AddEdgeContact(in FixedContactAnchors contact, scoped in CapsulePrismContactDepth native,
        in WideDirection3d normal, int vertex, int kind, int interval);
}
