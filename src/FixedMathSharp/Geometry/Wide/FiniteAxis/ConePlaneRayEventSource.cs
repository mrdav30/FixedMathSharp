//=======================================================================
// ConePlaneRayEventSource.cs
//=======================================================================
// MIT License, Copyright (c) 2024-present David Oravsky (mrdav30)
// See LICENSE file in the project root for full license information.
//=======================================================================

using System;

namespace FixedMathSharp.Geometry;

/// <summary>An unbounded shared plane or one authored finite boundary segment.</summary>
internal readonly struct ConePlaneRayEventSource
{
    internal readonly ConePlaneRayEventScope Scope;
    internal bool IsSegment => Scope != ConePlaneRayEventScope.Plane;
    internal readonly Vector3d A, B;
    internal static ConePlaneRayEventSource Plane => default;
    internal ConePlaneRayEventSource(FixedSegment segment)
    { Scope = ConePlaneRayEventScope.Segment; A = segment.Start; B = segment.End; }
    // Native rational cuts are borrowed by the synchronous sink, never encoded
    // as rounded authored endpoints or replayed from this provenance record.
    internal static ConePlaneRayEventSource NativeSegment => new(ConePlaneRayEventScope.NativeSegment);
    private ConePlaneRayEventSource(ConePlaneRayEventScope scope)
    { Scope = scope; A = B = default; }
}
