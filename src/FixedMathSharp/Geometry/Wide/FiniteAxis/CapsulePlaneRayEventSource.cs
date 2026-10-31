//=======================================================================
// CapsulePlaneRayEventSource.cs
//=======================================================================
// MIT License, Copyright (c) 2024-present David Oravsky (mrdav30)
// See LICENSE file in the project root for full license information.
//=======================================================================

using System;

namespace FixedMathSharp.Geometry;

/// <summary>An unbounded plane or one authored finite perimeter segment.</summary>
internal readonly struct CapsulePlaneRayEventSource
{
    internal readonly bool IsSegment;
    internal readonly Vector3d A, B;
    internal static CapsulePlaneRayEventSource Plane => default;
    internal CapsulePlaneRayEventSource(FixedSegment segment)
    { IsSegment = true; A = segment.Start; B = segment.End; }
}
