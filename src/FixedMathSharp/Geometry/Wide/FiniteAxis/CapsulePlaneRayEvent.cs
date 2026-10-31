//=======================================================================
// CapsulePlaneRayEvent.cs
//=======================================================================
// MIT License, Copyright (c) 2024-present David Oravsky (mrdav30)
// See LICENSE file in the project root for full license information.
//=======================================================================

using System;

namespace FixedMathSharp.Geometry;

/// <summary>A reconstructible exact point in the plane or its finite perimeter.</summary>
internal readonly struct CapsulePlaneRayEvent
{
    internal readonly CapsulePlaneRayEventKind Kind;
    internal readonly sbyte Feature, Branch, Orientation;
    internal CapsulePlaneRayEvent(CapsulePlaneRayEventKind kind, int feature = 0, int branch = 0, int orientation = 0)
    { Kind = kind; Feature = (sbyte)feature; Branch = (sbyte)branch; Orientation = (sbyte)orientation; }
    internal bool Matches(in CapsulePlaneRayEvent other) =>
        Kind == other.Kind && Feature == other.Feature && Branch == other.Branch && Orientation == other.Orientation;
}
