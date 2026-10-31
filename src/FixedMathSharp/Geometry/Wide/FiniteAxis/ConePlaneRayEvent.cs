//=======================================================================
// ConePlaneRayEvent.cs
//=======================================================================
// MIT License, Copyright (c) 2024-present David Oravsky (mrdav30)
// See LICENSE file in the project root for full license information.
//=======================================================================

using System;

namespace FixedMathSharp.Geometry;

/// <summary>A reconstructible finite feature in its shared plane or finite segment.</summary>
internal readonly struct ConePlaneRayEvent
{
    internal readonly ConePlaneRayEventKind Kind;
    internal readonly byte Feature, Quadrant;
    internal readonly sbyte Branch, Endpoint, Orientation;

    internal ConePlaneRayEvent(ConePlaneRayEventKind kind, int feature = 0, int quadrant = 0,
        int branch = 0, int endpoint = -1, int orientation = 0)
    {
        Kind = kind; Feature = (byte)feature; Quadrant = (byte)quadrant;
        Branch = (sbyte)branch; Endpoint = (sbyte)endpoint; Orientation = (sbyte)orientation;
    }

    internal bool Matches(in ConePlaneRayEvent other) => Kind == other.Kind && Feature == other.Feature
        && Quadrant == other.Quadrant && Branch == other.Branch && Endpoint == other.Endpoint;


}
