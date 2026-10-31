//=======================================================================
// ConePlaneRayEventKind.cs
//=======================================================================
// MIT License, Copyright (c) 2024-present David Oravsky (mrdav30)
// See LICENSE file in the project root for full license information.
//=======================================================================

using System;

namespace FixedMathSharp.Geometry;

internal enum ConePlaneRayEventKind : byte
{
    EdgeEndpoint, EdgeSide, EdgeStationary, LowerCircle, UpperRim, Generator,
    Axis, Apex, BaseStationary
}
