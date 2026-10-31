//=======================================================================
// CapsulePlaneRayEventVisitor.cs
//=======================================================================
// MIT License, Copyright (c) 2024-present David Oravsky (mrdav30)
// See LICENSE file in the project root for full license information.
//=======================================================================

using System;

namespace FixedMathSharp.Geometry;

/// <summary>Synchronous exact event consumption; borrowed candidate storage cannot escape.</summary>
internal delegate void CapsulePlaneRayEventVisitor(in CapsulePlaneRayEventSource source,
    in CapsulePlaneRayFrame frame, CapsulePlaneRayEvent descriptor, int orientation,
    scoped in CapsulePlaneRaySelection candidate);
