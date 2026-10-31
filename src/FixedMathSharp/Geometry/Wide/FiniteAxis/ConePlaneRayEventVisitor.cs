//=======================================================================
// ConePlaneRayEventVisitor.cs
//=======================================================================
// MIT License, Copyright (c) 2024-present David Oravsky (mrdav30)
// See LICENSE file in the project root for full license information.
//=======================================================================

using System;

namespace FixedMathSharp.Geometry;

/// <summary>Synchronous consumption of one admitted event; borrowed fields cannot escape the callback.</summary>
internal delegate void ConePlaneRayEventVisitor(in ConePlaneRayEventSource source, in ConePlaneRayFrame frame,
    ConePlaneRayEvent descriptor, scoped ConePlaneRayPoint point, scoped ReadOnlySpan<ulong> root,
    scoped in ConePlaneRaySelection positive, scoped in ConePlaneRaySelection negative);
