//=======================================================================
// CapsulePlaneRaySelectionMetadata.cs
//=======================================================================
// MIT License, Copyright (c) 2024-present David Oravsky (mrdav30)
// See LICENSE file in the project root for full license information.
//=======================================================================

using System;

namespace FixedMathSharp.Geometry;

/// <summary>Metadata over independently retained winner coefficient/sign banks.</summary>
internal readonly struct CapsulePlaneRaySelectionMetadata
{
    internal readonly bool HasValue, UsesValueRoot;
    internal readonly int Primitive;
    internal readonly CapsulePlaneRayEventSource Source;
    internal readonly CapsulePlaneRayEvent Event;
    internal readonly CapsulePlaneRayValueMetadata Value;
    internal CapsulePlaneRaySelectionMetadata(bool hasValue, bool usesValueRoot, int primitive,
        CapsulePlaneRayEventSource source, CapsulePlaneRayEvent descriptor, CapsulePlaneRayValueMetadata value)
    { HasValue = hasValue; UsesValueRoot = usesValueRoot; Primitive = primitive; Source = source; Event = descriptor; Value = value; }
}
