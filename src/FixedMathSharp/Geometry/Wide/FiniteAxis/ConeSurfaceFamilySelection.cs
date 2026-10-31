//=======================================================================
// ConeSurfaceFamilySelection.cs
//=======================================================================
// MIT License, Copyright (c) 2024-present David Oravsky (mrdav30)
// See LICENSE file in the project root for full license information.
//=======================================================================

using System;
using static FixedMathSharp.Geometry.ContactQuadratic;

namespace FixedMathSharp.Geometry;

/// <summary>Caller-owned finite pool of already admitted exact family events.</summary>
internal ref struct ConeSurfaceFamilySelection
{
    private readonly Span<ConeSurfaceFamilyEvent> events;
    internal int Count { get; private set; }
    internal ConeSurfaceFamilyEvent this[int index] => events[..Count][index];
    internal ConeSurfaceFamilySelection(Span<ConeSurfaceFamilyEvent> events) { this.events = events; Count = 0; }
    internal void Add(ConeSurfaceFamilyEvent item)
    {
        if (Count == events.Length) throw new InvalidOperationException("The caller-owned family event storage is full.");
        events[Count++] = item;
    }
}
