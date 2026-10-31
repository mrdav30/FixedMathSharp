//=======================================================================
// ConeSurfaceSelection.cs
//=======================================================================
// MIT License, Copyright (c) 2024-present David Oravsky (mrdav30)
// See LICENSE file in the project root for full license information.
//=======================================================================

using System;
using static FixedMathSharp.Geometry.CylinderContactAlgebra;
using static FixedMathSharp.Geometry.ContactQuadratic;

namespace FixedMathSharp.Geometry;

/// <summary>Caller-owned retained finite-support branches and degenerate families.</summary>
internal ref struct ConeSurfaceSelection
{
    private readonly Span<SegmentConeSurfaceCandidate> candidates;
    internal int Count { get; private set; }
    internal SegmentConeSurfaceCandidate this[int index] => candidates[..Count][index];

    internal ConeSurfaceSelection(Span<SegmentConeSurfaceCandidate> candidates)
    {
        this.candidates = candidates; Count = 0;
    }

    internal void Add(in SegmentConeSurfaceCandidate candidate)
    {
        if (Count == candidates.Length)
            throw new InvalidOperationException("The caller-owned surface candidate storage is full.");
        candidates[Count++] = candidate;
    }
}
