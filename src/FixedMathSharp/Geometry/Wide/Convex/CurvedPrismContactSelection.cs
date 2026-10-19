//=======================================================================
// MIT License, Copyright (c) 2024-present David Oravsky (mrdav30)
// See LICENSE file in the project root for full license information.
//=======================================================================
using System;

namespace FixedMathSharp.Geometry;

/// <summary>One exactly ranked curved support feature against the complete prism.</summary>
internal ref struct CurvedPrismContactSelection
{
    internal const int Bottom = -1, Free = 0, Top = 1;
    internal readonly Span<ulong> Values;
    internal readonly Span<int> Signs;
    internal int GapSign, Vertex, Feature;
    internal bool HasValue;

    internal CurvedPrismContactSelection(Span<ulong> values, Span<int> signs)
    {
        Values = values; Signs = signs;
        GapSign = Vertex = Feature = 0; HasValue = false;
    }

    internal readonly ConvexContactCandidate Candidate => new(Values, Signs, GapSign);

    internal void Keep(scoped ReadOnlySpan<ulong> values, scoped ReadOnlySpan<int> signs,
        int gapSign, int vertex, int feature)
    {
        if (HasValue && WideConvexPrismRelations.CompareConvexContactCandidates(
                new ConvexContactCandidate(values, signs, gapSign), Candidate) >= 0)
            return;
        values.CopyTo(Values); signs.CopyTo(Signs);
        GapSign = gapSign; Vertex = vertex; Feature = feature; HasValue = true;
    }
}
