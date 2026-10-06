//=======================================================================
// MIT License, Copyright (c) 2024-present David Oravsky (mrdav30)
// See LICENSE file in the project root for full license information.
//=======================================================================
using System;

namespace FixedMathSharp.Geometry;

/// <summary>One exact cone support winner and its unrounded local normal.</summary>
internal ref struct TriangleConeContactSelection
{
    internal const int Apex = 0, BasePole = 1, BaseRim = 2, Generator = 3;
    internal Span<ulong> Values;
    internal Span<int> Signs;
    internal int GapSign, Mask, Feature;
    internal bool HasValue;
    internal TriangleConeContactSelection(Span<ulong> values, Span<int> signs)
    {
        Values = values; Signs = signs;
        GapSign = Mask = Feature = 0; HasValue = false;
    }

    internal readonly bool Separated => HasValue && GapSign < 0;
    internal readonly ConvexContactCandidate Candidate => new(Values, Signs, GapSign);

    internal void Keep(scoped Span<ulong> values, scoped Span<int> signs, int gapSign, int mask, int feature)
    {
        if (Separated || HasValue && WideConvexPrismRelations.CompareConvexContactCandidates(
                new ConvexContactCandidate(values, signs, gapSign), Candidate) >= 0)
            return;
        values.CopyTo(Values); signs.CopyTo(Signs);
        GapSign = gapSign; Mask = mask; Feature = feature; HasValue = true;
    }
}
