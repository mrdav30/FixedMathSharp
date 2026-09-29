//=======================================================================
// MIT License, Copyright (c) 2024-present David Oravsky (mrdav30)
// See LICENSE file in the project root for full license information.
//=======================================================================
using System;
using static FixedMathSharp.Geometry.CylinderContactAlgebra;

namespace FixedMathSharp.Geometry;

/// <summary>One exact cone support winner and its unrounded local normal.</summary>
internal ref struct TriangleConeContactSelection
{
    internal const int Apex = 0, BasePole = 1, BaseRim = 2, Generator = 3;
    internal Span<ulong> Values, LocalNormal;
    internal Span<int> Signs, LocalSigns;
    internal int GapSign, Mask, Feature;
    internal bool HasValue;
    private readonly WideRationalBasis3d worldBasis;

    internal TriangleConeContactSelection(WideRationalBasis3d basis, Span<ulong> values, Span<int> signs,
        Span<ulong> localNormal, Span<int> localSigns)
    {
        worldBasis = basis; Values = values; Signs = signs;
        LocalNormal = localNormal; LocalSigns = localSigns;
        GapSign = Mask = Feature = 0; HasValue = false;
    }

    internal readonly bool Separated => HasValue && GapSign < 0;
    internal readonly ConvexContactCandidate Candidate => new(Values, Signs, GapSign);

    internal void Keep(scoped Span<ulong> values, scoped Span<int> signs, int gapSign, int mask, int feature)
    {
        if (Separated || HasValue && WideConvexPrismRelations.CompareConvexContactCandidates(
                new ConvexContactCandidate(values, signs, gapSign), Candidate) >= 0)
            return;
        values[..(7 * Words)].CopyTo(LocalNormal); signs[..7].CopyTo(LocalSigns);
        Span<ulong> transformed = stackalloc ulong[3 * Words];
        Span<int> transformedSigns = stackalloc int[3];
        WriteWorldDirection(worldBasis, LocalNormal[..(3 * Words)], LocalSigns[..3], transformed, transformedSigns);
        transformed.CopyTo(values); transformedSigns.CopyTo(signs);
        WriteWorldDirection(worldBasis, LocalNormal.Slice(3 * Words, 3 * Words), LocalSigns.Slice(3, 3), transformed, transformedSigns);
        transformed.CopyTo(values.Slice(3 * Words, 3 * Words)); transformedSigns.CopyTo(signs.Slice(3, 3));
        values.CopyTo(Values); signs.CopyTo(Signs);
        GapSign = gapSign; Mask = mask; Feature = feature; HasValue = true;
    }
}
