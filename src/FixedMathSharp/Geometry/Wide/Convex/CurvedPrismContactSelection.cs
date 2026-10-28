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
    internal int Window, Certificate;
    internal readonly PrismCurvedContactDomain Domain;

    internal CurvedPrismContactSelection(Span<ulong> values, Span<int> signs)
    {
        Values = values; Signs = signs;
        GapSign = Vertex = Feature = Window = 0; HasValue = false; Certificate = -1; Domain = default;
    }

    internal CurvedPrismContactSelection(Span<ulong> values, Span<int> signs, PrismCurvedContactDomain domain)
        : this(values, signs) { Domain = domain; }

    internal readonly ConvexContactCandidate Candidate => new(Values, Signs, GapSign);

    internal void Keep(in CurvedPrismContactGeometry geometry, scoped ReadOnlySpan<ulong> values, scoped ReadOnlySpan<int> signs,
        int gapSign, int vertex, int feature)
    {
        if (HasValue && WideConvexPrismRelations.CompareConvexContactCandidates(
                new ConvexContactCandidate(values, signs, gapSign), Candidate) >= 0)
            return;
        int window = 0, certificate = -1;
        if (Domain.Enabled && !CurvedPrismContact.TryAdmitAnalyticCandidate(geometry, Domain, values, signs,
            gapSign, vertex, feature, out window, out certificate)) return;
        values.CopyTo(Values); signs.CopyTo(Signs); Window = window; Certificate = certificate;
        GapSign = gapSign; Vertex = vertex; Feature = feature; HasValue = true;
    }
}
