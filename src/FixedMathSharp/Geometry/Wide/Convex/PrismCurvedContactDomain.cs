//=======================================================================
// PrismCurvedContactDomain.cs
//=======================================================================
// MIT License, Copyright (c) 2024-present David Oravsky (mrdav30)
// See LICENSE file in the project root for full license information.
//=======================================================================

using System;

namespace FixedMathSharp.Geometry;











/// <summary>Borrowed source windows and primitive-local support covectors; owns no topology or storage.</summary>
internal readonly ref struct PrismCurvedContactDomain
{
    internal readonly bool Enabled, IncludeFarBoundary;
    internal readonly ReadOnlySpan<PrismCurvedEdgeWindow> Edges;
    internal readonly ReadOnlySpan<WideDirection3d> FanCovectors;
    internal readonly ReadOnlySpan<PrismCurvedFaceCertificate> Certificates;
    internal PrismCurvedContactDomain(ReadOnlySpan<PrismCurvedEdgeWindow> edges, ReadOnlySpan<WideDirection3d> fanCovectors, bool includeFarBoundary = false, ReadOnlySpan<PrismCurvedFaceCertificate> certificates = default)
    { Enabled = true; IncludeFarBoundary = includeFarBoundary; Edges = edges; FanCovectors = fanCovectors; Certificates = certificates; }
}
