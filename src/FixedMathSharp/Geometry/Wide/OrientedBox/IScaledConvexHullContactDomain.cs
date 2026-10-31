//=======================================================================
// IScaledConvexHullContactDomain.cs
//=======================================================================
// MIT License, Copyright (c) 2024-present David Oravsky (mrdav30)
// See LICENSE file in the project root for full license information.
//=======================================================================

using System;

namespace FixedMathSharp.Geometry;

/// <summary>Exact retained features for scaled authored hull contact selection.</summary>
internal interface IScaledConvexHullContactDomain
{
    ReadOnlySpan<RationalSegmentInterval> GetEdgeIntervals(bool first, int start, int end, in WideDirection3d normal);
    bool ContainsPoint(bool first, int start, int end, in Signed832 numerator, in Signed832 denominator, in WideDirection3d normal);
    bool ContainsPoint(bool first, int start, int end, scoped ContactQuadratic numerator,
        scoped ContactQuadratic denominator, in WideDirection3d normal);
}
