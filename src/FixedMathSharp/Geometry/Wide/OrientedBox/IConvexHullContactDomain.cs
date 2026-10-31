//=======================================================================
// IConvexHullContactDomain.cs
//=======================================================================
// MIT License, Copyright (c) 2024-present David Oravsky (mrdav30)
// See LICENSE file in the project root for full license information.
//=======================================================================

using System;

namespace FixedMathSharp.Geometry;

/// <summary>Exact finite feature restrictions in the original hull vertex and edge identities.</summary>
internal interface IConvexHullContactDomain
{
    /// <summary>Closed retained intervals; empty rejects the edge and a default interval retains its full extent.</summary>
    ReadOnlySpan<RationalSegmentInterval> GetEdgeIntervals(bool first, int start, int end, in WideAxis3 normal);
    /// <summary>Applies all domain restrictions to an exact unit parameter, including edge ends and vertex fans.</summary>
    bool ContainsPoint(bool first, int start, int end, in Signed832 numerator, in Signed832 denominator, in WideAxis3 normal);
}
