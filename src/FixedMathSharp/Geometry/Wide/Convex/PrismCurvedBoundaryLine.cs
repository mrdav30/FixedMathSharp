//=======================================================================
// PrismCurvedBoundaryLine.cs
//=======================================================================
// MIT License, Copyright (c) 2024-present David Oravsky (mrdav30)
// See LICENSE file in the project root for full license information.
//=======================================================================

using System;

namespace FixedMathSharp.Geometry;

/// <summary>A retained native boundary line competing with one convex source's complete exit.</summary>
internal readonly struct PrismCurvedBoundaryLine
{
    internal readonly int Face, BoundaryEdge;
    internal readonly RationalSegmentInterval Clip;
    internal readonly Fixed64 Lower, Upper;
    internal PrismCurvedBoundaryLine(int face, RationalSegmentInterval clip, Fixed64 lower, Fixed64 upper, int boundaryEdge = -1)
    { Face = face; Clip = clip; Lower = lower; Upper = upper; BoundaryEdge = boundaryEdge; }
}
