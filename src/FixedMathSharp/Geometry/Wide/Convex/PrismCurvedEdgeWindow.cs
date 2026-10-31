//=======================================================================
// PrismCurvedEdgeWindow.cs
//=======================================================================
// MIT License, Copyright (c) 2024-present David Oravsky (mrdav30)
// See LICENSE file in the project root for full license information.
//=======================================================================

using System;

namespace FixedMathSharp.Geometry;

/// <summary>An open real-edge interval or closed point with one exact incident support fan.</summary>
internal readonly struct PrismCurvedEdgeWindow
{
    internal readonly int Edge, ReferenceVertex, FanStart, FanCount;
    internal readonly RationalSegmentInterval Interval;
    internal readonly bool Point;
    internal PrismCurvedEdgeWindow(int edge, RationalSegmentInterval interval, bool point,
        int referenceVertex, int fanStart, int fanCount)
    { Edge = edge; Interval = interval; Point = point; ReferenceVertex = referenceVertex; FanStart = fanStart; FanCount = fanCount; }
}
