//=======================================================================
// IForwardHullFrame3d.cs
//=======================================================================
// MIT License, Copyright (c) 2024-present David Oravsky (mrdav30)
// See LICENSE file in the project root for full license information.
//=======================================================================

using System;

namespace FixedMathSharp.Geometry;

/// <summary>Forward exact geometry needed by hull projection and finite witnesses.</summary>
internal interface IForwardHullFrame3d
{
    Signed192 RotationDenominator { get; }
    Signed320 Denominator { get; }
    RationalPoint3d GetPoint(Vector3d point);
    WideDirection3d GetFaceNormal(in FixedTriangle triangle);
    WideDirection3d GetEdgeDirection(Vector3d first, Vector3d second);
    WideDirection3d GetInwardAxis(in FixedTriangle triangle, Vector3d first, Vector3d second);
    bool TryGetTriangleHalfPlanes(in FixedTriangle triangle, in RationalPoint3d point, Span<Signed832> values);
}
