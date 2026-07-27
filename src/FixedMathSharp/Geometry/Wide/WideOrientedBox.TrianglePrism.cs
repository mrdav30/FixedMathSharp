//=======================================================================
// WideOrientedBox.TrianglePrism.cs
//=======================================================================
// MIT License, Copyright (c) 2024–present David Oravsky (mrdav30)
// See LICENSE file in the project root for full license information.
//=======================================================================

using System;

namespace FixedMathSharp.Geometry;

/// <content>
/// Contains logic for detecting and resolving contact between a triangle
/// (used as a triangular prism cross-section) and an oriented box, using
/// separating axis tests over the prism and triangle edges/normals.
/// </content>
internal static partial class WideOrientedBox
{
    internal static bool TryGetTrianglePrismContact(
        Vector3d triangleOrigin,
        FixedQuaternion triangleRotation,
        FixedTriangle triangle,
        Vector3d prismOrigin,
        Fixed64 prismRotation,
        ReadOnlySpan<Vector2d> prismLocalOffsets,
        Fixed64 prismHalfThickness,
        out FixedContactAnchors contact)
    {
        if (triangle.IsDegenerate)
        {
            contact = default;
            return false;
        }

        Span<Vector3d> trianglePoints = stackalloc Vector3d[3]
        {
            triangle.A,
            triangle.B,
            triangle.C,
        };
        RationalBasis triangleBasis = new(triangleRotation);
        var best = default(PolytopePenetration);
        WideAxis3 up = new(default, Signed320.One, default);
        WideAxis3 firstEdge = GetHullEdge(
            triangleBasis,
            triangle.A,
            triangle.B);
        WideAxis3 secondEdge = GetHullEdge(
            triangleBasis,
            triangle.A,
            triangle.C);
        WideGeometry.GetDifferenceCrossProduct3D(
            triangle.B.X,
            triangle.A.X,
            triangle.B.Y,
            triangle.A.Y,
            triangle.B.Z,
            triangle.A.Z,
            triangle.C.X,
            triangle.A.X,
            triangle.C.Y,
            triangle.A.Y,
            triangle.C.Z,
            triangle.A.Z,
            out Signed192 localNormalX,
            out Signed192 localNormalY,
            out Signed192 localNormalZ);
        if (!TryKeepTrianglePrismAxis(
                up,
                triangleOrigin,
                triangleBasis,
                trianglePoints,
                prismOrigin,
                prismRotation,
                prismLocalOffsets,
                prismHalfThickness,
                ref best)
            || !TryKeepTrianglePrismAxis(
                TransformLocalAxis(
                    triangleBasis,
                    localNormalX,
                    localNormalY,
                    localNormalZ),
                triangleOrigin,
                triangleBasis,
                trianglePoints,
                prismOrigin,
                prismRotation,
                prismLocalOffsets,
                prismHalfThickness,
                ref best))
        {
            contact = default;
            return false;
        }

        Span<WideAxis3> triangleEdges = stackalloc WideAxis3[3]
        {
            firstEdge,
            GetHullEdge(triangleBasis, triangle.B, triangle.C),
            GetHullEdge(triangleBasis, triangle.C, triangle.A),
        };
        for (int prismIndex = 0;
            prismIndex < prismLocalOffsets.Length;
            prismIndex++)
        {
            Vector2d prismStart = prismLocalOffsets[prismIndex];
            Vector2d prismEnd = prismLocalOffsets[
                prismIndex + 1 == prismLocalOffsets.Length
                    ? 0
                    : prismIndex + 1];
            GetRotatedPlanarEdge(
                prismRotation,
                prismStart,
                prismEnd,
                out Signed192 prismEdgeX,
                out Signed192 prismEdgeZ);
            WideAxis3 prismEdge = new(
                Signed320.ExtendValue(prismEdgeX),
                default,
                Signed320.ExtendValue(prismEdgeZ));
            WideAxis3 prismFace = new(
                prismEdge.Z,
                default,
                WideArithmetic.SubtractSigned320(
                    default,
                    prismEdge.X));
            if (!TryKeepTrianglePrismAxis(
                    prismFace,
                    triangleOrigin,
                    triangleBasis,
                    trianglePoints,
                    prismOrigin,
                    prismRotation,
                    prismLocalOffsets,
                    prismHalfThickness,
                    ref best))
            {
                contact = default;
                return false;
            }

            for (int triangleIndex = 0;
                triangleIndex < triangleEdges.Length;
                triangleIndex++)
            {
                WideAxis3 triangleEdge = triangleEdges[triangleIndex];
                if (!TryKeepTrianglePrismAxis(
                        CrossWithUp(triangleEdge),
                        triangleOrigin,
                        triangleBasis,
                        trianglePoints,
                        prismOrigin,
                        prismRotation,
                        prismLocalOffsets,
                        prismHalfThickness,
                        ref best)
                    || !TryKeepTrianglePrismAxis(
                        Cross(triangleEdge, prismEdge),
                        triangleOrigin,
                        triangleBasis,
                        trianglePoints,
                        prismOrigin,
                        prismRotation,
                        prismLocalOffsets,
                        prismHalfThickness,
                        ref best))
                {
                    contact = default;
                    return false;
                }
            }
        }

        WideAxis3 orientedAxis = best.Negate ? -best.Axis : best.Axis;
        Vector3d normal = WideGeometry.GetNormalized(
            Signed576.ExtendValue(orientedAxis.X),
            Signed576.ExtendValue(orientedAxis.Y),
            Signed576.ExtendValue(orientedAxis.Z));
        Vector3d triangleLocalPoint = GetHullSupportLocalPoint(
            triangleBasis,
            trianglePoints,
            orientedAxis,
            maximize: true);
        Vector3d prismLocalPoint = GetPrismSupportOffset(
            prismLocalOffsets,
            prismHalfThickness,
            prismRotation,
            orientedAxis);
        contact = new FixedContactAnchors(
            new FixedPointAnchor(
                triangleOrigin,
                triangleRotation,
                triangleLocalPoint),
            new FixedPointAnchor(
                prismOrigin,
                FixedQuaternion.FromAxisAngle(
                    Vector3d.Up,
                    -prismRotation),
                prismLocalPoint),
            normal,
            best.Depth,
            best.DepthIsClamped);
        return true;
    }

    private static bool TryKeepTrianglePrismAxis(
        WideAxis3 axis,
        Vector3d triangleOrigin,
        RationalBasis triangleBasis,
        ReadOnlySpan<Vector3d> trianglePoints,
        Vector3d prismOrigin,
        Fixed64 prismRotation,
        ReadOnlySpan<Vector2d> prismLocalOffsets,
        Fixed64 prismHalfThickness,
        ref PolytopePenetration best)
    {
        if (axis.IsZero)
            return true;

        GetHullProjectionInterval(
            trianglePoints,
            triangleBasis,
            axis,
            Signed192.One,
            out Signed576 triangleMinimum,
            out Signed576 triangleMaximum);
        Signed576 originProjection = WideArithmetic.MultiplySigned576(
            GetDifferenceProjection(
                prismOrigin,
                triangleOrigin,
                axis),
            Signed192.One);
        Signed576 prismMinimum = default;
        Signed576 prismMaximum = default;
        bool hasPlanarProjection = false;
        for (int index = 0; index < prismLocalOffsets.Length; index++)
        {
            Signed576 projection = GetRotatedPlanarOffsetProjection(
                prismLocalOffsets[index],
                prismRotation,
                axis);
            if (!hasPlanarProjection)
            {
                prismMinimum = projection;
                prismMaximum = projection;
                hasPlanarProjection = true;
            }
            else
            {
                if (CompareSigned(projection, prismMinimum) < 0)
                    prismMinimum = projection;
                if (CompareSigned(projection, prismMaximum) > 0)
                    prismMaximum = projection;
            }
        }

        Signed576 verticalRadius = WideArithmetic.MultiplySigned576(
            WideArithmetic.MultiplySigned576(
                Signed576.ExtendValue(
                    GetMagnitude(axis.Y)),
                Signed192.Raw(prismHalfThickness)),
            Signed192.One);
        prismMinimum = WideArithmetic.MultiplySigned576(
            WideArithmetic.SubtractSigned576(
                WideArithmetic.AddSigned576(
                    originProjection,
                    prismMinimum),
                verticalRadius),
            triangleBasis.Denominator);
        prismMaximum = WideArithmetic.MultiplySigned576(
            WideArithmetic.AddSigned576(
                WideArithmetic.AddSigned576(
                    originProjection,
                    prismMaximum),
                verticalRadius),
            triangleBasis.Denominator);
        Signed576 positiveOverlap = WideArithmetic.SubtractSigned576(
            triangleMaximum,
            prismMinimum);
        Signed576 negativeOverlap = WideArithmetic.SubtractSigned576(
            prismMaximum,
            triangleMinimum);
        if (positiveOverlap.Sign < 0 || negativeOverlap.Sign < 0)
            return false;

        bool negate =
            CompareSigned(positiveOverlap, negativeOverlap) > 0;
        Signed576 overlap = negate
            ? negativeOverlap
            : positiveOverlap;
        Signed320 commonDenominator = WideArithmetic.MultiplySigned192(
            triangleBasis.Denominator,
            Signed192.One);
        GetPolytopeDepth(
            overlap,
            axis,
            commonDenominator,
            out Fixed64 depth,
            out bool depthIsClamped,
            out Signed576 squaredAxisLength);
        if (ShouldReplacePolytope(
            overlap,
            squaredAxisLength,
            commonDenominator,
            depth,
            best))
        {
            best = new PolytopePenetration(
                axis,
                negate,
                depth,
                depthIsClamped,
                overlap,
                squaredAxisLength,
                commonDenominator);
        }

        return true;
    }
}
