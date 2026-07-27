//=======================================================================
// WideOrientedBox.TriangleCapsuleSlab.cs
//=======================================================================
// MIT License, Copyright (c) 2024-present David Oravsky (mrdav30)
// See LICENSE file in the project root for full license information.
//=======================================================================

using System;

namespace FixedMathSharp.Geometry;

/// <content>
/// Contains logic for detecting and resolving contact between a triangle and a capsule-shaped slab,
/// using wide (high-precision) arithmetic for robust deterministic collision resolution.
/// </content>
internal static partial class WideOrientedBox
{
    internal static bool TryGetTriangleCapsuleSlabContact(
        Vector3d triangleOrigin,
        FixedQuaternion triangleRotation,
        FixedTriangle triangle,
        Vector3d slabCenter,
        Fixed64 capsuleFrameRotation,
        Vector2d localCapsuleAxisDirection,
        Fixed64 capsuleAxisLength,
        Fixed64 capsuleRadius,
        Fixed64 slabHalfThickness,
        out FixedContactAnchors contact)
    {
        if (triangle.IsDegenerate)
        {
            contact = default;
            return false;
        }

        _ = Vector2d.TryRotate(
            localCapsuleAxisDirection,
            capsuleFrameRotation,
            out Vector2d capsuleAxisDirection);
        RationalBasis triangleBasis = new(triangleRotation);
        Signed320 commonDenominatorWide =
            WideArithmetic.MultiplySigned192(
                triangleBasis.Denominator,
                Signed192.Raw(Fixed64.Two));
        Signed192 commonDenominator = new(
            commonDenominatorWide.Word2,
            commonDenominatorWide.Word1,
            commonDenominatorWide.Word0);
        Span<Vector3d> trianglePoints = stackalloc Vector3d[3]
        {
            triangle.A,
            triangle.B,
            triangle.C,
        };
        Span<WideAxis3> triangleEdges = stackalloc WideAxis3[3]
        {
            GetHullEdge(triangleBasis, triangle.A, triangle.B),
            GetHullEdge(triangleBasis, triangle.B, triangle.C),
            GetHullEdge(triangleBasis, triangle.C, triangle.A),
        };
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
        WideAxis3 triangleNormal = TransformLocalAxis(
            triangleBasis,
            localNormalX,
            localNormalY,
            localNormalZ);
        WideAxis3 up = new(default, Signed320.One, default);
        WideAxis3 capsuleAxis = new(
            Signed320.ExtendValue(
                Signed192.Raw(capsuleAxisDirection.X)),
            default,
            Signed320.ExtendValue(
                Signed192.Raw(capsuleAxisDirection.Y)));
        WideAxis3 capsuleNormal = new(
            Signed320.ExtendValue(
                Signed192.Raw(capsuleAxisDirection.Y)),
            default,
            Signed320.ExtendValue(
                WideArithmetic.SubtractSigned192(
                    default,
                    Signed192.Raw(capsuleAxisDirection.X))));
        var best = default(RadialPenetration);
        if (!TryKeepTriangleCapsuleSlabAxis(
                up,
                triangleOrigin,
                triangleBasis,
                trianglePoints,
                commonDenominator,
                slabCenter,
                capsuleAxisDirection,
                capsuleAxisLength,
                capsuleRadius,
                slabHalfThickness,
                ref best)
            || !TryKeepTriangleCapsuleSlabAxis(
                triangleNormal,
                triangleOrigin,
                triangleBasis,
                trianglePoints,
                commonDenominator,
                slabCenter,
                capsuleAxisDirection,
                capsuleAxisLength,
                capsuleRadius,
                slabHalfThickness,
                ref best)
            || !TryKeepTriangleCapsuleSlabAxis(
                capsuleAxis,
                triangleOrigin,
                triangleBasis,
                trianglePoints,
                commonDenominator,
                slabCenter,
                capsuleAxisDirection,
                capsuleAxisLength,
                capsuleRadius,
                slabHalfThickness,
                ref best)
            || !TryKeepTriangleCapsuleSlabAxis(
                capsuleNormal,
                triangleOrigin,
                triangleBasis,
                trianglePoints,
                commonDenominator,
                slabCenter,
                capsuleAxisDirection,
                capsuleAxisLength,
                capsuleRadius,
                slabHalfThickness,
                ref best))
        {
            contact = default;
            return false;
        }

        for (int edgeIndex = 0;
            edgeIndex < triangleEdges.Length;
            edgeIndex++)
        {
            WideAxis3 edge = triangleEdges[edgeIndex];
            if (!TryKeepTriangleCapsuleSlabAxis(
                    CrossWithUp(edge),
                    triangleOrigin,
                    triangleBasis,
                    trianglePoints,
                    commonDenominator,
                    slabCenter,
                    capsuleAxisDirection,
                    capsuleAxisLength,
                    capsuleRadius,
                    slabHalfThickness,
                    ref best)
                || !TryKeepTriangleCapsuleSlabAxis(
                    Cross(edge, capsuleAxis),
                    triangleOrigin,
                    triangleBasis,
                    trianglePoints,
                    commonDenominator,
                    slabCenter,
                    capsuleAxisDirection,
                    capsuleAxisLength,
                    capsuleRadius,
                    slabHalfThickness,
                    ref best))
            {
                contact = default;
                return false;
            }
        }

        int endpointStart = capsuleAxisLength == Fixed64.Zero ? 1 : -1;
        for (int vertexIndex = 0;
            vertexIndex < trianglePoints.Length;
            vertexIndex++)
        {
            for (int endpointSign = endpointStart;
                endpointSign <= 1;
                endpointSign += 2)
            {
                WideAxis3 endpointAxis =
                    GetTriangleVertexToCapsuleEndpointAxis(
                        triangleOrigin,
                        slabCenter,
                        triangleBasis,
                        trianglePoints[vertexIndex],
                        capsuleAxisDirection,
                        capsuleAxisLength,
                        endpointSign);
                if (!TryKeepTriangleCapsuleSlabAxis(
                        endpointAxis,
                        triangleOrigin,
                        triangleBasis,
                        trianglePoints,
                        commonDenominator,
                        slabCenter,
                        capsuleAxisDirection,
                        capsuleAxisLength,
                        capsuleRadius,
                        slabHalfThickness,
                        ref best))
                {
                    contact = default;
                    return false;
                }
            }
        }

        WideAxis3 orientedAxis =
            best.Negate ? -best.Axis : best.Axis;
        Vector3d normal = WideGeometry.GetNormalized(
            Signed576.ExtendValue(orientedAxis.X),
            Signed576.ExtendValue(orientedAxis.Y),
            Signed576.ExtendValue(orientedAxis.Z));
        FixedPointAnchor slabAnchor = GetCapsuleSlabSupportAnchor(
            slabCenter,
            capsuleFrameRotation,
            localCapsuleAxisDirection,
            orientedAxis,
            capsuleAxisLength,
            capsuleRadius,
            slabHalfThickness);
        FixedPointAnchor triangleAnchor = triangle.GetClosestPointAnchor(
            triangleOrigin,
            triangleRotation,
            slabAnchor);
        contact = new FixedContactAnchors(
            triangleAnchor,
            slabAnchor,
            normal,
            best.Depth,
            best.DepthIsClamped);
        return true;
    }

    private static bool TryKeepTriangleCapsuleSlabAxis(
        WideAxis3 axis,
        Vector3d triangleOrigin,
        RationalBasis triangleBasis,
        ReadOnlySpan<Vector3d> trianglePoints,
        Signed192 commonDenominator,
        Vector3d slabCenter,
        Vector2d capsuleAxisDirection,
        Fixed64 capsuleAxisLength,
        Fixed64 capsuleRadius,
        Fixed64 slabHalfThickness,
        ref RadialPenetration best)
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
        Signed576 centerProjection = WideArithmetic.MultiplySigned576(
            WideArithmetic.MultiplySigned576(
                GetDifferenceProjection(
                    slabCenter,
                    triangleOrigin,
                    axis),
                Signed192.One),
            triangleBasis.Denominator);
        Signed576 verticalRadius = WideArithmetic.MultiplySigned576(
            WideArithmetic.MultiplySigned576(
                WideArithmetic.MultiplySigned576(
                    Signed576.ExtendValue(
                        GetMagnitude(axis.Y)),
                    Signed192.Raw(slabHalfThickness)),
                Signed192.One),
            triangleBasis.Denominator);
        Signed576 axialRadius = WideArithmetic.MultiplySigned576(
            WideArithmetic.MultiplySigned576(
                GetMagnitude(
                    GetPlanarAxisProjection(
                        axis,
                        capsuleAxisDirection)),
                Signed192.Raw(capsuleAxisLength)),
            triangleBasis.Denominator);
        Signed576 doubledCenter = WideArithmetic.AddSigned576(
            centerProjection,
            centerProjection);
        Signed576 doubledVertical = WideArithmetic.AddSigned576(
            verticalRadius,
            verticalRadius);
        Signed576 positiveBase = WideArithmetic.AddSigned576(
            WideArithmetic.SubtractSigned576(
                WideArithmetic.AddSigned576(
                    triangleMaximum,
                    triangleMaximum),
                doubledCenter),
            WideArithmetic.AddSigned576(
                doubledVertical,
                axialRadius));
        Signed576 negativeBase = WideArithmetic.AddSigned576(
            WideArithmetic.SubtractSigned576(
                doubledCenter,
                WideArithmetic.AddSigned576(
                    triangleMinimum,
                    triangleMinimum)),
            WideArithmetic.AddSigned576(
                doubledVertical,
                axialRadius));
        Signed576 planarSquared = WideArithmetic.AddSigned576(
            WideArithmetic.MultiplySigned320(axis.X, axis.X),
            WideArithmetic.MultiplySigned320(axis.Z, axis.Z));
        Signed320 radiusSquared = WideArithmetic.MultiplySigned192(
            Signed192.Raw(capsuleRadius),
            Signed192.Raw(capsuleRadius));
        Signed832 radialSquared =
            WideArithmetic.MultiplySigned576ToSigned832(
                Signed576.ExtendValue(radiusSquared),
                planarSquared);
        if (!DoesRadialOverlap(
                positiveBase,
                radialSquared,
                commonDenominator)
            || !DoesRadialOverlap(
                negativeBase,
                radialSquared,
                commonDenominator))
        {
            return false;
        }

        Signed576 squaredAxisLength = GetSquaredAxisLength(axis);
        GetRadialDepth(
            positiveBase,
            radialSquared,
            squaredAxisLength,
            commonDenominator,
            out Fixed64 positiveDepth,
            out bool positiveClamped);
        GetRadialDepth(
            negativeBase,
            radialSquared,
            squaredAxisLength,
            commonDenominator,
            out Fixed64 negativeDepth,
            out bool negativeClamped);
        bool negate =
            negativeDepth < positiveDepth
            || negativeDepth == positiveDepth
            && WideArithmetic.SubtractSigned576(
                negativeBase,
                positiveBase).Sign < 0;
        Fixed64 depth = negate
            ? negativeDepth
            : positiveDepth;
        bool depthIsClamped = negate
            ? negativeClamped
            : positiveClamped;
        Signed576 rational = negate
            ? negativeBase
            : positiveBase;
        if (!best.HasValue
            || depth < best.Depth
            || (depth == best.Depth
                && WideArithmetic.CompareRadialProjectionDepths(
                    rational,
                    radialSquared,
                    ToSigned576(Signed192.Signed(1L)),
                    squaredAxisLength,
                    best.Rational,
                    best.RadialNumerator,
                    best.RadialDenominator,
                    best.AxisSquared,
                    commonDenominator) < 0))
        {
            best = new RadialPenetration(
                axis,
                negate,
                depth,
                depthIsClamped,
                rational,
                radialSquared,
                ToSigned576(Signed192.Signed(1L)),
                squaredAxisLength);
        }
        return true;
    }

    private static bool DoesRadialOverlap(
        Signed576 rational,
        Signed832 radialSquared,
        Signed192 commonDenominator) =>
        rational.Sign >= 0
        || IsRadicalAtLeastRatio(
            radialSquared,
            WideArithmetic.SubtractSigned576(
                default,
                rational),
            commonDenominator);

    private static WideAxis3 GetTriangleVertexToCapsuleEndpointAxis(
        Vector3d triangleOrigin,
        Vector3d capsuleCenter,
        RationalBasis basis,
        Vector3d localVertex,
        Vector2d capsuleAxisDirection,
        Fixed64 capsuleAxisLength,
        int endpointSign)
    {
        Signed320 vertexX = GetLocalOffsetNumerator(
            basis.Xx,
            basis.Yx,
            basis.Zx,
            localVertex);
        Signed320 vertexZ = GetLocalOffsetNumerator(
            basis.Xz,
            basis.Yz,
            basis.Zz,
            localVertex);
        Signed192 twiceScale = Signed192.Raw(Fixed64.Two);
        Signed576 centerX = WideArithmetic.MultiplySigned576(
            Signed576.ExtendValue(
                WideArithmetic.MultiplySigned192(
                    WideArithmetic.SubtractSigned192(
                        Signed192.Raw(capsuleCenter.X),
                        Signed192.Raw(triangleOrigin.X)),
                    basis.Denominator)),
            twiceScale);
        Signed576 centerZ = WideArithmetic.MultiplySigned576(
            Signed576.ExtendValue(
                WideArithmetic.MultiplySigned192(
                    WideArithmetic.SubtractSigned192(
                        Signed192.Raw(capsuleCenter.Z),
                        Signed192.Raw(triangleOrigin.Z)),
                    basis.Denominator)),
            twiceScale);
        Signed576 endpointX = WideArithmetic.MultiplySigned576(
            WideArithmetic.MultiplySigned576(
                Signed576.ExtendValue(
                    Signed320.ExtendValue(
                        Signed192.Raw(capsuleAxisDirection.X))),
                Signed192.Signed(
                    endpointSign
                    * capsuleAxisLength.m_rawValue)),
            basis.Denominator);
        Signed576 endpointZ = WideArithmetic.MultiplySigned576(
            WideArithmetic.MultiplySigned576(
                Signed576.ExtendValue(
                    Signed320.ExtendValue(
                        Signed192.Raw(capsuleAxisDirection.Y))),
                Signed192.Signed(
                    endpointSign
                    * capsuleAxisLength.m_rawValue)),
            basis.Denominator);
        Signed576 vertexXWide = WideArithmetic.MultiplySigned576(
            Signed576.ExtendValue(vertexX),
            twiceScale);
        Signed576 vertexZWide = WideArithmetic.MultiplySigned576(
            Signed576.ExtendValue(vertexZ),
            twiceScale);
        // A full-domain center difference multiplied by a normalized quaternion
        // basis and the Q32.32 factor for two uses at most 165 signed bits.
        return new WideAxis3(
            Signed320.NarrowValue(
                WideArithmetic.SubtractSigned576(
                    WideArithmetic.AddSigned576(
                        centerX,
                        endpointX),
                    vertexXWide)),
            default,
            Signed320.NarrowValue(
                WideArithmetic.SubtractSigned576(
                    WideArithmetic.AddSigned576(
                        centerZ,
                        endpointZ),
                    vertexZWide)));
    }
}
