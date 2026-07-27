//=======================================================================
// WideOrientedBox.ConvexHull.cs
//=======================================================================
// MIT License, Copyright (c) 2024–present David Oravsky (mrdav30)
// See LICENSE file in the project root for full license information.
//=======================================================================

using System;

namespace FixedMathSharp.Geometry;

/// <content>
/// Convex hull contact generation logic for <see cref="WideOrientedBox"/>,
/// using separating axis tests against box axes, hull face normals, and edge cross products.
/// </content>
internal static partial class WideOrientedBox
{
    internal static bool TryGetConvexHullContact(
        Vector3d boxCenter,
        FixedQuaternion boxOrientation,
        Vector3d boxHalfExtents,
        Vector3d hullOrigin,
        FixedQuaternion hullOrientation,
        ReadOnlySpan<Vector3d> hullLocalOffsets,
        ReadOnlySpan<int> triangleVertexIndices,
        ReadOnlySpan<int> edgeVertexPairs,
        out FixedContactAnchors contact)
    {
        RationalBasis boxBasis = new(boxOrientation);
        RationalBasis hullBasis = new(hullOrientation);
        Span<WideAxis3> boxAxes = stackalloc WideAxis3[3]
        {
            GetBasisAxis(boxBasis, 0),
            GetBasisAxis(boxBasis, 1),
            GetBasisAxis(boxBasis, 2),
        };
        var best = default(PointSpanPenetration);
        for (int index = 0; index < boxAxes.Length; index++)
        {
            if (!TryKeepPointSpanAxis(
                    boxAxes[index],
                    boxCenter,
                    boxHalfExtents,
                    boxBasis,
                    hullOrigin,
                    hullBasis,
                    hullLocalOffsets,
                    ref best))
            {
                contact = default;
                return false;
            }
        }

        for (int index = 0;
            index < triangleVertexIndices.Length;
            index += 3)
        {
            Vector3d first = hullLocalOffsets[
                triangleVertexIndices[index]];
            Vector3d second = hullLocalOffsets[
                triangleVertexIndices[index + 1]];
            Vector3d third = hullLocalOffsets[
                triangleVertexIndices[index + 2]];
            WideGeometry.GetDifferenceCrossProduct3D(
                second.X,
                first.X,
                second.Y,
                first.Y,
                second.Z,
                first.Z,
                third.X,
                first.X,
                third.Y,
                first.Y,
                third.Z,
                first.Z,
                out Signed192 normalX,
                out Signed192 normalY,
                out Signed192 normalZ);
            WideAxis3 axis = TransformLocalAxis(
                hullBasis,
                normalX,
                normalY,
                normalZ);
            if (!TryKeepPointSpanAxis(
                    axis,
                    boxCenter,
                    boxHalfExtents,
                    boxBasis,
                    hullOrigin,
                    hullBasis,
                    hullLocalOffsets,
                    ref best))
            {
                contact = default;
                return false;
            }
        }

        for (int index = 0; index < edgeVertexPairs.Length; index += 2)
        {
            Vector3d start = hullLocalOffsets[edgeVertexPairs[index]];
            Vector3d end = hullLocalOffsets[edgeVertexPairs[index + 1]];
            WideAxis3 edge = TransformLocalAxis(
                hullBasis,
                WideArithmetic.SubtractSigned192(
                    Signed192.Raw(end.X),
                    Signed192.Raw(start.X)),
                WideArithmetic.SubtractSigned192(
                    Signed192.Raw(end.Y),
                    Signed192.Raw(start.Y)),
                WideArithmetic.SubtractSigned192(
                    Signed192.Raw(end.Z),
                    Signed192.Raw(start.Z)));
            for (int axisIndex = 0;
                axisIndex < boxAxes.Length;
                axisIndex++)
            {
                if (!TryKeepPointSpanAxis(
                        Cross(boxAxes[axisIndex], edge),
                        boxCenter,
                        boxHalfExtents,
                        boxBasis,
                        hullOrigin,
                        hullBasis,
                        hullLocalOffsets,
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
        Vector3d hullLocalPoint = GetPointSpanSupportLocalPoint(
            hullBasis,
            hullLocalOffsets,
            orientedAxis);

        Vector3d boxLocalPoint = GetLocalSupportPoint(
            normal,
            boxOrientation,
            boxHalfExtents);
        GetPointSpanDepth(
            best.ExactOverlap,
            best.ExactSquaredAxisLength,
            best.ExactCommonDenominator,
            out Fixed64 depth,
            out bool depthIsClamped);
        contact = new FixedContactAnchors(
            new FixedPointAnchor(
                boxCenter,
                boxOrientation,
                boxLocalPoint),
            new FixedPointAnchor(
                hullOrigin,
                hullOrientation,
                hullLocalPoint),
            normal,
            depth,
            depthIsClamped);
        return true;
    }

    private static bool TryKeepPointSpanAxis(
        WideAxis3 axis,
        Vector3d boxCenter,
        Vector3d boxHalfExtents,
        RationalBasis boxBasis,
        Vector3d hullOrigin,
        RationalBasis hullBasis,
        ReadOnlySpan<Vector3d> hullLocalOffsets,
        ref PointSpanPenetration best)
    {
        if (axis.IsZero)
            return true;

        Signed576 boxRadius = GetBoxProjectionRadiusNumerator(
            axis,
            boxHalfExtents,
            boxBasis);
        Signed576 minimum = GetTransformedOffsetProjection(
            hullLocalOffsets[0],
            hullBasis,
            axis);
        Signed576 maximum = minimum;
        for (int index = 1; index < hullLocalOffsets.Length; index++)
        {
            KeepProjection(
                GetTransformedOffsetProjection(
                    hullLocalOffsets[index],
                    hullBasis,
                    axis),
                ref minimum,
                ref maximum);
        }

        Signed576 originProjection = GetDifferenceProjection(
            hullOrigin,
            boxCenter,
            axis);
        Signed576 originNumerator = WideArithmetic.MultiplySigned576(
            originProjection,
            hullBasis.Denominator);
        minimum = WideArithmetic.MultiplySigned576(
            WideArithmetic.AddSigned576(originNumerator, minimum),
            boxBasis.Denominator);
        maximum = WideArithmetic.MultiplySigned576(
            WideArithmetic.AddSigned576(originNumerator, maximum),
            boxBasis.Denominator);
        boxRadius = WideArithmetic.MultiplySigned576(
            boxRadius,
            hullBasis.Denominator);
        Signed576 pushBoxNegative = WideArithmetic.SubtractSigned576(
            boxRadius,
            minimum);
        Signed576 pushBoxPositive = WideArithmetic.AddSigned576(
            maximum,
            boxRadius);
        if (pushBoxNegative.Sign < 0 || pushBoxPositive.Sign < 0)
            return false;

        bool negate =
            CompareSigned(pushBoxPositive, pushBoxNegative) < 0;
        Signed576 overlap = negate
            ? pushBoxPositive
            : pushBoxNegative;
        Signed320 commonDenominator = WideArithmetic.MultiplySigned192(
            boxBasis.Denominator,
            hullBasis.Denominator);
        Signed576 squaredAxisLength = GetSquaredLength(axis);
        if (ShouldReplacePointSpan(
            overlap,
            squaredAxisLength,
            commonDenominator,
            best))
        {
            best = new PointSpanPenetration(
                axis,
                negate,
                overlap,
                squaredAxisLength,
                commonDenominator);
        }
        return true;
    }

    private static Signed576 GetTransformedOffsetProjection(
        Vector3d localOffset,
        RationalBasis basis,
        WideAxis3 axis) =>
        WideArithmetic.AddSigned576(
            WideArithmetic.AddSigned576(
                WideArithmetic.MultiplySigned576(
                    GetBasisProjection(
                        axis,
                        basis.Xx,
                        basis.Xy,
                        basis.Xz),
                    Signed192.Raw(localOffset.X)),
                WideArithmetic.MultiplySigned576(
                    GetBasisProjection(
                        axis,
                        basis.Yx,
                        basis.Yy,
                        basis.Yz),
                    Signed192.Raw(localOffset.Y))),
            WideArithmetic.MultiplySigned576(
                GetBasisProjection(
                    axis,
                    basis.Zx,
                    basis.Zy,
                    basis.Zz),
                Signed192.Raw(localOffset.Z)));

    private static WideAxis3 TransformLocalAxis(
        RationalBasis basis,
        Signed192 localX,
        Signed192 localY,
        Signed192 localZ) =>
        new(
            GetTransformedAxisComponent(
                localX,
                localY,
                localZ,
                basis.Xx,
                basis.Yx,
                basis.Zx),
            GetTransformedAxisComponent(
                localX,
                localY,
                localZ,
                basis.Xy,
                basis.Yy,
                basis.Zy),
            GetTransformedAxisComponent(
                localX,
                localY,
                localZ,
                basis.Xz,
                basis.Yz,
                basis.Zz));

    private static Signed320 GetTransformedAxisComponent(
        Signed192 localX,
        Signed192 localY,
        Signed192 localZ,
        Signed192 basisX,
        Signed192 basisY,
        Signed192 basisZ) =>
        WideArithmetic.AddSigned320(
            WideArithmetic.AddSigned320(
                WideArithmetic.MultiplySigned192(localX, basisX),
                WideArithmetic.MultiplySigned192(localY, basisY)),
            WideArithmetic.MultiplySigned192(localZ, basisZ));

    private static void GetPointSpanDepth(
        Signed576 overlap,
        Signed576 squaredAxisLength,
        Signed320 commonDenominator,
        out Fixed64 depth,
        out bool depthIsClamped)
    {
        // Every nonzero hull SAT axis contains a normalized quaternion-basis
        // numerator; authored face and edge axes only add scale. This proves
        // the half-Q32 minimum required by the one-step rounding contract.
        depth = WideArithmetic.GetRoundedNonNegativeNormalizedDepth(
            overlap,
            squaredAxisLength,
            commonDenominator,
            out depthIsClamped);
    }

    private static bool ShouldReplacePointSpan(
        Signed576 overlap,
        Signed576 squaredAxisLength,
        Signed320 commonDenominator,
        in PointSpanPenetration best)
    {
        if (!best.HasValue)
            return true;
        return WideArithmetic.CompareNonNegativeNormalizedDepths(
            overlap,
            squaredAxisLength,
            commonDenominator,
            best.ExactOverlap,
            best.ExactSquaredAxisLength,
            best.ExactCommonDenominator) < 0;
    }

    private static Vector3d GetPointSpanSupportLocalPoint(
        RationalBasis hullBasis,
        ReadOnlySpan<Vector3d> hullLocalOffsets,
        WideAxis3 boxToHullAxis)
    {
        Vector3d support = hullLocalOffsets[0];
        Signed576 minimum = GetTransformedOffsetProjection(
            support,
            hullBasis,
            boxToHullAxis);
        for (int index = 1; index < hullLocalOffsets.Length; index++)
        {
            Vector3d localOffset = hullLocalOffsets[index];
            Signed576 projection = GetTransformedOffsetProjection(
                localOffset,
                hullBasis,
                boxToHullAxis);
            if (CompareSigned(projection, minimum) < 0)
            {
                minimum = projection;
                support = localOffset;
            }
        }

        return support;
    }
}
