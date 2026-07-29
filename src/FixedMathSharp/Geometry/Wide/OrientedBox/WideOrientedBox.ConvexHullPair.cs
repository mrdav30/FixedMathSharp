//=======================================================================
// WideOrientedBox.ConvexHullPair.cs
//=======================================================================
// MIT License, Copyright (c) 2024–present David Oravsky (mrdav30)
// See LICENSE file in the project root for full license information.
//=======================================================================

using System;

namespace FixedMathSharp.Geometry;

/// <content>
/// Provides high-precision convex hull containment and separating-axis
/// tests for oriented boxes against arbitrary convex hulls.
/// </content>
internal static partial class WideOrientedBox
{
    internal static bool ContainsConvexHullPoint(
        Vector3d hullOrigin,
        FixedQuaternion hullRotation,
        ReadOnlySpan<Vector3d> hullLocalPoints,
        ReadOnlySpan<int> triangleVertexIndices,
        Vector3d hullInteriorLocalPoint,
        FixedPointAnchor point)
    {
        if (!point.Rotation.IsNormalized())
            return false;

        WideRationalBasis3d hullBasis = new(hullRotation);
        WideRationalBasis3d pointBasis = new(point.Rotation);
        for (int index = 0; index < triangleVertexIndices.Length; index += 3)
        {
            Vector3d first = hullLocalPoints[triangleVertexIndices[index]];
            Vector3d second = hullLocalPoints[triangleVertexIndices[index + 1]];
            Vector3d third = hullLocalPoints[triangleVertexIndices[index + 2]];
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
            if (axis.IsZero)
                continue;

            Signed576 faceProjection = GetTransformedOffsetProjection(
                first,
                hullBasis,
                axis);
            Signed576 interiorDelta = WideArithmetic.SubtractSigned576(
                GetTransformedOffsetProjection(
                    hullInteriorLocalPoint,
                    hullBasis,
                    axis),
                faceProjection);
            if (interiorDelta.Sign == 0)
                continue;

            Signed576 originProjection = GetDifferenceProjection(
                point.Origin,
                hullOrigin,
                axis);
            Signed576 pointProjection = WideArithmetic.MultiplySigned576(
                WideArithmetic.AddSigned576(
                    WideArithmetic.MultiplySigned576(
                        originProjection,
                        pointBasis.Denominator),
                    GetTransformedOffsetProjection(
                        point.LocalPoint,
                        pointBasis,
                        axis)),
                hullBasis.Denominator);
            Signed576 faceProjectionCommon = WideArithmetic.MultiplySigned576(
                faceProjection,
                pointBasis.Denominator);
            Signed576 pointDelta = WideArithmetic.SubtractSigned576(
                pointProjection,
                faceProjectionCommon);
            if ((pointDelta.Sign != 0)
                & (pointDelta.Sign != interiorDelta.Sign))
            {
                return false;
            }
        }

        return true;
    }

    internal static bool TryGetConvexHullContact(
        Vector3d firstOrigin,
        FixedQuaternion firstRotation,
        ReadOnlySpan<Vector3d> firstLocalPoints,
        ReadOnlySpan<int> firstTriangleVertexIndices,
        ReadOnlySpan<int> firstEdgeVertexPairs,
        Vector3d secondOrigin,
        FixedQuaternion secondRotation,
        ReadOnlySpan<Vector3d> secondLocalPoints,
        ReadOnlySpan<int> secondTriangleVertexIndices,
        ReadOnlySpan<int> secondEdgeVertexPairs,
        out FixedContactAnchors contact)
    {
        WideRationalBasis3d firstBasis = new(firstRotation);
        WideRationalBasis3d secondBasis = new(secondRotation);
        var best = default(PointSpanPenetration);

        if (!TryKeepHullFaceAxes(
                firstLocalPoints,
                firstTriangleVertexIndices,
                firstBasis,
                firstOrigin,
                firstBasis,
                firstLocalPoints,
                secondOrigin,
                secondBasis,
                secondLocalPoints,
                ref best)
            || !TryKeepHullFaceAxes(
                secondLocalPoints,
                secondTriangleVertexIndices,
                secondBasis,
                firstOrigin,
                firstBasis,
                firstLocalPoints,
                secondOrigin,
                secondBasis,
                secondLocalPoints,
                ref best))
        {
            contact = default;
            return false;
        }

        for (int firstIndex = 0;
            firstIndex < firstEdgeVertexPairs.Length;
            firstIndex += 2)
        {
            WideAxis3 firstEdge = GetHullEdge(
                firstBasis,
                firstLocalPoints[firstEdgeVertexPairs[firstIndex]],
                firstLocalPoints[firstEdgeVertexPairs[firstIndex + 1]]);
            for (int secondIndex = 0;
                secondIndex < secondEdgeVertexPairs.Length;
                secondIndex += 2)
            {
                WideAxis3 secondEdge = GetHullEdge(
                    secondBasis,
                    secondLocalPoints[secondEdgeVertexPairs[secondIndex]],
                    secondLocalPoints[secondEdgeVertexPairs[secondIndex + 1]]);
                if (!TryKeepHullPairAxis(
                        Cross(firstEdge, secondEdge),
                        firstOrigin,
                        firstBasis,
                        firstLocalPoints,
                        secondOrigin,
                        secondBasis,
                        secondLocalPoints,
                        ref best))
                {
                    contact = default;
                    return false;
                }
            }
        }

        if (!best.HasValue)
        {
            contact = default;
            return false;
        }

        WideAxis3 orientedAxis = best.Negate ? -best.Axis : best.Axis;
        Vector3d normal = WideNormalization.GetNormalized(
            Signed576.ExtendValue(orientedAxis.X),
            Signed576.ExtendValue(orientedAxis.Y),
            Signed576.ExtendValue(orientedAxis.Z));
        Vector3d firstLocalPoint = GetHullSupportLocalPoint(
            firstBasis,
            firstLocalPoints,
            orientedAxis,
            maximize: true);
        Vector3d secondLocalPoint = GetHullSupportLocalPoint(
            secondBasis,
            secondLocalPoints,
            orientedAxis,
            maximize: false);
        GetPointSpanDepth(
            best.ExactOverlap,
            best.ExactSquaredAxisLength,
            best.ExactCommonDenominator,
            out Fixed64 depth,
            out bool depthIsClamped);
        contact = new FixedContactAnchors(
            new FixedPointAnchor(
                firstOrigin,
                firstRotation,
                firstLocalPoint),
            new FixedPointAnchor(
                secondOrigin,
                secondRotation,
                secondLocalPoint),
            normal,
            depth,
            depthIsClamped);
        return true;
    }

    private static bool TryKeepHullFaceAxes(
        ReadOnlySpan<Vector3d> axisSourcePoints,
        ReadOnlySpan<int> axisSourceTriangles,
        WideRationalBasis3d axisBasis,
        Vector3d firstOrigin,
        WideRationalBasis3d firstBasis,
        ReadOnlySpan<Vector3d> firstPoints,
        Vector3d secondOrigin,
        WideRationalBasis3d secondBasis,
        ReadOnlySpan<Vector3d> secondPoints,
        ref PointSpanPenetration best)
    {
        for (int index = 0; index < axisSourceTriangles.Length; index += 3)
        {
            Vector3d first = axisSourcePoints[axisSourceTriangles[index]];
            Vector3d second = axisSourcePoints[axisSourceTriangles[index + 1]];
            Vector3d third = axisSourcePoints[axisSourceTriangles[index + 2]];
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
            if (!TryKeepHullPairAxis(
                    TransformLocalAxis(
                        axisBasis,
                        normalX,
                        normalY,
                        normalZ),
                    firstOrigin,
                    firstBasis,
                    firstPoints,
                    secondOrigin,
                    secondBasis,
                    secondPoints,
                    ref best))
            {
                return false;
            }
        }

        return true;
    }

    private static bool TryKeepHullPairAxis(
        WideAxis3 axis,
        Vector3d firstOrigin,
        WideRationalBasis3d firstBasis,
        ReadOnlySpan<Vector3d> firstPoints,
        Vector3d secondOrigin,
        WideRationalBasis3d secondBasis,
        ReadOnlySpan<Vector3d> secondPoints,
        ref PointSpanPenetration best)
    {
        if (axis.IsZero)
            return true;

        GetHullProjectionInterval(
            firstPoints,
            firstBasis,
            axis,
            secondBasis.Denominator,
            out Signed576 firstMinimum,
            out Signed576 firstMaximum);
        GetHullProjectionInterval(
            secondPoints,
            secondBasis,
            axis,
            firstBasis.Denominator,
            out Signed576 secondMinimum,
            out Signed576 secondMaximum);
        Signed576 originProjection = GetDifferenceProjection(
            secondOrigin,
            firstOrigin,
            axis);
        Signed576 originCommon = WideArithmetic.MultiplySigned576(
            WideArithmetic.MultiplySigned576(
                originProjection,
                secondBasis.Denominator),
            firstBasis.Denominator);
        secondMinimum = WideArithmetic.AddSigned576(
            secondMinimum,
            originCommon);
        secondMaximum = WideArithmetic.AddSigned576(
            secondMaximum,
            originCommon);

        Signed576 pushFirstNegative = WideArithmetic.SubtractSigned576(
            firstMaximum,
            secondMinimum);
        Signed576 pushFirstPositive = WideArithmetic.SubtractSigned576(
            secondMaximum,
            firstMinimum);
        if (pushFirstNegative.Sign < 0 || pushFirstPositive.Sign < 0)
            return false;

        bool negate = CompareSigned(
            pushFirstPositive,
            pushFirstNegative) < 0;
        Signed576 overlap = negate
            ? pushFirstPositive
            : pushFirstNegative;
        Signed320 commonDenominator = WideArithmetic.MultiplySigned192(
            firstBasis.Denominator,
            secondBasis.Denominator);
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

    private static void GetHullProjectionInterval(
        ReadOnlySpan<Vector3d> points,
        WideRationalBasis3d basis,
        WideAxis3 axis,
        Signed192 otherDenominator,
        out Signed576 minimum,
        out Signed576 maximum)
    {
        minimum = WideArithmetic.MultiplySigned576(
            GetTransformedOffsetProjection(points[0], basis, axis),
            otherDenominator);
        maximum = minimum;
        for (int index = 1; index < points.Length; index++)
        {
            KeepProjection(
                WideArithmetic.MultiplySigned576(
                    GetTransformedOffsetProjection(
                        points[index],
                        basis,
                        axis),
                    otherDenominator),
                ref minimum,
                ref maximum);
        }
    }

    private static WideAxis3 GetHullEdge(
        WideRationalBasis3d basis,
        Vector3d start,
        Vector3d end) =>
        TransformLocalAxis(
            basis,
            WideArithmetic.SubtractSigned192(Signed192.Raw(end.X), Signed192.Raw(start.X)),
            WideArithmetic.SubtractSigned192(Signed192.Raw(end.Y), Signed192.Raw(start.Y)),
            WideArithmetic.SubtractSigned192(Signed192.Raw(end.Z), Signed192.Raw(start.Z)));

    private static Vector3d GetHullSupportLocalPoint(
        WideRationalBasis3d basis,
        ReadOnlySpan<Vector3d> points,
        WideAxis3 axis,
        bool maximize)
    {
        Vector3d best = points[0];
        Signed576 bestProjection = GetTransformedOffsetProjection(
            best,
            basis,
            axis);
        for (int index = 1; index < points.Length; index++)
        {
            Vector3d candidate = points[index];
            Signed576 projection = GetTransformedOffsetProjection(
                candidate,
                basis,
                axis);
            int comparison = CompareSigned(projection, bestProjection);
            if (maximize ? comparison > 0 : comparison < 0)
            {
                best = candidate;
                bestProjection = projection;
            }
        }

        return best;
    }
}
