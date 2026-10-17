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
            WideAxis3 axis = WideRigidProjection.TransformLocalAxis(
                hullBasis,
                normalX,
                normalY,
                normalZ);
            if (axis.IsZero)
                continue;

            WideRigidProjection.GetLocalAxisProjections(
                axis,
                hullBasis,
                out Signed576 hullAxisX,
                out Signed576 hullAxisY,
                out Signed576 hullAxisZ);
            Signed576 faceProjection = WideRigidProjection.GetLocalOffsetProjection(
                first,
                hullAxisX,
                hullAxisY,
                hullAxisZ);
            Signed576 interiorDelta = WideArithmetic.SubtractSigned576(
                WideRigidProjection.GetLocalOffsetProjection(
                    hullInteriorLocalPoint,
                    hullAxisX,
                    hullAxisY,
                    hullAxisZ),
                faceProjection);
            if (interiorDelta.Sign == 0)
                continue;

            Signed576 originProjection = WideRigidProjection.GetWorldOriginDifferenceProjection(
                point.Origin,
                hullOrigin,
                axis);
            WideRigidProjection.GetLocalAxisProjections(
                axis,
                pointBasis,
                out Signed576 pointAxisX,
                out Signed576 pointAxisY,
                out Signed576 pointAxisZ);
            Signed576 pointProjection = WideArithmetic.MultiplySigned576(
                WideArithmetic.AddSigned576(
                    WideArithmetic.MultiplySigned576(
                        originProjection,
                        pointBasis.Denominator),
                    WideRigidProjection.GetLocalOffsetProjection(
                        point.LocalPoint,
                        pointAxisX,
                        pointAxisY,
                        pointAxisZ)),
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
        if (!TryGetConvexHullPenetration(
                firstOrigin, firstBasis, firstLocalPoints, firstTriangleVertexIndices, firstEdgeVertexPairs,
                secondOrigin, secondBasis, secondLocalPoints, secondTriangleVertexIndices, secondEdgeVertexPairs,
                out WidePointSpanPenetration best, skipParallelDirections: false))
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
        Fixed64 depth = best.GetRoundedDepth(out bool depthIsClamped);
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

    internal static bool TryGetConvexHullPenetration(
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
        out WidePointSpanPenetration best, bool skipParallelDirections = false)
    {
        WideRationalBasis3d firstBasis = new(firstRotation);
        WideRationalBasis3d secondBasis = new(secondRotation);
        return TryGetConvexHullPenetration(
            firstOrigin, firstBasis, firstLocalPoints, firstTriangleVertexIndices, firstEdgeVertexPairs,
            secondOrigin, secondBasis, secondLocalPoints, secondTriangleVertexIndices, secondEdgeVertexPairs,
            out best, skipParallelDirections);
    }

    private static bool TryGetConvexHullPenetration(
        Vector3d firstOrigin, in WideRationalBasis3d firstBasis,
        ReadOnlySpan<Vector3d> firstLocalPoints, ReadOnlySpan<int> firstTriangleVertexIndices, ReadOnlySpan<int> firstEdgeVertexPairs,
        Vector3d secondOrigin, in WideRationalBasis3d secondBasis,
        ReadOnlySpan<Vector3d> secondLocalPoints, ReadOnlySpan<int> secondTriangleVertexIndices, ReadOnlySpan<int> secondEdgeVertexPairs,
        out WidePointSpanPenetration best, bool skipParallelDirections)
    {
        best = default;

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
                ref best, skipParallelDirections)
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
                ref best, skipParallelDirections))
        {
            best = default;
            return false;
        }

        for (int firstIndex = 0;
            firstIndex < firstEdgeVertexPairs.Length;
            firstIndex += 2)
        {
            if (skipParallelDirections && !IsFirstHullEdgeDirection(firstLocalPoints, firstEdgeVertexPairs, firstIndex)) continue;
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
                        WideAxis3.Cross(firstEdge, secondEdge),
                        firstOrigin,
                        firstBasis,
                        firstLocalPoints,
                        secondOrigin,
                        secondBasis,
                        secondLocalPoints,
                        ref best))
                {
                    best = default;
                    return false;
                }
            }
        }

        return best.HasValue;
    }

    internal static int VisitConvexHullContacts<TState>(
        Vector3d firstOrigin, FixedQuaternion firstRotation,
        ReadOnlySpan<Vector3d> firstLocalPoints, ReadOnlySpan<int> firstTriangleVertexIndices, ReadOnlySpan<int> firstEdgeVertexPairs,
        Vector3d secondOrigin, FixedQuaternion secondRotation,
        ReadOnlySpan<Vector3d> secondLocalPoints, ReadOnlySpan<int> secondTriangleVertexIndices, ReadOnlySpan<int> secondEdgeVertexPairs,
        in WidePointSpanPenetration penetration, TState state, Action<TState, FixedContactAnchors> visitor)
    {
        System.Diagnostics.Debug.Assert(penetration.HasValue);
        WideAxis3 axis = penetration.Negate ? -penetration.Axis : penetration.Axis;
        Vector3d normal = WideNormalization.GetNormalized(Signed576.ExtendValue(axis.X),
            Signed576.ExtendValue(axis.Y), Signed576.ExtendValue(axis.Z));
        Fixed64 depth = penetration.GetRoundedDepth(out bool clamped);
        WideRationalBasis3d firstBasis = new(firstRotation), secondBasis = new(secondRotation);
        int count = 0;
        for (int side = 0; side < 2; side++)
        {
            bool reverse = side != 0;
            Vector3d sourceOrigin = reverse ? secondOrigin : firstOrigin;
            Vector3d targetOrigin = reverse ? firstOrigin : secondOrigin;
            FixedQuaternion sourceRotation = reverse ? secondRotation : firstRotation;
            FixedQuaternion targetRotation = reverse ? firstRotation : secondRotation;
            ReadOnlySpan<Vector3d> sourcePoints = reverse ? secondLocalPoints : firstLocalPoints;
            ReadOnlySpan<Vector3d> targetPoints = reverse ? firstLocalPoints : secondLocalPoints;
            ReadOnlySpan<int> targetTriangles = reverse ? firstTriangleVertexIndices : secondTriangleVertexIndices;
            ReadOnlySpan<int> sourceEdges = reverse ? secondEdgeVertexPairs : firstEdgeVertexPairs;
            ReadOnlySpan<int> targetEdges = reverse ? firstEdgeVertexPairs : secondEdgeVertexPairs;
            ref readonly WideRationalBasis3d sourceBasis = ref (reverse ? ref secondBasis : ref firstBasis);
            ref readonly WideRationalBasis3d targetBasis = ref (reverse ? ref firstBasis : ref secondBasis);
            WideRigidProjection.GetLocalAxisProjections(axis, sourceBasis,
                out Signed576 sourceX, out Signed576 sourceY, out Signed576 sourceZ);
            WideRigidProjection.GetLocalAxisProjections(axis, targetBasis,
                out Signed576 targetX, out Signed576 targetY, out Signed576 targetZ);
            Signed576 sourceSupport = WideRigidProjection.GetLocalOffsetProjection(
                GetHullSupportLocalPoint(sourceBasis, sourcePoints, axis, maximize: !reverse),
                sourceX, sourceY, sourceZ);
            Signed576 targetSupport = WideRigidProjection.GetLocalOffsetProjection(
                GetHullSupportLocalPoint(targetBasis, targetPoints, axis, maximize: reverse),
                targetX, targetY, targetZ);
            int supportVertices = 0, admittedVertices = 0;
            foreach (Vector3d vertex in sourcePoints)
            {
                if (!IsSupport(vertex, sourceX, sourceY, sourceZ, sourceSupport)) continue;
                supportVertices++;
                var source = new FixedPointAnchor(sourceOrigin, sourceRotation, vertex);
                for (int triangle = 0; triangle < targetTriangles.Length; triangle += 3)
                {
                    Vector3d a = targetPoints[targetTriangles[triangle]],
                        b = targetPoints[targetTriangles[triangle + 1]],
                        c = targetPoints[targetTriangles[triangle + 2]];
                    if (!IsSupport(a, targetX, targetY, targetZ, targetSupport)
                        || !IsSupport(b, targetX, targetY, targetZ, targetSupport)
                        || !IsSupport(c, targetX, targetY, targetZ, targetSupport)) continue;
                    var face = new FixedTriangle(a, b, c);
                    if (!WideTriangleRelations.ContainsProjection(face, targetOrigin, targetRotation, source)) continue;
                    FixedPointAnchor target = WideTriangleRelations.GetClosestPointAnchor(
                        face, targetOrigin, targetRotation, source, preserveExactFace: true);
                    visitor(state, reverse
                        ? new FixedContactAnchors(target, source, normal, depth, clamped)
                        : new FixedContactAnchors(source, target, normal, depth, clamped));
                    count++;
                    admittedVertices++;
                    break;
                }
            }
            // Convexity makes admitted support vertices certify the complete projected feature.
            if (!reverse && supportVertices != 0 && admittedVertices == supportVertices) return count;
            for (int edge = 0; edge < sourceEdges.Length; edge += 2)
            {
                Vector3d start = sourcePoints[sourceEdges[edge]], end = sourcePoints[sourceEdges[edge + 1]];
                if (!IsSupport(start, sourceX, sourceY, sourceZ, sourceSupport)
                    || !IsSupport(end, sourceX, sourceY, sourceZ, sourceSupport)) continue;
                var first = new FixedPointAnchor(sourceOrigin, sourceRotation, start);
                var second = new FixedPointAnchor(sourceOrigin, sourceRotation, end);
                for (int triangle = 0; triangle < targetTriangles.Length; triangle += 3)
                {
                    int ia = targetTriangles[triangle], ib = targetTriangles[triangle + 1], ic = targetTriangles[triangle + 2];
                    Vector3d a = targetPoints[ia], b = targetPoints[ib], c = targetPoints[ic];
                    if (!IsSupport(a, targetX, targetY, targetZ, targetSupport)
                        || !IsSupport(b, targetX, targetY, targetZ, targetSupport)
                        || !IsSupport(c, targetX, targetY, targetZ, targetSupport)) continue;
                    var face = new FixedTriangle(a, b, c);
                    if (!WideTriangleRelations.TryGetProjectedSegmentInterval(face, targetOrigin, targetRotation, first, second,
                            out Signed832 lowerN, out Signed832 lowerD, out Signed832 upperN, out Signed832 upperD)) continue;
                    int mask = (HasEdge(ib, ic, targetEdges) ? 1 : 0)
                        | (HasEdge(ic, ia, targetEdges) ? 2 : 0) | (HasEdge(ia, ib, targetEdges) ? 4 : 0);
                    for (int endpoint = 0; endpoint < 2; endpoint++)
                    {
                        Signed832 n = endpoint == 0 ? lowerN : upperN, d = endpoint == 0 ? lowerD : upperD;
                        if (n.Sign == 0 || WideArithmetic.SubtractSigned832(d, n).IsZero) continue;
                        if (!WideTriangleRelations.TryGetProjectedSegmentFaceAnchors(face, targetOrigin, targetRotation,
                                first, second, n, d,
                                mask, out FixedPointAnchor source, out FixedPointAnchor target)) continue;
                        visitor(state, reverse
                            ? new FixedContactAnchors(target, source, normal, depth, clamped)
                            : new FixedContactAnchors(source, target, normal, depth, clamped));
                        count++;
                    }
                }
            }
        }
        if (count != 0) return count;
        WideRigidProjection.GetLocalAxisProjections(axis, firstBasis,
            out Signed576 firstX, out Signed576 firstY, out Signed576 firstZ);
        WideRigidProjection.GetLocalAxisProjections(axis, secondBasis,
            out Signed576 secondX, out Signed576 secondY, out Signed576 secondZ);
        Signed576 firstSupport = WideRigidProjection.GetLocalOffsetProjection(
            GetHullSupportLocalPoint(firstBasis, firstLocalPoints, axis, maximize: true), firstX, firstY, firstZ);
        Signed576 secondSupport = WideRigidProjection.GetLocalOffsetProjection(
            GetHullSupportLocalPoint(secondBasis, secondLocalPoints, axis, maximize: false), secondX, secondY, secondZ);
        for (int firstEdge = 0; firstEdge < firstEdgeVertexPairs.Length; firstEdge += 2)
        {
            Vector3d a = firstLocalPoints[firstEdgeVertexPairs[firstEdge]], b = firstLocalPoints[firstEdgeVertexPairs[firstEdge + 1]];
            if (!IsSupport(a, firstX, firstY, firstZ, firstSupport) || !IsSupport(b, firstX, firstY, firstZ, firstSupport)) continue;
            for (int secondEdge = 0; secondEdge < secondEdgeVertexPairs.Length; secondEdge += 2)
            {
                Vector3d c = secondLocalPoints[secondEdgeVertexPairs[secondEdge]], d = secondLocalPoints[secondEdgeVertexPairs[secondEdge + 1]];
                if (!IsSupport(c, secondX, secondY, secondZ, secondSupport) || !IsSupport(d, secondX, secondY, secondZ, secondSupport)) continue;
                if (!WideTriangleRelations.TryGetClosestNonparallelSegmentAnchors(
                        firstOrigin, firstRotation, new FixedSegment(a, b), secondOrigin, secondRotation, new FixedSegment(c, d),
                        out FixedPointAnchor firstAnchor, out FixedPointAnchor secondAnchor)) continue;
                visitor(state, new FixedContactAnchors(firstAnchor, secondAnchor, normal, depth, clamped));
                return 1;
            }
        }
        return 0;

        static bool IsSupport(Vector3d point, Signed576 x, Signed576 y, Signed576 z, Signed576 support) =>
            CompareSigned(WideRigidProjection.GetLocalOffsetProjection(point, x, y, z), support) == 0;

        static bool HasEdge(int first, int second, ReadOnlySpan<int> edges)
        {
            for (int index = 0; index < edges.Length; index += 2)
                if (edges[index] == first && edges[index + 1] == second
                    || edges[index] == second && edges[index + 1] == first) return true;
            return false;
        }
    }

    private static bool TryKeepHullFaceAxes(
        ReadOnlySpan<Vector3d> axisSourcePoints,
        ReadOnlySpan<int> axisSourceTriangles,
        in WideRationalBasis3d axisBasis,
        Vector3d firstOrigin,
        in WideRationalBasis3d firstBasis,
        ReadOnlySpan<Vector3d> firstPoints,
        Vector3d secondOrigin,
        in WideRationalBasis3d secondBasis,
        ReadOnlySpan<Vector3d> secondPoints,
        ref WidePointSpanPenetration best, bool skipParallelDirections)
    {
        for (int index = 0; index < axisSourceTriangles.Length; index += 3)
        {
            WideAxis3 local = GetLocalHullFaceAxis(axisSourcePoints, axisSourceTriangles, index);
            if (local.IsZero) continue;
            bool duplicate = false;
            for (int previous = 0; skipParallelDirections && previous < index; previous += 3)
            {
                WideAxis3 earlier = GetLocalHullFaceAxis(axisSourcePoints, axisSourceTriangles, previous);
                if (!earlier.IsZero && WideAxis3.Cross(local, earlier).IsZero) { duplicate = true; break; }
            }
            if (duplicate) continue;
            if (!TryKeepHullPairAxis(
                    WideRigidProjection.TransformLocalAxis(
                        axisBasis,
                        Signed192.NarrowProven(local.X),
                        Signed192.NarrowProven(local.Y),
                        Signed192.NarrowProven(local.Z)),
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

    private static WideAxis3 GetLocalHullFaceAxis(ReadOnlySpan<Vector3d> points, ReadOnlySpan<int> triangles, int index)
    {
        Vector3d a = points[triangles[index]], b = points[triangles[index + 1]], c = points[triangles[index + 2]];
        WideGeometry.GetDifferenceCrossProduct3D(b.X, a.X, b.Y, a.Y, b.Z, a.Z,
            c.X, a.X, c.Y, a.Y, c.Z, a.Z, out Signed192 x, out Signed192 y, out Signed192 z);
        return new WideAxis3(Signed320.ExtendValue(x), Signed320.ExtendValue(y), Signed320.ExtendValue(z));
    }

    private static bool IsFirstHullEdgeDirection(ReadOnlySpan<Vector3d> points, ReadOnlySpan<int> edges, int index)
    {
        WideAxis3 local = Direction(points[edges[index]], points[edges[index + 1]]);
        if (local.IsZero) return false;
        // Local edge/face crosses fit 132/264 bits; crossing already transformed SAT axes need not fit.
        for (int previous = 0; previous < index; previous += 2)
        {
            WideAxis3 earlier = Direction(points[edges[previous]], points[edges[previous + 1]]);
            if (!earlier.IsZero && WideAxis3.Cross(local, earlier).IsZero) return false;
        }
        return true;

        static WideAxis3 Direction(Vector3d first, Vector3d second) => new(
            Signed320.ExtendValue(WideArithmetic.Difference(second.X, first.X)),
            Signed320.ExtendValue(WideArithmetic.Difference(second.Y, first.Y)),
            Signed320.ExtendValue(WideArithmetic.Difference(second.Z, first.Z)));
    }

    private static bool TryKeepHullPairAxis(
        in WideAxis3 axis,
        Vector3d firstOrigin,
        in WideRationalBasis3d firstBasis,
        ReadOnlySpan<Vector3d> firstPoints,
        Vector3d secondOrigin,
        in WideRationalBasis3d secondBasis,
        ReadOnlySpan<Vector3d> secondPoints,
        ref WidePointSpanPenetration best)
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
        Signed576 originProjection = WideRigidProjection.GetWorldOriginDifferenceProjection(
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
        Signed576 squaredAxisLength = axis.SquaredLength;
        if (best.ShouldReplace(
            overlap,
            squaredAxisLength,
            commonDenominator))
        {
            best = new WidePointSpanPenetration(
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
        in WideRationalBasis3d basis,
        in WideAxis3 axis,
        Signed192 otherDenominator,
        out Signed576 minimum,
        out Signed576 maximum)
    {
        WideRigidProjection.GetLocalAxisProjections(
            axis,
            basis,
            out Signed576 localAxisX,
            out Signed576 localAxisY,
            out Signed576 localAxisZ);
        minimum = WideArithmetic.MultiplySigned576(
            WideRigidProjection.GetLocalOffsetProjection(
                points[0],
                localAxisX,
                localAxisY,
                localAxisZ),
            otherDenominator);
        maximum = minimum;
        for (int index = 1; index < points.Length; index++)
        {
            WideRigidProjection.IncludeProjection(
                WideArithmetic.MultiplySigned576(
                    WideRigidProjection.GetLocalOffsetProjection(
                        points[index],
                        localAxisX,
                        localAxisY,
                        localAxisZ),
                    otherDenominator),
                ref minimum,
                ref maximum);
        }
    }

    private static WideAxis3 GetHullEdge(
        in WideRationalBasis3d basis,
        Vector3d start,
        Vector3d end) =>
        WideRigidProjection.TransformLocalAxis(
            basis,
            WideArithmetic.SubtractSigned192(Signed192.Raw(end.X), Signed192.Raw(start.X)),
            WideArithmetic.SubtractSigned192(Signed192.Raw(end.Y), Signed192.Raw(start.Y)),
            WideArithmetic.SubtractSigned192(Signed192.Raw(end.Z), Signed192.Raw(start.Z)));

    private static Vector3d GetHullSupportLocalPoint(
        in WideRationalBasis3d basis,
        ReadOnlySpan<Vector3d> points,
        in WideAxis3 axis,
        bool maximize)
    {
        WideRigidProjection.GetLocalAxisProjections(
            axis,
            basis,
            out Signed576 localAxisX,
            out Signed576 localAxisY,
            out Signed576 localAxisZ);
        Vector3d best = points[0];
        Signed576 bestProjection = WideRigidProjection.GetLocalOffsetProjection(
            best,
            localAxisX,
            localAxisY,
            localAxisZ);
        for (int index = 1; index < points.Length; index++)
        {
            Vector3d candidate = points[index];
            Signed576 projection = WideRigidProjection.GetLocalOffsetProjection(
                candidate,
                localAxisX,
                localAxisY,
                localAxisZ);
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
