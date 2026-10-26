//=======================================================================
// WideOrientedBox.ScaledHull.cs
//=======================================================================
// MIT License, Copyright (c) 2024-present David Oravsky (mrdav30)
// See LICENSE file in the project root for full license information.
//=======================================================================
using System;

namespace FixedMathSharp.Geometry;

/// <summary>Exact signed exit retained before materializing a scaled hull's final contact.</summary>
internal readonly struct WideScaledHullPenetration
{
    internal readonly WideDirection3d Axis;
    internal readonly Signed832 Overlap, SquaredAxisLength;
    internal readonly Signed576 CommonDenominator;
    internal readonly bool Negate;
    internal bool HasValue { get; }
    internal WideScaledHullPenetration(WideDirection3d axis, bool negate, Signed832 overlap,
        Signed832 squaredAxisLength, Signed576 commonDenominator)
    {
        Axis = axis; Negate = negate; Overlap = overlap; SquaredAxisLength = squaredAxisLength;
        CommonDenominator = commonDenominator; HasValue = true;
    }
    internal WideScaledHullPenetration(in WidePointSpanPenetration value)
    {
        Axis = new(value.Axis); Negate = value.Negate;
        Overlap = Signed832.ExtendValue(value.ExactOverlap);
        SquaredAxisLength = Signed832.ExtendValue(value.ExactSquaredAxisLength);
        CommonDenominator = Signed576.ExtendValue(value.ExactCommonDenominator);
        HasValue = value.HasValue;
    }
    internal bool ShouldReplace(Signed832 overlap, Signed832 squaredAxisLength, Signed576 denominator) =>
        !HasValue || WideArithmetic.CompareNonNegativeNormalizedDepths(overlap, squaredAxisLength, denominator,
            Overlap, SquaredAxisLength, CommonDenominator) < 0;
    internal Vector3d GetNormal() => Axis.GetNormal(Negate);
    internal Fixed64 GetRoundedDepth(out bool isClamped) => WideArithmetic.GetRoundedNonNegativeNormalizedDepth(
        Overlap, SquaredAxisLength, CommonDenominator, out isClamped);
}

/// <summary>Admits an exact scaled-hull exit before depth ranking.</summary>
internal delegate bool ScaledHullPenetrationAdmission(in WideScaledHullPenetration candidate);

/// <summary>Exact retained features for scaled authored hull contact selection.</summary>
internal interface IScaledConvexHullContactDomain
{
    ReadOnlySpan<RationalSegmentInterval> GetEdgeIntervals(bool first, int start, int end, in WideDirection3d normal);
    bool ContainsPoint(bool first, int start, int end, in Signed832 numerator, in Signed832 denominator, in WideDirection3d normal);
    bool ContainsPoint(bool first, int start, int end, scoped ContactQuadratic numerator,
        scoped ContactQuadratic denominator, in WideDirection3d normal);
}

/// <content>Authored-frame SAT ingress retaining scale residuals through axis and interval selection.</content>
internal static partial class WideOrientedBox
{
    internal static ReadOnlySpan<int> BoxHullTriangles => new int[]
        { 0,2,3, 0,3,1, 4,5,7, 4,7,6, 0,1,5, 0,5,4, 2,6,7, 2,7,3, 0,4,6, 0,6,2, 1,3,7, 1,7,5 };
    internal static ReadOnlySpan<int> BoxHullEdges => new int[]
        { 0,1, 0,2, 0,4, 1,3, 1,5, 2,3, 2,6, 3,7, 4,5, 4,6, 5,7, 6,7 };

    internal static void GetBoxHullVertices(Vector3d halfExtents, Span<Vector3d> vertices)
    {
        for (int index = 0; index < 8; index++)
            vertices[index] = new Vector3d((index & 1) == 0 ? -halfExtents.X : halfExtents.X,
                (index & 2) == 0 ? -halfExtents.Y : halfExtents.Y,
                (index & 4) == 0 ? -halfExtents.Z : halfExtents.Z);
    }

    internal static bool TryGetConvexHullContact(Vector3d boxCenter, FixedQuaternion boxRotation,
        Vector3d boxHalfExtents, in ScaledRigidFrame3d hullFrame, Vector3d hullAnchorOrigin,
        ReadOnlySpan<Vector3d> hullPoints, ReadOnlySpan<int> triangles, ReadOnlySpan<int> edges,
        out FixedContactAnchors contact)
    {
        if (hullFrame.IsRigid)
            return TryGetConvexHullContact(boxCenter, boxRotation, boxHalfExtents,
                hullFrame.RigidOrigin, hullFrame.Rotation, hullPoints, triangles, edges, out contact);
        Span<Vector3d> vertices = stackalloc Vector3d[8];
        GetBoxHullVertices(boxHalfExtents, vertices);
        return TryGetConvexHullContact(ScaledRigidFrame3d.Rigid(boxCenter, boxRotation), boxCenter,
            vertices, BoxHullTriangles, BoxHullEdges, hullFrame, hullAnchorOrigin, hullPoints,
            triangles, edges, out contact);
    }

    internal static bool TryGetConvexHullContact(
        in ScaledRigidFrame3d firstFrame, Vector3d firstAnchorOrigin, ReadOnlySpan<Vector3d> firstPoints,
        ReadOnlySpan<int> firstTriangles, ReadOnlySpan<int> firstEdges,
        in ScaledRigidFrame3d secondFrame, Vector3d secondAnchorOrigin, ReadOnlySpan<Vector3d> secondPoints,
        ReadOnlySpan<int> secondTriangles, ReadOnlySpan<int> secondEdges, out FixedContactAnchors contact)
    {
        contact = default;
        if (!TryGetConvexHullPenetration(firstFrame, firstPoints, firstTriangles, firstEdges,
                secondFrame, secondPoints, secondTriangles, secondEdges, out var best)) return false;
        WideDirection3d axis = best.Negate ? -best.Axis : best.Axis;
        Signed832 firstSupport = GetScaledHullSupport(firstFrame, firstPoints, axis, true);
        Signed832 secondSupport = GetScaledHullSupport(secondFrame, secondPoints, axis, false);
        Vector3d firstLocal = default, secondLocal = default;
        bool firstFound = false, secondFound = false;
        foreach (Vector3d vertex in firstPoints)
        {
            RationalPoint3d point = firstFrame.GetPoint(vertex);
            if (!IsScaledHullSupport(point, axis, firstSupport)) continue;
            firstFound = point.TryGetLocalPointIn(firstAnchorOrigin, FixedQuaternion.Identity, out firstLocal);
            break;
        }
        foreach (Vector3d vertex in secondPoints)
        {
            RationalPoint3d point = secondFrame.GetPoint(vertex);
            if (!IsScaledHullSupport(point, axis, secondSupport)) continue;
            secondFound = point.TryGetLocalPointIn(secondAnchorOrigin, FixedQuaternion.Identity, out secondLocal);
            break;
        }
        if (!firstFound || !secondFound) return false;
        Fixed64 depth = best.GetRoundedDepth(out bool clamped);
        contact = new FixedContactAnchors(new FixedPointAnchor(firstAnchorOrigin, FixedQuaternion.Identity, firstLocal),
            new FixedPointAnchor(secondAnchorOrigin, FixedQuaternion.Identity, secondLocal), best.GetNormal(), depth, clamped);
        return true;
    }

    internal static int VisitConvexHullContacts<TState>(
        in ScaledRigidFrame3d firstFrame, Vector3d firstAnchorOrigin, ReadOnlySpan<Vector3d> firstPoints,
        ReadOnlySpan<int> firstTriangles, ReadOnlySpan<int> firstEdges,
        in ScaledRigidFrame3d secondFrame, Vector3d secondAnchorOrigin, ReadOnlySpan<Vector3d> secondPoints,
        ReadOnlySpan<int> secondTriangles, ReadOnlySpan<int> secondEdges,
        in WideScaledHullPenetration penetration, TState state, Action<TState, FixedContactAnchors> visitor,
        IScaledConvexHullContactDomain? domain = null, bool admissionOnly = false)
 =>
        VisitConvexHullContacts<ScaledRigidFrame3d, ScaledRigidFrame3d, TState>(firstFrame, firstAnchorOrigin, firstPoints,
            firstTriangles, firstEdges, secondFrame, secondAnchorOrigin, secondPoints, secondTriangles, secondEdges,
            penetration, state, visitor, domain, admissionOnly);

    internal static int VisitConvexHullContacts<TFirstFrame, TSecondFrame, TState>(
        in TFirstFrame firstFrame, Vector3d firstAnchorOrigin, ReadOnlySpan<Vector3d> firstPoints,
        ReadOnlySpan<int> firstTriangles, ReadOnlySpan<int> firstEdges,
        in TSecondFrame secondFrame, Vector3d secondAnchorOrigin, ReadOnlySpan<Vector3d> secondPoints,
        ReadOnlySpan<int> secondTriangles, ReadOnlySpan<int> secondEdges,
        in WideScaledHullPenetration penetration, TState state, Action<TState, FixedContactAnchors> visitor,
        IScaledConvexHullContactDomain? domain = null, bool admissionOnly = false)
        where TFirstFrame : struct, IForwardHullFrame3d
        where TSecondFrame : struct, IForwardHullFrame3d
    {
        WideDirection3d axis = penetration.Negate ? -penetration.Axis : penetration.Axis;
        Vector3d normal = admissionOnly ? default : axis.GetNormal(false);
        bool clamped = false;
        Fixed64 depth = admissionOnly ? default : penetration.GetRoundedDepth(out clamped);
        int count = VisitScaledHullSide(firstFrame, firstAnchorOrigin, firstPoints, firstEdges,
            secondFrame, secondAnchorOrigin, secondPoints, secondTriangles, secondEdges, axis,
            normal, depth, clamped, state, visitor, domain, admissionOnly, false, out bool complete);
        if (complete || admissionOnly && count != 0) return count;
        count += VisitScaledHullSide(secondFrame, secondAnchorOrigin, secondPoints, secondEdges,
            firstFrame, firstAnchorOrigin, firstPoints, firstTriangles, firstEdges, axis,
            normal, depth, clamped, state, visitor, domain, admissionOnly, true, out _);

        if (count != 0) return count;
        Signed832 firstSupport = GetScaledHullSupport(firstFrame, firstPoints, axis, true);
        Signed832 secondSupport = GetScaledHullSupport(secondFrame, secondPoints, axis, false);
        const int parameterWords = 27;
        Span<ulong> parameters = stackalloc ulong[8 * parameterWords]; Span<int> parameterSigns = stackalloc int[8];
        var firstN = ContactQuadratic.At(parameters, parameterSigns, 0, parameterWords);
        var firstD = ContactQuadratic.At(parameters, parameterSigns, 1, parameterWords);
        var secondN = ContactQuadratic.At(parameters, parameterSigns, 2, parameterWords);
        var secondD = ContactQuadratic.At(parameters, parameterSigns, 3, parameterWords);
        for (int firstEdge = 0; firstEdge < firstEdges.Length; firstEdge += 2)
        {
            int ia = firstEdges[firstEdge], ib = firstEdges[firstEdge + 1];
            Vector3d a = firstPoints[ia], b = firstPoints[ib];
            if (!IsScaledHullSupport(firstFrame.GetPoint(a), axis, firstSupport)
                || !IsScaledHullSupport(firstFrame.GetPoint(b), axis, firstSupport)) continue;
            for (int secondEdge = 0; secondEdge < secondEdges.Length; secondEdge += 2)
            {
                int ic = secondEdges[secondEdge], id = secondEdges[secondEdge + 1];
                Vector3d c = secondPoints[ic], d = secondPoints[id];
                if (!IsScaledHullSupport(secondFrame.GetPoint(c), axis, secondSupport)
                    || !IsScaledHullSupport(secondFrame.GetPoint(d), axis, secondSupport)) continue;
                var firstSegment = new FixedSegment(a, b); var secondSegment = new FixedSegment(c, d);
                if (!WideTriangleRelations.TryGetClosestNonparallelSegmentParameters(firstFrame.GetPoint(a), firstFrame.GetPoint(b),
                        secondFrame.GetPoint(c), secondFrame.GetPoint(d), firstN, firstD, secondN, secondD)) continue;
                if (domain != null && (!domain.ContainsPoint(true, ia, ib, firstN, firstD, axis)
                    || !domain.ContainsPoint(false, ic, id, secondN, secondD, axis))) continue;
                if (admissionOnly) return 1;
                if (!WideTriangleRelations.TryGetSegmentPointAnchor(firstFrame, firstSegment, firstN, firstD,
                        firstAnchorOrigin, out FixedPointAnchor firstAnchor)
                    || !WideTriangleRelations.TryGetSegmentPointAnchor(secondFrame, secondSegment, secondN, secondD,
                        secondAnchorOrigin, out FixedPointAnchor secondAnchor)) continue;
                visitor(state, new FixedContactAnchors(firstAnchor, secondAnchor, normal, depth, clamped));
                return 1;
            }
        }
        return 0;
    }

    /// <summary>Visits one selected hull support against a retained finite extrusion side.</summary>
    /// <remarks>
    /// A/B share local Y and A/C share local X/Z. The rectangle must lie on
    /// the second shape's selected support plane; the original full hull and
    /// full-solid penetration remain unchanged. Tangent cuts are never rounded.
    /// </remarks>
    internal static int VisitProjectedHullRectangleContacts<TSourceFrame, TRectangleFrame, TState>(
        in TSourceFrame sourceFrame, Vector3d sourceOrigin, ReadOnlySpan<Vector3d> sourcePoints,
        ReadOnlySpan<int> sourceTriangles, ReadOnlySpan<int> sourceEdges,
        in TRectangleFrame rectangleFrame, Vector3d rectangleOrigin, in FixedTriangle rectangleBasis,
        in RationalSegmentInterval tangentClip, in WideScaledHullPenetration penetration,
        TState state, Action<TState, FixedContactAnchors> visitor, bool admissionOnly = false)
        where TSourceFrame : struct, IForwardHullFrame3d
        where TRectangleFrame : struct, IForwardHullFrame3d
    {
        WideDirection3d axis = penetration.Negate ? -penetration.Axis : penetration.Axis;
        WideDirection3d faceNormal = rectangleFrame.GetFaceNormal(rectangleBasis);
        if (faceNormal.IsZero || !WideDirection3d.AreParallel(axis, faceNormal)) return 0;
        Signed832 support = GetScaledHullSupport(sourceFrame, sourcePoints, axis, true);
        Vector3d normal = admissionOnly ? default : axis.GetNormal(false);
        bool clamped = false;
        Fixed64 depth = admissionOnly ? default : penetration.GetRoundedDepth(out clamped);
        const int words = 32;
        Span<ulong> values = stackalloc ulong[10 * words]; Span<int> signs = stackalloc int[10];
        var lowerN = ContactQuadratic.At(values, signs, 0, words);
        var lowerD = ContactQuadratic.At(values, signs, 1, words);
        var upperN = ContactQuadratic.At(values, signs, 2, words);
        var upperD = ContactQuadratic.At(values, signs, 3, words);
        var one = ContactQuadratic.At(values, signs, 4, words); one.Set(Signed576.ExtendValue(Signed192.Signed(1)));
        int count = 0, supportVertices = 0;
        foreach (Vector3d vertex in sourcePoints)
        {
            RationalPoint3d point = sourceFrame.GetPoint(vertex);
            if (!IsScaledHullSupport(point, axis, support)) continue;
            supportVertices++;
            if (!WideTriangleRelations.TryGetProjectedRectangleInterval(rectangleBasis, rectangleFrame,
                    point, point, tangentClip, lowerN, lowerD, upperN, upperD)) continue;
            if (admissionOnly) return 1;
            if (!point.TryGetLocalPointIn(sourceOrigin, FixedQuaternion.Identity, out Vector3d local)
                || !WideTriangleRelations.TryGetProjectedPlanePointAnchor(rectangleBasis, rectangleFrame, point,
                    rectangleOrigin, out FixedPointAnchor target)) continue;
            visitor(state, new(new FixedPointAnchor(sourceOrigin, FixedQuaternion.Identity, local), target, normal, depth, clamped));
            count++;
        }
        if (count != 0 && count == supportVertices) return count;
        for (int edge = 0; edge < sourceEdges.Length; edge += 2)
        {
            RationalPoint3d first = sourceFrame.GetPoint(sourcePoints[sourceEdges[edge]]),
                second = sourceFrame.GetPoint(sourcePoints[sourceEdges[edge + 1]]);
            if (!IsScaledHullSupport(first, axis, support) || !IsScaledHullSupport(second, axis, support)
                || !WideTriangleRelations.TryGetProjectedRectangleInterval(rectangleBasis, rectangleFrame,
                    first, second, tangentClip, lowerN, lowerD, upperN, upperD)) continue;
            if (admissionOnly) return 1;
            for (int endpoint = 0; endpoint < 2; endpoint++)
            {
                var n = endpoint == 0 ? lowerN : upperN; var d = endpoint == 0 ? lowerD : upperD;
                if (n.Signs[0] == 0 || ContactQuadratic.CompareRatios(n, d, ReadOnlySpan<ulong>.Empty,
                        one, one, ReadOnlySpan<ulong>.Empty) == 0) continue;
                if (!WideTriangleRelations.TryGetProjectedRectangleFaceAnchors(rectangleBasis, rectangleFrame,
                        first, second, n, d, sourceOrigin, rectangleOrigin, out FixedPointAnchor source, out FixedPointAnchor target)) continue;
                visitor(state, new(source, target, normal, depth, clamped)); count++;
            }
        }
        // A small rectangle can lie wholly inside the projected hull face.
        // Its original horizontal edges supply exact rational cut corners.
        Span<Vector3d> corners = stackalloc Vector3d[4]
        {
            rectangleBasis.A, rectangleBasis.B, rectangleBasis.C,
            new(rectangleBasis.B.X, rectangleBasis.C.Y, rectangleBasis.B.Z)
        };
        for (int edge = 0; edge < 4; edge++)
        {
            bool horizontal = edge < 2;
            if (!horizontal && tangentClip.IsClipped)
            {
                Signed832 endpoint = edge == 2 ? default : Signed832.ExtendValue(Signed192.Signed(1));
                Signed832 divisor = Signed832.ExtendValue(Signed192.Signed(1));
                if (RationalSegmentInterval.CompareParameters(endpoint, divisor,
                        tangentClip.WideLowerNumerator, tangentClip.WideLowerDenominator) < 0
                    || RationalSegmentInterval.CompareParameters(endpoint, divisor,
                        tangentClip.WideUpperNumerator, tangentClip.WideUpperDenominator) > 0) continue;
            }
            int a = edge < 2 ? 2 * edge : edge - 2, b = edge < 2 ? 2 * edge + 1 : edge;
            RationalPoint3d first = rectangleFrame.GetPoint(corners[a]), second = rectangleFrame.GetPoint(corners[b]);
            for (int triangle = 0; triangle < sourceTriangles.Length; triangle += 3)
            {
                int ia = sourceTriangles[triangle], ib = sourceTriangles[triangle + 1], ic = sourceTriangles[triangle + 2];
                Vector3d x = sourcePoints[ia], y = sourcePoints[ib], z = sourcePoints[ic];
                if (!IsScaledHullSupport(sourceFrame.GetPoint(x), axis, support)
                    || !IsScaledHullSupport(sourceFrame.GetPoint(y), axis, support)
                    || !IsScaledHullSupport(sourceFrame.GetPoint(z), axis, support)) continue;
                var face = new FixedTriangle(x, y, z);
                if (!WideTriangleRelations.TryGetProjectedSegmentInterval(face, sourceFrame, first, second,
                        out Signed832 ln, out Signed832 ld, out Signed832 un, out Signed832 ud)
                    || horizontal && !IntersectScaledHullInterval(tangentClip, ref ln, ref ld, ref un, ref ud)) continue;
                if (admissionOnly) return 1;
                int mask = (HasScaledHullEdge(ib, ic, sourceEdges) ? 1 : 0)
                    | (HasScaledHullEdge(ic, ia, sourceEdges) ? 2 : 0) | (HasScaledHullEdge(ia, ib, sourceEdges) ? 4 : 0);
                for (int endpoint = 0; endpoint < 2; endpoint++)
                {
                    Signed832 n = endpoint == 0 ? ln : un, d = endpoint == 0 ? ld : ud;
                    bool corner = n.IsZero || WideArithmetic.SubtractSigned832(d, n).IsZero
                        || horizontal && tangentClip.IsClipped && (RationalSegmentInterval.CompareParameters(n, d,
                            tangentClip.WideLowerNumerator, tangentClip.WideLowerDenominator) == 0
                            || RationalSegmentInterval.CompareParameters(n, d, tangentClip.WideUpperNumerator,
                                tangentClip.WideUpperDenominator) == 0);
                    if (!WideTriangleRelations.TryGetProjectedFrameSegmentFaceAnchors(face, sourceFrame, first, second, n, d,
                            rectangleOrigin, sourceOrigin, mask, out FixedPointAnchor target, out FixedPointAnchor source,
                            requirePhysicalTargetBoundary: !corner)) continue;
                    visitor(state, new(source, target, normal, depth, clamped)); count++;
                }
            }
        }
        return count;
    }

    private static int VisitScaledHullSide<TSourceFrame, TTargetFrame, TState>(
        in TSourceFrame sourceFrame, Vector3d sourceOrigin, ReadOnlySpan<Vector3d> sourcePoints, ReadOnlySpan<int> sourceEdges,
        in TTargetFrame targetFrame, Vector3d targetOrigin, ReadOnlySpan<Vector3d> targetPoints,
        ReadOnlySpan<int> targetTriangles, ReadOnlySpan<int> targetEdges, in WideDirection3d axis,
        Vector3d normal, Fixed64 depth, bool clamped, TState state, Action<TState, FixedContactAnchors> visitor,
        IScaledConvexHullContactDomain? domain, bool admissionOnly, bool reverse, out bool allVerticesAdmitted)
        where TSourceFrame : struct, IForwardHullFrame3d
        where TTargetFrame : struct, IForwardHullFrame3d
    {
        allVerticesAdmitted = false;
        int count = 0;
        Signed832 sourceSupport = GetScaledHullSupport(sourceFrame, sourcePoints, axis, !reverse);
        Signed832 targetSupport = GetScaledHullSupport(targetFrame, targetPoints, axis, reverse);
        int supportVertices = 0, admittedVertices = 0;
        for (int vertex = 0; vertex < sourcePoints.Length; vertex++)
        {
            RationalPoint3d point = sourceFrame.GetPoint(sourcePoints[vertex]);
            if (!IsScaledHullSupport(point, axis, sourceSupport)) continue;
            supportVertices++;
            for (int triangle = 0; triangle < targetTriangles.Length; triangle += 3)
            {
                Vector3d a = targetPoints[targetTriangles[triangle]], b = targetPoints[targetTriangles[triangle + 1]],
                    c = targetPoints[targetTriangles[triangle + 2]];
                if (!IsScaledHullSupport(targetFrame.GetPoint(a), axis, targetSupport)
                    || !IsScaledHullSupport(targetFrame.GetPoint(b), axis, targetSupport)
                    || !IsScaledHullSupport(targetFrame.GetPoint(c), axis, targetSupport)) continue;
                var face = new FixedTriangle(a, b, c);
                if (!WideTriangleRelations.ContainsProjection(face, targetFrame, point)) continue;
                if (domain != null && !domain.ContainsPoint(!reverse, vertex, vertex, default,
                        Signed832.ExtendValue(Signed192.One), axis)) break;
                if (admissionOnly) return 1;
                if (!point.TryGetLocalPointIn(sourceOrigin, FixedQuaternion.Identity, out Vector3d local)
                    || !WideTriangleRelations.TryGetProjectedPointAnchor(face, targetFrame, point, targetOrigin,
                        out FixedPointAnchor target)) continue;
                var source = new FixedPointAnchor(sourceOrigin, FixedQuaternion.Identity, local);
                visitor(state, reverse ? new FixedContactAnchors(target, source, normal, depth, clamped)
                    : new FixedContactAnchors(source, target, normal, depth, clamped));
                count++; admittedVertices++;
                break;
            }
        }
        if (!reverse && supportVertices != 0 && supportVertices == admittedVertices)
        { allVerticesAdmitted = true; return count; }
        for (int edge = 0; edge < sourceEdges.Length; edge += 2)
        {
            int first = sourceEdges[edge], second = sourceEdges[edge + 1];
            RationalPoint3d start = sourceFrame.GetPoint(sourcePoints[first]), end = sourceFrame.GetPoint(sourcePoints[second]);
            if (!IsScaledHullSupport(start, axis, sourceSupport) || !IsScaledHullSupport(end, axis, sourceSupport)) continue;
            ReadOnlySpan<RationalSegmentInterval> intervals = domain == null ? default : domain.GetEdgeIntervals(!reverse, first, second, axis);
            if (domain != null && intervals.IsEmpty) continue;
            for (int triangle = 0; triangle < targetTriangles.Length; triangle += 3)
            {
                int ia = targetTriangles[triangle], ib = targetTriangles[triangle + 1], ic = targetTriangles[triangle + 2];
                Vector3d a = targetPoints[ia], b = targetPoints[ib], c = targetPoints[ic];
                if (!IsScaledHullSupport(targetFrame.GetPoint(a), axis, targetSupport)
                    || !IsScaledHullSupport(targetFrame.GetPoint(b), axis, targetSupport)
                    || !IsScaledHullSupport(targetFrame.GetPoint(c), axis, targetSupport)) continue;
                var face = new FixedTriangle(a, b, c);
                if (!WideTriangleRelations.TryGetProjectedSegmentInterval(face, targetFrame, start, end,
                        out Signed832 lowerN, out Signed832 lowerD, out Signed832 upperN, out Signed832 upperD)) continue;
                int mask = (HasScaledHullEdge(ib, ic, targetEdges) ? 1 : 0)
                    | (HasScaledHullEdge(ic, ia, targetEdges) ? 2 : 0) | (HasScaledHullEdge(ia, ib, targetEdges) ? 4 : 0);
                for (int clipIndex = 0; clipIndex < (domain == null ? 1 : intervals.Length); clipIndex++)
                {
                    RationalSegmentInterval clip = domain == null ? default : intervals[clipIndex];
                    Signed832 ln = lowerN, ld = lowerD, un = upperN, ud = upperD;
                    if (!IntersectScaledHullInterval(clip, ref ln, ref ld, ref un, ref ud)) continue;
                    for (int endpoint = 0; endpoint < 2; endpoint++)
                    {
                        Signed832 n = endpoint == 0 ? ln : un, d = endpoint == 0 ? ld : ud;
                        if (n.IsZero || WideArithmetic.SubtractSigned832(d, n).IsZero
                            || domain != null && !domain.ContainsPoint(!reverse, first, second, n, d, axis)) continue;
                        bool clipEnd = clip.IsClipped && (RationalSegmentInterval.CompareParameters(n, d,
                            clip.WideLowerNumerator, clip.WideLowerDenominator) == 0
                            || RationalSegmentInterval.CompareParameters(n, d, clip.WideUpperNumerator, clip.WideUpperDenominator) == 0);
                        if (!WideTriangleRelations.TryGetProjectedFrameSegmentFaceAnchors(face, targetFrame, start, end, n, d,
                                sourceOrigin, targetOrigin, mask, out FixedPointAnchor source, out FixedPointAnchor target,
                                requirePhysicalTargetBoundary: !clipEnd, materialize: !admissionOnly)) continue;
                        if (admissionOnly) return 1;
                        visitor(state, reverse ? new FixedContactAnchors(target, source, normal, depth, clamped)
                            : new FixedContactAnchors(source, target, normal, depth, clamped));
                        count++;
                    }
                }
            }
        }
            return count;
    }

    private static bool HasScaledHullEdge(int first, int second, ReadOnlySpan<int> edges)
    {
        for (int edge = 0; edge < edges.Length; edge += 2)
            if (edges[edge] == first && edges[edge + 1] == second || edges[edge] == second && edges[edge + 1] == first) return true;
        return false;
    }

    internal static Signed832 GetScaledHullSupport<TFrame>(in TFrame frame, ReadOnlySpan<Vector3d> points,
        in WideDirection3d axis, bool maximize) where TFrame : struct, IForwardHullFrame3d
    {
        Signed832 best = ProjectScaledHullPoint(frame.GetPoint(points[0]), axis, Signed192.One);
        for (int vertex = 1; vertex < points.Length; vertex++)
        {
            Signed832 candidate = ProjectScaledHullPoint(frame.GetPoint(points[vertex]), axis, Signed192.One);
            int comparison = WideArithmetic.SubtractSigned832(candidate, best).Sign;
            if (maximize ? comparison > 0 : comparison < 0) best = candidate;
        }
        return best;
    }

    internal static bool IsScaledHullSupport(in RationalPoint3d point, in WideDirection3d axis, Signed832 support) =>
        WideArithmetic.SubtractSigned832(ProjectScaledHullPoint(point, axis, Signed192.One), support).IsZero;

    private static bool IntersectScaledHullInterval(in RationalSegmentInterval interval,
        ref Signed832 ln, ref Signed832 ld, ref Signed832 un, ref Signed832 ud)
    {
        if (!interval.IsClipped) return true;
        if (RationalSegmentInterval.CompareParameters(interval.WideLowerNumerator, interval.WideLowerDenominator, ln, ld) > 0)
        { ln = interval.WideLowerNumerator; ld = interval.WideLowerDenominator; }
        if (RationalSegmentInterval.CompareParameters(interval.WideUpperNumerator, interval.WideUpperDenominator, un, ud) < 0)
        { un = interval.WideUpperNumerator; ud = interval.WideUpperDenominator; }
        return RationalSegmentInterval.CompareParameters(ln, ld, un, ud) <= 0;
    }

    /// <summary>
    /// Tests convex authored point spans whose centered exact scaled reach is at
    /// most 4096. The existing rigid ingress remains the scalar fast path.
    /// </summary>
    internal static bool TryGetConvexHullPenetration(
        in ScaledRigidFrame3d firstFrame, ReadOnlySpan<Vector3d> firstPoints,
        ReadOnlySpan<int> firstTriangles, ReadOnlySpan<int> firstEdges,
        in ScaledRigidFrame3d secondFrame, ReadOnlySpan<Vector3d> secondPoints,
        ReadOnlySpan<int> secondTriangles, ReadOnlySpan<int> secondEdges,
        out WideScaledHullPenetration best, bool skipParallelDirections = false,
        ScaledHullPenetrationAdmission? admission = null, bool planarPair = false)
 =>
        TryGetConvexHullPenetration<ScaledRigidFrame3d, ScaledRigidFrame3d>(firstFrame, firstPoints, firstTriangles, firstEdges,
            secondFrame, secondPoints, secondTriangles, secondEdges, out best, skipParallelDirections, admission, planarPair);

    internal static bool TryGetConvexHullPenetration<TFirstFrame, TSecondFrame>(
        in TFirstFrame firstFrame, ReadOnlySpan<Vector3d> firstPoints,
        ReadOnlySpan<int> firstTriangles, ReadOnlySpan<int> firstEdges,
        in TSecondFrame secondFrame, ReadOnlySpan<Vector3d> secondPoints,
        ReadOnlySpan<int> secondTriangles, ReadOnlySpan<int> secondEdges,
        out WideScaledHullPenetration best, bool skipParallelDirections = false,
        ScaledHullPenetrationAdmission? admission = null, bool planarPair = false)
        where TFirstFrame : struct, IForwardHullFrame3d
        where TSecondFrame : struct, IForwardHullFrame3d
    {
        best = default;
        bool hasAxis = false;
        Signed576 commonD = WideArithmetic.MultiplySigned320(firstFrame.Denominator, secondFrame.RotationDenominator);
        if (!TryKeepScaledHullFaceAxes(firstFrame, firstPoints, firstTriangles, firstEdges,
                firstFrame, firstPoints, secondFrame, secondPoints, commonD, ref best, ref hasAxis,
                skipParallelDirections, admission, planarPair)
            || !TryKeepScaledHullFaceAxes(secondFrame, secondPoints, secondTriangles, secondEdges,
                firstFrame, firstPoints, secondFrame, secondPoints, commonD, ref best, ref hasAxis,
                skipParallelDirections, admission, planarPair)) return false;
        for (int firstEdge = 0; firstEdge < firstEdges.Length; firstEdge += 2)
        {
            if (skipParallelDirections && !IsFirstHullEdgeDirection(firstPoints, firstEdges, firstEdge)) continue;
            WideDirection3d a = firstFrame.GetEdgeDirection(firstPoints[firstEdges[firstEdge]], firstPoints[firstEdges[firstEdge + 1]]);
            for (int secondEdge = 0; secondEdge < secondEdges.Length; secondEdge += 2)
            {
                if (skipParallelDirections && !IsFirstHullEdgeDirection(secondPoints, secondEdges, secondEdge)) continue;
                WideDirection3d b = secondFrame.GetEdgeDirection(secondPoints[secondEdges[secondEdge]], secondPoints[secondEdges[secondEdge + 1]]);
                if (!TryKeepScaledHullAxis(WideDirection3d.Cross(a, b), firstFrame, firstPoints,
                        secondFrame, secondPoints, commonD, ref best, ref hasAxis, admission))
                { best = default; return false; }
            }
        }
        return admission == null ? best.HasValue : hasAxis;
    }

    private static bool TryKeepScaledHullFaceAxes<TFrame, TFirstFrame, TSecondFrame>(
        in TFrame frame, ReadOnlySpan<Vector3d> points, ReadOnlySpan<int> triangles, ReadOnlySpan<int> edges,
        in TFirstFrame firstFrame, ReadOnlySpan<Vector3d> firstPoints,
        in TSecondFrame secondFrame, ReadOnlySpan<Vector3d> secondPoints, Signed576 commonD,
        ref WideScaledHullPenetration best, ref bool hasAxis, bool skipParallelDirections,
        ScaledHullPenetrationAdmission? admission, bool planarPair)
        where TFrame : struct, IForwardHullFrame3d
        where TFirstFrame : struct, IForwardHullFrame3d
        where TSecondFrame : struct, IForwardHullFrame3d
    {
        FixedTriangle plane = default;
        for (int triangle = 0; triangle < triangles.Length; triangle += 3)
        {
            var face = new FixedTriangle(points[triangles[triangle]], points[triangles[triangle + 1]], points[triangles[triangle + 2]]);
            WideDirection3d normal = frame.GetFaceNormal(face);
            if (normal.IsZero) continue;
            bool duplicate = false;
            // Exact world face normals remain below 284 bits, so the
            // parallel cross stays within Signed576 for both frame charts.
            for (int previous = 0; skipParallelDirections && previous < triangle; previous += 3)
                if (WideDirection3d.Cross(normal, frame.GetFaceNormal(new(points[triangles[previous]],
                        points[triangles[previous + 1]], points[triangles[previous + 2]]))).IsZero)
                { duplicate = true; break; }
            if (duplicate) continue;
            plane = face;
            if (!TryKeepScaledHullAxis(normal, firstFrame, firstPoints, secondFrame, secondPoints,
                    commonD, ref best, ref hasAxis, admission)) { best = default; return false; }
        }
        if (planarPair)
            for (int edge = 0; edge < edges.Length; edge += 2)
            {
                if (skipParallelDirections && !IsFirstHullEdgeDirection(points, edges, edge)) continue;
                WideDirection3d axis = frame.GetInwardAxis(plane, points[edges[edge]], points[edges[edge + 1]]);
                if (!TryKeepScaledHullAxis(axis, firstFrame, firstPoints, secondFrame, secondPoints,
                        commonD, ref best, ref hasAxis, admission)) { best = default; return false; }
            }
        return true;
    }

    private static bool TryKeepScaledHullAxis<TFirstFrame, TSecondFrame>(in WideDirection3d axis,
        in TFirstFrame firstFrame, ReadOnlySpan<Vector3d> firstPoints,
        in TSecondFrame secondFrame, ReadOnlySpan<Vector3d> secondPoints, Signed576 commonD,
        ref WideScaledHullPenetration best, ref bool hasAxis, ScaledHullPenetrationAdmission? admission)
        where TFirstFrame : struct, IForwardHullFrame3d
        where TSecondFrame : struct, IForwardHullFrame3d
    {
        if (axis.IsZero) return true;
        hasAxis = true;
        GetScaledHullProjection(firstFrame, firstPoints, axis, secondFrame.RotationDenominator,
            out Signed832 firstMin, out Signed832 firstMax);
        GetScaledHullProjection(secondFrame, secondPoints, axis, firstFrame.RotationDenominator,
            out Signed832 secondMin, out Signed832 secondMax);
        Signed832 negative = WideArithmetic.SubtractSigned832(firstMax, secondMin);
        Signed832 positive = WideArithmetic.SubtractSigned832(secondMax, firstMin);
        if (negative.Sign < 0 || positive.Sign < 0) return false;
        Signed832 squared = axis.SquaredLength;
        bool negate = WideArithmetic.SubtractSigned832(positive, negative).Sign < 0;
        Keep(negate ? positive : negative, negate, axis, squared, commonD, ref best, admission);
        if (admission != null) Keep(negate ? negative : positive, !negate, axis, squared, commonD, ref best, admission);
        return true;

        static void Keep(Signed832 overlap, bool negate, in WideDirection3d direction, Signed832 squared,
            Signed576 denominator, ref WideScaledHullPenetration best, ScaledHullPenetrationAdmission? admission)
        {
            if (!best.ShouldReplace(overlap, squared, denominator)) return;
            var candidate = new WideScaledHullPenetration(direction, negate, overlap, squared, denominator);
            if (admission == null || admission(candidate)) best = candidate;
        }
    }

    private static void GetScaledHullProjection<TFrame>(in TFrame frame, ReadOnlySpan<Vector3d> points,
        in WideDirection3d axis, Signed192 otherRotationDenominator, out Signed832 minimum, out Signed832 maximum)
        where TFrame : struct, IForwardHullFrame3d
    {
        minimum = maximum = ProjectScaledHullPoint(frame.GetPoint(points[0]), axis, otherRotationDenominator);
        for (int index = 1; index < points.Length; index++)
        {
            Signed832 projection = ProjectScaledHullPoint(frame.GetPoint(points[index]), axis, otherRotationDenominator);
            if (WideArithmetic.SubtractSigned832(projection, minimum).Sign < 0) minimum = projection;
            if (WideArithmetic.SubtractSigned832(projection, maximum).Sign > 0) maximum = projection;
        }
    }

    internal static Signed832 ProjectScaledHullPoint(in RationalPoint3d point, in WideDirection3d axis,
        Signed192 otherRotationDenominator) => WideArithmetic.AddSigned832(WideArithmetic.AddSigned832(
            WideArithmetic.MultiplySigned576ToSigned832(axis.X, WideArithmetic.MultiplySigned320(point.X, otherRotationDenominator)),
            WideArithmetic.MultiplySigned576ToSigned832(axis.Y, WideArithmetic.MultiplySigned320(point.Y, otherRotationDenominator))),
            WideArithmetic.MultiplySigned576ToSigned832(axis.Z, WideArithmetic.MultiplySigned320(point.Z, otherRotationDenominator)));
}
