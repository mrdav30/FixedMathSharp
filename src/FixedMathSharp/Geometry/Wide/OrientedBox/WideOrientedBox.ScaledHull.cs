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
    {
        WideDirection3d axis = penetration.Negate ? -penetration.Axis : penetration.Axis;
        Vector3d normal = admissionOnly ? default : axis.GetNormal(false);
        bool clamped = false;
        Fixed64 depth = admissionOnly ? default : penetration.GetRoundedDepth(out clamped);
        int count = 0;
        for (int side = 0; side < 2; side++)
        {
            bool reverse = side != 0;
            ref readonly ScaledRigidFrame3d sourceFrame = ref (reverse ? ref secondFrame : ref firstFrame);
            ref readonly ScaledRigidFrame3d targetFrame = ref (reverse ? ref firstFrame : ref secondFrame);
            Vector3d sourceOrigin = reverse ? secondAnchorOrigin : firstAnchorOrigin;
            Vector3d targetOrigin = reverse ? firstAnchorOrigin : secondAnchorOrigin;
            ReadOnlySpan<Vector3d> sourcePoints = reverse ? secondPoints : firstPoints;
            ReadOnlySpan<Vector3d> targetPoints = reverse ? firstPoints : secondPoints;
            ReadOnlySpan<int> sourceEdges = reverse ? secondEdges : firstEdges;
            ReadOnlySpan<int> targetEdges = reverse ? firstEdges : secondEdges;
            ReadOnlySpan<int> targetTriangles = reverse ? firstTriangles : secondTriangles;
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
            if (!reverse && supportVertices != 0 && supportVertices == admittedVertices) return count;
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
                            if (!WideTriangleRelations.TryGetProjectedSegmentFaceAnchors(face, targetFrame, start, end, n, d,
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
        }
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

    private static bool HasScaledHullEdge(int first, int second, ReadOnlySpan<int> edges)
    {
        for (int edge = 0; edge < edges.Length; edge += 2)
            if (edges[edge] == first && edges[edge + 1] == second || edges[edge] == second && edges[edge + 1] == first) return true;
        return false;
    }

    private static Signed832 GetScaledHullSupport(in ScaledRigidFrame3d frame, ReadOnlySpan<Vector3d> points,
        in WideDirection3d axis, bool maximize)
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

    private static bool IsScaledHullSupport(in RationalPoint3d point, in WideDirection3d axis, Signed832 support) =>
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
    {
        best = default;
        bool hasAxis = false;
        Signed576 commonD = WideArithmetic.MultiplySigned320(firstFrame.Denominator, secondFrame.RotationDenominator);
        for (int side = 0; side < 2; side++)
        {
            ref readonly ScaledRigidFrame3d frame = ref (side == 0 ? ref firstFrame : ref secondFrame);
            ReadOnlySpan<Vector3d> points = side == 0 ? firstPoints : secondPoints;
            ReadOnlySpan<int> triangles = side == 0 ? firstTriangles : secondTriangles;
            ReadOnlySpan<int> edges = side == 0 ? firstEdges : secondEdges;
            WideDirection3d planarNormal = default;
            for (int triangle = 0; triangle < triangles.Length; triangle += 3)
            {
                WideDirection3d local = GetScaledHullLocalFace(frame, points, triangles, triangle);
                if (local.IsZero) continue;
                bool duplicate = false;
                for (int previous = 0; skipParallelDirections && previous < triangle; previous += 3)
                    if (WideDirection3d.Cross(local, GetScaledHullLocalFace(frame, points, triangles, previous)).IsZero)
                    { duplicate = true; break; }
                if (duplicate) continue;
                planarNormal = local;
                if (!TryKeepScaledHullAxis(local.Transform(frame.GeometryBasis),
                        firstFrame, firstPoints, secondFrame, secondPoints, commonD, ref best, ref hasAxis, admission))
                { best = default; return false; }
            }
            if (!planarPair) continue;
            for (int edge = 0; edge < edges.Length; edge += 2)
            {
                if (skipParallelDirections && !IsFirstHullEdgeDirection(points, edges, edge)) continue;
                // Cross before rotation: 218+109+66 < 394 direction bits.
                // Crossing the world normal and edge would retain a redundant
                // quaternion denominator and needlessly exceed the square bank.
                WideDirection3d local = WideDirection3d.Cross(planarNormal,
                    WideDirection3d.GetLocalEdge(frame, points[edges[edge]], points[edges[edge + 1]]));
                if (!TryKeepScaledHullAxis(local.Transform(frame.GeometryBasis),
                        firstFrame, firstPoints, secondFrame, secondPoints, commonD, ref best, ref hasAxis, admission))
                { best = default; return false; }
            }
        }
        for (int firstEdge = 0; firstEdge < firstEdges.Length; firstEdge += 2)
        {
            if (skipParallelDirections && !IsFirstHullEdgeDirection(firstPoints, firstEdges, firstEdge)) continue;
            WideDirection3d a = WideDirection3d.GetLocalEdge(firstFrame,
                firstPoints[firstEdges[firstEdge]], firstPoints[firstEdges[firstEdge + 1]]).Transform(firstFrame.GeometryBasis);
            for (int secondEdge = 0; secondEdge < secondEdges.Length; secondEdge += 2)
            {
                if (skipParallelDirections && !IsFirstHullEdgeDirection(secondPoints, secondEdges, secondEdge)) continue;
                WideDirection3d b = WideDirection3d.GetLocalEdge(secondFrame,
                    secondPoints[secondEdges[secondEdge]], secondPoints[secondEdges[secondEdge + 1]]).Transform(secondFrame.GeometryBasis);
                if (!TryKeepScaledHullAxis(WideDirection3d.Cross(a, b), firstFrame, firstPoints,
                        secondFrame, secondPoints, commonD, ref best, ref hasAxis, admission))
                { best = default; return false; }
            }
        }
        return admission == null ? best.HasValue : hasAxis;
    }

    private static WideDirection3d GetScaledHullLocalFace(in ScaledRigidFrame3d frame,
        ReadOnlySpan<Vector3d> points, ReadOnlySpan<int> triangles, int index) => WideDirection3d.Cross(
            WideDirection3d.GetLocalEdge(frame, points[triangles[index]], points[triangles[index + 1]]),
            WideDirection3d.GetLocalEdge(frame, points[triangles[index]], points[triangles[index + 2]]));

    private static bool TryKeepScaledHullAxis(in WideDirection3d axis,
        in ScaledRigidFrame3d firstFrame, ReadOnlySpan<Vector3d> firstPoints,
        in ScaledRigidFrame3d secondFrame, ReadOnlySpan<Vector3d> secondPoints, Signed576 commonD,
        ref WideScaledHullPenetration best, ref bool hasAxis, ScaledHullPenetrationAdmission? admission)
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

    private static void GetScaledHullProjection(in ScaledRigidFrame3d frame, ReadOnlySpan<Vector3d> points,
        in WideDirection3d axis, Signed192 otherRotationDenominator, out Signed832 minimum, out Signed832 maximum)
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
