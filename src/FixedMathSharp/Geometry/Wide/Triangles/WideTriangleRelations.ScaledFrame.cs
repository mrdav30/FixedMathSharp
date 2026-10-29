//=======================================================================
// WideTriangleRelations.ScaledFrame.cs
//=======================================================================
// MIT License, Copyright (c) 2024-present David Oravsky (mrdav30)
// See LICENSE file in the project root for full license information.
//=======================================================================
using System;

namespace FixedMathSharp.Geometry;

/// <content>Finite triangle predicates and final witnesses over exact authored scaled frames.</content>
internal static partial class WideTriangleRelations
{
    internal static bool TryGetClosestNonparallelSegmentParameters(in RationalPoint3d firstStart,
        in RationalPoint3d firstEnd, in RationalPoint3d secondStart, in RationalPoint3d secondEnd,
        ContactQuadratic firstN, ContactQuadratic firstD, ContactQuadratic secondN, ContactQuadratic secondD)
    {
        Signed192 firstRotationD = WideArithmetic.TruncateSigned320By64(firstStart.Denominator),
            secondRotationD = WideArithmetic.TruncateSigned320By64(secondStart.Denominator);
        WideDirection3d a = Common(firstStart, secondRotationD), b = Common(firstEnd, secondRotationD),
            c = Common(secondStart, firstRotationD), d = Common(secondEnd, firstRotationD);
        WideDirection3d e = Difference(b, a), f = Difference(d, c), r = Difference(a, c);
        // All points share Q^2*R1*R2. The line Gram minors need at most
        // twice a full 832-bit coefficient; keep them borrowed until selection.
        const int words = 27;
        Span<ulong> values = stackalloc ulong[14 * words]; Span<int> signs = stackalloc int[14];
        var aa = ContactQuadratic.At(values, signs, 0, words);
        var ab = ContactQuadratic.At(values, signs, 1, words);
        var bb = ContactQuadratic.At(values, signs, 2, words);
        var ar = ContactQuadratic.At(values, signs, 3, words);
        var br = ContactQuadratic.At(values, signs, 4, words);
        var one = ContactQuadratic.At(values, signs, 5, words);
        var two = ContactQuadratic.At(values, signs, 6, words);
        aa.Set(WideDirection3d.Dot(e, e)); ab.Set(WideDirection3d.Dot(e, f)); bb.Set(WideDirection3d.Dot(f, f));
        ar.Set(WideDirection3d.Dot(e, r)); br.Set(WideDirection3d.Dot(f, r));
        ContactQuadratic.Multiply(aa, bb, ReadOnlySpan<ulong>.Empty, one);
        ContactQuadratic.Multiply(ab, ab, ReadOnlySpan<ulong>.Empty, two); one.Add(two, -1);
        if (one.Signs[0] <= 0) return false;
        one.CopyTo(firstD); one.CopyTo(secondD);
        ContactQuadratic.Multiply(ab, br, ReadOnlySpan<ulong>.Empty, one);
        ContactQuadratic.Multiply(bb, ar, ReadOnlySpan<ulong>.Empty, two); one.Add(two, -1);
        if (one.Signs[0] < 0) return false;
        one.CopyTo(firstN); firstD.CopyTo(two); two.Add(firstN, -1);
        if (two.Signs[0] < 0) return false;
        ContactQuadratic.Multiply(aa, br, ReadOnlySpan<ulong>.Empty, one);
        ContactQuadratic.Multiply(ab, ar, ReadOnlySpan<ulong>.Empty, two); one.Add(two, -1);
        if (one.Signs[0] < 0) return false;
        one.CopyTo(secondN); secondD.CopyTo(two); two.Add(secondN, -1);
        return two.Signs[0] >= 0;

        static WideDirection3d Common(in RationalPoint3d point, Signed192 otherD) => new(
            WideArithmetic.MultiplySigned320(point.X, otherD), WideArithmetic.MultiplySigned320(point.Y, otherD),
            WideArithmetic.MultiplySigned320(point.Z, otherD));
        static WideDirection3d Difference(in WideDirection3d end, in WideDirection3d start) => new(
            WideArithmetic.SubtractSigned576(end.X, start.X), WideArithmetic.SubtractSigned576(end.Y, start.Y),
            WideArithmetic.SubtractSigned576(end.Z, start.Z));
    }

    internal static bool TryGetSegmentPointAnchor(in ScaledRigidFrame3d frame, in FixedSegment segment,
        scoped ContactQuadratic n, scoped ContactQuadratic d, Vector3d origin, out FixedPointAnchor anchor) =>
        TryGetSegmentPointAnchor<ScaledRigidFrame3d>(frame, segment, n, d, origin, out anchor);

    internal static bool TryGetSegmentPointAnchor<TFrame>(in TFrame frame, in FixedSegment segment,
        scoped ContactQuadratic n, scoped ContactQuadratic d, Vector3d origin, out FixedPointAnchor anchor)
        where TFrame : struct, IForwardHullFrame3d =>
        TryGetSegmentPointAnchor(frame.GetPoint(segment.Start), frame.GetPoint(segment.End), n, d, origin, out anchor);

    private static bool TryGetSegmentPointAnchor(in RationalPoint3d a, in RationalPoint3d b,
        scoped ContactQuadratic n, scoped ContactQuadratic d, Vector3d origin, out FixedPointAnchor anchor)
    {
        anchor = default;
        System.Diagnostics.Debug.Assert(a.Denominator.Equals(b.Denominator));
        a.GetLocalPointNumerators(origin, FixedQuaternion.Identity, out Signed576 ax, out Signed576 ay, out Signed576 az, out _);
        b.GetLocalPointNumerators(origin, FixedQuaternion.Identity, out Signed576 bx, out Signed576 by, out Signed576 bz, out _);
        int words = Math.Max(ContactQuadratic.ActiveWords(n), ContactQuadratic.ActiveWords(d)) + 7;
        Span<ulong> values = stackalloc ulong[8 * words]; Span<int> signs = stackalloc int[8];
        var remaining = ContactQuadratic.At(values, signs, 0, words);
        var numerator = ContactQuadratic.At(values, signs, 1, words);
        var term = ContactQuadratic.At(values, signs, 2, words);
        var denominator = ContactQuadratic.At(values, signs, 3, words);
        d.CopyTo(remaining); remaining.Add(n, -1);
        ContactQuadratic.Scale(d, Signed576.ExtendValue(a.Denominator), denominator);
        Span<Signed576> first = stackalloc Signed576[3] { ax, ay, az }, second = stackalloc Signed576[3] { bx, by, bz };
        Vector3d local = default;
        for (int coordinate = 0; coordinate < 3; coordinate++)
        {
            EvaluateScaledEdgePoint(first[coordinate], second[coordinate], remaining, n, numerator, term);
            if (!Fixed64.TryGetSignedRawRatio(numerator.Rational, denominator.Rational,
                    numerator.Signs[0] < 0, out Fixed64 scalar)) return false;
            local[coordinate] = scalar;
        }
        anchor = new FixedPointAnchor(origin, FixedQuaternion.Identity, local);
        return true;
    }

    internal static int LocateEdgePoint(in FixedSegment source, in ScaledRigidFrame3d sourceFrame,
        Signed832 n, Signed832 d, in FixedSegment target, in ScaledRigidFrame3d targetFrame,
        in RationalSegmentInterval targetInterval = default)
    {
        Span<ulong> values = stackalloc ulong[52]; Span<int> signs = stackalloc int[4];
        var numerator = ContactQuadratic.At(values, signs, 0, 13);
        var denominator = ContactQuadratic.At(values, signs, 1, 13);
        numerator.Set(n); denominator.Set(d);
        return LocateEdgePoint(source, sourceFrame, numerator, denominator, target, targetFrame, targetInterval);
    }

    /// <summary>Locates a retained rational edge parameter without narrowing its transient numerator or denominator.</summary>
    internal static int LocateEdgePoint(in FixedSegment source, in ScaledRigidFrame3d sourceFrame,
        scoped ContactQuadratic n, scoped ContactQuadratic d, in FixedSegment target, in ScaledRigidFrame3d targetFrame,
        in RationalSegmentInterval targetInterval = default) =>
        LocateFrameEdgePoint(source, sourceFrame, n, d, target, targetFrame, targetInterval);

    /// <summary>Locates an exact retained point across forward-only scalar and authored charts.</summary>
    internal static int LocateFrameEdgePoint<TSource, TTarget>(in FixedSegment source, in TSource sourceFrame,
        scoped ContactQuadratic n, scoped ContactQuadratic d, in FixedSegment target, in TTarget targetFrame,
        in RationalSegmentInterval targetInterval = default)
        where TSource : struct, IForwardHullFrame3d
        where TTarget : struct, IForwardHullFrame3d
    {
        RationalPoint3d a = sourceFrame.GetPoint(source.Start), b = sourceFrame.GetPoint(source.End);
        return LocateFrameEdgePoint(a, b, n, d, target, targetFrame, targetInterval);
    }

    /// <summary>Locates an exact anchored point without rounding its retained feature term.</summary>
    internal static int LocatePointOnFrameEdge<TFrame>(in RationalPoint3d point,
        in FixedSegment target, in TFrame frame, in RationalSegmentInterval interval = default)
        where TFrame : struct, IForwardHullFrame3d
    {
        Span<ulong> values = stackalloc ulong[52]; Span<int> signs = stackalloc int[4];
        var n = ContactQuadratic.At(values, signs, 0, 13); var d = ContactQuadratic.At(values, signs, 1, 13);
        n.Clear(); d.Set(Signed832.ExtendValue(Signed192.Signed(1)));
        return LocateFrameEdgePoint(point, point, n, d, target, frame, interval);
    }

    private static int LocateFrameEdgePoint<TTarget>(in RationalPoint3d a, in RationalPoint3d b,
        scoped ContactQuadratic n, scoped ContactQuadratic d, in FixedSegment target, in TTarget targetFrame,
        in RationalSegmentInterval targetInterval) where TTarget : struct, IForwardHullFrame3d
    {
        RationalPoint3d targetA = targetFrame.GetPoint(target.Start), targetB = targetFrame.GetPoint(target.End);
        WideDirection3d first = GetScaledPointDifference(a, targetA, targetFrame.RotationDenominator),
            second = GetScaledPointDifference(b, targetA, targetFrame.RotationDenominator);
        WideDirection3d edge = targetFrame.GetEdgeDirection(target.Start, target.End);
        int words = Math.Max(ContactQuadratic.ActiveWords(n), ContactQuadratic.ActiveWords(d)) + 11;
        Span<ulong> values = stackalloc ulong[12 * words]; Span<int> signs = stackalloc int[12];
        var remaining = ContactQuadratic.At(values, signs, 0, words);
        var value = ContactQuadratic.At(values, signs, 1, words);
        var term = ContactQuadratic.At(values, signs, 2, words);
        var start = ContactQuadratic.At(values, signs, 3, words);
        var end = ContactQuadratic.At(values, signs, 4, words);
        var edgeD = ContactQuadratic.At(values, signs, 5, words);
        d.CopyTo(remaining); remaining.Add(n, -1);
        WideDirection3d firstCross = WideDirection3d.Cross(edge, first), secondCross = WideDirection3d.Cross(edge, second);
        Span<Signed576> crossA = stackalloc Signed576[3] { firstCross.X, firstCross.Y, firstCross.Z };
        Span<Signed576> crossB = stackalloc Signed576[3] { secondCross.X, secondCross.Y, secondCross.Z };
        for (int coordinate = 0; coordinate < 3; coordinate++)
        {
            EvaluateScaledEdgePoint(crossA[coordinate], crossB[coordinate], remaining, n, value, term);
            if (value.Signs[0] != 0) return -1;
        }
        if (edge.IsZero)
        {
            Span<Signed576> firstCoordinates = stackalloc Signed576[3] { first.X, first.Y, first.Z };
            Span<Signed576> secondCoordinates = stackalloc Signed576[3] { second.X, second.Y, second.Z };
            for (int coordinate = 0; coordinate < 3; coordinate++)
            {
                EvaluateScaledEdgePoint(firstCoordinates[coordinate], secondCoordinates[coordinate], remaining, n, value, term);
                if (value.Signs[0] != 0) return -1;
            }
            return 0;
        }
        int axis = !edge.X.IsZero ? 0 : !edge.Y.IsZero ? 1 : 2;
        Signed576 firstValue = axis == 0 ? first.X : axis == 1 ? first.Y : first.Z;
        Signed576 secondValue = axis == 0 ? second.X : axis == 1 ? second.Y : second.Z;
        EvaluateScaledEdgePoint(firstValue, secondValue, remaining, n, start, term);
        first = GetScaledPointDifference(a, targetB, targetFrame.RotationDenominator);
        second = GetScaledPointDifference(b, targetB, targetFrame.RotationDenominator);
        firstValue = axis == 0 ? first.X : axis == 1 ? first.Y : first.Z;
        secondValue = axis == 0 ? second.X : axis == 1 ? second.Y : second.Z;
        EvaluateScaledEdgePoint(firstValue, secondValue, remaining, n, end, term);
        int direction = (axis == 0 ? edge.X : axis == 1 ? edge.Y : edge.Z).Sign;
        int startSign = start.Signs[0], endSign = end.Signs[0];
        if (startSign != 0 && endSign != 0 && (startSign != direction || endSign != -direction)) return -1;
        if (targetInterval.IsClipped)
        {
            start.CopyTo(edgeD); edgeD.Add(end, -1);
            if (direction < 0) { start.MultiplySign(-1); edgeD.MultiplySign(-1); }
            if (!targetInterval.Contains(start, edgeD, ReadOnlySpan<ulong>.Empty)) return -1;
        }
        return startSign == 0 ? 0 : endSign == 0 ? 2 : 1;
    }

    private static void EvaluateScaledEdgePoint(Signed576 first, Signed576 second,
        ContactQuadratic remaining, ContactQuadratic n, ContactQuadratic value, ContactQuadratic term)
    {
        ContactQuadratic.Scale(remaining, first, value); ContactQuadratic.Scale(n, second, term); value.Add(term);
    }

    /// <summary>
    /// Clips an authored edge against an independently scaled finite triangle.
    /// The side bits and closed unit interval match the rigid coverage contract.
    /// </summary>
    internal static bool TryGetEdgeCoverage(in FixedTriangle source, in ScaledRigidFrame3d sourceFrame,
        in FixedTriangle target, in ScaledRigidFrame3d targetFrame, out Signed832 lowerN,
        out Signed832 lowerD, out Signed832 upperN, out Signed832 upperD, out int sides)
    {
        lowerN = default; lowerD = upperN = upperD = Signed832.ExtendValue(Signed192.One); sides = 0;
        if (sourceFrame.HasSameMapping(targetFrame))
        {
            // An invertible affine map preserves incidence, filled sides and
            // the original edge parameter. Remove it before clipping at all.
            if (!TryGetEdgeCoverage(source, Vector3d.Zero, FixedQuaternion.Identity, target,
                    Vector3d.Zero, FixedQuaternion.Identity, out RationalSegmentInterval interval, out sides)) return false;
            if (interval.IsClipped)
            {
                lowerN = Signed832.ExtendValue(Signed576.ExtendValue(interval.LowerNumerator));
                lowerD = Signed832.ExtendValue(Signed576.ExtendValue(interval.LowerDenominator));
                upperN = Signed832.ExtendValue(Signed576.ExtendValue(interval.UpperNumerator));
                upperD = Signed832.ExtendValue(Signed576.ExtendValue(interval.UpperDenominator));
            }
            return true;
        }
        WideDirection3d sourceLocalNormal = GetScaledTriangleLocalNormal(source, sourceFrame);
        WideDirection3d targetLocalNormal = GetScaledTriangleLocalNormal(target, targetFrame);
        if (sourceLocalNormal.IsZero || targetLocalNormal.IsZero) return false;
        WideDirection3d targetNormal = targetLocalNormal.Transform(targetFrame.GeometryBasis);
        RationalPoint3d a = sourceFrame.GetPoint(source.A), b = sourceFrame.GetPoint(source.B),
            c = sourceFrame.GetPoint(source.C), targetOrigin = targetFrame.GetPoint(target.A);
        if (!DotScaledDirections(targetNormal, GetScaledPointDifference(a, targetOrigin, targetFrame.RotationDenominator)).IsZero
            || !DotScaledDirections(targetNormal, GetScaledPointDifference(b, targetOrigin, targetFrame.RotationDenominator)).IsZero)
            return false;
        bool folded = !DotScaledDirections(targetNormal,
            GetScaledPointDifference(c, targetOrigin, targetFrame.RotationDenominator)).IsZero;
        if (!TryGetProjectedSegmentInterval(target, targetFrame, a, b, out lowerN, out lowerD, out upperN, out upperD))
            return false;
        if (folded) { sides = 4; return true; }
        WideDirection3d inward = WideDirection3d.Cross(sourceLocalNormal,
            WideDirection3d.GetLocalEdge(sourceFrame, source.A, source.B)).Transform(sourceFrame.GeometryBasis);
        Span<Vector3d> vertices = stackalloc Vector3d[3] { target.A, target.B, target.C };
        foreach (Vector3d vertex in vertices)
        {
            int side = DotScaledDirections(inward, GetScaledPointDifference(targetFrame.GetPoint(vertex), a,
                sourceFrame.RotationDenominator)).Sign;
            if (side > 0) sides |= 1;
            if (side < 0) sides |= 2;
        }
        return true;
    }

    /// <summary>Preserves the triangle pair's SAT and successive finite closest-witness contract.</summary>
    internal static bool TryGetContact(in FixedTriangle first, in ScaledRigidFrame3d firstFrame, Vector3d firstOrigin,
        in FixedTriangle second, in ScaledRigidFrame3d secondFrame, Vector3d secondOrigin, out FixedContactAnchors contact)
    {
        if (firstFrame.IsRigid && secondFrame.IsRigid)
            return TryGetContact(first,firstFrame.RigidOrigin,firstFrame.Rotation,
                second,secondFrame.RigidOrigin,secondFrame.Rotation,out contact);
        contact=default;
        if (GetScaledTriangleLocalNormal(first,firstFrame).IsZero
            || GetScaledTriangleLocalNormal(second,secondFrame).IsZero) return false;
        Span<Vector3d> firstPoints=stackalloc Vector3d[3]{first.A,first.B,first.C};
        Span<Vector3d> secondPoints=stackalloc Vector3d[3]{second.A,second.B,second.C};
        ReadOnlySpan<int> faces=stackalloc int[3]{0,1,2},edges=stackalloc int[6]{0,1,1,2,2,0};
        if(!WideOrientedBox.TryGetConvexHullPenetration(firstFrame,firstPoints,faces,edges,
                secondFrame,secondPoints,faces,edges,out WideScaledHullPenetration penetration,planarPair:true)
            ||!TryGetClosestPointAnchor(first,firstFrame,secondFrame.GetPoint(second.Centroid),firstOrigin,out FixedPointAnchor a)
            ||!TryGetClosestPointAnchor(second,secondFrame,RationalPoint3d.FromAnchor(a),secondOrigin,out FixedPointAnchor b))return false;
        Fixed64 depth=penetration.GetRoundedDepth(out bool clamped);
        contact=new FixedContactAnchors(a,b,penetration.GetNormal(),depth,clamped);return true;
    }

    /// <summary>Tests the finite orthogonal projection before rounding any vertex or witness.</summary>
    internal static bool ContainsProjection(in FixedTriangle triangle, in ScaledRigidFrame3d frame,
        in RationalPoint3d point) =>
        ContainsProjection<ScaledRigidFrame3d>(triangle, frame, point);

    internal static bool ContainsProjection<TFrame>(in FixedTriangle triangle, in TFrame frame,
        in RationalPoint3d point)
        where TFrame : struct, IForwardHullFrame3d
    {
        Span<Signed832> values = stackalloc Signed832[3];
        return GetScaledTriangleHalfPlanes(triangle, frame, point, values)
            && values[0].Sign >= 0 && values[1].Sign >= 0 && values[2].Sign >= 0;
    }

    /// <summary>
    /// Projects onto a finite exact face and rounds only the final witness in a
    /// caller-selected translated chart. Returns false outside the finite face
    /// or when the translated witness cannot fit a scalar coordinate.
    /// </summary>
    internal static bool TryGetProjectedPointAnchor(in FixedTriangle triangle, in ScaledRigidFrame3d frame,
        in RationalPoint3d point, Vector3d materializationOrigin, out FixedPointAnchor anchor) =>
        TryGetProjectedPointAnchor<ScaledRigidFrame3d>(triangle, frame, point, materializationOrigin, out anchor);

    internal static bool TryGetProjectedPointAnchor<TFrame>(in FixedTriangle triangle, in TFrame frame,
        in RationalPoint3d point, Vector3d materializationOrigin, out FixedPointAnchor anchor)
        where TFrame : struct, IForwardHullFrame3d =>
        TryGetProjectedPointAnchor(triangle, frame, point, materializationOrigin, default, default, out anchor);

    internal static bool TryGetProjectedPlanePointAnchor<TFrame>(in FixedTriangle plane, in TFrame frame,
        in RationalPoint3d point, Vector3d materializationOrigin, out FixedPointAnchor anchor)
        where TFrame : struct, IForwardHullFrame3d =>
        TryGetProjectedPointAnchor(plane, frame, point, materializationOrigin, default, default, out anchor, finite: false);

    private static bool TryGetProjectedPointAnchor<TFrame>(in FixedTriangle triangle, in TFrame frame,
        in RationalPoint3d point, Vector3d materializationOrigin,
        scoped ContactQuadratic squaredDistanceNumerator, scoped ContactQuadratic squaredDistanceDenominator,
        out FixedPointAnchor anchor, bool finite = true)
        where TFrame : struct, IForwardHullFrame3d
    {
        anchor = default;
        if (finite && !ContainsProjection(triangle, frame, point)) return false;
        WideDirection3d normal = frame.GetFaceNormal(triangle);
        if (normal.IsZero) return false;
        WideDirection3d delta = GetScaledPointDifference(point, frame.GetPoint(triangle.A), frame.RotationDenominator);
        Signed832 plane = DotScaledDirections(normal, delta), squared = normal.SquaredLength;
        Signed576 commonD = WideArithmetic.MultiplySigned320(point.Denominator, frame.RotationDenominator);
        // A projected witness needs <962 numerator bits and <897 denominator
        // bits. Keep those products transient; no wider persistent integer is needed.
        const int words = 16;
        Span<ulong> storage = stackalloc ulong[12 * words]; Span<int> signs = stackalloc int[12];
        ContactQuadratic metric = ContactQuadratic.At(storage, signs, 0, words),
            divisor = ContactQuadratic.At(storage, signs, 1, words),
            denominator = ContactQuadratic.At(storage, signs, 2, words),
            numerator = ContactQuadratic.At(storage, signs, 3, words),
            planeValue = ContactQuadratic.At(storage, signs, 4, words),
            term = ContactQuadratic.At(storage, signs, 5, words);
        metric.Set(squared); divisor.Set(commonD); planeValue.Set(plane);
        if (!squaredDistanceNumerator.Values.IsEmpty)
        {
            ContactQuadratic.Multiply(planeValue, planeValue, ReadOnlySpan<ulong>.Empty, squaredDistanceNumerator);
            ContactQuadratic.Multiply(divisor, divisor, ReadOnlySpan<ulong>.Empty, term);
            ContactQuadratic.Multiply(metric, term, ReadOnlySpan<ulong>.Empty, squaredDistanceDenominator);
        }
        ContactQuadratic.Multiply(metric, divisor, ReadOnlySpan<ulong>.Empty, denominator);
        Vector3d local = default;
        for (int coordinate = 0; coordinate < 3; coordinate++)
        {
            Signed320 raw = coordinate == 0 ? point.X : coordinate == 1 ? point.Y : point.Z;
            Signed576 relative = WideArithmetic.SubtractSigned576(
                WideArithmetic.MultiplySigned320(raw, frame.RotationDenominator),
                WideArithmetic.MultiplySigned576(commonD, Signed192.Raw(materializationOrigin[coordinate])));
            Signed576 component = coordinate == 0 ? normal.X : coordinate == 1 ? normal.Y : normal.Z;
            ContactQuadratic.Scale(metric, relative, numerator);
            ContactQuadratic.Scale(planeValue, component, term); numerator.Add(term, -1);
            if (!Fixed64.TryGetSignedRawRatio(numerator.Rational, denominator.Rational,
                    numerator.Signs[0] < 0, out Fixed64 value)) return false;
            local[coordinate] = value;
        }
        anchor = new FixedPointAnchor(materializationOrigin, FixedQuaternion.Identity, local);
        return true;
    }

    /// <summary>Clips a same-frame rational segment's projection to the exact finite triangle.</summary>
    internal static bool TryGetProjectedSegmentInterval(in FixedTriangle triangle, in ScaledRigidFrame3d frame,
        in RationalPoint3d first, in RationalPoint3d second,
        out Signed832 lowerN, out Signed832 lowerD, out Signed832 upperN, out Signed832 upperD) =>
        TryGetProjectedSegmentInterval<ScaledRigidFrame3d>(triangle, frame, first, second, out lowerN, out lowerD, out upperN, out upperD);

    internal static bool TryGetProjectedSegmentInterval<TFrame>(in FixedTriangle triangle, in TFrame frame,
        in RationalPoint3d first, in RationalPoint3d second,
        out Signed832 lowerN, out Signed832 lowerD, out Signed832 upperN, out Signed832 upperD)
        where TFrame : struct, IForwardHullFrame3d
    {
        System.Diagnostics.Debug.Assert(first.Denominator.Equals(second.Denominator));
        Span<Signed832> start = stackalloc Signed832[3], end = stackalloc Signed832[3];
        lowerN = default; lowerD = upperN = upperD = Signed832.ExtendValue(Signed192.One);
        if (!GetScaledTriangleHalfPlanes(triangle, frame, first, start)) return false;
        // Both points share the same nondegenerate face; only their values differ.
        GetScaledTriangleHalfPlanes(triangle, frame, second, end);
        for (int edge = 0; edge < 3; edge++)
            if (!ClipScaledHalfPlane(start[edge], end[edge], ref lowerN, ref lowerD, ref upperN, ref upperD)) return false;
        return true;
    }

    /// <summary>Clips a segment's orthogonal projection to A+u(B-A)+v(C-A), with retained u and v in [0,1].</summary>
    /// <remarks>
    /// Both source points must share a positive denominator. The four disjoint output
    /// banks require 32 words each: full 832-bit cuts times exact face half-planes
    /// need fewer than 1,667 bits, even when the resulting interval cannot fit
    /// Signed832. Fractions remain borrowed; no endpoint is materialized.
    /// </remarks>
    internal static bool TryGetProjectedRectangleInterval(in FixedTriangle rectangleBasis, in ScaledRigidFrame3d frame,
        in RationalPoint3d first, in RationalPoint3d second, in RationalSegmentInterval tangentClip,
        scoped ContactQuadratic lowerN, scoped ContactQuadratic lowerD,
        scoped ContactQuadratic upperN, scoped ContactQuadratic upperD) =>
        TryGetProjectedRectangleInterval<ScaledRigidFrame3d>(rectangleBasis, frame, first, second, tangentClip, lowerN, lowerD, upperN, upperD);

    internal static bool TryGetProjectedRectangleInterval<TFrame>(in FixedTriangle rectangleBasis, in TFrame frame,
        in RationalPoint3d first, in RationalPoint3d second, in RationalSegmentInterval tangentClip,
        scoped ContactQuadratic lowerN, scoped ContactQuadratic lowerD,
        scoped ContactQuadratic upperN, scoped ContactQuadratic upperD)
        where TFrame : struct, IForwardHullFrame3d
    {
        System.Diagnostics.Debug.Assert(first.Denominator.Equals(second.Denominator));
        Signed832 one = Signed832.ExtendValue(Signed192.One);
        Signed832 cutLowerN = default, cutLowerD = one, cutUpperN = one, cutUpperD = one;
        if (tangentClip.IsClipped)
        {
            cutLowerN = tangentClip.WideLowerNumerator; cutLowerD = tangentClip.WideLowerDenominator;
            cutUpperN = tangentClip.WideUpperNumerator; cutUpperD = tangentClip.WideUpperDenominator;
        }
        lowerN.Clear(); lowerD.Set(one); upperN.Set(one); upperD.Set(one);
        Span<Signed832> start = stackalloc Signed832[3], end = stackalloc Signed832[3];
        if (!GetScaledTriangleHalfPlanes(rectangleBasis, frame, first, start)) return false;
        GetScaledTriangleHalfPlanes(rectangleBasis, frame, second, end);
        const int words = 32;
        Span<ulong> values = stackalloc ulong[12 * words]; Span<int> signs = stackalloc int[12];
        var firstSum = ContactQuadratic.At(values, signs, 0, words);
        var secondSum = ContactQuadratic.At(values, signs, 1, words);
        var firstWall = ContactQuadratic.At(values, signs, 2, words);
        var secondWall = ContactQuadratic.At(values, signs, 3, words);
        var coefficient = ContactQuadratic.At(values, signs, 4, words);
        var term = ContactQuadratic.At(values, signs, 5, words);
        firstSum.Set(start[0]); secondSum.Set(end[0]);
        for (int index = 1; index < 3; index++)
        {
            coefficient.Set(start[index]); firstSum.Add(coefficient);
            coefficient.Set(end[index]); secondSum.Add(coefficient);
        }
        // Half-planes opposite B and C are u and v times their common metric;
        // the rectangle has no triangular u+v<=1 restriction.
        for (int wall = 0; wall < 4; wall++)
        {
            for (int side = 0; side < 2; side++)
            {
                var sum = side == 0 ? firstSum : secondSum;
                var output = side == 0 ? firstWall : secondWall;
                ReadOnlySpan<Signed832> point = side == 0 ? start : end;
                if (wall < 2)
                {
                    Signed832 numerator = wall == 0 ? cutLowerN : cutUpperN;
                    Signed832 denominator = wall == 0 ? cutLowerD : cutUpperD;
                    coefficient.Set(point[2]); ContactQuadratic.Scale(coefficient, denominator, output);
                    ContactQuadratic.Scale(sum, numerator, term); output.Add(term, -1);
                    if (wall != 0) output.MultiplySign(-1);
                }
                else if (wall == 2) output.Set(point[0]);
                else { sum.CopyTo(output); coefficient.Set(point[0]); output.Add(coefficient, -1); }
            }
            if (!ClipProjectedRectangleWall(firstWall, secondWall, lowerN, lowerD, upperN, upperD)) return false;
        }
        return true;
    }

    private static bool ClipProjectedRectangleWall(scoped ContactQuadratic first, scoped ContactQuadratic second,
        scoped ContactQuadratic lowerN, scoped ContactQuadratic lowerD,
        scoped ContactQuadratic upperN, scoped ContactQuadratic upperD)
    {
        int firstSign = first.Signs[0], secondSign = second.Signs[0];
        if (firstSign >= 0 && secondSign >= 0) return true;
        if (firstSign < 0 && secondSign < 0) return false;
        const int words = 32;
        Span<ulong> values = stackalloc ulong[4 * words]; Span<int> signs = stackalloc int[4];
        var numerator = ContactQuadratic.At(values, signs, 0, words);
        var denominator = ContactQuadratic.At(values, signs, 1, words);
        first.CopyTo(numerator); first.CopyTo(denominator); denominator.Add(second, -1);
        if (denominator.Signs[0] < 0) { numerator.MultiplySign(-1); denominator.MultiplySign(-1); }
        if (firstSign < 0)
        {
            if (ContactQuadratic.CompareRatios(numerator, denominator, ReadOnlySpan<ulong>.Empty,
                    lowerN, lowerD, ReadOnlySpan<ulong>.Empty) > 0)
            { numerator.CopyTo(lowerN); denominator.CopyTo(lowerD); }
        }
        else if (ContactQuadratic.CompareRatios(numerator, denominator, ReadOnlySpan<ulong>.Empty,
                upperN, upperD, ReadOnlySpan<ulong>.Empty) < 0)
        { numerator.CopyTo(upperN); denominator.CopyTo(upperD); }
        return ContactQuadratic.CompareRatios(lowerN, lowerD, ReadOnlySpan<ulong>.Empty,
            upperN, upperD, ReadOnlySpan<ulong>.Empty) <= 0;
    }

    internal static bool TryGetProjectedSegmentFaceAnchors(in FixedTriangle triangle, in ScaledRigidFrame3d frame,
        in RationalPoint3d first, in RationalPoint3d second, Signed832 n, Signed832 d,
        Vector3d sourceOrigin, Vector3d targetOrigin, int physicalBoundaryMask,
        out FixedPointAnchor source, out FixedPointAnchor target,
        bool requirePhysicalTargetBoundary = true, bool materialize = true) =>
        TryGetProjectedFrameSegmentFaceAnchors<ScaledRigidFrame3d>(triangle, frame, first, second, n, d, sourceOrigin, targetOrigin, physicalBoundaryMask, out source, out target, requirePhysicalTargetBoundary, materialize);

    internal static bool TryGetProjectedFrameSegmentFaceAnchors<TFrame>(in FixedTriangle triangle, in TFrame frame,
        in RationalPoint3d first, in RationalPoint3d second, Signed832 n, Signed832 d,
        Vector3d sourceOrigin, Vector3d targetOrigin, int physicalBoundaryMask,
        out FixedPointAnchor source, out FixedPointAnchor target,
        bool requirePhysicalTargetBoundary = true, bool materialize = true,
        scoped ContactQuadratic wideN = default, scoped ContactQuadratic wideD = default, bool rectangle = false)
        where TFrame : struct, IForwardHullFrame3d
    {
        source = target = default;
        int words = rectangle ? 48 : 32;
        Span<ulong> values = stackalloc ulong[20 * words]; Span<int> signs = stackalloc int[20];
        var sx = ContactQuadratic.At(values, signs, 0, words);
        var sy = ContactQuadratic.At(values, signs, 1, words);
        var sz = ContactQuadratic.At(values, signs, 2, words);
        var sd = ContactQuadratic.At(values, signs, 3, words);
        var tx = ContactQuadratic.At(values, signs, 4, words);
        var ty = ContactQuadratic.At(values, signs, 5, words);
        var tz = ContactQuadratic.At(values, signs, 6, words);
        var td = ContactQuadratic.At(values, signs, 7, words);
        if (!TryGetProjectedSegmentFaceWitness(triangle, frame, first, second, n, d,
                sx, sy, sz, sd, tx, ty, tz, td, out int edgeMask, wideN, wideD, rectangle)
            || requirePhysicalTargetBoundary && (edgeMask & physicalBoundaryMask) == 0) return false;
        if (!materialize) return true;
        var relative = ContactQuadratic.At(values, signs, 8, words);
        var term = ContactQuadratic.At(values, signs, 9, words);
        Vector3d sourcePoint = default, targetPoint = default;
        for (int coordinate = 0; coordinate < 3; coordinate++)
        {
            var sourceCoordinate = coordinate == 0 ? sx : coordinate == 1 ? sy : sz;
            var targetCoordinate = coordinate == 0 ? tx : coordinate == 1 ? ty : tz;
            sourceCoordinate.CopyTo(relative);
            ContactQuadratic.Scale(sd, Signed576.ExtendValue(Signed192.Raw(sourceOrigin[coordinate])), term); relative.Add(term, -1);
            if (!Fixed64.TryGetSignedRawRatio(relative.Rational, sd.Rational,
                    relative.Signs[0] < 0, out Fixed64 scalar)) return false;
            sourcePoint[coordinate] = scalar;
            targetCoordinate.CopyTo(relative);
            ContactQuadratic.Scale(td, Signed576.ExtendValue(Signed192.Raw(targetOrigin[coordinate])), term); relative.Add(term, -1);
            if (!Fixed64.TryGetSignedRawRatio(relative.Rational, td.Rational,
                    relative.Signs[0] < 0, out scalar)) return false;
            targetPoint[coordinate] = scalar;
        }
        source = new FixedPointAnchor(sourceOrigin, FixedQuaternion.Identity, sourcePoint);
        target = new FixedPointAnchor(targetOrigin, FixedQuaternion.Identity, targetPoint);
        return true;
    }

    /// <summary>Materializes a rectangle projection from an already clipped borrowed unit parameter.</summary>
    internal static bool TryGetProjectedRectangleFaceAnchors<TFrame>(in FixedTriangle rectangleBasis, in TFrame frame,
        in RationalPoint3d first, in RationalPoint3d second, scoped ContactQuadratic n, scoped ContactQuadratic d,
        Vector3d sourceOrigin, Vector3d targetOrigin, out FixedPointAnchor source, out FixedPointAnchor target)
        where TFrame : struct, IForwardHullFrame3d =>
        TryGetProjectedFrameSegmentFaceAnchors(rectangleBasis, frame, first, second, default, default,
            sourceOrigin, targetOrigin, 0, out source, out target, requirePhysicalTargetBoundary: false,
            wideN: n, wideD: d, rectangle: true);

    /// <summary>
    /// Retains the exact source point and its finite face projection as borrowed
    /// world-coordinate ratios. The zero barycentric weights identify opposite
    /// triangle edges; no coordinate or separation has been rounded.
    /// </summary>
    private static bool TryGetProjectedSegmentFaceWitness<TFrame>(in FixedTriangle triangle, in TFrame frame,
        in RationalPoint3d first, in RationalPoint3d second, Signed832 n, Signed832 d,
        scoped ContactQuadratic sourceX, scoped ContactQuadratic sourceY, scoped ContactQuadratic sourceZ,
        scoped ContactQuadratic sourceD, scoped ContactQuadratic targetX, scoped ContactQuadratic targetY,
        scoped ContactQuadratic targetZ, scoped ContactQuadratic targetD, out int edgeMask,
        scoped ContactQuadratic wideN = default, scoped ContactQuadratic wideD = default, bool rectangle = false)
        where TFrame : struct, IForwardHullFrame3d
    {
        edgeMask = 0;
        Span<Signed832> start = stackalloc Signed832[3], end = stackalloc Signed832[3];
        if (!GetScaledTriangleHalfPlanes(triangle, frame, first, start)) return false;
        GetScaledTriangleHalfPlanes(triangle, frame, second, end);
        // Compact triangle parameters retain the original 2048-bit banks.
        // Full rectangle cuts need <1667 parameter bits; adding an 832-bit
        // half-plane and 320-bit coordinate remains below 3072 bits.
        int words = rectangle ? 48 : 32;
        Span<ulong> values = stackalloc ulong[20 * words]; Span<int> signs = stackalloc int[20];
        var parameter = ContactQuadratic.At(values, signs, 0, words);
        var divisor = ContactQuadratic.At(values, signs, 1, words);
        var remaining = ContactQuadratic.At(values, signs, 2, words);
        var sum = ContactQuadratic.At(values, signs, 6, words);
        var term = ContactQuadratic.At(values, signs, 7, words);
        var coefficient = ContactQuadratic.At(values, signs, 8, words);
        if (!wideD.Values.IsEmpty) { wideN.CopyTo(parameter); wideD.CopyTo(divisor); }
        else { parameter.Set(n); divisor.Set(d); }
        divisor.CopyTo(remaining); remaining.Add(parameter, -1); sum.Clear();
        for (int corner = 0; corner < 3; corner++)
        {
            int edge = (corner + 1) % 3;
            var barycentric = ContactQuadratic.At(values, signs, 3 + corner, words);
            coefficient.Set(start[edge]); ContactQuadratic.Multiply(coefficient, remaining, ReadOnlySpan<ulong>.Empty, barycentric);
            coefficient.Set(end[edge]); ContactQuadratic.Multiply(coefficient, parameter, ReadOnlySpan<ulong>.Empty, term);
            barycentric.Add(term);
            if (barycentric.Signs[0] < 0 && (!rectangle || corner != 0)) return false;
            if (barycentric.Signs[0] == 0) edgeMask |= 1 << corner;
            sum.Add(barycentric);
        }
        // The three barycentric numerators sum to the nonzero face metric
        // times the positive input parameter denominator.
        System.Diagnostics.Debug.Assert(sum.Signs[0] > 0);
        if (rectangle)
            for (int corner = 1; corner < 3; corner++)
            {
                ContactQuadratic.At(values, signs, 3 + corner, words).CopyTo(term);
                term.Add(sum, -1);
                if (term.Signs[0] > 0) return false;
            }
        coefficient.Set(Signed576.ExtendValue(first.Denominator));
        ContactQuadratic.Multiply(divisor, coefficient, ReadOnlySpan<ulong>.Empty, sourceD);
        coefficient.Set(Signed576.ExtendValue(frame.Denominator));
        ContactQuadratic.Multiply(sum, coefficient, ReadOnlySpan<ulong>.Empty, targetD);
        Span<RationalPoint3d> corners = stackalloc RationalPoint3d[3]
            { frame.GetPoint(triangle.A), frame.GetPoint(triangle.B), frame.GetPoint(triangle.C) };
        for (int coordinate = 0; coordinate < 3; coordinate++)
        {
            var source = coordinate == 0 ? sourceX : coordinate == 1 ? sourceY : sourceZ;
            var target = coordinate == 0 ? targetX : coordinate == 1 ? targetY : targetZ;
            Signed320 a = coordinate == 0 ? first.X : coordinate == 1 ? first.Y : first.Z;
            Signed320 b = coordinate == 0 ? second.X : coordinate == 1 ? second.Y : second.Z;
            ContactQuadratic.Scale(remaining, Signed576.ExtendValue(a), source);
            ContactQuadratic.Scale(parameter, Signed576.ExtendValue(b), term); source.Add(term);
            target.Clear();
            for (int corner = 0; corner < 3; corner++)
            {
                RationalPoint3d point = corners[corner];
                Signed320 raw = coordinate == 0 ? point.X : coordinate == 1 ? point.Y : point.Z;
                ContactQuadratic.Scale(ContactQuadratic.At(values, signs, 3 + corner, words), Signed576.ExtendValue(raw), term);
                target.Add(term);
            }
        }
        return true;
    }

    internal static bool ClipScaledHalfPlane(Signed832 first, Signed832 second,
        ref Signed832 lowerN, ref Signed832 lowerD, ref Signed832 upperN, ref Signed832 upperD)
    {
        if (first.Sign >= 0 && second.Sign >= 0) return true;
        if (first.Sign < 0 && second.Sign < 0) return false;
        Signed832 n = first, d = WideArithmetic.SubtractSigned832(first, second);
        if (d.Sign < 0) { n = WideArithmetic.SubtractSigned832(default, n); d = WideArithmetic.SubtractSigned832(default, d); }
        if (first.Sign < 0)
        {
            if (WideArithmetic.CompareNonNegativeProducts(n, lowerD, lowerN, d) > 0) { lowerN = n; lowerD = d; }
        }
        else if (WideArithmetic.CompareNonNegativeProducts(n, upperD, upperN, d) < 0) { upperN = n; upperD = d; }
        return WideArithmetic.CompareNonNegativeProducts(lowerN, upperD, upperN, lowerD) <= 0;
    }

    internal static bool GetScaledTriangleHalfPlanes<TFrame>(in FixedTriangle triangle, in TFrame frame,
        in RationalPoint3d point, Span<Signed832> values) where TFrame : struct, IForwardHullFrame3d =>
        frame.TryGetTriangleHalfPlanes(triangle, point, values);

    internal static bool GetScaledTriangleHalfPlanes(in FixedTriangle triangle, in ScaledRigidFrame3d frame,
        in RationalPoint3d point, Span<Signed832> values)
    {
        WideDirection3d localNormal = GetScaledTriangleLocalNormal(triangle, frame);
        if (localNormal.IsZero) return false;
        Span<Vector3d> vertices = stackalloc Vector3d[3] { triangle.A, triangle.B, triangle.C };
        for (int edge = 0; edge < 3; edge++)
        {
            Vector3d a = vertices[edge], b = vertices[(edge + 1) % 3];
            WideDirection3d inward = WideDirection3d.Cross(localNormal, WideDirection3d.GetLocalEdge(frame, a, b))
                .Transform(frame.GeometryBasis);
            WideDirection3d delta = GetScaledPointDifference(point, frame.GetPoint(a), frame.RotationDenominator);
            values[edge] = DotScaledDirections(inward, delta);
        }
        return true;
    }

    internal static WideDirection3d GetScaledTriangleLocalNormal(in FixedTriangle triangle, in ScaledRigidFrame3d frame)
    {
        // Rigid geometry already uses physical local vertices. Retain the
        // original normal instead of injecting Q^4 through two unit-scale
        // edge numerators; that redundant factor also inflates fan minors.
        if (frame.IsRigid)
        {
            triangle.GetExactNormal(out Signed192 x, out Signed192 y, out Signed192 z, out _);
            return new WideDirection3d(Signed576.ExtendValue(x), Signed576.ExtendValue(y), Signed576.ExtendValue(z));
        }
        return WideDirection3d.Cross(WideDirection3d.GetLocalEdge(frame, triangle.A, triangle.B),
            WideDirection3d.GetLocalEdge(frame, triangle.A, triangle.C));
    }

    internal static WideDirection3d GetScaledPointDifference(in RationalPoint3d point,
        in RationalPoint3d reference, Signed192 referenceRotationDenominator)
    {
        Signed192 pointRotationD = WideArithmetic.TruncateSigned320By64(point.Denominator);
        return new WideDirection3d(Difference(point.X, reference.X, referenceRotationDenominator, pointRotationD),
            Difference(point.Y, reference.Y, referenceRotationDenominator, pointRotationD),
            Difference(point.Z, reference.Z, referenceRotationDenominator, pointRotationD));
        static Signed576 Difference(Signed320 point, Signed320 reference, Signed192 referenceD, Signed192 pointD) =>
            WideArithmetic.SubtractSigned576(WideArithmetic.MultiplySigned320(point, referenceD),
                WideArithmetic.MultiplySigned320(reference, pointD));
    }

    internal static Signed832 DotScaledDirections(in WideDirection3d first, in WideDirection3d second) =>
        WideArithmetic.AddSigned832(WideArithmetic.AddSigned832(WideArithmetic.MultiplySigned576ToSigned832(first.X, second.X),
            WideArithmetic.MultiplySigned576ToSigned832(first.Y, second.Y)), WideArithmetic.MultiplySigned576ToSigned832(first.Z, second.Z));
}

