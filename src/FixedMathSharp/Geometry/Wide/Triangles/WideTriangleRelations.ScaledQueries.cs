//=======================================================================
// WideTriangleRelations.ScaledQueries.cs
//=======================================================================
// MIT License, Copyright (c) 2024-present David Oravsky (mrdav30)
// See LICENSE file in the project root for full license information.
//=======================================================================

using System;

namespace FixedMathSharp.Geometry;

/// <content>Finite segment and closest-point queries over exact scaled source triangles.</content>
internal static partial class WideTriangleRelations
{
    /// <summary>Rounds scaled triangle area once and normalizes its exact local normal.</summary>
    /// <remarks>Source-frame centered reach is at most 4096 units; the squared normal fits Signed832.</remarks>
    internal static void GetScaledAreaAndNormal(in FixedTriangle triangle, in ScaledRigidFrame3d frame,
        out Fixed64 area, out Vector3d normal)
    {
        // Area needs the metric Q^4 cross factor even when the direction-only
        // rigid normal can cancel it.
        WideDirection3d exactNormal = WideDirection3d.Cross(
            WideDirection3d.GetLocalEdge(frame, triangle.A, triangle.B),
            WideDirection3d.GetLocalEdge(frame, triangle.A, triangle.C));
        normal = exactNormal.GetNormal(false);
        const int words = 13;
        Span<ulong> values = stackalloc ulong[4 * words]; Span<int> signs = stackalloc int[4];
        var numerator = ContactQuadratic.At(values, signs, 0, words);
        var denominator = ContactQuadratic.At(values, signs, 1, words);
        numerator.Set(exactNormal.SquaredLength);
        // Each edge carries Q^2 beyond its raw coordinate. Area's raw value
        // is |cross|/(2*Q^5), so the squared divisor is 4*Q^10.
        denominator.Set(new Signed576(0, 0, 0, 4, 0, 0, 0, 0, 0));
        bool represented = ContactQuadratic.TryRoundSquareRootRatio(numerator, denominator, ReadOnlySpan<ulong>.Empty, out area);
        System.Diagnostics.Debug.Assert(represented);
    }

    internal static bool TryGetProjectedSupportContact(in FixedTriangle triangle, in ScaledRigidFrame3d frame,
        in FixedPointAnchor support, Vector3d worldNormal, Vector3d materializationOrigin, out FixedContactAnchors contact)
    {
        contact = default;
        RationalPoint3d point = RationalPoint3d.FromAnchor(support);
        WideDirection3d normal = GetScaledTriangleLocalNormal(triangle, frame).Transform(frame.GeometryBasis);
        WideDirection3d offset = GetScaledPointDifference(point, frame.GetPoint(triangle.A), frame.RotationDenominator);
        Signed832 signedGap = WideDirection3d.Dot(normal, offset);
        var direction = new WideAxis3(Signed320.ExtendValue(Signed192.Raw(worldNormal.X)),
            Signed320.ExtendValue(Signed192.Raw(worldNormal.Y)), Signed320.ExtendValue(Signed192.Raw(worldNormal.Z)));
        int orientation = WideDirection3d.Dot(normal, direction).Sign;
        if (orientation == 0 || orientation * signedGap.Sign > 0
            || !TryGetProjectedPointAnchor(triangle, frame, point, materializationOrigin, out FixedPointAnchor meshAnchor)) return false;
        Signed832 overlap = signedGap.Sign < 0 ? WideArithmetic.SubtractSigned832(default, signedGap) : signedGap;
        Signed576 denominator = WideArithmetic.MultiplySigned320(point.Denominator, frame.RotationDenominator);
        Fixed64 depth = WideArithmetic.GetRoundedNonNegativeNormalizedDepth(overlap, normal.SquaredLength, denominator, out bool clamped);
        contact = new FixedContactAnchors(meshAnchor, support, worldNormal, depth, clamped);
        return true;
    }

    internal static bool TryGetSegmentIntersection(in FixedTriangle triangle, in ScaledRigidFrame3d frame,
        in FixedSegment segment, out Vector3d entry, out Vector3d exit)
    {
        entry = exit = default;
        RationalPoint3d first = RationalPoint3d.FromScalarPoint(segment.Start);
        RationalPoint3d second = RationalPoint3d.FromScalarPoint(segment.End);
        WideDirection3d normal = GetScaledTriangleLocalNormal(triangle, frame).Transform(frame.GeometryBasis);
        if (normal.IsZero) return false;
        RationalPoint3d origin = frame.GetPoint(triangle.A);
        Signed832 planeFirst = DotScaledDirections(normal, GetScaledPointDifference(first, origin, frame.RotationDenominator));
        Signed832 planeSecond = DotScaledDirections(normal, GetScaledPointDifference(second, origin, frame.RotationDenominator));
        if (planeFirst.IsZero && planeSecond.IsZero)
        {
            if (!TryGetProjectedSegmentInterval(triangle, frame, first, second,
                out Signed832 lowerN, out Signed832 lowerD, out Signed832 upperN, out Signed832 upperD)) return false;
            entry = MaterializeScaledSegmentPoint(segment, lowerN, lowerD);
            exit = MaterializeScaledSegmentPoint(segment, upperN, upperD);
            return true;
        }
        if (planeFirst.Sign == planeSecond.Sign) return false;
        Signed832 numerator = planeFirst, denominator = WideArithmetic.SubtractSigned832(planeFirst, planeSecond);
        if (denominator.Sign < 0)
        {
            numerator = WideArithmetic.SubtractSigned832(default, numerator);
            denominator = WideArithmetic.SubtractSigned832(default, denominator);
        }
        Span<Signed832> a = stackalloc Signed832[3], b = stackalloc Signed832[3];
        // The nonzero normal above proves both finite half-plane loaders exist.
        GetScaledTriangleHalfPlanes(triangle, frame, first, a);
        GetScaledTriangleHalfPlanes(triangle, frame, second, b);
        const int words = 24;
        Span<ulong> values = stackalloc ulong[8 * words]; Span<int> signs = stackalloc int[8];
        var left = ContactQuadratic.At(values, signs, 0, words);
        var right = ContactQuadratic.At(values, signs, 1, words);
        var term = ContactQuadratic.At(values, signs, 2, words);
        var sum = ContactQuadratic.At(values, signs, 3, words);
        Signed832 remaining = WideArithmetic.SubtractSigned832(denominator, numerator);
        // Half-plane values (<786 bits) times plane parameters (<610 bits)
        // fit the 1536-bit borrowed bank; no input or parameter is rounded.
        for (int edge = 0; edge < 3; edge++)
        {
            left.Set(a[edge]); right.Set(b[edge]);
            ContactQuadratic.Scale(left, remaining, sum);
            ContactQuadratic.Scale(right, numerator, term); sum.Add(term);
            if (sum.Signs[0] < 0) return false;
        }
        entry = MaterializeScaledSegmentPoint(segment, numerator, denominator);
        exit = entry;
        return true;
    }

    internal static bool TryGetClosestPointAnchor(in FixedTriangle triangle, in ScaledRigidFrame3d frame,
        Vector3d queryPoint, Vector3d materializationOrigin, out FixedPointAnchor anchor) =>
        TryGetClosestPointAnchor(triangle, frame, RationalPoint3d.FromScalarPoint(queryPoint), materializationOrigin, out anchor);

    internal static bool TryGetClosestPointAnchor(in FixedTriangle triangle, in ScaledRigidFrame3d frame,
        in RationalPoint3d query, Vector3d materializationOrigin, out FixedPointAnchor anchor) =>
        TryGetClosestPointAnchor(triangle, frame, query, materializationOrigin, default, default, out anchor);

    /// <summary>Also retains the exact squared raw-coordinate distance for cross-triangle selection.</summary>
    /// <remarks>
    /// Both borrowed output banks require 36 words. With admitted 4096-unit geometry,
    /// two source rotation denominators and a full-domain rational anchor query,
    /// the common denominator is below 294 bits, offsets below 360 and edges below
    /// 339. Edge squared lengths are below 680 bits; squared residual sums below
    /// 2090 and their squared denominators below 1950. Face ratios are smaller.
    /// Ratio comparison retains its complete transient cross products separately.
    /// </remarks>
    internal static bool TryGetClosestPointAnchor(in FixedTriangle triangle, in ScaledRigidFrame3d frame,
        in RationalPoint3d query, Vector3d materializationOrigin,
        scoped ContactQuadratic squaredDistanceNumerator, scoped ContactQuadratic squaredDistanceDenominator,
        out FixedPointAnchor anchor)
    {
        if (TryGetProjectedPointAnchor(triangle, frame, query, materializationOrigin,
                squaredDistanceNumerator, squaredDistanceDenominator, out anchor)) return true;
        Span<Vector3d> vertices = stackalloc Vector3d[3] { triangle.A, triangle.B, triangle.C };
        // A rational query can carry another rigid denominator. The exact
        // residual square needs fewer than 2300 bits in the admitted domain.
        const int words = 36;
        Span<ulong> values = stackalloc ulong[20 * words]; Span<int> signs = stackalloc int[20];
        var bestNumerator = ContactQuadratic.At(values, signs, 0, words);
        var bestDenominator = ContactQuadratic.At(values, signs, 1, words);
        var distance = ContactQuadratic.At(values, signs, 2, words);
        var distanceDenominator = ContactQuadratic.At(values, signs, 3, words);
        var delta = ContactQuadratic.At(values, signs, 4, words);
        var edgeValue = ContactQuadratic.At(values, signs, 5, words);
        var term = ContactQuadratic.At(values, signs, 6, words);
        var residual = ContactQuadratic.At(values, signs, 7, words);
        var square = ContactQuadratic.At(values, signs, 8, words);
        var divisor = ContactQuadratic.At(values, signs, 9, words);
        Signed192 queryFactor = WideArithmetic.TruncateSigned320By64(query.Denominator);
        Signed576 commonDenominator = WideArithmetic.MultiplySigned320(frame.Denominator, queryFactor);
        int selected = -1;
        Signed832 selectedN = default, selectedD = default;
        for (int edge = 0; edge < 3; edge++)
        {
            RationalPoint3d a = frame.GetPoint(vertices[edge]), b = frame.GetPoint(vertices[(edge + 1) % 3]);
            var direction = new WideDirection3d(
                WideArithmetic.MultiplySigned320(WideArithmetic.SubtractSigned320(b.X, a.X), queryFactor),
                WideArithmetic.MultiplySigned320(WideArithmetic.SubtractSigned320(b.Y, a.Y), queryFactor),
                WideArithmetic.MultiplySigned320(WideArithmetic.SubtractSigned320(b.Z, a.Z), queryFactor));
            WideDirection3d offset = GetScaledPointDifference(query, a, frame.RotationDenominator);
            Signed832 numerator = WideDirection3d.Dot(direction, offset), denominator = direction.SquaredLength;
            if (denominator.IsZero) continue;
            if (numerator.Sign < 0) numerator = default;
            else if (WideArithmetic.SubtractSigned832(numerator, denominator).Sign > 0) numerator = denominator;
            distance.Clear();
            for (int axis = 0; axis < 3; axis++)
            {
                delta.Set(axis == 0 ? offset.X : axis == 1 ? offset.Y : offset.Z);
                edgeValue.Set(axis == 0 ? direction.X : axis == 1 ? direction.Y : direction.Z);
                ContactQuadratic.Scale(delta, denominator, residual);
                ContactQuadratic.Scale(edgeValue, numerator, term); residual.Add(term, -1);
                ContactQuadratic.Multiply(residual, residual, ReadOnlySpan<ulong>.Empty, square);
                distance.Add(square);
            }
            divisor.Set(commonDenominator);
            ContactQuadratic.Scale(divisor, denominator, term);
            ContactQuadratic.Multiply(term, term, ReadOnlySpan<ulong>.Empty, distanceDenominator);
            // Squared residuals fit 2304 bits. Ratio comparison retains its
            // complete transient cross products before choosing the feature.
            if (selected >= 0 && ContactQuadratic.CompareRatios(distance, distanceDenominator, ReadOnlySpan<ulong>.Empty,
                bestNumerator, bestDenominator, ReadOnlySpan<ulong>.Empty) >= 0) continue;
            distance.CopyTo(bestNumerator); distanceDenominator.CopyTo(bestDenominator);
            selected = edge; selectedN = numerator; selectedD = denominator;
        }
        if (selected < 0)
        {
            if (!squaredDistanceNumerator.Values.IsEmpty)
            {
                WideDirection3d offset = GetScaledPointDifference(query, frame.GetPoint(triangle.A), frame.RotationDenominator);
                squaredDistanceNumerator.Set(offset.SquaredLength);
                divisor.Set(commonDenominator);
                ContactQuadratic.Multiply(divisor, divisor, ReadOnlySpan<ulong>.Empty, squaredDistanceDenominator);
            }
            bool represented = frame.GetPoint(triangle.A).TryGetLocalPointIn(materializationOrigin,
                FixedQuaternion.Identity, out Vector3d local);
            anchor = represented ? new FixedPointAnchor(materializationOrigin, FixedQuaternion.Identity, local) : default;
            return represented;
        }
        if (!squaredDistanceNumerator.Values.IsEmpty)
        {
            bestNumerator.CopyTo(squaredDistanceNumerator);
            bestDenominator.CopyTo(squaredDistanceDenominator);
        }
        return TryMaterializeScaledEdge(frame.GetPoint(vertices[selected]), frame.GetPoint(vertices[(selected + 1) % 3]),
            selectedN, selectedD, materializationOrigin, out anchor);
    }

    private static Vector3d MaterializeScaledSegmentPoint(in FixedSegment segment, Signed832 n, Signed832 d)
    {
        bool represented = TryMaterializeScaledEdge(RationalPoint3d.FromScalarPoint(segment.Start),
            RationalPoint3d.FromScalarPoint(segment.End), n, d, Vector3d.Zero, out FixedPointAnchor anchor);
        // A retained closed unit parameter is a convex combination of two
        // scalar endpoints, so zero-origin final materialization cannot overflow.
        System.Diagnostics.Debug.Assert(represented);
        return anchor.LocalPoint;
    }

    private static bool TryMaterializeScaledEdge(in RationalPoint3d a, in RationalPoint3d b,
        Signed832 n, Signed832 d, Vector3d samplingOrigin, out FixedPointAnchor anchor)
    {
        const int words = 13;
        Span<ulong> values = stackalloc ulong[4 * words]; Span<int> signs = stackalloc int[4];
        var numerator = ContactQuadratic.At(values, signs, 0, words);
        var denominator = ContactQuadratic.At(values, signs, 1, words);
        numerator.Set(n); denominator.Set(d);
        return TryGetSegmentPointAnchor(a, b, numerator, denominator, samplingOrigin, out anchor);
    }
}
