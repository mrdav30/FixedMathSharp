//=======================================================================
// WideOrientedBox.ConvexHullCapsule.Support.cs
//=======================================================================
// MIT License, Copyright (c) 2024-present David Oravsky (mrdav30)
// See LICENSE file in the project root for full license information.
//=======================================================================

using System;

namespace FixedMathSharp.Geometry;

/// <content>
/// Support utilities for convex-hull vs capsule tests: candidate feature
/// classification and wide, fixed-point projection interval computation
/// along a capsule axis within a rational basis.
/// </content>
internal static partial class WideOrientedBox
{
    #region Nested Types

    private enum HullCapsuleCandidateKind
    {
        Face,
        EdgeCross,
        VertexCore,
        EndpointEdge,
    }

    private readonly struct HullCapsuleCandidate
    {
        internal readonly HullCapsuleCandidateKind Kind;
        internal readonly int Index;
        internal readonly int EndpointSign;

        internal HullCapsuleCandidate(
            HullCapsuleCandidateKind kind,
            int index,
            int endpointSign)
        {
            Kind = kind;
            Index = index;
            EndpointSign = endpointSign;
        }
    }

    #endregion

    private static void GetHullCapsuleProjectionInterval(
        ReadOnlySpan<Vector3d> points,
        WideRationalBasis3d basis,
        CapsuleAxis3 axis,
        Span<ulong> minimum,
        out int minimumSign,
        Span<ulong> maximum,
        out int maximumSign)
    {
        // The axis and rational basis are invariant across every hull point.
        // Retain their exact projections; only the local coordinate products vary.
        Signed704 xProjection = GetCapsuleBasisProjection(
            axis, basis.Xx, basis.Xy, basis.Xz);
        Signed704 yProjection = GetCapsuleBasisProjection(
            axis, basis.Yx, basis.Yy, basis.Yz);
        Signed704 zProjection = GetCapsuleBasisProjection(
            axis, basis.Zx, basis.Zy, basis.Zz);
        BuildHullCapsuleLocalProjection(
            points[0],
            xProjection,
            yProjection,
            zProjection,
            minimum,
            out minimumSign);
        minimum.CopyTo(maximum);
        maximumSign = minimumSign;
        Span<ulong> candidate =
            stackalloc ulong[HullCapsuleMagnitudeWords];
        for (int index = 1; index < points.Length; index++)
        {
            BuildHullCapsuleLocalProjection(
                points[index],
                xProjection,
                yProjection,
                zProjection,
                candidate,
                out int candidateSign);
            if (CompareHullCapsuleSignedMagnitudes(
                    candidate,
                    candidateSign,
                    minimum,
                    minimumSign) < 0)
            {
                candidate.CopyTo(minimum);
                minimumSign = candidateSign;
            }
            if (CompareHullCapsuleSignedMagnitudes(
                    candidate,
                    candidateSign,
                    maximum,
                    maximumSign) > 0)
            {
                candidate.CopyTo(maximum);
                maximumSign = candidateSign;
            }
        }
    }

    private static void BuildHullCapsuleLocalProjection(
        Vector3d point,
        Signed704 xProjection,
        Signed704 yProjection,
        Signed704 zProjection,
        Span<ulong> result,
        out int resultSign)
    {
        // Each signed 704-bit projection times a raw signed 64-bit coordinate
        // has magnitude at most 2^766. Three terms need at most 769 signed
        // bits, so both products and their exact sum fit Signed832.
        Signed832 projection = WideArithmetic.AddSigned832(
            WideArithmetic.AddSigned832(
                WideArithmetic.MultiplySigned704ToSigned832(
                    xProjection, Signed192.Raw(point.X)),
                WideArithmetic.MultiplySigned704ToSigned832(
                    yProjection, Signed192.Raw(point.Y))),
            WideArithmetic.MultiplySigned704ToSigned832(
                zProjection, Signed192.Raw(point.Z)));
        WideArithmetic.GetMagnitude(projection, result);
        resultSign = projection.Sign;
    }

    private static Signed704 GetHullCapsuleCenterProjection(
        Vector3d end,
        Vector3d start,
        CapsuleAxis3 axis) =>
        WideArithmetic.AddSigned704(
            WideArithmetic.AddSigned704(
                WideArithmetic.MultiplySigned576ToSigned704(
                    axis.X,
                    Signed320.ExtendValue(
                        WideArithmetic.SubtractSigned192(
                            Signed192.Raw(end.X),
                            Signed192.Raw(start.X)))),
                WideArithmetic.MultiplySigned576ToSigned704(
                    axis.Y,
                    Signed320.ExtendValue(
                        WideArithmetic.SubtractSigned192(
                            Signed192.Raw(end.Y),
                            Signed192.Raw(start.Y))))),
            WideArithmetic.MultiplySigned576ToSigned704(
                axis.Z,
                Signed320.ExtendValue(
                    WideArithmetic.SubtractSigned192(
                        Signed192.Raw(end.Z),
                        Signed192.Raw(start.Z)))));

    private static void BuildHullCapsuleAxisSquared(
        CapsuleAxis3 axis,
        Span<ulong> result)
    {
        Span<ulong> component =
            stackalloc ulong[HullCapsuleMagnitudeWords];
        Span<ulong> square =
            stackalloc ulong[HullCapsuleMagnitudeWords];
        result.Clear();
        GetHullCapsuleMagnitude(axis.X, component);
        WideArithmetic.MultiplyMagnitudes(
            component,
            component,
            square);
        WideArithmetic.AddMagnitudeInto(square, result);
        GetHullCapsuleMagnitude(axis.Y, component);
        WideArithmetic.MultiplyMagnitudes(
            component,
            component,
            square);
        WideArithmetic.AddMagnitudeInto(square, result);
        GetHullCapsuleMagnitude(axis.Z, component);
        WideArithmetic.MultiplyMagnitudes(
            component,
            component,
            square);
        WideArithmetic.AddMagnitudeInto(square, result);
    }

    private static bool ContainsHullSphereSupportProjection(
        Vector3d hullOrigin, FixedQuaternion hullRotation, WideRationalBasis3d basis,
        ReadOnlySpan<Vector3d> points, ReadOnlySpan<int> triangles,
        CapsuleAxis3 orientedAxis, Vector3d sphereCenter)
    {
        Span<ulong> minimum = stackalloc ulong[HullCapsuleMagnitudeWords];
        Span<ulong> maximum = stackalloc ulong[HullCapsuleMagnitudeWords];
        GetHullCapsuleProjectionInterval(points, basis, orientedAxis,
            minimum, out _, maximum, out int maximumSign);
        Signed704 x = GetCapsuleBasisProjection(orientedAxis, basis.Xx, basis.Xy, basis.Xz);
        Signed704 y = GetCapsuleBasisProjection(orientedAxis, basis.Yx, basis.Yy, basis.Yz);
        Signed704 z = GetCapsuleBasisProjection(orientedAxis, basis.Zx, basis.Zy, basis.Zz);
        FixedPointAnchor center = new(sphereCenter, FixedQuaternion.Identity, Vector3d.Zero);
        // Only a finite triangle on the selected support plane can certify
        // the closest point. The winning axis may come from the opposite face.
        for (int index = 0; index < triangles.Length; index += 3)
        {
            Vector3d first = points[triangles[index]];
            Vector3d second = points[triangles[index + 1]];
            Vector3d third = points[triangles[index + 2]];
            if (IsHullCapsuleSupportPoint(first, x, y, z, maximum, maximumSign)
                && IsHullCapsuleSupportPoint(second, x, y, z, maximum, maximumSign)
                && IsHullCapsuleSupportPoint(third, x, y, z, maximum, maximumSign)
                && WideTriangleRelations.ContainsProjection(new FixedTriangle(first, second, third),
                    hullOrigin, hullRotation, center))
                return true;
        }
        return false;
    }

    private static FixedPointAnchor GetMatchedHullCapsuleSupportAnchor(
        Vector3d hullOrigin,
        FixedQuaternion hullRotation,
        WideRationalBasis3d basis,
        ReadOnlySpan<Vector3d> points,
        ReadOnlySpan<int> triangleVertexIndices,
        ReadOnlySpan<int> edgeVertexPairs,
        CapsuleAxis3 orientedAxis,
        in FixedPointAnchor target,
        Vector3d capsuleCenter, FixedQuaternion capsuleRotation, Vector3d capsuleAxis,
        Fixed64 capsuleLength, Fixed64 capsuleRadius, Vector3d normal, bool matchCore,
        out FixedPointAnchor matchedCapsule)
    {
        matchedCapsule = target;
        bool hasCoreMatch = false;
        Signed832 coreN = default, coreD = default;
        FixedTriangle coreFeature = default;
        FixedPointAnchor firstEnd = default, secondEnd = default;
        Signed704 xProjection = GetCapsuleBasisProjection(
            orientedAxis, basis.Xx, basis.Xy, basis.Xz);
        Signed704 yProjection = GetCapsuleBasisProjection(
            orientedAxis, basis.Yx, basis.Yy, basis.Yz);
        Signed704 zProjection = GetCapsuleBasisProjection(
            orientedAxis, basis.Zx, basis.Zy, basis.Zz);
        int supportIndex = 0;
        Span<ulong> best =
            stackalloc ulong[HullCapsuleMagnitudeWords];
        BuildHullCapsuleLocalProjection(
            points[0],
            xProjection,
            yProjection,
            zProjection,
            best,
            out int bestSign);
        Span<ulong> candidate =
            stackalloc ulong[HullCapsuleMagnitudeWords];
        for (int index = 1; index < points.Length; index++)
        {
            BuildHullCapsuleLocalProjection(
                points[index],
                xProjection,
                yProjection,
                zProjection,
                candidate,
                out int candidateSign);
            if (CompareHullCapsuleSignedMagnitudes(
                    candidate,
                    candidateSign,
                    best,
                    bestSign) <= 0)
            {
                continue;
            }
            supportIndex = index;
            candidate.CopyTo(best);
            bestSign = candidateSign;
        }

        FixedPointAnchor closest = new(
            hullOrigin,
            hullRotation,
            points[supportIndex]);
        for (int index = 0;
            index < triangleVertexIndices.Length;
            index += 3)
        {
            Vector3d first = points[
                triangleVertexIndices[index]];
            Vector3d second = points[
                triangleVertexIndices[index + 1]];
            Vector3d third = points[
                triangleVertexIndices[index + 2]];
            if (!IsHullCapsuleSupportPoint(
                    first,
                    xProjection,
                    yProjection,
                    zProjection,
                    best,
                    bestSign)
                || !IsHullCapsuleSupportPoint(
                    second,
                    xProjection,
                    yProjection,
                    zProjection,
                    best,
                    bestSign)
                || !IsHullCapsuleSupportPoint(
                    third,
                    xProjection,
                    yProjection,
                    zProjection,
                    best,
                    bestSign))
            {
                continue;
            }

            var triangle = new FixedTriangle(first, second, third);
            // A projection inside a support face is the unique nearest point
            // on the entire support set; boundary edges cannot improve it.
            // Keep the first authored certified face rather than ranking
            // triangulation differences introduced by final anchor rounding.
            if (WideTriangleRelations.ContainsProjection(triangle, hullOrigin, hullRotation, target))
                return WideTriangleRelations.GetClosestPointAnchor(triangle, hullOrigin, hullRotation, target,
                    preserveExactFace: true);

            if (matchCore)
                continue;

            FixedPointAnchor candidateAnchor =
                triangle
                    .GetClosestPointAnchor(
                        hullOrigin,
                        hullRotation,
                        target);
            if (target.CompareSquaredDistance(
                    candidateAnchor,
                    closest) < 0)
            {
                closest = candidateAnchor;
            }
        }

        if (matchCore)
        {
            // Ordinary midpoint contacts return above, before endpoint work.
            firstEnd = WideGeometry.GetCenteredCapsuleSupportAnchor(capsuleCenter, capsuleRotation,
                capsuleAxis, capsuleLength, Fixed64.Zero, -normal, -1);
            secondEnd = WideGeometry.GetCenteredCapsuleSupportAnchor(capsuleCenter, capsuleRotation,
                capsuleAxis, capsuleLength, Fixed64.Zero, -normal, 1);
            for (int index = 0; index < triangleVertexIndices.Length; index += 3)
            {
                var triangle = new FixedTriangle(points[triangleVertexIndices[index]],
                    points[triangleVertexIndices[index + 1]], points[triangleVertexIndices[index + 2]]);
                if (!GetHullTriangleNormal(points, triangleVertexIndices, index).IsZero
                    && IsHullCapsuleSupportPoint(triangle.A, xProjection, yProjection, zProjection, best, bestSign)
                    && IsHullCapsuleSupportPoint(triangle.B, xProjection, yProjection, zProjection, best, bestSign)
                    && IsHullCapsuleSupportPoint(triangle.C, xProjection, yProjection, zProjection, best, bestSign))
                    KeepHullCapsuleCoreMatch(triangle, hullOrigin, hullRotation, firstEnd, secondEnd,
                        capsuleCenter, capsuleRotation, capsuleAxis, capsuleLength,
                        ref hasCoreMatch, ref coreFeature, ref coreN, ref coreD);
            }
        }
        // A finite face match owns the footprint; edges/vertices cannot replace
        // it merely because their unrelated closest core parameter is smaller.
        bool hasFaceMatch = hasCoreMatch;
        for (int index = 0;
            !hasFaceMatch && index < edgeVertexPairs.Length;
            index += 2)
        {
            Vector3d start =
                points[edgeVertexPairs[index]];
            Vector3d end =
                points[edgeVertexPairs[index + 1]];
            if (!IsHullCapsuleSupportPoint(
                    start,
                    xProjection,
                    yProjection,
                    zProjection,
                    best,
                    bestSign)
                || !IsHullCapsuleSupportPoint(
                    end,
                    xProjection,
                    yProjection,
                    zProjection,
                    best,
                    bestSign))
            {
                continue;
            }

            if (matchCore)
                KeepHullCapsuleCoreMatch(new FixedTriangle(start, end, end), hullOrigin, hullRotation,
                    firstEnd, secondEnd, capsuleCenter, capsuleRotation, capsuleAxis, capsuleLength,
                    ref hasCoreMatch, ref coreFeature, ref coreN, ref coreD);
            if (hasCoreMatch)
                continue;

            FixedPointAnchor candidateAnchor =
                new FixedTriangle(start, end, end)
                    .GetClosestPointAnchor(
                        hullOrigin,
                        hullRotation,
                        target);
            if (target.CompareSquaredDistance(
                    candidateAnchor,
                    closest) < 0)
            {
                closest = candidateAnchor;
            }
        }
        if (matchCore)
        {
            for (int index = 0; !hasCoreMatch && index < points.Length; index++)
            {
                Vector3d point = points[index];
                if (IsHullCapsuleSupportPoint(point, xProjection, yProjection, zProjection, best, bestSign))
                    KeepHullCapsuleCoreMatch(new FixedTriangle(point, point, point), hullOrigin, hullRotation,
                        firstEnd, secondEnd, capsuleCenter, capsuleRotation, capsuleAxis, capsuleLength,
                        ref hasCoreMatch, ref coreFeature, ref coreN, ref coreD);
            }
            if (hasCoreMatch)
            {
                Signed832 magnitude = coreN.Sign < 0 ? WideArithmetic.SubtractSigned832(default, coreN) : coreN;
                bool isEndpoint = WideArithmetic.SubtractSigned832(
                    WideArithmetic.MultiplySigned832(coreD, Signed192.Raw(capsuleLength)),
                    WideArithmetic.AddSigned832(magnitude, magnitude)).IsZero;
                matchedCapsule = isEndpoint
                    ? WideGeometry.GetCenteredCapsuleSupportAnchor(capsuleCenter, capsuleRotation,
                        capsuleAxis, capsuleLength, capsuleRadius, -normal, coreN.Sign)
                    : new FixedPointAnchor(capsuleCenter, capsuleRotation,
                        WideTriangleRelations.MaterializeCorePoint(capsuleAxis, coreN, coreD),
                        target.LocalDisplacement, target.ExactLocalTerm);
                // Admitted faces are nondegenerate; edge/vertex features repeat B as C.
                if (coreFeature.A == coreFeature.B)
                    return FixedPointAnchor.FromValidatedFrame(hullOrigin, hullRotation, coreFeature.A);
                return WideTriangleRelations.GetClosestPointAnchor(coreFeature, hullOrigin, hullRotation, matchedCapsule,
                    preserveExactFace: true);
            }
        }
        return closest;
    }

    private static void KeepHullCapsuleCoreMatch(
        FixedTriangle feature, Vector3d origin, FixedQuaternion rotation,
        in FixedPointAnchor firstEnd, in FixedPointAnchor secondEnd,
        Vector3d center, FixedQuaternion coreRotation, Vector3d axis, Fixed64 length,
        ref bool found, ref FixedTriangle bestFeature, ref Signed832 bestN, ref Signed832 bestD)
    {
        if (WideTriangleRelations.TryGetProjectedCoreParameter(feature, origin, rotation,
                firstEnd, secondEnd, center, coreRotation, axis, length,
                out Signed832 n, out Signed832 d)
            && (!found || WideTriangleRelations.CompareCoreParameters(n, d, bestN, bestD) < 0))
        { found = true; bestFeature = feature; bestN = n; bestD = d; }
    }

    private static bool IsHullCapsuleSupportPoint(
        Vector3d point,
        Signed704 xProjection,
        Signed704 yProjection,
        Signed704 zProjection,
        ReadOnlySpan<ulong> supportProjection,
        int supportProjectionSign)
    {
        Span<ulong> projection =
            stackalloc ulong[HullCapsuleMagnitudeWords];
        BuildHullCapsuleLocalProjection(
            point,
            xProjection,
            yProjection,
            zProjection,
            projection,
            out int projectionSign);
        return CompareHullCapsuleSignedMagnitudes(
            projection,
            projectionSign,
            supportProjection,
            supportProjectionSign) == 0;
    }

    private static void GetHullCapsuleDepth(
        ReadOnlySpan<ulong> rational,
        int rationalSign,
        ReadOnlySpan<ulong> squaredAxisLength,
        Signed192 commonDenominator,
        Fixed64 radius,
        out Fixed64 depth,
        out bool depthIsClamped)
    {
        // floor(sqrt(floor(N/D))) equals floor(sqrt(N/D)). Reuse the exact
        // ratio-root owner instead of searching the entire scalar range.
        Span<ulong> numerator = stackalloc ulong[2 * HullCapsuleMagnitudeWords];
        Span<ulong> denominator = stackalloc ulong[2 * HullCapsuleMagnitudeWords];
        Span<ulong> common = stackalloc ulong[3];
        Span<ulong> commonSquared = stackalloc ulong[6];
        WideArithmetic.GetMagnitude(commonDenominator,
            out common[2], out common[1], out common[0]);
        WideArithmetic.MultiplyMagnitudes(rational, rational, numerator);
        WideArithmetic.MultiplyMagnitudes(common, common, commonSquared);
        WideArithmetic.MultiplyMagnitudes(squaredAxisLength, commonSquared, denominator);
        ulong distanceFloor = WideArithmetic.GetRatioFloorSquareRoot(
            numerator, denominator, unchecked((ulong)long.MaxValue));
        ulong floor = rationalSign < 0
            ? (ulong)radius.m_rawValue - distanceFloor
            : (ulong)radius.m_rawValue + distanceFloor;
        // Subtracting a noninteger distance puts this integer one unit above
        // the true floor. Decide that one correction before nearest-even rounding.
        if (rationalSign < 0 && CompareHullCapsuleDepthToTwiceRaw(
                rational, rationalSign, squaredAxisLength, commonDenominator,
                radius, new Signed192(0UL, 0UL, floor << 1)) < 0)
        {
            floor--;
        }

        if (floor >= unchecked((ulong)long.MaxValue))
        {
            depth = Fixed64.MaxValue;
            depthIsClamped =
                CompareHullCapsuleDepthToTwiceRaw(
                    rational,
                    rationalSign,
                    squaredAxisLength,
                    commonDenominator,
                    radius,
                    new Signed192(
                        0UL,
                        0UL,
                        unchecked((ulong)long.MaxValue << 1))) > 0;
            return;
        }

        int midpointComparison =
            CompareHullCapsuleDepthToTwiceRaw(
                rational,
                rationalSign,
                squaredAxisLength,
                commonDenominator,
                radius,
                new Signed192(
                    0UL,
                    0UL,
                    (floor << 1) | 1UL));
        // The comparator is -1, 0, or 1. Adding one plus the floor parity
        // encodes greater-than-half and midpoint-to-even as a 0/1 increment.
        ulong roundingIncrement =
            (ulong)((midpointComparison + 1 + (int)(floor & 1UL)) >> 1);
        depth = Fixed64.FromRaw(
            (long)(floor + roundingIncrement));
        depthIsClamped = false;
    }

    private static int CompareHullCapsuleDepthToTwiceRaw(
        ReadOnlySpan<ulong> rational,
        int rationalSign,
        ReadOnlySpan<ulong> squaredAxisLength,
        Signed192 commonDenominator,
        Fixed64 radius,
        Signed192 twiceRaw)
    {
        Span<ulong> twiceRational =
            stackalloc ulong[HullCapsuleMagnitudeWords];
        rational.CopyTo(twiceRational);
        WideArithmetic.AddMagnitudeInto(
            rational,
            twiceRational);
        Signed192 radialDelta = WideArithmetic.SubtractSigned192(
            WideArithmetic.AddSigned192(
                Signed192.Raw(radius),
                Signed192.Raw(radius)),
            twiceRaw);
        Signed320 coefficient = WideArithmetic.MultiplySigned192(
            commonDenominator,
            radialDelta);
        Span<ulong> coefficientMagnitude =
            stackalloc ulong[HullCapsuleMagnitudeWords];
        GetHullCapsuleMagnitude(
            coefficient,
            coefficientMagnitude);
        return WideArithmetic.GetSignedMagnitudeAndSquareRootSign(
            twiceRational,
            rationalSign,
            coefficientMagnitude,
            coefficient.Sign,
            squaredAxisLength);
    }

    private static void MultiplyHullCapsuleMagnitude(
        Span<ulong> value,
        Signed192 factor,
        Span<ulong> result)
    {
        Span<ulong> factorMagnitude =
            stackalloc ulong[HullCapsuleMagnitudeWords];
        Span<ulong> product =
            stackalloc ulong[HullCapsuleMagnitudeWords];
        GetHullCapsuleMagnitude(factor, factorMagnitude);
        WideArithmetic.MultiplyMagnitudes(
            value,
            factorMagnitude,
            product);
        product.CopyTo(result);
    }

    private static void MultiplyHullCapsuleMagnitude(
        Span<ulong> value,
        Signed320 factor,
        Span<ulong> result)
    {
        Span<ulong> factorMagnitude =
            stackalloc ulong[HullCapsuleMagnitudeWords];
        Span<ulong> product =
            stackalloc ulong[HullCapsuleMagnitudeWords];
        GetHullCapsuleMagnitude(factor, factorMagnitude);
        WideArithmetic.MultiplyMagnitudes(
            value,
            factorMagnitude,
            product);
        product.CopyTo(result);
    }

    private static void GetHullCapsuleMagnitude(
        Signed192 value,
        Span<ulong> result)
    {
        result.Clear();
        WideArithmetic.GetMagnitude(
            value,
            out result[2],
            out result[1],
            out result[0]);
    }

    private static void GetHullCapsuleMagnitude(
        Signed320 value,
        Span<ulong> result)
    {
        result.Clear();
        WideArithmetic.GetMagnitude(
            value,
            out result[4],
            out result[3],
            out result[2],
            out result[1],
            out result[0]);
    }

    private static void GetHullCapsuleMagnitude(
        Signed576 value,
        Span<ulong> result)
    {
        result.Clear();
        Span<ulong> magnitude = stackalloc ulong[9];
        WideArithmetic.GetMagnitude(value, magnitude);
        magnitude.CopyTo(result);
    }

    private static void GetHullCapsuleMagnitude(
        Signed704 value,
        Span<ulong> result)
    {
        result.Clear();
        Span<ulong> magnitude = stackalloc ulong[11];
        WideArithmetic.GetMagnitude(value, magnitude);
        magnitude.CopyTo(result);
    }

    private static int CompareHullCapsuleSignedMagnitudes(
        ReadOnlySpan<ulong> left,
        int leftSign,
        ReadOnlySpan<ulong> right,
        int rightSign)
    {
        if (leftSign != rightSign)
            return leftSign < rightSign ? -1 : 1;
        if (leftSign == 0)
            return 0;
        int comparison =
            WideArithmetic.CompareMagnitudeEqualLength(left, right);
        return leftSign > 0 ? comparison : -comparison;
    }

    private static void CombineHullCapsuleSignedMagnitudes(
        ReadOnlySpan<ulong> left,
        int leftSign,
        ReadOnlySpan<ulong> right,
        int rightSign,
        Span<ulong> result,
        out int resultSign)
    {
        if (leftSign == 0
            || WideArithmetic.IsZeroMagnitude(left))
        {
            right.CopyTo(result);
            resultSign = WideArithmetic.IsZeroMagnitude(right)
                ? 0
                : rightSign;
            return;
        }
        if (rightSign == 0
            || WideArithmetic.IsZeroMagnitude(right))
        {
            left.CopyTo(result);
            resultSign = leftSign;
            return;
        }
        if (leftSign == rightSign)
        {
            left.CopyTo(result);
            WideArithmetic.AddMagnitudeInto(right, result);
            resultSign = leftSign;
            return;
        }

        int comparison =
            WideArithmetic.CompareMagnitudeEqualLength(left, right);
        if (comparison == 0)
        {
            result.Clear();
            resultSign = 0;
        }
        else if (comparison > 0)
        {
            WideArithmetic.SubtractEqualMagnitudes(
                left,
                right,
                result);
            resultSign = leftSign;
        }
        else
        {
            WideArithmetic.SubtractEqualMagnitudes(
                right,
                left,
                result);
            resultSign = rightSign;
        }
    }
}
