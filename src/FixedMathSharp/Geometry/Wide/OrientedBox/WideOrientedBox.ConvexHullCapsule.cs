//=======================================================================
// WideOrientedBox.ConvexHullCapsule.cs
//=======================================================================
// MIT License, Copyright (c) 2024-present David Oravsky (mrdav30)
// See LICENSE file in the project root for full license information.
//=======================================================================

using System;

namespace FixedMathSharp.Geometry;

/// <content>
/// Provides wide-precision convex hull versus capsule contact generation,
/// testing face, edge, and vertex candidates to resolve the deepest
/// separating axis between an oriented convex hull and a capsule.
/// </content>
internal static partial class WideOrientedBox
{
    private const int HullCapsuleMagnitudeWords = 64;

    internal static bool TryGetConvexHullCenteredCapsuleContact(
        Vector3d hullOrigin,
        FixedQuaternion hullRotation,
        ReadOnlySpan<Vector3d> hullLocalPoints,
        ReadOnlySpan<int> triangleVertexIndices,
        ReadOnlySpan<int> edgeVertexPairs,
        Vector3d capsuleCenter,
        FixedQuaternion capsuleRotation,
        Vector3d capsuleLocalAxisDirection,
        Fixed64 capsuleAxisLength,
        Fixed64 capsuleRadius,
        out FixedContactAnchors contact)
    {
        WideRationalBasis3d hullBasis = new(hullRotation);
        GetRotatedLocalAxisNumerators(
            capsuleRotation,
            capsuleLocalAxisDirection,
            out Signed192 capsuleAxisX,
            out Signed192 capsuleAxisY,
            out Signed192 capsuleAxisZ,
            out Signed192 capsuleAxisDenominator);
        var capsuleAxis = new WideAxis3(
            Signed320.ExtendValue(capsuleAxisX),
            Signed320.ExtendValue(capsuleAxisY),
            Signed320.ExtendValue(capsuleAxisZ));

        if (!TryFindHullCapsuleCandidate(
                hullOrigin,
                hullRotation,
                hullBasis,
                hullLocalPoints,
                triangleVertexIndices,
                edgeVertexPairs,
                capsuleCenter,
                capsuleAxis,
                capsuleAxisDenominator,
                capsuleAxisLength,
                capsuleRadius,
                strict: false,
                out HullCapsuleCandidate best))
        {
            contact = default;
            return false;
        }

        Span<ulong> rational =
            stackalloc ulong[HullCapsuleMagnitudeWords];
        Span<ulong> squaredAxisLength =
            stackalloc ulong[HullCapsuleMagnitudeWords];
        _ = EvaluateHullCapsuleCandidate(
            best,
            hullOrigin,
            hullBasis,
            hullLocalPoints,
            triangleVertexIndices,
            edgeVertexPairs,
            capsuleCenter,
            capsuleAxis,
            capsuleAxisDenominator,
            capsuleAxisLength,
            capsuleRadius,
            rational,
            out int rationalSign,
            squaredAxisLength,
            out CapsuleAxis3 axis,
            out bool negate,
            out Signed192 commonDenominator);
        CapsuleAxis3 orientedAxis = negate ? -axis : axis;
        Vector3d normal = WideNormalization.GetNormalized(
            orientedAxis.X,
            orientedAxis.Y,
            orientedAxis.Z);
        int capsuleAxialSign = -GetCapsuleAxisProjection(
            orientedAxis,
            capsuleAxis).Sign;
        FixedPointAnchor capsuleAnchor =
            WideGeometry.GetCenteredCapsuleSupportAnchor(
                capsuleCenter,
                capsuleRotation,
                capsuleLocalAxisDirection,
                capsuleAxisLength,
                capsuleRadius,
                -normal,
                capsuleAxialSign);
        FixedPointAnchor hullAnchor =
            GetMatchedHullCapsuleSupportAnchor(
                hullOrigin,
                hullRotation,
                hullBasis,
                hullLocalPoints,
                triangleVertexIndices,
                edgeVertexPairs,
                orientedAxis,
                capsuleAnchor);
        GetHullCapsuleDepth(
            rational,
            rationalSign,
            squaredAxisLength,
            commonDenominator,
            capsuleRadius,
            out Fixed64 depth,
            out bool depthIsClamped);
        contact = new FixedContactAnchors(
            hullAnchor,
            capsuleAnchor,
            normal,
            depth,
            depthIsClamped);
        return true;
    }

    /// <summary>
    /// Tests positive overlap against every exact hull-capsule separating-axis
    /// candidate without rounding depth or constructing contact witnesses.
    /// Inputs have the same admitted geometry contract as the contact kernel.
    /// A zero-length capsule is a sphere; tangency is not positive overlap.
    /// </summary>
    internal static bool DoesConvexHullCenteredCapsuleStrictlyOverlap(
        Vector3d hullOrigin,
        FixedQuaternion hullRotation,
        ReadOnlySpan<Vector3d> hullLocalPoints,
        ReadOnlySpan<int> triangleVertexIndices,
        ReadOnlySpan<int> edgeVertexPairs,
        Vector3d capsuleCenter,
        FixedQuaternion capsuleRotation,
        Vector3d capsuleLocalAxisDirection,
        Fixed64 capsuleAxisLength,
        Fixed64 capsuleRadius)
    {
        WideRationalBasis3d hullBasis = new(hullRotation);
        GetRotatedLocalAxisNumerators(
            capsuleRotation,
            capsuleLocalAxisDirection,
            out Signed192 capsuleAxisX,
            out Signed192 capsuleAxisY,
            out Signed192 capsuleAxisZ,
            out Signed192 capsuleAxisDenominator);
        var capsuleAxis = new WideAxis3(
            Signed320.ExtendValue(capsuleAxisX),
            Signed320.ExtendValue(capsuleAxisY),
            Signed320.ExtendValue(capsuleAxisZ));
        return TryFindHullCapsuleCandidate(
            hullOrigin,
            hullRotation,
            hullBasis,
            hullLocalPoints,
            triangleVertexIndices,
            edgeVertexPairs,
            capsuleCenter,
            capsuleAxis,
            capsuleAxisDenominator,
            capsuleAxisLength,
            capsuleRadius,
            strict: true,
            out _);
    }

    private static bool TryFindHullCapsuleCandidate(
        Vector3d hullOrigin,
        FixedQuaternion hullRotation,
        WideRationalBasis3d hullBasis,
        ReadOnlySpan<Vector3d> hullLocalPoints,
        ReadOnlySpan<int> triangleVertexIndices,
        ReadOnlySpan<int> edgeVertexPairs,
        Vector3d capsuleCenter,
        WideAxis3 capsuleAxis,
        Signed192 capsuleAxisDenominator,
        Fixed64 capsuleAxisLength,
        Fixed64 capsuleRadius,
        bool strict,
        out HullCapsuleCandidate best)
    {
        best = default;
        bool hasBest = false;
        Span<ulong> bestRational = stackalloc ulong[HullCapsuleMagnitudeWords];
        Span<ulong> bestSquaredAxisLength = stackalloc ulong[HullCapsuleMagnitudeWords];
        int bestRationalSign = 0;
        WideAxis3 previousFaceNormal = default;
        for (int index = 0;
            index < triangleVertexIndices.Length;
            index += 3)
        {
            WideAxis3 faceNormal = GetHullTriangleNormal(
                hullLocalPoints, triangleVertexIndices, index);
            // Parallel local normals define the same whole-hull slab, even
            // with opposite winding. Preserve the first authored face on ties.
            // Raw-coordinate triangle normals need at most 130 signed bits;
            // their cross products therefore fit Signed320 exactly.
            if (!faceNormal.IsZero && !previousFaceNormal.IsZero
                && WideAxis3.Cross(previousFaceNormal, faceNormal).IsZero)
            {
                continue;
            }
            previousFaceNormal = faceNormal;
            if (!TryKeepHullCapsuleCandidate(
                    new HullCapsuleCandidate(
                        HullCapsuleCandidateKind.Face,
                        index,
                        0),
                    hullOrigin,
                    hullBasis,
                    hullLocalPoints,
                    triangleVertexIndices,
                    edgeVertexPairs,
                    capsuleCenter,
                    capsuleAxis,
                    capsuleAxisDenominator,
                    capsuleAxisLength,
                    capsuleRadius,
                    ref best,
                    ref hasBest,
                    bestRational,
                    ref bestRationalSign,
                    bestSquaredAxisLength,
                    strict))
            {
                return false;
            }
        }

        if (capsuleAxisLength == Fixed64.Zero && hasBest)
        {
            // A positive minimum face margin puts the center inside the hull.
            // Its nearest facet is the sphere's minimum exit; strict queries
            // retain only the last face and therefore cannot use this proof.
            if (!strict && bestRationalSign > 0)
                return true;

            if (!strict)
            {
                _ = EvaluateHullCapsuleCandidate(best, hullOrigin, hullBasis,
                    hullLocalPoints, triangleVertexIndices, edgeVertexPairs,
                    capsuleCenter, capsuleAxis, capsuleAxisDenominator,
                    capsuleAxisLength, capsuleRadius, bestRational,
                    out bestRationalSign, bestSquaredAxisLength,
                    out CapsuleAxis3 faceAxis, out bool negate, out _);
                if (ContainsHullSphereSupportProjection(hullOrigin, hullRotation,
                        hullBasis, hullLocalPoints, triangleVertexIndices,
                        negate ? -faceAxis : faceAxis, capsuleCenter))
                    return true;
            }

            // For a sphere, a closest face-interior point is represented by
            // the face axes. Otherwise the closest finite edge or vertex
            // supplies the remaining radial axis. Keep the old degenerate-
            // hull path when no nondegenerate face supplies a plane normal.
            if (!TryGetClosestHullSphereFeature(
                    hullOrigin, hullBasis, hullLocalPoints, edgeVertexPairs,
                    capsuleCenter, capsuleAxis, capsuleAxisDenominator,
                    out HullCapsuleCandidate closest))
            {
                return hasBest;
            }
            return TryKeepHullCapsuleCandidate(
                closest,
                hullOrigin,
                hullBasis,
                hullLocalPoints,
                triangleVertexIndices,
                edgeVertexPairs,
                capsuleCenter,
                capsuleAxis,
                capsuleAxisDenominator,
                capsuleAxisLength,
                capsuleRadius,
                ref best,
                ref hasBest,
                bestRational,
                ref bestRationalSign,
                bestSquaredAxisLength,
                strict);
        }

        for (int index = 0; index < edgeVertexPairs.Length; index += 2)
        {
            if (!TryKeepHullCapsuleCandidate(
                    new HullCapsuleCandidate(
                        HullCapsuleCandidateKind.EdgeCross,
                        index,
                        0),
                    hullOrigin,
                    hullBasis,
                    hullLocalPoints,
                    triangleVertexIndices,
                    edgeVertexPairs,
                    capsuleCenter,
                    capsuleAxis,
                    capsuleAxisDenominator,
                    capsuleAxisLength,
                    capsuleRadius,
                    ref best,
                    ref hasBest,
                    bestRational,
                    ref bestRationalSign,
                    bestSquaredAxisLength,
                    strict))
            {
                return false;
            }
        }

        for (int index = 0; index < hullLocalPoints.Length; index++)
        {
            if (!TryKeepHullCapsuleCandidate(
                    new HullCapsuleCandidate(
                        HullCapsuleCandidateKind.VertexCore,
                        index,
                        0),
                    hullOrigin,
                    hullBasis,
                    hullLocalPoints,
                    triangleVertexIndices,
                    edgeVertexPairs,
                    capsuleCenter,
                    capsuleAxis,
                    capsuleAxisDenominator,
                    capsuleAxisLength,
                    capsuleRadius,
                    ref best,
                    ref hasBest,
                    bestRational,
                    ref bestRationalSign,
                    bestSquaredAxisLength,
                    strict))
            {
                return false;
            }
        }

        for (int index = 0; index < edgeVertexPairs.Length; index += 2)
        {
            for (int endpointSign = -1;
                endpointSign <= 1;
                endpointSign += 2)
            {
                if (!TryKeepHullCapsuleCandidate(
                        new HullCapsuleCandidate(
                            HullCapsuleCandidateKind.EndpointEdge,
                            index,
                            endpointSign),
                        hullOrigin,
                        hullBasis,
                        hullLocalPoints,
                        triangleVertexIndices,
                        edgeVertexPairs,
                        capsuleCenter,
                        capsuleAxis,
                        capsuleAxisDenominator,
                        capsuleAxisLength,
                        capsuleRadius,
                        ref best,
                        ref hasBest,
                        bestRational,
                        ref bestRationalSign,
                        bestSquaredAxisLength,
                        strict))
                {
                    return false;
                }
            }
        }

        return hasBest;
    }

    private static bool TryGetClosestHullSphereFeature(
        Vector3d hullOrigin,
        WideRationalBasis3d hullBasis,
        ReadOnlySpan<Vector3d> points,
        ReadOnlySpan<int> edgeVertexPairs,
        Vector3d sphereCenter,
        WideAxis3 capsuleAxis,
        Signed192 capsuleAxisDenominator,
        out HullCapsuleCandidate closest)
    {
        closest = default;
        bool hasClosest = false;
        Span<ulong> denominator = stackalloc ulong[HullCapsuleMagnitudeWords];
        Span<ulong> squaredRay = stackalloc ulong[HullCapsuleMagnitudeWords];
        Span<ulong> bestDenominator = stackalloc ulong[HullCapsuleMagnitudeWords];
        Span<ulong> bestSquaredRay = stackalloc ulong[HullCapsuleMagnitudeWords];
        // All zero-core rays share the same positive rigid-frame scale.
        // An interior-edge ray also retains |edge|^2: maximizing this
        // denominator / |ray| selects the nearest exact feature without
        // rounding witnesses. Existing span comparisons retain the full
        // endpoint/edge product widths even at unrepresentable world points.
        for (int featureKind = 0; featureKind < 2; featureKind++)
        {
            bool isVertex = featureKind == 0;
            int featureCount = isVertex ? points.Length : edgeVertexPairs.Length / 2;
            for (int index = 0; index < featureCount; index++)
            {
                int featureIndex = isVertex ? index : index * 2;
                var candidate = new HullCapsuleCandidate(
                    isVertex ? HullCapsuleCandidateKind.VertexCore
                        : HullCapsuleCandidateKind.EndpointEdge,
                    featureIndex,
                    isVertex ? 0 : -1);
                Signed576 distanceDenominator =
                    Signed576.ExtendValue(Signed320.ExtendValue(Signed192.Signed(1L)));
                CapsuleAxis3 ray = isVertex
                    ? ToCapsuleAxis(GetCornerToAxisEndpointAxis3D(
                        hullOrigin, sphereCenter, hullBasis, points[index],
                        capsuleAxis, capsuleAxisDenominator, Fixed64.Zero, 0))
                    : GetCapsuleEndpointToHullEdgeAxis(
                        hullOrigin, sphereCenter, hullBasis,
                        points[edgeVertexPairs[featureIndex]],
                        points[edgeVertexPairs[featureIndex + 1]],
                        capsuleAxis, capsuleAxisDenominator, Fixed64.Zero, -1,
                        out distanceDenominator);
                // At distance zero no radial direction is defined; retain
                // the admitted face constraints instead of dividing by zero.
                if (ray.IsZero)
                    return false;
                GetHullCapsuleMagnitude(distanceDenominator, denominator);
                BuildHullCapsuleAxisSquared(ray, squaredRay);
                if (!hasClosest || WideArithmetic.CompareSignedNormalizedMagnitudes(
                        denominator, 1, squaredRay,
                        bestDenominator, 1, bestSquaredRay) > 0)
                {
                    closest = candidate;
                    hasClosest = true;
                    denominator.CopyTo(bestDenominator);
                    squaredRay.CopyTo(bestSquaredRay);
                }
            }
        }
        return hasClosest;
    }

    private static bool TryKeepHullCapsuleCandidate(
        HullCapsuleCandidate candidate,
        Vector3d hullOrigin,
        WideRationalBasis3d hullBasis,
        ReadOnlySpan<Vector3d> hullLocalPoints,
        ReadOnlySpan<int> triangleVertexIndices,
        ReadOnlySpan<int> edgeVertexPairs,
        Vector3d capsuleCenter,
        WideAxis3 capsuleAxis,
        Signed192 capsuleAxisDenominator,
        Fixed64 capsuleAxisLength,
        Fixed64 capsuleRadius,
        ref HullCapsuleCandidate best,
        ref bool hasBest,
        Span<ulong> bestRational,
        ref int bestRationalSign,
        Span<ulong> bestSquaredAxisLength,
        bool strict)
    {
        Span<ulong> rational =
            stackalloc ulong[HullCapsuleMagnitudeWords];
        Span<ulong> squaredAxisLength =
            stackalloc ulong[HullCapsuleMagnitudeWords];
        int status = EvaluateHullCapsuleCandidate(
            candidate,
            hullOrigin,
            hullBasis,
            hullLocalPoints,
            triangleVertexIndices,
            edgeVertexPairs,
            capsuleCenter,
            capsuleAxis,
            capsuleAxisDenominator,
            capsuleAxisLength,
            capsuleRadius,
            rational,
            out int rationalSign,
            squaredAxisLength,
            out _,
            out _,
            out _,
            strict);
        if (status < 0)
            return false;
        if (status == 0)
            return true;
        // The winning exact metric does not change until another axis wins.
        // Retain it rather than projecting the whole hull again on each comparison.
        if (strict || !hasBest || WideArithmetic.CompareSignedNormalizedMagnitudes(
                rational,
                rationalSign,
                squaredAxisLength,
                bestRational,
                bestRationalSign,
                bestSquaredAxisLength) < 0)
        {
            best = candidate;
            hasBest = true;
            rational.CopyTo(bestRational);
            bestRationalSign = rationalSign;
            squaredAxisLength.CopyTo(bestSquaredAxisLength);
        }
        return true;
    }

    // Returns -1 for separation, 0 for a degenerate axis, and 1 for overlap.
    private static int EvaluateHullCapsuleCandidate(
        HullCapsuleCandidate candidate,
        Vector3d hullOrigin,
        WideRationalBasis3d hullBasis,
        ReadOnlySpan<Vector3d> hullLocalPoints,
        ReadOnlySpan<int> triangleVertexIndices,
        ReadOnlySpan<int> edgeVertexPairs,
        Vector3d capsuleCenter,
        WideAxis3 capsuleAxis,
        Signed192 capsuleAxisDenominator,
        Fixed64 capsuleAxisLength,
        Fixed64 capsuleRadius,
        Span<ulong> rational,
        out int rationalSign,
        Span<ulong> squaredAxisLength,
        out CapsuleAxis3 axis,
        out bool negate,
        out Signed192 commonDenominator,
        bool strict = false)
    {
        axis = BuildHullCapsuleCandidateAxis(
            candidate,
            hullOrigin,
            hullBasis,
            hullLocalPoints,
            triangleVertexIndices,
            edgeVertexPairs,
            capsuleCenter,
            capsuleAxis,
            capsuleAxisDenominator,
            capsuleAxisLength);
        rational.Clear();
        squaredAxisLength.Clear();
        rationalSign = 0;
        negate = false;
        commonDenominator = default;
        if (axis.IsZero)
            return 0;

        // Axis magnitude has no geometric meaning. Remove common rational
        // frame scale before projection; retain the original full-width axis
        // when its components exceed the existing common-scale reducer.
        if (Signed320.TryNarrowSigned(axis.X, out Signed320 axisX)
            && Signed320.TryNarrowSigned(axis.Y, out Signed320 axisY)
            && Signed320.TryNarrowSigned(axis.Z, out Signed320 axisZ))
        {
            Span<Signed320> components = stackalloc Signed320[3] { axisX, axisY, axisZ };
            WideArithmetic.ReduceCommonScale(components);
            axis = ToCapsuleAxis(new WideAxis3(components[0], components[1], components[2]));
        }

        BuildHullCapsuleAxisSquared(axis, squaredAxisLength);
        Span<ulong> minimum =
            stackalloc ulong[HullCapsuleMagnitudeWords];
        Span<ulong> maximum =
            stackalloc ulong[HullCapsuleMagnitudeWords];
        GetHullCapsuleProjectionInterval(
            hullLocalPoints,
            hullBasis,
            axis,
            minimum,
            out int minimumSign,
            maximum,
            out int maximumSign);

        Signed704 centerProjection = GetHullCapsuleCenterProjection(
            capsuleCenter,
            hullOrigin,
            axis);
        Span<ulong> centerMagnitude =
            stackalloc ulong[HullCapsuleMagnitudeWords];
        centerMagnitude.Clear();
        GetHullCapsuleMagnitude(
            centerProjection,
            centerMagnitude);
        Span<ulong> hullDenominatorMagnitude =
            stackalloc ulong[HullCapsuleMagnitudeWords];
        GetHullCapsuleMagnitude(
            hullBasis.Denominator,
            hullDenominatorMagnitude);
        Span<ulong> scaledCenter =
            stackalloc ulong[HullCapsuleMagnitudeWords];
        WideArithmetic.MultiplyMagnitudes(
            centerMagnitude,
            hullDenominatorMagnitude,
            scaledCenter);

        Span<ulong> positive =
            stackalloc ulong[HullCapsuleMagnitudeWords];
        CombineHullCapsuleSignedMagnitudes(
            maximum,
            maximumSign,
            scaledCenter,
            -centerProjection.Sign,
            positive,
            out int positiveSign);
        Span<ulong> negative =
            stackalloc ulong[HullCapsuleMagnitudeWords];
        CombineHullCapsuleSignedMagnitudes(
            scaledCenter,
            centerProjection.Sign,
            minimum,
            -minimumSign,
            negative,
            out int negativeSign);

        Signed320 baseScale = WideArithmetic.MultiplySigned192(
            capsuleAxisDenominator,
            Signed192.Raw(Fixed64.Two));
        MultiplyHullCapsuleMagnitude(
            positive,
            baseScale,
            positive);
        MultiplyHullCapsuleMagnitude(
            negative,
            baseScale,
            negative);

        Signed704 alignment =
            GetCapsuleAxisProjection(axis, capsuleAxis);
        Span<ulong> axial =
            stackalloc ulong[HullCapsuleMagnitudeWords];
        axial.Clear();
        GetHullCapsuleMagnitude(alignment, axial);
        MultiplyHullCapsuleMagnitude(
            axial,
            Signed192.Raw(capsuleAxisLength),
            axial);
        MultiplyHullCapsuleMagnitude(
            axial,
            hullBasis.Denominator,
            axial);
        CombineHullCapsuleSignedMagnitudes(
            positive,
            positiveSign,
            axial,
            1,
            positive,
            out positiveSign);
        CombineHullCapsuleSignedMagnitudes(
            negative,
            negativeSign,
            axial,
            1,
            negative,
            out negativeSign);

        int directionComparison =
            CompareHullCapsuleSignedMagnitudes(
                negative,
                negativeSign,
                positive,
                positiveSign);
        negate = directionComparison < 0;
        ReadOnlySpan<ulong> selected =
            negate ? negative : positive;
        rationalSign = negate ? negativeSign : positiveSign;
        selected.CopyTo(rational);

        Signed576 commonWide = WideArithmetic.MultiplySigned576(
            Signed576.ExtendValue(
                WideArithmetic.MultiplySigned192(
                    hullBasis.Denominator,
                    capsuleAxisDenominator)),
            Signed192.Raw(Fixed64.Two));
        _ = Signed192.TryNarrowSigned(
            commonWide,
            out commonDenominator);
        Signed320 radialCoefficient =
            WideArithmetic.MultiplySigned192(
                commonDenominator,
                Signed192.Raw(capsuleRadius));
        Span<ulong> coefficient =
            stackalloc ulong[HullCapsuleMagnitudeWords];
        GetHullCapsuleMagnitude(
            radialCoefficient,
            coefficient);
        int overlapSign = WideArithmetic.GetSignedMagnitudeAndSquareRootSign(
                rational,
                rationalSign,
                coefficient,
                radialCoefficient.Sign,
                squaredAxisLength);
        return overlapSign < 0 || (overlapSign == 0 && strict)
            ? -1
            : 1;
    }

    private static WideAxis3 GetHullTriangleNormal(
        ReadOnlySpan<Vector3d> points,
        ReadOnlySpan<int> triangleVertexIndices,
        int index)
    {
        Vector3d first = points[
            triangleVertexIndices[index]];
        Vector3d second = points[
            triangleVertexIndices[index + 1]];
        Vector3d third = points[
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
        return new WideAxis3(
            Signed320.ExtendValue(normalX),
            Signed320.ExtendValue(normalY),
            Signed320.ExtendValue(normalZ));
    }

    private static CapsuleAxis3 BuildHullCapsuleCandidateAxis(
        HullCapsuleCandidate candidate,
        Vector3d hullOrigin,
        WideRationalBasis3d hullBasis,
        ReadOnlySpan<Vector3d> hullLocalPoints,
        ReadOnlySpan<int> triangleVertexIndices,
        ReadOnlySpan<int> edgeVertexPairs,
        Vector3d capsuleCenter,
        WideAxis3 capsuleAxis,
        Signed192 capsuleAxisDenominator,
        Fixed64 capsuleAxisLength)
    {
        if (candidate.Kind == HullCapsuleCandidateKind.Face)
        {
            WideAxis3 normal = GetHullTriangleNormal(
                hullLocalPoints, triangleVertexIndices, candidate.Index);
            return ToCapsuleAxis(WideRigidProjection.TransformLocalAxis(
                hullBasis,
                Signed192.NarrowValue(normal.X),
                Signed192.NarrowValue(normal.Y),
                Signed192.NarrowValue(normal.Z)));
        }
        if (candidate.Kind == HullCapsuleCandidateKind.EdgeCross)
        {
            WideAxis3 edge = GetHullEdge(
                hullBasis,
                hullLocalPoints[edgeVertexPairs[candidate.Index]],
                hullLocalPoints[edgeVertexPairs[candidate.Index + 1]]);
            return ToCapsuleAxis(WideAxis3.Cross(edge, capsuleAxis));
        }
        if (candidate.Kind == HullCapsuleCandidateKind.VertexCore)
        {
            return GetVertexToCapsuleAxis(
                hullOrigin,
                capsuleCenter,
                hullBasis,
                hullLocalPoints[candidate.Index],
                capsuleAxis,
                capsuleAxisDenominator,
                capsuleAxisLength);
        }

        return GetCapsuleEndpointToHullEdgeAxis(
            hullOrigin,
            capsuleCenter,
            hullBasis,
            hullLocalPoints[edgeVertexPairs[candidate.Index]],
            hullLocalPoints[edgeVertexPairs[candidate.Index + 1]],
            capsuleAxis,
            capsuleAxisDenominator,
            capsuleAxisLength,
            candidate.EndpointSign,
            out _);
    }

    private static CapsuleAxis3 GetCapsuleEndpointToHullEdgeAxis(
        Vector3d hullOrigin,
        Vector3d capsuleCenter,
        WideRationalBasis3d hullBasis,
        Vector3d localStart,
        Vector3d localEnd,
        WideAxis3 capsuleAxis,
        Signed192 capsuleAxisDenominator,
        Fixed64 capsuleAxisLength,
        int endpointSign,
        out Signed576 distanceDenominator)
    {
        distanceDenominator = Signed576.ExtendValue(Signed320.ExtendValue(Signed192.Signed(1L)));
        WideAxis3 startToEndpoint = GetCornerToAxisEndpointAxis3D(
            hullOrigin,
            capsuleCenter,
            hullBasis,
            localStart,
            capsuleAxis,
            capsuleAxisDenominator,
            capsuleAxisLength,
            endpointSign);
        WideAxis3 edge = GetHullEdge(
            hullBasis,
            localStart,
            localEnd);
        Signed576 edgeSquared = edge.SquaredLength;
        if (edgeSquared.IsZero)
            return ToCapsuleAxis(startToEndpoint);

        Signed576 projection =
            WideAxis3.Dot(startToEndpoint, edge);
        if (projection.Sign <= 0)
            return ToCapsuleAxis(startToEndpoint);
        Signed576 upperBound = WideArithmetic.MultiplySigned576(
            WideArithmetic.MultiplySigned576(
                edgeSquared,
                capsuleAxisDenominator),
            Signed192.Raw(Fixed64.Two));
        if (WideArithmetic.CompareNonNegative(
                projection,
                upperBound) >= 0)
        {
            return ToCapsuleAxis(GetCornerToAxisEndpointAxis3D(
                hullOrigin,
                capsuleCenter,
                hullBasis,
                localEnd,
                capsuleAxis,
                capsuleAxisDenominator,
                capsuleAxisLength,
                endpointSign));
        }

        distanceDenominator = edgeSquared;
        return GetPerpendicularAxis(
            startToEndpoint,
            edge,
            edgeSquared,
            projection);
    }

}
