//=======================================================================
// WideOrientedBox.ConvexHullCapsule.cs
//=======================================================================
// MIT License, Copyright (c) 2024-present David Oravsky (mrdav30)
// See LICENSE file in the project root for full license information.
//=======================================================================

using System;

namespace FixedMathSharp.Bounds;

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
        RationalBasis hullBasis = new(hullRotation);
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

        HullCapsuleCandidate best = default;
        bool hasBest = false;
        for (int index = 0;
            index < triangleVertexIndices.Length;
            index += 3)
        {
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
                    ref hasBest))
            {
                contact = default;
                return false;
            }
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
                    ref hasBest))
            {
                contact = default;
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
                    ref hasBest))
            {
                contact = default;
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
                        ref hasBest))
                {
                    contact = default;
                    return false;
                }
            }
        }

        if (!hasBest)
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
        Vector3d normal = WideGeometry.GetNormalized(
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

    private static bool TryKeepHullCapsuleCandidate(
        HullCapsuleCandidate candidate,
        Vector3d hullOrigin,
        RationalBasis hullBasis,
        ReadOnlySpan<Vector3d> hullLocalPoints,
        ReadOnlySpan<int> triangleVertexIndices,
        ReadOnlySpan<int> edgeVertexPairs,
        Vector3d capsuleCenter,
        WideAxis3 capsuleAxis,
        Signed192 capsuleAxisDenominator,
        Fixed64 capsuleAxisLength,
        Fixed64 capsuleRadius,
        ref HullCapsuleCandidate best,
        ref bool hasBest)
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
            out _);
        if (status < 0)
            return false;
        if (status == 0)
            return true;
        if (!hasBest)
        {
            best = candidate;
            hasBest = true;
            return true;
        }

        Span<ulong> bestRational =
            stackalloc ulong[HullCapsuleMagnitudeWords];
        Span<ulong> bestSquaredAxisLength =
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
            bestRational,
            out int bestRationalSign,
            bestSquaredAxisLength,
            out _,
            out _,
            out _);
        if (WideArithmetic.CompareSignedNormalizedMagnitudes(
                rational,
                rationalSign,
                squaredAxisLength,
                bestRational,
                bestRationalSign,
                bestSquaredAxisLength) < 0)
        {
            best = candidate;
        }
        return true;
    }

    // Returns -1 for separation, 0 for a degenerate axis, and 1 for overlap.
    private static int EvaluateHullCapsuleCandidate(
        HullCapsuleCandidate candidate,
        Vector3d hullOrigin,
        RationalBasis hullBasis,
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
        out Signed192 commonDenominator)
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
        return WideArithmetic.GetSignedMagnitudeAndSquareRootSign(
                rational,
                rationalSign,
                coefficient,
                radialCoefficient.Sign,
                squaredAxisLength) < 0
            ? -1
            : 1;
    }

    private static CapsuleAxis3 BuildHullCapsuleCandidateAxis(
        HullCapsuleCandidate candidate,
        Vector3d hullOrigin,
        RationalBasis hullBasis,
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
            Vector3d first = hullLocalPoints[
                triangleVertexIndices[candidate.Index]];
            Vector3d second = hullLocalPoints[
                triangleVertexIndices[candidate.Index + 1]];
            Vector3d third = hullLocalPoints[
                triangleVertexIndices[candidate.Index + 2]];
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
            return ToCapsuleAxis(TransformLocalAxis(
                hullBasis,
                normalX,
                normalY,
                normalZ));
        }
        if (candidate.Kind == HullCapsuleCandidateKind.EdgeCross)
        {
            WideAxis3 edge = GetHullEdge(
                hullBasis,
                hullLocalPoints[edgeVertexPairs[candidate.Index]],
                hullLocalPoints[edgeVertexPairs[candidate.Index + 1]]);
            return ToCapsuleAxis(Cross(edge, capsuleAxis));
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
            candidate.EndpointSign);
    }

    private static CapsuleAxis3 GetCapsuleEndpointToHullEdgeAxis(
        Vector3d hullOrigin,
        Vector3d capsuleCenter,
        RationalBasis hullBasis,
        Vector3d localStart,
        Vector3d localEnd,
        WideAxis3 capsuleAxis,
        Signed192 capsuleAxisDenominator,
        Fixed64 capsuleAxisLength,
        int endpointSign)
    {
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
        Signed576 edgeSquared = GetSquaredLength(edge);
        if (edgeSquared.IsZero)
            return ToCapsuleAxis(startToEndpoint);

        Signed576 projection =
            GetAxisProjection(startToEndpoint, edge);
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

        return GetPerpendicularAxis(
            startToEndpoint,
            edge,
            edgeSquared,
            projection);
    }

}
