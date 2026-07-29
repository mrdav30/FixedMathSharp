//=======================================================================
// WideCenteredCapsule2dRelations.cs
//=======================================================================
// MIT License, Copyright (c) 2024–present David Oravsky (mrdav30)
// See LICENSE file in the project root for full license information.
//=======================================================================

using System;

namespace FixedMathSharp.Geometry;

/// <summary>
/// Owns rigid-frame full-domain relations for conceptual 2D capsules.
/// </summary>
internal static class WideCenteredCapsule2dRelations
{
    private static readonly Fixed64 SideAlignmentTolerance =
        Fixed64.Epsilon * (Fixed64)16;
    private static readonly Signed192 DoubleScaleSquared =
        new(0UL, 2UL, 0UL);
    private static readonly Signed192 ScaleSquared =
        new(0UL, 1UL, 0UL);

    internal static bool TryGetMinimumTranslation(
        Vector2d center,
        Vector2d capsuleAxis,
        Fixed64 axisLength,
        Fixed64 radius,
        Vector2d convexOrigin,
        ReadOnlySpan<Vector2d> convexOriginOffsets,
        out Vector2d normal,
        out Fixed64 depth) =>
        TryGetMinimumTranslation(
            center,
            capsuleAxis,
            axisLength,
            radius,
            convexOrigin,
            Fixed64.Zero,
            convexOriginOffsets,
            out normal,
            out depth);

    internal static bool TryGetMinimumTranslation(
        Vector2d center,
        Vector2d capsuleAxis,
        Fixed64 axisLength,
        Fixed64 radius,
        Vector2d convexOrigin,
        Fixed64 convexRotation,
        ReadOnlySpan<Vector2d> convexOriginOffsets,
        out Vector2d normal,
        out Fixed64 depth) =>
        TryGetMinimumTranslation(
            center,
            capsuleAxis,
            axisLength,
            radius,
            convexOrigin,
            convexRotation,
            convexOriginOffsets,
            out normal,
            out depth,
            out _,
            out _);

    internal static bool TryGetContacts(
        Vector2d capsuleCenter,
        Fixed64 capsuleRotation,
        Vector2d localCapsuleAxis,
        Fixed64 axisLength,
        Fixed64 radius,
        Vector2d convexOrigin,
        Fixed64 convexRotation,
        ReadOnlySpan<Vector2d> convexVertexOffsets,
        Span<FixedPointAnchor2d> capsuleContacts,
        Span<FixedPointAnchor2d> convexContacts,
        out int contactCount,
        out Vector2d normal,
        out Fixed64 depth,
        out bool depthIsClamped)
    {
        _ = Vector2d.TryRotate(
            localCapsuleAxis,
            capsuleRotation,
            out Vector2d capsuleAxis);
        if (!TryGetMinimumTranslation(
                capsuleCenter,
                capsuleAxis,
                axisLength,
                radius,
                convexOrigin,
                convexRotation,
                convexVertexOffsets,
                out normal,
                out depth,
                out depthIsClamped,
                out bool axisFromEdge))
        {
            contactCount = default;
            return false;
        }

        _ = Vector2d.TryRotate(
            normal,
            -capsuleRotation,
            out Vector2d localNormal);
        localNormal = localNormal.Normalized;
        GetSupportFeature(
            capsuleCenter,
            convexOrigin,
            convexRotation,
            convexVertexOffsets,
            normal,
            axisFromEdge,
            out Vector2d featureStart,
            out Vector2d featureEnd);
        Vector2d featureDirection = WideNormalization.GetDirection(featureStart, featureEnd);
        if (axisLength > Fixed64.Epsilon
            && featureDirection != Vector2d.Zero
            && Vector2d.Dot(capsuleAxis, normal).Abs()
                <= SideAlignmentTolerance)
        {
            FixedPointAnchor2d firstCapsuleAnchor =
                GetCenteredCapsuleSideAnchor(
                    capsuleCenter,
                    capsuleRotation,
                    localCapsuleAxis,
                    -axisLength,
                    localNormal,
                    radius);
            FixedPointAnchor2d secondCapsuleAnchor =
                GetCenteredCapsuleSideAnchor(
                    capsuleCenter,
                    capsuleRotation,
                    localCapsuleAxis,
                    axisLength,
                    localNormal,
                    radius);
            if (WideConvex2dRelations.TryProjectAnchorOntoFeature(
                    firstCapsuleAnchor,
                    convexOrigin,
                    convexRotation,
                    featureStart,
                    featureEnd,
                    requireInteriorProjection: true,
                    out Vector2d firstConvexOffset)
                && WideConvex2dRelations.TryProjectAnchorOntoFeature(
                    secondCapsuleAnchor,
                    convexOrigin,
                    convexRotation,
                    featureStart,
                    featureEnd,
                    requireInteriorProjection: true,
                    out Vector2d secondConvexOffset))
            {
                capsuleContacts[0] = firstCapsuleAnchor;
                capsuleContacts[1] = secondCapsuleAnchor;
                convexContacts[0] = new FixedPointAnchor2d(
                    convexOrigin,
                    convexRotation,
                    firstConvexOffset);
                convexContacts[1] = new FixedPointAnchor2d(
                    convexOrigin,
                    convexRotation,
                    secondConvexOffset);
                contactCount = 2;
                return true;
            }
        }

        FixedPointAnchor2d capsuleContact =
            WideFiniteAxisIntersection.GetCenteredCapsuleSupportAnchor(
                capsuleCenter,
                capsuleRotation,
                localCapsuleAxis,
                axisLength,
                radius,
                localNormal);
        _ = WideConvex2dRelations.TryProjectAnchorOntoFeature(
            capsuleContact,
            convexOrigin,
            convexRotation,
            featureStart,
            featureEnd,
            requireInteriorProjection: false,
            out Vector2d convexOffset);

        capsuleContacts[0] = capsuleContact;
        convexContacts[0] = new FixedPointAnchor2d(
            convexOrigin,
            convexRotation,
            convexOffset);
        contactCount = 1;
        return true;
    }

    private static FixedPointAnchor2d GetCenteredCapsuleSideAnchor(
        Vector2d center,
        Fixed64 rotation,
        Vector2d localAxis,
        Fixed64 signedAxisLength,
        Vector2d localRadialDirection,
        Fixed64 radius)
    {
        Vector2d axialOffset =
            localAxis * (signedAxisLength / Fixed64.Two);
        Vector2d radialOffset = localRadialDirection * radius;
        FixedPointAnchorTerm2d exactLocalTerm =
            FixedPointAnchorTerm2d.CreateCenteredAxisSupport(
                localAxis,
                signedAxisLength,
                localRadialDirection,
                radius,
                axialOffset,
                radialOffset);
        return new FixedPointAnchor2d(
            center,
            rotation,
            axialOffset,
            radialOffset,
            exactLocalTerm);
    }

    private static bool TryGetMinimumTranslation(
        Vector2d center,
        Vector2d capsuleAxis,
        Fixed64 axisLength,
        Fixed64 radius,
        Vector2d convexOrigin,
        Fixed64 convexRotation,
        ReadOnlySpan<Vector2d> convexOriginOffsets,
        out Vector2d normal,
        out Fixed64 depth,
        out bool depthIsClamped,
        out bool bestAxisFromEdge)
    {
        bool found = false;
        Signed576 bestDepth = default;
        Vector2d bestNormal = default;
        bestAxisFromEdge = false;

        for (int i = 0; i < convexOriginOffsets.Length; i++)
        {
            Vector2d edgeDirection = WideConvex2dRelations.GetTransformedEdgeDirection(
                convexRotation,
                convexOriginOffsets[i],
                convexOriginOffsets[(i + 1) % convexOriginOffsets.Length]);
            if (edgeDirection == Vector2d.Zero)
                continue;

            Vector2d edgeNormal = new(-edgeDirection.Y, edgeDirection.X);
            if (!TryKeepAxis(
                    center,
                    capsuleAxis,
                    axisLength,
                    radius,
                    convexOrigin,
                    convexRotation,
                    convexOriginOffsets,
                    edgeNormal,
                    true,
                    ref found,
                    ref bestDepth,
                    ref bestNormal,
                    ref bestAxisFromEdge))
            {
                normal = default;
                depth = default;
                depthIsClamped = default;
                return false;
            }
        }

        Vector2d closestAxis = GetClosestVertexAxis(
            center,
            capsuleAxis,
            axisLength,
            convexOrigin,
            convexRotation,
            convexOriginOffsets);
        if (closestAxis != Vector2d.Zero
            && !TryKeepAxis(
                center,
                capsuleAxis,
                axisLength,
                radius,
                convexOrigin,
                convexRotation,
                convexOriginOffsets,
                closestAxis,
                false,
                ref found,
                ref bestDepth,
                ref bestNormal,
                ref bestAxisFromEdge))
        {
            normal = default;
            depth = default;
            depthIsClamped = default;
            return false;
        }

        if (!found)
        {
            normal = default;
            depth = default;
            depthIsClamped = default;
            return false;
        }

        normal = bestNormal;
        if (!Fixed64.TryGetSignedRawRatio(
                bestDepth,
                Signed576.ExtendValue(
                    Signed320.ExtendValue(
                        DoubleScaleSquared)),
                out depth))
        {
            depth = Fixed64.MaxValue;
            depthIsClamped = true;
            return true;
        }

        depthIsClamped = false;
        return true;
    }

    private static Vector2d GetClosestVertexAxis(
        Vector2d center,
        Vector2d capsuleAxis,
        Fixed64 axisLength,
        Vector2d convexOrigin,
        Fixed64 convexRotation,
        ReadOnlySpan<Vector2d> convexOriginOffsets)
    {
        bool found = false;
        Vector2d bestAxis = default;
        Signed576 bestSquaredDistance = default;
        for (int i = 0; i < convexOriginOffsets.Length; i++)
        {
            Vector2d candidate = GetDirectionFromCenteredAxis(
                convexOrigin,
                convexRotation,
                convexOriginOffsets[i],
                center,
                capsuleAxis,
                axisLength,
                out Signed576 candidateSquaredDistance);
            if (candidate == Vector2d.Zero)
                continue;
            if (found
                && WideArithmetic.SubtractSigned576(
                    candidateSquaredDistance,
                    bestSquaredDistance).Sign >= 0)
            {
                continue;
            }

            found = true;
            bestAxis = candidate;
            bestSquaredDistance = candidateSquaredDistance;
        }

        return bestAxis;
    }

    private static Vector2d GetDirectionFromCenteredAxis(
        Vector2d pointOrigin,
        Fixed64 pointRotation,
        Vector2d pointOriginOffset,
        Vector2d axisCenter,
        Vector2d axisDirection,
        Fixed64 axisLength,
        out Signed576 squaredDistance)
    {
        WideConvex2dRelations.GetRelativePointNumerators(
            pointOrigin,
            pointRotation,
            pointOriginOffset,
            axisCenter,
            out Signed192 relativeX,
            out Signed192 relativeY);
        Signed320 projection = WideArithmetic.AddSigned320(
            WideArithmetic.MultiplySigned192(
                relativeX,
                Signed192.Signed(axisDirection.X.m_rawValue)),
            WideArithmetic.MultiplySigned192(
                relativeY,
                Signed192.Signed(axisDirection.Y.m_rawValue)));
        Signed320 doubleProjection =
            WideArithmetic.AddSigned320(projection, projection);
        Signed320 axialThreshold = WideArithmetic.MultiplySigned192(
            Signed192.Signed(axisLength.m_rawValue),
            ScaleSquared);

        Signed320 directionX;
        Signed320 directionY;
        if (WideArithmetic.SubtractSigned320(
                doubleProjection,
                axialThreshold).Sign > 0)
        {
            Signed192 twiceRelativeX =
                WideArithmetic.AddSigned192(relativeX, relativeX);
            Signed192 twiceRelativeY =
                WideArithmetic.AddSigned192(relativeY, relativeY);
            Signed192 endpointX = NarrowProven(
                WideArithmetic.MultiplySigned192(
                    Signed192.Signed(axisDirection.X.m_rawValue),
                    Signed192.Signed(axisLength.m_rawValue)));
            Signed192 endpointY = NarrowProven(
                WideArithmetic.MultiplySigned192(
                    Signed192.Signed(axisDirection.Y.m_rawValue),
                    Signed192.Signed(axisLength.m_rawValue)));
            directionX = WideArithmetic.MultiplySigned192(
                WideArithmetic.SubtractSigned192(
                    twiceRelativeX,
                    endpointX),
                ScaleSquared);
            directionY = WideArithmetic.MultiplySigned192(
                WideArithmetic.SubtractSigned192(
                    twiceRelativeY,
                    endpointY),
                ScaleSquared);
        }
        else if (WideArithmetic.AddSigned320(
                     doubleProjection,
                     axialThreshold).Sign < 0)
        {
            Signed192 twiceRelativeX =
                WideArithmetic.AddSigned192(relativeX, relativeX);
            Signed192 twiceRelativeY =
                WideArithmetic.AddSigned192(relativeY, relativeY);
            Signed192 endpointX = NarrowProven(
                WideArithmetic.MultiplySigned192(
                    Signed192.Signed(axisDirection.X.m_rawValue),
                    Signed192.Signed(axisLength.m_rawValue)));
            Signed192 endpointY = NarrowProven(
                WideArithmetic.MultiplySigned192(
                    Signed192.Signed(axisDirection.Y.m_rawValue),
                    Signed192.Signed(axisLength.m_rawValue)));
            directionX = WideArithmetic.MultiplySigned192(
                WideArithmetic.AddSigned192(
                    twiceRelativeX,
                    endpointX),
                ScaleSquared);
            directionY = WideArithmetic.MultiplySigned192(
                WideArithmetic.AddSigned192(
                    twiceRelativeY,
                    endpointY),
                ScaleSquared);
        }
        else
        {
            Signed192 narrowProjection = NarrowProven(projection);
            Signed320 residualX = WideArithmetic.SubtractSigned320(
                WideArithmetic.MultiplySigned192(
                    relativeX,
                    ScaleSquared),
                WideArithmetic.MultiplySigned192(
                    Signed192.Signed(
                        axisDirection.X.m_rawValue),
                    narrowProjection));
            Signed320 residualY = WideArithmetic.SubtractSigned320(
                WideArithmetic.MultiplySigned192(
                    relativeY,
                    ScaleSquared),
                WideArithmetic.MultiplySigned192(
                    Signed192.Signed(
                        axisDirection.Y.m_rawValue),
                    narrowProjection));
            directionX = WideArithmetic.AddSigned320(
                residualX,
                residualX);
            directionY = WideArithmetic.AddSigned320(
                residualY,
                residualY);
        }

        squaredDistance = WideArithmetic.AddSigned576(
            WideArithmetic.MultiplySigned320(directionX, directionX),
            WideArithmetic.MultiplySigned320(directionY, directionY));
        return WideNormalization.GetNormalized(directionX, directionY);
    }

    private static Signed192 NarrowProven(Signed320 value)
    {
        _ = Signed192.TryNarrowSigned(value, out Signed192 result);
        return result;
    }

    private static void GetSupportFeature(
        Vector2d center,
        Vector2d convexOrigin,
        Fixed64 convexRotation,
        ReadOnlySpan<Vector2d> convexOriginOffsets,
        Vector2d normal,
        bool axisFromEdge,
        out Vector2d featureStart,
        out Vector2d featureEnd)
    {
        int minimumIndex = 0;
        Signed192 minimum = GetRelativeProjection(
            center,
            convexOrigin,
            convexRotation,
            convexOriginOffsets[0],
            normal);
        for (int i = 1; i < convexOriginOffsets.Length; i++)
        {
            Signed192 projection = GetRelativeProjection(
                center,
                convexOrigin,
                convexRotation,
                convexOriginOffsets[i],
                normal);
            if (WideArithmetic.SubtractSigned192(projection, minimum).Sign < 0)
            {
                minimum = projection;
                minimumIndex = i;
            }
        }

        featureStart = convexOriginOffsets[minimumIndex];
        featureEnd = featureStart;
        if (!axisFromEdge)
            return;

        int previousIndex = minimumIndex;
        do
        {
            previousIndex =
                (previousIndex + convexOriginOffsets.Length - 1)
                % convexOriginOffsets.Length;
        }
        while (convexOriginOffsets[previousIndex] == featureStart);

        int nextIndex = minimumIndex;
        do
        {
            nextIndex = (nextIndex + 1) % convexOriginOffsets.Length;
        }
        while (convexOriginOffsets[nextIndex] == featureStart);

        Vector2d previous = convexOriginOffsets[previousIndex];
        Vector2d next = convexOriginOffsets[nextIndex];
        Vector2d previousDirection = WideConvex2dRelations.GetTransformedEdgeDirection(
            convexRotation,
            previous,
            featureStart);
        Vector2d nextDirection = WideConvex2dRelations.GetTransformedEdgeDirection(
            convexRotation,
            featureStart,
            next);

        Fixed64 previousAlignment = Vector2d.Dot(previousDirection, normal).Abs();
        Fixed64 nextAlignment = Vector2d.Dot(nextDirection, normal).Abs();
        if (previousAlignment <= nextAlignment)
            featureStart = previous;
        else
            featureEnd = next;
    }

    private static bool TryKeepAxis(
        Vector2d center,
        Vector2d capsuleAxis,
        Fixed64 axisLength,
        Fixed64 radius,
        Vector2d convexOrigin,
        Fixed64 convexRotation,
        ReadOnlySpan<Vector2d> convexOriginOffsets,
        Vector2d axis,
        bool axisFromEdge,
        ref bool found,
        ref Signed576 bestDepth,
        ref Vector2d bestNormal,
        ref bool bestAxisFromEdge)
    {
        GetRelativeProjectionRange(
            center,
            convexOrigin,
            convexRotation,
            convexOriginOffsets,
            axis,
            out Signed192 minimum,
            out Signed192 maximum);
        Signed192 axialAlignment = WideGeometry.GetDifferenceDotProduct2D(
            capsuleAxis.X,
            Fixed64.Zero,
            capsuleAxis.Y,
            Fixed64.Zero,
            axis.X,
            Fixed64.Zero,
            axis.Y,
            Fixed64.Zero);
        if (axialAlignment.Sign < 0)
            axialAlignment = WideArithmetic.SubtractSigned192(default, axialAlignment);

        Signed320 extentNumerator = WideArithmetic.AddSigned320(
            WideArithmetic.MultiplySigned192(
                axialAlignment,
                Signed192.Signed(axisLength.m_rawValue)),
            WideArithmetic.MultiplySigned192(
                Signed192.Signed(radius.m_rawValue),
                DoubleScaleSquared));
        Signed320 minimumProjection = WideArithmetic.AddSigned320(
            Signed320.ExtendValue(minimum),
            Signed320.ExtendValue(minimum));
        Signed320 maximumProjection = WideArithmetic.AddSigned320(
            Signed320.ExtendValue(maximum),
            Signed320.ExtendValue(maximum));
        Signed576 positive = Signed576.ExtendValue(
            WideArithmetic.SubtractSigned320(
                extentNumerator,
                minimumProjection));
        Signed576 negative = Signed576.ExtendValue(
            WideArithmetic.AddSigned320(
                maximumProjection,
                extentNumerator));
        if (positive.Sign < 0 || negative.Sign < 0)
            return false;

        bool usePositive = WideArithmetic.CompareNonNegative(positive, negative) <= 0;
        Signed576 candidateDepth = usePositive ? positive : negative;
        if (!found
            || WideArithmetic.CompareNonNegative(candidateDepth, bestDepth) < 0)
        {
            found = true;
            bestDepth = candidateDepth;
            bestNormal = usePositive ? axis : -axis;
            bestAxisFromEdge = axisFromEdge;
        }

        return true;
    }

    private static void GetRelativeProjectionRange(
        Vector2d center,
        Vector2d convexOrigin,
        Fixed64 convexRotation,
        ReadOnlySpan<Vector2d> convexOriginOffsets,
        Vector2d axis,
        out Signed192 minimum,
        out Signed192 maximum)
    {
        minimum = GetRelativeProjection(
            center,
            convexOrigin,
            convexRotation,
            convexOriginOffsets[0],
            axis);
        maximum = minimum;
        for (int i = 1; i < convexOriginOffsets.Length; i++)
        {
            Signed192 projection = GetRelativeProjection(
                center,
                convexOrigin,
                convexRotation,
                convexOriginOffsets[i],
                axis);
            if (WideArithmetic.SubtractSigned192(projection, minimum).Sign < 0)
                minimum = projection;
            else if (WideArithmetic.SubtractSigned192(projection, maximum).Sign > 0)
                maximum = projection;
        }
    }

    private static Signed192 GetRelativeProjection(
        Vector2d center,
        Vector2d pointOrigin,
        Fixed64 pointRotation,
        Vector2d pointOriginOffset,
        Vector2d axis)
    {
        WideConvex2dRelations.GetRelativePointNumerators(
            pointOrigin,
            pointRotation,
            pointOriginOffset,
            center,
            out Signed192 differenceX,
            out Signed192 differenceY);
        Signed320 projection = WideArithmetic.AddSigned320(
            WideArithmetic.MultiplySigned192(
                differenceX,
                Signed192.Signed(axis.X.m_rawValue)),
            WideArithmetic.MultiplySigned192(
                differenceY,
                Signed192.Signed(axis.Y.m_rawValue)));
        return NarrowProven(projection);
    }

}
