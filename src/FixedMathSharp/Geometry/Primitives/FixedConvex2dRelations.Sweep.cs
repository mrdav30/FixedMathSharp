//=======================================================================
// FixedConvex2dRelations.Sweep.cs
//=======================================================================
// MIT License, Copyright (c) 2024-present David Oravsky (mrdav30)
// See LICENSE file in the project root for full license information.
//=======================================================================

using System;

namespace FixedMathSharp.Bounds;

/// <content>
/// Contains methods for finding the first distance where a translated or swept finite segment, 
/// conceptual circle, conceptual centered capsule, or translated convex polygon reaches a closed convex polygon
/// </content>
public static partial class FixedConvex2dRelations
{
    /// <summary>
    /// Finds the first distance where a finite directed segment reaches a
    /// closed convex polygon.
    /// </summary>
    public static bool TryGetSegmentFirstIntersectionDistance(
        Vector2d start,
        Vector2d direction,
        Fixed64 maximumDistance,
        Vector2d convexOrigin,
        ReadOnlySpan<Vector2d> convexVertexOffsets,
        out Fixed64 distance,
        out Vector2d normal,
        out Vector2d convexContactOffset)
    {
        ValidateSweep(direction, maximumDistance);
        ValidateVertexOffsets(convexVertexOffsets);
        if (WideConvex2dRelations.ContainsPoint(
                start,
                Vector2d.Zero,
                convexOrigin,
                convexVertexOffsets))
        {
            distance = Fixed64.Zero;
            normal = -direction;
            convexContactOffset =
                WideConvex2dRelations.GetSweptPointTargetContactOffset(
                start,
                Vector2d.Zero,
                Vector2d.Zero,
                convexOrigin,
                convexVertexOffsets,
                normal);
            return true;
        }

        return TryGetSweptCenteredCapsuleFirstDistanceCore(
            start,
            Vector2d.Right,
            Fixed64.Zero,
            Fixed64.Zero,
            direction,
            maximumDistance,
            convexOrigin,
            convexVertexOffsets,
            out distance,
            out normal,
            out convexContactOffset);
    }

    /// <summary>
    /// Finds the first distance where a finite directed segment reaches a
    /// rotated closed convex polygon.
    /// </summary>
    public static bool TryGetSegmentFirstIntersectionDistance(
        Vector2d start,
        Vector2d direction,
        Fixed64 maximumDistance,
        Vector2d convexOrigin,
        Fixed64 convexRotation,
        ReadOnlySpan<Vector2d> convexVertexOffsets,
        out Fixed64 distance,
        out Vector2d normal,
        out FixedPointAnchor2d convexContact)
    {
        ValidateSweep(direction, maximumDistance);
        ValidateVertexOffsets(convexVertexOffsets);
        Vector2d contactOffset;
        if (WideConvex2dRelations.ContainsPoint(
                start,
                Vector2d.Zero,
                convexOrigin,
                convexRotation,
                convexVertexOffsets))
        {
            distance = Fixed64.Zero;
            normal = -direction;
            contactOffset =
                WideConvex2dRelations.GetSweptPointTargetContactOffset(
                    start,
                    Vector2d.Zero,
                    Vector2d.Zero,
                    convexOrigin,
                    convexRotation,
                    convexVertexOffsets,
                    normal);
        }
        else if (!TryGetSweptCenteredCapsuleFirstDistanceCore(
                     start,
                     Vector2d.Right,
                     Fixed64.Zero,
                     Fixed64.Zero,
                     direction,
                     maximumDistance,
                     convexOrigin,
                     convexRotation,
                     convexVertexOffsets,
                     out distance,
                     out normal,
                     out contactOffset))
        {
            convexContact = default;
            return false;
        }

        convexContact = new FixedPointAnchor2d(
            convexOrigin,
            convexRotation,
            contactOffset);
        return true;
    }

    /// <summary>
    /// Finds the first distance where a translated conceptual circle reaches a
    /// closed convex polygon.
    /// </summary>
    public static bool TryGetSweptCircleFirstDistance(
        Vector2d circleCenter,
        Fixed64 circleRadius,
        Vector2d direction,
        Fixed64 maximumDistance,
        Vector2d convexOrigin,
        ReadOnlySpan<Vector2d> convexVertexOffsets,
        out Fixed64 distance,
        out Vector2d normal,
        out Vector2d convexContactOffset)
    {
        if (circleRadius < Fixed64.Zero)
            throw new ArgumentOutOfRangeException(nameof(circleRadius));
        ValidateSweep(direction, maximumDistance);
        ValidateVertexOffsets(convexVertexOffsets);
        if (TryGetCircleContact(
                circleCenter,
                Fixed64.Zero,
                circleRadius,
                convexOrigin,
                Fixed64.Zero,
                convexVertexOffsets,
                out _,
                out FixedPointAnchor2d initialContact,
                out normal,
                out _,
                out _))
        {
            distance = Fixed64.Zero;
            convexContactOffset = initialContact.LocalPoint;
            return true;
        }

        return TryGetSweptCenteredCapsuleFirstDistanceCore(
            circleCenter,
            Vector2d.Right,
            Fixed64.Zero,
            circleRadius,
            direction,
            maximumDistance,
            convexOrigin,
            convexVertexOffsets,
            out distance,
            out normal,
            out convexContactOffset);
    }

    /// <summary>
    /// Finds the first distance where a translated conceptual circle reaches
    /// a rotated closed convex polygon.
    /// </summary>
    public static bool TryGetSweptCircleFirstDistance(
        Vector2d circleCenter,
        Fixed64 circleRadius,
        Vector2d direction,
        Fixed64 maximumDistance,
        Vector2d convexOrigin,
        Fixed64 convexRotation,
        ReadOnlySpan<Vector2d> convexVertexOffsets,
        out Fixed64 distance,
        out Vector2d normal,
        out FixedPointAnchor2d convexContact)
    {
        if (circleRadius < Fixed64.Zero)
            throw new ArgumentOutOfRangeException(nameof(circleRadius));
        ValidateSweep(direction, maximumDistance);
        ValidateVertexOffsets(convexVertexOffsets);
        Vector2d contactOffset;
        if (TryGetCircleContact(
                circleCenter,
                Fixed64.Zero,
                circleRadius,
                convexOrigin,
                convexRotation,
                convexVertexOffsets,
                out _,
                out FixedPointAnchor2d initialContact,
                out normal,
                out _,
                out _))
        {
            distance = Fixed64.Zero;
            convexContact = initialContact;
            return true;
        }
        if (!TryGetSweptCenteredCapsuleFirstDistanceCore(
                circleCenter,
                Vector2d.Right,
                Fixed64.Zero,
                circleRadius,
                direction,
                maximumDistance,
                convexOrigin,
                convexRotation,
                convexVertexOffsets,
                out distance,
                out normal,
                out contactOffset))
        {
            convexContact = default;
            return false;
        }

        convexContact = new FixedPointAnchor2d(
            convexOrigin,
            convexRotation,
            contactOffset);
        return true;
    }

    /// <summary>
    /// Finds the first distance where a translated centered rotated capsule
    /// reaches a rotated closed convex polygon.
    /// </summary>
    public static bool TryGetSweptCenteredCapsuleFirstDistance(
        Vector2d capsuleCenter,
        Fixed64 capsuleRotation,
        Fixed64 capsuleAxisLength,
        Fixed64 capsuleRadius,
        Vector2d direction,
        Fixed64 maximumDistance,
        Vector2d convexOrigin,
        Fixed64 convexRotation,
        ReadOnlySpan<Vector2d> convexVertexOffsets,
        out Fixed64 distance,
        out Vector2d normal,
        out FixedPointAnchor2d convexContact)
    {
        if (capsuleAxisLength < Fixed64.Zero)
            throw new ArgumentOutOfRangeException(nameof(capsuleAxisLength));
        if (capsuleRadius < Fixed64.Zero)
            throw new ArgumentOutOfRangeException(nameof(capsuleRadius));
        ValidateSweep(direction, maximumDistance);
        ValidateVertexOffsets(convexVertexOffsets);

        Span<FixedPointAnchor2d> capsuleContacts =
            stackalloc FixedPointAnchor2d[2];
        Span<FixedPointAnchor2d> convexContacts =
            stackalloc FixedPointAnchor2d[2];
        if (FixedSegment2d.TryGetCenteredCapsuleConvexContacts(
                capsuleCenter,
                capsuleRotation,
                Vector2d.Forward,
                capsuleAxisLength,
                capsuleRadius,
                convexOrigin,
                convexRotation,
                convexVertexOffsets,
                capsuleContacts,
                convexContacts,
                out int contactCount,
                out normal,
                out _,
                out _))
        {
            distance = Fixed64.Zero;
            convexContact = convexContacts[contactCount - 1];
            return true;
        }

        Vector2d capsuleAxis = new(
            -FixedMath.Sin(capsuleRotation),
            FixedMath.Cos(capsuleRotation));
        if (!TryGetSweptCenteredCapsuleFirstDistanceCore(
                capsuleCenter,
                capsuleAxis,
                capsuleAxisLength,
                capsuleRadius,
                direction,
                maximumDistance,
                convexOrigin,
                convexRotation,
                convexVertexOffsets,
                out distance,
                out normal,
                out Vector2d contactOffset))
        {
            convexContact = default;
            return false;
        }

        convexContact = new FixedPointAnchor2d(
            convexOrigin,
            convexRotation,
            contactOffset);
        return true;
    }

    /// <summary>
    /// Finds the first distance where a translated conceptual centered
    /// capsule reaches a closed convex polygon.
    /// </summary>
    public static bool TryGetSweptCenteredCapsuleFirstDistance(
        Vector2d capsuleCenter,
        Vector2d capsuleAxisDirection,
        Fixed64 capsuleAxisLength,
        Fixed64 capsuleRadius,
        Vector2d direction,
        Fixed64 maximumDistance,
        Vector2d convexOrigin,
        ReadOnlySpan<Vector2d> convexVertexOffsets,
        out Fixed64 distance,
        out Vector2d normal,
        out Vector2d convexContactOffset)
    {
        ValidateCenteredAxis(
            capsuleAxisDirection,
            capsuleAxisLength);
        if (capsuleRadius < Fixed64.Zero)
            throw new ArgumentOutOfRangeException(nameof(capsuleRadius));
        ValidateSweep(direction, maximumDistance);
        ValidateVertexOffsets(convexVertexOffsets);
        Span<FixedPointAnchor2d> capsuleContacts =
            stackalloc FixedPointAnchor2d[2];
        Span<FixedPointAnchor2d> convexContacts =
            stackalloc FixedPointAnchor2d[2];
        if (WideCenteredCapsule2dRelations.TryGetContacts(
                capsuleCenter,
                Fixed64.Zero,
                capsuleAxisDirection,
                capsuleAxisLength,
                capsuleRadius,
                convexOrigin,
                Fixed64.Zero,
                convexVertexOffsets,
                capsuleContacts,
                convexContacts,
                out int contactCount,
                out normal,
                out _,
                out _))
        {
            distance = Fixed64.Zero;
            convexContactOffset =
                convexContacts[contactCount - 1].LocalPoint;
            return true;
        }

        return TryGetSweptCenteredCapsuleFirstDistanceCore(
            capsuleCenter,
            capsuleAxisDirection,
            capsuleAxisLength,
            capsuleRadius,
            direction,
            maximumDistance,
            convexOrigin,
            convexVertexOffsets,
            out distance,
            out normal,
            out convexContactOffset);
    }

    /// <summary>
    /// Finds the first distance where a translated conceptual centered capsule
    /// reaches a rotated closed convex polygon.
    /// </summary>
    public static bool TryGetSweptCenteredCapsuleFirstDistance(
        Vector2d capsuleCenter,
        Vector2d capsuleAxisDirection,
        Fixed64 capsuleAxisLength,
        Fixed64 capsuleRadius,
        Vector2d direction,
        Fixed64 maximumDistance,
        Vector2d convexOrigin,
        Fixed64 convexRotation,
        ReadOnlySpan<Vector2d> convexVertexOffsets,
        out Fixed64 distance,
        out Vector2d normal,
        out FixedPointAnchor2d convexContact)
    {
        ValidateCenteredAxis(capsuleAxisDirection, capsuleAxisLength);
        if (capsuleRadius < Fixed64.Zero)
            throw new ArgumentOutOfRangeException(nameof(capsuleRadius));
        ValidateSweep(direction, maximumDistance);
        ValidateVertexOffsets(convexVertexOffsets);
        Span<FixedPointAnchor2d> capsuleContacts =
            stackalloc FixedPointAnchor2d[2];
        Span<FixedPointAnchor2d> convexContacts =
            stackalloc FixedPointAnchor2d[2];
        if (WideCenteredCapsule2dRelations.TryGetContacts(
                capsuleCenter,
                Fixed64.Zero,
                capsuleAxisDirection,
                capsuleAxisLength,
                capsuleRadius,
                convexOrigin,
                convexRotation,
                convexVertexOffsets,
                capsuleContacts,
                convexContacts,
                out int contactCount,
                out normal,
                out _,
                out _))
        {
            distance = Fixed64.Zero;
            convexContact = convexContacts[contactCount - 1];
            return true;
        }

        if (!TryGetSweptCenteredCapsuleFirstDistanceCore(
                capsuleCenter,
                capsuleAxisDirection,
                capsuleAxisLength,
                capsuleRadius,
                direction,
                maximumDistance,
                convexOrigin,
                convexRotation,
                convexVertexOffsets,
                out distance,
                out normal,
                out Vector2d contactOffset))
        {
            convexContact = default;
            return false;
        }

        convexContact = new FixedPointAnchor2d(
            convexOrigin,
            convexRotation,
            contactOffset);
        return true;
    }

    /// <summary>
    /// Finds the first distance where a translated closed convex polygon
    /// reaches another closed convex polygon.
    /// </summary>
    public static bool TryGetSweptConvexFirstDistance(
        Vector2d moverOrigin,
        ReadOnlySpan<Vector2d> moverVertexOffsets,
        Vector2d direction,
        Fixed64 maximumDistance,
        Vector2d targetOrigin,
        ReadOnlySpan<Vector2d> targetVertexOffsets,
        out Fixed64 distance,
        out Vector2d normal,
        out Vector2d targetContactOffset) =>
        TryGetSweptConvexFirstDistanceCore(
            moverOrigin,
            Fixed64.Zero,
            moverVertexOffsets,
            direction,
            maximumDistance,
            targetOrigin,
            Fixed64.Zero,
            targetVertexOffsets,
            out distance,
            out normal,
            out targetContactOffset);

    /// <summary>
    /// Finds the first distance where a translated rotated convex polygon
    /// reaches another rotated closed convex polygon.
    /// </summary>
    public static bool TryGetSweptConvexFirstDistance(
        Vector2d moverOrigin,
        Fixed64 moverRotation,
        ReadOnlySpan<Vector2d> moverVertexOffsets,
        Vector2d direction,
        Fixed64 maximumDistance,
        Vector2d targetOrigin,
        Fixed64 targetRotation,
        ReadOnlySpan<Vector2d> targetVertexOffsets,
        out Fixed64 distance,
        out Vector2d normal,
        out FixedPointAnchor2d targetContact)
    {
        if (!TryGetSweptConvexFirstDistanceCore(
                moverOrigin,
                moverRotation,
                moverVertexOffsets,
                direction,
                maximumDistance,
                targetOrigin,
                targetRotation,
                targetVertexOffsets,
                out distance,
                out normal,
                out Vector2d targetContactOffset))
        {
            targetContact = default;
            return false;
        }

        targetContact = new FixedPointAnchor2d(
            targetOrigin,
            targetRotation,
            targetContactOffset);
        return true;
    }

    private static bool TryGetSweptConvexFirstDistanceCore(
        Vector2d moverOrigin,
        Fixed64 moverRotation,
        ReadOnlySpan<Vector2d> moverVertexOffsets,
        Vector2d direction,
        Fixed64 maximumDistance,
        Vector2d targetOrigin,
        Fixed64 targetRotation,
        ReadOnlySpan<Vector2d> targetVertexOffsets,
        out Fixed64 distance,
        out Vector2d normal,
        out Vector2d targetContactOffset)
    {
        ValidateVertexOffsets(moverVertexOffsets);
        ValidateVertexOffsets(targetVertexOffsets);
        ValidateSweep(direction, maximumDistance);
        Span<Vector2d> moverContacts = stackalloc Vector2d[2];
        Span<Vector2d> targetContacts = stackalloc Vector2d[2];
        if (WideConvex2dRelations.TryGetContactOffsets(
                moverOrigin,
                moverRotation,
                moverVertexOffsets,
                targetOrigin,
                targetRotation,
                targetVertexOffsets,
                moverContacts,
                targetContacts,
                out int contactCount,
                out normal,
                out _,
                out _))
        {
            distance = Fixed64.Zero;
            targetContactOffset = targetContacts[contactCount - 1];
            return true;
        }

        bool found = false;
        distance = default;
        normal = default;
        targetContactOffset = default;
        for (int moverIndex = 0;
            moverIndex < moverVertexOffsets.Length;
            moverIndex++)
        {
            Vector2d moverStart = moverVertexOffsets[moverIndex];
            Vector2d moverEnd = moverVertexOffsets[
                moverIndex + 1 == moverVertexOffsets.Length
                    ? 0
                    : moverIndex + 1];
            for (int targetIndex = 0;
                targetIndex < targetVertexOffsets.Length;
                targetIndex++)
            {
                Vector2d targetStart = targetVertexOffsets[targetIndex];
                Vector2d targetEnd = targetVertexOffsets[
                    targetIndex + 1 == targetVertexOffsets.Length
                        ? 0
                        : targetIndex + 1];
                if (WideFiniteAxisIntersection
                    .TryGetSweptOriginOffsetSegmentsFirstDistance(
                        moverOrigin,
                        moverRotation,
                        moverStart,
                        moverEnd,
                        Fixed64.Zero,
                        direction,
                        maximumDistance,
                        targetOrigin,
                        targetRotation,
                        targetStart,
                        targetEnd,
                        Fixed64.Zero,
                        out Fixed64 candidateDistance,
                        out Vector2d candidateNormal))
                {
                    candidateNormal = OrientSweepNormal(
                        candidateNormal,
                        direction);
                    Vector2d candidateTranslation =
                        direction * candidateDistance;
                    candidateNormal =
                        WideConvex2dRelations.OrientTargetToSourceNormal(
                            candidateNormal,
                            moverOrigin,
                            candidateTranslation,
                            targetOrigin);
                    if (!ShouldReplaceSweepCandidate(
                            candidateDistance,
                            candidateNormal,
                            direction,
                            found,
                            distance,
                            normal))
                    {
                        continue;
                    }
                    found = true;
                    distance = candidateDistance;
                    normal = candidateNormal;
                }
            }
        }

        if (!found)
            return false;

        targetContactOffset =
            WideConvex2dRelations.GetSweptConvexTargetContactOffset(
            moverOrigin,
            moverRotation,
            moverVertexOffsets,
            direction * distance,
            targetOrigin,
            targetRotation,
            targetVertexOffsets,
            normal);
        return true;
    }

    private static bool TryGetSweptCenteredCapsuleFirstDistanceCore(
        Vector2d center,
        Vector2d axisDirection,
        Fixed64 axisLength,
        Fixed64 radius,
        Vector2d direction,
        Fixed64 maximumDistance,
        Vector2d convexOrigin,
        ReadOnlySpan<Vector2d> convexVertexOffsets,
        out Fixed64 distance,
        out Vector2d normal,
        out Vector2d convexContactOffset) =>
        TryGetSweptCenteredCapsuleFirstDistanceCore(
            center,
            axisDirection,
            axisLength,
            radius,
            direction,
            maximumDistance,
            convexOrigin,
            Fixed64.Zero,
            convexVertexOffsets,
            out distance,
            out normal,
            out convexContactOffset);

    private static bool TryGetSweptCenteredCapsuleFirstDistanceCore(
        Vector2d center,
        Vector2d axisDirection,
        Fixed64 axisLength,
        Fixed64 radius,
        Vector2d direction,
        Fixed64 maximumDistance,
        Vector2d convexOrigin,
        Fixed64 convexRotation,
        ReadOnlySpan<Vector2d> convexVertexOffsets,
        out Fixed64 distance,
        out Vector2d normal,
        out Vector2d convexContactOffset)
    {
        bool found = false;
        distance = default;
        normal = default;
        convexContactOffset = default;
        for (int i = 0; i < convexVertexOffsets.Length; i++)
        {
            Vector2d start = convexVertexOffsets[i];
            Vector2d end = convexVertexOffsets[
                i + 1 == convexVertexOffsets.Length ? 0 : i + 1];
            if (WideFiniteAxisIntersection
                .TryGetSweptCenteredCapsuleOriginOffsetSegmentFirstDistance(
                    center,
                    axisDirection,
                    axisLength,
                    radius,
                    direction,
                    maximumDistance,
                    convexOrigin,
                    convexRotation,
                    start,
                    end,
                    Fixed64.Zero,
                    out Fixed64 candidateDistance,
                    out Vector2d candidateNormal))
            {
                candidateNormal = OrientSweepNormal(
                    candidateNormal,
                    direction);
                Vector2d candidateTranslation =
                    direction * candidateDistance;
                candidateNormal =
                    WideConvex2dRelations.OrientTargetToSourceNormal(
                        candidateNormal,
                        center,
                        candidateTranslation,
                        convexOrigin);
                if (!ShouldReplaceSweepCandidate(
                        candidateDistance,
                        candidateNormal,
                        direction,
                        found,
                        distance,
                        normal))
                {
                    continue;
                }
                found = true;
                distance = candidateDistance;
                normal = candidateNormal;
            }
        }

        if (!found)
            return false;

        FixedPointAnchor2d sourceSupport =
            WideFiniteAxisIntersection.GetCenteredCapsuleSupportAnchor(
                center,
                Fixed64.Zero,
                axisDirection,
                axisLength,
                radius,
                -normal);
        convexContactOffset =
            WideConvex2dRelations.GetSweptAnchorTargetContactOffset(
            sourceSupport,
            direction * distance,
            convexOrigin,
            convexRotation,
            convexVertexOffsets,
            normal);
        return true;
    }

    private static bool ShouldReplaceSweepCandidate(
        Fixed64 candidateDistance,
        Vector2d candidateNormal,
        Vector2d direction,
        bool found,
        Fixed64 bestDistance,
        Vector2d bestNormal) =>
        !found
        || candidateDistance < bestDistance
        || (candidateDistance == bestDistance
            && Vector2d.Dot(candidateNormal, direction)
                < Vector2d.Dot(bestNormal, direction));

    private static Vector2d OrientSweepNormal(
        Vector2d normal,
        Vector2d direction) =>
        Vector2d.Dot(normal, direction) > Fixed64.Zero
            ? -normal
            : normal;

    private static void ValidateSweep(
        Vector2d direction,
        Fixed64 maximumDistance)
    {
        if (!direction.IsNormalized())
            throw new ArgumentException(
                "Sweep direction must be normalized.",
                nameof(direction));
        if (maximumDistance < Fixed64.Zero)
        {
            throw new ArgumentOutOfRangeException(
                nameof(maximumDistance));
        }
    }

    private static void ValidateCenteredAxis(
        Vector2d axisDirection,
        Fixed64 axisLength)
    {
        if (!axisDirection.IsNormalized())
        {
            throw new ArgumentException(
                "Axis direction must be normalized.",
                nameof(axisDirection));
        }
        if (axisLength < Fixed64.Zero)
            throw new ArgumentOutOfRangeException(nameof(axisLength));
    }
}
