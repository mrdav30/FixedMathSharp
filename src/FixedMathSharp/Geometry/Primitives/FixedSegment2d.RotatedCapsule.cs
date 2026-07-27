//=======================================================================
// FixedSegment2d.RotatedCapsule.cs
//=======================================================================
// MIT License, Copyright (c) 2024–present David Oravsky (mrdav30)
// See LICENSE file in the project root for full license information.
//=======================================================================

using System;

namespace FixedMathSharp.Geometry;

/// <content>
/// Provides methods for testing point containment against a rotated capsule
/// shape, defined by a centered axis-aligned segment with an applied frame
/// rotation and a radius, using deterministic fixed-point arithmetic.
/// </content>
public partial struct FixedSegment2d
{
    /// <summary>
    /// Returns whether a point lies inside or on a centered rotated capsule.
    /// </summary>
    public static bool ContainsPointInCenteredCapsule(
        Vector2d point,
        Vector2d center,
        Fixed64 frameRotation,
        Fixed64 axisLength,
        Fixed64 radius) =>
        ContainsPointInCenteredCapsule(
            point,
            center,
            frameRotation,
            axisLength,
            radius,
            Fixed64.Zero,
            strict: false);

    /// <summary>
    /// Returns whether a point lies inside or on a radially expanded centered
    /// rotated capsule.
    /// </summary>
    public static bool ContainsPointInCenteredCapsule(
        Vector2d point,
        Vector2d center,
        Fixed64 frameRotation,
        Fixed64 axisLength,
        Fixed64 radius,
        Fixed64 radiusExpansion) =>
        ContainsPointInCenteredCapsule(
            point,
            center,
            frameRotation,
            axisLength,
            radius,
            radiusExpansion,
            strict: false);

    /// <summary>
    /// Returns whether a point lies within a centered capsule whose local
    /// positive Y axis is transformed by a scalar rigid-frame rotation.
    /// </summary>
    public static bool ContainsPointInCenteredCapsule(
        Vector2d point,
        Vector2d center,
        Fixed64 frameRotation,
        Fixed64 axisLength,
        Fixed64 radius,
        Fixed64 radiusExpansion,
        bool strict)
    {
        ValidateRotatedCapsule(axisLength, radius);
        if (radiusExpansion < Fixed64.Zero)
            throw new ArgumentOutOfRangeException(nameof(radiusExpansion));

        return WideFiniteAxisIntersection.ContainsPointInCenteredCapsule(
            point,
            center,
            GetRotatedPositiveYAxis(frameRotation),
            axisLength,
            radius,
            radiusExpansion,
            strict);
    }

    /// <summary>
    /// Returns the normalized direction from the closest point on a centered
    /// capsule axis expressed by a scalar rigid-frame rotation.
    /// </summary>
    public static Vector2d GetDirectionFromCenteredAxis(
        Vector2d point,
        Vector2d center,
        Fixed64 frameRotation,
        Fixed64 axisLength)
    {
        ValidateCenteredAxis(Vector2d.Forward, axisLength);
        return WideFiniteAxisIntersection.GetDirectionFromCenteredAxis(
            point,
            center,
            GetRotatedPositiveYAxis(frameRotation),
            axisLength);
    }

    /// <summary>
    /// Returns the rounded distance from a point to a centered rotated capsule.
    /// </summary>
    public static Fixed64 GetDistanceToCenteredCapsule(
        Vector2d point,
        Vector2d center,
        Fixed64 frameRotation,
        Fixed64 axisLength,
        Fixed64 radius)
    {
        _ = TryGetDistanceToCenteredCapsule(
            point,
            center,
            frameRotation,
            axisLength,
            radius,
            out Fixed64 distance);
        return distance;
    }

    /// <summary>
    /// Attempts to return the rounded distance from a point to a centered
    /// rotated capsule.
    /// </summary>
    public static bool TryGetDistanceToCenteredCapsule(
        Vector2d point,
        Vector2d center,
        Fixed64 frameRotation,
        Fixed64 axisLength,
        Fixed64 radius,
        out Fixed64 distance)
    {
        ValidateRotatedCapsule(axisLength, radius);
        return WideFiniteAxisIntersection.TryGetDistanceToCenteredCapsule(
            point,
            center,
            GetRotatedPositiveYAxis(frameRotation),
            axisLength,
            radius,
            out distance);
    }

    /// <summary>
    /// Attempts to materialize support for a centered rotated capsule.
    /// </summary>
    public static bool TryGetCenteredCapsuleSupport(
        Vector2d center,
        Fixed64 frameRotation,
        Fixed64 axisLength,
        Fixed64 radius,
        Vector2d direction,
        out Vector2d support)
    {
        ValidateRotatedCapsule(axisLength, radius);
        return WideFiniteAxisIntersection.TryGetCenteredCapsuleSupport(
            center,
            GetRotatedPositiveYAxis(frameRotation),
            axisLength,
            radius,
            direction,
            out support);
    }

    /// <summary>
    /// Attempts to materialize the selected surface point of a centered
    /// rotated capsule.
    /// </summary>
    public static bool TryGetSurfacePointOnCenteredCapsule(
        Vector2d point,
        Vector2d center,
        Fixed64 frameRotation,
        Fixed64 axisLength,
        Fixed64 radius,
        Vector2d surfaceDirection,
        out Vector2d surfacePoint)
    {
        ValidateRotatedCapsule(axisLength, radius);
        if (!surfaceDirection.IsNormalized())
        {
            throw new ArgumentException(
                "Surface direction must be normalized.",
                nameof(surfaceDirection));
        }

        return WideFiniteAxisIntersection.TryGetSurfacePointOnCenteredCapsule(
            point,
            center,
            GetRotatedPositiveYAxis(frameRotation),
            axisLength,
            radius,
            surfaceDirection,
            out surfacePoint);
    }

    /// <summary>
    /// Attempts to materialize the center-relative selected surface offset of
    /// a centered rotated capsule.
    /// </summary>
    public static bool TryGetSurfaceOffsetOnCenteredCapsule(
        Vector2d point,
        Vector2d center,
        Fixed64 frameRotation,
        Fixed64 axisLength,
        Fixed64 radius,
        Vector2d surfaceDirection,
        out Vector2d surfaceOffset)
    {
        ValidateRotatedCapsule(axisLength, radius);
        if (!surfaceDirection.IsNormalized())
        {
            throw new ArgumentException(
                "Surface direction must be normalized.",
                nameof(surfaceDirection));
        }

        return WideFiniteAxisIntersection.TryGetSurfaceOffsetOnCenteredCapsule(
            point,
            center,
            GetRotatedPositiveYAxis(frameRotation),
            axisLength,
            radius,
            surfaceDirection,
            out surfaceOffset);
    }

    /// <summary>
    /// Returns whether two centered capsules expressed by scalar rigid-frame
    /// rotations overlap, including exact tangency.
    /// </summary>
    public static bool DoCenteredCapsulesOverlap(
        Vector2d firstCenter,
        Fixed64 firstRotation,
        Fixed64 firstAxisLength,
        Fixed64 firstRadius,
        Vector2d secondCenter,
        Fixed64 secondRotation,
        Fixed64 secondAxisLength,
        Fixed64 secondRadius)
    {
        ValidateRotatedCapsule(firstAxisLength, firstRadius);
        ValidateRotatedCapsule(secondAxisLength, secondRadius);
        return WideFiniteAxisIntersection.DoCenteredCapsulesOverlap(
            firstCenter,
            GetRotatedPositiveYAxis(firstRotation),
            firstAxisLength,
            firstRadius,
            secondCenter,
            GetRotatedPositiveYAxis(secondRotation),
            secondAxisLength,
            secondRadius);
    }

    /// <summary>
    /// Attempts to build exact anchors between two centered capsules expressed
    /// by scalar rigid-frame rotations.
    /// </summary>
    public static bool TryGetCenteredCapsulesContact(
        Vector2d firstCenter,
        Fixed64 firstRotation,
        Fixed64 firstAxisLength,
        Fixed64 firstRadius,
        Vector2d secondCenter,
        Fixed64 secondRotation,
        Fixed64 secondAxisLength,
        Fixed64 secondRadius,
        Vector2d fallbackNormal,
        out FixedContactAnchors2d contact)
    {
        ValidateRotatedCapsule(firstAxisLength, firstRadius);
        ValidateRotatedCapsule(secondAxisLength, secondRadius);
        if (!fallbackNormal.IsNormalized())
        {
            throw new ArgumentException(
                "Fallback normal must be normalized.",
                nameof(fallbackNormal));
        }

        return WideFiniteAxisIntersection.TryGetCenteredCapsulesContact(
            firstCenter,
            firstRotation,
            GetRotatedPositiveYAxis(firstRotation),
            firstAxisLength,
            firstRadius,
            secondCenter,
            secondRotation,
            GetRotatedPositiveYAxis(secondRotation),
            secondAxisLength,
            secondRadius,
            fallbackNormal,
            out contact);
    }

    /// <summary>
    /// Finds the minimum translation that separates a centered rotated capsule
    /// from a closed convex polygon represented by origin-relative offsets.
    /// </summary>
    public static bool TryGetCenteredCapsuleConvexMinimumTranslation(
        Vector2d center,
        Fixed64 rotation,
        Fixed64 axisLength,
        Fixed64 radius,
        Vector2d convexOrigin,
        ReadOnlySpan<Vector2d> convexVertexOffsets,
        out Vector2d normal,
        out Fixed64 depth)
    {
        ValidateRotatedCapsule(axisLength, radius);
        if (convexVertexOffsets.Length < 3)
        {
            throw new ArgumentException(
                "A convex polygon requires at least three vertex offsets.",
                nameof(convexVertexOffsets));
        }

        return WideCenteredCapsule2dRelations.TryGetMinimumTranslation(
            center,
            GetRotatedPositiveYAxis(rotation),
            axisLength,
            radius,
            convexOrigin,
            convexVertexOffsets,
            out normal,
            out depth);
    }

    /// <summary>
    /// Finds the first distance where a translated centered rotated capsule
    /// intersects an endpoint-authored capsule axis.
    /// </summary>
    public static bool TryGetSweptCenteredCapsuleSegmentFirstDistance(
        Vector2d center,
        Fixed64 rotation,
        Fixed64 axisLength,
        Fixed64 radius,
        Vector2d direction,
        Fixed64 maximumDistance,
        FixedSegment2d targetAxis,
        Fixed64 targetRadius,
        out Fixed64 distance,
        out Vector2d normal)
    {
        ValidateSweep(
            Vector2d.Forward,
            axisLength,
            radius,
            direction,
            maximumDistance,
            targetRadius);
        return WideFiniteAxisIntersection.TryGetSweptCenteredCapsuleSegmentFirstDistance(
            center,
            GetRotatedPositiveYAxis(rotation),
            axisLength,
            radius,
            direction,
            maximumDistance,
            targetAxis,
            targetRadius,
            out distance,
            out normal);
    }

    /// <summary>
    /// Finds the first distance where a translated centered rotated capsule
    /// intersects another centered rotated capsule.
    /// </summary>
    public static bool TryGetSweptCenteredCapsulesFirstDistance(
        Vector2d center,
        Fixed64 rotation,
        Fixed64 axisLength,
        Fixed64 radius,
        Vector2d direction,
        Fixed64 maximumDistance,
        Vector2d targetCenter,
        Fixed64 targetRotation,
        Fixed64 targetAxisLength,
        Fixed64 targetRadius,
        out Fixed64 distance,
        out Vector2d normal)
    {
        ValidateSweep(
            Vector2d.Forward,
            axisLength,
            radius,
            direction,
            maximumDistance,
            targetRadius);
        ValidateCenteredAxis(Vector2d.Forward, targetAxisLength);
        return WideFiniteAxisIntersection.TryGetSweptCenteredCapsulesFirstDistance(
            center,
            GetRotatedPositiveYAxis(rotation),
            axisLength,
            radius,
            direction,
            maximumDistance,
            targetCenter,
            GetRotatedPositiveYAxis(targetRotation),
            targetAxisLength,
            targetRadius,
            out distance,
            out normal);
    }

    /// <summary>
    /// Finds the physical-distance interval where this segment intersects a
    /// centered rotated capsule.
    /// </summary>
    public readonly bool TryGetCapsuleIntersectionDistanceInterval(
        Vector2d center,
        Fixed64 rotation,
        Fixed64 axisLength,
        Fixed64 radius,
        Fixed64 radiusExpansion,
        Fixed64 totalDistance,
        out Fixed64 entryDistance,
        out Fixed64 exitDistance,
        out bool startContained,
        out bool endContainedStrict)
    {
        ValidateTotalDistance(totalDistance);
        ValidateRotatedCapsule(axisLength, radius);
        if (radiusExpansion < Fixed64.Zero)
            throw new ArgumentOutOfRangeException(nameof(radiusExpansion));

        return WideFiniteAxisIntersection.TryGetCapsuleDistanceInterval(
            this,
            center,
            GetRotatedPositiveYAxis(rotation),
            axisLength,
            radius,
            radiusExpansion,
            totalDistance,
            out entryDistance,
            out exitDistance,
            out startContained,
            out endContainedStrict);
    }

    private static Vector2d GetRotatedPositiveYAxis(Fixed64 rotation) =>
        new(-FixedMath.Sin(rotation), FixedMath.Cos(rotation));

    private static void ValidateRotatedCapsule(
        Fixed64 axisLength,
        Fixed64 radius)
    {
        ValidateCenteredAxis(Vector2d.Forward, axisLength);
        if (radius < Fixed64.Zero)
            throw new ArgumentOutOfRangeException(nameof(radius));
    }
}
