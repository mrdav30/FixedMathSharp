//=======================================================================
// FixedSegment.CenteredSurfaceAnchors.cs
//=======================================================================
// MIT License, Copyright (c) 2024–present David Oravsky (mrdav30)
// See LICENSE file in the project root for full license information.
//=======================================================================

using System;

namespace FixedMathSharp.Geometry;

/// <content>
/// Contains methods for computing the nearest surface feature of a 
/// centered 3D capsule, finite cylinder, or finite cone in its authoritative rigid frame.
/// </content>
public partial struct FixedSegment
{
    /// <summary>
    /// Attempts to return the nearest centered-capsule surface feature in its
    /// authoritative rigid frame.
    /// </summary>
    /// <remarks>
    /// The returned anchor retains odd full-length cap positions and its
    /// radial feature until final materialization. Distance is positive
    /// outside, zero on the selected surface lattice witness, and negative
    /// inside.
    /// </remarks>
    public static bool TryGetClosestCenteredCapsuleSurfaceAnchor(
        Vector3d point,
        Vector3d center,
        FixedQuaternion frameRotation,
        Vector3d localAxisDirection,
        Fixed64 axisLength,
        Fixed64 radius,
        Vector3d localFallbackRadialDirection,
        out FixedPointAnchor surfaceAnchor,
        out Vector3d outwardNormal,
        out Fixed64 signedDistance)
    {
        if (!frameRotation.IsNormalized())
        {
            throw new ArgumentException(
                "Frame rotation must be normalized.",
                nameof(frameRotation));
        }
        ValidateCenteredAxis(localAxisDirection, axisLength);
        ValidateCenteredCapsuleSurface(
            radius,
            localFallbackRadialDirection);
        if (Vector3d.Cross(
                localAxisDirection,
                localFallbackRadialDirection).IsZero)
        {
            throw new ArgumentException(
                "Fallback radial direction must not be parallel to the axis.",
                nameof(localFallbackRadialDirection));
        }

        return WideFiniteAxisIntersection
            .TryGetClosestCenteredCapsuleSurfaceAnchor(
                point,
                center,
                frameRotation,
                localAxisDirection,
                axisLength,
                radius,
                localFallbackRadialDirection,
                out surfaceAnchor,
                out outwardNormal,
                out signedDistance);
    }

    /// <summary>
    /// Attempts to return the nearest centered finite-cylinder surface feature
    /// in its authoritative rigid frame.
    /// </summary>
    /// <remarks>
    /// The returned anchor retains axial and radial feature terms separately,
    /// so callers do not need to inverse-rotate an already rounded world-space
    /// offset. Distance is positive outside, zero on the selected surface
    /// witness, and negative inside.
    /// </remarks>
    public static bool TryGetClosestCenteredFiniteCylinderSurfaceAnchor(
        Vector3d point,
        Vector3d center,
        FixedQuaternion frameRotation,
        Vector3d localAxisDirection,
        Fixed64 axisLength,
        Fixed64 radius,
        Vector3d localFallbackRadialDirection,
        out FixedPointAnchor surfaceAnchor,
        out Vector3d outwardNormal,
        out Fixed64 signedDistance)
    {
        ValidateCenteredSurfaceFrame(
            frameRotation,
            localAxisDirection,
            axisLength,
            radius,
            localFallbackRadialDirection);
        return WideFiniteAxisIntersection
            .TryGetClosestCenteredFiniteCylinderSurfaceAnchor(
                point,
                center,
                frameRotation,
                localAxisDirection,
                axisLength,
                radius,
                localFallbackRadialDirection,
                out surfaceAnchor,
                out outwardNormal,
                out signedDistance);
    }

    /// <summary>
    /// Attempts to return the nearest centered finite-cone surface feature in
    /// its authoritative rigid frame.
    /// </summary>
    /// <remarks>
    /// The local axis points from the base toward the apex. The returned anchor
    /// retains the selected meridian feature without a rounded world-offset
    /// round trip. Distance is positive outside, zero on the selected surface
    /// witness, and negative inside.
    /// </remarks>
    public static bool TryGetClosestCenteredFiniteConeSurfaceAnchor(
        Vector3d point,
        Vector3d center,
        FixedQuaternion frameRotation,
        Vector3d localAxisDirection,
        Fixed64 height,
        Fixed64 radius,
        Vector3d localFallbackRadialDirection,
        out FixedPointAnchor surfaceAnchor,
        out Vector3d outwardNormal,
        out Fixed64 signedDistance)
    {
        ValidateCenteredSurfaceFrame(
            frameRotation,
            localAxisDirection,
            height,
            radius,
            localFallbackRadialDirection);
        return WideFiniteAxisIntersection
            .TryGetClosestCenteredFiniteConeSurfaceAnchor(
                point,
                center,
                frameRotation,
                localAxisDirection,
                height,
                radius,
                localFallbackRadialDirection,
                out surfaceAnchor,
                out outwardNormal,
                out signedDistance);
    }

    private static void ValidateCenteredSurfaceFrame(
        FixedQuaternion frameRotation,
        Vector3d localAxisDirection,
        Fixed64 axisLength,
        Fixed64 radius,
        Vector3d localFallbackRadialDirection)
    {
        if (!frameRotation.IsNormalized())
        {
            throw new ArgumentException(
                "Frame rotation must be normalized.",
                nameof(frameRotation));
        }
        ValidateCenteredSurface(
            localAxisDirection,
            axisLength,
            radius,
            localFallbackRadialDirection,
            nameof(localAxisDirection),
            nameof(axisLength),
            nameof(radius),
            nameof(localFallbackRadialDirection));
    }
}
