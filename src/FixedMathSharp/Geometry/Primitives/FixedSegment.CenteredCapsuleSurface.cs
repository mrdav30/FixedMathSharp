//=======================================================================
// FixedSegment.CenteredCapsuleSurface.cs
//=======================================================================
// MIT License, Copyright (c) 2024–present David Oravsky (mrdav30)
// See LICENSE file in the project root for full license information.
//=======================================================================

using System;

namespace FixedMathSharp.Bounds;

/// <content>
/// Contains methods for computing the nearest surface feature of a centered without narrowing its conceptual axis.
/// </content>
public partial struct FixedSegment
{
    /// <summary>
    /// Attempts to return the selected point on a conceptual centered capsule
    /// surface without narrowing its axis point before applying the radial
    /// offset.
    /// </summary>
    public static bool TryGetSurfacePointOnCenteredCapsule(
        Vector3d point,
        Vector3d center,
        Vector3d axisDirection,
        Fixed64 axisLength,
        Fixed64 radius,
        Vector3d surfaceDirection,
        out Vector3d surfacePoint)
    {
        ValidateCenteredAxis(axisDirection, axisLength);
        ValidateCenteredCapsuleSurface(radius, surfaceDirection);
        return WideFiniteAxisIntersection.TryGetSurfacePointOnCenteredCapsule(
            point,
            center,
            axisDirection,
            axisLength,
            radius,
            surfaceDirection,
            out surfacePoint);
    }

    /// <summary>
    /// Attempts to return a centered-capsule surface offset relative to the
    /// capsule center.
    /// </summary>
    public static bool TryGetSurfaceOffsetOnCenteredCapsule(
        Vector3d point,
        Vector3d center,
        Vector3d axisDirection,
        Fixed64 axisLength,
        Fixed64 radius,
        Vector3d surfaceDirection,
        out Vector3d surfaceOffset)
    {
        ValidateCenteredAxis(axisDirection, axisLength);
        ValidateCenteredCapsuleSurface(radius, surfaceDirection);
        return WideFiniteAxisIntersection.TryGetSurfaceOffsetOnCenteredCapsule(
            point,
            center,
            axisDirection,
            axisLength,
            radius,
            surfaceDirection,
            out surfaceOffset);
    }

    /// <summary>
    /// Returns the centered-capsule surface anchor nearest to a world-space
    /// point in the supplied normalized world-space surface direction.
    /// </summary>
    /// <remarks>
    /// The closest axial offset and radial offset remain separate in an
    /// identity-rotation frame, avoiding an inverse-/forward-rotation
    /// round-trip when the caller already owns the world-space direction.
    /// </remarks>
    public static FixedPointAnchor GetSurfaceAnchorOnCenteredCapsule(
        Vector3d point,
        Vector3d center,
        Vector3d axisDirection,
        Fixed64 axisLength,
        Fixed64 radius,
        Vector3d surfaceDirection)
    {
        ValidateCenteredAxis(axisDirection, axisLength);
        ValidateCenteredCapsuleSurface(radius, surfaceDirection);
        return WideFiniteAxisIntersection.GetSurfaceAnchorOnCenteredCapsule(
            point,
            center,
            axisDirection,
            axisLength,
            radius,
            surfaceDirection);
    }

    /// <summary>
    /// Returns the centered-capsule surface anchor nearest to a world-space
    /// point in the supplied normalized local-space surface direction.
    /// </summary>
    public static FixedPointAnchor GetSurfaceAnchorOnCenteredCapsule(
        Vector3d point,
        Vector3d center,
        FixedQuaternion frameRotation,
        Vector3d localAxisDirection,
        Fixed64 axisLength,
        Fixed64 radius,
        Vector3d localSurfaceDirection)
    {
        if (!frameRotation.IsNormalized())
        {
            throw new ArgumentException(
                "Frame rotation must be normalized.",
                nameof(frameRotation));
        }
        ValidateCenteredAxis(localAxisDirection, axisLength);
        ValidateCenteredCapsuleSurface(radius, localSurfaceDirection);
        return WideFiniteAxisIntersection.GetSurfaceAnchorOnCenteredCapsule(
            point,
            center,
            frameRotation,
            localAxisDirection,
            axisLength,
            radius,
            localSurfaceDirection);
    }

    private static void ValidateCenteredCapsuleSurface(
        Fixed64 radius,
        Vector3d surfaceDirection)
    {
        if (radius < Fixed64.Zero)
            throw new ArgumentOutOfRangeException(nameof(radius));
        if (!surfaceDirection.IsNormalized())
        {
            throw new ArgumentException(
                "Surface direction must be normalized.",
                nameof(surfaceDirection));
        }
    }
}
