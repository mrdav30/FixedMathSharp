//=======================================================================
// FixedSegment.CapsuleSlabProjection.cs
//=======================================================================
// MIT License, Copyright (c) 2024-present David Oravsky (mrdav30)
// See LICENSE file in the project root for full license information.
//=======================================================================

using System;

namespace FixedMathSharp.Geometry;

/// <content>
/// Contains methods for computing the oriented penetration of a centered 3D capsule, finite cylinder, 
/// or finite cone and a centered planar capsule extruded through a world-Y slab.
/// </content>
public partial struct FixedSegment
{
    /// <summary>
    /// Attempts to return the oriented penetration of a centered 3D capsule
    /// and a centered planar capsule extruded through a world-Y slab.
    /// </summary>
    public static bool TryGetCenteredCapsuleCapsuleSlabAxisPenetration(
        Vector3d projectionAxis,
        Vector3d capsuleCenter,
        Vector3d capsuleAxisDirection,
        Fixed64 capsuleAxisLength,
        Fixed64 capsuleRadius,
        Vector3d slabCenter,
        Vector2d slabCapsuleAxisDirection,
        Fixed64 slabCapsuleAxisLength,
        Fixed64 slabCapsuleRadius,
        Fixed64 slabHalfThickness,
        out Vector3d orientedAxis,
        out Fixed64 depth,
        out bool depthIsClamped)
    {
        ValidateCapsuleSlabProjection(
            projectionAxis,
            capsuleAxisDirection,
            capsuleAxisLength,
            capsuleRadius,
            requirePositiveShapeLength: false,
            slabCapsuleAxisDirection,
            slabCapsuleAxisLength,
            slabCapsuleRadius,
            slabHalfThickness);
        return WideFiniteAxisProjection
            .TryGetCapsuleCapsuleSlabAxisPenetration(
                projectionAxis,
                capsuleCenter,
                capsuleAxisDirection,
                capsuleAxisLength,
                capsuleRadius,
                slabCenter,
                slabCapsuleAxisDirection,
                slabCapsuleAxisLength,
                slabCapsuleRadius,
                slabHalfThickness,
                out orientedAxis,
                out depth,
                out depthIsClamped);
    }

    /// <summary>
    /// Attempts to return the oriented penetration of a centered finite
    /// cylinder and a centered planar capsule extruded through a world-Y slab.
    /// </summary>
    public static bool TryGetCenteredFiniteCylinderCapsuleSlabAxisPenetration(
        Vector3d projectionAxis,
        Vector3d cylinderCenter,
        Vector3d cylinderAxisDirection,
        Fixed64 cylinderAxisLength,
        Fixed64 cylinderRadius,
        Vector3d slabCenter,
        Vector2d slabCapsuleAxisDirection,
        Fixed64 slabCapsuleAxisLength,
        Fixed64 slabCapsuleRadius,
        Fixed64 slabHalfThickness,
        out Vector3d orientedAxis,
        out Fixed64 depth,
        out bool depthIsClamped)
    {
        ValidateCapsuleSlabProjection(
            projectionAxis,
            cylinderAxisDirection,
            cylinderAxisLength,
            cylinderRadius,
            requirePositiveShapeLength: true,
            slabCapsuleAxisDirection,
            slabCapsuleAxisLength,
            slabCapsuleRadius,
            slabHalfThickness);
        return WideFiniteAxisProjection
            .TryGetCylinderCapsuleSlabAxisPenetration(
                projectionAxis,
                cylinderCenter,
                cylinderAxisDirection,
                cylinderAxisLength,
                cylinderRadius,
                slabCenter,
                slabCapsuleAxisDirection,
                slabCapsuleAxisLength,
                slabCapsuleRadius,
                slabHalfThickness,
                out orientedAxis,
                out depth,
                out depthIsClamped);
    }

    /// <summary>
    /// Attempts to return the oriented penetration of a centered finite cone
    /// and a centered planar capsule extruded through a world-Y slab.
    /// </summary>
    public static bool TryGetCenteredFiniteConeCapsuleSlabAxisPenetration(
        Vector3d projectionAxis,
        Vector3d coneCenter,
        Vector3d baseToApexDirection,
        Fixed64 coneHeight,
        Fixed64 coneRadius,
        Vector3d slabCenter,
        Vector2d slabCapsuleAxisDirection,
        Fixed64 slabCapsuleAxisLength,
        Fixed64 slabCapsuleRadius,
        Fixed64 slabHalfThickness,
        out Vector3d orientedAxis,
        out Fixed64 depth,
        out bool depthIsClamped)
    {
        ValidateCapsuleSlabProjection(
            projectionAxis,
            baseToApexDirection,
            coneHeight,
            coneRadius,
            requirePositiveShapeLength: true,
            slabCapsuleAxisDirection,
            slabCapsuleAxisLength,
            slabCapsuleRadius,
            slabHalfThickness);
        return WideFiniteAxisProjection
            .TryGetConeCapsuleSlabAxisPenetration(
                projectionAxis,
                coneCenter,
                baseToApexDirection,
                coneHeight,
                coneRadius,
                slabCenter,
                slabCapsuleAxisDirection,
                slabCapsuleAxisLength,
                slabCapsuleRadius,
                slabHalfThickness,
                out orientedAxis,
                out depth,
                out depthIsClamped);
    }

    private static void ValidateCapsuleSlabProjection(
        Vector3d projectionAxis,
        Vector3d shapeAxisDirection,
        Fixed64 shapeAxisLength,
        Fixed64 shapeRadius,
        bool requirePositiveShapeLength,
        Vector2d slabCapsuleAxisDirection,
        Fixed64 slabCapsuleAxisLength,
        Fixed64 slabCapsuleRadius,
        Fixed64 slabHalfThickness)
    {
        if (!projectionAxis.IsNormalized())
            throw new ArgumentException("Projection axis must be normalized.", nameof(projectionAxis));
        if (!shapeAxisDirection.IsNormalized())
            throw new ArgumentException("Shape axis direction must be normalized.", nameof(shapeAxisDirection));
        if (requirePositiveShapeLength
            ? shapeAxisLength <= Fixed64.Zero
            : shapeAxisLength < Fixed64.Zero)
        {
            throw new ArgumentOutOfRangeException(nameof(shapeAxisLength));
        }
        if (shapeRadius < Fixed64.Zero)
            throw new ArgumentOutOfRangeException(nameof(shapeRadius));
        if (!slabCapsuleAxisDirection.IsNormalized())
        {
            throw new ArgumentException(
                "Slab capsule axis direction must be normalized.",
                nameof(slabCapsuleAxisDirection));
        }
        if (slabCapsuleAxisLength < Fixed64.Zero)
            throw new ArgumentOutOfRangeException(nameof(slabCapsuleAxisLength));
        if (slabCapsuleRadius < Fixed64.Zero)
            throw new ArgumentOutOfRangeException(nameof(slabCapsuleRadius));
        if (slabHalfThickness <= Fixed64.Zero)
            throw new ArgumentOutOfRangeException(nameof(slabHalfThickness));
    }
}
