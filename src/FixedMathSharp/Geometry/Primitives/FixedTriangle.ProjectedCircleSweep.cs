//=======================================================================
// FixedTriangle.ProjectedCircleSweep.cs
//=======================================================================
// MIT License, Copyright (c) 2024-present David Oravsky (mrdav30)
// See LICENSE file in the project root for full license information.
//=======================================================================

using System;

namespace FixedMathSharp.Geometry;

/// <content>
/// Provides deterministic queries for projecting and sweeping a world X/Z
/// circle against a triangle clipped to a finite world-Y slab. Handles both
/// the static overlap case and the swept time-of-impact case, delegating the
/// core triangle transformation, slab clipping, and contact classification
/// to <see cref="WideOrientedBox"/>.
/// </content>
public partial struct FixedTriangle
{
    /// <summary>
    /// Attempts to find a triangle witness where a world-X/Z circle overlaps
    /// the triangle portion inside one finite world-Y slab.
    /// </summary>
    public readonly bool TryGetFiniteSlabProjectedCircleContact(
        Vector3d triangleOrigin,
        FixedQuaternion triangleRotation,
        Vector2d circleCenter,
        Fixed64 circleRadius,
        Fixed64 slabCenterY,
        Fixed64 slabHalfThickness,
        out FixedPointAnchor triangleContact)
    {
        ValidateRigidTriangleFrame(triangleRotation);
        if (circleRadius < Fixed64.Zero)
            throw new ArgumentOutOfRangeException(nameof(circleRadius));
        if (slabHalfThickness < Fixed64.Zero)
            throw new ArgumentOutOfRangeException(nameof(slabHalfThickness));

        return WideOrientedBox.TryGetFiniteSlabProjectedCircleSweep(
            this,
            triangleOrigin,
            triangleRotation,
            circleCenter,
            Vector2d.Right,
            Fixed64.Zero,
            circleRadius,
            slabCenterY,
            slabHalfThickness,
            out _,
            out triangleContact);
    }

    /// <summary>
    /// Finds the first distance where a circle swept in world X/Z reaches the
    /// triangle portion inside one finite world-Y slab.
    /// </summary>
    /// <remarks>
    /// Triangle transformation, slab clipping, and time-of-impact
    /// classification remain exact until the final Q32.32 distance and local
    /// triangle witness are rounded.
    /// </remarks>
    public readonly bool TryGetFiniteSlabProjectedCircleSweep(
        Vector3d triangleOrigin,
        FixedQuaternion triangleRotation,
        Vector2d circleStart,
        Vector2d normalizedDirection,
        Fixed64 maximumDistance,
        Fixed64 circleRadius,
        Fixed64 slabCenterY,
        Fixed64 slabHalfThickness,
        out Fixed64 distance,
        out FixedPointAnchor triangleContact)
    {
        ValidateRigidTriangleFrame(triangleRotation);
        if (!normalizedDirection.IsNormalized())
        {
            throw new ArgumentException(
                "Sweep direction must be normalized.",
                nameof(normalizedDirection));
        }
        if (maximumDistance < Fixed64.Zero)
            throw new ArgumentOutOfRangeException(nameof(maximumDistance));
        if (circleRadius < Fixed64.Zero)
            throw new ArgumentOutOfRangeException(nameof(circleRadius));
        if (slabHalfThickness < Fixed64.Zero)
            throw new ArgumentOutOfRangeException(nameof(slabHalfThickness));

        return WideOrientedBox.TryGetFiniteSlabProjectedCircleSweep(
            this,
            triangleOrigin,
            triangleRotation,
            circleStart,
            normalizedDirection,
            maximumDistance,
            circleRadius,
            slabCenterY,
            slabHalfThickness,
            out distance,
            out triangleContact);
    }
}
