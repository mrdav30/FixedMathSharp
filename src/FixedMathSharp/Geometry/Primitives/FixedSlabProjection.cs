//=======================================================================
// FixedSlabProjection.cs
//=======================================================================
// MIT License, Copyright (c) 2024–present David Oravsky (mrdav30)
// See LICENSE file in the project root for full license information.
//=======================================================================

using System;

namespace FixedMathSharp.Bounds;

/// <summary>
/// Provides planar support points for centered finite shapes clipped to a
/// closed world-Y slab.
/// </summary>
public static class FixedSlabProjection
{
    /// <summary>
    /// Attempts to return the support point of a centered capsule's X/Z
    /// projection after clipping the capsule to <paramref name="slabY"/>.
    /// </summary>
    public static bool TryGetCapsuleSupport(
        Vector3d center,
        Vector3d axisDirection,
        Fixed64 axisLength,
        Fixed64 radius,
        FixedRange slabY,
        Vector2d direction,
        out Vector2d support)
    {
        Validate(axisDirection, axisLength, nameof(axisLength), radius, slabY, direction, requirePositiveLength: false);
        return WideSlabProjection.TryGetCapsuleSupport(
            center, axisDirection, axisLength, radius, slabY, direction, out support);
    }

    /// <summary>
    /// Attempts to return the support point of a centered finite cylinder's
    /// X/Z projection after clipping the cylinder to <paramref name="slabY"/>.
    /// </summary>
    public static bool TryGetCylinderSupport(
        Vector3d center,
        Vector3d axisDirection,
        Fixed64 axisLength,
        Fixed64 radius,
        FixedRange slabY,
        Vector2d direction,
        out Vector2d support)
    {
        Validate(axisDirection, axisLength, nameof(axisLength), radius, slabY, direction, requirePositiveLength: true);
        return WideSlabProjection.TryGetCylinderSupport(
            center, axisDirection, axisLength, radius, slabY, direction, out support);
    }

    /// <summary>
    /// Attempts to return the support point of a centered finite cone's X/Z
    /// projection after clipping the cone to <paramref name="slabY"/>.
    /// </summary>
    /// <param name="center">The cone center.</param>
    /// <param name="axisDirection">The normalized direction from base to apex.</param>
    /// <param name="height">The positive full parametric height.</param>
    /// <param name="radius">The nonnegative base radius.</param>
    /// <param name="slabY">The inclusive world-Y clipping interval.</param>
    /// <param name="direction">The normalized X/Z support direction.</param>
    /// <param name="support">The winning planar support point when representable.</param>
    /// <returns><see langword="true"/> when the clipped cone has a representable support point.</returns>
    public static bool TryGetConeSupport(
        Vector3d center,
        Vector3d axisDirection,
        Fixed64 height,
        Fixed64 radius,
        FixedRange slabY,
        Vector2d direction,
        out Vector2d support)
    {
        Validate(axisDirection, height, nameof(height), radius, slabY, direction, requirePositiveLength: true);
        return WideSlabProjection.TryGetConeSupport(
            center, axisDirection, height, radius, slabY, direction, out support);
    }

    private static void Validate(
        Vector3d axisDirection,
        Fixed64 length,
        string lengthParameterName,
        Fixed64 radius,
        FixedRange slabY,
        Vector2d direction,
        bool requirePositiveLength)
    {
        if (!axisDirection.IsNormalized())
            throw new ArgumentException("Shape axis direction must be normalized.", nameof(axisDirection));
        if (requirePositiveLength ? length <= Fixed64.Zero : length < Fixed64.Zero)
            throw new ArgumentOutOfRangeException(lengthParameterName);
        if (radius < Fixed64.Zero)
            throw new ArgumentOutOfRangeException(nameof(radius));
        if (slabY.Min > slabY.Max)
            throw new ArgumentException("Slab minimum must not exceed its maximum.", nameof(slabY));
        if (!direction.IsNormalized())
            throw new ArgumentException("Planar support direction must be normalized.", nameof(direction));
    }
}
