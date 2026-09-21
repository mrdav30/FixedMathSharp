//=======================================================================
// FixedConvex2dRelations.PlanarSweep.cs
//=======================================================================
// MIT License, Copyright (c) 2024-present David Oravsky (mrdav30)
// See LICENSE file in the project root for full license information.
//=======================================================================

using System;

namespace FixedMathSharp.Geometry;

/// <content>Strict continuous planar capsule coverage.</content>
public static partial class FixedConvex2dRelations
{
    /// <summary>
    /// Tests exact closed contact of an upright capsule with a convex polygon,
    /// including tangency and zero-radius axis contact.
    /// </summary>
    /// <remarks>
    /// Uses the same shape and polygon preconditions as
    /// <see cref="IntersectsSweptUprightCapsuleStrict"/>, but tests one stationary
    /// pose and includes the boundary. No rounded contact normal or penetration
    /// depth participates in the classification.
    /// </remarks>
    /// <exception cref="ArgumentException">Fewer than three polygon vertices were supplied.</exception>
    /// <exception cref="ArgumentOutOfRangeException">Axis length or radius is negative.</exception>
    public static bool IntersectsUprightCapsule(
        Vector2d center, Fixed64 axisLength, Fixed64 radius,
        Vector2d convexOrigin, ReadOnlySpan<Vector2d> convexVertexOffsets)
    {
        ValidatePlanarCapsule(axisLength, radius, convexVertexOffsets);
        return WideConvex2dRelations.IntersectsSweptUprightCapsule(
            center, center, axisLength, radius, convexOrigin, convexVertexOffsets, strict: false);
    }

    /// <summary>
    /// Tests whether an upright capsule swept continuously between two centers
    /// overlaps a convex polygon's interior, excluding boundary-only contact.
    /// </summary>
    /// <remarks>
    /// The capsule axis is parallel to <see cref="Vector2d.Forward"/>. Its total
    /// height is axis length plus twice radius; zero axis length is a circle.
    /// Zero radius tests entry of the swept axis into the polygon interior.
    /// Conceptual half-axis endpoints, polygon vertices and the entire chord
    /// are evaluated without scalar saturation or rounding, including odd-raw
    /// axis lengths. Vertices must be boundary-ordered and convex in either
    /// winding; repeated/collinear edges are allowed. An entirely collinear
    /// polygon has no interior. Convexity is an authoring precondition, not
    /// checked by this query. Work is linear in the vertex count, without allocation.
    /// </remarks>
    /// <exception cref="ArgumentException">Fewer than three polygon vertices were supplied.</exception>
    /// <exception cref="ArgumentOutOfRangeException">Axis length or radius is negative.</exception>
    public static bool IntersectsSweptUprightCapsuleStrict(
        Vector2d startCenter, Vector2d endCenter, Fixed64 axisLength, Fixed64 radius,
        Vector2d convexOrigin, ReadOnlySpan<Vector2d> convexVertexOffsets)
    {
        ValidatePlanarCapsule(axisLength, radius, convexVertexOffsets);
        return WideConvex2dRelations.IntersectsSweptUprightCapsule(
            startCenter, endCenter, axisLength, radius, convexOrigin, convexVertexOffsets, strict: true);
    }

    private static void ValidatePlanarCapsule(
        Fixed64 axisLength, Fixed64 radius, ReadOnlySpan<Vector2d> convexVertexOffsets)
    {
        ValidateVertexOffsets(convexVertexOffsets);
        if (axisLength < Fixed64.Zero)
            throw new ArgumentOutOfRangeException(nameof(axisLength));
        if (radius < Fixed64.Zero)
            throw new ArgumentOutOfRangeException(nameof(radius));
    }
}
