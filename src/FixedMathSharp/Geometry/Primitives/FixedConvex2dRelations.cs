//=======================================================================
// FixedConvex2dRelations.cs
//=======================================================================
// MIT License, Copyright (c) 2024-present David Oravsky (mrdav30)
// See LICENSE file in the project root for full license information.
//=======================================================================

using System;

namespace FixedMathSharp.Geometry;

/// <summary>
/// Provides exact relations for convex 2D shapes represented by canonical
/// origins, scalar rotations, and local vertex offsets.
/// </summary>
public static partial class FixedConvex2dRelations
{
    /// <summary>
    /// Returns whether boundary-ordered vertices form a strictly convex
    /// polygon in either winding direction.
    /// </summary>
    public static bool IsStrictlyConvex(
        ReadOnlySpan<Vector2d> convexVertexOffsets)
    {
        ValidateVertexOffsets(convexVertexOffsets);
        return WideConvex2dRelations.IsStrictlyConvex(convexVertexOffsets);
    }

    /// <summary>
    /// Returns whether a world-space point lies inside or on a closed convex
    /// polygon represented by an origin and origin-relative vertices.
    /// </summary>
    public static bool ContainsPoint(
        Vector2d point,
        Vector2d convexOrigin,
        ReadOnlySpan<Vector2d> convexVertexOffsets)
    {
        ValidateVertexOffsets(convexVertexOffsets);
        return WideConvex2dRelations.ContainsPoint(
            point,
            Vector2d.Zero,
            convexOrigin,
            convexVertexOffsets);
    }

    /// <summary>
    /// Returns whether a world-space point lies inside or on a rotated closed
    /// convex polygon represented by a frame and local vertex offsets.
    /// </summary>
    public static bool ContainsPoint(
        Vector2d point,
        Vector2d convexOrigin,
        Fixed64 convexRotation,
        ReadOnlySpan<Vector2d> convexVertexOffsets)
    {
        ValidateVertexOffsets(convexVertexOffsets);
        return WideConvex2dRelations.ContainsPoint(
            point,
            Vector2d.Zero,
            convexOrigin,
            convexRotation,
            convexVertexOffsets);
    }

    /// <summary>
    /// Finds the polygon-relative closest point to a world-space point without
    /// materializing polygon vertices.
    /// </summary>
    public static Vector2d GetClosestPointOffset(
        Vector2d point,
        Vector2d convexOrigin,
        ReadOnlySpan<Vector2d> convexVertexOffsets)
    {
        ValidateVertexOffsets(convexVertexOffsets);
        return WideConvex2dRelations.GetClosestPointOffset(
            point,
            Vector2d.Zero,
            convexOrigin,
            convexVertexOffsets);
    }

    /// <summary>
    /// Finds the closest point on a rotated polygon while retaining that point
    /// in the polygon's supplied local frame.
    /// </summary>
    public static FixedPointAnchor2d GetClosestPointAnchor(
        Vector2d point,
        Vector2d convexOrigin,
        Fixed64 convexRotation,
        ReadOnlySpan<Vector2d> convexVertexOffsets)
    {
        ValidateVertexOffsets(convexVertexOffsets);
        return new FixedPointAnchor2d(
            convexOrigin,
            convexRotation,
            WideConvex2dRelations.GetClosestPointOffset(
                point,
                Vector2d.Zero,
                convexOrigin,
                convexRotation,
                convexVertexOffsets));
    }

    /// <summary>
    /// Returns the first authored polygon-relative vertex with the greatest
    /// exact projection along a direction.
    /// </summary>
    public static Vector2d GetSupportOffset(
        ReadOnlySpan<Vector2d> convexVertexOffsets,
        Vector2d direction)
    {
        ValidateVertexOffsets(convexVertexOffsets);
        return WideConvex2dRelations.GetSupportOffset(
            convexVertexOffsets,
            direction);
    }

    /// <summary>
    /// Returns the first local vertex with the greatest exact world-space
    /// projection along a direction.
    /// </summary>
    public static FixedPointAnchor2d GetSupportAnchor(
        Vector2d convexOrigin,
        Fixed64 convexRotation,
        ReadOnlySpan<Vector2d> convexVertexOffsets,
        Vector2d direction)
    {
        ValidateVertexOffsets(convexVertexOffsets);
        return new FixedPointAnchor2d(
            convexOrigin,
            convexRotation,
            WideConvex2dRelations.GetSupportOffset(
                convexRotation,
                convexVertexOffsets,
                direction));
    }

    /// <summary>
    /// Attempts to derive the area and centroid of a boundary-ordered convex
    /// polygon without narrowing cross products or weighted-coordinate sums.
    /// </summary>
    /// <remarks>
    /// Area is nonnegative and clamps to <see cref="Fixed64.MaxValue"/> when
    /// the conceptual value exceeds the scalar domain. The centroid uses one
    /// final round-half-to-even conversion per component.
    /// </remarks>
    /// <returns>
    /// <see langword="false"/> only when the polygon has exact zero signed
    /// area or its final centroid is outside the scalar domain.
    /// </returns>
    public static bool TryGetAreaAndCentroid(
        ReadOnlySpan<Vector2d> convexVertexOffsets,
        out Fixed64 area,
        out Vector2d centroid)
    {
        ValidateVertexOffsets(convexVertexOffsets);
        return WideConvex2dRelations.TryGetAreaAndCentroid(
            convexVertexOffsets,
            out area,
            out centroid);
    }

    /// <summary>
    /// Attempts to build up to two local-frame contact-anchor pairs between
    /// two rotated closed convex polygons.
    /// </summary>
    public static bool TryGetConvexContacts(
        Vector2d firstOrigin,
        Fixed64 firstRotation,
        ReadOnlySpan<Vector2d> firstVertexOffsets,
        Vector2d secondOrigin,
        Fixed64 secondRotation,
        ReadOnlySpan<Vector2d> secondVertexOffsets,
        Span<FixedPointAnchor2d> firstContacts,
        Span<FixedPointAnchor2d> secondContacts,
        out int contactCount,
        out Vector2d normal,
        out Fixed64 depth,
        out bool depthIsClamped)
    {
        ValidateVertexOffsets(firstVertexOffsets);
        ValidateVertexOffsets(secondVertexOffsets);
        if (firstContacts.Length < 2)
        {
            throw new ArgumentException(
                "Contact output requires capacity for two anchors.",
                nameof(firstContacts));
        }
        if (secondContacts.Length < 2)
        {
            throw new ArgumentException(
                "Contact output requires capacity for two anchors.",
                nameof(secondContacts));
        }

        Span<Vector2d> firstOffsets = stackalloc Vector2d[2];
        Span<Vector2d> secondOffsets = stackalloc Vector2d[2];
        if (!WideConvex2dRelations.TryGetContactOffsets(
                firstOrigin,
                firstRotation,
                firstVertexOffsets,
                secondOrigin,
                secondRotation,
                secondVertexOffsets,
                firstOffsets,
                secondOffsets,
                out contactCount,
                out normal,
                out depth,
                out depthIsClamped))
        {
            return false;
        }

        for (int i = 0; i < contactCount; i++)
        {
            firstContacts[i] = new FixedPointAnchor2d(
                firstOrigin,
                firstRotation,
                firstOffsets[i]);
            secondContacts[i] = new FixedPointAnchor2d(
                secondOrigin,
                secondRotation,
                secondOffsets[i]);
        }
        return true;
    }

    /// <summary>
    /// Attempts to build a contact-anchor pair between a conceptual circle and
    /// a rotated closed convex polygon.
    /// </summary>
    /// <remarks>
    /// A circle's rotation does not change its geometry, but it defines the
    /// rigid local frame of the returned circle anchor.
    /// </remarks>
    public static bool TryGetCircleContact(
        Vector2d circleCenter,
        Fixed64 circleRotation,
        Fixed64 circleRadius,
        Vector2d convexOrigin,
        Fixed64 convexRotation,
        ReadOnlySpan<Vector2d> convexVertexOffsets,
        out FixedPointAnchor2d circleContact,
        out FixedPointAnchor2d convexContact,
        out Vector2d normal,
        out Fixed64 depth,
        out bool depthIsClamped)
    {
        if (circleRadius < Fixed64.Zero)
            throw new ArgumentOutOfRangeException(nameof(circleRadius));
        ValidateVertexOffsets(convexVertexOffsets);

        Span<FixedPointAnchor2d> circleContacts =
            stackalloc FixedPointAnchor2d[2];
        Span<FixedPointAnchor2d> convexContacts =
            stackalloc FixedPointAnchor2d[2];
        if (!WideCenteredCapsule2dRelations.TryGetContacts(
                circleCenter,
                circleRotation,
                Vector2d.Right,
                Fixed64.Zero,
                circleRadius,
                convexOrigin,
                convexRotation,
                convexVertexOffsets,
                circleContacts,
                convexContacts,
                out _,
                out normal,
                out depth,
                out depthIsClamped))
        {
            circleContact = default;
            convexContact = default;
            return false;
        }

        circleContact = circleContacts[0];
        convexContact = convexContacts[0];
        return true;
    }

    private static void ValidateVertexOffsets(
        ReadOnlySpan<Vector2d> convexVertexOffsets)
    {
        if (convexVertexOffsets.Length < 3)
        {
            throw new ArgumentException(
                "A convex polygon requires at least three vertex offsets.",
                nameof(convexVertexOffsets));
        }
    }
}
