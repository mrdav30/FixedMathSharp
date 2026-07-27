//=======================================================================
// FixedSegment2d.Convex.cs
//=======================================================================
// MIT License, Copyright (c) 2024–present David Oravsky (mrdav30)
// See LICENSE file in the project root for full license information.
//=======================================================================

using System;

namespace FixedMathSharp.Geometry;

/// <content>
/// Capsule-versus-convex-polygon queries, including minimum translation
/// vector computation and contact point generation.
/// </content>
public partial struct FixedSegment2d
{
    /// <summary>
    /// Finds the minimum translation that separates a conceptual centered
    /// capsule from a closed convex polygon represented by an origin and
    /// origin-relative vertex offsets.
    /// </summary>
    /// <remarks>
    /// Offsets must be supplied in boundary order. Exact depth ties retain the
    /// first authored edge axis, and exact direction ties retain the first
    /// authored vertex. Neither shape's conceptual world vertices need to be
    /// representable.
    /// </remarks>
    /// <returns>
    /// <see langword="true"/> when the shapes overlap, including tangency;
    /// otherwise <see langword="false"/> and both outputs are zero.
    /// </returns>
    public static bool TryGetCenteredCapsuleConvexMinimumTranslation(
        Vector2d center,
        Vector2d axisDirection,
        Fixed64 axisLength,
        Fixed64 radius,
        Vector2d convexOrigin,
        ReadOnlySpan<Vector2d> convexVertexOffsets,
        out Vector2d normal,
        out Fixed64 depth)
    {
        ValidateCenteredAxis(axisDirection, axisLength);
        if (radius < Fixed64.Zero)
            throw new ArgumentOutOfRangeException(nameof(radius));
        if (convexVertexOffsets.Length < 3)
            throw new ArgumentException(
                "A convex polygon requires at least three vertex offsets.",
                nameof(convexVertexOffsets));

        return WideCenteredCapsule2dRelations.TryGetMinimumTranslation(
            center,
            axisDirection,
            axisLength,
            radius,
            convexOrigin,
            convexVertexOffsets,
            out normal,
            out depth);
    }

    /// <summary>
    /// Finds the minimum translation that separates a conceptual centered
    /// capsule from a rotated closed convex polygon.
    /// </summary>
    public static bool TryGetCenteredCapsuleConvexMinimumTranslation(
        Vector2d center,
        Vector2d axisDirection,
        Fixed64 axisLength,
        Fixed64 radius,
        Vector2d convexOrigin,
        Fixed64 convexRotation,
        ReadOnlySpan<Vector2d> convexVertexOffsets,
        out Vector2d normal,
        out Fixed64 depth)
    {
        ValidateCenteredAxis(axisDirection, axisLength);
        if (radius < Fixed64.Zero)
            throw new ArgumentOutOfRangeException(nameof(radius));
        if (convexVertexOffsets.Length < 3)
        {
            throw new ArgumentException(
                "A convex polygon requires at least three vertex offsets.",
                nameof(convexVertexOffsets));
        }

        return WideCenteredCapsule2dRelations.TryGetMinimumTranslation(
            center,
            axisDirection,
            axisLength,
            radius,
            convexOrigin,
            convexRotation,
            convexVertexOffsets,
            out normal,
            out depth);
    }

    /// <summary>
    /// Attempts to build contact anchors between a conceptual centered capsule
    /// and a rotated closed convex polygon.
    /// </summary>
    /// <remarks>
    /// <paramref name="localCapsuleAxisDirection"/> is expressed in the
    /// capsule frame. Returned capsule anchors retain their axial and radial
    /// feature terms separately.
    /// </remarks>
    public static bool TryGetCenteredCapsuleConvexContacts(
        Vector2d capsuleCenter,
        Fixed64 capsuleRotation,
        Vector2d localCapsuleAxisDirection,
        Fixed64 capsuleAxisLength,
        Fixed64 capsuleRadius,
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
        ValidateCenteredAxis(localCapsuleAxisDirection, capsuleAxisLength);
        if (capsuleRadius < Fixed64.Zero)
            throw new ArgumentOutOfRangeException(nameof(capsuleRadius));
        if (convexVertexOffsets.Length < 3)
        {
            throw new ArgumentException(
                "A convex polygon requires at least three vertex offsets.",
                nameof(convexVertexOffsets));
        }
        if (capsuleContacts.Length < 2)
        {
            throw new ArgumentException(
                "Contact output requires capacity for two anchors.",
                nameof(capsuleContacts));
        }
        if (convexContacts.Length < 2)
        {
            throw new ArgumentException(
                "Contact output requires capacity for two anchors.",
                nameof(convexContacts));
        }

        return WideCenteredCapsule2dRelations.TryGetContacts(
            capsuleCenter,
            capsuleRotation,
            localCapsuleAxisDirection,
            capsuleAxisLength,
            capsuleRadius,
            convexOrigin,
            convexRotation,
            convexVertexOffsets,
            capsuleContacts,
            convexContacts,
            out contactCount,
            out normal,
            out depth,
            out depthIsClamped);
    }
}
