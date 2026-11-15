//=======================================================================
// FixedSegment2d.ConvexQueries.cs
//=======================================================================
// MIT License, Copyright (c) 2024-present David Oravsky (mrdav30)
// See LICENSE file in the project root for full license information.
//=======================================================================

using System;

namespace FixedMathSharp.Geometry;

/// <content>
/// Contains centered-capsule versus convex-polygon contact and sweep queries.
/// </content>
public partial struct FixedSegment2d
{
    /// <summary>
    /// Returns whether a centered capsule expressed by a scalar rigid-frame
    /// rotation strictly penetrates a rotated convex polygon.
    /// </summary>
    /// <param name="center">The center in the query coordinate space.</param>
    /// <param name="rotation">The counterclockwise local-to-world rotation in radians.</param>
    /// <param name="axisLength">The full nonnegative length of the central segment. Zero makes the core a point.</param>
    /// <param name="radius">The nonnegative radius in coordinate units.</param>
    /// <param name="convexOrigin">The polygon frame's origin in world coordinates.</param>
    /// <param name="convexRotation">The polygon's counterclockwise local-to-world angle in radians.</param>
    /// <param name="convexVertexOffsets">
    /// At least three polygon offsets in boundary order, in either winding. The span is read during the call.
    /// </param>
    /// <returns>
    /// <see langword="true"/> when the exact minimum separating translation has positive length and
    /// <see langword="false"/> otherwise.
    /// </returns>
    /// <exception cref="ArgumentException"><paramref name="convexVertexOffsets"/> has fewer than three vertices.</exception>
    /// <exception cref="ArgumentOutOfRangeException">
    /// <paramref name="axisLength"/> is negative or <paramref name="radius"/> is negative.
    /// </exception>
    /// <remarks>
    /// The capsule's local positive Y axis uses the authored sine and cosine
    /// coefficients without renormalizing the derived world axis. Exact touch
    /// is excluded; positive penetration survives contact-depth rounding to zero.
    /// </remarks>
    public static bool DoesCenteredCapsulePenetrateConvex(
        Vector2d center,
        Fixed64 rotation,
        Fixed64 axisLength,
        Fixed64 radius,
        Vector2d convexOrigin,
        Fixed64 convexRotation,
        ReadOnlySpan<Vector2d> convexVertexOffsets) =>
        DoesCenteredCapsulePenetrateConvex(
            center, GetRotatedPositiveYAxis(rotation), axisLength, radius,
            convexOrigin, convexRotation, convexVertexOffsets);

    /// <summary>
    /// Returns whether a conceptual centered capsule has strictly positive
    /// penetration into a rotated convex polygon, excluding exact tangency.
    /// </summary>
    /// <param name="center">The center in the query coordinate space.</param>
    /// <param name="axisDirection">The normalized axis direction in world coordinates.</param>
    /// <param name="axisLength">The full nonnegative length of the central segment. Zero makes the core a point.</param>
    /// <param name="radius">The nonnegative radius in coordinate units.</param>
    /// <param name="convexOrigin">The polygon frame's origin in world coordinates.</param>
    /// <param name="convexRotation">The polygon's counterclockwise local-to-world angle in radians.</param>
    /// <param name="convexVertexOffsets">
    /// At least three polygon offsets in boundary order, in either winding. The span is read during the call.
    /// </param>
    /// <returns>
    /// <see langword="true"/> when the exact minimum separating translation has positive length and
    /// <see langword="false"/> otherwise.
    /// </returns>
    /// <exception cref="ArgumentException">
    /// <paramref name="axisDirection"/> is not normalized or <paramref name="convexVertexOffsets"/> has fewer than
    /// three vertices.
    /// </exception>
    /// <exception cref="ArgumentOutOfRangeException">
    /// <paramref name="axisLength"/> is negative or <paramref name="radius"/> is negative.
    /// </exception>
    /// <remarks>
    /// Offsets must describe a convex polygon in boundary order. The world-space
    /// capsule axis and rotated polygon vertices retain their exact authored
    /// components through classification; no normal or contact depth is rounded.
    /// Repeated vertices retain the finite closed hull: collinear offsets form
    /// a segment, and coincident offsets form a point. A point target strictly
    /// penetrates only when its minimum separating translation is positive.
    /// For a nondegenerate polygon, a zero-radius point or finite axis must enter
    /// its interior. A noncollapsed collinear boundary instead uses positive
    /// minimum separating translation: a transverse axis crossing can penetrate,
    /// while a point on the line or an axis running along it cannot.
    /// </remarks>
    public static bool DoesCenteredCapsulePenetrateConvex(
        Vector2d center,
        Vector2d axisDirection,
        Fixed64 axisLength,
        Fixed64 radius,
        Vector2d convexOrigin,
        Fixed64 convexRotation,
        ReadOnlySpan<Vector2d> convexVertexOffsets)
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

        return WideCenteredCapsule2dRelations.DoesCapsuleOverlapConvex(
            center, axisDirection, axisLength, radius,
            convexOrigin, convexRotation, convexVertexOffsets);
    }

    /// <summary>
    /// Finds the minimum translation that separates a conceptual centered
    /// capsule from a closed convex polygon represented by an origin and
    /// origin-relative vertex offsets.
    /// </summary>
    /// <param name="center">The center in the query coordinate space.</param>
    /// <param name="axisDirection">The normalized axis direction in world coordinates.</param>
    /// <param name="axisLength">The full nonnegative length of the central segment. Zero makes the core a point.</param>
    /// <param name="radius">The nonnegative radius in coordinate units.</param>
    /// <param name="convexOrigin">The polygon frame's origin in world coordinates.</param>
    /// <param name="convexVertexOffsets">
    /// At least three polygon offsets in boundary order, in either winding. The span is read during the call.
    /// </param>
    /// <param name="normal">The world-space normal from the curved shape toward the polygon when a contact is returned.</param>
    /// <param name="depth">The minimum exit distance rounded to fixed point on success, or zero on failure.</param>
    /// <exception cref="ArgumentException">
    /// <paramref name="axisDirection"/> is not normalized or <paramref name="convexVertexOffsets"/> has fewer than
    /// three vertices.
    /// </exception>
    /// <exception cref="ArgumentOutOfRangeException">
    /// <paramref name="axisLength"/> is negative or <paramref name="radius"/> is negative.
    /// </exception>
    /// <remarks>
    /// Offsets must be supplied in boundary order. Exact depth ties retain the
    /// first authored edge axis, and exact direction ties retain the first
    /// authored vertex. Neither shape's conceptual world vertices need to be
    /// representable. Classification and depth ordering retain unnormalized
    /// axes and exact radial extents. The selected normal is then normalized
    /// to representable components, and depth rounds once to nearest-even.
    /// Depths outside the scalar domain saturate to <see cref="Fixed64.MaxValue"/>.
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
    /// <param name="center">The center in the query coordinate space.</param>
    /// <param name="axisDirection">The normalized axis direction in world coordinates.</param>
    /// <param name="axisLength">The full nonnegative length of the central segment. Zero makes the core a point.</param>
    /// <param name="radius">The nonnegative radius in coordinate units.</param>
    /// <param name="convexOrigin">The polygon frame's origin in world coordinates.</param>
    /// <param name="convexRotation">The polygon's counterclockwise local-to-world angle in radians.</param>
    /// <param name="convexVertexOffsets">
    /// At least three polygon offsets in boundary order, in either winding. The span is read during the call.
    /// </param>
    /// <param name="normal">The world-space normal from the curved shape toward the polygon when a contact is returned.</param>
    /// <param name="depth">The minimum exit distance rounded to fixed point on success, or zero on failure.</param>
    /// <returns><see langword="true"/> when the closed shapes touch or overlap and <see langword="false"/> otherwise.</returns>
    /// <exception cref="ArgumentException">
    /// <paramref name="axisDirection"/> is not normalized or <paramref name="convexVertexOffsets"/> has fewer than
    /// three vertices.
    /// </exception>
    /// <exception cref="ArgumentOutOfRangeException">
    /// <paramref name="axisLength"/> is negative or <paramref name="radius"/> is negative.
    /// </exception>
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
    /// <param name="capsuleCenter">The capsule center in the query coordinate space.</param>
    /// <param name="capsuleRotation">The capsule's counterclockwise local-to-world angle in radians.</param>
    /// <param name="localCapsuleAxisDirection">The normalized capsule axis in shape-local coordinates.</param>
    /// <param name="capsuleAxisLength">The full nonnegative length of the capsule's central segment.</param>
    /// <param name="capsuleRadius">The capsule's nonnegative radius in coordinate units.</param>
    /// <param name="convexOrigin">The polygon frame's origin in world coordinates.</param>
    /// <param name="convexRotation">The polygon's counterclockwise local-to-world angle in radians.</param>
    /// <param name="convexVertexOffsets">
    /// At least three polygon offsets in boundary order, in either winding. The span is read during the call.
    /// </param>
    /// <param name="capsuleContacts">
    /// Caller-owned storage with capacity for two anchors. The first <paramref name="contactCount"/> elements are
    /// overwritten on success and match the other buffer by index. The unused tail and the buffer on a miss are
    /// preserved.
    /// </param>
    /// <param name="convexContacts">
    /// Caller-owned storage with capacity for two anchors. The first <paramref name="contactCount"/> elements are
    /// overwritten on success and match the other buffer by index. The unused tail and the buffer on a miss are
    /// preserved.
    /// </param>
    /// <param name="contactCount">The number of valid matched pairs, from one to two on success and zero on failure.</param>
    /// <param name="normal">The world-space normal from the curved shape toward the polygon when a contact is returned.</param>
    /// <param name="depth">The rounded penetration depth in coordinate units when a contact is returned.</param>
    /// <param name="depthIsClamped"><see langword="true"/> if the selected depth exceeds <see cref="Fixed64.MaxValue"/>.</param>
    /// <returns>
    /// <see langword="true"/> when the closed shapes touch or overlap and the requested contact is available and
    /// <see langword="false"/> otherwise.
    /// </returns>
    /// <exception cref="ArgumentException">
    /// <paramref name="localCapsuleAxisDirection"/> is not normalized, <paramref name="convexVertexOffsets"/> has
    /// fewer than three vertices, <paramref name="capsuleContacts"/> has fewer than two elements, or
    /// <paramref name="convexContacts"/> has fewer than two elements.
    /// </exception>
    /// <exception cref="ArgumentOutOfRangeException">
    /// <paramref name="capsuleAxisLength"/> is negative or <paramref name="capsuleRadius"/> is negative.
    /// </exception>
    /// <remarks>
    /// <paramref name="localCapsuleAxisDirection"/> is expressed in the
    /// capsule frame. Returned capsule anchors retain their axial and radial
    /// feature terms separately. Closed-contact classification precedes normal
    /// and depth rounding, so exact tangency returns true with zero depth.
    /// Supply nonoverlapping output spans to preserve both witnesses of each pair.
    /// A single side contact projects the opposing polygon feature onto the
    /// capsule's finite side before rounding its axial parameter.
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

    /// <summary>
    /// Finds the first distance where a translated conceptual centered
    /// capsule intersects an endpoint-authored capsule axis.
    /// </summary>
    /// <param name="center">The center in the query coordinate space.</param>
    /// <param name="axisDirection">The normalized axis direction in world coordinates.</param>
    /// <param name="axisLength">The full nonnegative length of the central segment. Zero makes the core a point.</param>
    /// <param name="radius">The nonnegative radius in coordinate units.</param>
    /// <param name="direction">The normalized query direction in world coordinates.</param>
    /// <param name="maximumDistance">The nonnegative maximum travel distance in coordinate units, including the endpoint.</param>
    /// <param name="targetAxis">The stationary capsule's central segment in world coordinates.</param>
    /// <param name="targetRadius">The stationary target's nonnegative radius in coordinate units.</param>
    /// <param name="distance">The first contact distance measured from the start on success.</param>
    /// <param name="normal">The sweep contact normal in world coordinates on success.</param>
    /// <returns>
    /// <see langword="true"/> when a contact is found within the tested travel interval and <see langword="false"/>
    /// otherwise.
    /// </returns>
    /// <exception cref="ArgumentException">
    /// <paramref name="axisDirection"/> is not normalized or <paramref name="direction"/> is not normalized.
    /// </exception>
    /// <exception cref="ArgumentOutOfRangeException">
    /// <paramref name="axisLength"/> is negative, <paramref name="radius"/> is negative,
    /// <paramref name="maximumDistance"/> is negative, or <paramref name="targetRadius"/> is negative.
    /// </exception>
    public static bool TryGetSweptCenteredCapsuleSegmentFirstDistance(
        Vector2d center,
        Vector2d axisDirection,
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
            axisDirection,
            axisLength,
            radius,
            direction,
            maximumDistance,
            targetRadius);
        return WideFiniteAxisIntersection.TryGetSweptCenteredCapsuleSegmentFirstDistance(
            center,
            axisDirection,
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
    /// Finds the first distance where a translated conceptual centered
    /// capsule intersects another conceptual centered capsule.
    /// </summary>
    /// <param name="center">The center in the query coordinate space.</param>
    /// <param name="axisDirection">The normalized axis direction in world coordinates.</param>
    /// <param name="axisLength">The full nonnegative length of the central segment. Zero makes the core a point.</param>
    /// <param name="radius">The nonnegative radius in coordinate units.</param>
    /// <param name="direction">The normalized query direction in world coordinates.</param>
    /// <param name="maximumDistance">The nonnegative maximum travel distance in coordinate units, including the endpoint.</param>
    /// <param name="targetCenter">The stationary target center in the query coordinate space.</param>
    /// <param name="targetAxisDirection">The normalized stationary target axis in world coordinates.</param>
    /// <param name="targetAxisLength">The full nonnegative length of the stationary target's central segment.</param>
    /// <param name="targetRadius">The stationary target's nonnegative radius in coordinate units.</param>
    /// <param name="distance">The first contact distance measured from the start on success.</param>
    /// <param name="normal">The sweep contact normal in world coordinates on success.</param>
    /// <returns>
    /// <see langword="true"/> when a contact is found within the tested travel interval and <see langword="false"/>
    /// otherwise.
    /// </returns>
    /// <exception cref="ArgumentException">
    /// <paramref name="axisDirection"/> is not normalized, <paramref name="direction"/> is not normalized, or
    /// <paramref name="targetAxisDirection"/> is not normalized.
    /// </exception>
    /// <exception cref="ArgumentOutOfRangeException">
    /// <paramref name="axisLength"/> is negative, <paramref name="radius"/> is negative,
    /// <paramref name="maximumDistance"/> is negative, <paramref name="targetAxisLength"/> is negative, or
    /// <paramref name="targetRadius"/> is negative.
    /// </exception>
    public static bool TryGetSweptCenteredCapsulesFirstDistance(
        Vector2d center,
        Vector2d axisDirection,
        Fixed64 axisLength,
        Fixed64 radius,
        Vector2d direction,
        Fixed64 maximumDistance,
        Vector2d targetCenter,
        Vector2d targetAxisDirection,
        Fixed64 targetAxisLength,
        Fixed64 targetRadius,
        out Fixed64 distance,
        out Vector2d normal)
    {
        ValidateSweep(
            axisDirection,
            axisLength,
            radius,
            direction,
            maximumDistance,
            targetRadius);
        ValidateCenteredAxis(
            targetAxisDirection,
            targetAxisLength,
            nameof(targetAxisDirection),
            nameof(targetAxisLength));
        return WideFiniteAxisIntersection.TryGetSweptCenteredCapsulesFirstDistance(
            center,
            axisDirection,
            axisLength,
            radius,
            direction,
            maximumDistance,
            targetCenter,
            targetAxisDirection,
            targetAxisLength,
            targetRadius,
            out distance,
            out normal);
    }

    private static void ValidateSweep(
        Vector2d axisDirection,
        Fixed64 axisLength,
        Fixed64 radius,
        Vector2d direction,
        Fixed64 maximumDistance,
        Fixed64 targetRadius)
    {
        ValidateCenteredAxis(axisDirection, axisLength);
        if (radius < Fixed64.Zero)
            throw new ArgumentOutOfRangeException(nameof(radius));
        if (!direction.IsNormalized())
            throw new ArgumentException("Sweep direction must be normalized.", nameof(direction));
        if (maximumDistance < Fixed64.Zero)
            throw new ArgumentOutOfRangeException(nameof(maximumDistance));
        if (targetRadius < Fixed64.Zero)
            throw new ArgumentOutOfRangeException(nameof(targetRadius));
    }
}
