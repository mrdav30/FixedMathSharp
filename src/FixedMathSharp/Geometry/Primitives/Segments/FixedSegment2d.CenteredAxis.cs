//=======================================================================
// FixedSegment2d.CenteredAxis.cs
//=======================================================================
// MIT License, Copyright (c) 2024-present David Oravsky (mrdav30)
// See LICENSE file in the project root for full license information.
//=======================================================================

using System;

namespace FixedMathSharp.Geometry;

/// <content>
/// Contains centered-axis and rotated-capsule geometry that avoids
/// materializing potentially unrepresentable segment endpoints.
/// </content>
public partial struct FixedSegment2d
{
    /// <summary>
    /// Returns whether the exact distance between two centered capsule axes is
    /// strictly less than their radius sum. Exact tangency is excluded.
    /// </summary>
    /// <param name="firstCenter">The first shape center in the query coordinate space.</param>
    /// <param name="firstAxisDirection">The normalized first shape axis in world coordinates.</param>
    /// <param name="firstAxisLength">The full nonnegative length of the first shape's central segment.</param>
    /// <param name="firstRadius">The first shape's nonnegative radius in coordinate units.</param>
    /// <param name="secondCenter">The second shape center in the query coordinate space.</param>
    /// <param name="secondAxisDirection">The normalized second shape axis in world coordinates.</param>
    /// <param name="secondAxisLength">The full nonnegative length of the second shape's central segment.</param>
    /// <param name="secondRadius">The second shape's nonnegative radius in coordinate units.</param>
    /// <returns>
    /// <see langword="true"/> when the exact closest core distance is strictly less than the combined radius and
    /// <see langword="false"/> otherwise.
    /// </returns>
    /// <exception cref="ArgumentException">
    /// <paramref name="firstAxisDirection"/> is not normalized or <paramref name="secondAxisDirection"/> is not
    /// normalized.
    /// </exception>
    /// <exception cref="ArgumentOutOfRangeException">
    /// <paramref name="firstAxisLength"/> is negative, <paramref name="firstRadius"/> is negative,
    /// <paramref name="secondAxisLength"/> is negative, or <paramref name="secondRadius"/> is negative.
    /// </exception>
    /// <remarks>
    /// Classification precedes endpoint, distance, and contact-depth rounding.
    /// Zero-length axes are points; a zero radius sum has no strict overlap.
    /// </remarks>
    public static bool DoCenteredCapsulesOverlapStrict(
        Vector2d firstCenter,
        Vector2d firstAxisDirection,
        Fixed64 firstAxisLength,
        Fixed64 firstRadius,
        Vector2d secondCenter,
        Vector2d secondAxisDirection,
        Fixed64 secondAxisLength,
        Fixed64 secondRadius)
    {
        ValidateCenteredAxis(firstAxisDirection, firstAxisLength,
            nameof(firstAxisDirection), nameof(firstAxisLength));
        if (firstRadius < Fixed64.Zero)
            throw new ArgumentOutOfRangeException(nameof(firstRadius));
        ValidateCenteredAxis(secondAxisDirection, secondAxisLength,
            nameof(secondAxisDirection), nameof(secondAxisLength));
        if (secondRadius < Fixed64.Zero)
            throw new ArgumentOutOfRangeException(nameof(secondRadius));

        return WideFiniteAxisIntersection.DoCenteredCapsulesOverlap(
            firstCenter, firstAxisDirection, firstAxisLength, firstRadius,
            secondCenter, secondAxisDirection, secondAxisLength, secondRadius,
            strict: true);
    }

    /// <summary>
    /// Returns whether two centered capsules expressed by scalar rigid-frame
    /// rotations have strictly positive radial overlap, excluding exact tangency.
    /// </summary>
    /// <param name="firstCenter">The first shape center in the query coordinate space.</param>
    /// <param name="firstRotation">The first shape's counterclockwise local-to-world angle in radians.</param>
    /// <param name="firstAxisLength">The full nonnegative length of the first shape's central segment.</param>
    /// <param name="firstRadius">The first shape's nonnegative radius in coordinate units.</param>
    /// <param name="secondCenter">The second shape center in the query coordinate space.</param>
    /// <param name="secondRotation">The second shape's counterclockwise local-to-world angle in radians.</param>
    /// <param name="secondAxisLength">The full nonnegative length of the second shape's central segment.</param>
    /// <param name="secondRadius">The second shape's nonnegative radius in coordinate units.</param>
    /// <returns>
    /// <see langword="true"/> when the exact closest core distance is strictly less than the combined radius and
    /// <see langword="false"/> otherwise.
    /// </returns>
    /// <exception cref="ArgumentOutOfRangeException">
    /// <paramref name="firstAxisLength"/> is negative, <paramref name="firstRadius"/> is negative,
    /// <paramref name="secondAxisLength"/> is negative, or <paramref name="secondRadius"/> is negative.
    /// </exception>
    public static bool DoCenteredCapsulesOverlapStrict(
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
            firstCenter, GetRotatedPositiveYAxis(firstRotation), firstAxisLength, firstRadius,
            secondCenter, GetRotatedPositiveYAxis(secondRotation), secondAxisLength, secondRadius,
            strict: true);
    }

    /// <summary>
    /// Attempts to return the rounded distance from a point to a conceptual
    /// centered finite axis without materializing its endpoints.
    /// </summary>
    /// <returns>
    /// True when the final distance is representable; otherwise false and
    /// <paramref name="distance"/> is <see cref="Fixed64.MaxValue"/>.
    /// </returns>
    /// <param name="point">The point in the query coordinate space.</param>
    /// <param name="center">The center in the query coordinate space.</param>
    /// <param name="axisDirection">The normalized axis direction in world coordinates.</param>
    /// <param name="axisLength">The full nonnegative length of the central segment. Zero makes the core a point.</param>
    /// <param name="distance">The rounded nonnegative distance in coordinate units on success.</param>
    /// <exception cref="ArgumentException"><paramref name="axisDirection"/> is not normalized.</exception>
    /// <exception cref="ArgumentOutOfRangeException"><paramref name="axisLength"/> is negative.</exception>
    public static bool TryGetDistanceToCenteredAxis(
        Vector2d point,
        Vector2d center,
        Vector2d axisDirection,
        Fixed64 axisLength,
        out Fixed64 distance)
    {
        ValidateCenteredAxis(axisDirection, axisLength);
        return WideFiniteAxisIntersection.TryGetDistanceToCenteredAxis(
            point,
            center,
            axisDirection,
            axisLength,
            out distance);
    }

    /// <summary>
    /// Attempts to return the rounded distance between two conceptual
    /// centered finite axes without materializing either pair of endpoints.
    /// </summary>
    /// <returns>
    /// True when the final distance is representable; otherwise false and
    /// <paramref name="distance"/> is <see cref="Fixed64.MaxValue"/>.
    /// </returns>
    /// <param name="firstCenter">The first shape center in the query coordinate space.</param>
    /// <param name="firstAxisDirection">The normalized first shape axis in world coordinates.</param>
    /// <param name="firstAxisLength">The full nonnegative length of the first shape's central segment.</param>
    /// <param name="secondCenter">The second shape center in the query coordinate space.</param>
    /// <param name="secondAxisDirection">The normalized second shape axis in world coordinates.</param>
    /// <param name="secondAxisLength">The full nonnegative length of the second shape's central segment.</param>
    /// <param name="distance">The rounded nonnegative distance in coordinate units on success.</param>
    /// <exception cref="ArgumentException">
    /// <paramref name="firstAxisDirection"/> is not normalized or <paramref name="secondAxisDirection"/> is not
    /// normalized.
    /// </exception>
    /// <exception cref="ArgumentOutOfRangeException">
    /// <paramref name="firstAxisLength"/> is negative or <paramref name="secondAxisLength"/> is negative.
    /// </exception>
    public static bool TryGetDistanceBetweenCenteredAxes(
        Vector2d firstCenter,
        Vector2d firstAxisDirection,
        Fixed64 firstAxisLength,
        Vector2d secondCenter,
        Vector2d secondAxisDirection,
        Fixed64 secondAxisLength,
        out Fixed64 distance)
    {
        ValidateCenteredAxis(
            firstAxisDirection,
            firstAxisLength,
            nameof(firstAxisDirection),
            nameof(firstAxisLength));
        ValidateCenteredAxis(
            secondAxisDirection,
            secondAxisLength,
            nameof(secondAxisDirection),
            nameof(secondAxisLength));
        return WideFiniteAxisIntersection.TryGetDistanceBetweenCenteredAxes(
            firstCenter,
            firstAxisDirection,
            firstAxisLength,
            secondCenter,
            secondAxisDirection,
            secondAxisLength,
            out distance);
    }

    /// <summary>
    /// Returns whether two conceptual centered capsules overlap, including
    /// exact tangency, without narrowing their axis distance or radius sum.
    /// </summary>
    /// <param name="firstCenter">The first shape center in the query coordinate space.</param>
    /// <param name="firstAxisDirection">The normalized first shape axis in world coordinates.</param>
    /// <param name="firstAxisLength">The full nonnegative length of the first shape's central segment.</param>
    /// <param name="firstRadius">The first shape's nonnegative radius in coordinate units.</param>
    /// <param name="secondCenter">The second shape center in the query coordinate space.</param>
    /// <param name="secondAxisDirection">The normalized second shape axis in world coordinates.</param>
    /// <param name="secondAxisLength">The full nonnegative length of the second shape's central segment.</param>
    /// <param name="secondRadius">The second shape's nonnegative radius in coordinate units.</param>
    /// <returns>
    /// <see langword="true"/> when the exact closest core distance is at most the combined radius and
    /// <see langword="false"/> otherwise.
    /// </returns>
    /// <exception cref="ArgumentException">
    /// <paramref name="firstAxisDirection"/> is not normalized or <paramref name="secondAxisDirection"/> is not
    /// normalized.
    /// </exception>
    /// <exception cref="ArgumentOutOfRangeException">
    /// <paramref name="firstAxisLength"/> is negative, <paramref name="firstRadius"/> is negative,
    /// <paramref name="secondAxisLength"/> is negative, or <paramref name="secondRadius"/> is negative.
    /// </exception>
    public static bool DoCenteredCapsulesOverlap(
        Vector2d firstCenter,
        Vector2d firstAxisDirection,
        Fixed64 firstAxisLength,
        Fixed64 firstRadius,
        Vector2d secondCenter,
        Vector2d secondAxisDirection,
        Fixed64 secondAxisLength,
        Fixed64 secondRadius)
    {
        ValidateCenteredAxis(
            firstAxisDirection,
            firstAxisLength,
            nameof(firstAxisDirection),
            nameof(firstAxisLength));
        if (firstRadius < Fixed64.Zero)
            throw new ArgumentOutOfRangeException(nameof(firstRadius));
        ValidateCenteredAxis(
            secondAxisDirection,
            secondAxisLength,
            nameof(secondAxisDirection),
            nameof(secondAxisLength));
        if (secondRadius < Fixed64.Zero)
            throw new ArgumentOutOfRangeException(nameof(secondRadius));

        return WideFiniteAxisIntersection.DoCenteredCapsulesOverlap(
            firstCenter,
            firstAxisDirection,
            firstAxisLength,
            firstRadius,
            secondCenter,
            secondAxisDirection,
            secondAxisLength,
            secondRadius);
    }

    /// <summary>
    /// Attempts to build an exact contact between two conceptual centered
    /// capsules.
    /// </summary>
    /// <param name="firstCenter">The first shape center in the query coordinate space.</param>
    /// <param name="firstAnchorRotation">
    /// The first shape anchor frame angle in radians. It does not rotate the separately supplied world axis.
    /// </param>
    /// <param name="firstAxisDirection">The normalized first shape axis in world coordinates.</param>
    /// <param name="firstAxisLength">The full nonnegative length of the first shape's central segment.</param>
    /// <param name="firstRadius">The first shape's nonnegative radius in coordinate units.</param>
    /// <param name="secondCenter">The second shape center in the query coordinate space.</param>
    /// <param name="secondAnchorRotation">
    /// The second shape anchor frame angle in radians. It does not rotate the separately supplied world axis.
    /// </param>
    /// <param name="secondAxisDirection">The normalized second shape axis in world coordinates.</param>
    /// <param name="secondAxisLength">The full nonnegative length of the second shape's central segment.</param>
    /// <param name="secondRadius">The second shape's nonnegative radius in coordinate units.</param>
    /// <param name="fallbackNormal">The normalized world-space normal used when the closest features coincide.</param>
    /// <param name="contact">
    /// The paired shape-local witnesses, world normal, and depth on success. Use this result only when the method
    /// returns <see langword="true"/>.
    /// </param>
    /// <returns>
    /// <see langword="true"/> when the closed shapes touch or overlap and the requested contact is available and
    /// <see langword="false"/> otherwise.
    /// </returns>
    /// <exception cref="ArgumentException">
    /// <paramref name="firstAxisDirection"/> is not normalized, <paramref name="secondAxisDirection"/> is not
    /// normalized, or <paramref name="fallbackNormal"/> is not normalized.
    /// </exception>
    /// <exception cref="ArgumentOutOfRangeException">
    /// <paramref name="firstAxisLength"/> is negative, <paramref name="firstRadius"/> is negative,
    /// <paramref name="secondAxisLength"/> is negative, or <paramref name="secondRadius"/> is negative.
    /// </exception>
    /// <remarks>
    /// Each surface witness retains its axial and radial contributions
    /// separately. Classification is independent of absolute world-point
    /// materialization.
    /// </remarks>
    public static bool TryGetCenteredCapsulesContact(
        Vector2d firstCenter,
        Fixed64 firstAnchorRotation,
        Vector2d firstAxisDirection,
        Fixed64 firstAxisLength,
        Fixed64 firstRadius,
        Vector2d secondCenter,
        Fixed64 secondAnchorRotation,
        Vector2d secondAxisDirection,
        Fixed64 secondAxisLength,
        Fixed64 secondRadius,
        Vector2d fallbackNormal,
        out FixedContactAnchors2d contact)
    {
        ValidateCenteredAxis(
            firstAxisDirection,
            firstAxisLength,
            nameof(firstAxisDirection),
            nameof(firstAxisLength));
        if (firstRadius < Fixed64.Zero)
            throw new ArgumentOutOfRangeException(nameof(firstRadius));
        ValidateCenteredAxis(
            secondAxisDirection,
            secondAxisLength,
            nameof(secondAxisDirection),
            nameof(secondAxisLength));
        if (secondRadius < Fixed64.Zero)
            throw new ArgumentOutOfRangeException(nameof(secondRadius));
        if (!fallbackNormal.IsNormalized())
            throw new ArgumentException("Fallback normal must be normalized.", nameof(fallbackNormal));

        return WideFiniteAxisIntersection.TryGetCenteredCapsulesContact(
            firstCenter,
            firstAnchorRotation,
            firstAxisDirection,
            firstAxisLength,
            firstRadius,
            secondCenter,
            secondAnchorRotation,
            secondAxisDirection,
            secondAxisLength,
            secondRadius,
            fallbackNormal,
            out contact);
    }

    /// <summary>
    /// Attempts to materialize one endpoint of a conceptual centered finite
    /// axis from its normalized direction and full length.
    /// </summary>
    /// <param name="center">The center in the query coordinate space.</param>
    /// <param name="axisDirection">The normalized axis direction in world coordinates.</param>
    /// <param name="axisLength">The full nonnegative length of the central segment. Zero makes the core a point.</param>
    /// <param name="positive">
    /// <see langword="true"/> selects the endpoint in the supplied axis direction. <see langword="false"/> selects
    /// the opposite endpoint.
    /// </param>
    /// <param name="endpoint">The selected endpoint in query coordinates on success.</param>
    /// <returns>
    /// <see langword="true"/> when the requested rounded result is representable and <see langword="false"/>
    /// otherwise.
    /// </returns>
    /// <exception cref="ArgumentException"><paramref name="axisDirection"/> is not normalized.</exception>
    /// <exception cref="ArgumentOutOfRangeException"><paramref name="axisLength"/> is negative.</exception>
    /// <remarks>
    /// The endpoint is rounded only after combining its center and half-length
    /// contribution. Failure is atomic.
    /// </remarks>
    public static bool TryGetCenteredAxisEndpoint(
        Vector2d center,
        Vector2d axisDirection,
        Fixed64 axisLength,
        bool positive,
        out Vector2d endpoint)
    {
        ValidateCenteredAxis(axisDirection, axisLength);
        return WideFiniteAxisIntersection.TryGetCenteredAxisEndpoint(
            center,
            axisDirection,
            axisLength,
            positive,
            out endpoint);
    }

    /// <summary>
    /// Attempts to materialize the support point of a conceptual centered
    /// capsule in a world-space direction.
    /// </summary>
    /// <param name="center">The center in the query coordinate space.</param>
    /// <param name="axisDirection">The normalized axis direction in world coordinates.</param>
    /// <param name="axisLength">The full nonnegative length of the central segment. Zero makes the core a point.</param>
    /// <param name="radius">The nonnegative radius in coordinate units.</param>
    /// <param name="direction">The world-space direction used to select support. Its magnitude is used as supplied.</param>
    /// <param name="support">The selected support point in world coordinates on success.</param>
    /// <returns>
    /// <see langword="true"/> when the requested rounded result is representable and <see langword="false"/>
    /// otherwise.
    /// </returns>
    /// <exception cref="ArgumentException"><paramref name="axisDirection"/> is not normalized.</exception>
    /// <exception cref="ArgumentOutOfRangeException">
    /// <paramref name="axisLength"/> is negative or <paramref name="radius"/> is negative.
    /// </exception>
    /// <remarks>
    /// The direction need not be normalized. A zero direction and an exact
    /// axial tie select the capsule center along the tied axis.
    /// </remarks>
    public static bool TryGetCenteredCapsuleSupport(
        Vector2d center,
        Vector2d axisDirection,
        Fixed64 axisLength,
        Fixed64 radius,
        Vector2d direction,
        out Vector2d support)
    {
        ValidateCenteredAxis(axisDirection, axisLength);
        if (radius < Fixed64.Zero)
            throw new ArgumentOutOfRangeException(nameof(radius));

        return WideFiniteAxisIntersection.TryGetCenteredCapsuleSupport(
            center,
            axisDirection,
            axisLength,
            radius,
            direction,
            out support);
    }

    /// <summary>
    /// Returns the support anchor of a conceptual centered capsule without
    /// combining its axial and radial feature components.
    /// </summary>
    /// <param name="center">The center in the query coordinate space.</param>
    /// <param name="frameRotation">The counterclockwise local-to-world rotation in radians.</param>
    /// <param name="localAxisDirection">The normalized axis in the shape's local coordinates.</param>
    /// <param name="axisLength">The full nonnegative length of the central segment. Zero makes the core a point.</param>
    /// <param name="radius">The nonnegative radius in coordinate units.</param>
    /// <param name="localDirection">The shape-local direction used to select support. Its magnitude is used as supplied.</param>
    /// <returns>A support anchor retaining the selected shape-local feature and its world frame.</returns>
    /// <exception cref="ArgumentException"><paramref name="localAxisDirection"/> is not normalized.</exception>
    /// <exception cref="ArgumentOutOfRangeException">
    /// <paramref name="axisLength"/> is negative or <paramref name="radius"/> is negative.
    /// </exception>
    /// <remarks>
    /// The direction need not be normalized. A zero direction and an exact
    /// axial tie select the axis center.
    /// </remarks>
    public static FixedPointAnchor2d GetCenteredCapsuleSupportAnchor(
        Vector2d center,
        Fixed64 frameRotation,
        Vector2d localAxisDirection,
        Fixed64 axisLength,
        Fixed64 radius,
        Vector2d localDirection)
    {
        ValidateCenteredAxis(localAxisDirection, axisLength);
        if (radius < Fixed64.Zero)
            throw new ArgumentOutOfRangeException(nameof(radius));

        return WideFiniteAxisIntersection.GetCenteredCapsuleSupportAnchor(
            center,
            frameRotation,
            localAxisDirection,
            axisLength,
            radius,
            localDirection);
    }

    /// <summary>Measures the nonnegative distance to a closed centered capsule.</summary>
    /// <param name="point">The point in the query coordinate space.</param>
    /// <param name="center">The center in the query coordinate space.</param>
    /// <param name="axisDirection">The normalized axis direction in world coordinates.</param>
    /// <param name="axisLength">The full nonnegative length of the central segment. Zero makes the core a point.</param>
    /// <param name="radius">The nonnegative radius in coordinate units.</param>
    /// <returns>
    /// The rounded distance to the closed capsule, zero for a contained point, or <see cref="Fixed64.MaxValue"/>
    /// for an unrepresentable distance.
    /// </returns>
    /// <remarks>
    /// The final value uses round-half-to-even. Points inside or on the capsule
    /// return zero.
    /// </remarks>
    /// <exception cref="ArgumentException">
    /// Thrown when <paramref name="axisDirection"/> is zero or not normalized.
    /// </exception>
    /// <exception cref="ArgumentOutOfRangeException">
    /// Thrown when <paramref name="axisLength"/> or <paramref name="radius"/>
    /// is negative.
    /// </exception>
    public static Fixed64 GetDistanceToCenteredCapsule(
        Vector2d point,
        Vector2d center,
        Vector2d axisDirection,
        Fixed64 axisLength,
        Fixed64 radius)
    {
        TryGetDistanceToCenteredCapsule(
            point,
            center,
            axisDirection,
            axisLength,
            radius,
            out Fixed64 distance);
        return distance;
    }

    /// <summary>
    /// Attempts to return the rounded nonnegative distance from a point to a
    /// conceptual centered capsule surface.
    /// </summary>
    /// <param name="point">The point in the query coordinate space.</param>
    /// <param name="center">The center in the query coordinate space.</param>
    /// <param name="axisDirection">The normalized axis direction in world coordinates.</param>
    /// <param name="axisLength">The full nonnegative length of the central segment. Zero makes the core a point.</param>
    /// <param name="radius">The nonnegative radius in coordinate units.</param>
    /// <param name="distance">The rounded nonnegative distance in coordinate units on success.</param>
    /// <remarks>The final value uses round-half-to-even.</remarks>
    /// <returns>
    /// True when the final distance is representable; otherwise false and
    /// <paramref name="distance"/> is <see cref="Fixed64.MaxValue"/>.
    /// Points inside or on the capsule return zero.
    /// </returns>
    /// <exception cref="ArgumentException">
    /// Thrown when <paramref name="axisDirection"/> is zero or not normalized.
    /// </exception>
    /// <exception cref="ArgumentOutOfRangeException">
    /// Thrown when <paramref name="axisLength"/> or <paramref name="radius"/>
    /// is negative.
    /// </exception>
    public static bool TryGetDistanceToCenteredCapsule(
        Vector2d point,
        Vector2d center,
        Vector2d axisDirection,
        Fixed64 axisLength,
        Fixed64 radius,
        out Fixed64 distance)
    {
        ValidateCenteredAxis(axisDirection, axisLength);
        if (radius < Fixed64.Zero)
            throw new ArgumentOutOfRangeException(nameof(radius));

        return WideFiniteAxisIntersection.TryGetDistanceToCenteredCapsule(
            point,
            center,
            axisDirection,
            axisLength,
            radius,
            out distance);
    }

    /// <summary>
    /// Returns whether a point lies inside or on a conceptual centered capsule.
    /// </summary>
    /// <param name="point">The point in the query coordinate space.</param>
    /// <param name="center">The center in the query coordinate space.</param>
    /// <param name="axisDirection">The normalized axis direction in world coordinates.</param>
    /// <param name="axisLength">The full nonnegative length of the central segment. Zero makes the core a point.</param>
    /// <param name="radius">The nonnegative radius in coordinate units.</param>
    /// <returns>
    /// <see langword="true"/> when the point is inside or on the boundary of the closed shape and
    /// <see langword="false"/> otherwise.
    /// </returns>
    /// <exception cref="ArgumentException">
    /// Thrown when <paramref name="axisDirection"/> is zero or not normalized.
    /// </exception>
    /// <exception cref="ArgumentOutOfRangeException">
    /// Thrown when <paramref name="axisLength"/> or <paramref name="radius"/>
    /// is negative.
    /// </exception>
    public static bool ContainsPointInCenteredCapsule(
        Vector2d point,
        Vector2d center,
        Vector2d axisDirection,
        Fixed64 axisLength,
        Fixed64 radius) =>
        ContainsPointInCenteredCapsule(
            point,
            center,
            axisDirection,
            axisLength,
            radius,
            Fixed64.Zero,
            strict: false);

    /// <summary>
    /// Returns whether a point lies strictly inside or inclusively within a
    /// conceptual centered capsule.
    /// </summary>
    /// <param name="point">The point in the query coordinate space.</param>
    /// <param name="center">The center in the query coordinate space.</param>
    /// <param name="axisDirection">The normalized axis direction in world coordinates.</param>
    /// <param name="axisLength">The full nonnegative length of the central segment. Zero makes the core a point.</param>
    /// <param name="radius">The nonnegative radius in coordinate units.</param>
    /// <param name="strict">
    /// <see langword="true"/> to require strict interior containment. <see langword="false"/> includes the
    /// boundary.
    /// </param>
    /// <returns>
    /// <see langword="true"/> when the point satisfies the requested containment test and <see langword="false"/>
    /// otherwise.
    /// </returns>
    /// <exception cref="ArgumentException"><paramref name="axisDirection"/> is not normalized.</exception>
    /// <exception cref="ArgumentOutOfRangeException">
    /// <paramref name="axisLength"/> is negative or <paramref name="radius"/> is negative.
    /// </exception>
    public static bool ContainsPointInCenteredCapsule(
        Vector2d point,
        Vector2d center,
        Vector2d axisDirection,
        Fixed64 axisLength,
        Fixed64 radius,
        bool strict) =>
        ContainsPointInCenteredCapsule(
            point,
            center,
            axisDirection,
            axisLength,
            radius,
            Fixed64.Zero,
            strict);

    /// <summary>
    /// Returns whether a point lies inside or on a conceptual centered capsule,
    /// including a nonnegative radial expansion.
    /// </summary>
    /// <param name="point">The point in the query coordinate space.</param>
    /// <param name="center">The center in the query coordinate space.</param>
    /// <param name="axisDirection">The normalized axis direction in world coordinates.</param>
    /// <param name="axisLength">The full nonnegative length of the central segment. Zero makes the core a point.</param>
    /// <param name="radius">The nonnegative radius in coordinate units.</param>
    /// <param name="radiusExpansion">A nonnegative addition to the radius in coordinate units.</param>
    /// <returns>
    /// <see langword="true"/> when the point is inside or on the boundary of the closed shape and
    /// <see langword="false"/> otherwise.
    /// </returns>
    /// <exception cref="ArgumentException">
    /// Thrown when <paramref name="axisDirection"/> is zero or not normalized.
    /// </exception>
    /// <exception cref="ArgumentOutOfRangeException">
    /// Thrown when <paramref name="axisLength"/>, <paramref name="radius"/>,
    /// or <paramref name="radiusExpansion"/> is negative.
    /// </exception>
    public static bool ContainsPointInCenteredCapsule(
        Vector2d point,
        Vector2d center,
        Vector2d axisDirection,
        Fixed64 axisLength,
        Fixed64 radius,
        Fixed64 radiusExpansion) =>
        ContainsPointInCenteredCapsule(
            point,
            center,
            axisDirection,
            axisLength,
            radius,
            radiusExpansion,
            strict: false);

    /// <summary>
    /// Returns whether a point lies strictly inside or inclusively within a
    /// conceptual centered capsule with nonnegative radial expansion.
    /// </summary>
    /// <param name="point">The point in the query coordinate space.</param>
    /// <param name="center">The center in the query coordinate space.</param>
    /// <param name="axisDirection">The normalized axis direction in world coordinates.</param>
    /// <param name="axisLength">The full nonnegative length of the central segment. Zero makes the core a point.</param>
    /// <param name="radius">The nonnegative radius in coordinate units.</param>
    /// <param name="radiusExpansion">A nonnegative addition to the radius in coordinate units.</param>
    /// <param name="strict">
    /// <see langword="true"/> to require strict interior containment. <see langword="false"/> includes the
    /// boundary.
    /// </param>
    /// <returns>
    /// <see langword="true"/> when the point satisfies the requested containment test and <see langword="false"/>
    /// otherwise.
    /// </returns>
    /// <exception cref="ArgumentException"><paramref name="axisDirection"/> is not normalized.</exception>
    /// <exception cref="ArgumentOutOfRangeException">
    /// <paramref name="axisLength"/> is negative, <paramref name="radius"/> is negative, or
    /// <paramref name="radiusExpansion"/> is negative.
    /// </exception>
    public static bool ContainsPointInCenteredCapsule(
        Vector2d point,
        Vector2d center,
        Vector2d axisDirection,
        Fixed64 axisLength,
        Fixed64 radius,
        Fixed64 radiusExpansion,
        bool strict)
    {
        ValidateCenteredAxis(axisDirection, axisLength);
        if (radius < Fixed64.Zero)
            throw new ArgumentOutOfRangeException(nameof(radius));
        if (radiusExpansion < Fixed64.Zero)
            throw new ArgumentOutOfRangeException(nameof(radiusExpansion));

        return WideFiniteAxisIntersection.ContainsPointInCenteredCapsule(
            point,
            center,
            axisDirection,
            axisLength,
            radius,
            radiusExpansion,
            strict);
    }

    /// <summary>
    /// Attempts to return the selected point on a conceptual centered capsule
    /// surface without narrowing its axis point before applying the radial offset.
    /// </summary>
    /// <param name="point">The point in the query coordinate space.</param>
    /// <param name="center">The center in the query coordinate space.</param>
    /// <param name="axisDirection">The normalized axis direction in world coordinates.</param>
    /// <param name="axisLength">The full nonnegative length of the central segment. Zero makes the core a point.</param>
    /// <param name="radius">The nonnegative radius in coordinate units.</param>
    /// <param name="surfaceDirection">
    /// The normalized world-space direction in which to offset the closest core point by the radius.
    /// </param>
    /// <param name="surfacePoint">The selected surface point in world coordinates on success.</param>
    /// <remarks>
    /// <paramref name="surfaceDirection"/> must be the nonzero result of
    /// <see cref="GetDirectionFromCenteredAxis(Vector2d, Vector2d, Vector2d, Fixed64)"/>,
    /// or a caller-selected normalized
    /// direction perpendicular to the axis when that method returns zero.
    /// </remarks>
    /// <returns>
    /// True when every final surface coordinate is representable; otherwise
    /// false and <paramref name="surfacePoint"/> is zero.
    /// </returns>
    /// <exception cref="ArgumentException">
    /// Thrown when <paramref name="axisDirection"/> is zero or not normalized,
    /// or when <paramref name="surfaceDirection"/> is not normalized.
    /// </exception>
    /// <exception cref="ArgumentOutOfRangeException">
    /// Thrown when <paramref name="axisLength"/> or <paramref name="radius"/>
    /// is negative.
    /// </exception>
    public static bool TryGetSurfacePointOnCenteredCapsule(
        Vector2d point,
        Vector2d center,
        Vector2d axisDirection,
        Fixed64 axisLength,
        Fixed64 radius,
        Vector2d surfaceDirection,
        out Vector2d surfacePoint)
    {
        ValidateCenteredAxis(axisDirection, axisLength);
        if (radius < Fixed64.Zero)
            throw new ArgumentOutOfRangeException(nameof(radius));
        if (!surfaceDirection.IsNormalized())
            throw new ArgumentException("Surface direction must be normalized.", nameof(surfaceDirection));

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
    /// Attempts to return the center-relative surface offset on a conceptual
    /// centered capsule nearest to a world-space point in the supplied surface
    /// direction.
    /// </summary>
    /// <param name="point">The point in the query coordinate space.</param>
    /// <param name="center">The center in the query coordinate space.</param>
    /// <param name="axisDirection">The normalized axis direction in world coordinates.</param>
    /// <param name="axisLength">The full nonnegative length of the central segment. Zero makes the core a point.</param>
    /// <param name="radius">The nonnegative radius in coordinate units.</param>
    /// <param name="surfaceDirection">
    /// The normalized world-space direction in which to offset the closest core point by the radius.
    /// </param>
    /// <param name="surfaceOffset">
    /// The selected surface offset relative to the shape center, expressed along world axes on success.
    /// </param>
    /// <returns>
    /// <see langword="true"/> when the requested rounded result is representable and <see langword="false"/>
    /// otherwise.
    /// </returns>
    /// <exception cref="ArgumentException">
    /// <paramref name="axisDirection"/> is not normalized or <paramref name="surfaceDirection"/> is not normalized.
    /// </exception>
    /// <exception cref="ArgumentOutOfRangeException">
    /// <paramref name="axisLength"/> is negative or <paramref name="radius"/> is negative.
    /// </exception>
    public static bool TryGetSurfaceOffsetOnCenteredCapsule(
        Vector2d point,
        Vector2d center,
        Vector2d axisDirection,
        Fixed64 axisLength,
        Fixed64 radius,
        Vector2d surfaceDirection,
        out Vector2d surfaceOffset)
    {
        ValidateCenteredAxis(axisDirection, axisLength);
        if (radius < Fixed64.Zero)
            throw new ArgumentOutOfRangeException(nameof(radius));
        if (!surfaceDirection.IsNormalized())
        {
            throw new ArgumentException(
                "Surface direction must be normalized.",
                nameof(surfaceDirection));
        }

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
    /// point in the supplied normalized surface direction.
    /// </summary>
    /// <param name="point">The point in the query coordinate space.</param>
    /// <param name="center">The center in the query coordinate space.</param>
    /// <param name="frameRotation">The counterclockwise local-to-world rotation in radians.</param>
    /// <param name="localAxisDirection">The normalized axis in the shape's local coordinates.</param>
    /// <param name="axisLength">The full nonnegative length of the central segment. Zero makes the core a point.</param>
    /// <param name="radius">The nonnegative radius in coordinate units.</param>
    /// <param name="localSurfaceDirection">
    /// The normalized shape-local direction in which to offset the closest core point by the radius.
    /// </param>
    /// <returns>The selected point retained in the shape's local frame.</returns>
    /// <exception cref="ArgumentException">
    /// <paramref name="localAxisDirection"/> is not normalized or <paramref name="localSurfaceDirection"/> is not
    /// normalized.
    /// </exception>
    /// <exception cref="ArgumentOutOfRangeException">
    /// <paramref name="axisLength"/> is negative or <paramref name="radius"/> is negative.
    /// </exception>
    public static FixedPointAnchor2d GetSurfaceAnchorOnCenteredCapsule(
        Vector2d point,
        Vector2d center,
        Fixed64 frameRotation,
        Vector2d localAxisDirection,
        Fixed64 axisLength,
        Fixed64 radius,
        Vector2d localSurfaceDirection)
    {
        ValidateCenteredAxis(localAxisDirection, axisLength);
        if (radius < Fixed64.Zero)
            throw new ArgumentOutOfRangeException(nameof(radius));
        if (!localSurfaceDirection.IsNormalized())
        {
            throw new ArgumentException(
                "Surface direction must be normalized.",
                nameof(localSurfaceDirection));
        }

        return WideFiniteAxisIntersection.GetSurfaceAnchorOnCenteredCapsule(
            point,
            center,
            frameRotation,
            localAxisDirection,
            axisLength,
            radius,
            localSurfaceDirection);
    }

    /// <summary>
    /// Returns the normalized direction from the closest point on a conceptual
    /// centered finite axis toward <paramref name="point"/>, or zero when the
    /// point lies on that axis.
    /// </summary>
    /// <param name="point">The point in the query coordinate space.</param>
    /// <param name="center">The center in the query coordinate space.</param>
    /// <param name="axisDirection">The normalized axis direction in world coordinates.</param>
    /// <param name="axisLength">The full nonnegative length of the central segment. Zero makes the core a point.</param>
    /// <returns>
    /// The normalized world direction from the closest point on the finite axis toward the query point, or zero for
    /// a point on the axis.
    /// </returns>
    /// <remarks>
    /// The conceptual endpoints are
    /// <c>center +/- axisDirection * (axisLength / 2)</c>. They are never
    /// constructed or narrowed before the closest-feature decision.
    /// </remarks>
    /// <exception cref="ArgumentException">
    /// Thrown when <paramref name="axisDirection"/> is zero or not normalized.
    /// </exception>
    /// <exception cref="ArgumentOutOfRangeException">
    /// Thrown when <paramref name="axisLength"/> is negative.
    /// </exception>
    public static Vector2d GetDirectionFromCenteredAxis(
        Vector2d point,
        Vector2d center,
        Vector2d axisDirection,
        Fixed64 axisLength)
    {
        ValidateCenteredAxis(axisDirection, axisLength);

        return WideFiniteAxisIntersection.GetDirectionFromCenteredAxis(
            point,
            center,
            axisDirection,
            axisLength);
    }

    /// <summary>
    /// Attempts to return the closest points on two conceptual centered finite
    /// axes described by normalized directions and full lengths.
    /// </summary>
    /// <returns>
    /// <see langword="true"/> when both final world points are representable;
    /// otherwise <see langword="false"/> and both outputs are zero.
    /// </returns>
    /// <param name="firstCenter">The first shape center in the query coordinate space.</param>
    /// <param name="firstAxisDirection">The normalized first shape axis in world coordinates.</param>
    /// <param name="firstAxisLength">The full nonnegative length of the first shape's central segment.</param>
    /// <param name="secondCenter">The second shape center in the query coordinate space.</param>
    /// <param name="secondAxisDirection">The normalized second shape axis in world coordinates.</param>
    /// <param name="secondAxisLength">The full nonnegative length of the second shape's central segment.</param>
    /// <param name="firstPoint">The closest point on the first axis in world coordinates on success.</param>
    /// <param name="secondPoint">The closest point on the second axis in world coordinates on success.</param>
    /// <exception cref="ArgumentException">
    /// <paramref name="firstAxisDirection"/> is not normalized or <paramref name="secondAxisDirection"/> is not
    /// normalized.
    /// </exception>
    /// <exception cref="ArgumentOutOfRangeException">
    /// <paramref name="firstAxisLength"/> is negative or <paramref name="secondAxisLength"/> is negative.
    /// </exception>
    public static bool TryGetClosestPointsBetweenCenteredAxes(
        Vector2d firstCenter,
        Vector2d firstAxisDirection,
        Fixed64 firstAxisLength,
        Vector2d secondCenter,
        Vector2d secondAxisDirection,
        Fixed64 secondAxisLength,
        out Vector2d firstPoint,
        out Vector2d secondPoint)
    {
        ValidateCenteredAxis(
            firstAxisDirection,
            firstAxisLength,
            nameof(firstAxisDirection),
            nameof(firstAxisLength));
        ValidateCenteredAxis(
            secondAxisDirection,
            secondAxisLength,
            nameof(secondAxisDirection),
            nameof(secondAxisLength));
        return WideFiniteAxisIntersection.TryGetClosestPointsBetweenCenteredAxes(
            firstCenter,
            firstAxisDirection,
            firstAxisLength,
            secondCenter,
            secondAxisDirection,
            secondAxisLength,
            out firstPoint,
            out secondPoint);
    }

    private static void ValidateCenteredAxis(Vector2d axisDirection, Fixed64 axisLength)
        => ValidateCenteredAxis(
            axisDirection,
            axisLength,
            nameof(axisDirection),
            nameof(axisLength));

    private static void ValidateCenteredAxis(
        Vector2d axisDirection,
        Fixed64 axisLength,
        string directionParameterName,
        string lengthParameterName)
    {
        if (!axisDirection.IsNormalized())
            throw new ArgumentException("Axis direction must be normalized.", directionParameterName);
        if (axisLength < Fixed64.Zero)
            throw new ArgumentOutOfRangeException(lengthParameterName);
    }

    /// <summary>
    /// Returns whether a point lies inside or on a centered rotated capsule.
    /// </summary>
    /// <param name="point">The point in the query coordinate space.</param>
    /// <param name="center">The center in the query coordinate space.</param>
    /// <param name="frameRotation">The counterclockwise local-to-world rotation in radians.</param>
    /// <param name="axisLength">The full nonnegative length of the central segment. Zero makes the core a point.</param>
    /// <param name="radius">The nonnegative radius in coordinate units.</param>
    /// <returns>
    /// <see langword="true"/> when the point is inside or on the boundary of the closed shape and
    /// <see langword="false"/> otherwise.
    /// </returns>
    /// <exception cref="ArgumentOutOfRangeException">
    /// <paramref name="axisLength"/> is negative or <paramref name="radius"/> is negative.
    /// </exception>
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
    /// <param name="point">The point in the query coordinate space.</param>
    /// <param name="center">The center in the query coordinate space.</param>
    /// <param name="frameRotation">The counterclockwise local-to-world rotation in radians.</param>
    /// <param name="axisLength">The full nonnegative length of the central segment. Zero makes the core a point.</param>
    /// <param name="radius">The nonnegative radius in coordinate units.</param>
    /// <param name="radiusExpansion">A nonnegative addition to the radius in coordinate units.</param>
    /// <returns>
    /// <see langword="true"/> when the point is inside or on the boundary of the closed shape and
    /// <see langword="false"/> otherwise.
    /// </returns>
    /// <exception cref="ArgumentOutOfRangeException">
    /// <paramref name="axisLength"/> is negative, <paramref name="radius"/> is negative, or
    /// <paramref name="radiusExpansion"/> is negative.
    /// </exception>
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
    /// <param name="point">The point in the query coordinate space.</param>
    /// <param name="center">The center in the query coordinate space.</param>
    /// <param name="frameRotation">The counterclockwise local-to-world rotation in radians.</param>
    /// <param name="axisLength">The full nonnegative length of the central segment. Zero makes the core a point.</param>
    /// <param name="radius">The nonnegative radius in coordinate units.</param>
    /// <param name="radiusExpansion">A nonnegative addition to the radius in coordinate units.</param>
    /// <param name="strict">
    /// <see langword="true"/> to require strict interior containment. <see langword="false"/> includes the
    /// boundary.
    /// </param>
    /// <returns>
    /// <see langword="true"/> when the point satisfies the requested containment test and <see langword="false"/>
    /// otherwise.
    /// </returns>
    /// <exception cref="ArgumentOutOfRangeException">
    /// <paramref name="axisLength"/> is negative, <paramref name="radius"/> is negative, or
    /// <paramref name="radiusExpansion"/> is negative.
    /// </exception>
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
    /// <param name="point">The point in the query coordinate space.</param>
    /// <param name="center">The center in the query coordinate space.</param>
    /// <param name="frameRotation">The counterclockwise local-to-world rotation in radians.</param>
    /// <param name="axisLength">The full nonnegative length of the central segment. Zero makes the core a point.</param>
    /// <returns>
    /// The normalized world direction from the closest point on the finite axis toward the query point, or zero for
    /// a point on the axis.
    /// </returns>
    /// <exception cref="ArgumentOutOfRangeException"><paramref name="axisLength"/> is negative.</exception>
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

    /// <summary>Measures the nonnegative distance to a closed centered capsule.</summary>
    /// <param name="point">The point in the query coordinate space.</param>
    /// <param name="center">The center in the query coordinate space.</param>
    /// <param name="frameRotation">The counterclockwise local-to-world rotation in radians.</param>
    /// <param name="axisLength">The full nonnegative length of the central segment. Zero makes the core a point.</param>
    /// <param name="radius">The nonnegative radius in coordinate units.</param>
    /// <returns>
    /// The rounded distance to the closed capsule, zero for a contained point, or <see cref="Fixed64.MaxValue"/>
    /// for an unrepresentable distance.
    /// </returns>
    /// <exception cref="ArgumentOutOfRangeException">
    /// <paramref name="axisLength"/> is negative or <paramref name="radius"/> is negative.
    /// </exception>
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
    /// <param name="point">The point in the query coordinate space.</param>
    /// <param name="center">The center in the query coordinate space.</param>
    /// <param name="frameRotation">The counterclockwise local-to-world rotation in radians.</param>
    /// <param name="axisLength">The full nonnegative length of the central segment. Zero makes the core a point.</param>
    /// <param name="radius">The nonnegative radius in coordinate units.</param>
    /// <param name="distance">The rounded nonnegative distance in coordinate units on success.</param>
    /// <returns><see langword="true"/> when the rounded distance is representable and <see langword="false"/> otherwise.</returns>
    /// <exception cref="ArgumentOutOfRangeException">
    /// <paramref name="axisLength"/> is negative or <paramref name="radius"/> is negative.
    /// </exception>
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
    /// <param name="center">The center in the query coordinate space.</param>
    /// <param name="frameRotation">The counterclockwise local-to-world rotation in radians.</param>
    /// <param name="axisLength">The full nonnegative length of the central segment. Zero makes the core a point.</param>
    /// <param name="radius">The nonnegative radius in coordinate units.</param>
    /// <param name="direction">The world-space direction used to select support. Its magnitude is used as supplied.</param>
    /// <param name="support">The selected support point in world coordinates on success.</param>
    /// <returns>
    /// <see langword="true"/> when the requested rounded result is representable and <see langword="false"/>
    /// otherwise.
    /// </returns>
    /// <exception cref="ArgumentOutOfRangeException">
    /// <paramref name="axisLength"/> is negative or <paramref name="radius"/> is negative.
    /// </exception>
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
    /// <param name="point">The point in the query coordinate space.</param>
    /// <param name="center">The center in the query coordinate space.</param>
    /// <param name="frameRotation">The counterclockwise local-to-world rotation in radians.</param>
    /// <param name="axisLength">The full nonnegative length of the central segment. Zero makes the core a point.</param>
    /// <param name="radius">The nonnegative radius in coordinate units.</param>
    /// <param name="surfaceDirection">
    /// The normalized world-space direction in which to offset the closest core point by the radius.
    /// </param>
    /// <param name="surfacePoint">The selected surface point in world coordinates on success.</param>
    /// <returns>
    /// <see langword="true"/> when the requested rounded result is representable and <see langword="false"/>
    /// otherwise.
    /// </returns>
    /// <exception cref="ArgumentException"><paramref name="surfaceDirection"/> is not normalized.</exception>
    /// <exception cref="ArgumentOutOfRangeException">
    /// <paramref name="axisLength"/> is negative or <paramref name="radius"/> is negative.
    /// </exception>
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
    /// <param name="point">The point in the query coordinate space.</param>
    /// <param name="center">The center in the query coordinate space.</param>
    /// <param name="frameRotation">The counterclockwise local-to-world rotation in radians.</param>
    /// <param name="axisLength">The full nonnegative length of the central segment. Zero makes the core a point.</param>
    /// <param name="radius">The nonnegative radius in coordinate units.</param>
    /// <param name="surfaceDirection">
    /// The normalized world-space direction in which to offset the closest core point by the radius.
    /// </param>
    /// <param name="surfaceOffset">
    /// The selected surface offset relative to the shape center, expressed along world axes on success.
    /// </param>
    /// <returns>
    /// <see langword="true"/> when the requested rounded result is representable and <see langword="false"/>
    /// otherwise.
    /// </returns>
    /// <exception cref="ArgumentException"><paramref name="surfaceDirection"/> is not normalized.</exception>
    /// <exception cref="ArgumentOutOfRangeException">
    /// <paramref name="axisLength"/> is negative or <paramref name="radius"/> is negative.
    /// </exception>
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
    /// <param name="firstCenter">The first shape center in the query coordinate space.</param>
    /// <param name="firstRotation">The first shape's counterclockwise local-to-world angle in radians.</param>
    /// <param name="firstAxisLength">The full nonnegative length of the first shape's central segment.</param>
    /// <param name="firstRadius">The first shape's nonnegative radius in coordinate units.</param>
    /// <param name="secondCenter">The second shape center in the query coordinate space.</param>
    /// <param name="secondRotation">The second shape's counterclockwise local-to-world angle in radians.</param>
    /// <param name="secondAxisLength">The full nonnegative length of the second shape's central segment.</param>
    /// <param name="secondRadius">The second shape's nonnegative radius in coordinate units.</param>
    /// <returns>
    /// <see langword="true"/> when the exact closest core distance is at most the combined radius and
    /// <see langword="false"/> otherwise.
    /// </returns>
    /// <exception cref="ArgumentOutOfRangeException">
    /// <paramref name="firstAxisLength"/> is negative, <paramref name="firstRadius"/> is negative,
    /// <paramref name="secondAxisLength"/> is negative, or <paramref name="secondRadius"/> is negative.
    /// </exception>
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
    /// <param name="firstCenter">The first shape center in the query coordinate space.</param>
    /// <param name="firstRotation">The first shape's counterclockwise local-to-world angle in radians.</param>
    /// <param name="firstAxisLength">The full nonnegative length of the first shape's central segment.</param>
    /// <param name="firstRadius">The first shape's nonnegative radius in coordinate units.</param>
    /// <param name="secondCenter">The second shape center in the query coordinate space.</param>
    /// <param name="secondRotation">The second shape's counterclockwise local-to-world angle in radians.</param>
    /// <param name="secondAxisLength">The full nonnegative length of the second shape's central segment.</param>
    /// <param name="secondRadius">The second shape's nonnegative radius in coordinate units.</param>
    /// <param name="fallbackNormal">The normalized world-space normal used when the closest features coincide.</param>
    /// <param name="contact">
    /// The paired shape-local witnesses, world normal, and depth on success. Use this result only when the method
    /// returns <see langword="true"/>.
    /// </param>
    /// <returns>
    /// <see langword="true"/> when the closed shapes touch or overlap and the requested contact is available and
    /// <see langword="false"/> otherwise.
    /// </returns>
    /// <exception cref="ArgumentException"><paramref name="fallbackNormal"/> is not normalized.</exception>
    /// <exception cref="ArgumentOutOfRangeException">
    /// <paramref name="firstAxisLength"/> is negative, <paramref name="firstRadius"/> is negative,
    /// <paramref name="secondAxisLength"/> is negative, or <paramref name="secondRadius"/> is negative.
    /// </exception>
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
    /// <param name="center">The center in the query coordinate space.</param>
    /// <param name="rotation">The counterclockwise local-to-world rotation in radians.</param>
    /// <param name="axisLength">The full nonnegative length of the central segment. Zero makes the core a point.</param>
    /// <param name="radius">The nonnegative radius in coordinate units.</param>
    /// <param name="convexOrigin">The polygon frame's origin in world coordinates.</param>
    /// <param name="convexVertexOffsets">
    /// At least three polygon offsets in boundary order, in either winding. The span is read during the call.
    /// </param>
    /// <param name="normal">The world-space normal from the curved shape toward the polygon when a contact is returned.</param>
    /// <param name="depth">The minimum exit distance rounded to fixed point on success, or zero on failure.</param>
    /// <returns><see langword="true"/> when the closed shapes touch or overlap and <see langword="false"/> otherwise.</returns>
    /// <exception cref="ArgumentException"><paramref name="convexVertexOffsets"/> has fewer than three vertices.</exception>
    /// <exception cref="ArgumentOutOfRangeException">
    /// <paramref name="axisLength"/> is negative or <paramref name="radius"/> is negative.
    /// </exception>
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
    /// <param name="center">The center in the query coordinate space.</param>
    /// <param name="rotation">The counterclockwise local-to-world rotation in radians.</param>
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
    /// <exception cref="ArgumentException"><paramref name="direction"/> is not normalized.</exception>
    /// <exception cref="ArgumentOutOfRangeException">
    /// <paramref name="axisLength"/> is negative, <paramref name="radius"/> is negative,
    /// <paramref name="maximumDistance"/> is negative, or <paramref name="targetRadius"/> is negative.
    /// </exception>
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
    /// <param name="center">The center in the query coordinate space.</param>
    /// <param name="rotation">The counterclockwise local-to-world rotation in radians.</param>
    /// <param name="axisLength">The full nonnegative length of the central segment. Zero makes the core a point.</param>
    /// <param name="radius">The nonnegative radius in coordinate units.</param>
    /// <param name="direction">The normalized query direction in world coordinates.</param>
    /// <param name="maximumDistance">The nonnegative maximum travel distance in coordinate units, including the endpoint.</param>
    /// <param name="targetCenter">The stationary target center in the query coordinate space.</param>
    /// <param name="targetRotation">The stationary target's counterclockwise local-to-world angle in radians.</param>
    /// <param name="targetAxisLength">The full nonnegative length of the stationary target's central segment.</param>
    /// <param name="targetRadius">The stationary target's nonnegative radius in coordinate units.</param>
    /// <param name="distance">The first contact distance measured from the start on success.</param>
    /// <param name="normal">The sweep contact normal in world coordinates on success.</param>
    /// <returns>
    /// <see langword="true"/> when a contact is found within the tested travel interval and <see langword="false"/>
    /// otherwise.
    /// </returns>
    /// <exception cref="ArgumentException"><paramref name="direction"/> is not normalized.</exception>
    /// <exception cref="ArgumentOutOfRangeException">
    /// <paramref name="axisLength"/> is negative, <paramref name="radius"/> is negative,
    /// <paramref name="maximumDistance"/> is negative, <paramref name="targetAxisLength"/> is negative, or
    /// <paramref name="targetRadius"/> is negative.
    /// </exception>
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
    /// <param name="center">The center in the query coordinate space.</param>
    /// <param name="rotation">The counterclockwise local-to-world rotation in radians.</param>
    /// <param name="axisLength">The full nonnegative length of the central segment. Zero makes the core a point.</param>
    /// <param name="radius">The nonnegative radius in coordinate units.</param>
    /// <param name="radiusExpansion">A nonnegative addition to the radius in coordinate units.</param>
    /// <param name="totalDistance">
    /// The caller-supplied distance mapped to the full segment. Zero is allowed only for coincident endpoints.
    /// </param>
    /// <param name="entryDistance">The first distance along the authored segment interval on success.</param>
    /// <param name="exitDistance">The last distance along the authored segment interval on success.</param>
    /// <param name="startContained">
    /// <see langword="true"/> when the exact start lies inside or on the expanded shape boundary.
    /// </param>
    /// <param name="endContainedStrict"><see langword="true"/> when the exact end lies strictly inside the expanded shape.</param>
    /// <returns>
    /// <see langword="true"/> when the path intersects the closed shape within the tested interval and
    /// <see langword="false"/> otherwise.
    /// </returns>
    /// <remarks>The exact chord parameter is mapped to the authored distance interval before the endpoints are rounded.</remarks>
    /// <exception cref="ArgumentException"><paramref name="totalDistance"/> is zero for a nonzero-length segment.</exception>
    /// <exception cref="ArgumentOutOfRangeException">
    /// <paramref name="axisLength"/> is negative, <paramref name="radius"/> is negative,
    /// <paramref name="radiusExpansion"/> is negative, or <paramref name="totalDistance"/> is negative.
    /// </exception>
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
