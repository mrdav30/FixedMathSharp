//=======================================================================
// FixedSegment2d.CenteredAxis.cs
//=======================================================================
// MIT License, Copyright (c) 2024–present David Oravsky (mrdav30)
// See LICENSE file in the project root for full license information.
//=======================================================================

using System;

namespace FixedMathSharp.Bounds;

/// <content>
/// Provides distance queries for conceptual "centered axis" representations
/// (a center point, direction, and length) without requiring the caller to
/// materialize the axis into concrete <see cref="FixedSegment2d"/> endpoints.
/// Includes point-to-axis and axis-to-axis distance calculations, delegating
/// the underlying wide-precision math to <see cref="WideFiniteAxisIntersection"/>.
/// </content>
public partial struct FixedSegment2d
{
    /// <summary>
    /// Attempts to return the rounded distance from a point to a conceptual
    /// centered finite axis without materializing its endpoints.
    /// </summary>
    /// <returns>
    /// True when the final distance is representable; otherwise false and
    /// <paramref name="distance"/> is <see cref="Fixed64.MaxValue"/>.
    /// </returns>
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

    /// <summary>
    /// Returns the rounded nonnegative distance from a point to a conceptual
    /// centered capsule surface, saturating to <see cref="Fixed64.MaxValue"/>
    /// when the distance is not representable.
    /// </summary>
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
}
