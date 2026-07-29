//=======================================================================
// FixedSegment.CenteredAxis.cs
//=======================================================================
// MIT License, Copyright (c) 2024–present David Oravsky (mrdav30)
// See LICENSE file in the project root for full license information.
//=======================================================================

using System;

namespace FixedMathSharp.Geometry;

/// <content>
/// Contains methods for computing the distance, support, and point containment 
/// of centered 3D capsules, finite cylinders, and finite cones without narrowing their conceptual axes.
/// </content>
public partial struct FixedSegment
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
        Vector3d point,
        Vector3d center,
        Vector3d axisDirection,
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
        Vector3d firstCenter,
        Vector3d firstAxisDirection,
        Fixed64 firstAxisLength,
        Vector3d secondCenter,
        Vector3d secondAxisDirection,
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
        Vector3d firstCenter,
        Vector3d firstAxisDirection,
        Fixed64 firstAxisLength,
        Fixed64 firstRadius,
        Vector3d secondCenter,
        Vector3d secondAxisDirection,
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
    /// Attempts to materialize one endpoint of a conceptual centered finite
    /// axis from its normalized direction and full length.
    /// </summary>
    /// <remarks>
    /// The endpoint is rounded only after combining its center and half-length
    /// contribution. Failure is atomic.
    /// </remarks>
    public static bool TryGetCenteredAxisEndpoint(
        Vector3d center,
        Vector3d axisDirection,
        Fixed64 axisLength,
        bool positive,
        out Vector3d endpoint)
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
    /// axial tie select the axis center.
    /// </remarks>
    public static bool TryGetCenteredCapsuleSupport(
        Vector3d center,
        Vector3d axisDirection,
        Fixed64 axisLength,
        Fixed64 radius,
        Vector3d direction,
        out Vector3d support)
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
    /// Attempts to return the support offset of a conceptual centered capsule
    /// relative to its center.
    /// </summary>
    /// <remarks>
    /// The direction need not be normalized. A zero direction and an exact
    /// axial tie select the axis center.
    /// </remarks>
    public static bool TryGetCenteredCapsuleSupportOffset(
        Vector3d axisDirection,
        Fixed64 axisLength,
        Fixed64 radius,
        Vector3d direction,
        out Vector3d supportOffset)
    {
        ValidateCenteredAxis(axisDirection, axisLength);
        if (radius < Fixed64.Zero)
            throw new ArgumentOutOfRangeException(nameof(radius));

        return WideFiniteAxisIntersection.TryGetCenteredCapsuleSupport(
            Vector3d.Zero,
            axisDirection,
            axisLength,
            radius,
            direction,
            out supportOffset);
    }

    /// <summary>
    /// Returns the support anchor of a centered capsule whose local positive Y
    /// axis is its conceptual center axis.
    /// </summary>
    /// <remarks>
    /// The rigid frame remains authoritative, so support construction does not
    /// feed the rounded derived world axis back into the geometry. The
    /// direction need not be normalized. A zero direction and an exact axial
    /// tie select the axis center.
    /// </remarks>
    public static FixedPointAnchor GetCenteredCapsuleSupportAnchor(
        Vector3d center,
        FixedQuaternion rotation,
        Fixed64 axisLength,
        Fixed64 radius,
        Vector3d direction)
    {
        if (!rotation.IsNormalized())
        {
            throw new ArgumentException(
                "Capsule rotation must be normalized.",
                nameof(rotation));
        }
        if (axisLength < Fixed64.Zero)
            throw new ArgumentOutOfRangeException(nameof(axisLength));
        if (radius < Fixed64.Zero)
            throw new ArgumentOutOfRangeException(nameof(radius));

        return WideGeometry.GetCenteredCapsuleSupportAnchor(
            center,
            rotation,
            axisLength,
            radius,
            direction);
    }

    /// <summary>
    /// Attempts to materialize the support point of a conceptual centered
    /// finite cylinder in a world-space direction.
    /// </summary>
    /// <remarks>
    /// The direction need not be normalized. A zero direction and an exact
    /// axial tie select the negative cap center. A direction parallel to the
    /// axis selects the cap center rather than an arbitrary rim point.
    /// </remarks>
    public static bool TryGetCenteredFiniteCylinderSupport(
        Vector3d center,
        Vector3d axisDirection,
        Fixed64 axisLength,
        Fixed64 radius,
        Vector3d direction,
        out Vector3d support)
    {
        if (!axisDirection.IsNormalized())
            throw new ArgumentException("Finite cylinder axis direction must be normalized.", nameof(axisDirection));
        if (axisLength <= Fixed64.Zero)
            throw new ArgumentOutOfRangeException(nameof(axisLength));
        if (radius < Fixed64.Zero)
            throw new ArgumentOutOfRangeException(nameof(radius));

        return WideFiniteAxisIntersection.TryGetCenteredFiniteCylinderSupport(
            center,
            axisDirection,
            axisLength,
            radius,
            direction,
            out support);
    }

    /// <summary>
    /// Attempts to return the support offset of a conceptual centered finite
    /// cylinder relative to its center.
    /// </summary>
    public static bool TryGetCenteredFiniteCylinderSupportOffset(
        Vector3d axisDirection,
        Fixed64 axisLength,
        Fixed64 radius,
        Vector3d direction,
        out Vector3d supportOffset)
    {
        if (!axisDirection.IsNormalized())
            throw new ArgumentException("Finite cylinder axis direction must be normalized.", nameof(axisDirection));
        if (axisLength <= Fixed64.Zero)
            throw new ArgumentOutOfRangeException(nameof(axisLength));
        if (radius < Fixed64.Zero)
            throw new ArgumentOutOfRangeException(nameof(radius));

        return WideFiniteAxisIntersection.TryGetCenteredFiniteCylinderSupport(
            Vector3d.Zero,
            axisDirection,
            axisLength,
            radius,
            direction,
            out supportOffset);
    }

    /// <summary>
    /// Returns the support anchor of a centered finite cylinder whose local
    /// positive Y axis is its conceptual center axis.
    /// </summary>
    /// <remarks>
    /// The rigid frame remains authoritative. Axial full-length/2 and radial
    /// direction-times-radius terms remain exact through final world-point or
    /// relative-offset materialization.
    /// </remarks>
    public static FixedPointAnchor GetCenteredFiniteCylinderSupportAnchor(
        Vector3d center,
        FixedQuaternion rotation,
        Fixed64 axisLength,
        Fixed64 radius,
        Vector3d direction)
    {
        if (!rotation.IsNormalized())
        {
            throw new ArgumentException(
                "Finite cylinder rotation must be normalized.",
                nameof(rotation));
        }
        if (axisLength <= Fixed64.Zero)
            throw new ArgumentOutOfRangeException(nameof(axisLength));
        if (radius < Fixed64.Zero)
            throw new ArgumentOutOfRangeException(nameof(radius));

        return WideGeometry.GetCenteredCylinderSupportAnchor(
            center,
            rotation,
            axisLength,
            radius,
            direction);
    }

    /// <summary>
    /// Attempts to materialize the support point of a conceptual centered
    /// finite cone in a world-space direction.
    /// </summary>
    /// <remarks>
    /// The axis points from the base toward the apex. The direction need not
    /// be normalized. Equal apex and base projections, including a zero
    /// direction, select the base.
    /// </remarks>
    public static bool TryGetCenteredFiniteConeSupport(
        Vector3d center,
        Vector3d axisDirection,
        Fixed64 height,
        Fixed64 radius,
        Vector3d direction,
        out Vector3d support)
    {
        if (!axisDirection.IsNormalized())
            throw new ArgumentException("Finite cone axis direction must be normalized.", nameof(axisDirection));
        if (height <= Fixed64.Zero)
            throw new ArgumentOutOfRangeException(nameof(height));
        if (radius < Fixed64.Zero)
            throw new ArgumentOutOfRangeException(nameof(radius));

        return WideFiniteAxisIntersection.TryGetCenteredFiniteConeSupport(
            center,
            axisDirection,
            height,
            radius,
            direction,
            out support);
    }

    /// <summary>
    /// Attempts to return the support offset of a conceptual centered finite
    /// cone relative to its center.
    /// </summary>
    public static bool TryGetCenteredFiniteConeSupportOffset(
        Vector3d axisDirection,
        Fixed64 height,
        Fixed64 radius,
        Vector3d direction,
        out Vector3d supportOffset)
    {
        if (!axisDirection.IsNormalized())
            throw new ArgumentException("Finite cone axis direction must be normalized.", nameof(axisDirection));
        if (height <= Fixed64.Zero)
            throw new ArgumentOutOfRangeException(nameof(height));
        if (radius < Fixed64.Zero)
            throw new ArgumentOutOfRangeException(nameof(radius));

        return WideFiniteAxisIntersection.TryGetCenteredFiniteConeSupport(
            Vector3d.Zero,
            axisDirection,
            height,
            radius,
            direction,
            out supportOffset);
    }

    /// <summary>
    /// Returns the support point of a centered finite cone as a local point in
    /// its rigid frame without materializing an absolute world coordinate.
    /// </summary>
    /// <remarks>
    /// The cone's local positive Y axis points from the base toward the apex.
    /// The direction need not be normalized. Equal apex and base projections,
    /// including a zero direction, select the base.
    /// </remarks>
    public static FixedPointAnchor GetCenteredFiniteConeSupportAnchor(
        Vector3d center,
        FixedQuaternion rotation,
        Fixed64 height,
        Fixed64 radius,
        Vector3d direction)
    {
        if (!rotation.IsNormalized())
        {
            throw new ArgumentException(
                "Finite cone rotation must be normalized.",
                nameof(rotation));
        }
        if (height <= Fixed64.Zero)
            throw new ArgumentOutOfRangeException(nameof(height));
        if (radius < Fixed64.Zero)
            throw new ArgumentOutOfRangeException(nameof(radius));

        return WideGeometry.GetCenteredConeSupportAnchor(
            center,
            rotation,
            height,
            radius,
            direction);
    }

    /// <summary>
    /// Returns the support point of a centered finite cone in a deterministic
    /// canonical axis frame without materializing an absolute world
    /// coordinate.
    /// </summary>
    public static FixedPointAnchor GetCenteredFiniteConeSupportAnchor(
        Vector3d center,
        Vector3d axisDirection,
        Fixed64 height,
        Fixed64 radius,
        Vector3d direction)
    {
        if (!axisDirection.IsNormalized())
        {
            throw new ArgumentException(
                "Finite cone axis direction must be normalized.",
                nameof(axisDirection));
        }
        if (height <= Fixed64.Zero)
            throw new ArgumentOutOfRangeException(nameof(height));
        if (radius < Fixed64.Zero)
            throw new ArgumentOutOfRangeException(nameof(radius));

        FixedQuaternion rotation =
            WideGeometry.GetCanonicalAxisRotation(axisDirection);
        return WideGeometry.GetCenteredConeSupportAnchor(
            center,
            rotation,
            height,
            radius,
            direction);
    }

    /// <summary>
    /// Returns whether a point lies in a centered finite cylinder whose flat
    /// caps are defined without constructing scalar coordinates.
    /// </summary>
    /// <param name="point">The point to classify.</param>
    /// <param name="center">The cylinder center.</param>
    /// <param name="axisDirection">The normalized cap-normal direction.</param>
    /// <param name="axisLength">The positive full physical axis length.</param>
    /// <param name="radius">The nonnegative radial extent.</param>
    /// <param name="strict">
    /// <see langword="true"/> to exclude the side and both flat caps;
    /// otherwise, boundaries are included.
    /// </param>
    /// <exception cref="ArgumentException">
    /// <paramref name="axisDirection"/> is zero or not normalized.
    /// </exception>
    /// <exception cref="ArgumentOutOfRangeException">
    /// <paramref name="axisLength"/> is not positive or
    /// <paramref name="radius"/> is negative.
    /// </exception>
    public static bool ContainsPointInCenteredFiniteCylinder(
        Vector3d point,
        Vector3d center,
        Vector3d axisDirection,
        Fixed64 axisLength,
        Fixed64 radius,
        bool strict = false)
    {
        if (!axisDirection.IsNormalized())
            throw new ArgumentException("Finite cylinder axis direction must be normalized.", nameof(axisDirection));
        if (axisLength <= Fixed64.Zero)
            throw new ArgumentOutOfRangeException(nameof(axisLength));
        if (radius < Fixed64.Zero)
            throw new ArgumentOutOfRangeException(nameof(radius));

        return WideFiniteAxisIntersection.ContainsPointInCenteredFiniteCylinder(
            point,
            center,
            axisDirection,
            axisLength,
            radius,
            strict);
    }

    /// <summary>
    /// Returns whether a point lies in a centered finite cylinder after
    /// independently expanding its flat caps and radial side.
    /// </summary>
    /// <param name="point">The point to classify.</param>
    /// <param name="center">The cylinder center.</param>
    /// <param name="axisDirection">The normalized cylinder axis.</param>
    /// <param name="axisLength">The positive full cylinder length.</param>
    /// <param name="radius">The nonnegative cylinder radius.</param>
    /// <param name="axialTolerance">
    /// The nonnegative distance added beyond each flat cap.
    /// </param>
    /// <param name="radialTolerance">
    /// The nonnegative distance added to the radial extent.
    /// </param>
    /// <param name="strict">
    /// <see langword="true"/> to exclude every expanded boundary.
    /// </param>
    public static bool ContainsPointInCenteredFiniteCylinder(
        Vector3d point,
        Vector3d center,
        Vector3d axisDirection,
        Fixed64 axisLength,
        Fixed64 radius,
        Fixed64 axialTolerance,
        Fixed64 radialTolerance,
        bool strict = false)
    {
        if (!axisDirection.IsNormalized())
            throw new ArgumentException("Finite cylinder axis direction must be normalized.", nameof(axisDirection));
        if (axisLength <= Fixed64.Zero)
            throw new ArgumentOutOfRangeException(nameof(axisLength));
        if (radius < Fixed64.Zero)
            throw new ArgumentOutOfRangeException(nameof(radius));
        if (axialTolerance < Fixed64.Zero)
            throw new ArgumentOutOfRangeException(nameof(axialTolerance));
        if (radialTolerance < Fixed64.Zero)
            throw new ArgumentOutOfRangeException(nameof(radialTolerance));

        return WideFiniteAxisIntersection.ContainsPointInCenteredFiniteCylinder(
            point,
            center,
            axisDirection,
            axisLength,
            radius,
            axialTolerance,
            radialTolerance,
            strict);
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
        Vector3d point,
        Vector3d center,
        Vector3d axisDirection,
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
        Vector3d point,
        Vector3d center,
        Vector3d axisDirection,
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
        Vector3d point,
        Vector3d center,
        Vector3d axisDirection,
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
        Vector3d point,
        Vector3d center,
        Vector3d axisDirection,
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
        Vector3d point,
        Vector3d center,
        Vector3d axisDirection,
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
        Vector3d point,
        Vector3d center,
        Vector3d axisDirection,
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
    public static Vector3d GetDirectionFromCenteredAxis(
        Vector3d point,
        Vector3d center,
        Vector3d axisDirection,
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
        Vector3d firstCenter,
        Vector3d firstAxisDirection,
        Fixed64 firstAxisLength,
        Vector3d secondCenter,
        Vector3d secondAxisDirection,
        Fixed64 secondAxisLength,
        out Vector3d firstPoint,
        out Vector3d secondPoint)
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

    /// <summary>
    /// Attempts to return the closest offsets on two conceptual centered
    /// finite axes, each relative to its own center.
    /// </summary>
    /// <remarks>
    /// This relation remains usable when either absolute closest point would
    /// exceed the public scalar coordinate range.
    /// </remarks>
    public static bool TryGetClosestOffsetsBetweenCenteredAxes(
        Vector3d firstCenter,
        Vector3d firstAxisDirection,
        Fixed64 firstAxisLength,
        Vector3d secondCenter,
        Vector3d secondAxisDirection,
        Fixed64 secondAxisLength,
        out Vector3d firstCenterOffset,
        out Vector3d secondCenterOffset)
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
        return WideFiniteAxisIntersection.TryGetClosestOffsetsBetweenCenteredAxes(
            firstCenter,
            firstAxisDirection,
            firstAxisLength,
            secondCenter,
            secondAxisDirection,
            secondAxisLength,
            out firstCenterOffset,
            out secondCenterOffset);
    }

    /// <summary>
    /// Returns the normalized direction from the closest point on the first
    /// conceptual centered axis to the closest point on the second.
    /// </summary>
    /// <remarks>
    /// Coincident closest points return zero. The direction is resolved from
    /// the exact wide relation without materializing either world point.
    /// </remarks>
    public static Vector3d GetClosestDirectionBetweenCenteredAxes(
        Vector3d firstCenter,
        Vector3d firstAxisDirection,
        Fixed64 firstAxisLength,
        Vector3d secondCenter,
        Vector3d secondAxisDirection,
        Fixed64 secondAxisLength)
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
        return WideFiniteAxisIntersection.GetClosestDirectionBetweenCenteredAxes(
            firstCenter,
            firstAxisDirection,
            firstAxisLength,
            secondCenter,
            secondAxisDirection,
            secondAxisLength);
    }

    /// <summary>
    /// Attempts to build the exact contact relation between two conceptual
    /// centered capsules without narrowing either finite axis.
    /// </summary>
    /// <remarks>
    /// Each capsule axis is expressed in its normalized rigid frame. The
    /// returned normal points from the first capsule toward the second.
    /// <paramref name="fallbackNormal"/> is used only when the closest axis
    /// points coincide. Axial and radial terms remain separate in each anchor,
    /// so classification does not depend on materializing a combined world
    /// point or center-relative offset.
    /// </remarks>
    /// <returns>
    /// <see langword="true"/> when the capsules overlap; otherwise
    /// <see langword="false"/>.
    /// </returns>
    public static bool TryGetCenteredCapsulesContact(
        Vector3d firstCenter,
        FixedQuaternion firstRotation,
        Vector3d firstLocalAxisDirection,
        Fixed64 firstAxisLength,
        Fixed64 firstRadius,
        Vector3d secondCenter,
        FixedQuaternion secondRotation,
        Vector3d secondLocalAxisDirection,
        Fixed64 secondAxisLength,
        Fixed64 secondRadius,
        Vector3d fallbackNormal,
        out FixedContactAnchors contact)
    {
        ValidateNormalizedRotation(firstRotation, nameof(firstRotation));
        ValidateNormalizedRotation(secondRotation, nameof(secondRotation));
        ValidateCenteredCapsulesContact(
            firstLocalAxisDirection,
            firstAxisLength,
            firstRadius,
            secondLocalAxisDirection,
            secondAxisLength,
            secondRadius,
            fallbackNormal);
        if (!WideOrientedBox.DoCenteredRigidCapsulesOverlap(
                firstCenter,
                firstRotation,
                firstLocalAxisDirection,
                firstAxisLength,
                firstRadius,
                secondCenter,
                secondRotation,
                secondLocalAxisDirection,
                secondAxisLength,
                secondRadius))
        {
            contact = default;
            return false;
        }
        _ = firstRotation.TryRotate(
            firstLocalAxisDirection,
            out Vector3d firstAxisDirection);
        _ = secondRotation.TryRotate(
            secondLocalAxisDirection,
            out Vector3d secondAxisDirection);
        firstAxisDirection = firstAxisDirection.Normalized;
        secondAxisDirection = secondAxisDirection.Normalized;

        return WideFiniteAxisIntersection.TryGetCenteredCapsulesContact(
            firstCenter,
            firstRotation,
            firstLocalAxisDirection,
            firstAxisDirection,
            firstAxisLength,
            firstRadius,
            secondCenter,
            secondRotation,
            secondLocalAxisDirection,
            secondAxisDirection,
            secondAxisLength,
            secondRadius,
            fallbackNormal,
            out contact);
    }

    private static void ValidateNormalizedRotation(
        FixedQuaternion rotation,
        string parameterName)
    {
        if (!rotation.IsNormalized())
        {
            throw new ArgumentException(
                "Capsule rotation must be normalized.",
                parameterName);
        }
    }

    private static void ValidateCenteredCapsulesContact(
        Vector3d firstAxisDirection,
        Fixed64 firstAxisLength,
        Fixed64 firstRadius,
        Vector3d secondAxisDirection,
        Fixed64 secondAxisLength,
        Fixed64 secondRadius,
        Vector3d fallbackNormal)
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
        if (firstRadius < Fixed64.Zero)
            throw new ArgumentOutOfRangeException(nameof(firstRadius));
        if (secondRadius < Fixed64.Zero)
            throw new ArgumentOutOfRangeException(nameof(secondRadius));
        if (!fallbackNormal.IsNormalized())
            throw new ArgumentException("Fallback normal must be normalized.", nameof(fallbackNormal));
    }

    private static void ValidateCenteredAxis(
        Vector3d axisDirection,
        Fixed64 axisLength) =>
        ValidateCenteredAxis(
            axisDirection,
            axisLength,
            nameof(axisDirection),
            nameof(axisLength));

    private static void ValidateCenteredAxis(
        Vector3d axisDirection,
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
