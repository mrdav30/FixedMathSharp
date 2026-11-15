//=======================================================================
// FixedSegment.CenteredAxis.cs
//=======================================================================
// MIT License, Copyright (c) 2024-present David Oravsky (mrdav30)
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
    /// Returns whether two centered capsules in normalized rigid frames have
    /// strictly positive radial overlap, excluding exact tangency.
    /// </summary>
    /// <param name="firstCenter">The first shape center in the query coordinate space.</param>
    /// <param name="firstRotation">The first shape's normalized local-to-world rotation.</param>
    /// <param name="firstLocalAxisDirection">The normalized first shape axis in shape-local coordinates.</param>
    /// <param name="firstAxisLength">The full nonnegative length of the first shape's central segment.</param>
    /// <param name="firstRadius">The first shape's nonnegative radius in coordinate units.</param>
    /// <param name="secondCenter">The second shape center in the query coordinate space.</param>
    /// <param name="secondRotation">The second shape's normalized local-to-world rotation.</param>
    /// <param name="secondLocalAxisDirection">The normalized second shape axis in shape-local coordinates.</param>
    /// <param name="secondAxisLength">The full nonnegative length of the second shape's central segment.</param>
    /// <param name="secondRadius">The second shape's nonnegative radius in coordinate units.</param>
    /// <returns>
    /// <see langword="true"/> when the exact closest core distance is strictly less than the combined radius and
    /// <see langword="false"/> otherwise.
    /// </returns>
    /// <exception cref="ArgumentException">
    /// <paramref name="firstRotation"/> is not normalized, <paramref name="firstLocalAxisDirection"/> is not
    /// normalized, <paramref name="secondRotation"/> is not normalized, or
    /// <paramref name="secondLocalAxisDirection"/> is not normalized.
    /// </exception>
    /// <exception cref="ArgumentOutOfRangeException">
    /// <paramref name="firstAxisLength"/> is negative, <paramref name="firstRadius"/> is negative,
    /// <paramref name="secondAxisLength"/> is negative, or <paramref name="secondRadius"/> is negative.
    /// </exception>
    /// <remarks>
    /// Each axis is expressed in its capsule's local frame. The exact rational
    /// quaternion rotation remains authoritative; derived rounded world axes,
    /// endpoint coordinates, and contact depths do not affect classification.
    /// Zero-length axes are points; a zero radius sum has no strict overlap.
    /// </remarks>
    public static bool DoCenteredCapsulesOverlapStrict(
        Vector3d firstCenter,
        FixedQuaternion firstRotation,
        Vector3d firstLocalAxisDirection,
        Fixed64 firstAxisLength,
        Fixed64 firstRadius,
        Vector3d secondCenter,
        FixedQuaternion secondRotation,
        Vector3d secondLocalAxisDirection,
        Fixed64 secondAxisLength,
        Fixed64 secondRadius)
    {
        ValidateNormalizedRotation(firstRotation, nameof(firstRotation));
        ValidateNormalizedRotation(secondRotation, nameof(secondRotation));
        ValidateCenteredAxis(firstLocalAxisDirection, firstAxisLength,
            nameof(firstLocalAxisDirection), nameof(firstAxisLength));
        if (firstRadius < Fixed64.Zero)
            throw new ArgumentOutOfRangeException(nameof(firstRadius));
        ValidateCenteredAxis(secondLocalAxisDirection, secondAxisLength,
            nameof(secondLocalAxisDirection), nameof(secondAxisLength));
        if (secondRadius < Fixed64.Zero)
            throw new ArgumentOutOfRangeException(nameof(secondRadius));

        return WideOrientedBox.DoCenteredRigidCapsulesOverlap(
            firstCenter, firstRotation, firstLocalAxisDirection, firstAxisLength, firstRadius,
            secondCenter, secondRotation, secondLocalAxisDirection, secondAxisLength, secondRadius,
            strict: true);
    }

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
        Vector3d firstCenter,
        Vector3d firstAxisDirection,
        Fixed64 firstAxisLength,
        Fixed64 firstRadius,
        Vector3d secondCenter,
        Vector3d secondAxisDirection,
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
    /// <param name="axisDirection">The normalized axis direction in world coordinates.</param>
    /// <param name="axisLength">The full nonnegative length of the central segment. Zero makes the core a point.</param>
    /// <param name="radius">The nonnegative radius in coordinate units.</param>
    /// <param name="direction">The world-space direction used to select support. Its magnitude is used as supplied.</param>
    /// <param name="supportOffset">The selected support point relative to the center, expressed along world axes on success.</param>
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
    /// <param name="center">The center in the query coordinate space.</param>
    /// <param name="rotation">The normalized quaternion from local coordinates to world coordinates.</param>
    /// <param name="axisLength">The full nonnegative length of the central segment. Zero makes the core a point.</param>
    /// <param name="radius">The nonnegative radius in coordinate units.</param>
    /// <param name="direction">The world-space direction used to select support. Its magnitude is used as supplied.</param>
    /// <returns>A support anchor retaining the selected shape-local feature and its world frame.</returns>
    /// <exception cref="ArgumentException"><paramref name="rotation"/> is not normalized.</exception>
    /// <exception cref="ArgumentOutOfRangeException">
    /// <paramref name="axisLength"/> is negative or <paramref name="radius"/> is negative.
    /// </exception>
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
    /// <param name="center">The center in the query coordinate space.</param>
    /// <param name="axisDirection">The normalized axis direction in world coordinates.</param>
    /// <param name="axisLength">The full positive distance between the cap centers.</param>
    /// <param name="radius">The nonnegative radius in coordinate units.</param>
    /// <param name="direction">The world-space direction used to select support. Its magnitude is used as supplied.</param>
    /// <param name="support">The selected support point in world coordinates on success.</param>
    /// <returns>
    /// <see langword="true"/> when the requested rounded result is representable and <see langword="false"/>
    /// otherwise.
    /// </returns>
    /// <exception cref="ArgumentException"><paramref name="axisDirection"/> is not normalized.</exception>
    /// <exception cref="ArgumentOutOfRangeException">
    /// <paramref name="axisLength"/> is not positive or <paramref name="radius"/> is negative.
    /// </exception>
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
    /// <param name="axisDirection">The normalized axis direction in world coordinates.</param>
    /// <param name="axisLength">The full positive distance between the cap centers.</param>
    /// <param name="radius">The nonnegative radius in coordinate units.</param>
    /// <param name="direction">The world-space direction used to select support. Its magnitude is used as supplied.</param>
    /// <param name="supportOffset">The selected support point relative to the center, expressed along world axes on success.</param>
    /// <returns>
    /// <see langword="true"/> when the requested rounded result is representable and <see langword="false"/>
    /// otherwise.
    /// </returns>
    /// <exception cref="ArgumentException"><paramref name="axisDirection"/> is not normalized.</exception>
    /// <exception cref="ArgumentOutOfRangeException">
    /// <paramref name="axisLength"/> is not positive or <paramref name="radius"/> is negative.
    /// </exception>
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
    /// <param name="center">The center in the query coordinate space.</param>
    /// <param name="rotation">The normalized quaternion from local coordinates to world coordinates.</param>
    /// <param name="axisLength">The full positive distance between the cap centers.</param>
    /// <param name="radius">The nonnegative radius in coordinate units.</param>
    /// <param name="direction">The world-space direction used to select support. Its magnitude is used as supplied.</param>
    /// <returns>A support anchor retaining the selected shape-local feature and its world frame.</returns>
    /// <exception cref="ArgumentException"><paramref name="rotation"/> is not normalized.</exception>
    /// <exception cref="ArgumentOutOfRangeException">
    /// <paramref name="axisLength"/> is not positive or <paramref name="radius"/> is negative.
    /// </exception>
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
    /// <param name="center">The center in the query coordinate space.</param>
    /// <param name="axisDirection">The normalized axis direction in world coordinates.</param>
    /// <param name="height">The full positive distance from the cone apex to its base center.</param>
    /// <param name="radius">The nonnegative radius in coordinate units.</param>
    /// <param name="direction">The world-space direction used to select support. Its magnitude is used as supplied.</param>
    /// <param name="support">The selected support point in world coordinates on success.</param>
    /// <returns>
    /// <see langword="true"/> when the requested rounded result is representable and <see langword="false"/>
    /// otherwise.
    /// </returns>
    /// <exception cref="ArgumentException"><paramref name="axisDirection"/> is not normalized.</exception>
    /// <exception cref="ArgumentOutOfRangeException">
    /// <paramref name="height"/> is not positive or <paramref name="radius"/> is negative.
    /// </exception>
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
    /// <param name="axisDirection">The normalized axis direction in world coordinates.</param>
    /// <param name="height">The full positive distance from the cone apex to its base center.</param>
    /// <param name="radius">The nonnegative radius in coordinate units.</param>
    /// <param name="direction">The world-space direction used to select support. Its magnitude is used as supplied.</param>
    /// <param name="supportOffset">The selected support point relative to the center, expressed along world axes on success.</param>
    /// <returns>
    /// <see langword="true"/> when the requested rounded result is representable and <see langword="false"/>
    /// otherwise.
    /// </returns>
    /// <exception cref="ArgumentException"><paramref name="axisDirection"/> is not normalized.</exception>
    /// <exception cref="ArgumentOutOfRangeException">
    /// <paramref name="height"/> is not positive or <paramref name="radius"/> is negative.
    /// </exception>
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
    /// <param name="center">The center in the query coordinate space.</param>
    /// <param name="rotation">The normalized quaternion from local coordinates to world coordinates.</param>
    /// <param name="height">The full positive distance from the cone apex to its base center.</param>
    /// <param name="radius">The nonnegative radius in coordinate units.</param>
    /// <param name="direction">The world-space direction used to select support. Its magnitude is used as supplied.</param>
    /// <returns>A support anchor retaining the selected shape-local feature and its world frame.</returns>
    /// <exception cref="ArgumentException"><paramref name="rotation"/> is not normalized.</exception>
    /// <exception cref="ArgumentOutOfRangeException">
    /// <paramref name="height"/> is not positive or <paramref name="radius"/> is negative.
    /// </exception>
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
    /// <param name="center">The center in the query coordinate space.</param>
    /// <param name="axisDirection">The normalized axis direction in world coordinates.</param>
    /// <param name="height">The full positive distance from the cone apex to its base center.</param>
    /// <param name="radius">The nonnegative radius in coordinate units.</param>
    /// <param name="direction">The world-space direction used to select support. Its magnitude is used as supplied.</param>
    /// <returns>A support anchor retaining the selected shape-local feature and its world frame.</returns>
    /// <exception cref="ArgumentException"><paramref name="axisDirection"/> is not normalized.</exception>
    /// <exception cref="ArgumentOutOfRangeException">
    /// <paramref name="height"/> is not positive or <paramref name="radius"/> is negative.
    /// </exception>
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
    /// <returns>
    /// <see langword="true"/> when the point satisfies the requested containment test and <see langword="false"/>
    /// otherwise.
    /// </returns>
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
    /// <returns>
    /// <see langword="true"/> when the point satisfies the requested containment test and <see langword="false"/>
    /// otherwise.
    /// </returns>
    /// <exception cref="ArgumentException"><paramref name="axisDirection"/> is not normalized.</exception>
    /// <exception cref="ArgumentOutOfRangeException">
    /// <paramref name="axisLength"/> is not positive, <paramref name="radius"/> is negative,
    /// <paramref name="axialTolerance"/> is negative, or <paramref name="radialTolerance"/> is negative.
    /// </exception>
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
    /// <param name="firstCenter">The first shape center in the query coordinate space.</param>
    /// <param name="firstAxisDirection">The normalized first shape axis in world coordinates.</param>
    /// <param name="firstAxisLength">The full nonnegative length of the first shape's central segment.</param>
    /// <param name="secondCenter">The second shape center in the query coordinate space.</param>
    /// <param name="secondAxisDirection">The normalized second shape axis in world coordinates.</param>
    /// <param name="secondAxisLength">The full nonnegative length of the second shape's central segment.</param>
    /// <param name="firstCenterOffset">The closest point on the first axis, relative to its center on success.</param>
    /// <param name="secondCenterOffset">The closest point on the second axis, relative to its center on success.</param>
    /// <returns>
    /// <see langword="true"/> when both selected witnesses can be represented and <see langword="false"/>
    /// otherwise.
    /// </returns>
    /// <exception cref="ArgumentException">
    /// <paramref name="firstAxisDirection"/> is not normalized or <paramref name="secondAxisDirection"/> is not
    /// normalized.
    /// </exception>
    /// <exception cref="ArgumentOutOfRangeException">
    /// <paramref name="firstAxisLength"/> is negative or <paramref name="secondAxisLength"/> is negative.
    /// </exception>
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
    /// <param name="firstCenter">The first shape center in the query coordinate space.</param>
    /// <param name="firstAxisDirection">The normalized first shape axis in world coordinates.</param>
    /// <param name="firstAxisLength">The full nonnegative length of the first shape's central segment.</param>
    /// <param name="secondCenter">The second shape center in the query coordinate space.</param>
    /// <param name="secondAxisDirection">The normalized second shape axis in world coordinates.</param>
    /// <param name="secondAxisLength">The full nonnegative length of the second shape's central segment.</param>
    /// <returns>
    /// The normalized direction from the closest point on the first axis toward its counterpart on the second, or
    /// zero for coincident witnesses.
    /// </returns>
    /// <exception cref="ArgumentException">
    /// <paramref name="firstAxisDirection"/> is not normalized or <paramref name="secondAxisDirection"/> is not
    /// normalized.
    /// </exception>
    /// <exception cref="ArgumentOutOfRangeException">
    /// <paramref name="firstAxisLength"/> is negative or <paramref name="secondAxisLength"/> is negative.
    /// </exception>
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
    /// <param name="firstCenter">The first shape center in the query coordinate space.</param>
    /// <param name="firstRotation">The first shape's normalized local-to-world rotation.</param>
    /// <param name="firstLocalAxisDirection">The normalized first shape axis in shape-local coordinates.</param>
    /// <param name="firstAxisLength">The full nonnegative length of the first shape's central segment.</param>
    /// <param name="firstRadius">The first shape's nonnegative radius in coordinate units.</param>
    /// <param name="secondCenter">The second shape center in the query coordinate space.</param>
    /// <param name="secondRotation">The second shape's normalized local-to-world rotation.</param>
    /// <param name="secondLocalAxisDirection">The normalized second shape axis in shape-local coordinates.</param>
    /// <param name="secondAxisLength">The full nonnegative length of the second shape's central segment.</param>
    /// <param name="secondRadius">The second shape's nonnegative radius in coordinate units.</param>
    /// <param name="fallbackNormal">The normalized world-space normal used when the closest features coincide.</param>
    /// <param name="contact">
    /// The paired shape-local witnesses, world normal, and depth on success. Use this result only when the method
    /// returns <see langword="true"/>.
    /// </param>
    /// <exception cref="ArgumentException">
    /// <paramref name="firstRotation"/> is not normalized, <paramref name="firstLocalAxisDirection"/> is not
    /// normalized, <paramref name="secondRotation"/> is not normalized, <paramref name="secondLocalAxisDirection"/>
    /// is not normalized, or <paramref name="fallbackNormal"/> is not normalized.
    /// </exception>
    /// <exception cref="ArgumentOutOfRangeException">
    /// <paramref name="firstAxisLength"/> is negative, <paramref name="firstRadius"/> is negative,
    /// <paramref name="secondAxisLength"/> is negative, or <paramref name="secondRadius"/> is negative.
    /// </exception>
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
        Vector3d firstAxisDirection = firstLocalAxisDirection;
        Vector3d secondAxisDirection = secondLocalAxisDirection;
        // Zero-core axes are irrelevant to the candidate; rotations still own
        // the radial anchors. Keep the exact rigid classifier above for all pairs.
        if (firstAxisLength != Fixed64.Zero || secondAxisLength != Fixed64.Zero)
        {
            _ = firstRotation.TryRotate(firstLocalAxisDirection, out firstAxisDirection);
            _ = secondRotation.TryRotate(secondLocalAxisDirection, out secondAxisDirection);
            firstAxisDirection = firstAxisDirection.Normalized;
            secondAxisDirection = secondAxisDirection.Normalized;
        }

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
