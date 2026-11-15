//=======================================================================
// FixedSegment.RigidQueries.cs
//=======================================================================
// MIT License, Copyright (c) 2024-present David Oravsky (mrdav30)
// See LICENSE file in the project root for full license information.
//=======================================================================

using System;

namespace FixedMathSharp.Geometry;

/// <content>
/// Contains intersection and contact queries for rigidly oriented finite
/// capsules, cylinders, and cones.
/// </content>
public partial struct FixedSegment
{
    /// <summary>
    /// Finds the physical-distance interval where this segment intersects a
    /// centered capsule whose local positive Y axis is transformed by a rigid
    /// quaternion frame.
    /// </summary>
    /// <param name="center">The center in the query coordinate space.</param>
    /// <param name="rotation">The normalized quaternion from local coordinates to world coordinates.</param>
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
    /// <exception cref="ArgumentException">
    /// <paramref name="rotation"/> is not normalized or <paramref name="totalDistance"/> is zero for a
    /// nonzero-length segment.
    /// </exception>
    /// <exception cref="ArgumentOutOfRangeException">
    /// <paramref name="axisLength"/> is negative, <paramref name="radius"/> is negative,
    /// <paramref name="radiusExpansion"/> is negative, or <paramref name="totalDistance"/> is negative.
    /// </exception>
    public readonly bool TryGetCapsuleIntersectionDistanceInterval(
        Vector3d center,
        FixedQuaternion rotation,
        Fixed64 axisLength,
        Fixed64 radius,
        Fixed64 radiusExpansion,
        Fixed64 totalDistance,
        out Fixed64 entryDistance,
        out Fixed64 exitDistance,
        out bool startContained,
        out bool endContainedStrict)
    {
        ValidateRigidFiniteAxisQuery(
            rotation,
            axisLength,
            radius,
            radiusExpansion,
            totalDistance,
            requirePositiveLength: false);
        return WideOrientedBox.TryGetRigidCapsuleDistanceInterval(
            this,
            center,
            rotation,
            axisLength,
            radius,
            radiusExpansion,
            totalDistance,
            out entryDistance,
            out exitDistance,
            out startContained,
            out endContainedStrict);
    }

    /// <summary>
    /// Finds the physical-distance interval where this segment intersects a
    /// centered finite cylinder whose local positive Y axis is transformed by
    /// a rigid quaternion frame.
    /// </summary>
    /// <param name="center">The center in the query coordinate space.</param>
    /// <param name="rotation">The normalized quaternion from local coordinates to world coordinates.</param>
    /// <param name="axisLength">The full positive distance between the cap centers.</param>
    /// <param name="radius">The nonnegative radius in coordinate units.</param>
    /// <param name="radiusExpansion">A nonnegative addition to the radius in coordinate units.</param>
    /// <param name="axialExpansion">A nonnegative addition to the distance from the axis center to each cap.</param>
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
    /// <exception cref="ArgumentException">
    /// <paramref name="rotation"/> is not normalized or <paramref name="totalDistance"/> is zero for a
    /// nonzero-length segment.
    /// </exception>
    /// <exception cref="ArgumentOutOfRangeException">
    /// <paramref name="axisLength"/> is not positive, <paramref name="radius"/> is negative,
    /// <paramref name="radiusExpansion"/> is negative, <paramref name="axialExpansion"/> is negative, or
    /// <paramref name="totalDistance"/> is negative.
    /// </exception>
    public readonly bool TryGetFiniteCylinderIntersectionDistanceInterval(
        Vector3d center,
        FixedQuaternion rotation,
        Fixed64 axisLength,
        Fixed64 radius,
        Fixed64 radiusExpansion,
        Fixed64 axialExpansion,
        Fixed64 totalDistance,
        out Fixed64 entryDistance,
        out Fixed64 exitDistance,
        out bool startContained,
        out bool endContainedStrict)
    {
        ValidateRigidFiniteAxisQuery(
            rotation,
            axisLength,
            radius,
            radiusExpansion,
            totalDistance,
            requirePositiveLength: true);
        if (axialExpansion < Fixed64.Zero)
            throw new ArgumentOutOfRangeException(nameof(axialExpansion));

        return WideOrientedBox.TryGetRigidCylinderDistanceInterval(
            this,
            center,
            rotation,
            axisLength,
            radius,
            radiusExpansion,
            axialExpansion,
            totalDistance,
            out entryDistance,
            out exitDistance,
            out startContained,
            out endContainedStrict);
    }

    /// <summary>
    /// Finds the physical-distance interval where this segment intersects a
    /// centered finite cone whose local positive Y axis runs from base to apex
    /// in a rigid quaternion frame.
    /// </summary>
    /// <param name="center">The center in the query coordinate space.</param>
    /// <param name="rotation">The normalized quaternion from local coordinates to world coordinates.</param>
    /// <param name="height">The full positive distance from the cone apex to its base center.</param>
    /// <param name="baseRadius">The nonnegative radius of the base disk.</param>
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
    /// <exception cref="ArgumentException">
    /// <paramref name="rotation"/> is not normalized or <paramref name="totalDistance"/> is zero for a
    /// nonzero-length segment.
    /// </exception>
    /// <exception cref="ArgumentOutOfRangeException">
    /// <paramref name="height"/> is not positive, <paramref name="baseRadius"/> is negative, or
    /// <paramref name="totalDistance"/> is negative.
    /// </exception>
    public readonly bool TryGetCenteredFiniteConeIntersectionDistanceInterval(
        Vector3d center,
        FixedQuaternion rotation,
        Fixed64 height,
        Fixed64 baseRadius,
        Fixed64 totalDistance,
        out Fixed64 entryDistance,
        out Fixed64 exitDistance,
        out bool startContained,
        out bool endContainedStrict)
    {
        ValidateRigidFiniteAxisQuery(
            rotation,
            height,
            baseRadius,
            Fixed64.Zero,
            totalDistance,
            requirePositiveLength: true);
        return WideOrientedBox.TryGetRigidConeDistanceInterval(
            this,
            center,
            rotation,
            height,
            baseRadius,
            totalDistance,
            out entryDistance,
            out exitDistance,
            out startContained,
            out endContainedStrict);
    }

    private readonly void ValidateRigidFiniteAxisQuery(
        FixedQuaternion rotation,
        Fixed64 axisLength,
        Fixed64 radius,
        Fixed64 radiusExpansion,
        Fixed64 totalDistance,
        bool requirePositiveLength)
    {
        ValidateTotalDistance(totalDistance);
        if (!rotation.IsNormalized())
        {
            throw new ArgumentException(
                "Finite-axis rotation must be normalized.",
                nameof(rotation));
        }
        if (requirePositiveLength
            ? axisLength <= Fixed64.Zero
            : axisLength < Fixed64.Zero)
        {
            throw new ArgumentOutOfRangeException(nameof(axisLength));
        }
        if (radius < Fixed64.Zero)
            throw new ArgumentOutOfRangeException(nameof(radius));
        if (radiusExpansion < Fixed64.Zero)
            throw new ArgumentOutOfRangeException(nameof(radiusExpansion));
    }

    /// <summary>
    /// Attempts to construct the minimum-translation contact between two
    /// centered finite cylinders.
    /// </summary>
    /// <param name="firstCenter">The first shape center in the query coordinate space.</param>
    /// <param name="firstRotation">The first shape's normalized local-to-world rotation.</param>
    /// <param name="firstLocalAxisDirection">The normalized first shape axis in shape-local coordinates.</param>
    /// <param name="firstAxisLength">The full positive distance between the cap centers.</param>
    /// <param name="firstRadius">The first shape's nonnegative radius in coordinate units.</param>
    /// <param name="secondCenter">The second shape center in the query coordinate space.</param>
    /// <param name="secondRotation">The second shape's normalized local-to-world rotation.</param>
    /// <param name="secondLocalAxisDirection">The normalized second shape axis in shape-local coordinates.</param>
    /// <param name="secondAxisLength">The full positive distance between the cap centers.</param>
    /// <param name="secondRadius">The second shape's nonnegative radius in coordinate units.</param>
    /// <param name="contact">
    /// The paired shape-local witnesses, world normal, and depth on success. Use this result only when the method
    /// returns <see langword="true"/>.
    /// </param>
    /// <returns>
    /// <see langword="true"/> when the closed shapes touch or overlap and the requested contact is available and
    /// <see langword="false"/> otherwise.
    /// </returns>
    /// <exception cref="ArgumentException">
    /// <paramref name="firstRotation"/> is not normalized, <paramref name="firstLocalAxisDirection"/> is not
    /// normalized, <paramref name="secondRotation"/> is not normalized, or
    /// <paramref name="secondLocalAxisDirection"/> is not normalized.
    /// </exception>
    /// <exception cref="ArgumentOutOfRangeException">
    /// <paramref name="firstAxisLength"/> is not positive, <paramref name="firstRadius"/> is negative,
    /// <paramref name="secondAxisLength"/> is not positive, or <paramref name="secondRadius"/> is negative.
    /// </exception>
    /// <remarks>
    /// Both finite axes remain authoritative in their normalized rigid frames.
    /// The returned normal points from the first cylinder toward the second.
    /// Zero-radius cylinders are finite segments. Oblique contacts, including
    /// positive-radius cylinder pairs, can require hundreds of KiB of bounded
    /// stack scratch without managed allocation. Allow at least a 1 MiB thread
    /// stack with adequate caller headroom; avoid deliberately small-stack workers.
    /// </remarks>
    public static bool TryGetCenteredFiniteCylindersContact(
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
        out FixedContactAnchors contact)
    {
        ValidateRigidFiniteShape(
            firstRotation,
            firstLocalAxisDirection,
            firstAxisLength,
            firstRadius,
            requirePositiveLength: true,
            nameof(firstRotation),
            nameof(firstLocalAxisDirection),
            nameof(firstAxisLength),
            nameof(firstRadius));
        ValidateRigidFiniteShape(
            secondRotation,
            secondLocalAxisDirection,
            secondAxisLength,
            secondRadius,
            requirePositiveLength: true,
            nameof(secondRotation),
            nameof(secondLocalAxisDirection),
            nameof(secondAxisLength),
            nameof(secondRadius));
        return WideConvexPrismRelations
            .TryGetCenteredFiniteCylindersContact(
                firstCenter,
                firstRotation,
                firstLocalAxisDirection,
                firstAxisLength,
                firstRadius,
                secondCenter,
                secondRotation,
                secondLocalAxisDirection,
                secondAxisLength,
                secondRadius,
                out contact);
    }

    /// <summary>
    /// Attempts to construct the minimum-translation contact between a
    /// centered finite cylinder and a centered capsule.
    /// </summary>
    /// <param name="cylinderCenter">The cylinder center in the query coordinate space.</param>
    /// <param name="cylinderRotation">The cylinder's normalized local-to-world rotation.</param>
    /// <param name="cylinderLocalAxisDirection">The normalized cylinder axis in shape-local coordinates.</param>
    /// <param name="cylinderAxisLength">The full positive distance between the cap centers.</param>
    /// <param name="cylinderRadius">The cylinder's nonnegative radius in coordinate units.</param>
    /// <param name="capsuleCenter">The capsule center in the query coordinate space.</param>
    /// <param name="capsuleRotation">The capsule's normalized local-to-world rotation.</param>
    /// <param name="capsuleLocalAxisDirection">The normalized capsule axis in shape-local coordinates.</param>
    /// <param name="capsuleAxisLength">The full nonnegative length of the capsule's central segment.</param>
    /// <param name="capsuleRadius">The capsule's nonnegative radius in coordinate units.</param>
    /// <param name="contact">
    /// The paired shape-local witnesses, world normal, and depth on success. Use this result only when the method
    /// returns <see langword="true"/>.
    /// </param>
    /// <returns>
    /// <see langword="true"/> when the closed shapes touch or overlap and the requested contact is available and
    /// <see langword="false"/> otherwise.
    /// </returns>
    /// <exception cref="ArgumentException">
    /// <paramref name="cylinderRotation"/> is not normalized, <paramref name="cylinderLocalAxisDirection"/> is not
    /// normalized, <paramref name="capsuleRotation"/> is not normalized, or
    /// <paramref name="capsuleLocalAxisDirection"/> is not normalized.
    /// </exception>
    /// <exception cref="ArgumentOutOfRangeException">
    /// <paramref name="cylinderAxisLength"/> is not positive, <paramref name="cylinderRadius"/> is negative,
    /// <paramref name="capsuleAxisLength"/> is negative, or <paramref name="capsuleRadius"/> is negative.
    /// </exception>
    /// <remarks>
    /// Both finite axes remain authoritative in their normalized rigid frames.
    /// The returned normal points from the cylinder toward the capsule.
    /// Separation, exact tangency, and minimum depth are decided before output
    /// rounding, including rim features and intersecting capsule cores. Depth
    /// rounds once to nearest-even; DepthIsClamped describes conceptual overflow.
    /// Exact ties retain stable feature order. Each anchor is an independent
    /// support point for the rounded normal, not a paired algebraic closest point.
    /// The exact curved-feature fallback uses bounded stack scratch storage;
    /// avoid deliberately small-stack worker threads for these queries.
    /// </remarks>
    public static bool TryGetCenteredFiniteCylinderCapsuleContact(
        Vector3d cylinderCenter,
        FixedQuaternion cylinderRotation,
        Vector3d cylinderLocalAxisDirection,
        Fixed64 cylinderAxisLength,
        Fixed64 cylinderRadius,
        Vector3d capsuleCenter,
        FixedQuaternion capsuleRotation,
        Vector3d capsuleLocalAxisDirection,
        Fixed64 capsuleAxisLength,
        Fixed64 capsuleRadius,
        out FixedContactAnchors contact)
    {
        ValidateRigidFiniteShape(
            cylinderRotation,
            cylinderLocalAxisDirection,
            cylinderAxisLength,
            cylinderRadius,
            requirePositiveLength: true,
            nameof(cylinderRotation),
            nameof(cylinderLocalAxisDirection),
            nameof(cylinderAxisLength),
            nameof(cylinderRadius));
        ValidateRigidFiniteShape(
            capsuleRotation,
            capsuleLocalAxisDirection,
            capsuleAxisLength,
            capsuleRadius,
            requirePositiveLength: false,
            nameof(capsuleRotation),
            nameof(capsuleLocalAxisDirection),
            nameof(capsuleAxisLength),
            nameof(capsuleRadius));
        return WideConvexPrismRelations
            .TryGetCenteredFiniteCylinderCapsuleContact(
                cylinderCenter,
                cylinderRotation,
                cylinderLocalAxisDirection,
                cylinderAxisLength,
                cylinderRadius,
                capsuleCenter,
                capsuleRotation,
                capsuleLocalAxisDirection,
                capsuleAxisLength,
                capsuleRadius,
                out contact);
    }

    private static void ValidateRigidFiniteShape(
        FixedQuaternion rotation,
        Vector3d localAxisDirection,
        Fixed64 axisLength,
        Fixed64 radius,
        bool requirePositiveLength,
        string rotationParameterName,
        string axisParameterName,
        string lengthParameterName,
        string radiusParameterName)
    {
        if (!rotation.IsNormalized())
            throw new ArgumentException("The rigid rotation must be normalized.", rotationParameterName);
        if (!localAxisDirection.IsNormalized())
            throw new ArgumentException("The local axis direction must be normalized.", axisParameterName);
        if (requirePositiveLength
            ? axisLength <= Fixed64.Zero
            : axisLength < Fixed64.Zero)
        {
            throw new ArgumentOutOfRangeException(lengthParameterName);
        }
        if (radius < Fixed64.Zero)
            throw new ArgumentOutOfRangeException(radiusParameterName);
    }
}
