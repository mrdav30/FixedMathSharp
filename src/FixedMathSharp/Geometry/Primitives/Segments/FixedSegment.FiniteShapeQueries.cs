//=======================================================================
// FixedSegment.FiniteShapeQueries.cs
//=======================================================================
// MIT License, Copyright (c) 2024-present David Oravsky (mrdav30)
// See LICENSE file in the project root for full license information.
//=======================================================================

using System;

namespace FixedMathSharp.Geometry;

/// <content>
/// Contains methods for computing the oriented penetration of a centered 3D capsule, finite cylinder, 
/// or finite cone and a centered planar capsule extruded through a world-Y slab.
/// </content>
public partial struct FixedSegment
{
    /// <summary>
    /// Attempts to return the oriented penetration of a centered 3D capsule
    /// and a centered planar capsule extruded through a world-Y slab.
    /// </summary>
    /// <param name="projectionAxis">The normalized world-space direction onto which both shapes are projected.</param>
    /// <param name="capsuleCenter">The capsule center in the query coordinate space.</param>
    /// <param name="capsuleAxisDirection">The normalized capsule axis in world coordinates.</param>
    /// <param name="capsuleAxisLength">The full nonnegative length of the capsule's central segment.</param>
    /// <param name="capsuleRadius">The capsule's nonnegative radius in coordinate units.</param>
    /// <param name="slabCenter">
    /// The slab center in world coordinates. X and Z locate its footprint and Y locates its midplane.
    /// </param>
    /// <param name="slabCapsuleAxisDirection">
    /// The normalized central-axis direction of the planar capsule in world X/Z coordinates.
    /// </param>
    /// <param name="slabCapsuleAxisLength">The full nonnegative length of the footprint's core segment.</param>
    /// <param name="slabCapsuleRadius">The nonnegative radial extent of the planar capsule.</param>
    /// <param name="slabHalfThickness">The half-thickness along world Y.</param>
    /// <param name="orientedAxis">The projection direction oriented for the selected exit on success.</param>
    /// <param name="depth">The rounded penetration depth in coordinate units on success.</param>
    /// <param name="depthIsClamped"><see langword="true"/> if the selected depth exceeds <see cref="Fixed64.MaxValue"/>.</param>
    /// <returns>
    /// <see langword="true"/> when the closed projected intervals overlap, including touching and
    /// <see langword="false"/> otherwise.
    /// </returns>
    /// <remarks>This test uses one projection axis. Use a shape contact or intersection query for a complete relation.</remarks>
    /// <exception cref="ArgumentException">
    /// <paramref name="projectionAxis"/> is not normalized, <paramref name="capsuleAxisDirection"/> is not
    /// normalized, or <paramref name="slabCapsuleAxisDirection"/> is not normalized.
    /// </exception>
    /// <exception cref="ArgumentOutOfRangeException">
    /// <paramref name="capsuleAxisLength"/> is negative, <paramref name="capsuleRadius"/> is negative,
    /// <paramref name="slabCapsuleAxisLength"/> is negative, <paramref name="slabCapsuleRadius"/> is negative, or
    /// <paramref name="slabHalfThickness"/> is negative.
    /// </exception>
    public static bool TryGetCenteredCapsuleCapsuleSlabAxisPenetration(
        Vector3d projectionAxis,
        Vector3d capsuleCenter,
        Vector3d capsuleAxisDirection,
        Fixed64 capsuleAxisLength,
        Fixed64 capsuleRadius,
        Vector3d slabCenter,
        Vector2d slabCapsuleAxisDirection,
        Fixed64 slabCapsuleAxisLength,
        Fixed64 slabCapsuleRadius,
        Fixed64 slabHalfThickness,
        out Vector3d orientedAxis,
        out Fixed64 depth,
        out bool depthIsClamped)
    {
        ValidateCapsuleSlabProjection(
            projectionAxis,
            capsuleAxisDirection,
            capsuleAxisLength,
            capsuleRadius,
            requirePositiveShapeLength: false,
            slabCapsuleAxisDirection,
            slabCapsuleAxisLength,
            slabCapsuleRadius,
            slabHalfThickness);
        return WideFiniteAxisProjection
            .TryGetCapsuleCapsuleSlabAxisPenetration(
                projectionAxis,
                capsuleCenter,
                capsuleAxisDirection,
                capsuleAxisLength,
                capsuleRadius,
                slabCenter,
                slabCapsuleAxisDirection,
                slabCapsuleAxisLength,
                slabCapsuleRadius,
                slabHalfThickness,
                out orientedAxis,
                out depth,
                out depthIsClamped);
    }

    /// <summary>
    /// Attempts to return the oriented penetration of a centered finite
    /// cylinder and a centered planar capsule extruded through a world-Y slab.
    /// </summary>
    /// <param name="projectionAxis">The normalized world-space direction onto which both shapes are projected.</param>
    /// <param name="cylinderCenter">The cylinder center in the query coordinate space.</param>
    /// <param name="cylinderAxisDirection">The normalized cylinder axis in world coordinates.</param>
    /// <param name="cylinderAxisLength">The full positive distance between the cap centers.</param>
    /// <param name="cylinderRadius">The cylinder's nonnegative radius in coordinate units.</param>
    /// <param name="slabCenter">
    /// The slab center in world coordinates. X and Z locate its footprint and Y locates its midplane.
    /// </param>
    /// <param name="slabCapsuleAxisDirection">
    /// The normalized central-axis direction of the planar capsule in world X/Z coordinates.
    /// </param>
    /// <param name="slabCapsuleAxisLength">The full nonnegative length of the footprint's core segment.</param>
    /// <param name="slabCapsuleRadius">The nonnegative radial extent of the planar capsule.</param>
    /// <param name="slabHalfThickness">The half-thickness along world Y.</param>
    /// <param name="orientedAxis">The projection direction oriented for the selected exit on success.</param>
    /// <param name="depth">The rounded penetration depth in coordinate units on success.</param>
    /// <param name="depthIsClamped"><see langword="true"/> if the selected depth exceeds <see cref="Fixed64.MaxValue"/>.</param>
    /// <returns>
    /// <see langword="true"/> when the closed projected intervals overlap, including touching and
    /// <see langword="false"/> otherwise.
    /// </returns>
    /// <remarks>This test uses one projection axis. Use a shape contact or intersection query for a complete relation.</remarks>
    /// <exception cref="ArgumentException">
    /// <paramref name="projectionAxis"/> is not normalized, <paramref name="cylinderAxisDirection"/> is not
    /// normalized, or <paramref name="slabCapsuleAxisDirection"/> is not normalized.
    /// </exception>
    /// <exception cref="ArgumentOutOfRangeException">
    /// <paramref name="cylinderAxisLength"/> is not positive, <paramref name="cylinderRadius"/> is negative,
    /// <paramref name="slabCapsuleAxisLength"/> is negative, <paramref name="slabCapsuleRadius"/> is negative, or
    /// <paramref name="slabHalfThickness"/> is negative.
    /// </exception>
    public static bool TryGetCenteredFiniteCylinderCapsuleSlabAxisPenetration(
        Vector3d projectionAxis,
        Vector3d cylinderCenter,
        Vector3d cylinderAxisDirection,
        Fixed64 cylinderAxisLength,
        Fixed64 cylinderRadius,
        Vector3d slabCenter,
        Vector2d slabCapsuleAxisDirection,
        Fixed64 slabCapsuleAxisLength,
        Fixed64 slabCapsuleRadius,
        Fixed64 slabHalfThickness,
        out Vector3d orientedAxis,
        out Fixed64 depth,
        out bool depthIsClamped)
    {
        ValidateCapsuleSlabProjection(
            projectionAxis,
            cylinderAxisDirection,
            cylinderAxisLength,
            cylinderRadius,
            requirePositiveShapeLength: true,
            slabCapsuleAxisDirection,
            slabCapsuleAxisLength,
            slabCapsuleRadius,
            slabHalfThickness);
        return WideFiniteAxisProjection
            .TryGetCylinderCapsuleSlabAxisPenetration(
                projectionAxis,
                cylinderCenter,
                cylinderAxisDirection,
                cylinderAxisLength,
                cylinderRadius,
                slabCenter,
                slabCapsuleAxisDirection,
                slabCapsuleAxisLength,
                slabCapsuleRadius,
                slabHalfThickness,
                out orientedAxis,
                out depth,
                out depthIsClamped);
    }

    /// <summary>
    /// Attempts to return the oriented penetration of a centered finite cone
    /// and a centered planar capsule extruded through a world-Y slab.
    /// </summary>
    /// <param name="projectionAxis">The normalized world-space direction onto which both shapes are projected.</param>
    /// <param name="coneCenter">The cone center in the query coordinate space.</param>
    /// <param name="baseToApexDirection">The normalized world direction from the base center to the apex.</param>
    /// <param name="coneHeight">The cone's full positive axis length.</param>
    /// <param name="coneRadius">The cone's nonnegative radius in coordinate units.</param>
    /// <param name="slabCenter">
    /// The slab center in world coordinates. X and Z locate its footprint and Y locates its midplane.
    /// </param>
    /// <param name="slabCapsuleAxisDirection">
    /// The normalized central-axis direction of the planar capsule in world X/Z coordinates.
    /// </param>
    /// <param name="slabCapsuleAxisLength">The full nonnegative length of the footprint's core segment.</param>
    /// <param name="slabCapsuleRadius">The nonnegative radial extent of the planar capsule.</param>
    /// <param name="slabHalfThickness">The half-thickness along world Y.</param>
    /// <param name="orientedAxis">The projection direction oriented for the selected exit on success.</param>
    /// <param name="depth">The rounded penetration depth in coordinate units on success.</param>
    /// <param name="depthIsClamped"><see langword="true"/> if the selected depth exceeds <see cref="Fixed64.MaxValue"/>.</param>
    /// <returns>
    /// <see langword="true"/> when the closed projected intervals overlap, including touching and
    /// <see langword="false"/> otherwise.
    /// </returns>
    /// <remarks>This test uses one projection axis. Use a shape contact or intersection query for a complete relation.</remarks>
    /// <exception cref="ArgumentException">
    /// <paramref name="projectionAxis"/> is not normalized, <paramref name="baseToApexDirection"/> is not
    /// normalized, or <paramref name="slabCapsuleAxisDirection"/> is not normalized.
    /// </exception>
    /// <exception cref="ArgumentOutOfRangeException">
    /// <paramref name="coneHeight"/> is not positive, <paramref name="coneRadius"/> is negative,
    /// <paramref name="slabCapsuleAxisLength"/> is negative, <paramref name="slabCapsuleRadius"/> is negative, or
    /// <paramref name="slabHalfThickness"/> is negative.
    /// </exception>
    public static bool TryGetCenteredFiniteConeCapsuleSlabAxisPenetration(
        Vector3d projectionAxis,
        Vector3d coneCenter,
        Vector3d baseToApexDirection,
        Fixed64 coneHeight,
        Fixed64 coneRadius,
        Vector3d slabCenter,
        Vector2d slabCapsuleAxisDirection,
        Fixed64 slabCapsuleAxisLength,
        Fixed64 slabCapsuleRadius,
        Fixed64 slabHalfThickness,
        out Vector3d orientedAxis,
        out Fixed64 depth,
        out bool depthIsClamped)
    {
        ValidateCapsuleSlabProjection(
            projectionAxis,
            baseToApexDirection,
            coneHeight,
            coneRadius,
            requirePositiveShapeLength: true,
            slabCapsuleAxisDirection,
            slabCapsuleAxisLength,
            slabCapsuleRadius,
            slabHalfThickness);
        return WideFiniteAxisProjection
            .TryGetConeCapsuleSlabAxisPenetration(
                projectionAxis,
                coneCenter,
                baseToApexDirection,
                coneHeight,
                coneRadius,
                slabCenter,
                slabCapsuleAxisDirection,
                slabCapsuleAxisLength,
                slabCapsuleRadius,
                slabHalfThickness,
                out orientedAxis,
                out depth,
                out depthIsClamped);
    }

    private static void ValidateCapsuleSlabProjection(
        Vector3d projectionAxis,
        Vector3d shapeAxisDirection,
        Fixed64 shapeAxisLength,
        Fixed64 shapeRadius,
        bool requirePositiveShapeLength,
        Vector2d slabCapsuleAxisDirection,
        Fixed64 slabCapsuleAxisLength,
        Fixed64 slabCapsuleRadius,
        Fixed64 slabHalfThickness)
    {
        if (!projectionAxis.IsNormalized())
            throw new ArgumentException("Projection axis must be normalized.", nameof(projectionAxis));
        if (!shapeAxisDirection.IsNormalized())
            throw new ArgumentException("Shape axis direction must be normalized.", nameof(shapeAxisDirection));
        if (requirePositiveShapeLength
            ? shapeAxisLength <= Fixed64.Zero
            : shapeAxisLength < Fixed64.Zero)
        {
            throw new ArgumentOutOfRangeException(nameof(shapeAxisLength));
        }
        if (shapeRadius < Fixed64.Zero)
            throw new ArgumentOutOfRangeException(nameof(shapeRadius));
        if (!slabCapsuleAxisDirection.IsNormalized())
        {
            throw new ArgumentException(
                "Slab capsule axis direction must be normalized.",
                nameof(slabCapsuleAxisDirection));
        }
        if (slabCapsuleAxisLength < Fixed64.Zero)
            throw new ArgumentOutOfRangeException(nameof(slabCapsuleAxisLength));
        if (slabCapsuleRadius < Fixed64.Zero)
            throw new ArgumentOutOfRangeException(nameof(slabCapsuleRadius));
        if (slabHalfThickness <= Fixed64.Zero)
            throw new ArgumentOutOfRangeException(nameof(slabHalfThickness));
    }

    /// <summary>
    /// Finds the closed parameter interval where this segment intersects a
    /// finite cone authored from its apex toward the center of its flat base.
    /// </summary>
    /// <param name="apex">The cone apex.</param>
    /// <param name="apexToBaseDirection">The normalized direction from apex to base.</param>
    /// <param name="height">
    /// The positive parametric axis extent; the conceptual base center is
    /// <c>apex + apexToBaseDirection * height</c>.
    /// </param>
    /// <param name="baseRadius">The nonnegative base radius.</param>
    /// <param name="entryParameter">The first intersecting segment parameter in [0, 1].</param>
    /// <param name="exitParameter">The last intersecting segment parameter in [0, 1].</param>
    /// <returns>
    /// <see langword="true"/> when the path intersects the closed shape within the tested interval and
    /// <see langword="false"/> otherwise.
    /// </returns>
    /// <exception cref="ArgumentException">
    /// <paramref name="apexToBaseDirection"/> is zero or not normalized.
    /// </exception>
    /// <exception cref="ArgumentOutOfRangeException">
    /// <paramref name="height"/> is not positive or <paramref name="baseRadius"/> is negative.
    /// </exception>
    public readonly bool TryGetFiniteConeIntersectionInterval(
        Vector3d apex,
        Vector3d apexToBaseDirection,
        Fixed64 height,
        Fixed64 baseRadius,
        out Fixed64 entryParameter,
        out Fixed64 exitParameter) =>
        TryGetFiniteConeIntersectionInterval(
            apex,
            apexToBaseDirection,
            height,
            baseRadius,
            out entryParameter,
            out exitParameter,
            out _,
            out _);

    /// <summary>
    /// Finds the closed parameter interval where this segment intersects a
    /// finite apex-authored cone and reports exact endpoint containment.
    /// </summary>
    /// <param name="apex">The cone apex in world coordinates.</param>
    /// <param name="apexToBaseDirection">The normalized world direction from the apex to the base center.</param>
    /// <param name="height">The full positive distance from the cone apex to its base center.</param>
    /// <param name="baseRadius">The nonnegative radius of the base disk.</param>
    /// <param name="entryParameter">
    /// The first parameter in the closed intersection interval, from zero at the start to one at the end on
    /// success.
    /// </param>
    /// <param name="exitParameter">
    /// The last parameter in the closed intersection interval, from zero at the start to one at the end on success.
    /// </param>
    /// <param name="startContained">
    /// <see langword="true"/> when the exact start lies inside or on the expanded shape boundary.
    /// </param>
    /// <param name="endContainedStrict"><see langword="true"/> when the exact end lies strictly inside the expanded shape.</param>
    /// <returns>
    /// <see langword="true"/> when the path intersects the closed shape within the tested interval and
    /// <see langword="false"/> otherwise.
    /// </returns>
    /// <exception cref="ArgumentException"><paramref name="apexToBaseDirection"/> is not normalized.</exception>
    /// <exception cref="ArgumentOutOfRangeException">
    /// <paramref name="height"/> is not positive or <paramref name="baseRadius"/> is negative.
    /// </exception>
    /// <remarks>
    /// <paramref name="startContained"/> includes the side, apex, and flat base.
    /// <paramref name="endContainedStrict"/> excludes every boundary. The
    /// classifications and quadratic solve remain wide until the final
    /// deterministic parameter conversion.
    /// </remarks>
    public readonly bool TryGetFiniteConeIntersectionInterval(
        Vector3d apex,
        Vector3d apexToBaseDirection,
        Fixed64 height,
        Fixed64 baseRadius,
        out Fixed64 entryParameter,
        out Fixed64 exitParameter,
        out bool startContained,
        out bool endContainedStrict)
    {
        ValidateFiniteCone(
            apexToBaseDirection,
            height,
            baseRadius,
            nameof(apexToBaseDirection),
            nameof(height));
        return WideFiniteConeIntersection.TryGetApexInterval(
            this,
            apex,
            apexToBaseDirection,
            height,
            baseRadius,
            out entryParameter,
            out exitParameter,
            out startContained,
            out endContainedStrict);
    }

    /// <summary>
    /// Finds the closed parameter interval where this segment intersects a
    /// centered finite cone.
    /// </summary>
    /// <param name="center">The midpoint between the apex and flat-base center.</param>
    /// <param name="baseToApexDirection">The normalized direction from base to apex.</param>
    /// <param name="height">
    /// The positive full parametric axis extent between base and apex.
    /// </param>
    /// <param name="baseRadius">The nonnegative base radius.</param>
    /// <param name="entryParameter">The first intersecting segment parameter in [0, 1].</param>
    /// <param name="exitParameter">The last intersecting segment parameter in [0, 1].</param>
    /// <returns>
    /// <see langword="true"/> when the path intersects the closed shape within the tested interval and
    /// <see langword="false"/> otherwise.
    /// </returns>
    /// <exception cref="ArgumentException">
    /// <paramref name="baseToApexDirection"/> is zero or not normalized.
    /// </exception>
    /// <exception cref="ArgumentOutOfRangeException">
    /// <paramref name="height"/> is not positive or <paramref name="baseRadius"/> is negative.
    /// </exception>
    public readonly bool TryGetCenteredFiniteConeIntersectionInterval(
        Vector3d center,
        Vector3d baseToApexDirection,
        Fixed64 height,
        Fixed64 baseRadius,
        out Fixed64 entryParameter,
        out Fixed64 exitParameter) =>
        TryGetCenteredFiniteConeIntersectionInterval(
            center,
            baseToApexDirection,
            height,
            baseRadius,
            out entryParameter,
            out exitParameter,
            out _,
            out _);

    /// <summary>
    /// Finds the closed parameter interval where this segment intersects a
    /// centered finite cone and reports exact endpoint containment.
    /// </summary>
    /// <param name="center">The center in the query coordinate space.</param>
    /// <param name="baseToApexDirection">The normalized world direction from the base center to the apex.</param>
    /// <param name="height">The full positive distance from the cone apex to its base center.</param>
    /// <param name="baseRadius">The nonnegative radius of the base disk.</param>
    /// <param name="entryParameter">
    /// The first parameter in the closed intersection interval, from zero at the start to one at the end on
    /// success.
    /// </param>
    /// <param name="exitParameter">
    /// The last parameter in the closed intersection interval, from zero at the start to one at the end on success.
    /// </param>
    /// <param name="startContained">
    /// <see langword="true"/> when the exact start lies inside or on the expanded shape boundary.
    /// </param>
    /// <param name="endContainedStrict"><see langword="true"/> when the exact end lies strictly inside the expanded shape.</param>
    /// <returns>
    /// <see langword="true"/> when the path intersects the closed shape within the tested interval and
    /// <see langword="false"/> otherwise.
    /// </returns>
    /// <exception cref="ArgumentException"><paramref name="baseToApexDirection"/> is not normalized.</exception>
    /// <exception cref="ArgumentOutOfRangeException">
    /// <paramref name="height"/> is not positive or <paramref name="baseRadius"/> is negative.
    /// </exception>
    public readonly bool TryGetCenteredFiniteConeIntersectionInterval(
        Vector3d center,
        Vector3d baseToApexDirection,
        Fixed64 height,
        Fixed64 baseRadius,
        out Fixed64 entryParameter,
        out Fixed64 exitParameter,
        out bool startContained,
        out bool endContainedStrict)
    {
        ValidateFiniteCone(
            baseToApexDirection,
            height,
            baseRadius,
            nameof(baseToApexDirection),
            nameof(height));
        return WideFiniteConeIntersection.TryGetCenteredInterval(
            this,
            center,
            baseToApexDirection,
            height,
            baseRadius,
            out entryParameter,
            out exitParameter,
            out startContained,
            out endContainedStrict);
    }

    /// <summary>
    /// Finds the physical-distance interval where this segment intersects a
    /// finite cone authored from its apex toward its flat base.
    /// </summary>
    /// <param name="apex">The cone apex in world coordinates.</param>
    /// <param name="apexToBaseDirection">The normalized world direction from the apex to the base center.</param>
    /// <param name="height">The full positive distance from the cone apex to its base center.</param>
    /// <param name="baseRadius">The nonnegative radius of the base disk.</param>
    /// <param name="totalDistance">
    /// The caller-supplied distance mapped to the full segment. Zero is allowed only for coincident endpoints.
    /// </param>
    /// <param name="entryDistance">The first distance along the authored segment interval on success.</param>
    /// <param name="exitDistance">The last distance along the authored segment interval on success.</param>
    /// <returns>
    /// <see langword="true"/> when the path intersects the closed shape within the tested interval and
    /// <see langword="false"/> otherwise.
    /// </returns>
    /// <remarks>The exact chord parameter is mapped to the authored distance interval before the endpoints are rounded.</remarks>
    /// <exception cref="ArgumentException">
    /// <paramref name="apexToBaseDirection"/> is not normalized or <paramref name="totalDistance"/> is zero for a
    /// nonzero-length segment.
    /// </exception>
    /// <exception cref="ArgumentOutOfRangeException">
    /// <paramref name="height"/> is not positive, <paramref name="baseRadius"/> is negative, or
    /// <paramref name="totalDistance"/> is negative.
    /// </exception>
    /// <remarks>
    /// Distances are solved directly from the wide polynomial; they are not
    /// reconstructed from rounded Q32.32 parameters.
    /// </remarks>
    public readonly bool TryGetFiniteConeIntersectionDistanceInterval(
        Vector3d apex,
        Vector3d apexToBaseDirection,
        Fixed64 height,
        Fixed64 baseRadius,
        Fixed64 totalDistance,
        out Fixed64 entryDistance,
        out Fixed64 exitDistance) =>
        TryGetFiniteConeIntersectionDistanceInterval(
            apex,
            apexToBaseDirection,
            height,
            baseRadius,
            totalDistance,
            out entryDistance,
            out exitDistance,
            out _,
            out _);

    /// <summary>
    /// Finds the physical-distance interval where this segment intersects an
    /// apex-authored finite cone and reports exact endpoint containment.
    /// </summary>
    /// <param name="apex">The cone apex in world coordinates.</param>
    /// <param name="apexToBaseDirection">The normalized world direction from the apex to the base center.</param>
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
    /// <paramref name="apexToBaseDirection"/> is not normalized or <paramref name="totalDistance"/> is zero for a
    /// nonzero-length segment.
    /// </exception>
    /// <exception cref="ArgumentOutOfRangeException">
    /// <paramref name="height"/> is not positive, <paramref name="baseRadius"/> is negative, or
    /// <paramref name="totalDistance"/> is negative.
    /// </exception>
    public readonly bool TryGetFiniteConeIntersectionDistanceInterval(
        Vector3d apex,
        Vector3d apexToBaseDirection,
        Fixed64 height,
        Fixed64 baseRadius,
        Fixed64 totalDistance,
        out Fixed64 entryDistance,
        out Fixed64 exitDistance,
        out bool startContained,
        out bool endContainedStrict)
    {
        ValidateTotalDistance(totalDistance);
        ValidateFiniteCone(
            apexToBaseDirection,
            height,
            baseRadius,
            nameof(apexToBaseDirection),
            nameof(height));
        return WideFiniteConeIntersection.TryGetApexDistanceInterval(
            this,
            apex,
            apexToBaseDirection,
            height,
            baseRadius,
            totalDistance,
            out entryDistance,
            out exitDistance,
            out startContained,
            out endContainedStrict);
    }

    /// <summary>
    /// Finds the physical-distance interval where this segment intersects a
    /// centered finite cone.
    /// </summary>
    /// <param name="center">The center in the query coordinate space.</param>
    /// <param name="baseToApexDirection">The normalized world direction from the base center to the apex.</param>
    /// <param name="height">The full positive distance from the cone apex to its base center.</param>
    /// <param name="baseRadius">The nonnegative radius of the base disk.</param>
    /// <param name="totalDistance">
    /// The caller-supplied distance mapped to the full segment. Zero is allowed only for coincident endpoints.
    /// </param>
    /// <param name="entryDistance">The first distance along the authored segment interval on success.</param>
    /// <param name="exitDistance">The last distance along the authored segment interval on success.</param>
    /// <returns>
    /// <see langword="true"/> when the path intersects the closed shape within the tested interval and
    /// <see langword="false"/> otherwise.
    /// </returns>
    /// <remarks>The exact chord parameter is mapped to the authored distance interval before the endpoints are rounded.</remarks>
    /// <exception cref="ArgumentException">
    /// <paramref name="baseToApexDirection"/> is not normalized or <paramref name="totalDistance"/> is zero for a
    /// nonzero-length segment.
    /// </exception>
    /// <exception cref="ArgumentOutOfRangeException">
    /// <paramref name="height"/> is not positive, <paramref name="baseRadius"/> is negative, or
    /// <paramref name="totalDistance"/> is negative.
    /// </exception>
    public readonly bool TryGetCenteredFiniteConeIntersectionDistanceInterval(
        Vector3d center,
        Vector3d baseToApexDirection,
        Fixed64 height,
        Fixed64 baseRadius,
        Fixed64 totalDistance,
        out Fixed64 entryDistance,
        out Fixed64 exitDistance) =>
        TryGetCenteredFiniteConeIntersectionDistanceInterval(
            center,
            baseToApexDirection,
            height,
            baseRadius,
            totalDistance,
            out entryDistance,
            out exitDistance,
            out _,
            out _);

    /// <summary>
    /// Finds the physical-distance interval where this segment intersects a
    /// centered finite cone and reports exact endpoint containment.
    /// </summary>
    /// <param name="center">The center in the query coordinate space.</param>
    /// <param name="baseToApexDirection">The normalized world direction from the base center to the apex.</param>
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
    /// <paramref name="baseToApexDirection"/> is not normalized or <paramref name="totalDistance"/> is zero for a
    /// nonzero-length segment.
    /// </exception>
    /// <exception cref="ArgumentOutOfRangeException">
    /// <paramref name="height"/> is not positive, <paramref name="baseRadius"/> is negative, or
    /// <paramref name="totalDistance"/> is negative.
    /// </exception>
    public readonly bool TryGetCenteredFiniteConeIntersectionDistanceInterval(
        Vector3d center,
        Vector3d baseToApexDirection,
        Fixed64 height,
        Fixed64 baseRadius,
        Fixed64 totalDistance,
        out Fixed64 entryDistance,
        out Fixed64 exitDistance,
        out bool startContained,
        out bool endContainedStrict)
    {
        ValidateTotalDistance(totalDistance);
        ValidateFiniteCone(
            baseToApexDirection,
            height,
            baseRadius,
            nameof(baseToApexDirection),
            nameof(height));
        return WideFiniteConeIntersection.TryGetCenteredDistanceInterval(
            this,
            center,
            baseToApexDirection,
            height,
            baseRadius,
            totalDistance,
            out entryDistance,
            out exitDistance,
            out startContained,
            out endContainedStrict);
    }

    /// <summary>
    /// Finds deterministic lattice witnesses for the first and last points where
    /// this segment intersects an apex-authored finite cone.
    /// </summary>
    /// <param name="apex">The cone apex in world coordinates.</param>
    /// <param name="apexToBaseDirection">The normalized world direction from the apex to the base center.</param>
    /// <param name="height">The full positive distance from the cone apex to its base center.</param>
    /// <param name="baseRadius">The nonnegative radius of the base disk.</param>
    /// <param name="entryPoint">The rounded entry witness in the segment's coordinate space on success.</param>
    /// <param name="exitPoint">The rounded exit witness in the segment's coordinate space on success.</param>
    /// <returns>
    /// <see langword="true"/> when both intersection endpoint witnesses can be materialized and
    /// <see langword="false"/> otherwise.
    /// </returns>
    /// <exception cref="ArgumentException"><paramref name="apexToBaseDirection"/> is not normalized.</exception>
    /// <exception cref="ArgumentOutOfRangeException">
    /// <paramref name="height"/> is not positive or <paramref name="baseRadius"/> is negative.
    /// </exception>
    /// <remarks>
    /// The solver uses the largest positive Q32.32 interval scale before exact
    /// authored-chord interpolation, so long segments do not amplify ordinary
    /// Q32.32 parameter rounding into coarse spatial error.
    /// </remarks>
    public readonly bool TryGetFiniteConeIntersectionPointInterval(
        Vector3d apex,
        Vector3d apexToBaseDirection,
        Fixed64 height,
        Fixed64 baseRadius,
        out Vector3d entryPoint,
        out Vector3d exitPoint)
    {
        ValidateFiniteCone(
            apexToBaseDirection,
            height,
            baseRadius,
            nameof(apexToBaseDirection),
            nameof(height));
        Fixed64 intervalScale = Start == End ? Fixed64.Zero : Fixed64.MaxValue;
        if (!WideFiniteConeIntersection.TryGetApexDistanceInterval(
                this,
                apex,
                apexToBaseDirection,
                height,
                baseRadius,
                intervalScale,
                out Fixed64 entry,
                out Fixed64 exit,
                out _,
                out _))
        {
            entryPoint = default;
            exitPoint = default;
            return false;
        }

        entryPoint = GetPointAtDistance(entry, intervalScale);
        exitPoint = GetPointAtDistance(exit, intervalScale);
        return true;
    }

    /// <summary>
    /// Finds the intersecting lattice point with the smallest axial distance
    /// from an apex-authored cone's apex.
    /// </summary>
    /// <param name="apex">The cone apex in world coordinates.</param>
    /// <param name="apexToBaseDirection">The normalized world direction from the apex to the base center.</param>
    /// <param name="height">The full positive distance from the cone apex to its base center.</param>
    /// <param name="baseRadius">The nonnegative radius of the base disk.</param>
    /// <param name="point">The selected point on the intersection in query coordinates on success.</param>
    /// <returns>
    /// <see langword="true"/> when a minimum-axial intersection witness can be returned and <see langword="false"/>
    /// otherwise.
    /// </returns>
    /// <exception cref="ArgumentException"><paramref name="apexToBaseDirection"/> is not normalized.</exception>
    /// <exception cref="ArgumentOutOfRangeException">
    /// <paramref name="height"/> is not positive or <paramref name="baseRadius"/> is negative.
    /// </exception>
    /// <remarks>
    /// The exact segment-axis projection selects the appropriate end of the
    /// admitted interval before one high-resolution authored-chord interpolation.
    /// </remarks>
    public readonly bool TryGetFiniteConeIntersectionMinimumAxialPoint(
        Vector3d apex,
        Vector3d apexToBaseDirection,
        Fixed64 height,
        Fixed64 baseRadius,
        out Vector3d point)
    {
        ValidateFiniteCone(
            apexToBaseDirection,
            height,
            baseRadius,
            nameof(apexToBaseDirection),
            nameof(height));
        Fixed64 intervalScale = Start == End ? Fixed64.Zero : Fixed64.MaxValue;
        if (!WideFiniteConeIntersection.TryGetApexDistanceInterval(
                this,
                apex,
                apexToBaseDirection,
                height,
                baseRadius,
                intervalScale,
                out Fixed64 entry,
                out Fixed64 exit,
                out _,
                out _))
        {
            point = default;
            return false;
        }

        Signed192 axialVelocity = WideGeometry.GetDifferenceDotProduct3D(
            End.X, Start.X, End.Y, Start.Y, End.Z, Start.Z,
            apexToBaseDirection.X, Fixed64.Zero,
            apexToBaseDirection.Y, Fixed64.Zero,
            apexToBaseDirection.Z, Fixed64.Zero);
        point = GetPointAtDistance(axialVelocity.Sign < 0 ? exit : entry, intervalScale);
        return true;
    }

    /// <summary>
    /// Finds deterministic lattice witnesses for the first and last points where
    /// this segment intersects a centered finite cone.
    /// </summary>
    /// <param name="center">The center in the query coordinate space.</param>
    /// <param name="baseToApexDirection">The normalized world direction from the base center to the apex.</param>
    /// <param name="height">The full positive distance from the cone apex to its base center.</param>
    /// <param name="baseRadius">The nonnegative radius of the base disk.</param>
    /// <param name="entryPoint">The rounded entry witness in the segment's coordinate space on success.</param>
    /// <param name="exitPoint">The rounded exit witness in the segment's coordinate space on success.</param>
    /// <returns>
    /// <see langword="true"/> when both intersection endpoint witnesses can be materialized and
    /// <see langword="false"/> otherwise.
    /// </returns>
    /// <exception cref="ArgumentException"><paramref name="baseToApexDirection"/> is not normalized.</exception>
    /// <exception cref="ArgumentOutOfRangeException">
    /// <paramref name="height"/> is not positive or <paramref name="baseRadius"/> is negative.
    /// </exception>
    public readonly bool TryGetCenteredFiniteConeIntersectionPointInterval(
        Vector3d center,
        Vector3d baseToApexDirection,
        Fixed64 height,
        Fixed64 baseRadius,
        out Vector3d entryPoint,
        out Vector3d exitPoint)
    {
        ValidateFiniteCone(
            baseToApexDirection,
            height,
            baseRadius,
            nameof(baseToApexDirection),
            nameof(height));
        Fixed64 intervalScale = Start == End ? Fixed64.Zero : Fixed64.MaxValue;
        if (!WideFiniteConeIntersection.TryGetCenteredDistanceInterval(
                this,
                center,
                baseToApexDirection,
                height,
                baseRadius,
                intervalScale,
                out Fixed64 entry,
                out Fixed64 exit,
                out _,
                out _))
        {
            entryPoint = default;
            exitPoint = default;
            return false;
        }

        entryPoint = GetPointAtDistance(entry, intervalScale);
        exitPoint = GetPointAtDistance(exit, intervalScale);
        return true;
    }

    /// <summary>
    /// Returns whether a point lies strictly inside or inclusively within a
    /// finite cone authored from its apex toward its flat base.
    /// </summary>
    /// <param name="point">The point in the query coordinate space.</param>
    /// <param name="apex">The cone apex in world coordinates.</param>
    /// <param name="apexToBaseDirection">The normalized world direction from the apex to the base center.</param>
    /// <param name="height">The full positive distance from the cone apex to its base center.</param>
    /// <param name="baseRadius">The nonnegative radius of the base disk.</param>
    /// <param name="strict">
    /// <see langword="true"/> to require strict interior containment. <see langword="false"/> includes the
    /// boundary.
    /// </param>
    /// <returns>
    /// <see langword="true"/> when the point satisfies the requested containment test and <see langword="false"/>
    /// otherwise.
    /// </returns>
    /// <exception cref="ArgumentException"><paramref name="apexToBaseDirection"/> is not normalized.</exception>
    /// <exception cref="ArgumentOutOfRangeException">
    /// <paramref name="height"/> is not positive or <paramref name="baseRadius"/> is negative.
    /// </exception>
    public static bool ContainsPointInFiniteCone(
        Vector3d point,
        Vector3d apex,
        Vector3d apexToBaseDirection,
        Fixed64 height,
        Fixed64 baseRadius,
        bool strict = false)
    {
        ValidateFiniteCone(
            apexToBaseDirection,
            height,
            baseRadius,
            nameof(apexToBaseDirection),
            nameof(height));
        return WideFiniteConeIntersection.ContainsPointInApexCone(
            point,
            apex,
            apexToBaseDirection,
            height,
            baseRadius,
            strict);
    }

    /// <summary>
    /// Returns whether a point lies strictly inside or inclusively within a
    /// centered finite cone whose direction points from base to apex.
    /// </summary>
    /// <param name="point">The point in the query coordinate space.</param>
    /// <param name="center">The center in the query coordinate space.</param>
    /// <param name="baseToApexDirection">The normalized world direction from the base center to the apex.</param>
    /// <param name="height">The full positive distance from the cone apex to its base center.</param>
    /// <param name="baseRadius">The nonnegative radius of the base disk.</param>
    /// <param name="strict">
    /// <see langword="true"/> to require strict interior containment. <see langword="false"/> includes the
    /// boundary.
    /// </param>
    /// <returns>
    /// <see langword="true"/> when the point satisfies the requested containment test and <see langword="false"/>
    /// otherwise.
    /// </returns>
    /// <exception cref="ArgumentException"><paramref name="baseToApexDirection"/> is not normalized.</exception>
    /// <exception cref="ArgumentOutOfRangeException">
    /// <paramref name="height"/> is not positive or <paramref name="baseRadius"/> is negative.
    /// </exception>
    public static bool ContainsPointInCenteredFiniteCone(
        Vector3d point,
        Vector3d center,
        Vector3d baseToApexDirection,
        Fixed64 height,
        Fixed64 baseRadius,
        bool strict = false)
    {
        ValidateFiniteCone(
            baseToApexDirection,
            height,
            baseRadius,
            nameof(baseToApexDirection),
            nameof(height));
        return WideFiniteConeIntersection.ContainsPointInCenteredCone(
            point,
            center,
            baseToApexDirection,
            height,
            baseRadius,
            strict);
    }

    private static void ValidateFiniteCone(
        Vector3d axisDirection,
        Fixed64 height,
        Fixed64 baseRadius,
        string axisParameterName,
        string heightParameterName)
    {
        if (!axisDirection.IsNormalized())
            throw new ArgumentException("Finite cone axis direction must be normalized.", axisParameterName);
        if (height <= Fixed64.Zero)
            throw new ArgumentOutOfRangeException(heightParameterName);
        if (baseRadius < Fixed64.Zero)
            throw new ArgumentOutOfRangeException(nameof(baseRadius));
    }
}
