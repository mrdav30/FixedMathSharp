//=======================================================================
// FixedSegment.CenteredSurfaces.cs
//=======================================================================
// MIT License, Copyright (c) 2024-present David Oravsky (mrdav30)
// See LICENSE file in the project root for full license information.
//=======================================================================

using System;

namespace FixedMathSharp.Geometry;

/// <content>
/// Contains methods for computing the nearest surface feature of a 
/// centered 3D capsule, finite cylinder, or finite cone in its authoritative rigid frame.
/// </content>
public partial struct FixedSegment
{
    /// <summary>
    /// Returns whether a centered finite cylinder and centered capsule have
    /// inclusive projections on a normalized separating axis.
    /// </summary>
    /// <param name="projectionAxis">The normalized world-space direction onto which both shapes are projected.</param>
    /// <param name="cylinderCenter">The cylinder center in the query coordinate space.</param>
    /// <param name="cylinderAxisDirection">The normalized cylinder axis in world coordinates.</param>
    /// <param name="cylinderAxisLength">The full positive distance between the cap centers.</param>
    /// <param name="cylinderRadius">The cylinder's nonnegative radius in coordinate units.</param>
    /// <param name="capsuleCenter">The capsule center in the query coordinate space.</param>
    /// <param name="capsuleAxisDirection">The normalized capsule axis in world coordinates.</param>
    /// <param name="capsuleAxisLength">The full nonnegative length of the capsule's central segment.</param>
    /// <param name="capsuleRadius">The capsule's nonnegative radius in coordinate units.</param>
    /// <returns>
    /// <see langword="true"/> when the closed projected intervals overlap, including touching and
    /// <see langword="false"/> otherwise.
    /// </returns>
    /// <exception cref="ArgumentException">
    /// <paramref name="projectionAxis"/> is not normalized, <paramref name="cylinderAxisDirection"/> is not
    /// normalized, or <paramref name="capsuleAxisDirection"/> is not normalized.
    /// </exception>
    /// <exception cref="ArgumentOutOfRangeException">
    /// <paramref name="cylinderAxisLength"/> is not positive, <paramref name="cylinderRadius"/> is negative,
    /// <paramref name="capsuleAxisLength"/> is negative, or <paramref name="capsuleRadius"/> is negative.
    /// </exception>
    public static bool DoCenteredFiniteCylinderAndCapsuleOverlapOnAxis(
        Vector3d projectionAxis,
        Vector3d cylinderCenter,
        Vector3d cylinderAxisDirection,
        Fixed64 cylinderAxisLength,
        Fixed64 cylinderRadius,
        Vector3d capsuleCenter,
        Vector3d capsuleAxisDirection,
        Fixed64 capsuleAxisLength,
        Fixed64 capsuleRadius)
    {
        ValidateCylinderCapsuleProjection(
            projectionAxis,
            cylinderAxisDirection,
            cylinderAxisLength,
            cylinderRadius,
            capsuleAxisDirection,
            capsuleAxisLength,
            capsuleRadius);
        return WideFiniteAxisProjection.DoCylinderCapsuleOverlapOnAxis(
            projectionAxis,
            cylinderCenter,
            cylinderAxisDirection,
            cylinderAxisLength,
            cylinderRadius,
            capsuleCenter,
            capsuleAxisDirection,
            capsuleAxisLength,
            capsuleRadius);
    }

    /// <summary>
    /// Attempts to return the oriented minimum translation and rounded depth
    /// for a centered finite-cylinder/capsule projection overlap.
    /// </summary>
    /// <param name="projectionAxis">The normalized world-space direction onto which both shapes are projected.</param>
    /// <param name="cylinderCenter">The cylinder center in the query coordinate space.</param>
    /// <param name="cylinderAxisDirection">The normalized cylinder axis in world coordinates.</param>
    /// <param name="cylinderAxisLength">The full positive distance between the cap centers.</param>
    /// <param name="cylinderRadius">The cylinder's nonnegative radius in coordinate units.</param>
    /// <param name="capsuleCenter">The capsule center in the query coordinate space.</param>
    /// <param name="capsuleAxisDirection">The normalized capsule axis in world coordinates.</param>
    /// <param name="capsuleAxisLength">The full nonnegative length of the capsule's central segment.</param>
    /// <param name="capsuleRadius">The capsule's nonnegative radius in coordinate units.</param>
    /// <param name="orientedAxis">The projection direction oriented for the selected exit on success.</param>
    /// <param name="depth">The rounded penetration depth in coordinate units on success.</param>
    /// <returns>
    /// <see langword="true"/> when the closed projected intervals overlap and the selected depth is representable
    /// and <see langword="false"/> otherwise.
    /// </returns>
    /// <remarks>This test uses one projection axis. Use a shape contact or intersection query for a complete relation.</remarks>
    /// <exception cref="ArgumentException">
    /// <paramref name="projectionAxis"/> is not normalized, <paramref name="cylinderAxisDirection"/> is not
    /// normalized, or <paramref name="capsuleAxisDirection"/> is not normalized.
    /// </exception>
    /// <exception cref="ArgumentOutOfRangeException">
    /// <paramref name="cylinderAxisLength"/> is not positive, <paramref name="cylinderRadius"/> is negative,
    /// <paramref name="capsuleAxisLength"/> is negative, or <paramref name="capsuleRadius"/> is negative.
    /// </exception>
    /// <remarks>
    /// The returned axis points from the cylinder toward the capsule; an exact
    /// center-projection tie retains <paramref name="projectionAxis"/>.
    /// Classification is independent of final depth representability through
    /// <see cref="DoCenteredFiniteCylinderAndCapsuleOverlapOnAxis"/>.
    /// </remarks>
    public static bool TryGetCenteredFiniteCylinderCapsuleAxisPenetration(
        Vector3d projectionAxis,
        Vector3d cylinderCenter,
        Vector3d cylinderAxisDirection,
        Fixed64 cylinderAxisLength,
        Fixed64 cylinderRadius,
        Vector3d capsuleCenter,
        Vector3d capsuleAxisDirection,
        Fixed64 capsuleAxisLength,
        Fixed64 capsuleRadius,
        out Vector3d orientedAxis,
        out Fixed64 depth)
    {
        ValidateCylinderCapsuleProjection(
            projectionAxis,
            cylinderAxisDirection,
            cylinderAxisLength,
            cylinderRadius,
            capsuleAxisDirection,
            capsuleAxisLength,
            capsuleRadius);
        return WideFiniteAxisProjection.TryGetCylinderCapsuleAxisPenetration(
            projectionAxis,
            cylinderCenter,
            cylinderAxisDirection,
            cylinderAxisLength,
            cylinderRadius,
            capsuleCenter,
            capsuleAxisDirection,
            capsuleAxisLength,
            capsuleRadius,
            out orientedAxis,
            out depth);
    }

    /// <summary>
    /// Attempts to return the oriented minimum translation for a centered
    /// finite-cylinder/capsule projection overlap, clamping an otherwise
    /// unrepresentable positive depth to <see cref="Fixed64.MaxValue"/>.
    /// </summary>
    /// <param name="projectionAxis">The normalized world-space direction onto which both shapes are projected.</param>
    /// <param name="cylinderCenter">The cylinder center in the query coordinate space.</param>
    /// <param name="cylinderAxisDirection">The normalized cylinder axis in world coordinates.</param>
    /// <param name="cylinderAxisLength">The full positive distance between the cap centers.</param>
    /// <param name="cylinderRadius">The cylinder's nonnegative radius in coordinate units.</param>
    /// <param name="capsuleCenter">The capsule center in the query coordinate space.</param>
    /// <param name="capsuleAxisDirection">The normalized capsule axis in world coordinates.</param>
    /// <param name="capsuleAxisLength">The full nonnegative length of the capsule's central segment.</param>
    /// <param name="capsuleRadius">The capsule's nonnegative radius in coordinate units.</param>
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
    /// normalized, or <paramref name="capsuleAxisDirection"/> is not normalized.
    /// </exception>
    /// <exception cref="ArgumentOutOfRangeException">
    /// <paramref name="cylinderAxisLength"/> is not positive, <paramref name="cylinderRadius"/> is negative,
    /// <paramref name="capsuleAxisLength"/> is negative, or <paramref name="capsuleRadius"/> is negative.
    /// </exception>
    public static bool TryGetCenteredFiniteCylinderCapsuleAxisPenetration(
        Vector3d projectionAxis,
        Vector3d cylinderCenter,
        Vector3d cylinderAxisDirection,
        Fixed64 cylinderAxisLength,
        Fixed64 cylinderRadius,
        Vector3d capsuleCenter,
        Vector3d capsuleAxisDirection,
        Fixed64 capsuleAxisLength,
        Fixed64 capsuleRadius,
        out Vector3d orientedAxis,
        out Fixed64 depth,
        out bool depthIsClamped)
    {
        ValidateCylinderCapsuleProjection(
            projectionAxis,
            cylinderAxisDirection,
            cylinderAxisLength,
            cylinderRadius,
            capsuleAxisDirection,
            capsuleAxisLength,
            capsuleRadius);
        return WideFiniteAxisProjection.TryGetCylinderCapsuleAxisPenetration(
            projectionAxis,
            cylinderCenter,
            cylinderAxisDirection,
            cylinderAxisLength,
            cylinderRadius,
            capsuleCenter,
            capsuleAxisDirection,
            capsuleAxisLength,
            capsuleRadius,
            out orientedAxis,
            out depth,
            out depthIsClamped);
    }

    /// <summary>
    /// Returns whether two centered finite cylinders have inclusive
    /// projections on a normalized separating axis.
    /// </summary>
    /// <param name="projectionAxis">The normalized world-space direction onto which both shapes are projected.</param>
    /// <param name="firstCenter">The first shape center in the query coordinate space.</param>
    /// <param name="firstAxisDirection">The normalized first shape axis in world coordinates.</param>
    /// <param name="firstAxisLength">The full positive distance between the cap centers.</param>
    /// <param name="firstRadius">The first shape's nonnegative radius in coordinate units.</param>
    /// <param name="secondCenter">The second shape center in the query coordinate space.</param>
    /// <param name="secondAxisDirection">The normalized second shape axis in world coordinates.</param>
    /// <param name="secondAxisLength">The full positive distance between the cap centers.</param>
    /// <param name="secondRadius">The second shape's nonnegative radius in coordinate units.</param>
    /// <returns>
    /// <see langword="true"/> when the closed projected intervals overlap, including touching and
    /// <see langword="false"/> otherwise.
    /// </returns>
    /// <exception cref="ArgumentException">
    /// <paramref name="projectionAxis"/> is not normalized, <paramref name="firstAxisDirection"/> is not
    /// normalized, or <paramref name="secondAxisDirection"/> is not normalized.
    /// </exception>
    /// <exception cref="ArgumentOutOfRangeException">
    /// <paramref name="firstAxisLength"/> is not positive, <paramref name="firstRadius"/> is negative,
    /// <paramref name="secondAxisLength"/> is not positive, or <paramref name="secondRadius"/> is negative.
    /// </exception>
    public static bool DoCenteredFiniteCylindersOverlapOnAxis(
        Vector3d projectionAxis,
        Vector3d firstCenter,
        Vector3d firstAxisDirection,
        Fixed64 firstAxisLength,
        Fixed64 firstRadius,
        Vector3d secondCenter,
        Vector3d secondAxisDirection,
        Fixed64 secondAxisLength,
        Fixed64 secondRadius)
    {
        ValidateCylinderProjection(
            projectionAxis,
            firstAxisDirection,
            firstAxisLength,
            firstRadius,
            nameof(firstAxisDirection),
            nameof(firstAxisLength),
            nameof(firstRadius));
        ValidateCylinderProjection(
            projectionAxis,
            secondAxisDirection,
            secondAxisLength,
            secondRadius,
            nameof(secondAxisDirection),
            nameof(secondAxisLength),
            nameof(secondRadius));
        return WideFiniteAxisProjection.DoCylindersOverlapOnAxis(
            projectionAxis,
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
    /// Attempts to return the oriented minimum translation and rounded depth
    /// for two centered finite-cylinder projections.
    /// </summary>
    /// <param name="projectionAxis">The normalized world-space direction onto which both shapes are projected.</param>
    /// <param name="firstCenter">The first shape center in the query coordinate space.</param>
    /// <param name="firstAxisDirection">The normalized first shape axis in world coordinates.</param>
    /// <param name="firstAxisLength">The full positive distance between the cap centers.</param>
    /// <param name="firstRadius">The first shape's nonnegative radius in coordinate units.</param>
    /// <param name="secondCenter">The second shape center in the query coordinate space.</param>
    /// <param name="secondAxisDirection">The normalized second shape axis in world coordinates.</param>
    /// <param name="secondAxisLength">The full positive distance between the cap centers.</param>
    /// <param name="secondRadius">The second shape's nonnegative radius in coordinate units.</param>
    /// <param name="orientedAxis">The projection direction oriented for the selected exit on success.</param>
    /// <param name="depth">The rounded penetration depth in coordinate units on success.</param>
    /// <returns>
    /// <see langword="true"/> when the closed projected intervals overlap and the selected depth is representable
    /// and <see langword="false"/> otherwise.
    /// </returns>
    /// <remarks>This test uses one projection axis. Use a shape contact or intersection query for a complete relation.</remarks>
    /// <exception cref="ArgumentException">
    /// <paramref name="projectionAxis"/> is not normalized, <paramref name="firstAxisDirection"/> is not
    /// normalized, or <paramref name="secondAxisDirection"/> is not normalized.
    /// </exception>
    /// <exception cref="ArgumentOutOfRangeException">
    /// <paramref name="firstAxisLength"/> is not positive, <paramref name="firstRadius"/> is negative,
    /// <paramref name="secondAxisLength"/> is not positive, or <paramref name="secondRadius"/> is negative.
    /// </exception>
    public static bool TryGetCenteredFiniteCylindersAxisPenetration(
        Vector3d projectionAxis,
        Vector3d firstCenter,
        Vector3d firstAxisDirection,
        Fixed64 firstAxisLength,
        Fixed64 firstRadius,
        Vector3d secondCenter,
        Vector3d secondAxisDirection,
        Fixed64 secondAxisLength,
        Fixed64 secondRadius,
        out Vector3d orientedAxis,
        out Fixed64 depth)
    {
        ValidateCylinderProjection(
            projectionAxis,
            firstAxisDirection,
            firstAxisLength,
            firstRadius,
            nameof(firstAxisDirection),
            nameof(firstAxisLength),
            nameof(firstRadius));
        ValidateCylinderProjection(
            projectionAxis,
            secondAxisDirection,
            secondAxisLength,
            secondRadius,
            nameof(secondAxisDirection),
            nameof(secondAxisLength),
            nameof(secondRadius));
        return WideFiniteAxisProjection.TryGetCylindersAxisPenetration(
            projectionAxis,
            firstCenter,
            firstAxisDirection,
            firstAxisLength,
            firstRadius,
            secondCenter,
            secondAxisDirection,
            secondAxisLength,
            secondRadius,
            out orientedAxis,
            out depth);
    }

    /// <summary>
    /// Attempts to return the oriented minimum translation for two centered
    /// finite-cylinder projections, clamping an otherwise unrepresentable
    /// positive depth to <see cref="Fixed64.MaxValue"/>.
    /// </summary>
    /// <param name="projectionAxis">The normalized world-space direction onto which both shapes are projected.</param>
    /// <param name="firstCenter">The first shape center in the query coordinate space.</param>
    /// <param name="firstAxisDirection">The normalized first shape axis in world coordinates.</param>
    /// <param name="firstAxisLength">The full positive distance between the cap centers.</param>
    /// <param name="firstRadius">The first shape's nonnegative radius in coordinate units.</param>
    /// <param name="secondCenter">The second shape center in the query coordinate space.</param>
    /// <param name="secondAxisDirection">The normalized second shape axis in world coordinates.</param>
    /// <param name="secondAxisLength">The full positive distance between the cap centers.</param>
    /// <param name="secondRadius">The second shape's nonnegative radius in coordinate units.</param>
    /// <param name="orientedAxis">The projection direction oriented for the selected exit on success.</param>
    /// <param name="depth">The rounded penetration depth in coordinate units on success.</param>
    /// <param name="depthIsClamped"><see langword="true"/> if the selected depth exceeds <see cref="Fixed64.MaxValue"/>.</param>
    /// <returns>
    /// <see langword="true"/> when the closed projected intervals overlap, including touching and
    /// <see langword="false"/> otherwise.
    /// </returns>
    /// <remarks>This test uses one projection axis. Use a shape contact or intersection query for a complete relation.</remarks>
    /// <exception cref="ArgumentException">
    /// <paramref name="projectionAxis"/> is not normalized, <paramref name="firstAxisDirection"/> is not
    /// normalized, or <paramref name="secondAxisDirection"/> is not normalized.
    /// </exception>
    /// <exception cref="ArgumentOutOfRangeException">
    /// <paramref name="firstAxisLength"/> is not positive, <paramref name="firstRadius"/> is negative,
    /// <paramref name="secondAxisLength"/> is not positive, or <paramref name="secondRadius"/> is negative.
    /// </exception>
    public static bool TryGetCenteredFiniteCylindersAxisPenetration(
        Vector3d projectionAxis,
        Vector3d firstCenter,
        Vector3d firstAxisDirection,
        Fixed64 firstAxisLength,
        Fixed64 firstRadius,
        Vector3d secondCenter,
        Vector3d secondAxisDirection,
        Fixed64 secondAxisLength,
        Fixed64 secondRadius,
        out Vector3d orientedAxis,
        out Fixed64 depth,
        out bool depthIsClamped)
    {
        ValidateCylinderProjection(
            projectionAxis,
            firstAxisDirection,
            firstAxisLength,
            firstRadius,
            nameof(firstAxisDirection),
            nameof(firstAxisLength),
            nameof(firstRadius));
        ValidateCylinderProjection(
            projectionAxis,
            secondAxisDirection,
            secondAxisLength,
            secondRadius,
            nameof(secondAxisDirection),
            nameof(secondAxisLength),
            nameof(secondRadius));
        return WideFiniteAxisProjection.TryGetCylindersAxisPenetration(
            projectionAxis,
            firstCenter,
            firstAxisDirection,
            firstAxisLength,
            firstRadius,
            secondCenter,
            secondAxisDirection,
            secondAxisLength,
            secondRadius,
            out orientedAxis,
            out depth,
            out depthIsClamped);
    }

    /// <summary>
    /// Returns whether a conceptual centered finite cylinder inclusively
    /// overlaps a sphere without materializing a contact witness.
    /// </summary>
    /// <param name="cylinderCenter">The cylinder center in the query coordinate space.</param>
    /// <param name="cylinderAxisDirection">The normalized cylinder axis in world coordinates.</param>
    /// <param name="cylinderAxisLength">The full positive distance between the cap centers.</param>
    /// <param name="cylinderRadius">The cylinder's nonnegative radius in coordinate units.</param>
    /// <param name="sphereCenter">The sphere center in the query coordinate space.</param>
    /// <param name="sphereRadius">The sphere's nonnegative radius in coordinate units.</param>
    /// <returns>
    /// <see langword="true"/> when the query finds closed overlap or tangency and <see langword="false"/>
    /// otherwise.
    /// </returns>
    /// <exception cref="ArgumentException"><paramref name="cylinderAxisDirection"/> is not normalized.</exception>
    /// <exception cref="ArgumentOutOfRangeException">
    /// <paramref name="cylinderAxisLength"/> is not positive, <paramref name="cylinderRadius"/> is negative, or
    /// <paramref name="sphereRadius"/> is negative.
    /// </exception>
    public static bool DoesCenteredFiniteCylinderOverlapSphere(
        Vector3d cylinderCenter,
        Vector3d cylinderAxisDirection,
        Fixed64 cylinderAxisLength,
        Fixed64 cylinderRadius,
        Vector3d sphereCenter,
        Fixed64 sphereRadius)
    {
        if (!cylinderAxisDirection.IsNormalized())
        {
            throw new ArgumentException(
                "Cylinder axis direction must be normalized.",
                nameof(cylinderAxisDirection));
        }
        if (cylinderAxisLength <= Fixed64.Zero)
            throw new ArgumentOutOfRangeException(nameof(cylinderAxisLength));
        if (cylinderRadius < Fixed64.Zero)
            throw new ArgumentOutOfRangeException(nameof(cylinderRadius));
        if (sphereRadius < Fixed64.Zero)
            throw new ArgumentOutOfRangeException(nameof(sphereRadius));

        return WideFiniteAxisIntersection.DoesCenteredFiniteCylinderOverlapSphere(
            cylinderCenter,
            cylinderAxisDirection,
            cylinderAxisLength,
            cylinderRadius,
            sphereCenter,
            sphereRadius);
    }

    /// <summary>
    /// Attempts to return the nearest representable surface witness, outward
    /// normal, and signed distance for a conceptual centered finite cylinder.
    /// </summary>
    /// <param name="point">The point in the query coordinate space.</param>
    /// <param name="center">The center in the query coordinate space.</param>
    /// <param name="axisDirection">The normalized axis direction in world coordinates.</param>
    /// <param name="axisLength">The full positive distance between the cap centers.</param>
    /// <param name="radius">The nonnegative radius in coordinate units.</param>
    /// <param name="fallbackRadialDirection">
    /// A normalized world direction that is not parallel to the axis. It resolves radial ties.
    /// </param>
    /// <param name="surfacePoint">The selected surface point in world coordinates on success.</param>
    /// <param name="outwardNormal">The outward normal of the selected surface feature in world coordinates on success.</param>
    /// <param name="signedDistance">
    /// The signed distance on success. The exact distance is negative inside and positive outside, before rounding.
    /// </param>
    /// <returns>
    /// <see langword="true"/> when the requested rounded result is representable and <see langword="false"/>
    /// otherwise.
    /// </returns>
    /// <exception cref="ArgumentException">
    /// <paramref name="axisDirection"/> is not normalized or <paramref name="fallbackRadialDirection"/> is not
    /// normalized or is parallel to the shape axis.
    /// </exception>
    /// <exception cref="ArgumentOutOfRangeException">
    /// <paramref name="axisLength"/> is not positive or <paramref name="radius"/> is negative.
    /// </exception>
    /// <remarks>
    /// Distance is positive outside, zero on the selected surface lattice
    /// witness, and negative inside. <paramref name="fallbackRadialDirection"/>
    /// selects the side normal only when the point lies on the cylinder axis.
    /// </remarks>
    public static bool TryGetClosestPointOnCenteredFiniteCylinderSurface(
        Vector3d point,
        Vector3d center,
        Vector3d axisDirection,
        Fixed64 axisLength,
        Fixed64 radius,
        Vector3d fallbackRadialDirection,
        out Vector3d surfacePoint,
        out Vector3d outwardNormal,
        out Fixed64 signedDistance)
    {
        ValidateCenteredSurface(
            axisDirection,
            axisLength,
            radius,
            fallbackRadialDirection,
            nameof(axisDirection),
            nameof(axisLength),
            nameof(radius),
            nameof(fallbackRadialDirection));
        return WideFiniteAxisIntersection.TryGetClosestPointOnCenteredFiniteCylinderSurface(
            point,
            center,
            axisDirection,
            axisLength,
            radius,
            fallbackRadialDirection,
            out surfacePoint,
            out outwardNormal,
            out signedDistance);
    }

    /// <summary>
    /// Attempts to return the nearest cylinder-surface offset relative to the
    /// cylinder center, together with its outward normal and signed distance.
    /// </summary>
    /// <param name="point">The point in the query coordinate space.</param>
    /// <param name="center">The center in the query coordinate space.</param>
    /// <param name="axisDirection">The normalized axis direction in world coordinates.</param>
    /// <param name="axisLength">The full positive distance between the cap centers.</param>
    /// <param name="radius">The nonnegative radius in coordinate units.</param>
    /// <param name="fallbackRadialDirection">
    /// A normalized world direction that is not parallel to the axis. It resolves radial ties.
    /// </param>
    /// <param name="surfaceOffset">
    /// The selected surface offset relative to the shape center, expressed along world axes on success.
    /// </param>
    /// <param name="outwardNormal">The outward normal of the selected surface feature in world coordinates on success.</param>
    /// <param name="signedDistance">
    /// The signed distance on success. The exact distance is negative inside and positive outside, before rounding.
    /// </param>
    /// <returns>
    /// <see langword="true"/> when the requested rounded result is representable and <see langword="false"/>
    /// otherwise.
    /// </returns>
    /// <exception cref="ArgumentException">
    /// <paramref name="axisDirection"/> is not normalized or <paramref name="fallbackRadialDirection"/> is not
    /// normalized or is parallel to the shape axis.
    /// </exception>
    /// <exception cref="ArgumentOutOfRangeException">
    /// <paramref name="axisLength"/> is not positive or <paramref name="radius"/> is negative.
    /// </exception>
    /// <remarks>
    /// Unlike the absolute-witness overload, this relation remains usable when
    /// adding the returned offset to <paramref name="center"/> would exceed the
    /// public scalar range.
    /// </remarks>
    public static bool TryGetClosestCenteredFiniteCylinderSurfaceOffset(
        Vector3d point,
        Vector3d center,
        Vector3d axisDirection,
        Fixed64 axisLength,
        Fixed64 radius,
        Vector3d fallbackRadialDirection,
        out Vector3d surfaceOffset,
        out Vector3d outwardNormal,
        out Fixed64 signedDistance)
    {
        ValidateCenteredSurface(
            axisDirection,
            axisLength,
            radius,
            fallbackRadialDirection,
            nameof(axisDirection),
            nameof(axisLength),
            nameof(radius),
            nameof(fallbackRadialDirection));
        return WideFiniteAxisIntersection.TryGetClosestCenteredFiniteCylinderSurfaceOffset(
            point,
            center,
            axisDirection,
            axisLength,
            radius,
            fallbackRadialDirection,
            out surfaceOffset,
            out outwardNormal,
            out signedDistance);
    }

    /// <summary>
    /// Returns whether a conceptual centered finite cone inclusively overlaps
    /// a sphere without materializing a contact witness.
    /// </summary>
    /// <param name="coneCenter">The cone center in the query coordinate space.</param>
    /// <param name="baseToApexDirection">The normalized world direction from the base center to the apex.</param>
    /// <param name="coneHeight">The cone's full positive axis length.</param>
    /// <param name="coneRadius">The cone's nonnegative radius in coordinate units.</param>
    /// <param name="sphereCenter">The sphere center in the query coordinate space.</param>
    /// <param name="sphereRadius">The sphere's nonnegative radius in coordinate units.</param>
    /// <returns>
    /// <see langword="true"/> when the query finds closed overlap or tangency and <see langword="false"/>
    /// otherwise.
    /// </returns>
    /// <exception cref="ArgumentException"><paramref name="baseToApexDirection"/> is not normalized.</exception>
    /// <exception cref="ArgumentOutOfRangeException">
    /// <paramref name="coneHeight"/> is not positive, <paramref name="coneRadius"/> is negative, or
    /// <paramref name="sphereRadius"/> is negative.
    /// </exception>
    public static bool DoesCenteredFiniteConeOverlapSphere(
        Vector3d coneCenter,
        Vector3d baseToApexDirection,
        Fixed64 coneHeight,
        Fixed64 coneRadius,
        Vector3d sphereCenter,
        Fixed64 sphereRadius)
    {
        ValidateCenteredCone(
            baseToApexDirection,
            coneHeight,
            coneRadius);
        if (sphereRadius < Fixed64.Zero)
            throw new ArgumentOutOfRangeException(nameof(sphereRadius));

        return WideFiniteAxisIntersection.DoesCenteredFiniteConeOverlapSphere(
            coneCenter,
            baseToApexDirection,
            coneHeight,
            coneRadius,
            sphereCenter,
            sphereRadius);
    }

    /// <summary>
    /// Attempts to return the nearest representable surface witness, outward
    /// normal, and signed distance for a conceptual centered finite cone.
    /// </summary>
    /// <param name="point">The point in the query coordinate space.</param>
    /// <param name="center">The center in the query coordinate space.</param>
    /// <param name="baseToApexDirection">The normalized world direction from the base center to the apex.</param>
    /// <param name="height">The full positive distance from the cone apex to its base center.</param>
    /// <param name="radius">The nonnegative radius in coordinate units.</param>
    /// <param name="fallbackRadialDirection">
    /// A normalized world direction that is not parallel to the axis. It resolves radial ties.
    /// </param>
    /// <param name="surfacePoint">The selected surface point in world coordinates on success.</param>
    /// <param name="outwardNormal">The outward normal of the selected surface feature in world coordinates on success.</param>
    /// <param name="signedDistance">
    /// The signed distance on success. The exact distance is negative inside and positive outside, before rounding.
    /// </param>
    /// <returns>
    /// <see langword="true"/> when the requested rounded result is representable and <see langword="false"/>
    /// otherwise.
    /// </returns>
    /// <exception cref="ArgumentException">
    /// <paramref name="baseToApexDirection"/> is not normalized or <paramref name="fallbackRadialDirection"/> is
    /// not normalized or is parallel to the shape axis.
    /// </exception>
    /// <exception cref="ArgumentOutOfRangeException">
    /// <paramref name="height"/> is not positive or <paramref name="radius"/> is negative.
    /// </exception>
    /// <remarks>
    /// The axis points from the base toward the apex. Distance is positive
    /// outside, zero on the selected surface lattice witness, and negative
    /// inside. <paramref name="fallbackRadialDirection"/> selects the side
    /// normal only for an axial point or apex tie.
    /// </remarks>
    public static bool TryGetClosestPointOnCenteredFiniteConeSurface(
        Vector3d point,
        Vector3d center,
        Vector3d baseToApexDirection,
        Fixed64 height,
        Fixed64 radius,
        Vector3d fallbackRadialDirection,
        out Vector3d surfacePoint,
        out Vector3d outwardNormal,
        out Fixed64 signedDistance)
    {
        ValidateCenteredSurface(
            baseToApexDirection,
            height,
            radius,
            fallbackRadialDirection,
            nameof(baseToApexDirection),
            nameof(height),
            nameof(radius),
            nameof(fallbackRadialDirection));
        return WideFiniteAxisIntersection.TryGetClosestPointOnCenteredFiniteConeSurface(
            point,
            center,
            baseToApexDirection,
            height,
            radius,
            fallbackRadialDirection,
            out surfacePoint,
            out outwardNormal,
            out signedDistance);
    }

    /// <summary>
    /// Attempts to return the nearest cone-surface offset relative to the cone
    /// center, together with its outward normal and signed distance.
    /// </summary>
    /// <param name="point">The point in the query coordinate space.</param>
    /// <param name="center">The center in the query coordinate space.</param>
    /// <param name="baseToApexDirection">The normalized world direction from the base center to the apex.</param>
    /// <param name="height">The full positive distance from the cone apex to its base center.</param>
    /// <param name="radius">The nonnegative radius in coordinate units.</param>
    /// <param name="fallbackRadialDirection">
    /// A normalized world direction that is not parallel to the axis. It resolves radial ties.
    /// </param>
    /// <param name="surfaceOffset">
    /// The selected surface offset relative to the shape center, expressed along world axes on success.
    /// </param>
    /// <param name="outwardNormal">The outward normal of the selected surface feature in world coordinates on success.</param>
    /// <param name="signedDistance">
    /// The signed distance on success. The exact distance is negative inside and positive outside, before rounding.
    /// </param>
    /// <returns>
    /// <see langword="true"/> when the requested rounded result is representable and <see langword="false"/>
    /// otherwise.
    /// </returns>
    /// <exception cref="ArgumentException">
    /// <paramref name="baseToApexDirection"/> is not normalized or <paramref name="fallbackRadialDirection"/> is
    /// not normalized or is parallel to the shape axis.
    /// </exception>
    /// <exception cref="ArgumentOutOfRangeException">
    /// <paramref name="height"/> is not positive or <paramref name="radius"/> is negative.
    /// </exception>
    /// <remarks>
    /// Unlike the absolute-witness overload, this relation remains usable when
    /// adding the returned offset to <paramref name="center"/> would exceed the
    /// public scalar range.
    /// </remarks>
    public static bool TryGetClosestCenteredFiniteConeSurfaceOffset(
        Vector3d point,
        Vector3d center,
        Vector3d baseToApexDirection,
        Fixed64 height,
        Fixed64 radius,
        Vector3d fallbackRadialDirection,
        out Vector3d surfaceOffset,
        out Vector3d outwardNormal,
        out Fixed64 signedDistance)
    {
        ValidateCenteredSurface(
            baseToApexDirection,
            height,
            radius,
            fallbackRadialDirection,
            nameof(baseToApexDirection),
            nameof(height),
            nameof(radius),
            nameof(fallbackRadialDirection));
        return WideFiniteAxisIntersection.TryGetClosestCenteredFiniteConeSurfaceOffset(
            point,
            center,
            baseToApexDirection,
            height,
            radius,
            fallbackRadialDirection,
            out surfaceOffset,
            out outwardNormal,
            out signedDistance);
    }

    private static void ValidateCenteredCone(
        Vector3d axisDirection,
        Fixed64 height,
        Fixed64 radius)
    {
        if (!axisDirection.IsNormalized())
        {
            throw new ArgumentException(
                "Cone axis direction must be normalized.",
                nameof(axisDirection));
        }
        if (height <= Fixed64.Zero)
            throw new ArgumentOutOfRangeException(nameof(height));
        if (radius < Fixed64.Zero)
            throw new ArgumentOutOfRangeException(nameof(radius));
    }

    private static void ValidateCylinderCapsuleProjection(
        Vector3d projectionAxis,
        Vector3d cylinderAxisDirection,
        Fixed64 cylinderAxisLength,
        Fixed64 cylinderRadius,
        Vector3d capsuleAxisDirection,
        Fixed64 capsuleAxisLength,
        Fixed64 capsuleRadius)
    {
        ValidateCylinderProjection(
            projectionAxis,
            cylinderAxisDirection,
            cylinderAxisLength,
            cylinderRadius,
            nameof(cylinderAxisDirection),
            nameof(cylinderAxisLength),
            nameof(cylinderRadius));
        if (!capsuleAxisDirection.IsNormalized())
        {
            throw new ArgumentException(
                "Capsule axis direction must be normalized.",
                nameof(capsuleAxisDirection));
        }
        if (capsuleAxisLength < Fixed64.Zero)
            throw new ArgumentOutOfRangeException(nameof(capsuleAxisLength));
        if (capsuleRadius < Fixed64.Zero)
            throw new ArgumentOutOfRangeException(nameof(capsuleRadius));
    }

    private static void ValidateCylinderProjection(
        Vector3d projectionAxis,
        Vector3d cylinderAxisDirection,
        Fixed64 cylinderAxisLength,
        Fixed64 cylinderRadius,
        string axisParameterName,
        string lengthParameterName,
        string radiusParameterName)
    {
        if (!projectionAxis.IsNormalized())
            throw new ArgumentException("Projection axis must be normalized.", nameof(projectionAxis));
        if (!cylinderAxisDirection.IsNormalized())
            throw new ArgumentException("Cylinder axis direction must be normalized.", axisParameterName);
        if (cylinderAxisLength <= Fixed64.Zero)
            throw new ArgumentOutOfRangeException(lengthParameterName);
        if (cylinderRadius < Fixed64.Zero)
            throw new ArgumentOutOfRangeException(radiusParameterName);
    }

    private static void ValidateCenteredSurface(
        Vector3d axisDirection,
        Fixed64 axisLength,
        Fixed64 radius,
        Vector3d fallbackRadialDirection,
        string axisParameterName,
        string lengthParameterName,
        string radiusParameterName,
        string fallbackParameterName)
    {
        if (!axisDirection.IsNormalized())
            throw new ArgumentException("Axis direction must be normalized.", axisParameterName);
        if (axisLength <= Fixed64.Zero)
            throw new ArgumentOutOfRangeException(lengthParameterName);
        if (radius < Fixed64.Zero)
            throw new ArgumentOutOfRangeException(radiusParameterName);
        if (!fallbackRadialDirection.IsNormalized())
        {
            throw new ArgumentException(
                "Fallback radial direction must be normalized.",
                fallbackParameterName);
        }
        if (Vector3d.Cross(axisDirection, fallbackRadialDirection).IsZero)
        {
            throw new ArgumentException(
                "Fallback radial direction must not be parallel to the axis.",
                fallbackParameterName);
        }
    }

    /// <summary>
    /// Attempts to return the selected point on a conceptual centered capsule
    /// surface without narrowing its axis point before applying the radial
    /// offset.
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
    public static bool TryGetSurfacePointOnCenteredCapsule(
        Vector3d point,
        Vector3d center,
        Vector3d axisDirection,
        Fixed64 axisLength,
        Fixed64 radius,
        Vector3d surfaceDirection,
        out Vector3d surfacePoint)
    {
        ValidateCenteredAxis(axisDirection, axisLength);
        ValidateCenteredCapsuleSurface(radius, surfaceDirection);
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
    /// Attempts to return a centered-capsule surface offset relative to the
    /// capsule center.
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
        Vector3d point,
        Vector3d center,
        Vector3d axisDirection,
        Fixed64 axisLength,
        Fixed64 radius,
        Vector3d surfaceDirection,
        out Vector3d surfaceOffset)
    {
        ValidateCenteredAxis(axisDirection, axisLength);
        ValidateCenteredCapsuleSurface(radius, surfaceDirection);
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
    /// point in the supplied normalized world-space surface direction.
    /// </summary>
    /// <param name="point">The point in the query coordinate space.</param>
    /// <param name="center">The center in the query coordinate space.</param>
    /// <param name="axisDirection">The normalized axis direction in world coordinates.</param>
    /// <param name="axisLength">The full nonnegative length of the central segment. Zero makes the core a point.</param>
    /// <param name="radius">The nonnegative radius in coordinate units.</param>
    /// <param name="surfaceDirection">
    /// The normalized world-space direction in which to offset the closest core point by the radius.
    /// </param>
    /// <returns>The selected point retained in the shape's local frame.</returns>
    /// <exception cref="ArgumentException">
    /// <paramref name="axisDirection"/> is not normalized or <paramref name="surfaceDirection"/> is not normalized.
    /// </exception>
    /// <exception cref="ArgumentOutOfRangeException">
    /// <paramref name="axisLength"/> is negative or <paramref name="radius"/> is negative.
    /// </exception>
    /// <remarks>
    /// The closest axial offset and radial offset remain separate in an
    /// identity-rotation frame, avoiding an inverse-/forward-rotation
    /// round-trip when the caller already owns the world-space direction.
    /// </remarks>
    public static FixedPointAnchor GetSurfaceAnchorOnCenteredCapsule(
        Vector3d point,
        Vector3d center,
        Vector3d axisDirection,
        Fixed64 axisLength,
        Fixed64 radius,
        Vector3d surfaceDirection)
    {
        ValidateCenteredAxis(axisDirection, axisLength);
        ValidateCenteredCapsuleSurface(radius, surfaceDirection);
        return WideFiniteAxisIntersection.GetSurfaceAnchorOnCenteredCapsule(
            point,
            center,
            axisDirection,
            axisLength,
            radius,
            surfaceDirection);
    }

    /// <summary>
    /// Returns the centered-capsule surface anchor nearest to a world-space
    /// point in the supplied normalized local-space surface direction.
    /// </summary>
    /// <param name="point">The point in the query coordinate space.</param>
    /// <param name="center">The center in the query coordinate space.</param>
    /// <param name="frameRotation">The normalized quaternion from local coordinates to world coordinates.</param>
    /// <param name="localAxisDirection">The normalized axis in the shape's local coordinates.</param>
    /// <param name="axisLength">The full nonnegative length of the central segment. Zero makes the core a point.</param>
    /// <param name="radius">The nonnegative radius in coordinate units.</param>
    /// <param name="localSurfaceDirection">
    /// The normalized shape-local direction in which to offset the closest core point by the radius.
    /// </param>
    /// <returns>The selected point retained in the shape's local frame.</returns>
    /// <exception cref="ArgumentException">
    /// <paramref name="frameRotation"/> is not normalized, <paramref name="localAxisDirection"/> is not normalized,
    /// or <paramref name="localSurfaceDirection"/> is not normalized.
    /// </exception>
    /// <exception cref="ArgumentOutOfRangeException">
    /// <paramref name="axisLength"/> is negative or <paramref name="radius"/> is negative.
    /// </exception>
    public static FixedPointAnchor GetSurfaceAnchorOnCenteredCapsule(
        Vector3d point,
        Vector3d center,
        FixedQuaternion frameRotation,
        Vector3d localAxisDirection,
        Fixed64 axisLength,
        Fixed64 radius,
        Vector3d localSurfaceDirection)
    {
        if (!frameRotation.IsNormalized())
        {
            throw new ArgumentException(
                "Frame rotation must be normalized.",
                nameof(frameRotation));
        }
        ValidateCenteredAxis(localAxisDirection, axisLength);
        ValidateCenteredCapsuleSurface(radius, localSurfaceDirection);
        return WideFiniteAxisIntersection.GetSurfaceAnchorOnCenteredCapsule(
            point,
            center,
            frameRotation,
            localAxisDirection,
            axisLength,
            radius,
            localSurfaceDirection);
    }

    private static void ValidateCenteredCapsuleSurface(
        Fixed64 radius,
        Vector3d surfaceDirection)
    {
        if (radius < Fixed64.Zero)
            throw new ArgumentOutOfRangeException(nameof(radius));
        if (!surfaceDirection.IsNormalized())
        {
            throw new ArgumentException(
                "Surface direction must be normalized.",
                nameof(surfaceDirection));
        }
    }

    /// <summary>
    /// Attempts to return the nearest centered-capsule surface feature in its
    /// authoritative rigid frame.
    /// </summary>
    /// <param name="point">The point in the query coordinate space.</param>
    /// <param name="center">The center in the query coordinate space.</param>
    /// <param name="frameRotation">The normalized quaternion from local coordinates to world coordinates.</param>
    /// <param name="localAxisDirection">The normalized axis in the shape's local coordinates.</param>
    /// <param name="axisLength">The full nonnegative length of the central segment. Zero makes the core a point.</param>
    /// <param name="radius">The nonnegative radius in coordinate units.</param>
    /// <param name="localFallbackRadialDirection">
    /// A normalized shape-local direction that is not parallel to the local axis. It resolves radial ties.
    /// </param>
    /// <param name="surfaceAnchor">The selected surface feature retained in the shape's local frame on success.</param>
    /// <param name="outwardNormal">The outward normal of the selected surface feature in world coordinates on success.</param>
    /// <param name="signedDistance">
    /// The signed distance on success. The exact distance is negative inside and positive outside, before rounding.
    /// </param>
    /// <returns>
    /// <see langword="true"/> when the requested rounded result is representable and <see langword="false"/>
    /// otherwise.
    /// </returns>
    /// <exception cref="ArgumentException">
    /// <paramref name="frameRotation"/> is not normalized, <paramref name="localAxisDirection"/> is not normalized,
    /// or <paramref name="localFallbackRadialDirection"/> is not normalized or is parallel to the shape axis.
    /// </exception>
    /// <exception cref="ArgumentOutOfRangeException">
    /// <paramref name="axisLength"/> is negative or <paramref name="radius"/> is negative.
    /// </exception>
    /// <remarks>
    /// The returned anchor retains odd full-length cap positions and its
    /// radial feature until final materialization. Distance is positive
    /// outside, zero on the selected surface lattice witness, and negative
    /// inside.
    /// </remarks>
    public static bool TryGetClosestCenteredCapsuleSurfaceAnchor(
        Vector3d point,
        Vector3d center,
        FixedQuaternion frameRotation,
        Vector3d localAxisDirection,
        Fixed64 axisLength,
        Fixed64 radius,
        Vector3d localFallbackRadialDirection,
        out FixedPointAnchor surfaceAnchor,
        out Vector3d outwardNormal,
        out Fixed64 signedDistance)
    {
        if (!frameRotation.IsNormalized())
        {
            throw new ArgumentException(
                "Frame rotation must be normalized.",
                nameof(frameRotation));
        }
        ValidateCenteredAxis(localAxisDirection, axisLength);
        ValidateCenteredCapsuleSurface(
            radius,
            localFallbackRadialDirection);
        if (Vector3d.Cross(
                localAxisDirection,
                localFallbackRadialDirection).IsZero)
        {
            throw new ArgumentException(
                "Fallback radial direction must not be parallel to the axis.",
                nameof(localFallbackRadialDirection));
        }

        return WideFiniteAxisIntersection
            .TryGetClosestCenteredCapsuleSurfaceAnchor(
                point,
                center,
                frameRotation,
                localAxisDirection,
                axisLength,
                radius,
                localFallbackRadialDirection,
                out surfaceAnchor,
                out outwardNormal,
                out signedDistance);
    }

    /// <summary>
    /// Attempts to return the nearest centered finite-cylinder surface feature
    /// in its authoritative rigid frame.
    /// </summary>
    /// <param name="point">The point in the query coordinate space.</param>
    /// <param name="center">The center in the query coordinate space.</param>
    /// <param name="frameRotation">The normalized quaternion from local coordinates to world coordinates.</param>
    /// <param name="localAxisDirection">The normalized axis in the shape's local coordinates.</param>
    /// <param name="axisLength">The full positive distance between the cap centers.</param>
    /// <param name="radius">The nonnegative radius in coordinate units.</param>
    /// <param name="localFallbackRadialDirection">
    /// A normalized shape-local direction that is not parallel to the local axis. It resolves radial ties.
    /// </param>
    /// <param name="surfaceAnchor">The selected surface feature retained in the shape's local frame on success.</param>
    /// <param name="outwardNormal">The outward normal of the selected surface feature in world coordinates on success.</param>
    /// <param name="signedDistance">
    /// The signed distance on success. The exact distance is negative inside and positive outside, before rounding.
    /// </param>
    /// <returns>
    /// <see langword="true"/> when the requested rounded result is representable and <see langword="false"/>
    /// otherwise.
    /// </returns>
    /// <exception cref="ArgumentException">
    /// <paramref name="frameRotation"/> is not normalized, <paramref name="localAxisDirection"/> is not normalized,
    /// or <paramref name="localFallbackRadialDirection"/> is not normalized or is parallel to the shape axis.
    /// </exception>
    /// <exception cref="ArgumentOutOfRangeException">
    /// <paramref name="axisLength"/> is not positive or <paramref name="radius"/> is negative.
    /// </exception>
    /// <remarks>
    /// The returned anchor retains axial and radial feature terms separately,
    /// so callers do not need to inverse-rotate an already rounded world-space
    /// offset. Distance is positive outside, zero on the selected surface
    /// witness, and negative inside.
    /// </remarks>
    public static bool TryGetClosestCenteredFiniteCylinderSurfaceAnchor(
        Vector3d point,
        Vector3d center,
        FixedQuaternion frameRotation,
        Vector3d localAxisDirection,
        Fixed64 axisLength,
        Fixed64 radius,
        Vector3d localFallbackRadialDirection,
        out FixedPointAnchor surfaceAnchor,
        out Vector3d outwardNormal,
        out Fixed64 signedDistance)
    {
        ValidateCenteredSurfaceFrame(
            frameRotation,
            localAxisDirection,
            axisLength,
            radius,
            localFallbackRadialDirection);
        return WideFiniteAxisIntersection
            .TryGetClosestCenteredFiniteCylinderSurfaceAnchor(
                point,
                center,
                frameRotation,
                localAxisDirection,
                axisLength,
                radius,
                localFallbackRadialDirection,
                out surfaceAnchor,
                out outwardNormal,
                out signedDistance);
    }

    /// <summary>
    /// Attempts to return the nearest centered finite-cone surface feature in
    /// its authoritative rigid frame.
    /// </summary>
    /// <param name="point">The point in the query coordinate space.</param>
    /// <param name="center">The center in the query coordinate space.</param>
    /// <param name="frameRotation">The normalized quaternion from local coordinates to world coordinates.</param>
    /// <param name="localAxisDirection">The normalized axis in the shape's local coordinates.</param>
    /// <param name="height">The full positive distance from the cone apex to its base center.</param>
    /// <param name="radius">The nonnegative radius in coordinate units.</param>
    /// <param name="localFallbackRadialDirection">
    /// A normalized shape-local direction that is not parallel to the local axis. It resolves radial ties.
    /// </param>
    /// <param name="surfaceAnchor">The selected surface feature retained in the shape's local frame on success.</param>
    /// <param name="outwardNormal">The outward normal of the selected surface feature in world coordinates on success.</param>
    /// <param name="signedDistance">
    /// The signed distance on success. The exact distance is negative inside and positive outside, before rounding.
    /// </param>
    /// <returns>
    /// <see langword="true"/> when the requested rounded result is representable and <see langword="false"/>
    /// otherwise.
    /// </returns>
    /// <exception cref="ArgumentException">
    /// <paramref name="frameRotation"/> is not normalized, <paramref name="localAxisDirection"/> is not normalized,
    /// or <paramref name="localFallbackRadialDirection"/> is not normalized or is parallel to the shape axis.
    /// </exception>
    /// <exception cref="ArgumentOutOfRangeException">
    /// <paramref name="height"/> is not positive or <paramref name="radius"/> is negative.
    /// </exception>
    /// <remarks>
    /// The local axis points from the base toward the apex. The returned anchor
    /// retains the selected meridian feature without a rounded world-offset
    /// round trip. Distance is positive outside, zero on the selected surface
    /// witness, and negative inside.
    /// </remarks>
    public static bool TryGetClosestCenteredFiniteConeSurfaceAnchor(
        Vector3d point,
        Vector3d center,
        FixedQuaternion frameRotation,
        Vector3d localAxisDirection,
        Fixed64 height,
        Fixed64 radius,
        Vector3d localFallbackRadialDirection,
        out FixedPointAnchor surfaceAnchor,
        out Vector3d outwardNormal,
        out Fixed64 signedDistance)
    {
        ValidateCenteredSurfaceFrame(
            frameRotation,
            localAxisDirection,
            height,
            radius,
            localFallbackRadialDirection);
        return WideFiniteAxisIntersection
            .TryGetClosestCenteredFiniteConeSurfaceAnchor(
                point,
                center,
                frameRotation,
                localAxisDirection,
                height,
                radius,
                localFallbackRadialDirection,
                out surfaceAnchor,
                out outwardNormal,
                out signedDistance);
    }

    private static void ValidateCenteredSurfaceFrame(
        FixedQuaternion frameRotation,
        Vector3d localAxisDirection,
        Fixed64 axisLength,
        Fixed64 radius,
        Vector3d localFallbackRadialDirection)
    {
        if (!frameRotation.IsNormalized())
        {
            throw new ArgumentException(
                "Frame rotation must be normalized.",
                nameof(frameRotation));
        }
        ValidateCenteredSurface(
            localAxisDirection,
            axisLength,
            radius,
            localFallbackRadialDirection,
            nameof(localAxisDirection),
            nameof(axisLength),
            nameof(radius),
            nameof(localFallbackRadialDirection));
    }
}
