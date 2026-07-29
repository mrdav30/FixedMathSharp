//=======================================================================
// FixedSegment.CenteredSurfaces.cs
//=======================================================================
// MIT License, Copyright (c) 2024–present David Oravsky (mrdav30)
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
