//=======================================================================
// FixedTriangle.RigidContacts.cs
//=======================================================================
// MIT License, Copyright (c) 2024–present David Oravsky (mrdav30)
// See LICENSE file in the project root for full license information.
//=======================================================================

using System;

namespace FixedMathSharp.Geometry;

/// <content>
/// Rigid-frame contact generation for <see cref="FixedTriangle"/> against
/// centered convex shapes (cylinders, cones, etc.) that are transformed by
/// their own position/rotation. Validates that the supplied triangle and
/// shape frames are rigid (uniform, non-degenerate) before computing a
/// support point on the shape and projecting/clamping it onto the triangle
/// to produce a <see cref="FixedContactAnchors"/> result.
/// </content>
public partial struct FixedTriangle
{
    /// <summary>
    /// Attempts to project one centered finite-cylinder support onto this
    /// rigidly transformed triangle.
    /// </summary>
    public readonly bool TryGetCenteredFiniteCylinderSupportContact(
        Vector3d triangleOrigin,
        FixedQuaternion triangleRotation,
        Vector3d cylinderCenter,
        FixedQuaternion cylinderRotation,
        Fixed64 cylinderHeight,
        Fixed64 cylinderRadius,
        Vector3d supportDirection,
        Vector3d normalTriangleToCylinder,
        out FixedContactAnchors contact)
    {
        ValidateRigidTriangleFrame(triangleRotation);
        ValidateRigidShapeFrame(cylinderRotation, nameof(cylinderRotation));
        ValidateCenteredSurfaceContact(
            cylinderHeight,
            cylinderRadius,
            normalTriangleToCylinder,
            nameof(cylinderHeight));
        FixedPointAnchor support =
            WideGeometry.GetCenteredCylinderSupportAnchor(
                cylinderCenter,
                cylinderRotation,
                cylinderHeight,
                cylinderRadius,
                supportDirection);
        return TryGetSupportContact(
            this,
            triangleOrigin,
            triangleRotation,
            support,
            normalTriangleToCylinder,
            out contact);
    }

    /// <summary>
    /// Attempts to project one centered finite-cone support onto this rigidly
    /// transformed triangle.
    /// </summary>
    public readonly bool TryGetCenteredFiniteConeSupportContact(
        Vector3d triangleOrigin,
        FixedQuaternion triangleRotation,
        Vector3d coneCenter,
        FixedQuaternion coneRotation,
        Fixed64 coneHeight,
        Fixed64 coneRadius,
        Vector3d supportDirection,
        Vector3d normalTriangleToCone,
        out FixedContactAnchors contact)
    {
        ValidateRigidTriangleFrame(triangleRotation);
        ValidateRigidShapeFrame(coneRotation, nameof(coneRotation));
        ValidateCenteredSurfaceContact(
            coneHeight,
            coneRadius,
            normalTriangleToCone,
            nameof(coneHeight));
        FixedPointAnchor support =
            WideGeometry.GetCenteredConeSupportAnchor(
                coneCenter,
                coneRotation,
                coneHeight,
                coneRadius,
                supportDirection);
        return TryGetSupportContact(
            this,
            triangleOrigin,
            triangleRotation,
            support,
            normalTriangleToCone,
            out contact);
    }

    /// <summary>
    /// Attempts to construct an exact contact between this rigidly transformed
    /// triangle and a centered capsule.
    /// </summary>
    /// <remarks>
    /// The returned triangle witness remains in the supplied triangle frame.
    /// The capsule witness remains in the supplied capsule frame so callers can
    /// retain stable rigid-frame feature identity independently of world pose.
    /// </remarks>
    public readonly bool TryGetCenteredCapsuleContact(
        Vector3d triangleOrigin,
        FixedQuaternion triangleRotation,
        Vector3d capsuleCenter,
        FixedQuaternion capsuleRotation,
        Fixed64 capsuleAxisLength,
        Fixed64 capsuleRadius,
        Vector3d fallbackNormal,
        out FixedContactAnchors contact)
    {
        ValidateRigidTriangleFrame(triangleRotation);
        ValidateRigidShapeFrame(capsuleRotation, nameof(capsuleRotation));
        Vector3d capsuleAxisDirection =
            capsuleRotation.Rotate(Vector3d.Up).Normalized;
        ValidateCenteredCapsuleContact(
            capsuleAxisDirection,
            capsuleAxisLength,
            capsuleRadius,
            fallbackNormal);

        var capsuleCenterAnchor = new FixedPointAnchor(
            capsuleCenter,
            FixedQuaternion.Identity,
            Vector3d.Zero);
        FixedQuaternion inverseTriangleRotation =
            triangleRotation.Inverse();
        if (capsuleCenterAnchor.TryGetLocalPointIn(
                triangleOrigin,
                triangleRotation,
                out Vector3d localCapsuleCenter))
        {
            return TryGetCenteredCapsuleContactInChart(
                this,
                localCapsuleCenter,
                inverseTriangleRotation.Rotate(capsuleAxisDirection),
                capsuleAxisLength,
                capsuleRadius,
                inverseTriangleRotation.Rotate(fallbackNormal),
                triangleOrigin,
                triangleRotation,
                capsuleCenter,
                capsuleRotation,
                capsuleAxisDirection,
                chartIsTriangleLocal: true,
                out contact);
        }

        FixedPointAnchor first = new(
            triangleOrigin,
            triangleRotation,
            A);
        FixedPointAnchor second = new(
            triangleOrigin,
            triangleRotation,
            B);
        FixedPointAnchor third = new(
            triangleOrigin,
            triangleRotation,
            C);
        bool triangleRepresentable =
            first.TryGetPoint(out Vector3d worldFirst)
            & second.TryGetPoint(out Vector3d worldSecond)
            & third.TryGetPoint(out Vector3d worldThird);
        if (!triangleRepresentable)
        {
            contact = default;
            return false;
        }

        return TryGetCenteredCapsuleContactInChart(
            new FixedTriangle(worldFirst, worldSecond, worldThird),
            capsuleCenter,
            capsuleAxisDirection,
            capsuleAxisLength,
            capsuleRadius,
            fallbackNormal,
            triangleOrigin,
            triangleRotation,
            capsuleCenter,
            capsuleRotation,
            capsuleAxisDirection,
            chartIsTriangleLocal: false,
            out contact);
    }

    /// <summary>
    /// Attempts to construct an exact contact between this rigidly transformed
    /// triangle and a sphere.
    /// </summary>
    /// <remarks>
    /// The relation selects whichever exact coordinate chart can represent the
    /// interacting features. The returned triangle witness always remains in
    /// the supplied triangle frame.
    /// </remarks>
    public readonly bool TryGetSphereContact(
        Vector3d triangleOrigin,
        FixedQuaternion triangleRotation,
        Vector3d sphereCenter,
        FixedQuaternion sphereRotation,
        Fixed64 sphereRadius,
        out FixedContactAnchors contact)
    {
        ValidateRigidTriangleFrame(triangleRotation);
        ValidateRigidShapeFrame(sphereRotation, nameof(sphereRotation));
        if (sphereRadius < Fixed64.Zero)
            throw new ArgumentOutOfRangeException(nameof(sphereRadius));

        var sphereCenterAnchor = new FixedPointAnchor(
            sphereCenter,
            FixedQuaternion.Identity,
            Vector3d.Zero);
        if (sphereCenterAnchor.TryGetLocalPointIn(
                triangleOrigin,
                triangleRotation,
                out Vector3d localSphereCenter))
        {
            return TryGetSphereContactInChart(
                this,
                localSphereCenter,
                sphereCenter,
                sphereRotation,
                sphereRadius,
                triangleOrigin,
                triangleRotation,
                chartIsTriangleLocal: true,
                out contact);
        }

        FixedPointAnchor first = new(
            triangleOrigin,
            triangleRotation,
            A);
        FixedPointAnchor second = new(
            triangleOrigin,
            triangleRotation,
            B);
        FixedPointAnchor third = new(
            triangleOrigin,
            triangleRotation,
            C);
        bool triangleRepresentable =
            first.TryGetPoint(out Vector3d worldFirst)
            & second.TryGetPoint(out Vector3d worldSecond)
            & third.TryGetPoint(out Vector3d worldThird);
        if (!triangleRepresentable)
        {
            contact = default;
            return false;
        }

        return TryGetSphereContactInChart(
            new FixedTriangle(worldFirst, worldSecond, worldThird),
            sphereCenter,
            sphereCenter,
            sphereRotation,
            sphereRadius,
            triangleOrigin,
            triangleRotation,
            chartIsTriangleLocal: false,
            out contact);
    }

    private static void ValidateCenteredSurfaceContact(
        Fixed64 height,
        Fixed64 radius,
        Vector3d normal,
        string heightParameterName)
    {
        if (height <= Fixed64.Zero)
            throw new ArgumentOutOfRangeException(heightParameterName);
        if (radius < Fixed64.Zero)
            throw new ArgumentOutOfRangeException(nameof(radius));
        if (!normal.IsNormalized())
            throw new ArgumentException("Triangle contact normal must be normalized.", nameof(normal));
    }

    private static bool TryGetSphereContactInChart(
        FixedTriangle triangle,
        Vector3d chartSphereCenter,
        Vector3d worldSphereCenter,
        FixedQuaternion sphereRotation,
        Fixed64 sphereRadius,
        Vector3d triangleOrigin,
        FixedQuaternion triangleRotation,
        bool chartIsTriangleLocal,
        out FixedContactAnchors contact)
    {
        Vector3d pointOnTriangle = triangle.ClosestPoint(chartSphereCenter);
        if (!Vector3d.TryGetDistance(
                chartSphereCenter,
                pointOnTriangle,
                out Fixed64 distance)
            || distance > sphereRadius)
        {
            contact = default;
            return false;
        }

        Vector3d chartNormal;
        if (distance > Fixed64.Epsilon)
        {
            _ = Vector3d.TrySubtract(
                chartSphereCenter,
                pointOnTriangle,
                out Vector3d separation);
            chartNormal = separation / distance;
        }
        else
        {
            chartNormal = triangle.Normal;
            if (Vector3d.CompareProjection(
                    chartSphereCenter,
                    triangle.Centroid,
                    chartNormal) < 0)
            {
                chartNormal = -chartNormal;
            }
        }

        Vector3d worldNormal = chartIsTriangleLocal
            ? triangleRotation.Rotate(chartNormal).Normalized
            : chartNormal;
        Vector3d triangleLocalPoint = pointOnTriangle;
        if (!chartIsTriangleLocal)
        {
            // A closest point of the materialized world triangle is a convex
            // combination of the representable source-local vertices.
            _ = new FixedPointAnchor(
                    pointOnTriangle,
                    FixedQuaternion.Identity,
                    Vector3d.Zero)
                .TryGetLocalPointIn(
                    triangleOrigin,
                    triangleRotation,
                    out triangleLocalPoint);
        }

        contact = new FixedContactAnchors(
            new FixedPointAnchor(
                triangleOrigin,
                triangleRotation,
                triangleLocalPoint),
            new FixedPointAnchor(
                worldSphereCenter,
                sphereRotation,
                sphereRotation.Inverse().Rotate(-worldNormal) * sphereRadius),
            worldNormal,
            sphereRadius - distance,
            depthIsClamped: false);
        return true;
    }

    private static bool TryGetCenteredCapsuleContactInChart(
        FixedTriangle triangle,
        Vector3d chartCapsuleCenter,
        Vector3d chartCapsuleAxis,
        Fixed64 capsuleAxisLength,
        Fixed64 capsuleRadius,
        Vector3d chartFallbackNormal,
        Vector3d triangleOrigin,
        FixedQuaternion triangleRotation,
        Vector3d worldCapsuleCenter,
        FixedQuaternion capsuleRotation,
        Vector3d worldCapsuleAxis,
        bool chartIsTriangleLocal,
        out FixedContactAnchors contact)
    {
        if (!WideFiniteAxisIntersection
            .TryGetCenteredCapsuleTriangleLocalContact(
                triangle,
                chartCapsuleCenter,
                chartCapsuleAxis,
                capsuleAxisLength,
                capsuleRadius,
                chartFallbackNormal,
                out Vector3d pointOnTriangle,
                out Fixed64 axisParameter,
                out Vector3d chartNormal,
                out Fixed64 depth,
                out bool depthIsClamped))
        {
            contact = default;
            return false;
        }

        Vector3d worldNormal = chartIsTriangleLocal
            ? triangleRotation.Rotate(chartNormal).Normalized
            : chartNormal;
        Vector3d triangleLocalPoint = pointOnTriangle;
        if (!chartIsTriangleLocal)
        {
            _ = new FixedPointAnchor(
                    pointOnTriangle,
                    FixedQuaternion.Identity,
                    Vector3d.Zero)
                .TryGetLocalPointIn(
                    triangleOrigin,
                    triangleRotation,
                    out triangleLocalPoint);
        }

        Vector3d capsuleLocalNormal =
            capsuleRotation.Inverse().Rotate(worldNormal);
        contact = new FixedContactAnchors(
            new FixedPointAnchor(
                triangleOrigin,
                triangleRotation,
                triangleLocalPoint),
            new FixedPointAnchor(
                worldCapsuleCenter,
                capsuleRotation,
                new Vector3d(
                    Fixed64.Zero,
                    axisParameter,
                    Fixed64.Zero),
                -capsuleLocalNormal * capsuleRadius),
            worldNormal,
            depth,
            depthIsClamped);
        return true;
    }

    private static bool TryGetSupportContact(
        FixedTriangle triangle,
        Vector3d triangleOrigin,
        FixedQuaternion triangleRotation,
        in FixedPointAnchor support,
        Vector3d worldNormal,
        out FixedContactAnchors contact)
    {
        if (!support.TryGetLocalPointIn(
                triangleOrigin,
                triangleRotation,
                out Vector3d localSupport))
        {
            contact = default;
            return false;
        }

        Vector3d localNormal =
            triangleRotation.Inverse().Rotate(worldNormal);
        if (!WideFiniteAxisIntersection.TryGetTriangleSupportPointContact(
                triangle,
                localSupport,
                localNormal,
                out Vector3d pointOnTriangle,
                out Fixed64 depth,
                out bool depthIsClamped))
        {
            contact = default;
            return false;
        }

        contact = new FixedContactAnchors(
            new FixedPointAnchor(
                triangleOrigin,
                triangleRotation,
                pointOnTriangle),
            support,
            worldNormal,
            depth,
            depthIsClamped);
        return true;
    }

    private static void ValidateRigidTriangleFrame(
        FixedQuaternion triangleRotation)
    {
        if (!triangleRotation.IsNormalized())
        {
            throw new ArgumentException(
                "The triangle rotation must be normalized.",
                nameof(triangleRotation));
        }
    }

    private static void ValidateRigidShapeFrame(
        FixedQuaternion rotation,
        string parameterName)
    {
        if (!rotation.IsNormalized())
        {
            throw new ArgumentException(
                "The shape rotation must be normalized.",
                parameterName);
        }
    }
}
