//=======================================================================
// WideGeometry.Anchors.cs
//=======================================================================
// MIT License, Copyright (c) 2024–present David Oravsky (mrdav30)
// See LICENSE file in the project root for full license information.
//=======================================================================

namespace FixedMathSharp.Geometry;

/// <content>
/// Exact scalar reductions for computing support anchors of centered axis-aligned shapes.
/// </content>
internal static partial class WideGeometry
{
    internal static FixedPointAnchor GetCenteredCapsuleSupportAnchor(
        Vector3d center,
        FixedQuaternion canonicalAxisRotation,
        Fixed64 axisLength,
        Fixed64 radius,
        Vector3d worldDirection) =>
        GetCenteredAxisSupportAnchor(
            center,
            canonicalAxisRotation,
            Vector3d.Up,
            axisLength,
            radius,
            worldDirection,
            CenteredAxisSupportKind.Capsule);

    internal static FixedPointAnchor GetCenteredCapsuleSupportAnchor(
        Vector3d center,
        FixedQuaternion rotation,
        Vector3d localAxis,
        Fixed64 axisLength,
        Fixed64 radius,
        Vector3d worldDirection) =>
        GetCenteredAxisSupportAnchor(
            center,
            rotation,
            localAxis,
            axisLength,
            radius,
            worldDirection,
            CenteredAxisSupportKind.Capsule);

    internal static FixedPointAnchor GetCenteredCapsuleSupportAnchor(
        Vector3d center,
        FixedQuaternion rotation,
        Vector3d localAxis,
        Fixed64 axisLength,
        Fixed64 radius,
        Vector3d worldDirection,
        int axialSign) =>
        GetCenteredAxisSupportAnchor(
            center,
            rotation,
            localAxis,
            axisLength,
            radius,
            worldDirection,
            CenteredAxisSupportKind.Capsule,
            axialSign);

    internal static FixedPointAnchor GetCenteredCylinderSupportAnchor(
        Vector3d center,
        FixedQuaternion canonicalAxisRotation,
        Fixed64 axisLength,
        Fixed64 radius,
        Vector3d worldDirection) =>
        GetCenteredAxisSupportAnchor(
            center,
            canonicalAxisRotation,
            Vector3d.Up,
            axisLength,
            radius,
            worldDirection,
            CenteredAxisSupportKind.Cylinder);

    internal static FixedPointAnchor GetCenteredCylinderSupportAnchor(
        Vector3d center,
        FixedQuaternion rotation,
        Vector3d localAxis,
        Fixed64 axisLength,
        Fixed64 radius,
        Vector3d worldDirection) =>
        GetCenteredAxisSupportAnchor(
            center,
            rotation,
            localAxis,
            axisLength,
            radius,
            worldDirection,
            CenteredAxisSupportKind.Cylinder);

    internal static FixedPointAnchor GetCenteredCylinderSupportAnchor(
        Vector3d center,
        FixedQuaternion rotation,
        Vector3d localAxis,
        Fixed64 axisLength,
        Fixed64 radius,
        Vector3d worldDirection,
        int axialSign) =>
        GetCenteredAxisSupportAnchor(
            center,
            rotation,
            localAxis,
            axisLength,
            radius,
            worldDirection,
            CenteredAxisSupportKind.Cylinder,
            axialSign);

    internal static FixedPointAnchor GetCenteredConeSupportAnchor(
        Vector3d center,
        FixedQuaternion canonicalAxisRotation,
        Fixed64 height,
        Fixed64 radius,
        Vector3d worldDirection) =>
        GetCenteredAxisSupportAnchor(
            center,
            canonicalAxisRotation,
            Vector3d.Up,
            height,
            radius,
            worldDirection,
            CenteredAxisSupportKind.Cone);

    internal static FixedPointAnchor GetCenteredConeSupportAnchor(
        Vector3d center,
        FixedQuaternion rotation,
        Vector3d localAxis,
        Fixed64 height,
        Fixed64 radius,
        Vector3d worldDirection,
        bool apexSupport) =>
        GetCenteredAxisSupportAnchor(
            center,
            rotation,
            localAxis,
            height,
            radius,
            worldDirection,
            CenteredAxisSupportKind.Cone,
            forcedConeApex: apexSupport);

    private static FixedPointAnchor GetCenteredAxisSupportAnchor(
        Vector3d center,
        FixedQuaternion canonicalAxisRotation,
        Vector3d localAxis,
        Fixed64 axisLength,
        Fixed64 radius,
        Vector3d worldDirection,
        CenteredAxisSupportKind kind,
        int? forcedAxialSign = null,
        bool? forcedConeApex = null)
    {
        Vector3d localDirection = WideNormalization.GetNormalized(
            canonicalAxisRotation.Inverse().Rotate(worldDirection));
        Fixed64 halfLength = axisLength / Fixed64.Two;
        int axialSign = forcedAxialSign
            ?? WideOrientedBox.GetRotatedLocalProjectionSign(
                canonicalAxisRotation,
                localAxis,
                worldDirection);
        bool positiveAxis = axialSign > 0;
        Vector3d localPoint =
            localAxis * (positiveAxis ? halfLength : -halfLength);
        Fixed64 signedAxisLength = positiveAxis
            ? axisLength
            : -axisLength;
        Vector3d localDisplacement;
        Vector3d localRadialDirection;
        if (kind == CenteredAxisSupportKind.Capsule)
        {
            if (axialSign == 0)
            {
                localPoint = Vector3d.Zero;
                signedAxisLength = Fixed64.Zero;
            }
            localRadialDirection = localDirection;
            localDisplacement = localDirection * radius;
        }
        else
        {
            if (kind == CenteredAxisSupportKind.Cylinder)
            {
                Fixed64 axialProjection =
                    Vector3d.Dot(localDirection, localAxis);
                localRadialDirection = WideNormalization.GetNormalized(
                    localDirection - (localAxis * axialProjection));
            }
            else
            {
                Vector2d planarDirection = WideNormalization.GetNormalized(
                    new Vector2d(localDirection.X, localDirection.Z));
                if (planarDirection == Vector2d.Zero)
                    planarDirection = Vector2d.Right;
                localRadialDirection = new Vector3d(
                    planarDirection.X,
                    Fixed64.Zero,
                    planarDirection.Y);
            }
            localDisplacement = localRadialDirection * radius;
            bool coneApex = forcedConeApex
                ?? WideFiniteAxisIntersection
                    .IsCenteredFiniteConeApexSupport(
                        Vector3d.Up,
                        axisLength,
                        radius,
                        localDirection);
            if (kind == CenteredAxisSupportKind.Cone
                && coneApex)
            {
                signedAxisLength = axisLength;
                localRadialDirection = Vector3d.Zero;
                localDisplacement = Vector3d.Zero;
            }
            else if (kind == CenteredAxisSupportKind.Cone)
            {
                signedAxisLength = -axisLength;
                localPoint = new Vector3d(
                    Fixed64.Zero,
                    -halfLength,
                    Fixed64.Zero);
            }
        }

        FixedPointAnchorTerm3d exactLocalTerm =
            FixedPointAnchorTerm3d.CreateCenteredAxisSupport(
                localAxis,
                signedAxisLength,
                localRadialDirection,
                radius,
                localPoint,
                localDisplacement);
        return new FixedPointAnchor(
            center,
            canonicalAxisRotation,
            localPoint,
            localDisplacement,
            exactLocalTerm);
    }
}
