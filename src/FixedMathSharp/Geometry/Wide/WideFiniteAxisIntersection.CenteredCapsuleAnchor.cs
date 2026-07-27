//=======================================================================
// WideFiniteAxisIntersection.CenteredCapsuleAnchor.cs
//=======================================================================
// MIT License, Copyright (c) 2024–present David Oravsky (mrdav30)
// See LICENSE file in the project root for full license information.
//=======================================================================

namespace FixedMathSharp.Geometry;

/// <content>
/// Provides methods for computing surface anchors on capsules centered at a given point,
/// supporting both 2D and 3D representations with rotation and axis-aligned variants.
/// </content>
internal static partial class WideFiniteAxisIntersection
{
    internal static FixedPointAnchor2d GetSurfaceAnchorOnCenteredCapsule(
        Vector2d point,
        Vector2d center,
        Fixed64 frameRotation,
        Vector2d localAxisDirection,
        Fixed64 axisLength,
        Fixed64 radius,
        Vector2d localNormal)
    {
        _ = Vector2d.TryRotate(
            localAxisDirection,
            frameRotation,
            out Vector2d axisDirection);
        GetClosestCenteredAxisRatio(
            point,
            center,
            axisDirection,
            axisLength,
            out Signed192 numerator,
            out Signed192 denominator);
        _ = TryGetCenteredCapsuleSurfaceCoordinate(
            Fixed64.Zero,
            Fixed64.One,
            numerator,
            denominator,
            Fixed64.Zero,
            Fixed64.Zero,
            out Fixed64 axialDistance);
        Vector2d roundedRadialOffset = localNormal * radius;
        return new FixedPointAnchor2d(
            center,
            frameRotation,
            localAxisDirection * axialDistance,
            roundedRadialOffset,
            FixedPointAnchorTerm2d.CreateRadialSupport(
                localNormal,
                radius,
                roundedRadialOffset));
    }

    internal static FixedPointAnchor GetSurfaceAnchorOnCenteredCapsule(
        Vector3d point,
        Vector3d center,
        FixedQuaternion frameRotation,
        Vector3d localAxisDirection,
        Fixed64 axisLength,
        Fixed64 radius,
        Vector3d localNormal)
    {
        GetRigidLocalPointRelation(
            point,
            center,
            frameRotation,
            localAxisDirection,
            out RigidLocalPointRelation relation);
        int cap = GetCenteredCap(relation, axisLength);
        Fixed64 axialDistance = GetClosestCenteredRigidAxisDistance(
            relation,
            axisLength,
            cap);
        Vector3d roundedRadialOffset = localNormal * radius;
        return new FixedPointAnchor(
            center,
            frameRotation,
            localAxisDirection * axialDistance,
            roundedRadialOffset,
            FixedPointAnchorTerm3d.CreateRadialSupport(
                localNormal,
                radius,
                roundedRadialOffset));
    }

    internal static FixedPointAnchor GetSurfaceAnchorOnCenteredCapsule(
        Vector3d point,
        Vector3d center,
        Vector3d axisDirection,
        Fixed64 axisLength,
        Fixed64 radius,
        Vector3d surfaceDirection)
    {
        GetClosestCenteredAxisRatio(
            point,
            center,
            axisDirection,
            axisLength,
            out Signed192 numerator,
            out Signed192 denominator);
        _ = TryGetCenteredCapsuleSurfaceCoordinate(
            Fixed64.Zero,
            axisDirection.X,
            numerator,
            denominator,
            Fixed64.Zero,
            Fixed64.Zero,
            out Fixed64 axialX);
        _ = TryGetCenteredCapsuleSurfaceCoordinate(
            Fixed64.Zero,
            axisDirection.Y,
            numerator,
            denominator,
            Fixed64.Zero,
            Fixed64.Zero,
            out Fixed64 axialY);
        _ = TryGetCenteredCapsuleSurfaceCoordinate(
            Fixed64.Zero,
            axisDirection.Z,
            numerator,
            denominator,
            Fixed64.Zero,
            Fixed64.Zero,
            out Fixed64 axialZ);
        Vector3d roundedRadialOffset = surfaceDirection * radius;
        return new FixedPointAnchor(
            center,
            FixedQuaternion.Identity,
            new Vector3d(axialX, axialY, axialZ),
            roundedRadialOffset,
            FixedPointAnchorTerm3d.CreateRadialSupport(
                surfaceDirection,
                radius,
                roundedRadialOffset));
    }
}
