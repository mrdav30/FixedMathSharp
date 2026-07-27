//=======================================================================
// WideFiniteAxisIntersection.CenteredSurfaces.cs
//=======================================================================
// MIT License, Copyright (c) 2024-present David Oravsky (mrdav30)
// See LICENSE file in the project root for full license information.
//=======================================================================

namespace FixedMathSharp.Geometry;

/// <content>
/// Provides overlap and closest-surface-point queries for axis-aligned (origin-centered)
/// finite cones and cylinders, including surface anchors, normals, and signed distances.
/// </content>
internal static partial class WideFiniteAxisIntersection
{
    internal static bool DoesCenteredFiniteConeOverlapSphere(
        Vector3d coneCenter,
        Vector3d axisDirection,
        Fixed64 height,
        Fixed64 radius,
        Vector3d sphereCenter,
        Fixed64 sphereRadius)
    {
        if (WideFiniteConeIntersection.ContainsPointInCenteredCone(
                sphereCenter,
                coneCenter,
                axisDirection,
                height,
                radius,
                strict: false))
        {
            return true;
        }

        return TryGetClosestCenteredFiniteConeSurfaceAnchor(
                sphereCenter,
                coneCenter,
                FixedQuaternion.Identity,
                axisDirection,
                height,
                radius,
                Vector3d.Zero,
                out _,
                out _,
                out Fixed64 distance)
            && distance <= sphereRadius;
    }

    internal static bool TryGetClosestPointOnCenteredFiniteConeSurface(
        Vector3d point,
        Vector3d center,
        Vector3d axisDirection,
        Fixed64 height,
        Fixed64 radius,
        Vector3d fallbackRadialDirection,
        out Vector3d surfacePoint,
        out Vector3d outwardNormal,
        out Fixed64 signedDistance)
    {
        if (!TryGetClosestCenteredFiniteConeSurfaceOffset(
                point,
                center,
                axisDirection,
                height,
                radius,
                fallbackRadialDirection,
                out Vector3d surfaceOffset,
                out outwardNormal,
                out signedDistance)
            || !Vector3d.TryAdd(center, surfaceOffset, out surfacePoint))
        {
            surfacePoint = default;
            outwardNormal = default;
            signedDistance = default;
            return false;
        }

        return true;
    }

    internal static bool TryGetClosestCenteredFiniteConeSurfaceOffset(
        Vector3d point,
        Vector3d center,
        Vector3d axisDirection,
        Fixed64 height,
        Fixed64 radius,
        Vector3d fallbackRadialDirection,
        out Vector3d surfaceOffset,
        out Vector3d outwardNormal,
        out Fixed64 signedDistance)
    {
        if (!TryGetClosestCenteredFiniteConeSurfaceAnchor(
                point,
                center,
                FixedQuaternion.Identity,
                axisDirection,
                height,
                radius,
                fallbackRadialDirection,
                out FixedPointAnchor anchor,
                out outwardNormal,
                out signedDistance)
            || !anchor.TryGetLocalPointIn(
                center,
                FixedQuaternion.Identity,
                out surfaceOffset))
        {
            surfaceOffset = default;
            outwardNormal = default;
            signedDistance = default;
            return false;
        }
        return true;
    }

    internal static bool TryGetClosestPointOnCenteredFiniteCylinderSurface(
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
        if (!TryGetClosestCenteredFiniteCylinderSurfaceOffset(
                point,
                center,
                axisDirection,
                axisLength,
                radius,
                fallbackRadialDirection,
                out Vector3d surfaceOffset,
                out outwardNormal,
                out signedDistance)
            || !Vector3d.TryAdd(center, surfaceOffset, out surfacePoint))
        {
            surfacePoint = default;
            outwardNormal = default;
            signedDistance = default;
            return false;
        }

        return true;
    }

    internal static bool TryGetClosestCenteredFiniteCylinderSurfaceOffset(
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
        if (TryGetClosestCenteredFiniteCylinderSurfaceAnchor(
                point,
                center,
                FixedQuaternion.Identity,
                axisDirection,
                axisLength,
                radius,
                fallbackRadialDirection,
                out FixedPointAnchor anchor,
                out outwardNormal,
                out signedDistance)
            && anchor.TryGetLocalPointIn(
                center,
                FixedQuaternion.Identity,
                out surfaceOffset))
        {
            return true;
        }

        surfaceOffset = default;
        outwardNormal = default;
        signedDistance = default;
        return false;
    }

    private static Vector3d GetCenteredConeSideNormal(
        Vector3d radialNormal,
        Vector3d axisDirection,
        Fixed64 height,
        Fixed64 radius)
    {
        Signed320 x = WideArithmetic.AddSigned320(
            WideArithmetic.MultiplySigned192(
                Signed192.Signed(radialNormal.X.m_rawValue),
                Signed192.Signed(height.m_rawValue)),
            WideArithmetic.MultiplySigned192(
                Signed192.Signed(axisDirection.X.m_rawValue),
                Signed192.Signed(radius.m_rawValue)));
        Signed320 y = WideArithmetic.AddSigned320(
            WideArithmetic.MultiplySigned192(
                Signed192.Signed(radialNormal.Y.m_rawValue),
                Signed192.Signed(height.m_rawValue)),
            WideArithmetic.MultiplySigned192(
                Signed192.Signed(axisDirection.Y.m_rawValue),
                Signed192.Signed(radius.m_rawValue)));
        Signed320 z = WideArithmetic.AddSigned320(
            WideArithmetic.MultiplySigned192(
                Signed192.Signed(radialNormal.Z.m_rawValue),
                Signed192.Signed(height.m_rawValue)),
            WideArithmetic.MultiplySigned192(
                Signed192.Signed(axisDirection.Z.m_rawValue),
                Signed192.Signed(radius.m_rawValue)));
        return WideGeometry.GetNormalized(x, y, z);
    }
}
