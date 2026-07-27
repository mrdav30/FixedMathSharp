//=======================================================================
// WideFiniteAxisIntersection.Containment.cs
//=======================================================================
// MIT License, Copyright (c) 2024–present David Oravsky (mrdav30)
// See LICENSE file in the project root for full license information.
//=======================================================================

namespace FixedMathSharp.Bounds;

/// <content>
/// Provides high-precision containment tests for points against finite cylinder
/// and capsule shapes, using wide (multi-word) arithmetic to avoid overflow
/// and precision loss when checking axial and radial bounds.
/// </content>
internal static partial class WideFiniteAxisIntersection
{
    internal static bool ContainsPointInCenteredFiniteCylinder(
        Vector3d point,
        Vector3d center,
        Vector3d axisDirection,
        Fixed64 axisLength,
        Fixed64 radius,
        bool strict)
    {
        Signed192 axisLengthSquared = GetDot(
            axisDirection,
            Vector3d.Zero,
            axisDirection,
            Vector3d.Zero);
        Signed192 distanceSquared = GetDot(point, center, point, center);
        Signed192 axisProjection = GetDot(
            point,
            center,
            axisDirection,
            Vector3d.Zero);
        Signed192 rawRadius = Signed192.Signed(radius.m_rawValue);
        return IsCenteredFiniteCylinderPointContained(
            distanceSquared,
            axisProjection,
            axisLengthSquared,
            GetSquaredRadius(rawRadius),
            GetCenteredAxialExtent(axisLengthSquared, axisLength, Fixed64.Zero),
            strict);
    }

    internal static bool ContainsPointInCenteredFiniteCylinder(
        Vector3d point,
        Vector3d center,
        Vector3d axisDirection,
        Fixed64 axisLength,
        Fixed64 radius,
        Fixed64 axialTolerance,
        Fixed64 radialTolerance,
        bool strict)
    {
        Signed192 axisLengthSquared = GetDot(
            axisDirection,
            Vector3d.Zero,
            axisDirection,
            Vector3d.Zero);
        Signed192 distanceSquared = GetDot(point, center, point, center);
        Signed192 axisProjection = GetDot(
            point,
            center,
            axisDirection,
            Vector3d.Zero);
        Signed192 expandedRadius = GetExpandedRadius(radius, radialTolerance);
        return IsCenteredFiniteCylinderPointContained(
            distanceSquared,
            axisProjection,
            axisLengthSquared,
            GetSquaredRadius(expandedRadius),
            GetCenteredAxialExtent(axisLengthSquared, axisLength, axialTolerance),
            strict);
    }

    private static bool IsCapsulePointContained(
        Signed192 startDistanceSquared,
        Signed192 endDistanceSquared,
        Signed192 axisProjection,
        Signed192 axisLengthSquared,
        Signed192 squaredRadius,
        bool strict)
    {
        if (axisProjection.Sign <= 0)
            return IsWithinRadius(startDistanceSquared, squaredRadius, strict);
        if (WideArithmetic.SubtractSigned192(axisProjection, axisLengthSquared).Sign >= 0)
            return IsWithinRadius(endDistanceSquared, squaredRadius, strict);

        int radialSign = GetRadialConstant(
            startDistanceSquared,
            axisProjection,
            axisLengthSquared,
            squaredRadius).Sign;
        return strict ? radialSign < 0 : radialSign <= 0;
    }

    private static bool IsFiniteCylinderPointContained(
        Signed192 distanceSquared,
        Signed192 axisProjection,
        Signed192 axisLengthSquared,
        Signed192 squaredRadius,
        bool strict)
    {
        int maximumProjectionSign = WideArithmetic.SubtractSigned192(
            axisProjection,
            axisLengthSquared).Sign;
        if (strict
                ? axisProjection.Sign <= 0 || maximumProjectionSign >= 0
                : axisProjection.Sign < 0 || maximumProjectionSign > 0)
        {
            return false;
        }

        int radialSign = GetRadialConstant(
            distanceSquared,
            axisProjection,
            axisLengthSquared,
            squaredRadius).Sign;
        return strict ? radialSign < 0 : radialSign <= 0;
    }

    private static bool IsCenteredFiniteCylinderPointContained(
        Signed192 distanceSquared,
        Signed192 axisProjection,
        Signed192 axisLengthSquared,
        Signed192 squaredRadius,
        Signed320 axialExtent,
        bool strict)
    {
        Signed320 scaledProjection = WideArithmetic.MultiplySigned192(DoubleParameterScale, axisProjection);
        Signed320 minimumProjection = WideArithmetic.SubtractSigned320(default, axialExtent);
        int minimumSign = WideArithmetic.SubtractSigned320(scaledProjection, minimumProjection).Sign;
        int maximumSign = WideArithmetic.SubtractSigned320(scaledProjection, axialExtent).Sign;
        if (strict
                ? minimumSign <= 0 || maximumSign >= 0
                : minimumSign < 0 || maximumSign > 0)
        {
            return false;
        }

        int radialSign = GetRadialConstant(
            distanceSquared,
            axisProjection,
            axisLengthSquared,
            squaredRadius).Sign;
        return strict ? radialSign < 0 : radialSign <= 0;
    }

    private static bool IsWithinRadius(
        Signed192 distanceSquared,
        Signed192 squaredRadius,
        bool strict)
    {
        int differenceSign = WideArithmetic.SubtractSigned192(distanceSquared, squaredRadius).Sign;
        return strict ? differenceSign < 0 : differenceSign <= 0;
    }

    private static Signed320 GetRadialConstant(
        Signed192 distanceSquared,
        Signed192 axisProjection,
        Signed192 axisLengthSquared,
        Signed192 squaredRadius) =>
        WideArithmetic.SubtractSigned320(
            WideArithmetic.MultiplySubtract(
                distanceSquared,
                axisLengthSquared,
                axisProjection,
                axisProjection),
            WideArithmetic.MultiplySigned192(squaredRadius, axisLengthSquared));
}
