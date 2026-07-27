//=======================================================================
// FixedBoundBox.FiniteAxis.cs
//=======================================================================
// MIT License, Copyright (c) 2024–present David Oravsky (mrdav30)
// See LICENSE file in the project root for full license information.
//=======================================================================

using System;

namespace FixedMathSharp.Bounds;

/// <content>
/// Contains methods for creating a representable-domain intersection of the tight axis-aligned bounds of finite
/// cones, cylinders, and capsules from their center-axis lengths and directions.
/// </content>
public partial struct FixedBoundBox
{
    /// <summary>
    /// Creates the representable-domain intersection of the tight axis-aligned
    /// bounds of a finite cone with a flat circular base.
    /// </summary>
    /// <param name="apex">The cone apex.</param>
    /// <param name="baseCenter">The center of the cone's flat base.</param>
    /// <param name="axisDirection">The normalized direction from apex to base.</param>
    /// <param name="baseRadius">The nonnegative base radius.</param>
    /// <remarks>
    /// Base-disk extents account for the exact squared length of a representably
    /// normalized axis. They are rounded outward so broad-phase bounds cannot
    /// omit a boundary point. Coordinates beyond the scalar domain are clipped.
    /// </remarks>
    /// <exception cref="ArgumentException">
    /// <paramref name="axisDirection"/> is zero or not normalized.
    /// </exception>
    /// <exception cref="ArgumentOutOfRangeException">
    /// <paramref name="baseRadius"/> is negative.
    /// </exception>
    public static FixedBoundBox FromFiniteConeClippedToDomain(
        Vector3d apex,
        Vector3d baseCenter,
        Vector3d axisDirection,
        Fixed64 baseRadius)
    {
        if (!axisDirection.IsNormalized())
            throw new ArgumentException("Finite cone axis direction must be normalized.", nameof(axisDirection));
        if (baseRadius < Fixed64.Zero)
            throw new ArgumentOutOfRangeException(nameof(baseRadius));

        Signed192 axisLengthSquared = GetFiniteAxisLengthSquared(axisDirection);
        Vector3d baseExtents = new(
            GetFiniteConeDiskExtent(axisLengthSquared, axisDirection.X, baseRadius),
            GetFiniteConeDiskExtent(axisLengthSquared, axisDirection.Y, baseRadius),
            GetFiniteConeDiskExtent(axisLengthSquared, axisDirection.Z, baseRadius));
        return FromMinMax(
            Vector3d.Min(apex, baseCenter - baseExtents),
            Vector3d.Max(apex, baseCenter + baseExtents));
    }

    /// <summary>
    /// Creates the representable-domain intersection of a centered capsule's
    /// tight axis-aligned bounds from its full center-axis length.
    /// </summary>
    public static FixedBoundBox FromCenteredCapsuleClippedToDomain(
        Vector3d center,
        Vector3d axisDirection,
        Fixed64 axisLength,
        Fixed64 radius)
    {
        ValidateCenteredFiniteAxis(axisDirection, axisLength, radius, requirePositiveLength: false);
        return FromCenteredFiniteAxisClippedToDomain(
            center,
            axisDirection,
            axisLength,
            new Vector3d(radius, radius, radius));
    }

    /// <summary>
    /// Creates the representable-domain intersection of a centered finite
    /// cylinder's tight axis-aligned bounds from its full axis length.
    /// </summary>
    public static FixedBoundBox FromCenteredFiniteCylinderClippedToDomain(
        Vector3d center,
        Vector3d axisDirection,
        Fixed64 axisLength,
        Fixed64 radius)
    {
        ValidateCenteredFiniteAxis(axisDirection, axisLength, radius, requirePositiveLength: true);
        Signed192 axisLengthSquared = GetFiniteAxisLengthSquared(axisDirection);
        return FromCenteredFiniteCylinderExtentsClippedToDomain(
            center,
            GetCenteredFiniteCylinderOutwardExtentRaw(
                axisLengthSquared, axisDirection.X, axisLength, radius),
            GetCenteredFiniteCylinderOutwardExtentRaw(
                axisLengthSquared, axisDirection.Y, axisLength, radius),
            GetCenteredFiniteCylinderOutwardExtentRaw(
                axisLengthSquared, axisDirection.Z, axisLength, radius));
    }

    /// <summary>
    /// Creates the representable-domain intersection of a centered finite
    /// cone's tight axis-aligned bounds from its full height.
    /// </summary>
    /// <param name="center">The midpoint between the base center and apex.</param>
    /// <param name="axisDirection">
    /// The normalized direction from the base center toward the apex.
    /// </param>
    /// <param name="height">The positive full base-to-apex height.</param>
    /// <param name="radius">The nonnegative base radius.</param>
    public static FixedBoundBox FromCenteredFiniteConeClippedToDomain(
        Vector3d center,
        Vector3d axisDirection,
        Fixed64 height,
        Fixed64 radius)
    {
        if (!axisDirection.IsNormalized())
            throw new ArgumentException("Finite cone axis direction must be normalized.", nameof(axisDirection));
        if (height <= Fixed64.Zero)
            throw new ArgumentOutOfRangeException(nameof(height));
        if (radius < Fixed64.Zero)
            throw new ArgumentOutOfRangeException(nameof(radius));

        Signed192 axisLengthSquared = GetFiniteAxisLengthSquared(axisDirection);
        return FromMinMax(
            new Vector3d(
                GetCenteredFiniteConeBound(
                    center.X,
                    axisDirection.X,
                    height,
                    GetFiniteConeDiskExtent(axisLengthSquared, axisDirection.X, radius),
                    minimum: true),
                GetCenteredFiniteConeBound(
                    center.Y,
                    axisDirection.Y,
                    height,
                    GetFiniteConeDiskExtent(axisLengthSquared, axisDirection.Y, radius),
                    minimum: true),
                GetCenteredFiniteConeBound(
                    center.Z,
                    axisDirection.Z,
                    height,
                    GetFiniteConeDiskExtent(axisLengthSquared, axisDirection.Z, radius),
                    minimum: true)),
            new Vector3d(
                GetCenteredFiniteConeBound(
                    center.X,
                    axisDirection.X,
                    height,
                    GetFiniteConeDiskExtent(axisLengthSquared, axisDirection.X, radius),
                    minimum: false),
                GetCenteredFiniteConeBound(
                    center.Y,
                    axisDirection.Y,
                    height,
                    GetFiniteConeDiskExtent(axisLengthSquared, axisDirection.Y, radius),
                    minimum: false),
                GetCenteredFiniteConeBound(
                    center.Z,
                    axisDirection.Z,
                    height,
                    GetFiniteConeDiskExtent(axisLengthSquared, axisDirection.Z, radius),
                    minimum: false)));
    }

    private static Signed192 GetFiniteAxisLengthSquared(Vector3d axisDirection) =>
        WideGeometry.GetDifferenceDotProduct3D(
            axisDirection.X, Fixed64.Zero,
            axisDirection.Y, Fixed64.Zero,
            axisDirection.Z, Fixed64.Zero,
            axisDirection.X, Fixed64.Zero,
            axisDirection.Y, Fixed64.Zero,
            axisDirection.Z, Fixed64.Zero);

    private static Fixed64 GetFiniteConeDiskExtent(
        Signed192 axisLengthSquared,
        Fixed64 axisComponent,
        Fixed64 radius)
    {
        Fixed64 floor = GetFiniteDiskFloorExtent(
            axisLengthSquared,
            axisComponent,
            radius,
            out Signed192 capacity,
            out Signed320 radiusSquared);
        Signed576 represented = GetFiniteDiskRepresentedSquare(
            floor,
            axisLengthSquared);
        Signed576 target = GetFiniteDiskTarget(radiusSquared, capacity);
        return WideArithmetic.CompareNonNegative(represented, target) == 0
            ? floor
            : Fixed64.FromRaw(floor.m_rawValue + 1L);
    }

    private static Fixed64 GetFiniteDiskFloorExtent(
        Signed192 axisLengthSquared,
        Fixed64 axisComponent,
        Fixed64 radius,
        out Signed192 capacity,
        out Signed320 radiusSquared)
    {
        Signed192 componentSquared = WideGeometry.GetDifferenceDotProduct3D(
            axisComponent, Fixed64.Zero,
            Fixed64.Zero, Fixed64.Zero,
            Fixed64.Zero, Fixed64.Zero,
            axisComponent, Fixed64.Zero,
            Fixed64.Zero, Fixed64.Zero,
            Fixed64.Zero, Fixed64.Zero);
        capacity = WideArithmetic.SubtractSigned192(
            axisLengthSquared,
            componentSquared);
        Signed192 radiusRaw = Signed192.Signed(radius.m_rawValue);
        radiusSquared = WideArithmetic.MultiplySigned192(radiusRaw, radiusRaw);
        if (capacity.IsZero || radius == Fixed64.Zero)
            return Fixed64.Zero;
        if (WideArithmetic.CompareMagnitude(capacity, axisLengthSquared) == 0)
            return radius;

        Signed320 capacityTimesAxisLengthSquared = WideArithmetic.MultiplySigned192(
            capacity,
            axisLengthSquared);
        Signed576 radicand = WideArithmetic.MultiplySigned320(
            radiusSquared,
            capacityTimesAxisLengthSquared);
        Signed320 scaledRoot = WideArithmetic.GetFloorSquareRootScaledByFixed64(radicand);
        Signed320 scaledAxisLengthSquared = WideArithmetic.MultiplySigned192(
            axisLengthSquared,
            Signed192.Signed(FixedMath.ONE_L));
        _ = Fixed64.TryGetSignedRawRatio(
            Signed576.ExtendValue(scaledRoot),
            Signed576.ExtendValue(scaledAxisLengthSquared),
            out Fixed64 candidate);

        Signed576 represented = GetFiniteDiskRepresentedSquare(
            candidate,
            axisLengthSquared);
        Signed576 target = GetFiniteDiskTarget(radiusSquared, capacity);
        return WideArithmetic.CompareNonNegative(represented, target) <= 0
            ? candidate
            : Fixed64.FromRaw(candidate.m_rawValue - 1L);
    }

    private static Signed576 GetFiniteDiskRepresentedSquare(
        Fixed64 extent,
        Signed192 axisLengthSquared)
    {
        Signed192 extentRaw = Signed192.Signed(extent.m_rawValue);
        return WideArithmetic.MultiplySigned576(
            Signed576.ExtendValue(
                WideArithmetic.MultiplySigned192(extentRaw, extentRaw)),
            axisLengthSquared);
    }

    private static Signed576 GetFiniteDiskTarget(
        Signed320 radiusSquared,
        Signed192 capacity) =>
        WideArithmetic.MultiplySigned576(
            Signed576.ExtendValue(radiusSquared),
            capacity);

    private static Signed192 GetCenteredFiniteCylinderOutwardExtentRaw(
        Signed192 axisLengthSquared,
        Fixed64 axisComponent,
        Fixed64 axisLength,
        Fixed64 radius)
    {
        Signed320 axialNumerator = WideArithmetic.MultiplySigned192(
            Signed192.Signed(axisComponent.Abs().m_rawValue),
            Signed192.Signed(axisLength.m_rawValue));
        Fixed64 axialCeiling = GetPositiveCeilingRawRatio(
            axialNumerator,
            WideGeometry.CenteredAxisDenominator);
        Fixed64 diskFloor = GetFiniteDiskFloorExtent(
            axisLengthSquared,
            axisComponent,
            radius,
            out Signed192 capacity,
            out Signed320 radiusSquared);
        Signed192 candidate = WideArithmetic.AddSigned192(
            Signed192.Signed(axialCeiling.m_rawValue),
            Signed192.Signed(diskFloor.m_rawValue));

        // floor(disk) + ceil(axis) is at most one raw unit below the
        // combined ceiling. Decide that final carry from the exact radical.
        if (!IsCenteredFiniteCylinderExtentSufficient(
                candidate,
                axialNumerator,
                axisLengthSquared,
                capacity,
                radiusSquared))
        {
            candidate = WideArithmetic.AddSigned192(
                candidate,
                Signed192.Signed(1L));
        }

        return candidate;
    }

    private static Fixed64 GetPositiveCeilingRawRatio(
        Signed320 numerator,
        Signed192 denominator)
    {
        Fixed64 candidate = Fixed64.GetSignedRawRatio(numerator, denominator);
        Signed320 represented = WideArithmetic.MultiplySigned192(
            Signed192.Signed(candidate.m_rawValue),
            denominator);
        return WideArithmetic.SubtractSigned320(represented, numerator).Sign >= 0
            ? candidate
            : Fixed64.FromRaw(candidate.m_rawValue + 1L);
    }

    private static bool IsCenteredFiniteCylinderExtentSufficient(
        Signed192 candidate,
        Signed320 axialNumerator,
        Signed192 axisLengthSquared,
        Signed192 diskCapacity,
        Signed320 radiusSquared)
    {
        Signed320 remainder = WideArithmetic.SubtractSigned320(
            WideArithmetic.MultiplySigned192(
                candidate,
                WideGeometry.CenteredAxisDenominator),
            axialNumerator);
        Signed576 left = WideArithmetic.MultiplySigned576(
            WideArithmetic.MultiplySigned320(remainder, remainder),
            axisLengthSquared);
        Signed320 denominatorSquared = WideArithmetic.MultiplySigned192(
            WideGeometry.CenteredAxisDenominator,
            WideGeometry.CenteredAxisDenominator);
        Signed576 right = WideArithmetic.MultiplySigned576(
            WideArithmetic.MultiplySigned320(denominatorSquared, radiusSquared),
            diskCapacity);
        return WideArithmetic.CompareNonNegative(left, right) >= 0;
    }

    private static FixedBoundBox FromCenteredFiniteCylinderExtentsClippedToDomain(
        Vector3d center,
        Signed192 xExtent,
        Signed192 yExtent,
        Signed192 zExtent) =>
        FromMinMax(
            new Vector3d(
                GetCenteredFiniteCylinderBound(center.X, xExtent, minimum: true),
                GetCenteredFiniteCylinderBound(center.Y, yExtent, minimum: true),
                GetCenteredFiniteCylinderBound(center.Z, zExtent, minimum: true)),
            new Vector3d(
                GetCenteredFiniteCylinderBound(center.X, xExtent, minimum: false),
                GetCenteredFiniteCylinderBound(center.Y, yExtent, minimum: false),
                GetCenteredFiniteCylinderBound(center.Z, zExtent, minimum: false)));

    private static Fixed64 GetCenteredFiniteCylinderBound(
        Fixed64 center,
        Signed192 outwardExtent,
        bool minimum)
    {
        Signed192 centerRaw = Signed192.Signed(center.m_rawValue);
        Signed192 boundRaw = minimum
            ? WideArithmetic.SubtractSigned192(centerRaw, outwardExtent)
            : WideArithmetic.AddSigned192(centerRaw, outwardExtent);
        if (Fixed64.TryGetSignedRawRatio(
                Signed576.ExtendValue(
                    Signed320.ExtendValue(boundRaw)),
                Signed576.ExtendValue(
                    Signed320.ExtendValue(
                        Signed192.Signed(1L))),
                out Fixed64 bound))
        {
            return bound;
        }

        return boundRaw.Sign < 0 ? Fixed64.MinValue : Fixed64.MaxValue;
    }

    private static Fixed64 GetCenteredFiniteConeBound(
        Fixed64 center,
        Fixed64 axisComponent,
        Fixed64 height,
        Fixed64 diskExtent,
        bool minimum)
    {
        Signed320 centerNumerator = WideArithmetic.MultiplySigned192(
            Signed192.Signed(center.m_rawValue),
            WideGeometry.CenteredAxisDenominator);
        Signed320 axialNumerator = WideArithmetic.MultiplySigned192(
            Signed192.Signed(axisComponent.m_rawValue),
            Signed192.Signed(height.m_rawValue));
        Signed320 apexNumerator = WideArithmetic.AddSigned320(
            centerNumerator,
            axialNumerator);
        Signed320 baseNumerator = WideArithmetic.SubtractSigned320(
            centerNumerator,
            axialNumerator);
        Signed320 radialNumerator = WideArithmetic.MultiplySigned192(
            Signed192.Signed(diskExtent.m_rawValue),
            WideGeometry.CenteredAxisDenominator);
        Signed320 baseRimNumerator = minimum
            ? WideArithmetic.SubtractSigned320(baseNumerator, radialNumerator)
            : WideArithmetic.AddSigned320(baseNumerator, radialNumerator);
        Signed320 boundNumerator =
            WideArithmetic.SubtractSigned320(apexNumerator, baseRimNumerator).Sign switch
            {
                < 0 when minimum => apexNumerator,
                > 0 when !minimum => apexNumerator,
                _ => baseRimNumerator,
            };
        return WideGeometry.GetRationalBoundClippedToDomain(
            boundNumerator,
            WideGeometry.CenteredAxisDenominator,
            minimum);
    }

    private static FixedBoundBox FromCenteredFiniteAxisClippedToDomain(
        Vector3d center,
        Vector3d axisDirection,
        Fixed64 axisLength,
        Vector3d radialExtents) =>
        FromMinMax(
            new Vector3d(
                WideGeometry.GetCenteredFiniteAxisBoundClippedToDomain(
                    center.X, axisDirection.X, axisLength, radialExtents.X, minimum: true),
                WideGeometry.GetCenteredFiniteAxisBoundClippedToDomain(
                    center.Y, axisDirection.Y, axisLength, radialExtents.Y, minimum: true),
                WideGeometry.GetCenteredFiniteAxisBoundClippedToDomain(
                    center.Z, axisDirection.Z, axisLength, radialExtents.Z, minimum: true)),
            new Vector3d(
                WideGeometry.GetCenteredFiniteAxisBoundClippedToDomain(
                    center.X, axisDirection.X, axisLength, radialExtents.X, minimum: false),
                WideGeometry.GetCenteredFiniteAxisBoundClippedToDomain(
                    center.Y, axisDirection.Y, axisLength, radialExtents.Y, minimum: false),
                WideGeometry.GetCenteredFiniteAxisBoundClippedToDomain(
                    center.Z, axisDirection.Z, axisLength, radialExtents.Z, minimum: false)));

    private static void ValidateCenteredFiniteAxis(
        Vector3d axisDirection,
        Fixed64 axisLength,
        Fixed64 radius,
        bool requirePositiveLength)
    {
        if (!axisDirection.IsNormalized())
            throw new ArgumentException("Finite-axis direction must be normalized.", nameof(axisDirection));
        if (requirePositiveLength ? axisLength <= Fixed64.Zero : axisLength < Fixed64.Zero)
            throw new ArgumentOutOfRangeException(nameof(axisLength));
        if (radius < Fixed64.Zero)
            throw new ArgumentOutOfRangeException(nameof(radius));
    }
}
