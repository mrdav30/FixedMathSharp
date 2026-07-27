//=======================================================================
// FixedBoundBox.RigidFiniteAxis.cs
//=======================================================================
// MIT License, Copyright (c) 2024-present David Oravsky (mrdav30)
// See LICENSE file in the project root for full license information.
//=======================================================================

using System;

namespace FixedMathSharp.Bounds;

/// <content>
/// Provides high-precision, allocation-free bounding box computations for
/// rigidly-rotated finite-axis shapes (e.g. capsules) by working directly
/// with extended-precision axis components instead of materializing the
/// rotated axis vector.
/// </content>
public partial struct FixedBoundBox
{
    /// <summary>
    /// Creates the representable-domain intersection of a centered capsule's
    /// tight axis-aligned bounds without materializing its rotated axis.
    /// </summary>
    public static FixedBoundBox FromCenteredCapsuleClippedToDomain(
        Vector3d center,
        FixedQuaternion rotation,
        Vector3d localAxisDirection,
        Fixed64 axisLength,
        Fixed64 radius)
    {
        ValidateRigidFiniteAxis(
            rotation,
            localAxisDirection,
            axisLength,
            radius,
            requirePositiveLength: false);
        GetRigidAxis(
            rotation,
            localAxisDirection,
            out Signed192 axisX,
            out Signed192 axisY,
            out Signed192 axisZ,
            out _,
            out Signed192 centeredDenominator);
        return FromMinMax(
            new Vector3d(
                GetRigidCapsuleBound(
                    center.X,
                    axisX,
                    axisLength,
                    radius,
                    centeredDenominator,
                    minimum: true),
                GetRigidCapsuleBound(
                    center.Y,
                    axisY,
                    axisLength,
                    radius,
                    centeredDenominator,
                    minimum: true),
                GetRigidCapsuleBound(
                    center.Z,
                    axisZ,
                    axisLength,
                    radius,
                    centeredDenominator,
                    minimum: true)),
            new Vector3d(
                GetRigidCapsuleBound(
                    center.X,
                    axisX,
                    axisLength,
                    radius,
                    centeredDenominator,
                    minimum: false),
                GetRigidCapsuleBound(
                    center.Y,
                    axisY,
                    axisLength,
                    radius,
                    centeredDenominator,
                    minimum: false),
                GetRigidCapsuleBound(
                    center.Z,
                    axisZ,
                    axisLength,
                    radius,
                    centeredDenominator,
                    minimum: false)));
    }

    /// <summary>
    /// Creates the representable-domain intersection of a centered finite
    /// cylinder's tight axis-aligned bounds without materializing its rotated
    /// axis.
    /// </summary>
    public static FixedBoundBox FromCenteredFiniteCylinderClippedToDomain(
        Vector3d center,
        FixedQuaternion rotation,
        Vector3d localAxisDirection,
        Fixed64 axisLength,
        Fixed64 radius)
    {
        ValidateRigidFiniteAxis(
            rotation,
            localAxisDirection,
            axisLength,
            radius,
            requirePositiveLength: true);
        GetRigidAxis(
            rotation,
            localAxisDirection,
            out Signed192 axisX,
            out Signed192 axisY,
            out Signed192 axisZ,
            out Signed320 axisSquared,
            out Signed192 centeredDenominator);
        return FromCenteredFiniteCylinderExtentsClippedToDomain(
            center,
            GetRigidCylinderOutwardExtentRaw(
                axisSquared,
                axisX,
                axisLength,
                radius,
                centeredDenominator),
            GetRigidCylinderOutwardExtentRaw(
                axisSquared,
                axisY,
                axisLength,
                radius,
                centeredDenominator),
            GetRigidCylinderOutwardExtentRaw(
                axisSquared,
                axisZ,
                axisLength,
                radius,
                centeredDenominator));
    }

    /// <summary>
    /// Creates the representable-domain intersection of a centered finite
    /// cone's axis-aligned bounds without materializing its rotated axis.
    /// </summary>
    /// <param name="center">The midpoint between the base center and apex.</param>
    /// <param name="rotation">The cone rigid-frame rotation.</param>
    /// <param name="localAxisDirection">
    /// The normalized local direction from the base center toward the apex.
    /// </param>
    /// <param name="height">The positive full base-to-apex height.</param>
    /// <param name="radius">The nonnegative base radius.</param>
    public static FixedBoundBox FromCenteredFiniteConeClippedToDomain(
        Vector3d center,
        FixedQuaternion rotation,
        Vector3d localAxisDirection,
        Fixed64 height,
        Fixed64 radius)
    {
        ValidateRigidFiniteAxis(
            rotation,
            localAxisDirection,
            height,
            radius,
            requirePositiveLength: true);
        GetRigidAxis(
            rotation,
            localAxisDirection,
            out Signed192 axisX,
            out Signed192 axisY,
            out Signed192 axisZ,
            out Signed320 axisSquared,
            out Signed192 centeredDenominator);
        return FromMinMax(
            new Vector3d(
                GetRigidConeBound(
                    center.X,
                    axisSquared,
                    axisX,
                    height,
                    radius,
                    centeredDenominator,
                    minimum: true),
                GetRigidConeBound(
                    center.Y,
                    axisSquared,
                    axisY,
                    height,
                    radius,
                    centeredDenominator,
                    minimum: true),
                GetRigidConeBound(
                    center.Z,
                    axisSquared,
                    axisZ,
                    height,
                    radius,
                    centeredDenominator,
                    minimum: true)),
            new Vector3d(
                GetRigidConeBound(
                    center.X,
                    axisSquared,
                    axisX,
                    height,
                    radius,
                    centeredDenominator,
                    minimum: false),
                GetRigidConeBound(
                    center.Y,
                    axisSquared,
                    axisY,
                    height,
                    radius,
                    centeredDenominator,
                    minimum: false),
                GetRigidConeBound(
                    center.Z,
                    axisSquared,
                    axisZ,
                    height,
                    radius,
                    centeredDenominator,
                    minimum: false)));
    }

    private static void GetRigidAxis(
        FixedQuaternion rotation,
        Vector3d localAxisDirection,
        out Signed192 axisX,
        out Signed192 axisY,
        out Signed192 axisZ,
        out Signed320 axisSquared,
        out Signed192 centeredDenominator)
    {
        WideOrientedBox.GetRotatedLocalAxisNumerators(
            rotation,
            localAxisDirection,
            out axisX,
            out axisY,
            out axisZ,
            out Signed192 rotationDenominator);
        axisSquared = WideArithmetic.AddSigned320(
            WideArithmetic.AddSigned320(
                WideArithmetic.MultiplySigned192(axisX, axisX),
                WideArithmetic.MultiplySigned192(axisY, axisY)),
            WideArithmetic.MultiplySigned192(axisZ, axisZ));
        Signed320 centeredDenominatorWide = WideArithmetic.MultiplySigned192(
            rotationDenominator,
            WideGeometry.CenteredAxisDenominator);
        // A normalized quaternion's squared-component denominator needs at
        // most 68 signed bits. The 34-bit centered Q32.32 factor therefore
        // keeps this exact product within Signed192.
        _ = Signed192.TryNarrowSigned(
            centeredDenominatorWide,
            out centeredDenominator);
    }

    private static Fixed64 GetRigidCapsuleBound(
        Fixed64 center,
        Signed192 axisComponent,
        Fixed64 axisLength,
        Fixed64 radius,
        Signed192 centeredDenominator,
        bool minimum)
    {
        Signed320 centerNumerator = WideArithmetic.MultiplySigned192(
            Signed192.Signed(center.m_rawValue),
            centeredDenominator);
        Signed320 axialNumerator = WideArithmetic.MultiplySigned192(
            GetMagnitude(axisComponent),
            Signed192.Signed(axisLength.m_rawValue));
        Signed320 radialNumerator = WideArithmetic.MultiplySigned192(
            Signed192.Signed(radius.m_rawValue),
            centeredDenominator);
        Signed320 extentNumerator = WideArithmetic.AddSigned320(
            axialNumerator,
            radialNumerator);
        Signed320 boundNumerator = minimum
            ? WideArithmetic.SubtractSigned320(
                centerNumerator,
                extentNumerator)
            : WideArithmetic.AddSigned320(
                centerNumerator,
                extentNumerator);
        return GetRigidRationalBoundClippedToDomain(
            boundNumerator,
            centeredDenominator,
            minimum);
    }

    private static Signed192 GetRigidCylinderOutwardExtentRaw(
        Signed320 axisSquared,
        Signed192 axisComponent,
        Fixed64 axisLength,
        Fixed64 radius,
        Signed192 centeredDenominator)
    {
        Signed320 axialNumerator = WideArithmetic.MultiplySigned192(
            GetMagnitude(axisComponent),
            Signed192.Signed(axisLength.m_rawValue));
        Fixed64 axialCeiling = GetRigidPositiveCeilingRawRatio(
            axialNumerator,
            centeredDenominator);
        Fixed64 diskFloor = GetRigidDiskFloorExtent(
            axisSquared,
            axisComponent,
            radius,
            out Signed320 capacity,
            out Signed320 radiusSquared);
        Signed192 candidate = WideArithmetic.AddSigned192(
            Signed192.Signed(axialCeiling.m_rawValue),
            Signed192.Signed(diskFloor.m_rawValue));
        if (!IsRigidCylinderExtentSufficient(
                candidate,
                axialNumerator,
                centeredDenominator,
                axisSquared,
                capacity,
                radiusSquared))
        {
            candidate = WideArithmetic.AddSigned192(
                candidate,
                Signed192.Signed(1L));
        }

        return candidate;
    }

    private static Fixed64 GetRigidPositiveCeilingRawRatio(
        Signed320 numerator,
        Signed192 denominator)
    {
        _ = Fixed64.TryGetSignedRawRatio(
            Signed576.ExtendValue(numerator),
            Signed576.ExtendValue(
                Signed320.ExtendValue(denominator)),
            out Fixed64 candidate);
        Signed320 represented = WideArithmetic.MultiplySigned192(
            Signed192.Signed(candidate.m_rawValue),
            denominator);
        return WideArithmetic.SubtractSigned320(
                represented,
                numerator).Sign >= 0
            ? candidate
            : Fixed64.FromRaw(candidate.m_rawValue + 1L);
    }

    private static Fixed64 GetRigidDiskFloorExtent(
        Signed320 axisSquared,
        Signed192 axisComponent,
        Fixed64 radius,
        out Signed320 capacity,
        out Signed320 radiusSquared)
    {
        capacity = WideArithmetic.SubtractSigned320(
            axisSquared,
            WideArithmetic.MultiplySigned192(
                axisComponent,
                axisComponent));
        Signed192 radiusRaw =
            Signed192.Signed(radius.m_rawValue);
        radiusSquared = WideArithmetic.MultiplySigned192(
            radiusRaw,
            radiusRaw);
        if (capacity.IsZero || radius == Fixed64.Zero)
            return Fixed64.Zero;
        if (WideArithmetic.SubtractSigned320(
                capacity,
                axisSquared).IsZero)
        {
            return radius;
        }

        Signed704 radicandWide = WideArithmetic.MultiplySigned320(
            radiusSquared,
            capacity,
            axisSquared);
        // Rotation preserves the normalized local-axis squared length:
        // axisSquared and capacity are each below 2^193. Combined with the
        // full-domain radius square, this radicand remains below 2^523.
        Signed576 radicand = new(
            radicandWide.Word8,
            radicandWide.Word7,
            radicandWide.Word6,
            radicandWide.Word5,
            radicandWide.Word4,
            radicandWide.Word3,
            radicandWide.Word2,
            radicandWide.Word1,
            radicandWide.Word0);
        Signed320 scaledRoot =
            WideArithmetic.GetFloorSquareRootScaledByFixed64(radicand);
        Signed576 scaledAxisSquared = WideArithmetic.MultiplySigned320(
            axisSquared,
            Signed320.ExtendValue(
                Signed192.Signed(FixedMath.ONE_L)));
        _ = Fixed64.TryGetSignedRawRatio(
            Signed576.ExtendValue(scaledRoot),
            scaledAxisSquared,
            out Fixed64 candidate);
        Signed576 represented = GetRigidDiskRepresentedSquare(
            candidate,
            axisSquared);
        Signed576 target = WideArithmetic.MultiplySigned320(
            radiusSquared,
            capacity);
        return WideArithmetic.CompareNonNegative(represented, target) <= 0
            ? candidate
            : Fixed64.FromRaw(candidate.m_rawValue - 1L);
    }

    private static Signed576 GetRigidDiskRepresentedSquare(
        Fixed64 extent,
        Signed320 axisSquared) =>
        WideArithmetic.MultiplySigned320(
            WideArithmetic.MultiplySigned192(
                Signed192.Signed(extent.m_rawValue),
                Signed192.Signed(extent.m_rawValue)),
            axisSquared);

    private static bool IsRigidCylinderExtentSufficient(
        Signed192 candidate,
        Signed320 axialNumerator,
        Signed192 centeredDenominator,
        Signed320 axisSquared,
        Signed320 diskCapacity,
        Signed320 radiusSquared)
    {
        Signed320 remainder = WideArithmetic.SubtractSigned320(
            WideArithmetic.MultiplySigned192(
                candidate,
                centeredDenominator),
            axialNumerator);
        Signed704 left = WideArithmetic.MultiplySigned320(
            remainder,
            remainder,
            axisSquared);
        Signed320 denominatorSquared = WideArithmetic.MultiplySigned192(
            centeredDenominator,
            centeredDenominator);
        Signed704 right = WideArithmetic.MultiplySigned320(
            denominatorSquared,
            radiusSquared,
            diskCapacity);
        return WideArithmetic.CompareNonNegative(left, right) >= 0;
    }

    private static Fixed64 GetRigidConeBound(
        Fixed64 center,
        Signed320 axisSquared,
        Signed192 axisComponent,
        Fixed64 height,
        Fixed64 radius,
        Signed192 centeredDenominator,
        bool minimum)
    {
        Signed320 centerNumerator = WideArithmetic.MultiplySigned192(
            Signed192.Signed(center.m_rawValue),
            centeredDenominator);
        Signed320 axialNumerator = WideArithmetic.MultiplySigned192(
            axisComponent,
            Signed192.Signed(height.m_rawValue));
        Signed320 apexNumerator = WideArithmetic.AddSigned320(
            centerNumerator,
            axialNumerator);
        Fixed64 apexBound = GetRigidRationalBoundClippedToDomain(
            apexNumerator,
            centeredDenominator,
            minimum);
        Fixed64 baseRimBound = GetRigidConeBaseRimBound(
            center,
            axisSquared,
            axisComponent,
            height,
            radius,
            centeredDenominator,
            minimum);
        return minimum
            ? apexBound <= baseRimBound ? apexBound : baseRimBound
            : apexBound >= baseRimBound ? apexBound : baseRimBound;
    }

    private static Fixed64 GetRigidConeBaseRimBound(
        Fixed64 center,
        Signed320 axisSquared,
        Signed192 axisComponent,
        Fixed64 height,
        Fixed64 radius,
        Signed192 centeredDenominator,
        bool minimum)
    {
        Signed320 baseOffsetNumerator = WideArithmetic.SubtractSigned320(
            default,
            WideArithmetic.MultiplySigned192(
                axisComponent,
                Signed192.Signed(height.m_rawValue)));
        Fixed64 baseOffset = GetRigidSignedRawRatio(
            baseOffsetNumerator,
            centeredDenominator,
            roundDown: minimum);
        Fixed64 diskFloor = GetRigidDiskFloorExtent(
            axisSquared,
            axisComponent,
            radius,
            out Signed320 capacity,
            out Signed320 radiusSquared);
        Signed192 candidate = WideArithmetic.AddSigned192(
            Signed192.Signed(center.m_rawValue),
            Signed192.Signed(baseOffset.m_rawValue));
        candidate = minimum
            ? WideArithmetic.SubtractSigned192(
                candidate,
                Signed192.Signed(diskFloor.m_rawValue))
            : WideArithmetic.AddSigned192(
                candidate,
                Signed192.Signed(diskFloor.m_rawValue));
        if (!IsRigidConeBaseRimBoundSufficient(
                candidate,
                center,
                baseOffsetNumerator,
                centeredDenominator,
                axisSquared,
                capacity,
                radiusSquared,
                minimum))
        {
            candidate = minimum
                ? WideArithmetic.SubtractSigned192(
                    candidate,
                    Signed192.Signed(1L))
                : WideArithmetic.AddSigned192(
                    candidate,
                    Signed192.Signed(1L));
        }

        return GetRigidRationalBoundClippedToDomain(
            WideArithmetic.MultiplySigned192(
                candidate,
                centeredDenominator),
            centeredDenominator,
            minimum);
    }

    private static Fixed64 GetRigidSignedRawRatio(
        Signed320 numerator,
        Signed192 denominator,
        bool roundDown)
    {
        _ = Fixed64.TryGetSignedRawRatio(
            Signed576.ExtendValue(numerator),
            Signed576.ExtendValue(
                Signed320.ExtendValue(denominator)),
            out Fixed64 candidate);
        Signed320 represented = WideArithmetic.MultiplySigned192(
            Signed192.Signed(candidate.m_rawValue),
            denominator);
        int comparison = WideArithmetic.SubtractSigned320(
            represented,
            numerator).Sign;
        if (roundDown && comparison > 0)
            return Fixed64.FromRaw(candidate.m_rawValue - 1L);
        if (!roundDown && comparison < 0)
            return Fixed64.FromRaw(candidate.m_rawValue + 1L);
        return candidate;
    }

    private static bool IsRigidConeBaseRimBoundSufficient(
        Signed192 candidate,
        Fixed64 center,
        Signed320 baseOffsetNumerator,
        Signed192 centeredDenominator,
        Signed320 axisSquared,
        Signed320 diskCapacity,
        Signed320 radiusSquared,
        bool minimum)
    {
        Signed192 candidateFromCenter = WideArithmetic.SubtractSigned192(
            candidate,
            Signed192.Signed(center.m_rawValue));
        Signed320 representedOffset = WideArithmetic.MultiplySigned192(
            candidateFromCenter,
            centeredDenominator);
        Signed320 radialGap = minimum
            ? WideArithmetic.SubtractSigned320(
                baseOffsetNumerator,
                representedOffset)
            : WideArithmetic.SubtractSigned320(
                representedOffset,
                baseOffsetNumerator);

        Signed704 left = WideArithmetic.MultiplySigned320(
            radialGap,
            radialGap,
            axisSquared);
        Signed320 denominatorSquared = WideArithmetic.MultiplySigned192(
            centeredDenominator,
            centeredDenominator);
        Signed704 right = WideArithmetic.MultiplySigned320(
            denominatorSquared,
            radiusSquared,
            diskCapacity);
        return WideArithmetic.CompareNonNegative(left, right) >= 0;
    }

    private static Fixed64 GetRigidRationalBoundClippedToDomain(
        Signed320 numerator,
        Signed192 denominator,
        bool minimum)
    {
        Signed320 minimumNumerator = WideArithmetic.MultiplySigned192(
            Signed192.Signed(Fixed64.MinValue.m_rawValue),
            denominator);
        Signed320 maximumNumerator = WideArithmetic.MultiplySigned192(
            Signed192.Signed(Fixed64.MaxValue.m_rawValue),
            denominator);
        if (WideArithmetic.SubtractSigned320(
                numerator,
                minimumNumerator).Sign < 0)
        {
            return Fixed64.MinValue;
        }
        if (WideArithmetic.SubtractSigned320(
                numerator,
                maximumNumerator).Sign > 0)
        {
            return Fixed64.MaxValue;
        }

        _ = Fixed64.TryGetSignedRawRatio(
            Signed576.ExtendValue(numerator),
            Signed576.ExtendValue(
                Signed320.ExtendValue(denominator)),
            out Fixed64 bound);
        Signed320 represented = WideArithmetic.MultiplySigned192(
            Signed192.Signed(bound.m_rawValue),
            denominator);
        int comparison = WideArithmetic.SubtractSigned320(
            represented,
            numerator).Sign;
        if (minimum && comparison > 0)
            return Fixed64.FromRaw(bound.m_rawValue - 1L);
        if (!minimum && comparison < 0)
            return Fixed64.FromRaw(bound.m_rawValue + 1L);
        return bound;
    }

    private static Signed192 GetMagnitude(Signed192 value)
    {
        if (value.Sign >= 0)
            return value;
        return WideArithmetic.SubtractSigned192(default, value);
    }

    private static void ValidateRigidFiniteAxis(
        FixedQuaternion rotation,
        Vector3d localAxisDirection,
        Fixed64 axisLength,
        Fixed64 radius,
        bool requirePositiveLength)
    {
        if (!rotation.IsNormalized())
        {
            throw new ArgumentException(
                "Finite-axis rotation must be normalized.",
                nameof(rotation));
        }
        if (!localAxisDirection.IsNormalized())
        {
            throw new ArgumentException(
                "Finite-axis local direction must be normalized.",
                nameof(localAxisDirection));
        }
        if (requirePositiveLength
            ? axisLength <= Fixed64.Zero
            : axisLength < Fixed64.Zero)
        {
            throw new ArgumentOutOfRangeException(nameof(axisLength));
        }
        if (radius < Fixed64.Zero)
            throw new ArgumentOutOfRangeException(nameof(radius));
    }
}
