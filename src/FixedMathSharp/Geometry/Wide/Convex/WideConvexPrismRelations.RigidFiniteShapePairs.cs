//=======================================================================
// WideConvexPrismRelations.RigidFiniteShapePairs.cs
//=======================================================================
// MIT License, Copyright (c) 2024-present David Oravsky (mrdav30)
// See LICENSE file in the project root for full license information.
//=======================================================================

using System;

namespace FixedMathSharp.Geometry;

/// <content>
/// Contains high-precision contact resolution routines for pairs of rigid, finite
/// convex prism-based shapes (e.g. cylinders and capsules) using exact rational
/// (Signed192) axis representations to avoid precision loss during rotation.
/// </content>
internal static partial class WideConvexPrismRelations
{
    internal static bool TryGetCenteredFiniteCylinderCapsuleContact(
        Vector3d cylinderCenter,
        FixedQuaternion cylinderRotation,
        Vector3d cylinderLocalAxis,
        Fixed64 cylinderLength,
        Fixed64 cylinderRadius,
        Vector3d capsuleCenter,
        FixedQuaternion capsuleRotation,
        Vector3d capsuleLocalAxis,
        Fixed64 capsuleLength,
        Fixed64 capsuleRadius,
        out FixedContactAnchors contact) =>
        TryGetCenteredFiniteCylinderCapsuleContact(
            cylinderCenter,
            cylinderRotation,
            cylinderLocalAxis,
            cylinderLength,
            cylinderRadius,
            capsuleCenter,
            capsuleRotation,
            capsuleLocalAxis,
            capsuleLength,
            capsuleRadius,
            out contact,
            out _);

    internal static bool TryGetCenteredFiniteCylinderCapsuleContact(
        Vector3d cylinderCenter,
        FixedQuaternion cylinderRotation,
        Vector3d cylinderLocalAxis,
        Fixed64 cylinderLength,
        Fixed64 cylinderRadius,
        Vector3d capsuleCenter,
        FixedQuaternion capsuleRotation,
        Vector3d capsuleLocalAxis,
        Fixed64 capsuleLength,
        Fixed64 capsuleRadius,
        out FixedContactAnchors contact,
        out bool usedWideCandidate)
    {
        usedWideCandidate = false;
        WideOrientedBox.GetRotatedLocalAxisNumerators(
            cylinderRotation,
            cylinderLocalAxis,
            out Signed192 cylinderAxisX,
            out Signed192 cylinderAxisY,
            out Signed192 cylinderAxisZ,
            out Signed192 cylinderDenominator);
        WideOrientedBox.GetRotatedLocalAxisNumerators(
            capsuleRotation,
            capsuleLocalAxis,
            out Signed192 capsuleAxisX,
            out Signed192 capsuleAxisY,
            out Signed192 capsuleAxisZ,
            out Signed192 capsuleDenominator);
        var cylinderAxis = new RigidAxis3(
            cylinderAxisX,
            cylinderAxisY,
            cylinderAxisZ,
            cylinderDenominator);
        var capsuleAxis = new RigidAxis3(
            capsuleAxisX,
            capsuleAxisY,
            capsuleAxisZ,
            capsuleDenominator);
        Axis3 cylinderCandidate = cylinderAxis.ToWide();
        Axis3 capsuleCandidate = capsuleAxis.ToWide();
        var best = default(CylinderCapsulePenetration);
        if (!TryKeepCylinderCapsuleAxis(
                cylinderCandidate,
                cylinderCenter,
                cylinderAxis,
                cylinderLength,
                cylinderRadius,
                capsuleCenter,
                capsuleAxis,
                capsuleLength,
                capsuleRadius,
                ref best)
            || !TryKeepCylinderCapsuleAxis(
                capsuleCandidate,
                cylinderCenter,
                cylinderAxis,
                cylinderLength,
                cylinderRadius,
                capsuleCenter,
                capsuleAxis,
                capsuleLength,
                capsuleRadius,
                ref best)
            || !TryKeepCylinderCapsuleAxis(
                Cross(cylinderCandidate, capsuleCandidate),
                cylinderCenter,
                cylinderAxis,
                cylinderLength,
                cylinderRadius,
                capsuleCenter,
                capsuleAxis,
                capsuleLength,
                capsuleRadius,
                ref best))
        {
            contact = default;
            return false;
        }
        Axis3 centerDifference = GetCenterDifferenceAxis(
            cylinderCenter,
            capsuleCenter);
        if (!TryKeepCylinderCapsuleAxis(
                centerDifference,
                cylinderCenter,
                cylinderAxis,
                cylinderLength,
                cylinderRadius,
                capsuleCenter,
                capsuleAxis,
                capsuleLength,
                capsuleRadius,
                ref best))
        {
            contact = default;
            return false;
        }
        if (!WideOrientedBox
            .TryKeepCenteredRigidCylinderCapsuleClosestAxis(
                cylinderCenter,
                cylinderRotation,
                cylinderLocalAxis,
                cylinderLength,
                cylinderRadius,
                capsuleCenter,
                capsuleRotation,
                capsuleLocalAxis,
                capsuleLength,
                capsuleRadius,
                ref best))
        {
            contact = default;
            return false;
        }

        Vector3d normal;
        Fixed64 depth;
        bool depthIsClamped;
        if (best.IsWide)
        {
            normal = best.WideNormal;
            depth = best.WideDepth;
            depthIsClamped = best.WideDepthIsClamped;
        }
        else
        {
            Axis3 orientedAxis = best.Negate
                ? new Axis3(
                     WideArithmetic.Negate(best.Axis.X),
                     WideArithmetic.Negate(best.Axis.Y),
                     WideArithmetic.Negate(best.Axis.Z))
                : best.Axis;
            normal = WideNormalization.GetNormalized(
                Signed576.ExtendValue(orientedAxis.X),
                Signed576.ExtendValue(orientedAxis.Y),
                Signed576.ExtendValue(orientedAxis.Z));
            GetRoundedCylinderCapsuleDepth(
                best.ExactDepth,
                capsuleRadius,
                out depth,
                out depthIsClamped);
        }
        FixedPointAnchor cylinderAnchor =
            WideGeometry.GetCenteredCylinderSupportAnchor(
                cylinderCenter,
                cylinderRotation,
                cylinderLocalAxis,
                cylinderLength,
                cylinderRadius,
                normal);
        FixedPointAnchor capsuleAnchor =
            WideGeometry.GetCenteredCapsuleSupportAnchor(
                capsuleCenter,
                capsuleRotation,
                capsuleLocalAxis,
                capsuleLength,
                capsuleRadius,
                -normal);
        contact = new FixedContactAnchors(
            cylinderAnchor,
            capsuleAnchor,
            normal,
            depth,
            depthIsClamped);
        usedWideCandidate = best.IsWide;
        return true;
    }

    private static bool TryKeepCylinderCapsuleAxis(
        Axis3 axis,
        Vector3d cylinderCenter,
        RigidAxis3 cylinderAxis,
        Fixed64 cylinderLength,
        Fixed64 cylinderRadius,
        Vector3d capsuleCenter,
        RigidAxis3 capsuleAxis,
        Fixed64 capsuleLength,
        Fixed64 capsuleRadius,
        ref CylinderCapsulePenetration best)
    {
        if (axis.IsZero)
            return true;

        Signed576 cylinderAlignment =
            GetAxisProjection(axis, cylinderAxis);
        Signed576 capsuleAlignment =
            GetAxisProjection(axis, capsuleAxis);
        Signed576 centerProjection = GetDifferenceProjection(
            capsuleCenter,
            cylinderCenter,
            axis);
        Signed576 cylinderAxial = WideArithmetic.MultiplySigned576(
            WideArithmetic.MultiplySigned576(
                GetMagnitude576(cylinderAlignment),
                Signed192.Raw(cylinderLength)),
            capsuleAxis.RotationDenominator);
        Signed576 capsuleAxial = WideArithmetic.MultiplySigned576(
            WideArithmetic.MultiplySigned576(
                GetMagnitude576(capsuleAlignment),
                Signed192.Raw(capsuleLength)),
            cylinderAxis.RotationDenominator);
        Signed576 scaledCenter = WideArithmetic.MultiplySigned576(
            WideArithmetic.MultiplySigned576(
                WideArithmetic.MultiplySigned576(
                    GetMagnitude576(centerProjection),
                    cylinderAxis.RotationDenominator),
                capsuleAxis.RotationDenominator),
            Signed192.Raw(Fixed64.Two));
        Signed576 rational = WideArithmetic.SubtractSigned576(
            WideArithmetic.AddSigned576(
                cylinderAxial,
                capsuleAxial),
            scaledCenter);
        Signed576 axisSquared = GetAxisSquared(axis);
        Signed320 cylinderAxisSquared =
            GetAxisSquared(cylinderAxis);
        Signed832 planeSquared = GetPlaneSquared(
            axisSquared,
            cylinderAxisSquared,
            cylinderAlignment);
        Signed576 commonWide = WideArithmetic.MultiplySigned576(
            Signed576.ExtendValue(
                WideArithmetic.MultiplySigned192(
                    cylinderAxis.RotationDenominator,
                    capsuleAxis.RotationDenominator)),
            Signed192.Raw(Fixed64.Two));
        _ = Signed192.TryNarrowSigned(
            commonWide,
            out Signed192 common);
        var exactDepth = new ProjectionDepth(
            Signed704.ExtendValue(rational),
            common,
            cylinderRadius,
            RadialKind.Disk,
            axisSquared,
            cylinderAxisSquared,
            planeSquared);
        if (!IsCylinderCapsuleProjectionNonNegative(
                exactDepth,
                capsuleRadius))
        {
            return false;
        }

        if (!best.HasValue
            || CompareSignedProjectionDepths(
                exactDepth,
                best.ExactDepth) < 0)
        {
            best = new CylinderCapsulePenetration(
                axis,
                centerProjection.Sign < 0,
                exactDepth);
        }
        return true;
    }

    private static Axis3 GetCenterDifferenceAxis(
        Vector3d start,
        Vector3d end) =>
        new(
            Signed320.ExtendValue(
                WideArithmetic.SubtractSigned192(
                    Signed192.Raw(end.X),
                    Signed192.Raw(start.X))),
            Signed320.ExtendValue(
                WideArithmetic.SubtractSigned192(
                    Signed192.Raw(end.Y),
                    Signed192.Raw(start.Y))),
            Signed320.ExtendValue(
                WideArithmetic.SubtractSigned192(
                    Signed192.Raw(end.Z),
                    Signed192.Raw(start.Z))));

    private static bool IsCylinderCapsuleProjectionNonNegative(
        in ProjectionDepth baseDepth,
        Fixed64 capsuleRadius)
    {
        if (IsProjectionNonNegative(baseDepth))
            return true;

        Span<ulong> disk = stackalloc ulong[40];
        Span<ulong> capsule = stackalloc ulong[40];
        Span<ulong> rational = stackalloc ulong[40];
        Span<ulong> zero = stackalloc ulong[40];
        BuildLocalDiskRadicand(baseDepth, disk);
        var capsuleDepth = new ProjectionDepth(
            default,
            baseDepth.Common,
            capsuleRadius,
            RadialKind.Capsule,
            baseDepth.AxisSquared,
            baseDepth.ShapeAxisSquared,
            default);
        BuildThresholdCapsuleRadicand(capsuleDepth, capsule);
        BuildProduct(
            baseDepth.Rational,
            baseDepth.Rational,
            baseDepth.ShapeAxisSquared,
            rational);
        zero.Clear();
        return CompareRadicalPairs(
            disk,
            capsule,
            rational,
            zero) >= 0;
    }

    private static void GetRoundedCylinderCapsuleDepth(
        in ProjectionDepth baseDepth,
        Fixed64 capsuleRadius,
        out Fixed64 depth,
        out bool isClamped)
    {
        if (!TryGetCylinderCapsuleDepthApproximation(
                baseDepth,
                capsuleRadius,
                out Fixed64 approximation))
        {
            Signed192 maximumTwiceRaw = new(
                0UL,
                0UL,
                unchecked((ulong)long.MaxValue << 1));
            int maximumComparison =
                CompareCylinderCapsuleDepthToTwiceRaw(
                    baseDepth,
                    capsuleRadius,
                    maximumTwiceRaw);
            if (maximumComparison >= 0)
            {
                depth = Fixed64.MaxValue;
                isClamped = maximumComparison > 0;
                return;
            }
            depth = GetRoundedCylinderCapsuleDepthBySearch(
                baseDepth,
                capsuleRadius);
            isClamped = false;
            return;
        }
        // Each scalar approximation is rounded once, so the combined estimate
        // is only a few raw units from the exact projection. Correct those
        // units directly with the exact midpoint comparator.
        while (true)
        {
            if (approximation == Fixed64.MaxValue)
            {
                Signed192 maximumTwiceRaw = new(
                    0UL,
                    0UL,
                    unchecked((ulong)long.MaxValue << 1));
                depth = GetRoundedCylinderCapsuleDepthBySearch(
                    baseDepth,
                    capsuleRadius);
                isClamped =
                    CompareCylinderCapsuleDepthToTwiceRaw(
                        baseDepth,
                        capsuleRadius,
                        maximumTwiceRaw) > 0;
                return;
            }

            if (approximation > Fixed64.Zero)
            {
                Signed192 lowerMidpoint = new(
                    0UL,
                    0UL,
                    unchecked((ulong)(
                        approximation.m_rawValue
                        + approximation.m_rawValue
                        - 1L)));
                int lowerComparison =
                    CompareCylinderCapsuleDepthToTwiceRaw(
                        baseDepth,
                        capsuleRadius,
                        lowerMidpoint);
                if (lowerComparison
                    < (approximation.m_rawValue & 1L))
                {
                    approximation = Fixed64.FromRaw(
                        approximation.m_rawValue - 1L);
                    continue;
                }
            }

            Signed192 upperMidpoint = new(
                0UL,
                0UL,
                unchecked((ulong)approximation.m_rawValue << 1) | 1UL);
            int upperComparison =
                CompareCylinderCapsuleDepthToTwiceRaw(
                    baseDepth,
                    capsuleRadius,
                    upperMidpoint);
            if (upperComparison
                + (approximation.m_rawValue & 1L) > 0)
            {
                approximation = Fixed64.FromRaw(
                    approximation.m_rawValue + 1L);
                continue;
            }

            depth = approximation;
            isClamped = false;
            return;
        }
    }

    private static bool TryGetCylinderCapsuleDepthApproximation(
        in ProjectionDepth baseDepth,
        Fixed64 capsuleRadius,
        out Fixed64 approximation)
    {
        Signed576 axisLength = WideArithmetic.GetFloorSquareRoot(
            Signed704.ExtendValue(baseDepth.AxisSquared));
        Signed576 rationalDenominator = WideArithmetic.MultiplySigned576(
            axisLength,
            baseDepth.Common);
        if (!Fixed64.TryGetSignedRawRatio(
                baseDepth.Rational,
                Signed704.ExtendValue(rationalDenominator),
                out Fixed64 rational))
        {
            approximation = default;
            return false;
        }
        Fixed64 radial =
            GetDiskDepthApproximation(baseDepth, axisLength);
        Signed192 combined = WideArithmetic.AddSigned192(
            WideArithmetic.AddSigned192(
                Signed192.Raw(rational),
                Signed192.Raw(radial)),
            Signed192.Raw(capsuleRadius));
        if (combined.Sign <= 0)
        {
            approximation = Fixed64.Zero;
            return true;
        }
        // Any upper word or low sign bit exceeds the nonnegative Fixed64 raw domain.
        if ((combined.High
                | combined.Middle
                | (combined.Low >> 63)) != 0UL)
        {
            approximation = default;
            return false;
        }
        approximation = Fixed64.FromRaw((long)combined.Low);
        return true;
    }

    private static Fixed64 GetRoundedCylinderCapsuleDepthBySearch(
        in ProjectionDepth baseDepth,
        Fixed64 capsuleRadius)
    {
        Signed192 maximumTwiceRaw = new(
            0UL,
            0UL,
            unchecked((ulong)long.MaxValue << 1));
        if (CompareCylinderCapsuleDepthToTwiceRaw(
                baseDepth,
                capsuleRadius,
                maximumTwiceRaw) >= 0)
        {
            return Fixed64.MaxValue;
        }

        ulong low = 0UL;
        ulong high = 1UL << 63;
        // The nonnegative Fixed64 raw domain contains exactly 2^63 values.
        // This upper-bound search therefore completes in at most 64 steps.
        while (low < high)
        {
            ulong midpoint = low + ((high - low) >> 1);
            int comparison =
                CompareCylinderCapsuleDepthToTwiceRaw(
                    baseDepth,
                    capsuleRadius,
                    new Signed192(
                        0UL,
                        0UL,
                        midpoint << 1));
            if (comparison >= 0)
            {
                low = midpoint + 1UL;
            }
            else
            {
                high = midpoint;
            }
        }

        ulong floor = low - 1UL;
        int midpointComparison =
            CompareCylinderCapsuleDepthToTwiceRaw(
                baseDepth,
                capsuleRadius,
                new Signed192(
                    0UL,
                    0UL,
                    (floor << 1) | 1UL));
        return Fixed64.FromRaw(
            (long)(floor + GetNearestEvenIncrement(
                midpointComparison,
                floor)));
    }

    private static int CompareCylinderCapsuleDepthToTwiceRaw(
        in ProjectionDepth baseDepth,
        Fixed64 capsuleRadius,
        Signed192 twiceRaw)
    {
        Signed192 target = WideArithmetic.SubtractSigned192(
            twiceRaw,
            WideArithmetic.AddSigned192(
                Signed192.Raw(capsuleRadius),
                Signed192.Raw(capsuleRadius)));
        if (target.Sign >= 0)
        {
            return CompareProjectionDepthToTwiceRaw(
                baseDepth,
                target);
        }
        if (IsProjectionNonNegative(baseDepth))
            return 1;

        Signed192 positiveTarget =
            WideArithmetic.SubtractSigned192(default, target);
        Span<ulong> radial = stackalloc ulong[40];
        Span<ulong> threshold = stackalloc ulong[40];
        Span<ulong> rational = stackalloc ulong[40];
        Span<ulong> zero = stackalloc ulong[40];
        BuildLocalDiskRadicand(baseDepth, radial);
        ShiftLeft(radial, 2);
        Signed320 thresholdCoefficient =
            WideArithmetic.MultiplySigned192(
                baseDepth.Common,
                positiveTarget);
        BuildThresholdRadicand(
            thresholdCoefficient,
            baseDepth.ShapeAxisSquared,
            baseDepth.AxisSquared,
            threshold);
        BuildProduct(
            baseDepth.Rational,
            baseDepth.Rational,
            baseDepth.ShapeAxisSquared,
            rational);
        ShiftLeft(rational, 2);
        zero.Clear();
        return CompareRadicalPairs(
            radial,
            threshold,
            rational,
            zero);
    }

    private static Signed576 GetMagnitude576(Signed576 value) =>
        value.Sign >= 0
            ? value
            : WideArithmetic.SubtractSigned576(default, value);

    internal static bool TryKeepWideCylinderCapsuleAxis(
        WideCandidateAxis3 candidate,
        Vector3d cylinderCenter,
        FixedQuaternion cylinderRotation,
        Vector3d cylinderLocalAxis,
        Fixed64 cylinderLength,
        Fixed64 cylinderRadius,
        Vector3d capsuleCenter,
        FixedQuaternion capsuleRotation,
        Vector3d capsuleLocalAxis,
        Fixed64 capsuleLength,
        Fixed64 capsuleRadius,
        ref CylinderCapsulePenetration best)
    {
        WideOrientedBox.GetRotatedLocalAxisNumerators(
            cylinderRotation,
            cylinderLocalAxis,
            out Signed192 cylinderAxisX,
            out Signed192 cylinderAxisY,
            out Signed192 cylinderAxisZ,
            out Signed192 cylinderDenominator);
        WideOrientedBox.GetRotatedLocalAxisNumerators(
            capsuleRotation,
            capsuleLocalAxis,
            out Signed192 capsuleAxisX,
            out Signed192 capsuleAxisY,
            out Signed192 capsuleAxisZ,
            out Signed192 capsuleDenominator);
        var cylinderAxis = new RigidAxis3(
            cylinderAxisX,
            cylinderAxisY,
            cylinderAxisZ,
            cylinderDenominator);
        var capsuleAxis = new RigidAxis3(
            capsuleAxisX,
            capsuleAxisY,
            capsuleAxisZ,
            capsuleDenominator);
        if (!candidate.TryNarrow(out Axis3 axis))
        {
            return TryKeepWideCylinderCapsuleAxisFallback(
                candidate,
                cylinderCenter,
                cylinderAxis,
                cylinderLength,
                cylinderRadius,
                capsuleCenter,
                capsuleAxis,
                capsuleLength,
                capsuleRadius,
                ref best);
        }
        return TryKeepCylinderCapsuleAxis(
            axis,
            cylinderCenter,
            cylinderAxis,
            cylinderLength,
            cylinderRadius,
            capsuleCenter,
            capsuleAxis,
            capsuleLength,
            capsuleRadius,
            ref best);
    }
}
