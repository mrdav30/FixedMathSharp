//=======================================================================
// WideConvexPrismRelations.Projection.cs
//=======================================================================
// MIT License, Copyright (c) 2024–present David Oravsky (mrdav30)
// See LICENSE file in the project root for full license information.
//=======================================================================

using System;

namespace FixedMathSharp.Geometry;

/// <content>
/// Contains projection helpers for computing wide-precision separating-axis
/// projection depths of convex prism shapes (capsules, disks, cones) onto arbitrary axes.
/// </content>
internal static partial class WideConvexPrismRelations
{
    private static void GetShapeProjectionDepths(
        FiniteShapeKind shapeKind,
        Axis3 axis,
        RigidAxis3 shapeAxis,
        Fixed64 axisLength,
        Fixed64 radius,
        Signed576 prismMinimum,
        Signed576 prismMaximum,
        out ProjectionDepth positive,
        out ProjectionDepth negative)
    {
        Signed576 alignment = GetAxisProjection(axis, shapeAxis);
        Signed704 axial = WideArithmetic.MultiplySigned576ToSigned704(
            alignment,
            Signed320.ExtendValue(Signed192.Raw(axisLength)));
        Signed192 commonWithoutScale = WideArithmetic.AddSigned192(
            shapeAxis.RotationDenominator,
            shapeAxis.RotationDenominator);
        Signed320 commonWide = WideArithmetic.MultiplySigned192(
            commonWithoutScale,
            Scale);
        _ = Signed192.TryNarrowSigned(
            Signed576.ExtendValue(commonWide),
            out Signed192 common);
        Signed704 scaledMinimum =
            WideArithmetic.MultiplySigned576ToSigned704(
                prismMinimum,
                Signed320.ExtendValue(commonWithoutScale));
        Signed704 scaledMaximum =
            WideArithmetic.MultiplySigned576ToSigned704(
                prismMaximum,
                Signed320.ExtendValue(commonWithoutScale));
        Signed576 axisSquared = GetAxisSquared(axis);
        Signed320 shapeAxisSquared = GetAxisSquared(shapeAxis);

        if (shapeKind != FiniteShapeKind.Cone)
        {
            Signed704 axialMagnitude = GetMagnitude(axial);
            RadialKind radialKind = shapeKind == FiniteShapeKind.Capsule
                ? RadialKind.Capsule
                : RadialKind.Disk;
            Signed832 planeSquared = radialKind == RadialKind.Disk
                ? GetPlaneSquared(
                    axisSquared,
                    shapeAxisSquared,
                    alignment)
                : default;
            positive = new ProjectionDepth(
                WideArithmetic.SubtractSigned704(
                    axialMagnitude,
                    scaledMinimum),
                common,
                radius,
                radialKind,
                axisSquared,
                shapeAxisSquared,
                planeSquared);
            negative = new ProjectionDepth(
                WideArithmetic.AddSigned704(
                    axialMagnitude,
                    scaledMaximum),
                common,
                radius,
                radialKind,
                axisSquared,
                shapeAxisSquared,
                planeSquared);
            return;
        }

        Signed832 conePlaneSquared = GetPlaneSquared(
            axisSquared,
            shapeAxisSquared,
            alignment);
        Signed704 doubledAxial = WideArithmetic.AddSigned704(axial, axial);
        bool positiveUsesBase = doubledAxial.Sign <= 0
            || IsDiskRadicalAtLeast(
                common,
                radius,
                conePlaneSquared,
                shapeAxisSquared,
                doubledAxial);
        Signed704 negativeBaseThreshold =
            WideArithmetic.SubtractSigned704(default, doubledAxial);
        bool negativeUsesBase = negativeBaseThreshold.Sign <= 0
            || IsDiskRadicalAtLeast(
                common,
                radius,
                conePlaneSquared,
                shapeAxisSquared,
                negativeBaseThreshold);
        positive = new ProjectionDepth(
            WideArithmetic.SubtractSigned704(
                positiveUsesBase
                    ? WideArithmetic.SubtractSigned704(default, axial)
                    : axial,
                scaledMinimum),
            common,
            radius,
            positiveUsesBase ? RadialKind.Disk : RadialKind.None,
            axisSquared,
            shapeAxisSquared,
            conePlaneSquared,
            positiveUsesBase ? ShapeSupportFeature.ConeBase : ShapeSupportFeature.ConeApex);
        negative = new ProjectionDepth(
            WideArithmetic.SubtractSigned704(
                scaledMaximum,
                negativeUsesBase
                    ? WideArithmetic.SubtractSigned704(default, axial)
                    : axial),
            common,
            radius,
            negativeUsesBase ? RadialKind.Disk : RadialKind.None,
            axisSquared,
            shapeAxisSquared,
            conePlaneSquared,
            negativeUsesBase ? ShapeSupportFeature.ConeBase : ShapeSupportFeature.ConeApex);
    }

    private static bool IsProjectionNonNegative(in ProjectionDepth depth)
    {
        if (depth.Rational.Sign >= 0)
            return true;
        if (depth.RadialKind == RadialKind.None)
            return false;

        if (depth.RadialKind == RadialKind.Capsule)
        {
            Span<ulong> left = stackalloc ulong[40];
            Span<ulong> right = stackalloc ulong[40];
            BuildLocalCapsuleRadicand(depth, left);
            BuildRationalSquare(depth.Rational, right);
            return CompareMagnitude(left, right) >= 0;
        }

        return IsDiskRadicalAtLeast(
            depth.Common,
            depth.Radius,
            depth.PlaneSquared,
            depth.ShapeAxisSquared,
            GetMagnitude(depth.Rational));
    }

    private static int CompareProjectionDepths(
        in ProjectionDepth left,
        in ProjectionDepth right)
    {
        Signed576 unit = Signed576.ExtendValue(
            Signed320.ExtendValue(
                Signed192.Signed(1L)));
        GetRadialProjection(
            left,
            unit,
            out Signed320 leftCoefficient,
            out Signed832 leftNumerator,
            out Signed576 leftDenominator);
        GetRadialProjection(
            right,
            unit,
            out Signed320 rightCoefficient,
            out Signed832 rightNumerator,
            out Signed576 rightDenominator);
        return WideArithmetic.CompareRadialProjectionDepths(
            left.Rational,
            leftCoefficient,
            leftNumerator,
            leftDenominator,
            left.AxisSquared,
            right.Rational,
            rightCoefficient,
            rightNumerator,
            rightDenominator,
            right.AxisSquared);
    }

    private static int CompareSignedProjectionDepths(
        in ProjectionDepth left,
        in ProjectionDepth right)
    {
        // Cylinder-capsule admission adds the same capsule radius to every
        // candidate, so base projections retain their ordering even when
        // either base is negative. Squared depths preserve order only within
        // one sign partition.
        bool leftNonNegative = IsProjectionNonNegative(left);
        bool rightNonNegative = IsProjectionNonNegative(right);
        if (leftNonNegative != rightNonNegative)
            return leftNonNegative ? 1 : -1;

        int squaredComparison = CompareProjectionDepths(left, right);
        return leftNonNegative
            ? squaredComparison
            : -squaredComparison;
    }

    private static void GetRadialProjection(
        in ProjectionDepth depth,
        Signed576 unit,
        out Signed320 coefficient,
        out Signed832 numerator,
        out Signed576 denominator)
    {
        if (depth.RadialKind == RadialKind.None)
        {
            coefficient = default;
            numerator = default;
            denominator = unit;
            return;
        }

        coefficient = WideArithmetic.MultiplySigned192(
            depth.Common,
            Signed192.Raw(depth.Radius));
        if (depth.RadialKind == RadialKind.Capsule)
        {
            numerator = Signed832.ExtendValue(
                depth.AxisSquared);
            denominator = unit;
        }
        else
        {
            numerator = depth.PlaneSquared;
            denominator = Signed576.ExtendValue(
                depth.ShapeAxisSquared);
        }
    }

    private static Fixed64 GetRoundedDepth(
        in ProjectionDepth depth,
        out bool isClamped)
    {
        if (CompareProjectionDepthToTwiceRaw(
                depth,
                new Signed192(0UL, 0UL, unchecked((ulong)long.MaxValue << 1))) > 0)
        {
            isClamped = true;
            return Fixed64.MaxValue;
        }

        Fixed64 approximation = GetDepthApproximation(depth);
        // Sign-aware rational roots and one-sided disk roots keep this estimate
        // at or below the exact depth. The admitted Q32-scaled axes keep exact
        // upward correction within five raw units.
        while (true)
        {
            Signed192 upperMidpoint = new(
                0UL,
                0UL,
                unchecked((ulong)approximation.m_rawValue << 1) | 1UL);
            int upperComparison =
                CompareProjectionDepthToTwiceRaw(
                    depth,
                    upperMidpoint);
            if ((approximation < Fixed64.MaxValue)
                & (upperComparison
                    + (approximation.m_rawValue & 1L) > 0))
            {
                approximation = Fixed64.FromRaw(
                    approximation.m_rawValue + 1L);
                continue;
            }

            isClamped = false;
            return approximation;
        }
    }

    private static Fixed64 GetDepthApproximation(
        in ProjectionDepth depth)
    {
        Signed704 axisSquared =
            Signed704.ExtendValue(depth.AxisSquared);
        Signed576 axisFloor =
            WideArithmetic.GetFloorSquareRoot(axisSquared);
        Signed576 axisCeiling =
            GetCeilingSquareRoot(axisSquared, axisFloor);
        Signed576 rationalAxisLength = depth.Rational.Sign < 0
            ? axisFloor
            : axisCeiling;
        Signed576 rationalDenominator = WideArithmetic.MultiplySigned576(
            rationalAxisLength,
            depth.Common);
        Fixed64 radial = depth.RadialKind switch
        {
            RadialKind.Capsule => depth.Radius,
            RadialKind.Disk => GetDiskDepthApproximation(
                depth,
                axisCeiling),
            _ => Fixed64.Zero,
        };
        Signed704 combinedNumerator = WideArithmetic.AddSigned704(
            depth.Rational,
            WideArithmetic.MultiplySigned576ToSigned704(
                rationalDenominator,
                Signed320.ExtendValue(Signed192.Raw(radial))));
        // Cancel rational and radial support before narrowing so individually
        // out-of-domain terms cannot discard a representable shallow depth.
        return Fixed64.GetNonNegativeRawRatioFloor(
            combinedNumerator,
            Signed704.ExtendValue(rationalDenominator));
    }

    private static Fixed64 GetDiskDepthApproximation(
        in ProjectionDepth depth,
        Signed576 axisCeiling)
    {
        Signed576 planeLength =
            WideArithmetic.GetFloorSquareRootOfProduct(
                depth.PlaneSquared,
                Signed192.Signed(1L));
        Signed192 shapeAxisFloor = WideArithmetic.GetFloorSquareRoot(
            depth.ShapeAxisSquared,
            out Signed192 shapeAxisRemainder);
        Signed192 shapeAxisCeiling = shapeAxisRemainder.IsZero
            ? shapeAxisFloor
            : WideArithmetic.AddSigned192(
                shapeAxisFloor,
                Signed192.Signed(1L));
        Signed576 numerator = WideArithmetic.MultiplySigned576(
            planeLength,
            Signed192.Raw(depth.Radius));
        Signed576 denominator = WideArithmetic.MultiplySigned576(
            axisCeiling,
            shapeAxisCeiling);
        return Fixed64.GetNonNegativeRawRatioFloor(
            Signed704.ExtendValue(numerator),
            Signed704.ExtendValue(denominator));
    }

    private static Signed576 GetCeilingSquareRoot(
        Signed704 squared,
        Signed576 floor)
    {
        _ = Signed320.TryNarrowSigned(
            floor,
            out Signed320 narrowFloor);
        Signed704 floorSquared =
            WideArithmetic.MultiplySigned576ToSigned704(
                floor,
                narrowFloor);
        return WideArithmetic.SubtractSigned704(
                squared,
                floorSquared).IsZero
            ? floor
            : WideArithmetic.AddSigned576(
                floor,
                Signed576.ExtendValue(
                    Signed320.ExtendValue(
                        Signed192.Signed(1L))));
    }

    private static ulong GetNearestEvenIncrement(
        int midpointComparison,
        ulong floor) =>
        unchecked((ulong)(
            (midpointComparison
                + (int)(floor & 1UL)
                + 1) >> 1));

    private static int CompareProjectionDepthToTwiceRaw(
        in ProjectionDepth depth,
        Signed192 twiceRaw)
    {
        Span<ulong> leftFirst = stackalloc ulong[40];
        Span<ulong> leftSecond = stackalloc ulong[40];
        Span<ulong> rightFirst = stackalloc ulong[40];
        Span<ulong> rightSecond = stackalloc ulong[40];
        leftFirst.Clear();
        leftSecond.Clear();
        rightFirst.Clear();
        rightSecond.Clear();

        Span<ulong> rationalTerm = depth.Rational.Sign >= 0
            ? leftFirst
            : rightFirst;
        BuildProduct(
            depth.Rational,
            depth.Rational,
            depth.ShapeAxisSquared,
            rationalTerm);
        ShiftLeft(rationalTerm, 2);
        if (depth.RadialKind != RadialKind.None)
        {
            Span<ulong> radialTerm = IsZero(leftFirst)
                ? leftFirst
                : leftSecond;
            if (depth.RadialKind == RadialKind.Capsule)
                BuildThresholdCapsuleRadicand(depth, radialTerm);
            else
                BuildLocalDiskRadicand(depth, radialTerm);
            ShiftLeft(radialTerm, 2);
        }

        Signed320 thresholdCoefficient =
            WideArithmetic.MultiplySigned192(depth.Common, twiceRaw);
        BuildThresholdRadicand(
            thresholdCoefficient,
            depth.ShapeAxisSquared,
            depth.AxisSquared,
            rightSecond);
        return CompareRadicalPairs(
            leftFirst,
            leftSecond,
            rightFirst,
            rightSecond);
    }

    private static bool IsDiskRadicalAtLeast(
        Signed192 common,
        Fixed64 radius,
        Signed832 planeSquared,
        Signed320 shapeAxisSquared,
        Signed704 threshold)
    {
        Span<ulong> radial = stackalloc ulong[40];
        Span<ulong> rational = stackalloc ulong[40];
        Signed320 coefficient = WideArithmetic.MultiplySigned192(
            common,
            Signed192.Raw(radius));
        BuildProduct(coefficient, coefficient, planeSquared, radial);
        BuildProduct(threshold, threshold, shapeAxisSquared, rational);
        return CompareMagnitude(radial, rational) >= 0;
    }

    private static Signed576 GetAxisProjection(
        Axis3 axis,
        RigidAxis3 direction) =>
        WideArithmetic.AddSigned576(
            WideArithmetic.AddSigned576(
                WideArithmetic.MultiplySigned320(
                    axis.X,
                    Signed320.ExtendValue(direction.X)),
                WideArithmetic.MultiplySigned320(
                    axis.Y,
                    Signed320.ExtendValue(direction.Y))),
            WideArithmetic.MultiplySigned320(
                axis.Z,
                Signed320.ExtendValue(direction.Z)));

    private static Signed576 GetAxisSquared(Axis3 axis) =>
        WideArithmetic.AddSigned576(
            WideArithmetic.AddSigned576(
                WideArithmetic.MultiplySigned320(axis.X, axis.X),
                WideArithmetic.MultiplySigned320(axis.Y, axis.Y)),
            WideArithmetic.MultiplySigned320(axis.Z, axis.Z));

    private static Signed320 GetAxisSquared(RigidAxis3 axis) =>
        WideArithmetic.AddSigned320(
            WideArithmetic.AddSigned320(
                WideArithmetic.MultiplySigned192(axis.X, axis.X),
                WideArithmetic.MultiplySigned192(axis.Y, axis.Y)),
            WideArithmetic.MultiplySigned192(axis.Z, axis.Z));

    private static Signed832 GetPlaneSquared(
        Signed576 axisSquared,
        Signed320 shapeAxisSquared,
        Signed576 alignment) =>
        WideArithmetic.SubtractSigned832(
            WideArithmetic.MultiplyNonNegativeToSigned832(
                Signed832.ExtendValue(axisSquared),
                shapeAxisSquared),
            WideArithmetic.MultiplySigned576ToSigned832(
                alignment,
                alignment));

    private static Signed704 GetMagnitude(Signed704 value) =>
        value.Sign >= 0
            ? value
            : WideArithmetic.SubtractSigned704(default, value);

    private static void BuildLocalCapsuleRadicand(
        in ProjectionDepth depth,
        Span<ulong> result)
    {
        Signed320 coefficient = WideArithmetic.MultiplySigned192(
            depth.Common,
            Signed192.Raw(depth.Radius));
        BuildProduct(
            coefficient,
            coefficient,
            depth.AxisSquared,
            result);
    }

    private static void BuildThresholdCapsuleRadicand(
        in ProjectionDepth depth,
        Span<ulong> result)
    {
        Signed320 coefficient = WideArithmetic.MultiplySigned192(
            depth.Common,
            Signed192.Raw(depth.Radius));
        BuildProduct(
            coefficient,
            coefficient,
            depth.ShapeAxisSquared,
            depth.AxisSquared,
            result);
    }

    private static void BuildLocalDiskRadicand(
        in ProjectionDepth depth,
        Span<ulong> result)
    {
        Signed320 coefficient = WideArithmetic.MultiplySigned192(
            depth.Common,
            Signed192.Raw(depth.Radius));
        BuildProduct(
            coefficient,
            coefficient,
            depth.PlaneSquared,
            result);
    }

    private static void BuildRationalSquare(
        Signed704 rational,
        Span<ulong> result) =>
        BuildProduct(rational, rational, result);

    private static void BuildThresholdRadicand(
        Signed320 coefficient,
        Signed320 shapeAxisSquared,
        Signed576 axisSquared,
        Span<ulong> result) =>
        BuildProduct(
            coefficient,
            coefficient,
            shapeAxisSquared,
            axisSquared,
            result);

    private static int CompareRadicalPairs(
        ReadOnlySpan<ulong> leftFirst,
        ReadOnlySpan<ulong> leftSecond,
        ReadOnlySpan<ulong> rightFirst,
        ReadOnlySpan<ulong> rightSecond)
    {
        int termWords = leftFirst.Length;
        int productWords = termWords * 2;
        Span<ulong> leftBase = stackalloc ulong[termWords];
        Span<ulong> rightBase = stackalloc ulong[termWords];
        AddMagnitudes(leftFirst, leftSecond, leftBase);
        AddMagnitudes(rightFirst, rightSecond, rightBase);
        int baseComparison = CompareMagnitude(leftBase, rightBase);
        Span<ulong> baseMagnitude = stackalloc ulong[termWords];
        if (baseComparison >= 0)
            SubtractMagnitudes(leftBase, rightBase, baseMagnitude);
        else
            SubtractMagnitudes(rightBase, leftBase, baseMagnitude);

        Span<ulong> leftProduct = stackalloc ulong[productWords];
        Span<ulong> rightProduct = stackalloc ulong[productWords];
        MultiplyMagnitudes(leftFirst, leftSecond, leftProduct);
        MultiplyMagnitudes(rightFirst, rightSecond, rightProduct);
        if (baseComparison >= 0)
        {
            return ComparePositiveRadicalDifference(
                baseMagnitude,
                leftProduct,
                rightProduct);
        }

        return -ComparePositiveRadicalDifference(
            baseMagnitude,
            rightProduct,
            leftProduct);
    }

    private static int ComparePositiveRadicalDifference(
        ReadOnlySpan<ulong> positiveBase,
        ReadOnlySpan<ulong> sameSideProduct,
        ReadOnlySpan<ulong> oppositeSideProduct)
    {
        int productWords = sameSideProduct.Length;
        int squaredWords = productWords * 2;
        Span<ulong> baseSquared = stackalloc ulong[productWords];
        Span<ulong> fourSame = stackalloc ulong[productWords];
        Span<ulong> fourOpposite = stackalloc ulong[productWords];
        MultiplyMagnitudes(positiveBase, positiveBase, baseSquared);
        sameSideProduct.CopyTo(fourSame);
        oppositeSideProduct.CopyTo(fourOpposite);
        ShiftLeft(fourSame, 2);
        ShiftLeft(fourOpposite, 2);

        Span<ulong> knownLeft = stackalloc ulong[productWords];
        AddMagnitudes(baseSquared, fourSame, knownLeft);
        int knownComparison = CompareMagnitude(
            knownLeft,
            fourOpposite);
        if (knownComparison > 0)
            return 1;

        Span<ulong> remainder = stackalloc ulong[productWords];
        if (knownComparison == 0)
        {
            return Math.Sign(
                GetActiveLength(positiveBase)
                * GetActiveLength(sameSideProduct));
        }

        SubtractMagnitudes(fourOpposite, knownLeft, remainder);
        Span<ulong> crossSquared = stackalloc ulong[squaredWords];
        Span<ulong> remainderSquared = stackalloc ulong[squaredWords];
        MultiplyMagnitudes(baseSquared, sameSideProduct, crossSquared);
        ShiftLeft(crossSquared, 4);
        MultiplyMagnitudes(remainder, remainder, remainderSquared);
        return CompareMagnitude(crossSquared, remainderSquared);
    }
}
