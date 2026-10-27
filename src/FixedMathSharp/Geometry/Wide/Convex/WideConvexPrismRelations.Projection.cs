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
/// projection depths of capsule/prism contacts and shared radial depth rounding.
/// </content>
internal static partial class WideConvexPrismRelations
{
    private static void GetCapsuleProjectionDepths(
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

        Signed704 axialMagnitude = GetMagnitude(axial);
        positive = new ProjectionDepth(
            WideArithmetic.SubtractSigned704(
                axialMagnitude,
                scaledMinimum),
            common,
            radius,
            RadialKind.Capsule,
            axisSquared,
            shapeAxisSquared);
        negative = new ProjectionDepth(
            WideArithmetic.AddSigned704(
                axialMagnitude,
                scaledMaximum),
            common,
            radius,
            RadialKind.Capsule,
            axisSquared,
            shapeAxisSquared);
    }

    private static bool IsProjectionNonNegative(in ProjectionDepth depth)
    {
        if (depth.Rational.Sign >= 0)
            return true;
        Span<ulong> left = stackalloc ulong[40];
        Span<ulong> right = stackalloc ulong[40];
        BuildLocalCapsuleRadicand(depth, left);
        BuildRationalSquare(depth.Rational, right);
        return WideArithmetic.CompareMagnitudeEqualLength(left, right) >= 0;
    }

    /// <summary>Tests the selected sphere support plane's exact center distance against its radius.</summary>
    internal static bool IsSphereProjectionWithinRadius(in ProjectionDepth depth) =>
        IsProjectionNonNegative(new ProjectionDepth(WideArithmetic.SubtractSigned704(default, depth.Rational), depth.Common,
            depth.Radius, depth.RadialKind, depth.AxisSquared, depth.ShapeAxisSquared));

    internal static int CompareProjectionDepths(
        in ProjectionDepth left,
        in ProjectionDepth right)
    {
        Signed576 unit = Signed576.ExtendValue(
            Signed320.ExtendValue(
                Signed192.Signed(1L)));
        Signed320 leftCoefficient = WideArithmetic.MultiplySigned192(left.Common, Signed192.Raw(left.Radius));
        Signed320 rightCoefficient = WideArithmetic.MultiplySigned192(right.Common, Signed192.Raw(right.Radius));
        // Equal radial terms and axis norms cancel exactly; widening the
        // rational subtraction preserves the full signed input range.
        if (left.AxisSquared.Equals(right.AxisSquared) && leftCoefficient.Equals(rightCoefficient))
            return WideArithmetic.SubtractSigned832(Signed832.ExtendValue(left.Rational), Signed832.ExtendValue(right.Rational)).Sign;
        return WideArithmetic.CompareRadialProjectionDepths(
            left.Rational,
            leftCoefficient,
            Signed832.ExtendValue(left.AxisSquared),
            unit,
            left.AxisSquared,
            right.Rational,
            rightCoefficient,
            Signed832.ExtendValue(right.AxisSquared),
            unit,
            right.AxisSquared);
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
        // The estimate is a lower bound at every axis scale. Ordinary scaled
        // axes need at most five corrections; a reduced rational basis can
        // make the integer-root enclosure wider, requiring a bounded search.
        for (int correction = 0; correction < 5; correction++)
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

        long low = approximation.m_rawValue, high = long.MaxValue;
        while (low < high)
        {
            long middle = low + ((high - low) >> 1);
            Signed192 upperMidpoint = new(0UL, 0UL, unchecked((ulong)middle << 1) | 1UL);
            int comparison = CompareProjectionDepthToTwiceRaw(depth, upperMidpoint);
            // Crossing an odd cell's upper half rounds to the next even value.
            if (comparison + (middle & 1L) > 0) low = middle + 1;
            else high = middle;
        }
        isClamped = false;
        return Fixed64.FromRaw(low);
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
        Fixed64 radial = depth.RadialKind == RadialKind.Capsule ? depth.Radius : Fixed64.Zero;
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
            BuildThresholdCapsuleRadicand(depth, radialTerm);
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

    // Both callers pack at most three nonzero signed radical terms into
    // these two pairs. A fourth term would invalidate the reduction below.
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
        WideArithmetic.AddEqualMagnitudes(leftFirst, leftSecond, leftBase);
        WideArithmetic.AddEqualMagnitudes(rightFirst, rightSecond, rightBase);
        int baseComparison = WideArithmetic.CompareMagnitudeEqualLength(leftBase, rightBase);
        Span<ulong> leftProduct = stackalloc ulong[productWords];
        Span<ulong> rightProduct = stackalloc ulong[productWords];
        WideArithmetic.MultiplyMagnitudes(leftFirst, leftSecond, leftProduct);
        WideArithmetic.MultiplyMagnitudes(rightFirst, rightSecond, rightProduct);
        // Squared radical sums differ by (a+b-c-d)+2*(sqrt(ab)-sqrt(cd)).
        // Agreeing signs decide this exactly; only opposing signs need the
        // larger squared comparison. Zero differences preserve exact ties.
        int productComparison = WideArithmetic.CompareMagnitudeEqualLength(leftProduct, rightProduct);
        if (baseComparison == 0)
            return productComparison;
        if (productComparison == 0 || productComparison == baseComparison)
            return baseComparison;
        // At least one pair product is zero. Opposing comparisons force
        // the product on the larger-base side to be zero, leaving B-2*sqrt(P).
        // Compare B² with 4P exactly; the opposite product is local scratch.
        Span<ulong> baseMagnitude = stackalloc ulong[termWords];
        if (baseComparison > 0)
            WideArithmetic.SubtractEqualMagnitudes(leftBase, rightBase, baseMagnitude);
        else
            WideArithmetic.SubtractEqualMagnitudes(rightBase, leftBase, baseMagnitude);
        Span<ulong> baseSquared = stackalloc ulong[productWords];
        WideArithmetic.MultiplyMagnitudes(baseMagnitude, baseMagnitude, baseSquared);
        Span<ulong> oppositeProduct = baseComparison > 0 ? rightProduct : leftProduct;
        ShiftLeft(oppositeProduct, 2);
        return baseComparison * WideArithmetic.CompareMagnitudeEqualLength(baseSquared, oppositeProduct);
    }
}
