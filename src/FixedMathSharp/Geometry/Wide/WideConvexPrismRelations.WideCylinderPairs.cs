//=======================================================================
// WideConvexPrismRelations.WideCylinderPairs.cs
//=======================================================================
// MIT License, Copyright (c) 2024-present David Oravsky (mrdav30)
// See LICENSE file in the project root for full license information.
//=======================================================================

using System;

namespace FixedMathSharp.Bounds;

/// <content>
/// Wide-precision helpers for resolving cylinder-cylinder separating axis
/// candidates, including axis alignment, center projection, and penetration
/// fallback calculations.
/// </content>
internal static partial class WideConvexPrismRelations
{
    private static bool TryKeepWideCylinderCylinderAxisFallback(
        WideCandidateAxis3 candidate,
        Vector3d firstCenter,
        RigidAxis3 firstAxis,
        Fixed64 firstLength,
        Fixed64 firstRadius,
        Vector3d secondCenter,
        RigidAxis3 secondAxis,
        Fixed64 secondLength,
        Fixed64 secondRadius,
        ref CylinderCylinderPenetration best)
    {
        Span<ulong> axisSquared =
            stackalloc ulong[WideCandidateWords];
        Span<ulong> firstAlignment =
            stackalloc ulong[WideCandidateWords];
        Span<ulong> secondAlignment =
            stackalloc ulong[WideCandidateWords];
        Span<ulong> centerProjection =
            stackalloc ulong[WideCandidateWords];
        BuildWideAxisSquared(candidate, axisSquared);
        BuildWideDot(
            candidate,
            firstAxis.X,
            firstAxis.Y,
            firstAxis.Z,
            firstAlignment,
            out _);
        BuildWideDot(
            candidate,
            secondAxis.X,
            secondAxis.Y,
            secondAxis.Z,
            secondAlignment,
            out _);
        BuildWideDot(
            candidate,
            WideArithmetic.SubtractSigned192(
                Signed192.Raw(secondCenter.X),
                Signed192.Raw(firstCenter.X)),
            WideArithmetic.SubtractSigned192(
                Signed192.Raw(secondCenter.Y),
                Signed192.Raw(firstCenter.Y)),
            WideArithmetic.SubtractSigned192(
                Signed192.Raw(secondCenter.Z),
                Signed192.Raw(firstCenter.Z)),
            centerProjection,
            out _);

        Span<ulong> firstAxial =
            stackalloc ulong[WideCandidateWords];
        Span<ulong> secondAxial =
            stackalloc ulong[WideCandidateWords];
        Span<ulong> scaledCenter =
            stackalloc ulong[WideCandidateWords];
        Span<ulong> axialSum =
            stackalloc ulong[WideCandidateWords];
        Span<ulong> rational =
            stackalloc ulong[WideCandidateWords];
        BuildWideScaledAlignment(
            firstAlignment,
            firstLength,
            secondAxis.RotationDenominator,
            firstAxial);
        BuildWideScaledAlignment(
            secondAlignment,
            secondLength,
            firstAxis.RotationDenominator,
            secondAxial);
        BuildWideScaledCenter(
            centerProjection,
            firstAxis.RotationDenominator,
            secondAxis.RotationDenominator,
            scaledCenter);
        AddMagnitudes(
            firstAxial,
            secondAxial,
            axialSum);
        CombineWideSignedMagnitudes(
            axialSum,
            1,
            scaledCenter,
            -1,
            rational,
            out int rationalSign);

        Signed320 firstAxisSquared =
            GetAxisSquared(firstAxis);
        Signed320 secondAxisSquared =
            GetAxisSquared(secondAxis);
        Span<ulong> firstPlaneSquared =
            stackalloc ulong[WideCandidateWords];
        Span<ulong> secondPlaneSquared =
            stackalloc ulong[WideCandidateWords];
        BuildWidePlaneSquared(
            axisSquared,
            firstAxisSquared,
            firstAlignment,
            firstPlaneSquared);
        BuildWidePlaneSquared(
            axisSquared,
            secondAxisSquared,
            secondAlignment,
            secondPlaneSquared);
        Signed576 commonWide = WideArithmetic.MultiplySigned576(
            Signed576.ExtendValue(
                WideArithmetic.MultiplySigned192(
                    firstAxis.RotationDenominator,
                    secondAxis.RotationDenominator)),
            Signed192.Raw(Fixed64.Two));
        _ = Signed192.TryNarrowSigned(
            commonWide,
            out Signed192 common);
        var depth = new WideCylinderPairDepth(
            rational,
            rationalSign,
            common,
            firstRadius,
            secondRadius,
            axisSquared,
            firstAxisSquared,
            secondAxisSquared,
            firstPlaneSquared,
            secondPlaneSquared);
        if (!IsWideCylinderPairDepthNonNegative(depth))
            return false;
        if (best.HasValue
            && CompareWideCylinderPairDepth(
                depth,
                best.Depth) >= 0)
        {
            return true;
        }

        GetRoundedWideCylinderPairDepth(
            depth,
            out Fixed64 roundedDepth,
            out bool depthIsClamped);
        best = new CylinderCylinderPenetration(
            GetWideCandidateNormal(candidate),
            roundedDepth,
            depthIsClamped);
        return true;
    }

    private static bool IsWideCylinderPairDepthNonNegative(
        WideCylinderPairDepth depth)
    {
        Span<ulong> radicands = stackalloc ulong[
            WideCandidateWords * 3];
        Span<int> signs = stackalloc int[3];
        radicands.Clear();
        signs.Clear();
        BuildWideCylinderPairLocalRadicands(
            depth,
            radicands,
            signs);
        return WideArithmetic.GetLinearRadicalSumSign(
            radicands,
            WideCandidateWords,
            signs) >= 0;
    }

    private static int CompareWideCylinderPairDepth(
        WideCylinderPairDepth left,
        in CylinderPairDepth right)
    {
        Span<ulong> radicands = stackalloc ulong[
            WideCandidateWords * 6];
        Span<int> signs = stackalloc int[6];
        radicands.Clear();
        signs.Clear();
        BuildWideCylinderPairCrossRadicands(
            left,
            right,
            radicands,
            signs);
        return WideArithmetic.GetLinearRadicalSumSign(
            radicands,
            WideCandidateWords,
            signs);
    }

    private static void BuildWideCylinderPairLocalRadicands(
        WideCylinderPairDepth depth,
        Span<ulong> radicands,
        Span<int> signs)
    {
        BuildWideCylinderPairRationalRadicand(
            depth,
            radicands.Slice(
                0,
                WideCandidateWords));
        signs[0] = depth.RationalSign;
        BuildWideCylinderPairDiskRadicand(
            depth,
            firstDisk: true,
            radicands.Slice(
                WideCandidateWords,
                WideCandidateWords));
        signs[1] = 1;
        BuildWideCylinderPairDiskRadicand(
            depth,
            firstDisk: false,
            radicands.Slice(
                WideCandidateWords * 2,
                WideCandidateWords));
        signs[2] = 1;
    }

    private static void BuildWideCylinderPairCrossRadicands(
        WideCylinderPairDepth left,
        in CylinderPairDepth right,
        Span<ulong> radicands,
        Span<int> signs)
    {
        BuildWideCylinderPairCrossRationalRadicand(
            left,
            right,
            radicands.Slice(
                0,
                WideCandidateWords));
        signs[0] = left.RationalSign;
        BuildWideCylinderPairCrossDiskRadicand(
            left,
            firstDisk: true,
            right,
            radicands.Slice(
                WideCandidateWords,
                WideCandidateWords));
        signs[1] = 1;
        BuildWideCylinderPairCrossDiskRadicand(
            left,
            firstDisk: false,
            right,
            radicands.Slice(
                WideCandidateWords * 2,
                WideCandidateWords));
        signs[2] = 1;

        BuildNarrowCylinderPairCrossRationalRadicand(
            right,
            left,
            radicands.Slice(
                WideCandidateWords * 3,
                WideCandidateWords));
        signs[3] = -right.Rational.Sign;
        BuildNarrowCylinderPairCrossDiskRadicand(
            right,
            firstDisk: true,
            left,
            radicands.Slice(
                WideCandidateWords * 4,
                WideCandidateWords));
        signs[4] = -1;
        BuildNarrowCylinderPairCrossDiskRadicand(
            right,
            firstDisk: false,
            left,
            radicands.Slice(
                WideCandidateWords * 5,
                WideCandidateWords));
        signs[5] = -1;
    }

    private static void BuildWideCylinderPairRationalRadicand(
        WideCylinderPairDepth depth,
        Span<ulong> result)
    {
        MultiplyMagnitudes(
            depth.Rational,
            depth.Rational,
            result);
        MultiplyWideCylinderPairBy(
            result,
            depth.FirstAxisSquared);
        MultiplyWideCylinderPairBy(
            result,
            depth.SecondAxisSquared);
    }

    private static void BuildWideCylinderPairDiskRadicand(
        WideCylinderPairDepth depth,
        bool firstDisk,
        Span<ulong> result)
    {
        Span<ulong> coefficient =
            stackalloc ulong[5];
        GetWideRadialCoefficient(
            depth.Common,
            firstDisk
                ? depth.FirstRadius
                : depth.SecondRadius,
            coefficient);
        MultiplyMagnitudes(
            coefficient,
            coefficient,
            result);
        MultiplyWideCylinderPairBy(
            result,
            firstDisk
                ? depth.FirstPlaneSquared
                : depth.SecondPlaneSquared);
        MultiplyWideCylinderPairBy(
            result,
            firstDisk
                ? depth.SecondAxisSquared
                : depth.FirstAxisSquared);
    }

    private static void BuildWideCylinderPairCrossRationalRadicand(
        WideCylinderPairDepth depth,
        in CylinderPairDepth other,
        Span<ulong> result)
    {
        MultiplyMagnitudes(
            depth.Rational,
            depth.Rational,
            result);
        MultiplyWideCylinderPairBy(
            result,
            other.AxisSquared);
        MultiplyWideCylinderPairBy(
            result,
            depth.FirstAxisSquared);
        MultiplyWideCylinderPairBy(
            result,
            depth.SecondAxisSquared);
        MultiplyWideCylinderPairBy(
            result,
            other.FirstAxisSquared);
        MultiplyWideCylinderPairBy(
            result,
            other.SecondAxisSquared);
    }

    private static void BuildWideCylinderPairCrossDiskRadicand(
        WideCylinderPairDepth depth,
        bool firstDisk,
        in CylinderPairDepth other,
        Span<ulong> result)
    {
        BuildWideCylinderPairDiskRadicand(
            depth,
            firstDisk,
            result);
        MultiplyWideCylinderPairBy(
            result,
            other.AxisSquared);
        MultiplyWideCylinderPairBy(
            result,
            other.FirstAxisSquared);
        MultiplyWideCylinderPairBy(
            result,
            other.SecondAxisSquared);
    }

    private static void BuildNarrowCylinderPairCrossRationalRadicand(
        in CylinderPairDepth depth,
        WideCylinderPairDepth other,
        Span<ulong> result)
    {
        Span<ulong> rationalWords =
            stackalloc ulong[11];
        WideArithmetic.GetMagnitude(
            depth.Rational,
            rationalWords);
        MultiplyMagnitudes(
            rationalWords,
            rationalWords,
            result);
        MultiplyWideCylinderPairBy(
            result,
            other.AxisSquared);
        MultiplyWideCylinderPairBy(
            result,
            depth.FirstAxisSquared);
        MultiplyWideCylinderPairBy(
            result,
            depth.SecondAxisSquared);
        MultiplyWideCylinderPairBy(
            result,
            other.FirstAxisSquared);
        MultiplyWideCylinderPairBy(
            result,
            other.SecondAxisSquared);
    }

    private static void BuildNarrowCylinderPairCrossDiskRadicand(
        in CylinderPairDepth depth,
        bool firstDisk,
        WideCylinderPairDepth other,
        Span<ulong> result)
    {
        Span<ulong> coefficient =
            stackalloc ulong[5];
        GetWideRadialCoefficient(
            depth.Common,
            firstDisk
                ? depth.FirstRadius
                : depth.SecondRadius,
            coefficient);
        MultiplyMagnitudes(
            coefficient,
            coefficient,
            result);
        if (firstDisk)
        {
            MultiplyWideCylinderPairBy(
                result,
                depth.FirstPlaneSquared);
            MultiplyWideCylinderPairBy(
                result,
                depth.SecondAxisSquared);
        }
        else
        {
            MultiplyWideCylinderPairBy(
                result,
                depth.SecondPlaneSquared);
            MultiplyWideCylinderPairBy(
                result,
                depth.FirstAxisSquared);
        }
        MultiplyWideCylinderPairBy(
            result,
            other.AxisSquared);
        MultiplyWideCylinderPairBy(
            result,
            other.FirstAxisSquared);
        MultiplyWideCylinderPairBy(
            result,
            other.SecondAxisSquared);
    }

    private static void MultiplyWideCylinderPairBy(
        Span<ulong> value,
        Signed320 factor)
    {
        Span<ulong> words = stackalloc ulong[5];
        GetMagnitude(factor, words);
        MultiplyWideCylinderPairBy(
            value,
            words);
    }

    private static void MultiplyWideCylinderPairBy(
        Span<ulong> value,
        Signed576 factor)
    {
        Span<ulong> words = stackalloc ulong[9];
        WideArithmetic.GetMagnitude(
            factor,
            words);
        MultiplyWideCylinderPairBy(
            value,
            words);
    }

    private static void MultiplyWideCylinderPairBy(
        Span<ulong> value,
        Signed832 factor)
    {
        Span<ulong> words = stackalloc ulong[13];
        WideArithmetic.GetMagnitude(
            factor,
            words);
        MultiplyWideCylinderPairBy(
            value,
            words);
    }

    private static void MultiplyWideCylinderPairBy(
        Span<ulong> value,
        ReadOnlySpan<ulong> factor)
    {
        Span<ulong> product =
            stackalloc ulong[WideCandidateWords];
        MultiplyMagnitudes(
            value,
            factor,
            product);
        product.CopyTo(value);
    }

    private static void GetRoundedWideCylinderPairDepth(
        WideCylinderPairDepth depth,
        out Fixed64 rounded,
        out bool isClamped)
    {
        ulong low = 0UL;
        ulong high = 1UL << 63;
        while (low < high)
        {
            ulong midpoint =
                low + ((high - low) >> 1);
            int comparison =
                CompareWideCylinderPairDepthToTwiceRaw(
                    depth,
                    new Signed192(
                        0UL,
                        0UL,
                        midpoint << 1));
            if (comparison >= 0)
                low = midpoint + 1UL;
            else
                high = midpoint;
        }

        ulong floor = low - 1UL;
        int midpointComparison =
            CompareWideCylinderPairDepthToTwiceRaw(
                depth,
                new Signed192(
                    0UL,
                    0UL,
                    (floor << 1) | 1UL));
        ulong roundedRaw = floor + GetNearestEvenIncrement(
            midpointComparison,
            floor);
        rounded = Fixed64.FromRaw(
            (long)Math.Min(
                roundedRaw,
                unchecked((ulong)long.MaxValue)));
        isClamped =
            CompareWideCylinderPairDepthToTwiceRaw(
                depth,
                new Signed192(
                    0UL,
                    0UL,
                    unchecked((ulong)long.MaxValue << 1))) > 0;
    }

    private static int CompareWideCylinderPairDepthToTwiceRaw(
        WideCylinderPairDepth depth,
        Signed192 twiceRaw)
    {
        Span<ulong> radicands = stackalloc ulong[
            WideCandidateWords * 4];
        Span<int> signs = stackalloc int[4];
        radicands.Clear();
        signs.Clear();
        BuildWideCylinderPairLocalRadicands(
            depth,
            radicands,
            signs);
        for (int index = 0; index < 3; index++)
        {
            ShiftLeft(
                radicands.Slice(
                    index * WideCandidateWords,
                    WideCandidateWords),
                2);
        }

        Signed320 thresholdCoefficient =
            WideArithmetic.MultiplySigned192(
                depth.Common,
                twiceRaw);
        Span<ulong> coefficient =
            stackalloc ulong[5];
        GetMagnitude(
            thresholdCoefficient,
            coefficient);
        Span<ulong> threshold = radicands.Slice(
            WideCandidateWords * 3,
            WideCandidateWords);
        MultiplyMagnitudes(
            coefficient,
            coefficient,
            threshold);
        MultiplyWideCylinderPairBy(
            threshold,
            depth.AxisSquared);
        MultiplyWideCylinderPairBy(
            threshold,
            depth.FirstAxisSquared);
        MultiplyWideCylinderPairBy(
            threshold,
            depth.SecondAxisSquared);
        signs[3] = -1;
        return WideArithmetic.GetLinearRadicalSumSign(
            radicands,
            WideCandidateWords,
            signs);
    }
}
