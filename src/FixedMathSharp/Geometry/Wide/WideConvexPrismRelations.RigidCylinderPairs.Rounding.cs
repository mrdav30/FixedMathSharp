//=======================================================================
// WideConvexPrismRelations.RigidCylinderPairs.Rounding.cs
//=======================================================================
// MIT License, Copyright (c) 2024-present David Oravsky (mrdav30)
// See LICENSE file in the project root for full license information.
//=======================================================================

using System;

namespace FixedMathSharp.Geometry;

/// <content>
/// Provides correctly-rounded penetration depth calculations for rigid cylinder
/// pairs, using fast scalar approximations corrected by exact fixed-point
/// comparisons, with a fallback binary search for edge cases near overflow.
/// </content>
internal static partial class WideConvexPrismRelations
{
    private static void GetRoundedCylinderCylinderDepth(
        in CylinderPairDepth depth,
        out Fixed64 result,
        out bool isClamped)
    {
        if (!TryGetCylinderPairDepthApproximation(
                depth,
                out Fixed64 approximation))
        {
            Signed192 maximumTwiceRaw = new(
                0UL,
                0UL,
                unchecked((ulong)long.MaxValue << 1));
            int maximumComparison =
                CompareCylinderPairDepthToTwiceRaw(
                    depth,
                    maximumTwiceRaw);
            if (maximumComparison >= 0)
            {
                result = Fixed64.MaxValue;
                isClamped = maximumComparison > 0;
                return;
            }
            result = GetRoundedCylinderPairDepthBySearch(depth);
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
                result = GetRoundedCylinderPairDepthBySearch(depth);
                isClamped =
                    CompareCylinderPairDepthToTwiceRaw(
                        depth,
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
                    CompareCylinderPairDepthToTwiceRaw(
                        depth,
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
                CompareCylinderPairDepthToTwiceRaw(
                    depth,
                    upperMidpoint);
            if (upperComparison
                + (approximation.m_rawValue & 1L) > 0)
            {
                approximation = Fixed64.FromRaw(
                    approximation.m_rawValue + 1L);
                continue;
            }

            result = approximation;
            isClamped = false;
            return;
        }
    }

    private static bool TryGetCylinderPairDepthApproximation(
        in CylinderPairDepth depth,
        out Fixed64 approximation)
    {
        Signed576 axisLength =
            WideArithmetic.GetFloorSquareRoot(
                Signed704.ExtendValue(
                    depth.AxisSquared));
        Signed576 rationalDenominator =
            WideArithmetic.MultiplySigned576(
                axisLength,
                depth.Common);
        if (!Fixed64.TryGetSignedRawRatio(
                depth.Rational,
                Signed704.ExtendValue(
                    rationalDenominator),
                out Fixed64 rational))
        {
            approximation = default;
            return false;
        }
        Fixed64 firstDisk =
            GetDiskDepthApproximation(
                depth.CreateFirstDiskDepth(),
                axisLength);
        Fixed64 secondDisk =
            GetDiskDepthApproximation(
                depth.CreateSecondDiskDepth(),
                axisLength);
        Signed192 combined = WideArithmetic.AddSigned192(
            WideArithmetic.AddSigned192(
                Signed192.Raw(rational),
                Signed192.Raw(firstDisk)),
            Signed192.Raw(secondDisk));
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

    private static Fixed64 GetRoundedCylinderPairDepthBySearch(
        in CylinderPairDepth depth)
    {
        Signed192 maximumTwiceRaw = new(
            0UL,
            0UL,
            unchecked((ulong)long.MaxValue << 1));
        if (CompareCylinderPairDepthToTwiceRaw(
                depth,
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
                CompareCylinderPairDepthToTwiceRaw(
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
            CompareCylinderPairDepthToTwiceRaw(
                depth,
                new Signed192(
                    0UL,
                    0UL,
                    (floor << 1) | 1UL));
        return Fixed64.FromRaw(
            (long)(floor + GetNearestEvenIncrement(
                midpointComparison,
                floor)));
    }

    private static int CompareCylinderPairDepthToTwiceRaw(
        in CylinderPairDepth depth,
        Signed192 twiceRaw)
    {
        Span<ulong> radicands = stackalloc ulong[
            CylinderPairRadicandWords * 4];
        Span<int> signs = stackalloc int[4];
        radicands.Clear();
        signs.Clear();
        BuildCylinderPairLocalRadicands(
            depth,
            radicands,
            signs);
        for (int index = 0; index < 3; index++)
            ShiftLeft(
                radicands.Slice(
                    index * CylinderPairRadicandWords,
                    CylinderPairRadicandWords),
                2);

        Signed320 thresholdCoefficient =
            WideArithmetic.MultiplySigned192(
                depth.Common,
                twiceRaw);
        Span<ulong> threshold = radicands.Slice(
            CylinderPairRadicandWords * 3,
            CylinderPairRadicandWords);
        Span<ulong> coefficientWords =
            stackalloc ulong[5];
        GetMagnitude(
            thresholdCoefficient,
            coefficientWords);
        MultiplyMagnitudes(
            coefficientWords,
            coefficientWords,
            threshold);
        MultiplyCylinderPairBy(
            threshold,
            depth.AxisSquared);
        MultiplyCylinderPairBy(
            threshold,
            depth.FirstAxisSquared);
        MultiplyCylinderPairBy(
            threshold,
            depth.SecondAxisSquared);
        signs[3] = -1;
        return WideArithmetic.GetLinearRadicalSumSign(
            radicands,
            CylinderPairRadicandWords,
            signs);
    }
}
