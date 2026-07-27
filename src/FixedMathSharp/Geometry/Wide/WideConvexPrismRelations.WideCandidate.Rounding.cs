//=======================================================================
// WideConvexPrismRelations.WideCandidate.Rounding.cs
//=======================================================================
// MIT License, Copyright (c) 2024-present David Oravsky (mrdav30)
// See LICENSE file in the project root for full license information.
//=======================================================================

using System;

namespace FixedMathSharp.Bounds;

/// <content>
/// Provides rounding utilities for wide-precision cylinder/capsule depth
/// calculations, using binary search to find the correctly rounded
/// (round-to-nearest-even) <see cref="Fixed64"/> result while detecting
/// clamping at the representable range boundary.
/// </content>
internal static partial class WideConvexPrismRelations
{
    private static void GetRoundedWideCylinderCapsuleDepth(
        WideCylinderCapsuleDepth depth,
        Fixed64 capsuleRadius,
        out Fixed64 rounded,
        out bool isClamped)
    {
        ulong low = 0UL;
        ulong high = 1UL << 63;
        while (low < high)
        {
            ulong midpoint = low + ((high - low) >> 1);
            int comparison =
                CompareWideCylinderCapsuleDepthToTwiceRaw(
                    depth,
                    capsuleRadius,
                    new Signed192(0UL, 0UL, midpoint << 1));
            if (comparison >= 0)
                low = midpoint + 1UL;
            else
                high = midpoint;
        }

        ulong floor = low - 1UL;
        if (floor == unchecked((ulong)long.MaxValue))
        {
            rounded = Fixed64.MaxValue;
            isClamped =
                CompareWideCylinderCapsuleDepthToTwiceRaw(
                    depth,
                    capsuleRadius,
                    new Signed192(
                        0UL,
                        0UL,
                        unchecked((ulong)long.MaxValue << 1))) > 0;
            return;
        }

        int midpointComparison =
            CompareWideCylinderCapsuleDepthToTwiceRaw(
                depth,
                capsuleRadius,
                new Signed192(
                    0UL,
                    0UL,
                    (floor << 1) | 1UL));
        rounded = Fixed64.FromRaw(
            (long)(floor + GetNearestEvenIncrement(
                midpointComparison,
                floor)));
        isClamped = false;
    }

    private static int CompareWideCylinderCapsuleDepthToTwiceRaw(
            WideCylinderCapsuleDepth depth,
            Fixed64 capsuleRadius,
            Signed192 twiceRaw)
    {
        Signed192 target = WideArithmetic.SubtractSigned192(
            twiceRaw,
            new Signed192(
                0UL,
                0UL,
                unchecked((ulong)(
                    capsuleRadius.m_rawValue
                    + capsuleRadius.m_rawValue))));
        Span<ulong> disk =
            stackalloc ulong[WideCandidateWords];
        Span<ulong> rational =
            stackalloc ulong[WideCandidateWords];
        Span<ulong> threshold =
            stackalloc ulong[WideCandidateWords];
        Span<ulong> leftFirst =
            stackalloc ulong[WideCandidateWords];
        Span<ulong> leftSecond =
            stackalloc ulong[WideCandidateWords];
        Span<ulong> rightFirst =
            stackalloc ulong[WideCandidateWords];
        Span<ulong> rightSecond =
            stackalloc ulong[WideCandidateWords];
        BuildWideDiskRadicand(depth, disk);
        BuildWideRationalRadicand(depth, rational);
        ShiftLeft(disk, 2);
        ShiftLeft(rational, 2);
        BuildWideThresholdRadicand(
            depth,
            target.Sign < 0
                ? WideArithmetic.SubtractSigned192(default, target)
                : target,
            threshold);
        leftFirst.Clear();
        leftSecond.Clear();
        rightFirst.Clear();
        rightSecond.Clear();
        if (target.Sign >= 0)
        {
            // Wide admission is reached only for a nonzero closest-point
            // residual. Its KKT support-minus-center term is strictly negative.
            CopyPositiveRadicand(
                rational,
                rightFirst,
                rightSecond);
            CopyPositiveRadicand(
                disk,
                leftFirst,
                leftSecond);
            CopyPositiveRadicand(
                threshold,
                rightFirst,
                rightSecond);
        }
        else
        {
            if (IsWideBaseProjectionNonNegative(depth))
                return 1;
            CopyPositiveRadicand(
                disk,
                leftFirst,
                leftSecond);
            CopyPositiveRadicand(
                threshold,
                leftFirst,
                leftSecond);
            CopyPositiveRadicand(
                rational,
                rightFirst,
                rightSecond);
        }
        return CompareWideRadicalPairs(
            leftFirst,
            leftSecond,
            rightFirst,
            rightSecond);
    }

    private static void BuildWideThresholdRadicand(
        WideCylinderCapsuleDepth depth,
        Signed192 twiceRaw,
        Span<ulong> result)
    {
        Signed320 coefficient =
            WideArithmetic.MultiplySigned192(
                depth.Common,
                twiceRaw);
        Span<ulong> coefficientMagnitude = stackalloc ulong[5];
        Span<ulong> shapeAxisMagnitude = stackalloc ulong[5];
        GetMagnitude(
            coefficient,
            coefficientMagnitude);
        GetMagnitude(
            depth.ShapeAxisSquared,
            shapeAxisMagnitude);
        Span<ulong> square =
            stackalloc ulong[WideCandidateWords];
        Span<ulong> withShapeAxis =
            stackalloc ulong[WideCandidateWords];
        MultiplyMagnitudes(
            coefficientMagnitude,
            coefficientMagnitude,
            square);
        MultiplyMagnitudes(
            square,
            shapeAxisMagnitude,
            withShapeAxis);
        MultiplyMagnitudes(
            withShapeAxis,
            depth.AxisSquared,
            result);
    }

    private static Vector3d GetWideCandidateNormal(
        WideCandidateAxis3 axis)
    {
        int maximumBitLength = Math.Max(
            GetMagnitudeBitLength(axis.X),
            Math.Max(
                GetMagnitudeBitLength(axis.Y),
                GetMagnitudeBitLength(axis.Z)));
        int shift = Math.Max(0, maximumBitLength - 380);
        // The candidate is the first-minus-second closest-point residual.
        // Its admitted nonzero center projection against second-minus-first
        // is strictly negative by the segment KKT relation.
        Signed576 x = GetShiftedSigned576(
            axis.X,
            -axis.XSign,
            shift);
        Signed576 y = GetShiftedSigned576(
            axis.Y,
            -axis.YSign,
            shift);
        Signed576 z = GetShiftedSigned576(
            axis.Z,
            -axis.ZSign,
            shift);
        return WideGeometry.GetNormalized(x, y, z);
    }

    private static Signed576 GetShiftedSigned576(
        ReadOnlySpan<ulong> magnitude,
        int sign,
        int shift)
    {
        Span<ulong> words = stackalloc ulong[9];
        Span<ulong> sourceWords = stackalloc ulong[7];
        words.Clear();
        sourceWords.Clear();
        int wordShift = shift >> 6;
        int bitShift = shift & 63;
        // Closest-candidate components contain 36 words. Shifting their
        // maximum to at most 380 bits leaves at least six source words; the
        // seventh word is a cleared carry sentinel at the maximum shift.
        int sourceLength = Math.Min(
            sourceWords.Length,
            magnitude.Length - wordShift);
        magnitude.Slice(
            wordShift,
            sourceLength).CopyTo(sourceWords);
        for (int index = 0; index < 6; index++)
        {
            ulong value = sourceWords[index] >> bitShift;
            if (bitShift != 0)
                value |= sourceWords[index + 1] << (64 - bitShift);
            words[index] = value;
        }
        Signed576 positive = new(
            words[8],
            words[7],
            words[6],
            words[5],
            words[4],
            words[3],
            words[2],
            words[1],
            words[0]);
        return sign < 0
            ? WideArithmetic.SubtractSigned576(default, positive)
            : positive;
    }

    private static int GetMagnitudeBitLength(
        ReadOnlySpan<ulong> magnitude)
    {
        int length = GetActiveLength(magnitude);
        if (length == 0)
            return 0;
        ulong high = magnitude[length - 1];
        int bits = (length - 1) << 6;
        while (high != 0UL)
        {
            bits++;
            high >>= 1;
        }
        return bits;
    }

    private static void CopyPositiveRadicand(
        ReadOnlySpan<ulong> radicand,
        Span<ulong> first,
        Span<ulong> second)
    {
        if (IsZero(first))
            radicand.CopyTo(first);
        else
            radicand.CopyTo(second);
    }

    private static int CompareWideRadicalPairs(
        ReadOnlySpan<ulong> leftFirst,
        ReadOnlySpan<ulong> leftSecond,
        ReadOnlySpan<ulong> rightFirst,
        ReadOnlySpan<ulong> rightSecond)
    {
        Span<ulong> leftBase =
            stackalloc ulong[WideCandidateWords];
        Span<ulong> rightBase =
            stackalloc ulong[WideCandidateWords];
        AddMagnitudes(
            leftFirst,
            leftSecond,
            leftBase);
        AddMagnitudes(
            rightFirst,
            rightSecond,
            rightBase);
        int baseComparison =
            CompareMagnitude(leftBase, rightBase);
        Span<ulong> baseMagnitude =
            stackalloc ulong[WideCandidateWords];
        if (baseComparison >= 0)
        {
            SubtractMagnitudes(
                leftBase,
                rightBase,
                baseMagnitude);
        }
        else
        {
            SubtractMagnitudes(
                rightBase,
                leftBase,
                baseMagnitude);
        }

        Span<ulong> leftProduct =
            stackalloc ulong[WideCandidateWords * 2];
        Span<ulong> rightProduct =
            stackalloc ulong[WideCandidateWords * 2];
        MultiplyMagnitudes(
            leftFirst,
            leftSecond,
            leftProduct);
        MultiplyMagnitudes(
            rightFirst,
            rightSecond,
            rightProduct);
        if (baseComparison >= 0)
        {
            return CompareWidePositiveRadicalDifference(
                baseMagnitude,
                leftProduct,
                rightProduct);
        }
        return -CompareWidePositiveRadicalDifference(
            baseMagnitude,
            rightProduct,
            leftProduct);
    }

    private static int CompareWidePositiveRadicalDifference(
        ReadOnlySpan<ulong> positiveBase,
        ReadOnlySpan<ulong> sameSideProduct,
        ReadOnlySpan<ulong> oppositeSideProduct)
    {
        Span<ulong> baseSquared =
            stackalloc ulong[WideCandidateWords * 2];
        Span<ulong> fourSame =
            stackalloc ulong[WideCandidateWords * 2];
        Span<ulong> fourOpposite =
            stackalloc ulong[WideCandidateWords * 2];
        MultiplyMagnitudes(
            positiveBase,
            positiveBase,
            baseSquared);
        sameSideProduct.CopyTo(fourSame);
        oppositeSideProduct.CopyTo(fourOpposite);
        ShiftLeft(fourSame, 2);
        ShiftLeft(fourOpposite, 2);
        Span<ulong> knownLeft =
            stackalloc ulong[WideCandidateWords * 2];
        AddMagnitudes(
            baseSquared,
            fourSame,
            knownLeft);
        int knownComparison =
            CompareMagnitude(knownLeft, fourOpposite);
        if (knownComparison > 0)
            return 1;
        if (knownComparison == 0)
        {
            return GetActiveLength(positiveBase)
                * GetActiveLength(sameSideProduct) == 0
                ? 0
                : 1;
        }

        Span<ulong> remainder =
            stackalloc ulong[WideCandidateWords * 2];
        SubtractMagnitudes(
            fourOpposite,
            knownLeft,
            remainder);
        Span<ulong> crossSquared =
            stackalloc ulong[WideCandidateWords * 4];
        Span<ulong> remainderSquared =
            stackalloc ulong[WideCandidateWords * 4];
        MultiplyMagnitudes(
            baseSquared,
            sameSideProduct,
            crossSquared);
        ShiftLeft(crossSquared, 4);
        MultiplyMagnitudes(
            remainder,
            remainder,
            remainderSquared);
        return CompareMagnitude(
            crossSquared,
            remainderSquared);
    }

    private static void CombineWideSignedMagnitudes(
        ReadOnlySpan<ulong> left,
        int leftSign,
        ReadOnlySpan<ulong> right,
        int rightSign,
        Span<ulong> result,
        out int resultSign)
    {
        if (leftSign == 0 || IsZero(left))
        {
            right.CopyTo(result);
            resultSign = IsZero(right) ? 0 : rightSign;
            return;
        }
        if (rightSign == 0 || IsZero(right))
        {
            left.CopyTo(result);
            resultSign = leftSign;
            return;
        }
        if (leftSign == rightSign)
        {
            AddMagnitudes(left, right, result);
            resultSign = leftSign;
            return;
        }

        int comparison = CompareMagnitude(left, right);
        if (comparison == 0)
        {
            result.Clear();
            resultSign = 0;
            return;
        }
        if (comparison > 0)
        {
            SubtractMagnitudes(left, right, result);
            resultSign = leftSign;
        }
        else
        {
            SubtractMagnitudes(right, left, result);
            resultSign = rightSign;
        }
    }
}
