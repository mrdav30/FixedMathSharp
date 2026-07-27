//=======================================================================
// WideFiniteAxisIntersection.NarrowSolver.cs
//=======================================================================
// MIT License, Copyright (c) 2024–present David Oravsky (mrdav30)
// See LICENSE file in the project root for full license information.
//=======================================================================

using System;

namespace FixedMathSharp.Geometry;

/// <content>
/// Narrowing helpers that reduce wide 320-bit radial coefficients down to
/// 192-bit values when their magnitude and common shift allow it, enabling
/// cheaper downstream arithmetic in the narrow intersection solver.
/// </content>
internal static partial class WideFiniteAxisIntersection
{
    // Squaring two values below 2^159 and subtracting their products stays
    // inside the signed 320-bit discriminant used by WideRayIntersection.
    private const int NarrowRadialCoefficientBits = 159;

    private static bool TryGetNarrowRadialCoefficients(
        Signed320 coefficient,
        Signed320 projection,
        Signed320 constant,
        out Signed192 narrowCoefficient,
        out Signed192 narrowProjection,
        out Signed192 narrowConstant)
    {
        int commonShift = Math.Min(
            GetTrailingZeroCount(coefficient),
            Math.Min(GetTrailingZeroCount(projection), GetTrailingZeroCount(constant)));
        if (commonShift > 0)
        {
            coefficient = ShiftRightExact(coefficient, commonShift);
            projection = ShiftRightExact(projection, commonShift);
            constant = ShiftRightExact(constant, commonShift);
        }

        int maximumBits = Math.Max(
            GetMagnitudeBitLength(coefficient),
            Math.Max(GetMagnitudeBitLength(projection), GetMagnitudeBitLength(constant)));
        if (maximumBits > NarrowRadialCoefficientBits)
        {
            narrowCoefficient = default;
            narrowProjection = default;
            narrowConstant = default;
            return false;
        }

        bool coefficientFits = Signed192.TryNarrowSigned(coefficient, out narrowCoefficient);
        bool projectionFits = Signed192.TryNarrowSigned(projection, out narrowProjection);
        bool constantFits = Signed192.TryNarrowSigned(constant, out narrowConstant);
        return coefficientFits && projectionFits && constantFits;
    }

    private static int GetMagnitudeBitLength(Signed320 value)
    {
        if (value.IsZero)
            return 0;

        WideArithmetic.GetMagnitude(
            value,
            out ulong word4,
            out ulong word3,
            out ulong word2,
            out ulong word1,
            out ulong word0);
        return WideArithmetic.GetBitLength(word4, word3, word2, word1, word0);
    }

    private static int GetTrailingZeroCount(Signed320 value)
    {
        if (value.Word0 != 0UL)
            return CountTrailingZeroes(value.Word0);
        if (value.Word1 != 0UL)
            return 64 + CountTrailingZeroes(value.Word1);
        if (value.Word2 != 0UL)
            return 128 + CountTrailingZeroes(value.Word2);
        return 192;
    }

    private static int CountTrailingZeroes(ulong value) =>
        63 - Fixed64.CountLeadingZeroes(value & unchecked(0UL - value));

    private static Signed320 ShiftRightExact(Signed320 value, int bits)
    {
        int wordShift = bits >> 6;
        int bitShift = bits & 63;
        return new Signed320(
            GetShiftedWord(value, 4, wordShift, bitShift),
            GetShiftedWord(value, 3, wordShift, bitShift),
            GetShiftedWord(value, 2, wordShift, bitShift),
            GetShiftedWord(value, 1, wordShift, bitShift),
            GetShiftedWord(value, 0, wordShift, bitShift));
    }

    private static ulong GetShiftedWord(Signed320 value, int index, int wordShift, int bitShift)
    {
        int sourceIndex = index + wordShift;
        ulong lower = GetSignedWord(value, sourceIndex);
        if (bitShift == 0)
            return lower;

        ulong upper = GetSignedWord(value, sourceIndex + 1);
        return (lower >> bitShift) | (upper << (64 - bitShift));
    }

    private static ulong GetSignedWord(Signed320 value, int index) => index switch
    {
        0 => value.Word0,
        1 => value.Word1,
        2 => value.Word2,
        3 => value.Word3,
        4 => value.Word4,
        _ => value.Sign < 0 ? ulong.MaxValue : 0UL,
    };
}
