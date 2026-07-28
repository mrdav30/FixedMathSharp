//=======================================================================
// WideArithmetic.Ratio.cs
//=======================================================================
// MIT License, Copyright (c) 2024-present David Oravsky (mrdav30)
// See LICENSE file in the project root for full license information.
//=======================================================================

using System;

namespace FixedMathSharp;

internal static partial class WideArithmetic
{
    internal static Signed320 GetSignedRatioWith64FractionBits(
        Signed576 numerator,
        Signed576 positiveDenominator)
    {
        if (numerator.Sign == 0)
            return default;

        Span<ulong> numeratorMagnitude = stackalloc ulong[9];
        Span<ulong> denominatorMagnitude = stackalloc ulong[10];
        Span<ulong> remainder = stackalloc ulong[10];
        Span<ulong> shiftedDenominator = stackalloc ulong[10];
        Span<ulong> quotient = stackalloc ulong[5];
        GetMagnitude(numerator, numeratorMagnitude);
        GetMagnitude(positiveDenominator, denominatorMagnitude);
        remainder.Clear();
        quotient.Clear();
        numeratorMagnitude.CopyTo(remainder[1..]);

        int quotientBit = GetMassRatioBitLength(remainder)
            - GetMassRatioBitLength(denominatorMagnitude);
        if (quotientBit >= 0)
        {
            ShiftMassRatioLeft(
                denominatorMagnitude,
                quotientBit,
                shiftedDenominator);
        }
        for (int bit = quotientBit; bit >= 0; bit--)
        {
            if (CompareMagnitudeEqualLength(
                    remainder,
                    shiftedDenominator) >= 0)
            {
                SubtractEqualMagnitudes(
                    remainder,
                    shiftedDenominator,
                    remainder);
                quotient[bit >> 6] |= 1UL << (bit & 63);
            }

            ShiftMassRatioRightOne(shiftedDenominator);
        }

        Span<ulong> twiceRemainder = stackalloc ulong[10];
        remainder.CopyTo(twiceRemainder);
        ShiftMassRatioLeftOne(twiceRemainder);
        int midpointComparison = CompareMagnitudeEqualLength(
            twiceRemainder,
            denominatorMagnitude);
        Signed320 magnitude = new(
            quotient[4],
            quotient[3],
            quotient[2],
            quotient[1],
            quotient[0]);
        if (midpointComparison > 0
            || (midpointComparison == 0
                && (quotient[0] & 1UL) != 0UL))
        {
            magnitude = AddSigned320(
                magnitude,
                Signed320.ExtendValue(Signed192.Signed(1)));
        }

        return numerator.Sign < 0
            ? SubtractSigned320(default, magnitude)
            : magnitude;
    }

    private static int GetMassRatioBitLength(
        ReadOnlySpan<ulong> value)
    {
        int index = value.Length - 1;
        while (value[index] == 0UL)
            index--;
        int leadingZeroes = 0;
        ulong word = value[index];
        for (ulong mask = 1UL << 63;
             (word & mask) == 0UL;
             mask >>= 1)
        {
            leadingZeroes++;
        }

        return (index * 64) + 64 - leadingZeroes;
    }

    private static void ShiftMassRatioLeft(
        ReadOnlySpan<ulong> source,
        int bits,
        Span<ulong> destination)
    {
        destination.Clear();
        int wordShift = bits >> 6;
        int bitShift = bits & 63;
        for (int index = 0; index < source.Length; index++)
        {
            int target = index + wordShift;
            if (target >= destination.Length)
                break;

            destination[target] |= source[index] << bitShift;
            if (bitShift != 0
                && target + 1 < destination.Length)
            {
                destination[target + 1] |=
                    source[index] >> (64 - bitShift);
            }
        }
    }

    private static void ShiftMassRatioRightOne(
        Span<ulong> value)
    {
        ulong carry = 0UL;
        for (int index = value.Length - 1; index >= 0; index--)
        {
            ulong nextCarry = value[index] << 63;
            value[index] = (value[index] >> 1) | carry;
            carry = nextCarry;
        }
    }

    private static void ShiftMassRatioLeftOne(
        Span<ulong> value)
    {
        ulong carry = 0UL;
        for (int index = 0; index < value.Length; index++)
        {
            ulong nextCarry = value[index] >> 63;
            value[index] = (value[index] << 1) | carry;
            carry = nextCarry;
        }
    }

}
