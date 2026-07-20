//=======================================================================
// Fixed64.WideRawRatio.cs
//=======================================================================
// MIT License, Copyright (c) 2024–present David Oravsky (mrdav30)
// See LICENSE file in the project root for full license information.
//=======================================================================

using System;

namespace FixedMathSharp;

public partial struct Fixed64
{
    /// <summary>
    /// Converts an exact signed nine-word ratio directly to a raw integer with
    /// round-half-to-even. Unlike <c>GetSignedRatio</c>, this method does not
    /// apply an additional Q32.32 scale.
    /// </summary>
    internal static bool TryGetSignedRawRatio(
        Signed576 numerator,
        Signed576 denominator,
        out Fixed64 result)
    {
        int numeratorSign = numerator.Sign;
        int denominatorSign = denominator.Sign;
        if (denominatorSign == 0)
        {
            result = default;
            return false;
        }
        if (numeratorSign == 0)
        {
            result = Zero;
            return true;
        }

        bool negative = numeratorSign != denominatorSign;
        Span<ulong> remainder = stackalloc ulong[9];
        Span<ulong> denominatorMagnitude = stackalloc ulong[9];
        WideArithmetic.GetMagnitude(numerator, remainder);
        WideArithmetic.GetMagnitude(denominator, denominatorMagnitude);

        int quotientBit = GetMagnitudeBitLength(remainder)
            - GetMagnitudeBitLength(denominatorMagnitude);
        if (quotientBit > 63)
        {
            result = default;
            return false;
        }

        ulong quotient = 0UL;
        if (quotientBit >= 0)
        {
            Span<ulong> shiftedDenominator = stackalloc ulong[9];
            ShiftLeftMagnitude(denominatorMagnitude, quotientBit, shiftedDenominator);
            for (int bit = quotientBit; bit >= 0; bit--)
            {
                if (CompareMagnitude(remainder, shiftedDenominator) >= 0)
                {
                    SubtractMagnitude(remainder, shiftedDenominator);
                    quotient |= 1UL << bit;
                }

                ShiftRightOne(shiftedDenominator);
            }
        }

        Span<ulong> denominatorMinusRemainder = stackalloc ulong[9];
        denominatorMagnitude.CopyTo(denominatorMinusRemainder);
        SubtractMagnitude(denominatorMinusRemainder, remainder);
        int midpointComparison = CompareMagnitude(remainder, denominatorMinusRemainder);
        if (midpointComparison > 0 || (midpointComparison == 0 && (quotient & 1UL) != 0UL))
        {
            quotient++;
            if (quotient == 0UL)
            {
                result = default;
                return false;
            }
        }

        ulong limit = negative ? 1UL << 63 : (ulong)long.MaxValue;
        if (quotient > limit)
        {
            result = default;
            return false;
        }

        long raw = negative ? unchecked(-(long)quotient) : (long)quotient;
        result = new Fixed64(raw);
        return true;
    }

    private static int GetMagnitudeBitLength(ReadOnlySpan<ulong> value)
    {
        if (value[8] != 0UL) return 576 - CountLeadingZeroes(value[8]);
        if (value[7] != 0UL) return 512 - CountLeadingZeroes(value[7]);
        if (value[6] != 0UL) return 448 - CountLeadingZeroes(value[6]);
        if (value[5] != 0UL) return 384 - CountLeadingZeroes(value[5]);
        if (value[4] != 0UL) return 320 - CountLeadingZeroes(value[4]);
        if (value[3] != 0UL) return 256 - CountLeadingZeroes(value[3]);
        if (value[2] != 0UL) return 192 - CountLeadingZeroes(value[2]);
        if (value[1] != 0UL) return 128 - CountLeadingZeroes(value[1]);
        return 64 - CountLeadingZeroes(value[0]);
    }

    private static void ShiftLeftMagnitude(
        ReadOnlySpan<ulong> source,
        int bits,
        Span<ulong> destination)
    {
        if (bits == 0)
        {
            source.CopyTo(destination);
            return;
        }

        destination[0] = source[0] << bits;
        for (int index = 1; index < destination.Length; index++)
            destination[index] = (source[index] << bits) | (source[index - 1] >> (64 - bits));
    }

    private static int CompareMagnitude(ReadOnlySpan<ulong> left, ReadOnlySpan<ulong> right)
    {
        for (int index = left.Length - 1; index >= 0; index--)
        {
            if (left[index] != right[index])
                return left[index] < right[index] ? -1 : 1;
        }

        return 0;
    }

    private static void SubtractMagnitude(Span<ulong> value, ReadOnlySpan<ulong> subtract)
    {
        ulong borrow = 0UL;
        for (int index = 0; index < value.Length; index++)
        {
            ulong subtrahend = unchecked(subtract[index] + borrow);
            ulong nextBorrow = subtrahend < subtract[index] || value[index] < subtrahend
                ? 1UL
                : 0UL;
            value[index] = unchecked(value[index] - subtrahend);
            borrow = nextBorrow;
        }
    }

    private static void ShiftRightOne(Span<ulong> value)
    {
        ulong carry = 0UL;
        for (int index = value.Length - 1; index >= 0; index--)
        {
            ulong nextCarry = value[index] << 63;
            value[index] = (value[index] >> 1) | carry;
            carry = nextCarry;
        }
    }
}
