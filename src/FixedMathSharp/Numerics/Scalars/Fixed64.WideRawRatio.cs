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
    /// Converts an exact signed five-word ratio with a positive single-word
    /// denominator to a raw integer with round-half-to-even.
    /// </summary>
    /// <remarks>
    /// The caller owns the invariant that the quotient is representable.
    /// </remarks>
    internal static Fixed64 GetSignedRawRatio(Signed320 numerator, Signed192 denominator)
    {
        int numeratorSign = numerator.Sign;
        if (numeratorSign == 0)
            return Zero;

        WideArithmetic.GetMagnitude(
            denominator,
            out _,
            out _,
            out ulong denominatorLow);
        WideArithmetic.GetMagnitude(
            numerator,
            out _,
            out _,
            out _,
            out ulong word1,
            out ulong word0);

        ulong quotient = Divide128By64(
            word1,
            word0,
            denominatorLow,
            out ulong remainder);
        int midpointComparison = remainder.CompareTo(denominatorLow - remainder);
        if (midpointComparison > 0 || (midpointComparison == 0 && (quotient & 1UL) != 0UL))
            quotient++;

        long raw = numeratorSign < 0 ? unchecked(-(long)quotient) : (long)quotient;
        return new Fixed64(raw);
    }

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
        if (WideArithmetic.TryNarrowSigned192(numerator, out Signed192 numerator192)
            && WideArithmetic.TryNarrowSigned192(denominator, out Signed192 denominator192))
        {
            Span<ulong> narrowRemainder = stackalloc ulong[3];
            Span<ulong> narrowDenominator = stackalloc ulong[3];
            WideArithmetic.GetMagnitude(
                numerator192,
                out narrowRemainder[2], out narrowRemainder[1], out narrowRemainder[0]);
            WideArithmetic.GetMagnitude(
                denominator192,
                out narrowDenominator[2], out narrowDenominator[1], out narrowDenominator[0]);
            return TryGetSignedRawRatioCore(narrowRemainder, narrowDenominator, negative, out result);
        }
        if (WideArithmetic.TryNarrowSigned320(numerator, out Signed320 numerator320)
            && WideArithmetic.TryNarrowSigned320(denominator, out Signed320 denominator320))
        {
            Span<ulong> mediumRemainder = stackalloc ulong[5];
            Span<ulong> mediumDenominator = stackalloc ulong[5];
            WideArithmetic.GetMagnitude(
                numerator320,
                out mediumRemainder[4], out mediumRemainder[3], out mediumRemainder[2],
                out mediumRemainder[1], out mediumRemainder[0]);
            WideArithmetic.GetMagnitude(
                denominator320,
                out mediumDenominator[4], out mediumDenominator[3], out mediumDenominator[2],
                out mediumDenominator[1], out mediumDenominator[0]);
            return TryGetSignedRawRatioCore(mediumRemainder, mediumDenominator, negative, out result);
        }

        Span<ulong> remainder = stackalloc ulong[9];
        Span<ulong> denominatorMagnitude = stackalloc ulong[9];
        WideArithmetic.GetMagnitude(numerator, remainder);
        WideArithmetic.GetMagnitude(denominator, denominatorMagnitude);
        return TryGetSignedRawRatioCore(remainder, denominatorMagnitude, negative, out result);
    }

    private static bool TryGetSignedRawRatioCore(
        Span<ulong> remainder,
        Span<ulong> denominatorMagnitude,
        bool negative,
        out Fixed64 result)
    {
        int remainderLength = GetActiveMagnitudeLength(remainder);
        int denominatorLength = GetActiveMagnitudeLength(denominatorMagnitude);
        int activeLength = Math.Max(remainderLength, denominatorLength);
        Span<ulong> activeRemainder = remainder[..activeLength];
        Span<ulong> activeDenominator = denominatorMagnitude[..activeLength];
        int quotientBit = GetMagnitudeBitLength(remainder[..remainderLength])
            - GetMagnitudeBitLength(denominatorMagnitude[..denominatorLength]);
        if (quotientBit > 63)
        {
            result = default;
            return false;
        }

        ulong quotient = 0UL;
        if (quotientBit >= 0)
        {
            Span<ulong> shiftedDenominatorStorage = stackalloc ulong[9];
            Span<ulong> shiftedDenominator = shiftedDenominatorStorage[..activeLength];
            ShiftLeftMagnitude(activeDenominator, quotientBit, shiftedDenominator);
            for (int bit = quotientBit; bit >= 0; bit--)
            {
                if (CompareMagnitude(activeRemainder, shiftedDenominator) >= 0)
                {
                    SubtractMagnitude(activeRemainder, shiftedDenominator);
                    quotient |= 1UL << bit;
                }

                ShiftRightOne(shiftedDenominator);
            }
        }

        Span<ulong> denominatorMinusRemainderStorage = stackalloc ulong[9];
        Span<ulong> denominatorMinusRemainder = denominatorMinusRemainderStorage[..activeLength];
        activeDenominator.CopyTo(denominatorMinusRemainder);
        SubtractMagnitude(denominatorMinusRemainder, activeRemainder);
        return TryCreateRawRatioResult(
            quotient,
            CompareMagnitude(activeRemainder, denominatorMinusRemainder),
            negative,
            out result);
    }

    private static bool TryCreateRawRatioResult(
        ulong quotient,
        int midpointComparison,
        bool negative,
        out Fixed64 result)
    {
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
        int index = value.Length - 1;
        return (index * 64) + 64 - CountLeadingZeroes(value[index]);
    }

    private static int GetActiveMagnitudeLength(ReadOnlySpan<ulong> value)
    {
        int length = value.Length;
        while (length > 1 && value[length - 1] == 0UL)
            length--;
        return length;
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
