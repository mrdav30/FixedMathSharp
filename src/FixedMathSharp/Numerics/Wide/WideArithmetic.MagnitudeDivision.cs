//=======================================================================
// MIT License, Copyright (c) 2024-present David Oravsky (mrdav30)
// See LICENSE file in the project root for full license information.
//=======================================================================
using System;

namespace FixedMathSharp;

/// <content>
/// Unsigned Euclidean division and integer-content reduction using supplied
/// scratch. Inputs are little-endian magnitudes, without hidden allocation.
/// </content>
internal static partial class WideArithmetic
{
    /// <summary>
    /// Divides by a nonzero magnitude. The quotient may be omitted. Scratch
    /// requires numerator.Length + divisor.Length + 1 words and must be
    /// disjoint from all inputs/outputs. A supplied quotient needs at least
    /// numerator.Length words; remainder needs at least divisor.Length words.
    /// Outputs must not overlap each other or divisor, but either may replace
    /// numerator. Unused output words are cleared.
    /// </summary>
    internal static void DivideMagnitudes(ReadOnlySpan<ulong> numerator,
        ReadOnlySpan<ulong> divisor, Span<ulong> quotient, Span<ulong> remainder,
        Span<ulong> scratch)
    {
        int numeratorLength = GetActiveMagnitudeLength(numerator);
        int divisorLength = GetActiveMagnitudeLength(divisor);
        System.Diagnostics.Debug.Assert(divisorLength > 0);
        if (numeratorLength < divisorLength)
        {
            numerator[..numeratorLength].CopyTo(remainder);
            remainder[numeratorLength..].Clear();
            quotient.Clear();
            return;
        }

        // Normalizing the high divisor bit bounds each trial quotient's
        // overestimate by two. One extra numerator word retains its carry.
        int shift = Fixed64.CountLeadingZeroes(divisor[divisorLength - 1]);
        Span<ulong> dividend = scratch[..(numeratorLength + 1)];
        Span<ulong> normalizedDivisor = scratch.Slice(numeratorLength + 1, divisorLength);
        ShiftMagnitudeForDivision(numerator[..numeratorLength], dividend, shift);
        ShiftMagnitudeForDivision(divisor[..divisorLength], normalizedDivisor, shift);
        quotient.Clear();
        ulong top = normalizedDivisor[divisorLength - 1];
        for (int offset = numeratorLength - divisorLength; offset >= 0; offset--)
        {
            ulong high = dividend[offset + divisorLength];
            ulong low = dividend[offset + divisorLength - 1];
            ulong trial;
            ulong trialRemainder;
            bool remainderOverflow;
            if (high == top)
            {
                trial = ulong.MaxValue;
                trialRemainder = unchecked(low + top);
                remainderOverflow = trialRemainder < low;
            }
            else
            {
                trial = Fixed64.Divide128By64(high, low, top, out trialRemainder);
                remainderOverflow = false;
            }

            if (divisorLength > 1)
            {
                for (int correction = 0; correction < 2 && !remainderOverflow; correction++)
                {
                    Fixed64.Multiply64To128(trial, normalizedDivisor[divisorLength - 2],
                        out ulong productHigh, out ulong productLow);
                    if (productHigh < trialRemainder
                        || (productHigh == trialRemainder
                            && productLow <= dividend[offset + divisorLength - 2]))
                        break;
                    trial--;
                    ulong previous = trialRemainder;
                    trialRemainder = unchecked(trialRemainder + top);
                    remainderOverflow = trialRemainder < previous;
                }
            }

            ulong carry = 0;
            for (int index = 0; index < divisorLength; index++)
            {
                Fixed64.Multiply64To128(trial, normalizedDivisor[index],
                    out ulong productHigh, out ulong productLow);
                ulong previous = productLow;
                productLow = unchecked(productLow + carry);
                carry = unchecked(productHigh + (productLow < previous ? 1UL : 0UL));
                ulong value = dividend[offset + index];
                dividend[offset + index] = unchecked(value - productLow);
                if (value < productLow)
                    carry++;
            }
            bool borrowed = dividend[offset + divisorLength] < carry;
            dividend[offset + divisorLength] = unchecked(dividend[offset + divisorLength] - carry);
            if (borrowed)
            {
                // The top-two-word test can leave one excess trial unit.
                // Add the divisor back; the quotient is now exact.
                trial--;
                carry = 0;
                for (int index = 0; index < divisorLength; index++)
                    dividend[offset + index] = AddSignedWord(dividend[offset + index],
                        normalizedDivisor[index], ref carry);
                dividend[offset + divisorLength] = unchecked(dividend[offset + divisorLength] + carry);
            }
            if (!quotient.IsEmpty)
                quotient[offset] = trial;
        }

        for (int index = 0; index < divisorLength; index++)
        {
            ulong upper = shift == 0 ? 0UL : dividend[index + 1] << (64 - shift);
            remainder[index] = (dividend[index] >> shift) | upper;
        }
        remainder[divisorLength..].Clear();
    }

    /// <summary>
    /// Replaces first with gcd(first, second), including gcd(0,0)=0.
    /// Both equal-width operands are scratch; they must not overlap.
    /// The supplied scratch requires twice that width plus one word.
    /// </summary>
    internal static void GetMagnitudeGreatestCommonDivisor(Span<ulong> first,
        Span<ulong> second, Span<ulong> scratch)
    {
        Span<ulong> left = first;
        Span<ulong> right = second;
        while (GetActiveMagnitudeLength(right) != 0)
        {
            DivideMagnitudes(left, right, Span<ulong>.Empty, left, scratch);
            Span<ulong> swap = left;
            left = right;
            right = swap;
        }
        left.CopyTo(first);
    }

    private static void ShiftMagnitudeForDivision(ReadOnlySpan<ulong> source,
        Span<ulong> destination, int shift)
    {
        ulong carry = 0;
        for (int index = 0; index < source.Length; index++)
        {
            ulong value = source[index];
            destination[index] = (value << shift) | carry;
            carry = shift == 0 ? 0UL : value >> (64 - shift);
        }
        if (destination.Length > source.Length)
            destination[source.Length] = carry;
    }
}
