//=======================================================================
// WideArithmetic.SquareRoot.cs
//=======================================================================
// MIT License, Copyright (c) 2024-present David Oravsky (mrdav30)
// See LICENSE file in the project root for full license information.
//=======================================================================
using System;

namespace FixedMathSharp;

/// <content>
/// Policy-neutral magnitude square-root enclosures and clipped ratio floors.
/// </content>
internal static partial class WideArithmetic
{
    /// <summary>
    /// Encloses sqrt(value) between lower*2^shift and upper*2^shift without
    /// mutating the little-endian magnitude. Empty and zero inputs give zero.
    /// Endpoints are nonnegative and at most 97 bits; they differ by zero or one.
    /// </summary>
    internal static void GetMagnitudeSquareRootBounds(ReadOnlySpan<ulong> value,
        out Signed192 lower, out Signed192 upper, out int shift)
    {
        int bits = GetMagnitudeBitLength(value);
        shift = bits <= 192 ? 0 : (bits - 191) / 2;
        // Removing an even number of bits keeps sqrt scaling exact. The
        // retained prefix has at most 192 bits, so the existing narrow root
        // owner suffices even for much wider input magnitudes.
        int discardedBits = 2 * shift;
        int wordShift = discardedBits >> 6;
        int bitShift = discardedBits & 63;
        Span<ulong> prefix = stackalloc ulong[3];
        for (int index = 0; index < prefix.Length; index++)
        {
            int sourceIndex = wordShift + index;
            ulong low = sourceIndex < value.Length ? value[sourceIndex] : 0;
            // A nonzero bitShift implies a 191/192-bit retained prefix.
            // The active input therefore has at least wordShift+4 words:
            // sourceIndex+1 exists even for the last prefix word.
            ulong high = bitShift != 0
                ? value[sourceIndex + 1] << (64 - bitShift) : 0;
            prefix[index] = (low >> bitShift) | high;
        }
        lower = GetFloorSquareRoot(new Signed320(0, 0, prefix[2], prefix[1], prefix[0]),
            out Signed192 remainder);
        bool discarded = false;
        for (int index = 0; index < wordShift; index++)
            discarded |= value[index] != 0;
        if (bitShift != 0)
            discarded |= (value[wordShift] & ((1UL << bitShift) - 1)) != 0;
        // A perfect prefix alone does not prove a perfect full input. Otherwise
        // value < (q+1)^2*2^(2*shift), including a carry from q's 96th bit.
        upper = remainder.IsZero && !discarded ? lower
            : AddSigned192(lower, new Signed192(0, 0, 1));
    }

    /// <summary>
    /// Returns min(cap, floor(sqrt(numerator/denominator))) without mutating
    /// inputs. Both little-endian magnitudes have equal padded widths of at
    /// least two words; denominator is positive. No quotient bits are truncated.
    /// </summary>
    internal static ulong GetRatioFloorSquareRoot(ReadOnlySpan<ulong> numerator,
        ReadOnlySpan<ulong> denominator, ulong cap)
    {
        int words = numerator.Length;
        // cap^2*D needs two extra words even though both input bounds fit W.
        // Division still needs a full-width quotient, not merely its low128 bits.
        Span<ulong> storage = stackalloc ulong[5 * words + 5];
        Span<ulong> product = storage[..(words + 2)];
        Span<ulong> quotient = storage.Slice(words + 2, words + 2);
        Span<ulong> remainder = storage.Slice(2 * words + 4, words);
        Span<ulong> division = storage[(3 * words + 4)..];
        Span<ulong> squareCap = stackalloc ulong[2];
        Fixed64.Multiply64To128(cap, cap, out squareCap[1], out squareCap[0]);
        MultiplyMagnitudes(denominator, squareCap, product);
        quotient.Clear();
        numerator.CopyTo(quotient);
        if (CompareMagnitudeEqualLength(quotient, product) >= 0)
            return cap;
        DivideMagnitudes(numerator, denominator, quotient[..words], remainder, division);
        // The unclipped quotient is <cap^2<2^128, so packing its two low words
        // loses no bits. floor(sqrt(floor(N/D))) equals floor(sqrt(N/D)).
        return GetFloorSquareRoot(new Signed320(0, 0, 0, quotient[1], quotient[0]), out _).Low;
    }
}
