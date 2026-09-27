//=======================================================================
// MIT License, Copyright (c) 2024-present David Oravsky (mrdav30)
// See LICENSE file in the project root for full license information.
//=======================================================================
using System;
using System.Runtime.CompilerServices;

namespace FixedMathSharp.Geometry;

/// <content>
/// Primitive square-free factors identifying repeated value roots.
/// </content>
internal static partial class WideFiniteAxisIntersection
{
    /// <summary>
    /// Writes the positive-leading primitive radical of gcd(F,F') for a
    /// degree-at-most-eight polynomial. Returns its degree, zero with factor
    /// one when no root repeats, or -1 for the zero polynomial. Output has
    /// five equal-width coefficient slots and five signs; unused storage is
    /// cleared. Inputs and output must not overlap. A coefficient capacity of
    /// ceil((ceil(inputBits/2)+5)/64) words suffices.
    /// </summary>
    internal static int GetFiniteValueRepeatedRootFactor(ReadOnlySpan<ulong> coefficients,
        ReadOnlySpan<sbyte> signs, Span<ulong> factor, Span<sbyte> factorSigns)
    {
        factor.Clear();
        factorSigns.Clear();
        int degree = signs.Length - 1;
        while (degree >= 0 && signs[degree] == 0)
            degree--;
        if (degree < 0)
            return -1;
        if (degree <= 1)
        {
            factor[0] = 1;
            factorSigns[0] = 1;
            return 0;
        }

        int inputWords = coefficients.Length / signs.Length;
        coefficients = coefficients[..((degree + 1) * inputWords)];
        signs = signs[..(degree + 1)];
        int bits = GetFiniteRootCoefficientBits(coefficients, degree + 1);
        // Both intermediate gcds divide F. The degree-eight factor-height
        // bound needs fewer than eleven additional bits, independent of the
        // much larger transient Sturm coefficients.
        int words = (bits + 11 + 63) / 64;
        Span<ulong> gcd = stackalloc ulong[8 * words];
        Span<sbyte> gcdSigns = stackalloc sbyte[8];
        int gcdDegree = GetFiniteValueDerivativeGcd(coefficients, signs, gcd, gcdSigns);
        if (gcdDegree == 0)
        {
            factor[0] = 1;
            factorSigns[0] = 1;
            return 0;
        }

        Span<ulong> repeated = stackalloc ulong[8 * words];
        Span<sbyte> repeatedSigns = stackalloc sbyte[8];
        int repeatedDegree = GetFiniteValueDerivativeGcd(
            gcd[..((gcdDegree + 1) * words)], gcdSigns[..(gcdDegree + 1)], repeated, repeatedSigns);
        // G/gcd(G,G') retains each repeated irreducible factor of F once.
        // Its square divides primitive F, so its degree is at most four.
        return DivideFiniteValueFactors(gcd[..((gcdDegree + 1) * words)],
            gcdSigns[..(gcdDegree + 1)], repeated[..((repeatedDegree + 1) * words)],
            repeatedSigns[..(repeatedDegree + 1)], factor, factorSigns);
    }

    // Each call releases its construction arena before the next gcd or the
    // quotient phase. Only the two small factor buffers survive both calls.
    [MethodImpl(MethodImplOptions.NoInlining)]
    private static int GetFiniteValueDerivativeGcd(ReadOnlySpan<ulong> coefficients,
        ReadOnlySpan<sbyte> signs, Span<ulong> factor, Span<sbyte> factorSigns)
    {
        int bits = GetFiniteRootCoefficientBits(coefficients, signs.Length);
        Span<ulong> arena = stackalloc ulong[(880 * (bits + 64) + 63) / 64 + 512];
        Span<int> offsets = stackalloc int[9];
        Span<int> widths = stackalloc int[9];
        Span<int> degrees = stackalloc int[9];
        Span<sbyte> sturmSigns = stackalloc sbyte[81];
        int count = BuildFiniteValueSturm(coefficients, signs, arena,
            offsets, widths, degrees, sturmSigns, out int used);
        int last = count - 1;
        int degree = degrees[last];
        // Sturm construction retains raw subresultants. Only this exported
        // gcd needs primitive content, before copying into factor-sized slots.
        NormalizeFiniteValueContent(arena.Slice(offsets[last], (degree + 1) * widths[last]),
            degree, widths[last], sturmSigns.Slice(last * 9, 9), arena[used..]);
        int words = factor.Length / factorSigns.Length;
        int leadingSign = sturmSigns[last * 9 + degree];
        factor.Clear();
        factorSigns.Clear();
        for (int index = 0; index <= degree; index++)
        {
            CopyFiniteRootMagnitude(arena.Slice(offsets[last] + index * widths[last], widths[last]),
                factor.Slice(index * words, words));
            factorSigns[index] = (sbyte)(sturmSigns[last * 9 + index] * leadingSign);
        }
        return degree;
    }

    private static int DivideFiniteValueFactors(ReadOnlySpan<ulong> numerator,
        ReadOnlySpan<sbyte> numeratorSigns, ReadOnlySpan<ulong> divisor,
        ReadOnlySpan<sbyte> divisorSigns, Span<ulong> quotient, Span<sbyte> quotientSigns)
    {
        int degree = numeratorSigns.Length - 1;
        int divisorDegree = divisorSigns.Length - 1;
        int inputWords = numerator.Length / numeratorSigns.Length;
        int divisorWords = divisor.Length / divisorSigns.Length;
        int outputWords = quotient.Length / quotientSigns.Length;
        int bits = Math.Max(GetFiniteRootCoefficientBits(numerator, numeratorSigns.Length),
            GetFiniteRootCoefficientBits(divisor, divisorSigns.Length));
        // A partial remainder is divisor times the unprocessed quotient.
        // Factor height and at most eight summands bound its coefficients by
        // 2*bits+32 bits. Exact integer division introduces no rational field.
        int words = (2 * bits + 95) / 64;
        Span<ulong> remainder = stackalloc ulong[(degree + 1) * words];
        Span<sbyte> remainderSigns = stackalloc sbyte[8];
        Span<ulong> term = stackalloc ulong[words];
        Span<ulong> product = stackalloc ulong[words];
        Span<ulong> integerRemainder = stackalloc ulong[words];
        Span<ulong> division = stackalloc ulong[2 * words + 1];
        for (int index = 0; index <= degree; index++)
        {
            CopyFiniteRootMagnitude(numerator.Slice(index * inputWords, inputWords),
                remainder.Slice(index * words, words));
            remainderSigns[index] = numeratorSigns[index];
        }
        ReadOnlySpan<ulong> leading = divisor.Slice(divisorDegree * divisorWords, divisorWords);
        for (int index = degree; index >= divisorDegree; index--)
        {
            int sign = remainderSigns[index];
            if (sign == 0)
                continue;
            int shift = index - divisorDegree;
            WideArithmetic.DivideMagnitudes(remainder.Slice(index * words, words), leading,
                term, integerRemainder, division);
            System.Diagnostics.Debug.Assert(GetRoundedCylinderWideLength(integerRemainder) == 0);
            CopyFiniteRootMagnitude(term, quotient.Slice(shift * outputWords, outputWords));
            quotientSigns[shift] = (sbyte)sign;
            for (int coefficient = 0; coefficient <= divisorDegree; coefficient++)
            {
                if (divisorSigns[coefficient] == 0)
                    continue;
                WideArithmetic.MultiplyMagnitudes(term,
                    divisor.Slice(coefficient * divisorWords, divisorWords), product);
                Span<ulong> current = remainder.Slice((shift + coefficient) * words, words);
                int currentSign = remainderSigns[shift + coefficient];
                WideArithmetic.AddShiftedSignedMagnitude(product,
                    -sign * divisorSigns[coefficient], 0, current, ref currentSign);
                remainderSigns[shift + coefficient] = (sbyte)currentSign;
            }
        }
        System.Diagnostics.Debug.Assert(GetRoundedCylinderWideLength(remainder) == 0);
        // Both operands are primitive and positive-leading. Gauss's lemma
        // makes their exact quotient primitive and positive-leading too.
        return degree - divisorDegree;
    }
}
