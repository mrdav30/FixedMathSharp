//=======================================================================
// MIT License, Copyright (c) 2024-present David Oravsky (mrdav30)
// See LICENSE file in the project root for full license information.
//=======================================================================
using System;
using System.Runtime.CompilerServices;

namespace FixedMathSharp.Geometry;

/// <content>Bounded exact certificates before general Sturm isolation.</content>
internal static partial class WideFiniteAxisIntersection
{
    private const int FiniteValueBernsteinDepth = 32;
    private const int FiniteValueBernsteinNodes = 256;
    private const uint FiniteValueSquareFreePrime = 65537;

    // This frame must return before the caller allocates its Sturm arena.
    // Failure is inconclusive and discards partial cells, never a root count.
    [MethodImpl(MethodImplOptions.NoInlining)]
    private static bool TryGetFiniteValueRootsBernstein(ReadOnlySpan<ulong> coefficients,
        ReadOnlySpan<sbyte> signs, Span<ulong> cells, Span<int> shifts, out int count)
    {
        count = 0;
        if (!HasFiniteValueSquareFreeCertificate(coefficients, signs))
            return false;

        int degree = signs.Length - 1;
        int coefficientCount = degree + 1;
        int inputWords = coefficients.Length / coefficientCount;
        int cellWords = cells.Length / 8;
        int bits = GetFiniteRootCoefficientBits(coefficients, coefficientCount);
        // n!*b_j=sum_i<=j a_i*C(j,i)*i!*(n-i)! is integral and
        // has height <B+20 for n<=8. A midpoint split multiplies child
        // Bernstein coefficients by the common positive scale 2^n, adding
        // at most n bits per level. Thus B+20+32*n bits suffice everywhere.
        // Current/left plus32 pending right rows require34*(n+1) slots:
        // <=296208 bytes at B<=7424, plus <1KiB signs/indices. The existing
        // containing frames and64KiB caller headroom remain below1MiB;
        // this storage is never live with the general880C Sturm arena.
        int words = (bits + 20 + FiniteValueBernsteinDepth * degree + 63) / 64;
        int rowWords = coefficientCount * words;
        Span<ulong> storage = stackalloc ulong[(FiniteValueBernsteinDepth + 2) * rowWords];
        Span<sbyte> signStorage = stackalloc sbyte[(FiniteValueBernsteinDepth + 2) * coefficientCount];
        Span<ulong> pendingNumerators = stackalloc ulong[FiniteValueBernsteinDepth];
        Span<int> pendingShifts = stackalloc int[FiniteValueBernsteinDepth];
        Span<ulong> current = storage[..rowWords];
        Span<sbyte> currentSigns = signStorage[..coefficientCount];
        Span<ulong> left = storage.Slice(rowWords, rowWords);
        Span<sbyte> leftSigns = signStorage.Slice(coefficientCount, coefficientCount);
        current.Clear();
        currentSigns.Clear();
        int factorial = 1;
        for (int index = 2; index <= degree; index++)
            factorial *= index;
        for (int index = 0; index <= degree; index++)
        {
            int weight = factorial;
            int sign = 0;
            Span<ulong> destination = current.Slice(index * words, words);
            for (int source = 0; source <= index; source++)
            {
                if (signs[source] != 0)
                {
                    MultiplyRoundedCylinderWideByWord(coefficients.Slice(source * inputWords, inputWords),
                        (ulong)weight, left[..words]);
                    WideArithmetic.AddShiftedSignedMagnitude(left[..words], signs[source], 0, destination, ref sign);
                }
                if (source < index)
                    weight = weight * (index - source) / (degree - source);
            }
            currentSigns[index] = (sbyte)sign;
        }

        int pending = 0;
        ulong lower = 0;
        int shift = 0;
        for (int visited = 0; visited < FiniteValueBernsteinNodes; visited++)
        {
            // Endpoints are exact Bernstein coefficients. Let Sturm own all
            // endpoint/singleton bookkeeping, including the excluded zero.
            if (currentSigns[0] == 0 || currentSigns[degree] == 0)
                break;
            int variations = 0;
            int previous = 0;
            for (int index = 0; index <= degree; index++)
            {
                int sign = currentSigns[index];
                if (sign == 0)
                    continue;
                if (previous != 0 && previous != sign)
                    variations++;
                previous = sign;
            }
            // Descartes in Bernstein form: variation0 certifies no interior
            // root; variation1 certifies exactly one, counting multiplicity.
            // Positive lower endpoints preserve the existing root contract.
            if (variations > 1 || variations == 1 && lower == 0)
            {
                if (shift == FiniteValueBernsteinDepth)
                    break;
                Span<ulong> right = storage.Slice((pending + 2) * rowWords, rowWords);
                Span<sbyte> rightSigns = signStorage.Slice((pending + 2) * coefficientCount, coefficientCount);
                SplitFiniteValueBernstein(current, currentSigns, left, leftSigns, right, rightSigns, words);
                if (leftSigns[degree] == 0) // Exact midpoint; do not lose a rational root.
                    break;
                pendingNumerators[pending] = 2 * lower + 1;
                pendingShifts[pending] = shift + 1;
                pending++;
                left.CopyTo(current);
                leftSigns.CopyTo(currentSigns);
                lower *= 2;
                shift++;
                continue;
            }
            if (variations == 1)
            {
                System.Diagnostics.Debug.Assert(count < degree);
                Span<ulong> destination = cells.Slice(count * cellWords, cellWords);
                destination.Clear();
                destination[0] = lower;
                shifts[count++] = shift;
            }
            if (pending == 0)
                return true;
            pending--;
            storage.Slice((pending + 2) * rowWords, rowWords).CopyTo(current);
            signStorage.Slice((pending + 2) * coefficientCount, coefficientCount).CopyTo(currentSigns);
            lower = pendingNumerators[pending];
            shift = pendingShifts[pending];
        }
        count = 0;
        return false;
    }

    private static void SplitFiniteValueBernstein(Span<ulong> triangle, Span<sbyte> triangleSigns,
        Span<ulong> left, Span<sbyte> leftSigns, Span<ulong> right, Span<sbyte> rightSigns, int words)
    {
        int degree = triangleSigns.Length - 1;
        // Integer de Casteljau: T^(r+1)_i=T^r_i+T^r_(i+1).
        // Child edges are2^(n-r)*T^r_0 and2^(n-r)*T^r_(n-r),
        // which uniformly scale both children's usual coefficients by2^n.
        for (int level = 0; level <= degree; level++)
        {
            if (level != 0)
                for (int index = 0; index <= degree - level; index++)
                {
                    int sign = triangleSigns[index];
                    WideArithmetic.AddShiftedSignedMagnitude(triangle.Slice((index + 1) * words, words),
                        triangleSigns[index + 1], 0, triangle.Slice(index * words, words), ref sign);
                    triangleSigns[index] = (sbyte)sign;
                }
            int edge = degree - level;
            Span<ulong> leftCoefficient = left.Slice(level * words, words);
            triangle[..words].CopyTo(leftCoefficient);
            ShiftFiniteRootLeft(leftCoefficient, edge);
            leftSigns[level] = triangleSigns[0];
            Span<ulong> rightCoefficient = right.Slice(edge * words, words);
            triangle.Slice(edge * words, words).CopyTo(rightCoefficient);
            ShiftFiniteRootLeft(rightCoefficient, edge);
            rightSigns[edge] = triangleSigns[edge];
        }
    }

    private static bool HasFiniteValueSquareFreeCertificate(ReadOnlySpan<ulong> coefficients,
        ReadOnlySpan<sbyte> signs)
    {
        int degree = signs.Length - 1;
        int words = coefficients.Length / signs.Length;
        Span<uint> left = stackalloc uint[9];
        Span<uint> right = stackalloc uint[9];
        for (int index = 0; index <= degree; index++)
        {
            int residue = 0;
            if (signs[index] != 0)
                for (int word = 0; word < words; word++)
                {
                    ulong value = coefficients[index * words + word];
                    // 2^16=-1 and2^64=1 modulo the Fermat prime65537.
                    residue = (residue + (int)(value & 65535) - (int)((value >> 16) & 65535)
                        + (int)((value >> 32) & 65535) - (int)(value >> 48)
                        + 2 * (int)FiniteValueSquareFreePrime) % (int)FiniteValueSquareFreePrime;
                }
            left[index] = (uint)(signs[index] < 0 && residue != 0
                ? (int)FiniteValueSquareFreePrime - residue : residue);
        }
        // Degree preservation and p>degree preserve both F and F'. If their
        // finite-field gcd is constant, the integer resultant is nonzero.
        // Failure is merely inconclusive: never claim that F repeats over Q.
        if (left[degree] == 0)
            return false;
        for (int index = 0; index < degree; index++)
            right[index] = (uint)((ulong)(index + 1) * left[index + 1] % FiniteValueSquareFreePrime);
        int leftDegree = degree;
        int rightDegree = degree - 1;
        while (rightDegree > 0)
        {
            // Pseudo-division over the field needs no inverses. Nonzero
            // leading multipliers cannot change the polynomial gcd.
            uint leading = right[rightDegree];
            for (int index = leftDegree; index >= rightDegree; index--)
            {
                uint cancel = left[index];
                int offset = index - rightDegree;
                for (int term = 0; term < index; term++)
                {
                    ulong value = (ulong)left[term] * leading % FiniteValueSquareFreePrime;
                    ulong subtract = term >= offset
                        ? (ulong)cancel * right[term - offset] % FiniteValueSquareFreePrime : 0;
                    left[term] = (uint)((value + FiniteValueSquareFreePrime - subtract) % FiniteValueSquareFreePrime);
                }
                left[index] = 0;
            }
            int remainderDegree = rightDegree - 1;
            while (remainderDegree >= 0 && left[remainderDegree] == 0)
                remainderDegree--;
            if (remainderDegree < 0)
                return false;
            Span<uint> swap = left;
            left = right;
            right = swap;
            leftDegree = rightDegree;
            rightDegree = remainderDegree;
        }
        return true;
    }
}
