//=======================================================================
// MIT License, Copyright (c) 2024-present David Oravsky (mrdav30)
// See LICENSE file in the project root for full license information.
//=======================================================================
using System;

namespace FixedMathSharp.Geometry;

/// <content>
/// Exact sign-at-root fallback via the signature of a four-by-four Hermite
/// trace form. This also detects equality at multiple algebraic roots.
/// </content>
internal static partial class WideFiniteAxisIntersection
{
    private static int GetFiniteRootHermiteIntervalSign(FiniteAxisPolynomialRoot root,
        ReadOnlySpan<ulong> query, ReadOnlySpan<sbyte> querySigns)
    {
        int degree = querySigns.Length - 1;
        int queryWords = query.Length / querySigns.Length;
        int boundBits = Math.Max(GetFiniteRootBits(root.LowerNumerator) + 1
            + Math.Max(-root.DenominatorShift, 0), Math.Max(root.DenominatorShift, 0) + 1);
        int words = (GetFiniteRootCoefficientBits(query, querySigns.Length) + boundBits + 65) / 64;
        Span<ulong> weighted = stackalloc ulong[(degree + 2) * words];
        Span<sbyte> weightedSigns = stackalloc sbyte[degree + 2];
        Span<ulong> bound = stackalloc ulong[words];
        Span<ulong> first = stackalloc ulong[words];
        Span<ulong> second = stackalloc ulong[words];
        int lowerQuery = 0;
        int upperQuery = 0;
        for (int endpoint = 0; endpoint < 2; endpoint++)
        {
            CopyFiniteRootMagnitude(root.LowerNumerator, bound);
            if (endpoint != 0)
                AddRoundedCylinderWord(bound, 0, 1);
            if (root.DenominatorShift < 0)
                ShiftFiniteRootLeft(bound, -root.DenominatorShift);
            weighted.Clear();
            weightedSigns.Clear();
            for (int index = 0; index <= degree + 1; index++)
            {
                first.Clear();
                second.Clear();
                sbyte firstSign = 0;
                sbyte secondSign = 0;
                if (index != 0)
                {
                    CopyFiniteRootMagnitude(query.Slice((index - 1) * queryWords, queryWords), first);
                    if (root.DenominatorShift > 0)
                        ShiftFiniteRootLeft(first, root.DenominatorShift);
                    firstSign = querySigns[index - 1];
                }
                if (index <= degree)
                {
                    MultiplyRoundedCylinderWide(query.Slice(index * queryWords, queryWords), bound, second);
                    // Both retained-cell endpoints are strictly positive.
                    secondSign = querySigns[index];
                }
                SubtractRoundedCylinderSigned(first, firstSign, second, secondSign,
                    weighted.Slice(index * words, words), out weightedSigns[index]);
            }
            int signature = GetFiniteRootHermiteSignature(root, weighted, weightedSigns);
            if (endpoint == 0)
                lowerQuery = signature;
            else
                upperQuery = signature;
        }
        // For every other real root, sign(x-a)-sign(x-b) cancels. The
        // sole root in (a,b) contributes twice its query sign. Endpoints
        // are nonroots by the retained-cell invariant.
        return (lowerQuery - upperQuery) / 2;
    }

    private static int GetFiniteRootHermiteSignature(FiniteAxisPolynomialRoot root,
        ReadOnlySpan<ulong> query, ReadOnlySpan<sbyte> querySigns)
    {
        int degree = root.Signs.Length - 1;
        int queryDegree = querySigns.Length - 1;
        int maximumTrace = queryDegree + 2 * degree - 2;
        int polynomialWords = root.Coefficients.Length / root.Signs.Length;
        int queryWords = query.Length / querySigns.Length;
        int pBits = root.CoefficientBits;
        int qBits = GetFiniteRootCoefficientBits(query, querySigns.Length);

        // Let c>0 be the leading coefficient. S_k=c^k sum(roots^k)
        // is integral by Newton identities. Cauchy's bound gives
        // |S_k| <= 4(2H)^k, including complex roots and multiplicities.
        // H_ij=sum q_l S_(i+j+l)c^(K-i-j-l) is c^K times the
        // Hermite matrix. Thus entries use <=qBits+K(pBits+1)+7 bits;
        // every <=4 determinant (<=24 terms) fits 4*entryBits+8 bits.
        // All spans use these actual widths, not the caller's padded slots.
        int traceWords = (maximumTrace * (pBits + 1) + 71) / 64;
        int entryBits = qBits + maximumTrace * (pBits + 1) + 7;
        int entryWords = (entryBits + 63) / 64;
        Span<ulong> powers = stackalloc ulong[(maximumTrace + 1) * traceWords];
        Span<ulong> traces = stackalloc ulong[(maximumTrace + 1) * traceWords];
        Span<sbyte> traceSigns = stackalloc sbyte[maximumTrace + 1];
        powers.Clear();
        traces.Clear();
        traceSigns.Clear();
        powers[0] = 1;
        ReadOnlySpan<ulong> leading = root.Coefficients.Slice(degree * polynomialWords, polynomialWords);
        for (int exponent = 1; exponent <= maximumTrace; exponent++)
            MultiplyRoundedCylinderWide(powers.Slice((exponent - 1) * traceWords, traceWords),
                leading, powers.Slice(exponent * traceWords, traceWords));
        BuildFiniteRootTraces(root, powers, traces, traceSigns, traceWords);

        Span<ulong> matrix = stackalloc ulong[degree * (degree + 1) / 2 * entryWords];
        Span<sbyte> matrixSigns = stackalloc sbyte[degree * (degree + 1) / 2];
        Span<ulong> first = stackalloc ulong[entryWords];
        Span<ulong> second = stackalloc ulong[entryWords];
        Span<ulong> sum = stackalloc ulong[entryWords];
        matrix.Clear();
        matrixSigns.Clear();
        for (int row = 0; row < degree; row++)
        for (int column = row; column < degree; column++)
        {
            int slot = GetFiniteRootMatrixIndex(degree, row, column);
            Span<ulong> entry = matrix.Slice(slot * entryWords, entryWords);
            for (int power = 0; power <= queryDegree; power++)
            {
                int trace = row + column + power;
                MultiplyRoundedCylinderWide(query.Slice(power * queryWords, queryWords),
                    traces.Slice(trace * traceWords, traceWords), first);
                MultiplyRoundedCylinderWide(first,
                    powers.Slice((maximumTrace - trace) * traceWords, traceWords), second);
                sbyte sign = (sbyte)(querySigns[power] * traceSigns[trace]);
                AddRoundedCylinderSigned(entry, matrixSigns[slot], second, sign, sum, out matrixSigns[slot]);
                sum.CopyTo(entry);
            }
        }
        return GetFiniteRootSymmetricSignature(matrix, matrixSigns, degree, entryBits);
    }

    private static void BuildFiniteRootTraces(FiniteAxisPolynomialRoot root,
        ReadOnlySpan<ulong> powers, Span<ulong> traces, Span<sbyte> signs, int words)
    {
        int degree = root.Signs.Length - 1;
        int polynomialWords = root.Coefficients.Length / root.Signs.Length;
        Span<ulong> first = stackalloc ulong[words];
        Span<ulong> second = stackalloc ulong[words];
        Span<ulong> sum = stackalloc ulong[words];
        traces[0] = (ulong)degree;
        signs[0] = 1;
        for (int exponent = 1; exponent < signs.Length; exponent++)
        {
            Span<ulong> result = traces.Slice(exponent * words, words);
            for (int index = 1; index <= Math.Min(exponent, degree); index++)
            {
                MultiplyRoundedCylinderWide(root.Coefficients.Slice((degree - index) * polynomialWords, polynomialWords),
                    powers.Slice((index - 1) * words, words), first);
                sbyte sign = (sbyte)(-root.Signs[degree - index] * root.Signs[degree]);
                if (index == exponent)
                    MultiplyRoundedCylinderWideByWord(first, (ulong)exponent, second);
                else
                {
                    MultiplyRoundedCylinderWide(first, traces.Slice((exponent - index) * words, words), second);
                    sign = (sbyte)(sign * signs[exponent - index]);
                }
                AddRoundedCylinderSigned(result, signs[exponent], second, sign, sum, out signs[exponent]);
                sum.CopyTo(result);
            }
        }
    }

    private static int GetFiniteRootSymmetricSignature(ReadOnlySpan<ulong> matrix,
        ReadOnlySpan<sbyte> signs, int degree, int entryBits)
    {
        int entryWords = matrix.Length / signs.Length;
        int words = (degree * entryBits + 79) / 64;
        Span<ulong> coefficient = stackalloc ulong[words];
        Span<ulong> product = stackalloc ulong[words];
        Span<ulong> scratch = stackalloc ulong[words];
        Span<ulong> sum = stackalloc ulong[words];
        Span<int> indices = stackalloc int[4];
        Span<int> permutation = stackalloc int[4];
        int positiveVariations = 0;
        int negativeVariations = 0;
        int previousPositive = 1;
        int previousNegative = 1;
        for (int order = 1; order <= degree; order++)
        {
            coefficient.Clear();
            sbyte coefficientSign = 0;
            // The coefficient is the sum of order-by-order principal minors.
            // There are at most 15 nonempty subsets and 24 permutations.
            for (int mask = 1; mask < (1 << degree); mask++)
            {
                int length = 0;
                for (int index = 0; index < degree; index++)
                {
                    if ((mask & (1 << index)) != 0)
                        indices[length++] = index;
                }
                if (length != order)
                    continue;
                for (int index = 0; index < order; index++)
                    permutation[index] = index;
                do
                {
                    product.Clear();
                    product[0] = 1;
                    sbyte productSign = 1;
                    for (int row = 0; row < order; row++)
                    {
                        int slot = GetFiniteRootMatrixIndex(degree, indices[row], indices[permutation[row]]);
                        MultiplyRoundedCylinderWide(product, matrix.Slice(slot * entryWords, entryWords), scratch);
                        scratch.CopyTo(product);
                        productSign = (sbyte)(productSign * signs[slot]);
                        for (int previous = 0; previous < row; previous++)
                        {
                            if (permutation[previous] > permutation[row])
                                productSign = (sbyte)-productSign;
                        }
                    }
                    AddRoundedCylinderSigned(coefficient, coefficientSign, product, productSign, sum, out coefficientSign);
                    sum.CopyTo(coefficient);
                } while (MoveNextFiniteRootPermutation(permutation[..order]));
            }
            if (coefficientSign == 0)
                continue;
            int positiveSign = (order & 1) != 0 ? -coefficientSign : coefficientSign;
            if (previousPositive != positiveSign)
                positiveVariations++;
            if (previousNegative != coefficientSign)
                negativeVariations++;
            previousPositive = positiveSign;
            previousNegative = coefficientSign;
        }
        // A real symmetric matrix has only real eigenvalues, so Descartes'
        // variation counts for det(tI-H) and det(tI+H) are exact, not bounds.
        // Hermite's signature theorem counts DISTINCT real roots weighted by
        // sign(Q). Repeated roots contribute positive multiplicity to the
        // rank-one form; complex conjugate blocks have signature zero.
        return positiveVariations - negativeVariations;
    }

    private static int GetFiniteRootMatrixIndex(int degree, int row, int column)
    {
        if (row > column)
        {
            int temporary = row;
            row = column;
            column = temporary;
        }
        return row * (2 * degree - row + 1) / 2 + column - row;
    }

    private static bool MoveNextFiniteRootPermutation(Span<int> permutation)
    {
        int pivot = permutation.Length - 2;
        while (pivot >= 0 && permutation[pivot] >= permutation[pivot + 1])
            pivot--;
        if (pivot < 0)
            return false;
        int successor = permutation.Length - 1;
        while (permutation[successor] <= permutation[pivot])
            successor--;
        int temporary = permutation[pivot];
        permutation[pivot] = permutation[successor];
        permutation[successor] = temporary;
        for (int left = pivot + 1, right = permutation.Length - 1; left < right; left++, right--)
        {
            temporary = permutation[left];
            permutation[left] = permutation[right];
            permutation[right] = temporary;
        }
        return true;
    }
}
