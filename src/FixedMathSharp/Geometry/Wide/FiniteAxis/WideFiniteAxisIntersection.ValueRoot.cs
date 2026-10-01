//=======================================================================
// MIT License, Copyright (c) 2024-present David Oravsky (mrdav30)
// See LICENSE file in the project root for full license information.
//=======================================================================
using System;

namespace FixedMathSharp.Geometry;

/// <content>
/// All-root value isolation, separate from the optimized largest-root quartic
/// contract. Signed raw subresultants preserve exact Sturm variations.
/// </content>
internal static partial class WideFiniteAxisIntersection
{
    internal static int GetFiniteValueRootCellWords(ReadOnlySpan<ulong> coefficients,
        ReadOnlySpan<sbyte> signs) =>
        (16 * (GetFiniteRootCoefficientBits(coefficients, signs.Length) + 11) + 255) / 64;

    internal static bool TryGetFiniteValueRoot(ReadOnlySpan<ulong> coefficients,
        ReadOnlySpan<sbyte> signs, int ordinal, Span<ulong> cell, out FiniteAxisValueRoot root)
    {
        root = default;
        int degree = signs.Length - 1;
        while (degree >= 0 && signs[degree] == 0)
            degree--;
        if (degree <= 0)
            return false;
        int words = coefficients.Length / signs.Length;
        coefficients = coefficients[..((degree + 1) * words)];
        signs = signs[..(degree + 1)];
        int bits = GetFiniteRootCoefficientBits(coefficients, signs.Length);
        // Graded surviving-coefficient slots avoid the uniform-width
        // pseudo-remainder blow-up. Construction scratch is released before
        // endpoint evaluation; neither phase holds another chain.
        Span<ulong> arena = stackalloc ulong[(880 * (bits + 64) + 63) / 64 + 512];
        Span<int> offsets = stackalloc int[9];
        Span<int> widths = stackalloc int[9];
        Span<int> degrees = stackalloc int[9];
        Span<sbyte> sturmSigns = stackalloc sbyte[81];
        int count = BuildFiniteValueSturm(coefficients, signs, arena,
            offsets, widths, degrees, sturmSigns, out int retainedWords);
        int rootCount = GetFiniteValueRootCount(cell, arena[..retainedWords], offsets, widths,
            degrees, sturmSigns, count, arena[retainedWords..], out int lowerVariations);
        if (ordinal < 0 || ordinal >= rootCount)
            return false;
        root = IsolateFiniteValueRoot(coefficients, signs, ordinal, rootCount, lowerVariations, cell, int.MaxValue,
            arena[..retainedWords], offsets, widths, degrees, sturmSigns, count,
            arena[retainedWords..], out _);
        return true;
    }

    internal static FiniteAxisValueRoots GetFiniteValueRoots(ReadOnlySpan<ulong> coefficients,
        ReadOnlySpan<sbyte> signs, Span<ulong> cells, Span<int> shifts)
    {
        bool compact = TryGetFiniteValueRoots(coefficients, signs, cells, shifts,
            out int count, out int rationalMask, out bool repeated);
        return new FiniteAxisValueRoots(cells, shifts, count, rationalMask, compact, repeated);
    }

    /// <summary>
    /// Isolates all distinct roots in (0,1] with one Sturm construction.
    /// Cells supplies eight equal, nonempty numerator slots and shifts supplies
    /// eight entries. Bit ordinal of rationalMask identifies singleton cells.
    /// Count is the total number of distinct roots on both outcomes. Returns
    /// false when compact storage is insufficient; discard all cell metadata
    /// on false and materialize valid ordinals with full-size storage.
    /// The hasRepeatedRoots flag concerns the whole polynomial, including roots
    /// outside (0,1], and stays valid on capacity failure. Zero and constant
    /// polynomials report no repeated roots.
    /// Inputs must not overlap outputs. Storage affects reuse, not semantics.
    /// </summary>
    internal static bool TryGetFiniteValueRoots(ReadOnlySpan<ulong> coefficients,
        ReadOnlySpan<sbyte> signs, Span<ulong> cells, Span<int> shifts,
        out int count, out int rationalMask, out bool hasRepeatedRoots)
    {
        count = rationalMask = 0;
        hasRepeatedRoots = false;
        cells.Clear();
        shifts.Clear();
        int degree = signs.Length - 1;
        while (degree >= 0 && signs[degree] == 0)
            degree--;
        if (degree <= 0)
            return true;
        int inputWords = coefficients.Length / signs.Length;
        coefficients = coefficients[..((degree + 1) * inputWords)];
        signs = signs[..(degree + 1)];
        int cellWords = cells.Length / 8;
        System.Diagnostics.Debug.Assert(cellWords > 0 && cells.Length == 8 * cellWords && shifts.Length == 8);
        int bits = GetFiniteRootCoefficientBits(coefficients, signs.Length);
        if (TryGetFiniteValueRootsBernstein(coefficients, signs, cells, shifts, out count))
            return true;
        // The same arena as single-root isolation; the only additional live
        // caller state is the compact output (544 bytes for eight-word cells).
        Span<ulong> arena = stackalloc ulong[(880 * (bits + 64) + 63) / 64 + 512];
        Span<int> offsets = stackalloc int[9];
        Span<int> widths = stackalloc int[9];
        Span<int> degrees = stackalloc int[9];
        Span<sbyte> sturmSigns = stackalloc sbyte[81];
        int chainCount = BuildFiniteValueSturm(coefficients, signs, arena,
            offsets, widths, degrees, sturmSigns, out int retainedWords);
        // The last nonzero row is gcd(F,F') up to a nonzero scalar. Its
        // degree answers this globally without normalizing or exporting it.
        hasRepeatedRoots = degrees[chainCount - 1] > 0;
        count = GetFiniteValueRootCount(cells[..cellWords], arena[..retainedWords], offsets,
            widths, degrees, sturmSigns, chainCount, arena[retainedWords..], out int lowerVariations);
        for (int ordinal = 0; ordinal < count; ordinal++)
        {
            FiniteAxisValueRoot root = IsolateFiniteValueRoot(coefficients, signs, ordinal, count, lowerVariations,
                    cells.Slice(ordinal * cellWords, cellWords), cellWords * 64 - 1,
                    arena[..retainedWords], offsets, widths, degrees, sturmSigns, chainCount,
                    arena[retainedWords..], out bool fits);
            if (!fits)
            {
                rationalMask = 0;
                return false;
            }
            shifts[ordinal] = root.DenominatorShift;
            if (root.IsRational)
                rationalMask |= 1 << ordinal;
        }
        return true;
    }

    // The caller obtained rootCount for this exact polynomial and supplies a
    // valid ordinal and GetFiniteValueRootCellWords-sized storage. The shared
    // isolation bound therefore completes without a capacity-failure route.
    internal static FiniteAxisValueRoot GetKnownFiniteValueRoot(ReadOnlySpan<ulong> coefficients,
        ReadOnlySpan<sbyte> signs, int ordinal, int rootCount, Span<ulong> cell)
    {
        int bits = GetFiniteRootCoefficientBits(coefficients, signs.Length);
        Span<ulong> arena = stackalloc ulong[(880 * (bits + 64) + 63) / 64 + 512];
        Span<int> offsets = stackalloc int[9];
        Span<int> widths = stackalloc int[9];
        Span<int> degrees = stackalloc int[9];
        Span<sbyte> sturmSigns = stackalloc sbyte[81];
        int count = BuildFiniteValueSturm(coefficients, signs, arena,
            offsets, widths, degrees, sturmSigns, out int retainedWords);
        cell.Clear();
        int lowerVariations = GetFiniteValueVariations(arena[..retainedWords], offsets, widths,
            degrees, sturmSigns, count, cell, 0, arena[retainedWords..]);
        return IsolateFiniteValueRoot(coefficients, signs, ordinal, rootCount, lowerVariations, cell, int.MaxValue,
            arena[..retainedWords], offsets, widths, degrees, sturmSigns, count,
            arena[retainedWords..], out _);
    }

    private static int GetFiniteValueRootCount(Span<ulong> cell, ReadOnlySpan<ulong> chain,
        ReadOnlySpan<int> offsets, ReadOnlySpan<int> widths, ReadOnlySpan<int> degrees,
        ReadOnlySpan<sbyte> sturmSigns, int count, Span<ulong> evaluation, out int lowerVariations)
    {
        cell.Clear();
        lowerVariations = GetFiniteValueVariations(chain, offsets, widths,
            degrees, sturmSigns, count, cell, 0, evaluation);
        cell[0] = 1;
        return lowerVariations - GetFiniteValueVariations(chain, offsets, widths,
            degrees, sturmSigns, count, cell, 0, evaluation);
    }

    private static FiniteAxisValueRoot IsolateFiniteValueRoot(ReadOnlySpan<ulong> coefficients,
        ReadOnlySpan<sbyte> signs, int ordinal, int rootCount, int lowerVariations, Span<ulong> cell, int maximumShift,
        scoped ReadOnlySpan<ulong> chain, scoped ReadOnlySpan<int> offsets,
        scoped ReadOnlySpan<int> widths, scoped ReadOnlySpan<int> degrees,
        scoped ReadOnlySpan<sbyte> sturmSigns, int count, scoped Span<ulong> evaluation,
        out bool fits)
    {
        fits = true;
        cell.Clear();
        cell[0] = 1;
        int remaining = rootCount;
        var root = new FiniteAxisValueRoot
        {
            Coefficients = coefficients,
            Signs = signs,
            LowerNumerator = cell,
            Ordinal = ordinal
        };
        if (ordinal == remaining - 1
            && EvaluateFiniteRootPolynomial(coefficients, signs, cell, 0, 0, evaluation) == 0)
        {
            root.IsRational = true;
            return root;
        }
        cell.Clear();

        // Distinct roots of degree-eight integer polynomials of height B,
        // including their factors, have separation >2^(-16*(B+11)-89).
        // The physical value interval is (0,1], so the retained numerator
        // needs no extra Cauchy-scale integer part. Zero is excluded.
        while (remaining != 1 || GetRoundedCylinderWideLength(cell) == 0
            || EvaluateFiniteRootPolynomial(coefficients, signs, cell, root.DenominatorShift, 0, evaluation) == 0
            || GetFiniteValueCellUpperSign(root, evaluation) == 0)
        {
            // For x in (0,1), a shift q needs at most q+1 numerator bits
            // including the upper endpoint. Stop before shifting beyond the
            // compact slot, even when a tiny numerator could fit farther.
            // Single-root callers supply the proven full isolation capacity.
            if (root.DenominatorShift >= maximumShift)
            {
                fits = false;
                return root;
            }
            RefineFiniteValueCell(ref root, chain, offsets, widths, degrees, sturmSigns,
                count, evaluation, ref ordinal, ref remaining, ref lowerVariations);
            if (root.IsRational)
                return root;
        }
        return root;
    }

    /// <summary>
    /// Refines an already isolated root without changing its ordinal or input
    /// polynomial. Caller-owned numerator storage must preserve the existing
    /// cell and provide at least targetShift+1 bits for the refined cell and
    /// its upper endpoint. Rational roots and already finer cells are unchanged.
    /// An optional knownLowerSign of -1 or +1 certifies that this same cell has
    /// opposite nonzero endpoint signs; zero requests exact endpoint evaluation.
    /// </summary>
    internal static void RefineFiniteValueRoot(ref FiniteAxisValueRoot root, int targetShift, int knownLowerSign = 0)
    {
        if (root.IsRational || root.DenominatorShift >= targetShift)
            return;
        int lowerSign = knownLowerSign != 0 ? knownLowerSign : EvaluateFiniteRootPolynomial(root.Coefficients, root.Signs,
            root.LowerNumerator, root.DenominatorShift, 0);
        int upperSign = knownLowerSign != 0 ? -knownLowerSign : GetFiniteValueCellUpperSign(root);
        if (lowerSign != 0 && lowerSign == -upperSign)
        {
            // The retained cell contains exactly one distinct root. Opposite
            // nonzero endpoint signs therefore certify an odd crossing, even
            // for repeated roots, using the same step as the quartic owner.
            // No Sturm construction or multi-row evaluation is needed here.
            int coefficientBits = GetFiniteRootCoefficientBits(root.Coefficients, root.Signs.Length);
            RefineFiniteValueCrossingRoot(ref root, targetShift, lowerSign, coefficientBits);
            return;
        }
        int degree = root.Signs.Length - 1;
        int bits = GetFiniteRootCoefficientBits(root.Coefficients, degree + 1);
        int unitBits = bits + 64;
        int retainedBits = (3 * degree + 1) * unitBits;
        int evaluationBits = 0;
        for (int index = 0; index <= degree; index++)
        {
            int height = index >= degree - 1 ? unitBits : (2 * degree - 2 * index - 1) * unitBits;
            if (index < degree - 1)
                retainedBits += (index + 1) * height;
            evaluationBits = Math.Max(evaluationBits,
                2 * (height + index * (targetShift + 1) + 95) + height + 128);
        }
        // Raw subresultant minor bounds control the retained rows. A
        // refinement may need more precision than isolation, so separately
        // budget exact dyadic evaluation and reuse the construction arena.
        int arenaBits = Math.Max(880 * unitBits, retainedBits + evaluationBits);
        Span<ulong> arena = stackalloc ulong[(arenaBits + 63) / 64 + 512];
        Span<int> offsets = stackalloc int[9];
        Span<int> widths = stackalloc int[9];
        Span<int> degrees = stackalloc int[9];
        Span<sbyte> signs = stackalloc sbyte[81];
        int count = BuildFiniteValueSturm(root.Coefficients, root.Signs, arena,
            offsets, widths, degrees, signs, out int used);
        Span<ulong> chain = arena[..used];
        Span<ulong> evaluation = arena[used..];
        int lowerVariations = GetFiniteValueVariations(chain, offsets, widths,
            degrees, signs, count, root.LowerNumerator, root.DenominatorShift, evaluation);
        int ordinal = 0;
        int remaining = 1;
        while (!root.IsRational && root.DenominatorShift < targetShift)
            RefineFiniteValueCell(ref root, chain, offsets, widths, degrees, signs,
                count, evaluation, ref ordinal, ref remaining, ref lowerVariations);
    }

    private static void RefineFiniteValueCell(ref FiniteAxisValueRoot root,
        scoped ReadOnlySpan<ulong> chain, scoped ReadOnlySpan<int> offsets, scoped ReadOnlySpan<int> widths,
        scoped ReadOnlySpan<int> degrees, scoped ReadOnlySpan<sbyte> signs, int count,
        scoped Span<ulong> evaluation, ref int ordinal, ref int remaining, ref int lowerVariations)
    {
        ShiftFiniteRootLeft(root.LowerNumerator, 1);
        AddRoundedCylinderWord(root.LowerNumerator, 0, 1);
        root.DenominatorShift++;
        int midpointVariations = GetFiniteValueVariations(chain, offsets, widths,
            degrees, signs, count, root.LowerNumerator, root.DenominatorShift, evaluation);
        int leftCount = lowerVariations - midpointVariations;
        if (ordinal < leftCount)
        {
            if (ordinal == leftCount - 1
                && EvaluateFiniteRootPolynomial(root.Coefficients, root.Signs,
                    root.LowerNumerator, root.DenominatorShift, 0, evaluation) == 0)
                root.IsRational = true;
            else
                root.LowerNumerator[0]--; // Odd midpoint, no borrow.
            remaining = leftCount;
        }
        else
        {
            ordinal -= leftCount;
            remaining -= leftCount;
            lowerVariations = midpointVariations;
        }
    }

    private static int GetFiniteValueCellUpperSign(FiniteAxisValueRoot root, Span<ulong> evaluation = default)
    {
        AddRoundedCylinderWord(root.LowerNumerator, 0, 1);
        int sign = evaluation.IsEmpty
            ? EvaluateFiniteRootPolynomial(root.Coefficients, root.Signs,
                root.LowerNumerator, root.DenominatorShift, 0)
            : EvaluateFiniteRootPolynomial(root.Coefficients, root.Signs,
                root.LowerNumerator, root.DenominatorShift, 0, evaluation);
        for (int index = 0; index < root.LowerNumerator.Length; index++)
        {
            ulong previous = root.LowerNumerator[index];
            root.LowerNumerator[index] = unchecked(previous - 1);
            if (previous != 0)
                break;
        }
        return sign;
    }

    private static int BuildFiniteValueSturm(ReadOnlySpan<ulong> coefficients,
        ReadOnlySpan<sbyte> signs, Span<ulong> arena, Span<int> offsets,
        Span<int> widths, Span<int> degrees, Span<sbyte> sturmSigns, out int used)
    {
        sturmSigns.Clear();
        int degree = signs.Length - 1;
        int inputWords = coefficients.Length / signs.Length;
        int words = Math.Max(1, (GetFiniteRootCoefficientBits(coefficients, signs.Length) + 63) / 64);
        offsets[0] = 0;
        widths[0] = words;
        degrees[0] = degree;
        used = (degree + 1) * words;
        for (int index = 0; index <= degree; index++)
            CopyFiniteRootMagnitude(coefficients.Slice(index * inputWords, inputWords),
                arena.Slice(index * words, words));
        signs.CopyTo(sturmSigns);
        NormalizeFiniteValueContent(arena[..used], degree, words, sturmSigns[..9], arena[used..]);
        offsets[1] = used;
        widths[1] = words + 1;
        degrees[1] = degree - 1;
        used += degree * (words + 1);
        DifferentiateFiniteAxisPolynomial(arena[..offsets[1]], sturmSigns[..(degree + 1)],
            arena.Slice(offsets[1], degree * (words + 1)), sturmSigns.Slice(9, degree));
        NormalizeFiniteValueContent(arena.Slice(offsets[1], degree * (words + 1)),
            degree - 1, words + 1, sturmSigns.Slice(9, 9), arena[used..]);
        int count = 2;
        if (degree == 1)
            return count;

        // Brown's subresultant PRS uses exact scalar divisors instead of a
        // content GCD after every pseudo-remainder. Keep its raw signs until
        // construction ends, then apply the separate Sturm row orientations.
        // For C=B+64 and h(j)=2*n-1-2*j, the retained minors need <=221C
        // bits. In a pair of actual degrees d>e, the second row is the
        // subresultant of natural index d-1 even after an abnormal drop;
        // its height is <=h(d-1)C, not the looser h(e)C. Graded prem slots
        // plus two products peak at215C; exact-division scratch peaks172C.
        // Scalar powers/divisors need <=28C each. Eight such scratch slots
        // give a total <=660C, within880C plus the existing rounding padding.
        // Store raw Brown rows as S_i=2^rowPower[i]*T_i, and its scalars
        // as c=2^cPower*cOdd, beta=2^betaPower*betaOdd. Only positive
        // powers of two are removed, so row signs and Sturm variations stay
        // unchanged. Normalized retained rows/scalars only shrink; transient
        // quotients still use the existing graded pseudo-remainder storage.
        int unitBits = GetFiniteRootCoefficientBits(coefficients, signs.Length) + 64;
        int scalarWords = (28 * unitBits + 63) / 64;
        int scalarOffset = arena.Length - (8 * scalarWords + 1);
        Span<ulong> scalarWork = arena[scalarOffset..];
        Span<ulong> c = scalarWork[..scalarWords];
        Span<ulong> beta = scalarWork.Slice(scalarWords, scalarWords);
        CopyFiniteRootMagnitude(arena.Slice(offsets[1] + (degree - 1) * widths[1], widths[1]), c);
        int cPower = CountRoundedCylinderTrailingZeroes(c);
        ShiftRoundedCylinderWideRight(c, cPower);
        int cSign = -sturmSigns[9 + degree - 1];
        beta.Clear();
        beta[0] = 1; // Initial degree gap is one: beta=(-1)^(gap+1).
        int betaSign = 1;
        int betaPower = 0;
        Span<int> rowPowers = stackalloc int[9];
        rowPowers[0] = rowPowers[1] = 0;
        Span<sbyte> rowSigns = stackalloc sbyte[9];
        rowSigns[0] = rowSigns[1] = 1;
        while (degrees[count - 1] > 0)
        {
            int left = count - 2;
            int right = count - 1;
            int power = degrees[left] - degrees[right] + 1;
            int pseudoSign = (power & 1) == 0 ? 1 : sturmSigns[right * 9 + degrees[right]];
            int nextDegree = BuildFiniteValueSubresultant(
                arena.Slice(offsets[left], (degrees[left] + 1) * widths[left]),
                sturmSigns.Slice(left * 9, degrees[left] + 1),
                arena.Slice(offsets[right], (degrees[right] + 1) * widths[right]),
                sturmSigns.Slice(right * 9, degrees[right] + 1),
                beta, betaSign * pseudoSign, arena.Slice(used, scalarOffset - used),
                sturmSigns.Slice(count * 9, 9), out int nextWords, out int removedPower);
            if (nextDegree < 0)
                break;
            // prem is homogeneous: the omitted input scale is
            // rowPower[left]+power*rowPower[right]. After division by
            // betaOdd, stripping 2^removedPower leaves the raw row exponent
            // below. The difference before adding removedPower may be
            // negative; integrality guarantees only the final sum is >=0.
            rowPowers[count] = rowPowers[left] + power * rowPowers[right]
                - betaPower + removedPower;
            System.Diagnostics.Debug.Assert(rowPowers[count] >= 0);
            offsets[count] = used;
            widths[count] = nextWords;
            degrees[count] = nextDegree;
            // prem=lc(right)^(gap+1)*rem. Multiplying each completed raw
            // row by this sign makes it a positive multiple of -rem(Sturm).
            rowSigns[count] = (sbyte)(-rowSigns[left] * betaSign * pseudoSign);
            used += (nextDegree + 1) * nextWords;
            if (nextDegree > 0)
                UpdateFiniteValueSubresultantScalars(
                    arena.Slice(offsets[right] + degrees[right] * widths[right], widths[right]),
                    sturmSigns[right * 9 + degrees[right]], rowPowers[right],
                    arena.Slice(offsets[count] + nextDegree * nextWords, nextWords),
                    sturmSigns[count * 9 + nextDegree], rowPowers[count], degrees[right] - nextDegree,
                    scalarWork, scalarWords, ref cSign, ref cPower, out betaSign, out betaPower);
            count++;
        }
        for (int row = 2; row < count; row++)
            for (int index = 0; index <= degrees[row]; index++)
                sturmSigns[row * 9 + index] *= rowSigns[row];
        return count;
    }

    private static void UpdateFiniteValueSubresultantScalars(ReadOnlySpan<ulong> previousLeading,
        int previousLeadingSign, int previousRowPower, ReadOnlySpan<ulong> leading,
        int leadingSign, int rowPower, int gap, Span<ulong> work, int words,
        ref int cSign, ref int cPower, out int betaSign, out int betaPower)
    {
        Span<ulong> c = work[..words];
        Span<ulong> beta = work.Slice(words, words);
        Span<ulong> numerator = work.Slice(2 * words, words);
        Span<ulong> denominator = work.Slice(3 * words, words);
        Span<ulong> product = work.Slice(4 * words, words);
        // beta=-previousLeading*c^gap; c_next=(-leading)^gap/c^(gap-1).
        // Each leading coefficient has both the row's omitted power and
        // its stored leading term's power. Track those separately and use
        // only odd magnitudes in the unchanged scalar recurrence. The odd
        // denominator divides the odd numerator exactly, since it is coprime
        // to every omitted power of two, including abnormal degree drops.
        int previousLeadingPower = CountRoundedCylinderTrailingZeroes(previousLeading);
        int leadingPower = CountRoundedCylinderTrailingZeroes(leading);
        betaPower = previousRowPower + previousLeadingPower + gap * cPower;
        int nextCPower = gap * (rowPower + leadingPower) - (gap - 1) * cPower;
        System.Diagnostics.Debug.Assert(betaPower >= 0 && nextCPower >= 0);
        // Reuse the denominator slot for leading odd parts before it is
        // needed for cOdd^(gap-1); no extra coefficient buffer is retained.
        CopyFiniteRootMagnitude(previousLeading, denominator);
        ShiftRoundedCylinderWideRight(denominator, previousLeadingPower);
        PowerFiniteValueMagnitude(c, gap, numerator, product);
        WideArithmetic.MultiplyMagnitudes(denominator, numerator, beta);
        betaSign = -previousLeadingSign * ((gap & 1) == 0 ? 1 : cSign);
        if (gap == 1)
        {
            CopyFiniteRootMagnitude(leading, c);
            ShiftRoundedCylinderWideRight(c, leadingPower);
            cSign = -leadingSign;
            cPower = nextCPower;
            return;
        }
        CopyFiniteRootMagnitude(leading, denominator);
        ShiftRoundedCylinderWideRight(denominator, leadingPower);
        PowerFiniteValueMagnitude(denominator, gap, numerator, product);
        PowerFiniteValueMagnitude(c, gap - 1, denominator, product);
        int nextSign = ((gap & 1) == 0 ? 1 : -leadingSign)
            * ((gap & 1) == 0 ? cSign : 1);
        Span<ulong> remainder = work.Slice(5 * words, words);
        WideArithmetic.DivideMagnitudes(numerator, denominator, c, remainder, work[(6 * words)..]);
        System.Diagnostics.Debug.Assert(GetRoundedCylinderWideLength(remainder) == 0);
        cSign = nextSign;
        cPower = nextCPower;
    }

    private static void PowerFiniteValueMagnitude(ReadOnlySpan<ulong> value, int power,
        Span<ulong> result, Span<ulong> product)
    {
        result.Clear();
        result[0] = 1;
        for (int index = 0; index < power; index++)
        {
            WideArithmetic.MultiplyMagnitudes(result, value, product);
            product.CopyTo(result);
        }
    }

    private static int BuildFiniteValueSubresultant(ReadOnlySpan<ulong> left,
        ReadOnlySpan<sbyte> leftSigns, ReadOnlySpan<ulong> right,
        ReadOnlySpan<sbyte> rightSigns, ReadOnlySpan<ulong> beta, int resultSign,
        Span<ulong> work, Span<sbyte> resultSigns,
        out int resultWords, out int removedPower)
    {
        int d = leftSigns.Length - 1;
        int e = rightSigns.Length - 1;
        int leftWords = left.Length / (d + 1);
        int rightWords = right.Length / (e + 1);
        int leftBits = GetFiniteRootCoefficientBits(left, d + 1);
        int rightBits = GetFiniteRootCoefficientBits(right, e + 1);
        Span<int> offsets = stackalloc int[9];
        Span<int> widths = stackalloc int[9];
        Span<sbyte> signs = stackalloc sbyte[9];
        int used = 0;
        int maximumWords = 0;
        for (int index = 0; index <= d; index++)
        {
            int cancellations = Math.Min(d - e + 1, d - index);
            int width = (leftBits + cancellations * (rightBits + 1) + 127) / 64;
            offsets[index] = used;
            widths[index] = width;
            used += width;
            maximumWords = Math.Max(maximumWords, width);
            CopyFiniteRootMagnitude(left.Slice(index * leftWords, leftWords),
                work.Slice(offsets[index], width));
            signs[index] = leftSigns[index];
        }
        Span<ulong> product = work.Slice(used, maximumWords);
        Span<ulong> subtract = work.Slice(used + maximumWords, maximumWords);
        ReadOnlySpan<ulong> divisorLeading = right.Slice(e * rightWords, rightWords);
        for (int degree = d; degree >= e; degree--)
        {
            // Even a zero leading term must multiply every remaining slot:
            // Brown's prem uses exactly d-e+1 leading-coefficient factors.
            ReadOnlySpan<ulong> leading = work.Slice(offsets[degree], widths[degree]);
            int shift = degree - e;
            for (int index = 0; index < degree; index++)
            {
                Span<ulong> coefficient = work.Slice(offsets[index], widths[index]);
                WideArithmetic.MultiplyMagnitudes(coefficient, divisorLeading, product);
                int sign = signs[index];
                if (signs[degree] != 0 && index >= shift && rightSigns[index - shift] != 0)
                {
                    WideArithmetic.MultiplyMagnitudes(leading,
                        right.Slice((index - shift) * rightWords, rightWords), subtract);
                    WideArithmetic.AddShiftedSignedMagnitude(subtract,
                        -signs[degree] * rightSigns[e] * rightSigns[index - shift], 0, product, ref sign);
                }
                product[..widths[index]].CopyTo(coefficient);
                signs[index] = (sbyte)sign;
            }
            signs[degree] = 0;
        }
        int remainderDegree = e - 1;
        while (remainderDegree >= 0 && signs[remainderDegree] == 0)
            remainderDegree--;
        resultWords = 0;
        removedPower = 0;
        if (remainderDegree < 0)
            return -1;

        // All surviving slots have the same width. Higher discarded slots
        // become exact-division scratch; no pseudo-division products remain live.
        int remainderWords = widths[0];
        int remainderLength = (remainderDegree + 1) * remainderWords;
        Span<ulong> remainder = work[..remainderLength];
        // beta stores only its odd part. It divides this smaller prem
        // exactly: divisibility of the raw Brown numerator cannot depend on
        // its omitted power of two when the divisor is odd.
        int betaWords = GetRoundedCylinderWideLength(beta);
        if (betaWords != 1 || beta[0] != 1)
        {
            Span<ulong> integerRemainder = work.Slice(remainderLength, remainderWords);
            Span<ulong> division = work.Slice(remainderLength + remainderWords, 2 * remainderWords + 1);
            for (int index = 0; index <= remainderDegree; index++)
            {
                Span<ulong> coefficient = remainder.Slice(index * remainderWords, remainderWords);
                WideArithmetic.DivideMagnitudes(coefficient, beta[..betaWords], coefficient, integerRemainder, division);
                System.Diagnostics.Debug.Assert(GetRoundedCylinderWideLength(integerRemainder) == 0);
            }
        }
        // U=prem(T_left,T_right)/betaOdd can exceed the raw next row when
        // its omitted scale exponent is negative. It fits the prem scratch,
        // but must lose its common power of two BEFORE retained width/used
        // accounting; only the normalized T_next is bounded by the raw row.
        removedPower = NormalizeFiniteAxisPolynomialPowerOfTwo(
            remainder, signs[..(remainderDegree + 1)]);
        for (int index = 0; index <= remainderDegree; index++)
            resultWords = Math.Max(resultWords,
                GetRoundedCylinderWideLength(remainder.Slice(index * remainderWords, remainderWords)));
        for (int index = 0; index <= remainderDegree; index++)
        {
            remainder.Slice(index * remainderWords, resultWords)
                .CopyTo(work.Slice(index * resultWords, resultWords));
            resultSigns[index] = (sbyte)(resultSign * signs[index]);
        }
        return remainderDegree;
    }

    private static void NormalizeFiniteValueContent(Span<ulong> coefficients, int degree,
        int words, ReadOnlySpan<sbyte> signs, Span<ulong> scratch)
    {
        if (degree == 0)
        {
            coefficients.Clear();
            coefficients[0] = 1;
            return;
        }
        // Variable scaling introduces large shared powers of two in later
        // remainders too. Strip those exactly before Euclid works on the
        // remaining integer content; roots and signed variations are unchanged.
        NormalizeFiniteAxisPolynomialPowerOfTwo(coefficients, signs[..(degree + 1)]);
        Span<ulong> content = scratch[..words];
        Span<ulong> other = scratch.Slice(words, words);
        Span<ulong> division = scratch.Slice(2 * words, 2 * words + 1);
        content.Clear();
        for (int index = 0; index <= degree; index++)
        {
            if (signs[index] == 0)
                continue;
            coefficients.Slice(index * words, words).CopyTo(other);
            WideArithmetic.GetMagnitudeGreatestCommonDivisor(content, other, division);
            if (content[0] == 1 && GetRoundedCylinderWideLength(content) == 1)
                return;
        }
        for (int index = 0; index <= degree; index++)
        {
            Span<ulong> coefficient = coefficients.Slice(index * words, words);
            WideArithmetic.DivideMagnitudes(coefficient, content, coefficient, other, division);
            System.Diagnostics.Debug.Assert(GetRoundedCylinderWideLength(other) == 0);
        }
    }

    private static int GetFiniteValueVariations(ReadOnlySpan<ulong> chain,
        ReadOnlySpan<int> offsets, ReadOnlySpan<int> widths, ReadOnlySpan<int> degrees,
        ReadOnlySpan<sbyte> signs, int count, ReadOnlySpan<ulong> numerator, int shift,
        Span<ulong> evaluation)
    {
        int previous = 0;
        int variations = 0;
        for (int index = 0; index < count; index++)
        {
            int sign = 0;
            for (int derivative = 0; derivative <= degrees[index] && sign == 0; derivative++)
                sign = EvaluateFiniteRootPolynomial(
                    chain.Slice(offsets[index], (degrees[index] + 1) * widths[index]),
                    signs.Slice(index * 9, degrees[index] + 1), numerator, shift, derivative, evaluation);
            if (previous != 0 && previous != sign)
                variations++;
            previous = sign;
        }
        return variations;
    }
}
