//=======================================================================
// MIT License, Copyright (c) 2024-present David Oravsky (mrdav30)
// See LICENSE file in the project root for full license information.
//=======================================================================
using System;

namespace FixedMathSharp.Geometry;

/// <content>
/// Identifies a known radius-offset value root by exact mapped dyadic cells.
/// No polynomial composition or additional algebraic solver is required.
/// </content>
internal static partial class WideFiniteAxisIntersection
{
    /// <summary>
    /// Maps alpha to beta=(sqrt(alpha)+sourceGapSign*sqrt(radiusSquared/2^valueShift))².
    /// The degree-at-most-four source and degree-at-most-eight target use the
    /// same scaled variable. Beta must be a positive target root. The radius
    /// square is in [0,2^valueShift]; the original signed gap plus radius is
    /// positive, so a negative sourceGapSign implies alpha&lt;radiusSquared/2^valueShift.
    /// Known membership is a strict precondition, not an absence query. Source
    /// storage is unchanged; the returned root borrows the target storage.
    /// </summary>
    internal static FiniteAxisValueRoot MapKnownFiniteValueRootRadicalOffset(scoped FiniteAxisValueRoot source,
        int sourceGapSign, scoped ReadOnlySpan<ulong> radiusSquared, int valueShift,
        ReadOnlySpan<ulong> targetCoefficients, ReadOnlySpan<sbyte> targetSigns,
        Span<ulong> targetCell)
    {
        System.Diagnostics.Debug.Assert(source.Signs.Length <= 5 && Math.Abs(sourceGapSign) == 1);
        int sourceBits = GetFiniteRootCoefficientBits(source.Coefficients, source.Signs.Length);
        int targetBits = GetFiniteRootCoefficientBits(targetCoefficients, targetSigns.Length);
        int targetShift = 16 * (targetBits + 11) + 100;
        int sourceShift = targetShift + sourceBits + 8;
        Span<ulong> sourceCell = stackalloc ulong[(Math.Max(sourceShift, source.DenominatorShift) + 127) / 64];
        CopyFiniteRootMagnitude(source.LowerNumerator, sourceCell);
        source.LowerNumerator = sourceCell;

        Span<ulong> batchCells = stackalloc ulong[8 * 8];
        Span<int> batchShifts = stackalloc int[8];
        FiniteAxisValueRoots roots = GetFiniteValueRoots(targetCoefficients, targetSigns, batchCells, batchShifts);

        // Cauchy's reciprocal bound gives alpha>2^(-Bf-2); its positive
        // cell lower endpoint exceeds alpha/2. Hence |d beta/d alpha| is
        // <2^(Bf+4), even across the negative branch's clipped minimum.
        // After refinement the mapped width is <2^(-q-4), target width <=2^-q.
        // Distinct target roots have separation >2^(-q+11), so overlapping
        // closed cells certify equality, using the known target membership.
        for (int ordinal = 0; ordinal < roots.Count - 1; ordinal++)
        {
            FiniteAxisValueRoot candidate = FiniteAxisValueRoots.GetRoot(roots, ordinal, targetCoefficients, targetSigns, targetCell);
            int increment = 8;
            bool refineSource = true;
            int relation = ClassifyRadicalOffsetCells(source, sourceGapSign, radiusSquared, valueShift, candidate);
            // A singleton source maps to one known target root, so its exact
            // point is either contained by this cell or strictly beyond it.
            // Unresolved overlap therefore implies a non-singleton source.
            while (relation == 0 && source.DenominatorShift < sourceShift)
            {
                // Recheck containment after refining only the source. Refining
                // both cells together can keep an identity map's cells equal.
                if (refineSource)
                    RefineFiniteValueRoot(ref source, Math.Min(sourceShift, source.DenominatorShift + increment));
                else
                {
                    RefineFiniteValueRoot(ref candidate, Math.Min(targetShift, candidate.DenominatorShift + increment));
                    increment = Math.Min(2 * increment, sourceShift);
                }
                refineSource = !refineSource;
                relation = ClassifyRadicalOffsetCells(source, sourceGapSign, radiusSquared, valueShift, candidate);
            }
            if (relation == 0)
            {
                // The source now meets its width bound. One final target
                // refinement supplies the other bound; rational and already
                // finer cells are unchanged. Remaining overlap proves equality.
                RefineFiniteValueRoot(ref candidate, targetShift);
                relation = ClassifyRadicalOffsetCells(source, sourceGapSign, radiusSquared, valueShift, candidate);
            }
            if (relation >= 0)
                return candidate;
        }
        // Beta is a positive target root by contract. The authoritative count
        // and exact rejection of every earlier ordinal identify the last root
        // constructively, including when compact isolation exhausted capacity.
        return FiniteAxisValueRoots.GetRoot(roots, roots.Count - 1, targetCoefficients, targetSigns, targetCell);
    }

    // -1: disjoint; 0: unresolved overlap; 1: the known target root is identified.
    // A mapped interval strictly inside an isolating cell contains its only root.
    // A rational target instead requires exact equality of a mapped point.
    private static int ClassifyRadicalOffsetCells(FiniteAxisValueRoot source, int sign,
        ReadOnlySpan<ulong> radiusSquared, int valueShift, FiniteAxisValueRoot target)
    {
        bool sourceUpper = !source.IsRational;
        bool targetUpper = !target.IsRational;
        bool maximumUpper = sign > 0 && sourceUpper;
        bool minimumUpper = sign < 0 && sourceUpper;
        // Ascending known-member search has rejected only earlier ordinals,
        // hence targetRoot <= beta. The mapped maximum cannot precede this
        // candidate; only the mapped minimum can prove it is an earlier root.
        // The admitted negative branch decreases below the radius square.
        // If the source cell reaches its minimum, the mapped lower bound is
        // zero; do not map an endpoint on the other side as a lower bound.
        bool clipped = sign < 0 && CompareRadicalOffsetEndpoints(source.LowerNumerator, source.DenominatorShift, sourceUpper,
            default, 0, 1, radiusSquared, valueShift, false) >= 0;
        int minimumToUpper = clipped ? -1 : CompareRadicalOffsetEndpoints(source.LowerNumerator, source.DenominatorShift, minimumUpper,
            radiusSquared, valueShift, sign, target.LowerNumerator, target.DenominatorShift, targetUpper);
        if (minimumToUpper > 0)
            return -1;
        // With a singleton source, minimum=maximum=beta. The earlier guard
        // proves beta<=target, while ascending known membership gives the
        // reverse inequality; no second endpoint comparison is needed.
        if (target.IsRational)
            return source.IsRational ? 1 : 0;
        if (clipped)
            return 0;
        return CompareRadicalOffsetEndpoints(source.LowerNumerator, source.DenominatorShift, minimumUpper,
                radiusSquared, valueShift, sign, target.LowerNumerator, target.DenominatorShift, false) > 0
            && CompareRadicalOffsetEndpoints(source.LowerNumerator, source.DenominatorShift, maximumUpper,
                radiusSquared, valueShift, sign, target.LowerNumerator, target.DenominatorShift, true) < 0 ? 1 : 0;
    }

    // Endpoints are (numerator + upper)/2^shift. Aligning exponents at L
    // gives E=A+tau-G and C=A*tau. The comparison is sign(E+2*sign*sqrt(C)).
    // Source and target lower numerators are positive: retained root cells
    // exclude zero, and dyadic comparisons handle zero before calling here.
    // Endpoints lie in (0,1], so E fits L+2 bits and E²/4C fit 2L+4.
    // The five-L-bit scratch frame returns before either root arena is live.
    private static int CompareRadicalOffsetEndpoints(ReadOnlySpan<ulong> source, int sourceShift, bool sourceUpper,
        ReadOnlySpan<ulong> radiusSquared, int valueShift, int sign,
        ReadOnlySpan<ulong> target, int targetShift, bool targetUpper)
    {
        int shift = Math.Max(sourceShift, Math.Max(valueShift, targetShift));
        int words = (shift + 65) / 64;
        Span<ulong> rational = stackalloc ulong[words];
        Span<ulong> one = stackalloc ulong[1] { 1 };
        rational.Clear();
        int rationalSign = 0;
        int radiusSign = GetRoundedCylinderWideLength(radiusSquared) == 0 ? 0 : 1;
        WideArithmetic.AddShiftedSignedMagnitude(source, 1, shift - sourceShift, rational, ref rationalSign);
        if (sourceUpper)
            WideArithmetic.AddShiftedSignedMagnitude(one, 1, shift - sourceShift, rational, ref rationalSign);
        WideArithmetic.AddShiftedSignedMagnitude(radiusSquared, radiusSign, shift - valueShift, rational, ref rationalSign);
        WideArithmetic.AddShiftedSignedMagnitude(target, -1, shift - targetShift, rational, ref rationalSign);
        if (targetUpper)
            WideArithmetic.AddShiftedSignedMagnitude(one, -1, shift - targetShift, rational, ref rationalSign);
        int radicalSign = radiusSign != 0 ? sign : 0;
        if (rationalSign == 0)
            return radicalSign;
        if (radicalSign == 0 || rationalSign == radicalSign)
            return rationalSign;

        Span<ulong> rationalSquared = stackalloc ulong[2 * words];
        Span<ulong> radicalSquared = stackalloc ulong[2 * words];
        WideArithmetic.MultiplyMagnitudes(rational, rational, rationalSquared);
        WideArithmetic.MultiplyMagnitudes(source, radiusSquared, radicalSquared);
        if (sourceUpper)
            WideArithmetic.AddMagnitudeInto(radiusSquared, radicalSquared);
        ShiftFiniteRootLeft(radicalSquared, 2 * shift - sourceShift - valueShift + 2);
        return rationalSign * WideArithmetic.CompareMagnitudeEqualLength(rationalSquared, radicalSquared);
    }
}
