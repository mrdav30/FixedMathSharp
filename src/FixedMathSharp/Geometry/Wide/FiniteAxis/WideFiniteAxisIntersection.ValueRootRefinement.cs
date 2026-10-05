//=======================================================================
// MIT License, Copyright (c) 2024-present David Oravsky (mrdav30)
// See LICENSE file in the project root for full license information.
//=======================================================================
using System;

namespace FixedMathSharp.Geometry;

/// <content>Certified byte-sized refinement of an isolated value-root crossing.</content>
internal static partial class WideFiniteAxisIntersection
{
    // Extra relative-precision bits reduce exact fallback near ill-conditioned
    // crossings. This is a measured work budget; it never decides acceptance.
    private const int FiniteValuePointGuardBits = 128;

    private static bool TryRefineFiniteValueCrossingRoot(ref FiniteAxisValueRoot root,
        int targetShift, int lowerSign, int coefficientBits)
    {
        // Secant values only predict one of 256 child cells. The retained
        // parent contains exactly one distinct root; matching opposite signs
        // at a child's endpoints therefore certify the very same root.
        // No derivative estimate, tolerance, or predictor can admit a cell.
        bool useByteSteps = targetShift - root.DenominatorShift >= 8;
        uint lowerHint = 0;
        uint upperHint = 0;
        int lowerBits = 0;
        int upperBits = 0;
        if (lowerSign == 0 || useByteSteps)
        {
            int shift = root.DenominatorShift;
            int precision = shift + FiniteValuePointGuardBits + (root.Signs.Length - 1)
                * (shift - GetFiniteRootBits(root.LowerNumerator) + 1);
            int lowerCertificate = GetFiniteValueApproximateSign(root.LowerNumerator, shift,
                root.Coefficients, root.Signs, precision, coefficientBits, out lowerHint, out lowerBits);
            AddRoundedCylinderWord(root.LowerNumerator, 0, 1);
            int upperCertificate = GetFiniteValueApproximateSign(root.LowerNumerator, shift,
                root.Coefficients, root.Signs, precision, coefficientBits, out upperHint, out upperBits);
            ulong borrow = 1;
            for (int word = 0; borrow != 0; word++)
                root.LowerNumerator[word] = WideArithmetic.SubtractWord(root.LowerNumerator[word], 0, ref borrow);
            if (lowerSign == 0)
            {
                // The same endpoint certificates both identify an odd crossing
                // and supply its prediction hints. An uncertain point is still
                // evaluated exactly. Equal signs or a zero parent endpoint
                // leave the original cell untouched for the Sturm fallback.
                lowerSign = lowerCertificate != 0 ? lowerCertificate
                    : EvaluateFiniteRootPolynomial(root.Coefficients, root.Signs,
                        root.LowerNumerator, shift, 0);
                int upperSign = upperCertificate != 0 ? upperCertificate : GetFiniteValueCellUpperSign(root);
                if (lowerSign == 0 || lowerSign != -upperSign)
                    return false;
            }
            useByteSteps = useByteSteps && lowerCertificate == lowerSign && upperCertificate == -lowerSign;
        }

        while (!root.IsRational && root.DenominatorShift < targetShift)
        {
            if (useByteSteps && targetShift - root.DenominatorShift >= 8)
            {
                int commonBits = Math.Max(lowerBits, upperBits);
                ulong lowerWeight = commonBits - lowerBits >= 32 ? 0UL : lowerHint >> (commonBits - lowerBits);
                ulong upperWeight = commonBits - upperBits >= 32 ? 0UL : upperHint >> (commonBits - upperBits);
                // At least one aligned hint retains its top bit. The quotient's
                // numerator is <2^40 and denominator <2^33, within ulong.
                ulong child = Math.Min(255UL, (lowerWeight << 8) / (lowerWeight + upperWeight));
                ShiftFiniteRootLeft(root.LowerNumerator, 8);
                AddRoundedCylinderWord(root.LowerNumerator, 0, child);
                int nextShift = root.DenominatorShift + 8;
                // This is only a throughput budget, never an acceptance
                // epsilon. Both endpoints use exactly this same precision.
                // Every candidate is inside the positive parent cell, so
                // k<=min(q,B+3), and the existing point scratch bound holds.
                // Hints are four scalars; no additional cell/arena is retained.
                int precision = nextShift + FiniteValuePointGuardBits + (root.Signs.Length - 1)
                    * (nextShift - GetFiniteRootBits(root.LowerNumerator) + 1);
                int lowerCertificate = GetFiniteValueApproximateSign(root.LowerNumerator, nextShift,
                    root.Coefficients, root.Signs, precision, coefficientBits, out lowerHint, out lowerBits);
                int lowerPointSign = lowerCertificate != 0 ? lowerCertificate
                    : EvaluateFiniteRootPolynomial(root.Coefficients, root.Signs, root.LowerNumerator, nextShift, 0);
                if (lowerPointSign == 0)
                {
                    KeepFiniteValueRefinementSingleton(ref root, nextShift);
                    return true;
                }
                AddRoundedCylinderWord(root.LowerNumerator, 0, 1);
                int upperCertificate = GetFiniteValueApproximateSign(root.LowerNumerator, nextShift,
                    root.Coefficients, root.Signs, precision, coefficientBits, out upperHint, out upperBits);
                int upperPointSign = upperCertificate != 0 ? upperCertificate
                    : EvaluateFiniteRootPolynomial(root.Coefficients, root.Signs, root.LowerNumerator, nextShift, 0);
                if (upperPointSign == 0)
                {
                    KeepFiniteValueRefinementSingleton(ref root, nextShift);
                    return true;
                }
                ulong borrow = 1;
                for (int word = 0; borrow != 0; word++)
                    root.LowerNumerator[word] = WideArithmetic.SubtractWord(root.LowerNumerator[word], 0, ref borrow);
                if (lowerPointSign == lowerSign && upperPointSign == -lowerSign)
                {
                    root.DenominatorShift = nextShift;
                    // Exact fallback may validate this cell without providing
                    // useful approximate values for another prediction.
                    useByteSteps = lowerCertificate != 0 && upperCertificate != 0;
                    continue;
                }
                // The proposed numerator is 256*oldLower+child, child<256.
                // Restore every old word, not merely the low word. Disable
                // prediction for the rest of this call after one failed try.
                ShiftRoundedCylinderWideRight(root.LowerNumerator, 8);
                useByteSteps = false;
            }
            RefineFiniteCrossingRoot(root.Coefficients, root.Signs, root.LowerNumerator,
                ref root.DenominatorShift, ref root.IsRational, lowerSign, coefficientBits);
        }
        return true;
    }

    private static void KeepFiniteValueRefinementSingleton(ref FiniteAxisValueRoot root, int shift)
    {
        // Bisection discovers the minimal dyadic denominator. A byte step may
        // find the same endpoint at a finer grid, so preserve that convention.
        int removed = Math.Min(shift, CountRoundedCylinderTrailingZeroes(root.LowerNumerator));
        ShiftRoundedCylinderWideRight(root.LowerNumerator, removed);
        root.DenominatorShift = shift - removed;
        root.IsRational = true;
    }
}
