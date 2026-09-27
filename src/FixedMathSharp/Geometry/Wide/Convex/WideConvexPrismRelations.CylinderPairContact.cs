//=======================================================================
// MIT License, Copyright (c) 2024-present David Oravsky (mrdav30)
// See LICENSE file in the project root for full license information.
//=======================================================================
using System;
using System.Runtime.CompilerServices;

namespace FixedMathSharp.Geometry;

/// <content>Complete nonparallel cylinder support-feature selection.</content>
internal static partial class WideConvexPrismRelations
{
    private const int CylinderPairValueWords = CylinderPairRimFeatures.CoefficientWords;
    private const int CylinderPairCellWords = (16 * (CylinderPairValueWords * 64 + 11) + 255) / 64;

    private static bool TryGetNonparallelCylinderPairPenetration(
        Vector3d firstCenter, FixedQuaternion firstRotation, Vector3d firstLocalAxis,
        Signed192 firstLength, Fixed64 firstRadius,
        Vector3d secondCenter, FixedQuaternion secondRotation, Vector3d secondLocalAxis,
        Signed192 secondLength, Fixed64 secondRadius,
        out Vector3d normal, out Fixed64 depth, out bool depthIsClamped)
    {
        normal = default;
        depth = default;
        depthIsClamped = false;
        var geometry = new CylinderPairGeometry(firstCenter, firstRotation, firstLocalAxis,
            firstLength, firstRadius, secondCenter, secondRotation, secondLocalAxis, secondLength, secondRadius);
        Span<ulong> bestAnalyticValues = stackalloc ulong[ConvexContactCandidate.Words * ConvexContactCandidate.Slots];
        Span<int> bestAnalyticSigns = stackalloc int[ConvexContactCandidate.Slots];
        Span<ulong> bestValues = stackalloc ulong[9 * CylinderPairValueWords];
        Span<sbyte> bestSigns = stackalloc sbyte[9];
        Span<ulong> bestCell = stackalloc ulong[CylinderPairCellWords];
        var best = new CylinderPairSelection(bestValues, bestSigns, bestCell, false);
        if (!TrySelectCylinderPairFeature(firstCenter, firstRotation, firstLocalAxis, firstLength, firstRadius,
                secondCenter, secondRotation, secondLocalAxis, secondLength, secondRadius, geometry,
                bestAnalyticValues, bestAnalyticSigns, out int analyticGapSign, ref best))
            return false;
        var analytic = new ConvexContactCandidate(bestAnalyticValues, bestAnalyticSigns, analyticGapSign);
        if (!best.HasRoot && !best.NormalReady)
        {
            normal = GetConvexContactCandidateNormal(analytic);
            GetRoundedConvexContactCandidateDepth(analytic, Fixed64.Zero, out depth, out depthIsClamped);
        }
        else
        {
            normal = best.NormalReady ? best.Normal : CylinderPairRimFeatures.GetSimpleNormal(
                geometry, best.FirstSign, best.SecondSign, ref best.Root, best.SlopeSign, 1);
            depth = Fixed64.Zero;
            if (!best.IsZero)
                CylinderPairRimFeatures.GetRoundedDepth(geometry, ref best.Root, out depth, out depthIsClamped);
        }
        return true;
    }

    // Release all transient candidate buffers before the selected algebraic
    // normal is rounded. Its certified sign queries own a separate large arena.
    [MethodImpl(MethodImplOptions.NoInlining)]
    private static bool TrySelectCylinderPairFeature(
        Vector3d firstCenter, FixedQuaternion firstRotation, Vector3d firstLocalAxis,
        Signed192 firstLength, Fixed64 firstRadius,
        Vector3d secondCenter, FixedQuaternion secondRotation, Vector3d secondLocalAxis,
        Signed192 secondLength, Fixed64 secondRadius, in CylinderPairGeometry geometry,
        Span<ulong> bestAnalyticValues, Span<int> bestAnalyticSigns, out int analyticGapSign,
        ref CylinderPairSelection best)
    {
        Span<ulong> analyticValues = stackalloc ulong[ConvexContactCandidate.Words * ConvexContactCandidate.Slots];
        Span<int> analyticSigns = stackalloc int[ConvexContactCandidate.Slots];
        analyticGapSign = 1;
        for (int axis = 0; axis < 3; axis++)
        {
            CylinderPairAnalyticFeatures.Build(geometry, axis, analyticValues, analyticSigns, out int gapSign);
            if (gapSign < 0)
                return false;
            var candidate = new ConvexContactCandidate(analyticValues, analyticSigns, gapSign);
            if (axis != 0 && CompareConvexContactCandidates(candidate,
                    new ConvexContactCandidate(bestAnalyticValues, bestAnalyticSigns, analyticGapSign)) >= 0)
                continue;
            analyticValues.CopyTo(bestAnalyticValues);
            analyticSigns.CopyTo(bestAnalyticSigns);
            analyticGapSign = gapSign;
        }
        var analytic = new ConvexContactCandidate(bestAnalyticValues, bestAnalyticSigns, analyticGapSign);
        best.IsZero = analyticGapSign == 0;
        Span<ulong> values = stackalloc ulong[9 * CylinderPairValueWords];
        Span<sbyte> signs = stackalloc sbyte[9];
        Span<ulong> cell = stackalloc ulong[CylinderPairCellWords];

        // Stable exact ties: cap axes, their cross, first/second side planes,
        // then cap pairs in ascending sign order and their root/branch order.
        for (int side = 0; side < 2; side++)
        {
            if (!TryGetCylinderPairSideFeature(firstCenter, firstRotation, firstLocalAxis, firstLength, firstRadius,
                    secondCenter, secondRotation, secondLocalAxis, secondLength, secondRadius,
                    geometry, side == 0, values, signs, cell, out FiniteAxisValueRoot sideRoot,
                    out Vector3d sideNormal, out int sideGapSign))
                continue;
            if (sideGapSign < 0)
                return false;
            if (sideGapSign == 0)
                best.KeepTouch(sideNormal);
            else if (ShouldKeepCylinderPairValue(geometry, sideRoot, analytic, best))
                best.KeepRoot(sideRoot, sideNormal);
        }
        for (int firstSign = -1; firstSign <= 1; firstSign += 2)
        {
            for (int secondSign = -1; secondSign <= 1; secondSign += 2)
            {
                CylinderPairRimFeatures.BuildValues(geometry, firstSign, secondSign, values, signs);
                if (signs[0] == 0)
                    for (int branch = 0; branch < 2; branch++)
                        if (CylinderPairRepeatedRim.TryGetZeroNormal(geometry, firstSign, secondSign, branch, out Vector3d zeroNormal))
                        {
                            best.KeepTouch(zeroNormal);
                            // The admitted common rim point proves intersection;
                            // its supporting normal proves the global gap is zero.
                            return true;
                        }
                if (!TryKeepCylinderPairCapValues(geometry, firstSign, secondSign, values, signs, cell, analytic, ref best))
                    return false;
            }
        }
        return true;
    }

    // Four admission polynomials (33,408 bytes plus signs) survive only one
    // cap pair. Their construction frames return before sign queries, and this
    // whole frame returns before final normal materialization. A ready bit
    // avoids constructing cap derivatives when earlier predicates reject.
    [MethodImpl(MethodImplOptions.NoInlining)]
    private static bool TryKeepCylinderPairCapValues(in CylinderPairGeometry geometry,
        int firstSign, int secondSign, scoped Span<ulong> values, scoped Span<sbyte> signs,
        scoped Span<ulong> cell, scoped ConvexContactCandidate analytic, ref CylinderPairSelection best)
    {
        Span<ulong> derivatives = stackalloc ulong[CylinderPairRimFeatures.AdmissionCoefficientCount * CylinderPairValueWords];
        Span<sbyte> derivativeSigns = stackalloc sbyte[CylinderPairRimFeatures.AdmissionCoefficientCount];
        int readyMask = 0;
        Span<ulong> factor = stackalloc ulong[5 * CylinderPairValueWords];
        Span<sbyte> factorSigns = stackalloc sbyte[5];
        // Deeply clustered roots retain the original exact fallback instead
        // of carrying eight full-size cells into admission and sign queries.
        Span<ulong> batchCells = stackalloc ulong[8 * 8];
        Span<int> batchShifts = stackalloc int[8];
        FiniteAxisValueRoots roots = WideFiniteAxisIntersection.GetFiniteValueRoots(
            values, signs, batchCells, batchShifts);
        if (roots.HasRepeatedRoots)
            WideFiniteAxisIntersection.GetFiniteValueRepeatedRootFactor(values, signs, factor, factorSigns);
        for (int ordinal = 0; ordinal < roots.Count; ordinal++)
        {
            scoped FiniteAxisValueRoot root = FiniteAxisValueRoots.GetRoot(roots, ordinal, values, signs, cell);
            if (roots.HasRepeatedRoots && TryHandleRepeatedCylinderPairValue(
                    geometry, firstSign, secondSign, root, factor, factorSigns, analytic, ref best))
                continue;
            int slope = CylinderPairRimFeatures.GetSimpleValueSlopeSign(root);
            if (!CylinderPairRimFeatures.TryAdmitSimple(geometry, firstSign, secondSign, ref root, slope,
                    derivatives, derivativeSigns, ref readyMask, out int gapSign))
                continue;
            if (gapSign < 0)
                return false;
            if (ShouldKeepCylinderPairValue(geometry, root, analytic, best))
                best.KeepSimpleRoot(root, firstSign, secondSign, slope);
        }
        return true;
    }

    private static bool ShouldKeepCylinderPairValue(in CylinderPairGeometry geometry, FiniteAxisValueRoot root,
        ConvexContactCandidate analytic, CylinderPairSelection best) => !best.IsZero && (best.HasRoot
            ? WideFiniteAxisIntersection.CompareFiniteValueRoots(root, best.Root) < 0
            : CylinderPairAnalyticFeatures.CompareRootSquared(geometry, root, analytic) < 0);

    [MethodImpl(MethodImplOptions.NoInlining)]
    private static bool TryHandleRepeatedCylinderPairValue(in CylinderPairGeometry geometry,
        int firstSign, int secondSign, scoped FiniteAxisValueRoot source,
        scoped ReadOnlySpan<ulong> factor, scoped ReadOnlySpan<sbyte> factorSigns, scoped ConvexContactCandidate analytic,
        ref CylinderPairSelection best)
    {
        // Each recovery family proves that admitted nonzero radial signs are
        // positive: negative signs require a unique, nondegenerate closest
        // disk pair, excluded by a rank-one pencil.
        // True classifies source as repeated even if no branch is admitted.
        Span<ulong> cell = stackalloc ulong[WideFiniteAxisIntersection.GetFiniteValueRootCellWords(factor, factorSigns)];
        Span<ulong> upper = stackalloc ulong[source.LowerNumerator.Length];
        source.LowerNumerator.CopyTo(upper);
        WideArithmetic.AddWord(upper, 0, 1);
        for (int ordinal = 0; WideFiniteAxisIntersection.TryGetFiniteValueRoot(factor, factorSigns,
            ordinal, cell, out FiniteAxisValueRoot root); ordinal++)
        {
            int lowerComparison = WideFiniteAxisIntersection.CompareFiniteValueRootToDyadic(
                root, source.LowerNumerator, source.DenominatorShift);
            if (source.IsRational ? lowerComparison != 0 : lowerComparison <= 0
                || WideFiniteAxisIntersection.CompareFiniteValueRootToDyadic(root, upper, source.DenominatorShift) >= 0)
                continue;
            // Repeated families cannot separate. Once this value cannot
            // improve, neither admission nor another equal-depth branch matters.
            if (!ShouldKeepCylinderPairValue(geometry, root, analytic, best))
                return true;
            for (int branch = 0; branch < 8; branch++)
            {
                if (!CylinderPairRepeatedRim.TryGetRankOneNormal(geometry, firstSign, secondSign, root,
                        branch, out Vector3d normal))
                    continue;
                best.KeepRoot(root, normal);
                return true;
            }
            // H=0 stationary normals cannot improve: an interior minimum
            // requires orthogonal rim tangents, for which a cap-axis gap is
            // strictly smaller. Boundary normals belong to the side planes.
            // A rank-two triple pencil crosses its stationary distance with
            // odd (cubic) order along the first rim. Its admitted point cannot
            // be an interior support minimum; cap boundaries are side cases.
            return true; // The repeated factor has exactly one root in source's cell.
        }
        return false;
    }

    private ref struct CylinderPairSelection
    {
        private readonly Span<ulong> _values;
        private readonly Span<sbyte> _signs;
        private readonly Span<ulong> _cell;
        internal FiniteAxisValueRoot Root;
        internal Vector3d Normal;
        internal bool HasRoot;
        internal bool NormalReady;
        internal bool IsZero;
        internal int FirstSign;
        internal int SecondSign;
        internal int SlopeSign;

        internal CylinderPairSelection(Span<ulong> values, Span<sbyte> signs, Span<ulong> cell, bool zero)
        {
            this = default;
            _values = values; _signs = signs; _cell = cell; IsZero = zero;
        }

        internal void KeepTouch(Vector3d normal)
        {
            if (IsZero)
                return;
            IsZero = true;
            HasRoot = false;
            NormalReady = true;
            Normal = normal;
        }

        internal void KeepRoot(scoped FiniteAxisValueRoot root, Vector3d normal)
        {
            root.Coefficients.CopyTo(_values);
            root.Signs.CopyTo(_signs);
            _cell.Clear();
            root.LowerNumerator.CopyTo(_cell);
            Root = new FiniteAxisValueRoot
            {
                Coefficients = _values[..root.Coefficients.Length], Signs = _signs[..root.Signs.Length],
                LowerNumerator = _cell, DenominatorShift = root.DenominatorShift,
                IsRational = root.IsRational, Ordinal = root.Ordinal
            };
            HasRoot = NormalReady = true;
            Normal = normal;
        }

        internal void KeepSimpleRoot(scoped FiniteAxisValueRoot root, int firstSign, int secondSign, int slope)
        {
            KeepRoot(root, default);
            NormalReady = false;
            FirstSign = firstSign; SecondSign = secondSign; SlopeSign = slope;
        }
    }
}
