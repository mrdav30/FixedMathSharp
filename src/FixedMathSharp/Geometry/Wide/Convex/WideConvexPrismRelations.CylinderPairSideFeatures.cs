//=======================================================================
// MIT License, Copyright (c) 2024-present David Oravsky (mrdav30)
// See LICENSE file in the project root for full license information.
//=======================================================================
using System;
using System.Runtime.CompilerServices;

namespace FixedMathSharp.Geometry;

/// <content>
/// Exact projected side-feature selection for cylinder-pair contacts.
/// </content>
internal static partial class WideConvexPrismRelations
{
    /// <summary>
    /// Selects the existing projected-cylinder minimum on one side-normal
    /// plane. A common-perpendicular winner remains with the earlier global
    /// analytic family. Positive full gaps return their exact value root;
    /// negative gaps and exact touches return their sign and normal directly.
    /// </summary>
    [MethodImpl(MethodImplOptions.NoInlining)]
    internal static bool TryGetCylinderPairSideFeature(
        Vector3d firstCenter, FixedQuaternion firstRotation, Vector3d firstLocalAxis,
        Signed192 firstLength, Fixed64 firstRadius,
        Vector3d secondCenter, FixedQuaternion secondRotation, Vector3d secondLocalAxis,
        Signed192 secondLength, Fixed64 secondRadius,
        in CylinderPairGeometry pairGeometry, bool firstSide,
        Span<ulong> valueCoefficients, Span<sbyte> valueSigns, Span<ulong> valueCell,
        out FiniteAxisValueRoot valueRoot, out Vector3d normal, out int gapSign)
    {
        valueRoot = default;
        normal = default;
        gapSign = 0;
        Vector3d cylinderCenter = firstSide ? secondCenter : firstCenter;
        FixedQuaternion cylinderRotation = firstSide ? secondRotation : firstRotation;
        Vector3d cylinderLocalAxis = firstSide ? secondLocalAxis : firstLocalAxis;
        Signed192 cylinderLength = firstSide ? secondLength : firstLength;
        Fixed64 cylinderRadius = firstSide ? secondRadius : firstRadius;
        Vector3d capsuleCenter = firstSide ? firstCenter : secondCenter;
        FixedQuaternion capsuleRotation = firstSide ? firstRotation : secondRotation;
        Vector3d capsuleLocalAxis = firstSide ? firstLocalAxis : secondLocalAxis;
        Signed192 capsuleLength = firstSide ? firstLength : secondLength;
        Fixed64 radiusOffset = firstSide ? firstRadius : secondRadius;
        WideOrientedBox.GetRotatedLocalAxisNumerators(cylinderRotation, cylinderLocalAxis,
            out Signed192 ax, out Signed192 ay, out Signed192 az, out Signed192 ad);
        WideOrientedBox.GetRotatedLocalAxisNumerators(capsuleRotation, capsuleLocalAxis,
            out Signed192 bx, out Signed192 by, out Signed192 bz, out Signed192 bd);
        var cylinder = new RigidAxis3(ax, ay, az, ad);
        var capsule = new RigidAxis3(bx, by, bz, bd);
        Axis3 major = Cross(cylinder.ToWide(), capsule.ToWide());
        // The pair owner already proved these exact axes are nonparallel.
        var geometry = new CylinderCapsuleFeatureGeometry(cylinderCenter, cylinder, cylinderLength,
            cylinderRadius, capsuleCenter, capsule, capsuleLength);
        const int candidateWords = ConvexContactCandidate.Words;
        Span<ulong> values = stackalloc ulong[ConvexContactCandidate.Slots * candidateWords];
        Span<int> candidateSigns = stackalloc int[ConvexContactCandidate.Slots];
        Span<ulong> best = stackalloc ulong[ConvexContactCandidate.Slots * candidateWords];
        Span<int> bestSigns = stackalloc int[ConvexContactCandidate.Slots];
        BuildCylinderCapsuleAxisCandidate(geometry, new CylinderCapsuleDirection(major),
            best, bestSigns, out int bestGapSign);
        bool majorWins = true;
        int capSign = 1;
        Axis3 minor = Cross(capsule.ToWide(), major);
        BuildCylinderCapsuleAxisCandidate(geometry, new CylinderCapsuleDirection(minor),
            values, candidateSigns, out int candidateGapSign);
        if (CompareConvexContactCandidates(new ConvexContactCandidate(values, candidateSigns, candidateGapSign),
                new ConvexContactCandidate(best, bestSigns, bestGapSign)) < 0)
        {
            values.CopyTo(best); candidateSigns.CopyTo(bestSigns);
            bestGapSign = candidateGapSign;
            majorWins = false;
            int orientation = DotCylinderCapsuleDirection(new CylinderCapsuleDirection(minor),
                geometry.DifferenceX, geometry.DifferenceY, geometry.DifferenceZ).Sign < 0 ? -1 : 1;
            capSign = GetAxisProjection(minor, cylinder).Sign * orientation;
        }

        bool perpendicular = GetAxisProjection(cylinder.ToWide(), capsule).IsZero;
        if (perpendicular)
        {
            for (int sign = -1; sign <= 1; sign += 2)
            {
                if (!BuildCylinderCapsuleProjectedCornerCandidate(geometry, major, sign,
                        values, candidateSigns, out candidateGapSign))
                    continue;
                // Admission proves both rectangle coordinate excesses >0.
                // The unique outward corner gap -sqrt(x²+y²) strictly beats
                // both principal gaps -x and -y; the other cap cannot admit.
                values.CopyTo(best); candidateSigns.CopyTo(bestSigns);
                bestGapSign = candidateGapSign;
                majorWins = false;
                capSign = sign;
            }
        }

        const int ellipseWords = CylinderCapsuleEllipseWords;
        Span<ulong> stationary = stackalloc ulong[5 * ellipseWords];
        Span<sbyte> stationarySigns = stackalloc sbyte[5];
        Span<ulong> signedGap = stackalloc ulong[3 * ellipseWords];
        Span<sbyte> signedGapSigns = stackalloc sbyte[3];
        Span<ulong> squaredGap = stackalloc ulong[10 * ellipseWords];
        Span<sbyte> squaredGapSigns = stackalloc sbyte[10];
        Span<ulong> numerator = squaredGap[..(5 * ellipseWords)];
        Span<ulong> denominator = squaredGap[(5 * ellipseWords)..];
        Span<sbyte> numeratorSigns = squaredGapSigns[..5], denominatorSigns = squaredGapSigns[5..];
        // The old quartic's proven maximum coefficient height is 1818 bits.
        Span<ulong> stationaryCell = stackalloc ulong[(9 * 1818 + 255) / 64];
        scoped FiniteAxisPolynomialRoot stationaryRoot = default;
        CylinderCapsuleEllipse ellipse = default;
        bool ellipseWins = false;
        if (!perpendicular && TryPrepareCylinderCapsuleEllipse(cylinderCenter, cylinder, cylinderLength,
            cylinderRadius, capsuleCenter, capsule, out ellipse))
        {
            BuildCylinderCapsuleEllipsePolynomials(ellipse, stationary, stationarySigns,
                signedGap, signedGapSigns, numerator, numeratorSigns, denominator, denominatorSigns);
            WideFiniteAxisIntersection.NormalizeFiniteAxisPolynomialPowerOfTwo(stationary, stationarySigns);
            WideFiniteAxisIntersection.NormalizeFiniteAxisPolynomialPowerOfTwo(signedGap, signedGapSigns);
            WideFiniteAxisIntersection.NormalizeFiniteAxisPolynomialPowerOfTwo(squaredGap, squaredGapSigns);
            if (WideFiniteAxisIntersection.TryGetLargestPositiveFiniteAxisRoot(stationary, stationarySigns,
                stationaryCell, out stationaryRoot))
            {
                int ellipseGapSign = WideFiniteAxisIntersection.GetSignAtFiniteAxisRoot(
                    ref stationaryRoot, signedGap, signedGapSigns);
                int comparison = ellipseGapSign.CompareTo(bestGapSign);
                // A zero smooth gap has one non-principal supporting normal;
                // neither principal-axis competitor can tie it at zero.
                if (comparison == 0)
                    comparison = ellipseGapSign * CompareCylinderCapsuleEllipseSquaredGap(ref stationaryRoot,
                        numerator, numeratorSigns, denominator, denominatorSigns,
                        new ConvexContactCandidate(best, bestSigns, bestGapSign));
                if (comparison < 0)
                {
                    ellipseWins = true;
                    majorWins = false;
                    bestGapSign = ellipseGapSign;
                    capSign = GetAxisProjection(ellipse.Minor, cylinder).Sign;
                }
            }
        }
        if (majorWins)
            return false;

        var analytic = new ConvexContactCandidate(best, bestSigns, bestGapSign);
        if (ellipseWins)
        {
            gapSign = CompareCylinderCapsuleEllipseDepthToTwiceRaw(ref stationaryRoot,
                numerator, numeratorSigns, denominator, denominatorSigns, bestGapSign, radiusOffset, 0UL);
            normal = new Vector3d(GetRoundedCylinderCapsuleEllipseNormalComponent(ref stationaryRoot, ellipse, 0),
                GetRoundedCylinderCapsuleEllipseNormalComponent(ref stationaryRoot, ellipse, 1),
                GetRoundedCylinderCapsuleEllipseNormalComponent(ref stationaryRoot, ellipse, 2));
        }
        else
        {
            gapSign = CompareConvexContactCandidateDepthToTwiceRaw(analytic, radiusOffset, default);
            normal = GetConvexContactCandidateNormal(analytic);
        }
        if (firstSide)
        {
            normal = -normal;
            capSign = -capSign;
        }
        if (gapSign <= 0)
            return true;

        int words = valueCoefficients.Length / 9;
        Signed320 sideRadius = firstSide ? pairGeometry.FirstRadius : pairGeometry.SecondRadius;
        Span<ulong> radiusSquared = stackalloc ulong[9];
        WideArithmetic.GetMagnitude(WideArithmetic.MultiplySigned320(sideRadius, sideRadius), radiusSquared);
        if (bestGapSign == 0)
        {
            valueCoefficients.Clear(); valueSigns.Clear();
            radiusSquared.CopyTo(valueCoefficients);
            valueSigns[0] = -1;
            valueCoefficients[words] = 1;
            valueSigns[1] = 1;
            WideFiniteAxisIntersection.ScaleFiniteAxisPolynomialVariable(valueCoefficients, 9, pairGeometry.ValueShift);
            // The positive radius square is the unique root of this linear
            // polynomial, and the geometry scale places it in (0,1].
            valueRoot = WideFiniteAxisIntersection.GetKnownFiniteValueRoot(
                valueCoefficients[..(2 * words)], valueSigns[..2], 0, 1, valueCell);
            return true;
        }

        Span<ulong> unshifted = stackalloc ulong[5 * words];
        Span<sbyte> unshiftedSigns = stackalloc sbyte[5];
        CylinderPairSideValuePolynomial.Build(pairGeometry, firstSide, capSign,
            valueCoefficients, valueSigns, unshifted, unshiftedSigns);
        WideFiniteAxisIntersection.ScaleFiniteAxisPolynomialVariable(unshifted, 5, pairGeometry.ValueShift);
        WideFiniteAxisIntersection.ScaleFiniteAxisPolynomialVariable(valueCoefficients, 9, pairGeometry.ValueShift);
        WideFiniteAxisIntersection.NormalizeFiniteAxisPolynomialPowerOfTwo(unshifted, unshiftedSigns);
        WideFiniteAxisIntersection.NormalizeFiniteAxisPolynomialPowerOfTwo(valueCoefficients, valueSigns);
        Span<ulong> unshiftedCell = stackalloc ulong[
            WideFiniteAxisIntersection.GetFiniteValueRootCellWords(unshifted, unshiftedSigns)];
        scoped FiniteAxisValueRoot unshiftedRoot = GetKnownCylinderPairSideUnshiftedRoot(
                pairGeometry, unshifted, unshiftedSigns, unshiftedCell,
                ellipseWins, ref stationaryRoot, numerator, numeratorSigns, denominator, denominatorSigns,
                analytic);
        // G(S)=F((sqrt(S)-r)^2)F((sqrt(S)+r)^2). The selected nonzero
        // h satisfies F(h²)=0, hence the admitted positive (r+h)² is a known
        // G-root. This is membership construction, not a candidate rejection.
        valueRoot = WideFiniteAxisIntersection.MapKnownFiniteValueRootRadicalOffset(unshiftedRoot, bestGapSign,
                radiusSquared, pairGeometry.ValueShift, valueCoefficients, valueSigns, valueCell);
        return true;
    }

    [MethodImpl(MethodImplOptions.NoInlining)]
    private static FiniteAxisValueRoot GetKnownCylinderPairSideUnshiftedRoot(in CylinderPairGeometry geometry,
        ReadOnlySpan<ulong> coefficients, ReadOnlySpan<sbyte> signs, Span<ulong> cell,
        bool ellipse, scoped ref FiniteAxisPolynomialRoot stationaryRoot,
        scoped ReadOnlySpan<ulong> numerator, scoped ReadOnlySpan<sbyte> numeratorSigns,
        scoped ReadOnlySpan<ulong> denominator, scoped ReadOnlySpan<sbyte> denominatorSigns,
        scoped ConvexContactCandidate analytic)
    {
        // Smooth stationary ellipse values satisfy F(h²)=0. A winning minor
        // axis is stationary (its major projection is zero); when the axes are
        // perpendicular, F=m²(l²+4H*rho*m) also includes both line endpoints
        // and the minor-axis value. The earlier major-axis family was excluded.
        Span<ulong> batchCells = stackalloc ulong[64];
        Span<int> batchShifts = stackalloc int[8];
        FiniteAxisValueRoots roots = WideFiniteAxisIntersection.GetFiniteValueRoots(
            coefficients, signs, batchCells, batchShifts);
        Span<ulong> upper = stackalloc ulong[cell.Length];
        // A winning smooth support minimum also locally minimizes ellipse
        // distance: D'=2*rho*h', with positive curvature radius rho. The
        // noncircular ellipse has a strictly farther stationary point. A
        // winning principal minor or finite-segment endpoint likewise has a
        // farther endpoint. That distance is another physical F-root, so the
        // greatest root cannot be the selected h². It must not be confused
        // with the smallest root: an inadmissible line root may precede it.
        System.Diagnostics.Debug.Assert(roots.Count >= 2);
        int lastEligibleOrdinal = roots.Count - 2;
        for (int ordinal = 0; ordinal < lastEligibleOrdinal; ordinal++)
        {
            FiniteAxisValueRoot candidate = FiniteAxisValueRoots.GetRoot(roots, ordinal, coefficients, signs, cell);
            if (candidate.IsRational)
            {
                if (CompareCylinderPairSideUnshiftedEndpoint(geometry, ellipse, ref stationaryRoot,
                    numerator, numeratorSigns, denominator, denominatorSigns, analytic,
                    cell, candidate.DenominatorShift) != 0) continue;
            }
            else
            {
                // Earlier ordinals have already been rejected, so this root
                // is at most the selected h² and its open lower endpoint is
                // strictly smaller. Only the upper endpoint can reject it.
#if DEBUG
                System.Diagnostics.Debug.Assert(CompareCylinderPairSideUnshiftedEndpoint(geometry, ellipse,
                    ref stationaryRoot, numerator, numeratorSigns, denominator, denominatorSigns, analytic,
                    cell, candidate.DenominatorShift) > 0);
#endif
                cell.CopyTo(upper);
                WideArithmetic.AddWord(upper, 0, 1);
                if (CompareCylinderPairSideUnshiftedEndpoint(geometry, ellipse, ref stationaryRoot,
                    numerator, numeratorSigns, denominator, denominatorSigns, analytic,
                    upper, candidate.DenominatorShift) >= 0) continue;
            }
            return candidate;
        }
        // Rejecting the earlier eligible roots identifies the last one by
        // exclusion; no full quartic comparison is needed again.
        return FiniteAxisValueRoots.GetRoot(roots, lastEligibleOrdinal, coefficients, signs, cell);
    }

    /// <summary>
    /// Compares the retained raw h² with endpoint*2^ValueShift/RawScale².
    /// The F-cell has at most 55065 denominator bits. Thus ellipse queries
    /// have fewer than 58500 bits; the existing quartic Hermite fallback,
    /// including its caller query/remainder and determinant scratch, stays
    /// below 700 KiB. It runs after the separate F-isolation arena has returned.
    /// </summary>
    [MethodImpl(MethodImplOptions.NoInlining)]
    private static int CompareCylinderPairSideUnshiftedEndpoint(in CylinderPairGeometry geometry,
        bool ellipse, scoped ref FiniteAxisPolynomialRoot root,
        scoped ReadOnlySpan<ulong> numerator, scoped ReadOnlySpan<sbyte> numeratorSigns,
        scoped ReadOnlySpan<ulong> denominator, scoped ReadOnlySpan<sbyte> denominatorSigns,
        scoped ConvexContactCandidate analytic, scoped ReadOnlySpan<ulong> endpoint, int shift)
    {
        int words = (shift + geometry.ValueShift + 2560 + 384 + 127) / 64;
        Span<ulong> rawScale = stackalloc ulong[3];
        Span<ulong> scaleSquared = stackalloc ulong[6];
        WideArithmetic.GetMagnitude(geometry.RawScale, out rawScale[2], out rawScale[1], out rawScale[0]);
        WideArithmetic.MultiplyMagnitudes(rawScale, rawScale, scaleSquared);
        Span<ulong> term = stackalloc ulong[words];
        if (ellipse)
        {
            Span<ulong> query = stackalloc ulong[5 * words];
            Span<sbyte> querySigns = stackalloc sbyte[5];
            query.Clear(); querySigns.Clear();
            for (int index = 0; index < 5; index++)
            {
                Span<ulong> coefficient = query.Slice(index * words, words);
                int sign = 0;
                WideArithmetic.MultiplyMagnitudes(numerator.Slice(index * CylinderCapsuleEllipseWords,
                    CylinderCapsuleEllipseWords), scaleSquared, term);
                WideArithmetic.AddShiftedSignedMagnitude(term, numeratorSigns[index], shift, coefficient, ref sign);
                WideArithmetic.MultiplyMagnitudes(denominator.Slice(index * CylinderCapsuleEllipseWords,
                    CylinderCapsuleEllipseWords), endpoint, term);
                WideArithmetic.AddShiftedSignedMagnitude(term, -denominatorSigns[index], geometry.ValueShift, coefficient, ref sign);
                querySigns[index] = (sbyte)sign;
            }
            return WideFiniteAxisIntersection.GetSignAtFiniteAxisRoot(ref root, query, querySigns);
        }
        Span<ulong> rational = stackalloc ulong[words];
        Span<ulong> radical = stackalloc ulong[words];
        rational.Clear(); radical.Clear();
        int rationalSign = 0, radicalSign = 0;
        WideArithmetic.MultiplyMagnitudes(analytic.GapRational, scaleSquared, term);
        WideArithmetic.AddShiftedSignedMagnitude(term, analytic.GapRationalSign, shift, rational, ref rationalSign);
        WideArithmetic.MultiplyMagnitudes(analytic.GapDenominator, endpoint, term);
        WideArithmetic.AddShiftedSignedMagnitude(term, -1, geometry.ValueShift, rational, ref rationalSign);
        WideArithmetic.MultiplyMagnitudes(analytic.GapRadical, scaleSquared, term);
        WideArithmetic.AddShiftedSignedMagnitude(term, analytic.GapRadicalSign, shift, radical, ref radicalSign);
        return GetConvexContactCandidateQuadraticSign(rational, rationalSign, radical, radicalSign, analytic.GapRadicand);
    }
}
