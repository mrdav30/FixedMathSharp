//=======================================================================
// WideConvexPrismRelations.CylinderCapsuleEllipse.cs
//=======================================================================
// MIT License, Copyright (c) 2024-present David Oravsky (mrdav30)
// See LICENSE file in the project root for full license information.
//=======================================================================

using System;

namespace FixedMathSharp.Geometry;

/// <content>
/// Exact projected-cap ellipse candidates for finite cylinder/capsule contact.
/// The capsule-interior normal is retained as one real quartic root; neither
/// the authored axes nor the candidate depth are rounded during selection.
/// </content>
internal static partial class WideConvexPrismRelations
{
    // Admitted normalized quaternions have squared raw norm < 2^65, and
    // local axes have raw norm < 2^33. Exact rotation preserves the latter
    // norm, so rotated-axis numerators and F are < 2^100. With the
    // intentionally loose bounds used here, U,B,C < 2^202, E < 2^404,
    // F < 2^100, |P| < 2^367, |Q| < 2^470, and K < 2^164.
    // The cross axes are < 2^201 and < 2^302 per component. A center
    // difference is < 2^64, so their three-term projections are < 2^267
    // and < 2^368. These bounds include unrestricted opposite-limit origins;
    // the positive GCD reduction below can only reduce them.
    // J has coefficients < 2^570; W < 2^1818 and S < 2^1244. Squaring S
    // sums at most three products, giving N < 2^2490; D < 2^2354.
    // Forty words preserve every bit, including coefficient-sum carries.
    private const int CylinderCapsuleEllipseWords = 40;
    // The quadratic analytic candidate uses at most forty words per scalar.
    // Eliminating its one radical from N/D - (A+B sqrt(C))/D0 produces
    // (N D0-D A)^2-D^2 B^2 C: five scalar widths plus sum carry bits.
    private const int CylinderCapsuleEllipseComparisonWords =
        5 * CylinderCapsuleEllipseWords + 2;

    /// <summary>
    /// Rational geometry for n(t) = (Major + t Minor) / sqrt(E(1+B t^2)).
    /// Lengths, center differences, and the eventual gap are in raw units.
    /// </summary>
    private readonly struct CylinderCapsuleEllipse
    {
        internal readonly Axis3 Major;
        internal readonly Axis3 Minor;
        internal readonly Signed320 U;
        internal readonly Signed320 B;
        internal readonly Signed320 C;
        internal readonly Signed576 E;
        internal readonly Signed192 F;
        internal readonly Signed320 K;
        internal readonly Signed576 P;
        internal readonly Signed576 Q;

        internal CylinderCapsuleEllipse(
            Axis3 major, Axis3 minor,
            Signed320 u, Signed320 b, Signed320 c, Signed576 e,
            Signed192 f, Signed320 k, Signed576 p, Signed576 q)
        {
            Major = major;
            Minor = minor;
            U = u;
            B = b;
            C = c;
            E = e;
            F = f;
            K = k;
            P = p;
            Q = q;
        }
    }

    /// <summary>
    /// Prepares the genuinely oblique family. The caller proves positive
    /// radius/core length and nonparallel, nonperpendicular axes. False means
    /// the query lies on a principal axis, whose boundaries already suffice.
    /// </summary>
    private static bool TryPrepareCylinderCapsuleEllipse(
        Vector3d cylinderCenter,
        in RigidAxis3 cylinderAxis,
        Signed192 cylinderLength,
        Fixed64 cylinderRadius,
        Vector3d capsuleCenter,
        in RigidAxis3 capsuleAxis,
        out CylinderCapsuleEllipse ellipse)
    {
        ellipse = default;
        // Cylinder half-axis raw coordinates are a*length/F, so their full
        // positive integer content cancels only jointly with F. The capsule
        // axis is used solely to define its perpendicular plane here; its
        // direction may be made primitive without changing authored length.
        // This removes quaternion normalization factors before forming any
        // products, and avoids a tiny artificial t caused by an oversized b.
        Signed192 f = Signed192.NarrowProven(WideArithmetic.MultiplySigned192(
            cylinderAxis.RotationDenominator, Signed192.Raw(Fixed64.Two)));
        RigidAxis3 reducedCylinder = ReduceCylinderCapsuleEllipseAxis(cylinderAxis, ref f);
        Signed192 unusedDenominator = default;
        RigidAxis3 reducedCapsule = ReduceCylinderCapsuleEllipseAxis(capsuleAxis, ref unusedDenominator);
        Axis3 a = reducedCylinder.ToWide();
        Axis3 b = reducedCapsule.ToWide();
        Axis3 major = Cross(a, b);
        Signed320 c = Signed320.NarrowValue(GetAxisProjection(a, reducedCapsule));
        Signed576 majorProjection = GetCylinderCapsuleEllipseProjection(
            capsuleCenter, cylinderCenter, major);
        if (majorProjection.IsZero)
            return false;

        Axis3 minor = Cross(b, major);
        Signed576 minorProjection = GetCylinderCapsuleEllipseProjection(
            capsuleCenter, cylinderCenter, minor);
        Signed576 e = GetAxisSquared(major);
        Signed576 p = WideArithmetic.SubtractSigned576(
            default,
            WideArithmetic.MultiplySigned576(GetMagnitude576(majorProjection), f));
        Signed576 q = WideArithmetic.SubtractSigned576(
            WideArithmetic.MultiplySigned576(e, cylinderLength),
            WideArithmetic.MultiplySigned576(GetMagnitude576(minorProjection), f));

        // Both reflection choices can only decrease the support gap: the
        // projected cylinder is symmetric in these orthogonal principal axes.
        // A zero minor projection uses the original axis as the canonical tie.
        ellipse = new CylinderCapsuleEllipse(
            OrientCylinderCapsuleEllipseAxis(major, majorProjection.Sign),
            OrientCylinderCapsuleEllipseAxis(minor, minorProjection.Sign),
            GetAxisSquared(reducedCylinder), GetAxisSquared(reducedCapsule), c, e,
            f, WideArithmetic.MultiplySigned192(f, Signed192.Raw(cylinderRadius)),
            p, q);
        return true;
    }

    private static RigidAxis3 ReduceCylinderCapsuleEllipseAxis(
        in RigidAxis3 axis, ref Signed192 denominator)
    {
        Signed192 divisor = WideArithmetic.GetGreatestCommonDivisor(
            WideArithmetic.GetGreatestCommonDivisor(axis.X, axis.Y),
            WideArithmetic.GetGreatestCommonDivisor(axis.Z, denominator));
        denominator = WideArithmetic.DivideExactSigned192(denominator, divisor);
        return new RigidAxis3(
            WideArithmetic.DivideExactSigned192(axis.X, divisor),
            WideArithmetic.DivideExactSigned192(axis.Y, divisor),
            WideArithmetic.DivideExactSigned192(axis.Z, divisor), default);
    }

    private static Axis3 OrientCylinderCapsuleEllipseAxis(Axis3 axis, int sign) =>
        sign < 0
            ? new Axis3(
                WideArithmetic.Negate(axis.X),
                WideArithmetic.Negate(axis.Y),
                WideArithmetic.Negate(axis.Z))
            : axis;

    private static Signed576 GetCylinderCapsuleEllipseProjection(
        Vector3d end, Vector3d start, Axis3 axis) =>
        WideArithmetic.AddSigned576(
            WideArithmetic.AddSigned576(
                WideArithmetic.MultiplySigned320(axis.X,
                    WideArithmetic.SubtractSigned192(
                        Signed192.Raw(end.X), Signed192.Raw(start.X))),
                WideArithmetic.MultiplySigned320(axis.Y,
                    WideArithmetic.SubtractSigned192(
                        Signed192.Raw(end.Y), Signed192.Raw(start.Y)))),
            WideArithmetic.MultiplySigned320(axis.Z,
                WideArithmetic.SubtractSigned192(
                    Signed192.Raw(end.Z), Signed192.Raw(start.Z))));

    /// <summary>
    /// Competes the sole curved interior candidate against the already ranked
    /// analytic families. Exact ties retain that earlier canonical candidate.
    /// True means this family won; intersects is then the closed-contact result.
    /// </summary>
    private static bool TryImproveCylinderCapsuleEllipse(
        in CylinderCapsuleEllipse ellipse,
        in ConvexContactCandidate analytic,
        Fixed64 cylinderRadius, Fixed64 capsuleRadius,
        out bool intersects,
        out Vector3d normal,
        out Fixed64 depth,
        out bool depthIsClamped)
    {
        intersects = false;
        normal = default;
        depth = default;
        depthIsClamped = false;
        const int words = CylinderCapsuleEllipseWords;
        Span<ulong> stationary = stackalloc ulong[5 * words];
        Span<sbyte> stationarySigns = stackalloc sbyte[5];
        Span<ulong> signedGap = stackalloc ulong[3 * words];
        Span<sbyte> signedGapSigns = stackalloc sbyte[3];
        Span<ulong> squaredGap = stackalloc ulong[10 * words];
        Span<sbyte> squaredGapSigns = stackalloc sbyte[10];
        Span<ulong> numerator = squaredGap.Slice(0, 5 * words);
        Span<sbyte> numeratorSigns = squaredGapSigns.Slice(0, 5);
        Span<ulong> denominator = squaredGap.Slice(5 * words, 5 * words);
        Span<sbyte> denominatorSigns = squaredGapSigns.Slice(5, 5);
        BuildCylinderCapsuleEllipsePolynomials(
            ellipse, stationary, stationarySigns, signedGap, signedGapSigns,
            numerator, numeratorSigns, denominator, denominatorSigns);
        WideFiniteAxisIntersection.NormalizeFiniteAxisPolynomialPowerOfTwo(stationary, stationarySigns);
        WideFiniteAxisIntersection.NormalizeFiniteAxisPolynomialPowerOfTwo(signedGap, signedGapSigns);
        // N and D must share exactly the same positive scale. Normalizing
        // them as one ten-coefficient polynomial preserves their quotient.
        WideFiniteAxisIntersection.NormalizeFiniteAxisPolynomialPowerOfTwo(squaredGap, squaredGapSigns);
        Span<ulong> rootCell = stackalloc ulong[
            WideFiniteAxisIntersection.GetFiniteAxisRootCellWords(
                stationary, stationarySigns)];
        if (!WideFiniteAxisIntersection.TryGetLargestPositiveFiniteAxisRoot(
                stationary, stationarySigns, rootCell,
                out FiniteAxisPolynomialRoot root))
        {
            return false;
        }

        int gapSign = WideFiniteAxisIntersection.GetSignAtFiniteAxisRoot(
            ref root, signedGap, signedGapSigns);
        int comparison = gapSign.CompareTo(analytic.GapSign);
        if (comparison == 0 && gapSign != 0)
        {
            comparison = gapSign * CompareCylinderCapsuleEllipseSquaredGap(
                ref root, numerator, numeratorSigns,
                denominator, denominatorSigns, analytic);
        }
        if (comparison >= 0)
            return false;

        int depthSign = CompareCylinderCapsuleEllipseDepthToTwiceRaw(
            ref root, numerator, numeratorSigns, denominator, denominatorSigns,
            gapSign, capsuleRadius, 0UL);
        if (depthSign < 0)
        {
            return true;
        }
        intersects = true;
        // The admission query already proves an exact touch has zero depth.
        if (depthSign > 0)
            GetRoundedCylinderCapsuleEllipseDepth(
                ref root, numerator, numeratorSigns, denominator, denominatorSigns,
                gapSign, cylinderRadius, capsuleRadius, out depth, out depthIsClamped);
        normal = new Vector3d(
            GetRoundedCylinderCapsuleEllipseNormalComponent(ref root, ellipse, 0),
            GetRoundedCylinderCapsuleEllipseNormalComponent(ref root, ellipse, 1),
            GetRoundedCylinderCapsuleEllipseNormalComponent(ref root, ellipse, 2));
        return true;
    }

    private static int CompareCylinderCapsuleEllipseSquaredGap(
        ref FiniteAxisPolynomialRoot root,
        scoped ReadOnlySpan<ulong> numerator, scoped ReadOnlySpan<sbyte> numeratorSigns,
        scoped ReadOnlySpan<ulong> denominator, scoped ReadOnlySpan<sbyte> denominatorSigns,
        scoped in ConvexContactCandidate analytic)
    {
        const int sourceWords = CylinderCapsuleEllipseWords;
        const int words = CylinderCapsuleEllipseComparisonWords;
        Span<ulong> difference = stackalloc ulong[5 * words];
        Span<sbyte> differenceSigns = stackalloc sbyte[5];
        Span<ulong> term = stackalloc ulong[words];
        difference.Clear();
        differenceSigns.Clear();
        for (int index = 0; index < 5; index++)
        {
            WideArithmetic.MultiplyMagnitudes(
                numerator.Slice(index * sourceWords, sourceWords),
                analytic.GapDenominator, term);
            AddCylinderCapsuleEllipseCoefficient(
                difference, differenceSigns, index, term, numeratorSigns[index], words);
            WideArithmetic.MultiplyMagnitudes(
                denominator.Slice(index * sourceWords, sourceWords),
                analytic.GapRational, term);
            AddCylinderCapsuleEllipseCoefficient(
                difference, differenceSigns, index, term,
                -denominatorSigns[index] * analytic.GapRationalSign, words);
        }
        int rationalSign = WideFiniteAxisIntersection.GetSignAtFiniteAxisRoot(
            ref root, difference, differenceSigns);
        int radicalSign = IsZero(analytic.GapRadicand)
            ? 0 : analytic.GapRadicalSign;
        if (radicalSign == 0)
            return rationalSign;
        if (rationalSign != radicalSign)
            return -radicalSign;

        Span<ulong> query = stackalloc ulong[9 * words];
        Span<sbyte> querySigns = stackalloc sbyte[9];
        WideFiniteAxisIntersection.MultiplyFiniteAxisPolynomials(
            difference, differenceSigns, difference, differenceSigns,
            query, querySigns, term);
        difference.Clear();
        for (int index = 0; index < 5; index++)
        {
            denominator.Slice(index * sourceWords, sourceWords)
                .CopyTo(difference.Slice(index * words, sourceWords));
            differenceSigns[index] = denominatorSigns[index];
        }
        Span<ulong> squaredDenominator = stackalloc ulong[9 * words];
        Span<sbyte> squaredDenominatorSigns = stackalloc sbyte[9];
        WideFiniteAxisIntersection.MultiplyFiniteAxisPolynomials(
            difference, differenceSigns, difference, differenceSigns,
            squaredDenominator, squaredDenominatorSigns, term);
        WideArithmetic.MultiplyMagnitudes(
            analytic.GapRadical, analytic.GapRadical, term);
        MultiplyBy(term, analytic.GapRadicand);
        ScaleCylinderCapsuleEllipsePolynomial(squaredDenominator, term, words);
        for (int index = 0; index < 9; index++)
        {
            AddCylinderCapsuleEllipseCoefficient(
                query, querySigns, index,
                squaredDenominator.Slice(index * words, words),
                -squaredDenominatorSigns[index], words);
        }
        return rationalSign * WideFiniteAxisIntersection.GetSignAtFiniteAxisRoot(
            ref root, query, querySigns);
    }

    private static int CompareCylinderCapsuleEllipseDepthToTwiceRaw(
        ref FiniteAxisPolynomialRoot root,
        scoped ReadOnlySpan<ulong> numerator, scoped ReadOnlySpan<sbyte> numeratorSigns,
        scoped ReadOnlySpan<ulong> denominator, scoped ReadOnlySpan<sbyte> denominatorSigns,
        int gapSign, Fixed64 capsuleRadius, ulong twiceRaw)
    {
        ulong twiceRadius = (ulong)capsuleRadius.m_rawValue << 1;
        int thresholdSign = twiceRaw.CompareTo(twiceRadius);
        if (gapSign != thresholdSign)
            return gapSign.CompareTo(thresholdSign);
        if (gapSign == 0)
            return 0;

        ulong magnitude = thresholdSign < 0
            ? twiceRadius - twiceRaw : twiceRaw - twiceRadius;
        return gapSign * CompareCylinderCapsuleEllipseMagnitudeToTwiceRaw(
            ref root, numerator, numeratorSigns, denominator, denominatorSigns, magnitude);
    }

    private static int CompareCylinderCapsuleEllipseMagnitudeToTwiceRaw(
        ref FiniteAxisPolynomialRoot root,
        scoped ReadOnlySpan<ulong> numerator, scoped ReadOnlySpan<sbyte> numeratorSigns,
        scoped ReadOnlySpan<ulong> denominator, scoped ReadOnlySpan<sbyte> denominatorSigns,
        ulong twiceRaw)
    {
        const int sourceWords = CylinderCapsuleEllipseWords;
        const int words = sourceWords + 3;
        int count = numeratorSigns.Length;
        Span<ulong> query = stackalloc ulong[count * words];
        Span<sbyte> querySigns = stackalloc sbyte[count];
        Span<ulong> threshold = stackalloc ulong[words];
        Span<ulong> term = stackalloc ulong[words];
        query.Clear();
        querySigns.Clear();
        threshold.Clear();
        threshold[0] = twiceRaw;
        SquareCylinderCapsuleEllipseMagnitude(threshold);
        for (int index = 0; index < count; index++)
        {
            term.Clear();
            numerator.Slice(index * sourceWords, sourceWords).CopyTo(term);
            ShiftLeft(term, 2);
            AddCylinderCapsuleEllipseCoefficient(
                query, querySigns, index, term, numeratorSigns[index], words);
            WideArithmetic.MultiplyMagnitudes(
                denominator.Slice(index * sourceWords, sourceWords), threshold, term);
            AddCylinderCapsuleEllipseCoefficient(
                query, querySigns, index, term, -denominatorSigns[index], words);
        }
        return WideFiniteAxisIntersection.GetSignAtFiniteAxisRoot(
            ref root, query, querySigns);
    }

    private static void GetRoundedCylinderCapsuleEllipseDepth(
        ref FiniteAxisPolynomialRoot root,
        ReadOnlySpan<ulong> numerator, ReadOnlySpan<sbyte> numeratorSigns,
        ReadOnlySpan<ulong> denominator, ReadOnlySpan<sbyte> denominatorSigns,
        int gapSign, Fixed64 cylinderRadius, Fixed64 capsuleRadius,
        out Fixed64 depth, out bool depthIsClamped)
    {
        depthIsClamped = false;
        if (gapSign == 0)
        {
            depth = capsuleRadius;
            return;
        }
        // The admitted major-axis candidate has gap R-|d.major|/sqrt(E)<R.
        // This ellipse won against that candidate, so its positive gap is
        // also below the cylinder radius. The sum of two nonnegative raw
        // Fixed64 radii fits ulong; it never wraps even at both maxima.
        ulong radiusRaw = (ulong)capsuleRadius.m_rawValue;
        ulong lower = gapSign > 0 ? radiusRaw : 0UL;
        ulong upper = gapSign < 0 ? radiusRaw : radiusRaw + (ulong)cylinderRadius.m_rawValue;
        if (upper > (ulong)long.MaxValue)
        {
            depthIsClamped = CompareCylinderCapsuleEllipseDepthToTwiceRaw(
                ref root, numerator, numeratorSigns, denominator, denominatorSigns,
                gapSign, capsuleRadius, (ulong)long.MaxValue << 1) > 0;
            if (depthIsClamped)
            {
                depth = Fixed64.MaxValue;
                return;
            }
            upper = (ulong)long.MaxValue;
        }
        while (lower < upper)
        {
            ulong middle = lower + ((upper - lower + 1UL) >> 1);
            int comparison = CompareCylinderCapsuleEllipseDepthToTwiceRaw(
                ref root, numerator, numeratorSigns, denominator, denominatorSigns,
                gapSign, capsuleRadius, middle << 1);
            if (comparison >= 0)
                lower = middle;
            else
                upper = middle - 1UL;
        }
        int midpointComparison = CompareCylinderCapsuleEllipseDepthToTwiceRaw(
            ref root, numerator, numeratorSigns, denominator, denominatorSigns,
            gapSign, capsuleRadius, (lower << 1) + 1UL);
        lower += GetNearestEvenIncrement(midpointComparison, lower);
        depth = Fixed64.FromRaw((long)lower);
    }

    private static Fixed64 GetRoundedCylinderCapsuleEllipseNormalComponent(
        ref FiniteAxisPolynomialRoot root,
        in CylinderCapsuleEllipse ellipse,
        int component)
    {
        const int words = CylinderCapsuleEllipseWords;
        Span<ulong> direction = stackalloc ulong[2 * words];
        Span<sbyte> directionSigns = stackalloc sbyte[2];
        Span<ulong> numerator = stackalloc ulong[3 * words];
        Span<sbyte> numeratorSigns = stackalloc sbyte[3];
        Span<ulong> denominator = stackalloc ulong[3 * words];
        Span<sbyte> denominatorSigns = stackalloc sbyte[3];
        BuildCylinderCapsuleEllipseNormalPolynomials(
            ellipse, component, direction, directionSigns,
            numerator, numeratorSigns, denominator, denominatorSigns);
        int sign = WideFiniteAxisIntersection.GetSignAtFiniteAxisRoot(
            ref root, direction, directionSigns);
        if (sign == 0)
            return Fixed64.Zero;

        ulong lower = 0UL;
        ulong upper = 1UL << FixedMath.SHIFT_AMOUNT_I;
        while (lower < upper)
        {
            ulong middle = lower + ((upper - lower + 1UL) >> 1);
            if (CompareCylinderCapsuleEllipseMagnitudeToTwiceRaw(
                    ref root, numerator, numeratorSigns, denominator, denominatorSigns, middle << 1) >= 0)
            {
                lower = middle;
            }
            else
            {
                upper = middle - 1UL;
            }
        }
        int midpointComparison = CompareCylinderCapsuleEllipseMagnitudeToTwiceRaw(
            ref root, numerator, numeratorSigns, denominator, denominatorSigns, (lower << 1) + 1UL);
        lower += GetNearestEvenIncrement(midpointComparison, lower);
        return Fixed64.FromRaw(sign < 0 ? -(long)lower : (long)lower);
    }

    /// <summary>
    /// Builds W(t), S(t), and the exact squared-gap N(t)/D(t), in ascending
    /// coefficient order with a separate sign per magnitude. W,N,D have five
    /// coefficient slots, S three; every slot has CylinderCapsuleEllipseWords.
    /// </summary>
    private static void BuildCylinderCapsuleEllipsePolynomials(
        in CylinderCapsuleEllipse ellipse,
        Span<ulong> stationary, Span<sbyte> stationarySigns,
        Span<ulong> signedGap, Span<sbyte> signedGapSigns,
        Span<ulong> squaredGapNumerator, Span<sbyte> numeratorSigns,
        Span<ulong> squaredGapDenominator, Span<sbyte> denominatorSigns)
    {
        const int words = CylinderCapsuleEllipseWords;
        Span<ulong> scalars = stackalloc ulong[8 * words];
        scalars.Clear();
        Span<ulong> u = scalars.Slice(0 * words, words);
        Span<ulong> b = scalars.Slice(1 * words, words);
        Span<ulong> cSquared = scalars.Slice(2 * words, words);
        Span<ulong> e = scalars.Slice(3 * words, words);
        Span<ulong> f = scalars.Slice(4 * words, words);
        Span<ulong> kSquared = scalars.Slice(5 * words, words);
        Span<ulong> p = scalars.Slice(6 * words, words);
        Span<ulong> q = scalars.Slice(7 * words, words);
        CopyCylinderCapsuleEllipseMagnitude(ellipse.U, u);
        CopyCylinderCapsuleEllipseMagnitude(ellipse.B, b);
        CopyCylinderCapsuleEllipseMagnitude(ellipse.C, cSquared);
        SquareCylinderCapsuleEllipseMagnitude(cSquared);
        WideArithmetic.GetMagnitude(ellipse.E, e);
        CopyCylinderCapsuleEllipseMagnitude(Signed320.ExtendValue(ellipse.F), f);
        CopyCylinderCapsuleEllipseMagnitude(ellipse.K, kSquared);
        SquareCylinderCapsuleEllipseMagnitude(kSquared);
        WideArithmetic.GetMagnitude(ellipse.P, p);
        WideArithmetic.GetMagnitude(ellipse.Q, q);

        Span<ulong> j = stackalloc ulong[2 * words];
        Span<sbyte> jSigns = stackalloc sbyte[2];
        j.Clear();
        q.CopyTo(j.Slice(0, words));
        WideArithmetic.MultiplyMagnitudes(b, p, j.Slice(words, words));
        jSigns[0] = (sbyte)ellipse.Q.Sign;
        jSigns[1] = 1; // P < 0, B > 0: J = Q - B P t.
        Span<ulong> jSquared = stackalloc ulong[3 * words];
        Span<sbyte> jSquaredSigns = stackalloc sbyte[3];
        Span<ulong> term = stackalloc ulong[words];
        WideFiniteAxisIntersection.MultiplyFiniteAxisPolynomials(
            j, jSigns, j, jSigns, jSquared, jSquaredSigns, term);

        Span<ulong> radial = stackalloc ulong[3 * words];
        Span<sbyte> radialSigns = stackalloc sbyte[3];
        radial.Clear();
        radialSigns.Clear();
        WideArithmetic.MultiplyMagnitudes(u, u, radial.Slice(0, words));
        WideArithmetic.MultiplyMagnitudes(u, cSquared, radial.Slice(2 * words, words));
        radialSigns[0] = 1;
        radialSigns[2] = 1;
        WideFiniteAxisIntersection.MultiplyFiniteAxisPolynomials(
            radial, radialSigns, jSquared, jSquaredSigns,
            stationary, stationarySigns, term);
        kSquared.CopyTo(term);
        MultiplyBy(term, e);
        MultiplyBy(term, e);
        MultiplyBy(term, e);
        AddCylinderCapsuleEllipseCoefficient(
            stationary, stationarySigns, 2, term, -1, words);

        // At the largest positive W root, J > 0 and the unsquared
        // stationary equation is J sqrt(U(U+C^2 t^2)) = K E sqrt(E) t.
        // Therefore gap = S / (F U J sqrt(E(1+B t^2))), where
        // S = K^2 E^2 t + U J(P+Q t). Its denominator is strictly positive.
        radial.Clear();
        radialSigns.Clear();
        p.CopyTo(radial.Slice(0, words));
        q.CopyTo(radial.Slice(words, words));
        radialSigns[0] = -1;
        radialSigns[1] = (sbyte)ellipse.Q.Sign;
        WideFiniteAxisIntersection.MultiplyFiniteAxisPolynomials(
            j, jSigns, radial.Slice(0, 2 * words), radialSigns.Slice(0, 2),
            signedGap, signedGapSigns, term);
        ScaleCylinderCapsuleEllipsePolynomial(signedGap, u, words);
        kSquared.CopyTo(term);
        MultiplyBy(term, e);
        MultiplyBy(term, e);
        AddCylinderCapsuleEllipseCoefficient(
            signedGap, signedGapSigns, 1, term, 1, words);
        WideFiniteAxisIntersection.MultiplyFiniteAxisPolynomials(
            signedGap, signedGapSigns, signedGap, signedGapSigns,
            squaredGapNumerator, numeratorSigns, term);

        radial.Clear();
        radialSigns.Clear();
        radial[0] = 1UL;
        b.CopyTo(radial.Slice(2 * words, words));
        radialSigns[0] = 1;
        radialSigns[2] = 1;
        WideFiniteAxisIntersection.MultiplyFiniteAxisPolynomials(
            jSquared, jSquaredSigns, radial, radialSigns,
            squaredGapDenominator, denominatorSigns, term);
        WideArithmetic.MultiplyMagnitudes(f, f, term);
        MultiplyBy(term, u);
        MultiplyBy(term, u);
        MultiplyBy(term, e);
        ScaleCylinderCapsuleEllipsePolynomial(squaredGapDenominator, term, words);
    }

    // Completeness of the retained root (the t=0 and t=infinity boundaries
    // are ordinary rational-axis candidates in the pair owner):
    //
    // Before reflection, e=a cross b, f=b cross e, e^2=E, f^2=B E,
    // and E=U B-C^2. The projected cap disk and projected cylinder half-axis
    // are symmetric about e and f, so reflecting either normal coordinate
    // toward its matching d projection cannot increase the support gap.
    // For n=(e+t f)/sqrt(E(1+B t^2)) after those independent reflections,
    // F gap(t)=[K sqrt(E(U+C^2 t^2)/U)+P+Q t]/sqrt(E(1+B t^2)).
    // Here P<0 and Q is unrestricted. For t>0 its derivative has the sign of
    // h(t)=Q/t+B|P|-K E sqrt(E)/(sqrt(U)*sqrt(U+C^2 t^2)).
    // If Q<=0, h is strictly increasing. If Q>0, h' changes sign at most
    // once, negative to positive, because t^3/(U+C^2 t^2)^(3/2) increases.
    // Thus at most one interior local minimum exists. Since h(infinity)>0,
    // it is the largest positive root; an even last root is harmless to retain.
    // When Q<0, J is nonpositive only below -Q/(B|P|), where h is negative;
    // the genuine root above that threshold is larger than every squared-sign
    // artifact. When Q>=0 every positive root already has J>0.
    // When P=0, the derivative has at most a positive-to-negative crossing,
    // so only the two boundaries can minimize. C=0, E=0 and K=0 are the
    // rectangle/circle/segment reductions, handled before this construction.

    /// <summary>
    /// Builds the signed component direction and its squared raw magnitude
    /// numerator/denominator once. Only the threshold changes during rounding;
    /// every comparison uses the same exact polynomials at the retained root.
    /// </summary>
    private static void BuildCylinderCapsuleEllipseNormalPolynomials(
        in CylinderCapsuleEllipse ellipse,
        int component,
        Span<ulong> direction, Span<sbyte> directionSigns,
        Span<ulong> numerator, Span<sbyte> numeratorSigns,
        Span<ulong> denominator, Span<sbyte> denominatorSigns)
    {
        const int words = CylinderCapsuleEllipseWords;
        Signed320 first = component == 0 ? ellipse.Major.X
            : component == 1 ? ellipse.Major.Y : ellipse.Major.Z;
        Signed320 second = component == 0 ? ellipse.Minor.X
            : component == 1 ? ellipse.Minor.Y : ellipse.Minor.Z;
        direction.Clear();
        CopyCylinderCapsuleEllipseMagnitude(first, direction.Slice(0, words));
        CopyCylinderCapsuleEllipseMagnitude(second, direction.Slice(words, words));
        directionSigns[0] = (sbyte)first.Sign;
        directionSigns[1] = (sbyte)second.Sign;
        Span<ulong> factor = stackalloc ulong[words];
        WideFiniteAxisIntersection.MultiplyFiniteAxisPolynomials(
            direction, directionSigns, direction, directionSigns,
            numerator, numeratorSigns, factor);
        // Raw squared normal = 2^64 * component(t)^2 / (E(1+B t^2)).
        // The shared half-raw comparison supplies the remaining factor four.
        for (int index = 0; index < 3; index++)
            ShiftLeft(numerator.Slice(index * words, words), 64);
        denominator.Clear();
        denominatorSigns.Clear();
        WideArithmetic.GetMagnitude(ellipse.E, denominator.Slice(0, words));
        CopyCylinderCapsuleEllipseMagnitude(ellipse.B, factor);
        WideArithmetic.MultiplyMagnitudes(denominator.Slice(0, words), factor,
            denominator.Slice(2 * words, words));
        denominatorSigns[0] = 1;
        denominatorSigns[2] = 1;
    }

    private static void CopyCylinderCapsuleEllipseMagnitude(
        Signed320 value, Span<ulong> destination)
    {
        destination.Clear();
        GetMagnitude(value, destination.Slice(0, 5));
    }

    private static void SquareCylinderCapsuleEllipseMagnitude(Span<ulong> value)
    {
        Span<ulong> square = stackalloc ulong[value.Length];
        WideArithmetic.MultiplyMagnitudes(value, value, square);
        square.CopyTo(value);
    }

    private static void ScaleCylinderCapsuleEllipsePolynomial(
        Span<ulong> coefficients, ReadOnlySpan<ulong> factor, int words)
    {
        for (int index = 0; index < coefficients.Length; index += words)
            MultiplyBy(coefficients.Slice(index, words), factor);
    }

    private static void AddCylinderCapsuleEllipseCoefficient(
        Span<ulong> coefficients, Span<sbyte> signs, int index,
        ReadOnlySpan<ulong> value, int sign, int words)
    {
        Span<ulong> result = stackalloc ulong[words];
        Span<ulong> coefficient = coefficients.Slice(index * words, words);
        CombineWideSignedMagnitudes(
            coefficient, signs[index], value, sign, result, out int combinedSign);
        result.CopyTo(coefficient);
        signs[index] = (sbyte)combinedSign;
    }
}
