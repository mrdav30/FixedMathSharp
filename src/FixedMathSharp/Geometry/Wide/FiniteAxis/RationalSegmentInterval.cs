//=======================================================================
// RationalSegmentInterval.cs
//=======================================================================
// MIT License, Copyright (c) 2024-present David Oravsky (mrdav30)
// See LICENSE file in the project root for full license information.
//=======================================================================

using System;

namespace FixedMathSharp.Geometry;

/// <summary>
/// An exact closed rational subinterval of a segment's unit parameter. The
/// default value is the complete segment; borrowed calls avoid enlarging events.
/// </summary>
internal readonly struct RationalSegmentInterval
{
    internal readonly Signed832 WideLowerNumerator, WideLowerDenominator, WideUpperNumerator, WideUpperDenominator;
    internal bool HasWideBounds { get; }
    internal Signed320 LowerNumerator => GetLegacyBound(WideLowerNumerator);
    internal Signed320 LowerDenominator => GetLegacyBound(WideLowerDenominator);
    internal Signed320 UpperNumerator => GetLegacyBound(WideUpperNumerator);
    internal Signed320 UpperDenominator => GetLegacyBound(WideUpperDenominator);
    internal bool IsClipped => !WideLowerDenominator.IsZero;

    private Signed320 GetLegacyBound(Signed832 value)
    {
        if (HasWideBounds) throw new OverflowException("This interval requires its exact wide bounds.");
        return new Signed320(value.Word4, value.Word3, value.Word2, value.Word1, value.Word0);
    }

    /// <summary>
    /// Materializes one endpoint on the original 2D edge, rounding each local
    /// coordinate once to nearest-even. The retained interval, not this final
    /// point, remains authoritative for incidence and admission.
    /// </summary>
    internal Vector2d GetEndpoint(FixedSegment2d edge, bool upper)
    {
        if (!IsClipped) return upper ? edge.End : edge.Start;
        if (HasWideBounds)
        {
            Signed832 wideN = upper ? WideUpperNumerator : WideLowerNumerator;
            Signed832 wideD = upper ? WideUpperDenominator : WideLowerDenominator;
            return new Vector2d(GetWideCoordinate(edge.Start.X, edge.End.X, wideN, wideD),
                GetWideCoordinate(edge.Start.Y, edge.End.Y, wideN, wideD));
        }
        Signed320 n = upper ? UpperNumerator : LowerNumerator;
        Signed320 d = upper ? UpperDenominator : LowerDenominator;
        // Full raw endpoints and a unit Signed320 ratio need fewer than 386
        // coefficient bits; existing Signed576 interpolation preserves them.
        return new Vector2d(
            WideConvex2dRelations.GetProjectedOffsetCoordinate(edge.Start.X,
                WideArithmetic.Difference(edge.End.X, edge.Start.X), n, d),
            WideConvex2dRelations.GetProjectedOffsetCoordinate(edge.Start.Y,
                WideArithmetic.Difference(edge.End.Y, edge.Start.Y), n, d));
    }

    /// <summary>
    /// Maps this source interval onto a collinear target segment in the same
    /// local frame, preserving closed endpoints without coordinate rounding.
    /// Degenerate, off-line and disjoint segments return false. The endpoint
    /// half-plane coefficients retain their full transient products. Reduced
    /// retained bounds must fit Signed832; unsupported widths throw rather than truncate.
    /// </summary>
    internal bool TryMapToSegment(FixedSegment source, FixedSegment target, out RationalSegmentInterval mapped)
    {
        mapped = default;
        if (source.Start == source.End || target.Start == target.End) return false;
        Span<Signed192> direction = stackalloc Signed192[3], first = stackalloc Signed192[3], second = stackalloc Signed192[3];
        int axis = 0;
        for (int coordinate = 0; coordinate < 3; coordinate++)
        {
            direction[coordinate] = WideArithmetic.Difference(source.End[coordinate], source.Start[coordinate]);
            first[coordinate] = WideArithmetic.Difference(target.Start[coordinate], source.Start[coordinate]);
            second[coordinate] = WideArithmetic.Difference(target.End[coordinate], source.Start[coordinate]);
            if (!direction[coordinate].IsZero) axis = coordinate;
        }
        for (int coordinate = 0; coordinate < 3; coordinate++)
            if (!WideArithmetic.MultiplySubtract(direction[axis], first[coordinate], direction[coordinate], first[axis]).IsZero
                || !WideArithmetic.MultiplySubtract(direction[axis], second[coordinate], direction[coordinate], second[axis]).IsZero)
                return false;

        if (HasWideBounds) return TryMapWide(direction[axis], first[axis], second[axis], out mapped);

        Signed320 lowerN = default, lowerD = Signed320.One, upperN = Signed320.One, upperD = Signed320.One;
        for (int bound = 0; bound < 2; bound++)
        {
            Signed320 n = IsClipped ? (bound == 0 ? LowerNumerator : UpperNumerator) : bound == 0 ? default : Signed320.One;
            Signed320 d = IsClipped ? (bound == 0 ? LowerDenominator : UpperDenominator) : Signed320.One;
            // A full raw difference and a Signed320 ratio need <386 bits.
            // Compare in Signed576 before enforcing the clipping bank's width.
            Signed576 offset = WideArithmetic.MultiplySigned320(n, direction[axis]);
            Signed576 a = WideArithmetic.SubtractSigned576(WideArithmetic.MultiplySigned320(d, first[axis]), offset);
            Signed576 b = WideArithmetic.SubtractSigned576(WideArithmetic.MultiplySigned320(d, second[axis]), offset);
            if ((direction[axis].Sign < 0) != (bound != 0))
            { a = WideArithmetic.SubtractSigned576(default, a); b = WideArithmetic.SubtractSigned576(default, b); }
            Signed576 difference = WideArithmetic.SubtractSigned576(a, b);
            if (difference.Sign < 0) difference = WideArithmetic.SubtractSigned576(default, difference);
            if (!Signed320.TryNarrowSigned(a, out Signed320 start) || !Signed320.TryNarrowSigned(b, out Signed320 end)
                || !Signed320.TryNarrowSigned(difference, out _))
                return TryMapWide(direction[axis], first[axis], second[axis], out mapped);
            if (!TryClipHalfPlane(start, end, ref lowerN, ref lowerD, ref upperN, ref upperD)) return false;
        }
        mapped = new RationalSegmentInterval(lowerN, lowerD, upperN, upperD);
        return true;
    }

    private bool TryMapWide(Signed192 direction, Signed192 first, Signed192 second, out RationalSegmentInterval mapped)
    {
        mapped = default;
        Signed832 lowerN = default, lowerD = Signed832.ExtendValue(Signed192.One), upperN = lowerD, upperD = lowerD;
        const int words = 15;
        Span<ulong> values = stackalloc ulong[14 * words]; Span<int> signs = stackalloc int[14];
        var parameter = ContactQuadratic.At(values, signs, 0, words);
        var denominator = ContactQuadratic.At(values, signs, 1, words);
        var offset = ContactQuadratic.At(values, signs, 2, words);
        var a = ContactQuadratic.At(values, signs, 3, words);
        var b = ContactQuadratic.At(values, signs, 4, words);
        var rootN = ContactQuadratic.At(values, signs, 5, words);
        var rootD = ContactQuadratic.At(values, signs, 6, words);
        for (int bound = 0; bound < 2; bound++)
        {
            parameter.Set(bound == 0 ? WideLowerNumerator : WideUpperNumerator);
            denominator.Set(bound == 0 ? WideLowerDenominator : WideUpperDenominator);
            ContactQuadratic.Scale(parameter, Signed576.ExtendValue(Signed320.ExtendValue(direction)), offset);
            ContactQuadratic.Scale(denominator, Signed576.ExtendValue(Signed320.ExtendValue(first)), a); a.Add(offset, -1);
            ContactQuadratic.Scale(denominator, Signed576.ExtendValue(Signed320.ExtendValue(second)), b); b.Add(offset, -1);
            if ((direction.Sign < 0) != (bound != 0)) { a.MultiplySign(-1); b.MultiplySign(-1); }
            if (a.Signs[0] >= 0 && b.Signs[0] >= 0) continue;
            if (a.Signs[0] < 0 && b.Signs[0] < 0) return false;
            a.CopyTo(rootN); a.CopyTo(rootD); rootD.Add(b, -1);
            if (rootD.Signs[0] < 0) { rootN.MultiplySign(-1); rootD.MultiplySign(-1); }
            PackReducedParameter(rootN.Rational, rootD.Rational, out Signed832 n, out Signed832 d);
            // A collinear nondegenerate target has a monotone source
            // parameter, in either direction. The two ordered source bounds
            // therefore clip different target endpoints, each at most once.
            // A crossing root lies strictly inside its rejected endpoint;
            // disjoint targets were rejected above by their two negative signs.
            if (a.Signs[0] < 0) { lowerN = n; lowerD = d; }
            else { upperN = n; upperD = d; }
        }
        mapped = new RationalSegmentInterval(lowerN, lowerD, upperN, upperD);
        return true;
    }

    internal static void PackReducedParameter(ReadOnlySpan<ulong> numerator, ReadOnlySpan<ulong> denominator,
        out Signed832 n, out Signed832 d)
    {
        int words = numerator.Length;
        Span<ulong> gcd = stackalloc ulong[words], quotient = stackalloc ulong[words];
        Span<ulong> remainder = stackalloc ulong[words], scratch = stackalloc ulong[2 * words + 1];
        numerator.CopyTo(gcd); denominator.CopyTo(quotient);
        WideArithmetic.GetMagnitudeGreatestCommonDivisor(gcd, quotient, scratch);
        WideArithmetic.DivideMagnitudes(numerator, gcd, quotient, remainder, scratch); n = Pack(quotient);
        WideArithmetic.DivideMagnitudes(denominator, gcd, quotient, remainder, scratch); d = Pack(quotient);
        static Signed832 Pack(ReadOnlySpan<ulong> magnitude)
        {
            if (WideArithmetic.GetActiveMagnitudeLength(magnitude) > 13 || (magnitude[12] & (1UL << 63)) != 0)
                throw new OverflowException("Mapped segment bounds must fit Signed832 after exact reduction.");
            return new Signed832(magnitude[12], magnitude[11], magnitude[10], magnitude[9], magnitude[8], magnitude[7],
                magnitude[6], magnitude[5], magnitude[4], magnitude[3], magnitude[2], magnitude[1], magnitude[0]);
        }
    }

    /// <summary>Compares nonnegative unit parameters with positive denominators without narrowing products.</summary>
    internal static int CompareParameters(Signed320 firstN, Signed320 firstD, Signed320 secondN, Signed320 secondD) =>
        WideArithmetic.CompareNonNegativeProducts(Signed832.ExtendValue(Signed576.ExtendValue(firstN)),
            Signed832.ExtendValue(Signed576.ExtendValue(secondD)), Signed832.ExtendValue(Signed576.ExtendValue(secondN)),
            Signed832.ExtendValue(Signed576.ExtendValue(firstD)));

    internal static int CompareParameters(Signed832 firstN, Signed832 firstD, Signed832 secondN, Signed832 secondD) =>
        WideArithmetic.CompareNonNegativeProducts(firstN, secondD, secondN, firstD);

    /// <summary>
    /// Clips a unit interval by the linear half-plane value at its original
    /// endpoints. Bounds have positive denominators; first-second must fit
    /// Signed320 (current framed coverage callers require fewer than 268 bits).
    /// </summary>
    internal static bool TryClipHalfPlane(Signed320 first, Signed320 second,
        ref Signed320 lowerN, ref Signed320 lowerD, ref Signed320 upperN, ref Signed320 upperD)
    {
        if (first.Sign >= 0 && second.Sign >= 0) return true;
        if (first.Sign < 0 && second.Sign < 0) return false;
        Signed320 n = first, d = WideArithmetic.SubtractSigned320(first, second);
        if (d.Sign < 0) { n = WideArithmetic.Negate(n); d = WideArithmetic.Negate(d); }
        if (first.Sign < 0)
        {
            if (CompareParameters(n, d, lowerN, lowerD) > 0) { lowerN = n; lowerD = d; }
        }
        else if (CompareParameters(n, d, upperN, upperD) < 0) { upperN = n; upperD = d; }
        return CompareParameters(lowerN, lowerD, upperN, upperD) <= 0;
    }

    internal RationalSegmentInterval(Signed320 lowerNumerator, Signed320 lowerDenominator,
        Signed320 upperNumerator, Signed320 upperDenominator)
        : this(Signed832.ExtendValue(Signed576.ExtendValue(lowerNumerator)),
            Signed832.ExtendValue(Signed576.ExtendValue(lowerDenominator)),
            Signed832.ExtendValue(Signed576.ExtendValue(upperNumerator)),
            Signed832.ExtendValue(Signed576.ExtendValue(upperDenominator)))
    {
    }

    internal RationalSegmentInterval(Signed832 lowerNumerator, Signed832 lowerDenominator,
        Signed832 upperNumerator, Signed832 upperDenominator)
    {
        if (lowerDenominator.Sign <= 0 || upperDenominator.Sign <= 0)
            throw new ArgumentOutOfRangeException(nameof(lowerDenominator), "Interval denominators must be positive.");
        if (lowerNumerator.Sign < 0 || upperNumerator.Sign < 0
            || WideArithmetic.SubtractSigned832(lowerDenominator, lowerNumerator).Sign < 0
            || WideArithmetic.SubtractSigned832(upperDenominator, upperNumerator).Sign < 0)
            throw new ArgumentOutOfRangeException(nameof(lowerNumerator), "Interval bounds must lie in [0,1].");
        if (CompareParameters(lowerNumerator, lowerDenominator, upperNumerator, upperDenominator) > 0)
            throw new ArgumentException("The lower interval bound must not exceed the upper bound.");
        this = default;
        if (lowerNumerator.IsZero && upperNumerator.Equals(upperDenominator)) return;
        WideLowerNumerator = lowerNumerator; WideLowerDenominator = lowerDenominator;
        WideUpperNumerator = upperNumerator; WideUpperDenominator = upperDenominator;
        HasWideBounds = !FitsLegacy(lowerNumerator) || !FitsLegacy(lowerDenominator)
            || !FitsLegacy(upperNumerator) || !FitsLegacy(upperDenominator);
    }

    /// <summary>Restricts an already admitted unit parameter; requires a clipped interval.</summary>
    internal bool Contains(scoped ContactQuadratic numerator, scoped ContactQuadratic denominator,
        scoped ReadOnlySpan<ulong> root)
    {
        System.Diagnostics.Debug.Assert(IsClipped);
        // Only the retained parameter width changes; its event radicand is unchanged.
        int words = Math.Max(ContactQuadratic.ActiveWords(numerator), ContactQuadratic.ActiveWords(denominator))
            + (HasWideBounds ? 14 : 6);
        Span<ulong> values = stackalloc ulong[4 * words]; Span<int> signs = stackalloc int[4];
        var query = ContactQuadratic.At(values, signs, 0, words);
        var term = ContactQuadratic.At(values, signs, 1, words);
        ScaleParameter(numerator, WideLowerDenominator, query);
        ScaleParameter(denominator, WideLowerNumerator, term); query.Add(term, -1);
        if (query.Sign(root) < 0) return false;
        ScaleParameter(denominator, WideUpperNumerator, query);
        ScaleParameter(numerator, WideUpperDenominator, term); query.Add(term, -1);
        return query.Sign(root) >= 0;
    }

    private static bool FitsLegacy(Signed832 value)
    {
        ulong extension = (value.Word4 & (1UL << 63)) != 0 ? ulong.MaxValue : 0UL;
        return value.Word5 == extension && value.Word6 == extension && value.Word7 == extension
            && value.Word8 == extension && value.Word9 == extension && value.Word10 == extension
            && value.Word11 == extension && value.Word12 == extension;
    }

    private static void ScaleParameter(ContactQuadratic value, Signed832 parameter, ContactQuadratic result)
    {
        Span<ulong> magnitude = stackalloc ulong[13]; WideArithmetic.GetMagnitude(parameter, magnitude);
        ContactQuadratic.Scale(value, magnitude, parameter.Sign, result);
    }

    private static Fixed64 GetWideCoordinate(Fixed64 start, Fixed64 end, Signed832 n, Signed832 d)
    {
        const int words = 15;
        Span<ulong> values = stackalloc ulong[8 * words]; Span<int> signs = stackalloc int[8];
        var parameter = ContactQuadratic.At(values, signs, 0, words);
        var denominator = ContactQuadratic.At(values, signs, 1, words);
        var numerator = ContactQuadratic.At(values, signs, 2, words);
        var term = ContactQuadratic.At(values, signs, 3, words);
        parameter.Set(n); denominator.Set(d);
        ContactQuadratic.Scale(denominator, Signed576.ExtendValue(Signed320.ExtendValue(Signed192.Raw(start))), numerator);
        ContactQuadratic.Scale(parameter, Signed576.ExtendValue(Signed320.ExtendValue(WideArithmetic.Difference(end, start))), term);
        numerator.Add(term);
        return ContactQuadratic.RoundRatio(numerator, denominator, ReadOnlySpan<ulong>.Empty);
    }
}
