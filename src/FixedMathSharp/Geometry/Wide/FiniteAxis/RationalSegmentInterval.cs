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
    internal readonly Signed320 LowerNumerator, LowerDenominator, UpperNumerator, UpperDenominator;
    internal bool IsClipped => !LowerDenominator.IsZero;

    /// <summary>
    /// Materializes one endpoint on the original 2D edge, rounding each local
    /// coordinate once to nearest-even. The retained interval, not this final
    /// point, remains authoritative for incidence and admission.
    /// </summary>
    internal Vector2d GetEndpoint(FixedSegment2d edge, bool upper)
    {
        if (!IsClipped) return upper ? edge.End : edge.Start;
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
    /// half-plane coefficients and their absolute differences must fit Signed320;
    /// unsupported widths throw rather than truncate an interval.
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
                throw new OverflowException("Mapped segment half-planes must fit Signed320.");
            if (!TryClipHalfPlane(start, end, ref lowerN, ref lowerD, ref upperN, ref upperD)) return false;
        }
        mapped = new RationalSegmentInterval(lowerN, lowerD, upperN, upperD);
        return true;
    }

    /// <summary>Compares nonnegative unit parameters with positive denominators without narrowing products.</summary>
    internal static int CompareParameters(Signed320 firstN, Signed320 firstD, Signed320 secondN, Signed320 secondD) =>
        WideArithmetic.CompareNonNegativeProducts(Signed832.ExtendValue(Signed576.ExtendValue(firstN)),
            Signed832.ExtendValue(Signed576.ExtendValue(secondD)), Signed832.ExtendValue(Signed576.ExtendValue(secondN)),
            Signed832.ExtendValue(Signed576.ExtendValue(firstD)));

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
    {
        if (lowerDenominator.Sign <= 0 || upperDenominator.Sign <= 0)
            throw new ArgumentOutOfRangeException(nameof(lowerDenominator), "Interval denominators must be positive.");
        if (lowerNumerator.Sign < 0 || upperNumerator.Sign < 0
            || WideArithmetic.SubtractSigned320(lowerDenominator, lowerNumerator).Sign < 0
            || WideArithmetic.SubtractSigned320(upperDenominator, upperNumerator).Sign < 0)
            throw new ArgumentOutOfRangeException(nameof(lowerNumerator), "Interval bounds must lie in [0,1].");
        if (CompareParameters(lowerNumerator, lowerDenominator, upperNumerator, upperDenominator) > 0)
            throw new ArgumentException("The lower interval bound must not exceed the upper bound.");
        this = default;
        if (lowerNumerator.IsZero && upperNumerator.Equals(upperDenominator)) return;
        LowerNumerator = lowerNumerator; LowerDenominator = lowerDenominator;
        UpperNumerator = upperNumerator; UpperDenominator = upperDenominator;
    }

    /// <summary>Restricts an already admitted unit parameter; requires a clipped interval.</summary>
    internal bool Contains(scoped ContactQuadratic numerator, scoped ContactQuadratic denominator,
        scoped ReadOnlySpan<ulong> root)
    {
        System.Diagnostics.Debug.Assert(IsClipped);
        // Bounds add at most 319 bits; the event radicand is unchanged.
        int words = Math.Max(ContactQuadratic.ActiveWords(numerator), ContactQuadratic.ActiveWords(denominator)) + 6;
        Span<ulong> values = stackalloc ulong[4 * words]; Span<int> signs = stackalloc int[4];
        var query = ContactQuadratic.At(values, signs, 0, words);
        var term = ContactQuadratic.At(values, signs, 1, words);
        ContactQuadratic.Scale(numerator, Signed576.ExtendValue(LowerDenominator), query);
        ContactQuadratic.Scale(denominator, Signed576.ExtendValue(LowerNumerator), term); query.Add(term, -1);
        if (query.Sign(root) < 0) return false;
        ContactQuadratic.Scale(denominator, Signed576.ExtendValue(UpperNumerator), query);
        ContactQuadratic.Scale(numerator, Signed576.ExtendValue(UpperDenominator), term); query.Add(term, -1);
        return query.Sign(root) >= 0;
    }
}
