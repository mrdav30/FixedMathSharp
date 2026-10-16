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

    internal RationalSegmentInterval(Signed320 lowerNumerator, Signed320 lowerDenominator,
        Signed320 upperNumerator, Signed320 upperDenominator)
    {
        if (lowerDenominator.Sign <= 0 || upperDenominator.Sign <= 0)
            throw new ArgumentOutOfRangeException(nameof(lowerDenominator), "Interval denominators must be positive.");
        if (lowerNumerator.Sign < 0 || upperNumerator.Sign < 0
            || WideArithmetic.SubtractSigned320(lowerDenominator, lowerNumerator).Sign < 0
            || WideArithmetic.SubtractSigned320(upperDenominator, upperNumerator).Sign < 0)
            throw new ArgumentOutOfRangeException(nameof(lowerNumerator), "Interval bounds must lie in [0,1].");
        if (WideArithmetic.CompareNonNegativeProducts(Signed832.ExtendValue(Signed576.ExtendValue(lowerNumerator)),
                Signed832.ExtendValue(Signed576.ExtendValue(upperDenominator)),
                Signed832.ExtendValue(Signed576.ExtendValue(upperNumerator)),
                Signed832.ExtendValue(Signed576.ExtendValue(lowerDenominator))) > 0)
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
