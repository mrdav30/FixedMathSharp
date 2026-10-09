//=======================================================================
// ContactQuadratic.cs
//=======================================================================
// MIT License, Copyright (c) 2024-present David Oravsky (mrdav30)
// See LICENSE file in the project root for full license information.
//=======================================================================
using System;
using static FixedMathSharp.Geometry.CylinderContactAlgebra;

namespace FixedMathSharp.Geometry;

/// <summary>Borrowed exact quadratic-field values and final nearest-even ratios.</summary>
/// <content>Owns quadratic-field arithmetic; ratio comparison and materialization share this borrowed layout.</content>
internal readonly ref partial struct ContactQuadratic
{
    internal readonly Span<ulong> Values;
    internal readonly Span<int> Signs;
    internal int FieldWords => Values.Length / 2;
    internal Span<ulong> Rational => Values[..FieldWords];
    internal Span<ulong> Radical => Values[FieldWords..];
    internal ContactQuadratic(Span<ulong> values, Span<int> signs) { Values = values; Signs = signs; }
    internal void Clear() { Values.Clear(); Signs.Clear(); }
    internal void Set(Signed576 value)
    {
        Clear(); Import(value, Rational); Signs[0] = value.Sign;
    }
    internal void CopyTo(ContactQuadratic destination, int multiplier = 1)
    {
        Rational.CopyTo(destination.Rational); Radical.CopyTo(destination.Radical);
        destination.Rational[FieldWords..].Clear(); destination.Radical[FieldWords..].Clear();
        destination.Signs[0] = Signs[0] * multiplier; destination.Signs[1] = Signs[1] * multiplier;
    }
    internal void MultiplySign(int sign) { Signs[0] *= sign; Signs[1] *= sign; }
    internal void Add(ContactQuadratic value, int multiplier = 1)
    {
        int rationalSign = Signs[0], radicalSign = Signs[1];
        CylinderContactAlgebra.Add(value.Rational, value.Signs[0] * multiplier, Rational, ref rationalSign);
        CylinderContactAlgebra.Add(value.Radical, value.Signs[1] * multiplier, Radical, ref radicalSign);
        Signs[0] = rationalSign; Signs[1] = radicalSign;
    }
    internal int Sign(ReadOnlySpan<ulong> root) =>
        WideConvexPrismRelations.GetConvexContactCandidateQuadraticSign(Rational, Signs[0], Radical, Signs[1], root);

    internal static ContactQuadratic At(Span<ulong> values, Span<int> signs, int index, int words = Words) =>
        new(values.Slice(index * 2 * words, 2 * words), signs.Slice(index * 2, 2));

    internal static Fixed64 RoundRatio(ContactQuadratic numerator, ContactQuadratic denominator, ReadOnlySpan<ulong> root,
        long low = long.MinValue, long high = long.MaxValue, long parityOffset = 0)
    {
        if (numerator.Signs[1] == 0 && denominator.Signs[1] == 0 && (parityOffset & 1) == 0)
        {
            bool represented = Fixed64.TryGetSignedRawRatio(numerator.Rational, denominator.Rational,
                numerator.Signs[0] < 0, out Fixed64 result);
            System.Diagnostics.Debug.Assert(represented);
            return result;
        }
        RefineRoundRatioBounds(numerator, denominator, root, ref low, ref high);
        int words = Math.Max(ActiveWords(numerator), ActiveWords(denominator)) + 2;
        Span<ulong> values = stackalloc ulong[4 * words];
        Span<int> signs = stackalloc int[4];
        ContactQuadratic doubled = At(values, signs, 0, words), query = At(values, signs, 1, words);
        // Retained fields are padded to their worst-case proof width. Scaling
        // uses only complete active products; two extra limbs cover doubling
        // and every signed 65-bit threshold without copying padded tails.
        Scale(numerator, Signed576.ExtendValue(Signed320.ExtendValue(Signed192.Signed(2))), doubled);
        while (low < high)
        {
            ulong span = unchecked((ulong)high - (ulong)low);
            long midpoint = unchecked((long)((ulong)low + (span >> 1) + (span & 1)));
            Signed192 threshold = WideArithmetic.AddSigned192(Signed192.Signed(midpoint), Signed192.Signed(midpoint));
            Scale(denominator, Signed576.ExtendValue(Signed320.ExtendValue(threshold)), query);
            query.MultiplySign(-1); query.Add(doubled);
            if (query.Sign(root) >= 0) low = midpoint;
            else high = midpoint - 1;
        }
        Signed192 half = WideArithmetic.AddSigned192(WideArithmetic.AddSigned192(Signed192.Signed(low), Signed192.Signed(low)), Signed192.Signed(1));
        Scale(denominator, Signed576.ExtendValue(Signed320.ExtendValue(half)), query);
        query.MultiplySign(-1); query.Add(doubled);
        int comparison = query.Sign(root);
        return Fixed64.FromRaw(low + (comparison > 0 || comparison == 0 && ((low ^ parityOffset) & 1) != 0 ? 1 : 0));
    }

    private static void RefineRoundRatioBounds(ContactQuadratic numerator, ContactQuadratic denominator,
        ReadOnlySpan<ulong> root, ref long low, ref long high)
    {
        WideArithmetic.GetMagnitudeSquareRootBounds(root,
            out Signed192 lowerRoot, out Signed192 upperRoot, out int shift);
        Span<ulong> roots = stackalloc ulong[4]
            { lowerRoot.Low, lowerRoot.Middle, upperRoot.Low, upperRoot.Middle };
        int words = Math.Max(2, Math.Max(Math.Max(ActiveWords(numerator), ActiveWords(denominator)),
            Math.Max(WideArithmetic.GetActiveMagnitudeLength(numerator.Radical),
                WideArithmetic.GetActiveMagnitudeLength(denominator.Radical)) + 2 + ((shift + 63) >> 6))) + 1;
        Span<ulong> bounds = stackalloc ulong[4 * words];
        Span<ulong> nMin = bounds[..words], nMax = bounds.Slice(words, words);
        Span<ulong> dMin = bounds.Slice(2 * words, words), dMax = bounds[(3 * words)..];
        WideConvexPrismRelations.GetConvexContactCandidateQuadraticBounds(numerator.Rational, numerator.Signs[0],
            numerator.Radical, numerator.Signs[1], roots, shift, nMin, nMax, out int nMinSign, out int nMaxSign);
        WideConvexPrismRelations.GetConvexContactCandidateQuadraticBounds(denominator.Rational, denominator.Signs[0],
            denominator.Radical, denominator.Signs[1], roots, shift, dMin, dMax, out int dMinSign, out _);
        // Division by a strictly positive interval is monotone in N; the
        // denominator endpoint reverses when N is negative. Cancellation can
        // make the enclosure unusable, so exact full-range search remains valid.
        if (dMinSign <= 0
            || !Fixed64.TryGetSignedRawRatio(nMin, nMinSign < 0 ? dMin : dMax, nMinSign < 0, out Fixed64 roundedMin)
            || !Fixed64.TryGetSignedRawRatio(nMax, nMaxSign < 0 ? dMax : dMin, nMaxSign < 0, out Fixed64 roundedMax)) return;
        // floor(x) lies in [roundEven(lower)-1, roundEven(upper)]. Preserve
        // the caller's clipped floor domain, including values outside it; the
        // original exact integer and half-raw signs still decide every output.
        long lower = roundedMin.m_rawValue == long.MinValue ? long.MinValue : roundedMin.m_rawValue - 1;
        low = Math.Min(high, Math.Max(low, lower));
        high = Math.Max(low, Math.Min(high, roundedMax.m_rawValue));
    }

    internal static void Scale(ContactQuadratic value, Signed576 scalar, ContactQuadratic result)
    {
        Span<ulong> magnitude = stackalloc ulong[9];
        Import(scalar, magnitude);
        Scale(value, magnitude, scalar.Sign, result);
    }

    // The caller proves that result has room for both complete products and
    // does not alias the source coefficient or scalar storage.
    internal static void Scale(ContactQuadratic value, ReadOnlySpan<ulong> magnitude,
        int scalarSign, ContactQuadratic result)
    {
        int rationalSign = value.Signs[0] * scalarSign, radicalSign = value.Signs[1] * scalarSign;
        // Retained sign zero is authoritative even when borrowed bank bytes
        // are stale. Clear output rather than scan/multiply a coefficient
        // that has no mathematical contribution.
        if (rationalSign == 0)
        {
            result.Rational.Clear(); result.Signs[0] = 0;
        }
        else
        {
            WideArithmetic.MultiplyMagnitudes(value.Rational, magnitude, result.Rational);
            result.Signs[0] = IsZero(result.Rational) ? 0 : rationalSign;
        }
        if (radicalSign == 0)
        {
            result.Radical.Clear(); result.Signs[1] = 0;
        }
        else
        {
            WideArithmetic.MultiplyMagnitudes(value.Radical, magnitude, result.Radical);
            result.Signs[1] = IsZero(result.Radical) ? 0 : radicalSign;
        }
    }

    internal static void Multiply(ContactQuadratic first, ContactQuadratic second, ReadOnlySpan<ulong> root, ContactQuadratic result)
    {
        int words = result.FieldWords;
        if (first.Signs[1] == 0 && second.Signs[1] == 0)
        {
            // Rational events need one complete product. Keep separate scratch
            // because result may alias either borrowed input; clear the unused
            // radical bank so later reuse cannot retain an old field.
            Span<ulong> rationalProduct = stackalloc ulong[words];
            int sign = first.Signs[0] * second.Signs[0];
            WideArithmetic.MultiplyMagnitudes(first.Rational, second.Rational, rationalProduct);
            rationalProduct.CopyTo(result.Rational); result.Radical.Clear();
            result.Signs[0] = IsZero(rationalProduct) ? 0 : sign; result.Signs[1] = 0;
            return;
        }
        Span<ulong> scratch = stackalloc ulong[3 * words];
        Span<ulong> rational = scratch[..words], radical = scratch.Slice(words, words), product = scratch[(2 * words)..];
        WideArithmetic.MultiplyMagnitudes(first.Rational, second.Rational, rational);
        int rationalSign = IsZero(rational) ? 0 : first.Signs[0] * second.Signs[0];
        WideArithmetic.MultiplyMagnitudes(first.Radical, second.Radical, product);
        WideArithmetic.MultiplyMagnitudes(product, root, radical);
        CylinderContactAlgebra.Add(radical, first.Signs[1] * second.Signs[1], rational, ref rationalSign);
        WideArithmetic.MultiplyMagnitudes(first.Rational, second.Radical, radical);
        int radicalSign = IsZero(radical) ? 0 : first.Signs[0] * second.Signs[1];
        WideArithmetic.MultiplyMagnitudes(first.Radical, second.Rational, product);
        CylinderContactAlgebra.Add(product, first.Signs[1] * second.Signs[0], radical, ref radicalSign);
        rational.CopyTo(result.Rational); radical.CopyTo(result.Radical);
        result.Signs[0] = rationalSign; result.Signs[1] = radicalSign;
    }

    internal static void MultiplyRoot(ContactQuadratic value, ReadOnlySpan<ulong> root, ContactQuadratic result)
    {
        WideArithmetic.MultiplyMagnitudes(value.Radical, root, result.Rational);
        value.Rational.CopyTo(result.Radical);
        result.Signs[0] = IsZero(result.Rational) ? 0 : value.Signs[1];
        result.Signs[1] = value.Signs[0];
    }

}
