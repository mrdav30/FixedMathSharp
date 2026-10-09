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
        int words = Math.Max(numerator.FieldWords, denominator.FieldWords) + 2;
        Span<ulong> values = stackalloc ulong[4 * words];
        Span<int> signs = stackalloc int[4];
        ContactQuadratic doubled = At(values, signs, 0, words), query = At(values, signs, 1, words);
        numerator.CopyTo(doubled); doubled.Add(numerator);
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

    internal static void Scale(ContactQuadratic value, Signed576 scalar, ContactQuadratic result)
    {
        Span<ulong> magnitude = stackalloc ulong[9];
        Import(scalar, magnitude);
        Scale(value, magnitude, scalar.Sign, result);
    }

    // The caller proves that result has room for both complete products.
    internal static void Scale(ContactQuadratic value, ReadOnlySpan<ulong> magnitude,
        int scalarSign, ContactQuadratic result)
    {
        WideArithmetic.MultiplyMagnitudes(value.Rational, magnitude, result.Rational);
        WideArithmetic.MultiplyMagnitudes(value.Radical, magnitude, result.Radical);
        result.Signs[0] = IsZero(result.Rational) ? 0 : value.Signs[0] * scalarSign;
        result.Signs[1] = IsZero(result.Radical) ? 0 : value.Signs[1] * scalarSign;
    }

    internal static void Multiply(ContactQuadratic first, ContactQuadratic second, ReadOnlySpan<ulong> root, ContactQuadratic result)
    {
        int words = result.FieldWords;
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
