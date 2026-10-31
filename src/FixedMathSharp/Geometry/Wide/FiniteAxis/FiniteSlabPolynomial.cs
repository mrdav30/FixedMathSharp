//=======================================================================
// FiniteSlabPolynomial.cs
//=======================================================================
// MIT License, Copyright (c) 2024-present David Oravsky (mrdav30)
// See LICENSE file in the project root for full license information.
//=======================================================================

using System;

namespace FixedMathSharp.Geometry;

/// <summary>
/// Borrowed polynomial arithmetic for this owner's affine and conic charts.
/// All limb operations remain in the existing wide arithmetic owner.
/// </summary>
internal readonly ref struct FiniteSlabPolynomial
{
    internal const int Words = 64;
    internal const int AffineWords = 128;
    internal int FieldWords => Values.Length / Signs.Length;
    internal readonly Span<ulong> Values;
    internal readonly Span<sbyte> Signs;
    internal int Count => Signs.Length;
    internal bool IsZero { get { foreach (sbyte sign in Signs) if (sign != 0) return false; return true; } }
    internal FiniteSlabPolynomial(Span<ulong> values, Span<sbyte> signs)
    { Values = values; Signs = signs; }
    internal static FiniteSlabPolynomial At(Span<ulong> values, Span<sbyte> signs, int index, int count) =>
        new(values.Slice(index * (values.Length / signs.Length), count * (values.Length / signs.Length)), signs.Slice(index, count));
    internal FiniteSlabPolynomial Slice(int index, int count) => At(Values, Signs, index, count);
    internal void Clear() { Values.Clear(); Signs.Clear(); }
    internal void Set(Signed576 value, int index = 0)
    {
        Span<ulong> slot = Values.Slice(index * FieldWords, FieldWords);
        WideArithmetic.GetMagnitude(value, slot); Signs[index] = (sbyte)value.Sign;
    }
    internal void Set(Signed832 value, int index = 0)
    {
        Span<ulong> slot = Values.Slice(index * FieldWords, FieldWords);
        WideArithmetic.GetMagnitude(value, slot); Signs[index] = (sbyte)value.Sign;
    }
    internal void Set(Signed320 value, int index = 0) => Set(Signed576.ExtendValue(value), index);
    internal void Set(int value, int index = 0) => Set(Signed320.ExtendValue(Signed192.Signed(value)), index);
    internal void Copy(FiniteSlabPolynomial source)
    {
        Clear();
        if (FieldWords == source.FieldWords)
        {
            source.Values.CopyTo(Values); source.Signs.CopyTo(Signs);
        }
        else Add(source);
    }
    internal void Add(FiniteSlabPolynomial source, int multiplier = 1, int offset = 0) =>
        WideFiniteAxisIntersection.AddFiniteAxisPolynomial(source.Values, source.Signs,
            Values, Signs, multiplier, offset);
    internal void Multiply(FiniteSlabPolynomial left, FiniteSlabPolynomial right)
    {
        Span<ulong> product = FieldWords == Words ? stackalloc ulong[Words] : stackalloc ulong[AffineWords];
        WideFiniteAxisIntersection.MultiplyFiniteAxisPolynomials(left.Values, left.Signs,
            right.Values, right.Signs, Values, Signs, product);
    }
    internal void Scale(FiniteSlabPolynomial source, Signed576 factor)
    {
        if (factor.IsZero) { Clear(); return; }
        // Polynomial coefficients are integers; these units are literal 1,
        // rather than the Q32.32 value retained by Signed576.One.
        if ((factor.Word0 == 1UL && factor.Equals(Signed576.ExtendValue(Signed192.Signed(1))))
            || (factor.Word0 == ulong.MaxValue && factor.Equals(Signed576.ExtendValue(Signed192.Signed(-1)))))
        {
            Clear(); Add(source, factor.Sign);
            return;
        }
        // The scalar carrier fills nine words; multiplication supports independent input strides.
        Span<ulong> data = stackalloc ulong[9];
        Span<sbyte> signs = stackalloc sbyte[1];
        FiniteSlabPolynomial scalar = new(data, signs); scalar.Set(factor);
        Multiply(source, scalar);
    }
    internal void Scale(FiniteSlabPolynomial source, Signed832 factor)
    {
        if (factor.IsZero) { Clear(); return; }
        if ((factor.Word0 == 1UL && factor.Equals(Signed832.ExtendValue(Signed192.Signed(1))))
            || (factor.Word0 == ulong.MaxValue && factor.Equals(Signed832.ExtendValue(Signed192.Signed(-1)))))
        {
            Clear(); Add(source, factor.Sign);
            return;
        }
        // The scalar carrier fills thirteen words, independently of the polynomial bank width.
        Span<ulong> data = stackalloc ulong[13];
        Span<sbyte> signs = stackalloc sbyte[1];
        FiniteSlabPolynomial scalar = new(data, signs); scalar.Set(factor);
        Multiply(source, scalar);
    }
    internal void Scale(FiniteSlabPolynomial source, Signed320 factor) => Scale(source, Signed576.ExtendValue(factor));
    internal static int Sign(scoped ref FiniteAxisValueRoot root, scoped FiniteSlabPolynomial query)
    {
        int degree = query.Count - 1;
        while (degree > 0 && query.Signs[degree] == 0) degree--;
        if (degree == 0) return query.Signs[0];
        // A common positive scale cannot change a sign. Strip it from a
        // scratch copy, never from a retained coordinate/denominator ratio;
        // otherwise exact-zero queries pay a needlessly large equality bound.
        Span<ulong> values = stackalloc ulong[query.Values.Length]; query.Values.CopyTo(values);
        WideFiniteAxisIntersection.NormalizeFiniteAxisPolynomialPowerOfTwo(values, query.Signs);
        return WideFiniteAxisIntersection.GetSignAtFiniteValueRootAndRefine(ref root, values, query.Signs);
    }
}
