//=======================================================================
// CylinderContactAlgebra.cs
//=======================================================================
// MIT License, Copyright (c) 2024-present David Oravsky (mrdav30)
// See LICENSE file in the project root for full license information.
//=======================================================================

using System;

namespace FixedMathSharp.Geometry;

/// <summary>Shared exact radial support-candidate encoding for finite cylinders.</summary>
internal static class CylinderContactAlgebra
{
    internal const int Words = ConvexContactCandidate.Words;

    /// <summary>Encodes a signed quadratic numerator over a positive square-root denominator.</summary>
    internal static int BuildQuadraticCandidate(ContactQuadratic numerator, ReadOnlySpan<ulong> root,
        ReadOnlySpan<ulong> denominator, Span<ulong> values, Span<int> signs)
    {
        // (A+B*sqrt(K))²=(A²+B²K)+2AB*sqrt(K). Squaring needs one
        // cross-product; its rational coefficient is always nonnegative.
        Span<ulong> product = stackalloc ulong[Words];
        Span<ulong> rational = Slot(values, 7), radical = Slot(values, 8);
        WideArithmetic.MultiplyMagnitudes(numerator.Rational, numerator.Rational, rational);
        WideArithmetic.MultiplyMagnitudes(numerator.Radical, numerator.Radical, product);
        WideArithmetic.MultiplyMagnitudes(product, root, radical);
        WideArithmetic.AddMagnitudeInto(radical, rational);
        WideArithmetic.MultiplyMagnitudes(numerator.Rational, numerator.Radical, radical);
        WideArithmetic.AddMagnitudeInto(radical, radical);
        root.CopyTo(Slot(values, 9)); denominator.CopyTo(Slot(values, 10));
        signs[7] = IsZero(rational) ? 0 : 1;
        signs[8] = IsZero(radical) ? 0 : numerator.Signs[0] * numerator.Signs[1];
        return numerator.Sign(root);
    }

    /// <summary>Encodes (p+sqrt(q))/sqrt(d), including its signed gap.</summary>
    internal static int BuildRadialCandidate(ReadOnlySpan<ulong> p, int pSign,
        ReadOnlySpan<ulong> q, ReadOnlySpan<ulong> d, Span<ulong> values, Span<int> signs)
    {
        Span<ulong> squared = stackalloc ulong[Words];
        WideArithmetic.MultiplyMagnitudes(p, p, squared);
        WideArithmetic.AddEqualMagnitudes(squared, q, Slot(values, 7));
        WideArithmetic.AddEqualMagnitudes(p, p, Slot(values, 8));
        q.CopyTo(Slot(values, 9));
        d.CopyTo(Slot(values, 10));
        signs[7] = IsZero(Slot(values, 7)) ? 0 : 1;
        signs[8] = pSign;
        return pSign < 0 ? WideArithmetic.CompareMagnitudeEqualLength(q, squared)
            : pSign > 0 || !IsZero(q) ? 1 : 0;
    }

    internal static void WriteWorldDirection(WideRationalBasis3d basis,
        ReadOnlySpan<ulong> direction, ReadOnlySpan<int> directionSigns,
        Span<ulong> values, Span<int> signs, int orientation = 1)
    {
        int Words = direction.Length / 3;
        for (int component = 0; component < 3; component++)
        {
            WideAxis3 row = component == 0 ? Axis(basis.Xx, basis.Yx, basis.Zx)
                : component == 1 ? Axis(basis.Xy, basis.Yy, basis.Zy) : Axis(basis.Xz, basis.Yz, basis.Zz);
            Dot(row, direction, directionSigns, Slot(values, component, Words), out int sign);
            signs[component] = orientation * sign;
        }
    }

    internal static void Dot(WideAxis3 axis, ReadOnlySpan<ulong> direction, ReadOnlySpan<int> signs,
        Span<ulong> result, out int sign)
    {
        int Words = result.Length;
        Span<ulong> component = stackalloc ulong[Words];
        Span<ulong> product = stackalloc ulong[Words];
        result.Clear(); sign = 0;
        for (int index = 0; index < 3; index++)
        {
            Signed320 value = Component(axis, index);
            Import(value, component);
            WideArithmetic.MultiplyMagnitudes(component, Slot(direction, index, Words), product);
            Add(product, value.Sign * signs[index], result, ref sign);
        }
    }

    internal static void Dot(in WideDirection3d axis, ReadOnlySpan<ulong> direction, ReadOnlySpan<int> signs,
        Span<ulong> result, out int sign)
    {
        int Words = result.Length;
        Span<ulong> component = stackalloc ulong[Words], product = stackalloc ulong[Words];
        result.Clear(); sign = 0;
        for (int index = 0; index < 3; index++)
        {
            Signed576 value = Component(axis, index);
            Import(value, component);
            WideArithmetic.MultiplyMagnitudes(component, Slot(direction, index, Words), product);
            Add(product, value.Sign * signs[index], result, ref sign);
        }
    }

    internal static void ProjectPerpendicular(WideAxis3 axis, WideAxis3 value,
        Span<ulong> direction, Span<int> signs)
    {
        Span<ulong> work = stackalloc ulong[4 * Words];
        Span<ulong> u = Slot(work, 0), dot = Slot(work, 1), component = Slot(work, 2), product = Slot(work, 3);
        Import(axis.SquaredLength, u);
        Signed576 projection = WideAxis3.Dot(axis, value);
        Import(projection, dot);
        for (int index = 0; index < 3; index++)
        {
            Signed320 v = Component(value, index);
            Signed320 a = Component(axis, index);
            Import(v, component);
            WideArithmetic.MultiplyMagnitudes(u, component, Slot(direction, index));
            int sign = v.Sign;
            Import(a, component);
            WideArithmetic.MultiplyMagnitudes(dot, component, product);
            Add(product, -projection.Sign * a.Sign, Slot(direction, index), ref sign);
            signs[index] = sign;
        }
    }

    internal static void SumSquares(ReadOnlySpan<ulong> direction, Span<ulong> result)
    {
        int Words = result.Length;
        Span<ulong> product = stackalloc ulong[Words];
        result.Clear();
        for (int index = 0; index < 3; index++)
        {
            ReadOnlySpan<ulong> component = Slot(direction, index, Words);
            WideArithmetic.MultiplyMagnitudes(component, component, product);
            WideArithmetic.AddMagnitudeInto(product, result);
        }
    }

    internal static void ProjectPerpendicular(in WideDirection3d axis, in WideDirection3d value,
        Span<ulong> direction, Span<int> signs)
    {
        Span<ulong> work = stackalloc ulong[4 * Words];
        Span<ulong> norm = Slot(work, 0), dot = Slot(work, 1), component = Slot(work, 2), product = Slot(work, 3);
        Import(axis.SquaredLength, norm);
        Signed832 projection = WideDirection3d.Dot(axis, value);
        Import(projection, dot);
        for (int index = 0; index < 3; index++)
        {
            Signed576 v = Component(value, index), a = Component(axis, index);
            Import(v, component);
            WideArithmetic.MultiplyMagnitudes(norm, component, Slot(direction, index));
            int sign = v.Sign;
            Import(a, component);
            WideArithmetic.MultiplyMagnitudes(dot, component, product);
            Add(product, -projection.Sign * a.Sign, Slot(direction, index), ref sign);
            signs[index] = sign;
        }
    }

    internal static void WriteDirection(WideAxis3 axis, Span<ulong> direction, Span<int> signs)
    {
        int Words = direction.Length / 3;
        for (int index = 0; index < 3; index++)
        {
            Signed320 value = Component(axis, index);
            Import(value, Slot(direction, index, Words)); signs[index] = value.Sign;
        }
    }
    internal static void WriteDirection(in WideDirection3d axis, Span<ulong> direction, Span<int> signs)
    {
        int Words = direction.Length / 3;
        for (int index = 0; index < 3; index++)
        {
            Signed576 value = Component(axis, index);
            Import(value, Slot(direction, index, Words)); signs[index] = value.Sign;
        }
    }

    internal static void Add(ReadOnlySpan<ulong> value, int sign, Span<ulong> destination, ref int destinationSign) =>
        WideArithmetic.AddShiftedSignedMagnitude(value, sign, 0, destination, ref destinationSign);
    internal static bool IsZero(ReadOnlySpan<ulong> value) => WideArithmetic.GetActiveMagnitudeLength(value) == 0;
    internal static Span<ulong> Slot(Span<ulong> values, int index) => values.Slice(index * Words, Words);
    internal static ReadOnlySpan<ulong> Slot(ReadOnlySpan<ulong> values, int index) => values.Slice(index * Words, Words);
    internal static Span<ulong> Slot(Span<ulong> values, int index, int words) => values.Slice(index * words, words);
    internal static ReadOnlySpan<ulong> Slot(ReadOnlySpan<ulong> values, int index, int words) => values.Slice(index * words, words);
    internal static void Import(Signed320 value, Span<ulong> result) => Import(Signed576.ExtendValue(value), result);
    internal static void Import(Signed576 value, Span<ulong> result) => WideArithmetic.GetMagnitude(value, result);
    internal static void Import(Signed832 value, Span<ulong> result) => WideArithmetic.GetMagnitude(value, result);
    internal static void DifferenceOfProducts(Signed832 a, Signed832 b, Signed832 c, Signed832 d,
        Span<ulong> result, out int sign)
    {
        Span<ulong> left = stackalloc ulong[13], right = stackalloc ulong[13], product = stackalloc ulong[result.Length];
        WideArithmetic.GetMagnitude(a, left); WideArithmetic.GetMagnitude(b, right);
        WideArithmetic.MultiplyMagnitudes(left, right, result);
        sign = a.Sign * b.Sign;
        WideArithmetic.GetMagnitude(c, left); WideArithmetic.GetMagnitude(d, right);
        WideArithmetic.MultiplyMagnitudes(left, right, product);
        Add(product, -c.Sign * d.Sign, result, ref sign);
    }
    internal static Signed320 Component(in WideAxis3 value, int index) =>
        index == 0 ? value.X : index == 1 ? value.Y : value.Z;
    internal static Signed576 Component(in WideDirection3d value, int index) =>
        index == 0 ? value.X : index == 1 ? value.Y : value.Z;
    private static WideAxis3 Axis(Signed192 x, Signed192 y, Signed192 z) => new(
        Signed320.ExtendValue(x), Signed320.ExtendValue(y), Signed320.ExtendValue(z));
}
