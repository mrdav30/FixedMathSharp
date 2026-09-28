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
        for (int component = 0; component < 3; component++)
        {
            WideAxis3 row = component == 0 ? Axis(basis.Xx, basis.Yx, basis.Zx)
                : component == 1 ? Axis(basis.Xy, basis.Yy, basis.Zy) : Axis(basis.Xz, basis.Yz, basis.Zz);
            Dot(row, direction, directionSigns, Slot(values, component), out int sign);
            signs[component] = orientation * sign;
        }
    }

    internal static void Dot(WideAxis3 axis, ReadOnlySpan<ulong> direction, ReadOnlySpan<int> signs,
        Span<ulong> result, out int sign)
    {
        Span<ulong> component = stackalloc ulong[Words];
        Span<ulong> product = stackalloc ulong[Words];
        result.Clear(); sign = 0;
        for (int index = 0; index < 3; index++)
        {
            Signed320 value = Component(axis, index);
            Import(value, component);
            WideArithmetic.MultiplyMagnitudes(component, Slot(direction, index), product);
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
        Span<ulong> product = stackalloc ulong[Words];
        result.Clear();
        for (int index = 0; index < 3; index++)
        {
            ReadOnlySpan<ulong> component = Slot(direction, index);
            WideArithmetic.MultiplyMagnitudes(component, component, product);
            WideArithmetic.AddMagnitudeInto(product, result);
        }
    }

    internal static void WriteDirection(WideAxis3 axis, Span<ulong> direction, Span<int> signs)
    {
        for (int index = 0; index < 3; index++)
        {
            Signed320 value = Component(axis, index);
            Import(value, Slot(direction, index)); signs[index] = value.Sign;
        }
    }

    internal static void Add(ReadOnlySpan<ulong> value, int sign, Span<ulong> destination, ref int destinationSign) =>
        WideArithmetic.AddShiftedSignedMagnitude(value, sign, 0, destination, ref destinationSign);
    internal static bool IsZero(ReadOnlySpan<ulong> value) => WideArithmetic.GetActiveMagnitudeLength(value) == 0;
    internal static Span<ulong> Slot(Span<ulong> values, int index) => values.Slice(index * Words, Words);
    internal static ReadOnlySpan<ulong> Slot(ReadOnlySpan<ulong> values, int index) => values.Slice(index * Words, Words);
    internal static void Import(Signed320 value, Span<ulong> result) => Import(Signed576.ExtendValue(value), result);
    internal static void Import(Signed576 value, Span<ulong> result)
    {
        result.Clear(); WideArithmetic.GetMagnitude(value, result[..9]);
    }
    internal static Signed320 Component(in WideAxis3 value, int index) =>
        index == 0 ? value.X : index == 1 ? value.Y : value.Z;
    private static WideAxis3 Axis(Signed192 x, Signed192 y, Signed192 z) => new(
        Signed320.ExtendValue(x), Signed320.ExtendValue(y), Signed320.ExtendValue(z));
}
