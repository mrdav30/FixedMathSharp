//=======================================================================
// MIT License, Copyright (c) 2024-present David Oravsky (mrdav30)
// See LICENSE file in the project root for full license information.
//=======================================================================
using System;
using static FixedMathSharp.Geometry.CylinderContactAlgebra;
using static FixedMathSharp.Geometry.ContactQuadratic;

namespace FixedMathSharp.Geometry;

/// <summary>The exact fixed-latitude generator normal circle and its triangle fan.</summary>
internal static class TriangleConeGeneratorFeatures
{
    internal static void KeepAll(in TriangleCircularGeometry geometry, Fixed64 height, Fixed64 radius,
        ref TriangleConeContactSelection selection)
    {
        Span<ulong> values = stackalloc ulong[ConvexContactCandidate.Slots * Words];
        Span<int> signs = stackalloc int[ConvexContactCandidate.Slots];
        Span<ulong> work = stackalloc ulong[6 * Words];
        Span<ulong> hSquared = Slot(work, 0), rSquared = Slot(work, 1), length = Slot(work, 2);
        Span<ulong> metric = Slot(work, 3), product = Slot(work, 4);
        Signed192 h = Signed192.Raw(height), r = Signed192.Raw(radius);
        Import(WideArithmetic.MultiplySigned192(h, h), hSquared);
        Import(WideArithmetic.MultiplySigned192(r, r), rSquared);
        WideArithmetic.AddEqualMagnitudes(hSquared, rSquared, length);
        values.Clear(); signs.Clear();
        Write(values, signs, 0, Signed576.ExtendValue(Signed320.ExtendValue(h)));
        Write(values, signs, 1, Signed576.ExtendValue(Signed320.ExtendValue(WideArithmetic.SubtractSigned192(default, r))));
        // A zero-radial vertex can own the whole constant support circle.
        Keep(geometry, values, signs, length, ref selection);
        for (int vertex = 0; vertex < 3 && !selection.Separated; vertex++)
        {
            WideAxis3 p = geometry.Vertex(vertex);
            Signed576 radial = TriangleRimContactAlgebra.RadialDot(p, p);
            if (radial.Sign != 0)
            {
                for (int orientation = -1; orientation <= 1; orientation += 2)
                {
                    values.Clear(); signs.Clear();
                    Write(values, signs, 0, WideArithmetic.MultiplySigned320(p.X, h), orientation);
                    Write(values, signs, 2, WideArithmetic.MultiplySigned320(p.Z, h), orientation);
                    Write(values, signs, 4, Signed576.ExtendValue(Signed320.ExtendValue(r)), -1);
                    Import(radial, Slot(values, 6));
                    WideArithmetic.MultiplyMagnitudes(Slot(values, 6), length, metric);
                    Keep(geometry, values, signs, metric, ref selection);
                }
            }
            WideAxis3 e = geometry.Edge(vertex);
            Signed576 eSquared = TriangleRimContactAlgebra.RadialDot(e, e);
            if (eSquared.Sign == 0)
                continue;
            values.Clear(); signs.Clear();
            Import(eSquared, metric);
            WideArithmetic.MultiplyMagnitudes(metric, hSquared, Slot(values, 6));
            Import(WideArithmetic.MultiplySigned320(e.Y, e.Y), product);
            WideArithmetic.MultiplyMagnitudes(product, rSquared, Slot(work, 5));
            int discriminantSign = 1;
            Add(Slot(work, 5), -1, Slot(values, 6), ref discriminantSign);
            if (discriminantSign < 0)
                continue;
            Write(values, signs, 0, WideArithmetic.MultiplySigned576(WideArithmetic.MultiplySigned320(e.Y, e.X), radius.m_rawValue));
            Write(values, signs, 1, WideArithmetic.MultiplySigned576(eSquared, radius.m_rawValue), -1);
            Write(values, signs, 2, WideArithmetic.MultiplySigned576(WideArithmetic.MultiplySigned320(e.Y, e.Z), radius.m_rawValue));
            Write(values, signs, 3, Signed576.ExtendValue(e.Z), -1);
            Write(values, signs, 5, Signed576.ExtendValue(e.X));
            WideArithmetic.MultiplyMagnitudes(metric, metric, product);
            WideArithmetic.MultiplyMagnitudes(product, length, metric);
            Keep(geometry, values, signs, metric, ref selection);
            if (discriminantSign != 0 && !selection.Separated)
            {
                // Keep transforms the candidate normal only when selected.
                // Rebuild the local rational terms before the other branch.
                Write(values, signs, 0, WideArithmetic.MultiplySigned576(WideArithmetic.MultiplySigned320(e.Y, e.X), radius.m_rawValue));
                Write(values, signs, 1, WideArithmetic.MultiplySigned576(eSquared, radius.m_rawValue), -1);
                Write(values, signs, 2, WideArithmetic.MultiplySigned576(WideArithmetic.MultiplySigned320(e.Y, e.Z), radius.m_rawValue));
                Write(values, signs, 3, Signed576.ExtendValue(e.Z));
                Slot(values, 4).Clear(); signs[4] = 0;
                Write(values, signs, 5, Signed576.ExtendValue(e.X), -1);
                Keep(geometry, values, signs, metric, ref selection);
            }
        }
    }

    internal static void Keep(in TriangleCircularGeometry geometry, scoped Span<ulong> values, scoped Span<int> signs,
        scoped ReadOnlySpan<ulong> metric, ref TriangleConeContactSelection selection)
    {
        Span<ulong> work = stackalloc ulong[8 * Words];
        Span<int> workSigns = stackalloc int[6];
        ContactQuadratic maximum = At(work, workSigns, 0), current = At(work, workSigns, 1), difference = At(work, workSigns, 2);
        ReadOnlySpan<ulong> root = Slot(values, 6);
        Dot(geometry.A, values, signs, maximum);
        int mask = 1;
        for (int vertex = 1; vertex < 3; vertex++)
        {
            Dot(geometry.Vertex(vertex), values, signs, current);
            current.CopyTo(difference); difference.Add(maximum, -1);
            int comparison = difference.Sign(root);
            if (comparison > 0) { current.CopyTo(maximum); mask = 1 << vertex; }
            else if (comparison == 0) mask |= 1 << vertex;
        }
        Component(values, signs, 1, current);
        Scale(current, Signed576.ExtendValue(geometry.HalfHeight), difference);
        maximum.Add(difference, -1);
        Import(Signed320.ExtendValue(geometry.RawScale), Slot(work, 6));
        WideArithmetic.MultiplyMagnitudes(Slot(work, 6), Slot(work, 6), Slot(work, 7));
        WideArithmetic.MultiplyMagnitudes(Slot(work, 7), metric, Slot(work, 6));
        int gapSign = BuildQuadraticCandidate(maximum, root, Slot(work, 6), values, signs);
        selection.Keep(values, signs, gapSign, mask, TriangleConeContactSelection.Generator);
    }

    internal static void Dot(WideAxis3 axis, ReadOnlySpan<ulong> normal, ReadOnlySpan<int> signs, ContactQuadratic result)
    {
        CylinderContactAlgebra.Dot(axis, normal[..(3 * Words)], signs[..3], result.Rational, out int rationalSign);
        CylinderContactAlgebra.Dot(axis, normal.Slice(3 * Words, 3 * Words), signs.Slice(3, 3), result.Radical, out int radicalSign);
        result.Signs[0] = rationalSign; result.Signs[1] = radicalSign;
    }

    internal static void Component(ReadOnlySpan<ulong> normal, ReadOnlySpan<int> signs, int component, ContactQuadratic result)
    {
        Slot(normal, component).CopyTo(result.Rational); Slot(normal, component + 3).CopyTo(result.Radical);
        result.Signs[0] = signs[component]; result.Signs[1] = signs[component + 3];
    }

    private static void Write(Span<ulong> values, Span<int> signs, int slot, Signed576 value, int orientation = 1)
    {
        Import(value, Slot(values, slot)); signs[slot] = orientation * value.Sign;
    }
}
