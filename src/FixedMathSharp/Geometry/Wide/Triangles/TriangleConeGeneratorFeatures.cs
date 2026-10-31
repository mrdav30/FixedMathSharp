//=======================================================================
// TriangleConeGeneratorFeatures.cs
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
        Span<ulong> work = stackalloc ulong[7 * Words];
        Span<ulong> hSquared = Slot(work, 0), rSquared = Slot(work, 1), length = Slot(work, 2);
        Span<ulong> metric = Slot(work, 3), product = Slot(work, 4);
        Signed192 h = Signed192.Raw(height), r = Signed192.Raw(radius);
        Import(WideArithmetic.MultiplySigned192(h, h), hSquared);
        Import(WideArithmetic.MultiplySigned192(r, r), rSquared);
        WideArithmetic.AddEqualMagnitudes(hSquared, rSquared, length);
        // All generator directions share the raw-coordinate scale. Include
        // its square once in the length factor used by every gap denominator.
        Import(Signed576.ExtendValue(geometry.RawScale), product);
        WideArithmetic.MultiplyMagnitudes(product, product, Slot(work, 6));
        WideArithmetic.MultiplyMagnitudes(length, Slot(work, 6), product);
        product.CopyTo(length);
        values.Clear(); signs.Clear();
        Write(values, signs, 0, Signed832.ExtendValue(Signed576.ExtendValue(h)));
        Write(values, signs, 1, Signed832.ExtendValue(Signed576.ExtendValue(WideArithmetic.SubtractSigned192(default, r))));
        // A zero-radial vertex can own the whole constant support circle.
        Keep(geometry, values, signs, length, ref selection);
        for (int vertex = 0; vertex < 3 && !selection.Separated; vertex++)
        {
            WideDirection3d p = geometry.Vertex(vertex);
            Signed832 radial = CircularRimContactAlgebra.RadialDot(p, p);
            if (radial.Sign != 0)
            {
                values.Clear(); signs.Clear();
                Write(values, signs, 0, Signed832.ExtendValue(WideArithmetic.MultiplySigned576(p.X, h)), -1);
                Write(values, signs, 2, Signed832.ExtendValue(WideArithmetic.MultiplySigned576(p.Z, h)), -1);
                Write(values, signs, 4, Signed832.ExtendValue(Signed576.ExtendValue(r)), -1);
                Import(radial, Slot(values, 6));
                WideArithmetic.MultiplyMagnitudes(Slot(values, 6), length, metric);
                for (int orientation = -1; orientation <= 1; orientation += 2)
                {
                    Keep(geometry, values, signs, metric, ref selection);
                    signs[0] = -signs[0]; signs[2] = -signs[2];
                }
            }
            WideDirection3d e = geometry.Edge(vertex);
            Signed832 eSquared = CircularRimContactAlgebra.RadialDot(e, e);
            if (eSquared.Sign == 0)
                continue;
            values.Clear(); signs.Clear();
            Import(eSquared, metric);
            WideArithmetic.MultiplyMagnitudes(metric, hSquared, Slot(values, 6));
            Import(WideArithmetic.MultiplySigned576(e.Y, e.Y), product);
            WideArithmetic.MultiplyMagnitudes(product, rSquared, Slot(work, 5));
            int discriminantSign = 1;
            Add(Slot(work, 5), -1, Slot(values, 6), ref discriminantSign);
            if (discriminantSign < 0)
                continue;
            Write(values, signs, 0, WideArithmetic.MultiplySigned832(WideArithmetic.MultiplySigned576(e.Y, e.X), Signed192.Raw(radius)));
            Write(values, signs, 1, WideArithmetic.MultiplySigned832(eSquared, Signed192.Raw(radius)), -1);
            Write(values, signs, 2, WideArithmetic.MultiplySigned832(WideArithmetic.MultiplySigned576(e.Y, e.Z), Signed192.Raw(radius)));
            Write(values, signs, 3, Signed832.ExtendValue(e.Z), -1);
            Write(values, signs, 5, Signed832.ExtendValue(e.X));
            WideArithmetic.MultiplyMagnitudes(metric, metric, product);
            WideArithmetic.MultiplyMagnitudes(product, length, metric);
            Keep(geometry, values, signs, metric, ref selection);
            if (discriminantSign != 0 && !selection.Separated)
            {
                // Selection preserves the local input: the other quadratic
                // branch changes only the two radial radical signs.
                signs[3] = -signs[3]; signs[5] = -signs[5];
                Keep(geometry, values, signs, metric, ref selection);
            }
        }
    }

    internal static void Keep(in TriangleCircularGeometry geometry, scoped Span<ulong> values, scoped Span<int> signs,
        scoped ReadOnlySpan<ulong> denominator, ref TriangleConeContactSelection selection)
    {
        Span<ulong> work = stackalloc ulong[6 * Words];
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
        Scale(current, Signed832.ExtendValue(geometry.HalfHeight), difference);
        maximum.Add(difference, -1);
        int gapSign = BuildQuadraticCandidate(maximum, root, denominator, values, signs);
        selection.Keep(values, signs, gapSign, mask, TriangleConeContactSelection.Generator);
    }

    internal static void Dot(in WideDirection3d axis, ReadOnlySpan<ulong> normal, ReadOnlySpan<int> signs, ContactQuadratic result)
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

    private static void Write(Span<ulong> values, Span<int> signs, int slot, Signed832 value, int orientation = 1)
    {
        Import(value, Slot(values, slot)); signs[slot] = orientation * value.Sign;
    }
}
