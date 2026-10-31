//=======================================================================
// FiniteSlabRaySelection.NativePrism.cs
//=======================================================================
// MIT License, Copyright (c) 2024-present David Oravsky (mrdav30)
// See LICENSE file in the project root for full license information.
//=======================================================================

using System;

namespace FixedMathSharp.Geometry;

/// <content>Retains native point fields from the same quadratic active-set root used for selection.</content>
internal ref partial struct FiniteSlabRaySelection
{
    internal readonly int CompareNativeDepth(in CurvedPrismContactGeometry geometry, in FiniteSlabRayFrame frame,
        scoped CurvedPrismContactValue native)
    {
        if (!native.HasValue) return -1;
        int axis = !frame.Plane.Normal.X.IsZero ? 0 : !frame.Plane.Normal.Y.IsZero ? 1 : 2;
        int words = DepthCoefficientWords() + 6;
        Span<ulong> values = stackalloc ulong[4 * words], root = stackalloc ulong[2 * words + 1];
        Span<int> signs = stackalloc int[4];
        var n = ContactQuadratic.At(values, signs, 0, words); var d = ContactQuadratic.At(values, signs, 1, words);
        GetPhysicalDepthRatio(frame, axis, root, n, d);
        // The ratio is the selected coordinate displacement. Its component
        // metric restores the full physical length, without rounding t.
        return native.CompareRootRatio(geometry, n, d, root, frame.Plane.NormalSquared,
            Signed320.NarrowValue(axis == 0 ? frame.Plane.Normal.X : axis == 1 ? frame.Plane.Normal.Y : frame.Plane.Normal.Z));
    }

    // The defining equations have at most three coefficients. Degree-nine
    // polynomials elsewhere only test admission; they never define this root.
    private readonly void GetRootParameter(scoped Span<ulong> radical,
        scoped ContactQuadratic numerator, scoped ContactQuadratic denominator)
    {
        radical.Clear();
        int words = numerator.FieldWords;
        Span<ulong> values = stackalloc ulong[6 * words]; Span<int> signs = stackalloc int[6];
        var a = ContactQuadratic.At(values, signs, 0, words);
        var b = ContactQuadratic.At(values, signs, 1, words);
        var c = ContactQuadratic.At(values, signs, 2, words);
        CopyDepthCoefficient(Polynomial, 2, a); a.Add(a);
        CopyDepthCoefficient(Polynomial, 1, b);
        CopyDepthCoefficient(Polynomial, 0, c); c.Add(c);
        int branch = 1;
        if (a.Signs[0] != 0)
        {
            Span<ulong> cell = stackalloc ulong[CellWords]; FiniteAxisValueRoot root = Restore(cell);
            Span<ulong> derivativeValues = stackalloc ulong[2 * Words]; Span<sbyte> derivativeSigns = stackalloc sbyte[2];
            FiniteSlabPolynomial derivative = new(derivativeValues, derivativeSigns);
            derivative.Clear(); derivative.Add(Polynomial.Slice(1, 1)); derivative.Add(Polynomial.Slice(2, 1), 2, 1);
            int derivativeSign = FiniteSlabPolynomial.Sign(ref root, derivative);
            if (derivativeSign != 0) branch = derivativeSign * a.Signs[0];
        }
        bool represented = ContactQuadratic.TryGetParameter(a, b, c, branch, radical, numerator, denominator);
        System.Diagnostics.Debug.Assert(represented);
    }

    internal readonly void GetNativeLinePointFields(in FiniteSlabRayFrame frame, in WideAxis3 a,
        in WideAxis3 e, Span<ulong> source, Span<int> sourceSigns,
        Span<ulong> exit, Span<int> exitSigns, scoped ContactQuadratic denominator, scoped Span<ulong> radical)
    {
        int words = denominator.FieldWords;
        Span<ulong> values = stackalloc ulong[14 * words]; Span<int> signs = stackalloc int[14];
        var uN = ContactQuadratic.At(values, signs, 0, words); var uD = ContactQuadratic.At(values, signs, 1, words);
        var l = ContactQuadratic.At(values, signs, 2, words);
        var t = ContactQuadratic.At(values, signs, 3, words); var coefficient = ContactQuadratic.At(values, signs, 4, words);
        var term = ContactQuadratic.At(values, signs, 5, words); var product = ContactQuadratic.At(values, signs, 6, words);
        GetRootParameter(radical, uN, uD);
        Evaluate(Lambda, uN, uD, radical, l, coefficient, product);
        Evaluate(T, uN, uD, radical, t, coefficient, product);
        Evaluate(D, uN, uD, radical, denominator, coefficient, product);
        for (int axis = 0; axis < 3; axis++)
        {
            var point = ContactQuadratic.At(source, sourceSigns, axis, words);
            ContactQuadratic.Scale(denominator, Signed576.ExtendValue(Component(a, axis)), point);
            ContactQuadratic.Scale(l, Signed576.ExtendValue(Component(e, axis)), term); point.Add(term);
            var moved = ContactQuadratic.At(exit, exitSigns, axis, words); point.CopyTo(moved);
            ContactQuadratic.Scale(t, Component(frame.RayNormal, axis), term); moved.Add(term);
        }
        if (denominator.Sign(radical) < 0)
        {
            denominator.MultiplySign(-1);
            for (int axis = 0; axis < 3; axis++)
            {
                ContactQuadratic.At(source, sourceSigns, axis, words).MultiplySign(-1);
                ContactQuadratic.At(exit, exitSigns, axis, words).MultiplySign(-1);
            }
        }
    }

    private static void Evaluate(scoped FiniteSlabPolynomial polynomial, scoped ContactQuadratic uN,
        scoped ContactQuadratic uD, ReadOnlySpan<ulong> radical, scoped ContactQuadratic result,
        scoped ContactQuadratic coefficient, scoped ContactQuadratic term)
    {
        CopyDepthCoefficient(polynomial, 0, coefficient);
        ContactQuadratic.Multiply(coefficient, uD, radical, result);
        System.Diagnostics.Debug.Assert(polynomial.Count == 2);
        CopyDepthCoefficient(polynomial, 1, coefficient);
        ContactQuadratic.Multiply(coefficient, uN, radical, term); result.Add(term);
    }
}
