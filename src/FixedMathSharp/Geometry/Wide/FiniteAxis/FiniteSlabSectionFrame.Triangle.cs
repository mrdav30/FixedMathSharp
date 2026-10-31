//=======================================================================
// FiniteSlabSectionFrame.Triangle.cs
//=======================================================================
// MIT License, Copyright (c) 2024-present David Oravsky (mrdav30)
// See LICENSE file in the project root for full license information.
//=======================================================================

using System;

namespace FixedMathSharp.Geometry;

/// <content>Closed triangle admission using the shared finite section and one exact plane witness.</content>
internal readonly partial struct FiniteSlabSectionFrame
{
    /// <summary>Tests closed triangle overlap without producing contact depths or rounded anchors.</summary>
    internal bool IntersectsTriangle(FixedTriangle triangle, ReadOnlySpan<Vector2d> prismOffsets = default)
    {
        for (int edge = 0; edge < 3; edge++)
            if (IntersectsSegment(triangle.GetEdge(edge), prismOffsets)) return true;
        CapsulePlaneRayFrame plane = _radial.ShapeFrame.HasExactScale
            ? new(triangle, SourceFrame, _radial) : new(triangle, _radial);
        if (plane.NormalSquared.IsZero) return false;
        // The slab and its plane section are convex and connected. With no
        // closed triangle edge crossing, any overlap contains the whole
        // section inside the triangle. One admitted section point decides it.
        const int words = 22;
        Span<ulong> values = stackalloc ulong[24 * words]; Span<int> signs = stackalloc int[24];
        Span<ulong> root = stackalloc ulong[_radial.ShapeFrame.HasExactScale ? 13 : 9]; root.Clear();
        ContactQuadratic x = ContactQuadratic.At(values, signs, 0, words),
            y = ContactQuadratic.At(values, signs, 1, words), z = ContactQuadratic.At(values, signs, 2, words),
            d = ContactQuadratic.At(values, signs, 3, words), term = ContactQuadratic.At(values, signs, 4, words),
            work = ContactQuadratic.At(values, signs, 5, words), metric = ContactQuadratic.At(values, signs, 6, words),
            radius = ContactQuadratic.At(values, signs, 7, words), gap = ContactQuadratic.At(values, signs, 8, words);
        x.Clear(); y.Clear(); z.Clear(); d.Clear();
        bool admitted = prismOffsets.IsEmpty
            ? TryGetCurvedPlanePoint(plane, x, y, z, d, term, work, metric, radius, gap, root)
            : TryGetPrismPlanePoint(plane, prismOffsets, x, y, z, d, term, work);
        return admitted && plane.ContainsTrianglePoint(triangle, x, y, z, d, root);
    }

    private bool TryGetCurvedPlanePoint(in CapsulePlaneRayFrame plane, ContactQuadratic x,
        ContactQuadratic y, ContactQuadratic z, ContactQuadratic d, ContactQuadratic term,
        ContactQuadratic work, ContactQuadratic metric, ContactQuadratic radius, ContactQuadratic gap, Span<ulong> root)
    {
        WideDirection3d n = plane.Normal;
        Signed832 radialMetric = WideArithmetic.AddSigned832(Product(n.X, n.X),
            Product(n.Z, n.Z));
        if (radialMetric.IsZero)
        {
            Signed832 extent = Product(Absolute(n.Y), _cap);
            if (WideArithmetic.SubtractSigned832(Absolute(plane.PlaneConstant), extent).Sign > 0) return false;
            y.Set(plane.PlaneConstant); y.MultiplySign(n.Y.Sign); d.Set(Signed832.ExtendValue(Absolute(n.Y)));
            return true;
        }
        if (_radial.ShapeFrame.HasExactScale) WideArithmetic.GetMagnitude(radialMetric, root);
        else
        {
            // The rigid normal retains the original 576-bit radial metric.
            // Its compact root bank intentionally has only nine limbs.
            Signed576 compactMetric = Signed576.NarrowValue(radialMetric);
            WideArithmetic.GetMagnitude(compactMetric, root);
        }
        Signed320 r = Signed320.NarrowValue(WideArithmetic.MultiplySigned320(_radial.ShapeFrame.CoordinateDenominator, _radial.ShapeFrame.Radius));
        Signed832 axialSupport = WideArithmetic.AddSigned832(Product(Absolute(n.Y), _cap),
            Product(Absolute(n.Z), _radial.ShapeFrame.Cap));
        // For symmetric S, q=(k/support(N))*supportPoint(N) is admitted.
        // Clear the common sqrt(radialMetric) denominator before composing
        // its coordinates: qD=R*radialMetric+axialSupport*sqrt(radialMetric).
        metric.Set(radialMetric); radius.Set(Signed832.ExtendValue(r));
        ContactQuadratic.Multiply(metric, radius, root, d);
        term.Set(axialSupport); term.Rational.CopyTo(d.Radical); d.Signs[1] = axialSupport.Sign;
        gap.Clear(); term.Set(Absolute(plane.PlaneConstant)); term.Rational.CopyTo(gap.Radical);
        gap.Signs[1] = term.Signs[0]; gap.Add(d, -1);
        if (gap.Sign(root) > 0) return false;
        if (d.Sign(root) == 0)
        {
            // A zero-width core orthogonal to this plane has only k=0.
            // Its origin is a valid section point, including a line family.
            d.Set(Signed832.ExtendValue(Signed320.One));
            return true;
        }
        term.Set(Product(r, n.X));
        ContactQuadratic.Scale(term, plane.PlaneConstant, x);
        term.Set(Product(r, n.Z));
        ContactQuadratic.Scale(term, plane.PlaneConstant, z);
        term.Set(Signed832.ExtendValue(Signed(_cap, n.Y.Sign)));
        ContactQuadratic.Scale(term, plane.PlaneConstant, work); work.Rational.CopyTo(y.Radical); y.Signs[1] = work.Signs[0];
        term.Set(Signed832.ExtendValue(Signed(_radial.ShapeFrame.Cap, n.Z.Sign)));
        ContactQuadratic.Scale(term, plane.PlaneConstant, work); work.Rational.CopyTo(z.Radical); z.Signs[1] = work.Signs[0];
        // Coordinate coefficients <930 bits and denominator <730 bits fit
        // 22 limbs; ContainsTrianglePoint derives its wider wall-product bank.
        return true;
    }

    private bool TryGetPrismPlanePoint(in CapsulePlaneRayFrame plane, ReadOnlySpan<Vector2d> offsets,
        ContactQuadratic x, ContactQuadratic y, ContactQuadratic z, ContactQuadratic d,
        ContactQuadratic term, ContactQuadratic work)
    {
        _ = GetPrismWinding(offsets);
        Signed320 scale = WideArithmetic.AddSigned320(_radial.ShapeFrame.CoordinateDenominator, _radial.ShapeFrame.CoordinateDenominator);
        WideAxis3 low = default, high = default;
        Signed832 lowValue = default, highValue = default;
        for (int index = 0; index < offsets.Length; index++)
        {
            WideAxis3 point = new(Signed320.NarrowValue(WideArithmetic.MultiplySigned320(scale, Signed192.Raw(offsets[index].X))), default,
                Signed320.NarrowValue(WideArithmetic.MultiplySigned320(scale, Signed192.Raw(offsets[index].Y))));
            Signed832 value = WideDirection3d.Dot(plane.Normal, point);
            if (index == 0 || WideArithmetic.SubtractSigned832(value, lowValue).Sign < 0) { low = point; lowValue = value; }
            if (index == 0 || WideArithmetic.SubtractSigned832(value, highValue).Sign > 0) { high = point; highValue = value; }
        }
        Signed320 cap = Signed(_cap, plane.Normal.Y.Sign);
        low = new(low.X, WideArithmetic.Negate(cap), low.Z); high = new(high.X, cap, high.Z);
        lowValue = WideDirection3d.Dot(plane.Normal, low); highValue = WideDirection3d.Dot(plane.Normal, high);
        if (WideArithmetic.SubtractSigned832(plane.PlaneConstant, lowValue).Sign < 0
            || WideArithmetic.SubtractSigned832(plane.PlaneConstant, highValue).Sign > 0) return false;
        Signed832 difference = WideArithmetic.SubtractSigned832(highValue, lowValue);
        // A valid convex footprint and positive thickness span all three
        // dimensions, so a nonzero plane normal has positive support width.
        System.Diagnostics.Debug.Assert(difference.Sign > 0);
        Signed832 fraction = WideArithmetic.SubtractSigned832(plane.PlaneConstant, lowValue);
        d.Set(difference);
        for (int axis = 0; axis < 3; axis++)
        {
            ContactQuadratic coordinate = axis == 0 ? x : axis == 1 ? y : z;
            Signed320 first = Component(low, axis), last = Component(high, axis);
            term.Set(Signed832.ExtendValue(first)); ContactQuadratic.Scale(term, difference, coordinate);
            term.Set(Signed832.ExtendValue(WideArithmetic.SubtractSigned320(last, first)));
            ContactQuadratic.Scale(term, fraction, work); coordinate.Add(work);
        }
        return true;
    }

    private static Signed832 Product(Signed576 first, Signed576 second) =>
        WideArithmetic.MultiplySigned576ToSigned832(first, second);
    private static Signed832 Product(Signed576 first, Signed320 second) =>
        WideArithmetic.MultiplySigned576ToSigned832(first, second);
    private static Signed832 Product(Signed320 first, Signed576 second) =>
        WideArithmetic.MultiplySigned576ToSigned832(second, first);
    private static Signed576 Absolute(Signed576 value) => value.Sign < 0 ? WideArithmetic.SubtractSigned576(default, value) : value;
    private static Signed832 Absolute(Signed832 value) => value.Sign < 0 ? WideArithmetic.SubtractSigned832(default, value) : value;
    private static Signed320 Signed(Signed320 value, int sign) => sign == 0 ? default : sign < 0 ? WideArithmetic.Negate(value) : value;
    private static Signed320 Component(WideAxis3 value, int axis) => axis == 0 ? value.X : axis == 1 ? value.Y : value.Z;
}
