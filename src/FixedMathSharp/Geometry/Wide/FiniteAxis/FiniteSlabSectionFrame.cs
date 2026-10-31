//=======================================================================
// FiniteSlabSectionFrame.cs
//=======================================================================
// MIT License, Copyright (c) 2024-present David Oravsky (mrdav30)
// See LICENSE file in the project root for full license information.
//=======================================================================

using System;

namespace FixedMathSharp.Geometry;

/// <summary>Exact closed admission for a flat-capped circle or Z-axis stadium extrusion.</summary>
/// <content>Owns finite point, segment, and triangle section admission in one exact rigid frame.</content>
internal readonly partial struct FiniteSlabSectionFrame
{
    // The planar capsule is the Y-core capsule kernel evaluated at (X,Z,0).
    // Full endpoint disks and the central rectangle have exactly the stadium
    // union; extra hemisphere clipping would only add redundant constraints.
    private readonly CapsuleFiniteSectionFrame _radial;
    private readonly Signed320 _cap;
    private ScaledRigidFrame3d SourceFrame => _radial.ShapeFrame.SourceFrame;
    internal CapsuleFiniteSectionFrame RadialFrame => _radial;
    internal Signed320 Cap => _cap;
    internal WideAxis3 Transform(Vector3d point) => _radial.Transform(point);
    internal int PointWords => _radial.ShapeFrame.HasExactScale ? FiniteSectionPoint.AffineWords : FiniteSectionPoint.Words;
    internal int RootWords => _radial.ShapeFrame.HasExactScale ? FiniteSectionPoint.AffineRootWords : FiniteSectionPoint.RootWords;

    /// <summary>Prepares an exact primitive-local circle extrusion for pretransformed source coordinates.</summary>
    internal FiniteSlabSectionFrame(Fixed64 height, Fixed64 radius, Signed320 coordinateDenominator)
    {
        _radial = new CapsuleFiniteSectionFrame(Fixed64.Zero, radius, coordinateDenominator);
        _cap = Signed320.NarrowValue(WideArithmetic.MultiplySigned320(coordinateDenominator,
            Signed320.ExtendValue(Signed192.Raw(height))));
    }

    internal FiniteSlabSectionFrame(Vector3d origin, FixedQuaternion rotation,
        Vector3d center, FixedQuaternion slabRotation, Fixed64 length, Fixed64 radius, Fixed64 halfThickness, bool fullHeight = false)
    {
        if (halfThickness <= Fixed64.Zero) throw new ArgumentOutOfRangeException(nameof(halfThickness));
        _radial = new CapsuleFiniteSectionFrame(origin, rotation, center, slabRotation, length, radius);
        // Transformed coordinates carry 2D, so an authored full height
        // supplies the exact cap directly, including an odd raw half-height.
        Signed192 height = fullHeight ? Signed192.Raw(halfThickness)
            : WideArithmetic.AddSigned192(Signed192.Raw(halfThickness), Signed192.Raw(halfThickness));
        _cap = WideArithmetic.MultiplySigned192(height, _radial.ShapeFrame.Denominator);
    }

    internal FiniteSlabSectionFrame(in ScaledRigidFrame3d sourceFrame,
        Vector3d center, FixedQuaternion slabRotation, Fixed64 length, Fixed64 radius, Fixed64 halfThickness, bool fullHeight = false)
    {
        if (sourceFrame.IsRigid)
        {
            this = new FiniteSlabSectionFrame(sourceFrame.RigidOrigin, sourceFrame.Rotation,
                center, slabRotation, length, radius, halfThickness, fullHeight);
            return;
        }
        if (halfThickness <= Fixed64.Zero || halfThickness > (Fixed64)(fullHeight ? 8192 : 4096))
            throw new ArgumentOutOfRangeException(nameof(halfThickness));
        _radial = new CapsuleFiniteSectionFrame(sourceFrame, center, slabRotation, length, radius);
        Signed192 height = fullHeight ? Signed192.Raw(halfThickness)
            : WideArithmetic.AddSigned192(Signed192.Raw(halfThickness), Signed192.Raw(halfThickness));
        _cap = Signed320.NarrowValue(WideArithmetic.MultiplySigned320(_radial.ShapeFrame.CoordinateDenominator, height));
    }

    internal bool ContainsPoint(Vector3d point)
    {
        WideAxis3 p = _radial.Transform(point);
        return WideArithmetic.AddSigned320(p.Y, _cap).Sign >= 0
            && WideArithmetic.SubtractSigned320(p.Y, _cap).Sign <= 0
            && _radial.ContainsTransformedPoint(Radial(p));
    }

    internal bool ContainsPoint(Vector3d point, ReadOnlySpan<Vector2d> prismOffsets)
    {
        if (prismOffsets.IsEmpty) return ContainsPoint(point);
        WideAxis3 p = Transform(point);
        if (WideArithmetic.AddSigned320(p.Y, _cap).Sign < 0
            || WideArithmetic.SubtractSigned320(p.Y, _cap).Sign > 0) return false;
        int winding = GetPrismWinding(prismOffsets);
        for (int edge = 0; edge < prismOffsets.Length; edge++)
            if (PrismWall(p, prismOffsets[edge], prismOffsets[(edge + 1) % prismOffsets.Length]).Sign * winding < 0)
                return false;
        return true;
    }

    internal bool IntersectsSegment(FixedSegment segment, ReadOnlySpan<Vector2d> prismOffsets)
    {
        if (prismOffsets.IsEmpty) return IntersectsSegment(segment);
        Span<ulong> values = stackalloc ulong[4 * PointWords]; Span<int> signs = stackalloc int[4];
        Span<ulong> root = stackalloc ulong[RootWords]; root.Clear();
        return TryGetPrismSegmentParameter(segment, prismOffsets,
            ContactQuadratic.At(values, signs, 0, PointWords),
            ContactQuadratic.At(values, signs, 1, PointWords), root);
    }

    /// <summary>Tests an exact pretransformed subsegment in the existing radial and height intervals.</summary>
    internal bool IntersectsTransformedSegment(WideAxis3 start, WideAxis3 end, in RationalSegmentInterval clip)
    {
        WideAxis3 delta = Difference(end, start);
        for (int primitive = 0; primitive < (_radial.Length == Fixed64.Zero ? 1 : 3); primitive++)
            if (TryGetPrimitive(start, delta, primitive, out Signed832 a, out Signed832 b, out Signed832 c,
                    out var ln, out var ld, out var un, out var ud)
                && CapsuleFiniteSectionFrame.IntersectsQuadratic(a, b, c, ln, ld, un, ud, clip)) return true;
        return false;
    }

    /// <summary>Tests a retained exact subsegment without rounding either cut endpoint.</summary>
    internal bool IntersectsSegment(FixedSegment segment, in RationalSegmentInterval clip) =>
        IntersectsTransformedSegment(Transform(segment.Start), Transform(segment.End), clip);

    internal bool TryGetFirstSegmentParameter(FixedSegment segment, ReadOnlySpan<Vector2d> prismOffsets,
        ContactQuadratic numerator, ContactQuadratic denominator, Span<ulong> root) => prismOffsets.IsEmpty
        ? TryGetFirstSegmentParameter(segment, numerator, denominator, root)
        : TryGetPrismSegmentParameter(segment, prismOffsets, numerator, denominator, root);

    /// <summary>Retains the first finite parameter admitted by the slab's projected footprint, without height clipping.</summary>
    internal bool TryGetFirstProjectedSegmentParameter(FixedSegment segment, ReadOnlySpan<Vector2d> prismOffsets,
        ContactQuadratic numerator, ContactQuadratic denominator, Span<ulong> root) => prismOffsets.IsEmpty
        ? TryGetFirstSegmentParameter(segment, numerator, denominator, root, includeHeight: false)
        : TryGetPrismSegmentParameter(segment, prismOffsets, numerator, denominator, root, includeHeight: false);

    private bool TryGetPrismSegmentParameter(FixedSegment segment, ReadOnlySpan<Vector2d> offsets,
        ContactQuadratic numerator, ContactQuadratic denominator, Span<ulong> root, bool includeHeight = true)
    {
        numerator.Clear(); denominator.Set(Signed576.ExtendValue(Signed320.ExtendValue(Signed192.Signed(1)))); root.Clear();
        Span<ulong> values = stackalloc ulong[8 * PointWords]; Span<int> signs = stackalloc int[8];
        ContactQuadratic upperN = ContactQuadratic.At(values, signs, 0, PointWords),
            upperD = ContactQuadratic.At(values, signs, 1, PointWords),
            n = ContactQuadratic.At(values, signs, 2, PointWords),
            d = ContactQuadratic.At(values, signs, 3, PointWords);
        upperN.Set(Signed576.ExtendValue(Signed320.ExtendValue(Signed192.Signed(1)))); upperN.CopyTo(upperD);
        WideAxis3 first = Transform(segment.Start), last = Transform(segment.End);
        int winding = GetPrismWinding(offsets);
        for (int wall = includeHeight ? -2 : 0; wall < offsets.Length; wall++)
        {
            Signed576 a, b;
            if (wall == -2)
            {
                a = Signed576.ExtendValue(WideArithmetic.AddSigned320(first.Y, _cap));
                b = Signed576.ExtendValue(WideArithmetic.AddSigned320(last.Y, _cap));
            }
            else if (wall == -1)
            {
                a = Signed576.ExtendValue(WideArithmetic.SubtractSigned320(_cap, first.Y));
                b = Signed576.ExtendValue(WideArithmetic.SubtractSigned320(_cap, last.Y));
            }
            else
            {
                Vector2d start = offsets[wall], end = offsets[(wall + 1) % offsets.Length];
                a = PrismWall(first, start, end); b = PrismWall(last, start, end);
                if (winding < 0) { a = WideArithmetic.SubtractSigned576(default, a); b = WideArithmetic.SubtractSigned576(default, b); }
            }
            Signed576 velocity = WideArithmetic.SubtractSigned576(b, a);
            if (velocity.IsZero) { if (a.Sign < 0) return false; continue; }
            if (velocity.Sign > 0)
            {
                n.Set(WideArithmetic.SubtractSigned576(default, a)); d.Set(velocity);
                if (ContactQuadratic.CompareRatios(n, d, root, numerator, denominator, root) > 0)
                { n.CopyTo(numerator); d.CopyTo(denominator); }
            }
            else
            {
                n.Set(a); d.Set(WideArithmetic.SubtractSigned576(default, velocity));
                if (ContactQuadratic.CompareRatios(n, d, root, upperN, upperD, root) < 0)
                { n.CopyTo(upperN); d.CopyTo(upperD); }
            }
            if (ContactQuadratic.CompareRatios(numerator, denominator, root, upperN, upperD, root) > 0) return false;
        }
        return true;
    }

    private Signed576 PrismWall(WideAxis3 point, Vector2d start, Vector2d end)
    {
        Signed320 scale = WideArithmetic.AddSigned320(_radial.ShapeFrame.CoordinateDenominator, _radial.ShapeFrame.CoordinateDenominator);
        Signed320 x = WideArithmetic.SubtractSigned320(point.X, Signed320.NarrowValue(WideArithmetic.MultiplySigned320(scale, Signed192.Raw(start.X))));
        Signed320 z = WideArithmetic.SubtractSigned320(point.Z, Signed320.NarrowValue(WideArithmetic.MultiplySigned320(scale, Signed192.Raw(start.Y))));
        Signed320 ex = Signed320.ExtendValue(WideArithmetic.SubtractSigned192(Signed192.Raw(end.X), Signed192.Raw(start.X)));
        Signed320 ez = Signed320.ExtendValue(WideArithmetic.SubtractSigned192(Signed192.Raw(end.Y), Signed192.Raw(start.Y)));
        return WideArithmetic.SubtractSigned576(WideArithmetic.MultiplySigned320(ex, z), WideArithmetic.MultiplySigned320(ez, x));
    }

    internal static int GetPrismWinding(ReadOnlySpan<Vector2d> offsets)
    {
        if (offsets.Length < 3) throw new ArgumentException("A convex prism requires at least three offsets.", nameof(offsets));
        Vector2d a = offsets[0];
        for (int index = 1; index + 1 < offsets.Length; index++)
        {
            Signed192 x = WideArithmetic.SubtractSigned192(Signed192.Raw(offsets[index].X), Signed192.Raw(a.X));
            Signed192 z = WideArithmetic.SubtractSigned192(Signed192.Raw(offsets[index].Y), Signed192.Raw(a.Y));
            Signed192 y = WideArithmetic.SubtractSigned192(Signed192.Raw(offsets[index + 1].X), Signed192.Raw(a.X));
            Signed192 w = WideArithmetic.SubtractSigned192(Signed192.Raw(offsets[index + 1].Y), Signed192.Raw(a.Y));
            int sign = WideArithmetic.SubtractSigned320(WideArithmetic.MultiplySigned192(x, w), WideArithmetic.MultiplySigned192(z, y)).Sign;
            if (sign != 0) return sign;
        }
        throw new ArgumentException("The convex prism offsets must span an area.", nameof(offsets));
    }

    internal bool IntersectsSegment(FixedSegment segment)
    {
        WideAxis3 start = _radial.Transform(segment.Start), delta = Difference(_radial.Transform(segment.End), start);
        for (int primitive = 0; primitive < (_radial.Length == Fixed64.Zero ? 1 : 3); primitive++)
            if (TryGetPrimitive(start, delta, primitive, out Signed832 a, out Signed832 b, out Signed832 c,
                    out Signed320 ln, out Signed320 ld, out Signed320 un, out Signed320 ud)
                && CapsuleFiniteSectionFrame.IntersectsQuadratic(a, b, c, ln, ld, un, ud)) return true;
        return false;
    }

    internal bool TryGetFirstSegmentParameter(FixedSegment segment,
        ContactQuadratic numerator, ContactQuadratic denominator, Span<ulong> root)
        => TryGetFirstSegmentParameter(segment, numerator, denominator, root, includeHeight: true);

    private bool TryGetFirstSegmentParameter(FixedSegment segment,
        ContactQuadratic numerator, ContactQuadratic denominator, Span<ulong> root, bool includeHeight)
    {
        numerator.Clear(); denominator.Clear(); root.Clear();
        WideAxis3 start = _radial.Transform(segment.Start), delta = Difference(_radial.Transform(segment.End), start);
        Span<ulong> values = stackalloc ulong[10 * PointWords]; Span<int> signs = stackalloc int[10];
        Span<ulong> candidateRoot = stackalloc ulong[RootWords];
        ContactQuadratic qa = ContactQuadratic.At(values, signs, 0, PointWords),
            qb = ContactQuadratic.At(values, signs, 1, PointWords),
            qc = ContactQuadratic.At(values, signs, 2, PointWords),
            n = ContactQuadratic.At(values, signs, 3, PointWords),
            d = ContactQuadratic.At(values, signs, 4, PointWords);
        bool found = false;
        for (int primitive = 0; primitive < (_radial.Length == Fixed64.Zero ? 1 : 3); primitive++)
        {
            if (!TryGetPrimitive(start, delta, primitive, out Signed832 a, out Signed832 b, out Signed832 c,
                    out Signed320 ln, out Signed320 ld, out Signed320 un, out Signed320 ud, includeHeight)
                || !CapsuleFiniteSectionFrame.IntersectsQuadratic(a, b, c, ln, ld, un, ud)) continue;
            n.Clear(); d.Clear(); candidateRoot.Clear();
            SetFirstQuadraticParameter(a, b, c, ln, ld, qa, qb, qc, n, d, candidateRoot);
            if (!found || ContactQuadratic.CompareRatios(n, d, candidateRoot, numerator, denominator, root) < 0)
            {
                n.CopyTo(numerator); d.CopyTo(denominator); candidateRoot.CopyTo(root); found = true;
            }
        }
        return found;
    }

    private bool TryGetPrimitive(WideAxis3 start, WideAxis3 delta, int primitive,
        out Signed832 a, out Signed832 b, out Signed832 c,
        out Signed320 ln, out Signed320 ld, out Signed320 un, out Signed320 ud, bool includeHeight = true)
    {
        if (!_radial.TryGetPrimitiveWide(Radial(start), Radial(delta), primitive, out a, out b, out c,
                out ln, out ld, out un, out ud)) return false;
        if (!includeHeight) return true;
        Signed320 rn = ln, rd = ld, sn = un, sd = ud;
        if (!CapsuleFiniteSectionFrame.TryGetCenteredAxisInterval(start.Y, delta.Y, _cap, out ln, out ld, out un, out ud))
            return false;
        // Doubled normalized-frame coordinates and differences are <198 bits.
        // Interval cross products fit 398 bits; radial coefficients <400,
        // discriminants <801. Flat clipping adds no polynomial factors, so
        // the existing ten-limb point and seventeen-limb root banks suffice.
        if (Compare(rn, rd, ln, ld) > 0) { ln = rn; ld = rd; }
        if (Compare(sn, sd, un, ud) < 0) { un = sn; ud = sd; }
        return Compare(ln, ld, un, ud) <= 0;
    }

    private static int Compare(Signed320 n, Signed320 d, Signed320 otherN, Signed320 otherD) =>
        WideArithmetic.SubtractSigned832(WideArithmetic.MultiplySigned576ToSigned832(Signed576.ExtendValue(n), otherD), WideArithmetic.MultiplySigned576ToSigned832(Signed576.ExtendValue(otherN), d)).Sign;

    private static void SetFirstQuadraticParameter(Signed832 a, Signed832 b, Signed832 c,
        Signed320 lowerN, Signed320 lowerD, ContactQuadratic qa, ContactQuadratic qb, ContactQuadratic qc,
        ContactQuadratic numerator, ContactQuadratic denominator, Span<ulong> root)
    {
        if (WideFiniteConeIntersection.GetPolynomialSignAtRationalParameter(a, b, c, lowerN, lowerD) <= 0)
        {
            numerator.Set(Signed576.ExtendValue(lowerN)); denominator.Set(Signed576.ExtendValue(lowerD));
            return;
        }
        qa.Set(a); qb.Set(b); qc.Set(c);
        bool retained = ContactQuadratic.TryGetParameter(qa, qb, qc, -1, root, numerator, denominator);
        if (!retained) retained = ContactQuadratic.TryGetParameter(qa, qb, qc, 1, root, numerator, denominator);
        System.Diagnostics.Debug.Assert(retained);
    }

    private static WideAxis3 Radial(WideAxis3 point) => new(point.X, point.Z, default);
    private static WideAxis3 Difference(WideAxis3 end, WideAxis3 start) => new(
        WideArithmetic.SubtractSigned320(end.X, start.X), WideArithmetic.SubtractSigned320(end.Y, start.Y),
        WideArithmetic.SubtractSigned320(end.Z, start.Z));
}
