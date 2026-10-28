//=======================================================================
// CapsuleFiniteSectionFrame.cs
//=======================================================================
// MIT License, Copyright (c) 2024-present David Oravsky (mrdav30)
// See LICENSE file in the project root for full license information.
//=======================================================================
using System;

namespace FixedMathSharp.Geometry;

/// <summary>Exact closed finite-capsule admission in an authored rigid frame.</summary>
internal readonly struct CapsuleFiniteSectionFrame
{
    internal readonly CylinderPolytopeFrame ShapeFrame;
    internal readonly Fixed64 Length;
    private readonly Signed576 _radiusSquared;
    private readonly Signed832 _wideRadiusSquared;

    /// <summary>Exact capsule-local coordinates supplied by a retained rational source chart.</summary>
    internal CapsuleFiniteSectionFrame(Fixed64 length, Fixed64 radius, Signed320 coordinateDenominator)
    {
        Length = length;
        ShapeFrame = new CylinderPolytopeFrame(length, radius, coordinateDenominator);
        Signed320 scaledRadius = Signed320.NarrowValue(WideArithmetic.MultiplySigned320(coordinateDenominator, Signed320.ExtendValue(ShapeFrame.Radius)));
        _wideRadiusSquared = ShapeFrame.HasExactScale ? Product(scaledRadius, scaledRadius) : default;
        _radiusSquared = ShapeFrame.HasExactScale ? default : WideArithmetic.MultiplySigned320(scaledRadius, scaledRadius);
    }

    internal CapsuleFiniteSectionFrame(Vector3d origin, FixedQuaternion rotation,
        Vector3d center, FixedQuaternion capsuleRotation, Fixed64 length, Fixed64 radius)
    {
        _wideRadiusSquared = default;
        if (length < Fixed64.Zero) throw new ArgumentOutOfRangeException(nameof(length));
        if (radius < Fixed64.Zero) throw new ArgumentOutOfRangeException(nameof(radius));
        if (!rotation.IsNormalized()) throw new ArgumentException("The authored rotation must be normalized.", nameof(rotation));
        if (!capsuleRotation.IsNormalized()) throw new ArgumentException("The capsule rotation must be normalized.", nameof(capsuleRotation));
        Length = length;
        ShapeFrame = new CylinderPolytopeFrame(center, capsuleRotation, length, radius, origin, rotation);
        Signed320 scaledRadius = WideArithmetic.MultiplySigned192(ShapeFrame.Radius, ShapeFrame.Denominator);
        _radiusSquared = WideArithmetic.MultiplySigned320(scaledRadius, scaledRadius);
    }

    internal CapsuleFiniteSectionFrame(in ScaledRigidFrame3d sourceFrame,
        Vector3d center, FixedQuaternion capsuleRotation, Fixed64 length, Fixed64 radius)
    {
        if (sourceFrame.IsRigid)
        {
            this = new CapsuleFiniteSectionFrame(sourceFrame.RigidOrigin, sourceFrame.Rotation,
                center, capsuleRotation, length, radius);
            return;
        }
        if (length < Fixed64.Zero || length > (Fixed64)8192)
            throw new ArgumentOutOfRangeException(nameof(length));
        if (radius < Fixed64.Zero || radius > (Fixed64)4096)
            throw new ArgumentOutOfRangeException(nameof(radius));
        if (!capsuleRotation.IsNormalized())
            throw new ArgumentException("The capsule rotation must be normalized.", nameof(capsuleRotation));
        Length = length;
        ShapeFrame = new CylinderPolytopeFrame(sourceFrame, center, capsuleRotation, Signed192.Raw(length), radius);
        Signed320 scaledRadius = Signed320.NarrowValue(
            WideArithmetic.MultiplySigned320(ShapeFrame.CoordinateDenominator, ShapeFrame.Radius));
        _wideRadiusSquared = Product(scaledRadius, scaledRadius);
        _radiusSquared = default;
    }

    internal WideAxis3 Transform(Vector3d point) => ShapeFrame.Transform(point);
    internal bool ContainsPoint(Vector3d point) => ContainsTransformedPoint(Transform(point));

    internal bool ContainsTransformedPoint(WideAxis3 point)
    {
        Signed320 y = point.Y.Sign < 0 ? WideArithmetic.Negate(point.Y) : point.Y;
        y = WideArithmetic.SubtractSigned320(y, ShapeFrame.Cap);
        if (y.Sign < 0) y = default;
        var offset = new WideAxis3(point.X, y, point.Z);
        return ShapeFrame.HasExactScale
            ? WideArithmetic.SubtractSigned832(DotWide(offset, offset), _wideRadiusSquared).Sign <= 0
            : WideArithmetic.SubtractSigned576(Dot(offset, offset), _radiusSquared).Sign <= 0;
    }

    internal bool IntersectsSegment(FixedSegment segment)
    {
        WideAxis3 start = Transform(segment.Start), delta = Difference(Transform(segment.End), start);
        if (ShapeFrame.HasExactScale)
            return IntersectsWideSegment(start, delta, default);
        for (int primitive = 0; primitive < (Length == Fixed64.Zero ? 1 : 3); primitive++)
            if (TryGetPrimitive(start, delta, primitive, out Signed576 a, out Signed576 b, out Signed576 c,
                    out Signed320 lowerN, out Signed320 lowerD, out Signed320 upperN, out Signed320 upperD)
                && IntersectsQuadratic(a, b, c, lowerN, lowerD, upperN, upperD)) return true;
        return false;
    }

    /// <summary>Tests the retained exact subsegment, without rounding its endpoints.</summary>
    internal bool IntersectsSegment(FixedSegment segment, in RationalSegmentInterval clip)
    {
        if (!clip.IsClipped) return IntersectsSegment(segment);
        WideAxis3 start = Transform(segment.Start), delta = Difference(Transform(segment.End), start);
        if (ShapeFrame.HasExactScale || clip.HasWideBounds)
            return IntersectsWideSegment(start, delta, clip);
        for (int primitive = 0; primitive < (Length == Fixed64.Zero ? 1 : 3); primitive++)
        {
            if (!TryGetPrimitive(start, delta, primitive, out Signed576 a, out Signed576 b, out Signed576 c,
                out Signed320 ln, out Signed320 ld, out Signed320 un, out Signed320 ud)) continue;
            if (RationalSegmentInterval.CompareParameters(ln, ld, clip.LowerNumerator, clip.LowerDenominator) < 0)
            { ln = clip.LowerNumerator; ld = clip.LowerDenominator; }
            if (RationalSegmentInterval.CompareParameters(un, ud, clip.UpperNumerator, clip.UpperDenominator) > 0)
            { un = clip.UpperNumerator; ud = clip.UpperDenominator; }
            if (RationalSegmentInterval.CompareParameters(ln, ld, un, ud) <= 0
                && IntersectsQuadratic(a, b, c, ln, ld, un, ud)) return true;
        }
        return false;
    }

    private bool IntersectsWideSegment(WideAxis3 start, WideAxis3 delta, in RationalSegmentInterval clip)
    {
        for (int primitive = 0; primitive < (Length == Fixed64.Zero ? 1 : 3); primitive++)
        {
            if (!TryGetPrimitiveWide(start, delta, primitive, out Signed832 a, out Signed832 b, out Signed832 c,
                    out Signed320 ln, out Signed320 ld, out Signed320 un, out Signed320 ud))
                continue;
            Signed832 lowerN = Signed832.ExtendValue(Signed576.ExtendValue(ln)),
                lowerD = Signed832.ExtendValue(Signed576.ExtendValue(ld)),
                upperN = Signed832.ExtendValue(Signed576.ExtendValue(un)),
                upperD = Signed832.ExtendValue(Signed576.ExtendValue(ud));
            if (clip.IsClipped)
            {
                if (RationalSegmentInterval.CompareParameters(lowerN, lowerD, clip.WideLowerNumerator, clip.WideLowerDenominator) < 0)
                { lowerN = clip.WideLowerNumerator; lowerD = clip.WideLowerDenominator; }
                if (RationalSegmentInterval.CompareParameters(upperN, upperD, clip.WideUpperNumerator, clip.WideUpperDenominator) > 0)
                { upperN = clip.WideUpperNumerator; upperD = clip.WideUpperDenominator; }
            }
            if (RationalSegmentInterval.CompareParameters(lowerN, lowerD, upperN, upperD) <= 0
                && IntersectsQuadratic(a, b, c, lowerN, lowerD, upperN, upperD))
                return true;
        }
        return false;
    }

    internal bool TryGetPrimitiveWide(WideAxis3 start, WideAxis3 delta, int primitive,
        out Signed832 a, out Signed832 b, out Signed832 c,
        out Signed320 lowerN, out Signed320 lowerD, out Signed320 upperN, out Signed320 upperD)
    {
        if (!ShapeFrame.HasExactScale)
        {
            bool found = TryGetPrimitive(start, delta, primitive, out Signed576 sa, out Signed576 sb, out Signed576 sc,
                out lowerN, out lowerD, out upperN, out upperD);
            a = Signed832.ExtendValue(sa); b = Signed832.ExtendValue(sb); c = Signed832.ExtendValue(sc);
            return found;
        }
        lowerN = default;
        lowerD = upperN = upperD = Signed320.One;
        if (primitive == 0 && Length != Fixed64.Zero)
        {
            a = b = c = default;
            if (!TryGetCenteredAxisInterval(start.Y, delta.Y, ShapeFrame.Cap,
                    out lowerN, out lowerD, out upperN, out upperD))
                return false;
            start = new WideAxis3(start.X, default, start.Z);
            delta = new WideAxis3(delta.X, default, delta.Z);
        }
        else
        {
            Signed320 cap = primitive == 1 ? WideArithmetic.Negate(ShapeFrame.Cap) : ShapeFrame.Cap;
            start = new WideAxis3(start.X, WideArithmetic.SubtractSigned320(start.Y, cap), start.Z);
        }
        a = DotWide(delta, delta); b = DotWide(start, delta);
        c = WideArithmetic.SubtractSigned832(DotWide(start, start), _wideRadiusSquared);
        return true;
    }

    /// <summary>Retains the first closed section parameter in the input segment's order.</summary>
    internal bool TryGetFirstSegmentParameter(FixedSegment segment,
        ContactQuadratic numerator, ContactQuadratic denominator, Span<ulong> root)
    {
        if (ShapeFrame.HasExactScale)
            return TryGetFirstWideSegmentParameter(segment, numerator, denominator, root);
        numerator.Clear(); denominator.Clear(); root.Clear();
        WideAxis3 start = Transform(segment.Start), delta = Difference(Transform(segment.End), start);
        Span<ulong> values = stackalloc ulong[10 * FiniteSectionPoint.Words];
        Span<int> signs = stackalloc int[10];
        Span<ulong> candidateRoot = stackalloc ulong[FiniteSectionPoint.RootWords];
        ContactQuadratic qa = ContactQuadratic.At(values, signs, 0, FiniteSectionPoint.Words);
        ContactQuadratic qb = ContactQuadratic.At(values, signs, 1, FiniteSectionPoint.Words);
        ContactQuadratic qc = ContactQuadratic.At(values, signs, 2, FiniteSectionPoint.Words);
        ContactQuadratic n = ContactQuadratic.At(values, signs, 3, FiniteSectionPoint.Words);
        ContactQuadratic d = ContactQuadratic.At(values, signs, 4, FiniteSectionPoint.Words);
        bool found = false;
        for (int primitive = 0; primitive < (Length == Fixed64.Zero ? 1 : 3); primitive++)
        {
            if (!TryGetPrimitive(start, delta, primitive, out Signed576 a, out Signed576 b, out Signed576 c,
                    out Signed320 lowerN, out Signed320 lowerD, out Signed320 upperN, out Signed320 upperD)
                || !IntersectsQuadratic(a, b, c, lowerN, lowerD, upperN, upperD)) continue;
            n.Clear(); d.Clear(); candidateRoot.Clear();
            GetFirstQuadraticParameter(a, b, c, lowerN, lowerD, qa, qb, qc, n, d, candidateRoot);
            if (!found || ContactQuadratic.CompareRatios(n, d, candidateRoot, numerator, denominator, root) < 0)
            {
                n.CopyTo(numerator); d.CopyTo(denominator); candidateRoot.CopyTo(root); found = true;
            }
        }
        return found;
    }

    private bool TryGetFirstWideSegmentParameter(FixedSegment segment,
        ContactQuadratic numerator, ContactQuadratic denominator, Span<ulong> root)
    {
        numerator.Clear(); denominator.Clear(); root.Clear();
        WideAxis3 start = Transform(segment.Start), delta = Difference(Transform(segment.End), start);
        int words = numerator.FieldWords;
        Span<ulong> values = stackalloc ulong[10 * words];
        Span<int> signs = stackalloc int[10];
        Span<ulong> candidateRoot = stackalloc ulong[root.Length];
        ContactQuadratic qa = ContactQuadratic.At(values, signs, 0, words);
        ContactQuadratic qb = ContactQuadratic.At(values, signs, 1, words);
        ContactQuadratic qc = ContactQuadratic.At(values, signs, 2, words);
        ContactQuadratic n = ContactQuadratic.At(values, signs, 3, words);
        ContactQuadratic d = ContactQuadratic.At(values, signs, 4, words);
        bool found = false;
        for (int primitive = 0; primitive < (Length == Fixed64.Zero ? 1 : 3); primitive++)
        {
            if (!TryGetPrimitiveWide(start, delta, primitive, out Signed832 a, out Signed832 b, out Signed832 c,
                    out Signed320 ln, out Signed320 ld, out Signed320 un, out Signed320 ud)
                || !IntersectsQuadratic(a, b, c, ln, ld, un, ud))
                continue;
            n.Clear(); d.Clear(); candidateRoot.Clear();
            if (WideFiniteConeIntersection.GetPolynomialSignAtRationalParameter(a, b, c, ln, ld) <= 0)
            {
                n.Set(Signed576.ExtendValue(ln)); d.Set(Signed576.ExtendValue(ld));
            }
            else
            {
                qa.Set(a); qb.Set(b); qc.Set(c);
                bool retained = ContactQuadratic.TryGetParameter(qa, qb, qc, -1, candidateRoot, n, d);
                if (!retained) retained = ContactQuadratic.TryGetParameter(qa, qb, qc, 1, candidateRoot, n, d);
                System.Diagnostics.Debug.Assert(retained);
            }
            if (!found || ContactQuadratic.CompareRatios(n, d, candidateRoot, numerator, denominator, root) < 0)
            {
                n.CopyTo(numerator); d.CopyTo(denominator); candidateRoot.CopyTo(root); found = true;
            }
        }
        return found;
    }

    internal bool TryGetPrimitive(WideAxis3 start, WideAxis3 delta, int primitive,
        out Signed576 a, out Signed576 b, out Signed576 c,
        out Signed320 lowerN, out Signed320 lowerD, out Signed320 upperN, out Signed320 upperD)
    {
        lowerN = default;
        lowerD = upperN = upperD = Signed320.ExtendValue(Signed192.Signed(1));
        // The capsule is exactly a finite cylinder plus both full endpoint
        // balls. Full balls avoid hemisphere clipping without changing the
        // union, and the zero-core capsule needs only one ball.
        bool body = primitive == 0 && Length != Fixed64.Zero;
        if (body)
        {
            a = b = c = default;
            if (!TryGetCenteredAxisInterval(start.Y, delta.Y, ShapeFrame.Cap, out lowerN, out lowerD, out upperN, out upperD)) return false;
            start = new WideAxis3(start.X, default, start.Z);
            delta = new WideAxis3(delta.X, default, delta.Z);
        }
        else
        {
            Signed320 cap = primitive == 1 ? WideArithmetic.Negate(ShapeFrame.Cap) : ShapeFrame.Cap;
            start = new WideAxis3(start.X, WideArithmetic.SubtractSigned320(start.Y, cap), start.Z);
        }
        // Doubled rigid coordinates and differences are below 198 bits;
        // squared coefficients fit 400 bits and discriminants 801 bits.
        a = Dot(delta, delta); b = Dot(start, delta);
        c = WideArithmetic.SubtractSigned576(Dot(start, start), _radiusSquared);
        return true;
    }

    internal static bool TryGetCenteredAxisInterval(Signed320 start, Signed320 velocity, Signed320 cap,
        out Signed320 lowerN, out Signed320 lowerD, out Signed320 upperN, out Signed320 upperD)
    {
        lowerN = default;
        lowerD = upperN = upperD = Signed320.ExtendValue(Signed192.Signed(1));
        if (velocity.IsZero)
            return WideArithmetic.AddSigned320(start, cap).Sign >= 0
                && WideArithmetic.SubtractSigned320(start, cap).Sign <= 0;
        Signed320 first = WideArithmetic.SubtractSigned320(WideArithmetic.Negate(cap), start);
        Signed320 second = WideArithmetic.SubtractSigned320(cap, start);
        Signed320 divisor = velocity;
        if (velocity.Sign < 0)
        {
            (first, second) = (WideArithmetic.Negate(second), WideArithmetic.Negate(first));
            divisor = WideArithmetic.Negate(divisor);
        }
        if (second.Sign < 0 || WideArithmetic.SubtractSigned320(first, divisor).Sign > 0) return false;
        if (first.Sign > 0) { lowerN = first; lowerD = divisor; }
        if (WideArithmetic.SubtractSigned320(second, divisor).Sign < 0) { upperN = second; upperD = divisor; }
        return true;
    }

    internal static bool IntersectsQuadratic(Signed576 a, Signed576 b, Signed576 c,
        Signed320 lowerN, Signed320 lowerD, Signed320 upperN, Signed320 upperD)
    {
        if (PolynomialSign(a, b, c, lowerN, lowerD) <= 0 || PolynomialSign(a, b, c, upperN, upperD) <= 0) return true;
        if (a.Sign == 0 || b.Sign >= 0) return false;
        Signed576 vertex = WideArithmetic.SubtractSigned576(default, b);
        if (CompareProducts(vertex, lowerD, a, lowerN) < 0 || CompareProducts(vertex, upperD, a, upperN) > 0) return false;
        System.Diagnostics.Debug.Assert(c.Sign > 0);
        return WideArithmetic.CompareNonNegativeProducts(Signed832.ExtendValue(b), Signed832.ExtendValue(b),
            Signed832.ExtendValue(a), Signed832.ExtendValue(c)) >= 0;
    }

    internal static bool IntersectsQuadratic(Signed832 a, Signed832 b, Signed832 c,
        Signed320 lowerN, Signed320 lowerD, Signed320 upperN, Signed320 upperD)
    {
        if (WideFiniteConeIntersection.GetPolynomialSignAtRationalParameter(a, b, c, lowerN, lowerD) <= 0
            || WideFiniteConeIntersection.GetPolynomialSignAtRationalParameter(a, b, c, upperN, upperD) <= 0)
            return true;
        if (a.Sign <= 0 || b.Sign >= 0)
            return false;
        Signed832 numerator = WideArithmetic.SubtractSigned832(default, b);
        if (WideArithmetic.CompareNonNegativeProducts(numerator, Signed832.ExtendValue(Signed576.ExtendValue(lowerD)),
                a, Signed832.ExtendValue(Signed576.ExtendValue(lowerN))) < 0
            || WideArithmetic.CompareNonNegativeProducts(numerator, Signed832.ExtendValue(Signed576.ExtendValue(upperD)),
                a, Signed832.ExtendValue(Signed576.ExtendValue(upperN))) > 0)
            return false;
        return WideArithmetic.CompareNonNegativeProducts(b, b, a, c) >= 0;
    }

    internal static bool IntersectsQuadratic(Signed832 a, Signed832 b, Signed832 c,
        Signed832 lowerN, Signed832 lowerD, Signed832 upperN, Signed832 upperD)
    {
        if (WideFiniteConeIntersection.GetPolynomialSignAtRationalParameter(a, b, c, lowerN, lowerD) <= 0
            || WideFiniteConeIntersection.GetPolynomialSignAtRationalParameter(a, b, c, upperN, upperD) <= 0)
            return true;
        if (a.Sign <= 0 || b.Sign >= 0) return false;
        Signed832 numerator = WideArithmetic.SubtractSigned832(default, b);
        if (WideArithmetic.CompareNonNegativeProducts(numerator, lowerD, a, lowerN) < 0
            || WideArithmetic.CompareNonNegativeProducts(numerator, upperD, a, upperN) > 0)
            return false;
        return WideArithmetic.CompareNonNegativeProducts(b, b, a, c) >= 0;
    }

    /// <summary>Tests a rational borrowed quadratic on a closed clipped segment.</summary>
    internal static bool IntersectsQuadratic(scoped ContactQuadratic a, scoped ContactQuadratic b,
        scoped ContactQuadratic c, Signed320 lowerN, Signed320 lowerD, Signed320 upperN, Signed320 upperD)
    {
        Signed832 ln = Signed832.ExtendValue(Signed576.ExtendValue(lowerN));
        Signed832 ld = Signed832.ExtendValue(Signed576.ExtendValue(lowerD));
        Signed832 un = Signed832.ExtendValue(Signed576.ExtendValue(upperN));
        Signed832 ud = Signed832.ExtendValue(Signed576.ExtendValue(upperD));
        if (WideFiniteConeIntersection.GetBorrowedPolynomialSignAtRationalParameter(a, b, c, ln, ld) <= 0
            || WideFiniteConeIntersection.GetBorrowedPolynomialSignAtRationalParameter(a, b, c, un, ud) <= 0)
            return true;
        if (a.Signs[0] <= 0 || b.Signs[0] >= 0) return false;

        // Positive endpoints leave only an interior convex minimum. Preserve
        // both complete products, including the discriminant's subtraction bit.
        int coefficientWords = Math.Max(a.FieldWords, Math.Max(b.FieldWords, c.FieldWords));
        int words = Math.Max(2 * coefficientWords, coefficientWords + 5) + 1;
        Span<ulong> storage = stackalloc ulong[4 * words];
        Span<int> signs = stackalloc int[4];
        ContactQuadratic left = ContactQuadratic.At(storage, signs, 0, words);
        ContactQuadratic right = ContactQuadratic.At(storage, signs, 1, words);
        ContactQuadratic.Scale(b, ld, left); left.MultiplySign(-1);
        ContactQuadratic.Scale(a, ln, right); left.Add(right, -1);
        if (left.Signs[0] < 0) return false;
        ContactQuadratic.Scale(b, ud, left); left.MultiplySign(-1);
        ContactQuadratic.Scale(a, un, right); left.Add(right, -1);
        if (left.Signs[0] > 0) return false;
        ContactQuadratic.Multiply(b, b, ReadOnlySpan<ulong>.Empty, left);
        ContactQuadratic.Multiply(a, c, ReadOnlySpan<ulong>.Empty, right); left.Add(right, -1);
        return left.Signs[0] >= 0;
    }

    private static Signed832 DotWide(WideAxis3 left, WideAxis3 right) => WideArithmetic.AddSigned832(
        WideArithmetic.AddSigned832(Product(left.X, right.X), Product(left.Y, right.Y)), Product(left.Z, right.Z));

    private static Signed832 Product(Signed320 first, Signed320 second) =>
        WideArithmetic.MultiplySigned576ToSigned832(Signed576.ExtendValue(first), second);

    private static int PolynomialSign(Signed576 a, Signed576 b, Signed576 c, Signed320 n, Signed320 d) =>
        WideFiniteConeIntersection.GetPolynomialSignAtRationalParameter(Signed832.ExtendValue(a),
            Signed832.ExtendValue(b), Signed832.ExtendValue(c), n, d);

    /// <summary>Retains the first point of an already admitted convex quadratic interval.</summary>
    internal static void GetFirstQuadraticParameter(Signed576 a, Signed576 b, Signed576 c,
        Signed320 lowerN, Signed320 lowerD, ContactQuadratic qa, ContactQuadratic qb, ContactQuadratic qc,
        ContactQuadratic numerator, ContactQuadratic denominator, Span<ulong> root)
    {
        if (PolynomialSign(a, b, c, lowerN, lowerD) <= 0)
        {
            numerator.Set(Signed576.ExtendValue(lowerN)); denominator.Set(Signed576.ExtendValue(lowerD));
            return;
        }
        qa.Set(a); qb.Set(b); qc.Set(c);
        // The lower root starts the admitted interval. Repeated roots use
        // the sole +1 branch and remain closed point intersections.
        bool retained = ContactQuadratic.TryGetParameter(qa, qb, qc, -1, root, numerator, denominator);
        if (!retained) retained = ContactQuadratic.TryGetParameter(qa, qb, qc, 1, root, numerator, denominator);
        System.Diagnostics.Debug.Assert(retained);
    }

    private static int CompareProducts(Signed576 left, Signed320 leftScale, Signed576 right, Signed320 rightScale) =>
        WideArithmetic.SubtractSigned832(WideArithmetic.MultiplySigned576ToSigned832(left, leftScale),
            WideArithmetic.MultiplySigned576ToSigned832(right, rightScale)).Sign;

    private static Signed576 Dot(WideAxis3 left, WideAxis3 right) =>
        WideArithmetic.AddSigned576(WideArithmetic.AddSigned576(WideArithmetic.MultiplySigned320(left.X, right.X),
            WideArithmetic.MultiplySigned320(left.Y, right.Y)), WideArithmetic.MultiplySigned320(left.Z, right.Z));

    private static WideAxis3 Difference(WideAxis3 end, WideAxis3 start) =>
        new(WideArithmetic.SubtractSigned320(end.X, start.X), WideArithmetic.SubtractSigned320(end.Y, start.Y),
            WideArithmetic.SubtractSigned320(end.Z, start.Z));
}
