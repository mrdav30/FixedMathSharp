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

    internal CapsuleFiniteSectionFrame(Vector3d origin, FixedQuaternion rotation,
        Vector3d center, FixedQuaternion capsuleRotation, Fixed64 length, Fixed64 radius)
    {
        if (length < Fixed64.Zero) throw new ArgumentOutOfRangeException(nameof(length));
        if (radius < Fixed64.Zero) throw new ArgumentOutOfRangeException(nameof(radius));
        if (!rotation.IsNormalized()) throw new ArgumentException("The authored rotation must be normalized.", nameof(rotation));
        if (!capsuleRotation.IsNormalized()) throw new ArgumentException("The capsule rotation must be normalized.", nameof(capsuleRotation));
        Length = length;
        ShapeFrame = new CylinderPolytopeFrame(center, capsuleRotation, length, radius, origin, rotation);
        Signed320 scaledRadius = WideArithmetic.MultiplySigned192(ShapeFrame.Radius, ShapeFrame.Denominator);
        _radiusSquared = WideArithmetic.MultiplySigned320(scaledRadius, scaledRadius);
    }

    internal WideAxis3 Transform(Vector3d point) => ShapeFrame.Transform(point);
    internal bool ContainsPoint(Vector3d point) => ContainsTransformedPoint(Transform(point));

    internal bool ContainsTransformedPoint(WideAxis3 point)
    {
        Signed320 y = point.Y.Sign < 0 ? WideArithmetic.Negate(point.Y) : point.Y;
        y = WideArithmetic.SubtractSigned320(y, ShapeFrame.Cap);
        if (y.Sign < 0) y = default;
        var offset = new WideAxis3(point.X, y, point.Z);
        return WideArithmetic.SubtractSigned576(Dot(offset, offset), _radiusSquared).Sign <= 0;
    }

    internal bool IntersectsSegment(FixedSegment segment)
    {
        WideAxis3 start = Transform(segment.Start), delta = Difference(Transform(segment.End), start);
        for (int primitive = 0; primitive < (Length == Fixed64.Zero ? 1 : 3); primitive++)
            if (TryGetPrimitive(start, delta, primitive, out Signed576 a, out Signed576 b, out Signed576 c,
                    out Signed320 lowerN, out Signed320 lowerD, out Signed320 upperN, out Signed320 upperD)
                && IntersectsQuadratic(a, b, c, lowerN, lowerD, upperN, upperD)) return true;
        return false;
    }

    /// <summary>Retains the first closed section parameter in the input segment's order.</summary>
    internal bool TryGetFirstSegmentParameter(FixedSegment segment,
        ContactQuadratic numerator, ContactQuadratic denominator, Span<ulong> root)
    {
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
            if (PolynomialSign(a, b, c, lowerN, lowerD) <= 0)
            {
                n.Set(Signed576.ExtendValue(lowerN)); d.Set(Signed576.ExtendValue(lowerD));
            }
            else
            {
                qa.Set(a); qb.Set(b); qc.Set(c);
                // Every primitive is convex. Its lower root starts the
                // admitted interval; a repeated root is a closed point.
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

    private bool TryGetPrimitive(WideAxis3 start, WideAxis3 delta, int primitive,
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
            if (!TryGetBodyInterval(start.Y, delta.Y, out lowerN, out lowerD, out upperN, out upperD)) return false;
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

    private bool TryGetBodyInterval(Signed320 start, Signed320 velocity,
        out Signed320 lowerN, out Signed320 lowerD, out Signed320 upperN, out Signed320 upperD)
    {
        lowerN = default;
        lowerD = upperN = upperD = Signed320.ExtendValue(Signed192.Signed(1));
        if (velocity.IsZero)
            return WideArithmetic.AddSigned320(start, ShapeFrame.Cap).Sign >= 0
                && WideArithmetic.SubtractSigned320(start, ShapeFrame.Cap).Sign <= 0;
        Signed320 first = WideArithmetic.SubtractSigned320(WideArithmetic.Negate(ShapeFrame.Cap), start);
        Signed320 second = WideArithmetic.SubtractSigned320(ShapeFrame.Cap, start);
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

    private static bool IntersectsQuadratic(Signed576 a, Signed576 b, Signed576 c,
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

    private static int PolynomialSign(Signed576 a, Signed576 b, Signed576 c, Signed320 n, Signed320 d) =>
        WideFiniteConeIntersection.GetPolynomialSignAtRationalParameter(Signed832.ExtendValue(a),
            Signed832.ExtendValue(b), Signed832.ExtendValue(c), n, d);

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
