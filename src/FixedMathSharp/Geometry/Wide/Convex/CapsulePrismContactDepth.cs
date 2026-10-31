//=======================================================================
// CapsulePrismContactDepth.cs
//=======================================================================
// MIT License, Copyright (c) 2024-present David Oravsky (mrdav30)
// See LICENSE file in the project root for full license information.
//=======================================================================

using System;

namespace FixedMathSharp.Geometry;

/// <summary>Borrowed exact capsule/prism depth from a support projection or certified closest pair.</summary>
internal readonly ref struct CapsulePrismContactDepth
{
    private readonly WideConvexPrismRelations.ProjectionDepth _projection;
    private readonly ContactQuadratic _distanceSquared, _distanceDenominator;
    private readonly Fixed64 _radius;
    private readonly bool _edge;

    internal CapsulePrismContactDepth(in WideConvexPrismRelations.ProjectionDepth projection)
    {
        this = default; _projection = projection;
        System.Diagnostics.Debug.Assert(projection.RadialKind == WideConvexPrismRelations.RadialKind.Capsule);
    }
    internal CapsulePrismContactDepth(ContactQuadratic distanceSquared, ContactQuadratic denominator, Fixed64 radius)
    { this = default; _edge = true; _distanceSquared = distanceSquared; _distanceDenominator = denominator; _radius = radius; }

    internal int Compare(scoped in CapsulePlaneRaySelection ray, in CapsulePlaneRayFrame frame)
    {
        int qWords = _edge ? ContactQuadratic.ActiveWords(_distanceSquared) : 0;
        int dWords = _edge ? ContactQuadratic.ActiveWords(_distanceDenominator) : 0;
        // Projection P² needs <1409 bits; the remaining terms fit 24 words.
        // Closest-pair products retain their complete active rational banks.
        int words = _edge ? Math.Max(qWords, dWords + 2) + 2 : 24;
        Span<ulong> values = stackalloc ulong[16 * words], root = stackalloc ulong[_edge ? qWords + dWords : 9];
        Span<int> signs = stackalloc int[16];
        var n = ContactQuadratic.At(values, signs, 0, words); var d = ContactQuadratic.At(values, signs, 1, words);
        var p = ContactQuadratic.At(values, signs, 2, words); var c = ContactQuadratic.At(values, signs, 3, words);
        var a = ContactQuadratic.At(values, signs, 4, words); var t = ContactQuadratic.At(values, signs, 5, words);
        var u = ContactQuadratic.At(values, signs, 6, words); var v = ContactQuadratic.At(values, signs, 7, words);
        root.Clear();
        if (_edge)
        {
            // (R-sqrt(Q/D))² = (R²D+Q-2R*sqrt(QD))/D.
            _distanceSquared.CopyTo(n); _distanceDenominator.CopyTo(d);
            p.Set(Signed832.ExtendValue(Signed192.Raw(_radius)));
            ContactQuadratic.Multiply(p, p, ReadOnlySpan<ulong>.Empty, t);
            ContactQuadratic.Multiply(t, d, ReadOnlySpan<ulong>.Empty, u); n.Add(u);
            p.CopyTo(t); t.Add(t); t.Rational[..WideArithmetic.GetActiveMagnitudeLength(t.Rational)].CopyTo(n.Radical);
            n.Signs[1] = -t.Signs[0];
            WideArithmetic.MultiplyMagnitudes(_distanceSquared.Rational[..qWords], _distanceDenominator.Rational[..dWords], root);
        }
        else
        {
            // (R+P/(C*sqrt(A)))² retains C: ray and support charts need not share it.
            p.Set(Signed832.ExtendValue(_projection.Rational)); c.Set(Signed576.ExtendValue(_projection.Common));
            a.Set(_projection.AxisSquared);
            ContactQuadratic.Scale(c, Signed576.ExtendValue(Signed192.Raw(_projection.Radius)), t);
            ContactQuadratic.Multiply(p, p, ReadOnlySpan<ulong>.Empty, n);
            ContactQuadratic.Multiply(t, t, ReadOnlySpan<ulong>.Empty, u);
            ContactQuadratic.Multiply(u, a, ReadOnlySpan<ulong>.Empty, v); n.Add(v);
            ContactQuadratic.Multiply(p, t, ReadOnlySpan<ulong>.Empty, u); u.Add(u);
            u.Rational[..WideArithmetic.GetActiveMagnitudeLength(u.Rational)].CopyTo(n.Radical); n.Signs[1] = u.Signs[0];
            ContactQuadratic.Multiply(c, c, ReadOnlySpan<ulong>.Empty, u);
            ContactQuadratic.Multiply(u, a, ReadOnlySpan<ulong>.Empty, d);
            WideArithmetic.GetMagnitude(_projection.AxisSquared, root);
        }
        return ray.CompareSquaredDepth(frame, n, d, root);
    }
}
