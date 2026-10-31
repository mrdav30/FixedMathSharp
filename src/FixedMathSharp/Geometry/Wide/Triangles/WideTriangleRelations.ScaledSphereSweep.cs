//=======================================================================
// WideTriangleRelations.ScaledSphereSweep.cs
//=======================================================================
// MIT License, Copyright (c) 2024-present David Oravsky (mrdav30)
// See LICENSE file in the project root for full license information.
//=======================================================================

using System;

namespace FixedMathSharp.Geometry;

/// <content>Retains finite face, edge and vertex roots for a sphere swept against an authored affine triangle.</content>
internal static partial class WideTriangleRelations
{
    internal static bool TryGetSweptSphereDistance(in FixedTriangle triangle, in ScaledRigidFrame3d frame,
        in FixedSegment query, Fixed64 length, Fixed64 radius, out Fixed64 distance)
    {
        distance = Fixed64.MaxValue;
        Span<WideDirection3d> points = stackalloc WideDirection3d[3];
        Signed320 denominator = frame.Denominator;
        for (int i = 0; i < 3; i++)
        {
            frame.GetPoint(i == 0 ? triangle.A : i == 1 ? triangle.B : triangle.C)
                .GetLocalPointNumerators(query.Start, FixedQuaternion.Identity,
                    out Signed576 x, out Signed576 y, out Signed576 z, out denominator);
            points[i] = new WideDirection3d(x, y, z);
        }
        // The caller has already admitted this chord's scalar length; every
        // component difference is bounded by that same representable magnitude.
        bool represented = Vector3d.TrySubtract(query.End, query.Start, out Vector3d delta);
        System.Diagnostics.Debug.Assert(represented);
        var velocity = new WideDirection3d(
            WideArithmetic.MultiplySigned320(denominator, Signed192.Raw(delta.X)),
            WideArithmetic.MultiplySigned320(denominator, Signed192.Raw(delta.Y)),
            WideArithmetic.MultiplySigned320(denominator, Signed192.Raw(delta.Z)));
        Signed576 expandedRadius = WideArithmetic.MultiplySigned320(denominator, Signed192.Raw(radius));
        Signed832 radiusSquare = WideArithmetic.MultiplySigned576ToSigned832(expandedRadius, expandedRadius);
        WideDirection3d normal = GetScaledTriangleLocalNormal(triangle, frame).Transform(frame.GeometryBasis);
        bool found = false;
        for (int feature = 0; feature < 7; feature++)
        {
            int vertex = feature < 3 ? feature : feature < 6 ? feature - 3 : 0;
            WideDirection3d point = points[vertex];
            WideDirection3d edge = feature is >= 3 and < 6 ? Difference(points[(vertex + 1) % 3], point) : default;
            if (feature == 6 && normal.IsZero || feature is >= 3 and < 6 && edge.IsZero) continue;
            if (TryGetScaledSphereFeatureRoot(triangle, frame, points, velocity, normal, point, edge,
                radiusSquare, feature, length, out Fixed64 candidate) && candidate < distance)
            { distance = candidate; found = true; }
        }
        return found;
    }

    private static bool TryGetScaledSphereFeatureRoot(in FixedTriangle triangle, in ScaledRigidFrame3d frame,
        ReadOnlySpan<WideDirection3d> points, in WideDirection3d velocity, in WideDirection3d normal,
        in WideDirection3d point, in WideDirection3d edge, Signed832 radiusSquare, int feature,
        Fixed64 length, out Fixed64 distance)
    {
        // Source-frame numerators are <320 bits and normals <351 bits.
        // The largest coefficients are <1346 bits, discriminants <2694 bits;
        // root admission and output scaling stay below this 3072-bit bank.
        const int words = 48;
        Span<ulong> storage = stackalloc ulong[20 * words]; Span<int> signs = stackalloc int[20];
        var a = ContactQuadratic.At(storage, signs, 0, words);
        var b = ContactQuadratic.At(storage, signs, 1, words);
        var c = ContactQuadratic.At(storage, signs, 2, words);
        var work = ContactQuadratic.At(storage, signs, 3, words);
        var term = ContactQuadratic.At(storage, signs, 4, words);
        var first = ContactQuadratic.At(storage, signs, 5, words);
        var second = ContactQuadratic.At(storage, signs, 6, words);
        var numerator = ContactQuadratic.At(storage, signs, 7, words);
        var divisor = ContactQuadratic.At(storage, signs, 8, words);
        var parameterWork = ContactQuadratic.At(storage, signs, 9, words);
        Signed832 vv = WideDirection3d.Dot(velocity, velocity), vp = WideDirection3d.Dot(velocity, point);
        Signed832 pp = WideDirection3d.Dot(point, point);
        Signed832 ee = default, ev = default, ep = default;
        if (feature < 3)
        {
            a.Set(vv); b.Set(vp); b.MultiplySign(-1);
            c.Set(WideArithmetic.SubtractSigned832(pp, radiusSquare));
        }
        else if (feature < 6)
        {
            ee = WideDirection3d.Dot(edge, edge); ev = WideDirection3d.Dot(edge, velocity);
            ep = WideDirection3d.Dot(edge, point);
            Multiply(ee, vv, first, second, a); Multiply(ev, ev, first, second, term); a.Add(term, -1);
            Multiply(ev, ep, first, second, b); Multiply(ee, vp, first, second, term); b.Add(term, -1);
            Multiply(ee, WideArithmetic.SubtractSigned832(pp, radiusSquare), first, second, c);
            Multiply(ep, ep, first, second, term); c.Add(term, -1);
        }
        else
        {
            Signed832 nv = WideDirection3d.Dot(normal, velocity), np = WideDirection3d.Dot(normal, point);
            Multiply(nv, nv, first, second, a); Multiply(nv, np, first, second, b); b.MultiplySign(-1);
            Multiply(np, np, first, second, c);
            Multiply(normal.SquaredLength, radiusSquare, first, second, term); c.Add(term, -1);
        }
        distance = Fixed64.MaxValue; bool found = false;
        Span<ulong> root = stackalloc ulong[2 * words];
        for (int branch = 0; branch < 3; branch++)
        {
            if (branch == 0)
            {
                if (c.Signs[0] > 0) continue;
                numerator.Clear(); divisor.Set(Signed576.ExtendValue(Signed192.Signed(1))); root.Clear();
            }
            else if (!ContactQuadratic.TryGetParameter(a, b, c, branch == 1 ? -1 : 1, root, numerator, divisor)) continue;
            if (numerator.Sign(root) < 0) continue;
            divisor.CopyTo(work); work.Add(numerator, -1); if (work.Sign(root) < 0) continue;
            if (feature is >= 3 and < 6)
            {
                ContactQuadratic.Scale(numerator, ev, work); ContactQuadratic.Scale(divisor, ep, term); work.Add(term, -1);
                if (work.Sign(root) < 0) continue;
                ContactQuadratic.Scale(divisor, ee, term); term.Add(work, -1); if (term.Sign(root) < 0) continue;
            }
            else if (feature == 6)
            {
                bool admitted = true;
                WideDirection3d localNormal = GetScaledTriangleLocalNormal(triangle, frame);
                for (int edgeIndex = 0; edgeIndex < 3; edgeIndex++)
                {
                    FixedSegment authoredEdge = triangle.GetEdge(edgeIndex);
                    WideDirection3d inward = WideDirection3d.Cross(localNormal,
                        WideDirection3d.GetLocalEdge(frame, authoredEdge.Start, authoredEdge.End)).Transform(frame.GeometryBasis);
                    ContactQuadratic.Scale(numerator, WideDirection3d.Dot(inward, velocity), work);
                    ContactQuadratic.Scale(divisor, WideDirection3d.Dot(inward, points[edgeIndex]), term);
                    work.Add(term, -1); if (work.Sign(root) < 0) { admitted = false; break; }
                }
                if (!admitted) continue;
            }
            ContactQuadratic.Scale(numerator, Signed576.ExtendValue(Signed192.Raw(length)), parameterWork);
            Fixed64 candidate = ContactQuadratic.RoundRatio(parameterWork, divisor, root, 0, length.m_rawValue);
            if (candidate < distance) { distance = candidate; found = true; }
        }
        return found;
    }

    private static WideDirection3d Difference(in WideDirection3d first, in WideDirection3d second) =>
        new(WideArithmetic.SubtractSigned576(first.X, second.X), WideArithmetic.SubtractSigned576(first.Y, second.Y),
            WideArithmetic.SubtractSigned576(first.Z, second.Z));

    private static void Multiply(Signed832 firstValue, Signed832 secondValue,
        scoped ContactQuadratic first, scoped ContactQuadratic second, scoped ContactQuadratic result)
    {
        first.Set(firstValue); second.Set(secondValue);
        ContactQuadratic.Multiply(first, second, ReadOnlySpan<ulong>.Empty, result);
    }
}
