//=======================================================================
// WidePlanarProjection.ScaledTriangle.cs
//=======================================================================
// MIT License, Copyright (c) 2024-present David Oravsky (mrdav30)
// See LICENSE file in the project root for full license information.
//=======================================================================
using System;

namespace FixedMathSharp.Geometry;

/// <content>Retains exact scaled source triangles through projected finite-feature selection.</content>
internal static partial class WidePlanarProjection
{
    internal static bool TryGetTriangleRelation(Vector2d center, Fixed64 radius, in FixedTriangle triangle,
        in ScaledRigidFrame3d frame, out PlanarProjectionRelation relation)
    {
        if (frame.IsRigid)
            return TryGetTriangleRelation(center, radius, triangle, frame.RigidOrigin, frame.Rotation, out relation);
        Span<RationalPoint3d> points = stackalloc RationalPoint3d[3]
            { frame.GetPoint(triangle.A), frame.GetPoint(triangle.B), frame.GetPoint(triangle.C) };
        Span<Signed320> x = stackalloc Signed320[3], z = stackalloc Signed320[3];
        for (int corner = 0; corner < 3; corner++)
        {
            x[corner] = Signed320.NarrowValue(WideArithmetic.SubtractSigned576(Signed576.ExtendValue(points[corner].X),
                WideArithmetic.MultiplySigned320(frame.Denominator, Signed192.Raw(center.X))));
            z[corner] = Signed320.NarrowValue(WideArithmetic.SubtractSigned576(Signed576.ExtendValue(points[corner].Z),
                WideArithmetic.MultiplySigned320(frame.Denominator, Signed192.Raw(center.Y))));
        }
        int winding = 0; bool contained = true;
        for (int edge = 0; edge < 3; edge++)
        {
            int end = (edge + 1) % 3;
            Signed576 cross = WideArithmetic.SubtractSigned576(WideArithmetic.MultiplySigned320(x[edge], z[end]),
                WideArithmetic.MultiplySigned320(z[edge], x[end]));
            if (cross.IsZero) continue;
            if (winding == 0) winding = cross.Sign;
            else if (winding != cross.Sign) contained = false;
        }
        if (contained && winding != 0) { relation = PlanarProjectionRelation.Contained; return true; }

        // Endpoint offsets need <263 bits. An interior edge projection retains
        // <750-bit numerators and its squared distance in transient 2048-bit banks.
        const int words = 32;
        Span<ulong> storage = stackalloc ulong[16 * words]; Span<int> signs = stackalloc int[16];
        var bestN = ContactQuadratic.At(storage, signs, 0, words);
        var bestD = ContactQuadratic.At(storage, signs, 1, words);
        var distance = ContactQuadratic.At(storage, signs, 2, words);
        var denominator = ContactQuadratic.At(storage, signs, 3, words);
        var coefficient = ContactQuadratic.At(storage, signs, 4, words);
        var term = ContactQuadratic.At(storage, signs, 5, words);
        var limit = ContactQuadratic.At(storage, signs, 6, words);
        var scaled = ContactQuadratic.At(storage, signs, 7, words);
        Signed832 bestX = default, bestZ = default, bestDenominator = default;
        for (int edge = 0; edge < 3; edge++)
        {
            int end = (edge + 1) % 3;
            GetScaledProjectedDistance(x[edge], z[edge], x[end], z[end], frame.Denominator,
                out Signed832 dx, out Signed832 dz, out Signed832 divisor);
            coefficient.Set(dx); ContactQuadratic.Multiply(coefficient, coefficient, ReadOnlySpan<ulong>.Empty, distance);
            coefficient.Set(dz); ContactQuadratic.Multiply(coefficient, coefficient, ReadOnlySpan<ulong>.Empty, term); distance.Add(term);
            coefficient.Set(divisor); ContactQuadratic.Multiply(coefficient, coefficient, ReadOnlySpan<ulong>.Empty, denominator);
            if (edge != 0 && ContactQuadratic.CompareRatios(distance, denominator, ReadOnlySpan<ulong>.Empty,
                bestN, bestD, ReadOnlySpan<ulong>.Empty) >= 0) continue;
            distance.CopyTo(bestN); denominator.CopyTo(bestD);
            bestX = dx; bestZ = dz; bestDenominator = divisor;
        }
        if (bestN.Signs[0] == 0) { relation = PlanarProjectionRelation.Contained; return true; }
        Signed576 rawRadius = Signed576.ExtendValue(Signed320.ExtendValue(Signed192.Raw(radius)));
        ContactQuadratic.Scale(bestD, rawRadius, limit); ContactQuadratic.Scale(limit, rawRadius, scaled);
        if (ContactQuadratic.CompareRatios(bestN, bestD, ReadOnlySpan<ulong>.Empty,
                scaled, bestD, ReadOnlySpan<ulong>.Empty) > 0)
        { relation = default; return false; }
        ulong floor = WideArithmetic.GetRatioFloorSquareRoot(bestN.Rational, bestD.Rational, (ulong)radius.m_rawValue);
        Signed192 midpoint = WideArithmetic.AddSigned192(WideArithmetic.AddSigned192(Signed192.Signed((long)floor),
            Signed192.Signed((long)floor)), Signed192.Signed(1));
        Signed576 rawMidpoint = Signed576.ExtendValue(Signed320.ExtendValue(midpoint));
        ContactQuadratic.Scale(bestD, rawMidpoint, limit); ContactQuadratic.Scale(limit, rawMidpoint, scaled);
        ContactQuadratic.Scale(bestN, Signed576.ExtendValue(Signed320.ExtendValue(Signed192.Signed(4))), limit);
        scaled.Add(limit, -1);
        if (scaled.Signs[0] < 0 || scaled.Signs[0] == 0 && (floor & 1UL) != 0) floor++;
        _ = Fixed64.TryGetSignedRawRatio(bestX, bestDenominator, 0, out Fixed64 offsetX);
        _ = Fixed64.TryGetSignedRawRatio(bestZ, bestDenominator, 0, out Fixed64 offsetZ);
        relation = new PlanarProjectionRelation(Fixed64.FromRaw((long)floor), new Vector2d(offsetX, offsetZ),
            WideNormalization.GetNormalized(bestX, bestZ));
        return true;
    }

    private static void GetScaledProjectedDistance(Signed320 ax, Signed320 az, Signed320 bx, Signed320 bz,
        Signed320 denominator, out Signed832 x, out Signed832 z, out Signed832 divisor)
    {
        Signed320 ex = WideArithmetic.SubtractSigned320(bx, ax), ez = WideArithmetic.SubtractSigned320(bz, az);
        Signed576 squared = WideArithmetic.AddSigned576(WideArithmetic.MultiplySigned320(ex, ex), WideArithmetic.MultiplySigned320(ez, ez));
        Signed576 projection = WideArithmetic.SubtractSigned576(default,
            WideArithmetic.AddSigned576(WideArithmetic.MultiplySigned320(ax, ex), WideArithmetic.MultiplySigned320(az, ez)));
        if (squared.IsZero || projection.Sign <= 0)
        {
            x = Signed832.ExtendValue(Signed576.ExtendValue(ax)); z = Signed832.ExtendValue(Signed576.ExtendValue(az));
            divisor = Signed832.ExtendValue(Signed576.ExtendValue(denominator)); return;
        }
        if (WideArithmetic.SubtractSigned576(projection, squared).Sign >= 0)
        {
            x = Signed832.ExtendValue(Signed576.ExtendValue(bx)); z = Signed832.ExtendValue(Signed576.ExtendValue(bz));
            divisor = Signed832.ExtendValue(Signed576.ExtendValue(denominator)); return;
        }
        x = WideArithmetic.AddSigned832(WideArithmetic.MultiplySigned576ToSigned832(squared, ax),
            WideArithmetic.MultiplySigned576ToSigned832(projection, ex));
        z = WideArithmetic.AddSigned832(WideArithmetic.MultiplySigned576ToSigned832(squared, az),
            WideArithmetic.MultiplySigned576ToSigned832(projection, ez));
        divisor = WideArithmetic.MultiplySigned576ToSigned832(squared, denominator);
    }
}
