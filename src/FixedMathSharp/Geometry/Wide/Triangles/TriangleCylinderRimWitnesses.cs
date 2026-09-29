//=======================================================================
// MIT License, Copyright (c) 2024-present David Oravsky (mrdav30)
// See LICENSE file in the project root for full license information.
//=======================================================================
using System;
using static FixedMathSharp.Geometry.CylinderContactAlgebra;

namespace FixedMathSharp.Geometry;

/// <summary>Selected rim witnesses before either support point is rounded.</summary>
internal static class TriangleCylinderRimWitnesses
{
    internal static Vector3d GetAnalyticPoint(in TriangleCylinderGeometry geometry, FixedTriangle triangle,
        int mask, ReadOnlySpan<ulong> normal, ReadOnlySpan<int> normalSigns, int cap)
    {
        int first = (mask & 1) != 0 ? 0 : 1;
        if ((mask & (mask - 1)) == 0)
            return Vertex(triangle, (mask & 4) != 0 ? 2 : first);
        bool face = mask == 7;
        int second = face ? 1 : (mask & 4) != 0 ? 2 : 1;
        WideAxis3 e = geometry.EdgeFromTo(first, second);
        WideAxis3 f = face ? -geometry.Edge(2) : default;
        WideAxis3 p0 = new(default, cap > 0 ? WideArithmetic.Negate(geometry.HalfHeight) : geometry.HalfHeight, default);
        WideAxis3 offset = TriangleCylinderGeometry.Subtract(p0, geometry.Vertex(first));
        Signed576 ee = e.SquaredLength, ff = f.SquaredLength, ef = WideAxis3.Dot(e, f);
        Signed576 a = WideAxis3.Dot(offset, e), b = WideAxis3.Dot(offset, f);
        Span<ulong> radial = stackalloc ulong[3 * Words];
        Span<int> radialSigns = stackalloc int[3];
        normal.CopyTo(radial); normalSigns.CopyTo(radialSigns);
        Slot(radial, 1).Clear(); radialSigns[1] = 0;
        Span<ulong> weights = stackalloc ulong[6 * Words];
        Span<int> signs = stackalloc int[4];
        weights.Clear(); signs.Clear();
        // Slots: denominator, edge rational/radical, face rational/radical,
        // radial norm squared. Coordinates are (A+B/sqrt(K))/D.
        SumSquares(radial, Slot(weights, 5));
        Span<ulong> edgeRadical = stackalloc ulong[Words];
        Span<ulong> faceRadical = stackalloc ulong[Words];
        GetRadialProjection(geometry.Radius, e, radial, radialSigns, edgeRadical, out int edgeSign);
        if (face)
        {
            GetRadialProjection(geometry.Radius, f, radial, radialSigns, faceRadical, out int faceSign);
            Import(WideArithmetic.MultiplySigned832(DifferenceOfProducts(ee, ff, ef, ef), geometry.EdgeScale), Slot(weights, 0));
            Signed832 firstWeight = DifferenceOfProducts(ff, a, ef, b);
            Signed832 secondWeight = DifferenceOfProducts(ee, b, ef, a);
            Import(firstWeight, Slot(weights, 1)); signs[0] = firstWeight.Sign;
            Import(secondWeight, Slot(weights, 3)); signs[2] = secondWeight.Sign;
            Combine(ff, edgeRadical, edgeSign, ef, faceRadical, faceSign, Slot(weights, 2), out signs[1]);
            Combine(ee, faceRadical, faceSign, ef, edgeRadical, edgeSign, Slot(weights, 4), out signs[3]);
        }
        else
        {
            CylinderContactAlgebra.Import(WideArithmetic.MultiplySigned576(ee, geometry.EdgeScale), Slot(weights, 0));
            CylinderContactAlgebra.Import(a, Slot(weights, 1)); signs[0] = a.Sign;
            edgeRadical.CopyTo(Slot(weights, 2)); signs[1] = edgeSign;
        }
        Vector3d va = Vertex(triangle, first), vb = Vertex(triangle, second), vc = face ? triangle.C : va;
        // Exact feature admission proves this affine projection is on the
        // selected edge/face. No clamp of an already-rounded foot is needed.
        return new Vector3d(RoundAnalyticCoordinate(va.X, vb.X, vc.X, weights, signs),
            RoundAnalyticCoordinate(va.Y, vb.Y, vc.Y, weights, signs),
            RoundAnalyticCoordinate(va.Z, vb.Z, vc.Z, weights, signs));
    }

    internal static Vector3d GetRootPoint(in TriangleCylinderGeometry geometry, FixedTriangle triangle,
        int edge, WideAxis3 first, WideAxis3 second, ref FiniteAxisValueRoot root)
    {
        int next = (edge + 1) % 3;
        WideAxis3 tangent0 = new(WideArithmetic.Negate(first.Z), default, first.X);
        WideAxis3 tangent1 = new(WideArithmetic.Negate(second.Z), default, second.X);
        Signed576 a0 = WideAxis3.Dot(geometry.Vertex(edge), tangent0), a1 = WideAxis3.Dot(geometry.Vertex(edge), tangent1);
        Signed576 b0 = WideAxis3.Dot(geometry.Vertex(next), tangent0), b1 = WideAxis3.Dot(geometry.Vertex(next), tangent1);
        Signed576 d0 = WideArithmetic.SubtractSigned576(b0, a0), d1 = WideArithmetic.SubtractSigned576(b1, a1);
        Span<ulong> query = stackalloc ulong[18];
        Span<sbyte> signs = stackalloc sbyte[2];
        int denominatorSign = Sign(ref root, d0, d1, query, signs);
        // q-p is parallel n, so q lies in the exact radial tangent plane.
        // A zero denominator would make this edge meridional: its stationary
        // normal is the earlier projected-Up principal direction and K=0.
        // Such a root cannot be an admitted K!=0 winner.
        System.Diagnostics.Debug.Assert(denominatorSign != 0);
        Vector3d aVertex = Vertex(triangle, edge), bVertex = Vertex(triangle, next);
        return new Vector3d(
            RoundRootCoordinate(aVertex.X, bVertex.X, a0, a1, b0, b1, d0, d1, denominatorSign, ref root),
            RoundRootCoordinate(aVertex.Y, bVertex.Y, a0, a1, b0, b1, d0, d1, denominatorSign, ref root),
            RoundRootCoordinate(aVertex.Z, bVertex.Z, a0, a1, b0, b1, d0, d1, denominatorSign, ref root));
    }

    private static Fixed64 RoundRootCoordinate(Fixed64 a, Fixed64 b, Signed576 a0, Signed576 a1,
        Signed576 b0, Signed576 b1, Signed576 d0, Signed576 d1, int denominatorSign, ref FiniteAxisValueRoot root)
    {
        // Shifted tangent-distance coefficients <432 bits; authored-coordinate
        // numerators and midpoint queries <500, within Signed576. The
        // retained root sign authority rounds the linear ratio directly.
        Signed576 n0 = WideArithmetic.SubtractSigned576(WideArithmetic.MultiplySigned576(b0, a.m_rawValue),
            WideArithmetic.MultiplySigned576(a0, b.m_rawValue));
        Signed576 n1 = WideArithmetic.SubtractSigned576(WideArithmetic.MultiplySigned576(b1, a.m_rawValue),
            WideArithmetic.MultiplySigned576(a1, b.m_rawValue));
        n0 = WideArithmetic.AddSigned576(n0, n0); n1 = WideArithmetic.AddSigned576(n1, n1);
        Span<ulong> query = stackalloc ulong[18];
        Span<sbyte> signs = stackalloc sbyte[2];
        long low = Math.Min(a.m_rawValue, b.m_rawValue), high = Math.Max(a.m_rawValue, b.m_rawValue);
        while (low < high)
        {
            ulong span = unchecked((ulong)high - (ulong)low);
            long midpoint = unchecked((long)((ulong)low + (span >> 1) + (span & 1)));
            Signed192 threshold = WideArithmetic.AddSigned192(Signed192.Signed(midpoint), Signed192.Signed(midpoint));
            int comparison = Sign(ref root, WideArithmetic.SubtractSigned576(n0, WideArithmetic.MultiplySigned576(d0, threshold)),
                WideArithmetic.SubtractSigned576(n1, WideArithmetic.MultiplySigned576(d1, threshold)), query, signs) * denominatorSign;
            if (comparison >= 0) low = midpoint;
            else high = midpoint - 1;
        }
        Signed192 half = WideArithmetic.AddSigned192(WideArithmetic.AddSigned192(Signed192.Signed(low), Signed192.Signed(low)), Signed192.Signed(1));
        int halfSign = Sign(ref root, WideArithmetic.SubtractSigned576(n0, WideArithmetic.MultiplySigned576(d0, half)),
            WideArithmetic.SubtractSigned576(n1, WideArithmetic.MultiplySigned576(d1, half)), query, signs) * denominatorSign;
        return Fixed64.FromRaw(low + (halfSign > 0 || halfSign == 0 && (low & 1) != 0 ? 1 : 0));
    }

    private static int Sign(ref FiniteAxisValueRoot root, Signed576 constant, Signed576 linear,
        scoped Span<ulong> query, scoped Span<sbyte> signs)
    {
        WideArithmetic.GetMagnitude(constant, query[..9]); signs[0] = (sbyte)constant.Sign;
        WideArithmetic.GetMagnitude(linear, query[9..]); signs[1] = (sbyte)linear.Sign;
        return WideFiniteAxisIntersection.GetSignAtFiniteValueRootAndRefine(ref root, query, signs);
    }

    private static Fixed64 RoundAnalyticCoordinate(Fixed64 a, Fixed64 b, Fixed64 c,
        ReadOnlySpan<ulong> weights, ReadOnlySpan<int> signs)
    {
        Span<ulong> rational = stackalloc ulong[Words];
        Span<ulong> radical = stackalloc ulong[Words];
        rational.Clear(); radical.Clear(); int rationalSign = 0, radicalSign = 0;
        Accumulate(Signed192.Raw(a), Slot(weights, 0), 1, rational, ref rationalSign);
        Signed192 e = WideArithmetic.SubtractSigned192(Signed192.Raw(b), Signed192.Raw(a));
        Signed192 f = WideArithmetic.SubtractSigned192(Signed192.Raw(c), Signed192.Raw(a));
        Accumulate(e, Slot(weights, 1), signs[0], rational, ref rationalSign);
        Accumulate(f, Slot(weights, 3), signs[2], rational, ref rationalSign);
        Accumulate(e, Slot(weights, 2), signs[1], radical, ref radicalSign);
        Accumulate(f, Slot(weights, 4), signs[3], radical, ref radicalSign);
        ReadOnlySpan<ulong> k = Slot(weights, 5);
        ReadOnlySpan<ulong> denominator = Slot(weights, 0);
        WideArithmetic.AddMagnitudeInto(radical, radical);
        // Compare x=(A+B/sqrt(K))/D with t/2 by the sign of
        // 2B+(2A-tD)*sqrt(K). Shifted directions <630 bits, K<1262 and Gram
        // minors<827. A/B remain <895/<1535 bits, including raw midpoint
        // products. Forty words suffice; the shared sign owner sizes squares.
        long low = Math.Min(a.m_rawValue, Math.Min(b.m_rawValue, c.m_rawValue));
        long high = Math.Max(a.m_rawValue, Math.Max(b.m_rawValue, c.m_rawValue));
        while (low < high)
        {
            ulong span = unchecked((ulong)high - (ulong)low);
            long midpoint = unchecked((long)((ulong)low + (span >> 1) + (span & 1)));
            Signed192 threshold = WideArithmetic.AddSigned192(Signed192.Signed(midpoint), Signed192.Signed(midpoint));
            if (Compare(rational, rationalSign, radical, radicalSign, denominator, k, threshold) >= 0) low = midpoint;
            else high = midpoint - 1;
        }
        Signed192 half = WideArithmetic.AddSigned192(WideArithmetic.AddSigned192(Signed192.Signed(low), Signed192.Signed(low)), Signed192.Signed(1));
        int comparison = Compare(rational, rationalSign, radical, radicalSign, denominator, k, half);
        return Fixed64.FromRaw(low + (comparison > 0 || comparison == 0 && (low & 1) != 0 ? 1 : 0));
    }

    private static int Compare(ReadOnlySpan<ulong> rational, int rationalSign, ReadOnlySpan<ulong> radical,
        int radicalSign, ReadOnlySpan<ulong> denominator, ReadOnlySpan<ulong> k, Signed192 threshold)
    {
        Span<ulong> query = stackalloc ulong[Words];
        Span<ulong> value = stackalloc ulong[Words];
        Span<ulong> product = stackalloc ulong[Words];
        WideArithmetic.AddEqualMagnitudes(rational, rational, query);
        CylinderContactAlgebra.Import(Signed320.ExtendValue(threshold), value);
        WideArithmetic.MultiplyMagnitudes(value, denominator, product);
        Add(product, -threshold.Sign, query, ref rationalSign);
        return WideConvexPrismRelations.GetConvexContactCandidateQuadraticSign(radical, radicalSign, query, rationalSign, k);
    }

    private static void GetRadialProjection(Signed320 radius, WideAxis3 edge,
        ReadOnlySpan<ulong> radial, ReadOnlySpan<int> signs, Span<ulong> result, out int sign)
    {
        Span<ulong> dot = stackalloc ulong[Words];
        Span<ulong> magnitude = stackalloc ulong[Words];
        Dot(edge, radial, signs, dot, out sign);
        CylinderContactAlgebra.Import(radius, magnitude);
        WideArithmetic.MultiplyMagnitudes(dot, magnitude, result); sign = -sign;
    }

    private static void Combine(Signed576 a, ReadOnlySpan<ulong> first, int firstSign,
        Signed576 b, ReadOnlySpan<ulong> second, int secondSign, Span<ulong> result, out int sign)
    {
        Span<ulong> factor = stackalloc ulong[Words];
        Span<ulong> product = stackalloc ulong[Words];
        CylinderContactAlgebra.Import(a, factor);
        WideArithmetic.MultiplyMagnitudes(factor, first, result); sign = a.Sign * firstSign;
        CylinderContactAlgebra.Import(b, factor);
        WideArithmetic.MultiplyMagnitudes(factor, second, product);
        Add(product, -b.Sign * secondSign, result, ref sign);
    }

    private static void Accumulate(Signed192 scalar, ReadOnlySpan<ulong> value, int sign,
        Span<ulong> sum, ref int sumSign)
    {
        Span<ulong> factor = stackalloc ulong[Words];
        Span<ulong> product = stackalloc ulong[Words];
        CylinderContactAlgebra.Import(Signed320.ExtendValue(scalar), factor);
        WideArithmetic.MultiplyMagnitudes(factor, value, product);
        Add(product, sign * scalar.Sign, sum, ref sumSign);
    }
    private static Signed832 DifferenceOfProducts(Signed576 a, Signed576 b, Signed576 c, Signed576 d) =>
        WideArithmetic.SubtractSigned832(WideArithmetic.MultiplySigned576ToSigned832(a, b), WideArithmetic.MultiplySigned576ToSigned832(c, d));
    private static void Import(Signed832 value, Span<ulong> result)
    {
        result.Clear(); WideArithmetic.GetMagnitude(value, result[..13]);
    }
    private static Vector3d Vertex(FixedTriangle triangle, int index) => index == 0 ? triangle.A : index == 1 ? triangle.B : triangle.C;
}
