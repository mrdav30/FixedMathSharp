//=======================================================================
// TriangleCylinderRimWitnesses.cs
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
    internal static Vector3d GetAnalyticPoint(in TriangleCircularGeometry geometry, FixedTriangle triangle,
        int mask, ReadOnlySpan<ulong> normal, ReadOnlySpan<int> normalSigns, int cap, Signed576 radialRadius)
    {
        bool admitted = TryGetAnalyticPoint(geometry, triangle, mask, normal, normalSigns, cap, radialRadius,
            out Vector3d point, false);
        System.Diagnostics.Debug.Assert(admitted);
        return point;
    }

    internal static bool TryGetAnalyticPoint(in TriangleCircularGeometry geometry, FixedTriangle triangle,
        int mask, ReadOnlySpan<ulong> normal, ReadOnlySpan<int> normalSigns, int cap, Signed576 radialRadius,
        out Vector3d point, bool requireAdmission = true)
    {
        point = default;
        int first = (mask & 1) != 0 ? 0 : 1;
        if ((mask & (mask - 1)) == 0)
        {
            int vertex = (mask & 4) != 0 ? 2 : first;
            if (requireAdmission)
            {
                Span<ulong> vertexWeights = stackalloc ulong[6 * Words]; Span<int> vertexSigns = stackalloc int[4];
                vertexWeights.Clear(); vertexSigns.Clear(); vertexWeights[0] = 1;
                WideArithmetic.MultiplyMagnitudes(Slot(normal, 0), Slot(normal, 0), Slot(vertexWeights, 5));
                WideArithmetic.MultiplyMagnitudes(Slot(normal, 2), Slot(normal, 2), Slot(vertexWeights, 1));
                WideArithmetic.AddMagnitudeInto(Slot(vertexWeights, 1), Slot(vertexWeights, 5));
                Slot(vertexWeights, 1).Clear();
                if (!HasMatchingPoint(geometry, vertex, default, default, vertexWeights, vertexSigns,
                        normal, normalSigns, cap, radialRadius)) return false;
            }
            point = geometry.WitnessVertex(triangle, vertex);
            return true;
        }
        bool face = mask == 7;
        int second = face ? 1 : (mask & 4) != 0 ? 2 : 1;
        WideDirection3d e = geometry.EdgeFromTo(first, second);
        WideDirection3d f = face ? -geometry.Edge(2) : default;
        WideDirection3d p0 = new(default, cap > 0 ? WideArithmetic.Negate(geometry.HalfHeight) : geometry.HalfHeight, default);
        WideDirection3d offset = TriangleCircularGeometry.Subtract(p0, geometry.Vertex(first));
        Signed832 ee = e.SquaredLength, ff = f.SquaredLength, ef = WideDirection3d.Dot(e, f);
        Signed832 a = WideDirection3d.Dot(offset, e), b = WideDirection3d.Dot(offset, f);
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
        if (radialRadius.IsZero)
        {
            Slot(weights, 5).Clear(); Slot(weights, 5)[0] = 1;
        }
        Span<ulong> edgeRadical = stackalloc ulong[Words];
        Span<ulong> faceRadical = stackalloc ulong[Words];
        GetRadialProjection(radialRadius, e, radial, radialSigns, edgeRadical, out int edgeSign);
        if (face)
        {
            GetRadialProjection(radialRadius, f, radial, radialSigns, faceRadical, out int faceSign);
            Span<ulong> determinant = stackalloc ulong[Words], factor = stackalloc ulong[Words];
            CylinderContactAlgebra.DifferenceOfProducts(ee, ff, ef, ef, determinant, out _);
            CylinderContactAlgebra.Import(Signed576.ExtendValue(geometry.EdgeScale), factor);
            WideArithmetic.MultiplyMagnitudes(determinant, factor, Slot(weights, 0));
            CylinderContactAlgebra.DifferenceOfProducts(ff, a, ef, b, Slot(weights, 1), out signs[0]);
            CylinderContactAlgebra.DifferenceOfProducts(ee, b, ef, a, Slot(weights, 3), out signs[2]);
            Combine(ff, edgeRadical, edgeSign, ef, faceRadical, faceSign, Slot(weights, 2), out signs[1]);
            Combine(ee, faceRadical, faceSign, ef, edgeRadical, edgeSign, Slot(weights, 4), out signs[3]);
        }
        else
        {
            CylinderContactAlgebra.Import(WideArithmetic.MultiplySigned832(ee, geometry.EdgeScale), Slot(weights, 0));
            CylinderContactAlgebra.Import(a, Slot(weights, 1)); signs[0] = a.Sign;
            edgeRadical.CopyTo(Slot(weights, 2)); signs[1] = edgeSign;
        }
        if (requireAdmission && !HasMatchingPoint(geometry, first, e, f, weights, signs,
                normal, normalSigns, cap, radialRadius)) return false;
        Vector3d va = Vertex(triangle, first), vb = Vertex(triangle, second), vc = face ? triangle.C : va;
        // Exact feature admission proves this affine projection is on the
        // selected edge/face. No clamp of an already-rounded foot is needed.
        if (geometry.IsAffine)
            point = new Vector3d(
                RoundAnalyticCoordinate(geometry.WitnessCoordinate(triangle, first, 0), geometry.WitnessCoordinate(triangle, second, 0), geometry.WitnessCoordinate(triangle, face ? 2 : first, 0), geometry.RawScale, weights, signs),
                RoundAnalyticCoordinate(geometry.WitnessCoordinate(triangle, first, 1), geometry.WitnessCoordinate(triangle, second, 1), geometry.WitnessCoordinate(triangle, face ? 2 : first, 1), geometry.RawScale, weights, signs),
                RoundAnalyticCoordinate(geometry.WitnessCoordinate(triangle, first, 2), geometry.WitnessCoordinate(triangle, second, 2), geometry.WitnessCoordinate(triangle, face ? 2 : first, 2), geometry.RawScale, weights, signs));
        else point = new Vector3d(RoundAnalyticCoordinate(va.X, vb.X, vc.X, weights, signs),
            RoundAnalyticCoordinate(va.Y, vb.Y, vc.Y, weights, signs),
            RoundAnalyticCoordinate(va.Z, vb.Z, vc.Z, weights, signs));
        return true;
    }

    private static bool HasMatchingPoint(in TriangleCircularGeometry geometry, int first,
        in WideDirection3d edge, in WideDirection3d faceEdge, ReadOnlySpan<ulong> weights, ReadOnlySpan<int> signs,
        ReadOnlySpan<ulong> normal, ReadOnlySpan<int> normalSigns, int cap, Signed576 radius)
    {
        int words = geometry.IsAffine ? 80 : Words;
        Span<ulong> values = stackalloc ulong[16 * words]; Span<int> valueSigns = stackalloc int[16];
        ContactQuadratic beta = ContactQuadratic.At(values, valueSigns, 0, words),
            gamma = ContactQuadratic.At(values, valueSigns, 1, words), denominator = ContactQuadratic.At(values, valueSigns, 2, words),
            alpha = ContactQuadratic.At(values, valueSigns, 3, words), term = ContactQuadratic.At(values, valueSigns, 4, words),
            query = ContactQuadratic.At(values, valueSigns, 5, words), product = ContactQuadratic.At(values, valueSigns, 6, words),
            difference = ContactQuadratic.At(values, valueSigns, 7, words);
        values.Clear(); valueSigns.Clear();
        Slot(weights, 2).CopyTo(beta.Rational); Slot(weights, 1).CopyTo(beta.Radical);
        beta.Signs[0] = signs[1]; beta.Signs[1] = signs[0];
        Slot(weights, 4).CopyTo(gamma.Rational); Slot(weights, 3).CopyTo(gamma.Radical);
        gamma.Signs[0] = signs[3]; gamma.Signs[1] = signs[2];
        Slot(weights, 0).CopyTo(denominator.Radical); denominator.Signs[1] = 1;
        ReadOnlySpan<ulong> root = Slot(weights, 5);
        denominator.CopyTo(alpha); alpha.Add(beta, -1); alpha.Add(gamma, -1);
        if (alpha.Sign(root) < 0 || beta.Sign(root) < 0 || gamma.Sign(root) < 0) return false;
        Span<ulong> residualValues = stackalloc ulong[6 * words]; Span<int> residualSigns = stackalloc int[6];
        for (int component = 0; component < 3; component++)
        {
            ContactQuadratic residual = ContactQuadratic.At(residualValues, residualSigns, component, words);
            Signed576 offset = Component(geometry.Vertex(first), component);
            if (component == 1) offset = WideArithmetic.AddSigned576(offset, cap > 0 ? geometry.HalfHeight : WideArithmetic.Negate(geometry.HalfHeight));
            ContactQuadratic.Scale(denominator, offset, residual);
            ContactQuadratic.Scale(beta, Component(edge, component), term); residual.Add(term);
            ContactQuadratic.Scale(gamma, Component(faceEdge, component), term); residual.Add(term);
            if (component != 1)
            {
                query.Clear(); Slot(weights, 0).CopyTo(query.Rational); query.Signs[0] = 1;
                ContactQuadratic.Scale(query, radius, product);
                ContactQuadratic.Scale(product, Slot(normal, component), normalSigns[component], difference);
                residual.Add(difference);
            }
        }
        int axis = normalSigns[0] != 0 ? 0 : normalSigns[1] != 0 ? 1 : 2;
        for (int component = 0; component < 3; component++)
        {
            ContactQuadratic.Scale(ContactQuadratic.At(residualValues, residualSigns, component, words),
                Slot(normal, axis), normalSigns[axis], query);
            ContactQuadratic.Scale(ContactQuadratic.At(residualValues, residualSigns, axis, words),
                Slot(normal, component), normalSigns[component], product);
            query.Add(product, -1);
            if (query.Sign(root) != 0) return false;
        }
        return true;
    }

    internal static bool IsRootPointOnEdge(in TriangleCircularGeometry geometry, int edge,
        in WideDirection3d first, in WideDirection3d second, ref FiniteAxisValueRoot root)
    {
        WideDirection3d t0 = new(WideArithmetic.Negate(first.Z), default, first.X);
        WideDirection3d t1 = new(WideArithmetic.Negate(second.Z), default, second.X);
        Signed832 a0 = WideDirection3d.Dot(geometry.Vertex(edge), t0), a1 = WideDirection3d.Dot(geometry.Vertex(edge), t1);
        Signed832 b0 = WideDirection3d.Dot(geometry.Vertex((edge + 1) % 3), t0), b1 = WideDirection3d.Dot(geometry.Vertex((edge + 1) % 3), t1);
        Span<ulong> query = stackalloc ulong[26]; Span<sbyte> signs = stackalloc sbyte[2];
        int denominator = Sign(ref root, WideArithmetic.SubtractSigned832(b0, a0), WideArithmetic.SubtractSigned832(b1, a1), query, signs);
        return denominator != 0 && Sign(ref root, a0, a1, query, signs) * denominator <= 0
            && Sign(ref root, b0, b1, query, signs) * denominator >= 0;
    }

    internal static Vector3d GetRootPoint(in TriangleCircularGeometry geometry, FixedTriangle triangle,
        int edge, in WideDirection3d first, in WideDirection3d second, ref FiniteAxisValueRoot root)
    {
        int next = (edge + 1) % 3;
        WideDirection3d tangent0 = new(WideArithmetic.Negate(first.Z), default, first.X);
        WideDirection3d tangent1 = new(WideArithmetic.Negate(second.Z), default, second.X);
        Signed832 a0 = WideDirection3d.Dot(geometry.Vertex(edge), tangent0), a1 = WideDirection3d.Dot(geometry.Vertex(edge), tangent1);
        Signed832 b0 = WideDirection3d.Dot(geometry.Vertex(next), tangent0), b1 = WideDirection3d.Dot(geometry.Vertex(next), tangent1);
        Signed832 d0 = WideArithmetic.SubtractSigned832(b0, a0), d1 = WideArithmetic.SubtractSigned832(b1, a1);
        Span<ulong> query = stackalloc ulong[26];
        Span<sbyte> signs = stackalloc sbyte[2];
        int denominatorSign = Sign(ref root, d0, d1, query, signs);
        // q-p is parallel n, so q lies in the exact radial tangent plane.
        // A zero denominator would make this edge meridional: its stationary
        // normal is the earlier projected-Up principal direction and K=0.
        // Such a root cannot be an admitted K!=0 winner.
        System.Diagnostics.Debug.Assert(denominatorSign != 0);
        Vector3d aVertex = Vertex(triangle, edge), bVertex = Vertex(triangle, next);
        if (geometry.IsAffine)
            return new Vector3d(
                RoundRootCoordinate(geometry.WitnessCoordinate(triangle, edge, 0), geometry.WitnessCoordinate(triangle, next, 0), geometry.RawScale, a0, a1, b0, b1, d0, d1, denominatorSign, ref root),
                RoundRootCoordinate(geometry.WitnessCoordinate(triangle, edge, 1), geometry.WitnessCoordinate(triangle, next, 1), geometry.RawScale, a0, a1, b0, b1, d0, d1, denominatorSign, ref root),
                RoundRootCoordinate(geometry.WitnessCoordinate(triangle, edge, 2), geometry.WitnessCoordinate(triangle, next, 2), geometry.RawScale, a0, a1, b0, b1, d0, d1, denominatorSign, ref root));
        return new Vector3d(
            RoundRootCoordinate(aVertex.X, bVertex.X, a0, a1, b0, b1, d0, d1, denominatorSign, ref root),
            RoundRootCoordinate(aVertex.Y, bVertex.Y, a0, a1, b0, b1, d0, d1, denominatorSign, ref root),
            RoundRootCoordinate(aVertex.Z, bVertex.Z, a0, a1, b0, b1, d0, d1, denominatorSign, ref root));
    }

    internal static Fixed64 RoundRootCoordinate(Signed576 a, Signed576 b, Signed320 scale,
        Signed832 a0, Signed832 a1, Signed832 b0, Signed832 b1, Signed832 d0, Signed832 d1,
        int denominatorSign, ref FiniteAxisValueRoot root)
    {
        // Complete linear-ratio coefficients stay borrowed: physical coordinate
        // numerators need not fit a scalar or a thirteen-word product.
        Span<ulong> data = stackalloc ulong[4 * Words], temporary = stackalloc ulong[Words];
        Span<int> dataSigns = stackalloc int[4]; data.Clear(); dataSigns.Clear();
        Span<ulong> query = stackalloc ulong[2 * Words]; Span<sbyte> querySigns = stackalloc sbyte[2];
        for (int coefficient = 0; coefficient < 2; coefficient++)
        {
            Import(coefficient == 0 ? b0 : b1, temporary);
            Accumulate(a, temporary, (coefficient == 0 ? b0 : b1).Sign, Slot(data, coefficient), ref dataSigns[coefficient]);
            Import(coefficient == 0 ? a0 : a1, temporary);
            Accumulate(b, temporary, -(coefficient == 0 ? a0 : a1).Sign, Slot(data, coefficient), ref dataSigns[coefficient]);
            WideArithmetic.AddMagnitudeInto(Slot(data, coefficient), Slot(data, coefficient));
            Import(coefficient == 0 ? d0 : d1, temporary);
            Accumulate(Signed576.ExtendValue(scale), temporary, (coefficient == 0 ? d0 : d1).Sign,
                Slot(data, coefficient + 2), ref dataSigns[coefficient + 2]);
        }
        long ar = RoundEndpoint(a, scale), br = RoundEndpoint(b, scale);
        // This overload materializes admitted affine witnesses in the finite
        // shape's translated chart (reach <=16384), far from scalar endpoints.
        // The full-domain authored-coordinate overload retains its own bounds.
        long low = Math.Min(ar, br) - 1, high = Math.Max(ar, br) + 1;
        while (low < high)
        {
            ulong span = unchecked((ulong)high - (ulong)low);
            long midpoint = unchecked((long)((ulong)low + (span >> 1) + (span & 1)));
            Signed192 threshold = WideArithmetic.AddSigned192(Signed192.Signed(midpoint), Signed192.Signed(midpoint));
            int comparison = CompareRootRatio(data, dataSigns, threshold, query, querySigns, ref root) * denominatorSign;
            if (comparison >= 0) low = midpoint; else high = midpoint - 1;
        }
        Signed192 half = WideArithmetic.AddSigned192(WideArithmetic.AddSigned192(Signed192.Signed(low), Signed192.Signed(low)), Signed192.Signed(1));
        int halfSign = CompareRootRatio(data, dataSigns, half, query, querySigns, ref root) * denominatorSign;
        return Fixed64.FromRaw(low + (halfSign > 0 || halfSign == 0 && (low & 1) != 0 ? 1 : 0));
    }

    private static int CompareRootRatio(scoped ReadOnlySpan<ulong> data, scoped ReadOnlySpan<int> signs, Signed192 threshold,
        scoped Span<ulong> query, scoped Span<sbyte> querySigns, ref FiniteAxisValueRoot root)
    {
        for (int coefficient = 0; coefficient < 2; coefficient++)
        {
            Span<ulong> target = Slot(query, coefficient);
            Slot(data, coefficient).CopyTo(target); int sign = signs[coefficient];
            Accumulate(Signed576.ExtendValue(threshold), Slot(data, coefficient + 2), -signs[coefficient + 2], target, ref sign);
            querySigns[coefficient] = (sbyte)sign;
        }
        return WideFiniteAxisIntersection.GetSignAtFiniteValueRootAndRefine(ref root, query, querySigns);
    }

    internal static Fixed64 RoundRootCoordinate(Fixed64 a, Fixed64 b, Signed832 a0, Signed832 a1,
        Signed832 b0, Signed832 b1, Signed832 d0, Signed832 d1, int denominatorSign, ref FiniteAxisValueRoot root)
    {
        // Shifted tangent-distance coefficients <432 bits; authored-coordinate
        // numerators and midpoint queries <500, within Signed832. The
        // retained root sign authority rounds the linear ratio directly.
        Signed832 n0 = WideArithmetic.SubtractSigned832(WideArithmetic.MultiplySigned832(b0, Signed192.Raw(a)),
            WideArithmetic.MultiplySigned832(a0, Signed192.Raw(b)));
        Signed832 n1 = WideArithmetic.SubtractSigned832(WideArithmetic.MultiplySigned832(b1, Signed192.Raw(a)),
            WideArithmetic.MultiplySigned832(a1, Signed192.Raw(b)));
        n0 = WideArithmetic.AddSigned832(n0, n0); n1 = WideArithmetic.AddSigned832(n1, n1);
        Span<ulong> query = stackalloc ulong[26];
        Span<sbyte> signs = stackalloc sbyte[2];
        long low = Math.Min(a.m_rawValue, b.m_rawValue), high = Math.Max(a.m_rawValue, b.m_rawValue);
        while (low < high)
        {
            ulong span = unchecked((ulong)high - (ulong)low);
            long midpoint = unchecked((long)((ulong)low + (span >> 1) + (span & 1)));
            Signed192 threshold = WideArithmetic.AddSigned192(Signed192.Signed(midpoint), Signed192.Signed(midpoint));
            int comparison = Sign(ref root, WideArithmetic.SubtractSigned832(n0, WideArithmetic.MultiplySigned832(d0, threshold)),
                WideArithmetic.SubtractSigned832(n1, WideArithmetic.MultiplySigned832(d1, threshold)), query, signs) * denominatorSign;
            if (comparison >= 0) low = midpoint;
            else high = midpoint - 1;
        }
        Signed192 half = WideArithmetic.AddSigned192(WideArithmetic.AddSigned192(Signed192.Signed(low), Signed192.Signed(low)), Signed192.Signed(1));
        int halfSign = Sign(ref root, WideArithmetic.SubtractSigned832(n0, WideArithmetic.MultiplySigned832(d0, half)),
            WideArithmetic.SubtractSigned832(n1, WideArithmetic.MultiplySigned832(d1, half)), query, signs) * denominatorSign;
        return Fixed64.FromRaw(low + (halfSign > 0 || halfSign == 0 && (low & 1) != 0 ? 1 : 0));
    }

    private static int Sign(ref FiniteAxisValueRoot root, Signed832 constant, Signed832 linear,
        scoped Span<ulong> query, scoped Span<sbyte> signs)
    {
        WideArithmetic.GetMagnitude(constant, query[..13]); signs[0] = (sbyte)constant.Sign;
        WideArithmetic.GetMagnitude(linear, query[13..]); signs[1] = (sbyte)linear.Sign;
        return WideFiniteAxisIntersection.GetSignAtFiniteValueRootAndRefine(ref root, query, signs);
    }

    private static Fixed64 RoundAnalyticCoordinate(Fixed64 a, Fixed64 b, Fixed64 c,
        ReadOnlySpan<ulong> weights, ReadOnlySpan<int> signs) => RoundAnalyticCoordinate(
            Signed576.ExtendValue(Signed192.Raw(a)), Signed576.ExtendValue(Signed192.Raw(b)),
            Signed576.ExtendValue(Signed192.Raw(c)), Signed320.ExtendValue(Signed192.Signed(1)), weights, signs,
            Math.Min(a.m_rawValue, Math.Min(b.m_rawValue, c.m_rawValue)),
            Math.Max(a.m_rawValue, Math.Max(b.m_rawValue, c.m_rawValue)), hasScalarBounds: true);

    private static Fixed64 RoundAnalyticCoordinate(Signed576 a, Signed576 b, Signed576 c, Signed320 scale,
        ReadOnlySpan<ulong> weights, ReadOnlySpan<int> signs,
        long low = 0, long high = 0, bool hasScalarBounds = false)
    {
        Span<ulong> rational = stackalloc ulong[Words];
        Span<ulong> radical = stackalloc ulong[Words];
        rational.Clear(); radical.Clear(); int rationalSign = 0, radicalSign = 0;
        Accumulate(a, Slot(weights, 0), 1, rational, ref rationalSign);
        Signed576 e = WideArithmetic.SubtractSigned576(b, a);
        Signed576 f = WideArithmetic.SubtractSigned576(c, a);
        Accumulate(e, Slot(weights, 1), signs[0], rational, ref rationalSign);
        Accumulate(f, Slot(weights, 3), signs[2], rational, ref rationalSign);
        Accumulate(e, Slot(weights, 2), signs[1], radical, ref radicalSign);
        Accumulate(f, Slot(weights, 4), signs[3], radical, ref radicalSign);
        ReadOnlySpan<ulong> k = Slot(weights, 5);
        // The scalar bridge already owns exact raw endpoints and unit scale.
        // Borrow its existing denominator; dividing every endpoint by one or
        // multiplying this denominator by one only repeats known information.
        Span<ulong> scaledDenominator = stackalloc ulong[hasScalarBounds ? 0 : Words];
        scoped ReadOnlySpan<ulong> denominator = Slot(weights, 0);
        if (!hasScalarBounds)
        {
            Span<ulong> scaleMagnitude = stackalloc ulong[5];
            WideArithmetic.GetMagnitude(scale, out scaleMagnitude[4], out scaleMagnitude[3], out scaleMagnitude[2], out scaleMagnitude[1], out scaleMagnitude[0]);
            WideArithmetic.MultiplyMagnitudes(denominator, scaleMagnitude, scaledDenominator);
            denominator = scaledDenominator;
        }
        if (radicalSign == 0)
        {
            // Apex projections and exact cancellation are rational A/D.
            // Feature admission bounds A/D within the authored coordinates;
            // the existing raw-ratio owner performs the same nearest-even rounding.
            bool represented = Fixed64.TryGetSignedRawRatio(rational, denominator, rationalSign < 0, out Fixed64 result);
            System.Diagnostics.Debug.Assert(represented);
            return result;
        }
        WideArithmetic.AddMagnitudeInto(radical, radical);
        // Compare x=(A+B/sqrt(K))/D with t/2 by the sign of
        // 2B+(2A-tD)*sqrt(K). Shifted directions <630 bits, K<1262 and Gram
        // minors<827. A/B remain <895/<1535 bits, including raw midpoint
        // products. Forty words suffice; the shared sign owner sizes squares.
        if (!hasScalarBounds)
        {
            long ar = RoundEndpoint(a, scale), br = RoundEndpoint(b, scale), cr = RoundEndpoint(c, scale);
            low = Math.Min(ar, Math.Min(br, cr)); high = Math.Max(ar, Math.Max(br, cr));
        }
        if (low != long.MinValue) low--;
        if (high != long.MaxValue) high++;
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
        CylinderContactAlgebra.Import(Signed576.ExtendValue(threshold), value);
        WideArithmetic.MultiplyMagnitudes(value, denominator, product);
        Add(product, -threshold.Sign, query, ref rationalSign);
        return WideConvexPrismRelations.GetConvexContactCandidateQuadraticSign(radical, radicalSign, query, rationalSign, k);
    }

    private static void GetRadialProjection(Signed576 radius, in WideDirection3d edge,
        ReadOnlySpan<ulong> radial, ReadOnlySpan<int> signs, Span<ulong> result, out int sign)
    {
        Span<ulong> dot = stackalloc ulong[Words];
        Span<ulong> magnitude = stackalloc ulong[Words];
        Dot(edge, radial, signs, dot, out sign);
        CylinderContactAlgebra.Import(radius, magnitude);
        // Apex witnesses use zero radius: its product must also have zero sign
        // before combining weights, even when the unscaled projection is nonzero.
        WideArithmetic.MultiplyMagnitudes(dot, magnitude, result); sign = -sign * radius.Sign;
    }

    private static void Combine(Signed832 a, ReadOnlySpan<ulong> first, int firstSign,
        Signed832 b, ReadOnlySpan<ulong> second, int secondSign, Span<ulong> result, out int sign)
    {
        Span<ulong> factor = stackalloc ulong[Words];
        Span<ulong> product = stackalloc ulong[Words];
        CylinderContactAlgebra.Import(a, factor);
        WideArithmetic.MultiplyMagnitudes(factor, first, result); sign = a.Sign * firstSign;
        CylinderContactAlgebra.Import(b, factor);
        WideArithmetic.MultiplyMagnitudes(factor, second, product);
        Add(product, -b.Sign * secondSign, result, ref sign);
    }

    private static void Accumulate(Signed576 scalar, ReadOnlySpan<ulong> value, int sign,
        Span<ulong> sum, ref int sumSign)
    {
        Span<ulong> factor = stackalloc ulong[Words];
        Span<ulong> product = stackalloc ulong[Words];
        CylinderContactAlgebra.Import(scalar, factor);
        WideArithmetic.MultiplyMagnitudes(factor, value, product);
        Add(product, sign * scalar.Sign, sum, ref sumSign);
    }
    private static long RoundEndpoint(Signed576 numerator, Signed320 denominator)
    {
        Span<ulong> n = stackalloc ulong[9], d = stackalloc ulong[5];
        WideArithmetic.GetMagnitude(numerator, n);
        WideArithmetic.GetMagnitude(denominator, out d[4], out d[3], out d[2], out d[1], out d[0]);
        bool represented = Fixed64.TryGetSignedRawRatio(n, d, numerator.Sign < 0, out Fixed64 value);
        System.Diagnostics.Debug.Assert(represented);
        return value.m_rawValue;
    }

    private static Vector3d Vertex(FixedTriangle triangle, int index) => index == 0 ? triangle.A : index == 1 ? triangle.B : triangle.C;
}
