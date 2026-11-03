//=======================================================================
// CurvedPrismGeneratorFeatures.cs
//=======================================================================
// MIT License, Copyright (c) 2024-present David Oravsky (mrdav30)
// See LICENSE file in the project root for full license information.
//=======================================================================

using System;
using static FixedMathSharp.Geometry.CylinderContactAlgebra;

namespace FixedMathSharp.Geometry;

/// <summary>Exact cone-generator normal-circle extrema against the complete prism.</summary>
internal static class CurvedPrismGeneratorFeatures
{
    internal static void KeepAll(in CurvedPrismContactGeometry geometry,
        ref CurvedPrismContactSelection selection)
    {
        // The cone has apex H=T*A and base center -H, radius R. With q=A²,
        // its generator normals satisfy 2T*A.n=R*sqrt(n²-(A.n)²/q)>=0.
        // A vertex patch has two azimuthal extrema; changes of the prism's
        // supporting vertex occur only at its real edges. These extrema and
        // edge intersections exhaust the piecewise-linear generator circle.
        // Accepted raw axes have |A_i|<2^33 and q<2^65. Primitive-axis
        // reduction may give T<2^161, but T²*q retains its original <2^321 bound;
        // R<2^163, |P_i|<2^165 and |E_i|<2^166. Thus V<2^234,
        // d<2^202, k<2^400, U<2^370, C<2^200 and |Δ|<2^730.
        // All narrowed vectors/products fit 576 bits; k² and qΔ fit 832.
        // Normal coefficients stay below 600 bits, squared-gap coefficients
        // below 1600 and the final denominator below 1400, within forty words.
        Signed320 q = geometry.SquaredAxis;
        Signed576 tSquared = WideArithmetic.MultiplySigned320(Signed320.ExtendValue(geometry.HalfFactor), geometry.HalfFactor);
        Signed576 fourTq = Signed576.NarrowValue(WideArithmetic.MultiplySigned576ToSigned832(tSquared, q));
        fourTq = WideArithmetic.AddSigned576(fourTq, fourTq);
        fourTq = WideArithmetic.AddSigned576(fourTq, fourTq);
        Signed576 radiusSquared = WideArithmetic.MultiplySigned320(geometry.Radius, geometry.Radius);
        Signed576 length = WideArithmetic.AddSigned576(fourTq, radiusSquared);
        if (selection.HasValue && !selection.Domain.Enabled &&
            CanExcludeCircle(geometry, selection.Candidate, radiusSquared, length)) return;

        Span<ulong> values = stackalloc ulong[ConvexContactCandidate.Slots * Words];
        Span<int> signs = stackalloc int[ConvexContactCandidate.Slots];
        Span<ulong> work = stackalloc ulong[4 * Words];
        Span<ulong> factor = Slot(work, 0), axisSquared = Slot(work, 1);
        Span<ulong> commonMetric = Slot(work, 2), product = Slot(work, 3);
        Import(length, factor); Import(q, axisSquared);
        WideArithmetic.MultiplyMagnitudes(factor, axisSquared, product);
        Import(WideArithmetic.MultiplySigned192(geometry.RawScale, geometry.RawScale), factor);
        WideArithmetic.MultiplyMagnitudes(product, factor, commonMetric);

        // If a supporting vertex lies on the axis, its circle value is
        // constant. One seed covers a whole-circle patch; otherwise real-edge
        // intersections supply the ends of that vertex's admitted arc.
        // Complete admission precedes this collector, so the first exact zero
        // attains the global lower bound and owns all later ties.
        Signed576 one = Signed576.ExtendValue(Signed192.Signed(1));
        WideDirection3d seed = !geometry.Axis.Y.IsZero || !geometry.Axis.Z.IsZero
            ? new WideDirection3d(one, default, default)
            : new WideDirection3d(default, one, default);
        KeepVertex(geometry, seed, commonMetric, values, signs, ref selection);
        for (int vertex = 0; vertex < geometry.VertexCount && (!selection.HasValue || selection.GapSign != 0); vertex++)
            KeepVertex(geometry, geometry.Vertex(vertex), commonMetric, values, signs, ref selection, vertex);

        for (int edge = 0; edge < geometry.EdgeCount && (!selection.HasValue || selection.GapSign != 0); edge++)
        {
            geometry.Edge(edge, out int start, out int end);
            WideDirection3d e = CurvedPrismContact.Difference(geometry.Vertex(end), geometry.Vertex(start));
            Signed576 g = Signed576.NarrowValue(e.SquaredLength);
            Signed576 d = Signed576.NarrowValue(WideDirection3d.Dot(e, geometry.Axis));
            Signed576 dSquared = Signed576.NarrowValue(WideArithmetic.MultiplySigned576(d, d));
            Signed576 k = WideArithmetic.SubtractSigned576(
                Signed576.NarrowValue(WideArithmetic.MultiplySigned576ToSigned832(g, q)),
                dSquared);
            if (k.IsZero)
                continue;
            Signed832 discriminant = WideArithmetic.SubtractSigned832(
                WideArithmetic.MultiplySigned576(fourTq, k),
                WideArithmetic.MultiplySigned576(radiusSquared, dSquared));
            if (discriminant.Sign < 0)
                continue;

            // U=g*A-d*E, C=A×E, k=q*g-d² and Δ=4T²qk-R²d².
            // n=Rq*U +/- C*sqrt(qΔ) is perpendicular to E and has
            // A.n=Rqk>=0, preserving the unsquared generator condition.
            // Its norm²=q*k²*(4T²q+R²), with no remaining radical.
            WideDirection3d axis = new(geometry.Axis);
            WideDirection3d u = WideDirection3d.DifferenceScaled(axis, g, e, d);
            WideDirection3d cross = WideDirection3d.Cross(axis, e);
            Signed320 radialScale = Signed320.NarrowValue(
                WideArithmetic.MultiplySigned320(geometry.Radius, q));
            values.Clear(); signs.Clear();
            for (int component = 0; component < 3; component++)
            {
                Write(values, signs, component, WideArithmetic.MultiplySigned576ToSigned832(
                    Component(u, component), radialScale));
                Write(values, signs, component + 3, Signed832.ExtendValue(Component(cross, component)), -1);
            }
            Import(WideArithmetic.MultiplySigned832(discriminant, q), Slot(values, 6));
            Signed832 metricFactor = WideArithmetic.MultiplySigned576(k, k);
            Keep(geometry, values, signs, commonMetric, metricFactor, ref selection, start);
            if (discriminant.Sign != 0)
            {
                for (int component = 3; component < 6; component++) signs[component] = -signs[component];
                Keep(geometry, values, signs, commonMetric, metricFactor, ref selection, start);
            }
        }
    }

    private static bool CanExcludeCircle(in CurvedPrismContactGeometry geometry,
        scoped ConvexContactCandidate incumbent, Signed576 radiusSquared, Signed576 length)
    {
        WideDirection3d up = geometry.Plane(1);
        Signed320 denominator = Signed320.NarrowValue(Signed576.NarrowValue(WideDirection3d.Dot(up, geometry.Axis)));
        if (denominator.IsZero) return false;
        if (denominator.Sign < 0) { up = -up; denominator = WideArithmetic.Negate(denominator); }
        Signed320 numerator = Signed320.NarrowValue(Signed576.NarrowValue(geometry.ProjectionMinimum(up)));
        Signed576 lambda = WideArithmetic.SubtractSigned576(
            WideArithmetic.MultiplySigned320(Signed320.ExtendValue(geometry.HalfFactor), denominator),
            Signed576.ExtendValue(numerator));
        if (lambda.Sign <= 0) return false;
        // p=(numerator/denominator)*Axis is the far native-Up slab point.
        // It must belong to the complete prism, including degenerate extents.
        // Up<65 bits gives numerator<232, denominator<99 and lambda<261.
        // Side planes<233 and vertices<165 give plane minima<400 and axial
        // dots<267. Both membership products and their difference fit 576 bits.
        for (int plane = 0; plane < geometry.PlaneCount; plane++)
        {
            WideDirection3d face = geometry.Plane(plane);
            for (int orientation = 0; orientation < 2; orientation++, face = -face)
            {
                Signed320 projection = Signed320.NarrowValue(Signed576.NarrowValue(WideDirection3d.Dot(face, geometry.Axis)));
                Signed576 left = WideArithmetic.MultiplySigned320(numerator, projection);
                Signed832 right = WideArithmetic.MultiplySigned832(geometry.ProjectionMinimum(face), denominator);
                if (WideArithmetic.SubtractSigned832(Signed832.ExtendValue(left), right).Sign < 0) return false;
            }
        }
        // Every generator normal has Axis.n/|n|=R*sqrt(q)/sqrt(length).
        // Since minPrism(n)<=p.n, its gap is at least
        // lambda*R*sqrt(q)/(denominator*S*sqrt(length)). Exact analytic ties
        // already retain the earlier feature and every witness term.
        const int boundWords = 16;
        Span<ulong> values = stackalloc ulong[ConvexContactCandidate.Slots * boundWords];
        Span<int> signs = stackalloc int[ConvexContactCandidate.Slots];
        Span<ulong> work = stackalloc ulong[3 * boundWords];
        values.Clear(); signs.Clear();
        values[0] = 1; signs[0] = signs[7] = 1;
        Span<ulong> first = Slot(work, 0, boundWords), second = Slot(work, 1, boundWords);
        Span<ulong> product = Slot(work, 2, boundWords);
        // lambda<261, R<163 and q<65 give numerator<913 bits;
        // S<99, denominator<99 and length<327 give denominator<723.
        // Sixteen-word fields retain every complete product without rounding.
        Import(WideArithmetic.MultiplySigned576ToSigned832(lambda, lambda), first);
        Import(radiusSquared, second);
        WideArithmetic.MultiplyMagnitudes(first, second, product);
        Import(geometry.SquaredAxis, first);
        WideArithmetic.MultiplyMagnitudes(product, first, Slot(values, 7, boundWords));
        Import(WideArithmetic.MultiplySigned192(geometry.RawScale, geometry.RawScale), first);
        Import(WideArithmetic.MultiplySigned320(denominator, denominator), second);
        WideArithmetic.MultiplyMagnitudes(first, second, product);
        Import(length, first);
        WideArithmetic.MultiplyMagnitudes(product, first, Slot(values, 10, boundWords));
        return WideConvexPrismRelations.CompareConvexContactCandidates(
            new ConvexContactCandidate(values, signs, 1, boundWords), incumbent) >= 0;
    }

    private static void KeepVertex(in CurvedPrismContactGeometry geometry, in WideDirection3d point,
        scoped ReadOnlySpan<ulong> commonMetric, scoped Span<ulong> values, scoped Span<int> signs,
        ref CurvedPrismContactSelection selection, int requiredVertex = -1)
    {
        Signed576 projection = Signed576.NarrowValue(WideDirection3d.Dot(point, geometry.Axis));
        WideDirection3d radial = WideDirection3d.DifferenceScaled(point, Signed576.ExtendValue(geometry.SquaredAxis),
            new WideDirection3d(geometry.Axis), projection);
        if (radial.IsZero)
            return;
        Signed832 radialSquared = radial.SquaredLength;
        Signed576 twiceTq = WideArithmetic.MultiplySigned320(geometry.SquaredAxis, geometry.HalfFactor);
        Signed320 radialScale = Signed320.NarrowValue(WideArithmetic.AddSigned576(twiceTq, twiceTq));
        values.Clear(); signs.Clear();
        // V=qP-(A.P)A; n=+/-2TqV+R*A*sqrt(V²).
        // Both extrema have norm²=q*V²*(4T²q+R²).
        for (int component = 0; component < 3; component++)
        {
            Write(values, signs, component, WideArithmetic.MultiplySigned576ToSigned832(
                Component(radial, component), radialScale), -1);
            Write(values, signs, component + 3, Signed832.ExtendValue(WideArithmetic.MultiplySigned320(
                geometry.Radius, Component(geometry.Axis, component))));
        }
        Import(radialSquared, Slot(values, 6));
        Keep(geometry, values, signs, commonMetric, radialSquared, ref selection, requiredVertex);
        for (int component = 0; component < 3; component++) signs[component] = -signs[component];
        Keep(geometry, values, signs, commonMetric, radialSquared, ref selection, requiredVertex);
    }

    private static void Keep(in CurvedPrismContactGeometry geometry,
        scoped Span<ulong> values, scoped Span<int> signs, scoped ReadOnlySpan<ulong> commonMetric,
        Signed832 metricFactor, ref CurvedPrismContactSelection selection, int requiredVertex)
    {
        Span<ulong> work = stackalloc ulong[6 * Words];
        Span<int> workSigns = stackalloc int[4];
        ContactQuadratic minimum = ContactQuadratic.At(work, workSigns, 0);
        ContactQuadratic gap = ContactQuadratic.At(work, workSigns, 1);
        geometry.GetMinimum(values, signs, minimum, out int vertex);
        if (requiredVertex >= 0)
        {
            // A real vertex extremum or edge intersection belongs to this
            // patch only when its generating vertex minimizes the prism dot.
            // Equality includes every tied edge endpoint; the artificial
            // whole-circle seed remains unrestricted.
            WideDirection3d point = geometry.Vertex(requiredVertex);
            Dot(point, values[..(3 * Words)], signs[..3], gap.Rational, out int pointRationalSign);
            Dot(point, values.Slice(3 * Words, 3 * Words), signs.Slice(3, 3),
                gap.Radical, out int pointRadicalSign);
            gap.Signs[0] = pointRationalSign; gap.Signs[1] = pointRadicalSign;
            gap.Add(minimum, -1);
            if (gap.Sign(Slot(values, 6)) != 0) return;
        }
        Dot(geometry.Half, values[..(3 * Words)], signs[..3], gap.Rational, out int rationalSign);
        Dot(geometry.Half, values.Slice(3 * Words, 3 * Words), signs.Slice(3, 3),
            gap.Radical, out int radicalSign);
        gap.Signs[0] = rationalSign; gap.Signs[1] = radicalSign;
        gap.Add(minimum, -1);
        Import(metricFactor, Slot(work, 4));
        WideArithmetic.MultiplyMagnitudes(commonMetric, Slot(work, 4), Slot(work, 5));
        int gapSign = BuildQuadraticCandidate(gap, Slot(values, 6), Slot(work, 5), values, signs);
        selection.Keep(geometry, values, signs, gapSign, vertex, CurvedPrismContactSelection.Free);
    }

    private static void Write(Span<ulong> values, Span<int> signs, int slot, Signed832 value, int orientation = 1)
    {
        Import(value, Slot(values, slot)); signs[slot] = orientation * value.Sign;
    }
}
