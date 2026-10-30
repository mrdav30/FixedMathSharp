//=======================================================================
// MIT License, Copyright (c) 2024-present David Oravsky (mrdav30)
// See LICENSE file in the project root for full license information.
//=======================================================================
using System;
using static FixedMathSharp.Geometry.CylinderContactAlgebra;
using static FixedMathSharp.Geometry.ContactQuadratic;
using static FixedMathSharp.Geometry.TriangleQuadraticSlice;

namespace FixedMathSharp.Geometry;

/// <summary>Matched witnesses on the stadium prism's free-core support seam.</summary>
internal static class TriangleCapsuleSlabWitnesses
{
    // Rigid seam products fit forty words (<2520 bits). Affine seam normals
    // stay below 344 bits, blended weights below 2515, and materialization
    // coefficients below 4000: eighty words retain complete products.
    // Rational cap admission squares stay below 3440 bits; quadratic sign
    // comparisons reserve their complete squared scratch separately.
    internal static void GetPoints(in TriangleCircularGeometry geometry, FixedTriangle triangle,
        int mask, ReadOnlySpan<ulong> normal, ReadOnlySpan<int> normalSigns,
        WideDirection3d coreOffset, Vector2d authoredAxis, Fixed64 coreLength, ref Vector3d radialPoint,
        out Vector3d trianglePoint, out Vector3d coreAndAxialPoint, out FixedPointAnchorTerm3d exactTerm)
    {
        bool admitted = TryGetPoints(geometry, triangle, mask, normal, normalSigns, coreOffset, authoredAxis, coreLength,
            ref radialPoint, out trianglePoint, out coreAndAxialPoint, out exactTerm);
        System.Diagnostics.Debug.Assert(admitted);
    }

    internal static bool TryGetPoints(in TriangleCircularGeometry geometry, FixedTriangle triangle,
        int mask, ReadOnlySpan<ulong> normal, ReadOnlySpan<int> normalSigns,
        WideDirection3d coreOffset, Vector2d authoredAxis, Fixed64 coreLength, ref Vector3d radialPoint,
        out Vector3d trianglePoint, out Vector3d coreAndAxialPoint, out FixedPointAnchorTerm3d exactTerm)
    {
        int Words = geometry.ParameterWords;
        trianglePoint = coreAndAxialPoint = default; exactTerm = default;
        WideDirection3d axis = new(Signed576.ExtendValue(Signed192.Raw(authoredAxis.X)), default,
            Signed576.ExtendValue(Signed192.Raw(authoredAxis.Y)));
        Signed832 bound = WideDirection3d.Dot(coreOffset, axis);
        Span<Signed832> projections = stackalloc Signed832[3];
        for (int vertex = 0; vertex < 3; vertex++)
            projections[vertex] = WideDirection3d.Dot(geometry.Vertex(vertex), axis);
        Span<ulong> weights = stackalloc ulong[6 * Words];
        Span<int> weightSigns = stackalloc int[6];
        Span<ulong> root = stackalloc ulong[Words];
        weights.Clear(); weightSigns.Clear(); root.Clear();
        bool pole = normalSigns[0] == 0 && normalSigns[2] == 0;
        int cap = normalSigns[1];
        WideDirection3d compact = new(ReadComponent(normal, normalSigns, 0),
            ReadComponent(normal, normalSigns, 1), ReadComponent(normal, normalSigns, 2));
        Signed832 metric = WideArithmetic.AddSigned832(WideArithmetic.MultiplySigned576(compact.X, compact.X),
            WideArithmetic.MultiplySigned576(compact.Z, compact.Z));
        Import(metric, root);
        bool admitted = pole
            ? TryGetCapWeights(geometry, mask, axis, coreOffset, projections, bound, weights, weightSigns)
            : cap == 0 ? TryGetSideWeights(geometry, mask, projections, bound, weights, weightSigns)
            : TryGetObliqueWeights(geometry, mask, axis, compact, cap, projections, bound, root, weights, weightSigns);
        if (!admitted) return false;

        GetMaterials(geometry, triangle, axis, authoredAxis, coreLength, compact, metric, projections, bound,
            cap, pole, root, weights, weightSigns, ref radialPoint, out trianglePoint, out coreAndAxialPoint, out exactTerm);
        return true;
    }

    private static bool TryGetSideWeights(in TriangleCircularGeometry geometry, int mask,
        ReadOnlySpan<Signed832> projections, Signed832 coreBound,
        Span<ulong> weights, Span<int> weightSigns)
    {
        int Words = geometry.ParameterWords;
        Span<Signed832> axial = stackalloc Signed832[3];
        Signed832 half = Signed832.ExtendValue(geometry.HalfHeight);
        for (int vertex = 0; vertex < 3; vertex++)
        {
            axial[vertex] = Signed832.ExtendValue(geometry.Vertex(vertex).Y);
            if ((mask & (1 << vertex)) != 0 && InInterval(projections[vertex], coreBound)
                && InInterval(axial[vertex], half))
            {
                At(weights, weightSigns, vertex, weights.Length / weightSigns.Length).Set(Signed832.ExtendValue(Signed576.One));
                return true;
            }
        }
        // After excluding admitted vertices, test both axial boundaries.
        // If neither meets the feature, a nonempty intersection must meet
        // the negative core boundary: one confined to the positive boundary
        // would end at an admitted vertex or an already-tested axial side.
        Span<ulong> distances = stackalloc ulong[6 * Words];
        Span<int> signs = stackalloc int[6];
        Span<ulong> root = stackalloc ulong[Words]; root.Clear();
        for (int side = -1; side <= 1; side += 2)
        {
            for (int vertex = 0; vertex < 3; vertex++)
                At(distances, signs, vertex, distances.Length / signs.Length).Set(WideArithmetic.SubtractSigned832(axial[vertex], side > 0 ? half : Negate(half)));
            if (TrySlice(mask, distances, signs, projections, coreBound, root, weights, weightSigns))
                return true;
        }
        for (int vertex = 0; vertex < 3; vertex++)
            At(distances, signs, vertex, distances.Length / signs.Length).Set(WideArithmetic.AddSigned832(projections[vertex], coreBound));
        return TrySlice(mask, distances, signs, axial, half, root, weights, weightSigns);
    }

    private static bool TryGetCapWeights(in TriangleCircularGeometry geometry, int mask, WideDirection3d axis,
        WideDirection3d coreOffset, ReadOnlySpan<Signed832> projections, Signed832 bound,
        Span<ulong> weights, Span<int> weightSigns)
    {
        int Words = geometry.ParameterWords;
        Span<ulong> distances = stackalloc ulong[6 * Words];
        Span<int> signs = stackalloc int[6];
        Span<ulong> root = stackalloc ulong[Words]; root.Clear();
        WideDirection3d transverse = new(WideArithmetic.Negate(axis.Z), default, axis.X);
        for (int vertex = 0; vertex < 3; vertex++)
            At(distances, signs, vertex, distances.Length / signs.Length).Set(WideDirection3d.Dot(geometry.Vertex(vertex), transverse));
        if (TrySlice(mask, distances, signs, projections, bound, root, weights, weightSigns))
            return true;

        // With no core/feature intersection, the closest planar pair has a
        // triangle vertex or a core endpoint. These are witness constructions,
        // not an endpoint-cylinder union used to classify the stadium.
        for (int vertex = 0; vertex < 3; vertex++)
        {
            if ((mask & (1 << vertex)) == 0)
                continue;
            weights.Clear(); weightSigns.Clear();
            At(weights, weightSigns, vertex, weights.Length / weightSigns.Length).Set(Signed832.ExtendValue(Signed576.One));
            if (WithinRadius(geometry, axis, projections, bound, weights, weightSigns))
                return true;
        }
        GetEndpointCapWeights(geometry, mask, coreOffset, -1, weights, weightSigns);
        if (WithinRadius(geometry, axis, projections, bound, weights, weightSigns))
            return true;
        // The preceding exhaustive closest-feature cases have failed; the
        // other core endpoint must supply the admitted cap witness.
        GetEndpointCapWeights(geometry, mask, coreOffset, 1, weights, weightSigns);
        return WithinRadius(geometry, axis, projections, bound, weights, weightSigns);
    }

    private static void GetEndpointCapWeights(in TriangleCircularGeometry geometry, int mask, WideDirection3d coreOffset,
        int side, Span<ulong> weights, Span<int> weightSigns)
    {
        int Words = geometry.ParameterWords;
        Span<ulong> rationalWeights = stackalloc ulong[3 * CylinderContactAlgebra.Words];
        TriangleCircularGeometry endpoint = geometry.AtCoreRegion(coreOffset, side);
        TriangleCylinderWitnesses.GetCapWeights(endpoint, mask, rationalWeights);
        weights.Clear(); weightSigns.Clear();
        for (int vertex = 0; vertex < 3; vertex++)
        {
            Slot(rationalWeights, vertex).CopyTo(At(weights, weightSigns, vertex, weights.Length / weightSigns.Length).Rational);
            At(weights, weightSigns, vertex, weights.Length / weightSigns.Length).Signs[0] = IsZero(Slot(rationalWeights, vertex)) ? 0 : 1;
        }
    }

    private static bool TryGetObliqueWeights(in TriangleCircularGeometry geometry, int mask, WideDirection3d axis,
        WideDirection3d compact, int cap,
        ReadOnlySpan<Signed832> projections, Signed832 bound, ReadOnlySpan<ulong> root,
        Span<ulong> weights, Span<int> weightSigns)
    {
        int Words = geometry.ParameterWords;
        // Seam ownership supplies Up, its planar perpendicular, or one
        // retained edge crossed with the raw authored axis (<230 bits).
        WideDirection3d tangent = WideDirection3d.Cross(axis, compact);

        Span<ulong> distances = stackalloc ulong[6 * Words];
        Span<int> signs = stackalloc int[6];
        Span<ulong> scratch = stackalloc ulong[4 * Words];
        Span<int> scratchSigns = stackalloc int[4];
        ContactQuadratic radical = At(scratch, scratchSigns, 0, scratch.Length / scratchSigns.Length), factor = At(scratch, scratchSigns, 1, scratch.Length / scratchSigns.Length);
        // sqrt(K)*tangent·(v-p) =
        // (tangent·v + cap*H*tangent.Y)*sqrt(K) - R*n.Y*tangent.Y.
        radical.Set(WideArithmetic.MultiplySigned576(compact.Y, tangent.Y));
        Scale(radical, Signed832.ExtendValue(geometry.Radius), factor);
        for (int vertex = 0; vertex < 3; vertex++)
        {
            ContactQuadratic distance = At(distances, signs, vertex, distances.Length / signs.Length);
            factor.CopyTo(distance, -1);
            Signed832 axial = WideArithmetic.MultiplySigned576(geometry.HalfHeight, tangent.Y);
            Signed832 value = WideArithmetic.AddSigned832(WideDirection3d.Dot(geometry.Vertex(vertex), tangent),
                cap > 0 ? axial : Negate(axial));
            Import(value, distance.Radical); distance.Signs[1] = value.Sign;
        }
        return TrySlice(mask, distances, signs, projections, bound, root, weights, weightSigns);
    }

    private static bool TrySlice(int mask, Span<ulong> distances, Span<int> distanceSigns,
        ReadOnlySpan<Signed832> projections, Signed832 bound, ReadOnlySpan<ulong> root,
        Span<ulong> weights, Span<int> weightSigns)
    {
        int Words = weights.Length / weightSigns.Length;
        Span<ulong> values = stackalloc ulong[8 * Words];
        Span<int> signs = stackalloc int[8];
        for (int vertex = 0; vertex < 3; vertex++)
            At(values, signs, vertex, values.Length / signs.Length).Set(projections[vertex]);
        At(values, signs, 3, values.Length / signs.Length).Set(bound);
        return TriangleQuadraticSlice.TryGetWeights(mask, distances, distanceSigns,
            values[..(6 * Words)], signs[..6], At(values, signs, 3, values.Length / signs.Length), root,
            weights, weightSigns, out _);
    }

    private static bool WithinRadius(in TriangleCircularGeometry geometry, WideDirection3d axis,
        ReadOnlySpan<Signed832> projections, Signed832 bound, Span<ulong> weights, Span<int> signs)
    {
        int Words = geometry.ParameterWords;
        Span<ulong> values = stackalloc ulong[16 * Words];
        Span<int> valueSigns = stackalloc int[16];
        Span<ulong> root = stackalloc ulong[Words]; root.Clear();
        ContactQuadratic denominator = At(values, valueSigns, 0, values.Length / valueSigns.Length), projection = At(values, valueSigns, 1, values.Length / valueSigns.Length);
        ContactQuadratic coordinate = At(values, valueSigns, 2, values.Length / valueSigns.Length), scaled = At(values, valueSigns, 3, values.Length / valueSigns.Length);
        ContactQuadratic difference = At(values, valueSigns, 4, values.Length / valueSigns.Length), squared = At(values, valueSigns, 5, values.Length / valueSigns.Length);
        ContactQuadratic sum = At(values, valueSigns, 6, values.Length / valueSigns.Length), limit = At(values, valueSigns, 7, values.Length / valueSigns.Length);
        SumWeights(weights, signs, denominator);
        Sum(projections, weights, signs, projection);
        ClampProjection(denominator, projection, bound, root);
        Signed832 axisSquared = axis.SquaredLength;
        sum.Clear();
        for (int component = 0; component < 3; component += 2)
        {
            SumCoordinate(geometry, weights, signs, component, coordinate);
            Scale(coordinate, axisSquared, difference);
            Scale(projection, Signed832.ExtendValue(Component(axis, component)), scaled);
            difference.Add(scaled, -1);
            Multiply(difference, difference, root, squared);
            sum.Add(squared);
        }
        Signed832 radiusAxis = WideArithmetic.MultiplySigned576(Signed576.NarrowValue(axisSquared), geometry.Radius);
        Scale(denominator, radiusAxis, limit);
        Multiply(limit, limit, root, squared);
        sum.Add(squared, -1);
        return sum.Sign(root) <= 0;
    }

    private static void GetMaterials(in TriangleCircularGeometry geometry, FixedTriangle triangle, WideDirection3d axis,
        Vector2d authoredAxis, Fixed64 coreLength,
        WideDirection3d normal, Signed832 metric, ReadOnlySpan<Signed832> projections, Signed832 bound,
        int cap, bool pole, ReadOnlySpan<ulong> root, Span<ulong> weights, Span<int> signs,
        ref Vector3d radialPoint, out Vector3d trianglePoint, out Vector3d coreAndAxialPoint,
        out FixedPointAnchorTerm3d exactTerm)
    {
        int Words = geometry.ParameterWords;
        Span<ulong> values = stackalloc ulong[16 * Words];
        Span<int> valueSigns = stackalloc int[16];
        ContactQuadratic denominator = At(values, valueSigns, 0, values.Length / valueSigns.Length), projection = At(values, valueSigns, 1, values.Length / valueSigns.Length);
        ContactQuadratic coordinate = At(values, valueSigns, 2, values.Length / valueSigns.Length), coreNumerator = At(values, valueSigns, 3, values.Length / valueSigns.Length);
        ContactQuadratic coreDenominator = At(values, valueSigns, 4, values.Length / valueSigns.Length), product = At(values, valueSigns, 5, values.Length / valueSigns.Length);
        ContactQuadratic combinedNumerator = At(values, valueSigns, 6, values.Length / valueSigns.Length), combinedDenominator = At(values, valueSigns, 7, values.Length / valueSigns.Length);
        SumWeights(weights, signs, denominator);
        Sum(projections, weights, signs, projection);
        if (pole)
            ClampProjection(denominator, projection, bound, root);
        int endpointSign = projection.Sign(root);
        Scale(denominator, bound, coordinate);
        coordinate.MultiplySign(endpointSign); coordinate.Add(projection, -1);
        if (coordinate.Sign(root) != 0)
            endpointSign = 0;
        Vector3d endpointCore = endpointSign == 0 ? default
            : TriangleCapsuleSlabEndpointWitnesses.GetCore(authoredAxis, coreLength, -endpointSign, out _);
        int integralMask = 0;
        trianglePoint = new Vector3d(
            RoundTriangleCoordinate(geometry, triangle, 0, weights, signs, denominator, root),
            RoundTriangleCoordinate(geometry, triangle, 1, weights, signs, denominator, root),
            RoundTriangleCoordinate(geometry, triangle, 2, weights, signs, denominator, root));

        Signed832 axisSquared = axis.SquaredLength;
        Signed832 scale = WideArithmetic.MultiplySigned832(axisSquared, geometry.RawScale);
        Scale(denominator, scale, coreDenominator);
        Vector3d core = default;
        radialPoint = default;
        for (int component = 0; component < 3; component += 2)
        {
            Scale(projection, Signed832.ExtendValue(Component(axis, component)), coreNumerator);
            int radialSign;
            if (pole)
            {
                // The cap's combined planar point is q itself, not two
                // independently rounded core/projection components.
                SumCoordinate(geometry, weights, signs, component, coordinate);
                Scale(coordinate, axisSquared, combinedNumerator);
                coreDenominator.CopyTo(combinedDenominator);
                combinedNumerator.CopyTo(product); product.Add(coreNumerator, -1);
                radialSign = product.Sign(root);
            }
            else
            {
                // K*C - R*n_i*sumWeights*a²*sqrt(K), over K times
                // C's denominator, is the exact combined support coordinate.
                Scale(coreNumerator, metric, combinedNumerator);
                Scale(coreDenominator, metric, combinedDenominator);
                Scale(denominator, axisSquared, product);
                Signed832 radialFactor = WideArithmetic.MultiplySigned576(geometry.Radius, Component(normal, component));
                Scale(product, radialFactor, coordinate);
                MultiplyRoot(coordinate, root, product);
                combinedNumerator.Add(product, -1);
                radialSign = -Component(normal, component).Sign * geometry.Radius.Sign;
            }
            if (endpointSign != 0)
            {
                // An integral radial offset leaves the exact endpoint's
                // half-core residual meaningful. Test the bounded radial
                // value itself, never rounded(P)-rounded(C), which can
                // differ by one at a half tie or exceed the scalar domain.
                if (!pole)
                    product.MultiplySign(-1);
                Fixed64 radial = RoundRatio(product, combinedDenominator, root);
                Scale(combinedDenominator, Signed832.ExtendValue(Signed576.ExtendValue(Signed192.Raw(radial))), coordinate);
                coordinate.Add(product, -1);
                if (coordinate.Sign(root) == 0)
                {
                    if (component == 0) { core.X = endpointCore.X; radialPoint.X = radial; }
                    else { core.Z = endpointCore.Z; radialPoint.Z = radial; }
                    integralMask |= 1 << component;
                    continue;
                }
            }
            // An integer base on the radial side of C keeps P-base within
            // [-Max,Max], even when P itself cannot fit a scalar. Its parity
            // participates in the residual tie, preserving one rounding of P.
            Fixed64 presentation = GetCorePresentation(coreNumerator, coreDenominator, radialSign, root);
            Scale(combinedDenominator, Signed832.ExtendValue(Signed576.ExtendValue(Signed192.Raw(presentation))), product);
            combinedNumerator.Add(product, -1);
            Fixed64 residual = RoundRatio(combinedNumerator, combinedDenominator, root,
                parityOffset: presentation.m_rawValue);
            if (component == 0) { core.X = presentation; radialPoint.X = residual; }
            else { core.Z = presentation; radialPoint.Z = residual; }
        }
        Scale(denominator, Signed832.ExtendValue(Signed576.ExtendValue(geometry.RawScale)), coreDenominator);
        if (cap == 0)
            SumCoordinate(geometry, weights, signs, 1, coreNumerator);
        else
            Scale(denominator, Signed832.ExtendValue(cap > 0 ? WideArithmetic.Negate(geometry.HalfHeight) : geometry.HalfHeight), coreNumerator);
        core.Y = RoundRatio(coreNumerator, coreDenominator, root);
        coreAndAxialPoint = core;
        exactTerm = integralMask == 0 ? default : TriangleCapsuleSlabEndpointWitnesses.CreateTerm(
            authoredAxis, endpointSign > 0 ? coreLength : -coreLength, core, integralMask);
    }

    private static Fixed64 GetCorePresentation(ContactQuadratic numerator, ContactQuadratic denominator, int radialSign, ReadOnlySpan<ulong> root)
    {
        int Words = denominator.FieldWords;
        Fixed64 rounded = RoundRatio(numerator, denominator, root);
        Span<ulong> values = stackalloc ulong[2 * Words];
        Span<int> signs = stackalloc int[2];
        ContactQuadratic difference = At(values, signs, 0, values.Length / signs.Length);
        Scale(denominator, Signed832.ExtendValue(Signed576.ExtendValue(Signed192.Raw(rounded))), difference);
        difference.MultiplySign(-1); difference.Add(numerator);
        int comparison = difference.Sign(root);
        if (radialSign >= 0 && comparison > 0)
            return Fixed64.FromRaw(rounded.m_rawValue + 1);
        if (radialSign < 0 && comparison < 0)
            return Fixed64.FromRaw(rounded.m_rawValue - 1);
        return rounded;
    }

    private static void ClampProjection(ContactQuadratic denominator, ContactQuadratic projection, Signed832 bound, ReadOnlySpan<ulong> root)
    {
        int Words = denominator.FieldWords;
        Span<ulong> values = stackalloc ulong[4 * Words];
        Span<int> signs = stackalloc int[4];
        ContactQuadratic limit = At(values, signs, 0, values.Length / signs.Length), difference = At(values, signs, 1, values.Length / signs.Length);
        Scale(denominator, bound, limit);
        projection.CopyTo(difference); difference.Add(limit, -1);
        if (difference.Sign(root) > 0)
            limit.CopyTo(projection);
        else
        {
            projection.CopyTo(difference); difference.Add(limit);
            if (difference.Sign(root) < 0)
                limit.CopyTo(projection, -1);
        }
    }

    private static void SumCoordinate(in TriangleCircularGeometry geometry, Span<ulong> weights,
        Span<int> signs, int component, ContactQuadratic result)
    {
        Span<Signed832> coordinates = stackalloc Signed832[3];
        for (int vertex = 0; vertex < 3; vertex++)
            coordinates[vertex] = Signed832.ExtendValue(Component(geometry.Vertex(vertex), component));
        Sum(coordinates, weights, signs, result);
    }

    private static bool InInterval(Signed832 value, Signed832 bound) =>
        WideArithmetic.SubtractSigned832(value, bound).Sign <= 0
        && WideArithmetic.AddSigned832(value, bound).Sign >= 0;
    private static Signed832 Negate(Signed832 value) => WideArithmetic.SubtractSigned832(default, value);
    private static Signed576 ReadComponent(ReadOnlySpan<ulong> values, ReadOnlySpan<int> signs, int component)
    {
        ReadOnlySpan<ulong> value = Slot(values, component);
        Signed576 result = new(value[8], value[7], value[6], value[5], value[4], value[3], value[2], value[1], value[0]);
        return signs[component] < 0 ? WideArithmetic.Negate(result) : result;
    }
}
