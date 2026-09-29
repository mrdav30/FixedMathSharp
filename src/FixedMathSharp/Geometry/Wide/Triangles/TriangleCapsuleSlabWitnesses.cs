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
    // The seam's retained edge normal is <230 bits, plane-distance
    // coefficients <732, and the zero-coordinate blend <1740. Combined
    // coordinate/rounding products stay below 2520 bits; rational cap
    // projection weights <832 and their radius checks <2300 fit 40 words.
    internal static void GetPoints(in TriangleCircularGeometry geometry, FixedTriangle triangle,
        int mask, ReadOnlySpan<ulong> normal, ReadOnlySpan<int> normalSigns,
        WideAxis3 coreOffset, Vector2d authoredAxis, Fixed64 coreLength, ref Vector3d radialPoint,
        out Vector3d trianglePoint, out Vector3d coreAndAxialPoint, out FixedPointAnchorTerm3d exactTerm)
    {
        WideAxis3 axis = new(Signed320.ExtendValue(Signed192.Raw(authoredAxis.X)), default,
            Signed320.ExtendValue(Signed192.Raw(authoredAxis.Y)));
        Signed576 bound = WideAxis3.Dot(coreOffset, axis);
        Span<Signed576> projections = stackalloc Signed576[3];
        for (int vertex = 0; vertex < 3; vertex++)
            projections[vertex] = WideAxis3.Dot(geometry.Vertex(vertex), axis);
        Span<ulong> weights = stackalloc ulong[6 * Words];
        Span<int> weightSigns = stackalloc int[6];
        Span<ulong> root = stackalloc ulong[Words];
        weights.Clear(); weightSigns.Clear(); root.Clear();
        bool pole = normalSigns[0] == 0 && normalSigns[2] == 0;
        int cap = normalSigns[1];
        WideAxis3 compact = new(ReadComponent(normal, normalSigns, 0),
            ReadComponent(normal, normalSigns, 1), ReadComponent(normal, normalSigns, 2));
        Signed576 metric = WideArithmetic.AddSigned576(WideArithmetic.MultiplySigned320(compact.X, compact.X),
            WideArithmetic.MultiplySigned320(compact.Z, compact.Z));
        Import(metric, root);
        if (pole)
            GetCapWeights(geometry, mask, axis, coreOffset, projections, bound, weights, weightSigns);
        else if (cap == 0)
            GetSideWeights(geometry, mask, projections, bound, weights, weightSigns);
        else if ((mask & (mask - 1)) == 0)
        {
            // A positive oblique vertex gap cannot minimize a linear
            // rectangle quadrant. At zero, earlier poles or incident AB
            // own A/B ties; only AB's strict third-vertex support can win.
            System.Diagnostics.Debug.Assert(mask == 4);
            At(weights, weightSigns, 2).Set(Signed576.One);
        }
        else
            GetObliqueWeights(geometry, mask, axis, compact, cap,
                projections, bound, root, weights, weightSigns);

        GetMaterials(geometry, triangle, axis, authoredAxis, coreLength, compact, metric, projections, bound,
            cap, pole, root, weights, weightSigns, ref radialPoint, out trianglePoint, out coreAndAxialPoint, out exactTerm);
    }

    private static void GetSideWeights(in TriangleCircularGeometry geometry, int mask,
        ReadOnlySpan<Signed576> projections, Signed576 coreBound,
        Span<ulong> weights, Span<int> weightSigns)
    {
        Span<Signed576> axial = stackalloc Signed576[3];
        Signed576 half = Signed576.ExtendValue(geometry.HalfHeight);
        for (int vertex = 0; vertex < 3; vertex++)
        {
            axial[vertex] = Signed576.ExtendValue(geometry.Vertex(vertex).Y);
            if ((mask & (1 << vertex)) != 0 && InInterval(projections[vertex], coreBound)
                && InInterval(axial[vertex], half))
            {
                At(weights, weightSigns, vertex).Set(Signed576.One);
                return;
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
                At(distances, signs, vertex).Set(WideArithmetic.SubtractSigned576(axial[vertex], side > 0 ? half : Negate(half)));
            if (TrySlice(mask, distances, signs, projections, coreBound, root, weights, weightSigns))
                return;
        }
        for (int vertex = 0; vertex < 3; vertex++)
            At(distances, signs, vertex).Set(WideArithmetic.AddSigned576(projections[vertex], coreBound));
        bool found = TrySlice(mask, distances, signs, axial, half, root, weights, weightSigns);
        System.Diagnostics.Debug.Assert(found);
    }

    private static void GetCapWeights(in TriangleCircularGeometry geometry, int mask, WideAxis3 axis,
        WideAxis3 coreOffset, ReadOnlySpan<Signed576> projections, Signed576 bound,
        Span<ulong> weights, Span<int> weightSigns)
    {
        Span<ulong> distances = stackalloc ulong[6 * Words];
        Span<int> signs = stackalloc int[6];
        Span<ulong> root = stackalloc ulong[Words]; root.Clear();
        WideAxis3 transverse = new(WideArithmetic.Negate(axis.Z), default, axis.X);
        for (int vertex = 0; vertex < 3; vertex++)
            At(distances, signs, vertex).Set(WideAxis3.Dot(geometry.Vertex(vertex), transverse));
        if (TrySlice(mask, distances, signs, projections, bound, root, weights, weightSigns))
            return;

        // With no core/feature intersection, the closest planar pair has a
        // triangle vertex or a core endpoint. These are witness constructions,
        // not an endpoint-cylinder union used to classify the stadium.
        for (int vertex = 0; vertex < 3; vertex++)
        {
            if ((mask & (1 << vertex)) == 0)
                continue;
            weights.Clear(); weightSigns.Clear();
            At(weights, weightSigns, vertex).Set(Signed576.One);
            if (WithinRadius(geometry, axis, projections, bound, weights, weightSigns))
                return;
        }
        GetEndpointCapWeights(geometry, mask, coreOffset, -1, weights, weightSigns);
        if (WithinRadius(geometry, axis, projections, bound, weights, weightSigns))
            return;
        // The preceding exhaustive closest-feature cases have failed; the
        // other core endpoint must supply the admitted cap witness.
        GetEndpointCapWeights(geometry, mask, coreOffset, 1, weights, weightSigns);
        System.Diagnostics.Debug.Assert(WithinRadius(geometry, axis, projections, bound, weights, weightSigns));
    }

    private static void GetEndpointCapWeights(in TriangleCircularGeometry geometry, int mask, WideAxis3 coreOffset,
        int side, Span<ulong> weights, Span<int> weightSigns)
    {
        Span<ulong> rationalWeights = stackalloc ulong[3 * Words];
        TriangleCircularGeometry endpoint = geometry.AtCoreRegion(coreOffset, side);
        TriangleCylinderWitnesses.GetCapWeights(endpoint, mask, rationalWeights);
        weights.Clear(); weightSigns.Clear();
        for (int vertex = 0; vertex < 3; vertex++)
        {
            Slot(rationalWeights, vertex).CopyTo(At(weights, weightSigns, vertex).Rational);
            At(weights, weightSigns, vertex).Signs[0] = IsZero(Slot(rationalWeights, vertex)) ? 0 : 1;
        }
    }

    private static void GetObliqueWeights(in TriangleCircularGeometry geometry, int mask, WideAxis3 axis,
        WideAxis3 compact, int cap,
        ReadOnlySpan<Signed576> projections, Signed576 bound, ReadOnlySpan<ulong> root,
        Span<ulong> weights, Span<int> weightSigns)
    {
        // Seam ownership supplies Up, its planar perpendicular, or one
        // retained edge crossed with the raw authored axis (<230 bits).
        WideAxis3 tangent = WideAxis3.Cross(axis, compact);

        Span<ulong> distances = stackalloc ulong[6 * Words];
        Span<int> signs = stackalloc int[6];
        Span<ulong> scratch = stackalloc ulong[4 * Words];
        Span<int> scratchSigns = stackalloc int[4];
        ContactQuadratic radical = At(scratch, scratchSigns, 0), factor = At(scratch, scratchSigns, 1);
        // sqrt(K)*tangent·(v-p) =
        // (tangent·v + cap*H*tangent.Y)*sqrt(K) - R*n.Y*tangent.Y.
        radical.Set(WideArithmetic.MultiplySigned320(compact.Y, tangent.Y));
        Scale(radical, Signed576.ExtendValue(geometry.Radius), factor);
        for (int vertex = 0; vertex < 3; vertex++)
        {
            ContactQuadratic distance = At(distances, signs, vertex);
            factor.CopyTo(distance, -1);
            Signed576 axial = WideArithmetic.MultiplySigned320(geometry.HalfHeight, tangent.Y);
            Signed576 value = WideArithmetic.AddSigned576(WideAxis3.Dot(geometry.Vertex(vertex), tangent),
                cap > 0 ? axial : Negate(axial));
            Import(value, distance.Radical); distance.Signs[1] = value.Sign;
        }
        bool found = TrySlice(mask, distances, signs, projections, bound, root, weights, weightSigns);
        System.Diagnostics.Debug.Assert(found);
    }

    private static bool TrySlice(int mask, Span<ulong> distances, Span<int> distanceSigns,
        ReadOnlySpan<Signed576> projections, Signed576 bound, ReadOnlySpan<ulong> root,
        Span<ulong> weights, Span<int> weightSigns)
    {
        Span<ulong> values = stackalloc ulong[8 * Words];
        Span<int> signs = stackalloc int[8];
        for (int vertex = 0; vertex < 3; vertex++)
            At(values, signs, vertex).Set(projections[vertex]);
        At(values, signs, 3).Set(bound);
        return TriangleQuadraticSlice.TryGetWeights(mask, distances, distanceSigns,
            values[..(6 * Words)], signs[..6], At(values, signs, 3), root,
            weights, weightSigns, out _);
    }

    private static bool WithinRadius(in TriangleCircularGeometry geometry, WideAxis3 axis,
        ReadOnlySpan<Signed576> projections, Signed576 bound, Span<ulong> weights, Span<int> signs)
    {
        Span<ulong> values = stackalloc ulong[16 * Words];
        Span<int> valueSigns = stackalloc int[16];
        Span<ulong> root = stackalloc ulong[Words]; root.Clear();
        ContactQuadratic denominator = At(values, valueSigns, 0), projection = At(values, valueSigns, 1);
        ContactQuadratic coordinate = At(values, valueSigns, 2), scaled = At(values, valueSigns, 3);
        ContactQuadratic difference = At(values, valueSigns, 4), squared = At(values, valueSigns, 5);
        ContactQuadratic sum = At(values, valueSigns, 6), limit = At(values, valueSigns, 7);
        SumWeights(weights, signs, denominator);
        Sum(projections, weights, signs, projection);
        ClampProjection(denominator, projection, bound, root);
        Signed576 axisSquared = axis.SquaredLength;
        sum.Clear();
        for (int component = 0; component < 3; component += 2)
        {
            SumCoordinate(geometry, weights, signs, component, coordinate);
            Scale(coordinate, axisSquared, difference);
            Scale(projection, Signed576.ExtendValue(Component(axis, component)), scaled);
            difference.Add(scaled, -1);
            Multiply(difference, difference, root, squared);
            sum.Add(squared);
        }
        Signed576 radiusAxis = WideArithmetic.MultiplySigned320(Signed320.NarrowValue(axisSquared), geometry.Radius);
        Scale(denominator, radiusAxis, limit);
        Multiply(limit, limit, root, squared);
        sum.Add(squared, -1);
        return sum.Sign(root) <= 0;
    }

    private static void GetMaterials(in TriangleCircularGeometry geometry, FixedTriangle triangle, WideAxis3 axis,
        Vector2d authoredAxis, Fixed64 coreLength,
        WideAxis3 normal, Signed576 metric, ReadOnlySpan<Signed576> projections, Signed576 bound,
        int cap, bool pole, ReadOnlySpan<ulong> root, Span<ulong> weights, Span<int> signs,
        ref Vector3d radialPoint, out Vector3d trianglePoint, out Vector3d coreAndAxialPoint,
        out FixedPointAnchorTerm3d exactTerm)
    {
        Span<ulong> values = stackalloc ulong[16 * Words];
        Span<int> valueSigns = stackalloc int[16];
        ContactQuadratic denominator = At(values, valueSigns, 0), projection = At(values, valueSigns, 1);
        ContactQuadratic coordinate = At(values, valueSigns, 2), coreNumerator = At(values, valueSigns, 3);
        ContactQuadratic coreDenominator = At(values, valueSigns, 4), product = At(values, valueSigns, 5);
        ContactQuadratic combinedNumerator = At(values, valueSigns, 6), combinedDenominator = At(values, valueSigns, 7);
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
            RoundTriangleCoordinate(triangle.A.X, triangle.B.X, triangle.C.X, weights, signs, denominator, root),
            RoundTriangleCoordinate(triangle.A.Y, triangle.B.Y, triangle.C.Y, weights, signs, denominator, root),
            RoundTriangleCoordinate(triangle.A.Z, triangle.B.Z, triangle.C.Z, weights, signs, denominator, root));

        Signed576 axisSquared = axis.SquaredLength;
        Signed576 scale = WideArithmetic.MultiplySigned576(axisSquared, geometry.RawScale);
        Scale(denominator, scale, coreDenominator);
        Vector3d core = default;
        radialPoint = default;
        for (int component = 0; component < 3; component += 2)
        {
            Scale(projection, Signed576.ExtendValue(Component(axis, component)), coreNumerator);
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
                Signed576 radialFactor = WideArithmetic.MultiplySigned320(geometry.Radius, Component(normal, component));
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
                Scale(combinedDenominator, Signed576.ExtendValue(Signed320.ExtendValue(Signed192.Raw(radial))), coordinate);
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
            Scale(combinedDenominator, Signed576.ExtendValue(Signed320.ExtendValue(Signed192.Raw(presentation))), product);
            combinedNumerator.Add(product, -1);
            Fixed64 residual = RoundRatio(combinedNumerator, combinedDenominator, root,
                parityOffset: presentation.m_rawValue);
            if (component == 0) { core.X = presentation; radialPoint.X = residual; }
            else { core.Z = presentation; radialPoint.Z = residual; }
        }
        Scale(denominator, Signed576.ExtendValue(Signed320.ExtendValue(geometry.RawScale)), coreDenominator);
        if (cap == 0)
            SumCoordinate(geometry, weights, signs, 1, coreNumerator);
        else
            Scale(denominator, Signed576.ExtendValue(cap > 0 ? WideArithmetic.Negate(geometry.HalfHeight) : geometry.HalfHeight), coreNumerator);
        core.Y = RoundRatio(coreNumerator, coreDenominator, root);
        coreAndAxialPoint = core;
        exactTerm = integralMask == 0 ? default : TriangleCapsuleSlabEndpointWitnesses.CreateTerm(
            authoredAxis, endpointSign > 0 ? coreLength : -coreLength, core, integralMask);
    }

    private static Fixed64 GetCorePresentation(ContactQuadratic numerator, ContactQuadratic denominator, int radialSign, ReadOnlySpan<ulong> root)
    {
        Fixed64 rounded = RoundRatio(numerator, denominator, root);
        Span<ulong> values = stackalloc ulong[2 * Words];
        Span<int> signs = stackalloc int[2];
        ContactQuadratic difference = At(values, signs, 0);
        Scale(denominator, Signed576.ExtendValue(Signed320.ExtendValue(Signed192.Raw(rounded))), difference);
        difference.MultiplySign(-1); difference.Add(numerator);
        int comparison = difference.Sign(root);
        if (radialSign >= 0 && comparison > 0)
            return Fixed64.FromRaw(rounded.m_rawValue + 1);
        if (radialSign < 0 && comparison < 0)
            return Fixed64.FromRaw(rounded.m_rawValue - 1);
        return rounded;
    }

    private static void ClampProjection(ContactQuadratic denominator, ContactQuadratic projection, Signed576 bound, ReadOnlySpan<ulong> root)
    {
        Span<ulong> values = stackalloc ulong[4 * Words];
        Span<int> signs = stackalloc int[4];
        ContactQuadratic limit = At(values, signs, 0), difference = At(values, signs, 1);
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
        Span<Signed576> coordinates = stackalloc Signed576[3];
        for (int vertex = 0; vertex < 3; vertex++)
            coordinates[vertex] = Signed576.ExtendValue(Component(geometry.Vertex(vertex), component));
        Sum(coordinates, weights, signs, result);
    }

    private static bool InInterval(Signed576 value, Signed576 bound) =>
        WideArithmetic.SubtractSigned576(value, bound).Sign <= 0
        && WideArithmetic.AddSigned576(value, bound).Sign >= 0;
    private static Signed576 Negate(Signed576 value) => WideArithmetic.SubtractSigned576(default, value);
    private static Signed320 ReadComponent(ReadOnlySpan<ulong> values, ReadOnlySpan<int> signs, int component)
    {
        ReadOnlySpan<ulong> value = Slot(values, component);
        Signed320 result = new(value[4], value[3], value[2], value[1], value[0]);
        return signs[component] < 0 ? WideArithmetic.Negate(result) : result;
    }
}
