//=======================================================================
// MIT License, Copyright (c) 2024-present David Oravsky (mrdav30)
// See LICENSE file in the project root for full license information.
//=======================================================================
using System;
using static FixedMathSharp.Geometry.CylinderContactAlgebra;

namespace FixedMathSharp.Geometry;

/// <summary>Matched witnesses on the stadium prism's free-core support seam.</summary>
internal static class TriangleCapsuleSlabWitnesses
{
    internal static void GetPoints(in TriangleCylinderGeometry geometry, FixedTriangle triangle,
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
            Q(weights, weightSigns, 2).Set(Signed576.One);
        }
        else
            GetObliqueWeights(geometry, mask, axis, compact, cap,
                projections, bound, root, weights, weightSigns);

        GetMaterials(geometry, triangle, axis, authoredAxis, coreLength, compact, metric, projections, bound,
            cap, pole, root, weights, weightSigns, ref radialPoint, out trianglePoint, out coreAndAxialPoint, out exactTerm);
    }

    private static void GetSideWeights(in TriangleCylinderGeometry geometry, int mask,
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
                Q(weights, weightSigns, vertex).Set(Signed576.One);
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
                Q(distances, signs, vertex).Set(WideArithmetic.SubtractSigned576(axial[vertex], side > 0 ? half : Negate(half)));
            if (TrySlice(mask, distances, signs, projections, coreBound, root, weights, weightSigns))
                return;
        }
        for (int vertex = 0; vertex < 3; vertex++)
            Q(distances, signs, vertex).Set(WideArithmetic.AddSigned576(projections[vertex], coreBound));
        bool found = TrySlice(mask, distances, signs, axial, half, root, weights, weightSigns);
        System.Diagnostics.Debug.Assert(found);
    }

    private static void GetCapWeights(in TriangleCylinderGeometry geometry, int mask, WideAxis3 axis,
        WideAxis3 coreOffset, ReadOnlySpan<Signed576> projections, Signed576 bound,
        Span<ulong> weights, Span<int> weightSigns)
    {
        Span<ulong> distances = stackalloc ulong[6 * Words];
        Span<int> signs = stackalloc int[6];
        Span<ulong> root = stackalloc ulong[Words]; root.Clear();
        WideAxis3 transverse = new(WideArithmetic.Negate(axis.Z), default, axis.X);
        for (int vertex = 0; vertex < 3; vertex++)
            Q(distances, signs, vertex).Set(WideAxis3.Dot(geometry.Vertex(vertex), transverse));
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
            Q(weights, weightSigns, vertex).Set(Signed576.One);
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

    private static void GetEndpointCapWeights(in TriangleCylinderGeometry geometry, int mask, WideAxis3 coreOffset,
        int side, Span<ulong> weights, Span<int> weightSigns)
    {
        Span<ulong> rationalWeights = stackalloc ulong[3 * Words];
        TriangleCylinderGeometry endpoint = geometry.AtCoreRegion(coreOffset, side);
        TriangleCylinderWitnesses.GetCapWeights(endpoint, mask, rationalWeights);
        weights.Clear(); weightSigns.Clear();
        for (int vertex = 0; vertex < 3; vertex++)
        {
            Slot(rationalWeights, vertex).CopyTo(Q(weights, weightSigns, vertex).Rational);
            Q(weights, weightSigns, vertex).Signs[0] = IsZero(Slot(rationalWeights, vertex)) ? 0 : 1;
        }
    }

    private static void GetObliqueWeights(in TriangleCylinderGeometry geometry, int mask, WideAxis3 axis,
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
        Quad radical = Q(scratch, scratchSigns, 0), factor = Q(scratch, scratchSigns, 1);
        // sqrt(K)*tangent·(v-p) =
        // (tangent·v + cap*H*tangent.Y)*sqrt(K) - R*n.Y*tangent.Y.
        radical.Set(WideArithmetic.MultiplySigned320(compact.Y, tangent.Y));
        Scale(radical, Signed576.ExtendValue(geometry.Radius), factor);
        for (int vertex = 0; vertex < 3; vertex++)
        {
            Quad distance = Q(distances, signs, vertex);
            factor.CopyTo(distance, -1);
            Signed576 axial = WideArithmetic.MultiplySigned320(geometry.HalfHeight, tangent.Y);
            Signed576 value = WideArithmetic.AddSigned576(WideAxis3.Dot(geometry.Vertex(vertex), tangent),
                cap > 0 ? axial : Negate(axial));
            Import(value, distance.Radical); distance.Signs[1] = value.Sign;
        }
        bool found = TrySlice(mask, distances, signs, projections, bound, root, weights, weightSigns);
        System.Diagnostics.Debug.Assert(found);
    }

    /// <summary>
    /// Slices the selected feature by one exact plane, then admits its free
    /// interval coordinate. Quadratic weights retain A+B*sqrt(K) throughout;
    /// no intersection parameter or support point rounds before projection.
    /// </summary>
    private static bool TrySlice(int mask, Span<ulong> distances, Span<int> distanceSigns,
        ReadOnlySpan<Signed576> projections, Signed576 bound, ReadOnlySpan<ulong> root,
        Span<ulong> weights, Span<int> weightSigns)
    {
        Span<int> planeSigns = stackalloc int[3];
        Span<ulong> candidates = stackalloc ulong[12 * Words];
        Span<int> candidateSigns = stackalloc int[12];
        candidates.Clear(); candidateSigns.Clear();
        int count = 0;
        for (int vertex = 0; vertex < 3; vertex++)
        {
            planeSigns[vertex] = Q(distances, distanceSigns, vertex).Sign(root);
            if ((mask & (1 << vertex)) != 0 && planeSigns[vertex] == 0)
                Q(candidates, candidateSigns, 3 * count++ + vertex).Set(Signed576.One);
        }
        for (int edge = 0; edge < 3; edge++)
        {
            int next = (edge + 1) % 3;
            if ((mask & (1 << edge)) == 0 || (mask & (1 << next)) == 0
                || planeSigns[edge] * planeSigns[next] >= 0)
                continue;
            Q(distances, distanceSigns, next).CopyTo(Q(candidates, candidateSigns, 3 * count + edge), planeSigns[next]);
            Q(distances, distanceSigns, edge).CopyTo(Q(candidates, candidateSigns, 3 * count + next), planeSigns[edge]);
            count++;
        }
        System.Diagnostics.Debug.Assert(count <= 2);
        Span<ulong> values = stackalloc ulong[12 * Words];
        Span<int> valueSigns = stackalloc int[12];
        Quad denominator = Q(values, valueSigns, 0), limit = Q(values, valueSigns, 1);
        Quad comparison = Q(values, valueSigns, 2), first = Q(values, valueSigns, 4), second = Q(values, valueSigns, 5);
        Span<int> projectionSigns = stackalloc int[2];
        for (int index = 0; index < count; index++)
        {
            Span<ulong> candidate = candidates.Slice(index * 6 * Words, 6 * Words);
            Span<int> signs = candidateSigns.Slice(index * 6, 6);
            SumWeights(candidate, signs, denominator);
            Quad projection = Q(values, valueSigns, 4 + index);
            Sum(projections, candidate, signs, projection);
            projectionSigns[index] = projection.Sign(root);
            Scale(denominator, bound, limit);
            projection.CopyTo(comparison); comparison.Add(limit, -1);
            if (comparison.Sign(root) > 0)
                continue;
            projection.CopyTo(comparison); comparison.Add(limit);
            if (comparison.Sign(root) < 0)
                continue;
            candidate.CopyTo(weights); signs.CopyTo(weightSigns);
            return true;
        }
        if (count != 2 || projectionSigns[0] * projectionSigns[1] >= 0)
            return false;
        // Neither endpoint is admitted, but they straddle the interval. A
        // direct blend at coordinate zero keeps one quadratic field and
        // avoids nested rounded ratios (or a growing clipping polygon).
        Quad product = Q(values, valueSigns, 3);
        for (int vertex = 0; vertex < 3; vertex++)
        {
            Quad result = Q(weights, weightSigns, vertex);
            Multiply(Q(candidates, candidateSigns, vertex), second, root, result);
            result.MultiplySign(projectionSigns[1]);
            Multiply(Q(candidates, candidateSigns, vertex + 3), first, root, product);
            result.Add(product, projectionSigns[0]);
        }
        return true;
    }

    private static bool WithinRadius(in TriangleCylinderGeometry geometry, WideAxis3 axis,
        ReadOnlySpan<Signed576> projections, Signed576 bound, Span<ulong> weights, Span<int> signs)
    {
        Span<ulong> values = stackalloc ulong[16 * Words];
        Span<int> valueSigns = stackalloc int[16];
        Span<ulong> root = stackalloc ulong[Words]; root.Clear();
        Quad denominator = Q(values, valueSigns, 0), projection = Q(values, valueSigns, 1);
        Quad coordinate = Q(values, valueSigns, 2), scaled = Q(values, valueSigns, 3);
        Quad difference = Q(values, valueSigns, 4), squared = Q(values, valueSigns, 5);
        Quad sum = Q(values, valueSigns, 6), limit = Q(values, valueSigns, 7);
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

    private static void GetMaterials(in TriangleCylinderGeometry geometry, FixedTriangle triangle, WideAxis3 axis,
        Vector2d authoredAxis, Fixed64 coreLength,
        WideAxis3 normal, Signed576 metric, ReadOnlySpan<Signed576> projections, Signed576 bound,
        int cap, bool pole, ReadOnlySpan<ulong> root, Span<ulong> weights, Span<int> signs,
        ref Vector3d radialPoint, out Vector3d trianglePoint, out Vector3d coreAndAxialPoint,
        out FixedPointAnchorTerm3d exactTerm)
    {
        Span<ulong> values = stackalloc ulong[16 * Words];
        Span<int> valueSigns = stackalloc int[16];
        Quad denominator = Q(values, valueSigns, 0), projection = Q(values, valueSigns, 1);
        Quad coordinate = Q(values, valueSigns, 2), coreNumerator = Q(values, valueSigns, 3);
        Quad coreDenominator = Q(values, valueSigns, 4), product = Q(values, valueSigns, 5);
        Quad combinedNumerator = Q(values, valueSigns, 6), combinedDenominator = Q(values, valueSigns, 7);
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

    private static Fixed64 GetCorePresentation(Quad numerator, Quad denominator, int radialSign, ReadOnlySpan<ulong> root)
    {
        Fixed64 rounded = RoundRatio(numerator, denominator, root);
        Span<ulong> values = stackalloc ulong[2 * Words];
        Span<int> signs = stackalloc int[2];
        Quad difference = Q(values, signs, 0);
        Scale(denominator, Signed576.ExtendValue(Signed320.ExtendValue(Signed192.Raw(rounded))), difference);
        difference.MultiplySign(-1); difference.Add(numerator);
        int comparison = difference.Sign(root);
        if (radialSign >= 0 && comparison > 0)
            return Fixed64.FromRaw(rounded.m_rawValue + 1);
        if (radialSign < 0 && comparison < 0)
            return Fixed64.FromRaw(rounded.m_rawValue - 1);
        return rounded;
    }

    private static void ClampProjection(Quad denominator, Quad projection, Signed576 bound, ReadOnlySpan<ulong> root)
    {
        Span<ulong> values = stackalloc ulong[4 * Words];
        Span<int> signs = stackalloc int[4];
        Quad limit = Q(values, signs, 0), difference = Q(values, signs, 1);
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

    private static Fixed64 RoundTriangleCoordinate(Fixed64 a, Fixed64 b, Fixed64 c,
        Span<ulong> weights, Span<int> signs, Quad denominator, ReadOnlySpan<ulong> root)
    {
        Span<Signed576> coordinates = stackalloc Signed576[3]
        {
            Signed576.ExtendValue(Signed320.ExtendValue(Signed192.Raw(a))),
            Signed576.ExtendValue(Signed320.ExtendValue(Signed192.Raw(b))),
            Signed576.ExtendValue(Signed320.ExtendValue(Signed192.Raw(c)))
        };
        Span<ulong> values = stackalloc ulong[2 * Words];
        Span<int> valueSigns = stackalloc int[2];
        Quad numerator = Q(values, valueSigns, 0);
        Sum(coordinates, weights, signs, numerator);
        return RoundRatio(numerator, denominator, root,
            Math.Min(a.m_rawValue, Math.Min(b.m_rawValue, c.m_rawValue)),
            Math.Max(a.m_rawValue, Math.Max(b.m_rawValue, c.m_rawValue)));
    }

    private static Fixed64 RoundRatio(Quad numerator, Quad denominator, ReadOnlySpan<ulong> root,
        long low = long.MinValue, long high = long.MaxValue, long parityOffset = 0)
    {
        if (numerator.Signs[1] == 0 && denominator.Signs[1] == 0 && (parityOffset & 1) == 0)
        {
            bool represented = Fixed64.TryGetSignedRawRatio(numerator.Rational, denominator.Rational,
                numerator.Signs[0] < 0, out Fixed64 result);
            System.Diagnostics.Debug.Assert(represented);
            return result;
        }
        Span<ulong> values = stackalloc ulong[4 * Words];
        Span<int> signs = stackalloc int[4];
        Quad doubled = Q(values, signs, 0), query = Q(values, signs, 1);
        numerator.CopyTo(doubled); doubled.Add(numerator);
        while (low < high)
        {
            ulong span = unchecked((ulong)high - (ulong)low);
            long midpoint = unchecked((long)((ulong)low + (span >> 1) + (span & 1)));
            Signed192 threshold = WideArithmetic.AddSigned192(Signed192.Signed(midpoint), Signed192.Signed(midpoint));
            Scale(denominator, Signed576.ExtendValue(Signed320.ExtendValue(threshold)), query);
            query.MultiplySign(-1); query.Add(doubled);
            if (query.Sign(root) >= 0) low = midpoint;
            else high = midpoint - 1;
        }
        Signed192 half = WideArithmetic.AddSigned192(WideArithmetic.AddSigned192(Signed192.Signed(low), Signed192.Signed(low)), Signed192.Signed(1));
        Scale(denominator, Signed576.ExtendValue(Signed320.ExtendValue(half)), query);
        query.MultiplySign(-1); query.Add(doubled);
        int comparison = query.Sign(root);
        return Fixed64.FromRaw(low + (comparison > 0 || comparison == 0 && ((low ^ parityOffset) & 1) != 0 ? 1 : 0));
    }

    private static void SumCoordinate(in TriangleCylinderGeometry geometry, Span<ulong> weights,
        Span<int> signs, int component, Quad result)
    {
        Span<Signed576> coordinates = stackalloc Signed576[3];
        for (int vertex = 0; vertex < 3; vertex++)
            coordinates[vertex] = Signed576.ExtendValue(Component(geometry.Vertex(vertex), component));
        Sum(coordinates, weights, signs, result);
    }

    private static void Sum(ReadOnlySpan<Signed576> coordinates, Span<ulong> weights, Span<int> signs, Quad result)
    {
        Span<ulong> values = stackalloc ulong[2 * Words];
        Span<int> productSigns = stackalloc int[2];
        Quad product = Q(values, productSigns, 0);
        result.Clear();
        for (int vertex = 0; vertex < 3; vertex++)
        {
            Scale(Q(weights, signs, vertex), coordinates[vertex], product);
            result.Add(product);
        }
    }

    private static void SumWeights(Span<ulong> weights, Span<int> signs, Quad result)
    {
        result.Clear();
        for (int vertex = 0; vertex < 3; vertex++)
            result.Add(Q(weights, signs, vertex));
    }

    private static void Scale(Quad value, Signed576 scalar, Quad result)
    {
        Span<ulong> magnitude = stackalloc ulong[Words];
        Import(scalar, magnitude);
        WideArithmetic.MultiplyMagnitudes(value.Rational, magnitude, result.Rational);
        WideArithmetic.MultiplyMagnitudes(value.Radical, magnitude, result.Radical);
        result.Signs[0] = IsZero(result.Rational) ? 0 : value.Signs[0] * scalar.Sign;
        result.Signs[1] = IsZero(result.Radical) ? 0 : value.Signs[1] * scalar.Sign;
    }

    private static void Multiply(Quad first, Quad second, ReadOnlySpan<ulong> root, Quad result)
    {
        Span<ulong> scratch = stackalloc ulong[3 * Words];
        Span<ulong> rational = Slot(scratch, 0), radical = Slot(scratch, 1), product = Slot(scratch, 2);
        WideArithmetic.MultiplyMagnitudes(first.Rational, second.Rational, rational);
        int rationalSign = IsZero(rational) ? 0 : first.Signs[0] * second.Signs[0];
        WideArithmetic.MultiplyMagnitudes(first.Radical, second.Radical, product);
        WideArithmetic.MultiplyMagnitudes(product, root, radical);
        Add(radical, first.Signs[1] * second.Signs[1], rational, ref rationalSign);
        WideArithmetic.MultiplyMagnitudes(first.Rational, second.Radical, radical);
        int radicalSign = IsZero(radical) ? 0 : first.Signs[0] * second.Signs[1];
        WideArithmetic.MultiplyMagnitudes(first.Radical, second.Rational, product);
        Add(product, first.Signs[1] * second.Signs[0], radical, ref radicalSign);
        rational.CopyTo(result.Rational); radical.CopyTo(result.Radical);
        result.Signs[0] = rationalSign; result.Signs[1] = radicalSign;
    }

    private static void MultiplyRoot(Quad value, ReadOnlySpan<ulong> root, Quad result)
    {
        WideArithmetic.MultiplyMagnitudes(value.Radical, root, result.Rational);
        value.Rational.CopyTo(result.Radical);
        result.Signs[0] = IsZero(result.Rational) ? 0 : value.Signs[1];
        result.Signs[1] = value.Signs[0];
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
    private static Quad Q(Span<ulong> values, Span<int> signs, int index) =>
        new(values.Slice(index * 2 * Words, 2 * Words), signs.Slice(index * 2, 2));

    // Only the seam needs quadratic barycentric weights. Their compact edge
    // normal is <230 bits, plane-distance coefficients <732, and the single
    // zero-coordinate blend stays below 1740 bits. Combined-coordinate and
    // ratio-rounding products remain below 2520, within the shared 40 words.
    // Rational cap projection weights stay below 832 bits; its squared
    // radius check, including the raw-axis metric, stays below 2300 bits.
    private readonly ref struct Quad
    {
        internal readonly Span<ulong> Values;
        internal readonly Span<int> Signs;
        internal Span<ulong> Rational => Values[..Words];
        internal Span<ulong> Radical => Values[Words..];
        internal Quad(Span<ulong> values, Span<int> signs) { Values = values; Signs = signs; }
        internal void Clear() { Values.Clear(); Signs.Clear(); }
        internal void Set(Signed576 value)
        {
            Clear(); Import(value, Rational); Signs[0] = value.Sign;
        }
        internal void CopyTo(Quad destination, int multiplier = 1)
        {
            Values.CopyTo(destination.Values);
            destination.Signs[0] = Signs[0] * multiplier; destination.Signs[1] = Signs[1] * multiplier;
        }
        internal void MultiplySign(int sign) { Signs[0] *= sign; Signs[1] *= sign; }
        internal void Add(Quad value, int multiplier = 1)
        {
            int rationalSign = Signs[0], radicalSign = Signs[1];
            CylinderContactAlgebra.Add(value.Rational, value.Signs[0] * multiplier, Rational, ref rationalSign);
            CylinderContactAlgebra.Add(value.Radical, value.Signs[1] * multiplier, Radical, ref radicalSign);
            Signs[0] = rationalSign; Signs[1] = radicalSign;
        }
        internal int Sign(ReadOnlySpan<ulong> root) =>
            WideConvexPrismRelations.GetConvexContactCandidateQuadraticSign(Rational, Signs[0], Radical, Signs[1], root);
    }
}
