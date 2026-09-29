//=======================================================================
// MIT License, Copyright (c) 2024-present David Oravsky (mrdav30)
// See LICENSE file in the project root for full license information.
//=======================================================================
using System;
using static FixedMathSharp.Geometry.CylinderContactAlgebra;
using static FixedMathSharp.Geometry.ContactQuadratic;
using static FixedMathSharp.Geometry.TriangleQuadraticSlice;

namespace FixedMathSharp.Geometry;

/// <summary>Matched authored-frame witnesses on the cone's free generator.</summary>
internal static class TriangleConeWitnesses
{
    internal static void GetGenerator(in TriangleCircularGeometry geometry, FixedTriangle triangle,
        Fixed64 height, Fixed64 radius, int mask, ReadOnlySpan<ulong> normal, ReadOnlySpan<int> normalSigns,
        Vector3d center, FixedQuaternion rotation, out Vector3d trianglePoint, out FixedPointAnchor coneAnchor)
    {
        ReadOnlySpan<ulong> root = Slot(normal, 6);
        // Only compact face/generator owners enter here: effective normal
        // height <458 bits. Oversized projected axes explicitly defer equality.
        Span<ulong> d = stackalloc ulong[6 * Words];
        Span<int> dSigns = stackalloc int[6];
        Span<ulong> distances = stackalloc ulong[6 * Words];
        Span<int> distanceSigns = stackalloc int[6];
        Span<ulong> projections = stackalloc ulong[6 * Words];
        Span<int> projectionSigns = stackalloc int[6];
        Span<ulong> weights = stackalloc ulong[6 * Words];
        Span<int> weightSigns = stackalloc int[6];
        Span<ulong> work = stackalloc ulong[14 * Words];
        Span<int> workSigns = stackalloc int[14];
        ContactQuadratic component = At(work, workSigns, 0), product = At(work, workSigns, 1);
        ContactQuadratic bound = At(work, workSigns, 2), sum = At(work, workSigns, 3);
        ContactQuadratic denominator = At(work, workSigns, 4), tNumerator = At(work, workSigns, 5), tDenominator = At(work, workSigns, 6);
        Signed192 h = Signed192.Raw(height), r = Signed192.Raw(radius);
        Signed576 hh = Signed576.ExtendValue(WideArithmetic.MultiplySigned192(h, h));
        Signed576 rr = Signed576.ExtendValue(WideArithmetic.MultiplySigned192(r, r));
        Signed576 length = WideArithmetic.AddSigned576(hh, rr);
        for (int axis = 0; axis < 3; axis++)
        {
            TriangleConeGeneratorFeatures.Component(normal, normalSigns, axis, component);
            Scale(component, axis == 1 ? hh : Negate(rr), At(d, dSigns, axis));
        }
        TriangleConeGeneratorFeatures.Component(normal, normalSigns, 1, component);
        Scale(component, Negate(Signed576.ExtendValue(Signed320.ExtendValue(h))), product);
        Scale(product, length, component);
        Scale(component, Signed576.ExtendValue(Signed320.ExtendValue(geometry.RawScale)), bound);
        System.Diagnostics.Debug.Assert(bound.Sign(root) > 0);
        for (int vertex = 0; vertex < 3; vertex++)
        {
            WideAxis3 p = geometry.Vertex(vertex);
            ContactQuadratic distance = At(distances, distanceSigns, vertex);
            TriangleConeGeneratorFeatures.Component(normal, normalSigns, 0, component);
            Scale(component, Signed576.ExtendValue(p.Z), distance);
            TriangleConeGeneratorFeatures.Component(normal, normalSigns, 2, component);
            Scale(component, Signed576.ExtendValue(p.X), product);
            distance.Add(product, -1);
            sum.Clear();
            for (int axis = 0; axis < 3; axis++)
            {
                Signed320 coordinate = CylinderContactAlgebra.Component(p, axis);
                if (axis == 1) coordinate = WideArithmetic.SubtractSigned320(coordinate, geometry.HalfHeight);
                Scale(At(d, dSigns, axis), Signed576.ExtendValue(coordinate), product);
                sum.Add(product);
            }
            ContactQuadratic projection = At(projections, projectionSigns, vertex);
            sum.CopyTo(projection); projection.Add(sum); projection.Add(bound, -1);
        }
        weights.Clear(); weightSigns.Clear();
        bool found = TryGetWeights(mask, distances, distanceSigns, projections, projectionSigns, bound, root,
            weights, weightSigns, out bool centered);
        System.Diagnostics.Debug.Assert(found);
        SumWeights(weights, weightSigns, denominator);
        trianglePoint = new Vector3d(
            RoundTriangleCoordinate(triangle.A.X, triangle.B.X, triangle.C.X, weights, weightSigns, denominator, root),
            RoundTriangleCoordinate(triangle.A.Y, triangle.B.Y, triangle.C.Y, weights, weightSigns, denominator, root),
            RoundTriangleCoordinate(triangle.A.Z, triangle.B.Z, triangle.C.Z, weights, weightSigns, denominator, root));
        if (centered)
        {
            // The larger blended triangle weights must not be reprojected:
            // their semantic generator parameter is exactly one half.
            tNumerator.Set(Signed576.ExtendValue(Signed320.ExtendValue(Signed192.Signed(1))));
            tDenominator.Set(Signed576.ExtendValue(Signed320.ExtendValue(Signed192.Signed(2))));
        }
        else
        {
            SumWeighted(projections, projectionSigns, weights, weightSigns, root, tNumerator);
            Multiply(denominator, bound, root, tDenominator);
            tNumerator.Add(tDenominator);
            tDenominator.CopyTo(product); tDenominator.Add(product);
        }
        int endpoint = tNumerator.Sign(root) == 0 ? 1 : 0;
        tNumerator.CopyTo(component); component.Add(tDenominator, -1);
        if (component.Sign(root) == 0) endpoint = -1;
        Vector3d conePoint = new(
            RoundRadial(0, normal, normalSigns, rr, h, tNumerator, tDenominator, root, radius),
            RoundAxial(h, tNumerator, tDenominator, root),
            RoundRadial(2, normal, normalSigns, rr, h, tNumerator, tDenominator, root, radius));
        coneAnchor = endpoint == 0 ? new FixedPointAnchor(center, rotation, conePoint)
            : TriangleCircularGeometry.GetSupport(center, rotation, Signed192.Raw(height),
                new Vector3d(conePoint.X, Fixed64.Zero, conePoint.Z), -endpoint);
    }

    private static Fixed64 RoundRadial(int axis, ReadOnlySpan<ulong> normal, ReadOnlySpan<int> signs,
        Signed576 radiusSquared, Signed192 height, ContactQuadratic tNumerator, ContactQuadratic tDenominator,
        ReadOnlySpan<ulong> root, Fixed64 radius)
    {
        // Endpoint weights <658 bits give t numerators <1448 and final
        // coordinate numerators <2040. Center blends use t=1/2 directly.
        Span<ulong> work = stackalloc ulong[8 * Words];
        Span<int> workSigns = stackalloc int[8];
        ContactQuadratic component = At(work, workSigns, 0), scaled = At(work, workSigns, 1);
        ContactQuadratic numerator = At(work, workSigns, 2), denominator = At(work, workSigns, 3);
        TriangleConeGeneratorFeatures.Component(normal, signs, axis, component);
        Scale(component, radiusSquared, scaled);
        Multiply(scaled, tNumerator, root, numerator);
        TriangleConeGeneratorFeatures.Component(normal, signs, 1, component);
        Scale(component, Signed576.ExtendValue(Signed320.ExtendValue(height)), scaled);
        Multiply(scaled, tDenominator, root, denominator);
        numerator.MultiplySign(-1); denominator.MultiplySign(-1);
        return RoundRatio(numerator, denominator, root, -radius.m_rawValue, radius.m_rawValue);
    }

    private static Fixed64 RoundAxial(Signed192 height, ContactQuadratic tNumerator,
        ContactQuadratic tDenominator, ReadOnlySpan<ulong> root)
    {
        Span<ulong> work = stackalloc ulong[6 * Words];
        Span<int> signs = stackalloc int[6];
        ContactQuadratic difference = At(work, signs, 0), numerator = At(work, signs, 1), denominator = At(work, signs, 2);
        tDenominator.CopyTo(difference); difference.Add(tNumerator, -1); difference.Add(tNumerator, -1);
        Scale(difference, Signed576.ExtendValue(Signed320.ExtendValue(height)), numerator);
        tDenominator.CopyTo(denominator); denominator.Add(tDenominator);
        return RoundRatio(numerator, denominator, root);
    }

    private static Signed576 Negate(Signed576 value) => WideArithmetic.SubtractSigned576(default, value);
}
