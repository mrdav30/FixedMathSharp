//=======================================================================
// SegmentConeSurfaceCandidates.SideWitnesses.cs
//=======================================================================
// MIT License, Copyright (c) 2024-present David Oravsky (mrdav30)
// See LICENSE file in the project root for full license information.
//=======================================================================
using System;
using static FixedMathSharp.Geometry.CylinderContactAlgebra;
using static FixedMathSharp.Geometry.ContactQuadratic;

namespace FixedMathSharp.Geometry;

/// <content>Final materialization of admitted side points and exact family representatives.</content>
internal static partial class SegmentConeSurfaceCandidates
{
    internal static FixedContactAnchors GetSideContact(in SegmentConeSurfaceCandidate candidate)
    {
        int Words = candidate.Input.Words;
        var geometry = new SegmentConeSurfaceGeometry(candidate.Input);
        Span<ulong> values = stackalloc ulong[24 * Words];
        Span<int> signs = stackalloc int[24];
        Span<ulong> root = stackalloc ulong[Words];
        bool normal = BuildSideNormal(geometry, candidate.RootOrdinal, candidate.Chart, values, signs, root);
        bool foot = TryBuildSideFoot(geometry, candidate.RootOrdinal, values, signs, root, out bool interval);
        System.Diagnostics.Debug.Assert(normal && foot && !interval);
        return MaterializeSidePoint(geometry, values, signs, root);
    }

    private static FixedContactAnchors MaterializeSidePoint(in SegmentConeSurfaceGeometry geometry,
        Span<ulong> values, Span<int> signs, ReadOnlySpan<ulong> root)
    {
        int Words = geometry.Words;
        // Normal coefficients <462 bits, metric <925, t coefficients <662.
        // Complete q numerators stay <1790 bits, denominators <1720 after
        // the frame scale. Parallel slices use the reduced rational normal.
        Span<ulong> work = stackalloc ulong[12 * Words];
        Span<int> workSigns = stackalloc int[12];
        ContactQuadratic point = At(work, workSigns, 0, Words), product = At(work, workSigns, 1, Words);
        ContactQuadratic q = At(work, workSigns, 2, Words), temporary = At(work, workSigns, 3, Words);
        ContactQuadratic common = At(work, workSigns, 4, Words), qDenominator = At(work, workSigns, 5, Words);
        ContactQuadratic numerator = At(values, signs, 5, Words), denominator = At(values, signs, 6, Words);
        ContactQuadratic metric = At(values, signs, 7, Words), gap = At(values, signs, 8, Words);
        Multiply(denominator, metric, root, common);
        Scale(common, Signed832.ExtendValue(geometry.RawScale), qDenominator);
        Vector3d first = RoundSourceSegmentPoint(geometry, numerator, denominator, root), second = default;
        for (int axis = 0; axis < 3; axis++)
        {
            Scale(denominator, Signed832.ExtendValue(Component(geometry.A, axis)), point);
            Scale(numerator, Signed832.ExtendValue(Component(geometry.Edge, axis)), product); point.Add(product);
            Multiply(point, metric, root, q);
            Multiply(gap, At(values, signs, axis, Words), root, product);
            Multiply(product, denominator, root, temporary); q.Add(temporary, -1);
            second[axis] = RoundRatio(q, qDenominator, root);
        }
        Vector3d normal = MaterializeQuadraticNormal(values, signs, root, geometry.Input.ConeRotation);
        Span<ulong> scale = stackalloc ulong[Words];
        Import(Signed576.ExtendValue(geometry.RawScale), scale);
        System.Diagnostics.Debug.Assert(metric.Signs[1] == 0);
        bool represented = TryRoundRootRatio(gap, metric, root, metric.Rational, scale, out Fixed64 depth);
        return new FixedContactAnchors(new FixedPointAnchor(geometry.Input.Origin, geometry.Input.Rotation, first),
            new FixedPointAnchor(geometry.Input.Center, geometry.Input.ConeRotation, second), normal,
            represented ? depth : Fixed64.MaxValue, !represented);
    }
}
