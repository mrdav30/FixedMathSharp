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
    internal static FixedContactAnchors GetSideContact(SegmentConeSurfaceCandidate candidate)
    {
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
        // Normal coefficients <462 bits, metric <925, t coefficients <662.
        // Complete q numerators stay <1790 bits, denominators <1720 after
        // the frame scale. Parallel slices use the reduced rational normal.
        Span<ulong> work = stackalloc ulong[12 * Words];
        Span<int> workSigns = stackalloc int[12];
        ContactQuadratic point = At(work, workSigns, 0), product = At(work, workSigns, 1);
        ContactQuadratic q = At(work, workSigns, 2), temporary = At(work, workSigns, 3);
        ContactQuadratic common = At(work, workSigns, 4), qDenominator = At(work, workSigns, 5);
        ContactQuadratic numerator = At(values, signs, 5), denominator = At(values, signs, 6);
        ContactQuadratic metric = At(values, signs, 7), gap = At(values, signs, 8);
        Multiply(denominator, metric, root, common);
        Scale(common, Signed576.ExtendValue(Signed320.ExtendValue(geometry.RawScale)), qDenominator);
        Vector3d first = RoundSegmentPoint(geometry.Input.Segment, numerator, denominator, root), second = default;
        for (int axis = 0; axis < 3; axis++)
        {
            Scale(denominator, Signed576.ExtendValue(Component(geometry.A, axis)), point);
            Scale(numerator, Signed576.ExtendValue(Component(geometry.Edge, axis)), product); point.Add(product);
            Multiply(point, metric, root, q);
            Multiply(gap, At(values, signs, axis), root, product);
            Multiply(product, denominator, root, temporary); q.Add(temporary, -1);
            second[axis] = RoundRatio(q, qDenominator, root);
        }
        Vector3d normal = MaterializeQuadraticNormal(values, signs, root, geometry.Input.ConeRotation);
        Span<ulong> scale = stackalloc ulong[Words];
        Import(Signed320.ExtendValue(geometry.RawScale), scale);
        System.Diagnostics.Debug.Assert(metric.Signs[1] == 0);
        bool represented = TryRoundRootRatio(gap, metric, root, metric.Rational, scale, out Fixed64 depth);
        return new FixedContactAnchors(new FixedPointAnchor(geometry.Input.Origin, geometry.Input.Rotation, first),
            new FixedPointAnchor(geometry.Input.Center, geometry.Input.ConeRotation, second), normal,
            represented ? depth : Fixed64.MaxValue, !represented);
    }
}
