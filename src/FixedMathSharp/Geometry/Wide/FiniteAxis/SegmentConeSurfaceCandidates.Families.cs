//=======================================================================
// SegmentConeSurfaceCandidates.Families.cs
//=======================================================================
// MIT License, Copyright (c) 2024-present David Oravsky (mrdav30)
// See LICENSE file in the project root for full license information.
//=======================================================================
using System;
using static FixedMathSharp.Geometry.CylinderContactAlgebra;
using static FixedMathSharp.Geometry.ContactQuadratic;

namespace FixedMathSharp.Geometry;

/// <content>Explicit finite parameter domains for degenerate support families.</content>
internal static partial class SegmentConeSurfaceCandidates
{
    internal static bool TryGetIntervalEndpointContact(SegmentConeSurfaceCandidate candidate, bool upper, out FixedContactAnchors contact)
    {
        contact = default;
        if (candidate.Family != ConeSurfaceFamily.SegmentInterval
            || candidate.Feature == ConeSurfaceFeature.Side && candidate.Input.Radius == Fixed64.Zero)
            return false;
        System.Diagnostics.Debug.Assert(candidate.Feature == ConeSurfaceFeature.Base || candidate.Feature == ConeSurfaceFeature.Side);
        var geometry = new SegmentConeSurfaceGeometry(candidate.Input);
        Span<ulong> values = stackalloc ulong[24 * Words];
        Span<int> signs = stackalloc int[24];
        Span<ulong> root = stackalloc ulong[Words];
        if (candidate.Feature == ConeSurfaceFeature.Side)
        {
            bool normal = BuildSideNormal(geometry, -1, candidate.Chart, values, signs, root);
            bool foot = TryBuildSideFoot(geometry, -1, values, signs, root, out bool interval);
#if DEBUG
            System.Diagnostics.Debug.Assert(normal && foot && interval && IsZero(root));
#endif
        }
        ContactQuadratic numerator = At(values, signs, 5), denominator = At(values, signs, 6);
        bool exists = ConeSectionPoint.TryGetSegmentParameter(candidate.Input.Segment, geometry.Finite, upper, numerator, denominator, root);
        System.Diagnostics.Debug.Assert(exists);
        if (candidate.Feature == ConeSurfaceFeature.Base)
            contact = MaterializeBaseContact(geometry, numerator, denominator, root);
        else
        {
            ClipSideEndpoint(geometry, upper, values, signs, root);
            bool admitted = AdmitsSideParameter(geometry, values, signs, root);
            System.Diagnostics.Debug.Assert(admitted);
            contact = MaterializeSidePoint(geometry, values, signs, root);
        }
        return true;
    }

    private static void AccumulateBase(in SegmentConeSurfaceGeometry geometry, ref ConeSurfaceSelection selection)
    {
        if (geometry.Edge.Y.IsZero)
        {
            selection.Add(new SegmentConeSurfaceCandidate(geometry.Input, ConeSurfaceFeature.Base, ConeSurfaceFamily.SegmentInterval));
            return;
        }
        int endpoint = geometry.Edge.Y.Sign > 0 ? 1 : 0;
        if (geometry.ContainsParameter(endpoint == 0 ? Fixed64.Zero : Fixed64.One))
            selection.Add(new SegmentConeSurfaceCandidate(geometry.Input, ConeSurfaceFeature.Base,
                ConeSurfaceFamily.None, 0, endpoint, endpoint == 0 ? ConeSurfacePointLocation.Start : ConeSurfacePointLocation.End));
    }

    internal static FixedContactAnchors GetBaseContact(SegmentConeSurfaceCandidate candidate) =>
        MaterializeBaseContact(candidate.Input, candidate.RootOrdinal == 0 ? Fixed64.Zero : Fixed64.One);

    internal static bool TryGetIntervalContact(SegmentConeSurfaceCandidate candidate, Fixed64 parameter,
        out FixedContactAnchors contact)
    {
        contact = default;
        if (candidate.Feature == ConeSurfaceFeature.Side)
            return TryGetSideIntervalContact(candidate, parameter, out contact);
        if (candidate.Family != ConeSurfaceFamily.SegmentInterval || candidate.Feature != ConeSurfaceFeature.Base)
            return false;
        var geometry = new SegmentConeSurfaceGeometry(candidate.Input);
        if (!geometry.ContainsParameter(parameter))
            return false;
        contact = MaterializeBaseContact(candidate.Input, parameter);
        return true;
    }

    private static FixedContactAnchors MaterializeBaseContact(SegmentConeSurfaceInput input, Fixed64 parameter)
    {
        var geometry = new SegmentConeSurfaceGeometry(input);
        Span<ulong> values = stackalloc ulong[4 * Words];
        Span<int> signs = stackalloc int[4];
        ContactQuadratic numerator = At(values, signs, 0), denominator = At(values, signs, 1);
        numerator.Set(Signed576.ExtendValue(Signed320.ExtendValue(Signed192.Raw(parameter))));
        denominator.Set(Signed576.ExtendValue(Signed320.ExtendValue(Signed192.Raw(Fixed64.One))));
        return MaterializeBaseContact(geometry, numerator, denominator, ReadOnlySpan<ulong>.Empty);
    }

    private static FixedContactAnchors MaterializeBaseContact(in SegmentConeSurfaceGeometry geometry,
        ContactQuadratic numerator, ContactQuadratic denominator, ReadOnlySpan<ulong> root)
    {
        Span<ulong> values = stackalloc ulong[8 * Words];
        Span<int> signs = stackalloc int[8];
        ContactQuadratic coordinate = At(values, signs, 0), product = At(values, signs, 1), divisor = At(values, signs, 2), gap = At(values, signs, 3);
        Scale(denominator, Signed576.ExtendValue(Signed320.ExtendValue(geometry.RawScale)), divisor);
        Vector3d radial = default;
        for (int axis = 0; axis < 3; axis++)
        {
            Scale(denominator, Signed576.ExtendValue(Component(geometry.A, axis)), coordinate);
            Scale(numerator, Signed576.ExtendValue(Component(geometry.Edge, axis)), product); coordinate.Add(product);
            if (axis == 0) radial.X = RoundRatio(coordinate, divisor, root);
            else if (axis == 2) radial.Z = RoundRatio(coordinate, divisor, root);
            else
            {
                coordinate.CopyTo(gap);
                Scale(denominator, Signed576.ExtendValue(geometry.Finite.ShapeFrame.Cap), product); gap.Add(product);
            }
        }
        Fixed64 depth = RoundRatio(gap, divisor, root, 0, long.MaxValue);
        Vector3d normal = MaterializeRationalNormal(default,
            Signed832.ExtendValue(Signed576.ExtendValue(Signed320.ExtendValue(Signed192.Signed(1)))), default, geometry.Input.ConeRotation);
        return new FixedContactAnchors(
            new FixedPointAnchor(geometry.Input.Origin, geometry.Input.Rotation,
                RoundSegmentPoint(geometry.Input.Segment, numerator, denominator, root)),
            TriangleCircularGeometry.GetSupport(geometry.Input.Center, geometry.Input.ConeRotation,
                Signed192.Raw(geometry.Input.Height), radial, 1), normal, depth, false);
    }

    private static Vector3d RoundSegmentPoint(FixedSegment segment, ContactQuadratic numerator,
        ContactQuadratic denominator, ReadOnlySpan<ulong> root)
    {
        Span<ulong> values = stackalloc ulong[4 * Words];
        Span<int> signs = stackalloc int[4];
        ContactQuadratic point = At(values, signs, 0), product = At(values, signs, 1);
        Vector3d result = default;
        for (int axis = 0; axis < 3; axis++)
        {
            Scale(denominator, Signed576.ExtendValue(Signed320.ExtendValue(Signed192.Raw(segment.Start[axis]))), point);
            Scale(numerator, Signed576.ExtendValue(Signed320.ExtendValue(WideArithmetic.Difference(segment.End[axis], segment.Start[axis]))), product); point.Add(product);
            result[axis] = RoundRatio(point, denominator, root);
        }
        return result;
    }

    private static void ClipSideEndpoint(in SegmentConeSurfaceGeometry geometry, bool upper,
        Span<ulong> normals, Span<int> normalSigns, ReadOnlySpan<ulong> root)
    {
        Span<ulong> values = stackalloc ulong[8 * Words];
        Span<int> signs = stackalloc int[8];
        ContactQuadratic bound = At(values, signs, 0), denominator = At(values, signs, 1), constant = At(values, signs, 2), product = At(values, signs, 3);
        ContactQuadratic metric = At(normals, normalSigns, 7);
        Scale(metric, Signed576.ExtendValue(geometry.A.Y), constant);
        Multiply(At(normals, normalSigns, 8), At(normals, normalSigns, 1), root, product); constant.Add(product, -1);
        int direction = geometry.Edge.Y.Sign;
        Scale(metric, Signed576.ExtendValue(geometry.Finite.ShapeFrame.Cap), bound);
        bound.MultiplySign((upper ? 1 : -1) * direction); bound.Add(constant, -1); bound.MultiplySign(direction);
        Scale(metric, Signed576.ExtendValue(geometry.Edge.Y), denominator); denominator.MultiplySign(direction);
        ContactQuadratic current = At(normals, normalSigns, 5), currentD = At(normals, normalSigns, 6);
        int comparison = CompareRatios(bound, denominator, root, current, currentD, root);
        if (upper ? comparison < 0 : comparison > 0)
        {
            bound.CopyTo(current); denominator.CopyTo(currentD);
        }
    }
}
