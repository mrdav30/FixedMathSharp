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
    private static void AccumulateBase(in SegmentConeSurfaceGeometry geometry, ref ConeSurfaceSelection selection)
    {

        if (geometry.Edge.Y.IsZero)
        {
            selection.Add(new SegmentConeSurfaceCandidate(geometry.Input, ConeSurfaceFeature.Base, ConeSurfaceFamily.SegmentInterval));
            return;
        }
        int endpoint = geometry.Edge.Y.Sign > 0 ? 1 : 0;
        Vector3d point = endpoint == 0 ? geometry.Input.Segment.Start : geometry.Input.Segment.End;
        if (geometry.Finite.ContainsTransformedPoint(geometry.Finite.Transform(point)))
            selection.Add(new SegmentConeSurfaceCandidate(geometry.Input, ConeSurfaceFeature.Base,
                ConeSurfaceFamily.None, 0, endpoint, endpoint == 0 ? ConeSurfacePointLocation.Start : ConeSurfacePointLocation.End));
    }

    internal static FixedContactAnchors GetBaseContact(in SegmentConeSurfaceCandidate candidate) =>
        MaterializeBaseContact(candidate.Input, candidate.RootOrdinal == 0 ? Fixed64.Zero : Fixed64.One);

    private static FixedContactAnchors MaterializeBaseContact(in SegmentConeSurfaceInput input, Fixed64 parameter)
    {
        int Words = input.Words;
        var geometry = new SegmentConeSurfaceGeometry(input);
        Span<ulong> values = stackalloc ulong[4 * Words];
        Span<int> signs = stackalloc int[4];
        ContactQuadratic numerator = At(values, signs, 0, Words), denominator = At(values, signs, 1, Words);
        numerator.Set(Signed832.ExtendValue(Signed576.ExtendValue(Signed192.Raw(parameter))));
        denominator.Set(Signed832.ExtendValue(Signed576.ExtendValue(Signed192.Raw(Fixed64.One))));
        return MaterializeBaseContact(geometry, numerator, denominator, ReadOnlySpan<ulong>.Empty);
    }

    private static FixedContactAnchors MaterializeBaseContact(in SegmentConeSurfaceGeometry geometry,
        ContactQuadratic numerator, ContactQuadratic denominator, ReadOnlySpan<ulong> root)
    {
        int Words = geometry.Words;
        Span<ulong> values = stackalloc ulong[8 * Words];
        Span<int> signs = stackalloc int[8];
        ContactQuadratic coordinate = At(values, signs, 0, Words), product = At(values, signs, 1, Words), divisor = At(values, signs, 2, Words), gap = At(values, signs, 3, Words);
        Scale(denominator, Signed832.ExtendValue(geometry.RawScale), divisor);
        Vector3d radial = default;
        for (int axis = 0; axis < 3; axis++)
        {
            Scale(denominator, Signed832.ExtendValue(Component(geometry.A, axis)), coordinate);
            Scale(numerator, Signed832.ExtendValue(Component(geometry.Edge, axis)), product); coordinate.Add(product);
            if (axis == 0) radial.X = RoundRatio(coordinate, divisor, root);
            else if (axis == 2) radial.Z = RoundRatio(coordinate, divisor, root);
            else
            {
                coordinate.CopyTo(gap);
                Scale(denominator, Signed832.ExtendValue(geometry.Cap), product); gap.Add(product);
            }
        }
        Fixed64 depth = RoundRatio(gap, divisor, root, 0, long.MaxValue);
        Vector3d normal = MaterializeRationalNormal(default,
            Signed832.ExtendValue(Signed576.ExtendValue(Signed192.Signed(1))), default, geometry.Input.ConeRotation);
        return new FixedContactAnchors(
            new FixedPointAnchor(geometry.Input.Origin, geometry.Input.Rotation,
                RoundSourceSegmentPoint(geometry, numerator, denominator, root)),
            TriangleCircularGeometry.GetSupport(geometry.Input.Center, geometry.Input.ConeRotation,
                Signed192.Raw(geometry.Input.Height), radial, 1), normal, depth, false);
    }

    private static Vector3d RoundSourceSegmentPoint(in SegmentConeSurfaceGeometry geometry,
        ContactQuadratic numerator, ContactQuadratic denominator, ReadOnlySpan<ulong> root)
    {
        if (!geometry.Input.IsAffine) return RoundSegmentPoint(geometry.Input.Segment, numerator, denominator, root);
        int Words = Math.Max(numerator.FieldWords, denominator.FieldWords);
        Span<ulong> values = stackalloc ulong[6 * Words]; Span<int> signs = stackalloc int[6];
        ContactQuadratic coordinate = At(values, signs, 0, Words), term = At(values, signs, 1, Words), divisor = At(values, signs, 2, Words);
        Scale(denominator, Signed576.ExtendValue(geometry.RawScale), divisor);
        Vector3d point = default;
        for (int axis = 0; axis < 3; axis++)
        {
            Scale(denominator, Component(geometry.A, axis), coordinate);
            Scale(numerator, Component(geometry.Edge, axis), term); coordinate.Add(term);
            point[axis] = RoundRatio(coordinate, divisor, root);
        }
        return point;
    }

    private static Vector3d RoundSegmentPoint(FixedSegment segment, ContactQuadratic numerator,
        ContactQuadratic denominator, ReadOnlySpan<ulong> root)
    {
        int Words = Math.Max(numerator.FieldWords, denominator.FieldWords);
        Span<ulong> values = stackalloc ulong[4 * Words];
        Span<int> signs = stackalloc int[4];
        ContactQuadratic point = At(values, signs, 0, Words), product = At(values, signs, 1, Words);
        Vector3d result = default;
        for (int axis = 0; axis < 3; axis++)
        {
            Scale(denominator, Signed832.ExtendValue(Signed576.ExtendValue(Signed192.Raw(segment.Start[axis]))), point);
            Scale(numerator, Signed832.ExtendValue(Signed576.ExtendValue(WideArithmetic.Difference(segment.End[axis], segment.Start[axis]))), product); point.Add(product);
            result[axis] = RoundRatio(point, denominator, root);
        }
        return result;
    }

    private static void ClipSideEndpoint(in SegmentConeSurfaceGeometry geometry, bool upper,
        Span<ulong> normals, Span<int> normalSigns, ReadOnlySpan<ulong> root)
    {
        int Words = geometry.Words;
        Span<ulong> values = stackalloc ulong[8 * Words];
        Span<int> signs = stackalloc int[8];
        ContactQuadratic bound = At(values, signs, 0, Words), denominator = At(values, signs, 1, Words), constant = At(values, signs, 2, Words), product = At(values, signs, 3, Words);
        ContactQuadratic metric = At(normals, normalSigns, 7, Words);
        Scale(metric, Signed832.ExtendValue(geometry.A.Y), constant);
        Multiply(At(normals, normalSigns, 8, Words), At(normals, normalSigns, 1, Words), root, product); constant.Add(product, -1);
        int direction = geometry.Edge.Y.Sign;
        Scale(metric, Signed832.ExtendValue(geometry.Cap), bound);
        bound.MultiplySign((upper ? 1 : -1) * direction); bound.Add(constant, -1); bound.MultiplySign(direction);
        Scale(metric, Signed832.ExtendValue(geometry.Edge.Y), denominator); denominator.MultiplySign(direction);
        ContactQuadratic current = At(normals, normalSigns, 5, Words), currentD = At(normals, normalSigns, 6, Words);
        int comparison = CompareRatios(bound, denominator, root, current, currentD, root);
        if (upper ? comparison < 0 : comparison > 0)
        {
            bound.CopyTo(current); denominator.CopyTo(currentD);
        }
    }
}
