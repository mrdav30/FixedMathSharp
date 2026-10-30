//=======================================================================
// MIT License, Copyright (c) 2024-present David Oravsky (mrdav30)
// See LICENSE file in the project root for full license information.
//=======================================================================
using System;
using static FixedMathSharp.Geometry.CylinderContactAlgebra;

namespace FixedMathSharp.Geometry;

/// <summary>Whole-volume contact between a validated closed convex hull and a finite cylinder.</summary>
internal static class ConvexHullCylinderContact
{
    /// <summary>
    /// Gets whole-volume contact when the cylinder center is inside or on the
    /// validated closed hull. False declines center admission, not exterior overlap.
    /// </summary>
    internal static bool TryGetContactWithContainedCenter(in ScaledRigidFrame3d source, ReadOnlySpan<Vector3d> points,
        ReadOnlySpan<int> triangles, Vector3d center, FixedQuaternion rotation, Fixed64 height, Fixed64 radius,
        out FixedContactAnchors contact)
    {
        contact = default;
        if (!WideOrientedBox.ContainsConvexHullPoint(source, points, triangles, center)) return false;
        bool found = TryGetContact(source, points, triangles, center, rotation, height, radius, out contact);
        System.Diagnostics.Debug.Assert(found, "An admitted center must have whole-volume contact.");
        return found;
    }

    internal static bool TryGetContact(in ScaledRigidFrame3d source, ReadOnlySpan<Vector3d> points,
        ReadOnlySpan<int> triangles, Vector3d center, FixedQuaternion rotation, Fixed64 height, Fixed64 radius,
        out FixedContactAnchors contact)
    {
        contact = default;
        var hull = new CylinderHullSupport(source, points, center, rotation, height, radius);
        Span<ulong> bestValues = stackalloc ulong[ConvexContactCandidate.Slots * Words], values = stackalloc ulong[ConvexContactCandidate.Slots * Words];
        Span<int> bestSigns = stackalloc int[ConvexContactCandidate.Slots], signs = stackalloc int[ConvexContactCandidate.Slots];
        Span<ulong> bestDirection = stackalloc ulong[3 * Words], direction = stackalloc ulong[3 * Words];
        Span<int> bestDirectionSigns = stackalloc int[3], directionSigns = stackalloc int[3];
        int bestTriangle = -1, bestMask = 0, gap = 0;
        bool certified = false;
        for (int index = 0; index < triangles.Length; index += 3)
        {
            FixedTriangle triangle = GetTriangle(points, triangles, index);
            var geometry = GetGeometry(triangle, source, hull, triangles, index);
            if (!TriangleCylinderAnalyticFeatures.TryGetBest(geometry, default, default, values, signs,
                    direction, directionSigns, out int candidateGap, out int mask, out _, out _, hull)) return false;
            if (bestTriangle >= 0 && WideConvexPrismRelations.CompareConvexContactCandidates(
                    new ConvexContactCandidate(values, signs, candidateGap), new ConvexContactCandidate(bestValues, bestSigns, gap)) >= 0) continue;
            values.CopyTo(bestValues); signs.CopyTo(bestSigns);
            direction.CopyTo(bestDirection); directionSigns.CopyTo(bestDirectionSigns);
            bestTriangle = index; bestMask = mask; gap = candidateGap;
            if (hull.HasBallMinimumCertificate(source, triangles, new ConvexContactCandidate(bestValues, bestSigns, gap)))
            { certified = true; break; }
        }
        if (bestTriangle < 0) return false;
        var bestGeometry = GetGeometry(GetTriangle(points, triangles, bestTriangle), source, hull, triangles, bestTriangle);
        var analytic = new ConvexContactCandidate(bestValues, bestSigns, gap);
        bestGeometry.GetAnalyticDepth(analytic, bestDirectionSigns, bestMask, out Fixed64 depth, out bool clamped);

        int valueWords = bestGeometry.ValueWords;
        Span<ulong> rootValues = stackalloc ulong[5 * valueWords]; Span<sbyte> rootSigns = stackalloc sbyte[5];
        Span<ulong> rootCell = stackalloc ulong[(16 * (valueWords * 64 + 11) + 255) / 64];
        TriangleCylinderEdgeContacts.Selection rootSelection = default;
        if (!certified && radius != Fixed64.Zero)
            for (int index = 0; index < triangles.Length; index += 3)
            {
                var geometry = GetGeometry(GetTriangle(points, triangles, index), source, hull, triangles, index);
                if (!TriangleCylinderEdgeContacts.TryKeepContacts(geometry, default, default, analytic, depth, clamped,
                        rootValues, rootSigns, rootCell, ref rootSelection, hull, index)) return false;
            }
        Vector3d origin = source.IsRigid ? source.RigidOrigin : center;
        FixedQuaternion anchorRotation = source.IsRigid ? source.Rotation : rotation;
        if (rootSelection.HasValue)
        {
            FixedTriangle triangle = GetTriangle(points, triangles, rootSelection.Triangle);
            var geometry = GetGeometry(triangle, source, hull, triangles, rootSelection.Triangle);
            TriangleCylinderEdgeContacts.GetMaterials(geometry, default, default, default, triangle, rootSelection, radius,
                out Vector3d normal, out Vector3d radial, out Vector3d point, out _);
            if (rootSelection.IsZero) { depth = Fixed64.Zero; clamped = false; }
            else
            {
                FiniteAxisValueRoot root = TriangleCylinderEdgeContacts.Restore(rootValues, rootSigns, rootCell, rootSelection);
                ConvexContactValueRoot.GetRoundedDepth(hull.RawScale, hull.ValueShift, ref root, out depth, out clamped);
            }
            contact = new FixedContactAnchors(new FixedPointAnchor(origin, anchorRotation, point),
                TriangleCircularGeometry.GetSupport(center, rotation, Signed192.Raw(height), radial, rootSelection.Cap), normal, depth, clamped);
            return true;
        }
        Vector3d analyticNormal = WideConvexPrismRelations.GetConvexContactCandidateNormal(analytic);
        bestDirection.CopyTo(values); bestDirectionSigns.CopyTo(signs);
        Slot(values, 1).Clear(); signs[1] = 0;
        bool pole = bestDirectionSigns[0] == 0 && bestDirectionSigns[2] == 0;
        Vector3d analyticRadial = pole ? default
            : -WideConvexPrismRelations.GetConvexContactCandidateScaledNormal(new ConvexContactCandidate(values, signs, gap), radius);
        // A triangulated support face may carry the matching point in a different
        // triangle from the first candidate. Certify the finite witness before rounding.
        for (int dimension = 3; dimension >= 1; dimension--)
            for (int index = 0; index < triangles.Length; index += 3)
            {
                FixedTriangle triangle = GetTriangle(points, triangles, index);
                var geometry = GetGeometry(triangle, source, hull, triangles, index);
                int mask = GetSupportMask(geometry, bestDirection, bestDirectionSigns);
                int count = (mask & 1) + ((mask >> 1) & 1) + ((mask >> 2) & 1);
                if (count != dimension) continue;
                int vertex = (mask & 1) != 0 ? 0 : (mask & 2) != 0 ? 1 : 2;
                if (!hull.Supports(geometry.Vertex(vertex), bestDirection, bestDirectionSigns)) continue;
                FixedPointAnchor cylinderAnchor;
                Vector3d point;
                int cap = bestDirectionSigns[1];
                if (cap == 0)
                {
                    if (!TriangleCylinderWitnesses.TryGetSidePoint(geometry, triangle, mask, bestDirection, bestDirectionSigns,
                            out point, out Fixed64 axial, out int boundaryCap)) continue;
                    cylinderAnchor = boundaryCap != 0
                        ? TriangleCircularGeometry.GetSupport(center, rotation, Signed192.Raw(height), analyticRadial, boundaryCap)
                        : new FixedPointAnchor(center, rotation, new Vector3d(Fixed64.Zero, axial, Fixed64.Zero), analyticRadial);
                }
                else
                {
                    Vector3d radial = analyticRadial;
                    if (pole)
                    {
                        if (!TriangleCylinderWitnesses.TryGetCapPoint(geometry, triangle, mask, out point, out radial)) continue;
                    }
                    else if (!TriangleCylinderRimWitnesses.TryGetAnalyticPoint(geometry, triangle, mask,
                                 bestDirection, bestDirectionSigns, cap, hull.Radius, out point)) continue;
                    cylinderAnchor = TriangleCircularGeometry.GetSupport(center, rotation, Signed192.Raw(height), radial, cap);
                }
                contact = new FixedContactAnchors(new FixedPointAnchor(origin, anchorRotation, point), cylinderAnchor,
                    analyticNormal, depth, clamped);
                return true;
            }
        System.Diagnostics.Debug.Fail("A whole-hull minimum must have a finite matching witness.");
        return false;
    }

    private static FixedTriangle GetTriangle(ReadOnlySpan<Vector3d> points, ReadOnlySpan<int> triangles, int index) =>
        new(points[triangles[index]], points[triangles[index + 1]], points[triangles[index + 2]]);

    private static TriangleCircularGeometry GetGeometry(FixedTriangle triangle, in ScaledRigidFrame3d source,
        in CylinderHullSupport hull, ReadOnlySpan<int> triangles, int index) =>
        new(triangle, source, hull, triangles[index], triangles[index + 1], triangles[index + 2]);

    private static int GetSupportMask(in TriangleCircularGeometry geometry, ReadOnlySpan<ulong> direction, ReadOnlySpan<int> signs)
    {
        Span<ulong> projection = stackalloc ulong[Words];
        int best = 0, mask = 1;
        for (int i = 1; i < 3; i++)
        {
            Dot(TriangleCircularGeometry.Subtract(geometry.Vertex(i), geometry.Vertex(best)), direction, signs, projection, out int sign);
            if (sign > 0) { best = i; mask = 1 << i; }
            else if (sign == 0) mask |= 1 << i;
        }
        return mask;
    }
}
