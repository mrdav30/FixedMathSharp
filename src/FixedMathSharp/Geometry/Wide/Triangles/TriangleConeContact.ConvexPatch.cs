//=======================================================================
// TriangleConeContact.ConvexPatch.cs
//=======================================================================
// MIT License, Copyright (c) 2024-present David Oravsky (mrdav30)
// See LICENSE file in the project root for full license information.
//=======================================================================
using System;
using static FixedMathSharp.Geometry.CylinderContactAlgebra;
using static FixedMathSharp.Geometry.CircularRimContactAlgebra;

namespace FixedMathSharp.Geometry;

/// <content>Complete trusted convex-polygon contacts reuse the finite-cone normal fan.</content>
internal static partial class TriangleConeContact
{
    /// <summary>
    /// Solves one filled, coplanar, strictly convex polygon. The ordered corner
    /// ring excludes repeated and collinear vertices. The seed belongs to the
    /// polygon and intersects the cone whenever the polygon has contact;
    /// optional bounds enclose the polygon.
    /// These topology and seed invariants are established by the borrower.
    /// </summary>
    internal static bool TryGetConvexPatchContact(FixedTriangle seed, Vector3d origin, FixedQuaternion rotation,
        ReadOnlySpan<Vector3d> vertices, ReadOnlySpan<int> corners, Vector3d center,
        FixedQuaternion coneRotation, Fixed64 height, Fixed64 radius, out FixedContactAnchors contact,
        ReadOnlySpan<Vector3d> witnessBounds = default)
    {
        System.Diagnostics.Debug.Assert(corners.Length >= 3 && height > Fixed64.Zero && radius > Fixed64.Zero);
        contact = default;
        var frame = new CylinderPolytopeFrame(center, coneRotation, Signed192.Raw(height), radius, origin, rotation);
        var basis = new WideRationalBasis3d(coneRotation);
        int valueShift = TriangleCircularGeometry.GetPolygonValueShift(frame, vertices, corners);
        Span<ulong> values = stackalloc ulong[ConvexContactCandidate.Slots * Words];
        Span<int> signs = stackalloc int[ConvexContactCandidate.Slots];
        var selection = new TriangleConeContactSelection(values, signs);
        int winningCorner = 0;
        Span<ulong> chartValues = stackalloc ulong[ConvexContactCandidate.Slots * Words];
        Span<int> chartSigns = stackalloc int[ConvexContactCandidate.Slots];
        for (int index = 0; index < corners.Length; index++)
        {
            FixedTriangle triangle = GetCornerTriangle(vertices, corners, index);
            var geometry = new TriangleCircularGeometry(triangle, frame, basis, valueShift);
            // For a convex corner, support at A (or either incident edge or
            // the face) is support of the whole polygon. B/C/BC are artificial.
            var chart = new TriangleConeContactSelection(chartValues, chartSigns, requiredMask: 1);
            KeepAnalytic(geometry, height, radius, ref chart);
            if (chart.Separated)
                return false;
            if (selection.HasValue && WideConvexPrismRelations.CompareConvexContactCandidates(
                    chart.Candidate, selection.Candidate) >= 0)
                continue;
            chart.Values.CopyTo(selection.Values); chart.Signs.CopyTo(selection.Signs);
            selection.GapSign = chart.GapSign; selection.Mask = chart.Mask;
            selection.Feature = chart.Feature; selection.HasValue = true;
            winningCorner = index;
        }
        FixedTriangle winner = GetCornerTriangle(vertices, corners, winningCorner);
        var winnerGeometry = new TriangleCircularGeometry(winner, frame, basis, valueShift);
        Fixed64 depth;
        bool clamped;
        if (selection.Feature == TriangleConeContactSelection.Generator)
            WideConvexPrismRelations.GetRoundedConvexContactCandidateDepth(selection.Candidate,
                Fixed64.Zero, out depth, out clamped);
        else
            winnerGeometry.GetAnalyticDepth(selection.Candidate, selection.Signs, selection.Mask, out depth, out clamped);
        System.Diagnostics.Debug.Assert(!clamped);

        // One common RawScale/ValueShift makes every mapped squared-gap root
        // directly comparable. Preserve root coefficients/cell until all true
        // edges have contributed; rounded depths cannot select the winner.
        // Requiring AB enumerates each exposed edge once, rather than also
        // repeating it as AC at the next corner.
        Span<ulong> rootValues = stackalloc ulong[5 * ValueWords];
        Span<sbyte> rootSigns = stackalloc sbyte[5];
        Span<ulong> rootCell = stackalloc ulong[TriangleConeRimContacts.ValueCellWords];
        TriangleConeRimContacts.Selection rim = default;
        for (int index = 0; index < corners.Length; index++)
        {
            FixedTriangle triangle = GetCornerTriangle(vertices, corners, index);
            var geometry = new TriangleCircularGeometry(triangle, frame, basis, valueShift);
            if (!TriangleConeRimContacts.TryKeepContacts(geometry, height, radius, selection.Candidate, depth,
                    rootValues, rootSigns, rootCell, ref rim, requiredMask: 3, chartIndex: index))
                return false;
        }
        Vector3d normal, point;
        FixedPointAnchor coneAnchor;
        if (rim.HasValue)
        {
            winner = GetCornerTriangle(vertices, corners, rim.Chart);
            winnerGeometry = new TriangleCircularGeometry(winner, frame, basis, valueShift);
            TriangleConeRimContacts.GetContactMaterials(winnerGeometry, winner, height, radius,
                rootValues, rootSigns, rootCell, rim, out normal, out Vector3d radialPoint,
                out point, out depth, out clamped);
            coneAnchor = TriangleCircularGeometry.GetSupport(center, coneRotation, Signed192.Raw(height), radialPoint, 1);
        }
        else if (selection.Mask == 7 && selection.Feature == TriangleConeContactSelection.Generator)
        {
            // Corner ears need not cover the polygon's center. A complete fan
            // admits the exact supporting generator slice before rounding its
            // matched witnesses; an equally supporting Apex may lie outside.
            GetPolygonGeneratorWitness(vertices, corners, frame, basis, valueShift, height, radius,
                center, coneRotation, ref selection, out normal, out point, out coneAnchor);
        }
        else
        {
            if (selection.Mask == 7 && selection.Feature == TriangleConeContactSelection.BasePole)
            {
                winner = seed;
                winnerGeometry = new TriangleCircularGeometry(seed, frame, basis, valueShift);
            }
            GetAnalyticWitnesses(winnerGeometry, winner, height, radius, center, coneRotation,
                ref selection, out normal, out point, out coneAnchor,
                witnessBounds.IsEmpty ? vertices : witnessBounds);
        }
        contact = new FixedContactAnchors(new FixedPointAnchor(origin, rotation, point), coneAnchor,
            normal, depth, clamped);
        return true;
    }

    private static FixedTriangle GetCornerTriangle(ReadOnlySpan<Vector3d> vertices,
        ReadOnlySpan<int> corners, int index) => new(vertices[corners[index]],
            vertices[corners[(index + 1) % corners.Length]],
            vertices[corners[(index + corners.Length - 1) % corners.Length]]);

    private static void GetPolygonGeneratorWitness(ReadOnlySpan<Vector3d> vertices, ReadOnlySpan<int> corners,
        in CylinderPolytopeFrame frame, in WideRationalBasis3d basis, int valueShift,
        Fixed64 height, Fixed64 radius, Vector3d center, FixedQuaternion coneRotation,
        ref TriangleConeContactSelection selection, out Vector3d normal, out Vector3d point,
        out FixedPointAnchor coneAnchor)
    {
        Span<ulong> localNormal = stackalloc ulong[7 * Words];
        Span<int> localSigns = stackalloc int[7];
        selection.Values[..(7 * Words)].CopyTo(localNormal); selection.Signs[..7].CopyTo(localSigns);
        for (int index = 1; index + 2 < corners.Length; index++)
        {
            var triangle = new FixedTriangle(vertices[corners[0]], vertices[corners[index]], vertices[corners[index + 1]]);
            var geometry = new TriangleCircularGeometry(triangle, frame, basis, valueShift);
            if (!TriangleConeWitnesses.TryGetGenerator(geometry, triangle, height, radius, 7,
                    localNormal, localSigns, center, coneRotation, out point, out coneAnchor))
                continue;
            normal = TransformAnalyticNormal(geometry, localNormal, localSigns, ref selection);
            return;
        }
        // A global face minimum has a matched polygon/cone support pair.
        // If earlier fan triangles declined it, the final triangle must admit
        // that slice. The ordinary generator owner verifies this invariant.
        var finalTriangle = new FixedTriangle(vertices[corners[0]],
            vertices[corners[^2]], vertices[corners[^1]]);
        var finalGeometry = new TriangleCircularGeometry(finalTriangle, frame, basis, valueShift);
        TriangleConeWitnesses.GetGenerator(finalGeometry, finalTriangle, height, radius, 7,
            localNormal, localSigns, center, coneRotation, out point, out coneAnchor);
        normal = TransformAnalyticNormal(finalGeometry, localNormal, localSigns, ref selection);
    }
}
