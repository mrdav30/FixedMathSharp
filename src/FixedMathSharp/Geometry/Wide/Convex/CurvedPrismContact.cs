//=======================================================================
// MIT License, Copyright (c) 2024-present David Oravsky (mrdav30)
// See LICENSE file in the project root for full license information.
//=======================================================================
using System;
using static FixedMathSharp.Geometry.CylinderContactAlgebra;

namespace FixedMathSharp.Geometry;

/// <summary>Complete native-yaw prism support-fan selection for finite cones and cylinders.</summary>
/// <content>Selects exact analytic, generator and rim contacts against native convex prisms.</content>
internal static partial class CurvedPrismContact
{
    internal static bool TryGetContact(Vector3d center, FixedQuaternion rotation, Vector3d localAxis,
        Fixed64 length, Fixed64 radius, Vector3d prismOrigin, Fixed64 yaw,
        ReadOnlySpan<Vector2d> offsets, Fixed64 halfThickness, bool cone, out FixedContactAnchors contact)
    {
        contact = default;
        Span<WideDirection3d> vertices = stackalloc WideDirection3d[32];
        var geometry = new CurvedPrismContactGeometry(center, rotation, localAxis, length, radius,
            prismOrigin, yaw, offsets, halfThickness, cone && radius != Fixed64.Zero, vertices);
        Span<ulong> values = stackalloc ulong[ConvexContactCandidate.Slots * Words];
        Span<int> signs = stackalloc int[ConvexContactCandidate.Slots];
        var selection = new CurvedPrismContactSelection(values, signs);
        Span<ulong> direction = stackalloc ulong[3 * Words];
        Span<int> directionSigns = stackalloc int[3];
        KeepAxis(geometry, new WideDirection3d(geometry.Axis), direction, directionSigns, ref selection);
        bool minimumCertified = selection.GapSign >= 0 && HasPoleMinimumCertificate(geometry, selection);
        if (!minimumCertified) KeepAxis(geometry, geometry.Plane(1), direction, directionSigns, ref selection);
        if (selection.GapSign < 0) return false;
        // Zero meridional curvature excludes positive open vertex/rim minima,
        // but not negative separating ones. Keep complete closed admission
        // independent of the positive minimum-translation support inventory.
        if (!minimumCertified && !WideConvexPrismRelations.IntersectsCurvedPrism(cone, center, rotation, localAxis,
                length, radius, prismOrigin, yaw, offsets, halfThickness))
            return false;
        // Complete admission bounds every gap below by zero; an exact initial
        // zero is already the earliest global minimum, including its witnesses.
        minimumCertified |= selection.GapSign == 0;
        if (!minimumCertified) KeepAnalytic(geometry, ref selection);
        WideConvexPrismRelations.GetRoundedConvexContactCandidateDepth(selection.Candidate,
            Fixed64.Zero, out Fixed64 depth, out bool clamped);
        CurvedPrismRimContacts.Selection rim = default;
        // Complete admission bounds every support gap below by zero. An exact
        // analytic zero already meets that bound, so no rim can improve it.
        if (!minimumCertified && selection.GapSign != 0)
            CurvedPrismRimContacts.GetContact(geometry, selection.Candidate, depth, clamped,
                out rim, out depth, out clamped);
        Vector3d normal;
        FixedPointAnchor shapeAnchor, prismAnchor;
        if (rim.HasValue)
            CurvedPrismRimContacts.GetMaterials(geometry, rim, center, rotation, prismOrigin,
                out normal, out shapeAnchor, out prismAnchor);
        else
            GetAnalyticWitnesses(geometry, selection, center, rotation, prismOrigin,
                out normal, out shapeAnchor, out prismAnchor);
        contact = new FixedContactAnchors(shapeAnchor, prismAnchor, normal, depth, clamped);
        return true;
    }

    private static void KeepAnalytic(in CurvedPrismContactGeometry geometry,
        ref CurvedPrismContactSelection selection)
    {
        Span<ulong> direction = stackalloc ulong[3 * Words];
        Span<int> signs = stackalloc int[3];
        var axis = new WideDirection3d(geometry.Axis);
        WideDirection3d up = geometry.Plane(1);
        for (int index = 0; index < geometry.EdgeCount; index++)
        {
            geometry.Edge(index, out int start, out int end);
            WideDirection3d edge = Difference(geometry.Vertex(end), geometry.Vertex(start));
            if (edge.IsZero) continue;
            if (index % 3 == 0)
                KeepAxis(geometry, WideDirection3d.Cross(edge, up), direction, signs, ref selection);
            // These rational arc endpoints also cover K=0 stationary roots,
            // which the quartic owner deliberately leaves to the analytic fan.
            // Only supporting real edges bound those arcs; face directions
            // above and vertex extrema below remain unrestricted.
            CircularRimContactAlgebra.GetBasis(edge, out WideDirection3d first, out WideDirection3d second);
            KeepAxis(geometry, first, direction, signs, ref selection, start);
            KeepAxis(geometry, second, direction, signs, ref selection, start);
            ProjectPerpendicular(edge, axis, direction, signs);
            KeepDirection(geometry, direction, signs, ref selection, start);
            for (int cap = -1; cap <= 1; cap += 2)
            {
                if (geometry.IsCone && cap > 0) continue;
                WideDirection3d offset = Difference(cap < 0 ? -geometry.Half : geometry.Half, geometry.Vertex(start));
                ProjectPerpendicular(edge, offset, direction, signs);
                KeepDirection(geometry, direction, signs, ref selection, start);
            }
            KeepAxis(geometry, WideDirection3d.Cross(axis, edge), direction, signs, ref selection, start);
            if (selection.HasValue && selection.GapSign == 0) return;
        }
        if (geometry.IsCone)
            CurvedPrismGeneratorFeatures.KeepAll(geometry, ref selection);
        else
            for (int index = 0; index < geometry.VertexCount && (!selection.HasValue || selection.GapSign != 0); index++)
            {
                ProjectPerpendicular(axis, geometry.Vertex(index), direction, signs);
                KeepDirection(geometry, direction, signs, ref selection);
            }
    }

    private static bool HasPoleMinimumCertificate(in CurvedPrismContactGeometry geometry,
        scoped CurvedPrismContactSelection selection)
    {
        if (geometry.IsCone && selection.Feature > 0) return false;
        var axis = new WideDirection3d(geometry.Axis);
        WideDirection3d normal = selection.Feature < 0 ? -axis : axis;
        Signed320 minimum = Signed320.NarrowValue(Signed576.NarrowValue(geometry.ProjectionMinimum(normal)));
        if (minimum.Sign < 0) return false;
        Signed320 half = Signed320.NarrowValue(Signed576.NarrowValue(WideDirection3d.Dot(geometry.Half, axis)));
        Signed320 gap = WideArithmetic.SubtractSigned320(half, minimum);
        Signed576 gapSquared = WideArithmetic.MultiplySigned320(gap, gap);
        Signed576 radiusSquared = WideArithmetic.MultiplySigned320(geometry.Radius, geometry.Radius);
        Signed832 radial = WideArithmetic.MultiplySigned576ToSigned832(radiusSquared, geometry.SquaredAxis);
        bool insideBall = geometry.IsCone
            ? WideArithmetic.SubtractSigned832(WideArithmetic.MultiplySigned832(radial, minimum),
                WideArithmetic.MultiplySigned576ToSigned832(gapSquared, half)).Sign >= 0
            : WideArithmetic.SubtractSigned832(radial, Signed832.ExtendValue(gapSquared)).Sign >= 0;
        if (!insideBall) return false;
        // At a cap pole, C=(minimum/q)*normal is the disk center in the
        // opposing prism plane. If C belongs to the prism and ball(C,depth)
        // belongs to the primitive, their Minkowski difference contains that
        // ball. The pole's upper bound is therefore the global minimum.
        // Cylinder: depth<=half-height and depth<=R. Cone base: the exact
        // side-distance test reduces to R²*q*minimum >= half*gap².
        // All vectors here are the raw authored axis: scalars <202 bits,
        // plane dots <401 and complete products <610, within Signed832.
        for (int plane = 0; plane < geometry.PlaneCount; plane++)
        {
            WideDirection3d face = geometry.Plane(plane);
            if (face.IsZero) continue;
            for (int orientation = 0; orientation < 2; orientation++, face = -face)
            {
                Signed320 projection = Signed320.NarrowValue(Signed576.NarrowValue(WideDirection3d.Dot(face, normal)));
                Signed576 left = WideArithmetic.MultiplySigned320(minimum, projection);
                Signed832 right = WideArithmetic.MultiplySigned832(geometry.ProjectionMinimum(face), geometry.SquaredAxis);
                if (WideArithmetic.SubtractSigned832(Signed832.ExtendValue(left), right).Sign < 0) return false;
            }
        }
        return true;
    }

    private static void KeepAxis(in CurvedPrismContactGeometry geometry, in WideDirection3d axis,
        scoped Span<ulong> direction, scoped Span<int> signs, ref CurvedPrismContactSelection selection,
        int requiredVertex = -1)
    {
        if (axis.IsZero) return;
        WriteDirection(axis, direction, signs);
        KeepDirection(geometry, direction, signs, ref selection, requiredVertex);
    }

    private static void KeepDirection(in CurvedPrismContactGeometry geometry,
        scoped Span<ulong> direction, scoped Span<int> directionSigns, ref CurvedPrismContactSelection selection,
        int requiredVertex = -1)
    {
        if (selection.HasValue && selection.GapSign < 0 || IsZero(direction)) return;
        Span<ulong> values = stackalloc ulong[ConvexContactCandidate.Slots * Words];
        Span<int> signs = stackalloc int[ConvexContactCandidate.Slots];
        for (int orientation = 0; orientation < 2; orientation++)
        {
            if (geometry.TryBuildCandidate(direction, directionSigns, values, signs, requiredVertex,
                    out int gap, out int vertex, out int feature))
                selection.Keep(geometry, values, signs, gap, vertex, feature);
            for (int component = 0; component < 3; component++) directionSigns[component] = -directionSigns[component];
        }
    }

    internal static WideDirection3d Difference(in WideDirection3d first, in WideDirection3d second) => new(
        WideArithmetic.SubtractSigned576(first.X, second.X), WideArithmetic.SubtractSigned576(first.Y, second.Y),
        WideArithmetic.SubtractSigned576(first.Z, second.Z));
}
