//=======================================================================
// MIT License, Copyright (c) 2024-present David Oravsky (mrdav30)
// See LICENSE file in the project root for full license information.
//=======================================================================
using System;
using static FixedMathSharp.Geometry.CylinderContactAlgebra;

namespace FixedMathSharp.Geometry;

/// <summary>Face, cap-pole, vertex/side and edge/side boundaries of the support fan.</summary>
internal static class TriangleCylinderAnalyticFeatures
{
    private static WideDirection3d Up => new(default, Signed576.One, default);

    internal static bool TryGetSeed(in TriangleCircularGeometry geometry, in WideDirection3d coreOffset, in WideDirection3d coreAxis,
        Span<ulong> values, Span<int> signs, Span<ulong> direction, Span<int> directionSigns,
        out int gapSign, out int supportMask, out int coreSign, in CylinderHullSupport hull, bool includeSeam)
    {
        var selection = new Selection(values, signs, direction, directionSigns);
        if (includeSeam) KeepSeam(geometry, coreAxis, ref selection, hull);
        Span<ulong> normal = stackalloc ulong[3 * Words]; Span<int> normalSigns = stackalloc int[3];
        WideDirection3d face = geometry.FaceNormal;
        if (!face.IsZero && WideDirection3d.Dot(face, coreAxis).Sign == 0)
        {
            // Match the complete fan's compact seam normal, retaining its
            // witness widths even when the hull face carries unreduced scale.
            face = WideDirection3d.Cross(geometry.Edge(0), coreAxis);
            if (face.IsZero) face = WideDirection3d.Cross(geometry.Edge(1), coreAxis);
        }
        WriteDirection(face, normal, normalSigns);
        for (int region = -1; region <= 1; region++)
        {
            var endpoint = region == 0 ? geometry : geometry.AtCoreRegion(coreOffset, region);
            KeepAxis(endpoint, coreAxis, region, normal, normalSigns, ref selection, hull);
        }
        gapSign = selection.GapSign; supportMask = selection.SupportMask; coreSign = selection.CoreSign;
        if (!selection.HasValue) return false;
        WriteWorldDirection(geometry.WorldBasis, direction, directionSigns, values, signs);
        return gapSign >= 0;
    }

    internal static bool TryGetBest(in TriangleCircularGeometry geometry, in WideDirection3d coreOffset, in WideDirection3d coreAxis,
        Span<ulong> bestValues, Span<int> bestSigns, Span<ulong> bestDirection,
        Span<int> bestDirectionSigns, out int gapSign, out int supportMask, out int coreSign, out bool faceMinimumCertified,
        in CylinderHullSupport hull = default)
    {
        var selection = new Selection(bestValues, bestSigns, bestDirection, bestDirectionSigns);
        faceMinimumCertified = false;
        if (coreAxis.IsZero)
            KeepRegion(geometry, coreAxis, 0, ref selection, out faceMinimumCertified, hull);
        else
        {
            KeepSeam(geometry, coreAxis, ref selection, hull);
            // The centered cylinder is contained in the stadium. Its existing
            // inscribed ball therefore remains a lower bound. An exactly
            // core-perpendicular face has the same support in both shapes;
            // Up or Up x core already attained it in the explicit seam fan.
            faceMinimumCertified = hull.IsEmpty && selection.GapSign >= 0
                && WideDirection3d.Dot(geometry.FaceNormal, coreAxis).Sign == 0
                && geometry.HasFaceMinimumCertificate();
            for (int region = -1; region <= 1 && selection.GapSign >= 0 && !faceMinimumCertified; region += 2)
            {
                TriangleCircularGeometry endpoint = geometry.AtCoreRegion(coreOffset, region);
                KeepRegion(endpoint, coreAxis, region, ref selection, out _, hull);
            }
            if (hull.IsEmpty && !faceMinimumCertified && selection.GapSign >= 0)
                faceMinimumCertified = geometry.HasFaceDiskMinimumCertificate(coreAxis,
                    new ConvexContactCandidate(selection.Values, selection.Signs, selection.GapSign));
        }
        // Selection and its face certificates read gap fields only. Transform
        // the retained local direction once, after the exact winner is known.
        WriteWorldDirection(geometry.WorldBasis, selection.Direction, selection.DirectionSigns, selection.Values, selection.Signs);
        gapSign = selection.GapSign; supportMask = selection.SupportMask; coreSign = selection.CoreSign;
        return gapSign >= 0;
    }

    private static void KeepSeam(in TriangleCircularGeometry geometry, in WideDirection3d coreAxis, ref Selection selection, in CylinderHullSupport hull)
    {
        Span<ulong> direction = stackalloc ulong[3 * Words];
        Span<int> signs = stackalloc int[3];
        WriteDirection(Up, direction, signs);
        KeepAxis(geometry, coreAxis, 0, direction, signs, ref selection, hull);
        WriteDirection(new WideDirection3d(WideArithmetic.Negate(coreAxis.Z), default, coreAxis.X), direction, signs);
        KeepAxis(geometry, coreAxis, 0, direction, signs, ref selection, hull);
        for (int edge = 0; edge < 3; edge++)
        {
            WriteDirection(WideDirection3d.Cross(geometry.Edge(edge), coreAxis), direction, signs);
            KeepAxis(geometry, coreAxis, 0, direction, signs, ref selection, hull);
        }
    }

    private static void KeepRegion(in TriangleCircularGeometry geometry, in WideDirection3d coreAxis, int region,
        ref Selection selection, out bool faceMinimumCertified, in CylinderHullSupport hull)
    {
        Span<ulong> direction = stackalloc ulong[3 * Words];
        Span<int> signs = stackalloc int[3];
        WriteDirection(Up, direction, signs);
        KeepAxis(geometry, coreAxis, region, direction, signs, ref selection, hull);
        // Horizontal face directions duplicate the already tested +/-Up,
        // including their support masks and first-winner tie ordering.
        if (!geometry.FaceNormal.X.IsZero || !geometry.FaceNormal.Z.IsZero)
        {
            WriteDirection(geometry.FaceNormal, direction, signs);
            KeepAxis(geometry, coreAxis, region, direction, signs, ref selection, hull);
        }
        faceMinimumCertified = hull.IsEmpty && coreAxis.IsZero && selection.GapSign >= 0 && geometry.HasFaceMinimumCertificate();
        for (int vertex = 0; vertex < 3 && selection.GapSign >= 0 && !faceMinimumCertified; vertex++)
        {
            WideDirection3d point = geometry.Vertex(vertex);
            WriteDirection(new WideDirection3d(point.X, default, point.Z), direction, signs);
            KeepAxis(geometry, coreAxis, region, direction, signs, ref selection, hull);
            WideDirection3d edge = geometry.Edge(vertex);
            bool oblique = !edge.Y.IsZero && (!edge.X.IsZero || !edge.Z.IsZero);
            if (oblique)
            {
                CircularRimContactAlgebra.GetBasis(edge, out WideDirection3d first, out WideDirection3d second);
                WriteDirection(first, direction, signs);
                KeepAxis(geometry, coreAxis, region, direction, signs, ref selection, hull);
                WriteDirection(second, direction, signs);
                KeepAxis(geometry, coreAxis, region, direction, signs, ref selection, hull);
            }
            WriteDirection(new WideDirection3d(WideArithmetic.Negate(edge.Z), default, edge.X), direction, signs);
            KeepAxis(geometry, coreAxis, region, direction, signs, ref selection, hull);
            // Principal axes and K=0 directions are rational and must be
            // included separately from the stationary quartic's division by K.
            // Parallel/perpendicular edges project a circle/rectangle. Their
            // positive minima are the vertex-radial, pole, side or face
            // boundaries above; an admitted negative interior also has a
            // negative admitted boundary. Only oblique ellipses need extras.
            if (oblique)
            {
                ProjectPerpendicular(edge, Up, direction, signs);
                KeepAxis(geometry, coreAxis, region, direction, signs, ref selection, hull);
            }
            for (int cap = -1; cap <= 1; cap += 2)
            {
                if (oblique)
                {
                    ProjectPerpendicular(edge, geometry.CapOffset(vertex, cap), direction, signs);
                    KeepAxis(geometry, coreAxis, region, direction, signs, ref selection, hull);
                }
                if (HasVertexRimSeparation(geometry, vertex, cap, coreAxis, region, hull))
                {
                    selection.GapSign = -1;
                    return;
                }
            }
        }
    }

    private static void KeepAxis(in TriangleCircularGeometry geometry, in WideDirection3d coreAxis, int region, scoped Span<ulong> direction,
        scoped Span<int> directionSigns, ref Selection selection, in CylinderHullSupport hull)
    {
        if (selection.HasValue && selection.GapSign < 0)
            return;
        if (directionSigns[0] == 0 && directionSigns[1] == 0 && directionSigns[2] == 0)
            return;
        Span<ulong> values = stackalloc ulong[ConvexContactCandidate.Slots * Words];
        Span<int> signs = stackalloc int[ConvexContactCandidate.Slots];
        Span<ulong> projection = stackalloc ulong[Words];
        int alignment = 0;
        if (!coreAxis.IsZero)
            Dot(coreAxis, direction, directionSigns, projection, out alignment);
        bool invariantsReady = false;
        for (int orientation = 0; orientation < 2; orientation++)
        {
            if (coreAxis.IsZero || (region == 0 ? alignment == 0 : alignment * region > 0))
            {
                if (!hull.IsEmpty)
                {
                    int mask = geometry.GetSupportMask(direction, directionSigns);
                    int vertex = (mask & 1) != 0 ? 0 : (mask & 2) != 0 ? 1 : 2;
                    if (!hull.Supports(geometry, geometry.Vertex(vertex), direction, directionSigns)) goto Reverse;
                }
                int gapSign = BuildAxis(geometry, direction, directionSigns, values, signs, invariantsReady, out int supportMask);
                invariantsReady = true;
                var candidate = new ConvexContactCandidate(values, signs, gapSign);
                if (!selection.HasValue || WideConvexPrismRelations.CompareConvexContactCandidates(candidate,
                        new ConvexContactCandidate(selection.Values, selection.Signs, selection.GapSign)) < 0)
                {
                    values.CopyTo(selection.Values); signs.CopyTo(selection.Signs);
                    direction.CopyTo(selection.Direction); directionSigns.CopyTo(selection.DirectionSigns);
                    selection.HasValue = true; selection.GapSign = gapSign; selection.SupportMask = supportMask;
                    selection.CoreSign = region;
                }
            }
        Reverse:
            for (int component = 0; component < 3; component++)
                directionSigns[component] = -directionSigns[component];
            alignment = -alignment;
        }
    }

    private static int BuildAxis(in TriangleCircularGeometry geometry,
        ReadOnlySpan<ulong> direction, ReadOnlySpan<int> directionSigns,
        Span<ulong> values, Span<int> signs, bool invariantsReady, out int mask)
    {
        values[..(9 * Words)].Clear(); signs.Clear();
        Span<ulong> work = stackalloc ulong[8 * Words];
        Span<ulong> rational = Slot(work, 0), current = Slot(work, 1), difference = Slot(work, 2);
        Span<ulong> temporary = Slot(work, 3), radial = Slot(work, 4), scale = Slot(work, 5);
        Span<ulong> metric = Slot(work, 6), denominator = Slot(work, 7);
        Dot(geometry.A, direction, directionSigns, rational, out int rationalSign);
        mask = 1;
        for (int vertex = 1; vertex < 3; vertex++)
        {
            Dot(geometry.Vertex(vertex), direction, directionSigns, current, out int currentSign);
            current.CopyTo(difference);
            int comparison = currentSign;
            Add(rational, -rationalSign, difference, ref comparison);
            if (comparison > 0)
            {
                current.CopyTo(rational); rationalSign = currentSign; mask = 1 << vertex;
            }
            else if (comparison == 0)
                mask |= 1 << vertex;
        }
        Import(geometry.HalfHeight, current);
        WideArithmetic.MultiplyMagnitudes(current, Slot(direction, 1), temporary);
        Add(temporary, 1, rational, ref rationalSign);
        // +/-N have identical R²|N_h|² and RawScale²|N|². Borrow the
        // working candidate's disjoint slots only after an admitted build;
        // a core region may admit the second orientation alone.
        if (invariantsReady)
            return BuildRadialCandidate(rational, rationalSign, Slot(values, 9), Slot(values, 10), values, signs);
        WideArithmetic.MultiplyMagnitudes(Slot(direction, 0), Slot(direction, 0), radial);
        WideArithmetic.MultiplyMagnitudes(Slot(direction, 2), Slot(direction, 2), temporary);
        WideArithmetic.AddMagnitudeInto(temporary, radial);
        Import(geometry.Radius, current);
        WideArithmetic.MultiplyMagnitudes(current, current, temporary);
        WideArithmetic.MultiplyMagnitudes(radial, temporary, current);
        SumSquares(direction, metric);
        Import(Signed576.ExtendValue(geometry.RawScale), scale);
        WideArithmetic.MultiplyMagnitudes(scale, scale, temporary);
        WideArithmetic.MultiplyMagnitudes(metric, temporary, denominator);
        return BuildRadialCandidate(rational, rationalSign, current, denominator, values, signs);
    }

    private static bool HasVertexRimSeparation(in TriangleCircularGeometry geometry, int vertex, int cap,
        in WideDirection3d coreAxis, int region, in CylinderHullSupport hull)
    {
        WideDirection3d c = geometry.CapOffset(vertex, cap);
        if (c.Y.Sign * cap > 0)
            return false;
        Signed832 radialSquared = WideArithmetic.AddSigned832(
            WideArithmetic.MultiplySigned576(c.X, c.X), WideArithmetic.MultiplySigned576(c.Z, c.Z));
        if (WideArithmetic.SubtractSigned832(radialSquared,
                WideArithmetic.MultiplySigned576(geometry.Radius, geometry.Radius)).Sign <= 0)
            return false;
        Span<ulong> work = stackalloc ulong[4 * Words];
        Span<ulong> rational = Slot(work, 0), radical = Slot(work, 1), root = Slot(work, 2), radius = Slot(work, 3);
        Import(radialSquared, root); Import(geometry.Radius, radius);
        // The endpoint's closest-rim residual is a separator of the whole
        // stadium only in this endpoint's open core-normal hemisphere.
        if (!coreAxis.IsZero && ResidualProjectionSign(c, coreAxis, rational, radical, root, radius) * region <= 0)
            return false;
        for (int other = 0; other < 3; other++)
        {
            if (other == vertex)
                continue;
            if (ResidualProjectionSign(c, geometry.EdgeFromTo(other, vertex), rational, radical, root, radius) < 0)
                return false;
        }
        for (int other = 0; other < hull.Count; other++)
            if (ResidualProjectionSign(c, TriangleCircularGeometry.Subtract(geometry.Vertex(vertex), geometry.TransformHullVertex(hull.Vertex(other))),
                    rational, radical, root, radius) < 0)
                return false;
        return true;
    }

    private static int ResidualProjectionSign(in WideDirection3d c, in WideDirection3d axis, Span<ulong> rational,
        Span<ulong> radical, ReadOnlySpan<ulong> root, ReadOnlySpan<ulong> radius)
    {
        Signed832 planar = WideArithmetic.AddSigned832(
            WideArithmetic.MultiplySigned576(c.X, axis.X), WideArithmetic.MultiplySigned576(c.Z, axis.Z));
        Import(planar, radical);
        WideArithmetic.MultiplyMagnitudes(radical, radius, rational);
        Signed832 full = WideDirection3d.Dot(c, axis);
        Import(full, radical);
        return WideConvexPrismRelations.GetConvexContactCandidateQuadraticSign(
            rational, planar.Sign, radical, -full.Sign, root);
    }

    private ref struct Selection
    {
        internal Span<ulong> Values, Direction;
        internal Span<int> Signs, DirectionSigns;
        internal int GapSign, SupportMask, CoreSign;
        internal bool HasValue;
        internal Selection(Span<ulong> values, Span<int> signs, Span<ulong> direction, Span<int> directionSigns)
        {
            Values = values; Signs = signs; Direction = direction; DirectionSigns = directionSigns;
            GapSign = SupportMask = CoreSign = 0; HasValue = false;
        }
    }
}
