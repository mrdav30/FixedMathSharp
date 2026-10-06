//=======================================================================
// MIT License, Copyright (c) 2024-present David Oravsky (mrdav30)
// See LICENSE file in the project root for full license information.
//=======================================================================
using System;
using static FixedMathSharp.Geometry.CylinderContactAlgebra;

namespace FixedMathSharp.Geometry;

/// <summary>Complete finite-cone/triangle support-fan contact ownership.</summary>
internal static class TriangleConeContact
{
    internal static bool TryGetContact(FixedTriangle triangle, Vector3d origin, FixedQuaternion rotation,
        Vector3d center, FixedQuaternion coneRotation, Fixed64 height, Fixed64 radius,
        out FixedContactAnchors contact)
    {
        contact = default;
        if (triangle.IsDegenerate)
            return false;
        if (radius == Fixed64.Zero)
            return TriangleCylinderContact.TryGetContact(triangle, origin, rotation, center, coneRotation,
                Signed192.Raw(height), radius, out contact, out _);
        var geometry = new TriangleCircularGeometry(triangle, origin, rotation, center, coneRotation,
            Signed192.Raw(height), radius);
        Span<ulong> bestValues = stackalloc ulong[ConvexContactCandidate.Slots * Words];
        Span<int> bestSigns = stackalloc int[ConvexContactCandidate.Slots];
        Span<ulong> localNormal = stackalloc ulong[7 * Words];
        Span<int> localSigns = stackalloc int[7];
        var selection = new TriangleConeContactSelection(geometry.WorldBasis, bestValues, bestSigns, localNormal, localSigns);
        bool faceMinimumCertified = KeepAnalytic(geometry, height, radius, ref selection);
        if (selection.Separated)
            return false;
        var best = selection.Candidate;
        bool rootWinner = false;
        Vector3d normal = default, radialPoint = default, point = default;
        // Generator normals can mix rational and radical components;
        // only rational local normals use the principal-axis cancellation.
        Fixed64 depth;
        bool clamped;
        if (selection.Feature == TriangleConeContactSelection.Generator)
            WideConvexPrismRelations.GetRoundedConvexContactCandidateDepth(best, Fixed64.Zero, out depth, out clamped);
        else
            geometry.GetAnalyticDepth(best, localSigns, selection.Mask, out depth, out clamped);
        // Both exact face orientations bound this winner by half the cone's
        // width. Diameter=max(2R,sqrt(R^2+H^2))<=2*MaxValue, so it cannot clamp.
        // Exact orthogonal frames and the generator seam preserve this bound.
        System.Diagnostics.Debug.Assert(!clamped);
        if (!faceMinimumCertified && !TriangleConeRimContacts.TryGetContact(geometry, triangle, height, radius, best, depth,
                out rootWinner, out normal, out radialPoint, out point, out depth, out clamped))
            return false;
        FixedPointAnchor coneAnchor;
        if (rootWinner)
            coneAnchor = TriangleCircularGeometry.GetSupport(center, coneRotation, Signed192.Raw(height), radialPoint, 1);
        else
        {
            normal = WideConvexPrismRelations.GetConvexContactCandidateNormal(best);
            if (selection.Feature == TriangleConeContactSelection.Generator)
                TriangleConeWitnesses.GetGenerator(geometry, triangle, height, radius, selection.Mask,
                    localNormal, localSigns, center, coneRotation, out point, out coneAnchor);
            else if (selection.Feature == TriangleConeContactSelection.BasePole)
            {
                point = TriangleCylinderWitnesses.GetCapPoint(geometry, triangle, selection.Mask, out radialPoint);
                coneAnchor = TriangleCircularGeometry.GetSupport(center, coneRotation, Signed192.Raw(height), radialPoint, 1);
            }
            else
            {
                bool apex = selection.Feature == TriangleConeContactSelection.Apex;
                point = TriangleCylinderRimWitnesses.GetAnalyticPoint(geometry, triangle, selection.Mask,
                    localNormal[..(3 * Words)], localSigns[..3], apex ? -1 : 1, apex ? default : geometry.Radius);
                if (!apex)
                {
                    // Scale the exact radial direction before normalization;
                    // a sub-raw full-normal component must not erase a rim.
                    Slot(localNormal, 1).Clear(); localSigns[1] = 0;
                    var radialCandidate = new ConvexContactCandidate(localNormal, localSigns, 0);
                    radialPoint = -WideConvexPrismRelations.GetConvexContactCandidateScaledNormal(radialCandidate, radius);
                }
                coneAnchor = TriangleCircularGeometry.GetSupport(center, coneRotation, Signed192.Raw(height), radialPoint, apex ? -1 : 1);
            }
        }
        contact = new FixedContactAnchors(new FixedPointAnchor(origin, rotation, point), coneAnchor,
            normal, depth, clamped);
        return true;
    }

    private static bool KeepAnalytic(in TriangleCircularGeometry geometry, Fixed64 height, Fixed64 radius,
        ref TriangleConeContactSelection selection)
    {
        Span<ulong> direction = stackalloc ulong[3 * Words];
        Span<int> signs = stackalloc int[3];
        WriteDirection(new WideAxis3(default, Signed320.One, default), direction, signs);
        KeepAxis(geometry, direction, signs, true, ref selection);
        // Horizontal faces duplicate the already tested +/-Up support masks
        // and gaps. Their equal candidates retain the earlier axial winner.
        if (!geometry.FaceNormal.X.IsZero || !geometry.FaceNormal.Z.IsZero)
        {
            WriteDirection(geometry.FaceNormal, direction, signs);
            KeepAxis(geometry, direction, signs, true, ref selection);
        }
        if (selection.Separated) return false;
        if (HasFaceMinimumCertificate(geometry, selection.Candidate)) return true;
        TriangleConeGeneratorFeatures.KeepAll(geometry, height, radius, ref selection);
        for (int vertex = 0; vertex < 3 && !selection.Separated; vertex++)
        {
            WideAxis3 p = geometry.Vertex(vertex), edge = geometry.Edge(vertex);
            WriteDirection(new WideAxis3(p.X, default, p.Z), direction, signs);
            KeepAxis(geometry, direction, signs, false, ref selection);
            WriteDirection(new WideAxis3(WideArithmetic.Negate(edge.Z), default, edge.X), direction, signs);
            KeepAxis(geometry, direction, signs, false, ref selection);
            if (edge.Y.IsZero || edge.X.IsZero && edge.Z.IsZero)
                continue;
            CircularRimContactAlgebra.GetBasis(edge, out WideAxis3 first, out WideAxis3 second);
            WriteDirection(first, direction, signs); KeepAxis(geometry, direction, signs, false, ref selection);
            WriteDirection(second, direction, signs); KeepAxis(geometry, direction, signs, false, ref selection);
            ProjectPerpendicular(edge, new WideAxis3(default, Signed320.One, default), direction, signs);
            KeepAxis(geometry, direction, signs, false, ref selection);
            ProjectPerpendicular(edge, geometry.CapOffset(vertex, 1), direction, signs);
            KeepAxis(geometry, direction, signs, false, ref selection);
        }
        return false;
    }

    private static bool HasFaceMinimumCertificate(in TriangleCircularGeometry geometry, ConvexContactCandidate best)
    {
        // A cone contains its full axial segment and each base-disk diameter.
        // After the nonnegative +/-face tests, a face-plane radius-d disk
        // about the segment's projection minus that segment contains ball(d).
        // The retained support attains d, proving the global minimum.
        if (geometry.FaceNormal.X.IsZero && geometry.FaceNormal.Z.IsZero)
            return geometry.HasFaceDiskMinimumCertificate(new WideAxis3(default, Signed320.One, default), best);
        if (!geometry.FaceNormal.Y.IsZero)
            return false;
        WideAxis3 axis = geometry.FaceNormal.Z.IsZero
            ? new WideAxis3(Signed320.One, default, default)
            : geometry.FaceNormal.X.IsZero ? new WideAxis3(default, default, Signed320.One) : default;
        if (axis.IsZero)
            return false;
        // Translate only the certificate to the base center. Original
        // geometry and selection remain authoritative for paired witnesses.
        // Canonical axes keep tangents <231 and shifted dots <431 bits,
        // within the shared certificate's established fixed-carrier bounds.
        TriangleCircularGeometry baseCentered = geometry.AtCoreRegion(new WideAxis3(default, geometry.HalfHeight, default), 1);
        return baseCentered.HasFaceDiskMinimumCertificate(axis, best);
    }

    private static void KeepAxis(in TriangleCircularGeometry geometry, scoped Span<ulong> direction,
        scoped Span<int> directionSigns, bool compactGenerator, ref TriangleConeContactSelection selection)
    {
        if (selection.Separated || directionSigns[0] == 0 && directionSigns[1] == 0 && directionSigns[2] == 0)
            return;
        for (int orientation = 0; orientation < 2 && !selection.Separated; orientation++)
        {
            KeepDirection(geometry, direction, directionSigns, compactGenerator, ref selection);
            for (int component = 0; component < 3; component++) directionSigns[component] = -directionSigns[component];
        }
    }

    private static void KeepDirection(in TriangleCircularGeometry geometry, scoped ReadOnlySpan<ulong> direction,
        scoped ReadOnlySpan<int> directionSigns, bool compactGenerator, ref TriangleConeContactSelection selection)
    {
        Span<ulong> values = stackalloc ulong[ConvexContactCandidate.Slots * Words];
        Span<int> signs = stackalloc int[ConvexContactCandidate.Slots];
        Span<ulong> work = stackalloc ulong[10 * Words];
        Span<ulong> rational = Slot(work, 0), current = Slot(work, 1), difference = Slot(work, 2);
        Span<ulong> temporary = Slot(work, 3), radial = Slot(work, 4), radiusSquared = Slot(work, 5);
        Span<ulong> metric = Slot(work, 6), denominator = Slot(work, 7), axial = Slot(work, 8), scale = Slot(work, 9);
        values.Clear(); signs.Clear();
        direction.CopyTo(values); directionSigns.CopyTo(signs);
        Dot(geometry.A, direction, directionSigns, rational, out int rationalSign);
        int mask = 1;
        for (int vertex = 1; vertex < 3; vertex++)
        {
            Dot(geometry.Vertex(vertex), direction, directionSigns, current, out int currentSign);
            current.CopyTo(difference); int comparison = currentSign;
            Add(rational, -rationalSign, difference, ref comparison);
            if (comparison > 0) { current.CopyTo(rational); rationalSign = currentSign; mask = 1 << vertex; }
            else if (comparison == 0) mask |= 1 << vertex;
        }
        WideArithmetic.MultiplyMagnitudes(Slot(direction, 0), Slot(direction, 0), radial);
        WideArithmetic.MultiplyMagnitudes(Slot(direction, 2), Slot(direction, 2), temporary);
        WideArithmetic.AddMagnitudeInto(temporary, radial);
        Import(geometry.Radius, current);
        WideArithmetic.MultiplyMagnitudes(current, current, radiusSquared);
        WideArithmetic.MultiplyMagnitudes(radial, radiusSquared, current);
        current.CopyTo(radiusSquared);
        Import(geometry.HalfHeight, current);
        WideArithmetic.MultiplyMagnitudes(current, Slot(direction, 1), axial);
        int side = 1;
        if (directionSigns[1] < 0)
        {
            WideArithmetic.AddEqualMagnitudes(axial, axial, current);
            WideArithmetic.MultiplyMagnitudes(current, current, temporary);
            side = WideArithmetic.CompareMagnitudeEqualLength(radiusSquared, temporary);
        }
        SumSquares(direction, metric);
        if (side == 0)
        {
            // Only face-sized axes reach this owner. Larger K=0/projected
            // chart directions defer equality to the complete generator fan.
            if (compactGenerator)
                TriangleConeGeneratorFeatures.Keep(geometry, values, signs, metric, ref selection);
            return;
        }
        bool apex = side < 0;
        Add(axial, directionSigns[1] * (apex ? -1 : 1), rational, ref rationalSign);
        if (apex) radiusSquared.Clear();
        Import(Signed320.ExtendValue(geometry.RawScale), scale);
        WideArithmetic.MultiplyMagnitudes(scale, scale, temporary);
        WideArithmetic.MultiplyMagnitudes(metric, temporary, denominator);
        int gapSign = BuildRadialCandidate(rational, rationalSign, radiusSquared, denominator, values, signs);
        int feature = apex ? TriangleConeContactSelection.Apex
            : IsZero(radial) ? TriangleConeContactSelection.BasePole : TriangleConeContactSelection.BaseRim;
        selection.Keep(values, signs, gapSign, mask, feature);
    }

}
