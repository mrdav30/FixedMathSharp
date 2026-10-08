//=======================================================================
// TriangleConeContact.Patch.cs
//=======================================================================
// MIT License, Copyright (c) 2024-present David Oravsky (mrdav30)
// See LICENSE file in the project root for full license information.
//=======================================================================
using System;
using static FixedMathSharp.Geometry.CylinderContactAlgebra;

namespace FixedMathSharp.Geometry;

/// <content>
/// Provides exact sufficient face certificates for filled coplanar patches.
/// </content>
internal static partial class TriangleConeContact
{
    /// <summary>
    /// Certifies a minimum face exit for one connected, filled coplanar patch.
    /// </summary>
    /// <remarks>
    /// The borrower supplies a nonempty complete perimeter, including holes,
    /// with valid indices and nonzero edges, and normalized rigid frames. This
    /// method neither validates topology nor treats the patch as a convex solid.
    /// False means that the sufficient certificate does not apply; the complete
    /// finite-triangle contact remains authoritative for exposed features.
    /// Optional witness bounds must enclose the complete patch in its authored
    /// local frame; two committed AABB extrema suffice. Without them, the
    /// supplied patch vertices bound final witness-coordinate rounding.
    /// </remarks>
    internal static bool TryGetPatchFaceContact(FixedTriangle seed, Vector3d origin,
        FixedQuaternion rotation, ReadOnlySpan<Vector3d> patchVertices,
        ReadOnlySpan<int> orientedBoundaryPairs, Vector3d center,
        FixedQuaternion coneRotation, Fixed64 height, Fixed64 radius,
        out FixedContactAnchors contact, ReadOnlySpan<Vector3d> witnessBounds = default)
    {
        contact = default;
        if (seed.IsDegenerate || orientedBoundaryPairs.IsEmpty)
            return false;

        var geometry = new TriangleCircularGeometry(seed, origin, rotation, center,
            coneRotation, Signed192.Raw(height), radius);
        Span<ulong> values = stackalloc ulong[ConvexContactCandidate.Slots * Words];
        Span<int> signs = stackalloc int[ConvexContactCandidate.Slots];
        var selection = new TriangleConeContactSelection(values, signs);
        Span<ulong> direction = stackalloc ulong[3 * Words];
        Span<int> directionSigns = stackalloc int[3];
        bool axialFace = geometry.FaceNormal.X.IsZero && geometry.FaceNormal.Z.IsZero;
        // Preserve the single-triangle owner's +Up winner at an axial tie,
        // independently of authored face winding.
        WriteDirection(axialFace ? new WideAxis3(default, Signed320.One, default)
            : geometry.FaceNormal, direction, directionSigns);
        KeepAxis(geometry, direction, directionSigns, true, ref selection);
        if (selection.Separated)
            return false;
        System.Diagnostics.Debug.Assert(selection.Mask == 7);

        var frame = new CylinderPolytopeFrame(origin, rotation, height, radius, center, coneRotation);
        Signed192 scale = WideArithmetic.AddSigned192(frame.Denominator, frame.Denominator);
        seed.GetExactNormal(out Signed192 nx, out Signed192 ny, out Signed192 nz, out _);
        var normal = new WideAxis3(Signed320.ExtendValue(nx), Signed320.ExtendValue(ny), Signed320.ExtendValue(nz));
        bool cardinalSide = geometry.FaceNormal.Y.IsZero
            && (geometry.FaceNormal.X.IsZero || geometry.FaceNormal.Z.IsZero);
        bool certified = TryCertifyPatchChord(seed, origin, rotation, patchVertices,
            orientedBoundaryPairs, center, coneRotation, height, radius, geometry,
            frame, scale, normal, axialFace, cardinalSide, ref selection);
        if (!certified && !HasPatchProjectionCertificate(seed, origin, rotation,
                patchVertices, orientedBoundaryPairs, center, coneRotation, height, radius,
                geometry, frame, scale, normal, selection.Candidate))
            return false;

        // At the generator seam the apex is an exact, equally supporting face
        // witness. This face certificate needs no generator interpolation.
        if (selection.Feature == TriangleConeContactSelection.Generator)
            selection.Feature = TriangleConeContactSelection.Apex;
        geometry.GetAnalyticDepth(selection.Candidate, signs, selection.Mask,
            out Fixed64 depth, out bool clamped);
        System.Diagnostics.Debug.Assert(!clamped);
        GetAnalyticWitnesses(geometry, seed, height, radius, center, coneRotation,
            ref selection, out Vector3d worldNormal, out Vector3d point, out FixedPointAnchor coneAnchor,
            witnessBounds.IsEmpty ? patchVertices : witnessBounds);
        contact = new FixedContactAnchors(new FixedPointAnchor(origin, rotation, point),
            coneAnchor, worldNormal, depth, clamped);
        return true;
    }

    private static bool TryCertifyPatchChord(FixedTriangle seed, Vector3d origin,
        FixedQuaternion rotation, ReadOnlySpan<Vector3d> vertices, ReadOnlySpan<int> boundary,
        Vector3d center, FixedQuaternion coneRotation, Fixed64 height, Fixed64 radius,
        in TriangleCircularGeometry geometry, in CylinderPolytopeFrame frame, Signed192 scale,
        WideAxis3 normal, bool axialFace, bool cardinalSide, ref TriangleConeContactSelection selection)
    {
        if (!axialFace && !cardinalSide
            && selection.Feature != TriangleConeContactSelection.Apex
            && selection.Feature != TriangleConeContactSelection.Generator)
            return false;

        int cap = cardinalSide || selection.Feature == TriangleConeContactSelection.BasePole ? 1 : -1;
        FixedPointAnchor seedPoint = TriangleCircularGeometry.GetSupport(center, coneRotation,
            Signed192.Raw(height), Vector3d.Zero, cap);
        if (!WideTriangleRelations.ContainsProjection(seed, origin, rotation, seedPoint))
            return false;

        WideAxis3 core = frame.Transform(Vector3d.Zero);
        WideAxis3 halfAxis = WideRigidProjection.TransformLocalAxis(frame.Basis,
            default, Signed192.Raw(height), default);
        WideAxis3 first = AddPatchAxis(core, halfAxis);
        WideAxis3 second = TriangleCircularGeometry.Subtract(core, halfAxis);
        if (cardinalSide)
        {
            Signed192 diameter = WideArithmetic.AddSigned192(Signed192.Raw(radius), Signed192.Raw(radius));
            WideAxis3 radial = WideRigidProjection.TransformLocalAxis(frame.Basis,
                geometry.FaceNormal.Z.IsZero ? diameter : default, default,
                geometry.FaceNormal.Z.IsZero ? default : diameter);
            first = AddPatchAxis(second, radial);
            second = TriangleCircularGeometry.Subtract(second, radial);
        }

        // Axial/cardinal chords attain both face extrema. For a tilted apex
        // winner only the base-center clearance needs proof; the apex attains d.
        // The base center is already on the opposite plane side: the apex
        // support condition gives H*|n.Y| >= R*|n.radial|. If the base center
        // shared the apex side, the opposite base-rim exit would be strictly
        // shorter than the retained apex exit. The earlier +/-face ranking
        // excludes that case (generator equality permits a base on the plane).
        if (!axialFace && !cardinalSide)
        {
            Span<ulong> work = stackalloc ulong[6 * Words];
            Span<ulong> clearanceValues = stackalloc ulong[ConvexContactCandidate.Slots * Words];
            Span<int> clearanceSigns = stackalloc int[ConvexContactCandidate.Slots];
            PreparePatchClearance(scale, work, clearanceValues, clearanceSigns);
            WideAxis3 relative = TriangleCircularGeometry.Subtract(second, ScalePatchPoint(seed.A, scale));
            Signed576 projection = WideAxis3.Dot(relative, normal);
            if (!TriangleCircularGeometry.HasFaceDiskClearance(projection, projection.IsZero ? 0 : 1, normal,
                    selection.Candidate, work, clearanceValues, clearanceSigns))
                return false;
        }
        return HasPatchChordTubeCertificate(vertices, boundary, scale, normal,
            first, second, selection.Candidate);
    }

    private static bool HasPatchChordTubeCertificate(ReadOnlySpan<Vector3d> vertices,
        ReadOnlySpan<int> boundary, Signed192 scale, WideAxis3 normal,
        WideAxis3 first, WideAxis3 second, ConvexContactCandidate best)
    {
        Span<ulong> work = stackalloc ulong[6 * Words];
        Span<ulong> clearanceValues = stackalloc ulong[ConvexContactCandidate.Slots * Words];
        Span<int> clearanceSigns = stackalloc int[ConvexContactCandidate.Slots];
        PreparePatchClearance(scale, work, clearanceValues, clearanceSigns);
        // A segment inside the cone spans both plane sides by at least d.
        // Its projected radius-d tube cannot cross any complete perimeter
        // line, and one endpoint projection is in the seed. Therefore every
        // translation shorter than d preserves a patch intersection. The
        // retained exact face support attains d, proving it globally minimal.
        for (int index = 0; index < boundary.Length; index += 2)
        {
            Vector3d a = vertices[boundary[index]], b = vertices[boundary[index + 1]];
            WideAxis3 tangent = WideAxis3.Cross(normal, TriangleCircularGeometry.Subtract(
                RawPatchPoint(b), RawPatchPoint(a)));
            WideAxis3 offset = ScalePatchPoint(a, scale);
            Signed576 firstDistance = WideAxis3.Dot(TriangleCircularGeometry.Subtract(first, offset), tangent);
            Signed576 secondDistance = WideAxis3.Dot(TriangleCircularGeometry.Subtract(second, offset), tangent);
            if (firstDistance.Sign == 0 || firstDistance.Sign != secondDistance.Sign
                || !TriangleCircularGeometry.HasFaceDiskClearance(firstDistance, 1, tangent,
                    best, work, clearanceValues, clearanceSigns)
                || !TriangleCircularGeometry.HasFaceDiskClearance(secondDistance, 1, tangent,
                    best, work, clearanceValues, clearanceSigns))
                return false;
        }
        return true;
    }

    private static bool HasPatchProjectionCertificate(FixedTriangle seed, Vector3d origin,
        FixedQuaternion rotation, ReadOnlySpan<Vector3d> vertices, ReadOnlySpan<int> boundary,
        Vector3d center, FixedQuaternion coneRotation, Fixed64 height, Fixed64 radius,
        in TriangleCircularGeometry geometry, in CylinderPolytopeFrame meshFrame,
        Signed192 scale, WideAxis3 normal, ConvexContactCandidate best)
    {
        var centerAnchor = new FixedPointAnchor(center, FixedQuaternion.Identity, Vector3d.Zero);
        if (!WideTriangleRelations.ContainsProjection(seed, origin, rotation, centerAnchor))
            return false;
        WideAxis3 core = meshFrame.Transform(Vector3d.Zero);
        var coneFrame = new CylinderPolytopeFrame(center, coneRotation, height, radius, origin, rotation);
        Span<ulong> meshDirection = stackalloc ulong[3 * Words];
        Span<int> meshSigns = stackalloc int[3];
        Span<ulong> coneDirection = stackalloc ulong[3 * Words];
        Span<int> coneSigns = stackalloc int[3];
        Span<ulong> clearanceValues = stackalloc ulong[ConvexContactCandidate.Slots * Words];
        Span<int> clearanceSigns = stackalloc int[ConvexContactCandidate.Slots];
        // The complete cone projection enlarged by d stays within the patch
        // if its nearest exact support clears every complete perimeter line.
        // Seed membership fixes the connected filled component; absolute
        // center-side orientation also permits concave patches and holes.
        // Rational relative bases are exactly orthogonal, independently of
        // rounded public quaternion components. A raw mesh tangent<196 bits
        // transforms to <328 bits; point/cap/radius<198 give squared gaps<1056
        // and denominator<920. Cross-ranking stays<2000 bits, within the
        // established forty-word cone algebra, with no carrier narrowing.
        for (int index = 0; index < boundary.Length; index += 2)
        {
            Vector3d a = vertices[boundary[index]], b = vertices[boundary[index + 1]];
            WideAxis3 tangent = WideAxis3.Cross(normal, TriangleCircularGeometry.Subtract(
                RawPatchPoint(b), RawPatchPoint(a)));
            Signed576 projection = WideAxis3.Dot(TriangleCircularGeometry.Subtract(core, ScalePatchPoint(a, scale)), tangent);
            if (projection.IsZero)
                return false;
            WriteDirection(tangent, meshDirection, meshSigns);
            WriteWorldDirection(coneFrame.Basis, meshDirection, meshSigns,
                coneDirection, coneSigns, projection.Sign);
            TriangleCircularGeometry pointGeometry = geometry.AtPoint(coneFrame, a);
            var clearance = new TriangleConeContactSelection(clearanceValues, clearanceSigns);
            KeepDirection(pointGeometry, coneDirection, coneSigns, true, ref clearance);
            // The support owner encodes point-max minus cone-min. Negating
            // that signed gap gives the nearest cone-to-line clearance; its
            // squared exact candidate fields already have the same magnitude.
            if (WideConvexPrismRelations.CompareConvexContactCandidates(
                    new ConvexContactCandidate(clearanceValues, clearanceSigns, -clearance.GapSign), best) < 0)
                return false;
        }
        return true;
    }

    private static void PreparePatchClearance(Signed192 scale, Span<ulong> work,
        Span<ulong> values, Span<int> signs)
    {
        // Normal<130 and raw edge<65 give tangent<196 bits. Rational endpoint
        // offsets<198 give dots<396; norm²<394, scale²<262 and numerator²<792.
        // They fit the shared forty-word face-disk/candidate algebra without narrowing.
        work.Clear(); values.Clear(); signs.Clear();
        Import(Signed320.ExtendValue(scale), Slot(work, 3));
        WideArithmetic.MultiplyMagnitudes(Slot(work, 3), Slot(work, 3), Slot(work, 4));
    }

    private static WideAxis3 RawPatchPoint(Vector3d point) => new(
        Signed320.ExtendValue(Signed192.Raw(point.X)), Signed320.ExtendValue(Signed192.Raw(point.Y)),
        Signed320.ExtendValue(Signed192.Raw(point.Z)));

    private static WideAxis3 ScalePatchPoint(Vector3d point, Signed192 scale) => new(
        WideArithmetic.MultiplySigned192(Signed192.Raw(point.X), scale),
        WideArithmetic.MultiplySigned192(Signed192.Raw(point.Y), scale),
        WideArithmetic.MultiplySigned192(Signed192.Raw(point.Z), scale));

    private static WideAxis3 AddPatchAxis(WideAxis3 first, WideAxis3 second) => new(
        WideArithmetic.AddSigned320(first.X, second.X), WideArithmetic.AddSigned320(first.Y, second.Y),
        WideArithmetic.AddSigned320(first.Z, second.Z));
}
