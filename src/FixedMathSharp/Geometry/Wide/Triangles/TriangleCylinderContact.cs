//=======================================================================
// MIT License, Copyright (c) 2024-present David Oravsky (mrdav30)
// See LICENSE file in the project root for full license information.
//=======================================================================
using System;
using static FixedMathSharp.Geometry.CylinderContactAlgebra;

namespace FixedMathSharp.Geometry;

/// <summary>Complete finite-cylinder/triangle support-fan contact ownership.</summary>
internal static class TriangleCylinderContact
{
    internal static bool TryGetContact(FixedTriangle triangle, Vector3d origin, FixedQuaternion rotation,
        Vector3d cylinderCenter, FixedQuaternion cylinderRotation, Signed192 height, Fixed64 radius,
        out FixedContactAnchors contact, out bool isCapFaceContact,
        Vector2d coreDirection = default, Fixed64 coreLength = default)
    {
        contact = default; isCapFaceContact = false;
        if (triangle.IsDegenerate)
            return false;
        var geometry = new TriangleCircularGeometry(triangle, origin, rotation,
            cylinderCenter, cylinderRotation, height, radius);
        return TryGetContact(triangle, geometry, origin, rotation, cylinderCenter, cylinderRotation,
            height, radius, out contact, out isCapFaceContact, coreDirection, coreLength);
    }

    internal static bool TryGetContact(FixedTriangle triangle, in ScaledRigidFrame3d sourceFrame,
        Vector3d cylinderCenter, FixedQuaternion cylinderRotation, Signed192 height, Fixed64 radius,
        out FixedContactAnchors contact, out bool isCapFaceContact,
        Vector2d coreDirection = default, Fixed64 coreLength = default)
    {
        if (sourceFrame.IsRigid)
            return TryGetContact(triangle, sourceFrame.RigidOrigin, sourceFrame.Rotation,
                cylinderCenter, cylinderRotation, height, radius, out contact, out isCapFaceContact, coreDirection, coreLength);
        contact = default; isCapFaceContact = false;
        var geometry = new TriangleCircularGeometry(triangle, sourceFrame, cylinderCenter, cylinderRotation, height, radius);
        // Authored area can be sub-epsilon while exact scale gives a large
        // physical face. Only exact collapse invalidates the affine fan.
        if (geometry.FaceNormal.IsZero) return false;
        return TryGetContact(triangle, geometry, cylinderCenter, cylinderRotation, cylinderCenter, cylinderRotation,
            height, radius, out contact, out isCapFaceContact, coreDirection, coreLength);
    }

    internal static bool DoesPenetrate(FixedTriangle triangle, in ScaledRigidFrame3d sourceFrame,
        Vector3d center, FixedQuaternion rotation, Fixed64 height, Fixed64 radius)
    {
        var geometry = new TriangleCircularGeometry(triangle, sourceFrame, center, rotation, Signed192.Raw(height), radius);
        if (geometry.FaceNormal.IsZero) return false;
        return TryGetContact(triangle, geometry, center, rotation, center, rotation, Signed192.Raw(height), radius,
            out _, out _, default, default, strictOnly: true);
    }

    private static bool TryGetContact(FixedTriangle triangle, in TriangleCircularGeometry inputGeometry,
        Vector3d origin, FixedQuaternion rotation, Vector3d cylinderCenter, FixedQuaternion cylinderRotation,
        Signed192 height, Fixed64 radius, out FixedContactAnchors contact, out bool isCapFaceContact,
        Vector2d coreDirection, Fixed64 coreLength, bool strictOnly = false)
    {
        contact = default; isCapFaceContact = false;
        WideDirection3d coreOffset = default, coreAxis = default;
        TriangleCircularGeometry coreGeometry;
        scoped ref readonly TriangleCircularGeometry geometry = ref inputGeometry;
        if (coreLength != Fixed64.Zero)
        {
            coreGeometry = inputGeometry.WithCore(coreDirection, coreLength, out coreOffset);
            geometry = ref coreGeometry;
            coreAxis = new WideDirection3d(Signed576.ExtendValue(Signed192.Raw(coreDirection.X)), default,
                Signed576.ExtendValue(Signed192.Raw(coreDirection.Y)));
        }
        Span<ulong> values = stackalloc ulong[ConvexContactCandidate.Slots * Words];
        Span<int> signs = stackalloc int[ConvexContactCandidate.Slots];
        Span<ulong> direction = stackalloc ulong[3 * Words];
        Span<int> directionSigns = stackalloc int[3];
        if (!TriangleCylinderAnalyticFeatures.TryGetBest(geometry, coreOffset, coreAxis, values, signs, direction, directionSigns,
                out int gapSign, out int mask, out int coreSign, out bool faceMinimumCertified))
            return false;
        if (strictOnly && gapSign == 0) return false;
        bool strictOverlap = gapSign > 0;
        var best = new ConvexContactCandidate(values, signs, gapSign);
        TriangleCircularGeometry endpointGeometry;
        scoped ref readonly TriangleCircularGeometry analyticGeometry = ref geometry;
        if (coreSign != 0)
        {
            endpointGeometry = geometry.AtCoreRegion(coreOffset, coreSign);
            analyticGeometry = ref endpointGeometry;
        }
        analyticGeometry.GetAnalyticDepth(best, directionSigns, mask, out Fixed64 depth, out bool clamped);
        Vector3d normal = default, radialPoint = default, rootPoint = default;
        bool edgeWinner = false;
        int integralRadialMask = 5;
        int cap = directionSigns[1];
        if (radius != Fixed64.Zero && !faceMinimumCertified)
        {
            if (!TriangleCylinderEdgeContacts.TryGetContact(geometry, coreOffset, coreAxis, coreDirection, coreLength,
                    triangle, best, depth, clamped, radius, out edgeWinner, out normal, out radialPoint, out rootPoint, out Fixed64 edgeDepth,
                    out bool edgeClamped, out int edgeMask, out int edgeCap, out int edgeCoreSign, out integralRadialMask, out strictOverlap, materialize: !strictOnly))
                return false;
            if (edgeWinner)
            {
                mask = edgeMask; cap = edgeCap; coreSign = edgeCoreSign;
                depth = edgeDepth; clamped = edgeClamped;
            }
        }
        if (strictOnly) return strictOverlap;
        bool pole = !edgeWinner && directionSigns[0] == 0 && directionSigns[2] == 0;
        if (!edgeWinner)
        {
            normal = WideConvexPrismRelations.GetConvexContactCandidateNormal(best);
            if (!pole && (coreAxis.IsZero || coreSign != 0))
            {
                // Round R*Nrad/|Nrad| directly from the selected exact direction.
                // A sub-raw full-normal component must not erase a finite rim.
                direction.CopyTo(values[..(3 * Words)]);
                directionSigns.CopyTo(signs[..3]);
                Slot(values, 1).Clear(); signs[1] = 0;
                radialPoint = -WideConvexPrismRelations.GetConvexContactCandidateScaledNormal(best, radius);
            }
        }
        Vector3d core = default;
        if (coreSign != 0)
        {
            core = TriangleCapsuleSlabEndpointWitnesses.GetCore(coreDirection, coreLength, coreSign,
                out FixedPointAnchorTerm3d term);
            if (!edgeWinner && radius != Fixed64.Zero)
                radialPoint = TriangleCapsuleSlabEndpointWitnesses.AdjustAnalytic(direction, directionSigns,
                    radius, core, term, radialPoint, out integralRadialMask);
        }
        isCapFaceContact = pole && mask == 7;
        FixedPointAnchor cylinderAnchor = TriangleCircularGeometry.GetSupport(cylinderCenter, cylinderRotation,
            height, radialPoint, cap);
        FixedPointAnchor triangleAnchor;
        if (!coreAxis.IsZero && coreSign == 0)
        {
            TriangleCapsuleSlabWitnesses.GetPoints(geometry, triangle, mask, direction, directionSigns,
                coreOffset, coreDirection, coreLength, ref radialPoint, out Vector3d point,
                out Vector3d coreAndAxialPoint, out FixedPointAnchorTerm3d exactTerm);
            triangleAnchor = new FixedPointAnchor(origin, rotation, point);
            cylinderAnchor = new FixedPointAnchor(cylinderCenter, cylinderRotation, coreAndAxialPoint, radialPoint, exactTerm);
        }
        else if (cap == 0)
        {
            Vector3d point = TriangleCylinderWitnesses.GetSidePoint(analyticGeometry, triangle, mask,
                direction, directionSigns, out Fixed64 axial, out int boundaryCap);
            triangleAnchor = new FixedPointAnchor(origin, rotation, point);
            cylinderAnchor = boundaryCap != 0
                ? TriangleCircularGeometry.GetSupport(cylinderCenter, cylinderRotation, height, radialPoint, boundaryCap)
                : new FixedPointAnchor(cylinderCenter, cylinderRotation,
                    new Vector3d(Fixed64.Zero, axial, Fixed64.Zero), radialPoint);
        }
        else
        {
            Vector3d projectedRadial = default;
            Vector3d point = edgeWinner ? rootPoint
                : pole ? TriangleCylinderWitnesses.GetCapPoint(analyticGeometry, triangle, mask, out projectedRadial)
                : TriangleCylinderRimWitnesses.GetAnalyticPoint(analyticGeometry, triangle, mask, direction, directionSigns, cap, analyticGeometry.Radius);
            triangleAnchor = new FixedPointAnchor(origin, rotation, point);
            if (pole)
                cylinderAnchor = new FixedPointAnchor(cylinderCenter, cylinderRotation,
                    cylinderAnchor.LocalPoint, projectedRadial, cylinderAnchor.ExactLocalTerm);
        }
        if (coreSign != 0)
            cylinderAnchor = AddCoreEndpoint(cylinderAnchor, core, coreDirection, coreLength, coreSign, integralRadialMask);
        contact = new FixedContactAnchors(triangleAnchor, cylinderAnchor, normal, depth, clamped);
        return true;
    }

    private static FixedPointAnchor AddCoreEndpoint(FixedPointAnchor anchor, Vector3d core,
        Vector2d axis, Fixed64 length, int coreSign, int integralRadialMask)
    {
        FixedPointAnchorTerm3d term = TriangleCapsuleSlabEndpointWitnesses.CreateTerm(axis,
            coreSign > 0 ? -length : length, core, integralRadialMask);
        // Stadium slab heights are twice an authored half-thickness, so the
        // cap coordinate is exact and only the half-core residual remains.
        return new FixedPointAnchor(anchor.Origin, anchor.Rotation,
            new Vector3d(core.X, anchor.LocalPoint.Y, core.Z), anchor.LocalDisplacement, term);
    }

}
