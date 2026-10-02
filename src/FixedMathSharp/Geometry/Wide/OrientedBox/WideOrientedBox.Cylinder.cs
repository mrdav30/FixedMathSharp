//=======================================================================
// MIT License, Copyright (c) 2024-present David Oravsky (mrdav30)
// See LICENSE file in the project root for full license information.
//=======================================================================
using System;

namespace FixedMathSharp.Geometry;

/// <content>Exact minimum-depth contact with a finite cylinder.</content>
internal static partial class WideOrientedBox
{
    internal static bool TryGetCenteredCylinderContact(
        Vector3d center, FixedQuaternion orientation, Vector3d halfExtents,
        Vector3d cylinderCenter, FixedQuaternion cylinderRotation,
        Vector3d localCylinderAxisDirection, Fixed64 cylinderAxisLength,
        Fixed64 cylinderRadius, out FixedContactAnchors contact) =>
        TryGetCenteredCylinderContact(center, orientation, halfExtents,
            cylinderCenter, cylinderRotation, localCylinderAxisDirection,
            cylinderAxisLength, cylinderRadius, out contact, out _);

    internal static bool TryGetCenteredCylinderContact(
        Vector3d center, FixedQuaternion orientation, Vector3d halfExtents,
        Vector3d cylinderCenter, FixedQuaternion cylinderRotation,
        Vector3d localCylinderAxisDirection, Fixed64 cylinderAxisLength,
        Fixed64 cylinderRadius, out FixedContactAnchors contact,
        out CenteredCylinderContactFeature feature)
    {
        contact = default;
        feature = default;
        if (!CanOverlapCenteredCapsule(center, orientation, halfExtents,
                cylinderCenter, cylinderRotation, localCylinderAxisDirection,
                cylinderAxisLength, cylinderRadius))
            return false;

        var geometry = new BoxCylinderGeometry(center, orientation, halfExtents,
            cylinderCenter, cylinderRotation, localCylinderAxisDirection,
            cylinderAxisLength, cylinderRadius);
        Span<ulong> values = stackalloc ulong[ConvexContactCandidate.Slots * ConvexContactCandidate.Words];
        Span<int> signs = stackalloc int[ConvexContactCandidate.Slots];
        if (!BoxCylinderAnalyticFeatures.TryGetBest(geometry, values, signs,
                out int gapSign, out Vector3d localSigns))
            return false;
        var best = new ConvexContactCandidate(values, signs, gapSign);
        WideConvexPrismRelations.GetRoundedConvexContactCandidateDepth(best, Fixed64.Zero,
            out Fixed64 analyticDepth, out bool analyticClamped);
        bool edgeWinner = false;
        Vector3d normal = default;
        Fixed64 depth = default;
        bool clamped = false;
        if (cylinderRadius != Fixed64.Zero)
        {
            if (!BoxCylinderEdgeContacts.TryGetContact(geometry, best, analyticDepth, analyticClamped,
                    out edgeWinner, out normal, out depth, out clamped, out Vector3d edgeSigns))
                return false;
            if (edgeWinner)
                localSigns = edgeSigns;
        }
        if (!edgeWinner)
        {
            normal = WideConvexPrismRelations.GetConvexContactCandidateNormal(best);
            depth = analyticDepth;
            clamped = analyticClamped;
            feature = GetCylinderCapFace(geometry.Axis, localSigns);
        }

        // Preserve the existing lexicographically negative support tie. The
        // signs come from the exact selected direction, never its rounded normal.
        Vector3d localPoint = new(localSigns.X > Fixed64.Zero ? halfExtents.X : -halfExtents.X,
            localSigns.Y > Fixed64.Zero ? halfExtents.Y : -halfExtents.Y,
            localSigns.Z > Fixed64.Zero ? halfExtents.Z : -halfExtents.Z);
        FixedPointAnchor cylinderAnchor = WideGeometry.GetCenteredCylinderSupportAnchor(
            cylinderCenter, cylinderRotation, localCylinderAxisDirection,
            cylinderAxisLength, cylinderRadius, -normal);
        contact = new FixedContactAnchors(new FixedPointAnchor(center, orientation, localPoint),
            cylinderAnchor, normal, depth, clamped);
        return true;
    }

    private static CenteredCylinderContactFeature GetCylinderCapFace(WideAxis3 cylinderAxis, Vector3d localSigns)
    {
        int x = localSigns.X.m_rawValue.CompareTo(0L);
        int y = localSigns.Y.m_rawValue.CompareTo(0L);
        int z = localSigns.Z.m_rawValue.CompareTo(0L);
        if (x != 0 && y == 0 && z == 0 && cylinderAxis.Y.IsZero && cylinderAxis.Z.IsZero)
            return new CenteredCylinderContactFeature(0, x, -x * cylinderAxis.X.Sign);
        if (y != 0 && x == 0 && z == 0 && cylinderAxis.X.IsZero && cylinderAxis.Z.IsZero)
            return new CenteredCylinderContactFeature(1, y, -y * cylinderAxis.Y.Sign);
        if (z != 0 && x == 0 && y == 0 && cylinderAxis.X.IsZero && cylinderAxis.Y.IsZero)
            return new CenteredCylinderContactFeature(2, z, -z * cylinderAxis.Z.Sign);
        return default;
    }
}
