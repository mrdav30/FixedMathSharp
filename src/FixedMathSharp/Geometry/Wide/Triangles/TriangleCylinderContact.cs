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
        out FixedContactAnchors contact, out bool isCapFaceContact)
    {
        contact = default; isCapFaceContact = false;
        if (triangle.IsDegenerate)
            return false;
        var geometry = new TriangleCylinderGeometry(triangle, origin, rotation,
            cylinderCenter, cylinderRotation, height, radius);
        Span<ulong> values = stackalloc ulong[ConvexContactCandidate.Slots * Words];
        Span<int> signs = stackalloc int[ConvexContactCandidate.Slots];
        Span<ulong> direction = stackalloc ulong[3 * Words];
        Span<int> directionSigns = stackalloc int[3];
        if (!TriangleCylinderAnalyticFeatures.TryGetBest(geometry, values, signs, direction, directionSigns,
                out int gapSign, out int mask, out bool faceMinimumCertified))
            return false;
        var best = new ConvexContactCandidate(values, signs, gapSign);
        Vector3d normal = default, radialPoint = default, rootPoint = default;
        Fixed64 depth = default;
        bool clamped = false, edgeWinner = false;
        int cap = directionSigns[1];
        if (radius != Fixed64.Zero && !faceMinimumCertified)
        {
            if (!TriangleCylinderEdgeContacts.TryGetContact(geometry, triangle, best, radius, out edgeWinner,
                    out normal, out radialPoint, out rootPoint, out depth, out clamped, out int edgeMask, out int edgeCap))
                return false;
            if (edgeWinner)
            {
                mask = edgeMask; cap = edgeCap;
            }
        }
        bool pole = !edgeWinner && directionSigns[0] == 0 && directionSigns[2] == 0;
        if (!edgeWinner)
        {
            normal = WideConvexPrismRelations.GetConvexContactCandidateNormal(best);
            GetAnalyticDepth(geometry, best, directionSigns, mask, out depth, out clamped);
            if (!pole)
            {
                // Round R*Nrad/|Nrad| directly from the selected exact direction.
                // A sub-raw full-normal component must not erase a finite rim.
                direction.CopyTo(values[..(3 * Words)]);
                directionSigns.CopyTo(signs[..3]);
                Slot(values, 1).Clear(); signs[1] = 0;
                radialPoint = -WideConvexPrismRelations.GetConvexContactCandidateScaledNormal(best, radius);
            }
        }
        isCapFaceContact = pole && mask == 7;
        FixedPointAnchor cylinderAnchor = GetSupport(cylinderCenter, cylinderRotation,
            height, radialPoint, cap);
        FixedPointAnchor triangleAnchor;
        if (cap == 0)
        {
            Vector3d point = TriangleCylinderWitnesses.GetSidePoint(geometry, triangle, mask,
                direction, directionSigns, out Fixed64 axial, out int boundaryCap);
            triangleAnchor = new FixedPointAnchor(origin, rotation, point);
            cylinderAnchor = boundaryCap != 0
                ? GetSupport(cylinderCenter, cylinderRotation, height, radialPoint, boundaryCap)
                : new FixedPointAnchor(cylinderCenter, cylinderRotation,
                    new Vector3d(Fixed64.Zero, axial, Fixed64.Zero), radialPoint);
        }
        else
        {
            Vector3d projectedRadial = default;
            Vector3d point = edgeWinner ? rootPoint
                : pole ? TriangleCylinderWitnesses.GetCapPoint(geometry, triangle, mask, out projectedRadial)
                : TriangleCylinderRimWitnesses.GetAnalyticPoint(geometry, triangle, mask, direction, directionSigns, cap);
            triangleAnchor = new FixedPointAnchor(origin, rotation, point);
            if (pole)
                cylinderAnchor = new FixedPointAnchor(cylinderCenter, cylinderRotation,
                    cylinderAnchor.LocalPoint, projectedRadial, cylinderAnchor.ExactLocalTerm);
        }
        contact = new FixedContactAnchors(triangleAnchor, cylinderAnchor, normal, depth, clamped);
        return true;
    }

    private static void GetAnalyticDepth(in TriangleCylinderGeometry geometry,
        ConvexContactCandidate candidate, ReadOnlySpan<int> directionSigns, int mask,
        out Fixed64 depth, out bool clamped)
    {
        int components = (directionSigns[0] == 0 ? 0 : 1)
            + (directionSigns[1] == 0 ? 0 : 1) + (directionSigns[2] == 0 ? 0 : 1);
        if (components != 1)
        {
            WideConvexPrismRelations.GetRoundedConvexContactCandidateDepth(candidate, Fixed64.Zero, out depth, out clamped);
            return;
        }
        // A single local component cancels its magnitude from the normalized
        // support gap. Every vertex in the selected mask has this coordinate.
        int axis = directionSigns[0] != 0 ? 0 : directionSigns[1] != 0 ? 1 : 2;
        int vertex = (mask & 1) != 0 ? 0 : (mask & 2) != 0 ? 1 : 2;
        Signed320 coordinate = Component(geometry.Vertex(vertex), axis);
        Signed320 numerator = WideArithmetic.AddSigned320(axis == 1 ? geometry.HalfHeight : geometry.Radius,
            directionSigns[axis] < 0 ? WideArithmetic.Negate(coordinate) : coordinate);
        // Both numerator and exact clamp threshold fit 199 bits. Test exact
        // overflow before nearest-even rounding, including Max+fractions.
        Signed320 maximum = WideArithmetic.MultiplySigned192(geometry.RawScale, Signed192.Raw(Fixed64.MaxValue));
        clamped = WideArithmetic.SubtractSigned320(numerator, maximum).Sign > 0;
        if (clamped)
        {
            depth = Fixed64.MaxValue;
            return;
        }
        bool represented = Fixed64.TryGetSignedRawRatio(Signed576.ExtendValue(numerator),
            Signed576.ExtendValue(Signed320.ExtendValue(geometry.RawScale)), out depth);
        System.Diagnostics.Debug.Assert(represented);
    }

    private static FixedPointAnchor GetSupport(Vector3d center, FixedQuaternion rotation,
        Signed192 height, Vector3d radialPoint, int cap)
    {
        ulong floor = height.Low >> 1;
        Fixed64 halfHeight = Fixed64.FromRaw((long)(floor + ((height.Low & 1UL) != 0 && (floor & 1UL) != 0 ? 1UL : 0UL)));
        Vector3d axial = new(Fixed64.Zero, cap >= 0 ? -halfHeight : halfHeight, Fixed64.Zero);
        // Public cylinder heights fit one Fixed64 and can retain an odd raw
        // half-height. Slab heights may span 64 unsigned bits but are even,
        // because their authored half-thickness was representable and exact.
        FixedPointAnchorTerm3d term = height.Low <= long.MaxValue
            ? FixedPointAnchorTerm3d.CreateCenteredAxisSupport(Vector3d.Up,
                Fixed64.FromRaw(cap >= 0 ? -(long)height.Low : (long)height.Low),
                Vector3d.Zero, Fixed64.Zero, axial, Vector3d.Zero)
            : default;
        return new FixedPointAnchor(center, rotation, axial, radialPoint, term);
    }

}
