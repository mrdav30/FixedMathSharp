//=======================================================================
// WideOrientedBox.ConvexHull.cs
//=======================================================================
// MIT License, Copyright (c) 2024–present David Oravsky (mrdav30)
// See LICENSE file in the project root for full license information.
//=======================================================================

using System;

namespace FixedMathSharp.Geometry;

/// <content>
/// Convex hull contact generation logic for <see cref="WideOrientedBox"/>,
/// using separating axis tests against box axes, hull face normals, and edge cross products.
/// </content>
internal static partial class WideOrientedBox
{
    internal static bool TryGetConvexHullContact(
        Vector3d boxCenter,
        FixedQuaternion boxOrientation,
        Vector3d boxHalfExtents,
        Vector3d hullOrigin,
        FixedQuaternion hullOrientation,
        ReadOnlySpan<Vector3d> hullLocalOffsets,
        ReadOnlySpan<int> triangleVertexIndices,
        ReadOnlySpan<int> edgeVertexPairs,
        out FixedContactAnchors contact)
    {
        WideRationalBasis3d boxBasis = new(boxOrientation);
        WideRationalBasis3d hullBasis = new(hullOrientation);
        WideRationalBasis3d relativeBasis =
            WideRationalBasis3d.CreateRelative(boxBasis, hullBasis);
        Signed320 unit = Signed320.One;
        Span<WideAxis3> boxAxes = stackalloc WideAxis3[3]
        {
            new(unit, default, default),
            new(default, unit, default),
            new(default, default, unit),
        };
        GetRelativeLocalPointNumerators(
            hullOrigin,
            boxCenter,
            boxBasis,
            out Signed192 translationX,
            out Signed192 translationY,
            out Signed192 translationZ);
        var hullTranslation = new WideAxis3(
            WideArithmetic.MultiplySigned192(
                translationX,
                hullBasis.Denominator),
            WideArithmetic.MultiplySigned192(
                translationY,
                hullBasis.Denominator),
            WideArithmetic.MultiplySigned192(
                translationZ,
                hullBasis.Denominator));
        var best = default(WidePointSpanPenetration);
        for (int index = 0; index < boxAxes.Length; index++)
        {
            if (!TryKeepPointSpanAxis(
                    boxAxes[index],
                    boxHalfExtents,
                    relativeBasis,
                    hullTranslation,
                    hullLocalOffsets,
                    ref best))
            {
                contact = default;
                return false;
            }
        }

        for (int index = 0;
            index < triangleVertexIndices.Length;
            index += 3)
        {
            Vector3d first = hullLocalOffsets[
                triangleVertexIndices[index]];
            Vector3d second = hullLocalOffsets[
                triangleVertexIndices[index + 1]];
            Vector3d third = hullLocalOffsets[
                triangleVertexIndices[index + 2]];
            WideGeometry.GetDifferenceCrossProduct3D(
                second.X,
                first.X,
                second.Y,
                first.Y,
                second.Z,
                first.Z,
                third.X,
                first.X,
                third.Y,
                first.Y,
                third.Z,
                first.Z,
                out Signed192 normalX,
                out Signed192 normalY,
                out Signed192 normalZ);
            WideAxis3 axis = WideRigidProjection.TransformLocalAxis(
                relativeBasis,
                normalX,
                normalY,
                normalZ);
            if (!TryKeepPointSpanAxis(
                    axis,
                    boxHalfExtents,
                    relativeBasis,
                    hullTranslation,
                    hullLocalOffsets,
                    ref best))
            {
                contact = default;
                return false;
            }
        }

        for (int index = 0; index < edgeVertexPairs.Length; index += 2)
        {
            Vector3d start = hullLocalOffsets[edgeVertexPairs[index]];
            Vector3d end = hullLocalOffsets[edgeVertexPairs[index + 1]];
            WideAxis3 edge = WideRigidProjection.TransformLocalAxis(
                relativeBasis,
                WideArithmetic.SubtractSigned192(
                    Signed192.Raw(end.X),
                    Signed192.Raw(start.X)),
                WideArithmetic.SubtractSigned192(
                    Signed192.Raw(end.Y),
                    Signed192.Raw(start.Y)),
                WideArithmetic.SubtractSigned192(
                    Signed192.Raw(end.Z),
                    Signed192.Raw(start.Z)));
            for (int axisIndex = 0;
                axisIndex < boxAxes.Length;
                axisIndex++)
            {
                if (!TryKeepPointSpanAxis(
                        WideAxis3.Cross(boxAxes[axisIndex], edge),
                        boxHalfExtents,
                        relativeBasis,
                        hullTranslation,
                        hullLocalOffsets,
                        ref best))
                {
                    contact = default;
                    return false;
                }
            }
        }

        WideAxis3 orientedAxis = best.Negate ? -best.Axis : best.Axis;
        WideRigidProjection.TransformLocalAxis(
            boxBasis,
            orientedAxis,
            out Signed576 worldX,
            out Signed576 worldY,
            out Signed576 worldZ);
        Vector3d normal = WideNormalization.GetNormalized(
            worldX,
            worldY,
            worldZ);
        Vector3d hullLocalPoint = GetHullSupportLocalPoint(
            relativeBasis,
            hullLocalOffsets,
            orientedAxis,
            maximize: false);

        Vector3d boxLocalPoint = GetLocalSupportPoint(
            normal,
            boxOrientation,
            boxHalfExtents);
        Fixed64 depth = best.GetRoundedDepth(out bool depthIsClamped);
        contact = new FixedContactAnchors(
            new FixedPointAnchor(
                boxCenter,
                boxOrientation,
                boxLocalPoint),
            new FixedPointAnchor(
                hullOrigin,
                hullOrientation,
                hullLocalPoint),
            normal,
            depth,
            depthIsClamped);
        return true;
    }

    private static bool TryKeepPointSpanAxis(
        in WideAxis3 axis,
        Vector3d boxHalfExtents,
        in WideRationalBasis3d relativeBasis,
        in WideAxis3 hullTranslation,
        ReadOnlySpan<Vector3d> hullLocalOffsets,
        ref WidePointSpanPenetration best)
    {
        if (axis.IsZero)
            return true;

        WideRigidProjection.GetLocalAxisProjections(
            axis,
            relativeBasis,
            out Signed576 localAxisX,
            out Signed576 localAxisY,
            out Signed576 localAxisZ);
        Signed576 minimum = WideRigidProjection.GetLocalOffsetProjection(
            hullLocalOffsets[0],
            localAxisX,
            localAxisY,
            localAxisZ);
        Signed576 maximum = minimum;
        for (int index = 1; index < hullLocalOffsets.Length; index++)
        {
            WideRigidProjection.IncludeProjection(
                WideRigidProjection.GetLocalOffsetProjection(
                    hullLocalOffsets[index],
                    localAxisX,
                    localAxisY,
                    localAxisZ),
                ref minimum,
                ref maximum);
        }

        Signed576 originProjection = WideAxis3.Dot(
            axis,
            hullTranslation);
        minimum = WideArithmetic.AddSigned576(originProjection, minimum);
        maximum = WideArithmetic.AddSigned576(originProjection, maximum);
        Signed576 boxRadius = WideArithmetic.MultiplySigned576(
            GetLocalBoxProjectionRadiusNumerator(axis, boxHalfExtents),
            relativeBasis.Denominator);
        Signed576 pushBoxNegative = WideArithmetic.SubtractSigned576(
            boxRadius,
            minimum);
        Signed576 pushBoxPositive = WideArithmetic.AddSigned576(
            maximum,
            boxRadius);
        if (pushBoxNegative.Sign < 0 || pushBoxPositive.Sign < 0)
            return false;

        bool negate =
            CompareSigned(pushBoxPositive, pushBoxNegative) < 0;
        Signed576 overlap = negate
            ? pushBoxPositive
            : pushBoxNegative;
        Signed320 commonDenominator =
            Signed320.ExtendValue(relativeBasis.Denominator);
        Signed576 squaredAxisLength = axis.SquaredLength;
        if (best.ShouldReplace(
            overlap,
            squaredAxisLength,
            commonDenominator))
        {
            best = new WidePointSpanPenetration(
                axis,
                negate,
                overlap,
                squaredAxisLength,
                commonDenominator);
        }
        return true;
    }

}
