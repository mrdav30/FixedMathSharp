//=======================================================================
// WideConvexPrismRelations.cs
//=======================================================================
// MIT License, Copyright (c) 2024–present David Oravsky (mrdav30)
// See LICENSE file in the project root for full license information.
//=======================================================================

using System;

namespace FixedMathSharp.Bounds;

/// <summary>
/// Owns full-domain finite-surface versus convex-prism contact construction.
/// </summary>
internal static partial class WideConvexPrismRelations
{
    internal static bool TryGetCenteredCapsuleContact(
        Vector3d center,
        FixedQuaternion rotation,
        Vector3d localAxisDirection,
        Fixed64 axisLength,
        Fixed64 radius,
        Vector3d prismOrigin,
        Fixed64 prismRotation,
        ReadOnlySpan<Vector2d> prismLocalOffsets,
        Fixed64 prismHalfThickness,
        out FixedContactAnchors contact) =>
        TryGetAnchoredContact(
            FiniteShapeKind.Capsule,
            center,
            rotation,
            localAxisDirection,
            axisLength,
            radius,
            prismOrigin,
            prismRotation,
            prismLocalOffsets,
            prismHalfThickness,
            out contact);

    internal static bool TryGetCenteredCylinderContact(
        Vector3d center,
        FixedQuaternion rotation,
        Vector3d localAxisDirection,
        Fixed64 axisLength,
        Fixed64 radius,
        Vector3d prismOrigin,
        Fixed64 prismRotation,
        ReadOnlySpan<Vector2d> prismLocalOffsets,
        Fixed64 prismHalfThickness,
        out FixedContactAnchors contact) =>
        TryGetAnchoredContact(
            FiniteShapeKind.Cylinder,
            center,
            rotation,
            localAxisDirection,
            axisLength,
            radius,
            prismOrigin,
            prismRotation,
            prismLocalOffsets,
            prismHalfThickness,
            out contact);

    internal static bool TryGetCenteredConeContact(
        Vector3d center,
        FixedQuaternion rotation,
        Vector3d localAxisDirection,
        Fixed64 height,
        Fixed64 radius,
        Vector3d prismOrigin,
        Fixed64 prismRotation,
        ReadOnlySpan<Vector2d> prismLocalOffsets,
        Fixed64 prismHalfThickness,
        out FixedContactAnchors contact) =>
        TryGetAnchoredContact(
            FiniteShapeKind.Cone,
            center,
            rotation,
            localAxisDirection,
            height,
            radius,
            prismOrigin,
            prismRotation,
            prismLocalOffsets,
            prismHalfThickness,
            out contact);

    private static bool TryGetAnchoredContact(
        FiniteShapeKind shapeKind,
        Vector3d center,
        FixedQuaternion shapeRotation,
        Vector3d localAxisDirection,
        Fixed64 axisLength,
        Fixed64 radius,
        Vector3d prismOrigin,
        Fixed64 prismRotation,
        ReadOnlySpan<Vector2d> prismLocalOffsets,
        Fixed64 prismHalfThickness,
        out FixedContactAnchors contact)
    {
        if (!TryGetContact(
                shapeKind,
                center,
                shapeRotation,
                localAxisDirection,
                axisLength,
                radius,
                prismOrigin,
                prismRotation,
                prismLocalOffsets,
                prismHalfThickness,
                out Penetration penetration))
        {
            contact = default;
            return false;
        }

        contact = new FixedContactAnchors(
            penetration.ShapeAnchor,
            new FixedPointAnchor(
                prismOrigin,
                penetration.PrismOrientation,
                penetration.PrismOffset),
            penetration.Normal,
            penetration.Depth,
            penetration.DepthIsClamped);
        return true;
    }

    private static bool TryGetContact(
        FiniteShapeKind shapeKind,
        Vector3d center,
        FixedQuaternion shapeRotation,
        Vector3d localAxisDirection,
        Fixed64 axisLength,
        Fixed64 radius,
        Vector3d prismOrigin,
        Fixed64 prismRotation,
        ReadOnlySpan<Vector2d> prismLocalOffsets,
        Fixed64 prismHalfThickness,
        out Penetration contact)
    {
        var best = default(PenetrationCandidate);
        WideOrientedBox.GetRotatedLocalAxisNumerators(
            shapeRotation,
            localAxisDirection,
            out Signed192 shapeAxisX,
            out Signed192 shapeAxisY,
            out Signed192 shapeAxisZ,
            out Signed192 shapeRotationDenominator);
        var rigidShapeAxis = new RigidAxis3(
            shapeAxisX,
            shapeAxisY,
            shapeAxisZ,
            shapeRotationDenominator);
        Axis3 shapeAxis = rigidShapeAxis.ToWide();
        Axis3 up = FromDirection(Vector3d.Up);
        FixedQuaternion prismOrientation =
            FixedQuaternion.FromAxisAngle(
                Vector3d.Up,
                -prismRotation);
        if (!TryKeepAxis(
                up,
                rigidShapeAxis,
                shapeKind,
                center,
                axisLength,
                radius,
                shapeRotation,
                localAxisDirection,
                prismOrigin,
                prismRotation,
                prismOrientation,
                prismLocalOffsets,
                prismHalfThickness,
                ref best)
            || !TryKeepAxis(
                shapeAxis,
                rigidShapeAxis,
                shapeKind,
                center,
                axisLength,
                radius,
                shapeRotation,
                localAxisDirection,
                prismOrigin,
                prismRotation,
                prismOrientation,
                prismLocalOffsets,
                prismHalfThickness,
                ref best)
            || !TryKeepAxis(
                Cross(shapeAxis, up),
                rigidShapeAxis,
                shapeKind,
                center,
                axisLength,
                radius,
                shapeRotation,
                localAxisDirection,
                prismOrigin,
                prismRotation,
                prismOrientation,
                prismLocalOffsets,
                prismHalfThickness,
                ref best))
        {
            contact = default;
            return false;
        }

        for (int index = 0; index < prismLocalOffsets.Length; index++)
        {
            Vector2d start = prismLocalOffsets[index];
            Vector2d end = prismLocalOffsets[
                index + 1 == prismLocalOffsets.Length ? 0 : index + 1];
            Axis3 edge = GetPlanarEdge(
                prismRotation,
                start,
                end);
            Axis3 face = new(
                edge.Z,
                default,
                WideArithmetic.Negate(edge.X));
            Axis3 planarVertexAxis = GetPlanarVertexAxis(
                center,
                prismOrigin,
                prismRotation,
                start);
            // Incident vertex axes reject every face-separated convex input;
            // the face remains necessary only as an overlap-depth candidate.
            _ = TryKeepAxis(
                face,
                rigidShapeAxis,
                shapeKind,
                center,
                axisLength,
                radius,
                shapeRotation,
                localAxisDirection,
                prismOrigin,
                prismRotation,
                prismOrientation,
                prismLocalOffsets,
                prismHalfThickness,
                ref best);
            if (!TryKeepAxis(
                    Cross(shapeAxis, edge),
                    rigidShapeAxis,
                    shapeKind,
                    center,
                    axisLength,
                    radius,
                    shapeRotation,
                    localAxisDirection,
                    prismOrigin,
                    prismRotation,
                    prismOrientation,
                    prismLocalOffsets,
                    prismHalfThickness,
                    ref best)
                || !TryKeepAxis(
                    planarVertexAxis,
                    rigidShapeAxis,
                    shapeKind,
                    center,
                    axisLength,
                    radius,
                    shapeRotation,
                    localAxisDirection,
                    prismOrigin,
                    prismRotation,
                    prismOrientation,
                    prismLocalOffsets,
                    prismHalfThickness,
                    ref best))
            {
                contact = default;
                return false;
            }

            for (int verticalSign = -1;
                verticalSign <= 1;
                verticalSign += 2)
            {
                for (int endpointSign = -1;
                    endpointSign <= 1;
                    endpointSign += 2)
                {
                    Axis3 endpointAxis = GetEndpointToPrismVertexAxis(
                        center,
                        rigidShapeAxis,
                        axisLength,
                        endpointSign,
                        prismOrigin,
                        prismRotation,
                        start,
                        prismHalfThickness,
                        verticalSign);
                    if (!TryKeepAxis(
                            endpointAxis,
                            rigidShapeAxis,
                            shapeKind,
                            center,
                            axisLength,
                            radius,
                            shapeRotation,
                            localAxisDirection,
                            prismOrigin,
                            prismRotation,
                            prismOrientation,
                            prismLocalOffsets,
                            prismHalfThickness,
                            ref best))
                    {
                        contact = default;
                        return false;
                    }
                }
            }
        }

        contact = MaterializeContact(
            best,
            shapeKind,
            rigidShapeAxis,
            center,
            shapeRotation,
            localAxisDirection,
            axisLength,
            radius,
            prismOrigin,
            prismOrientation,
            prismLocalOffsets,
            prismHalfThickness);
        return true;
    }

    private static bool TryKeepAxis(
        Axis3 axis,
        RigidAxis3 shapeAxis,
        FiniteShapeKind shapeKind,
        Vector3d center,
        Fixed64 axisLength,
        Fixed64 radius,
        FixedQuaternion shapeRotation,
        Vector3d localAxisDirection,
        Vector3d prismOrigin,
        Fixed64 prismRotation,
        FixedQuaternion prismOrientation,
        ReadOnlySpan<Vector2d> prismLocalOffsets,
        Fixed64 prismHalfThickness,
        ref PenetrationCandidate best)
    {
        if (axis.IsZero)
            return true;

        GetPrismProjection(
            center,
            prismOrigin,
            prismRotation,
            prismLocalOffsets,
            prismHalfThickness,
            axis,
            out Signed576 prismMinimum,
            out Signed576 prismMaximum,
            out Vector3d minimumPrismOffset,
            out Vector3d maximumPrismOffset,
            out bool minimumPlanarTie,
            out bool maximumPlanarTie);
        GetShapeProjectionDepths(
            shapeKind,
            axis,
            shapeAxis,
            axisLength,
            radius,
            prismMinimum,
            prismMaximum,
            out ProjectionDepth positiveDepth,
            out ProjectionDepth negativeDepth);
        if (!IsProjectionNonNegative(positiveDepth)
            || !IsProjectionNonNegative(negativeDepth))
        {
            return false;
        }

        bool usePositiveNormal;
        bool useMinimumPrismOffset;
        ProjectionDepth selectedDepth;
        ShapeSupportFeature supportFeature;
        bool planarTie;
        if (prismMinimum.Sign >= 0)
        {
            usePositiveNormal = true;
            useMinimumPrismOffset = true;
            selectedDepth = positiveDepth;
            supportFeature = positiveDepth.SupportFeature;
            planarTie = minimumPlanarTie;
        }
        else if (prismMaximum.Sign <= 0)
        {
            usePositiveNormal = false;
            useMinimumPrismOffset = false;
            selectedDepth = negativeDepth;
            supportFeature = negativeDepth.SupportFeature;
            planarTie = maximumPlanarTie;
        }
        else if (CompareProjectionDepths(
                     positiveDepth,
                     negativeDepth) >= 0)
        {
            usePositiveNormal = true;
            useMinimumPrismOffset = false;
            selectedDepth = negativeDepth;
            supportFeature = negativeDepth.SupportFeature;
            planarTie = maximumPlanarTie;
        }
        else
        {
            usePositiveNormal = false;
            useMinimumPrismOffset = true;
            selectedDepth = positiveDepth;
            supportFeature = positiveDepth.SupportFeature;
            planarTie = minimumPlanarTie;
        }

        if (best.HasValue
            && CompareProjectionDepths(
                selectedDepth,
                best.ExactDepth) >= 0)
        {
            return true;
        }

        Vector3d prismOffset = useMinimumPrismOffset
            ? minimumPrismOffset
            : maximumPrismOffset;
        best = new PenetrationCandidate(
            axis,
            !usePositiveNormal,
            supportFeature,
            prismOffset,
            planarTie,
            selectedDepth);
        return true;
    }

    private static void GetPrismProjection(
        Vector3d sourceOrigin,
        Vector3d prismOrigin,
        Fixed64 prismRotation,
        ReadOnlySpan<Vector2d> offsets,
        Fixed64 halfThickness,
        Axis3 axis,
        out Signed576 minimum,
        out Signed576 maximum,
        out Vector3d minimumOffset,
        out Vector3d maximumOffset,
        out bool minimumPlanarTie,
        out bool maximumPlanarTie)
    {
        Signed576 originProjection = ScaleProjection(
            GetDifferenceProjection(
                prismOrigin,
                sourceOrigin,
                axis));
        Fixed64 minimumY = axis.Y.Sign >= 0
            ? -halfThickness
            : halfThickness;
        Fixed64 maximumY = -minimumY;
        int minimumIndex = 0;
        int maximumIndex = 0;
        Signed576 minimumPlanar = GetRotatedPlanarProjection(
            offsets[0],
            prismRotation,
            axis);
        Signed576 maximumPlanar = minimumPlanar;
        minimumPlanarTie = false;
        maximumPlanarTie = false;
        for (int index = 1; index < offsets.Length; index++)
        {
            Signed576 projection = GetRotatedPlanarProjection(
                offsets[index],
                prismRotation,
                axis);
            int minimumComparison = Compare(projection, minimumPlanar);
            if (minimumComparison < 0)
            {
                minimumPlanar = projection;
                minimumIndex = index;
                minimumPlanarTie = false;
            }
            else if (minimumComparison == 0
                && offsets[index] != offsets[minimumIndex])
            {
                minimumPlanarTie = true;
            }

            int maximumComparison = Compare(projection, maximumPlanar);
            if (maximumComparison > 0)
            {
                maximumPlanar = projection;
                maximumIndex = index;
                maximumPlanarTie = false;
            }
            else if (maximumComparison == 0
                && offsets[index] != offsets[maximumIndex])
            {
                maximumPlanarTie = true;
            }
        }

        Signed576 verticalRadius = ScaleProjection(
            WideArithmetic.MultiplySigned320(
                axis.Y.Sign < 0 ? WideArithmetic.Negate(axis.Y) : axis.Y,
                Signed320.ExtendValue(Signed192.Raw(halfThickness))));
        minimum = WideArithmetic.SubtractSigned576(
            WideArithmetic.AddSigned576(originProjection, minimumPlanar),
            verticalRadius);
        maximum = WideArithmetic.AddSigned576(
            WideArithmetic.AddSigned576(originProjection, maximumPlanar),
            verticalRadius);
        Vector2d minimumPlanarOffset = offsets[minimumIndex];
        Vector2d maximumPlanarOffset = offsets[maximumIndex];
        minimumOffset = new Vector3d(
            minimumPlanarOffset.X,
            minimumY,
            minimumPlanarOffset.Y);
        maximumOffset = new Vector3d(
            maximumPlanarOffset.X,
            maximumY,
            maximumPlanarOffset.Y);
    }

    private static Axis3 GetEndpointToPrismVertexAxis(
        Vector3d sourceCenter,
        RigidAxis3 sourceAxis,
        Fixed64 sourceLength,
        int endpointSign,
        Vector3d prismOrigin,
        Fixed64 prismRotation,
        Vector2d prismOffset,
        Fixed64 prismHalfThickness,
        int verticalSign)
    {
        WideConvex2dRelations.GetRelativePointNumerators(
            new Vector2d(prismOrigin.X, prismOrigin.Z),
            prismRotation,
            prismOffset,
            new Vector2d(sourceCenter.X, sourceCenter.Z),
            out Signed192 relativeX,
            out Signed192 relativeZ);
        Signed192 relativeY = Signed192.NarrowProven(
            WideArithmetic.MultiplySigned192(
                WideArithmetic.AddSigned192(
                    WideArithmetic.SubtractSigned192(
                        Signed192.Raw(prismOrigin.Y),
                        Signed192.Raw(sourceCenter.Y)),
                    Signed192.Raw(verticalSign * prismHalfThickness)),
                Scale));
        return new Axis3(
            GetScaledEndpointDifference(
                relativeX,
                sourceAxis.X,
                sourceAxis.RotationDenominator,
                sourceLength,
                endpointSign),
            GetScaledEndpointDifference(
                relativeY,
                sourceAxis.Y,
                sourceAxis.RotationDenominator,
                sourceLength,
                endpointSign),
            GetScaledEndpointDifference(
                relativeZ,
                sourceAxis.Z,
                sourceAxis.RotationDenominator,
                sourceLength,
                endpointSign));
    }

    private static Signed320 GetScaledEndpointDifference(
        Signed192 relativeNumerator,
        Signed192 sourceAxisNumerator,
        Signed192 sourceRotationDenominator,
        Fixed64 sourceLength,
        int endpointSign)
    {
        Signed320 scaledRelative = WideArithmetic.MultiplySigned192(
            relativeNumerator,
            sourceRotationDenominator);
        return
        WideArithmetic.SubtractSigned320(
            WideArithmetic.AddSigned320(
                scaledRelative,
                scaledRelative),
            WideArithmetic.MultiplySigned192(
                sourceAxisNumerator,
                Signed192.Signed(
                    endpointSign * sourceLength.m_rawValue)));
    }

    private static Axis3 GetPlanarEdge(
        Fixed64 rotation,
        Vector2d start,
        Vector2d end)
    {
        WideConvex2dRelations.GetRelativePointNumerators(
            Vector2d.Zero,
            rotation,
            end,
            Vector2d.Zero,
            out Signed192 endX,
            out Signed192 endY);
        WideConvex2dRelations.GetRelativePointNumerators(
            Vector2d.Zero,
            rotation,
            start,
            Vector2d.Zero,
            out Signed192 startX,
            out Signed192 startY);
        return new Axis3(
            Signed320.ExtendValue(
                WideArithmetic.SubtractSigned192(endX, startX)),
            default,
            Signed320.ExtendValue(
                WideArithmetic.SubtractSigned192(endY, startY)));
    }

    private static Axis3 GetPlanarVertexAxis(
        Vector3d sourceCenter,
        Vector3d prismOrigin,
        Fixed64 prismRotation,
        Vector2d prismOffset)
    {
        WideConvex2dRelations.GetRelativePointNumerators(
            new Vector2d(prismOrigin.X, prismOrigin.Z),
            prismRotation,
            prismOffset,
            new Vector2d(sourceCenter.X, sourceCenter.Z),
            out Signed192 x,
            out Signed192 z);
        return new Axis3(
            Signed320.ExtendValue(x),
            default,
            Signed320.ExtendValue(z));
    }

    private static Axis3 Cross(Axis3 left, Axis3 right) =>
        new(
            SubtractProducts(left.Y, right.Z, left.Z, right.Y),
            SubtractProducts(left.Z, right.X, left.X, right.Z),
            SubtractProducts(left.X, right.Y, left.Y, right.X));

    private static Signed320 SubtractProducts(
        Signed320 firstLeft,
        Signed320 firstRight,
        Signed320 secondLeft,
        Signed320 secondRight)
    {
        Signed576 value = WideArithmetic.SubtractSigned576(
            WideArithmetic.MultiplySigned320(firstLeft, firstRight),
            WideArithmetic.MultiplySigned320(secondLeft, secondRight));
        _ = Signed320.TryNarrowSigned(value, out Signed320 result);
        return result;
    }

    private static Signed576 GetDifferenceProjection(
        Vector3d end,
        Vector3d start,
        Axis3 axis) =>
        WideArithmetic.AddSigned576(
            WideArithmetic.AddSigned576(
                WideArithmetic.MultiplySigned320(
                    Signed320.ExtendValue(
                        WideArithmetic.SubtractSigned192(
                            Signed192.Raw(end.X),
                            Signed192.Raw(start.X))),
                    axis.X),
                WideArithmetic.MultiplySigned320(
                    Signed320.ExtendValue(
                        WideArithmetic.SubtractSigned192(
                            Signed192.Raw(end.Y),
                            Signed192.Raw(start.Y))),
                    axis.Y)),
            WideArithmetic.MultiplySigned320(
                Signed320.ExtendValue(
                    WideArithmetic.SubtractSigned192(
                        Signed192.Raw(end.Z),
                        Signed192.Raw(start.Z))),
                axis.Z));

    private static Signed576 GetRotatedPlanarProjection(
        Vector2d offset,
        Fixed64 rotation,
        Axis3 axis)
    {
        WideConvex2dRelations.GetRelativePointNumerators(
            Vector2d.Zero,
            rotation,
            offset,
            Vector2d.Zero,
            out Signed192 x,
            out Signed192 y);
        return WideArithmetic.AddSigned576(
            WideArithmetic.MultiplySigned320(
                Signed320.ExtendValue(x),
                axis.X),
            WideArithmetic.MultiplySigned320(
                Signed320.ExtendValue(y),
                axis.Z));
    }

    private static Signed576 ScaleProjection(Signed576 projection) =>
        WideArithmetic.MultiplySigned576(projection, Scale);
}
