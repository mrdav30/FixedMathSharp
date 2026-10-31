//=======================================================================
// WideConvexPrismRelations.cs
//=======================================================================
// MIT License, Copyright (c) 2024-present David Oravsky (mrdav30)
// See LICENSE file in the project root for full license information.
//=======================================================================

using System;

namespace FixedMathSharp.Geometry;





/// <summary>Owns full-domain finite-surface versus convex-prism contact construction.</summary>
/// <content>Dispatches finite shapes and ranks exact prism support candidates.</content>
internal static partial class WideConvexPrismRelations
{
    internal static bool TryGetSphereContact(Vector3d center, Fixed64 radius, Vector3d prismOrigin,
        Fixed64 prismRotation, ReadOnlySpan<Vector2d> offsets, Fixed64 halfThickness,
        PrismPointAdmission admission, out FixedContactAnchors contact, out WideDirection3d axis, out ProjectionDepth depth, out bool overlaps)
    {
        bool found = TryGetContact(center, FixedQuaternion.Identity, Vector3d.Up, Fixed64.Zero, radius,
            prismOrigin, prismRotation, offsets, halfThickness, out contact, admission, out var selected, out overlaps);
        depth = selected.ExactDepth;
        Axis3 direction = selected.Negate ? Negate(selected.Axis) : selected.Axis;
        axis = selected.HasValue ? new WideDirection3d(direction.X, direction.Y, direction.Z) : default;
        return found;
    }

    internal static bool VisitCapsuleContacts(Vector3d center, FixedQuaternion rotation, Fixed64 length, Fixed64 radius,
        Vector3d origin, Fixed64 yaw, ReadOnlySpan<Vector2d> offsets, Fixed64 halfThickness,
        PrismPointAdmission admission, IPrismCapsuleContactDomain domain, out bool overlaps)
    {
        // The ordinary finite owner remains the full-solid overlap oracle;
        // removing a support from response must never remove geometric overlap.
        overlaps = TryGetContact(center, rotation, Vector3d.Up, length, radius, origin, yaw, offsets,
            halfThickness, out _, null, out var selected, out _);
        if (!overlaps) return false;
        WideOrientedBox.GetRotatedLocalAxisNumerators(rotation, Vector3d.Up, out var x, out var y, out var z, out var denominator);
        var shapeAxis = new RigidAxis3(x, y, z, denominator);
        Axis3 direction = selected.Negate ? Negate(selected.Axis) : selected.Axis;
        int axialSign = GetAxisProjection(direction, shapeAxis).Sign;
        FixedQuaternion prismRotation = FixedQuaternion.FromAxisAngle(Vector3d.Up, -yaw);
        FixedPointAnchor unused = default;
        // The unfiltered minimum remains minimal when its exact support has a
        // retained interval. Only a rejected original axis requires another SAT.
        if (!TryGetMatchedPrismAnchor(selected, direction, shapeAxis, center, rotation, Vector3d.Up, length,
                default, axialSign, origin, yaw, prismRotation, offsets, halfThickness,
                ref unused, out _, admission, materialize: false, capsuleDomain: domain))
        {
            if (!TryGetContact(center, rotation, Vector3d.Up, length, radius, origin, yaw, offsets,
                    halfThickness, out _, admission, out selected, out _, domain, false))
                return !PrismContainsPoint(center, origin, yaw, offsets, halfThickness, out _)
                    && TryGetPrismEdgeContact(shapeAxis, center, rotation, length, radius, origin, yaw,
                        prismRotation, offsets, halfThickness, out _, domain);
            direction = selected.Negate ? Negate(selected.Axis) : selected.Axis;
            axialSign = GetAxisProjection(direction, shapeAxis).Sign;
        }
        Vector3d normal = WideNormalization.GetNormalized(Signed576.ExtendValue(direction.X), Signed576.ExtendValue(direction.Y), Signed576.ExtendValue(direction.Z));
        var wideNormal = new WideDirection3d(direction.X, direction.Y, direction.Z);
        Span<Signed832> projected = stackalloc Signed832[6];
        bool prepared = TryGetPrismSupportInterval(selected, direction, shapeAxis, center, length, axialSign, origin, yaw,
            offsets, halfThickness, projected, out Signed832 pointD, out var solidClip);
        System.Diagnostics.Debug.Assert(prepared); // The selected axis already passed this exact finite admission.
        ReadOnlySpan<RationalSegmentInterval> intervals = domain.GetSupportIntervals(wideNormal, projected, pointD, solidClip);
        Signed832 one = Signed832.ExtendValue(Signed192.Signed(1)), two = Signed832.ExtendValue(Signed192.Signed(2));
        bool physical = IsProjectedPrismPointWithinCapsule(projected, pointD, one, two, center - origin, rotation, length, radius);
        System.Diagnostics.Debug.Assert(domain.IncludeRecovery || physical);
        bool freeCore = axialSign == 0 && length > Fixed64.Zero, useExtremes = freeCore && physical;
        FixedPointAnchor baseShape = WideGeometry.GetCenteredCapsuleSupportAnchor(center, rotation, Vector3d.Up, length, Fixed64.Zero, normal, axialSign);
        Vector3d radialOffset = GetPrismCapsuleRadialOffset(direction, rotation, radius);
        Fixed64 depth = GetRoundedDepth(selected.ExactDepth, out bool clamped);
        bool found = false, hasPhysical = false;
        for (int index = 0; index < intervals.Length; index++)
        {
            RationalSegmentInterval retained = intervals[index];
            Signed832 lowerN = retained.IsClipped ? retained.WideLowerNumerator : default, lowerD = retained.IsClipped ? retained.WideLowerDenominator : one;
            Signed832 upperN = retained.IsClipped ? retained.WideUpperNumerator : one, upperD = retained.IsClipped ? retained.WideUpperDenominator : one;
            bool sameEndpoint = useExtremes && RationalSegmentInterval.CompareParameters(lowerN, lowerD, upperN, upperD) == 0;
            Signed832 primaryN = default, primaryD = one; bool primary = false;
            // Retain the preferred point for stable pressure placement and both
            // genuine extrema for angular leverage. Exact parameter deduplication
            // keeps collapsed or endpoint-selected families from adding duplicates.
            for (int sample = 0; sample < (useExtremes ? 3 : 1); sample++)
            {
                Signed832 tN, tD;
                if (sample == 0)
                {
                    primary = TryGetPrismDomainParameter(direction, selected.ExactDepth, projected, pointD, yaw,
                        retained, admission, out primaryN, out primaryD);
                    if (!primary) continue;
                    tN = primaryN; tD = primaryD;
                }
                else
                {
                    tN = sample == 1 ? lowerN : upperN; tD = sample == 1 ? lowerD : upperD;
                    if (sample == 2 && sameEndpoint
                        || primary && RationalSegmentInterval.CompareParameters(tN, tD, primaryN, primaryD) == 0) continue;
                    // Admission also supplies the consumer's exact incident fan;
                    // it must belong to this sample when the callback consumes it.
                    if (!AdmitPrismSpherePoint(direction, selected.ExactDepth, projected, pointD, yaw, admission, tN, tD)) continue;
                }
                FixedPointAnchor shape = freeCore ? GetPrismCoreAnchor(baseShape, Vector3d.Up, length, normal, tN, tD) : baseShape;
                shape = new FixedPointAnchor(center, rotation, shape.LocalPoint, radialOffset, shape.ExactLocalTerm);
                FixedPointAnchor target = MaterializePrismProjection(projected, pointD, tN, tD, origin, prismRotation);
                var native = new CapsulePrismContactDepth(selected.ExactDepth);
                domain.AddSupportContact(new(shape, target, normal, depth, clamped), native, wideNormal, selected.ExactDepth, physical, index);
                found = true; hasPhysical |= physical;
            }
        }
        // Match the ordinary finite owner: a physical supporting face already
        // supplies the convex contact. Recovery-only/empty faces still need a real edge.
        // A core point inside the prism rules out every nonzero separating
        // closest-edge pair, independently of which surfaces admit response.
        if (!hasPhysical && !PrismContainsPoint(center, origin, yaw, offsets, halfThickness, out _))
            found |= TryGetPrismEdgeContact(shapeAxis, center, rotation, length, radius, origin, yaw,
                prismRotation, offsets, halfThickness, out _, domain);
        return found;
    }

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
        TryGetContact(
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
        CurvedPrismContact.TryGetContact(
            center,
            rotation,
            localAxisDirection,
            axisLength,
            radius,
            prismOrigin,
            prismRotation,
            prismLocalOffsets,
            prismHalfThickness,
            false,
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
        CurvedPrismContact.TryGetContact(
            center,
            rotation,
            localAxisDirection,
            height,
            radius,
            prismOrigin,
            prismRotation,
            prismLocalOffsets,
            prismHalfThickness,
            true,
            out contact);

    private static bool TryGetContact(
        Vector3d center,
        FixedQuaternion shapeRotation,
        Vector3d localAxisDirection,
        Fixed64 axisLength,
        Fixed64 radius,
        Vector3d prismOrigin,
        Fixed64 prismRotation,
        ReadOnlySpan<Vector2d> prismLocalOffsets,
        Fixed64 prismHalfThickness,
        out FixedContactAnchors contact) =>
        TryGetContact(center, shapeRotation, localAxisDirection, axisLength, radius, prismOrigin,
            prismRotation, prismLocalOffsets, prismHalfThickness, out contact, null, out _, out _);

    private static bool TryGetContact(Vector3d center, FixedQuaternion shapeRotation, Vector3d localAxisDirection,
        Fixed64 axisLength, Fixed64 radius, Vector3d prismOrigin, Fixed64 prismRotation,
        ReadOnlySpan<Vector2d> prismLocalOffsets, Fixed64 prismHalfThickness, out FixedContactAnchors contact,
        PrismPointAdmission? admission, out PenetrationCandidate best, out bool overlaps,
        IPrismCapsuleContactDomain? capsuleDomain = null, bool materialize = true)
    {
        best = default;
        overlaps = false;
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
                center,
                axisLength,
                radius,
                prismOrigin,
                prismRotation,
                prismLocalOffsets,
                prismHalfThickness,
                ref best, admission, capsuleDomain, shapeRotation)
            || !TryKeepAxis(
                shapeAxis,
                rigidShapeAxis,
                center,
                axisLength,
                radius,
                prismOrigin,
                prismRotation,
                prismLocalOffsets,
                prismHalfThickness,
                ref best, admission, capsuleDomain, shapeRotation)
            || !TryKeepAxis(
                Cross(shapeAxis, up),
                rigidShapeAxis,
                center,
                axisLength,
                radius,
                prismOrigin,
                prismRotation,
                prismLocalOffsets,
                prismHalfThickness,
                ref best, admission, capsuleDomain, shapeRotation))
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
            if (!TryKeepAxis(
                face,
                rigidShapeAxis,
                center,
                axisLength,
                radius,
                prismOrigin,
                prismRotation,
                prismLocalOffsets,
                prismHalfThickness,
                ref best, admission, capsuleDomain, shapeRotation)
                || !TryKeepAxis(
                    Cross(shapeAxis, edge),
                    rigidShapeAxis,
                    center,
                    axisLength,
                    radius,
                    prismOrigin,
                    prismRotation,
                    prismLocalOffsets,
                    prismHalfThickness,
                    ref best, admission, capsuleDomain, shapeRotation)
                || !TryKeepAxis(
                    planarVertexAxis,
                    rigidShapeAxis,
                    center,
                    axisLength,
                    radius,
                    prismOrigin,
                    prismRotation,
                    prismLocalOffsets,
                    prismHalfThickness,
                    ref best, admission, capsuleDomain, shapeRotation))
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
                            center,
                            axisLength,
                            radius,
                            prismOrigin,
                            prismRotation,
                            prismLocalOffsets,
                            prismHalfThickness,
                            ref best, admission, capsuleDomain, shapeRotation))
                    {
                        contact = default;
                        return false;
                    }
                }
            }
        }

        // A point footprint and parallel vertical core have no edge/cross
        // normal. Append a perpendicular bound without replacing earlier ties.
        if (WideConvex2dRelations.GetPlanarSweepWinding(prismLocalOffsets) == 0
            && !TryKeepAxis(FromDirection(Vector3d.Right), rigidShapeAxis, center, axisLength, radius,
                prismOrigin, prismRotation, prismLocalOffsets, prismHalfThickness, ref best, admission, capsuleDomain, shapeRotation))
        {
            contact = default;
            return false;
        }

        if (admission != null && capsuleDomain == null && !KeepSphereEdgeAxes(center, radius, prismOrigin, prismRotation, prismLocalOffsets,
                prismHalfThickness, rigidShapeAxis, admission, ref best))
        { contact = default; return false; }
        overlaps = true;
        if (!best.HasValue) { contact = default; return false; }
        if (!materialize) { contact = default; return true; }
        return TryMaterializeContact(
            best,
            rigidShapeAxis,
            center,
            shapeRotation,
            localAxisDirection,
            axisLength,
            radius,
            prismOrigin,
            prismRotation,
            prismOrientation,
            prismLocalOffsets,
            prismHalfThickness,
            out contact);
    }

    private static bool TryKeepAxis(
        Axis3 axis,
        RigidAxis3 shapeAxis,
        Vector3d center,
        Fixed64 axisLength,
        Fixed64 radius,
        Vector3d prismOrigin,
        Fixed64 prismRotation,
        ReadOnlySpan<Vector2d> prismLocalOffsets,
        Fixed64 prismHalfThickness,
        ref PenetrationCandidate best, PrismPointAdmission? admission, IPrismCapsuleContactDomain? capsuleDomain,
        in FixedQuaternion shapeRotation)
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
            out Vector3d maximumPrismOffset);
        GetCapsuleProjectionDepths(
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

        if (admission != null)
        {
            // Solid admission above still checks both sides. Exposure ranks both
            // oriented exits independently, including a farther genuine boundary.
            KeepSphereProjection(axis, false, minimumPrismOffset, positiveDepth, shapeAxis, center,
                prismOrigin, prismRotation, prismLocalOffsets, prismHalfThickness, admission, ref best, axisLength, capsuleDomain, shapeRotation);
            KeepSphereProjection(axis, true, maximumPrismOffset, negativeDepth, shapeAxis, center,
                prismOrigin, prismRotation, prismLocalOffsets, prismHalfThickness, admission, ref best, axisLength, capsuleDomain, shapeRotation);
            return true;
        }

        bool usePositiveNormal;
        ProjectionDepth selectedDepth;
        if (prismMinimum.Sign >= 0)
        {
            usePositiveNormal = true;
            selectedDepth = positiveDepth;
        }
        else if (prismMaximum.Sign <= 0)
        {
            usePositiveNormal = false;
            selectedDepth = negativeDepth;
        }
        else if (CompareProjectionDepths(
                     positiveDepth,
                     negativeDepth) >= 0)
        {
            usePositiveNormal = false;
            selectedDepth = negativeDepth;
        }
        else
        {
            usePositiveNormal = true;
            selectedDepth = positiveDepth;
        }

        if (best.HasValue
            && CompareProjectionDepths(
                selectedDepth,
                best.ExactDepth) >= 0)
        {
            return true;
        }

        // +axis pairs the capsule maximum with the prism minimum; -axis
        // pairs its minimum with the prism maximum, including containment.
        Vector3d prismOffset = usePositiveNormal
            ? minimumPrismOffset
            : maximumPrismOffset;
        best = new PenetrationCandidate(
            axis,
            !usePositiveNormal,
            prismOffset,
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
        out Vector3d maximumOffset)
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
            }

            int maximumComparison = Compare(projection, maximumPlanar);
            if (maximumComparison > 0)
            {
                maximumPlanar = projection;
                maximumIndex = index;
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
