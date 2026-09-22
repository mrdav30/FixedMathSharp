//=======================================================================
// WideCenteredCapsule2dRelations.cs
//=======================================================================
// MIT License, Copyright (c) 2024–present David Oravsky (mrdav30)
// See LICENSE file in the project root for full license information.
//=======================================================================

using System;
using RotationFrame2d = FixedMathSharp.Geometry.WideConvex2dRelations.RotationFrame2d;

namespace FixedMathSharp.Geometry;

/// <summary>
/// Owns rigid-frame full-domain relations for conceptual 2D capsules.
/// </summary>
internal static class WideCenteredCapsule2dRelations
{
    // Full-domain bounds (S = 2^32): relative point numerators are below 2^97,
    // exact axis components below 2^99, and signed overlaps below 2^199.
    // Squared-overlap/axis products stay below 2^597, fitting Signed832.
    // No world endpoint, unit axis, or radial square root is narrowed to Fixed64
    // until classification and minimum-depth ordering are complete.
    private static readonly Fixed64 SideAlignmentTolerance =
        Fixed64.Epsilon * (Fixed64)16;

    private readonly struct ContactAxis
    {
        internal readonly Signed192 X;
        internal readonly Signed192 Y;
        internal readonly Signed320 SquaredLength;
        internal readonly Signed320 Overlap;
        internal readonly bool FromEdge;

        internal bool HasValue => !SquaredLength.IsZero;

        internal ContactAxis(
            Signed192 x,
            Signed192 y,
            Signed320 squaredLength,
            Signed320 overlap,
            bool fromEdge)
        {
            X = x;
            Y = y;
            SquaredLength = squaredLength;
            Overlap = overlap;
            FromEdge = fromEdge;
        }
    }

    internal static bool TryGetMinimumTranslation(
        Vector2d center,
        Vector2d capsuleAxis,
        Fixed64 axisLength,
        Fixed64 radius,
        Vector2d convexOrigin,
        ReadOnlySpan<Vector2d> convexOriginOffsets,
        out Vector2d normal,
        out Fixed64 depth) =>
        TryGetMinimumTranslation(
            center,
            capsuleAxis,
            axisLength,
            radius,
            convexOrigin,
            Fixed64.Zero,
            convexOriginOffsets,
            out normal,
            out depth);

    internal static bool TryGetMinimumTranslation(
        Vector2d center,
        Vector2d capsuleAxis,
        Fixed64 axisLength,
        Fixed64 radius,
        Vector2d convexOrigin,
        Fixed64 convexRotation,
        ReadOnlySpan<Vector2d> convexOriginOffsets,
        out Vector2d normal,
        out Fixed64 depth) =>
        TryGetMinimumTranslation(
            center,
            capsuleAxis,
            axisLength,
            radius,
            convexOrigin,
            convexRotation,
            convexOriginOffsets,
            out normal,
            out depth,
            out _,
            out _);

    internal static bool TryGetContacts(
        Vector2d capsuleCenter,
        Fixed64 capsuleRotation,
        Vector2d localCapsuleAxis,
        Fixed64 axisLength,
        Fixed64 radius,
        Vector2d convexOrigin,
        Fixed64 convexRotation,
        ReadOnlySpan<Vector2d> convexVertexOffsets,
        Span<FixedPointAnchor2d> capsuleContacts,
        Span<FixedPointAnchor2d> convexContacts,
        out int contactCount,
        out Vector2d normal,
        out Fixed64 depth,
        out bool depthIsClamped)
    {
        _ = Vector2d.TryRotate(
            localCapsuleAxis,
            capsuleRotation,
            out Vector2d capsuleAxis);
        if (!TryGetMinimumTranslation(
                capsuleCenter,
                capsuleAxis,
                axisLength,
                radius,
                convexOrigin,
                convexRotation,
                convexVertexOffsets,
                out normal,
                out depth,
                out depthIsClamped,
                out ContactAxis contactAxis))
        {
            contactCount = default;
            return false;
        }

        _ = Vector2d.TryRotate(
            normal,
            -capsuleRotation,
            out Vector2d localNormal);
        localNormal = localNormal.Normalized;
        GetSupportFeature(
            capsuleCenter,
            convexOrigin,
            new RotationFrame2d(convexRotation),
            convexVertexOffsets,
            contactAxis,
            out Vector2d featureStart,
            out Vector2d featureEnd);
        Vector2d featureDirection = WideNormalization.GetDirection(featureStart, featureEnd);
        if (axisLength > Fixed64.Epsilon
            && featureDirection != Vector2d.Zero
            && Vector2d.Dot(capsuleAxis, normal).Abs()
                <= SideAlignmentTolerance)
        {
            FixedPointAnchor2d firstCapsuleAnchor =
                GetCenteredCapsuleSideAnchor(
                    capsuleCenter,
                    capsuleRotation,
                    localCapsuleAxis,
                    -axisLength,
                    localNormal,
                    radius);
            FixedPointAnchor2d secondCapsuleAnchor =
                GetCenteredCapsuleSideAnchor(
                    capsuleCenter,
                    capsuleRotation,
                    localCapsuleAxis,
                    axisLength,
                    localNormal,
                    radius);
            if (WideConvex2dRelations.TryProjectAnchorOntoFeature(
                    firstCapsuleAnchor,
                    convexOrigin,
                    convexRotation,
                    featureStart,
                    featureEnd,
                    requireInteriorProjection: true,
                    out Vector2d firstConvexOffset)
                && WideConvex2dRelations.TryProjectAnchorOntoFeature(
                    secondCapsuleAnchor,
                    convexOrigin,
                    convexRotation,
                    featureStart,
                    featureEnd,
                    requireInteriorProjection: true,
                    out Vector2d secondConvexOffset))
            {
                capsuleContacts[0] = firstCapsuleAnchor;
                capsuleContacts[1] = secondCapsuleAnchor;
                convexContacts[0] = new FixedPointAnchor2d(
                    convexOrigin,
                    convexRotation,
                    firstConvexOffset);
                convexContacts[1] = new FixedPointAnchor2d(
                    convexOrigin,
                    convexRotation,
                    secondConvexOffset);
                contactCount = 2;
                return true;
            }
        }

        FixedPointAnchor2d capsuleContact =
            WideFiniteAxisIntersection.GetCenteredCapsuleSupportAnchor(
                capsuleCenter,
                capsuleRotation,
                localCapsuleAxis,
                axisLength,
                radius,
                localNormal);
        _ = WideConvex2dRelations.TryProjectAnchorOntoFeature(
            capsuleContact,
            convexOrigin,
            convexRotation,
            featureStart,
            featureEnd,
            requireInteriorProjection: false,
            out Vector2d convexOffset);

        // A side support is an entire segment, not its arbitrary midpoint.
        // Resolve its axial witness from the opposing feature using the exact
        // selected axis; a rounded normal can turn a side tie into an end cap.
        if (axisLength > Fixed64.Zero
            && WideArithmetic.AddSigned320(
                WideArithmetic.MultiplySigned192(contactAxis.X, Signed192.Raw(capsuleAxis.X)),
                WideArithmetic.MultiplySigned192(contactAxis.Y, Signed192.Raw(capsuleAxis.Y))).IsZero)
        {
            capsuleContact = GetProjectedSideAnchor(
                capsuleCenter, capsuleRotation, localCapsuleAxis, axisLength,
                localNormal, radius, convexOrigin, convexRotation, convexOffset);
        }

        capsuleContacts[0] = capsuleContact;
        convexContacts[0] = new FixedPointAnchor2d(
            convexOrigin,
            convexRotation,
            convexOffset);
        contactCount = 1;
        return true;
    }

    private static FixedPointAnchor2d GetProjectedSideAnchor(
        Vector2d center,
        Fixed64 rotation,
        Vector2d localAxis,
        Fixed64 axisLength,
        Vector2d localNormal,
        Fixed64 radius,
        Vector2d pointOrigin,
        Fixed64 pointRotation,
        Vector2d pointOffset)
    {
        RotationFrame2d frame = new(rotation);
        WideConvex2dRelations.GetRotatedOffset(frame, localAxis,
            out Signed192 axisX, out Signed192 axisY);
        WideConvex2dRelations.GetRotatedOffset(frame, localNormal,
            out Signed192 radialX, out Signed192 radialY);
        GetRelativePointNumerators(pointOrigin, new RotationFrame2d(pointRotation),
            pointOffset, center, out Signed192 pointX, out Signed192 pointY);

        // Axis and point numerators have denominator S^2. Subtract the radial
        // support at denominator S^3 before projecting onto the conceptual side.
        // Twice the local axial distance in raw units is 2*dot(delta,axis)/|axis|^2.
        Signed192 deltaX = Signed192.NarrowValue(WideArithmetic.SubtractSigned320(
            WideArithmetic.MultiplySigned192(pointX, Signed192.One),
            WideArithmetic.MultiplySigned192(radialX, Signed192.Raw(radius))));
        Signed192 deltaY = Signed192.NarrowValue(WideArithmetic.SubtractSigned320(
            WideArithmetic.MultiplySigned192(pointY, Signed192.One),
            WideArithmetic.MultiplySigned192(radialY, Signed192.Raw(radius))));
        Signed320 projection = WideArithmetic.AddSigned320(
            WideArithmetic.MultiplySigned192(deltaX, axisX),
            WideArithmetic.MultiplySigned192(deltaY, axisY));
        Signed320 numerator = WideArithmetic.AddSigned320(projection, projection);
        Signed320 denominator = WideArithmetic.AddSigned320(
            WideArithmetic.MultiplySigned192(axisX, axisX),
            WideArithmetic.MultiplySigned192(axisY, axisY));
        Signed576 limit = WideArithmetic.MultiplySigned320(denominator, Signed192.Raw(axisLength));
        Signed576 signedNumerator = Signed576.ExtendValue(numerator);
        Fixed64 signedLength;
        if (WideArithmetic.SubtractSigned576(signedNumerator, limit).Sign >= 0)
            signedLength = axisLength;
        else if (WideArithmetic.AddSigned576(signedNumerator, limit).Sign <= 0)
            signedLength = -axisLength;
        else
            _ = Fixed64.TryGetSignedRawRatio(signedNumerator,
                Signed576.ExtendValue(denominator), out signedLength);

        return GetCenteredCapsuleSideAnchor(
            center, rotation, localAxis, signedLength, localNormal, radius);
    }

    private static FixedPointAnchor2d GetCenteredCapsuleSideAnchor(
        Vector2d center,
        Fixed64 rotation,
        Vector2d localAxis,
        Fixed64 signedAxisLength,
        Vector2d localRadialDirection,
        Fixed64 radius)
    {
        Vector2d axialOffset =
            localAxis * (signedAxisLength / Fixed64.Two);
        Vector2d radialOffset = localRadialDirection * radius;
        FixedPointAnchorTerm2d exactLocalTerm =
            FixedPointAnchorTerm2d.CreateCenteredAxisSupport(
                localAxis,
                signedAxisLength,
                localRadialDirection,
                radius,
                axialOffset,
                radialOffset);
        return new FixedPointAnchor2d(
            center,
            rotation,
            axialOffset,
            radialOffset,
            exactLocalTerm);
    }

    private static bool TryGetMinimumTranslation(
        Vector2d center,
        Vector2d capsuleAxis,
        Fixed64 axisLength,
        Fixed64 radius,
        Vector2d convexOrigin,
        Fixed64 convexRotation,
        ReadOnlySpan<Vector2d> convexOriginOffsets,
        out Vector2d normal,
        out Fixed64 depth,
        out bool depthIsClamped,
        out ContactAxis best)
    {
        RotationFrame2d frame = new(convexRotation);
        normal = default;
        depth = default;
        depthIsClamped = false;
        best = default;
        for (int i = 0; i < convexOriginOffsets.Length; i++)
        {
            WideConvex2dRelations.GetRotatedOffset(
                frame, convexOriginOffsets[i],
                out Signed192 startX, out Signed192 startY);
            WideConvex2dRelations.GetRotatedOffset(
                frame, convexOriginOffsets[(i + 1) % convexOriginOffsets.Length],
                out Signed192 endX, out Signed192 endY);
            Signed192 axisX = WideArithmetic.SubtractSigned192(startY, endY);
            Signed192 axisY = WideArithmetic.SubtractSigned192(endX, startX);
            if (axisX.IsZero && axisY.IsZero)
                continue;

            if (!TryKeepAxis(
                    center, capsuleAxis, axisLength, radius,
                    convexOrigin, frame, convexOriginOffsets,
                    axisX, axisY, true, ref best))
            {
                return false;
            }
        }

        if (!best.HasValue)
            return false;

        // The segment contributes its own pair of sides to the polygon's
        // Minkowski expansion. A closest endpoint/vertex axis need not cover it.
        if (axisLength > Fixed64.Zero
            && !TryKeepAxis(
                center, capsuleAxis, axisLength, radius,
                convexOrigin, frame, convexOriginOffsets,
                WideArithmetic.Negate(Signed192.Raw(capsuleAxis.Y)),
                Signed192.Raw(capsuleAxis.X), false, ref best))
        {
            return false;
        }

        GetClosestVertexAxis(
            center, capsuleAxis, axisLength,
            convexOrigin, frame, convexOriginOffsets,
            out Signed192 closestX, out Signed192 closestY);
        if ((!closestX.IsZero || !closestY.IsZero)
            && !TryKeepAxis(
                center, capsuleAxis, axisLength, radius,
                convexOrigin, frame, convexOriginOffsets,
                closestX, closestY, false, ref best))
        {
            return false;
        }

        normal = WideNormalization.GetNormalized(best.X, best.Y);
        depth = GetDepth(best, radius, out depthIsClamped);
        return true;
    }

    private static void GetClosestVertexAxis(
        Vector2d center,
        Vector2d capsuleAxis,
        Fixed64 axisLength,
        Vector2d convexOrigin,
        RotationFrame2d convexRotation,
        ReadOnlySpan<Vector2d> convexOriginOffsets,
        out Signed192 bestX,
        out Signed192 bestY)
    {
        bool found = false;
        bestX = default;
        bestY = default;
        Signed576 bestSquaredDistance = default;
        Signed192 directionSquared = Signed192.NarrowValue(
            WideArithmetic.AddSigned320(
                WideArithmetic.MultiplySigned192(
                    Signed192.Raw(capsuleAxis.X), Signed192.Raw(capsuleAxis.X)),
                WideArithmetic.MultiplySigned192(
                    Signed192.Raw(capsuleAxis.Y), Signed192.Raw(capsuleAxis.Y))));
        for (int i = 0; i < convexOriginOffsets.Length; i++)
        {
            GetDirectionFromCenteredAxis(
                convexOrigin, convexRotation, convexOriginOffsets[i],
                center, capsuleAxis, axisLength, directionSquared,
                out Signed192 candidateX, out Signed192 candidateY,
                out Signed576 candidateSquaredDistance);
            if (candidateX.IsZero && candidateY.IsZero)
                continue;
            if (found && WideArithmetic.CompareNonNegative(
                    candidateSquaredDistance, bestSquaredDistance) >= 0)
            {
                continue;
            }

            found = true;
            bestX = candidateX;
            bestY = candidateY;
            bestSquaredDistance = candidateSquaredDistance;
        }
    }

    private static void GetDirectionFromCenteredAxis(
        Vector2d pointOrigin,
        RotationFrame2d pointRotation,
        Vector2d pointOriginOffset,
        Vector2d axisCenter,
        Vector2d axisDirection,
        Fixed64 axisLength,
        Signed192 directionSquared,
        out Signed192 directionX,
        out Signed192 directionY,
        out Signed576 squaredDistance)
    {
        GetRelativePointNumerators(
            pointOrigin, pointRotation, pointOriginOffset, axisCenter,
            out Signed192 relativeX, out Signed192 relativeY);
        Signed192 axisX = Signed192.Raw(axisDirection.X);
        Signed192 axisY = Signed192.Raw(axisDirection.Y);
        Signed320 projection = WideArithmetic.AddSigned320(
            WideArithmetic.MultiplySigned192(relativeX, axisX),
            WideArithmetic.MultiplySigned192(relativeY, axisY));
        Signed320 doubleProjection = WideArithmetic.AddSigned320(projection, projection);
        Signed320 axialThreshold = WideArithmetic.MultiplySigned192(
            Signed192.Raw(axisLength), directionSquared);

        // A supplied normalized direction still has representable components.
        // Use its actual squared length when clamping the closest parameter.
        if (WideArithmetic.SubtractSigned320(doubleProjection, axialThreshold).Sign > 0
            || WideArithmetic.AddSigned320(doubleProjection, axialThreshold).Sign < 0)
        {
            Signed192 signedLength = projection.Sign > 0
                ? Signed192.Raw(axisLength)
                : WideArithmetic.Negate(Signed192.Raw(axisLength));
            directionX = WideArithmetic.SubtractSigned192(
                WideArithmetic.AddSigned192(relativeX, relativeX),
                Signed192.NarrowValue(WideArithmetic.MultiplySigned192(axisX, signedLength)));
            directionY = WideArithmetic.SubtractSigned192(
                WideArithmetic.AddSigned192(relativeY, relativeY),
                Signed192.NarrowValue(WideArithmetic.MultiplySigned192(axisY, signedLength)));
            Signed320 endpointSquared = WideArithmetic.AddSigned320(
                WideArithmetic.MultiplySigned192(directionX, directionX),
                WideArithmetic.MultiplySigned192(directionY, directionY));
            squaredDistance = WideArithmetic.MultiplySigned320(endpointSquared, directionSquared);
            return;
        }

        Signed320 cross = WideArithmetic.SubtractSigned320(
            WideArithmetic.MultiplySigned192(axisX, relativeY),
            WideArithmetic.MultiplySigned192(axisY, relativeX));
        directionX = cross.IsZero ? default : WideArithmetic.Negate(axisY);
        directionY = cross.IsZero ? default : axisX;
        if (cross.Sign < 0)
        {
            directionX = WideArithmetic.Negate(directionX);
            directionY = WideArithmetic.Negate(directionY);
        }

        // Endpoint and interior distances share denominator 4 * |direction|^2.
        // The interior axis is exactly perpendicular, without a rounded projection.
        Signed320 twiceCross = WideArithmetic.AddSigned320(cross, cross);
        squaredDistance = WideArithmetic.MultiplySigned320(twiceCross, twiceCross);
    }

    private static void GetSupportFeature(
        Vector2d center,
        Vector2d convexOrigin,
        RotationFrame2d convexRotation,
        ReadOnlySpan<Vector2d> convexOriginOffsets,
        ContactAxis axis,
        out Vector2d featureStart,
        out Vector2d featureEnd)
    {
        int minimumIndex = 0;
        Signed320 minimum = GetRelativeProjection(
            center,
            convexOrigin,
            convexRotation,
            convexOriginOffsets[0],
            axis.X, axis.Y);
        for (int i = 1; i < convexOriginOffsets.Length; i++)
        {
            Signed320 projection = GetRelativeProjection(
                center,
                convexOrigin,
                convexRotation,
                convexOriginOffsets[i],
                axis.X, axis.Y);
            if (WideArithmetic.SubtractSigned320(projection, minimum).Sign < 0)
            {
                minimum = projection;
                minimumIndex = i;
            }
        }

        featureStart = convexOriginOffsets[minimumIndex];
        featureEnd = featureStart;
        if (!axis.FromEdge)
            return;

        int previousIndex = minimumIndex;
        do
        {
            previousIndex =
                (previousIndex + convexOriginOffsets.Length - 1)
                % convexOriginOffsets.Length;
        }
        while (convexOriginOffsets[previousIndex] == featureStart);

        int nextIndex = minimumIndex;
        do
        {
            nextIndex = (nextIndex + 1) % convexOriginOffsets.Length;
        }
        while (convexOriginOffsets[nextIndex] == featureStart);

        Vector2d previous = convexOriginOffsets[previousIndex];
        Signed320 previousProjection = GetRelativeProjection(
            center, convexOrigin, convexRotation, previous, axis.X, axis.Y);
        if (WideArithmetic.SubtractSigned320(previousProjection, minimum).IsZero)
            featureStart = previous;
        else if (WideArithmetic.SubtractSigned320(
                     GetRelativeProjection(center, convexOrigin, convexRotation,
                         convexOriginOffsets[nextIndex], axis.X, axis.Y), minimum).IsZero)
            featureEnd = convexOriginOffsets[nextIndex];
    }

    private static bool TryKeepAxis(
        Vector2d center,
        Vector2d capsuleAxis,
        Fixed64 axisLength,
        Fixed64 radius,
        Vector2d convexOrigin,
        RotationFrame2d convexRotation,
        ReadOnlySpan<Vector2d> convexOriginOffsets,
        Signed192 axisX,
        Signed192 axisY,
        bool axisFromEdge,
        ref ContactAxis best)
    {
        GetRelativeProjectionRange(
            center, convexOrigin, convexRotation, convexOriginOffsets,
            axisX, axisY, out Signed320 minimum, out Signed320 maximum);
        Signed192 axialAlignment = Signed192.NarrowValue(WideArithmetic.AddSigned320(
            WideArithmetic.MultiplySigned192(Signed192.Raw(capsuleAxis.X), axisX),
            WideArithmetic.MultiplySigned192(Signed192.Raw(capsuleAxis.Y), axisY)));
        if (axialAlignment.Sign < 0)
            axialAlignment = WideArithmetic.Negate(axialAlignment);
        Signed320 axialExtent = WideArithmetic.MultiplySigned192(
            axialAlignment, Signed192.Raw(axisLength));
        Signed320 positive = WideArithmetic.SubtractSigned320(
            axialExtent, WideArithmetic.AddSigned320(minimum, minimum));
        Signed320 negative = WideArithmetic.AddSigned320(
            axialExtent, WideArithmetic.AddSigned320(maximum, maximum));
        bool usePositive = WideArithmetic.SubtractSigned320(positive, negative).Sign <= 0;
        Signed320 overlap = usePositive ? positive : negative;
        Signed320 axisSquared = WideArithmetic.AddSigned320(
            WideArithmetic.MultiplySigned192(axisX, axisX),
            WideArithmetic.MultiplySigned192(axisY, axisY));
        ContactAxis candidate = new(
            usePositive ? axisX : WideArithmetic.Negate(axisX),
            usePositive ? axisY : WideArithmetic.Negate(axisY),
            axisSquared, overlap, axisFromEdge);

        // With S = 2^32, the exact depth in raw units is
        // radius.Raw + overlap / (2*S*sqrt(axisSquared)). Test its sign
        // before rounding either the axis, radial support, or public depth.
        if (CompareDepthToTwiceRaw(candidate, radius, default) < 0)
            return false;
        if (!best.HasValue || CompareDepths(candidate, best) < 0)
            best = candidate;
        return true;
    }

    private static int CompareDepths(ContactAxis left, ContactAxis right)
    {
        // The radius is identical on every axis and cancels. Compare signed
        // normalized axial overlaps; squaring reverses order for negative ones.
        int leftSign = left.Overlap.Sign;
        int rightSign = right.Overlap.Sign;
        if (leftSign != rightSign)
            return leftSign.CompareTo(rightSign);
        Signed576 leftSquared = WideArithmetic.MultiplySigned320(left.Overlap, left.Overlap);
        Signed576 rightSquared = WideArithmetic.MultiplySigned320(right.Overlap, right.Overlap);
        int comparison = WideArithmetic.SubtractSigned832(
            WideArithmetic.MultiplySigned576ToSigned832(leftSquared, right.SquaredLength),
            WideArithmetic.MultiplySigned576ToSigned832(rightSquared, left.SquaredLength)).Sign;
        return leftSign < 0 ? -comparison : comparison;
    }

    private static int CompareDepthToTwiceRaw(
        ContactAxis axis,
        Fixed64 radius,
        Signed192 twiceRaw)
    {
        Signed192 threshold = Signed192.NarrowValue(WideArithmetic.MultiplySigned192(
            WideArithmetic.SubtractSigned192(
                twiceRaw,
                WideArithmetic.AddSigned192(Signed192.Raw(radius), Signed192.Raw(radius))),
            Signed192.One));
        int overlapSign = axis.Overlap.Sign;
        int thresholdSign = threshold.Sign;
        if (overlapSign != thresholdSign)
            return overlapSign.CompareTo(thresholdSign);
        Signed576 left = WideArithmetic.MultiplySigned320(axis.Overlap, axis.Overlap);
        Signed576 right = WideArithmetic.MultiplySigned320(
            WideArithmetic.MultiplySigned192(threshold, threshold), axis.SquaredLength);
        int comparison = WideArithmetic.SubtractSigned576(left, right).Sign;
        return overlapSign < 0 ? -comparison : comparison;
    }

    private static Fixed64 GetDepth(ContactAxis axis, Fixed64 radius, out bool isClamped)
    {
        Signed192 twiceMaximum = WideArithmetic.AddSigned192(
            Signed192.Signed(long.MaxValue), Signed192.Signed(long.MaxValue));
        isClamped = CompareDepthToTwiceRaw(axis, radius, twiceMaximum) > 0;
        if (isClamped)
            return Fixed64.MaxValue;

        Signed320 root = WideArithmetic.GetFloorSquareRootScaledByFixed64(
            Signed576.ExtendValue(axis.SquaredLength));
        long first = GetDepthBound(axis.Overlap, radius, root);
        long second = GetDepthBound(axis.Overlap, radius,
            WideArithmetic.AddSigned320(root, Signed320.ExtendValue(Signed192.Signed(1))));
        long low = Math.Min(first, second);
        long high = Math.Max(first, second);
        // The exact root lies between these adjacent integer roots. Their
        // rounded depths bound the answer, even for sub-raw axes or cancellation
        // against the radius. Only a straddled midpoint needs exact refinement.
        while (low < high)
        {
            long candidate = (long)(((ulong)low + (ulong)high + 1UL) >> 1);
            Signed192 midpoint = WideArithmetic.SubtractSigned192(
                WideArithmetic.AddSigned192(
                    Signed192.Signed(candidate), Signed192.Signed(candidate)),
                Signed192.Signed(1));
            int comparison = CompareDepthToTwiceRaw(axis, radius, midpoint);
            if (comparison > 0 || (comparison == 0 && (candidate & 1L) == 0L))
                low = candidate;
            else
                high = candidate - 1L;
        }
        return Fixed64.FromRaw(low);
    }

    private static long GetDepthBound(Signed320 overlap, Fixed64 radius, Signed320 scaledRoot)
    {
        Signed320 denominator = WideArithmetic.AddSigned320(scaledRoot, scaledRoot);
        Signed576 numerator = WideArithmetic.AddSigned576(
            Signed576.ExtendValue(overlap),
            WideArithmetic.MultiplySigned320(denominator, Signed192.Raw(radius)));
        if (numerator.Sign <= 0)
            return 0L;
        Signed576 maximum = WideArithmetic.MultiplySigned320(
            denominator, Signed192.Signed(long.MaxValue));
        if (WideArithmetic.SubtractSigned576(numerator, maximum).Sign >= 0)
            return long.MaxValue;

        // Clip the root bound before conversion, so nearest-even narrowing is
        // representable even when an approximation lies outside the raw domain.
        _ = Fixed64.TryGetSignedRawRatio(
            numerator, Signed576.ExtendValue(denominator), out Fixed64 bound);
        return bound.m_rawValue;
    }

    private static void GetRelativeProjectionRange(
        Vector2d center,
        Vector2d convexOrigin,
        RotationFrame2d convexRotation,
        ReadOnlySpan<Vector2d> convexOriginOffsets,
        Signed192 axisX,
        Signed192 axisY,
        out Signed320 minimum,
        out Signed320 maximum)
    {
        minimum = GetRelativeProjection(
            center, convexOrigin, convexRotation, convexOriginOffsets[0], axisX, axisY);
        maximum = minimum;
        for (int i = 1; i < convexOriginOffsets.Length; i++)
        {
            Signed320 projection = GetRelativeProjection(
                center, convexOrigin, convexRotation, convexOriginOffsets[i], axisX, axisY);
            if (WideArithmetic.SubtractSigned320(projection, minimum).Sign < 0)
                minimum = projection;
            else if (WideArithmetic.SubtractSigned320(projection, maximum).Sign > 0)
                maximum = projection;
        }
    }

    private static Signed320 GetRelativeProjection(
        Vector2d center,
        Vector2d pointOrigin,
        RotationFrame2d pointRotation,
        Vector2d pointOriginOffset,
        Signed192 axisX,
        Signed192 axisY)
    {
        GetRelativePointNumerators(
            pointOrigin, pointRotation, pointOriginOffset, center,
            out Signed192 differenceX, out Signed192 differenceY);
        return WideArithmetic.AddSigned320(
            WideArithmetic.MultiplySigned192(differenceX, axisX),
            WideArithmetic.MultiplySigned192(differenceY, axisY));
    }
    private static void GetRelativePointNumerators(
        Vector2d origin,
        RotationFrame2d rotation,
        Vector2d offset,
        Vector2d center,
        out Signed192 x,
        out Signed192 y)
    {
        WideConvex2dRelations.GetRotatedOffset(rotation, offset, out x, out y);
        x = WideArithmetic.AddSigned192(x, WideArithmetic.SubtractSigned192(
            WideArithmetic.Scale(origin.X), WideArithmetic.Scale(center.X)));
        y = WideArithmetic.AddSigned192(y, WideArithmetic.SubtractSigned192(
            WideArithmetic.Scale(origin.Y), WideArithmetic.Scale(center.Y)));
    }

}
