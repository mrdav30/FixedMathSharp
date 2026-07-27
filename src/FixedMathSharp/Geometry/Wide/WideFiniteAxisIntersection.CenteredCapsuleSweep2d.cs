//=======================================================================
// WideFiniteAxisIntersection.CenteredCapsuleSweep2d.cs
//=======================================================================
// MIT License, Copyright (c) 2024–present David Oravsky (mrdav30)
// See LICENSE file in the project root for full license information.
//=======================================================================

namespace FixedMathSharp.Geometry;

/// <content>
/// Provides 2D swept intersection tests between a center-defined capsule (axis + radius)
/// moving along a direction and a target segment-based capsule, using wide (Signed192) precision.
/// </content>
internal static partial class WideFiniteAxisIntersection
{
    #region Nested Types

    private readonly struct WideAxis2d
    {
        internal readonly Signed192 StartX;
        internal readonly Signed192 StartY;
        internal readonly Signed192 EndX;
        internal readonly Signed192 EndY;

        internal WideAxis2d(Signed192 startX, Signed192 startY, Signed192 endX, Signed192 endY)
        {
            StartX = startX;
            StartY = startY;
            EndX = endX;
            EndY = endY;
        }
    }

    #endregion

    internal static bool TryGetSweptCenteredCapsuleSegmentFirstDistance(
        Vector2d center,
        Vector2d axis,
        Fixed64 axisLength,
        Fixed64 radius,
        Vector2d direction,
        Fixed64 maximumDistance,
        FixedSegment2d targetAxis,
        Fixed64 targetRadius,
        out Fixed64 distance,
        out Vector2d normal)
    {
        GetCenteredAxisEndpoints(center, axis, axisLength, out WideAxis2d mover);
        GetAuthoredAxisEndpoints(targetAxis, out WideAxis2d target);
        return TryGetSweptAxesFirstDistance(
            mover,
            radius,
            direction,
            maximumDistance,
            target,
            targetRadius,
            axis,
            out distance,
            out normal);
    }

    internal static bool TryGetSweptCenteredCapsuleOriginOffsetSegmentFirstDistance(
        Vector2d center,
        Vector2d axis,
        Fixed64 axisLength,
        Fixed64 radius,
        Vector2d direction,
        Fixed64 maximumDistance,
        Vector2d targetOrigin,
        Fixed64 targetRotation,
        Vector2d targetStartOffset,
        Vector2d targetEndOffset,
        Fixed64 targetRadius,
        out Fixed64 distance,
        out Vector2d normal)
    {
        GetCenteredAxisEndpoints(center, axis, axisLength, out WideAxis2d mover);
        GetRotatedOriginOffsetAxisEndpoints(
            targetOrigin,
            targetRotation,
            targetStartOffset,
            targetEndOffset,
            out WideAxis2d target);
        return TryGetSweptAxesFirstDistance(
            mover,
            radius,
            direction,
            maximumDistance,
            target,
            targetRadius,
            axis,
            out distance,
            out normal);
    }

    internal static bool TryGetSweptOriginOffsetSegmentsFirstDistance(
        Vector2d moverOrigin,
        Fixed64 moverRotation,
        Vector2d moverStartOffset,
        Vector2d moverEndOffset,
        Fixed64 moverRadius,
        Vector2d direction,
        Fixed64 maximumDistance,
        Vector2d targetOrigin,
        Fixed64 targetRotation,
        Vector2d targetStartOffset,
        Vector2d targetEndOffset,
        Fixed64 targetRadius,
        out Fixed64 distance,
        out Vector2d normal)
    {
        GetRotatedOriginOffsetAxisEndpoints(
            moverOrigin,
            moverRotation,
            moverStartOffset,
            moverEndOffset,
            out WideAxis2d mover);
        GetRotatedOriginOffsetAxisEndpoints(
            targetOrigin,
            targetRotation,
            targetStartOffset,
            targetEndOffset,
            out WideAxis2d target);
        return TryGetSweptAxesFirstDistance(
            mover,
            moverRadius,
            direction,
            maximumDistance,
            target,
            targetRadius,
            WideConvex2dRelations.GetTransformedEdgeDirection(
                moverRotation,
                moverStartOffset,
                moverEndOffset),
            out distance,
            out normal);
    }

    internal static bool TryGetSweptCenteredCapsulesFirstDistance(
        Vector2d center,
        Vector2d axis,
        Fixed64 axisLength,
        Fixed64 radius,
        Vector2d direction,
        Fixed64 maximumDistance,
        Vector2d targetCenter,
        Vector2d targetAxis,
        Fixed64 targetAxisLength,
        Fixed64 targetRadius,
        out Fixed64 distance,
        out Vector2d normal)
    {
        GetCenteredAxisEndpoints(center, axis, axisLength, out WideAxis2d mover);
        GetCenteredAxisEndpoints(targetCenter, targetAxis, targetAxisLength, out WideAxis2d target);
        return TryGetSweptAxesFirstDistance(
            mover,
            radius,
            direction,
            maximumDistance,
            target,
            targetRadius,
            axis,
            out distance,
            out normal);
    }

    private static bool TryGetSweptAxesFirstDistance(
        WideAxis2d mover,
        Fixed64 moverRadius,
        Vector2d direction,
        Fixed64 maximumDistance,
        WideAxis2d target,
        Fixed64 targetRadius,
        Vector2d moverAxis,
        out Fixed64 distance,
        out Vector2d normal)
    {
        Signed192 radius = WideArithmetic.AddSigned192(
            Signed192.Signed(moverRadius.m_rawValue),
            Signed192.Signed(targetRadius.m_rawValue));
        Signed192 scaledRadius = Signed192.NarrowValue(
            WideArithmetic.MultiplySigned192(radius, DoubleParameterScale));
        GetSweepVelocity(
            direction,
            maximumDistance,
            out Signed192 displacementPerRawX,
            out Signed192 displacementPerRawY,
            out Signed192 velocityX,
            out Signed192 velocityY);

        bool found = false;
        distance = default;
        normal = default;
        TryKeepEndpointAxisSweep(
            mover.StartX, mover.StartY,
            displacementPerRawX, displacementPerRawY,
            velocityX, velocityY,
            target, scaledRadius, maximumDistance, reverseNormal: false,
            ref found, ref distance, ref normal);
        TryKeepEndpointAxisSweep(
            mover.EndX, mover.EndY,
            displacementPerRawX, displacementPerRawY,
            velocityX, velocityY,
            target, scaledRadius, maximumDistance, reverseNormal: false,
            ref found, ref distance, ref normal);
        TryKeepEndpointAxisSweep(
            target.StartX, target.StartY,
            WideArithmetic.SubtractSigned192(default, displacementPerRawX),
            WideArithmetic.SubtractSigned192(default, displacementPerRawY),
            WideArithmetic.SubtractSigned192(default, velocityX),
            WideArithmetic.SubtractSigned192(default, velocityY),
            mover, scaledRadius, maximumDistance, reverseNormal: true,
            ref found, ref distance, ref normal);
        TryKeepEndpointAxisSweep(
            target.EndX, target.EndY,
            WideArithmetic.SubtractSigned192(default, displacementPerRawX),
            WideArithmetic.SubtractSigned192(default, displacementPerRawY),
            WideArithmetic.SubtractSigned192(default, velocityX),
            WideArithmetic.SubtractSigned192(default, velocityY),
            mover, scaledRadius, maximumDistance, reverseNormal: true,
            ref found, ref distance, ref normal);

        if (AxesIntersect(mover, target) && (!found || distance > Fixed64.Zero))
        {
            distance = Fixed64.Zero;
            normal = new Vector2d(-moverAxis.Y, moverAxis.X);
            return true;
        }

        return found;
    }

    private static void TryKeepEndpointAxisSweep(
        Signed192 startX,
        Signed192 startY,
        Signed192 displacementPerRawX,
        Signed192 displacementPerRawY,
        Signed192 velocityX,
        Signed192 velocityY,
        WideAxis2d target,
        Signed192 scaledRadius,
        Fixed64 maximumDistance,
        bool reverseNormal,
        ref bool found,
        ref Fixed64 bestDistance,
        ref Vector2d bestNormal)
    {
        bool candidateFound = false;
        Fixed64 candidateDistance = default;
        TryKeepCircleSweep(
            startX, startY, velocityX, velocityY,
            target.StartX, target.StartY, scaledRadius, maximumDistance,
            ref candidateFound, ref candidateDistance);
        TryKeepCircleSweep(
            startX, startY, velocityX, velocityY,
            target.EndX, target.EndY, scaledRadius, maximumDistance,
            ref candidateFound, ref candidateDistance);
        TryKeepSideSweep(
            startX, startY,
            displacementPerRawX, displacementPerRawY,
            velocityX, velocityY,
            target, scaledRadius, maximumDistance,
            ref candidateFound, ref candidateDistance);
        if (!candidateFound || (found && candidateDistance >= bestDistance))
            return;

        found = true;
        bestDistance = candidateDistance;
        bestNormal = GetEndpointAxisNormal(
            startX,
            startY,
            displacementPerRawX,
            displacementPerRawY,
            candidateDistance,
            target);
        if (bestNormal == Vector2d.Zero)
        {
            bestNormal = GetAxisSweepFallbackNormal(
                target,
                velocityX,
                velocityY);
        }
        if (reverseNormal)
            bestNormal = -bestNormal;
    }

    private static Vector2d GetAxisSweepFallbackNormal(
        WideAxis2d target,
        Signed192 velocityX,
        Signed192 velocityY)
    {
        Vector2d edgeDirection = WideGeometry.GetNormalized(
            WideArithmetic.SubtractSigned192(target.EndX, target.StartX),
            WideArithmetic.SubtractSigned192(target.EndY, target.StartY));
        Vector2d normal = edgeDirection.RightHandNormal;
        if (normal == Vector2d.Zero)
            normal = WideGeometry.GetNormalized(velocityX, velocityY);

        Signed320 velocityProjection = AddProducts(
            Signed192.Signed(normal.X.m_rawValue),
            velocityX,
            Signed192.Signed(normal.Y.m_rawValue),
            velocityY);
        return velocityProjection.Sign < 0 ? -normal : normal;
    }

    private static void TryKeepCircleSweep(
        Signed192 startX,
        Signed192 startY,
        Signed192 velocityX,
        Signed192 velocityY,
        Signed192 circleX,
        Signed192 circleY,
        Signed192 scaledRadius,
        Fixed64 maximumDistance,
        ref bool found,
        ref Fixed64 distance)
    {
        Signed192 x = WideArithmetic.SubtractSigned192(startX, circleX);
        Signed192 y = WideArithmetic.SubtractSigned192(startY, circleY);
        Signed320 coefficient = AddProducts(velocityX, velocityX, velocityY, velocityY);
        Signed320 projection = AddProducts(x, velocityX, y, velocityY);
        Signed320 constant = WideArithmetic.SubtractSigned320(
            AddProducts(x, x, y, y),
            WideArithmetic.MultiplySigned192(scaledRadius, scaledRadius));
        KeepFirstSweepRoot(
            Signed576.ExtendValue(coefficient),
            Signed576.ExtendValue(projection),
            Signed576.ExtendValue(constant),
            maximumDistance,
            ref found,
            ref distance);
    }

    private static void TryKeepSideSweep(
        Signed192 startX,
        Signed192 startY,
        Signed192 displacementPerRawX,
        Signed192 displacementPerRawY,
        Signed192 velocityX,
        Signed192 velocityY,
        WideAxis2d target,
        Signed192 scaledRadius,
        Fixed64 maximumDistance,
        ref bool found,
        ref Fixed64 distance)
    {
        Signed192 deltaX = WideArithmetic.SubtractSigned192(target.EndX, target.StartX);
        Signed192 deltaY = WideArithmetic.SubtractSigned192(target.EndY, target.StartY);
        Signed192 offsetX = WideArithmetic.SubtractSigned192(startX, target.StartX);
        Signed192 offsetY = WideArithmetic.SubtractSigned192(startY, target.StartY);
        Signed320 startCross = WideArithmetic.MultiplySubtract(
            offsetX, deltaY, offsetY, deltaX);
        Signed320 velocityCross = WideArithmetic.MultiplySubtract(
            velocityX, deltaY, velocityY, deltaX);
        Signed320 deltaSquared = AddProducts(deltaX, deltaX, deltaY, deltaY);
        if (deltaSquared.IsZero)
            return;

        Signed576 coefficient = WideArithmetic.MultiplySigned320(velocityCross, velocityCross);
        Signed576 projection = WideArithmetic.MultiplySigned320(startCross, velocityCross);
        Signed576 constant = WideArithmetic.SubtractSigned576(
            WideArithmetic.MultiplySigned320(startCross, startCross),
            WideArithmetic.MultiplySigned320(
                WideArithmetic.MultiplySigned192(scaledRadius, scaledRadius),
                deltaSquared));
        bool sideFound = WideFiniteConeIntersection.TrySolveUnitPolynomial(
            coefficient,
            projection,
            constant,
            maximumDistance,
            out Fixed64 entry,
            out _);
        if (!sideFound
            || !IsWithinAxisAtDistance(
                startX,
                startY,
                displacementPerRawX,
                displacementPerRawY,
                entry,
                target,
                deltaX,
                deltaY,
                deltaSquared)
            || (found && entry >= distance))
        {
            return;
        }

        found = true;
        distance = entry;
    }

    private static bool IsWithinAxisAtDistance(
        Signed192 startX,
        Signed192 startY,
        Signed192 displacementPerRawX,
        Signed192 displacementPerRawY,
        Fixed64 distance,
        WideAxis2d target,
        Signed192 deltaX,
        Signed192 deltaY,
        Signed320 deltaSquared)
    {
        GetPointAtDistance(
            startX,
            startY,
            displacementPerRawX,
            displacementPerRawY,
            distance,
            out Signed192 pointX,
            out Signed192 pointY);
        Signed320 projection = AddProducts(
            WideArithmetic.SubtractSigned192(pointX, target.StartX),
            deltaX,
            WideArithmetic.SubtractSigned192(pointY, target.StartY),
            deltaY);
        return projection.Sign >= 0
            && WideArithmetic.SubtractSigned320(projection, deltaSquared).Sign <= 0;
    }

    private static void KeepFirstSweepRoot(
        Signed576 coefficient,
        Signed576 projection,
        Signed576 constant,
        Fixed64 maximumDistance,
        ref bool found,
        ref Fixed64 distance)
    {
        if (!WideFiniteConeIntersection.TrySolveUnitPolynomial(
                coefficient,
                projection,
                constant,
                maximumDistance,
                out Fixed64 entry,
                out _)
            || (found && entry >= distance))
        {
            return;
        }

        found = true;
        distance = entry;
    }

    private static Vector2d GetEndpointAxisNormal(
        Signed192 startX,
        Signed192 startY,
        Signed192 displacementPerRawX,
        Signed192 displacementPerRawY,
        Fixed64 distance,
        WideAxis2d target)
    {
        GetPointAtDistance(
            startX,
            startY,
            displacementPerRawX,
            displacementPerRawY,
            distance,
            out Signed192 pointX,
            out Signed192 pointY);
        Signed192 deltaX = WideArithmetic.SubtractSigned192(target.EndX, target.StartX);
        Signed192 deltaY = WideArithmetic.SubtractSigned192(target.EndY, target.StartY);
        Signed320 deltaSquared = AddProducts(deltaX, deltaX, deltaY, deltaY);
        Signed320 projection = AddProducts(
            WideArithmetic.SubtractSigned192(pointX, target.StartX),
            deltaX,
            WideArithmetic.SubtractSigned192(pointY, target.StartY),
            deltaY);
        if (projection.Sign <= 0)
        {
            return WideGeometry.GetNormalized(
                WideArithmetic.SubtractSigned192(target.StartX, pointX),
                WideArithmetic.SubtractSigned192(target.StartY, pointY));
        }
        if (WideArithmetic.SubtractSigned320(projection, deltaSquared).Sign >= 0)
        {
            return WideGeometry.GetNormalized(
                WideArithmetic.SubtractSigned192(target.EndX, pointX),
                WideArithmetic.SubtractSigned192(target.EndY, pointY));
        }

        Signed320 closestX = Signed320.NarrowValue(WideArithmetic.SubtractSigned576(
            WideArithmetic.AddSigned576(
                WideArithmetic.MultiplySigned320(
                    Signed320.ExtendValue(target.StartX),
                    deltaSquared),
                WideArithmetic.MultiplySigned320(
                    Signed320.ExtendValue(deltaX),
                    projection)),
            WideArithmetic.MultiplySigned320(
                Signed320.ExtendValue(pointX),
                deltaSquared)));
        Signed320 closestY = Signed320.NarrowValue(WideArithmetic.SubtractSigned576(
            WideArithmetic.AddSigned576(
                WideArithmetic.MultiplySigned320(
                    Signed320.ExtendValue(target.StartY),
                    deltaSquared),
                WideArithmetic.MultiplySigned320(
                    Signed320.ExtendValue(deltaY),
                    projection)),
            WideArithmetic.MultiplySigned320(
                Signed320.ExtendValue(pointY),
                deltaSquared)));
        return WideGeometry.GetNormalized(closestX, closestY);
    }

    private static void GetPointAtDistance(
        Signed192 startX,
        Signed192 startY,
        Signed192 displacementPerRawX,
        Signed192 displacementPerRawY,
        Fixed64 distance,
        out Signed192 pointX,
        out Signed192 pointY)
    {
        Signed192 rawDistance = Signed192.Signed(distance.m_rawValue);
        pointX = Signed192.NarrowValue(WideArithmetic.AddSigned320(
            Signed320.ExtendValue(startX),
            WideArithmetic.MultiplySigned192(displacementPerRawX, rawDistance)));
        pointY = Signed192.NarrowValue(WideArithmetic.AddSigned320(
            Signed320.ExtendValue(startY),
            WideArithmetic.MultiplySigned192(displacementPerRawY, rawDistance)));
    }

    private static bool AxesIntersect(WideAxis2d first, WideAxis2d second)
    {
        int firstStart = Orientation(first.StartX, first.StartY, first.EndX, first.EndY, second.StartX, second.StartY);
        int firstEnd = Orientation(first.StartX, first.StartY, first.EndX, first.EndY, second.EndX, second.EndY);
        int secondStart = Orientation(second.StartX, second.StartY, second.EndX, second.EndY, first.StartX, first.StartY);
        int secondEnd = Orientation(second.StartX, second.StartY, second.EndX, second.EndY, first.EndX, first.EndY);
        return (firstStart == 0 && IsWithinBounds(first, second.StartX, second.StartY))
            || (firstEnd == 0 && IsWithinBounds(first, second.EndX, second.EndY))
            || (secondStart == 0 && IsWithinBounds(second, first.StartX, first.StartY))
            || (secondEnd == 0 && IsWithinBounds(second, first.EndX, first.EndY))
            || (firstStart != firstEnd && secondStart != secondEnd);
    }

    private static int Orientation(
        Signed192 startX,
        Signed192 startY,
        Signed192 endX,
        Signed192 endY,
        Signed192 pointX,
        Signed192 pointY) =>
        WideArithmetic.MultiplySubtract(
            WideArithmetic.SubtractSigned192(endX, startX),
            WideArithmetic.SubtractSigned192(pointY, startY),
            WideArithmetic.SubtractSigned192(endY, startY),
            WideArithmetic.SubtractSigned192(pointX, startX)).Sign;

    private static bool IsWithinBounds(WideAxis2d axis, Signed192 x, Signed192 y) =>
        IsBetween(x, axis.StartX, axis.EndX)
        && IsBetween(y, axis.StartY, axis.EndY);

    private static bool IsBetween(Signed192 value, Signed192 first, Signed192 second)
    {
        if (WideArithmetic.SubtractSigned192(first, second).Sign > 0)
            (first, second) = (second, first);
        return WideArithmetic.SubtractSigned192(value, first).Sign >= 0
            && WideArithmetic.SubtractSigned192(value, second).Sign <= 0;
    }

    private static void GetSweepVelocity(
        Vector2d direction,
        Fixed64 maximumDistance,
        out Signed192 displacementPerRawX,
        out Signed192 displacementPerRawY,
        out Signed192 x,
        out Signed192 y)
    {
        Signed192 rawX = Signed192.Signed(direction.X.m_rawValue);
        Signed192 rawY = Signed192.Signed(direction.Y.m_rawValue);
        displacementPerRawX = WideArithmetic.AddSigned192(rawX, rawX);
        displacementPerRawY = WideArithmetic.AddSigned192(rawY, rawY);
        Signed192 distance = Signed192.Signed(maximumDistance.m_rawValue);
        x = Signed192.NarrowValue(WideArithmetic.MultiplySigned192(displacementPerRawX, distance));
        y = Signed192.NarrowValue(WideArithmetic.MultiplySigned192(displacementPerRawY, distance));
    }

    private static void GetCenteredAxisEndpoints(
        Vector2d center,
        Vector2d axis,
        Fixed64 axisLength,
        out WideAxis2d endpoints)
    {
        Signed192 startX = GetCenteredEndpointNumerator(center.X, axis.X, axisLength, positive: false);
        Signed192 startY = GetCenteredEndpointNumerator(center.Y, axis.Y, axisLength, positive: false);
        Signed192 endX = GetCenteredEndpointNumerator(center.X, axis.X, axisLength, positive: true);
        Signed192 endY = GetCenteredEndpointNumerator(center.Y, axis.Y, axisLength, positive: true);
        endpoints = new WideAxis2d(startX, startY, endX, endY);
    }

    private static void GetAuthoredAxisEndpoints(FixedSegment2d axis, out WideAxis2d endpoints) =>
        endpoints = new WideAxis2d(
            ScaleAuthoredCoordinate(axis.Start.X),
            ScaleAuthoredCoordinate(axis.Start.Y),
            ScaleAuthoredCoordinate(axis.End.X),
            ScaleAuthoredCoordinate(axis.End.Y));

    private static void GetRotatedOriginOffsetAxisEndpoints(
        Vector2d origin,
        Fixed64 rotation,
        Vector2d startOffset,
        Vector2d endOffset,
        out WideAxis2d endpoints)
    {
        GetRotatedOriginOffsetCoordinate(
            origin,
            rotation,
            startOffset,
            out Signed192 startX,
            out Signed192 startY);
        GetRotatedOriginOffsetCoordinate(
            origin,
            rotation,
            endOffset,
            out Signed192 endX,
            out Signed192 endY);
        endpoints = new WideAxis2d(startX, startY, endX, endY);
    }

    private static void GetRotatedOriginOffsetCoordinate(
        Vector2d origin,
        Fixed64 rotation,
        Vector2d offset,
        out Signed192 x,
        out Signed192 y)
    {
        Signed192 cosine =
            Signed192.Signed(
                FixedMath.Cos(rotation).m_rawValue);
        Signed192 sine =
            Signed192.Signed(
                FixedMath.Sin(rotation).m_rawValue);
        Signed320 rotatedX = WideArithmetic.SubtractSigned320(
            WideArithmetic.MultiplySigned192(
                Signed192.Signed(offset.X.m_rawValue),
                cosine),
            WideArithmetic.MultiplySigned192(
                Signed192.Signed(offset.Y.m_rawValue),
                sine));
        Signed320 rotatedY = WideArithmetic.AddSigned320(
            WideArithmetic.MultiplySigned192(
                Signed192.Signed(offset.X.m_rawValue),
                sine),
            WideArithmetic.MultiplySigned192(
                Signed192.Signed(offset.Y.m_rawValue),
                cosine));
        Signed320 originX = WideArithmetic.MultiplySigned192(
            Signed192.Signed(origin.X.m_rawValue),
            DoubleParameterScale);
        Signed320 originY = WideArithmetic.MultiplySigned192(
            Signed192.Signed(origin.Y.m_rawValue),
            DoubleParameterScale);
        x = Signed192.NarrowValue(
            WideArithmetic.AddSigned320(
                originX,
                WideArithmetic.AddSigned320(rotatedX, rotatedX)));
        y = Signed192.NarrowValue(
            WideArithmetic.AddSigned320(
                originY,
                WideArithmetic.AddSigned320(rotatedY, rotatedY)));
    }

    private static Signed192 GetCenteredEndpointNumerator(
        Fixed64 center,
        Fixed64 axis,
        Fixed64 axisLength,
        bool positive)
    {
        Signed320 centerTerm = WideArithmetic.MultiplySigned192(
            Signed192.Signed(center.m_rawValue),
            DoubleParameterScale);
        Signed320 axisTerm = WideArithmetic.MultiplySigned192(
            Signed192.Signed(axis.m_rawValue),
            Signed192.Signed(axisLength.m_rawValue));
        return Signed192.NarrowValue(positive
            ? WideArithmetic.AddSigned320(centerTerm, axisTerm)
            : WideArithmetic.SubtractSigned320(centerTerm, axisTerm));
    }

    private static Signed192 ScaleAuthoredCoordinate(Fixed64 value) =>
        Signed192.NarrowValue(WideArithmetic.MultiplySigned192(
            Signed192.Signed(value.m_rawValue),
            DoubleParameterScale));
}
