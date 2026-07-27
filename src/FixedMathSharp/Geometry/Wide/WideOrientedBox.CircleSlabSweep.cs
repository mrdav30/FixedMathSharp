//=======================================================================
// WideOrientedBox.CircleSlabSweep.cs
//=======================================================================
// MIT License, Copyright (c) 2024–present David Oravsky (mrdav30)
// See LICENSE file in the project root for full license information.
//=======================================================================

using System;

namespace FixedMathSharp.Bounds;

/// <content>
/// High-precision sweep tests between a circle and an oriented box slab,
/// using wide fixed-point arithmetic to avoid overflow and precision loss.
/// </content>
internal static partial class WideOrientedBox
{
    private static readonly Signed192 SweepRawScale = Signed192.Signed(FixedMath.ONE_L);
    private static readonly Signed192 SweepDoubleRawScale = Signed192.Signed(FixedMath.ONE_L * 2L);
    private static readonly Signed192 SweepOneCoefficient = Signed192.Signed(1L);

    #region Nested Types

    private readonly struct RationalPointDistance
    {
        internal readonly Signed576 Denominator;
        internal readonly Signed832 SquaredDistance;

        internal RationalPointDistance(
            Signed576 denominator,
            Signed832 squaredDistance)
        {
            Denominator = denominator;
            SquaredDistance = squaredDistance;
        }
    }

    #endregion

    internal static bool TryGetCircleSlabSweepDistance(
        Vector3d center,
        FixedQuaternion orientation,
        Vector3d halfExtents,
        Vector3d slabStartCenter,
        Vector2d normalizedDirection,
        Fixed64 maxDistance,
        Fixed64 slabHalfThickness,
        Fixed64 radius,
        out Fixed64 distance)
    {
        if (TryGetCircleSlabContact(
                center,
                orientation,
                halfExtents,
                slabStartCenter,
                Fixed64.Zero,
                slabHalfThickness,
                radius,
                out _))
        {
            distance = Fixed64.Zero;
            return true;
        }

        if (maxDistance == Fixed64.Zero)
        {
            distance = default;
            return false;
        }

        RationalBasis basis = new(orientation);
        Signed192 lowerY = WideArithmetic.SubtractSigned192(
            Signed192.Raw(slabStartCenter.Y),
            Signed192.Raw(slabHalfThickness));
        Signed192 upperY = WideArithmetic.AddSigned192(
            Signed192.Raw(slabStartCenter.Y),
            Signed192.Raw(slabHalfThickness));
        Span<SweepPlanarConstraint> constraints =
            stackalloc SweepPlanarConstraint[16];
        if (!TryBuildCircleSlabProjectionConstraints(
                center,
                halfExtents,
                basis,
                lowerY,
                upperY,
                constraints,
                out int constraintCount))
        {
            distance = default;
            return false;
        }

        Span<SweepRationalPoint> vertices =
            stackalloc SweepRationalPoint[32];
        BuildCircleSlabProjectionVertices(
            center,
            halfExtents,
            basis,
            lowerY,
            upperY,
            vertices,
            out int vertexCount);
        Vector2d planarStart = new(slabStartCenter.X, slabStartCenter.Z);
        bool found = false;
        Fixed64 best = Fixed64.MaxValue;
        for (int index = 0; index < constraintCount; index++)
        {
            if (!TryGetExpandedEdgeSweepDistance(
                    constraints,
                    constraintCount,
                    index,
                    planarStart,
                    normalizedDirection,
                    maxDistance,
                    radius,
                    out Fixed64 candidate))
            {
                continue;
            }
            best = FixedMath.Min(best, candidate);
            found = true;
        }

        for (int index = 0; index < vertexCount; index++)
        {
            if (!TryGetRationalPointSweepDistance(
                    vertices[index],
                    planarStart,
                    normalizedDirection,
                    maxDistance,
                    radius,
                    out Fixed64 candidate))
            {
                continue;
            }

            best = FixedMath.Min(best, candidate);
            found = true;
        }

        distance = found ? best : default;
        return found;
    }

    internal static Fixed64 GetCircleSlabSeparationLowerBound(
        Vector3d center,
        FixedQuaternion orientation,
        Vector3d halfExtents,
        Vector3d slabCenter,
        Fixed64 slabHalfThickness,
        Fixed64 radius)
    {
        RationalBasis basis = new(orientation);
        Signed192 lowerY = WideArithmetic.SubtractSigned192(
            Signed192.Raw(slabCenter.Y),
            Signed192.Raw(slabHalfThickness));
        Signed192 upperY = WideArithmetic.AddSigned192(
            Signed192.Raw(slabCenter.Y),
            Signed192.Raw(slabHalfThickness));
        Span<SweepPlanarConstraint> constraints =
            stackalloc SweepPlanarConstraint[16];
        if (!TryBuildCircleSlabProjectionConstraints(
                center,
                halfExtents,
                basis,
                lowerY,
                upperY,
                constraints,
                out int count))
        {
            return GetCircleSlabVerticalSeparationLowerBound(
                center,
                halfExtents,
                basis,
                lowerY,
                upperY);
        }

        Vector2d planarCenter = new(slabCenter.X, slabCenter.Z);
        Fixed64 lowerBound = Fixed64.Zero;
        bool hasPlanarViolation = false;
        bool hasClosestEdgeFeature = false;
        for (int index = 0; index < count; index++)
        {
            SweepPlanarConstraint constraint = constraints[index];
            Signed576 violation = WideArithmetic.SubtractSigned576(
                GetSweepProjection(constraint, planarCenter),
                constraint.K);
            if (violation.Sign <= 0)
                continue;

            hasPlanarViolation = true;
            Signed576 normalSquared = WideArithmetic.AddSigned576(
                WideArithmetic.MultiplySigned320(
                    constraint.A,
                    constraint.A),
                WideArithmetic.MultiplySigned320(
                    constraint.C,
                    constraint.C));
            Signed576 normalFloor = WideArithmetic.GetFloorSquareRoot(
                Signed704.ExtendValue(normalSquared));
            Signed832 floorSquared =
                WideArithmetic.MultiplySigned576ToSigned832(
                    normalFloor,
                    normalFloor);
            Signed832 remainderSign = WideArithmetic.SubtractSigned832(
                floorSquared,
                Signed832.ExtendValue(normalSquared));
            long ceilingIncrementRaw =
                (long)(remainderSign.Word12 >> 63);
            normalFloor = WideArithmetic.AddSigned576(
                normalFloor,
                Signed576.ExtendValue(
                    Signed320.ExtendValue(
                        Signed192.Signed(
                            ceilingIncrementRaw))));

            Fixed64 axisDistance = Fixed64.GetNonNegativeRawRatioFloor(
                Signed704.ExtendValue(violation),
                Signed704.ExtendValue(normalFloor));
            Fixed64 candidate = axisDistance > radius
                ? axisDistance - radius
                : Fixed64.Zero;
            if (candidate > lowerBound)
                lowerBound = candidate;

            if (IsClosestProjectionOnConstraint(
                    constraints,
                    count,
                    index,
                    planarCenter,
                    violation,
                    Signed320.NarrowValue(normalSquared)))
            {
                hasClosestEdgeFeature = true;
                // This valid orthogonal projection is the closest point of
                // the convex projection. Every remaining half-space distance
                // is therefore bounded by the candidate already retained.
                break;
            }
        }

        if (hasPlanarViolation && !hasClosestEdgeFeature)
        {
            Fixed64 vertexDistance =
                GetClosestProjectionVertexDistanceLowerBound(
                    center,
                    halfExtents,
                    basis,
                    lowerY,
                    upperY,
                    planarCenter);
            Fixed64 vertexGap = vertexDistance > radius
                ? vertexDistance - radius
                : Fixed64.Zero;
            if (vertexGap > lowerBound)
                lowerBound = vertexGap;
        }

        return lowerBound;
    }

    private static bool IsClosestProjectionOnConstraint(
        ReadOnlySpan<SweepPlanarConstraint> constraints,
        int constraintCount,
        int constraintIndex,
        Vector2d point,
        Signed576 violation,
        Signed320 normalSquared)
    {
        SweepPlanarConstraint constraint = constraints[constraintIndex];
        for (int index = 0; index < constraintCount; index++)
        {
            SweepPlanarConstraint candidate = constraints[index];
            Signed576 candidateViolation =
                WideArithmetic.SubtractSigned576(
                    GetSweepProjection(candidate, point),
                    candidate.K);
            Signed320 normalDot = Signed320.NarrowValue(
                WideArithmetic.AddSigned576(
                    WideArithmetic.MultiplySigned320(
                        candidate.A,
                        constraint.A),
                    WideArithmetic.MultiplySigned320(
                        candidate.C,
                        constraint.C)));
            Signed704 projectedViolation =
                WideArithmetic.SubtractSigned704(
                    WideArithmetic.MultiplySigned576ToSigned704(
                        candidateViolation,
                        normalSquared),
                    WideArithmetic.MultiplySigned576ToSigned704(
                        violation,
                        normalDot));
            if (projectedViolation.Sign > 0)
                return false;
        }

        return true;
    }

    private static Fixed64 GetClosestProjectionVertexDistanceLowerBound(
        Vector3d center,
        Vector3d halfExtents,
        RationalBasis basis,
        Signed192 lowerY,
        Signed192 upperY,
        Vector2d point)
    {
        Span<SweepRationalPoint> vertices =
            stackalloc SweepRationalPoint[32];
        BuildCircleSlabProjectionVertices(
            center,
            halfExtents,
            basis,
            lowerY,
            upperY,
            vertices,
            out int vertexCount);
        // The caller already proved that the finite box/slab intersection is
        // nonempty, so its compact planar projection has at least one vertex.
        RationalPointDistance closest =
            GetRationalPointDistance(vertices[0], point);
        for (int index = 1; index < vertexCount; index++)
        {
            RationalPointDistance candidate =
                GetRationalPointDistance(vertices[index], point);
            if (CompareRationalPointDistances(candidate, closest) < 0)
                closest = candidate;
        }

        return GetRationalPointDistanceLowerBound(closest);
    }

    private static RationalPointDistance GetRationalPointDistance(
        SweepRationalPoint point,
        Vector2d reference)
    {
        Signed576 deltaX = WideArithmetic.SubtractSigned576(
            WideArithmetic.MultiplySigned576(
                point.Denominator,
                Signed192.Raw(reference.X)),
            point.X);
        Signed576 deltaZ = WideArithmetic.SubtractSigned576(
            WideArithmetic.MultiplySigned576(
                point.Denominator,
                Signed192.Raw(reference.Y)),
            point.Z);
        Signed832 squaredDistance = WideArithmetic.AddSigned832(
            WideArithmetic.MultiplySigned576ToSigned832(deltaX, deltaX),
            WideArithmetic.MultiplySigned576ToSigned832(deltaZ, deltaZ));
        return new RationalPointDistance(
            point.Denominator,
            squaredDistance);
    }

    private static int CompareRationalPointDistances(
        RationalPointDistance first,
        RationalPointDistance second)
    {
        Signed832 firstDenominatorSquared =
            WideArithmetic.MultiplySigned576ToSigned832(
                first.Denominator,
                first.Denominator);
        Signed832 secondDenominatorSquared =
            WideArithmetic.MultiplySigned576ToSigned832(
                second.Denominator,
                second.Denominator);
        Span<ulong> firstSquaredMagnitude = stackalloc ulong[13];
        Span<ulong> secondSquaredMagnitude = stackalloc ulong[13];
        Span<ulong> firstDenominatorMagnitude = stackalloc ulong[13];
        Span<ulong> secondDenominatorMagnitude = stackalloc ulong[13];
        WideArithmetic.GetMagnitude(
            first.SquaredDistance,
            firstSquaredMagnitude);
        WideArithmetic.GetMagnitude(
            second.SquaredDistance,
            secondSquaredMagnitude);
        WideArithmetic.GetMagnitude(
            firstDenominatorSquared,
            firstDenominatorMagnitude);
        WideArithmetic.GetMagnitude(
            secondDenominatorSquared,
            secondDenominatorMagnitude);
        Span<ulong> left =
            stackalloc ulong[TriangleSweepMagnitudeWords];
        Span<ulong> right =
            stackalloc ulong[TriangleSweepMagnitudeWords];
        MultiplyMagnitudes(
            firstSquaredMagnitude,
            secondDenominatorMagnitude,
            left);
        MultiplyMagnitudes(
            secondSquaredMagnitude,
            firstDenominatorMagnitude,
            right);
        return CompareMagnitudes(left, right);
    }

    private static Fixed64 GetRationalPointDistanceLowerBound(
        RationalPointDistance distance)
    {
        long low = 0L;
        long high = Fixed64.MaxValue.m_rawValue;
        while (low < high)
        {
            long difference = high - low;
            long middle =
                low + (difference >> 1) + (difference & 1L);
            Signed576 scaledDistance = WideArithmetic.MultiplySigned576(
                distance.Denominator,
                Signed192.Signed(middle));
            Signed832 squaredCandidate =
                WideArithmetic.MultiplySigned576ToSigned832(
                    scaledDistance,
                    scaledDistance);
            if (WideArithmetic.SubtractSigned832(
                    squaredCandidate,
                    distance.SquaredDistance).Sign <= 0)
            {
                low = middle;
            }
            else
            {
                high = middle - 1L;
            }
        }

        return Fixed64.FromRaw(low);
    }

    private static Fixed64 GetCircleSlabVerticalSeparationLowerBound(
        Vector3d center,
        Vector3d halfExtents,
        RationalBasis basis,
        Signed192 lowerY,
        Signed192 upperY)
    {
        Signed320 boxRadius = GetExtentNumerator(
            basis.Xy,
            basis.Yy,
            basis.Zy,
            halfExtents);
        Signed320 boxCenter = WideArithmetic.MultiplySigned192(
            Signed192.Raw(center.Y),
            basis.Denominator);
        Signed320 boxMinimum =
            WideArithmetic.SubtractSigned320(boxCenter, boxRadius);
        Signed320 boxMaximum =
            WideArithmetic.AddSigned320(boxCenter, boxRadius);
        Signed320 slabMinimum =
            WideArithmetic.MultiplySigned192(lowerY, basis.Denominator);
        Signed320 slabMaximum =
            WideArithmetic.MultiplySigned192(upperY, basis.Denominator);
        Signed320 gap = CompareSigned(boxMaximum, slabMinimum) < 0
            ? WideArithmetic.SubtractSigned320(slabMinimum, boxMaximum)
            : WideArithmetic.SubtractSigned320(boxMinimum, slabMaximum);
        return Fixed64.GetNonNegativeRawRatioFloor(
            Signed704.ExtendValue(
                Signed576.ExtendValue(gap)),
            Signed704.ExtendValue(
                Signed576.ExtendValue(
                    Signed320.ExtendValue(
                        basis.Denominator))));
    }

    private static bool TryGetExpandedEdgeSweepDistance(
        ReadOnlySpan<SweepPlanarConstraint> constraints,
        int constraintCount,
        int constraintIndex,
        Vector2d start,
        Vector2d direction,
        Fixed64 maxDistance,
        Fixed64 radius,
        out Fixed64 distance)
    {
        SweepPlanarConstraint edge = constraints[constraintIndex];
        Signed576 startViolation = WideArithmetic.SubtractSigned576(
            GetSweepProjection(edge, start),
            edge.K);
        if (startViolation.Sign <= 0)
        {
            distance = default;
            return false;
        }

        Signed576 normalSquared = WideArithmetic.AddSigned576(
            WideArithmetic.MultiplySigned320(edge.A, edge.A),
            WideArithmetic.MultiplySigned320(edge.C, edge.C));
        Signed320 radiusSquared =
            WideArithmetic.MultiplySigned192(Signed192.Raw(radius), Signed192.Raw(radius));
        Signed832 expandedSquared =
            WideArithmetic.MultiplyNonNegativeToSigned832(
                Signed832.ExtendValue(
                    Signed576.ExtendValue(radiusSquared)),
                Signed320.NarrowValue(normalSquared));
        Signed832 startSquared =
            WideArithmetic.MultiplySigned576ToSigned832(
                startViolation,
                startViolation);
        if (WideArithmetic.SubtractSigned832(
                startSquared,
                expandedSquared).Sign <= 0)
        {
            distance = default;
            return false;
        }

        Signed576 velocity = GetSweepProjection(edge, direction);
        if (velocity.Sign >= 0
            || CompareExpandedEdgeAt(
                startViolation,
                velocity,
                normalSquared,
                radius,
                Signed192.Signed(maxDistance.m_rawValue),
                SweepRawScale) > 0)
        {
            distance = default;
            return false;
        }

        long maxRaw = maxDistance.m_rawValue;
        // The public contract has already rejected a zero maximum distance.
        long high = maxRaw - 1L;
        if (CompareExpandedEdgeAtHalf(
                startViolation,
                velocity,
                normalSquared,
                radius,
                high) > 0)
        {
            distance = maxDistance;
        }
        else
        {
            long low = 0L;
            while (low < high)
            {
                long middle = low + ((high - low) >> 1);
                if (CompareExpandedEdgeAtHalf(
                        startViolation,
                        velocity,
                        normalSquared,
                        radius,
                        middle) <= 0)
                {
                    high = middle;
                }
                else
                {
                    low = middle + 1L;
                }
            }

            int midpointComparison = CompareExpandedEdgeAtHalf(
                startViolation,
                velocity,
                normalSquared,
                radius,
                low);
            distance = Fixed64.FromRaw(
                low + GetHalfToEvenIncrement(midpointComparison, low));
        }

        return IsRoundedEdgeFeatureValid(
            constraints,
            constraintCount,
            constraintIndex,
            start,
            direction,
            distance,
            radius);
    }

    private static int CompareExpandedEdgeAtHalf(
        Signed576 startViolation,
        Signed576 velocity,
        Signed576 normalSquared,
        Fixed64 radius,
        long lowerRaw)
    {
        Signed192 lower = Signed192.Signed(lowerRaw);
        Signed192 midpoint = WideArithmetic.AddSigned192(
            WideArithmetic.AddSigned192(lower, lower),
            SweepOneCoefficient);
        return CompareExpandedEdgeAt(
            startViolation,
            velocity,
            normalSquared,
            radius,
            midpoint,
            SweepDoubleRawScale);
    }

    private static int CompareExpandedEdgeAt(
        Signed576 startViolation,
        Signed576 velocity,
        Signed576 normalSquared,
        Fixed64 radius,
        Signed192 timeNumerator,
        Signed192 timeDenominator)
    {
        Signed576 left = WideArithmetic.AddSigned576(
            WideArithmetic.MultiplySigned576(
                startViolation,
                timeDenominator),
            WideArithmetic.MultiplySigned576(
                velocity,
                timeNumerator));
        if (left.Sign <= 0)
            return -1;

        Signed832 leftSquared =
            WideArithmetic.MultiplySigned576ToSigned832(left, left);
        Signed320 scaledRadius = WideArithmetic.MultiplySigned192(
            Signed192.Raw(radius),
            timeDenominator);
        Signed832 rightSquared =
            WideArithmetic.MultiplyNonNegativeToSigned832(
                Signed832.ExtendValue(
                    WideArithmetic.MultiplySigned320(
                        scaledRadius,
                        scaledRadius)),
                Signed320.NarrowValue(normalSquared));
        return WideArithmetic.SubtractSigned832(
            leftSquared,
            rightSquared).Sign;
    }

    private static bool IsRoundedEdgeFeatureValid(
        ReadOnlySpan<SweepPlanarConstraint> constraints,
        int constraintCount,
        int constraintIndex,
        Vector2d start,
        Vector2d direction,
        Fixed64 distance,
        Fixed64 radius)
    {
        SweepPlanarConstraint edge = constraints[constraintIndex];
        Vector2d normal = WideGeometry.GetNormalized(
            Signed576.ExtendValue(edge.A),
            Signed576.ExtendValue(edge.C));
        Vector2d radial = normal * radius;
        for (int index = 0; index < constraintCount; index++)
        {
            SweepPlanarConstraint candidate = constraints[index];
            Signed576 startViolation = WideArithmetic.SubtractSigned576(
                WideArithmetic.SubtractSigned576(
                    GetSweepProjection(candidate, start),
                    GetSweepProjection(candidate, radial)),
                candidate.K);
            Signed576 violation = WideArithmetic.AddSigned576(
                WideArithmetic.MultiplySigned576(
                    startViolation,
                    SweepRawScale),
                WideArithmetic.MultiplySigned576(
                    GetSweepProjection(candidate, direction),
                    Signed192.Raw(distance)));
            Signed320 coefficientMagnitude = WideArithmetic.AddSigned320(
                GetMagnitude(candidate.A),
                GetMagnitude(candidate.C));
            Signed576 oneRawTolerance = WideArithmetic.MultiplySigned576(
                WideArithmetic.MultiplySigned320(
                    coefficientMagnitude,
                    Signed320.ExtendValue(
                        Signed192.Raw(Fixed64.MinIncrement))),
                SweepRawScale);
            if (CompareSigned(violation, oneRawTolerance) > 0)
                return false;
        }

        return true;
    }

    private static bool TryGetRationalPointSweepDistance(
        SweepRationalPoint point,
        Vector2d start,
        Vector2d direction,
        Fixed64 maxDistance,
        Fixed64 radius,
        out Fixed64 distance)
    {
        Signed576 deltaX = WideArithmetic.SubtractSigned576(
            WideArithmetic.MultiplySigned576(
                point.Denominator,
                Signed192.Raw(start.X)),
            point.X);
        Signed576 deltaZ = WideArithmetic.SubtractSigned576(
            WideArithmetic.MultiplySigned576(
                point.Denominator,
                Signed192.Raw(start.Y)),
            point.Z);
        Signed576 directionX = WideArithmetic.MultiplySigned576(
            point.Denominator,
            Signed192.Raw(direction.X));
        Signed576 directionZ = WideArithmetic.MultiplySigned576(
            point.Denominator,
            Signed192.Raw(direction.Y));
        Signed576 radiusNumerator = WideArithmetic.MultiplySigned576(
            point.Denominator,
            Signed192.Raw(radius));
        Signed832 constant = WideArithmetic.SubtractSigned832(
            WideArithmetic.AddSigned832(
                WideArithmetic.MultiplySigned576ToSigned832(deltaX, deltaX),
                WideArithmetic.MultiplySigned576ToSigned832(deltaZ, deltaZ)),
            WideArithmetic.MultiplySigned576ToSigned832(
                radiusNumerator,
                radiusNumerator));
        Signed832 projection = WideArithmetic.AddSigned832(
            WideArithmetic.MultiplySigned576ToSigned832(
                deltaX,
                directionX),
            WideArithmetic.MultiplySigned576ToSigned832(
                deltaZ,
                directionZ));
        // Triangle-sweep callers share this vertex solver and can reach it
        // without a whole-shape start-overlap precheck.
        if (constant.Sign <= 0)
        {
            distance = Fixed64.Zero;
            return true;
        }
        if (projection.Sign >= 0)
        {
            distance = default;
            return false;
        }

        Signed832 directionSquared = WideArithmetic.AddSigned832(
            WideArithmetic.MultiplySigned576ToSigned832(
                directionX,
                directionX),
            WideArithmetic.MultiplySigned576ToSigned832(
                directionZ,
                directionZ));
        Signed832 negativeProjection =
            WideArithmetic.SubtractSigned832(default, projection);
        bool closestBeyondMaximum =
            WideArithmetic.CompareNonNegativeProducts(
                negativeProjection,
                Signed832.ExtendValue(SweepRawScale),
                directionSquared,
                Signed832.ExtendValue(Signed192.Signed(maxDistance.m_rawValue))) > 0;
        if (closestBeyondMaximum)
        {
            if (CompareRationalPointAt(
                    deltaX,
                    deltaZ,
                    directionX,
                    directionZ,
                    radiusNumerator,
                    Signed192.Signed(maxDistance.m_rawValue),
                    SweepRawScale) > 0)
            {
                distance = default;
                return false;
            }
        }
        else if (WideArithmetic.CompareNonNegativeProducts(
                negativeProjection,
                negativeProjection,
                directionSquared,
                constant) < 0)
        {
            distance = default;
            return false;
        }

        Fixed64 closest = maxDistance;
        if (!closestBeyondMaximum)
        {
            // The exact comparison above proves this nonnegative quotient is
            // no greater than maxDistance, so its final Q32.32 value is
            // representable.
            _ = Fixed64.TryGetSignedRawRatio(
                negativeProjection,
                directionSquared,
                FixedMath.SHIFT_AMOUNT_I,
                out closest);
        }

        long maximumSearchRaw = closest.m_rawValue;
        long high = Math.Max(0L, maximumSearchRaw - 1L);
        if (CompareRationalPointAtHalf(
                deltaX,
                deltaZ,
                directionX,
                directionZ,
                radiusNumerator,
                high) > 0)
        {
            distance = closestBeyondMaximum ? maxDistance : closest;
            return true;
        }

        long low = 0L;
        while (low < high)
        {
            long middle = low + ((high - low) >> 1);
            if (CompareRationalPointAtHalf(
                    deltaX,
                    deltaZ,
                    directionX,
                    directionZ,
                    radiusNumerator,
                    middle) <= 0)
            {
                high = middle;
            }
            else
            {
                low = middle + 1L;
            }
        }

        int midpointComparison = CompareRationalPointAtHalf(
            deltaX,
            deltaZ,
            directionX,
            directionZ,
            radiusNumerator,
            low);
        distance = Fixed64.FromRaw(
            low + GetHalfToEvenIncrement(midpointComparison, low));
        return true;
    }

    private static long GetHalfToEvenIncrement(
        int midpointComparison,
        long lowerRaw)
    {
        int nonzero =
            (int)((uint)(midpointComparison | -midpointComparison) >> 31);
        int zero = nonzero ^ 1;
        return zero & (lowerRaw & 1L);
    }

    private static int CompareRationalPointAtHalf(
        Signed576 deltaX,
        Signed576 deltaZ,
        Signed576 directionX,
        Signed576 directionZ,
        Signed576 radiusNumerator,
        long lowerRaw)
    {
        Signed192 lower = Signed192.Signed(lowerRaw);
        Signed192 midpoint = WideArithmetic.AddSigned192(
            WideArithmetic.AddSigned192(lower, lower),
            SweepOneCoefficient);
        return CompareRationalPointAt(
            deltaX,
            deltaZ,
            directionX,
            directionZ,
            radiusNumerator,
            midpoint,
            SweepDoubleRawScale);
    }

    private static int CompareRationalPointAt(
        Signed576 deltaX,
        Signed576 deltaZ,
        Signed576 directionX,
        Signed576 directionZ,
        Signed576 radiusNumerator,
        Signed192 timeNumerator,
        Signed192 timeDenominator)
    {
        Signed576 x = WideArithmetic.AddSigned576(
            WideArithmetic.MultiplySigned576(deltaX, timeDenominator),
            WideArithmetic.MultiplySigned576(directionX, timeNumerator));
        Signed576 z = WideArithmetic.AddSigned576(
            WideArithmetic.MultiplySigned576(deltaZ, timeDenominator),
            WideArithmetic.MultiplySigned576(directionZ, timeNumerator));
        Signed576 scaledRadius =
            WideArithmetic.MultiplySigned576(
                radiusNumerator,
                timeDenominator);
        return WideArithmetic.SubtractSigned832(
            WideArithmetic.AddSigned832(
                WideArithmetic.MultiplySigned576ToSigned832(x, x),
                WideArithmetic.MultiplySigned576ToSigned832(z, z)),
            WideArithmetic.MultiplySigned576ToSigned832(
                scaledRadius,
                scaledRadius)).Sign;
    }

    private static Signed576 GetSweepProjection(
        SweepPlanarConstraint constraint,
        Vector2d point) =>
        WideArithmetic.AddSigned576(
            WideArithmetic.MultiplySigned320(
                constraint.A,
                Signed320.ExtendValue(Signed192.Raw(point.X))),
            WideArithmetic.MultiplySigned320(
                constraint.C,
                Signed320.ExtendValue(Signed192.Raw(point.Y))));
}
