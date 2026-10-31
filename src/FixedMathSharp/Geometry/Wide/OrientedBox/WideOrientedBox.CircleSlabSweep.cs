//=======================================================================
// WideOrientedBox.CircleSlabSweep.cs
//=======================================================================
// MIT License, Copyright (c) 2024-present David Oravsky (mrdav30)
// See LICENSE file in the project root for full license information.
//=======================================================================

using System;

namespace FixedMathSharp.Geometry;

/// <content>
/// High-precision sweep tests between a circle and an oriented box slab,
/// using wide fixed-point arithmetic to avoid overflow and precision loss.
/// </content>
internal static partial class WideOrientedBox
{
    private static readonly Signed192 SweepRawScale = Signed192.Signed(FixedMath.ONE_L);
    private static readonly Signed192 SweepOneCoefficient = Signed192.Signed(1L);

    #region Nested Types

    private readonly struct SweepTrajectory
    {
        internal readonly Signed192 DirectionX;
        internal readonly Signed192 DirectionZ;
        internal readonly Signed192 TimeScale;
        internal readonly bool IsSegment;

        internal SweepTrajectory(Vector2d direction)
        {
            DirectionX = Signed192.Raw(direction.X);
            DirectionZ = Signed192.Raw(direction.Y);
            TimeScale = SweepRawScale;
            IsSegment = false;
        }

        internal SweepTrajectory(FixedSegment2d segment, Fixed64 totalDistance)
        {
            DirectionX = WideArithmetic.SubtractSigned192(Signed192.Raw(segment.End.X), Signed192.Raw(segment.Start.X));
            DirectionZ = WideArithmetic.SubtractSigned192(Signed192.Raw(segment.End.Y), Signed192.Raw(segment.Start.Y));
            TimeScale = totalDistance == Fixed64.Zero ? SweepRawScale : Signed192.Raw(totalDistance);
            IsSegment = true;
        }

        internal Signed192 DoubleTimeScale => WideArithmetic.AddSigned192(TimeScale, TimeScale);
    }

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
        => TryGetCircleSlabSweepDistanceCore(
            center, orientation, halfExtents, slabStartCenter,
            new SweepTrajectory(normalizedDirection), maxDistance, slabHalfThickness, radius, out distance);

    internal static bool TryGetCircleSlabSweepDistance(
        Vector3d center,
        FixedQuaternion orientation,
        Vector3d halfExtents,
        FixedSegment2d segment,
        Fixed64 totalDistance,
        Fixed64 slabCenterY,
        Fixed64 slabHalfThickness,
        Fixed64 radius,
        out Fixed64 distance)
        => TryGetCircleSlabSweepDistanceCore(
            center, orientation, halfExtents,
            new Vector3d(segment.Start.X, slabCenterY, segment.Start.Y),
            new SweepTrajectory(segment, totalDistance), totalDistance, slabHalfThickness, radius, out distance);

    private static bool TryGetCircleSlabSweepDistanceCore(
        Vector3d center,
        FixedQuaternion orientation,
        Vector3d halfExtents,
        Vector3d slabStartCenter,
        in SweepTrajectory trajectory,
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

        WideRationalBasis3d basis = new(orientation);
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
                    trajectory,
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
                    trajectory,
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
        WideRationalBasis3d basis = new(orientation);
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
        WideRationalBasis3d basis,
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
        WideArithmetic.MultiplyMagnitudes(
            firstSquaredMagnitude,
            secondDenominatorMagnitude,
            left);
        WideArithmetic.MultiplyMagnitudes(
            secondSquaredMagnitude,
            firstDenominatorMagnitude,
            right);
        return WideArithmetic.CompareMagnitudeEqualLength(left, right);
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
        WideRationalBasis3d basis,
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
        in SweepTrajectory trajectory,
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

        Signed576 velocity = GetSweepProjection(edge, trajectory);
        if (velocity.Sign >= 0
            || CompareExpandedEdgeAt(
                startViolation,
                velocity,
                normalSquared,
                radius,
                Signed192.Signed(maxDistance.m_rawValue),
                trajectory.TimeScale) > 0)
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
                high, trajectory) > 0)
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
                        middle, trajectory) <= 0)
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
                low, trajectory);
            distance = Fixed64.FromRaw(
                low + GetHalfToEvenIncrement(midpointComparison, low));
        }

        return IsExactEdgeFeatureValid(
            constraints, constraintCount, constraintIndex, start,
            trajectory, startViolation, velocity, normalSquared, radius);
    }

    private static int CompareExpandedEdgeAtHalf(
        Signed576 startViolation,
        Signed576 velocity,
        Signed576 normalSquared,
        Fixed64 radius,
        long lowerRaw,
        in SweepTrajectory trajectory)
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
            trajectory.DoubleTimeScale);
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
            return left.IsZero && radius == Fixed64.Zero ? 0 : -1;

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

    private static bool IsExactEdgeFeatureValid(
        ReadOnlySpan<SweepPlanarConstraint> constraints,
        int constraintCount,
        int constraintIndex,
        Vector2d start,
        in SweepTrajectory trajectory,
        Signed576 edgeStartViolation,
        Signed576 edgeVelocity,
        Signed576 edgeNormalSquared,
        Fixed64 radius)
    {
        SweepPlanarConstraint edge = constraints[constraintIndex];
        for (int index = 0; index < constraintCount; index++)
        {
            SweepPlanarConstraint candidate = constraints[index];
            Signed576 startViolation = WideArithmetic.SubtractSigned576(GetSweepProjection(candidate, start), candidate.K);
            Signed576 velocity = GetSweepProjection(candidate, trajectory);
            Signed576 normalDot = WideArithmetic.AddSigned576(
                WideArithmetic.MultiplySigned320(candidate.A, edge.A),
                WideArithmetic.MultiplySigned320(candidate.C, edge.C));
            Signed704 radicalCoefficient = WideArithmetic.SubtractSigned704(
                WideArithmetic.MultiplySigned576ToSigned704(edgeStartViolation, Signed320.NarrowValue(velocity)),
                WideArithmetic.MultiplySigned576ToSigned704(startViolation, Signed320.NarrowValue(edgeVelocity)));
            Signed832 rational = WideArithmetic.MultiplySigned832(
                WideArithmetic.SubtractSigned832(
                    WideArithmetic.MultiplySigned576ToSigned832(edgeVelocity, normalDot),
                    WideArithmetic.MultiplySigned576ToSigned832(velocity, edgeNormalSquared)),
                Signed192.Raw(radius));
            // At the exact edge root, the boundary-foot violation has sign
            // (F*V-E*W)*sqrt(L) + R*(E*H-F*L); the time denominator cancels.
            // Normalized frames bound planar normals to 134 bits, velocities
            // to 200, the radical coefficient to 402 and the R-term to 535.
            if (WideArithmetic.CompareSignedLinearRadicalToZero(
                    rational, radicalCoefficient, edgeNormalSquared,
                    Signed320.ExtendValue(SweepOneCoefficient)) > 0)
                return false;
        }

        return true;
    }

    private static bool TryGetRationalPointSweepDistance(
        SweepRationalPoint point,
        Vector2d start,
        in SweepTrajectory trajectory,
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
            trajectory.DirectionX);
        Signed576 directionZ = WideArithmetic.MultiplySigned576(
            point.Denominator,
            trajectory.DirectionZ);
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
                Signed832.ExtendValue(trajectory.TimeScale),
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
                    trajectory.TimeScale, trajectory.IsSegment) > 0)
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

        // Entry is (-B-sqrt(B*B-A*C))*TimeScale/A. Two 832-bit products
        // fit 27 limbs; shared quadratic rounding retains exact nearest-even ties.
        const int words = 27;
        Span<ulong> values = stackalloc ulong[6 * words];
        Span<int> signs = stackalloc int[6];
        ContactQuadratic entry = ContactQuadratic.At(values, signs, 0, words);
        ContactQuadratic divisor = ContactQuadratic.At(values, signs, 1, words);
        ContactQuadratic work = ContactQuadratic.At(values, signs, 2, words);
        entry.Clear(); divisor.Clear(); work.Clear();
        WideArithmetic.GetMagnitude(negativeProjection, entry.Rational[..13]); entry.Signs[0] = 1;
        WideArithmetic.GetMagnitude(directionSquared, divisor.Rational[..13]); divisor.Signs[0] = 1;
        WideArithmetic.GetMagnitude(constant, work.Rational[..13]);
        Span<ulong> discriminant = stackalloc ulong[words];
        WideArithmetic.MultiplyMagnitudes(entry.Rational, entry.Rational, discriminant);
        WideArithmetic.MultiplyMagnitudes(divisor.Rational, work.Rational, work.Radical);
        WideArithmetic.SubtractEqualMagnitudes(discriminant, work.Radical, discriminant);
        entry.Radical[0] = 1; entry.Signs[1] = -1;
        ContactQuadratic.Scale(entry, Signed576.ExtendValue(Signed320.ExtendValue(trajectory.TimeScale)), work);
        work.CopyTo(entry);
        // Earlier admission bounds this positive entering root by the maximum distance.
        distance = ContactQuadratic.RoundRatio(entry, divisor, discriminant, 0L, maxDistance.m_rawValue);
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

    private static int CompareRationalPointAt(
        Signed576 deltaX,
        Signed576 deltaZ,
        Signed576 directionX,
        Signed576 directionZ,
        Signed576 radiusNumerator,
        Signed192 timeNumerator,
        Signed192 timeDenominator,
        bool isSegment)
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
        if (!isSegment)
        {
            return WideArithmetic.SubtractSigned832(
                WideArithmetic.AddSigned832(
                    WideArithmetic.MultiplySigned576ToSigned832(x, x),
                    WideArithmetic.MultiplySigned576ToSigned832(z, z)),
                WideArithmetic.MultiplySigned576ToSigned832(scaledRadius, scaledRadius)).Sign;
        }

        // A segment-distance denominator can add 30 bits beyond the direction
        // scale; retain the full 1152-bit squares before comparing them.
        Span<ulong> xMagnitude = stackalloc ulong[9];
        Span<ulong> zMagnitude = stackalloc ulong[9];
        Span<ulong> radiusMagnitude = stackalloc ulong[9];
        Span<ulong> xSquared = stackalloc ulong[18];
        Span<ulong> zSquared = stackalloc ulong[18];
        Span<ulong> radiusSquared = stackalloc ulong[18];
        WideArithmetic.GetMagnitude(x, xMagnitude);
        WideArithmetic.GetMagnitude(z, zMagnitude);
        WideArithmetic.GetMagnitude(scaledRadius, radiusMagnitude);
        WideArithmetic.MultiplyMagnitudes(xMagnitude, xMagnitude, xSquared);
        WideArithmetic.MultiplyMagnitudes(zMagnitude, zMagnitude, zSquared);
        WideArithmetic.MultiplyMagnitudes(radiusMagnitude, radiusMagnitude, radiusSquared);
        WideArithmetic.AddEqualMagnitudes(xSquared, zSquared, xSquared);
        return WideArithmetic.CompareMagnitudeEqualLength(xSquared, radiusSquared);
    }

    private static Signed576 GetSweepProjection(SweepPlanarConstraint constraint, in SweepTrajectory trajectory) =>
        WideArithmetic.AddSigned576(
            WideArithmetic.MultiplySigned320(constraint.A, Signed320.ExtendValue(trajectory.DirectionX)),
            WideArithmetic.MultiplySigned320(constraint.C, Signed320.ExtendValue(trajectory.DirectionZ)));

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
