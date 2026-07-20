//=======================================================================
// FixedSegment.Distance.cs
//=======================================================================
// MIT License, Copyright (c) 2024–present David Oravsky (mrdav30)
// See LICENSE file in the project root for full license information.
//=======================================================================

using System;
using System.Runtime.CompilerServices;

namespace FixedMathSharp.Bounds;

public partial struct FixedSegment
{
    /// <summary>
    /// Finds the physical-distance interval where this segment intersects an
    /// endpoint-authored capsule.
    /// </summary>
    public readonly bool TryGetCapsuleIntersectionDistanceInterval(
        FixedSegment capsuleAxis,
        Fixed64 radius,
        Fixed64 totalDistance,
        out Fixed64 entryDistance,
        out Fixed64 exitDistance) =>
        TryGetCapsuleIntersectionDistanceInterval(
            capsuleAxis,
            radius,
            Fixed64.Zero,
            totalDistance,
            out entryDistance,
            out exitDistance,
            out _,
            out _);

    /// <summary>
    /// Finds the physical-distance interval where this segment intersects an
    /// endpoint-authored, radially expanded capsule and reports exact endpoint
    /// containment.
    /// </summary>
    public readonly bool TryGetCapsuleIntersectionDistanceInterval(
        FixedSegment capsuleAxis,
        Fixed64 radius,
        Fixed64 radiusExpansion,
        Fixed64 totalDistance,
        out Fixed64 entryDistance,
        out Fixed64 exitDistance,
        out bool startContained,
        out bool endContainedStrict)
    {
        ValidateTotalDistance(totalDistance);
        if (radius < Fixed64.Zero)
            throw new ArgumentOutOfRangeException(nameof(radius));
        if (radiusExpansion < Fixed64.Zero)
            throw new ArgumentOutOfRangeException(nameof(radiusExpansion));

        return WideFiniteAxisIntersection.TryGetCapsuleDistanceInterval(
            this,
            capsuleAxis,
            radius,
            radiusExpansion,
            totalDistance,
            out entryDistance,
            out exitDistance,
            out startContained,
            out endContainedStrict);
    }

    /// <summary>
    /// Finds the physical-distance interval where this segment intersects a
    /// centered capsule.
    /// </summary>
    public readonly bool TryGetCapsuleIntersectionDistanceInterval(
        Vector3d center,
        Vector3d axisDirection,
        Fixed64 axisHalfLength,
        Fixed64 radius,
        Fixed64 totalDistance,
        out Fixed64 entryDistance,
        out Fixed64 exitDistance) =>
        TryGetCapsuleIntersectionDistanceInterval(
            center,
            axisDirection,
            axisHalfLength,
            radius,
            Fixed64.Zero,
            totalDistance,
            out entryDistance,
            out exitDistance,
            out _,
            out _);

    /// <summary>
    /// Finds the physical-distance interval where this segment intersects a
    /// centered, radially expanded capsule and reports exact endpoint
    /// containment.
    /// </summary>
    public readonly bool TryGetCapsuleIntersectionDistanceInterval(
        Vector3d center,
        Vector3d axisDirection,
        Fixed64 axisHalfLength,
        Fixed64 radius,
        Fixed64 radiusExpansion,
        Fixed64 totalDistance,
        out Fixed64 entryDistance,
        out Fixed64 exitDistance,
        out bool startContained,
        out bool endContainedStrict)
    {
        ValidateTotalDistance(totalDistance);
        ValidateCenteredAxis(axisDirection, axisHalfLength);
        if (radius < Fixed64.Zero)
            throw new ArgumentOutOfRangeException(nameof(radius));
        if (radiusExpansion < Fixed64.Zero)
            throw new ArgumentOutOfRangeException(nameof(radiusExpansion));

        return WideFiniteAxisIntersection.TryGetCapsuleDistanceInterval(
            this,
            center,
            axisDirection,
            axisHalfLength,
            radius,
            radiusExpansion,
            totalDistance,
            out entryDistance,
            out exitDistance,
            out startContained,
            out endContainedStrict);
    }

    /// <summary>
    /// Finds the physical-distance interval where this segment intersects an
    /// endpoint-authored finite cylinder.
    /// </summary>
    public readonly bool TryGetFiniteCylinderIntersectionDistanceInterval(
        FixedSegment cylinderAxis,
        Fixed64 radius,
        Fixed64 totalDistance,
        out Fixed64 entryDistance,
        out Fixed64 exitDistance) =>
        TryGetFiniteCylinderIntersectionDistanceInterval(
            cylinderAxis,
            radius,
            Fixed64.Zero,
            totalDistance,
            out entryDistance,
            out exitDistance,
            out _,
            out _);

    /// <summary>
    /// Finds the physical-distance interval where this segment intersects an
    /// endpoint-authored, radially expanded finite cylinder and reports exact
    /// endpoint containment.
    /// </summary>
    public readonly bool TryGetFiniteCylinderIntersectionDistanceInterval(
        FixedSegment cylinderAxis,
        Fixed64 radius,
        Fixed64 radiusExpansion,
        Fixed64 totalDistance,
        out Fixed64 entryDistance,
        out Fixed64 exitDistance,
        out bool startContained,
        out bool endContainedStrict)
    {
        ValidateTotalDistance(totalDistance);
        if (cylinderAxis.Start == cylinderAxis.End)
            throw new ArgumentException("A finite cylinder axis must have nonzero length.", nameof(cylinderAxis));
        if (radius < Fixed64.Zero)
            throw new ArgumentOutOfRangeException(nameof(radius));
        if (radiusExpansion < Fixed64.Zero)
            throw new ArgumentOutOfRangeException(nameof(radiusExpansion));

        return WideFiniteAxisIntersection.TryGetFiniteCylinderDistanceInterval(
            this,
            cylinderAxis,
            radius,
            radiusExpansion,
            totalDistance,
            out entryDistance,
            out exitDistance,
            out startContained,
            out endContainedStrict);
    }

    /// <summary>
    /// Finds the physical-distance interval where this segment intersects a
    /// centered, affinely expanded finite cylinder and reports exact endpoint
    /// containment.
    /// </summary>
    public readonly bool TryGetFiniteCylinderIntersectionDistanceInterval(
        Vector3d center,
        Vector3d axisDirection,
        Fixed64 axisHalfLength,
        Fixed64 radius,
        Fixed64 radiusExpansion,
        Fixed64 axialExpansion,
        Fixed64 totalDistance,
        out Fixed64 entryDistance,
        out Fixed64 exitDistance,
        out bool startContained,
        out bool endContainedStrict)
    {
        ValidateTotalDistance(totalDistance);
        if (!axisDirection.IsNormalized())
            throw new ArgumentException("Finite cylinder axis direction must be normalized.", nameof(axisDirection));
        if (axisHalfLength <= Fixed64.Zero)
            throw new ArgumentOutOfRangeException(nameof(axisHalfLength));
        if (radius < Fixed64.Zero)
            throw new ArgumentOutOfRangeException(nameof(radius));
        if (radiusExpansion < Fixed64.Zero)
            throw new ArgumentOutOfRangeException(nameof(radiusExpansion));
        if (axialExpansion < Fixed64.Zero)
            throw new ArgumentOutOfRangeException(nameof(axialExpansion));

        return WideFiniteAxisIntersection.TryGetFiniteCylinderDistanceInterval(
            this,
            center,
            axisDirection,
            axisHalfLength,
            radius,
            radiusExpansion,
            axialExpansion,
            totalDistance,
            out entryDistance,
            out exitDistance,
            out startContained,
            out endContainedStrict);
    }

    /// <summary>
    /// Finds the physical-distance interval where this segment intersects an
    /// affinely expanded finite cylinder whose unexpanded cap centers are
    /// supplied by <paramref name="cylinderAxis"/>.
    /// </summary>
    public readonly bool TryGetFiniteCylinderIntersectionDistanceInterval(
        FixedSegment cylinderAxis,
        Fixed64 axisHalfLength,
        Fixed64 radius,
        Fixed64 radiusExpansion,
        Fixed64 axialExpansion,
        Fixed64 totalDistance,
        out Fixed64 entryDistance,
        out Fixed64 exitDistance)
    {
        ValidateTotalDistance(totalDistance);
        if (cylinderAxis.Start == cylinderAxis.End)
            throw new ArgumentException("A finite cylinder axis must have nonzero length.", nameof(cylinderAxis));
        if (axisHalfLength <= Fixed64.Zero)
            throw new ArgumentOutOfRangeException(nameof(axisHalfLength));
        if (radius < Fixed64.Zero)
            throw new ArgumentOutOfRangeException(nameof(radius));
        if (radiusExpansion < Fixed64.Zero)
            throw new ArgumentOutOfRangeException(nameof(radiusExpansion));
        if (axialExpansion < Fixed64.Zero)
            throw new ArgumentOutOfRangeException(nameof(axialExpansion));

        return WideFiniteAxisIntersection.TryGetFiniteCylinderDistanceInterval(
            this,
            cylinderAxis,
            axisHalfLength,
            radius,
            radiusExpansion,
            axialExpansion,
            totalDistance,
            out entryDistance,
            out exitDistance);
    }

    /// <summary>
    /// Reconstructs the point at an authored physical distance along this
    /// segment using one exact chord interpolation per coordinate.
    /// </summary>
    /// <remarks>
    /// This method does not normalize <see cref="Delta"/>. It therefore retains
    /// representable chord components that would round away in a normalized
    /// direction. The final coordinates use round-half-to-even.
    /// </remarks>
    /// <exception cref="ArgumentOutOfRangeException">
    /// Thrown when <paramref name="totalDistance"/> is negative, or when
    /// <paramref name="distance"/> is outside [0, <paramref name="totalDistance"/>].
    /// </exception>
    /// <exception cref="ArgumentException">
    /// Thrown when <paramref name="totalDistance"/> is zero and this segment is
    /// not a point.
    /// </exception>
    public readonly Vector3d GetPointAtDistance(Fixed64 distance, Fixed64 totalDistance)
    {
        ValidateDistance(distance, totalDistance);
        if (distance == Fixed64.Zero)
            return Start;
        if (distance == totalDistance)
            return End;

        return new Vector3d(
            GetCoordinateAtDistance(Start.X, End.X, distance, totalDistance),
            GetCoordinateAtDistance(Start.Y, End.Y, distance, totalDistance),
            GetCoordinateAtDistance(Start.Z, End.Z, distance, totalDistance));
    }

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    private readonly void ValidateDistance(Fixed64 distance, Fixed64 totalDistance)
    {
        ValidateTotalDistance(totalDistance);
        if (distance < Fixed64.Zero || distance > totalDistance)
            throw new ArgumentOutOfRangeException(nameof(distance));
    }

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    private readonly void ValidateTotalDistance(Fixed64 totalDistance)
    {
        if (totalDistance < Fixed64.Zero)
            throw new ArgumentOutOfRangeException(nameof(totalDistance));
        if (totalDistance == Fixed64.Zero && Start != End)
            throw new ArgumentException("Zero total distance requires a zero-length segment.", nameof(totalDistance));
    }

    private static Fixed64 GetCoordinateAtDistance(
        Fixed64 start,
        Fixed64 end,
        Fixed64 distance,
        Fixed64 totalDistance)
    {
        if (start == end)
            return start;

        Signed192 distanceRaw = WideArithmetic.FromSignedRaw(distance.m_rawValue);
        Signed192 remainingRaw = WideArithmetic.FromSignedRaw(
            totalDistance.m_rawValue - distance.m_rawValue);
        Signed320 numerator = WideArithmetic.AddSigned320(
            WideArithmetic.MultiplySigned192(
                WideArithmetic.FromSignedRaw(start.m_rawValue),
                remainingRaw),
            WideArithmetic.MultiplySigned192(
                WideArithmetic.FromSignedRaw(end.m_rawValue),
                distanceRaw));
        Signed576 denominator = WideArithmetic.ExtendToSigned576(
            WideArithmetic.FromSignedRaw(totalDistance.m_rawValue));
        Fixed64.TryGetSignedRawRatio(
            WideArithmetic.ExtendToSigned576(numerator),
            denominator,
            out Fixed64 coordinate);
        return coordinate;
    }
}
