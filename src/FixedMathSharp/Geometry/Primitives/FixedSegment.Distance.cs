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
    /// Finds the first physical distance where this segment intersects the
    /// exact spherical dilation of a centered finite cylinder.
    /// </summary>
    /// <param name="center">Cylinder center.</param>
    /// <param name="axisDirection">Normalized cylinder axis direction.</param>
    /// <param name="axisHalfLength">Positive unexpanded cylinder half-length.</param>
    /// <param name="radius">Nonnegative unexpanded cylinder radius.</param>
    /// <param name="sphericalExpansion">Nonnegative spherical dilation radius.</param>
    /// <param name="totalDistance">Nonnegative physical length represented by this segment.</param>
    /// <param name="distance">First intersection distance when one exists.</param>
    /// <exception cref="ArgumentException">
    /// Thrown when <paramref name="axisDirection"/> is not normalized, or when
    /// <paramref name="totalDistance"/> is zero for a non-point segment.
    /// </exception>
    /// <exception cref="ArgumentOutOfRangeException">
    /// Thrown when the half-length is not positive, or when the radius,
    /// expansion, or total distance is negative.
    /// </exception>
    public readonly bool TryGetSweptSphereFiniteCylinderIntersectionDistance(
        Vector3d center,
        Vector3d axisDirection,
        Fixed64 axisHalfLength,
        Fixed64 radius,
        Fixed64 sphericalExpansion,
        Fixed64 totalDistance,
        out Fixed64 distance)
    {
        ValidateSphericallyExpandedFiniteCylinderArguments(
            axisDirection,
            axisHalfLength,
            radius,
            sphericalExpansion,
            totalDistance);
        return WideFiniteAxisIntersection.TryGetSphericallyExpandedFiniteCylinderFirstDistance(
            this,
            center,
            axisDirection,
            axisHalfLength,
            radius,
            sphericalExpansion,
            totalDistance,
            out distance);
    }

    /// <summary>
    /// Finds the physical-distance interval where this segment intersects the
    /// exact spherical dilation of a centered finite cylinder.
    /// </summary>
    /// <remarks>
    /// Unlike affine radial and axial expansion, spherical dilation preserves
    /// the cylinder's rounded cap rims. Final distances use round-half-to-even.
    /// </remarks>
    /// <param name="center">Cylinder center.</param>
    /// <param name="axisDirection">Normalized cylinder axis direction.</param>
    /// <param name="axisHalfLength">Positive unexpanded cylinder half-length.</param>
    /// <param name="radius">Nonnegative unexpanded cylinder radius.</param>
    /// <param name="sphericalExpansion">Nonnegative spherical dilation radius.</param>
    /// <param name="totalDistance">Nonnegative physical length represented by this segment.</param>
    /// <param name="entryDistance">First intersection distance when an interval exists.</param>
    /// <param name="exitDistance">Last intersection distance when an interval exists.</param>
    /// <param name="startContained">Whether the segment start lies in the closed dilation.</param>
    /// <param name="endContainedStrict">Whether the segment end lies strictly inside the dilation.</param>
    /// <exception cref="ArgumentException">
    /// Thrown when <paramref name="axisDirection"/> is not normalized, or when
    /// <paramref name="totalDistance"/> is zero for a non-point segment.
    /// </exception>
    /// <exception cref="ArgumentOutOfRangeException">
    /// Thrown when the half-length is not positive, or when the radius,
    /// expansion, or total distance is negative.
    /// </exception>
    public readonly bool TryGetSweptSphereFiniteCylinderIntersectionDistanceInterval(
        Vector3d center,
        Vector3d axisDirection,
        Fixed64 axisHalfLength,
        Fixed64 radius,
        Fixed64 sphericalExpansion,
        Fixed64 totalDistance,
        out Fixed64 entryDistance,
        out Fixed64 exitDistance,
        out bool startContained,
        out bool endContainedStrict)
    {
        ValidateSphericallyExpandedFiniteCylinderArguments(
            axisDirection,
            axisHalfLength,
            radius,
            sphericalExpansion,
            totalDistance);

        return WideFiniteAxisIntersection.TryGetSphericallyExpandedFiniteCylinderDistanceInterval(
            this,
            center,
            axisDirection,
            axisHalfLength,
            radius,
            sphericalExpansion,
            totalDistance,
            out entryDistance,
            out exitDistance,
            out startContained,
            out endContainedStrict);
    }

    private readonly void ValidateSphericallyExpandedFiniteCylinderArguments(
        Vector3d axisDirection,
        Fixed64 axisHalfLength,
        Fixed64 radius,
        Fixed64 sphericalExpansion,
        Fixed64 totalDistance)
    {
        ValidateTotalDistance(totalDistance);
        ValidateCenteredAxis(axisDirection, axisHalfLength);
        if (axisHalfLength <= Fixed64.Zero)
            throw new ArgumentOutOfRangeException(nameof(axisHalfLength));
        if (radius < Fixed64.Zero)
            throw new ArgumentOutOfRangeException(nameof(radius));
        if (sphericalExpansion < Fixed64.Zero)
            throw new ArgumentOutOfRangeException(nameof(sphericalExpansion));
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

        Signed192 distanceRaw = WideArithmetic.FromSignedRaw(distance.m_rawValue);
        Signed192 totalDistanceRaw = WideArithmetic.FromSignedRaw(totalDistance.m_rawValue);
        return new Vector3d(
            WideGeometry.InterpolateCoordinate(Start.X, End.X, distanceRaw, totalDistanceRaw),
            WideGeometry.InterpolateCoordinate(Start.Y, End.Y, distanceRaw, totalDistanceRaw),
            WideGeometry.InterpolateCoordinate(Start.Z, End.Z, distanceRaw, totalDistanceRaw));
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

}
