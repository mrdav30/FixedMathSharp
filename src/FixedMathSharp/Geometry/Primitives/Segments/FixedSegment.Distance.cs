//=======================================================================
// FixedSegment.Distance.cs
//=======================================================================
// MIT License, Copyright (c) 2024–present David Oravsky (mrdav30)
// See LICENSE file in the project root for full license information.
//=======================================================================

using System;
using System.Runtime.CompilerServices;

namespace FixedMathSharp.Geometry;

/// <content>
/// Distance and intersection-interval queries for <see cref="FixedSegment"/>,
/// including sphere, capsule, and finite-axis intersection tests.
/// </content>
public partial struct FixedSegment
{
    /// <summary>
    /// Finds the physical-distance interval where this segment intersects a
    /// sphere.
    /// </summary>
    public readonly bool TryGetSphereIntersectionDistanceInterval(
        FixedBoundSphere sphere,
        Fixed64 totalDistance,
        out Fixed64 entryDistance,
        out Fixed64 exitDistance) =>
        TryGetSphereIntersectionDistanceInterval(
            sphere,
            Fixed64.Zero,
            totalDistance,
            out entryDistance,
            out exitDistance,
            out _,
            out _);

    /// <summary>
    /// Finds the physical-distance interval where this segment intersects a
    /// radially expanded sphere and reports exact endpoint containment.
    /// </summary>
    public readonly bool TryGetSphereIntersectionDistanceInterval(
        FixedBoundSphere sphere,
        Fixed64 radiusExpansion,
        Fixed64 totalDistance,
        out Fixed64 entryDistance,
        out Fixed64 exitDistance,
        out bool startContained,
        out bool endContainedStrict)
    {
        ValidateTotalDistance(totalDistance);
        if (radiusExpansion < Fixed64.Zero)
            throw new ArgumentOutOfRangeException(nameof(radiusExpansion));

        return WideFiniteAxisIntersection.TryGetSphereDistanceInterval(
            this,
            sphere,
            radiusExpansion,
            totalDistance,
            out entryDistance,
            out exitDistance,
            out startContained,
            out endContainedStrict);
    }

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
        Fixed64 axisLength,
        Fixed64 radius,
        Fixed64 totalDistance,
        out Fixed64 entryDistance,
        out Fixed64 exitDistance) =>
        TryGetCapsuleIntersectionDistanceInterval(
            center,
            axisDirection,
            axisLength,
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
        Fixed64 axisLength,
        Fixed64 radius,
        Fixed64 radiusExpansion,
        Fixed64 totalDistance,
        out Fixed64 entryDistance,
        out Fixed64 exitDistance,
        out bool startContained,
        out bool endContainedStrict)
    {
        ValidateTotalDistance(totalDistance);
        ValidateCenteredAxis(axisDirection, axisLength);
        if (radius < Fixed64.Zero)
            throw new ArgumentOutOfRangeException(nameof(radius));
        if (radiusExpansion < Fixed64.Zero)
            throw new ArgumentOutOfRangeException(nameof(radiusExpansion));

        return WideFiniteAxisIntersection.TryGetCapsuleDistanceInterval(
            this,
            center,
            axisDirection,
            axisLength,
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
        Fixed64 axisLength,
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
        if (axisLength <= Fixed64.Zero)
            throw new ArgumentOutOfRangeException(nameof(axisLength));
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
            axisLength,
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
    /// <param name="axisLength">Positive unexpanded cylinder axis length.</param>
    /// <param name="radius">Nonnegative unexpanded cylinder radius.</param>
    /// <param name="sphericalExpansion">Nonnegative spherical dilation radius.</param>
    /// <param name="totalDistance">Nonnegative physical length represented by this segment.</param>
    /// <param name="distance">First intersection distance when one exists.</param>
    /// <exception cref="ArgumentException">
    /// Thrown when <paramref name="axisDirection"/> is not normalized, or when
    /// <paramref name="totalDistance"/> is zero for a non-point segment.
    /// </exception>
    /// <exception cref="ArgumentOutOfRangeException">
    /// Thrown when the axis length is not positive, or when the radius,
    /// expansion, or total distance is negative.
    /// </exception>
    public readonly bool TryGetSweptSphereFiniteCylinderIntersectionDistance(
        Vector3d center,
        Vector3d axisDirection,
        Fixed64 axisLength,
        Fixed64 radius,
        Fixed64 sphericalExpansion,
        Fixed64 totalDistance,
        out Fixed64 distance)
    {
        ValidateSphericallyExpandedFiniteCylinderArguments(
            axisDirection,
            axisLength,
            radius,
            sphericalExpansion,
            totalDistance);
        return WideFiniteAxisIntersection.TryGetSphericallyExpandedFiniteCylinderFirstDistance(
            this,
            center,
            axisDirection,
            axisLength,
            radius,
            sphericalExpansion,
            totalDistance,
            out distance);
    }

    /// <summary>
    /// Finds the first physical distance where this segment intersects the
    /// exact spherical dilation of a centered finite cylinder described by
    /// its nonnegative half-axis length.
    /// </summary>
    /// <remarks>
    /// This explicit half-axis contract retains the conceptual full length in
    /// wide arithmetic, so half lengths greater than
    /// <see cref="Fixed64.MaxValue"/> / 2 remain valid.
    /// </remarks>
    /// <param name="center">Cylinder center.</param>
    /// <param name="axisDirection">Normalized cylinder axis direction.</param>
    /// <param name="halfAxisLength">Nonnegative distance from the center to either flat cap.</param>
    /// <param name="radius">Nonnegative unexpanded cylinder radius.</param>
    /// <param name="sphericalExpansion">Nonnegative spherical dilation radius.</param>
    /// <param name="totalDistance">Nonnegative physical length represented by this segment.</param>
    /// <param name="distance">First intersection distance when one exists.</param>
    /// <exception cref="ArgumentException">
    /// Thrown when <paramref name="axisDirection"/> is not normalized, or when
    /// <paramref name="totalDistance"/> is zero for a non-point segment.
    /// </exception>
    /// <exception cref="ArgumentOutOfRangeException">
    /// Thrown when the half-axis length, radius, expansion, or total distance
    /// is negative.
    /// </exception>
    public readonly bool TryGetSweptSphereCenteredFiniteCylinderIntersectionDistanceFromHalfAxisLength(
        Vector3d center,
        Vector3d axisDirection,
        Fixed64 halfAxisLength,
        Fixed64 radius,
        Fixed64 sphericalExpansion,
        Fixed64 totalDistance,
        out Fixed64 distance)
    {
        ValidateTotalDistance(totalDistance);
        ValidateCenteredAxis(axisDirection, halfAxisLength);
        if (radius < Fixed64.Zero)
            throw new ArgumentOutOfRangeException(nameof(radius));
        if (sphericalExpansion < Fixed64.Zero)
            throw new ArgumentOutOfRangeException(nameof(sphericalExpansion));

        return WideFiniteAxisIntersection
            .TryGetSphericallyExpandedFiniteCylinderFirstDistanceFromHalfAxisLength(
                this,
                center,
                axisDirection,
                halfAxisLength,
                radius,
                sphericalExpansion,
                totalDistance,
                out distance);
    }

    /// <summary>
    /// Finds the first physical distance where this segment intersects the
    /// exact spherical dilation of an axis-aligned box.
    /// </summary>
    /// <param name="box">Unexpanded axis-aligned box.</param>
    /// <param name="sphericalExpansion">Nonnegative spherical dilation radius.</param>
    /// <param name="totalDistance">Nonnegative physical length represented by this segment.</param>
    /// <param name="distance">First intersection distance when one exists.</param>
    /// <exception cref="ArgumentException">
    /// Thrown when <paramref name="totalDistance"/> is zero for a non-point segment.
    /// </exception>
    /// <exception cref="ArgumentOutOfRangeException">
    /// Thrown when the expansion or total distance is negative.
    /// </exception>
    public readonly bool TryGetSweptSphereBoxIntersectionDistance(
        FixedBoundBox box,
        Fixed64 sphericalExpansion,
        Fixed64 totalDistance,
        out Fixed64 distance)
    {
        ValidateTotalDistance(totalDistance);
        if (sphericalExpansion < Fixed64.Zero)
            throw new ArgumentOutOfRangeException(nameof(sphericalExpansion));

        return WideFiniteAxisIntersection.TryGetSphericallyExpandedBoxFirstDistance(
            this,
            box,
            sphericalExpansion,
            totalDistance,
            out distance);
    }

    /// <summary>
    /// Finds the first physical distance where this segment intersects the
    /// exact spherical dilation of an oriented box.
    /// </summary>
    /// <param name="box">Unexpanded oriented box.</param>
    /// <param name="sphericalExpansion">Nonnegative spherical dilation radius.</param>
    /// <param name="totalDistance">Nonnegative physical length represented by this segment.</param>
    /// <param name="distance">First intersection distance when one exists.</param>
    /// <exception cref="ArgumentException">
    /// Thrown when <paramref name="totalDistance"/> is zero for a non-point segment.
    /// </exception>
    /// <exception cref="ArgumentOutOfRangeException">
    /// Thrown when the expansion or total distance is negative.
    /// </exception>
    public readonly bool TryGetSweptSphereOrientedBoxIntersectionDistance(
        FixedOrientedBox box,
        Fixed64 sphericalExpansion,
        Fixed64 totalDistance,
        out Fixed64 distance)
    {
        ValidateTotalDistance(totalDistance);
        if (sphericalExpansion < Fixed64.Zero)
            throw new ArgumentOutOfRangeException(nameof(sphericalExpansion));

        return WideOrientedBox.TryGetSweptSphereIntersectionDistance(
            box.Center,
            box.Orientation,
            box.HalfExtents,
            this,
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
    /// <param name="axisLength">Positive unexpanded cylinder axis length.</param>
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
    /// Thrown when the axis length is not positive, or when the radius,
    /// expansion, or total distance is negative.
    /// </exception>
    public readonly bool TryGetSweptSphereFiniteCylinderIntersectionDistanceInterval(
        Vector3d center,
        Vector3d axisDirection,
        Fixed64 axisLength,
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
            axisLength,
            radius,
            sphericalExpansion,
            totalDistance);

        return WideFiniteAxisIntersection.TryGetSphericallyExpandedFiniteCylinderDistanceInterval(
            this,
            center,
            axisDirection,
            axisLength,
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
        Fixed64 axisLength,
        Fixed64 radius,
        Fixed64 sphericalExpansion,
        Fixed64 totalDistance)
    {
        ValidateTotalDistance(totalDistance);
        ValidateCenteredAxis(axisDirection, axisLength);
        if (axisLength <= Fixed64.Zero)
            throw new ArgumentOutOfRangeException(nameof(axisLength));
        if (radius < Fixed64.Zero)
            throw new ArgumentOutOfRangeException(nameof(radius));
        if (sphericalExpansion < Fixed64.Zero)
            throw new ArgumentOutOfRangeException(nameof(sphericalExpansion));
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

        Signed192 distanceRaw = Signed192.Signed(distance.m_rawValue);
        Signed192 totalDistanceRaw = Signed192.Signed(totalDistance.m_rawValue);
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
