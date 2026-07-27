//=======================================================================
// FixedRay.FiniteAxis.cs
//=======================================================================
// MIT License, Copyright (c) 2024–present David Oravsky (mrdav30)
// See LICENSE file in the project root for full license information.
//=======================================================================

using System;

namespace FixedMathSharp.Geometry;

/// <content>
/// Contains methods for finding the first distance where a ray reaches a finite axis-aligned capsule or cylinder.
/// </content>
public partial struct FixedRay
{
    /// <summary>
    /// Gets the closed parameter interval where this ray overlaps a capsule,
    /// clipped to <c>[0, <paramref name="maxParameter"/>]</c>.
    /// </summary>
    public readonly bool TryGetCapsuleIntersectionInterval(
        FixedSegment capsuleAxis,
        Fixed64 radius,
        Fixed64 maxParameter,
        out Fixed64 entryParameter,
        out Fixed64 exitParameter) =>
        TryGetCapsuleIntersectionInterval(
            capsuleAxis, radius, Fixed64.Zero, maxParameter,
            out entryParameter, out exitParameter, out _, out _);

    /// <summary>
    /// Gets the closed parameter interval where this ray overlaps a radially
    /// expanded capsule and reports exact bounded-endpoint containment.
    /// </summary>
    /// <remarks>
    /// Direction need not be normalized. When it is normalized, returned
    /// parameters are physical distances. The origin is tested inclusively;
    /// the point at <paramref name="maxParameter"/> is tested strictly.
    /// </remarks>
    public readonly bool TryGetCapsuleIntersectionInterval(
        FixedSegment capsuleAxis,
        Fixed64 radius,
        Fixed64 radiusExpansion,
        Fixed64 maxParameter,
        out Fixed64 entryParameter,
        out Fixed64 exitParameter,
        out bool originContained,
        out bool maximumContainedStrict)
    {
        if (radius < Fixed64.Zero)
            throw new ArgumentOutOfRangeException(nameof(radius));
        if (radiusExpansion < Fixed64.Zero)
            throw new ArgumentOutOfRangeException(nameof(radiusExpansion));
        if (!ValidateMaximum(
                maxParameter,
                out entryParameter,
                out exitParameter,
                out originContained,
                out maximumContainedStrict))
        {
            return false;
        }

        return WideFiniteAxisIntersection.TryGetCapsuleInterval(
            this, maxParameter, capsuleAxis, radius, radiusExpansion,
            out entryParameter, out exitParameter,
            out originContained, out maximumContainedStrict);
    }

    /// <summary>
    /// Gets the closed parameter interval where this ray overlaps a centered
    /// capsule, clipped to <c>[0, <paramref name="maxParameter"/>]</c>.
    /// </summary>
    public readonly bool TryGetCapsuleIntersectionInterval(
        Vector3d center,
        Vector3d axisDirection,
        Fixed64 axisLength,
        Fixed64 radius,
        Fixed64 maxParameter,
        out Fixed64 entryParameter,
        out Fixed64 exitParameter) =>
        TryGetCapsuleIntersectionInterval(
            center, axisDirection, axisLength, radius, Fixed64.Zero, maxParameter,
            out entryParameter, out exitParameter, out _, out _);

    /// <summary>
    /// Gets the closed parameter interval where this ray overlaps a centered,
    /// radially expanded capsule and reports exact endpoint containment.
    /// </summary>
    public readonly bool TryGetCapsuleIntersectionInterval(
        Vector3d center,
        Vector3d axisDirection,
        Fixed64 axisLength,
        Fixed64 radius,
        Fixed64 radiusExpansion,
        Fixed64 maxParameter,
        out Fixed64 entryParameter,
        out Fixed64 exitParameter,
        out bool originContained,
        out bool maximumContainedStrict)
    {
        ValidateCenteredCapsule(axisDirection, axisLength, radius, radiusExpansion);
        if (!ValidateMaximum(
                maxParameter,
                out entryParameter,
                out exitParameter,
                out originContained,
                out maximumContainedStrict))
        {
            return false;
        }

        return WideFiniteAxisIntersection.TryGetCapsuleInterval(
            this, maxParameter, center, axisDirection, axisLength, radius, radiusExpansion,
            out entryParameter, out exitParameter,
            out originContained, out maximumContainedStrict);
    }

    /// <summary>
    /// Gets the closed parameter interval where this ray overlaps a finite
    /// cylinder, clipped to <c>[0, <paramref name="maxParameter"/>]</c>.
    /// </summary>
    public readonly bool TryGetFiniteCylinderIntersectionInterval(
        FixedSegment cylinderAxis,
        Fixed64 radius,
        Fixed64 maxParameter,
        out Fixed64 entryParameter,
        out Fixed64 exitParameter) =>
        TryGetFiniteCylinderIntersectionInterval(
            cylinderAxis, radius, Fixed64.Zero, maxParameter,
            out entryParameter, out exitParameter, out _, out _);

    /// <summary>
    /// Gets the closed parameter interval where this ray overlaps a radially
    /// expanded finite cylinder and reports exact endpoint containment.
    /// </summary>
    public readonly bool TryGetFiniteCylinderIntersectionInterval(
        FixedSegment cylinderAxis,
        Fixed64 radius,
        Fixed64 radiusExpansion,
        Fixed64 maxParameter,
        out Fixed64 entryParameter,
        out Fixed64 exitParameter,
        out bool originContained,
        out bool maximumContainedStrict)
    {
        if (cylinderAxis.Start == cylinderAxis.End)
            throw new ArgumentException("A finite cylinder axis must have nonzero length.", nameof(cylinderAxis));
        if (radius < Fixed64.Zero)
            throw new ArgumentOutOfRangeException(nameof(radius));
        if (radiusExpansion < Fixed64.Zero)
            throw new ArgumentOutOfRangeException(nameof(radiusExpansion));
        if (!ValidateMaximum(
                maxParameter,
                out entryParameter,
                out exitParameter,
                out originContained,
                out maximumContainedStrict))
        {
            return false;
        }

        return WideFiniteAxisIntersection.TryGetFiniteCylinderInterval(
            this, maxParameter, cylinderAxis, radius, radiusExpansion,
            out entryParameter, out exitParameter,
            out originContained, out maximumContainedStrict);
    }

    /// <summary>
    /// Gets the closed parameter interval where this ray overlaps a centered,
    /// affinely expanded finite cylinder.
    /// </summary>
    public readonly bool TryGetFiniteCylinderIntersectionInterval(
        Vector3d center,
        Vector3d axisDirection,
        Fixed64 axisLength,
        Fixed64 radius,
        Fixed64 radiusExpansion,
        Fixed64 axialExpansion,
        Fixed64 maxParameter,
        out Fixed64 entryParameter,
        out Fixed64 exitParameter) =>
        TryGetFiniteCylinderIntersectionInterval(
            center, axisDirection, axisLength,
            radius, radiusExpansion, axialExpansion, maxParameter,
            out entryParameter, out exitParameter, out _, out _);

    /// <summary>
    /// Gets the closed parameter interval where this ray overlaps a centered,
    /// affinely expanded finite cylinder and reports exact endpoint containment.
    /// </summary>
    public readonly bool TryGetFiniteCylinderIntersectionInterval(
        Vector3d center,
        Vector3d axisDirection,
        Fixed64 axisLength,
        Fixed64 radius,
        Fixed64 radiusExpansion,
        Fixed64 axialExpansion,
        Fixed64 maxParameter,
        out Fixed64 entryParameter,
        out Fixed64 exitParameter,
        out bool originContained,
        out bool maximumContainedStrict)
    {
        if (!axisDirection.IsNormalized())
            throw new ArgumentException("Finite cylinder axis direction must be normalized.", nameof(axisDirection));
        if (axisLength <= Fixed64.Zero)
            throw new ArgumentOutOfRangeException(nameof(axisLength));
        ValidateCylinderExpansions(radius, radiusExpansion, axialExpansion);
        if (!ValidateMaximum(
                maxParameter,
                out entryParameter,
                out exitParameter,
                out originContained,
                out maximumContainedStrict))
        {
            return false;
        }

        return WideFiniteAxisIntersection.TryGetFiniteCylinderInterval(
            this, maxParameter, center, axisDirection, axisLength,
            radius, radiusExpansion, axialExpansion,
            out entryParameter, out exitParameter,
            out originContained, out maximumContainedStrict);
    }

    private static void ValidateCenteredCapsule(
        Vector3d axisDirection,
        Fixed64 axisLength,
        Fixed64 radius,
        Fixed64 radiusExpansion)
    {
        if (!axisDirection.IsNormalized())
            throw new ArgumentException("Capsule axis direction must be normalized.", nameof(axisDirection));
        if (axisLength < Fixed64.Zero)
            throw new ArgumentOutOfRangeException(nameof(axisLength));
        if (radius < Fixed64.Zero)
            throw new ArgumentOutOfRangeException(nameof(radius));
        if (radiusExpansion < Fixed64.Zero)
            throw new ArgumentOutOfRangeException(nameof(radiusExpansion));
    }

    private static void ValidateCylinderExpansions(
        Fixed64 radius,
        Fixed64 radiusExpansion,
        Fixed64 axialExpansion)
    {
        if (radius < Fixed64.Zero)
            throw new ArgumentOutOfRangeException(nameof(radius));
        if (radiusExpansion < Fixed64.Zero)
            throw new ArgumentOutOfRangeException(nameof(radiusExpansion));
        if (axialExpansion < Fixed64.Zero)
            throw new ArgumentOutOfRangeException(nameof(axialExpansion));
    }

    private static bool ValidateMaximum(
        Fixed64 maxParameter,
        out Fixed64 entryParameter,
        out Fixed64 exitParameter,
        out bool originContained,
        out bool maximumContainedStrict)
    {
        entryParameter = default;
        exitParameter = default;
        originContained = false;
        maximumContainedStrict = false;
        return maxParameter >= Fixed64.Zero;
    }
}
