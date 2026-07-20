//=======================================================================
// FixedRay2d.FiniteAxis.cs
//=======================================================================
// MIT License, Copyright (c) 2024–present David Oravsky (mrdav30)
// See LICENSE file in the project root for full license information.
//=======================================================================

using System;

namespace FixedMathSharp.Bounds;

public partial struct FixedRay2d
{
    /// <summary>
    /// Gets the closed parameter interval where this ray overlaps a capsule,
    /// clipped to <c>[0, <paramref name="maxParameter"/>]</c>.
    /// </summary>
    public readonly bool TryGetCapsuleIntersectionInterval(
        FixedSegment2d capsuleAxis,
        Fixed64 radius,
        Fixed64 maxParameter,
        out Fixed64 entryParameter,
        out Fixed64 exitParameter) =>
        TryGetCapsuleIntersectionInterval(
            capsuleAxis,
            radius,
            Fixed64.Zero,
            maxParameter,
            out entryParameter,
            out exitParameter,
            out _,
            out _);

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
        FixedSegment2d capsuleAxis,
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
        if (maxParameter < Fixed64.Zero)
        {
            entryParameter = default;
            exitParameter = default;
            originContained = false;
            maximumContainedStrict = false;
            return false;
        }

        return WideFiniteAxisIntersection.TryGetCapsuleInterval(
            this,
            maxParameter,
            capsuleAxis,
            radius,
            radiusExpansion,
            out entryParameter,
            out exitParameter,
            out originContained,
            out maximumContainedStrict);
    }

    /// <summary>
    /// Gets the closed parameter interval where this ray overlaps a centered
    /// capsule, clipped to <c>[0, <paramref name="maxParameter"/>]</c>.
    /// </summary>
    public readonly bool TryGetCapsuleIntersectionInterval(
        Vector2d center,
        Vector2d axisDirection,
        Fixed64 axisHalfLength,
        Fixed64 radius,
        Fixed64 maxParameter,
        out Fixed64 entryParameter,
        out Fixed64 exitParameter) =>
        TryGetCapsuleIntersectionInterval(
            center,
            axisDirection,
            axisHalfLength,
            radius,
            Fixed64.Zero,
            maxParameter,
            out entryParameter,
            out exitParameter,
            out _,
            out _);

    /// <summary>
    /// Gets the closed parameter interval where this ray overlaps a centered,
    /// radially expanded capsule and reports exact endpoint containment.
    /// </summary>
    public readonly bool TryGetCapsuleIntersectionInterval(
        Vector2d center,
        Vector2d axisDirection,
        Fixed64 axisHalfLength,
        Fixed64 radius,
        Fixed64 radiusExpansion,
        Fixed64 maxParameter,
        out Fixed64 entryParameter,
        out Fixed64 exitParameter,
        out bool originContained,
        out bool maximumContainedStrict)
    {
        if (!axisDirection.IsNormalized())
            throw new ArgumentException("Capsule axis direction must be normalized.", nameof(axisDirection));
        if (axisHalfLength < Fixed64.Zero)
            throw new ArgumentOutOfRangeException(nameof(axisHalfLength));
        if (radius < Fixed64.Zero)
            throw new ArgumentOutOfRangeException(nameof(radius));
        if (radiusExpansion < Fixed64.Zero)
            throw new ArgumentOutOfRangeException(nameof(radiusExpansion));
        if (maxParameter < Fixed64.Zero)
        {
            entryParameter = default;
            exitParameter = default;
            originContained = false;
            maximumContainedStrict = false;
            return false;
        }

        return WideFiniteAxisIntersection.TryGetCapsuleInterval(
            this,
            maxParameter,
            center,
            axisDirection,
            axisHalfLength,
            radius,
            radiusExpansion,
            out entryParameter,
            out exitParameter,
            out originContained,
            out maximumContainedStrict);
    }
}
