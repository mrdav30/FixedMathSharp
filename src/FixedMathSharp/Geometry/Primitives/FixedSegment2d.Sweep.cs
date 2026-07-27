//=======================================================================
// FixedSegment2d.Sweep.cs
//=======================================================================
// MIT License, Copyright (c) 2024–present David Oravsky (mrdav30)
// See LICENSE file in the project root for full license information.
//=======================================================================

using System;

namespace FixedMathSharp.Bounds;

/// <content>
/// Sweep-based intersection tests for capsule shapes built on <see cref="FixedSegment2d"/> axes.
/// </content>
public partial struct FixedSegment2d
{
    /// <summary>
    /// Finds the first distance where a translated conceptual centered
    /// capsule intersects an endpoint-authored capsule axis.
    /// </summary>
    public static bool TryGetSweptCenteredCapsuleSegmentFirstDistance(
        Vector2d center,
        Vector2d axisDirection,
        Fixed64 axisLength,
        Fixed64 radius,
        Vector2d direction,
        Fixed64 maximumDistance,
        FixedSegment2d targetAxis,
        Fixed64 targetRadius,
        out Fixed64 distance,
        out Vector2d normal)
    {
        ValidateSweep(
            axisDirection,
            axisLength,
            radius,
            direction,
            maximumDistance,
            targetRadius);
        return WideFiniteAxisIntersection.TryGetSweptCenteredCapsuleSegmentFirstDistance(
            center,
            axisDirection,
            axisLength,
            radius,
            direction,
            maximumDistance,
            targetAxis,
            targetRadius,
            out distance,
            out normal);
    }

    /// <summary>
    /// Finds the first distance where a translated conceptual centered
    /// capsule intersects another conceptual centered capsule.
    /// </summary>
    public static bool TryGetSweptCenteredCapsulesFirstDistance(
        Vector2d center,
        Vector2d axisDirection,
        Fixed64 axisLength,
        Fixed64 radius,
        Vector2d direction,
        Fixed64 maximumDistance,
        Vector2d targetCenter,
        Vector2d targetAxisDirection,
        Fixed64 targetAxisLength,
        Fixed64 targetRadius,
        out Fixed64 distance,
        out Vector2d normal)
    {
        ValidateSweep(
            axisDirection,
            axisLength,
            radius,
            direction,
            maximumDistance,
            targetRadius);
        ValidateCenteredAxis(
            targetAxisDirection,
            targetAxisLength,
            nameof(targetAxisDirection),
            nameof(targetAxisLength));
        return WideFiniteAxisIntersection.TryGetSweptCenteredCapsulesFirstDistance(
            center,
            axisDirection,
            axisLength,
            radius,
            direction,
            maximumDistance,
            targetCenter,
            targetAxisDirection,
            targetAxisLength,
            targetRadius,
            out distance,
            out normal);
    }

    private static void ValidateSweep(
        Vector2d axisDirection,
        Fixed64 axisLength,
        Fixed64 radius,
        Vector2d direction,
        Fixed64 maximumDistance,
        Fixed64 targetRadius)
    {
        ValidateCenteredAxis(axisDirection, axisLength);
        if (radius < Fixed64.Zero)
            throw new ArgumentOutOfRangeException(nameof(radius));
        if (!direction.IsNormalized())
            throw new ArgumentException("Sweep direction must be normalized.", nameof(direction));
        if (maximumDistance < Fixed64.Zero)
            throw new ArgumentOutOfRangeException(nameof(maximumDistance));
        if (targetRadius < Fixed64.Zero)
            throw new ArgumentOutOfRangeException(nameof(targetRadius));
    }
}
