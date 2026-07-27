//=======================================================================
// FixedSegment.RigidFiniteAxisQuery.cs
//=======================================================================
// MIT License, Copyright (c) 2024–present David Oravsky (mrdav30)
// See LICENSE file in the project root for full license information.
//=======================================================================

using System;

namespace FixedMathSharp.Bounds;

/// <content>
/// Intersection queries between this segment and rigidly-oriented finite
/// axis-aligned primitives (capsule, cylinder, cone) defined by a center,
/// quaternion rotation, and axis length.
/// </content>
public partial struct FixedSegment
{
    /// <summary>
    /// Finds the physical-distance interval where this segment intersects a
    /// centered capsule whose local positive Y axis is transformed by a rigid
    /// quaternion frame.
    /// </summary>
    public readonly bool TryGetCapsuleIntersectionDistanceInterval(
        Vector3d center,
        FixedQuaternion rotation,
        Fixed64 axisLength,
        Fixed64 radius,
        Fixed64 radiusExpansion,
        Fixed64 totalDistance,
        out Fixed64 entryDistance,
        out Fixed64 exitDistance,
        out bool startContained,
        out bool endContainedStrict)
    {
        ValidateRigidFiniteAxisQuery(
            rotation,
            axisLength,
            radius,
            radiusExpansion,
            totalDistance,
            requirePositiveLength: false);
        return WideOrientedBox.TryGetRigidCapsuleDistanceInterval(
            this,
            center,
            rotation,
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
    /// Finds the physical-distance interval where this segment intersects a
    /// centered finite cylinder whose local positive Y axis is transformed by
    /// a rigid quaternion frame.
    /// </summary>
    public readonly bool TryGetFiniteCylinderIntersectionDistanceInterval(
        Vector3d center,
        FixedQuaternion rotation,
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
        ValidateRigidFiniteAxisQuery(
            rotation,
            axisLength,
            radius,
            radiusExpansion,
            totalDistance,
            requirePositiveLength: true);
        if (axialExpansion < Fixed64.Zero)
            throw new ArgumentOutOfRangeException(nameof(axialExpansion));

        return WideOrientedBox.TryGetRigidCylinderDistanceInterval(
            this,
            center,
            rotation,
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
    /// Finds the physical-distance interval where this segment intersects a
    /// centered finite cone whose local positive Y axis runs from base to apex
    /// in a rigid quaternion frame.
    /// </summary>
    public readonly bool TryGetCenteredFiniteConeIntersectionDistanceInterval(
        Vector3d center,
        FixedQuaternion rotation,
        Fixed64 height,
        Fixed64 baseRadius,
        Fixed64 totalDistance,
        out Fixed64 entryDistance,
        out Fixed64 exitDistance,
        out bool startContained,
        out bool endContainedStrict)
    {
        ValidateRigidFiniteAxisQuery(
            rotation,
            height,
            baseRadius,
            Fixed64.Zero,
            totalDistance,
            requirePositiveLength: true);
        return WideOrientedBox.TryGetRigidConeDistanceInterval(
            this,
            center,
            rotation,
            height,
            baseRadius,
            totalDistance,
            out entryDistance,
            out exitDistance,
            out startContained,
            out endContainedStrict);
    }

    private readonly void ValidateRigidFiniteAxisQuery(
        FixedQuaternion rotation,
        Fixed64 axisLength,
        Fixed64 radius,
        Fixed64 radiusExpansion,
        Fixed64 totalDistance,
        bool requirePositiveLength)
    {
        ValidateTotalDistance(totalDistance);
        if (!rotation.IsNormalized())
        {
            throw new ArgumentException(
                "Finite-axis rotation must be normalized.",
                nameof(rotation));
        }
        if (requirePositiveLength
            ? axisLength <= Fixed64.Zero
            : axisLength < Fixed64.Zero)
        {
            throw new ArgumentOutOfRangeException(nameof(axisLength));
        }
        if (radius < Fixed64.Zero)
            throw new ArgumentOutOfRangeException(nameof(radius));
        if (radiusExpansion < Fixed64.Zero)
            throw new ArgumentOutOfRangeException(nameof(radiusExpansion));
    }
}
