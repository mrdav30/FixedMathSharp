//=======================================================================
// FixedSegment.RigidQueries.cs
//=======================================================================
// MIT License, Copyright (c) 2024–present David Oravsky (mrdav30)
// See LICENSE file in the project root for full license information.
//=======================================================================

using System;

namespace FixedMathSharp.Geometry;

/// <content>
/// Contains intersection and contact queries for rigidly oriented finite
/// capsules, cylinders, and cones.
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

    /// <summary>
    /// Attempts to construct the minimum-translation contact between two
    /// centered finite cylinders.
    /// </summary>
    /// <remarks>
    /// Both finite axes remain authoritative in their normalized rigid frames.
    /// The returned normal points from the first cylinder toward the second.
    /// </remarks>
    public static bool TryGetCenteredFiniteCylindersContact(
        Vector3d firstCenter,
        FixedQuaternion firstRotation,
        Vector3d firstLocalAxisDirection,
        Fixed64 firstAxisLength,
        Fixed64 firstRadius,
        Vector3d secondCenter,
        FixedQuaternion secondRotation,
        Vector3d secondLocalAxisDirection,
        Fixed64 secondAxisLength,
        Fixed64 secondRadius,
        out FixedContactAnchors contact)
    {
        ValidateRigidFiniteShape(
            firstRotation,
            firstLocalAxisDirection,
            firstAxisLength,
            firstRadius,
            requirePositiveLength: true,
            nameof(firstRotation),
            nameof(firstLocalAxisDirection),
            nameof(firstAxisLength),
            nameof(firstRadius));
        ValidateRigidFiniteShape(
            secondRotation,
            secondLocalAxisDirection,
            secondAxisLength,
            secondRadius,
            requirePositiveLength: true,
            nameof(secondRotation),
            nameof(secondLocalAxisDirection),
            nameof(secondAxisLength),
            nameof(secondRadius));
        return WideConvexPrismRelations
            .TryGetCenteredFiniteCylindersContact(
                firstCenter,
                firstRotation,
                firstLocalAxisDirection,
                firstAxisLength,
                firstRadius,
                secondCenter,
                secondRotation,
                secondLocalAxisDirection,
                secondAxisLength,
                secondRadius,
                out contact);
    }

    /// <summary>
    /// Attempts to construct the minimum-translation contact between a
    /// centered finite cylinder and a centered capsule.
    /// </summary>
    /// <remarks>
    /// Both finite axes remain authoritative in their normalized rigid frames.
    /// The returned normal points from the cylinder toward the capsule.
    /// </remarks>
    public static bool TryGetCenteredFiniteCylinderCapsuleContact(
        Vector3d cylinderCenter,
        FixedQuaternion cylinderRotation,
        Vector3d cylinderLocalAxisDirection,
        Fixed64 cylinderAxisLength,
        Fixed64 cylinderRadius,
        Vector3d capsuleCenter,
        FixedQuaternion capsuleRotation,
        Vector3d capsuleLocalAxisDirection,
        Fixed64 capsuleAxisLength,
        Fixed64 capsuleRadius,
        out FixedContactAnchors contact)
    {
        ValidateRigidFiniteShape(
            cylinderRotation,
            cylinderLocalAxisDirection,
            cylinderAxisLength,
            cylinderRadius,
            requirePositiveLength: true,
            nameof(cylinderRotation),
            nameof(cylinderLocalAxisDirection),
            nameof(cylinderAxisLength),
            nameof(cylinderRadius));
        ValidateRigidFiniteShape(
            capsuleRotation,
            capsuleLocalAxisDirection,
            capsuleAxisLength,
            capsuleRadius,
            requirePositiveLength: false,
            nameof(capsuleRotation),
            nameof(capsuleLocalAxisDirection),
            nameof(capsuleAxisLength),
            nameof(capsuleRadius));
        return WideConvexPrismRelations
            .TryGetCenteredFiniteCylinderCapsuleContact(
                cylinderCenter,
                cylinderRotation,
                cylinderLocalAxisDirection,
                cylinderAxisLength,
                cylinderRadius,
                capsuleCenter,
                capsuleRotation,
                capsuleLocalAxisDirection,
                capsuleAxisLength,
                capsuleRadius,
                out contact);
    }

    private static void ValidateRigidFiniteShape(
        FixedQuaternion rotation,
        Vector3d localAxisDirection,
        Fixed64 axisLength,
        Fixed64 radius,
        bool requirePositiveLength,
        string rotationParameterName,
        string axisParameterName,
        string lengthParameterName,
        string radiusParameterName)
    {
        if (!rotation.IsNormalized())
            throw new ArgumentException("The rigid rotation must be normalized.", rotationParameterName);
        if (!localAxisDirection.IsNormalized())
            throw new ArgumentException("The local axis direction must be normalized.", axisParameterName);
        if (requirePositiveLength
            ? axisLength <= Fixed64.Zero
            : axisLength < Fixed64.Zero)
        {
            throw new ArgumentOutOfRangeException(lengthParameterName);
        }
        if (radius < Fixed64.Zero)
            throw new ArgumentOutOfRangeException(radiusParameterName);
    }
}
