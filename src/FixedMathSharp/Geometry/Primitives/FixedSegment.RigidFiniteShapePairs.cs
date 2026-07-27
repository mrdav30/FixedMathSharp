//=======================================================================
// FixedSegment.RigidFiniteShapePairs.cs
//=======================================================================
// MIT License, Copyright (c) 2024-present David Oravsky (mrdav30)
// See LICENSE file in the project root for full license information.
//=======================================================================

using System;

namespace FixedMathSharp.Bounds;

/// <content>
/// Contact generation between rigid finite shapes (cylinders and capsules)
/// defined by a center, rotation, local axis direction, axis length, and radius.
/// </content>
public partial struct FixedSegment
{
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
