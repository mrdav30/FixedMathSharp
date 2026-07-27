//=======================================================================
// FixedConvexPrismRelations.cs
//=======================================================================
// MIT License, Copyright (c) 2024–present David Oravsky (mrdav30)
// See LICENSE file in the project root for full license information.
//=======================================================================

using FixedMathSharp.Geometry;
using System;

namespace FixedMathSharp;

/// <summary>
/// Provides exact owner-relative contacts between centered finite surfaces
/// and vertical convex prisms.
/// </summary>
public static class FixedConvexPrismRelations
{
    /// <summary>
    /// Attempts to construct canonical contact anchors between a rigidly
    /// transformed triangle and a rotated vertical convex prism.
    /// </summary>
    public static bool TryGetTriangleContact(
        Vector3d triangleOrigin,
        FixedQuaternion triangleRotation,
        FixedTriangle triangle,
        Vector3d prismOrigin,
        Fixed64 prismRotation,
        ReadOnlySpan<Vector2d> prismLocalOffsets,
        Fixed64 prismHalfThickness,
        out FixedContactAnchors contact)
    {
        if (!triangleRotation.IsNormalized())
            throw new ArgumentException(
                "The triangle rotation must be normalized.",
                nameof(triangleRotation));
        ValidatePrism(
            Fixed64.Zero,
            prismLocalOffsets,
            prismHalfThickness);
        return WideOrientedBox.TryGetTrianglePrismContact(
            triangleOrigin,
            triangleRotation,
            triangle,
            prismOrigin,
            prismRotation,
            prismLocalOffsets,
            prismHalfThickness,
            out contact);
    }

    /// <summary>
    /// Attempts to construct canonical contact anchors between a sphere and a
    /// rotated vertical convex prism.
    /// </summary>
    public static bool TryGetSphereContact(
        Vector3d sphereCenter,
        Fixed64 sphereRadius,
        Vector3d prismOrigin,
        Fixed64 prismRotation,
        ReadOnlySpan<Vector2d> prismLocalOffsets,
        Fixed64 prismHalfThickness,
        out FixedContactAnchors contact)
    {
        ValidatePrism(
            sphereRadius,
            prismLocalOffsets,
            prismHalfThickness);
        return WideConvexPrismRelations.TryGetCenteredCapsuleContact(
            sphereCenter,
            FixedQuaternion.Identity,
            Vector3d.Up,
            Fixed64.Zero,
            sphereRadius,
            prismOrigin,
            prismRotation,
            prismLocalOffsets,
            prismHalfThickness,
            out contact);
    }

    /// <summary>
    /// Attempts to construct canonical contact anchors between a centered
    /// capsule and a rotated vertical convex prism.
    /// </summary>
    public static bool TryGetCenteredCapsuleContact(
        Vector3d capsuleCenter,
        FixedQuaternion capsuleRotation,
        Vector3d localCapsuleAxisDirection,
        Fixed64 capsuleAxisLength,
        Fixed64 capsuleRadius,
        Vector3d prismOrigin,
        Fixed64 prismRotation,
        ReadOnlySpan<Vector2d> prismLocalOffsets,
        Fixed64 prismHalfThickness,
        out FixedContactAnchors contact)
    {
        Validate(
            capsuleRotation,
            localCapsuleAxisDirection,
            capsuleAxisLength,
            nameof(capsuleAxisLength),
            capsuleRadius,
            prismLocalOffsets,
            prismHalfThickness,
            requirePositiveLength: false);
        return WideConvexPrismRelations.TryGetCenteredCapsuleContact(
            capsuleCenter,
            capsuleRotation,
            localCapsuleAxisDirection,
            capsuleAxisLength,
            capsuleRadius,
            prismOrigin,
            prismRotation,
            prismLocalOffsets,
            prismHalfThickness,
            out contact);
    }

    /// <summary>
    /// Attempts to construct canonical contact anchors between a centered
    /// finite cylinder and a rotated vertical convex prism.
    /// </summary>
    public static bool TryGetCenteredCylinderContact(
        Vector3d cylinderCenter,
        FixedQuaternion cylinderRotation,
        Vector3d localCylinderAxisDirection,
        Fixed64 cylinderAxisLength,
        Fixed64 cylinderRadius,
        Vector3d prismOrigin,
        Fixed64 prismRotation,
        ReadOnlySpan<Vector2d> prismLocalOffsets,
        Fixed64 prismHalfThickness,
        out FixedContactAnchors contact)
    {
        Validate(
            cylinderRotation,
            localCylinderAxisDirection,
            cylinderAxisLength,
            nameof(cylinderAxisLength),
            cylinderRadius,
            prismLocalOffsets,
            prismHalfThickness,
            requirePositiveLength: true);
        return WideConvexPrismRelations.TryGetCenteredCylinderContact(
            cylinderCenter,
            cylinderRotation,
            localCylinderAxisDirection,
            cylinderAxisLength,
            cylinderRadius,
            prismOrigin,
            prismRotation,
            prismLocalOffsets,
            prismHalfThickness,
            out contact);
    }

    /// <summary>
    /// Attempts to construct canonical contact anchors between a centered
    /// finite cone and a rotated vertical convex prism.
    /// </summary>
    public static bool TryGetCenteredConeContact(
        Vector3d coneCenter,
        FixedQuaternion coneRotation,
        Vector3d localConeAxisDirection,
        Fixed64 coneHeight,
        Fixed64 coneRadius,
        Vector3d prismOrigin,
        Fixed64 prismRotation,
        ReadOnlySpan<Vector2d> prismLocalOffsets,
        Fixed64 prismHalfThickness,
        out FixedContactAnchors contact)
    {
        Validate(
            coneRotation,
            localConeAxisDirection,
            coneHeight,
            nameof(coneHeight),
            coneRadius,
            prismLocalOffsets,
            prismHalfThickness,
            requirePositiveLength: true);
        return WideConvexPrismRelations.TryGetCenteredConeContact(
            coneCenter,
            coneRotation,
            localConeAxisDirection,
            coneHeight,
            coneRadius,
            prismOrigin,
            prismRotation,
            prismLocalOffsets,
            prismHalfThickness,
            out contact);
    }

    private static void Validate(
        FixedQuaternion rotation,
        Vector3d localAxisDirection,
        Fixed64 length,
        string lengthParameterName,
        Fixed64 radius,
        ReadOnlySpan<Vector2d> prismLocalOffsets,
        Fixed64 prismHalfThickness,
        bool requirePositiveLength)
    {
        if (!rotation.IsNormalized())
            throw new ArgumentException(
                "Shape rotation must be normalized.",
                nameof(rotation));
        if (!localAxisDirection.IsNormalized())
            throw new ArgumentException(
                "Local shape axis direction must be normalized.",
                nameof(localAxisDirection));
        if (requirePositiveLength ? length <= Fixed64.Zero : length < Fixed64.Zero)
            throw new ArgumentOutOfRangeException(lengthParameterName);
        ValidatePrism(radius, prismLocalOffsets, prismHalfThickness);
    }

    private static void ValidatePrism(
        Fixed64 radius,
        ReadOnlySpan<Vector2d> prismLocalOffsets,
        Fixed64 prismHalfThickness)
    {
        if (radius < Fixed64.Zero)
            throw new ArgumentOutOfRangeException(nameof(radius));
        if (prismLocalOffsets.Length < 3)
        {
            throw new ArgumentException(
                "A convex prism requires at least three ordered boundary offsets.",
                nameof(prismLocalOffsets));
        }
        if (prismHalfThickness <= Fixed64.Zero)
            throw new ArgumentOutOfRangeException(nameof(prismHalfThickness));
    }
}
