//=======================================================================
// FixedConvexPrismRelations.cs
//=======================================================================
// MIT License, Copyright (c) 2024–present David Oravsky (mrdav30)
// See LICENSE file in the project root for full license information.
//=======================================================================

using System;
using FixedMathSharp.Geometry;

namespace FixedMathSharp;

/// <summary>
/// Provides exact owner-relative contacts between centered finite surfaces
/// and vertical convex prisms.
/// </summary>
public static class FixedConvexPrismRelations
{
    /// <summary>
    /// Determines whether an upright cylinder translated between two
    /// bottom-center points has strict positive-volume overlap with a rotated
    /// vertical convex prism at any shared continuous parameter in [0, 1].
    /// </summary>
    /// <remarks>
    /// Planar tangency, vertical tangency, and contact that exists only where
    /// the planar and vertical strict intervals meet at one boundary parameter
    /// return <see langword="false"/>. A zero radius requires the swept axis
    /// point to enter the strict prism footprint. The joint parameter relation
    /// is evaluated with exact wide intermediates and does not publish rounded
    /// interval endpoints.
    /// </remarks>
    /// <param name="bottomStart">The cylinder bottom center at parameter zero.</param>
    /// <param name="bottomEnd">The cylinder bottom center at parameter one.</param>
    /// <param name="radius">The nonnegative cylinder radius.</param>
    /// <param name="height">The positive full cylinder height.</param>
    /// <param name="prismOrigin">The center of the vertical convex prism.</param>
    /// <param name="prismRotation">The prism footprint rotation about world Y.</param>
    /// <param name="prismLocalOffsets">At least three ordered convex-footprint offsets.</param>
    /// <param name="prismHalfThickness">The positive prism half-thickness along world Y.</param>
    /// <exception cref="ArgumentOutOfRangeException">
    /// <paramref name="radius"/> is negative, <paramref name="height"/> is not
    /// positive, or <paramref name="prismHalfThickness"/> is not positive.
    /// </exception>
    /// <exception cref="ArgumentException">
    /// Fewer than three ordered prism offsets were supplied.
    /// </exception>
    public static bool IntersectsSweptUprightCylinderStrict(
        Vector3d bottomStart,
        Vector3d bottomEnd,
        Fixed64 radius,
        Fixed64 height,
        Vector3d prismOrigin,
        Fixed64 prismRotation,
        ReadOnlySpan<Vector2d> prismLocalOffsets,
        Fixed64 prismHalfThickness)
    {
        if (height <= Fixed64.Zero)
            throw new ArgumentOutOfRangeException(nameof(height));
        ValidatePrism(radius, prismLocalOffsets, prismHalfThickness);
        return WideConvexPrismRelations
            .IntersectsSweptUprightCylinderStrict(
                bottomStart,
                bottomEnd,
                radius,
                height,
                prismOrigin,
                prismRotation,
                prismLocalOffsets,
                prismHalfThickness);
    }

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
    /// Attempts to find the minimum-depth contact between a centered
    /// finite cylinder and a rotated vertical convex prism.
    /// </summary>
    /// <remarks>
    /// The complete support fan includes prism faces, real edges and vertices
    /// against both flat caps, the side and circular rims. Classification and
    /// minimum-depth ranking use the exact authored geometry. Touching is
    /// contact with zero depth; an overlap smaller than half a raw unit can
    /// also round to zero. A zero radius is the finite axis segment.
    /// <para>
    /// The normal points from the cylinder toward the prism. Depth and each
    /// unit-normal component round nearest-even after selection. Unclamped
    /// depth differs from the exact minimum by at most half a raw unit;
    /// translating along the rounded normal additionally introduces an error
    /// proportional to depth. <see cref="FixedContactAnchors.DepthIsClamped"/>
    /// identifies an exact minimum above <see cref="Fixed64.MaxValue"/>.
    /// </para>
    /// <para>
    /// Both finite support witnesses are matched before final coordinate
    /// rounding. The first anchor retains the cylinder center and the second
    /// the prism origin; both use the cylinder rotation as their local frame.
    /// The prism's scalar yaw remains authoritative. Use the anchor operations
    /// to compare or materialize points rather than interpreting the second
    /// anchor's local coordinates as prism footprint offsets.
    /// </para>
    /// </remarks>
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
    /// Attempts to find the minimum-depth contact between a centered
    /// finite cone and a rotated vertical convex prism.
    /// </summary>
    /// <remarks>
    /// The complete support fan includes prism faces, real edges and vertices
    /// against the base disk and rim, apex and lateral generators.
    /// Classification and minimum-depth ranking use the exact authored
    /// geometry. Touching is contact with zero depth; an overlap smaller than
    /// half a raw unit can also round to zero. A zero radius is the finite axis
    /// segment.
    /// <para>
    /// The normal points from the cone toward the prism. Depth and each
    /// unit-normal component round nearest-even after selection. Unclamped
    /// depth differs from the exact minimum by at most half a raw unit;
    /// translating along the rounded normal additionally introduces an error
    /// proportional to depth. <see cref="FixedContactAnchors.DepthIsClamped"/>
    /// identifies an exact minimum above <see cref="Fixed64.MaxValue"/>.
    /// </para>
    /// <para>
    /// Both finite support witnesses are matched before final coordinate
    /// rounding. The first anchor retains the cone center and the second the
    /// prism origin; both use the cone rotation as their local frame. The
    /// prism's scalar yaw remains authoritative. Use the anchor operations to
    /// compare or materialize points rather than interpreting the second
    /// anchor's local coordinates as prism footprint offsets.
    /// </para>
    /// </remarks>
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
