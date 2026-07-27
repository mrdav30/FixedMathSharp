//=======================================================================
// FixedTriangle.CapsuleSlab.cs
//=======================================================================
// MIT License, Copyright (c) 2024-present David Oravsky (mrdav30)
// See LICENSE file in the project root for full license information.
//=======================================================================

using System;

namespace FixedMathSharp.Geometry;

/// <content>
/// Contact generation between a rigidly transformed triangle and vertical
/// circle/capsule slabs.
/// </content>
public partial struct FixedTriangle
{
    /// <summary>
    /// Attempts to construct canonical contact anchors between this rigidly
    /// transformed triangle and one vertical circle slab.
    /// </summary>
    public readonly bool TryGetCircleSlabContact(
        Vector3d triangleOrigin,
        FixedQuaternion triangleRotation,
        Vector3d slabCenter,
        Fixed64 circleFrameRotation,
        Fixed64 slabHalfThickness,
        Fixed64 circleRadius,
        out FixedContactAnchors contact)
    {
        ValidateRigidTriangleFrame(triangleRotation);
        ValidateCapsuleSlab(
            Vector2d.Right,
            Fixed64.Zero,
            circleRadius,
            slabHalfThickness);
        return WideOrientedBox.TryGetTriangleCapsuleSlabContact(
            triangleOrigin,
            triangleRotation,
            this,
            slabCenter,
            circleFrameRotation,
            Vector2d.Right,
            Fixed64.Zero,
            circleRadius,
            slabHalfThickness,
            out contact);
    }

    /// <summary>
    /// Attempts to construct canonical contact anchors between this rigidly
    /// transformed triangle and one vertical centered-capsule slab.
    /// </summary>
    public readonly bool TryGetCenteredCapsuleSlabContact(
        Vector3d triangleOrigin,
        FixedQuaternion triangleRotation,
        Vector3d slabCenter,
        Fixed64 capsuleFrameRotation,
        Vector2d localCapsuleAxisDirection,
        Fixed64 capsuleAxisLength,
        Fixed64 capsuleRadius,
        Fixed64 slabHalfThickness,
        out FixedContactAnchors contact)
    {
        ValidateRigidTriangleFrame(triangleRotation);
        ValidateCapsuleSlab(
            localCapsuleAxisDirection,
            capsuleAxisLength,
            capsuleRadius,
            slabHalfThickness);
        return WideOrientedBox.TryGetTriangleCapsuleSlabContact(
            triangleOrigin,
            triangleRotation,
            this,
            slabCenter,
            capsuleFrameRotation,
            localCapsuleAxisDirection,
            capsuleAxisLength,
            capsuleRadius,
            slabHalfThickness,
            out contact);
    }

    private static void ValidateCapsuleSlab(
        Vector2d localAxisDirection,
        Fixed64 axisLength,
        Fixed64 radius,
        Fixed64 halfThickness)
    {
        if (!localAxisDirection.IsNormalized())
        {
            throw new ArgumentException(
                "Capsule axis direction must be normalized.",
                nameof(localAxisDirection));
        }
        if (axisLength < Fixed64.Zero)
            throw new ArgumentOutOfRangeException(nameof(axisLength));
        if (radius < Fixed64.Zero)
            throw new ArgumentOutOfRangeException(nameof(radius));
        if (halfThickness <= Fixed64.Zero)
            throw new ArgumentOutOfRangeException(nameof(halfThickness));
    }
}
