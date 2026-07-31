//=======================================================================
// FixedQuaternion.ScaledTransform.cs
//=======================================================================
// MIT License, Copyright (c) 2024–present David Oravsky (mrdav30)
// See LICENSE file in the project root for full license information.
//=======================================================================

namespace FixedMathSharp;

/// <content>
/// Provides scaled-point transformation helpers for combining rotation, scale,
/// and displacement into a single deterministic world-space result.
/// </content>
public partial struct FixedQuaternion
{
    /// <summary>
    /// Attempts to transform a component-scaled local point by this rotation
    /// and a world origin with one final round-half-to-even conversion per
    /// component.
    /// </summary>
    public bool TryTransformScaledPoint(
        Vector3d origin,
        Vector3d localPoint,
        Vector3d scale,
        out Vector3d result) =>
        TryTransformScaledPoint(
            origin,
            localPoint,
            scale,
            Vector3d.Zero,
            out result);

    /// <summary>
    /// Attempts to transform a component-scaled local point plus an unscaled
    /// local displacement by this rotation and a world origin with one final
    /// round-half-to-even conversion per component.
    /// </summary>
    /// <remarks>
    /// Computes
    /// <c>origin + Rotate(scale * localPoint + localDisplacement)</c> without
    /// narrowing the scaled point, local sum, or rotated offset independently.
    /// The zero quaternion preserves the legacy rotation contract and returns
    /// <paramref name="origin"/>.
    /// </remarks>
    public bool TryTransformScaledPoint(
        Vector3d origin,
        Vector3d localPoint,
        Vector3d scale,
        Vector3d localDisplacement,
        out Vector3d result)
    {
        if (this == Zero)
        {
            result = origin;
            return true;
        }

        return WideVector3dTransform.TryTransformScaledPoint(
            origin,
            this,
            localPoint,
            scale,
            localDisplacement,
            out result);
    }

    /// <summary>
    /// Attempts to inverse-transform a world point by this rotation, a world
    /// origin, and a component scale with one final round-half-to-even
    /// conversion per local component.
    /// </summary>
    /// <remarks>
    /// Computes <c>InverseRotate(worldPoint - origin) / scale</c> without
    /// narrowing the world offset or rotated point independently. A zero
    /// quaternion or any zero scale component returns <see langword="false"/>.
    /// </remarks>
    /// <param name="origin">The world-space origin of the scaled local frame.</param>
    /// <param name="worldPoint">The world-space point to inverse-transform.</param>
    /// <param name="scale">The component scale of the local frame.</param>
    /// <param name="result">
    /// The local-space point on success; otherwise zero.
    /// </param>
    /// <returns>
    /// <see langword="true"/> when this quaternion is nonzero, every scale
    /// component is nonzero, and every final local coordinate is representable;
    /// otherwise <see langword="false"/>.
    /// </returns>
    public bool TryInverseTransformScaledPoint(
        Vector3d origin,
        Vector3d worldPoint,
        Vector3d scale,
        out Vector3d result)
    {
        if (this == Zero
            || scale.X == Fixed64.Zero
            || scale.Y == Fixed64.Zero
            || scale.Z == Fixed64.Zero)
        {
            result = default;
            return false;
        }

        return WideVector3dTransform.TryInverseTransformScaledPoint(
            origin,
            this,
            worldPoint,
            scale,
            out result);
    }
}
