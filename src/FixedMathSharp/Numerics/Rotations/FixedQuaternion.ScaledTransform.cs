//=======================================================================
// FixedQuaternion.ScaledTransform.cs
//=======================================================================
// MIT License, Copyright (c) 2024–present David Oravsky (mrdav30)
// See LICENSE file in the project root for full license information.
//=======================================================================

using FixedMathSharp.Geometry;

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

        return WideOrientedBox.TryMaterializeScaledLocalPoint(
            origin,
            this,
            localPoint,
            scale,
            localDisplacement,
            out result);
    }
}
