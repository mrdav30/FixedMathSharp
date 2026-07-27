//=======================================================================
// FixedContactLocalPoints.cs
//=======================================================================
// MIT License, Copyright (c) 2024–present David Oravsky (mrdav30)
// See LICENSE file in the project root for full license information.
//=======================================================================

namespace FixedMathSharp.Geometry;

/// <summary>
/// Stores one compact local-point pair in a multi-contact relation.
/// </summary>
/// <remarks>
/// The pair reuses the rigid frames, normal, depth, and depth-clamping state
/// carried by the relation's primary <see cref="FixedContactAnchors"/>.
/// </remarks>
public readonly struct FixedContactLocalPoints
{
    internal FixedContactLocalPoints(
        Vector3d firstLocalPoint,
        Vector3d secondLocalPoint)
    {
        FirstLocalPoint = firstLocalPoint;
        SecondLocalPoint = secondLocalPoint;
    }

    /// <summary>
    /// The point in the primary relation's first rigid frame.
    /// </summary>
    public Vector3d FirstLocalPoint { get; }

    /// <summary>
    /// The point in the primary relation's second rigid frame.
    /// </summary>
    public Vector3d SecondLocalPoint { get; }
}
