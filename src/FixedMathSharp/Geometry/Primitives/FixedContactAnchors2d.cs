//=======================================================================
// FixedContactAnchors2d.cs
//=======================================================================
// MIT License, Copyright (c) 2024–present David Oravsky (mrdav30)
// See LICENSE file in the project root for full license information.
//=======================================================================

namespace FixedMathSharp.Geometry;

/// <summary>
/// Describes one planar contact constraint using points retained in their
/// rigid local frames.
/// </summary>
public readonly struct FixedContactAnchors2d
{
    internal FixedContactAnchors2d(
        FixedPointAnchor2d firstAnchor,
        FixedPointAnchor2d secondAnchor,
        Vector2d normal,
        Fixed64 depth,
        bool depthIsClamped)
    {
        FirstAnchor = firstAnchor;
        SecondAnchor = secondAnchor;
        Normal = normal;
        Depth = depth;
        DepthIsClamped = depthIsClamped;
    }

    /// <summary>
    /// The contact point retained in the first shape's rigid frame.
    /// </summary>
    public FixedPointAnchor2d FirstAnchor { get; }

    /// <summary>
    /// The contact point retained in the second shape's rigid frame.
    /// </summary>
    public FixedPointAnchor2d SecondAnchor { get; }

    /// <summary>
    /// The contact normal directed from the first shape toward the second.
    /// </summary>
    public Vector2d Normal { get; }

    /// <summary>
    /// The representable penetration depth.
    /// </summary>
    public Fixed64 Depth { get; }

    /// <summary>
    /// Whether the conceptual penetration exceeds <see cref="Fixed64.MaxValue"/>.
    /// </summary>
    public bool DepthIsClamped { get; }
}
