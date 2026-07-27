//=======================================================================
// FixedContactAnchors.cs
//=======================================================================
// MIT License, Copyright (c) 2024–present David Oravsky (mrdav30)
// See LICENSE file in the project root for full license information.
//=======================================================================

namespace FixedMathSharp.Bounds;

/// <summary>
/// Describes one 3D contact constraint using points retained in their rigid
/// local frames.
/// </summary>
public readonly struct FixedContactAnchors
{
    internal FixedContactAnchors(
        FixedPointAnchor firstAnchor,
        FixedPointAnchor secondAnchor,
        Vector3d normal,
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
    public FixedPointAnchor FirstAnchor { get; }

    /// <summary>
    /// The contact point retained in the second shape's rigid frame.
    /// </summary>
    public FixedPointAnchor SecondAnchor { get; }

    /// <summary>
    /// The contact normal directed from the first shape toward the second.
    /// </summary>
    public Vector3d Normal { get; }

    /// <summary>
    /// The representable penetration depth.
    /// </summary>
    public Fixed64 Depth { get; }

    /// <summary>
    /// Whether the conceptual penetration exceeds <see cref="Fixed64.MaxValue"/>.
    /// </summary>
    public bool DepthIsClamped { get; }
}
