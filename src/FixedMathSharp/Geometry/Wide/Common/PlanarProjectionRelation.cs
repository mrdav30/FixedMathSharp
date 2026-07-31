//=======================================================================
// PlanarProjectionRelation.cs
//=======================================================================
// MIT License, Copyright (c) 2024-present David Oravsky (mrdav30)
// See LICENSE file in the project root for full license information.
//=======================================================================

namespace FixedMathSharp.Geometry;

/// <summary>
/// Retains the representable distance and offset from a planar query point to
/// an exact projected solid.
/// </summary>
internal readonly struct PlanarProjectionRelation
{
    internal static readonly PlanarProjectionRelation Contained =
        new(
            Fixed64.Zero,
            Vector2d.Zero,
            Vector2d.Zero,
            isContained: true);

    internal PlanarProjectionRelation(
        Fixed64 distance,
        Vector2d offset,
        Vector2d direction)
        : this(distance, offset, direction, isContained: false)
    {
    }

    private PlanarProjectionRelation(
        Fixed64 distance,
        Vector2d offset,
        Vector2d direction,
        bool isContained)
    {
        Distance = distance;
        Offset = offset;
        Direction = direction;
        IsContained = isContained;
    }

    internal Fixed64 Distance { get; }

    internal Vector2d Offset { get; }

    internal Vector2d Direction { get; }

    internal bool IsContained { get; }
}
