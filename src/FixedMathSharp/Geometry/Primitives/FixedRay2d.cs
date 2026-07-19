//=======================================================================
// FixedRay2d.cs
//=======================================================================
// MIT License, Copyright (c) 2024–present David Oravsky (mrdav30)
// See LICENSE file in the project root for full license information.
//=======================================================================

using MemoryPack;
using System;
using System.Runtime.CompilerServices;
using System.Text.Json.Serialization;

namespace FixedMathSharp.Bounds;

/// <summary>
/// Represents a ray with an origin and direction in two-dimensional fixed-point space.
/// </summary>
/// <remarks>
/// The direction is not normalized by construction. Intersection methods return
/// the ray parameter for the first forward hit; when <see cref="Direction"/> is
/// normalized, that parameter is also the distance from <see cref="Position"/>.
/// </remarks>
[Serializable]
[MemoryPackable]
public partial struct FixedRay2d : IEquatable<FixedRay2d>
{
    #region Fields

    /// <summary>
    /// The origin of the ray.
    /// </summary>
    [JsonInclude]
    [MemoryPackOrder(0)]
    public Vector2d Position;

    /// <summary>
    /// The direction of the ray.
    /// </summary>
    [JsonInclude]
    [MemoryPackOrder(1)]
    public Vector2d Direction;

    #endregion

    #region Constructors

    /// <summary>
    /// Initializes a new ray with the specified origin and direction.
    /// </summary>
    [JsonConstructor]
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public FixedRay2d(Vector2d position, Vector2d direction)
    {
        Position = position;
        Direction = direction;
    }

    #endregion

    #region Methods

    /// <summary>
    /// Gets the point at the specified ray parameter.
    /// </summary>
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public Vector2d GetPoint(Fixed64 distance) => Position + (Direction * distance);

    /// <summary>
    /// Finds the first forward intersection with the specified bounding area, including boundary-only contact.
    /// </summary>
    public Fixed64? Intersects(FixedBoundArea area)
    {
        Fixed64 tMin = Fixed64.Zero;
        Fixed64 tMax = Fixed64.MaxValue;

        if (!ClipAxis(Position.X, Direction.X, area.Min.X, area.Max.X, ref tMin, ref tMax))
            return null;

        if (!ClipAxis(Position.Y, Direction.Y, area.Min.Y, area.Max.Y, ref tMin, ref tMax))
            return null;

        return tMin;
    }

    /// <summary>
    /// Finds the first forward intersection with the specified bounding circle, including boundary-only contact.
    /// </summary>
    public Fixed64? Intersects(FixedBoundCircle circle) =>
        WideRayIntersection.Intersects(Position, Direction, circle, Fixed64.MaxValue);

    /// <summary>
    /// Finds the first forward intersection with the specified bounding circle
    /// at or before <paramref name="maxParameter"/>.
    /// </summary>
    /// <remarks>
    /// Offset differences, quadratic products, the discriminant, and root
    /// ordering are evaluated without fixed-point saturation. The returned
    /// parameter uses deterministic round-half-to-even conversion.
    /// </remarks>
    public Fixed64? Intersects(FixedBoundCircle circle, Fixed64 maxParameter) =>
        WideRayIntersection.Intersects(Position, Direction, circle, maxParameter);

    /// <summary>
    /// Finds the first forward intersection with the specified bounding circle,
    /// expanded by <paramref name="radiusExpansion"/>, at or before
    /// <paramref name="maxParameter"/>.
    /// </summary>
    /// <remarks>
    /// The two radii are combined in wide arithmetic, so their sum may exceed
    /// <see cref="Fixed64.MaxValue"/> without saturation.
    /// </remarks>
    /// <exception cref="ArgumentOutOfRangeException">
    /// <paramref name="radiusExpansion"/> is negative.
    /// </exception>
    public Fixed64? Intersects(
        FixedBoundCircle circle,
        Fixed64 radiusExpansion,
        Fixed64 maxParameter) =>
        WideRayIntersection.Intersects(Position, Direction, circle, radiusExpansion, maxParameter);

    /// <summary>
    /// Deconstructs the ray into origin and direction.
    /// </summary>
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public void Deconstruct(out Vector2d position, out Vector2d direction)
    {
        position = Position;
        direction = Direction;
    }

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    private static bool ClipAxis(
        Fixed64 position,
        Fixed64 direction,
        Fixed64 min,
        Fixed64 max,
        ref Fixed64 tMin,
        ref Fixed64 tMax)
    {
        if (direction == Fixed64.Zero)
            return position >= min && position <= max;

        Fixed64 t1 = (min - position) / direction;
        Fixed64 t2 = (max - position) / direction;

        if (t1 > t2)
            (t2, t1) = (t1, t2);

        if (t1 > tMin)
            tMin = t1;

        if (t2 < tMax)
            tMax = t2;

        return tMin <= tMax;
    }

    #endregion

    #region Operators

    /// <summary>
    /// Determines whether two rays are equal.
    /// </summary>
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public static bool operator ==(FixedRay2d left, FixedRay2d right) => left.Equals(right);

    /// <summary>
    /// Determines whether two rays are not equal.
    /// </summary>
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public static bool operator !=(FixedRay2d left, FixedRay2d right) => !left.Equals(right);

    #endregion

    #region Equality

    /// <inheritdoc />
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public bool Equals(FixedRay2d other)
    {
        return Position == other.Position && Direction == other.Direction;
    }

    /// <inheritdoc />
    public override bool Equals(object? obj)
    {
        return obj is FixedRay2d other && Equals(other);
    }

    /// <inheritdoc />
    public override int GetHashCode()
    {
        unchecked
        {
            int hash = 17;
            hash = (hash * 31) + Position.StateHash;
            hash = (hash * 31) + Direction.StateHash;
            return hash;
        }
    }

    #endregion
}
