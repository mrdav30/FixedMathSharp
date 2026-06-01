//=======================================================================
// FixedRay.cs
//=======================================================================
// MIT License, Copyright (c) 2024–present David Oravsky (mrdav30)
// See LICENSE file in the project root for full license information.
//=======================================================================

using MemoryPack;
using System;
using System.Runtime.CompilerServices;
using System.Text.Json.Serialization;

namespace FixedMathSharp;

/// <summary>
/// Represents a ray with an origin and direction in three-dimensional space.
/// </summary>
/// <remarks>
/// Intersection methods return the ray parameter for the first forward hit. If <see cref="Direction"/> is normalized,
/// that parameter is also the distance from <see cref="Position"/>.
/// </remarks>
[Serializable]
[MemoryPackable]
public partial struct FixedRay : IEquatable<FixedRay>
{
    #region Fields

    /// <summary>
    /// The origin of the ray.
    /// </summary>
    [JsonInclude]
    [MemoryPackOrder(0)]
    public Vector3d Position;

    /// <summary>
    /// The direction of the ray.
    /// </summary>
    [JsonInclude]
    [MemoryPackOrder(1)]
    public Vector3d Direction;

    #endregion

    #region Constructors

    /// <summary>
    /// Initializes a new ray with the specified origin and direction.
    /// </summary>
    [JsonConstructor]
    public FixedRay(Vector3d position, Vector3d direction)
    {
        Position = position;
        Direction = direction;
    }

    #endregion

    #region Methods

    /// <summary>
    /// Finds the first forward intersection with the specified plane.
    /// </summary>
    public Fixed64? Intersects(FixedPlane plane)
    {
        Fixed64 denominator = plane.DotNormal(Direction);
        if (IsNearlyZero(denominator))
            return null;

        Fixed64 t = -plane.DotCoordinate(Position) / denominator;
        return t < Fixed64.Zero ? null : t;
    }

    /// <summary>
    /// Finds the first forward intersection with the specified bounding box.
    /// </summary>
    public Fixed64? Intersects(BoundingBox box)
    {
        return IntersectsBoxLike(box.Min, box.Max);
    }

    /// <summary>
    /// Finds the first forward intersection with the specified bounding area.
    /// </summary>
    public Fixed64? Intersects(BoundingArea area)
    {
        return IntersectsBoxLike(area.Min, area.Max);
    }

    /// <summary>
    /// Finds the first forward intersection with the specified bounding sphere.
    /// </summary>
    public Fixed64? Intersects(BoundingSphere sphere)
    {
        Fixed64 directionLengthSquared = Direction.SqrMagnitude;
        if (directionLengthSquared == Fixed64.Zero)
            return sphere.Contains(Position) ? Fixed64.Zero : null;

        Vector3d offset = Position - sphere.Center;
        Fixed64 c = Vector3d.Dot(offset, offset) - sphere.SqrRadius;
        if (c <= Fixed64.Zero)
            return Fixed64.Zero;

        Fixed64 b = Vector3d.Dot(offset, Direction);
        if (b > Fixed64.Zero)
            return null;

        Fixed64 discriminant = (b * b) - (directionLengthSquared * c);
        if (discriminant < Fixed64.Zero)
            return null;

        Fixed64 t = (-b - FixedMath.Sqrt(discriminant)) / directionLengthSquared;
        return t < Fixed64.Zero ? null : t;
    }

    /// <summary>
    /// Finds the first forward intersection with the specified frustum.
    /// </summary>
    public Fixed64? Intersects(BoundingFrustum frustum)
    {
        if (frustum == null)
            throw new ArgumentNullException(nameof(frustum));

        return frustum.Intersects(this);
    }

    private Fixed64? IntersectsBoxLike(Vector3d min, Vector3d max)
    {
        Fixed64 tMin = Fixed64.Zero;
        Fixed64 tMax = Fixed64.MAX_VALUE;

        if (!ClipAxis(Position.x, Direction.x, min.x, max.x, ref tMin, ref tMax))
            return null;

        if (!ClipAxis(Position.y, Direction.y, min.y, max.y, ref tMin, ref tMax))
            return null;

        if (!ClipAxis(Position.z, Direction.z, min.z, max.z, ref tMin, ref tMax))
            return null;

        return tMax < Fixed64.Zero ? null : tMin;
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
        if (IsNearlyZero(direction))
            return position >= min && position <= max;

        Fixed64 t1 = (min - position) / direction;
        Fixed64 t2 = (max - position) / direction;

        if (t1 > t2)
        {
            Fixed64 temp = t1;
            t1 = t2;
            t2 = temp;
        }

        if (t1 > tMin)
            tMin = t1;

        if (t2 < tMax)
            tMax = t2;

        return tMin <= tMax;
    }

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    internal static bool IsNearlyZero(Fixed64 value)
    {
        return value.Abs() <= Fixed64.Epsilon;
    }

    /// <summary>
    /// Deconstructs the ray into its origin and direction.
    /// </summary>
    public void Deconstruct(out Vector3d position, out Vector3d direction)
    {
        position = Position;
        direction = Direction;
    }

    #endregion

    #region Operators

    /// <summary>
    /// Determines whether two rays are equal.
    /// </summary>
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public static bool operator ==(FixedRay left, FixedRay right) => left.Equals(right);

    /// <summary>
    /// Determines whether two rays are not equal.
    /// </summary>
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public static bool operator !=(FixedRay left, FixedRay right) => !left.Equals(right);

    #endregion

    #region Equality

    /// <inheritdoc/>
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public override bool Equals(object? obj) => obj is FixedRay other && Equals(other);

    /// <inheritdoc/>
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public bool Equals(FixedRay other)
    {
        return Position.Equals(other.Position) && Direction.Equals(other.Direction);
    }

    /// <inheritdoc/>
    public override int GetHashCode()
    {
        unchecked
        {
            int hash = 17;
            hash = hash * 23 + Position.GetHashCode();
            hash = hash * 23 + Direction.GetHashCode();
            return hash;
        }
    }

    #endregion
}
