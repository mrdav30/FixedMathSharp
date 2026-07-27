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

namespace FixedMathSharp.Geometry;

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
    /// Gets the point at the specified ray parameter with one final
    /// round-half-to-even conversion per coordinate.
    /// </summary>
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public Vector3d GetPoint(Fixed64 parameter) => new(
        Fixed64.MultiplyAdd(Direction.X, parameter, Position.X),
        Fixed64.MultiplyAdd(Direction.Y, parameter, Position.Y),
        Fixed64.MultiplyAdd(Direction.Z, parameter, Position.Z));

    /// <summary>
    /// Attempts to get the point at the specified ray parameter with one final
    /// round-half-to-even conversion per coordinate.
    /// </summary>
    /// <param name="parameter">The parametric distance along the ray direction.</param>
    /// <param name="point">
    /// The point when every final coordinate is representable; otherwise,
    /// <see langword="default"/>.
    /// </param>
    /// <returns>
    /// <see langword="true"/> when every final coordinate is representable;
    /// otherwise, <see langword="false"/>.
    /// </returns>
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public readonly bool TryGetPoint(Fixed64 parameter, out Vector3d point)
    {
        if (!Fixed64.TryMultiplyAdd(Direction.X, parameter, Position.X, out Fixed64 x)
            || !Fixed64.TryMultiplyAdd(Direction.Y, parameter, Position.Y, out Fixed64 y)
            || !Fixed64.TryMultiplyAdd(Direction.Z, parameter, Position.Z, out Fixed64 z))
        {
            point = default;
            return false;
        }

        point = new Vector3d(x, y, z);
        return true;
    }

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
    public Fixed64? Intersects(FixedBoundBox box)
    {
        return IntersectsBoxLike(box.Min, box.Max);
    }

    /// <summary>
    /// Finds the first forward intersection with the specified bounding sphere.
    /// </summary>
    public Fixed64? Intersects(FixedBoundSphere sphere) =>
        WideRayIntersection.Intersects(Position, Direction, sphere, Fixed64.MaxValue);

    /// <summary>
    /// Finds the first forward intersection with the specified bounding sphere
    /// at or before <paramref name="maxParameter"/>.
    /// </summary>
    /// <remarks>
    /// Offset differences, quadratic products, the discriminant, and root
    /// ordering are evaluated without fixed-point saturation. The returned
    /// parameter uses deterministic round-half-to-even conversion.
    /// </remarks>
    public Fixed64? Intersects(FixedBoundSphere sphere, Fixed64 maxParameter) =>
        WideRayIntersection.Intersects(Position, Direction, sphere, maxParameter);

    /// <summary>
    /// Finds the first forward intersection with the specified bounding sphere,
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
        FixedBoundSphere sphere,
        Fixed64 radiusExpansion,
        Fixed64 maxParameter) =>
        WideRayIntersection.Intersects(Position, Direction, sphere, radiusExpansion, maxParameter);

    /// <summary>
    /// Gets the closed parameter interval where this ray overlaps the sphere,
    /// clipped to <c>[0, <paramref name="maxParameter"/>]</c>.
    /// </summary>
    /// <remarks>
    /// Direction need not be normalized. Exact root clipping precedes
    /// deterministic round-half-to-even conversion of both endpoints.
    /// </remarks>
    public bool TryGetIntersectionInterval(
        FixedBoundSphere sphere,
        Fixed64 maxParameter,
        out Fixed64 entry,
        out Fixed64 exit) =>
        WideRayIntersection.TryGetInterval(
            Position,
            Direction,
            sphere,
            maxParameter,
            out entry,
            out exit);

    /// <summary>
    /// Gets the closed parameter interval where this ray overlaps the sphere
    /// expanded by <paramref name="radiusExpansion"/>, clipped to
    /// <c>[0, <paramref name="maxParameter"/>]</c>.
    /// </summary>
    /// <exception cref="ArgumentOutOfRangeException">
    /// <paramref name="radiusExpansion"/> is negative.
    /// </exception>
    public bool TryGetIntersectionInterval(
        FixedBoundSphere sphere,
        Fixed64 radiusExpansion,
        Fixed64 maxParameter,
        out Fixed64 entry,
        out Fixed64 exit) =>
        WideRayIntersection.TryGetInterval(
            Position,
            Direction,
            sphere,
            radiusExpansion,
            maxParameter,
            out entry,
            out exit);

    /// <summary>
    /// Finds the first forward intersection with the specified frustum.
    /// </summary>
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public Fixed64? Intersects(FixedBoundFrustum frustum)
    {
        return frustum.Intersects(this);
    }

    private Fixed64? IntersectsBoxLike(Vector3d min, Vector3d max)
    {
        Fixed64 tMin = Fixed64.Zero;
        Fixed64 tMax = Fixed64.MaxValue;

        if (!ClipAxis(Position.X, Direction.X, min.X, max.X, ref tMin, ref tMax))
            return null;

        if (!ClipAxis(Position.Y, Direction.Y, min.Y, max.Y, ref tMin, ref tMax))
            return null;

        if (!ClipAxis(Position.Z, Direction.Z, min.Z, max.Z, ref tMin, ref tMax))
            return null;

        return tMin;
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
