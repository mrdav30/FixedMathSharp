//=======================================================================
// FixedRay2d.cs
//=======================================================================
// MIT License, Copyright (c) 2024-present David Oravsky (mrdav30)
// See LICENSE file in the project root for full license information.
//=======================================================================

using System;
using System.Runtime.CompilerServices;
using System.Text.Json.Serialization;
using MemoryPack;

namespace FixedMathSharp.Geometry;

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
    /// <param name="position">The ray origin in the supplied coordinate space.</param>
    /// <param name="direction">The direction vector in the origin's coordinate space. The ray stores it as supplied.</param>
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
    /// <param name="parameter">The multiplier on the stored direction. Negative values evaluate points behind the origin.</param>
    /// <returns>The origin plus the scaled direction, rounded nearest-even and saturated per coordinate.</returns>
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public Vector2d GetPoint(Fixed64 parameter) => new(
        Fixed64.MultiplyAdd(Direction.X, parameter, Position.X),
        Fixed64.MultiplyAdd(Direction.Y, parameter, Position.Y));

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
    public readonly bool TryGetPoint(Fixed64 parameter, out Vector2d point)
    {
        if (!Fixed64.TryMultiplyAdd(Direction.X, parameter, Position.X, out Fixed64 x)
            || !Fixed64.TryMultiplyAdd(Direction.Y, parameter, Position.Y, out Fixed64 y))
        {
            point = default;
            return false;
        }

        point = new Vector2d(x, y);
        return true;
    }

    /// <summary>Tests the ray against the bound using fixed-point slabs.</summary>
    /// <param name="area">The area in the same coordinate space as this ray.</param>
    /// <returns>
    /// The computed nonnegative entry parameter, or <see langword="null"/> when the computed interval is empty.
    /// Starting inside or on the boundary returns zero.
    /// </returns>
    /// <remarks>
    /// The slab endpoint differences and division use saturating fixed-point arithmetic. A reported parameter can
    /// be <see cref="Fixed64.MaxValue"/> when the geometric entry lies beyond the scalar range.
    /// </remarks>
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
    /// <param name="circle">The circle in the same coordinate space as this ray.</param>
    /// <returns>
    /// The first nonnegative ray parameter, or <see langword="null"/> when no hit lies in the tested interval.
    /// Starting inside or on the boundary returns zero.
    /// </returns>
    public Fixed64? Intersects(FixedBoundCircle circle) =>
        WideRayIntersection.Intersects(Position, Direction, circle, Fixed64.MaxValue);

    /// <summary>
    /// Finds the first forward intersection with the specified bounding circle
    /// at or before <paramref name="maxParameter"/>.
    /// </summary>
    /// <param name="circle">The circle in the same coordinate space as this ray.</param>
    /// <param name="maxParameter">The inclusive upper bound on the ray parameter. A negative value gives an empty interval.</param>
    /// <returns>
    /// The first nonnegative ray parameter, or <see langword="null"/> when no hit lies in the tested interval.
    /// Starting inside or on the boundary returns zero.
    /// </returns>
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
    /// <param name="circle">The circle in the same coordinate space as this ray.</param>
    /// <param name="radiusExpansion">A nonnegative addition to the radius, in coordinate units.</param>
    /// <param name="maxParameter">The inclusive upper bound on the ray parameter. A negative value gives an empty interval.</param>
    /// <returns>
    /// The first nonnegative ray parameter, or <see langword="null"/> when no hit lies in the tested interval.
    /// Starting inside or on the boundary returns zero.
    /// </returns>
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
    /// Gets the closed parameter interval where this ray overlaps the circle,
    /// clipped to <c>[0, <paramref name="maxParameter"/>]</c>.
    /// </summary>
    /// <param name="circle">The circle in the same coordinate space as this ray.</param>
    /// <param name="maxParameter">The inclusive upper bound on the ray parameter. A negative value gives an empty interval.</param>
    /// <param name="entry">The lower bound of the overlap interval on success, or the default value on failure.</param>
    /// <param name="exit">The upper bound of the overlap interval on success, or the default value on failure.</param>
    /// <returns>
    /// <see langword="true"/> when the ray overlaps the closed shape within the requested parameter interval and
    /// <see langword="false"/> otherwise.
    /// </returns>
    /// <remarks>
    /// Direction need not be normalized. Exact root clipping precedes
    /// deterministic round-half-to-even conversion of both endpoints.
    /// </remarks>
    public bool TryGetIntersectionInterval(
        FixedBoundCircle circle,
        Fixed64 maxParameter,
        out Fixed64 entry,
        out Fixed64 exit) =>
        WideRayIntersection.TryGetInterval(
            Position,
            Direction,
            circle,
            maxParameter,
            out entry,
            out exit);

    /// <summary>
    /// Gets the closed parameter interval where this ray overlaps the circle
    /// expanded by <paramref name="radiusExpansion"/>, clipped to
    /// <c>[0, <paramref name="maxParameter"/>]</c>.
    /// </summary>
    /// <param name="circle">The circle in the same coordinate space as this ray.</param>
    /// <param name="radiusExpansion">A nonnegative addition to the radius, in coordinate units.</param>
    /// <param name="maxParameter">The inclusive upper bound on the ray parameter. A negative value gives an empty interval.</param>
    /// <param name="entry">The lower bound of the overlap interval on success, or the default value on failure.</param>
    /// <param name="exit">The upper bound of the overlap interval on success, or the default value on failure.</param>
    /// <returns>
    /// <see langword="true"/> when the ray overlaps the closed shape within the requested parameter interval and
    /// <see langword="false"/> otherwise.
    /// </returns>
    /// <exception cref="ArgumentOutOfRangeException">
    /// <paramref name="radiusExpansion"/> is negative.
    /// </exception>
    public bool TryGetIntersectionInterval(
        FixedBoundCircle circle,
        Fixed64 radiusExpansion,
        Fixed64 maxParameter,
        out Fixed64 entry,
        out Fixed64 exit) =>
        WideRayIntersection.TryGetInterval(
            Position,
            Direction,
            circle,
            radiusExpansion,
            maxParameter,
            out entry,
            out exit);

    /// <summary>
    /// Gets the closed parameter interval where this ray overlaps a capsule,
    /// clipped to <c>[0, <paramref name="maxParameter"/>]</c>.
    /// </summary>
    /// <param name="capsuleAxis">The central segment in the ray's coordinate space. It can have coincident endpoints.</param>
    /// <param name="radius">The nonnegative radius in coordinate units.</param>
    /// <param name="maxParameter">The inclusive upper bound on the ray parameter. A negative value gives an empty interval.</param>
    /// <param name="entryParameter">
    /// The lower ray parameter of the overlap interval on success, or the default value on failure.
    /// </param>
    /// <param name="exitParameter">
    /// The upper ray parameter of the overlap interval on success, or the default value on failure.
    /// </param>
    /// <returns>
    /// <see langword="true"/> when the ray overlaps the closed shape within the requested parameter interval and
    /// <see langword="false"/> otherwise.
    /// </returns>
    /// <exception cref="ArgumentOutOfRangeException"><paramref name="radius"/> is negative.</exception>
    public readonly bool TryGetCapsuleIntersectionInterval(
        FixedSegment2d capsuleAxis,
        Fixed64 radius,
        Fixed64 maxParameter,
        out Fixed64 entryParameter,
        out Fixed64 exitParameter) =>
        TryGetCapsuleIntersectionInterval(
            capsuleAxis,
            radius,
            Fixed64.Zero,
            maxParameter,
            out entryParameter,
            out exitParameter,
            out _,
            out _);

    /// <summary>
    /// Gets the closed parameter interval where this ray overlaps a radially
    /// expanded capsule and reports exact bounded-endpoint containment.
    /// </summary>
    /// <param name="capsuleAxis">The central segment in the ray's coordinate space. It can have coincident endpoints.</param>
    /// <param name="radius">The nonnegative radius in coordinate units.</param>
    /// <param name="radiusExpansion">A nonnegative addition to the radius, in coordinate units.</param>
    /// <param name="maxParameter">The inclusive upper bound on the ray parameter. A negative value gives an empty interval.</param>
    /// <param name="entryParameter">
    /// The lower ray parameter of the overlap interval on success, or the default value on failure.
    /// </param>
    /// <param name="exitParameter">
    /// The upper ray parameter of the overlap interval on success, or the default value on failure.
    /// </param>
    /// <param name="originContained">
    /// <see langword="true"/> if the ray origin lies on or inside the expanded shape, computed before rounding. An
    /// empty interval returns <see langword="false"/>.
    /// </param>
    /// <param name="maximumContainedStrict">
    /// <see langword="true"/> if the point at <paramref name="maxParameter"/> lies strictly inside the expanded
    /// shape, computed before rounding. An empty interval returns <see langword="false"/>.
    /// </param>
    /// <returns>
    /// <see langword="true"/> when the ray overlaps the closed shape within the requested parameter interval and
    /// <see langword="false"/> otherwise.
    /// </returns>
    /// <exception cref="ArgumentOutOfRangeException">
    /// <paramref name="radius"/> or <paramref name="radiusExpansion"/> are negative.
    /// </exception>
    /// <remarks>
    /// Direction need not be normalized. When it is normalized, returned
    /// parameters are physical distances. The origin is tested inclusively;
    /// the point at <paramref name="maxParameter"/> is tested strictly.
    /// </remarks>
    public readonly bool TryGetCapsuleIntersectionInterval(
        FixedSegment2d capsuleAxis,
        Fixed64 radius,
        Fixed64 radiusExpansion,
        Fixed64 maxParameter,
        out Fixed64 entryParameter,
        out Fixed64 exitParameter,
        out bool originContained,
        out bool maximumContainedStrict)
    {
        if (radius < Fixed64.Zero)
            throw new ArgumentOutOfRangeException(nameof(radius));
        if (radiusExpansion < Fixed64.Zero)
            throw new ArgumentOutOfRangeException(nameof(radiusExpansion));
        if (maxParameter < Fixed64.Zero)
        {
            entryParameter = default;
            exitParameter = default;
            originContained = false;
            maximumContainedStrict = false;
            return false;
        }

        return WideFiniteAxisIntersection.TryGetCapsuleInterval(
            this,
            maxParameter,
            capsuleAxis,
            radius,
            radiusExpansion,
            out entryParameter,
            out exitParameter,
            out originContained,
            out maximumContainedStrict);
    }

    /// <summary>
    /// Gets the closed parameter interval where this ray overlaps a centered
    /// capsule, clipped to <c>[0, <paramref name="maxParameter"/>]</c>.
    /// </summary>
    /// <param name="center">The shape center in world coordinates.</param>
    /// <param name="axisDirection">The normalized axis direction in world coordinates.</param>
    /// <param name="axisLength">The full nonnegative length of the central segment.</param>
    /// <param name="radius">The nonnegative radius in coordinate units.</param>
    /// <param name="maxParameter">The inclusive upper bound on the ray parameter. A negative value gives an empty interval.</param>
    /// <param name="entryParameter">
    /// The lower ray parameter of the overlap interval on success, or the default value on failure.
    /// </param>
    /// <param name="exitParameter">
    /// The upper ray parameter of the overlap interval on success, or the default value on failure.
    /// </param>
    /// <returns>
    /// <see langword="true"/> when the ray overlaps the closed shape within the requested parameter interval and
    /// <see langword="false"/> otherwise.
    /// </returns>
    /// <exception cref="ArgumentException"><paramref name="axisDirection"/> is not normalized.</exception>
    /// <exception cref="ArgumentOutOfRangeException">
    /// <paramref name="radius"/> or <paramref name="axisLength"/> are negative.
    /// </exception>
    public readonly bool TryGetCapsuleIntersectionInterval(
        Vector2d center,
        Vector2d axisDirection,
        Fixed64 axisLength,
        Fixed64 radius,
        Fixed64 maxParameter,
        out Fixed64 entryParameter,
        out Fixed64 exitParameter) =>
        TryGetCapsuleIntersectionInterval(
            center,
            axisDirection,
            axisLength,
            radius,
            Fixed64.Zero,
            maxParameter,
            out entryParameter,
            out exitParameter,
            out _,
            out _);

    /// <summary>
    /// Gets the closed parameter interval where this ray overlaps a centered,
    /// radially expanded capsule and reports exact endpoint containment.
    /// </summary>
    /// <param name="center">The shape center in world coordinates.</param>
    /// <param name="axisDirection">The normalized axis direction in world coordinates.</param>
    /// <param name="axisLength">The full nonnegative length of the central segment.</param>
    /// <param name="radius">The nonnegative radius in coordinate units.</param>
    /// <param name="radiusExpansion">A nonnegative addition to the radius, in coordinate units.</param>
    /// <param name="maxParameter">The inclusive upper bound on the ray parameter. A negative value gives an empty interval.</param>
    /// <param name="entryParameter">
    /// The lower ray parameter of the overlap interval on success, or the default value on failure.
    /// </param>
    /// <param name="exitParameter">
    /// The upper ray parameter of the overlap interval on success, or the default value on failure.
    /// </param>
    /// <param name="originContained">
    /// <see langword="true"/> if the ray origin lies on or inside the expanded shape, computed before rounding. An
    /// empty interval returns <see langword="false"/>.
    /// </param>
    /// <param name="maximumContainedStrict">
    /// <see langword="true"/> if the point at <paramref name="maxParameter"/> lies strictly inside the expanded
    /// shape, computed before rounding. An empty interval returns <see langword="false"/>.
    /// </param>
    /// <returns>
    /// <see langword="true"/> when the ray overlaps the closed shape within the requested parameter interval and
    /// <see langword="false"/> otherwise.
    /// </returns>
    /// <exception cref="ArgumentException"><paramref name="axisDirection"/> is not normalized.</exception>
    /// <exception cref="ArgumentOutOfRangeException">
    /// <paramref name="radius"/> or <paramref name="radiusExpansion"/> or <paramref name="axisLength"/> are
    /// negative.
    /// </exception>
    public readonly bool TryGetCapsuleIntersectionInterval(
        Vector2d center,
        Vector2d axisDirection,
        Fixed64 axisLength,
        Fixed64 radius,
        Fixed64 radiusExpansion,
        Fixed64 maxParameter,
        out Fixed64 entryParameter,
        out Fixed64 exitParameter,
        out bool originContained,
        out bool maximumContainedStrict)
    {
        if (!axisDirection.IsNormalized())
            throw new ArgumentException("Capsule axis direction must be normalized.", nameof(axisDirection));
        if (axisLength < Fixed64.Zero)
            throw new ArgumentOutOfRangeException(nameof(axisLength));
        if (radius < Fixed64.Zero)
            throw new ArgumentOutOfRangeException(nameof(radius));
        if (radiusExpansion < Fixed64.Zero)
            throw new ArgumentOutOfRangeException(nameof(radiusExpansion));
        if (maxParameter < Fixed64.Zero)
        {
            entryParameter = default;
            exitParameter = default;
            originContained = false;
            maximumContainedStrict = false;
            return false;
        }

        return WideFiniteAxisIntersection.TryGetCapsuleInterval(
            this,
            maxParameter,
            center,
            axisDirection,
            axisLength,
            radius,
            radiusExpansion,
            out entryParameter,
            out exitParameter,
            out originContained,
            out maximumContainedStrict);
    }

    /// <summary>
    /// Deconstructs the ray into origin and direction.
    /// </summary>
    /// <param name="position">The ray origin in the supplied coordinate space.</param>
    /// <param name="direction">The stored direction. Its magnitude is preserved.</param>
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

    /// <summary>Tests whether the stored geometry is equal.</summary>
    /// <param name="left">The first value to compare.</param>
    /// <param name="right">The second value to compare.</param>
    /// <returns><see langword="true"/> when the positions and directions match and <see langword="false"/> otherwise.</returns>
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public static bool operator ==(FixedRay2d left, FixedRay2d right) => left.Equals(right);

    /// <summary>Tests whether the stored geometry is different.</summary>
    /// <param name="left">The first value to compare.</param>
    /// <param name="right">The second value to compare.</param>
    /// <returns><see langword="true"/> when the stored values differ and <see langword="false"/> otherwise.</returns>
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public static bool operator !=(FixedRay2d left, FixedRay2d right) => !left.Equals(right);

    #endregion

    #region Equality

    /// <summary>Compares the stored geometry for equality.</summary>
    /// <param name="other">The value to compare with this instance.</param>
    /// <returns><see langword="true"/> when the positions and directions match and <see langword="false"/> otherwise.</returns>
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public bool Equals(FixedRay2d other)
    {
        return Position == other.Position && Direction == other.Direction;
    }

    /// <summary>Compares the stored geometry for equality.</summary>
    /// <param name="obj">The value to compare with this instance.</param>
    /// <returns>
    /// <see langword="true"/> when <paramref name="obj"/> is a <see cref="FixedRay2d"/> and the positions and
    /// directions match and <see langword="false"/> otherwise.
    /// </returns>
    public override bool Equals(object? obj)
    {
        return obj is FixedRay2d other && Equals(other);
    }

    /// <summary>Computes a hash code for the stored geometry.</summary>
    /// <returns>A hash code consistent with structural equality.</returns>
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
