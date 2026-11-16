//=======================================================================
// FixedRay.cs
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
    /// <param name="position">The ray origin in the supplied coordinate space.</param>
    /// <param name="direction">The direction vector in the origin's coordinate space. The ray stores it as supplied.</param>
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
    /// <param name="parameter">The multiplier on the stored direction. Negative values evaluate points behind the origin.</param>
    /// <returns>The origin plus the scaled direction, rounded nearest-even and saturated per coordinate.</returns>
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
    /// <param name="plane">The plane in the same coordinate space as this ray.</param>
    /// <returns>
    /// The nonnegative parameter at which the ray crosses the plane, or <see langword="null"/> when the crossing is
    /// behind the origin or the absolute direction-normal dot product is at most <see cref="Fixed64.Epsilon"/>.
    /// </returns>
    public Fixed64? Intersects(FixedPlane plane)
    {
        Fixed64 denominator = plane.DotNormal(Direction);
        if (IsNearlyZero(denominator))
            return null;

        Fixed64 t = -plane.DotCoordinate(Position) / denominator;
        return t < Fixed64.Zero ? null : t;
    }

    /// <summary>Finds the first forward intersection with the axis-aligned box.</summary>
    /// <param name="box">The box in the same coordinate space as this instance.</param>
    /// <returns>
    /// The entry parameter rounded half to even, or <see langword="null"/> when the exact
    /// intersection is empty within [0, <see cref="Fixed64.MaxValue"/>].
    /// Starting inside or on the boundary returns zero.
    /// </returns>
    /// <remarks>
    /// Clipping uses exact slab ratios. The rounded parameter can fall on either side of
    /// a boundary whose exact entry is between two representable parameters.
    /// </remarks>
    public Fixed64? Intersects(FixedBoundBox box)
    {
        Vector3d min = box.Min;
        Vector3d max = box.Max;
        RaySlabInterval interval = RaySlabInterval.Forward;
        return interval.Clip(Position.X, Direction.X, min.X, max.X)
            && interval.Clip(Position.Y, Direction.Y, min.Y, max.Y)
            && interval.Clip(Position.Z, Direction.Z, min.Z, max.Z)
            ? interval.RoundEntry()
            : null;
    }

    /// <summary>
    /// Finds the first forward intersection with the specified bounding sphere.
    /// </summary>
    /// <param name="sphere">The sphere in the same coordinate space as this instance.</param>
    /// <returns>
    /// The first nonnegative ray parameter, or <see langword="null"/> when no hit lies in the tested interval.
    /// Starting inside or on the boundary returns zero.
    /// </returns>
    public Fixed64? Intersects(FixedBoundSphere sphere) =>
        WideRayIntersection.Intersects(Position, Direction, sphere, Fixed64.MaxValue);

    /// <summary>
    /// Finds the first forward intersection with the specified bounding sphere
    /// at or before <paramref name="maxParameter"/>.
    /// </summary>
    /// <param name="sphere">The sphere in the same coordinate space as this instance.</param>
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
    public Fixed64? Intersects(FixedBoundSphere sphere, Fixed64 maxParameter) =>
        WideRayIntersection.Intersects(Position, Direction, sphere, maxParameter);

    /// <summary>
    /// Finds the first forward intersection with the specified bounding sphere,
    /// expanded by <paramref name="radiusExpansion"/>, at or before
    /// <paramref name="maxParameter"/>.
    /// </summary>
    /// <param name="sphere">The sphere in the same coordinate space as this instance.</param>
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
        FixedBoundSphere sphere,
        Fixed64 radiusExpansion,
        Fixed64 maxParameter) =>
        WideRayIntersection.Intersects(Position, Direction, sphere, radiusExpansion, maxParameter);

    /// <summary>
    /// Gets the closed parameter interval where this ray overlaps the sphere,
    /// clipped to <c>[0, <paramref name="maxParameter"/>]</c>.
    /// </summary>
    /// <param name="sphere">The sphere in the same coordinate space as this instance.</param>
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
    /// <param name="sphere">The sphere in the same coordinate space as this instance.</param>
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
    /// <param name="frustum">The frustum in the same coordinate space as this ray.</param>
    /// <returns>
    /// The computed nonnegative entry parameter, or <see langword="null"/> if clipping against the six planes
    /// yields an empty interval.
    /// </returns>
    /// <remarks>
    /// The test treats a direction-normal dot product with magnitude at most <see cref="Fixed64.Epsilon"/> as
    /// parallel to that plane. Clipping uses fixed-point arithmetic.
    /// </remarks>
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public Fixed64? Intersects(FixedBoundFrustum frustum)
    {
        return frustum.Intersects(this);
    }

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    internal static bool IsNearlyZero(Fixed64 value)
    {
        return value.Abs() <= Fixed64.Epsilon;
    }

    /// <summary>
    /// Deconstructs the ray into its origin and direction.
    /// </summary>
    /// <param name="position">The ray origin in the supplied coordinate space.</param>
    /// <param name="direction">The stored direction. Its magnitude is preserved.</param>
    public void Deconstruct(out Vector3d position, out Vector3d direction)
    {
        position = Position;
        direction = Direction;
    }

    #endregion

    #region Operators

    /// <summary>Tests whether the stored geometry is equal.</summary>
    /// <param name="left">The first value to compare.</param>
    /// <param name="right">The second value to compare.</param>
    /// <returns><see langword="true"/> when the positions and directions match and <see langword="false"/> otherwise.</returns>
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public static bool operator ==(FixedRay left, FixedRay right) => left.Equals(right);

    /// <summary>Tests whether the stored geometry is different.</summary>
    /// <param name="left">The first value to compare.</param>
    /// <param name="right">The second value to compare.</param>
    /// <returns><see langword="true"/> when the stored values differ and <see langword="false"/> otherwise.</returns>
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public static bool operator !=(FixedRay left, FixedRay right) => !left.Equals(right);

    #endregion

    #region Equality

    /// <summary>Compares the stored geometry for equality.</summary>
    /// <param name="obj">The value to compare with this instance.</param>
    /// <returns>
    /// <see langword="true"/> when <paramref name="obj"/> is a <see cref="FixedRay"/> and the positions and
    /// directions match and <see langword="false"/> otherwise.
    /// </returns>
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public override bool Equals(object? obj) => obj is FixedRay other && Equals(other);

    /// <summary>Compares the stored geometry for equality.</summary>
    /// <param name="other">The value to compare with this instance.</param>
    /// <returns><see langword="true"/> when the positions and directions match and <see langword="false"/> otherwise.</returns>
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public bool Equals(FixedRay other)
    {
        return Position.Equals(other.Position) && Direction.Equals(other.Direction);
    }

    /// <summary>Computes a hash code for the stored geometry.</summary>
    /// <returns>A hash code consistent with structural equality.</returns>
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
        FixedSegment capsuleAxis,
        Fixed64 radius,
        Fixed64 maxParameter,
        out Fixed64 entryParameter,
        out Fixed64 exitParameter) =>
        TryGetCapsuleIntersectionInterval(
            capsuleAxis, radius, Fixed64.Zero, maxParameter,
            out entryParameter, out exitParameter, out _, out _);

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
        FixedSegment capsuleAxis,
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
        if (!ValidateMaximum(
                maxParameter,
                out entryParameter,
                out exitParameter,
                out originContained,
                out maximumContainedStrict))
        {
            return false;
        }

        return WideFiniteAxisIntersection.TryGetCapsuleInterval(
            this, maxParameter, capsuleAxis, radius, radiusExpansion,
            out entryParameter, out exitParameter,
            out originContained, out maximumContainedStrict);
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
        Vector3d center,
        Vector3d axisDirection,
        Fixed64 axisLength,
        Fixed64 radius,
        Fixed64 maxParameter,
        out Fixed64 entryParameter,
        out Fixed64 exitParameter) =>
        TryGetCapsuleIntersectionInterval(
            center, axisDirection, axisLength, radius, Fixed64.Zero, maxParameter,
            out entryParameter, out exitParameter, out _, out _);

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
        Vector3d center,
        Vector3d axisDirection,
        Fixed64 axisLength,
        Fixed64 radius,
        Fixed64 radiusExpansion,
        Fixed64 maxParameter,
        out Fixed64 entryParameter,
        out Fixed64 exitParameter,
        out bool originContained,
        out bool maximumContainedStrict)
    {
        ValidateCenteredCapsule(axisDirection, axisLength, radius, radiusExpansion);
        if (!ValidateMaximum(
                maxParameter,
                out entryParameter,
                out exitParameter,
                out originContained,
                out maximumContainedStrict))
        {
            return false;
        }

        return WideFiniteAxisIntersection.TryGetCapsuleInterval(
            this, maxParameter, center, axisDirection, axisLength, radius, radiusExpansion,
            out entryParameter, out exitParameter,
            out originContained, out maximumContainedStrict);
    }

    /// <summary>
    /// Gets the closed parameter interval where this ray overlaps a finite
    /// cylinder, clipped to <c>[0, <paramref name="maxParameter"/>]</c>.
    /// </summary>
    /// <param name="cylinderAxis">
    /// The segment between the cap centers in the ray's coordinate space. Its endpoints must be distinct.
    /// </param>
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
    /// <exception cref="ArgumentException"><paramref name="cylinderAxis"/> has coincident endpoints.</exception>
    /// <exception cref="ArgumentOutOfRangeException"><paramref name="radius"/> is negative.</exception>
    public readonly bool TryGetFiniteCylinderIntersectionInterval(
        FixedSegment cylinderAxis,
        Fixed64 radius,
        Fixed64 maxParameter,
        out Fixed64 entryParameter,
        out Fixed64 exitParameter) =>
        TryGetFiniteCylinderIntersectionInterval(
            cylinderAxis, radius, Fixed64.Zero, maxParameter,
            out entryParameter, out exitParameter, out _, out _);

    /// <summary>
    /// Gets the closed parameter interval where this ray overlaps a radially
    /// expanded finite cylinder and reports exact endpoint containment.
    /// </summary>
    /// <param name="cylinderAxis">
    /// The segment between the cap centers in the ray's coordinate space. Its endpoints must be distinct.
    /// </param>
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
    /// <exception cref="ArgumentException"><paramref name="cylinderAxis"/> has coincident endpoints.</exception>
    /// <exception cref="ArgumentOutOfRangeException">
    /// <paramref name="radius"/> or <paramref name="radiusExpansion"/> are negative.
    /// </exception>
    public readonly bool TryGetFiniteCylinderIntersectionInterval(
        FixedSegment cylinderAxis,
        Fixed64 radius,
        Fixed64 radiusExpansion,
        Fixed64 maxParameter,
        out Fixed64 entryParameter,
        out Fixed64 exitParameter,
        out bool originContained,
        out bool maximumContainedStrict)
    {
        if (cylinderAxis.Start == cylinderAxis.End)
            throw new ArgumentException("A finite cylinder axis must have nonzero length.", nameof(cylinderAxis));
        if (radius < Fixed64.Zero)
            throw new ArgumentOutOfRangeException(nameof(radius));
        if (radiusExpansion < Fixed64.Zero)
            throw new ArgumentOutOfRangeException(nameof(radiusExpansion));
        if (!ValidateMaximum(
                maxParameter,
                out entryParameter,
                out exitParameter,
                out originContained,
                out maximumContainedStrict))
        {
            return false;
        }

        return WideFiniteAxisIntersection.TryGetFiniteCylinderInterval(
            this, maxParameter, cylinderAxis, radius, radiusExpansion,
            out entryParameter, out exitParameter,
            out originContained, out maximumContainedStrict);
    }

    /// <summary>
    /// Gets the closed parameter interval where this ray overlaps a centered,
    /// affinely expanded finite cylinder.
    /// </summary>
    /// <param name="center">The shape center in world coordinates.</param>
    /// <param name="axisDirection">The normalized axis direction in world coordinates.</param>
    /// <param name="axisLength">The full positive distance between the cap centers.</param>
    /// <param name="radius">The nonnegative radius in coordinate units.</param>
    /// <param name="radiusExpansion">A nonnegative addition to the radius, in coordinate units.</param>
    /// <param name="axialExpansion">A nonnegative distance to extend the cylinder at each axial end.</param>
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
    /// <paramref name="radius"/> or <paramref name="radiusExpansion"/> or <paramref name="axialExpansion"/> are
    /// negative. <paramref name="axisLength"/> must also be positive.
    /// </exception>
    public readonly bool TryGetFiniteCylinderIntersectionInterval(
        Vector3d center,
        Vector3d axisDirection,
        Fixed64 axisLength,
        Fixed64 radius,
        Fixed64 radiusExpansion,
        Fixed64 axialExpansion,
        Fixed64 maxParameter,
        out Fixed64 entryParameter,
        out Fixed64 exitParameter) =>
        TryGetFiniteCylinderIntersectionInterval(
            center, axisDirection, axisLength,
            radius, radiusExpansion, axialExpansion, maxParameter,
            out entryParameter, out exitParameter, out _, out _);

    /// <summary>
    /// Gets the closed parameter interval where this ray overlaps a centered,
    /// affinely expanded finite cylinder and reports exact endpoint containment.
    /// </summary>
    /// <param name="center">The shape center in world coordinates.</param>
    /// <param name="axisDirection">The normalized axis direction in world coordinates.</param>
    /// <param name="axisLength">The full positive distance between the cap centers.</param>
    /// <param name="radius">The nonnegative radius in coordinate units.</param>
    /// <param name="radiusExpansion">A nonnegative addition to the radius, in coordinate units.</param>
    /// <param name="axialExpansion">A nonnegative distance to extend the cylinder at each axial end.</param>
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
    /// <paramref name="radius"/> or <paramref name="radiusExpansion"/> or <paramref name="axialExpansion"/> are
    /// negative. <paramref name="axisLength"/> must also be positive.
    /// </exception>
    public readonly bool TryGetFiniteCylinderIntersectionInterval(
        Vector3d center,
        Vector3d axisDirection,
        Fixed64 axisLength,
        Fixed64 radius,
        Fixed64 radiusExpansion,
        Fixed64 axialExpansion,
        Fixed64 maxParameter,
        out Fixed64 entryParameter,
        out Fixed64 exitParameter,
        out bool originContained,
        out bool maximumContainedStrict)
    {
        if (!axisDirection.IsNormalized())
            throw new ArgumentException("Finite cylinder axis direction must be normalized.", nameof(axisDirection));
        if (axisLength <= Fixed64.Zero)
            throw new ArgumentOutOfRangeException(nameof(axisLength));
        ValidateCylinderExpansions(radius, radiusExpansion, axialExpansion);
        if (!ValidateMaximum(
                maxParameter,
                out entryParameter,
                out exitParameter,
                out originContained,
                out maximumContainedStrict))
        {
            return false;
        }

        return WideFiniteAxisIntersection.TryGetFiniteCylinderInterval(
            this, maxParameter, center, axisDirection, axisLength,
            radius, radiusExpansion, axialExpansion,
            out entryParameter, out exitParameter,
            out originContained, out maximumContainedStrict);
    }

    private static void ValidateCenteredCapsule(
        Vector3d axisDirection,
        Fixed64 axisLength,
        Fixed64 radius,
        Fixed64 radiusExpansion)
    {
        if (!axisDirection.IsNormalized())
            throw new ArgumentException("Capsule axis direction must be normalized.", nameof(axisDirection));
        if (axisLength < Fixed64.Zero)
            throw new ArgumentOutOfRangeException(nameof(axisLength));
        if (radius < Fixed64.Zero)
            throw new ArgumentOutOfRangeException(nameof(radius));
        if (radiusExpansion < Fixed64.Zero)
            throw new ArgumentOutOfRangeException(nameof(radiusExpansion));
    }

    private static void ValidateCylinderExpansions(
        Fixed64 radius,
        Fixed64 radiusExpansion,
        Fixed64 axialExpansion)
    {
        if (radius < Fixed64.Zero)
            throw new ArgumentOutOfRangeException(nameof(radius));
        if (radiusExpansion < Fixed64.Zero)
            throw new ArgumentOutOfRangeException(nameof(radiusExpansion));
        if (axialExpansion < Fixed64.Zero)
            throw new ArgumentOutOfRangeException(nameof(axialExpansion));
    }

    private static bool ValidateMaximum(
        Fixed64 maxParameter,
        out Fixed64 entryParameter,
        out Fixed64 exitParameter,
        out bool originContained,
        out bool maximumContainedStrict)
    {
        entryParameter = default;
        exitParameter = default;
        originContained = false;
        maximumContainedStrict = false;
        return maxParameter >= Fixed64.Zero;
    }
}
