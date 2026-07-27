//=======================================================================
// FixedBoundCircle.cs
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
/// Represents a normalized two-dimensional circular bound.
/// </summary>
/// <remarks>
/// Radius inputs are normalized by absolute value so a public circle cannot
/// retain a negative radius through construction, assignment, or state load.
/// </remarks>
[Serializable]
[MemoryPackable]
public partial struct FixedBoundCircle : IEquatable<FixedBoundCircle>
{
    #region Nested Types

    /// <summary>
    /// Represents the normalized serializable state of a two-dimensional circular bound.
    /// </summary>
    [Serializable]
    [MemoryPackable]
    public readonly partial struct BoundingCircleState
    {
        /// <inheritdoc cref="FixedBoundCircle.Center"/>
        [JsonInclude]
        [MemoryPackInclude]
        public readonly Vector2d Center;

        /// <inheritdoc cref="FixedBoundCircle.Radius"/>
        [JsonInclude]
        [MemoryPackInclude]
        public readonly Fixed64 Radius;

        /// <summary>
        /// Initializes a normalized state from center and radius.
        /// </summary>
        [JsonConstructor]
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public BoundingCircleState(Vector2d center, Fixed64 radius)
        {
            Center = center;
            Radius = NormalizeRadius(radius);
        }
    }

    #endregion

    #region Fields

    private Fixed64 _radius;

    #endregion

    #region Constructors

    /// <summary>
    /// Initializes a new instance from a center point and radius.
    /// </summary>
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public FixedBoundCircle(Vector2d center, Fixed64 radius)
    {
        Center = center;
        _radius = NormalizeRadius(radius);
    }

    /// <summary>
    /// Initializes a new instance from serialized or caller-provided state.
    /// </summary>
    [JsonConstructor]
    public FixedBoundCircle(BoundingCircleState state)
    {
        Center = state.Center;
        _radius = state.Radius;
    }

    #endregion

    #region Properties

    /// <summary>
    /// The center point of the circle.
    /// </summary>
    [JsonIgnore]
    [MemoryPackIgnore]
    public Vector2d Center { get; set; }

    /// <summary>
    /// The non-negative radius of the circle. Assigned values are normalized by absolute value.
    /// </summary>
    [JsonIgnore]
    [MemoryPackIgnore]
    public Fixed64 Radius
    {
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        get => _radius;

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        set => _radius = NormalizeRadius(value);
    }

    /// <summary>
    /// The normalized axis-aligned representable-domain intersection that
    /// contains every representable point of the circle.
    /// </summary>
    [JsonIgnore]
    [MemoryPackIgnore]
    public FixedBoundArea Bounds
    {
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        get => FixedBoundArea.FromCenterAndScopeClippedToDomain(Center, new Vector2d(Radius, Radius));
    }

    /// <summary>
    /// Gets the current normalized state of the circle.
    /// </summary>
    [JsonInclude]
    [MemoryPackInclude]
    public BoundingCircleState State => new(Center, Radius);

    #endregion

    #region Spatial Queries

    /// <summary>
    /// Determines whether the point is inside this circle, including the boundary.
    /// </summary>
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public bool Contains(Vector2d point)
    {
        return WideGeometry.CompareDistanceToRadiusSum(Center, point, Radius, Fixed64.Zero) <= 0;
    }

    /// <summary>
    /// Returns whether the point lies strictly inside this circle.
    /// </summary>
    /// <remarks>
    /// Boundary points and every point tested against a zero-radius circle
    /// return <see langword="false"/>.
    /// </remarks>
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public bool ContainsStrict(Vector2d point) =>
        Radius > Fixed64.Zero
        && WideGeometry.CompareDistanceToRadiusSum(Center, point, Radius, Fixed64.Zero) < 0;

    /// <summary>
    /// Classifies another circle against this circle using boundary-inclusive overlap.
    /// </summary>
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public FixedEnclosureType Contains(FixedBoundCircle circle)
    {
        if (WideGeometry.CompareDistanceToRadiusSum(Center, circle.Center, Radius, circle.Radius) > 0)
            return FixedEnclosureType.Disjoint;

        Fixed64 radiusDifference = Radius - circle.Radius;
        if (radiusDifference >= Fixed64.Zero
            && WideGeometry.CompareDistanceToRadiusSum(
                Center,
                circle.Center,
                radiusDifference,
                Fixed64.Zero) <= 0)
            return FixedEnclosureType.Contains;

        return FixedEnclosureType.Intersects;
    }

    /// <summary>
    /// Determines whether another circle intersects this circle, including boundary-only contact.
    /// </summary>
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public bool Intersects(FixedBoundCircle circle) => Contains(circle) != FixedEnclosureType.Disjoint;

    /// <summary>
    /// Determines whether an area intersects this circle, including boundary-only contact.
    /// </summary>
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public bool Intersects(FixedBoundArea area)
    {
        return WideGeometry.CompareDistanceToRadiusSum(
            Center,
            area.ClampPoint(Center),
            Radius,
            Fixed64.Zero) <= 0;
    }

    /// <summary>
    /// Determines whether another circle overlaps this circle with positive area.
    /// </summary>
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public bool IntersectsStrict(FixedBoundCircle circle)
    {
        return Radius > Fixed64.Zero
            && circle.Radius > Fixed64.Zero
            && WideGeometry.CompareDistanceToRadiusSum(Center, circle.Center, Radius, circle.Radius) < 0;
    }

    /// <summary>
    /// Determines whether an area overlaps this circle with positive area.
    /// </summary>
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public bool IntersectsStrict(FixedBoundArea area)
    {
        return Radius > Fixed64.Zero
            && area.Min.X < area.Max.X
            && area.Min.Y < area.Max.Y
            && WideGeometry.CompareDistanceToRadiusSum(
                Center,
                area.ClampPoint(Center),
                Radius,
                Fixed64.Zero) < 0;
    }

    /// <summary>
    /// Clamps a point to this circle, returning the point unchanged when it is already inside.
    /// </summary>
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public Vector2d ClampPoint(Vector2d point)
    {
        return Contains(point) ? point : ProjectPoint(point);
    }

    /// <summary>
    /// Projects a point onto the circle boundary. The center is returned when the direction is degenerate.
    /// </summary>
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public Vector2d ProjectPoint(Vector2d point)
    {
        Vector2d direction = point - Center;
        if (direction.EqualsZero())
            return Center;

        return Center + direction.NormalizeInPlace() * Radius;
    }

    #endregion

    #region Deconstruction

    /// <summary>
    /// Deconstructs the circle into center and normalized radius.
    /// </summary>
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public void Deconstruct(out Vector2d center, out Fixed64 radius)
    {
        center = Center;
        radius = Radius;
    }

    #endregion

    #region Equality

    /// <inheritdoc />
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public bool Equals(FixedBoundCircle other)
    {
        return Center == other.Center && Radius == other.Radius;
    }

    /// <inheritdoc />
    public override bool Equals(object? obj)
    {
        return obj is FixedBoundCircle other && Equals(other);
    }

    /// <inheritdoc />
    public override int GetHashCode()
    {
        unchecked
        {
            int hash = 17;
            hash = (hash * 31) + Center.StateHash;
            hash = (hash * 31) + Radius.GetHashCode();
            return hash;
        }
    }

    /// <summary>
    /// Determines whether two circles have the same normalized state.
    /// </summary>
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public static bool operator ==(FixedBoundCircle left, FixedBoundCircle right) => left.Equals(right);

    /// <summary>
    /// Determines whether two circles have different normalized state.
    /// </summary>
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public static bool operator !=(FixedBoundCircle left, FixedBoundCircle right) => !left.Equals(right);

    #endregion

    #region Helpers

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    private static Fixed64 NormalizeRadius(Fixed64 radius) => FixedMath.Abs(radius);

    #endregion
}
