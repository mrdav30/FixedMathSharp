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

namespace FixedMathSharp.Bounds;

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
    /// The squared radius of the circle.
    /// </summary>
    [JsonIgnore]
    [MemoryPackIgnore]
    public Fixed64 RadiusSquared
    {
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        get => Radius * Radius;
    }

    /// <summary>
    /// The normalized axis-aligned area that contains the circle.
    /// </summary>
    [JsonIgnore]
    [MemoryPackIgnore]
    public FixedBoundArea Bounds
    {
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        get => FixedBoundArea.FromCenterAndScope(Center, new Vector2d(Radius, Radius));
    }

    /// <summary>
    /// Gets or sets the current normalized state of the circle.
    /// </summary>
    [JsonInclude]
    [MemoryPackInclude]
    public BoundingCircleState State
    {
        get => new(Center, Radius);

        internal set
        {
            Center = value.Center;
            Radius = value.Radius;
        }
    }

    #endregion

    #region Spatial Queries

    /// <summary>
    /// Determines whether the point is inside this circle, including the boundary.
    /// </summary>
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public bool Contains(Vector2d point)
    {
        return Vector2d.DistanceSquared(Center, point) <= RadiusSquared;
    }

    /// <summary>
    /// Classifies another circle against this circle using boundary-inclusive overlap.
    /// </summary>
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public FixedEnclosureType Contains(FixedBoundCircle circle)
    {
        Fixed64 sqDistance = Vector2d.DistanceSquared(Center, circle.Center);
        Fixed64 combinedRadius = Radius + circle.Radius;

        if (sqDistance > combinedRadius * combinedRadius)
            return FixedEnclosureType.Disjoint;

        Fixed64 radiusDifference = Radius - circle.Radius;
        if (radiusDifference >= Fixed64.Zero && sqDistance <= radiusDifference * radiusDifference)
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
        return Vector2d.DistanceSquared(Center, area.ClampPoint(Center)) <= RadiusSquared;
    }

    /// <summary>
    /// Determines whether another circle overlaps this circle with positive area.
    /// </summary>
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public bool IntersectsStrict(FixedBoundCircle circle)
    {
        Fixed64 combinedRadius = Radius + circle.Radius;
        return Radius > Fixed64.Zero
            && circle.Radius > Fixed64.Zero
            && Vector2d.DistanceSquared(Center, circle.Center) < combinedRadius * combinedRadius;
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
            && Vector2d.DistanceSquared(Center, area.ClampPoint(Center)) < RadiusSquared;
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
