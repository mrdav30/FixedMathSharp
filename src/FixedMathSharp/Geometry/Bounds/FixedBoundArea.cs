//=======================================================================
// FixedBoundArea.cs
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
/// Represents a normalized two-dimensional axis-aligned bounding area.
/// </summary>
/// <remarks>
/// FixedMathSharp 2D geometry is plain <see cref="Vector2d"/> plane math. Use
/// <see cref="FromMinMax"/>, <see cref="FromCenterAndSize"/>, or
/// <see cref="FromCenterAndScope"/> so construction intent is explicit.
/// </remarks>
[Serializable]
[MemoryPackable]
public partial struct FixedBoundArea : IEquatable<FixedBoundArea>
{
    #region Nested Types

    /// <summary>
    /// Represents the normalized serializable state of a two-dimensional axis-aligned bounding area.
    /// </summary>
    [Serializable]
    [MemoryPackable]
    public readonly partial struct BoundingAreaState
    {
        /// <inheritdoc cref="FixedBoundArea.Min"/>
        [JsonInclude]
        [MemoryPackInclude]
        public readonly Vector2d Min;

        /// <inheritdoc cref="FixedBoundArea.Max"/>
        [JsonInclude]
        [MemoryPackInclude]
        public readonly Vector2d Max;

        /// <summary>
        /// Initializes a normalized state from minimum and maximum corners.
        /// </summary>
        [JsonConstructor]
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public BoundingAreaState(Vector2d min, Vector2d max)
        {
            Min = ComponentMin(min, max);
            Max = ComponentMax(min, max);
        }
    }

    #endregion

    #region Constructors

    /// <summary>
    /// Initializes a new instance from serialized or caller-provided state.
    /// </summary>
    [JsonConstructor]
    public FixedBoundArea(BoundingAreaState state)
    {
        State = state;
    }

    #endregion

    #region Properties

    /// <summary>
    /// The minimum corner of the area.
    /// </summary>
    [JsonIgnore]
    [MemoryPackIgnore]
    public Vector2d Min { get; private set; }

    /// <summary>
    /// The maximum corner of the area.
    /// </summary>
    [JsonIgnore]
    [MemoryPackIgnore]
    public Vector2d Max { get; private set; }

    /// <summary>
    /// The center of the area.
    /// </summary>
    [JsonIgnore]
    [MemoryPackIgnore]
    public Vector2d Center
    {
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        get => (Min + Max) * Fixed64.Half;

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        set
        {
            Vector2d half = (Max - Min) * Fixed64.Half;
            Min = value - half;
            Max = value + half;
        }
    }

    /// <summary>
    /// The total width and height of the area. Assigned values are normalized by absolute component value.
    /// </summary>
    [JsonIgnore]
    [MemoryPackIgnore]
    public Vector2d Size
    {
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        get => Max - Min;

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        set
        {
            Vector2d half = Vector2d.Abs(value) * Fixed64.Half;
            Vector2d center = (Min + Max) * Fixed64.Half;
            Min = center - half;
            Max = center + half;
        }
    }

    /// <summary>
    /// The half-size of the area in both axes.
    /// </summary>
    [JsonIgnore]
    [MemoryPackIgnore]
    public Vector2d Scope
    {
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        get => (Max - Min) * Fixed64.Half;
    }

    /// <summary>
    /// Gets or sets the current normalized state of the area.
    /// </summary>
    [JsonInclude]
    [MemoryPackInclude]
    public BoundingAreaState State
    {
        get => new(Min, Max);

        internal set
        {
            SetMinMax(value.Min, value.Max);
        }
    }

    #endregion

    #region Factories

    /// <summary>
    /// Creates a normalized area from minimum and maximum corners.
    /// </summary>
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public static FixedBoundArea FromMinMax(Vector2d min, Vector2d max)
    {
        var area = default(FixedBoundArea);
        area.SetMinMax(min, max);
        return area;
    }

    /// <summary>
    /// Creates an area from a center point and total size.
    /// </summary>
    /// <remarks>
    /// Negative size components are normalized by absolute value.
    /// </remarks>
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public static FixedBoundArea FromCenterAndSize(Vector2d center, Vector2d size)
    {
        return FromCenterAndScope(center, size * Fixed64.Half);
    }

    /// <summary>
    /// Creates an area from a center point and half-size scope.
    /// </summary>
    /// <remarks>
    /// Negative scope components are normalized by absolute value.
    /// </remarks>
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public static FixedBoundArea FromCenterAndScope(Vector2d center, Vector2d scope)
    {
        Vector2d normalizedScope = Vector2d.Abs(scope);
        return new FixedBoundArea
        {
            Min = center - normalizedScope,
            Max = center + normalizedScope
        };
    }

    #endregion

    #region Mutators

    /// <summary>
    /// Sets the normalized bounds of the area by specifying minimum and maximum points.
    /// </summary>
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public void SetMinMax(Vector2d min, Vector2d max)
    {
        Min = ComponentMin(min, max);
        Max = ComponentMax(min, max);
    }

    #endregion

    #region Spatial Queries

    /// <summary>
    /// Determines whether the point is inside this area, including the boundary.
    /// </summary>
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public bool Contains(Vector2d point)
    {
        return point.X >= Min.X && point.X <= Max.X
            && point.Y >= Min.Y && point.Y <= Max.Y;
    }

    /// <summary>
    /// Classifies another area against this area using boundary-inclusive overlap.
    /// </summary>
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public FixedEnclosureType Contains(FixedBoundArea area)
    {
        if (area.Min.X >= Min.X && area.Max.X <= Max.X
            && area.Min.Y >= Min.Y && area.Max.Y <= Max.Y)
            return FixedEnclosureType.Contains;

        return Intersects(area)
            ? FixedEnclosureType.Intersects
            : FixedEnclosureType.Disjoint;
    }

    /// <summary>
    /// Determines whether this area overlaps another area, including boundary-only contact.
    /// </summary>
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public bool Intersects(FixedBoundArea area)
    {
        return Min.X <= area.Max.X && Max.X >= area.Min.X
            && Min.Y <= area.Max.Y && Max.Y >= area.Min.Y;
    }

    /// <summary>
    /// Determines whether this area overlaps another area with positive area on both axes.
    /// </summary>
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public bool IntersectsStrict(FixedBoundArea area)
    {
        return HasPositiveArea() && area.HasPositiveArea()
            && Min.X < area.Max.X && Max.X > area.Min.X
            && Min.Y < area.Max.Y && Max.Y > area.Min.Y;
    }

    /// <summary>
    /// Clamps a point to the area boundary or interior.
    /// </summary>
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public Vector2d ClampPoint(Vector2d point)
    {
        return new Vector2d(
            FixedMath.Clamp(point.X, Min.X, Max.X),
            FixedMath.Clamp(point.Y, Min.Y, Max.Y));
    }

    /// <summary>
    /// Projects a point onto this area by clamping it to the boundary or interior.
    /// </summary>
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public Vector2d ProjectPoint(Vector2d point) => ClampPoint(point);

    #endregion

    #region Deconstruction

    /// <summary>
    /// Deconstructs the area into normalized minimum and maximum corners.
    /// </summary>
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public void Deconstruct(out Vector2d min, out Vector2d max)
    {
        min = Min;
        max = Max;
    }

    #endregion

    #region Equality

    /// <inheritdoc />
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public bool Equals(FixedBoundArea other)
    {
        return Min == other.Min && Max == other.Max;
    }

    /// <inheritdoc />
    public override bool Equals(object? obj)
    {
        return obj is FixedBoundArea other && Equals(other);
    }

    /// <inheritdoc />
    public override int GetHashCode()
    {
        unchecked
        {
            int hash = 17;
            hash = (hash * 31) + Min.StateHash;
            hash = (hash * 31) + Max.StateHash;
            return hash;
        }
    }

    /// <summary>
    /// Determines whether two areas have the same normalized bounds.
    /// </summary>
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public static bool operator ==(FixedBoundArea left, FixedBoundArea right) => left.Equals(right);

    /// <summary>
    /// Determines whether two areas have different normalized bounds.
    /// </summary>
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public static bool operator !=(FixedBoundArea left, FixedBoundArea right) => !left.Equals(right);

    #endregion

    #region Static Operations

    /// <summary>
    /// Creates a new area that contains both input areas.
    /// </summary>
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public static FixedBoundArea Union(FixedBoundArea a, FixedBoundArea b)
    {
        return FromMinMax(ComponentMin(a.Min, b.Min), ComponentMax(a.Max, b.Max));
    }

    #endregion

    #region Helpers

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    private bool HasPositiveArea()
    {
        return Min.X < Max.X && Min.Y < Max.Y;
    }

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    private static Vector2d ComponentMin(Vector2d a, Vector2d b)
    {
        return new Vector2d(FixedMath.Min(a.X, b.X), FixedMath.Min(a.Y, b.Y));
    }

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    private static Vector2d ComponentMax(Vector2d a, Vector2d b)
    {
        return new Vector2d(FixedMath.Max(a.X, b.X), FixedMath.Max(a.Y, b.Y));
    }

    #endregion
}
