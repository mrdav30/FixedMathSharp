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

namespace FixedMathSharp.Geometry;

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
    /// The center of the area, rounded to the nearest-even Q32.32 lattice point.
    /// </summary>
    /// <remarks>
    /// Assigning a different center preserves a conservative half-extent. An
    /// odd raw-unit span can therefore expand by one raw unit so the assigned
    /// center remains exact and the previous area is not under-represented.
    /// </remarks>
    /// <exception cref="OverflowException">
    /// An assigned center would place an endpoint outside the scalar domain.
    /// </exception>
    [JsonIgnore]
    [MemoryPackIgnore]
    public Vector2d Center
    {
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        get => new(
            FixedMath.Midpoint(Min.X, Max.X),
            FixedMath.Midpoint(Min.Y, Max.Y));

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        set
        {
            if (value != Center)
                SetCenterAndHalfSize(value, Scope);
        }
    }

    /// <summary>
    /// The exact total width and height of the area.
    /// </summary>
    /// <remarks>
    /// Assigned values are normalized by absolute component value and divided
    /// outward. An odd raw-unit size therefore expands by one raw unit. Reading
    /// this property throws rather than returning a saturated value when an
    /// exact component span is not representable by <see cref="Fixed64"/>.
    /// </remarks>
    /// <exception cref="OverflowException">
    /// A component span is not representable, or an assigned size would place
    /// an endpoint outside the scalar domain.
    /// </exception>
    [JsonIgnore]
    [MemoryPackIgnore]
    public Vector2d Size
    {
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        get => new(
            WideGeometry.GetIntervalSize(Min.X, Max.X),
            WideGeometry.GetIntervalSize(Min.Y, Max.Y));

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        set
        {
            SetCenterAndHalfSize(Center, GetHalfSize(value));
        }
    }

    /// <summary>
    /// The smallest representable half-extent that conservatively contains the
    /// area around <see cref="Center"/>.
    /// </summary>
    /// <exception cref="OverflowException">
    /// A conservative half-extent is outside the representable scalar domain.
    /// </exception>
    [JsonIgnore]
    [MemoryPackIgnore]
    public Vector2d Scope
    {
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        get => new(
            WideGeometry.GetIntervalScope(Min.X, Max.X),
            WideGeometry.GetIntervalScope(Min.Y, Max.Y));
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
    /// Negative size components are normalized by absolute value. Odd raw-unit
    /// sizes are divided outward and therefore expand by one raw unit.
    /// </remarks>
    /// <exception cref="OverflowException">
    /// The centered area would place an endpoint outside the scalar domain.
    /// </exception>
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public static FixedBoundArea FromCenterAndSize(Vector2d center, Vector2d size)
    {
        var area = default(FixedBoundArea);
        area.SetCenterAndHalfSize(center, GetHalfSize(size));
        return area;
    }

    /// <summary>
    /// Creates an area from a center point and half-size scope.
    /// </summary>
    /// <remarks>
    /// Negative scope components are normalized by absolute value.
    /// </remarks>
    /// <exception cref="OverflowException">
    /// A scope magnitude is not representable, or the centered area would
    /// place an endpoint outside the scalar domain.
    /// </exception>
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public static FixedBoundArea FromCenterAndScope(Vector2d center, Vector2d scope)
    {
        var area = default(FixedBoundArea);
        area.SetCenterAndHalfSize(center, GetScopeMagnitude(scope));
        return area;
    }

    /// <summary>
    /// Creates the representable-domain intersection of an area described by a
    /// center point and total size.
    /// </summary>
    /// <remarks>
    /// Negative size components are normalized by absolute value and odd raw-
    /// unit sizes divide outward. Endpoints outside the scalar domain are
    /// explicitly clipped to <see cref="Fixed64.MinValue"/> or
    /// <see cref="Fixed64.MaxValue"/>.
    /// </remarks>
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public static FixedBoundArea FromCenterAndSizeClippedToDomain(Vector2d center, Vector2d size)
    {
        var area = default(FixedBoundArea);
        area.SetCenterAndHalfSizeClippedToDomain(center, GetHalfSize(size));
        return area;
    }

    /// <summary>
    /// Creates the representable-domain intersection of an area described by a
    /// center point and half-size scope.
    /// </summary>
    /// <remarks>
    /// Negative scope components are normalized by absolute value. Endpoints
    /// outside the scalar domain are explicitly clipped to
    /// <see cref="Fixed64.MinValue"/> or <see cref="Fixed64.MaxValue"/>.
    /// </remarks>
    /// <exception cref="OverflowException">
    /// A scope magnitude is not representable.
    /// </exception>
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public static FixedBoundArea FromCenterAndScopeClippedToDomain(Vector2d center, Vector2d scope)
    {
        var area = default(FixedBoundArea);
        area.SetCenterAndHalfSizeClippedToDomain(center, GetScopeMagnitude(scope));
        return area;
    }

    /// <summary>
    /// Creates the representable-domain intersection of bounds described by a
    /// center and normalized center-relative minimum and maximum offsets.
    /// </summary>
    /// <remarks>
    /// Each endpoint is formed by one final saturating add, which is the
    /// explicit clipping operation. Asymmetric offsets are preserved.
    /// </remarks>
    /// <exception cref="ArgumentException">
    /// A minimum offset component exceeds the matching maximum component.
    /// </exception>
    public static FixedBoundArea FromCenterAndOffsetsClippedToDomain(
        Vector2d center,
        Vector2d minimumOffset,
        Vector2d maximumOffset)
    {
        if (minimumOffset.X > maximumOffset.X
            || minimumOffset.Y > maximumOffset.Y)
        {
            throw new ArgumentException(
                "Minimum offsets must not exceed maximum offsets.",
                nameof(minimumOffset));
        }

        return FromMinMax(
            center + minimumOffset,
            center + maximumOffset);
    }

    /// <summary>
    /// Creates the representable-domain intersection of bounds around rotated
    /// local offsets without materializing any transformed point.
    /// </summary>
    public static FixedBoundArea FromRotatedOffsetsClippedToDomain(
        Vector2d origin,
        Fixed64 rotation,
        ReadOnlySpan<Vector2d> localOffsets)
    {
        if (localOffsets.IsEmpty)
        {
            throw new ArgumentException(
                "At least one local offset is required.",
                nameof(localOffsets));
        }

        return WideConvex2dRelations.GetBoundsClippedToDomain(
            origin,
            rotation,
            localOffsets);
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
    /// Classifies a circle against this area using boundary-inclusive overlap.
    /// </summary>
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public FixedEnclosureType Contains(FixedBoundCircle circle)
    {
        if (WideGeometry.ContainsCenteredExtent(Min.X, Max.X, circle.Center.X, circle.Radius)
            && WideGeometry.ContainsCenteredExtent(Min.Y, Max.Y, circle.Center.Y, circle.Radius))
            return FixedEnclosureType.Contains;

        return Intersects(circle)
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
    /// Determines whether this area overlaps a circle, including boundary-only contact.
    /// </summary>
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public bool Intersects(FixedBoundCircle circle) => circle.Intersects(this);

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
    /// Determines whether this area overlaps a circle with positive area.
    /// </summary>
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public bool IntersectsStrict(FixedBoundCircle circle) => circle.IntersectsStrict(this);

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

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    private void SetCenterAndHalfSize(Vector2d center, Vector2d halfSize)
    {
        if (!Vector2d.TrySubtract(center, halfSize, out Vector2d min)
            || !Vector2d.TryAdd(center, halfSize, out Vector2d max))
        {
            throw CreateUnrepresentableBoundsException();
        }

        Min = min;
        Max = max;
    }

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    private void SetCenterAndHalfSizeClippedToDomain(Vector2d center, Vector2d halfSize)
    {
        Vector2d min = center - halfSize;
        Vector2d max = center + halfSize;
        Min = min;
        Max = max;
    }

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    private static Vector2d GetHalfSize(Vector2d size) => new(
        WideGeometry.GetHalfSizeMagnitude(size.X),
        WideGeometry.GetHalfSizeMagnitude(size.Y));

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    private static Vector2d GetScopeMagnitude(Vector2d scope) => new(
        WideGeometry.GetExtentMagnitude(scope.X),
        WideGeometry.GetExtentMagnitude(scope.Y));

    private static OverflowException CreateUnrepresentableBoundsException() =>
        new("The centered area places at least one endpoint outside the representable Fixed64 range.");

    #endregion
}
