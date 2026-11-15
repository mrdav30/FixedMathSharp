//=======================================================================
// FixedBoundCircle.cs
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

    #region Fields

    private Fixed64 _radius;

    #endregion

    #region Constructors

    /// <summary>
    /// Initializes a new instance from a center point and radius.
    /// </summary>
    /// <param name="center">The center in the bound's coordinate space.</param>
    /// <param name="radius">The radius in coordinate units. Negative values are normalized by absolute value.</param>
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public FixedBoundCircle(Vector2d center, Fixed64 radius)
    {
        Center = center;
        _radius = NormalizeRadius(radius);
    }

    /// <summary>
    /// Initializes a new instance from serialized or caller-provided state.
    /// </summary>
    /// <param name="state">The bound state to normalize and copy.</param>
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
    /// <value>The current center in the bound's coordinate space.</value>
    [JsonIgnore]
    [MemoryPackIgnore]
    public Vector2d Center { get; set; }

    /// <summary>
    /// The non-negative radius of the circle. Assigned values are normalized by absolute value.
    /// </summary>
    /// <value>The nonnegative radius in coordinate units.</value>
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
    /// <value>The axis-aligned bounds clipped to the representable coordinate domain.</value>
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
    /// <value>A copy of the current center and normalized radius.</value>
    [JsonInclude]
    [MemoryPackInclude]
    public BoundingCircleState State => new(Center, Radius);

    #endregion

    #region Spatial Queries

    /// <summary>
    /// Determines whether the point is inside this circle, including the boundary.
    /// </summary>
    /// <param name="point">The point in the same coordinate space as this circle.</param>
    /// <returns><see langword="true"/> when the point lies on the boundary or inside and <see langword="false"/> otherwise.</returns>
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public bool Contains(Vector2d point)
    {
        return WideGeometry.CompareDistanceToRadiusSum(Center, point, Radius, Fixed64.Zero) <= 0;
    }

    /// <summary>
    /// Returns whether the point lies strictly inside this circle.
    /// </summary>
    /// <param name="point">The point in the same coordinate space as this circle.</param>
    /// <returns><see langword="true"/> when the point lies in the interior and <see langword="false"/> otherwise.</returns>
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
    /// <param name="circle">The circle in the same coordinate space as this circle.</param>
    /// <returns>
    /// <see cref="FixedEnclosureType.Contains"/> if the supplied circle is wholly inside, including the boundary,
    /// <see cref="FixedEnclosureType.Intersects"/> for other overlap or contact, or
    /// <see cref="FixedEnclosureType.Disjoint"/> for separation.
    /// </returns>
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
    /// <param name="circle">The circle in the same coordinate space as this circle.</param>
    /// <returns><see langword="true"/> when the shapes overlap or touch and <see langword="false"/> otherwise.</returns>
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public bool Intersects(FixedBoundCircle circle) => Contains(circle) != FixedEnclosureType.Disjoint;

    /// <summary>
    /// Determines whether an area intersects this circle, including boundary-only contact.
    /// </summary>
    /// <param name="area">The area in the same coordinate space as this circle.</param>
    /// <returns><see langword="true"/> when the shapes overlap or touch and <see langword="false"/> otherwise.</returns>
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
    /// <param name="circle">The circle in the same coordinate space as this circle.</param>
    /// <returns><see langword="true"/> when the shared region has positive area and <see langword="false"/> otherwise.</returns>
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
    /// <param name="area">The area in the same coordinate space as this circle.</param>
    /// <returns><see langword="true"/> when the shared region has positive area and <see langword="false"/> otherwise.</returns>
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
    /// <param name="point">The point in the same coordinate space as this circle.</param>
    /// <returns>The supplied point if it is contained, or its closest point on the boundary.</returns>
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public Vector2d ClampPoint(Vector2d point)
    {
        return Contains(point) ? point : ProjectPoint(point);
    }

    /// <summary>
    /// Projects a point onto the circle boundary. The center is returned when the direction is degenerate.
    /// </summary>
    /// <param name="point">The point in the same coordinate space as this circle.</param>
    /// <returns>The point on the radial surface toward the supplied point, or the center when the direction is zero.</returns>
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
    /// <param name="center">The current center.</param>
    /// <param name="radius">The normalized radius.</param>
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public void Deconstruct(out Vector2d center, out Fixed64 radius)
    {
        center = Center;
        radius = Radius;
    }

    #endregion

    #region Equality

    /// <summary>Compares the stored geometry for equality.</summary>
    /// <param name="other">The value to compare with this instance.</param>
    /// <returns><see langword="true"/> when the centers and radii match and <see langword="false"/> otherwise.</returns>
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public bool Equals(FixedBoundCircle other)
    {
        return Center == other.Center && Radius == other.Radius;
    }

    /// <summary>Compares the stored geometry for equality.</summary>
    /// <param name="obj">The value to compare with this instance.</param>
    /// <returns>
    /// <see langword="true"/> when <paramref name="obj"/> is a <see cref="FixedBoundCircle"/> and the centers and
    /// radii match and <see langword="false"/> otherwise.
    /// </returns>
    public override bool Equals(object? obj)
    {
        return obj is FixedBoundCircle other && Equals(other);
    }

    /// <summary>Computes a hash code for the stored geometry.</summary>
    /// <returns>A hash code consistent with structural equality.</returns>
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

    /// <summary>Tests whether the stored geometry is equal.</summary>
    /// <param name="left">The first value to compare.</param>
    /// <param name="right">The second value to compare.</param>
    /// <returns><see langword="true"/> when the centers and radii match and <see langword="false"/> otherwise.</returns>
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public static bool operator ==(FixedBoundCircle left, FixedBoundCircle right) => left.Equals(right);

    /// <summary>Tests whether the stored geometry is different.</summary>
    /// <param name="left">The first value to compare.</param>
    /// <param name="right">The second value to compare.</param>
    /// <returns><see langword="true"/> when the stored values differ and <see langword="false"/> otherwise.</returns>
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public static bool operator !=(FixedBoundCircle left, FixedBoundCircle right) => !left.Equals(right);

    #endregion

    #region Helpers

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    private static Fixed64 NormalizeRadius(Fixed64 radius) => FixedMath.Abs(radius);

    #endregion
}
