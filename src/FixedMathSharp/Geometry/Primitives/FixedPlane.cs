//=======================================================================
// FixedPlane.cs
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
/// Represents a plane in 3D space using a normal vector and a distance from the origin.
/// </summary>
[Serializable]
[MemoryPackable]
public partial struct FixedPlane : IEquatable<FixedPlane>
{
    #region Fields

    /// <summary>
    /// The plane normal.
    /// </summary>
    [JsonInclude]
    [MemoryPackOrder(0)]
    public Vector3d Normal;

    /// <summary>
    /// The plane distance component.
    /// </summary>
    [JsonInclude]
    [MemoryPackOrder(1)]
    public Fixed64 D;

    #endregion

    #region Constructors

    /// <summary>
    /// Initializes a new plane from a normal and distance component.
    /// </summary>
    [JsonConstructor]
    public FixedPlane(Vector3d normal, Fixed64 d)
    {
        Normal = normal;
        D = d;
    }

    /// <summary>
    /// Initializes a new plane from the normal components and distance component.
    /// </summary>
    public FixedPlane(Fixed64 x, Fixed64 y, Fixed64 z, Fixed64 d)
        : this(new Vector3d(x, y, z), d)
    {
    }

    /// <summary>
    /// Initializes a new plane that contains the specified points.
    /// </summary>
    public FixedPlane(Vector3d a, Vector3d b, Vector3d c)
    {
        Vector3d ab = b - a;
        Vector3d ac = c - a;

        Normal = Vector3d.Cross(ab, ac).Normalize();
        D = -Vector3d.Dot(Normal, a);
    }

    /// <summary>
    /// Initializes a new plane that contains the specified point with the specified normal.
    /// </summary>
    public FixedPlane(Vector3d pointOnPlane, Vector3d normal)
    {
        Normal = normal;
        D = -Vector3d.Dot(pointOnPlane, normal);
    }

    #endregion

    #region Methods

    /// <summary>
    /// Gets the dot product of the specified coordinate and this plane.
    /// </summary>
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public Fixed64 DotCoordinate(Vector3d value)
    {
        return Vector3d.Dot(Normal, value) + D;
    }

    /// <summary>
    /// Gets the dot product of the specified vector and this plane's normal.
    /// </summary>
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public Fixed64 DotNormal(Vector3d value)
    {
        return Vector3d.Dot(Normal, value);
    }

    /// <summary>
    /// Normalizes this plane in place.
    /// </summary>
    public void Normalize()
    {
        this = Normalize(this);
    }

    /// <summary>
    /// Gets a normalized copy of the specified plane.
    /// </summary>
    public static FixedPlane Normalize(FixedPlane value)
    {
        Fixed64 length = value.Normal.Magnitude;
        if (length == Fixed64.Zero)
            throw new InvalidOperationException("Cannot normalize a plane with a zero-length normal.");

        Fixed64 factor = Fixed64.One / length;
        return new FixedPlane(value.Normal * factor, value.D * factor);
    }

    /// <summary>
    /// Classifies a bounding box relative to this plane.
    /// </summary>
    public FixedPlaneIntersectionType Intersects(BoundingBox box)
    {
        return IntersectsBoxLike(box.Min, box.Max);
    }

    /// <summary>
    /// Classifies a bounding area relative to this plane.
    /// </summary>
    public FixedPlaneIntersectionType Intersects(BoundingArea area)
    {
        return IntersectsBoxLike(area.Min, area.Max);
    }

    /// <summary>
    /// Classifies a bounding sphere relative to this plane.
    /// </summary>
    public FixedPlaneIntersectionType Intersects(BoundingSphere sphere)
    {
        Fixed64 distance = DotCoordinate(sphere.Center);

        if (distance > sphere.Radius)
            return FixedPlaneIntersectionType.Front;

        if (distance < -sphere.Radius)
            return FixedPlaneIntersectionType.Back;

        return FixedPlaneIntersectionType.Intersecting;
    }

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    internal FixedPlaneIntersectionType Intersects(Vector3d point)
    {
        Fixed64 distance = DotCoordinate(point);

        if (distance > Fixed64.Zero)
            return FixedPlaneIntersectionType.Front;

        if (distance < Fixed64.Zero)
            return FixedPlaneIntersectionType.Back;

        return FixedPlaneIntersectionType.Intersecting;
    }

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    private FixedPlaneIntersectionType IntersectsBoxLike(Vector3d min, Vector3d max)
    {
        Vector3d positive = new(
            Normal.x >= Fixed64.Zero ? max.x : min.x,
            Normal.y >= Fixed64.Zero ? max.y : min.y,
            Normal.z >= Fixed64.Zero ? max.z : min.z);

        if (DotCoordinate(positive) < Fixed64.Zero)
            return FixedPlaneIntersectionType.Back;

        Vector3d negative = new(
            Normal.x >= Fixed64.Zero ? min.x : max.x,
            Normal.y >= Fixed64.Zero ? min.y : max.y,
            Normal.z >= Fixed64.Zero ? min.z : max.z);

        if (DotCoordinate(negative) > Fixed64.Zero)
            return FixedPlaneIntersectionType.Front;

        return FixedPlaneIntersectionType.Intersecting;
    }

    /// <summary>
    /// Deconstructs the plane into its normal and distance components.
    /// </summary>
    public void Deconstruct(out Vector3d normal, out Fixed64 d)
    {
        normal = Normal;
        d = D;
    }

    #endregion

    #region Operators

    /// <summary>
    /// Determines whether two planes are equal.
    /// </summary>
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public static bool operator ==(FixedPlane left, FixedPlane right) => left.Equals(right);

    /// <summary>
    /// Determines whether two planes are not equal.
    /// </summary>
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public static bool operator !=(FixedPlane left, FixedPlane right) => !left.Equals(right);

    #endregion

    #region Equality

    /// <inheritdoc/>
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public override bool Equals(object? obj) => obj is FixedPlane other && Equals(other);

    /// <inheritdoc/>
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public bool Equals(FixedPlane other) => Normal.Equals(other.Normal) && D.Equals(other.D);

    /// <inheritdoc/>
    public override int GetHashCode()
    {
        unchecked
        {
            int hash = 17;
            hash = hash * 23 + Normal.GetHashCode();
            hash = hash * 23 + D.GetHashCode();
            return hash;
        }
    }

    #endregion
}
