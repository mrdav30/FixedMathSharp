//=======================================================================
// FixedPlane.cs
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
    /// <param name="normal">The normal in plane coordinates. The constructor retains its magnitude.</param>
    /// <param name="d">The constant in the plane equation <c>dot(Normal, point) + D = 0</c>.</param>
    [JsonConstructor]
    public FixedPlane(Vector3d normal, Fixed64 d)
    {
        Normal = normal;
        D = d;
    }

    /// <summary>
    /// Initializes a new plane from the normal components and distance component.
    /// </summary>
    /// <param name="x">The X component of the normal.</param>
    /// <param name="y">The Y component of the normal.</param>
    /// <param name="z">The Z component of the normal.</param>
    /// <param name="d">The constant in the plane equation <c>dot(Normal, point) + D = 0</c>.</param>
    public FixedPlane(Fixed64 x, Fixed64 y, Fixed64 z, Fixed64 d)
        : this(new Vector3d(x, y, z), d)
    {
    }

    /// <summary>
    /// Initializes a new plane that contains the specified points.
    /// </summary>
    /// <param name="a">The first point on the plane in winding order.</param>
    /// <param name="b">The second point on the plane.</param>
    /// <param name="c">The third point, noncollinear with the first two for a valid plane.</param>
    public FixedPlane(Vector3d a, Vector3d b, Vector3d c)
    {
        Vector3d ab = b - a;
        Vector3d ac = c - a;

        Normal = Vector3d.Cross(ab, ac).NormalizeInPlace();
        D = -Vector3d.Dot(Normal, a);
    }

    /// <summary>
    /// Initializes a new plane that contains the specified point with the specified normal.
    /// </summary>
    /// <param name="pointOnPlane">A point on the plane in the same coordinate space as the normal.</param>
    /// <param name="normal">The normal in plane coordinates. The constructor retains its magnitude.</param>
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
    /// <param name="value">The point in the plane's coordinate space.</param>
    /// <returns>The plane equation evaluated at the point. With a unit normal, this is a signed distance.</returns>
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public Fixed64 DotCoordinate(Vector3d value)
    {
        return Vector3d.Dot(Normal, value) + D;
    }

    /// <summary>
    /// Gets the dot product of the specified vector and this plane's normal.
    /// </summary>
    /// <param name="value">The vector in the same coordinate space as the normal.</param>
    /// <returns>The signed dot product with the normal, independent of the plane's constant term.</returns>
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public Fixed64 DotNormal(Vector3d value)
    {
        return Vector3d.Dot(Normal, value);
    }

    /// <summary>
    /// Rescales this plane in place using the reciprocal of its normal length.
    /// </summary>
    /// <exception cref="ArithmeticException">The normal length is zero.</exception>
    /// <remarks>
    /// Uses the rounded, saturating scale of <see cref="GetNormalized(FixedPlane)"/>.
    /// The plane is replaced only after the calculation succeeds.
    /// </remarks>
    public void NormalizeInPlace()
    {
        this = GetNormalized(this);
    }

    /// <summary>
    /// Rescales a plane using the reciprocal of its normal length.
    /// </summary>
    /// <param name="value">The plane to normalize.</param>
    /// <returns>A copy with both the normal and constant term multiplied by the fixed-point reciprocal.</returns>
    /// <remarks>
    /// The reciprocal and products use rounded, saturating fixed-point arithmetic. In particular,
    /// a very small normal length can saturate the reciprocal and produce a normal shorter than one.
    /// </remarks>
    /// <exception cref="ArithmeticException">The normal length is zero.</exception>
    public static FixedPlane GetNormalized(FixedPlane value)
    {
        Fixed64 length = value.Normal.Magnitude;

        if (length == Fixed64.Zero)
            throw new ArithmeticException("Cannot normalize a plane with a zero-length normal.");

        Fixed64 factor = Fixed64.One / length;
        return new FixedPlane(value.Normal * factor, value.D * factor);
    }

    /// <summary>
    /// Classifies a bounding box relative to this plane.
    /// </summary>
    /// <param name="box">The box in the same coordinate space as this instance.</param>
    /// <returns>
    /// <see cref="FixedPlaneIntersectionType.Front"/> on the positive side,
    /// <see cref="FixedPlaneIntersectionType.Back"/> on the negative side, or
    /// <see cref="FixedPlaneIntersectionType.Intersecting"/> for touching or crossing.
    /// </returns>
    public FixedPlaneIntersectionType Intersects(FixedBoundBox box)
    {
        return IntersectsBoxLike(box.Min, box.Max);
    }

    /// <summary>
    /// Classifies a bounding sphere relative to this plane.
    /// </summary>
    /// <param name="sphere">The sphere in the same coordinate space as this instance.</param>
    /// <returns>
    /// <see cref="FixedPlaneIntersectionType.Front"/> on the positive side,
    /// <see cref="FixedPlaneIntersectionType.Back"/> on the negative side, or
    /// <see cref="FixedPlaneIntersectionType.Intersecting"/> for touching or crossing.
    /// </returns>
    /// <remarks>The radius comparison assumes a unit-length plane normal.</remarks>
    public FixedPlaneIntersectionType Intersects(FixedBoundSphere sphere)
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
            Normal.X >= Fixed64.Zero ? max.X : min.X,
            Normal.Y >= Fixed64.Zero ? max.Y : min.Y,
            Normal.Z >= Fixed64.Zero ? max.Z : min.Z);

        if (DotCoordinate(positive) < Fixed64.Zero)
            return FixedPlaneIntersectionType.Back;

        Vector3d negative = new(
            Normal.X >= Fixed64.Zero ? min.X : max.X,
            Normal.Y >= Fixed64.Zero ? min.Y : max.Y,
            Normal.Z >= Fixed64.Zero ? min.Z : max.Z);

        if (DotCoordinate(negative) > Fixed64.Zero)
            return FixedPlaneIntersectionType.Front;

        return FixedPlaneIntersectionType.Intersecting;
    }

    /// <summary>
    /// Deconstructs the plane into its normal and distance components.
    /// </summary>
    /// <param name="normal">The stored normal vector.</param>
    /// <param name="d">The stored constant term.</param>
    public void Deconstruct(out Vector3d normal, out Fixed64 d)
    {
        normal = Normal;
        d = D;
    }

    #endregion

    #region Operators

    /// <summary>Tests whether the stored geometry is equal.</summary>
    /// <param name="left">The first value to compare.</param>
    /// <param name="right">The second value to compare.</param>
    /// <returns><see langword="true"/> when the normals and constant terms match and <see langword="false"/> otherwise.</returns>
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public static bool operator ==(FixedPlane left, FixedPlane right) => left.Equals(right);

    /// <summary>Tests whether the stored geometry is different.</summary>
    /// <param name="left">The first value to compare.</param>
    /// <param name="right">The second value to compare.</param>
    /// <returns><see langword="true"/> when the stored values differ and <see langword="false"/> otherwise.</returns>
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public static bool operator !=(FixedPlane left, FixedPlane right) => !left.Equals(right);

    #endregion

    #region Equality

    /// <summary>Compares the stored geometry for equality.</summary>
    /// <param name="obj">The value to compare with this instance.</param>
    /// <returns>
    /// <see langword="true"/> when <paramref name="obj"/> is a <see cref="FixedPlane"/> and the normals and
    /// constant terms match and <see langword="false"/> otherwise.
    /// </returns>
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public override bool Equals(object? obj) => obj is FixedPlane other && Equals(other);

    /// <summary>Compares the stored geometry for equality.</summary>
    /// <param name="other">The value to compare with this instance.</param>
    /// <returns><see langword="true"/> when the normals and constant terms match and <see langword="false"/> otherwise.</returns>
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public bool Equals(FixedPlane other) => Normal.Equals(other.Normal) && D.Equals(other.D);

    /// <summary>Computes a hash code for the stored geometry.</summary>
    /// <returns>A hash code consistent with structural equality.</returns>
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
