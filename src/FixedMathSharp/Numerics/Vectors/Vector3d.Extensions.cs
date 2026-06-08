//=======================================================================
// Vector3d.Extensions.cs
//=======================================================================
// MIT License, Copyright (c) 2024–present David Oravsky (mrdav30)
// See LICENSE file in the project root for full license information.
//=======================================================================

using FixedMathSharp.Bounds;
using System.Runtime.CompilerServices;

namespace FixedMathSharp;

/// <summary>
/// Provides extension methods for the Vector3d type to support additional vector operations, comparisons, and conversions.
/// </summary>
public static partial class Vector3dExtensions
{
    #region Vector3d Operations

    /// <inheritdoc cref="Vector3d.Clamp(Vector3d, Vector3d, Vector3d)" />
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public static Vector3d Clamp(this Vector3d value, Vector3d min, Vector3d max) => Vector3d.Clamp(value, min, max);

    /// <inheritdoc cref="Vector3d.Clamp(Vector3d, Vector3d, Vector3d)" />
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public static Vector3d ClampOne(this Vector3d value) => Vector3d.Clamp(value, Vector3d.Negative, Vector3d.One);

    /// <inheritdoc cref="Vector3d.ClampMagnitude(Vector3d, Fixed64)" />
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public static Vector3d ClampMagnitude(this Vector3d value, Fixed64 maxMagnitude) =>
        Vector3d.ClampMagnitude(value, maxMagnitude);

    /// <summary>
    /// Checks if the distance between two vectors is less than or equal to a specified factor.
    /// </summary>
    /// <param name="me">The current vector.</param>
    /// <param name="other">The vector to compare distance to.</param>
    /// <param name="factor">The maximum allowable distance.</param>
    /// <returns>True if the distance between the vectors is less than or equal to the factor, false otherwise.</returns>
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public static bool CheckDistance(this Vector3d me, Vector3d other, Fixed64 factor)
    {
        if (factor < Fixed64.Zero)
            return false;

        return Vector3d.DistanceSquared(me, other) <= factor * factor;
    }

    /// <inheritdoc cref="Vector3d.Distance(Vector3d, Vector3d)" />
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public static Fixed64 Distance(this Vector3d start, Vector3d end) => Vector3d.Distance(start, end);

    /// <inheritdoc cref="Vector3d.DistanceSquared(Vector3d, Vector3d)" />
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public static Fixed64 DistanceSquared(this Vector3d start, Vector3d end) => Vector3d.DistanceSquared(start, end);

    /// <inheritdoc cref="Vector3d.Dot(Vector3d, Vector3d)" />
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public static Fixed64 Dot(this Vector3d lhs, Vector3d rhs) => Vector3d.Dot(lhs, rhs);

    /// <inheritdoc cref="Vector3d.Cross(Vector3d, Vector3d)" />
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public static Vector3d Cross(this Vector3d lhs, Vector3d rhs) => Vector3d.Cross(lhs, rhs);

    /// <inheritdoc cref="Vector3d.CrossProduct(Vector3d, Vector3d)" />
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public static Fixed64 CrossProduct(this Vector3d lhs, Vector3d rhs) => Vector3d.CrossProduct(lhs, rhs);

    /// <inheritdoc cref="Vector3d.Lerp(Vector3d, Vector3d, Fixed64)" />
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public static Vector3d Lerp(this Vector3d start, Vector3d end, Fixed64 amount) => Vector3d.Lerp(start, end, amount);

    /// <inheritdoc cref="Vector3d.UnclampedLerp(Vector3d, Vector3d, Fixed64)" />
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public static Vector3d UnclampedLerp(this Vector3d start, Vector3d end, Fixed64 amount) =>
        Vector3d.UnclampedLerp(start, end, amount);

    /// <inheritdoc cref="Vector3d.Slerp(Vector3d, Vector3d, Fixed64)" />
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public static Vector3d Slerp(this Vector3d start, Vector3d end, Fixed64 amount) => Vector3d.Slerp(start, end, amount);

    /// <inheritdoc cref="Vector3d.Project(Vector3d, Vector3d)" />
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public static Vector3d Project(this Vector3d vector, Vector3d onNormal) => Vector3d.Project(vector, onNormal);

    /// <inheritdoc cref="Vector3d.ProjectOnPlane(Vector3d, Vector3d)" />
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public static Vector3d ProjectOnPlane(this Vector3d vector, Vector3d planeNormal) =>
        Vector3d.ProjectOnPlane(vector, planeNormal);

    /// <inheritdoc cref="Vector3d.ProjectOnPlane(Vector3d, FixedPlane)" />
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public static Vector3d ProjectOnPlane(this Vector3d point, FixedPlane plane) => Vector3d.ProjectOnPlane(point, plane);

    /// <inheritdoc cref="Vector3d.Angle(Vector3d, Vector3d)" />
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public static Fixed64 Angle(this Vector3d from, Vector3d to) => Vector3d.Angle(from, to);

    /// <inheritdoc cref="Vector3d.Reflect(Vector3d, Vector3d)" />
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public static Vector3d Reflect(this Vector3d vector, Vector3d normal) => Vector3d.Reflect(vector, normal);

    /// <inheritdoc cref="Vector3d.Rotate(Vector3d, Vector3d, FixedQuaternion)" />
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public static Vector3d Rotate(this Vector3d source, Vector3d position, FixedQuaternion rotation) =>
        Vector3d.Rotate(source, position, rotation);

    /// <inheritdoc cref="Vector3d.InverseRotate(Vector3d, Vector3d, FixedQuaternion)" />
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public static Vector3d InverseRotate(this Vector3d source, Vector3d position, FixedQuaternion rotation) =>
        Vector3d.InverseRotate(source, position, rotation);

    /// <inheritdoc cref="Vector3d.Transform(Vector3d, Fixed4x4)" />
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public static Vector3d Transform(this Vector3d vector, Fixed4x4 matrix) => Vector3d.Transform(vector, matrix);

    #endregion

    #region Conversion

    /// <inheritdoc cref="Vector3d.ToDegrees(Vector3d)" />
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public static Vector3d ToDegrees(this Vector3d radians) => Vector3d.ToDegrees(radians);

    /// <inheritdoc cref="Vector3d.ToRadians(Vector3d)" />
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public static Vector3d ToRadians(this Vector3d degrees) => Vector3d.ToRadians(degrees);

    /// <inheritdoc cref="Vector3d.Abs(Vector3d)" />
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public static Vector3d Abs(this Vector3d value) => Vector3d.Abs(value);

    /// <inheritdoc cref="Vector3d.Sign(Vector3d)" />
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public static Vector3d Sign(this Vector3d value) => Vector3d.Sign(value);

    #endregion

    #region Equality

    /// <summary>
    /// Compares two vectors for approximate equality, allowing a fixed absolute difference.
    /// </summary>
    /// <param name="me">The current vector.</param>
    /// <param name="other">The vector to compare against.</param>
    /// <param name="allowedDifference">The allowed absolute difference between each component.</param>
    /// <returns>True if the components are within the allowed difference, false otherwise.</returns>
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public static bool FuzzyEqualAbsolute(this Vector3d me, Vector3d other, Fixed64 allowedDifference) =>
        (me.X - other.X).Abs() <= allowedDifference
        && (me.Y - other.Y).Abs() <= allowedDifference
        && (me.Z - other.Z).Abs() <= allowedDifference;

    /// <summary>
    /// Compares two vectors for approximate equality, allowing a fractional difference (percentage).
    /// Handles zero components by only using the allowed percentage difference.
    /// </summary>
    /// <param name="me">The current vector.</param>
    /// <param name="other">The vector to compare against.</param>
    /// <param name="percentage">The allowed fractional difference (percentage) for each component.</param>
    /// <returns>True if the components are within the allowed percentage difference, false otherwise.</returns>
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public static bool FuzzyEqual(this Vector3d me, Vector3d other, Fixed64? percentage = null)
    {
        Fixed64 p = percentage ?? Fixed64.Epsilon;
        return me.X.FuzzyComponentEqual(other.X, p) &&
                me.Y.FuzzyComponentEqual(other.Y, p) &&
                me.Z.FuzzyComponentEqual(other.Z, p);
    }

    #endregion
}
