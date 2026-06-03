//=======================================================================
// Vector3d.Extensions.cs
//=======================================================================
// MIT License, Copyright (c) 2024–present David Oravsky (mrdav30)
// See LICENSE file in the project root for full license information.
//=======================================================================

using System.Runtime.CompilerServices;

namespace FixedMathSharp;

/// <summary>
/// Provides extension methods for the Vector3d type to support additional vector operations, comparisons, and conversions.
/// </summary>
public static partial class Vector3dExtensions
{
    #region Vector3d Operations

    /// <summary>
    /// Clamps each component of the vector to the range [-1, 1] in place and returns the modified vector.
    /// </summary>
    /// <param name="v">The vector to clamp.</param>
    /// <returns>The clamped vector with each component between -1 and 1.</returns>
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public static Vector3d ClampOneInPlace(this Vector3d v)
    {
        v.X = v.X.ClampOne();
        v.Y = v.Y.ClampOne();
        v.Z = v.Z.ClampOne();
        return v;
    }

    /// <inheritdoc cref="Vector3d.ClampMagnitude(Vector3d, Fixed64)" />
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
        var dis = Vector3d.Distance(me, other);
        return dis <= factor;
    }

    /// <inheritdoc cref="Vector3d.Rotate(Vector3d, Vector3d, FixedQuaternion)" />
    public static Vector3d Rotate(this Vector3d source, Vector3d position, FixedQuaternion rotation) =>
        Vector3d.Rotate(source, position, rotation);

    /// <inheritdoc cref="Vector3d.InverseRotate(Vector3d, Vector3d, FixedQuaternion)" />
    public static Vector3d InverseRotate(this Vector3d source, Vector3d position, FixedQuaternion rotation) =>
        Vector3d.InverseRotate(source, position, rotation);

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
    public static Vector3d Sign(Vector3d value) => Vector3d.Sign(value);

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