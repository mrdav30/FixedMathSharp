//=======================================================================
// Vector2d.Extensions.cs
//=======================================================================
// MIT License, Copyright (c) 2024–present David Oravsky (mrdav30)
// See LICENSE file in the project root for full license information.
//=======================================================================

using System.Runtime.CompilerServices;

namespace FixedMathSharp;

/// <summary>
/// Provides extension methods for the Vector2d type to support additional vector operations, comparisons, and conversions.
/// </summary>
public static partial class Vector2dExtensions
{
    #region Vector2d Operations

    /// <inheritdoc cref="Vector2d.Clamp(Vector2d, Vector2d, Vector2d)" />
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public static Vector2d Clamp(this Vector2d value, Vector2d min, Vector2d max) => Vector2d.Clamp(value, min, max);

    /// <inheritdoc cref="Vector2d.Clamp(Vector2d, Vector2d, Vector2d)" />
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public static Vector2d ClampOne(this Vector2d value) => Vector2d.Clamp(value, Vector2d.Negative, Vector2d.One);

    /// <summary>
    /// Checks if the distance between two vectors is less than or equal to a specified factor.
    /// </summary>
    /// <param name="me">The current vector.</param>
    /// <param name="other">The vector to compare distance to.</param>
    /// <param name="factor">The maximum allowable distance.</param>
    /// <returns>True if the distance between the vectors is less than or equal to the factor, false otherwise.</returns>  
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public static bool CheckDistance(this Vector2d me, Vector2d other, Fixed64 factor)
    {
        if (factor < Fixed64.Zero)
            return false;

        return Vector2d.DistanceSquared(me, other) <= factor * factor;
    }

    /// <inheritdoc cref="Vector2d.Rotate(Vector2d, Fixed64)" />
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public static Vector2d Rotate(this Vector2d vec, Fixed64 angleInRadians) => Vector2d.Rotate(vec, angleInRadians);

    /// <inheritdoc cref="Vector2d.Abs(Vector2d)" />
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public static Vector2d Abs(this Vector2d value) => Vector2d.Abs(value);

    /// <inheritdoc cref="Vector2d.Sign(Vector2d)" />
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public static Vector2d Sign(this Vector2d value) => Vector2d.Sign(value);

    #endregion

    #region Conversion

    /// <inheritdoc cref="Vector2d.ToDegrees(Vector2d)" />
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public static Vector2d ToDegrees(this Vector2d radians) => Vector2d.ToDegrees(radians);

    /// <inheritdoc cref="Vector2d.ToRadians(Vector2d)" />
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public static Vector2d ToRadians(this Vector2d degrees) => Vector2d.ToRadians(degrees);

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
    public static bool FuzzyEqualAbsolute(this Vector2d me, Vector2d other, Fixed64 allowedDifference)
    {
        return (me.X - other.X).Abs() <= allowedDifference &&
               (me.Y - other.Y).Abs() <= allowedDifference;
    }

    /// <summary>
    /// Compares two vectors for approximate equality, allowing a fractional difference (percentage).
    /// Handles zero components by only using the allowed percentage difference.
    /// </summary>
    /// <param name="me">The current vector.</param>
    /// <param name="other">The vector to compare against.</param>
    /// <param name="percentage">The allowed fractional difference (percentage) for each component.</param>
    /// <returns>True if the components are within the allowed percentage difference, false otherwise.</returns>
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public static bool FuzzyEqual(this Vector2d me, Vector2d other, Fixed64? percentage = null)
    {
        Fixed64 p = percentage ?? Fixed64.Epsilon;
        return me.X.FuzzyComponentEqual(other.X, p) && me.Y.FuzzyComponentEqual(other.Y, p);
    }

    #endregion
}
