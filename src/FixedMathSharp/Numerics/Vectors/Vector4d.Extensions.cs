//=======================================================================
// Vector4d.Extensions.cs
//=======================================================================
// MIT License, Copyright (c) 2024–present David Oravsky (mrdav30)
// See LICENSE file in the project root for full license information.
//=======================================================================

using System.Runtime.CompilerServices;

namespace FixedMathSharp;

/// <summary>
/// Provides extension methods for the Vector4d type to support additional vector operations and comparisons.
/// </summary>
public static partial class Vector4dExtensions
{
    #region Vector4d Operations

    /// <inheritdoc cref="Vector4d.Clamp(Vector4d, Vector4d, Vector4d)" />
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public static Vector4d Clamp(this Vector4d value, Vector4d min, Vector4d max) => Vector4d.Clamp(value, min, max);

    /// <inheritdoc cref="Vector4d.Clamp(Vector4d, Vector4d, Vector4d)" />
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public static Vector4d ClampOne(this Vector4d value) => Vector4d.Clamp(value, Vector4d.Negative, Vector4d.One);

    /// <inheritdoc cref="Vector4d.Lerp(Vector4d, Vector4d, Fixed64)" />
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public static Vector4d Lerp(this Vector4d start, Vector4d end, Fixed64 amount) => Vector4d.Lerp(start, end, amount);

    /// <inheritdoc cref="Vector4d.UnclampedLerp(Vector4d, Vector4d, Fixed64)" />
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public static Vector4d UnclampedLerp(this Vector4d start, Vector4d end, Fixed64 amount) =>
        Vector4d.UnclampedLerp(start, end, amount);

    /// <inheritdoc cref="Vector4d.Midpoint(Vector4d, Vector4d)" />
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public static Vector4d Midpoint(this Vector4d value, Vector4d other) => Vector4d.Midpoint(value, other);

    /// <inheritdoc cref="Vector4d.Max(Vector4d, Vector4d)" />
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public static Vector4d Max(this Vector4d value, Vector4d other) => Vector4d.Max(value, other);

    /// <inheritdoc cref="Vector4d.Min(Vector4d, Vector4d)" />
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public static Vector4d Min(this Vector4d value, Vector4d other) => Vector4d.Min(value, other);

    /// <inheritdoc cref="Vector4d.Transform(Fixed4x4, Vector4d)" />
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public static Vector4d Transform(this Vector4d vector, Fixed4x4 matrix) => Vector4d.Transform(matrix, vector);

    #endregion

    #region Conversion

    /// <inheritdoc cref="Vector4d.Abs(Vector4d)" />
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public static Vector4d Abs(this Vector4d value) => Vector4d.Abs(value);

    /// <inheritdoc cref="Vector4d.Sign(Vector4d)" />
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public static Vector4d Sign(this Vector4d value) => Vector4d.Sign(value);

    #endregion

    #region Equality

    /// <summary>
    /// Compares two vectors for approximate equality, allowing a fixed absolute difference.
    /// </summary>
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public static bool FuzzyEqualAbsolute(this Vector4d me, Vector4d other, Fixed64 allowedDifference) =>
        (me.X - other.X).Abs() <= allowedDifference &&
        (me.Y - other.Y).Abs() <= allowedDifference &&
        (me.Z - other.Z).Abs() <= allowedDifference &&
        (me.W - other.W).Abs() <= allowedDifference;

    /// <summary>
    /// Compares two vectors for approximate equality, allowing a fractional difference.
    /// </summary>
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public static bool FuzzyEqual(this Vector4d me, Vector4d other, Fixed64? percentage = null)
    {
        Fixed64 p = percentage ?? Fixed64.Epsilon;
        return me.X.FuzzyComponentEqual(other.X, p) &&
               me.Y.FuzzyComponentEqual(other.Y, p) &&
               me.Z.FuzzyComponentEqual(other.Z, p) &&
               me.W.FuzzyComponentEqual(other.W, p);
    }

    #endregion
}
