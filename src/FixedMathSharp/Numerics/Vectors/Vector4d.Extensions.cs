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

    /// <summary>
    /// Clamps each component of the vector to the range [-1, 1] and returns the clamped vector.
    /// </summary>
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public static Vector4d ClampOneInPlace(this Vector4d v)
    {
        v.X = v.X.ClampOne();
        v.Y = v.Y.ClampOne();
        v.Z = v.Z.ClampOne();
        v.W = v.W.ClampOne();
        return v;
    }

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
