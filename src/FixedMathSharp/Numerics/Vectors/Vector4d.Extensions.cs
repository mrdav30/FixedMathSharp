//=======================================================================
// Vector4d.Extensions.cs
//=======================================================================
// MIT License, Copyright (c) 2024-present David Oravsky (mrdav30)
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

    /// <summary>Clamps every component to the interval [−1, 1].</summary>
    /// <param name="value">The vector to clamp.</param>
    /// <returns>A copy with each component independently clamped.</returns>
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public static Vector4d ClampOne(this Vector4d value) => Vector4d.Clamp(value, Vector4d.Negative, Vector4d.One);

    /// <summary>Linearly interpolates between two vectors.</summary>
    /// <param name="start">The starting vector.</param>
    /// <param name="end">The ending vector in the same coordinate space.</param>
    /// <param name="amount">The interpolation fraction, clamped to [0, 1].</param>
    /// <returns>
    /// <paramref name="start"/> for factors at or below zero, <paramref name="end"/> for factors at or above one,
    /// or the interpolated vector rounded to nearest with ties to even per component.
    /// </returns>
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public static Vector4d Lerp(this Vector4d start, Vector4d end, Fixed64 amount) => Vector4d.Lerp(start, end, amount);

    /// <summary>Linearly interpolates between two vectors.</summary>
    /// <param name="start">The starting vector.</param>
    /// <param name="end">The ending vector in the same coordinate space.</param>
    /// <param name="amount">The interpolation fraction. Values outside [0, 1] extrapolate.</param>
    /// <returns>
    /// <paramref name="start"/> at zero, <paramref name="end"/> at one,
    /// or the result of <c>start + (end − start) * amount</c> using saturating component arithmetic.
    /// </returns>
    /// <remarks>A saturated component difference can cause results near a factor of one to differ sharply from the ending vector.</remarks>
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public static Vector4d UnclampedLerp(this Vector4d start, Vector4d end, Fixed64 amount) =>
        Vector4d.UnclampedLerp(start, end, amount);

    /// <summary>Computes the component-wise midpoint.</summary>
    /// <param name="value">The first vector.</param>
    /// <param name="other">The second vector in the same coordinate space.</param>
    /// <returns>The midpoint rounded to the nearest representable vector with ties to even per component.</returns>
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public static Vector4d Midpoint(this Vector4d value, Vector4d other) => Vector4d.Midpoint(value, other);

    /// <summary>Selects the component-wise maxima.</summary>
    /// <param name="value">The first vector.</param>
    /// <param name="other">The second vector in the same coordinate space.</param>
    /// <returns>A vector containing the larger value from each matching component pair.</returns>
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public static Vector4d Max(this Vector4d value, Vector4d other) => Vector4d.Max(value, other);

    /// <summary>Selects the component-wise minima.</summary>
    /// <param name="value">The first vector.</param>
    /// <param name="other">The second vector in the same coordinate space.</param>
    /// <returns>A vector containing the smaller value from each matching component pair.</returns>
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

    /// <summary>Compares two vectors within a component-wise absolute tolerance.</summary>
    /// <param name="me">The reference vector.</param>
    /// <param name="other">The vector to compare in the same coordinate space.</param>
    /// <param name="allowedDifference">The nonnegative absolute tolerance per component, in input units.</param>
    /// <returns><see langword="true"/> if all corresponding components agree within the tolerance, including the boundary, otherwise <see langword="false"/>.</returns>
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public static bool FuzzyEqualAbsolute(this Vector4d me, Vector4d other, Fixed64 allowedDifference) =>
        (me.X - other.X).Abs() <= allowedDifference &&
        (me.Y - other.Y).Abs() <= allowedDifference &&
        (me.Z - other.Z).Abs() <= allowedDifference &&
        (me.W - other.W).Abs() <= allowedDifference;

    /// <summary>Compares two vectors within a component-wise relative tolerance.</summary>
    /// <param name="me">The reference vector.</param>
    /// <param name="other">The vector to compare in the same coordinate space.</param>
    /// <param name="percentage">The allowed relative error as a nonnegative fraction. Null selects <see cref="Fixed64.Epsilon"/>.</param>
    /// <returns><see langword="true"/> if all corresponding components agree within the tolerance, including the boundary, otherwise <see langword="false"/>.</returns>
    /// <remarks>The tolerance is relative to each reference component’s magnitude. A magnitude below <see cref="Fixed64.Epsilon"/> uses the fraction as an absolute tolerance.</remarks>
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
