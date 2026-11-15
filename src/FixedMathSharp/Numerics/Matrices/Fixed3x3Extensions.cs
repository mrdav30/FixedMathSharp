//=======================================================================
// Fixed3x3Extensions.cs
//=======================================================================
// MIT License, Copyright (c) 2024-present David Oravsky (mrdav30)
// See LICENSE file in the project root for full license information.
//=======================================================================

using System.Runtime.CompilerServices;

namespace FixedMathSharp;

/// <summary>Provides receiver-based transforms and comparisons for <see cref="Fixed3x3"/>.</summary>
public static class Fixed3x3Extensions
{
    #region Transformations

    /// <inheritdoc cref="Fixed3x3.ExtractScaleMagnitudes(Fixed3x3)" />
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public static Vector3d ExtractScaleMagnitudes(this Fixed3x3 matrix) => Fixed3x3.ExtractScaleMagnitudes(matrix);

    /// <inheritdoc cref="Fixed3x3.ExtractLossyScale(Fixed3x3)" />
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public static Vector3d ExtractLossyScale(this Fixed3x3 matrix) => Fixed3x3.ExtractLossyScale(matrix);

    /// <summary>Interpolates the corresponding entries of two matrices.</summary>
    /// <param name="matrix">The starting matrix.</param>
    /// <param name="target">The ending matrix.</param>
    /// <param name="amount">The interpolation fraction, clamped to the inclusive range from zero to one.</param>
    /// <returns>The matrix of entrywise interpolated values.</returns>
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public static Fixed3x3 Lerp(this Fixed3x3 matrix, Fixed3x3 target, Fixed64 amount) =>
        Fixed3x3.Lerp(matrix, target, amount);

    /// <inheritdoc cref="Fixed3x3.Transpose(Fixed3x3)" />
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public static Fixed3x3 Transpose(this Fixed3x3 matrix) => Fixed3x3.Transpose(matrix);

    /// <inheritdoc cref="Fixed3x3.TransformDirection(Fixed3x3, Vector3d)" />
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public static Vector3d TransformDirection(this Fixed3x3 matrix, Vector3d direction) =>
        Fixed3x3.TransformDirection(matrix, direction);

    /// <inheritdoc cref="Fixed3x3.InverseTransformDirection(Fixed3x3, Vector3d)" />
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public static Vector3d InverseTransformDirection(this Fixed3x3 matrix, Vector3d direction) =>
        Fixed3x3.InverseTransformDirection(matrix, direction);

    #endregion

    #region Equality

    /// <summary>
    /// Compares two Fixed3x3 for approximate equality, allowing a fixed absolute difference between components.
    /// </summary>
    /// <param name="f1">The current Fixed3x3.</param>
    /// <param name="f2">The Fixed3x3 to compare against.</param>
    /// <param name="allowedDifference">The inclusive absolute error allowed per matrix entry. Use a nonnegative value.</param>
    /// <returns><see langword="true"/> if every absolute fixed-point difference is at most the tolerance, otherwise <see langword="false"/>.</returns>
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public static bool FuzzyEqualAbsolute(this Fixed3x3 f1, Fixed3x3 f2, Fixed64 allowedDifference) =>
         (f1.M11 - f2.M11).Abs() <= allowedDifference &&
        (f1.M12 - f2.M12).Abs() <= allowedDifference &&
        (f1.M13 - f2.M13).Abs() <= allowedDifference &&
        (f1.M21 - f2.M21).Abs() <= allowedDifference &&
        (f1.M22 - f2.M22).Abs() <= allowedDifference &&
        (f1.M23 - f2.M23).Abs() <= allowedDifference &&
        (f1.M31 - f2.M31).Abs() <= allowedDifference &&
        (f1.M32 - f2.M32).Abs() <= allowedDifference &&
        (f1.M33 - f2.M33).Abs() <= allowedDifference;

    /// <summary>Compares matrix entries using an error limit relative to the first matrix.</summary>
    /// <param name="f1">The reference matrix used to scale each entry's tolerance.</param>
    /// <param name="f2">The matrix to compare against the reference.</param>
    /// <param name="percentage">The nonnegative error fraction. For example, 0.01 specifies 1%. Null uses <see cref="Fixed64.Epsilon"/>.</param>
    /// <returns><see langword="true"/> if every absolute fixed-point difference is at most its limit, otherwise <see langword="false"/>.</returns>
    /// <remarks>
    /// Each limit is the absolute reference entry multiplied by the fraction. If the absolute
    /// reference entry is smaller than <see cref="Fixed64.Epsilon"/>, the fraction itself is used as
    /// an absolute limit. Swapping the operands can change the result.
    /// </remarks>
    public static bool FuzzyEqual(this Fixed3x3 f1, Fixed3x3 f2, Fixed64? percentage = null)
    {
        Fixed64 p = percentage ?? Fixed64.Epsilon;
        return f1.M11.FuzzyComponentEqual(f2.M11, p) &&
               f1.M12.FuzzyComponentEqual(f2.M12, p) &&
               f1.M13.FuzzyComponentEqual(f2.M13, p) &&
               f1.M21.FuzzyComponentEqual(f2.M21, p) &&
               f1.M22.FuzzyComponentEqual(f2.M22, p) &&
               f1.M23.FuzzyComponentEqual(f2.M23, p) &&
               f1.M31.FuzzyComponentEqual(f2.M31, p) &&
               f1.M32.FuzzyComponentEqual(f2.M32, p) &&
               f1.M33.FuzzyComponentEqual(f2.M33, p);
    }

    #endregion
}
