//=======================================================================
// Fixed3x3.Extensions.cs
//=======================================================================
// MIT License, Copyright (c) 2024–present David Oravsky (mrdav30)
// See LICENSE file in the project root for full license information.
//=======================================================================

using System.Runtime.CompilerServices;

namespace FixedMathSharp;

/// <summary>
/// Provides extension methods for the Fixed3x3 structure, enabling additional transformation and comparison operations.
/// </summary>
public static class Fixed3x3Extensions
{
    #region Transformations

    /// <inheritdoc cref="Fixed3x3.ExtractScale(Fixed3x3)" />
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public static Vector3d ExtractScale(this Fixed3x3 matrix) => Fixed3x3.ExtractScale(matrix);

    /// <inheritdoc cref="Fixed3x3.ExtractLossyScale(Fixed3x3)" />
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public static Vector3d ExtractLossyScale(this Fixed3x3 matrix) => Fixed3x3.ExtractLossyScale(matrix);

    /// <inheritdoc cref="Fixed3x3.SetScale(Fixed3x3, Vector3d)" />
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public static Fixed3x3 SetScale(this ref Fixed3x3 matrix, Vector3d localScale) =>
        matrix = Fixed3x3.SetScale(matrix, localScale);

    /// <inheritdoc cref="Fixed3x3.SetGlobalScale(Fixed3x3, Vector3d)" />
    public static Fixed3x3 SetGlobalScale(this ref Fixed3x3 matrix, Vector3d globalScale) =>
        matrix = Fixed3x3.SetGlobalScale(matrix, globalScale);

    /// <inheritdoc cref="Fixed3x3.Lerp(Fixed3x3, Fixed3x3, Fixed64)" />
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
    /// <param name="allowedDifference">The allowed absolute difference between each component.</param>
    /// <returns>True if the components are within the allowed difference, false otherwise.</returns>
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

    /// <summary>
    /// Compares two Fixed3x3 for approximate equality, allowing a fractional percentage (defaults to ~1%) difference between components.
    /// </summary>
    /// <param name="f1">The current Fixed3x3.</param>
    /// <param name="f2">The Fixed3x3 to compare against.</param>
    /// <param name="percentage">The allowed fractional difference (percentage) for each component.</param>
    /// <returns>True if the components are within the allowed percentage difference, false otherwise.</returns>
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
