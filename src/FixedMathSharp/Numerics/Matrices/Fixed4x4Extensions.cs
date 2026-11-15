//=======================================================================
// Fixed4x4Extensions.cs
//=======================================================================
// MIT License, Copyright (c) 2024-present David Oravsky (mrdav30)
// See LICENSE file in the project root for full license information.
//=======================================================================

using System.Runtime.CompilerServices;

namespace FixedMathSharp;

/// <summary>
/// Provides extension methods for the Fixed4x4 structure to simplify common matrix operations 
/// such as extracting scale, setting transformation components, and transforming points.
/// </summary>
public static class Fixed4x4Extensions
{
    #region Extraction, and Setters

    /// <inheritdoc cref="Fixed4x4.ExtractLossyScale(Fixed4x4)" />
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public static Vector3d ExtractLossyScale(this Fixed4x4 matrix) => Fixed4x4.ExtractLossyScale(matrix);

    /// <inheritdoc cref="Fixed4x4.ExtractScaleMagnitudes(Fixed4x4)" />
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public static Vector3d ExtractScaleMagnitudes(this Fixed4x4 matrix) => Fixed4x4.ExtractScaleMagnitudes(matrix);

    /// <inheritdoc cref="Fixed4x4.ExtractTranslation(Fixed4x4)" />
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public static Vector3d ExtractTranslation(this Fixed4x4 matrix) => Fixed4x4.ExtractTranslation(matrix);

    /// <inheritdoc cref="Fixed4x4.ExtractRotation(Fixed4x4)" />
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public static FixedQuaternion ExtractRotation(this Fixed4x4 matrix) => Fixed4x4.ExtractRotation(matrix);

    /// <inheritdoc cref="Fixed4x4.Decompose(Fixed4x4, out Vector3d, out FixedQuaternion, out Vector3d)" />
    public static bool Decompose(
        this Fixed4x4 matrix,
        out Vector3d translation,
        out FixedQuaternion rotation,
        out Vector3d scale) => Fixed4x4.Decompose(matrix, out translation, out rotation, out scale);

    /// <summary>Replaces the matrix's translation entries in place.</summary>
    /// <param name="matrix">The matrix whose fourth row is updated.</param>
    /// <param name="position">The new translation along X, Y, and Z.</param>
    /// <returns>The updated matrix value.</returns>
    public static Fixed4x4 SetTranslation(this ref Fixed4x4 matrix, Vector3d position) =>
        matrix = Fixed4x4.SetTranslation(matrix, position);

    /// <summary>Replaces the matrix's upper-left rotation basis in place.</summary>
    /// <param name="matrix">The matrix to update, retaining its unsigned basis scale and translation.</param>
    /// <param name="rotation">The new orientation. A zero quaternion represents identity.</param>
    /// <returns>The updated matrix value.</returns>
    public static Fixed4x4 SetRotation(this ref Fixed4x4 matrix, FixedQuaternion rotation) =>
        matrix = Fixed4x4.SetRotation(matrix, rotation);

    /// <summary>Replaces the matrix in place with a normalized basis and zero translation.</summary>
    /// <param name="matrix">The matrix whose basis rows are independently normalized.</param>
    /// <returns>The updated affine matrix value.</returns>
    /// <remarks>Zero rows remain zero. The angles between nonzero rows are preserved to fixed-point precision.</remarks>
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public static Fixed4x4 NormalizeRotationMatrix(this ref Fixed4x4 matrix) =>
        matrix = Fixed4x4.NormalizeRotationMatrix(matrix);

    /// <summary>Linearly interpolates between two matrices component-wise.</summary>
    /// <param name="matrix">The starting matrix.</param>
    /// <param name="target">The ending matrix.</param>
    /// <param name="amount">The interpolation fraction, clamped to the range zero to one.</param>
    /// <returns>The matrix interpolated entry by entry.</returns>
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public static Fixed4x4 Lerp(this Fixed4x4 matrix, Fixed4x4 target, Fixed64 amount) =>
        Fixed4x4.Lerp(matrix, target, amount);

    /// <inheritdoc cref="Fixed4x4.Transpose(Fixed4x4)" />
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public static Fixed4x4 Transpose(this Fixed4x4 matrix) => Fixed4x4.Transpose(matrix);

    /// <inheritdoc cref="Fixed4x4.TransformPoint(Fixed4x4, Vector3d)" />
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public static Vector3d TransformPoint(this Fixed4x4 matrix, Vector3d point) =>
        Fixed4x4.TransformPoint(matrix, point);

    /// <inheritdoc cref="Fixed4x4.Transform(Fixed4x4, Vector4d)" />
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public static Vector4d Transform(this Fixed4x4 matrix, Vector4d vector) =>
        Fixed4x4.Transform(matrix, vector);

    /// <inheritdoc cref="Fixed4x4.InverseTransformPoint(Fixed4x4, Vector3d)" />
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public static Vector3d InverseTransformPoint(this Fixed4x4 matrix, Vector3d point) =>
        Fixed4x4.InverseTransformPoint(matrix, point);

    #endregion

    #region Equality

    /// <summary>
    /// Compares two Fixed4x4 for approximate equality, allowing a fixed absolute difference between components.
    /// </summary>
    /// <param name="f1">The current Fixed4x4.</param>
    /// <param name="f2">The Fixed4x4 to compare against.</param>
    /// <param name="allowedDifference">The allowed absolute difference between each component.</param>
    /// <returns>True if the components are within the allowed difference, false otherwise.</returns>
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public static bool FuzzyEqualAbsolute(this Fixed4x4 f1, Fixed4x4 f2, Fixed64 allowedDifference) =>
        FuzzyEqualAbsoluteRow1(f1, f2, allowedDifference) &&
        FuzzyEqualAbsoluteRow2(f1, f2, allowedDifference) &&
        FuzzyEqualAbsoluteRow3(f1, f2, allowedDifference) &&
        FuzzyEqualAbsoluteRow4(f1, f2, allowedDifference);

    /// <summary>
    /// Compares matrix entries using an error limit relative to the first matrix.
    /// </summary>
    /// <param name="f1">The reference matrix used to scale each entry's tolerance.</param>
    /// <param name="f2">The matrix to compare against the reference.</param>
    /// <param name="percentage">The nonnegative error fraction. Null uses <see cref="Fixed64.Epsilon"/>.</param>
    /// <returns><see langword="true"/> if every absolute fixed-point difference is at most its limit.</returns>
    /// <remarks>
    /// Each limit is the absolute reference entry multiplied by the fraction. If the absolute
    /// entry is smaller than <see cref="Fixed64.Epsilon"/>, the fraction is used as an absolute limit.
    /// Swapping the operands can change the result.
    /// </remarks>
    public static bool FuzzyEqual(this Fixed4x4 f1, Fixed4x4 f2, Fixed64? percentage = null)
    {
        Fixed64 p = percentage ?? Fixed64.Epsilon;
        return FuzzyEqualRow1(f1, f2, p) &&
               FuzzyEqualRow2(f1, f2, p) &&
               FuzzyEqualRow3(f1, f2, p) &&
               FuzzyEqualRow4(f1, f2, p);
    }

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    private static bool FuzzyEqualAbsoluteRow1(Fixed4x4 f1, Fixed4x4 f2, Fixed64 allowedDifference) =>
        (f1.M11 - f2.M11).Abs() <= allowedDifference &&
        (f1.M12 - f2.M12).Abs() <= allowedDifference &&
        (f1.M13 - f2.M13).Abs() <= allowedDifference &&
        (f1.M14 - f2.M14).Abs() <= allowedDifference;

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    private static bool FuzzyEqualAbsoluteRow2(Fixed4x4 f1, Fixed4x4 f2, Fixed64 allowedDifference) =>
        (f1.M21 - f2.M21).Abs() <= allowedDifference &&
        (f1.M22 - f2.M22).Abs() <= allowedDifference &&
        (f1.M23 - f2.M23).Abs() <= allowedDifference &&
        (f1.M24 - f2.M24).Abs() <= allowedDifference;

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    private static bool FuzzyEqualAbsoluteRow3(Fixed4x4 f1, Fixed4x4 f2, Fixed64 allowedDifference) =>
        (f1.M31 - f2.M31).Abs() <= allowedDifference &&
        (f1.M32 - f2.M32).Abs() <= allowedDifference &&
        (f1.M33 - f2.M33).Abs() <= allowedDifference &&
        (f1.M34 - f2.M34).Abs() <= allowedDifference;

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    private static bool FuzzyEqualAbsoluteRow4(Fixed4x4 f1, Fixed4x4 f2, Fixed64 allowedDifference) =>
        (f1.M41 - f2.M41).Abs() <= allowedDifference &&
        (f1.M42 - f2.M42).Abs() <= allowedDifference &&
        (f1.M43 - f2.M43).Abs() <= allowedDifference &&
        (f1.M44 - f2.M44).Abs() <= allowedDifference;

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    private static bool FuzzyEqualRow1(Fixed4x4 f1, Fixed4x4 f2, Fixed64 percentage) =>
        f1.M11.FuzzyComponentEqual(f2.M11, percentage) &&
        f1.M12.FuzzyComponentEqual(f2.M12, percentage) &&
        f1.M13.FuzzyComponentEqual(f2.M13, percentage) &&
        f1.M14.FuzzyComponentEqual(f2.M14, percentage);

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    private static bool FuzzyEqualRow2(Fixed4x4 f1, Fixed4x4 f2, Fixed64 percentage) =>
        f1.M21.FuzzyComponentEqual(f2.M21, percentage) &&
        f1.M22.FuzzyComponentEqual(f2.M22, percentage) &&
        f1.M23.FuzzyComponentEqual(f2.M23, percentage) &&
        f1.M24.FuzzyComponentEqual(f2.M24, percentage);

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    private static bool FuzzyEqualRow3(Fixed4x4 f1, Fixed4x4 f2, Fixed64 percentage) =>
        f1.M31.FuzzyComponentEqual(f2.M31, percentage) &&
        f1.M32.FuzzyComponentEqual(f2.M32, percentage) &&
        f1.M33.FuzzyComponentEqual(f2.M33, percentage) &&
        f1.M34.FuzzyComponentEqual(f2.M34, percentage);

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    private static bool FuzzyEqualRow4(Fixed4x4 f1, Fixed4x4 f2, Fixed64 percentage) =>
        f1.M41.FuzzyComponentEqual(f2.M41, percentage) &&
        f1.M42.FuzzyComponentEqual(f2.M42, percentage) &&
        f1.M43.FuzzyComponentEqual(f2.M43, percentage) &&
        f1.M44.FuzzyComponentEqual(f2.M44, percentage);

    #endregion
}
