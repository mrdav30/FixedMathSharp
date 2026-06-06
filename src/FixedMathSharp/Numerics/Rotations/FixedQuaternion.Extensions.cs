//=======================================================================
// FixedQuaternion.Extensions.cs
//=======================================================================
// MIT License, Copyright (c) 2024–present David Oravsky (mrdav30)
// See LICENSE file in the project root for full license information.
//=======================================================================

using System.Runtime.CompilerServices;

namespace FixedMathSharp;

/// <summary>
/// Provides extension methods for performing operations and comparisons on FixedQuaternion instances, including
/// approximate equality checks and angular velocity calculations.
/// </summary>
public static partial class FixedQuaternionExtensions
{
    #region Operations

    /// <inheritdoc cref="FixedQuaternion.ToAngularVelocity" />
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public static Vector3d ToAngularVelocity(
        this FixedQuaternion currentRotation,
        FixedQuaternion previousRotation,
        Fixed64 deltaTime) => FixedQuaternion.ToAngularVelocity(currentRotation, previousRotation, deltaTime);

    /// <inheritdoc cref="FixedQuaternion.Lerp(FixedQuaternion, FixedQuaternion, Fixed64)" />
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public static FixedQuaternion Lerp(this FixedQuaternion start, FixedQuaternion end, Fixed64 amount) =>
        FixedQuaternion.Lerp(start, end, amount);

    /// <inheritdoc cref="FixedQuaternion.Slerp(FixedQuaternion, FixedQuaternion, Fixed64)" />
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public static FixedQuaternion Slerp(this FixedQuaternion start, FixedQuaternion end, Fixed64 amount) =>
        FixedQuaternion.Slerp(start, end, amount);

    /// <inheritdoc cref="FixedQuaternion.Angle(FixedQuaternion, FixedQuaternion)" />
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public static Fixed64 Angle(this FixedQuaternion start, FixedQuaternion end) => FixedQuaternion.Angle(start, end);

    /// <inheritdoc cref="FixedQuaternion.Dot(FixedQuaternion, FixedQuaternion)" />
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public static Fixed64 Dot(this FixedQuaternion lhs, FixedQuaternion rhs) => FixedQuaternion.Dot(lhs, rhs);

    /// <inheritdoc cref="FixedQuaternion.QuaternionLog(FixedQuaternion)" />
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public static Vector3d QuaternionLog(this FixedQuaternion q) => FixedQuaternion.QuaternionLog(q);

    #endregion

    #region Equality

    /// <summary>
    /// Compares two quaternions for approximate equality, allowing a fixed absolute difference between components.
    /// </summary>
    /// <param name="q1">The current quaternion.</param>
    /// <param name="q2">The quaternion to compare against.</param>
    /// <param name="allowedDifference">The allowed absolute difference between each component.</param>
    /// <returns>True if the components are within the allowed difference, false otherwise.</returns>
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public static bool FuzzyEqualAbsolute(this FixedQuaternion q1, FixedQuaternion q2, Fixed64 allowedDifference) =>
        (q1.X - q2.X).Abs() <= allowedDifference &&
        (q1.Y - q2.Y).Abs() <= allowedDifference &&
        (q1.Z - q2.Z).Abs() <= allowedDifference &&
        (q1.W - q2.W).Abs() <= allowedDifference;

    /// <summary>
    /// Compares two quaternions for approximate equality, allowing a fractional percentage (defaults to ~1%) difference between components.
    /// </summary>
    /// <param name="q1">The current quaternion.</param>
    /// <param name="q2">The quaternion to compare against.</param>
    /// <param name="percentage">The allowed fractional difference (percentage) for each component.</param>
    /// <returns>True if the components are within the allowed percentage difference, false otherwise.</returns>
    public static bool FuzzyEqual(this FixedQuaternion q1, FixedQuaternion q2, Fixed64? percentage = null)
    {
        Fixed64 p = percentage ?? Fixed64.Epsilon;
        return q1.X.FuzzyComponentEqual(q2.X, p) &&
               q1.Y.FuzzyComponentEqual(q2.Y, p) &&
               q1.Z.FuzzyComponentEqual(q2.Z, p) &&
               q1.W.FuzzyComponentEqual(q2.W, p);
    }

    #endregion
}
