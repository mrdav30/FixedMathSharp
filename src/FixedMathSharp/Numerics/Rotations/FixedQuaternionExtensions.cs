//=======================================================================
// FixedQuaternionExtensions.cs
//=======================================================================
// MIT License, Copyright (c) 2024-present David Oravsky (mrdav30)
// See LICENSE file in the project root for full license information.
//=======================================================================

using System.Runtime.CompilerServices;

namespace FixedMathSharp;

/// <summary>
/// Provides extension methods for performing operations and comparisons on FixedQuaternion instances, including
/// approximate equality checks and angular velocity calculations.
/// </summary>
public static class FixedQuaternionExtensions
{
    #region Operations

    /// <inheritdoc cref="FixedQuaternion.ToAngularVelocity" />
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public static Vector3d ToAngularVelocity(
        this FixedQuaternion currentRotation,
        FixedQuaternion previousRotation,
        Fixed64 deltaTime) => FixedQuaternion.ToAngularVelocity(currentRotation, previousRotation, deltaTime);

    /// <summary>Interpolates two rotations with a component-wise linear blend.</summary>
    /// <param name="start">The starting quaternion.</param>
    /// <param name="end">The ending quaternion.</param>
    /// <param name="amount">The interpolation fraction, clamped to [0, 1].</param>
    /// <returns>The normalized blend, using the closer signed representation of the ending quaternion.</returns>
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public static FixedQuaternion Lerp(this FixedQuaternion start, FixedQuaternion end, Fixed64 amount) =>
        FixedQuaternion.Lerp(start, end, amount);

    /// <summary>Interpolates two unit quaternions along the shorter rotation arc.</summary>
    /// <param name="start">The starting quaternion.</param>
    /// <param name="end">The ending quaternion.</param>
    /// <param name="amount">The interpolation fraction, clamped to [0, 1].</param>
    /// <returns>The interpolated quaternion. Nearly coincident inputs use a linear blend and the result is not renormalized.</returns>
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public static FixedQuaternion Slerp(this FixedQuaternion start, FixedQuaternion end, Fixed64 amount) =>
        FixedQuaternion.Slerp(start, end, amount);

    /// <summary>Computes the angle between two rotations.</summary>
    /// <param name="start">The first rotation.</param>
    /// <param name="end">The second rotation.</param>
    /// <returns>The smallest angle between the rotations, in degrees.</returns>
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public static Fixed64 Angle(this FixedQuaternion start, FixedQuaternion end) => FixedQuaternion.Angle(start, end);

    /// <summary>Computes the quaternion dot product.</summary>
    /// <param name="lhs">The first quaternion.</param>
    /// <param name="rhs">The second quaternion.</param>
    /// <returns>The sum of the four component products using rounded, saturating fixed-point operations.</returns>
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

    /// <summary>Compares quaternion components within a relative tolerance.</summary>
    /// <param name="q1">The reference quaternion.</param>
    /// <param name="q2">The quaternion to compare.</param>
    /// <param name="percentage">The allowed relative error as a nonnegative fraction. Null selects <see cref="Fixed64.Epsilon"/>.</param>
    /// <returns><see langword="true"/> if all corresponding components agree within the tolerance, otherwise <see langword="false"/>.</returns>
    /// <remarks>A reference component with magnitude below <see cref="Fixed64.Epsilon"/> uses the fraction directly as an absolute tolerance. Opposite signs can encode the same rotation with different components.</remarks>
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
