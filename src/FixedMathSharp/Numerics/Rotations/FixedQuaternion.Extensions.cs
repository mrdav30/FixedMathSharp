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
    /// <inheritdoc cref="FixedQuaternion.ToAngularVelocity" />
    public static Vector3d ToAngularVelocity(
        this FixedQuaternion currentRotation,
        FixedQuaternion previousRotation,
        Fixed64 deltaTime) => FixedQuaternion.ToAngularVelocity(currentRotation, previousRotation, deltaTime);

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