//=======================================================================
// FixedQuaternion.Operators.cs
//=======================================================================
// MIT License, Copyright (c) 2024-present David Oravsky (mrdav30)
// See LICENSE file in the project root for full license information.
//=======================================================================

using System.Runtime.CompilerServices;

namespace FixedMathSharp;

/// <content>
/// Operator overloads for <see cref="FixedQuaternion"/>, including multiplication, scaling, addition,
/// subtraction, negation, and equality comparisons.
/// </content>
public partial struct FixedQuaternion
{
    #region Operators

    /// <summary>
    /// Multiplies two quaternions, combining their rotations into a single quaternion.
    /// </summary>
    /// <remarks>
    /// Quaternion multiplication is not commutative; the order of operands affects the result. 
    /// This operation is commonly used to concatenate rotations.
    /// </remarks>
    /// <param name="a">The first quaternion to multiply.</param>
    /// <param name="b">The second quaternion to multiply.</param>
    /// <returns>A new FixedQuaternion representing the combined rotation of the two input quaternions.</returns>
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public static FixedQuaternion operator *(FixedQuaternion a, FixedQuaternion b) =>
        new((a.W * b.X) + (a.X * b.W) + (a.Y * b.Z) - (a.Z * b.Y),
            (a.W * b.Y) - (a.X * b.Z) + (a.Y * b.W) + (a.Z * b.X),
            (a.W * b.Z) + (a.X * b.Y) - (a.Y * b.X) + (a.Z * b.W),
            (a.W * b.W) - (a.X * b.X) - (a.Y * b.Y) - (a.Z * b.Z));

    /// <summary>
    /// Multiplies each component of the specified quaternion by the given scalar value.
    /// </summary>
    /// <param name="q">The quaternion whose components are to be multiplied.</param>
    /// <param name="scalar">The scalar value by which to multiply each component of the quaternion.</param>
    /// <returns>A new FixedQuaternion whose components are the result of multiplying the corresponding components of the input
    /// quaternion by the scalar value.</returns>
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public static FixedQuaternion operator *(FixedQuaternion q, Fixed64 scalar) =>
        new(q.X * scalar, q.Y * scalar, q.Z * scalar, q.W * scalar);

    /// <inheritdoc cref="operator *(FixedQuaternion, Fixed64)"/>
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public static FixedQuaternion operator *(Fixed64 scalar, FixedQuaternion q) =>
        new(q.X * scalar, q.Y * scalar, q.Z * scalar, q.W * scalar);

    /// <summary>
    /// Divides each component of the specified quaternion by the given scalar value.
    /// </summary>
    /// <param name="q">The quaternion whose components are to be divided.</param>
    /// <param name="scalar">The scalar value by which to divide each component of the quaternion.</param>
    /// <returns>The component-wise quotients, individually rounded and saturated.</returns>
    /// <exception cref="System.DivideByZeroException"><paramref name="scalar"/> is zero.</exception>
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public static FixedQuaternion operator /(FixedQuaternion q, Fixed64 scalar) =>
        new(q.X / scalar, q.Y / scalar, q.Z / scalar, q.W / scalar);

    /// <summary>
    /// Adds two quaternions component-wise and returns the resulting quaternion.
    /// </summary>
    /// <remarks>
    /// This operation performs a simple component-wise addition.
    /// </remarks>
    /// <param name="q1">The first quaternion to add.</param>
    /// <param name="q2">The second quaternion to add.</param>
    /// <returns>A new FixedQuaternion whose components are the sums of the corresponding components of q1 and q2.</returns>
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public static FixedQuaternion operator +(FixedQuaternion q1, FixedQuaternion q2) =>
        new(q1.X + q2.X, q1.Y + q2.Y, q1.Z + q2.Z, q1.W + q2.W);

    /// <summary>
    /// Subtracts two quaternions component-wise and returns the resulting quaternion.
    /// </summary>
    /// <param name="q1">The quaternion to subtract from.</param>
    /// <param name="q2">The quaternion to subtract.</param>
    /// <returns>The component-wise difference with each result saturated at the representable endpoints.</returns>
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public static FixedQuaternion operator -(FixedQuaternion q1, FixedQuaternion q2) =>
        new(q1.X - q2.X, q1.Y - q2.Y, q1.Z - q2.Z, q1.W - q2.W);

    /// <summary>
    /// Negates each component of the specified quaternion.
    /// </summary>
    /// <param name="q">The quaternion to negate.</param>
    /// <returns>The component-wise negation with each result saturated at the representable endpoints.</returns>
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public static FixedQuaternion operator -(FixedQuaternion q) =>
        new(-q.X, -q.Y, -q.Z, -q.W);

    /// <summary>
    /// Determines whether two FixedQuaternion instances are equal.
    /// </summary>
    /// <param name="left">The left operand.</param>
    /// <param name="right">The right operand.</param>
    /// <returns><see langword="true"/> if all corresponding components are exactly equal, otherwise <see langword="false"/>.</returns>
    /// <remarks>Opposite signs can encode the same rotation with different components.</remarks>
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public static bool operator ==(FixedQuaternion left, FixedQuaternion right) => left.Equals(right);

    /// <summary>
    /// Determines whether two FixedQuaternion instances are not equal.
    /// </summary>
    /// <param name="left">The left operand.</param>
    /// <param name="right">The right operand.</param>
    /// <returns><see langword="true"/> if at least one corresponding component differs, otherwise <see langword="false"/>.</returns>
    /// <remarks>Opposite signs can encode the same rotation with different components.</remarks>
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public static bool operator !=(FixedQuaternion left, FixedQuaternion right) => !left.Equals(right);

    #endregion
}
