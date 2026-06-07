//=======================================================================
// Fixed4x4.Operators.cs
//=======================================================================
// MIT License, Copyright (c) 2024–present David Oravsky (mrdav30)
// See LICENSE file in the project root for full license information.
//=======================================================================

using System.Runtime.CompilerServices;

namespace FixedMathSharp;

public partial struct Fixed4x4
{
    #region Operators

    /// <summary>
    /// Negates the specified matrix by multiplying all its values by -1.
    /// </summary>
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public static Fixed4x4 operator -(Fixed4x4 value)
    {
        Fixed4x4 result = default;
        result.M11 = -value.M11;
        result.M12 = -value.M12;
        result.M13 = -value.M13;
        result.M14 = -value.M14;
        result.M21 = -value.M21;
        result.M22 = -value.M22;
        result.M23 = -value.M23;
        result.M24 = -value.M24;
        result.M31 = -value.M31;
        result.M32 = -value.M32;
        result.M33 = -value.M33;
        result.M34 = -value.M34;
        result.M41 = -value.M41;
        result.M42 = -value.M42;
        result.M43 = -value.M43;
        result.M44 = -value.M44;
        return result;
    }

    /// <summary>
    /// Adds two matrices element-wise.
    /// </summary>
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public static Fixed4x4 operator +(Fixed4x4 lhs, Fixed4x4 rhs) =>
        new(
            lhs.M11 + rhs.M11, lhs.M12 + rhs.M12, lhs.M13 + rhs.M13, lhs.M14 + rhs.M14,
            lhs.M21 + rhs.M21, lhs.M22 + rhs.M22, lhs.M23 + rhs.M23, lhs.M24 + rhs.M24,
            lhs.M31 + rhs.M31, lhs.M32 + rhs.M32, lhs.M33 + rhs.M33, lhs.M34 + rhs.M34,
            lhs.M41 + rhs.M41, lhs.M42 + rhs.M42, lhs.M43 + rhs.M43, lhs.M44 + rhs.M44);

    /// <summary>
    /// Subtracts two matrices element-wise.
    /// </summary>
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public static Fixed4x4 operator -(Fixed4x4 lhs, Fixed4x4 rhs) =>
        new(
            lhs.M11 - rhs.M11, lhs.M12 - rhs.M12, lhs.M13 - rhs.M13, lhs.M14 - rhs.M14,
            lhs.M21 - rhs.M21, lhs.M22 - rhs.M22, lhs.M23 - rhs.M23, lhs.M24 - rhs.M24,
            lhs.M31 - rhs.M31, lhs.M32 - rhs.M32, lhs.M33 - rhs.M33, lhs.M34 - rhs.M34,
            lhs.M41 - rhs.M41, lhs.M42 - rhs.M42, lhs.M43 - rhs.M43, lhs.M44 - rhs.M44);

    /// <summary>
    /// Multiplies two 4x4 matrices using standard matrix multiplication.
    /// </summary>
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public static Fixed4x4 operator *(Fixed4x4 lhs, Fixed4x4 rhs)
    {
        if (lhs.IsAffine && rhs.IsAffine)
        {
            // Optimized affine multiplication (skips full 4×4 multiplication)
            return new Fixed4x4(
                lhs.M11 * rhs.M11 + lhs.M12 * rhs.M21 + lhs.M13 * rhs.M31,
                lhs.M11 * rhs.M12 + lhs.M12 * rhs.M22 + lhs.M13 * rhs.M32,
                lhs.M11 * rhs.M13 + lhs.M12 * rhs.M23 + lhs.M13 * rhs.M33,
                Fixed64.Zero,

                lhs.M21 * rhs.M11 + lhs.M22 * rhs.M21 + lhs.M23 * rhs.M31,
                lhs.M21 * rhs.M12 + lhs.M22 * rhs.M22 + lhs.M23 * rhs.M32,
                lhs.M21 * rhs.M13 + lhs.M22 * rhs.M23 + lhs.M23 * rhs.M33,
                Fixed64.Zero,

                lhs.M31 * rhs.M11 + lhs.M32 * rhs.M21 + lhs.M33 * rhs.M31,
                lhs.M31 * rhs.M12 + lhs.M32 * rhs.M22 + lhs.M33 * rhs.M32,
                lhs.M31 * rhs.M13 + lhs.M32 * rhs.M23 + lhs.M33 * rhs.M33,
                Fixed64.Zero,

                lhs.M41 * rhs.M11 + lhs.M42 * rhs.M21 + lhs.M43 * rhs.M31 + rhs.M41,
                lhs.M41 * rhs.M12 + lhs.M42 * rhs.M22 + lhs.M43 * rhs.M32 + rhs.M42,
                lhs.M41 * rhs.M13 + lhs.M42 * rhs.M23 + lhs.M43 * rhs.M33 + rhs.M43,
                Fixed64.One
            );
        }

        // Full 4×4 multiplication (fallback for perspective matrices)
        return new Fixed4x4(
            // Upper-left 3×3 matrix multiplication (rotation & scale)
            lhs.M11 * rhs.M11 + lhs.M12 * rhs.M21 + lhs.M13 * rhs.M31 + lhs.M14 * rhs.M41,
            lhs.M11 * rhs.M12 + lhs.M12 * rhs.M22 + lhs.M13 * rhs.M32 + lhs.M14 * rhs.M42,
            lhs.M11 * rhs.M13 + lhs.M12 * rhs.M23 + lhs.M13 * rhs.M33 + lhs.M14 * rhs.M43,
            lhs.M11 * rhs.M14 + lhs.M12 * rhs.M24 + lhs.M13 * rhs.M34 + lhs.M14 * rhs.M44,

            lhs.M21 * rhs.M11 + lhs.M22 * rhs.M21 + lhs.M23 * rhs.M31 + lhs.M24 * rhs.M41,
            lhs.M21 * rhs.M12 + lhs.M22 * rhs.M22 + lhs.M23 * rhs.M32 + lhs.M24 * rhs.M42,
            lhs.M21 * rhs.M13 + lhs.M22 * rhs.M23 + lhs.M23 * rhs.M33 + lhs.M24 * rhs.M43,
            lhs.M21 * rhs.M14 + lhs.M22 * rhs.M24 + lhs.M23 * rhs.M34 + lhs.M24 * rhs.M44,

            lhs.M31 * rhs.M11 + lhs.M32 * rhs.M21 + lhs.M33 * rhs.M31 + lhs.M34 * rhs.M41,
            lhs.M31 * rhs.M12 + lhs.M32 * rhs.M22 + lhs.M33 * rhs.M32 + lhs.M34 * rhs.M42,
            lhs.M31 * rhs.M13 + lhs.M32 * rhs.M23 + lhs.M33 * rhs.M33 + lhs.M34 * rhs.M43,
            lhs.M31 * rhs.M14 + lhs.M32 * rhs.M24 + lhs.M33 * rhs.M34 + lhs.M34 * rhs.M44,

            // Compute new translation
            lhs.M41 * rhs.M11 + lhs.M42 * rhs.M21 + lhs.M43 * rhs.M31 + lhs.M44 * rhs.M41,
            lhs.M41 * rhs.M12 + lhs.M42 * rhs.M22 + lhs.M43 * rhs.M32 + lhs.M44 * rhs.M42,
            lhs.M41 * rhs.M13 + lhs.M42 * rhs.M23 + lhs.M43 * rhs.M33 + lhs.M44 * rhs.M43,
            lhs.M41 * rhs.M14 + lhs.M42 * rhs.M24 + lhs.M43 * rhs.M34 + lhs.M44 * rhs.M44
        );
    }

    /// <summary>
    /// Multiplies every matrix component by a scalar.
    /// </summary>
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public static Fixed4x4 operator *(Fixed4x4 matrix, Fixed64 scalar) =>
        new(
            matrix.M11 * scalar, matrix.M12 * scalar, matrix.M13 * scalar, matrix.M14 * scalar,
            matrix.M21 * scalar, matrix.M22 * scalar, matrix.M23 * scalar, matrix.M24 * scalar,
            matrix.M31 * scalar, matrix.M32 * scalar, matrix.M33 * scalar, matrix.M34 * scalar,
            matrix.M41 * scalar, matrix.M42 * scalar, matrix.M43 * scalar, matrix.M44 * scalar);

    /// <summary>
    /// Multiplies every matrix component by a scalar.
    /// </summary>
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public static Fixed4x4 operator *(Fixed64 scalar, Fixed4x4 matrix) => matrix * scalar;

    /// <summary>
    /// Divides every matrix component by a scalar.
    /// </summary>
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public static Fixed4x4 operator /(Fixed4x4 matrix, Fixed64 scalar)
    {
        Fixed64 inverse = Fixed64.One / scalar;
        return matrix * inverse;
    }

    /// <summary>
    /// Determines whether two Fixed4x4 instances are equal.
    /// </summary>
    /// <param name="left">The first Fixed4x4 instance to compare.</param>
    /// <param name="right">The second Fixed4x4 instance to compare.</param>
    /// <returns>true if the specified Fixed4x4 instances are equal; otherwise, false.</returns>
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public static bool operator ==(Fixed4x4 left, Fixed4x4 right) => left.Equals(right);

    /// <summary>
    /// Determines whether two Fixed4x4 instances are not equal.
    /// </summary>
    /// <param name="left">The first Fixed4x4 instance to compare.</param>
    /// <param name="right">The second Fixed4x4 instance to compare.</param>
    /// <returns>true if the specified Fixed4x4 instances are not equal; otherwise, false.</returns>
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public static bool operator !=(Fixed4x4 left, Fixed4x4 right) => !(left == right);

    #endregion
}
