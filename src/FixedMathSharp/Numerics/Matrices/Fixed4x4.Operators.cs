//=======================================================================
// Fixed4x4.Operators.cs
//=======================================================================
// MIT License, Copyright (c) 2024–present David Oravsky (mrdav30)
// See LICENSE file in the project root for full license information.
//=======================================================================

using System.Runtime.CompilerServices;

namespace FixedMathSharp;

/// <content>
/// Operator overloads for <see cref="Fixed4x4"/>, including negation, addition,
/// subtraction, and multiplication.
/// </content>
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
    /// Attempts to multiply two matrices with one exact, round-half-to-even conversion per cell.
    /// </summary>
    /// <remarks>
    /// Unlike <see cref="operator *(Fixed4x4, Fixed4x4)"/>, this method does not saturate
    /// intermediate products or partial sums. On failure, <paramref name="result"/> is zero.
    /// </remarks>
    public static bool TryMultiply(Fixed4x4 lhs, Fixed4x4 rhs, out Fixed4x4 result)
    {
        bool representable = TryGetExactProductSum(lhs.M11, rhs.M11, lhs.M12, rhs.M21, lhs.M13, rhs.M31, lhs.M14, rhs.M41, out Fixed64 m11)
            & TryGetExactProductSum(lhs.M11, rhs.M12, lhs.M12, rhs.M22, lhs.M13, rhs.M32, lhs.M14, rhs.M42, out Fixed64 m12)
            & TryGetExactProductSum(lhs.M11, rhs.M13, lhs.M12, rhs.M23, lhs.M13, rhs.M33, lhs.M14, rhs.M43, out Fixed64 m13)
            & TryGetExactProductSum(lhs.M11, rhs.M14, lhs.M12, rhs.M24, lhs.M13, rhs.M34, lhs.M14, rhs.M44, out Fixed64 m14)
            & TryGetExactProductSum(lhs.M21, rhs.M11, lhs.M22, rhs.M21, lhs.M23, rhs.M31, lhs.M24, rhs.M41, out Fixed64 m21)
            & TryGetExactProductSum(lhs.M21, rhs.M12, lhs.M22, rhs.M22, lhs.M23, rhs.M32, lhs.M24, rhs.M42, out Fixed64 m22)
            & TryGetExactProductSum(lhs.M21, rhs.M13, lhs.M22, rhs.M23, lhs.M23, rhs.M33, lhs.M24, rhs.M43, out Fixed64 m23)
            & TryGetExactProductSum(lhs.M21, rhs.M14, lhs.M22, rhs.M24, lhs.M23, rhs.M34, lhs.M24, rhs.M44, out Fixed64 m24)
            & TryGetExactProductSum(lhs.M31, rhs.M11, lhs.M32, rhs.M21, lhs.M33, rhs.M31, lhs.M34, rhs.M41, out Fixed64 m31)
            & TryGetExactProductSum(lhs.M31, rhs.M12, lhs.M32, rhs.M22, lhs.M33, rhs.M32, lhs.M34, rhs.M42, out Fixed64 m32)
            & TryGetExactProductSum(lhs.M31, rhs.M13, lhs.M32, rhs.M23, lhs.M33, rhs.M33, lhs.M34, rhs.M43, out Fixed64 m33)
            & TryGetExactProductSum(lhs.M31, rhs.M14, lhs.M32, rhs.M24, lhs.M33, rhs.M34, lhs.M34, rhs.M44, out Fixed64 m34)
            & TryGetExactProductSum(lhs.M41, rhs.M11, lhs.M42, rhs.M21, lhs.M43, rhs.M31, lhs.M44, rhs.M41, out Fixed64 m41)
            & TryGetExactProductSum(lhs.M41, rhs.M12, lhs.M42, rhs.M22, lhs.M43, rhs.M32, lhs.M44, rhs.M42, out Fixed64 m42)
            & TryGetExactProductSum(lhs.M41, rhs.M13, lhs.M42, rhs.M23, lhs.M43, rhs.M33, lhs.M44, rhs.M43, out Fixed64 m43)
            & TryGetExactProductSum(lhs.M41, rhs.M14, lhs.M42, rhs.M24, lhs.M43, rhs.M34, lhs.M44, rhs.M44, out Fixed64 m44);
        if (!representable)
        {
            result = Zero;
            return false;
        }

        result = new Fixed4x4(
            m11, m12, m13, m14,
            m21, m22, m23, m24,
            m31, m32, m33, m34,
            m41, m42, m43, m44);
        return true;
    }

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

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    private static bool TryGetExactProductSum(
        Fixed64 firstLeft,
        Fixed64 firstRight,
        Fixed64 secondLeft,
        Fixed64 secondRight,
        Fixed64 thirdLeft,
        Fixed64 thirdRight,
        Fixed64 fourthLeft,
        Fixed64 fourthRight,
        out Fixed64 result)
    {
        Signed192 sum = AddProduct(default, firstLeft, firstRight);
        sum = AddProduct(sum, secondLeft, secondRight);
        sum = AddProduct(sum, thirdLeft, thirdRight);
        sum = AddProduct(sum, fourthLeft, fourthRight);
        bool negative = sum.Sign < 0;
        WideArithmetic.GetMagnitude(sum, out ulong high, out ulong middle, out ulong low);
        if (high != 0UL || (middle >> FixedMath.SHIFT_AMOUNT_I) != 0UL)
        {
            result = default;
            return false;
        }

        ulong magnitude = (middle << FixedMath.SHIFT_AMOUNT_I)
            | (low >> FixedMath.SHIFT_AMOUNT_I);
        ulong guardMask = 1UL << (FixedMath.SHIFT_AMOUNT_I - 1);
        if ((low & guardMask) != 0UL
            && ((low & (guardMask - 1UL)) != 0UL || (magnitude & 1UL) != 0UL))
        {
            if (magnitude == ulong.MaxValue)
            {
                result = default;
                return false;
            }

            magnitude++;
        }

        ulong limit = negative ? 1UL << 63 : (ulong)long.MaxValue;
        if (magnitude > limit)
        {
            result = default;
            return false;
        }

        result = Fixed64.FromRaw(negative
            ? magnitude == 1UL << 63 ? long.MinValue : -(long)magnitude
            : (long)magnitude);
        return true;

        static Signed192 AddProduct(Signed192 sum, Fixed64 left, Fixed64 right)
        {
            long leftRaw = left.m_rawValue;
            long rightRaw = right.m_rawValue;
            Fixed64.Multiply64To128(
                Fixed64.AbsToUInt64(leftRaw),
                Fixed64.AbsToUInt64(rightRaw),
                out ulong productHigh,
                out ulong productLow);
            Signed192 product = new(0UL, productHigh, productLow);
            if ((leftRaw ^ rightRaw) < 0L)
                product = WideArithmetic.SubtractSigned192(default, product);

            return WideArithmetic.AddSigned192(sum, product);
        }
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
