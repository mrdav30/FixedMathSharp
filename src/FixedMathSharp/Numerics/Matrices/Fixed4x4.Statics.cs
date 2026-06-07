//=======================================================================
// Fixed4x4.Statics.cs
//=======================================================================
// MIT License, Copyright (c) 2024–present David Oravsky (mrdav30)
// See LICENSE file in the project root for full license information.
//=======================================================================

using System;
using System.Runtime.CompilerServices;

namespace FixedMathSharp;

public partial struct Fixed4x4
{
    #region Static Matrix Operators

    /// <summary>
    /// Linearly interpolates between two matrices component-wise.
    /// </summary>
    public static Fixed4x4 Lerp(Fixed4x4 a, Fixed4x4 b, Fixed64 t) =>
         new(
            FixedMath.Lerp(a.M11, b.M11, t),
            FixedMath.Lerp(a.M12, b.M12, t),
            FixedMath.Lerp(a.M13, b.M13, t),
            FixedMath.Lerp(a.M14, b.M14, t),
            FixedMath.Lerp(a.M21, b.M21, t),
            FixedMath.Lerp(a.M22, b.M22, t),
            FixedMath.Lerp(a.M23, b.M23, t),
            FixedMath.Lerp(a.M24, b.M24, t),
            FixedMath.Lerp(a.M31, b.M31, t),
            FixedMath.Lerp(a.M32, b.M32, t),
            FixedMath.Lerp(a.M33, b.M33, t),
            FixedMath.Lerp(a.M34, b.M34, t),
            FixedMath.Lerp(a.M41, b.M41, t),
            FixedMath.Lerp(a.M42, b.M42, t),
            FixedMath.Lerp(a.M43, b.M43, t),
            FixedMath.Lerp(a.M44, b.M44, t));

    /// <summary>
    /// Transposes the matrix by swapping rows and columns.
    /// </summary>
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public static Fixed4x4 Transpose(Fixed4x4 matrix) =>
         new(
            matrix.M11, matrix.M21, matrix.M31, matrix.M41,
            matrix.M12, matrix.M22, matrix.M32, matrix.M42,
            matrix.M13, matrix.M23, matrix.M33, matrix.M43,
            matrix.M14, matrix.M24, matrix.M34, matrix.M44);

    /// <summary>
    /// Divides each component of one matrix by the corresponding component of another matrix.
    /// </summary>
    /// <exception cref="DivideByZeroException">
    /// Thrown when any divisor component is zero.
    /// </exception>
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public static Fixed4x4 ComponentDivide(Fixed4x4 dividend, Fixed4x4 divisor) =>
        new(
            dividend.M11 / divisor.M11,
            dividend.M12 / divisor.M12,
            dividend.M13 / divisor.M13,
            dividend.M14 / divisor.M14,
            dividend.M21 / divisor.M21,
            dividend.M22 / divisor.M22,
            dividend.M23 / divisor.M23,
            dividend.M24 / divisor.M24,
            dividend.M31 / divisor.M31,
            dividend.M32 / divisor.M32,
            dividend.M33 / divisor.M33,
            dividend.M34 / divisor.M34,
            dividend.M41 / divisor.M41,
            dividend.M42 / divisor.M42,
            dividend.M43 / divisor.M43,
            dividend.M44 / divisor.M44);

    /// <summary>
    /// Divides one matrix by another using inverse matrix division.
    /// </summary>
    /// <remarks>
    /// This is equivalent to <c>dividend * Invert(divisor)</c>. Use <see cref="ComponentDivide"/>
    /// when each matrix component should be divided by the corresponding component of another matrix.
    /// </remarks>
    /// <exception cref="InvalidOperationException">
    /// Thrown when <paramref name="divisor"/> is not invertible.
    /// </exception>
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public static Fixed4x4 InverseDivide(Fixed4x4 dividend, Fixed4x4 divisor)
    {
        if (!Invert(divisor, out Fixed4x4 inverseDivisor))
            throw new InvalidOperationException("Matrix divisor is not invertible.");

        return dividend * inverseDivisor;
    }

    /// <summary>
    /// Inverts the matrix if it is invertible (i.e., if the determinant is not zero).
    /// </summary>
    /// <remarks>
    /// To Invert a FixedMatrix4x4, we need to calculate the inverse for each element. 
    /// This involves computing the cofactor for each element, 
    /// which is the determinant of the submatrix when the row and column of that element are removed, 
    /// multiplied by a sign based on the element's position. 
    /// After computing all cofactors, the result is transposed to get the inverse matrix.
    /// </remarks>
    public static bool Invert(Fixed4x4 matrix, out Fixed4x4 result)
    {
        if (!matrix.IsAffine)
            return FullInvert(matrix, out result);

        Fixed64 det = matrix.GetDeterminant();

        if (det == Fixed64.Zero)
        {
            result = Identity;
            return false;
        }

        Fixed64 invDet = Fixed64.One / det;

        // Invert the 3×3 upper-left rotation/scale matrix
        result = new Fixed4x4(
            (matrix.M22 * matrix.M33 - matrix.M23 * matrix.M32) * invDet,
            (matrix.M13 * matrix.M32 - matrix.M12 * matrix.M33) * invDet,
            (matrix.M12 * matrix.M23 - matrix.M13 * matrix.M22) * invDet, Fixed64.Zero,

            (matrix.M23 * matrix.M31 - matrix.M21 * matrix.M33) * invDet,
            (matrix.M11 * matrix.M33 - matrix.M13 * matrix.M31) * invDet,
            (matrix.M13 * matrix.M21 - matrix.M11 * matrix.M23) * invDet, Fixed64.Zero,

            (matrix.M21 * matrix.M32 - matrix.M22 * matrix.M31) * invDet,
            (matrix.M12 * matrix.M31 - matrix.M11 * matrix.M32) * invDet,
            (matrix.M11 * matrix.M22 - matrix.M12 * matrix.M21) * invDet, Fixed64.Zero,

            Fixed64.Zero, Fixed64.Zero, Fixed64.Zero, Fixed64.One  // Ensure homogeneous coordinate stays valid
        );

        result.M41 = -(matrix.M41 * result.M11 + matrix.M42 * result.M21 + matrix.M43 * result.M31);
        result.M42 = -(matrix.M41 * result.M12 + matrix.M42 * result.M22 + matrix.M43 * result.M32);
        result.M43 = -(matrix.M41 * result.M13 + matrix.M42 * result.M23 + matrix.M43 * result.M33);
        result.M44 = Fixed64.One;

        return true;
    }

    private static bool FullInvert(Fixed4x4 matrix, out Fixed4x4 result)
    {
        Fixed64 det = matrix.GetDeterminant();

        if (det == Fixed64.Zero)
        {
            result = Fixed4x4.Identity;
            return false;
        }

        Fixed64 invDet = Fixed64.One / det;

        // Inversion using cofactors and determinants of 3x3 submatrices
        result = new Fixed4x4
        {
            // First row
            M11 = invDet * ((matrix.M22 * matrix.M33 * matrix.M44 + matrix.M23 * matrix.M34 * matrix.M42 + matrix.M24 * matrix.M32 * matrix.M43)
                          - (matrix.M24 * matrix.M33 * matrix.M42 + matrix.M22 * matrix.M34 * matrix.M43 + matrix.M23 * matrix.M32 * matrix.M44)),
            M12 = invDet * ((matrix.M12 * matrix.M34 * matrix.M43 + matrix.M13 * matrix.M32 * matrix.M44 + matrix.M14 * matrix.M33 * matrix.M42)
                          - (matrix.M14 * matrix.M32 * matrix.M43 + matrix.M12 * matrix.M33 * matrix.M44 + matrix.M13 * matrix.M34 * matrix.M42)),
            M13 = invDet * ((matrix.M12 * matrix.M23 * matrix.M44 + matrix.M13 * matrix.M24 * matrix.M42 + matrix.M14 * matrix.M22 * matrix.M43)
                          - (matrix.M14 * matrix.M23 * matrix.M42 + matrix.M12 * matrix.M24 * matrix.M43 + matrix.M13 * matrix.M22 * matrix.M44)),
            M14 = invDet * ((matrix.M12 * matrix.M24 * matrix.M33 + matrix.M13 * matrix.M22 * matrix.M34 + matrix.M14 * matrix.M23 * matrix.M32)
                          - (matrix.M14 * matrix.M22 * matrix.M33 + matrix.M12 * matrix.M23 * matrix.M34 + matrix.M13 * matrix.M24 * matrix.M32)),

            // Second row
            M21 = invDet * ((matrix.M21 * matrix.M34 * matrix.M43 + matrix.M23 * matrix.M31 * matrix.M44 + matrix.M24 * matrix.M33 * matrix.M41)
                          - (matrix.M24 * matrix.M31 * matrix.M43 + matrix.M21 * matrix.M33 * matrix.M44 + matrix.M23 * matrix.M34 * matrix.M41)),
            M22 = invDet * ((matrix.M11 * matrix.M33 * matrix.M44 + matrix.M13 * matrix.M34 * matrix.M41 + matrix.M14 * matrix.M31 * matrix.M43)
                          - (matrix.M14 * matrix.M31 * matrix.M43 + matrix.M11 * matrix.M34 * matrix.M43 + matrix.M13 * matrix.M31 * matrix.M44)),
            M23 = invDet * ((matrix.M11 * matrix.M24 * matrix.M43 + matrix.M13 * matrix.M21 * matrix.M44 + matrix.M14 * matrix.M23 * matrix.M41)
                          - (matrix.M14 * matrix.M21 * matrix.M43 + matrix.M11 * matrix.M23 * matrix.M44 + matrix.M13 * matrix.M24 * matrix.M41)),
            M24 = invDet * ((matrix.M11 * matrix.M23 * matrix.M34 + matrix.M13 * matrix.M24 * matrix.M31 + matrix.M14 * matrix.M21 * matrix.M33)
                          - (matrix.M14 * matrix.M21 * matrix.M33 + matrix.M11 * matrix.M24 * matrix.M33 + matrix.M13 * matrix.M23 * matrix.M31)),

            // Third row
            M31 = invDet * ((matrix.M21 * matrix.M32 * matrix.M44 + matrix.M22 * matrix.M34 * matrix.M41 + matrix.M24 * matrix.M31 * matrix.M42)
                          - (matrix.M24 * matrix.M31 * matrix.M42 + matrix.M21 * matrix.M34 * matrix.M42 + matrix.M22 * matrix.M31 * matrix.M44)),
            M32 = invDet * ((matrix.M11 * matrix.M34 * matrix.M42 + matrix.M12 * matrix.M31 * matrix.M44 + matrix.M14 * matrix.M32 * matrix.M41)
                          - (matrix.M14 * matrix.M31 * matrix.M42 + matrix.M11 * matrix.M32 * matrix.M44 + matrix.M12 * matrix.M34 * matrix.M41)),
            M33 = invDet * ((matrix.M11 * matrix.M22 * matrix.M44 + matrix.M12 * matrix.M24 * matrix.M41 + matrix.M14 * matrix.M21 * matrix.M42)
                          - (matrix.M14 * matrix.M21 * matrix.M42 + matrix.M11 * matrix.M24 * matrix.M42 + matrix.M12 * matrix.M21 * matrix.M44)),
            M34 = invDet * ((matrix.M11 * matrix.M24 * matrix.M32 + matrix.M12 * matrix.M21 * matrix.M34 + matrix.M14 * matrix.M22 * matrix.M31)
                          - (matrix.M14 * matrix.M21 * matrix.M32 + matrix.M11 * matrix.M22 * matrix.M34 + matrix.M12 * matrix.M24 * matrix.M31)),

            // Fourth row
            M41 = invDet * ((matrix.M21 * matrix.M33 * matrix.M42 + matrix.M22 * matrix.M31 * matrix.M43 + matrix.M23 * matrix.M32 * matrix.M41)
                          - (matrix.M23 * matrix.M31 * matrix.M42 + matrix.M21 * matrix.M32 * matrix.M43 + matrix.M22 * matrix.M33 * matrix.M41)),
            M42 = invDet * ((matrix.M11 * matrix.M32 * matrix.M43 + matrix.M12 * matrix.M33 * matrix.M41 + matrix.M13 * matrix.M31 * matrix.M42)
                          - (matrix.M13 * matrix.M31 * matrix.M42 + matrix.M11 * matrix.M33 * matrix.M42 + matrix.M12 * matrix.M31 * matrix.M43)),
            M43 = invDet * ((matrix.M11 * matrix.M23 * matrix.M42 + matrix.M12 * matrix.M21 * matrix.M43 + matrix.M13 * matrix.M22 * matrix.M41)
                          - (matrix.M13 * matrix.M21 * matrix.M42 + matrix.M11 * matrix.M22 * matrix.M43 + matrix.M12 * matrix.M23 * matrix.M41)),
            M44 = invDet * ((matrix.M11 * matrix.M22 * matrix.M33 + matrix.M12 * matrix.M23 * matrix.M31 + matrix.M13 * matrix.M21 * matrix.M32)
                          - (matrix.M13 * matrix.M21 * matrix.M32 + matrix.M11 * matrix.M23 * matrix.M32 + matrix.M12 * matrix.M22 * matrix.M31)),
        };

        return true;
    }

    /// <summary>
    /// Transforms a 4D vector by a 4x4 matrix, preserving the computed W component.
    /// </summary>
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public static Vector4d Transform(Fixed4x4 matrix, Vector4d vector) => Vector4d.Transform(matrix, vector);

    /// <summary>
    /// Transforms a point from local space to world space using this transformation matrix.
    /// </summary>
    /// <remarks>
    /// FixedMathSharp applies matrices using a row-vector convention: <c>point * matrix</c>.
    /// This is equivalent to <see cref="Vector3d.operator *(Vector3d, Fixed4x4)"/>.
    /// </remarks>
    /// <param name="matrix">The transformation matrix.</param>
    /// <param name="point">The local-space point.</param>
    /// <returns>The transformed point in world space.</returns>
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public static Vector3d TransformPoint(Fixed4x4 matrix, Vector3d point)
    {
        if (matrix.IsAffine)
            return new Vector3d(
                point.X * matrix.M11 + point.Y * matrix.M21 + point.Z * matrix.M31 + matrix.M41,
                point.X * matrix.M12 + point.Y * matrix.M22 + point.Z * matrix.M32 + matrix.M42,
                point.X * matrix.M13 + point.Y * matrix.M23 + point.Z * matrix.M33 + matrix.M43
            );

        return FullTransformPoint(matrix, point);
    }

    private static Vector3d FullTransformPoint(Fixed4x4 matrix, Vector3d point)
    {
        // Full 4×4 transformation (needed for perspective projections)
        Fixed64 w = matrix.M14 * point.X + matrix.M24 * point.Y + matrix.M34 * point.Z + matrix.M44;
        if (w == Fixed64.Zero) w = Fixed64.One;  // Prevent divide-by-zero

        return new Vector3d(
            (point.X * matrix.M11 + point.Y * matrix.M21 + point.Z * matrix.M31 + matrix.M41) / w,
            (point.X * matrix.M12 + point.Y * matrix.M22 + point.Z * matrix.M32 + matrix.M42) / w,
            (point.X * matrix.M13 + point.Y * matrix.M23 + point.Z * matrix.M33 + matrix.M43) / w
        );
    }

    /// <summary>
    /// Transforms a point from world space into the local space of the matrix.
    /// </summary>
    /// <param name="matrix">The transformation matrix.</param>
    /// <param name="point">The world-space point.</param>
    /// <returns>The local-space point relative to the transformation matrix.</returns>
    public static Vector3d InverseTransformPoint(Fixed4x4 matrix, Vector3d point)
    {
        // Invert the transformation matrix
        if (!Invert(matrix, out Fixed4x4 inverseMatrix))
            throw new InvalidOperationException("Matrix is not invertible.");

        if (inverseMatrix.IsAffine)
        {
            return new Vector3d(
                point.X * inverseMatrix.M11 + point.Y * inverseMatrix.M21 + point.Z * inverseMatrix.M31 + inverseMatrix.M41,
                point.X * inverseMatrix.M12 + point.Y * inverseMatrix.M22 + point.Z * inverseMatrix.M32 + inverseMatrix.M42,
                point.X * inverseMatrix.M13 + point.Y * inverseMatrix.M23 + point.Z * inverseMatrix.M33 + inverseMatrix.M43
            );
        }

        return FullInverseTransformPoint(inverseMatrix, point);
    }

    private static Vector3d FullInverseTransformPoint(Fixed4x4 matrix, Vector3d point)
    {
        // Full 4×4 transformation (needed for perspective projections)
        Fixed64 w = matrix.M14 * point.X + matrix.M24 * point.Y + matrix.M34 * point.Z + matrix.M44;
        if (w == Fixed64.Zero) w = Fixed64.One;  // Prevent divide-by-zero

        return new Vector3d(
            (point.X * matrix.M11 + point.Y * matrix.M21 + point.Z * matrix.M31 + matrix.M41) / w,
            (point.X * matrix.M12 + point.Y * matrix.M22 + point.Z * matrix.M32 + matrix.M42) / w,
            (point.X * matrix.M13 + point.Y * matrix.M23 + point.Z * matrix.M33 + matrix.M43) / w
        );
    }

    #endregion
}
