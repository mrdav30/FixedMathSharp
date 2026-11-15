//=======================================================================
// Fixed3x3.cs
//=======================================================================
// MIT License, Copyright (c) 2024-present David Oravsky (mrdav30)
// See LICENSE file in the project root for full license information.
//=======================================================================

using System;
using System.Globalization;
using System.Runtime.CompilerServices;
using System.Text.Json.Serialization;
using FixedMathSharp.Geometry;
using MemoryPack;

namespace FixedMathSharp;

/// <summary>Represents a 3×3 fixed-point matrix for linear transformations and products.</summary>
/// <remarks>
/// For 3D transforms, vectors multiply from the left and the rows are the transformed
/// X, Y, and Z basis axes. Matrix products compose left to right for row vectors.
/// </remarks>
[Serializable]
[MemoryPackable]
public partial struct Fixed3x3 : IEquatable<Fixed3x3>, IFormattable
#if NET10_0_OR_GREATER
    , ISpanFormattable
#endif
{
    #region Static Readonly

    /// <summary>
    /// Returns the identity matrix (no scaling, rotation, or translation).
    /// </summary>
    public static readonly Fixed3x3 Identity = new(Vector3d.FromDouble(1f, 0f, 0f), Vector3d.FromDouble(0f, 1f, 0f), Vector3d.FromDouble(0f, 0f, 1f));

    /// <summary>
    /// Returns a matrix with all elements set to zero.
    /// </summary>
    public static readonly Fixed3x3 Zero = new(Vector3d.FromDouble(0f, 0f, 0f), Vector3d.FromDouble(0f, 0f, 0f), Vector3d.FromDouble(0f, 0f, 0f));

    #endregion

    #region Fields

    // First row

    /// <summary>
    /// Represents the element in the first row and first column of the matrix.
    /// </summary>
    [JsonInclude]
    [MemoryPackOrder(0)]
    public Fixed64 M11;
    /// <summary>
    /// Represents the element in the first row and second column of the matrix.
    /// </summary>
    [JsonInclude]
    [MemoryPackOrder(1)]
    public Fixed64 M12;
    /// <summary>
    /// Represents the element in the first row and third column of the matrix.
    /// </summary>
    [JsonInclude]
    [MemoryPackOrder(2)]
    public Fixed64 M13;

    // Second Row

    /// <summary>
    /// Represents the element in the second row and first column of the matrix.
    /// </summary>
    [JsonInclude]
    [MemoryPackOrder(3)]
    public Fixed64 M21;
    /// <summary>
    /// Represents the element in the second row and second column of the matrix.
    /// </summary>
    [JsonInclude]
    [MemoryPackOrder(4)]
    public Fixed64 M22;
    /// <summary>
    /// Represents the element in the second row and third column of the matrix.
    /// </summary>
    [JsonInclude]
    [MemoryPackOrder(5)]
    public Fixed64 M23;

    // Third Row

    /// <summary>
    /// Represents the element in the third row and first column of the matrix.
    /// </summary>
    [JsonInclude]
    [MemoryPackOrder(6)]
    public Fixed64 M31;
    /// <summary>
    /// Represents the element in the third row and second column of the matrix.
    /// </summary>
    [JsonInclude]
    [MemoryPackOrder(7)]
    public Fixed64 M32;
    /// <summary>
    /// Represents the element in the third row and third column of the matrix.
    /// </summary>
    [JsonInclude]
    [MemoryPackOrder(8)]
    public Fixed64 M33;

    #endregion

    #region Constructors

    /// <summary>Creates a matrix from nine entries in row order.</summary>
    /// <param name="m11">The first row, first column entry.</param>
    /// <param name="m12">The first row, second column entry.</param>
    /// <param name="m13">The first row, third column entry.</param>
    /// <param name="m21">The second row, first column entry.</param>
    /// <param name="m22">The second row, second column entry.</param>
    /// <param name="m23">The second row, third column entry.</param>
    /// <param name="m31">The third row, first column entry.</param>
    /// <param name="m32">The third row, second column entry.</param>
    /// <param name="m33">The third row, third column entry.</param>
    public Fixed3x3(
        Fixed64 m11, Fixed64 m12, Fixed64 m13,
        Fixed64 m21, Fixed64 m22, Fixed64 m23,
        Fixed64 m31, Fixed64 m32, Fixed64 m33
    )
    {
        M11 = m11; M12 = m12; M13 = m13;
        M21 = m21; M22 = m22; M23 = m23;
        M31 = m31; M32 = m32; M33 = m33;
    }

    /// <summary>Creates a matrix from three row vectors.</summary>
    /// <param name="m11_m12_m13">The first row, with X, Y, and Z as columns one through three.</param>
    /// <param name="m21_m22_m23">The second row, with X, Y, and Z as columns one through three.</param>
    /// <param name="m31_m32_m33">The third row, with X, Y, and Z as columns one through three.</param>
    public Fixed3x3(
        Vector3d m11_m12_m13,
        Vector3d m21_m22_m23,
        Vector3d m31_m32_m33
    ) : this(
        m11_m12_m13.X,
        m11_m12_m13.Y,
        m11_m12_m13.Z,
        m21_m22_m23.X,
        m21_m22_m23.Y,
        m21_m22_m23.Z,
        m31_m32_m33.X,
        m31_m32_m33.Y,
        m31_m32_m33.Z)
    { }

    #endregion

    #region Properties

    /// <summary>
    /// Gets or sets the matrix element at the specified index.
    /// </summary>
    /// <remarks>
    /// The index is the zero-based row plus four times the zero-based column.
    /// </remarks>
    /// <param name="index">The zero-based index of the matrix element to get or set. Valid values are 0, 1, 2, 4, 5, 6, 8, 9, and 10.</param>
    /// <value>The entry at <paramref name="index"/>.</value>
    /// <exception cref="IndexOutOfRangeException"><paramref name="index"/> is not one of the supported indices. The matrix is unchanged.</exception>
    [JsonIgnore]
    [MemoryPackIgnore]
    public Fixed64 this[int index]
    {
        get
        {
            return index switch
            {
                0 => M11,
                1 => M21,
                2 => M31,
                4 => M12,
                5 => M22,
                6 => M32,
                8 => M13,
                9 => M23,
                10 => M33,
                _ => throw new IndexOutOfRangeException("Invalid matrix index!"),
            };
        }
        set
        {
            switch (index)
            {
                case 0:
                    M11 = value;
                    break;
                case 1:
                    M21 = value;
                    break;
                case 2:
                    M31 = value;
                    break;
                case 4:
                    M12 = value;
                    break;
                case 5:
                    M22 = value;
                    break;
                case 6:
                    M32 = value;
                    break;
                case 8:
                    M13 = value;
                    break;
                case 9:
                    M23 = value;
                    break;
                case 10:
                    M33 = value;
                    break;
                default:
                    throw new IndexOutOfRangeException("Invalid matrix index!");
            }
        }
    }

    #endregion

    #region Methods (Instance)

    /// <summary>Replaces this matrix with the basis rows produced by <see cref="GetNormalized(Fixed3x3)"/>.</summary>
    /// <returns>The updated matrix.</returns>
    public Fixed3x3 NormalizeInPlace() => this = GetNormalized(this);

    /// <summary>
    /// Calculates the determinant of a 3x3 matrix.
    /// </summary>
    /// <returns>The cofactor expansion evaluated with fixed-point rounding and saturation.</returns>
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public Fixed64 GetDeterminant() =>
         M11 * (M22 * M33 - M23 * M32) -
         M12 * (M21 * M33 - M23 * M31) +
         M13 * (M21 * M32 - M22 * M31);

    /// <summary>Returns a diagonal matrix with the reciprocals of the nonzero diagonal entries.</summary>
    /// <returns>
    /// The original matrix if <see cref="M22"/> is zero. Otherwise, a matrix with each nonzero
    /// diagonal entry replaced by its fixed-point reciprocal and all other entries set to zero.
    /// </returns>
    public Fixed3x3 InvertDiagonal()
    {
        try
        {
            if (M22 == Fixed64.Zero)
                throw new ArgumentException("Cannot invert a diagonal matrix with zero elements on the diagonal.");
        }
        catch (ArgumentException)
        {
            return this;
        }

        return new Fixed3x3(
            M11 != Fixed64.Zero ? Fixed64.One / M11 : Fixed64.Zero, Fixed64.Zero, Fixed64.Zero,
            Fixed64.Zero, Fixed64.One / M22, Fixed64.Zero,
            Fixed64.Zero, Fixed64.Zero, M33 != Fixed64.Zero ? Fixed64.One / M33 : Fixed64.Zero
        );
    }

    #endregion

    #region Static Matrix Generators and Transformations

    /// <summary>
    /// Creates a 3x3 matrix representing a rotation around the X-axis.
    /// </summary>
    /// <param name="angle">The angle of rotation in radians.</param>
    /// <returns>A 3x3 rotation matrix.</returns>
    public static Fixed3x3 CreateRotationX(Fixed64 angle)
    {
        Fixed64 cos = FixedMath.Cos(angle);
        Fixed64 sin = FixedMath.Sin(angle);

        return new Fixed3x3(
            Fixed64.One, Fixed64.Zero, Fixed64.Zero,
            Fixed64.Zero, cos, sin,
            Fixed64.Zero, -sin, cos
        );
    }

    /// <summary>
    /// Creates a 3x3 matrix representing a rotation around the Y-axis.
    /// </summary>
    /// <param name="angle">The angle of rotation in radians.</param>
    /// <returns>A 3x3 rotation matrix.</returns>
    public static Fixed3x3 CreateRotationY(Fixed64 angle)
    {
        Fixed64 cos = FixedMath.Cos(angle);
        Fixed64 sin = FixedMath.Sin(angle);

        return new Fixed3x3(
            cos, Fixed64.Zero, -sin,
            Fixed64.Zero, Fixed64.One, Fixed64.Zero,
            sin, Fixed64.Zero, cos
        );
    }

    /// <summary>
    /// Creates a 3x3 matrix representing a rotation around the Z-axis.
    /// </summary>
    /// <param name="angle">The angle of rotation in radians.</param>
    /// <returns>A 3x3 rotation matrix.</returns>
    public static Fixed3x3 CreateRotationZ(Fixed64 angle)
    {
        Fixed64 cos = FixedMath.Cos(angle);
        Fixed64 sin = FixedMath.Sin(angle);

        return new Fixed3x3(
            cos, sin, Fixed64.Zero,
            -sin, cos, Fixed64.Zero,
            Fixed64.Zero, Fixed64.Zero, Fixed64.One
        );
    }

    /// <summary>
    /// Creates a 3x3 shear matrix.
    /// </summary>
    /// <param name="shX">The coupling between X and Y, stored in M12 and M21.</param>
    /// <param name="shY">The coupling between X and Z, stored in M13 and M31.</param>
    /// <param name="shZ">The coupling between Y and Z, stored in M23 and M32.</param>
    /// <returns>A symmetric shear matrix with unit diagonal entries.</returns>
    public static Fixed3x3 CreateShear(Fixed64 shX, Fixed64 shY, Fixed64 shZ) =>
         new(Fixed64.One, shX, shY,
            shX, Fixed64.One, shZ,
            shY, shZ, Fixed64.One);

    /// <summary>
    /// Creates a scaling matrix that applies a uniform or non-uniform scale transformation.
    /// </summary>
    /// <param name="scale">The scale factors along the X, Y, and Z axes.</param>
    /// <returns>A 3x3 scaling matrix.</returns>
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public static Fixed3x3 CreateScale(Vector3d scale) =>
        new(scale.X, Fixed64.Zero, Fixed64.Zero,
            Fixed64.Zero, scale.Y, Fixed64.Zero,
            Fixed64.Zero, Fixed64.Zero, scale.Z);

    /// <summary>
    /// Creates a uniform scaling matrix with the same scale factor on all axes.
    /// </summary>
    /// <param name="scaleFactor">The uniform scale factor.</param>
    /// <returns>A 3x3 scaling matrix with uniform scaling.</returns>
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public static Fixed3x3 CreateScale(Fixed64 scaleFactor) =>
        CreateScale(new Vector3d(scaleFactor, scaleFactor, scaleFactor));

    /// <summary>
    /// Creates a symmetric matrix containing second-order barycentric product sums for three vectors.
    /// </summary>
    /// <param name="a">The first vertex in the common coordinate space.</param>
    /// <param name="b">The second vertex in the common coordinate space.</param>
    /// <param name="c">The third vertex in the common coordinate space.</param>
    /// <returns>The symmetric matrix of sums, in squared input units.</returns>
    /// <remarks>
    /// Diagonal entries use <see cref="FixedMath.SumSquaredBarycentricProducts(Fixed64, Fixed64, Fixed64)"/>.
    /// Off-diagonal entries use <see cref="FixedMath.SumBarycentricProducts(Fixed64, Fixed64, Fixed64, Fixed64, Fixed64, Fixed64)"/>.
    /// </remarks>
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public static Fixed3x3 CreateBarycentricProductSums(Vector3d a, Vector3d b, Vector3d c)
    {
        Fixed64 xx = FixedMath.SumSquaredBarycentricProducts(a.X, b.X, c.X);
        Fixed64 yy = FixedMath.SumSquaredBarycentricProducts(a.Y, b.Y, c.Y);
        Fixed64 zz = FixedMath.SumSquaredBarycentricProducts(a.Z, b.Z, c.Z);
        Fixed64 xy = FixedMath.SumBarycentricProducts(a.X, b.X, c.X, a.Y, b.Y, c.Y);
        Fixed64 xz = FixedMath.SumBarycentricProducts(a.X, b.X, c.X, a.Z, b.Z, c.Z);
        Fixed64 yz = FixedMath.SumBarycentricProducts(a.Y, b.Y, c.Y, a.Z, b.Z, c.Z);

        return new Fixed3x3(
            xx, xy, xz,
            xy, yy, yz,
            xz, yz, zz);
    }

    /// <summary>Normalizes the first two basis rows and rebuilds the third from their cross product.</summary>
    /// <param name="matrix">The matrix whose first two rows define the basis.</param>
    /// <returns>The adjusted copy. A zero first or second row stays zero, and a zero cross product gives a zero third row.</returns>
    /// <remarks>
    /// The angle between the first two nonzero rows is preserved subject to fixed-point rounding.
    /// </remarks>
    public static Fixed3x3 GetNormalized(Fixed3x3 matrix)
    {
        var x = new Vector3d(matrix.M11, matrix.M12, matrix.M13).NormalizeInPlace();
        var y = new Vector3d(matrix.M21, matrix.M22, matrix.M23).NormalizeInPlace();
        var z = Vector3d.Cross(x, y).NormalizeInPlace();

        matrix.M11 = x.X; matrix.M12 = x.Y; matrix.M13 = x.Z;
        matrix.M21 = y.X; matrix.M22 = y.Y; matrix.M23 = y.Z;
        matrix.M31 = z.X; matrix.M32 = z.Y; matrix.M33 = z.Z;

        return matrix;
    }

    /// <summary>
    /// Extracts the unsigned magnitudes of the matrix basis rows.
    /// </summary>
    /// <param name="matrix">The matrix containing the basis rows.</param>
    /// <returns>The nonnegative basis magnitudes along X, Y, and Z.</returns>
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public static Vector3d ExtractScaleMagnitudes(Fixed3x3 matrix) =>
        new(new Vector3d(matrix.M11, matrix.M12, matrix.M13).Magnitude,
            new Vector3d(matrix.M21, matrix.M22, matrix.M23).Magnitude,
            new Vector3d(matrix.M31, matrix.M32, matrix.M33).Magnitude);

    /// <summary>
    /// Extracts canonical signed lossy scale from the matrix basis rows.
    /// </summary>
    /// <param name="matrix">The matrix containing the basis rows.</param>
    /// <returns>The X, Y, and Z row lengths with the recovered reflection sign on X.</returns>
    /// <remarks>A reflected basis assigns its one recoverable negative sign to X.</remarks>
    public static Vector3d ExtractLossyScale(Fixed3x3 matrix)
    {
        Vector3d scale = ExtractScaleMagnitudes(matrix);
        if (WideGeometry.GetTripleProductSign(
            matrix.M11, matrix.M12, matrix.M13,
            matrix.M21, matrix.M22, matrix.M23,
            matrix.M31, matrix.M32, matrix.M33) < 0)
        {
            scale.X = -scale.X;
        }

        return scale;
    }

    #endregion

    #region Static Matrix Operations

    /// <summary>
    /// Linearly interpolates between two matrices.
    /// </summary>
    /// <param name="a">The starting matrix.</param>
    /// <param name="b">The ending matrix.</param>
    /// <param name="t">The interpolation fraction, clamped to the inclusive range from zero to one.</param>
    /// <returns>The matrix of entrywise interpolated values.</returns>
    public static Fixed3x3 Lerp(Fixed3x3 a, Fixed3x3 b, Fixed64 t) =>
        new(FixedMath.Lerp(a.M11, b.M11, t), FixedMath.Lerp(a.M12, b.M12, t), FixedMath.Lerp(a.M13, b.M13, t),
            FixedMath.Lerp(a.M21, b.M21, t), FixedMath.Lerp(a.M22, b.M22, t), FixedMath.Lerp(a.M23, b.M23, t),
            FixedMath.Lerp(a.M31, b.M31, t), FixedMath.Lerp(a.M32, b.M32, t), FixedMath.Lerp(a.M33, b.M33, t));

    /// <summary>
    /// Transposes the matrix (swaps rows and columns).
    /// </summary>
    /// <param name="matrix">The matrix to transpose.</param>
    /// <returns>A copy whose rows are the original columns.</returns>
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public static Fixed3x3 Transpose(Fixed3x3 matrix) =>
        new(matrix.M11, matrix.M21, matrix.M31,
            matrix.M12, matrix.M22, matrix.M32,
            matrix.M13, matrix.M23, matrix.M33);

    /// <summary>Attempts to invert the matrix using its fixed-point determinant.</summary>
    /// <param name="matrix">The matrix to invert.</param>
    /// <param name="result">The fixed-point cofactor inverse on success, or <see langword="null"/> on failure.</param>
    /// <returns><see langword="true"/> if the computed determinant is nonzero, otherwise <see langword="false"/>.</returns>
    /// <remarks>Products and sums can round or saturate. Success means an inverse was computed from the rounded determinant.</remarks>
    public static bool Invert(Fixed3x3 matrix, out Fixed3x3? result)
    {
        // Calculate the determinant
        Fixed64 det = matrix.GetDeterminant();

        if (det == Fixed64.Zero)
        {
            result = null;
            return false;
        }

        // Calculate the inverse
        Fixed64 invDet = Fixed64.One / det;

        // Compute the inverse matrix
        result = new Fixed3x3(
            invDet * (matrix.M22 * matrix.M33 - matrix.M32 * matrix.M23),
            invDet * (matrix.M13 * matrix.M32 - matrix.M12 * matrix.M33),
            invDet * (matrix.M12 * matrix.M23 - matrix.M13 * matrix.M22),

            invDet * (matrix.M23 * matrix.M31 - matrix.M21 * matrix.M33),
            invDet * (matrix.M11 * matrix.M33 - matrix.M13 * matrix.M31),
            invDet * (matrix.M13 * matrix.M21 - matrix.M11 * matrix.M23),

            invDet * (matrix.M21 * matrix.M32 - matrix.M22 * matrix.M31),
            invDet * (matrix.M12 * matrix.M31 - matrix.M11 * matrix.M32),
            invDet * (matrix.M11 * matrix.M22 - matrix.M12 * matrix.M21)
        );

        return true;
    }

    /// <summary>Applies a linear transform to a direction as a row vector.</summary>
    /// <remarks>
    /// Computes <c>direction * matrix</c> using ordinary fixed-point operations. Scale and shear can change the length.
    /// </remarks>
    /// <param name="matrix">The transformation matrix.</param>
    /// <param name="direction">The local-space direction vector.</param>
    /// <returns>The transformed direction in world space.</returns>
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public static Vector3d TransformDirection(Fixed3x3 matrix, Vector3d direction) =>
        new(direction.X * matrix.M11 + direction.Y * matrix.M21 + direction.Z * matrix.M31,
            direction.X * matrix.M12 + direction.Y * matrix.M22 + direction.Z * matrix.M32,
            direction.X * matrix.M13 + direction.Y * matrix.M23 + direction.Z * matrix.M33);

    /// <summary>
    /// Attempts to transform a direction with one final round-half-to-even
    /// conversion per component and no intermediate saturation.
    /// </summary>
    /// <param name="matrix">The linear transform from input space to output space.</param>
    /// <param name="direction">The input-space direction. It can have any length.</param>
    /// <param name="result">The transformed direction on success, or <see cref="Vector3d.Zero"/> on failure.</param>
    /// <returns><see langword="true"/> if all three rounded components fit in <see cref="Fixed64"/>, otherwise <see langword="false"/>.</returns>
    /// <remarks>Computes <c>direction * matrix</c>. Scale and shear can change the length.</remarks>
    public static bool TryTransformDirection(
        Fixed3x3 matrix,
        Vector3d direction,
        out Vector3d result)
    {
        bool representable = Fixed64.TryAddProducts(
                direction.X,
                matrix.M11,
                direction.Y,
                matrix.M21,
                direction.Z,
                matrix.M31,
                out Fixed64 x)
            & Fixed64.TryAddProducts(
                direction.X,
                matrix.M12,
                direction.Y,
                matrix.M22,
                direction.Z,
                matrix.M32,
                out Fixed64 y)
            & Fixed64.TryAddProducts(
                direction.X,
                matrix.M13,
                direction.Y,
                matrix.M23,
                direction.Z,
                matrix.M33,
                out Fixed64 z);
        if (!representable)
        {
            result = default;
            return false;
        }

        result = new Vector3d(x, y, z);
        return true;
    }

    /// <summary>
    /// Transforms a direction from world space into the local space of the matrix.
    /// Ignores translation.
    /// </summary>
    /// <param name="matrix">The transformation matrix.</param>
    /// <param name="direction">The world-space direction.</param>
    /// <returns>The transformed local-space direction.</returns>
    /// <exception cref="InvalidOperationException"><see cref="Invert(Fixed3x3, out Fixed3x3?)"/> could not invert the matrix.</exception>
    public static Vector3d InverseTransformDirection(Fixed3x3 matrix, Vector3d direction)
    {
        bool canInvert = !Invert(matrix, out Fixed3x3? inverseMatrix) || !inverseMatrix.HasValue;
        if (canInvert)
            throw new InvalidOperationException("Matrix is not invertible.");

        return TransformDirection(inverseMatrix!.Value, direction);
    }

    #endregion

    #region Operators

    /// <summary>
    /// Subtracts each corresponding element of one Fixed3x3 matrix from another.
    /// </summary>
    /// <param name="a">The first Fixed3x3 matrix (the minuend).</param>
    /// <param name="b">The second Fixed3x3 matrix (the subtrahend).</param>
    /// <returns>
    /// A Fixed3x3 matrix whose elements are the result of subtracting each element of parameter b from the corresponding element of parameter a.
    /// </returns>
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public static Fixed3x3 operator -(Fixed3x3 a, Fixed3x3 b) =>
        new(a.M11 - b.M11, a.M12 - b.M12, a.M13 - b.M13,
            a.M21 - b.M21, a.M22 - b.M22, a.M23 - b.M23,
            a.M31 - b.M31, a.M32 - b.M32, a.M33 - b.M33);

    /// <summary>
    /// Adds two Fixed3x3 matrices element-wise.
    /// </summary>
    /// <param name="a">The first matrix to add.</param>
    /// <param name="b">The second matrix to add.</param>
    /// <returns>A Fixed3x3 matrix whose elements are the sums of the corresponding elements of the input matrices.</returns>
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public static Fixed3x3 operator +(Fixed3x3 a, Fixed3x3 b) =>
        new(a.M11 + b.M11, a.M12 + b.M12, a.M13 + b.M13,
            a.M21 + b.M21, a.M22 + b.M22, a.M23 + b.M23,
            a.M31 + b.M31, a.M32 + b.M32, a.M33 + b.M33);

    /// <summary>
    /// Negates all elements of the matrix.
    /// </summary>
    /// <param name="a">The matrix to negate.</param>
    /// <returns>A matrix of negated entries, each saturated to the <see cref="Fixed64"/> range.</returns>
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public static Fixed3x3 operator -(Fixed3x3 a) =>
        new(-a.M11, -a.M12, -a.M13,
            -a.M21, -a.M22, -a.M23,
            -a.M31, -a.M32, -a.M33);

    /// <summary>
    /// Performs matrix multiplication on two 3x3 matrices.
    /// </summary>
    /// <remarks>For row vectors, this composes applying <paramref name="a"/> followed by <paramref name="b"/>.</remarks>
    /// <param name="a">The first matrix to multiply.</param>
    /// <param name="b">The second matrix to multiply.</param>
    /// <returns>A new Fixed3x3 instance that is the product of the two input matrices.</returns>
    public static Fixed3x3 operator *(Fixed3x3 a, Fixed3x3 b) =>
        new(a.M11 * b.M11 + a.M12 * b.M21 + a.M13 * b.M31,
            a.M11 * b.M12 + a.M12 * b.M22 + a.M13 * b.M32,
            a.M11 * b.M13 + a.M12 * b.M23 + a.M13 * b.M33,

            a.M21 * b.M11 + a.M22 * b.M21 + a.M23 * b.M31,
            a.M21 * b.M12 + a.M22 * b.M22 + a.M23 * b.M32,
            a.M21 * b.M13 + a.M22 * b.M23 + a.M23 * b.M33,

            a.M31 * b.M11 + a.M32 * b.M21 + a.M33 * b.M31,
            a.M31 * b.M12 + a.M32 * b.M22 + a.M33 * b.M32,
            a.M31 * b.M13 + a.M32 * b.M23 + a.M33 * b.M33);

    /// <summary>
    /// Multiplies each element of the specified matrix by the given scalar value.
    /// </summary>
    /// <param name="a">The matrix whose elements are to be multiplied.</param>
    /// <param name="scalar">The scalar value by which to multiply each element of the matrix.</param>
    /// <returns>
    /// A new Fixed3x3 matrix whose elements are the result of multiplying each element of the input matrix by the scalar value.
    /// </returns>
    public static Fixed3x3 operator *(Fixed3x3 a, Fixed64 scalar) =>
        new(a.M11 * scalar, a.M12 * scalar, a.M13 * scalar,
            a.M21 * scalar, a.M22 * scalar, a.M23 * scalar,
            a.M31 * scalar, a.M32 * scalar, a.M33 * scalar);

    /// <inheritdoc cref="operator *(Fixed3x3, Fixed64)"/>
    public static Fixed3x3 operator *(Fixed64 scalar, Fixed3x3 a) => a * scalar;

    /// <summary>
    /// Divides each element of the specified matrix by the given scalar value.
    /// </summary>
    /// <remarks>
    /// Division is performed element-wise. 
    /// The result may lose precision if the divisor does not evenly divide the matrix elements.
    /// </remarks>
    /// <param name="a">The matrix whose elements are to be divided.</param>
    /// <param name="divisor">The nonzero integer by which to divide each matrix entry.</param>
    /// <returns>
    /// A new Fixed3x3 matrix whose elements are the result of dividing the corresponding elements of the input matrix by the specified scalar.
    /// </returns>
    /// <exception cref="DivideByZeroException"><paramref name="divisor"/> is zero.</exception>
    public static Fixed3x3 operator /(Fixed3x3 a, int divisor) =>
         new(a.M11 / divisor, a.M12 / divisor, a.M13 / divisor,
             a.M21 / divisor, a.M22 / divisor, a.M23 / divisor,
             a.M31 / divisor, a.M32 / divisor, a.M33 / divisor);

    /// <summary>
    /// Determines whether two Fixed3x3 instances are equal.
    /// </summary>
    /// <param name="left">The first Fixed3x3 instance to compare.</param>
    /// <param name="right">The second Fixed3x3 instance to compare.</param>
    /// <returns>true if the specified Fixed3x3 instances are equal; otherwise, false.</returns>
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public static bool operator ==(Fixed3x3 left, Fixed3x3 right) => left.Equals(right);

    /// <summary>
    /// Determines whether two Fixed3x3 instances are not equal.
    /// </summary>
    /// <param name="left">The first Fixed3x3 instance to compare.</param>
    /// <param name="right">The second Fixed3x3 instance to compare.</param>
    /// <returns>true if the specified Fixed3x3 instances are not equal; otherwise, false.</returns>
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public static bool operator !=(Fixed3x3 left, Fixed3x3 right) => !left.Equals(right);

    #endregion

    #region Equality and HashCode Overrides

    /// <summary>Tests two matrices for exact component equality.</summary>
    /// <param name="other">The matrix to compare with this instance.</param>
    /// <returns><see langword="true"/> if all nine entries match exactly.</returns>
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public bool Equals(Fixed3x3 other) =>
        M11 == other.M11 && M12 == other.M12 && M13 == other.M13 &&
        M21 == other.M21 && M22 == other.M22 && M23 == other.M23 &&
        M31 == other.M31 && M32 == other.M32 && M33 == other.M33;


    /// <summary>Tests whether an object is a matrix with the same entries.</summary>
    /// <param name="obj">The object to compare with this matrix.</param>
    /// <returns><see langword="true"/> if the object is a <see cref="Fixed3x3"/> and all entries match exactly.</returns>
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public override bool Equals(object? obj) => obj is Fixed3x3 other && Equals(other);

    /// <summary>Computes a hash code from the nine matrix entries.</summary>
    /// <returns>The hash code for this matrix value.</returns>
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public override int GetHashCode()
    {
        unchecked
        {
            int hash = 17;
            hash = hash * 23 + M11.GetHashCode();
            hash = hash * 23 + M12.GetHashCode();
            hash = hash * 23 + M13.GetHashCode();
            hash = hash * 23 + M21.GetHashCode();
            hash = hash * 23 + M22.GetHashCode();
            hash = hash * 23 + M23.GetHashCode();
            hash = hash * 23 + M31.GetHashCode();
            hash = hash * 23 + M32.GetHashCode();
            hash = hash * 23 + M33.GetHashCode();
            return hash;
        }
    }

    #endregion

    #region Conversion

    /// <summary>
    /// Returns a string that represents the current matrix in a readable format.
    /// </summary>
    /// <remarks>
    /// This method is useful for debugging or logging the contents of the matrix. 
    /// The returned string lists the matrix elements in row-major order.
    /// </remarks>
    /// <returns>The bracketed entries in row order using invariant-culture general numeric formatting.</returns>
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public override string ToString() => ToString(null, CultureInfo.InvariantCulture);

    /// <summary>
    /// Returns a string that represents the current matrix in a readable format.
    /// </summary>
    /// <param name="format">The numeric format for each entry. Null or empty uses general formatting.</param>
    /// <param name="formatProvider">The numeric format provider. Null selects the invariant culture.</param>
    /// <returns>The bracketed entries in row order, with commas between entries and semicolons between rows.</returns>
    /// <remarks>Formatting converts each entry to <see cref="double"/> for display.</remarks>
    /// <exception cref="FormatException"><paramref name="format"/> is an invalid numeric format.</exception>
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public string ToString(string? format, IFormatProvider? formatProvider)
    {
        Fixed3x3 value = this;
        return FixedDiagnosticsFormatter.ToString((Span<char> destination, out int charsWritten) =>
            value.TryFormat(destination, out charsWritten, format.AsSpan(), formatProvider));
    }

    /// <summary>
    /// Formats this matrix into the provided destination buffer.
    /// </summary>
    /// <param name="destination">The caller-owned buffer, written from its start.</param>
    /// <param name="charsWritten">The total characters written on success, or zero when the text does not fit.</param>
    /// <param name="format">The numeric format for each entry. An empty span uses general formatting.</param>
    /// <param name="provider">The numeric format provider. Null selects the invariant culture.</param>
    /// <returns><see langword="true"/> if the entire matrix fits, otherwise <see langword="false"/>.</returns>
    /// <remarks>The text matches <see cref="ToString(string, IFormatProvider)"/>. The buffer can contain a partial prefix on failure.</remarks>
    /// <exception cref="FormatException"><paramref name="format"/> is an invalid numeric format encountered while writing. A prefix may already have been written.</exception>
    public bool TryFormat(
        Span<char> destination,
        out int charsWritten,
        ReadOnlySpan<char> format,
        IFormatProvider? provider)
    {
        int written = 0;
        if (!FixedDiagnosticsFormatter.Append('[', destination, ref written) ||
            !AppendRow(M11, M12, M13, destination, ref written, format, provider) ||
            !FixedDiagnosticsFormatter.Append("; ", destination, ref written) ||
            !AppendRow(M21, M22, M23, destination, ref written, format, provider) ||
            !FixedDiagnosticsFormatter.Append("; ", destination, ref written) ||
            !AppendRow(M31, M32, M33, destination, ref written, format, provider) ||
            !FixedDiagnosticsFormatter.Append(']', destination, ref written))
        {
            charsWritten = 0;
            return false;
        }

        charsWritten = written;
        return true;
    }

    private static bool AppendRow(
        Fixed64 x,
        Fixed64 y,
        Fixed64 z,
        Span<char> destination,
        ref int charsWritten,
        ReadOnlySpan<char> format,
        IFormatProvider? provider)
    {
        return FixedDiagnosticsFormatter.Append(x, destination, ref charsWritten, format, provider) &&
               FixedDiagnosticsFormatter.Append(", ", destination, ref charsWritten) &&
               FixedDiagnosticsFormatter.Append(y, destination, ref charsWritten, format, provider) &&
               FixedDiagnosticsFormatter.Append(", ", destination, ref charsWritten) &&
               FixedDiagnosticsFormatter.Append(z, destination, ref charsWritten, format, provider);
    }

    #endregion
}
