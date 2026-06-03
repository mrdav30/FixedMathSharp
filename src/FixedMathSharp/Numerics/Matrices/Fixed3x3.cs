//=======================================================================
// Fixed3x3.cs
//=======================================================================
// MIT License, Copyright (c) 2024–present David Oravsky (mrdav30)
// See LICENSE file in the project root for full license information.
//=======================================================================

using FixedMathSharp.Support;
using MemoryPack;
using System;
using System.Runtime.CompilerServices;
using System.Text.Json.Serialization;

namespace FixedMathSharp;

/// <summary>
/// Represents a 3x3 matrix used for linear transformations in 2D and 3D space, such as rotation, scaling, and shearing.
/// </summary>
/// <remarks>
/// A 3x3 matrix handles only linear transformations and is typically used when translation is not needed.
/// It operates on directions, orientations, and vectors within a given space without affecting position.
/// This matrix is more lightweight compared to a 4x4 matrix, making it ideal when translation and perspective are unnecessary.
/// 
/// Use Cases:
/// - Rotating or scaling objects around the origin in 2D and 3D space.
/// - Transforming vectors and normals (e.g., in lighting calculations).
/// - Used in physics engines for inertia tensors or to represent local orientations.
/// - Useful when optimizing transformations, as it omits the overhead of translation and perspective.
/// </remarks>
[Serializable]
[MemoryPackable]
public partial struct Fixed3x3 : IEquatable<Fixed3x3>
{
    #region Static Readonly

    /// <summary>
    /// Returns the identity matrix (no scaling, rotation, or translation).
    /// </summary>
    public static readonly Fixed3x3 Identity = new(Vector3d.FromFloatPoint(1f, 0f, 0f), Vector3d.FromFloatPoint(0f, 1f, 0f), Vector3d.FromFloatPoint(0f, 0f, 1f));

    /// <summary>
    /// Returns a matrix with all elements set to zero.
    /// </summary>
    public static readonly Fixed3x3 Zero = new(Vector3d.FromFloatPoint(0f, 0f, 0f), Vector3d.FromFloatPoint(0f, 0f, 0f), Vector3d.FromFloatPoint(0f, 0f, 0f));

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

    /// <summary>
    /// Initializes a new FixedMatrix3x3 with the specified elements.
    /// </summary>
    public Fixed3x3(
        Fixed64 m11, Fixed64 m12, Fixed64 m13,
        Fixed64 m21, Fixed64 m22, Fixed64 m23,
        Fixed64 m31, Fixed64 m32, Fixed64 m33
    )
    {
        this.M11 = m11; this.M12 = m12; this.M13 = m13;
        this.M21 = m21; this.M22 = m22; this.M23 = m23;
        this.M31 = m31; this.M32 = m32; this.M33 = m33;
    }

    /// <summary>
    /// Initializes a new FixedMatrix3x3 using three Vector3d values representing the rows.
    /// </summary>
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
    /// The mapping between indices and matrix elements is non-sequential. 
    /// Ensure that the index corresponds to a valid matrix element.
    /// </remarks>
    /// <param name="index">The zero-based index of the matrix element to get or set. Valid values are 0, 1, 2, 4, 5, 6, 8, 9, and 10.</param>
    /// <returns>The matrix element at the specified index.</returns>
    /// <exception cref="IndexOutOfRangeException">Thrown when the specified index is not one of the valid matrix element indices.</exception>
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

    /// <inheritdoc cref="Normalize(Fixed3x3)" />
    public Fixed3x3 Normalize() => this = Normalize(this);

    /// <inheritdoc cref="ResetScaleToIdentity(Fixed3x3)" />
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public Fixed3x3 ResetScaleToIdentity() => this = ResetScaleToIdentity(this);

    /// <summary>
    /// Calculates the determinant of a 3x3 matrix.
    /// </summary>
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public Fixed64 GetDeterminant() =>
         M11 * (M22 * M33 - M23 * M32) -
         M12 * (M21 * M33 - M23 * M31) +
         M13 * (M21 * M32 - M22 * M31);

    /// <summary>
    /// Inverts the diagonal elements of the matrix.
    /// </summary>
    /// <remarks>
    /// protects against the case where you would have an infinite value on the diagonal, which would cause problems in subsequent computations.
    /// If m00 or m22 are zero, handle that as a special case and manually set the inverse to zero,
    /// since for a theoretical object with no inertia along those axes, it would be impossible to impart a rotation in those directions
    ///
    ///  bear in mind that having a zero on the inertia tensor's diagonal isn't generally valid for real,
    ///  3-dimensional objects (unless they are "infinitely thin" along one axis),
    ///  so if you end up with such a tensor, it's a sign that something else might be wrong in your setup.        
    /// </remarks>
    public Fixed3x3 InvertDiagonal()
    {
        try
        {
            FixedThrowHelper.ThrowIfArgument(M22 == Fixed64.Zero, "Cannot invert a diagonal matrix with zero elements on the diagonal.");
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
            Fixed64.Zero, cos, -sin,
            Fixed64.Zero, sin, cos
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
            cos, Fixed64.Zero, sin,
            Fixed64.Zero, Fixed64.One, Fixed64.Zero,
            -sin, Fixed64.Zero, cos
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
            cos, -sin, Fixed64.Zero,
            sin, cos, Fixed64.Zero,
            Fixed64.Zero, Fixed64.Zero, Fixed64.One
        );
    }

    /// <summary>
    /// Creates a 3x3 shear matrix.
    /// </summary>
    /// <param name="shX">Shear factor along the X-axis.</param>
    /// <param name="shY">Shear factor along the Y-axis.</param>
    /// <param name="shZ">Shear factor along the Z-axis.</param>
    /// <returns>A 3x3 shear matrix.</returns>
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
    /// Normalizes the basis vectors of a 3x3 matrix to ensure they are orthogonal and unit length.
    /// </summary>
    /// <remarks>
    /// This method recalculates and normalizes the X, Y, and Z basis vectors of the matrix to avoid numerical drift 
    /// that can occur after multiple transformations. It also ensures that the Z-axis is recomputed to maintain 
    /// orthogonality by taking the cross-product of the normalized X and Y axes.
    /// 
    /// Use Cases:
    /// - Ensuring stability and correctness after repeated transformations involving rotation and scaling.
    /// - Useful in physics calculations where orthogonal matrices are required (e.g., inertia tensors or rotations).
    /// </remarks>
    public static Fixed3x3 Normalize(Fixed3x3 matrix)
    {
        var x = new Vector3d(matrix.M11, matrix.M12, matrix.M13).Normalize();
        var y = new Vector3d(matrix.M21, matrix.M22, matrix.M23).Normalize();
        var z = Vector3d.Cross(x, y).Normalize();

        matrix.M11 = x.X; matrix.M12 = x.Y; matrix.M13 = x.Z;
        matrix.M21 = y.X; matrix.M22 = y.Y; matrix.M23 = y.Z;
        matrix.M31 = z.X; matrix.M32 = z.Y; matrix.M33 = z.Z;

        return matrix;
    }

    /// <summary>
    /// Resets the scaling part of the matrix to identity (1,1,1).
    /// </summary>
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public static Fixed3x3 ResetScaleToIdentity(Fixed3x3 matrix)
    {
        matrix.M11 = Fixed64.One;  // Reset scale on X-axis
        matrix.M22 = Fixed64.One;  // Reset scale on Y-axis
        matrix.M33 = Fixed64.One;  // Reset scale on Z-axis
        return matrix;
    }

    /// <inheritdoc cref="SetLossyScale(Fixed64, Fixed64, Fixed64)" />
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public static Fixed3x3 SetLossyScale(Vector3d scale) => SetLossyScale(scale.X, scale.Y, scale.Z);

    /// <summary>
    /// Creates a scaling matrix (puts the 'scale' vector down the diagonal)
    /// </summary>
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public static Fixed3x3 SetLossyScale(Fixed64 x, Fixed64 y, Fixed64 z) =>
        new(x, Fixed64.Zero, Fixed64.Zero,
            Fixed64.Zero, y, Fixed64.Zero,
            Fixed64.Zero, Fixed64.Zero, z);

    /// <summary>
    /// Applies the provided local scale to the matrix by modifying the diagonal elements.
    /// </summary>
    /// <param name="matrix">The matrix to set the scale against.</param>
    /// <param name="localScale">A Vector3d representing the local scale to apply.</param>
    public static Fixed3x3 SetScale(Fixed3x3 matrix, Vector3d localScale)
    {
        matrix.M11 = localScale.X; // Apply scale on X-axis
        matrix.M22 = localScale.Y; // Apply scale on Y-axis
        matrix.M33 = localScale.Z; // Apply scale on Z-axis

        return matrix;
    }

    /// <summary>
    /// Sets the global scale of an object using FixedMatrix3x3.
    /// Similar to SetGlobalScale for FixedMatrix4x4, but for a 3x3 matrix.
    /// </summary>
    /// <param name="matrix">The transformation matrix (3x3) representing the object's global state.</param>
    /// <param name="globalScale">The desired global scale represented as a Vector3d.</param>
    /// <remarks>
    /// The method extracts the current global scale from the matrix and computes the new local scale 
    /// by dividing the desired global scale by the current global scale. 
    /// The new local scale is then applied to the matrix.
    /// </remarks>
    public static Fixed3x3 SetGlobalScale(Fixed3x3 matrix, Vector3d globalScale)
    {
        // normalize the matrix to avoid drift in the rotation component
        matrix.Normalize();

        // Reset the local scaling portion of the matrix
        matrix.ResetScaleToIdentity();

        // Compute the new local scale by dividing the desired global scale by the current global scale
        Vector3d newLocalScale = new(
            globalScale.X / Fixed64.One,
            globalScale.Y / Fixed64.One,
            globalScale.Z / Fixed64.One
        );

        // Apply the new local scale to the matrix
        return matrix.SetScale(newLocalScale);
    }

    /// <summary>
    /// Extracts the scaling factors from the matrix by returning the diagonal elements.
    /// </summary>
    /// <returns>A Vector3d representing the scale along X, Y, and Z axes.</returns>
    public static Vector3d ExtractScale(Fixed3x3 matrix) =>
        new(new Vector3d(matrix.M11, matrix.M12, matrix.M13).Magnitude,
            new Vector3d(matrix.M21, matrix.M22, matrix.M23).Magnitude,
            new Vector3d(matrix.M31, matrix.M32, matrix.M33).Magnitude);

    /// <summary>
    /// Extracts the scaling factors from the matrix by returning the diagonal elements (lossy).
    /// </summary>
    /// <returns>A Vector3d representing the scale along X, Y, and Z axes (lossy).</returns>
    public static Vector3d ExtractLossyScale(Fixed3x3 matrix) => new(matrix.M11, matrix.M22, matrix.M33);

    #endregion

    #region Static Matrix Operations

    /// <summary>
    /// Linearly interpolates between two matrices.
    /// </summary>
    public static Fixed3x3 Lerp(Fixed3x3 a, Fixed3x3 b, Fixed64 t) =>
        new(Fixed64.Lerp(a.M11, b.M11, t), Fixed64.Lerp(a.M12, b.M12, t), Fixed64.Lerp(a.M13, b.M13, t),
            Fixed64.Lerp(a.M21, b.M21, t), Fixed64.Lerp(a.M22, b.M22, t), Fixed64.Lerp(a.M23, b.M23, t),
            Fixed64.Lerp(a.M31, b.M31, t), Fixed64.Lerp(a.M32, b.M32, t), Fixed64.Lerp(a.M33, b.M33, t));

    /// <summary>
    /// Transposes the matrix (swaps rows and columns).
    /// </summary>
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public static Fixed3x3 Transpose(Fixed3x3 matrix) =>
        new(matrix.M11, matrix.M21, matrix.M31,
            matrix.M12, matrix.M22, matrix.M32,
            matrix.M13, matrix.M23, matrix.M33);

    /// <summary>
    /// Attempts to invert the matrix. If the determinant is zero, returns false and sets result to null.
    /// </summary>
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

    /// <summary>
    /// Transforms a direction vector from local space to world space using this transformation matrix.
    /// Ignores translation.
    /// </summary>
    /// <param name="matrix">The transformation matrix.</param>
    /// <param name="direction">The local-space direction vector.</param>
    /// <returns>The transformed direction in world space.</returns>
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public static Vector3d TransformDirection(Fixed3x3 matrix, Vector3d direction) =>
        new(matrix.M11 * direction.X + matrix.M12 * direction.Y + matrix.M13 * direction.Z,
            matrix.M21 * direction.X + matrix.M22 * direction.Y + matrix.M23 * direction.Z,
            matrix.M31 * direction.X + matrix.M32 * direction.Y + matrix.M33 * direction.Z);

    /// <summary>
    /// Transforms a direction from world space into the local space of the matrix.
    /// Ignores translation.
    /// </summary>
    /// <param name="matrix">The transformation matrix.</param>
    /// <param name="direction">The world-space direction.</param>
    /// <returns>The transformed local-space direction.</returns>
    public static Vector3d InverseTransformDirection(Fixed3x3 matrix, Vector3d direction)
    {
        bool canInvert = !Invert(matrix, out Fixed3x3? inverseMatrix) || !inverseMatrix.HasValue;
        FixedThrowHelper.ThrowIfInvalid(canInvert, "Matrix is not invertible.");

        return new Vector3d(
            inverseMatrix!.Value.M11 * direction.X + inverseMatrix.Value.M12 * direction.Y + inverseMatrix.Value.M13 * direction.Z,
            inverseMatrix.Value.M21 * direction.X + inverseMatrix.Value.M22 * direction.Y + inverseMatrix.Value.M23 * direction.Z,
            inverseMatrix.Value.M31 * direction.X + inverseMatrix.Value.M32 * direction.Y + inverseMatrix.Value.M33 * direction.Z
        );
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
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public static Fixed3x3 operator -(Fixed3x3 a) =>
        new(-a.M11, -a.M12, -a.M13,
            -a.M21, -a.M22, -a.M23,
            -a.M31, -a.M32, -a.M33);

    /// <summary>
    /// Performs matrix multiplication on two 3x3 matrices.
    /// </summary>
    /// <remarks>Matrix multiplication is not commutative; the order of operands affects the result.</remarks>
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
    /// <param name="divisor">The scalar value by which to divide each element of the matrix.</param>
    /// <returns>
    /// A new Fixed3x3 matrix whose elements are the result of dividing the corresponding elements of the input matrix by the specified scalar.
    /// </returns>
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

    /// <inheritdoc/>
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public bool Equals(Fixed3x3 other) =>
        M11 == other.M11 && M12 == other.M12 && M13 == other.M13 &&
        M21 == other.M21 && M22 == other.M22 && M23 == other.M23 &&
        M31 == other.M31 && M32 == other.M32 && M33 == other.M33;


    /// <inheritdoc/>
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public override bool Equals(object? obj) => obj is Fixed3x3 other && Equals(other);

    /// <inheritdoc/>
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
    /// <returns>A string containing the matrix elements formatted as "[m00, m01, m02; m10, m11, m12; m20, m21, m22]".</returns>
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public override string ToString() =>
         $"[{M11}, {M12}, {M13}; {M21}, {M22}, {M23}; {M31}, {M32}, {M33}]";

    #endregion
}