//=======================================================================
// Fixed4x4.cs
//=======================================================================
// MIT License, Copyright (c) 2024–present David Oravsky (mrdav30)
// See LICENSE file in the project root for full license information.
//=======================================================================

using MemoryPack;
using System;
using System.Runtime.CompilerServices;
using System.Text.Json.Serialization;

namespace FixedMathSharp;

/// <summary>
/// Represents a 4x4 matrix used for transformations in 3D space, including translation, rotation, scaling, and perspective projection.
/// </summary>
/// <remarks>
/// A 4x4 matrix is the standard structure for 3D transformations because it can handle both linear transformations (rotation, scaling) 
/// and affine transformations (translation, shearing, and perspective projections). 
/// It is commonly used in graphics pipelines, game engines, and 3D rendering systems.
/// 
/// Use Cases:
/// - Transforming objects in 3D space (position, orientation, and size).
/// - Combining multiple transformations (e.g., model-view-projection matrices).
/// - Applying translations, which require an extra dimension for homogeneous coordinates.
/// - Useful in animation, physics engines, and 3D rendering for full transformation control.
/// </remarks>
[Serializable]
[MemoryPackable]
public partial struct Fixed4x4 : IEquatable<Fixed4x4>, IFormattable
#if NET8_0_OR_GREATER
    , ISpanFormattable
#endif
{
    #region Static Readonly Fields

    /// <summary>
    /// Returns the identity matrix (diagonal elements set to 1).
    /// </summary>
    public static readonly Fixed4x4 Identity = new(
        Fixed64.One, Fixed64.Zero, Fixed64.Zero, Fixed64.Zero,
        Fixed64.Zero, Fixed64.One, Fixed64.Zero, Fixed64.Zero,
        Fixed64.Zero, Fixed64.Zero, Fixed64.One, Fixed64.Zero,
        Fixed64.Zero, Fixed64.Zero, Fixed64.Zero, Fixed64.One);

    /// <summary>
    /// Returns a matrix with all elements set to zero.
    /// </summary>
    public static readonly Fixed4x4 Zero = new(
        Fixed64.Zero, Fixed64.Zero, Fixed64.Zero, Fixed64.Zero,
        Fixed64.Zero, Fixed64.Zero, Fixed64.Zero, Fixed64.Zero,
        Fixed64.Zero, Fixed64.Zero, Fixed64.Zero, Fixed64.Zero,
        Fixed64.Zero, Fixed64.Zero, Fixed64.Zero, Fixed64.Zero);

    #endregion
    #region Fields and Constants

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
    /// <summary>
    /// Represents the element in the first row and fourth column of the matrix.
    /// </summary>
    [JsonInclude]
    [MemoryPackOrder(3)]
    public Fixed64 M14;

    // Second row

    /// <summary>
    /// Represents the element in the second row and first column of the matrix.
    /// </summary>
    [JsonInclude]
    [MemoryPackOrder(4)]
    public Fixed64 M21;
    /// <summary>
    /// Represents the element in the second row and second column of the matrix.
    /// </summary>
    [JsonInclude]
    [MemoryPackOrder(5)]
    public Fixed64 M22;
    /// <summary>
    /// Represents the element in the second row and third column of the matrix.
    /// </summary>
    [JsonInclude]
    [MemoryPackOrder(6)]
    public Fixed64 M23;
    /// <summary>
    /// Represents the element in the second row and fourth column of the matrix.
    /// </summary>
    [JsonInclude]
    [MemoryPackOrder(7)]
    public Fixed64 M24;

    // Third row

    /// <summary>
    /// Represents the element in the third row and first column of the matrix.
    /// </summary>
    [JsonInclude]
    [MemoryPackOrder(8)]
    public Fixed64 M31;
    /// <summary>
    /// Represents the element in the third row and second column of the matrix.
    /// </summary>
    [JsonInclude]
    [MemoryPackOrder(9)]
    public Fixed64 M32;
    /// <summary>
    /// Represents the element in the third row and third column of the matrix.
    /// </summary>
    [JsonInclude]
    [MemoryPackOrder(10)]
    public Fixed64 M33;
    /// <summary>
    /// Represents the element in the third row and fourth column of the matrix.
    /// </summary>
    [JsonInclude]
    [MemoryPackOrder(11)]
    public Fixed64 M34;

    // Fourth row

    /// <summary>
    /// Represents the element in the fourth row and first column of the matrix.
    /// </summary>
    [JsonInclude]
    [MemoryPackOrder(12)]
    public Fixed64 M41;
    /// <summary>
    /// Represents the element in the fourth row and second column of the matrix.
    /// </summary>
    [JsonInclude]
    [MemoryPackOrder(13)]
    public Fixed64 M42;
    /// <summary>
    /// Represents the element in the fourth row and third column of the matrix.
    /// </summary>
    [JsonInclude]
    [MemoryPackOrder(14)]
    public Fixed64 M43;
    /// <summary>
    /// Represents the element in the fourth row and fourth column of the matrix.
    /// </summary>
    [JsonInclude]
    [MemoryPackOrder(15)]
    public Fixed64 M44;

    #endregion
    #region Constructors

    /// <summary>
    /// Initializes a new FixedMatrix4x4 with individual elements.
    /// </summary>
    public Fixed4x4(
        Fixed64 m11, Fixed64 m12, Fixed64 m13, Fixed64 m14,
        Fixed64 m21, Fixed64 m22, Fixed64 m23, Fixed64 m24,
        Fixed64 m31, Fixed64 m32, Fixed64 m33, Fixed64 m34,
        Fixed64 m41, Fixed64 m42, Fixed64 m43, Fixed64 m44
    )
    {
        M11 = m11; M12 = m12; M13 = m13; M14 = m14;
        M21 = m21; M22 = m22; M23 = m23; M24 = m24;
        M31 = m31; M32 = m32; M33 = m33; M34 = m34;
        M41 = m41; M42 = m42; M43 = m43; M44 = m44;
    }

    /// <summary>
    /// Creates a matrix from four row vectors.
    /// </summary>
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public static Fixed4x4 FromRows(Vector4d row1, Vector4d row2, Vector4d row3, Vector4d row4)
    {
        return new Fixed4x4(
            row1.X, row1.Y, row1.Z, row1.W,
            row2.X, row2.Y, row2.Z, row2.W,
            row3.X, row3.Y, row3.Z, row3.W,
            row4.X, row4.Y, row4.Z, row4.W);
    }

    /// <summary>
    /// Creates a matrix from four column vectors.
    /// </summary>
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public static Fixed4x4 FromColumns(Vector4d column1, Vector4d column2, Vector4d column3, Vector4d column4)
    {
        return new Fixed4x4(
            column1.X, column2.X, column3.X, column4.X,
            column1.Y, column2.Y, column3.Y, column4.Y,
            column1.Z, column2.Z, column3.Z, column4.Z,
            column1.W, column2.W, column3.W, column4.W);
    }

    #endregion
    #region Properties

    /// <summary>
    /// Gets a value indicating whether the matrix represents an affine transformation.
    /// </summary>
    /// <remarks>
    /// An affine transformation is one where the bottom row is (0, 0, 0, 1), 
    /// allowing for efficient operations such as translation, scaling, rotation, and shearing without perspective distortion.
    /// </remarks>
    [JsonIgnore]
    [MemoryPackIgnore]
    public readonly bool IsAffine => M44 == Fixed64.One
        && M14 == Fixed64.Zero
        && M24 == Fixed64.Zero
        && M34 == Fixed64.Zero;

    /// <inheritdoc cref="ExtractTranslation(Fixed4x4)" />
    [JsonIgnore]
    [MemoryPackIgnore]
    public readonly Vector3d Translation => ExtractTranslation(this);

    /// <summary>
    /// Gets the right direction vector for this instance.
    /// </summary>
    [JsonIgnore]
    [MemoryPackIgnore]
    public readonly Vector3d Right => ExtractRight(this);

    /// <summary>
    /// Gets the left direction vector for this instance.
    /// </summary>
    [JsonIgnore]
    [MemoryPackIgnore]
    public readonly Vector3d Left => -ExtractRight(this);

    /// <summary>
    /// Gets the upward direction vector for this instance.
    /// </summary>
    [JsonIgnore]
    [MemoryPackIgnore]
    public readonly Vector3d Up => ExtractUp(this);

    /// <summary>
    /// Gets the downward direction vector for this instance.
    /// </summary>
    [JsonIgnore]
    [MemoryPackIgnore]
    public readonly Vector3d Down => -ExtractUp(this);

    /// <summary>
    /// Gets the forward direction vector for this instance.
    /// </summary>
    [JsonIgnore]
    [MemoryPackIgnore]
    public readonly Vector3d Forward => ExtractForward(this);

    /// <summary>
    /// Gets the backward direction vector for this instance.
    /// </summary>
    [JsonIgnore]
    [MemoryPackIgnore]
    public readonly Vector3d Backward => -ExtractForward(this);

    /// <inheritdoc cref="ExtractScale(Fixed4x4)" />
    [JsonIgnore]
    [MemoryPackIgnore]
    public readonly Vector3d Scale => ExtractScale(this);

    /// <inheritdoc cref="ExtractRotation(Fixed4x4)" />
    [JsonIgnore]
    [MemoryPackIgnore]
    public readonly FixedQuaternion Rotation => ExtractRotation(this);

    /// <summary>
    /// Gets or sets the matrix element at the specified linear index.
    /// </summary>
    /// <remarks>Matrix elements are indexed in row-major order from 0 to 15.</remarks>
    /// <param name="index">The zero-based linear index of the matrix element to get or set. Must be in the range 0 to 15.</param>
    /// <returns>The matrix element at the specified index.</returns>
    /// <exception cref="IndexOutOfRangeException">Thrown when the specified index is less than 0 or greater than 15.</exception>
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
                3 => M41,
                4 => M12,
                5 => M22,
                6 => M32,
                7 => M42,
                8 => M13,
                9 => M23,
                10 => M33,
                11 => M43,
                12 => M14,
                13 => M24,
                14 => M34,
                15 => M44,
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
                case 3:
                    M41 = value;
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
                case 7:
                    M42 = value;
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
                case 11:
                    M43 = value;
                    break;
                case 12:
                    M14 = value;
                    break;
                case 13:
                    M24 = value;
                    break;
                case 14:
                    M34 = value;
                    break;
                case 15:
                    M44 = value;
                    break;
                default:
                    throw new IndexOutOfRangeException("Invalid matrix index!");
            }
        }
    }

    #endregion
    #region Methods (Instance)

    /// <summary>
    /// Calculates the determinant of a 4x4 matrix.
    /// </summary>
    public Fixed64 GetDeterminant()
    {
        if (IsAffine)
        {
            return M11 * (M22 * M33 - M23 * M32)
                 - M12 * (M21 * M33 - M23 * M31)
                 + M13 * (M21 * M32 - M22 * M31);
        }

        // Process as full 4x4 matrix
        Fixed64 minor0 = M33 * M44 - M34 * M43;
        Fixed64 minor1 = M32 * M44 - M34 * M42;
        Fixed64 minor2 = M32 * M43 - M33 * M42;
        Fixed64 cofactor0 = M31 * M44 - M34 * M41;
        Fixed64 cofactor1 = M31 * M43 - M33 * M41;
        Fixed64 cofactor2 = M31 * M42 - M32 * M41;
        return M11 * (M22 * minor0 - M23 * minor1 + M24 * minor2)
            - M12 * (M21 * minor0 - M23 * cofactor0 + M24 * cofactor1)
            + M13 * (M21 * minor1 - M22 * cofactor0 + M24 * cofactor2)
            - M14 * (M21 * minor2 - M22 * cofactor1 + M23 * cofactor2);
    }

    /// <inheritdoc cref="Fixed4x4.ResetScaleToIdentity(Fixed4x4)" />
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public Fixed4x4 ResetScaleToIdentity() => this = ResetScaleToIdentity(this);

    /// <summary>
    /// Sets the translation, scale, and rotation components onto the matrix.
    /// </summary>
    /// <param name="translation">The translation vector.</param>
    /// <param name="scale">The scale vector.</param>
    /// <param name="rotation">The rotation quaternion.</param>
    public void SetTransform(Vector3d translation, FixedQuaternion rotation, Vector3d scale) =>
        this = CreateTransform(translation, rotation, scale);

    #endregion
}
