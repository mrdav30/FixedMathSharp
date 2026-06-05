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
public partial struct Fixed4x4 : IEquatable<Fixed4x4>
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
        this.M11 = m11; this.M12 = m12; this.M13 = m13; this.M14 = m14;
        this.M21 = m21; this.M22 = m22; this.M23 = m23; this.M24 = m24;
        this.M31 = m31; this.M32 = m32; this.M33 = m33; this.M34 = m34;
        this.M41 = m41; this.M42 = m42; this.M43 = m43; this.M44 = m44;
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

    #region Static Matrix Generators and Transformations

    /// <summary>
    /// Creates a translation matrix from the specified 3-dimensional vector.
    /// </summary>
    /// <param name="position"></param>
    /// <returns>The translation matrix.</returns>
    public static Fixed4x4 CreateTranslation(Vector3d position)
    {
        Fixed4x4 result = default;
        result.M11 = Fixed64.One;
        result.M12 = Fixed64.Zero;
        result.M13 = Fixed64.Zero;
        result.M14 = Fixed64.Zero;
        result.M21 = Fixed64.Zero;
        result.M22 = Fixed64.One;
        result.M23 = Fixed64.Zero;
        result.M24 = Fixed64.Zero;
        result.M31 = Fixed64.Zero;
        result.M32 = Fixed64.Zero;
        result.M33 = Fixed64.One;
        result.M34 = Fixed64.Zero;
        result.M41 = position.X;
        result.M42 = position.Y;
        result.M43 = position.Z;
        result.M44 = Fixed64.One;
        return result;
    }

    /// <summary>
    /// Creates a translation matrix from the specified coordinates.
    /// </summary>
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public static Fixed4x4 CreateTranslation(Fixed64 x, Fixed64 y, Fixed64 z) =>
        CreateTranslation(new Vector3d(x, y, z));

    /// <summary>
    /// Creates a rotation matrix from a quaternion.
    /// </summary>
    /// <param name="rotation">The quaternion representing the rotation.</param>
    /// <returns>A 4x4 matrix representing the rotation.</returns>
    public static Fixed4x4 CreateRotation(FixedQuaternion rotation)
    {
        Fixed3x3 rotationMatrix = rotation.ToMatrix3x3();

        return new Fixed4x4(
            rotationMatrix.M11, rotationMatrix.M12, rotationMatrix.M13, Fixed64.Zero,
            rotationMatrix.M21, rotationMatrix.M22, rotationMatrix.M23, Fixed64.Zero,
            rotationMatrix.M31, rotationMatrix.M32, rotationMatrix.M33, Fixed64.Zero,
            Fixed64.Zero, Fixed64.Zero, Fixed64.Zero, Fixed64.One);
    }

    /// <summary>
    /// Creates a rotation matrix around the X axis.
    /// </summary>
    public static Fixed4x4 CreateRotationX(Fixed64 angle) =>
        FromRotationMatrix(Fixed3x3.CreateRotationX(angle));

    /// <summary>
    /// Creates a rotation matrix around the Y axis.
    /// </summary>
    public static Fixed4x4 CreateRotationY(Fixed64 angle) =>
        FromRotationMatrix(Fixed3x3.CreateRotationY(angle));

    /// <summary>
    /// Creates a rotation matrix around the Z axis.
    /// </summary>
    public static Fixed4x4 CreateRotationZ(Fixed64 angle) =>
        FromRotationMatrix(Fixed3x3.CreateRotationZ(angle));

    /// <summary>
    /// Creates a rotation matrix from an axis and angle.
    /// </summary>
    public static Fixed4x4 CreateFromAxisAngle(Vector3d axis, Fixed64 angle) =>
        CreateRotation(FixedQuaternion.FromAxisAngle(axis, angle));

    /// <summary>
    /// Creates a rotation matrix from pitch, yaw, and roll angles in radians.
    /// </summary>
    public static Fixed4x4 CreateFromEulerAngles(Fixed64 pitch, Fixed64 yaw, Fixed64 roll) =>
        CreateRotation(FixedQuaternion.FromEulerAngles(pitch, yaw, roll));

    /// <summary>
    /// Creates a scale matrix from a 3-dimensional vector.
    /// </summary>
    /// <param name="scale">The vector representing the scale along each axis.</param>
    /// <returns>A 4x4 matrix representing the scale transformation.</returns>
    public static Fixed4x4 CreateScale(Vector3d scale) =>
        new(
            scale.X, Fixed64.Zero, Fixed64.Zero, Fixed64.Zero,
            Fixed64.Zero, scale.Y, Fixed64.Zero, Fixed64.Zero,
            Fixed64.Zero, Fixed64.Zero, scale.Z, Fixed64.Zero,
            Fixed64.Zero, Fixed64.Zero, Fixed64.Zero, Fixed64.One);

    /// <summary>
    /// Creates a uniform scale matrix.
    /// </summary>
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public static Fixed4x4 CreateScale(Fixed64 scale) => CreateScale(new Vector3d(scale, scale, scale));

    /// <summary>
    /// Creates a non-uniform scale matrix from individual scale components.
    /// </summary>
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public static Fixed4x4 CreateScale(Fixed64 x, Fixed64 y, Fixed64 z) => CreateScale(new Vector3d(x, y, z));

    /// <summary>
    /// Creates a view matrix looking from a camera position toward a target.
    /// </summary>
    public static Fixed4x4 CreateLookAt(Vector3d cameraPosition, Vector3d cameraTarget, Vector3d cameraUpVector)
    {
        Vector3d forward = cameraTarget - cameraPosition;

        if (forward.SqrMagnitude == Fixed64.Zero)
            throw new ArgumentException("Camera position and target must be different.");

        forward = forward.Normalize();
        Vector3d right = Vector3d.Cross(cameraUpVector, forward);

        if (right.SqrMagnitude == Fixed64.Zero)
            throw new ArgumentException("Camera up vector must not be parallel to the view direction.");

        right = right.Normalize();
        Vector3d up = Vector3d.Cross(forward, right).Normalize();

        return new Fixed4x4(
            right.X, right.Y, right.Z, Fixed64.Zero,
            up.X, up.Y, up.Z, Fixed64.Zero,
            forward.X, forward.Y, forward.Z, Fixed64.Zero,
            -Vector3d.Dot(right, cameraPosition),
            -Vector3d.Dot(up, cameraPosition),
            -Vector3d.Dot(forward, cameraPosition),
            Fixed64.One);
    }

    /// <summary>
    /// Creates an orthographic projection matrix centered on the origin.
    /// </summary>
    public static Fixed4x4 CreateOrthographic(Fixed64 width, Fixed64 height, Fixed64 zNearPlane, Fixed64 zFarPlane)
    {
        if (width <= Fixed64.Zero)
            throw new ArgumentOutOfRangeException(nameof(width), "Width must be greater than zero.");

        if (height <= Fixed64.Zero)
            throw new ArgumentOutOfRangeException(nameof(height), "Height must be greater than zero.");

        Fixed64 halfWidth = width * Fixed64.Half;
        Fixed64 halfHeight = height * Fixed64.Half;
        return CreateOrthographicOffCenter(-halfWidth, halfWidth, -halfHeight, halfHeight, zNearPlane, zFarPlane);
    }

    /// <summary>
    /// Creates an off-center orthographic projection matrix.
    /// </summary>
    public static Fixed4x4 CreateOrthographicOffCenter(
        Fixed64 left,
        Fixed64 right,
        Fixed64 bottom,
        Fixed64 top,
        Fixed64 zNearPlane,
        Fixed64 zFarPlane)
    {
        if (left == right)
            throw new ArgumentOutOfRangeException(nameof(right), "Right must be different from left.");

        if (bottom == top)
            throw new ArgumentOutOfRangeException(nameof(top), "Top must be different from bottom.");

        ValidateDepthRange(zNearPlane, zFarPlane);

        Fixed64 width = right - left;
        Fixed64 height = top - bottom;
        Fixed64 depth = zFarPlane - zNearPlane;

        return new Fixed4x4(
            Fixed64.Two / width, Fixed64.Zero, Fixed64.Zero, Fixed64.Zero,
            Fixed64.Zero, Fixed64.Two / height, Fixed64.Zero, Fixed64.Zero,
            Fixed64.Zero, Fixed64.Zero, Fixed64.One / depth, Fixed64.Zero,
            (left + right) / (left - right),
            (top + bottom) / (bottom - top),
            -zNearPlane / depth,
            Fixed64.One);
    }

    /// <summary>
    /// Creates a perspective projection matrix centered on the near plane.
    /// </summary>
    public static Fixed4x4 CreatePerspective(
        Fixed64 width,
        Fixed64 height,
        Fixed64 nearPlaneDistance,
        Fixed64 farPlaneDistance)
    {
        if (width <= Fixed64.Zero)
            throw new ArgumentOutOfRangeException(nameof(width), "Width must be greater than zero.");

        if (height <= Fixed64.Zero)
            throw new ArgumentOutOfRangeException(nameof(height), "Height must be greater than zero.");

        ValidatePerspectiveDepthRange(nearPlaneDistance, farPlaneDistance);

        Fixed64 depth = farPlaneDistance - nearPlaneDistance;
        Fixed64 twoNear = Fixed64.Two * nearPlaneDistance;

        return new Fixed4x4(
            twoNear / width, Fixed64.Zero, Fixed64.Zero, Fixed64.Zero,
            Fixed64.Zero, twoNear / height, Fixed64.Zero, Fixed64.Zero,
            Fixed64.Zero, Fixed64.Zero, farPlaneDistance / depth, Fixed64.One,
            Fixed64.Zero, Fixed64.Zero, -(nearPlaneDistance * farPlaneDistance) / depth, Fixed64.Zero);
    }

    /// <summary>
    /// Creates a perspective projection matrix from a vertical field of view.
    /// </summary>
    public static Fixed4x4 CreatePerspectiveFieldOfView(
        Fixed64 fieldOfView,
        Fixed64 aspectRatio,
        Fixed64 nearPlaneDistance,
        Fixed64 farPlaneDistance)
    {
        if (fieldOfView <= Fixed64.Zero || fieldOfView >= Fixed64.Pi)
            throw new ArgumentOutOfRangeException(nameof(fieldOfView), "Field of view must be greater than zero and less than PI.");

        if (aspectRatio <= Fixed64.Zero)
            throw new ArgumentOutOfRangeException(nameof(aspectRatio), "Aspect ratio must be greater than zero.");

        ValidatePerspectiveDepthRange(nearPlaneDistance, farPlaneDistance);

        Fixed64 yScale = Fixed64.One / FixedMath.Tan(fieldOfView * Fixed64.Half);
        Fixed64 xScale = yScale / aspectRatio;
        Fixed64 depth = farPlaneDistance - nearPlaneDistance;

        return new Fixed4x4(
            xScale, Fixed64.Zero, Fixed64.Zero, Fixed64.Zero,
            Fixed64.Zero, yScale, Fixed64.Zero, Fixed64.Zero,
            Fixed64.Zero, Fixed64.Zero, farPlaneDistance / depth, Fixed64.One,
            Fixed64.Zero, Fixed64.Zero, -(nearPlaneDistance * farPlaneDistance) / depth, Fixed64.Zero);
    }

    /// <summary>
    /// Creates an off-center perspective projection matrix.
    /// </summary>
    public static Fixed4x4 CreatePerspectiveOffCenter(
        Fixed64 left,
        Fixed64 right,
        Fixed64 bottom,
        Fixed64 top,
        Fixed64 nearPlaneDistance,
        Fixed64 farPlaneDistance)
    {
        if (left == right)
            throw new ArgumentOutOfRangeException(nameof(right), "Right must be different from left.");

        if (bottom == top)
            throw new ArgumentOutOfRangeException(nameof(top), "Top must be different from bottom.");

        ValidatePerspectiveDepthRange(nearPlaneDistance, farPlaneDistance);

        Fixed64 width = right - left;
        Fixed64 height = top - bottom;
        Fixed64 depth = farPlaneDistance - nearPlaneDistance;
        Fixed64 twoNear = Fixed64.Two * nearPlaneDistance;

        return new Fixed4x4(
            twoNear / width, Fixed64.Zero, Fixed64.Zero, Fixed64.Zero,
            Fixed64.Zero, twoNear / height, Fixed64.Zero, Fixed64.Zero,
            (left + right) / (left - right),
            (top + bottom) / (bottom - top),
            farPlaneDistance / depth,
            Fixed64.One,
            Fixed64.Zero, Fixed64.Zero, -(nearPlaneDistance * farPlaneDistance) / depth, Fixed64.Zero);
    }

    /// <summary>
    /// Creates a world matrix from a position and orientation basis.
    /// </summary>
    public static Fixed4x4 CreateWorld(Vector3d position, Vector3d forward, Vector3d up)
    {
        if (forward.SqrMagnitude == Fixed64.Zero)
            throw new ArgumentException("Forward vector must be non-zero.");

        forward = forward.Normalize();
        Vector3d right = Vector3d.Cross(up, forward);

        if (right.SqrMagnitude == Fixed64.Zero)
            throw new ArgumentException("Up vector must not be parallel to forward.");

        right = right.Normalize();
        up = Vector3d.Cross(forward, right).Normalize();

        return new Fixed4x4(
            right.X, right.Y, right.Z, Fixed64.Zero,
            up.X, up.Y, up.Z, Fixed64.Zero,
            forward.X, forward.Y, forward.Z, Fixed64.Zero,
            position.X, position.Y, position.Z, Fixed64.One);
    }

    /// <summary>
    /// Constructs a transformation matrix from translation, scale, and rotation.
    /// This method ensures that the rotation is properly normalized, applies the scale to the
    /// rotational basis, and sets the translation component separately.
    /// </summary>
    /// <remarks>
    /// - Uses a normalized rotation matrix to maintain numerical stability.
    /// - Applies non-uniform scaling to the rotation before setting translation.
    /// - Preferred when ensuring transformations remain mathematically correct.
    /// - If the rotation is already normalized and combined transformations are needed, consider using <see cref="ScaleRotateTranslate"/>.
    /// </remarks>
    /// <param name="translation">The translation vector.</param>
    /// <param name="scale">The scale vector.</param>
    /// <param name="rotation">The rotation quaternion.</param>
    /// <returns>A transformation matrix incorporating translation, rotation, and scale.</returns>
    public static Fixed4x4 CreateTransform(Vector3d translation, FixedQuaternion rotation, Vector3d scale)
    {
        Fixed3x3 rotationMatrix = rotation.ToMatrix3x3();

        return new Fixed4x4(
            rotationMatrix.M11 * scale.X, rotationMatrix.M12 * scale.X, rotationMatrix.M13 * scale.X, Fixed64.Zero,
            rotationMatrix.M21 * scale.Y, rotationMatrix.M22 * scale.Y, rotationMatrix.M23 * scale.Y, Fixed64.Zero,
            rotationMatrix.M31 * scale.Z, rotationMatrix.M32 * scale.Z, rotationMatrix.M33 * scale.Z, Fixed64.Zero,
            translation.X, translation.Y, translation.Z, Fixed64.One);
    }

    /// <summary>
    /// Constructs a transformation matrix from translation, rotation, and scale by multiplying
    /// separate matrices in the order: Scale * Rotation * Translation.
    /// </summary>
    /// <remarks>
    /// - This method directly multiplies the scale, rotation, and translation matrices.
    /// - Ensures that scale is applied first to preserve correct axis scaling.
    /// - Then rotation is applied so that rotation is not affected by non-uniform scaling.
    /// - Finally, translation moves the object to its correct world position.
    /// </remarks>
    public static Fixed4x4 ScaleRotateTranslate(Vector3d translation, FixedQuaternion rotation, Vector3d scale)
    {
        return CreateTransform(translation, rotation, scale);
    }

    /// <summary>
    /// Constructs a transformation matrix from translation, rotation, and scale by multiplying
    /// matrices in the order: Translation * Rotation * Scale (T * R * S).
    /// </summary>
    /// <remarks>
    /// - Use this method when transformations need to be applied **relative to an object's local origin**.
    /// - Example use cases include **animation systems**, **hierarchical transformations**, and **UI transformations**.
    /// - If you need to apply world-space transformations, use <see cref="CreateTransform"/> instead.
    /// </remarks>
    public static Fixed4x4 TranslateRotateScale(Vector3d translation, FixedQuaternion rotation, Vector3d scale)
    {
        Fixed3x3 rotationMatrix = rotation.ToMatrix3x3();

        return new Fixed4x4(
            rotationMatrix.M11 * scale.X, rotationMatrix.M12 * scale.Y, rotationMatrix.M13 * scale.Z, Fixed64.Zero,
            rotationMatrix.M21 * scale.X, rotationMatrix.M22 * scale.Y, rotationMatrix.M23 * scale.Z, Fixed64.Zero,
            rotationMatrix.M31 * scale.X, rotationMatrix.M32 * scale.Y, rotationMatrix.M33 * scale.Z, Fixed64.Zero,
            (translation.X * rotationMatrix.M11 + translation.Y * rotationMatrix.M21 + translation.Z * rotationMatrix.M31) * scale.X,
            (translation.X * rotationMatrix.M12 + translation.Y * rotationMatrix.M22 + translation.Z * rotationMatrix.M32) * scale.Y,
            (translation.X * rotationMatrix.M13 + translation.Y * rotationMatrix.M23 + translation.Z * rotationMatrix.M33) * scale.Z,
            Fixed64.One);
    }

    #endregion

    #region Decomposition, Extraction, and Setters

    /// <summary>
    /// Extracts the translation component from the 4x4 matrix.
    /// </summary>
    /// <param name="matrix">The matrix from which to extract the translation.</param>
    /// <returns>A Vector3d representing the translation component.</returns>
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public static Vector3d ExtractTranslation(Fixed4x4 matrix) => new(matrix.M41, matrix.M42, matrix.M43);

    /// <summary>
    /// Extracts the right direction from the 4x4 matrix.
    /// </summary>
    public static Vector3d ExtractRight(Fixed4x4 matrix) => new Vector3d(matrix.M11, matrix.M12, matrix.M13).Normalize();

    /// <summary>
    /// Extracts the up direction from the 4x4 matrix.
    /// </summary>
    /// <remarks>
    /// This is the surface normal if the matrix represents ground orientation.
    /// </remarks>
    /// <param name="matrix"></param>
    /// <returns>A <see cref="Vector3d"/> representing the up direction.</returns>
    public static Vector3d ExtractUp(Fixed4x4 matrix) => new Vector3d(matrix.M21, matrix.M22, matrix.M23).Normalize();

    /// <summary>
    /// Extracts the forward direction from the 4x4 matrix.
    /// </summary>
    public static Vector3d ExtractForward(Fixed4x4 matrix) => new Vector3d(matrix.M31, matrix.M32, matrix.M33).Normalize();

    /// <summary>
    /// Extracts the scaling factors from the matrix by calculating the magnitudes of the basis vectors (non-lossy).
    /// </summary>
    /// <returns>A Vector3d representing the precise scale along the X, Y, and Z axes.</returns>
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public static Vector3d ExtractScale(Fixed4x4 matrix) =>
        new(
            new Vector3d(matrix.M11, matrix.M12, matrix.M13).Magnitude,
            new Vector3d(matrix.M21, matrix.M22, matrix.M23).Magnitude,
            new Vector3d(matrix.M31, matrix.M32, matrix.M33).Magnitude);

    /// <summary>
    /// Extracts the scaling factors from the matrix by returning the diagonal elements (lossy).
    /// </summary>
    /// <returns>A Vector3d representing the scale along X, Y, and Z axes (lossy).</returns>
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public static Vector3d ExtractLossyScale(Fixed4x4 matrix) => new(matrix.M11, matrix.M22, matrix.M33);

    /// <summary>
    /// Extracts the rotation component from the 4x4 matrix by normalizing the rotation matrix.
    /// </summary>
    /// <param name="matrix">The matrix from which to extract the rotation.</param>
    /// <returns>A FixedQuaternion representing the rotation component.</returns>
    public static FixedQuaternion ExtractRotation(Fixed4x4 matrix)
    {
        Vector3d scale = ExtractScale(matrix);

        // prevent divide by zero exception
        Fixed64 scaleX = scale.X == Fixed64.Zero ? Fixed64.One : scale.X;
        Fixed64 scaleY = scale.Y == Fixed64.Zero ? Fixed64.One : scale.Y;
        Fixed64 scaleZ = scale.Z == Fixed64.Zero ? Fixed64.One : scale.Z;

        Fixed4x4 normalizedMatrix = new(
            matrix.M11 / scaleX, matrix.M12 / scaleX, matrix.M13 / scaleX, Fixed64.Zero,
            matrix.M21 / scaleY, matrix.M22 / scaleY, matrix.M23 / scaleY, Fixed64.Zero,
            matrix.M31 / scaleZ, matrix.M32 / scaleZ, matrix.M33 / scaleZ, Fixed64.Zero,
            Fixed64.Zero, Fixed64.Zero, Fixed64.Zero, Fixed64.One
        );

        return FixedQuaternion.FromMatrix(normalizedMatrix);
    }

    /// <summary>
    /// Decomposes a 4x4 matrix into its translation, scale, and rotation components.
    /// </summary>
    /// <param name="matrix">The 4x4 matrix to decompose.</param>
    /// <param name="scale">The extracted scale component.</param>
    /// <param name="rotation">The extracted rotation component as a quaternion.</param>
    /// <param name="translation">The extracted translation component.</param>
    /// <returns>True if decomposition was successful, otherwise false.</returns>
    public static bool Decompose(
        Fixed4x4 matrix,
        out Vector3d scale,
        out FixedQuaternion rotation,
        out Vector3d translation)
    {
        // Extract scale by calculating the magnitudes of the basis vectors
        scale = ExtractScale(matrix);

        // prevent divide by zero exception
        scale = new Vector3d(
             scale.X == Fixed64.Zero ? Fixed64.One : scale.X,
             scale.Y == Fixed64.Zero ? Fixed64.One : scale.Y,
             scale.Z == Fixed64.Zero ? Fixed64.One : scale.Z);

        // normalize rotation and scaling
        Fixed4x4 normalizedMatrix = ApplyScaleToRotation(matrix, Vector3d.One / scale);

        // Extract translation
        translation = new Vector3d(normalizedMatrix.M41, normalizedMatrix.M42, normalizedMatrix.M43);

        // Check the determinant to ensure correct handedness
        Fixed64 determinant = normalizedMatrix.GetDeterminant();
        if (determinant < Fixed64.Zero)
        {
            // Adjust for left-handed coordinate system by flipping one of the axes
            scale.X = -scale.X;
            normalizedMatrix.M11 = -normalizedMatrix.M11;
            normalizedMatrix.M12 = -normalizedMatrix.M12;
            normalizedMatrix.M13 = -normalizedMatrix.M13;
        }

        // Extract the rotation component from the orthogonalized matrix
        rotation = FixedQuaternion.FromMatrix(normalizedMatrix);

        return true;
    }

    /// <summary>
    /// Sets the translation component of the 4x4 matrix.
    /// </summary>
    /// <param name="matrix">The matrix to modify.</param>
    /// <param name="translation">The new translation vector.</param>
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public static Fixed4x4 SetTranslation(Fixed4x4 matrix, Vector3d translation)
    {
        matrix.M41 = translation.X;
        matrix.M42 = translation.Y;
        matrix.M43 = translation.Z;
        return matrix;
    }

    /// <summary>
    /// Sets the scale component of the 4x4 matrix by assigning the provided scale vector to the matrix's diagonal elements.
    /// </summary>
    /// <param name="matrix">The matrix to modify. Typically an identity or transformation matrix.</param>
    /// <param name="scale">The new scale vector to apply along the X, Y, and Z axes.</param>
    /// <remarks>
    /// Best used for applying scale to an identity matrix or resetting the scale on an existing matrix.
    /// For non-uniform scaling in combination with rotation, use <see cref="ApplyScaleToRotation"/>.
    /// </remarks>
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public static Fixed4x4 SetScale(Fixed4x4 matrix, Vector3d scale)
    {
        matrix.M11 = scale.X;
        matrix.M22 = scale.Y;
        matrix.M33 = scale.Z;
        return matrix;
    }

    /// <summary>
    /// Applies non-uniform scaling to the 4x4 matrix by multiplying the scale vector with the rotation matrix's basis vectors.
    /// </summary>
    /// <param name="matrix">The matrix to modify. Should already contain a valid rotation component.</param>
    /// <param name="scale">The scale vector to apply along the X, Y, and Z axes.</param>
    /// <remarks>
    /// Use this method when scaling is required in combination with an existing rotation, ensuring proper axis alignment.
    /// </remarks>
    public static Fixed4x4 ApplyScaleToRotation(Fixed4x4 matrix, Vector3d scale)
    {
        // Scale each row of the rotation matrix
        matrix.M11 *= scale.X;
        matrix.M12 *= scale.X;
        matrix.M13 *= scale.X;

        matrix.M21 *= scale.Y;
        matrix.M22 *= scale.Y;
        matrix.M23 *= scale.Y;

        matrix.M31 *= scale.Z;
        matrix.M32 *= scale.Z;
        matrix.M33 *= scale.Z;

        return matrix;
    }

    /// <summary>
    /// Resets the scaling part of the matrix to identity (1,1,1).
    /// </summary>
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public static Fixed4x4 ResetScaleToIdentity(Fixed4x4 matrix)
    {
        matrix.M11 = Fixed64.One;  // X scale
        matrix.M22 = Fixed64.One;  // Y scale
        matrix.M33 = Fixed64.One;  // Z scale

        return matrix;
    }

    /// <summary>
    /// Sets the global scale of an object using a 4x4 transformation matrix.
    /// </summary>
    /// <param name="matrix">The transformation matrix representing the object's global state.</param>
    /// <param name="globalScale">The desired global scale as a vector.</param>
    /// <remarks>
    /// The method extracts the current global scale from the matrix and computes the new local scale 
    /// by dividing the desired global scale by the current global scale. 
    /// The new local scale is then applied to the matrix.
    /// </remarks>
    public static Fixed4x4 SetGlobalScale(Fixed4x4 matrix, Vector3d globalScale)
    {
        // normalize the matrix to avoid drift in the rotation component
        matrix = NormalizeRotationMatrix(matrix);

        // Reset the local scaling portion of the matrix
        matrix.ResetScaleToIdentity();

        // Compute the new local scale by dividing the desired global scale by the current scale (which was reset to (1, 1, 1))
        Vector3d newLocalScale = new(
           globalScale.X / Fixed64.One,
           globalScale.Y / Fixed64.One,
           globalScale.Z / Fixed64.One
        );

        // Apply the new local scale directly to the matrix
        return ApplyScaleToRotation(matrix, newLocalScale);
    }

    /// <summary>
    /// Replaces the rotation component of the 4x4 matrix using the provided quaternion, without affecting the translation component.
    /// </summary>
    /// <param name="matrix">The matrix to modify. The rotation will replace the upper-left 3x3 portion of the matrix.</param>
    /// <param name="rotation">The quaternion representing the new rotation to apply.</param>
    /// <remarks>
    /// This method preserves the matrix's translation component. For complete transformation updates, use <see cref="SetTransform"/>.
    /// </remarks>
    public static Fixed4x4 SetRotation(Fixed4x4 matrix, FixedQuaternion rotation)
    {
        Fixed3x3 rotationMatrix = rotation.ToMatrix3x3();

        Vector3d scale = ExtractScale(matrix);

        // Apply rotation to the upper-left 3x3 matrix

        matrix.M11 = rotationMatrix.M11 * scale.X;
        matrix.M12 = rotationMatrix.M12 * scale.X;
        matrix.M13 = rotationMatrix.M13 * scale.X;

        matrix.M21 = rotationMatrix.M21 * scale.Y;
        matrix.M22 = rotationMatrix.M22 * scale.Y;
        matrix.M23 = rotationMatrix.M23 * scale.Y;

        matrix.M31 = rotationMatrix.M31 * scale.Z;
        matrix.M32 = rotationMatrix.M32 * scale.Z;
        matrix.M33 = rotationMatrix.M33 * scale.Z;

        return matrix;
    }

    /// <summary>
    /// Normalizes the rotation component of a 4x4 matrix by ensuring the basis vectors are orthogonal and unit length.
    /// </summary>
    /// <remarks>
    /// This method recalculates the X, Y, and Z basis vectors from the upper-left 3x3 portion of the matrix, ensuring they are orthogonal and normalized. 
    /// The remaining components of the matrix are reset to maintain a valid transformation structure.
    /// 
    /// Use Cases:
    /// - Ensuring the rotation component remains stable and accurate after multiple transformations.
    /// - Used in 3D transformations to prevent numerical drift from affecting the orientation over time.
    /// - Essential for cases where precise orientation is required, such as animations or physics simulations.
    /// </remarks>
    public static Fixed4x4 NormalizeRotationMatrix(Fixed4x4 matrix)
    {
        Vector3d basisX = new Vector3d(matrix.M11, matrix.M12, matrix.M13).Normalize();
        Vector3d basisY = new Vector3d(matrix.M21, matrix.M22, matrix.M23).Normalize();
        Vector3d basisZ = new Vector3d(matrix.M31, matrix.M32, matrix.M33).Normalize();

        return new Fixed4x4(
            basisX.X, basisX.Y, basisX.Z, Fixed64.Zero,
            basisY.X, basisY.Y, basisY.Z, Fixed64.Zero,
            basisZ.X, basisZ.Y, basisZ.Z, Fixed64.Zero,
            Fixed64.Zero, Fixed64.Zero, Fixed64.Zero, Fixed64.One
        );
    }

    #endregion

    #region Static Matrix Operators

    /// <summary>
    /// Linearly interpolates between two matrices component-wise.
    /// </summary>
    public static Fixed4x4 Lerp(Fixed4x4 a, Fixed4x4 b, Fixed64 t) =>
         new(
            Fixed64.Lerp(a.M11, b.M11, t),
            Fixed64.Lerp(a.M12, b.M12, t),
            Fixed64.Lerp(a.M13, b.M13, t),
            Fixed64.Lerp(a.M14, b.M14, t),
            Fixed64.Lerp(a.M21, b.M21, t),
            Fixed64.Lerp(a.M22, b.M22, t),
            Fixed64.Lerp(a.M23, b.M23, t),
            Fixed64.Lerp(a.M24, b.M24, t),
            Fixed64.Lerp(a.M31, b.M31, t),
            Fixed64.Lerp(a.M32, b.M32, t),
            Fixed64.Lerp(a.M33, b.M33, t),
            Fixed64.Lerp(a.M34, b.M34, t),
            Fixed64.Lerp(a.M41, b.M41, t),
            Fixed64.Lerp(a.M42, b.M42, t),
            Fixed64.Lerp(a.M43, b.M43, t),
            Fixed64.Lerp(a.M44, b.M44, t));

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

    #region Equality and HashCode Overrides

    /// <inheritdoc/>
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public override bool Equals(object? obj) => obj is Fixed4x4 x && Equals(x);

    /// <inheritdoc/>
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public bool Equals(Fixed4x4 other) =>
        M11 == other.M11 && M12 == other.M12 && M13 == other.M13 && M14 == other.M14 &&
        M21 == other.M21 && M22 == other.M22 && M23 == other.M23 && M24 == other.M24 &&
        M31 == other.M31 && M32 == other.M32 && M33 == other.M33 && M34 == other.M34 &&
        M41 == other.M41 && M42 == other.M42 && M43 == other.M43 && M44 == other.M44;

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
            hash = hash * 23 + M14.GetHashCode();
            hash = hash * 23 + M21.GetHashCode();
            hash = hash * 23 + M22.GetHashCode();
            hash = hash * 23 + M23.GetHashCode();
            hash = hash * 23 + M24.GetHashCode();
            hash = hash * 23 + M31.GetHashCode();
            hash = hash * 23 + M32.GetHashCode();
            hash = hash * 23 + M33.GetHashCode();
            hash = hash * 23 + M34.GetHashCode();
            hash = hash * 23 + M41.GetHashCode();
            hash = hash * 23 + M42.GetHashCode();
            hash = hash * 23 + M43.GetHashCode();
            hash = hash * 23 + M44.GetHashCode();
            return hash;
        }
    }

    #endregion    

    #region Private Helpers

    private static Fixed4x4 FromRotationMatrix(Fixed3x3 matrix) =>
        new(
            matrix.M11, matrix.M12, matrix.M13, Fixed64.Zero,
            matrix.M21, matrix.M22, matrix.M23, Fixed64.Zero,
            matrix.M31, matrix.M32, matrix.M33, Fixed64.Zero,
            Fixed64.Zero, Fixed64.Zero, Fixed64.Zero, Fixed64.One);

    private static void ValidateDepthRange(Fixed64 nearPlaneDistance, Fixed64 farPlaneDistance)
    {
        if (nearPlaneDistance < Fixed64.Zero)
            throw new ArgumentOutOfRangeException(nameof(nearPlaneDistance), "Near plane distance must be greater than or equal to zero.");

        if (farPlaneDistance <= nearPlaneDistance)
            throw new ArgumentOutOfRangeException(nameof(farPlaneDistance), "Far plane distance must be greater than or equal to zero.");
    }

    private static void ValidatePerspectiveDepthRange(Fixed64 nearPlaneDistance, Fixed64 farPlaneDistance)
    {
        if (nearPlaneDistance <= Fixed64.Zero)
            throw new ArgumentOutOfRangeException(nameof(nearPlaneDistance), "Near plane distance must be greater than zero.");

        if (farPlaneDistance <= nearPlaneDistance)
            throw new ArgumentOutOfRangeException(nameof(farPlaneDistance), "Far plane distance must be greater than near plane distance.");
    }

    #endregion
}
