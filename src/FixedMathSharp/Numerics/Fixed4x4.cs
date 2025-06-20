using MessagePack;
using System;
using System.Runtime.CompilerServices;

namespace FixedMathSharp
{
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
    [MessagePackObject]
    public struct Fixed4x4 : IEquatable<Fixed4x4>
    {
        #region Fields and Constants

        [Key(0)]
        public Fixed64 m00;
        [Key(1)]
        public Fixed64 m01;
        [Key(2)]
        public Fixed64 m02;
        [Key(3)]
        public Fixed64 m03;

        [Key(4)]
        public Fixed64 m10;
        [Key(5)]
        public Fixed64 m11;
        [Key(6)]
        public Fixed64 m12;
        [Key(7)]
        public Fixed64 m13;

        [Key(8)]
        public Fixed64 m20;
        [Key(9)]
        public Fixed64 m21;
        [Key(10)]
        public Fixed64 m22;
        [Key(11)]
        public Fixed64 m23;

        [Key(12)]
        public Fixed64 m30;
        [Key(13)]
        public Fixed64 m31;
        [Key(14)]
        public Fixed64 m32;
        [Key(15)]
        public Fixed64 m33;

        /// <summary>
        /// Returns the identity matrix (diagonal elements set to 1).
        /// </summary>
        public static readonly Fixed4x4 Identity = new Fixed4x4(
            Fixed64.One, Fixed64.Zero, Fixed64.Zero, Fixed64.Zero,
            Fixed64.Zero, Fixed64.One, Fixed64.Zero, Fixed64.Zero,
            Fixed64.Zero, Fixed64.Zero, Fixed64.One, Fixed64.Zero,
            Fixed64.Zero, Fixed64.Zero, Fixed64.Zero, Fixed64.One);

        /// <summary>
        /// Returns a matrix with all elements set to zero.
        /// </summary>
        public static readonly Fixed4x4 Zero = new Fixed4x4(
            Fixed64.Zero, Fixed64.Zero, Fixed64.Zero, Fixed64.Zero,
            Fixed64.Zero, Fixed64.Zero, Fixed64.Zero, Fixed64.Zero,
            Fixed64.Zero, Fixed64.Zero, Fixed64.Zero, Fixed64.Zero,
            Fixed64.Zero, Fixed64.Zero, Fixed64.Zero, Fixed64.Zero);
       
        #endregion

        #region Constructors

        /// <summary>
        /// Initializes a new FixedMatrix4x4 with individual elements.
        /// </summary>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public Fixed4x4(
            Fixed64 m00, Fixed64 m01, Fixed64 m02, Fixed64 m03,
            Fixed64 m10, Fixed64 m11, Fixed64 m12, Fixed64 m13,
            Fixed64 m20, Fixed64 m21, Fixed64 m22, Fixed64 m23,
            Fixed64 m30, Fixed64 m31, Fixed64 m32, Fixed64 m33
        )
        {
            this.m00 = m00; this.m01 = m01; this.m02 = m02; this.m03 = m03;
            this.m10 = m10; this.m11 = m11; this.m12 = m12; this.m13 = m13;
            this.m20 = m20; this.m21 = m21; this.m22 = m22; this.m23 = m23;
            this.m30 = m30; this.m31 = m31; this.m32 = m32; this.m33 = m33;
        }

        #endregion

        #region Properties and Methods (Instance)

        [IgnoreMember]
        public readonly bool IsAffine => (m33 == Fixed64.One) && (m03 == Fixed64.Zero && m13 == Fixed64.Zero && m23 == Fixed64.Zero);

        /// <inheritdoc cref="ExtractTranslation(Fixed4x4)" />
        [IgnoreMember]
        public readonly Vector3d Translation => ExtractTranslation(this);

        [IgnoreMember]
        public readonly Vector3d Up => ExtractUp(this);

        /// <inheritdoc cref="ExtractScale(Fixed4x4)" />
        [IgnoreMember]
        public readonly Vector3d Scale => ExtractScale(this);

        /// <inheritdoc cref="ExtractRotation(Fixed4x4)" />
        [IgnoreMember]
        public readonly FixedQuaternion Rotation => ExtractRotation(this);

        [IgnoreMember]
        public Fixed64 this[int index]
        {
            get
            {
                return index switch
                {
                    0 => m00,
                    1 => m10,
                    2 => m20,
                    3 => m30,
                    4 => m01,
                    5 => m11,
                    6 => m21,
                    7 => m31,
                    8 => m02,
                    9 => m12,
                    10 => m22,
                    11 => m32,
                    12 => m03,
                    13 => m13,
                    14 => m23,
                    15 => m33,
                    _ => throw new IndexOutOfRangeException("Invalid matrix index!"),
                };
            }
            set
            {
                switch (index)
                {
                    case 0:
                        m00 = value;
                        break;
                    case 1:
                        m10 = value;
                        break;
                    case 2:
                        m20 = value;
                        break;
                    case 3:
                        m30 = value;
                        break;
                    case 4:
                        m01 = value;
                        break;
                    case 5:
                        m11 = value;
                        break;
                    case 6:
                        m21 = value;
                        break;
                    case 7:
                        m31 = value;
                        break;
                    case 8:
                        m02 = value;
                        break;
                    case 9:
                        m12 = value;
                        break;
                    case 10:
                        m22 = value;
                        break;
                    case 11:
                        m32 = value;
                        break;
                    case 12:
                        m03 = value;
                        break;
                    case 13:
                        m13 = value;
                        break;
                    case 14:
                        m23 = value;
                        break;
                    case 15:
                        m33 = value;
                        break;
                    default:
                        throw new IndexOutOfRangeException("Invalid matrix index!");
                }
            }
        }

        /// <summary>
        /// Calculates the determinant of a 4x4 matrix.
        /// </summary>
        public Fixed64 GetDeterminant()
        {
            if (IsAffine)
            {
                return m00 * (m11 * m22 - m12 * m21)
                     - m01 * (m10 * m22 - m12 * m20)
                     + m02 * (m10 * m21 - m11 * m20);
            }

            // Process as full 4x4 matrix
            Fixed64 minor0 = m22 * m33 - m23 * m32;
            Fixed64 minor1 = m21 * m33 - m23 * m31;
            Fixed64 minor2 = m21 * m32 - m22 * m31;
            Fixed64 cofactor0 = m20 * m33 - m23 * m30;
            Fixed64 cofactor1 = m20 * m32 - m22 * m30;
            Fixed64 cofactor2 = m20 * m31 - m21 * m30;
            return m00 * (m11 * minor0 - m12 * minor1 + m13 * minor2)
                - m01 * (m10 * minor0 - m12 * cofactor0 + m13 * cofactor1)
                + m02 * (m10 * minor1 - m11 * cofactor0 + m13 * cofactor2)
                - m03 * (m10 * minor2 - m11 * cofactor1 + m12 * cofactor2);
        }

        /// <inheritdoc cref="Fixed4x4.ResetScaleToIdentity(Fixed4x4)" />
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public Fixed4x4 ResetScaleToIdentity()
        {
            return this = ResetScaleToIdentity(this);
        }

        /// <summary>
        /// Sets the translation, scale, and rotation components onto the matrix.
        /// </summary>
        /// <param name="translation">The translation vector.</param>
        /// <param name="scale">The scale vector.</param>
        /// <param name="rotation">The rotation quaternion.</param>
        public void SetTransform(Vector3d translation, FixedQuaternion rotation, Vector3d scale)
        {
            this = CreateTransform(translation, rotation, scale);
        }

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
            result.m00 = Fixed64.One;
            result.m01 = Fixed64.Zero;
            result.m02 = Fixed64.Zero;
            result.m03 = Fixed64.Zero;
            result.m10 = Fixed64.Zero;
            result.m11 = Fixed64.One;
            result.m12 = Fixed64.Zero;
            result.m13 = Fixed64.Zero;
            result.m20 = Fixed64.Zero;
            result.m21 = Fixed64.Zero;
            result.m22 = Fixed64.One;
            result.m23 = Fixed64.Zero;
            result.m30 = position.x;
            result.m31 = position.y;
            result.m32 = position.z;
            result.m33 = Fixed64.One;
            return result;
        }

        /// <summary>
        /// Creates a rotation matrix from a quaternion.
        /// </summary>
        /// <param name="rotation">The quaternion representing the rotation.</param>
        /// <returns>A 4x4 matrix representing the rotation.</returns>
        public static Fixed4x4 CreateRotation(FixedQuaternion rotation)
        {
            Fixed3x3 rotationMatrix = rotation.ToMatrix3x3();

            return new Fixed4x4(
                rotationMatrix.m00, rotationMatrix.m01, rotationMatrix.m02, Fixed64.Zero,
                rotationMatrix.m10, rotationMatrix.m11, rotationMatrix.m12, Fixed64.Zero,
                rotationMatrix.m20, rotationMatrix.m21, rotationMatrix.m22, Fixed64.Zero,
                Fixed64.Zero, Fixed64.Zero, Fixed64.Zero, Fixed64.One
            );
        }

        /// <summary>
        /// Creates a scale matrix from a 3-dimensional vector.
        /// </summary>
        /// <param name="scale">The vector representing the scale along each axis.</param>
        /// <returns>A 4x4 matrix representing the scale transformation.</returns>
        public static Fixed4x4 CreateScale(Vector3d scale)
        {
            return new Fixed4x4(
                scale.x, Fixed64.Zero, Fixed64.Zero, Fixed64.Zero,
                Fixed64.Zero, scale.y, Fixed64.Zero, Fixed64.Zero,
                Fixed64.Zero, Fixed64.Zero, scale.z, Fixed64.Zero,
                Fixed64.Zero, Fixed64.Zero, Fixed64.Zero, Fixed64.One
            );
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
            // Create the rotation matrix and normalize it
            Fixed4x4 rotationMatrix = CreateRotation(rotation);
            rotationMatrix = NormalizeRotationMatrix(rotationMatrix);

            // Apply scale directly to the rotation matrix
            rotationMatrix = ApplyScaleToRotation(rotationMatrix, scale);

            // Apply the translation to the combined matrix
            rotationMatrix = SetTranslation(rotationMatrix, translation);

            return rotationMatrix;
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
            // Create translation matrix
            Fixed4x4 translationMatrix = CreateTranslation(translation);

            // Create rotation matrix using the quaternion
            Fixed4x4 rotationMatrix = CreateRotation(rotation);

            // Create scaling matrix
            Fixed4x4 scalingMatrix = CreateScale(scale);

            // Combine all transformations
            return (scalingMatrix * rotationMatrix) * translationMatrix;
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
            // Create translation matrix
            Fixed4x4 translationMatrix = CreateTranslation(translation);

            // Create rotation matrix using the quaternion
            Fixed4x4 rotationMatrix = CreateRotation(rotation);

            // Create scaling matrix
            Fixed4x4 scalingMatrix = CreateScale(scale);

            // Combine all transformations
            return (translationMatrix * rotationMatrix) * scalingMatrix;
        }

        #endregion

        #region Decomposition, Extraction, and Setters

        /// <summary>
        /// Extracts the translation component from the 4x4 matrix.
        /// </summary>
        /// <param name="matrix">The matrix from which to extract the translation.</param>
        /// <returns>A Vector3d representing the translation component.</returns>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static Vector3d ExtractTranslation(Fixed4x4 matrix)
        {
            return new Vector3d(matrix.m30, matrix.m31, matrix.m32);
        }

        /// <summary>
        /// Extracts the up direction from the 4x4 matrix.
        /// </summary>
        /// <remarks>
        /// This is the surface normal if the matrix represents ground orientation.
        /// </remarks>
        /// <param name="matrix"></param>
        /// <returns>A <see cref="Vector3d"/> representing the up direction.</returns>
        public static Vector3d ExtractUp(Fixed4x4 matrix)
        {
            return new Vector3d(matrix.m10, matrix.m11, matrix.m12).Normalize();
        }

        /// <summary>
        /// Extracts the scaling factors from the matrix by calculating the magnitudes of the basis vectors (non-lossy).
        /// </summary>
        /// <returns>A Vector3d representing the precise scale along the X, Y, and Z axes.</returns>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static Vector3d ExtractScale(Fixed4x4 matrix)
        {
            return new Vector3d(
                new Vector3d(matrix.m00, matrix.m01, matrix.m02).Magnitude,  // X scale
                new Vector3d(matrix.m10, matrix.m11, matrix.m12).Magnitude,  // Y scale
                new Vector3d(matrix.m20, matrix.m21, matrix.m22).Magnitude   // Z scale
            );
        }

        /// <summary>
        /// Extracts the scaling factors from the matrix by returning the diagonal elements (lossy).
        /// </summary>
        /// <returns>A Vector3d representing the scale along X, Y, and Z axes (lossy).</returns>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static Vector3d ExtractLossyScale(Fixed4x4 matrix)
        {
            return new Vector3d(matrix.m00, matrix.m11, matrix.m22);
        }

        /// <summary>
        /// Extracts the rotation component from the 4x4 matrix by normalizing the rotation matrix.
        /// </summary>
        /// <param name="matrix">The matrix from which to extract the rotation.</param>
        /// <returns>A FixedQuaternion representing the rotation component.</returns>
        public static FixedQuaternion ExtractRotation(Fixed4x4 matrix)
        {
            Vector3d scale = ExtractScale(matrix);

            // prevent divide by zero exception
            Fixed64 scaleX = scale.x == Fixed64.Zero ? Fixed64.One : scale.x;
            Fixed64 scaleY = scale.y == Fixed64.Zero ? Fixed64.One : scale.y;
            Fixed64 scaleZ = scale.z == Fixed64.Zero ? Fixed64.One : scale.z;

            Fixed4x4 normalizedMatrix = new Fixed4x4(
                matrix.m00 / scaleX, matrix.m01 / scaleY, matrix.m02 / scaleZ, Fixed64.Zero,
                matrix.m10 / scaleX, matrix.m11 / scaleY, matrix.m12 / scaleZ, Fixed64.Zero,
                matrix.m20 / scaleX, matrix.m21 / scaleY, matrix.m22 / scaleZ, Fixed64.Zero,
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
                 scale.x == Fixed64.Zero ? Fixed64.One : scale.x,
                 scale.y == Fixed64.Zero ? Fixed64.One : scale.y,
                 scale.z == Fixed64.Zero ? Fixed64.One : scale.z);

            // normalize rotation and scaling
            Fixed4x4 normalizedMatrix = ApplyScaleToRotation(matrix, Vector3d.One / scale);

            // Extract translation
            translation = new Vector3d(normalizedMatrix.m30, normalizedMatrix.m31, normalizedMatrix.m32);

            // Check the determinant to ensure correct handedness
            Fixed64 determinant = normalizedMatrix.GetDeterminant();
            if (determinant < Fixed64.Zero)
            {
                // Adjust for left-handed coordinate system by flipping one of the axes
                scale.x = -scale.x;
                normalizedMatrix.m00 = -normalizedMatrix.m00;
                normalizedMatrix.m01 = -normalizedMatrix.m01;
                normalizedMatrix.m02 = -normalizedMatrix.m02;
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
            matrix.m30 = translation.x;
            matrix.m31 = translation.y;
            matrix.m32 = translation.z;
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
            matrix.m00 = scale.x;
            matrix.m11 = scale.y;
            matrix.m22 = scale.z;
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
            matrix.m00 *= scale.x;
            matrix.m01 *= scale.x;
            matrix.m02 *= scale.x;

            matrix.m10 *= scale.y;
            matrix.m11 *= scale.y;
            matrix.m12 *= scale.y;

            matrix.m20 *= scale.z;
            matrix.m21 *= scale.z;
            matrix.m22 *= scale.z;

            return matrix;
        }

        /// <summary>
        /// Resets the scaling part of the matrix to identity (1,1,1).
        /// </summary>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static Fixed4x4 ResetScaleToIdentity(Fixed4x4 matrix)
        {
            matrix.m00 = Fixed64.One;  // X scale
            matrix.m11 = Fixed64.One;  // Y scale
            matrix.m22 = Fixed64.One;  // Z scale

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
            Vector3d newLocalScale = new Vector3d(
               globalScale.x / Fixed64.One,
               globalScale.y / Fixed64.One,
               globalScale.z / Fixed64.One
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

            matrix.m00 = rotationMatrix.m00 * scale.x;
            matrix.m01 = rotationMatrix.m01 * scale.x;
            matrix.m02 = rotationMatrix.m02 * scale.x;

            matrix.m10 = rotationMatrix.m10 * scale.y;
            matrix.m11 = rotationMatrix.m11 * scale.y;
            matrix.m12 = rotationMatrix.m12 * scale.y;

            matrix.m20 = rotationMatrix.m20 * scale.z;
            matrix.m21 = rotationMatrix.m21 * scale.z;
            matrix.m22 = rotationMatrix.m22 * scale.z;

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
            Vector3d basisX = new Vector3d(matrix.m00, matrix.m01, matrix.m02).Normalize();
            Vector3d basisY = new Vector3d(matrix.m10, matrix.m11, matrix.m12).Normalize();
            Vector3d basisZ = new Vector3d(matrix.m20, matrix.m21, matrix.m22).Normalize();

            return new Fixed4x4(
                basisX.x, basisX.y, basisX.z, Fixed64.Zero,
                basisY.x, basisY.y, basisY.z, Fixed64.Zero,
                basisZ.x, basisZ.y, basisZ.z, Fixed64.Zero,
                Fixed64.Zero, Fixed64.Zero, Fixed64.Zero, Fixed64.One
            );
        }

        #endregion

        #region Static Matrix Operators

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
                (matrix.m11 * matrix.m22 - matrix.m12 * matrix.m21) * invDet,
                (matrix.m02 * matrix.m21 - matrix.m01 * matrix.m22) * invDet,
                (matrix.m01 * matrix.m12 - matrix.m02 * matrix.m11) * invDet, Fixed64.Zero,

                (matrix.m12 * matrix.m20 - matrix.m10 * matrix.m22) * invDet,
                (matrix.m00 * matrix.m22 - matrix.m02 * matrix.m20) * invDet,
                (matrix.m02 * matrix.m10 - matrix.m00 * matrix.m12) * invDet, Fixed64.Zero,

                (matrix.m10 * matrix.m21 - matrix.m11 * matrix.m20) * invDet,
                (matrix.m01 * matrix.m20 - matrix.m00 * matrix.m21) * invDet,
                (matrix.m00 * matrix.m11 - matrix.m01 * matrix.m10) * invDet, Fixed64.Zero,

                Fixed64.Zero, Fixed64.Zero, Fixed64.Zero, Fixed64.One  // Ensure homogeneous coordinate stays valid
            );

            Fixed3x3 rotationScaleInverse = new Fixed3x3(
                result.m00, result.m01, result.m02,
                result.m10, result.m11, result.m12,
                result.m20, result.m21, result.m22
            );

            // Correct translation component
            Vector3d transformedTranslation = new Vector3d(matrix.m30, matrix.m31, matrix.m32);
            transformedTranslation = -Fixed3x3.TransformDirection(rotationScaleInverse, transformedTranslation);

            result.m30 = transformedTranslation.x;
            result.m31 = transformedTranslation.y;
            result.m32 = transformedTranslation.z;
            result.m33 = Fixed64.One;

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
                m00 = invDet * ((matrix.m11 * matrix.m22 * matrix.m33 + matrix.m12 * matrix.m23 * matrix.m31 + matrix.m13 * matrix.m21 * matrix.m32)
                              - (matrix.m13 * matrix.m22 * matrix.m31 + matrix.m11 * matrix.m23 * matrix.m32 + matrix.m12 * matrix.m21 * matrix.m33)),
                m01 = invDet * ((matrix.m01 * matrix.m23 * matrix.m32 + matrix.m02 * matrix.m21 * matrix.m33 + matrix.m03 * matrix.m22 * matrix.m31)
                              - (matrix.m03 * matrix.m21 * matrix.m32 + matrix.m01 * matrix.m22 * matrix.m33 + matrix.m02 * matrix.m23 * matrix.m31)),
                m02 = invDet * ((matrix.m01 * matrix.m12 * matrix.m33 + matrix.m02 * matrix.m13 * matrix.m31 + matrix.m03 * matrix.m11 * matrix.m32)
                              - (matrix.m03 * matrix.m12 * matrix.m31 + matrix.m01 * matrix.m13 * matrix.m32 + matrix.m02 * matrix.m11 * matrix.m33)),
                m03 = invDet * ((matrix.m01 * matrix.m13 * matrix.m22 + matrix.m02 * matrix.m11 * matrix.m23 + matrix.m03 * matrix.m12 * matrix.m21)
                              - (matrix.m03 * matrix.m11 * matrix.m22 + matrix.m01 * matrix.m12 * matrix.m23 + matrix.m02 * matrix.m13 * matrix.m21)),

                // Second row
                m10 = invDet * ((matrix.m10 * matrix.m23 * matrix.m32 + matrix.m12 * matrix.m20 * matrix.m33 + matrix.m13 * matrix.m22 * matrix.m30)
                              - (matrix.m13 * matrix.m20 * matrix.m32 + matrix.m10 * matrix.m22 * matrix.m33 + matrix.m12 * matrix.m23 * matrix.m30)),
                m11 = invDet * ((matrix.m00 * matrix.m22 * matrix.m33 + matrix.m02 * matrix.m23 * matrix.m30 + matrix.m03 * matrix.m20 * matrix.m32)
                              - (matrix.m03 * matrix.m20 * matrix.m32 + matrix.m00 * matrix.m23 * matrix.m32 + matrix.m02 * matrix.m20 * matrix.m33)),
                m12 = invDet * ((matrix.m00 * matrix.m13 * matrix.m32 + matrix.m02 * matrix.m10 * matrix.m33 + matrix.m03 * matrix.m12 * matrix.m30)
                              - (matrix.m03 * matrix.m10 * matrix.m32 + matrix.m00 * matrix.m12 * matrix.m33 + matrix.m02 * matrix.m13 * matrix.m30)),
                m13 = invDet * ((matrix.m00 * matrix.m12 * matrix.m23 + matrix.m02 * matrix.m13 * matrix.m20 + matrix.m03 * matrix.m10 * matrix.m22)
                              - (matrix.m03 * matrix.m10 * matrix.m22 + matrix.m00 * matrix.m13 * matrix.m22 + matrix.m02 * matrix.m12 * matrix.m20)),

                // Third row
                m20 = invDet * ((matrix.m10 * matrix.m21 * matrix.m33 + matrix.m11 * matrix.m23 * matrix.m30 + matrix.m13 * matrix.m20 * matrix.m31)
                              - (matrix.m13 * matrix.m20 * matrix.m31 + matrix.m10 * matrix.m23 * matrix.m31 + matrix.m11 * matrix.m20 * matrix.m33)),
                m21 = invDet * ((matrix.m00 * matrix.m23 * matrix.m31 + matrix.m01 * matrix.m20 * matrix.m33 + matrix.m03 * matrix.m21 * matrix.m30)
                              - (matrix.m03 * matrix.m20 * matrix.m31 + matrix.m00 * matrix.m21 * matrix.m33 + matrix.m01 * matrix.m23 * matrix.m30)),
                m22 = invDet * ((matrix.m00 * matrix.m11 * matrix.m33 + matrix.m01 * matrix.m13 * matrix.m30 + matrix.m03 * matrix.m10 * matrix.m31)
                              - (matrix.m03 * matrix.m10 * matrix.m31 + matrix.m00 * matrix.m13 * matrix.m31 + matrix.m01 * matrix.m10 * matrix.m33)),
                m23 = invDet * ((matrix.m00 * matrix.m13 * matrix.m21 + matrix.m01 * matrix.m10 * matrix.m23 + matrix.m03 * matrix.m11 * matrix.m20)
                              - (matrix.m03 * matrix.m10 * matrix.m21 + matrix.m00 * matrix.m11 * matrix.m23 + matrix.m01 * matrix.m13 * matrix.m20)),

                // Fourth row
                m30 = invDet * ((matrix.m10 * matrix.m22 * matrix.m31 + matrix.m11 * matrix.m20 * matrix.m32 + matrix.m12 * matrix.m21 * matrix.m30)
                              - (matrix.m12 * matrix.m20 * matrix.m31 + matrix.m10 * matrix.m21 * matrix.m32 + matrix.m11 * matrix.m22 * matrix.m30)),
                m31 = invDet * ((matrix.m00 * matrix.m21 * matrix.m32 + matrix.m01 * matrix.m22 * matrix.m30 + matrix.m02 * matrix.m20 * matrix.m31)
                              - (matrix.m02 * matrix.m20 * matrix.m31 + matrix.m00 * matrix.m22 * matrix.m31 + matrix.m01 * matrix.m20 * matrix.m32)),
                m32 = invDet * ((matrix.m00 * matrix.m12 * matrix.m31 + matrix.m01 * matrix.m10 * matrix.m32 + matrix.m02 * matrix.m11 * matrix.m30)
                              - (matrix.m02 * matrix.m10 * matrix.m31 + matrix.m00 * matrix.m11 * matrix.m32 + matrix.m01 * matrix.m12 * matrix.m30)),
                m33 = invDet * ((matrix.m00 * matrix.m11 * matrix.m22 + matrix.m01 * matrix.m12 * matrix.m20 + matrix.m02 * matrix.m10 * matrix.m21)
                              - (matrix.m02 * matrix.m10 * matrix.m21 + matrix.m00 * matrix.m12 * matrix.m21 + matrix.m01 * matrix.m11 * matrix.m20)),
            };

            return true;
        }

        /// <summary>
        /// Transforms a point from local space to world space using this transformation matrix.
        /// </summary>
        /// <remarks>
        /// This is the same as doing `<see cref="Fixed4x4"/> a * <see cref="Vector3d"/> b`
        /// </remarks>
        /// <param name="matrix">The transformation matrix.</param>
        /// <param name="point">The local-space point.</param>
        /// <returns>The transformed point in world space.</returns>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static Vector3d TransformPoint(Fixed4x4 matrix, Vector3d point)
        {
            if (matrix.IsAffine)
            {
                return new Vector3d(
                    matrix.m00 * point.x + matrix.m01 * point.y + matrix.m02 * point.z + matrix.m30,
                    matrix.m10 * point.x + matrix.m11 * point.y + matrix.m12 * point.z + matrix.m31,
                    matrix.m20 * point.x + matrix.m21 * point.y + matrix.m22 * point.z + matrix.m32
                );
            }

            return FullTransformPoint(matrix, point);
        }

        private static Vector3d FullTransformPoint(Fixed4x4 matrix, Vector3d point)
        {
            // Full 4×4 transformation (needed for perspective projections)
            Fixed64 w = matrix.m03 * point.x + matrix.m13 * point.y + matrix.m23 * point.z + matrix.m33;
            if (w == Fixed64.Zero) w = Fixed64.One;  // Prevent divide-by-zero

            return new Vector3d(
                (matrix.m00 * point.x + matrix.m01 * point.y + matrix.m02 * point.z + matrix.m30) / w,
                (matrix.m10 * point.x + matrix.m11 * point.y + matrix.m12 * point.z + matrix.m31) / w,
                (matrix.m20 * point.x + matrix.m21 * point.y + matrix.m22 * point.z + matrix.m32) / w
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
                    inverseMatrix.m00 * point.x + inverseMatrix.m01 * point.y + inverseMatrix.m02 * point.z + inverseMatrix.m30,
                    inverseMatrix.m10 * point.x + inverseMatrix.m11 * point.y + inverseMatrix.m12 * point.z + inverseMatrix.m31,
                    inverseMatrix.m20 * point.x + inverseMatrix.m21 * point.y + inverseMatrix.m22 * point.z + inverseMatrix.m32
                );
            }

            return FullInverseTransformPoint(inverseMatrix, point);
        }

        private static Vector3d FullInverseTransformPoint(Fixed4x4 matrix, Vector3d point)
        {
            // Full 4×4 transformation (needed for perspective projections)
            Fixed64 w = matrix.m03 * point.x + matrix.m13 * point.y + matrix.m23 * point.z + matrix.m33;
            if (w == Fixed64.Zero) w = Fixed64.One;  // Prevent divide-by-zero

            return new Vector3d(
                (matrix.m00 * point.x + matrix.m01 * point.y + matrix.m02 * point.z + matrix.m30) / w,
                (matrix.m10 * point.x + matrix.m11 * point.y + matrix.m12 * point.z + matrix.m31) / w,
                (matrix.m20 * point.x + matrix.m21 * point.y + matrix.m22 * point.z + matrix.m32) / w
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
            result.m00 = -value.m00;
            result.m01 = -value.m01;
            result.m02 = -value.m02;
            result.m03 = -value.m03;
            result.m10 = -value.m10;
            result.m11 = -value.m11;
            result.m12 = -value.m12;
            result.m13 = -value.m13;
            result.m20 = -value.m20;
            result.m21 = -value.m21;
            result.m22 = -value.m22;
            result.m23 = -value.m23;
            result.m30 = -value.m30;
            result.m31 = -value.m31;
            result.m32 = -value.m32;
            result.m33 = -value.m33;
            return result;
        }

        /// <summary>
        /// Adds two matrices element-wise.
        /// </summary>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static Fixed4x4 operator +(Fixed4x4 lhs, Fixed4x4 rhs)
        {
            return new Fixed4x4(
                lhs.m00 + rhs.m00, lhs.m01 + rhs.m01, lhs.m02 + rhs.m02, lhs.m03 + rhs.m03,
                lhs.m10 + rhs.m10, lhs.m11 + rhs.m11, lhs.m12 + rhs.m12, lhs.m13 + rhs.m13,
                lhs.m20 + rhs.m20, lhs.m21 + rhs.m21, lhs.m22 + rhs.m22, lhs.m23 + rhs.m23,
                lhs.m30 + rhs.m30, lhs.m31 + rhs.m31, lhs.m32 + rhs.m32, lhs.m33 + rhs.m33);
        }

        /// <summary>
        /// Subtracts two matrices element-wise.
        /// </summary>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static Fixed4x4 operator -(Fixed4x4 lhs, Fixed4x4 rhs)
        {
            return new Fixed4x4(
                lhs.m00 - rhs.m00, lhs.m01 - rhs.m01, lhs.m02 - rhs.m02, lhs.m03 - rhs.m03,
                lhs.m10 - rhs.m10, lhs.m11 - rhs.m11, lhs.m12 - rhs.m12, lhs.m13 - rhs.m13,
                lhs.m20 - rhs.m20, lhs.m21 - rhs.m21, lhs.m22 - rhs.m22, lhs.m23 - rhs.m23,
                lhs.m30 - rhs.m30, lhs.m31 - rhs.m31, lhs.m32 - rhs.m32, lhs.m33 - rhs.m33);
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
                    lhs.m00 * rhs.m00 + lhs.m01 * rhs.m10 + lhs.m02 * rhs.m20,
                    lhs.m00 * rhs.m01 + lhs.m01 * rhs.m11 + lhs.m02 * rhs.m21,
                    lhs.m00 * rhs.m02 + lhs.m01 * rhs.m12 + lhs.m02 * rhs.m22,
                    Fixed64.Zero,

                    lhs.m10 * rhs.m00 + lhs.m11 * rhs.m10 + lhs.m12 * rhs.m20,
                    lhs.m10 * rhs.m01 + lhs.m11 * rhs.m11 + lhs.m12 * rhs.m21,
                    lhs.m10 * rhs.m02 + lhs.m11 * rhs.m12 + lhs.m12 * rhs.m22,
                    Fixed64.Zero,

                    lhs.m20 * rhs.m00 + lhs.m21 * rhs.m10 + lhs.m22 * rhs.m20,
                    lhs.m20 * rhs.m01 + lhs.m21 * rhs.m11 + lhs.m22 * rhs.m21,
                    lhs.m20 * rhs.m02 + lhs.m21 * rhs.m12 + lhs.m22 * rhs.m22,
                    Fixed64.Zero,

                    lhs.m30 * rhs.m00 + lhs.m31 * rhs.m10 + lhs.m32 * rhs.m20 + rhs.m30,
                    lhs.m30 * rhs.m01 + lhs.m31 * rhs.m11 + lhs.m32 * rhs.m21 + rhs.m31,
                    lhs.m30 * rhs.m02 + lhs.m31 * rhs.m12 + lhs.m32 * rhs.m22 + rhs.m32,
                    Fixed64.One
                );
            }

            // Full 4×4 multiplication (fallback for perspective matrices)
            return new Fixed4x4(
                // Upper-left 3×3 matrix multiplication (rotation & scale)
                lhs.m00 * rhs.m00 + lhs.m01 * rhs.m10 + lhs.m02 * rhs.m20 + lhs.m03 * rhs.m30,
                lhs.m00 * rhs.m01 + lhs.m01 * rhs.m11 + lhs.m02 * rhs.m21 + lhs.m03 * rhs.m31,
                lhs.m00 * rhs.m02 + lhs.m01 * rhs.m12 + lhs.m02 * rhs.m22 + lhs.m03 * rhs.m32,
                lhs.m00 * rhs.m03 + lhs.m01 * rhs.m13 + lhs.m02 * rhs.m23 + lhs.m03 * rhs.m33,

                lhs.m10 * rhs.m00 + lhs.m11 * rhs.m10 + lhs.m12 * rhs.m20 + lhs.m13 * rhs.m30,
                lhs.m10 * rhs.m01 + lhs.m11 * rhs.m11 + lhs.m12 * rhs.m21 + lhs.m13 * rhs.m31,
                lhs.m10 * rhs.m02 + lhs.m11 * rhs.m12 + lhs.m12 * rhs.m22 + lhs.m13 * rhs.m32,
                lhs.m10 * rhs.m03 + lhs.m11 * rhs.m13 + lhs.m12 * rhs.m23 + lhs.m13 * rhs.m33,

                lhs.m20 * rhs.m00 + lhs.m21 * rhs.m10 + lhs.m22 * rhs.m20 + lhs.m23 * rhs.m30,
                lhs.m20 * rhs.m01 + lhs.m21 * rhs.m11 + lhs.m22 * rhs.m21 + lhs.m23 * rhs.m31,
                lhs.m20 * rhs.m02 + lhs.m21 * rhs.m12 + lhs.m22 * rhs.m22 + lhs.m23 * rhs.m32,
                lhs.m20 * rhs.m03 + lhs.m21 * rhs.m13 + lhs.m22 * rhs.m23 + lhs.m23 * rhs.m33,

                // Compute new translation
                lhs.m30 * rhs.m00 + lhs.m31 * rhs.m10 + lhs.m32 * rhs.m20 + lhs.m33 * rhs.m30,
                lhs.m30 * rhs.m01 + lhs.m31 * rhs.m11 + lhs.m32 * rhs.m21 + lhs.m33 * rhs.m31,
                lhs.m30 * rhs.m02 + lhs.m31 * rhs.m12 + lhs.m32 * rhs.m22 + lhs.m33 * rhs.m32,
                lhs.m30 * rhs.m03 + lhs.m31 * rhs.m13 + lhs.m32 * rhs.m23 + lhs.m33 * rhs.m33
            );
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static bool operator ==(Fixed4x4 left, Fixed4x4 right)
        {
            return left.Equals(right);
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static bool operator !=(Fixed4x4 left, Fixed4x4 right)
        {
            return !(left == right);
        }

        #endregion

        #region Equality and HashCode Overrides

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public override bool Equals(object? obj)
        {
            return obj is Fixed4x4 x && Equals(x);
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public bool Equals(Fixed4x4 other)
        {
            return m00 == other.m00 && m01 == other.m01 && m02 == other.m02 && m03 == other.m03 &&
                   m10 == other.m10 && m11 == other.m11 && m12 == other.m12 && m13 == other.m13 &&
                   m20 == other.m20 && m21 == other.m21 && m22 == other.m22 && m23 == other.m23 &&
                   m30 == other.m30 && m31 == other.m31 && m32 == other.m32 && m33 == other.m33;
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public override int GetHashCode()
        {
            unchecked
            {
                int hash = 17;
                hash = hash * 23 + m00.GetHashCode();
                hash = hash * 23 + m01.GetHashCode();
                hash = hash * 23 + m02.GetHashCode();
                hash = hash * 23 + m10.GetHashCode();
                hash = hash * 23 + m11.GetHashCode();
                hash = hash * 23 + m12.GetHashCode();
                hash = hash * 23 + m20.GetHashCode();
                hash = hash * 23 + m21.GetHashCode();
                hash = hash * 23 + m22.GetHashCode();
                hash = hash * 23 + m23.GetHashCode();
                hash = hash * 23 + m30.GetHashCode();
                hash = hash * 23 + m31.GetHashCode();
                hash = hash * 23 + m32.GetHashCode();
                hash = hash * 23 + m33.GetHashCode();
                return hash;
            }
        }

        #endregion    
    }
}