using System;
using System.Numerics;
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
    public struct Fixed4x4 : IEquatable<Fixed4x4>
    {
        #region Fields and Constants

        public Fixed64 m00, m01, m02, m03;
        public Fixed64 m10, m11, m12, m13;
        public Fixed64 m20, m21, m22, m23;
        public Fixed64 m30, m31, m32, m33;

        /// <summary>
        /// Returns the identity matrix (diagonal elements set to 1).
        /// </summary>
        public static readonly Fixed4x4 Identity = new Fixed4x4(
            Fixed64.One, Fixed64.Zero, Fixed64.Zero, Fixed64.Zero,
            Fixed64.Zero, Fixed64.One, Fixed64.Zero, Fixed64.Zero,
            Fixed64.Zero, Fixed64.Zero, Fixed64.One, Fixed64.Zero,
            Fixed64.Zero, Fixed64.Zero, Fixed64.Zero, Fixed64.One);

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

        /// <summary>
        /// Gets or sets the translation component of this matrix.
        /// </summary>
        /// <returns>
        /// The translation component of the current instance.
        /// </returns>
        public Vector3d Translation
        {
            [MethodImpl(MethodImplOptions.AggressiveInlining)]
            get
            {
                return new Vector3d(m30, m31, m32);
            }
            [MethodImpl(MethodImplOptions.AggressiveInlining)]
            set
            {
                m30 = value.x;
                m31 = value.y;
                m32 = value.z;
            }
        }

        /// <summary>
        /// Calculates the determinant of a 4x4 matrix.
        /// </summary>
        public Fixed64 GetDeterminant()
        {
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
        public void SetTransform(Vector3d translation, Vector3d scale, FixedQuaternion rotation)
        {
            this = CreateTransform(translation, scale, rotation);
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
        /// Creates a matrix from the provided translation, scale, and rotation components.
        /// </summary>
        /// <param name="translation">The translation vector.</param>
        /// <param name="scale">The scale vector.</param>
        /// <param name="rotation">The rotation quaternion.</param>
        public static Fixed4x4 CreateTransform(Vector3d translation, Vector3d scale, FixedQuaternion rotation)
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
        /// Creates a 4x4 transformation matrix from translation, rotation, and scale.
        /// </summary>
        /// <param name="translation">Translation vector.</param>
        /// <param name="rotation">Rotation as a quaternion.</param>
        /// <param name="scale">Scale vector.</param>
        /// <returns>A combined transformation matrix.</returns>
        public static Fixed4x4 TRS(Vector3d translation, FixedQuaternion rotation, Vector3d scale)
        {
            // Create translation matrix
            Fixed4x4 translationMatrix = new Fixed4x4(
                Fixed64.One, Fixed64.Zero, Fixed64.Zero, translation.x,
                Fixed64.Zero, Fixed64.One, Fixed64.Zero, translation.y,
                Fixed64.Zero, Fixed64.Zero, Fixed64.One, translation.z,
                Fixed64.Zero, Fixed64.Zero, Fixed64.Zero, Fixed64.One);

            // Create rotation matrix using the quaternion
            Fixed64 xx = rotation.x * rotation.x;
            Fixed64 xy = rotation.x * rotation.y;
            Fixed64 xz = rotation.x * rotation.z;
            Fixed64 xw = rotation.x * rotation.w;

            Fixed64 yy = rotation.y * rotation.y;
            Fixed64 yz = rotation.y * rotation.z;
            Fixed64 yw = rotation.y * rotation.w;

            Fixed64 zz = rotation.z * rotation.z;
            Fixed64 zw = rotation.z * rotation.w;

            Fixed4x4 rotationMatrix = new Fixed4x4(
                Fixed64.One - 2 * (yy + zz), 2 * (xy - zw), 2 * (xz + yw), Fixed64.Zero,
                2 * (xy + zw), Fixed64.One - 2 * (xx + zz), 2 * (yz - xw), Fixed64.Zero,
                2 * (xz - yw), 2 * (yz + xw), Fixed64.One - 2 * (xx + yy), Fixed64.Zero,
                Fixed64.Zero, Fixed64.Zero, Fixed64.Zero, Fixed64.One);

            // Create scaling matrix
            Fixed4x4 scalingMatrix = new Fixed4x4(
                scale.x, Fixed64.Zero, Fixed64.Zero, Fixed64.Zero,
                Fixed64.Zero, scale.y, Fixed64.Zero, Fixed64.Zero,
                Fixed64.Zero, Fixed64.Zero, scale.z, Fixed64.Zero,
                Fixed64.Zero, Fixed64.Zero, Fixed64.Zero, Fixed64.One);

            // Combine all transformations
            return translationMatrix * rotationMatrix * scalingMatrix;
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

            Fixed4x4 normalizedMatrix = new Fixed4x4(
                matrix.m00 / scale.x, matrix.m01 / scale.y, matrix.m02 / scale.z, Fixed64.Zero,
                matrix.m10 / scale.x, matrix.m11 / scale.y, matrix.m12 / scale.z, Fixed64.Zero,
                matrix.m20 / scale.x, matrix.m21 / scale.y, matrix.m22 / scale.z, Fixed64.Zero,
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
            // Extract translation
            translation = ExtractTranslation(matrix);

            // Extract scale by calculating the magnitudes of the basis vectors
            scale = ExtractScale(matrix);

            // Adjust the basis vectors by dividing by their scale (normalization)
            Fixed4x4 normalizedMatrix = new Fixed4x4(
                matrix.m00 / scale.x, matrix.m01 / scale.y, matrix.m02 / scale.z, Fixed64.Zero,
                matrix.m10 / scale.x, matrix.m11 / scale.y, matrix.m12 / scale.z, Fixed64.Zero,
                matrix.m20 / scale.x, matrix.m21 / scale.y, matrix.m22 / scale.z, Fixed64.Zero,
                Fixed64.Zero, Fixed64.Zero, Fixed64.Zero, Fixed64.One
            );

            // Normalize the basis vectors and handle possible precision issues
            Vector3d basisX = new Vector3d(normalizedMatrix.m00, normalizedMatrix.m01, normalizedMatrix.m02).Normalize();
            Vector3d basisY = new Vector3d(normalizedMatrix.m10, normalizedMatrix.m11, normalizedMatrix.m12).Normalize();
            Vector3d basisZ = new Vector3d(normalizedMatrix.m20, normalizedMatrix.m21, normalizedMatrix.m22).Normalize();

            // Handle near-zero scale components by setting default axes
            Fixed64 epsilon = (Fixed64)0.0001;
            if (scale.x < epsilon) basisX = new Vector3d(Fixed64.One, Fixed64.Zero, Fixed64.Zero);
            if (scale.y < epsilon) basisY = Vector3d.Cross(basisZ, basisX).Normalize();
            if (scale.z < epsilon) basisZ = Vector3d.Cross(basisX, basisY).Normalize();

            // Recompute the matrix with corrected orthogonal basis vectors
            normalizedMatrix = new Fixed4x4(
                basisX.x, basisX.y, basisX.z, Fixed64.Zero,
                basisY.x, basisY.y, basisY.z, Fixed64.Zero,
                basisZ.x, basisZ.y, basisZ.z, Fixed64.Zero,
                Fixed64.Zero, Fixed64.Zero, Fixed64.Zero, Fixed64.One
            );

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

            // Apply rotation to the upper-left 3x3 matrix
            matrix.m00 = rotationMatrix.m00;
            matrix.m01 = rotationMatrix.m01;
            matrix.m02 = rotationMatrix.m02;

            matrix.m10 = rotationMatrix.m10;
            matrix.m11 = rotationMatrix.m11;
            matrix.m12 = rotationMatrix.m12;

            matrix.m20 = rotationMatrix.m20;
            matrix.m21 = rotationMatrix.m21;
            matrix.m22 = rotationMatrix.m22;

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

        #region Matrix Operators

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
        public static bool Invert(Fixed4x4 matrix, out Fixed4x4? result)
        {
            Fixed64 det = matrix.GetDeterminant();

            if (det == Fixed64.Zero)
            {
                result = null;
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
        /// Multiplies two matrices using matrix multiplication rules.
        /// </summary>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static Fixed4x4 operator *(Fixed4x4 lhs, Fixed4x4 rhs)
        {
            return new Fixed4x4(
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
                lhs.m30 * rhs.m00 + lhs.m31 * rhs.m10 + lhs.m32 * rhs.m20 + lhs.m33 * rhs.m30,
                lhs.m30 * rhs.m01 + lhs.m31 * rhs.m11 + lhs.m32 * rhs.m21 + lhs.m33 * rhs.m31,
                lhs.m30 * rhs.m02 + lhs.m31 * rhs.m12 + lhs.m32 * rhs.m22 + lhs.m33 * rhs.m32,
                lhs.m30 * rhs.m03 + lhs.m31 * rhs.m13 + lhs.m32 * rhs.m23 + lhs.m33 * rhs.m33);
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
        public override bool Equals(object obj)
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