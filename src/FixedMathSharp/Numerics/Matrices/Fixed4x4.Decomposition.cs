//=======================================================================
// Fixed4x4.Decomposition.cs
//=======================================================================
// MIT License, Copyright (c) 2024–present David Oravsky (mrdav30)
// See LICENSE file in the project root for full license information.
//=======================================================================

using System.Runtime.CompilerServices;

namespace FixedMathSharp;

public partial struct Fixed4x4
{
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
    public static Vector3d ExtractRight(Fixed4x4 matrix) => new Vector3d(matrix.M11, matrix.M12, matrix.M13).NormalizeInPlace();

    /// <summary>
    /// Extracts the up direction from the 4x4 matrix.
    /// </summary>
    /// <remarks>
    /// This is the surface normal if the matrix represents ground orientation.
    /// </remarks>
    /// <param name="matrix"></param>
    /// <returns>A <see cref="Vector3d"/> representing the up direction.</returns>
    public static Vector3d ExtractUp(Fixed4x4 matrix) => new Vector3d(matrix.M21, matrix.M22, matrix.M23).NormalizeInPlace();

    /// <summary>
    /// Extracts the forward direction from the 4x4 matrix.
    /// </summary>
    public static Vector3d ExtractForward(Fixed4x4 matrix) => new Vector3d(matrix.M31, matrix.M32, matrix.M33).NormalizeInPlace();

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
    /// Decomposes a 4x4 matrix into its translation, rotation, and scale components.
    /// </summary>
    /// <param name="matrix">The 4x4 matrix to decompose.</param>
    /// <param name="translation">The extracted translation component.</param>
    /// <param name="rotation">The extracted rotation component as a quaternion.</param>
    /// <param name="scale">The extracted scale component.</param>
    /// <returns>True if decomposition was successful, otherwise false.</returns>
    public static bool Decompose(
        Fixed4x4 matrix,
        out Vector3d translation,
        out FixedQuaternion rotation,
        out Vector3d scale)
    {
        // Extract scale by calculating the magnitudes of the basis vectors
        scale = ExtractScale(matrix);

        // prevent divide by zero exception
        scale = new Vector3d(
             scale.X == Fixed64.Zero ? Fixed64.One : scale.X,
             scale.Y == Fixed64.Zero ? Fixed64.One : scale.Y,
             scale.Z == Fixed64.Zero ? Fixed64.One : scale.Z);

        // normalize rotation and scaling
        var inverseScale = new Vector3d(
            FixedMath.FastDiv(Fixed64.One, scale.X),
            FixedMath.FastDiv(Fixed64.One, scale.Y),
            FixedMath.FastDiv(Fixed64.One, scale.Z));
        Fixed4x4 normalizedMatrix = ApplyScaleToRotation(matrix, inverseScale);

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
    /// The method extracts translation and rotation, then rebuilds the transform with the requested
    /// scale so component mutation does not silently drop position or distort rotation.
    /// </remarks>
    public static Fixed4x4 SetGlobalScale(Fixed4x4 matrix, Vector3d globalScale)
    {
        Vector3d translation = ExtractTranslation(matrix);
        FixedQuaternion rotation = ExtractRotation(matrix);

        return CreateTransform(translation, rotation, globalScale);
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
        Vector3d basisX = new Vector3d(matrix.M11, matrix.M12, matrix.M13).NormalizeInPlace();
        Vector3d basisY = new Vector3d(matrix.M21, matrix.M22, matrix.M23).NormalizeInPlace();
        Vector3d basisZ = new Vector3d(matrix.M31, matrix.M32, matrix.M33).NormalizeInPlace();

        return new Fixed4x4(
            basisX.X, basisX.Y, basisX.Z, Fixed64.Zero,
            basisY.X, basisY.Y, basisY.Z, Fixed64.Zero,
            basisZ.X, basisZ.Y, basisZ.Z, Fixed64.Zero,
            Fixed64.Zero, Fixed64.Zero, Fixed64.Zero, Fixed64.One
        );
    }

    #endregion
}
