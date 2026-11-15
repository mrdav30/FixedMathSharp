//=======================================================================
// Fixed4x4.Decomposition.cs
//=======================================================================
// MIT License, Copyright (c) 2024-present David Oravsky (mrdav30)
// See LICENSE file in the project root for full license information.
//=======================================================================

using System.Runtime.CompilerServices;
using FixedMathSharp.Geometry;

namespace FixedMathSharp;

/// <content>
/// Provides methods for decomposing and extracting components (translation, rotation axes, and scale)
/// from a <see cref="Fixed4x4"/> matrix.
/// </content>
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
    /// <param name="matrix">The matrix containing the +X basis row.</param>
    /// <returns>The normalized first basis row, or zero for a zero row.</returns>
    public static Vector3d ExtractRight(Fixed4x4 matrix) => new Vector3d(matrix.M11, matrix.M12, matrix.M13).NormalizeInPlace();

    /// <summary>
    /// Extracts the up direction from the 4x4 matrix.
    /// </summary>
    /// <remarks>
    /// This is the surface normal if the matrix represents ground orientation.
    /// </remarks>
    /// <returns>A <see cref="Vector3d"/> representing the up direction.</returns>
    /// <param name="matrix">The matrix containing the +Y basis row.</param>
    public static Vector3d ExtractUp(Fixed4x4 matrix) => new Vector3d(matrix.M21, matrix.M22, matrix.M23).NormalizeInPlace();

    /// <summary>
    /// Extracts the forward direction from the 4x4 matrix.
    /// </summary>
    /// <param name="matrix">The matrix containing the +Z basis row.</param>
    /// <returns>The normalized third basis row, or zero for a zero row.</returns>
    public static Vector3d ExtractForward(Fixed4x4 matrix) => new Vector3d(matrix.M31, matrix.M32, matrix.M33).NormalizeInPlace();

    /// <summary>
    /// Extracts the unsigned magnitudes of the matrix basis rows.
    /// </summary>
    /// <returns>The nonnegative basis magnitudes along X, Y, and Z.</returns>
    /// <param name="matrix">The matrix containing the three basis rows.</param>
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public static Vector3d ExtractScaleMagnitudes(Fixed4x4 matrix) =>
        new(
            new Vector3d(matrix.M11, matrix.M12, matrix.M13).Magnitude,
            new Vector3d(matrix.M21, matrix.M22, matrix.M23).Magnitude,
            new Vector3d(matrix.M31, matrix.M32, matrix.M33).Magnitude);

    /// <summary>
    /// Extracts canonical signed lossy scale from the matrix basis rows.
    /// </summary>
    /// <remarks>A reflected basis assigns its one recoverable negative sign to X.</remarks>
    /// <param name="matrix">The matrix containing the basis rows to measure.</param>
    /// <returns>The three basis row lengths, saturated if necessary, with X negated for a reflected basis.</returns>
    public static Vector3d ExtractLossyScale(Fixed4x4 matrix)
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

    /// <summary>
    /// Attempts to extract canonical signed lossy scale from the matrix basis rows.
    /// </summary>
    /// <remarks>
    /// This operation accepts affine, non-affine, singular, and sheared matrices.
    /// It fails only when at least one basis-row magnitude is outside the
    /// representable <see cref="Fixed64"/> range. A reflected basis assigns its
    /// one recoverable negative sign to X. Every failure writes zero.
    /// </remarks>
    /// <param name="matrix">The matrix containing the basis rows to measure.</param>
    /// <param name="scale">The signed row lengths on success, or <see cref="Vector3d.Zero"/> on failure.</param>
    /// <returns><see langword="true"/> if every basis row length is representable.</returns>
    public static bool TryExtractLossyScale(Fixed4x4 matrix, out Vector3d scale)
    {
        bool representable =
            Vector3d.TryGetMagnitude(
                new Vector3d(matrix.M11, matrix.M12, matrix.M13),
                out Fixed64 scaleX)
            & Vector3d.TryGetMagnitude(
                new Vector3d(matrix.M21, matrix.M22, matrix.M23),
                out Fixed64 scaleY)
            & Vector3d.TryGetMagnitude(
                new Vector3d(matrix.M31, matrix.M32, matrix.M33),
                out Fixed64 scaleZ);
        if (!representable)
        {
            scale = Vector3d.Zero;
            return false;
        }

        scale = new Vector3d(scaleX, scaleY, scaleZ);
        if (WideGeometry.GetTripleProductSign(
            matrix.M11, matrix.M12, matrix.M13,
            matrix.M21, matrix.M22, matrix.M23,
            matrix.M31, matrix.M32, matrix.M33) < 0)
        {
            scale.X = -scale.X;
        }

        return true;
    }

    /// <summary>
    /// Estimates a rotation from the matrix's basis rows.
    /// </summary>
    /// <param name="matrix">The matrix containing the orientation basis.</param>
    /// <returns>The quaternion computed after removing unsigned basis scale.</returns>
    /// <remarks>Use <see cref="Decompose"/> when shear, reflections, or degenerate basis rows must be handled explicitly.</remarks>
    public static FixedQuaternion ExtractRotation(Fixed4x4 matrix)
    {
        Vector3d scale = ExtractScaleMagnitudes(matrix);

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
    /// Attempts to decompose an affine 4x4 matrix into strict translation,
    /// rotation, and canonical lossy-scale components.
    /// </summary>
    /// <remarks>
    /// Perspective, shear, singular or unrepresentable basis magnitudes, and
    /// normalized rotation bases that cannot round-trip within <see cref="Fixed64.Epsilon"/>
    /// are rejected. Reconstructed scaled coefficients must also be representable.
    /// For each row's extracted scale s and one raw unit q, coefficient reconstruction error
    /// is bounded by <c>abs(s) * (Epsilon + q/2) + q/2</c>.
    /// Reflections assign their single negative scale sign to X.
    /// Every failure writes zero translation, identity rotation, and unit scale.
    /// </remarks>
    /// <param name="matrix">The row-vector matrix to decompose.</param>
    /// <param name="translation">The translation in output coordinates on success, or zero on failure.</param>
    /// <param name="rotation">The orientation on success, or the identity quaternion on failure.</param>
    /// <param name="scale">The signed local scale on success, or unit scale on failure.</param>
    /// <returns><see langword="true"/> if the matrix can be reconstructed as scale, rotation, and translation within tolerance.</returns>
    public static bool Decompose(
        Fixed4x4 matrix,
        out Vector3d translation,
        out FixedQuaternion rotation,
        out Vector3d scale)
    {
        translation = Vector3d.Zero;
        rotation = FixedQuaternion.Identity;
        scale = Vector3d.One;

        if (!matrix.IsAffine)
            return false;

        Vector3d basisX = new(matrix.M11, matrix.M12, matrix.M13);
        Vector3d basisY = new(matrix.M21, matrix.M22, matrix.M23);
        Vector3d basisZ = new(matrix.M31, matrix.M32, matrix.M33);
        if (!Vector3d.TryGetMagnitude(basisX, out Fixed64 scaleX)
            || !Vector3d.TryGetMagnitude(basisY, out Fixed64 scaleY)
            || !Vector3d.TryGetMagnitude(basisZ, out Fixed64 scaleZ)
            || scaleX == Fixed64.Zero
            || scaleY == Fixed64.Zero
            || scaleZ == Fixed64.Zero)
        {
            return false;
        }

        basisX /= scaleX;
        basisY /= scaleY;
        basisZ /= scaleZ;
        if (!IsNormalizedOrthogonalBasis(basisX, basisY, basisZ))
            return false;

        int handedness = WideGeometry.GetTripleProductSign(
            matrix.M11, matrix.M12, matrix.M13,
            matrix.M21, matrix.M22, matrix.M23,
            matrix.M31, matrix.M32, matrix.M33);
        if (handedness < 0)
        {
            scaleX = -scaleX;
            basisX = -basisX;
        }

        Fixed3x3 normalizedBasis = new(
            basisX,
            basisY,
            basisZ);
        FixedQuaternion candidateRotation = FixedQuaternion.FromMatrix(normalizedBasis).Normalized;
        Fixed3x3 reconstructed = candidateRotation.ToMatrix3x3();
        if (!reconstructed.FuzzyEqualAbsolute(normalizedBasis, Fixed64.Epsilon))
            return false;

        Vector3d candidateTranslation = new(matrix.M41, matrix.M42, matrix.M43);
        Vector3d candidateScale = new(scaleX, scaleY, scaleZ);
        // The normalized gate bounds row error by |scale| * (Epsilon + half a raw unit)
        // plus final half-raw rounding. Only representability remains to be checked here.
        if (!TryCreateTransform(
            candidateTranslation,
            reconstructed,
            candidateScale,
            out _))
        {
            return false;
        }

        translation = candidateTranslation;
        rotation = candidateRotation;
        scale = candidateScale;
        return true;
    }

    private static bool IsNormalizedOrthogonalBasis(Vector3d x, Vector3d y, Vector3d z) =>
        Vector3d.TryGetMagnitude(x, out Fixed64 xLength)
        && Vector3d.TryGetMagnitude(y, out Fixed64 yLength)
        && Vector3d.TryGetMagnitude(z, out Fixed64 zLength)
        && FixedMath.Abs(xLength - Fixed64.One) <= Fixed64.Epsilon
        && FixedMath.Abs(yLength - Fixed64.One) <= Fixed64.Epsilon
        && FixedMath.Abs(zLength - Fixed64.One) <= Fixed64.Epsilon
        && FixedMath.Abs(Vector3d.Dot(x, y)) <= Fixed64.Epsilon
        && FixedMath.Abs(Vector3d.Dot(x, z)) <= Fixed64.Epsilon
        && FixedMath.Abs(Vector3d.Dot(y, z)) <= Fixed64.Epsilon;

    /// <summary>
    /// Sets the translation component of the 4x4 matrix.
    /// </summary>
    /// <param name="matrix">The matrix to modify.</param>
    /// <param name="translation">The new translation vector.</param>
    /// <returns>A copy with the last row’s X, Y, and Z translation entries replaced.</returns>
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public static Fixed4x4 SetTranslation(Fixed4x4 matrix, Vector3d translation)
    {
        matrix.M41 = translation.X;
        matrix.M42 = translation.Y;
        matrix.M43 = translation.Z;
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
    /// <returns>A copy with the upper-left basis rows scaled by their corresponding scale factors.</returns>
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
    /// Replaces the rotation component of the 4x4 matrix using the provided quaternion, without affecting the translation component.
    /// </summary>
    /// <param name="matrix">The matrix to modify. The rotation will replace the upper-left 3x3 portion of the matrix.</param>
    /// <param name="rotation">The quaternion representing the new rotation to apply.</param>
    /// <remarks>
    /// Quaternion magnitude does not affect the replacement rotation; zero
    /// represents identity. This method preserves the matrix's translation
    /// component. For complete transformation updates, use <see cref="SetTransform"/>.
    /// </remarks>
    /// <returns>A copy with the new rotation rows scaled by the original unsigned basis magnitudes.</returns>
    public static Fixed4x4 SetRotation(Fixed4x4 matrix, FixedQuaternion rotation)
    {
        Fixed3x3 rotationMatrix = rotation.ToMatrix3x3();

        Vector3d scale = ExtractScaleMagnitudes(matrix);

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
    /// Normalizes each basis row independently.
    /// </summary>
    /// <remarks>
    /// Zero rows remain zero. The angles between nonzero rows are preserved to fixed-point precision.
    /// </remarks>
    /// <param name="matrix">The matrix containing the three basis rows to normalize.</param>
    /// <returns>An affine matrix containing independently normalized basis rows and zero translation.</returns>
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
