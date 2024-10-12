namespace FixedMathSharp
{
    public static class FixedMatrix3x3Extensions
    {
        /// <summary>
        /// Extracts the scaling factors from the matrix by returning the diagonal elements.
        /// </summary>
        /// <returns>A Vector3d representing the scale along X, Y, and Z axes.</returns>
        public static Vector3d ExtractScale(this FixedMatrix3x3 matrix)
        {
            return new Vector3d(
                matrix.m00, // Scale along X-axis
                matrix.m11, // Scale along Y-axis
                matrix.m22  // Scale along Z-axis
            );
        }

        /// <summary>
        /// Resets the scaling part of the matrix to identity (1,1,1).
        /// </summary>
        public static void SetIdentityScale(this ref FixedMatrix3x3 matrix)
        {
            matrix.m00 = FixedMath.One;  // Reset scale on X-axis
            matrix.m11 = FixedMath.One;  // Reset scale on Y-axis
            matrix.m22 = FixedMath.One;  // Reset scale on Z-axis
        }

        /// <summary>
        /// Applies the provided local scale to the matrix by modifying the diagonal elements.
        /// </summary>
        /// <param name="localScale">A Vector3d representing the local scale to apply.</param>
        public static void ApplyScale(this ref FixedMatrix3x3 matrix, Vector3d localScale)
        {
            matrix.m00 *= localScale.x; // Apply scale on X-axis
            matrix.m11 *= localScale.y; // Apply scale on Y-axis
            matrix.m22 *= localScale.z; // Apply scale on Z-axis
        }
    }
}