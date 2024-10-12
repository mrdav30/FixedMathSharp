namespace FixedMathSharp
{
    public static class FixedMatrix4x4Extensions
    {
        /// <summary>
        /// Extracts the scaling factors from the matrix by returning the diagonal elements.
        /// </summary>
        /// <returns>A Vector3d representing the scale along X, Y, and Z axes.</returns>
        public static Vector3d ExtractScale(this FixedMatrix4x4 matrix)
        {
            return new Vector3d(
                matrix.m00,  // X scale
                matrix.m11,  // Y scale
                matrix.m22   // Z scale
            );
        }

        /// <summary>
        /// Resets the scaling part of the matrix to identity (1,1,1).
        /// </summary>
        public static void SetIdentityScale(this ref FixedMatrix4x4 matrix)
        {
            matrix.m00 = FixedMath.One;  // X scale
            matrix.m11 = FixedMath.One;  // Y scale
            matrix.m22 = FixedMath.One;  // Z scale
        }

        /// <summary>
        /// Applies the provided local scale to the matrix by modifying the diagonal elements.
        /// </summary>
        /// <param name="localScale">A Vector3d representing the local scale to apply.</param>
        public static void ApplyScale(this ref FixedMatrix4x4 matrix, Vector3d localScale)
        {
            matrix.m00 *= localScale.x;  // X scale
            matrix.m11 *= localScale.y;  // Y scale
            matrix.m22 *= localScale.z;  // Z scale
        }
    }
}