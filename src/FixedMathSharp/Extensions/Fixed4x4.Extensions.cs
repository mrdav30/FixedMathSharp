using System.Runtime.CompilerServices;

namespace FixedMathSharp
{
    public static class Fixed4x4Extensions
    {
        #region Extraction, and Setters

        /// <inheritdoc cref="Fixed4x4.ExtractScale(Fixed4x4)" />
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static Vector3d ExtractScale(this Fixed4x4 matrix)
        {
            return Fixed4x4.ExtractScale(matrix);
        }

        /// <inheritdoc cref="Fixed4x4.ExtractLossyScale(Fixed4x4)" />
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static Vector3d ExtractLossyScale(this Fixed4x4 matrix)
        {
            return Fixed4x4.ExtractLossyScale(matrix);
        }

        /// <inheritdoc cref="Fixed4x4.ExtractTranslation(Fixed4x4)" />
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static Vector3d ExtractTranslation(this Fixed4x4 matrix)
        {
            return Fixed4x4.ExtractTranslation(matrix);
        }

        /// <inheritdoc cref="Fixed4x4.ExtractRotation(Fixed4x4)" />
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static FixedQuaternion ExtractRotation(this Fixed4x4 matrix)
        {
            return Fixed4x4.ExtractRotation(matrix);
        }

        /// <inheritdoc cref="Fixed4x4.SetGlobalScale(Fixed4x4, Vector3d)" />
        public static Fixed4x4 SetGlobalScale(this ref Fixed4x4 matrix, Vector3d globalScale)
        {
            return matrix = Fixed4x4.SetGlobalScale(matrix, globalScale);
        }

        /// <inheritdoc cref="Fixed4x4.TransformPoint(Fixed4x4, Vector3d)" />
        public static Vector3d TransformPoint(this Fixed4x4 matrix, Vector3d point)
        {
            return Fixed4x4.TransformPoint(matrix, point);
        }

        /// <inheritdoc cref="Fixed4x4.InverseTransformPoint(Fixed4x4, Vector3d)" />
        public static Vector3d InverseTransformPoint(this Fixed4x4 matrix, Vector3d point)
        {
            return Fixed4x4.InverseTransformPoint(matrix, point);
        }

        #endregion
    }
}