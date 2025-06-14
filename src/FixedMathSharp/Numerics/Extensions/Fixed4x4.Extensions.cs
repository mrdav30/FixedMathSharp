using System.Runtime.CompilerServices;

namespace FixedMathSharp
{
    public static class Fixed4x4Extensions
    {
        #region Extraction, and Setters

        /// <inheritdoc cref="Fixed4x4.ExtractLossyScale(Fixed4x4)" />
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static Vector3d ExtractLossyScale(this Fixed4x4 matrix)
        {
            return Fixed4x4.ExtractLossyScale(matrix);
        }

        /// <inheritdoc cref="Fixed4x4.SetGlobalScale(Fixed4x4, Vector3d)" />
        public static Fixed4x4 SetGlobalScale(this ref Fixed4x4 matrix, Vector3d globalScale)
        {
            return matrix = Fixed4x4.SetGlobalScale(matrix, globalScale);
        }

        /// <inheritdoc cref="Fixed4x4.SetTranslation(Fixed4x4, Vector3d)" />
        public static Fixed4x4 SetTranslation(this ref Fixed4x4 matrix, Vector3d position)
        {
            return matrix = Fixed4x4.SetTranslation(matrix, position);
        }

        /// <inheritdoc cref="Fixed4x4.SetRotation(Fixed4x4, FixedQuaternion)" />
        public static Fixed4x4 SetRotation(this ref Fixed4x4 matrix, FixedQuaternion rotation)
        {
            return matrix = Fixed4x4.SetRotation(matrix, rotation);
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