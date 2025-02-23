using System.Runtime.CompilerServices;

namespace FixedMathSharp
{
    public static class Fixed3x3Extensions
    {
        #region Transformations

        /// <inheritdoc cref="Fixed3x3.ExtractScale(Fixed3x3)" />
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static Vector3d ExtractScale(this Fixed3x3 matrix)
        {
            return Fixed3x3.ExtractScale(matrix);
        }

        /// <inheritdoc cref="Fixed3x3.SetScale(Fixed3x3, Vector3d)" />
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static Fixed3x3 SetScale(this ref Fixed3x3 matrix, Vector3d localScale)
        {
            return matrix = Fixed3x3.SetScale(matrix, localScale);
        }

        /// <inheritdoc cref="Fixed3x3.SetGlobalScale(Fixed3x3, Vector3d)" />
        public static Fixed3x3 SetGlobalScale(this ref Fixed3x3 matrix, Vector3d globalScale)
        {
            return matrix = Fixed3x3.SetGlobalScale(matrix, globalScale);
        }

        #endregion

        #region Equality

        /// <summary>
        /// Compares two Fixed3x3 for approximate equality, allowing a fixed absolute difference between components.
        /// </summary>
        /// <param name="f1">The current Fixed3x3.</param>
        /// <param name="f2">The Fixed3x3 to compare against.</param>
        /// <param name="allowedDifference">The allowed absolute difference between each component.</param>
        /// <returns>True if the components are within the allowed difference, false otherwise.</returns>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static bool FuzzyEqualAbsolute(this Fixed3x3 f1, Fixed3x3 f2, Fixed64 allowedDifference)
        {
            return (f1.m00 - f2.m00).Abs() <= allowedDifference &&
                   (f1.m01 - f2.m01).Abs() <= allowedDifference &&
                   (f1.m02 - f2.m02).Abs() <= allowedDifference &&
                   (f1.m10 - f2.m10).Abs() <= allowedDifference &&
                   (f1.m11 - f2.m11).Abs() <= allowedDifference &&
                   (f1.m12 - f2.m12).Abs() <= allowedDifference &&
                   (f1.m20 - f2.m20).Abs() <= allowedDifference &&
                   (f1.m21 - f2.m21).Abs() <= allowedDifference &&
                   (f1.m22 - f2.m22).Abs() <= allowedDifference;
        }

        /// <summary>
        /// Compares two Fixed3x3 for approximate equality, allowing a fractional percentage (defaults to ~1%) difference between components.
        /// </summary>
        /// <param name="f1">The current Fixed3x3.</param>
        /// <param name="f2">The Fixed3x3 to compare against.</param>
        /// <param name="percentage">The allowed fractional difference (percentage) for each component.</param>
        /// <returns>True if the components are within the allowed percentage difference, false otherwise.</returns>
        public static bool FuzzyEqual(this Fixed3x3 f1, Fixed3x3 f2, Fixed64? percentage = null)
        {
            Fixed64 p = percentage ?? Fixed64.Epsilon;
            return f1.m00.FuzzyComponentEqual(f2.m00, p) &&
                   f1.m01.FuzzyComponentEqual(f2.m01, p) &&
                   f1.m02.FuzzyComponentEqual(f2.m02, p) &&
                   f1.m10.FuzzyComponentEqual(f2.m10, p) &&
                   f1.m11.FuzzyComponentEqual(f2.m11, p) &&
                   f1.m12.FuzzyComponentEqual(f2.m12, p) &&
                   f1.m20.FuzzyComponentEqual(f2.m20, p) &&
                   f1.m21.FuzzyComponentEqual(f2.m21, p) &&
                   f1.m22.FuzzyComponentEqual(f2.m22, p);
        }

        #endregion
    }
}