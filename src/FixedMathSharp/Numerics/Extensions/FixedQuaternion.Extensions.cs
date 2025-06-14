using System.Runtime.CompilerServices;

namespace FixedMathSharp
{
    public static partial class FixedQuaternionExtensions
    {
        /// <inheritdoc cref="FixedQuaternion.ToAngularVelocity" />
        public static Vector3d ToAngularVelocity(
            this FixedQuaternion currentRotation,
            FixedQuaternion previousRotation,
            Fixed64 deltaTime)
        {
            return FixedQuaternion.ToAngularVelocity(currentRotation, previousRotation, deltaTime);
        }

        #region Equality

        /// <summary>
        /// Compares two quaternions for approximate equality, allowing a fixed absolute difference between components.
        /// </summary>
        /// <param name="q1">The current quaternion.</param>
        /// <param name="q2">The quaternion to compare against.</param>
        /// <param name="allowedDifference">The allowed absolute difference between each component.</param>
        /// <returns>True if the components are within the allowed difference, false otherwise.</returns>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static bool FuzzyEqualAbsolute(this FixedQuaternion q1, FixedQuaternion q2, Fixed64 allowedDifference)
        {
            return (q1.x - q2.x).Abs() <= allowedDifference &&
                   (q1.y - q2.y).Abs() <= allowedDifference &&
                   (q1.z - q2.z).Abs() <= allowedDifference &&
                   (q1.w - q2.w).Abs() <= allowedDifference;
        }

        /// <summary>
        /// Compares two quaternions for approximate equality, allowing a fractional percentage (defaults to ~1%) difference between components.
        /// </summary>
        /// <param name="q1">The current quaternion.</param>
        /// <param name="q2">The quaternion to compare against.</param>
        /// <param name="percentage">The allowed fractional difference (percentage) for each component.</param>
        /// <returns>True if the components are within the allowed percentage difference, false otherwise.</returns>
        public static bool FuzzyEqual(this FixedQuaternion q1, FixedQuaternion q2, Fixed64? percentage = null)
        {
            Fixed64 p = percentage ?? Fixed64.Epsilon;
            return q1.x.FuzzyComponentEqual(q2.x, p) &&
                   q1.y.FuzzyComponentEqual(q2.y, p) &&
                   q1.z.FuzzyComponentEqual(q2.z, p) &&
                   q1.w.FuzzyComponentEqual(q2.w, p);
        }

        #endregion
    }
}