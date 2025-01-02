#if UNITY_EDITOR
using UnityEngine;

namespace FixedMathSharp
{
    /// <summary>
    /// Provides extension methods for converting between Unity Quaternions and FixedMathSharp FixedQuaternions.
    /// </summary>
    public static partial class FixedQuaternionExtensions
    {
        /// <summary>
        /// Converts a Unity Quaternion to a FixedMathSharp FixedQuaternion.
        /// </summary>
        /// <param name="quat">The Unity Quaternion to convert.</param>
        /// <returns>A FixedQuaternion with components corresponding to the input Unity Quaternion.</returns>
        public static FixedQuaternion ToFixedQuaternion(this Quaternion quat)
        {
            return new FixedQuaternion(
                (Fixed64)quat.x,
                (Fixed64)quat.y,
                (Fixed64)quat.z,
                (Fixed64)quat.w
            );
        }

        /// <summary>
        /// Converts a FixedMathSharp FixedQuaternion to a Unity Quaternion.
        /// </summary>
        /// <param name="quat">The FixedQuaternion to convert.</param>
        /// <returns>A Unity Quaternion with components corresponding to the input FixedQuaternion.</returns>
        public static Quaternion ToQuaternion(this FixedQuaternion quat)
        {
            return new Quaternion(
                (float)quat.x,
                (float)quat.y,
                (float)quat.z,
                (float)quat.w
            );
        }
    }
}
#endif