#if UNITY_EDITOR
using UnityEngine;

namespace FixedMathSharp
{
    /// <summary>
    /// Provides extension methods for converting between Unity Vector types and FixedMathSharp Vector3d types.
    /// </summary>
    public static partial class Vector3dExtensions
    {
        /// <summary>
        /// Converts a Unity Vector2 to a FixedMathSharp Vector3d, using the x and y components from Vector2, 
        /// and assigning a specified y-value for the Vector3d's y-component.
        /// </summary>
        /// <param name="vec">The Unity Vector2 to convert.</param>
        /// <param name="y">The y-value for the Vector3d's y-component.</param>
        /// <returns>A FixedMathSharp Vector3d with the given components.</returns>
        public static Vector3d ToVector3d(this Vector2 vec, float y = 0)
        {
            return new Vector3d(vec.x, y, vec.y);
        }

        /// <summary>
        /// Converts a Unity Vector3 to a FixedMathSharp Vector3d.
        /// </summary>
        /// <param name="vec">The Unity Vector3 to convert.</param>
        /// <returns>A FixedMathSharp Vector3d with the corresponding components from the Unity Vector3.</returns>
        public static Vector3d ToVector3d(this Vector3 vec)
        {
            return new Vector3d(vec.x, vec.y, vec.z);
        }

        /// <summary>
        /// Converts a FixedMathSharp Vector3d to a Unity Vector2 by discarding the z-component.
        /// </summary>
        /// <param name="vec">The FixedMathSharp Vector3d to convert.</param>
        /// <returns>A Unity Vector2 with the x and y components from the FixedMathSharp Vector3d.</returns>
        public static Vector2 ToVector2(this Vector3d vec)
        {
            return new Vector2(vec.x.ToPreciseFloat(), vec.y.ToPreciseFloat());
        }

        /// <summary>
        /// Converts a FixedMathSharp Vector3d to a Unity Vector3.
        /// </summary>
        /// <param name="vec">The FixedMathSharp Vector3d to convert.</param>
        /// <returns>A Unity Vector3 with the corresponding components from the FixedMathSharp Vector3d.</returns>
        public static Vector3 ToVector3(this Vector3d vec)
        {
            return new Vector3(vec.x.ToPreciseFloat(), vec.y.ToPreciseFloat(), vec.z.ToPreciseFloat());
        }

        /// <summary>
        /// Converts a FixedMathSharp Vector3d to a Unity Vector3, assigning the y-component to a specific value.
        /// </summary>
        /// <param name="vec">The FixedMathSharp Vector3d to convert.</param>
        /// <param name="y">The value to assign to the y-component of the resulting Unity Vector3.</param>
        /// <returns>A Unity Vector3 with the specified y-component and the other components from the FixedMathSharp Vector3d.</returns>
        public static Vector3 ToVector3(this Vector3d vec, float y = 0f)
        {
            return new Vector3(vec.x.ToPreciseFloat(), y, vec.z.ToPreciseFloat());
        }

        /// <summary>
        /// Converts a FixedMathSharp Vector3d to a Unity Vector4, assigning a specified w-value for the Vector4's w-component.
        /// </summary>
        /// <param name="vec">The FixedMathSharp Vector3d to convert.</param>
        /// <param name="w">The w-value for the resulting Vector4's w-component.</param>
        /// <returns>A Unity Vector4 with the x, y, and z components from the FixedMathSharp Vector3d and the specified w-component.</returns>
        public static Vector4 ToVector4(this Vector3d vec, float w = 0f)
        {
            return new Vector4(vec.x.ToPreciseFloat(), vec.y.ToPreciseFloat(), vec.z.ToPreciseFloat(), w);
        }
    }
}
#endif