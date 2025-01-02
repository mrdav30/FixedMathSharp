#if UNITY_EDITOR
using UnityEngine;

namespace FixedMathSharp
{
    public static partial class Vector2dExtensions
    {
        /// <summary>
        /// Converts a Unity Vector2 to a FixedMathSharp Vector2d.
        /// </summary>
        /// <param name="vec2">The Unity Vector2 to convert.</param>
        /// <returns>A FixedMathSharp Vector2d.</returns>
        public static Vector2d ToVector2d(this Vector2 vec2)
        {
            return new Vector2d(vec2.x, vec2.y);
        }

        /// <summary>
        /// Converts a Unity Vector3 to a FixedMathSharp Vector2d, using the x and z components of the Vector3.
        /// </summary>
        /// <param name="vec">The Unity Vector3 to convert.</param>
        /// <returns>A FixedMathSharp Vector2d.</returns>
        public static Vector2d ToVector2d(this Vector3 vec)
        {
            return new Vector2d(vec.x, vec.z);
        }

        /// <summary>
        /// Converts a FixedMathSharp Vector2d to a Unity Vector2.
        /// </summary>
        /// <param name="vec">The FixedMathSharp Vector2d to convert.</param>
        /// <returns>A Unity Vector2.</returns>
        public static Vector2 ToVector2(this Vector2d vec)
        {
            return new Vector2(vec.x.ToPreciseFloat(), vec.y.ToPreciseFloat());
        }

        /// <summary>
        /// Converts a FixedMathSharp Vector2d to a Unity Vector3, with the specified y-value.
        /// </summary>
        /// <param name="vec">The FixedMathSharp Vector2d to convert.</param>
        /// <param name="y">The y-value to use in the resulting Vector3.</param>
        /// <returns>A Unity Vector3.</returns>
        public static Vector3 ToVector3(this Vector2d vec, float y = 0f)
        {
            return new Vector3(vec.x.ToPreciseFloat(), y, vec.y.ToPreciseFloat());
        }
    }
}
#endif