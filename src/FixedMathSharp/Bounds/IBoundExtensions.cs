using System.Runtime.CompilerServices;

namespace FixedMathSharp
{
    internal static class IBoundExtensions
    {
        /// <inheritdoc cref="IBound.ProjectPoint(Vector3d)" />
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static Vector3d ProjectPointWithinBounds(this IBound bounds, Vector3d point)
        {
            return new Vector3d(
                FixedMath.Clamp(point.x, bounds.Min.x, bounds.Max.x),
                FixedMath.Clamp(point.y, bounds.Min.y, bounds.Max.y),
                FixedMath.Clamp(point.z, bounds.Min.z, bounds.Max.z)
            );
        }
    }
}
