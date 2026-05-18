using System.Runtime.CompilerServices;

namespace FixedMathSharp;

/// <summary>
/// Provides extension methods for the Vector4d type to support additional vector operations and comparisons.
/// </summary>
public static partial class Vector4dExtensions
{
    #region Vector4d Operations

    /// <summary>
    /// Clamps each component of the vector to the range [-1, 1] and returns the clamped vector.
    /// </summary>
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public static Vector4d ClampOneInPlace(this Vector4d v)
    {
        v.x = v.x.ClampOne();
        v.y = v.y.ClampOne();
        v.z = v.z.ClampOne();
        v.w = v.w.ClampOne();
        return v;
    }

    #endregion

    #region Conversion

    /// <inheritdoc cref="Vector4d.Abs(Vector4d)" />
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public static Vector4d Abs(this Vector4d value)
    {
        return Vector4d.Abs(value);
    }

    /// <inheritdoc cref="Vector4d.Sign(Vector4d)" />
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public static Vector4d Sign(this Vector4d value)
    {
        return Vector4d.Sign(value);
    }

    #endregion

    #region Equality

    /// <summary>
    /// Compares two vectors for approximate equality, allowing a fixed absolute difference.
    /// </summary>
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public static bool FuzzyEqualAbsolute(this Vector4d me, Vector4d other, Fixed64 allowedDifference)
    {
        return (me.x - other.x).Abs() <= allowedDifference &&
               (me.y - other.y).Abs() <= allowedDifference &&
               (me.z - other.z).Abs() <= allowedDifference &&
               (me.w - other.w).Abs() <= allowedDifference;
    }

    /// <summary>
    /// Compares two vectors for approximate equality, allowing a fractional difference.
    /// </summary>
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public static bool FuzzyEqual(this Vector4d me, Vector4d other, Fixed64? percentage = null)
    {
        Fixed64 p = percentage ?? Fixed64.Epsilon;
        return me.x.FuzzyComponentEqual(other.x, p) &&
               me.y.FuzzyComponentEqual(other.y, p) &&
               me.z.FuzzyComponentEqual(other.z, p) &&
               me.w.FuzzyComponentEqual(other.w, p);
    }

    #endregion
}
