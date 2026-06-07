//=======================================================================
// Vector2d.Conversions.cs
//=======================================================================
// MIT License, Copyright (c) 2024–present David Oravsky (mrdav30)
// See LICENSE file in the project root for full license information.
//=======================================================================

using System.Runtime.CompilerServices;

namespace FixedMathSharp;

public partial struct Vector2d
{
    #region Conversion

    /// <summary>
    /// Returns a string representation of this vector.
    /// </summary>
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public override readonly string ToString() =>
        string.Format("({0}, {1})",
            (double)X,
            (double)Y);

    /// <summary>
    /// Returns a string representation of this vector with components formatted to a fixed number of decimal places.
    /// </summary>
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public readonly string ToFormattedString() =>
        string.Format("({0}, {1})",
            X.ToFormattedDouble(),
            Y.ToFormattedDouble());

    /// <summary>
    /// Converts this <see cref="Vector2d"/> to a <see cref="Vector3d"/>, 
    /// mapping the Y component of this vector to the Z axis in the resulting vector.
    /// </summary>
    /// <param name="z">The value to assign to the Y axis of the resulting <see cref="Vector3d"/>.</param>
    /// <returns>
    /// A new <see cref="Vector3d"/> where (X, Y) from this <see cref="Vector2d"/> 
    /// become (X, Z) in the resulting vector, with the provided Z parameter assigned to Y.
    /// </returns>
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public Vector3d ToVector3d(Fixed64 z) => new(X, z, Y);

    /// <summary>
    /// Converts this <see cref="Vector2d"/> to a <see cref="Vector4d"/> with explicit Z and W components.
    /// </summary>
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public Vector4d ToVector4d(Fixed64 z, Fixed64 w) => new(X, Y, z, w);

    /// <summary>
    /// Deconstructs the Vector2d into its two Fixed64 components.
    /// </summary>
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public void Deconstruct(out Fixed64 x, out Fixed64 y)
    {
        x = X;
        y = Y;
    }

    /// <summary>
    /// Deconstructs the Vector2d into its two int components.
    /// </summary>
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public void Deconstruct(out int x, out int y)
    {
        x = X.RoundToInt();
        y = Y.RoundToInt();
    }

    /// <summary>
    /// Deconstructs the Vector2d into its two long components.
    /// </summary>
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public void Deconstruct(out long x, out long y)
    {
        x = X.m_rawValue;
        y = Y.m_rawValue;
    }

    /// <summary>
    /// Deconstructs the Vector2d into its two double components.
    /// </summary>
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public void Deconstruct(out double x, out double y)
    {
        x = (double)X;
        y = (double)Y;
    }

    /// <summary>
    /// Converts each component of the vector from radians to degrees.
    /// </summary>
    /// <param name="radians">The vector with components in radians.</param>
    /// <returns>A new vector with components converted to degrees.</returns>
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public static Vector2d ToDegrees(Vector2d radians) =>
        new(FixedMath.RadToDeg(radians.X),
            FixedMath.RadToDeg(radians.Y));

    /// <summary>
    /// Converts each component of the vector from degrees to radians.
    /// </summary>
    /// <param name="degrees">The vector with components in degrees.</param>
    /// <returns>A new vector with components converted to radians.</returns>
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public static Vector2d ToRadians(Vector2d degrees) =>
        new(FixedMath.DegToRad(degrees.X),
            FixedMath.DegToRad(degrees.Y));

    #endregion
}
