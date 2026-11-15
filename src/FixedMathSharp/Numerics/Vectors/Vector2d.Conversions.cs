//=======================================================================
// Vector2d.Conversions.cs
//=======================================================================
// MIT License, Copyright (c) 2024-present David Oravsky (mrdav30)
// See LICENSE file in the project root for full license information.
//=======================================================================

using System;
using System.Globalization;
using System.Runtime.CompilerServices;

namespace FixedMathSharp;

/// <content>
/// Conversion, formatting, and deconstruction utilities for <see cref="Vector2d"/>.
/// </content>
public partial struct Vector2d
{
    /// <summary>
    /// Returns a string representation of this vector.
    /// </summary>
    /// <returns>A parenthesized, comma-separated list of components, using the default invariant numeric format.</returns>
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public override readonly string ToString() => ToString(null, CultureInfo.InvariantCulture);

    /// <summary>
    /// Returns a string representation of this vector.
    /// </summary>
    /// <param name="format">The numeric format for each component. A null or empty string uses the default format.</param>
    /// <param name="formatProvider">The provider used for all components. Null selects the invariant culture.</param>
    /// <returns>A parenthesized, comma-separated list of formatted components.</returns>
    /// <exception cref="FormatException">The component format is invalid.</exception>
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public readonly string ToString(string? format, IFormatProvider? formatProvider)
    {
        Vector2d value = this;
        return FixedDiagnosticsFormatter.ToString((Span<char> destination, out int charsWritten) =>
            value.TryFormat(destination, out charsWritten, format.AsSpan(), formatProvider));
    }

    /// <summary>
    /// Formats this vector into the provided destination buffer.
    /// </summary>
    /// <param name="destination">The caller-owned output span. This method writes from its start and may leave a partial prefix on failure.</param>
    /// <param name="charsWritten">The number of characters written on success, or zero when the destination is too small.</param>
    /// <param name="format">The numeric format for each component. An empty span selects the default format.</param>
    /// <param name="provider">The provider used for all components. Null selects the invariant culture.</param>
    /// <returns><see langword="true"/> if all components and punctuation fit, otherwise <see langword="false"/>.</returns>
    /// <exception cref="FormatException">The component format is invalid.</exception>
    public readonly bool TryFormat(
        Span<char> destination,
        out int charsWritten,
        ReadOnlySpan<char> format,
        IFormatProvider? provider)
    {
        int written = 0;
        if (!FixedDiagnosticsFormatter.Append('(', destination, ref written) ||
            !FixedDiagnosticsFormatter.Append(X, destination, ref written, format, provider) ||
            !FixedDiagnosticsFormatter.Append(", ", destination, ref written) ||
            !FixedDiagnosticsFormatter.Append(Y, destination, ref written, format, provider) ||
            !FixedDiagnosticsFormatter.Append(')', destination, ref written))
        {
            charsWritten = 0;
            return false;
        }

        charsWritten = written;
        return true;
    }

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
    /// <param name="z">The new Z component.</param>
    /// <param name="w">The new W component.</param>
    /// <returns>The existing X and Y components with the supplied Z and W components.</returns>
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public Vector4d ToVector4d(Fixed64 z, Fixed64 w) => new(X, Y, z, w);

    /// <summary>
    /// Deconstructs the Vector2d into its two Fixed64 components.
    /// </summary>
    /// <param name="x">The fixed-point X component.</param>
    /// <param name="y">The fixed-point Y component.</param>
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public void Deconstruct(out Fixed64 x, out Fixed64 y)
    {
        x = X;
        y = Y;
    }

    /// <summary>
    /// Deconstructs the Vector2d into its two int components.
    /// </summary>
    /// <param name="x">The X component rounded to the nearest integer with ties to even.</param>
    /// <param name="y">The Y component rounded to the nearest integer with ties to even.</param>
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public void Deconstruct(out int x, out int y)
    {
        x = X.RoundToInt();
        y = Y.RoundToInt();
    }

    /// <summary>
    /// Deconstructs the Vector2d into its two long components.
    /// </summary>
    /// <param name="x">The signed 64-bit raw Q32.32 payload of the X component.</param>
    /// <param name="y">The signed 64-bit raw Q32.32 payload of the Y component.</param>
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public void Deconstruct(out long x, out long y)
    {
        x = X.m_rawValue;
        y = Y.m_rawValue;
    }

    /// <summary>
    /// Deconstructs the Vector2d into its two double components.
    /// </summary>
    /// <param name="x">The double-precision approximation of the X component.</param>
    /// <param name="y">The double-precision approximation of the Y component.</param>
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
}
