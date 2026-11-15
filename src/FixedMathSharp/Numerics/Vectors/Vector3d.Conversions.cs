//=======================================================================
// Vector3d.Conversions.cs
//=======================================================================
// MIT License, Copyright (c) 2024-present David Oravsky (mrdav30)
// See LICENSE file in the project root for full license information.
//=======================================================================

using System;
using System.Globalization;
using System.Runtime.CompilerServices;

namespace FixedMathSharp;

/// <content>
/// Conversion, deconstruction, and formatting utilities for <see cref="Vector3d"/>.
/// </content>
public partial struct Vector3d
{
    /// <summary>
    /// Returns a string that represents the current object in the format "(x, y, z)".
    /// </summary>
    /// <returns>A parenthesized, comma-separated list of components, using the default invariant numeric format.</returns>
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public override string ToString() => ToString(null, CultureInfo.InvariantCulture);

    /// <summary>
    /// Returns a string that represents the current object in the format "(x, y, z)".
    /// </summary>
    /// <param name="format">The numeric format for each component. A null or empty string uses the default format.</param>
    /// <param name="formatProvider">The provider used for all components. Null selects the invariant culture.</param>
    /// <returns>A parenthesized, comma-separated list of formatted components.</returns>
    /// <exception cref="FormatException">The component format is invalid.</exception>
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public string ToString(string? format, IFormatProvider? formatProvider)
    {
        Vector3d value = this;
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
    public bool TryFormat(
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
            !FixedDiagnosticsFormatter.Append(", ", destination, ref written) ||
            !FixedDiagnosticsFormatter.Append(Z, destination, ref written, format, provider) ||
            !FixedDiagnosticsFormatter.Append(')', destination, ref written))
        {
            charsWritten = 0;
            return false;
        }

        charsWritten = written;
        return true;
    }

    /// <summary>
    /// Converts this <see cref="Vector3d"/> to a <see cref="Vector2d"/>, 
    /// dropping the Y component (height) of this vector in the resulting vector.
    /// </summary>
    /// <returns>
    /// A new <see cref="Vector2d"/> where (X, Z) from this <see cref="Vector3d"/> 
    /// become (X, Y) in the resulting vector.
    /// </returns>
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public Vector2d ToVector2d() => new(X, Z);

    /// <summary>
    /// Converts this <see cref="Vector3d"/> to a <see cref="Vector4d"/> with an explicit W component.
    /// </summary>
    /// <param name="w">The new W component.</param>
    /// <returns>The existing X, Y, and Z components with the supplied W component.</returns>
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public Vector4d ToVector4d(Fixed64 w) => new(X, Y, Z, w);

    /// <summary>
    /// Deconstructs the Vector3d into its three Fixed64 components.
    /// </summary>
    /// <param name="x">The fixed-point X component.</param>
    /// <param name="y">The fixed-point Y component.</param>
    /// <param name="z">The fixed-point Z component.</param>
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public void Deconstruct(out Fixed64 x, out Fixed64 y, out Fixed64 z)
    {
        x = X;
        y = Y;
        z = Z;
    }

    /// <summary>
    /// Deconstructs the Vector3d into its three int components.
    /// </summary>
    /// <param name="x">The X component rounded to the nearest integer with ties to even.</param>
    /// <param name="y">The Y component rounded to the nearest integer with ties to even.</param>
    /// <param name="z">The Z component rounded to the nearest integer with ties to even.</param>
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public void Deconstruct(out int x, out int y, out int z)
    {
        x = X.RoundToInt();
        y = Y.RoundToInt();
        z = Z.RoundToInt();
    }

    /// <summary>
    /// Deconstructs the Vector3d into its three long components.
    /// </summary>
    /// <param name="x">The signed 64-bit raw Q32.32 payload of the X component.</param>
    /// <param name="y">The signed 64-bit raw Q32.32 payload of the Y component.</param>
    /// <param name="z">The signed 64-bit raw Q32.32 payload of the Z component.</param>
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public void Deconstruct(out long x, out long y, out long z)
    {
        x = X.m_rawValue;
        y = Y.m_rawValue;
        z = Z.m_rawValue;
    }

    /// <summary>
    /// Deconstructs the Vector3d into its three double components.
    /// </summary>
    /// <param name="x">The double-precision approximation of the X component.</param>
    /// <param name="y">The double-precision approximation of the Y component.</param>
    /// <param name="z">The double-precision approximation of the Z component.</param>
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public void Deconstruct(out double x, out double y, out double z)
    {
        x = (double)X;
        y = (double)Y;
        z = (double)Z;
    }

    /// <summary>
    /// Converts each component of the vector from radians to degrees.
    /// </summary>
    /// <param name="radians">The vector with components in radians.</param>
    /// <returns>A new vector with components converted to degrees.</returns>
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public static Vector3d ToDegrees(Vector3d radians) =>
         new(FixedMath.RadToDeg(radians.X),
             FixedMath.RadToDeg(radians.Y),
             FixedMath.RadToDeg(radians.Z));

    /// <summary>
    /// Converts each component of the vector from degrees to radians.
    /// </summary>
    /// <param name="degrees">The vector with components in degrees.</param>
    /// <returns>A new vector with components converted to radians.</returns>
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public static Vector3d ToRadians(Vector3d degrees) =>
        new(FixedMath.DegToRad(degrees.X),
            FixedMath.DegToRad(degrees.Y),
            FixedMath.DegToRad(degrees.Z));
}
