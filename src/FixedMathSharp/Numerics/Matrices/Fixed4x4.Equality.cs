//=======================================================================
// Fixed4x4.Equality.cs
//=======================================================================
// MIT License, Copyright (c) 2024–present David Oravsky (mrdav30)
// See LICENSE file in the project root for full license information.
//=======================================================================

using System;
using System.Globalization;
using System.Runtime.CompilerServices;

namespace FixedMathSharp;

public partial struct Fixed4x4
{
    #region Equality and HashCode Overrides

    /// <inheritdoc/>
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public override bool Equals(object? obj) => obj is Fixed4x4 x && Equals(x);

    /// <inheritdoc/>
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public bool Equals(Fixed4x4 other) =>
        M11 == other.M11 && M12 == other.M12 && M13 == other.M13 && M14 == other.M14 &&
        M21 == other.M21 && M22 == other.M22 && M23 == other.M23 && M24 == other.M24 &&
        M31 == other.M31 && M32 == other.M32 && M33 == other.M33 && M34 == other.M34 &&
        M41 == other.M41 && M42 == other.M42 && M43 == other.M43 && M44 == other.M44;

    /// <inheritdoc/>
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public override int GetHashCode()
    {
        unchecked
        {
            int hash = 17;
            hash = hash * 23 + M11.GetHashCode();
            hash = hash * 23 + M12.GetHashCode();
            hash = hash * 23 + M13.GetHashCode();
            hash = hash * 23 + M14.GetHashCode();
            hash = hash * 23 + M21.GetHashCode();
            hash = hash * 23 + M22.GetHashCode();
            hash = hash * 23 + M23.GetHashCode();
            hash = hash * 23 + M24.GetHashCode();
            hash = hash * 23 + M31.GetHashCode();
            hash = hash * 23 + M32.GetHashCode();
            hash = hash * 23 + M33.GetHashCode();
            hash = hash * 23 + M34.GetHashCode();
            hash = hash * 23 + M41.GetHashCode();
            hash = hash * 23 + M42.GetHashCode();
            hash = hash * 23 + M43.GetHashCode();
            hash = hash * 23 + M44.GetHashCode();
            return hash;
        }
    }

    /// <summary>
    /// Returns a string that represents the current matrix in a readable format.
    /// </summary>
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public override string ToString() => ToString(null, CultureInfo.InvariantCulture);

    /// <summary>
    /// Returns a string that represents the current matrix in a readable format.
    /// </summary>
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public string ToString(string? format, IFormatProvider? formatProvider)
    {
        Fixed4x4 value = this;
        return FixedDiagnosticsFormatter.ToString((Span<char> destination, out int charsWritten) =>
            value.TryFormat(destination, out charsWritten, format.AsSpan(), formatProvider));
    }

    /// <summary>
    /// Formats this matrix into the provided destination buffer.
    /// </summary>
    public bool TryFormat(
        Span<char> destination,
        out int charsWritten,
        ReadOnlySpan<char> format,
        IFormatProvider? provider)
    {
        int written = 0;
        if (!FixedDiagnosticsFormatter.Append('[', destination, ref written) ||
            !AppendRow(M11, M12, M13, M14, destination, ref written, format, provider) ||
            !FixedDiagnosticsFormatter.Append("; ", destination, ref written) ||
            !AppendRow(M21, M22, M23, M24, destination, ref written, format, provider) ||
            !FixedDiagnosticsFormatter.Append("; ", destination, ref written) ||
            !AppendRow(M31, M32, M33, M34, destination, ref written, format, provider) ||
            !FixedDiagnosticsFormatter.Append("; ", destination, ref written) ||
            !AppendRow(M41, M42, M43, M44, destination, ref written, format, provider) ||
            !FixedDiagnosticsFormatter.Append(']', destination, ref written))
        {
            charsWritten = 0;
            return false;
        }

        charsWritten = written;
        return true;
    }

    private static bool AppendRow(
        Fixed64 x,
        Fixed64 y,
        Fixed64 z,
        Fixed64 w,
        Span<char> destination,
        ref int charsWritten,
        ReadOnlySpan<char> format,
        IFormatProvider? provider)
    {
        return FixedDiagnosticsFormatter.Append(x, destination, ref charsWritten, format, provider) &&
               FixedDiagnosticsFormatter.Append(", ", destination, ref charsWritten) &&
               FixedDiagnosticsFormatter.Append(y, destination, ref charsWritten, format, provider) &&
               FixedDiagnosticsFormatter.Append(", ", destination, ref charsWritten) &&
               FixedDiagnosticsFormatter.Append(z, destination, ref charsWritten, format, provider) &&
               FixedDiagnosticsFormatter.Append(", ", destination, ref charsWritten) &&
               FixedDiagnosticsFormatter.Append(w, destination, ref charsWritten, format, provider);
    }

    #endregion
}
