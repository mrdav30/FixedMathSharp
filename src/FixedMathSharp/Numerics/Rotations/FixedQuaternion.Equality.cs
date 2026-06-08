//=======================================================================
// FixedQuaternion.Equality.cs
//=======================================================================
// MIT License, Copyright (c) 2024–present David Oravsky (mrdav30)
// See LICENSE file in the project root for full license information.
//=======================================================================

using System;
using System.Globalization;
using System.Runtime.CompilerServices;

namespace FixedMathSharp;

public partial struct FixedQuaternion
{
    #region Equality and HashCode Overrides

    /// <inheritdoc/>
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public override bool Equals(object? obj) => obj is FixedQuaternion other && Equals(other);

    /// <inheritdoc/>
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public bool Equals(FixedQuaternion other) =>
        X == other.X && Y == other.Y && Z == other.Z && W == other.W;

    /// <inheritdoc/>
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public override int GetHashCode() =>
        X.GetHashCode() ^ Y.GetHashCode() << 2 ^ Z.GetHashCode() >> 2 ^ W.GetHashCode();

    /// <summary>
    /// Returns a string that represents the current object in the format "(x, y, z, w)".
    /// </summary>
    /// <returns>A string containing the values of the object formatted as a tuple.</returns>
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public override string ToString() => ToString(null, CultureInfo.InvariantCulture);

    /// <summary>
    /// Returns a string that represents the current object in the format "(x, y, z, w)".
    /// </summary>
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public string ToString(string? format, IFormatProvider? formatProvider)
    {
        FixedQuaternion value = this;
        return FixedDiagnosticsFormatter.ToString((Span<char> destination, out int charsWritten) =>
            value.TryFormat(destination, out charsWritten, format.AsSpan(), formatProvider));
    }

    /// <summary>
    /// Formats this quaternion into the provided destination buffer.
    /// </summary>
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
            !FixedDiagnosticsFormatter.Append(", ", destination, ref written) ||
            !FixedDiagnosticsFormatter.Append(W, destination, ref written, format, provider) ||
            !FixedDiagnosticsFormatter.Append(')', destination, ref written))
        {
            charsWritten = 0;
            return false;
        }

        charsWritten = written;
        return true;
    }

    #endregion
}
