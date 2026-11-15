//=======================================================================
// FixedQuaternion.Equality.cs
//=======================================================================
// MIT License, Copyright (c) 2024-present David Oravsky (mrdav30)
// See LICENSE file in the project root for full license information.
//=======================================================================

using System;
using System.Globalization;
using System.Runtime.CompilerServices;

namespace FixedMathSharp;

/// <content>
/// Equality, hashing, and string formatting for <see cref="FixedQuaternion"/>.
/// </content>
public partial struct FixedQuaternion
{
    #region Equality and HashCode Overrides

    /// <summary>Compares this quaternion with an object.</summary>
    /// <param name="obj">The object to compare.</param>
    /// <returns><see langword="true"/> if the object is a quaternion with all four corresponding components equal, otherwise <see langword="false"/>.</returns>
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public override bool Equals(object? obj) => obj is FixedQuaternion other && Equals(other);

    /// <summary>Compares the four stored components for exact equality.</summary>
    /// <param name="other">The quaternion to compare.</param>
    /// <returns><see langword="true"/> if all four corresponding components are exactly equal, otherwise <see langword="false"/>.</returns>
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public bool Equals(FixedQuaternion other) =>
        X == other.X && Y == other.Y && Z == other.Z && W == other.W;

    /// <summary>Computes a hash code of the quaternion components.</summary>
    /// <returns>A hash combining the X, Y, Z, and W raw component hash codes.</returns>
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public override int GetHashCode() =>
        X.GetHashCode() ^ Y.GetHashCode() << 2 ^ Z.GetHashCode() >> 2 ^ W.GetHashCode();

    /// <summary>
    /// Returns a string that represents the current object in the format "(x, y, z, w)".
    /// </summary>
    /// <returns>A parenthesized, comma-separated list in X, Y, Z, W order, using the default invariant numeric format.</returns>
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public override string ToString() => ToString(null, CultureInfo.InvariantCulture);

    /// <summary>
    /// Returns a string that represents the current object in the format "(x, y, z, w)".
    /// </summary>
    /// <param name="format">The numeric format for each component. A null or empty string selects the default format.</param>
    /// <param name="formatProvider">The provider used for all components. Null selects the invariant culture.</param>
    /// <returns>A parenthesized, comma-separated list of formatted components in X, Y, Z, W order.</returns>
    /// <exception cref="FormatException">The numeric format is invalid.</exception>
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
    /// <param name="destination">Caller-owned output span. This method writes from its start and may leave a partial prefix on failure.</param>
    /// <param name="charsWritten">The number of characters written on success, or zero when the destination is too small.</param>
    /// <param name="format">The numeric format for each component. An empty span selects the default format.</param>
    /// <param name="provider">The provider used for all components. Null selects the invariant culture.</param>
    /// <returns><see langword="true"/> if all four components and punctuation fit, otherwise <see langword="false"/>.</returns>
    /// <exception cref="FormatException">The numeric format is invalid.</exception>
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
