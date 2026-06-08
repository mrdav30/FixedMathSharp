//=======================================================================
// FixedDiagnosticsFormatter.cs
//=======================================================================
// MIT License, Copyright (c) 2024-present David Oravsky (mrdav30)
// See LICENSE file in the project root for full license information.
//=======================================================================

using System;
using System.Buffers;
using System.Globalization;
using System.Runtime.CompilerServices;

namespace FixedMathSharp;

internal static class FixedDiagnosticsFormatter
{
    internal delegate bool TryFormatHandler(Span<char> destination, out int charsWritten);

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    internal static bool TryFormat(
        Fixed64 value,
        Span<char> destination,
        out int charsWritten,
        ReadOnlySpan<char> format,
        IFormatProvider? provider)
    {
        return ((double)value).TryFormat(
            destination,
            out charsWritten,
            format,
            provider ?? CultureInfo.InvariantCulture);
    }

    internal static string ToString(TryFormatHandler handler)
    {
        Span<char> initialBuffer = stackalloc char[256];
        if (handler(initialBuffer, out int charsWritten))
            return new string(initialBuffer[..charsWritten]);

        int length = 512;
        while (true)
        {
            char[] rented = ArrayPool<char>.Shared.Rent(length);
            try
            {
                if (handler(rented.AsSpan(0, length), out charsWritten))
                    return new string(rented, 0, charsWritten);
            }
            finally
            {
                ArrayPool<char>.Shared.Return(rented);
            }

            length *= 2;
        }
    }

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    internal static bool Append(char value, Span<char> destination, ref int charsWritten)
    {
        if ((uint)charsWritten >= (uint)destination.Length)
            return false;

        destination[charsWritten++] = value;
        return true;
    }

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    internal static bool Append(ReadOnlySpan<char> value, Span<char> destination, ref int charsWritten)
    {
        if (value.Length > destination.Length - charsWritten)
            return false;

        value.CopyTo(destination[charsWritten..]);
        charsWritten += value.Length;
        return true;
    }

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    internal static bool Append(
        Fixed64 value,
        Span<char> destination,
        ref int charsWritten,
        ReadOnlySpan<char> format,
        IFormatProvider? provider)
    {
        if (!TryFormat(value, destination[charsWritten..], out int written, format, provider))
            return false;

        charsWritten += written;
        return true;
    }
}
