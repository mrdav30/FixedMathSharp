//=======================================================================
// Signed192.cs
//=======================================================================
// MIT License, Copyright (c) 2024–present David Oravsky (mrdav30)
// See LICENSE file in the project root for full license information.
//=======================================================================

using System.Runtime.CompilerServices;

namespace FixedMathSharp;

/// <summary>
/// A signed two's-complement value wide enough for sums of full-domain
/// Q32.32 endpoint-difference products.
/// </summary>
internal readonly struct Signed192
{
    public static Signed192 One => Signed(Fixed64.One.m_rawValue);

    internal readonly ulong High;
    internal readonly ulong Middle;
    internal readonly ulong Low;

    internal Signed192(ulong high, ulong middle, ulong low)
    {
        High = high;
        Middle = middle;
        Low = low;
    }

    internal bool IsZero => (High | Middle | Low) == 0UL;

    internal int Sign => IsZero ? 0 : (High & (1UL << 63)) != 0UL ? -1 : 1;

    internal static Signed192 Raw(Fixed64 value) => Signed(value.m_rawValue);

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    internal static Signed192 Signed(long value)
    {
        ulong extension = value < 0L ? ulong.MaxValue : 0UL;
        return new Signed192(extension, extension, unchecked((ulong)value));
    }

    /// <summary>
    /// Narrows a five-word signed value to a three-word signed value, discarding the two most significant words.
    /// </summary>
    internal static Signed192 NarrowValue(Signed320 value) => new(value.Word2, value.Word1, value.Word0);

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    internal static bool TryNarrowSigned(Signed320 value, out Signed192 result)
    {
        ulong extension = (value.Word2 & (1UL << 63)) != 0UL ? ulong.MaxValue : 0UL;
        if (value.Word4 != extension || value.Word3 != extension)
        {
            result = default;
            return false;
        }

        result = new Signed192(value.Word2, value.Word1, value.Word0);
        return true;
    }

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    internal static Signed192 NarrowProven(Signed320 value)
    {
        _ = TryNarrowSigned(value, out Signed192 result);
        return result;
    }

    /// <summary>
    /// Narrow a Signed576 value to a Signed192 value, discarding the upper 384 bits.
    /// </summary>
    internal static Signed192 NarrowValue(Signed576 value) => new(value.Word2, value.Word1, value.Word0);

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    internal static bool TryNarrowSigned(Signed576 value, out Signed192 result)
    {
        ulong extension = (value.Word2 & (1UL << 63)) != 0UL ? ulong.MaxValue : 0UL;
        if (value.Word8 != extension || value.Word7 != extension || value.Word6 != extension
            || value.Word5 != extension || value.Word4 != extension || value.Word3 != extension)
        {
            result = default;
            return false;
        }

        result = new Signed192(value.Word2, value.Word1, value.Word0);
        return true;
    }

    internal bool Equals(Signed192 other) =>
        ((High ^ other.High)
         | (Middle ^ other.Middle)
         | (Low ^ other.Low)) == 0UL;
}
