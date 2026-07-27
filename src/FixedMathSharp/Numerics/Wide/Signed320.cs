//=======================================================================
// Signed320.cs
//=======================================================================
// MIT License, Copyright (c) 2024–present David Oravsky (mrdav30)
// See LICENSE file in the project root for full license information.
//=======================================================================

using System.Runtime.CompilerServices;

namespace FixedMathSharp;

/// <summary>
/// A signed five-word value used by exact geometry for products and
/// differences of three-component dot products.
/// </summary>
internal readonly struct Signed320
{
    public static readonly Signed320 One = ExtendValue(Signed192.One);

    internal readonly ulong Word4;
    internal readonly ulong Word3;
    internal readonly ulong Word2;
    internal readonly ulong Word1;
    internal readonly ulong Word0;

    internal Signed320(ulong word4, ulong word3, ulong word2, ulong word1, ulong word0)
    {
        Word4 = word4;
        Word3 = word3;
        Word2 = word2;
        Word1 = word1;
        Word0 = word0;
    }

    internal bool IsZero => (Word4 | Word3 | Word2 | Word1 | Word0) == 0UL;

    internal int Sign => IsZero ? 0 : (Word4 & (1UL << 63)) != 0UL ? -1 : 1;

    /// <summary>
    /// Narrow a Signed576 value to a Signed320 value, discarding the upper 256 bits.
    /// </summary>
    internal static Signed320 NarrowValue(Signed576 value) =>
        new(value.Word4,
            value.Word3,
            value.Word2,
            value.Word1,
            value.Word0);

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    internal static bool TryNarrowSigned(Signed576 value, out Signed320 result)
    {
        ulong extension = (value.Word4 & (1UL << 63)) != 0UL ? ulong.MaxValue : 0UL;
        if (value.Word8 != extension || value.Word7 != extension
            || value.Word6 != extension || value.Word5 != extension)
        {
            result = default;
            return false;
        }

        result = new Signed320(value.Word4, value.Word3, value.Word2, value.Word1, value.Word0);
        return true;
    }

    /// <summary>
    /// Sign-extends an exact three-word value to five words.
    /// </summary>
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    internal static Signed320 ExtendValue(Signed192 value)
    {
        ulong extension = value.Sign < 0 ? ulong.MaxValue : 0UL;
        return new Signed320(extension, extension, value.High, value.Middle, value.Low);
    }

    internal bool Equals(Signed320 other) =>
        ((Word4 ^ other.Word4)
         | (Word3 ^ other.Word3)
         | (Word2 ^ other.Word2)
         | (Word1 ^ other.Word1)
         | (Word0 ^ other.Word0)) == 0UL;
}
