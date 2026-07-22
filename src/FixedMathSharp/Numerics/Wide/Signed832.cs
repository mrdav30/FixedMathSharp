//=======================================================================
// Signed832.cs
//=======================================================================
// MIT License, Copyright (c) 2024–present David Oravsky (mrdav30)
// See LICENSE file in the project root for full license information.
//=======================================================================

namespace FixedMathSharp;

/// <summary>
/// A signed thirteen-word value used by full-domain conic discriminants.
/// </summary>
internal readonly struct Signed832
{
    internal readonly ulong Word12;
    internal readonly ulong Word11;
    internal readonly ulong Word10;
    internal readonly ulong Word9;
    internal readonly ulong Word8;
    internal readonly ulong Word7;
    internal readonly ulong Word6;
    internal readonly ulong Word5;
    internal readonly ulong Word4;
    internal readonly ulong Word3;
    internal readonly ulong Word2;
    internal readonly ulong Word1;
    internal readonly ulong Word0;

    internal Signed832(
        ulong word12,
        ulong word11,
        ulong word10,
        ulong word9,
        ulong word8,
        ulong word7,
        ulong word6,
        ulong word5,
        ulong word4,
        ulong word3,
        ulong word2,
        ulong word1,
        ulong word0)
    {
        Word12 = word12;
        Word11 = word11;
        Word10 = word10;
        Word9 = word9;
        Word8 = word8;
        Word7 = word7;
        Word6 = word6;
        Word5 = word5;
        Word4 = word4;
        Word3 = word3;
        Word2 = word2;
        Word1 = word1;
        Word0 = word0;
    }

    internal bool IsZero =>
        (Word12 | Word11 | Word10 | Word9 | Word8 | Word7 | Word6 |
         Word5 | Word4 | Word3 | Word2 | Word1 | Word0) == 0UL;

    internal int Sign => IsZero ? 0 : (Word12 & (1UL << 63)) != 0UL ? -1 : 1;
}
