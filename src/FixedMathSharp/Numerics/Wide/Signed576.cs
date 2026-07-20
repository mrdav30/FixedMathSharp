//=======================================================================
// Signed576.cs
//=======================================================================
// MIT License, Copyright (c) 2024–present David Oravsky (mrdav30)
// See LICENSE file in the project root for full license information.
//=======================================================================

namespace FixedMathSharp;

/// <summary>
/// A signed nine-word value used by exact finite-axis geometry evaluations.
/// </summary>
internal readonly struct Signed576
{
    internal readonly ulong Word8;
    internal readonly ulong Word7;
    internal readonly ulong Word6;
    internal readonly ulong Word5;
    internal readonly ulong Word4;
    internal readonly ulong Word3;
    internal readonly ulong Word2;
    internal readonly ulong Word1;
    internal readonly ulong Word0;

    internal Signed576(
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
        (Word8 | Word7 | Word6 | Word5 | Word4 | Word3 | Word2 | Word1 | Word0) == 0UL;

    internal int Sign => IsZero ? 0 : (Word8 & (1UL << 63)) != 0UL ? -1 : 1;
}
