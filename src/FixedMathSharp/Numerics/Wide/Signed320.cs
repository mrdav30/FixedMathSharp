//=======================================================================
// Signed320.cs
//=======================================================================
// MIT License, Copyright (c) 2024–present David Oravsky (mrdav30)
// See LICENSE file in the project root for full license information.
//=======================================================================

namespace FixedMathSharp;

/// <summary>
/// A signed five-word value used by exact geometry for products and
/// differences of three-component dot products.
/// </summary>
internal readonly struct Signed320
{
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
}
