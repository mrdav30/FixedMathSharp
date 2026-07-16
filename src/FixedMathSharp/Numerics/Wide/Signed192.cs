//=======================================================================
// Signed192.cs
//=======================================================================
// MIT License, Copyright (c) 2024–present David Oravsky (mrdav30)
// See LICENSE file in the project root for full license information.
//=======================================================================

namespace FixedMathSharp;

/// <summary>
/// A signed two's-complement value wide enough for sums of full-domain
/// Q32.32 endpoint-difference products.
/// </summary>
internal readonly struct Signed192
{
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
}
