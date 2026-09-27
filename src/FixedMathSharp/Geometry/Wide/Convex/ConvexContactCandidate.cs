//=======================================================================
// MIT License, Copyright (c) 2024-present David Oravsky (mrdav30)
// See LICENSE file in the project root for full license information.
//=======================================================================
using System;

namespace FixedMathSharp.Geometry;

/// <summary>
/// A nonzero normal a+b sqrt(K) and signed raw gap
/// GapSign sqrt((A+B sqrt(C))/D), with positive D and nonnegative radicand.
/// </summary>
/// <remarks>
/// Each little-endian magnitude occupies forty words. Slots 0..2 are a,
/// 3..5 are b, 6 is K, and 7..10 are A, B, C, D. Signs 0..5, 7, and 8
/// are -1, 0, or 1, with zero sign exactly for zero magnitude; unused
/// signs do not affect the value. GapSign is zero exactly when the gap is
/// zero. Builders prove these invariants before admission; this view neither
/// copies nor mutates their borrowed storage.
/// </remarks>
internal readonly ref struct ConvexContactCandidate
{
    internal const int Words = 40;
    internal const int Slots = 11;

    internal ConvexContactCandidate(ReadOnlySpan<ulong> values, ReadOnlySpan<int> signs, int gapSign)
    {
        Values = values;
        Signs = signs;
        GapSign = gapSign;
    }

    internal ReadOnlySpan<ulong> Values { get; }
    internal ReadOnlySpan<int> Signs { get; }
    internal int GapSign { get; }
    internal ReadOnlySpan<ulong> NormalRational(int component) => Slot(component);
    internal ReadOnlySpan<ulong> NormalRadical(int component) => Slot(component + 3);
    internal ReadOnlySpan<ulong> NormalRadicand => Slot(6);
    internal ReadOnlySpan<ulong> GapRational => Slot(7);
    internal int GapRationalSign => Signs[7];
    internal ReadOnlySpan<ulong> GapRadical => Slot(8);
    internal int GapRadicalSign => Signs[8];
    internal ReadOnlySpan<ulong> GapRadicand => Slot(9);
    internal ReadOnlySpan<ulong> GapDenominator => Slot(10);
    private ReadOnlySpan<ulong> Slot(int slot) => Values.Slice(slot * Words, Words);
}
