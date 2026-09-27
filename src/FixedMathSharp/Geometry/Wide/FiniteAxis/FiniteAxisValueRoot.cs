//=======================================================================
// MIT License, Copyright (c) 2024-present David Oravsky (mrdav30)
// See LICENSE file in the project root for full license information.
//=======================================================================
using System;

namespace FixedMathSharp.Geometry;

/// <summary>
/// One ordinal-selected root in (0,1] of a degree-at-most-eight integer value
/// polynomial. Borrowed coefficients and cell storage outlive this view.
/// </summary>
internal ref struct FiniteAxisValueRoot
{
    internal ReadOnlySpan<ulong> Coefficients;
    internal ReadOnlySpan<sbyte> Signs;
    internal Span<ulong> LowerNumerator;
    internal int DenominatorShift;
    internal int Ordinal;
    internal bool IsRational;
}
