//=======================================================================
// MIT License, Copyright (c) 2024-present David Oravsky (mrdav30)
// See LICENSE file in the project root for full license information.
//=======================================================================
using System;

namespace FixedMathSharp.Geometry;

/// <summary>
/// The largest positive real root of an integer polynomial of degree at most
/// four. Coefficient/sign spans and the mutable dyadic-cell storage are borrowed
/// from the caller and must remain valid for this view's lifetime.
/// </summary>
/// <remarks>
/// A nonrational root is the sole root in the open cell
/// (LowerNumerator, LowerNumerator+1)/2^DenominatorShift; both endpoints are
/// nonroots and LowerNumerator is strictly positive. A rational root equals
/// LowerNumerator/2^DenominatorShift exactly.
/// Only WideFiniteAxisIntersection constructs or refines this representation.
/// </remarks>
internal ref struct FiniteAxisPolynomialRoot
{
    internal ReadOnlySpan<ulong> Coefficients;
    internal ReadOnlySpan<sbyte> Signs;
    internal Span<ulong> LowerNumerator;
    internal int DenominatorShift;
    internal bool IsRational;
    internal int CoefficientBits;
    internal sbyte CrossingSign;
}
