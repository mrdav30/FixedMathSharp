//=======================================================================
// WideScaledHullPenetration.cs
//=======================================================================
// MIT License, Copyright (c) 2024-present David Oravsky (mrdav30)
// See LICENSE file in the project root for full license information.
//=======================================================================

using System;

namespace FixedMathSharp.Geometry;

/// <summary>Exact signed exit retained before materializing a scaled hull's final contact.</summary>
internal readonly struct WideScaledHullPenetration
{
    internal readonly WideDirection3d Axis;
    internal readonly Signed832 Overlap, SquaredAxisLength;
    internal readonly Signed576 CommonDenominator;
    internal readonly bool Negate;
    internal bool HasValue { get; }
    internal WideScaledHullPenetration(WideDirection3d axis, bool negate, Signed832 overlap,
        Signed832 squaredAxisLength, Signed576 commonDenominator)
    {
        Axis = axis; Negate = negate; Overlap = overlap; SquaredAxisLength = squaredAxisLength;
        CommonDenominator = commonDenominator; HasValue = true;
    }
    internal WideScaledHullPenetration(in WidePointSpanPenetration value)
    {
        Axis = new(value.Axis); Negate = value.Negate;
        Overlap = Signed832.ExtendValue(value.ExactOverlap);
        SquaredAxisLength = Signed832.ExtendValue(value.ExactSquaredAxisLength);
        CommonDenominator = Signed576.ExtendValue(value.ExactCommonDenominator);
        HasValue = value.HasValue;
    }
    internal bool ShouldReplace(Signed832 overlap, Signed832 squaredAxisLength, Signed576 denominator) =>
        !HasValue || WideArithmetic.CompareNonNegativeNormalizedDepths(overlap, squaredAxisLength, denominator,
            Overlap, SquaredAxisLength, CommonDenominator) < 0;
    internal Vector3d GetNormal() => Axis.GetNormal(Negate);
    internal Fixed64 GetRoundedDepth(out bool isClamped) => WideArithmetic.GetRoundedNonNegativeNormalizedDepth(
        Overlap, SquaredAxisLength, CommonDenominator, out isClamped);
}
