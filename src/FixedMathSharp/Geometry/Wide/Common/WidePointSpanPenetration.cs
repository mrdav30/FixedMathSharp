//=======================================================================
// WidePointSpanPenetration.cs
//=======================================================================
// MIT License, Copyright (c) 2024-present David Oravsky (mrdav30)
// See LICENSE file in the project root for full license information.
//=======================================================================

namespace FixedMathSharp.Geometry;

/// <summary>
/// Retains one exact point-span SAT penetration for ranking and materialization.
/// </summary>
internal readonly struct WidePointSpanPenetration
{
    internal readonly WideAxis3 Axis;
    internal readonly bool Negate;
    internal readonly Signed576 ExactOverlap;
    internal readonly Signed576 ExactSquaredAxisLength;
    internal readonly Signed320 ExactCommonDenominator;

    internal WidePointSpanPenetration(
        WideAxis3 axis,
        bool negate,
        Signed576 exactOverlap,
        Signed576 exactSquaredAxisLength,
        Signed320 exactCommonDenominator)
    {
        Axis = axis;
        Negate = negate;
        ExactOverlap = exactOverlap;
        ExactSquaredAxisLength = exactSquaredAxisLength;
        ExactCommonDenominator = exactCommonDenominator;
        HasValue = true;
    }

    internal bool HasValue { get; }

    internal bool ShouldReplace(
        Signed576 overlap,
        Signed576 squaredAxisLength,
        Signed320 commonDenominator)
    {
        if (!HasValue)
            return true;
        return WideArithmetic.CompareNonNegativeNormalizedDepths(
            overlap,
            squaredAxisLength,
            commonDenominator,
            ExactOverlap,
            ExactSquaredAxisLength,
            ExactCommonDenominator) < 0;
    }

    internal Fixed64 GetRoundedDepth(out bool isClamped) =>
        WideArithmetic.GetRoundedNonNegativeNormalizedDepth(
            ExactOverlap,
            ExactSquaredAxisLength,
            ExactCommonDenominator,
            out isClamped);
}
