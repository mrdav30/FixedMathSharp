//=======================================================================
// WideOrientedBox.PointAnchorDistance.cs
//=======================================================================
// MIT License, Copyright (c) 2024–present David Oravsky (mrdav30)
// See LICENSE file in the project root for full license information.
//=======================================================================

namespace FixedMathSharp.Geometry;

/// <content>
/// Wide-oriented box point anchor distance operations.
/// </content>
internal static partial class WideOrientedBox
{
    internal static int CompareSquaredDistances(
        in FixedPointAnchor reference,
        in FixedPointAnchor first,
        in FixedPointAnchor second)
    {
        if (!reference.ExactLocalTerm.IsZero
            || !first.ExactLocalTerm.IsZero
            || !second.ExactLocalTerm.IsZero
            || reference.LocalTranslation != Vector3d.Zero
            || first.LocalTranslation != Vector3d.Zero
            || second.LocalTranslation != Vector3d.Zero)
        {
            return CompareExactSquaredDistances(
                reference,
                first,
                second);
        }

        GetSquaredOffsetRatio(
            first,
            reference,
            out Signed832 firstNumerator,
            out Signed576 firstDenominator);
        GetSquaredOffsetRatio(
            second,
            reference,
            out Signed832 secondNumerator,
            out Signed576 secondDenominator);
        return WideArithmetic.CompareNonNegativeSquaredRatios(
            firstNumerator,
            firstDenominator,
            secondNumerator,
            secondDenominator);
    }

    private static int CompareExactSquaredDistances(
        in FixedPointAnchor reference,
        in FixedPointAnchor first,
        in FixedPointAnchor second)
    {
        GetExactSquaredOffsetRatio(
            first,
            reference,
            out Signed832 firstNumerator,
            out Signed576 firstDenominator);
        GetExactSquaredOffsetRatio(
            second,
            reference,
            out Signed832 secondNumerator,
            out Signed576 secondDenominator);
        return WideArithmetic.CompareNonNegativeSquaredRatios(
            firstNumerator,
            firstDenominator,
            secondNumerator,
            secondDenominator);
    }

    private static void GetExactSquaredOffsetRatio(
        in FixedPointAnchor first,
        in FixedPointAnchor second,
        out Signed832 squaredNumerator,
        out Signed576 denominator)
    {
        GetExactRelativeOffsetRatio(
            first.Origin,
            first.Rotation,
            first.LocalPoint,
            first.LocalDisplacement,
            first.LocalTranslation,
            first.ExactLocalTerm,
            second.Origin,
            second.Rotation,
            second.LocalPoint,
            second.LocalDisplacement,
            second.LocalTranslation,
            second.ExactLocalTerm,
            out Signed576 x,
            out Signed576 y,
            out Signed576 z,
            out denominator);
        squaredNumerator = WideArithmetic.AddSigned832(
            WideArithmetic.AddSigned832(
                WideArithmetic.MultiplySigned576ToSigned832(x, x),
                WideArithmetic.MultiplySigned576ToSigned832(y, y)),
            WideArithmetic.MultiplySigned576ToSigned832(z, z));
    }

    private static void GetSquaredOffsetRatio(
        in FixedPointAnchor first,
        in FixedPointAnchor second,
        out Signed832 squaredNumerator,
        out Signed576 denominator)
    {
        RationalBasis firstBasis = new(first.Rotation);
        RationalBasis secondBasis = new(second.Rotation);
        Signed320 coordinateDenominator =
            WideArithmetic.MultiplySigned192(
                firstBasis.Denominator,
                secondBasis.Denominator);
        Signed576 x = GetRelativeOffsetNumerator(
            first.Origin.X,
            firstBasis.Xx,
            firstBasis.Yx,
            firstBasis.Zx,
            firstBasis.Denominator,
            first.LocalPoint,
            first.LocalDisplacement,
            second.Origin.X,
            secondBasis.Xx,
            secondBasis.Yx,
            secondBasis.Zx,
            secondBasis.Denominator,
            second.LocalPoint,
            second.LocalDisplacement,
            coordinateDenominator);
        Signed576 y = GetRelativeOffsetNumerator(
            first.Origin.Y,
            firstBasis.Xy,
            firstBasis.Yy,
            firstBasis.Zy,
            firstBasis.Denominator,
            first.LocalPoint,
            first.LocalDisplacement,
            second.Origin.Y,
            secondBasis.Xy,
            secondBasis.Yy,
            secondBasis.Zy,
            secondBasis.Denominator,
            second.LocalPoint,
            second.LocalDisplacement,
            coordinateDenominator);
        Signed576 z = GetRelativeOffsetNumerator(
            first.Origin.Z,
            firstBasis.Xz,
            firstBasis.Yz,
            firstBasis.Zz,
            firstBasis.Denominator,
            first.LocalPoint,
            first.LocalDisplacement,
            second.Origin.Z,
            secondBasis.Xz,
            secondBasis.Yz,
            secondBasis.Zz,
            secondBasis.Denominator,
            second.LocalPoint,
            second.LocalDisplacement,
            coordinateDenominator);
        squaredNumerator = WideArithmetic.AddSigned832(
            WideArithmetic.AddSigned832(
                WideArithmetic.MultiplySigned576ToSigned832(x, x),
                WideArithmetic.MultiplySigned576ToSigned832(y, y)),
            WideArithmetic.MultiplySigned576ToSigned832(z, z));
        denominator = Signed576.ExtendValue(
            coordinateDenominator);
    }
}
