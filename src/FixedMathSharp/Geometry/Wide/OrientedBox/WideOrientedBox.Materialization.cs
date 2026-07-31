//=======================================================================
// WideOrientedBox.Materialization.cs
//=======================================================================
// MIT License, Copyright (c) 2024–present David Oravsky (mrdav30)
// See LICENSE file in the project root for full license information.
//=======================================================================

namespace FixedMathSharp.Geometry;

/// <content>
/// Provides high-precision materialization helpers for converting wide (extended-precision)
/// oriented box coordinates back into standard <see cref="Fixed64"/>/<see cref="Vector3d"/> values.
/// </content>
internal static partial class WideOrientedBox
{
    private static Vector3d GetRationalLocalPoint(
        WideRationalBasis3d basis,
        Signed320 localX,
        Signed320 localY,
        Signed320 localZ)
    {
        Signed576 denominator = ToSigned576(basis.Denominator);
        _ = Fixed64.TryGetSignedRawRatio(
            Signed576.ExtendValue(localX),
            denominator,
            out Fixed64 x);
        _ = Fixed64.TryGetSignedRawRatio(
            Signed576.ExtendValue(localY),
            denominator,
            out Fixed64 y);
        _ = Fixed64.TryGetSignedRawRatio(
            Signed576.ExtendValue(localZ),
            denominator,
            out Fixed64 z);
        return new Vector3d(x, y, z);
    }

    private static bool TryMaterializeCoordinate(
        Fixed64 center,
        Signed192 axisX,
        Signed192 axisY,
        Signed192 axisZ,
        Signed192 denominator,
        Signed576 denominatorWide,
        Vector3d localPoint,
        out Fixed64 coordinate)
    {
        Signed320 numerator = WideArithmetic.AddSigned320(
            WideArithmetic.MultiplySigned192(Signed192.Raw(center), denominator),
            WideArithmetic.AddSigned320(
                WideArithmetic.AddSigned320(
                    WideArithmetic.MultiplySigned192(Signed192.Raw(localPoint.X), axisX),
                    WideArithmetic.MultiplySigned192(Signed192.Raw(localPoint.Y), axisY)),
                WideArithmetic.MultiplySigned192(Signed192.Raw(localPoint.Z), axisZ)));
        return Fixed64.TryGetSignedRawRatio(
            Signed576.ExtendValue(numerator),
            denominatorWide,
            out coordinate);
    }

    private static bool TryMaterializeCoordinate(
        Fixed64 firstOrigin,
        Fixed64 secondOrigin,
        Signed192 axisX,
        Signed192 axisY,
        Signed192 axisZ,
        Signed192 denominator,
        Signed576 denominatorWide,
        Vector3d localPoint,
        out Fixed64 coordinate)
    {
        Signed192 combinedOrigin = WideArithmetic.AddSigned192(
            Signed192.Raw(firstOrigin),
            Signed192.Raw(secondOrigin));
        Signed320 numerator = WideArithmetic.AddSigned320(
            WideArithmetic.MultiplySigned192(combinedOrigin, denominator),
            WideArithmetic.AddSigned320(
                WideArithmetic.AddSigned320(
                    WideArithmetic.MultiplySigned192(Signed192.Raw(localPoint.X), axisX),
                    WideArithmetic.MultiplySigned192(Signed192.Raw(localPoint.Y), axisY)),
                WideArithmetic.MultiplySigned192(Signed192.Raw(localPoint.Z), axisZ)));
        return Fixed64.TryGetSignedRawRatio(
            Signed576.ExtendValue(numerator),
            denominatorWide,
            out coordinate);
    }

    private static bool TryMaterializeRationalOffset(
        WideRationalBasis3d basis,
        Vector3d localOffset,
        out Vector3d worldOffset)
    {
        Signed576 denominator = ToSigned576(basis.Denominator);
        bool representable = TryMaterializeCoordinate(
            Fixed64.Zero,
            basis.Xx,
            basis.Yx,
            basis.Zx,
            basis.Denominator,
            denominator,
            localOffset,
            out Fixed64 x)
            & TryMaterializeCoordinate(
                Fixed64.Zero,
                basis.Xy,
                basis.Yy,
                basis.Zy,
                basis.Denominator,
                denominator,
                localOffset,
                out Fixed64 y)
            & TryMaterializeCoordinate(
                Fixed64.Zero,
                basis.Xz,
                basis.Yz,
                basis.Zz,
                basis.Denominator,
                denominator,
                localOffset,
                out Fixed64 z);
        if (!representable)
        {
            worldOffset = default;
            return false;
        }

        worldOffset = new Vector3d(x, y, z);
        return true;
    }

    private static bool TryMaterializeSupportDifferenceCoordinate(
        Fixed64 center,
        Fixed64 otherOrigin,
        Fixed64 otherOffset,
        Signed192 axisX,
        Signed192 axisY,
        Signed192 axisZ,
        Signed192 basisDenominator,
        Signed576 denominator,
        Vector3d localSupport,
        out Fixed64 coordinate)
    {
        Signed192 relativeOrigin = WideArithmetic.SubtractSigned192(
            WideArithmetic.SubtractSigned192(Signed192.Raw(center), Signed192.Raw(otherOrigin)),
            Signed192.Raw(otherOffset));
        Signed320 numerator = WideArithmetic.AddSigned320(
            WideArithmetic.MultiplySigned192(relativeOrigin, basisDenominator),
            WideArithmetic.AddSigned320(
                WideArithmetic.AddSigned320(
                    WideArithmetic.MultiplySigned192(Signed192.Raw(localSupport.X), axisX),
                    WideArithmetic.MultiplySigned192(Signed192.Raw(localSupport.Y), axisY)),
                WideArithmetic.MultiplySigned192(Signed192.Raw(localSupport.Z), axisZ)));
        return Fixed64.TryGetSignedRawRatio(
            Signed576.ExtendValue(numerator),
            denominator,
            out coordinate);
    }

    private static bool TryMaterializeBoxSupportDifferenceCoordinate(
        Fixed64 center,
        Fixed64 otherCenter,
        Signed192 axisX,
        Signed192 axisY,
        Signed192 axisZ,
        Signed192 basisDenominator,
        Vector3d localSupport,
        Signed192 otherAxisX,
        Signed192 otherAxisY,
        Signed192 otherAxisZ,
        Signed192 otherBasisDenominator,
        Vector3d otherLocalSupport,
        Signed320 denominator,
        Signed576 denominatorWide,
        out Fixed64 coordinate)
    {
        Signed320 supportNumerator = WideArithmetic.GetDotProduct3D(
            Signed192.Raw(localSupport.X),
            Signed192.Raw(localSupport.Y),
            Signed192.Raw(localSupport.Z),
            axisX,
            axisY,
            axisZ);
        Signed320 otherSupportNumerator = WideArithmetic.GetDotProduct3D(
            Signed192.Raw(otherLocalSupport.X),
            Signed192.Raw(otherLocalSupport.Y),
            Signed192.Raw(otherLocalSupport.Z),
            otherAxisX,
            otherAxisY,
            otherAxisZ);
        Signed576 numerator = WideArithmetic.SubtractSigned576(
            WideArithmetic.AddSigned576(
                WideArithmetic.MultiplySigned320(
                    Signed320.ExtendValue(
                        WideArithmetic.SubtractSigned192(
                            Signed192.Raw(center),
                            Signed192.Raw(otherCenter))),
                    denominator),
                WideArithmetic.MultiplySigned320(
                    supportNumerator,
                    Signed320.ExtendValue(
                        otherBasisDenominator))),
            WideArithmetic.MultiplySigned320(
                otherSupportNumerator,
                Signed320.ExtendValue(
                    basisDenominator)));
        return Fixed64.TryGetSignedRawRatio(
            numerator,
            denominatorWide,
            out coordinate);
    }

    private static bool TryGetRelativeOffsetCoordinate(
        Fixed64 firstOrigin,
        Fixed64 firstOffset,
        Fixed64 secondOrigin,
        Signed192 axisX,
        Signed192 axisY,
        Signed192 axisZ,
        Signed192 basisDenominator,
        Signed576 denominator,
        Vector3d secondLocalPoint,
        out Fixed64 coordinate)
    {
        Signed192 relativeOrigin = WideArithmetic.SubtractSigned192(
            WideArithmetic.AddSigned192(
                Signed192.Raw(firstOrigin),
                Signed192.Raw(firstOffset)),
            Signed192.Raw(secondOrigin));
        Signed320 rotatedLocalPoint = WideArithmetic.AddSigned320(
            WideArithmetic.AddSigned320(
                WideArithmetic.MultiplySigned192(
                    Signed192.Raw(secondLocalPoint.X),
                    axisX),
                WideArithmetic.MultiplySigned192(
                    Signed192.Raw(secondLocalPoint.Y),
                    axisY)),
            WideArithmetic.MultiplySigned192(
                Signed192.Raw(secondLocalPoint.Z),
                axisZ));
        Signed320 numerator = WideArithmetic.SubtractSigned320(
            WideArithmetic.MultiplySigned192(
                relativeOrigin,
                basisDenominator),
            rotatedLocalPoint);
        return Fixed64.TryGetSignedRawRatio(
            Signed576.ExtendValue(numerator),
            denominator,
            out coordinate);
    }
}
