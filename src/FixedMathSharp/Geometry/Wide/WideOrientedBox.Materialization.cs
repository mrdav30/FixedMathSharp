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
        RationalBasis basis,
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

    private static bool TryMaterializeScaledCoordinate(
        Fixed64 center,
        Signed192 axisX,
        Signed192 axisY,
        Signed192 axisZ,
        Signed320 scaledDenominator,
        Signed576 denominator,
        Vector3d localPoint,
        Vector3d scale,
        Vector3d localDisplacement,
        out Fixed64 coordinate)
    {
        Signed576 scaledLocal = WideArithmetic.AddSigned576(
            WideArithmetic.AddSigned576(
                WideArithmetic.MultiplySigned320(
                    WideArithmetic.MultiplySigned192(
                        Signed192.Raw(localPoint.X),
                        Signed192.Raw(scale.X)),
                    Signed320.ExtendValue(axisX)),
                WideArithmetic.MultiplySigned320(
                    WideArithmetic.MultiplySigned192(
                        Signed192.Raw(localPoint.Y),
                        Signed192.Raw(scale.Y)),
                    Signed320.ExtendValue(axisY))),
            WideArithmetic.MultiplySigned320(
                WideArithmetic.MultiplySigned192(
                    Signed192.Raw(localPoint.Z),
                    Signed192.Raw(scale.Z)),
                Signed320.ExtendValue(axisZ)));
        Signed320 displacement = WideArithmetic.AddSigned320(
            WideArithmetic.AddSigned320(
                WideArithmetic.MultiplySigned192(
                    Signed192.Raw(localDisplacement.X),
                    axisX),
                WideArithmetic.MultiplySigned192(
                    Signed192.Raw(localDisplacement.Y),
                    axisY)),
            WideArithmetic.MultiplySigned192(
                Signed192.Raw(localDisplacement.Z),
                axisZ));
        Signed576 numerator = WideArithmetic.AddSigned576(
            WideArithmetic.AddSigned576(
                WideArithmetic.MultiplySigned320(
                    Signed320.ExtendValue(Signed192.Raw(center)),
                    scaledDenominator),
                scaledLocal),
            WideArithmetic.MultiplySigned320(
                displacement,
                Signed320.ExtendValue(Signed192.One)));
        return Fixed64.TryGetSignedRawRatio(
            numerator,
            denominator,
            out coordinate);
    }

    internal static bool TryComposeScaledLocalPoints(
        Vector3d outerLocalPoint,
        Vector3d outerScale,
        Vector3d innerFrameOffset,
        Vector3d innerFrameScale,
        Vector3d innerLocalDisplacement,
        FixedQuaternion innerRotation,
        out Vector3d result)
    {
        RationalBasis basis = new(innerRotation);
        Signed320 scaledDenominator = WideArithmetic.MultiplySigned192(
            basis.Denominator,
            Signed192.One);
        Signed576 denominator =
            Signed576.ExtendValue(scaledDenominator);
        bool representable = TryComposeScaledLocalCoordinate(
                outerLocalPoint.X,
                outerScale.X,
                basis.Xx,
                basis.Yx,
                basis.Zx,
                basis.Denominator,
                denominator,
                innerFrameOffset.X,
                innerFrameScale.X,
                innerLocalDisplacement,
                out Fixed64 x)
            & TryComposeScaledLocalCoordinate(
                outerLocalPoint.Y,
                outerScale.Y,
                basis.Xy,
                basis.Yy,
                basis.Zy,
                basis.Denominator,
                denominator,
                innerFrameOffset.Y,
                innerFrameScale.Y,
                innerLocalDisplacement,
                out Fixed64 y)
            & TryComposeScaledLocalCoordinate(
                outerLocalPoint.Z,
                outerScale.Z,
                basis.Xz,
                basis.Yz,
                basis.Zz,
                basis.Denominator,
                denominator,
                innerFrameOffset.Z,
                innerFrameScale.Z,
                innerLocalDisplacement,
                out Fixed64 z);
        if (!representable)
        {
            result = default;
            return false;
        }

        result = new Vector3d(x, y, z);
        return true;
    }

    internal static FixedMassPoint CreateMassPoint(
        Vector3d outerLocalPoint,
        Vector3d outerScale,
        Vector3d innerFrameOffset,
        Vector3d innerFrameScale,
        Vector3d innerLocalDisplacement,
        FixedQuaternion innerRotation)
    {
        RationalBasis basis = new(innerRotation);
        Signed576 denominator = Signed576.ExtendValue(
            WideArithmetic.MultiplySigned192(
                basis.Denominator,
                Signed192.One));
        Signed320 x = WideArithmetic.GetSignedRatioWith64FractionBits(
            GetComposedScaledLocalCoordinateNumerator(
                outerLocalPoint.X,
                outerScale.X,
                basis.Xx,
                basis.Yx,
                basis.Zx,
                basis.Denominator,
                innerFrameOffset.X,
                innerFrameScale.X,
                innerLocalDisplacement),
            denominator);
        Signed320 y = WideArithmetic.GetSignedRatioWith64FractionBits(
            GetComposedScaledLocalCoordinateNumerator(
                outerLocalPoint.Y,
                outerScale.Y,
                basis.Xy,
                basis.Yy,
                basis.Zy,
                basis.Denominator,
                innerFrameOffset.Y,
                innerFrameScale.Y,
                innerLocalDisplacement),
            denominator);
        Signed320 z = WideArithmetic.GetSignedRatioWith64FractionBits(
            GetComposedScaledLocalCoordinateNumerator(
                outerLocalPoint.Z,
                outerScale.Z,
                basis.Xz,
                basis.Yz,
                basis.Zz,
                basis.Denominator,
                innerFrameOffset.Z,
                innerFrameScale.Z,
                innerLocalDisplacement),
            denominator);
        return new FixedMassPoint(x, y, z);
    }

    private static bool TryComposeScaledLocalCoordinate(
        Fixed64 outerLocalPoint,
        Fixed64 outerScale,
        Signed192 axisX,
        Signed192 axisY,
        Signed192 axisZ,
        Signed192 rotationDenominator,
        Signed576 denominator,
        Fixed64 innerFrameOffset,
        Fixed64 innerFrameScale,
        Vector3d innerLocalDisplacement,
        out Fixed64 coordinate)
    {
        return Fixed64.TryGetSignedRawRatio(
            GetComposedScaledLocalCoordinateNumerator(
                outerLocalPoint,
                outerScale,
                axisX,
                axisY,
                axisZ,
                rotationDenominator,
                innerFrameOffset,
                innerFrameScale,
                innerLocalDisplacement),
            denominator,
            out coordinate);
    }

    private static Signed576 GetComposedScaledLocalCoordinateNumerator(
        Fixed64 outerLocalPoint,
        Fixed64 outerScale,
        Signed192 axisX,
        Signed192 axisY,
        Signed192 axisZ,
        Signed192 rotationDenominator,
        Fixed64 innerFrameOffset,
        Fixed64 innerFrameScale,
        Vector3d innerLocalDisplacement)
    {
        Signed576 outerScaled = WideArithmetic.MultiplySigned320(
            WideArithmetic.MultiplySigned192(
                Signed192.Raw(outerLocalPoint),
                Signed192.Raw(outerScale)),
            Signed320.ExtendValue(rotationDenominator));
        Signed576 innerScaled = WideArithmetic.MultiplySigned320(
            WideArithmetic.MultiplySigned192(
                Signed192.Raw(innerFrameOffset),
                Signed192.Raw(innerFrameScale)),
            Signed320.ExtendValue(rotationDenominator));
        Signed320 displacement = WideArithmetic.AddSigned320(
            WideArithmetic.AddSigned320(
                WideArithmetic.MultiplySigned192(
                    Signed192.Raw(innerLocalDisplacement.X),
                    axisX),
                WideArithmetic.MultiplySigned192(
                    Signed192.Raw(innerLocalDisplacement.Y),
                    axisY)),
            WideArithmetic.MultiplySigned192(
                Signed192.Raw(innerLocalDisplacement.Z),
                axisZ));
        return WideArithmetic.AddSigned576(
            WideArithmetic.AddSigned576(
                outerScaled,
                innerScaled),
            WideArithmetic.MultiplySigned320(
                displacement,
                Signed320.ExtendValue(Signed192.One)));
    }

    private static bool TryMaterializeRationalOffset(
        RationalBasis basis,
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
        Signed320 supportNumerator = GetProjection(
            Signed192.Raw(localSupport.X),
            Signed192.Raw(localSupport.Y),
            Signed192.Raw(localSupport.Z),
            axisX,
            axisY,
            axisZ);
        Signed320 otherSupportNumerator = GetProjection(
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
