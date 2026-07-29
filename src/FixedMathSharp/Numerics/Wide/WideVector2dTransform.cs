//=======================================================================
// WideVector2dTransform.cs
//=======================================================================
// MIT License, Copyright (c) 2024–present David Oravsky (mrdav30)
// See LICENSE file in the project root for full license information.
//=======================================================================

using FixedMathSharp.Geometry;

namespace FixedMathSharp;

/// <summary>
/// Provides methods for transforming two-dimensional points and 
/// vectors using wide arithmetic to avoid overflow and maintain precision.
/// </summary>
internal static partial class WideVector2dTransform
{
    internal static bool TryTransformScaledPoint(
        Vector2d origin,
        Vector2d localPoint,
        Vector2d scale,
        Vector2d localDisplacement,
        Fixed64 angleInRadians,
        out Vector2d result)
    {
        Fixed64 cosine = FixedMath.Cos(angleInRadians);
        Fixed64 sine = FixedMath.Sin(angleInRadians);
        Signed320 denominatorWide = Product(Fixed64.One, Fixed64.One);
        Signed576 denominator =
            Signed576.ExtendValue(denominatorWide);
        GetScaledRotationNumerators(
            localPoint,
            scale,
            localDisplacement,
            cosine,
            sine,
            out Signed576 rotatedX,
            out Signed576 rotatedY);
        bool representable = Fixed64.TryGetSignedRawRatio(
                WideArithmetic.AddSigned576(
                    WideArithmetic.MultiplySigned320(
                        Signed320.ExtendValue(Signed192.Raw(origin.X)),
                        denominatorWide),
                    rotatedX),
                denominator,
                out Fixed64 x)
            & Fixed64.TryGetSignedRawRatio(
                WideArithmetic.AddSigned576(
                    WideArithmetic.MultiplySigned320(
                        Signed320.ExtendValue(Signed192.Raw(origin.Y)),
                        denominatorWide),
                    rotatedY),
                denominator,
                out Fixed64 y);
        if (!representable)
        {
            result = default;
            return false;
        }

        result = new Vector2d(x, y);
        return true;
    }

    internal static bool TryComposeScaledLocalPoints(
        Vector2d outerLocalPoint,
        Vector2d outerScale,
        Vector2d innerFrameOffset,
        Vector2d innerFrameScale,
        Vector2d innerLocalDisplacement,
        Fixed64 innerAngleInRadians,
        out Vector2d result)
    {
        Fixed64 cosine = FixedMath.Cos(innerAngleInRadians);
        Fixed64 sine = FixedMath.Sin(innerAngleInRadians);
        Signed320 one =
            Signed320.ExtendValue(Signed192.One);
        Signed576 denominator = Signed576.ExtendValue(
            Product(Fixed64.One, Fixed64.One));
        GetScaledRotationNumerators(
            Vector2d.Zero,
            Vector2d.One,
            innerLocalDisplacement,
            cosine,
            sine,
            out Signed576 rotatedX,
            out Signed576 rotatedY);
        Signed576 outerX = WideArithmetic.MultiplySigned320(
            Product(outerLocalPoint.X, outerScale.X),
            one);
        Signed576 outerY = WideArithmetic.MultiplySigned320(
            Product(outerLocalPoint.Y, outerScale.Y),
            one);
        Signed576 innerX = WideArithmetic.MultiplySigned320(
            Product(innerFrameOffset.X, innerFrameScale.X),
            one);
        Signed576 innerY = WideArithmetic.MultiplySigned320(
            Product(innerFrameOffset.Y, innerFrameScale.Y),
            one);
        bool representable = Fixed64.TryGetSignedRawRatio(
                WideArithmetic.AddSigned576(
                    WideArithmetic.AddSigned576(outerX, innerX),
                    rotatedX),
                denominator,
                out Fixed64 x)
            & Fixed64.TryGetSignedRawRatio(
                WideArithmetic.AddSigned576(
                    WideArithmetic.AddSigned576(outerY, innerY),
                    rotatedY),
                denominator,
                out Fixed64 y);
        if (!representable)
        {
            result = default;
            return false;
        }

        result = new Vector2d(x, y);
        return true;
    }

    internal static bool TryTransformPoint(
        Vector2d origin,
        Vector2d localPoint,
        Fixed64 angleInRadians,
        out Vector2d result) =>
        TryTransformCompositePoint(
            origin,
            localPoint,
            Vector2d.Zero,
            angleInRadians,
            out result);

    internal static bool TryTransformCompositePoint(
        Vector2d origin,
        Vector2d localPoint,
        Vector2d localDisplacement,
        Fixed64 angleInRadians,
        out Vector2d result)
    {
        Fixed64 cosine = FixedMath.Cos(angleInRadians);
        Fixed64 sine = FixedMath.Sin(angleInRadians);
        Signed576 denominator = Signed576.One;
        GetRotationNumerators(
            localPoint,
            localDisplacement,
            cosine,
            sine,
            out Signed320 rotatedX,
            out Signed320 rotatedY);
        bool representable = TryRoundCoordinate(
                WideArithmetic.AddSigned320(
                    Scale(origin.X),
                    rotatedX),
                denominator,
                out Fixed64 x)
            & TryRoundCoordinate(
                WideArithmetic.AddSigned320(
                    Scale(origin.Y),
                    rotatedY),
                denominator,
                out Fixed64 y);
        if (!representable)
        {
            result = default;
            return false;
        }

        result = new Vector2d(x, y);
        return true;
    }

    internal static bool TryTransformPoint(
        Vector2d firstOrigin,
        Vector2d secondOrigin,
        Vector2d localPoint,
        Fixed64 angleInRadians,
        out Vector2d result)
    {
        Fixed64 cosine = FixedMath.Cos(angleInRadians);
        Fixed64 sine = FixedMath.Sin(angleInRadians);
        Signed576 denominator = Signed576.One;
        GetRotationNumerators(
            localPoint,
            cosine,
            sine,
            out Signed320 rotatedX,
            out Signed320 rotatedY);
        bool representable = TryRoundCoordinate(
                GetTransformNumerator(
                    firstOrigin.X,
                    secondOrigin.X,
                    rotatedX),
                denominator,
                out Fixed64 x)
            & TryRoundCoordinate(
                GetTransformNumerator(
                    firstOrigin.Y,
                    secondOrigin.Y,
                    rotatedY),
                denominator,
                out Fixed64 y);
        if (!representable)
        {
            result = default;
            return false;
        }

        result = new Vector2d(x, y);
        return true;
    }

    internal static bool TryGetRelativeOffset(
        Vector2d firstOrigin,
        Vector2d firstOffset,
        Vector2d secondOrigin,
        Vector2d secondLocalPoint,
        Fixed64 angleInRadians,
        out Vector2d result)
    {
        Fixed64 cosine = FixedMath.Cos(angleInRadians);
        Fixed64 sine = FixedMath.Sin(angleInRadians);
        Signed576 denominator = Signed576.One;
        GetRotationNumerators(
            secondLocalPoint,
            cosine,
            sine,
            out Signed320 rotatedX,
            out Signed320 rotatedY);
        bool representable = TryRoundCoordinate(
                GetRelativeNumerator(
                    firstOrigin.X,
                    firstOffset.X,
                    secondOrigin.X,
                    rotatedX),
                denominator,
                out Fixed64 x)
            & TryRoundCoordinate(
                GetRelativeNumerator(
                    firstOrigin.Y,
                    firstOffset.Y,
                    secondOrigin.Y,
                    rotatedY),
                denominator,
                out Fixed64 y);
        if (!representable)
        {
            result = default;
            return false;
        }

        result = new Vector2d(x, y);
        return true;
    }

    internal static bool TryGetLocalPointIn(
        Vector2d pointOrigin,
        Vector2d pointLocalPoint,
        Vector2d pointLocalDisplacement,
        Fixed64 pointAngleInRadians,
        Vector2d frameOrigin,
        Fixed64 frameAngleInRadians,
        out Vector2d localPoint)
    {
        Fixed64 pointCosine = FixedMath.Cos(pointAngleInRadians);
        Fixed64 pointSine = FixedMath.Sin(pointAngleInRadians);
        Fixed64 frameCosine = FixedMath.Cos(frameAngleInRadians);
        Fixed64 frameSine = FixedMath.Sin(frameAngleInRadians);
        GetRotationNumerators(
            pointLocalPoint,
            pointLocalDisplacement,
            pointCosine,
            pointSine,
            out Signed320 pointX,
            out Signed320 pointY);
        Signed320 deltaX = WideArithmetic.AddSigned320(
            WideArithmetic.SubtractSigned320(
                Scale(pointOrigin.X),
                Scale(frameOrigin.X)),
            pointX);
        Signed320 deltaY = WideArithmetic.AddSigned320(
            WideArithmetic.SubtractSigned320(
                Scale(pointOrigin.Y),
                Scale(frameOrigin.Y)),
            pointY);
        Signed576 xNumerator = WideArithmetic.AddSigned576(
            WideArithmetic.MultiplySigned576(
                Signed576.ExtendValue(deltaX),
                Signed192.Raw(frameCosine)),
            WideArithmetic.MultiplySigned576(
                Signed576.ExtendValue(deltaY),
                Signed192.Raw(frameSine)));
        Signed576 yNumerator = WideArithmetic.SubtractSigned576(
            WideArithmetic.MultiplySigned576(
                Signed576.ExtendValue(deltaY),
                Signed192.Raw(frameCosine)),
            WideArithmetic.MultiplySigned576(
                Signed576.ExtendValue(deltaX),
                Signed192.Raw(frameSine)));
        Signed576 denominator = Signed576.ExtendValue(
            WideArithmetic.AddSigned320(
                Product(frameCosine, frameCosine),
                Product(frameSine, frameSine)));
        bool representable =
            Fixed64.TryGetSignedRawRatio(
                xNumerator,
                denominator,
                out Fixed64 x)
            & Fixed64.TryGetSignedRawRatio(
                yNumerator,
                denominator,
                out Fixed64 y);
        localPoint = representable ? new Vector2d(x, y) : default;
        return representable;
    }

    internal static bool TryGetRelativeOffset(
        Vector2d firstOrigin,
        Vector2d firstLocalPoint,
        Fixed64 firstAngleInRadians,
        Vector2d secondOrigin,
        Vector2d secondLocalPoint,
        Fixed64 secondAngleInRadians,
        out Vector2d result) =>
        TryGetRelativeOffset(
            firstOrigin,
            firstLocalPoint,
            Vector2d.Zero,
            firstAngleInRadians,
            secondOrigin,
            secondLocalPoint,
            Vector2d.Zero,
            secondAngleInRadians,
            out result);

    internal static bool TryGetRelativeOffset(
        Vector2d firstOrigin,
        Vector2d firstLocalPoint,
        Vector2d firstLocalDisplacement,
        Fixed64 firstAngleInRadians,
        Vector2d secondOrigin,
        Vector2d secondLocalPoint,
        Vector2d secondLocalDisplacement,
        Fixed64 secondAngleInRadians,
        out Vector2d result)
    {
        Fixed64 firstCosine = FixedMath.Cos(firstAngleInRadians);
        Fixed64 firstSine = FixedMath.Sin(firstAngleInRadians);
        Fixed64 secondCosine = FixedMath.Cos(secondAngleInRadians);
        Fixed64 secondSine = FixedMath.Sin(secondAngleInRadians);
        Signed576 denominator = Signed576.One;
        GetRotationNumerators(
            firstLocalPoint,
            firstLocalDisplacement,
            firstCosine,
            firstSine,
            out Signed320 firstRotatedX,
            out Signed320 firstRotatedY);
        GetRotationNumerators(
            secondLocalPoint,
            secondLocalDisplacement,
            secondCosine,
            secondSine,
            out Signed320 secondRotatedX,
            out Signed320 secondRotatedY);
        bool representable = TryRoundCoordinate(
                GetRelativeNumerator(
                    firstOrigin.X,
                    firstRotatedX,
                    secondOrigin.X,
                    secondRotatedX),
                denominator,
                out Fixed64 x)
            & TryRoundCoordinate(
                GetRelativeNumerator(
                    firstOrigin.Y,
                    firstRotatedY,
                    secondOrigin.Y,
                    secondRotatedY),
                denominator,
                out Fixed64 y);
        if (!representable)
        {
            result = default;
            return false;
        }

        result = new Vector2d(x, y);
        return true;
    }

    internal static void GetRotationNumerators(
        Vector2d localPoint,
        Vector2d localDisplacement,
        Fixed64 cosine,
        Fixed64 sine,
        out Signed320 x,
        out Signed320 y)
    {
        GetRotationNumerators(
            localPoint,
            cosine,
            sine,
            out x,
            out y);
        GetRotationNumerators(
            localDisplacement,
            cosine,
            sine,
            out Signed320 displacementX,
            out Signed320 displacementY);
        x = WideArithmetic.AddSigned320(x, displacementX);
        y = WideArithmetic.AddSigned320(y, displacementY);
    }

    private static void GetRotationNumerators(
        Vector2d localPoint,
        Fixed64 cosine,
        Fixed64 sine,
        out Signed320 x,
        out Signed320 y)
    {
        x = WideArithmetic.SubtractSigned320(
            Product(localPoint.X, cosine),
            Product(localPoint.Y, sine));
        y = WideArithmetic.AddSigned320(
            Product(localPoint.X, sine),
            Product(localPoint.Y, cosine));
    }

    private static void GetScaledRotationNumerators(
        Vector2d localPoint,
        Vector2d scale,
        Vector2d localDisplacement,
        Fixed64 cosine,
        Fixed64 sine,
        out Signed576 x,
        out Signed576 y)
    {
        Signed320 localX = Product(localPoint.X, scale.X);
        Signed320 localY = Product(localPoint.Y, scale.Y);
        Signed320 one =
            Signed320.ExtendValue(Signed192.One);
        Signed576 displacementX = WideArithmetic.MultiplySigned320(
            WideArithmetic.SubtractSigned320(
                Product(localDisplacement.X, cosine),
                Product(localDisplacement.Y, sine)),
            one);
        Signed576 displacementY = WideArithmetic.MultiplySigned320(
            WideArithmetic.AddSigned320(
                Product(localDisplacement.X, sine),
                Product(localDisplacement.Y, cosine)),
            one);
        x = WideArithmetic.AddSigned576(
            WideArithmetic.SubtractSigned576(
                WideArithmetic.MultiplySigned320(
                    localX,
                    Signed320.ExtendValue(Signed192.Raw(cosine))),
                WideArithmetic.MultiplySigned320(
                    localY,
                    Signed320.ExtendValue(Signed192.Raw(sine)))),
            displacementX);
        y = WideArithmetic.AddSigned576(
            WideArithmetic.AddSigned576(
                WideArithmetic.MultiplySigned320(
                    localX,
                    Signed320.ExtendValue(Signed192.Raw(sine))),
                WideArithmetic.MultiplySigned320(
                    localY,
                    Signed320.ExtendValue(Signed192.Raw(cosine)))),
            displacementY);
    }

    private static Signed320 GetRelativeNumerator(
        Fixed64 firstOrigin,
        Fixed64 firstOffset,
        Fixed64 secondOrigin,
        Signed320 rotatedLocalPoint) =>
        WideArithmetic.SubtractSigned320(
            WideArithmetic.SubtractSigned320(
                WideArithmetic.AddSigned320(
                    Scale(firstOrigin),
                    Scale(firstOffset)),
                Scale(secondOrigin)),
            rotatedLocalPoint);

    private static Signed320 GetRelativeNumerator(
        Fixed64 firstOrigin,
        Signed320 firstRotatedLocalPoint,
        Fixed64 secondOrigin,
        Signed320 secondRotatedLocalPoint) =>
        WideArithmetic.SubtractSigned320(
            WideArithmetic.AddSigned320(
                WideArithmetic.SubtractSigned320(
                    Scale(firstOrigin),
                    Scale(secondOrigin)),
                firstRotatedLocalPoint),
            secondRotatedLocalPoint);

    private static Signed320 GetTransformNumerator(
        Fixed64 firstOrigin,
        Fixed64 secondOrigin,
        Signed320 rotatedLocalPoint) =>
        WideArithmetic.AddSigned320(
            WideArithmetic.AddSigned320(
                Scale(firstOrigin),
                Scale(secondOrigin)),
            rotatedLocalPoint);

    internal static Signed320 Scale(Fixed64 value)
    {
        long raw = value.m_rawValue;
        ulong extension = raw < 0L ? ulong.MaxValue : 0UL;
        ulong unsignedRaw = unchecked((ulong)raw);
        return new Signed320(
            extension,
            extension,
            extension,
            (unsignedRaw >> FixedMath.SHIFT_AMOUNT_I)
                | (extension << FixedMath.SHIFT_AMOUNT_I),
            unsignedRaw << FixedMath.SHIFT_AMOUNT_I);
    }

    private static Signed320 Product(Fixed64 left, Fixed64 right) =>
        WideArithmetic.MultiplySigned192(Signed192.Raw(left), Signed192.Raw(right));

    private static bool TryRoundCoordinate(
        Signed320 numerator,
        Signed576 denominator,
        out Fixed64 result) =>
        Fixed64.TryGetSignedRawRatio(
            Signed576.ExtendValue(numerator),
            denominator,
            out result);

}
