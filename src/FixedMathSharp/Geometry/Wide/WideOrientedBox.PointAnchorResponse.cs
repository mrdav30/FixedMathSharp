//=======================================================================
// WideOrientedBox.PointAnchorResponse.cs
//=======================================================================
// MIT License, Copyright (c) 2024–present David Oravsky (mrdav30)
// See LICENSE file in the project root for full license information.
//=======================================================================

namespace FixedMathSharp.Geometry;

/// <content>
/// Exact point-anchor response products.
/// </content>
internal static partial class WideOrientedBox
{
    internal static FixedLever GetLever(
        in FixedPointAnchor first,
        in FixedPointAnchor second)
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
            out Signed576 denominator);
        return new FixedLever(x, y, z, denominator);
    }

    internal static bool TryGetLeverVector(
        in FixedLever lever,
        out Vector3d vector)
    {
        bool representable = Fixed64.TryGetSignedRawRatio(
                lever.XNumerator,
                lever.Denominator,
                out Fixed64 x)
            & Fixed64.TryGetSignedRawRatio(
                lever.YNumerator,
                lever.Denominator,
                out Fixed64 y)
            & Fixed64.TryGetSignedRawRatio(
                lever.ZNumerator,
                lever.Denominator,
                out Fixed64 z);
        vector = representable ? new Vector3d(x, y, z) : default;
        return representable;
    }

    internal static bool TryGetCrossProductProjection(
        in FixedLever lever,
        Vector3d crossVector,
        Vector3d projectionVector,
        out Fixed64 projection)
    {
        GetRawCrossProduct(
            crossVector,
            projectionVector,
            out Signed320 coefficientX,
            out Signed320 coefficientY,
            out Signed320 coefficientZ);
        Signed704 numerator = WideArithmetic.AddSigned704(
            WideArithmetic.AddSigned704(
                WideArithmetic.MultiplySigned576ToSigned704(
                    lever.XNumerator,
                    coefficientX),
                WideArithmetic.MultiplySigned576ToSigned704(
                    lever.YNumerator,
                    coefficientY)),
            WideArithmetic.MultiplySigned576ToSigned704(
                lever.ZNumerator,
                coefficientZ));
        Signed320 fixedScaleSquared = WideArithmetic.MultiplySigned192(
            Signed192.One,
            Signed192.One);
        Signed704 denominator =
            WideArithmetic.MultiplySigned576ToSigned704(
                lever.Denominator,
                fixedScaleSquared);
        return Fixed64.TryGetSignedRawRatio(
            numerator,
            denominator,
            out projection);
    }

    internal static bool TryGetCrossProductQuadraticForm(
        in FixedLever lever,
        Vector3d crossVector,
        Fixed3x3 transform,
        out Fixed64 result)
    {
        GetCrossProductNumerators(
            lever.XNumerator,
            lever.YNumerator,
            lever.ZNumerator,
            crossVector,
            out Signed704 crossX,
            out Signed704 crossY,
            out Signed704 crossZ);

        Signed832 numerator = GetQuadraticTerm(
            crossX, crossX, transform.M11);
        numerator = WideArithmetic.AddSigned832(
            numerator,
            GetQuadraticTerm(crossX, crossY, transform.M21));
        numerator = WideArithmetic.AddSigned832(
            numerator,
            GetQuadraticTerm(crossX, crossZ, transform.M31));
        numerator = WideArithmetic.AddSigned832(
            numerator,
            GetQuadraticTerm(crossY, crossX, transform.M12));
        numerator = WideArithmetic.AddSigned832(
            numerator,
            GetQuadraticTerm(crossY, crossY, transform.M22));
        numerator = WideArithmetic.AddSigned832(
            numerator,
            GetQuadraticTerm(crossY, crossZ, transform.M32));
        numerator = WideArithmetic.AddSigned832(
            numerator,
            GetQuadraticTerm(crossZ, crossX, transform.M13));
        numerator = WideArithmetic.AddSigned832(
            numerator,
            GetQuadraticTerm(crossZ, crossY, transform.M23));
        numerator = WideArithmetic.AddSigned832(
            numerator,
            GetQuadraticTerm(crossZ, crossZ, transform.M33));

        Signed832 denominator = WideArithmetic.MultiplySigned576ToSigned832(
            lever.Denominator,
            lever.Denominator);
        // A raw quadratic result carries four Q32.32 denominator factors.
        denominator = WideArithmetic.MultiplySigned832(
            denominator,
            Signed192.One);
        denominator = WideArithmetic.MultiplySigned832(
            denominator,
            Signed192.One);
        denominator = WideArithmetic.MultiplySigned832(
            denominator,
            Signed192.One);
        denominator = WideArithmetic.MultiplySigned832(
            denominator,
            Signed192.One);
        return Fixed64.TryGetSignedRawRatio(
            numerator,
            denominator,
            0,
            out result);
    }

    internal static bool TryGetTransformedScaledCrossProduct(
        in FixedLever lever,
        Vector3d crossVector,
        Fixed3x3 transform,
        Fixed64 firstMultiplier,
        Fixed64 secondMultiplier,
        Fixed64 divisor,
        out Vector3d result)
    {
        GetCrossProductNumerators(
            lever.XNumerator,
            lever.YNumerator,
            lever.ZNumerator,
            crossVector,
            out Signed704 crossX,
            out Signed704 crossY,
            out Signed704 crossZ);
        Signed832 transformedX = TransformCrossProduct(
            crossX,
            crossY,
            crossZ,
            transform.M11,
            transform.M21,
            transform.M31);
        Signed832 transformedY = TransformCrossProduct(
            crossX,
            crossY,
            crossZ,
            transform.M12,
            transform.M22,
            transform.M32);
        Signed832 transformedZ = TransformCrossProduct(
            crossX,
            crossY,
            crossZ,
            transform.M13,
            transform.M23,
            transform.M33);
        Signed320 fixedScaleSquared = WideArithmetic.MultiplySigned192(
            Signed192.One,
            Signed192.One);
        Signed704 transformedDenominator =
            WideArithmetic.MultiplySigned576ToSigned704(
                lever.Denominator,
                fixedScaleSquared);
        Signed320 scaleAndDivisor = WideArithmetic.MultiplySigned192(
            Signed192.One,
            Signed192.Raw(divisor));
        Signed832 denominator =
            WideArithmetic.MultiplySigned704ToSigned832(
                transformedDenominator,
                scaleAndDivisor);
        bool representable = TryGetScaledComponent(
                transformedX,
                firstMultiplier,
                secondMultiplier,
                denominator,
                out Fixed64 resultX)
            & TryGetScaledComponent(
                transformedY,
                firstMultiplier,
                secondMultiplier,
                denominator,
                out Fixed64 resultY)
            & TryGetScaledComponent(
                transformedZ,
                firstMultiplier,
                secondMultiplier,
                denominator,
                out Fixed64 resultZ);
        result = representable
            ? new Vector3d(resultX, resultY, resultZ)
            : default;
        return representable;
    }

    private static void GetRawCrossProduct(
        Vector3d first,
        Vector3d second,
        out Signed320 x,
        out Signed320 y,
        out Signed320 z)
    {
        x = WideArithmetic.SubtractSigned320(
            WideArithmetic.MultiplySigned192(
                Signed192.Raw(first.Y),
                Signed192.Raw(second.Z)),
            WideArithmetic.MultiplySigned192(
                Signed192.Raw(first.Z),
                Signed192.Raw(second.Y)));
        y = WideArithmetic.SubtractSigned320(
            WideArithmetic.MultiplySigned192(
                Signed192.Raw(first.Z),
                Signed192.Raw(second.X)),
            WideArithmetic.MultiplySigned192(
                Signed192.Raw(first.X),
                Signed192.Raw(second.Z)));
        z = WideArithmetic.SubtractSigned320(
            WideArithmetic.MultiplySigned192(
                Signed192.Raw(first.X),
                Signed192.Raw(second.Y)),
            WideArithmetic.MultiplySigned192(
                Signed192.Raw(first.Y),
                Signed192.Raw(second.X)));
    }

    private static void GetCrossProductNumerators(
        Signed576 x,
        Signed576 y,
        Signed576 z,
        Vector3d vector,
        out Signed704 crossX,
        out Signed704 crossY,
        out Signed704 crossZ)
    {
        Signed320 vectorX =
            Signed320.ExtendValue(Signed192.Raw(vector.X));
        Signed320 vectorY =
            Signed320.ExtendValue(Signed192.Raw(vector.Y));
        Signed320 vectorZ =
            Signed320.ExtendValue(Signed192.Raw(vector.Z));
        crossX = WideArithmetic.SubtractSigned704(
            WideArithmetic.MultiplySigned576ToSigned704(y, vectorZ),
            WideArithmetic.MultiplySigned576ToSigned704(z, vectorY));
        crossY = WideArithmetic.SubtractSigned704(
            WideArithmetic.MultiplySigned576ToSigned704(z, vectorX),
            WideArithmetic.MultiplySigned576ToSigned704(x, vectorZ));
        crossZ = WideArithmetic.SubtractSigned704(
            WideArithmetic.MultiplySigned576ToSigned704(x, vectorY),
            WideArithmetic.MultiplySigned576ToSigned704(y, vectorX));
    }

    private static Signed832 TransformCrossProduct(
        Signed704 x,
        Signed704 y,
        Signed704 z,
        Fixed64 coefficientX,
        Fixed64 coefficientY,
        Fixed64 coefficientZ) =>
        WideArithmetic.AddSigned832(
            WideArithmetic.AddSigned832(
                WideArithmetic.MultiplySigned704ToSigned832(
                    x,
                    Signed192.Raw(coefficientX)),
                WideArithmetic.MultiplySigned704ToSigned832(
                    y,
                    Signed192.Raw(coefficientY))),
            WideArithmetic.MultiplySigned704ToSigned832(
                z,
                Signed192.Raw(coefficientZ)));

    private static bool TryGetScaledComponent(
        Signed832 value,
        Fixed64 firstMultiplier,
        Fixed64 secondMultiplier,
        Signed832 denominator,
        out Fixed64 result) =>
        Fixed64.TryGetSignedRawRatio(
            WideArithmetic.MultiplySigned832(
                WideArithmetic.MultiplySigned832(
                    value,
                    Signed192.Raw(firstMultiplier)),
                Signed192.Raw(secondMultiplier)),
            denominator,
            0,
            out result);

    private static Signed832 GetQuadraticTerm(
        Signed704 left,
        Signed704 right,
        Fixed64 coefficient)
    {
        if (left.IsZero | right.IsZero | coefficient == Fixed64.Zero)
            return default;

        return WideArithmetic.MultiplySigned704ToSigned832(
            left,
            right,
            Signed192.Raw(coefficient));
    }
}
