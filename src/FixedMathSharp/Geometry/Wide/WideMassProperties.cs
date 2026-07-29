//=======================================================================
// WideMassProperties.cs
//=======================================================================
// MIT License, Copyright (c) 2024-present David Oravsky (mrdav30)
// See LICENSE file in the project root for full license information.
//=======================================================================

using System;

namespace FixedMathSharp.Geometry;

/// <summary>
/// Owns exact bounded arithmetic for semantic mass-property values.
/// </summary>
internal static class WideMassProperties
{
    private static readonly Signed192 One = Signed192.One;
    private static readonly Signed320 Point3dDenominator =
        WideArithmetic.MultiplySigned192(One, One);
    private static readonly Signed576 WeightMeasureDenominator =
        PowerOfOne(3);
    private static readonly Signed576 ParallelAxis2dDenominator =
        PowerOfOne(4);
    private static readonly Signed576 ParallelAxis3dDenominator =
        PowerOfOne(6);

    internal static FixedMassWeight CreateWeight(Fixed64 measure)
    {
        EnsureNonNegative(measure, nameof(measure));
        return CreateWeightCore(
            measure,
            Fixed64.One,
            Fixed64.One,
            Fixed64.One);
    }

    internal static FixedMassWeight CreateWeight(
        Fixed64 first,
        Fixed64 second)
    {
        EnsureNonNegative(first, nameof(first));
        EnsureNonNegative(second, nameof(second));
        return CreateWeightCore(
            first,
            second,
            Fixed64.One,
            Fixed64.One);
    }

    internal static FixedMassWeight CreateWeight(
        Fixed64 first,
        Fixed64 second,
        Fixed64 third)
    {
        EnsureNonNegative(first, nameof(first));
        EnsureNonNegative(second, nameof(second));
        EnsureNonNegative(third, nameof(third));
        return CreateWeightCore(
            first,
            second,
            third,
            Fixed64.One);
    }

    internal static FixedMassWeight CreateWeight(
        Fixed64 first,
        Fixed64 second,
        Fixed64 third,
        Fixed64 fourth)
    {
        EnsureNonNegative(first, nameof(first));
        EnsureNonNegative(second, nameof(second));
        EnsureNonNegative(third, nameof(third));
        EnsureNonNegative(fourth, nameof(fourth));
        return CreateWeightCore(first, second, third, fourth);
    }

    internal static FixedMassWeight CreateAreaWeight(
        Signed320 signedDoubleArea)
    {
        Signed320 absoluteArea = signedDoubleArea.Sign < 0
            ? WideArithmetic.SubtractSigned320(
                default,
                signedDoubleArea)
            : signedDoubleArea;
        Signed320 halfScale = WideArithmetic.MultiplySigned192(
            Signed192.Raw(Fixed64.Half),
            One);
        return new FixedMassWeight(
            Signed320.NarrowValue(
                WideArithmetic.MultiplySigned320(
                    absoluteArea,
                    halfScale)));
    }

    internal static FixedMassWeight CreateTriangleAreaWeight(
        Signed320 squaredDoubleArea)
    {
        Signed320 scaledRoot =
            WideArithmetic.GetFloorSquareRootScaledByFixed64(
                Signed576.ExtendValue(squaredDoubleArea));
        Signed576 numerator = WideArithmetic.MultiplySigned320(
            scaledRoot,
            Signed320.ExtendValue(
                Signed192.Raw(Fixed64.Half)));
        return new FixedMassWeight(
            Signed320.NarrowValue(numerator));
    }

    internal static bool TryGetMeasure(
        FixedMassWeight weight,
        out Fixed64 measure) =>
        Fixed64.TryGetSignedRawRatio(
            Signed576.ExtendValue(weight.Numerator),
            WeightMeasureDenominator,
            out measure);

    internal static bool TryAdd(
        FixedMassWeight first,
        FixedMassWeight second,
        out FixedMassWeight result)
    {
        Signed576 sum = WideArithmetic.AddSigned576(
            Signed576.ExtendValue(first.Numerator),
            Signed576.ExtendValue(second.Numerator));
        if (!Signed320.TryNarrowSigned(sum, out Signed320 numerator)
            || numerator.Sign < 0)
        {
            result = default;
            return false;
        }

        result = new FixedMassWeight(numerator);
        return true;
    }

    internal static bool TryGetProportionalShare(
        FixedMassWeight weight,
        Fixed64 total,
        FixedMassWeight totalWeight,
        out Fixed64 share)
    {
        if (total < Fixed64.Zero
            || totalWeight.Numerator.Sign <= 0
            || WideArithmetic.CompareNonNegative(
                Signed576.ExtendValue(weight.Numerator),
                Signed576.ExtendValue(totalWeight.Numerator)) > 0)
        {
            share = default;
            return false;
        }

        Signed576 numerator = WideArithmetic.MultiplySigned320(
            weight.Numerator,
            Signed320.ExtendValue(Signed192.Raw(total)));
        return Fixed64.TryGetSignedRawRatio(
            numerator,
            Signed576.ExtendValue(totalWeight.Numerator),
            out share);
    }

    internal static FixedMassPoint CreatePoint(Vector3d point) =>
        new(
            Product(Signed192.Raw(point.X), One, One),
            Product(Signed192.Raw(point.Y), One, One),
            Product(Signed192.Raw(point.Z), One, One));

    internal static FixedMassPoint CreatePoint(
        Vector3d outerPoint,
        Vector3d outerScale,
        Vector3d innerOffset,
        Vector3d innerScale,
        Vector3d innerDisplacement,
        FixedQuaternion innerRotation) =>
        WideOrientedBox.CreateMassPoint(
            outerPoint,
            outerScale,
            innerOffset,
            innerScale,
            innerDisplacement,
            innerRotation);

    internal static FixedMassPoint2d CreatePoint(Vector2d point) =>
        new(
            WideArithmetic.MultiplySigned192(Signed192.Raw(point.X), One),
            WideArithmetic.MultiplySigned192(Signed192.Raw(point.Y), One));

    internal static FixedMassPoint2d CreatePoint(
        Vector2d outerPoint,
        Vector2d outerScale,
        Vector2d innerOffset,
        Vector2d innerScale,
        Vector2d innerDisplacement,
        Fixed64 innerRotation)
    {
        Fixed64 cosine = FixedMath.Cos(innerRotation);
        Fixed64 sine = FixedMath.Sin(innerRotation);
        Signed320 rotatedX = WideArithmetic.SubtractSigned320(
            WideArithmetic.MultiplySigned192(
                Signed192.Raw(innerDisplacement.X),
                Signed192.Raw(cosine)),
            WideArithmetic.MultiplySigned192(
                Signed192.Raw(innerDisplacement.Y),
                Signed192.Raw(sine)));
        Signed320 rotatedY = WideArithmetic.AddSigned320(
            WideArithmetic.MultiplySigned192(
                Signed192.Raw(innerDisplacement.X),
                Signed192.Raw(sine)),
            WideArithmetic.MultiplySigned192(
                Signed192.Raw(innerDisplacement.Y),
                Signed192.Raw(cosine)));
        return new FixedMassPoint2d(
            WideArithmetic.AddSigned320(
                WideArithmetic.AddSigned320(
                    WideArithmetic.MultiplySigned192(
                        Signed192.Raw(outerPoint.X),
                        Signed192.Raw(outerScale.X)),
                    WideArithmetic.MultiplySigned192(
                        Signed192.Raw(innerOffset.X),
                        Signed192.Raw(innerScale.X))),
                rotatedX),
            WideArithmetic.AddSigned320(
                WideArithmetic.AddSigned320(
                    WideArithmetic.MultiplySigned192(
                        Signed192.Raw(outerPoint.Y),
                        Signed192.Raw(outerScale.Y)),
                    WideArithmetic.MultiplySigned192(
                        Signed192.Raw(innerOffset.Y),
                        Signed192.Raw(innerScale.Y))),
                rotatedY));
    }

    internal static bool TryGetPoint(
        FixedMassPoint point,
        out Vector3d result)
    {
        Signed576 denominator =
            Signed576.ExtendValue(Point3dDenominator);
        bool representable = Fixed64.TryGetSignedRawRatio(
                Signed576.ExtendValue(point.XNumerator),
                denominator,
                out Fixed64 x)
            & Fixed64.TryGetSignedRawRatio(
                Signed576.ExtendValue(point.YNumerator),
                denominator,
                out Fixed64 y)
            & Fixed64.TryGetSignedRawRatio(
                Signed576.ExtendValue(point.ZNumerator),
                denominator,
                out Fixed64 z);
        result = representable
            ? new Vector3d(x, y, z)
            : default;
        return representable;
    }

    internal static bool TryGetPoint(
        FixedMassPoint2d point,
        out Vector2d result)
    {
        Signed576 denominator =
            Signed576.ExtendValue(Signed320.ExtendValue(One));
        bool representable = Fixed64.TryGetSignedRawRatio(
                Signed576.ExtendValue(point.XNumerator),
                denominator,
                out Fixed64 x)
            & Fixed64.TryGetSignedRawRatio(
                Signed576.ExtendValue(point.YNumerator),
                denominator,
                out Fixed64 y);
        result = representable
            ? new Vector2d(x, y)
            : default;
        return representable;
    }

    internal static bool TryGetWeightedAverage(
        ReadOnlySpan<FixedMassPoint> points,
        ReadOnlySpan<FixedMassWeight> weights,
        out Vector3d average)
    {
        ValidateWeightedInputs(points.Length, weights.Length);
        Signed576 totalWeight = default;
        Signed576 x = default;
        Signed576 y = default;
        Signed576 z = default;
        for (int i = 0; i < points.Length; i++)
        {
            Signed320 weight = weights[i].Numerator;
            totalWeight = WideArithmetic.AddSigned576(
                totalWeight,
                Signed576.ExtendValue(weight));
            x = WideArithmetic.AddSigned576(
                x,
                WideArithmetic.MultiplySigned320(
                    points[i].XNumerator,
                    weight));
            y = WideArithmetic.AddSigned576(
                y,
                WideArithmetic.MultiplySigned320(
                    points[i].YNumerator,
                    weight));
            z = WideArithmetic.AddSigned576(
                z,
                WideArithmetic.MultiplySigned320(
                    points[i].ZNumerator,
                    weight));
        }

        if (totalWeight.Sign <= 0)
        {
            average = default;
            return false;
        }

        Signed576 denominator = WideArithmetic.MultiplySigned576(
            totalWeight,
            One,
            One);
        bool representable = Fixed64.TryGetSignedRawRatio(
                x,
                denominator,
                out Fixed64 resultX)
            & Fixed64.TryGetSignedRawRatio(
                y,
                denominator,
                out Fixed64 resultY)
            & Fixed64.TryGetSignedRawRatio(
                z,
                denominator,
                out Fixed64 resultZ);
        average = representable
            ? new Vector3d(resultX, resultY, resultZ)
            : default;
        return representable;
    }

    internal static bool TryGetWeightedAverage(
        ReadOnlySpan<FixedMassPoint2d> points,
        ReadOnlySpan<FixedMassWeight> weights,
        out Vector2d average)
    {
        ValidateWeightedInputs(points.Length, weights.Length);
        Signed576 totalWeight = default;
        Signed576 x = default;
        Signed576 y = default;
        for (int i = 0; i < points.Length; i++)
        {
            Signed320 weight = weights[i].Numerator;
            totalWeight = WideArithmetic.AddSigned576(
                totalWeight,
                Signed576.ExtendValue(weight));
            x = WideArithmetic.AddSigned576(
                x,
                WideArithmetic.MultiplySigned320(
                    points[i].XNumerator,
                    weight));
            y = WideArithmetic.AddSigned576(
                y,
                WideArithmetic.MultiplySigned320(
                    points[i].YNumerator,
                    weight));
        }

        if (totalWeight.Sign <= 0)
        {
            average = default;
            return false;
        }

        Signed576 denominator = WideArithmetic.MultiplySigned576(
            totalWeight,
            One);
        bool representable = Fixed64.TryGetSignedRawRatio(
                x,
                denominator,
                out Fixed64 resultX)
            & Fixed64.TryGetSignedRawRatio(
                y,
                denominator,
                out Fixed64 resultY);
        average = representable
            ? new Vector2d(resultX, resultY)
            : default;
        return representable;
    }

    internal static bool TryGetWeightedAverage(
        ReadOnlySpan<Fixed64> values,
        ReadOnlySpan<FixedMassWeight> weights,
        out Fixed64 average)
    {
        ValidateWeightedInputs(values.Length, weights.Length);
        Signed576 totalWeight = default;
        Signed576 weightedValue = default;
        for (int i = 0; i < values.Length; i++)
        {
            Signed320 weight = weights[i].Numerator;
            totalWeight = WideArithmetic.AddSigned576(
                totalWeight,
                Signed576.ExtendValue(weight));
            weightedValue = WideArithmetic.AddSigned576(
                weightedValue,
                WideArithmetic.MultiplySigned320(
                    weight,
                    Signed320.ExtendValue(
                        Signed192.Raw(values[i]))));
        }

        if (totalWeight.Sign <= 0)
        {
            average = default;
            return false;
        }

        return Fixed64.TryGetSignedRawRatio(
            weightedValue,
            totalWeight,
            out average);
    }

    internal static bool TryAddParallelAxisTensor(
        FixedMassPoint point,
        Fixed3x3 centerTensor,
        Fixed64 mass,
        Vector3d referencePoint,
        out Fixed3x3 tensor)
    {
        if (mass < Fixed64.Zero)
        {
            tensor = default;
            return false;
        }
        if (mass == Fixed64.Zero)
        {
            tensor = centerTensor;
            return true;
        }

        Signed320 dx = WideArithmetic.SubtractSigned320(
            point.XNumerator,
            Product(Signed192.Raw(referencePoint.X), One, One));
        Signed320 dy = WideArithmetic.SubtractSigned320(
            point.YNumerator,
            Product(Signed192.Raw(referencePoint.Y), One, One));
        Signed320 dz = WideArithmetic.SubtractSigned320(
            point.ZNumerator,
            Product(Signed192.Raw(referencePoint.Z), One, One));
        Signed576 dxSquared = WideArithmetic.MultiplySigned320(dx, dx);
        Signed576 dySquared = WideArithmetic.MultiplySigned320(dy, dy);
        Signed576 dzSquared = WideArithmetic.MultiplySigned320(dz, dz);
        bool representable = TryGetParallelAxisValue(
                WideArithmetic.AddSigned576(dySquared, dzSquared),
                mass,
                ParallelAxis3dDenominator,
                out Fixed64 xx)
            & TryGetParallelAxisValue(
                WideArithmetic.AddSigned576(dxSquared, dzSquared),
                mass,
                ParallelAxis3dDenominator,
                out Fixed64 yy)
            & TryGetParallelAxisValue(
                WideArithmetic.AddSigned576(dxSquared, dySquared),
                mass,
                ParallelAxis3dDenominator,
                out Fixed64 zz)
            & TryGetParallelAxisValue(
                WideArithmetic.MultiplySigned320(dx, dy),
                mass,
                ParallelAxis3dDenominator,
                out Fixed64 xy)
            & TryGetParallelAxisValue(
                WideArithmetic.MultiplySigned320(dx, dz),
                mass,
                ParallelAxis3dDenominator,
                out Fixed64 xz)
            & TryGetParallelAxisValue(
                WideArithmetic.MultiplySigned320(dy, dz),
                mass,
                ParallelAxis3dDenominator,
                out Fixed64 yz);
        if (!representable)
        {
            tensor = default;
            return false;
        }

        representable = Fixed64.TryAdd(centerTensor.M11, xx, out Fixed64 m11)
            & Fixed64.TrySubtract(centerTensor.M12, xy, out Fixed64 m12)
            & Fixed64.TrySubtract(centerTensor.M13, xz, out Fixed64 m13)
            & Fixed64.TrySubtract(centerTensor.M21, xy, out Fixed64 m21)
            & Fixed64.TryAdd(centerTensor.M22, yy, out Fixed64 m22)
            & Fixed64.TrySubtract(centerTensor.M23, yz, out Fixed64 m23)
            & Fixed64.TrySubtract(centerTensor.M31, xz, out Fixed64 m31)
            & Fixed64.TrySubtract(centerTensor.M32, yz, out Fixed64 m32)
            & Fixed64.TryAdd(centerTensor.M33, zz, out Fixed64 m33);
        tensor = representable
            ? new Fixed3x3(
                m11, m12, m13,
                m21, m22, m23,
                m31, m32, m33)
            : default;
        return representable;
    }

    internal static bool TryAddParallelAxisMoment(
        FixedMassPoint2d point,
        Fixed64 centerMoment,
        Fixed64 mass,
        Vector2d referencePoint,
        out Fixed64 moment)
    {
        if (mass < Fixed64.Zero)
        {
            moment = default;
            return false;
        }
        if (mass == Fixed64.Zero)
        {
            moment = centerMoment;
            return true;
        }

        Signed320 dx = WideArithmetic.SubtractSigned320(
            point.XNumerator,
            WideArithmetic.MultiplySigned192(
                Signed192.Raw(referencePoint.X),
                One));
        Signed320 dy = WideArithmetic.SubtractSigned320(
            point.YNumerator,
            WideArithmetic.MultiplySigned192(
                Signed192.Raw(referencePoint.Y),
                One));
        Signed576 squaredDistance = WideArithmetic.AddSigned576(
            WideArithmetic.MultiplySigned320(dx, dx),
            WideArithmetic.MultiplySigned320(dy, dy));
        if (!TryGetParallelAxisValue(
                squaredDistance,
                mass,
                ParallelAxis2dDenominator,
                out Fixed64 shift))
        {
            moment = default;
            return false;
        }

        return Fixed64.TryAdd(centerMoment, shift, out moment);
    }

    private static FixedMassWeight CreateWeightCore(
        Fixed64 first,
        Fixed64 second,
        Fixed64 third,
        Fixed64 fourth)
    {
        Signed576 product = WideArithmetic.MultiplySigned576(
            Signed576.ExtendValue(
                Signed320.ExtendValue(Signed192.Raw(first))),
            Signed192.Raw(second),
            Signed192.Raw(third),
            Signed192.Raw(fourth));
        return new FixedMassWeight(Signed320.NarrowValue(product));
    }

    private static bool TryGetParallelAxisValue(
        Signed576 squaredValue,
        Fixed64 mass,
        Signed576 denominator,
        out Fixed64 value) =>
        Fixed64.TryGetSignedRawRatio(
            WideArithmetic.MultiplySigned576(
                squaredValue,
                Signed192.Raw(mass)),
            denominator,
            out value);

    private static Signed320 Product(
        Signed192 first,
        Signed192 second,
        Signed192 third) =>
        Signed320.NarrowValue(
            WideArithmetic.MultiplySigned576(
                Signed576.ExtendValue(
                    Signed320.ExtendValue(first)),
                second,
                third));

    private static Signed576 PowerOfOne(int exponent)
    {
        Signed576 result = Signed576.ExtendValue(
            Signed320.ExtendValue(Signed192.Signed(1)));
        for (int i = 0; i < exponent; i++)
            result = WideArithmetic.MultiplySigned576(result, One);
        return result;
    }

    private static void EnsureNonNegative(
        Fixed64 value,
        string parameterName)
    {
        if (value < Fixed64.Zero)
        {
            throw new ArgumentOutOfRangeException(
                parameterName,
                "Mass-property weight factors cannot be negative.");
        }
    }

    private static void ValidateWeightedInputs(
        int pointCount,
        int weightCount)
    {
        if (pointCount != weightCount)
        {
            throw new ArgumentException(
                "Mass points and weights must have the same length.",
                "weights");
        }
    }
}
