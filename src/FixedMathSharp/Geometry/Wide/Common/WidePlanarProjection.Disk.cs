//=======================================================================
// WidePlanarProjection.Disk.cs
//=======================================================================
// MIT License, Copyright (c) 2024-present David Oravsky (mrdav30)
// See LICENSE file in the project root for full license information.
//=======================================================================

using System;

namespace FixedMathSharp.Geometry;

/// <content>
/// Exact projected-disk admission and correctly rounded planar distance.
/// </content>
internal static partial class WidePlanarProjection
{
    private static bool TryGetProjectedDiskRelation(
        Vector2d circleCenter,
        Fixed64 circleRadius,
        RationalPoint diskCenter,
        Signed192 coordinateDenominator,
        Signed192 axisX,
        Signed192 axisY,
        Signed192 axisZ,
        Fixed64 diskRadius,
        out PlanarProjectionRelation relation)
    {
        if (axisY.IsZero)
        {
            GetProjectedDiskSegmentEndpoints(
                diskCenter,
                axisX,
                axisZ,
                diskRadius,
                out RationalPoint first,
                out RationalPoint second);
            return TryCreateRelation(
                GetSegmentDistance(
                    circleCenter,
                    coordinateDenominator,
                    first,
                    second),
                circleRadius,
                out relation);
        }

        RationalDistance centerDistance = GetPointDistance(
            circleCenter,
            coordinateDenominator,
            diskCenter);
        if (axisX.IsZero && axisZ.IsZero)
        {
            return TryCreateRadiallyExpandedRelation(
                centerDistance,
                diskRadius,
                circleRadius,
                out relation);
        }

        int maximumComparison = CompareProjectedDiskDistanceToRadius(
            centerDistance,
            coordinateDenominator,
            axisX,
            axisY,
            axisZ,
            diskRadius,
            Signed192.Raw(circleRadius),
            radiusDenominator: 1);
        if (maximumComparison > 0)
        {
            relation = default;
            return false;
        }

        int zeroComparison = CompareProjectedDiskDistanceToRadius(
            centerDistance,
            coordinateDenominator,
            axisX,
            axisY,
            axisZ,
            diskRadius,
            default,
            radiusDenominator: 1);
        if (zeroComparison <= 0)
        {
            relation = PlanarProjectionRelation.Contained;
            return true;
        }

        GetProjectedDiskClosestOffset(
            centerDistance,
            coordinateDenominator,
            axisX,
            axisY,
            axisZ,
            diskRadius,
            out Fixed64 approximateDistance,
            out Vector2d direction);
        long low = GetProjectedDiskCeilingDistance(
            centerDistance,
            coordinateDenominator,
            axisX,
            axisY,
            axisZ,
            diskRadius,
            circleRadius,
            approximateDistance);

        long floor = low - 1L;
        Signed192 midpointNumerator = WideArithmetic.AddSigned192(
            WideArithmetic.AddSigned192(
                Signed192.Signed(floor),
                Signed192.Signed(floor)),
            Signed192.Signed(1L));
        int midpointComparison = CompareProjectedDiskDistanceToRadius(
            centerDistance,
            coordinateDenominator,
            axisX,
            axisY,
            axisZ,
            diskRadius,
            midpointNumerator,
            radiusDenominator: 2);
        long rounded = midpointComparison < 0
            ? floor
            : midpointComparison > 0
                ? low
                : floor + (floor & 1L);
        Fixed64 distance = Fixed64.FromRaw(rounded);
        relation = new PlanarProjectionRelation(
            distance,
            direction * distance,
            direction);
        return true;
    }

    private static long GetProjectedDiskCeilingDistance(
        RationalDistance centerDistance,
        Signed192 coordinateDenominator,
        Signed192 axisX,
        Signed192 axisY,
        Signed192 axisZ,
        Fixed64 diskRadius,
        Fixed64 maximum,
        Fixed64 approximate)
    {
        long candidate = approximate.m_rawValue;
        if (candidate > maximum.m_rawValue)
            candidate = maximum.m_rawValue;

        int comparison = CompareProjectedDiskDistanceToRadius(
            centerDistance,
            coordinateDenominator,
            axisX,
            axisY,
            axisZ,
            diskRadius,
            Signed192.Signed(candidate),
            radiusDenominator: 1);
        if (comparison <= 0)
        {
            long high = candidate;
            long step = 1L;
            while (high > 1L)
            {
                long probe = high > step ? high - step : 0L;
                if (probe == 0L
                    || CompareProjectedDiskDistanceToRadius(
                        centerDistance,
                        coordinateDenominator,
                        axisX,
                        axisY,
                        axisZ,
                        diskRadius,
                        Signed192.Signed(probe),
                        radiusDenominator: 1) > 0)
                {
                    return FindProjectedDiskCeilingDistance(
                        centerDistance,
                        coordinateDenominator,
                        axisX,
                        axisY,
                        axisZ,
                        diskRadius,
                        probe + 1L,
                        high);
                }

                high = probe;
                step = (long)Math.Min(
                    (ulong)step << 1,
                    (ulong)high);
            }

            return 1L;
        }

        long low = candidate + 1L;
        long upper = maximum.m_rawValue;
        long upwardStep = 1L;
        while (low < upper)
        {
            long remaining = upper - candidate;
            long probe = candidate + Math.Min(
                upwardStep,
                remaining);
            if (CompareProjectedDiskDistanceToRadius(
                centerDistance,
                coordinateDenominator,
                axisX,
                axisY,
                axisZ,
                diskRadius,
                Signed192.Signed(probe),
                radiusDenominator: 1) <= 0)
            {
                return FindProjectedDiskCeilingDistance(
                    centerDistance,
                    coordinateDenominator,
                    axisX,
                    axisY,
                    axisZ,
                    diskRadius,
                    low,
                    probe);
            }

            low = probe + 1L;
            upwardStep = (long)Math.Min(
                (ulong)upwardStep << 1,
                (ulong)(upper - candidate));
        }

        return upper;
    }

    private static long FindProjectedDiskCeilingDistance(
        RationalDistance centerDistance,
        Signed192 coordinateDenominator,
        Signed192 axisX,
        Signed192 axisY,
        Signed192 axisZ,
        Fixed64 diskRadius,
        long low,
        long high)
    {
        while (low < high)
        {
            long middle = low + ((high - low) >> 1);
            int comparison = CompareProjectedDiskDistanceToRadius(
                centerDistance,
                coordinateDenominator,
                axisX,
                axisY,
                axisZ,
                diskRadius,
                Signed192.Signed(middle),
                radiusDenominator: 1);
            if (comparison <= 0)
                high = middle;
            else
                low = middle + 1L;
        }

        return low;
    }

    private static void GetProjectedDiskClosestOffset(
        RationalDistance centerDistance,
        Signed192 coordinateDenominator,
        Signed192 axisX,
        Signed192 axisY,
        Signed192 axisZ,
        Fixed64 diskRadius,
        out Fixed64 approximateDistance,
        out Vector2d direction)
    {
        Signed320 relativeX = Signed320.NarrowValue(centerDistance.X);
        Signed320 relativeZ = Signed320.NarrowValue(centerDistance.Z);
        Signed320 scaledRadius = WideArithmetic.MultiplySigned192(
            coordinateDenominator,
            Signed192.Raw(diskRadius));
        Signed320 scale = GetLargestPositiveMagnitude(
            relativeX,
            relativeZ,
            default,
            scaledRadius,
            default);
        Fixed64 queryX = -Fixed64.GetSignedRatio(relativeX, scale);
        Fixed64 queryZ = -Fixed64.GetSignedRatio(relativeZ, scale);
        Fixed64 radius = Fixed64.GetSignedRatio(scaledRadius, scale);
        Vector2d axis = WideNormalization.GetNormalized(axisX, axisZ);
        Vector2d perpendicular = new(-axis.Y, axis.X);
        Fixed64 along = Vector2d.Dot(new Vector2d(queryX, queryZ), axis);
        Fixed64 across = Vector2d.Dot(
            new Vector2d(queryX, queryZ),
            perpendicular);
        Fixed64 axisYRatio = FixedMath.Abs(
            WideNormalization.GetNormalized(
                axisX,
                axisY,
                axisZ).Y);
        Fixed64 minor = radius * axisYRatio;
        Vector2d localOffset = GetEllipseClosestOffset(
            along,
            across,
            minor,
            radius);
        Vector2d worldOffset =
            axis * localOffset.X + perpendicular * localOffset.Y;
        direction = worldOffset.Normalized;

        Signed576 distanceNumerator =
            WideArithmetic.MultiplySigned320(
                scale,
                Signed320.ExtendValue(
                    Signed192.Raw(worldOffset.Magnitude)));
        _ = Fixed64.TryGetSignedRawRatio(
            distanceNumerator,
            Signed576.ExtendValue(
                WideArithmetic.MultiplySigned192(
                    coordinateDenominator,
                    Signed192.Raw(Fixed64.One))),
            out approximateDistance);
    }

    private static Vector2d GetEllipseClosestOffset(
        Fixed64 queryX,
        Fixed64 queryY,
        Fixed64 radiusX,
        Fixed64 radiusY)
    {
        Fixed64 radiusXSquared = radiusX * radiusX;
        Fixed64 radiusYSquared = radiusY * radiusY;
        Fixed64 low = Fixed64.Zero;
        Fixed64 high = Fixed64.Two;
        while (high.m_rawValue - low.m_rawValue > 1L)
        {
            Fixed64 middle = Fixed64.FromRaw(
                low.m_rawValue
                + ((high.m_rawValue - low.m_rawValue) >> 1));
            Fixed64 x = GetEllipseConstraintCoordinate(
                radiusX,
                queryX,
                middle + radiusXSquared);
            Fixed64 y = GetEllipseConstraintCoordinate(
                radiusY,
                queryY,
                middle + radiusYSquared);
            if (x * x + y * y > Fixed64.One)
                low = middle;
            else
                high = middle;
        }

        Fixed64 closestX = Fixed64.MultiplyDivide(
            radiusXSquared,
            queryX,
            high + radiusXSquared,
            out _);
        Fixed64 closestY = Fixed64.MultiplyDivide(
            radiusYSquared,
            queryY,
            high + radiusYSquared,
            out _);
        return new Vector2d(
            closestX - queryX,
            closestY - queryY);
    }

    private static Fixed64 GetEllipseConstraintCoordinate(
        Fixed64 radius,
        Fixed64 query,
        Fixed64 denominator) =>
        Fixed64.MultiplyDivide(
            radius,
            query,
            denominator,
            out _);

    private static void GetProjectedDiskSegmentEndpoints(
        RationalPoint diskCenter,
        Signed192 axisX,
        Signed192 axisZ,
        Fixed64 diskRadius,
        out RationalPoint first,
        out RationalPoint second)
    {
        Signed192 scaledRadius = WideArithmetic.AddSigned192(
            Signed192.Raw(diskRadius),
            Signed192.Raw(diskRadius));

        Signed320 offsetX = WideArithmetic.MultiplySigned192(
            WideArithmetic.SubtractSigned192(
                default,
                axisZ),
            scaledRadius);
        Signed320 offsetZ = WideArithmetic.MultiplySigned192(
            axisX,
            scaledRadius);
        first = new RationalPoint(
            WideArithmetic.AddSigned320(
                diskCenter.X,
                offsetX),
            WideArithmetic.AddSigned320(
                diskCenter.Z,
                offsetZ));
        second = new RationalPoint(
            WideArithmetic.SubtractSigned320(
                diskCenter.X,
                offsetX),
            WideArithmetic.SubtractSigned320(
                diskCenter.Z,
                offsetZ));
    }

    private static int CompareProjectedDiskDistanceToRadius(
        RationalDistance centerDistance,
        Signed192 coordinateDenominator,
        Signed192 axisX,
        Signed192 axisY,
        Signed192 axisZ,
        Fixed64 diskRadius,
        Signed192 radiusNumerator,
        int radiusDenominator)
    {
        Signed320 relativeX = Signed320.NarrowValue(
            centerDistance.X);
        Signed320 relativeZ = Signed320.NarrowValue(
            centerDistance.Z);
        int centerSign = EvaluateProjectedDiskConic(
            relativeX,
            relativeZ,
            coordinateDenominator,
            axisX,
            axisY,
            axisZ,
            diskRadius);
        if (centerSign <= 0)
            return radiusNumerator.IsZero ? 0 : -1;
        if (radiusNumerator.IsZero)
            return 1;

        int rootCount = CountProjectedDiskCircleRoots(
            relativeX,
            relativeZ,
            coordinateDenominator,
            axisX,
            axisY,
            axisZ,
            diskRadius,
            radiusNumerator,
            radiusDenominator);
        if (rootCount != 0)
            return rootCount == 1 ? 0 : -1;

        int centerDistanceComparison = radiusDenominator == 1
            ? CompareDistanceToRaw(
                centerDistance,
                radiusNumerator)
            : CompareDistanceToTwiceRaw(
                centerDistance,
                radiusNumerator);
        return centerDistanceComparison <= 0 ? -1 : 1;
    }

    private static int CountProjectedDiskCircleRoots(
        Signed320 relativeX,
        Signed320 relativeZ,
        Signed192 coordinateDenominator,
        Signed192 axisX,
        Signed192 axisY,
        Signed192 axisZ,
        Fixed64 diskRadius,
        Signed192 radiusNumerator,
        int radiusDenominator)
    {
        CreateProjectedDiskCirclePolynomial(
            relativeX,
            relativeZ,
            coordinateDenominator,
            axisX,
            axisY,
            axisZ,
            diskRadius,
            radiusNumerator,
            radiusDenominator,
            1,
            0,
            out Signed832 constant,
            out Signed832 linear,
            out Signed832 quadratic,
            out Signed832 cubic,
            out Signed832 quartic);
        if (quartic.Sign == 0)
        {
            CreateProjectedDiskCirclePolynomial(
                relativeX,
                relativeZ,
                coordinateDenominator,
                axisX,
                axisY,
                axisZ,
                diskRadius,
                radiusNumerator,
                radiusDenominator,
                -1,
                0,
                out constant,
                out linear,
                out quadratic,
                out cubic,
                out quartic);
        }

        return WideFiniteAxisIntersection.CountProjectionDomainQuarticRoots(
            constant,
            linear,
            quadratic,
            cubic,
            quartic);
    }

    private static void CreateProjectedDiskCirclePolynomial(
        Signed320 relativeX,
        Signed320 relativeZ,
        Signed192 coordinateDenominator,
        Signed192 axisX,
        Signed192 axisY,
        Signed192 axisZ,
        Fixed64 diskRadius,
        Signed192 radiusNumerator,
        int radiusDenominator,
        int directionX,
        int directionZ,
        out Signed832 constant,
        out Signed832 linear,
        out Signed832 quadratic,
        out Signed832 cubic,
        out Signed832 quartic)
    {
        Signed320 radiusScale = WideArithmetic.MultiplySigned192(
            radiusNumerator,
            coordinateDenominator);
        Signed320 scaledX = MultiplyBySmall(
            relativeX,
            radiusDenominator);
        Signed320 scaledZ = MultiplyBySmall(
            relativeZ,
            radiusDenominator);
        Signed320 radialX = MultiplyBySmall(
            radiusScale,
            directionX);
        Signed320 radialZ = MultiplyBySmall(
            radiusScale,
            directionZ);
        Signed320 perpendicularX = MultiplyBySmall(
            radiusScale,
            -directionZ * 2);
        Signed320 perpendicularZ = MultiplyBySmall(
            radiusScale,
            directionX * 2);
        Signed320 x0 = WideArithmetic.AddSigned320(
            scaledX,
            radialX);
        Signed320 x1 = perpendicularX;
        Signed320 x2 = WideArithmetic.SubtractSigned320(
            scaledX,
            radialX);
        Signed320 z0 = WideArithmetic.AddSigned320(
            scaledZ,
            radialZ);
        Signed320 z1 = perpendicularZ;
        Signed320 z2 = WideArithmetic.SubtractSigned320(
            scaledZ,
            radialZ);

        Signed320 axisY2 = WideArithmetic.MultiplySigned192(
            axisY,
            axisY);
        GetSquaredPolynomial(
            x0,
            x1,
            x2,
            out Signed576 xSquared0,
            out Signed576 xSquared1,
            out Signed576 xSquared2,
            out Signed576 xSquared3,
            out Signed576 xSquared4);
        GetSquaredPolynomial(
            z0,
            z1,
            z2,
            out Signed576 zSquared0,
            out Signed576 zSquared1,
            out Signed576 zSquared2,
            out Signed576 zSquared3,
            out Signed576 zSquared4);
        Signed576 dot0 = WideArithmetic.AddSigned576(
            WideArithmetic.MultiplySigned320(
                x0,
                Signed320.ExtendValue(axisX)),
            WideArithmetic.MultiplySigned320(
                z0,
                Signed320.ExtendValue(axisZ)));
        Signed576 dot1 = WideArithmetic.AddSigned576(
            WideArithmetic.MultiplySigned320(
                x1,
                Signed320.ExtendValue(axisX)),
            WideArithmetic.MultiplySigned320(
                z1,
                Signed320.ExtendValue(axisZ)));
        Signed576 dot2 = WideArithmetic.AddSigned576(
            WideArithmetic.MultiplySigned320(
                x2,
                Signed320.ExtendValue(axisX)),
            WideArithmetic.MultiplySigned320(
                z2,
                Signed320.ExtendValue(axisZ)));

        Signed192 commonDenominator = coordinateDenominator;
        if (radiusDenominator == 2)
        {
            commonDenominator = WideArithmetic.AddSigned192(
                commonDenominator,
                commonDenominator);
        }
        Signed320 diskRadiusSquared = WideArithmetic.MultiplySigned192(
            Signed192.Raw(diskRadius),
            Signed192.Raw(diskRadius));
        Signed576 diskAndAxis = WideArithmetic.MultiplySigned320(
            diskRadiusSquared,
            axisY2);
        Signed320 denominatorSquared =
            WideArithmetic.MultiplySigned192(
                commonDenominator,
                commonDenominator);
        Signed832 limit =
            WideArithmetic.MultiplySigned576ToSigned832(
                diskAndAxis,
                denominatorSquared);

        constant = WideArithmetic.SubtractSigned832(
            GetProjectedDiskConicCoefficient(
                xSquared0,
                zSquared0,
                axisY2,
                dot0,
                dot0),
            limit);
        linear = WideArithmetic.AddSigned832(
            GetProjectedDiskRadialCoefficient(
                xSquared1,
                zSquared1,
                axisY2),
            DoubleSigned832(
                WideArithmetic.MultiplySigned576ToSigned832(
                    dot0,
                    dot1)));
        quadratic = WideArithmetic.AddSigned832(
            GetProjectedDiskConicCoefficient(
                xSquared2,
                zSquared2,
                axisY2,
                dot1,
                dot1),
            DoubleSigned832(
                WideArithmetic.MultiplySigned576ToSigned832(
                    dot0,
                    dot2)));
        quadratic = WideArithmetic.SubtractSigned832(
            quadratic,
            DoubleSigned832(limit));
        cubic = WideArithmetic.AddSigned832(
            GetProjectedDiskRadialCoefficient(
                xSquared3,
                zSquared3,
                axisY2),
            DoubleSigned832(
                WideArithmetic.MultiplySigned576ToSigned832(
                    dot1,
                    dot2)));
        quartic = WideArithmetic.SubtractSigned832(
            GetProjectedDiskConicCoefficient(
                xSquared4,
                zSquared4,
                axisY2,
                dot2,
                dot2),
            limit);
    }

    private static Signed832 GetProjectedDiskConicCoefficient(
        Signed576 xSquared,
        Signed576 zSquared,
        Signed320 axisYSquared,
        Signed576 firstDot,
        Signed576 secondDot) =>
        WideArithmetic.AddSigned832(
            GetProjectedDiskRadialCoefficient(
                xSquared,
                zSquared,
                axisYSquared),
            WideArithmetic.MultiplySigned576ToSigned832(
                firstDot,
                secondDot));

    private static Signed832 GetProjectedDiskRadialCoefficient(
        Signed576 xSquared,
        Signed576 zSquared,
        Signed320 axisYSquared) =>
        WideArithmetic.MultiplySigned576ToSigned832(
            WideArithmetic.AddSigned576(
                xSquared,
                zSquared),
            axisYSquared);

    private static void GetSquaredPolynomial(
        Signed320 constant,
        Signed320 linear,
        Signed320 quadratic,
        out Signed576 result0,
        out Signed576 result1,
        out Signed576 result2,
        out Signed576 result3,
        out Signed576 result4)
    {
        result0 = WideArithmetic.MultiplySigned320(
            constant,
            constant);
        result1 = WideArithmetic.AddSigned576(
            WideArithmetic.MultiplySigned320(
                constant,
                linear),
            WideArithmetic.MultiplySigned320(
                constant,
                linear));
        result2 = WideArithmetic.AddSigned576(
            WideArithmetic.MultiplySigned320(
                linear,
                linear),
            WideArithmetic.AddSigned576(
                WideArithmetic.MultiplySigned320(
                    constant,
                    quadratic),
                WideArithmetic.MultiplySigned320(
                    constant,
                    quadratic)));
        result3 = WideArithmetic.AddSigned576(
            WideArithmetic.MultiplySigned320(
                linear,
                quadratic),
            WideArithmetic.MultiplySigned320(
                linear,
                quadratic));
        result4 = WideArithmetic.MultiplySigned320(
            quadratic,
            quadratic);
    }

    private static Signed320 MultiplyBySmall(
        Signed320 value,
        int multiplier)
    {
        if (multiplier == 0)
            return default;
        Signed320 result = multiplier < 0
            ? WideArithmetic.SubtractSigned320(default, value)
            : value;
        return multiplier is 2 or -2
            ? WideArithmetic.AddSigned320(result, result)
            : result;
    }

    private static Signed832 DoubleSigned832(Signed832 value) =>
        WideArithmetic.AddSigned832(value, value);

    private static int EvaluateProjectedDiskConic(
        Signed320 relativeX,
        Signed320 relativeZ,
        Signed192 coordinateDenominator,
        Signed192 axisX,
        Signed192 axisY,
        Signed192 axisZ,
        Fixed64 diskRadius)
    {
        Signed576 distanceSquared = WideArithmetic.AddSigned576(
            WideArithmetic.MultiplySigned320(
                relativeX,
                relativeX),
            WideArithmetic.MultiplySigned320(
                relativeZ,
                relativeZ));
        Signed576 dot = WideArithmetic.AddSigned576(
            WideArithmetic.MultiplySigned320(
                relativeX,
                Signed320.ExtendValue(axisX)),
            WideArithmetic.MultiplySigned320(
                relativeZ,
                Signed320.ExtendValue(axisZ)));
        Signed320 axisY2 = WideArithmetic.MultiplySigned192(
            axisY,
            axisY);
        Signed832 left = WideArithmetic.AddSigned832(
            WideArithmetic.MultiplySigned576ToSigned832(
                distanceSquared,
                axisY2),
            WideArithmetic.MultiplySigned576ToSigned832(
                dot,
                dot));
        Signed320 radiusSquared = WideArithmetic.MultiplySigned192(
            Signed192.Raw(diskRadius),
            Signed192.Raw(diskRadius));
        Signed576 radiusAndAxis = WideArithmetic.MultiplySigned320(
            radiusSquared,
            axisY2);
        Signed320 denominatorSquared =
            WideArithmetic.MultiplySigned192(
                coordinateDenominator,
                coordinateDenominator);
        Signed832 right =
            WideArithmetic.MultiplySigned576ToSigned832(
                radiusAndAxis,
                denominatorSquared);
        return WideArithmetic.SubtractSigned832(left, right).Sign;
    }
}
