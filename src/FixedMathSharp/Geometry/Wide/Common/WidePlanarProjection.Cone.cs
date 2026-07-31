//=======================================================================
// WidePlanarProjection.Cone.cs
//=======================================================================
// MIT License, Copyright (c) 2024-present David Oravsky (mrdav30)
// See LICENSE file in the project root for full license information.
//=======================================================================

using System;

namespace FixedMathSharp.Geometry;

/// <content>
/// Exact projected-cone hull admission and tangent-segment reduction.
/// </content>
internal static partial class WidePlanarProjection
{
    private const int ConeComparisonWordCount = 56;

    private static bool TryGetRotatedConeRelation(
        Vector2d circleCenter,
        Fixed64 circleRadius,
        RationalPoint baseCenter,
        RationalPoint apex,
        Signed192 coordinateDenominator,
        Signed192 axisDenominator,
        Signed192 axisX,
        Signed192 axisY,
        Signed192 axisZ,
        Fixed64 coneHeight,
        Fixed64 coneRadius,
        out PlanarProjectionRelation relation)
    {
        RationalDistance baseDistance = GetPointDistance(
            circleCenter,
            coordinateDenominator,
            baseCenter);
        RationalDistance apexDistance = GetPointDistance(
            circleCenter,
            coordinateDenominator,
            apex);
        GetConeProjectionData(
            baseDistance,
            coordinateDenominator,
            axisDenominator,
            axisX,
            axisY,
            axisZ,
            coneHeight,
            coneRadius,
            out Signed320 planarAxisSquared,
            out Signed320 axisSquared,
            out Signed576 delta,
            out Signed576 axisProjection,
            out Signed576 perpendicularProjection,
            out Signed576 apexProjectionGap);

        int maximumComparison = CompareConeDistanceToRadius(
            baseDistance,
            apexDistance,
            coordinateDenominator,
            axisDenominator,
            axisX,
            axisY,
            axisZ,
            coneHeight,
            coneRadius,
            planarAxisSquared,
            axisSquared,
            delta,
            axisProjection,
            perpendicularProjection,
            apexProjectionGap,
            Signed192.Raw(circleRadius),
            radiusDenominator: 1);
        if (maximumComparison > 0)
        {
            relation = default;
            return false;
        }

        int zeroComparison = CompareConeDistanceToRadius(
            baseDistance,
            apexDistance,
            coordinateDenominator,
            axisDenominator,
            axisX,
            axisY,
            axisZ,
            coneHeight,
            coneRadius,
            planarAxisSquared,
            axisSquared,
            delta,
            axisProjection,
            perpendicularProjection,
            apexProjectionGap,
            default,
            radiusDenominator: 1);
        if (zeroComparison <= 0)
        {
            relation = PlanarProjectionRelation.Contained;
            return true;
        }

        GetProjectedConeClosestOffset(
            baseDistance,
            apexDistance,
            coordinateDenominator,
            axisX,
            axisY,
            axisZ,
            coneRadius,
            out Fixed64 approximateDistance,
            out Vector2d direction);
        long low = GetProjectedConeCeilingDistance(
            baseDistance,
            apexDistance,
            coordinateDenominator,
            axisDenominator,
            axisX,
            axisY,
            axisZ,
            coneHeight,
            coneRadius,
            planarAxisSquared,
            axisSquared,
            delta,
            axisProjection,
            perpendicularProjection,
            apexProjectionGap,
            circleRadius,
            approximateDistance);

        long floor = low - 1L;
        Signed192 midpointNumerator = WideArithmetic.AddSigned192(
            WideArithmetic.AddSigned192(
                Signed192.Signed(floor),
                Signed192.Signed(floor)),
            Signed192.Signed(1L));
        int midpointComparison = CompareConeDistanceToRadius(
            baseDistance,
            apexDistance,
            coordinateDenominator,
            axisDenominator,
            axisX,
            axisY,
            axisZ,
            coneHeight,
            coneRadius,
            planarAxisSquared,
            axisSquared,
            delta,
            axisProjection,
            perpendicularProjection,
            apexProjectionGap,
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

    private static long GetProjectedConeCeilingDistance(
        RationalDistance baseDistance,
        RationalDistance apexDistance,
        Signed192 coordinateDenominator,
        Signed192 axisDenominator,
        Signed192 axisX,
        Signed192 axisY,
        Signed192 axisZ,
        Fixed64 coneHeight,
        Fixed64 coneRadius,
        Signed320 planarAxisSquared,
        Signed320 axisSquared,
        Signed576 delta,
        Signed576 axisProjection,
        Signed576 perpendicularProjection,
        Signed576 apexProjectionGap,
        Fixed64 maximum,
        Fixed64 approximate)
    {
        long candidate = approximate.m_rawValue;
        if (candidate > maximum.m_rawValue)
            candidate = maximum.m_rawValue;

        int comparison = CompareConeDistanceToRadius(
            baseDistance,
            apexDistance,
            coordinateDenominator,
            axisDenominator,
            axisX,
            axisY,
            axisZ,
            coneHeight,
            coneRadius,
            planarAxisSquared,
            axisSquared,
            delta,
            axisProjection,
            perpendicularProjection,
            apexProjectionGap,
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
                    || CompareConeDistanceToRadius(
                        baseDistance,
                        apexDistance,
                        coordinateDenominator,
                        axisDenominator,
                        axisX,
                        axisY,
                        axisZ,
                        coneHeight,
                        coneRadius,
                        planarAxisSquared,
                        axisSquared,
                        delta,
                        axisProjection,
                        perpendicularProjection,
                        apexProjectionGap,
                        Signed192.Signed(probe),
                        radiusDenominator: 1) > 0)
                {
                    return FindProjectedConeCeilingDistance(
                        baseDistance,
                        apexDistance,
                        coordinateDenominator,
                        axisDenominator,
                        axisX,
                        axisY,
                        axisZ,
                        coneHeight,
                        coneRadius,
                        planarAxisSquared,
                        axisSquared,
                        delta,
                        axisProjection,
                        perpendicularProjection,
                        apexProjectionGap,
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
            if (CompareConeDistanceToRadius(
                baseDistance,
                apexDistance,
                coordinateDenominator,
                axisDenominator,
                axisX,
                axisY,
                axisZ,
                coneHeight,
                coneRadius,
                planarAxisSquared,
                axisSquared,
                delta,
                axisProjection,
                perpendicularProjection,
                apexProjectionGap,
                Signed192.Signed(probe),
                radiusDenominator: 1) <= 0)
            {
                return FindProjectedConeCeilingDistance(
                    baseDistance,
                    apexDistance,
                    coordinateDenominator,
                    axisDenominator,
                    axisX,
                    axisY,
                    axisZ,
                    coneHeight,
                    coneRadius,
                    planarAxisSquared,
                    axisSquared,
                    delta,
                    axisProjection,
                    perpendicularProjection,
                    apexProjectionGap,
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

    private static long FindProjectedConeCeilingDistance(
        RationalDistance baseDistance,
        RationalDistance apexDistance,
        Signed192 coordinateDenominator,
        Signed192 axisDenominator,
        Signed192 axisX,
        Signed192 axisY,
        Signed192 axisZ,
        Fixed64 coneHeight,
        Fixed64 coneRadius,
        Signed320 planarAxisSquared,
        Signed320 axisSquared,
        Signed576 delta,
        Signed576 axisProjection,
        Signed576 perpendicularProjection,
        Signed576 apexProjectionGap,
        long low,
        long high)
    {
        while (low < high)
        {
            long middle = low + ((high - low) >> 1);
            int comparison = CompareConeDistanceToRadius(
                baseDistance,
                apexDistance,
                coordinateDenominator,
                axisDenominator,
                axisX,
                axisY,
                axisZ,
                coneHeight,
                coneRadius,
                planarAxisSquared,
                axisSquared,
                delta,
                axisProjection,
                perpendicularProjection,
                apexProjectionGap,
                Signed192.Signed(middle),
                radiusDenominator: 1);
            if (comparison <= 0)
                high = middle;
            else
                low = middle + 1L;
        }

        return low;
    }

    private static void GetProjectedConeClosestOffset(
        RationalDistance baseDistance,
        RationalDistance apexDistance,
        Signed192 coordinateDenominator,
        Signed192 axisX,
        Signed192 axisY,
        Signed192 axisZ,
        Fixed64 coneRadius,
        out Fixed64 approximateDistance,
        out Vector2d direction)
    {
        Signed320 baseX = Signed320.NarrowValue(baseDistance.X);
        Signed320 baseZ = Signed320.NarrowValue(baseDistance.Z);
        Signed320 apexX = WideArithmetic.SubtractSigned320(
            Signed320.NarrowValue(apexDistance.X),
            baseX);
        Signed320 apexZ = WideArithmetic.SubtractSigned320(
            Signed320.NarrowValue(apexDistance.Z),
            baseZ);
        Signed320 scaledRadius = WideArithmetic.MultiplySigned192(
            coordinateDenominator,
            Signed192.Raw(coneRadius));
        Signed320 scale = GetLargestPositiveMagnitude(
            baseX,
            baseZ,
            apexX,
            apexZ,
            scaledRadius);
        Vector2d query = new(
            -Fixed64.GetSignedRatio(baseX, scale),
            -Fixed64.GetSignedRatio(baseZ, scale));
        Vector2d apex = new(
            Fixed64.GetSignedRatio(apexX, scale),
            Fixed64.GetSignedRatio(apexZ, scale));
        Vector2d axis = WideNormalization.GetNormalized(axisX, axisZ);
        Vector2d perpendicular = new(-axis.Y, axis.X);
        Vector2d localQuery = new(
            Vector2d.Dot(query, axis),
            Vector2d.Dot(query, perpendicular));
        Fixed64 apexLength = Vector2d.Dot(apex, axis);
        Fixed64 radius = Fixed64.GetSignedRatio(scaledRadius, scale);
        Fixed64 minor = radius * FixedMath.Abs(
            WideNormalization.GetNormalized(
                axisX,
                axisY,
                axisZ).Y);
        Vector2d localOffset;
        if (apexLength <= minor)
        {
            localOffset = GetEllipseClosestOffset(
                localQuery.X,
                localQuery.Y,
                minor,
                radius);
        }
        else
        {
            Fixed64 tangentX = Fixed64.MultiplyDivide(
                minor,
                minor,
                apexLength,
                out _);
            Fixed64 ratio = tangentX / apexLength;
            Fixed64 tangentY = radius * FixedMath.Sqrt(
                FixedMath.Max(
                    Fixed64.Zero,
                    Fixed64.One - ratio));
            Vector2d ellipseOffset = GetEllipseClosestOffset(
                localQuery.X,
                localQuery.Y,
                minor,
                radius);
            Vector2d ellipsePoint = localQuery + ellipseOffset;
            bool found = ellipsePoint.X <= tangentX;
            localOffset = ellipseOffset;
            KeepCloserOffset(
                new Vector2d(apexLength, Fixed64.Zero) - localQuery,
                ref found,
                ref localOffset);
            KeepCloserOffset(
                GetSegmentOffset(
                    localQuery,
                    new Vector2d(tangentX, tangentY),
                    new Vector2d(apexLength, Fixed64.Zero)),
                ref found,
                ref localOffset);
            KeepCloserOffset(
                GetSegmentOffset(
                    localQuery,
                    new Vector2d(tangentX, -tangentY),
                    new Vector2d(apexLength, Fixed64.Zero)),
                ref found,
                ref localOffset);
        }

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

    private static Vector2d GetSegmentOffset(
        Vector2d point,
        Vector2d first,
        Vector2d second)
    {
        Vector2d edge = second - first;
        Fixed64 projection = FixedMath.Clamp(
            Vector2d.Dot(point - first, edge) / edge.MagnitudeSquared,
            Fixed64.Zero,
            Fixed64.One);
        return first + edge * projection - point;
    }

    private static void KeepCloserOffset(
        Vector2d candidate,
        ref bool found,
        ref Vector2d best)
    {
        if (!found || candidate.MagnitudeSquared < best.MagnitudeSquared)
        {
            found = true;
            best = candidate;
        }
    }

    private static int CompareConeDistanceToRadius(
        RationalDistance baseDistance,
        RationalDistance apexDistance,
        Signed192 coordinateDenominator,
        Signed192 axisDenominator,
        Signed192 axisX,
        Signed192 axisY,
        Signed192 axisZ,
        Fixed64 coneHeight,
        Fixed64 coneRadius,
        Signed320 planarAxisSquared,
        Signed320 axisSquared,
        Signed576 delta,
        Signed576 axisProjection,
        Signed576 perpendicularProjection,
        Signed576 apexProjectionGap,
        Signed192 radiusNumerator,
        int radiusDenominator)
    {
        int baseComparison = CompareProjectedDiskDistanceToRadius(
            baseDistance,
            coordinateDenominator,
            axisX,
            axisY,
            axisZ,
            coneRadius,
            radiusNumerator,
            radiusDenominator);
        if (delta.Sign <= 0)
            return baseComparison;

        if (IsPointInConeProjectionWedge(
                coordinateDenominator,
                axisDenominator,
                axisY,
                coneHeight,
                coneRadius,
                planarAxisSquared,
                delta,
                axisProjection,
                perpendicularProjection,
                apexProjectionGap))
        {
            return radiusNumerator.IsZero ? 0 : -1;
        }

        int comparison = Math.Min(
            baseComparison,
            CompareRationalDistanceToRadius(
                apexDistance,
                radiusNumerator,
                radiusDenominator));
        if (TryCompareConeTangentDistanceToRadius(
                coordinateDenominator,
                coneHeight,
                coneRadius,
                planarAxisSquared,
                axisSquared,
                delta,
                perpendicularProjection,
                apexProjectionGap,
                radiusNumerator,
                radiusDenominator,
                out int tangentComparison))
        {
            comparison = Math.Min(
                comparison,
                tangentComparison);
        }

        return comparison;
    }

    private static void GetConeProjectionData(
        RationalDistance baseDistance,
        Signed192 coordinateDenominator,
        Signed192 axisDenominator,
        Signed192 axisX,
        Signed192 axisY,
        Signed192 axisZ,
        Fixed64 coneHeight,
        Fixed64 coneRadius,
        out Signed320 planarAxisSquared,
        out Signed320 axisSquared,
        out Signed576 delta,
        out Signed576 axisProjection,
        out Signed576 perpendicularProjection,
        out Signed576 apexProjectionGap)
    {
        Signed320 relativeX = WideArithmetic.SubtractSigned320(
            default,
            Signed320.NarrowValue(baseDistance.X));
        Signed320 relativeZ = WideArithmetic.SubtractSigned320(
            default,
            Signed320.NarrowValue(baseDistance.Z));
        Signed320 axisX2 = WideArithmetic.MultiplySigned192(
            axisX,
            axisX);
        Signed320 axisY2 = WideArithmetic.MultiplySigned192(
            axisY,
            axisY);
        Signed320 axisZ2 = WideArithmetic.MultiplySigned192(
            axisZ,
            axisZ);
        planarAxisSquared = WideArithmetic.AddSigned320(
            axisX2,
            axisZ2);
        axisSquared = WideArithmetic.AddSigned320(
            planarAxisSquared,
            axisY2);
        Signed320 heightSquared = WideArithmetic.MultiplySigned192(
            Signed192.Raw(coneHeight),
            Signed192.Raw(coneHeight));
        Signed320 radiusSquared = WideArithmetic.MultiplySigned192(
            Signed192.Raw(coneRadius),
            Signed192.Raw(coneRadius));
        delta = WideArithmetic.SubtractSigned576(
            WideArithmetic.MultiplySigned320(
                heightSquared,
                planarAxisSquared),
            WideArithmetic.MultiplySigned320(
                radiusSquared,
                axisY2));
        axisProjection = WideArithmetic.AddSigned576(
            WideArithmetic.MultiplySigned320(
                relativeX,
                Signed320.ExtendValue(axisX)),
            WideArithmetic.MultiplySigned320(
                relativeZ,
                Signed320.ExtendValue(axisZ)));
        perpendicularProjection = WideArithmetic.SubtractSigned576(
            WideArithmetic.MultiplySigned320(
                relativeZ,
                Signed320.ExtendValue(axisX)),
            WideArithmetic.MultiplySigned320(
                relativeX,
                Signed320.ExtendValue(axisZ)));
        Signed576 heightTerm = WideArithmetic.MultiplySigned576(
            WideArithmetic.MultiplySigned576(
                Signed576.ExtendValue(planarAxisSquared),
                Signed192.Raw(coneHeight)),
            coordinateDenominator);
        Signed576 projectionTerm = WideArithmetic.MultiplySigned576(
            axisProjection,
            axisDenominator);
        apexProjectionGap = WideArithmetic.SubtractSigned576(
            heightTerm,
            projectionTerm);
    }

    private static bool IsPointInConeProjectionWedge(
        Signed192 coordinateDenominator,
        Signed192 axisDenominator,
        Signed192 axisY,
        Fixed64 coneHeight,
        Fixed64 coneRadius,
        Signed320 planarAxisSquared,
        Signed576 delta,
        Signed576 axisProjection,
        Signed576 perpendicularProjection,
        Signed576 apexProjectionGap)
    {
        if (apexProjectionGap.Sign < 0)
            return false;

        Signed576 tangentStartLeft = WideArithmetic.MultiplySigned576(
            WideArithmetic.MultiplySigned576(
                axisProjection,
                Signed192.Raw(coneHeight)),
            axisDenominator);
        Signed576 tangentStartRight = WideArithmetic.MultiplySigned576(
            WideArithmetic.MultiplySigned320(
                WideArithmetic.MultiplySigned192(
                    Signed192.Raw(coneRadius),
                    Signed192.Raw(coneRadius)),
                WideArithmetic.MultiplySigned192(
                    axisY,
                    axisY)),
            coordinateDenominator);
        if (WideArithmetic.SubtractSigned576(
                tangentStartLeft,
                tangentStartRight).Sign < 0)
        {
            return false;
        }

        Signed576 absolutePerpendicular =
            WideArithmetic.Absolute(perpendicularProjection);
        Signed576 radiusGap = WideArithmetic.MultiplySigned576(
            apexProjectionGap,
            Signed192.Raw(coneRadius));
        return WideArithmetic.CompareSignedLinearRadicalToZero(
                   Signed832.ExtendValue(
                       WideArithmetic.SubtractSigned576(
                           default,
                           radiusGap)),
                   Signed704.ExtendValue(absolutePerpendicular),
                   delta,
                   Signed320.ExtendValue(Signed192.Signed(1L))) <= 0;
    }

    private static bool TryCompareConeTangentDistanceToRadius(
        Signed192 coordinateDenominator,
        Fixed64 coneHeight,
        Fixed64 coneRadius,
        Signed320 planarAxisSquared,
        Signed320 axisSquared,
        Signed576 delta,
        Signed576 perpendicularProjection,
        Signed576 apexProjectionGap,
        Signed192 radiusNumerator,
        int radiusDenominator,
        out int comparison)
    {
        Signed576 absolutePerpendicular =
            WideArithmetic.Absolute(perpendicularProjection);
        Signed576 radiusPerpendicular = WideArithmetic.MultiplySigned576(
            absolutePerpendicular,
            Signed192.Raw(coneRadius));
        Signed704 lowerRational =
            WideArithmetic.MultiplySigned576ToSigned704(
                radiusPerpendicular,
                axisSquared);
        int lowerGate = WideArithmetic.CompareSignedLinearRadicalToZero(
            Signed832.ExtendValue(lowerRational),
            Signed704.ExtendValue(apexProjectionGap),
            delta,
            Signed320.ExtendValue(Signed192.Signed(1L)));
        if (lowerGate <= 0)
        {
            comparison = default;
            return false;
        }

        Signed320 sideSquared = WideArithmetic.AddSigned320(
            WideArithmetic.MultiplySigned192(
                Signed192.Raw(coneHeight),
                Signed192.Raw(coneHeight)),
            WideArithmetic.MultiplySigned192(
                Signed192.Raw(coneRadius),
                Signed192.Raw(coneRadius)));
        Signed576 upperCoefficient = WideArithmetic.SubtractSigned576(
            WideArithmetic.MultiplySigned576(
                apexProjectionGap,
                Signed192.Raw(coneHeight)),
            WideArithmetic.MultiplySigned576(
                WideArithmetic.MultiplySigned320(
                    sideSquared,
                    planarAxisSquared),
                coordinateDenominator));
        Signed576 heightRadiusPerpendicular =
            WideArithmetic.MultiplySigned576(
                absolutePerpendicular,
                Signed192.Raw(coneHeight),
                Signed192.Raw(coneRadius));
        Signed704 upperRational =
            WideArithmetic.MultiplySigned576ToSigned704(
                heightRadiusPerpendicular,
                axisSquared);
        int upperGate = WideArithmetic.CompareSignedLinearRadicalToZero(
            Signed832.ExtendValue(upperRational),
            Signed704.ExtendValue(upperCoefficient),
            delta,
            Signed320.ExtendValue(Signed192.Signed(1L)));
        if (upperGate >= 0)
        {
            comparison = default;
            return false;
        }

        Signed576 radiusGap = WideArithmetic.MultiplySigned576(
            apexProjectionGap,
            Signed192.Raw(coneRadius));
        comparison = CompareConeTangentSquaredDistance(
            absolutePerpendicular,
            delta,
            radiusGap,
            coordinateDenominator,
            planarAxisSquared,
            sideSquared,
            radiusNumerator,
            radiusDenominator);
        return true;
    }

    private static int CompareConeTangentSquaredDistance(
        Signed576 radicalCoefficient,
        Signed576 radicand,
        Signed576 rationalMagnitude,
        Signed192 coordinateDenominator,
        Signed320 planarAxisSquared,
        Signed320 sideSquared,
        Signed192 radiusNumerator,
        int radiusDenominator)
    {
        Span<ulong> first = stackalloc ulong[ConeComparisonWordCount];
        Span<ulong> second = stackalloc ulong[ConeComparisonWordCount];
        Span<ulong> third = stackalloc ulong[ConeComparisonWordCount];
        Span<ulong> result = stackalloc ulong[ConeComparisonWordCount];
        Span<ulong> other = stackalloc ulong[ConeComparisonWordCount];
        Span<ulong> coefficient = stackalloc ulong[ConeComparisonWordCount];
        Span<ulong> delta = stackalloc ulong[ConeComparisonWordCount];
        Span<ulong> rational = stackalloc ulong[ConeComparisonWordCount];
        Span<ulong> radius = stackalloc ulong[ConeComparisonWordCount];
        Span<ulong> denominator = stackalloc ulong[ConeComparisonWordCount];
        Span<ulong> planar = stackalloc ulong[ConeComparisonWordCount];
        Span<ulong> side = stackalloc ulong[ConeComparisonWordCount];
        first.Clear();
        second.Clear();
        third.Clear();
        result.Clear();
        other.Clear();
        coefficient.Clear();
        delta.Clear();
        rational.Clear();
        radius.Clear();
        denominator.Clear();
        planar.Clear();
        side.Clear();
        WideArithmetic.GetMagnitude(radicalCoefficient, coefficient);
        WideArithmetic.GetMagnitude(radicand, delta);
        WideArithmetic.GetMagnitude(rationalMagnitude, rational);
        WideArithmetic.GetMagnitude(
            radiusNumerator,
            out radius[2],
            out radius[1],
            out radius[0]);
        WideArithmetic.GetMagnitude(
            coordinateDenominator,
            out denominator[2],
            out denominator[1],
            out denominator[0]);
        WideArithmetic.GetMagnitude(
            planarAxisSquared,
            out planar[4],
            out planar[3],
            out planar[2],
            out planar[1],
            out planar[0]);
        WideArithmetic.GetMagnitude(
            sideSquared,
            out side[4],
            out side[3],
            out side[2],
            out side[1],
            out side[0]);

        WideArithmetic.MultiplyMagnitudes(
            coefficient,
            coefficient,
            first);
        WideArithmetic.MultiplyMagnitudes(first, delta, second);
        WideArithmetic.MultiplyMagnitudes(rational, rational, first);
        WideArithmetic.AddEqualMagnitudes(second, first, result);
        if (radiusDenominator == 2)
        {
            result.CopyTo(first);
            WideArithmetic.AddEqualMagnitudes(first, first, second);
            WideArithmetic.AddEqualMagnitudes(second, second, result);
        }

        WideArithmetic.MultiplyMagnitudes(radius, radius, first);
        WideArithmetic.MultiplyMagnitudes(
            denominator,
            denominator,
            second);
        WideArithmetic.MultiplyMagnitudes(first, second, third);
        WideArithmetic.MultiplyMagnitudes(planar, planar, first);
        WideArithmetic.MultiplyMagnitudes(third, first, second);
        WideArithmetic.MultiplyMagnitudes(second, side, other);

        int rationalComparison =
            WideArithmetic.CompareMagnitudeEqualLength(result, other);
        if (rationalComparison <= 0)
            return -1;
        WideArithmetic.SubtractEqualMagnitudes(
            result,
            other,
            rational);

        WideArithmetic.GetMagnitude(rationalMagnitude, result);
        WideArithmetic.MultiplyMagnitudes(coefficient, result, first);
        first.CopyTo(second);
        WideArithmetic.AddEqualMagnitudes(second, second, first);
        if (radiusDenominator == 2)
        {
            first.CopyTo(second);
            WideArithmetic.AddEqualMagnitudes(second, second, first);
            first.CopyTo(second);
            WideArithmetic.AddEqualMagnitudes(second, second, first);
        }
        WideArithmetic.MultiplyMagnitudes(first, first, second);
        WideArithmetic.MultiplyMagnitudes(second, delta, other);
        WideArithmetic.MultiplyMagnitudes(rational, rational, result);
        return WideArithmetic.CompareMagnitudeEqualLength(result, other);
    }

    private static int CompareRationalDistanceToRadius(
        RationalDistance distance,
        Signed192 radiusNumerator,
        int radiusDenominator) =>
        radiusDenominator == 1
            ? CompareDistanceToRaw(distance, radiusNumerator)
            : CompareDistanceToTwiceRaw(distance, radiusNumerator);

}
