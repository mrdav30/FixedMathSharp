//=======================================================================
// WideFiniteAxisIntersection.CenteredConeSurfaceAnchors.cs
//=======================================================================
// MIT License, Copyright (c) 2024–present David Oravsky (mrdav30)
// See LICENSE file in the project root for full license information.
//=======================================================================

namespace FixedMathSharp.Bounds;

/// <content>
/// Provides helpers for computing the closest surface anchor, outward normal,
/// and signed distance from a point to a centered finite cone's lateral surface.
/// </content>
internal static partial class WideFiniteAxisIntersection
{
    internal static bool TryGetClosestCenteredFiniteConeSurfaceAnchor(
        Vector3d point,
        Vector3d center,
        FixedQuaternion frameRotation,
        Vector3d localAxisDirection,
        Fixed64 height,
        Fixed64 radius,
        Vector3d localFallbackRadialDirection,
        out FixedPointAnchor surfaceAnchor,
        out Vector3d outwardNormal,
        out Fixed64 signedDistance)
    {
        GetRigidLocalPointRelation(
            point,
            center,
            frameRotation,
            localAxisDirection,
            out RigidLocalPointRelation relation);
        Vector3d localRadialDirection = GetLocalRadialDirection(
            relation,
            localAxisDirection,
            localFallbackRadialDirection);
        GetRigidCenteredConeMeridian(
            relation,
            height,
            radius,
            out Fixed64 surfaceRadius,
            out Fixed64 signedAxisLength,
            out bool side,
            out bool contained);

        surfaceAnchor = CreateCenteredSurfaceAnchor(
            center,
            frameRotation,
            localAxisDirection,
            signedAxisLength,
            localRadialDirection,
            surfaceRadius);
        var reference = new FixedPointAnchor(
            point,
            FixedQuaternion.Identity,
            Vector3d.Zero);
        bool representable = reference.TryGetOffsetFrom(
            surfaceAnchor,
            out Vector3d surfaceToPoint);
        representable &= Vector3d.TryGetMagnitude(
            surfaceToPoint,
            out Fixed64 distance);
        if (!representable)
        {
            surfaceAnchor = default;
            outwardNormal = default;
            signedDistance = default;
            return false;
        }

        signedDistance = contained ? -distance : distance;
        Vector3d localNormal = side
            ? GetCenteredConeSideNormal(
                localRadialDirection,
                localAxisDirection,
                height,
                radius)
            : -localAxisDirection;
        _ = frameRotation.TryRotate(
            localNormal,
            out Vector3d rotatedNormal);
        outwardNormal = WideGeometry.GetNormalized(rotatedNormal);
        return true;
    }

    private static void GetRigidCenteredConeMeridian(
        RigidLocalPointRelation relation,
        Fixed64 height,
        Fixed64 radius,
        out Fixed64 surfaceRadius,
        out Fixed64 signedAxisLength,
        out bool side,
        out bool contained)
    {
        GetCenteredConeHeightRelation(
            relation,
            height,
            out Signed576 heightFromBaseNumerator,
            out Signed320 heightFromBaseDenominator);
        Signed192 radiusRaw = Signed192.Signed(
            radius.m_rawValue);
        int radialToRadius = WideArithmetic
            .CompareNonNegativeRadicalToRatio(
                relation.RadialNumerator,
                relation.RadialDenominator,
                Signed320.ExtendValue(radiusRaw),
                Scale320);
        side = IsCenteredConeSideNearest(
            relation,
            height,
            radius,
            heightFromBaseNumerator,
            heightFromBaseDenominator);
        if (side)
        {
            GetClosestCenteredConeSideCoordinates(
                relation,
                height,
                radius,
                heightFromBaseNumerator,
                heightFromBaseDenominator,
                out surfaceRadius,
                out signedAxisLength);
        }
        else
        {
            // A base selection proves the radial projection lies inside the
            // disk; otherwise the shared rim is also the nearest side feature.
            _ = TryGetCenteredRadialDistance(
                relation,
                out surfaceRadius);
            signedAxisLength = -height;
        }
        contained = IsPointContainedInRigidCenteredCone(
            relation,
            height,
            radius);
    }

    private static void GetClosestCenteredConeSideCoordinates(
        RigidLocalPointRelation relation,
        Fixed64 height,
        Fixed64 radius,
        Signed576 heightFromBaseNumerator,
        Signed320 heightFromBaseDenominator,
        out Fixed64 surfaceRadius,
        out Fixed64 signedAxisLength)
    {
        Signed192 heightRaw = Signed192.Signed(
            height.m_rawValue);
        Signed192 radiusRaw = Signed192.Signed(
            radius.m_rawValue);
        Signed192 sideSquared = WideArithmetic.AddSigned192(
            GetSquaredRadius(heightRaw),
            GetSquaredRadius(radiusRaw));
        Signed576 parameterNumerator =
            GetCenteredConeSideParameterNumerator(
                heightFromBaseNumerator,
                heightFromBaseDenominator,
                heightRaw,
                radiusRaw,
                GetSquaredRadius(radiusRaw));
        int startComparison = CompareCenteredConeSideParameterToEndpoint(
            relation,
            parameterNumerator,
            heightFromBaseDenominator,
            radiusRaw,
            sideSquared,
            endpoint: 0);
        if (startComparison <= 0)
        {
            surfaceRadius = radius;
            signedAxisLength = -height;
            return;
        }

        int endComparison = CompareCenteredConeSideParameterToEndpoint(
            relation,
            parameterNumerator,
            heightFromBaseDenominator,
            radiusRaw,
            sideSquared,
            endpoint: 1);
        if (endComparison >= 0)
        {
            surfaceRadius = Fixed64.Zero;
            signedAxisLength = height;
            return;
        }

        Signed576 denominator = WideArithmetic.MultiplySigned320(
            heightFromBaseDenominator,
            Signed320.ExtendValue(sideSquared));
        Signed576 radicalCoefficient = WideArithmetic.MultiplySigned320(
            heightFromBaseDenominator,
            Signed320.ExtendValue(
                WideArithmetic.SubtractSigned192(default, radiusRaw)));
        GetCenteredConeSideApproximation(
            relation,
            height,
            radius,
            heightFromBaseNumerator,
            heightFromBaseDenominator,
            out Fixed64 approximateRadius,
            out Fixed64 approximateAxisLength);

        surfaceRadius = RoundBoundedCenteredConeCoordinate(
            WideArithmetic.MultiplySigned576(
                WideArithmetic.SubtractSigned576(
                    denominator,
                    parameterNumerator),
                radiusRaw),
            WideArithmetic.MultiplySigned576(
                radicalCoefficient,
                WideArithmetic.SubtractSigned192(default, radiusRaw)),
            relation,
            denominator,
            Fixed64.Zero,
            radius,
            approximateRadius);

        Signed576 heightAndParameter = WideArithmetic.MultiplySigned576(
            parameterNumerator,
            heightRaw);
        Signed576 heightAndDenominator = WideArithmetic.MultiplySigned576(
            denominator,
            heightRaw);
        Signed576 heightAndRadical = WideArithmetic.MultiplySigned576(
            radicalCoefficient,
            heightRaw);
        signedAxisLength = RoundBoundedCenteredConeCoordinate(
            WideArithmetic.SubtractSigned576(
                WideArithmetic.AddSigned576(
                    heightAndParameter,
                    heightAndParameter),
                heightAndDenominator),
            WideArithmetic.AddSigned576(
                heightAndRadical,
                heightAndRadical),
            relation,
            denominator,
            -height,
            height,
            approximateAxisLength);
    }

    private static void GetCenteredConeSideApproximation(
        RigidLocalPointRelation relation,
        Fixed64 height,
        Fixed64 radius,
        Signed576 heightFromBaseNumerator,
        Signed320 heightFromBaseDenominator,
        out Fixed64 surfaceRadius,
        out Fixed64 signedAxisLength)
    {
        bool representable = TryGetCenteredRadialDistance(
            relation,
            out Fixed64 radialDistance);
        representable &= Fixed64.TryGetSignedRawRatio(
            heightFromBaseNumerator,
            Signed576.ExtendValue(
                heightFromBaseDenominator),
            out Fixed64 heightFromBase);
        if (!representable)
        {
            surfaceRadius = radius / Fixed64.Two;
            signedAxisLength = Fixed64.Zero;
            return;
        }

        Vector2d start = new(radius, Fixed64.Zero);
        Vector2d end = new(Fixed64.Zero, height);
        Signed192 denominator = WideGeometry.GetDifferenceDotProduct2D(
            end.X, start.X, end.Y, start.Y,
            end.X, start.X, end.Y, start.Y);
        Signed192 numerator = WideGeometry.GetDifferenceDotProduct2D(
            radialDistance, start.X, heightFromBase, start.Y,
            end.X, start.X, end.Y, start.Y);
        Signed192 remaining = WideArithmetic.SubtractSigned192(
            denominator,
            numerator);
        _ = Fixed64.TryGetSignedRawRatio(
            Signed576.ExtendValue(
                WideArithmetic.MultiplySigned192(
                    Signed192.Signed(radius.m_rawValue),
                    remaining)),
            Signed576.ExtendValue(
                Signed320.ExtendValue(denominator)),
            out surfaceRadius);
        _ = Fixed64.TryGetSignedRawRatio(
            Signed576.ExtendValue(
                WideArithmetic.MultiplySigned192(
                    Signed192.Signed(height.m_rawValue),
                    numerator)),
            Signed576.ExtendValue(
                Signed320.ExtendValue(denominator)),
            out Fixed64 sideHeight);
        _ = Fixed64.TrySubtractProducts(
            Fixed64.Two,
            sideHeight,
            height,
            Fixed64.One,
            out signedAxisLength);
    }

    private static Fixed64 RoundBoundedCenteredConeCoordinate(
        Signed576 rational,
        Signed576 radicalCoefficient,
        RigidLocalPointRelation relation,
        Signed576 denominator,
        Fixed64 minimum,
        Fixed64 maximum,
        Fixed64 approximation)
    {
        long low = minimum.m_rawValue;
        long high = maximum.m_rawValue;
        long candidate = approximation.Clamp(minimum, maximum).m_rawValue;
        int candidateComparison = CompareCenteredConeCoordinateToRaw(
            rational,
            radicalCoefficient,
            relation,
            denominator,
            candidate);
        if (candidateComparison >= 0)
        {
            ulong distanceToHigh = unchecked((ulong)high - (ulong)candidate);
            long nextCandidate = unchecked(
                candidate + (long)((distanceToHigh | (0UL - distanceToHigh)) >> 63));
            if (CompareCenteredConeCoordinateToRaw(
                    rational,
                    radicalCoefficient,
                    relation,
                    denominator,
                    nextCandidate) < 0)
            {
                low = candidate;
            }
            else
            {
                low = nextCandidate;
            }
        }
        else
        {
            high = candidate - 1L;
            if (CompareCenteredConeCoordinateToRaw(
                    rational,
                    radicalCoefficient,
                    relation,
                    denominator,
                    high) >= 0)
            {
                low = high;
            }
        }

        while (low < high)
        {
            ulong span = unchecked((ulong)high - (ulong)low);
            long midpoint = unchecked(
                (long)((ulong)low + ((span + 1UL) >> 1)));
            if (CompareCenteredConeCoordinateToRaw(
                    rational,
                    radicalCoefficient,
                    relation,
                    denominator,
                    midpoint) >= 0)
            {
                low = midpoint;
            }
            else
            {
                high = midpoint - 1L;
            }
        }

        Signed192 doubledMidpoint = WideArithmetic.AddSigned192(
            WideArithmetic.AddSigned192(
                Signed192.Signed(low),
                Signed192.Signed(low)),
            Scale);
        Signed576 midpointRational = WideArithmetic.SubtractSigned576(
            WideArithmetic.AddSigned576(rational, rational),
            WideArithmetic.MultiplySigned576(
                denominator,
                doubledMidpoint));
        int midpointComparison =
            WideArithmetic.CompareSignedLinearRadicalToZero(
                Signed832.ExtendValue(midpointRational),
                Signed704.ExtendValue(
                    WideArithmetic.AddSigned576(
                        radicalCoefficient,
                        radicalCoefficient)),
                relation.RadialNumerator,
                relation.RadialDenominator);
        return Fixed64.FromRaw(
            low + GetHalfToEvenIncrement(midpointComparison, low));
    }

    private static int CompareCenteredConeCoordinateToRaw(
        Signed576 rational,
        Signed576 radicalCoefficient,
        RigidLocalPointRelation relation,
        Signed576 denominator,
        long raw)
    {
        Signed832 candidate = WideArithmetic.MultiplySigned576ToSigned832(
            denominator,
            Signed320.ExtendValue(
                Signed192.Signed(raw)));
        return WideArithmetic.CompareSignedLinearRadicalToZero(
            WideArithmetic.SubtractSigned832(
                Signed832.ExtendValue(rational),
                candidate),
            Signed704.ExtendValue(radicalCoefficient),
            relation.RadialNumerator,
            relation.RadialDenominator);
    }

    private static void GetCenteredConeHeightRelation(
        RigidLocalPointRelation relation,
        Fixed64 height,
        out Signed576 numerator,
        out Signed320 denominator)
    {
        Signed320 denominatorAndAxis = WideArithmetic.MultiplySigned192(
            relation.Denominator,
            relation.AxisSquared);
        Signed576 heightTerm = WideArithmetic.MultiplySigned320(
            denominatorAndAxis,
            Signed320.ExtendValue(
                Signed192.Signed(
                    height.m_rawValue)));
        Signed576 projectionTerm = WideArithmetic.MultiplySigned320(
            relation.Projection,
            Signed320.ExtendValue(
                Signed192.Signed(
                    Fixed64.One.m_rawValue)));
        numerator = WideArithmetic.AddSigned576(
            heightTerm,
            WideArithmetic.AddSigned576(
                projectionTerm,
                projectionTerm));
        denominator = WideArithmetic.AddSigned320(
            denominatorAndAxis,
            denominatorAndAxis);
    }

    private static bool IsPointContainedInRigidCenteredCone(
        RigidLocalPointRelation relation,
        Fixed64 height,
        Fixed64 radius)
    {
        if (GetCenteredCap(relation, height) != 0)
            return false;

        Signed320 denominatorAndAxis = WideArithmetic.MultiplySigned192(
            relation.Denominator,
            relation.AxisSquared);
        Signed576 remaining = WideArithmetic.SubtractSigned576(
            WideArithmetic.MultiplySigned320(
                denominatorAndAxis,
                Signed320.ExtendValue(
                    Signed192.Signed(
                        height.m_rawValue))),
            WideArithmetic.AddSigned576(
                WideArithmetic.MultiplySigned320(
                    relation.Projection,
                    Signed320.ExtendValue(
                        Signed192.Signed(
                            Fixed64.One.m_rawValue))),
                WideArithmetic.MultiplySigned320(
                    relation.Projection,
                    Signed320.ExtendValue(
                        Signed192.Signed(
                            Fixed64.One.m_rawValue)))));
        _ = Signed320.TryNarrowSigned(
            remaining,
            out Signed320 narrowRemaining);
        Signed192 heightSquared = GetSquaredRadius(
            Signed192.Signed(
                height.m_rawValue));
        Signed320 axisAndHeightSquared =
            WideArithmetic.MultiplySigned192(
                relation.AxisSquared,
                heightSquared);
        Signed704 left = WideArithmetic.MultiplySigned576ToSigned704(
            relation.RadialNumerator,
            axisAndHeightSquared);
        left = WideArithmetic.AddSigned704(left, left);
        left = WideArithmetic.AddSigned704(left, left);

        Signed576 remainingSquared = WideArithmetic.MultiplySigned320(
            narrowRemaining,
            narrowRemaining);
        Signed192 radiusSquared = GetSquaredRadius(
            Signed192.Signed(
                radius.m_rawValue));
        Signed704 right = Signed704.ExtendValue(
            WideArithmetic.MultiplySigned576(
                remainingSquared,
                radiusSquared));
        return WideArithmetic.CompareNonNegative(left, right) <= 0;
    }

    private static bool IsCenteredConeSideNearest(
        RigidLocalPointRelation relation,
        Fixed64 height,
        Fixed64 radius,
        Signed576 heightFromBaseNumerator,
        Signed320 heightFromBaseDenominator)
    {
        Signed192 heightRaw = Signed192.Signed(
            height.m_rawValue);
        Signed192 radiusRaw = Signed192.Signed(
            radius.m_rawValue);
        Signed192 heightSquared = GetSquaredRadius(heightRaw);
        Signed192 radiusSquared = GetSquaredRadius(radiusRaw);
        Signed192 sideSquared = WideArithmetic.AddSigned192(
            heightSquared,
            radiusSquared);
        Signed576 sideParameterNumerator =
            GetCenteredConeSideParameterNumerator(
                heightFromBaseNumerator,
                heightFromBaseDenominator,
                heightRaw,
                radiusRaw,
                radiusSquared);
        int startComparison = CompareCenteredConeSideParameterToEndpoint(
            relation,
            sideParameterNumerator,
            heightFromBaseDenominator,
            radiusRaw,
            sideSquared,
            endpoint: 0);
        int radialToRadius = WideArithmetic
            .CompareNonNegativeRadicalToRatio(
                relation.RadialNumerator,
                relation.RadialDenominator,
                Signed320.ExtendValue(radiusRaw),
                Scale320);
        if (startComparison <= 0)
            return radialToRadius >= 0;

        int endComparison = CompareCenteredConeSideParameterToEndpoint(
            relation,
            sideParameterNumerator,
            heightFromBaseDenominator,
            radiusRaw,
            sideSquared,
            endpoint: 1);
        bool baseInterior = radialToRadius <= 0;
        if (endComparison >= 0)
        {
            return CompareCenteredConeApexToBaseDistance(
                relation,
                heightFromBaseNumerator,
                heightFromBaseDenominator,
                heightRaw,
                radiusRaw,
                heightSquared,
                radiusSquared,
                baseInterior) <= 0;
        }

        return CompareCenteredConeInteriorSideToBaseDistance(
            relation,
            heightFromBaseNumerator,
            heightFromBaseDenominator,
            heightRaw,
            radiusRaw,
            heightSquared,
            radiusSquared,
            sideSquared,
            baseInterior) <= 0;
    }

    private static Signed576 GetCenteredConeSideParameterNumerator(
        Signed576 heightFromBaseNumerator,
        Signed320 heightFromBaseDenominator,
        Signed192 height,
        Signed192 radius,
        Signed192 radiusSquared) =>
        WideArithmetic.AddSigned576(
            WideArithmetic.MultiplySigned320(
                heightFromBaseDenominator,
                Signed320.ExtendValue(radiusSquared)),
            WideArithmetic.MultiplySigned576(
                heightFromBaseNumerator,
                height));

    private static int CompareCenteredConeSideParameterToEndpoint(
        RigidLocalPointRelation relation,
        Signed576 sideParameterNumerator,
        Signed320 heightFromBaseDenominator,
        Signed192 radius,
        Signed192 sideSquared,
        int endpoint)
    {
        Signed576 rational = sideParameterNumerator;
        if (endpoint != 0)
        {
            rational = WideArithmetic.SubtractSigned576(
                rational,
                WideArithmetic.MultiplySigned320(
                    heightFromBaseDenominator,
                    Signed320.ExtendValue(
                        sideSquared)));
        }
        Signed576 radialCoefficient = WideArithmetic.MultiplySigned320(
            heightFromBaseDenominator,
            Signed320.ExtendValue(
                WideArithmetic.SubtractSigned192(
                    default,
                    radius)));
        return WideArithmetic.CompareSignedLinearRadicalToZero(
            Signed832.ExtendValue(rational),
            Signed704.ExtendValue(radialCoefficient),
            relation.RadialNumerator,
            relation.RadialDenominator);
    }

    private static int CompareCenteredConeApexToBaseDistance(
        RigidLocalPointRelation relation,
        Signed576 heightFromBaseNumerator,
        Signed320 heightFromBaseDenominator,
        Signed192 height,
        Signed192 radius,
        Signed192 heightSquared,
        Signed192 radiusSquared,
        bool baseInterior)
    {
        Signed576 twiceHeightAndY = WideArithmetic.AddSigned576(
            WideArithmetic.MultiplySigned576(
                heightFromBaseNumerator,
                height),
            WideArithmetic.MultiplySigned576(
                heightFromBaseNumerator,
                height));
        Signed576 rational = WideArithmetic.SubtractSigned576(
            WideArithmetic.MultiplySigned320(
                heightFromBaseDenominator,
                Signed320.ExtendValue(
                    baseInterior
                        ? heightSquared
                        : WideArithmetic.SubtractSigned192(
                            heightSquared,
                            radiusSquared))),
            twiceHeightAndY);
        if (baseInterior)
        {
            Signed704 result = WideArithmetic.AddSigned704(
                WideArithmetic.MultiplySigned576ToSigned704(
                    relation.RadialNumerator,
                    heightFromBaseDenominator),
                WideArithmetic.MultiplySigned576ToSigned704(
                    rational,
                    relation.RadialDenominator));
            return result.Sign;
        }

        Signed576 radialCoefficient = WideArithmetic.MultiplySigned320(
            heightFromBaseDenominator,
            Signed320.ExtendValue(
                WideArithmetic.AddSigned192(
                    radius,
                    radius)));
        return WideArithmetic.CompareSignedLinearRadicalToZero(
            Signed832.ExtendValue(rational),
            Signed704.ExtendValue(radialCoefficient),
            relation.RadialNumerator,
            relation.RadialDenominator);
    }

    private static int CompareCenteredConeInteriorSideToBaseDistance(
        RigidLocalPointRelation relation,
        Signed576 heightFromBaseNumerator,
        Signed320 heightFromBaseDenominator,
        Signed192 height,
        Signed192 radius,
        Signed192 heightSquared,
        Signed192 radiusSquared,
        Signed192 sideSquared,
        bool baseInterior)
    {
        Signed576 denominatorSquared = WideArithmetic.MultiplySigned320(
            heightFromBaseDenominator,
            heightFromBaseDenominator);
        _ = Signed320.TryNarrowSigned(
            denominatorSquared,
            out Signed320 narrowDenominatorSquared);
        Signed576 radialAndHeightSquared =
            WideArithmetic.MultiplySigned576(
                relation.RadialNumerator,
                heightSquared);
        Signed832 rational = WideArithmetic.MultiplySigned576ToSigned832(
            radialAndHeightSquared,
            denominatorSquared);

        Signed576 heightRadius = Signed576.ExtendValue(
            WideArithmetic.MultiplySigned192(
                height,
                radius));
        // The normalized rigid-frame and local-axis denominators are both
        // Q32.32 squared magnitudes, so their product remains Signed192-wide.
        _ = Signed192.TryNarrowSigned(
            Signed576.ExtendValue(
                heightFromBaseDenominator),
            out Signed192 narrowHeightDenominator);
        Signed576 meridianOffset = WideArithmetic.SubtractSigned576(
            WideArithmetic.MultiplySigned576(
                heightFromBaseNumerator,
                radius),
            WideArithmetic.MultiplySigned576(
                heightRadius,
                narrowHeightDenominator));
        _ = Signed320.TryNarrowSigned(
            meridianOffset,
            out Signed320 narrowMeridianOffset);
        Signed576 meridianOffsetSquared =
            WideArithmetic.MultiplySigned320(
                narrowMeridianOffset,
                narrowMeridianOffset);
        rational = WideArithmetic.AddSigned832(
            rational,
            Signed832.ExtendValue(
                WideArithmetic.MultiplySigned576ToSigned704(
                    meridianOffsetSquared,
                    relation.RadialDenominator)));

        Signed576 heightNumeratorSquared =
            WideArithmetic.MultiplySigned320(
                Signed320.NarrowValue(heightFromBaseNumerator),
                Signed320.NarrowValue(heightFromBaseNumerator));
        Signed576 denominatorAndSide = WideArithmetic.MultiplySigned320(
            relation.RadialDenominator,
            Signed320.ExtendValue(sideSquared));
        rational = WideArithmetic.SubtractSigned832(
            rational,
            WideArithmetic.MultiplySigned576ToSigned832(
                heightNumeratorSquared,
                denominatorAndSide));

        if (!baseInterior)
        {
            Signed576 radialAndSideSquared =
                WideArithmetic.MultiplySigned576(
                    relation.RadialNumerator,
                    sideSquared);
            rational = WideArithmetic.SubtractSigned832(
                rational,
                WideArithmetic.MultiplySigned576ToSigned832(
                    radialAndSideSquared,
                    denominatorSquared));
            Signed320 radiusAndSideSquared =
                WideArithmetic.MultiplySigned192(
                    radiusSquared,
                    sideSquared);
            rational = WideArithmetic.SubtractSigned832(
                rational,
                Signed832.ExtendValue(
                    WideArithmetic.MultiplySigned320(
                        radiusAndSideSquared,
                        narrowDenominatorSquared,
                        relation.RadialDenominator)));
        }

        Signed576 heightAndDenominator =
            WideArithmetic.MultiplySigned320(
                heightFromBaseDenominator,
                Signed320.ExtendValue(height));
        _ = Signed320.TryNarrowSigned(
            heightAndDenominator,
            out Signed320 narrowHeightAndDenominator);
        Signed704 radicalCoefficient =
            WideArithmetic.MultiplySigned320(
                narrowHeightAndDenominator,
                narrowMeridianOffset,
                relation.RadialDenominator);
        radicalCoefficient = WideArithmetic.AddSigned704(
            radicalCoefficient,
            radicalCoefficient);
        if (!baseInterior)
        {
            Signed320 radiusAndSide = WideArithmetic.MultiplySigned192(
                radius,
                sideSquared);
            Signed704 baseCoefficient = WideArithmetic.MultiplySigned320(
                radiusAndSide,
                narrowDenominatorSquared,
                relation.RadialDenominator);
            baseCoefficient = WideArithmetic.AddSigned704(
                baseCoefficient,
                baseCoefficient);
            radicalCoefficient = WideArithmetic.AddSigned704(
                radicalCoefficient,
                baseCoefficient);
        }

        return WideArithmetic.CompareSignedLinearRadicalToZero(
            rational,
            radicalCoefficient,
            relation.RadialNumerator,
            relation.RadialDenominator);
    }

}
