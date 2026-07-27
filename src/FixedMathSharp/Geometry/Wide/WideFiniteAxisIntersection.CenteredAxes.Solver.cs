//=======================================================================
// WideFiniteAxisIntersection.CenteredAxes.Solver.cs
//=======================================================================
// MIT License, Copyright (c) 2024–present David Oravsky (mrdav30)
// See LICENSE file in the project root for full license information.
//=======================================================================

namespace FixedMathSharp.Bounds;

/// <content>
/// Solver logic for finding the closest points between two finite axes
/// defined relative to their centers, using high-precision wide arithmetic.
/// </content>
internal static partial class WideFiniteAxisIntersection
{
    private static void GetClosestCenteredAxisParameters(
        Signed192 firstLengthSquared,
        Signed192 directionsDot,
        Signed192 secondLengthSquared,
        Signed192 firstDirectionDotDifference,
        Signed192 secondDirectionDotDifference,
        Fixed64 firstLength,
        Fixed64 secondLength,
        out Signed320 firstNumerator,
        out Signed320 secondNumerator,
        out Signed320 denominator)
    {
        Signed320 determinant = WideArithmetic.MultiplySubtract(
            firstLengthSquared,
            secondLengthSquared,
            directionsDot,
            directionsDot);
        if (determinant.Sign > 0)
        {
            firstNumerator = WideArithmetic.MultiplySubtract(
                directionsDot,
                secondDirectionDotDifference,
                secondLengthSquared,
                firstDirectionDotDifference);
            secondNumerator = WideArithmetic.MultiplySubtract(
                firstLengthSquared,
                secondDirectionDotDifference,
                directionsDot,
                firstDirectionDotDifference);
            denominator = determinant;
            if (!ClampCenteredAxisParameter(
                    ref firstNumerator,
                    ref denominator,
                    firstLength))
            {
                if (!ClampCenteredAxisParameter(
                        ref secondNumerator,
                        ref denominator,
                        secondLength))
                {
                    return;
                }

                ProjectFirstFromCenteredAxisCap(
                    secondNumerator,
                    firstLengthSquared,
                    directionsDot,
                    firstDirectionDotDifference,
                    out firstNumerator,
                    out denominator);
                if (!ClampCenteredAxisParameter(
                        ref firstNumerator,
                        ref denominator,
                        firstLength))
                {
                    secondNumerator = WideArithmetic.MultiplySigned192(
                        new Signed192(
                            secondNumerator.Word2,
                            secondNumerator.Word1,
                            secondNumerator.Word0),
                        firstLengthSquared);
                    return;
                }

                SetCenteredAxisCap(ref secondNumerator, ref denominator, secondLength);
                return;
            }
        }
        else
        {
            firstNumerator = Signed320.ExtendValue(
                Signed192.Signed(-firstLength.m_rawValue));
            denominator = Signed320.ExtendValue(DoubleParameterScale);
        }

        ProjectSecondFromCenteredAxisCap(
            firstNumerator,
            directionsDot,
            secondLengthSquared,
            secondDirectionDotDifference,
            out secondNumerator,
            out denominator);
        if (!ClampCenteredAxisParameter(
                ref secondNumerator,
                ref denominator,
                secondLength))
        {
            firstNumerator = WideArithmetic.MultiplySigned192(
                new Signed192(
                    firstNumerator.Word2,
                    firstNumerator.Word1,
                    firstNumerator.Word0),
                secondLengthSquared);
            return;
        }

        ProjectFirstFromCenteredAxisCap(
            secondNumerator,
            firstLengthSquared,
            directionsDot,
            firstDirectionDotDifference,
            out firstNumerator,
            out denominator);
        if (!ClampCenteredAxisParameter(
                ref firstNumerator,
                ref denominator,
                firstLength))
        {
            secondNumerator = WideArithmetic.MultiplySigned192(
                new Signed192(
                    secondNumerator.Word2,
                    secondNumerator.Word1,
                    secondNumerator.Word0),
                firstLengthSquared);
            return;
        }

        SetCenteredAxisCap(ref secondNumerator, ref denominator, secondLength);
    }

    private static void GetCenteredAxesSquaredDistance(
        Vector2d firstCenter,
        Vector2d firstAxis,
        Fixed64 firstLength,
        Vector2d secondCenter,
        Vector2d secondAxis,
        Fixed64 secondLength,
        out Signed832 squaredNumerator,
        out Signed320 denominator)
    {
        GetCenteredAxesCandidate(
            firstCenter,
            firstAxis,
            firstLength,
            secondCenter,
            secondAxis,
            secondLength,
            out CenteredAxesCandidate2d candidate);
        squaredNumerator = candidate.SquaredDistanceNumerator;
        denominator = candidate.ParameterDenominator;
    }

    private static void GetCenteredAxesCandidate(
        Vector2d firstCenter,
        Vector2d firstAxis,
        Fixed64 firstLength,
        Vector2d secondCenter,
        Vector2d secondAxis,
        Fixed64 secondLength,
        out CenteredAxesCandidate2d candidate)
    {
        GetClosestCenteredAxisParameters(
            GetDot(firstAxis, Vector2d.Zero, firstAxis, Vector2d.Zero),
            GetDot(firstAxis, Vector2d.Zero, secondAxis, Vector2d.Zero),
            GetDot(secondAxis, Vector2d.Zero, secondAxis, Vector2d.Zero),
            GetDot(firstCenter, secondCenter, firstAxis, Vector2d.Zero),
            GetDot(firstCenter, secondCenter, secondAxis, Vector2d.Zero),
            firstLength,
            secondLength,
            out Signed320 firstNumerator,
            out Signed320 secondNumerator,
            out Signed320 denominator);
        Signed576 firstX = GetCenteredAxisPointNumerator(
            firstCenter.X,
            firstAxis.X,
            firstNumerator,
            denominator);
        Signed576 firstY = GetCenteredAxisPointNumerator(
            firstCenter.Y,
            firstAxis.Y,
            firstNumerator,
            denominator);
        Signed576 secondX = GetCenteredAxisPointNumerator(
            secondCenter.X,
            secondAxis.X,
            secondNumerator,
            denominator);
        Signed576 secondY = GetCenteredAxisPointNumerator(
            secondCenter.Y,
            secondAxis.Y,
            secondNumerator,
            denominator);
        Signed576 differenceX = WideArithmetic.SubtractSigned576(firstX, secondX);
        Signed576 differenceY = WideArithmetic.SubtractSigned576(firstY, secondY);
        candidate = new CenteredAxesCandidate2d(
            firstX,
            firstY,
            secondX,
            secondY,
            denominator,
            WideArithmetic.AddSigned832(
                WideArithmetic.MultiplySigned576ToSigned832(differenceX, differenceX),
                WideArithmetic.MultiplySigned576ToSigned832(differenceY, differenceY)));
    }

    private static void GetCenteredAxesSquaredDistance(
        Vector3d firstCenter,
        Vector3d firstAxis,
        Fixed64 firstLength,
        Vector3d secondCenter,
        Vector3d secondAxis,
        Fixed64 secondLength,
        out Signed832 squaredNumerator,
        out Signed320 denominator)
    {
        GetCenteredAxesCandidate(
            firstCenter,
            firstAxis,
            firstLength,
            secondCenter,
            secondAxis,
            secondLength,
            out CenteredAxesCandidate candidate);
        squaredNumerator = candidate.SquaredDistanceNumerator;
        denominator = candidate.ParameterDenominator;
    }

    private static void GetCenteredAxesCandidate(
        Vector3d firstCenter,
        Vector3d firstAxis,
        Fixed64 firstLength,
        Vector3d secondCenter,
        Vector3d secondAxis,
        Fixed64 secondLength,
        out CenteredAxesCandidate candidate)
    {
        GetClosestCenteredAxisParameters(
            GetDot(firstAxis, Vector3d.Zero, firstAxis, Vector3d.Zero),
            GetDot(firstAxis, Vector3d.Zero, secondAxis, Vector3d.Zero),
            GetDot(secondAxis, Vector3d.Zero, secondAxis, Vector3d.Zero),
            GetDot(firstCenter, secondCenter, firstAxis, Vector3d.Zero),
            GetDot(firstCenter, secondCenter, secondAxis, Vector3d.Zero),
            firstLength,
            secondLength,
            out Signed320 firstNumerator,
            out Signed320 secondNumerator,
            out Signed320 denominator);
        Signed576 firstX = GetCenteredAxisPointNumerator(
            firstCenter.X,
            firstAxis.X,
            firstNumerator,
            denominator);
        Signed576 firstY = GetCenteredAxisPointNumerator(
            firstCenter.Y,
            firstAxis.Y,
            firstNumerator,
            denominator);
        Signed576 firstZ = GetCenteredAxisPointNumerator(
            firstCenter.Z,
            firstAxis.Z,
            firstNumerator,
            denominator);
        Signed576 secondX = GetCenteredAxisPointNumerator(
            secondCenter.X,
            secondAxis.X,
            secondNumerator,
            denominator);
        Signed576 secondY = GetCenteredAxisPointNumerator(
            secondCenter.Y,
            secondAxis.Y,
            secondNumerator,
            denominator);
        Signed576 secondZ = GetCenteredAxisPointNumerator(
            secondCenter.Z,
            secondAxis.Z,
            secondNumerator,
            denominator);
        Signed576 differenceX = WideArithmetic.SubtractSigned576(firstX, secondX);
        Signed576 differenceY = WideArithmetic.SubtractSigned576(firstY, secondY);
        Signed576 differenceZ = WideArithmetic.SubtractSigned576(firstZ, secondZ);
        Signed832 squaredDistance = WideArithmetic.AddSigned832(
            WideArithmetic.AddSigned832(
                WideArithmetic.MultiplySigned576ToSigned832(differenceX, differenceX),
                WideArithmetic.MultiplySigned576ToSigned832(differenceY, differenceY)),
            WideArithmetic.MultiplySigned576ToSigned832(differenceZ, differenceZ));
        candidate = new CenteredAxesCandidate(
            firstX,
            firstY,
            firstZ,
            secondX,
            secondY,
            secondZ,
            firstNumerator,
            secondNumerator,
            denominator,
            squaredDistance);
    }

    private static bool IsWithinCombinedRadius(
        Signed832 squaredDistanceNumerator,
        Signed320 distanceDenominator,
        Fixed64 firstRadius,
        Fixed64 secondRadius)
    {
        Signed192 combinedRadius = WideArithmetic.AddSigned192(
            Signed192.Signed(firstRadius.m_rawValue),
            Signed192.Signed(secondRadius.m_rawValue));
        Signed576 radiusNumerator = WideArithmetic.MultiplySigned320(
            distanceDenominator,
            Signed320.ExtendValue(combinedRadius));
        return WideArithmetic.SubtractSigned832(
            squaredDistanceNumerator,
            WideArithmetic.MultiplySigned576ToSigned832(
                radiusNumerator,
                radiusNumerator)).Sign <= 0;
    }

    private static void ProjectSecondFromCenteredAxisCap(
        Signed320 firstCapNumerator,
        Signed192 directionsDot,
        Signed192 secondLengthSquared,
        Signed192 secondDirectionDotDifference,
        out Signed320 secondNumerator,
        out Signed320 denominator)
    {
        Signed192 capNumerator = new(
            firstCapNumerator.Word2,
            firstCapNumerator.Word1,
            firstCapNumerator.Word0);
        secondNumerator = WideArithmetic.AddSigned320(
            WideArithmetic.MultiplySigned192(
                DoubleParameterScale,
                secondDirectionDotDifference),
            WideArithmetic.MultiplySigned192(capNumerator, directionsDot));
        denominator = WideArithmetic.MultiplySigned192(
            DoubleParameterScale,
            secondLengthSquared);
    }

    private static void ProjectFirstFromCenteredAxisCap(
        Signed320 secondCapNumerator,
        Signed192 firstLengthSquared,
        Signed192 directionsDot,
        Signed192 firstDirectionDotDifference,
        out Signed320 firstNumerator,
        out Signed320 denominator)
    {
        Signed192 capNumerator = new(
            secondCapNumerator.Word2,
            secondCapNumerator.Word1,
            secondCapNumerator.Word0);
        firstNumerator = WideArithmetic.SubtractSigned320(
            WideArithmetic.MultiplySigned192(capNumerator, directionsDot),
            WideArithmetic.MultiplySigned192(
                DoubleParameterScale,
                firstDirectionDotDifference));
        denominator = WideArithmetic.MultiplySigned192(
            DoubleParameterScale,
            firstLengthSquared);
    }

    private static bool ClampCenteredAxisParameter(
        ref Signed320 numerator,
        ref Signed320 denominator,
        Fixed64 axisLength)
    {
        Signed576 scaledNumerator = WideArithmetic.MultiplySigned320(
            numerator,
            Signed320.ExtendValue(DoubleParameterScale));
        Signed576 limit = WideArithmetic.MultiplySigned320(
            denominator,
            Signed320.ExtendValue(
                Signed192.Signed(axisLength.m_rawValue)));
        if (WideArithmetic.SubtractSigned576(
                scaledNumerator,
                WideArithmetic.SubtractSigned576(default, limit)).Sign < 0)
        {
            numerator = Signed320.ExtendValue(
                Signed192.Signed(-axisLength.m_rawValue));
            denominator = Signed320.ExtendValue(DoubleParameterScale);
            return true;
        }
        if (WideArithmetic.SubtractSigned576(scaledNumerator, limit).Sign > 0)
        {
            numerator = Signed320.ExtendValue(
                Signed192.Signed(axisLength.m_rawValue));
            denominator = Signed320.ExtendValue(DoubleParameterScale);
            return true;
        }

        return false;
    }

    private static void SetCenteredAxisCap(
        ref Signed320 numerator,
        ref Signed320 denominator,
        Fixed64 axisLength)
    {
        bool positive = numerator.Sign > 0;
        numerator = Signed320.ExtendValue(
            Signed192.Signed(
                positive ? axisLength.m_rawValue : -axisLength.m_rawValue));
        denominator = Signed320.ExtendValue(DoubleParameterScale);
    }

    private static bool TryRoundCenteredAxesDistance(
        Signed832 squaredNumerator,
        Signed320 denominator,
        out Fixed64 distance)
    {
        Signed576 root = WideArithmetic.GetFloorSquareRootOfProduct(
            squaredNumerator,
            Scale);
        Signed576 wideDenominator = Signed576.ExtendValue(denominator);
        distance = Fixed64.GetNonNegativeRawRatioFloor(
            Signed704.ExtendValue(root),
            Signed704.ExtendValue(wideDenominator));

        Signed192 doubledMidpoint = new(
            0UL,
            unchecked((ulong)distance.m_rawValue) >> 63,
            (unchecked((ulong)distance.m_rawValue) << 1) | 1UL);
        Signed576 midpointNumerator = WideArithmetic.MultiplySigned320(
            denominator,
            Signed320.ExtendValue(doubledMidpoint));
        Signed832 fourSquaredNumerator = WideArithmetic.AddSigned832(
            WideArithmetic.AddSigned832(squaredNumerator, squaredNumerator),
            WideArithmetic.AddSigned832(squaredNumerator, squaredNumerator));
        int midpointComparison = WideArithmetic.SubtractSigned832(
            fourSquaredNumerator,
            WideArithmetic.MultiplySigned576ToSigned832(
                midpointNumerator,
                midpointNumerator)).Sign;
        long increment =
            GetHalfToEvenIncrement(midpointComparison, distance.m_rawValue);
        if (distance == Fixed64.MaxValue && increment != 0L)
            return false;

        distance = Fixed64.FromRaw(distance.m_rawValue + increment);
        return true;
    }

    private static long GetHalfToEvenIncrement(
        int midpointComparison,
        long lowerRaw)
    {
        int negative = (int)((uint)midpointComparison >> 31);
        int nonzero = (int)(
            (uint)(midpointComparison | -midpointComparison) >> 31);
        int positive = nonzero & (negative ^ 1);
        int zero = nonzero ^ 1;
        return (long)positive | ((long)zero & (lowerRaw & 1L));
    }

    private static bool TryGetCenteredAxisPointCoordinate(
        Fixed64 center,
        Fixed64 axis,
        Signed320 parameterNumerator,
        Signed320 parameterDenominator,
        out Fixed64 coordinate)
    {
        Signed576 numerator = GetCenteredAxisPointNumerator(
            center,
            axis,
            parameterNumerator,
            parameterDenominator);
        return Fixed64.TryGetSignedRawRatio(
            numerator,
            Signed576.ExtendValue(parameterDenominator),
            out coordinate);
    }

    private static Signed576 GetCenteredAxisPointNumerator(
        Fixed64 center,
        Fixed64 axis,
        Signed320 parameterNumerator,
        Signed320 parameterDenominator) =>
        WideArithmetic.AddSigned576(
            WideArithmetic.MultiplySigned320(
                parameterDenominator,
                Signed320.ExtendValue(
                    Signed192.Signed(center.m_rawValue))),
            WideArithmetic.MultiplySigned320(
                parameterNumerator,
                Signed320.ExtendValue(
                    Signed192.Signed(axis.m_rawValue))));

    private static Vector2d GetCenteredAxisLocalOffset(
        Signed576 axisPointX,
        Signed576 axisPointY,
        Vector2d center,
        Signed320 axisPointDenominator,
        Fixed64 frameRotation)
    {
        Signed576 centerX = WideArithmetic.MultiplySigned320(
            axisPointDenominator,
            Signed320.ExtendValue(
                Signed192.Signed(center.X.m_rawValue)));
        Signed576 centerY = WideArithmetic.MultiplySigned320(
            axisPointDenominator,
            Signed320.ExtendValue(
                Signed192.Signed(center.Y.m_rawValue)));
        Signed576 deltaX =
            WideArithmetic.SubtractSigned576(axisPointX, centerX);
        Signed576 deltaY =
            WideArithmetic.SubtractSigned576(axisPointY, centerY);
        Fixed64 cosine = FixedMath.Cos(frameRotation);
        Fixed64 sine = FixedMath.Sin(frameRotation);
        Signed192 cosineRaw =
            Signed192.Signed(cosine.m_rawValue);
        Signed192 sineRaw =
            Signed192.Signed(sine.m_rawValue);
        Signed576 xNumerator = WideArithmetic.AddSigned576(
            WideArithmetic.MultiplySigned576(deltaX, cosineRaw),
            WideArithmetic.MultiplySigned576(deltaY, sineRaw));
        Signed576 yNumerator = WideArithmetic.SubtractSigned576(
            WideArithmetic.MultiplySigned576(deltaY, cosineRaw),
            WideArithmetic.MultiplySigned576(deltaX, sineRaw));
        Signed192 fixedScale =
            Signed192.Signed(Fixed64.One.m_rawValue);
        xNumerator = WideArithmetic.MultiplySigned576(
            xNumerator,
            fixedScale);
        yNumerator = WideArithmetic.MultiplySigned576(
            yNumerator,
            fixedScale);
        Signed320 rotationDenominator = WideArithmetic.AddSigned320(
            WideArithmetic.MultiplySigned192(cosineRaw, cosineRaw),
            WideArithmetic.MultiplySigned192(sineRaw, sineRaw));
        Signed576 denominator = WideArithmetic.MultiplySigned320(
            axisPointDenominator,
            rotationDenominator);
        _ = Fixed64.TryGetSignedRawRatio(
            xNumerator,
            denominator,
            out Fixed64 x);
        _ = Fixed64.TryGetSignedRawRatio(
            yNumerator,
            denominator,
            out Fixed64 y);
        return new Vector2d(x, y);
    }

    private static void GetLocalRadialFeature(
        Vector2d worldDirection,
        Fixed64 radius,
        Fixed64 frameRotation,
        out Vector2d localDirection,
        out Vector2d roundedLocalOffset)
    {
        _ = Vector2d.TryRotate(
            worldDirection,
            -frameRotation,
            out localDirection);
        localDirection = WideGeometry.GetNormalized(localDirection);
        roundedLocalOffset = localDirection * radius;
    }

    private static bool TryGetCenteredCapsulesDepth(
        Signed832 squaredDistanceNumerator,
        Signed320 distanceDenominator,
        Fixed64 firstRadius,
        Fixed64 secondRadius,
        out Fixed64 depth,
        out bool depthIsClamped)
    {
        Signed192 combinedRadius = WideArithmetic.AddSigned192(
            Signed192.Signed(firstRadius.m_rawValue),
            Signed192.Signed(secondRadius.m_rawValue));
        GetCenteredRadiusDepth(
            squaredDistanceNumerator,
            Signed576.ExtendValue(distanceDenominator),
            combinedRadius,
            out depth,
            out depthIsClamped);
        return true;
    }

    private static void GetCenteredRadiusDepth(
        Signed832 squaredDistanceNumerator,
        Signed576 distanceDenominator,
        Signed192 radius,
        out Fixed64 depth,
        out bool depthIsClamped)
    {
        Signed192 maximumThreshold = WideArithmetic.SubtractSigned192(
            radius,
            Signed192.Signed(Fixed64.MaxValue.m_rawValue));
        if (maximumThreshold.Sign > 0
            && CompareSquaredDistanceToCenteredDepthThreshold(
                squaredDistanceNumerator,
                distanceDenominator,
                maximumThreshold) < 0)
        {
            depth = Fixed64.MaxValue;
            depthIsClamped = true;
            return;
        }

        Signed576 root = WideArithmetic.GetFloorSquareRootOfProduct(
            squaredDistanceNumerator,
            Scale);
        Signed576 radiusNumerator = WideArithmetic.MultiplySigned576(
            distanceDenominator,
            radius);
        Signed576 approximateDepthNumerator =
            WideArithmetic.SubtractSigned576(radiusNumerator, root);
        depth = Fixed64.GetNonNegativeRawRatioFloor(
            Signed704.ExtendValue(approximateDepthNumerator),
            Signed704.ExtendValue(distanceDenominator));

        Signed192 floorThreshold = WideArithmetic.SubtractSigned192(
            radius,
            Signed192.Signed(depth.m_rawValue));
        if (CompareSquaredDistanceToCenteredDepthThreshold(
                squaredDistanceNumerator,
                distanceDenominator,
                floorThreshold) > 0)
        {
            depth = Fixed64.FromRaw(depth.m_rawValue - 1L);
        }

        Signed192 doubledRadius = WideArithmetic.AddSigned192(
            radius,
            radius);
        Signed192 doubledMidpoint = WideArithmetic.AddSigned192(
            WideArithmetic.AddSigned192(
                Signed192.Signed(depth.m_rawValue),
                Signed192.Signed(depth.m_rawValue)),
            Scale);
        Signed192 distanceThreshold = WideArithmetic.SubtractSigned192(
            doubledRadius,
            doubledMidpoint);
        if (distanceThreshold.Sign <= 0)
        {
            depthIsClamped = false;
            return;
        }

        int comparison = CompareSquaredDistanceToCenteredDepthThreshold(
            squaredDistanceNumerator,
            distanceDenominator,
            distanceThreshold,
            quadrupleDistance: true);
        depth = Fixed64.FromRaw(
            depth.m_rawValue
            + GetHalfToEvenIncrement(-comparison, depth.m_rawValue));

        depthIsClamped = false;
    }

    private static int CompareSquaredDistanceToCenteredDepthThreshold(
        Signed832 squaredDistanceNumerator,
        Signed576 distanceDenominator,
        Signed192 distanceThreshold,
        bool quadrupleDistance = false)
    {
        Signed576 thresholdNumerator = WideArithmetic.MultiplySigned576(
            distanceDenominator,
            distanceThreshold);
        Signed832 left = squaredDistanceNumerator;
        if (quadrupleDistance)
        {
            left = WideArithmetic.AddSigned832(
                WideArithmetic.AddSigned832(left, left),
                WideArithmetic.AddSigned832(left, left));
        }
        return WideArithmetic.SubtractSigned832(
            left,
            WideArithmetic.MultiplySigned576ToSigned832(
                thresholdNumerator,
                thresholdNumerator)).Sign;
    }
}
