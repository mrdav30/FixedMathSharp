//=======================================================================
// WidePlanarProjection.cs
//=======================================================================
// MIT License, Copyright (c) 2024-present David Oravsky (mrdav30)
// See LICENSE file in the project root for full license information.
//=======================================================================

using System;

namespace FixedMathSharp.Geometry;

/// <summary>
/// Owns exact point relations to complete X/Z projections without
/// materializing conceptual world-space features.
/// </summary>
internal static partial class WidePlanarProjection
{
    #region Nested Types

    private readonly struct RationalPoint
    {
        internal RationalPoint(Signed320 x, Signed320 z)
        {
            X = x;
            Z = z;
        }

        internal readonly Signed320 X;
        internal readonly Signed320 Z;
    }

    private readonly struct RationalDistance
    {
        internal RationalDistance(
            Signed576 x,
            Signed576 z,
            Signed576 denominator)
        {
            X = x;
            Z = z;
            Denominator = denominator;
            SquaredDistance = WideArithmetic.AddSigned832(
                WideArithmetic.MultiplySigned576ToSigned832(x, x),
                WideArithmetic.MultiplySigned576ToSigned832(z, z));
        }

        internal readonly Signed576 X;
        internal readonly Signed576 Z;
        internal readonly Signed576 Denominator;
        internal readonly Signed832 SquaredDistance;
    }

    #endregion

    internal static bool TryGetSphereRelation(
        Vector2d circleCenter,
        Fixed64 circleRadius,
        Vector3d sphereCenter,
        Fixed64 sphereRadius,
        out PlanarProjectionRelation relation)
    {
        Vector2d projectedCenter = new(
            sphereCenter.X,
            sphereCenter.Z);
        if (WideGeometry.CompareDistanceToRadiusSum(
                circleCenter,
                projectedCenter,
                circleRadius,
                sphereRadius) > 0)
        {
            relation = default;
            return false;
        }

        _ = WideFiniteAxisIntersection.TryGetDistanceToCenteredCapsule(
            circleCenter,
            projectedCenter,
            Vector2d.Right,
            Fixed64.Zero,
            sphereRadius,
            out Fixed64 distance);
        if (distance == Fixed64.Zero)
        {
            relation = PlanarProjectionRelation.Contained;
            return true;
        }

        Vector2d normal =
            WideNormalization.GetDirection(projectedCenter, circleCenter);
        relation = new PlanarProjectionRelation(
            distance,
            -normal * distance);
        return true;
    }

    internal static bool TryGetCenteredCapsuleRelation(
        Vector2d circleCenter,
        Fixed64 circleRadius,
        Vector3d capsuleCenter,
        FixedQuaternion capsuleRotation,
        Vector3d capsuleLocalAxis,
        Fixed64 capsuleAxisLength,
        Fixed64 capsuleRadius,
        out PlanarProjectionRelation relation)
    {
        WideRationalBasis3d basis = new(capsuleRotation);
        GetRotatedLocalAxis(
            basis,
            capsuleLocalAxis,
            out Signed320 axisX,
            out _,
            out Signed320 axisZ);
        GetCenteredAxisEndpoints(
            capsuleCenter,
            basis.Denominator,
            axisX,
            axisZ,
            capsuleAxisLength,
            out Signed192 axisDenominator,
            out RationalPoint first,
            out RationalPoint second);
        RationalDistance axisDistance = GetSegmentDistance(
            circleCenter,
            axisDenominator,
            first,
            second);
        return TryCreateRadiallyExpandedRelation(
            axisDistance,
            capsuleRadius,
            circleRadius,
            out relation);
    }

    internal static bool TryGetCenteredCylinderRelation(
        Vector2d circleCenter,
        Fixed64 circleRadius,
        Vector3d cylinderCenter,
        FixedQuaternion cylinderRotation,
        Fixed64 cylinderAxisLength,
        Fixed64 cylinderRadius,
        out PlanarProjectionRelation relation)
    {
        WideRationalBasis3d basis = new(cylinderRotation);
        Signed192 axisX = basis.Yx;
        Signed192 axisY = basis.Yy;
        Signed192 axisZ = basis.Yz;
        GetCenteredBasisAxisEndpoints(
            cylinderCenter,
            basis.Denominator,
            axisX,
            axisZ,
            cylinderAxisLength,
            out Signed192 axisDenominator,
            out RationalPoint first,
            out RationalPoint second);
        RationalDistance axisDistance = GetSegmentDistance(
            circleCenter,
            axisDenominator,
            first,
            second,
            out int feature);
        if (feature == 0)
        {
            return TryCreateRadiallyExpandedRelation(
                axisDistance,
                cylinderRadius,
                circleRadius,
                out relation);
        }

        return TryGetProjectedDiskRelation(
            circleCenter,
            circleRadius,
            feature < 0 ? first : second,
            axisDenominator,
            axisX,
            axisY,
            axisZ,
            cylinderRadius,
            out relation);
    }

    internal static bool TryGetCenteredConeRelation(
        Vector2d circleCenter,
        Fixed64 circleRadius,
        Vector3d coneCenter,
        FixedQuaternion coneRotation,
        Fixed64 coneHeight,
        Fixed64 coneRadius,
        out PlanarProjectionRelation relation)
    {
        WideRationalBasis3d basis = new(coneRotation);
        Signed192 axisX = basis.Yx;
        Signed192 axisY = basis.Yy;
        Signed192 axisZ = basis.Yz;
        GetCenteredBasisAxisEndpoints(
            coneCenter,
            basis.Denominator,
            axisX,
            axisZ,
            coneHeight,
            out Signed192 axisDenominator,
            out RationalPoint baseCenter,
            out RationalPoint apex);
        if (axisY.IsZero)
        {
            GetProjectedDiskSegmentEndpoints(
                baseCenter,
                axisX,
                axisZ,
                coneRadius,
                out RationalPoint first,
                out RationalPoint second);
            Span<RationalPoint> triangle =
                stackalloc RationalPoint[3];
            triangle[0] = first;
            triangle[1] = second;
            triangle[2] = apex;
            return TryGetConvexRelation(
                circleCenter,
                circleRadius,
                axisDenominator,
                triangle,
                out relation);
        }
        if (axisX.IsZero && axisZ.IsZero)
        {
            return TryGetProjectedDiskRelation(
                circleCenter,
                circleRadius,
                baseCenter,
                axisDenominator,
                axisX,
                axisY,
                axisZ,
                coneRadius,
                out relation);
        }

        return TryGetRotatedConeRelation(
            circleCenter,
            circleRadius,
            baseCenter,
            apex,
            axisDenominator,
            basis.Denominator,
            axisX,
            axisY,
            axisZ,
            coneHeight,
            coneRadius,
            out relation);
    }

    private static bool TryCreateRadiallyExpandedRelation(
        RationalDistance axisDistance,
        Fixed64 shapeRadius,
        Fixed64 queryRadius,
        out PlanarProjectionRelation relation)
    {
        Signed576 shapeRadiusNumerator =
            WideArithmetic.MultiplySigned576(
                axisDistance.Denominator,
                Signed192.Raw(shapeRadius));
        Signed832 shapeRadiusSquared =
            WideArithmetic.MultiplySigned576ToSigned832(
                shapeRadiusNumerator,
                shapeRadiusNumerator);
        if (WideArithmetic.SubtractSigned832(
                axisDistance.SquaredDistance,
                shapeRadiusSquared).Sign <= 0)
        {
            relation = PlanarProjectionRelation.Contained;
            return true;
        }

        if (!TryGetRadialGap(
                axisDistance,
                shapeRadius,
                queryRadius,
                out Fixed64 distance))
        {
            relation = default;
            return false;
        }

        Vector2d direction = WideNormalization.GetNormalized(
            axisDistance.X,
            axisDistance.Z);
        Vector2d offset = direction * distance;
        relation = new PlanarProjectionRelation(distance, offset);
        return true;
    }

    private static void GetRotatedLocalAxis(
        WideRationalBasis3d basis,
        Vector3d localAxis,
        out Signed320 x,
        out Signed320 y,
        out Signed320 z)
    {
        x = GetLocalOffsetNumerator(
            basis.Xx,
            basis.Yx,
            basis.Zx,
            localAxis);
        y = GetLocalOffsetNumerator(
            basis.Xy,
            basis.Yy,
            basis.Zy,
            localAxis);
        z = GetLocalOffsetNumerator(
            basis.Xz,
            basis.Yz,
            basis.Zz,
            localAxis);
    }

    private static Signed320 GetLocalOffsetNumerator(
        Signed192 x,
        Signed192 y,
        Signed192 z,
        Vector3d localPoint) =>
        WideArithmetic.AddSigned320(
            WideArithmetic.AddSigned320(
                WideArithmetic.MultiplySigned192(
                    x,
                    Signed192.Raw(localPoint.X)),
                WideArithmetic.MultiplySigned192(
                    y,
                    Signed192.Raw(localPoint.Y))),
            WideArithmetic.MultiplySigned192(
                z,
                Signed192.Raw(localPoint.Z)));

    private static RationalPoint GetWorldPoint(
        Vector3d origin,
        WideRationalBasis3d basis,
        Vector3d localPoint) =>
        new(
            WideArithmetic.AddSigned320(
                WideArithmetic.MultiplySigned192(
                    Signed192.Raw(origin.X),
                    basis.Denominator),
                GetLocalOffsetNumerator(
                    basis.Xx,
                    basis.Yx,
                    basis.Zx,
                    localPoint)),
            WideArithmetic.AddSigned320(
                WideArithmetic.MultiplySigned192(
                    Signed192.Raw(origin.Z),
                    basis.Denominator),
                GetLocalOffsetNumerator(
                    basis.Xz,
                    basis.Yz,
                    basis.Zz,
                    localPoint)));

    private static void GetCenteredAxisEndpoints(
        Vector3d center,
        Signed192 denominator,
        Signed320 axisX,
        Signed320 axisZ,
        Fixed64 axisLength,
        out Signed192 axisDenominator,
        out RationalPoint first,
        out RationalPoint second)
    {
        Signed192 halfLengthNumerator =
            Signed192.Signed(axisLength.m_rawValue);
        axisDenominator = Signed192.NarrowValue(
            WideArithmetic.AddSigned320(
                WideArithmetic.MultiplySigned192(
                    denominator,
                    Signed192.Raw(Fixed64.One)),
                WideArithmetic.MultiplySigned192(
                    denominator,
                    Signed192.Raw(Fixed64.One))));
        Signed320 centerX = WideArithmetic.MultiplySigned192(
            Signed192.Raw(center.X),
            axisDenominator);
        Signed320 centerZ = WideArithmetic.MultiplySigned192(
            Signed192.Raw(center.Z),
            axisDenominator);
        Signed320 offsetX = Signed320.NarrowValue(
            WideArithmetic.MultiplySigned320(
                axisX,
                Signed320.ExtendValue(halfLengthNumerator)));
        Signed320 offsetZ = Signed320.NarrowValue(
            WideArithmetic.MultiplySigned320(
                axisZ,
                Signed320.ExtendValue(halfLengthNumerator)));
        first = new RationalPoint(
            WideArithmetic.SubtractSigned320(centerX, offsetX),
            WideArithmetic.SubtractSigned320(centerZ, offsetZ));
        second = new RationalPoint(
            WideArithmetic.AddSigned320(centerX, offsetX),
            WideArithmetic.AddSigned320(centerZ, offsetZ));
    }

    private static void GetCenteredBasisAxisEndpoints(
        Vector3d center,
        Signed192 denominator,
        Signed192 axisX,
        Signed192 axisZ,
        Fixed64 axisLength,
        out Signed192 axisDenominator,
        out RationalPoint first,
        out RationalPoint second)
    {
        axisDenominator = WideArithmetic.AddSigned192(
            denominator,
            denominator);
        Signed320 centerX = WideArithmetic.MultiplySigned192(
            Signed192.Raw(center.X),
            axisDenominator);
        Signed320 centerZ = WideArithmetic.MultiplySigned192(
            Signed192.Raw(center.Z),
            axisDenominator);
        Signed320 offsetX = WideArithmetic.MultiplySigned192(
            axisX,
            Signed192.Raw(axisLength));
        Signed320 offsetZ = WideArithmetic.MultiplySigned192(
            axisZ,
            Signed192.Raw(axisLength));
        first = new RationalPoint(
            WideArithmetic.SubtractSigned320(centerX, offsetX),
            WideArithmetic.SubtractSigned320(centerZ, offsetZ));
        second = new RationalPoint(
            WideArithmetic.AddSigned320(centerX, offsetX),
            WideArithmetic.AddSigned320(centerZ, offsetZ));
    }

    private static bool TryGetRadialGap(
        RationalDistance distance,
        Fixed64 shapeRadius,
        Fixed64 maximumGap,
        out Fixed64 gap)
    {
        Signed192 shapeRaw = Signed192.Raw(shapeRadius);
        Signed192 maximumRaw = WideArithmetic.AddSigned192(
            shapeRaw,
            Signed192.Raw(maximumGap));
        if (CompareDistanceToRaw(distance, maximumRaw) > 0)
        {
            gap = default;
            return false;
        }

        long low = 0L;
        long high = maximumGap.m_rawValue;
        while (low < high)
        {
            long difference = high - low;
            long middle =
                low + (difference >> 1) + (difference & 1L);
            Signed192 candidate = WideArithmetic.AddSigned192(
                shapeRaw,
                Signed192.Signed(middle));
            if (CompareDistanceToRaw(distance, candidate) >= 0)
                low = middle;
            else
                high = middle - 1L;
        }

        if (low != maximumGap.m_rawValue)
        {
            Signed192 midpoint = WideArithmetic.AddSigned192(
                WideArithmetic.AddSigned192(
                    WideArithmetic.AddSigned192(shapeRaw, shapeRaw),
                    WideArithmetic.AddSigned192(
                        Signed192.Signed(low),
                        Signed192.Signed(low))),
                Signed192.Signed(1L));
            int comparison =
                CompareDistanceToTwiceRaw(distance, midpoint);
            low += GetHalfToEvenIncrement(comparison, low);
        }

        gap = Fixed64.FromRaw(low);
        return true;
    }

    private static RationalDistance GetPointDistance(
        Vector2d point,
        Signed192 denominator,
        RationalPoint target)
    {
        Signed320 x = WideArithmetic.SubtractSigned320(
            target.X,
            WideArithmetic.MultiplySigned192(
                Signed192.Raw(point.X),
                denominator));
        Signed320 z = WideArithmetic.SubtractSigned320(
            target.Z,
            WideArithmetic.MultiplySigned192(
                Signed192.Raw(point.Y),
                denominator));
        return new RationalDistance(
            Signed576.ExtendValue(x),
            Signed576.ExtendValue(z),
            Signed576.ExtendValue(
                Signed320.ExtendValue(denominator)));
    }

    private static RationalDistance GetSegmentDistance(
        Vector2d point,
        Signed192 denominator,
        RationalPoint first,
        RationalPoint second) =>
        GetSegmentDistance(
            point,
            denominator,
            first,
            second,
            out _);

    private static RationalDistance GetSegmentDistance(
        Vector2d point,
        Signed192 denominator,
        RationalPoint first,
        RationalPoint second,
        out int feature)
    {
        Signed320 firstX = WideArithmetic.SubtractSigned320(
            first.X,
            WideArithmetic.MultiplySigned192(
                Signed192.Raw(point.X),
                denominator));
        Signed320 firstZ = WideArithmetic.SubtractSigned320(
            first.Z,
            WideArithmetic.MultiplySigned192(
                Signed192.Raw(point.Y),
                denominator));
        Signed320 edgeX = WideArithmetic.SubtractSigned320(
            second.X,
            first.X);
        Signed320 edgeZ = WideArithmetic.SubtractSigned320(
            second.Z,
            first.Z);
        Signed576 edgeSquared = WideArithmetic.AddSigned576(
            WideArithmetic.MultiplySigned320(edgeX, edgeX),
            WideArithmetic.MultiplySigned320(edgeZ, edgeZ));
        if (edgeSquared.IsZero)
        {
            feature = -1;
            return new RationalDistance(
                Signed576.ExtendValue(firstX),
                Signed576.ExtendValue(firstZ),
                Signed576.ExtendValue(
                    Signed320.ExtendValue(denominator)));
        }

        Signed576 projection = WideArithmetic.SubtractSigned576(
            default,
            WideArithmetic.AddSigned576(
                WideArithmetic.MultiplySigned320(firstX, edgeX),
                WideArithmetic.MultiplySigned320(firstZ, edgeZ)));
        if (projection.Sign <= 0)
        {
            feature = -1;
            return new RationalDistance(
                Signed576.ExtendValue(firstX),
                Signed576.ExtendValue(firstZ),
                Signed576.ExtendValue(
                    Signed320.ExtendValue(denominator)));
        }
        if (WideArithmetic.SubtractSigned576(
                projection,
                edgeSquared).Sign >= 0)
        {
            feature = 1;
            return GetPointDistance(point, denominator, second);
        }

        feature = 0;
        Signed576 x = WideArithmetic.AddSigned576(
            WideArithmetic.MultiplySigned576(
                edgeSquared,
                Signed192.NarrowValue(firstX)),
            WideArithmetic.MultiplySigned576(
                projection,
                Signed192.NarrowValue(edgeX)));
        Signed576 z = WideArithmetic.AddSigned576(
            WideArithmetic.MultiplySigned576(
                edgeSquared,
                Signed192.NarrowValue(firstZ)),
            WideArithmetic.MultiplySigned576(
                projection,
                Signed192.NarrowValue(edgeZ)));
        Signed576 combinedDenominator =
            WideArithmetic.MultiplySigned576(
                edgeSquared,
                denominator);
        return new RationalDistance(x, z, combinedDenominator);
    }

    private static bool IsWithinRadius(
        RationalDistance distance,
        Fixed64 radius)
    {
        Signed576 scaledRadius = WideArithmetic.MultiplySigned576(
            distance.Denominator,
            Signed192.Raw(radius));
        Signed832 squaredRadius =
            WideArithmetic.MultiplySigned576ToSigned832(
                scaledRadius,
                scaledRadius);
        return WideArithmetic.SubtractSigned832(
            distance.SquaredDistance,
            squaredRadius).Sign <= 0;
    }

    private static bool TryGetDistance(
        RationalDistance distance,
        Fixed64 maximum,
        out Fixed64 result)
    {
        if (!IsWithinRadius(distance, maximum))
        {
            result = default;
            return false;
        }

        long low = 0L;
        long high = maximum.m_rawValue;
        while (low < high)
        {
            long difference = high - low;
            long middle =
                low + (difference >> 1) + (difference & 1L);
            if (CompareDistanceToRaw(distance, middle) >= 0)
                low = middle;
            else
                high = middle - 1L;
        }

        if (low != maximum.m_rawValue)
        {
            long doubledLow = low + low;
            int midpointComparison =
                CompareDistanceToTwiceRaw(
                    distance,
                    doubledLow + 1L);
            low += GetHalfToEvenIncrement(midpointComparison, low);
        }

        result = Fixed64.FromRaw(low);
        return true;
    }

    private static int CompareDistanceToRaw(
        RationalDistance distance,
        long raw) =>
        CompareDistanceToRaw(
            distance,
            Signed192.Signed(raw));

    private static int CompareDistanceToRaw(
        RationalDistance distance,
        Signed192 raw)
    {
        Signed576 scaled = WideArithmetic.MultiplySigned576(
            distance.Denominator,
            raw);
        Signed832 squared = WideArithmetic.MultiplySigned576ToSigned832(
            scaled,
            scaled);
        return WideArithmetic.SubtractSigned832(
            distance.SquaredDistance,
            squared).Sign;
    }

    private static int CompareDistanceToTwiceRaw(
        RationalDistance distance,
        long twiceRaw) =>
        CompareDistanceToTwiceRaw(
            distance,
            Signed192.Signed(twiceRaw));

    private static int CompareDistanceToTwiceRaw(
        RationalDistance distance,
        Signed192 twiceRaw)
    {
        Signed832 fourDistance = WideArithmetic.AddSigned832(
            WideArithmetic.AddSigned832(
                distance.SquaredDistance,
                distance.SquaredDistance),
            WideArithmetic.AddSigned832(
                distance.SquaredDistance,
                distance.SquaredDistance));
        Signed576 scaled = WideArithmetic.MultiplySigned576(
            distance.Denominator,
            twiceRaw);
        Signed832 squared = WideArithmetic.MultiplySigned576ToSigned832(
            scaled,
            scaled);
        return WideArithmetic.SubtractSigned832(
            fourDistance,
            squared).Sign;
    }

    private static Vector2d GetOffset(RationalDistance distance)
    {
        _ = Fixed64.TryGetSignedRawRatio(
            distance.X,
            distance.Denominator,
            out Fixed64 x);
        _ = Fixed64.TryGetSignedRawRatio(
            distance.Z,
            distance.Denominator,
            out Fixed64 z);
        return new Vector2d(x, z);
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

    private static Signed320 GetLargestPositiveMagnitude(
        Signed320 first,
        Signed320 second,
        Signed320 third,
        Signed320 fourth,
        Signed320 fifth)
    {
        Signed320 largest =
            WideArithmetic.CompareMagnitude(first, second) >= 0
                ? first
                : second;
        if (WideArithmetic.CompareMagnitude(largest, third) < 0)
            largest = third;
        if (WideArithmetic.CompareMagnitude(largest, fourth) < 0)
            largest = fourth;
        if (WideArithmetic.CompareMagnitude(largest, fifth) < 0)
            largest = fifth;
        return largest.Sign < 0
            ? WideArithmetic.SubtractSigned320(default, largest)
            : largest;
    }
}
