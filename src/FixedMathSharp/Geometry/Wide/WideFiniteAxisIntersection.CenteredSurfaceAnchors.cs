//=======================================================================
// WideFiniteAxisIntersection.CenteredSurfaceAnchors.cs
//=======================================================================
// MIT License, Copyright (c) 2024–present David Oravsky (mrdav30)
// See LICENSE file in the project root for full license information.
//=======================================================================

namespace FixedMathSharp.Geometry;

/// <content>
/// Provides methods for computing the closest surface anchor points on centered,
/// finite-axis shapes (e.g. capsules) relative to a given world-space point.
/// </content>
internal static partial class WideFiniteAxisIntersection
{
    internal static bool TryGetClosestCenteredCapsuleSurfaceAnchor(
        Vector3d point,
        Vector3d center,
        FixedQuaternion frameRotation,
        Vector3d localAxisDirection,
        Fixed64 axisLength,
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
        int cap = GetCenteredCap(relation, axisLength);
        Vector3d localSurfaceDirection = GetLocalCapsuleSurfaceDirection(
            relation,
            localAxisDirection,
            axisLength,
            cap,
            localFallbackRadialDirection);
        CreateCenteredCapsuleAnchors(
            center,
            frameRotation,
            localAxisDirection,
            axisLength,
            radius,
            localSurfaceDirection,
            cap,
            relation,
            out FixedPointAnchor axisAnchor,
            out surfaceAnchor);

        var pointAnchor = new FixedPointAnchor(
            point,
            FixedQuaternion.Identity,
            Vector3d.Zero);
        bool representable = pointAnchor.TryGetOffsetFrom(
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

        bool contained = axisAnchor.CompareSquaredDistance(
            pointAnchor,
            surfaceAnchor) <= 0;
        signedDistance = contained ? -distance : distance;
        _ = frameRotation.TryRotate(
            localSurfaceDirection,
            out Vector3d rotatedNormal);
        outwardNormal = WideGeometry.GetNormalized(rotatedNormal);
        return true;
    }

    private static Vector3d GetLocalCapsuleSurfaceDirection(
        RigidLocalPointRelation relation,
        Vector3d localAxisDirection,
        Fixed64 axisLength,
        int cap,
        Vector3d localFallbackDirection)
    {
        if (cap == 0)
        {
            return GetLocalRadialDirection(
                relation,
                localAxisDirection,
                localFallbackDirection);
        }

        Signed192 axisLengthRaw = Signed192.Signed(
            axisLength.m_rawValue);
        Signed192 axisX = Signed192.Signed(
            localAxisDirection.X.m_rawValue);
        Signed192 axisY = Signed192.Signed(
            localAxisDirection.Y.m_rawValue);
        Signed192 axisZ = Signed192.Signed(
            localAxisDirection.Z.m_rawValue);
        Signed192 sign = Signed192.Signed(cap);
        Signed192 pointScale = Signed192.Signed(
            Fixed64.Two.m_rawValue);
        Signed320 x = GetLocalCapsuleCapDelta(
            relation.X,
            relation.Denominator,
            pointScale,
            axisX,
            axisLengthRaw,
            sign);
        Signed320 y = GetLocalCapsuleCapDelta(
            relation.Y,
            relation.Denominator,
            pointScale,
            axisY,
            axisLengthRaw,
            sign);
        Signed320 z = GetLocalCapsuleCapDelta(
            relation.Z,
            relation.Denominator,
            pointScale,
            axisZ,
            axisLengthRaw,
            sign);
        return WideGeometry.GetNormalized(x, y, z);
    }

    private static Signed320 GetLocalCapsuleCapDelta(
        Signed192 pointNumerator,
        Signed192 denominator,
        Signed192 pointScale,
        Signed192 axisComponent,
        Signed192 axisLength,
        Signed192 sign)
    {
        Signed320 scaledPoint = WideArithmetic.MultiplySigned192(
            pointNumerator,
            pointScale);
        _ = Signed192.TryNarrowSigned(
            WideArithmetic.MultiplySigned192(
                axisComponent,
                sign),
            out Signed192 signedAxisComponent);
        Signed320 axisAndLength = WideArithmetic.MultiplySigned192(
            signedAxisComponent,
            axisLength);
        _ = Signed192.TryNarrowSigned(
            axisAndLength,
            out Signed192 narrowAxisAndLength);
        Signed320 endpoint = WideArithmetic.MultiplySigned192(
            denominator,
            narrowAxisAndLength);
        return WideArithmetic.SubtractSigned320(
            scaledPoint,
            endpoint);
    }

    internal static bool TryGetClosestCenteredFiniteCylinderSurfaceAnchor(
        Vector3d point,
        Vector3d center,
        FixedQuaternion frameRotation,
        Vector3d localAxisDirection,
        Fixed64 axisLength,
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
        int cap = GetCenteredCap(relation, axisLength);
        Signed576 radialBound = GetCenteredRadialBound(
            relation,
            radius);
        bool radialOutside = CompareSigned576(
            relation.RadialNumerator,
            radialBound) > 0;
        bool contained =
            cap == 0
            && CompareSigned576(
                relation.RadialNumerator,
                radialBound) <= 0;
        Vector3d localRadialDirection = GetLocalRadialDirection(
            relation,
            localAxisDirection,
            localFallbackRadialDirection);
        FixedPointAnchor sideAnchor = CreateCenteredCylinderSideAnchor(
            center,
            frameRotation,
            localAxisDirection,
            axisLength,
            radius,
            localRadialDirection,
            cap,
            relation);

        bool side = radialOutside || cap == 0;
        FixedPointAnchor capAnchor = default;
        if (!side || contained)
        {
            _ = TryGetCenteredRadialDistance(
                relation,
                out Fixed64 radialDistance);
            capAnchor = CreateCenteredSurfaceAnchor(
                center,
                frameRotation,
                localAxisDirection,
                relation.Projection.Sign >= 0
                    ? axisLength
                    : -axisLength,
                localRadialDirection,
                radialDistance);
        }
        if (contained)
        {
            var pointAnchor = new FixedPointAnchor(
                point,
                FixedQuaternion.Identity,
                Vector3d.Zero);
            side = pointAnchor.CompareSquaredDistance(
                sideAnchor,
                capAnchor) <= 0;
        }
        surfaceAnchor = side ? sideAnchor : capAnchor;

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
        if (side && cap != 0 && radialOutside)
        {
            outwardNormal = WideGeometry.GetNormalized(surfaceToPoint);
        }
        else
        {
            Vector3d localNormal = side
                ? localRadialDirection
                : (cap > 0 || relation.Projection.Sign >= 0
                    ? localAxisDirection
                    : -localAxisDirection);
            _ = frameRotation.TryRotate(
                localNormal,
                out Vector3d rotatedNormal);
            outwardNormal = WideGeometry.GetNormalized(rotatedNormal);
        }
        return true;
    }

    private static FixedPointAnchor CreateCenteredCylinderSideAnchor(
        Vector3d center,
        FixedQuaternion frameRotation,
        Vector3d localAxisDirection,
        Fixed64 axisLength,
        Fixed64 radius,
        Vector3d localRadialDirection,
        int cap,
        RigidLocalPointRelation relation)
    {
        if (cap != 0)
        {
            return CreateCenteredSurfaceAnchor(
                center,
                frameRotation,
                localAxisDirection,
                cap > 0 ? axisLength : -axisLength,
                localRadialDirection,
                radius);
        }

        Fixed64 axialDistance =
            GetClosestCenteredRigidAxisDistance(
                relation,
                axisLength,
                cap);
        Vector3d roundedRadialOffset = localRadialDirection * radius;
        return new FixedPointAnchor(
            center,
            frameRotation,
            localAxisDirection * axialDistance,
            roundedRadialOffset,
            FixedPointAnchorTerm3d.CreateRadialSupport(
                localRadialDirection,
                radius,
                roundedRadialOffset));
    }

    private static void CreateCenteredCapsuleAnchors(
        Vector3d center,
        FixedQuaternion frameRotation,
        Vector3d localAxisDirection,
        Fixed64 axisLength,
        Fixed64 radius,
        Vector3d localRadialDirection,
        int cap,
        RigidLocalPointRelation relation,
        out FixedPointAnchor axisAnchor,
        out FixedPointAnchor surfaceAnchor)
    {
        if (cap != 0)
        {
            Fixed64 signedLength = cap > 0
                ? axisLength
                : -axisLength;
            axisAnchor = CreateCenteredSurfaceAnchor(
                center,
                frameRotation,
                localAxisDirection,
                signedLength,
                localRadialDirection,
                Fixed64.Zero);
            surfaceAnchor = CreateCenteredSurfaceAnchor(
                center,
                frameRotation,
                localAxisDirection,
                signedLength,
                localRadialDirection,
                radius);
            return;
        }

        Fixed64 axialDistance =
            GetClosestCenteredRigidAxisDistance(
                relation,
                axisLength,
                cap);
        Vector3d localAxisOffset =
            localAxisDirection * axialDistance;
        axisAnchor = new FixedPointAnchor(
            center,
            frameRotation,
            localAxisOffset);
        Vector3d roundedRadialOffset =
            localRadialDirection * radius;
        surfaceAnchor = new FixedPointAnchor(
            center,
            frameRotation,
            localAxisOffset,
            roundedRadialOffset,
            FixedPointAnchorTerm3d.CreateRadialSupport(
                localRadialDirection,
                radius,
                roundedRadialOffset));
    }

    private static Fixed64 GetClosestCenteredRigidAxisDistance(
        RigidLocalPointRelation relation,
        Fixed64 axisLength,
        int cap)
    {
        if (cap != 0)
        {
            return cap > 0
                ? axisLength / Fixed64.Two
                : -(axisLength / Fixed64.Two);
        }

        Signed576 numerator = WideArithmetic.MultiplySigned320(
            relation.Projection,
            Signed320.ExtendValue(
                Signed192.Signed(
                    Fixed64.One.m_rawValue)));
        Signed320 denominator = WideArithmetic.MultiplySigned192(
            relation.Denominator,
            relation.AxisSquared);
        // Interior classification proves the ratio lies within the centered
        // axis's representable half-length.
        _ = Fixed64.TryGetSignedRawRatio(
            numerator,
            Signed576.ExtendValue(denominator),
            out Fixed64 axialDistance);
        return axialDistance;
    }

    private static FixedPointAnchor CreateCenteredSurfaceAnchor(
        Vector3d center,
        FixedQuaternion frameRotation,
        Vector3d localAxisDirection,
        Fixed64 signedAxisLength,
        Vector3d localRadialDirection,
        Fixed64 radialDistance)
    {
        Vector3d roundedAxialOffset =
            localAxisDirection * (signedAxisLength / Fixed64.Two);
        Vector3d roundedRadialOffset =
            localRadialDirection * radialDistance;
        return new FixedPointAnchor(
            center,
            frameRotation,
            roundedAxialOffset,
            roundedRadialOffset,
            FixedPointAnchorTerm3d.CreateCenteredAxisSupport(
                localAxisDirection,
                signedAxisLength,
                localRadialDirection,
                radialDistance,
                roundedAxialOffset,
                roundedRadialOffset));
    }

    private static bool TryGetCenteredRadialDistance(
        RigidLocalPointRelation relation,
        out Fixed64 radialDistance)
    {
        Signed576 numeratorRoot = WideArithmetic.GetFloorSquareRoot(
            Signed704.ExtendValue(
                relation.RadialNumerator));
        Signed576 denominatorRootWide =
            WideArithmetic.GetFloorSquareRoot(
                Signed704.ExtendValue(
                    Signed576.ExtendValue(
                        relation.RadialDenominator)));
        Fixed64 candidate = Fixed64.GetNonNegativeRawRatioFloor(
            Signed704.ExtendValue(numeratorRoot),
            Signed704.ExtendValue(denominatorRootWide));

        Signed320 one = Signed320.ExtendValue(
            Signed192.Signed(1L));
        Signed320 candidateRaw = Signed320.ExtendValue(
            Signed192.Signed(candidate.m_rawValue));
        int candidateComparison =
            WideArithmetic.CompareNonNegativeRadicalToRatio(
                relation.RadialNumerator,
                relation.RadialDenominator,
                candidateRaw,
                one);
        candidate = Fixed64.FromRaw(
            candidate.m_rawValue + (candidateComparison >> 31));
        if (candidate != Fixed64.MaxValue)
        {
            Signed320 next = Signed320.ExtendValue(
                Signed192.Signed(candidate.m_rawValue + 1L));
            int nextComparison =
                WideArithmetic.CompareNonNegativeRadicalToRatio(
                    relation.RadialNumerator,
                    relation.RadialDenominator,
                    next,
                    one);
            candidate = Fixed64.FromRaw(
                candidate.m_rawValue
                + GetNonNegativeComparisonIncrement(nextComparison));
        }

        Signed192 doubledMidpoint = WideArithmetic.AddSigned192(
            WideArithmetic.AddSigned192(
                Signed192.Signed(candidate.m_rawValue),
                Signed192.Signed(candidate.m_rawValue)),
            Signed192.Signed(1L));
        int midpointComparison =
            WideArithmetic.CompareNonNegativeRadicalToRatio(
                relation.RadialNumerator,
                relation.RadialDenominator,
                Signed320.ExtendValue(doubledMidpoint),
                    Signed320.ExtendValue(
                        Signed192.Signed(2L)));
        long increment =
            GetHalfToEvenIncrement(midpointComparison, candidate.m_rawValue);
        if (candidate == Fixed64.MaxValue && increment != 0L)
        {
            radialDistance = default;
            return false;
        }

        radialDistance =
            Fixed64.FromRaw(candidate.m_rawValue + increment);
        return true;
    }

    private static long GetNonNegativeComparisonIncrement(int comparison) =>
        1L ^ ((uint)comparison >> 31);

    private static Vector3d GetLocalRadialDirection(
        RigidLocalPointRelation relation,
        Vector3d localAxisDirection,
        Vector3d localFallbackRadialDirection)
    {
        GetLocalRadialComponents(
            relation.X,
            relation.Y,
            relation.Z,
            relation.Projection,
            relation.AxisSquared,
            localAxisDirection,
            out Signed320 radialX,
            out Signed320 radialY,
            out Signed320 radialZ);
        Vector3d radial = WideGeometry.GetNormalized(
            radialX,
            radialY,
            radialZ);
        if (!radial.IsZero)
            return radial;

        Signed192 fallbackX = Signed192.Signed(
            localFallbackRadialDirection.X.m_rawValue);
        Signed192 fallbackY = Signed192.Signed(
            localFallbackRadialDirection.Y.m_rawValue);
        Signed192 fallbackZ = Signed192.Signed(
            localFallbackRadialDirection.Z.m_rawValue);
        Signed320 fallbackProjection = GetProjection(
            fallbackX,
            fallbackY,
            fallbackZ,
            localAxisDirection);
        GetLocalRadialComponents(
            fallbackX,
            fallbackY,
            fallbackZ,
            fallbackProjection,
            relation.AxisSquared,
            localAxisDirection,
            out radialX,
            out radialY,
            out radialZ);
        return WideGeometry.GetNormalized(radialX, radialY, radialZ);
    }

    private static Signed320 GetProjection(
        Signed192 x,
        Signed192 y,
        Signed192 z,
        Vector3d localAxisDirection) =>
        WideArithmetic.AddSigned320(
            WideArithmetic.AddSigned320(
                WideArithmetic.MultiplySigned192(
                    x,
                    Signed192.Signed(
                        localAxisDirection.X.m_rawValue)),
                WideArithmetic.MultiplySigned192(
                    y,
                    Signed192.Signed(
                        localAxisDirection.Y.m_rawValue))),
            WideArithmetic.MultiplySigned192(
                z,
                Signed192.Signed(
                    localAxisDirection.Z.m_rawValue)));

    private static int GetCenteredCap(
        RigidLocalPointRelation relation,
        Fixed64 axisLength)
    {
        Signed576 scaledProjection = WideArithmetic.MultiplySigned320(
            relation.Projection,
            Signed320.ExtendValue(
                Signed192.Signed(
                    Fixed64.One.m_rawValue)));
        scaledProjection = WideArithmetic.AddSigned576(
            scaledProjection,
            scaledProjection);
        Signed320 denominatorAndAxis = WideArithmetic.MultiplySigned192(
            relation.Denominator,
            relation.AxisSquared);
        Signed576 axialExtent = WideArithmetic.MultiplySigned320(
            denominatorAndAxis,
            Signed320.ExtendValue(
                Signed192.Signed(
                    axisLength.m_rawValue)));
        return CompareMagnitudeSigned576(
            scaledProjection,
            axialExtent) > 0
            ? relation.Projection.Sign
            : 0;
    }

    private static Signed576 GetCenteredRadialBound(
        RigidLocalPointRelation relation,
        Fixed64 radius)
    {
        Signed192 squaredRadius = GetSquaredRadius(
            Signed192.Signed(radius.m_rawValue));
        return WideArithmetic.MultiplySigned320(
            relation.RadialDenominator,
            Signed320.ExtendValue(squaredRadius));
    }

    private static void GetLocalRadialComponents(
        Signed192 x,
        Signed192 y,
        Signed192 z,
        Signed320 projection,
        Signed192 axisSquared,
        Vector3d localAxisDirection,
        out Signed320 radialX,
        out Signed320 radialY,
        out Signed320 radialZ)
    {
        radialX = GetLocalRadialComponent(
            x,
            projection,
            axisSquared,
            localAxisDirection.X);
        radialY = GetLocalRadialComponent(
            y,
            projection,
            axisSquared,
            localAxisDirection.Y);
        radialZ = GetLocalRadialComponent(
            z,
            projection,
            axisSquared,
            localAxisDirection.Z);
    }

    private static Signed320 GetLocalRadialComponent(
        Signed192 pointComponent,
        Signed320 projection,
        Signed192 axisSquared,
        Fixed64 axisComponent)
    {
        Signed576 projectedAxis = WideArithmetic.MultiplySigned320(
            projection,
            Signed320.ExtendValue(
                Signed192.Signed(
                    axisComponent.m_rawValue)));
        _ = Signed320.TryNarrowSigned(
            projectedAxis,
            out Signed320 narrowProjectedAxis);
        return WideArithmetic.SubtractSigned320(
            WideArithmetic.MultiplySigned192(
                pointComponent,
                axisSquared),
            narrowProjectedAxis);
    }

    private static int CompareSigned576(
        Signed576 left,
        Signed576 right) =>
        WideArithmetic.SubtractSigned576(left, right).Sign;

    private static int CompareMagnitudeSigned576(
        Signed576 left,
        Signed576 right)
    {
        if (left.Sign < 0)
            left = WideArithmetic.SubtractSigned576(default, left);
        return WideArithmetic.CompareNonNegative(left, right);
    }

}
