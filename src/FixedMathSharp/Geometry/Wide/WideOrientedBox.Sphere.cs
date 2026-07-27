//=======================================================================
// WideOrientedBox.Sphere.cs
//=======================================================================
// MIT License, Copyright (c) 2024–present David Oravsky (mrdav30)
// See LICENSE file in the project root for full license information.
//=======================================================================

namespace FixedMathSharp.Bounds;

/// <content>
/// Provides high-precision (wide arithmetic) contact generation and 
/// swept-sphere intersection tests between an oriented box and a sphere.
/// </content>
internal static partial class WideOrientedBox
{
    internal static bool TryGetSweptSphereIntersectionDistance(
        Vector3d center,
        FixedQuaternion orientation,
        Vector3d halfExtents,
        FixedSegment query,
        Fixed64 sphericalExpansion,
        Fixed64 segmentLength,
        out Fixed64 distance)
    {
        RationalBasis basis = new(orientation);
        GetPointProjections(
            query.Start,
            center,
            basis,
            out Signed320 startX,
            out Signed320 startY,
            out Signed320 startZ);
        GetPointProjections(
            query.End,
            center,
            basis,
            out Signed320 endX,
            out Signed320 endY,
            out Signed320 endZ);
        Signed192 narrowStartX = Signed192.NarrowValue(startX);
        Signed192 narrowStartY = Signed192.NarrowValue(startY);
        Signed192 narrowStartZ = Signed192.NarrowValue(startZ);
        Signed192 narrowEndX = Signed192.NarrowValue(endX);
        Signed192 narrowEndY = Signed192.NarrowValue(endY);
        Signed192 narrowEndZ = Signed192.NarrowValue(endZ);
        Signed192 extentX = Signed192.NarrowValue(
            GetExtentNumerator(halfExtents.X, basis.Denominator));
        Signed192 extentY = Signed192.NarrowValue(
            GetExtentNumerator(halfExtents.Y, basis.Denominator));
        Signed192 extentZ = Signed192.NarrowValue(
            GetExtentNumerator(halfExtents.Z, basis.Denominator));

        return Bounds.WideFiniteAxisIntersection
            .TryGetSphericallyExpandedProjectedBoxFirstDistance(
                narrowStartX,
                narrowStartY,
                narrowStartZ,
                narrowEndX,
                narrowEndY,
                narrowEndZ,
                extentX,
                extentY,
                extentZ,
                basis.Denominator,
                sphericalExpansion,
                segmentLength,
                out distance);
    }

    internal static bool TryGetSphereContact(
        Vector3d center,
        FixedQuaternion orientation,
        Vector3d halfExtents,
        Vector3d sphereCenter,
        FixedQuaternion sphereRotation,
        Fixed64 sphereRadius,
        out FixedContactAnchors contact)
    {
        RationalBasis basis = new(orientation);
        GetPointProjections(
            sphereCenter,
            center,
            basis,
            out Signed320 localX,
            out Signed320 localY,
            out Signed320 localZ);
        Signed320 xExtent = GetExtentNumerator(halfExtents.X, basis.Denominator);
        Signed320 yExtent = GetExtentNumerator(halfExtents.Y, basis.Denominator);
        Signed320 zExtent = GetExtentNumerator(halfExtents.Z, basis.Denominator);
        bool outsideX = WideArithmetic.CompareMagnitude(localX, xExtent) > 0;
        bool outsideY = WideArithmetic.CompareMagnitude(localY, yExtent) > 0;
        bool outsideZ = WideArithmetic.CompareMagnitude(localZ, zExtent) > 0;

        Signed320 boxLocalX = GetClampedNumerator(localX, xExtent, outsideX);
        Signed320 boxLocalY = GetClampedNumerator(localY, yExtent, outsideY);
        Signed320 boxLocalZ = GetClampedNumerator(localZ, zExtent, outsideZ);
        if (!(outsideX || outsideY || outsideZ))
        {
            return TryGetContainedSphereContact(
                center,
                orientation,
                basis,
                sphereCenter,
                sphereRotation,
                sphereRadius,
                localX,
                localY,
                localZ,
                xExtent,
                yExtent,
                zExtent,
                boxLocalX,
                boxLocalY,
                boxLocalZ,
                out contact);
        }

        Signed320 denominatorSquared = WideArithmetic.MultiplySigned192(
            basis.Denominator,
            basis.Denominator);
        GetRationalLocalOffsetNumerators(
            basis,
            boxLocalX,
            boxLocalY,
            boxLocalZ,
            out Signed576 boxOffsetX,
            out Signed576 boxOffsetY,
            out Signed576 boxOffsetZ);
        Signed576 deltaX = GetOriginRelativeNumerator(
            sphereCenter.X,
            center.X,
            denominatorSquared,
            boxOffsetX);
        Signed576 deltaY = GetOriginRelativeNumerator(
            sphereCenter.Y,
            center.Y,
            denominatorSquared,
            boxOffsetY);
        Signed576 deltaZ = GetOriginRelativeNumerator(
            sphereCenter.Z,
            center.Z,
            denominatorSquared,
            boxOffsetZ);
        Signed832 squaredDistance = SumSquares(deltaX, deltaY, deltaZ);
        Signed576 scaledRadius = WideArithmetic.MultiplySigned320(
            Signed320.ExtendValue(Signed192.Raw(sphereRadius)),
            denominatorSquared);
        Signed832 squaredRadius = WideArithmetic.MultiplySigned576ToSigned832(
            scaledRadius,
            scaledRadius);
        if (WideArithmetic.SubtractSigned832(squaredDistance, squaredRadius).Sign > 0)
        {
            contact = default;
            return false;
        }

        Vector3d normal = WideGeometry.GetNormalized(deltaX, deltaY, deltaZ);
        Vector3d boxLocalPoint = GetRationalLocalPoint(
            basis,
            boxLocalX,
            boxLocalY,
            boxLocalZ);
        FixedPointAnchor sphereAnchor = GetSphereSurfaceAnchor(
            sphereCenter,
            sphereRotation,
            sphereRadius,
            deltaX,
            deltaY,
            deltaZ,
            true);
        Fixed64 depth = GetRoundedRadialDepth(
            sphereRadius,
            squaredDistance,
            denominatorSquared);
        contact = new FixedContactAnchors(
            new FixedPointAnchor(
                center,
                orientation,
                boxLocalPoint),
            sphereAnchor,
            normal,
            depth,
            false);
        return true;
    }

    private static bool TryGetContainedSphereContact(
        Vector3d center,
        FixedQuaternion orientation,
        RationalBasis basis,
        Vector3d sphereCenter,
        FixedQuaternion sphereRotation,
        Fixed64 sphereRadius,
        Signed320 localX,
        Signed320 localY,
        Signed320 localZ,
        Signed320 xExtent,
        Signed320 yExtent,
        Signed320 zExtent,
        Signed320 boxLocalX,
        Signed320 boxLocalY,
        Signed320 boxLocalZ,
        out FixedContactAnchors contact)
    {
        Signed320 xClearance = WideArithmetic.SubtractSigned320(
            xExtent,
            GetMagnitude(localX));
        Signed320 yClearance = WideArithmetic.SubtractSigned320(
            yExtent,
            GetMagnitude(localY));
        Signed320 zClearance = WideArithmetic.SubtractSigned320(
            zExtent,
            GetMagnitude(localZ));
        int axis;
        bool negative;
        if (WideArithmetic.CompareMagnitude(xClearance, yClearance) <= 0
            && WideArithmetic.CompareMagnitude(xClearance, zClearance) <= 0)
        {
            axis = 0;
            negative = localX.Sign < 0;
            boxLocalX = negative
                ? WideArithmetic.SubtractSigned320(default, xExtent)
                : xExtent;
        }
        else if (WideArithmetic.CompareMagnitude(yClearance, zClearance) <= 0)
        {
            axis = 1;
            negative = localY.Sign < 0;
            boxLocalY = negative
                ? WideArithmetic.SubtractSigned320(default, yExtent)
                : yExtent;
        }
        else
        {
            axis = 2;
            negative = localZ.Sign < 0;
            boxLocalZ = negative
                ? WideArithmetic.SubtractSigned320(default, zExtent)
                : zExtent;
        }

        Vector3d boxLocalPoint = GetRationalLocalPoint(
            basis,
            boxLocalX,
            boxLocalY,
            boxLocalZ);

        Vector3d normal = axis switch
        {
            0 => GetAxis(
                basis.Xx,
                basis.Xy,
                basis.Xz,
                basis.Denominator,
                negative),
            1 => GetAxis(
                basis.Yx,
                basis.Yy,
                basis.Yz,
                basis.Denominator,
                negative),
            _ => GetAxis(
                basis.Zx,
                basis.Zy,
                basis.Zz,
                basis.Denominator,
                negative),
        };
        FixedPointAnchor sphereAnchor = axis switch
        {
            0 => GetSphereSurfaceAnchor(
                sphereCenter,
                sphereRotation,
                sphereRadius,
                ToSigned576(basis.Xx),
                ToSigned576(basis.Xy),
                ToSigned576(basis.Xz),
                !negative),
            1 => GetSphereSurfaceAnchor(
                sphereCenter,
                sphereRotation,
                sphereRadius,
                ToSigned576(basis.Yx),
                ToSigned576(basis.Yy),
                ToSigned576(basis.Yz),
                !negative),
            _ => GetSphereSurfaceAnchor(
                sphereCenter,
                sphereRotation,
                sphereRadius,
                ToSigned576(basis.Zx),
                ToSigned576(basis.Zy),
                ToSigned576(basis.Zz),
                !negative),
        };
        Signed320 clearance = axis switch
        {
            0 => xClearance,
            1 => yClearance,
            _ => zClearance,
        };
        Signed320 depthNumerator = WideArithmetic.AddSigned320(
            WideArithmetic.MultiplySigned192(
                Signed192.Raw(sphereRadius),
                basis.Denominator),
            clearance);
        Signed320 maximumNumerator = WideArithmetic.MultiplySigned192(
            Signed192.Raw(Fixed64.MaxValue),
            basis.Denominator);
        bool depthIsClamped = CompareSigned(
            depthNumerator,
            maximumNumerator) > 0;
        Fixed64 depth;
        if (depthIsClamped)
        {
            depth = Fixed64.MaxValue;
        }
        else
        {
            _ = Fixed64.TryGetSignedRawRatio(
                Signed576.ExtendValue(depthNumerator),
                ToSigned576(basis.Denominator),
                out depth);
        }

        contact = new FixedContactAnchors(
            new FixedPointAnchor(
                center,
                orientation,
                boxLocalPoint),
            sphereAnchor,
            normal,
            depth,
            depthIsClamped);
        return true;
    }

    private static FixedPointAnchor GetSphereSurfaceAnchor(
        Vector3d sphereCenter,
        FixedQuaternion rotation,
        Fixed64 radius,
        Signed576 directionX,
        Signed576 directionY,
        Signed576 directionZ,
        bool negate)
    {
        if (negate)
        {
            directionX = WideArithmetic.SubtractSigned576(
                default,
                directionX);
            directionY = WideArithmetic.SubtractSigned576(
                default,
                directionY);
            directionZ = WideArithmetic.SubtractSigned576(
                default,
                directionZ);
        }

        RationalBasis basis = new(rotation);
        Signed576 localX = GetProjectionNumerator(
            directionX,
            directionY,
            directionZ,
            basis.Xx,
            basis.Xy,
            basis.Xz);
        Signed576 localY = GetProjectionNumerator(
            directionX,
            directionY,
            directionZ,
            basis.Yx,
            basis.Yy,
            basis.Yz);
        Signed576 localZ = GetProjectionNumerator(
            directionX,
            directionY,
            directionZ,
            basis.Zx,
            basis.Zy,
            basis.Zz);
        Vector3d localDirection = WideGeometry.GetNormalized(
            localX,
            localY,
            localZ);
        Vector3d roundedOffset = localDirection * radius;
        FixedPointAnchorTerm3d exactTerm =
            FixedPointAnchorTerm3d.CreateRadialSupport(
                localDirection,
                radius,
                roundedOffset);
        return new FixedPointAnchor(
            sphereCenter,
            rotation,
            Vector3d.Zero,
            roundedOffset,
            exactTerm);
    }

    private static Signed576 GetProjectionNumerator(
        Signed576 directionX,
        Signed576 directionY,
        Signed576 directionZ,
        Signed192 axisX,
        Signed192 axisY,
        Signed192 axisZ) =>
        WideArithmetic.AddSigned576(
            WideArithmetic.AddSigned576(
                WideArithmetic.MultiplySigned576(directionX, axisX),
                WideArithmetic.MultiplySigned576(directionY, axisY)),
            WideArithmetic.MultiplySigned576(directionZ, axisZ));

    private static void GetRationalLocalOffsetNumerators(
        RationalBasis basis,
        Signed320 localX,
        Signed320 localY,
        Signed320 localZ,
        out Signed576 x,
        out Signed576 y,
        out Signed576 z)
    {
        x = GetRationalLocalOffsetNumerator(
            basis.Xx,
            basis.Yx,
            basis.Zx,
            localX,
            localY,
            localZ);
        y = GetRationalLocalOffsetNumerator(
            basis.Xy,
            basis.Yy,
            basis.Zy,
            localX,
            localY,
            localZ);
        z = GetRationalLocalOffsetNumerator(
            basis.Xz,
            basis.Yz,
            basis.Zz,
            localX,
            localY,
            localZ);
    }

    private static Signed576 GetRationalLocalOffsetNumerator(
        Signed192 axisX,
        Signed192 axisY,
        Signed192 axisZ,
        Signed320 localX,
        Signed320 localY,
        Signed320 localZ) =>
        WideArithmetic.AddSigned576(
            WideArithmetic.AddSigned576(
                WideArithmetic.MultiplySigned320(
                    localX,
                    Signed320.ExtendValue(axisX)),
                WideArithmetic.MultiplySigned320(
                    localY,
                    Signed320.ExtendValue(axisY))),
            WideArithmetic.MultiplySigned320(
                localZ,
                Signed320.ExtendValue(axisZ)));

    private static Signed576 GetOriginRelativeNumerator(
        Fixed64 point,
        Fixed64 origin,
        Signed320 denominatorSquared,
        Signed576 offsetNumerator) =>
        WideArithmetic.SubtractSigned576(
            WideArithmetic.MultiplySigned320(
                Signed320.ExtendValue(
                    WideArithmetic.SubtractSigned192(Signed192.Raw(point), Signed192.Raw(origin))),
                denominatorSquared),
            offsetNumerator);

    private static Signed832 SumSquares(
        Signed576 x,
        Signed576 y,
        Signed576 z) =>
        WideArithmetic.AddSigned832(
            WideArithmetic.AddSigned832(
                WideArithmetic.MultiplySigned576ToSigned832(x, x),
                WideArithmetic.MultiplySigned576ToSigned832(y, y)),
            WideArithmetic.MultiplySigned576ToSigned832(z, z));

    private static Fixed64 GetRoundedRadialDepth(
        Fixed64 radius,
        Signed832 squaredDistance,
        Signed320 denominator)
    {
        Signed576 root = WideArithmetic.GetFloorSquareRootOfProduct(
            squaredDistance,
            Signed192.Signed(1L));
        _ = Fixed64.TryGetSignedRawRatio(
            root,
            Signed576.ExtendValue(denominator),
            out Fixed64 roundedDistance);
        Fixed64 depth = radius - roundedDistance;
        CorrectRoundedRadialDepth(
            radius,
            squaredDistance,
            denominator,
            ref depth);
        return depth;
    }

    private static void CorrectRoundedRadialDepth(
        Fixed64 radius,
        Signed832 squaredDistance,
        Signed320 denominator,
        ref Fixed64 depth)
    {
        if (depth > Fixed64.Zero)
        {
            Signed192 lowerMidpoint = WideArithmetic.SubtractSigned192(
                WideArithmetic.AddSigned192(Signed192.Raw(depth), Signed192.Raw(depth)),
                Signed192.Signed(1L));
            int comparison = CompareRadialDepthToTwiceRaw(
                radius,
                squaredDistance,
                denominator,
                lowerMidpoint);
            depth = Fixed64.FromRaw(
                depth.m_rawValue
                - GetLowerMidpointAdjustment(
                    comparison,
                    depth.m_rawValue));
        }

        Signed192 upperMidpoint = WideArithmetic.AddSigned192(
            WideArithmetic.AddSigned192(Signed192.Raw(depth), Signed192.Raw(depth)),
            Signed192.Signed(1L));
        int upperComparison = CompareRadialDepthToTwiceRaw(
            radius,
            squaredDistance,
            denominator,
            upperMidpoint);
        long remainingRaw = radius.m_rawValue - depth.m_rawValue;
        long hasRoom =
            (long)((ulong)(remainingRaw | -remainingRaw) >> 63);
        depth = Fixed64.FromRaw(
            depth.m_rawValue
            + (GetUpperMidpointAdjustment(
                upperComparison,
                depth.m_rawValue) & hasRoom));
    }

    private static long GetLowerMidpointAdjustment(
        int comparison,
        long depthRaw)
    {
        int negative = (int)((uint)comparison >> 31);
        int nonzero = (int)((uint)(comparison | -comparison) >> 31);
        int zero = nonzero ^ 1;
        int oddTie = zero & (int)(depthRaw & 1L);
        return negative | oddTie;
    }

    private static long GetUpperMidpointAdjustment(
        int comparison,
        long depthRaw)
    {
        int positive = (int)((uint)-comparison >> 31);
        int nonzero = (int)((uint)(comparison | -comparison) >> 31);
        int zero = nonzero ^ 1;
        int oddTie = zero & (int)(depthRaw & 1L);
        return positive | oddTie;
    }

    private static int CompareRadialDepthToTwiceRaw(
        Fixed64 radius,
        Signed832 squaredDistance,
        Signed320 denominator,
        Signed192 twiceRaw)
    {
        Signed192 twiceRadius = WideArithmetic.AddSigned192(
            Signed192.Raw(radius),
            Signed192.Raw(radius));
        Signed192 remaining = WideArithmetic.SubtractSigned192(
            twiceRadius,
            twiceRaw);
        Signed576 threshold = WideArithmetic.MultiplySigned320(
            Signed320.ExtendValue(remaining),
            denominator);
        Signed832 thresholdSquared =
            WideArithmetic.MultiplySigned576ToSigned832(threshold, threshold);
        Signed832 fourSquaredDistance = WideArithmetic.AddSigned832(
            WideArithmetic.AddSigned832(squaredDistance, squaredDistance),
            WideArithmetic.AddSigned832(squaredDistance, squaredDistance));
        return WideArithmetic.SubtractSigned832(
            thresholdSquared,
            fourSquaredDistance).Sign;
    }
}
