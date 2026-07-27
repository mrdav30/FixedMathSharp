//=======================================================================
// WideOrientedBox.CircleSlab.cs
//=======================================================================
// MIT License, Copyright (c) 2024–present David Oravsky (mrdav30)
// See LICENSE file in the project root for full license information.
//=======================================================================

using System;

namespace FixedMathSharp.Geometry;

/// <content>
/// Provides SIMD-friendly separating-axis tests for detecting contact between
/// an oriented box and a circle constrained to a slab (thickened plane).
/// </content>
internal static partial class WideOrientedBox
{
    internal static bool TryGetCircleSlabContact(
        Vector3d center,
        FixedQuaternion orientation,
        Vector3d halfExtents,
        Vector3d slabCenter,
        Fixed64 circleFrameRotation,
        Fixed64 slabHalfThickness,
        Fixed64 radius,
        out FixedContactAnchors contact)
    {
        RationalBasis basis = new(orientation);
        var best = default(RadialPenetration);
        WideAxis3 up = new(default, Signed320.One, default);
        if (!TryKeepCircleSlabAxis(
                up,
                center,
                halfExtents,
                basis,
                slabCenter,
                slabHalfThickness,
                radius,
                ref best))
        {
            contact = default;
            return false;
        }

        Span<WideAxis3> boxAxes = stackalloc WideAxis3[3]
        {
            new(
                Signed320.ExtendValue(basis.Xx),
                Signed320.ExtendValue(basis.Xy),
                Signed320.ExtendValue(basis.Xz)),
            new(
                Signed320.ExtendValue(basis.Yx),
                Signed320.ExtendValue(basis.Yy),
                Signed320.ExtendValue(basis.Yz)),
            new(
                Signed320.ExtendValue(basis.Zx),
                Signed320.ExtendValue(basis.Zy),
                Signed320.ExtendValue(basis.Zz)),
        };
        for (int index = 0; index < boxAxes.Length; index++)
        {
            if (!TryKeepCircleSlabAxis(
                    boxAxes[index],
                    center,
                    halfExtents,
                    basis,
                    slabCenter,
                    slabHalfThickness,
                    radius,
                    ref best)
                || !TryKeepCircleSlabAxis(
                    CrossWithUp(boxAxes[index]),
                    center,
                    halfExtents,
                    basis,
                    slabCenter,
                    slabHalfThickness,
                    radius,
                    ref best))
            {
                contact = default;
                return false;
            }
        }

        for (int cornerIndex = 0; cornerIndex < FixedOrientedBox.CornerCount; cornerIndex++)
        {
            Vector3d localCorner = new(
                (cornerIndex & 1) == 0 ? -halfExtents.X : halfExtents.X,
                (cornerIndex & 2) == 0 ? -halfExtents.Y : halfExtents.Y,
                (cornerIndex & 4) == 0 ? -halfExtents.Z : halfExtents.Z);
            WideAxis3 cornerAxis = GetCornerToOriginAxis(
                center,
                slabCenter,
                basis,
                localCorner);
            if (!TryKeepCircleSlabAxis(
                    cornerAxis,
                    center,
                    halfExtents,
                    basis,
                    slabCenter,
                    slabHalfThickness,
                    radius,
                    ref best))
            {
                contact = default;
                return false;
            }
        }

        WideAxis3 orientedAxis = best.Negate ? -best.Axis : best.Axis;
        Vector3d normal = WideGeometry.GetNormalized(
            Signed576.ExtendValue(orientedAxis.X),
            Signed576.ExtendValue(orientedAxis.Y),
            Signed576.ExtendValue(orientedAxis.Z));
        Vector3d boxLocalPoint = GetCircleSlabBoxContactPoint(
            center,
            orientation,
            halfExtents,
            slabCenter,
            basis,
            orientedAxis);
        FixedPointAnchor slabAnchor = GetCircleSlabSupportAnchor(
            slabCenter,
            orientedAxis,
            circleFrameRotation,
            slabHalfThickness,
            radius);
        contact = new FixedContactAnchors(
            new FixedPointAnchor(center, orientation, boxLocalPoint),
            slabAnchor,
            normal,
            best.Depth,
            best.DepthIsClamped);
        return true;
    }

    private static Vector3d GetCircleSlabBoxContactPoint(
        Vector3d boxCenter,
        FixedQuaternion boxOrientation,
        Vector3d halfExtents,
        Vector3d slabCenter,
        RationalBasis basis,
        WideAxis3 boxToSlabAxis)
    {
        var slabCenterAnchor = new FixedPointAnchor(
            slabCenter,
            FixedQuaternion.Identity,
            Vector3d.Zero);
        if (!slabCenterAnchor.TryGetLocalPointIn(
                boxCenter,
                boxOrientation,
                out Vector3d localCenter))
        {
            return GetLocalAxisSupportPoint(
                basis,
                halfExtents,
                boxToSlabAxis);
        }

        bool outsideX = localCenter.X < -halfExtents.X
            || localCenter.X > halfExtents.X;
        bool outsideY = localCenter.Y < -halfExtents.Y
            || localCenter.Y > halfExtents.Y;
        bool outsideZ = localCenter.Z < -halfExtents.Z
            || localCenter.Z > halfExtents.Z;
        Vector3d closest = new(
            FixedMath.Clamp(
                localCenter.X,
                -halfExtents.X,
                halfExtents.X),
            FixedMath.Clamp(
                localCenter.Y,
                -halfExtents.Y,
                halfExtents.Y),
            FixedMath.Clamp(
                localCenter.Z,
                -halfExtents.Z,
                halfExtents.Z));
        if (outsideX || outsideY || outsideZ)
            return closest;

        Signed576 localNormalX = GetBasisProjection(
            boxToSlabAxis,
            basis.Xx,
            basis.Xy,
            basis.Xz);
        Signed576 localNormalY = GetBasisProjection(
            boxToSlabAxis,
            basis.Yx,
            basis.Yy,
            basis.Yz);
        Signed576 localNormalZ = GetBasisProjection(
            boxToSlabAxis,
            basis.Zx,
            basis.Zy,
            basis.Zz);
        Signed576 magnitudeX = GetMagnitude(localNormalX);
        Signed576 magnitudeY = GetMagnitude(localNormalY);
        Signed576 magnitudeZ = GetMagnitude(localNormalZ);
        if (CompareSigned(magnitudeX, magnitudeY) >= 0
            && CompareSigned(magnitudeX, magnitudeZ) >= 0)
        {
            closest.X = localNormalX.Sign < 0
                ? -halfExtents.X
                : halfExtents.X;
        }
        else if (CompareSigned(magnitudeY, magnitudeZ) >= 0)
        {
            closest.Y = localNormalY.Sign < 0
                ? -halfExtents.Y
                : halfExtents.Y;
        }
        else
        {
            closest.Z = localNormalZ.Sign < 0
                ? -halfExtents.Z
                : halfExtents.Z;
        }

        return closest;
    }

    private static bool TryKeepCircleSlabAxis(
        WideAxis3 axis,
        Vector3d boxCenter,
        Vector3d halfExtents,
        RationalBasis basis,
        Vector3d slabCenter,
        Fixed64 slabHalfThickness,
        Fixed64 radius,
        ref RadialPenetration best)
    {
        if (axis.IsZero)
            return true;

        Signed576 boxRadius = GetBoxProjectionRadiusNumerator(
            axis,
            halfExtents,
            basis);
        Signed576 centerProjection = GetDifferenceProjection(
            slabCenter,
            boxCenter,
            axis);
        bool negate = centerProjection.Sign < 0;
        Signed576 centerMagnitude = GetMagnitude(centerProjection);
        Signed576 verticalRadius = WideArithmetic.MultiplySigned576(
            Signed576.ExtendValue(GetMagnitude(axis.Y)),
            Signed192.Raw(slabHalfThickness));
        Signed576 rational = WideArithmetic.AddSigned576(
            boxRadius,
            WideArithmetic.MultiplySigned576(
                WideArithmetic.SubtractSigned576(
                    verticalRadius,
                    centerMagnitude),
                basis.Denominator));
        Signed576 planarSquared = WideArithmetic.AddSigned576(
            WideArithmetic.MultiplySigned320(axis.X, axis.X),
            WideArithmetic.MultiplySigned320(axis.Z, axis.Z));
        Signed320 radiusSquared = WideArithmetic.MultiplySigned192(
            Signed192.Raw(radius),
            Signed192.Raw(radius));
        Signed832 radialSquared = WideArithmetic.MultiplySigned576ToSigned832(
            Signed576.ExtendValue(radiusSquared),
            planarSquared);
        if (rational.Sign < 0
            && !IsRadicalAtLeastRatio(
                radialSquared,
                WideArithmetic.SubtractSigned576(default, rational),
                basis.Denominator))
        {
            return false;
        }

        Signed576 squaredAxisLength = GetSquaredAxisLength(axis);
        GetRadialDepth(
            rational,
            radialSquared,
            squaredAxisLength,
            basis.Denominator,
            out Fixed64 depth,
            out bool depthIsClamped);
        if (!best.HasValue
            || depth < best.Depth
            || (depth == best.Depth
                && WideArithmetic.CompareRadialProjectionDepths(
                    rational,
                    radialSquared,
                    ToSigned576(Signed192.Signed(1L)),
                    squaredAxisLength,
                    best.Rational,
                    best.RadialNumerator,
                    best.RadialDenominator,
                    best.AxisSquared,
                    basis.Denominator) < 0))
        {
            best = new RadialPenetration(
                axis,
                negate,
                depth,
                depthIsClamped,
                rational,
                radialSquared,
                ToSigned576(Signed192.Signed(1L)),
                squaredAxisLength);
        }
        return true;
    }

    private static bool IsRadicalAtLeastRatio(
        Signed832 radicalSquared,
        Signed576 ratioNumerator,
        Signed192 ratioDenominator)
    {
        Span<ulong> radicalMagnitude = stackalloc ulong[13];
        Span<ulong> denominatorMagnitude = stackalloc ulong[3];
        Span<ulong> denominatorSquared = stackalloc ulong[6];
        Span<ulong> left = stackalloc ulong[TriangleSweepMagnitudeWords];
        Span<ulong> numeratorMagnitude = stackalloc ulong[9];
        Span<ulong> right = stackalloc ulong[TriangleSweepMagnitudeWords];
        WideArithmetic.GetMagnitude(radicalSquared, radicalMagnitude);
        WideArithmetic.GetMagnitude(
            ratioDenominator,
            out denominatorMagnitude[2],
            out denominatorMagnitude[1],
            out denominatorMagnitude[0]);
        MultiplyMagnitudes(
            denominatorMagnitude,
            denominatorMagnitude,
            denominatorSquared);
        MultiplyMagnitudes(
            radicalMagnitude,
            denominatorSquared,
            left);
        WideArithmetic.GetMagnitude(
            ratioNumerator,
            numeratorMagnitude);
        MultiplyMagnitudes(
            numeratorMagnitude,
            numeratorMagnitude,
            right);
        return CompareMagnitudes(left, right) >= 0;
    }

    private static void GetRadialDepth(
        Signed576 rational,
        Signed832 radialSquared,
        Signed576 squaredAxisLength,
        Signed192 basisDenominator,
        out Fixed64 depth,
        out bool depthIsClamped)
    {
        Signed576 radial = WideArithmetic.GetFloorSquareRootOfProduct(
            radialSquared,
            Signed192.Signed(1L));
        Signed576 numerator = WideArithmetic.AddSigned576(
            rational,
            WideArithmetic.MultiplySigned576(radial, basisDenominator));
        numerator = WideArithmetic.ClampToNonNegative(numerator);
        Signed576 axisLength = WideArithmetic.GetFloorSquareRoot(
            Signed704.ExtendValue(squaredAxisLength));
        Signed576 denominator = WideArithmetic.MultiplySigned576(
            axisLength,
            basisDenominator);
        if (!Fixed64.TryGetSignedRawRatio(numerator, denominator, out depth))
        {
            depth = Fixed64.MaxValue;
            depthIsClamped = true;
            return;
        }

        depthIsClamped = false;
    }

    private static Signed576 GetSquaredAxisLength(WideAxis3 axis) =>
        WideArithmetic.AddSigned576(
            WideArithmetic.AddSigned576(
                WideArithmetic.MultiplySigned320(axis.X, axis.X),
                WideArithmetic.MultiplySigned320(axis.Y, axis.Y)),
            WideArithmetic.MultiplySigned320(axis.Z, axis.Z));

    private static WideAxis3 GetCornerToOriginAxis(
        Vector3d boxCenter,
        Vector3d otherOrigin,
        RationalBasis basis,
        Vector3d localCorner)
    {
        Signed320 cornerX = GetLocalOffsetNumerator(
            basis.Xx,
            basis.Yx,
            basis.Zx,
            localCorner);
        Signed320 cornerZ = GetLocalOffsetNumerator(
            basis.Xz,
            basis.Yz,
            basis.Zz,
            localCorner);
        return new WideAxis3(
            WideArithmetic.SubtractSigned320(
                WideArithmetic.MultiplySigned192(
                    WideArithmetic.SubtractSigned192(
                        Signed192.Raw(otherOrigin.X),
                        Signed192.Raw(boxCenter.X)),
                    basis.Denominator),
                cornerX),
            default,
            WideArithmetic.SubtractSigned320(
                WideArithmetic.MultiplySigned192(
                    WideArithmetic.SubtractSigned192(
                        Signed192.Raw(otherOrigin.Z),
                        Signed192.Raw(boxCenter.Z)),
                    basis.Denominator),
                cornerZ));
    }

    private static Signed320 GetLocalOffsetNumerator(
        Signed192 axisX,
        Signed192 axisY,
        Signed192 axisZ,
        Vector3d localOffset) =>
        WideArithmetic.AddSigned320(
            WideArithmetic.AddSigned320(
                WideArithmetic.MultiplySigned192(
                    Signed192.Raw(localOffset.X),
                    axisX),
                WideArithmetic.MultiplySigned192(
                    Signed192.Raw(localOffset.Y),
                    axisY)),
            WideArithmetic.MultiplySigned192(
                Signed192.Raw(localOffset.Z),
                axisZ));

    private static FixedPointAnchor GetCircleSlabSupportAnchor(
        Vector3d slabCenter,
        WideAxis3 boxToSlabAxis,
        Fixed64 circleFrameRotation,
        Fixed64 halfThickness,
        Fixed64 radius)
    {
        Vector2d localTowardBox = GetLocalPlanarSupportDirection(
            boxToSlabAxis,
            circleFrameRotation);
        Vector3d localRadialDirection = new(
            localTowardBox.X,
            Fixed64.Zero,
            localTowardBox.Y);
        Vector3d roundedRadialOffset =
            localRadialDirection * radius;
        Vector3d localPoint = new(
            Fixed64.Zero,
            boxToSlabAxis.Y.Sign >= 0 ? -halfThickness : halfThickness,
            Fixed64.Zero);
        return new FixedPointAnchor(
            slabCenter,
            FixedQuaternion.FromAxisAngle(
                Vector3d.Up,
                -circleFrameRotation),
            localPoint,
            roundedRadialOffset,
            FixedPointAnchorTerm3d.CreateRadialSupport(
                localRadialDirection,
                radius,
                roundedRadialOffset));
    }

    private static Vector2d GetLocalPlanarSupportDirection(
        WideAxis3 boxToSlabAxis,
        Fixed64 frameRotation)
    {
        Signed576 worldX = WideArithmetic.SubtractSigned576(
            default,
            Signed576.ExtendValue(boxToSlabAxis.X));
        Signed576 worldY = WideArithmetic.SubtractSigned576(
            default,
            Signed576.ExtendValue(boxToSlabAxis.Z));
        Signed192 cosine = Signed192.Raw(FixedMath.Cos(frameRotation));
        Signed192 sine = Signed192.Raw(FixedMath.Sin(frameRotation));
        Signed576 localX = WideArithmetic.AddSigned576(
            WideArithmetic.MultiplySigned576(worldX, cosine),
            WideArithmetic.MultiplySigned576(worldY, sine));
        Signed576 localY = WideArithmetic.SubtractSigned576(
            WideArithmetic.MultiplySigned576(worldY, cosine),
            WideArithmetic.MultiplySigned576(worldX, sine));
        return WideGeometry.GetNormalized(localX, localY);
    }
}
