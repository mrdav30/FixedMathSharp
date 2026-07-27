//=======================================================================
// WideOrientedBox.CapsuleSlab.cs
//=======================================================================
// MIT License, Copyright (c) 2024–present David Oravsky (mrdav30)
// See LICENSE file in the project root for full license information.
//=======================================================================

using System;

namespace FixedMathSharp.Bounds;

/// <content>
/// Wide-precision contact generation between an oriented box and a capsule-slab pair.
/// </content>
internal static partial class WideOrientedBox
{
    internal static bool TryGetCenteredCapsuleSlabContact(
        Vector3d center,
        FixedQuaternion orientation,
        Vector3d halfExtents,
        Vector3d slabCenter,
        Fixed64 capsuleFrameRotation,
        Vector2d localCapsuleAxisDirection,
        Fixed64 capsuleAxisLength,
        Fixed64 capsuleRadius,
        Fixed64 slabHalfThickness,
        out FixedContactAnchors contact)
    {
        _ = Vector2d.TryRotate(
            localCapsuleAxisDirection,
            capsuleFrameRotation,
            out Vector2d capsuleAxisDirection);
        RationalBasis basis = new(orientation);
        Signed320 commonDenominatorWide = WideArithmetic.MultiplySigned192(
            basis.Denominator,
            Signed192.Raw(Fixed64.Two));
        // A normalized quaternion bounds the squared-component sum to roughly
        // 2^64 raw units; applying the Q32.32 factor for two remains below 100
        // signed bits.
        Signed192 commonDenominator = new(
            commonDenominatorWide.Word2,
            commonDenominatorWide.Word1,
            commonDenominatorWide.Word0);
        var best = default(RadialPenetration);
        WideAxis3 up = new(default, Signed320.One, default);
        if (!TryKeepCapsuleSlabAxis(
                up,
                center,
                halfExtents,
                basis,
                commonDenominator,
                slabCenter,
                capsuleAxisDirection,
                capsuleAxisLength,
                capsuleRadius,
                slabHalfThickness,
                ref best))
        {
            contact = default;
            return false;
        }

        Span<RationalAxis3> boxAxes = stackalloc RationalAxis3[3]
        {
            new(basis.Xx, basis.Xy, basis.Xz),
            new(basis.Yx, basis.Yy, basis.Yz),
            new(basis.Zx, basis.Zy, basis.Zz),
        };
        WideAxis3 capsuleAxis = new(
            Signed320.ExtendValue(Signed192.Raw(capsuleAxisDirection.X)),
            default,
            Signed320.ExtendValue(Signed192.Raw(capsuleAxisDirection.Y)));
        WideAxis3 capsuleNormal = new(
            Signed320.ExtendValue(Signed192.Raw(capsuleAxisDirection.Y)),
            default,
            Signed320.ExtendValue(
                WideArithmetic.SubtractSigned192(
                    default,
                    Signed192.Raw(capsuleAxisDirection.X))));
        if (!TryKeepCapsuleSlabAxis(
                capsuleAxis,
                center,
                halfExtents,
                basis,
                commonDenominator,
                slabCenter,
                capsuleAxisDirection,
                capsuleAxisLength,
                capsuleRadius,
                slabHalfThickness,
                ref best)
            || !TryKeepCapsuleSlabAxis(
                capsuleNormal,
                center,
                halfExtents,
                basis,
                commonDenominator,
                slabCenter,
                capsuleAxisDirection,
                capsuleAxisLength,
                capsuleRadius,
                slabHalfThickness,
                ref best))
        {
            contact = default;
            return false;
        }

        for (int index = 0; index < boxAxes.Length; index++)
        {
            WideAxis3 boxAxis = boxAxes[index].ToWide();
            if (!TryKeepCapsuleSlabAxis(
                    boxAxis,
                    center,
                    halfExtents,
                    basis,
                    commonDenominator,
                    slabCenter,
                    capsuleAxisDirection,
                    capsuleAxisLength,
                    capsuleRadius,
                    slabHalfThickness,
                    ref best))
            {
                contact = default;
                return false;
            }
            if (!TryKeepCapsuleSlabAxis(
                    CrossWithUp(boxAxis),
                    center,
                    halfExtents,
                    basis,
                    commonDenominator,
                    slabCenter,
                    capsuleAxisDirection,
                    capsuleAxisLength,
                    capsuleRadius,
                    slabHalfThickness,
                    ref best))
            {
                contact = default;
                return false;
            }
            if (!TryKeepCapsuleSlabAxis(
                    CrossWithPlanarEdge(
                        boxAxes[index],
                        Signed192.Raw(capsuleAxisDirection.X),
                        Signed192.Raw(capsuleAxisDirection.Y)),
                    center,
                    halfExtents,
                    basis,
                    commonDenominator,
                    slabCenter,
                    capsuleAxisDirection,
                    capsuleAxisLength,
                    capsuleRadius,
                    slabHalfThickness,
                    ref best))
            {
                contact = default;
                return false;
            }
        }

        for (int cornerIndex = 0;
            cornerIndex < FixedOrientedBox.CornerCount;
            cornerIndex++)
        {
            Vector3d localCorner = new(
                (cornerIndex & 1) == 0 ? -halfExtents.X : halfExtents.X,
                (cornerIndex & 2) == 0 ? -halfExtents.Y : halfExtents.Y,
                (cornerIndex & 4) == 0 ? -halfExtents.Z : halfExtents.Z);
            for (int endpointSign = -1; endpointSign <= 1; endpointSign += 2)
            {
                WideAxis3 endpointAxis = GetCornerToCapsuleEndpointAxis(
                    center,
                    slabCenter,
                    basis,
                    localCorner,
                    capsuleAxisDirection,
                    capsuleAxisLength,
                    endpointSign);
                if (!TryKeepCapsuleSlabAxis(
                        endpointAxis,
                        center,
                        halfExtents,
                        basis,
                        commonDenominator,
                        slabCenter,
                        capsuleAxisDirection,
                        capsuleAxisLength,
                        capsuleRadius,
                        slabHalfThickness,
                        ref best))
                {
                    contact = default;
                    return false;
                }
            }
        }

        WideAxis3 orientedAxis = best.Negate ? -best.Axis : best.Axis;
        Vector3d normal = WideGeometry.GetNormalized(
            Signed576.ExtendValue(orientedAxis.X),
            Signed576.ExtendValue(orientedAxis.Y),
            Signed576.ExtendValue(orientedAxis.Z));
        Vector3d boxLocalPoint =
            GetLocalAxisSupportPoint(basis, halfExtents, orientedAxis);
        FixedPointAnchor slabAnchor = GetCapsuleSlabSupportAnchor(
            slabCenter,
            capsuleFrameRotation,
            localCapsuleAxisDirection,
            orientedAxis,
            capsuleAxisLength,
            capsuleRadius,
            slabHalfThickness);
        contact = new FixedContactAnchors(
            new FixedPointAnchor(center, orientation, boxLocalPoint),
            slabAnchor,
            normal,
            best.Depth,
            best.DepthIsClamped);
        return true;
    }

    private static bool TryKeepCapsuleSlabAxis(
        WideAxis3 axis,
        Vector3d boxCenter,
        Vector3d halfExtents,
        RationalBasis basis,
        Signed192 commonDenominator,
        Vector3d slabCenter,
        Vector2d capsuleAxisDirection,
        Fixed64 capsuleAxisLength,
        Fixed64 capsuleRadius,
        Fixed64 slabHalfThickness,
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
        Signed576 baseRational = WideArithmetic.AddSigned576(
            boxRadius,
            WideArithmetic.MultiplySigned576(
                WideArithmetic.SubtractSigned576(
                    verticalRadius,
                    centerMagnitude),
                basis.Denominator));
        Signed576 rational = WideArithmetic.MultiplySigned576(
            baseRational,
            Signed192.Raw(Fixed64.Two));
        Signed576 alignment = GetMagnitude(
            GetPlanarAxisProjection(axis, capsuleAxisDirection));
        rational = WideArithmetic.AddSigned576(
            rational,
            WideArithmetic.MultiplySigned576(
                WideArithmetic.MultiplySigned576(
                    alignment,
                    Signed192.Raw(capsuleAxisLength)),
                basis.Denominator));

        Signed576 planarSquared = WideArithmetic.AddSigned576(
            WideArithmetic.MultiplySigned320(axis.X, axis.X),
            WideArithmetic.MultiplySigned320(axis.Z, axis.Z));
        Signed320 radiusSquared = WideArithmetic.MultiplySigned192(
            Signed192.Raw(capsuleRadius),
            Signed192.Raw(capsuleRadius));
        Signed832 radialSquared = WideArithmetic.MultiplySigned576ToSigned832(
            Signed576.ExtendValue(radiusSquared),
            planarSquared);
        if (rational.Sign < 0
            && !IsRadicalAtLeastRatio(
                radialSquared,
                WideArithmetic.SubtractSigned576(default, rational),
                commonDenominator))
        {
            return false;
        }

        Signed576 squaredAxisLength = GetSquaredAxisLength(axis);
        GetRadialDepth(
            rational,
            radialSquared,
            squaredAxisLength,
            commonDenominator,
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
                    commonDenominator) < 0))
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

    private static Signed576 GetPlanarAxisProjection(
        WideAxis3 axis,
        Vector2d planarDirection) =>
        WideArithmetic.AddSigned576(
            WideArithmetic.MultiplySigned576(
                Signed576.ExtendValue(axis.X),
                Signed192.Raw(planarDirection.X)),
            WideArithmetic.MultiplySigned576(
                Signed576.ExtendValue(axis.Z),
                Signed192.Raw(planarDirection.Y)));

    private static WideAxis3 GetCornerToCapsuleEndpointAxis(
        Vector3d boxCenter,
        Vector3d capsuleCenter,
        RationalBasis basis,
        Vector3d localCorner,
        Vector2d capsuleAxisDirection,
        Fixed64 capsuleAxisLength,
        int endpointSign)
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
        Signed192 twiceScale = Signed192.Raw(Fixed64.Two);
        Signed576 centerX = WideArithmetic.MultiplySigned576(
            Signed576.ExtendValue(
                WideArithmetic.MultiplySigned192(
                    WideArithmetic.SubtractSigned192(
                        Signed192.Raw(capsuleCenter.X),
                        Signed192.Raw(boxCenter.X)),
                    basis.Denominator)),
            twiceScale);
        Signed576 centerZ = WideArithmetic.MultiplySigned576(
            Signed576.ExtendValue(
                WideArithmetic.MultiplySigned192(
                    WideArithmetic.SubtractSigned192(
                        Signed192.Raw(capsuleCenter.Z),
                        Signed192.Raw(boxCenter.Z)),
                    basis.Denominator)),
            twiceScale);
        Signed576 endpointX = WideArithmetic.MultiplySigned576(
            WideArithmetic.MultiplySigned576(
                Signed576.ExtendValue(
                    Signed320.ExtendValue(
                        Signed192.Raw(capsuleAxisDirection.X))),
                Signed192.Signed(
                    endpointSign * capsuleAxisLength.m_rawValue)),
            basis.Denominator);
        Signed576 endpointZ = WideArithmetic.MultiplySigned576(
            WideArithmetic.MultiplySigned576(
                Signed576.ExtendValue(
                    Signed320.ExtendValue(
                        Signed192.Raw(capsuleAxisDirection.Y))),
                Signed192.Signed(
                    endpointSign * capsuleAxisLength.m_rawValue)),
            basis.Denominator);
        Signed576 cornerXWide = WideArithmetic.MultiplySigned576(
            Signed576.ExtendValue(cornerX),
            twiceScale);
        Signed576 cornerZWide = WideArithmetic.MultiplySigned576(
            Signed576.ExtendValue(cornerZ),
            twiceScale);
        // A full-domain center difference multiplied by a normalized quaternion
        // basis and the Q32.32 factor for two uses at most 165 signed bits.
        return new WideAxis3(
            Signed320.NarrowValue(WideArithmetic.SubtractSigned576(
                WideArithmetic.AddSigned576(centerX, endpointX),
                cornerXWide)),
            default,
            Signed320.NarrowValue(WideArithmetic.SubtractSigned576(
                WideArithmetic.AddSigned576(centerZ, endpointZ),
                cornerZWide)));
    }

    private static FixedPointAnchor GetCapsuleSlabSupportAnchor(
        Vector3d slabCenter,
        Fixed64 capsuleFrameRotation,
        Vector2d localCapsuleAxisDirection,
        WideAxis3 boxToSlabAxis,
        Fixed64 capsuleAxisLength,
        Fixed64 capsuleRadius,
        Fixed64 slabHalfThickness)
    {
        Vector2d localTowardBox = GetLocalPlanarSupportDirection(
            boxToSlabAxis,
            capsuleFrameRotation);
        FixedPointAnchor2d planar =
            FixedSegment2d.GetCenteredCapsuleSupportAnchor(
                Vector2d.Zero,
                capsuleFrameRotation,
                localCapsuleAxisDirection,
                capsuleAxisLength,
                capsuleRadius,
                localTowardBox);
        return new FixedPointAnchor(
            slabCenter,
            FixedQuaternion.FromAxisAngle(
                Vector3d.Up,
                -capsuleFrameRotation),
            new Vector3d(
                planar.LocalPoint.X,
                boxToSlabAxis.Y.Sign >= 0
                    ? -slabHalfThickness
                    : slabHalfThickness,
                planar.LocalPoint.Y),
            new Vector3d(
                planar.LocalDisplacement.X,
                Fixed64.Zero,
                planar.LocalDisplacement.Y),
            FixedPointAnchorTerm3d.LiftPlanarXZ(
                planar.ExactLocalTerm));
    }
}
