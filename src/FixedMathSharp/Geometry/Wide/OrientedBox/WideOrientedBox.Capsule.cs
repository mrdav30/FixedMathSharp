//=======================================================================
// WideOrientedBox.Capsule.cs
//=======================================================================
// MIT License, Copyright (c) 2024–present David Oravsky (mrdav30)
// See LICENSE file in the project root for full license information.
//=======================================================================

using System;

namespace FixedMathSharp.Geometry;

/// <content>
/// Contains oriented-box vs capsule contact generation logic using wide (extended-precision) arithmetic.
/// </content>
internal static partial class WideOrientedBox
{
    #region Nested Types

    private readonly struct CapsulePenetration
    {
        internal readonly CapsuleAxis3 Axis;
        internal readonly bool Negate;
        internal readonly Signed704 WideRational;
        internal readonly Signed832 WideSquaredAxisLength;
        internal readonly int FeatureRank;

        internal CapsulePenetration(
            CapsuleAxis3 axis,
            bool negate,
            Signed704 rational,
            Signed832 squaredAxisLength,
            int featureRank)
        {
            Axis = axis;
            Negate = negate;
            WideRational = rational;
            WideSquaredAxisLength = squaredAxisLength;
            FeatureRank = featureRank;
            HasValue = true;
        }

        internal bool HasValue { get; }
    }

    #endregion

    internal static bool TryGetCenteredCapsuleContact(
        Vector3d center,
        FixedQuaternion orientation,
        Vector3d halfExtents,
        Vector3d capsuleCenter,
        FixedQuaternion capsuleRotation,
        Vector3d localCapsuleAxisDirection,
        Fixed64 capsuleAxisLength,
        Fixed64 capsuleRadius,
        out FixedContactAnchors contact)
    {
        GetRotatedLocalAxisNumerators(
            capsuleRotation,
            localCapsuleAxisDirection,
            out Signed192 capsuleAxisX,
            out Signed192 capsuleAxisY,
            out Signed192 capsuleAxisZ,
            out Signed192 capsuleAxisDenominator);
        WideAxis3 capsuleAxis = new(
            Signed320.ExtendValue(capsuleAxisX),
            Signed320.ExtendValue(capsuleAxisY),
            Signed320.ExtendValue(capsuleAxisZ));
        if (!TryGetCenteredCapsulePenetration(
                center,
                orientation,
                halfExtents,
                capsuleCenter,
                capsuleAxis,
                capsuleAxisDenominator,
                capsuleAxisLength,
                capsuleRadius,
                out WideRationalBasis3d basis,
                out Signed192 commonDenominator,
                out CapsulePenetration best))
        {
            contact = default;
            return false;
        }

        GetWideCapsuleDepth(
            best.WideRational,
            best.WideSquaredAxisLength,
            commonDenominator,
            capsuleRadius,
            out Fixed64 depth,
            out bool depthIsClamped);
        CapsuleAxis3 orientedAxis = best.Negate ? -best.Axis : best.Axis;
        Vector3d normal = WideNormalization.GetNormalized(
            orientedAxis.X,
            orientedAxis.Y,
            orientedAxis.Z);
        FixedPointAnchor capsuleAnchor = GetMatchedCapsuleSupportAnchor(
            center,
            capsuleCenter,
            capsuleRotation,
            localCapsuleAxisDirection,
            capsuleAxisLength,
            capsuleRadius,
            normal,
            !GetCapsuleAxisProjection(orientedAxis, capsuleAxis).IsZero);
        Vector3d boxLocalPoint = GetMatchedBoxSupportLocalPoint(
            basis,
            halfExtents,
            center,
            orientation,
            capsuleAnchor,
            orientedAxis);
        contact = new FixedContactAnchors(
            new FixedPointAnchor(
                center,
                orientation,
                boxLocalPoint),
            capsuleAnchor,
            normal,
            depth,
            depthIsClamped);
        return true;
    }

    /// <summary>
    /// Returns whether the exact SAT admits a possible box/capsule overlap.
    /// Unrepresentable edge-axis witnesses are conservatively treated as
    /// inconclusive rather than as separation.
    /// </summary>
    internal static bool CanOverlapCenteredCapsule(
        Vector3d center,
        FixedQuaternion orientation,
        Vector3d halfExtents,
        Vector3d capsuleCenter,
        Vector3d capsuleAxisDirection,
        Fixed64 capsuleAxisLength,
        Fixed64 capsuleRadius)
    {
        WideAxis3 capsuleAxis = new(
            Signed320.ExtendValue(
                Signed192.Raw(capsuleAxisDirection.X)),
            Signed320.ExtendValue(
                Signed192.Raw(capsuleAxisDirection.Y)),
            Signed320.ExtendValue(
                Signed192.Raw(capsuleAxisDirection.Z)));
        return
        TryGetCenteredCapsulePenetration(
            center,
            orientation,
            halfExtents,
            capsuleCenter,
            capsuleAxis,
            Signed192.Signed(1L),
            capsuleAxisLength,
            capsuleRadius,
            out _,
            out _,
            out _);
    }

    internal static bool CanOverlapCenteredCapsule(
        Vector3d center,
        FixedQuaternion orientation,
        Vector3d halfExtents,
        Vector3d capsuleCenter,
        FixedQuaternion capsuleRotation,
        Vector3d localCapsuleAxisDirection,
        Fixed64 capsuleAxisLength,
        Fixed64 capsuleRadius)
    {
        GetRotatedLocalAxisNumerators(
            capsuleRotation,
            localCapsuleAxisDirection,
            out Signed192 capsuleAxisX,
            out Signed192 capsuleAxisY,
            out Signed192 capsuleAxisZ,
            out Signed192 capsuleAxisDenominator);
        WideAxis3 capsuleAxis = new(
            Signed320.ExtendValue(capsuleAxisX),
            Signed320.ExtendValue(capsuleAxisY),
            Signed320.ExtendValue(capsuleAxisZ));
        return TryGetCenteredCapsulePenetration(
            center,
            orientation,
            halfExtents,
            capsuleCenter,
            capsuleAxis,
            capsuleAxisDenominator,
            capsuleAxisLength,
            capsuleRadius,
            out _,
            out _,
            out _);
    }

    private static bool TryGetCenteredCapsulePenetration(
        Vector3d center,
        FixedQuaternion orientation,
        Vector3d halfExtents,
        Vector3d capsuleCenter,
        WideAxis3 capsuleAxis,
        Signed192 capsuleAxisDenominator,
        Fixed64 capsuleAxisLength,
        Fixed64 capsuleRadius,
        out WideRationalBasis3d basis,
        out Signed192 commonDenominator,
        out CapsulePenetration best)
    {
        basis = new WideRationalBasis3d(orientation);
        Signed576 commonDenominatorWide = WideArithmetic.MultiplySigned576(
            Signed576.ExtendValue(
                WideArithmetic.MultiplySigned192(
                    basis.Denominator,
                    capsuleAxisDenominator)),
            Signed192.Raw(Fixed64.Two));
        // Each normalized rigid-frame denominator needs at most 68 signed
        // bits. Their product plus the Q32.32 factor for two needs at most
        // 170 signed bits, so this narrowing is exact.
        _ = Signed192.TryNarrowSigned(
            commonDenominatorWide,
            out commonDenominator);
        Span<WideAxis3> boxAxes = stackalloc WideAxis3[3]
        {
            GetBasisAxis(basis, 2),
            GetBasisAxis(basis, 1),
            GetBasisAxis(basis, 0),
        };
        best = default;
        for (int index = 0; index < boxAxes.Length; index++)
        {
            if (!TryKeepCapsuleAxis(
                    boxAxes[index],
                    center,
                    halfExtents,
                    basis,
                    commonDenominator,
                    capsuleCenter,
                    capsuleAxis,
                    capsuleAxisDenominator,
                    capsuleAxisLength,
                    capsuleRadius,
                    0,
                    ref best)
                || !TryKeepCapsuleAxis(
                    Cross(boxAxes[index], capsuleAxis),
                    center,
                    halfExtents,
                    basis,
                    commonDenominator,
                    capsuleCenter,
                    capsuleAxis,
                    capsuleAxisDenominator,
                    capsuleAxisLength,
                    capsuleRadius,
                    1,
                    ref best))
            {
                return false;
            }
        }

        for (int vertexIndex = 0;
            vertexIndex < FixedOrientedBox.CornerCount;
            vertexIndex++)
        {
            Vector3d localVertex = new(
                (vertexIndex & 1) == 0
                    ? -halfExtents.X
                    : halfExtents.X,
                (vertexIndex & 2) == 0
                    ? -halfExtents.Y
                    : halfExtents.Y,
                (vertexIndex & 4) == 0
                    ? -halfExtents.Z
                    : halfExtents.Z);
            if (!TryKeepCapsuleAxis(
                    GetVertexToCapsuleAxis(
                        center,
                        capsuleCenter,
                        basis,
                        localVertex,
                        capsuleAxis,
                        capsuleAxisDenominator,
                        capsuleAxisLength),
                    center,
                    halfExtents,
                    basis,
                    commonDenominator,
                    capsuleCenter,
                    capsuleAxis,
                    capsuleAxisDenominator,
                    capsuleAxisLength,
                    capsuleRadius,
                    1,
                    ref best))
            {
                return false;
            }
        }

        for (int edgeAxisIndex = 0; edgeAxisIndex < 3; edgeAxisIndex++)
        {
            for (int firstSign = -1; firstSign <= 1; firstSign += 2)
            {
                for (int secondSign = -1;
                    secondSign <= 1;
                    secondSign += 2)
                {
                    for (int endpointSign = -1;
                        endpointSign <= 1;
                        endpointSign += 2)
                    {
                        if (!TryKeepCapsuleAxis(
                                GetCapsuleEndpointToBoxEdgeAxis(
                                    center,
                                    capsuleCenter,
                                    basis,
                                    halfExtents,
                                    edgeAxisIndex,
                                    firstSign,
                                    secondSign,
                                    endpointSign,
                                    capsuleAxis,
                                    capsuleAxisDenominator,
                                    capsuleAxisLength),
                                center,
                                halfExtents,
                                basis,
                                commonDenominator,
                                capsuleCenter,
                                capsuleAxis,
                                capsuleAxisDenominator,
                                capsuleAxisLength,
                                capsuleRadius,
                                1,
                                ref best))
                        {
                            return false;
                        }
                    }
                }
            }
        }

        return true;
    }

    private static Vector3d GetLocalEdgeCenter(
        Vector3d halfExtents,
        int edgeAxisIndex,
        int firstSign,
        int secondSign) =>
        edgeAxisIndex switch
        {
            0 => new Vector3d(
                Fixed64.Zero,
                firstSign * halfExtents.Y,
                secondSign * halfExtents.Z),
            1 => new Vector3d(
                firstSign * halfExtents.X,
                Fixed64.Zero,
                secondSign * halfExtents.Z),
            _ => new Vector3d(
                firstSign * halfExtents.X,
                secondSign * halfExtents.Y,
                Fixed64.Zero),
        };

    private static Vector3d GetMatchedBoxSupportLocalPoint(
        WideRationalBasis3d basis,
        Vector3d halfExtents,
        Vector3d boxCenter,
        FixedQuaternion boxRotation,
        in FixedPointAnchor otherAnchor,
        WideAxis3 featureAxis)
        => GetMatchedBoxSupportLocalPoint(
            basis,
            halfExtents,
            boxCenter,
            boxRotation,
            otherAnchor,
            GetBasisProjection(
                featureAxis,
                basis.Xx,
                basis.Xy,
                basis.Xz),
            GetBasisProjection(
                featureAxis,
                basis.Yx,
                basis.Yy,
                basis.Yz),
            GetBasisProjection(
                featureAxis,
                basis.Zx,
                basis.Zy,
                basis.Zz));

    private static Vector3d GetMatchedBoxSupportLocalPoint(
        WideRationalBasis3d basis,
        Vector3d halfExtents,
        Vector3d boxCenter,
        FixedQuaternion boxRotation,
        in FixedPointAnchor otherAnchor,
        Signed576 xProjection,
        Signed576 yProjection,
        Signed576 zProjection)
    {
        var boxOrigin = new FixedPointAnchor(
            boxCenter,
            FixedQuaternion.Identity,
            Vector3d.Zero);
        bool hasTargetLocalPoint = otherAnchor.TryGetLocalPointIn(
            boxCenter,
            boxRotation,
            out Vector3d targetLocalPoint);
        Fixed64 localX = GetMatchedSupportCoordinate(
            otherAnchor,
            boxOrigin,
            GetAxis(
                basis.Xx,
                basis.Xy,
                basis.Xz,
                basis.Denominator,
                false),
            halfExtents.X,
            xProjection);
        Fixed64 localY = GetMatchedSupportCoordinate(
            otherAnchor,
            boxOrigin,
            GetAxis(
                basis.Yx,
                basis.Yy,
                basis.Yz,
                basis.Denominator,
                false),
            halfExtents.Y,
            yProjection);
        Fixed64 localZ = GetMatchedSupportCoordinate(
            otherAnchor,
            boxOrigin,
            GetAxis(
                basis.Zx,
                basis.Zy,
                basis.Zz,
                basis.Denominator,
                false),
            halfExtents.Z,
            zProjection);
        if (!hasTargetLocalPoint)
            return new Vector3d(localX, localY, localZ);
        if (xProjection.IsZero)
        {
            localX = FixedMath.Clamp(
                targetLocalPoint.X,
                -halfExtents.X,
                halfExtents.X);
        }
        if (yProjection.IsZero)
        {
            localY = FixedMath.Clamp(
                targetLocalPoint.Y,
                -halfExtents.Y,
                halfExtents.Y);
        }
        if (zProjection.IsZero)
        {
            localZ = FixedMath.Clamp(
                targetLocalPoint.Z,
                -halfExtents.Z,
                halfExtents.Z);
        }
        return new Vector3d(localX, localY, localZ);
    }

    private static FixedPointAnchor GetMatchedCapsuleSupportAnchor(
        Vector3d boxCenter,
        Vector3d capsuleCenter,
        FixedQuaternion capsuleRotation,
        Vector3d localCapsuleAxisDirection,
        Fixed64 capsuleAxisLength,
        Fixed64 capsuleRadius,
        Vector3d boxToCapsuleNormal,
        bool hasAxialProjection)
    {
        if (hasAxialProjection)
        {
            return WideGeometry.GetCenteredCapsuleSupportAnchor(
                capsuleCenter,
                capsuleRotation,
                localCapsuleAxisDirection,
                capsuleAxisLength,
                capsuleRadius,
                -boxToCapsuleNormal);
        }

        Vector3d localNormal = WideNormalization.GetNormalized(
            capsuleRotation.Inverse().Rotate(-boxToCapsuleNormal));
        return FixedSegment.GetSurfaceAnchorOnCenteredCapsule(
            boxCenter,
            capsuleCenter,
            capsuleRotation,
            localCapsuleAxisDirection,
            capsuleAxisLength,
            capsuleRadius,
            localNormal);
    }

    private static Fixed64 GetMatchedSupportCoordinate(
        in FixedPointAnchor targetAnchor,
        in FixedPointAnchor boxOrigin,
        Vector3d boxAxis,
        Fixed64 extent,
        Signed576 directionProjection)
    {
        bool isFree = directionProjection.IsZero;
        if (!isFree && directionProjection.Sign > 0)
            return extent;
        if (!isFree && directionProjection.Sign < 0)
            return -extent;

        WidePointAnchor3d.GetProjectedOffsetRatio(
            targetAnchor,
            boxOrigin,
            boxAxis,
            out Signed704 numerator,
            out Signed704 denominator);
        // A free coordinate belongs to the retained SAT feature. If its
        // matched capsule point lay outside this box slab, the corresponding
        // cross or endpoint-edge candidate would replace that feature.
        _ = Fixed64.TryGetSignedRawRatio(
            numerator,
            denominator,
            out Fixed64 target);
        return FixedMath.Clamp(target, -extent, extent);
    }

    private static bool TryKeepCapsuleAxis(
        WideAxis3 axis,
        Vector3d boxCenter,
        Vector3d halfExtents,
        WideRationalBasis3d basis,
        Signed192 commonDenominator,
        Vector3d capsuleCenter,
        WideAxis3 capsuleAxis,
        Signed192 capsuleAxisDenominator,
        Fixed64 capsuleAxisLength,
        Fixed64 capsuleRadius,
        int featureRank,
        ref CapsulePenetration best)
        => TryKeepCapsuleAxis(
            ToCapsuleAxis(axis),
            boxCenter,
            halfExtents,
            basis,
            commonDenominator,
            capsuleCenter,
            capsuleAxis,
            capsuleAxisDenominator,
            capsuleAxisLength,
            capsuleRadius,
            featureRank,
            ref best);

}
