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
        WideRationalBasis3d boxBasis = new(orientation);
        WideRationalBasis3d capsuleBasis = new(capsuleRotation);
        WideRationalBasis3d relativeBasis =
            WideRationalBasis3d.CreateRelative(boxBasis, capsuleBasis);
        WideAxis3 capsuleAxis = WideRigidProjection.TransformLocalAxis(
            relativeBasis,
            Signed192.Raw(localCapsuleAxisDirection.X),
            Signed192.Raw(localCapsuleAxisDirection.Y),
            Signed192.Raw(localCapsuleAxisDirection.Z));
        if (!TryGetCenteredCapsulePenetration(
                center,
                halfExtents,
                capsuleCenter,
                boxBasis,
                capsuleBasis.Denominator,
                capsuleAxis,
                relativeBasis.Denominator,
                GetScaledCapsuleAxisSquared(
                    localCapsuleAxisDirection,
                    capsuleBasis.Denominator),
                GetCapsuleCenterAxisProjection(
                    boxBasis.Denominator,
                    center,
                    capsuleCenter,
                    capsuleBasis,
                    localCapsuleAxisDirection),
                capsuleAxisLength,
                capsuleRadius,
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
        Signed576 worldX = Signed576.NarrowValue(
            GetCapsuleBasisProjection(
                orientedAxis,
                boxBasis.Xx,
                boxBasis.Yx,
                boxBasis.Zx));
        Signed576 worldY = Signed576.NarrowValue(
            GetCapsuleBasisProjection(
                orientedAxis,
                boxBasis.Xy,
                boxBasis.Yy,
                boxBasis.Zy));
        Signed576 worldZ = Signed576.NarrowValue(
            GetCapsuleBasisProjection(
                orientedAxis,
                boxBasis.Xz,
                boxBasis.Yz,
                boxBasis.Zz));
        Vector3d normal = WideNormalization.GetNormalized(
            worldX,
            worldY,
            worldZ);
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
            boxBasis,
            halfExtents,
            center,
            orientation,
            capsuleAnchor,
            orientedAxis.X,
            orientedAxis.Y,
            orientedAxis.Z);
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
        WideRationalBasis3d boxBasis = new(orientation);
        GetDirectionProjections(
            capsuleAxisDirection,
            boxBasis,
            out Signed320 capsuleAxisX,
            out Signed320 capsuleAxisY,
            out Signed320 capsuleAxisZ);
        var authoredCapsuleAxis = new WideAxis3(
            Signed320.ExtendValue(Signed192.Raw(capsuleAxisDirection.X)),
            Signed320.ExtendValue(Signed192.Raw(capsuleAxisDirection.Y)),
            Signed320.ExtendValue(Signed192.Raw(capsuleAxisDirection.Z)));
        return TryGetCenteredCapsulePenetration(
            center,
            halfExtents,
            capsuleCenter,
            boxBasis,
            Signed192.Signed(1L),
            new WideAxis3(
                capsuleAxisX,
                capsuleAxisY,
                capsuleAxisZ),
            boxBasis.Denominator,
            authoredCapsuleAxis.SquaredLength,
            WideArithmetic.MultiplySigned576(
                WideRigidProjection.GetWorldOriginDifferenceProjection(
                    capsuleCenter,
                    center,
                    authoredCapsuleAxis),
                boxBasis.Denominator),
            capsuleAxisLength,
            capsuleRadius,
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
        WideRationalBasis3d boxBasis = new(orientation);
        WideRationalBasis3d capsuleBasis = new(capsuleRotation);
        WideRationalBasis3d relativeBasis =
            WideRationalBasis3d.CreateRelative(boxBasis, capsuleBasis);
        WideAxis3 capsuleAxis = WideRigidProjection.TransformLocalAxis(
            relativeBasis,
            Signed192.Raw(localCapsuleAxisDirection.X),
            Signed192.Raw(localCapsuleAxisDirection.Y),
            Signed192.Raw(localCapsuleAxisDirection.Z));
        return TryGetCenteredCapsulePenetration(
            center,
            halfExtents,
            capsuleCenter,
            boxBasis,
            capsuleBasis.Denominator,
            capsuleAxis,
            relativeBasis.Denominator,
            GetScaledCapsuleAxisSquared(
                localCapsuleAxisDirection,
                capsuleBasis.Denominator),
            GetCapsuleCenterAxisProjection(
                boxBasis.Denominator,
                center,
                capsuleCenter,
                capsuleBasis,
                localCapsuleAxisDirection),
            capsuleAxisLength,
            capsuleRadius,
            out _,
            out _);
    }

    private static bool TryGetCenteredCapsulePenetration(
        Vector3d boxCenter,
        Vector3d halfExtents,
        Vector3d capsuleCenter,
        in WideRationalBasis3d boxBasis,
        Signed192 translationScale,
        in WideAxis3 capsuleAxis,
        Signed192 capsuleAxisDenominator,
        Signed576 capsuleAxisSquared,
        Signed576 capsuleCenterAxisProjection,
        Fixed64 capsuleAxisLength,
        Fixed64 capsuleRadius,
        out Signed192 commonDenominator,
        out CapsulePenetration best)
    {
        Signed320 commonDenominatorWide = WideArithmetic.MultiplySigned192(
            capsuleAxisDenominator,
            Signed192.Raw(Fixed64.Two));
        commonDenominator = Signed192.NarrowProven(commonDenominatorWide);
        GetRelativeLocalPointNumerators(
            capsuleCenter,
            boxCenter,
            boxBasis,
            out Signed192 translationX,
            out Signed192 translationY,
            out Signed192 translationZ);
        var translation = new WideAxis3(
            Signed320.ExtendValue(translationX),
            Signed320.ExtendValue(translationY),
            Signed320.ExtendValue(translationZ));
        Signed320 centerScale = WideArithmetic.MultiplySigned192(
            translationScale,
            Signed192.Raw(Fixed64.Two));
        Signed320 axialScale = Signed320.ExtendValue(
            Signed192.Raw(capsuleAxisLength));
        var twiceTranslation = new WideAxis3(
            Signed320.NarrowValue(WideArithmetic.MultiplySigned320(
                translation.X,
                centerScale)),
            Signed320.NarrowValue(WideArithmetic.MultiplySigned320(
                translation.Y,
                centerScale)),
            Signed320.NarrowValue(WideArithmetic.MultiplySigned320(
                translation.Z,
                centerScale)));
        var scaledExtents = new WideAxis3(
            WideArithmetic.MultiplySigned192(
                Signed192.Raw(halfExtents.X),
                commonDenominator),
            WideArithmetic.MultiplySigned192(
                Signed192.Raw(halfExtents.Y),
                commonDenominator),
            WideArithmetic.MultiplySigned192(
                Signed192.Raw(halfExtents.Z),
                commonDenominator));
        Signed704 vertexCapBound =
            WideArithmetic.MultiplySigned576ToSigned704(
                capsuleAxisSquared,
                WideArithmetic.MultiplySigned192(
                    boxBasis.Denominator,
                    Signed192.Raw(capsuleAxisLength)));
        Signed320 unit = Signed320.One;
        Span<WideAxis3> boxAxes = stackalloc WideAxis3[3]
        {
            new(default, default, unit),
            new(default, unit, default),
            new(unit, default, default),
        };
        best = default;
        for (int index = 0; index < boxAxes.Length; index++)
        {
            if (!TryKeepCapsuleAxis(
                    ToCapsuleAxis(boxAxes[index]),
                    scaledExtents,
                    twiceTranslation,
                    commonDenominator,
                    capsuleAxis,
                    axialScale,
                    capsuleRadius,
                    0,
                    ref best)
                || !TryKeepCapsuleAxis(
                    ToCapsuleAxis(WideAxis3.Cross(
                        boxAxes[index],
                        capsuleAxis)),
                    scaledExtents,
                    twiceTranslation,
                    commonDenominator,
                    capsuleAxis,
                    axialScale,
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
                    GetLocalVertexToCapsuleAxis(
                        translation,
                        boxBasis.Denominator,
                        localVertex,
                        capsuleAxis,
                        capsuleAxisSquared,
                        capsuleCenterAxisProjection,
                        centerScale,
                        axialScale,
                        vertexCapBound),
                    scaledExtents,
                    twiceTranslation,
                    commonDenominator,
                    capsuleAxis,
                    axialScale,
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
                    Vector3d localEdgeCenter = GetLocalEdgeCenter(
                        halfExtents,
                        edgeAxisIndex,
                        firstSign,
                        secondSign);
                    WideAxis3 edgeDifference =
                        GetLocalPointToCapsuleCenterAxis(
                            translation,
                            boxBasis.Denominator,
                            localEdgeCenter);
                    Signed320 edgeBound = edgeAxisIndex switch
                    {
                        0 => scaledExtents.X,
                        1 => scaledExtents.Y,
                        _ => scaledExtents.Z,
                    };
                    for (int endpointSign = -1;
                        endpointSign <= 1;
                        endpointSign += 2)
                    {
                        if (!TryKeepCapsuleAxis(
                                GetCapsuleEndpointToBoxEdgeAxis(
                                    edgeDifference,
                                    centerScale,
                                    axialScale,
                                    edgeAxisIndex,
                                    endpointSign,
                                    capsuleAxis,
                                    edgeBound),
                                scaledExtents,
                                twiceTranslation,
                                commonDenominator,
                                capsuleAxis,
                                axialScale,
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
            WideRigidProjection.GetBasisAxisProjection(
                featureAxis,
                basis.Xx,
                basis.Xy,
                basis.Xz),
            WideRigidProjection.GetBasisAxisProjection(
                featureAxis,
                basis.Yx,
                basis.Yy,
                basis.Yz),
            WideRigidProjection.GetBasisAxisProjection(
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

}
