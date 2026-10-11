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
    internal static bool DoesCenteredCapsulePenetrate(
        Vector3d center, FixedQuaternion orientation, Vector3d halfExtents,
        Vector3d capsuleCenter, FixedQuaternion capsuleRotation,
        Vector3d localCapsuleAxisDirection, Fixed64 capsuleAxisLength, Fixed64 capsuleRadius)
    {
        if (capsuleAxisLength == Fixed64.Zero)
            return DoesSpherePenetrate(center, orientation, halfExtents, capsuleCenter, capsuleRadius);

        WideRationalBasis3d boxBasis = new(orientation);
        WideRationalBasis3d capsuleBasis = new(capsuleRotation);
        WideRationalBasis3d relativeBasis = WideRationalBasis3d.CreateRelative(boxBasis, capsuleBasis);
        WideAxis3 capsuleAxis = WideRigidProjection.TransformLocalAxis(
            relativeBasis, Signed192.Raw(localCapsuleAxisDirection.X),
            Signed192.Raw(localCapsuleAxisDirection.Y), Signed192.Raw(localCapsuleAxisDirection.Z));
        return TryGetCenteredCapsulePenetration(
                center, halfExtents, capsuleCenter, boxBasis, capsuleBasis.Denominator,
                capsuleAxis, relativeBasis.Denominator,
                GetScaledCapsuleAxisSquared(localCapsuleAxisDirection, capsuleBasis.Denominator),
                GetCapsuleCenterAxisProjection(boxBasis.Denominator, center, capsuleCenter,
                    capsuleBasis, localCapsuleAxisDirection),
                capsuleAxisLength, capsuleRadius, out _, out _, strict: true);
    }

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
        int axialSign = -GetCapsuleAxisProjection(orientedAxis, capsuleAxis).Sign;
        FixedPointAnchor capsuleAnchor = GetMatchedCapsuleSupportAnchor(
            center, capsuleCenter, capsuleRotation, localCapsuleAxisDirection,
            capsuleAxisLength, capsuleRadius, normal, axialSign);
        Vector3d boxLocalPoint = GetMatchedBoxSupportLocalPoint(
            halfExtents, center, orientation, capsuleAnchor,
            orientedAxis.X, orientedAxis.Y, orientedAxis.Z, out bool needsCoreMatch);
        FixedPointAnchor boxAnchor = new(center, orientation, boxLocalPoint);
        int freeCount = (orientedAxis.X.IsZero ? 1 : 0)
            + (orientedAxis.Y.IsZero ? 1 : 0) + (orientedAxis.Z.IsZero ? 1 : 0);
        // Only a face's two free coordinates certify its entire footprint.
        // Edge/vertex constraints require the paired finite-core relation.
        if (axialSign == 0 && capsuleAxisLength > Fixed64.Zero && (needsCoreMatch || freeCount < 2))
        {
            // Compose only the winning support rectangle/edge/vertex; the shared
            // finite-core clip does not repeat SAT or materialize world points.
            Span<Vector3d> support = stackalloc Vector3d[4];
            Span<Signed576> projections = stackalloc Signed576[3]
                { orientedAxis.X, orientedAxis.Y, orientedAxis.Z };
            int count = 1 << freeCount;
            for (int corner = 0; corner < count; corner++)
            {
                Vector3d point = default;
                int freeAxis = 0;
                for (int axis = 0; axis < 3; axis++)
                {
                    bool positive = projections[axis].IsZero
                        ? (corner & (1 << freeAxis++)) != 0 : projections[axis].Sign > 0;
                    point[axis] = positive ? halfExtents[axis] : -halfExtents[axis];
                }
                support[corner] = point;
            }
            ReadOnlySpan<int> triangles = stackalloc int[6] { 0, 1, 3, 0, 3, 2 };
            ReadOnlySpan<int> edges = stackalloc int[8] { 0, 1, 1, 3, 3, 2, 2, 0 };
            boxAnchor = GetMatchedHullCapsuleSupportAnchor(center, orientation, boxBasis,
                support[..count], count == 4 ? triangles : ReadOnlySpan<int>.Empty,
                count == 4 ? edges : count == 2 ? edges[..2] : ReadOnlySpan<int>.Empty,
                new CapsuleAxis3(worldX, worldY, worldZ), capsuleAnchor,
                capsuleCenter, capsuleRotation, localCapsuleAxisDirection,
                capsuleAxisLength, capsuleRadius, normal, true, out capsuleAnchor);
        }
        contact = new FixedContactAnchors(
            boxAnchor,
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
        out CapsulePenetration best,
        bool strict = false)
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
                    ref best,
                    strict)
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
                    ref best,
                    strict))
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
                    ref best,
                    strict))
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
                                ref best,
                                strict))
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
                basis.Zz),
            out _);

    private static Vector3d GetMatchedBoxSupportLocalPoint(
        Vector3d halfExtents,
        Vector3d boxCenter,
        FixedQuaternion boxRotation,
        in FixedPointAnchor otherAnchor,
        Signed576 xProjection,
        Signed576 yProjection,
        Signed576 zProjection,
        out bool needsCoreMatch)
    {
        if (!xProjection.IsZero && !yProjection.IsZero && !zProjection.IsZero)
        {
            needsCoreMatch = false;
            return new Vector3d(xProjection.Sign > 0 ? halfExtents.X : -halfExtents.X,
                yProjection.Sign > 0 ? halfExtents.Y : -halfExtents.Y,
                zProjection.Sign > 0 ? halfExtents.Z : -halfExtents.Z);
        }
        WideTriangleRelations.GetPointLocalNumerators(otherAnchor, boxCenter, boxRotation,
            out Signed576 x, out Signed576 y, out Signed576 z, out Signed320 denominator);
        Fixed64 localX = GetMatchedSupportCoordinate(x, denominator, halfExtents.X,
            xProjection, out bool clipX);
        Fixed64 localY = GetMatchedSupportCoordinate(y, denominator, halfExtents.Y,
            yProjection, out bool clipY);
        Fixed64 localZ = GetMatchedSupportCoordinate(z, denominator, halfExtents.Z,
            zProjection, out bool clipZ);
        needsCoreMatch = clipX || clipY || clipZ;
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
        int axialSign)
    {
        if (axialSign != 0)
        {
            return WideGeometry.GetCenteredCapsuleSupportAnchor(
                capsuleCenter,
                capsuleRotation,
                localCapsuleAxisDirection,
                capsuleAxisLength,
                capsuleRadius,
                -boxToCapsuleNormal,
                axialSign);
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
        Signed576 numerator,
        Signed320 denominator,
        Fixed64 extent,
        Signed576 directionProjection,
        out bool needsCoreMatch)
    {
        needsCoreMatch = false;
        if (directionProjection.Sign != 0)
            return directionProjection.Sign > 0 ? extent : -extent;

        // Certify the exact free-coordinate interval before rounding; projecting
        // onto a rounded box basis could hide a clip near a rotated boundary.
        Signed576 magnitude = numerator.Sign < 0
            ? WideArithmetic.SubtractSigned576(default, numerator) : numerator;
        Signed576 limit = WideArithmetic.MultiplySigned320(denominator,
            Signed320.ExtendValue(Signed192.Raw(extent)));
        needsCoreMatch = WideArithmetic.SubtractSigned576(limit, magnitude).Sign < 0;
        if (needsCoreMatch)
            return numerator.Sign > 0 ? extent : -extent;

        // The bounded free coordinate is representable; round it once.
        _ = Fixed64.TryGetSignedRawRatio(numerator, Signed576.ExtendValue(denominator), out Fixed64 target);
        return target;
    }

}
