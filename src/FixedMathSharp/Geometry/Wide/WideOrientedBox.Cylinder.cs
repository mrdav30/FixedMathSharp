//=======================================================================
// WideOrientedBox.Cylinder.cs
//=======================================================================
// MIT License, Copyright (c) 2024–present David Oravsky (mrdav30)
// See LICENSE file in the project root for full license information.
//=======================================================================

using System;

namespace FixedMathSharp.Geometry;

/// <content>
/// Cylinder-vs-oriented-box intersection and penetration logic, including
/// support types for tracking axis-aligned penetration depth and contact
/// features between a box face and a cylinder cap.
/// </content>
internal static partial class WideOrientedBox
{
    #region Nested Types

    private readonly struct CylinderPenetration
    {
        internal readonly WideAxis3 Axis;
        internal readonly bool Negate;
        internal readonly Fixed64 Depth;
        internal readonly bool DepthIsClamped;
        internal readonly Signed576 Rational;
        internal readonly Signed832 RadialNumerator;
        internal readonly Signed576 RadialDenominator;
        internal readonly Signed576 SquaredAxisLength;

        internal CylinderPenetration(
            WideAxis3 axis,
            bool negate,
            Fixed64 depth,
            bool depthIsClamped,
            Signed576 rational,
            Signed832 radialNumerator,
            Signed576 radialDenominator,
            Signed576 squaredAxisLength)
        {
            Axis = axis;
            Negate = negate;
            Depth = depth;
            DepthIsClamped = depthIsClamped;
            Rational = rational;
            RadialNumerator = radialNumerator;
            RadialDenominator = radialDenominator;
            SquaredAxisLength = squaredAxisLength;
            HasValue = true;
        }

        internal bool HasValue { get; }
    }

    internal readonly struct CenteredCylinderContactFeature
    {
        internal CenteredCylinderContactFeature(
            int boxFaceAxis,
            int boxFaceSign,
            int cylinderCapSign)
        {
            BoxFaceAxis = boxFaceAxis;
            BoxFaceSign = boxFaceSign;
            CylinderCapSign = cylinderCapSign;
            IsCapFace = true;
        }

        internal int BoxFaceAxis { get; }
        internal int BoxFaceSign { get; }
        internal int CylinderCapSign { get; }
        internal bool IsCapFace { get; }
    }

    #endregion

    internal static bool TryGetCenteredCylinderContact(
        Vector3d center,
        FixedQuaternion orientation,
        Vector3d halfExtents,
        Vector3d cylinderCenter,
        FixedQuaternion cylinderRotation,
        Vector3d localCylinderAxisDirection,
        Fixed64 cylinderAxisLength,
        Fixed64 cylinderRadius,
        out FixedContactAnchors contact)
        => TryGetCenteredCylinderContact(
            center,
            orientation,
            halfExtents,
            cylinderCenter,
            cylinderRotation,
            localCylinderAxisDirection,
            cylinderAxisLength,
            cylinderRadius,
            out contact,
            out _);

    internal static bool TryGetCenteredCylinderContact(
        Vector3d center,
        FixedQuaternion orientation,
        Vector3d halfExtents,
        Vector3d cylinderCenter,
        FixedQuaternion cylinderRotation,
        Vector3d localCylinderAxisDirection,
        Fixed64 cylinderAxisLength,
        Fixed64 cylinderRadius,
        out FixedContactAnchors contact,
        out CenteredCylinderContactFeature feature)
    {
        GetRotatedLocalAxisNumerators(
            cylinderRotation,
            localCylinderAxisDirection,
            out Signed192 cylinderAxisX,
            out Signed192 cylinderAxisY,
            out Signed192 cylinderAxisZ,
            out Signed192 cylinderAxisDenominator);
        WideAxis3 cylinderAxis = new(
            Signed320.ExtendValue(cylinderAxisX),
            Signed320.ExtendValue(cylinderAxisY),
            Signed320.ExtendValue(cylinderAxisZ));
        if (!CanOverlapCenteredCapsule(
                center,
                orientation,
                halfExtents,
                cylinderCenter,
                cylinderRotation,
                localCylinderAxisDirection,
                cylinderAxisLength,
                cylinderRadius))
        {
            contact = default;
            feature = default;
            return false;
        }

        RationalBasis basis = new(orientation);
        Signed576 commonDenominatorWide = WideArithmetic.MultiplySigned576(
            Signed576.ExtendValue(
                WideArithmetic.MultiplySigned192(
                    basis.Denominator,
                    cylinderAxisDenominator)),
            Signed192.Raw(Fixed64.Two));
        // Each normalized rigid-frame denominator needs at most 68 signed
        // bits. Their product plus the Q32.32 factor for two needs at most
        // 170 signed bits, so this narrowing is exact.
        _ = Signed192.TryNarrowSigned(
            commonDenominatorWide,
            out Signed192 commonDenominator);
        Span<WideAxis3> boxAxes = stackalloc WideAxis3[3]
        {
            GetBasisAxis(basis, 0),
            GetBasisAxis(basis, 1),
            GetBasisAxis(basis, 2),
        };
        var best = default(CylinderPenetration);
        if (!TryKeepCylinderAxis(
                cylinderAxis,
                center,
                halfExtents,
                basis,
                commonDenominator,
                cylinderCenter,
                cylinderAxis,
                cylinderAxisDenominator,
                cylinderAxisLength,
                cylinderRadius,
                ref best))
        {
            contact = default;
            feature = default;
            return false;
        }

        for (int index = 0; index < boxAxes.Length; index++)
        {
            if (!TryKeepCylinderAxis(
                    boxAxes[index],
                    center,
                    halfExtents,
                    basis,
                    commonDenominator,
                    cylinderCenter,
                    cylinderAxis,
                    cylinderAxisDenominator,
                    cylinderAxisLength,
                    cylinderRadius,
                    ref best)
                || !TryKeepCylinderAxis(
                    Cross(boxAxes[index], cylinderAxis),
                    center,
                    halfExtents,
                    basis,
                    commonDenominator,
                    cylinderCenter,
                    cylinderAxis,
                    cylinderAxisDenominator,
                    cylinderAxisLength,
                    cylinderRadius,
                    ref best))
            {
                contact = default;
                feature = default;
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
                WideAxis3 endpointAxis = GetCornerToAxisEndpointAxis3D(
                    center,
                    cylinderCenter,
                    basis,
                    localCorner,
                    cylinderAxis,
                    cylinderAxisDenominator,
                    cylinderAxisLength,
                    endpointSign);
                if (!TryKeepCylinderAxis(
                        endpointAxis,
                        center,
                        halfExtents,
                        basis,
                        commonDenominator,
                        cylinderCenter,
                        cylinderAxis,
                        cylinderAxisDenominator,
                        cylinderAxisLength,
                        cylinderRadius,
                        ref best))
                {
                    contact = default;
                    feature = default;
                    return false;
                }
            }
        }

        WideAxis3 orientedAxis = best.Negate ? -best.Axis : best.Axis;
        Vector3d normal = WideGeometry.GetNormalized(
            Signed576.ExtendValue(orientedAxis.X),
            Signed576.ExtendValue(orientedAxis.Y),
            Signed576.ExtendValue(orientedAxis.Z));
        Vector3d boxLocalPoint = GetLocalAxisSupportPoint(
            basis,
            halfExtents,
            orientedAxis);
        FixedPointAnchor cylinderAnchor =
            WideGeometry.GetCenteredCylinderSupportAnchor(
                cylinderCenter,
                cylinderRotation,
                localCylinderAxisDirection,
                cylinderAxisLength,
                cylinderRadius,
                -normal);
        contact = new FixedContactAnchors(
            new FixedPointAnchor(
                center,
                orientation,
                boxLocalPoint),
            cylinderAnchor,
            normal,
            best.Depth,
            best.DepthIsClamped);
        feature = GetCenteredCylinderContactFeature(
            orientedAxis,
            cylinderAxis,
            basis);
        return true;
    }

    private static CenteredCylinderContactFeature
        GetCenteredCylinderContactFeature(
            WideAxis3 orientedAxis,
            WideAxis3 cylinderAxis,
            RationalBasis basis)
    {
        if (!AreParallel(orientedAxis, cylinderAxis))
            return default;

        for (int index = 0; index < 3; index++)
        {
            WideAxis3 boxAxis = GetBasisAxis(basis, index);
            if (!AreParallel(orientedAxis, boxAxis))
                continue;

            return new CenteredCylinderContactFeature(
                index,
                GetAxisProjection(orientedAxis, boxAxis).Sign,
                -GetAxisProjection(orientedAxis, cylinderAxis).Sign);
        }

        return default;
    }

    private static bool AreParallel(WideAxis3 first, WideAxis3 second)
    {
        Signed576 x = WideArithmetic.SubtractSigned576(
            WideArithmetic.MultiplySigned320(first.Y, second.Z),
            WideArithmetic.MultiplySigned320(first.Z, second.Y));
        Signed576 y = WideArithmetic.SubtractSigned576(
            WideArithmetic.MultiplySigned320(first.Z, second.X),
            WideArithmetic.MultiplySigned320(first.X, second.Z));
        Signed576 z = WideArithmetic.SubtractSigned576(
            WideArithmetic.MultiplySigned320(first.X, second.Y),
            WideArithmetic.MultiplySigned320(first.Y, second.X));
        return x.IsZero && y.IsZero && z.IsZero;
    }

    private static WideAxis3 GetCornerToAxisEndpointAxis3D(
        Vector3d boxCenter,
        Vector3d axisCenter,
        RationalBasis basis,
        Vector3d localCorner,
        WideAxis3 axis,
        Signed192 axisDenominator,
        Fixed64 axisLength,
        int endpointSign) =>
        new(
            GetCornerToAxisEndpointComponent(
                boxCenter.X,
                axisCenter.X,
                basis.Xx,
                basis.Yx,
                basis.Zx,
                basis.Denominator,
                localCorner,
                axis.X,
                axisDenominator,
                axisLength,
                endpointSign),
            GetCornerToAxisEndpointComponent(
                boxCenter.Y,
                axisCenter.Y,
                basis.Xy,
                basis.Yy,
                basis.Zy,
                basis.Denominator,
                localCorner,
                axis.Y,
                axisDenominator,
                axisLength,
                endpointSign),
            GetCornerToAxisEndpointComponent(
                boxCenter.Z,
                axisCenter.Z,
                basis.Xz,
                basis.Yz,
                basis.Zz,
                basis.Denominator,
                localCorner,
                axis.Z,
                axisDenominator,
                axisLength,
                endpointSign));

    private static Signed320 GetCornerToAxisEndpointComponent(
        Fixed64 boxCenter,
        Fixed64 axisCenter,
        Signed192 basisX,
        Signed192 basisY,
        Signed192 basisZ,
        Signed192 basisDenominator,
        Vector3d localCorner,
        Signed320 axisComponent,
        Signed192 axisDenominator,
        Fixed64 axisLength,
        int endpointSign)
    {
        Signed192 twiceScale = Signed192.Raw(Fixed64.Two);
        Signed320 corner = GetLocalOffsetNumerator(
            basisX,
            basisY,
            basisZ,
            localCorner);
        Signed576 center = WideArithmetic.MultiplySigned576(
            WideArithmetic.MultiplySigned320(
                WideArithmetic.MultiplySigned192(
                    WideArithmetic.SubtractSigned192(
                        Signed192.Raw(axisCenter),
                        Signed192.Raw(boxCenter)),
                    basisDenominator),
                Signed320.ExtendValue(axisDenominator)),
            twiceScale);
        Signed576 endpoint = WideArithmetic.MultiplySigned576(
            WideArithmetic.MultiplySigned576(
                Signed576.ExtendValue(axisComponent),
                Signed192.Signed(
                    endpointSign * axisLength.m_rawValue)),
            basisDenominator);
        Signed576 cornerWide = WideArithmetic.MultiplySigned576(
            WideArithmetic.MultiplySigned320(
                corner,
                Signed320.ExtendValue(axisDenominator)),
            twiceScale);
        Signed576 component = WideArithmetic.SubtractSigned576(
            WideArithmetic.AddSigned576(center, endpoint),
            cornerWide);
        // The center term needs at most 235 signed bits (65-bit delta, two
        // 68-bit rigid denominators, and the 34-bit Q32.32 factor for two).
        // The endpoint and corner terms are no wider, so the final difference
        // fits exactly in Signed320.
        return new Signed320(
            component.Word4,
            component.Word3,
            component.Word2,
            component.Word1,
            component.Word0);
    }

    private static bool TryKeepCylinderAxis(
        WideAxis3 axis,
        Vector3d boxCenter,
        Vector3d halfExtents,
        RationalBasis basis,
        Signed192 commonDenominator,
        Vector3d cylinderCenter,
        WideAxis3 cylinderAxis,
        Signed192 cylinderAxisDenominator,
        Fixed64 cylinderAxisLength,
        Fixed64 cylinderRadius,
        ref CylinderPenetration best)
    {
        if (axis.IsZero)
            return true;

        Signed576 boxRadius = GetBoxProjectionRadiusNumerator(
            axis,
            halfExtents,
            basis);
        Signed576 centerProjection = GetDifferenceProjection(
            cylinderCenter,
            boxCenter,
            axis);
        bool negate = centerProjection.Sign < 0;
        Signed576 baseRational = WideArithmetic.SubtractSigned576(
            boxRadius,
            WideArithmetic.MultiplySigned576(
                GetMagnitude(centerProjection),
                basis.Denominator));
        Signed576 rational = WideArithmetic.MultiplySigned576(
            WideArithmetic.MultiplySigned576(
                baseRational,
                cylinderAxisDenominator),
            Signed192.Raw(Fixed64.Two));
        Signed576 alignmentWide = GetAxisProjection(
            axis,
            cylinderAxis);
        Signed576 alignmentMagnitude = GetMagnitude(alignmentWide);
        rational = WideArithmetic.AddSigned576(
            rational,
            WideArithmetic.MultiplySigned576(
                WideArithmetic.MultiplySigned576(
                    alignmentMagnitude,
                    Signed192.Raw(cylinderAxisLength)),
                basis.Denominator));

        Signed576 cylinderAxisSquared = GetSquaredLength(cylinderAxis);
        Signed576 projectionSquared = GetSquaredLength(axis);
        Signed832 firstPlaneTerm = WideArithmetic.MultiplySigned576ToSigned832(
            projectionSquared,
            cylinderAxisSquared);
        Signed832 planeSquared = WideArithmetic.SubtractSigned832(
            firstPlaneTerm,
            WideArithmetic.MultiplySigned576ToSigned832(
                alignmentWide,
                alignmentWide));
        Signed320 radiusSquared = WideArithmetic.MultiplySigned192(
            Signed192.Raw(cylinderRadius),
            Signed192.Raw(cylinderRadius));
        Signed832 radialNumerator = WideArithmetic.MultiplyNonNegativeToSigned832(
            planeSquared,
            radiusSquared);
        if (rational.Sign < 0
            && !IsCylinderRadicalAtLeastRatio(
                radialNumerator,
                cylinderAxisSquared,
                WideArithmetic.SubtractSigned576(default, rational),
                commonDenominator))
        {
            return false;
        }

        GetCylinderDepth(
            rational,
            radialNumerator,
            cylinderAxisSquared,
            projectionSquared,
            commonDenominator,
            out Fixed64 depth,
            out bool depthIsClamped);
        bool shouldReplace = !best.HasValue || depth < best.Depth;
        if (best.HasValue && depth == best.Depth)
        {
            shouldReplace = WideArithmetic.CompareRadialProjectionDepths(
                rational,
                radialNumerator,
                cylinderAxisSquared,
                projectionSquared,
                best.Rational,
                best.RadialNumerator,
                best.RadialDenominator,
                best.SquaredAxisLength,
                commonDenominator) < 0;
        }
        if (shouldReplace)
        {
            best = new CylinderPenetration(
                axis,
                negate,
                depth,
                depthIsClamped,
                rational,
                radialNumerator,
                cylinderAxisSquared,
                projectionSquared);
        }
        return true;
    }

    private static bool IsCylinderRadicalAtLeastRatio(
        Signed832 radicalNumerator,
        Signed576 radicalDenominator,
        Signed576 ratioNumerator,
        Signed192 ratioDenominator)
    {
        Span<ulong> radical = stackalloc ulong[13];
        Span<ulong> radicalDivisor = stackalloc ulong[9];
        Span<ulong> ratio = stackalloc ulong[9];
        Span<ulong> ratioDivisor = stackalloc ulong[3];
        Span<ulong> squared = stackalloc ulong[18];
        Span<ulong> ratioDivisorSquared = stackalloc ulong[6];
        Span<ulong> left = stackalloc ulong[TriangleSweepMagnitudeWords];
        Span<ulong> right = stackalloc ulong[TriangleSweepMagnitudeWords];
        WideArithmetic.GetMagnitude(radicalNumerator, radical);
        WideArithmetic.GetMagnitude(
            radicalDenominator,
            radicalDivisor);
        WideArithmetic.GetMagnitude(ratioNumerator, ratio);
        WideArithmetic.GetMagnitude(
            ratioDenominator,
            out ratioDivisor[2],
            out ratioDivisor[1],
            out ratioDivisor[0]);
        WideArithmetic.MultiplyMagnitudes(
            ratioDivisor,
            ratioDivisor,
            ratioDivisorSquared);
        WideArithmetic.MultiplyMagnitudes(radical, ratioDivisorSquared, left);
        WideArithmetic.MultiplyMagnitudes(ratio, ratio, squared);
        WideArithmetic.MultiplyMagnitudes(squared, radicalDivisor, right);
        return WideArithmetic.CompareMagnitudeEqualLength(left, right) >= 0;
    }

    private static void GetCylinderDepth(
        Signed576 rational,
        Signed832 radialNumerator,
        Signed576 radialDenominator,
        Signed576 squaredAxisLength,
        Signed192 commonDenominator,
        out Fixed64 depth,
        out bool depthIsClamped)
    {
        Signed576 radial = WideArithmetic.GetFloorSquareRootOfProduct(
            radialNumerator,
            Signed192.Signed(1L));
        Signed576 radialDivisorWide = WideArithmetic.GetFloorSquareRoot(
            Signed704.ExtendValue(radialDenominator));
        // A rotated unit-axis numerator needs at most 103 signed bits. Its
        // squared length therefore needs at most 208, and this square root
        // needs at most 104, so the Signed192 narrowing is exact.
        _ = Signed192.TryNarrowSigned(
            radialDivisorWide,
            out Signed192 radialDivisor);
        Signed576 numerator = WideArithmetic.AddSigned576(
            WideArithmetic.MultiplySigned576(rational, radialDivisor),
            WideArithmetic.MultiplySigned576(radial, commonDenominator));
        Signed576 axisLength = WideArithmetic.GetFloorSquareRoot(
            Signed704.ExtendValue(squaredAxisLength));
        Signed320 commonAndRadial = WideArithmetic.MultiplySigned192(
            commonDenominator,
            radialDivisor);
        Signed704 denominator = WideArithmetic.MultiplySigned576ToSigned704(
            axisLength,
            commonAndRadial);
        if (!Fixed64.TryGetSignedRawRatio(
                Signed704.ExtendValue(numerator),
                denominator,
                out depth))
        {
            depth = Fixed64.MaxValue;
            depthIsClamped = true;
            return;
        }

        depthIsClamped = false;
    }
}
