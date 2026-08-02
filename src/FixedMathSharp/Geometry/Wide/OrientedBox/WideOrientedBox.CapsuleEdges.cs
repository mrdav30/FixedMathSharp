//=======================================================================
// WideOrientedBox.CapsuleEdges.cs
//=======================================================================
// MIT License, Copyright (c) 2024-present David Oravsky (mrdav30)
// See LICENSE file in the project root for full license information.
//=======================================================================

using System;

namespace FixedMathSharp.Geometry;

/// <content>
/// Handles wide-precision capsule edge tests against oriented boxes,
/// including axis projection and containment calculations.
/// </content>
internal static partial class WideOrientedBox
{
    #region Nested Types

    private readonly struct CapsuleAxis3
    {
        internal readonly Signed576 X;
        internal readonly Signed576 Y;
        internal readonly Signed576 Z;

        internal CapsuleAxis3(Signed576 x, Signed576 y, Signed576 z)
        {
            X = x;
            Y = y;
            Z = z;
        }

        internal bool IsZero => X.IsZero && Y.IsZero && Z.IsZero;

        public static CapsuleAxis3 operator -(CapsuleAxis3 value) =>
            new(
                WideArithmetic.SubtractSigned576(default, value.X),
                WideArithmetic.SubtractSigned576(default, value.Y),
                WideArithmetic.SubtractSigned576(default, value.Z));
    }

    #endregion

    private static CapsuleAxis3 GetVertexToCapsuleAxis(
        Vector3d boxCenter,
        Vector3d capsuleCenter,
        WideRationalBasis3d basis,
        Vector3d localVertex,
        WideAxis3 capsuleAxis,
        Signed192 capsuleAxisDenominator,
        Fixed64 capsuleAxisLength)
    {
        WideAxis3 centerAxis = GetCornerToAxisCenterAxis3D(
            boxCenter,
            capsuleCenter,
            basis,
            localVertex,
            capsuleAxisDenominator);
        Signed576 projection = WideAxis3.Dot(
            centerAxis,
            capsuleAxis);
        Signed576 capsuleSquared = capsuleAxis.SquaredLength;
        Signed576 projectionAtHalfLength = WideArithmetic.MultiplySigned576(
            GetMagnitude(projection),
            Signed192.Raw(Fixed64.Two));
        Signed576 capBound = WideArithmetic.MultiplySigned576(
            WideArithmetic.MultiplySigned576(
                capsuleSquared,
                Signed192.Raw(capsuleAxisLength)),
            basis.Denominator);
        if (WideArithmetic.CompareNonNegative(
                projectionAtHalfLength,
                capBound) >= 0)
        {
            int endpointSign = projection.Sign > 0 ? -1 : 1;
            return ToCapsuleAxis(GetCornerToAxisEndpointAxis3D(
                boxCenter,
                capsuleCenter,
                basis,
                localVertex,
                capsuleAxis,
                capsuleAxisDenominator,
                capsuleAxisLength,
                endpointSign));
        }

        return GetPerpendicularAxis(
            centerAxis,
            capsuleAxis,
            capsuleSquared,
            projection);
    }

    private static CapsuleAxis3 GetCapsuleEndpointToBoxEdgeAxis(
        Vector3d boxCenter,
        Vector3d capsuleCenter,
        WideRationalBasis3d basis,
        Vector3d halfExtents,
        int edgeAxisIndex,
        int firstSign,
        int secondSign,
        int endpointSign,
        WideAxis3 capsuleAxis,
        Signed192 capsuleAxisDenominator,
        Fixed64 capsuleAxisLength)
    {
        Vector3d localEdgeCenter = GetLocalEdgeCenter(
            halfExtents,
            edgeAxisIndex,
            firstSign,
            secondSign);
        WideAxis3 endpointAxis = GetCornerToAxisEndpointAxis3D(
            boxCenter,
            capsuleCenter,
            basis,
            localEdgeCenter,
            capsuleAxis,
            capsuleAxisDenominator,
            capsuleAxisLength,
            endpointSign);
        WideAxis3 edgeAxis = basis.GetAxis(edgeAxisIndex);
        Signed576 projection = WideAxis3.Dot(endpointAxis, edgeAxis);
        Signed576 edgeSquared = edgeAxis.SquaredLength;
        Fixed64 edgeExtent = GetEdgeAxisExtent(
            halfExtents,
            edgeAxisIndex);
        Signed576 edgeBound = WideArithmetic.MultiplySigned576(
            WideArithmetic.MultiplySigned576(
                edgeSquared,
                capsuleAxisDenominator),
            Signed192.Raw(edgeExtent));
        edgeBound = WideArithmetic.AddSigned576(edgeBound, edgeBound);
        if (WideArithmetic.CompareNonNegative(
                GetMagnitude(projection),
                edgeBound) >= 0)
        {
            int edgeSign = projection.Sign < 0 ? -1 : 1;
            Vector3d localVertex = AddEdgeAxisExtent(
                localEdgeCenter,
                edgeAxisIndex,
                edgeSign,
                edgeExtent);
            return ToCapsuleAxis(GetCornerToAxisEndpointAxis3D(
                boxCenter,
                capsuleCenter,
                basis,
                localVertex,
                capsuleAxis,
                capsuleAxisDenominator,
                capsuleAxisLength,
                endpointSign));
        }

        return GetPerpendicularAxis(
            endpointAxis,
            edgeAxis,
            edgeSquared,
            projection);
    }

    private static WideAxis3 GetCornerToAxisCenterAxis3D(
        Vector3d boxCenter,
        Vector3d axisCenter,
        WideRationalBasis3d basis,
        Vector3d localCorner,
        Signed192 axisDenominator) =>
        new(
            GetCornerToAxisCenterComponent(
                boxCenter.X,
                axisCenter.X,
                basis.Xx,
                basis.Yx,
                basis.Zx,
                basis.Denominator,
                localCorner,
                axisDenominator),
            GetCornerToAxisCenterComponent(
                boxCenter.Y,
                axisCenter.Y,
                basis.Xy,
                basis.Yy,
                basis.Zy,
                basis.Denominator,
                localCorner,
                axisDenominator),
            GetCornerToAxisCenterComponent(
                boxCenter.Z,
                axisCenter.Z,
                basis.Xz,
                basis.Yz,
                basis.Zz,
                basis.Denominator,
                localCorner,
                axisDenominator));

    private static Signed320 GetCornerToAxisCenterComponent(
        Fixed64 boxCenter,
        Fixed64 axisCenter,
        Signed192 basisX,
        Signed192 basisY,
        Signed192 basisZ,
        Signed192 basisDenominator,
        Vector3d localCorner,
        Signed192 axisDenominator)
    {
        Signed320 corner = GetLocalOffsetNumerator(
            basisX,
            basisY,
            basisZ,
            localCorner);
        Signed576 center = WideArithmetic.MultiplySigned320(
            WideArithmetic.MultiplySigned192(
                WideArithmetic.SubtractSigned192(
                    Signed192.Raw(axisCenter),
                    Signed192.Raw(boxCenter)),
                basisDenominator),
            Signed320.ExtendValue(axisDenominator));
        Signed576 cornerWide = WideArithmetic.MultiplySigned320(
            corner,
            Signed320.ExtendValue(axisDenominator));
        // The 65-bit center delta and at-most-134-bit rotated corner each
        // retain both at-most-68-bit rigid denominators. Their difference
        // needs at most 203 signed bits and therefore fits Signed320 exactly.
        return Signed320.NarrowValue(WideArithmetic.SubtractSigned576(center, cornerWide));
    }

    private static CapsuleAxis3 GetPerpendicularAxis(
        WideAxis3 difference,
        WideAxis3 lineAxis,
        Signed576 lineSquared,
        Signed576 projection) =>
        new(
            GetPerpendicularComponent(
                difference.X,
                lineAxis.X,
                lineSquared,
                projection),
            GetPerpendicularComponent(
                difference.Y,
                lineAxis.Y,
                lineSquared,
                projection),
            GetPerpendicularComponent(
                difference.Z,
                lineAxis.Z,
                lineSquared,
                projection));

    private static Signed576 GetPerpendicularComponent(
        Signed320 difference,
        Signed320 line,
        Signed576 lineSquared,
        Signed576 projection)
    {
        Signed704 first = WideArithmetic.MultiplySigned576ToSigned704(
            lineSquared,
            difference);
        Signed704 second = WideArithmetic.MultiplySigned576ToSigned704(
            projection,
            line);
        // Vertex-to-axis terms need at most 411 signed bits; endpoint-to-edge
        // terms need at most 377. Their difference therefore fits Signed576
        // exactly before it becomes a candidate separating axis.
        return Signed576.NarrowValue(
            WideArithmetic.SubtractSigned704(first, second));
    }

    private static bool TryKeepCapsuleAxis(
        CapsuleAxis3 axis,
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
    {
        if (axis.IsZero)
            return true;

        Signed576 boxRadius = GetCapsuleBoxRadius(
            axis,
            halfExtents,
            basis);
        Signed576 centerProjection = GetCapsuleCenterProjection(
            capsuleCenter,
            boxCenter,
            axis);
        bool negate = centerProjection.Sign < 0;
        Signed576 baseRational = WideArithmetic.SubtractSigned576(
            boxRadius,
            WideArithmetic.MultiplySigned576(
                GetMagnitude(centerProjection),
                basis.Denominator));
        Signed320 baseScale = WideArithmetic.MultiplySigned192(
            capsuleAxisDenominator,
            Signed192.Raw(Fixed64.Two));
        Signed704 rational = WideArithmetic.MultiplySigned576ToSigned704(
            baseRational,
            baseScale);
        // A 411-bit candidate component projected onto the at-most-103-bit
        // rotated capsule axis needs at most 517 signed bits including the
        // three-term sum, so this Signed576 narrowing is exact.
        Signed576 alignment = Signed576.NarrowValue(
            GetCapsuleAxisProjection(axis, capsuleAxis));
        Signed320 axialScale = WideArithmetic.MultiplySigned192(
            Signed192.Raw(capsuleAxisLength),
            basis.Denominator);
        Signed704 axial = WideArithmetic.MultiplySigned576ToSigned704(
            GetMagnitude(alignment),
            axialScale);
        rational = WideArithmetic.AddSigned704(rational, axial);

        Signed832 squaredAxisLength = GetCapsuleSquaredLength(axis);
        if (rational.Sign < 0
            && !IsWideCapsuleRadicalAtLeastRational(
                squaredAxisLength,
                commonDenominator,
                capsuleRadius,
                rational))
        {
            return false;
        }

        bool shouldReplace = !best.HasValue;
        if (best.HasValue)
        {
            int comparison = CompareWideCapsuleDepth(
                rational,
                squaredAxisLength,
                best.WideRational,
                best.WideSquaredAxisLength);
            shouldReplace = comparison < 0
                || (comparison == 0 && featureRank < best.FeatureRank);
        }
        if (shouldReplace)
        {
            best = new CapsulePenetration(
                axis,
                negate,
                rational,
                squaredAxisLength,
                featureRank);
        }
        return true;
    }

    // A 411-bit candidate component projected onto an at-most-68-bit box
    // basis component needs at most 482 signed bits including the three-term
    // sum, so each Signed704-to-Signed576 narrowing below is exact.
    private static Signed576 GetCapsuleBoxRadius(
        CapsuleAxis3 axis,
        Vector3d halfExtents,
        WideRationalBasis3d basis) =>
        WideArithmetic.AddSigned576(
            WideArithmetic.AddSigned576(
                WideArithmetic.MultiplySigned576(
                    GetMagnitude(Signed576.NarrowValue(
                        GetCapsuleBasisProjection(
                            axis,
                            basis.Xx,
                            basis.Xy,
                            basis.Xz))),
                    Signed192.Raw(halfExtents.X)),
                WideArithmetic.MultiplySigned576(
                    GetMagnitude(Signed576.NarrowValue(
                        GetCapsuleBasisProjection(
                            axis,
                            basis.Yx,
                            basis.Yy,
                            basis.Yz))),
                    Signed192.Raw(halfExtents.Y))),
            WideArithmetic.MultiplySigned576(
                GetMagnitude(Signed576.NarrowValue(
                    GetCapsuleBasisProjection(
                        axis,
                        basis.Zx,
                        basis.Zy,
                        basis.Zz))),
                Signed192.Raw(halfExtents.Z)));

    // A 411-bit candidate component times a 65-bit center delta needs at most
    // 479 signed bits including the three-term sum.
    private static Signed576 GetCapsuleCenterProjection(
        Vector3d end,
        Vector3d start,
        CapsuleAxis3 axis) =>
        Signed576.NarrowValue(WideArithmetic.AddSigned704(
            WideArithmetic.AddSigned704(
                WideArithmetic.MultiplySigned576ToSigned704(
                    axis.X,
                    Signed320.ExtendValue(
                        WideArithmetic.SubtractSigned192(
                            Signed192.Raw(end.X),
                            Signed192.Raw(start.X)))),
                WideArithmetic.MultiplySigned576ToSigned704(
                    axis.Y,
                    Signed320.ExtendValue(
                        WideArithmetic.SubtractSigned192(
                            Signed192.Raw(end.Y),
                            Signed192.Raw(start.Y))))),
            WideArithmetic.MultiplySigned576ToSigned704(
                axis.Z,
                Signed320.ExtendValue(
                    WideArithmetic.SubtractSigned192(
                        Signed192.Raw(end.Z),
                        Signed192.Raw(start.Z))))));

    private static Signed704 GetCapsuleBasisProjection(
        CapsuleAxis3 axis,
        Signed192 x,
        Signed192 y,
        Signed192 z) =>
        WideArithmetic.AddSigned704(
            WideArithmetic.AddSigned704(
                WideArithmetic.MultiplySigned576ToSigned704(
                    axis.X,
                    Signed320.ExtendValue(x)),
                WideArithmetic.MultiplySigned576ToSigned704(
                    axis.Y,
                    Signed320.ExtendValue(y))),
            WideArithmetic.MultiplySigned576ToSigned704(
                axis.Z,
                Signed320.ExtendValue(z)));

    private static Signed704 GetCapsuleAxisProjection(
        CapsuleAxis3 axis,
        WideAxis3 direction) =>
        WideArithmetic.AddSigned704(
            WideArithmetic.AddSigned704(
                WideArithmetic.MultiplySigned576ToSigned704(
                    axis.X,
                    direction.X),
                WideArithmetic.MultiplySigned576ToSigned704(
                    axis.Y,
                    direction.Y)),
            WideArithmetic.MultiplySigned576ToSigned704(
                axis.Z,
                direction.Z));

    private static Signed832 GetCapsuleSquaredLength(
        CapsuleAxis3 axis) =>
        WideArithmetic.AddSigned832(
            WideArithmetic.AddSigned832(
                WideArithmetic.MultiplySigned576ToSigned832(
                    axis.X,
                    axis.X),
                WideArithmetic.MultiplySigned576ToSigned832(
                    axis.Y,
                    axis.Y)),
            WideArithmetic.MultiplySigned576ToSigned832(
                axis.Z,
                axis.Z));

    private static bool IsWideCapsuleRadicalAtLeastRational(
        Signed832 squaredAxisLength,
        Signed192 commonDenominator,
        Fixed64 radius,
        Signed704 rational)
    {
        Span<ulong> axis = stackalloc ulong[13];
        Span<ulong> coefficient = stackalloc ulong[5];
        Span<ulong> coefficientSquared = stackalloc ulong[10];
        Span<ulong> rationalMagnitude = stackalloc ulong[11];
        Span<ulong> left = stackalloc ulong[TriangleSweepMagnitudeWords];
        Span<ulong> right = stackalloc ulong[TriangleSweepMagnitudeWords];
        WideArithmetic.GetMagnitude(squaredAxisLength, axis);
        Signed320 coefficientValue = WideArithmetic.MultiplySigned192(
            commonDenominator,
            Signed192.Raw(radius));
        WideArithmetic.GetMagnitude(
            coefficientValue,
            out coefficient[4],
            out coefficient[3],
            out coefficient[2],
            out coefficient[1],
            out coefficient[0]);
        WideArithmetic.MultiplyMagnitudes(
            coefficient,
            coefficient,
            coefficientSquared);
        WideArithmetic.MultiplyMagnitudes(coefficientSquared, axis, left);
        WideArithmetic.GetMagnitude(rational, rationalMagnitude);
        WideArithmetic.MultiplyMagnitudes(
            rationalMagnitude,
            rationalMagnitude,
            right);
        return WideArithmetic.CompareMagnitudeEqualLength(left, right) >= 0;
    }

    private static void GetWideCapsuleDepth(
        Signed704 rational,
        Signed832 squaredAxisLength,
        Signed192 commonDenominator,
        Fixed64 radius,
        out Fixed64 depth,
        out bool depthIsClamped)
    {
        Signed576 axisLength = WideArithmetic.GetFloorSquareRootOfProduct(
            squaredAxisLength,
            Signed192.Signed(1L));
        Signed704 denominator = WideArithmetic.MultiplySigned576ToSigned704(
            axisLength,
            Signed320.ExtendValue(commonDenominator));
        Signed320 radialScale = WideArithmetic.MultiplySigned192(
            commonDenominator,
            Signed192.Raw(radius));
        // Rigid basis axes have an exact integer length. Every other capsule
        // candidate retains both rigid Q32.32 denominators, so flooring its
        // radical is still finer than one Q32.32 quotient rounding interval.
        Signed704 numerator = WideArithmetic.AddSigned704(
            rational,
            WideArithmetic.MultiplySigned576ToSigned704(
                axisLength,
                radialScale));
        if (!Fixed64.TryGetSignedRawRatio(numerator, denominator, out depth))
        {
            depth = Fixed64.MaxValue;
            depthIsClamped = true;
            return;
        }

        depthIsClamped = false;
    }

    private static int CompareWideCapsuleDepth(
        Signed704 candidateRational,
        Signed832 candidateSquaredAxisLength,
        Signed704 currentRational,
        Signed832 currentSquaredAxisLength)
    {
        int candidateSign = candidateRational.Sign;
        int currentSign = currentRational.Sign;
        Span<ulong> candidate = stackalloc ulong[11];
        Span<ulong> current = stackalloc ulong[11];
        Span<ulong> candidateAxis = stackalloc ulong[13];
        Span<ulong> currentAxis = stackalloc ulong[13];
        WideArithmetic.GetMagnitude(candidateRational, candidate);
        WideArithmetic.GetMagnitude(currentRational, current);
        WideArithmetic.GetMagnitude(
            candidateSquaredAxisLength,
            candidateAxis);
        WideArithmetic.GetMagnitude(
            currentSquaredAxisLength,
            currentAxis);
        return WideArithmetic.CompareSignedNormalizedMagnitudes(
            candidate,
            candidateSign,
            candidateAxis,
            current,
            currentSign,
            currentAxis);
    }

    private static Vector3d GetEdgeAxisOffset(
        int edgeAxisIndex,
        int sign,
        Fixed64 extent) =>
        edgeAxisIndex switch
        {
            0 => new Vector3d(sign * extent, Fixed64.Zero, Fixed64.Zero),
            1 => new Vector3d(Fixed64.Zero, sign * extent, Fixed64.Zero),
            _ => new Vector3d(Fixed64.Zero, Fixed64.Zero, sign * extent),
        };

    private static Vector3d AddEdgeAxisExtent(
        Vector3d localEdgeCenter,
        int edgeAxisIndex,
        int sign,
        Fixed64 extent) =>
        localEdgeCenter + GetEdgeAxisOffset(
            edgeAxisIndex,
            sign,
            extent);

    private static Fixed64 GetEdgeAxisExtent(
        Vector3d halfExtents,
        int edgeAxisIndex) =>
        edgeAxisIndex switch
        {
            0 => halfExtents.X,
            1 => halfExtents.Y,
            _ => halfExtents.Z,
        };

    private static CapsuleAxis3 ToCapsuleAxis(WideAxis3 axis) =>
        new(
            Signed576.ExtendValue(axis.X),
            Signed576.ExtendValue(axis.Y),
            Signed576.ExtendValue(axis.Z));

    private static Vector3d GetMatchedBoxSupportLocalPoint(
        WideRationalBasis3d basis,
        Vector3d halfExtents,
        Vector3d boxCenter,
        FixedQuaternion boxRotation,
        in FixedPointAnchor otherAnchor,
        CapsuleAxis3 featureAxis)
    {
        // Vertex-to-axis perpendicular components need at most 412 signed
        // bits. Basis, center-delta, and capsule-axis projections add at most
        // 103 bits, so every caller remains below the nine-word range.
        Signed576 xProjection = Signed576.NarrowValue(
            GetCapsuleBasisProjection(
                featureAxis,
                basis.Xx,
                basis.Xy,
                basis.Xz));
        Signed576 yProjection = Signed576.NarrowValue(
            GetCapsuleBasisProjection(
                featureAxis,
                basis.Yx,
                basis.Yy,
                basis.Yz));
        Signed576 zProjection = Signed576.NarrowValue(
            GetCapsuleBasisProjection(
                featureAxis,
                basis.Zx,
                basis.Zy,
                basis.Zz));
        return GetMatchedBoxSupportLocalPoint(
            basis,
            halfExtents,
            boxCenter,
            boxRotation,
            otherAnchor,
            xProjection,
            yProjection,
            zProjection);
    }
}
