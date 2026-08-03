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

    private static CapsuleAxis3 GetLocalVertexToCapsuleAxis(
        in WideAxis3 translation,
        Signed192 boxDenominator,
        Vector3d localVertex,
        in WideAxis3 capsuleAxis,
        in Signed576 capsuleAxisSquared,
        in Signed576 capsuleCenterAxisProjection,
        in Signed320 centerScale,
        in Signed320 axialScale,
        in Signed704 capBound)
    {
        WideAxis3 difference = GetLocalPointToCapsuleCenterAxis(
            translation,
            boxDenominator,
            localVertex);
        var localVertexAxis = new WideAxis3(
            Signed320.ExtendValue(Signed192.Raw(localVertex.X)),
            Signed320.ExtendValue(Signed192.Raw(localVertex.Y)),
            Signed320.ExtendValue(Signed192.Raw(localVertex.Z)));
        Signed576 projection = WideArithmetic.SubtractSigned576(
            capsuleCenterAxisProjection,
            WideAxis3.Dot(localVertexAxis, capsuleAxis));
        Signed704 projectionAtHalfLength =
            WideArithmetic.MultiplySigned576ToSigned704(
                GetMagnitude(projection),
                centerScale);
        if (WideArithmetic.CompareNonNegative(
                projectionAtHalfLength,
                capBound) >= 0)
        {
            return GetLocalPointToCapsuleEndpointAxis(
                difference,
                centerScale,
                axialScale,
                capsuleAxis,
                projection.Sign > 0 ? -1 : 1);
        }

        var scaledDifference = new WideAxis3(
            Signed320.NarrowValue(WideArithmetic.MultiplySigned320(
                difference.X,
                Signed320.ExtendValue(boxDenominator))),
            Signed320.NarrowValue(WideArithmetic.MultiplySigned320(
                difference.Y,
                Signed320.ExtendValue(boxDenominator))),
            Signed320.NarrowValue(WideArithmetic.MultiplySigned320(
                difference.Z,
                Signed320.ExtendValue(boxDenominator))));
        return GetPerpendicularAxis(
            scaledDifference,
            capsuleAxis,
            capsuleAxisSquared,
            projection);
    }

    private static CapsuleAxis3 GetCapsuleEndpointToBoxEdgeAxis(
        in WideAxis3 edgeDifference,
        in Signed320 centerScale,
        in Signed320 axialScale,
        int edgeAxisIndex,
        int endpointSign,
        in WideAxis3 capsuleAxis,
        in Signed320 edgeBound)
    {
        CapsuleAxis3 endpointAxis = GetLocalPointToCapsuleEndpointAxis(
            edgeDifference,
            centerScale,
            axialScale,
            capsuleAxis,
            endpointSign);
        Signed576 projection = edgeAxisIndex switch
        {
            0 => endpointAxis.X,
            1 => endpointAxis.Y,
            _ => endpointAxis.Z,
        };
        Signed576 wideEdgeBound = Signed576.ExtendValue(edgeBound);
        Signed576 edgeComponent = default;
        if (WideArithmetic.CompareNonNegative(
                GetMagnitude(projection),
                wideEdgeBound) >= 0)
        {
            edgeComponent = projection.Sign < 0
                ? WideArithmetic.AddSigned576(projection, wideEdgeBound)
                : WideArithmetic.SubtractSigned576(projection, wideEdgeBound);
        }

        return edgeAxisIndex switch
        {
            0 => new CapsuleAxis3(
                edgeComponent,
                endpointAxis.Y,
                endpointAxis.Z),
            1 => new CapsuleAxis3(
                endpointAxis.X,
                edgeComponent,
                endpointAxis.Z),
            _ => new CapsuleAxis3(
                endpointAxis.X,
                endpointAxis.Y,
                edgeComponent),
        };
    }

    private static WideAxis3 GetLocalPointToCapsuleCenterAxis(
        in WideAxis3 translation,
        Signed192 boxDenominator,
        Vector3d localPoint) =>
        new(
            WideArithmetic.SubtractSigned320(
                translation.X,
                WideArithmetic.MultiplySigned192(
                    Signed192.Raw(localPoint.X),
                    boxDenominator)),
            WideArithmetic.SubtractSigned320(
                translation.Y,
                WideArithmetic.MultiplySigned192(
                    Signed192.Raw(localPoint.Y),
                    boxDenominator)),
            WideArithmetic.SubtractSigned320(
                translation.Z,
                WideArithmetic.MultiplySigned192(
                    Signed192.Raw(localPoint.Z),
                    boxDenominator)));

    private static CapsuleAxis3 GetLocalPointToCapsuleEndpointAxis(
        in WideAxis3 difference,
        in Signed320 centerScale,
        in Signed320 axialScale,
        in WideAxis3 capsuleAxis,
        int endpointSign)
    {
        return new CapsuleAxis3(
            GetLocalEndpointComponent(
                difference.X,
                capsuleAxis.X,
                centerScale,
                axialScale,
                endpointSign),
            GetLocalEndpointComponent(
                difference.Y,
                capsuleAxis.Y,
                centerScale,
                axialScale,
                endpointSign),
            GetLocalEndpointComponent(
                difference.Z,
                capsuleAxis.Z,
                centerScale,
                axialScale,
                endpointSign));
    }

    private static Signed576 GetLocalEndpointComponent(
        in Signed320 difference,
        in Signed320 axis,
        in Signed320 centerScale,
        in Signed320 axialScale,
        int endpointSign)
    {
        Signed576 center = WideArithmetic.MultiplySigned320(
            difference,
            centerScale);
        Signed576 axial = WideArithmetic.MultiplySigned320(
            axis,
            axialScale);
        return endpointSign < 0
            ? WideArithmetic.SubtractSigned576(center, axial)
            : WideArithmetic.AddSigned576(center, axial);
    }

    private static Signed576 GetScaledCapsuleAxisSquared(
        Vector3d localAxis,
        Signed192 basisDenominator)
    {
        var rawAxis = new WideAxis3(
            Signed320.ExtendValue(Signed192.Raw(localAxis.X)),
            Signed320.ExtendValue(Signed192.Raw(localAxis.Y)),
            Signed320.ExtendValue(Signed192.Raw(localAxis.Z)));
        Signed320 denominatorSquared = WideArithmetic.MultiplySigned192(
            basisDenominator,
            basisDenominator);
        // Rotation preserves length, so |C*u|^2 = Dc^2*|u|^2. A validated
        // Q32.32 unit axis and normalized-quaternion denominator keep this
        // exact product below 196 bits.
        return Signed576.NarrowValue(
            WideArithmetic.MultiplySigned576ToSigned704(
                rawAxis.SquaredLength,
                denominatorSquared));
    }

    private static Signed576 GetCapsuleCenterAxisProjection(
        Signed192 boxDenominator,
        Vector3d boxCenter,
        Vector3d capsuleCenter,
        in WideRationalBasis3d capsuleBasis,
        Vector3d localCapsuleAxis)
    {
        GetRelativeLocalPointNumerators(
            capsuleCenter,
            boxCenter,
            capsuleBasis,
            out Signed192 translationX,
            out Signed192 translationY,
            out Signed192 translationZ);
        var capsuleTranslation = new WideAxis3(
            Signed320.ExtendValue(translationX),
            Signed320.ExtendValue(translationY),
            Signed320.ExtendValue(translationZ));
        var rawAxis = new WideAxis3(
            Signed320.ExtendValue(Signed192.Raw(localCapsuleAxis.X)),
            Signed320.ExtendValue(Signed192.Raw(localCapsuleAxis.Y)),
            Signed320.ExtendValue(Signed192.Raw(localCapsuleAxis.Z)));
        // (C^T*delta) dot u = delta dot (C*u). Multiplying by Db gives
        // the same reduced scalar as each old world vertex projection,
        // without constructing either world vector per vertex.
        return WideArithmetic.MultiplySigned576(
            WideAxis3.Dot(capsuleTranslation, rawAxis),
            boxDenominator);
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
        in WideAxis3 difference,
        in WideAxis3 lineAxis,
        in Signed576 lineSquared,
        in Signed576 projection) =>
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
        in Signed320 difference,
        in Signed320 line,
        in Signed576 lineSquared,
        in Signed576 projection)
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
        in CapsuleAxis3 axis,
        in WideAxis3 scaledExtents,
        in WideAxis3 twiceTranslation,
        Signed192 commonDenominator,
        in WideAxis3 capsuleAxis,
        in Signed320 axialScale,
        Fixed64 capsuleRadius,
        int featureRank,
        ref CapsulePenetration best)
    {
        if (axis.IsZero)
            return true;

        Signed704 boxRadius = GetLocalCapsuleBoxRadius(
            axis,
            scaledExtents);
        Signed704 centerProjection = GetLocalCapsuleCenterProjection(
            axis,
            twiceTranslation);
        bool negate = centerProjection.Sign < 0;
        Signed704 rational = WideArithmetic.SubtractSigned704(
            boxRadius,
            negate
                ? WideArithmetic.SubtractSigned704(
                    default,
                    centerProjection)
                : centerProjection);
        // Every candidate norm is strictly below 2^411. Normalized quaternion
        // denominators are below 2^65, so the relative-axis norm
        // |A| = Db*Dc*|u| is below 2^163 for a validated Q32.32 unit axis.
        // Cauchy therefore bounds |candidate dot A| below 2^574, inside the
        // positive 575-bit magnitude of Signed576. This correlated norm bound
        // is tighter than summing independent component-width maxima.
        Signed576 alignment = Signed576.NarrowValue(
            GetCapsuleAxisProjection(axis, capsuleAxis));
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

    private static Signed704 GetLocalCapsuleBoxRadius(
        in CapsuleAxis3 axis,
        in WideAxis3 scaledExtents) =>
        WideArithmetic.AddSigned704(
            WideArithmetic.AddSigned704(
                WideArithmetic.MultiplySigned576ToSigned704(
                    GetMagnitude(axis.X),
                    scaledExtents.X),
                WideArithmetic.MultiplySigned576ToSigned704(
                    GetMagnitude(axis.Y),
                    scaledExtents.Y)),
            WideArithmetic.MultiplySigned576ToSigned704(
                GetMagnitude(axis.Z),
                scaledExtents.Z));

    private static Signed704 GetLocalCapsuleCenterProjection(
        in CapsuleAxis3 axis,
        in WideAxis3 twiceTranslation) =>
        WideArithmetic.AddSigned704(
            WideArithmetic.AddSigned704(
                WideArithmetic.MultiplySigned576ToSigned704(
                    axis.X,
                    twiceTranslation.X),
                WideArithmetic.MultiplySigned576ToSigned704(
                    axis.Y,
                    twiceTranslation.Y)),
            WideArithmetic.MultiplySigned576ToSigned704(
                axis.Z,
                twiceTranslation.Z));

    private static Signed704 GetCapsuleBasisProjection(
        in CapsuleAxis3 axis,
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
        in CapsuleAxis3 axis,
        in WideAxis3 direction) =>
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
        in CapsuleAxis3 axis) =>
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

    private static CapsuleAxis3 ToCapsuleAxis(in WideAxis3 axis) =>
        new(
            Signed576.ExtendValue(axis.X),
            Signed576.ExtendValue(axis.Y),
            Signed576.ExtendValue(axis.Z));
}
