//=======================================================================
// WideOrientedBox.ConvexHullCapsule.Support.cs
//=======================================================================
// MIT License, Copyright (c) 2024-present David Oravsky (mrdav30)
// See LICENSE file in the project root for full license information.
//=======================================================================

using System;

namespace FixedMathSharp.Geometry;

/// <content>
/// Support utilities for convex-hull vs capsule tests: candidate feature
/// classification and wide, fixed-point projection interval computation
/// along a capsule axis within a rational basis.
/// </content>
internal static partial class WideOrientedBox
{
    #region Nested Types

    private enum HullCapsuleCandidateKind
    {
        Face,
        EdgeCross,
        VertexCore,
        EndpointEdge,
    }

    private readonly struct HullCapsuleCandidate
    {
        internal readonly HullCapsuleCandidateKind Kind;
        internal readonly int Index;
        internal readonly int EndpointSign;

        internal HullCapsuleCandidate(
            HullCapsuleCandidateKind kind,
            int index,
            int endpointSign)
        {
            Kind = kind;
            Index = index;
            EndpointSign = endpointSign;
        }
    }

    #endregion

    private static void GetHullCapsuleProjectionInterval(
        ReadOnlySpan<Vector3d> points,
        RationalBasis basis,
        CapsuleAxis3 axis,
        Span<ulong> minimum,
        out int minimumSign,
        Span<ulong> maximum,
        out int maximumSign)
    {
        BuildHullCapsuleLocalProjection(
            points[0],
            basis,
            axis,
            minimum,
            out minimumSign);
        minimum.CopyTo(maximum);
        maximumSign = minimumSign;
        Span<ulong> candidate =
            stackalloc ulong[HullCapsuleMagnitudeWords];
        for (int index = 1; index < points.Length; index++)
        {
            BuildHullCapsuleLocalProjection(
                points[index],
                basis,
                axis,
                candidate,
                out int candidateSign);
            if (CompareHullCapsuleSignedMagnitudes(
                    candidate,
                    candidateSign,
                    minimum,
                    minimumSign) < 0)
            {
                candidate.CopyTo(minimum);
                minimumSign = candidateSign;
            }
            if (CompareHullCapsuleSignedMagnitudes(
                    candidate,
                    candidateSign,
                    maximum,
                    maximumSign) > 0)
            {
                candidate.CopyTo(maximum);
                maximumSign = candidateSign;
            }
        }
    }

    private static void BuildHullCapsuleLocalProjection(
        Vector3d point,
        RationalBasis basis,
        CapsuleAxis3 axis,
        Span<ulong> result,
        out int resultSign)
    {
        Span<ulong> x =
            stackalloc ulong[HullCapsuleMagnitudeWords];
        Span<ulong> y =
            stackalloc ulong[HullCapsuleMagnitudeWords];
        Span<ulong> z =
            stackalloc ulong[HullCapsuleMagnitudeWords];
        BuildHullCapsuleProjectionTerm(
            GetCapsuleBasisProjection(
                axis,
                basis.Xx,
                basis.Xy,
                basis.Xz),
            Signed192.Raw(point.X),
            x,
            out int xSign);
        BuildHullCapsuleProjectionTerm(
            GetCapsuleBasisProjection(
                axis,
                basis.Yx,
                basis.Yy,
                basis.Yz),
            Signed192.Raw(point.Y),
            y,
            out int ySign);
        BuildHullCapsuleProjectionTerm(
            GetCapsuleBasisProjection(
                axis,
                basis.Zx,
                basis.Zy,
                basis.Zz),
            Signed192.Raw(point.Z),
            z,
            out int zSign);
        Span<ulong> xy =
            stackalloc ulong[HullCapsuleMagnitudeWords];
        CombineHullCapsuleSignedMagnitudes(
            x,
            xSign,
            y,
            ySign,
            xy,
            out int xySign);
        CombineHullCapsuleSignedMagnitudes(
            xy,
            xySign,
            z,
            zSign,
            result,
            out resultSign);
    }

    private static void BuildHullCapsuleProjectionTerm(
        Signed704 projection,
        Signed192 coordinate,
        Span<ulong> result,
        out int resultSign)
    {
        Span<ulong> projectionMagnitude =
            stackalloc ulong[HullCapsuleMagnitudeWords];
        projectionMagnitude.Clear();
        GetHullCapsuleMagnitude(
            projection,
            projectionMagnitude);
        Span<ulong> coordinateMagnitude =
            stackalloc ulong[HullCapsuleMagnitudeWords];
        GetHullCapsuleMagnitude(
            coordinate,
            coordinateMagnitude);
        WideArithmetic.MultiplyMagnitudes(
            projectionMagnitude,
            coordinateMagnitude,
            result);
        resultSign = projection.Sign * coordinate.Sign;
    }

    private static Signed704 GetHullCapsuleCenterProjection(
        Vector3d end,
        Vector3d start,
        CapsuleAxis3 axis) =>
        WideArithmetic.AddSigned704(
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
                        Signed192.Raw(start.Z)))));

    private static void BuildHullCapsuleAxisSquared(
        CapsuleAxis3 axis,
        Span<ulong> result)
    {
        Span<ulong> component =
            stackalloc ulong[HullCapsuleMagnitudeWords];
        Span<ulong> square =
            stackalloc ulong[HullCapsuleMagnitudeWords];
        result.Clear();
        GetHullCapsuleMagnitude(axis.X, component);
        WideArithmetic.MultiplyMagnitudes(
            component,
            component,
            square);
        WideArithmetic.AddMagnitudeInto(square, result);
        GetHullCapsuleMagnitude(axis.Y, component);
        WideArithmetic.MultiplyMagnitudes(
            component,
            component,
            square);
        WideArithmetic.AddMagnitudeInto(square, result);
        GetHullCapsuleMagnitude(axis.Z, component);
        WideArithmetic.MultiplyMagnitudes(
            component,
            component,
            square);
        WideArithmetic.AddMagnitudeInto(square, result);
    }

    private static FixedPointAnchor GetMatchedHullCapsuleSupportAnchor(
        Vector3d hullOrigin,
        FixedQuaternion hullRotation,
        RationalBasis basis,
        ReadOnlySpan<Vector3d> points,
        ReadOnlySpan<int> triangleVertexIndices,
        ReadOnlySpan<int> edgeVertexPairs,
        CapsuleAxis3 orientedAxis,
        in FixedPointAnchor target)
    {
        int supportIndex = 0;
        Span<ulong> best =
            stackalloc ulong[HullCapsuleMagnitudeWords];
        BuildHullCapsuleLocalProjection(
            points[0],
            basis,
            orientedAxis,
            best,
            out int bestSign);
        Span<ulong> candidate =
            stackalloc ulong[HullCapsuleMagnitudeWords];
        for (int index = 1; index < points.Length; index++)
        {
            BuildHullCapsuleLocalProjection(
                points[index],
                basis,
                orientedAxis,
                candidate,
                out int candidateSign);
            if (CompareHullCapsuleSignedMagnitudes(
                    candidate,
                    candidateSign,
                    best,
                    bestSign) <= 0)
            {
                continue;
            }
            supportIndex = index;
            candidate.CopyTo(best);
            bestSign = candidateSign;
        }

        FixedPointAnchor closest = new(
            hullOrigin,
            hullRotation,
            points[supportIndex]);
        for (int index = 0;
            index < triangleVertexIndices.Length;
            index += 3)
        {
            Vector3d first = points[
                triangleVertexIndices[index]];
            Vector3d second = points[
                triangleVertexIndices[index + 1]];
            Vector3d third = points[
                triangleVertexIndices[index + 2]];
            if (!IsHullCapsuleSupportPoint(
                    first,
                    basis,
                    orientedAxis,
                    best,
                    bestSign)
                || !IsHullCapsuleSupportPoint(
                    second,
                    basis,
                    orientedAxis,
                    best,
                    bestSign)
                || !IsHullCapsuleSupportPoint(
                    third,
                    basis,
                    orientedAxis,
                    best,
                    bestSign))
            {
                continue;
            }

            FixedPointAnchor candidateAnchor =
                new FixedTriangle(first, second, third)
                    .GetClosestPointAnchor(
                        hullOrigin,
                        hullRotation,
                        target);
            if (target.CompareSquaredDistance(
                    candidateAnchor,
                    closest) < 0)
            {
                closest = candidateAnchor;
            }
        }

        for (int index = 0;
            index < edgeVertexPairs.Length;
            index += 2)
        {
            Vector3d start =
                points[edgeVertexPairs[index]];
            Vector3d end =
                points[edgeVertexPairs[index + 1]];
            if (!IsHullCapsuleSupportPoint(
                    start,
                    basis,
                    orientedAxis,
                    best,
                    bestSign)
                || !IsHullCapsuleSupportPoint(
                    end,
                    basis,
                    orientedAxis,
                    best,
                    bestSign))
            {
                continue;
            }

            FixedPointAnchor candidateAnchor =
                new FixedTriangle(start, end, end)
                    .GetClosestPointAnchor(
                        hullOrigin,
                        hullRotation,
                        target);
            if (target.CompareSquaredDistance(
                    candidateAnchor,
                    closest) < 0)
            {
                closest = candidateAnchor;
            }
        }
        return closest;
    }

    private static bool IsHullCapsuleSupportPoint(
        Vector3d point,
        RationalBasis basis,
        CapsuleAxis3 orientedAxis,
        ReadOnlySpan<ulong> supportProjection,
        int supportProjectionSign)
    {
        Span<ulong> projection =
            stackalloc ulong[HullCapsuleMagnitudeWords];
        BuildHullCapsuleLocalProjection(
            point,
            basis,
            orientedAxis,
            projection,
            out int projectionSign);
        return CompareHullCapsuleSignedMagnitudes(
            projection,
            projectionSign,
            supportProjection,
            supportProjectionSign) == 0;
    }

    private static void GetHullCapsuleDepth(
        ReadOnlySpan<ulong> rational,
        int rationalSign,
        ReadOnlySpan<ulong> squaredAxisLength,
        Signed192 commonDenominator,
        Fixed64 radius,
        out Fixed64 depth,
        out bool depthIsClamped)
    {
        ulong floor = 0UL;
        ulong high = unchecked((ulong)long.MaxValue);
        while (floor < high)
        {
            ulong midpoint = floor + ((high - floor + 1UL) >> 1);
            int comparison = CompareHullCapsuleDepthToTwiceRaw(
                rational,
                rationalSign,
                squaredAxisLength,
                commonDenominator,
                radius,
                new Signed192(0UL, 0UL, midpoint << 1));
            if (comparison >= 0)
            {
                floor = midpoint;
            }
            else
            {
                high = midpoint - 1UL;
            }
        }

        if (floor == unchecked((ulong)long.MaxValue))
        {
            depth = Fixed64.MaxValue;
            depthIsClamped =
                CompareHullCapsuleDepthToTwiceRaw(
                    rational,
                    rationalSign,
                    squaredAxisLength,
                    commonDenominator,
                    radius,
                    new Signed192(
                        0UL,
                        0UL,
                        unchecked((ulong)long.MaxValue << 1))) > 0;
            return;
        }

        int midpointComparison =
            CompareHullCapsuleDepthToTwiceRaw(
                rational,
                rationalSign,
                squaredAxisLength,
                commonDenominator,
                radius,
                new Signed192(
                    0UL,
                    0UL,
                    (floor << 1) | 1UL));
        // The comparator is -1, 0, or 1. Adding one plus the floor parity
        // encodes greater-than-half and midpoint-to-even as a 0/1 increment.
        ulong roundingIncrement =
            (ulong)((midpointComparison + 1 + (int)(floor & 1UL)) >> 1);
        depth = Fixed64.FromRaw(
            (long)(floor + roundingIncrement));
        depthIsClamped = false;
    }

    private static int CompareHullCapsuleDepthToTwiceRaw(
        ReadOnlySpan<ulong> rational,
        int rationalSign,
        ReadOnlySpan<ulong> squaredAxisLength,
        Signed192 commonDenominator,
        Fixed64 radius,
        Signed192 twiceRaw)
    {
        Span<ulong> twiceRational =
            stackalloc ulong[HullCapsuleMagnitudeWords];
        rational.CopyTo(twiceRational);
        WideArithmetic.AddMagnitudeInto(
            rational,
            twiceRational);
        Signed192 radialDelta = WideArithmetic.SubtractSigned192(
            WideArithmetic.AddSigned192(
                Signed192.Raw(radius),
                Signed192.Raw(radius)),
            twiceRaw);
        Signed320 coefficient = WideArithmetic.MultiplySigned192(
            commonDenominator,
            radialDelta);
        Span<ulong> coefficientMagnitude =
            stackalloc ulong[HullCapsuleMagnitudeWords];
        GetHullCapsuleMagnitude(
            coefficient,
            coefficientMagnitude);
        return WideArithmetic.GetSignedMagnitudeAndSquareRootSign(
            twiceRational,
            rationalSign,
            coefficientMagnitude,
            coefficient.Sign,
            squaredAxisLength);
    }

    private static void MultiplyHullCapsuleMagnitude(
        Span<ulong> value,
        Signed192 factor,
        Span<ulong> result)
    {
        Span<ulong> factorMagnitude =
            stackalloc ulong[HullCapsuleMagnitudeWords];
        Span<ulong> product =
            stackalloc ulong[HullCapsuleMagnitudeWords];
        GetHullCapsuleMagnitude(factor, factorMagnitude);
        WideArithmetic.MultiplyMagnitudes(
            value,
            factorMagnitude,
            product);
        product.CopyTo(result);
    }

    private static void MultiplyHullCapsuleMagnitude(
        Span<ulong> value,
        Signed320 factor,
        Span<ulong> result)
    {
        Span<ulong> factorMagnitude =
            stackalloc ulong[HullCapsuleMagnitudeWords];
        Span<ulong> product =
            stackalloc ulong[HullCapsuleMagnitudeWords];
        GetHullCapsuleMagnitude(factor, factorMagnitude);
        WideArithmetic.MultiplyMagnitudes(
            value,
            factorMagnitude,
            product);
        product.CopyTo(result);
    }

    private static void GetHullCapsuleMagnitude(
        Signed192 value,
        Span<ulong> result)
    {
        result.Clear();
        WideArithmetic.GetMagnitude(
            value,
            out result[2],
            out result[1],
            out result[0]);
    }

    private static void GetHullCapsuleMagnitude(
        Signed320 value,
        Span<ulong> result)
    {
        result.Clear();
        WideArithmetic.GetMagnitude(
            value,
            out result[4],
            out result[3],
            out result[2],
            out result[1],
            out result[0]);
    }

    private static void GetHullCapsuleMagnitude(
        Signed576 value,
        Span<ulong> result)
    {
        result.Clear();
        Span<ulong> magnitude = stackalloc ulong[9];
        WideArithmetic.GetMagnitude(value, magnitude);
        magnitude.CopyTo(result);
    }

    private static void GetHullCapsuleMagnitude(
        Signed704 value,
        Span<ulong> result)
    {
        result.Clear();
        Span<ulong> magnitude = stackalloc ulong[11];
        WideArithmetic.GetMagnitude(value, magnitude);
        magnitude.CopyTo(result);
    }

    private static int CompareHullCapsuleSignedMagnitudes(
        ReadOnlySpan<ulong> left,
        int leftSign,
        ReadOnlySpan<ulong> right,
        int rightSign)
    {
        if (leftSign != rightSign)
            return leftSign < rightSign ? -1 : 1;
        if (leftSign == 0)
            return 0;
        int comparison =
            WideArithmetic.CompareMagnitudeEqualLength(left, right);
        return leftSign > 0 ? comparison : -comparison;
    }

    private static void CombineHullCapsuleSignedMagnitudes(
        ReadOnlySpan<ulong> left,
        int leftSign,
        ReadOnlySpan<ulong> right,
        int rightSign,
        Span<ulong> result,
        out int resultSign)
    {
        if (leftSign == 0
            || WideArithmetic.IsZeroMagnitude(left))
        {
            right.CopyTo(result);
            resultSign = WideArithmetic.IsZeroMagnitude(right)
                ? 0
                : rightSign;
            return;
        }
        if (rightSign == 0
            || WideArithmetic.IsZeroMagnitude(right))
        {
            left.CopyTo(result);
            resultSign = leftSign;
            return;
        }
        if (leftSign == rightSign)
        {
            left.CopyTo(result);
            WideArithmetic.AddMagnitudeInto(right, result);
            resultSign = leftSign;
            return;
        }

        int comparison =
            WideArithmetic.CompareMagnitudeEqualLength(left, right);
        if (comparison == 0)
        {
            result.Clear();
            resultSign = 0;
        }
        else if (comparison > 0)
        {
            WideArithmetic.SubtractEqualMagnitudes(
                left,
                right,
                result);
            resultSign = leftSign;
        }
        else
        {
            WideArithmetic.SubtractEqualMagnitudes(
                right,
                left,
                result);
            resultSign = rightSign;
        }
    }
}
