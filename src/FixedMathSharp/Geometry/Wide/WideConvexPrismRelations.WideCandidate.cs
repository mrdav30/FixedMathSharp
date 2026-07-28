//=======================================================================
// WideConvexPrismRelations.WideCandidate.cs
//=======================================================================
// MIT License, Copyright (c) 2024-present David Oravsky (mrdav30)
// See LICENSE file in the project root for full license information.
//=======================================================================

using System;

namespace FixedMathSharp.Geometry;

/// <content>
/// Wide-precision candidate axis helpers for cylinder/capsule separating-axis
/// tests, using high-word arithmetic to build and evaluate axis projections
/// without precision loss.
/// </content>
internal static partial class WideConvexPrismRelations
{
    private static bool TryKeepWideCylinderCapsuleAxisFallback(
        WideCandidateAxis3 candidate,
        Vector3d cylinderCenter,
        RigidAxis3 cylinderAxis,
        Fixed64 cylinderLength,
        Fixed64 cylinderRadius,
        Vector3d capsuleCenter,
        RigidAxis3 capsuleAxis,
        Fixed64 capsuleLength,
        Fixed64 capsuleRadius,
        ref CylinderCapsulePenetration best)
    {
        Span<ulong> axisSquared =
            stackalloc ulong[WideCandidateWords];
        BuildWideAxisSquared(candidate, axisSquared);
        Span<ulong> cylinderAlignment =
            stackalloc ulong[WideCandidateWords];
        BuildWideDot(
            candidate,
            cylinderAxis.X,
            cylinderAxis.Y,
            cylinderAxis.Z,
            cylinderAlignment,
            out int cylinderAlignmentSign);
        Span<ulong> capsuleAlignment =
            stackalloc ulong[WideCandidateWords];
        BuildWideDot(
            candidate,
            capsuleAxis.X,
            capsuleAxis.Y,
            capsuleAxis.Z,
            capsuleAlignment,
            out int capsuleAlignmentSign);
        Span<ulong> centerProjection =
            stackalloc ulong[WideCandidateWords];
        BuildWideDot(
            candidate,
            WideArithmetic.SubtractSigned192(
                Signed192.Raw(capsuleCenter.X),
                Signed192.Raw(cylinderCenter.X)),
            WideArithmetic.SubtractSigned192(
                Signed192.Raw(capsuleCenter.Y),
                Signed192.Raw(cylinderCenter.Y)),
            WideArithmetic.SubtractSigned192(
                Signed192.Raw(capsuleCenter.Z),
                Signed192.Raw(cylinderCenter.Z)),
            centerProjection,
            out _);

        Span<ulong> cylinderAxial =
            stackalloc ulong[WideCandidateWords];
        BuildWideScaledAlignment(
            cylinderAlignment,
            cylinderLength,
            capsuleAxis.RotationDenominator,
            cylinderAxial);
        Span<ulong> capsuleAxial =
            stackalloc ulong[WideCandidateWords];
        BuildWideScaledAlignment(
            capsuleAlignment,
            capsuleLength,
            cylinderAxis.RotationDenominator,
            capsuleAxial);
        Span<ulong> scaledCenter =
            stackalloc ulong[WideCandidateWords];
        BuildWideScaledCenter(
            centerProjection,
            cylinderAxis.RotationDenominator,
            capsuleAxis.RotationDenominator,
            scaledCenter);
        Span<ulong> rational =
            stackalloc ulong[WideCandidateWords];
        Span<ulong> axialSum =
            stackalloc ulong[WideCandidateWords];
        AddMagnitudes(
            cylinderAxial,
            capsuleAxial,
            axialSum);
        CombineWideSignedMagnitudes(
            axialSum,
            1,
            scaledCenter,
            -1,
            rational,
            out int rationalSign);
        // A non-narrowable closest-point residual is nonzero. The endpoint
        // KKT relation makes support minus center -K * |residual|^2, K > 0.

        Signed320 cylinderAxisSquared =
            GetAxisSquared(cylinderAxis);
        Span<ulong> planeSquared =
            stackalloc ulong[WideCandidateWords];
        BuildWidePlaneSquared(
            axisSquared,
            cylinderAxisSquared,
            cylinderAlignment,
            planeSquared);
        Signed576 commonWide = WideArithmetic.MultiplySigned576(
            Signed576.ExtendValue(
                WideArithmetic.MultiplySigned192(
                    cylinderAxis.RotationDenominator,
                    capsuleAxis.RotationDenominator)),
            Signed192.Raw(Fixed64.Two));
        _ = Signed192.TryNarrowSigned(
            commonWide,
            out Signed192 common);
        var depth = new WideCylinderCapsuleDepth(
            rational,
            rationalSign,
            common,
            cylinderRadius,
            axisSquared,
            cylinderAxisSquared,
            planeSquared);
        if (!IsWideCylinderCapsuleProjectionNonNegative(
                depth,
                capsuleRadius))
        {
            return false;
        }

        if (CompareWideProjectionDepth(
                depth,
                best.ExactDepth) >= 0)
        {
            return true;
        }

        GetRoundedWideCylinderCapsuleDepth(
            depth,
            capsuleRadius,
            out Fixed64 roundedDepth,
            out bool depthIsClamped);
        Vector3d normal = GetWideCandidateNormal(candidate);
        best = new CylinderCapsulePenetration(
            normal,
            roundedDepth,
            depthIsClamped);
        return true;
    }

    private static void BuildWideAxisSquared(
        WideCandidateAxis3 axis,
        Span<ulong> result)
    {
        Span<ulong> xSquared =
            stackalloc ulong[WideCandidateWords];
        Span<ulong> ySquared =
            stackalloc ulong[WideCandidateWords];
        Span<ulong> zSquared =
            stackalloc ulong[WideCandidateWords];
        Span<ulong> sum =
            stackalloc ulong[WideCandidateWords];
        MultiplyMagnitudes(axis.X, axis.X, xSquared);
        MultiplyMagnitudes(axis.Y, axis.Y, ySquared);
        MultiplyMagnitudes(axis.Z, axis.Z, zSquared);
        AddMagnitudes(xSquared, ySquared, sum);
        AddMagnitudes(sum, zSquared, result);
    }

    private static void BuildWideDot(
        WideCandidateAxis3 axis,
        Signed192 x,
        Signed192 y,
        Signed192 z,
        Span<ulong> result,
        out int sign)
    {
        Span<ulong> xMagnitude = stackalloc ulong[3];
        Span<ulong> yMagnitude = stackalloc ulong[3];
        Span<ulong> zMagnitude = stackalloc ulong[3];
        WideArithmetic.GetMagnitude(
            x,
            out xMagnitude[2],
            out xMagnitude[1],
            out xMagnitude[0]);
        WideArithmetic.GetMagnitude(
            y,
            out yMagnitude[2],
            out yMagnitude[1],
            out yMagnitude[0]);
        WideArithmetic.GetMagnitude(
            z,
            out zMagnitude[2],
            out zMagnitude[1],
            out zMagnitude[0]);
        Span<ulong> xTerm =
            stackalloc ulong[WideCandidateWords];
        Span<ulong> yTerm =
            stackalloc ulong[WideCandidateWords];
        Span<ulong> zTerm =
            stackalloc ulong[WideCandidateWords];
        Span<ulong> xy =
            stackalloc ulong[WideCandidateWords];
        MultiplyMagnitudes(axis.X, xMagnitude, xTerm);
        MultiplyMagnitudes(axis.Y, yMagnitude, yTerm);
        MultiplyMagnitudes(axis.Z, zMagnitude, zTerm);
        CombineWideSignedMagnitudes(
            xTerm,
            axis.XSign * x.Sign,
            yTerm,
            axis.YSign * y.Sign,
            xy,
            out int xySign);
        CombineWideSignedMagnitudes(
            xy,
            xySign,
            zTerm,
            axis.ZSign * z.Sign,
            result,
            out sign);
    }

    private static void BuildWideScaledAlignment(
        ReadOnlySpan<ulong> alignment,
        Fixed64 length,
        Signed192 otherDenominator,
        Span<ulong> result)
    {
        Span<ulong> lengthMagnitude = stackalloc ulong[3];
        Span<ulong> denominatorMagnitude = stackalloc ulong[3];
        WideArithmetic.GetMagnitude(
            Signed192.Raw(length),
            out lengthMagnitude[2],
            out lengthMagnitude[1],
            out lengthMagnitude[0]);
        WideArithmetic.GetMagnitude(
            otherDenominator,
            out denominatorMagnitude[2],
            out denominatorMagnitude[1],
            out denominatorMagnitude[0]);
        Span<ulong> withLength =
            stackalloc ulong[WideCandidateWords];
        MultiplyMagnitudes(
            alignment,
            lengthMagnitude,
            withLength);
        MultiplyMagnitudes(
            withLength,
            denominatorMagnitude,
            result);
    }

    private static void BuildWideScaledCenter(
        ReadOnlySpan<ulong> projection,
        Signed192 firstDenominator,
        Signed192 secondDenominator,
        Span<ulong> result)
    {
        Span<ulong> firstMagnitude = stackalloc ulong[3];
        Span<ulong> secondMagnitude = stackalloc ulong[3];
        Span<ulong> twiceMagnitude = stackalloc ulong[3];
        WideArithmetic.GetMagnitude(
            firstDenominator,
            out firstMagnitude[2],
            out firstMagnitude[1],
            out firstMagnitude[0]);
        WideArithmetic.GetMagnitude(
            secondDenominator,
            out secondMagnitude[2],
            out secondMagnitude[1],
            out secondMagnitude[0]);
        WideArithmetic.GetMagnitude(
            Signed192.Raw(Fixed64.Two),
            out twiceMagnitude[2],
            out twiceMagnitude[1],
            out twiceMagnitude[0]);
        Span<ulong> firstProduct =
            stackalloc ulong[WideCandidateWords];
        Span<ulong> secondProduct =
            stackalloc ulong[WideCandidateWords];
        MultiplyMagnitudes(
            projection,
            firstMagnitude,
            firstProduct);
        MultiplyMagnitudes(
            firstProduct,
            secondMagnitude,
            secondProduct);
        MultiplyMagnitudes(
            secondProduct,
            twiceMagnitude,
            result);
    }

    private static void BuildWidePlaneSquared(
        ReadOnlySpan<ulong> axisSquared,
        Signed320 shapeAxisSquared,
        ReadOnlySpan<ulong> alignment,
        Span<ulong> result)
    {
        Span<ulong> shapeAxisMagnitude = stackalloc ulong[5];
        GetMagnitude(shapeAxisSquared, shapeAxisMagnitude);
        Span<ulong> first =
            stackalloc ulong[WideCandidateWords];
        Span<ulong> second =
            stackalloc ulong[WideCandidateWords];
        MultiplyMagnitudes(
            axisSquared,
            shapeAxisMagnitude,
            first);
        MultiplyMagnitudes(
            alignment,
            alignment,
            second);
        SubtractMagnitudes(first, second, result);
    }

    private static bool IsWideCylinderCapsuleProjectionNonNegative(
        WideCylinderCapsuleDepth depth,
        Fixed64 capsuleRadius)
    {
        Span<ulong> disk =
            stackalloc ulong[WideCandidateWords];
        Span<ulong> capsule =
            stackalloc ulong[WideCandidateWords];
        Span<ulong> rational =
            stackalloc ulong[WideCandidateWords];
        Span<ulong> zero =
            stackalloc ulong[WideCandidateWords];
        BuildWideDiskRadicand(depth, disk);
        BuildWideCapsuleRadicand(
            depth,
            capsuleRadius,
            capsule);
        BuildWideRationalRadicand(depth, rational);
        zero.Clear();
        return CompareWideRadicalPairs(
            disk,
            capsule,
            rational,
            zero) >= 0;
    }

    private static bool IsWideBaseProjectionNonNegative(
        WideCylinderCapsuleDepth depth)
    {
        Span<ulong> disk =
            stackalloc ulong[WideCandidateWords];
        Span<ulong> rational =
            stackalloc ulong[WideCandidateWords];
        BuildWideDiskRadicand(depth, disk);
        BuildWideRationalRadicand(depth, rational);
        return CompareMagnitude(disk, rational) >= 0;
    }

    private static int CompareWideProjectionDepth(
        WideCylinderCapsuleDepth left,
        in ProjectionDepth right)
    {
        Span<ulong> radicands = stackalloc ulong[
            WideCandidateWords * 4];
        Span<int> signs = stackalloc int[4];
        radicands.Clear();
        signs.Clear();
        BuildWideRationalRadicand(
            left,
            right.AxisSquared,
            radicands.Slice(
                0,
                WideCandidateWords));
        signs[0] = left.RationalSign;
        BuildWideDiskRadicand(
            left,
            right.AxisSquared,
            radicands.Slice(
                WideCandidateWords,
                WideCandidateWords));
        signs[1] = 1;
        BuildNarrowRationalRadicand(
            right,
            left.AxisSquared,
            radicands.Slice(
                WideCandidateWords * 2,
                WideCandidateWords));
        signs[2] = -right.Rational.Sign;
        BuildNarrowDiskRadicand(
            right,
            left.AxisSquared,
            radicands.Slice(
                WideCandidateWords * 3,
                WideCandidateWords));
        signs[3] = -1;
        return WideArithmetic.GetLinearRadicalSumSign(
            radicands,
            WideCandidateWords,
            signs);
    }

    private static void BuildWideRationalRadicand(
        WideCylinderCapsuleDepth depth,
        Span<ulong> result)
    {
        Span<ulong> shapeAxisMagnitude = stackalloc ulong[5];
        GetMagnitude(
            depth.ShapeAxisSquared,
            shapeAxisMagnitude);
        Span<ulong> square =
            stackalloc ulong[WideCandidateWords];
        MultiplyMagnitudes(
            depth.Rational,
            depth.Rational,
            square);
        MultiplyMagnitudes(
            square,
            shapeAxisMagnitude,
            result);
    }

    private static void BuildWideRationalRadicand(
        WideCylinderCapsuleDepth depth,
        Signed576 otherAxisSquared,
        Span<ulong> result)
    {
        Span<ulong> baseRadicand =
            stackalloc ulong[WideCandidateWords];
        Span<ulong> otherMagnitude = stackalloc ulong[9];
        BuildWideRationalRadicand(
            depth,
            baseRadicand);
        WideArithmetic.GetMagnitude(
            otherAxisSquared,
            otherMagnitude);
        MultiplyMagnitudes(
            baseRadicand,
            otherMagnitude,
            result);
    }

    private static void BuildWideDiskRadicand(
        WideCylinderCapsuleDepth depth,
        Span<ulong> result)
    {
        Span<ulong> coefficient = stackalloc ulong[5];
        GetWideRadialCoefficient(
            depth.Common,
            depth.Radius,
            coefficient);
        Span<ulong> coefficientSquared =
            stackalloc ulong[WideCandidateWords];
        MultiplyMagnitudes(
            coefficient,
            coefficient,
            coefficientSquared);
        MultiplyMagnitudes(
            coefficientSquared,
            depth.PlaneSquared,
            result);
    }

    private static void BuildWideDiskRadicand(
        WideCylinderCapsuleDepth depth,
        Signed576 otherAxisSquared,
        Span<ulong> result)
    {
        Span<ulong> baseRadicand =
            stackalloc ulong[WideCandidateWords];
        Span<ulong> otherMagnitude = stackalloc ulong[9];
        BuildWideDiskRadicand(
            depth,
            baseRadicand);
        WideArithmetic.GetMagnitude(
            otherAxisSquared,
            otherMagnitude);
        MultiplyMagnitudes(
            baseRadicand,
            otherMagnitude,
            result);
    }

    private static void BuildWideCapsuleRadicand(
        WideCylinderCapsuleDepth depth,
        Fixed64 radius,
        Span<ulong> result)
    {
        Span<ulong> coefficient = stackalloc ulong[5];
        Span<ulong> shapeAxisMagnitude = stackalloc ulong[5];
        GetWideRadialCoefficient(
            depth.Common,
            radius,
            coefficient);
        GetMagnitude(
            depth.ShapeAxisSquared,
            shapeAxisMagnitude);
        Span<ulong> coefficientSquared =
            stackalloc ulong[WideCandidateWords];
        Span<ulong> withShapeAxis =
            stackalloc ulong[WideCandidateWords];
        MultiplyMagnitudes(
            coefficient,
            coefficient,
            coefficientSquared);
        MultiplyMagnitudes(
            coefficientSquared,
            shapeAxisMagnitude,
            withShapeAxis);
        MultiplyMagnitudes(
            withShapeAxis,
            depth.AxisSquared,
            result);
    }

    private static void BuildNarrowRationalRadicand(
        in ProjectionDepth depth,
        ReadOnlySpan<ulong> otherAxisSquared,
        Span<ulong> result)
    {
        Span<ulong> rationalMagnitude = stackalloc ulong[11];
        Span<ulong> shapeAxisMagnitude = stackalloc ulong[5];
        WideArithmetic.GetMagnitude(
            depth.Rational,
            rationalMagnitude);
        GetMagnitude(
            depth.ShapeAxisSquared,
            shapeAxisMagnitude);
        Span<ulong> square =
            stackalloc ulong[WideCandidateWords];
        Span<ulong> withShapeAxis =
            stackalloc ulong[WideCandidateWords];
        MultiplyMagnitudes(
            rationalMagnitude,
            rationalMagnitude,
            square);
        MultiplyMagnitudes(
            square,
            shapeAxisMagnitude,
            withShapeAxis);
        MultiplyMagnitudes(
            withShapeAxis,
            otherAxisSquared,
            result);
    }

    private static void BuildNarrowDiskRadicand(
        in ProjectionDepth depth,
        ReadOnlySpan<ulong> otherAxisSquared,
        Span<ulong> result)
    {
        Span<ulong> coefficient = stackalloc ulong[5];
        Span<ulong> planeMagnitude = stackalloc ulong[13];
        GetWideRadialCoefficient(
            depth.Common,
            depth.Radius,
            coefficient);
        WideArithmetic.GetMagnitude(
            depth.PlaneSquared,
            planeMagnitude);
        Span<ulong> coefficientSquared =
            stackalloc ulong[WideCandidateWords];
        Span<ulong> withPlane =
            stackalloc ulong[WideCandidateWords];
        MultiplyMagnitudes(
            coefficient,
            coefficient,
            coefficientSquared);
        MultiplyMagnitudes(
            coefficientSquared,
            planeMagnitude,
            withPlane);
        MultiplyMagnitudes(
            withPlane,
            otherAxisSquared,
            result);
    }

    private static void GetWideRadialCoefficient(
        Signed192 common,
        Fixed64 radius,
        Span<ulong> result)
    {
        Signed320 coefficient =
            WideArithmetic.MultiplySigned192(
                common,
                Signed192.Raw(radius));
        GetMagnitude(coefficient, result);
    }

}
