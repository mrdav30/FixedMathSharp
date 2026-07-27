//=======================================================================
// WideConvexPrismRelations.RigidCylinderPairs.cs
//=======================================================================
// MIT License, Copyright (c) 2024-present David Oravsky (mrdav30)
// See LICENSE file in the project root for full license information.
//=======================================================================

using System;

namespace FixedMathSharp.Bounds;

/// <content>
/// Contains routines for computing contact information between pairs of rigid, finite cylinders.
/// </content>
internal static partial class WideConvexPrismRelations
{
    internal static bool TryGetCenteredFiniteCylindersContact(
        Vector3d firstCenter,
        FixedQuaternion firstRotation,
        Vector3d firstLocalAxis,
        Fixed64 firstLength,
        Fixed64 firstRadius,
        Vector3d secondCenter,
        FixedQuaternion secondRotation,
        Vector3d secondLocalAxis,
        Fixed64 secondLength,
        Fixed64 secondRadius,
        out FixedContactAnchors contact) =>
        TryGetCenteredFiniteCylindersContact(
            firstCenter,
            firstRotation,
            firstLocalAxis,
            firstLength,
            firstRadius,
            secondCenter,
            secondRotation,
            secondLocalAxis,
            secondLength,
            secondRadius,
            out contact,
            out _,
            out _);

    internal static bool TryGetCenteredFiniteCylindersContact(
        Vector3d firstCenter,
        FixedQuaternion firstRotation,
        Vector3d firstLocalAxis,
        Fixed64 firstLength,
        Fixed64 firstRadius,
        Vector3d secondCenter,
        FixedQuaternion secondRotation,
        Vector3d secondLocalAxis,
        Fixed64 secondLength,
        Fixed64 secondRadius,
        out FixedContactAnchors contact,
        out bool usedWideCandidate,
        out bool usedMultiRadicalRanking)
    {
        usedWideCandidate = false;
        usedMultiRadicalRanking = false;
        WideOrientedBox.GetRotatedLocalAxisNumerators(
            firstRotation,
            firstLocalAxis,
            out Signed192 firstAxisX,
            out Signed192 firstAxisY,
            out Signed192 firstAxisZ,
            out Signed192 firstDenominator);
        WideOrientedBox.GetRotatedLocalAxisNumerators(
            secondRotation,
            secondLocalAxis,
            out Signed192 secondAxisX,
            out Signed192 secondAxisY,
            out Signed192 secondAxisZ,
            out Signed192 secondDenominator);
        var firstAxis = new RigidAxis3(
            firstAxisX,
            firstAxisY,
            firstAxisZ,
            firstDenominator);
        var secondAxis = new RigidAxis3(
            secondAxisX,
            secondAxisY,
            secondAxisZ,
            secondDenominator);
        Axis3 firstCandidate = firstAxis.ToWide();
        Axis3 secondCandidate = secondAxis.ToWide();
        var best = default(CylinderCylinderPenetration);
        if (!TryKeepCylinderCylinderAxis(
                firstCandidate,
                firstCenter,
                firstAxis,
                firstLength,
                firstRadius,
                secondCenter,
                secondAxis,
                secondLength,
                secondRadius,
                ref best)
            || !TryKeepCylinderCylinderAxis(
                secondCandidate,
                firstCenter,
                firstAxis,
                firstLength,
                firstRadius,
                secondCenter,
                secondAxis,
                secondLength,
                secondRadius,
                ref best)
            || !TryKeepCylinderCylinderAxis(
                Cross(firstCandidate, secondCandidate),
                firstCenter,
                firstAxis,
                firstLength,
                firstRadius,
                secondCenter,
                secondAxis,
                secondLength,
                secondRadius,
                ref best))
        {
            contact = default;
            return false;
        }
        if (!WideOrientedBox
            .TryKeepCenteredRigidCylinderCylinderClosestAxis(
                firstCenter,
                firstRotation,
                firstLocalAxis,
                firstLength,
                firstRadius,
                secondCenter,
                secondRotation,
                secondLocalAxis,
                secondLength,
                secondRadius,
                ref best))
        {
            contact = default;
            return false;
        }
        Vector3d normal;
        Fixed64 depth;
        bool depthIsClamped;
        if (best.IsWide)
        {
            normal = best.WideNormal;
            depth = best.WideDepth;
            depthIsClamped = best.WideDepthIsClamped;
        }
        else
        {
            Axis3 orientedAxis = best.Negate
                ? new Axis3(
                    WideArithmetic.Negate(best.Axis.X),
                    WideArithmetic.Negate(best.Axis.Y),
                    WideArithmetic.Negate(best.Axis.Z))
                : best.Axis;
            normal = WideGeometry.GetNormalized(
                Signed576.ExtendValue(orientedAxis.X),
                Signed576.ExtendValue(orientedAxis.Y),
                Signed576.ExtendValue(orientedAxis.Z));
            GetRoundedCylinderCylinderDepth(
                best.Depth,
                out depth,
                out depthIsClamped);
        }
        FixedPointAnchor firstAnchor =
            WideGeometry.GetCenteredCylinderSupportAnchor(
                firstCenter,
                firstRotation,
                firstLocalAxis,
                firstLength,
                firstRadius,
                normal);
        FixedPointAnchor secondAnchor =
            WideGeometry.GetCenteredCylinderSupportAnchor(
                secondCenter,
                secondRotation,
                secondLocalAxis,
                secondLength,
                secondRadius,
                -normal);
        contact = new FixedContactAnchors(
            firstAnchor,
            secondAnchor,
            normal,
            depth,
            depthIsClamped);
        usedWideCandidate = best.IsWide;
        usedMultiRadicalRanking =
            best.IsWide
            || !best.Depth.TryGetFastDepth(out _);
        return true;
    }

    internal static bool TryKeepWideCylinderCylinderAxis(
        WideCandidateAxis3 candidate,
        Vector3d firstCenter,
        FixedQuaternion firstRotation,
        Vector3d firstLocalAxis,
        Fixed64 firstLength,
        Fixed64 firstRadius,
        Vector3d secondCenter,
        FixedQuaternion secondRotation,
        Vector3d secondLocalAxis,
        Fixed64 secondLength,
        Fixed64 secondRadius,
        ref CylinderCylinderPenetration best)
    {
        WideOrientedBox.GetRotatedLocalAxisNumerators(
            firstRotation,
            firstLocalAxis,
            out Signed192 firstAxisX,
            out Signed192 firstAxisY,
            out Signed192 firstAxisZ,
            out Signed192 firstDenominator);
        WideOrientedBox.GetRotatedLocalAxisNumerators(
            secondRotation,
            secondLocalAxis,
            out Signed192 secondAxisX,
            out Signed192 secondAxisY,
            out Signed192 secondAxisZ,
            out Signed192 secondDenominator);
        var firstAxis = new RigidAxis3(
            firstAxisX,
            firstAxisY,
            firstAxisZ,
            firstDenominator);
        var secondAxis = new RigidAxis3(
            secondAxisX,
            secondAxisY,
            secondAxisZ,
            secondDenominator);
        if (!candidate.TryNarrow(out Axis3 axis))
        {
            return TryKeepWideCylinderCylinderAxisFallback(
                candidate,
                firstCenter,
                firstAxis,
                firstLength,
                firstRadius,
                secondCenter,
                secondAxis,
                secondLength,
                secondRadius,
                ref best);
        }
        return TryKeepCylinderCylinderAxis(
            axis,
            firstCenter,
            firstAxis,
            firstLength,
            firstRadius,
            secondCenter,
            secondAxis,
            secondLength,
            secondRadius,
            ref best);
    }

    private static bool TryKeepCylinderCylinderAxis(
        Axis3 axis,
        Vector3d firstCenter,
        RigidAxis3 firstAxis,
        Fixed64 firstLength,
        Fixed64 firstRadius,
        Vector3d secondCenter,
        RigidAxis3 secondAxis,
        Fixed64 secondLength,
        Fixed64 secondRadius,
        ref CylinderCylinderPenetration best)
    {
        if (axis.IsZero)
            return true;

        Signed576 firstAlignment =
            GetAxisProjection(axis, firstAxis);
        Signed576 secondAlignment =
            GetAxisProjection(axis, secondAxis);
        Signed576 centerProjection = GetDifferenceProjection(
            secondCenter,
            firstCenter,
            axis);
        Signed576 firstAxial = WideArithmetic.MultiplySigned576(
            WideArithmetic.MultiplySigned576(
                GetMagnitude576(firstAlignment),
                Signed192.Raw(firstLength)),
            secondAxis.RotationDenominator);
        Signed576 secondAxial = WideArithmetic.MultiplySigned576(
            WideArithmetic.MultiplySigned576(
                GetMagnitude576(secondAlignment),
                Signed192.Raw(secondLength)),
            firstAxis.RotationDenominator);
        Signed576 scaledCenter = WideArithmetic.MultiplySigned576(
            WideArithmetic.MultiplySigned576(
                WideArithmetic.MultiplySigned576(
                    GetMagnitude576(centerProjection),
                    firstAxis.RotationDenominator),
                secondAxis.RotationDenominator),
            Signed192.Raw(Fixed64.Two));
        Signed704 rational = Signed704.ExtendValue(
            WideArithmetic.SubtractSigned576(
                WideArithmetic.AddSigned576(
                    firstAxial,
                    secondAxial),
                scaledCenter));
        Signed576 axisSquared = GetAxisSquared(axis);
        Signed320 firstAxisSquared = GetAxisSquared(firstAxis);
        Signed320 secondAxisSquared = GetAxisSquared(secondAxis);
        Signed832 firstPlaneSquared = GetPlaneSquared(
            axisSquared,
            firstAxisSquared,
            firstAlignment);
        Signed832 secondPlaneSquared = GetPlaneSquared(
            axisSquared,
            secondAxisSquared,
            secondAlignment);
        Signed576 commonWide = WideArithmetic.MultiplySigned576(
            Signed576.ExtendValue(
                WideArithmetic.MultiplySigned192(
                    firstAxis.RotationDenominator,
                    secondAxis.RotationDenominator)),
            Signed192.Raw(Fixed64.Two));
        _ = Signed192.TryNarrowSigned(
            commonWide,
            out Signed192 common);
        var depth = new CylinderPairDepth(
            rational,
            common,
            firstRadius,
            secondRadius,
            axisSquared,
            firstAxisSquared,
            secondAxisSquared,
            firstPlaneSquared,
            secondPlaneSquared);
        if (!IsCylinderPairDepthNonNegative(depth))
            return false;
        if (!best.HasValue
            || CompareCylinderPairDepths(
                depth,
                best.Depth) < 0)
        {
            best = new CylinderCylinderPenetration(
                axis,
                centerProjection.Sign < 0,
                depth);
        }
        return true;
    }

    private static bool IsCylinderPairDepthNonNegative(
        in CylinderPairDepth depth)
    {
        if (depth.TryGetFastDepth(
                out ProjectionDepth fastDepth))
        {
            return IsProjectionNonNegative(fastDepth);
        }

        Span<ulong> radicands = stackalloc ulong[
            CylinderPairRadicandWords * 3];
        Span<int> signs = stackalloc int[3];
        radicands.Clear();
        signs.Clear();
        BuildCylinderPairLocalRadicands(
            depth,
            radicands,
            signs);
        return WideArithmetic.GetLinearRadicalSumSign(
            radicands,
            CylinderPairRadicandWords,
            signs) >= 0;
    }

    private static int CompareCylinderPairDepths(
        in CylinderPairDepth left,
        in CylinderPairDepth right)
    {
        if (left.TryGetFastDepth(
                out ProjectionDepth leftFast)
            && right.TryGetFastDepth(
                out ProjectionDepth rightFast))
        {
            return CompareProjectionDepths(
                leftFast,
                rightFast);
        }

        Span<ulong> radicands = stackalloc ulong[
            CylinderPairRadicandWords * 6];
        Span<int> signs = stackalloc int[6];
        radicands.Clear();
        signs.Clear();
        BuildCylinderPairCrossRadicands(
            left,
            right,
            radicands,
            signs);
        return WideArithmetic.GetLinearRadicalSumSign(
            radicands,
            CylinderPairRadicandWords,
            signs);
    }

    private static void BuildCylinderPairLocalRadicands(
        in CylinderPairDepth depth,
        Span<ulong> radicands,
        Span<int> signs)
    {
        Span<ulong> rational = radicands.Slice(
            0,
            CylinderPairRadicandWords);
        BuildCylinderPairRationalRadicand(
            depth.Rational,
            depth.FirstAxisSquared,
            depth.SecondAxisSquared,
            rational);
        signs[0] = depth.Rational.Sign;

        Span<ulong> firstDisk = radicands.Slice(
            CylinderPairRadicandWords,
            CylinderPairRadicandWords);
        Signed320 firstCoefficient =
            WideArithmetic.MultiplySigned192(
                depth.Common,
                Signed192.Raw(depth.FirstRadius));
        BuildCylinderPairDiskRadicand(
            firstCoefficient,
            depth.FirstPlaneSquared,
            depth.SecondAxisSquared,
            firstDisk);
        signs[1] = 1;

        Span<ulong> secondDisk = radicands.Slice(
            CylinderPairRadicandWords * 2,
            CylinderPairRadicandWords);
        Signed320 secondCoefficient =
            WideArithmetic.MultiplySigned192(
                depth.Common,
                Signed192.Raw(depth.SecondRadius));
        BuildCylinderPairDiskRadicand(
            secondCoefficient,
            depth.SecondPlaneSquared,
            depth.FirstAxisSquared,
            secondDisk);
        signs[2] = 1;
    }

    private static void BuildCylinderPairCrossRadicands(
        in CylinderPairDepth left,
        in CylinderPairDepth right,
        Span<ulong> radicands,
        Span<int> signs)
    {
        BuildCylinderPairCrossRationalRadicand(
            left,
            right,
            radicands.Slice(
                0,
                CylinderPairRadicandWords));
        signs[0] = left.Rational.Sign;
        BuildCylinderPairCrossDiskRadicand(
            left,
            firstDisk: true,
            right,
            radicands.Slice(
                CylinderPairRadicandWords,
                CylinderPairRadicandWords));
        signs[1] = 1;
        BuildCylinderPairCrossDiskRadicand(
            left,
            firstDisk: false,
            right,
            radicands.Slice(
                CylinderPairRadicandWords * 2,
                CylinderPairRadicandWords));
        signs[2] = 1;

        BuildCylinderPairCrossRationalRadicand(
            right,
            left,
            radicands.Slice(
                CylinderPairRadicandWords * 3,
                CylinderPairRadicandWords));
        signs[3] = -right.Rational.Sign;
        BuildCylinderPairCrossDiskRadicand(
            right,
            firstDisk: true,
            left,
            radicands.Slice(
                CylinderPairRadicandWords * 4,
                CylinderPairRadicandWords));
        signs[4] = -1;
        BuildCylinderPairCrossDiskRadicand(
            right,
            firstDisk: false,
            left,
            radicands.Slice(
                CylinderPairRadicandWords * 5,
                CylinderPairRadicandWords));
        signs[5] = -1;
    }

    private static void BuildCylinderPairRationalRadicand(
        Signed704 rational,
        Signed320 firstAxisSquared,
        Signed320 secondAxisSquared,
        Span<ulong> result)
    {
        Span<ulong> words = stackalloc ulong[11];
        WideArithmetic.GetMagnitude(rational, words);
        MultiplyMagnitudes(words, words, result);
        MultiplyCylinderPairBy(
            result,
            firstAxisSquared);
        MultiplyCylinderPairBy(
            result,
            secondAxisSquared);
    }

    private static void BuildCylinderPairDiskRadicand(
        Signed320 coefficient,
        Signed832 planeSquared,
        Signed320 otherAxisSquared,
        Span<ulong> result)
    {
        Span<ulong> coefficientWords =
            stackalloc ulong[5];
        GetMagnitude(coefficient, coefficientWords);
        MultiplyMagnitudes(
            coefficientWords,
            coefficientWords,
            result);
        MultiplyCylinderPairBy(
            result,
            planeSquared);
        MultiplyCylinderPairBy(
            result,
            otherAxisSquared);
    }

    private static void BuildCylinderPairCrossRationalRadicand(
        in CylinderPairDepth depth,
        in CylinderPairDepth other,
        Span<ulong> result)
    {
        Span<ulong> rationalWords =
            stackalloc ulong[11];
        WideArithmetic.GetMagnitude(
            depth.Rational,
            rationalWords);
        MultiplyMagnitudes(
            rationalWords,
            rationalWords,
            result);
        MultiplyCylinderPairBy(
            result,
            other.AxisSquared);
        MultiplyCylinderPairBy(
            result,
            depth.FirstAxisSquared);
        MultiplyCylinderPairBy(
            result,
            depth.SecondAxisSquared);
        MultiplyCylinderPairBy(
            result,
            other.FirstAxisSquared);
        MultiplyCylinderPairBy(
            result,
            other.SecondAxisSquared);
    }

    private static void BuildCylinderPairCrossDiskRadicand(
        in CylinderPairDepth depth,
        bool firstDisk,
        in CylinderPairDepth other,
        Span<ulong> result)
    {
        Fixed64 radius = firstDisk
            ? depth.FirstRadius
            : depth.SecondRadius;
        Signed832 planeSquared = firstDisk
            ? depth.FirstPlaneSquared
            : depth.SecondPlaneSquared;
        Signed320 uncancelledAxisSquared = firstDisk
            ? depth.SecondAxisSquared
            : depth.FirstAxisSquared;
        Signed320 coefficient =
            WideArithmetic.MultiplySigned192(
                depth.Common,
                Signed192.Raw(radius));
        Span<ulong> coefficientWords =
            stackalloc ulong[5];
        GetMagnitude(
            coefficient,
            coefficientWords);
        MultiplyMagnitudes(
            coefficientWords,
            coefficientWords,
            result);
        MultiplyCylinderPairBy(
            result,
            planeSquared);
        MultiplyCylinderPairBy(
            result,
            other.AxisSquared);
        MultiplyCylinderPairBy(
            result,
            uncancelledAxisSquared);
        MultiplyCylinderPairBy(
            result,
            other.FirstAxisSquared);
        MultiplyCylinderPairBy(
            result,
            other.SecondAxisSquared);
    }

    private static void MultiplyCylinderPairBy(
        Span<ulong> value,
        Signed320 factor)
    {
        Span<ulong> words = stackalloc ulong[5];
        GetMagnitude(factor, words);
        MultiplyCylinderPairBy(value, words);
    }

    private static void MultiplyCylinderPairBy(
        Span<ulong> value,
        Signed576 factor)
    {
        Span<ulong> words = stackalloc ulong[9];
        WideArithmetic.GetMagnitude(factor, words);
        MultiplyCylinderPairBy(value, words);
    }

    private static void MultiplyCylinderPairBy(
        Span<ulong> value,
        Signed832 factor)
    {
        Span<ulong> words = stackalloc ulong[13];
        WideArithmetic.GetMagnitude(factor, words);
        MultiplyCylinderPairBy(value, words);
    }

    private static void MultiplyCylinderPairBy(
        Span<ulong> value,
        ReadOnlySpan<ulong> factor)
    {
        Span<ulong> product =
            stackalloc ulong[CylinderPairRadicandWords];
        MultiplyMagnitudes(
            value,
            factor,
            product);
        product.CopyTo(value);
    }
}
