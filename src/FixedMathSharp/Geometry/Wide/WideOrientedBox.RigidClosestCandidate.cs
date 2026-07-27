//=======================================================================
// WideOrientedBox.RigidClosestCandidate.cs
//=======================================================================
// MIT License, Copyright (c) 2024-present David Oravsky (mrdav30)
// See LICENSE file in the project root for full license information.
//=======================================================================

using System;

namespace FixedMathSharp.Bounds;

/// <content>
/// Provides rigid closest-axis/closest-point candidate resolution for cylinder-capsule
/// and cylinder-cylinder pairs, used to determine penetration depth and axis data
/// for collision response.
/// </content>
internal static partial class WideOrientedBox
{
    internal static bool TryKeepCenteredRigidCylinderCapsuleClosestAxis(
        Vector3d cylinderCenter,
        FixedQuaternion cylinderRotation,
        Vector3d cylinderLocalAxis,
        Fixed64 cylinderLength,
        Fixed64 cylinderRadius,
        Vector3d capsuleCenter,
        FixedQuaternion capsuleRotation,
        Vector3d capsuleLocalAxis,
        Fixed64 capsuleLength,
        Fixed64 capsuleRadius,
        ref WideConvexPrismRelations.CylinderCapsulePenetration best)
    {
        var unused = default(
            WideConvexPrismRelations.CylinderCylinderPenetration);
        return TryKeepCenteredRigidCylinderClosestAxis(
            cylinderCenter,
            cylinderRotation,
            cylinderLocalAxis,
            cylinderLength,
            cylinderRadius,
            capsuleCenter,
            capsuleRotation,
            capsuleLocalAxis,
            capsuleLength,
            capsuleRadius,
            RigidClosestPairKind.CylinderCapsule,
            ref best,
            ref unused);
    }

    internal static bool TryKeepCenteredRigidCylinderCylinderClosestAxis(
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
        ref WideConvexPrismRelations.CylinderCylinderPenetration best)
    {
        var unused = default(
            WideConvexPrismRelations.CylinderCapsulePenetration);
        return TryKeepCenteredRigidCylinderClosestAxis(
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
            RigidClosestPairKind.CylinderCylinder,
            ref unused,
            ref best);
    }

    private static bool TryKeepCenteredRigidCylinderClosestAxis(
        Vector3d cylinderCenter,
        FixedQuaternion cylinderRotation,
        Vector3d cylinderLocalAxis,
        Fixed64 cylinderLength,
        Fixed64 cylinderRadius,
        Vector3d capsuleCenter,
        FixedQuaternion capsuleRotation,
        Vector3d capsuleLocalAxis,
        Fixed64 capsuleLength,
        Fixed64 capsuleRadius,
        RigidClosestPairKind pairKind,
        ref WideConvexPrismRelations.CylinderCapsulePenetration
            cylinderCapsuleBest,
        ref WideConvexPrismRelations.CylinderCylinderPenetration
            cylinderCylinderBest)
    {
        GetRotatedLocalAxisNumerators(
            cylinderRotation,
            cylinderLocalAxis,
            out Signed192 firstAxisX,
            out Signed192 firstAxisY,
            out Signed192 firstAxisZ,
            out Signed192 firstRotationDenominator);
        GetRotatedLocalAxisNumerators(
            capsuleRotation,
            capsuleLocalAxis,
            out Signed192 secondAxisX,
            out Signed192 secondAxisY,
            out Signed192 secondAxisZ,
            out Signed192 secondRotationDenominator);
        var firstAxis = new RigidSegmentAxis(
            firstAxisX,
            firstAxisY,
            firstAxisZ,
            firstRotationDenominator,
            cylinderLength);
        var secondAxis = new RigidSegmentAxis(
            secondAxisX,
            secondAxisY,
            secondAxisZ,
            secondRotationDenominator,
            capsuleLength);
        Signed576 relationDenominator = WideArithmetic.AddSigned576(
            WideArithmetic.MultiplySigned320(
                firstAxis.Denominator,
                secondAxis.Denominator),
            WideArithmetic.MultiplySigned320(
                firstAxis.Denominator,
                secondAxis.Denominator));
        GetRigidSegmentLowerDifference(
            cylinderCenter,
            firstAxis,
            capsuleCenter,
            secondAxis,
            relationDenominator,
            out Signed576 relationX,
            out Signed576 relationY,
            out Signed576 relationZ);
        Signed576 firstSquared = Dot(firstAxis, firstAxis);
        Signed576 directionsDot = Dot(firstAxis, secondAxis);
        Signed576 secondSquared = Dot(secondAxis, secondAxis);
        Signed704 firstDotDifference = Dot(
            firstAxis,
            relationX,
            relationY,
            relationZ);
        Signed704 secondDotDifference = Dot(
            secondAxis,
            relationX,
            relationY,
            relationZ);

        Span<ulong> determinant =
            stackalloc ulong[TriangleSweepMagnitudeWords];
        GetProductDifference(
            firstSquared,
            secondSquared,
            directionsDot,
            directionsDot,
            determinant,
            out int determinantSign);
        Span<ulong> firstCore =
            stackalloc ulong[TriangleSweepMagnitudeWords];
        Span<ulong> secondCore =
            stackalloc ulong[TriangleSweepMagnitudeWords];
        int firstCoreSign = 0;
        int secondCoreSign = 0;
        Span<ulong> firstNumerator =
            stackalloc ulong[TriangleSweepMagnitudeWords];
        Span<ulong> firstDenominator =
            stackalloc ulong[TriangleSweepMagnitudeWords];
        Span<ulong> secondNumerator =
            stackalloc ulong[TriangleSweepMagnitudeWords];
        Span<ulong> secondDenominator =
            stackalloc ulong[TriangleSweepMagnitudeWords];
        int firstNumeratorSign;
        int secondNumeratorSign;
        int firstFeature;
        int secondFeature;
        if (determinantSign > 0)
        {
            GetProductDifference(
                directionsDot,
                secondDotDifference,
                secondSquared,
                firstDotDifference,
                firstCore,
                out firstCoreSign);
            SetRigidSegmentRatio(
                firstCore,
                firstAxis.Denominator,
                determinant,
                relationDenominator,
                firstNumerator,
                firstDenominator);
            firstNumeratorSign = firstCoreSign;
            firstFeature = ClampUnitRatio(
                firstNumerator,
                firstDenominator,
                firstNumeratorSign);

            if (firstFeature == RigidSegmentLower)
            {
                SetRigidSegmentProjectionRatio(
                    secondDotDifference,
                    secondAxis.Denominator,
                    secondSquared,
                    relationDenominator,
                    secondNumerator,
                    secondDenominator);
                secondNumeratorSign = secondDotDifference.Sign;
            }
            else if (firstFeature == RigidSegmentUpper)
            {
                Signed704 upperProjection = WideArithmetic.AddSigned704(
                    secondDotDifference,
                    DoubleProduct(
                        directionsDot,
                        secondAxis.Denominator));
                SetRigidSegmentProjectionRatio(
                    upperProjection,
                    secondAxis.Denominator,
                    secondSquared,
                    relationDenominator,
                    secondNumerator,
                    secondDenominator);
                secondNumeratorSign = upperProjection.Sign;
            }
            else
            {
                GetProductDifference(
                    firstSquared,
                    secondDotDifference,
                    directionsDot,
                    firstDotDifference,
                    secondCore,
                    out secondCoreSign);
                SetRigidSegmentRatio(
                    secondCore,
                    secondAxis.Denominator,
                    determinant,
                    relationDenominator,
                    secondNumerator,
                    secondDenominator);
                secondNumeratorSign = secondCoreSign;
            }
        }
        else
        {
            firstNumerator.Clear();
            firstDenominator.Clear();
            firstDenominator[0] = 1UL;
            firstNumeratorSign = 0;
            firstFeature = RigidSegmentLower;
            SetRigidSegmentProjectionRatio(
                secondDotDifference,
                secondAxis.Denominator,
                secondSquared,
                relationDenominator,
                secondNumerator,
                secondDenominator);
            secondNumeratorSign = secondDotDifference.Sign;
        }

        secondFeature = ClampUnitRatio(
            secondNumerator,
            secondDenominator,
            secondNumeratorSign);
        if (secondFeature == RigidSegmentLower)
        {
            SetRigidSegmentProjectionRatio(
                WideArithmetic.SubtractSigned704(
                    default,
                    firstDotDifference),
                firstAxis.Denominator,
                firstSquared,
                relationDenominator,
                firstNumerator,
                firstDenominator);
            firstNumeratorSign = -firstDotDifference.Sign;
            firstFeature = ClampUnitRatio(
                firstNumerator,
                firstDenominator,
                firstNumeratorSign);
        }
        else if (secondFeature == RigidSegmentUpper)
        {
            Signed704 upperProjection = WideArithmetic.SubtractSigned704(
                DoubleProduct(
                    directionsDot,
                    firstAxis.Denominator),
                firstDotDifference);
            SetRigidSegmentProjectionRatio(
                upperProjection,
                firstAxis.Denominator,
                firstSquared,
                relationDenominator,
                firstNumerator,
                firstDenominator);
            firstNumeratorSign = upperProjection.Sign;
            firstFeature = ClampUnitRatio(
                firstNumerator,
                firstDenominator,
                firstNumeratorSign);
        }

        if (firstFeature == RigidSegmentUpper)
        {
            AddRigidSegmentDisplacement(
                ref relationX,
                firstAxis.X,
                secondAxis.Denominator);
            AddRigidSegmentDisplacement(
                ref relationY,
                firstAxis.Y,
                secondAxis.Denominator);
            AddRigidSegmentDisplacement(
                ref relationZ,
                firstAxis.Z,
                secondAxis.Denominator);
        }
        if (secondFeature == RigidSegmentUpper)
        {
            SubtractRigidSegmentDisplacement(
                ref relationX,
                secondAxis.X,
                firstAxis.Denominator);
            SubtractRigidSegmentDisplacement(
                ref relationY,
                secondAxis.Y,
                firstAxis.Denominator);
            SubtractRigidSegmentDisplacement(
                ref relationZ,
                secondAxis.Z,
                firstAxis.Denominator);
        }

        Span<ulong> candidateX =
            stackalloc ulong[TriangleSweepMagnitudeWords];
        Span<ulong> candidateY =
            stackalloc ulong[TriangleSweepMagnitudeWords];
        Span<ulong> candidateZ =
            stackalloc ulong[TriangleSweepMagnitudeWords];
        int candidateXSign;
        int candidateYSign;
        int candidateZSign;
        if (firstFeature == RigidSegmentInterior
            && secondFeature == RigidSegmentInterior)
        {
            BuildRigidLinePairClosestComponent(
                relationX,
                determinant,
                firstAxis.X,
                firstCore,
                firstCoreSign,
                secondAxis.X,
                secondCore,
                secondCoreSign,
                candidateX,
                out candidateXSign);
            BuildRigidLinePairClosestComponent(
                relationY,
                determinant,
                firstAxis.Y,
                firstCore,
                firstCoreSign,
                secondAxis.Y,
                secondCore,
                secondCoreSign,
                candidateY,
                out candidateYSign);
            BuildRigidLinePairClosestComponent(
                relationZ,
                determinant,
                firstAxis.Z,
                firstCore,
                firstCoreSign,
                secondAxis.Z,
                secondCore,
                secondCoreSign,
                candidateZ,
                out candidateZSign);
        }
        else if (firstFeature == RigidSegmentInterior)
        {
            Signed704 adjustedDot = Dot(
                firstAxis,
                relationX,
                relationY,
                relationZ);
            BuildRigidPointLineClosestComponent(
                relationX,
                firstSquared,
                firstAxis.X,
                adjustedDot,
                candidateX,
                out candidateXSign);
            BuildRigidPointLineClosestComponent(
                relationY,
                firstSquared,
                firstAxis.Y,
                adjustedDot,
                candidateY,
                out candidateYSign);
            BuildRigidPointLineClosestComponent(
                relationZ,
                firstSquared,
                firstAxis.Z,
                adjustedDot,
                candidateZ,
                out candidateZSign);
        }
        else if (secondFeature == RigidSegmentInterior)
        {
            Signed704 adjustedDot = Dot(
                secondAxis,
                relationX,
                relationY,
                relationZ);
            BuildRigidPointLineClosestComponent(
                relationX,
                secondSquared,
                secondAxis.X,
                adjustedDot,
                candidateX,
                out candidateXSign);
            BuildRigidPointLineClosestComponent(
                relationY,
                secondSquared,
                secondAxis.Y,
                adjustedDot,
                candidateY,
                out candidateYSign);
            BuildRigidPointLineClosestComponent(
                relationZ,
                secondSquared,
                secondAxis.Z,
                adjustedDot,
                candidateZ,
                out candidateZSign);
        }
        else
        {
            GetSignedMagnitude(
                relationX,
                candidateX,
                out candidateXSign);
            GetSignedMagnitude(
                relationY,
                candidateY,
                out candidateYSign);
            GetSignedMagnitude(
                relationZ,
                candidateZ,
                out candidateZSign);
        }

        RemoveCommonPowerOfTwo(
            candidateX,
            candidateY,
            candidateZ);
        var candidate = new WideConvexPrismRelations.WideCandidateAxis3(
            candidateX,
            candidateXSign,
            candidateY,
            candidateYSign,
            candidateZ,
            candidateZSign);
        if (pairKind == RigidClosestPairKind.CylinderCapsule)
        {
            return WideConvexPrismRelations
                .TryKeepWideCylinderCapsuleAxis(
                    candidate,
                    cylinderCenter,
                    cylinderRotation,
                    cylinderLocalAxis,
                    cylinderLength,
                    cylinderRadius,
                    capsuleCenter,
                    capsuleRotation,
                    capsuleLocalAxis,
                    capsuleLength,
                    capsuleRadius,
                    ref cylinderCapsuleBest);
        }
        return WideConvexPrismRelations
            .TryKeepWideCylinderCylinderAxis(
                candidate,
                cylinderCenter,
                cylinderRotation,
                cylinderLocalAxis,
                cylinderLength,
                cylinderRadius,
                capsuleCenter,
                capsuleRotation,
                capsuleLocalAxis,
                capsuleLength,
                capsuleRadius,
                ref cylinderCylinderBest);
    }

    private enum RigidClosestPairKind
    {
        CylinderCapsule,
        CylinderCylinder,
    }

    private static void BuildRigidPointLineClosestComponent(
        Signed576 relation,
        Signed576 axisSquared,
        Signed320 axisComponent,
        Signed704 axisDot,
        Span<ulong> result,
        out int resultSign)
    {
        Span<ulong> relationMagnitude = stackalloc ulong[9];
        Span<ulong> axisSquaredMagnitude = stackalloc ulong[9];
        Span<ulong> axisMagnitude = stackalloc ulong[5];
        Span<ulong> dotMagnitude = stackalloc ulong[11];
        WideArithmetic.GetMagnitude(
            relation,
            relationMagnitude);
        WideArithmetic.GetMagnitude(
            axisSquared,
            axisSquaredMagnitude);
        WideArithmetic.GetMagnitude(
            axisComponent,
            out axisMagnitude[4],
            out axisMagnitude[3],
            out axisMagnitude[2],
            out axisMagnitude[1],
            out axisMagnitude[0]);
        WideArithmetic.GetMagnitude(axisDot, dotMagnitude);
        Span<ulong> relationTerm =
            stackalloc ulong[TriangleSweepMagnitudeWords];
        Span<ulong> axisTerm =
            stackalloc ulong[TriangleSweepMagnitudeWords];
        MultiplyMagnitudes(
            relationMagnitude,
            axisSquaredMagnitude,
            relationTerm);
        MultiplyMagnitudes(
            axisMagnitude,
            dotMagnitude,
            axisTerm);
        CombineSignedMagnitudes(
            relationTerm,
            relation.Sign,
            axisTerm,
            -(axisComponent.Sign * axisDot.Sign),
            result,
            out resultSign);
    }

    private static void BuildRigidLinePairClosestComponent(
        Signed576 relation,
        ReadOnlySpan<ulong> determinant,
        Signed320 firstAxisComponent,
        ReadOnlySpan<ulong> firstCore,
        int firstCoreSign,
        Signed320 secondAxisComponent,
        ReadOnlySpan<ulong> secondCore,
        int secondCoreSign,
        Span<ulong> result,
        out int resultSign)
    {
        Span<ulong> relationMagnitude = stackalloc ulong[9];
        Span<ulong> firstAxisMagnitude = stackalloc ulong[5];
        Span<ulong> secondAxisMagnitude = stackalloc ulong[5];
        WideArithmetic.GetMagnitude(
            relation,
            relationMagnitude);
        WideArithmetic.GetMagnitude(
            firstAxisComponent,
            out firstAxisMagnitude[4],
            out firstAxisMagnitude[3],
            out firstAxisMagnitude[2],
            out firstAxisMagnitude[1],
            out firstAxisMagnitude[0]);
        WideArithmetic.GetMagnitude(
            secondAxisComponent,
            out secondAxisMagnitude[4],
            out secondAxisMagnitude[3],
            out secondAxisMagnitude[2],
            out secondAxisMagnitude[1],
            out secondAxisMagnitude[0]);
        Span<ulong> relationTerm =
            stackalloc ulong[TriangleSweepMagnitudeWords];
        Span<ulong> firstTerm =
            stackalloc ulong[TriangleSweepMagnitudeWords];
        Span<ulong> secondTerm =
            stackalloc ulong[TriangleSweepMagnitudeWords];
        Span<ulong> firstCombined =
            stackalloc ulong[TriangleSweepMagnitudeWords];
        MultiplyMagnitudes(
            relationMagnitude,
            determinant,
            relationTerm);
        MultiplyMagnitudes(
            firstAxisMagnitude,
            firstCore,
            firstTerm);
        MultiplyMagnitudes(
            secondAxisMagnitude,
            secondCore,
            secondTerm);
        CombineSignedMagnitudes(
            relationTerm,
            relation.Sign,
            firstTerm,
            firstAxisComponent.Sign * firstCoreSign,
            firstCombined,
            out int firstCombinedSign);
        CombineSignedMagnitudes(
            firstCombined,
            firstCombinedSign,
            secondTerm,
            -(secondAxisComponent.Sign * secondCoreSign),
            result,
            out resultSign);
    }

    private static void GetSignedMagnitude(
        Signed576 value,
        Span<ulong> magnitude,
        out int sign)
    {
        magnitude.Clear();
        WideArithmetic.GetMagnitude(value, magnitude);
        sign = value.Sign;
    }

    private static void RemoveCommonPowerOfTwo(
        Span<ulong> x,
        Span<ulong> y,
        Span<ulong> z)
    {
        int shift = Math.Min(
            GetTrailingZeroCount(x),
            Math.Min(
                GetTrailingZeroCount(y),
                GetTrailingZeroCount(z)));
        // Rigid-frame terms retain at least one shared binary scale factor.
        // A zero vector is the only candidate that cannot be reduced.
        if (shift == int.MaxValue)
            return;
        ShiftRightMagnitude(x, shift);
        ShiftRightMagnitude(y, shift);
        ShiftRightMagnitude(z, shift);
    }

    private static int GetTrailingZeroCount(
        ReadOnlySpan<ulong> value)
    {
        for (int index = 0; index < value.Length; index++)
        {
            ulong word = value[index];
            if (word == 0UL)
                continue;
            int count = index << 6;
            while ((word & 1UL) == 0UL)
            {
                count++;
                word >>= 1;
            }
            return count;
        }
        return int.MaxValue;
    }

    private static void ShiftRightMagnitude(
        Span<ulong> value,
        int shift)
    {
        int wordShift = shift >> 6;
        int bitShift = shift & 63;
        for (int index = 0; index < value.Length; index++)
        {
            int source = index + wordShift;
            ulong result = source < value.Length
                ? value[source] >> bitShift
                : 0UL;
            if (bitShift != 0
                && source + 1 < value.Length)
            {
                result |= value[source + 1] << (64 - bitShift);
            }
            value[index] = result;
        }
    }
}
