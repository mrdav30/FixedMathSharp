//=======================================================================
// WideOrientedBox.CoulombResponse.cs
//=======================================================================
// MIT License, Copyright (c) 2026–present David Oravsky (mrdav30)
// See LICENSE file in the project root for full license information.
//=======================================================================

using System;
using System.Runtime.CompilerServices;

namespace FixedMathSharp.Geometry;

internal static partial class WideOrientedBox
{
    private const int MaxCoulombWords = MaxResponseWords * 2;
    private const int MaxCoulombSquareWords = MaxCoulombWords * 2;
    // Normal-response ratios use fewer than 46 active words. A common tangent
    // denominator therefore uses fewer than 92, its squared magnitude fewer
    // than 185 including carry, and the largest radial comparison fewer than
    // 307. The remaining words prevent silent product truncation.
    private const int MaxCoulombComparisonWords = 320;

    [MethodImpl(MethodImplOptions.NoInlining)]
    internal static bool TryGetCoulombLineResponse(
        in FixedLeverNormalConstraint3d normalConstraint,
        in FixedLeverResponseOperand3d firstTangent,
        in FixedLeverResponseOperand3d secondTangent,
        Vector3d tangent,
        Fixed64 accumulatedTangentImpulse,
        Fixed64 staticFriction,
        Fixed64 dynamicFriction,
        out FixedLeverCoulombResponse3d response)
    {
        response = default;
        bool inputsValid =
            staticFriction >= Fixed64.Zero
            & dynamicFriction >= Fixed64.Zero
            & tangent.IsNormalized();
        if (!inputsValid)
            return false;

        bool constraintValid =
            FixedMath.Abs(Vector3d.Dot(
                normalConstraint.Normal,
                tangent)) <= Fixed64.Epsilon
            & HaveMatchingParticipants(
                normalConstraint.First,
                firstTangent)
            & HaveMatchingParticipants(
                normalConstraint.Second,
                secondTangent);
        if (!constraintValid)
            return false;

        Span<ulong> normalNumerator = stackalloc ulong[MaxResponseWords];
        Span<ulong> normalDenominator = stackalloc ulong[MaxResponseWords];
        Span<ulong> tangentNumerator = stackalloc ulong[MaxResponseWords];
        Span<ulong> tangentDenominator = stackalloc ulong[MaxResponseWords];
        if (!TryGetCompletedNormalAccumulatorRatio(
                normalConstraint,
                normalNumerator,
                normalDenominator))
            return false;
        GetBilateralImpulseRatio(
            firstTangent,
            secondTangent,
            tangent,
            tangentNumerator,
            tangentDenominator,
            out int tangentSign);

        Span<ulong> desiredNumerator = stackalloc ulong[MaxResponseWords];
        AddFixedToRatio(
            tangentNumerator,
            tangentDenominator,
            tangentSign,
            accumulatedTangentImpulse,
            desiredNumerator,
            out int desiredSign);

        Span<ulong> staticNumerator = stackalloc ulong[MaxResponseWords];
        Span<ulong> staticDenominator = stackalloc ulong[MaxResponseWords];
        GetFrictionLimit(
            normalNumerator,
            normalDenominator,
            staticFriction,
            staticNumerator,
            staticDenominator);

        Span<ulong> appliedNumerator = stackalloc ulong[MaxCoulombWords];
        Span<ulong> appliedDenominator = stackalloc ulong[MaxCoulombWords];
        int appliedSign;
        Span<ulong> accumulatedNumerator = stackalloc ulong[MaxCoulombWords];
        Span<ulong> accumulatedDenominator = stackalloc ulong[MaxCoulombWords];
        int accumulatedSign;
        appliedNumerator.Clear();
        appliedDenominator.Clear();
        accumulatedNumerator.Clear();
        accumulatedDenominator.Clear();
        Span<ulong> clampedNumerator = stackalloc ulong[MaxCoulombWords];
        Span<ulong> clampedDenominator = stackalloc ulong[MaxResponseWords];
        if (CompareRatios(
                desiredNumerator,
                tangentDenominator,
                staticNumerator,
                staticDenominator) <= 0)
        {
            tangentNumerator.CopyTo(appliedNumerator);
            tangentDenominator.CopyTo(appliedDenominator);
            appliedSign = tangentSign;
            desiredNumerator.CopyTo(accumulatedNumerator);
            tangentDenominator.CopyTo(accumulatedDenominator);
            accumulatedSign = desiredSign;
        }
        else
        {
            GetFrictionLimit(
                normalNumerator,
                normalDenominator,
                dynamicFriction,
                clampedNumerator,
                clampedDenominator);
            clampedNumerator.CopyTo(accumulatedNumerator);
            clampedDenominator.CopyTo(accumulatedDenominator);
            accumulatedSign = WideArithmetic.IsZeroMagnitude(
                clampedNumerator)
                ? 0
                : desiredSign;
            SubtractFixedFromRatio(
                clampedNumerator,
                clampedDenominator,
                accumulatedSign,
                accumulatedTangentImpulse,
                appliedNumerator,
                appliedDenominator,
                out appliedSign);
        }

        bool resolved = TryGetLinearVelocityDelta(
            firstTangent.LinearImpulseAxis,
            firstTangent.InverseMass,
            appliedNumerator,
            appliedDenominator,
            appliedSign,
            out Vector3d firstLinear);
        resolved &= TryGetAngularVelocityDelta(
            firstTangent.Lever,
            -tangent,
            firstTangent.InverseInertia,
            appliedNumerator,
            appliedDenominator,
            appliedSign,
            out Vector3d firstAngular);
        resolved &= TryGetLinearVelocityDelta(
            secondTangent.LinearImpulseAxis,
            secondTangent.InverseMass,
            appliedNumerator,
            appliedDenominator,
            appliedSign,
            out Vector3d secondLinear);
        resolved &= TryGetAngularVelocityDelta(
            secondTangent.Lever,
            tangent,
            secondTangent.InverseInertia,
            appliedNumerator,
            appliedDenominator,
            appliedSign,
            out Vector3d secondAngular);
        if (!resolved)
            return false;

        bool hasAccumulatedProjection = Fixed64.TryGetSignedRawRatio(
            accumulatedNumerator,
            accumulatedDenominator,
            accumulatedSign < 0,
            out Fixed64 accumulatedProjection);
        response = new FixedLeverCoulombResponse3d(
            !WideArithmetic.IsZeroMagnitude(appliedNumerator),
            firstLinear,
            firstAngular,
            secondLinear,
            secondAngular,
            hasAccumulatedProjection,
            accumulatedProjection,
            hasSecondaryAccumulatedImpulse: false,
            secondaryAccumulatedImpulse: default);
        return true;
    }

    [MethodImpl(MethodImplOptions.NoInlining)]
    internal static bool TryGetCoulombDiskResponse(
        in FixedLeverNormalConstraint3d normalConstraint,
        in FixedLeverResponseOperand3d primaryFirst,
        in FixedLeverResponseOperand3d primarySecond,
        Vector3d primaryTangent,
        in FixedLeverResponseOperand3d secondaryFirst,
        in FixedLeverResponseOperand3d secondarySecond,
        Vector3d secondaryTangent,
        Fixed64 staticFriction,
        Fixed64 dynamicFriction,
        out FixedLeverCoulombResponse3d response)
    {
        response = default;
        bool inputsValid =
            staticFriction >= Fixed64.Zero
            & dynamicFriction >= Fixed64.Zero
            & primaryTangent.IsNormalized()
            & secondaryTangent.IsNormalized()
            & FixedMath.Abs(Vector3d.Dot(
                primaryTangent,
                secondaryTangent)) <= Fixed64.Epsilon
            & FixedMath.Abs(Vector3d.Dot(
                normalConstraint.Normal,
                primaryTangent)) <= Fixed64.Epsilon
            & FixedMath.Abs(Vector3d.Dot(
                normalConstraint.Normal,
                secondaryTangent)) <= Fixed64.Epsilon
            & HaveMatchingParticipants(
                normalConstraint.First,
                primaryFirst)
            & HaveMatchingParticipants(
                normalConstraint.Second,
                primarySecond)
            & HaveMatchingParticipants(primaryFirst, secondaryFirst)
            & HaveMatchingParticipants(primarySecond, secondarySecond);
        if (!inputsValid)
            return false;

        Span<ulong> normalNumerator = stackalloc ulong[MaxResponseWords];
        Span<ulong> normalDenominator = stackalloc ulong[MaxResponseWords];
        Span<ulong> primaryNumerator = stackalloc ulong[MaxResponseWords];
        Span<ulong> primaryDenominator = stackalloc ulong[MaxResponseWords];
        Span<ulong> secondaryNumerator = stackalloc ulong[MaxResponseWords];
        Span<ulong> secondaryDenominator = stackalloc ulong[MaxResponseWords];
        if (!TryGetCompletedNormalAccumulatorRatio(
                normalConstraint,
                normalNumerator,
                normalDenominator))
            return false;
        GetBilateralImpulseRatio(
            primaryFirst,
            primarySecond,
            primaryTangent,
            primaryNumerator,
            primaryDenominator,
            out int primarySign);
        GetBilateralImpulseRatio(
            secondaryFirst,
            secondarySecond,
            secondaryTangent,
            secondaryNumerator,
            secondaryDenominator,
            out int secondarySign);

        Span<ulong> commonDenominator = stackalloc ulong[MaxCoulombWords];
        Span<ulong> primaryAtCommon = stackalloc ulong[MaxCoulombWords];
        Span<ulong> secondaryAtCommon = stackalloc ulong[MaxCoulombWords];
        WideArithmetic.MultiplyMagnitudes(
            primaryDenominator,
            secondaryDenominator,
            commonDenominator);
        WideArithmetic.MultiplyMagnitudes(
            primaryNumerator,
            secondaryDenominator,
            primaryAtCommon);
        WideArithmetic.MultiplyMagnitudes(
            secondaryNumerator,
            primaryDenominator,
            secondaryAtCommon);

        Span<ulong> magnitudeSquared = stackalloc ulong[MaxCoulombSquareWords];
        Span<ulong> square = stackalloc ulong[MaxCoulombSquareWords];
        WideArithmetic.MultiplyMagnitudes(
            primaryAtCommon,
            primaryAtCommon,
            magnitudeSquared);
        WideArithmetic.MultiplyMagnitudes(
            secondaryAtCommon,
            secondaryAtCommon,
            square);
        WideArithmetic.AddMagnitudeInto(square, magnitudeSquared);

        Span<ulong> staticNumerator = stackalloc ulong[MaxResponseWords];
        Span<ulong> staticDenominator = stackalloc ulong[MaxResponseWords];
        GetFrictionLimit(
            normalNumerator,
            normalDenominator,
            staticFriction,
            staticNumerator,
            staticDenominator);
        bool withinStaticLimit = IsVectorWithinLimit(
            magnitudeSquared,
            commonDenominator,
            staticNumerator,
            staticDenominator);

        Span<ulong> dynamicNumerator = stackalloc ulong[MaxResponseWords];
        Span<ulong> dynamicDenominator = stackalloc ulong[MaxResponseWords];
        if (!withinStaticLimit)
        {
            GetFrictionLimit(
                normalNumerator,
                normalDenominator,
                dynamicFriction,
                dynamicNumerator,
                dynamicDenominator);
        }

        bool hasImpulse =
            !WideArithmetic.IsZeroMagnitude(magnitudeSquared)
            && (withinStaticLimit
                || !WideArithmetic.IsZeroMagnitude(dynamicNumerator));
        if (!hasImpulse)
        {
            response = new FixedLeverCoulombResponse3d(
                hasAppliedImpulse: false,
                Vector3d.Zero,
                Vector3d.Zero,
                Vector3d.Zero,
                Vector3d.Zero,
                hasPrimaryAccumulatedImpulse: true,
                Fixed64.Zero,
                hasSecondaryAccumulatedImpulse: true,
                Fixed64.Zero);
            return true;
        }

        bool resolved = TryGetDiskVelocityDeltas(
            primaryFirst,
            primarySecond,
            secondaryFirst,
            secondarySecond,
            primaryTangent,
            secondaryTangent,
            primaryAtCommon,
            primarySign,
            secondaryAtCommon,
            secondarySign,
            commonDenominator,
            withinStaticLimit,
            dynamicNumerator,
            dynamicDenominator,
            magnitudeSquared,
            out Vector3d firstLinear,
            out Vector3d firstAngular,
            out Vector3d secondLinear,
            out Vector3d secondAngular);
        if (!resolved)
            return false;

        bool hasPrimaryProjection;
        bool hasSecondaryProjection;
        Fixed64 primaryProjection;
        Fixed64 secondaryProjection;
        if (withinStaticLimit)
        {
            hasPrimaryProjection = Fixed64.TryGetSignedRawRatio(
                primaryAtCommon,
                commonDenominator,
                primarySign < 0,
                out primaryProjection);
            hasSecondaryProjection = Fixed64.TryGetSignedRawRatio(
                secondaryAtCommon,
                commonDenominator,
                secondarySign < 0,
                out secondaryProjection);
        }
        else
        {
            Span<ulong> projectedNumerator =
                stackalloc ulong[MaxCoulombWords + MaxResponseWords];
            WideArithmetic.MultiplyMagnitudes(
                primaryAtCommon,
                dynamicNumerator,
                projectedNumerator);
            hasPrimaryProjection = TryGetSignedRatioOverSquareRoot(
                projectedNumerator,
                dynamicDenominator,
                magnitudeSquared,
                primarySign < 0,
                out primaryProjection);
            WideArithmetic.MultiplyMagnitudes(
                secondaryAtCommon,
                dynamicNumerator,
                projectedNumerator);
            hasSecondaryProjection = TryGetSignedRatioOverSquareRoot(
                projectedNumerator,
                dynamicDenominator,
                magnitudeSquared,
                secondarySign < 0,
                out secondaryProjection);
        }

        response = new FixedLeverCoulombResponse3d(
            hasAppliedImpulse: true,
            firstLinear,
            firstAngular,
            secondLinear,
            secondAngular,
            hasPrimaryProjection,
            primaryProjection,
            hasSecondaryProjection,
            secondaryProjection);
        return true;
    }

    private static bool TryGetCompletedNormalAccumulatorRatio(
        in FixedLeverNormalConstraint3d constraint,
        Span<ulong> numerator,
        Span<ulong> denominator)
    {
        numerator.Clear();
        denominator.Clear();
        FixedLeverResponseOperand3d first = constraint.First;
        FixedLeverResponseOperand3d second = constraint.Second;
        bool inputsValid =
            first.Lever.Denominator.Sign != 0
            & second.Lever.Denominator.Sign != 0
            & constraint.Restitution >= Fixed64.Zero
            & constraint.RestitutionVelocityThreshold >= Fixed64.Zero
            & constraint.AccumulatedImpulse >= Fixed64.Zero
            & constraint.PositiveImpulseScale >= Fixed64.Zero
            & constraint.NegativeImpulseScale >= Fixed64.Zero
            & first.InverseMass >= Fixed64.Zero
            & second.InverseMass >= Fixed64.Zero
            & constraint.Normal.IsNormalized();
        if (!inputsValid)
            return false;

        _ = WideLever3d.TryGetRelativePointVelocityRatio(
            first.LinearVelocity,
            first.AngularVelocity,
            first.Lever.Value,
            second.LinearVelocity,
            second.AngularVelocity,
            second.Lever.Value,
            constraint.Normal,
            out Signed832 velocityNumerator,
            out Signed832 velocityDenominator);

        Span<ulong> effectiveNumerator = stackalloc ulong[MaxResponseWords];
        Span<ulong> effectiveDenominator = stackalloc ulong[MaxResponseWords];
        GetEffectiveMassRatio(
            first,
            second,
            constraint.Normal,
            effectiveNumerator,
            effectiveDenominator);
        int velocitySign =
            velocityNumerator.Sign * velocityDenominator.Sign;
        if (velocitySign == 0
            || WideArithmetic.IsZeroMagnitude(effectiveNumerator))
        {
            SetMagnitude(constraint.AccumulatedImpulse, numerator);
            denominator[0] = 1UL;
            return true;
        }

        bool hasVelocityProjection = Fixed64.TryGetSignedRawRatio(
            velocityNumerator,
            velocityDenominator,
            0,
            out Fixed64 velocityProjection);
        bool applyRestitution = velocitySign < 0
            & (!hasVelocityProjection
                | velocityProjection
                    < -constraint.RestitutionVelocityThreshold);
        Fixed64 scale = velocitySign < 0
            ? constraint.PositiveImpulseScale
            : constraint.NegativeImpulseScale;
        Span<ulong> impulseNumerator = stackalloc ulong[MaxResponseWords];
        Span<ulong> impulseDenominator = stackalloc ulong[MaxResponseWords];
        BuildImpulseRatio(
            velocityNumerator,
            velocityDenominator,
            effectiveNumerator,
            effectiveDenominator,
            applyRestitution ? constraint.Restitution : Fixed64.Zero,
            scale,
            impulseNumerator,
            impulseDenominator);

        Span<ulong> accumulatorMagnitude =
            stackalloc ulong[MaxResponseWords];
        Span<ulong> accumulatorAtImpulseDenominator =
            stackalloc ulong[MaxResponseWords];
        SetMagnitude(constraint.AccumulatedImpulse, accumulatorMagnitude);
        WideArithmetic.MultiplyMagnitudes(
            accumulatorMagnitude,
            impulseDenominator,
            accumulatorAtImpulseDenominator);
        int impulseSign = -velocitySign;
        if (impulseSign < 0
            && WideArithmetic.CompareMagnitudeEqualLength(
                impulseNumerator,
                accumulatorAtImpulseDenominator) >= 0)
        {
            denominator[0] = 1UL;
            return true;
        }

        accumulatorAtImpulseDenominator.CopyTo(numerator);
        if (impulseSign > 0)
            WideArithmetic.AddMagnitudeInto(impulseNumerator, numerator);
        else
            WideArithmetic.SubtractEqualMagnitudes(
                accumulatorAtImpulseDenominator,
                impulseNumerator,
                numerator);
        impulseDenominator.CopyTo(denominator);
        return true;
    }

    private static void GetBilateralImpulseRatio(
        in FixedLeverResponseOperand3d first,
        in FixedLeverResponseOperand3d second,
        Vector3d tangent,
        Span<ulong> numerator,
        Span<ulong> denominator,
        out int sign)
    {
        numerator.Clear();
        denominator.Clear();
        sign = 0;
        _ = WideLever3d.TryGetRelativePointVelocityRatio(
            first.LinearVelocity,
            first.AngularVelocity,
            first.Lever.Value,
            second.LinearVelocity,
            second.AngularVelocity,
            second.Lever.Value,
            tangent,
            out Signed832 velocityNumerator,
            out Signed832 velocityDenominator);

        Span<ulong> effectiveNumerator = stackalloc ulong[MaxResponseWords];
        Span<ulong> effectiveDenominator = stackalloc ulong[MaxResponseWords];
        GetEffectiveMassRatio(
            first,
            second,
            tangent,
            effectiveNumerator,
            effectiveDenominator);
        int velocitySign =
            velocityNumerator.Sign * velocityDenominator.Sign;
        if (velocitySign == 0
            || WideArithmetic.IsZeroMagnitude(effectiveNumerator))
        {
            denominator[0] = 1UL;
            return;
        }

        BuildImpulseRatio(
            velocityNumerator,
            velocityDenominator,
            effectiveNumerator,
            effectiveDenominator,
            Fixed64.Zero,
            Fixed64.One,
            numerator,
        denominator);
        sign = -velocitySign;
    }

    private static void GetFrictionLimit(
        ReadOnlySpan<ulong> normalNumerator,
        ReadOnlySpan<ulong> normalDenominator,
        Fixed64 friction,
        Span<ulong> numerator,
        Span<ulong> denominator)
    {
        Span<ulong> frictionMagnitude =
            stackalloc ulong[3];
        Span<ulong> fixedScale =
            stackalloc ulong[1];
        SetMagnitude(friction, frictionMagnitude);
        fixedScale.Clear();
        fixedScale[0] = (ulong)FixedMath.ONE_L;
        WideArithmetic.MultiplyMagnitudes(
            normalNumerator,
            frictionMagnitude,
            numerator);
        WideArithmetic.MultiplyMagnitudes(
            normalDenominator,
            fixedScale,
            denominator);
    }

    private static void AddFixedToRatio(
        ReadOnlySpan<ulong> ratioNumerator,
        ReadOnlySpan<ulong> ratioDenominator,
        int ratioSign,
        Fixed64 value,
        Span<ulong> resultNumerator,
        out int resultSign)
    {
        Span<ulong> valueMagnitude = stackalloc ulong[3];
        Span<ulong> valueAtDenominator =
            stackalloc ulong[MaxResponseWords];
        SetMagnitude(value, valueMagnitude);
        WideArithmetic.MultiplyMagnitudes(
            valueMagnitude,
            ratioDenominator,
            valueAtDenominator);
        AddSignedMagnitudes(
            ratioNumerator,
            ratioSign,
            valueAtDenominator,
            value == Fixed64.Zero ? 0 : value < Fixed64.Zero ? -1 : 1,
            resultNumerator,
            out resultSign);
    }

    private static void SubtractFixedFromRatio(
        ReadOnlySpan<ulong> ratioNumerator,
        ReadOnlySpan<ulong> ratioDenominator,
        int ratioSign,
        Fixed64 value,
        Span<ulong> resultNumerator,
        Span<ulong> resultDenominator,
        out int resultSign)
    {
        Span<ulong> valueMagnitude = stackalloc ulong[3];
        Span<ulong> valueAtDenominator =
            stackalloc ulong[MaxCoulombWords];
        SetMagnitude(value, valueMagnitude);
        WideArithmetic.MultiplyMagnitudes(
            valueMagnitude,
            ratioDenominator,
            valueAtDenominator);
        AddSignedMagnitudes(
            ratioNumerator,
            ratioSign,
            valueAtDenominator,
            value == Fixed64.Zero ? 0 : value < Fixed64.Zero ? 1 : -1,
            resultNumerator,
            out resultSign);
        resultDenominator.Clear();
        ratioDenominator.CopyTo(resultDenominator);
    }

    private static int CompareRatios(
        ReadOnlySpan<ulong> firstNumerator,
        ReadOnlySpan<ulong> firstDenominator,
        ReadOnlySpan<ulong> secondNumerator,
        ReadOnlySpan<ulong> secondDenominator)
    {
        Span<ulong> first = stackalloc ulong[MaxCoulombWords];
        Span<ulong> second = stackalloc ulong[MaxCoulombWords];
        WideArithmetic.MultiplyMagnitudes(
            firstNumerator,
            secondDenominator,
            first);
        WideArithmetic.MultiplyMagnitudes(
            secondNumerator,
            firstDenominator,
            second);
        return WideArithmetic.CompareMagnitudeEqualLength(first, second);
    }

    private static bool IsVectorWithinLimit(
        ReadOnlySpan<ulong> vectorMagnitudeSquared,
        ReadOnlySpan<ulong> vectorDenominator,
        ReadOnlySpan<ulong> limitNumerator,
        ReadOnlySpan<ulong> limitDenominator)
    {
        Span<ulong> denominatorSquared =
            stackalloc ulong[MaxCoulombSquareWords];
        Span<ulong> limitNumeratorSquared =
            stackalloc ulong[MaxCoulombSquareWords];
        Span<ulong> limitDenominatorSquared =
            stackalloc ulong[MaxCoulombSquareWords];
        Span<ulong> left = stackalloc ulong[MaxCoulombComparisonWords];
        Span<ulong> right = stackalloc ulong[MaxCoulombComparisonWords];
        WideArithmetic.MultiplyMagnitudes(
            vectorDenominator,
            vectorDenominator,
            denominatorSquared);
        WideArithmetic.MultiplyMagnitudes(
            limitNumerator,
            limitNumerator,
            limitNumeratorSquared);
        WideArithmetic.MultiplyMagnitudes(
            limitDenominator,
            limitDenominator,
            limitDenominatorSquared);
        WideArithmetic.MultiplyMagnitudes(
            vectorMagnitudeSquared,
            limitDenominatorSquared,
            left);
        WideArithmetic.MultiplyMagnitudes(
            limitNumeratorSquared,
            denominatorSquared,
            right);
        return WideArithmetic.CompareMagnitudeEqualLength(left, right) <= 0;
    }

    private static bool TryGetDiskVelocityDeltas(
        in FixedLeverResponseOperand3d primaryFirst,
        in FixedLeverResponseOperand3d primarySecond,
        in FixedLeverResponseOperand3d secondaryFirst,
        in FixedLeverResponseOperand3d secondarySecond,
        Vector3d primaryTangent,
        Vector3d secondaryTangent,
        ReadOnlySpan<ulong> primaryNumerator,
        int primarySign,
        ReadOnlySpan<ulong> secondaryNumerator,
        int secondarySign,
        ReadOnlySpan<ulong> commonDenominator,
        bool rational,
        ReadOnlySpan<ulong> radialNumerator,
        ReadOnlySpan<ulong> radialDenominator,
        ReadOnlySpan<ulong> radicand,
        out Vector3d firstLinear,
        out Vector3d firstAngular,
        out Vector3d secondLinear,
        out Vector3d secondAngular)
    {
        bool resolved = TryGetDiskLinearVelocityDelta(
            primaryFirst.LinearImpulseAxis,
            secondaryFirst.LinearImpulseAxis,
            primaryFirst.InverseMass,
            primaryNumerator,
            primarySign,
            secondaryNumerator,
            secondarySign,
            commonDenominator,
            rational,
            radialNumerator,
            radialDenominator,
            radicand,
            out firstLinear);
        resolved &= TryGetDiskAngularVelocityDelta(
            primaryFirst.Lever,
            -primaryTangent,
            -secondaryTangent,
            primaryFirst.InverseInertia,
            primaryNumerator,
            primarySign,
            secondaryNumerator,
            secondarySign,
            commonDenominator,
            rational,
            radialNumerator,
            radialDenominator,
            radicand,
            out firstAngular);
        resolved &= TryGetDiskLinearVelocityDelta(
            primarySecond.LinearImpulseAxis,
            secondarySecond.LinearImpulseAxis,
            primarySecond.InverseMass,
            primaryNumerator,
            primarySign,
            secondaryNumerator,
            secondarySign,
            commonDenominator,
            rational,
            radialNumerator,
            radialDenominator,
            radicand,
            out secondLinear);
        resolved &= TryGetDiskAngularVelocityDelta(
            primarySecond.Lever,
            primaryTangent,
            secondaryTangent,
            primarySecond.InverseInertia,
            primaryNumerator,
            primarySign,
            secondaryNumerator,
            secondarySign,
            commonDenominator,
            rational,
            radialNumerator,
            radialDenominator,
            radicand,
            out secondAngular);
        return resolved;
    }

    private static bool TryGetDiskLinearVelocityDelta(
        Vector3d primaryAxis,
        Vector3d secondaryAxis,
        Fixed64 inverseMass,
        ReadOnlySpan<ulong> primaryNumerator,
        int primarySign,
        ReadOnlySpan<ulong> secondaryNumerator,
        int secondarySign,
        ReadOnlySpan<ulong> commonDenominator,
        bool rational,
        ReadOnlySpan<ulong> radialNumerator,
        ReadOnlySpan<ulong> radialDenominator,
        ReadOnlySpan<ulong> radicand,
        out Vector3d result)
    {
        bool xResolved = TryGetDiskLinearComponent(
            primaryAxis.X,
            secondaryAxis.X,
            inverseMass,
            primaryNumerator,
            primarySign,
            secondaryNumerator,
            secondarySign,
            commonDenominator,
            rational,
            radialNumerator,
            radialDenominator,
            radicand,
            out Fixed64 x);
        bool yResolved = TryGetDiskLinearComponent(
            primaryAxis.Y,
            secondaryAxis.Y,
            inverseMass,
            primaryNumerator,
            primarySign,
            secondaryNumerator,
            secondarySign,
            commonDenominator,
            rational,
            radialNumerator,
            radialDenominator,
            radicand,
            out Fixed64 y);
        bool zResolved = TryGetDiskLinearComponent(
            primaryAxis.Z,
            secondaryAxis.Z,
            inverseMass,
            primaryNumerator,
            primarySign,
            secondaryNumerator,
            secondarySign,
            commonDenominator,
            rational,
            radialNumerator,
            radialDenominator,
            radicand,
            out Fixed64 z);
        result = xResolved & yResolved & zResolved
            ? new Vector3d(x, y, z)
            : default;
        return xResolved & yResolved & zResolved;
    }

    private static bool TryGetDiskLinearComponent(
        Fixed64 primaryAxis,
        Fixed64 secondaryAxis,
        Fixed64 inverseMass,
        ReadOnlySpan<ulong> primaryNumerator,
        int primarySign,
        ReadOnlySpan<ulong> secondaryNumerator,
        int secondarySign,
        ReadOnlySpan<ulong> commonDenominator,
        bool rational,
        ReadOnlySpan<ulong> radialNumerator,
        ReadOnlySpan<ulong> radialDenominator,
        ReadOnlySpan<ulong> radicand,
        out Fixed64 result)
    {
        Span<ulong> combined = stackalloc ulong[MaxCoulombWords + 2];
        GetWeightedSignedSum(
            primaryAxis,
            primaryNumerator,
            primarySign,
            secondaryAxis,
            secondaryNumerator,
            secondarySign,
            combined,
            out int combinedSign);
        if (combinedSign == 0 || inverseMass == Fixed64.Zero)
        {
            result = Fixed64.Zero;
            return true;
        }

        Span<ulong> inverseMassMagnitude =
            stackalloc ulong[3];
        Span<ulong> fixedScale = stackalloc ulong[1];
        Span<ulong> numerator = stackalloc ulong[MaxCoulombComparisonWords / 2];
        Span<ulong> denominator =
            stackalloc ulong[MaxCoulombComparisonWords / 2];
        SetMagnitude(inverseMass, inverseMassMagnitude);
        fixedScale.Clear();
        fixedScale[0] = (ulong)FixedMath.ONE_L;
        WideArithmetic.MultiplyMagnitudes(
            combined,
            inverseMassMagnitude,
            numerator);
        Multiply3(
            commonDenominator,
            fixedScale,
            fixedScale,
            denominator);
        if (rational)
        {
            return Fixed64.TryGetSignedRawRatio(
                numerator,
                denominator,
                combinedSign < 0,
                out result);
        }

        Span<ulong> scaledNumerator =
            stackalloc ulong[MaxCoulombComparisonWords / 2];
        Span<ulong> scaledDenominator =
            stackalloc ulong[MaxCoulombComparisonWords / 2];
        WideArithmetic.MultiplyMagnitudes(
            numerator,
            radialNumerator,
            scaledNumerator);
        WideArithmetic.MultiplyMagnitudes(
            radialDenominator,
            fixedScale,
            scaledDenominator);
        WideArithmetic.MultiplyMagnitudes(
            scaledDenominator,
            fixedScale,
            denominator);
        return TryGetSignedRatioOverSquareRoot(
            scaledNumerator,
            denominator,
            radicand,
            combinedSign < 0,
            out result);
    }

    private static bool TryGetDiskAngularVelocityDelta(
        in FixedLever lever,
        Vector3d primaryAxis,
        Vector3d secondaryAxis,
        Fixed3x3 inverseInertia,
        ReadOnlySpan<ulong> primaryNumerator,
        int primarySign,
        ReadOnlySpan<ulong> secondaryNumerator,
        int secondarySign,
        ReadOnlySpan<ulong> commonDenominator,
        bool rational,
        ReadOnlySpan<ulong> radialNumerator,
        ReadOnlySpan<ulong> radialDenominator,
        ReadOnlySpan<ulong> radicand,
        out Vector3d result)
    {
        WideLever3d.GetTransformedCrossProduct(
            lever.Value,
            primaryAxis,
            inverseInertia,
            out Signed832 primaryX,
            out Signed832 primaryY,
            out Signed832 primaryZ);
        WideLever3d.GetTransformedCrossProduct(
            lever.Value,
            secondaryAxis,
            inverseInertia,
            out Signed832 secondaryX,
            out Signed832 secondaryY,
            out Signed832 secondaryZ);
        Signed320 fixedScaleSquared = WideArithmetic.MultiplySigned192(
            Signed192.One,
            Signed192.One);
        Signed704 transformedDenominator =
            WideArithmetic.MultiplySigned576ToSigned704(
                lever.Denominator,
                fixedScaleSquared);

        bool xResolved = TryGetDiskAngularComponent(
            primaryX,
            secondaryX,
            transformedDenominator,
            primaryNumerator,
            primarySign,
            secondaryNumerator,
            secondarySign,
            commonDenominator,
            rational,
            radialNumerator,
            radialDenominator,
            radicand,
            out Fixed64 x);
        bool yResolved = TryGetDiskAngularComponent(
            primaryY,
            secondaryY,
            transformedDenominator,
            primaryNumerator,
            primarySign,
            secondaryNumerator,
            secondarySign,
            commonDenominator,
            rational,
            radialNumerator,
            radialDenominator,
            radicand,
            out Fixed64 y);
        bool zResolved = TryGetDiskAngularComponent(
            primaryZ,
            secondaryZ,
            transformedDenominator,
            primaryNumerator,
            primarySign,
            secondaryNumerator,
            secondarySign,
            commonDenominator,
            rational,
            radialNumerator,
            radialDenominator,
            radicand,
            out Fixed64 z);
        result = xResolved & yResolved & zResolved
            ? new Vector3d(x, y, z)
            : default;
        return xResolved & yResolved & zResolved;
    }

    private static bool TryGetDiskAngularComponent(
        Signed832 primary,
        Signed832 secondary,
        Signed704 transformedDenominator,
        ReadOnlySpan<ulong> primaryNumerator,
        int primarySign,
        ReadOnlySpan<ulong> secondaryNumerator,
        int secondarySign,
        ReadOnlySpan<ulong> commonDenominator,
        bool rational,
        ReadOnlySpan<ulong> radialNumerator,
        ReadOnlySpan<ulong> radialDenominator,
        ReadOnlySpan<ulong> radicand,
        out Fixed64 result)
    {
        Span<ulong> primaryMagnitude = stackalloc ulong[13];
        Span<ulong> secondaryMagnitude = stackalloc ulong[13];
        SetMagnitude(primary, primaryMagnitude);
        SetMagnitude(secondary, secondaryMagnitude);
        Span<ulong> primaryProduct =
            stackalloc ulong[MaxCoulombComparisonWords / 2];
        Span<ulong> secondaryProduct =
            stackalloc ulong[MaxCoulombComparisonWords / 2];
        WideArithmetic.MultiplyMagnitudes(
            primaryMagnitude,
            primaryNumerator,
            primaryProduct);
        WideArithmetic.MultiplyMagnitudes(
            secondaryMagnitude,
            secondaryNumerator,
            secondaryProduct);
        Span<ulong> combined =
            stackalloc ulong[MaxCoulombComparisonWords / 2];
        AddSignedMagnitudes(
            primaryProduct,
            primary.Sign * primarySign,
            secondaryProduct,
            secondary.Sign * secondarySign,
            combined,
            out int combinedSign);
        if (combinedSign == 0)
        {
            result = Fixed64.Zero;
            return true;
        }

        Span<ulong> transformedDenominatorMagnitude =
            stackalloc ulong[11];
        Span<ulong> fixedScale = stackalloc ulong[1];
        Span<ulong> denominator =
            stackalloc ulong[MaxCoulombComparisonWords / 2];
        SetMagnitude(transformedDenominator, transformedDenominatorMagnitude);
        fixedScale.Clear();
        fixedScale[0] = (ulong)FixedMath.ONE_L;
        Multiply3(
            transformedDenominatorMagnitude,
            commonDenominator,
            fixedScale,
            denominator);
        if (rational)
        {
            return Fixed64.TryGetSignedRawRatio(
                combined,
                denominator,
                combinedSign < 0,
                out result);
        }

        Span<ulong> scaledNumerator =
            stackalloc ulong[MaxCoulombComparisonWords / 2];
        Span<ulong> scaledDenominator =
            stackalloc ulong[MaxCoulombComparisonWords / 2];
        WideArithmetic.MultiplyMagnitudes(
            combined,
            radialNumerator,
            scaledNumerator);
        WideArithmetic.MultiplyMagnitudes(
            transformedDenominatorMagnitude,
            radialDenominator,
            scaledDenominator);
        WideArithmetic.MultiplyMagnitudes(
            scaledDenominator,
            fixedScale,
            denominator);
        return TryGetSignedRatioOverSquareRoot(
            scaledNumerator,
            denominator,
            radicand,
            combinedSign < 0,
            out result);
    }

    private static void GetWeightedSignedSum(
        Fixed64 firstFactor,
        ReadOnlySpan<ulong> firstMagnitude,
        int firstSign,
        Fixed64 secondFactor,
        ReadOnlySpan<ulong> secondMagnitude,
        int secondSign,
        Span<ulong> result,
        out int resultSign)
    {
        Span<ulong> factorMagnitude = stackalloc ulong[3];
        Span<ulong> first = stackalloc ulong[MaxCoulombWords + 2];
        Span<ulong> second = stackalloc ulong[MaxCoulombWords + 2];
        SetMagnitude(firstFactor, factorMagnitude);
        WideArithmetic.MultiplyMagnitudes(
            factorMagnitude,
            firstMagnitude,
            first);
        SetMagnitude(secondFactor, factorMagnitude);
        WideArithmetic.MultiplyMagnitudes(
            factorMagnitude,
            secondMagnitude,
            second);
        AddSignedMagnitudes(
            first,
            firstFactor == Fixed64.Zero
                ? 0
                : firstFactor < Fixed64.Zero ? -firstSign : firstSign,
            second,
            secondFactor == Fixed64.Zero
                ? 0
                : secondFactor < Fixed64.Zero ? -secondSign : secondSign,
            result,
            out resultSign);
    }

    private static void AddSignedMagnitudes(
        ReadOnlySpan<ulong> first,
        int firstSign,
        ReadOnlySpan<ulong> second,
        int secondSign,
        Span<ulong> result,
        out int resultSign)
    {
        result.Clear();
        resultSign = 0;
        if (firstSign != 0 & !WideArithmetic.IsZeroMagnitude(first))
            WideArithmetic.AddSignedMagnitude(
                first,
                firstSign,
                result,
                ref resultSign);
        if (secondSign != 0 & !WideArithmetic.IsZeroMagnitude(second))
            WideArithmetic.AddSignedMagnitude(
                second,
                secondSign,
                result,
                ref resultSign);
    }

    private static bool TryGetSignedRatioOverSquareRoot(
        ReadOnlySpan<ulong> numerator,
        ReadOnlySpan<ulong> denominator,
        ReadOnlySpan<ulong> radicand,
        bool negative,
        out Fixed64 result)
    {
        if (WideArithmetic.IsZeroMagnitude(numerator))
        {
            result = Fixed64.Zero;
            return true;
        }

        ulong limit = negative ? 1UL << 63 : (ulong)long.MaxValue;
        if (CompareRatioOverSquareRoot(
                numerator,
                denominator,
                radicand,
                limit) > 0)
        {
            result = default;
            return false;
        }

        ulong low = 0UL;
        ulong high = limit;
        while (low < high)
        {
            ulong difference = high - low;
            ulong middle = low + (difference >> 1) + (difference & 1UL);
            if (CompareRatioOverSquareRoot(
                    numerator,
                    denominator,
                    radicand,
                    middle) >= 0)
            {
                low = middle;
            }
            else
            {
                high = middle - 1UL;
            }
        }

        ulong quotient = low;
        Span<ulong> doubledQuotient = stackalloc ulong[2];
        doubledQuotient[0] = (quotient << 1) | 1UL;
        doubledQuotient[1] = quotient >> 63;
        int midpointComparison = CompareRatioOverSquareRoot(
            numerator,
            denominator,
            radicand,
            doubledQuotient,
            numeratorMultiplier: 2UL);
        bool roundUp = midpointComparison > 0
            | (midpointComparison == 0 & (quotient & 1UL) != 0UL);
        if (roundUp & quotient < limit)
        {
            quotient++;
        }

        result = new Fixed64(
            negative ? unchecked(-(long)quotient) : (long)quotient);
        return true;
    }

    private static int CompareRatioOverSquareRoot(
        ReadOnlySpan<ulong> numerator,
        ReadOnlySpan<ulong> denominator,
        ReadOnlySpan<ulong> radicand,
        ulong candidate)
    {
        Span<ulong> candidateMagnitude = stackalloc ulong[1];
        candidateMagnitude.Clear();
        candidateMagnitude[0] = candidate;
        return CompareRatioOverSquareRoot(
            numerator,
            denominator,
            radicand,
            candidateMagnitude,
            numeratorMultiplier: 1UL);
    }

    private static int CompareRatioOverSquareRoot(
        ReadOnlySpan<ulong> numerator,
        ReadOnlySpan<ulong> denominator,
        ReadOnlySpan<ulong> radicand,
        ReadOnlySpan<ulong> candidate,
        ulong numeratorMultiplier)
    {
        Span<ulong> multiplier = stackalloc ulong[1];
        multiplier.Clear();
        multiplier[0] = numeratorMultiplier;
        Span<ulong> scaledNumerator =
            stackalloc ulong[(MaxCoulombComparisonWords / 2) + 1];
        Span<ulong> left = stackalloc ulong[MaxCoulombComparisonWords];
        Span<ulong> denominatorTimesCandidate =
            stackalloc ulong[(MaxCoulombComparisonWords / 2) + 1];
        Span<ulong> rightBase =
            stackalloc ulong[MaxCoulombComparisonWords];
        Span<ulong> right = stackalloc ulong[MaxCoulombComparisonWords];
        WideArithmetic.MultiplyMagnitudes(
            numerator,
            multiplier,
            scaledNumerator);
        WideArithmetic.MultiplyMagnitudes(
            scaledNumerator,
            scaledNumerator,
            left);
        WideArithmetic.MultiplyMagnitudes(
            denominator,
            candidate,
            denominatorTimesCandidate);
        WideArithmetic.MultiplyMagnitudes(
            denominatorTimesCandidate,
            denominatorTimesCandidate,
            rightBase);
        WideArithmetic.MultiplyMagnitudes(rightBase, radicand, right);
        return WideArithmetic.CompareMagnitudeEqualLength(left, right);
    }

    private static bool HaveMatchingParticipants(
        in FixedLeverResponseOperand3d first,
        in FixedLeverResponseOperand3d second) =>
        first.Lever.XNumerator.Equals(second.Lever.XNumerator)
        & first.Lever.YNumerator.Equals(second.Lever.YNumerator)
        & first.Lever.ZNumerator.Equals(second.Lever.ZNumerator)
        & first.Lever.Denominator.Equals(second.Lever.Denominator)
        & first.InverseMass == second.InverseMass
        & first.InverseInertia == second.InverseInertia;
}
