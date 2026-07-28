//=======================================================================
// WideOrientedBox.NormalResponse.cs
//=======================================================================
// MIT License, Copyright (c) 2026–present David Oravsky (mrdav30)
// See LICENSE file in the project root for full license information.
//=======================================================================

using System;
using System.Runtime.CompilerServices;

namespace FixedMathSharp.Geometry;

internal static partial class WideOrientedBox
{
    // Valid point-anchor denominators are below 202 bits. The largest completed
    // response ratio is below 2,944 bits, including one carry word.
    private const int MaxResponseWords = 48;

    [MethodImpl(MethodImplOptions.NoInlining)]
    internal static bool TryGetNormalResponse(
        in FixedLeverResponseOperand3d first,
        in FixedLeverResponseOperand3d second,
        Vector3d normal,
        Fixed64 restitution,
        Fixed64 restitutionVelocityThreshold,
        Fixed64 accumulatedImpulse,
        Fixed64 positiveImpulseScale,
        Fixed64 negativeImpulseScale,
        bool includeAccumulator,
        out FixedLeverNormalResponse3d response)
    {
        response = default;
        if (first.Lever.Denominator.Sign == 0
            || second.Lever.Denominator.Sign == 0
            || restitution < Fixed64.Zero
            || restitutionVelocityThreshold < Fixed64.Zero
            || accumulatedImpulse < Fixed64.Zero
            || positiveImpulseScale < Fixed64.Zero
            || negativeImpulseScale < Fixed64.Zero
            || first.InverseMass < Fixed64.Zero
            || second.InverseMass < Fixed64.Zero
            || !normal.IsNormalized()
            || !TryGetRelativePointVelocityRatio(
                first.LinearVelocity,
                first.AngularVelocity,
                first.Lever,
                second.LinearVelocity,
                second.AngularVelocity,
                second.Lever,
                normal,
                out Signed832 velocityNumerator,
                out Signed832 velocityDenominator))
        {
            return false;
        }

        int velocitySign =
            velocityNumerator.Sign * velocityDenominator.Sign;
        bool hasNormalVelocity = Fixed64.TryGetSignedRawRatio(
            velocityNumerator,
            velocityDenominator,
            0,
            out Fixed64 normalVelocity);
        Span<ulong> effectiveNumerator =
            stackalloc ulong[MaxResponseWords];
        Span<ulong> effectiveDenominator =
            stackalloc ulong[MaxResponseWords];
        GetEffectiveMassRatio(
            first,
            second,
            normal,
            effectiveNumerator,
            effectiveDenominator);

        bool denominatorTooSmall = IsZero(effectiveNumerator);
        if (denominatorTooSmall)
        {
            if (velocitySign < 0 && !includeAccumulator)
                return false;

            response = CreateZeroResponse(
                velocitySign < 0,
                hasNormalVelocity,
                normalVelocity,
                includeAccumulator,
                accumulatedImpulse);
            return true;
        }

        if (velocitySign == 0)
        {
            response = CreateZeroResponse(
                isClosing: false,
                hasNormalVelocity,
                normalVelocity,
                includeAccumulator,
                accumulatedImpulse);
            return true;
        }

        bool applyRestitution = velocitySign < 0
            && (!hasNormalVelocity
                || normalVelocity < -restitutionVelocityThreshold);
        Fixed64 impulseScale = velocitySign < 0
            ? positiveImpulseScale
            : negativeImpulseScale;
        Span<ulong> impulseNumerator =
            stackalloc ulong[MaxResponseWords];
        Span<ulong> impulseDenominator =
            stackalloc ulong[MaxResponseWords];
        BuildImpulseRatio(
            velocityNumerator,
            velocityDenominator,
            effectiveNumerator,
            effectiveDenominator,
            applyRestitution ? restitution : Fixed64.Zero,
            impulseScale,
            impulseNumerator,
            impulseDenominator);
        int impulseSign = -velocitySign;

        Span<ulong> appliedNumerator =
            stackalloc ulong[MaxResponseWords];
        Span<ulong> appliedDenominator =
            stackalloc ulong[MaxResponseWords];
        appliedNumerator.Clear();
        appliedDenominator.Clear();
        ResolveUnilateralImpulse(
            impulseNumerator,
            impulseDenominator,
            impulseSign,
            accumulatedImpulse,
            includeAccumulator,
            appliedNumerator,
            appliedDenominator,
            out int appliedSign,
            out bool useImpulseRatio,
            out bool hasAccumulatedProjection,
            out Fixed64 accumulatedProjection);

        ReadOnlySpan<ulong> resolvedImpulseNumerator =
            useImpulseRatio ? impulseNumerator : appliedNumerator;
        ReadOnlySpan<ulong> resolvedImpulseDenominator =
            useImpulseRatio ? impulseDenominator : appliedDenominator;
        bool hasAppliedImpulse = !IsZero(resolvedImpulseNumerator);
        bool hasAppliedProjection = Fixed64.TryGetSignedRawRatio(
            resolvedImpulseNumerator,
            resolvedImpulseDenominator,
            appliedSign < 0,
            out Fixed64 appliedImpulse);
        bool resolved = TryGetLinearVelocityDelta(
            first.LinearImpulseAxis,
            first.InverseMass,
            resolvedImpulseNumerator,
            resolvedImpulseDenominator,
            appliedSign,
            out Vector3d firstLinear);
        resolved &= TryGetAngularVelocityDelta(
            first.Lever,
            -normal,
            first.InverseInertia,
            resolvedImpulseNumerator,
            resolvedImpulseDenominator,
            appliedSign,
            out Vector3d firstAngular);
        resolved &= TryGetLinearVelocityDelta(
            second.LinearImpulseAxis,
            second.InverseMass,
            resolvedImpulseNumerator,
            resolvedImpulseDenominator,
            appliedSign,
            out Vector3d secondLinear);
        resolved &= TryGetAngularVelocityDelta(
            second.Lever,
            normal,
            second.InverseInertia,
            resolvedImpulseNumerator,
            resolvedImpulseDenominator,
            appliedSign,
            out Vector3d secondAngular);
        if (!resolved)
            return false;

        response = new FixedLeverNormalResponse3d(
            velocitySign < 0,
            hasAppliedImpulse,
            firstLinear,
            firstAngular,
            secondLinear,
            secondAngular,
            hasNormalVelocity,
            normalVelocity,
            hasAppliedProjection,
            appliedImpulse,
            includeAccumulator && hasAccumulatedProjection,
            accumulatedProjection);
        return true;
    }

    private static FixedLeverNormalResponse3d CreateZeroResponse(
        bool isClosing,
        bool hasNormalVelocity,
        Fixed64 normalVelocity,
        bool includeAccumulator,
        Fixed64 accumulatedImpulse) =>
        new(
            isClosing,
            hasAppliedImpulse: false,
            Vector3d.Zero,
            Vector3d.Zero,
            Vector3d.Zero,
            Vector3d.Zero,
            hasNormalVelocity,
            normalVelocity,
            hasAppliedImpulseProjection: true,
            Fixed64.Zero,
            includeAccumulator,
            accumulatedImpulse);

    private static void GetEffectiveMassRatio(
        in FixedLeverResponseOperand3d first,
        in FixedLeverResponseOperand3d second,
        Vector3d normal,
        Span<ulong> resultNumerator,
        Span<ulong> resultDenominator)
    {
        Signed192 normalSquared = GetRawDot(normal, normal);

        Signed192 firstAllowed =
            GetRawDot(first.LinearImpulseAxis, -normal);
        Signed192 secondAllowed =
            GetRawDot(second.LinearImpulseAxis, normal);
        Signed320 linearNumerator = default;
        if (firstAllowed.Sign > 0
            && first.InverseMass > Fixed64.Zero)
        {
            linearNumerator = WideArithmetic.MultiplySigned192(
                firstAllowed,
                Signed192.Raw(first.InverseMass));
        }
        if (secondAllowed.Sign > 0
            && second.InverseMass > Fixed64.Zero)
        {
            linearNumerator = WideArithmetic.AddSigned320(
                linearNumerator,
                WideArithmetic.MultiplySigned192(
                    secondAllowed,
                    Signed192.Raw(second.InverseMass)));
        }

        GetCrossProductQuadraticFormRatio(
            first.Lever,
            normal,
            first.InverseInertia,
            out Signed832 firstAngularNumerator,
            out Signed832 firstAngularDenominator);
        GetCrossProductQuadraticFormRatio(
            second.Lever,
            normal,
            second.InverseInertia,
            out Signed832 secondAngularNumerator,
            out Signed832 secondAngularDenominator);

        Span<ulong> linearNumeratorMagnitude =
            stackalloc ulong[MaxResponseWords];
        Span<ulong> linearDenominatorMagnitude =
            stackalloc ulong[MaxResponseWords];
        Span<ulong> firstNumeratorMagnitude =
            stackalloc ulong[MaxResponseWords];
        Span<ulong> firstDenominatorMagnitude =
            stackalloc ulong[MaxResponseWords];
        Span<ulong> secondNumeratorMagnitude =
            stackalloc ulong[MaxResponseWords];
        Span<ulong> secondDenominatorMagnitude =
            stackalloc ulong[MaxResponseWords];
        SetMagnitude(linearNumerator, linearNumeratorMagnitude);
        SetMagnitude(normalSquared, linearDenominatorMagnitude);
        SetNonNegativeRatio(
            firstAngularNumerator,
            firstAngularDenominator,
            firstNumeratorMagnitude,
            firstDenominatorMagnitude);
        SetNonNegativeRatio(
            secondAngularNumerator,
            secondAngularDenominator,
            secondNumeratorMagnitude,
            secondDenominatorMagnitude);

        Span<ulong> temporary = stackalloc ulong[MaxResponseWords];
        Multiply3(
            linearDenominatorMagnitude,
            firstDenominatorMagnitude,
            secondDenominatorMagnitude,
            resultDenominator);
        Multiply3(
            linearNumeratorMagnitude,
            firstDenominatorMagnitude,
            secondDenominatorMagnitude,
            resultNumerator);
        Multiply3(
            firstNumeratorMagnitude,
            linearDenominatorMagnitude,
            secondDenominatorMagnitude,
            temporary);
        WideArithmetic.AddMagnitudeInto(temporary, resultNumerator);
        Multiply3(
            secondNumeratorMagnitude,
            linearDenominatorMagnitude,
            firstDenominatorMagnitude,
            temporary);
        WideArithmetic.AddMagnitudeInto(temporary, resultNumerator);
    }

    private static void BuildImpulseRatio(
        Signed832 velocityNumerator,
        Signed832 velocityDenominator,
        ReadOnlySpan<ulong> effectiveNumerator,
        ReadOnlySpan<ulong> effectiveDenominator,
        Fixed64 restitution,
        Fixed64 impulseScale,
        Span<ulong> numerator,
        Span<ulong> denominator)
    {
        Span<ulong> velocityNumeratorMagnitude =
            stackalloc ulong[MaxResponseWords];
        Span<ulong> velocityDenominatorMagnitude =
            stackalloc ulong[MaxResponseWords];
        Span<ulong> restitutionMagnitude =
            stackalloc ulong[MaxResponseWords];
        Span<ulong> scaleMagnitude =
            stackalloc ulong[MaxResponseWords];
        Span<ulong> fixedScale =
            stackalloc ulong[MaxResponseWords];
        SetMagnitude(velocityNumerator, velocityNumeratorMagnitude);
        SetMagnitude(velocityDenominator, velocityDenominatorMagnitude);
        SetUnsignedSum(
            (ulong)FixedMath.ONE_L,
            (ulong)restitution.m_rawValue,
            restitutionMagnitude);
        SetMagnitude(impulseScale, scaleMagnitude);
        fixedScale.Clear();
        fixedScale[0] = (ulong)FixedMath.ONE_L;

        Multiply4(
            velocityNumeratorMagnitude,
            effectiveDenominator,
            restitutionMagnitude,
            scaleMagnitude,
            numerator);
        Multiply3(
            velocityDenominatorMagnitude,
            effectiveNumerator,
            fixedScale,
            denominator);
    }

    private static void ResolveUnilateralImpulse(
        ReadOnlySpan<ulong> impulseNumerator,
        ReadOnlySpan<ulong> impulseDenominator,
        int impulseSign,
        Fixed64 accumulatedImpulse,
        bool includeAccumulator,
        Span<ulong> appliedNumerator,
        Span<ulong> appliedDenominator,
        out int appliedSign,
        out bool useImpulseRatio,
        out bool hasAccumulatedProjection,
        out Fixed64 accumulatedProjection)
    {
        appliedNumerator.Clear();
        appliedDenominator.Clear();
        useImpulseRatio = false;
        hasAccumulatedProjection = false;
        accumulatedProjection = default;
        if (!includeAccumulator)
        {
            if (impulseSign <= 0)
            {
                appliedDenominator[0] = 1UL;
                appliedSign = 0;
                return;
            }

            appliedSign = 1;
            useImpulseRatio = true;
            return;
        }

        Span<ulong> accumulatorMagnitude =
            stackalloc ulong[MaxResponseWords];
        Span<ulong> accumulatedAtImpulseDenominator =
            stackalloc ulong[MaxResponseWords];
        SetMagnitude(accumulatedImpulse, accumulatorMagnitude);
        WideArithmetic.MultiplyMagnitudes(
            accumulatorMagnitude,
            impulseDenominator,
            accumulatedAtImpulseDenominator);
        if (impulseSign < 0
            && WideArithmetic.CompareMagnitudeEqualLength(
                impulseNumerator,
                accumulatedAtImpulseDenominator) >= 0)
        {
            accumulatorMagnitude.CopyTo(appliedNumerator);
            appliedDenominator[0] = 1UL;
            appliedSign = IsZero(accumulatorMagnitude) ? 0 : -1;
            hasAccumulatedProjection = true;
            accumulatedProjection = Fixed64.Zero;
            return;
        }

        Span<ulong> completedAccumulator =
            stackalloc ulong[MaxResponseWords];
        if (impulseSign > 0)
        {
            accumulatedAtImpulseDenominator.CopyTo(completedAccumulator);
            WideArithmetic.AddMagnitudeInto(
                impulseNumerator,
                completedAccumulator);
        }
        else
        {
            WideArithmetic.SubtractEqualMagnitudes(
                accumulatedAtImpulseDenominator,
                impulseNumerator,
                completedAccumulator);
        }

        hasAccumulatedProjection = Fixed64.TryGetSignedRawRatio(
            completedAccumulator,
            impulseDenominator,
            negative: false,
            out accumulatedProjection);
        appliedSign = impulseSign;
        useImpulseRatio = true;
    }

    private static bool TryGetLinearVelocityDelta(
        Vector3d impulseAxis,
        Fixed64 inverseMass,
        ReadOnlySpan<ulong> impulseNumerator,
        ReadOnlySpan<ulong> impulseDenominator,
        int impulseSign,
        out Vector3d result)
    {
        if (inverseMass == Fixed64.Zero
            || impulseAxis == Vector3d.Zero
            || IsZero(impulseNumerator))
        {
            result = Vector3d.Zero;
            return true;
        }

        bool xResolved = TryGetLinearComponent(
            impulseAxis.X,
            inverseMass,
            impulseNumerator,
            impulseDenominator,
            impulseSign,
            out Fixed64 x);
        bool yResolved = TryGetLinearComponent(
            impulseAxis.Y,
            inverseMass,
            impulseNumerator,
            impulseDenominator,
            impulseSign,
            out Fixed64 y);
        bool zResolved = TryGetLinearComponent(
            impulseAxis.Z,
            inverseMass,
            impulseNumerator,
            impulseDenominator,
            impulseSign,
            out Fixed64 z);
        result = xResolved & yResolved & zResolved
            ? new Vector3d(x, y, z)
            : default;
        return xResolved & yResolved & zResolved;
    }

    private static bool TryGetLinearComponent(
        Fixed64 axis,
        Fixed64 inverseMass,
        ReadOnlySpan<ulong> impulseNumerator,
        ReadOnlySpan<ulong> impulseDenominator,
        int impulseSign,
        out Fixed64 result)
    {
        if (axis == Fixed64.Zero)
        {
            result = Fixed64.Zero;
            return true;
        }

        Span<ulong> axisMagnitude =
            stackalloc ulong[MaxResponseWords];
        Span<ulong> inverseMassMagnitude =
            stackalloc ulong[MaxResponseWords];
        Span<ulong> fixedScale =
            stackalloc ulong[MaxResponseWords];
        Span<ulong> numerator =
            stackalloc ulong[MaxResponseWords];
        Span<ulong> denominator =
            stackalloc ulong[MaxResponseWords];
        SetMagnitude(axis, axisMagnitude);
        SetMagnitude(inverseMass, inverseMassMagnitude);
        fixedScale.Clear();
        fixedScale[0] = (ulong)FixedMath.ONE_L;
        Multiply3(
            axisMagnitude,
            inverseMassMagnitude,
            impulseNumerator,
            numerator);
        Multiply3(
            impulseDenominator,
            fixedScale,
            fixedScale,
            denominator);
        return Fixed64.TryGetSignedRawRatio(
            numerator,
            denominator,
            (axis < Fixed64.Zero) != (impulseSign < 0),
            out result);
    }

    private static bool TryGetAngularVelocityDelta(
        in FixedLever lever,
        Vector3d impulseAxis,
        Fixed3x3 inverseInertia,
        ReadOnlySpan<ulong> impulseNumerator,
        ReadOnlySpan<ulong> impulseDenominator,
        int impulseSign,
        out Vector3d result)
    {
        if (IsZero(impulseNumerator))
        {
            result = Vector3d.Zero;
            return true;
        }

        GetTransformedCrossProduct(
            lever,
            impulseAxis,
            inverseInertia,
            out Signed832 transformedX,
            out Signed832 transformedY,
            out Signed832 transformedZ);
        Signed320 fixedScaleSquared = WideArithmetic.MultiplySigned192(
            Signed192.One,
            Signed192.One);
        Signed704 transformedDenominator =
            WideArithmetic.MultiplySigned576ToSigned704(
                lever.Denominator,
                fixedScaleSquared);
        bool xResolved = TryGetAngularComponent(
            transformedX,
            transformedDenominator,
            impulseNumerator,
            impulseDenominator,
            impulseSign,
            out Fixed64 x);
        bool yResolved = TryGetAngularComponent(
            transformedY,
            transformedDenominator,
            impulseNumerator,
            impulseDenominator,
            impulseSign,
            out Fixed64 y);
        bool zResolved = TryGetAngularComponent(
            transformedZ,
            transformedDenominator,
            impulseNumerator,
            impulseDenominator,
            impulseSign,
            out Fixed64 z);
        result = xResolved & yResolved & zResolved
            ? new Vector3d(x, y, z)
            : default;
        return xResolved & yResolved & zResolved;
    }

    private static bool TryGetAngularComponent(
        Signed832 transformed,
        Signed704 transformedDenominator,
        ReadOnlySpan<ulong> impulseNumerator,
        ReadOnlySpan<ulong> impulseDenominator,
        int impulseSign,
        out Fixed64 result)
    {
        if (transformed.Sign == 0)
        {
            result = Fixed64.Zero;
            return true;
        }

        Span<ulong> transformedMagnitude =
            stackalloc ulong[MaxResponseWords];
        Span<ulong> transformedDenominatorMagnitude =
            stackalloc ulong[MaxResponseWords];
        Span<ulong> fixedScale =
            stackalloc ulong[MaxResponseWords];
        Span<ulong> numerator =
            stackalloc ulong[MaxResponseWords];
        Span<ulong> denominator =
            stackalloc ulong[MaxResponseWords];
        SetMagnitude(transformed, transformedMagnitude);
        SetMagnitude(transformedDenominator, transformedDenominatorMagnitude);
        fixedScale.Clear();
        fixedScale[0] = (ulong)FixedMath.ONE_L;
        WideArithmetic.MultiplyMagnitudes(
            transformedMagnitude,
            impulseNumerator,
            numerator);
        Multiply3(
            transformedDenominatorMagnitude,
            impulseDenominator,
            fixedScale,
            denominator);
        return Fixed64.TryGetSignedRawRatio(
            numerator,
            denominator,
            (transformed.Sign < 0) != (impulseSign < 0),
            out result);
    }

    private static Signed192 GetRawDot(Vector3d left, Vector3d right) =>
        WideGeometry.GetDifferenceDotProduct3D(
            left.X,
            Fixed64.Zero,
            left.Y,
            Fixed64.Zero,
            left.Z,
            Fixed64.Zero,
            right.X,
            Fixed64.Zero,
            right.Y,
            Fixed64.Zero,
            right.Z,
            Fixed64.Zero);

    private static void SetNonNegativeRatio(
        Signed832 numerator,
        Signed832 denominator,
        Span<ulong> numeratorMagnitude,
        Span<ulong> denominatorMagnitude)
    {
        numeratorMagnitude.Clear();
        denominatorMagnitude.Clear();
        if (numerator.Sign <= 0)
        {
            denominatorMagnitude[0] = 1UL;
            return;
        }

        SetMagnitude(numerator, numeratorMagnitude);
        SetMagnitude(denominator, denominatorMagnitude);
    }

    private static bool IsZero(ReadOnlySpan<ulong> value)
    {
        for (int index = 0; index < value.Length; index++)
        {
            if (value[index] != 0UL)
                return false;
        }
        return true;
    }

    private static void SetUnsignedSum(
        ulong first,
        ulong second,
        Span<ulong> result)
    {
        result.Clear();
        result[0] = unchecked(first + second);
    }

    private static void SetMagnitude(
        Fixed64 value,
        Span<ulong> result)
    {
        result.Clear();
        WideArithmetic.GetMagnitude(
            Signed192.Raw(value),
            out result[2],
            out result[1],
            out result[0]);
    }

    private static void SetMagnitude(
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

    private static void SetMagnitude(
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

    private static void SetMagnitude(
        Signed704 value,
        Span<ulong> result)
    {
        result.Clear();
        WideArithmetic.GetMagnitude(value, result);
    }

    private static void SetMagnitude(
        Signed832 value,
        Span<ulong> result)
    {
        result.Clear();
        WideArithmetic.GetMagnitude(value, result);
    }

    private static void Multiply3(
        ReadOnlySpan<ulong> first,
        ReadOnlySpan<ulong> second,
        ReadOnlySpan<ulong> third,
        Span<ulong> result)
    {
        Span<ulong> temporary =
            stackalloc ulong[result.Length];
        WideArithmetic.MultiplyMagnitudes(first, second, temporary);
        WideArithmetic.MultiplyMagnitudes(temporary, third, result);
    }

    private static void Multiply4(
        ReadOnlySpan<ulong> first,
        ReadOnlySpan<ulong> second,
        ReadOnlySpan<ulong> third,
        ReadOnlySpan<ulong> fourth,
        Span<ulong> result)
    {
        Span<ulong> firstProduct =
            stackalloc ulong[MaxResponseWords];
        Span<ulong> secondProduct =
            stackalloc ulong[MaxResponseWords];
        WideArithmetic.MultiplyMagnitudes(first, second, firstProduct);
        WideArithmetic.MultiplyMagnitudes(third, fourth, secondProduct);
        WideArithmetic.MultiplyMagnitudes(
            firstProduct,
            secondProduct,
            result);
    }

}
