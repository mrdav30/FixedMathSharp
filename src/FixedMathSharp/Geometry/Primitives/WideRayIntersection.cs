//=======================================================================
// WideRayIntersection.cs
//=======================================================================
// MIT License, Copyright (c) 2024–present David Oravsky (mrdav30)
// See LICENSE file in the project root for full license information.
//=======================================================================

using System;
using System.Runtime.CompilerServices;

namespace FixedMathSharp.Bounds;

/// <summary>
/// Owns exact full-domain ray/circle and ray/sphere quadratic reduction.
/// </summary>
internal static class WideRayIntersection
{
    private static readonly Signed192 RawScale = WideArithmetic.FromSignedRaw(FixedMath.ONE_L);
    private static readonly Signed192 DoubleRawScale = WideArithmetic.FromSignedRaw(FixedMath.ONE_L * 2L);

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public static Fixed64? Intersects(
        Vector2d position,
        Vector2d direction,
        FixedBoundCircle circle,
        Fixed64 maxParameter)
    {
        if (maxParameter < Fixed64.Zero)
            return null;

        return IntersectsWide(
            position,
            direction,
            circle.Center,
            WideArithmetic.FromSignedRaw(circle.Radius.m_rawValue),
            maxParameter);
    }

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public static Fixed64? Intersects(
        Vector2d position,
        Vector2d direction,
        FixedBoundCircle circle,
        Fixed64 radiusExpansion,
        Fixed64 maxParameter)
    {
        if (radiusExpansion < Fixed64.Zero)
            throw new ArgumentOutOfRangeException(nameof(radiusExpansion), "Radius expansion must be non-negative.");
        if (radiusExpansion == Fixed64.Zero)
            return Intersects(position, direction, circle, maxParameter);
        if (maxParameter < Fixed64.Zero)
            return null;

        Signed192 expandedRadius = WideArithmetic.AddSigned192(
            WideArithmetic.FromSignedRaw(circle.Radius.m_rawValue),
            WideArithmetic.FromSignedRaw(radiusExpansion.m_rawValue));
        return IntersectsWide(
            position,
            direction,
            circle.Center,
            expandedRadius,
            maxParameter);
    }

    private static Fixed64? IntersectsWide(
        Vector2d position,
        Vector2d direction,
        Vector2d center,
        Signed192 radius,
        Fixed64 maxParameter)
    {
        Signed192 directionLengthSquared = WideGeometry.GetDifferenceDotProduct2D(
            direction.X, Fixed64.Zero, direction.Y, Fixed64.Zero,
            direction.X, Fixed64.Zero, direction.Y, Fixed64.Zero);
        Signed192 projection = WideGeometry.GetDifferenceDotProduct2D(
            position.X, center.X, position.Y, center.Y,
            direction.X, Fixed64.Zero, direction.Y, Fixed64.Zero);
        Signed192 distanceSquared = WideGeometry.GetDifferenceDotProduct2D(
            position.X, center.X, position.Y, center.Y,
            position.X, center.X, position.Y, center.Y);
        return Solve(
            directionLengthSquared,
            projection,
            WideArithmetic.SubtractSigned192(distanceSquared, GetRadiusSquared(radius)),
            maxParameter);
    }

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public static Fixed64? Intersects(
        Vector3d position,
        Vector3d direction,
        FixedBoundSphere sphere,
        Fixed64 maxParameter)
    {
        if (maxParameter < Fixed64.Zero)
            return null;

        return IntersectsWide(
            position,
            direction,
            sphere.Center,
            WideArithmetic.FromSignedRaw(sphere.Radius.m_rawValue),
            maxParameter);
    }

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public static Fixed64? Intersects(
        Vector3d position,
        Vector3d direction,
        FixedBoundSphere sphere,
        Fixed64 radiusExpansion,
        Fixed64 maxParameter)
    {
        if (radiusExpansion < Fixed64.Zero)
            throw new ArgumentOutOfRangeException(nameof(radiusExpansion), "Radius expansion must be non-negative.");
        if (radiusExpansion == Fixed64.Zero)
            return Intersects(position, direction, sphere, maxParameter);
        if (maxParameter < Fixed64.Zero)
            return null;

        Signed192 expandedRadius = WideArithmetic.AddSigned192(
            WideArithmetic.FromSignedRaw(sphere.Radius.m_rawValue),
            WideArithmetic.FromSignedRaw(radiusExpansion.m_rawValue));
        return IntersectsWide(
            position,
            direction,
            sphere.Center,
            expandedRadius,
            maxParameter);
    }

    private static Fixed64? IntersectsWide(
        Vector3d position,
        Vector3d direction,
        Vector3d center,
        Signed192 radius,
        Fixed64 maxParameter)
    {
        Signed192 directionLengthSquared = WideGeometry.GetDifferenceDotProduct3D(
            direction.X, Fixed64.Zero, direction.Y, Fixed64.Zero, direction.Z, Fixed64.Zero,
            direction.X, Fixed64.Zero, direction.Y, Fixed64.Zero, direction.Z, Fixed64.Zero);
        Signed192 projection = WideGeometry.GetDifferenceDotProduct3D(
            position.X, center.X, position.Y, center.Y, position.Z, center.Z,
            direction.X, Fixed64.Zero, direction.Y, Fixed64.Zero, direction.Z, Fixed64.Zero);
        Signed192 distanceSquared = WideGeometry.GetDifferenceDotProduct3D(
            position.X, center.X, position.Y, center.Y, position.Z, center.Z,
            position.X, center.X, position.Y, center.Y, position.Z, center.Z);
        return Solve(
            directionLengthSquared,
            projection,
            WideArithmetic.SubtractSigned192(distanceSquared, GetRadiusSquared(radius)),
            maxParameter);
    }

    private static Fixed64? Solve(
        Signed192 directionLengthSquared,
        Signed192 projection,
        Signed192 constant,
        Fixed64 maxParameter)
    {
        if (constant.Sign <= 0)
            return Fixed64.Zero;
        if (directionLengthSquared.IsZero || projection.Sign >= 0 || maxParameter == Fixed64.Zero)
            return null;

        Signed320 discriminant = WideArithmetic.MultiplySubtract(
            projection,
            projection,
            directionLengthSquared,
            constant);
        if (discriminant.Sign < 0)
            return null;

        Signed192 negativeProjection = WideArithmetic.SubtractSigned192(default, projection);
        Signed192 squareRoot = WideArithmetic.GetFloorSquareRoot(discriminant, out _);
        long approximateRaw = GetFloorRatioRaw(
            constant,
            WideArithmetic.AddSigned192(negativeProjection, squareRoot));
        long maxRaw = maxParameter.m_rawValue;
        Signed320 derivativeAtMax = WideArithmetic.SubtractSigned320(
            WideArithmetic.MultiplySigned192(
                directionLengthSquared,
                WideArithmetic.FromSignedRaw(maxRaw)),
            WideArithmetic.MultiplySigned192(negativeProjection, RawScale));
        if (derivativeAtMax.Sign < 0
            && EvaluatePolynomial(
                directionLengthSquared,
                projection,
                constant,
                WideArithmetic.FromSignedRaw(maxRaw),
                RawScale).Sign > 0)
        {
            return null;
        }

        long closestRaw = GetFloorRatioRaw(negativeProjection, directionLengthSquared);
        long highRaw = closestRaw < maxRaw ? closestRaw : maxRaw;
        Signed320 highValue = EvaluatePolynomial(
            directionLengthSquared,
            projection,
            constant,
            WideArithmetic.FromSignedRaw(highRaw),
            RawScale);

        long floorRootRaw;
        bool exactRoot;
        if (highValue.Sign > 0)
        {
            floorRootRaw = highRaw;
            exactRoot = false;
        }
        else
        {
            floorRootRaw = FindFloorRoot(
                directionLengthSquared,
                projection,
                constant,
                highRaw,
                approximateRaw,
                out exactRoot);
        }

        if (exactRoot || floorRootRaw == maxRaw)
            return Fixed64.FromRaw(floorRootRaw);

        Signed192 floorRoot = WideArithmetic.FromSignedRaw(floorRootRaw);
        Signed192 midpointNumerator = WideArithmetic.AddSigned192(
            WideArithmetic.AddSigned192(floorRoot, floorRoot),
            WideArithmetic.FromSignedRaw(1L));
        Signed320 midpointValue = EvaluatePolynomial(
            directionLengthSquared,
            projection,
            constant,
            midpointNumerator,
            DoubleRawScale);
        Signed320 midpointDerivative = WideArithmetic.AddSigned320(
            WideArithmetic.MultiplySigned192(directionLengthSquared, midpointNumerator),
            WideArithmetic.MultiplySigned192(projection, DoubleRawScale));

        if (midpointValue.IsZero && midpointDerivative.Sign <= 0)
        {
            return Fixed64.FromRaw((floorRootRaw & 1L) == 0L
                ? floorRootRaw
                : floorRootRaw + 1L);
        }

        bool roundUp = midpointDerivative.Sign < 0 && midpointValue.Sign > 0;
        return Fixed64.FromRaw(roundUp ? floorRootRaw + 1L : floorRootRaw);
    }

    private static long FindFloorRoot(
        Signed192 directionLengthSquared,
        Signed192 projection,
        Signed192 constant,
        long highRaw,
        long approximateRaw,
        out bool exactRoot)
    {
        long candidateRaw = Math.Min(approximateRaw, highRaw);

        Signed320 candidateValue = EvaluatePolynomial(
            directionLengthSquared,
            projection,
            constant,
            WideArithmetic.FromSignedRaw(candidateRaw),
            RawScale);
        if (candidateValue.IsZero)
        {
            exactRoot = true;
            return candidateRaw;
        }

        if (candidateValue.Sign > 0)
        {
            // The analytic seed is never below the entry root, so its integer
            // floor cannot be below the root's integer floor. A positive value
            // proves that both floors are the same.
            exactRoot = false;
            return candidateRaw;
        }

        long lowRaw = 0L;
        highRaw = candidateRaw;
        long step = 1L;
        // Replacing the exact square root by its floor can overshoot the entry
        // root by fewer than 2^32 parameter raw units, so doubling cannot
        // overflow a signed 64-bit step before the bracket is found.
        while (true)
        {
            long nextRaw = highRaw - Math.Min(step, highRaw);
            Signed320 nextValue = EvaluatePolynomial(
                directionLengthSquared,
                projection,
                constant,
                WideArithmetic.FromSignedRaw(nextRaw),
                RawScale);
            if (nextValue.Sign > 0)
            {
                lowRaw = nextRaw;
                break;
            }

            highRaw = nextRaw;
            step <<= 1;
        }

        while (highRaw - lowRaw > 1L)
        {
            long middleRaw = lowRaw + ((highRaw - lowRaw) >> 1);
            Signed320 middleValue = EvaluatePolynomial(
                directionLengthSquared,
                projection,
                constant,
                WideArithmetic.FromSignedRaw(middleRaw),
                RawScale);
            if (middleValue.Sign > 0)
                lowRaw = middleRaw;
            else
                highRaw = middleRaw;
        }

        // A downward correction is reachable only when the discriminant root
        // was not integral. The entry root therefore cannot be an integer raw
        // parameter inside this bracket.
        exactRoot = false;
        return lowRaw;
    }

    private static long GetFloorRatioRaw(Signed192 numerator, Signed192 denominator)
    {
        Fixed64 rounded = Fixed64.GetSignedRatio(numerator, denominator);
        long raw = rounded.m_rawValue;
        if (raw <= 0L)
            return raw;

        Signed320 represented = WideArithmetic.MultiplySigned192(
            denominator,
            WideArithmetic.FromSignedRaw(raw));
        Signed320 exact = WideArithmetic.MultiplySigned192(numerator, RawScale);
        return WideArithmetic.CompareMagnitude(represented, exact) > 0 ? raw - 1L : raw;
    }

    private static Signed192 GetRadiusSquared(Signed192 radius)
    {
        _ = WideArithmetic.TryNarrowSigned192(
            WideArithmetic.MultiplySigned192(radius, radius),
            out Signed192 radiusSquared);
        return radiusSquared;
    }

    private static Signed320 EvaluatePolynomial(
        Signed192 directionLengthSquared,
        Signed192 projection,
        Signed192 constant,
        Signed192 timeNumerator,
        Signed192 timeDenominator)
    {
        _ = WideArithmetic.TryNarrowSigned192(
            WideArithmetic.MultiplySigned192(timeNumerator, timeNumerator),
            out Signed192 timeSquared);
        _ = WideArithmetic.TryNarrowSigned192(
            WideArithmetic.MultiplySigned192(timeNumerator, timeDenominator),
            out Signed192 timeProduct);
        _ = WideArithmetic.TryNarrowSigned192(
            WideArithmetic.MultiplySigned192(timeDenominator, timeDenominator),
            out Signed192 denominatorSquared);

        Signed320 first = WideArithmetic.MultiplySigned192(directionLengthSquared, timeSquared);
        Signed320 second = WideArithmetic.MultiplySigned192(
            WideArithmetic.AddSigned192(projection, projection),
            timeProduct);
        Signed320 third = WideArithmetic.MultiplySigned192(constant, denominatorSquared);
        return WideArithmetic.AddSigned320(WideArithmetic.AddSigned320(first, second), third);
    }

}
