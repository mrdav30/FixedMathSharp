//=======================================================================
// WideFiniteConeIntersection.cs
//=======================================================================
// MIT License, Copyright (c) 2024–present David Oravsky (mrdav30)
// See LICENSE file in the project root for full license information.
//=======================================================================

using System;

namespace FixedMathSharp.Geometry;

/// <summary>
/// Owns exact full-domain finite-cone containment and segment reduction.
/// </summary>
internal static class WideFiniteConeIntersection
{
    private static readonly Signed192 One = Signed192.Signed(1L);
    private static readonly Signed192 Four = Signed192.Signed(4L);
    private static readonly Signed192 AxisScaleSquared = new(0UL, 1UL, 0UL);

    #region Nested Types

    private readonly struct RationalBound
    {
        internal readonly Signed192 Numerator;
        internal readonly Signed192 Denominator;

        internal RationalBound(Signed192 numerator, Signed192 denominator)
        {
            Numerator = numerator;
            Denominator = denominator;
        }
    }

    private readonly struct ConeData
    {
        internal readonly Signed192 StartAxial;
        internal readonly Signed192 AxialVelocity;
        internal readonly Signed192 MaximumAxial;
        internal readonly Signed576 Coefficient;
        internal readonly Signed576 Projection;
        internal readonly Signed576 Constant;

        internal ConeData(
            Signed192 startAxial,
            Signed192 axialVelocity,
            Signed192 maximumAxial,
            Signed576 coefficient,
            Signed576 projection,
            Signed576 constant)
        {
            StartAxial = startAxial;
            AxialVelocity = axialVelocity;
            MaximumAxial = maximumAxial;
            Coefficient = coefficient;
            Projection = projection;
            Constant = constant;
        }

        internal ConeData NegatedPolynomial() =>
            new(
                StartAxial,
                AxialVelocity,
                MaximumAxial,
                WideArithmetic.SubtractSigned576(default, Coefficient),
                WideArithmetic.SubtractSigned576(default, Projection),
                WideArithmetic.SubtractSigned576(default, Constant));
    }

    #endregion

    internal static bool TryGetApexInterval(
        FixedSegment query,
        Vector3d apex,
        Vector3d apexToBaseDirection,
        Fixed64 height,
        Fixed64 baseRadius,
        out Fixed64 entry,
        out Fixed64 exit,
        out bool startContained,
        out bool endContainedStrict) =>
        TrySolve(
            CreateApexData(query, apex, apexToBaseDirection, height, baseRadius),
            Fixed64.One,
            out entry,
            out exit,
            out startContained,
            out endContainedStrict);

    internal static bool TrySolveUnitPolynomial(
        Signed576 coefficient,
        Signed576 projection,
        Signed576 constant,
        Fixed64 outputScale,
        out Fixed64 entry,
        out Fixed64 exit) =>
        TrySolveBoundedPolynomial(
            new ConeData(default, default, default, coefficient, projection, constant),
            new RationalBound(default, One),
            new RationalBound(One, One),
            outputScale,
            out entry,
            out exit);

    internal static bool TryGetApexDistanceInterval(
        FixedSegment query,
        Vector3d apex,
        Vector3d apexToBaseDirection,
        Fixed64 height,
        Fixed64 baseRadius,
        Fixed64 totalDistance,
        out Fixed64 entry,
        out Fixed64 exit,
        out bool startContained,
        out bool endContainedStrict) =>
        TrySolve(
            CreateApexData(query, apex, apexToBaseDirection, height, baseRadius),
            totalDistance,
            out entry,
            out exit,
            out startContained,
            out endContainedStrict);

    internal static bool TryGetCenteredInterval(
        FixedSegment query,
        Vector3d center,
        Vector3d baseToApexDirection,
        Fixed64 height,
        Fixed64 baseRadius,
        out Fixed64 entry,
        out Fixed64 exit,
        out bool startContained,
        out bool endContainedStrict) =>
        TrySolve(
            CreateCenteredData(query, center, baseToApexDirection, height, baseRadius),
            Fixed64.One,
            out entry,
            out exit,
            out startContained,
            out endContainedStrict);

    internal static bool TryGetCenteredDistanceInterval(
        FixedSegment query,
        Vector3d center,
        Vector3d baseToApexDirection,
        Fixed64 height,
        Fixed64 baseRadius,
        Fixed64 totalDistance,
        out Fixed64 entry,
        out Fixed64 exit,
        out bool startContained,
        out bool endContainedStrict) =>
        TrySolve(
            CreateCenteredData(query, center, baseToApexDirection, height, baseRadius),
            totalDistance,
            out entry,
            out exit,
            out startContained,
            out endContainedStrict);

    internal static bool ContainsPointInApexCone(
        Vector3d point,
        Vector3d apex,
        Vector3d apexToBaseDirection,
        Fixed64 height,
        Fixed64 baseRadius,
        bool strict)
    {
        Signed192 heightRaw = Signed192.Signed(height.m_rawValue);
        Signed192 axisLengthSquared = GetDot(
            apexToBaseDirection,
            Vector3d.Zero,
            apexToBaseDirection,
            Vector3d.Zero);
        bool exactUnitAxis = IsExactUnitAxis(axisLengthSquared);
        Signed192 axisProjection = GetDot(
            point,
            apex,
            apexToBaseDirection,
            Vector3d.Zero);
        Signed192 axial = exactUnitAxis ? axisProjection : GetScaledRaw(axisProjection);
        return ContainsPoint(
            point,
            apex,
            heightRaw,
            axisLengthSquared,
            axisProjection,
            axial,
            exactUnitAxis ? GetScaledRaw(heightRaw) : GetAxisHeightProduct(axisLengthSquared, heightRaw),
            One,
            exactUnitAxis,
            baseRadius,
            strict);
    }

    internal static bool ContainsPointInCenteredCone(
        Vector3d point,
        Vector3d center,
        Vector3d baseToApexDirection,
        Fixed64 height,
        Fixed64 baseRadius,
        bool strict)
    {
        Signed192 heightRaw = Signed192.Signed(height.m_rawValue);
        Signed192 axisLengthSquared = GetDot(
            baseToApexDirection,
            Vector3d.Zero,
            baseToApexDirection,
            Vector3d.Zero);
        bool exactUnitAxis = IsExactUnitAxis(axisLengthSquared);
        Signed192 axisProjection = GetDot(point, center, baseToApexDirection, Vector3d.Zero);
        Signed192 maximumAxial = exactUnitAxis
            ? GetScaledRaw(heightRaw)
            : GetAxisHeightProduct(axisLengthSquared, heightRaw);
        Signed192 axial = exactUnitAxis
            ? WideArithmetic.SubtractSigned192(GetHalfScaledRaw(heightRaw), axisProjection)
            : WideArithmetic.SubtractSigned192(maximumAxial, WideArithmetic.Double(GetScaledRaw(axisProjection)));
        return ContainsPoint(
            point,
            center,
            heightRaw,
            axisLengthSquared,
            axisProjection,
            axial,
            exactUnitAxis ? maximumAxial : WideArithmetic.Double(maximumAxial),
            exactUnitAxis ? One : Four,
            exactUnitAxis,
            baseRadius,
            strict);
    }

    private static bool ContainsPoint(
        Vector3d point,
        Vector3d origin,
        Signed192 heightRaw,
        Signed192 axisLengthSquared,
        Signed192 axisProjection,
        Signed192 axial,
        Signed192 maximumAxial,
        Signed192 radialScale,
        bool exactUnitAxis,
        Fixed64 baseRadius,
        bool strict)
    {
        Signed192 distanceSquared = GetDot(point, origin, point, origin);
        Signed320 radial = GetRadialTerm(distanceSquared, axisProjection, axisLengthSquared);
        Signed320 heightSquared = WideArithmetic.MultiplySigned192(heightRaw, heightRaw);
        Signed192 radiusRaw = Signed192.Signed(baseRadius.m_rawValue);
        Signed320 radiusSquared = WideArithmetic.MultiplySigned192(radiusRaw, radiusRaw);
        Signed576 polynomial = SubtractConeTerms(
            heightSquared,
            radial,
            axisLengthSquared,
            radialScale,
            exactUnitAxis,
            radiusSquared,
            WideArithmetic.MultiplySigned192(axial, axial));
        return IsContained(axial, polynomial, maximumAxial, strict);
    }

    private static bool TrySolve(
        ConeData data,
        Fixed64 outputScale,
        out Fixed64 entry,
        out Fixed64 exit,
        out bool startContained,
        out bool endContainedStrict)
    {
        startContained = IsContained(
            data.StartAxial,
            data.Constant,
            data.MaximumAxial,
            strict: false);
        Signed192 endAxial = WideArithmetic.AddSigned192(data.StartAxial, data.AxialVelocity);
        endContainedStrict = false;

        if (!TryGetAxialInterval(data, out RationalBound lower, out RationalBound upper))
        {
            entry = default;
            exit = default;
            return false;
        }

        if (endAxial.Sign > 0
            && WideArithmetic.SubtractSigned192(endAxial, data.MaximumAxial).Sign < 0)
        {
            endContainedStrict = Evaluate(data, One, One).Sign < 0;
        }

        return TrySolveBoundedPolynomial(data, lower, upper, outputScale, out entry, out exit);
    }

    private static bool TrySolveBoundedPolynomial(
        ConeData data,
        RationalBound lower,
        RationalBound upper,
        Fixed64 outputScale,
        out Fixed64 entry,
        out Fixed64 exit) =>
        TrySolveBoundedUnitPolynomial(
            Signed832.ExtendValue(data.Coefficient),
            Signed832.ExtendValue(data.Projection),
            Signed832.ExtendValue(data.Constant),
            Signed320.ExtendValue(lower.Numerator),
            Signed320.ExtendValue(lower.Denominator),
            Signed320.ExtendValue(upper.Numerator),
            Signed320.ExtendValue(upper.Denominator),
            outputScale,
            out entry,
            out exit);

    /// <summary>
    /// Reduces an axially clipped conic on [0, 1]. Rigid triangle coefficients
    /// have fewer than 660 magnitude bits and clip ratios at most 200; midpoint
    /// root evaluations fit Signed832, while clipped evaluations use fixed
    /// transient magnitude storage. The admitted cone interval is connected.
    /// </summary>
    internal static bool TrySolveBoundedUnitPolynomial(
        Signed832 coefficient,
        Signed832 projection,
        Signed832 constant,
        Signed320 lowerNumerator,
        Signed320 lowerDenominator,
        Signed320 upperNumerator,
        Signed320 upperDenominator,
        Fixed64 outputScale,
        out Fixed64 entry,
        out Fixed64 exit)
    {
        bool compact = TryGetCompactPolynomial(
            coefficient, projection, constant,
            lowerNumerator, lowerDenominator, upperNumerator, upperDenominator,
            out ConeData data);
        bool lowerContained = (compact
            ? Evaluate(data, Signed192.NarrowValue(lowerNumerator), Signed192.NarrowValue(lowerDenominator)).Sign
            : GetPolynomialSignAtRationalParameter(coefficient, projection, constant, lowerNumerator, lowerDenominator)) <= 0;
        bool upperContained = (compact
            ? Evaluate(data, Signed192.NarrowValue(upperNumerator), Signed192.NarrowValue(upperDenominator)).Sign
            : GetPolynomialSignAtRationalParameter(coefficient, projection, constant, upperNumerator, upperDenominator)) <= 0;

        if (lowerContained && upperContained)
        {
            entry = Round(lowerNumerator, lowerDenominator, outputScale);
            exit = Round(upperNumerator, upperDenominator, outputScale);
            return true;
        }

        if (coefficient.IsZero)
        {
            if (!lowerContained && !upperContained)
            {
                entry = default;
                exit = default;
                return false;
            }

            Fixed64 root = compact
                ? RoundLinearRoot(data, outputScale)
                : RoundLinearRoot(projection, constant, outputScale);
            entry = lowerContained ? Round(lowerNumerator, lowerDenominator, outputScale) : root;
            exit = upperContained ? Round(upperNumerator, upperDenominator, outputScale) : root;
            return true;
        }

        bool opensUp = coefficient.Sign > 0;
        if (!opensUp && !lowerContained && !upperContained)
        {
            entry = default;
            exit = default;
            return false;
        }

        if (opensUp
            && ((!lowerContained && (compact
                    ? EvaluateDerivative(data, new RationalBound(Signed192.NarrowValue(lowerNumerator), Signed192.NarrowValue(lowerDenominator))).Sign
                    : GetDerivativeSign(coefficient, projection, lowerNumerator, lowerDenominator)) >= 0)
                || (!upperContained && (compact
                    ? EvaluateDerivative(data, new RationalBound(Signed192.NarrowValue(upperNumerator), Signed192.NarrowValue(upperDenominator))).Sign
                    : GetDerivativeSign(coefficient, projection, upperNumerator, upperDenominator)) <= 0)))
        {
            entry = default;
            exit = default;
            return false;
        }

        if (!opensUp)
        {
            coefficient = WideArithmetic.SubtractSigned832(default, coefficient);
            projection = WideArithmetic.SubtractSigned832(default, projection);
            constant = WideArithmetic.SubtractSigned832(default, constant);
        }
        ConeData normalized = opensUp ? data : data.NegatedPolynomial();
        Signed832 discriminant = compact
            ? WideArithmetic.SubtractSigned832(
                WideArithmetic.MultiplySigned576ToSigned832(normalized.Projection, normalized.Projection),
                WideArithmetic.MultiplySigned576ToSigned832(normalized.Coefficient, normalized.Constant))
            : default;
        int discriminantSign = compact ? discriminant.Sign
            : constant.Sign < 0 ? 1
            : WideArithmetic.CompareNonNegativeProducts(projection, projection, coefficient, constant);
        if (discriminantSign < 0)
        {
            entry = default;
            exit = default;
            return false;
        }

        Signed192 outputScaleRaw = Signed192.Signed(outputScale.m_rawValue);
        Signed192 outputScaleSquared = SquareRaw(outputScale.m_rawValue);
        Signed576 scaledSquareRoot = compact
            ? WideArithmetic.GetFloorSquareRootOfProduct(discriminant, outputScaleSquared)
            : default;

        if (opensUp)
        {
            entry = lowerContained
                ? Round(lowerNumerator, lowerDenominator, outputScale)
                : compact ? RoundLowerRoot(normalized, scaledSquareRoot, outputScaleRaw)
                : RoundRoot(coefficient, projection, constant, outputScale, upperRoot: false);
            exit = upperContained
                ? Round(upperNumerator, upperDenominator, outputScale)
                : compact ? RoundUpperRoot(normalized, scaledSquareRoot, outputScaleRaw)
                : RoundRoot(coefficient, projection, constant, outputScale, upperRoot: true);
            return true;
        }

        entry = lowerContained
            ? Round(lowerNumerator, lowerDenominator, outputScale)
            : compact ? RoundUpperRoot(normalized, scaledSquareRoot, outputScaleRaw)
            : RoundRoot(coefficient, projection, constant, outputScale, upperRoot: true);
        exit = upperContained
            ? Round(upperNumerator, upperDenominator, outputScale)
            : compact ? RoundLowerRoot(normalized, scaledSquareRoot, outputScaleRaw)
            : RoundRoot(coefficient, projection, constant, outputScale, upperRoot: false);
        return true;
    }

    private static bool TryGetCompactPolynomial(
        Signed832 coefficient,
        Signed832 projection,
        Signed832 constant,
        Signed320 lowerNumerator,
        Signed320 lowerDenominator,
        Signed320 upperNumerator,
        Signed320 upperDenominator,
        out ConeData data)
    {
        int coefficientBits = GetMagnitudeBitLength(coefficient);
        int projectionBits = GetMagnitudeBitLength(projection);
        int constantBits = GetMagnitudeBitLength(constant);
        int coefficientMaximum = Math.Max(coefficientBits, Math.Max(projectionBits, constantBits));
        int boundMaximum = Math.Max(
            Math.Max(GetMagnitudeBitLength(lowerNumerator), GetMagnitudeBitLength(lowerDenominator)),
            Math.Max(GetMagnitudeBitLength(upperNumerator), GetMagnitudeBitLength(upperDenominator)));
        // Rational squares fit Signed320, evaluated sums and the discriminant
        // fit Signed832, and the linear numerator retains its 63-bit scale.
        // The discriminant proof also bounds the scaled quadratic-root
        // numerator below 480 bits, inside the existing Signed576 owner.
        if (coefficientMaximum > 575 || boundMaximum > 159
            || coefficientMaximum + 2 * boundMaximum > 828
            || 2 * projectionBits > 830 || coefficientBits + constantBits > 830
            || (coefficient.IsZero && constantBits > 512))
        {
            data = default;
            return false;
        }

        data = new ConeData(default, default, default,
            NarrowCoefficient(coefficient), NarrowCoefficient(projection), NarrowCoefficient(constant));
        return true;
    }

    private static Signed576 NarrowCoefficient(Signed832 value) =>
        new(value.Word8, value.Word7, value.Word6, value.Word5, value.Word4,
            value.Word3, value.Word2, value.Word1, value.Word0);

    private static int GetMagnitudeBitLength(Signed832 value)
    {
        Span<ulong> magnitude = stackalloc ulong[13];
        WideArithmetic.GetMagnitude(value, magnitude);
        return WideArithmetic.GetMagnitudeBitLength(magnitude);
    }

    private static int GetMagnitudeBitLength(Signed320 value)
    {
        Span<ulong> magnitude = stackalloc ulong[5];
        WideArithmetic.GetMagnitude(value, out magnitude[4], out magnitude[3],
            out magnitude[2], out magnitude[1], out magnitude[0]);
        return WideArithmetic.GetMagnitudeBitLength(magnitude);
    }

    /// <summary>
    /// Returns the exact sign of A*n*n + 2*B*n*d + C*d*d without narrowing
    /// rigid-frame clip ratios or their homogenized products.
    /// </summary>
    internal static int GetPolynomialSignAtRationalParameter(
        Signed832 coefficient,
        Signed832 projection,
        Signed832 constant,
        Signed320 numerator,
        Signed320 denominator) =>
        EvaluateWidePolynomialSign(coefficient, projection, constant, numerator, denominator, derivative: false);

    private static int GetDerivativeSign(
        Signed832 coefficient,
        Signed832 projection,
        Signed320 numerator,
        Signed320 denominator) =>
        EvaluateWidePolynomialSign(coefficient, projection, default, numerator, denominator, derivative: true);

    private static int EvaluateWidePolynomialSign(
        Signed832 coefficient,
        Signed832 projection,
        Signed832 constant,
        Signed320 numerator,
        Signed320 denominator,
        bool derivative)
    {
        // Even the full carrier products occupy fewer than 1,474 bits including
        // the doubled middle term and addition carries; 24 words retain them.
        // Current rigid query bounds are smaller: 660 + 2*200 + 2 < 1,063 bits.
        Span<ulong> n = stackalloc ulong[5];
        Span<ulong> d = stackalloc ulong[5];
        Span<ulong> factor = stackalloc ulong[10];
        Span<ulong> coefficientMagnitude = stackalloc ulong[13];
        Span<ulong> product = stackalloc ulong[24];
        Span<ulong> sum = stackalloc ulong[24];
        WideArithmetic.GetMagnitude(numerator, out n[4], out n[3], out n[2], out n[1], out n[0]);
        WideArithmetic.GetMagnitude(denominator, out d[4], out d[3], out d[2], out d[1], out d[0]);
        sum.Clear();
        int sign = 0;
        for (int term = 0; term < (derivative ? 2 : 3); term++)
        {
            Signed832 value = term == 0 ? coefficient : term == 1 ? projection : constant;
            WideArithmetic.GetMagnitude(value, coefficientMagnitude);
            int factorSign;
            if (derivative)
            {
                factor.Clear();
                (term == 0 ? n : d).CopyTo(factor);
                factorSign = term == 0 ? numerator.Sign : denominator.Sign;
            }
            else
            {
                WideArithmetic.MultiplyMagnitudes(term == 2 ? d : n, term == 0 ? n : d, factor);
                factorSign = term == 0 ? numerator.Sign * numerator.Sign
                    : term == 1 ? numerator.Sign * denominator.Sign : denominator.Sign * denominator.Sign;
            }
            WideArithmetic.MultiplyMagnitudes(coefficientMagnitude, factor, product);
            WideArithmetic.AddShiftedSignedMagnitude(product, value.Sign * factorSign,
                !derivative && term == 1 ? 1 : 0, sum, ref sign);
        }
        return sign;
    }

    private static Fixed64 RoundRoot(
        Signed832 coefficient,
        Signed832 projection,
        Signed832 constant,
        Fixed64 outputScale,
        bool upperRoot)
    {
        ulong low = 0UL;
        ulong high = (ulong)outputScale.m_rawValue;
        Signed192 denominator = new(0UL, 0UL, high << 1);
        while (low < high)
        {
            ulong candidate = low + ((high - low) >> 1);
            Signed192 numerator = new(0UL, 0UL, (candidate << 1) | 1UL);
            Signed832 first = WideArithmetic.MultiplySigned832(coefficient, numerator);
            Signed832 second = WideArithmetic.MultiplySigned832(projection, denominator);
            int derivativeSign = WideArithmetic.AddSigned832(first, second).Sign;
            Signed832 mixed = WideArithmetic.MultiplySigned832(second, numerator);
            int polynomialSign = WideArithmetic.AddSigned832(
                WideArithmetic.AddSigned832(
                    WideArithmetic.MultiplySigned832(first, numerator),
                    WideArithmetic.AddSigned832(mixed, mixed)),
                WideArithmetic.MultiplySigned832(
                    WideArithmetic.MultiplySigned832(constant, denominator), denominator)).Sign;
            bool exactRoot = polynomialSign == 0 && (upperRoot ? derivativeSign >= 0 : derivativeSign <= 0);
            bool precedes = exactRoot ? (candidate & 1UL) != 0UL
                : upperRoot ? derivativeSign < 0 || polynomialSign < 0
                : derivativeSign < 0 && polynomialSign > 0;
            if (precedes)
                low = candidate + 1UL;
            else
                high = candidate;
        }
        return Fixed64.FromRaw((long)low);
    }

    private static Fixed64 RoundLinearRoot(Signed832 projection, Signed832 constant, Fixed64 outputScale)
    {
        Signed832 numerator = WideArithmetic.MultiplySigned832(
            WideArithmetic.SubtractSigned832(default, constant), Signed192.Raw(outputScale));
        Signed832 denominator = WideArithmetic.AddSigned832(projection, projection);
        _ = Fixed64.TryGetSignedRawRatio(numerator, denominator, 0, out Fixed64 root);
        return root;
    }

    private static Fixed64 Round(Signed320 numerator, Signed320 denominator, Fixed64 outputScale)
    {
        _ = Fixed64.TryGetSignedRawRatio(
            WideArithmetic.MultiplySigned320(numerator, Signed192.Raw(outputScale)),
            Signed576.ExtendValue(denominator), out Fixed64 result);
        return result;
    }

    private static Fixed64 RoundLinearRoot(ConeData data, Fixed64 outputScale)
    {
        Signed192 outputScaleRaw = Signed192.Signed(outputScale.m_rawValue);
        Signed576 numerator = WideArithmetic.MultiplySigned576(
            WideArithmetic.SubtractSigned576(default, data.Constant),
            outputScaleRaw);
        Signed576 denominator = WideArithmetic.AddSigned576(data.Projection, data.Projection);
        _ = Fixed64.TryGetSignedRawRatio(numerator, denominator, out Fixed64 root);
        return root;
    }

    private static Fixed64 RoundLowerRoot(
        ConeData normalized,
        Signed576 scaledSquareRoot,
        Signed192 outputScaleRaw)
    {
        Signed576 negativeScaledProjection = WideArithmetic.SubtractSigned576(
            default,
            WideArithmetic.MultiplySigned576(normalized.Projection, outputScaleRaw));
        Signed576 numerator = WideArithmetic.SubtractSigned576(
            negativeScaledProjection,
            scaledSquareRoot);
        _ = Fixed64.TryGetSignedRawRatio(numerator, normalized.Coefficient, out Fixed64 candidate);

        long upperRaw = candidate.m_rawValue;
        Signed192 upper = Signed192.Signed(upperRaw);
        Signed192 midpoint = WideArithmetic.SubtractSigned192(
            WideArithmetic.AddSigned192(upper, upper),
            One);
        Signed192 doubleScale = WideArithmetic.AddSigned192(outputScaleRaw, outputScaleRaw);
        Signed832 value = Evaluate(normalized, midpoint, doubleScale);
        if (value.IsZero)
            return candidate;

        return Fixed64.FromRaw(value.Sign > 0 ? upperRaw : upperRaw - 1L);
    }

    private static Fixed64 RoundUpperRoot(
        ConeData normalized,
        Signed576 scaledSquareRoot,
        Signed192 outputScaleRaw)
    {
        Signed576 negativeScaledProjection = WideArithmetic.SubtractSigned576(
            default,
            WideArithmetic.MultiplySigned576(normalized.Projection, outputScaleRaw));
        Signed576 numerator = WideArithmetic.AddSigned576(
            negativeScaledProjection,
            scaledSquareRoot);
        _ = Fixed64.TryGetSignedRawRatio(numerator, normalized.Coefficient, out Fixed64 candidate);

        long lowerRaw = candidate.m_rawValue;
        Signed192 lower = Signed192.Signed(lowerRaw);
        Signed192 midpoint = WideArithmetic.AddSigned192(
            WideArithmetic.AddSigned192(lower, lower),
            One);
        Signed192 doubleScale = WideArithmetic.AddSigned192(outputScaleRaw, outputScaleRaw);
        Signed832 value = Evaluate(normalized, midpoint, doubleScale);
        if (value.IsZero)
            return candidate;

        return Fixed64.FromRaw(value.Sign <= 0 ? lowerRaw + 1L : lowerRaw);
    }

    private static bool TryGetAxialInterval(
        ConeData data,
        out RationalBound lower,
        out RationalBound upper)
    {
        RationalBound zero = new(default, One);
        RationalBound one = new(One, One);
        lower = zero;
        upper = one;

        if (data.AxialVelocity.IsZero)
        {
            return data.StartAxial.Sign >= 0
                && WideArithmetic.SubtractSigned192(data.StartAxial, data.MaximumAxial).Sign <= 0;
        }

        RationalBound first = Normalize(
            WideArithmetic.SubtractSigned192(default, data.StartAxial),
            data.AxialVelocity);
        RationalBound second = Normalize(
            WideArithmetic.SubtractSigned192(data.MaximumAxial, data.StartAxial),
            data.AxialVelocity);
        if (Compare(first, second) > 0)
            (first, second) = (second, first);

        if (Compare(second, zero) < 0 || Compare(first, one) > 0)
            return false;
        if (Compare(first, zero) > 0)
            lower = first;
        if (Compare(second, one) < 0)
            upper = second;
        return true;
    }

    private static ConeData CreateApexData(
        FixedSegment query,
        Vector3d apex,
        Vector3d apexToBaseDirection,
        Fixed64 height,
        Fixed64 baseRadius)
    {
        Signed192 heightRaw = Signed192.Signed(height.m_rawValue);
        return CreateData(
            query,
            apex,
            apexToBaseDirection,
            heightRaw,
            centered: false,
            baseRadius);
    }

    private static ConeData CreateCenteredData(
        FixedSegment query,
        Vector3d center,
        Vector3d baseToApexDirection,
        Fixed64 height,
        Fixed64 baseRadius)
    {
        Signed192 heightRaw = Signed192.Signed(height.m_rawValue);
        return CreateData(
            query,
            center,
            baseToApexDirection,
            heightRaw,
            centered: true,
            baseRadius);
    }

    private static ConeData CreateData(
        FixedSegment query,
        Vector3d origin,
        Vector3d axisDirection,
        Signed192 heightRaw,
        bool centered,
        Fixed64 baseRadius)
    {
        Signed192 startDistanceSquared = GetDot(query.Start, origin, query.Start, origin);
        Signed192 startDirectionProjection = GetDot(query.Start, origin, query.End, query.Start);
        Signed192 directionLengthSquared = GetDot(query.End, query.Start, query.End, query.Start);
        Signed192 axisLengthSquared = GetDot(axisDirection, Vector3d.Zero, axisDirection, Vector3d.Zero);
        bool exactUnitAxis = IsExactUnitAxis(axisLengthSquared);
        Signed192 startAxisProjection = GetDot(query.Start, origin, axisDirection, Vector3d.Zero);
        Signed192 directionAxisProjection = GetDot(query.End, query.Start, axisDirection, Vector3d.Zero);
        Signed192 maximumAxisHeight = exactUnitAxis
            ? GetScaledRaw(heightRaw)
            : GetAxisHeightProduct(axisLengthSquared, heightRaw);
        Signed192 startAxial;
        Signed192 axialVelocity;
        Signed192 maximumAxial;
        Signed192 radialScale;
        if (exactUnitAxis)
        {
            startAxial = centered
                ? WideArithmetic.SubtractSigned192(GetHalfScaledRaw(heightRaw), startAxisProjection)
                : startAxisProjection;
            axialVelocity = centered
                ? WideArithmetic.SubtractSigned192(default, directionAxisProjection)
                : directionAxisProjection;
            maximumAxial = maximumAxisHeight;
            radialScale = One;
        }
        else
        {
            Signed192 scaledStartAxisProjection = GetScaledRaw(startAxisProjection);
            Signed192 scaledDirectionAxisProjection = GetScaledRaw(directionAxisProjection);
            startAxial = centered
                ? WideArithmetic.SubtractSigned192(maximumAxisHeight, WideArithmetic.Double(scaledStartAxisProjection))
                : scaledStartAxisProjection;
            axialVelocity = centered
                ? WideArithmetic.SubtractSigned192(default, WideArithmetic.Double(scaledDirectionAxisProjection))
                : scaledDirectionAxisProjection;
            maximumAxial = centered
                ? WideArithmetic.Double(maximumAxisHeight)
                : maximumAxisHeight;
            radialScale = centered ? Four : One;
        }

        Signed320 radialCoefficient = GetRadialTerm(
            directionLengthSquared,
            directionAxisProjection,
            axisLengthSquared);
        Signed320 radialProjection = GetRadialTerm(
            startDirectionProjection,
            startAxisProjection,
            directionAxisProjection,
            axisLengthSquared);
        Signed320 radialConstant = GetRadialTerm(
            startDistanceSquared,
            startAxisProjection,
            axisLengthSquared);
        Signed320 heightSquared = WideArithmetic.MultiplySigned192(heightRaw, heightRaw);
        Signed192 radiusRaw = Signed192.Signed(baseRadius.m_rawValue);
        Signed320 radiusSquared = WideArithmetic.MultiplySigned192(radiusRaw, radiusRaw);
        Signed320 axialVelocitySquared = WideArithmetic.MultiplySigned192(axialVelocity, axialVelocity);
        Signed320 axialProduct = WideArithmetic.MultiplySigned192(startAxial, axialVelocity);
        Signed320 startAxialSquared = WideArithmetic.MultiplySigned192(startAxial, startAxial);

        return new ConeData(
            startAxial,
            axialVelocity,
            maximumAxial,
            SubtractConeTerms(
                heightSquared, radialCoefficient, axisLengthSquared, radialScale,
                exactUnitAxis,
                radiusSquared, axialVelocitySquared),
            SubtractConeTerms(
                heightSquared, radialProjection, axisLengthSquared, radialScale,
                exactUnitAxis,
                radiusSquared, axialProduct),
            SubtractConeTerms(
                heightSquared, radialConstant, axisLengthSquared, radialScale,
                exactUnitAxis,
                radiusSquared, startAxialSquared));
    }

    private static Signed320 GetRadialTerm(
        Signed192 squaredLength,
        Signed192 axisProjection,
        Signed192 axisLengthSquared) =>
        WideArithmetic.SubtractSigned320(
            WideArithmetic.MultiplySigned192(squaredLength, axisLengthSquared),
            WideArithmetic.MultiplySigned192(axisProjection, axisProjection));

    private static Signed320 GetRadialTerm(
        Signed192 dot,
        Signed192 startAxisProjection,
        Signed192 directionAxisProjection,
        Signed192 axisLengthSquared) =>
        WideArithmetic.SubtractSigned320(
            WideArithmetic.MultiplySigned192(dot, axisLengthSquared),
            WideArithmetic.MultiplySigned192(startAxisProjection, directionAxisProjection));

    private static Signed576 SubtractConeTerms(
        Signed320 heightSquared,
        Signed320 radialTerm,
        Signed192 axisLengthSquared,
        Signed192 radialScale,
        bool exactUnitAxis,
        Signed320 radiusSquared,
        Signed320 axialTerm)
    {
        if (exactUnitAxis)
        {
            return WideArithmetic.SubtractSigned576(
                WideArithmetic.MultiplySigned320(heightSquared, radialTerm),
                WideArithmetic.MultiplySigned320(radiusSquared, axialTerm));
        }

        Signed576 radial = WideArithmetic.MultiplySigned576(
            WideArithmetic.MultiplySigned576(
                WideArithmetic.MultiplySigned320(heightSquared, radialTerm),
                axisLengthSquared),
            radialScale);
        Signed576 axial = WideArithmetic.MultiplySigned320(radiusSquared, axialTerm);
        return WideArithmetic.SubtractSigned576(radial, axial);
    }

    private static Signed832 Evaluate(ConeData data, Signed192 numerator, Signed192 denominator)
    {
        // Rigid-frame coefficients have fewer than 390 magnitude bits after
        // their common quaternion-denominator factor is removed. Clipped
        // rational bounds have at most 132-bit magnitudes, so every homogenized
        // term and their sum fit below 656 bits. Signed832 also retains the
        // wider legacy conic intermediates without the former eleven-word
        // truncation.
        Signed320 numeratorSquared = WideArithmetic.MultiplySigned192(numerator, numerator);
        Signed320 numeratorDenominator = WideArithmetic.MultiplySigned192(numerator, denominator);
        Signed320 denominatorSquared = WideArithmetic.MultiplySigned192(denominator, denominator);
        Signed832 first = WideArithmetic.MultiplySigned576ToSigned832(data.Coefficient, numeratorSquared);
        Signed832 second = WideArithmetic.MultiplySigned576ToSigned832(data.Projection, numeratorDenominator);
        Signed832 third = WideArithmetic.MultiplySigned576ToSigned832(data.Constant, denominatorSquared);
        return WideArithmetic.AddSigned832(
            WideArithmetic.AddSigned832(first, WideArithmetic.AddSigned832(second, second)),
            third);
    }

    private static Signed832 EvaluateDerivative(ConeData data, RationalBound bound) =>
        WideArithmetic.AddSigned832(
            WideArithmetic.MultiplySigned576ToSigned832(
                data.Coefficient,
                Signed320.ExtendValue(bound.Numerator)),
            WideArithmetic.MultiplySigned576ToSigned832(
                data.Projection,
                Signed320.ExtendValue(bound.Denominator)));

    private static bool IsContained(
        Signed192 axial,
        Signed576 polynomial,
        Signed192 maximumAxial,
        bool strict) =>
        IsContained(axial, polynomial.Sign, maximumAxial, strict);

    private static bool IsContained(
        Signed192 axial,
        int polynomialSign,
        Signed192 maximumAxial,
        bool strict)
    {
        int maximumSign = WideArithmetic.SubtractSigned192(axial, maximumAxial).Sign;
        return strict
            ? axial.Sign > 0 && maximumSign < 0 && polynomialSign < 0
            : axial.Sign >= 0 && maximumSign <= 0 && polynomialSign <= 0;
    }

    private static Signed192 GetScaledRaw(Signed192 value) =>
        new((value.High << FixedMath.SHIFT_AMOUNT_I) | (value.Middle >> FixedMath.SHIFT_AMOUNT_I),
            (value.Middle << FixedMath.SHIFT_AMOUNT_I) | (value.Low >> FixedMath.SHIFT_AMOUNT_I),
            value.Low << FixedMath.SHIFT_AMOUNT_I);

    private static Signed192 GetAxisHeightProduct(
        Signed192 axisLengthSquared,
        Signed192 heightRaw)
    {
        Signed320 product = WideArithmetic.MultiplySigned192(axisLengthSquared, heightRaw);
        // IsNormalized bounds the positive axis square near 2^64; multiplying
        // by a positive Fixed64 raw height therefore occupies fewer than 129 bits.
        return new Signed192(product.Word2, product.Word1, product.Word0);
    }

    private static Signed192 GetHalfScaledRaw(Signed192 value)
    {
        Signed192 scaled = GetScaledRaw(value);
        ulong high = scaled.High;
        ulong middle = scaled.Middle;
        ulong low = scaled.Low;
        WideArithmetic.ShiftRightOne(ref high, ref middle, ref low);
        return new Signed192(high, middle, low);
    }

    private static bool IsExactUnitAxis(Signed192 axisLengthSquared) =>
        axisLengthSquared.High == AxisScaleSquared.High
        && axisLengthSquared.Middle == AxisScaleSquared.Middle
        && axisLengthSquared.Low == AxisScaleSquared.Low;

    private static Signed192 SquareRaw(long raw)
    {
        ulong magnitude = (ulong)raw;
        Fixed64.Multiply64To128(magnitude, magnitude, out ulong high, out ulong low);
        return new Signed192(0UL, high, low);
    }

    private static RationalBound Normalize(Signed192 numerator, Signed192 denominator)
    {
        if (denominator.Sign >= 0)
            return new RationalBound(numerator, denominator);

        return new RationalBound(
            WideArithmetic.SubtractSigned192(default, numerator),
            WideArithmetic.SubtractSigned192(default, denominator));
    }

    private static int Compare(RationalBound left, RationalBound right) =>
        WideArithmetic.MultiplySubtract(
            left.Numerator,
            right.Denominator,
            right.Numerator,
            left.Denominator).Sign;

    private static Signed192 GetDot(
        Vector3d leftEnd,
        Vector3d leftStart,
        Vector3d rightEnd,
        Vector3d rightStart) =>
        WideGeometry.GetDifferenceDotProduct3D(
            leftEnd.X, leftStart.X, leftEnd.Y, leftStart.Y, leftEnd.Z, leftStart.Z,
            rightEnd.X, rightStart.X, rightEnd.Y, rightStart.Y, rightEnd.Z, rightStart.Z);

    internal static int EvaluateBoundedUnitPolynomialSign(
            Signed576 coefficient,
            Signed576 projection,
            Signed576 constant,
            Signed192 numerator,
            Signed192 denominator) =>
            Evaluate(
                new ConeData(
                    default,
                    default,
                    default,
                    coefficient,
                    projection,
                    constant),
                numerator,
                denominator).Sign;

    internal static int EvaluateBoundedUnitPolynomialDerivativeSign(
        Signed576 coefficient,
        Signed576 projection,
        Signed192 numerator,
        Signed192 denominator) =>
        EvaluateDerivative(
            new ConeData(
                default,
                default,
                default,
                coefficient,
                projection,
                default),
            new RationalBound(numerator, denominator)).Sign;

    internal static bool TrySolveBoundedUnitPolynomial(
        Signed576 coefficient,
        Signed576 projection,
        Signed576 constant,
        Signed192 lowerNumerator,
        Signed192 lowerDenominator,
        Signed192 upperNumerator,
        Signed192 upperDenominator,
        Fixed64 outputScale,
        out Fixed64 entry,
        out Fixed64 exit) =>
        TrySolveBoundedPolynomial(
            new ConeData(
                default,
                default,
                default,
                coefficient,
                projection,
                constant),
            new RationalBound(lowerNumerator, lowerDenominator),
            new RationalBound(upperNumerator, upperDenominator),
            outputScale,
            out entry,
            out exit);
}
