//=======================================================================
// WideFiniteConeIntersection.cs
//=======================================================================
// MIT License, Copyright (c) 2024–present David Oravsky (mrdav30)
// See LICENSE file in the project root for full license information.
//=======================================================================

namespace FixedMathSharp.Geometry;

/// <summary>
/// Owns exact full-domain finite-cone containment and segment reduction.
/// </summary>
internal static partial class WideFiniteConeIntersection
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

    internal static int GetPolynomialSignAtScaledParameter(
        Signed576 coefficient,
        Signed576 projection,
        Signed576 constant,
        Fixed64 parameter,
        Fixed64 scale) =>
        Evaluate(
            new ConeData(default, default, default, coefficient, projection, constant),
            Signed192.Signed(parameter.m_rawValue),
            Signed192.Signed(scale.m_rawValue)).Sign;

    internal static int GetPolynomialSignAtRationalParameter(
        Signed576 coefficient,
        Signed576 projection,
        Signed576 constant,
        Signed192 numerator,
        Signed192 denominator) =>
        Evaluate(
            new ConeData(default, default, default, coefficient, projection, constant),
            numerator,
            denominator).Sign;

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
        out Fixed64 exit)
    {
        Signed832 lowerValue = Evaluate(data, lower.Numerator, lower.Denominator);
        Signed832 upperValue = Evaluate(data, upper.Numerator, upper.Denominator);
        bool lowerContained = lowerValue.Sign <= 0;
        bool upperContained = upperValue.Sign <= 0;

        if (data.Coefficient.IsZero)
        {
            if (data.Projection.IsZero)
            {
                if (!lowerContained)
                {
                    entry = default;
                    exit = default;
                    return false;
                }

                entry = Round(lower, outputScale);
                exit = Round(upper, outputScale);
                return true;
            }

            if (lowerContained && upperContained)
            {
                entry = Round(lower, outputScale);
                exit = Round(upper, outputScale);
                return true;
            }
            if (!lowerContained && !upperContained)
            {
                entry = default;
                exit = default;
                return false;
            }

            Fixed64 root = RoundLinearRoot(data, outputScale);
            entry = lowerContained ? Round(lower, outputScale) : root;
            exit = upperContained ? Round(upper, outputScale) : root;
            return true;
        }

        if (lowerContained && upperContained)
        {
            entry = Round(lower, outputScale);
            exit = Round(upper, outputScale);
            return true;
        }

        bool opensUp = data.Coefficient.Sign > 0;
        if (!opensUp && !lowerContained && !upperContained)
        {
            entry = default;
            exit = default;
            return false;
        }

        if (opensUp
            && ((!lowerContained && EvaluateDerivative(data, lower).Sign >= 0)
                || (!upperContained && EvaluateDerivative(data, upper).Sign <= 0)))
        {
            entry = default;
            exit = default;
            return false;
        }

        ConeData normalized = opensUp ? data : data.NegatedPolynomial();
        Signed832 discriminant = WideArithmetic.SubtractSigned832(
            WideArithmetic.MultiplySigned576ToSigned832(normalized.Projection, normalized.Projection),
            WideArithmetic.MultiplySigned576ToSigned832(normalized.Coefficient, normalized.Constant));
        if (discriminant.Sign < 0)
        {
            entry = default;
            exit = default;
            return false;
        }

        Signed192 outputScaleRaw = Signed192.Signed(outputScale.m_rawValue);
        Signed192 outputScaleSquared = SquareRaw(outputScale.m_rawValue);
        Signed576 scaledSquareRoot = WideArithmetic.GetFloorSquareRootOfProduct(
            discriminant,
            outputScaleSquared);

        if (opensUp)
        {
            entry = lowerContained
                ? Round(lower, outputScale)
                : RoundLowerRoot(normalized, scaledSquareRoot, outputScaleRaw);
            exit = upperContained
                ? Round(upper, outputScale)
                : RoundUpperRoot(normalized, scaledSquareRoot, outputScaleRaw);
            return true;
        }

        entry = lowerContained
            ? Round(lower, outputScale)
            : RoundUpperRoot(normalized, scaledSquareRoot, outputScaleRaw);
        exit = upperContained
            ? Round(upper, outputScale)
            : RoundLowerRoot(normalized, scaledSquareRoot, outputScaleRaw);
        return true;
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

    private static Fixed64 Round(RationalBound value, Fixed64 outputScale)
    {
        Signed320 numerator = WideArithmetic.MultiplySigned192(
            value.Numerator,
            Signed192.Signed(outputScale.m_rawValue));
        _ = Fixed64.TryGetSignedRawRatio(
            Signed576.ExtendValue(numerator),
            Signed576.ExtendValue(Signed320.ExtendValue(value.Denominator)),
            out Fixed64 result);
        return result;
    }

    private static Signed192 GetDot(
        Vector3d leftEnd,
        Vector3d leftStart,
        Vector3d rightEnd,
        Vector3d rightStart) =>
        WideGeometry.GetDifferenceDotProduct3D(
            leftEnd.X, leftStart.X, leftEnd.Y, leftStart.Y, leftEnd.Z, leftStart.Z,
            rightEnd.X, rightStart.X, rightEnd.Y, rightStart.Y, rightEnd.Z, rightStart.Z);
}
