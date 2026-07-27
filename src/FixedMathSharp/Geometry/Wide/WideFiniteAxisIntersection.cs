//=======================================================================
// WideFiniteAxisIntersection.cs
//=======================================================================
// MIT License, Copyright (c) 2024–present David Oravsky (mrdav30)
// See LICENSE file in the project root for full license information.
//=======================================================================

namespace FixedMathSharp.Bounds;

/// <summary>
/// Owns exact full-domain query-segment intersections with finite radial axes.
/// </summary>
internal static partial class WideFiniteAxisIntersection
{
    private static readonly Signed192 Scale = Signed192.Signed(1L);
    private static readonly Signed320 Scale320 = Signed320.ExtendValue(Scale);
    private static readonly Signed192 ParameterScale = Signed192.Signed(FixedMath.ONE_L);
    private static readonly Signed192 DoubleParameterScale = Signed192.Signed(FixedMath.ONE_L * 2L);

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

    private readonly struct RationalBound320
    {
        internal readonly Signed320 Numerator;
        internal readonly Signed320 Denominator;

        internal RationalBound320(Signed320 numerator, Signed320 denominator)
        {
            Numerator = numerator;
            Denominator = denominator;
        }
    }

    #endregion

    internal static bool TryGetCapsuleInterval(
        FixedSegment2d query,
        FixedSegment2d axis,
        Fixed64 radius,
        Fixed64 radiusExpansion,
        out Fixed64 entry,
        out Fixed64 exit,
        out bool startContained,
        out bool endContainedStrict)
    {
        Signed192 expandedRadius = GetExpandedRadius(radius, radiusExpansion);
        Signed192 squaredRadius = GetSquaredRadius(expandedRadius);
        Signed192 axisLengthSquared = GetDot(axis.End, axis.Start, axis.End, axis.Start);
        Signed192 queryLengthSquared = GetDot(query.End, query.Start, query.End, query.Start);
        Signed192 startDistanceSquared = GetDot(query.Start, axis.Start, query.Start, axis.Start);
        Signed192 endDistanceSquared = GetDot(query.End, axis.Start, query.End, axis.Start);
        Signed192 directionsDot = GetDot(query.End, query.Start, axis.End, axis.Start);
        Signed192 startAxisProjection = GetDot(query.Start, axis.Start, axis.End, axis.Start);
        Signed192 endAxisProjection = GetDot(query.End, axis.Start, axis.End, axis.Start);
        Signed192 startDirectionProjection = GetDot(query.Start, axis.Start, query.End, query.Start);

        if (axisLengthSquared.IsZero)
        {
            startContained = IsWithinRadius(startDistanceSquared, squaredRadius, strict: false);
            endContainedStrict = IsWithinRadius(endDistanceSquared, squaredRadius, strict: true);
            return TryGetCircleInterval(
                query.Start,
                query.End,
                axis.Start,
                expandedRadius,
                out entry,
                out exit);
        }

        startContained = IsCapsulePointContained(
            startDistanceSquared,
            GetDot(query.Start, axis.End, query.Start, axis.End),
            startAxisProjection,
            axisLengthSquared,
            squaredRadius,
            strict: false);
        endContainedStrict = IsCapsulePointContained(
            endDistanceSquared,
            GetDot(query.End, axis.End, query.End, axis.End),
            endAxisProjection,
            axisLengthSquared,
            squaredRadius,
            strict: true);
        bool found = TryGetFiniteAxisInterval(
            queryLengthSquared,
            axisLengthSquared,
            startDistanceSquared,
            directionsDot,
            startAxisProjection,
            startDirectionProjection,
            expandedRadius,
            Fixed64.One,
            out entry,
            out exit);
        Merge(
            TryGetCircleInterval(query.Start, query.End, axis.Start, expandedRadius, out Fixed64 startEntry, out Fixed64 startExit),
            startEntry,
            startExit,
            ref found,
            ref entry,
            ref exit);
        Merge(
            TryGetCircleInterval(query.Start, query.End, axis.End, expandedRadius, out Fixed64 endEntry, out Fixed64 endExit),
            endEntry,
            endExit,
            ref found,
            ref entry,
            ref exit);
        return found;
    }

    internal static bool TryGetCapsuleInterval(
        FixedSegment query,
        FixedSegment axis,
        Fixed64 radius,
        Fixed64 radiusExpansion,
        out Fixed64 entry,
        out Fixed64 exit,
        out bool startContained,
        out bool endContainedStrict)
    {
        Signed192 expandedRadius = GetExpandedRadius(radius, radiusExpansion);
        Signed192 squaredRadius = GetSquaredRadius(expandedRadius);
        Signed192 axisLengthSquared = GetDot(axis.End, axis.Start, axis.End, axis.Start);
        Signed192 queryLengthSquared = GetDot(query.End, query.Start, query.End, query.Start);
        Signed192 startDistanceSquared = GetDot(query.Start, axis.Start, query.Start, axis.Start);
        Signed192 endDistanceSquared = GetDot(query.End, axis.Start, query.End, axis.Start);
        Signed192 directionsDot = GetDot(query.End, query.Start, axis.End, axis.Start);
        Signed192 startAxisProjection = GetDot(query.Start, axis.Start, axis.End, axis.Start);
        Signed192 endAxisProjection = GetDot(query.End, axis.Start, axis.End, axis.Start);
        Signed192 startDirectionProjection = GetDot(query.Start, axis.Start, query.End, query.Start);

        if (axisLengthSquared.IsZero)
        {
            startContained = IsWithinRadius(startDistanceSquared, squaredRadius, strict: false);
            endContainedStrict = IsWithinRadius(endDistanceSquared, squaredRadius, strict: true);
            return TryGetSphereInterval(
                query.Start,
                query.End,
                axis.Start,
                expandedRadius,
                out entry,
                out exit);
        }

        startContained = IsCapsulePointContained(
            startDistanceSquared,
            GetDot(query.Start, axis.End, query.Start, axis.End),
            startAxisProjection,
            axisLengthSquared,
            squaredRadius,
            strict: false);
        endContainedStrict = IsCapsulePointContained(
            endDistanceSquared,
            GetDot(query.End, axis.End, query.End, axis.End),
            endAxisProjection,
            axisLengthSquared,
            squaredRadius,
            strict: true);
        bool found = TryGetFiniteAxisInterval(
            queryLengthSquared,
            axisLengthSquared,
            startDistanceSquared,
            directionsDot,
            startAxisProjection,
            startDirectionProjection,
            expandedRadius,
            Fixed64.One,
            out entry,
            out exit);
        Merge(
            TryGetSphereInterval(query.Start, query.End, axis.Start, expandedRadius, out Fixed64 startEntry, out Fixed64 startExit),
            startEntry,
            startExit,
            ref found,
            ref entry,
            ref exit);
        Merge(
            TryGetSphereInterval(query.Start, query.End, axis.End, expandedRadius, out Fixed64 endEntry, out Fixed64 endExit),
            endEntry,
            endExit,
            ref found,
            ref entry,
            ref exit);
        return found;
    }

    internal static bool TryGetFiniteCylinderInterval(
        FixedSegment query,
        FixedSegment axis,
        Fixed64 radius,
        Fixed64 radiusExpansion,
        out Fixed64 entry,
        out Fixed64 exit,
        out bool startContained,
        out bool endContainedStrict)
    {
        Signed192 expandedRadius = GetExpandedRadius(radius, radiusExpansion);
        Signed192 squaredRadius = GetSquaredRadius(expandedRadius);
        Signed192 axisLengthSquared = GetDot(axis.End, axis.Start, axis.End, axis.Start);
        Signed192 queryLengthSquared = GetDot(query.End, query.Start, query.End, query.Start);
        Signed192 startDistanceSquared = GetDot(query.Start, axis.Start, query.Start, axis.Start);
        Signed192 endDistanceSquared = GetDot(query.End, axis.Start, query.End, axis.Start);
        Signed192 directionsDot = GetDot(query.End, query.Start, axis.End, axis.Start);
        Signed192 startAxisProjection = GetDot(query.Start, axis.Start, axis.End, axis.Start);
        Signed192 endAxisProjection = GetDot(query.End, axis.Start, axis.End, axis.Start);
        Signed192 startDirectionProjection = GetDot(query.Start, axis.Start, query.End, query.Start);
        startContained = IsFiniteCylinderPointContained(
            startDistanceSquared,
            startAxisProjection,
            axisLengthSquared,
            squaredRadius,
            strict: false);
        endContainedStrict = IsFiniteCylinderPointContained(
            endDistanceSquared,
            endAxisProjection,
            axisLengthSquared,
            squaredRadius,
            strict: true);
        return TryGetFiniteAxisInterval(
            queryLengthSquared,
            axisLengthSquared,
            startDistanceSquared,
            directionsDot,
            startAxisProjection,
            startDirectionProjection,
            expandedRadius,
            Fixed64.One,
            out entry,
            out exit);
    }

    internal static bool TryGetFiniteCylinderInterval(
        FixedSegment query,
        Vector3d center,
        Vector3d axisDirection,
        Fixed64 axisLength,
        Fixed64 radius,
        Fixed64 radiusExpansion,
        Fixed64 axialExpansion,
        out Fixed64 entry,
        out Fixed64 exit,
        out bool startContained,
        out bool endContainedStrict)
    {
        Signed192 queryLengthSquared = GetDot(query.End, query.Start, query.End, query.Start);
        Signed192 axisLengthSquared = GetDot(axisDirection, Vector3d.Zero, axisDirection, Vector3d.Zero);
        Signed192 startDistanceSquared = GetDot(query.Start, center, query.Start, center);
        Signed192 endDistanceSquared = GetDot(query.End, center, query.End, center);
        Signed192 directionsDot = GetDot(query.End, query.Start, axisDirection, Vector3d.Zero);
        Signed192 startAxisProjection = GetDot(query.Start, center, axisDirection, Vector3d.Zero);
        Signed192 endAxisProjection = GetDot(query.End, center, axisDirection, Vector3d.Zero);
        Signed192 startDirectionProjection = GetDot(query.Start, center, query.End, query.Start);
        Signed192 expandedRadius = GetExpandedRadius(radius, radiusExpansion);
        Signed192 squaredRadius = GetSquaredRadius(expandedRadius);
        Signed320 axialExtent = GetCenteredAxialExtent(
            axisLengthSquared,
            axisLength,
            axialExpansion);

        startContained = IsCenteredFiniteCylinderPointContained(
            startDistanceSquared,
            startAxisProjection,
            axisLengthSquared,
            squaredRadius,
            axialExtent,
            strict: false);
        endContainedStrict = IsCenteredFiniteCylinderPointContained(
            endDistanceSquared,
            endAxisProjection,
            axisLengthSquared,
            squaredRadius,
            axialExtent,
            strict: true);

        if (!TryGetCenteredAxialInterval(
                startAxisProjection,
                directionsDot,
                axialExtent,
                Fixed64.One,
                out RationalBound320 lower,
                out RationalBound320 upper))
        {
            entry = default;
            exit = default;
            return false;
        }

        return TryGetFiniteAxisInterval(
            queryLengthSquared,
            axisLengthSquared,
            startDistanceSquared,
            directionsDot,
            startAxisProjection,
            startDirectionProjection,
            expandedRadius,
            lower,
            upper,
            Fixed64.One,
            out entry,
            out exit);
    }

    private static bool TryGetFiniteAxisInterval(
        Signed192 queryLengthSquared,
        Signed192 axisLengthSquared,
        Signed192 startDistanceSquared,
        Signed192 directionsDot,
        Signed192 startAxisProjection,
        Signed192 startDirectionProjection,
        Signed192 radius,
        RationalBound320 lower,
        RationalBound320 upper,
        Fixed64 maxParameter,
        out Fixed64 entry,
        out Fixed64 exit)
    {
        entry = default;
        exit = default;

        Signed320 radialCoefficient = WideArithmetic.MultiplySubtract(
            queryLengthSquared,
            axisLengthSquared,
            directionsDot,
            directionsDot);
        Signed320 radialProjection = WideArithmetic.MultiplySubtract(
            startDirectionProjection,
            axisLengthSquared,
            startAxisProjection,
            directionsDot);
        Signed320 radialConstant = GetRadialConstant(
            startDistanceSquared,
            startAxisProjection,
            axisLengthSquared,
            GetSquaredRadius(radius));
        return TrySolveBoundedQuadratic(
            radialCoefficient,
            radialProjection,
            radialConstant,
            lower,
            upper,
            maxParameter,
            out entry,
            out exit);
    }

    private static bool TryGetFiniteAxisInterval(
        Signed192 queryLengthSquared,
        Signed192 axisLengthSquared,
        Signed192 startDistanceSquared,
        Signed192 directionsDot,
        Signed192 startAxisProjection,
        Signed192 startDirectionProjection,
        Signed192 radius,
        Fixed64 maxParameter,
        out Fixed64 entry,
        out Fixed64 exit)
    {
        entry = default;
        exit = default;

        if (!TryGetAxialInterval(
                startAxisProjection,
                directionsDot,
                axisLengthSquared,
                maxParameter,
                out RationalBound lower,
                out RationalBound upper))
        {
            return false;
        }

        Signed320 radialCoefficient = WideArithmetic.MultiplySubtract(
            queryLengthSquared,
            axisLengthSquared,
            directionsDot,
            directionsDot);
        Signed320 radialProjection = WideArithmetic.MultiplySubtract(
            startDirectionProjection,
            axisLengthSquared,
            startAxisProjection,
            directionsDot);
        Signed320 radialConstant = GetRadialConstant(
            startDistanceSquared,
            startAxisProjection,
            axisLengthSquared,
            GetSquaredRadius(radius));

        if (radialCoefficient.IsZero)
        {
            if (radialConstant.Sign > 0)
                return false;

            entry = Round(lower);
            exit = Round(upper);
            return true;
        }

        Signed576 lowerValue = EvaluatePolynomial(
            radialCoefficient,
            radialProjection,
            radialConstant,
            lower.Numerator,
            lower.Denominator);
        Signed576 upperValue = EvaluatePolynomial(
            radialCoefficient,
            radialProjection,
            radialConstant,
            upper.Numerator,
            upper.Denominator);
        int lowerDerivative = EvaluateDerivative(
            radialCoefficient,
            radialProjection,
            lower.Numerator,
            lower.Denominator).Sign;
        int upperDerivative = EvaluateDerivative(
            radialCoefficient,
            radialProjection,
            upper.Numerator,
            upper.Denominator).Sign;

        if ((lowerValue.Sign > 0 && lowerDerivative >= 0)
            || (upperValue.Sign > 0 && upperDerivative <= 0))
        {
            return false;
        }

        if (lowerValue.Sign <= 0 && upperValue.Sign <= 0)
        {
            entry = Round(lower);
            exit = Round(upper);
            return true;
        }

        if (TryGetNarrowRadialCoefficients(
                radialCoefficient,
                radialProjection,
                radialConstant,
                out Signed192 narrowCoefficient,
                out Signed192 narrowProjection,
                out Signed192 narrowConstant))
        {
            if (!WideRayIntersection.TrySolveInterval(
                    narrowCoefficient,
                    narrowProjection,
                    narrowConstant,
                    maxParameter,
                    out Fixed64 radialEntry,
                    out Fixed64 radialExit))
            {
                return false;
            }

            entry = lowerValue.Sign <= 0 ? Round(lower) : radialEntry;
            exit = upperValue.Sign <= 0 ? Round(upper) : radialExit;
            return true;
        }

        Signed576 discriminant = WideArithmetic.SubtractSigned576(
            WideArithmetic.MultiplySigned320(radialProjection, radialProjection),
            WideArithmetic.MultiplySigned320(radialCoefficient, radialConstant));
        if (discriminant.Sign < 0)
            return false;

        Signed320 scaledSquareRoot = WideArithmetic.GetFloorSquareRootScaledByFixed64(discriminant);

        entry = lowerValue.Sign <= 0
            ? Round(lower)
            : RoundLowerRoot(radialCoefficient, radialProjection, radialConstant, scaledSquareRoot);
        exit = upperValue.Sign <= 0
            ? Round(upper)
            : RoundUpperRoot(radialCoefficient, radialProjection, radialConstant, scaledSquareRoot, maxParameter);
        return true;
    }

    private static bool TryGetAxialInterval(
        Signed192 projection,
        Signed192 velocity,
        Signed192 axisLengthSquared,
        Fixed64 maxParameter,
        out RationalBound lower,
        out RationalBound upper)
    {
        RationalBound maximum = GetMaximumBound(maxParameter);
        lower = new RationalBound(default, Scale);
        upper = maximum;

        if (velocity.IsZero)
        {
            return projection.Sign >= 0
                && WideArithmetic.SubtractSigned192(projection, axisLengthSquared).Sign <= 0;
        }

        RationalBound first = Normalize(
            WideArithmetic.SubtractSigned192(default, projection),
            velocity);
        RationalBound second = Normalize(
            WideArithmetic.SubtractSigned192(axisLengthSquared, projection),
            velocity);
        if (Compare(first, second) > 0)
            (first, second) = (second, first);

        if (second.Numerator.Sign < 0 || Compare(first, maximum) > 0)
            return false;

        if (first.Numerator.Sign > 0)
            lower = first;
        if (Compare(second, maximum) < 0)
            upper = second;
        return true;
    }

    private static bool TryGetCenteredAxialInterval(
        Signed192 projection,
        Signed192 velocity,
        Signed320 axialExtent,
        Fixed64 maxParameter,
        out RationalBound320 lower,
        out RationalBound320 upper)
    {
        Signed320 scaledProjection = WideArithmetic.MultiplySigned192(DoubleParameterScale, projection);
        Signed320 scaledVelocity = WideArithmetic.MultiplySigned192(DoubleParameterScale, velocity);
        Signed320 minimumProjection = WideArithmetic.SubtractSigned320(default, axialExtent);
        RationalBound320 maximum = GetMaximumBound320(maxParameter);
        lower = new RationalBound320(default, Signed320.ExtendValue(Scale));
        upper = maximum;

        if (scaledVelocity.IsZero)
        {
            return WideArithmetic.SubtractSigned320(scaledProjection, minimumProjection).Sign >= 0
                && WideArithmetic.SubtractSigned320(scaledProjection, axialExtent).Sign <= 0;
        }

        RationalBound320 first = Normalize(
            WideArithmetic.SubtractSigned320(minimumProjection, scaledProjection),
            scaledVelocity);
        RationalBound320 second = Normalize(
            WideArithmetic.SubtractSigned320(axialExtent, scaledProjection),
            scaledVelocity);
        if (Compare(first, second) > 0)
            (first, second) = (second, first);

        if (second.Numerator.Sign < 0 || Compare(first, maximum) > 0)
            return false;

        if (first.Numerator.Sign > 0)
            lower = first;
        if (Compare(second, maximum) < 0)
            upper = second;
        return true;
    }

    private static Signed320 GetCenteredAxialExtent(
        Signed192 axisLengthSquared,
        Fixed64 axisLength,
        Fixed64 axialExpansion) =>
        GetCenteredAxialExtent(
            axisLengthSquared,
            Signed192.Signed(axisLength.m_rawValue),
            axialExpansion);

    private static Signed320 GetCenteredAxialExtent(
        Signed192 axisLengthSquared,
        Signed192 axisLength,
        Fixed64 axialExpansion) =>
        WideArithmetic.AddSigned320(
            WideArithmetic.MultiplySigned192(
                axisLength,
                axisLengthSquared),
            WideArithmetic.MultiplySigned192(
                WideArithmetic.AddSigned192(
                    Signed192.Signed(axialExpansion.m_rawValue),
                    Signed192.Signed(axialExpansion.m_rawValue)),
                axisLengthSquared));

    private static RationalBound Normalize(Signed192 numerator, Signed192 denominator)
    {
        if (denominator.Sign >= 0)
            return new RationalBound(numerator, denominator);

        return new RationalBound(
            WideArithmetic.SubtractSigned192(default, numerator),
            WideArithmetic.SubtractSigned192(default, denominator));
    }

    private static RationalBound320 Normalize(Signed320 numerator, Signed320 denominator)
    {
        if (denominator.Sign >= 0)
            return new RationalBound320(numerator, denominator);

        return new RationalBound320(
            WideArithmetic.SubtractSigned320(default, numerator),
            WideArithmetic.SubtractSigned320(default, denominator));
    }

    private static int Compare(RationalBound left, RationalBound right) =>
        WideArithmetic.MultiplySubtract(
            left.Numerator,
            right.Denominator,
            right.Numerator,
            left.Denominator).Sign;

    private static int Compare(RationalBound320 left, RationalBound320 right) =>
        WideArithmetic.SubtractSigned576(
            WideArithmetic.MultiplySigned320(left.Numerator, right.Denominator),
            WideArithmetic.MultiplySigned320(right.Numerator, left.Denominator)).Sign;

    private static RationalBound GetMaximumBound(Fixed64 maxParameter) =>
        new(Signed192.Signed(maxParameter.m_rawValue), ParameterScale);

    private static RationalBound320 GetMaximumBound320(Fixed64 maxParameter) =>
        new(
            Signed320.ExtendValue(
                Signed192.Signed(maxParameter.m_rawValue)),
            Signed320.ExtendValue(ParameterScale));

    private static Fixed64 Round(RationalBound value) =>
        Fixed64.GetSignedRatio(value.Numerator, value.Denominator);

    private static Fixed64 Round(RationalBound320 value) =>
        Fixed64.GetSignedRatio(value.Numerator, value.Denominator);

    private static bool TryGetCircleInterval(
        Vector2d start,
        Vector2d end,
        Vector2d center,
        Signed192 radius,
        out Fixed64 entry,
        out Fixed64 exit)
    {
        Signed192 directionLengthSquared = GetDot(end, start, end, start);
        Signed192 projection = GetDot(start, center, end, start);
        Signed192 distanceSquared = GetDot(start, center, start, center);
        return WideRayIntersection.TrySolveInterval(
            directionLengthSquared,
            projection,
            WideArithmetic.SubtractSigned192(distanceSquared, GetSquaredRadius(radius)),
            Fixed64.One,
            out entry,
            out exit);
    }

    private static bool TryGetSphereInterval(
        Vector3d start,
        Vector3d end,
        Vector3d center,
        Signed192 radius,
        out Fixed64 entry,
        out Fixed64 exit)
    {
        Signed192 directionLengthSquared = GetDot(end, start, end, start);
        Signed192 projection = GetDot(start, center, end, start);
        Signed192 distanceSquared = GetDot(start, center, start, center);
        return WideRayIntersection.TrySolveInterval(
            directionLengthSquared,
            projection,
            WideArithmetic.SubtractSigned192(distanceSquared, GetSquaredRadius(radius)),
            Fixed64.One,
            out entry,
            out exit);
    }

    private static Signed192 GetExpandedRadius(Fixed64 radius, Fixed64 radiusExpansion) =>
        WideArithmetic.AddSigned192(
            Signed192.Signed(radius.m_rawValue),
            Signed192.Signed(radiusExpansion.m_rawValue));

    private static Signed192 GetSquaredRadius(Signed192 radius)
    {
        WideArithmetic.GetMagnitude(radius, out _, out _, out ulong rawRadius);
        Fixed64.Multiply64To128(rawRadius, rawRadius, out ulong middle, out ulong low);
        return new Signed192(0UL, middle, low);
    }

    private static Signed192 GetDot(
        Vector2d leftEnd,
        Vector2d leftStart,
        Vector2d rightEnd,
        Vector2d rightStart) =>
        WideGeometry.GetDifferenceDotProduct2D(
            leftEnd.X,
            leftStart.X,
            leftEnd.Y,
            leftStart.Y,
            rightEnd.X,
            rightStart.X,
            rightEnd.Y,
            rightStart.Y);

    private static Signed192 GetDot(
        Vector3d leftEnd,
        Vector3d leftStart,
        Vector3d rightEnd,
        Vector3d rightStart) =>
        WideGeometry.GetDifferenceDotProduct3D(
            leftEnd.X,
            leftStart.X,
            leftEnd.Y,
            leftStart.Y,
            leftEnd.Z,
            leftStart.Z,
            rightEnd.X,
            rightStart.X,
            rightEnd.Y,
            rightStart.Y,
            rightEnd.Z,
            rightStart.Z);

    private static void Merge(
        bool candidateFound,
        Fixed64 candidateEntry,
        Fixed64 candidateExit,
        ref bool found,
        ref Fixed64 entry,
        ref Fixed64 exit)
    {
        if (!candidateFound)
            return;

        if (!found)
        {
            found = true;
            entry = candidateEntry;
            exit = candidateExit;
            return;
        }

        if (candidateEntry < entry)
            entry = candidateEntry;
        if (candidateExit > exit)
            exit = candidateExit;
    }

    private static bool TrySolveBoundedQuadratic(
        Signed320 coefficient,
        Signed320 projection,
        Signed320 constant,
        RationalBound320 lower,
        RationalBound320 upper,
        Fixed64 maxParameter,
        out Fixed64 entry,
        out Fixed64 exit)
    {
        entry = default;
        exit = default;

        if (coefficient.IsZero)
        {
            if (constant.Sign > 0)
                return false;

            entry = Round(lower);
            exit = Round(upper);
            return true;
        }

        Signed704 lowerValue = EvaluatePolynomial(
            coefficient,
            projection,
            constant,
            lower.Numerator,
            lower.Denominator);
        Signed704 upperValue = EvaluatePolynomial(
            coefficient,
            projection,
            constant,
            upper.Numerator,
            upper.Denominator);
        int lowerDerivative = EvaluateDerivative(
            coefficient,
            projection,
            lower.Numerator,
            lower.Denominator).Sign;
        int upperDerivative = EvaluateDerivative(
            coefficient,
            projection,
            upper.Numerator,
            upper.Denominator).Sign;

        if ((lowerValue.Sign > 0 && lowerDerivative >= 0)
            || (upperValue.Sign > 0 && upperDerivative <= 0))
        {
            return false;
        }

        if (lowerValue.Sign <= 0 && upperValue.Sign <= 0)
        {
            entry = Round(lower);
            exit = Round(upper);
            return true;
        }

        if (TryGetNarrowRadialCoefficients(
                coefficient,
                projection,
                constant,
                out Signed192 narrowCoefficient,
                out Signed192 narrowProjection,
                out Signed192 narrowConstant))
        {
            if (!WideRayIntersection.TrySolveInterval(
                    narrowCoefficient,
                    narrowProjection,
                    narrowConstant,
                    maxParameter,
                    out Fixed64 radialEntry,
                    out Fixed64 radialExit))
            {
                return false;
            }

            entry = lowerValue.Sign <= 0 ? Round(lower) : radialEntry;
            exit = upperValue.Sign <= 0 ? Round(upper) : radialExit;
            return true;
        }

        Signed576 discriminant = WideArithmetic.SubtractSigned576(
            WideArithmetic.MultiplySigned320(projection, projection),
            WideArithmetic.MultiplySigned320(coefficient, constant));
        if (discriminant.Sign < 0)
            return false;

        Signed320 scaledSquareRoot = WideArithmetic.GetFloorSquareRootScaledByFixed64(discriminant);
        entry = lowerValue.Sign <= 0
            ? Round(lower)
            : RoundLowerRoot(coefficient, projection, constant, scaledSquareRoot);
        exit = upperValue.Sign <= 0
            ? Round(upper)
            : RoundUpperRoot(coefficient, projection, constant, scaledSquareRoot, maxParameter);
        return true;
    }
}
