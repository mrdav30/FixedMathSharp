//=======================================================================
// WideFiniteAxisIntersection.Ray.cs
//=======================================================================
// MIT License, Copyright (c) 2024–present David Oravsky (mrdav30)
// See LICENSE file in the project root for full license information.
//=======================================================================

namespace FixedMathSharp.Geometry;

/// <content>
/// Provides ray/segment-based intersection helpers for finite-axis shapes,
/// including capsule interval computation against a <see cref="FixedRay2d"/>.
/// </content>
internal static partial class WideFiniteAxisIntersection
{
    internal static bool TryGetCapsuleInterval(
        FixedRay2d query,
        Fixed64 maxParameter,
        FixedSegment2d axis,
        Fixed64 radius,
        Fixed64 radiusExpansion,
        out Fixed64 entry,
        out Fixed64 exit,
        out bool originContained,
        out bool maximumContainedStrict)
    {
        Signed192 expandedRadius = GetExpandedRadius(radius, radiusExpansion);
        Signed192 squaredRadius = GetSquaredRadius(expandedRadius);
        Signed192 axisLengthSquared = GetDot(axis.End, axis.Start, axis.End, axis.Start);
        Signed192 queryLengthSquared = GetDirectionDot(query.Direction, query.Direction);
        Signed192 startDistanceSquared = GetDot(query.Position, axis.Start, query.Position, axis.Start);
        Signed192 directionsDot = GetDirectionDot(query.Direction, axis.End, axis.Start);
        Signed192 startAxisProjection = GetDot(query.Position, axis.Start, axis.End, axis.Start);
        Signed192 startDirectionProjection = GetDirectionDot(query.Direction, query.Position, axis.Start);

        originContained = axisLengthSquared.IsZero
            ? IsWithinRadius(startDistanceSquared, squaredRadius, strict: false)
            : IsCapsulePointContained(
                startDistanceSquared,
                GetDot(query.Position, axis.End, query.Position, axis.End),
                startAxisProjection,
                axisLengthSquared,
                squaredRadius,
                strict: false);
        maximumContainedStrict = IsCapsuleContainedAtParameter(
            query,
            maxParameter,
            axis,
            expandedRadius,
            queryLengthSquared,
            axisLengthSquared,
            startDistanceSquared,
            directionsDot,
            startAxisProjection,
            startDirectionProjection);

        if (axisLengthSquared.IsZero)
        {
            return TryGetCircleInterval(
                query.Position,
                query.Direction,
                axis.Start,
                expandedRadius,
                maxParameter,
                out entry,
                out exit);
        }

        bool found = TryGetFiniteAxisInterval(
            queryLengthSquared,
            axisLengthSquared,
            startDistanceSquared,
            directionsDot,
            startAxisProjection,
            startDirectionProjection,
            expandedRadius,
            maxParameter,
            out entry,
            out exit);
        Merge(
            TryGetCircleInterval(
                query.Position, query.Direction, axis.Start, expandedRadius, maxParameter,
                out Fixed64 startEntry, out Fixed64 startExit),
            startEntry, startExit, ref found, ref entry, ref exit);
        Merge(
            TryGetCircleInterval(
                query.Position, query.Direction, axis.End, expandedRadius, maxParameter,
                out Fixed64 endEntry, out Fixed64 endExit),
            endEntry, endExit, ref found, ref entry, ref exit);
        return found;
    }

    internal static bool TryGetCapsuleInterval(
        FixedRay query,
        Fixed64 maxParameter,
        FixedSegment axis,
        Fixed64 radius,
        Fixed64 radiusExpansion,
        out Fixed64 entry,
        out Fixed64 exit,
        out bool originContained,
        out bool maximumContainedStrict)
    {
        Signed192 expandedRadius = GetExpandedRadius(radius, radiusExpansion);
        Signed192 squaredRadius = GetSquaredRadius(expandedRadius);
        Signed192 axisLengthSquared = GetDot(axis.End, axis.Start, axis.End, axis.Start);
        Signed192 queryLengthSquared = GetDirectionDot(query.Direction, query.Direction);
        Signed192 startDistanceSquared = GetDot(query.Position, axis.Start, query.Position, axis.Start);
        Signed192 directionsDot = GetDirectionDot(query.Direction, axis.End, axis.Start);
        Signed192 startAxisProjection = GetDot(query.Position, axis.Start, axis.End, axis.Start);
        Signed192 startDirectionProjection = GetDirectionDot(query.Direction, query.Position, axis.Start);

        originContained = axisLengthSquared.IsZero
            ? IsWithinRadius(startDistanceSquared, squaredRadius, strict: false)
            : IsCapsulePointContained(
                startDistanceSquared,
                GetDot(query.Position, axis.End, query.Position, axis.End),
                startAxisProjection,
                axisLengthSquared,
                squaredRadius,
                strict: false);
        maximumContainedStrict = IsCapsuleContainedAtParameter(
            query,
            maxParameter,
            axis,
            expandedRadius,
            queryLengthSquared,
            axisLengthSquared,
            startDistanceSquared,
            directionsDot,
            startAxisProjection,
            startDirectionProjection);

        if (axisLengthSquared.IsZero)
        {
            return TryGetSphereInterval(
                query.Position,
                query.Direction,
                axis.Start,
                expandedRadius,
                maxParameter,
                out entry,
                out exit);
        }

        bool found = TryGetFiniteAxisInterval(
            queryLengthSquared,
            axisLengthSquared,
            startDistanceSquared,
            directionsDot,
            startAxisProjection,
            startDirectionProjection,
            expandedRadius,
            maxParameter,
            out entry,
            out exit);
        Merge(
            TryGetSphereInterval(
                query.Position, query.Direction, axis.Start, expandedRadius, maxParameter,
                out Fixed64 startEntry, out Fixed64 startExit),
            startEntry, startExit, ref found, ref entry, ref exit);
        Merge(
            TryGetSphereInterval(
                query.Position, query.Direction, axis.End, expandedRadius, maxParameter,
                out Fixed64 endEntry, out Fixed64 endExit),
            endEntry, endExit, ref found, ref entry, ref exit);
        return found;
    }

    internal static bool TryGetCapsuleInterval(
        FixedRay2d query,
        Fixed64 maxParameter,
        Vector2d center,
        Vector2d axisDirection,
        Fixed64 axisLength,
        Fixed64 radius,
        Fixed64 radiusExpansion,
        out Fixed64 entry,
        out Fixed64 exit,
        out bool originContained,
        out bool maximumContainedStrict)
    {
        Signed192 expandedRadius = GetExpandedRadius(radius, radiusExpansion);
        Signed192 squaredRadius = GetSquaredRadius(expandedRadius);
        Signed192 axisLengthSquared = GetDot(axisDirection, Vector2d.Zero, axisDirection, Vector2d.Zero);
        Signed192 queryLengthSquared = GetDirectionDot(query.Direction, query.Direction);
        Signed192 startDistanceSquared = GetDot(query.Position, center, query.Position, center);
        Signed192 directionsDot = GetDirectionDot(query.Direction, axisDirection);
        Signed192 startAxisProjection = GetDot(query.Position, center, axisDirection, Vector2d.Zero);
        Signed192 startDirectionProjection = GetDirectionDot(query.Direction, query.Position, center);
        Signed320 axialExtent = GetCenteredAxialExtent(
            axisLengthSquared,
            axisLength,
            Fixed64.Zero);

        originContained = IsCenteredCapsulePointContained(
            query.Position, center, axisDirection, axisLength,
            startDistanceSquared, startAxisProjection, axisLengthSquared,
            squaredRadius, expandedRadius, axialExtent, strict: false);
        maximumContainedStrict = IsCenteredCapsuleContainedAtParameter(
            query,
            maxParameter,
            center,
            axisDirection,
            axisLength,
            expandedRadius,
            queryLengthSquared,
            axisLengthSquared,
            startDistanceSquared,
            directionsDot,
            startAxisProjection,
            startDirectionProjection,
            axialExtent);

        bool found = false;
        entry = default;
        exit = default;
        if (TryGetCenteredAxialInterval(
                startAxisProjection,
                directionsDot,
                axialExtent,
                maxParameter,
                out RationalBound320 lower,
                out RationalBound320 upper))
        {
            found = TryGetFiniteAxisInterval(
                queryLengthSquared,
                axisLengthSquared,
                startDistanceSquared,
                directionsDot,
                startAxisProjection,
                startDirectionProjection,
                expandedRadius,
                lower,
                upper,
                maxParameter,
                out entry,
                out exit);
        }

        MergeCenteredCapInterval(
            query, maxParameter, center, axisDirection, axisLength,
            expandedRadius, positiveCap: false, ref found, ref entry, ref exit);
        MergeCenteredCapInterval(
            query, maxParameter, center, axisDirection, axisLength,
            expandedRadius, positiveCap: true, ref found, ref entry, ref exit);
        return found;
    }

    internal static bool TryGetCapsuleInterval(
        FixedRay query,
        Fixed64 maxParameter,
        Vector3d center,
        Vector3d axisDirection,
        Fixed64 axisLength,
        Fixed64 radius,
        Fixed64 radiusExpansion,
        out Fixed64 entry,
        out Fixed64 exit,
        out bool originContained,
        out bool maximumContainedStrict)
    {
        Signed192 expandedRadius = GetExpandedRadius(radius, radiusExpansion);
        Signed192 squaredRadius = GetSquaredRadius(expandedRadius);
        Signed192 axisLengthSquared = GetDot(axisDirection, Vector3d.Zero, axisDirection, Vector3d.Zero);
        Signed192 queryLengthSquared = GetDirectionDot(query.Direction, query.Direction);
        Signed192 startDistanceSquared = GetDot(query.Position, center, query.Position, center);
        Signed192 directionsDot = GetDirectionDot(query.Direction, axisDirection);
        Signed192 startAxisProjection = GetDot(query.Position, center, axisDirection, Vector3d.Zero);
        Signed192 startDirectionProjection = GetDirectionDot(query.Direction, query.Position, center);
        Signed320 axialExtent = GetCenteredAxialExtent(
            axisLengthSquared,
            axisLength,
            Fixed64.Zero);

        originContained = IsCenteredCapsulePointContained(
            query.Position, center, axisDirection, axisLength,
            startDistanceSquared, startAxisProjection, axisLengthSquared,
            squaredRadius, expandedRadius, axialExtent, strict: false);
        maximumContainedStrict = IsCenteredCapsuleContainedAtParameter(
            query,
            maxParameter,
            center,
            axisDirection,
            axisLength,
            expandedRadius,
            queryLengthSquared,
            axisLengthSquared,
            startDistanceSquared,
            directionsDot,
            startAxisProjection,
            startDirectionProjection,
            axialExtent);

        bool found = false;
        entry = default;
        exit = default;
        if (TryGetCenteredAxialInterval(
                startAxisProjection,
                directionsDot,
                axialExtent,
                maxParameter,
                out RationalBound320 lower,
                out RationalBound320 upper))
        {
            found = TryGetFiniteAxisInterval(
                queryLengthSquared,
                axisLengthSquared,
                startDistanceSquared,
                directionsDot,
                startAxisProjection,
                startDirectionProjection,
                expandedRadius,
                lower,
                upper,
                maxParameter,
                out entry,
                out exit);
        }

        MergeCenteredCapInterval(
            query, maxParameter, center, axisDirection, axisLength,
            expandedRadius, positiveCap: false, ref found, ref entry, ref exit);
        MergeCenteredCapInterval(
            query, maxParameter, center, axisDirection, axisLength,
            expandedRadius, positiveCap: true, ref found, ref entry, ref exit);
        return found;
    }

    internal static bool TryGetFiniteCylinderInterval(
        FixedRay query,
        Fixed64 maxParameter,
        FixedSegment axis,
        Fixed64 radius,
        Fixed64 radiusExpansion,
        out Fixed64 entry,
        out Fixed64 exit,
        out bool originContained,
        out bool maximumContainedStrict)
    {
        Signed192 expandedRadius = GetExpandedRadius(radius, radiusExpansion);
        Signed192 squaredRadius = GetSquaredRadius(expandedRadius);
        Signed192 axisLengthSquared = GetDot(axis.End, axis.Start, axis.End, axis.Start);
        Signed192 queryLengthSquared = GetDirectionDot(query.Direction, query.Direction);
        Signed192 startDistanceSquared = GetDot(query.Position, axis.Start, query.Position, axis.Start);
        Signed192 directionsDot = GetDirectionDot(query.Direction, axis.End, axis.Start);
        Signed192 startAxisProjection = GetDot(query.Position, axis.Start, axis.End, axis.Start);
        Signed192 startDirectionProjection = GetDirectionDot(query.Direction, query.Position, axis.Start);

        originContained = IsFiniteCylinderPointContained(
            startDistanceSquared, startAxisProjection, axisLengthSquared, squaredRadius, strict: false);
        maximumContainedStrict = IsFiniteCylinderContainedAtParameter(
            maxParameter,
            expandedRadius,
            queryLengthSquared,
            axisLengthSquared,
            startDistanceSquared,
            directionsDot,
            startAxisProjection,
            startDirectionProjection);
        return TryGetFiniteAxisInterval(
            queryLengthSquared,
            axisLengthSquared,
            startDistanceSquared,
            directionsDot,
            startAxisProjection,
            startDirectionProjection,
            expandedRadius,
            maxParameter,
            out entry,
            out exit);
    }

    internal static bool TryGetFiniteCylinderInterval(
        FixedRay query,
        Fixed64 maxParameter,
        Vector3d center,
        Vector3d axisDirection,
        Fixed64 axisLength,
        Fixed64 radius,
        Fixed64 radiusExpansion,
        Fixed64 axialExpansion,
        out Fixed64 entry,
        out Fixed64 exit,
        out bool originContained,
        out bool maximumContainedStrict)
    {
        Signed192 queryLengthSquared = GetDirectionDot(query.Direction, query.Direction);
        Signed192 axisLengthSquared = GetDot(axisDirection, Vector3d.Zero, axisDirection, Vector3d.Zero);
        Signed192 startDistanceSquared = GetDot(query.Position, center, query.Position, center);
        Signed192 directionsDot = GetDirectionDot(query.Direction, axisDirection);
        Signed192 startAxisProjection = GetDot(query.Position, center, axisDirection, Vector3d.Zero);
        Signed192 startDirectionProjection = GetDirectionDot(query.Direction, query.Position, center);
        Signed192 expandedRadius = GetExpandedRadius(radius, radiusExpansion);
        Signed192 squaredRadius = GetSquaredRadius(expandedRadius);
        Signed320 axialExtent = GetCenteredAxialExtent(
            axisLengthSquared,
            axisLength,
            axialExpansion);

        originContained = IsCenteredFiniteCylinderPointContained(
            startDistanceSquared, startAxisProjection, axisLengthSquared,
            squaredRadius, axialExtent, strict: false);
        maximumContainedStrict = IsCenteredFiniteCylinderContainedAtParameter(
            maxParameter,
            expandedRadius,
            queryLengthSquared,
            axisLengthSquared,
            startDistanceSquared,
            directionsDot,
            startAxisProjection,
            startDirectionProjection,
            axialExtent);

        if (!TryGetCenteredAxialInterval(
                startAxisProjection,
                directionsDot,
                axialExtent,
                maxParameter,
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
            maxParameter,
            out entry,
            out exit);
    }

    private static bool IsCapsuleContainedAtParameter(
        FixedRay2d query,
        Fixed64 parameter,
        FixedSegment2d axis,
        Signed192 radius,
        Signed192 queryLengthSquared,
        Signed192 axisLengthSquared,
        Signed192 startDistanceSquared,
        Signed192 directionsDot,
        Signed192 startAxisProjection,
        Signed192 startDirectionProjection)
    {
        if (axisLengthSquared.IsZero)
        {
            return IsQuadraticContained(
                queryLengthSquared,
                startDirectionProjection,
                WideArithmetic.SubtractSigned192(startDistanceSquared, GetSquaredRadius(radius)),
                parameter);
        }

        Signed320 projection = EvaluateLinear(startAxisProjection, directionsDot, parameter);
        if (projection.Sign <= 0)
        {
            return IsQuadraticContained(
                queryLengthSquared,
                startDirectionProjection,
                WideArithmetic.SubtractSigned192(startDistanceSquared, GetSquaredRadius(radius)),
                parameter);
        }

        Signed320 maximumProjection = WideArithmetic.MultiplySigned192(axisLengthSquared, ParameterScale);
        if (WideArithmetic.SubtractSigned320(projection, maximumProjection).Sign >= 0)
        {
            Signed192 endDistanceSquared = GetDot(query.Position, axis.End, query.Position, axis.End);
            Signed192 endDirectionProjection = GetDirectionDot(query.Direction, query.Position, axis.End);
            return IsQuadraticContained(
                queryLengthSquared,
                endDirectionProjection,
                WideArithmetic.SubtractSigned192(endDistanceSquared, GetSquaredRadius(radius)),
                parameter);
        }

        GetRadialPolynomial(
            queryLengthSquared, axisLengthSquared, startDistanceSquared,
            directionsDot, startAxisProjection, startDirectionProjection, radius,
            out Signed320 coefficient, out Signed320 radialProjection, out Signed320 constant);
        return IsQuadraticContained(coefficient, radialProjection, constant, parameter);
    }

    private static bool IsCapsuleContainedAtParameter(
        FixedRay query,
        Fixed64 parameter,
        FixedSegment axis,
        Signed192 radius,
        Signed192 queryLengthSquared,
        Signed192 axisLengthSquared,
        Signed192 startDistanceSquared,
        Signed192 directionsDot,
        Signed192 startAxisProjection,
        Signed192 startDirectionProjection)
    {
        if (axisLengthSquared.IsZero)
        {
            return IsQuadraticContained(
                queryLengthSquared,
                startDirectionProjection,
                WideArithmetic.SubtractSigned192(startDistanceSquared, GetSquaredRadius(radius)),
                parameter);
        }

        Signed320 projection = EvaluateLinear(startAxisProjection, directionsDot, parameter);
        if (projection.Sign <= 0)
        {
            return IsQuadraticContained(
                queryLengthSquared,
                startDirectionProjection,
                WideArithmetic.SubtractSigned192(startDistanceSquared, GetSquaredRadius(radius)),
                parameter);
        }

        Signed320 maximumProjection = WideArithmetic.MultiplySigned192(axisLengthSquared, ParameterScale);
        if (WideArithmetic.SubtractSigned320(projection, maximumProjection).Sign >= 0)
        {
            Signed192 endDistanceSquared = GetDot(query.Position, axis.End, query.Position, axis.End);
            Signed192 endDirectionProjection = GetDirectionDot(query.Direction, query.Position, axis.End);
            return IsQuadraticContained(
                queryLengthSquared,
                endDirectionProjection,
                WideArithmetic.SubtractSigned192(endDistanceSquared, GetSquaredRadius(radius)),
                parameter);
        }

        GetRadialPolynomial(
            queryLengthSquared, axisLengthSquared, startDistanceSquared,
            directionsDot, startAxisProjection, startDirectionProjection, radius,
            out Signed320 coefficient, out Signed320 radialProjection, out Signed320 constant);
        return IsQuadraticContained(coefficient, radialProjection, constant, parameter);
    }

    private static bool IsFiniteCylinderContainedAtParameter(
        Fixed64 parameter,
        Signed192 radius,
        Signed192 queryLengthSquared,
        Signed192 axisLengthSquared,
        Signed192 startDistanceSquared,
        Signed192 directionsDot,
        Signed192 startAxisProjection,
        Signed192 startDirectionProjection)
    {
        Signed320 projection = EvaluateLinear(startAxisProjection, directionsDot, parameter);
        Signed320 maximumProjection = WideArithmetic.MultiplySigned192(axisLengthSquared, ParameterScale);
        int minimumSign = projection.Sign;
        int maximumSign = WideArithmetic.SubtractSigned320(projection, maximumProjection).Sign;
        if (minimumSign <= 0 || maximumSign >= 0)
        {
            return false;
        }

        GetRadialPolynomial(
            queryLengthSquared, axisLengthSquared, startDistanceSquared,
            directionsDot, startAxisProjection, startDirectionProjection, radius,
            out Signed320 coefficient, out Signed320 radialProjection, out Signed320 constant);
        return IsQuadraticContained(coefficient, radialProjection, constant, parameter);
    }

    private static bool IsCenteredFiniteCylinderContainedAtParameter(
        Fixed64 parameter,
        Signed192 radius,
        Signed192 queryLengthSquared,
        Signed192 axisLengthSquared,
        Signed192 startDistanceSquared,
        Signed192 directionsDot,
        Signed192 startAxisProjection,
        Signed192 startDirectionProjection,
        Signed320 axialExtent)
    {
        Signed320 projection = EvaluateLinear(startAxisProjection, directionsDot, parameter);
        projection = WideArithmetic.AddSigned320(projection, projection);
        Signed320 minimumProjection = WideArithmetic.SubtractSigned320(default, axialExtent);
        int minimumSign = WideArithmetic.SubtractSigned320(projection, minimumProjection).Sign;
        int maximumSign = WideArithmetic.SubtractSigned320(projection, axialExtent).Sign;
        if (minimumSign <= 0 || maximumSign >= 0)
        {
            return false;
        }

        GetRadialPolynomial(
            queryLengthSquared, axisLengthSquared, startDistanceSquared,
            directionsDot, startAxisProjection, startDirectionProjection, radius,
            out Signed320 coefficient, out Signed320 radialProjection, out Signed320 constant);
        return IsQuadraticContained(coefficient, radialProjection, constant, parameter);
    }

    private static bool IsCenteredCapsuleContainedAtParameter(
        FixedRay2d query,
        Fixed64 parameter,
        Vector2d center,
        Vector2d axisDirection,
        Fixed64 axisLength,
        Signed192 radius,
        Signed192 queryLengthSquared,
        Signed192 axisLengthSquared,
        Signed192 startDistanceSquared,
        Signed192 directionsDot,
        Signed192 startAxisProjection,
        Signed192 startDirectionProjection,
        Signed320 axialExtent)
    {
        Signed320 projection = EvaluateLinear(startAxisProjection, directionsDot, parameter);
        projection = WideArithmetic.AddSigned320(projection, projection);
        int cap = GetCenteredCap(projection, axialExtent);
        if (cap != 0)
        {
            GetCenteredCapPolynomial(
                query, center, axisDirection, axisLength, radius, cap > 0,
                out Signed320 coefficient, out Signed320 capProjection, out Signed320 constant);
            return IsQuadraticContained(coefficient, capProjection, constant, parameter);
        }

        GetRadialPolynomial(
            queryLengthSquared, axisLengthSquared, startDistanceSquared,
            directionsDot, startAxisProjection, startDirectionProjection, radius,
            out Signed320 radialCoefficient, out Signed320 radialProjection, out Signed320 radialConstant);
        return IsQuadraticContained(radialCoefficient, radialProjection, radialConstant, parameter);
    }

    private static bool IsCenteredCapsuleContainedAtParameter(
        FixedRay query,
        Fixed64 parameter,
        Vector3d center,
        Vector3d axisDirection,
        Fixed64 axisLength,
        Signed192 radius,
        Signed192 queryLengthSquared,
        Signed192 axisLengthSquared,
        Signed192 startDistanceSquared,
        Signed192 directionsDot,
        Signed192 startAxisProjection,
        Signed192 startDirectionProjection,
        Signed320 axialExtent)
    {
        Signed320 projection = EvaluateLinear(startAxisProjection, directionsDot, parameter);
        projection = WideArithmetic.AddSigned320(projection, projection);
        int cap = GetCenteredCap(projection, axialExtent);
        if (cap != 0)
        {
            GetCenteredCapPolynomial(
                query, center, axisDirection, axisLength, radius, cap > 0,
                out Signed320 coefficient, out Signed320 capProjection, out Signed320 constant);
            return IsQuadraticContained(coefficient, capProjection, constant, parameter);
        }

        GetRadialPolynomial(
            queryLengthSquared, axisLengthSquared, startDistanceSquared,
            directionsDot, startAxisProjection, startDirectionProjection, radius,
            out Signed320 radialCoefficient, out Signed320 radialProjection, out Signed320 radialConstant);
        return IsQuadraticContained(radialCoefficient, radialProjection, radialConstant, parameter);
    }

    private static int GetCenteredCap(Signed320 scaledProjection, Signed320 axialExtent)
    {
        if (WideArithmetic.AddSigned320(scaledProjection, axialExtent).Sign <= 0)
            return -1;
        return WideArithmetic.SubtractSigned320(scaledProjection, axialExtent).Sign >= 0
            ? 1
            : 0;
    }

    private static void MergeCenteredCapInterval(
        FixedRay2d query,
        Fixed64 maxParameter,
        Vector2d center,
        Vector2d axisDirection,
        Fixed64 axisLength,
        Signed192 radius,
        bool positiveCap,
        ref bool found,
        ref Fixed64 entry,
        ref Fixed64 exit)
    {
        GetCenteredCapPolynomial(
            query, center, axisDirection, axisLength, radius, positiveCap,
            out Signed320 coefficient, out Signed320 projection, out Signed320 constant);
        RationalBound320 maximum = GetMaximumBound320(maxParameter);
        Signed320 one = Scale320;
        Merge(
            TrySolveBoundedQuadratic(
                coefficient, projection, constant,
                new RationalBound320(default, one), maximum, maxParameter,
                out Fixed64 capEntry, out Fixed64 capExit),
            capEntry, capExit, ref found, ref entry, ref exit);
    }

    private static void MergeCenteredCapInterval(
        FixedRay query,
        Fixed64 maxParameter,
        Vector3d center,
        Vector3d axisDirection,
        Fixed64 axisLength,
        Signed192 radius,
        bool positiveCap,
        ref bool found,
        ref Fixed64 entry,
        ref Fixed64 exit)
    {
        GetCenteredCapPolynomial(
            query, center, axisDirection, axisLength, radius, positiveCap,
            out Signed320 coefficient, out Signed320 projection, out Signed320 constant);
        RationalBound320 maximum = GetMaximumBound320(maxParameter);
        Signed320 one = Scale320;
        Merge(
            TrySolveBoundedQuadratic(
                coefficient, projection, constant,
                new RationalBound320(default, one), maximum, maxParameter,
                out Fixed64 capEntry, out Fixed64 capExit),
            capEntry, capExit, ref found, ref entry, ref exit);
    }

    private static void GetCenteredCapPolynomial(
        FixedRay2d query,
        Vector2d center,
        Vector2d axisDirection,
        Fixed64 axisLength,
        Signed192 radius,
        bool positiveCap,
        out Signed320 coefficient,
        out Signed320 projection,
        out Signed320 constant)
    {
        Signed192 x = GetCenteredCapOffset(
            query.Position.X, center.X, axisDirection.X, axisLength, positiveCap);
        Signed192 y = GetCenteredCapOffset(
            query.Position.Y, center.Y, axisDirection.Y, axisLength, positiveCap);
        Signed192 velocityX = ScaleByCenteredAxis(
            Signed192.Signed(query.Direction.X.m_rawValue));
        Signed192 velocityY = ScaleByCenteredAxis(
            Signed192.Signed(query.Direction.Y.m_rawValue));
        Signed192 scaledRadius = ScaleByCenteredAxis(radius);
        coefficient = WideArithmetic.AddProducts(velocityX, velocityX, velocityY, velocityY);
        projection = WideArithmetic.AddProducts(x, velocityX, y, velocityY);
        constant = WideArithmetic.SubtractSigned320(
            WideArithmetic.AddProducts(x, x, y, y),
            WideArithmetic.MultiplySigned192(scaledRadius, scaledRadius));
    }

    private static void GetCenteredCapPolynomial(
        FixedRay query,
        Vector3d center,
        Vector3d axisDirection,
        Fixed64 axisLength,
        Signed192 radius,
        bool positiveCap,
        out Signed320 coefficient,
        out Signed320 projection,
        out Signed320 constant)
    {
        Signed192 x = GetCenteredCapOffset(
            query.Position.X, center.X, axisDirection.X, axisLength, positiveCap);
        Signed192 y = GetCenteredCapOffset(
            query.Position.Y, center.Y, axisDirection.Y, axisLength, positiveCap);
        Signed192 z = GetCenteredCapOffset(
            query.Position.Z, center.Z, axisDirection.Z, axisLength, positiveCap);
        Signed192 velocityX = ScaleByCenteredAxis(
            Signed192.Signed(query.Direction.X.m_rawValue));
        Signed192 velocityY = ScaleByCenteredAxis(
            Signed192.Signed(query.Direction.Y.m_rawValue));
        Signed192 velocityZ = ScaleByCenteredAxis(
            Signed192.Signed(query.Direction.Z.m_rawValue));
        Signed192 scaledRadius = ScaleByCenteredAxis(radius);
        coefficient = WideArithmetic.AddProducts(
            velocityX, velocityX, velocityY, velocityY, velocityZ, velocityZ);
        projection = WideArithmetic.AddProducts(x, velocityX, y, velocityY, z, velocityZ);
        constant = WideArithmetic.SubtractSigned320(
            WideArithmetic.AddProducts(x, x, y, y, z, z),
            WideArithmetic.MultiplySigned192(scaledRadius, scaledRadius));
    }

    private static Signed320 EvaluateLinear(
        Signed192 constant,
        Signed192 velocity,
        Fixed64 parameter) =>
        WideArithmetic.AddSigned320(
            WideArithmetic.MultiplySigned192(constant, ParameterScale),
            WideArithmetic.MultiplySigned192(
                velocity,
                Signed192.Signed(parameter.m_rawValue)));

    private static void GetRadialPolynomial(
        Signed192 queryLengthSquared,
        Signed192 axisLengthSquared,
        Signed192 startDistanceSquared,
        Signed192 directionsDot,
        Signed192 startAxisProjection,
        Signed192 startDirectionProjection,
        Signed192 radius,
        out Signed320 coefficient,
        out Signed320 projection,
        out Signed320 constant)
    {
        coefficient = WideArithmetic.MultiplySubtract(
            queryLengthSquared, axisLengthSquared, directionsDot, directionsDot);
        projection = WideArithmetic.MultiplySubtract(
            startDirectionProjection, axisLengthSquared, startAxisProjection, directionsDot);
        constant = GetRadialConstant(
            startDistanceSquared,
            startAxisProjection,
            axisLengthSquared,
            GetSquaredRadius(radius));
    }

    private static bool IsQuadraticContained(
        Signed192 coefficient,
        Signed192 projection,
        Signed192 constant,
        Fixed64 parameter) =>
        IsQuadraticContained(
            Signed320.ExtendValue(coefficient),
            Signed320.ExtendValue(projection),
            Signed320.ExtendValue(constant),
            parameter);

    private static bool IsQuadraticContained(
        Signed320 coefficient,
        Signed320 projection,
        Signed320 constant,
        Fixed64 parameter)
    {
        Signed320 numerator = Signed320.ExtendValue(
            Signed192.Signed(parameter.m_rawValue));
        Signed320 denominator = Signed320.ExtendValue(ParameterScale);
        int sign = EvaluatePolynomial(
            coefficient,
            projection,
            constant,
            numerator,
            denominator).Sign;
        return sign < 0;
    }

    private static bool TryGetCircleInterval(
        Vector2d position,
        Vector2d direction,
        Vector2d center,
        Signed192 radius,
        Fixed64 maxParameter,
        out Fixed64 entry,
        out Fixed64 exit) =>
        WideRayIntersection.TrySolveInterval(
            GetDirectionDot(direction, direction),
            GetDirectionDot(direction, position, center),
            WideArithmetic.SubtractSigned192(
                GetDot(position, center, position, center),
                GetSquaredRadius(radius)),
            maxParameter,
            out entry,
            out exit);

    private static bool TryGetSphereInterval(
        Vector3d position,
        Vector3d direction,
        Vector3d center,
        Signed192 radius,
        Fixed64 maxParameter,
        out Fixed64 entry,
        out Fixed64 exit) =>
        WideRayIntersection.TrySolveInterval(
            GetDirectionDot(direction, direction),
            GetDirectionDot(direction, position, center),
            WideArithmetic.SubtractSigned192(
                GetDot(position, center, position, center),
                GetSquaredRadius(radius)),
            maxParameter,
            out entry,
            out exit);

    private static Signed192 GetDirectionDot(Vector2d left, Vector2d right) =>
        GetDot(left, Vector2d.Zero, right, Vector2d.Zero);

    private static Signed192 GetDirectionDot(
        Vector2d direction,
        Vector2d point,
        Vector2d origin) =>
        GetDot(direction, Vector2d.Zero, point, origin);

    private static Signed192 GetDirectionDot(Vector3d left, Vector3d right) =>
        GetDot(left, Vector3d.Zero, right, Vector3d.Zero);

    private static Signed192 GetDirectionDot(
        Vector3d direction,
        Vector3d point,
        Vector3d origin) =>
        GetDot(direction, Vector3d.Zero, point, origin);
}
