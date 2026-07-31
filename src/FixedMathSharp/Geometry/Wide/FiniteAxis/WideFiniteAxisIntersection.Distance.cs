//=======================================================================
// WideFiniteAxisIntersection.Distance.cs
//=======================================================================
// MIT License, Copyright (c) 2024–present David Oravsky (mrdav30)
// See LICENSE file in the project root for full license information.
//=======================================================================

namespace FixedMathSharp.Geometry;

/// <content>
/// Physical-distance intervals for radial and finite-axis intersection tests.
/// </content>
internal static partial class WideFiniteAxisIntersection
{
    internal static bool TryGetCircleDistanceInterval(
        FixedSegment2d query,
        FixedBoundCircle circle,
        Fixed64 radiusExpansion,
        Fixed64 segmentLength,
        out Fixed64 entry,
        out Fixed64 exit,
        out bool startContained,
        out bool endContainedStrict)
    {
        Signed192 expandedRadius = GetExpandedRadius(circle.Radius, radiusExpansion);
        Signed192 squaredRadius = GetSquaredRadius(expandedRadius);
        startContained = IsWithinRadius(
            GetDot(query.Start, circle.Center, query.Start, circle.Center),
            squaredRadius,
            strict: false);
        endContainedStrict = IsWithinRadius(
            GetDot(query.End, circle.Center, query.End, circle.Center),
            squaredRadius,
            strict: true);
        return TryGetCircleDistanceInterval(
            query.Start,
            query.End,
            circle.Center,
            expandedRadius,
            segmentLength,
            out entry,
            out exit);
    }

    internal static bool TryGetSphereDistanceInterval(
        FixedSegment query,
        FixedBoundSphere sphere,
        Fixed64 radiusExpansion,
        Fixed64 segmentLength,
        out Fixed64 entry,
        out Fixed64 exit,
        out bool startContained,
        out bool endContainedStrict)
    {
        Signed192 expandedRadius = GetExpandedRadius(sphere.Radius, radiusExpansion);
        Signed192 squaredRadius = GetSquaredRadius(expandedRadius);
        startContained = IsWithinRadius(
            GetDot(query.Start, sphere.Center, query.Start, sphere.Center),
            squaredRadius,
            strict: false);
        endContainedStrict = IsWithinRadius(
            GetDot(query.End, sphere.Center, query.End, sphere.Center),
            squaredRadius,
            strict: true);
        return TryGetSphereDistanceInterval(
            query.Start,
            query.End,
            sphere.Center,
            expandedRadius,
            segmentLength,
            out entry,
            out exit);
    }

    internal static bool TryGetCapsuleDistanceInterval(
        FixedSegment2d query,
        FixedSegment2d axis,
        Fixed64 radius,
        Fixed64 radiusExpansion,
        Fixed64 segmentLength,
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
        Signed192 startDirectionProjection = GetDot(query.Start, axis.Start, query.End, query.Start);

        if (axisLengthSquared.IsZero)
        {
            startContained = IsWithinRadius(startDistanceSquared, squaredRadius, strict: false);
            endContainedStrict = IsWithinRadius(endDistanceSquared, squaredRadius, strict: true);
            return TryGetCircleDistanceInterval(
                query.Start,
                query.End,
                axis.Start,
                expandedRadius,
                segmentLength,
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
            GetDot(query.End, axis.Start, axis.End, axis.Start),
            axisLengthSquared,
            squaredRadius,
            strict: true);
        bool found = TryGetFiniteAxisDistanceInterval(
            queryLengthSquared,
            axisLengthSquared,
            startDistanceSquared,
            directionsDot,
            startAxisProjection,
            startDirectionProjection,
            expandedRadius,
            segmentLength,
            out entry,
            out exit);
        Merge(
            TryGetCircleDistanceInterval(query.Start, query.End, axis.Start, expandedRadius, segmentLength, out Fixed64 startEntry, out Fixed64 startExit),
            startEntry,
            startExit,
            ref found,
            ref entry,
            ref exit);
        Merge(
            TryGetCircleDistanceInterval(query.Start, query.End, axis.End, expandedRadius, segmentLength, out Fixed64 endEntry, out Fixed64 endExit),
            endEntry,
            endExit,
            ref found,
            ref entry,
            ref exit);
        return found;
    }

    internal static bool TryGetCapsuleDistanceInterval(
        FixedSegment query,
        FixedSegment axis,
        Fixed64 radius,
        Fixed64 radiusExpansion,
        Fixed64 segmentLength,
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
        Signed192 startDirectionProjection = GetDot(query.Start, axis.Start, query.End, query.Start);

        if (axisLengthSquared.IsZero)
        {
            startContained = IsWithinRadius(startDistanceSquared, squaredRadius, strict: false);
            endContainedStrict = IsWithinRadius(endDistanceSquared, squaredRadius, strict: true);
            return TryGetSphereDistanceInterval(
                query.Start,
                query.End,
                axis.Start,
                expandedRadius,
                segmentLength,
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
            GetDot(query.End, axis.Start, axis.End, axis.Start),
            axisLengthSquared,
            squaredRadius,
            strict: true);
        bool found = TryGetFiniteAxisDistanceInterval(
            queryLengthSquared,
            axisLengthSquared,
            startDistanceSquared,
            directionsDot,
            startAxisProjection,
            startDirectionProjection,
            expandedRadius,
            segmentLength,
            out entry,
            out exit);
        Merge(
            TryGetSphereDistanceInterval(query.Start, query.End, axis.Start, expandedRadius, segmentLength, out Fixed64 startEntry, out Fixed64 startExit),
            startEntry,
            startExit,
            ref found,
            ref entry,
            ref exit);
        Merge(
            TryGetSphereDistanceInterval(query.Start, query.End, axis.End, expandedRadius, segmentLength, out Fixed64 endEntry, out Fixed64 endExit),
            endEntry,
            endExit,
            ref found,
            ref entry,
            ref exit);
        return found;
    }

    internal static bool TryGetCapsuleDistanceInterval(
        FixedSegment2d query,
        Vector2d center,
        Vector2d axisDirection,
        Fixed64 axisLength,
        Fixed64 radius,
        Fixed64 radiusExpansion,
        Fixed64 segmentLength,
        out Fixed64 entry,
        out Fixed64 exit,
        out bool startContained,
        out bool endContainedStrict)
    {
        Signed192 expandedRadius = GetExpandedRadius(radius, radiusExpansion);
        Signed192 squaredRadius = GetSquaredRadius(expandedRadius);
        Signed192 axisLengthSquared = GetDot(axisDirection, Vector2d.Zero, axisDirection, Vector2d.Zero);
        Signed192 queryLengthSquared = GetDot(query.End, query.Start, query.End, query.Start);
        Signed192 startDistanceSquared = GetDot(query.Start, center, query.Start, center);
        Signed192 directionsDot = GetDot(query.End, query.Start, axisDirection, Vector2d.Zero);
        Signed192 startAxisProjection = GetDot(query.Start, center, axisDirection, Vector2d.Zero);
        Signed192 endAxisProjection = GetDot(query.End, center, axisDirection, Vector2d.Zero);
        Signed192 startDirectionProjection = GetDot(query.Start, center, query.End, query.Start);
        Signed320 axialExtent = GetCenteredAxialExtent(axisLengthSquared, axisLength, Fixed64.Zero);

        startContained = IsCenteredCapsulePointContained(
            query.Start, center, axisDirection, axisLength,
            startDistanceSquared, startAxisProjection, axisLengthSquared,
            squaredRadius, expandedRadius, axialExtent, strict: false);
        endContainedStrict = IsCenteredCapsulePointContained(
            query.End, center, axisDirection, axisLength,
            GetDot(query.End, center, query.End, center), endAxisProjection,
            axisLengthSquared, squaredRadius, expandedRadius, axialExtent, strict: true);

        bool found = false;
        entry = default;
        exit = default;
        if (TryGetCenteredAxialInterval(
                startAxisProjection,
                directionsDot,
                axialExtent,
                Fixed64.One,
                out RationalBound320 lower,
                out RationalBound320 upper))
        {
            found = TryGetFiniteAxisDistanceInterval(
                queryLengthSquared, axisLengthSquared, startDistanceSquared,
                directionsDot, startAxisProjection, startDirectionProjection,
                expandedRadius, lower, upper, segmentLength, out entry, out exit);
        }

        MergeCenteredCapDistanceInterval(query, center, axisDirection, axisLength, expandedRadius, segmentLength, false, ref found, ref entry, ref exit);
        MergeCenteredCapDistanceInterval(query, center, axisDirection, axisLength, expandedRadius, segmentLength, true, ref found, ref entry, ref exit);
        return found;
    }

    internal static bool TryGetCapsuleDistanceInterval(
        FixedSegment query,
        Vector3d center,
        Vector3d axisDirection,
        Fixed64 axisLength,
        Fixed64 radius,
        Fixed64 radiusExpansion,
        Fixed64 segmentLength,
        out Fixed64 entry,
        out Fixed64 exit,
        out bool startContained,
        out bool endContainedStrict)
    {
        Signed192 expandedRadius = GetExpandedRadius(radius, radiusExpansion);
        Signed192 squaredRadius = GetSquaredRadius(expandedRadius);
        Signed192 axisLengthSquared = GetDot(axisDirection, Vector3d.Zero, axisDirection, Vector3d.Zero);
        Signed192 queryLengthSquared = GetDot(query.End, query.Start, query.End, query.Start);
        Signed192 startDistanceSquared = GetDot(query.Start, center, query.Start, center);
        Signed192 directionsDot = GetDot(query.End, query.Start, axisDirection, Vector3d.Zero);
        Signed192 startAxisProjection = GetDot(query.Start, center, axisDirection, Vector3d.Zero);
        Signed192 endAxisProjection = GetDot(query.End, center, axisDirection, Vector3d.Zero);
        Signed192 startDirectionProjection = GetDot(query.Start, center, query.End, query.Start);
        Signed320 axialExtent = GetCenteredAxialExtent(axisLengthSquared, axisLength, Fixed64.Zero);

        startContained = IsCenteredCapsulePointContained(
            query.Start, center, axisDirection, axisLength,
            startDistanceSquared, startAxisProjection, axisLengthSquared,
            squaredRadius, expandedRadius, axialExtent, strict: false);
        endContainedStrict = IsCenteredCapsulePointContained(
            query.End, center, axisDirection, axisLength,
            GetDot(query.End, center, query.End, center), endAxisProjection,
            axisLengthSquared, squaredRadius, expandedRadius, axialExtent, strict: true);

        bool found = false;
        entry = default;
        exit = default;
        if (TryGetCenteredAxialInterval(
                startAxisProjection,
                directionsDot,
                axialExtent,
                Fixed64.One,
                out RationalBound320 lower,
                out RationalBound320 upper))
        {
            found = TryGetFiniteAxisDistanceInterval(
                queryLengthSquared, axisLengthSquared, startDistanceSquared,
                directionsDot, startAxisProjection, startDirectionProjection,
                expandedRadius, lower, upper, segmentLength, out entry, out exit);
        }

        MergeCenteredCapDistanceInterval(query, center, axisDirection, axisLength, expandedRadius, segmentLength, false, ref found, ref entry, ref exit);
        MergeCenteredCapDistanceInterval(query, center, axisDirection, axisLength, expandedRadius, segmentLength, true, ref found, ref entry, ref exit);
        return found;
    }

    internal static bool TryGetFiniteCylinderDistanceInterval(
        FixedSegment query,
        FixedSegment axis,
        Fixed64 radius,
        Fixed64 radiusExpansion,
        Fixed64 segmentLength,
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
        Signed192 startDirectionProjection = GetDot(query.Start, axis.Start, query.End, query.Start);
        startContained = IsFiniteCylinderPointContained(
            startDistanceSquared, startAxisProjection, axisLengthSquared, squaredRadius, strict: false);
        endContainedStrict = IsFiniteCylinderPointContained(
            endDistanceSquared,
            GetDot(query.End, axis.Start, axis.End, axis.Start),
            axisLengthSquared,
            squaredRadius,
            strict: true);
        return TryGetFiniteAxisDistanceInterval(
            queryLengthSquared, axisLengthSquared, startDistanceSquared,
            directionsDot, startAxisProjection, startDirectionProjection,
            expandedRadius, segmentLength, out entry, out exit);
    }

    internal static bool TryGetFiniteCylinderDistanceInterval(
        FixedSegment query,
        Vector3d center,
        Vector3d axisDirection,
        Fixed64 axisLength,
        Fixed64 radius,
        Fixed64 radiusExpansion,
        Fixed64 axialExpansion,
        Fixed64 segmentLength,
        out Fixed64 entry,
        out Fixed64 exit,
        out bool startContained,
        out bool endContainedStrict) =>
        TryGetFiniteCylinderDistanceInterval(
            query,
            center,
            axisDirection,
            Signed192.Signed(axisLength.m_rawValue),
            radius,
            radiusExpansion,
            axialExpansion,
            segmentLength,
            out entry,
            out exit,
            out startContained,
            out endContainedStrict);

    internal static bool TryGetFiniteCylinderDistanceInterval(
        FixedSegment query,
        Vector3d center,
        Vector3d axisDirection,
        Signed192 axisLength,
        Fixed64 radius,
        Fixed64 radiusExpansion,
        Fixed64 axialExpansion,
        Fixed64 segmentLength,
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
        Signed192 startDirectionProjection = GetDot(query.Start, center, query.End, query.Start);
        Signed192 expandedRadius = GetExpandedRadius(radius, radiusExpansion);
        Signed192 squaredRadius = GetSquaredRadius(expandedRadius);
        Signed320 axialExtent = GetCenteredAxialExtent(axisLengthSquared, axisLength, axialExpansion);
        startContained = IsCenteredFiniteCylinderPointContained(
            startDistanceSquared, startAxisProjection, axisLengthSquared,
            squaredRadius, axialExtent, strict: false);
        endContainedStrict = IsCenteredFiniteCylinderPointContained(
            endDistanceSquared,
            GetDot(query.End, center, axisDirection, Vector3d.Zero),
            axisLengthSquared,
            squaredRadius,
            axialExtent,
            strict: true);

        if (!TryGetCenteredAxialInterval(
                startAxisProjection, directionsDot, axialExtent, Fixed64.One,
                out RationalBound320 lower, out RationalBound320 upper))
        {
            entry = default;
            exit = default;
            return false;
        }

        return TryGetFiniteAxisDistanceInterval(
            queryLengthSquared, axisLengthSquared, startDistanceSquared,
            directionsDot, startAxisProjection, startDirectionProjection,
            expandedRadius, lower, upper, segmentLength, out entry, out exit);
    }

    private static bool TryGetFiniteAxisDistanceInterval(
        Signed192 queryLengthSquared,
        Signed192 axisLengthSquared,
        Signed192 startDistanceSquared,
        Signed192 directionsDot,
        Signed192 startAxisProjection,
        Signed192 startDirectionProjection,
        Signed192 radius,
        Fixed64 segmentLength,
        out Fixed64 entry,
        out Fixed64 exit)
    {
        if (!TryGetAxialInterval(
                startAxisProjection,
                directionsDot,
                axisLengthSquared,
                Fixed64.One,
                out RationalBound lower,
                out RationalBound upper))
        {
            entry = default;
            exit = default;
            return false;
        }

        return TryGetFiniteAxisDistanceInterval(
            queryLengthSquared,
            axisLengthSquared,
            startDistanceSquared,
            directionsDot,
            startAxisProjection,
            startDirectionProjection,
            radius,
            new RationalBound320(
                Signed320.ExtendValue(lower.Numerator),
                Signed320.ExtendValue(lower.Denominator)),
            new RationalBound320(
                Signed320.ExtendValue(upper.Numerator),
                Signed320.ExtendValue(upper.Denominator)),
            segmentLength,
            out entry,
            out exit);
    }

    private static bool TryGetFiniteAxisDistanceInterval(
        Signed192 queryLengthSquared,
        Signed192 axisLengthSquared,
        Signed192 startDistanceSquared,
        Signed192 directionsDot,
        Signed192 startAxisProjection,
        Signed192 startDirectionProjection,
        Signed192 radius,
        RationalBound320 lower,
        RationalBound320 upper,
        Fixed64 segmentLength,
        out Fixed64 entry,
        out Fixed64 exit)
    {
        Signed320 radialCoefficient = WideArithmetic.MultiplySubtract(
            queryLengthSquared, axisLengthSquared, directionsDot, directionsDot);
        Signed320 radialProjection = WideArithmetic.MultiplySubtract(
            startDirectionProjection, axisLengthSquared, startAxisProjection, directionsDot);
        Signed320 radialConstant = GetRadialConstant(
            startDistanceSquared,
            startAxisProjection,
            axisLengthSquared,
            GetSquaredRadius(radius));
        return TrySolveBoundedQuadraticAtDistance(
            radialCoefficient,
            radialProjection,
            radialConstant,
            lower,
            upper,
            segmentLength,
            out entry,
            out exit);
    }

    private static bool TryGetCircleDistanceInterval(
        Vector2d start,
        Vector2d end,
        Vector2d center,
        Signed192 radius,
        Fixed64 segmentLength,
        out Fixed64 entry,
        out Fixed64 exit) =>
        TrySolveUnitQuadraticAtDistance(
            Signed320.ExtendValue(GetDot(end, start, end, start)),
            Signed320.ExtendValue(GetDot(start, center, end, start)),
            Signed320.ExtendValue(WideArithmetic.SubtractSigned192(
                GetDot(start, center, start, center),
                GetSquaredRadius(radius))),
            segmentLength,
            out entry,
            out exit);

    private static bool TryGetSphereDistanceInterval(
        Vector3d start,
        Vector3d end,
        Vector3d center,
        Signed192 radius,
        Fixed64 segmentLength,
        out Fixed64 entry,
        out Fixed64 exit) =>
        TrySolveUnitQuadraticAtDistance(
            Signed320.ExtendValue(GetDot(end, start, end, start)),
            Signed320.ExtendValue(GetDot(start, center, end, start)),
            Signed320.ExtendValue(WideArithmetic.SubtractSigned192(
                GetDot(start, center, start, center),
                GetSquaredRadius(radius))),
            segmentLength,
            out entry,
            out exit);

    private static void MergeCenteredCapDistanceInterval(
        FixedSegment2d query,
        Vector2d center,
        Vector2d axisDirection,
        Fixed64 axisLength,
        Signed192 expandedRadius,
        Fixed64 segmentLength,
        bool positiveCap,
        ref bool found,
        ref Fixed64 entry,
        ref Fixed64 exit)
    {
        GetCenteredCapPolynomial(
            query, center, axisDirection, axisLength, expandedRadius, positiveCap,
            out Signed320 coefficient, out Signed320 projection, out Signed320 constant);
        Merge(
            TrySolveUnitQuadraticAtDistance(
                coefficient, projection, constant, segmentLength,
                out Fixed64 capEntry, out Fixed64 capExit),
            capEntry, capExit, ref found, ref entry, ref exit);
    }

    private static void MergeCenteredCapDistanceInterval(
        FixedSegment query,
        Vector3d center,
        Vector3d axisDirection,
        Fixed64 axisLength,
        Signed192 expandedRadius,
        Fixed64 segmentLength,
        bool positiveCap,
        ref bool found,
        ref Fixed64 entry,
        ref Fixed64 exit)
    {
        GetCenteredCapPolynomial(
            query, center, axisDirection, axisLength, expandedRadius, positiveCap,
            out Signed320 coefficient, out Signed320 projection, out Signed320 constant);
        Merge(
            TrySolveUnitQuadraticAtDistance(
                coefficient, projection, constant, segmentLength,
                out Fixed64 capEntry, out Fixed64 capExit),
            capEntry, capExit, ref found, ref entry, ref exit);
    }
}
