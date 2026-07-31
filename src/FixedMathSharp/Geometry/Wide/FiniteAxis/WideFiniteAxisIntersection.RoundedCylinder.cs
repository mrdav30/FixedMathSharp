//=======================================================================
// WideFiniteAxisIntersection.RoundedCylinder.cs
//=======================================================================
// MIT License, Copyright (c) 2024–present David Oravsky (mrdav30)
// See LICENSE file in the project root for full license information.
//=======================================================================

namespace FixedMathSharp.Geometry;

/// <content>
/// Rounded-cylinder (capsule-swept torus/cylinder) intersection tests for wide, high-precision
/// finite-axis geometry, using extended-width fixed-point arithmetic for numerical robustness.
/// </content>
internal static partial class WideFiniteAxisIntersection
{
    #region Nested Types

    private readonly struct RoundedCylinderBaseData
    {
        internal readonly Signed192 AxisLengthSquared;
        internal readonly Signed192 G0;
        internal readonly Signed192 G1;
        internal readonly Signed192 G2;
        internal readonly Signed192 StartAxisProjection;
        internal readonly Signed192 DirectionAxisProjection;
        internal readonly Signed320 Q0;
        internal readonly Signed320 Q1;
        internal readonly Signed320 Q2;
        internal readonly Signed192 Radius;
        internal readonly Signed192 Expansion;

        internal RoundedCylinderBaseData(
            Signed192 axisLengthSquared,
            Signed192 g0,
            Signed192 g1,
            Signed192 g2,
            Signed192 startAxisProjection,
            Signed192 directionAxisProjection,
            Signed320 q0,
            Signed320 q1,
            Signed320 q2,
            Signed192 radius,
            Signed192 expansion)
        {
            AxisLengthSquared = axisLengthSquared;
            G0 = g0;
            G1 = g1;
            G2 = g2;
            StartAxisProjection = startAxisProjection;
            DirectionAxisProjection = directionAxisProjection;
            Q0 = q0;
            Q1 = q1;
            Q2 = q2;
            Radius = radius;
            Expansion = expansion;
        }
    }

    private readonly struct RoundedCylinderTorusPolynomial
    {
        internal readonly Signed320 N0;
        internal readonly Signed320 N1;
        internal readonly Signed320 N2;
        internal readonly Signed576 H0;
        internal readonly Signed576 H1;
        internal readonly Signed576 H2;
        internal readonly Signed576 H3;
        internal readonly Signed576 H4;

        internal RoundedCylinderTorusPolynomial(
            Signed320 n0,
            Signed320 n1,
            Signed320 n2,
            Signed576 h0,
            Signed576 h1,
            Signed576 h2,
            Signed576 h3,
            Signed576 h4)
        {
            N0 = n0;
            N1 = n1;
            N2 = n2;
            H0 = h0;
            H1 = h1;
            H2 = h2;
            H3 = h3;
            H4 = h4;
        }
    }

    private readonly struct RoundedCylinderQuadraticBound
    {
        internal static readonly RoundedCylinderQuadraticBound Zero = new(
            default,
            Scale320,
            default,
            0);
        internal static readonly RoundedCylinderQuadraticBound One = new(
            Scale320,
            Scale320,
            default,
            0);

        internal readonly Signed320 Numerator;
        internal readonly Signed320 Denominator;
        internal readonly Signed576 Discriminant;
        internal readonly int RadicalSign;

        internal RoundedCylinderQuadraticBound(
            Signed320 numerator,
            Signed320 denominator,
            Signed576 discriminant,
            int radicalSign)
        {
            Numerator = numerator;
            Denominator = denominator;
            Discriminant = discriminant;
            RadicalSign = radicalSign;
        }
    }

    #endregion

    internal static bool DoesCenteredFiniteCylinderOverlapSphere(
        Vector3d cylinderCenter,
        Vector3d cylinderAxisDirection,
        Fixed64 cylinderAxisLength,
        Fixed64 cylinderRadius,
        Vector3d sphereCenter,
        Fixed64 sphereRadius)
    {
        _ = TryGetSphericallyExpandedFiniteCylinderDistanceIntervalWithFullAxisLength(
            new FixedSegment(sphereCenter, sphereCenter),
            cylinderCenter,
            cylinderAxisDirection,
            cylinderAxisLength,
            cylinderRadius,
            sphereRadius,
            Fixed64.Zero,
            calculateExit: false,
            out _,
            out _,
            out bool contained,
            out _);
        return contained;
    }

    internal static bool TryGetSphericallyExpandedFiniteCylinderDistanceInterval(
        FixedSegment query,
        Vector3d center,
        Vector3d axisDirection,
        Fixed64 axisLength,
        Fixed64 radius,
        Fixed64 sphericalExpansion,
        Fixed64 segmentLength,
        out Fixed64 entry,
        out Fixed64 exit,
        out bool startContained,
        out bool endContainedStrict) =>
        TryGetSphericallyExpandedFiniteCylinderDistanceIntervalWithFullAxisLength(
            query,
            center,
            axisDirection,
            axisLength,
            radius,
            sphericalExpansion,
            segmentLength,
            calculateExit: true,
            out entry,
            out exit,
            out startContained,
            out endContainedStrict);

    internal static bool TryGetSphericallyExpandedFiniteCylinderFirstDistance(
        FixedSegment query,
        Vector3d center,
        Vector3d axisDirection,
        Fixed64 axisLength,
        Fixed64 radius,
        Fixed64 sphericalExpansion,
        Fixed64 segmentLength,
        out Fixed64 distance) =>
        TryGetSphericallyExpandedFiniteCylinderDistanceIntervalWithFullAxisLength(
            query,
            center,
            axisDirection,
            axisLength,
            radius,
            sphericalExpansion,
            segmentLength,
            calculateExit: false,
            out distance,
            out _,
            out _,
            out _);

    internal static bool TryGetSphericallyExpandedFiniteCylinderFirstDistanceFromHalfAxisLength(
        FixedSegment query,
        Vector3d center,
        Vector3d axisDirection,
        Fixed64 halfAxisLength,
        Fixed64 radius,
        Fixed64 sphericalExpansion,
        Fixed64 segmentLength,
        out Fixed64 distance)
    {
        Signed192 halfAxisLengthRaw =
            Signed192.Signed(halfAxisLength.m_rawValue);
        Signed192 fullAxisLength = WideArithmetic.AddSigned192(
            halfAxisLengthRaw,
            halfAxisLengthRaw);
        return TryGetSphericallyExpandedFiniteCylinderDistanceInterval(
            query,
            center,
            axisDirection,
            fullAxisLength,
            radius,
            sphericalExpansion,
            segmentLength,
            calculateExit: false,
            out distance,
            out _,
            out _,
            out _);
    }

    internal static bool TryGetSphericallyExpandedFiniteCylinderDirectionFirstDistanceFromHalfAxisLength(
        Vector3d position,
        Vector3d direction,
        Vector3d center,
        Vector3d axisDirection,
        Fixed64 halfAxisLength,
        Fixed64 radius,
        Fixed64 sphericalExpansion,
        Fixed64 totalDistance,
        out Fixed64 distance)
    {
        Signed192 halfAxisLengthRaw =
            Signed192.Signed(halfAxisLength.m_rawValue);
        Signed192 fullAxisLength = WideArithmetic.AddSigned192(
            halfAxisLengthRaw,
            halfAxisLengthRaw);

        if (sphericalExpansion == Fixed64.Zero)
        {
            return TryGetFiniteCylinderDirectionFirstDistance(
                position,
                direction,
                center,
                axisDirection,
                fullAxisLength,
                radius,
                Fixed64.Zero,
                Fixed64.Zero,
                totalDistance,
                out distance,
                out _);
        }

        bool found = TryGetFiniteCylinderDirectionFirstDistance(
            position,
            direction,
            center,
            axisDirection,
            fullAxisLength,
            radius,
            sphericalExpansion,
            Fixed64.Zero,
            totalDistance,
            out distance,
            out bool sideStartContained);
        RoundedCylinderBaseData baseData = CreateRoundedCylinderBaseData(
            position,
            direction,
            center,
            axisDirection,
            radius,
            sphericalExpansion);
        Fixed64 exit = default;
        return MergeSphericallyExpandedFiniteCylinderRoundedIntervals(
            baseData,
            fullAxisLength,
            totalDistance,
            calculateExit: false,
            ref found,
            ref distance,
            ref exit,
            sideStartContained,
            false,
            out _,
            out _);
    }

    private static bool TryGetSphericallyExpandedFiniteCylinderDistanceIntervalWithFullAxisLength(
        FixedSegment query,
        Vector3d center,
        Vector3d axisDirection,
        Fixed64 axisLength,
        Fixed64 radius,
        Fixed64 sphericalExpansion,
        Fixed64 segmentLength,
        bool calculateExit,
        out Fixed64 entry,
        out Fixed64 exit,
        out bool startContained,
        out bool endContainedStrict)
    {
        if (sphericalExpansion == Fixed64.Zero)
        {
            return TryGetFiniteCylinderDistanceInterval(
                query,
                center,
                axisDirection,
                axisLength,
                radius,
                Fixed64.Zero,
                Fixed64.Zero,
                segmentLength,
                out entry,
                out exit,
                out startContained,
                out endContainedStrict);
        }

        if (radius == Fixed64.Zero)
        {
            return TryGetCapsuleDistanceInterval(
                query,
                center,
                axisDirection,
                axisLength,
                Fixed64.Zero,
                sphericalExpansion,
                segmentLength,
                out entry,
                out exit,
                out startContained,
                out endContainedStrict);
        }

        return TryGetSphericallyExpandedFiniteCylinderDistanceInterval(
            query,
            center,
            axisDirection,
            Signed192.Signed(axisLength.m_rawValue),
            radius,
            sphericalExpansion,
            segmentLength,
            calculateExit,
            out entry,
            out exit,
            out startContained,
            out endContainedStrict);
    }

    private static bool TryGetSphericallyExpandedFiniteCylinderDistanceInterval(
        FixedSegment query,
        Vector3d center,
        Vector3d axisDirection,
        Signed192 axisLength,
        Fixed64 radius,
        Fixed64 sphericalExpansion,
        Fixed64 segmentLength,
        bool calculateExit,
        out Fixed64 entry,
        out Fixed64 exit,
        out bool startContained,
        out bool endContainedStrict)
    {
        if (sphericalExpansion == Fixed64.Zero)
        {
            return TryGetFiniteCylinderDistanceInterval(
                query,
                center,
                axisDirection,
                axisLength,
                radius,
                Fixed64.Zero,
                Fixed64.Zero,
                segmentLength,
                out entry,
                out exit,
                out startContained,
                out endContainedStrict);
        }

        bool found = TryGetFiniteCylinderDistanceInterval(
            query,
            center,
            axisDirection,
            axisLength,
            radius,
            sphericalExpansion,
            Fixed64.Zero,
            segmentLength,
            out entry,
            out exit,
            out bool sideStartContained,
            out bool sideEndContainedStrict);

        RoundedCylinderBaseData baseData = CreateRoundedCylinderBaseData(
            query,
            center,
            axisDirection,
            radius,
            sphericalExpansion);
        return MergeSphericallyExpandedFiniteCylinderRoundedIntervals(
            baseData,
            axisLength,
            segmentLength,
            calculateExit,
            ref found,
            ref entry,
            ref exit,
            sideStartContained,
            sideEndContainedStrict,
            out startContained,
            out endContainedStrict);
    }

    private static bool MergeSphericallyExpandedFiniteCylinderRoundedIntervals(
        RoundedCylinderBaseData baseData,
        Signed192 axisLength,
        Fixed64 segmentLength,
        bool calculateExit,
        ref bool found,
        ref Fixed64 entry,
        ref Fixed64 exit,
        bool sideStartContained,
        bool sideEndContainedStrict,
        out bool startContained,
        out bool endContainedStrict)
    {
        Merge(
            TryGetRoundedCylinderCapCoreDistanceInterval(
                baseData,
                axisLength,
                baseData.Radius,
                segmentLength,
                positiveCap: false,
                out Fixed64 capEntry,
                out Fixed64 capExit,
                out bool negativeCapStartContained,
                out bool negativeCapEndContainedStrict),
            capEntry,
            capExit,
            ref found,
            ref entry,
            ref exit);
        Merge(
            TryGetRoundedCylinderCapCoreDistanceInterval(
                baseData,
                axisLength,
                baseData.Radius,
                segmentLength,
                positiveCap: true,
                out capEntry,
                out capExit,
                out bool positiveCapStartContained,
                out bool positiveCapEndContainedStrict),
            capEntry,
            capExit,
            ref found,
            ref entry,
            ref exit);
        bool negativeStartContained = false;
        bool negativeEndContainedStrict = false;
        if (RoundedCylinderRimBoundsIntersect(
                baseData,
                axisLength,
                segmentLength,
                positiveCap: false))
        {
            MergeRoundedCylinderRimInterval(
                CreateRoundedCylinderTorusPolynomial(baseData, axisLength, positiveCap: false),
                segmentLength,
                ref found,
                ref entry,
                ref exit,
                calculateExit,
                out negativeStartContained,
                out negativeEndContainedStrict);
        }

        bool positiveStartContained = false;
        bool positiveEndContainedStrict = false;
        if (RoundedCylinderRimBoundsIntersect(
                baseData,
                axisLength,
                segmentLength,
                positiveCap: true))
        {
            MergeRoundedCylinderRimInterval(
                CreateRoundedCylinderTorusPolynomial(baseData, axisLength, positiveCap: true),
                segmentLength,
                ref found,
                ref entry,
                ref exit,
                calculateExit,
                out positiveStartContained,
                out positiveEndContainedStrict);
        }

        startContained = sideStartContained
            || negativeCapStartContained
            || positiveCapStartContained
            || negativeStartContained
            || positiveStartContained;
        endContainedStrict = sideEndContainedStrict
            || negativeCapEndContainedStrict
            || positiveCapEndContainedStrict
            || negativeEndContainedStrict
            || positiveEndContainedStrict;
        return found;
    }

    private static bool TryGetRoundedCylinderCapCoreDistanceInterval(
        RoundedCylinderBaseData data,
        Signed192 axisLength,
        Signed192 radialRadius,
        Fixed64 segmentLength,
        bool positiveCap,
        out Fixed64 entry,
        out Fixed64 exit,
        out bool startContained,
        out bool endContainedStrict)
    {
        Signed320 radiusSquared = WideArithmetic.MultiplySigned192(radialRadius, radialRadius);
        Signed320 radialLimit = MultiplyToSigned320(radiusSquared, data.AxisLengthSquared);
        Signed320 radial0 = WideArithmetic.SubtractSigned320(data.Q0, radialLimit);
        Signed320 radialProjection = ShiftRightExact(data.Q1, 1);
        Signed320 radial2 = data.Q2;

        Signed320 scaledStartProjection = WideArithmetic.MultiplySigned192(
            DoubleParameterScale,
            data.StartAxisProjection);
        Signed320 scaledDirectionProjection = WideArithmetic.MultiplySigned192(
            DoubleParameterScale,
            data.DirectionAxisProjection);
        Signed320 capProjection = WideArithmetic.MultiplySigned192(
            axisLength,
            data.AxisLengthSquared);
        Signed320 z0 = positiveCap
            ? WideArithmetic.SubtractSigned320(scaledStartProjection, capProjection)
            : WideArithmetic.AddSigned320(scaledStartProjection, capProjection);
        Signed320 z1 = scaledDirectionProjection;
        Signed320 expansionSquared = WideArithmetic.MultiplySigned192(data.Expansion, data.Expansion);
        Signed320 scaleSquared = WideArithmetic.MultiplySigned192(
            DoubleParameterScale,
            DoubleParameterScale);
        Signed320 axialLimit = MultiplyToSigned320(
            MultiplyToSigned320(expansionSquared, scaleSquared),
            data.AxisLengthSquared);
        Signed320 axial0 = WideArithmetic.SubtractSigned320(
            MultiplyToSigned320(z0, z0),
            axialLimit);
        Signed320 axial1 = MultiplyToSigned320(z0, z1);
        Signed320 axial2 = MultiplyToSigned320(z1, z1);

        int startRadialSign = radial0.Sign;
        int startAxialSign = axial0.Sign;
        int endRadialSign = WideArithmetic.AddSigned320(
            WideArithmetic.AddSigned320(radial0, data.Q1),
            radial2).Sign;
        int endAxialSign = WideArithmetic.AddSigned320(
            WideArithmetic.AddSigned320(axial0, axial1),
            WideArithmetic.AddSigned320(axial1, axial2)).Sign;
        startContained = startRadialSign <= 0 && startAxialSign <= 0;
        endContainedStrict = endRadialSign < 0 && endAxialSign < 0;
        bool endRadialContained = endRadialSign <= 0;
        bool endAxialContained = endAxialSign <= 0;

        if (!TrySolveUnitQuadraticAtDistance(
                radial2,
                radialProjection,
                radial0,
                segmentLength,
                out Fixed64 radialEntry,
                out Fixed64 radialExit)
            || !TrySolveUnitQuadraticAtDistance(
                axial2,
                axial1,
                axial0,
                segmentLength,
                out Fixed64 axialEntry,
                out Fixed64 axialExit))
        {
            entry = default;
            exit = default;
            return false;
        }

        entry = radialEntry >= axialEntry ? radialEntry : axialEntry;
        exit = radialExit <= axialExit ? radialExit : axialExit;
        if (entry < exit)
            return true;
        if (entry > exit)
            return false;

        RoundedCylinderQuadraticBound radialEntryBound = CreateRoundedCylinderQuadraticBound(
            radial2,
            radialProjection,
            radial0,
            startRadialSign <= 0,
            endRadialContained,
            entryBound: true);
        RoundedCylinderQuadraticBound radialExitBound = CreateRoundedCylinderQuadraticBound(
            radial2,
            radialProjection,
            radial0,
            startRadialSign <= 0,
            endRadialContained,
            entryBound: false);
        RoundedCylinderQuadraticBound axialEntryBound = CreateRoundedCylinderQuadraticBound(
            axial2,
            axial1,
            axial0,
            startAxialSign <= 0,
            endAxialContained,
            entryBound: true);
        RoundedCylinderQuadraticBound axialExitBound = CreateRoundedCylinderQuadraticBound(
            axial2,
            axial1,
            axial0,
            startAxialSign <= 0,
            endAxialContained,
            entryBound: false);
        return IsRoundedCylinderEntryBeforeOrEqualToExit(radialEntryBound, axialExitBound)
            && IsRoundedCylinderEntryBeforeOrEqualToExit(axialEntryBound, radialExitBound);
    }

    private static bool RoundedCylinderRimBoundsIntersect(
        RoundedCylinderBaseData data,
        Signed192 axisLength,
        Fixed64 segmentLength,
        bool positiveCap) =>
        TryGetRoundedCylinderCapCoreDistanceInterval(
            data,
            axisLength,
            WideArithmetic.AddSigned192(data.Radius, data.Expansion),
            segmentLength,
            positiveCap,
            out _,
            out _,
            out _,
            out _);

    private static RoundedCylinderQuadraticBound CreateRoundedCylinderQuadraticBound(
        Signed320 coefficient,
        Signed320 projection,
        Signed320 constant,
        bool startContained,
        bool endContained,
        bool entryBound)
    {
        if (entryBound && startContained)
            return RoundedCylinderQuadraticBound.Zero;
        if (!entryBound && endContained)
            return RoundedCylinderQuadraticBound.One;

        Signed576 discriminant = WideArithmetic.SubtractSigned576(
            WideArithmetic.MultiplySigned320(projection, projection),
            WideArithmetic.MultiplySigned320(coefficient, constant));
        return new RoundedCylinderQuadraticBound(
            WideArithmetic.SubtractSigned320(default, projection),
            coefficient,
            discriminant,
            entryBound ? -1 : 1);
    }

    private static void MergeRoundedCylinderRimInterval(
        RoundedCylinderTorusPolynomial polynomial,
        Fixed64 segmentLength,
        ref bool found,
        ref Fixed64 entry,
        ref Fixed64 exit,
        bool calculateExit,
        out bool startContained,
        out bool endContainedStrict)
    {
        startContained = IsPointInRoundedCylinderRimTube(polynomial, atEnd: false, strict: false);
        endContainedStrict = IsPointInRoundedCylinderRimTube(polynomial, atEnd: true, strict: true);
        bool rimFound = TryGetRoundedCylinderRimDistanceInterval(
            polynomial,
            segmentLength,
            startContained,
            IsPointInRoundedCylinderRimTube(polynomial, atEnd: true, strict: false),
            calculateExit,
            out Fixed64 rimEntry,
            out Fixed64 rimExit);
        Merge(rimFound, rimEntry, rimExit, ref found, ref entry, ref exit);
    }

    private static bool IsPointInRoundedCylinderRimTube(
        RoundedCylinderTorusPolynomial polynomial,
        bool atEnd,
        bool strict)
    {
        Signed320 n = atEnd
            ? WideArithmetic.AddSigned320(
                WideArithmetic.AddSigned320(polynomial.N0, polynomial.N1),
                polynomial.N2)
            : polynomial.N0;
        if (!strict && n.Sign <= 0)
            return true;
        if (strict && n.Sign < 0)
            return true;

        Signed576 h = atEnd
            ? AddFive(
                polynomial.H0,
                polynomial.H1,
                polynomial.H2,
                polynomial.H3,
                polynomial.H4)
            : polynomial.H0;
        return strict ? h.Sign < 0 : h.Sign <= 0;
    }

    private static bool TryGetFiniteCylinderDirectionFirstDistance(
        Vector3d position,
        Vector3d direction,
        Vector3d center,
        Vector3d axisDirection,
        Signed192 axisLength,
        Fixed64 radius,
        Fixed64 radiusExpansion,
        Fixed64 axialExpansion,
        Fixed64 totalDistance,
        out Fixed64 distance,
        out bool startContained)
    {
        Signed192 queryLengthSquared = GetDirectionDot(direction, direction);
        Signed192 axisLengthSquared = GetDot(
            axisDirection, Vector3d.Zero, axisDirection, Vector3d.Zero);
        Signed192 startDistanceSquared = GetDot(position, center, position, center);
        Signed192 directionsDot = GetDirectionDot(direction, axisDirection);
        Signed192 startAxisProjection = GetDot(
            position, center, axisDirection, Vector3d.Zero);
        Signed192 startDirectionProjection = GetDirectionDot(direction, position, center);
        Signed192 expandedRadius = GetExpandedRadius(radius, radiusExpansion);
        Signed192 squaredRadius = GetSquaredRadius(expandedRadius);
        Signed320 axialExtent = GetCenteredAxialExtent(
            axisLengthSquared, axisLength, axialExpansion);
        startContained = IsCenteredFiniteCylinderPointContained(
            startDistanceSquared, startAxisProjection, axisLengthSquared,
            squaredRadius, axialExtent, strict: false);

        if (!TryGetCenteredAxialInterval(
                startAxisProjection, directionsDot, axialExtent, Fixed64.One,
                out RationalBound320 lower, out RationalBound320 upper))
        {
            distance = default;
            return false;
        }

        return TryGetFiniteAxisDistanceInterval(
            queryLengthSquared, axisLengthSquared, startDistanceSquared,
            directionsDot, startAxisProjection, startDirectionProjection,
            expandedRadius, lower, upper, totalDistance, out distance, out _);
    }

    private static RoundedCylinderBaseData CreateRoundedCylinderBaseData(
        FixedSegment query,
        Vector3d center,
        Vector3d axisDirection,
        Fixed64 radius,
        Fixed64 sphericalExpansion)
    {
        Signed192 axisLengthSquared = GetDot(
            axisDirection,
            Vector3d.Zero,
            axisDirection,
            Vector3d.Zero);
        Signed192 startDistanceSquared = GetDot(query.Start, center, query.Start, center);
        Signed192 directionLengthSquared = GetDot(query.End, query.Start, query.End, query.Start);
        Signed192 startDirectionProjection = GetDot(query.Start, center, query.End, query.Start);
        Signed192 startAxisProjection = GetDot(query.Start, center, axisDirection, Vector3d.Zero);
        Signed192 directionAxisProjection = GetDot(query.End, query.Start, axisDirection, Vector3d.Zero);

        return CreateRoundedCylinderBaseData(
            axisLengthSquared,
            startDistanceSquared,
            directionLengthSquared,
            startDirectionProjection,
            startAxisProjection,
            directionAxisProjection,
            radius,
            sphericalExpansion);
    }

    private static RoundedCylinderBaseData CreateRoundedCylinderBaseData(
        Vector3d position,
        Vector3d direction,
        Vector3d center,
        Vector3d axisDirection,
        Fixed64 radius,
        Fixed64 sphericalExpansion)
    {
        Signed192 axisLengthSquared = GetDot(
            axisDirection, Vector3d.Zero, axisDirection, Vector3d.Zero);
        Signed192 startDistanceSquared = GetDot(position, center, position, center);
        Signed192 directionLengthSquared = GetDirectionDot(direction, direction);
        Signed192 startDirectionProjection = GetDirectionDot(direction, position, center);
        Signed192 startAxisProjection = GetDot(position, center, axisDirection, Vector3d.Zero);
        Signed192 directionAxisProjection = GetDirectionDot(direction, axisDirection);
        return CreateRoundedCylinderBaseData(
            axisLengthSquared,
            startDistanceSquared,
            directionLengthSquared,
            startDirectionProjection,
            startAxisProjection,
            directionAxisProjection,
            radius,
            sphericalExpansion);
    }

    private static RoundedCylinderBaseData CreateRoundedCylinderBaseData(
        Signed192 axisLengthSquared,
        Signed192 startDistanceSquared,
        Signed192 directionLengthSquared,
        Signed192 startDirectionProjection,
        Signed192 startAxisProjection,
        Signed192 directionAxisProjection,
        Fixed64 radius,
        Fixed64 sphericalExpansion)
    {
        Signed320 q0 = WideArithmetic.MultiplySubtract(
            startDistanceSquared,
            axisLengthSquared,
            startAxisProjection,
            startAxisProjection);
        Signed320 q1 = Twice(WideArithmetic.MultiplySubtract(
            startDirectionProjection,
            axisLengthSquared,
            startAxisProjection,
            directionAxisProjection));
        Signed320 q2 = WideArithmetic.MultiplySubtract(
            directionLengthSquared,
            axisLengthSquared,
            directionAxisProjection,
            directionAxisProjection);

        return new RoundedCylinderBaseData(
            axisLengthSquared,
            startDistanceSquared,
            startDirectionProjection,
            directionLengthSquared,
            startAxisProjection,
            directionAxisProjection,
            q0,
            q1,
            q2,
            Signed192.Signed(radius.m_rawValue),
            Signed192.Signed(sphericalExpansion.m_rawValue));
    }

    private static RoundedCylinderTorusPolynomial CreateRoundedCylinderTorusPolynomial(
        RoundedCylinderBaseData data,
        Signed192 axisLength,
        bool positiveCap)
    {
        Signed320 scaleSquared = WideArithmetic.MultiplySigned192(
            DoubleParameterScale,
            DoubleParameterScale);
        Signed320 heightSquared = WideArithmetic.MultiplySigned192(axisLength, axisLength);
        Signed320 radiusSquared = WideArithmetic.MultiplySigned192(data.Radius, data.Radius);
        Signed320 expansionSquared = WideArithmetic.MultiplySigned192(data.Expansion, data.Expansion);
        Signed320 radiusDifference = WideArithmetic.SubtractSigned320(radiusSquared, expansionSquared);
        Signed320 constant = WideArithmetic.AddSigned320(
            MultiplyToSigned320(heightSquared, data.AxisLengthSquared),
            MultiplyToSigned320(radiusDifference, scaleSquared));

        Signed320 axial0 = MultiplyThreeToSigned320(
            DoubleParameterScale,
            axisLength,
            data.StartAxisProjection);
        Signed320 axial1 = MultiplyThreeToSigned320(
            DoubleParameterScale,
            axisLength,
            data.DirectionAxisProjection);
        if (positiveCap)
        {
            axial0 = WideArithmetic.SubtractSigned320(default, axial0);
            axial1 = WideArithmetic.SubtractSigned320(default, axial1);
        }

        Signed320 n0 = WideArithmetic.AddSigned320(
            WideArithmetic.AddSigned320(
                MultiplyToSigned320(scaleSquared, data.G0),
                Twice(axial0)),
            constant);
        Signed320 n1 = WideArithmetic.AddSigned320(
            Twice(MultiplyToSigned320(scaleSquared, data.G1)),
            Twice(axial1));
        Signed320 n2 = MultiplyToSigned320(scaleSquared, data.G2);

        Signed320 rimScale = Four(MultiplyToSigned320(
            MultiplyToSigned320(radiusSquared, scaleSquared),
            scaleSquared));
        Signed576 h0 = WideArithmetic.SubtractSigned576(
            WideArithmetic.MultiplySigned576(
                WideArithmetic.MultiplySigned320(n0, n0),
                data.AxisLengthSquared),
            WideArithmetic.MultiplySigned320(rimScale, data.Q0));
        Signed576 h1 = WideArithmetic.SubtractSigned576(
            Twice(WideArithmetic.MultiplySigned576(
                WideArithmetic.MultiplySigned320(n0, n1),
                data.AxisLengthSquared)),
            WideArithmetic.MultiplySigned320(rimScale, data.Q1));
        Signed576 h2 = WideArithmetic.SubtractSigned576(
            WideArithmetic.MultiplySigned576(
                WideArithmetic.AddSigned576(
                    WideArithmetic.MultiplySigned320(n1, n1),
                    Twice(WideArithmetic.MultiplySigned320(n0, n2))),
                data.AxisLengthSquared),
            WideArithmetic.MultiplySigned320(rimScale, data.Q2));
        Signed576 h3 = Twice(WideArithmetic.MultiplySigned576(
            WideArithmetic.MultiplySigned320(n1, n2),
            data.AxisLengthSquared));
        Signed576 h4 = WideArithmetic.MultiplySigned576(
            WideArithmetic.MultiplySigned320(n2, n2),
            data.AxisLengthSquared);
        return new RoundedCylinderTorusPolynomial(n0, n1, n2, h0, h1, h2, h3, h4);
    }

    private static Signed320 MultiplyToSigned320(Signed320 left, Signed192 right) =>
        Signed320.NarrowValue(WideArithmetic.MultiplySigned576(Signed576.ExtendValue(left), right));

    private static Signed320 MultiplyToSigned320(Signed320 left, Signed320 right) =>
        Signed320.NarrowValue(WideArithmetic.MultiplySigned320(left, right));

    private static Signed320 Twice(Signed320 value) => WideArithmetic.AddSigned320(value, value);

    private static Signed320 Four(Signed320 value) => Twice(Twice(value));

    private static Signed576 Twice(Signed576 value) => WideArithmetic.AddSigned576(value, value);

    private static Signed576 AddFive(
        Signed576 first,
        Signed576 second,
        Signed576 third,
        Signed576 fourth,
        Signed576 fifth) =>
        WideArithmetic.AddSigned576(
            WideArithmetic.AddSigned576(
                WideArithmetic.AddSigned576(first, second),
                WideArithmetic.AddSigned576(third, fourth)),
            fifth);

}
