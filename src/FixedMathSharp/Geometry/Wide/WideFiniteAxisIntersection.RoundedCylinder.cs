//=======================================================================
// WideFiniteAxisIntersection.RoundedCylinder.cs
//=======================================================================
// MIT License, Copyright (c) 2024–present David Oravsky (mrdav30)
// See LICENSE file in the project root for full license information.
//=======================================================================

namespace FixedMathSharp.Bounds;

internal static partial class WideFiniteAxisIntersection
{
    internal static bool TryGetSphericallyExpandedFiniteCylinderDistanceInterval(
        FixedSegment query,
        Vector3d center,
        Vector3d axisDirection,
        Fixed64 axisHalfLength,
        Fixed64 radius,
        Fixed64 sphericalExpansion,
        Fixed64 segmentLength,
        out Fixed64 entry,
        out Fixed64 exit,
        out bool startContained,
        out bool endContainedStrict) =>
        TryGetSphericallyExpandedFiniteCylinderDistanceInterval(
            query,
            center,
            axisDirection,
            axisHalfLength,
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
        Fixed64 axisHalfLength,
        Fixed64 radius,
        Fixed64 sphericalExpansion,
        Fixed64 segmentLength,
        out Fixed64 distance) =>
        TryGetSphericallyExpandedFiniteCylinderDistanceInterval(
            query,
            center,
            axisDirection,
            axisHalfLength,
            radius,
            sphericalExpansion,
            segmentLength,
            calculateExit: false,
            out distance,
            out _,
            out _,
            out _);

    private static bool TryGetSphericallyExpandedFiniteCylinderDistanceInterval(
        FixedSegment query,
        Vector3d center,
        Vector3d axisDirection,
        Fixed64 axisHalfLength,
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
                axisHalfLength,
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
                axisHalfLength,
                Fixed64.Zero,
                sphericalExpansion,
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
            axisHalfLength,
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
        Merge(
            TryGetRoundedCylinderCapCoreDistanceInterval(
                baseData,
                axisHalfLength,
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
                axisHalfLength,
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
                axisHalfLength,
                segmentLength,
                positiveCap: false))
        {
            MergeRoundedCylinderRimInterval(
                CreateRoundedCylinderTorusPolynomial(baseData, axisHalfLength, positiveCap: false),
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
                axisHalfLength,
                segmentLength,
                positiveCap: true))
        {
            MergeRoundedCylinderRimInterval(
                CreateRoundedCylinderTorusPolynomial(baseData, axisHalfLength, positiveCap: true),
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
        Fixed64 axisHalfLength,
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
            ParameterScale,
            data.StartAxisProjection);
        Signed320 scaledDirectionProjection = WideArithmetic.MultiplySigned192(
            ParameterScale,
            data.DirectionAxisProjection);
        Signed320 capProjection = WideArithmetic.MultiplySigned192(
            WideArithmetic.FromSignedRaw(axisHalfLength.m_rawValue),
            data.AxisLengthSquared);
        Signed320 z0 = positiveCap
            ? WideArithmetic.SubtractSigned320(scaledStartProjection, capProjection)
            : WideArithmetic.AddSigned320(scaledStartProjection, capProjection);
        Signed320 z1 = scaledDirectionProjection;
        Signed320 expansionSquared = WideArithmetic.MultiplySigned192(data.Expansion, data.Expansion);
        Signed320 scaleSquared = WideArithmetic.MultiplySigned192(ParameterScale, ParameterScale);
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
        Fixed64 axisHalfLength,
        Fixed64 segmentLength,
        bool positiveCap) =>
        TryGetRoundedCylinderCapCoreDistanceInterval(
            data,
            axisHalfLength,
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
            WideArithmetic.FromSignedRaw(radius.m_rawValue),
            WideArithmetic.FromSignedRaw(sphericalExpansion.m_rawValue));
    }

    private static RoundedCylinderTorusPolynomial CreateRoundedCylinderTorusPolynomial(
        RoundedCylinderBaseData data,
        Fixed64 axisHalfLength,
        bool positiveCap)
    {
        Signed192 halfLengthRaw = WideArithmetic.FromSignedRaw(axisHalfLength.m_rawValue);
        Signed320 scaleSquared = WideArithmetic.MultiplySigned192(ParameterScale, ParameterScale);
        Signed320 heightSquared = WideArithmetic.MultiplySigned192(halfLengthRaw, halfLengthRaw);
        Signed320 radiusSquared = WideArithmetic.MultiplySigned192(data.Radius, data.Radius);
        Signed320 expansionSquared = WideArithmetic.MultiplySigned192(data.Expansion, data.Expansion);
        Signed320 radiusDifference = WideArithmetic.SubtractSigned320(radiusSquared, expansionSquared);
        Signed320 constant = WideArithmetic.AddSigned320(
            MultiplyToSigned320(heightSquared, data.AxisLengthSquared),
            MultiplyToSigned320(radiusDifference, scaleSquared));

        Signed320 axial0 = MultiplyThreeToSigned320(
            ParameterScale,
            halfLengthRaw,
            data.StartAxisProjection);
        Signed320 axial1 = MultiplyThreeToSigned320(
            ParameterScale,
            halfLengthRaw,
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
            MultiplyByAxisLength(WideArithmetic.MultiplySigned320(n0, n0), data.AxisLengthSquared),
            WideArithmetic.MultiplySigned320(rimScale, data.Q0));
        Signed576 h1 = WideArithmetic.SubtractSigned576(
            Twice(MultiplyByAxisLength(WideArithmetic.MultiplySigned320(n0, n1), data.AxisLengthSquared)),
            WideArithmetic.MultiplySigned320(rimScale, data.Q1));
        Signed576 h2 = WideArithmetic.SubtractSigned576(
            MultiplyByAxisLength(
                WideArithmetic.AddSigned576(
                    WideArithmetic.MultiplySigned320(n1, n1),
                    Twice(WideArithmetic.MultiplySigned320(n0, n2))),
                data.AxisLengthSquared),
            WideArithmetic.MultiplySigned320(rimScale, data.Q2));
        Signed576 h3 = Twice(MultiplyByAxisLength(
            WideArithmetic.MultiplySigned320(n1, n2),
            data.AxisLengthSquared));
        Signed576 h4 = MultiplyByAxisLength(
            WideArithmetic.MultiplySigned320(n2, n2),
            data.AxisLengthSquared);
        return new RoundedCylinderTorusPolynomial(n0, n1, n2, h0, h1, h2, h3, h4);
    }

    private static Signed576 MultiplyByAxisLength(Signed576 value, Signed192 axisLengthSquared) =>
        WideArithmetic.MultiplySigned576(value, axisLengthSquared);

    private static Signed320 MultiplyToSigned320(Signed320 left, Signed192 right) =>
        Narrow(WideArithmetic.MultiplySigned576(WideArithmetic.ExtendToSigned576(left), right));

    private static Signed320 MultiplyToSigned320(Signed320 left, Signed320 right) =>
        Narrow(WideArithmetic.MultiplySigned320(left, right));

    private static Signed320 Narrow(Signed576 value)
        => new(value.Word4, value.Word3, value.Word2, value.Word1, value.Word0);

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
            WideArithmetic.ExtendToSigned320(WideFiniteAxisIntersection.One),
            default,
            0);
        internal static readonly RoundedCylinderQuadraticBound One = new(
            WideArithmetic.ExtendToSigned320(WideFiniteAxisIntersection.One),
            WideArithmetic.ExtendToSigned320(WideFiniteAxisIntersection.One),
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
}
