//=======================================================================
// WideOrientedBox.RigidFiniteAxisQuery.cs
//=======================================================================
// MIT License, Copyright (c) 2024–present David Oravsky (mrdav30)
// See LICENSE file in the project root for full license information.
//=======================================================================

namespace FixedMathSharp.Geometry;

/// <content>
/// Rigid-transform (no-scale) finite axis intersection queries for oriented boxes,
/// including capsule distance interval and axial bounds computations.
/// </content>
internal static partial class WideOrientedBox
{
    #region Nested Types

    private readonly struct RigidLocalRay
    {
        internal readonly Signed192 StartX;
        internal readonly Signed192 StartY;
        internal readonly Signed192 StartZ;
        internal readonly Signed192 EndX;
        internal readonly Signed192 EndY;
        internal readonly Signed192 EndZ;
        internal readonly Signed192 DeltaX;
        internal readonly Signed192 DeltaY;
        internal readonly Signed192 DeltaZ;
        internal readonly Signed192 Denominator;

        internal RigidLocalRay(
            Signed192 startX,
            Signed192 startY,
            Signed192 startZ,
            Signed192 endX,
            Signed192 endY,
            Signed192 endZ,
            Signed192 denominator)
        {
            StartX = startX;
            StartY = startY;
            StartZ = startZ;
            EndX = endX;
            EndY = endY;
            EndZ = endZ;
            DeltaX = WideArithmetic.SubtractSigned192(endX, startX);
            DeltaY = WideArithmetic.SubtractSigned192(endY, startY);
            DeltaZ = WideArithmetic.SubtractSigned192(endZ, startZ);
            Denominator = denominator;
        }
    }

    private readonly struct RigidRayBound
    {
        internal readonly Signed192 Numerator;
        internal readonly Signed192 Denominator;

        internal RigidRayBound(
            Signed192 numerator,
            Signed192 denominator)
        {
            Numerator = numerator;
            Denominator = denominator;
        }
    }

    #endregion

    internal static bool TryGetRigidCapsuleDistanceInterval(
        FixedSegment query,
        Vector3d center,
        FixedQuaternion rotation,
        Fixed64 axisLength,
        Fixed64 radius,
        Fixed64 radiusExpansion,
        Fixed64 totalDistance,
        out Fixed64 entry,
        out Fixed64 exit,
        out bool startContained,
        out bool endContainedStrict)
    {
        WideRationalBasis3d basis = new(rotation);
        if (TryGetCardinalLocalYAxis(basis, out Vector3d axisDirection))
        {
            return WideFiniteAxisIntersection.TryGetCapsuleDistanceInterval(
                query,
                center,
                axisDirection,
                axisLength,
                radius,
                radiusExpansion,
                totalDistance,
                out entry,
                out exit,
                out startContained,
                out endContainedStrict);
        }

        RigidLocalRay ray = GetRigidLocalRay(query, center, basis);
        Signed192 length = ScaleRigidDimension(axisLength, ray.Denominator);
        Signed192 expandedRadius = ScaleRigidDimensionSum(
            radius,
            radiusExpansion,
            ray.Denominator);
        startContained = IsCapsuleContained(
            ray.StartX,
            ray.StartY,
            ray.StartZ,
            length,
            expandedRadius,
            strict: false);
        endContainedStrict = IsCapsuleContained(
            ray.EndX,
            ray.EndY,
            ray.EndZ,
            length,
            expandedRadius,
            strict: true);

        bool found = false;
        entry = default;
        exit = default;
        if (TryGetCenteredAxialBounds(
                ray.StartY,
                ray.DeltaY,
                length,
                out RigidRayBound lower,
                out RigidRayBound upper))
        {
            GetRadialPolynomial(
                ray.StartX,
                ray.StartZ,
                ray.DeltaX,
                ray.DeltaZ,
                expandedRadius,
                out Signed576 coefficient,
                out Signed576 projection,
                out Signed576 constant);
            MergeRigidInterval(
                WideFiniteConeIntersection.TrySolveBoundedUnitPolynomial(
                    coefficient,
                    projection,
                    constant,
                    lower.Numerator,
                    lower.Denominator,
                    upper.Numerator,
                    upper.Denominator,
                    totalDistance,
                    out Fixed64 sideEntry,
                    out Fixed64 sideExit),
                sideEntry,
                sideExit,
                ref found,
                ref entry,
                ref exit);
        }

        MergeRigidCapsuleCap(
            ray,
            length,
            expandedRadius,
            positive: false,
            totalDistance,
            ref found,
            ref entry,
            ref exit);
        MergeRigidCapsuleCap(
            ray,
            length,
            expandedRadius,
            positive: true,
            totalDistance,
            ref found,
            ref entry,
            ref exit);
        return found;
    }

    internal static bool TryGetRigidCylinderDistanceInterval(
        FixedSegment query,
        Vector3d center,
        FixedQuaternion rotation,
        Fixed64 axisLength,
        Fixed64 radius,
        Fixed64 radiusExpansion,
        Fixed64 axialExpansion,
        Fixed64 totalDistance,
        out Fixed64 entry,
        out Fixed64 exit,
        out bool startContained,
        out bool endContainedStrict)
    {
        WideRationalBasis3d basis = new(rotation);
        if (TryGetCardinalLocalYAxis(basis, out Vector3d axisDirection))
        {
            return WideFiniteAxisIntersection.TryGetFiniteCylinderDistanceInterval(
                query,
                center,
                axisDirection,
                axisLength,
                radius,
                radiusExpansion,
                axialExpansion,
                totalDistance,
                out entry,
                out exit,
                out startContained,
                out endContainedStrict);
        }

        RigidLocalRay ray = GetRigidLocalRay(query, center, basis);
        Signed192 length = ScaleRigidDimensionWithDoubleExpansion(
            axisLength,
            axialExpansion,
            ray.Denominator);
        Signed192 expandedRadius = ScaleRigidDimensionSum(
            radius,
            radiusExpansion,
            ray.Denominator);
        startContained = IsCylinderContained(
            ray.StartX,
            ray.StartY,
            ray.StartZ,
            length,
            expandedRadius,
            strict: false);
        endContainedStrict = IsCylinderContained(
            ray.EndX,
            ray.EndY,
            ray.EndZ,
            length,
            expandedRadius,
            strict: true);

        if (!TryGetCenteredAxialBounds(
                ray.StartY,
                ray.DeltaY,
                length,
                out RigidRayBound lower,
                out RigidRayBound upper))
        {
            entry = default;
            exit = default;
            return false;
        }

        GetRadialPolynomial(
            ray.StartX,
            ray.StartZ,
            ray.DeltaX,
            ray.DeltaZ,
            expandedRadius,
            out Signed576 coefficient,
            out Signed576 projection,
            out Signed576 constant);
        return WideFiniteConeIntersection.TrySolveBoundedUnitPolynomial(
            coefficient,
            projection,
            constant,
            lower.Numerator,
            lower.Denominator,
            upper.Numerator,
            upper.Denominator,
            totalDistance,
            out entry,
            out exit);
    }

    internal static bool TryGetRigidConeDistanceInterval(
        FixedSegment query,
        Vector3d center,
        FixedQuaternion rotation,
        Fixed64 height,
        Fixed64 radius,
        Fixed64 totalDistance,
        out Fixed64 entry,
        out Fixed64 exit,
        out bool startContained,
        out bool endContainedStrict)
    {
        WideRationalBasis3d basis = new(rotation);
        if (TryGetCardinalLocalYAxis(basis, out Vector3d axisDirection))
        {
            return WideFiniteConeIntersection.TryGetCenteredDistanceInterval(
                query,
                center,
                axisDirection,
                height,
                radius,
                totalDistance,
                out entry,
                out exit,
                out startContained,
                out endContainedStrict);
        }

        RigidLocalRay ray = GetRigidLocalRay(query, center, basis);
        GetConePolynomial(
            ray,
            Signed192.Raw(height),
            Signed192.Raw(radius),
            out Signed192 startAxial,
            out Signed192 axialVelocity,
            out Signed192 maximumAxial,
            out Signed576 coefficient,
            out Signed576 projection,
            out Signed576 constant);
        startContained = IsConeContained(
            startAxial,
            maximumAxial,
            constant.Sign,
            strict: false);
        Signed192 endAxial =
            WideArithmetic.AddSigned192(startAxial, axialVelocity);
        endContainedStrict = IsConeContained(
            endAxial,
            maximumAxial,
            EvaluatePolynomialAtOne(
                coefficient,
                projection,
                constant).Sign,
            strict: true);

        if (!TryGetLinearBounds(
                startAxial,
                axialVelocity,
                default,
                maximumAxial,
                out RigidRayBound lower,
                out RigidRayBound upper))
        {
            entry = default;
            exit = default;
            return false;
        }

        return WideFiniteConeIntersection.TrySolveBoundedUnitPolynomial(
            coefficient,
            projection,
            constant,
            lower.Numerator,
            lower.Denominator,
            upper.Numerator,
            upper.Denominator,
            totalDistance,
            out entry,
            out exit);
    }

    private static RigidLocalRay GetRigidLocalRay(
        FixedSegment query,
        Vector3d center,
        WideRationalBasis3d basis)
    {
        GetRelativeLocalPointNumerators(
            query.Start,
            center,
            basis,
            out Signed192 startX,
            out Signed192 startY,
            out Signed192 startZ);
        GetRelativeLocalPointNumerators(
            query.End,
            center,
            basis,
            out Signed192 endX,
            out Signed192 endY,
            out Signed192 endZ);
        return new RigidLocalRay(
            startX,
            startY,
            startZ,
            endX,
            endY,
            endZ,
            basis.Denominator);
    }

    private static bool TryGetCardinalLocalYAxis(
        WideRationalBasis3d basis,
        out Vector3d axisDirection)
    {
        if (basis.Yy.IsZero
            && basis.Yz.IsZero
            && WideArithmetic.CompareMagnitude(basis.Yx, basis.Denominator) == 0)
        {
            axisDirection = basis.Yx.Sign > 0 ? Vector3d.Right : Vector3d.Left;
            return true;
        }

        if (basis.Yx.IsZero
            && basis.Yz.IsZero
            && WideArithmetic.CompareMagnitude(basis.Yy, basis.Denominator) == 0)
        {
            axisDirection = basis.Yy.Sign > 0 ? Vector3d.Up : Vector3d.Down;
            return true;
        }

        if (basis.Yx.IsZero
            && basis.Yy.IsZero
            && WideArithmetic.CompareMagnitude(basis.Yz, basis.Denominator) == 0)
        {
            axisDirection = basis.Yz.Sign > 0 ? Vector3d.Forward : Vector3d.Backward;
            return true;
        }

        axisDirection = default;
        return false;
    }

    private static Signed192 ScaleRigidDimension(
        Fixed64 value,
        Signed192 denominator) =>
        NarrowRigidQueryValue(
            WideArithmetic.MultiplySigned192(
                Signed192.Raw(value),
                denominator));

    private static Signed192 ScaleRigidDimensionSum(
        Fixed64 first,
        Fixed64 second,
        Signed192 denominator) =>
        NarrowRigidQueryValue(
            WideArithmetic.MultiplySigned192(
                WideArithmetic.AddSigned192(
                    Signed192.Raw(first),
                    Signed192.Raw(second)),
                denominator));

    private static Signed192 ScaleRigidDimensionWithDoubleExpansion(
        Fixed64 value,
        Fixed64 expansion,
        Signed192 denominator)
    {
        Signed192 expanded = WideArithmetic.AddSigned192(
            Signed192.Raw(value),
            WideArithmetic.AddSigned192(
                Signed192.Raw(expansion),
                Signed192.Raw(expansion)));
        return NarrowRigidQueryValue(
            WideArithmetic.MultiplySigned192(expanded, denominator));
    }

    private static Signed192 NarrowRigidQueryValue(Signed320 value)
    {
        _ = Signed192.TryNarrowSigned(value, out Signed192 result);
        return result;
    }

    private static void GetRadialPolynomial(
        Signed192 startX,
        Signed192 startZ,
        Signed192 deltaX,
        Signed192 deltaZ,
        Signed192 radius,
        out Signed576 coefficient,
        out Signed576 projection,
        out Signed576 constant)
    {
        Signed320 coefficientNarrow = AddRigidSquares(
            deltaX,
            deltaZ);
        Signed320 projectionNarrow = AddRigidProducts(
            startX,
            deltaX,
            startZ,
            deltaZ);
        Signed320 constantNarrow = WideArithmetic.SubtractSigned320(
            AddRigidSquares(startX, startZ),
            WideArithmetic.MultiplySigned192(radius, radius));
        coefficient =
            Signed576.ExtendValue(coefficientNarrow);
        projection =
            Signed576.ExtendValue(projectionNarrow);
        constant =
            Signed576.ExtendValue(constantNarrow);
    }

    private static void MergeRigidCapsuleCap(
        RigidLocalRay ray,
        Signed192 length,
        Signed192 radius,
        bool positive,
        Fixed64 totalDistance,
        ref bool found,
        ref Fixed64 entry,
        ref Fixed64 exit)
    {
        Signed192 startX = WideArithmetic.Double(ray.StartX);
        Signed192 startY = positive
            ? WideArithmetic.SubtractSigned192(
                WideArithmetic.Double(ray.StartY),
                length)
            : WideArithmetic.AddSigned192(
                WideArithmetic.Double(ray.StartY),
                length);
        Signed192 startZ = WideArithmetic.Double(ray.StartZ);
        Signed192 deltaX = WideArithmetic.Double(ray.DeltaX);
        Signed192 deltaY = WideArithmetic.Double(ray.DeltaY);
        Signed192 deltaZ = WideArithmetic.Double(ray.DeltaZ);
        Signed192 doubledRadius = WideArithmetic.Double(radius);
        GetSpherePolynomial(
            startX,
            startY,
            startZ,
            deltaX,
            deltaY,
            deltaZ,
            doubledRadius,
            out Signed576 coefficient,
            out Signed576 projection,
            out Signed576 constant);
        MergeRigidInterval(
            WideFiniteConeIntersection.TrySolveUnitPolynomial(
                coefficient,
                projection,
                constant,
                totalDistance,
                out Fixed64 capEntry,
                out Fixed64 capExit),
            capEntry,
            capExit,
            ref found,
            ref entry,
            ref exit);
    }

    private static void GetSpherePolynomial(
        Signed192 startX,
        Signed192 startY,
        Signed192 startZ,
        Signed192 deltaX,
        Signed192 deltaY,
        Signed192 deltaZ,
        Signed192 radius,
        out Signed576 coefficient,
        out Signed576 projection,
        out Signed576 constant)
    {
        coefficient = Signed576.ExtendValue(
            AddRigidSquares(deltaX, deltaY, deltaZ));
        projection = Signed576.ExtendValue(
            AddRigidProducts(
                startX,
                deltaX,
                startY,
                deltaY,
                startZ,
                deltaZ));
        constant = Signed576.ExtendValue(
            WideArithmetic.SubtractSigned320(
                AddRigidSquares(startX, startY, startZ),
                WideArithmetic.MultiplySigned192(radius, radius)));
    }

    private static void GetConePolynomial(
        RigidLocalRay ray,
        Signed192 height,
        Signed192 radius,
        out Signed192 startAxial,
        out Signed192 axialVelocity,
        out Signed192 maximumAxial,
        out Signed576 coefficient,
        out Signed576 projection,
        out Signed576 constant)
    {
        Signed192 doubledStartX = WideArithmetic.Double(ray.StartX);
        Signed192 doubledStartZ = WideArithmetic.Double(ray.StartZ);
        Signed192 doubledDeltaX = WideArithmetic.Double(ray.DeltaX);
        Signed192 doubledDeltaZ = WideArithmetic.Double(ray.DeltaZ);
        Signed192 scaledHeight = NarrowRigidQueryValue(
            WideArithmetic.MultiplySigned192(
                height,
                ray.Denominator));
        startAxial = WideArithmetic.SubtractSigned192(
            scaledHeight,
            WideArithmetic.Double(ray.StartY));
        axialVelocity =
            WideArithmetic.SubtractSigned192(
                default,
                WideArithmetic.Double(ray.DeltaY));
        maximumAxial = WideArithmetic.Double(scaledHeight);

        Signed320 heightSquared =
            WideArithmetic.MultiplySigned192(height, height);
        Signed320 radiusSquared =
            WideArithmetic.MultiplySigned192(radius, radius);
        Signed320 radialCoefficient =
            AddRigidSquares(doubledDeltaX, doubledDeltaZ);
        Signed320 radialProjection =
            AddRigidProducts(
                doubledStartX,
                doubledDeltaX,
                doubledStartZ,
                doubledDeltaZ);
        Signed320 radialConstant =
            AddRigidSquares(doubledStartX, doubledStartZ);
        // The local point numerators already carry one quaternion denominator.
        // Keeping height and radius unscaled cancels the common positive D²
        // factor before the quadratic is formed. Under IsNormalized, D has at
        // most 65 magnitude bits, each rigid local numerator has fewer than
        // 130, and these factored coefficients have fewer than 390.
        coefficient = WideArithmetic.SubtractSigned576(
            WideArithmetic.MultiplySigned320(
                heightSquared,
                radialCoefficient),
            WideArithmetic.MultiplySigned320(
                radiusSquared,
                WideArithmetic.MultiplySigned192(
                    axialVelocity,
                    axialVelocity)));
        projection = WideArithmetic.SubtractSigned576(
            WideArithmetic.MultiplySigned320(
                heightSquared,
                radialProjection),
            WideArithmetic.MultiplySigned320(
                radiusSquared,
                WideArithmetic.MultiplySigned192(
                    startAxial,
                    axialVelocity)));
        constant = WideArithmetic.SubtractSigned576(
            WideArithmetic.MultiplySigned320(
                heightSquared,
                radialConstant),
            WideArithmetic.MultiplySigned320(
                radiusSquared,
                WideArithmetic.MultiplySigned192(
                    startAxial,
                    startAxial)));
    }

    private static bool TryGetCenteredAxialBounds(
        Signed192 startY,
        Signed192 deltaY,
        Signed192 length,
        out RigidRayBound lower,
        out RigidRayBound upper) =>
        TryGetLinearBounds(
            WideArithmetic.Double(startY),
            WideArithmetic.Double(deltaY),
            WideArithmetic.SubtractSigned192(default, length),
            length,
            out lower,
            out upper);

    private static bool TryGetLinearBounds(
        Signed192 start,
        Signed192 delta,
        Signed192 minimum,
        Signed192 maximum,
        out RigidRayBound lower,
        out RigidRayBound upper)
    {
        Signed192 one = Signed192.Signed(1L);
        lower = new RigidRayBound(default, one);
        upper = new RigidRayBound(one, one);
        if (delta.IsZero)
        {
            return WideArithmetic.SubtractSigned192(
                    start,
                    minimum).Sign >= 0
                && WideArithmetic.SubtractSigned192(
                    start,
                    maximum).Sign <= 0;
        }

        RigidRayBound first = NormalizeRigidBound(
            WideArithmetic.SubtractSigned192(minimum, start),
            delta);
        RigidRayBound second = NormalizeRigidBound(
            WideArithmetic.SubtractSigned192(maximum, start),
            delta);
        if (CompareRigidBounds(first, second) > 0)
            (first, second) = (second, first);
        if (CompareRigidBounds(second, lower) < 0
            || CompareRigidBounds(first, upper) > 0)
        {
            return false;
        }

        if (CompareRigidBounds(first, lower) > 0)
            lower = first;
        if (CompareRigidBounds(second, upper) < 0)
            upper = second;
        return CompareRigidBounds(lower, upper) <= 0;
    }

    private static RigidRayBound NormalizeRigidBound(
        Signed192 numerator,
        Signed192 denominator)
    {
        if (denominator.Sign >= 0)
            return new RigidRayBound(numerator, denominator);

        return new RigidRayBound(
            WideArithmetic.SubtractSigned192(default, numerator),
            WideArithmetic.SubtractSigned192(default, denominator));
    }

    private static int CompareRigidBounds(
        RigidRayBound left,
        RigidRayBound right) =>
        WideArithmetic.MultiplySubtract(
            left.Numerator,
            right.Denominator,
            right.Numerator,
            left.Denominator).Sign;

    private static bool IsCylinderContained(
        Signed192 x,
        Signed192 y,
        Signed192 z,
        Signed192 length,
        Signed192 radius,
        bool strict)
    {
        int lower = WideArithmetic.AddSigned192(
            WideArithmetic.Double(y),
            length).Sign;
        int upper = WideArithmetic.SubtractSigned192(
            WideArithmetic.Double(y),
            length).Sign;
        int radial = WideArithmetic.SubtractSigned320(
            AddRigidSquares(x, z),
            WideArithmetic.MultiplySigned192(radius, radius)).Sign;
        return strict
            ? lower > 0 && upper < 0 && radial < 0
            : lower >= 0 && upper <= 0 && radial <= 0;
    }

    private static bool IsCapsuleContained(
        Signed192 x,
        Signed192 y,
        Signed192 z,
        Signed192 length,
        Signed192 radius,
        bool strict)
    {
        if (IsCylinderContained(x, y, z, length, radius, strict))
            return true;

        Signed192 doubledX = WideArithmetic.Double(x);
        Signed192 doubledY = WideArithmetic.Double(y);
        Signed192 doubledZ = WideArithmetic.Double(z);
        Signed192 doubledRadius = WideArithmetic.Double(radius);
        Signed320 radiusSquared =
            WideArithmetic.MultiplySigned192(
                doubledRadius,
                doubledRadius);
        Signed320 negativeCap = AddRigidSquares(
            doubledX,
            WideArithmetic.AddSigned192(doubledY, length),
            doubledZ);
        Signed320 positiveCap = AddRigidSquares(
            doubledX,
            WideArithmetic.SubtractSigned192(doubledY, length),
            doubledZ);
        int negative = WideArithmetic.SubtractSigned320(
            negativeCap,
            radiusSquared).Sign;
        int positive = WideArithmetic.SubtractSigned320(
            positiveCap,
            radiusSquared).Sign;
        return strict
            ? negative < 0 || positive < 0
            : negative <= 0 || positive <= 0;
    }

    private static bool IsConeContained(
        Signed192 axial,
        Signed192 maximumAxial,
        int polynomialSign,
        bool strict)
    {
        int maximum =
            WideArithmetic.SubtractSigned192(
                axial,
                maximumAxial).Sign;
        return strict
            ? axial.Sign > 0 && maximum < 0 && polynomialSign < 0
            : axial.Sign >= 0 && maximum <= 0 && polynomialSign <= 0;
    }

    private static Signed704 EvaluatePolynomialAtOne(
        Signed576 coefficient,
        Signed576 projection,
        Signed576 constant) =>
        WideArithmetic.AddSigned704(
            WideArithmetic.AddSigned704(
                Signed704.ExtendValue(coefficient),
                WideArithmetic.AddSigned704(
                    Signed704.ExtendValue(projection),
                    Signed704.ExtendValue(projection))),
            Signed704.ExtendValue(constant));

    private static Signed320 AddRigidSquares(
        Signed192 first,
        Signed192 second) =>
        WideArithmetic.AddSigned320(
            WideArithmetic.MultiplySigned192(first, first),
            WideArithmetic.MultiplySigned192(second, second));

    private static Signed320 AddRigidSquares(
        Signed192 first,
        Signed192 second,
        Signed192 third) =>
        WideArithmetic.AddSigned320(
            AddRigidSquares(first, second),
            WideArithmetic.MultiplySigned192(third, third));

    private static Signed320 AddRigidProducts(
        Signed192 firstLeft,
        Signed192 firstRight,
        Signed192 secondLeft,
        Signed192 secondRight) =>
        WideArithmetic.AddSigned320(
            WideArithmetic.MultiplySigned192(
                firstLeft,
                firstRight),
            WideArithmetic.MultiplySigned192(
                secondLeft,
                secondRight));

    private static Signed320 AddRigidProducts(
        Signed192 firstLeft,
        Signed192 firstRight,
        Signed192 secondLeft,
        Signed192 secondRight,
        Signed192 thirdLeft,
        Signed192 thirdRight) =>
        WideArithmetic.AddSigned320(
            AddRigidProducts(
                firstLeft,
                firstRight,
                secondLeft,
                secondRight),
            WideArithmetic.MultiplySigned192(
                thirdLeft,
                thirdRight));

    private static void MergeRigidInterval(
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
}
