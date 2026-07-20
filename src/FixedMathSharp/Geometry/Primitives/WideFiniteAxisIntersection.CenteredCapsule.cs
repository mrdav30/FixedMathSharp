//=======================================================================
// WideFiniteAxisIntersection.CenteredCapsule.cs
//=======================================================================
// MIT License, Copyright (c) 2024–present David Oravsky (mrdav30)
// See LICENSE file in the project root for full license information.
//=======================================================================

namespace FixedMathSharp.Bounds;

internal static partial class WideFiniteAxisIntersection
{
    internal static bool TryGetSurfacePointOnCenteredCapsule(
        Vector2d point,
        Vector2d center,
        Vector2d axisDirection,
        Fixed64 axisHalfLength,
        Fixed64 radius,
        Vector2d normal,
        out Vector2d surfacePoint)
    {
        GetClosestCenteredAxisRatio(
            point,
            center,
            axisDirection,
            axisHalfLength,
            out Signed192 numerator,
            out Signed192 denominator);
        if (!TryGetCenteredCapsuleSurfaceCoordinate(
                center.X, axisDirection.X, numerator, denominator, normal.X, radius, out Fixed64 x)
            || !TryGetCenteredCapsuleSurfaceCoordinate(
                center.Y, axisDirection.Y, numerator, denominator, normal.Y, radius, out Fixed64 y))
        {
            surfacePoint = default;
            return false;
        }

        surfacePoint = new Vector2d(x, y);
        return true;
    }

    internal static bool TryGetSurfacePointOnCenteredCapsule(
        Vector3d point,
        Vector3d center,
        Vector3d axisDirection,
        Fixed64 axisHalfLength,
        Fixed64 radius,
        Vector3d normal,
        out Vector3d surfacePoint)
    {
        GetClosestCenteredAxisRatio(
            point,
            center,
            axisDirection,
            axisHalfLength,
            out Signed192 numerator,
            out Signed192 denominator);
        if (!TryGetCenteredCapsuleSurfaceCoordinate(
                center.X, axisDirection.X, numerator, denominator, normal.X, radius, out Fixed64 x)
            || !TryGetCenteredCapsuleSurfaceCoordinate(
                center.Y, axisDirection.Y, numerator, denominator, normal.Y, radius, out Fixed64 y)
            || !TryGetCenteredCapsuleSurfaceCoordinate(
                center.Z, axisDirection.Z, numerator, denominator, normal.Z, radius, out Fixed64 z))
        {
            surfacePoint = default;
            return false;
        }

        surfacePoint = new Vector3d(x, y, z);
        return true;
    }

    internal static bool ContainsPointInCenteredCapsule(
        Vector2d point,
        Vector2d center,
        Vector2d axisDirection,
        Fixed64 axisHalfLength,
        Fixed64 radius,
        Fixed64 radiusExpansion,
        bool strict)
    {
        Signed192 expandedRadius = GetExpandedRadius(radius, radiusExpansion);
        Signed192 axisLengthSquared = GetDot(axisDirection, Vector2d.Zero, axisDirection, Vector2d.Zero);
        Signed192 axisProjection = GetDot(point, center, axisDirection, Vector2d.Zero);
        return IsCenteredCapsulePointContained(
            point,
            center,
            axisDirection,
            axisHalfLength,
            GetDot(point, center, point, center),
            axisProjection,
            axisLengthSquared,
            GetSquaredRadius(expandedRadius),
            expandedRadius,
            GetCenteredAxialExtent(axisLengthSquared, axisHalfLength, Fixed64.Zero),
            strict);
    }

    internal static bool ContainsPointInCenteredCapsule(
        Vector3d point,
        Vector3d center,
        Vector3d axisDirection,
        Fixed64 axisHalfLength,
        Fixed64 radius,
        Fixed64 radiusExpansion,
        bool strict)
    {
        Signed192 expandedRadius = GetExpandedRadius(radius, radiusExpansion);
        Signed192 axisLengthSquared = GetDot(axisDirection, Vector3d.Zero, axisDirection, Vector3d.Zero);
        Signed192 axisProjection = GetDot(point, center, axisDirection, Vector3d.Zero);
        return IsCenteredCapsulePointContained(
            point,
            center,
            axisDirection,
            axisHalfLength,
            GetDot(point, center, point, center),
            axisProjection,
            axisLengthSquared,
            GetSquaredRadius(expandedRadius),
            expandedRadius,
            GetCenteredAxialExtent(axisLengthSquared, axisHalfLength, Fixed64.Zero),
            strict);
    }

    internal static Vector2d GetDirectionFromCenteredAxis(
        Vector2d point,
        Vector2d center,
        Vector2d axisDirection,
        Fixed64 axisHalfLength)
    {
        Signed192 differenceX = GetComponentDifference(point.X, center.X);
        Signed192 differenceY = GetComponentDifference(point.Y, center.Y);
        Signed192 axisLengthSquared = GetDot(axisDirection, Vector2d.Zero, axisDirection, Vector2d.Zero);
        Signed192 axisProjection = GetDot(point, center, axisDirection, Vector2d.Zero);
        int cap = GetCenteredCap(
            axisProjection,
            GetCenteredAxialExtent(axisLengthSquared, axisHalfLength, Fixed64.Zero));
        if (cap != 0)
        {
            return WideGeometry.GetNormalized(
                GetCenteredCapOffset(point.X, center.X, axisDirection.X, axisHalfLength, cap > 0),
                GetCenteredCapOffset(point.Y, center.Y, axisDirection.Y, axisHalfLength, cap > 0));
        }

        Signed192 axisX = WideArithmetic.FromSignedRaw(axisDirection.X.m_rawValue);
        Signed192 axisY = WideArithmetic.FromSignedRaw(axisDirection.Y.m_rawValue);
        return WideGeometry.GetNormalized(
            GetRadialComponent(axisLengthSquared, differenceX, axisX, axisProjection),
            GetRadialComponent(axisLengthSquared, differenceY, axisY, axisProjection));
    }

    internal static Vector3d GetDirectionFromCenteredAxis(
        Vector3d point,
        Vector3d center,
        Vector3d axisDirection,
        Fixed64 axisHalfLength)
    {
        Signed192 differenceX = GetComponentDifference(point.X, center.X);
        Signed192 differenceY = GetComponentDifference(point.Y, center.Y);
        Signed192 differenceZ = GetComponentDifference(point.Z, center.Z);
        Signed192 axisLengthSquared = GetDot(axisDirection, Vector3d.Zero, axisDirection, Vector3d.Zero);
        Signed192 axisProjection = GetDot(point, center, axisDirection, Vector3d.Zero);
        int cap = GetCenteredCap(
            axisProjection,
            GetCenteredAxialExtent(axisLengthSquared, axisHalfLength, Fixed64.Zero));
        if (cap != 0)
        {
            return WideGeometry.GetNormalized(
                GetCenteredCapOffset(point.X, center.X, axisDirection.X, axisHalfLength, cap > 0),
                GetCenteredCapOffset(point.Y, center.Y, axisDirection.Y, axisHalfLength, cap > 0),
                GetCenteredCapOffset(point.Z, center.Z, axisDirection.Z, axisHalfLength, cap > 0));
        }

        Signed192 axisX = WideArithmetic.FromSignedRaw(axisDirection.X.m_rawValue);
        Signed192 axisY = WideArithmetic.FromSignedRaw(axisDirection.Y.m_rawValue);
        Signed192 axisZ = WideArithmetic.FromSignedRaw(axisDirection.Z.m_rawValue);
        return WideGeometry.GetNormalized(
            GetRadialComponent(axisLengthSquared, differenceX, axisX, axisProjection),
            GetRadialComponent(axisLengthSquared, differenceY, axisY, axisProjection),
            GetRadialComponent(axisLengthSquared, differenceZ, axisZ, axisProjection));
    }

    internal static bool TryGetCapsuleInterval(
        FixedSegment2d query,
        Vector2d center,
        Vector2d axisDirection,
        Fixed64 axisHalfLength,
        Fixed64 radius,
        Fixed64 radiusExpansion,
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
        Signed320 axialExtent = GetCenteredAxialExtent(
            axisLengthSquared,
            axisHalfLength,
            Fixed64.Zero);

        startContained = IsCenteredCapsulePointContained(
            query.Start,
            center,
            axisDirection,
            axisHalfLength,
            startDistanceSquared,
            startAxisProjection,
            axisLengthSquared,
            squaredRadius,
            expandedRadius,
            axialExtent,
            strict: false);
        endContainedStrict = IsCenteredCapsulePointContained(
            query.End,
            center,
            axisDirection,
            axisHalfLength,
            GetDot(query.End, center, query.End, center),
            endAxisProjection,
            axisLengthSquared,
            squaredRadius,
            expandedRadius,
            axialExtent,
            strict: true);

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
                Fixed64.One,
                out entry,
                out exit);
        }

        MergeCenteredCapInterval(
            query,
            center,
            axisDirection,
            axisHalfLength,
            expandedRadius,
            positiveCap: false,
            ref found,
            ref entry,
            ref exit);
        MergeCenteredCapInterval(
            query,
            center,
            axisDirection,
            axisHalfLength,
            expandedRadius,
            positiveCap: true,
            ref found,
            ref entry,
            ref exit);
        return found;
    }

    internal static bool TryGetCapsuleInterval(
        FixedSegment query,
        Vector3d center,
        Vector3d axisDirection,
        Fixed64 axisHalfLength,
        Fixed64 radius,
        Fixed64 radiusExpansion,
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
        Signed320 axialExtent = GetCenteredAxialExtent(
            axisLengthSquared,
            axisHalfLength,
            Fixed64.Zero);

        startContained = IsCenteredCapsulePointContained(
            query.Start,
            center,
            axisDirection,
            axisHalfLength,
            startDistanceSquared,
            startAxisProjection,
            axisLengthSquared,
            squaredRadius,
            expandedRadius,
            axialExtent,
            strict: false);
        endContainedStrict = IsCenteredCapsulePointContained(
            query.End,
            center,
            axisDirection,
            axisHalfLength,
            GetDot(query.End, center, query.End, center),
            endAxisProjection,
            axisLengthSquared,
            squaredRadius,
            expandedRadius,
            axialExtent,
            strict: true);

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
                Fixed64.One,
                out entry,
                out exit);
        }

        MergeCenteredCapInterval(
            query,
            center,
            axisDirection,
            axisHalfLength,
            expandedRadius,
            positiveCap: false,
            ref found,
            ref entry,
            ref exit);
        MergeCenteredCapInterval(
            query,
            center,
            axisDirection,
            axisHalfLength,
            expandedRadius,
            positiveCap: true,
            ref found,
            ref entry,
            ref exit);
        return found;
    }

    private static bool IsCenteredCapsulePointContained(
        Vector2d point,
        Vector2d center,
        Vector2d axisDirection,
        Fixed64 axisHalfLength,
        Signed192 distanceSquared,
        Signed192 axisProjection,
        Signed192 axisLengthSquared,
        Signed192 squaredRadius,
        Signed192 expandedRadius,
        Signed320 axialExtent,
        bool strict)
    {
        int cap = GetCenteredCap(axisProjection, axialExtent);
        if (cap == 0)
        {
            int radialSign = GetRadialConstant(
                distanceSquared,
                axisProjection,
                axisLengthSquared,
                squaredRadius).Sign;
            return strict ? radialSign < 0 : radialSign <= 0;
        }

        int capSign = GetCenteredCapConstant(
            point,
            center,
            axisDirection,
            axisHalfLength,
            expandedRadius,
            positiveCap: cap > 0).Sign;
        return strict ? capSign < 0 : capSign <= 0;
    }

    private static bool IsCenteredCapsulePointContained(
        Vector3d point,
        Vector3d center,
        Vector3d axisDirection,
        Fixed64 axisHalfLength,
        Signed192 distanceSquared,
        Signed192 axisProjection,
        Signed192 axisLengthSquared,
        Signed192 squaredRadius,
        Signed192 expandedRadius,
        Signed320 axialExtent,
        bool strict)
    {
        int cap = GetCenteredCap(axisProjection, axialExtent);
        if (cap == 0)
        {
            int radialSign = GetRadialConstant(
                distanceSquared,
                axisProjection,
                axisLengthSquared,
                squaredRadius).Sign;
            return strict ? radialSign < 0 : radialSign <= 0;
        }

        int capSign = GetCenteredCapConstant(
            point,
            center,
            axisDirection,
            axisHalfLength,
            expandedRadius,
            positiveCap: cap > 0).Sign;
        return strict ? capSign < 0 : capSign <= 0;
    }

    private static int GetCenteredCap(Signed192 axisProjection, Signed320 axialExtent)
    {
        Signed320 scaledProjection = WideArithmetic.MultiplySigned192(ParameterScale, axisProjection);
        if (WideArithmetic.AddSigned320(scaledProjection, axialExtent).Sign <= 0)
            return -1;
        return WideArithmetic.SubtractSigned320(scaledProjection, axialExtent).Sign >= 0
            ? 1
            : 0;
    }

    private static void MergeCenteredCapInterval(
        FixedSegment2d query,
        Vector2d center,
        Vector2d axisDirection,
        Fixed64 axisHalfLength,
        Signed192 expandedRadius,
        bool positiveCap,
        ref bool found,
        ref Fixed64 entry,
        ref Fixed64 exit)
    {
        GetCenteredCapPolynomial(
            query,
            center,
            axisDirection,
            axisHalfLength,
            expandedRadius,
            positiveCap,
            out Signed320 coefficient,
            out Signed320 projection,
            out Signed320 constant);
        Merge(
            TrySolveUnitQuadratic(coefficient, projection, constant, out Fixed64 capEntry, out Fixed64 capExit),
            capEntry,
            capExit,
            ref found,
            ref entry,
            ref exit);
    }

    private static void MergeCenteredCapInterval(
        FixedSegment query,
        Vector3d center,
        Vector3d axisDirection,
        Fixed64 axisHalfLength,
        Signed192 expandedRadius,
        bool positiveCap,
        ref bool found,
        ref Fixed64 entry,
        ref Fixed64 exit)
    {
        GetCenteredCapPolynomial(
            query,
            center,
            axisDirection,
            axisHalfLength,
            expandedRadius,
            positiveCap,
            out Signed320 coefficient,
            out Signed320 projection,
            out Signed320 constant);
        Merge(
            TrySolveUnitQuadratic(coefficient, projection, constant, out Fixed64 capEntry, out Fixed64 capExit),
            capEntry,
            capExit,
            ref found,
            ref entry,
            ref exit);
    }

    private static void GetCenteredCapPolynomial(
        FixedSegment2d query,
        Vector2d center,
        Vector2d axisDirection,
        Fixed64 axisHalfLength,
        Signed192 expandedRadius,
        bool positiveCap,
        out Signed320 coefficient,
        out Signed320 projection,
        out Signed320 constant)
    {
        Signed192 x = GetCenteredCapOffset(
            query.Start.X,
            center.X,
            axisDirection.X,
            axisHalfLength,
            positiveCap);
        Signed192 y = GetCenteredCapOffset(
            query.Start.Y,
            center.Y,
            axisDirection.Y,
            axisHalfLength,
            positiveCap);
        Signed192 velocityX = GetScaledDifference(query.End.X, query.Start.X);
        Signed192 velocityY = GetScaledDifference(query.End.Y, query.Start.Y);
        Signed192 scaledRadius = ScaleByParameter(expandedRadius);

        coefficient = AddProducts(velocityX, velocityX, velocityY, velocityY);
        projection = AddProducts(x, velocityX, y, velocityY);
        constant = WideArithmetic.SubtractSigned320(
            AddProducts(x, x, y, y),
            WideArithmetic.MultiplySigned192(scaledRadius, scaledRadius));
    }

    private static void GetCenteredCapPolynomial(
        FixedSegment query,
        Vector3d center,
        Vector3d axisDirection,
        Fixed64 axisHalfLength,
        Signed192 expandedRadius,
        bool positiveCap,
        out Signed320 coefficient,
        out Signed320 projection,
        out Signed320 constant)
    {
        Signed192 x = GetCenteredCapOffset(
            query.Start.X,
            center.X,
            axisDirection.X,
            axisHalfLength,
            positiveCap);
        Signed192 y = GetCenteredCapOffset(
            query.Start.Y,
            center.Y,
            axisDirection.Y,
            axisHalfLength,
            positiveCap);
        Signed192 z = GetCenteredCapOffset(
            query.Start.Z,
            center.Z,
            axisDirection.Z,
            axisHalfLength,
            positiveCap);
        Signed192 velocityX = GetScaledDifference(query.End.X, query.Start.X);
        Signed192 velocityY = GetScaledDifference(query.End.Y, query.Start.Y);
        Signed192 velocityZ = GetScaledDifference(query.End.Z, query.Start.Z);
        Signed192 scaledRadius = ScaleByParameter(expandedRadius);

        coefficient = AddProducts(velocityX, velocityX, velocityY, velocityY, velocityZ, velocityZ);
        projection = AddProducts(x, velocityX, y, velocityY, z, velocityZ);
        constant = WideArithmetic.SubtractSigned320(
            AddProducts(x, x, y, y, z, z),
            WideArithmetic.MultiplySigned192(scaledRadius, scaledRadius));
    }

    private static Signed320 GetCenteredCapConstant(
        Vector2d point,
        Vector2d center,
        Vector2d axisDirection,
        Fixed64 axisHalfLength,
        Signed192 expandedRadius,
        bool positiveCap)
    {
        Signed192 x = GetCenteredCapOffset(
            point.X,
            center.X,
            axisDirection.X,
            axisHalfLength,
            positiveCap);
        Signed192 y = GetCenteredCapOffset(
            point.Y,
            center.Y,
            axisDirection.Y,
            axisHalfLength,
            positiveCap);
        Signed192 scaledRadius = ScaleByParameter(expandedRadius);
        return WideArithmetic.SubtractSigned320(
            AddProducts(x, x, y, y),
            WideArithmetic.MultiplySigned192(scaledRadius, scaledRadius));
    }

    private static Signed320 GetCenteredCapConstant(
        Vector3d point,
        Vector3d center,
        Vector3d axisDirection,
        Fixed64 axisHalfLength,
        Signed192 expandedRadius,
        bool positiveCap)
    {
        Signed192 x = GetCenteredCapOffset(
            point.X,
            center.X,
            axisDirection.X,
            axisHalfLength,
            positiveCap);
        Signed192 y = GetCenteredCapOffset(
            point.Y,
            center.Y,
            axisDirection.Y,
            axisHalfLength,
            positiveCap);
        Signed192 z = GetCenteredCapOffset(
            point.Z,
            center.Z,
            axisDirection.Z,
            axisHalfLength,
            positiveCap);
        Signed192 scaledRadius = ScaleByParameter(expandedRadius);
        return WideArithmetic.SubtractSigned320(
            AddProducts(x, x, y, y, z, z),
            WideArithmetic.MultiplySigned192(scaledRadius, scaledRadius));
    }

    private static Signed192 GetCenteredCapOffset(
        Fixed64 point,
        Fixed64 center,
        Fixed64 axisDirection,
        Fixed64 axisHalfLength,
        bool positiveCap)
    {
        Signed192 difference = WideArithmetic.SubtractSigned192(
            WideArithmetic.FromSignedRaw(point.m_rawValue),
            WideArithmetic.FromSignedRaw(center.m_rawValue));
        Signed320 scaledDifference = WideArithmetic.MultiplySigned192(difference, ParameterScale);
        Signed320 directionalOffset = WideArithmetic.MultiplySigned192(
            WideArithmetic.FromSignedRaw(axisDirection.m_rawValue),
            WideArithmetic.FromSignedRaw(axisHalfLength.m_rawValue));
        return NarrowProven(positiveCap
            ? WideArithmetic.SubtractSigned320(scaledDifference, directionalOffset)
            : WideArithmetic.AddSigned320(scaledDifference, directionalOffset));
    }

    private static Signed320 GetRadialComponent(
        Signed192 axisLengthSquared,
        Signed192 difference,
        Signed192 axisComponent,
        Signed192 axisProjection) =>
        WideArithmetic.SubtractSigned320(
            WideArithmetic.MultiplySigned192(axisLengthSquared, difference),
            WideArithmetic.MultiplySigned192(axisComponent, axisProjection));

    private static void GetClosestCenteredAxisRatio(
        Vector2d point,
        Vector2d center,
        Vector2d axisDirection,
        Fixed64 axisHalfLength,
        out Signed192 numerator,
        out Signed192 denominator)
    {
        Signed192 axisLengthSquared = GetDot(axisDirection, Vector2d.Zero, axisDirection, Vector2d.Zero);
        Signed192 axisProjection = GetDot(point, center, axisDirection, Vector2d.Zero);
        int cap = GetCenteredCap(
            axisProjection,
            GetCenteredAxialExtent(axisLengthSquared, axisHalfLength, Fixed64.Zero));
        if (cap == 0)
        {
            numerator = axisProjection;
            denominator = axisLengthSquared;
            return;
        }

        numerator = WideArithmetic.FromSignedRaw(
            cap < 0 ? -axisHalfLength.m_rawValue : axisHalfLength.m_rawValue);
        denominator = ParameterScale;
    }

    private static void GetClosestCenteredAxisRatio(
        Vector3d point,
        Vector3d center,
        Vector3d axisDirection,
        Fixed64 axisHalfLength,
        out Signed192 numerator,
        out Signed192 denominator)
    {
        Signed192 axisLengthSquared = GetDot(axisDirection, Vector3d.Zero, axisDirection, Vector3d.Zero);
        Signed192 axisProjection = GetDot(point, center, axisDirection, Vector3d.Zero);
        int cap = GetCenteredCap(
            axisProjection,
            GetCenteredAxialExtent(axisLengthSquared, axisHalfLength, Fixed64.Zero));
        if (cap == 0)
        {
            numerator = axisProjection;
            denominator = axisLengthSquared;
            return;
        }

        numerator = WideArithmetic.FromSignedRaw(
            cap < 0 ? -axisHalfLength.m_rawValue : axisHalfLength.m_rawValue);
        denominator = ParameterScale;
    }

    private static bool TryGetCenteredCapsuleSurfaceCoordinate(
        Fixed64 center,
        Fixed64 axisDirection,
        Signed192 axisNumerator,
        Signed192 axisDenominator,
        Fixed64 normal,
        Fixed64 radius,
        out Fixed64 coordinate)
    {
        Signed320 centerTerm = MultiplyThreeToSigned320(
            WideArithmetic.FromSignedRaw(center.m_rawValue),
            axisDenominator,
            ParameterScale);
        Signed320 axisTerm = MultiplyThreeToSigned320(
            WideArithmetic.FromSignedRaw(axisDirection.m_rawValue),
            axisNumerator,
            ParameterScale);
        Signed320 radialTerm = MultiplyThreeToSigned320(
            WideArithmetic.FromSignedRaw(normal.m_rawValue),
            WideArithmetic.FromSignedRaw(radius.m_rawValue),
            axisDenominator);
        Signed320 denominator = MultiplyThreeToSigned320(
            axisDenominator,
            ParameterScale,
            ParameterScale);
        Signed320 numerator = WideArithmetic.AddSigned320(
            WideArithmetic.AddSigned320(centerTerm, axisTerm),
            radialTerm);
        return Fixed64.TryGetSignedRatio(numerator, denominator, out coordinate);
    }

    private static Signed320 MultiplyThreeToSigned320(
        Signed192 first,
        Signed192 second,
        Signed192 third) =>
        NarrowProven(WideArithmetic.MultiplySigned320(
            WideArithmetic.MultiplySigned192(first, second),
            WideArithmetic.ExtendToSigned320(third)));

    private static Signed320 GetCenteredAxisOffsetComponent(
        Fixed64 point,
        Fixed64 center,
        Fixed64 axisDirection,
        Signed192 axisNumerator,
        Signed192 axisDenominator) =>
        WideArithmetic.MultiplySubtract(
            GetComponentDifference(point, center),
            axisDenominator,
            WideArithmetic.FromSignedRaw(axisDirection.m_rawValue),
            axisNumerator);

    // Centered-capsule surface terms use at most 194 signed bits. The upper
    // four words are therefore sign extension before this mechanical narrow.
    private static Signed320 NarrowProven(Signed576 value) =>
        new(value.Word4, value.Word3, value.Word2, value.Word1, value.Word0);

    private static Signed192 GetComponentDifference(Fixed64 point, Fixed64 center) =>
        WideArithmetic.SubtractSigned192(
            WideArithmetic.FromSignedRaw(point.m_rawValue),
            WideArithmetic.FromSignedRaw(center.m_rawValue));

    private static Signed192 GetScaledDifference(Fixed64 end, Fixed64 start) =>
        ScaleByParameter(WideArithmetic.SubtractSigned192(
            WideArithmetic.FromSignedRaw(end.m_rawValue),
            WideArithmetic.FromSignedRaw(start.m_rawValue)));

    private static Signed192 ScaleByParameter(Signed192 value) =>
        NarrowProven(WideArithmetic.MultiplySigned192(value, ParameterScale));

    // A full-domain coordinate difference times Q32.32 scale, plus one
    // normalized-axis component times a nonnegative Fixed64 half-length, uses
    // at most 97 signed bits. The upper two words are therefore sign extension.
    private static Signed192 NarrowProven(Signed320 value) =>
        new(value.Word2, value.Word1, value.Word0);

    private static Signed320 AddProducts(
        Signed192 left1,
        Signed192 right1,
        Signed192 left2,
        Signed192 right2) =>
        WideArithmetic.AddSigned320(
            WideArithmetic.MultiplySigned192(left1, right1),
            WideArithmetic.MultiplySigned192(left2, right2));

    private static Signed320 AddProducts(
        Signed192 left1,
        Signed192 right1,
        Signed192 left2,
        Signed192 right2,
        Signed192 left3,
        Signed192 right3) =>
        WideArithmetic.AddSigned320(
            AddProducts(left1, right1, left2, right2),
            WideArithmetic.MultiplySigned192(left3, right3));

    private static bool TrySolveUnitQuadratic(
        Signed320 coefficient,
        Signed320 projection,
        Signed320 constant,
        out Fixed64 entry,
        out Fixed64 exit)
    {
        Signed320 one = WideArithmetic.ExtendToSigned320(One);
        return TrySolveBoundedQuadratic(
            coefficient,
            projection,
            constant,
            new RationalBound320(default, one),
            new RationalBound320(one, one),
            Fixed64.One,
            out entry,
            out exit);
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
